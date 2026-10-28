#include "ATTrafficSubsystem.h"
#include "ATRouter.h"
#include "Algo/Reverse.h"
#include "AutoTrains.h"
#include "Buildables/FGBuildableRailroadStation.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "FGLocomotive.h"
#include "FGRailroadSubsystem.h"
#include "FGRailroadTimeTable.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "FGTrain.h"
#include "FGTrainStationIdentifier.h"
#include "HAL/PlatformTime.h"
#include "RailroadNavigation.h"

namespace
{
	// A plan adds at most this many waits.
	constexpr int32 PlanMaxWaits = 32;
	// An open-ended time: a stay nobody knows the end of, or everything after a wait for one. [s]
	constexpr double OpenTime = 1e12;
	// A docked train past its expected departure is expected to leave this soon, renewed every update. [s]
	constexpr double OverdueDepartSeconds = 1.0;
	// A departure that couldn't be predicted is tried again after this long. [s]
	constexpr double PredictRetrySeconds = 10.0;
	// Route search: each train every SearchIntervalSeconds (spread out), at once when its planned arrival has got
	// SearchWorseSeconds later than at its last search, and at least every SearchFailedSeconds while its plan fails;
	// the gain a new route needs and how long it's kept are settings. [s]
	constexpr float SearchIntervalSeconds = 5.f;
	constexpr double SearchWorseSeconds = 15.0;
	constexpr float SearchFailedSeconds = 1.f;
	// A track the search can only get onto after an open-ended booking costs this. [s]
	constexpr float SearchBlockedSeconds = 1e6f;
	// Trains that waited longer than WaitDumpSeconds are described in the log every WaitDumpInterval. [s]
	constexpr double WaitDumpSeconds = 60.0;
	constexpr double WaitDumpInterval = 30.0;
	// A train slower than this whose follow limit leaves it less than this is named as held by its leader. [cm/s] [cm]
	constexpr float FollowStuckSpeed = 50.f;
	constexpr float FollowStuckDistance = 3000.f;
	// The earliest-entry search gives up after this many shifts.
	constexpr int32 EntryMaxShifts = 16;
	// Safe-interval search: stops closer together than this count as one; a wait shorter than SippMinWaitSeconds is
	// dropped; a train slower than SippStandingSpeed counts as standing. [cm] [s] [cm/s]
	constexpr float SippMinStopGap = 100.f;
	constexpr double SippMinWaitSeconds = 0.5;
	constexpr float SippStandingSpeed = 50.f;
	// A plan's wait this close to its end doesn't hold back the grant. [s]
	constexpr double PlanWaitGraceSeconds = 0.5;
	// Planner counters are logged this often. [s]
	constexpr double PlanSummarySeconds = 60.0;
	// An update slower than this is logged, at most once per SlowLogSeconds. [s]
	constexpr double SlowUpdateSeconds = 0.02;
	constexpr double SlowLogSeconds = 10.0;

	bool IsOpen(double Time)
	{
		return Time >= OpenTime * 0.5;
	}

	FString NameOf(const AFGTrain* Train)
	{
		return Train ? Train->GetTrainName().ToString() : TEXT("?");
	}

	FString StationName(const AFGBuildableRailroadStation* Station)
	{
		const AFGTrainStationIdentifier* Identifier = Station ? Station->GetStationIdentifier() : nullptr;
		return Identifier ? Identifier->GetStationName().ToString() : TEXT("?");
	}

	FString ResourceName(const AFGBuildableRailroadTrack* Track, int32 Joint)
	{
		return Joint != INDEX_NONE ? FString::Printf(TEXT("switch %d"), Joint)
			: GetNameSafe(Track).Replace(TEXT("Build_RailroadTrack_C_"), TEXT("track "));
	}

	FString SecondsText(double Seconds)
	{
		return IsOpen(Seconds) ? FString(TEXT("an unknown time")) : FString::Printf(TEXT("%.0f s"), FMath::Max(Seconds, 0.0));
	}

	// A free time window at a SIPP stop, and the train whose booking ends the block before it.
	struct FSippInterval
	{
		double Start = 0.0;
		double End = 0.0;
		const AFGTrain* After = nullptr;
	};

	// The best arrival found at a SIPP stop within one of its free windows: the window, the departure from the stop
	// before and that stop's label, and the train it waited for, if it waited.
	struct FSippLabel
	{
		double Arrive = 0.0;
		int32 Interval = 0;
		double PrevDepart = 0.0;
		int32 PrevLabel = INDEX_NONE;
		const AFGTrain* Cause = nullptr;
	};

	// Free windows from From on, outside every block in Blocks.
	template <typename TBlock>
	TArray<FSippInterval> GetFreeIntervals(TArray<TBlock> Blocks, double From)
	{
		Blocks.Sort([](const TBlock& A, const TBlock& B) { return A.Lo < B.Lo; });
		TArray<FSippInterval> Free;
		double Start = From;
		const AFGTrain* After = nullptr;
		for (const TBlock& Block : Blocks)
		{
			if (Block.Hi <= Start)
			{
				continue;
			}
			if (Block.Lo > Start)
			{
				Free.Add({Start, Block.Lo, After});
			}
			Start = Block.Hi;
			After = Block.Owner;
			if (IsOpen(Start))
			{
				return Free;
			}
		}
		Free.Add({Start, OpenTime, After});
		return Free;
	}

	// Earliest departure in [From, To] for a move lasting Duration that overlaps none of Blocks, or -1; Cause becomes the
	// train whose block delayed it last.
	template <typename TBlock>
	double GetEarliestDeparture(const TArray<TBlock>& Blocks, double From, double To, double Duration, const AFGTrain*& Cause)
	{
		double Depart = From;
		for (int32 Round = 0; Round < 64 && Depart <= To; ++Round)
		{
			bool bMoved = false;
			for (const TBlock& Block : Blocks)
			{
				if (Depart < Block.Hi && Depart + Duration > Block.Lo)
				{
					Depart = Block.Hi;
					Cause = Block.Owner;
					bMoved = true;
				}
			}
			if (!bMoved)
			{
				return Depart;
			}
		}
		return -1.0;
	}

	// Time from standstill over Distance, accelerating at Accel up to Cruise. [s]
	double RestTime(float Distance, float Cruise, float Accel)
	{
		if (Distance <= 0.f)
		{
			return 0.0;
		}
		const float ToCruise = FMath::Square(Cruise) / (2.f * Accel);
		return Distance <= ToCruise ? FMath::Sqrt(2.f * Distance / Accel) : Cruise / Accel + (Distance - ToCruise) / Cruise;
	}

	// The front's motion along a route: legs between stops, each speeding up towards cruise and braking into its end.
	class FProfile
	{
	public:
		FProfile(float InSpeed, double InNow, float InCruise, float InAccel, float InBrake)
			: Cruise(FMath::Max(InCruise, InSpeed)), Accel(InAccel), Brake(InBrake), NextStart(0.f), NextSpeed(InSpeed), NextTime(InNow)
		{
		}

		// Adds a stop at At until Until, or as soon as the train gets there if later; stops come in route order.
		void AddStop(float At, double Until)
		{
			AddLeg(FMath::Max(At, NextStart), 0.f);
			NextTime = FMath::Max(NextTime, Until);
			NextSpeed = 0.f;
		}

		// Ends the motion at End: stopped there for Dwell, then pulling away (the goal), or carrying on at cruise.
		void Finish(float End, bool bStop, double Dwell)
		{
			AddLeg(FMath::Max(End, NextStart), bStop ? 0.f : Cruise);
			bStopsAtEnd = bStop;
			EndDwell = Dwell;
		}

		// Time the front passes At; at a stop, the time it moves off again.
		double TimeAt(float At) const
		{
			if (Legs.Num() == 0 || At <= Legs[0].Start)
			{
				return Legs.Num() > 0 ? Legs[0].Time : 0.0;
			}
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				const FLeg& Leg = Legs[Index];
				if (At < Leg.End || (At <= Leg.End && Index == Legs.Num() - 1))
				{
					return Leg.Time + Leg.TimeTo(At - Leg.Start);
				}
				if (At <= Leg.End + 1.f && Index + 1 < Legs.Num())
				{
					return Legs[Index + 1].Time;
				}
			}
			const FLeg& Last = Legs.Last();
			const double AtEnd = Last.Time + Last.TimeTo(Last.End - Last.Start);
			if (bStopsAtEnd)
			{
				// Past the goal: once the stay is over, pulling away from standstill.
				return IsOpen(EndDwell) ? OpenTime : AtEnd + EndDwell + RestTime(At - Last.End, Cruise, Accel);
			}
			return AtEnd + (At - Last.End) / Cruise;
		}

	private:
		struct FLeg
		{
			float Start = 0.f;
			float End = 0.f;
			double Time = 0.0;
			float From = 0.f;
			float To = 0.f;
			float Peak = 0.f;
			float SpeedUp = 0.f;
			float SlowDown = 0.f;
			float Accel = 1.f;
			float Decel = 1.f;

			// Seconds from the leg's start to Distance into it.
			double TimeTo(float Distance) const
			{
				const float Length = End - Start;
				Distance = FMath::Clamp(Distance, 0.f, Length);
				double Seconds = 0.0;
				const float InSpeedUp = FMath::Min(Distance, SpeedUp);
				Seconds += InSpeedUp > 0.f ? (FMath::Sqrt(FMath::Square(From) + 2.f * Accel * InSpeedUp) - From) / Accel : 0.0;
				if (Distance <= SpeedUp)
				{
					return Seconds;
				}
				const float Cruising = Length - SpeedUp - SlowDown;
				const float InCruise = FMath::Min(Distance - SpeedUp, Cruising);
				Seconds += Peak > 0.f ? InCruise / Peak : 0.0;
				if (Distance <= SpeedUp + Cruising)
				{
					return Seconds;
				}
				const float InSlowDown = Distance - SpeedUp - Cruising;
				const float Speed = FMath::Sqrt(FMath::Max(FMath::Square(Peak) - 2.f * Decel * InSlowDown, 0.f));
				return Seconds + (Peak - Speed) / Decel;
			}
		};

		void AddLeg(float End, float To)
		{
			FLeg Leg;
			Leg.Start = NextStart;
			Leg.End = End;
			Leg.Time = NextTime;
			Leg.From = NextSpeed;
			Leg.To = FMath::Min(To, Cruise);
			Leg.Accel = Accel;
			Leg.Decel = Brake;
			const float Length = End - NextStart;
			if (Length > 0.f)
			{
				// The highest speed reachable between speeding up from From and braking to To.
				const float PeakSquared = (Length + FMath::Square(Leg.From) / (2.f * Accel) + FMath::Square(Leg.To) / (2.f * Brake)) / (1.f / (2.f * Accel) + 1.f / (2.f * Brake));
				Leg.Peak = FMath::Min(Cruise, FMath::Sqrt(FMath::Max(PeakSquared, 0.f)));
				if (Leg.Peak < Leg.From)
				{
					// Already too fast to speed up: brakes all the way, harder than planned if it must.
					Leg.Peak = Leg.From;
					Leg.SpeedUp = 0.f;
					Leg.SlowDown = Length;
					Leg.Decel = FMath::Max((FMath::Square(Leg.From) - FMath::Square(Leg.To)) / (2.f * Length), 1.f);
				}
				else
				{
					Leg.SpeedUp = (FMath::Square(Leg.Peak) - FMath::Square(Leg.From)) / (2.f * Accel);
					Leg.SlowDown = FMath::Max((FMath::Square(Leg.Peak) - FMath::Square(Leg.To)) / (2.f * Brake), 0.f);
					const float Over = Leg.SpeedUp + Leg.SlowDown - Length;
					if (Over > 0.f)
					{
						Leg.SpeedUp = FMath::Max(Leg.SpeedUp - Over * 0.5f, 0.f);
						Leg.SlowDown = Length - Leg.SpeedUp;
					}
				}
			}
			Legs.Add(Leg);
			NextStart = End;
			NextTime = Leg.Time + Leg.TimeTo(Length);
			NextSpeed = Leg.To;
		}

		float Cruise;
		float Accel;
		float Brake;
		float NextStart;
		float NextSpeed;
		double NextTime;
		bool bStopsAtEnd = false;
		double EndDwell = OpenTime;
		TArray<FLeg> Legs;
	};
}

void AATTrafficSubsystem::CollectPasses(const FATTrafficTrain& State, TArray<FPass>& Out) const
{
	Out.Reset();
	TSet<const AFGBuildableRailroadTrack*> OnRoute;
	TSet<int32> JointsOnRoute;
	for (int32 Index = 0; Index < State.Route.Num(); ++Index)
	{
		const FATRouteStep& Step = State.Route[Index];
		const float End = Step.EndDistance - State.FrontAhead;
		const float Begin = End - Step.Track->GetLength();
		if (Index > 0)
		{
			const UFGRailroadTrackConnectionComponent* Entry = State.Route[Index - 1].Exit;
			const int32 Joint = GetJoint(Entry);
			if (Joint != INDEX_NONE)
			{
				FPass Pass;
				Pass.Key.Joint = Joint;
				Pass.Exit = Step.Exit;
				Pass.Via = Entry;
				Pass.RouteIndex = Index;
				Pass.EnterAt = Begin - GetFoulDistance(Entry);
				Pass.ClearAt = Begin + State.Length + GetJointFoulDistance(Joint);
				if (Pass.EnterAt <= 0.f && !State.OccupiedJoints.Contains(Joint))
				{
					// The front is in the zone already, though no vehicle counts as in it yet: entering now.
					Pass.EnterAt = 0.f;
				}
				Out.Add(Pass);
				JointsOnRoute.Add(Joint);
			}
		}
		FPass Pass;
		Pass.Key.Track = Step.Track;
		Pass.Exit = Step.Exit;
		Pass.RouteIndex = Index;
		Pass.EnterAt = Index == 0 ? -1.f : Begin;
		Pass.ClearAt = End + State.Length;
		Out.Add(Pass);
		OnRoute.Add(Step.Track);
	}
	// The way the tail runs over the tracks and switches behind the route, walking back from the route's first track:
	// the connection it leaves each track through, and for each switch the connections it enters and leaves it by.
	TMap<const AFGBuildableRailroadTrack*, const UFGRailroadTrackConnectionComponent*> TailExit;
	TMap<int32, TPair<const UFGRailroadTrackConnectionComponent*, const UFGRailroadTrackConnectionComponent*>> TailJoint;
	if (State.Route.Num() > 0 && State.Route[0].Exit)
	{
		const UFGRailroadTrackConnectionComponent* Ahead = State.Route[0].Exit;
		const UFGRailroadTrackConnectionComponent* Entry = Ahead->GetOpposite();
		while (Entry && TailExit.Num() < State.Occupied.Num())
		{
			const UFGRailroadTrackConnectionComponent* Behind = nullptr;
			for (const UFGRailroadTrackConnectionComponent* Other : Entry->GetConnections())
			{
				const AFGBuildableRailroadTrack* Track = Other ? Other->GetTrack() : nullptr;
				if (!Behind && Track && State.Occupied.Contains(const_cast<AFGBuildableRailroadTrack*>(Track)) && !OnRoute.Contains(Track) && !TailExit.Contains(Track))
				{
					Behind = Other;
				}
			}
			if (!Behind)
			{
				break;
			}
			TailExit.Add(Behind->GetTrack(), Behind);
			const int32 Joint = GetJoint(Behind);
			if (Joint != INDEX_NONE)
			{
				TailJoint.Add(Joint, TPair<const UFGRailroadTrackConnectionComponent*, const UFGRailroadTrackConnectionComponent*>(Behind, Ahead));
			}
			Ahead = Behind;
			Entry = Behind->GetOpposite();
		}
	}
	// What the tail still holds behind the front.
	for (const AFGBuildableRailroadTrack* Track : State.Occupied)
	{
		if (OnRoute.Contains(Track))
		{
			continue;
		}
		FPass Pass;
		Pass.Key.Track = Track;
		Pass.Exit = TailExit.FindRef(Track);
		Pass.EnterAt = -1.f;
		const float Clear = GetTailClearDistance(State, Track, INDEX_NONE);
		Pass.ClearAt = Clear >= 0.f ? Clear : State.Length;
		Out.Add(Pass);
	}
	for (const int32 Joint : State.OccupiedJoints)
	{
		if (JointsOnRoute.Contains(Joint))
		{
			continue;
		}
		FPass Pass;
		Pass.Key.Joint = Joint;
		if (const TPair<const UFGRailroadTrackConnectionComponent*, const UFGRailroadTrackConnectionComponent*>* Way = TailJoint.Find(Joint))
		{
			Pass.Via = Way->Key;
			Pass.Exit = Way->Value;
		}
		Pass.EnterAt = -1.f;
		const float Clear = GetTailClearDistance(State, nullptr, Joint);
		Pass.ClearAt = Clear >= 0.f ? Clear : State.Length + GetJointFoulDistance(Joint);
		Out.Add(Pass);
	}
	Out.StableSort([](const FPass& A, const FPass& B) { return A.EnterAt < B.EnterAt; });
}

void AATTrafficSubsystem::TimePasses(const FATTrafficTrain& State, const FRouteSpeed& Speed, const TArray<FWait>& Waits, double Now, double GoalDwell,
	TArray<FPass>& Passes) const
{
	FProfile Profile(State.Speed, Now, Speed.Cruise, Speed.Accel, Speed.Brake);
	for (const FWait& Wait : Waits)
	{
		Profile.AddStop(Wait.At, Wait.Until);
	}
	const float End = State.Route.Num() > 0 ? State.Route.Last().EndDistance - State.FrontAhead : 0.f;
	Profile.Finish(End, true, GoalDwell);
	for (FPass& Pass : Passes)
	{
		Pass.Enter = Pass.EnterAt <= 0.f ? Now : Profile.TimeAt(Pass.EnterAt);
		Pass.Clear = Profile.TimeAt(Pass.ClearAt);
	}
}

const AATTrafficSubsystem::FBooking* AATTrafficSubsystem::FindConflict(const FATTrafficTrain& Self, const FPass& Pass, double Headway, EConflictScope Scope,
	double& OutNeedEnter) const
{
	const AFGTrain* SelfTrain = Self.Train.Get();
	const FBooking* Found = nullptr;
	OutNeedEnter = Pass.Enter;
	auto Check = [&](const FResourceKey& Key)
	{
		const TArray<FBooking>* Bookings = mBookings.Find(Key);
		if (!Bookings)
		{
			return;
		}
		const bool bSameResource = Key == Pass.Key;
		for (const FBooking& Booking : *Bookings)
		{
			if (Booking.Owner == SelfTrain)
			{
				continue;
			}
			// Nobody goes ahead of a locked booking. Along one line the order is fixed: a train ahead on this route comes
			// first wherever its bookings meet this route further on, and one behind has to pass this train before it gets
			// to its own.
			bool bFirst = Booking.bLocked;
			if (Pass.RouteIndex != INDEX_NONE)
			{
				if (FATTrafficTrain* const* Other = mStateOf.Find(Booking.Owner))
				{
					const int32 Ahead = FrontIndexOn(Self, **Other);
					if (Ahead != INDEX_NONE && Ahead <= Pass.RouteIndex)
					{
						bFirst = true;
					}
					else if (IsBehind(**Other, Self, Key))
					{
						continue;
					}
				}
			}
			if (!IsInScope(Self, Booking, bFirst, Scope))
			{
				continue;
			}
			double Need = Pass.Enter;
			if (bSameResource && Pass.Exit && Booking.Exit == Pass.Exit && Booking.Via == Pass.Via)
			{
				// The same way: either one may go first if it stays a headway ahead at both ends.
				const bool bBehind = Pass.Enter >= Booking.Enter + Headway && Pass.Clear >= Booking.Clear + Headway;
				const bool bAhead = !bFirst && Booking.Enter >= Pass.Enter + Headway && Booking.Clear >= Pass.Clear + Headway;
				if (bBehind || bAhead)
				{
					continue;
				}
				Need = IsOpen(Booking.Clear) ? OpenTime : Pass.Enter + FMath::Max(Booking.Enter + Headway - Pass.Enter, Booking.Clear + Headway - Pass.Clear);
			}
			else if (Pass.Enter < Booking.Clear + Headway && (bFirst || Pass.Clear + Headway > Booking.Enter))
			{
				Need = IsOpen(Booking.Clear) ? OpenTime : Booking.Clear + Headway;
			}
			else
			{
				continue;
			}
			if (!Found || Need > OutNeedEnter)
			{
				Found = &Booking;
				OutNeedEnter = Need;
			}
		}
	};
	if (Pass.Key.Joint != INDEX_NONE)
	{
		Check(Pass.Key);
	}
	else
	{
		for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Pass.Key.Track))
		{
			FResourceKey Key;
			Key.Track = Conflict;
			Check(Key);
		}
	}
	return Found;
}

void AATTrafficSubsystem::PlanTrain(const FATTrafficTrain& State, double Now, const TArray<FWait>& FixedWaits, double GoalDwell, EConflictScope Scope,
	FPlan& Plan) const
{
	const FRouteSpeed Speed = GetRouteSpeed(State);
	const double Headway = GetSafetyMarginSeconds();
	const AFGTrain* Self = State.Train.Get();
	Plan = FPlan();
	Plan.Waits = FixedWaits;
	Plan.Committed = GetStoppingDistance(State, State.Speed);
	TArray<FPass> Base;
	CollectPasses(State, Base);
	// Held: what the train holds from its last grant (route steps it holds from its front on, and the switches between
	// them). Locked: what it stands on, can't stop short of, or holds within its full-brake stopping distance plus the
	// stop distance; only what it holds or can't avoid, never a resource ahead it merely stands close to.
	const float LockedWithin = Plan.Committed + GetStopDistance();
	int32 HeldIndex = INDEX_NONE;
	while (HeldIndex + 1 < State.Route.Num() && HoldsTrack(State, State.Route[HeldIndex + 1].Track))
	{
		++HeldIndex;
	}
	for (FPass& Pass : Base)
	{
		const bool bHolds = Pass.RouteIndex != INDEX_NONE && Pass.RouteIndex <= HeldIndex;
		Pass.bLocked = Pass.RouteIndex == INDEX_NONE || Pass.EnterAt <= 0.f || Pass.EnterAt <= Plan.Committed || (bHolds && Pass.EnterAt <= LockedWithin);
		Pass.bHeld = !Pass.bLocked && bHolds;
	}
	// Where the train can stand: the end of each route track, short by its stop margin.
	TArray<float> StopPoints;
	for (const FATRouteStep& Step : State.Route)
	{
		StopPoints.Add(Step.EndDistance - State.FrontAhead - GetStopMargin(Step.Exit));
	}
	// The timetable from the safe-interval search; the loop below only corrects what its simplified timing missed.
	TArray<const AFGTrain*> WaitCauses;
	const double MinDepart = FixedWaits.Num() > 0 ? FixedWaits.Last().Until : Now;
	if (!PlanSafeIntervals(State, Speed, Now, Base, MinDepart, GoalDwell, Scope, Plan.Committed, Plan.Waits, WaitCauses))
	{
		Plan.Failure = TEXT("no free slot further along its route");
	}
	auto FirstCause = [&WaitCauses]() -> const AFGTrain*
	{
		for (const AFGTrain* Cause : WaitCauses)
		{
			if (Cause)
			{
				return Cause;
			}
		}
		return nullptr;
	};
	// Waits go in route order: none before the last fixed one.
	const float MinWaitAt = FixedWaits.Num() > 0 ? FixedWaits.Last().At : 0.f;
	for (int32 Attempt = 0; Attempt <= PlanMaxWaits; ++Attempt)
	{
		TArray<FPass> Passes = Base;
		TimePasses(State, Speed, Plan.Waits, Now, GoalDwell, Passes);
		const FPass* Conflicting = nullptr;
		const FBooking* Booking = nullptr;
		double NeedEnter = 0.0;
		for (const FPass& Pass : Passes)
		{
			// Nothing is planned beyond a wait for good.
			if (IsOpen(Pass.Enter))
			{
				continue;
			}
			if (Pass.bLocked)
			{
				// The train's own whatever others booked later (they plan around it): note another train that has it
				// locked as well.
				double Unused = 0.0;
				const FBooking* Other = Pass.EnterAt > 0.f && !Plan.CommittedWith ? FindConflict(State, Pass, 0.0, EConflictScope::LockedOnly, Unused) : nullptr;
				if (Other)
				{
					Plan.CommittedWith = Other->Owner;
					Plan.CommittedAt = ResourceName(Pass.Key.Track, Pass.Key.Joint);
				}
				continue;
			}
			// Held: kept unless a train that can't stop any more needs it.
			Booking = FindConflict(State, Pass, Headway, Pass.bHeld ? EConflictScope::LockedOnly : Scope, NeedEnter);
			if (Booking)
			{
				Conflicting = &Pass;
				break;
			}
		}
		if (!Conflicting)
		{
			MarkCommitted(State, Speed, Plan.Waits, Now, GoalDwell, Headway, StopPoints, Passes);
			Plan.Passes = MoveTemp(Passes);
			Plan.WaitFor = FirstCause();
			return;
		}
		// Wait at the last place to stand before the resource so the train enters it once the booking has cleared.
		int32 Best = INDEX_NONE;
		for (int32 Index = 0; Index < StopPoints.Num(); ++Index)
		{
			if (StopPoints[Index] >= Plan.Committed && StopPoints[Index] >= MinWaitAt && StopPoints[Index] < Conflicting->EnterAt)
			{
				Best = Index;
			}
		}
		if (Best == INDEX_NONE || Attempt == PlanMaxWaits)
		{
			// No place to wait for it: the train stands for good at the last place before it (its braking brings it to a
			// stop there) and books nothing beyond.
			Plan.Failure = Best == INDEX_NONE
				? FString::Printf(TEXT("no place to wait before %s for %s"), *ResourceName(Conflicting->Key.Track, Conflicting->Key.Joint), *NameOf(Booking->Owner))
				: FString::Printf(TEXT("no plan after %d waits"), PlanMaxWaits);
			const float ConflictAt = Conflicting->EnterAt;
			const AFGTrain* Cause = Booking->Owner;
			float StandAt = FMath::Max(MinWaitAt, 0.f);
			for (const float Point : StopPoints)
			{
				if (Point >= StandAt && Point < ConflictAt)
				{
					StandAt = Point;
				}
			}
			while (Plan.Waits.Num() > 0 && Plan.Waits.Last().At >= StandAt)
			{
				Plan.Waits.Pop();
				WaitCauses.Pop();
			}
			Plan.Waits.Add({StandAt, OpenTime});
			WaitCauses.Add(Cause);
			Passes = Base;
			TimePasses(State, Speed, Plan.Waits, Now, GoalDwell, Passes);
			MarkCommitted(State, Speed, Plan.Waits, Now, GoalDwell, Headway, StopPoints, Passes);
			Plan.Passes = MoveTemp(Passes);
			Plan.WaitFor = FirstCause();
			return;
		}
		const float At = StopPoints[Best];
		double Until = IsOpen(NeedEnter) ? OpenTime : NeedEnter - RestTime(Conflicting->EnterAt - At, Speed.Cruise, Speed.Accel);
		const int32 Existing = Plan.Waits.IndexOfByPredicate([At](const FWait& Wait) { return FMath::IsNearlyEqual(Wait.At, At, 1.f); });
		if (Existing != INDEX_NONE)
		{
			// Still in conflict after waiting there: wait longer.
			Until = IsOpen(Until) ? OpenTime : FMath::Max(Until, Plan.Waits[Existing].Until + 1.0);
			Plan.Waits[Existing].Until = Until;
			WaitCauses[Existing] = Booking->Owner;
		}
		else
		{
			int32 Insert = 0;
			while (Insert < Plan.Waits.Num() && Plan.Waits[Insert].At < At)
			{
				++Insert;
			}
			Plan.Waits.Insert({At, Until}, Insert);
			WaitCauses.Insert(Booking->Owner, Insert);
		}
	}
}

void AATTrafficSubsystem::MarkCommitted(const FATTrafficTrain& State, const FRouteSpeed& Speed, const TArray<FWait>& Waits, double Now, double GoalDwell,
	double Headway, const TArray<float>& StopPoints, TArray<FPass>& Passes) const
{
	// The front gets at least into what the train holds or can't stop short of ahead.
	float From = 0.f;
	for (const FPass& Pass : Passes)
	{
		if (Pass.RouteIndex != INDEX_NONE && (Pass.bLocked || Pass.bHeld))
		{
			From = FMath::Max(From, Pass.EnterAt);
		}
	}
	FProfile Profile(State.Speed, Now, Speed.Cruise, Speed.Accel, Speed.Brake);
	for (const FWait& Wait : Waits)
	{
		Profile.AddStop(Wait.At, Wait.Until);
	}
	const float End = State.Route.Num() > 0 ? State.Route.Last().EndDistance - State.FrontAhead : 0.f;
	Profile.Finish(End, true, GoalDwell);
	const AFGTrain* Self = State.Train.Get();
	// Standing with the front at At from Arrive on holds up a train planned before this one.
	auto HoldsUp = [&](float At, double Arrive)
	{
		for (const FPass& Pass : Passes)
		{
			if (Pass.EnterAt > At || Pass.ClearAt <= At)
			{
				continue;
			}
			TArray<FResourceKey, TInlineAllocator<4>> Keys;
			if (Pass.Key.Joint != INDEX_NONE)
			{
				Keys.Add(Pass.Key);
			}
			else
			{
				for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Pass.Key.Track))
				{
					FResourceKey Crossing;
					Crossing.Track = Conflict;
					Keys.Add(Crossing);
				}
			}
			for (const FResourceKey& Key : Keys)
			{
				const TArray<FBooking>* Bookings = mBookings.Find(Key);
				if (!Bookings)
				{
					continue;
				}
				for (const FBooking& Booking : *Bookings)
				{
					// Trains not driven plan nothing, ones planned after this one keep clear of it, and ones the same way
					// queue behind it.
					if (Booking.Owner == Self || Booking.Kind == EBookingKind::Standing || Booking.Clear + Headway <= Arrive
						|| (Booking.Kind == EBookingKind::Moving && PlansAfter(Booking.Owner, State))
						|| (Key == Pass.Key && Pass.Exit && Booking.Exit == Pass.Exit && Booking.Via == Pass.Via))
					{
						continue;
					}
					FATTrafficTrain* const* Other = mStateOf.Find(Booking.Owner);
					if (Other && IsBehind(**Other, State, Key))
					{
						continue;
					}
					return true;
				}
			}
		}
		return false;
	};
	float Reach = From;
	TArray<float> Places;
	if (From <= 0.f)
	{
		Places.Add(0.f);
	}
	for (const float Point : StopPoints)
	{
		if (Point >= From)
		{
			Places.Add(Point);
		}
	}
	for (const float At : Places)
	{
		Reach = At;
		// Arriving, not moving off again after a wait there.
		const double Arrive = At <= 0.f ? Now : Profile.TimeAt(At - 1.f);
		if (IsOpen(Arrive) || !HoldsUp(At, Arrive))
		{
			break;
		}
	}
	for (FPass& Pass : Passes)
	{
		Pass.bCommitted = !IsOpen(Pass.Enter) && Pass.EnterAt <= Reach;
	}
}

void AATTrafficSubsystem::Book(const FATTrafficTrain& State, const TArray<FPass>& Passes, EBookingKind Kind)
{
	const AFGTrain* Booker = State.Train.Get();
	if (!Booker)
	{
		return;
	}
	FOwnerBookings& Own = mOwnerBookings.FindOrAdd(FObjectKey(Booker));
	Own.Owner = Booker;
	for (const FPass& Pass : Passes)
	{
		if (IsOpen(Pass.Enter))
		{
			continue;
		}
		FBooking Booking;
		Booking.Owner = Booker;
		Booking.Enter = Pass.Enter;
		Booking.Clear = Pass.Clear;
		Booking.Exit = Pass.Exit;
		Booking.Via = Pass.Via;
		Booking.Kind = Kind;
		Booking.bLocked = Pass.bLocked;
		Booking.bHeld = Pass.bHeld;
		Booking.bCommitted = Pass.bCommitted;
		mBookings.FindOrAdd(Pass.Key).Add(Booking);
		Own.Keys.Add(Pass.Key);
	}
}

void AATTrafficSubsystem::BookStanding(const FATTrafficTrain& State, double Now, double Until, EBookingKind Kind)
{
	TArray<FPass> Standing;
	for (const AFGBuildableRailroadTrack* Track : State.Occupied)
	{
		FPass Pass;
		Pass.Key.Track = Track;
		Pass.Enter = Now;
		Pass.Clear = Until;
		Pass.bLocked = true;
		Standing.Add(Pass);
	}
	for (const int32 Joint : State.OccupiedJoints)
	{
		FPass Pass;
		Pass.Key.Joint = Joint;
		Pass.Enter = Now;
		Pass.Clear = Until;
		Pass.bLocked = true;
		Standing.Add(Pass);
	}
	Book(State, Standing, Kind);
}

void AATTrafficSubsystem::RemoveBookings(const AFGTrain* Booker)
{
	FOwnerBookings Own;
	if (!mOwnerBookings.RemoveAndCopyValue(FObjectKey(Booker), Own))
	{
		return;
	}
	for (const FResourceKey& Key : Own.Keys)
	{
		if (TArray<FBooking>* Bookings = mBookings.Find(Key))
		{
			Bookings->RemoveAll([Booker](const FBooking& Booking) { return Booking.Owner == Booker; });
			if (Bookings->Num() == 0)
			{
				mBookings.Remove(Key);
			}
		}
	}
}

float AATTrafficSubsystem::GetExpectedDwell(const AFGTrain* Train, const AFGBuildableRailroadStation* Station) const
{
	if (!Station)
	{
		return mSettings.DefaultStaySeconds;
	}
	if (const float* Own = mDwell.Find(TPair<FObjectKey, FObjectKey>(FObjectKey(Train), FObjectKey(Station))))
	{
		return *Own;
	}
	if (const float* Any = mStationDwell.Find(FObjectKey(Station)))
	{
		return *Any;
	}
	return mSettings.DefaultStaySeconds;
}

void AATTrafficSubsystem::TrackDocking(FATTrafficTrain& State, AFGTrain* Train, double Now)
{
	const bool bDocked = Train->IsDocked();
	if (bDocked && !State.bWasDocked)
	{
		// A dock already going on when this mod first saw the train started at an unknown time.
		State.DockedAt = State.bSeenUndocked ? Now : -1.0;
		State.DockStation = Train->mDockedAtStation.Get();
		State.PredictedRoute.Reset();
		State.PredictedStation = nullptr;
		State.PredictedAt = -1e9;
	}
	else if (!bDocked && State.bWasDocked)
	{
		const AFGBuildableRailroadStation* Station = State.DockStation.Get();
		if (State.DockedAt >= 0.0 && Station)
		{
			const float Dwell = Now - State.DockedAt;
			const float Before = GetExpectedDwell(Train, Station);
			mDwell.Add(TPair<FObjectKey, FObjectKey>(FObjectKey(Train), FObjectKey(Station)), Dwell);
			mStationDwell.Add(FObjectKey(Station), Dwell);
			UE_LOG(LogAutoTrains, Log, TEXT("planner: %s left %s after %.1f s (expected %.1f s)"), *NameOf(Train), *StationName(Station), Dwell, Before);
		}
		State.DockedAt = -1.0;
		State.PredictedRoute.Reset();
		State.PredictedStation = nullptr;
		State.LegStart = -1.0;
	}
	if (!bDocked)
	{
		State.bSeenUndocked = true;
	}
	State.bWasDocked = bDocked;
}

float AATTrafficSubsystem::ComputeFrontAhead(const FATTrafficTrain& State, const TArray<FATRouteStep>& Route) const
{
	float Front = 0.f;
	for (const FATVehicleSpot& Vehicle : State.Vehicles)
	{
		// Only tracks within the train's length count: a route can come back over a track the train stands on.
		for (const FATRouteStep& Step : Route)
		{
			if (Step.EndDistance - Step.Track->GetLength() > State.Length)
			{
				break;
			}
			if (Step.Track == Vehicle.Track && Step.Exit)
			{
				const float ToExit = FMath::Abs(Step.Exit->GetTrackPosition().Offset - Vehicle.Offset);
				Front = FMath::Max(Front, Step.EndDistance - ToExit + Vehicle.HalfLength);
				break;
			}
		}
	}
	return Front;
}

void AATTrafficSubsystem::PredictDeparture(FATTrafficTrain& State, double Now)
{
	State.PredictedAt = Now;
	State.PredictedRoute.Reset();
	State.PredictedStation = nullptr;
	AFGTrain* Train = State.Train.Get();
	AFGRailroadTimeTable* TimeTable = Train ? Train->GetTimeTable() : nullptr;
	if (!TimeTable)
	{
		return;
	}
	TArray<FTimeTableStop> Stops;
	TimeTable->GetStops(Stops);
	if (Stops.Num() == 0)
	{
		return;
	}
	// The next stop at another station than the one docked at, from the timetable's current stop on.
	const AFGBuildableRailroadStation* Docked = State.DockStation.Get();
	const int32 Current = FMath::Clamp(TimeTable->GetCurrentStop(), 0, Stops.Num() - 1);
	AFGBuildableRailroadStation* Next = nullptr;
	for (int32 Offset = 0; Offset < Stops.Num() && !Next; ++Offset)
	{
		const FTimeTableStop& Stop = Stops[(Current + Offset) % Stops.Num()];
		AFGBuildableRailroadStation* Station = Stop.Station ? Stop.Station->GetStation() : nullptr;
		if (Station && Station != Docked)
		{
			Next = Station;
		}
	}
	if (!Next)
	{
		return;
	}
	// The game's pathfinder from each locomotive facing its own way; the train leaves with the one of the shortest path.
	TArray<FATRouteStep> Best;
	float BestLength = MAX_flt;
	const AFGLocomotive* BestLocomotive = nullptr;
	for (AFGRailroadSubsystem::TTrainIterator It(Train->GetFirstVehicle()); It; ++It)
	{
		AFGLocomotive* Locomotive = Cast<AFGLocomotive>(*It);
		if (!Locomotive)
		{
			continue;
		}
		const FRailroadPathFindingResult Result = FRailroadNavigation::FindPathSync(Locomotive, Next);
		if (Result.Result != ERailroadPathFindingResult::RPFR_Success || !Result.Path.IsValid() || Result.Path->PathPoints.Num() == 0)
		{
			continue;
		}
		const FRailroadTrackPosition& Position = Locomotive->GetTrackPosition();
		TArray<FATRouteStep> Route;
		float Distance = 0.f;
		for (int32 Index = 0; Index < Result.Path->PathPoints.Num(); ++Index)
		{
			UFGRailroadTrackConnectionComponent* Exit = Result.Path->PathPoints[Index].TrackConnection.Get();
			AFGBuildableRailroadTrack* Track = Exit ? Exit->GetTrack() : nullptr;
			if (!Track)
			{
				break;
			}
			// From the locomotive to its first connection, then whole tracks.
			Distance += Index == 0 ? (Track == Position.Track.Get() ? FMath::Abs(Exit->GetTrackPosition().Offset - Position.Offset) : 0.f) : Track->GetLength();
			Route.Add({Track, Exit, Distance});
		}
		if (Route.Num() > 0 && Distance < BestLength)
		{
			BestLength = Distance;
			Best = MoveTemp(Route);
			BestLocomotive = Locomotive;
		}
	}
	if (Best.Num() == 0)
	{
		UE_LOG(LogAutoTrains, Log, TEXT("planner: %s docked at %s: no path found to its next stop %s; it stays booked where it stands"), *NameOf(Train),
			*StationName(Docked), *StationName(Next));
		return;
	}
	State.PredictedRoute = MoveTemp(Best);
	State.PredictedFrontAhead = ComputeFrontAhead(State, State.PredictedRoute);
	State.PredictedStation = Next;
	++mPlanPredictions;
	UE_LOG(LogAutoTrains, Log, TEXT("planner: %s docked at %s, expected to stay %.0f s; next leg to %s: %d tracks, %.0f m, leaving with %s"), *NameOf(Train),
		*StationName(Docked), GetExpectedDwell(Train, Docked), *StationName(Next), State.PredictedRoute.Num(), BestLength / 100.f,
		BestLocomotive == Train->GetMultipleUnitMaster() ? TEXT("its lead locomotive") : TEXT("another locomotive (the other way)"));
}

void AATTrafficSubsystem::PlanDocked(FATTrafficTrain& State, double Now)
{
	AFGTrain* Train = State.Train.Get();
	const AFGBuildableRailroadStation* Station = State.DockStation.Get();
	const double Start = State.DockedAt >= 0.0 ? State.DockedAt : Now;
	const double Depart = FMath::Max(Start + GetExpectedDwell(Train, Station), Now + OverdueDepartSeconds);
	if (State.PredictedRoute.Num() == 0 && Now - State.PredictedAt > PredictRetrySeconds && Train && Train->IsSelfDrivingEnabled())
	{
		PredictDeparture(State, Now);
	}
	if (State.PredictedRoute.Num() == 0)
	{
		BookStanding(State, Now, Depart, EBookingKind::Predicted);
		return;
	}
	// The next leg, planned like a self-driving train standing at its start until it leaves. It goes before what
	// self-driving trains have booked but don't hold yet: they plan around it at their next turn, so none queues where
	// the departing train needs to pass.
	FATTrafficTrain Leaving = State;
	Leaving.Route = State.PredictedRoute;
	Leaving.FrontAhead = State.PredictedFrontAhead;
	Leaving.Speed = 0.f;
	IndexRoute(Leaving);
	TArray<FWait> Fixed;
	Fixed.Add({0.f, Depart});
	FPlan Plan;
	PlanTrain(Leaving, Now, Fixed, GetExpectedDwell(Train, State.PredictedStation.Get()), EConflictScope::NotUnlockedMoving, Plan);
	Book(Leaving, Plan.Passes, EBookingKind::Predicted);
}

void AATTrafficSubsystem::PlanMoving(FATTrafficTrain& State, double Now)
{
	AFGTrain* Train = State.Train.Get();
	FPlan Plan;
	PlanTrain(State, Now, TArray<FWait>(), GetExpectedDwell(Train, State.GoalStation.Get()), EConflictScope::Ranked, Plan);
	Book(State, Plan.Passes, EBookingKind::Moving);
	++mPlans;

	State.PlanTrackEnter.Init(OpenTime, State.Route.Num());
	State.PlanJointEnter.Init(OpenTime, State.Route.Num());
	State.PlanTrackLocked.Init(false, State.Route.Num());
	State.PlanJointLocked.Init(false, State.Route.Num());
	for (const FPass& Pass : Plan.Passes)
	{
		if (Pass.RouteIndex == INDEX_NONE || IsOpen(Pass.Enter))
		{
			continue;
		}
		const bool bJoint = Pass.Key.Joint != INDEX_NONE;
		(bJoint ? State.PlanJointEnter : State.PlanTrackEnter)[Pass.RouteIndex] = Pass.Enter;
		(bJoint ? State.PlanJointLocked : State.PlanTrackLocked)[Pass.RouteIndex] = Pass.bLocked;
	}
	const FWait* First = Plan.Waits.FindByPredicate([Now](const FWait& Wait) { return Wait.Until > Now; });
	State.PlanWaitAt = First ? First->At : -1.f;
	State.PlanDepart = First ? First->Until : -1.0;
	State.PlanWaitFor = const_cast<AFGTrain*>(Plan.WaitFor);
	State.PlanArrival = State.PlanTrackEnter.Num() > 0 ? State.PlanTrackEnter.Last() : -1.0;
	if (State.SearchedArrival < 0.0 && !IsOpen(State.PlanArrival))
	{
		State.SearchedArrival = State.PlanArrival;
	}
	mPlanWaits += First ? 1 : 0;

	if (Plan.Failure != State.PlanFailure)
	{
		if (!Plan.Failure.IsEmpty())
		{
			++mPlanFailures;
			UE_LOG(LogAutoTrains, Log, TEXT("planner: %s: %s; it stands before it until that train has passed"), *NameOf(Train), *Plan.Failure);
		}
		State.PlanFailure = Plan.Failure;
	}
	const FString Committed = Plan.CommittedWith ? FString::Printf(TEXT("%s at %s"), *NameOf(Plan.CommittedWith), *Plan.CommittedAt) : FString();
	if (Committed != State.PlanCommitted)
	{
		if (!Committed.IsEmpty())
		{
			++mPlanCommitted;
			UE_LOG(LogAutoTrains, Warning, TEXT("planner: %s can't stop short of %s any more, which %s has booked at the same time"), *NameOf(Train),
				*Plan.CommittedAt, *NameOf(Plan.CommittedWith));
		}
		State.PlanCommitted = Committed;
	}
	State.PlanNote = FString::Printf(TEXT("at its goal in %s"), *SecondsText(State.PlanArrival - Now));
	if (First)
	{
		State.PlanNote += FString::Printf(TEXT(", plan waits %.0f m ahead for %s until %s from now"), First->At / 100.f, *NameOf(Plan.WaitFor),
			*SecondsText(First->Until - Now));
	}
}

bool AATTrafficSubsystem::CanEnter(FATTrafficTrain& Self, const FResourceKey& Key, int32 Index, double Enter, bool bLocked, bool bHeldBefore,
	const UFGRailroadTrackConnectionComponent* Via, const UFGRailroadTrackConnectionComponent* Exit, double Now, const FATTrafficTrain*& OutBlocker,
	FString& OutWhy) const
{
	const AFGTrain* SelfTrain = Self.Train.Get();
	if (IsOpen(Enter))
	{
		OutBlocker = mTrains.Find(Self.PlanWaitFor);
		OutWhy = TEXT("its plan has no slot there yet");
		return false;
	}
	// A train ahead of this one on its route the same way is followed through the same track or switch the same way; the
	// follow limit keeps the distance.
	auto IsFollowed = [this, &Self](const FATTrafficTrain* Other) { return Other && Other->bManaged && GetRouteOrder(Self, *Other) > 0; };
	TArray<FResourceKey, TInlineAllocator<4>> Keys;
	if (Key.Joint != INDEX_NONE)
	{
		Keys.Add(Key);
	}
	else
	{
		for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Key.Track))
		{
			FResourceKey Crossing;
			Crossing.Track = Conflict;
			Keys.Add(Crossing);
		}
	}
	// Held from its last grant and locked (it may not stop short of it at service braking): the train keeps it whatever
	// was booked since; only a train standing there or holding it stops it.
	const bool bKeeps = bHeldBefore && bLocked;
	for (const FResourceKey& Checked : Keys)
	{
		const TArray<FBooking>* Bookings = bKeeps ? nullptr : mBookings.Find(Checked);
		if (!Bookings)
		{
			continue;
		}
		for (const FBooking& Booking : *Bookings)
		{
			if (Booking.Owner == SelfTrain)
			{
				continue;
			}
			// A train ahead on this route goes first here, and one behind can't get here before this one has passed. Else
			// locked bookings go first, then bookings in order of entry; ties by pointer so both trains see the same order.
			FATTrafficTrain* const* Other = mStateOf.Find(Booking.Owner);
			const int32 Ahead = Other ? FrontIndexOn(Self, **Other) : INDEX_NONE;
			const bool bAheadHere = Ahead != INDEX_NONE && Ahead <= Index;
			if (!bAheadHere && Other && IsBehind(**Other, Self, Checked))
			{
				continue;
			}
			const bool bEarlier = Booking.Enter < Enter || (Booking.Enter == Enter && Booking.Owner < SelfTrain);
			const bool bFirst = bAheadHere || (Booking.bLocked != bLocked ? Booking.bLocked : bEarlier);
			if (!bFirst)
			{
				continue;
			}
			if (Checked == Key && Exit && Booking.Exit == Exit && Booking.Via == Via && Other && IsFollowed(*Other))
			{
				Self.FollowedTrains.Add((*Other)->Train);
				continue;
			}
			OutBlocker = Other ? *Other : nullptr;
			OutWhy = FString::Printf(TEXT("booked first on %s (%s, enters in %s, clear in %s)"), *ResourceName(Checked.Track, Checked.Joint),
				Booking.bLocked ? TEXT("locked") : Booking.bHeld ? TEXT("held") : Booking.bCommitted ? TEXT("committed") : TEXT("planned"), *SecondsText(Booking.Enter - Now), *SecondsText(Booking.Clear - Now));
			return false;
		}
	}
	// Nobody else stands there or holds it from its own grant, apart from a train followed the same way: only one train
	// at a time holds a resource, whatever the bookings say.
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const FATTrafficTrain& Other = Pair.Value;
		if (&Other == &Self)
		{
			continue;
		}
		bool bHolds = false;
		const AFGBuildableRailroadTrack* On = nullptr;
		if (Key.Joint != INDEX_NONE)
		{
			bHolds = HoldsJoint(Other, Key.Joint);
		}
		else
		{
			for (const FResourceKey& Checked : Keys)
			{
				if (!bHolds && HoldsTrack(Other, Checked.Track))
				{
					bHolds = true;
					On = Checked.Track;
				}
			}
		}
		if (!bHolds)
		{
			continue;
		}
		// Holding it behind this train on the same line: it has to pass this one first.
		FResourceKey HeldKey;
		HeldKey.Track = Key.Joint != INDEX_NONE ? nullptr : On;
		HeldKey.Joint = Key.Joint;
		if (IsBehind(Other, Self, HeldKey))
		{
			continue;
		}
		if ((Key.Joint != INDEX_NONE || On == Key.Track) && IsFollowed(&Other))
		{
			Self.FollowedTrains.Add(Other.Train);
			continue;
		}
		const bool bStands = Key.Joint != INDEX_NONE ? Other.OccupiedJoints.Contains(Key.Joint) : Other.Occupied.Contains(const_cast<AFGBuildableRailroadTrack*>(On));
		OutBlocker = &Other;
		OutWhy = FString::Printf(TEXT("%s %s"), bStands ? TEXT("stands on") : TEXT("holds"), *ResourceName(On, Key.Joint));
		return false;
	}
	return true;
}

void AATTrafficSubsystem::GrantFromPlan(FATTrafficTrain& State, double Now)
{
	// What the train held from its last grant.
	TSet<const AFGBuildableRailroadTrack*> HeldTracks;
	for (const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim : State.Claims)
	{
		HeldTracks.Add(Claim.Get());
	}
	const TSet<int32> HeldJoints = State.ClaimedJoints;
	State.Claims.Reset();
	State.FollowedTrains.Reset();
	State.BlockedBy = nullptr;
	State.BlockedReason.Reset();
	State.BlockedKey.Reset();
	// Enough to stop from the speed it is heading for, so it can speed up.
	const float Heading = FMath::Max(State.Speed, GetRouteSpeed(State).Cruise);
	const float Need = GetStoppingDistance(State, Heading) + State.Length + mSettings.LookaheadMetres * 100.f;
	auto TimeOf = [](const TArray<double>& Times, int32 Index) { return Times.IsValidIndex(Index) ? Times[Index] : OpenTime; };
	auto LockOf = [](const TArray<bool>& Locks, int32 Index) { return Locks.IsValidIndex(Index) && Locks[Index]; };
	bool bBlocked = false;
	// Passing through a deadlock: its way ahead whatever else is there.
	if (State.GhostUntil >= 0.0)
	{
		for (int32 Index = 0; Index < State.Route.Num(); ++Index)
		{
			State.Claims.Add(State.Route[Index].Track);
			if (State.Route[Index].EndDistance - State.FrontAhead >= Need)
			{
				break;
			}
		}
		UpdateAuthority(State);
		UpdateFollowLeaders(State);
		State.ClearSeconds = -1.f;
		return;
	}
	// The plan waits somewhere ahead and doesn't move off right away.
	const bool bPlanWaits = State.PlanWaitAt >= 0.f && State.PlanDepart > Now + PlanWaitGraceSeconds;
	for (int32 Index = 0; Index < State.Route.Num() && !bBlocked; ++Index)
	{
		const FATRouteStep& Step = State.Route[Index];
		const float Begin = Step.EndDistance - Step.Track->GetLength() - State.FrontAhead;
		// Its plan stands it where it is: nothing past what it stands on.
		if (bPlanWaits && State.PlanWaitAt <= 100.f && !State.Occupied.Contains(Step.Track))
		{
			State.BlockedBy = State.PlanWaitFor;
			State.BlockedReason = FString::Printf(TEXT("waits for %s where it stands: its plan moves off in %s"), *NameOf(State.PlanWaitFor.Get()),
				*SecondsText(State.PlanDepart - Now));
			State.BlockedKey = NameOf(State.PlanWaitFor.Get()) + TEXT("|plan|here");
			bBlocked = true;
			break;
		}
		const FATTrafficTrain* Blocker = nullptr;
		FString Why;
		FString Where;
		const UFGRailroadTrackConnectionComponent* Entry = Index > 0 ? State.Route[Index - 1].Exit : nullptr;
		const int32 Joint = GetJoint(Entry);
		if (Joint != INDEX_NONE && !State.OccupiedJoints.Contains(Joint))
		{
			FResourceKey Key;
			Key.Joint = Joint;
			if (!CanEnter(State, Key, Index, TimeOf(State.PlanJointEnter, Index), LockOf(State.PlanJointLocked, Index), HeldJoints.Contains(Joint), Entry, Step.Exit, Now, Blocker, Why))
			{
				Where = FString::Printf(TEXT("the switch before %s"), *ResourceName(Step.Track, INDEX_NONE));
				bBlocked = true;
			}
		}
		if (!bBlocked && !State.Occupied.Contains(Step.Track))
		{
			FResourceKey Key;
			Key.Track = Step.Track;
			if (!CanEnter(State, Key, Index, TimeOf(State.PlanTrackEnter, Index), LockOf(State.PlanTrackLocked, Index), HeldTracks.Contains(Step.Track), nullptr, Step.Exit, Now, Blocker, Why))
			{
				Where = ResourceName(Step.Track, INDEX_NONE);
				bBlocked = true;
			}
		}
		if (bBlocked)
		{
			State.BlockedAtTrack = Where.StartsWith(TEXT("the switch")) ? nullptr : Step.Track;
			State.BlockedAtJoint = Where.StartsWith(TEXT("the switch")) ? Joint : INDEX_NONE;
			State.BlockedAtIndex = Index;
			State.BlockedBy = Blocker ? Blocker->Train : nullptr;
			State.BlockedReason = FString::Printf(TEXT("waits for %s before %s, %.0f m ahead: %s"), *NameOf(Blocker ? Blocker->Train.Get() : nullptr), *Where,
				FMath::Max(Begin, 0.f) / 100.f, *Why);
			// Logged again only when the train or the place changes.
			State.BlockedKey = NameOf(Blocker ? Blocker->Train.Get() : nullptr) + TEXT("|") + Where;
			break;
		}
		State.Claims.Add(Step.Track);
		if (Step.EndDistance - State.FrontAhead >= Need)
		{
			break;
		}
		// No further than where its plan waits: that's a place it can stand without blocking anyone.
		if (bPlanWaits && Step.EndDistance - State.FrontAhead - GetStopMargin(Step.Exit) >= State.PlanWaitAt - 100.f)
		{
			State.BlockedBy = State.PlanWaitFor;
			State.BlockedReason = FString::Printf(TEXT("waits for %s at the end of %s: its plan moves off in %s"), *NameOf(State.PlanWaitFor.Get()),
				*ResourceName(Step.Track, INDEX_NONE), *SecondsText(State.PlanDepart - Now));
			State.BlockedKey = NameOf(State.PlanWaitFor.Get()) + TEXT("|plan|") + ResourceName(Step.Track, INDEX_NONE);
			bBlocked = true;
			break;
		}
	}
	UpdateAuthority(State);
	UpdateFollowLeaders(State);
	// Standing behind a train in its way that holds it there: named, so a cycle of such trains shows as a deadlock.
	if (!bBlocked && State.Speed < FollowStuckSpeed)
	{
		if (const AFGTrain* Train = State.Train.Get())
		{
			float HardLimit = -1.f;
			float PaceSpeed = MAX_flt;
			FATFollowInfo Info;
			GetFollowLimit(Train, State, HardLimit, PaceSpeed, &Info);
			if (HardLimit >= 0.f && HardLimit < FollowStuckDistance && Info.Leader.IsValid())
			{
				State.BlockedBy = Info.Leader;
				State.BlockedReason = FString::Printf(TEXT("stays behind %s, %.0f m ahead"), *NameOf(Info.Leader.Get()), Info.Gap / 100.f);
				State.BlockedKey = NameOf(Info.Leader.Get()) + TEXT("|behind");
			}
		}
	}
	// Held where the plan waits: roll up to arrive as it moves off.
	State.ClearSeconds = bBlocked && State.PlanDepart > Now && !IsOpen(State.PlanDepart) ? State.PlanDepart - Now : -1.f;
}

float AATTrafficSubsystem::GetSearchStepSeconds(const AFGTrain* Self, const AFGBuildableRailroadTrack* Track, const UFGRailroadTrackConnectionComponent* Exit,
	double Now, float Arrival, const FRouteSpeed& Speed) const
{
	const double Headway = GetSafetyMarginSeconds();
	FATTrafficTrain* const* SelfState = mStateOf.Find(Self);
	const float Length = Track->GetLength();
	const float Drive = Length / Speed.Cruise;
	const double Reached = Now + Arrival;
	const UFGRailroadTrackConnectionComponent* Entry = Exit ? Exit->GetOpposite() : nullptr;
	const int32 Joint = GetJoint(Entry);
	const double FoulBefore = Joint != INDEX_NONE ? GetFoulDistance(Entry) / Speed.Cruise : 0.0;
	const double JointHeld = Joint != INDEX_NONE ? (Speed.Length + GetJointFoulDistance(Joint)) / Speed.Cruise : 0.0;
	const double TrackHeld = (Length + Speed.Length) / Speed.Cruise;
	double Enter = Reached;
	for (int32 Shift = 0; Shift < EntryMaxShifts; ++Shift)
	{
		double Need = Enter;
		const double Clear = Enter + TrackHeld;
		for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Track))
		{
			FResourceKey Key;
			Key.Track = Conflict;
			const TArray<FBooking>* Bookings = mBookings.Find(Key);
			if (!Bookings)
			{
				continue;
			}
			for (const FBooking& Booking : *Bookings)
			{
				if (Booking.Owner == Self || (SelfState && !IsInScope(**SelfState, Booking, Booking.bLocked, EConflictScope::Ranked)))
				{
					continue;
				}
				if (Conflict == Track && Booking.Exit == Exit)
				{
					// Following the same way: a headway behind it at both ends, or ahead of it.
					const bool bBehind = Enter >= Booking.Enter + Headway && Clear >= Booking.Clear + Headway;
					const bool bAhead = !Booking.bLocked && Booking.Enter >= Enter + Headway && Booking.Clear >= Clear + Headway;
					if (!bBehind && !bAhead)
					{
						Need = FMath::Max(Need, IsOpen(Booking.Clear) ? OpenTime : Enter + FMath::Max(Booking.Enter + Headway - Enter, Booking.Clear + Headway - Clear));
					}
				}
				else if (Enter < Booking.Clear + Headway && (Booking.bLocked || Clear + Headway > Booking.Enter))
				{
					Need = FMath::Max(Need, IsOpen(Booking.Clear) ? OpenTime : Booking.Clear + Headway);
				}
			}
		}
		if (Joint != INDEX_NONE)
		{
			FResourceKey Key;
			Key.Joint = Joint;
			if (const TArray<FBooking>* Bookings = mBookings.Find(Key))
			{
				for (const FBooking& Booking : *Bookings)
				{
					if (Booking.Owner != Self && (!SelfState || IsInScope(**SelfState, Booking, Booking.bLocked, EConflictScope::Ranked)) && Enter - FoulBefore < Booking.Clear + Headway && (Booking.bLocked || Enter + JointHeld + Headway > Booking.Enter))
					{
						Need = FMath::Max(Need, IsOpen(Booking.Clear) ? OpenTime : Booking.Clear + Headway + FoulBefore);
					}
				}
			}
		}
		if (IsOpen(Need))
		{
			return SearchBlockedSeconds;
		}
		if (Need <= Enter + 1e-3)
		{
			break;
		}
		Enter = Need;
	}
	// Waiting costs the wait and, up to a full stop, the time lost braking and picking up speed again.
	const double Wait = Enter - Reached;
	return static_cast<float>(Wait + FMath::Min(Wait, static_cast<double>(Speed.StopLoss))) + Drive;
}

void AATTrafficSubsystem::ConsiderPlannedReroute(FATTrafficTrain& State, double Now, float DeltaSeconds)
{
	AFGTrain* Train = State.Train.Get();
	if (!Train || !mRerouteEnabled || !Train->mAtcData.Path.IsValid())
	{
		return;
	}
	if (State.RerouteTimer < 0.f)
	{
		// Spread the searches out over the interval.
		State.RerouteTimer = FMath::FRandRange(0.f, SearchIntervalSeconds);
	}
	State.RerouteTimer -= DeltaSeconds;
	if (!State.PlanFailure.IsEmpty())
	{
		State.RerouteTimer = FMath::Min(State.RerouteTimer, SearchFailedSeconds);
	}
	const bool bWorse = State.SearchedArrival >= 0.0 && !IsOpen(State.SearchedArrival) && State.PlanArrival - State.SearchedArrival > SearchWorseSeconds;
	if ((State.RerouteTimer > 0.f && !bWorse) || State.RerouteCooldown > 0.f || State.bAuthorityToGoal || State.Route.Num() < 3)
	{
		return;
	}
	State.RerouteTimer = State.PlanFailure.IsEmpty() ? SearchIntervalSeconds : SearchFailedSeconds;
	State.SearchedArrival = State.PlanArrival;

	// The route can only change beyond what the train holds and what it can no longer stop short of.
	const float Stopping = GetStoppingDistance(State, State.Speed);
	int32 Commit = 0;
	for (int32 Index = 0; Index < State.Route.Num(); ++Index)
	{
		const FATRouteStep& Step = State.Route[Index];
		if (!HoldsTrack(State, Step.Track) && Step.EndDistance - Step.Track->GetLength() - State.FrontAhead >= Stopping + GetStopMargin(Step.Exit))
		{
			break;
		}
		Commit = Index;
	}
	if (Commit >= State.Route.Num() - 2)
	{
		return;
	}
	const double Begin = FPlatformTime::Seconds();
	UFGRailroadTrackConnectionComponent* Start = State.Route[Commit].Exit;
	UFGRailroadTrackConnectionComponent* Goal = State.Route.Last().Exit;
	const AFGTrain* Self = Train;
	const FRouteSpeed Speed = GetRouteSpeed(State);
	auto Step = [this, Self, Now, &Speed](const AFGBuildableRailroadTrack* Track, const UFGRailroadTrackConnectionComponent* Exit, float Arrival)
	{
		return GetSearchStepSeconds(Self, Track, Exit, Now, Arrival, Speed);
	};
	// Both routes are timed from the end of the committed part, as the search times them.
	const float StartSeconds = TimeToCover(FMath::Max(0.f, State.Route[Commit].EndDistance - State.FrontAhead), State.Speed, Speed.Accel, Speed.Cruise);
	float CurrentSeconds = StartSeconds;
	for (int32 Index = Commit + 1; Index < State.Route.Num(); ++Index)
	{
		CurrentSeconds += Step(State.Route[Index].Track, State.Route[Index].Exit, CurrentSeconds);
	}
	const TArray<UFGRailroadTrackConnectionComponent*> Alternative = FATRouter::FindFastestRoute(Start, Goal, StartSeconds, Step);
	if (Alternative.Num() < 2)
	{
		return;
	}
	bool bSame = Alternative.Num() == State.Route.Num() - Commit;
	float NewSeconds = StartSeconds;
	for (int32 Index = 1; Index < Alternative.Num(); ++Index)
	{
		NewSeconds += Step(Alternative[Index]->GetTrack(), Alternative[Index], NewSeconds);
		bSame = bSame && Alternative[Index] == State.Route[Commit + Index].Exit;
	}
	if (bSame || NewSeconds > CurrentSeconds - FMath::Max(mSettings.RerouteMinGainSeconds, CurrentSeconds * mSettings.RerouteMinGainPercent / 100.f))
	{
		return;
	}

	TArray<UFGRailroadTrackConnectionComponent*> Connections;
	for (int32 Index = 0; Index <= Commit; ++Index)
	{
		Connections.Add(State.Route[Index].Exit);
	}
	Connections.Append(&Alternative[1], Alternative.Num() - 1);
	const FRailroadPathSharedPtr Path = FATRouter::BuildPath(Connections, Train->mAtcData.Path->Station.Get());
	const bool bApplied = FATRouter::ApplyPath(Train, Path, Goal);
	++State.Reroutes;
	++mPlanReroutes;
	State.RerouteCooldown = mSettings.RerouteHoldSeconds;
	State.SearchedArrival = -1.0;
	UE_LOG(LogAutoTrains, Log, TEXT("planner: %s rerouted after %s: %.0f -> %.0f s to its goal through the bookings, %d -> %d tracks, applied %s (%.2f ms)"),
		*NameOf(Train), *ResourceName(State.Route[Commit].Track, INDEX_NONE), CurrentSeconds, NewSeconds, State.Route.Num() - Commit - 1, Alternative.Num() - 1,
		bApplied ? TEXT("yes") : TEXT("NO"), (FPlatformTime::Seconds() - Begin) * 1000.0);
	if (!bApplied)
	{
		return;
	}
	// The route as the new path has it, until the next update reads it from the train; the path starts at the first step.
	TArray<FATRouteStep> Route;
	for (int32 Index = 0; Index <= Commit; ++Index)
	{
		Route.Add(State.Route[Index]);
	}
	float Distance = State.Route[Commit].EndDistance;
	for (int32 Index = 1; Index < Alternative.Num(); ++Index)
	{
		AFGBuildableRailroadTrack* Track = Alternative[Index]->GetTrack();
		Distance += Track->GetLength();
		Route.Add({Track, Alternative[Index], Distance});
	}
	State.Route = MoveTemp(Route);
	State.RoutePath = Train->mAtcData.Path.Get();
	State.RouteFirstSegment = 0;
	IndexRoute(State);
}

void AATTrafficSubsystem::UpdatePlannedTraffic(float DeltaSeconds)
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}
	const double Begin = FPlatformTime::Seconds();
	const double Now = World->GetTimeSeconds();

	mStateOf.Reset();
	TSet<FObjectKey> Present;
	for (TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		if (AFGTrain* Train = Pair.Value.Train.Get())
		{
			mStateOf.Add(Train, &Pair.Value);
			Present.Add(FObjectKey(Train));
			IndexRoute(Pair.Value);
		}
	}
	// Bookings of trains that are gone.
	TArray<const AFGTrain*> Gone;
	for (const TPair<FObjectKey, FOwnerBookings>& Pair : mOwnerBookings)
	{
		if (!Present.Contains(Pair.Key))
		{
			Gone.Add(Pair.Value.Owner);
		}
	}
	for (const AFGTrain* Booker : Gone)
	{
		RemoveBookings(Booker);
	}

	// Turns: trains the mod doesn't drive first (they only stand), docked trains by departure, then self-driving trains
	// in rank order. Each removes only its own bookings and books around everyone else's in its scope.
	struct FTurn
	{
		FATTrafficTrain* State = nullptr;
		float Order = 0.f;
		int32 Kind = 0;
	};
	TArray<FTurn> Turns;
	TArray<FATTrafficTrain*> SelfDriving;
	for (const TPair<const AFGTrain*, FATTrafficTrain*>& Pair : mStateOf)
	{
		FATTrafficTrain& State = *Pair.Value;
		const AFGTrain* Train = Pair.Key;
		State.PlanRank = INDEX_NONE;
		if (State.bManaged && State.Route.Num() > 0)
		{
			if (State.LegStart < 0.0 || State.LegGoal != State.GoalStation)
			{
				State.LegStart = Now;
				State.LegGoal = State.GoalStation;
			}
			SelfDriving.Add(&State);
			continue;
		}
		FTurn Turn;
		Turn.State = &State;
		if (Train->IsDocked())
		{
			Turn.Kind = 1;
			Turn.Order = State.DockedAt >= 0.0 ? State.DockedAt + GetExpectedDwell(Train, State.DockStation.Get()) - Now : 0.f;
		}
		Turns.Add(Turn);
	}
	Turns.Sort([](const FTurn& A, const FTurn& B)
	{
		if (A.Kind != B.Kind)
		{
			return A.Kind < B.Kind;
		}
		if (A.Order != B.Order)
		{
			return A.Order < B.Order;
		}
		return NameOf(A.State->Train.Get()) < NameOf(B.State->Train.Get());
	});
	RankTrains(SelfDriving);
	for (FATTrafficTrain* State : SelfDriving)
	{
		Turns.Add({State, 0.f, 2});
	}
	for (const FTurn& Turn : Turns)
	{
		FATTrafficTrain& State = *Turn.State;
		RemoveBookings(State.Train.Get());
		if (Turn.Kind == 2)
		{
			ConsiderPlannedReroute(State, Now, DeltaSeconds);
			PlanMoving(State, Now);
		}
		else if (Turn.Kind == 1)
		{
			PlanDocked(State, Now);
		}
		else
		{
			BookStanding(State, Now, OpenTime, EBookingKind::Standing);
		}
	}

	// Movement authority from the bookings, in the same order.
	for (const FTurn& Turn : Turns)
	{
		if (Turn.Kind == 2)
		{
			GrantFromPlan(*Turn.State, Now);
		}
	}
	// Check: no two trains hold conflicting resources, apart from following the same way or passing through a deadlock.
	for (const FTurn& Turn : Turns)
	{
		const FATTrafficTrain& State = *Turn.State;
		if (Turn.Kind != 2 || State.GhostUntil >= 0.0)
		{
			continue;
		}
		for (const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim : State.Claims)
		{
			const AFGBuildableRailroadTrack* Track = Claim.Get();
			if (!Track || State.Occupied.Contains(const_cast<AFGBuildableRailroadTrack*>(Track)))
			{
				continue;
			}
			for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Track))
			{
				for (const TPair<const AFGTrain*, FATTrafficTrain*>& Pair : mStateOf)
				{
					const FATTrafficTrain& Other = *Pair.Value;
					if (&Other == &State || Other.GhostUntil >= 0.0 || !HoldsTrack(Other, Conflict) || (Conflict == Track && (State.FollowedTrains.Contains(Other.Train) || Other.FollowedTrains.Contains(State.Train))))
					{
						continue;
					}
					// One behind the other on the same line: the one behind can't get there first.
					FResourceKey ClaimKey;
					ClaimKey.Track = Track;
					FResourceKey HeldKey;
					HeldKey.Track = Conflict;
					if (IsBehind(Other, State, HeldKey) || IsBehind(State, Other, ClaimKey))
					{
						continue;
					}
					const FString Pair2 = NameOf(State.Train.Get()) + TEXT("|") + NameOf(Other.Train.Get());
					if (Pair2 != mLastDoubleHold)
					{
						mLastDoubleHold = Pair2;
						UE_LOG(LogAutoTrains, Warning, TEXT("planner: CHECK FAILED: %s was granted %s while %s holds %s"), *NameOf(State.Train.Get()), *ResourceName(Track, INDEX_NONE),
							*NameOf(Other.Train.Get()), *ResourceName(Conflict, INDEX_NONE));
					}
				}
			}
		}
	}

	// Check: no train stands on a track crossing one another train holds; logged with both trains' state, once a pair.
	for (const FTurn& Turn : Turns)
	{
		const FATTrafficTrain& State = *Turn.State;
		for (const AFGBuildableRailroadTrack* Track : State.Occupied)
		{
			for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Track))
			{
				if (Conflict == Track)
				{
					continue;
				}
				for (const TPair<const AFGTrain*, FATTrafficTrain*>& Pair : mStateOf)
				{
					const FATTrafficTrain& Other = *Pair.Value;
					if (&Other == &State || !HoldsTrack(Other, Conflict))
					{
						continue;
					}
					const FString Key = NameOf(State.Train.Get()) + TEXT("|") + NameOf(Other.Train.Get()) + TEXT("|") + ResourceName(Track, INDEX_NONE);
					if (mLoggedCrossings.Contains(Key))
					{
						continue;
					}
					mLoggedCrossings.Add(Key);
					const AFGTrain* OtherTrain = Other.Train.Get();
					const float OtherAt = OtherTrain ? GetDistanceToTrackStart(OtherTrain, Other, Conflict) : -1.f;
					UE_LOG(LogAutoTrains, Warning, TEXT("planner: CROSSING: %s (%s, %.0f km/h, granted it: %s, may drive %.0f m%s%s) stands on %s, which crosses %s held by %s (%s, %.0f km/h, %s, it %s)"),
						*NameOf(State.Train.Get()), State.bManaged ? TEXT("driving") : TEXT("not driven"), State.Speed * 0.036f,
						State.Claims.ContainsByPredicate([Track](const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim) { return Claim.Get() == Track; }) ? TEXT("yes") : TEXT("no"),
						GetAuthorityDistance(State.Train.Get()) / 100.f, State.BlockedReason.IsEmpty() ? TEXT("") : TEXT(", "), *State.BlockedReason,
						*ResourceName(Track, INDEX_NONE), *ResourceName(Conflict, INDEX_NONE), *NameOf(OtherTrain), Other.bManaged ? TEXT("driving") : TEXT("not driven"),
						Other.Speed * 0.036f, Other.Occupied.Contains(const_cast<AFGBuildableRailroadTrack*>(Conflict)) ? TEXT("stands on it") : TEXT("granted it"),
						OtherAt > -1e8f ? *FString::Printf(TEXT("%.0f m ahead"), OtherAt / 100.f) : TEXT("off its path"));
				}
			}
		}
	}
	if (mLoggedCrossings.Num() > 500)
	{
		mLoggedCrossings.Reset();
	}
	if (Now >= mWaitDumpAt)
	{
		mWaitDumpAt = Now + WaitDumpInterval;
		for (const FTurn& Turn : Turns)
		{
			if (Turn.Kind == 2 && Turn.State->WaitingSeconds > WaitDumpSeconds)
			{
				LogWaitDump(*Turn.State, Now);
			}
		}
	}

	++mPlanUpdates;
	const double Took = FPlatformTime::Seconds() - Begin;
	mPlanSeconds += Took;
	if (Took > SlowUpdateSeconds && Now >= mPlanSlowLoggedAt + SlowLogSeconds)
	{
		UE_LOG(LogAutoTrains, Warning, TEXT("planner: an update took %.1f ms (%d trains)"), Took * 1000.0, Turns.Num());
		mPlanSlowLoggedAt = Now;
	}
	if (Now >= mPlanSummaryAt)
	{
		int32 Driving = 0;
		for (const FTurn& Turn : Turns)
		{
			Driving += Turn.Kind == 2 ? 1 : 0;
		}
		int32 Bookings = 0;
		for (const TPair<FResourceKey, TArray<FBooking>>& Pair : mBookings)
		{
			Bookings += Pair.Value.Num();
		}
		UE_LOG(LogAutoTrains, Log, TEXT("planner: last %.0f s: %d trains (%d driving), %d bookings now, %.2f ms per update; %d plans, %d with a wait, %d failed, %d committed conflicts, %d reroutes, %d departures predicted"),
			PlanSummarySeconds, Turns.Num(), Driving, Bookings, mPlanUpdates > 0 ? mPlanSeconds * 1000.0 / mPlanUpdates : 0.0, mPlans, mPlanWaits, mPlanFailures,
			mPlanCommitted, mPlanReroutes, mPlanPredictions);
		mPlans = mPlanWaits = mPlanFailures = mPlanCommitted = mPlanReroutes = mPlanPredictions = mPlanUpdates = 0;
		mPlanSeconds = 0.0;
		mPlanSummaryAt = Now + PlanSummarySeconds;
	}
}

void AATTrafficSubsystem::LogWaitDump(const FATTrafficTrain& State, double Now) const
{
	auto Describe = [this, Now](const FATTrafficTrain* Other) -> FString
	{
		if (!Other)
		{
			return TEXT("?");
		}
		const AFGTrain* Train = Other->Train.Get();
		FString Drive;
		if (Train && Other->bManaged)
		{
			float HardLimit = -1.f;
			float PaceSpeed = MAX_flt;
			FATFollowInfo Info;
			GetFollowLimit(Train, *Other, HardLimit, PaceSpeed, &Info);
			const float Authority = GetAuthorityDistance(Train);
			Drive = FString::Printf(TEXT(", may drive %s, target %.0f km/h%s"), Authority < 0.f ? TEXT("to its goal") : *FString::Printf(TEXT("%.0f m"), Authority / 100.f),
				Train->mAtcData.TargetSpeed * 0.036f,
				HardLimit >= 0.f ? *FString::Printf(TEXT(", follows %s with %.0f m"), *NameOf(Info.Leader.Get()), HardLimit / 100.f) : TEXT(""));
		}
		return FString::Printf(TEXT("%s (%s, rank %d, %.0f km/h, waited %.0f s, %d claims%s%s%s)"), *NameOf(Train), Other->bManaged ? TEXT("driving") : TEXT("not driven"),
			Other->PlanRank, Other->Speed * 0.036f, Other->WaitingSeconds, Other->Claims.Num(), *Drive, Other->BlockedReason.IsEmpty() ? TEXT("") : TEXT(", "), *Other->BlockedReason);
	};
	const int32 Index = State.BlockedAtIndex;
	const bool bJoint = State.BlockedAtJoint != INDEX_NONE;
	const TArray<double>& Enters = bJoint ? State.PlanJointEnter : State.PlanTrackEnter;
	const TArray<bool>& Locks = bJoint ? State.PlanJointLocked : State.PlanTrackLocked;
	FString Lines = FString::Printf(TEXT("planner: WAIT %s: %s; %s; its booking there %s%s"), *Describe(&State), *State.PlanNote, *ResourceName(State.BlockedAtTrack, State.BlockedAtJoint),
		Enters.IsValidIndex(Index) ? *FString::Printf(TEXT("enters in %s"), *SecondsText(Enters[Index] - Now)) : TEXT("missing"),
		Locks.IsValidIndex(Index) && Locks[Index] ? TEXT(", locked") : TEXT(""));
	TArray<FResourceKey, TInlineAllocator<4>> Keys;
	if (bJoint)
	{
		FResourceKey Key;
		Key.Joint = State.BlockedAtJoint;
		Keys.Add(Key);
	}
	else if (State.BlockedAtTrack)
	{
		for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(State.BlockedAtTrack))
		{
			FResourceKey Key;
			Key.Track = Conflict;
			Keys.Add(Key);
		}
	}
	for (const FResourceKey& Key : Keys)
	{
		const TArray<FBooking>* Bookings = mBookings.Find(Key);
		for (const FBooking& Booking : Bookings ? *Bookings : TArray<FBooking>())
		{
			FATTrafficTrain* const* Other = mStateOf.Find(Booking.Owner);
			Lines += FString::Printf(TEXT("\n    %s: %s %s-%s s%s%s%s, kind %d; %s"), *ResourceName(Key.Track, Key.Joint), *NameOf(Booking.Owner), *SecondsText(Booking.Enter - Now),
				*SecondsText(Booking.Clear - Now), Booking.bLocked ? TEXT(" locked") : TEXT(""), Booking.bHeld ? TEXT(" held") : TEXT(""),
				Booking.bCommitted ? TEXT(" committed") : TEXT(""), static_cast<int32>(Booking.Kind),
				Other ? *Describe(*Other) : TEXT("gone"));
		}
	}
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Lines);
}

void AATTrafficSubsystem::IndexRoute(FATTrafficTrain& State) const
{
	State.RouteIndexOf.Reset();
	State.JointIndexOf.Reset();
	for (int32 Index = 0; Index < State.Route.Num(); ++Index)
	{
		State.RouteIndexOf.FindOrAdd(State.Route[Index].Track, Index);
		const int32 Joint = Index > 0 ? GetJoint(State.Route[Index - 1].Exit) : INDEX_NONE;
		if (Joint != INDEX_NONE)
		{
			State.JointIndexOf.FindOrAdd(Joint, Index);
		}
	}
}

int32 AATTrafficSubsystem::FrontIndexOn(const FATTrafficTrain& Of, const FATTrafficTrain& Other) const
{
	if (Other.Route.Num() == 0)
	{
		return INDEX_NONE;
	}
	const int32* Index = Of.RouteIndexOf.Find(Other.Route[0].Track);
	if (!Index || Of.Route[*Index].Exit != Other.Route[0].Exit)
	{
		return INDEX_NONE;
	}
	// On Of's front track: ahead only if its front is nearer the track's end.
	return *Index > 0 || Other.Route[0].EndDistance - Other.FrontAhead < Of.Route[0].EndDistance - Of.FrontAhead ? *Index : INDEX_NONE;
}

bool AATTrafficSubsystem::IsInScope(const FATTrafficTrain& Self, const FBooking& Booking, bool bFirst, EConflictScope Scope) const
{
	if (bFirst)
	{
		return true;
	}
	switch (Scope)
	{
	case EConflictScope::LockedOnly:
		return false;
	case EConflictScope::NotUnlockedMoving:
		return Booking.Kind != EBookingKind::Moving || Booking.bHeld || Booking.bCommitted;
	default:
		return Booking.Kind != EBookingKind::Moving || Booking.bHeld || Booking.bCommitted || !PlansAfter(Booking.Owner, Self);
	}
}

bool AATTrafficSubsystem::PlansAfter(const AFGTrain* Booker, const FATTrafficTrain& Self) const
{
	FATTrafficTrain* const* Other = mStateOf.Find(Booker);
	return Other && (*Other)->PlanRank != INDEX_NONE && Self.PlanRank != INDEX_NONE && (*Other)->PlanRank > Self.PlanRank;
}

void AATTrafficSubsystem::RankTrains(TArray<FATTrafficTrain*>& Trains) const
{
	Trains.Sort([](const FATTrafficTrain& A, const FATTrafficTrain& B)
	{
		if (A.LegStart != B.LegStart)
		{
			return A.LegStart < B.LegStart;
		}
		return NameOf(A.Train.Get()) < NameOf(B.Train.Get());
	});
	// Ahead[I]: how many of the trains not yet ranked are ahead of train I on its route.
	const int32 Count = Trains.Num();
	TArray<bool> Behind;
	Behind.SetNumZeroed(Count * Count);
	TArray<int32> Ahead;
	Ahead.SetNumZeroed(Count);
	for (int32 I = 0; I < Count; ++I)
	{
		for (int32 J = 0; J < Count; ++J)
		{
			if (I != J && FrontIndexOn(*Trains[I], *Trains[J]) != INDEX_NONE)
			{
				Behind[I * Count + J] = true;
				++Ahead[I];
			}
		}
	}
	// The first in leg order with none ahead of it, or the first in leg order if each has one ahead (a loop).
	TArray<FATTrafficTrain*> Ranked;
	TArray<bool> Done;
	Done.SetNumZeroed(Count);
	while (Ranked.Num() < Count)
	{
		int32 Pick = INDEX_NONE;
		for (int32 I = 0; I < Count && Pick == INDEX_NONE; ++I)
		{
			if (!Done[I] && Ahead[I] == 0)
			{
				Pick = I;
			}
		}
		for (int32 I = 0; I < Count && Pick == INDEX_NONE; ++I)
		{
			if (!Done[I])
			{
				Pick = I;
			}
		}
		Done[Pick] = true;
		Trains[Pick]->PlanRank = Ranked.Num();
		Ranked.Add(Trains[Pick]);
		for (int32 I = 0; I < Count; ++I)
		{
			if (Behind[I * Count + Pick])
			{
				--Ahead[I];
			}
		}
	}
	Trains = MoveTemp(Ranked);
}

bool AATTrafficSubsystem::IsBehind(const FATTrafficTrain& Other, const FATTrafficTrain& Self, const FResourceKey& Key) const
{
	// Self's front on Other's route, Key further along it, and both routes the same tracks from there up to Key: Self
	// gets there first along one line.
	const int32 SelfAt = FrontIndexOn(Other, Self);
	if (SelfAt == INDEX_NONE)
	{
		return false;
	}
	const int32* OtherKeyAt = Key.Joint != INDEX_NONE ? Other.JointIndexOf.Find(Key.Joint) : Other.RouteIndexOf.Find(Key.Track);
	const int32* SelfKeyAt = Key.Joint != INDEX_NONE ? Self.JointIndexOf.Find(Key.Joint) : Self.RouteIndexOf.Find(Key.Track);
	if (!OtherKeyAt || !SelfKeyAt || *OtherKeyAt <= SelfAt || *OtherKeyAt - SelfAt != *SelfKeyAt)
	{
		return false;
	}
	for (int32 Step = 0; Step <= *SelfKeyAt; ++Step)
	{
		if (Other.Route[SelfAt + Step].Track != Self.Route[Step].Track)
		{
			return false;
		}
	}
	return true;
}

void AATTrafficSubsystem::CollectBlocks(const FATTrafficTrain& Self, const FPass& Pass, double Headway, EConflictScope Scope, TArray<FBlock>& Out) const
{
	const AFGTrain* SelfTrain = Self.Train.Get();
	auto Check = [&](const FResourceKey& Key)
	{
		const TArray<FBooking>* Bookings = mBookings.Find(Key);
		if (!Bookings)
		{
			return;
		}
		for (const FBooking& Booking : *Bookings)
		{
			if (Booking.Owner == SelfTrain)
			{
				continue;
			}
			// Nobody goes ahead of a locked booking or of a train ahead on this line; one behind on it can't get there first.
			bool bFirst = Booking.bLocked;
			if (Pass.RouteIndex != INDEX_NONE)
			{
				if (FATTrafficTrain* const* Other = mStateOf.Find(Booking.Owner))
				{
					const int32 Ahead = FrontIndexOn(Self, **Other);
					if (Ahead != INDEX_NONE && Ahead <= Pass.RouteIndex)
					{
						bFirst = true;
					}
					else if (IsBehind(**Other, Self, Key))
					{
						continue;
					}
				}
			}
			if (!IsInScope(Self, Booking, bFirst, Scope))
			{
				continue;
			}
			const double Clear =IsOpen(Booking.Clear) ? OpenTime : Booking.Clear + Headway;
			if (Key == Pass.Key && Pass.Exit && Booking.Exit == Pass.Exit && Booking.Via == Pass.Via)
			{
				// The same way: behind a train that goes first, else just not entering together.
				Out.Add({bFirst ? -OpenTime : Booking.Enter - Headway, Booking.Enter + Headway, Booking.Owner});
				continue;
			}
			Out.Add({bFirst ? -OpenTime : Booking.Enter - Headway, Clear, Booking.Owner});
		}
	};
	if (Pass.Key.Joint != INDEX_NONE)
	{
		Check(Pass.Key);
		return;
	}
	for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Pass.Key.Track))
	{
		FResourceKey Key;
		Key.Track = Conflict;
		Check(Key);
	}
}

bool AATTrafficSubsystem::PlanSafeIntervals(const FATTrafficTrain& State, const FRouteSpeed& Speed, double Now, const TArray<FPass>& Passes, double MinDepart,
	double GoalDwell, EConflictScope Scope, float Committed, TArray<FWait>& OutWaits, TArray<const AFGTrain*>& OutCauses) const
{
	OutWaits.Reset();
	OutCauses.Reset();
	if (State.Route.Num() == 0)
	{
		return true;
	}
	// Stops: where the front is now, the end of each route track short by its stop margin, and the goal.
	TArray<float> Stops;
	Stops.Add(0.f);
	for (int32 Index = 0; Index < State.Route.Num(); ++Index)
	{
		const FATRouteStep& Step = State.Route[Index];
		const bool bGoal = Index == State.Route.Num() - 1;
		const float At = Step.EndDistance - State.FrontAhead - (bGoal ? 0.f : GetStopMargin(Step.Exit));
		if (At > Stops.Last() + SippMinStopGap)
		{
			Stops.Add(At);
		}
	}
	const int32 Count = Stops.Num();
	// Driving time from each stop to the next without stopping, and what stopping first adds. [s]
	TArray<double> Drive;
	Drive.SetNumZeroed(Count);
	for (int32 Index = 0; Index + 1 < Count; ++Index)
	{
		const float Length = Stops[Index + 1] - Stops[Index];
		Drive[Index] = Index == 0 ? TimeToCover(Length, State.Speed, Speed.Accel, Speed.Cruise) : Length / Speed.Cruise;
	}
	const bool bStandingNow = State.Speed < SippStandingSpeed || MinDepart > Now;
	auto CanWait = [&](int32 Index) { return Index == 0 ? bStandingNow : Stops[Index] >= Committed; };
	auto Duration = [&](int32 Index, bool bStopped) { return Drive[Index] + (bStopped && !(Index == 0 && State.Speed < SippStandingSpeed) ? Speed.StopLoss : 0.0); };

	// What standing at each stop and driving each move must keep clear of: every other booking on what the train's
	// body covers there, or touches on the way.
	TArray<TArray<FBlock>> Standing;
	TArray<TArray<FBlock>> Moving;
	Standing.SetNum(Count);
	Moving.SetNum(Count);
	TArray<FBlock> Blocks;
	for (const FPass& Pass : Passes)
	{
		// Its own whatever others booked: they plan around it.
		if (Pass.bLocked)
		{
			continue;
		}
		Blocks.Reset();
		CollectBlocks(State, Pass, GetSafetyMarginSeconds(), Pass.bHeld ? EConflictScope::LockedOnly : Scope, Blocks);
		if (Blocks.Num() == 0)
		{
			continue;
		}
		int32 First = 0;
		while (First < Count && Stops[First] < Pass.EnterAt)
		{
			++First;
		}
		int32 Last = First - 1;
		while (Last + 1 < Count && Stops[Last + 1] < Pass.ClearAt)
		{
			++Last;
		}
		for (int32 Index = First; Index <= Last; ++Index)
		{
			Standing[Index].Append(Blocks);
		}
		for (int32 Index = FMath::Max(First - 1, 0); Index <= Last && Index + 1 < Count; ++Index)
		{
			Moving[Index].Append(Blocks);
		}
	}
	TArray<TArray<FSippInterval>> Free;
	Free.SetNum(Count);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		Free[Index] = GetFreeIntervals(Standing[Index], Now);
	}
	// The train is at the first stop now, whatever is booked there.
	if (Free[0].Num() == 0 || Free[0][0].Start > Now)
	{
		Free[0].Insert({Now, Now, nullptr}, 0);
	}
	// At the goal it stays its dwell and pulls away.
	const double Stay = IsOpen(GoalDwell) ? OpenTime : GoalDwell + RestTime(State.Length, Speed.Cruise, Speed.Accel);

	TArray<TArray<FSippLabel>> Labels;
	Labels.SetNum(Count);
	Labels[0].Add({Now, 0, Now, INDEX_NONE, nullptr});
	auto Offer = [&](int32 Index, int32 Interval, double Arrive, double Depart, int32 Prev, const AFGTrain* Cause)
	{
		if (Index == Count - 1 && Arrive + Stay > Free[Index][Interval].End && !IsOpen(Free[Index][Interval].End))
		{
			return;
		}
		FSippLabel* Known = Labels[Index].FindByPredicate([Interval](const FSippLabel& Label) { return Label.Interval == Interval; });
		if (!Known)
		{
			Labels[Index].Add({Arrive, Interval, Depart, Prev, Cause});
		}
		else if (Arrive < Known->Arrive)
		{
			*Known = {Arrive, Interval, Depart, Prev, Cause};
		}
	};
	for (int32 Index = 0; Index + 1 < Count; ++Index)
	{
		for (int32 LabelIndex = 0; LabelIndex < Labels[Index].Num(); ++LabelIndex)
		{
			const FSippLabel Label = Labels[Index][LabelIndex];
			const FSippInterval& Here = Free[Index][Label.Interval];
			const double Earliest = Index == 0 ? FMath::Max(Label.Arrive, MinDepart) : Label.Arrive;
			for (int32 Next = 0; Next < Free[Index + 1].Num(); ++Next)
			{
				const FSippInterval& There = Free[Index + 1][Next];
				// Straight on, leaving as soon as it got here.
				if (Earliest <= Label.Arrive)
				{
					const double Arrive = Label.Arrive + Duration(Index, false);
					const AFGTrain* Unused = nullptr;
					if (Arrive >= There.Start && Arrive <= There.End && GetEarliestDeparture(Moving[Index], Label.Arrive, Label.Arrive, Duration(Index, false), Unused) >= 0.0)
					{
						Offer(Index + 1, Next, Arrive, Label.Arrive, LabelIndex, nullptr);
						continue;
					}
				}
				// Waiting here first.
				if (!CanWait(Index))
				{
					continue;
				}
				const double Move = Duration(Index, true);
				const AFGTrain* Cause = There.Start - Move > Earliest ? There.After : nullptr;
				const double Depart = GetEarliestDeparture(Moving[Index], FMath::Max(Earliest, There.Start - Move), FMath::Min(Here.End, There.End - Move), Move, Cause);
				if (Depart >= 0.0)
				{
					Offer(Index + 1, Next, Depart + Move, Depart, LabelIndex, Cause);
				}
			}
		}
	}

	// The goal if reached, else the furthest stop reached, where the train then stands for good.
	int32 Reached = Count - 1;
	while (Reached > 0 && Labels[Reached].Num() == 0)
	{
		--Reached;
	}
	int32 Best = 0;
	for (int32 LabelIndex = 1; LabelIndex < Labels[Reached].Num(); ++LabelIndex)
	{
		if (Labels[Reached][LabelIndex].Arrive < Labels[Reached][Best].Arrive)
		{
			Best = LabelIndex;
		}
	}
	TArray<FWait> Waits;
	TArray<const AFGTrain*> Causes;
	const bool bAll = Reached == Count - 1;
	if (!bAll && CanWait(Reached))
	{
		Waits.Add({Stops[Reached], OpenTime});
		const FSippInterval* Blocked = Free.IsValidIndex(Reached + 1) && Free[Reached + 1].Num() > 0 ? &Free[Reached + 1][0] : nullptr;
		Causes.Add(Blocked ? Blocked->After : nullptr);
	}
	for (int32 Index = Reached, LabelIndex = Best; Index > 0; --Index)
	{
		const FSippLabel& Label = Labels[Index][LabelIndex];
		const FSippLabel& Prev = Labels[Index - 1][Label.PrevLabel];
		if (Label.PrevDepart > Prev.Arrive + SippMinWaitSeconds)
		{
			Waits.Add({Stops[Index - 1], Label.PrevDepart});
			Causes.Add(Label.Cause);
		}
		LabelIndex = Label.PrevLabel;
	}
	Algo::Reverse(Waits);
	Algo::Reverse(Causes);
	OutWaits = MoveTemp(Waits);
	OutCauses = MoveTemp(Causes);
	return bAll;
}
