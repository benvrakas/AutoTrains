#include "ATTrafficSubsystem.h"
#include "AutoTrains.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "FGTrain.h"
#include "HAL/PlatformTime.h"

namespace
{
	// Bookings on one resource keep this far apart beyond the safety margin, for trains drifting from their plans. [s]
	constexpr double PlannerDriftSeconds = 2.0;
	// A plan adds at most this many waits.
	constexpr int32 PlanMaxWaits = 32;
	// An open-ended time: a docked train's stay, or everything after a wait for one. [s]
	constexpr double OpenTime = 1e12;
	// Shadow mode: a plan's wait this far before the claims' reach holds a train the claims let go; a train the claims
	// hold that the plan has past its stop within GoSeconds is let go. A train's verdict is logged when it changes, and
	// again after ShadowLogSeconds. [cm] [s]
	constexpr float HoldMargin = 3000.f;
	constexpr double GoSeconds = 10.0;
	constexpr double ShadowLogSeconds = 30.0;
	constexpr double ShadowSummarySeconds = 60.0;
	// The traffic subsystem plans this often in shadow mode. [s]
	constexpr double PlannerUpdateSeconds = 1.0;

	FString NameOf(const FATTrafficTrain* State)
	{
		const AFGTrain* Train = State ? State->Train.Get() : nullptr;
		return Train ? Train->GetTrainName().ToString() : TEXT("?");
	}

	FString ResourceName(const AFGBuildableRailroadTrack* Track, int32 Joint)
	{
		return Joint != INDEX_NONE ? FString::Printf(TEXT("switch %d"), Joint)
			: GetNameSafe(Track).Replace(TEXT("Build_RailroadTrack_C_"), TEXT("track "));
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

		// Ends the motion at End: stopped there for good (the goal), or carrying on at cruise.
		void Finish(float End, bool bStop)
		{
			AddLeg(FMath::Max(End, NextStart), bStop ? 0.f : Cruise);
			bStopsAtEnd = bStop;
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
			if (bStopsAtEnd)
			{
				return OpenTime;
			}
			return Last.Time + Last.TimeTo(Last.End - Last.Start) + (At - Last.End) / Cruise;
		}

		bool IsOpenAfter(double Time) const { return Time >= OpenTime * 0.5; }

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
		TArray<FLeg> Legs;
	};
}

FString AATTrafficSubsystem::SetPlannerShadow(bool bEnabled)
{
	mPlannerShadow = bEnabled;
	mPlannerTimer = 0.f;
	mBookings.Reset();
	mShadow.Reset();
	mShadowHolds = mShadowGos = mShadowFailures = mShadowCommittedConflicts = mShadowPlans = 0;
	mShadowSeconds = 0.0;
	mShadowSummaryAt = GetWorld() ? GetWorld()->GetTimeSeconds() + ShadowSummarySeconds : 0.0;
	const FString Message = bEnabled ? TEXT("Planner shadow mode on: plans are logged, the claim system still drives.") : TEXT("Planner shadow mode off.");
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
	return Message;
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
				Pass.EnterAt = Begin - GetFoulDistance(Entry);
				Pass.ClearAt = Begin + State.Length + GetJointFoulDistance(Joint);
				if (Pass.EnterAt <= 0.f && !State.OccupiedJoints.Contains(Joint))
				{
					// The front is in the zone but the train holds it only by its claim: entering now.
					Pass.EnterAt = 0.f;
				}
				Out.Add(Pass);
				JointsOnRoute.Add(Joint);
			}
		}
		FPass Pass;
		Pass.Key.Track = Step.Track;
		Pass.EnterAt = Index == 0 ? -1.f : Begin;
		Pass.ClearAt = End + State.Length;
		Out.Add(Pass);
		OnRoute.Add(Step.Track);
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
		Pass.EnterAt = -1.f;
		const float Clear = GetTailClearDistance(State, nullptr, Joint);
		Pass.ClearAt = Clear >= 0.f ? Clear : State.Length + GetJointFoulDistance(Joint);
		Out.Add(Pass);
	}
	Out.StableSort([](const FPass& A, const FPass& B) { return A.EnterAt < B.EnterAt; });
}

void AATTrafficSubsystem::TimePasses(const FATTrafficTrain& State, const FRouteSpeed& Speed, const TArray<FWait>& Waits, double Now, TArray<FPass>& Passes) const
{
	FProfile Profile(State.Speed, Now, Speed.Cruise, Speed.Accel, Speed.Brake);
	for (const FWait& Wait : Waits)
	{
		Profile.AddStop(Wait.At, Wait.Until);
	}
	const float End = State.Route.Num() > 0 ? State.Route.Last().EndDistance - State.FrontAhead : 0.f;
	// A route ends at the goal, where the train docks and stays for a time nobody knows.
	Profile.Finish(End, true);
	for (FPass& Pass : Passes)
	{
		Pass.Enter = Pass.EnterAt <= 0.f ? Now : Profile.TimeAt(Pass.EnterAt);
		Pass.Clear = Profile.TimeAt(Pass.ClearAt);
	}
}

const AATTrafficSubsystem::FBooking* AATTrafficSubsystem::FindConflict(const FATTrafficTrain& State, const FPass& Pass, double Headway) const
{
	const FBooking* Found = nullptr;
	auto Check = [&](const FResourceKey& Key)
	{
		const TArray<FBooking>* Bookings = mBookings.Find(Key);
		if (!Bookings)
		{
			return;
		}
		for (const FBooking& Booking : *Bookings)
		{
			if (Booking.Train == &State)
			{
				continue;
			}
			if (Pass.Enter < Booking.Clear + Headway && Pass.Clear + Headway > Booking.Enter && (!Found || Booking.Clear > Found->Clear))
			{
				Found = &Booking;
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

void AATTrafficSubsystem::PlanTrain(const FATTrafficTrain& State, double Now, FPlan& Plan) const
{
	const FRouteSpeed Speed = GetRouteSpeed(State);
	const double Headway = GetSafetyMarginSeconds() + PlannerDriftSeconds;
	Plan = FPlan();
	Plan.Committed = GetStoppingDistance(State, State.Speed);
	TArray<FPass> Base;
	CollectPasses(State, Base);
	// Where the train can stand: the end of each route track, short by its stop margin.
	TArray<float> StopPoints;
	for (const FATRouteStep& Step : State.Route)
	{
		StopPoints.Add(Step.EndDistance - State.FrontAhead - GetStopMargin(Step.Exit));
	}
	TArray<const FATTrafficTrain*> WaitCauses;
	for (int32 Attempt = 0; Attempt <= PlanMaxWaits; ++Attempt)
	{
		TArray<FPass> Passes = Base;
		TimePasses(State, Speed, Plan.Waits, Now, Passes);
		const FPass* Conflicting = nullptr;
		const FBooking* Booking = nullptr;
		for (const FPass& Pass : Passes)
		{
			// Nothing is planned beyond a wait for good; what the train is committed to can't change.
			if (Pass.Enter >= OpenTime * 0.5 || Pass.EnterAt <= Plan.Committed)
			{
				continue;
			}
			Booking = FindConflict(State, Pass, Headway);
			if (Booking)
			{
				Conflicting = &Pass;
				break;
			}
		}
		if (!Conflicting)
		{
			Plan.Passes = MoveTemp(Passes);
			Plan.WaitFor = WaitCauses.Num() > 0 ? WaitCauses[0] : nullptr;
			return;
		}
		// Wait at the last place to stand before the resource so the train enters it once the booking has cleared.
		int32 Best = INDEX_NONE;
		for (int32 Index = 0; Index < StopPoints.Num(); ++Index)
		{
			if (StopPoints[Index] >= Plan.Committed && StopPoints[Index] < Conflicting->EnterAt)
			{
				Best = Index;
			}
		}
		if (Best == INDEX_NONE)
		{
			Plan.Failure = FString::Printf(TEXT("no place to wait before %s for %s"), *ResourceName(Conflicting->Key.Track, Conflicting->Key.Joint),
				*NameOf(Booking->Train));
			Plan.Passes = MoveTemp(Passes);
			Plan.WaitFor = Booking->Train;
			return;
		}
		const float At = StopPoints[Best];
		const double Need = Booking->Clear >= OpenTime * 0.5 ? OpenTime : Booking->Clear + Headway;
		double Until = Need >= OpenTime ? OpenTime : Need - RestTime(Conflicting->EnterAt - At, Speed.Cruise, Speed.Accel);
		const int32 Existing = Plan.Waits.IndexOfByPredicate([At](const FWait& Wait) { return FMath::IsNearlyEqual(Wait.At, At, 1.f); });
		if (Existing != INDEX_NONE)
		{
			// Still in conflict after waiting there: wait longer.
			Until = FMath::Max(Until, Plan.Waits[Existing].Until + 1.0);
			Plan.Waits[Existing].Until = Until;
			WaitCauses[Existing] = Booking->Train;
		}
		else
		{
			int32 Insert = 0;
			while (Insert < Plan.Waits.Num() && Plan.Waits[Insert].At < At)
			{
				++Insert;
			}
			Plan.Waits.Insert({At, Until}, Insert);
			WaitCauses.Insert(Booking->Train, Insert);
		}
	}
	Plan.Failure = FString::Printf(TEXT("no plan after %d waits"), PlanMaxWaits);
	Plan.WaitFor = WaitCauses.Num() > 0 ? WaitCauses[0] : nullptr;
}

void AATTrafficSubsystem::Book(const FATTrafficTrain& State, const TArray<FPass>& Passes, bool bCommitted)
{
	for (const FPass& Pass : Passes)
	{
		if (Pass.Enter >= OpenTime * 0.5)
		{
			continue;
		}
		FBooking Booking;
		Booking.Train = &State;
		Booking.Enter = Pass.Enter;
		Booking.Clear = Pass.Clear;
		Booking.bCommitted = bCommitted;
		mBookings.FindOrAdd(Pass.Key).Add(Booking);
	}
}

void AATTrafficSubsystem::UpdatePlanner()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}
	const double Begin = FPlatformTime::Seconds();
	const double Now = World->GetTimeSeconds();
	const double Headway = GetSafetyMarginSeconds() + PlannerDriftSeconds;
	mBookings.Reset();

	TArray<const FATTrafficTrain*> Managed;
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const FATTrafficTrain& State = Pair.Value;
		if (!State.Train.IsValid())
		{
			continue;
		}
		if (State.bManaged && State.Route.Num() > 0)
		{
			Managed.Add(&State);
			continue;
		}
		// A train this mod doesn't drive, or a docked one, holds where it stands until it actually leaves.
		TArray<FPass> Standing;
		for (const AFGBuildableRailroadTrack* Track : State.Occupied)
		{
			FPass Pass;
			Pass.Key.Track = Track;
			Pass.Enter = Now;
			Pass.Clear = OpenTime;
			Standing.Add(Pass);
		}
		for (const int32 Joint : State.OccupiedJoints)
		{
			FPass Pass;
			Pass.Key.Joint = Joint;
			Pass.Enter = Now;
			Pass.Clear = OpenTime;
			Standing.Add(Pass);
		}
		Book(State, Standing, true);
	}

	// Each moving train's committed stretch, at its motion now: it can't stop short of it whatever else wants it.
	TMap<const FATTrafficTrain*, TArray<FPass>> CommittedPasses;
	for (const FATTrafficTrain* State : Managed)
	{
		TArray<FPass> Passes;
		CollectPasses(*State, Passes);
		TimePasses(*State, GetRouteSpeed(*State), TArray<FWait>(), Now, Passes);
		const float Committed = GetStoppingDistance(*State, State->Speed);
		Passes.RemoveAll([Committed](const FPass& Pass) { return Pass.EnterAt > Committed; });
		CommittedPasses.Add(State, Passes);
	}
	for (const FATTrafficTrain* State : Managed)
	{
		// Two committed stretches meeting on a resource: a collision course the plan can't avoid any more.
		for (const FPass& Pass : CommittedPasses[State])
		{
			const FBooking* Other = FindConflict(*State, Pass, 0.0);
			if (Other && Other->bCommitted && Other->Train && Other->Train->bManaged)
			{
				++mShadowCommittedConflicts;
				FShadowState& Shadow = mShadow.FindOrAdd(State->Train);
				const FString Verdict = FString::Printf(TEXT("committed with %s"), *NameOf(Other->Train));
				if (Shadow.Verdict != Verdict || Now - Shadow.LoggedAt > ShadowLogSeconds)
				{
					UE_LOG(LogAutoTrains, Log, TEXT("planner: %s and %s are both committed to %s (%.1f-%.1f s and %.1f-%.1f s from now)"), *NameOf(State),
						*NameOf(Other->Train), *ResourceName(Pass.Key.Track, Pass.Key.Joint), Pass.Enter - Now, FMath::Min(Pass.Clear - Now, 9999.0),
						Other->Enter - Now, FMath::Min(Other->Clear - Now, 9999.0));
					Shadow.Verdict = Verdict;
					Shadow.LoggedAt = Now;
				}
				break;
			}
		}
		Book(*State, CommittedPasses[State], true);
	}

	// Plans in priority order, each around the bookings before it.
	TMap<const FATTrafficTrain*, float> Priority;
	for (const FATTrafficTrain* State : Managed)
	{
		Priority.Add(State, GetClaimPriority(*State));
	}
	Managed.Sort([&Priority](const FATTrafficTrain& A, const FATTrafficTrain& B) { return Priority.FindRef(&A) < Priority.FindRef(&B); });
	TMap<const FATTrafficTrain*, const FATTrafficTrain*> WaitsFor;
	for (const FATTrafficTrain* State : Managed)
	{
		FPlan Plan;
		PlanTrain(*State, Now, Plan);
		Book(*State, Plan.Passes, false);
		++mShadowPlans;
		if (Plan.WaitFor)
		{
			WaitsFor.Add(State, Plan.WaitFor);
		}
		LogShadow(*State, Plan, Now);
	}

	// A cycle of first waits: every train in it waits for the next.
	TSet<const FATTrafficTrain*> Reported;
	for (const TPair<const FATTrafficTrain*, const FATTrafficTrain*>& Pair : WaitsFor)
	{
		TArray<const FATTrafficTrain*> Chain;
		const FATTrafficTrain* Current = Pair.Key;
		while (Current && !Chain.Contains(Current) && Chain.Num() < 64)
		{
			Chain.Add(Current);
			const FATTrafficTrain* const* Next = WaitsFor.Find(Current);
			Current = Next ? *Next : nullptr;
		}
		if (Current != Pair.Key || Reported.Contains(Pair.Key))
		{
			continue;
		}
		TArray<FString> Names;
		for (const FATTrafficTrain* Member : Chain)
		{
			Names.Add(NameOf(Member));
			Reported.Add(Member);
		}
		UE_LOG(LogAutoTrains, Warning, TEXT("planner: waiting cycle %s -> %s"), *FString::Join(Names, TEXT(" -> ")), *Names[0]);
	}

	mShadowSeconds += FPlatformTime::Seconds() - Begin;
	if (Now >= mShadowSummaryAt)
	{
		UE_LOG(LogAutoTrains, Log, TEXT("planner (shadow): %d plans in %.0f s, %.2f ms per update; train-seconds where the plan would hold a train the claims let go %d, let go one the claims hold %d; failures %d, committed conflicts %d"),
			mShadowPlans, ShadowSummarySeconds, mShadowSeconds * 1000.0 / (ShadowSummarySeconds / PlannerUpdateSeconds), mShadowHolds, mShadowGos,
			mShadowFailures, mShadowCommittedConflicts);
		mShadowPlans = mShadowHolds = mShadowGos = mShadowFailures = mShadowCommittedConflicts = 0;
		mShadowSeconds = 0.0;
		mShadowSummaryAt = Now + ShadowSummarySeconds;
	}
}

void AATTrafficSubsystem::LogShadow(const FATTrafficTrain& State, const FPlan& Plan, double Now)
{
	const AFGTrain* Train = State.Train.Get();
	if (!Train)
	{
		return;
	}
	const float Authority = GetAuthorityDistance(Train);
	const FWait* FirstWait = Plan.Waits.FindByPredicate([Now](const FWait& Wait) { return Wait.Until > Now + 1.0; });
	FString Verdict;
	FString Detail;
	if (!Plan.Failure.IsEmpty())
	{
		Verdict = TEXT("fails");
		Detail = Plan.Failure;
		++mShadowFailures;
	}
	else if (FirstWait && (Authority < 0.f || FirstWait->At < Authority - HoldMargin))
	{
		Verdict = FString::Printf(TEXT("hold for %s"), *NameOf(Plan.WaitFor));
		Detail = FString::Printf(TEXT("the claims let it go %s, the plan waits at %.0f m for %s, %s"),
			Authority < 0.f ? TEXT("to its goal") : *FString::Printf(TEXT("to %.0f m"), Authority / 100.f), FirstWait->At / 100.f, *NameOf(Plan.WaitFor),
			FirstWait->Until >= OpenTime * 0.5 ? TEXT("until it leaves") : *FString::Printf(TEXT("%.0f s"), FirstWait->Until - Now));
		++mShadowHolds;
	}
	else if (State.BlockedBy.IsValid() && Authority >= 0.f && State.Speed < 100.f)
	{
		// Held by the claims: does the plan have the train past that point soon?
		TArray<FPass> Probe;
		FPass Pass;
		Pass.EnterAt = Authority + HoldMargin;
		Pass.ClearAt = Pass.EnterAt;
		Probe.Add(Pass);
		TimePasses(State, GetRouteSpeed(State), Plan.Waits, Now, Probe);
		if (Probe[0].Enter - Now < GoSeconds && State.Route.Num() > 0 && State.Route.Last().EndDistance - State.FrontAhead > Pass.EnterAt)
		{
			Verdict = TEXT("go");
			Detail = FString::Printf(TEXT("the claims hold it at %.0f m for %s (%s), the plan has it past there in %.0f s"), Authority / 100.f,
				State.BlockedBy.IsValid() ? *State.BlockedBy->GetTrainName().ToString() : TEXT("?"), *State.BlockedReason, Probe[0].Enter - Now);
			++mShadowGos;
		}
	}
	FShadowState& Shadow = mShadow.FindOrAdd(State.Train);
	if (Verdict != Shadow.Verdict || (!Verdict.IsEmpty() && Now - Shadow.LoggedAt > ShadowLogSeconds))
	{
		if (!Verdict.IsEmpty())
		{
			UE_LOG(LogAutoTrains, Log, TEXT("planner: %s: %s"), *NameOf(&State), *Detail);
		}
		Shadow.Verdict = Verdict;
		Shadow.LoggedAt = Now;
	}
}
