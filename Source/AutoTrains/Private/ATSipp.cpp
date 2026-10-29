#include "ATSipp.h"
#include "Algo/Reverse.h"
#include "Async/Async.h"
#include "Async/ParallelFor.h"
#include "HAL/PlatformTime.h"

void FATSippTable::Init(int32 Resources, int32 Owners)
{
	ByResource.Reset();
	ByResource.SetNum(Resources);
	OwnerResources.Reset();
	OwnerResources.SetNum(Owners);
}

void FATSippTable::Add(int32 Resource, const FATSippBooking& Booking)
{
	if (ByResource.IsValidIndex(Resource) && OwnerResources.IsValidIndex(Booking.Owner))
	{
		ByResource[Resource].Add(Booking);
		OwnerResources[Booking.Owner].Add(Resource);
	}
}

void FATSippTable::Remove(int32 Owner)
{
	if (!OwnerResources.IsValidIndex(Owner))
	{
		return;
	}
	for (const int32 Resource : OwnerResources[Owner])
	{
		ByResource[Resource].RemoveAll([Owner](const FATSippBooking& Booking) { return Booking.Owner == Owner; });
	}
	OwnerResources[Owner].Reset();
}

namespace
{
	// Search size limit per plan (some MB at most).
	constexpr int32 MaxStates = 200000;
	// A train standing at a track end tries at most MaxWindows windows of the way on, each found within MaxShifts
	// moves of its departure.
	constexpr int32 MaxWindows = 4;
	constexpr int32 MaxShifts = 24;
	// A wait shorter than this isn't one. [s]
	constexpr double MinWaitSeconds = 0.5;
	// A move shorter than this is none, and a train slower than this at a track end stops there instead. [cm] [cm/s]
	constexpr float MinMove = 1.f;
	constexpr float MinPassSpeed = 50.f;
	// Speeds a train may pass a track end at besides the fastest and slowest it can reach there, as shares of its
	// cruising speed, and the number of speed bands its states there are told apart by.
	constexpr float PassShares[] = {0.25f, 0.5f, 0.75f};
	// Gravity, and the least speeding up and braking a train plans with on a slope. [cm/s^2]
	constexpr float Gravity = 980.f;
	constexpr float MinSlopeAccel = 5.f;
	constexpr float MinSlopeBrake = 20.f;
	constexpr float SpeedBands = 32.f;

	bool IsNever(double Time)
	{
		return Time >= ATSippNever * 0.5;
	}

	// Time from standstill over Distance, speeding up at Accel to Top. [s]
	double RestTime(float Distance, float Top, float Accel)
	{
		if (Distance <= 0.f)
		{
			return 0.0;
		}
		const float ToTop = FMath::Square(Top) / (2.f * Accel);
		return Distance <= ToTop ? FMath::Sqrt(2.f * Distance / Accel) : Top / Accel + (Distance - ToTop) / Top;
	}

	// The fastest motion over Length from speed From to speed To: speeding up at Accel towards Top, then braking at
	// Brake. [cm] [cm/s] [cm/s^2]
	struct FEdge
	{
		float Length = 0.f;
		float From = 0.f;
		float To = 0.f;
		float Peak = 0.f;
		float SpeedUp = 0.f;
		float SlowDown = 0.f;
		float Accel = 1.f;
		float Brake = 1.f;

		// Returns false if To can't be reached from From over Length; bHarder: braking harder than Brake where it must.
		bool Make(float InLength, float InFrom, float InTo, float Top, float InAccel, float InBrake, bool bHarder)
		{
			Length = InLength;
			From = InFrom;
			To = InTo;
			Accel = InAccel;
			Brake = InBrake;
			const float FromSquared = FMath::Square(From);
			const float ToSquared = FMath::Square(To);
			if (bHarder && Length > 0.f)
			{
				Brake = FMath::Max(Brake, (FromSquared - ToSquared) / (2.f * Length));
			}
			if (ToSquared > FromSquared + 2.f * Accel * Length + 1.f || ToSquared < FromSquared - 2.f * Brake * Length - 1.f)
			{
				return false;
			}
			// The highest speed reachable between speeding up from From and braking to To.
			const float PeakSquared = (Length + FromSquared / (2.f * Accel) + ToSquared / (2.f * Brake)) / (1.f / (2.f * Accel) + 1.f / (2.f * Brake));
			Peak = FMath::Clamp(FMath::Sqrt(FMath::Max(PeakSquared, 0.f)), FMath::Max(From, To), FMath::Max3(Top, From, To));
			SpeedUp = FMath::Max((FMath::Square(Peak) - FromSquared) / (2.f * Accel), 0.f);
			SlowDown = FMath::Max((FMath::Square(Peak) - ToSquared) / (2.f * Brake), 0.f);
			const float Over = SpeedUp + SlowDown - Length;
			if (Over > 0.f)
			{
				SpeedUp = FMath::Max(SpeedUp - Over * 0.5f, 0.f);
				SlowDown = FMath::Max(Length - SpeedUp, 0.f);
			}
			return true;
		}

		// Seconds from the start to Distance along it.
		double TimeTo(float Distance) const
		{
			Distance = FMath::Clamp(Distance, 0.f, Length);
			double Seconds = 0.0;
			const float InSpeedUp = FMath::Min(Distance, SpeedUp);
			Seconds += InSpeedUp > 0.f ? (FMath::Sqrt(FMath::Square(From) + 2.f * Accel * InSpeedUp) - From) / Accel : 0.0;
			if (Distance <= SpeedUp)
			{
				return Seconds;
			}
			const float Cruising = FMath::Max(Length - SpeedUp - SlowDown, 0.f);
			const float InCruise = FMath::Min(Distance - SpeedUp, Cruising);
			Seconds += Peak > 0.f ? InCruise / Peak : 0.0;
			if (Distance <= SpeedUp + Cruising)
			{
				return Seconds;
			}
			const float InSlowDown = Distance - SpeedUp - Cruising;
			const float Speed = FMath::Sqrt(FMath::Max(FMath::Square(Peak) - 2.f * Brake * InSlowDown, 0.f));
			return Seconds + (Peak - Speed) / Brake;
		}
	};

	// What entering a resource at a time commits the train to: its tail clears it no earlier than ClearLo (a train
	// ahead the same way, LoBy) and no later than ClearHi (the next booking, HiBy's, counting only because its train
	// plans first if bHiByRank); Key tells the windows apart.
	struct FGap
	{
		double ClearLo = -ATSippNever;
		double ClearHi = ATSippNever;
		int32 LoBy = INDEX_NONE;
		int32 HiBy = INDEX_NONE;
		bool bHiByRank = false;
		uint32 Key = 0;
	};

	// A resource the train has entered and not yet cleared: where its tail clears it [cm] and within which times, and
	// where it begins [cm].
	struct FOpenResource
	{
		int32 Resource = INDEX_NONE;
		float ClearAt = 0.f;
		FGap Gap;
		float StartAt = 0.f;
	};

	// The front at Pos [cm from where it is now] at Arrive with Speed, at the end of route step Step's track (Node, the
	// connection at NodePos), stopped short of it or passing it; it got there over a run of track starting with
	// connection First, leaving the parent at Leave over Edge, later than it got there for Cause's booking. Opens
	// [OpenFirst, +NumOpen): what the move here started on. BlockedBy/BlockedAt: the last booking that kept a move on from
	// here, and whether it counted only because its train plans first.
	struct FState
	{
		int32 Node = INDEX_NONE;
		int32 Step = -1;
		int32 Parent = INDEX_NONE;
		float NodePos = 0.f;
		float Pos = 0.f;
		float Speed = 0.f;
		double Leave = 0.0;
		double Arrive = 0.0;
		FEdge Edge;
		bool bStopped = false;
		uint32 Key = 0;
		int32 Cause = INDEX_NONE;
		int32 First = INDEX_NONE;
		int32 OpenFirst = 0;
		int32 NumOpen = 0;
		int32 BlockedBy = INDEX_NONE;
		int32 BlockedAt = INDEX_NONE;
		bool bBlockedByRank = false;
	};

	// The connections of a run of track.
	using FRun = TArray<int32, TInlineAllocator<16>>;

	struct FQueued
	{
		double Arrive = 0.0;
		int32 State = INDEX_NONE;
		bool operator<(const FQueued& Other) const { return Arrive < Other.Arrive; }
	};

	// One train's safe-interval search over (track end, speed, window) states with the train's own motion, body and
	// stops, against the bookings of the table in its scope. A move runs from one place anything is decided (a switch,
	// a crossing, a platform, a one-way signal, the goal, the end of the fixed part of the route) to the next; the
	// tracks in between are each booked, but nothing is decided on them.
	class FPlanner
	{
	public:
		FPlanner(const FATSippCycle& InCycle, const FATSippTable& InTable, int32 InSlot)
			: Cycle(InCycle), Graph(*InCycle.Graph), Table(InTable), Train(InCycle.Trains[InSlot]), Slot(InSlot), Headway(InCycle.Headway)
		{
			Accel = FMath::Max(Train.Accel, 1.f);
			Brake = FMath::Max(Train.Brake, 1.f);
			Top = FMath::Max3(Train.Cruise, Train.Speed, MinPassSpeed * 2.f);
			ForcedReach = FMath::Square(Train.Speed) / (2.f * Brake);
		}

		// Plans the train along Route (bLastIsGoal: its last step is the goal), else to its furthest stop there, where it
		// then stands for good.
		void Plan(const TArray<int32>& InRoute, bool bLastIsGoal, FATSippResult& Out)
		{
			const int32 Final = Search(InRoute, false, bLastIsGoal);
			const int32 Stand = Final != INDEX_NONE ? Final : FurthestStopped != INDEX_NONE ? FurthestStopped : FurthestAny;
			Build(Stand, Final != INDEX_NONE, Out);
			Out.States = States.Num();
			Out.RejectedMotion = RejectedMotion;
			Out.RejectedTail = RejectedTail;
			Out.RejectedStay = RejectedStay;
		}

		// The fastest way from where the train is, along its route up to step Commit, then over the whole network to the
		// goal of its route; returns the route found (empty if none) and its arrival.
		TArray<int32> FindRoute(const TArray<int32>& InRoute, double& OutArrival)
		{
			const int32 Final = Search(InRoute, true, true);
			OutArrival = Final != INDEX_NONE ? States[Final].Arrive : ATSippNever;
			return Final != INDEX_NONE ? RouteTo(Final) : TArray<int32>();
		}

		int32 NumStates() const { return States.Num(); }

	private:
		int32 Search(const TArray<int32>& InRoute, bool bInFree, bool bInGoalAtEnd)
		{
			Route = &InRoute;
			bFree = bInFree;
			bGoalAtEnd = bInGoalAtEnd;
			Goal = InRoute.Num() > 0 ? InRoute.Last() : INDEX_NONE;
			States.Reset();
			Opens.Reset();
			Best.Reset();
			ReachedTracks.Reset();
			RejectedMotion = RejectedTail = RejectedStay = 0;
			FState Start;
			Start.bStopped = Train.bStanding;
			Start.Speed = Train.bStanding ? 0.f : Train.Speed;
			Start.Arrive = Cycle.Now;
			Start.Leave = Cycle.Now;
			States.Add(Start);
			FurthestStopped = Train.bStanding ? 0 : INDEX_NONE;
			FurthestAny = 0;
			if (InRoute.Num() == 0)
			{
				return INDEX_NONE;
			}
			TArray<FQueued> Queue;
			Queue.HeapPush({Cycle.Now, 0});
			while (Queue.Num() > 0)
			{
				FQueued Item;
				Queue.HeapPop(Item);
				const FState& State = States[Item.State];
				if (Item.State != 0)
				{
					const double* Known = Best.Find(KeyOf(State));
					if (Known && *Known < State.Arrive - 1e-6)
					{
						continue;
					}
				}
				Note(Item.State);
				if (State.Step >= 0 && IsGoal(State.Step, State.Node))
				{
					if (State.bStopped && StayFits(Item.State))
					{
						return Item.State;
					}
					RejectedStay += State.bStopped ? 1 : 0;
					continue;
				}
				if (States.Num() >= MaxStates)
				{
					break;
				}
				const int32 Before = States.Num();
				Expand(Item.State);
				for (int32 Index = Before; Index < States.Num(); ++Index)
				{
					const FState& Reached = States[Index];
					double& Stored = Best.FindOrAdd(KeyOf(Reached), ATSippNever);
					if (Reached.Arrive < Stored)
					{
						Stored = Reached.Arrive;
						Queue.HeapPush({Reached.Arrive, Index});
					}
				}
			}
			return INDEX_NONE;
		}

		// Keeps the furthest stop reached (and the furthest state of any kind) for a plan that can't reach the goal.
		void Note(int32 Index)
		{
			const FState& State = States[Index];
			auto Better = [&](int32 Known) { return Known == INDEX_NONE || State.Step > States[Known].Step || (State.Step == States[Known].Step && State.Arrive < States[Known].Arrive); };
			if (State.bStopped && Better(FurthestStopped))
			{
				FurthestStopped = Index;
			}
			if (Better(FurthestAny))
			{
				FurthestAny = Index;
			}
		}

		bool IsGoal(int32 Step, int32 Node) const
		{
			return bFree ? Step > Train.Commit && Node == Goal : bGoalAtEnd && Step == Route->Num() - 1;
		}

		uint64 KeyOf(const FState& State) const
		{
			const int32 Where = bFree ? State.Node : State.Step;
			const uint32 Band = State.bStopped ? 0u : 1u + static_cast<uint32>(FMath::Min(State.Speed / Top * SpeedBands, SpeedBands));
			return (static_cast<uint64>(static_cast<uint32>(Where)) << 32) | HashCombine(State.Key, Band);
		}

		bool InScope(const FATSippBooking& Booking, bool bFirst) const
		{
			if (bFirst || Booking.Kind != EATSippKind::Moving || Booking.bHeld || Booking.bCommitted)
			{
				return true;
			}
			return Train.bRanked && !PlansAfter(Booking.Owner);
		}

		bool PlansAfter(int32 Owner) const
		{
			const int32 Rank = Cycle.Trains.IsValidIndex(Owner) ? Cycle.Trains[Owner].Rank : INDEX_NONE;
			return Rank != INDEX_NONE && Train.Rank != INDEX_NONE && Rank > Train.Rank;
		}

		bool IsBehind(int32 Owner, int32 Resource) const
		{
			const TSet<int32>* Resources = Train.Behind.Find(Owner);
			return Resources && Resources->Contains(Resource);
		}

		// The resources Resource conflicts with: a track's crossing tracks (itself first), or the joint itself.
		template <typename TFunc>
		void ForEachConflict(int32 Resource, TFunc&& Func) const
		{
			if (Resource < Graph.Length.Num())
			{
				for (const int32 Conflict : Graph.Conflicts[Resource])
				{
					Func(Conflict);
				}
			}
			else
			{
				Func(Resource);
			}
		}

		// Whether the train may enter Resource (leaving its track through Exit, coming from Via) at Enter: no booking in
		// its scope there or on a conflicting resource is in the way, and a train the same way is a headway ahead or
		// behind at both ends. Sets the gap it then has to clear the resource in, else the earliest later entry that might
		// work, the train in the way and whether its booking counts only because that train plans first.
		bool GapAt(int32 Resource, int32 Exit, int32 Via, double Enter, FGap& Out, double& OutRetry, int32& OutBy, bool& bOutByRank) const
		{
			Out = FGap();
			OutRetry = Enter;
			OutBy = INDEX_NONE;
			bOutByRank = false;
			double After = -ATSippNever;
			bool bClash = false;
			const bool bHeld = Train.Held.Contains(Resource);
			ForEachConflict(Resource, [&](int32 Checked)
			{
				for (const FATSippBooking& Booking : Table.ByResource[Checked])
				{
					if (Booking.Owner == Slot || IsBehind(Booking.Owner, Checked))
					{
						continue;
					}
					const bool bFirst = Booking.bLocked || Train.First.Contains(Booking.Owner);
					// Held from its last grant: kept unless a train that goes first needs it.
					if ((bHeld && !bFirst) || !InScope(Booking, bFirst))
					{
						continue;
					}
					const bool bOpen = IsNever(Booking.Clear);
					const bool bByRank = !bFirst && Booking.Kind == EATSippKind::Moving && !Booking.bHeld && !Booking.bCommitted;
					auto LowerHi = [&](double Hi)
					{
						if (Hi < Out.ClearHi)
						{
							Out.ClearHi = Hi;
							Out.HiBy = Booking.Owner;
							Out.bHiByRank = bByRank;
						}
					};
					if (Checked == Resource && Exit != INDEX_NONE && Booking.Exit == Exit && Booking.Via == Via)
					{
						// The same way: behind it, or ahead of it at both ends unless it goes first.
						if (Enter >= Booking.Enter + Headway)
						{
							After = FMath::Max(After, Booking.Enter + Headway);
							const double Lo = bOpen ? ATSippNever : Booking.Clear + Headway;
							if (Lo > Out.ClearLo)
							{
								Out.ClearLo = Lo;
								Out.LoBy = Booking.Owner;
							}
						}
						else if (!bFirst && Enter <= Booking.Enter - Headway)
						{
							LowerHi(bOpen ? ATSippNever : Booking.Clear - Headway);
						}
						else
						{
							bClash = true;
							OutRetry = FMath::Max(OutRetry, Booking.Enter + Headway);
							OutBy = Booking.Owner;
							bOutByRank = bByRank;
						}
						continue;
					}
					const double Lo = bFirst ? -ATSippNever : Booking.Enter - Headway;
					const double Hi = bOpen ? ATSippNever : Booking.Clear + Headway;
					if (Enter >= Hi)
					{
						After = FMath::Max(After, Hi);
					}
					else if (Enter < Lo)
					{
						LowerHi(Lo);
					}
					else
					{
						bClash = true;
						OutRetry = FMath::Max(OutRetry, Hi);
						OutBy = Booking.Owner;
						bOutByRank = bByRank;
					}
				}
			});
			if (bClash)
			{
				return false;
			}
			Out.Key = HashCombine(HashCombine(GetTypeHash(After), GetTypeHash(Out.ClearHi)), GetTypeHash(Out.ClearLo));
			return true;
		}

		// When the front passes Pos on the way to state Index (at a stop, when it got there). [world s]
		double TimeAtState(int32 Index, float Pos) const
		{
			for (int32 At = Index; At != INDEX_NONE && States[At].Parent != INDEX_NONE; At = States[At].Parent)
			{
				const FState& State = States[At];
				const float From = States[State.Parent].Pos;
				if (Pos > From && Pos <= State.Pos)
				{
					return State.Leave + State.Edge.TimeTo(Pos - From);
				}
				if (Pos > State.Pos)
				{
					return State.Arrive;
				}
			}
			return Cycle.Now;
		}

		// Calls Func for every resource the train has entered by state Index and whose tail-clear point is past MinPos.
		template <typename TFunc>
		void ForEachOpen(int32 Index, float MinPos, TFunc&& Func) const
		{
			const float Reach = Train.Length + Graph.MaxJointFoul;
			for (int32 At = Index; At != INDEX_NONE; At = States[At].Parent)
			{
				const FState& State = States[At];
				if (State.NodePos + Reach < MinPos && State.Step >= 0)
				{
					break;
				}
				for (int32 Open = State.OpenFirst; Open < State.OpenFirst + State.NumOpen; ++Open)
				{
					if (Opens[Open].ClearAt > MinPos)
					{
						Func(Opens[Open]);
					}
				}
			}
		}

		bool OnPath(int32 Index, int32 Track) const
		{
			for (int32 At = Index; At != INDEX_NONE && States[At].Step >= 0; At = States[At].Parent)
			{
				if (Graph.TrackOf[States[At].Node] == Track)
				{
					return true;
				}
			}
			return false;
		}

		// Whether the way on past connection Node ends without a switch or is closed by a one-way signal: a train stopping
		// there leaves backwards.
		bool IsDeadEnd(int32 Node) const
		{
			for (int32 Count = 0; Node != INDEX_NONE && Count < Graph.Length.Num(); ++Count)
			{
				if (Graph.OneWay[Node] || Graph.Attached[Node].Num() == 0)
				{
					return true;
				}
				if (Graph.Attached[Node].Num() > 1 || Graph.JointOf[Node] != INDEX_NONE)
				{
					return false;
				}
				Node = Graph.OppositeOf[Graph.Attached[Node][0]];
			}
			return Node == INDEX_NONE;
		}

		// Where the last switch the way to state Index passes begins [cm]; lowest if it passes none.
		float LastSwitchAt(int32 Index) const
		{
			float At = -MAX_flt;
			for (int32 Link = Index; Link != INDEX_NONE; Link = States[Link].Parent)
			{
				const FState& State = States[Link];
				for (int32 Open = State.OpenFirst; Open < State.OpenFirst + State.NumOpen; ++Open)
				{
					if (Opens[Open].Resource >= Graph.Length.Num() && Opens[Open].StartAt < States[Index].Pos)
					{
						At = FMath::Max(At, Opens[Open].StartAt);
					}
				}
			}
			return At;
		}

		// At the goal: everything under the body stays free through the stay and while it pulls out; at a dead end it
		// backs out, so everything from the last switch on its way stays free until it has backed out past that. Else the
		// booking in the way is noted.
		bool StayFits(int32 Index)
		{
			const FState State = States[Index];
			const bool bDeadEnd = IsDeadEnd(State.Node);
			const float SwitchAt = bDeadEnd ? LastSwitchAt(Index) : MAX_flt;
			bool bFits = true;
			ForEachOpen(Index, FMath::Min(State.Pos, SwitchAt), [&](const FOpenResource& Open)
			{
				if (Open.ClearAt <= State.Pos && Open.StartAt < SwitchAt)
				{
					return;
				}
				const float Out = bDeadEnd ? FMath::Max(State.Pos - Open.StartAt, 0.f) : Open.ClearAt - State.Pos;
				const double Clear = IsNever(Train.GoalDwell) ? ATSippNever : State.Arrive + Train.GoalDwell + RestTime(Out, Top, Accel);
				if (bFits && Clear > Open.Gap.ClearHi)
				{
					NoteBlock(Index, Open.Gap.HiBy, Open.Resource, Open.Gap.bHiByRank);
				}
				bFits = bFits && Clear <= Open.Gap.ClearHi && Clear >= Open.Gap.ClearLo;
			});
			return bFits;
		}

		void Expand(int32 Index)
		{
			const FState State = States[Index];
			TArray<int32, TInlineAllocator<8>> Nexts;
			if (!bFree || State.Step < Train.Commit)
			{
				if (Route->IsValidIndex(State.Step + 1))
				{
					Nexts.Add((*Route)[State.Step + 1]);
				}
			}
			else if (!Graph.OneWay[State.Node])
			{
				for (const int32 Entry : Graph.Attached[State.Node])
				{
					if (Graph.OppositeOf.IsValidIndex(Entry) && Graph.OppositeOf[Entry] != INDEX_NONE)
					{
						Nexts.Add(Graph.OppositeOf[Entry]);
					}
				}
			}
			FRun Run;
			for (const int32 Next : Nexts)
			{
				RunFrom(State, Next, Run);
				const int32 End = Run.Last();
				const int32 Track = Graph.TrackOf[End];
				if (bFree && State.Step >= Train.Commit && ReachedTracks.Contains(Track) && OnPath(Index, Track))
				{
					continue;
				}
				ReachedTracks.Add(Track);
				float NodePos = State.Step < 0 ? FMath::Max(Train.FirstEnd, State.Pos + MinMove) : State.NodePos + Graph.Length[Graph.TrackOf[Run[0]]];
				for (int32 Link = 1; Link < Run.Num(); ++Link)
				{
					NodePos += Graph.Length[Graph.TrackOf[Run[Link]]];
				}
				const bool bGoalNode = IsGoal(State.Step + Run.Num(), End);
				const float SpeedSquared = FMath::Square(State.Speed);
				// Gravity on the body over the move: uphill slows speeding up and helps braking, downhill the other way.
				const float Grade = BodyGrade(Index, Run, NodePos);
				const float RunAccel = FMath::Max(Accel - Gravity * Grade, MinSlopeAccel);
				const float RunBrake = FMath::Max(Brake + Gravity * Grade, MinSlopeBrake);
				const float StopAt = NodePos - (bGoalNode ? 0.f : Graph.StopMarginOf[End]);
				if (StopAt - State.Pos >= MinMove && (StopAt <= ForcedReach || SpeedSquared <= 2.f * RunBrake * (StopAt - State.Pos) + 1.f))
				{
					Move(Index, Run, NodePos, StopAt, 0.f, true, RunAccel, RunBrake);
				}
				const float Pass = NodePos - State.Pos;
				if (bGoalNode || Pass < MinMove)
				{
					continue;
				}
				const float Fastest = FMath::Min(Top, FMath::Sqrt(SpeedSquared + 2.f * RunAccel * Pass));
				const float Slowest = FMath::Max(FMath::Sqrt(FMath::Max(SpeedSquared - 2.f * RunBrake * Pass, 0.f)), MinPassSpeed);
				TArray<float, TInlineAllocator<6>> Speeds;
				// However slow: a train standing at a track end gets past it only creeping.
				if (Fastest > 0.f)
				{
					Speeds.Add(Fastest);
				}
				for (const float Share : PassShares)
				{
					const float Speed = Train.Cruise * Share;
					if (Speed > Slowest + 1.f && Speed < Fastest - 1.f && Speed >= MinPassSpeed)
					{
						Speeds.Add(Speed);
					}
				}
				if (Slowest >= MinPassSpeed && Slowest < Fastest - 1.f)
				{
					Speeds.Add(Slowest);
				}
				for (const float Speed : Speeds)
				{
					Move(Index, Run, NodePos, NodePos, Speed, false, RunAccel, RunBrake);
				}
			}
		}

		// The connections of the run of track from state State's track end on through First, up to the next place anything
		// is decided.
		void RunFrom(const FState& State, int32 First, FRun& Out) const
		{
			Out.Reset();
			Out.Add(First);
			while (Out.Num() <= Graph.Length.Num())
			{
				const int32 Step = State.Step + Out.Num();
				const int32 Node = Out.Last();
				if (IsGoal(Step, Node) || (bFree && Step == Train.Commit))
				{
					return;
				}
				const int32 Through = Graph.Through[Node];
				if (!bFree || Step < Train.Commit)
				{
					if (!Route->IsValidIndex(Step + 1) || Through != (*Route)[Step + 1])
					{
						return;
					}
				}
				else if (Through == INDEX_NONE)
				{
					return;
				}
				Out.Add(Through);
			}
		}


		// Mean slope under the body over the move from state Index over Run to NodePos: half the front's rise and half the
		// tail's over the distance, from the heights of the track ends on the way and of the train's cars at the start.
		// [cm/cm]
		float BodyGrade(int32 Index, const FRun& Run, float NodePos) const
		{
			const float From = States[Index].Pos;
			const float Distance = NodePos - From;
			if (Distance < MinMove)
			{
				return 0.f;
			}
			TArray<FVector2f, TInlineAllocator<64>> Points;
			float Pos = NodePos;
			for (int32 Link = Run.Num() - 1; Link >= 0; --Link)
			{
				Points.Add({Pos, Graph.ZOf[Run[Link]]});
				Pos -= Graph.Length[Graph.TrackOf[Run[Link]]];
			}
			for (int32 At = Index; At != INDEX_NONE && States[At].Step >= 0; At = States[At].Parent)
			{
				Points.Add({States[At].NodePos, Graph.ZOf[States[At].Node]});
				if (States[At].NodePos < From - Train.Length)
				{
					break;
				}
			}
			Points.Add({Train.FrontZAt, Train.FrontZ});
			Points.Add({Train.RearZAt, Train.RearZ});
			Points.Sort([](const FVector2f& A, const FVector2f& B) { return A.X < B.X; });
			auto ZAt = [&Points](float X)
			{
				if (X <= Points[0].X)
				{
					return Points[0].Y;
				}
				for (int32 Point = 1; Point < Points.Num(); ++Point)
				{
					if (X <= Points[Point].X)
					{
						const float Span = Points[Point].X - Points[Point - 1].X;
						return Span > 0.f ? FMath::Lerp(Points[Point - 1].Y, Points[Point].Y, (X - Points[Point - 1].X) / Span) : Points[Point].Y;
					}
				}
				return Points.Last().Y;
			};
			return (ZAt(NodePos) - ZAt(From) + ZAt(NodePos - Train.Length) - ZAt(From - Train.Length)) * 0.5f / Distance;
		}

		void NoteBlock(int32 Index, int32 By, int32 Resource, bool bByRank)
		{
			if (By != INDEX_NONE)
			{
				States[Index].BlockedBy = By;
				States[Index].BlockedAt = Resource;
				States[Index].bBlockedByRank = bByRank;
			}
		}

		// Adds the states of the move from state Index over Run to the end of its last track (at NodePos), arriving at To
		// [cm] with Speed, speeding up at RunAccel and braking at RunBrake [cm/s^2]: leaving at once, or, standing, at the
		// first time it fits in each of the next windows of the way on.
		void Move(int32 Index, const FRun& Run, float NodePos, float To, float Speed, bool bStop, float RunAccel, float RunBrake)
		{
			const FState State = States[Index];
			const float Distance = To - State.Pos;
			FEdge Edge;
			if (!Edge.Make(Distance, State.Speed, Speed, Top, RunAccel, RunBrake, To <= ForcedReach))
			{
				++RejectedMotion;
				return;
			}
			// What the move starts on: the switch joint at the track end it comes from, then each track of the run.
			struct FNew
			{
				int32 Resource;
				int32 Exit;
				int32 Via;
				float EnterAt;
				float ClearAt;
			};
			TArray<FNew, TInlineAllocator<16>> News;
			if (State.Step >= 0)
			{
				const int32 Joint = Graph.JointOf[State.Node];
				if (Joint != INDEX_NONE && !Train.Locked.Contains(Graph.JointResource(Joint)))
				{
					News.Add({Graph.JointResource(Joint), Run[0], State.Node, State.NodePos - Graph.FoulOf[State.Node], State.NodePos + Train.Length + Graph.JointFoul[Joint]});
				}
			}
			float TrackStart = State.Step >= 0 ? State.NodePos : 0.f;
			for (int32 Link = 0; Link < Run.Num(); ++Link)
			{
				const int32 Track = Graph.TrackOf[Run[Link]];
				const float TrackEnd = Link == Run.Num() - 1 ? NodePos : State.Step < 0 && Link == 0 ? FMath::Max(Train.FirstEnd, State.Pos + MinMove) : TrackStart + Graph.Length[Track];
				if (!Train.Locked.Contains(Track))
				{
					News.Add({Track, Run[Link], INDEX_NONE, TrackStart, TrackEnd + Train.Length});
				}
				TrackStart = TrackEnd;
			}
			const bool bCanWait = State.bStopped;
			const double Earliest = State.Step < 0 ? FMath::Max(State.Arrive, Train.MinDepart) : State.Arrive;
			double Leave = Earliest;
			int32 Cause = INDEX_NONE;
			int32 Windows = 0;
			for (int32 Shift = 0; Shift < MaxShifts && Windows < (bCanWait ? MaxWindows : 1); ++Shift)
			{
				double Later = Leave;
				bool bFits = true;
				TArray<FGap, TInlineAllocator<16>> Gaps;
				TArray<double, TInlineAllocator<16>> Enters;
				Gaps.SetNum(News.Num());
				Enters.SetNum(News.Num());
				for (int32 New = 0; New < News.Num(); ++New)
				{
					const bool bOnMove = News[New].EnterAt > State.Pos;
					Enters[New] = bOnMove ? Leave + Edge.TimeTo(News[New].EnterAt - State.Pos) : TimeAtState(Index, News[New].EnterAt);
					double Retry = 0.0;
					int32 By = INDEX_NONE;
					bool bByRank = false;
					if (!GapAt(News[New].Resource, News[New].Exit, News[New].Via, Enters[New], Gaps[New], Retry, By, bByRank))
					{
						NoteBlock(Index, By, News[New].Resource, bByRank);
						// Entered before the stop, or no waiting: no later departure helps.
						if (!bOnMove || !bCanWait || IsNever(Retry))
						{
							return;
						}
						bFits = false;
						Later = FMath::Max(Later, Leave + (Retry - Enters[New]) + 1e-3);
						Cause = By;
					}
				}
				if (bFits)
				{
					// Tails clearing on this move keep to their windows; ones clearing later still can. The booking in the way
					// is noted.
					bool bHopeless = false;
					auto Check = [&](const FOpenResource& Open)
					{
						const double Clear = Open.ClearAt <= To ? Leave + Edge.TimeTo(Open.ClearAt - State.Pos) : Leave + Edge.TimeTo(Distance) + (Open.ClearAt - To) / Top;
						if (Clear > Open.Gap.ClearHi)
						{
							bHopeless = true;
							NoteBlock(Index, Open.Gap.HiBy, Open.Resource, Open.Gap.bHiByRank);
						}
						else if (Open.ClearAt <= To && Clear < Open.Gap.ClearLo)
						{
							bFits = false;
							Later = FMath::Max(Later, Leave + (Open.Gap.ClearLo - Clear) + 1e-3);
							Cause = Open.Gap.LoBy;
							NoteBlock(Index, Open.Gap.LoBy, Open.Resource, false);
						}
					};
					ForEachOpen(Index, State.Pos, Check);
					for (int32 New = 0; New < News.Num(); ++New)
					{
						Check({News[New].Resource, News[New].ClearAt, Gaps[New]});
					}
					if (bHopeless)
					{
						++RejectedTail;
						return;
					}
				}
				if (!bFits)
				{
					if (!bCanWait || Later <= Leave || IsNever(Later))
					{
						RejectedTail += Cause == INDEX_NONE ? 1 : 0;
						return;
					}
					Leave = Later;
					continue;
				}
				FState Next;
				Next.Node = Run.Last();
				Next.First = Run[0];
				Next.Step = State.Step + Run.Num();
				Next.Parent = Index;
				Next.NodePos = NodePos;
				Next.Pos = To;
				Next.Speed = Speed;
				Next.Leave = Leave;
				Next.Arrive = Leave + Edge.TimeTo(Distance);
				Next.Edge = Edge;
				Next.bStopped = bStop;
				Next.Cause = Leave > Earliest + MinWaitSeconds ? Cause : INDEX_NONE;
				Next.OpenFirst = Opens.Num();
				Next.NumOpen = News.Num();
				uint32 Key = 0;
				double Ends = ATSippNever;
				double EndEnter = 0.0;
				for (int32 New = 0; New < News.Num(); ++New)
				{
					const float StartAt = News[New].Resource < Graph.Length.Num() ? News[New].ClearAt - Train.Length - Graph.Length[News[New].Resource] : News[New].EnterAt;
					Opens.Add({News[New].Resource, News[New].ClearAt, Gaps[New], StartAt});
					Key = HashCombine(Key, Gaps[New].Key);
					if (Gaps[New].ClearHi < Ends)
					{
						Ends = Gaps[New].ClearHi;
						EndEnter = Enters[New];
					}
				}
				Next.Key = Key;
				States.Add(Next);
				++Windows;
				if (!bCanWait || IsNever(Ends))
				{
					return;
				}
				// The next window: entering once the booking that ends this one has gone by.
				Leave += Ends - EndEnter + 1e-3;
			}
		}

		// The connections of the way to state Index, every track of each run.
		TArray<int32> RouteTo(int32 Index) const
		{
			TArray<int32> Chain;
			for (int32 At = Index; At != INDEX_NONE && States[At].Step >= 0; At = States[At].Parent)
			{
				Chain.Add(At);
			}
			Algo::Reverse(Chain);
			TArray<int32> Nodes;
			for (const int32 At : Chain)
			{
				const FState& State = States[At];
				int32 Node = State.First;
				Nodes.Add(Node);
				for (int32 Link = States[State.Parent].Step + 2; Link <= State.Step; ++Link)
				{
					Node = Graph.Through[Node];
					Nodes.Add(Node);
				}
			}
			return Nodes;
		}

		// The plan from the way to state Final: its route, passes timed and flagged, waits, and the committed stretch.
		void Build(int32 Final, bool bReached, FATSippResult& Out) const
		{
			Out = FATSippResult();
			Out.bPlanned = true;
			TArray<int32> Chain;
			for (int32 At = Final; At != INDEX_NONE; At = States[At].Parent)
			{
				Chain.Add(At);
			}
			Algo::Reverse(Chain);
			const FState& Last = States[Final];
			auto TimeAt = [&](float Pos) -> double
			{
				if (Pos <= Last.Pos)
				{
					return TimeAtState(Final, Pos);
				}
				// Past the goal: after the stay, pulling away from standstill; past a stand for good: never.
				return bReached && !IsNever(Train.GoalDwell) ? Last.Arrive + Train.GoalDwell + RestTime(Pos - Last.Pos, Top, Accel) : ATSippNever;
			};
			Out.Route = RouteTo(Final);
			// Where the front is at the end of each route step.
			TArray<float> StepEnd;
			for (int32 Step = 0; Step < Out.Route.Num(); ++Step)
			{
				StepEnd.Add(Step == 0 ? FMath::Max(Train.FirstEnd, MinMove) : StepEnd.Last() + Graph.Length[Graph.TrackOf[Out.Route[Step]]]);
			}
			for (const FATSippPass& Tail : Train.Tail)
			{
				FATSippPass Pass = Tail;
				Pass.bLocked = true;
				Out.Passes.Add(Pass);
			}
			for (int32 Step = 0; Step < Out.Route.Num(); ++Step)
			{
				const int32 Node = Out.Route[Step];
				const float From = Step > 0 ? StepEnd[Step - 1] : 0.f;
				if (Step > 0)
				{
					const int32 Previous = Out.Route[Step - 1];
					const int32 Joint = Graph.JointOf[Previous];
					if (Joint != INDEX_NONE)
					{
						FATSippPass Pass;
						Pass.Resource = Graph.JointResource(Joint);
						Pass.Exit = Node;
						Pass.Via = Previous;
						// The front already in the zone: entering now.
						Pass.EnterAt = FMath::Max(From - Graph.FoulOf[Previous], 0.f);
						Pass.ClearAt = From + Train.Length + Graph.JointFoul[Joint];
						Pass.Step = Step;
						Out.Passes.Add(Pass);
					}
				}
				FATSippPass Pass;
				Pass.Resource = Graph.TrackOf[Node];
				Pass.Exit = Node;
				Pass.EnterAt = Step == 0 ? -1.f : From;
				Pass.ClearAt = StepEnd[Step] + Train.Length;
				Pass.Step = Step;
				Out.Passes.Add(Pass);
			}
			for (FATSippPass& Pass : Out.Passes)
			{
				if (Pass.Step != INDEX_NONE)
				{
					Pass.bLocked = Train.Locked.Contains(Pass.Resource);
					Pass.bHeld = !Pass.bLocked && Train.Held.Contains(Pass.Resource);
				}
				Pass.Enter = Pass.EnterAt <= 0.f ? Cycle.Now : TimeAt(Pass.EnterAt);
				Pass.Clear = TimeAt(Pass.ClearAt);
			}
			// At a dead end: held from the last switch on until it has backed out past that.
			if (bReached && !IsNever(Train.GoalDwell) && IsDeadEnd(Last.Node))
			{
				auto StartOf = [&](const FATSippPass& Pass)
				{
					const bool bTrack = Pass.Resource < Graph.Length.Num();
					if (Pass.Step == INDEX_NONE)
					{
						return Pass.ClearAt - Train.Length - (bTrack ? Graph.Length[Pass.Resource] : Graph.JointFoul[Pass.Resource - Graph.Length.Num()]);
					}
					return bTrack && Pass.Step == 0 ? StepEnd[0] - Graph.Length[Pass.Resource] : Pass.EnterAt;
				};
				float SwitchAt = -MAX_flt;
				for (const FATSippPass& Pass : Out.Passes)
				{
					const float Start = StartOf(Pass);
					if (Pass.Resource >= Graph.Length.Num() && Start < Last.Pos)
					{
						SwitchAt = FMath::Max(SwitchAt, Start);
					}
				}
				for (FATSippPass& Pass : Out.Passes)
				{
					const float Start = StartOf(Pass);
					if (Pass.ClearAt > Last.Pos || Start >= SwitchAt)
					{
						Pass.Clear = FMath::Max(Pass.Clear, Last.Arrive + Train.GoalDwell + RestTime(FMath::Max(Last.Pos - Start, 0.f), Top, Accel));
					}
				}
			}
			// Waits: where the plan stands longer than a moment, and for good at the end of a plan that can't reach the goal.
			for (int32 Link = 0; Link < Chain.Num(); ++Link)
			{
				const FState& Here = States[Chain[Link]];
				double Until = -1.0;
				int32 For = INDEX_NONE;
				if (Link + 1 < Chain.Num())
				{
					const FState& Next = States[Chain[Link + 1]];
					if (Next.Leave > Here.Arrive + MinWaitSeconds)
					{
						Until = Next.Leave;
						For = Next.Cause;
					}
				}
				else if (!bReached)
				{
					Until = ATSippNever;
					For = Here.BlockedBy;
				}
				if (Until > Cycle.Now && Out.WaitStep == INDEX_NONE)
				{
					Out.WaitStep = Here.Step;
					Out.WaitUntil = Until;
					Out.WaitFor = For;
				}
			}
			Out.Arrival = bReached ? Last.Arrive : ATSippNever;
			Out.bFailed = !bReached;
			Out.StandStep = Last.Step;
			Out.BlockedBy = Last.BlockedBy;
			Out.BlockedAt = Last.BlockedAt;
			Out.bBlockedByRank = Last.bBlockedByRank;
			MarkCommitted(Chain, Final, Out);
			FindCommittedConflict(Out);
		}

		// Committed: from the furthest pass the train holds or can't stop short of (else from where it is) up to the first
		// place it can stand where its whole body is on nothing a train planned before it has booked from its arrival on.
		void MarkCommitted(const TArray<int32>& Chain, int32 Final, FATSippResult& Out) const
		{
			float From = 0.f;
			for (const FATSippPass& Pass : Out.Passes)
			{
				if (Pass.Step != INDEX_NONE && (Pass.bLocked || Pass.bHeld))
				{
					From = FMath::Max(From, Pass.EnterAt);
				}
			}
			TArray<float, TInlineAllocator<64>> Places;
			if (From <= 0.f)
			{
				Places.Add(0.f);
			}
			for (int32 Link = 1; Link < Chain.Num(); ++Link)
			{
				const FState& State = States[Chain[Link]];
				const bool bGoal = Link == Chain.Num() - 1 && !Out.bFailed;
				const float StopAt = State.NodePos - (bGoal ? 0.f : Graph.StopMarginOf[State.Node]);
				if (StopAt >= From)
				{
					Places.Add(StopAt);
				}
			}
			auto HoldsUp = [&](float At, double Arrive)
			{
				for (const FATSippPass& Pass : Out.Passes)
				{
					if (Pass.EnterAt > At || Pass.ClearAt <= At)
					{
						continue;
					}
					bool bHolds = false;
					ForEachConflict(Pass.Resource, [&](int32 Checked)
					{
						for (const FATSippBooking& Booking : Table.ByResource[Checked])
						{
							// Trains not driven plan nothing, ones planned after this one keep clear of it, and ones the same
							// way queue behind it.
							if (bHolds || Booking.Owner == Slot || Booking.Kind == EATSippKind::Standing || Booking.Clear + Headway <= Arrive
								|| (Booking.Kind == EATSippKind::Moving && PlansAfter(Booking.Owner))
								|| (Checked == Pass.Resource && Pass.Exit != INDEX_NONE && Booking.Exit == Pass.Exit && Booking.Via == Pass.Via)
								|| IsBehind(Booking.Owner, Checked))
							{
								continue;
							}
							bHolds = true;
						}
					});
					if (bHolds)
					{
						return true;
					}
				}
				return false;
			};
			float Reach = From;
			for (const float At : Places)
			{
				Reach = At;
				const double Arrive = At <= 0.f ? Cycle.Now : TimeAtState(Final, At - 1.f);
				if (IsNever(Arrive) || !HoldsUp(At, Arrive))
				{
					break;
				}
			}
			for (FATSippPass& Pass : Out.Passes)
			{
				Pass.bCommitted = !IsNever(Pass.Enter) && Pass.EnterAt <= Reach;
			}
		}

		// A train that goes first booked on something this one can't stop short of any more, at the same time.
		void FindCommittedConflict(FATSippResult& Out) const
		{
			for (const FATSippPass& Pass : Out.Passes)
			{
				if (!Pass.bLocked || Pass.EnterAt <= 0.f || IsNever(Pass.Enter))
				{
					continue;
				}
				ForEachConflict(Pass.Resource, [&](int32 Checked)
				{
					for (const FATSippBooking& Booking : Table.ByResource[Checked])
					{
						if (Out.CommittedWith != INDEX_NONE || Booking.Owner == Slot || IsBehind(Booking.Owner, Checked)
							|| !(Booking.bLocked || Train.First.Contains(Booking.Owner)))
						{
							continue;
						}
						const bool bSameWay = Checked == Pass.Resource && Pass.Exit != INDEX_NONE && Booking.Exit == Pass.Exit && Booking.Via == Pass.Via;
						const bool bClash = bSameWay ? !(Pass.Enter >= Booking.Enter && Pass.Clear >= Booking.Clear) : Pass.Enter < Booking.Clear;
						if (bClash)
						{
							Out.CommittedWith = Booking.Owner;
							Out.CommittedAt = Checked;
						}
					}
				});
				if (Out.CommittedWith != INDEX_NONE)
				{
					return;
				}
			}
		}

		const FATSippCycle& Cycle;
		const FATSippGraph& Graph;
		const FATSippTable& Table;
		const FATSippTrain& Train;
		const int32 Slot;
		const double Headway;
		float Accel = 1.f;
		float Brake = 1.f;
		float Top = 1.f;
		// How far the front gets before service braking from the speed now could stop it: moves ending within it may
		// brake harder, as the train has to. [cm]
		float ForcedReach = 0.f;
		const TArray<int32>* Route = nullptr;
		bool bFree = false;
		bool bGoalAtEnd = true;
		int32 Goal = INDEX_NONE;
		TArray<FState> States;
		TArray<FOpenResource> Opens;
		TMap<uint64, double> Best;
		TSet<int32> ReachedTracks;
		int32 FurthestStopped = INDEX_NONE;
		int32 FurthestAny = 0;
		// Moves left out because the train can't reach the speed, its tail can't keep to a window, or the stay at the
		// goal doesn't fit, for the log.
		int32 RejectedMotion = 0;
		int32 RejectedTail = 0;
		int32 RejectedStay = 0;
	};
}

void ATSipp::Launch(const TSharedRef<FATSippCycle, ESPMode::ThreadSafe>& Cycle)
{
	AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask, [Cycle]()
	{
		Run(*Cycle);
		Cycle->bDone.store(true, std::memory_order_release);
	});
}

void ATSipp::Run(FATSippCycle& Cycle)
{
	const double Begin = FPlatformTime::Seconds();
	const int32 Count = Cycle.Trains.Num();
	Cycle.Results.Reset();
	Cycle.Results.SetNum(Count);
	if (!Cycle.Graph.IsValid())
	{
		return;
	}
	// Route searches first, side by side, against the bookings as the cycle starts; each new route is then planned
	// exactly at its train's turn.
	TArray<int32> Searching;
	for (int32 Slot = 0; Slot < Count; ++Slot)
	{
		const FATSippTrain& Train = Cycle.Trains[Slot];
		if (Train.Turn == EATSippTurn::Plan && Train.bSearch && Train.bGoal && Train.Route.Num() > Train.Commit + 2)
		{
			Searching.Add(Slot);
		}
	}
	TArray<TArray<int32>> Found;
	Found.SetNum(Count);
	TArray<double> FoundArrival;
	FoundArrival.Init(ATSippNever, Count);
	TArray<int32> FoundStates;
	FoundStates.Init(0, Count);
	TArray<double> FoundMilliseconds;
	FoundMilliseconds.Init(0.0, Count);
	ParallelFor(Searching.Num(), [&](int32 Index)
	{
		const int32 Slot = Searching[Index];
		const double Start = FPlatformTime::Seconds();
		FPlanner Planner(Cycle, Cycle.Table, Slot);
		Found[Slot] = Planner.FindRoute(Cycle.Trains[Slot].Route, FoundArrival[Slot]);
		FoundStates[Slot] = Planner.NumStates();
		FoundMilliseconds[Slot] = (FPlatformTime::Seconds() - Start) * 1000.0;
	});

	auto PlanTurn = [&](int32 Slot)
	{
		const FATSippTrain& Train = Cycle.Trains[Slot];
		FATSippResult& Result = Cycle.Results[Slot];
		Result = FATSippResult();
		Cycle.Table.Remove(Slot);
		if (Train.Turn == EATSippTurn::Standing)
		{
			Result.bPlanned = true;
			for (const int32 Resource : Train.Standing)
			{
				FATSippBooking Booking;
				Booking.Owner = Slot;
				Booking.Enter = Cycle.Now;
				Booking.Clear = Train.StandUntil;
				Booking.Kind = Train.Kind;
				Booking.bLocked = true;
				Cycle.Table.Add(Resource, Booking);
			}
			return;
		}
		FPlanner Planner(Cycle, Cycle.Table, Slot);
		Planner.Plan(Train.Route, Train.bGoal, Result);
		const TArray<int32>& Route = Found[Slot];
		if (Route.Num() > 0 && Route != Train.Route)
		{
			// Taken only if clearly faster, or if the route it has leads nowhere.
			const double Current = Result.bFailed ? ATSippNever : Result.Arrival - Cycle.Now;
			const double Gain = IsNever(Current) ? 0.0 : FMath::Max<double>(Train.MinGainSeconds, Current * Train.MinGainPercent / 100.0);
			if (IsNever(Current) || FoundArrival[Slot] - Cycle.Now <= Current - Gain)
			{
				FATSippResult Other;
				Planner.Plan(Route, true, Other);
				if (!Other.bFailed && (IsNever(Current) || Other.Arrival - Cycle.Now <= Current - Gain))
				{
					Other.bRerouted = true;
					Other.OldArrival = Result.Arrival;
					Result = MoveTemp(Other);
				}
			}
		}
		Result.SearchStates = FoundStates[Slot];
		Result.SearchMilliseconds = FoundMilliseconds[Slot];
		for (const FATSippPass& Pass : Result.Passes)
		{
			if (IsNever(Pass.Enter))
			{
				continue;
			}
			FATSippBooking Booking;
			Booking.Owner = Slot;
			Booking.Enter = Pass.Enter;
			Booking.Clear = Pass.Clear;
			Booking.Exit = Pass.Exit;
			Booking.Via = Pass.Via;
			Booking.Kind = Train.Kind;
			Booking.bLocked = Pass.bLocked;
			Booking.bHeld = Pass.bHeld;
			Booking.bCommitted = Pass.bCommitted;
			Cycle.Table.Add(Pass.Resource, Booking);
		}
	};

	// The turns in order; a self-driving train whose plan fails on the booking of one planned before it, which counts
	// only because that one plans first, goes before it and the turns are planned again (paper sec. 6, priority
	// adjustment), up to MaxRetries times and RetrySeconds. A train is never put before one ahead of it on its route, nor
	// back before one that went before it this leg.
	const FATSippTable Initial = Cycle.Table;
	Cycle.Order.Reset();
	for (int32 Slot = 0; Slot < Count; ++Slot)
	{
		Cycle.Order.Add(Slot);
	}
	for (int32 Attempt = 0;; ++Attempt)
	{
		int32 Rank = 0;
		for (const int32 Slot : Cycle.Order)
		{
			FATSippTrain& Train = Cycle.Trains[Slot];
			Train.Rank = Train.Turn == EATSippTurn::Plan && Train.Kind == EATSippKind::Moving ? Rank++ : INDEX_NONE;
		}
		if (Attempt > 0)
		{
			Cycle.Table = Initial;
		}
		for (const int32 Slot : Cycle.Order)
		{
			PlanTurn(Slot);
		}
		int32 Failed = INDEX_NONE;
		int32 Before = INDEX_NONE;
		for (int32 Position = 0; Position < Cycle.Order.Num() && Failed == INDEX_NONE; ++Position)
		{
			const int32 Slot = Cycle.Order[Position];
			const FATSippTrain& Train = Cycle.Trains[Slot];
			const FATSippResult& Result = Cycle.Results[Slot];
			const int32 By = Result.BlockedBy;
			if (Train.Kind != EATSippKind::Moving || !Result.bFailed || !Result.bBlockedByRank || !Cycle.Trains.IsValidIndex(By)
				|| Cycle.Trains[By].Kind != EATSippKind::Moving || Train.First.Contains(By) || Cycle.Trains[By].MovedAheadOf.Contains(Slot))
			{
				continue;
			}
			const int32 ByPosition = Cycle.Order.IndexOfByKey(By);
			bool bAllowed = ByPosition < Position && !Cycle.Reordered.Contains(TPair<int32, int32>(By, Slot));
			for (int32 Between = ByPosition; bAllowed && Between < Position; ++Between)
			{
				bAllowed = !Train.First.Contains(Cycle.Order[Between]);
			}
			if (bAllowed)
			{
				Failed = Slot;
				Before = ByPosition;
			}
		}
		if (Failed == INDEX_NONE)
		{
			break;
		}
		if (Attempt >= Cycle.MaxRetries || FPlatformTime::Seconds() - Begin > Cycle.RetrySeconds)
		{
			Cycle.RetryLimited = Failed;
			break;
		}
		Cycle.Reordered.Add(TPair<int32, int32>(Failed, Cycle.Order[Before]));
		Cycle.Order.Remove(Failed);
		Cycle.Order.Insert(Failed, Before);
	}
	Cycle.Milliseconds = (FPlatformTime::Seconds() - Begin) * 1000.0;
}
