#include "ATRouteSearch.h"
#include "Async/Async.h"
#include "HAL/PlatformTime.h"

namespace
{
	// Search size limit per request (a few MB at most).
	constexpr int32 MaxStates = 50000;
	// How far behind the front a track can still be under the train's body or a switch's fouling zone. [cm]
	constexpr float BodyReachMargin = 5000.f;

	using EWay = FATSearchBlock::EWay;

	// A window of entry times. [world s]
	struct FInterval
	{
		double Lo = 0.0;
		double Hi = 0.0;
	};

	// Entry times a booking rules out, and how it meets the train. [world s]
	struct FForbidden
	{
		double Lo = 0.0;
		double Hi = 0.0;
		const FATSearchBooking* Booking = nullptr;
		EWay Way = EWay::Crossing;
	};

	// The front at the end of Node's track, having entered it at Enter in a window that allowed entering up to Hi, and
	// reaching the end at Arrive [world s]; Along is the track length from the start to there [cm], Step the index of
	// Node in a route searched along (-1 at the start).
	struct FState
	{
		int32 Node = INDEX_NONE;
		int32 Window = 0;
		int32 Parent = INDEX_NONE;
		int32 Step = -1;
		double Enter = 0.0;
		double Hi = ATSearchNever;
		double Arrive = 0.0;
		float Along = 0.f;
	};

	struct FOpen
	{
		double Arrive = 0.0;
		int32 State = INDEX_NONE;
		bool operator<(const FOpen& Other) const { return Arrive < Other.Arrive; }
	};

	class FSearch
	{
	public:
		explicit FSearch(const FATSearchRequest& InRequest)
			: Request(InRequest), Graph(*InRequest.Graph)
		{
		}

		void Reset()
		{
			States.Reset();
			FState Start;
			Start.Node = Request.Start;
			Start.Enter = Request.StartTime;
			Start.Arrive = Request.StartTime;
			States.Add(Start);
			ReachedTracks.Reset();
		}

		// The entry times at which the front, coming from From, may not enter Node's track: while the track, a track
		// crossing it or the switch at its start would be booked by someone else as the train passes, or a train the
		// same way would not stay a headway ahead at both ends.
		void CollectForbidden(int32 From, int32 Node, TArray<FForbidden>& Out) const
		{
			const double Speed = Request.Cruise;
			const double H = Request.Headway;
			const int32 Track = Graph.TrackOf[Node];
			const double OnTrack = (Graph.Length[Track] + Request.Length) / Speed;
			for (const int32 Conflict : Graph.Conflicts[Track])
			{
				const TArray<FATSearchBooking>* Bookings = Request.TrackBookings.Find(Conflict);
				if (!Bookings)
				{
					continue;
				}
				for (const FATSearchBooking& Booking : *Bookings)
				{
					const bool bOpen = Booking.Clear >= ATSearchNever;
					if (Conflict == Track && Booking.Exit == Node)
					{
						const double Behind = bOpen ? ATSearchNever : FMath::Max(Booking.Enter + H, Booking.Clear + H - OnTrack);
						const double Ahead = Booking.bLocked ? -ATSearchNever : FMath::Min(Booking.Enter - H, (bOpen ? ATSearchNever : Booking.Clear) - H - OnTrack);
						if (Behind > Ahead)
						{
							Out.Add({Ahead, Behind, &Booking, EWay::SameWay});
						}
					}
					else
					{
						Out.Add({Booking.bLocked ? -ATSearchNever : Booking.Enter - H - OnTrack, bOpen ? ATSearchNever : Booking.Clear + H, &Booking, EWay::Crossing});
					}
				}
			}
			const int32 Entry = Graph.OppositeOf[Node];
			const int32 Joint = Entry != INDEX_NONE ? Graph.JointOf[Entry] : INDEX_NONE;
			const TArray<FATSearchBooking>* Bookings = Joint != INDEX_NONE ? Request.JointBookings.Find(Joint) : nullptr;
			if (!Bookings)
			{
				return;
			}
			// The zone is entered Before ahead of the track and left OnJoint after entering the track.
			const double Before = Graph.FoulOf[Entry] / Speed;
			const double OnJoint = (Request.Length + Graph.JointFoul.FindRef(Joint)) / Speed;
			for (const FATSearchBooking& Booking : *Bookings)
			{
				const bool bOpen = Booking.Clear >= ATSearchNever;
				if (Booking.Exit == Node && Booking.Via == From)
				{
					const double Behind = bOpen ? ATSearchNever : FMath::Max(Booking.Enter + H + Before, Booking.Clear + H - OnJoint);
					const double Ahead = Booking.bLocked ? -ATSearchNever : FMath::Min(Booking.Enter - H + Before, (bOpen ? ATSearchNever : Booking.Clear) - H - OnJoint);
					if (Behind > Ahead)
					{
						Out.Add({Ahead, Behind, &Booking, EWay::SameWay});
					}
				}
				else
				{
					Out.Add({Booking.bLocked ? -ATSearchNever : Booking.Enter - H - OnJoint, bOpen ? ATSearchNever : Booking.Clear + H + Before, &Booking, EWay::Switch});
				}
			}
		}

		// The windows of entry times outside CollectForbidden's.
		const TArray<FInterval>& WindowsOf(int32 From, int32 Node)
		{
			const uint64 Key = (static_cast<uint64>(static_cast<uint32>(From)) << 32) | static_cast<uint32>(Node);
			if (const TArray<FInterval>* Known = Cache.Find(Key))
			{
				return *Known;
			}
			TArray<FForbidden> Forbidden;
			CollectForbidden(From, Node, Forbidden);
			Forbidden.Sort([](const FForbidden& A, const FForbidden& B) { return A.Lo < B.Lo; });
			TArray<FInterval>& Windows = Cache.Add(Key);
			double Lo = -ATSearchNever;
			for (const FForbidden& Block : Forbidden)
			{
				if (Block.Lo > Lo + 1e-3)
				{
					Windows.Add({Lo, Block.Lo});
				}
				Lo = FMath::Max(Lo, Block.Hi);
			}
			if (Lo < ATSearchNever)
			{
				Windows.Add({Lo, ATSearchNever});
			}
			return Windows;
		}

		// Latest the front may move off from the end of the state's track: every track still under the body, and every
		// switch whose fouling zone it still reaches, must be left by the end of the window it was entered in, and what
		// the body holds at the start before another train is booked onto it. OutLimit becomes the state whose window
		// sets it, or -1 for a hold with OutHold its index.
		double LatestDeparture(int32 StateIndex, int32* OutLimit = nullptr, int32* OutHold = nullptr) const
		{
			const double Speed = Request.Cruise;
			double Latest = ATSearchNever;
			auto Limit = [&](double Time, int32 State, int32 Hold)
			{
				if (Time < Latest)
				{
					Latest = Time;
					if (OutLimit)
					{
						*OutLimit = State;
					}
					if (OutHold)
					{
						*OutHold = Hold;
					}
				}
			};
			const float Along = States[StateIndex].Along;
			for (int32 Hold = 0; Hold < Request.Holds.Num(); ++Hold)
			{
				if (Request.Holds[Hold].Reach > Along)
				{
					Limit(Request.Holds[Hold].Latest + Along / Speed, -1, Hold);
				}
			}
			float Behind = 0.f;
			for (int32 Index = StateIndex; Index != INDEX_NONE && States[Index].Parent != INDEX_NONE; Index = States[Index].Parent)
			{
				const FState& State = States[Index];
				if (Behind > Request.Length + BodyReachMargin)
				{
					break;
				}
				if (State.Hi < ATSearchNever)
				{
					const int32 Track = Graph.TrackOf[State.Node];
					const float Length = Graph.Length[Track];
					const float TrackLeft = Request.Length - Behind;
					if (TrackLeft > 0.f)
					{
						Limit(State.Hi + (Length + Request.Length) / Speed - TrackLeft / Speed, Index, INDEX_NONE);
					}
					const int32 Entry = Graph.OppositeOf[State.Node];
					const int32 Joint = Entry != INDEX_NONE ? Graph.JointOf[Entry] : INDEX_NONE;
					if (Joint != INDEX_NONE)
					{
						const float Foul = Graph.JointFoul.FindRef(Joint);
						const float JointLeft = Request.Length + Foul - Behind - Length;
						if (JointLeft > 0.f)
						{
							Limit(State.Hi + (Request.Length + Foul) / Speed - JointLeft / Speed, Index, INDEX_NONE);
						}
					}
				}
				Behind += Graph.Length[Graph.TrackOf[State.Node]];
			}
			return Latest;
		}

		// Returns true if the way to the state already runs over Track.
		bool OnPath(int32 StateIndex, int32 Track) const
		{
			for (int32 Index = StateIndex; Index != INDEX_NONE; Index = States[Index].Parent)
			{
				if (Graph.TrackOf[States[Index].Node] == Track)
				{
					return true;
				}
			}
			return false;
		}

		// Adds a state for each window in which the front, at the end of the state's track, can go on to Node's track,
		// waiting first if it may; OutStates gets their indices.
		void Enter(int32 StateIndex, int32 Node, TArray<int32>& OutStates)
		{
			if (Node == INDEX_NONE || States.Num() >= MaxStates)
			{
				return;
			}
			const int32 Track = Graph.TrackOf[Node];
			if (ReachedTracks.Contains(Track) && OnPath(StateIndex, Track))
			{
				return;
			}
			const double Reached = States[StateIndex].Arrive;
			const double Latest = LatestDeparture(StateIndex);
			const double Drive = Graph.Length[Track] / Request.Cruise;
			const TArray<FInterval>& Windows = WindowsOf(States[StateIndex].Node, Node);
			for (int32 Window = 0; Window < Windows.Num(); ++Window)
			{
				const FInterval& Free = Windows[Window];
				if (Free.Hi < Reached)
				{
					continue;
				}
				const double EnterAt = FMath::Max(Free.Lo, Reached);
				if (EnterAt > Latest)
				{
					break;
				}
				// A wait costs, up to a full stop, the time lost braking and picking up speed again.
				const double Wait = EnterAt - Reached;
				FState Next;
				Next.Node = Node;
				Next.Window = Window;
				Next.Parent = StateIndex;
				Next.Step = States[StateIndex].Step + 1;
				Next.Enter = EnterAt;
				Next.Hi = Free.Hi;
				Next.Arrive = EnterAt + Drive + FMath::Min(Wait, static_cast<double>(Request.StopLoss));
				Next.Along = States[StateIndex].Along + Graph.Length[Track];
				OutStates.Add(States.Add(Next));
				ReachedTracks.Add(Track);
			}
		}

		// The connections after the start on the way to the state.
		TArray<int32> PathTo(int32 StateIndex) const
		{
			TArray<int32> Path;
			for (int32 Index = StateIndex; Index != INDEX_NONE && States[Index].Parent != INDEX_NONE; Index = States[Index].Parent)
			{
				Path.Add(States[Index].Node);
			}
			Algo::Reverse(Path);
			return Path;
		}

		// Returns true if the front, at From's track end, may go on through Node's track.
		bool CanPass(int32 From, int32 Node) const
		{
			const int32 Entry = Graph.OppositeOf.IsValidIndex(Node) ? Graph.OppositeOf[Node] : INDEX_NONE;
			return !Graph.OneWay[From] && Graph.Attached[From].Contains(Entry);
		}

		// Earliest arrival at Goal's track end, along Route only if given (then at its last step); returns the state, or
		// INDEX_NONE. OutFurthest becomes the state furthest along Route that the search reached.
		int32 Find(int32 Goal, const TArray<int32>* Route, int32* OutFurthest = nullptr)
		{
			Reset();
			TMap<uint64, double> Best;
			TArray<FOpen> Open;
			Open.HeapPush({Request.StartTime, 0});
			TArray<int32> Next;
			int32 Furthest = 0;
			while (Open.Num() > 0)
			{
				FOpen Item;
				Open.HeapPop(Item);
				const FState State = States[Item.State];
				const uint64 Key = (static_cast<uint64>(State.Node) << 32) | static_cast<uint32>(State.Window);
				const double* Known = Item.State != 0 ? Best.Find(Key) : nullptr;
				if (Known && *Known < State.Arrive)
				{
					continue;
				}
				if (Route)
				{
					if (State.Step > States[Furthest].Step)
					{
						Furthest = Item.State;
					}
					if (State.Step == Route->Num() - 1)
					{
						if (OutFurthest)
						{
							*OutFurthest = Item.State;
						}
						return Item.State;
					}
				}
				else if (State.Node == Goal && Item.State != 0)
				{
					return Item.State;
				}
				TArray<int32, TInlineAllocator<4>> Nodes;
				if (Route)
				{
					const int32 NextNode = (*Route)[State.Step + 1];
					if (CanPass(State.Node, NextNode))
					{
						Nodes.Add(NextNode);
					}
				}
				else if (!Graph.OneWay[State.Node])
				{
					for (const int32 Entry : Graph.Attached[State.Node])
					{
						Nodes.Add(Graph.OppositeOf.IsValidIndex(Entry) ? Graph.OppositeOf[Entry] : INDEX_NONE);
					}
				}
				for (const int32 Node : Nodes)
				{
					Next.Reset();
					Enter(Item.State, Node, Next);
					for (const int32 Candidate : Next)
					{
						const FState& Reached = States[Candidate];
						const uint64 NextKey = (static_cast<uint64>(Reached.Node) << 32) | static_cast<uint32>(Reached.Window);
						double& Stored = Best.FindOrAdd(NextKey, ATSearchNever);
						if (Reached.Arrive < Stored)
						{
							Stored = Reached.Arrive;
							Open.HeapPush({Reached.Arrive, Candidate});
						}
					}
				}
			}
			if (OutFurthest)
			{
				*OutFurthest = Furthest;
			}
			return INDEX_NONE;
		}

		// Why Route can't be driven on from the state furthest along it.
		FATSearchBlock DescribeBlock(const TArray<int32>& Route, int32 Furthest)
		{
			FATSearchBlock Block;
			const FState& At = States[Furthest];
			Block.Step = At.Step + 1;
			if (!Route.IsValidIndex(Block.Step))
			{
				return Block;
			}
			const int32 Node = Route[Block.Step];
			if (!CanPass(At.Node, Node))
			{
				Block.Way = Graph.OneWay[At.Node] ? EWay::Signal : EWay::NotAttached;
				return Block;
			}
			Block.Reached = At.Arrive;
			int32 LimitState = INDEX_NONE;
			int32 Hold = INDEX_NONE;
			Block.Latest = LatestDeparture(Furthest, &LimitState, &Hold);
			if (LimitState == -1 && Request.Holds.IsValidIndex(Hold))
			{
				Block.LimitStep = -1;
				Block.LimitOwner = Request.Holds[Hold].Owner;
			}
			else if (States.IsValidIndex(LimitState))
			{
				Block.LimitStep = States[LimitState].Step;
				// The booking whose blocked time starts where that state's window ends.
				TArray<FForbidden> Closing;
				CollectForbidden(States[States[LimitState].Parent].Node, States[LimitState].Node, Closing);
				for (const FForbidden& Forbidden : Closing)
				{
					if (FMath::IsNearlyEqual(Forbidden.Lo, States[LimitState].Hi, 1e-3))
					{
						Block.LimitOwner = Forbidden.Booking->Owner;
					}
				}
			}
			// The booking that keeps the step closed the longest from the arrival on.
			TArray<FForbidden> Forbidden;
			CollectForbidden(At.Node, Node, Forbidden);
			double Covered = -ATSearchNever;
			for (const FForbidden& Candidate : Forbidden)
			{
				if (Candidate.Lo <= Block.Reached && Candidate.Hi > Block.Reached && Candidate.Hi > Covered)
				{
					Covered = Candidate.Hi;
					Block.Owner = Candidate.Booking->Owner;
					Block.Way = Candidate.Way;
					Block.bLocked = Candidate.Booking->bLocked;
				}
			}
			Block.Opens = Block.Reached;
			for (const FInterval& Window : WindowsOf(At.Node, Node))
			{
				if (Window.Hi >= Block.Reached)
				{
					Block.Opens = FMath::Max(Window.Lo, Block.Reached);
					break;
				}
			}
			return Block;
		}

		const FATSearchRequest& Request;
		const FATSearchGraph& Graph;
		TArray<FState> States;
		TMap<uint64, TArray<FInterval>> Cache;
		TSet<int32> ReachedTracks;
	};
}

void ATRouteSearch::Launch(const TSharedRef<FATSearchTask, ESPMode::ThreadSafe>& Task)
{
	AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask, [Task]()
	{
		Run(*Task);
		Task->bDone.store(true, std::memory_order_release);
	});
}

void ATRouteSearch::Run(FATSearchTask& Task)
{
	const double Begin = FPlatformTime::Seconds();
	const FATSearchRequest& Request = Task.Request;
	FATSearchResult& Result = Task.Result;
	Result.Arrival = ATSearchNever;
	Result.CurrentArrival = ATSearchNever;
	const FATSearchGraph* Graph = Request.Graph.Get();
	if (!Graph || !Graph->TrackOf.IsValidIndex(Request.Start) || !Graph->TrackOf.IsValidIndex(Request.Goal))
	{
		return;
	}
	FSearch Search(Request);
	if (Request.Current.Num() > 0)
	{
		int32 Furthest = 0;
		const int32 Reached = Search.Find(INDEX_NONE, &Request.Current, &Furthest);
		if (Reached != INDEX_NONE)
		{
			Result.CurrentArrival = Search.States[Reached].Arrive;
		}
		else
		{
			Result.CurrentBlock = Search.DescribeBlock(Request.Current, Furthest);
		}
	}
	const int32 Found = Search.Find(Request.Goal, nullptr);
	Result.States = Search.States.Num();
	if (Found != INDEX_NONE)
	{
		Result.Route = Search.PathTo(Found);
		Result.Arrival = Search.States[Found].Arrive;
	}
	Result.Milliseconds = (FPlatformTime::Seconds() - Begin) * 1000.0;
}
