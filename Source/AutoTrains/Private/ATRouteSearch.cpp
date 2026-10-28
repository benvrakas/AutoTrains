#include "ATRouteSearch.h"
#include "Async/Async.h"
#include "HAL/PlatformTime.h"

namespace
{
	// Search size limit per request (a few MB at most).
	constexpr int32 MaxStates = 50000;
	// How far behind the front a track can still be under the train's body or a switch's fouling zone. [cm]
	constexpr float BodyReachMargin = 5000.f;

	// A window of entry times. [world s]
	struct FInterval
	{
		double Lo = 0.0;
		double Hi = 0.0;
	};

	// The front at the end of Node's track, having entered it at Enter in a window that allowed entering up to Hi, and
	// reaching the end at Arrive. [world s]
	struct FState
	{
		int32 Node = INDEX_NONE;
		int32 Window = 0;
		int32 Parent = INDEX_NONE;
		double Enter = 0.0;
		double Hi = ATSearchNever;
		double Arrive = 0.0;
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

		// The entry windows of Node's track, entered through the connection at its other end: times the front may enter
		// so that neither the track (nor a track crossing it) nor the switch at its start is booked while the train is on
		// it, or a train ahead the same way stays a headway ahead at both ends.
		const TArray<FInterval>& WindowsOf(int32 Node)
		{
			if (const TArray<FInterval>* Known = Cache.Find(Node))
			{
				return *Known;
			}
			const double Speed = Request.Cruise;
			const double H = Request.Headway;
			const int32 Track = Graph.TrackOf[Node];
			const double OnTrack = (Graph.Length[Track] + Request.Length) / Speed;
			TArray<FInterval> Forbidden;
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
							Forbidden.Add({Ahead, Behind});
						}
					}
					else
					{
						Forbidden.Add({Booking.bLocked ? -ATSearchNever : Booking.Enter - H - OnTrack, bOpen ? ATSearchNever : Booking.Clear + H});
					}
				}
			}
			const int32 Entry = Graph.OppositeOf[Node];
			const int32 Joint = Entry != INDEX_NONE ? Graph.JointOf[Entry] : INDEX_NONE;
			if (Joint != INDEX_NONE)
			{
				if (const TArray<FATSearchBooking>* Bookings = Request.JointBookings.Find(Joint))
				{
					const double Before = Graph.FoulOf[Entry] / Speed;
					const double OnJoint = (Request.Length + Graph.JointFoul.FindRef(Joint)) / Speed;
					for (const FATSearchBooking& Booking : *Bookings)
					{
						Forbidden.Add({Booking.bLocked ? -ATSearchNever : Booking.Enter - H - OnJoint, Booking.Clear >= ATSearchNever ? ATSearchNever : Booking.Clear + H + Before});
					}
				}
			}
			Forbidden.Sort([](const FInterval& A, const FInterval& B) { return A.Lo < B.Lo; });
			TArray<FInterval>& Windows = Cache.Add(Node);
			double From = -ATSearchNever;
			for (const FInterval& Window : Forbidden)
			{
				if (Window.Lo > From + 1e-3)
				{
					Windows.Add({From, Window.Lo});
				}
				From = FMath::Max(From, Window.Hi);
			}
			if (From < ATSearchNever)
			{
				Windows.Add({From, ATSearchNever});
			}
			return Windows;
		}

		// Latest the front may move off from the end of the state's track: every track still under the body, and every
		// switch whose fouling zone it still reaches, must be left by the end of the window it was entered in.
		double LatestDeparture(int32 StateIndex) const
		{
			const double Speed = Request.Cruise;
			double Latest = ATSearchNever;
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
						Latest = FMath::Min(Latest, State.Hi + (Length + Request.Length) / Speed - TrackLeft / Speed);
					}
					const int32 Entry = Graph.OppositeOf[State.Node];
					const int32 Joint = Entry != INDEX_NONE ? Graph.JointOf[Entry] : INDEX_NONE;
					if (Joint != INDEX_NONE)
					{
						const float Foul = Graph.JointFoul.FindRef(Joint);
						const float JointLeft = Request.Length + Foul - Behind - Length;
						if (JointLeft > 0.f)
						{
							Latest = FMath::Min(Latest, State.Hi + (Request.Length + Foul) / Speed - JointLeft / Speed);
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
			const TArray<FInterval>& Windows = WindowsOf(Node);
			for (int32 Window = 0; Window < Windows.Num(); ++Window)
			{
				const FInterval& Free = Windows[Window];
				if (Free.Hi < Reached)
				{
					continue;
				}
				if (Free.Lo > Latest)
				{
					break;
				}
				const double EnterAt = FMath::Max(Free.Lo, Reached);
				// A wait costs, up to a full stop, the time lost braking and picking up speed again.
				const double Wait = EnterAt - Reached;
				FState Next;
				Next.Node = Node;
				Next.Window = Window;
				Next.Parent = StateIndex;
				Next.Enter = EnterAt;
				Next.Hi = Free.Hi;
				Next.Arrive = EnterAt + Drive + FMath::Min(Wait, static_cast<double>(Request.StopLoss));
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

		const FATSearchRequest& Request;
		const FATSearchGraph& Graph;
		TArray<FState> States;
		TMap<int32, TArray<FInterval>> Cache;
		TSet<int32> ReachedTracks;
	};

	// When the front reaches the end of Route's last track going this way, taking the first window it can at each track.
	double TimeRoute(FSearch& Search, const TArray<int32>& Route)
	{
		Search.Reset();
		int32 At = 0;
		TArray<int32> Next;
		for (const int32 Node : Route)
		{
			const int32 From = Search.States[At].Node;
			const int32 Entry = Search.Graph.OppositeOf.IsValidIndex(Node) ? Search.Graph.OppositeOf[Node] : INDEX_NONE;
			if (Search.Graph.OneWay[From] || !Search.Graph.Attached[From].Contains(Entry))
			{
				return ATSearchNever;
			}
			Next.Reset();
			Search.Enter(At, Node, Next);
			if (Next.Num() == 0)
			{
				return ATSearchNever;
			}
			At = Next[0];
			for (const int32 Candidate : Next)
			{
				At = Search.States[Candidate].Arrive < Search.States[At].Arrive ? Candidate : At;
			}
		}
		return Search.States[At].Arrive;
	}
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
	Result.CurrentArrival = TimeRoute(Search, Request.Current);

	Search.Reset();
	TMap<uint64, double> Best;
	TArray<FOpen> Open;
	Open.HeapPush({Request.StartTime, 0});
	TArray<int32> Next;
	int32 Found = INDEX_NONE;
	while (Open.Num() > 0)
	{
		FOpen Item;
		Open.HeapPop(Item);
		const FState State = Search.States[Item.State];
		const uint64 Key = (static_cast<uint64>(State.Node) << 32) | static_cast<uint32>(State.Window);
		const double* Known = Item.State != 0 ? Best.Find(Key) : nullptr;
		if (Known && *Known < State.Arrive)
		{
			continue;
		}
		if (State.Node == Request.Goal && Item.State != 0)
		{
			Found = Item.State;
			break;
		}
		if (Graph->OneWay[State.Node])
		{
			continue;
		}
		for (const int32 Entry : Graph->Attached[State.Node])
		{
			Next.Reset();
			Search.Enter(Item.State, Graph->OppositeOf.IsValidIndex(Entry) ? Graph->OppositeOf[Entry] : INDEX_NONE, Next);
			for (const int32 Candidate : Next)
			{
				const FState& Reached = Search.States[Candidate];
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
	Result.States = Search.States.Num();
	if (Found != INDEX_NONE)
	{
		Result.Route = Search.PathTo(Found);
		Result.Arrival = Search.States[Found].Arrive;
	}
	Result.Milliseconds = (FPlatformTime::Seconds() - Begin) * 1000.0;
}
