#include "ATTrafficSubsystem.h"
#include "ATRouter.h"
#include "ATSipp.h"
#include "AutoTrains.h"
#include "Buildables/FGBuildableRailroadStation.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "Buildables/FGBuildableTrainPlatformCargo.h"
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
	// An open-ended time: a stay nobody knows the end of, or everything after a wait for one. [s]
	constexpr double OpenTime = ATSippNever;
	// A docked train past its expected departure is expected to leave this soon, renewed every update. [s]
	constexpr double OverdueDepartSeconds = 1.0;
	// A departure that couldn't be predicted is tried again after this long. [s]
	constexpr double PredictRetrySeconds = 10.0;
	// How long after leaving a train may still be set to the route searched for its departure. [s]
	constexpr double DepartRouteSeconds = 10.0;
	// Trains that waited longer than WaitDumpSeconds are described in the log every WaitDumpInterval. [s]
	constexpr double WaitDumpSeconds = 60.0;
	constexpr double WaitDumpInterval = 30.0;
	// A train slower than this whose follow limit leaves it less than this is named as held by its leader. [cm/s] [cm]
	constexpr float FollowStuckSpeed = 50.f;
	constexpr float FollowStuckDistance = 3000.f;
	// A train slower than this plans from standing. [cm/s]
	constexpr float StandingSpeed = 50.f;
	// A plan's wait this close to its end doesn't hold back the grant. [s]
	constexpr double PlanWaitGraceSeconds = 0.5;
	// Planner counters are logged this often. [s]
	constexpr double PlanSummarySeconds = 60.0;
	// Time the docking itself adds to a freight platform's transfer, until a stay has been measured, and the share a new
	// measurement moves the transfer stay by. [s]
	constexpr float DockingSeconds = 2.f;
	constexpr float TransferStayRate = 0.2f;
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

float AATTrafficSubsystem::GetExpectedStay(const AFGTrain* Train, const AFGBuildableRailroadStation* Station, const FTrainDockingRuleSet* Rules, int32 Transfer) const
{
	const TPair<FObjectKey, FObjectKey> Key{FObjectKey(Train), FObjectKey(Station)};
	if (!Station || !Rules || Rules->DockingDefinition == ETrainDockingDefinition::TDD_FullyLoadUnload)
	{
		const float* Own = Station ? mDwell.Find(Key) : nullptr;
		return Own ? *Own : mSettings.DefaultStaySeconds;
	}
	const bool* Last = mLastTransfer.Find(Key);
	const bool bTransfer = Transfer >= 0 ? Transfer > 0 : !Last || *Last;
	const float Once = bTransfer ? GetTransferStay() : Rules->DockForDuration;
	return Rules->IsDurationAndRule ? FMath::Max(Rules->DockForDuration, Once) : Once;
}

float AATTrafficSubsystem::GetTransferStay() const
{
	return mTransferStay > 0.f ? mTransferStay : mPlatformSeconds > 0.f ? mPlatformSeconds + DockingSeconds : mSettings.DefaultStaySeconds;
}

const FTrainDockingRuleSet* AATTrafficSubsystem::GetStopRules(const AFGTrain* Train, int32 Index, const AFGBuildableRailroadStation* Station, FTrainDockingRuleSet& Storage)
{
	AFGRailroadTimeTable* TimeTable = Train ? Train->GetTimeTable() : nullptr;
	TArray<FTimeTableStop> Stops;
	if (TimeTable)
	{
		TimeTable->GetStops(Stops);
	}
	if (!Station || !Stops.IsValidIndex(Index) || !Stops[Index].Station || Stops[Index].Station->GetStation() != Station)
	{
		return nullptr;
	}
	Storage = Stops[Index].DockingRuleSet;
	return &Storage;
}

void AATTrafficSubsystem::TrackDocking(FATTrafficTrain& State, AFGTrain* Train, double Now)
{
	const bool bDocked = Train->IsDocked();
	if (bDocked && !State.bWasDocked)
	{
		// A dock already going on when this mod first saw the train started at an unknown time.
		State.DockedAt = State.bSeenUndocked ? Now : -1.0;
		State.DockStation = Train->mDockedAtStation.Get();
		State.DockTransfer = -1;
		State.PredictedRoute.Reset();
		State.PredictedStation = nullptr;
		State.PredictedAt = -1e9;
		State.bPredictFailed = false;
		State.bPredictRerouted = false;
		State.DepartRoute.Reset();
	}
	else if (!bDocked && State.bWasDocked)
	{
		const AFGBuildableRailroadStation* Station = State.DockStation.Get();
		if (State.DockedAt >= 0.0 && Station)
		{
			const float Dwell = Now - State.DockedAt;
			const float Before = GetExpectedStay(Train, Station, &State.DockRules, State.DockTransfer);
			const TPair<FObjectKey, FObjectKey> Key{FObjectKey(Train), FObjectKey(Station)};
			const bool bFully = State.DockRules.DockingDefinition == ETrainDockingDefinition::TDD_FullyLoadUnload;
			if (bFully)
			{
				mDwell.Add(Key, Dwell);
			}
			else
			{
				mLastTransfer.Add(Key, State.DockTransfer > 0);
				if (State.DockTransfer > 0)
				{
					mTransferStay = mTransferStay > 0.f ? mTransferStay + (Dwell - mTransferStay) * TransferStayRate : Dwell;
				}
			}
			UE_LOG(LogAutoTrains, Log, TEXT("planner: %s left %s after %.1f s (expected %.1f s; %s, %s)"), *NameOf(Train), *StationName(Station), Dwell, Before,
				bFully ? TEXT("fully load/unload") : TEXT("load/unload once"), State.DockTransfer > 0 ? TEXT("cargo moved") : State.DockTransfer == 0 ? TEXT("no cargo to move") : TEXT("cargo unknown"));
		}
		State.DockedAt = -1.0;
		// Leaving on the route searched for its departure, once the game has given it a path.
		State.DepartRoute.Reset();
		if (State.bPredictRerouted)
		{
			for (const FATRouteStep& Step : State.PredictedRoute)
			{
				State.DepartRoute.Add(Step.Exit);
			}
			State.DepartRouteUntil = Now + DepartRouteSeconds;
		}
		State.PredictedRoute.Reset();
		State.PredictedStation = nullptr;
		State.bPredictFailed = false;
		State.bPredictRerouted = false;
		State.LegStart = -1.0;
	}
	// While docked: the stop's rules, and whether the freight platforms move cargo.
	AFGBuildableRailroadStation* Docked = bDocked ? State.DockStation.Get() : nullptr;
	if (Docked)
	{
		State.DockRules = Docked->GetDockedTrainRuleSet();
		bool bUndecided = false;
		for (AFGBuildableTrainPlatform* Platform : Docked->mDockedPlatformList)
		{
			const AFGBuildableTrainPlatformCargo* Cargo = Cast<AFGBuildableTrainPlatformCargo>(Platform);
			if (!Cargo)
			{
				continue;
			}
			if (mPlatformSeconds < 0.f)
			{
				mPlatformSeconds = FMath::Max(Cargo->mTimeToCompleteLoad, Cargo->mTimeToCompleteUnload);
				UE_LOG(LogAutoTrains, Log, TEXT("planner: freight platforms move cargo in %.1f s"), mPlatformSeconds);
			}
			const ETrainPlatformDockingStatus Status = Cargo->GetDockingStatus();
			if (Status == ETrainPlatformDockingStatus::ETPDS_Loading || Status == ETrainPlatformDockingStatus::ETPDS_Unloading)
			{
				State.DockTransfer = 1;
			}
			bUndecided = bUndecided || (Status != ETrainPlatformDockingStatus::ETPDS_Complete && Status != ETrainPlatformDockingStatus::ETPDS_IdleWaitForTime);
		}
		if (State.DockTransfer < 0 && !bUndecided)
		{
			State.DockTransfer = 0;
		}
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
	int32 NextStop = INDEX_NONE;
	for (int32 Offset = 0; Offset < Stops.Num() && !Next; ++Offset)
	{
		const FTimeTableStop& Stop = Stops[(Current + Offset) % Stops.Num()];
		AFGBuildableRailroadStation* Station = Stop.Station ? Stop.Station->GetStation() : nullptr;
		if (Station && Station != Docked)
		{
			Next = Station;
			NextStop = (Current + Offset) % Stops.Num();
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
	State.PredictedStop = NextStop;
	++mPlanPredictions;
	UE_LOG(LogAutoTrains, Log, TEXT("planner: %s docked at %s, expected to stay %.0f s; next leg to %s: %d tracks, %.0f m, leaving with %s"), *NameOf(Train),
		*StationName(Docked), GetExpectedStay(Train, Docked, &State.DockRules, State.DockTransfer), *StationName(Next), State.PredictedRoute.Num(), BestLength / 100.f,
		BestLocomotive == Train->GetMultipleUnitMaster() ? TEXT("its lead locomotive") : TEXT("another locomotive (the other way)"));
}

void AATTrafficSubsystem::ApplyDepartureReroute(FATTrafficTrain& State, const FATSippCycle& Cycle, int32 Slot)
{
	const FATSippGraph& Graph = *Cycle.Graph;
	const FATSippResult& Result = Cycle.Results[Slot];
	if (Result.Route.Num() == 0 || !Graph.Connections.IsValidIndex(Result.Route[0]) || Graph.Connections[Result.Route[0]].Get() != State.PredictedRoute[0].Exit)
	{
		return;
	}
	TArray<FATRouteStep> Route;
	float Distance = State.PredictedRoute[0].EndDistance;
	for (int32 Index = 0; Index < Result.Route.Num(); ++Index)
	{
		UFGRailroadTrackConnectionComponent* Exit = Graph.Connections[Result.Route[Index]].Get();
		AFGBuildableRailroadTrack* Track = Exit ? Exit->GetTrack() : nullptr;
		if (!Track)
		{
			return;
		}
		Distance += Index > 0 ? Track->GetLength() : 0.f;
		Route.Add({Track, Exit, Distance});
	}
	UE_LOG(LogAutoTrains, Log, TEXT("planner: %s's departure from %s rerouted: %s -> %.0f s to %s, %d -> %d tracks (route search: %d states, %.2f ms on a worker)"),
		*NameOf(State.Train.Get()), *StationName(State.DockStation.Get()), IsOpen(Result.OldArrival) ? TEXT("no way") : *FString::Printf(TEXT("%.0f s"), Result.OldArrival - Cycle.Now),
		Result.Arrival - Cycle.Now, *StationName(State.PredictedStation.Get()), State.PredictedRoute.Num(), Route.Num(), Result.SearchStates, Result.SearchMilliseconds);
	State.PredictedRoute = MoveTemp(Route);
	State.bPredictRerouted = true;
	++mPlanReroutes;
}

void AATTrafficSubsystem::ApplyDepartRoute(FATTrafficTrain& State, AFGTrain* Train, double Now)
{
	if (State.DepartRoute.Num() == 0)
	{
		return;
	}
	if (Now > State.DepartRouteUntil)
	{
		UE_LOG(LogAutoTrains, Log, TEXT("planner: %s left on the game's own path: its path never started the way of the route searched for its departure"), *NameOf(Train));
		State.DepartRoute.Reset();
		return;
	}
	if (Train->IsDocked() || !Train->mAtcData.Path.IsValid() || State.Route.Num() == 0)
	{
		return;
	}
	// From where the game's path and the searched route meet: the connection the front leaves its track through.
	const int32 From = State.DepartRoute.IndexOfByPredicate([&State](const TWeakObjectPtr<UFGRailroadTrackConnectionComponent>& Exit) { return Exit.Get() == State.Route[0].Exit; });
	if (From == INDEX_NONE)
	{
		return;
	}
	TArray<UFGRailroadTrackConnectionComponent*> Connections;
	for (int32 Index = From; Index < State.DepartRoute.Num(); ++Index)
	{
		UFGRailroadTrackConnectionComponent* Exit = State.DepartRoute[Index].Get();
		if (!Exit || !Exit->GetTrack())
		{
			State.DepartRoute.Reset();
			return;
		}
		Connections.Add(Exit);
	}
	State.DepartRoute.Reset();
	const bool bApplied = FATRouter::ApplyPath(Train, FATRouter::BuildPath(Connections, Train->mAtcData.Path->Station.Get()), Connections.Last());
	State.RerouteCooldown = mSettings.RerouteHoldSeconds;
	UE_LOG(LogAutoTrains, Log, TEXT("planner: %s left on the route searched for its departure (%d tracks), applied %s"), *NameOf(Train), Connections.Num(),
		bApplied ? TEXT("yes") : TEXT("NO"));
}

bool AATTrafficSubsystem::CanEnter(FATTrafficTrain& Self, const FResourceKey& Key, int32 Index, double Enter, bool bLocked, bool bHeldBefore,
	const UFGRailroadTrackConnectionComponent* Via, const UFGRailroadTrackConnectionComponent* Exit, double Now, const FATTrafficTrain*& OutBlocker,
	FString& OutWhy) const
{
	const AFGTrain* SelfTrain = Self.Train.Get();
	// Without a slot in its plan nothing new is given, but what it held from its last grant it keeps.
	const bool bUnplanned = IsOpen(Enter);
	if (bUnplanned && !bHeldBefore)
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
	// Held from its last grant and locked (it may not stop short of it at service braking) or without a slot in its
	// plan: the train keeps it whatever was booked since; only a train standing there or holding it stops it.
	const bool bKeeps = bHeldBefore && (bLocked || bUnplanned);
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
	auto TrackEnter = [&State](const AFGBuildableRailroadTrack* Track) { const double* Enter = State.PlanTrackEnter.Find(Track); return Enter ? *Enter : OpenTime; };
	auto JointEnter = [&State](int32 Joint) { const double* Enter = State.PlanJointEnter.Find(Joint); return Enter ? *Enter : OpenTime; };
	// Where the plan's first wait stands the front now.
	State.PlanWaitAt = -1.f;
	if (State.PlanDepart >= 0.0 && State.bPlanWaitHere)
	{
		State.PlanWaitAt = 0.f;
	}
	else if (State.PlanDepart >= 0.0 && State.PlanWaitExit.IsValid())
	{
		for (const FATRouteStep& Step : State.Route)
		{
			if (Step.Exit == State.PlanWaitExit.Get())
			{
				State.PlanWaitAt = FMath::Max(Step.EndDistance - State.FrontAhead - GetStopMargin(Step.Exit), 0.f);
				break;
			}
		}
	}
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
			if (!CanEnter(State, Key, Index, JointEnter(Joint), State.PlanJointLocked.Contains(Joint), HeldJoints.Contains(Joint), Entry, Step.Exit, Now, Blocker, Why))
			{
				Where = FString::Printf(TEXT("the switch before %s"), *ResourceName(Step.Track, INDEX_NONE));
				bBlocked = true;
			}
		}
		if (!bBlocked && !State.Occupied.Contains(Step.Track))
		{
			FResourceKey Key;
			Key.Track = Step.Track;
			if (!CanEnter(State, Key, Index, TrackEnter(Step.Track), State.PlanTrackLocked.Contains(Step.Track), HeldTracks.Contains(Step.Track), nullptr, Step.Exit, Now, Blocker, Why))
			{
				Where = ResourceName(Step.Track, INDEX_NONE);
				bBlocked = true;
			}
		}
		if (bBlocked)
		{
			State.BlockedAtTrack = Where.StartsWith(TEXT("the switch")) ? nullptr : Step.Track;
			State.BlockedAtJoint = Where.StartsWith(TEXT("the switch")) ? Joint : INDEX_NONE;
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
			FATFollowInfo Info;
			GetFollowLimit(Train, State, HardLimit, &Info);
			if (HardLimit >= 0.f && HardLimit < FollowStuckDistance && Info.Leader.IsValid())
			{
				State.BlockedBy = Info.Leader;
				State.BlockedReason = FString::Printf(TEXT("stays behind %s, %.0f m ahead"), *NameOf(Info.Leader.Get()), Info.Gap / 100.f);
				State.BlockedKey = NameOf(Info.Leader.Get()) + TEXT("|behind");
			}
		}
	}
}

int32 AATTrafficSubsystem::GetRouteCommit(const FATTrafficTrain& State) const
{
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
	return Commit;
}

bool AATTrafficSubsystem::IsSearchDue(FATTrafficTrain& State, float DeltaSeconds)
{
	AFGTrain* Train = State.Train.Get();
	if (!Train || !mRerouteEnabled || !Train->mAtcData.Path.IsValid())
	{
		return false;
	}
	if (State.RerouteTimer < 0.f)
	{
		// Spread the searches out over the interval.
		State.RerouteTimer = FMath::FRandRange(0.f, mSettings.RouteSearchSeconds);
	}
	State.RerouteTimer -= DeltaSeconds;
	if (!State.PlanFailure.IsEmpty())
	{
		State.RerouteTimer = FMath::Min(State.RerouteTimer, mSettings.RouteSearchStuckSeconds);
	}
	const bool bWorse = State.SearchedArrival >= 0.0 && !IsOpen(State.SearchedArrival) && State.PlanArrival - State.SearchedArrival > mSettings.RouteSearchLateSeconds;
	if ((State.RerouteTimer > 0.f && !bWorse) || State.RerouteCooldown > 0.f || State.Route.Num() < 3)
	{
		return false;
	}
	State.RerouteTimer = State.PlanFailure.IsEmpty() ? mSettings.RouteSearchSeconds : mSettings.RouteSearchStuckSeconds;
	State.SearchedArrival = State.PlanArrival;
	return true;
}

void AATTrafficSubsystem::FillSippTrain(const FATTrafficTrain& State, const FATSippGraph& Graph, const TMap<const AFGTrain*, int32>& SlotOf, FATSippTrain& Out) const
{
	auto ResourceOf = [&Graph](const FResourceKey& Key)
	{
		if (Key.Joint != INDEX_NONE)
		{
			return Graph.JointFoul.IsValidIndex(Key.Joint) ? Graph.JointResource(Key.Joint) : INDEX_NONE;
		}
		const int32* Track = Graph.TrackIndex.Find(Key.Track);
		return Track ? *Track : INDEX_NONE;
	};
	auto NodeOf = [&Graph](const UFGRailroadTrackConnectionComponent* Connection)
	{
		const int32* Node = Connection ? Graph.ConnectionIndex.Find(Connection) : nullptr;
		return Node ? *Node : INDEX_NONE;
	};
	// The route as far as the network copy knows it; cut short, it has no goal.
	Out.Route.Reset();
	Out.bGoal = true;
	for (const FATRouteStep& Step : State.Route)
	{
		const int32 Node = NodeOf(Step.Exit);
		if (Node == INDEX_NONE)
		{
			Out.bGoal = false;
			break;
		}
		Out.Route.Add(Node);
	}
	Out.FirstEnd = State.Route.Num() > 0 ? State.Route[0].EndDistance - State.FrontAhead : 0.f;
	const FRouteSpeed Speed = GetRouteSpeed(State);
	Out.Length = Speed.Length;
	Out.Cruise = Speed.Cruise;
	Out.Accel = Speed.Accel;
	Out.Brake = Speed.Brake;
	Out.Speed = State.Speed;
	Out.FrontZ = State.FrontZ;
	Out.RearZ = State.RearZ;
	Out.FrontZAt = State.FrontZAt;
	Out.RearZAt = State.RearZAt;
	Out.bStanding = State.Speed < StandingSpeed;
	// Held: what the train holds from its last grant (route steps it holds from its front on, and the switches between
	// them). Locked: what it stands on, can't stop short of, or holds within its full-brake stopping distance plus the
	// stop distance; only what it holds or can't avoid, never a resource ahead it merely stands close to.
	TArray<FPass> Passes;
	CollectPasses(State, Passes);
	const float Committed = GetStoppingDistance(State, State.Speed);
	const float LockedWithin = Committed + GetStopDistance();
	int32 HeldIndex = INDEX_NONE;
	while (HeldIndex + 1 < State.Route.Num() && HoldsTrack(State, State.Route[HeldIndex + 1].Track))
	{
		++HeldIndex;
	}
	for (const FPass& Pass : Passes)
	{
		const int32 Resource = ResourceOf(Pass.Key);
		if (Resource == INDEX_NONE)
		{
			continue;
		}
		const bool bHolds = Pass.RouteIndex != INDEX_NONE && Pass.RouteIndex <= HeldIndex;
		if (Pass.RouteIndex == INDEX_NONE || Pass.EnterAt <= 0.f || Pass.EnterAt <= Committed || (bHolds && Pass.EnterAt <= LockedWithin))
		{
			Out.Locked.Add(Resource);
		}
		else if (bHolds)
		{
			Out.Held.Add(Resource);
		}
		if (Pass.RouteIndex == INDEX_NONE)
		{
			FATSippPass Tail;
			Tail.Resource = Resource;
			Tail.Exit = NodeOf(Pass.Exit);
			Tail.Via = NodeOf(Pass.Via);
			Tail.EnterAt = -1.f;
			Tail.ClearAt = Pass.ClearAt;
			Out.Tail.Add(Tail);
		}
	}
	// Along one line the order is fixed: a train ahead on this route comes first wherever its bookings meet this
	// route, and one behind has to pass this train before it gets to the tracks both routes share from here on.
	for (const TPair<const AFGTrain*, FATTrafficTrain*>& Pair : mStateOf)
	{
		const FATTrafficTrain& Other = *Pair.Value;
		const int32* OtherSlot = SlotOf.Find(Pair.Key);
		if (!OtherSlot || Pair.Key == State.Train.Get())
		{
			continue;
		}
		if (FrontIndexOn(State, Other) != INDEX_NONE)
		{
			Out.First.Add(*OtherSlot);
			continue;
		}
		if (State.MovedAheadOf.Contains(Other.Train))
		{
			Out.MovedAheadOf.Add(*OtherSlot);
		}
		const int32 SelfAt = FrontIndexOn(Other, State);
		if (SelfAt == INDEX_NONE)
		{
			continue;
		}
		TSet<int32>& Keys = Out.Behind.FindOrAdd(*OtherSlot);
		for (int32 Step = 1; Step < State.Route.Num() && SelfAt + Step < Other.Route.Num() && Other.Route[SelfAt + Step].Track == State.Route[Step].Track; ++Step)
		{
			FResourceKey TrackKey;
			TrackKey.Track = State.Route[Step].Track;
			Keys.Add(ResourceOf(TrackKey));
			FResourceKey JointKey;
			JointKey.Joint = GetJoint(State.Route[Step - 1].Exit);
			if (JointKey.Joint != INDEX_NONE)
			{
				Keys.Add(ResourceOf(JointKey));
			}
		}
	}
}

void AATTrafficSubsystem::StartPlanCycle(double Now, float DeltaSeconds, bool bWait)
{
	if (!mSearchGraph.IsValid())
	{
		return;
	}
	const FATSippGraph& Graph = *mSearchGraph;
	// Turns: trains the mod doesn't drive first (they only stand), docked trains by departure, then self-driving trains
	// in rank order.
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
				State.MovedAheadOf.Reset();
			}
			SelfDriving.Add(&State);
			continue;
		}
		FTurn Turn;
		Turn.State = &State;
		if (Train->IsDocked())
		{
			Turn.Kind = 1;
			Turn.Order = State.DockedAt >= 0.0 ? State.DockedAt + GetExpectedStay(Train, State.DockStation.Get(), &State.DockRules, State.DockTransfer) - Now : 0.f;
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

	const TSharedRef<FATSippCycle, ESPMode::ThreadSafe> Cycle = MakeShared<FATSippCycle, ESPMode::ThreadSafe>();
	Cycle->Graph = mSearchGraph;
	Cycle->Now = Now;
	Cycle->Headway = GetSafetyMarginSeconds();
	// Retries end by half the update interval, so the cycle is in by the next update.
	Cycle->MaxRetries = FMath::RoundToInt(mSettings.PlanRetries);
	Cycle->RetrySeconds = mSettings.UpdateIntervalSeconds * 0.5;
	Cycle->MaxStates = FMath::RoundToInt(mSettings.PlanSearchLimit);
	TMap<const AFGTrain*, int32> SlotOf;
	for (const FTurn& Turn : Turns)
	{
		SlotOf.Add(Turn.State->Train.Get(), Cycle->Owners.Num());
		Cycle->Owners.Add(Turn.State->Train);
	}
	auto ResourceOf = [&Graph](const FResourceKey& Key)
	{
		if (Key.Joint != INDEX_NONE)
		{
			return Graph.JointFoul.IsValidIndex(Key.Joint) ? Graph.JointResource(Key.Joint) : INDEX_NONE;
		}
		const int32* Track = Graph.TrackIndex.Find(Key.Track);
		return Track ? *Track : INDEX_NONE;
	};
	auto NodeOf = [&Graph](const UFGRailroadTrackConnectionComponent* Connection)
	{
		const int32* Node = Connection ? Graph.ConnectionIndex.Find(Connection) : nullptr;
		return Node ? *Node : INDEX_NONE;
	};
	Cycle->Trains.SetNum(Turns.Num());
	for (int32 Slot = 0; Slot < Turns.Num(); ++Slot)
	{
		FATTrafficTrain& State = *Turns[Slot].State;
		FATSippTrain& Out = Cycle->Trains[Slot];
		AFGTrain* Train = State.Train.Get();
		Out.Rank = State.PlanRank;
		if (Turns[Slot].Kind == 2)
		{
			FillSippTrain(State, Graph, SlotOf, Out);
			Out.Turn = EATSippTurn::Plan;
			Out.Kind = EATSippKind::Moving;
			Out.bRanked = true;
			Out.MinDepart = Now;
			FTrainDockingRuleSet Rules;
			const AFGRailroadTimeTable* TimeTable = Train ? Train->GetTimeTable() : nullptr;
			Out.GoalDwell = GetExpectedStay(Train, State.GoalStation.Get(), GetStopRules(Train, TimeTable ? TimeTable->GetCurrentStop() : INDEX_NONE, State.GoalStation.Get(), Rules), -1);
			Out.Commit = GetRouteCommit(State);
			Out.bSearch = IsSearchDue(State, DeltaSeconds) && Out.Commit < Out.Route.Num() - 2;
			Out.MinGainSeconds = mSettings.RerouteMinGainSeconds;
			Out.MinGainPercent = mSettings.RerouteMinGainPercent;
			continue;
		}
		if (Turns[Slot].Kind == 1)
		{
			const double Start = State.DockedAt >= 0.0 ? State.DockedAt : Now;
			const double Depart = FMath::Max(Start + GetExpectedStay(Train, State.DockStation.Get(), &State.DockRules, State.DockTransfer), Now + OverdueDepartSeconds);
			if (State.PredictedRoute.Num() == 0 && Now - State.PredictedAt > PredictRetrySeconds && Train && Train->IsSelfDrivingEnabled())
			{
				PredictDeparture(State, Now);
			}
			if (State.PredictedRoute.Num() > 0)
			{
				// The next leg, planned like a self-driving train standing at its start until it leaves. It goes before
				// what self-driving trains have booked but don't hold yet: they plan around it at their turn, so none
				// queues where the departing train needs to pass.
				FATTrafficTrain Leaving = State;
				Leaving.Route = State.PredictedRoute;
				Leaving.FrontAhead = State.PredictedFrontAhead;
				Leaving.Speed = 0.f;
				IndexRoute(Leaving);
				FillSippTrain(Leaving, Graph, SlotOf, Out);
				Out.Turn = EATSippTurn::Plan;
				Out.Kind = EATSippKind::Predicted;
				Out.bRanked = false;
				Out.MinDepart = Depart;
				FTrainDockingRuleSet Rules;
				Out.GoalDwell = GetExpectedStay(Train, State.PredictedStation.Get(), GetStopRules(Train, State.PredictedStop, State.PredictedStation.Get(), Rules), -1);
				// A departure that finds no way searches the whole network for another, at most every route search while stuck.
				State.RerouteTimer -= DeltaSeconds;
				Out.Commit = 0;
				Out.bSearch = mRerouteEnabled && State.bPredictFailed && State.RerouteTimer <= 0.f && Out.Route.Num() > 2;
				Out.MinGainSeconds = mSettings.RerouteMinGainSeconds;
				Out.MinGainPercent = mSettings.RerouteMinGainPercent;
				if (Out.bSearch)
				{
					State.RerouteTimer = mSettings.RouteSearchStuckSeconds;
				}
				continue;
			}
			Out.Kind = EATSippKind::Predicted;
			Out.StandUntil = Depart;
		}
		else
		{
			Out.Kind = EATSippKind::Standing;
			Out.StandUntil = ATSippNever;
		}
		Out.Turn = EATSippTurn::Standing;
		for (const AFGBuildableRailroadTrack* Track : State.Occupied)
		{
			FResourceKey Key;
			Key.Track = Track;
			Out.Standing.Add(ResourceOf(Key));
		}
		for (const int32 Joint : State.OccupiedJoints)
		{
			FResourceKey Key;
			Key.Joint = Joint;
			Out.Standing.Add(ResourceOf(Key));
		}
		Out.Standing.Remove(INDEX_NONE);
	}
	// The bookings as they are: what trains later in the order keep from earlier ones until their turn.
	Cycle->Table.Init(Graph.NumResources(), Turns.Num());
	for (const TPair<FResourceKey, TArray<FBooking>>& Pair : mBookings)
	{
		const int32 Resource = ResourceOf(Pair.Key);
		if (Resource == INDEX_NONE)
		{
			continue;
		}
		for (const FBooking& Booking : Pair.Value)
		{
			const int32* Slot = SlotOf.Find(Booking.Owner);
			if (!Slot)
			{
				continue;
			}
			FATSippBooking Copy;
			Copy.Owner = *Slot;
			Copy.Enter = Booking.Enter;
			Copy.Clear = IsOpen(Booking.Clear) ? ATSippNever : Booking.Clear;
			Copy.Exit = NodeOf(Booking.Exit);
			Copy.Via = NodeOf(Booking.Via);
			Copy.Kind = static_cast<EATSippKind>(Booking.Kind);
			Copy.bLocked = Booking.bLocked;
			Copy.bHeld = Booking.bHeld;
			Copy.bCommitted = Booking.bCommitted;
			Cycle->Table.Add(Resource, Copy);
		}
	}
	mCycle = Cycle;
	if (bWait)
	{
		ATSipp::Run(*Cycle);
		ApplyPlanCycle();
		return;
	}
	ATSipp::Launch(Cycle);
}

void AATTrafficSubsystem::ApplyPlanCycle()
{
	const TSharedPtr<FATSippCycle, ESPMode::ThreadSafe> Cycle = MoveTemp(mCycle);
	mCycle.Reset();
	if (!Cycle.IsValid() || !Cycle->Graph.IsValid())
	{
		return;
	}
	const FATSippGraph& Graph = *Cycle->Graph;
	++mCycles;
	mCycleMilliseconds += Cycle->Milliseconds;
	// The cycle's bookings replace all others.
	mBookings.Reset();
	mOwnerBookings.Reset();
	for (int32 Resource = 0; Resource < Cycle->Table.ByResource.Num(); ++Resource)
	{
		FResourceKey Key;
		if (Resource < Graph.Length.Num())
		{
			Key.Track = Graph.Tracks[Resource];
		}
		else
		{
			Key.Joint = Resource - Graph.Length.Num();
		}
		for (const FATSippBooking& Booking : Cycle->Table.ByResource[Resource])
		{
			const AFGTrain* Booker = Cycle->Owners.IsValidIndex(Booking.Owner) ? Cycle->Owners[Booking.Owner].Get() : nullptr;
			if (!Booker)
			{
				continue;
			}
			FBooking Copy;
			Copy.Owner = Booker;
			Copy.Enter = Booking.Enter;
			Copy.Clear = Booking.Clear;
			Copy.Exit = Graph.Connections.IsValidIndex(Booking.Exit) ? Graph.Connections[Booking.Exit].Get() : nullptr;
			Copy.Via = Graph.Connections.IsValidIndex(Booking.Via) ? Graph.Connections[Booking.Via].Get() : nullptr;
			Copy.Kind = static_cast<EBookingKind>(Booking.Kind);
			Copy.bLocked = Booking.bLocked;
			Copy.bHeld = Booking.bHeld;
			Copy.bCommitted = Booking.bCommitted;
			mBookings.FindOrAdd(Key).Add(Copy);
			FOwnerBookings& Own = mOwnerBookings.FindOrAdd(FObjectKey(Booker));
			Own.Owner = Booker;
			Own.Keys.Add(Key);
		}
	}
	// Trains moved up after a failed plan stay before the train in their way for the rest of their leg.
	for (const TPair<int32, int32>& Move : Cycle->Reordered)
	{
		FATTrafficTrain* Moved = mTrains.Find(Cycle->Owners[Move.Key]);
		FATTrafficTrain* Passed = mTrains.Find(Cycle->Owners[Move.Value]);
		if (!Moved || !Passed)
		{
			continue;
		}
		Moved->LegStart = FMath::Min(Moved->LegStart, Passed->LegStart - 0.001);
		Moved->MovedAheadOf.Add(Passed->Train);
		++mPlanReorders;
		UE_LOG(LogAutoTrains, Log, TEXT("planner: %s is planned before %s from now on: its plan found no way past %s's booking"), *NameOf(Moved->Train.Get()),
			*NameOf(Passed->Train.Get()), *NameOf(Passed->Train.Get()));
	}
	const FString Limited = Cycle->Owners.IsValidIndex(Cycle->RetryLimited) ? NameOf(Cycle->Owners[Cycle->RetryLimited].Get()) : FString();
	if (Limited != mLastRetryLimited && !Limited.IsEmpty())
	{
		UE_LOG(LogAutoTrains, Log, TEXT("planner: planning retry limit reached (%d retries, %.1f ms): %s stays held up by a train planned before it"),
			Cycle->Reordered.Num(), Cycle->Milliseconds, *Limited);
	}
	mLastRetryLimited = Limited;
	for (int32 Slot = 0; Slot < Cycle->Trains.Num(); ++Slot)
	{
		const FATSippTrain& Input = Cycle->Trains[Slot];
		FATTrafficTrain* State = mTrains.Find(Cycle->Owners[Slot]);
		if (State && Input.Kind == EATSippKind::Moving)
		{
			State->PlanRank = Input.Rank;
		}
		if (State && State->bManaged && Input.Turn == EATSippTurn::Plan && Input.Kind == EATSippKind::Moving && Cycle->Results[Slot].bPlanned)
		{
			ApplyPlan(*State, *Cycle, Slot);
		}
		if (State && Input.Turn == EATSippTurn::Plan && Input.Kind == EATSippKind::Predicted && Cycle->Results[Slot].bPlanned && State->PredictedRoute.Num() > 0)
		{
			State->bPredictFailed = Cycle->Results[Slot].bFailed;
			if (Cycle->Results[Slot].bRerouted)
			{
				ApplyDepartureReroute(*State, *Cycle, Slot);
			}
		}
	}
}

void AATTrafficSubsystem::ApplyPlan(FATTrafficTrain& State, const FATSippCycle& Cycle, int32 Slot)
{
	const FATSippGraph& Graph = *Cycle.Graph;
	const FATSippResult& Result = Cycle.Results[Slot];
	const double Now = Cycle.Now;
	AFGTrain* Train = State.Train.Get();
	auto TrainOf = [&Cycle](int32 Index) -> AFGTrain* { return Cycle.Owners.IsValidIndex(Index) ? Cycle.Owners[Index].Get() : nullptr; };
	auto Name = [&Graph](int32 Resource)
	{
		return !Graph.Length.IsValidIndex(Resource) ? FString::Printf(TEXT("switch %d"), Resource - Graph.Length.Num()) : ResourceName(Graph.Tracks[Resource], INDEX_NONE);
	};
	if (Result.bRerouted)
	{
		ApplyReroute(State, Cycle, Slot);
	}
	++mPlans;
	State.bHasPlan = true;
	State.PlanTrackEnter.Reset();
	State.PlanJointEnter.Reset();
	State.PlanTrackLocked.Reset();
	State.PlanJointLocked.Reset();
	for (const FATSippPass& Pass : Result.Passes)
	{
		if (Pass.Step == INDEX_NONE || IsOpen(Pass.Enter))
		{
			continue;
		}
		if (Graph.Length.IsValidIndex(Pass.Resource))
		{
			State.PlanTrackEnter.Add(Graph.Tracks[Pass.Resource], Pass.Enter);
			if (Pass.bLocked)
			{
				State.PlanTrackLocked.Add(Graph.Tracks[Pass.Resource]);
			}
		}
		else
		{
			const int32 Joint = Pass.Resource - Graph.Length.Num();
			State.PlanJointEnter.Add(Joint, Pass.Enter);
			if (Pass.bLocked)
			{
				State.PlanJointLocked.Add(Joint);
			}
		}
	}
	const bool bWaits = Result.WaitStep != INDEX_NONE;
	State.bPlanWaitHere = bWaits && Result.WaitStep < 0;
	State.PlanWaitExit = bWaits && Result.Route.IsValidIndex(Result.WaitStep) ? Graph.Connections[Result.Route[Result.WaitStep]] : TWeakObjectPtr<UFGRailroadTrackConnectionComponent>();
	State.PlanDepart = bWaits ? Result.WaitUntil : -1.0;
	State.PlanWaitFor = TrainOf(Result.WaitFor);
	State.PlanArrival = Result.bFailed ? OpenTime : Result.Arrival;
	if (State.SearchedArrival < 0.0 && !IsOpen(State.PlanArrival))
	{
		State.SearchedArrival = State.PlanArrival;
	}
	mPlanWaits += bWaits ? 1 : 0;

	FString Failure;
	if (Result.bFailed)
	{
		Failure = Result.BlockedAt != INDEX_NONE ? FString::Printf(TEXT("no way on past %s for %s"), *Name(Result.BlockedAt), *NameOf(TrainOf(Result.BlockedBy)))
			: FString(TEXT("no way to its goal"));
	}
	if (Failure != State.PlanFailure)
	{
		if (!Failure.IsEmpty())
		{
			++mPlanFailures;
			UE_LOG(LogAutoTrains, Log, TEXT("planner: %s: %s; it stands before it until that train has passed (%d states; left out: %d moves out of its reach, %d tail windows missed, %d stays at the goal that don't fit)"),
				*NameOf(Train), *Failure, Result.States, Result.RejectedMotion, Result.RejectedTail, Result.RejectedStay);
		}
		State.PlanFailure = Failure;
	}
	const FString Committed = Result.CommittedWith != INDEX_NONE ? FString::Printf(TEXT("%s at %s"), *NameOf(TrainOf(Result.CommittedWith)), *Name(Result.CommittedAt)) : FString();
	if (Committed != State.PlanCommitted)
	{
		// Trains passing through a deadlock run into others on purpose.
		if (!Committed.IsEmpty() && State.GhostUntil < 0.0)
		{
			++mPlanCommitted;
			UE_LOG(LogAutoTrains, Warning, TEXT("planner: %s can't stop short of %s any more, which %s has booked at the same time"), *NameOf(Train),
				*Name(Result.CommittedAt), *NameOf(TrainOf(Result.CommittedWith)));
		}
		State.PlanCommitted = Committed;
	}
	State.PlanNote = FString::Printf(TEXT("at its goal in %s"), *SecondsText(State.PlanArrival - Now));
	if (bWaits)
	{
		const FString Where = State.bPlanWaitHere ? FString(TEXT("where it stands"))
			: FString::Printf(TEXT("at the end of %s"), *Name(Graph.TrackOf[Result.Route[Result.WaitStep]]));
		State.PlanNote += FString::Printf(TEXT(", plan waits %s for %s until %s from now"), *Where, *NameOf(TrainOf(Result.WaitFor)), *SecondsText(Result.WaitUntil - Now));
	}
}

void AATTrafficSubsystem::ApplyReroute(FATTrafficTrain& State, const FATSippCycle& Cycle, int32 Slot)
{
	AFGTrain* Train = State.Train.Get();
	const FATSippGraph& Graph = *Cycle.Graph;
	const FATSippTrain& Input = Cycle.Trains[Slot];
	const FATSippResult& Result = Cycle.Results[Slot];
	if (!Train || !Train->mAtcData.Path.IsValid() || !Input.Route.IsValidIndex(Input.Commit))
	{
		return;
	}
	// Only while the train is still on the route planned from, and can still change it after the planned commit.
	const UFGRailroadTrackConnectionComponent* CommitExit = Graph.Connections[Input.Route[Input.Commit]].Get();
	int32 Commit = INDEX_NONE;
	for (int32 Index = 0; Index < State.Route.Num() && Commit == INDEX_NONE; ++Index)
	{
		Commit = State.Route[Index].Exit == CommitExit ? Index : INDEX_NONE;
	}
	if (Commit == INDEX_NONE || Commit < GetRouteCommit(State) || State.Route.Num() - Commit != Input.Route.Num() - Input.Commit)
	{
		return;
	}
	for (int32 Index = 1; Commit + Index < State.Route.Num(); ++Index)
	{
		if (State.Route[Commit + Index].Exit != Graph.Connections[Input.Route[Input.Commit + Index]].Get())
		{
			return;
		}
	}
	TArray<UFGRailroadTrackConnectionComponent*> Connections;
	for (int32 Index = 0; Index <= Commit; ++Index)
	{
		Connections.Add(State.Route[Index].Exit);
	}
	for (int32 Index = Input.Commit + 1; Index < Result.Route.Num(); ++Index)
	{
		UFGRailroadTrackConnectionComponent* Connection = Graph.Connections[Result.Route[Index]].Get();
		if (!Connection || !Connection->GetTrack())
		{
			return;
		}
		Connections.Add(Connection);
	}
	UFGRailroadTrackConnectionComponent* Goal = Connections.Last();
	const FRailroadPathSharedPtr Path = FATRouter::BuildPath(Connections, Train->mAtcData.Path->Station.Get());
	const bool bApplied = FATRouter::ApplyPath(Train, Path, Goal);
	++State.Reroutes;
	++mPlanReroutes;
	State.RerouteCooldown = mSettings.RerouteHoldSeconds;
	State.SearchedArrival = -1.0;
	UE_LOG(LogAutoTrains, Log, TEXT("planner: %s rerouted after %s: %s -> %.0f s to its goal, %d -> %d tracks, applied %s (route search: %d states, %.2f ms on a worker)"),
		*NameOf(Train), *ResourceName(State.Route[Commit].Track, INDEX_NONE), IsOpen(Result.OldArrival) ? TEXT("no way") : *FString::Printf(TEXT("%.0f s"), Result.OldArrival - Cycle.Now),
		Result.Arrival - Cycle.Now, Input.Route.Num() - Input.Commit - 1, Result.Route.Num() - Input.Commit - 1, bApplied ? TEXT("yes") : TEXT("NO"), Result.SearchStates,
		Result.SearchMilliseconds);
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
	for (int32 Index = Commit + 1; Index < Connections.Num(); ++Index)
	{
		AFGBuildableRailroadTrack* Track = Connections[Index]->GetTrack();
		Distance += Track->GetLength();
		Route.Add({Track, Connections[Index], Distance});
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
	if (mCycle.IsValid() && mCycle->bDone.load(std::memory_order_acquire))
	{
		ApplyPlanCycle();
	}

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

	// Movement authority from the bookings, in planning order.
	TArray<FATTrafficTrain*> Driving;
	for (const TPair<const AFGTrain*, FATTrafficTrain*>& Pair : mStateOf)
	{
		if (Pair.Value->bManaged && Pair.Value->Route.Num() > 0)
		{
			Driving.Add(Pair.Value);
		}
	}
	// Before the first cycle is in, no train would be given track: the first one is planned at once, on this thread.
	if (!mPlannedOnce && Driving.ContainsByPredicate([](const FATTrafficTrain* State) { return !State->bHasPlan; }))
	{
		mPlannedOnce = true;
		mCycle.Reset();
		StartPlanCycle(Now, mSinceCycle + DeltaSeconds, true);
		mSinceCycle = 0.f;
	}
	Driving.Sort([](const FATTrafficTrain& A, const FATTrafficTrain& B)
	{
		return static_cast<uint32>(A.PlanRank) < static_cast<uint32>(B.PlanRank);
	});
	for (FATTrafficTrain* State : Driving)
	{
		GrantFromPlan(*State, Now);
	}

	// Check: no two trains hold conflicting resources, apart from following the same way or passing through a deadlock.
	for (const FATTrafficTrain* Checked : Driving)
	{
		const FATTrafficTrain& State = *Checked;
		if (State.GhostUntil >= 0.0)
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
	for (const TPair<const AFGTrain*, FATTrafficTrain*>& Checked : mStateOf)
	{
		const FATTrafficTrain& State = *Checked.Value;
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
		for (const FATTrafficTrain* State : Driving)
		{
			if (State->WaitingSeconds > WaitDumpSeconds)
			{
				LogWaitDump(*State, Now);
			}
		}
	}

	// The next planning cycle, once the last one is in.
	mSinceCycle += DeltaSeconds;
	if (!mCycle.IsValid())
	{
		StartPlanCycle(Now, mSinceCycle, false);
		mSinceCycle = 0.f;
	}
	else
	{
		++mCyclesLate;
	}

	++mPlanUpdates;
	const double Took = FPlatformTime::Seconds() - Begin;
	mPlanSeconds += Took;
	if (Took > SlowUpdateSeconds && Now >= mPlanSlowLoggedAt + SlowLogSeconds)
	{
		UE_LOG(LogAutoTrains, Warning, TEXT("planner: an update took %.1f ms (%d trains)"), Took * 1000.0, mStateOf.Num());
		mPlanSlowLoggedAt = Now;
	}
	if (Now >= mPlanSummaryAt)
	{
		int32 Bookings = 0;
		for (const TPair<FResourceKey, TArray<FBooking>>& Pair : mBookings)
		{
			Bookings += Pair.Value.Num();
		}
		UE_LOG(LogAutoTrains, Log, TEXT("planner: last %.0f s: %d trains (%d driving), %d bookings now, %.2f ms per update; %d cycles, %.1f ms each on a worker, %d updates waited for one, %d trains moved up after a failed plan; %d plans, %d with a wait, %d failed, %d committed conflicts, %d reroutes, %d departures predicted"),
			PlanSummarySeconds, mStateOf.Num(), Driving.Num(), Bookings, mPlanUpdates > 0 ? mPlanSeconds * 1000.0 / mPlanUpdates : 0.0, mCycles,
			mCycles > 0 ? mCycleMilliseconds / mCycles : 0.0, mCyclesLate, mPlanReorders, mPlans, mPlanWaits, mPlanFailures, mPlanCommitted, mPlanReroutes, mPlanPredictions);
		mPlans = mPlanWaits = mPlanFailures = mPlanCommitted = mPlanReroutes = mPlanPredictions = mPlanUpdates = mCycles = mCyclesLate = mPlanReorders = 0;
		mCycleMilliseconds = 0.0;
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
			FATFollowInfo Info;
			GetFollowLimit(Train, *Other, HardLimit, &Info);
			const float Authority = GetAuthorityDistance(Train);
			Drive = FString::Printf(TEXT(", may drive %s, target %.0f km/h%s"), Authority < 0.f ? TEXT("to its goal") : *FString::Printf(TEXT("%.0f m"), Authority / 100.f),
				Train->mAtcData.TargetSpeed * 0.036f,
				HardLimit >= 0.f ? *FString::Printf(TEXT(", follows %s with %.0f m"), *NameOf(Info.Leader.Get()), HardLimit / 100.f) : TEXT(""));
		}
		return FString::Printf(TEXT("%s (%s, rank %d, %.0f km/h, waited %.0f s, %d claims%s%s%s)"), *NameOf(Train), Other->bManaged ? TEXT("driving") : TEXT("not driven"),
			Other->PlanRank, Other->Speed * 0.036f, Other->WaitingSeconds, Other->Claims.Num(), *Drive, Other->BlockedReason.IsEmpty() ? TEXT("") : TEXT(", "), *Other->BlockedReason);
	};
	const bool bJoint = State.BlockedAtJoint != INDEX_NONE;
	const double* Enter = bJoint ? State.PlanJointEnter.Find(State.BlockedAtJoint) : State.PlanTrackEnter.Find(State.BlockedAtTrack);
	const bool bLocked = bJoint ? State.PlanJointLocked.Contains(State.BlockedAtJoint) : State.PlanTrackLocked.Contains(State.BlockedAtTrack);
	FString Lines = FString::Printf(TEXT("planner: WAIT %s: %s; %s; its booking there %s%s"), *Describe(&State), *State.PlanNote, *ResourceName(State.BlockedAtTrack, State.BlockedAtJoint),
		Enter ? *FString::Printf(TEXT("enters in %s"), *SecondsText(*Enter - Now)) : TEXT("missing"), bLocked ? TEXT(", locked") : TEXT(""));
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
