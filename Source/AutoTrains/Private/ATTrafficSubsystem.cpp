#include "ATTrafficSubsystem.h"
#include "ATControlSubsystem.h"
#include "ATRouter.h"
#include "AutoTrains.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "EngineUtils.h"
#include "FGLocomotive.h"
#include "FGRailroadSubsystem.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "FGRailroadVehicle.h"
#include "FGTrain.h"
#include "Subsystem/SubsystemActorManager.h"

namespace
{
	constexpr float TrafficUpdateInterval = 0.25f;
	// Claims reach this far past the braking distance plus the train's length. [cm]
	constexpr float ClaimLookahead = 20000.f;
	// Trains stop this far before the end of their authority. [cm]
	constexpr float AuthorityStopMargin = 1000.f;
	// Other trains' routes count for stopping spots up to this far ahead of them. [cm]
	constexpr float OthersRouteHorizon = 300000.f;
	// Planning deceleration: shares of the measured full-brake deceleration and of the consist's rating. Loaded trains
	// were measured braking at 2/3 of their rating, light ones at 1.4x.
	constexpr float MeasuredBrakeShare = 0.85f;
	constexpr float RatedBrakeShare = 0.6f;
	// A train over its braking curve by more than this gets full brake. [cm/s]
	constexpr float OverspeedTolerance = 100.f;
	constexpr double ConflictRebuildSeconds = 30.0;
	constexpr double ConflictRebuildMinSeconds = 2.0;

	// How often each train's route is checked for congestion. [s]
	constexpr float RerouteInterval = 5.f;
	// Congestion costs, as extra track length. [cm]
	constexpr float StandingTrainPenalty = 150000.f;
	constexpr float MovingTrainPenalty = 40000.f;
	constexpr float ClaimPenalty = 30000.f;
	constexpr float UpcomingSameWayPenalty = 5000.f;
	constexpr float UpcomingOtherWayPenalty = 80000.f;
	// A new route must be cheaper than the current one by this much and this share to be taken.
	constexpr float RerouteMinGain = 30000.f;
	constexpr float RerouteMinGainShare = 0.15f;

	FString TrackName(const AFGBuildableRailroadTrack* Track)
	{
		return GetNameSafe(Track).Replace(TEXT("Build_RailroadTrack_C_"), TEXT("track "));
	}

	FString TrainName(const AFGTrain* Train)
	{
		return Train ? Train->GetTrainName().ToString() : TEXT("?");
	}

	// Adds the tracks attached at Connection.
	void AddAttachedTracks(const UFGRailroadTrackConnectionComponent* Connection, TSet<AFGBuildableRailroadTrack*>& Out)
	{
		if (!Connection)
		{
			return;
		}
		for (const UFGRailroadTrackConnectionComponent* Other : Connection->GetConnections())
		{
			if (Other && Other->GetTrack())
			{
				Out.Add(Other->GetTrack());
			}
		}
	}
}

AATTrafficSubsystem::AATTrafficSubsystem()
{
	ReplicationPolicy = ESubsystemReplicationPolicy::SpawnOnServer;
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
}

AATTrafficSubsystem* AATTrafficSubsystem::Get(const UObject* WorldContext)
{
	UWorld* World = WorldContext ? WorldContext->GetWorld() : nullptr;
	USubsystemActorManager* Manager = World ? World->GetSubsystem<USubsystemActorManager>() : nullptr;
	return Manager ? Manager->GetSubsystemActor<AATTrafficSubsystem>() : nullptr;
}

void AATTrafficSubsystem::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!mEnabled)
	{
		return;
	}
	mUpdateTimer -= DeltaSeconds;
	if (mUpdateTimer <= 0.f)
	{
		mUpdateTimer += TrafficUpdateInterval;
		UpdateTraffic(TrafficUpdateInterval);
	}
}

FString AATTrafficSubsystem::SetEnabled(bool bEnabled)
{
	mEnabled = bEnabled;
	mTrains.Reset();
	mUpdateTimer = 0.f;
	if (bEnabled)
	{
		RebuildConflicts();
	}
	const FString Message = bEnabled ? TEXT("Traffic control on: self-driving trains ignore signals and claim track ahead.") : TEXT("Traffic control off.");
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
	return Message;
}

FString AATTrafficSubsystem::SetRerouteEnabled(bool bEnabled)
{
	mRerouteEnabled = bEnabled;
	const FString Message = bEnabled ? TEXT("Congestion rerouting on.") : TEXT("Congestion rerouting off.");
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
	return Message;
}

bool AATTrafficSubsystem::IsManaged(const AFGTrain* Train) const
{
	const FATTrafficTrain* State = mEnabled ? mTrains.Find(Train) : nullptr;
	return State && State->bManaged;
}

void AATTrafficSubsystem::UpdateTraffic(float DeltaSeconds)
{
	const double Now = FPlatformTime::Seconds();
	if (Now - mConflictsBuiltAt > ConflictRebuildSeconds || (mConflictsStale && Now - mConflictsBuiltAt > ConflictRebuildMinSeconds))
	{
		RebuildConflicts();
	}

	TArray<AFGTrain*> Trains;
	if (AFGRailroadSubsystem* Railroad = AFGRailroadSubsystem::Get(GetWorld()))
	{
		Railroad->GetAllTrains(Trains);
	}

	TSet<TWeakObjectPtr<AFGTrain>> Seen;
	for (AFGTrain* Train : Trains)
	{
		if (!IsValid(Train))
		{
			continue;
		}
		Seen.Add(Train);
		FATTrafficTrain& State = mTrains.FindOrAdd(Train);
		ReadTrain(State, Train);

		// Measure full-brake deceleration between updates; docking catches and crashes are far above 20 m/s^2.
		if (State.bFullBrake && State.LastSpeed > 500.f && State.Speed > 300.f)
		{
			const float Deceleration = (State.LastSpeed - State.Speed) / DeltaSeconds;
			if (Deceleration > 50.f && Deceleration < 2000.f)
			{
				const float Previous = State.MeasuredBrake;
				State.MeasuredBrake = Previous > 0.f ? Previous * 0.7f + Deceleration * 0.3f : Deceleration;
				if (Previous <= 0.f)
				{
					UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s brakes at %.2f m/s2 (rated %.2f); planning with %.2f"), *TrainName(Train),
						State.MeasuredBrake / 100.f, Train->mConsistData.MaxBrakeDeceleration / 100.f, GetPlanningDeceleration(State) / 100.f);
				}
			}
		}
		State.LastSpeed = State.Speed;
	}
	for (auto It = mTrains.CreateIterator(); It; ++It)
	{
		if (!Seen.Contains(It.Key()))
		{
			It.RemoveCurrent();
		}
	}

	TArray<FATTrafficTrain*> Managed;
	for (TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		ReleasePassedClaims(Pair.Value);
		if (Pair.Value.bManaged)
		{
			Managed.Add(&Pair.Value);
		}
	}

	// The train that has waited longest claims first; ties go by name so the order is stable.
	Managed.Sort([](const FATTrafficTrain& A, const FATTrafficTrain& B)
	{
		if (A.WaitingSeconds != B.WaitingSeconds)
		{
			return A.WaitingSeconds > B.WaitingSeconds;
		}
		return TrainName(A.Train.Get()) < TrainName(B.Train.Get());
	});
	for (FATTrafficTrain* State : Managed)
	{
		const FString PreviousReason = State->BlockedReason;
		ExtendClaims(*State);
		UpdateAuthority(*State);
		if (State->BlockedReason != PreviousReason && !State->BlockedReason.IsEmpty())
		{
			UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s: %s"), *TrainName(State->Train.Get()), *State->BlockedReason);
		}

		if (mRerouteEnabled)
		{
			if (State->RerouteTimer < 0.f)
			{
				// Spread the checks out over the interval.
				State->RerouteTimer = FMath::FRandRange(0.f, RerouteInterval);
			}
			State->RerouteTimer -= DeltaSeconds;
			if (State->RerouteTimer <= 0.f)
			{
				State->RerouteTimer = RerouteInterval;
				ConsiderReroute(*State);
			}
		}

		const float Distance = GetAuthorityDistance(State->Train.Get());
		const bool bHeld = !State->bAuthorityToGoal && Distance >= 0.f && Distance < AuthorityStopMargin + 5000.f;
		if (State->Speed < 50.f && bHeld)
		{
			State->WaitingSeconds += DeltaSeconds;
		}
		else if (State->Speed > 100.f)
		{
			State->WaitingSeconds = 0.f;
		}
	}
}

void AATTrafficSubsystem::ReadTrain(FATTrafficTrain& State, AFGTrain* Train) const
{
	State.Train = Train;
	State.Length = Train->mConsistData.Length;
	State.Speed = FMath::Abs(Train->mSimulationData.Velocity);
	State.Route.Reset();
	State.Occupied.Reset();
	State.FrontAhead = 0.f;

	const FTrainAtcData& Atc = Train->mAtcData;
	State.bManaged = Train->IsSelfDrivingEnabled() && !Train->IsDocked() && Atc.Path.IsValid()
		&& Atc.Path->PathPoints.IsValidIndex(Atc.CurrentPathSegment);
	if (State.bManaged)
	{
		float Distance = Atc.CurrentConnectionDistance;
		for (int32 Index = Atc.CurrentPathSegment; Index < Atc.Path->PathPoints.Num(); ++Index)
		{
			UFGRailroadTrackConnectionComponent* Exit = Atc.Path->PathPoints[Index].TrackConnection.Get();
			AFGBuildableRailroadTrack* Track = Exit ? Exit->GetTrack() : nullptr;
			if (!Track)
			{
				break;
			}
			if (Index > Atc.CurrentPathSegment)
			{
				Distance += Track->GetLength();
			}
			State.Route.Add({Track, Exit, Distance});
		}
	}

	for (AFGRailroadSubsystem::TTrainIterator It(Train->GetFirstVehicle()); It; ++It)
	{
		const AFGRailroadVehicle* Vehicle = *It;
		const FRailroadTrackPosition& Position = Vehicle->GetTrackPosition();
		AFGBuildableRailroadTrack* Track = Position.Track.Get();
		if (!Track)
		{
			continue;
		}
		State.Occupied.Add(Track);
		// A vehicle near a track end overhangs onto the tracks attached there.
		const float Reach = Vehicle->GetLength() * 0.5f + 200.f;
		for (int32 End = 0; End < 2; ++End)
		{
			const UFGRailroadTrackConnectionComponent* Connection = Track->GetConnection(End);
			if (Connection && FMath::Abs(Connection->GetTrackPosition().Offset - Position.Offset) < Reach)
			{
				AddAttachedTracks(Connection, State.Occupied);
			}
		}
		// ATC distances are from the MU master; find how far the train's front is ahead of it.
		for (const FATRouteStep& Step : State.Route)
		{
			if (Step.Track == Track && Step.Exit)
			{
				const float ToExit = FMath::Abs(Step.Exit->GetTrackPosition().Offset - Position.Offset);
				State.FrontAhead = FMath::Max(State.FrontAhead, Step.EndDistance - ToExit + Vehicle->GetLength() * 0.5f);
				break;
			}
		}
	}

	if (!State.bManaged)
	{
		State.Claims.Reset();
		State.AuthorityTrack = nullptr;
		State.bAuthorityToGoal = false;
		State.BlockedReason.Reset();
		State.BlockedBy = nullptr;
		State.WaitingSeconds = 0.f;
		State.bFullBrake = false;
		State.bCatchArmed = false;
	}
}

void AATTrafficSubsystem::ReleasePassedClaims(FATTrafficTrain& State) const
{
	State.Claims.RemoveAll([&State](const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim)
	{
		AFGBuildableRailroadTrack* Track = Claim.Get();
		if (!Track)
		{
			return true;
		}
		if (State.Occupied.Contains(Track))
		{
			return false;
		}
		return !State.Route.ContainsByPredicate([Track](const FATRouteStep& Step) { return Step.Track == Track; });
	});
}

void AATTrafficSubsystem::ExtendClaims(FATTrafficTrain& State)
{
	AFGTrain* Train = State.Train.Get();
	if (!Train || State.Route.Num() == 0)
	{
		return;
	}
	const float Need = FMath::Square(State.Speed) / (2.f * GetPlanningDeceleration(State)) + State.Length + ClaimLookahead;

	int32 Index = 0;
	while (Index < State.Route.Num() && HoldsTrack(State, State.Route[Index].Track))
	{
		++Index;
	}
	if (Index > 0 && State.Route[Index - 1].EndDistance - State.FrontAhead >= Need)
	{
		return;
	}

	State.BlockedReason.Reset();
	State.BlockedBy = nullptr;
	TArray<AFGBuildableRailroadTrack*> Pending;
	FString StopReason;
	for (; Index < State.Route.Num(); ++Index)
	{
		const FATRouteStep& Step = State.Route[Index];
		const FATTrafficTrain* Blocker = nullptr;
		AFGBuildableRailroadTrack* Conflict = nullptr;
		bool bStandsOnIt = false;
		if (!IsTrackFree(Step.Track, State, Blocker, Conflict, bStandsOnIt))
		{
			State.BlockedBy = Blocker ? Blocker->Train : nullptr;
			State.BlockedReason = FString::Printf(TEXT("waits for %s, which %s %s%s, %.0f m ahead"),
				*TrainName(Blocker ? Blocker->Train.Get() : nullptr), bStandsOnIt ? TEXT("stands on") : TEXT("has claimed"), *TrackName(Conflict),
				Conflict == Step.Track ? TEXT("") : *FString::Printf(TEXT(" (conflicts with %s on this train's route)"), *TrackName(Step.Track)),
				(Step.EndDistance - Step.Track->GetLength() - State.FrontAhead) / 100.f);
			break;
		}
		Pending.Add(Step.Track);
		const bool bGoal = Index == State.Route.Num() - 1;
		if (bGoal || IsValidStop(State, Index, StopReason))
		{
			for (AFGBuildableRailroadTrack* Track : Pending)
			{
				State.Claims.Add(Track);
			}
			Pending.Reset();
			if (bGoal || Step.EndDistance - State.FrontAhead >= Need)
			{
				break;
			}
		}
	}
	if (Pending.Num() > 0 && State.BlockedReason.IsEmpty())
	{
		State.BlockedReason = FString::Printf(TEXT("no safe place to stop beyond %s: %s"), *TrackName(Pending[0]), *StopReason);
	}
	else if (Pending.Num() > 0)
	{
		State.BlockedReason += FString::Printf(TEXT("; can't stop before it: %s"), *StopReason);
	}
}

void AATTrafficSubsystem::UpdateAuthority(FATTrafficTrain& State) const
{
	State.AuthorityTrack = nullptr;
	State.bAuthorityToGoal = false;
	int32 Last = INDEX_NONE;
	for (int32 Index = 0; Index < State.Route.Num() && HoldsTrack(State, State.Route[Index].Track); ++Index)
	{
		Last = Index;
	}
	if (Last != INDEX_NONE)
	{
		State.AuthorityTrack = State.Route[Last].Track;
		State.bAuthorityToGoal = Last == State.Route.Num() - 1;
	}
}

bool AATTrafficSubsystem::IsTrackFree(const AFGBuildableRailroadTrack* Track, const FATTrafficTrain& Self, const FATTrafficTrain*& OutBlocker,
	AFGBuildableRailroadTrack*& OutConflict, bool& bOutStandsOnIt) const
{
	const TArray<AFGBuildableRailroadTrack*>& Conflicts = GetConflicts(Track);
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const FATTrafficTrain& Other = Pair.Value;
		if (&Other == &Self)
		{
			continue;
		}
		for (AFGBuildableRailroadTrack* Conflict : Conflicts)
		{
			if (HoldsTrack(Other, Conflict))
			{
				OutBlocker = &Other;
				OutConflict = Conflict;
				bOutStandsOnIt = Other.Occupied.Contains(Conflict);
				return false;
			}
		}
	}
	return true;
}

bool AATTrafficSubsystem::IsValidStop(const FATTrafficTrain& Self, int32 EndIndex, FString& OutReason) const
{
	const float FrontAtStop = Self.Route[EndIndex].EndDistance - Self.FrontAhead;
	const float TailAtStop = FrontAtStop - Self.Length;
	for (int32 Index = EndIndex; Index >= 0; --Index)
	{
		const FATRouteStep& Step = Self.Route[Index];
		const float TrackEnd = Step.EndDistance - Self.FrontAhead;
		if (TrackEnd <= TailAtStop)
		{
			break;
		}
		if (IsJunction(Step.Track))
		{
			OutReason = FString::Printf(TEXT("the train would stand on junction %s"), *TrackName(Step.Track));
			return false;
		}
		if (!IsOutOfOthersWay(Self, Step, OutReason))
		{
			return false;
		}
	}
	// The tail would still be on tracks behind the route; those are the ones the train occupies now.
	const float RouteStart = Self.Route[0].EndDistance - Self.Route[0].Track->GetLength() - Self.FrontAhead;
	if (TailAtStop < RouteStart)
	{
		for (AFGBuildableRailroadTrack* Track : Self.Occupied)
		{
			if (IsJunction(Track))
			{
				OutReason = FString::Printf(TEXT("the tail would stand on junction %s"), *TrackName(Track));
				return false;
			}
		}
	}
	return true;
}

bool AATTrafficSubsystem::IsOutOfOthersWay(const FATTrafficTrain& Self, const FATRouteStep& Step, FString& OutReason) const
{
	// Non-junction tracks conflict only with themselves, so only trains coming the other way on this track matter;
	// trains going the same way reach it from behind and queue.
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const FATTrafficTrain& Other = Pair.Value;
		if (&Other == &Self || !Other.bManaged)
		{
			continue;
		}
		for (const FATRouteStep& OtherStep : Other.Route)
		{
			if (OtherStep.EndDistance - Other.FrontAhead > OthersRouteHorizon)
			{
				break;
			}
			if (OtherStep.Track == Step.Track && OtherStep.Exit != Step.Exit)
			{
				OutReason = FString::Printf(TEXT("%s comes the other way over %s"), *TrainName(Other.Train.Get()), *TrackName(Step.Track));
				return false;
			}
		}
	}
	return true;
}

void AATTrafficSubsystem::BuildCongestion(const FATTrafficTrain& Self, TMap<const AFGBuildableRailroadTrack*, FCongestion>& Out) const
{
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const FATTrafficTrain& Other = Pair.Value;
		if (&Other == &Self)
		{
			continue;
		}
		const float StandPenalty = Other.Speed < 100.f ? StandingTrainPenalty : MovingTrainPenalty;
		for (const AFGBuildableRailroadTrack* Track : Other.Occupied)
		{
			for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Track))
			{
				Out.FindOrAdd(Conflict).Penalty += StandPenalty;
			}
		}
		for (const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim : Other.Claims)
		{
			const AFGBuildableRailroadTrack* Track = Claim.Get();
			if (Track && !Other.Occupied.Contains(const_cast<AFGBuildableRailroadTrack*>(Track)))
			{
				for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Track))
				{
					Out.FindOrAdd(Conflict).Penalty += ClaimPenalty;
				}
			}
		}
		if (Other.bManaged)
		{
			for (const FATRouteStep& Step : Other.Route)
			{
				if (Step.EndDistance - Other.FrontAhead > OthersRouteHorizon)
				{
					break;
				}
				Out.FindOrAdd(Step.Track).UpcomingExits.Add(Step.Exit);
			}
		}
	}
}

float AATTrafficSubsystem::GetCongestionCost(const TMap<const AFGBuildableRailroadTrack*, FCongestion>& Congestion, const AFGBuildableRailroadTrack* Track,
	const UFGRailroadTrackConnectionComponent* Exit)
{
	const FCongestion* Entry = Congestion.Find(Track);
	if (!Entry)
	{
		return 0.f;
	}
	float Cost = Entry->Penalty;
	for (const UFGRailroadTrackConnectionComponent* UpcomingExit : Entry->UpcomingExits)
	{
		Cost += UpcomingExit == Exit ? UpcomingSameWayPenalty : UpcomingOtherWayPenalty;
	}
	return Cost;
}

void AATTrafficSubsystem::ConsiderReroute(FATTrafficTrain& State)
{
	AFGTrain* Train = State.Train.Get();
	if (!Train || State.bAuthorityToGoal || State.Route.Num() < 3 || !Train->mAtcData.Path.IsValid())
	{
		return;
	}
	// The route can only change beyond the claims: the train may already be committed to those tracks and switches.
	int32 Commit = 0;
	for (int32 Index = 0; Index < State.Route.Num(); ++Index)
	{
		if (State.Route[Index].Track == State.AuthorityTrack.Get())
		{
			Commit = Index;
			break;
		}
	}
	if (Commit >= State.Route.Num() - 2)
	{
		return;
	}
	UFGRailroadTrackConnectionComponent* Start = State.Route[Commit].Exit;
	UFGRailroadTrackConnectionComponent* Goal = State.Route.Last().Exit;

	const double Begin = FPlatformTime::Seconds();
	TMap<const AFGBuildableRailroadTrack*, FCongestion> Congestion;
	BuildCongestion(State, Congestion);
	float CurrentCost = 0.f;
	float CurrentLength = 0.f;
	for (int32 Index = Commit + 1; Index < State.Route.Num(); ++Index)
	{
		CurrentLength += State.Route[Index].Track->GetLength();
		CurrentCost += State.Route[Index].Track->GetLength() + GetCongestionCost(Congestion, State.Route[Index].Track, State.Route[Index].Exit);
	}

	const TArray<UFGRailroadTrackConnectionComponent*> Alternative = FATRouter::FindRoute(Start, Goal, false,
		[&Congestion](const AFGBuildableRailroadTrack* Track, const UFGRailroadTrackConnectionComponent* Exit) { return GetCongestionCost(Congestion, Track, Exit); });
	if (Alternative.Num() < 2)
	{
		return;
	}
	bool bSame = Alternative.Num() == State.Route.Num() - Commit;
	float NewCost = 0.f;
	for (int32 Index = 1; Index < Alternative.Num(); ++Index)
	{
		const AFGBuildableRailroadTrack* Track = Alternative[Index]->GetTrack();
		NewCost += (Track ? Track->GetLength() : 0.f) + GetCongestionCost(Congestion, Track, Alternative[Index]);
		bSame = bSame && Alternative[Index] == State.Route[Commit + Index].Exit;
	}
	if (bSame || NewCost > CurrentCost - FMath::Max(RerouteMinGain, CurrentCost * RerouteMinGainShare))
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
	UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s rerouted after %s: cost %.0f -> %.0f m, length %.0f -> %.0f m, applied %s (%.2f ms)"),
		*TrainName(Train), *TrackName(State.Route[Commit].Track), CurrentCost / 100.f, NewCost / 100.f, CurrentLength / 100.f,
		FATRouter::GetRouteLength(Alternative) / 100.f, bApplied ? TEXT("yes") : TEXT("NO"), (FPlatformTime::Seconds() - Begin) * 1000.0);
}

void AATTrafficSubsystem::RebuildConflicts()
{
	const double Begin = FPlatformTime::Seconds();
	mConflicts.Reset();
	auto Entry = [this](AFGBuildableRailroadTrack* Track) -> TArray<AFGBuildableRailroadTrack*>&
	{
		TArray<AFGBuildableRailroadTrack*>& Conflicts = mConflicts.FindOrAdd(Track);
		if (Conflicts.Num() == 0)
		{
			Conflicts.Add(Track);
		}
		return Conflicts;
	};
	auto Link = [&Entry](AFGBuildableRailroadTrack* A, AFGBuildableRailroadTrack* B)
	{
		if (A && B && A != B)
		{
			Entry(A).AddUnique(B);
			Entry(B).AddUnique(A);
		}
	};

	int32 TrackCount = 0;
	int32 SwitchJoints = 0;
	for (TActorIterator<AFGBuildableRailroadTrack> It(GetWorld()); It; ++It)
	{
		AFGBuildableRailroadTrack* Track = *It;
		if (!IsValid(Track))
		{
			continue;
		}
		++TrackCount;
		Entry(Track);
		for (AFGBuildableRailroadTrack* Overlapping : Track->GetOverlappingTracks())
		{
			Link(Track, Overlapping);
		}
		// Every track meeting at a joint of three or more tracks fouls the others near it.
		for (int32 End = 0; End < 2; ++End)
		{
			const UFGRailroadTrackConnectionComponent* Connection = Track->GetConnection(End);
			if (!Connection)
			{
				continue;
			}
			TSet<AFGBuildableRailroadTrack*> Joint;
			Joint.Add(Track);
			for (const UFGRailroadTrackConnectionComponent* Facing : Connection->GetConnections())
			{
				if (!Facing)
				{
					continue;
				}
				if (Facing->GetTrack())
				{
					Joint.Add(Facing->GetTrack());
				}
				for (const UFGRailroadTrackConnectionComponent* Beside : Facing->GetConnections())
				{
					if (Beside && Beside->GetTrack())
					{
						Joint.Add(Beside->GetTrack());
					}
				}
			}
			if (Joint.Num() > 2)
			{
				++SwitchJoints;
				for (AFGBuildableRailroadTrack* A : Joint)
				{
					for (AFGBuildableRailroadTrack* B : Joint)
					{
						Link(A, B);
					}
				}
			}
		}
	}
	int32 Junctions = 0;
	for (const TPair<const AFGBuildableRailroadTrack*, TArray<AFGBuildableRailroadTrack*>>& Pair : mConflicts)
	{
		Junctions += Pair.Value.Num() > 1 ? 1 : 0;
	}
	mConflictsBuiltAt = FPlatformTime::Seconds();
	mConflictsStale = false;
	UE_LOG(LogAutoTrains, Log, TEXT("traffic: conflict graph rebuilt: %d tracks, %d junction tracks, %d switch joint sides, %.1f ms"),
		TrackCount, Junctions, SwitchJoints, (mConflictsBuiltAt - Begin) * 1000.0);
}

const TArray<AFGBuildableRailroadTrack*>& AATTrafficSubsystem::GetConflicts(const AFGBuildableRailroadTrack* Track) const
{
	if (const TArray<AFGBuildableRailroadTrack*>* Conflicts = mConflicts.Find(Track))
	{
		return *Conflicts;
	}
	// A track built since the last rebuild; treat it as conflicting with itself only until the graph catches up.
	mConflictsStale = true;
	static TArray<AFGBuildableRailroadTrack*> Single;
	Single.Reset();
	Single.Add(const_cast<AFGBuildableRailroadTrack*>(Track));
	return Single;
}

bool AATTrafficSubsystem::IsJunction(const AFGBuildableRailroadTrack* Track) const
{
	return Track && GetConflicts(Track).Num() > 1;
}

bool AATTrafficSubsystem::HoldsTrack(const FATTrafficTrain& State, const AFGBuildableRailroadTrack* Track) const
{
	return State.Occupied.Contains(const_cast<AFGBuildableRailroadTrack*>(Track))
		|| State.Claims.ContainsByPredicate([Track](const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim) { return Claim.Get() == Track; });
}

float AATTrafficSubsystem::GetPlanningDeceleration(const FATTrafficTrain& State)
{
	const AFGTrain* Train = State.Train.Get();
	const float Rated = Train ? Train->mConsistData.MaxBrakeDeceleration : 100.f;
	const float Planning = State.MeasuredBrake > 0.f ? State.MeasuredBrake * MeasuredBrakeShare : Rated * RatedBrakeShare;
	return FMath::Clamp(Planning, 50.f, 800.f);
}

float AATTrafficSubsystem::GetAuthorityDistance(const AFGTrain* Train) const
{
	const FATTrafficTrain* State = mTrains.Find(Train);
	if (!State || !State->bManaged || State->bAuthorityToGoal)
	{
		return -1.f;
	}
	const AFGBuildableRailroadTrack* Authority = State->AuthorityTrack.Get();
	const FTrainAtcData& Atc = Train->mAtcData;
	if (!Authority || !Atc.Path.IsValid() || !Atc.Path->PathPoints.IsValidIndex(Atc.CurrentPathSegment))
	{
		return 0.f;
	}
	float Distance = Atc.CurrentConnectionDistance;
	for (int32 Index = Atc.CurrentPathSegment; Index < Atc.Path->PathPoints.Num(); ++Index)
	{
		const UFGRailroadTrackConnectionComponent* Exit = Atc.Path->PathPoints[Index].TrackConnection.Get();
		const AFGBuildableRailroadTrack* Track = Exit ? Exit->GetTrack() : nullptr;
		if (!Track)
		{
			break;
		}
		if (Index > Atc.CurrentPathSegment)
		{
			Distance += Track->GetLength();
		}
		if (Track == Authority)
		{
			return FMath::Max(0.f, Distance - State->FrontAhead);
		}
	}
	return 0.f;
}

void AATTrafficSubsystem::ApplyAuthority(AFGTrain* Train)
{
	FATTrafficTrain* State = mEnabled ? mTrains.Find(Train) : nullptr;
	if (!State || !State->bManaged)
	{
		return;
	}
	AATControlSubsystem::IgnoreSignalStops(Train);
	const float Distance = GetAuthorityDistance(Train);
	if (Distance < 0.f)
	{
		State->bFullBrake = false;
		State->bCatchArmed = false;
		return;
	}

	FTrainAtcData& Atc = Train->mAtcData;
	const float Speed = FMath::Abs(Train->mSimulationData.Velocity);
	const float Usable = Distance - AuthorityStopMargin;
	const float Allowed = Usable > 0.f ? FMath::Sqrt(2.f * GetPlanningDeceleration(*State) * Usable) : 0.f;
	const bool bWasFullBrake = State->bFullBrake;
	State->bFullBrake = Speed > Allowed + OverspeedTolerance;
	Atc.TargetSpeed = State->bFullBrake ? 0.f : FMath::Min(Atc.TargetSpeed, Allowed);
	if (State->bFullBrake && !bWasFullBrake && Speed > Allowed + 500.f)
	{
		UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s over its braking curve (%.0f km/h, allowed %.0f, %.0f m to go): full brake"), *TrainName(Train),
			Speed * 0.036f, Allowed * 0.036f, Distance / 100.f);
	}

	// Backstop: when the authority ends at the master's next connection, point the restrictive point at it, so the
	// game's catch holds the train at the stop point whatever its speed. Clear keeps the wait timer and docking out.
	const UFGRailroadTrackConnectionComponent* Next = Atc.CurrentConnection.Get();
	const float StopOffset = State->FrontAhead + AuthorityStopMargin;
	const bool bArm = Next && Next->GetTrack() == State->AuthorityTrack.Get() && Atc.CurrentConnectionDistance > StopOffset
		&& (!Atc.NextRestrictiveSignalConnection.IsValid() || Atc.NextRestrictiveSignalDistance > Atc.CurrentConnectionDistance);
	if (bArm)
	{
		Atc.NextRestrictiveSignalConnection = const_cast<UFGRailroadTrackConnectionComponent*>(Next);
		Atc.NextRestrictiveSignalSpeed = 0.f;
		Atc.NextRestrictiveSignalDistance = Atc.CurrentConnectionDistance;
		Atc.NextRestrictiveSignalGrade = 0.f;
		Atc.NextRestrictiveSignalAspect = ERailroadSignalAspect::RSA_Clear;
		Atc.NextRestrictiveSignalStopOffset = StopOffset;
	}
	if (bArm != State->bCatchArmed)
	{
		UE_LOG(LogAutoTrains, Verbose, TEXT("traffic: %s catch %s (%.0f m to the connection, stop offset %.0f m, %.0f km/h)"), *TrainName(Train),
			bArm ? TEXT("armed") : TEXT("released"), Atc.CurrentConnectionDistance / 100.f, StopOffset / 100.f, Speed * 0.036f);
	}
	State->bCatchArmed = bArm;
}

FString AATTrafficSubsystem::DescribeForTelemetry(const AFGTrain* Train) const
{
	const FATTrafficTrain* State = mEnabled ? mTrains.Find(Train) : nullptr;
	if (!State)
	{
		return TEXT("0,-1,0,0,,0,0,0");
	}
	const float Distance = GetAuthorityDistance(Train);
	return FString::Printf(TEXT("%d,%.1f,%d,%.1f,%s,%d,%d,%.2f"), State->bManaged ? 1 : 0, Distance >= 0.f ? Distance / 100.f : -1.f, State->Claims.Num(),
		State->WaitingSeconds, *TrainName(State->BlockedBy.Get()).Replace(TEXT(","), TEXT(" ")), State->bFullBrake ? 1 : 0, State->bCatchArmed ? 1 : 0,
		GetPlanningDeceleration(*State) / 100.f);
}

TArray<FString> AATTrafficSubsystem::DescribeAll() const
{
	TArray<FString> Lines;
	if (!mEnabled)
	{
		Lines.Add(TEXT("Traffic control is off (/at traffic on)."));
		return Lines;
	}
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const FATTrafficTrain& State = Pair.Value;
		if (!State.bManaged)
		{
			continue;
		}
		const float Distance = GetAuthorityDistance(State.Train.Get());
		Lines.Add(FString::Printf(TEXT("%s: %.0f km/h, %d claims, may drive %s, brakes at %.1f m/s2, waited %.0f s, rerouted %d times%s%s"), *TrainName(State.Train.Get()),
			State.Speed * 0.036f, State.Claims.Num(), Distance < 0.f ? TEXT("to its stop") : *FString::Printf(TEXT("%.0f m"), Distance / 100.f),
			GetPlanningDeceleration(State) / 100.f, State.WaitingSeconds, State.Reroutes, State.BlockedReason.IsEmpty() ? TEXT("") : TEXT(", "), *State.BlockedReason));
	}
	if (Lines.Num() == 0)
	{
		Lines.Add(TEXT("No self-driving trains on the move."));
	}
	for (const FString& Line : Lines)
	{
		UE_LOG(LogAutoTrains, Log, TEXT("traffic status: %s"), *Line);
	}
	return Lines;
}
