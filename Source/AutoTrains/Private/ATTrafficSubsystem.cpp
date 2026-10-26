#include "ATTrafficSubsystem.h"
#include "ATControlSubsystem.h"
#include "AutoTrains.h"
#include "Buildables/FGBuildableRailroadTrack.h"
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
	// Share of the consist's brake deceleration used for planning; trains measured braking harder than this.
	constexpr float PlanningBrakeShare = 0.9f;
	constexpr double ConflictCacheSeconds = 10.0;

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
	mConflictCache.Reset();
	mUpdateTimer = 0.f;
	const FString Message = bEnabled ? TEXT("Traffic control on: self-driving trains ignore signals and claim track ahead.") : TEXT("Traffic control off.");
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
		ReadTrain(mTrains.FindOrAdd(Train), Train);
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
	const float Need = FMath::Square(State.Speed) / (2.f * GetPlanningDeceleration(Train)) + State.Length + ClaimLookahead;

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
		if (!IsTrackFree(Step.Track, State, Blocker))
		{
			State.BlockedBy = Blocker ? Blocker->Train : nullptr;
			State.BlockedReason = FString::Printf(TEXT("waits for %s, which holds %s or a track crossing it, %.0f m ahead"),
				*TrainName(Blocker ? Blocker->Train.Get() : nullptr), *TrackName(Step.Track), (Step.EndDistance - Step.Track->GetLength() - State.FrontAhead) / 100.f);
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

bool AATTrafficSubsystem::IsTrackFree(const AFGBuildableRailroadTrack* Track, const FATTrafficTrain& Self, const FATTrafficTrain*& OutBlocker) const
{
	const TArray<AFGBuildableRailroadTrack*>& Conflicts = const_cast<AATTrafficSubsystem*>(this)->GetConflicts(const_cast<AFGBuildableRailroadTrack*>(Track));
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
				return false;
			}
		}
	}
	return true;
}

bool AATTrafficSubsystem::IsValidStop(const FATTrafficTrain& Self, int32 EndIndex, FString& OutReason) const
{
	AATTrafficSubsystem* MutableThis = const_cast<AATTrafficSubsystem*>(this);
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
		if (MutableThis->IsJunction(Step.Track))
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
			if (MutableThis->IsJunction(Track))
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

const TArray<AFGBuildableRailroadTrack*>& AATTrafficSubsystem::GetConflicts(AFGBuildableRailroadTrack* Track)
{
	const double Now = FPlatformTime::Seconds();
	FTrackConflicts& Entry = mConflictCache.FindOrAdd(Track);
	if (Entry.Conflicts.Num() == 0 || Now - Entry.ReadTime > ConflictCacheSeconds)
	{
		Entry.Conflicts.Reset();
		Entry.Conflicts.Add(Track);
		for (AFGBuildableRailroadTrack* Overlapping : Track->GetOverlappingTracks())
		{
			if (Overlapping && Overlapping != Track)
			{
				Entry.Conflicts.AddUnique(Overlapping);
			}
		}
		Entry.ReadTime = Now;
	}
	return Entry.Conflicts;
}

bool AATTrafficSubsystem::IsJunction(AFGBuildableRailroadTrack* Track)
{
	return Track && GetConflicts(Track).Num() > 1;
}

bool AATTrafficSubsystem::HoldsTrack(const FATTrafficTrain& State, const AFGBuildableRailroadTrack* Track) const
{
	return State.Occupied.Contains(const_cast<AFGBuildableRailroadTrack*>(Track))
		|| State.Claims.ContainsByPredicate([Track](const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim) { return Claim.Get() == Track; });
}

float AATTrafficSubsystem::GetPlanningDeceleration(const AFGTrain* Train)
{
	return FMath::Max(Train->mConsistData.MaxBrakeDeceleration * PlanningBrakeShare, 50.f);
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
	if (!IsManaged(Train))
	{
		return;
	}
	AATControlSubsystem::IgnoreSignalStops(Train);
	const float Distance = GetAuthorityDistance(Train);
	if (Distance < 0.f)
	{
		return;
	}
	const float Usable = Distance - AuthorityStopMargin;
	const float Allowed = Usable > 0.f ? FMath::Sqrt(2.f * GetPlanningDeceleration(Train) * Usable) : 0.f;
	FTrainAtcData& Atc = Train->mAtcData;
	Atc.TargetSpeed = FMath::Min(Atc.TargetSpeed, Allowed);
}

FString AATTrafficSubsystem::DescribeForTelemetry(const AFGTrain* Train) const
{
	const FATTrafficTrain* State = mEnabled ? mTrains.Find(Train) : nullptr;
	if (!State)
	{
		return TEXT("0,-1,0,0,");
	}
	const float Distance = GetAuthorityDistance(Train);
	return FString::Printf(TEXT("%d,%.1f,%d,%.1f,%s"), State->bManaged ? 1 : 0, Distance >= 0.f ? Distance / 100.f : -1.f, State->Claims.Num(),
		State->WaitingSeconds, *TrainName(State->BlockedBy.Get()).Replace(TEXT(","), TEXT(" ")));
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
		Lines.Add(FString::Printf(TEXT("%s: %.0f km/h, %d claims, may drive %s, waited %.0f s%s%s"), *TrainName(State.Train.Get()),
			State.Speed * 0.036f, State.Claims.Num(), Distance < 0.f ? TEXT("to its stop") : *FString::Printf(TEXT("%.0f m"), Distance / 100.f),
			State.WaitingSeconds, State.BlockedReason.IsEmpty() ? TEXT("") : TEXT(", "), *State.BlockedReason));
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
