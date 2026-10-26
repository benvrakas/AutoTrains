#include "ATTrafficSubsystem.h"
#include "ATConfig.h"
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
	// From the Mods menu: tighter margins, and trains passing through each other.
	bool GAggressive = true;
	bool GPhaseThrough = false;
	// Seconds of slack for following and going first. [s]
	float GSafetyMargin = 0.f;
	int32 GPhaseThroughCount = 0;

	constexpr float TrafficUpdateInterval = 0.25f;
	constexpr float SettingsInterval = 1.f;
	// Claims reach this far past the braking distance plus the train's length; less when aggressive. [cm]
	float ClaimLookahead() { return GAggressive ? 10000.f : 20000.f; }
	// Trains stop this far before the end of their authority, or SwitchStopMargin before a switch. [cm]
	constexpr float AuthorityStopMargin = 1000.f;
	constexpr float SwitchStopMargin = 2500.f;
	// Distance along each track from a switch point within which a vehicle fouls the switch. [cm]
	constexpr float JointFoulDistance = 2000.f;
	// Other trains' routes count for stopping spots up to this far ahead of them. [cm]
	constexpr float OthersRouteHorizon = 300000.f;
	// Planning deceleration: shares of the measured full-brake deceleration and of the consist's rating. Loaded trains
	// were measured braking at 2/3 of their rating, light ones at 1.4x.
	float MeasuredBrakeShare() { return GAggressive ? 0.95f : 0.85f; }
	float RatedBrakeShare() { return GAggressive ? 0.8f : 0.6f; }
	// Braking is weaker at higher speed; a speed band above every measured one plans with this share of the weakest.
	constexpr float UnmeasuredBandShare = 0.75f;
	// A train over its braking curve by more than this gets full brake. [cm/s]
	constexpr float OverspeedTolerance = 100.f;
	constexpr double ConflictRebuildSeconds = 30.0;
	constexpr double ConflictRebuildMinSeconds = 2.0;

	// Taking over another train's claim: the holder must be able to stop this much short of the conflict as well, and
	// the taker must clear it this long before the holder could arrive. Taker time assumes half its acceleration,
	// holder time its full acceleration. [cm] [s]
	float PreemptStopBuffer() { return GAggressive ? 1000.f : 3000.f; }
	float PreemptTimeMargin() { return GSafetyMargin; }
	float PreemptTakerAccelerationShare() { return GAggressive ? 0.9f : 0.5f; }

	// Claim order: each second waited counts as this many seconds sooner through. [s/s]
	float WaitingWeight() { return GAggressive ? 0.5f : 1.f; }

	// Following: a train may count track another train holds as free if that train is moving at least FollowMinSpeed
	// and clears it the safety margin before the follower could get there. [s] [cm/s]
	float FollowTimeMargin() { return GSafetyMargin; }
	constexpr float FollowMinSpeed = 500.f;
	// Returned by GetDistanceToTrackStart for a track that isn't on the train's path.
	constexpr float TrackNotOnPath = -1e9f;
	// A leader standing still this long gets track it shares with a follower back. [s]
	constexpr float LeaderStopSeconds = 2.f;
	// Other trains whose route passes a crossing or switch within this distance keep a train from stopping on it. [cm]
	float NeedHorizon() { return GAggressive ? 150000.f : 300000.f; }

	// A train that waited this long counts for deadlock detection. [s]
	constexpr float DeadlockWaitSeconds = 20.f;

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
	// A train that rerouted doesn't reroute again for this long, so it can't flip between two routes. [s]
	constexpr float RerouteCooldownSeconds = 30.f;

	float GetMass(const AFGTrain* Train)
	{
		return Train ? FMath::Max(Train->mConsistData.Mass, 1.f) : 1.f;
	}

	int32 GetBrakeBand(float Speed)
	{
		return FMath::Clamp(FMath::FloorToInt(Speed / ATBrakeBandWidth), 0, ATBrakeBands - 1);
	}

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
	mSettingsTimer -= DeltaSeconds;
	if (mSettingsTimer <= 0.f)
	{
		mSettingsTimer = SettingsInterval;
		ApplySettings();
	}
	if (!mEnabled)
	{
		return;
	}
	mUpdateTimer -= DeltaSeconds;
	mSinceUpdate += DeltaSeconds;
	if (mUpdateTimer <= 0.f)
	{
		mUpdateTimer = TrafficUpdateInterval;
		UpdateTraffic(mSinceUpdate);
		mSinceUpdate = 0.f;
	}
	mPhaseReportTimer -= DeltaSeconds;
	if (mPhaseReportTimer <= 0.f)
	{
		mPhaseReportTimer = 10.f;
		if (GPhaseThroughCount > 0)
		{
			UE_LOG(LogAutoTrains, Log, TEXT("traffic: trains passed through each other %d times in the last 10 s"), GPhaseThroughCount);
			GPhaseThroughCount = 0;
		}
	}
}

void AATTrafficSubsystem::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	GPhaseThrough = false;
	Super::EndPlay(EndPlayReason);
}

void AATTrafficSubsystem::ApplySettings()
{
	FATSettings Settings;
	if (!UATConfig::Read(this, Settings))
	{
		return;
	}
	if (!mConfigTraffic.IsSet() || mConfigTraffic.GetValue() != Settings.bTrafficControl)
	{
		mConfigTraffic = Settings.bTrafficControl;
		if (mEnabled != Settings.bTrafficControl)
		{
			SetEnabled(Settings.bTrafficControl);
		}
	}
	if (!mConfigRerouting.IsSet() || mConfigRerouting.GetValue() != Settings.bRerouting)
	{
		mConfigRerouting = Settings.bRerouting;
		if (mRerouteEnabled != Settings.bRerouting)
		{
			SetRerouteEnabled(Settings.bRerouting);
		}
	}
	const bool bPhaseThrough = mEnabled && Settings.bPhaseThrough;
	if (bPhaseThrough != GPhaseThrough || Settings.bAggressive != GAggressive || Settings.SafetyMargin != GSafetyMargin)
	{
		UE_LOG(LogAutoTrains, Log, TEXT("Settings: trains pass through each other %s, aggressive timing %s, safety margin %.1f s"),
			bPhaseThrough ? TEXT("on") : TEXT("off"), Settings.bAggressive ? TEXT("on") : TEXT("off"), Settings.SafetyMargin);
	}
	GPhaseThrough = bPhaseThrough;
	GAggressive = Settings.bAggressive;
	GSafetyMargin = Settings.SafetyMargin;
}

bool AATTrafficSubsystem::ShouldPhaseThrough()
{
	return GPhaseThrough;
}

void AATTrafficSubsystem::CountPhaseThrough()
{
	++GPhaseThroughCount;
}

FString AATTrafficSubsystem::SetEnabled(bool bEnabled)
{
	mEnabled = bEnabled;
	if (!bEnabled)
	{
		GPhaseThrough = false;
	}
	mTrains.Reset();
	mUpdateTimer = 0.f;
	mSinceUpdate = 0.f;
	mLastDeadlock.Reset();
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

		// Measure full-brake deceleration per speed band between updates with full brake on at both; a catch ends at
		// standstill, and docking catches and crashes are far above 20 m/s^2.
		if (State.bFullBrake && State.bWasFullBrake && State.LastSpeed > 500.f && State.Speed > 300.f && DeltaSeconds > 0.f)
		{
			const float Deceleration = (State.LastSpeed - State.Speed) / DeltaSeconds;
			if (Deceleration > 50.f && Deceleration < 2000.f)
			{
				const int32 Band = GetBrakeBand((State.LastSpeed + State.Speed) * 0.5f);
				const float Force = Deceleration * GetMass(Train);
				float& Stored = State.BrakeForce[Band];
				const bool bFirst = Stored <= 0.f;
				// Weaker samples count more, so the plan errs on the safe side.
				Stored = bFirst ? Force : Force < Stored ? Stored * 0.5f + Force * 0.5f : Stored * 0.85f + Force * 0.15f;
				if (bFirst)
				{
					UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s brakes at %.2f m/s2 at %.0f-%.0f km/h (rated %.2f, %.0f t); planning with %.2f"), *TrainName(Train),
						Deceleration / 100.f, Band * ATBrakeBandWidth * 0.036f, (Band + 1) * ATBrakeBandWidth * 0.036f, Train->mConsistData.MaxBrakeDeceleration / 100.f,
						GetMass(Train) / 1000.f, GetPlanningDeceleration(State) / 100.f);
				}
			}
		}
		State.bWasFullBrake = State.bFullBrake;
		State.LastSpeed = State.Speed;
		State.StoppedSeconds = State.Speed < 100.f ? State.StoppedSeconds + DeltaSeconds : 0.f;
		State.RerouteCooldown = FMath::Max(0.f, State.RerouteCooldown - DeltaSeconds);
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
		RefreshClaimedJoints(Pair.Value);
		if (Pair.Value.bManaged)
		{
			Managed.Add(&Pair.Value);
		}
	}

	// The train that would get through its next track soonest claims first, so a quick train isn't held for a slow
	// one; time already waited counts in a train's favour so none waits forever. Ties go by name.
	TMap<const FATTrafficTrain*, float> Priority;
	for (const FATTrafficTrain* State : Managed)
	{
		Priority.Add(State, GetClaimPriority(*State));
	}
	Managed.Sort([&Priority](const FATTrafficTrain& A, const FATTrafficTrain& B)
	{
		const float PriorityA = Priority.FindRef(&A);
		const float PriorityB = Priority.FindRef(&B);
		if (PriorityA != PriorityB)
		{
			return PriorityA < PriorityB;
		}
		return TrainName(A.Train.Get()) < TrainName(B.Train.Get());
	});
	for (FATTrafficTrain* State : Managed)
	{
		const FString PreviousReason = State->BlockedReason;
		RevalidateFollowClaims(*State);
		ExtendClaims(*State);
		UpdateAuthority(*State);
		UpdateFollowLeaders(*State);
		State->ClearSeconds = State->BlockedBy.IsValid() ? GetBlockerClearSeconds(*State) : -1.f;
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
		const bool bHeld = !State->bAuthorityToGoal && Distance >= 0.f && Distance < State->StopMargin + 5000.f;
		if (State->Speed < 50.f && bHeld)
		{
			State->WaitingSeconds += DeltaSeconds;
		}
		else if (State->Speed > 100.f)
		{
			State->WaitingSeconds = 0.f;
		}
	}
	LogDeadlocks();
}

void AATTrafficSubsystem::ReadTrain(FATTrafficTrain& State, AFGTrain* Train) const
{
	State.Train = Train;
	State.Length = Train->mConsistData.Length;
	State.Speed = FMath::Abs(Train->mSimulationData.Velocity);
	State.Route.Reset();
	State.Occupied.Reset();
	State.OccupiedJoints.Reset();
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
		const float HalfLength = Vehicle->GetLength() * 0.5f;
		for (int32 End = 0; End < 2; ++End)
		{
			const UFGRailroadTrackConnectionComponent* Connection = Track->GetConnection(End);
			if (!Connection)
			{
				continue;
			}
			const float ToEnd = FMath::Abs(Connection->GetTrackPosition().Offset - Position.Offset);
			// A vehicle near a track end overhangs onto the tracks attached there.
			if (ToEnd < HalfLength + 200.f)
			{
				AddAttachedTracks(Connection, State.Occupied);
			}
			const int32 Joint = GetJoint(Connection);
			if (Joint != INDEX_NONE && ToEnd < HalfLength + JointFoulDistance)
			{
				State.OccupiedJoints.Add(Joint);
			}
		}
		// ATC distances are from the MU master; find how far the train's front is ahead of it.
		for (const FATRouteStep& Step : State.Route)
		{
			if (Step.Track == Track && Step.Exit)
			{
				const float ToExit = FMath::Abs(Step.Exit->GetTrackPosition().Offset - Position.Offset);
				State.FrontAhead = FMath::Max(State.FrontAhead, Step.EndDistance - ToExit + HalfLength);
				break;
			}
		}
	}

	if (!State.bManaged)
	{
		State.Claims.Reset();
		State.ClaimedJoints.Reset();
		State.FollowLeaders.Reset();
		State.AuthorityTrack = nullptr;
		State.bAuthorityToGoal = false;
		State.BlockedReason.Reset();
		State.BlockedBy = nullptr;
		State.WaitingSeconds = 0.f;
		State.bFullBrake = false;
		State.bCatchArmed = false;
		State.ClearSeconds = -1.f;
	}
}

void AATTrafficSubsystem::ReleasePassedClaims(FATTrafficTrain& State) const
{
	// Keep only claims the train stands on or that run unbroken along its route from its front; a claim past a gap
	// (left over after a reroute, or after giving way) would block others without the train being able to use it.
	TSet<const AFGBuildableRailroadTrack*> Reachable;
	for (const FATRouteStep& Step : State.Route)
	{
		const bool bHeld = State.Occupied.Contains(Step.Track)
			|| State.Claims.ContainsByPredicate([&Step](const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim) { return Claim.Get() == Step.Track; });
		if (!bHeld)
		{
			break;
		}
		Reachable.Add(Step.Track);
	}
	State.Claims.RemoveAll([&State, &Reachable](const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim)
	{
		AFGBuildableRailroadTrack* Track = Claim.Get();
		return !Track || (!State.Occupied.Contains(Track) && !Reachable.Contains(Track));
	});
}

void AATTrafficSubsystem::RefreshClaimedJoints(FATTrafficTrain& State) const
{
	State.ClaimedJoints.Reset();
	for (int32 Index = 0; Index + 1 < State.Route.Num() && HoldsTrack(State, State.Route[Index].Track) && HoldsTrack(State, State.Route[Index + 1].Track); ++Index)
	{
		const int32 Joint = GetJoint(State.Route[Index].Exit);
		if (Joint == INDEX_NONE)
		{
			continue;
		}
		State.ClaimedJoints.Add(Joint);
		if (State.Occupied.Contains(State.Route[Index].Track) && State.Occupied.Contains(State.Route[Index + 1].Track))
		{
			State.OccupiedJoints.Add(Joint);
		}
	}
}

void AATTrafficSubsystem::ExtendClaims(FATTrafficTrain& State)
{
	AFGTrain* Train = State.Train.Get();
	if (!Train || State.Route.Num() == 0)
	{
		return;
	}
	const float Need = FMath::Square(State.Speed) / (2.f * GetPlanningDeceleration(State)) + State.Length + ClaimLookahead();

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
	TArray<int32> Pending;
	TArray<FPreemption> Preemptions;
	// Trains this chunk follows through; it mustn't take track they still need and haven't claimed.
	TSet<const FATTrafficTrain*> Followed;
	// A train is never both followed and overtaken: no going first ahead of a train this one follows (now or from
	// earlier claims), and no following a train it goes first ahead of.
	auto IsFollowing = [&State, &Followed](const FATTrafficTrain* Holder)
	{
		return Followed.Contains(Holder) || State.FollowLeaders.Contains(Holder->Train);
	};
	auto IsPreempted = [&Preemptions](const FATTrafficTrain* Holder)
	{
		return Preemptions.ContainsByPredicate([Holder](const FPreemption& Preemption) { return Preemption.Holder == Holder; });
	};
	// Distance from the front at which this train has cleared everything it would take over. [cm]
	float ClearDistance = 0.f;
	FString StopReason;
	for (; Index < State.Route.Num(); ++Index)
	{
		const FATRouteStep& Step = State.Route[Index];
		const float StepStart = Step.EndDistance - Step.Track->GetLength() - State.FrontAhead;
		FATTrafficTrain* Holder = nullptr;
		bool bStands = false;
		FString What;

		// Trains already dealt with at this step: ones that move on and clear it before this train could get there,
		// and ones whose claim it would take over.
		TSet<const FATTrafficTrain*> Leaving;
		bool bBlocked = false;

		// The switch between the previous track and this one.
		const int32 Joint = Index > 0 ? GetJoint(State.Route[Index - 1].Exit) : INDEX_NONE;
		while (Joint != INDEX_NONE && !IsJointFree(Joint, State, Holder, bStands, &Leaving))
		{
			What = FString::Printf(TEXT("the switch before %s"), *TrackName(Step.Track));
			if (!IsPreempted(Holder) && ((GAggressive && IsSameWayLeader(State, *Holder, Index - 1))
				|| IsClearingInTime(State, *Holder, nullptr, Joint, StepStart - JointFoulDistance)))
			{
				Leaving.Add(Holder);
				Followed.Add(Holder);
				continue;
			}
			if (bStands || !Holder->bManaged || IsFollowing(Holder))
			{
				State.BlockedBy = Holder->Train;
				State.BlockedReason = FString::Printf(TEXT("waits for %s, which %s %s, %.0f m ahead"), *TrainName(Holder->Train.Get()),
					bStands ? TEXT("stands at") : TEXT("is ahead through"), *What, StepStart / 100.f);
				bBlocked = true;
				break;
			}
			// Every train with a claim here must be able to give way, not just the first one found.
			Preemptions.Add({Holder, nullptr, Joint});
			ClearDistance = FMath::Max(ClearDistance, StepStart + State.Length + JointFoulDistance);
			Leaving.Add(Holder);
		}
		if (bBlocked)
		{
			break;
		}

		AFGBuildableRailroadTrack* Conflict = nullptr;
		Holder = nullptr;
		while (!IsTrackFree(Step.Track, State, Holder, Conflict, bStands, &Leaving))
		{
			What = Conflict == Step.Track ? TrackName(Conflict) : FString::Printf(TEXT("%s (crossing %s on this train's route)"), *TrackName(Conflict), *TrackName(Step.Track));
			if (!IsPreempted(Holder) && ((GAggressive && IsSameWayLeader(State, *Holder, Index))
				|| IsClearingInTime(State, *Holder, Conflict, INDEX_NONE, StepStart)))
			{
				Leaving.Add(Holder);
				Followed.Add(Holder);
				continue;
			}
			if (bStands || !Holder->bManaged || IsFollowing(Holder))
			{
				State.BlockedBy = Holder->Train;
				State.BlockedReason = FString::Printf(TEXT("waits for %s, which %s %s, %.0f m ahead"), *TrainName(Holder->Train.Get()),
					bStands ? TEXT("stands on") : TEXT("is ahead on"), *What, StepStart / 100.f);
				bBlocked = true;
				break;
			}
			Preemptions.Add({Holder, Conflict, INDEX_NONE});
			ClearDistance = FMath::Max(ClearDistance, Step.EndDistance - State.FrontAhead + State.Length);
			Leaving.Add(Holder);
		}
		if (bBlocked)
		{
			break;
		}

		Pending.Add(Index);
		const bool bGoal = Index == State.Route.Num() - 1;
		if (!bGoal && !IsValidStop(State, Index, StopReason))
		{
			continue;
		}
		// Following a train through stays behind it: no taking track or a switch on its route it hasn't claimed yet.
		const FATTrafficTrain* Overtaken = nullptr;
		const AFGBuildableRailroadTrack* OvertakenAt = nullptr;
		for (const FATTrafficTrain* Leader : Followed)
		{
			for (const int32 PendingIndex : Pending)
			{
				const AFGBuildableRailroadTrack* Track = State.Route[PendingIndex].Track;
				const int32 PendingJoint = PendingIndex > 0 ? GetJoint(State.Route[PendingIndex - 1].Exit) : INDEX_NONE;
				for (int32 LeaderIndex = 0; LeaderIndex < Leader->Route.Num() && !Overtaken; ++LeaderIndex)
				{
					const FATRouteStep& LeaderStep = Leader->Route[LeaderIndex];
					const bool bNeedsTrack = LeaderStep.Track == Track && !HoldsTrack(*Leader, Track);
					const bool bNeedsJoint = PendingJoint != INDEX_NONE && LeaderIndex + 1 < Leader->Route.Num() && GetJoint(LeaderStep.Exit) == PendingJoint
						&& !HoldsJoint(*Leader, PendingJoint);
					if (bNeedsTrack || bNeedsJoint)
					{
						Overtaken = Leader;
						OvertakenAt = Track;
					}
				}
			}
			if (Overtaken)
			{
				break;
			}
		}
		if (Overtaken)
		{
			State.BlockedBy = Overtaken->Train;
			State.BlockedReason = FString::Printf(TEXT("follows %s, which still needs %s"), *TrainName(Overtaken->Train.Get()), *TrackName(OvertakenAt));
			Pending.Reset();
			break;
		}

		if (Preemptions.Num() > 0)
		{
			FString PreemptReason;
			if (!CanPreempt(State, Preemptions, ClearDistance, PreemptReason))
			{
				State.BlockedBy = Preemptions[0].Holder->Train;
				State.BlockedReason = PreemptReason;
				Pending.Reset();
				break;
			}
			// Give way tentatively and check everything this train would take is then free; if not, the other trains get
			// their claims back, so a failed attempt costs nobody anything.
			TMap<FATTrafficTrain*, FATTrafficTrain> Before;
			for (const FPreemption& Preemption : Preemptions)
			{
				if (!Before.Contains(Preemption.Holder))
				{
					Before.Add(Preemption.Holder, *Preemption.Holder);
				}
			}
			ApplyPreemptions(Preemptions);
			bool bAllFree = true;
			for (const int32 PendingIndex : Pending)
			{
				const FATRouteStep& PendingStep = State.Route[PendingIndex];
				const float PendingStart = PendingStep.EndDistance - PendingStep.Track->GetLength() - State.FrontAhead;
				const int32 PendingJoint = PendingIndex > 0 ? GetJoint(State.Route[PendingIndex - 1].Exit) : INDEX_NONE;
				TSet<const FATTrafficTrain*> PendingLeaving;
				FATTrafficTrain* Other = nullptr;
				AFGBuildableRailroadTrack* OtherConflict = nullptr;
				bool bOtherStands = false;
				bool bFree = true;
				while (PendingJoint != INDEX_NONE && !IsJointFree(PendingJoint, State, Other, bOtherStands, &PendingLeaving))
				{
					if ((GAggressive && IsSameWayLeader(State, *Other, PendingIndex - 1)) || IsClearingInTime(State, *Other, nullptr, PendingJoint, PendingStart - JointFoulDistance))
					{
						PendingLeaving.Add(Other);
						continue;
					}
					bFree = false;
					break;
				}
				while (bFree && !IsTrackFree(PendingStep.Track, State, Other, OtherConflict, bOtherStands, &PendingLeaving))
				{
					if ((GAggressive && IsSameWayLeader(State, *Other, PendingIndex)) || IsClearingInTime(State, *Other, OtherConflict, INDEX_NONE, PendingStart))
					{
						PendingLeaving.Add(Other);
						continue;
					}
					bFree = false;
				}
				if (!bFree)
				{
					bAllFree = false;
					State.BlockedBy = Other ? Other->Train : nullptr;
					State.BlockedReason = FString::Printf(TEXT("waits for %s at %s, which another train would give way at"), *TrainName(Other ? Other->Train.Get() : nullptr),
						*TrackName(PendingStep.Track));
					break;
				}
			}
			if (!bAllFree)
			{
				for (TPair<FATTrafficTrain*, FATTrafficTrain>& Pair : Before)
				{
					*Pair.Key = MoveTemp(Pair.Value);
				}
				Pending.Reset();
				break;
			}
			for (const FPreemption& Preemption : Preemptions)
			{
				const FATTrafficTrain* Old = Before.Find(Preemption.Holder);
				const int32 GivenUp = Old ? Old->Claims.Num() - Preemption.Holder->Claims.Num() : 0;
				// A train listed for several tracks or switches is logged once, at the first.
				if (GivenUp > 0)
				{
					UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s goes first over %s; %s gives up %d claims"), *TrainName(State.Train.Get()),
						Preemption.Joint != INDEX_NONE ? TEXT("a switch") : *TrackName(Preemption.Track), *TrainName(Preemption.Holder->Train.Get()), GivenUp);
					Before.Remove(Preemption.Holder);
				}
			}
			Preemptions.Reset();
			ClearDistance = 0.f;
		}
		for (const int32 PendingIndex : Pending)
		{
			State.Claims.Add(State.Route[PendingIndex].Track);
		}
		Pending.Reset();
		RefreshClaimedJoints(State);
		if (bGoal || Step.EndDistance - State.FrontAhead >= Need)
		{
			break;
		}
	}
	if (Pending.Num() > 0 && State.BlockedReason.IsEmpty())
	{
		State.BlockedReason = FString::Printf(TEXT("no safe place to stop beyond %s: %s"), *TrackName(State.Route[Pending[0]].Track), *StopReason);
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
	State.StopMargin = AuthorityStopMargin;
	int32 Last = INDEX_NONE;
	for (int32 Index = 0; Index < State.Route.Num() && HoldsTrack(State, State.Route[Index].Track); ++Index)
	{
		Last = Index;
	}
	if (Last != INDEX_NONE)
	{
		State.AuthorityTrack = State.Route[Last].Track;
		State.bAuthorityToGoal = Last == State.Route.Num() - 1;
		State.StopMargin = GetStopMargin(State.Route[Last].Exit);
	}
	RefreshClaimedJoints(State);
}

bool AATTrafficSubsystem::IsTrackFree(const AFGBuildableRailroadTrack* Track, const FATTrafficTrain& Self, FATTrafficTrain*& OutHolder,
	AFGBuildableRailroadTrack*& OutConflict, bool& bOutStandsOnIt, const TSet<const FATTrafficTrain*>* Ignore)
{
	const TArray<AFGBuildableRailroadTrack*>& Conflicts = GetConflicts(Track);
	// A train standing there wins over one that only claimed it, so the reason shows what really blocks.
	for (int32 Pass = 0; Pass < 2; ++Pass)
	{
		for (TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
		{
			FATTrafficTrain& Other = Pair.Value;
			if (&Other == &Self || (Ignore && Ignore->Contains(&Other)))
			{
				continue;
			}
			for (AFGBuildableRailroadTrack* Conflict : Conflicts)
			{
				const bool bStands = Other.Occupied.Contains(Conflict);
				if (Pass == 0 ? bStands : HoldsTrack(Other, Conflict))
				{
					OutHolder = &Other;
					OutConflict = Conflict;
					bOutStandsOnIt = bStands;
					return false;
				}
			}
		}
	}
	return true;
}

bool AATTrafficSubsystem::IsJointFree(int32 Joint, const FATTrafficTrain& Self, FATTrafficTrain*& OutHolder, bool& bOutStandsAtIt,
	const TSet<const FATTrafficTrain*>* Ignore)
{
	for (int32 Pass = 0; Pass < 2; ++Pass)
	{
		for (TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
		{
			FATTrafficTrain& Other = Pair.Value;
			if (&Other == &Self || (Ignore && Ignore->Contains(&Other)))
			{
				continue;
			}
			const bool bStands = Other.OccupiedJoints.Contains(Joint);
			if (Pass == 0 ? bStands : HoldsJoint(Other, Joint))
			{
				OutHolder = &Other;
				bOutStandsAtIt = bStands;
				return false;
			}
		}
	}
	return true;
}

int32 AATTrafficSubsystem::FindOnRoute(const FATTrafficTrain& Holder, const FPreemption& Preemption) const
{
	for (int32 Index = 0; Index < Holder.Route.Num(); ++Index)
	{
		const FATRouteStep& Step = Holder.Route[Index];
		if (Preemption.Joint != INDEX_NONE ? GetJoint(Step.Exit) == Preemption.Joint : Step.Track == Preemption.Track)
		{
			return Index;
		}
	}
	return INDEX_NONE;
}

bool AATTrafficSubsystem::CanPreempt(const FATTrafficTrain& Self, const TArray<FPreemption>& Preemptions, float ClearDistance, FString& OutReason) const
{
	const AFGTrain* SelfTrain = Self.Train.Get();
	const float SelfTime = TimeToCover(ClearDistance, Self.Speed, GetAcceleration(Self, PreemptTakerAccelerationShare()),
		SelfTrain ? SelfTrain->mConsistData.MaxSpeed : 0.f);
	for (const FPreemption& Preemption : Preemptions)
	{
		const FATTrafficTrain& Holder = *Preemption.Holder;
		const FString What = Preemption.Joint != INDEX_NONE ? FString(TEXT("a switch")) : TrackName(Preemption.Track);
		const int32 At = FindOnRoute(Holder, Preemption);
		if (At == INDEX_NONE)
		{
			OutReason = FString::Printf(TEXT("waits for %s, whose claim on %s is off its route"), *TrainName(Holder.Train.Get()), *What);
			return false;
		}
		const FATRouteStep& Step = Holder.Route[At];
		const float HolderDistance = Preemption.Joint != INDEX_NONE
			? Step.EndDistance - Holder.FrontAhead - JointFoulDistance
			: Step.EndDistance - Step.Track->GetLength() - Holder.FrontAhead;
		const float HolderStop = FMath::Square(Holder.Speed) / (2.f * GetPlanningDeceleration(Holder)) + Holder.StopMargin + PreemptStopBuffer();
		if (HolderDistance < HolderStop)
		{
			OutReason = FString::Printf(TEXT("waits for %s, which has claimed %s and is too close to give it up (%.0f m)"), *TrainName(Holder.Train.Get()), *What,
				HolderDistance / 100.f);
			return false;
		}
		const AFGTrain* HolderTrain = Holder.Train.Get();
		const float HolderTime = TimeToCover(HolderDistance, Holder.Speed, GetAcceleration(Holder, 1.f), HolderTrain ? HolderTrain->mConsistData.MaxSpeed : 0.f);
		if (SelfTime + PreemptTimeMargin() > HolderTime)
		{
			OutReason = FString::Printf(TEXT("waits for %s, which has claimed %s and can be there in %.0f s; this train needs %.0f s to clear it"),
				*TrainName(Holder.Train.Get()), *What, HolderTime, SelfTime);
			return false;
		}
	}
	return true;
}

void AATTrafficSubsystem::ApplyPreemptions(const TArray<FPreemption>& Preemptions)
{
	for (const FPreemption& Preemption : Preemptions)
	{
		FATTrafficTrain& Holder = *Preemption.Holder;
		const int32 At = FindOnRoute(Holder, Preemption);
		if (At == INDEX_NONE)
		{
			continue;
		}
		// The holder keeps its claims short of the conflict: up to the track before it, or up to the switch.
		const int32 Cut = Preemption.Joint != INDEX_NONE ? At + 1 : At;
		TSet<const AFGBuildableRailroadTrack*> Dropped;
		for (int32 Index = Cut; Index < Holder.Route.Num(); ++Index)
		{
			if (!Holder.Occupied.Contains(Holder.Route[Index].Track))
			{
				Dropped.Add(Holder.Route[Index].Track);
			}
		}
		Holder.Claims.RemoveAll([&Dropped](const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim) { return Dropped.Contains(Claim.Get()); });
		UpdateAuthority(Holder);
	}
}

bool AATTrafficSubsystem::IsValidStop(const FATTrafficTrain& Self, int32 EndIndex, FString& OutReason) const
{
	const float FrontAtStop = Self.Route[EndIndex].EndDistance - Self.FrontAhead - GetStopMargin(Self.Route[EndIndex].Exit);
	const float TailAtStop = FrontAtStop - Self.Length;
	for (int32 Index = EndIndex; Index >= 0; --Index)
	{
		const FATRouteStep& Step = Self.Route[Index];
		const float TrackEnd = Step.EndDistance - Self.FrontAhead;
		if (TrackEnd <= TailAtStop)
		{
			break;
		}
		if (TrackEnd - Step.Track->GetLength() >= FrontAtStop)
		{
			continue;
		}
		// Standing on a crossing or at a switch is fine as long as no other train needs to get through it soon.
		if (IsCrossing(Step.Track) && IsNeededByOthers(Self, Step.Track, INDEX_NONE))
		{
			OutReason = FString::Printf(TEXT("the train would stand on crossing %s, which another train needs"), *TrackName(Step.Track));
			return false;
		}
		if (!IsOutOfOthersWay(Self, Step, OutReason))
		{
			return false;
		}
	}

	// Switches within the fouling zone of the stopped train, at the ends of its route tracks and behind the first one.
	auto FoulsAt = [&](float JointPosition) { return JointPosition > TailAtStop - JointFoulDistance && JointPosition < FrontAtStop + JointFoulDistance; };
	for (int32 Index = 0; Index <= EndIndex; ++Index)
	{
		const FATRouteStep& Step = Self.Route[Index];
		const int32 Joint = GetJoint(Step.Exit);
		if (Joint != INDEX_NONE && FoulsAt(Step.EndDistance - Self.FrontAhead) && IsNeededByOthers(Self, nullptr, Joint))
		{
			OutReason = FString::Printf(TEXT("the train would stand at the switch after %s, which another train needs"), *TrackName(Step.Track));
			return false;
		}
	}
	const FATRouteStep& First = Self.Route[0];
	const float RouteStart = First.EndDistance - First.Track->GetLength() - Self.FrontAhead;
	const int32 FirstJoint = First.Exit ? GetJoint(First.Exit->GetOpposite()) : INDEX_NONE;
	if (FirstJoint != INDEX_NONE && FoulsAt(RouteStart) && IsNeededByOthers(Self, nullptr, FirstJoint))
	{
		OutReason = FString::Printf(TEXT("the train would stand at the switch before %s, which another train needs"), *TrackName(First.Track));
		return false;
	}

	// The tail would still be on tracks behind the route; those are the ones the train occupies now.
	if (TailAtStop < RouteStart)
	{
		for (AFGBuildableRailroadTrack* Track : Self.Occupied)
		{
			if (IsCrossing(Track) && IsNeededByOthers(Self, Track, INDEX_NONE))
			{
				OutReason = FString::Printf(TEXT("the tail would stand on crossing %s, which another train needs"), *TrackName(Track));
				return false;
			}
		}
		for (const int32 Joint : Self.OccupiedJoints)
		{
			if (IsNeededByOthers(Self, nullptr, Joint))
			{
				OutReason = TEXT("the tail would still stand at a switch another train needs");
				return false;
			}
		}
	}
	return true;
}

bool AATTrafficSubsystem::IsNeededByOthers(const FATTrafficTrain& Self, const AFGBuildableRailroadTrack* Track, int32 Joint) const
{
	const TArray<AFGBuildableRailroadTrack*>* Conflicts = Track ? &GetConflicts(Track) : nullptr;
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const FATTrafficTrain& Other = Pair.Value;
		if (&Other == &Self || !Other.bManaged)
		{
			continue;
		}
		for (int32 Index = 0; Index < Other.Route.Num(); ++Index)
		{
			const FATRouteStep& Step = Other.Route[Index];
			if (Step.EndDistance - Step.Track->GetLength() - Other.FrontAhead > NeedHorizon())
			{
				break;
			}
			if (Joint != INDEX_NONE && Index + 1 < Other.Route.Num() && GetJoint(Step.Exit) == Joint)
			{
				return true;
			}
			// The crossing itself is Track's own business; only the tracks crossing it count here.
			if (Conflicts && Step.Track != Track && Conflicts->Contains(Step.Track))
			{
				return true;
			}
		}
	}
	return false;
}

bool AATTrafficSubsystem::IsClearingInTime(const FATTrafficTrain& Self, const FATTrafficTrain& Holder, const AFGBuildableRailroadTrack* Track, int32 Joint,
	float SelfDistance) const
{
	const AFGTrain* HolderTrain = Holder.Train.Get();
	const AFGTrain* SelfTrain = Self.Train.Get();
	if (!HolderTrain || !SelfTrain || !Holder.bManaged || Holder.Speed < FollowMinSpeed)
	{
		return false;
	}
	// The holder must keep moving: its own authority has to reach past its braking distance and its length.
	const float HolderAuthority = GetAuthorityDistance(HolderTrain);
	const float HolderBrake = FMath::Square(Holder.Speed) / (2.f * GetPlanningDeceleration(Holder));
	if (!Holder.bAuthorityToGoal && HolderAuthority - Holder.StopMargin < HolderBrake + Holder.Length)
	{
		return false;
	}

	const float ClearDistance = GetClearDistance(Holder, Track, Joint);
	// A holder that reaches its station first stops there instead of clearing.
	if (Holder.bAuthorityToGoal && Holder.Route.Num() > 0 && Holder.Route.Last().EndDistance - Holder.FrontAhead < ClearDistance + HolderBrake)
	{
		return false;
	}
	const float ClearTime = ClearDistance / Holder.Speed;
	const float Arrival = TimeToCover(FMath::Max(0.f, SelfDistance), Self.Speed, GetAcceleration(Self, 1.f), SelfTrain->mConsistData.MaxSpeed);
	return ClearTime + FollowTimeMargin() < Arrival;
}

float AATTrafficSubsystem::GetClearDistance(const FATTrafficTrain& Holder, const AFGBuildableRailroadTrack* Track, int32 Joint) const
{
	// Past the end of the track or the switch's zone; a track behind the holder's route is one its tail is on.
	for (const FATRouteStep& Step : Holder.Route)
	{
		const bool bThere = Joint != INDEX_NONE ? GetJoint(Step.Exit) == Joint : Step.Track == Track;
		if (bThere)
		{
			return Step.EndDistance - Holder.FrontAhead + Holder.Length + (Joint != INDEX_NONE ? JointFoulDistance : 0.f);
		}
	}
	return Holder.Length + JointFoulDistance;
}

float AATTrafficSubsystem::GetBlockerClearSeconds(const FATTrafficTrain& State) const
{
	const FATTrafficTrain* Blocker = mTrains.Find(State.BlockedBy);
	const AFGTrain* BlockerTrain = Blocker ? Blocker->Train.Get() : nullptr;
	if (!BlockerTrain || !Blocker->bManaged || Blocker->StoppedSeconds >= LeaderStopSeconds || Blocker->Route.Num() == 0)
	{
		return -1.f;
	}
	// Where State meets the blocker: the first track beyond its claims the blocker holds or crosses, or the switch
	// before it.
	float ClearDistance = -1.f;
	bool bPastClaims = false;
	for (int32 Index = 0; Index < State.Route.Num() && ClearDistance < 0.f; ++Index)
	{
		const FATRouteStep& Step = State.Route[Index];
		if (!bPastClaims && HoldsTrack(State, Step.Track))
		{
			continue;
		}
		bPastClaims = true;
		const int32 Joint = Index > 0 ? GetJoint(State.Route[Index - 1].Exit) : INDEX_NONE;
		if (Joint != INDEX_NONE && HoldsJoint(*Blocker, Joint))
		{
			ClearDistance = GetClearDistance(*Blocker, nullptr, Joint);
			break;
		}
		for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Step.Track))
		{
			if (HoldsTrack(*Blocker, Conflict))
			{
				ClearDistance = GetClearDistance(*Blocker, Conflict, INDEX_NONE);
				break;
			}
		}
	}
	if (ClearDistance < 0.f)
	{
		return -1.f;
	}
	// The blocker must be able to get that far without stopping, and not stop at its station first.
	const float Reach = Blocker->bAuthorityToGoal ? Blocker->Route.Last().EndDistance - Blocker->FrontAhead
		: GetAuthorityDistance(BlockerTrain) - Blocker->StopMargin;
	if (Reach < ClearDistance)
	{
		return -1.f;
	}
	return TimeToCover(ClearDistance, Blocker->Speed, GetAcceleration(*Blocker, 1.f), BlockerTrain->mConsistData.MaxSpeed);
}

bool AATTrafficSubsystem::IsLeaderOf(const FATTrafficTrain& Self, const FATTrafficTrain& Other) const
{
	if (!Other.bManaged || Other.Route.Num() == 0 || Self.Route.Num() == 0)
	{
		return false;
	}
	const FATRouteStep& OtherStep = Other.Route[0];
	for (int32 Index = 0; Index < Self.Route.Num(); ++Index)
	{
		if (Self.Route[Index].Track != OtherStep.Track)
		{
			continue;
		}
		if (Self.Route[Index].Exit != OtherStep.Exit)
		{
			return false;
		}
		// On the same track: the one nearer its end is ahead.
		return Index > 0 || OtherStep.EndDistance - Other.FrontAhead < Self.Route[0].EndDistance - Self.FrontAhead;
	}
	// A train whose route runs over the track Self is on now comes up behind it.
	const AFGBuildableRailroadTrack* SelfTrack = Self.Route[0].Track;
	return !Other.Route.ContainsByPredicate([SelfTrack](const FATRouteStep& Step) { return Step.Track == SelfTrack; });
}

bool AATTrafficSubsystem::IsSameWayLeader(const FATTrafficTrain& Self, const FATTrafficTrain& Holder, int32 FromIndex) const
{
	if (!Holder.bManaged || Holder.Route.Num() == 0 || Holder.StoppedSeconds >= LeaderStopSeconds)
	{
		return false;
	}
	const FATRouteStep& HolderStep = Holder.Route[0];
	for (int32 Index = FMath::Max(FromIndex, 0); Index < Self.Route.Num(); ++Index)
	{
		if (Self.Route[Index].Track == HolderStep.Track)
		{
			return Self.Route[Index].Exit == HolderStep.Exit;
		}
	}
	return false;
}

void AATTrafficSubsystem::RevalidateFollowClaims(FATTrafficTrain& State)
{
	for (int32 Index = 0; Index < State.Route.Num(); ++Index)
	{
		const FATRouteStep& Step = State.Route[Index];
		if (State.Occupied.Contains(Step.Track))
		{
			continue;
		}
		if (!HoldsTrack(State, Step.Track))
		{
			return;
		}
		const int32 Joint = Index > 0 ? GetJoint(State.Route[Index - 1].Exit) : INDEX_NONE;
		// Shared claims stay while the leader keeps moving; the follow limit keeps the distance. A leader that has
		// stopped for a while gets the track to itself again.
		const FATTrafficTrain* Stopped = nullptr;
		for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
		{
			const FATTrafficTrain& Other = Pair.Value;
			if (&Other == &State || Other.StoppedSeconds < LeaderStopSeconds || !IsLeaderOf(State, Other))
			{
				continue;
			}
			bool bShares = Joint != INDEX_NONE && HoldsJoint(Other, Joint);
			for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Step.Track))
			{
				bShares = bShares || HoldsTrack(Other, Conflict);
			}
			if (bShares)
			{
				Stopped = &Other;
				break;
			}
		}
		if (!Stopped)
		{
			continue;
		}
		TSet<const AFGBuildableRailroadTrack*> Dropped;
		for (int32 Later = Index; Later < State.Route.Num(); ++Later)
		{
			if (!State.Occupied.Contains(State.Route[Later].Track))
			{
				Dropped.Add(State.Route[Later].Track);
			}
		}
		const int32 Before = State.Claims.Num();
		State.Claims.RemoveAll([&Dropped](const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim) { return Dropped.Contains(Claim.Get()); });
		UpdateAuthority(State);
		UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s gives up %d claims: %s has stopped on %s"), *TrainName(State.Train.Get()),
			Before - State.Claims.Num(), *TrainName(Stopped->Train.Get()), *TrackName(Step.Track));
		return;
	}
}

void AATTrafficSubsystem::UpdateFollowLeaders(FATTrafficTrain& State) const
{
	State.FollowLeaders.Reset();
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const FATTrafficTrain& Other = Pair.Value;
		if (&Other == &State)
		{
			continue;
		}
		if (!IsLeaderOf(State, Other))
		{
			continue;
		}
		const bool bShares = State.Claims.ContainsByPredicate([this, &Other](const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim)
		{
			const AFGBuildableRailroadTrack* Track = Claim.Get();
			if (!Track)
			{
				return false;
			}
			for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Track))
			{
				if (HoldsTrack(Other, Conflict))
				{
					return true;
				}
			}
			return false;
		}) || State.ClaimedJoints.Array().ContainsByPredicate([&Other](int32 Joint) { return HoldsJoint(Other, Joint); });
		if (bShares)
		{
			State.FollowLeaders.Add(Other.Train);
		}
	}
}

float AATTrafficSubsystem::GetDistanceToTrackStart(const AFGTrain* Train, const FATTrafficTrain& State, const AFGBuildableRailroadTrack* Track) const
{
	const FTrainAtcData& Atc = Train->mAtcData;
	if (!Atc.Path.IsValid() || !Atc.Path->PathPoints.IsValidIndex(Atc.CurrentPathSegment))
	{
		return TrackNotOnPath;
	}
	float Distance = Atc.CurrentConnectionDistance;
	for (int32 Index = Atc.CurrentPathSegment; Index < Atc.Path->PathPoints.Num(); ++Index)
	{
		const UFGRailroadTrackConnectionComponent* Exit = Atc.Path->PathPoints[Index].TrackConnection.Get();
		const AFGBuildableRailroadTrack* PathTrack = Exit ? Exit->GetTrack() : nullptr;
		if (!PathTrack)
		{
			break;
		}
		if (Index > Atc.CurrentPathSegment)
		{
			Distance += PathTrack->GetLength();
		}
		if (PathTrack == Track)
		{
			return Distance - PathTrack->GetLength() - State.FrontAhead;
		}
	}
	return TrackNotOnPath;
}

float AATTrafficSubsystem::GetFollowLimit(const AFGTrain* Train, const FATTrafficTrain& State) const
{
	float Limit = -1.f;
	for (const TWeakObjectPtr<AFGTrain>& LeaderTrain : State.FollowLeaders)
	{
		const FATTrafficTrain* Leader = mTrains.Find(LeaderTrain);
		if (!Leader || Leader->Route.Num() == 0)
		{
			continue;
		}
		const float LeaderBrake = FMath::Square(Leader->Speed) / (2.f * GetPlanningDeceleration(*Leader));
		// A leader ahead on this train's route: stop behind its tail, allowing for the distance the leader itself
		// needs to stop.
		const FATRouteStep& LeaderStep = Leader->Route[0];
		const bool bOnRoute = State.Route.ContainsByPredicate([&LeaderStep](const FATRouteStep& Step)
		{
			return Step.Track == LeaderStep.Track && Step.Exit == LeaderStep.Exit;
		});
		const float LeaderTrackStart = bOnRoute ? GetDistanceToTrackStart(Train, State, LeaderStep.Track) : TrackNotOnPath;
		if (LeaderTrackStart > TrackNotOnPath)
		{
			const float Tail = LeaderTrackStart + LeaderStep.Track->GetLength() - (LeaderStep.EndDistance - Leader->FrontAhead + Leader->Length);
			const float LeaderLimit = FMath::Max(Tail, 0.f) + LeaderBrake;
			Limit = Limit < 0.f ? LeaderLimit : FMath::Min(Limit, LeaderLimit);
			continue;
		}
		// A leader merging in ahead: stop before the first track along this train's route it holds (or crosses).
		for (const FATRouteStep& Step : State.Route)
		{
			bool bHeld = false;
			for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Step.Track))
			{
				bHeld = bHeld || HoldsTrack(*Leader, Conflict);
			}
			if (!bHeld)
			{
				continue;
			}
			const float Start = GetDistanceToTrackStart(Train, State, Step.Track);
			if (Start <= TrackNotOnPath)
			{
				break;
			}
			const float LeaderLimit = FMath::Max(Start, 0.f) + LeaderBrake;
			Limit = Limit < 0.f ? LeaderLimit : FMath::Min(Limit, LeaderLimit);
			break;
		}
	}
	return Limit;
}

bool AATTrafficSubsystem::IsOutOfOthersWay(const FATTrafficTrain& Self, const FATRouteStep& Step, FString& OutReason) const
{
	// Only trains coming the other way on this track matter; trains going the same way reach it from behind and queue.
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

void AATTrafficSubsystem::LogDeadlocks()
{
	// Follow each long-waiting train's chain of trains it waits for; a chain that comes back is a deadlock.
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const FATTrafficTrain& Start = Pair.Value;
		if (!Start.bManaged || Start.WaitingSeconds < DeadlockWaitSeconds)
		{
			continue;
		}
		TArray<const FATTrafficTrain*> Chain;
		const FATTrafficTrain* Current = &Start;
		while (Current && !Chain.Contains(Current) && Chain.Num() < 64)
		{
			Chain.Add(Current);
			Current = mTrains.Find(Current->BlockedBy);
			if (Current && (!Current->bManaged || Current->WaitingSeconds < DeadlockWaitSeconds))
			{
				Current = nullptr;
			}
		}
		if (Current != &Start)
		{
			continue;
		}
		TArray<FString> Names;
		for (const FATTrafficTrain* Member : Chain)
		{
			Names.Add(TrainName(Member->Train.Get()));
		}
		TArray<FString> Sorted = Names;
		Sorted.Sort();
		const FString Signature = FString::Join(Sorted, TEXT("|"));
		if (Signature == mLastDeadlock)
		{
			return;
		}
		mLastDeadlock = Signature;
		UE_LOG(LogAutoTrains, Warning, TEXT("traffic: DEADLOCK: %s -> %s"), *FString::Join(Names, TEXT(" -> ")), *Names[0]);
		for (const FATTrafficTrain* Member : Chain)
		{
			UE_LOG(LogAutoTrains, Warning, TEXT("traffic: DEADLOCK:   %s: %s"), *TrainName(Member->Train.Get()), *Member->BlockedReason);
		}
		return;
	}
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
	if (!Train || State.bAuthorityToGoal || State.Route.Num() < 3 || !Train->mAtcData.Path.IsValid() || State.RerouteCooldown > 0.f)
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
	State.RerouteCooldown = RerouteCooldownSeconds;
	UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s rerouted after %s: cost %.0f -> %.0f m, length %.0f -> %.0f m, applied %s (%.2f ms)"),
		*TrainName(Train), *TrackName(State.Route[Commit].Track), CurrentCost / 100.f, NewCost / 100.f, CurrentLength / 100.f,
		FATRouter::GetRouteLength(Alternative) / 100.f, bApplied ? TEXT("yes") : TEXT("NO"), (FPlatformTime::Seconds() - Begin) * 1000.0);
}

void AATTrafficSubsystem::RebuildConflicts()
{
	const double Begin = FPlatformTime::Seconds();
	mConflicts.Reset();
	mJointOf.Reset();

	TArray<AFGBuildableRailroadTrack*> Tracks;
	for (TActorIterator<AFGBuildableRailroadTrack> It(GetWorld()); It; ++It)
	{
		if (IsValid(*It))
		{
			Tracks.Add(*It);
		}
	}

	// Switch joints: the connections where three or more tracks meet.
	int32 JointCount = 0;
	for (AFGBuildableRailroadTrack* Track : Tracks)
	{
		for (int32 End = 0; End < 2; ++End)
		{
			const UFGRailroadTrackConnectionComponent* Connection = Track->GetConnection(End);
			if (!Connection)
			{
				continue;
			}
			TSet<const UFGRailroadTrackConnectionComponent*> Connections;
			Connections.Add(Connection);
			for (const UFGRailroadTrackConnectionComponent* Facing : Connection->GetConnections())
			{
				if (!Facing)
				{
					continue;
				}
				Connections.Add(Facing);
				for (const UFGRailroadTrackConnectionComponent* Beside : Facing->GetConnections())
				{
					if (Beside)
					{
						Connections.Add(Beside);
					}
				}
			}
			TSet<const AFGBuildableRailroadTrack*> JointTracks;
			for (const UFGRailroadTrackConnectionComponent* Member : Connections)
			{
				JointTracks.Add(Member->GetTrack());
			}
			if (JointTracks.Num() <= 2)
			{
				continue;
			}
			int32 Id = INDEX_NONE;
			for (const UFGRailroadTrackConnectionComponent* Member : Connections)
			{
				if (const int32* Existing = mJointOf.Find(Member))
				{
					Id = *Existing;
					break;
				}
			}
			if (Id == INDEX_NONE)
			{
				Id = JointCount++;
			}
			for (const UFGRailroadTrackConnectionComponent* Member : Connections)
			{
				mJointOf.Add(Member, Id);
			}
		}
	}

	// Crossings: the game's overlapping tracks, both ways, except branches of the same switch, which the joint covers.
	auto Entry = [this](AFGBuildableRailroadTrack* Track) -> TArray<AFGBuildableRailroadTrack*>&
	{
		TArray<AFGBuildableRailroadTrack*>& Conflicts = mConflicts.FindOrAdd(Track);
		if (Conflicts.Num() == 0)
		{
			Conflicts.Add(Track);
		}
		return Conflicts;
	};
	auto SharesJoint = [this](const AFGBuildableRailroadTrack* A, const AFGBuildableRailroadTrack* B)
	{
		for (int32 EndA = 0; EndA < 2; ++EndA)
		{
			const int32 Joint = GetJoint(A->GetConnection(EndA));
			if (Joint == INDEX_NONE)
			{
				continue;
			}
			for (int32 EndB = 0; EndB < 2; ++EndB)
			{
				if (GetJoint(B->GetConnection(EndB)) == Joint)
				{
					return true;
				}
			}
		}
		return false;
	};
	for (AFGBuildableRailroadTrack* Track : Tracks)
	{
		Entry(Track);
		for (AFGBuildableRailroadTrack* Overlapping : Track->GetOverlappingTracks())
		{
			if (Overlapping && Overlapping != Track && !SharesJoint(Track, Overlapping))
			{
				Entry(Track).AddUnique(Overlapping);
				Entry(Overlapping).AddUnique(Track);
			}
		}
	}
	int32 Crossings = 0;
	for (const TPair<const AFGBuildableRailroadTrack*, TArray<AFGBuildableRailroadTrack*>>& Pair : mConflicts)
	{
		Crossings += Pair.Value.Num() > 1 ? 1 : 0;
	}
	mConflictsBuiltAt = FPlatformTime::Seconds();
	mConflictsStale = false;
	UE_LOG(LogAutoTrains, Log, TEXT("traffic: conflict model rebuilt: %d tracks, %d switch joints, %d crossing tracks, %.1f ms"),
		Tracks.Num(), JointCount, Crossings, (mConflictsBuiltAt - Begin) * 1000.0);
}

const TArray<AFGBuildableRailroadTrack*>& AATTrafficSubsystem::GetConflicts(const AFGBuildableRailroadTrack* Track) const
{
	if (const TArray<AFGBuildableRailroadTrack*>* Conflicts = mConflicts.Find(Track))
	{
		return *Conflicts;
	}
	// A track built since the last rebuild; treat it as conflicting with itself only until the model catches up.
	mConflictsStale = true;
	static TArray<AFGBuildableRailroadTrack*> Single;
	Single.Reset();
	Single.Add(const_cast<AFGBuildableRailroadTrack*>(Track));
	return Single;
}

bool AATTrafficSubsystem::IsCrossing(const AFGBuildableRailroadTrack* Track) const
{
	return Track && GetConflicts(Track).Num() > 1;
}

int32 AATTrafficSubsystem::GetJoint(const UFGRailroadTrackConnectionComponent* Connection) const
{
	const int32* Joint = Connection ? mJointOf.Find(Connection) : nullptr;
	return Joint ? *Joint : INDEX_NONE;
}

float AATTrafficSubsystem::GetStopMargin(const UFGRailroadTrackConnectionComponent* Exit) const
{
	return GetJoint(Exit) != INDEX_NONE ? SwitchStopMargin : AuthorityStopMargin;
}

bool AATTrafficSubsystem::HoldsTrack(const FATTrafficTrain& State, const AFGBuildableRailroadTrack* Track) const
{
	return State.Occupied.Contains(const_cast<AFGBuildableRailroadTrack*>(Track))
		|| State.Claims.ContainsByPredicate([Track](const TWeakObjectPtr<AFGBuildableRailroadTrack>& Claim) { return Claim.Get() == Track; });
}

bool AATTrafficSubsystem::HoldsJoint(const FATTrafficTrain& State, int32 Joint)
{
	return State.OccupiedJoints.Contains(Joint) || State.ClaimedJoints.Contains(Joint);
}

float AATTrafficSubsystem::GetPlanningDeceleration(const FATTrafficTrain& State, float Speed)
{
	const AFGTrain* Train = State.Train.Get();
	const float Mass = GetMass(Train);
	const int32 Top = GetBrakeBand(Speed >= 0.f ? Speed : State.Speed);
	// A stop from this speed brakes through every band below it. Braking is weaker at higher speed, so a measured
	// band above this one bounds it too.
	float Lower = MAX_flt;
	float Higher = MAX_flt;
	bool bTopMeasured = false;
	for (int32 Band = 0; Band < ATBrakeBands; ++Band)
	{
		if (State.BrakeForce[Band] <= 0.f)
		{
			continue;
		}
		const float Deceleration = State.BrakeForce[Band] / Mass;
		if (Band <= Top)
		{
			Lower = FMath::Min(Lower, Deceleration);
			bTopMeasured = bTopMeasured || Band == Top;
		}
		else
		{
			Higher = FMath::Min(Higher, Deceleration);
		}
	}
	float Measured;
	if (bTopMeasured)
	{
		Measured = Lower;
	}
	else if (Higher < MAX_flt)
	{
		Measured = FMath::Min(Lower, Higher);
	}
	else if (Lower < MAX_flt)
	{
		Measured = Lower * UnmeasuredBandShare;
	}
	else
	{
		const float Rated = Train ? Train->mConsistData.MaxBrakeDeceleration : 100.f;
		return FMath::Clamp(Rated * RatedBrakeShare(), 50.f, 800.f);
	}
	return FMath::Clamp(Measured * MeasuredBrakeShare(), 50.f, 800.f);
}

float AATTrafficSubsystem::GetClaimPriority(const FATTrafficTrain& State) const
{
	const AFGTrain* Train = State.Train.Get();
	int32 Index = 0;
	while (Index < State.Route.Num() && HoldsTrack(State, State.Route[Index].Track))
	{
		++Index;
	}
	if (!Train || Index >= State.Route.Num())
	{
		return -1e6f;
	}
	// Time until the train's tail has passed the first track it still needs.
	const float Distance = State.Route[Index].EndDistance - State.FrontAhead + State.Length;
	const float ClearTime = TimeToCover(Distance, State.Speed, GetAcceleration(State, 1.f), Train->mConsistData.MaxSpeed);
	return ClearTime - State.WaitingSeconds * WaitingWeight();
}

float AATTrafficSubsystem::GetAcceleration(const FATTrafficTrain& State, float Share)
{
	const AFGTrain* Train = State.Train.Get();
	const float Mass = Train ? FMath::Max(Train->mConsistData.Mass, 1.f) : 1.f;
	const float Tractive = Train ? Train->mConsistData.MaxTractiveEffort : 0.f;
	return FMath::Clamp(Tractive / Mass * Share, 20.f, 600.f);
}

float AATTrafficSubsystem::TimeToCover(float Distance, float Speed, float Acceleration, float MaxSpeed)
{
	if (Distance <= 0.f)
	{
		return 0.f;
	}
	const float Top = FMath::Max3(MaxSpeed, Speed, 100.f);
	const float Start = FMath::Min(Speed, Top);
	const float ToTop = (FMath::Square(Top) - FMath::Square(Start)) / (2.f * Acceleration);
	if (Distance <= ToTop)
	{
		return (FMath::Sqrt(FMath::Square(Start) + 2.f * Acceleration * Distance) - Start) / Acceleration;
	}
	return (Top - Start) / Acceleration + (Distance - ToTop) / Top;
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
	// Behind a leader on shared track: keep able to stop where the leader could stop.
	const float FollowLimit = GetFollowLimit(Train, *State);
	if (Distance < 0.f && FollowLimit < 0.f)
	{
		State->bFullBrake = false;
		State->bCatchArmed = false;
		return;
	}

	FTrainAtcData& Atc = Train->mAtcData;
	const float Speed = FMath::Abs(Train->mSimulationData.Velocity);
	const float Usable = Distance - State->StopMargin;
	const float FollowUsable = FollowLimit - AuthorityStopMargin;
	auto GetAllowed = [Distance, FollowLimit, Usable, FollowUsable](float Deceleration)
	{
		float Result = MAX_flt;
		if (Distance >= 0.f)
		{
			Result = Usable > 0.f ? FMath::Sqrt(2.f * Deceleration * Usable) : 0.f;
		}
		if (FollowLimit >= 0.f)
		{
			Result = FMath::Min(Result, FollowUsable > 0.f ? FMath::Sqrt(2.f * Deceleration * FollowUsable) : 0.f);
		}
		return Result;
	};
	const float Deceleration = GetPlanningDeceleration(*State, Speed);
	const float Allowed = GetAllowed(Deceleration);
	float Limit = MAX_flt;
	if (Distance >= 0.f)
	{
		Limit = Distance;
	}
	if (FollowLimit >= 0.f)
	{
		Limit = FMath::Min(Limit, FollowLimit);
	}
	// Braking is weaker at higher speed: the speed it may still reach is planned with that speed's braking.
	float Target = Allowed;
	if (Allowed > Speed && Allowed < MAX_flt)
	{
		Target = FMath::Min(Allowed, GetAllowed(FMath::Min(Deceleration, GetPlanningDeceleration(*State, Allowed))));
	}
	// Waiting for a train that will clear: drive at the speed that reaches the braking point as it clears, so the
	// train rolls on instead of stopping.
	if (Distance >= 0.f && Usable > 0.f && State->ClearSeconds > 0.f)
	{
		const float Seconds = State->ClearSeconds;
		Target = FMath::Min(Target, Deceleration * (FMath::Sqrt(FMath::Square(Seconds) + 2.f * Usable / Deceleration) - Seconds));
	}
	const bool bWasFullBrake = State->bFullBrake;
	State->bFullBrake = Speed > Allowed + OverspeedTolerance;
	Atc.TargetSpeed = State->bFullBrake ? 0.f : FMath::Min(Atc.TargetSpeed, Target);
	if (State->bFullBrake && !bWasFullBrake && Speed > Allowed + 500.f)
	{
		UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s over its braking curve (%.0f km/h, allowed %.0f, %.0f m to go%s): full brake"), *TrainName(Train),
			Speed * 0.036f, Allowed * 0.036f, Limit / 100.f, Limit == FollowLimit ? TEXT(", behind a leader") : TEXT(""));
	}
	if (Distance < 0.f)
	{
		State->bCatchArmed = false;
		return;
	}

	// Backstop: when the stop point lies before the master's next connection, point the restrictive point at that
	// connection with the stop offset that puts the stop there, so the game's catch holds the train at the stop point
	// whatever its speed. Clear keeps the wait timer and docking out.
	const UFGRailroadTrackConnectionComponent* Next = Atc.CurrentConnection.Get();
	const float ToStop = Distance - State->StopMargin;
	const float StopOffset = Atc.CurrentConnectionDistance - ToStop;
	const bool bArm = Next && ToStop > 0.f && StopOffset >= 0.f
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
		Lines.Add(FString::Printf(TEXT("%s: %.0f km/h, %d claims, may drive %s, brakes at %.1f m/s2, waited %.0f s, rerouted %d times%s%s%s"), *TrainName(State.Train.Get()),
			State.Speed * 0.036f, State.Claims.Num(),
			Distance < 0.f ? TEXT("to its stop") : *FString::Printf(TEXT("%.0f m"), FMath::Max(0.f, Distance - State.StopMargin) / 100.f),
			GetPlanningDeceleration(State) / 100.f, State.WaitingSeconds, State.Reroutes, State.BlockedReason.IsEmpty() ? TEXT("") : TEXT(", "), *State.BlockedReason,
			State.ClearSeconds > 0.f ? *FString::Printf(TEXT(" (clears in %.0f s)"), State.ClearSeconds) : TEXT("")));
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
