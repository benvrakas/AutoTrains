#include "ATTrafficSubsystem.h"
#include "ATConfig.h"
#include "ATControlSubsystem.h"
#include "ATRouter.h"
#include "AutoTrains.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "EngineUtils.h"
#include "Curves/CurveFloat.h"
#include "FGLocomotive.h"
#include "FGLocomotiveMovementComponent.h"
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
	// Top speed of every self-driving train, 0 for each train's own [cm/s], and the acceleration multiplier.
	float GTopSpeed = 0.f;
	float GAccelerationScale = 1.f;
	int32 GPhaseThroughCount = 0;
	// Tracks the two vehicles of the last contact let through were on.
	TWeakObjectPtr<AFGBuildableRailroadTrack> GPhaseTrack;
	TWeakObjectPtr<AFGBuildableRailroadTrack> GPhaseOtherTrack;

	constexpr float TrafficUpdateInterval = 0.25f;
	constexpr float SettingsInterval = 1.f;
	// Claims reach this far past the braking distance plus the train's length; less when aggressive. [cm]
	float ClaimLookahead() { return GAggressive ? 10000.f : 20000.f; }
	// Trains stop this far before the end of their authority, or SwitchStopMargin before a switch. [cm]
	constexpr float AuthorityStopMargin = 1000.f;
	constexpr float SwitchStopMargin = 2500.f;
	// Distance along each track from a switch point within which a vehicle fouls the switch. [cm]
	constexpr float JointFoulDistance = 2000.f;
	// A vehicle counts as on the next track only once it reaches this far past the end of its own; station tracks
	// are exactly one vehicle long. [cm]
	constexpr float OverhangTolerance = 50.f;
	// Other trains' routes count for stopping spots up to this far ahead of them. [cm]
	constexpr float OthersRouteHorizon = 300000.f;
	// Planning deceleration: shares of the measured full-brake deceleration and of the consist's rating. Loaded trains
	// were measured braking at 2/3 of their rating, light ones at 1.4x.
	float MeasuredBrakeShare() { return GAggressive ? 0.95f : 0.85f; }
	float RatedBrakeShare() { return GAggressive ? 0.8f : 0.6f; }
	// Above every measured band the plan scales the highest one down in proportion to speed, but not below this share
	// of the rating; a train never measured plans with its rated share up to FallbackFullSpeed and scales it down in
	// proportion above. Loaded trains were measured at 1.2 m/s2 at 100 km/h and 5.8 at 40 km/h. [cm/s]
	constexpr float MinRatedBrakeShare = 0.4f;
	constexpr float FallbackFullSpeed = 1400.f;
	// Seconds the brakes take to build up, counted as travel at the starting speed; about 1 s in the measurements.
	constexpr float BrakeLagSeconds = 0.75f;
	constexpr float MinPlanDeceleration = 50.f;
	// Forced braking plans the stop with at least this distance left, so the last tick doesn't divide by nothing. [cm]
	constexpr float ForcedBrakeMinDistance = 100.f;
	// A brake sample counts for at most this multiple of the consist's rating; stronger ones came from catches or
	// contact with another train.
	constexpr float MaxRatedBrakeMultiple = 1.25f;
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
	// A train behind a leader crossing its route drives so it gets there this much later than the time the leader
	// clears it plus the safety margin. [s]
	constexpr float CrossingPaceSlackSeconds = 1.f;
	// How far along its path a follower looks for the vehicles of the trains it follows. [cm]
	constexpr float GapSearchDistance = 300000.f;
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
	// A train waiting for the train being routed can't get out of its way, and one that holds track the other way may
	// stop on it facing the train being routed: a new route doesn't run against held track.
	constexpr float WaitingForSelfPenalty = 10000000.f;
	constexpr float HeadOnPenalty = 10000000.f;
	constexpr float ClaimPenalty = 30000.f;
	constexpr float UpcomingSameWayPenalty = 5000.f;
	constexpr float UpcomingOtherWayPenalty = 80000.f;
	// A new route must be cheaper than the current one by this much and this share to be taken.
	constexpr float RerouteMinGain = 30000.f;
	constexpr float RerouteMinGainShare = 0.15f;
	// A train that rerouted doesn't reroute again for this long, so it can't flip between two routes. [s]
	constexpr float RerouteCooldownSeconds = 30.f;

	// Top speed a train runs to: the setting, or the consist's own. [cm/s]
	float GetTopSpeed(const AFGTrain* Train)
	{
		return GTopSpeed > 0.f ? GTopSpeed : (Train ? Train->mConsistData.MaxSpeed : 0.f);
	}

	// A locomotive's traction is stretched to this much over the top speed setting, so it still pulls there.
	constexpr float TopSpeedTractionHeadroom = 1.05f;

	// Speed at which a traction curve has dropped to almost nothing past its peak, at most MaxVelocity. [km/h]
	float GetTractionTop(const FRichCurve& Traction, float MaxVelocity)
	{
		float Peak = 0.f;
		for (float Speed = 0.f; Speed <= MaxVelocity; Speed += 1.f)
		{
			const float Value = Traction.Eval(Speed);
			Peak = FMath::Max(Peak, Value);
			if (Peak > 0.f && Value <= Peak * 0.01f)
			{
				return Speed;
			}
		}
		return MaxVelocity;
	}

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
			// Name the trains standing on the tracks of the last contact.
			auto NamesOn = [this](const AFGBuildableRailroadTrack* Track)
			{
				TArray<FString> Names;
				for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
				{
					if (Track && Pair.Value.Occupied.Contains(const_cast<AFGBuildableRailroadTrack*>(Track)))
					{
						Names.Add(TrainName(Pair.Value.Train.Get()));
					}
				}
				return Names.Num() > 0 ? FString::Join(Names, TEXT("/")) : FString(TEXT("?"));
			};
			UE_LOG(LogAutoTrains, Log, TEXT("traffic: trains passed through each other %d times in the last 10 s; last %s on %s and %s on %s"), GPhaseThroughCount,
				*NamesOn(GPhaseTrack.Get()), *TrackName(GPhaseTrack.Get()), *NamesOn(GPhaseOtherTrack.Get()), *TrackName(GPhaseOtherTrack.Get()));
			GPhaseThroughCount = 0;
		}
	}
}

void AATTrafficSubsystem::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	GPhaseThrough = false;
	RestoreLocomotivePower();
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
	const float TopSpeed = Settings.TopSpeedKmh / 0.036f;
	if (bPhaseThrough != GPhaseThrough || Settings.bAggressive != GAggressive || Settings.SafetyMargin != GSafetyMargin || TopSpeed != GTopSpeed
		|| Settings.Acceleration != GAccelerationScale)
	{
		UE_LOG(LogAutoTrains, Log, TEXT("Settings: trains pass through each other %s, aggressive timing %s, safety margin %.1f s, top speed %s, acceleration x%.2f"),
			bPhaseThrough ? TEXT("on") : TEXT("off"), Settings.bAggressive ? TEXT("on") : TEXT("off"), Settings.SafetyMargin,
			TopSpeed > 0.f ? *FString::Printf(TEXT("%.0f km/h"), Settings.TopSpeedKmh) : TEXT("each train's own"), Settings.Acceleration);
	}
	GTopSpeed = TopSpeed;
	GAccelerationScale = Settings.Acceleration;
	GPhaseThrough = bPhaseThrough;
	GAggressive = Settings.bAggressive;
	GSafetyMargin = Settings.SafetyMargin;
}

bool AATTrafficSubsystem::ShouldPhaseThrough()
{
	return GPhaseThrough;
}

void AATTrafficSubsystem::CountPhaseThrough(AFGBuildableRailroadTrack* Track, AFGBuildableRailroadTrack* OtherTrack)
{
	++GPhaseThroughCount;
	GPhaseTrack = Track;
	GPhaseOtherTrack = OtherTrack;
}

FString AATTrafficSubsystem::SetEnabled(bool bEnabled)
{
	mEnabled = bEnabled;
	if (!bEnabled)
	{
		GPhaseThrough = false;
		RestoreLocomotivePower();
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
		ApplyLocomotivePower(Train, Train->IsSelfDrivingEnabled() ? GAccelerationScale : 1.f, Train->IsSelfDrivingEnabled() ? GTopSpeed * 0.036f : 0.f);

		// Measure full-brake deceleration per speed band between updates with full brake on at both; a catch ends at
		// standstill, and docking catches and crashes are far above 20 m/s^2.
		if (State.bFullBrake && State.bWasFullBrake && !State.bForcedSinceUpdate && State.LastSpeed > 500.f && State.Speed > 300.f && DeltaSeconds > 0.f)
		{
			const float Raw = (State.LastSpeed - State.Speed) / DeltaSeconds;
			if (Raw > 50.f && Raw < 2000.f)
			{
				const float Deceleration = FMath::Min(Raw, Train->mConsistData.MaxBrakeDeceleration * MaxRatedBrakeMultiple);
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
		State.bForcedSinceUpdate = State.bForcedBrake;
		State.LastSpeed = State.Speed;
		RefreshBrakePlan(State);
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
	State.Vehicles.Reset();
	State.OccupiedJoints.Reset();
	State.FrontAhead = 0.f;
	State.RoutePath = nullptr;

	const FTrainAtcData& Atc = Train->mAtcData;
	State.bManaged = Train->IsSelfDrivingEnabled() && !Train->IsDocked() && Atc.Path.IsValid()
		&& Atc.Path->PathPoints.IsValidIndex(Atc.CurrentPathSegment);
	if (State.bManaged)
	{
		State.RoutePath = Atc.Path.Get();
		State.RouteFirstSegment = Atc.CurrentPathSegment;
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
		State.Vehicles.Add({Track, Position.Offset, HalfLength});
		for (int32 End = 0; End < 2; ++End)
		{
			const UFGRailroadTrackConnectionComponent* Connection = Track->GetConnection(End);
			if (!Connection)
			{
				continue;
			}
			const float ToEnd = FMath::Abs(Connection->GetTrackPosition().Offset - Position.Offset);
			// A vehicle reaching past a track end overhangs onto the tracks attached there; switches nearby are
			// covered by the fouling zone below.
			if (ToEnd < HalfLength - OverhangTolerance)
			{
				AddAttachedTracks(Connection, State.Occupied);
			}
			const int32 Joint = GetJoint(Connection);
			if (Joint != INDEX_NONE && ToEnd < HalfLength + JointFoulDistance)
			{
				State.OccupiedJoints.Add(Joint);
			}
		}
		// ATC distances are from the MU master; find how far the train's front is ahead of it. Only tracks within the
		// train's length count: a route can come back over a track the train stands on.
		for (const FATRouteStep& Step : State.Route)
		{
			if (Step.EndDistance - Step.Track->GetLength() > State.Length)
			{
				break;
			}
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
		State.FollowedTrains.Reset();
		State.AuthorityTrack = nullptr;
		State.bAuthorityToGoal = false;
		State.BlockedReason.Reset();
		State.BlockedBy = nullptr;
		State.WaitingSeconds = 0.f;
		State.bFullBrake = false;
		State.bForcedBrake = false;
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
	const float Need = GetStoppingDistance(State, State.Speed) + State.Length + ClaimLookahead();

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
	// Two trains never follow each other.
	auto FollowsThis = [&State](const FATTrafficTrain* Holder)
	{
		return Holder->FollowedTrains.Contains(State.Train);
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
			if (!IsPreempted(Holder) && !FollowsThis(Holder) && ((GAggressive && IsSameWayLeader(State, *Holder, Index - 1, nullptr, Joint))
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
			if (!IsPreempted(Holder) && !FollowsThis(Holder) && ((GAggressive && IsSameWayLeader(State, *Holder, Index, Conflict, INDEX_NONE))
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
					if (!FollowsThis(Other) && ((GAggressive && IsSameWayLeader(State, *Other, PendingIndex - 1, nullptr, PendingJoint))
						|| IsClearingInTime(State, *Other, nullptr, PendingJoint, PendingStart - JointFoulDistance)))
					{
						PendingLeaving.Add(Other);
						Followed.Add(Other);
						continue;
					}
					bFree = false;
					break;
				}
				while (bFree && !IsTrackFree(PendingStep.Track, State, Other, OtherConflict, bOtherStands, &PendingLeaving))
				{
					if (!FollowsThis(Other) && ((GAggressive && IsSameWayLeader(State, *Other, PendingIndex, OtherConflict, INDEX_NONE))
						|| IsClearingInTime(State, *Other, OtherConflict, INDEX_NONE, PendingStart)))
					{
						PendingLeaving.Add(Other);
						Followed.Add(Other);
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
		for (const FATTrafficTrain* Leader : Followed)
		{
			State.FollowedTrains.Add(Leader->Train);
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
	State.AuthorityPath = nullptr;
	State.AuthoritySegment = INDEX_NONE;
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
		State.AuthorityPath = State.RoutePath;
		State.AuthoritySegment = State.RouteFirstSegment + Last;
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
		GetTopSpeed(SelfTrain));
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
		const float HolderStop = GetStoppingDistance(Holder, Holder.Speed) + Holder.StopMargin + PreemptStopBuffer();
		if (HolderDistance < HolderStop)
		{
			OutReason = FString::Printf(TEXT("waits for %s, which has claimed %s and is too close to give it up (%.0f m)"), *TrainName(Holder.Train.Get()), *What,
				HolderDistance / 100.f);
			return false;
		}
		const AFGTrain* HolderTrain = Holder.Train.Get();
		const float HolderTime = TimeToCover(HolderDistance, Holder.Speed, GetAcceleration(Holder, 1.f), GetTopSpeed(HolderTrain));
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
	// A holder behind Self on its own way has to pass where Self is first.
	if (GetRouteOrder(Holder, Self) > 0)
	{
		return false;
	}
	// The holder must keep moving: its own authority has to reach past its braking distance and its length.
	const float HolderAuthority = GetAuthorityDistance(HolderTrain);
	const float HolderBrake = GetStoppingDistance(Holder, Holder.Speed);
	if (!Holder.bAuthorityToGoal && HolderAuthority - Holder.StopMargin < HolderBrake + Holder.Length)
	{
		return false;
	}

	if (!IsClearedPast(Holder, Track, Joint))
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
	const float Arrival = TimeToCover(FMath::Max(0.f, SelfDistance), Self.Speed, GetAcceleration(Self, 1.f), GetTopSpeed(SelfTrain));
	return ClearTime + FollowTimeMargin() < Arrival;
}

bool AATTrafficSubsystem::IsClearedPast(const FATTrafficTrain& Holder, const AFGBuildableRailroadTrack* Track, int32 Joint) const
{
	const float Reach = Holder.bAuthorityToGoal ? (Holder.Route.Num() > 0 ? Holder.Route.Last().EndDistance - Holder.FrontAhead : 0.f)
		: GetAuthorityDistance(Holder.Train.Get()) - Holder.StopMargin;
	return Reach >= GetClearDistance(Holder, Track, Joint);
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
	return TimeToCover(ClearDistance, Blocker->Speed, GetAcceleration(*Blocker, 1.f), GetTopSpeed(BlockerTrain));
}

int32 AATTrafficSubsystem::GetRouteOrder(const FATTrafficTrain& Self, const FATTrafficTrain& Other) const
{
	if (Other.Route.Num() == 0 || Self.Route.Num() == 0)
	{
		return 0;
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
			return -1;
		}
		// On the same track: the one nearer its end is ahead.
		return Index > 0 || OtherStep.EndDistance - Other.FrontAhead < Self.Route[0].EndDistance - Self.FrontAhead ? 1 : -1;
	}
	return 0;
}

bool AATTrafficSubsystem::IsLeaderOf(const FATTrafficTrain& Self, const FATTrafficTrain& Other) const
{
	if (!Other.bManaged)
	{
		return false;
	}
	const int32 Order = GetRouteOrder(Self, Other);
	// Merging or crossing traffic leads only if Self chose to follow it.
	return Order != 0 ? Order > 0 : Self.FollowedTrains.Contains(Other.Train);
}

bool AATTrafficSubsystem::IsSameWayLeader(const FATTrafficTrain& Self, const FATTrafficTrain& Holder, int32 FromIndex, const AFGBuildableRailroadTrack* Track,
	int32 Joint) const
{
	if (!Holder.bManaged || Holder.Route.Num() == 0 || Holder.StoppedSeconds >= LeaderStopSeconds || GetRouteOrder(Self, Holder) <= 0
		|| !IsClearedPast(Holder, Track, Joint))
	{
		return false;
	}
	// A holder that stops at its station on this train's way would only be queued behind.
	const AFGBuildableRailroadTrack* HolderGoal = Holder.Route.Last().Track;
	if (Self.Route.ContainsByPredicate([HolderGoal](const FATRouteStep& Step) { return Step.Track == HolderGoal; }))
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
			// A train this mod doesn't drive (docked, player driven) counts as stopped.
			if (&Other == &State || (Other.bManaged && (Other.StoppedSeconds < LeaderStopSeconds || !IsLeaderOf(State, Other))))
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
	const TArray<TWeakObjectPtr<AFGTrain>> Previous = State.FollowLeaders;
	State.FollowLeaders.Reset();
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const FATTrafficTrain& Other = Pair.Value;
		if (&Other == &State)
		{
			continue;
		}
		if (Other.bManaged && !IsLeaderOf(State, Other))
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
	for (auto It = State.FollowedTrains.CreateIterator(); It; ++It)
	{
		if (!State.FollowLeaders.Contains(*It))
		{
			It.RemoveCurrent();
		}
	}
	State.LeadersMovingAway.Reset();
	for (const TWeakObjectPtr<AFGTrain>& LeaderTrain : State.FollowLeaders)
	{
		const FATTrafficTrain* Leader = mTrains.Find(LeaderTrain);
		if (!Leader || !Leader->bManaged || GetRouteOrder(State, *Leader) < 0)
		{
			continue;
		}
		// A leader whose route runs over this train's route the other way comes towards it.
		const bool bOpposes = Leader->Route.ContainsByPredicate([&State](const FATRouteStep& LeaderStep)
		{
			return State.Route.ContainsByPredicate([&LeaderStep](const FATRouteStep& Step) { return Step.Track == LeaderStep.Track && Step.Exit != LeaderStep.Exit; });
		});
		if (!bOpposes)
		{
			State.LeadersMovingAway.Add(LeaderTrain);
		}
	}
	const double Now = GetWorld()->GetTimeSeconds();
	for (auto It = State.FollowLeaderSince.CreateIterator(); It; ++It)
	{
		if (!State.FollowLeaders.Contains(It.Key()))
		{
			It.RemoveCurrent();
		}
	}
	for (const TWeakObjectPtr<AFGTrain>& Leader : State.FollowLeaders)
	{
		if (!Previous.Contains(Leader) || !State.FollowLeaderSince.Contains(Leader))
		{
			State.FollowLeaderSince.Add(Leader, Now);
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

float AATTrafficSubsystem::GetFollowLimit(const AFGTrain* Train, const FATTrafficTrain& State, float& OutHardLimit, float& OutPaceSpeed, FATFollowInfo* OutInfo) const
{
	float Limit = -1.f;
	OutHardLimit = -1.f;
	OutPaceSpeed = MAX_flt;
	const FATTrafficTrain* Current = nullptr;
	auto Add = [&Limit, &OutHardLimit, &Current, OutInfo](float Gap, float SoftBrake, float HardBrake, bool bVehicleOnPath)
	{
		const float Hard = Gap + HardBrake;
		if (OutHardLimit < 0.f || Hard < OutHardLimit)
		{
			OutHardLimit = Hard;
			if (OutInfo)
			{
				OutInfo->Leader = Current->Train;
				OutInfo->Gap = Gap;
				OutInfo->LeaderBrake = HardBrake;
				OutInfo->bVehicleOnPath = bVehicleOnPath;
			}
		}
		Limit = Limit < 0.f ? Gap + SoftBrake : FMath::Min(Limit, Gap + SoftBrake);
	};
	for (const TWeakObjectPtr<AFGTrain>& LeaderTrain : State.FollowLeaders)
	{
		const FATTrafficTrain* Leader = mTrains.Find(LeaderTrain);
		const AFGTrain* LeaderTrainNow = Leader ? Leader->Train.Get() : nullptr;
		if (!LeaderTrainNow)
		{
			continue;
		}
		Current = Leader;
		// The braking distance of a leader from its speed now: with the strongest braking a sample counts for, and
		// with its planning braking. A train this mod doesn't drive is a standing obstacle.
		float SoftBrake = 0.f;
		float HardBrake = 0.f;
		if (Leader->bManaged)
		{
			const float LeaderSpeed = FMath::Abs(LeaderTrainNow->mSimulationData.Velocity);
			const float Strongest = FMath::Max(LeaderTrainNow->mConsistData.MaxBrakeDeceleration * MaxRatedBrakeMultiple, GetPlanningDeceleration(*Leader, LeaderSpeed));
			SoftBrake = GetLeaderTravel(*Leader, FMath::Square(LeaderSpeed) / (2.f * Strongest));
			HardBrake = GetLeaderTravel(*Leader, GetStoppingDistance(*Leader, LeaderSpeed));
		}

		// A leader on this train's path: stop behind its nearest vehicle; one moving away along it also gets the
		// distance it travels before it stops.
		const float Gap = GetGapToVehicles(Train, State, *Leader);
		if (Gap >= 0.f)
		{
			const bool bMovesAway = State.LeadersMovingAway.Contains(LeaderTrain);
			Add(Gap, bMovesAway ? SoftBrake : 0.f, bMovesAway ? HardBrake : 0.f, true);
			continue;
		}
		// A leader following this train somewhere holds track there only behind it.
		if (!Leader->bManaged || Leader->FollowedTrains.Contains(State.Train))
		{
			continue;
		}
		// A leader merging in or crossing ahead: stop before the first track along this train's route it holds (or
		// crosses).
		for (const FATRouteStep& Step : State.Route)
		{
			const AFGBuildableRailroadTrack* Held = nullptr;
			for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Step.Track))
			{
				if (!Held && HoldsTrack(*Leader, Conflict))
				{
					Held = Conflict;
				}
			}
			if (!Held)
			{
				continue;
			}
			const float Start = GetDistanceToTrackStart(Train, State, Step.Track);
			if (Start <= TrackNotOnPath)
			{
				break;
			}
			// A leader merging the same way moves on ahead; one crossing leaves the conflict only once it has passed:
			// while it clears the crossing before this train, at its speed now, gets there, this train only keeps to a
			// speed that gets it there after the leader.
			const bool bSameWay = Leader->Route.ContainsByPredicate([&Step](const FATRouteStep& LeaderStep)
			{
				return LeaderStep.Track == Step.Track && LeaderStep.Exit == Step.Exit;
			});
			const float Pace = bSameWay ? -1.f : GetCrossingPaceSpeed(Train, *Leader, Held, Start);
			if (Pace >= 0.f)
			{
				OutPaceSpeed = FMath::Min(OutPaceSpeed, Pace);
				break;
			}
			Add(FMath::Max(Start, 0.f), bSameWay ? SoftBrake : 0.f, bSameWay ? HardBrake : 0.f, false);
			break;
		}
	}
	return Limit;
}

float AATTrafficSubsystem::GetCrossingPaceSpeed(const AFGTrain* Train, const FATTrafficTrain& Leader, const AFGBuildableRailroadTrack* Track, float Start) const
{
	const AFGTrain* LeaderTrain = Leader.Train.Get();
	const float LeaderSpeed = LeaderTrain ? FMath::Abs(LeaderTrain->mSimulationData.Velocity) : 0.f;
	const float Speed = FMath::Abs(Train->mSimulationData.Velocity);
	if (!LeaderTrain || LeaderSpeed < FollowMinSpeed || Start <= 0.f || !IsClearedPast(Leader, Track, INDEX_NONE))
	{
		return -1.f;
	}
	const float Seconds = GetClearDistance(Leader, Track, INDEX_NONE) / LeaderSpeed + FollowTimeMargin();
	if (Speed * Seconds >= Start)
	{
		return -1.f;
	}
	return Start / (Seconds + CrossingPaceSlackSeconds);
}

float AATTrafficSubsystem::GetLeaderTravel(const FATTrafficTrain& Leader, float Braking) const
{
	float Travel = Braking;
	if (Leader.bAuthorityToGoal)
	{
		if (Leader.Route.Num() > 0)
		{
			Travel = FMath::Min(Travel, FMath::Max(0.f, Leader.Route.Last().EndDistance - Leader.FrontAhead));
		}
	}
	else
	{
		const float Authority = GetAuthorityDistance(Leader.Train.Get());
		if (Authority >= 0.f)
		{
			Travel = FMath::Min(Travel, FMath::Max(0.f, Authority - Leader.StopMargin));
		}
	}
	return Travel;
}

float AATTrafficSubsystem::GetGapToVehicles(const AFGTrain* Train, const FATTrafficTrain& State, const FATTrafficTrain& Other) const
{
	const FTrainAtcData& Atc = Train->mAtcData;
	if (Other.Vehicles.Num() == 0 || !Atc.Path.IsValid() || !Atc.Path->PathPoints.IsValidIndex(Atc.CurrentPathSegment))
	{
		return -1.f;
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
		const float TrackStart = Distance - PathTrack->GetLength() - State.FrontAhead;
		if (TrackStart > GapSearchDistance)
		{
			break;
		}
		float Nearest = -1.f;
		for (const FATVehicleSpot& Vehicle : Other.Vehicles)
		{
			if (Vehicle.Track != PathTrack)
			{
				continue;
			}
			// From the track's start, in this train's direction, to the vehicle's centre; behind the front doesn't count.
			const float Centre = TrackStart + PathTrack->GetLength() - FMath::Abs(Exit->GetTrackPosition().Offset - Vehicle.Offset);
			if (Centre < 0.f)
			{
				continue;
			}
			const float Gap = FMath::Max(Centre - Vehicle.HalfLength, 0.f);
			Nearest = Nearest < 0.f ? Gap : FMath::Min(Nearest, Gap);
		}
		if (Nearest >= 0.f)
		{
			return Nearest;
		}
	}
	return -1.f;
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
		const float StandPenalty = Other.BlockedBy == Self.Train ? WaitingForSelfPenalty : (Other.Speed < 100.f ? StandingTrainPenalty : MovingTrainPenalty);
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
			for (const FATRouteStep& Step : Other.Route)
			{
				if (!HoldsTrack(Other, Step.Track))
				{
					break;
				}
				Out.FindOrAdd(Step.Track).HeldExits.Add(Step.Exit);
			}
		}
	}
}

float AATTrafficSubsystem::GetCongestionCost(const TMap<const AFGBuildableRailroadTrack*, FCongestion>& Congestion, const AFGBuildableRailroadTrack* Track,
	const UFGRailroadTrackConnectionComponent* Exit, bool bNewRoute)
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
	for (const UFGRailroadTrackConnectionComponent* HeldExit : Entry->HeldExits)
	{
		if (bNewRoute && HeldExit != Exit)
		{
			Cost += HeadOnPenalty;
		}
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
		CurrentCost += State.Route[Index].Track->GetLength() + GetCongestionCost(Congestion, State.Route[Index].Track, State.Route[Index].Exit, false);
	}

	const TArray<UFGRailroadTrackConnectionComponent*> Alternative = FATRouter::FindRoute(Start, Goal, false,
		[&Congestion](const AFGBuildableRailroadTrack* Track, const UFGRailroadTrackConnectionComponent* Exit) { return GetCongestionCost(Congestion, Track, Exit, true); });
	if (Alternative.Num() < 2)
	{
		return;
	}
	bool bSame = Alternative.Num() == State.Route.Num() - Commit;
	float NewCost = 0.f;
	for (int32 Index = 1; Index < Alternative.Num(); ++Index)
	{
		const AFGBuildableRailroadTrack* Track = Alternative[Index]->GetTrack();
		NewCost += (Track ? Track->GetLength() : 0.f) + GetCongestionCost(Congestion, Track, Alternative[Index], true);
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

void AATTrafficSubsystem::RefreshBrakePlan(FATTrafficTrain& State) const
{
	const AFGTrain* Train = State.Train.Get();
	const float Mass = GetMass(Train);
	const float Rated = Train ? Train->mConsistData.MaxBrakeDeceleration : 100.f;
	int32 Highest = INDEX_NONE;
	float Above = -1.f;
	for (int32 Band = ATBrakeBands - 1; Band >= 0; --Band)
	{
		if (State.BrakeForce[Band] > 0.f)
		{
			Above = State.BrakeForce[Band] / Mass * MeasuredBrakeShare();
			Highest = Highest == INDEX_NONE ? Band : Highest;
		}
		State.PlanDecel[Band] = Above;
	}
	for (int32 Band = 0; Band < ATBrakeBands; ++Band)
	{
		float& Plan = State.PlanDecel[Band];
		const float Middle = (Band + 0.5f) * ATBrakeBandWidth;
		if (Highest == INDEX_NONE)
		{
			Plan = Rated * RatedBrakeShare() * FMath::Min(1.f, FallbackFullSpeed / Middle);
		}
		else if (Band > Highest)
		{
			Plan = FMath::Max(State.PlanDecel[Highest] * (Highest + 0.5f) / (Band + 0.5f), Rated * MinRatedBrakeShare);
		}
		Plan = FMath::Clamp(Plan, MinPlanDeceleration, 800.f);
	}
}

float AATTrafficSubsystem::GetPlanningDeceleration(const FATTrafficTrain& State, float Speed)
{
	return FMath::Max(State.PlanDecel[GetBrakeBand(Speed >= 0.f ? Speed : State.Speed)], MinPlanDeceleration);
}

float AATTrafficSubsystem::GetStoppingDistance(const FATTrafficTrain& State, float Speed)
{
	float Distance = Speed * BrakeLagSeconds;
	for (int32 Band = 0; Band < ATBrakeBands; ++Band)
	{
		const float Low = Band * ATBrakeBandWidth;
		if (Speed <= Low)
		{
			break;
		}
		const float High = Band == ATBrakeBands - 1 ? Speed : FMath::Min(Low + ATBrakeBandWidth, Speed);
		Distance += (FMath::Square(High) - FMath::Square(Low)) / (2.f * FMath::Max(State.PlanDecel[Band], MinPlanDeceleration));
	}
	return Distance;
}

float AATTrafficSubsystem::GetAllowedSpeed(const FATTrafficTrain& State, float Distance)
{
	if (Distance <= 0.f)
	{
		return 0.f;
	}
	// Braking distance from the bottom of each band down to standstill, without the build-up.
	float Below = 0.f;
	for (int32 Band = 0; Band < ATBrakeBands; ++Band)
	{
		const float Low = Band * ATBrakeBandWidth;
		const float High = Low + ATBrakeBandWidth;
		const float Deceleration = FMath::Max(State.PlanDecel[Band], MinPlanDeceleration);
		// The speed in this band whose stop takes Distance: Below + (v^2 - Low^2) / (2 a) + v * Lag = Distance.
		const float Constant = Below - FMath::Square(Low) / (2.f * Deceleration) - Distance;
		const float Speed = Deceleration * (FMath::Sqrt(FMath::Square(BrakeLagSeconds) - 2.f * Constant / Deceleration) - BrakeLagSeconds);
		if (Speed <= High || Band == ATBrakeBands - 1)
		{
			return FMath::Max(Speed, 0.f);
		}
		Below += (FMath::Square(High) - FMath::Square(Low)) / (2.f * Deceleration);
	}
	return 0.f;
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
	const float ClearTime = TimeToCover(Distance, State.Speed, GetAcceleration(State, 1.f), GetTopSpeed(Train));
	return ClearTime - State.WaitingSeconds * WaitingWeight();
}

float AATTrafficSubsystem::GetAcceleration(const FATTrafficTrain& State, float Share)
{
	const AFGTrain* Train = State.Train.Get();
	const float Mass = Train ? FMath::Max(Train->mConsistData.Mass, 1.f) : 1.f;
	const float Tractive = Train ? Train->mConsistData.MaxTractiveEffort : 0.f;
	return FMath::Clamp(Tractive / Mass * Share, 20.f, 1200.f);
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
	// On the path the route was read from, the authority is at a known segment; a route can pass its track twice.
	const int32 AuthoritySegment = Atc.Path.Get() == State->AuthorityPath ? State->AuthoritySegment : INDEX_NONE;
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
		if (Track == Authority && (AuthoritySegment == INDEX_NONE || Index >= AuthoritySegment))
		{
			return FMath::Max(0.f, Distance - State->FrontAhead);
		}
	}
	return 0.f;
}

void AATTrafficSubsystem::ApplyAuthority(AFGTrain* Train, float DeltaSeconds)
{
	FATTrafficTrain* State = mEnabled ? mTrains.Find(Train) : nullptr;
	if (!State || !State->bManaged)
	{
		return;
	}
	AATControlSubsystem::IgnoreSignalStops(Train);
	const float Distance = GetAuthorityDistance(Train);
	// Behind a leader on shared track: keep able to stop where the leader could stop.
	float HardFollowLimit = -1.f;
	float PaceSpeed = MAX_flt;
	FATFollowInfo FollowInfo;
	const float FollowLimit = GetFollowLimit(Train, *State, HardFollowLimit, PaceSpeed, &FollowInfo);
	if (Distance < 0.f && FollowLimit < 0.f && PaceSpeed == MAX_flt)
	{
		State->bFullBrake = false;
		State->bForcedBrake = false;
		State->bCatchArmed = false;
		return;
	}

	FTrainAtcData& Atc = Train->mAtcData;
	const float Speed = FMath::Abs(Train->mSimulationData.Velocity);
	const float Usable = Distance - State->StopMargin;
	// The follow limit with the leaders' strongest braking caps the speed; with their planning braking it decides
	// full brake.
	auto GetAllowed = [State, Distance, Usable](float Follow)
	{
		float Result = MAX_flt;
		if (Distance >= 0.f)
		{
			Result = GetAllowedSpeed(*State, Usable);
		}
		if (Follow >= 0.f)
		{
			Result = FMath::Min(Result, GetAllowedSpeed(*State, Follow - AuthorityStopMargin));
		}
		return Result;
	};
	const float Deceleration = GetPlanningDeceleration(*State, Speed);
	const float Allowed = GetAllowed(FollowLimit);
	const float AllowedHard = GetAllowed(HardFollowLimit);
	float Target = FMath::Min(Allowed, PaceSpeed);
	// Waiting for a train that will clear: drive at the speed that reaches the braking point as it clears, so the
	// train rolls on instead of stopping.
	if (Distance >= 0.f && Usable > 0.f && State->ClearSeconds > 0.f)
	{
		const float Seconds = State->ClearSeconds;
		Target = FMath::Min(Target, Deceleration * (FMath::Sqrt(FMath::Square(Seconds) + 2.f * Usable / Deceleration) - Seconds));
	}
	// Forced braking: when its own brakes won't stop the train in time, take off the rest of the speed directly, at
	// the deceleration that stops it at the stop point, the way the game's block signals hold a train. Once started
	// it holds, with full brake, until the train's own brakes suffice.
	const float StopWithin = FMath::Min(Distance >= 0.f ? Usable : MAX_flt, HardFollowLimit >= 0.f ? HardFollowLimit - AuthorityStopMargin : MAX_flt);
	const float Needed = StopWithin < MAX_flt ? FMath::Square(Speed) / (2.f * FMath::Max(StopWithin, ForcedBrakeMinDistance)) : 0.f;
	const bool bOverCurve = Speed > AllowedHard + OverspeedTolerance;
	const bool bWasFullBrake = State->bFullBrake;
	const bool bWasForced = State->bForcedBrake;
	State->bForcedBrake = DeltaSeconds > 0.f && Speed > 0.f && Needed > Deceleration && (bOverCurve || bWasForced);
	State->bFullBrake = bOverCurve || State->bForcedBrake;
	Atc.TargetSpeed = State->bFullBrake ? 0.f : FMath::Min(Atc.TargetSpeed, Target);
	if (State->bForcedBrake)
	{
		const float Velocity = Train->mSimulationData.Velocity;
		const float NewSpeed = FMath::Max(0.f, Speed - (Needed - Deceleration) * DeltaSeconds);
		Train->mSimulationData.Velocity = Velocity < 0.f ? -NewSpeed : NewSpeed;
		State->bForcedSinceUpdate = true;
		if (!bWasForced)
		{
			UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s forced braking at %.1f m/s2 (%.0f km/h, %.0f m to stop, its brakes %.1f m/s2)"), *TrainName(Train),
				Needed / 100.f, Speed * 0.036f, StopWithin / 100.f, Deceleration / 100.f);
		}
	}
	if (State->bFullBrake && !bWasFullBrake && Speed > AllowedHard + 500.f)
	{
		FString Behind;
		if (HardFollowLimit >= 0.f && (Distance < 0.f || HardFollowLimit - AuthorityStopMargin < Usable))
		{
			const double* Since = State->FollowLeaderSince.Find(FollowInfo.Leader);
			Behind = FString::Printf(TEXT(", behind %s: %s %.0f m, its braking %.0f m, leader for %.1f s"), *TrainName(FollowInfo.Leader.Get()),
				FollowInfo.bVehicleOnPath ? TEXT("nearest vehicle") : TEXT("first track it holds"), FollowInfo.Gap / 100.f, FollowInfo.LeaderBrake / 100.f,
				Since ? GetWorld()->GetTimeSeconds() - *Since : -1.0);
		}
		UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s over its braking curve (%.0f km/h, allowed %.0f, %.0f m to go%s): full brake"), *TrainName(Train),
			Speed * 0.036f, AllowedHard * 0.036f, (Behind.IsEmpty() ? Distance : HardFollowLimit) / 100.f, *Behind);
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

void AATTrafficSubsystem::ApplyLocomotivePower(AFGTrain* Train, float Scale, float TopSpeed)
{
	bool bChanged = false;
	for (AFGRailroadSubsystem::TTrainIterator It(Train->GetFirstVehicle()); It; ++It)
	{
		AFGRailroadVehicle* Vehicle = *It;
		UFGRailroadVehicleMovementComponent* Movement = Vehicle ? Vehicle->GetRailroadVehicleMovementComponent() : nullptr;
		if (!Movement)
		{
			continue;
		}
		AFGLocomotive* Locomotive = Cast<AFGLocomotive>(Vehicle);
		const bool bLocomotive = Locomotive && Locomotive->GetLocomotiveMovementComponent();
		FATVehicleBase* Base = mVehicleBase.Find(Vehicle);
		// The game has set its own values again: start over from those.
		const float Applied = Base ? Base->MaxTractiveEffort * Base->Scale : 0.f;
		if (Base && (!FMath::IsNearlyEqual(Movement->mMaxTractiveEffort, Applied, FMath::Abs(Applied) * 1e-4f + 1.f)
			|| !FMath::IsNearlyEqual(Movement->mMaxVelocity, Base->MaxVelocitySet, 0.01f)))
		{
			mVehicleBase.Remove(Vehicle);
			Base = nullptr;
		}
		if (!Base)
		{
			if (Scale == 1.f && TopSpeed <= 0.f)
			{
				continue;
			}
			Base = &mVehicleBase.Add(Vehicle);
			Base->MaxTractiveEffort = Movement->mMaxTractiveEffort;
			Base->MaxVelocity = Movement->mMaxVelocity;
			Base->MaxVelocitySet = Movement->mMaxVelocity;
			if (bLocomotive)
			{
				const UFGLocomotiveMovementComponent* LocomotiveMovement = Locomotive->GetLocomotiveMovementComponent();
				Base->Traction = *LocomotiveMovement->mTractiveEffortCurve.GetRichCurveConst();
				Base->TractionAsset = LocomotiveMovement->mTractiveEffortCurve.ExternalCurve;
				Base->PowerConsumption = Locomotive->mPowerConsumption;
				Base->TractionTop = GetTractionTop(Base->Traction, Base->MaxVelocity);
			}
		}
		const float VehicleScale = bLocomotive ? Scale : 1.f;
		const float SpeedScale = bLocomotive && Base->TractionTop > 0.f && TopSpeed > Base->TractionTop ? TopSpeed * TopSpeedTractionHeadroom / Base->TractionTop : 1.f;
		const float MaxVelocity = FMath::Max(Base->MaxVelocity, TopSpeed);
		if (Base->Scale == VehicleScale && Base->SpeedScale == SpeedScale && Base->MaxVelocitySet == MaxVelocity)
		{
			continue;
		}
		SetVehicleScale(*Vehicle, *Base, VehicleScale, SpeedScale, MaxVelocity);
		bChanged = true;
		if (VehicleScale == 1.f && SpeedScale == 1.f && MaxVelocity == Base->MaxVelocity)
		{
			mVehicleBase.Remove(Vehicle);
		}
	}
	if (bChanged)
	{
		Train->RecalculateConsistData();
		UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s locomotives at x%.2f traction and power draw, top speed %.0f km/h (train tractive effort %.0f kN)"), *TrainName(Train), Scale,
			Train->mConsistData.MaxSpeed * 0.036f, Train->mConsistData.MaxTractiveEffort / 100000.f);
	}
}

void AATTrafficSubsystem::SetVehicleScale(AFGRailroadVehicle& Vehicle, FATVehicleBase& Base, float Scale, float SpeedScale, float MaxVelocity)
{
	UFGRailroadVehicleMovementComponent* Movement = Vehicle.GetRailroadVehicleMovementComponent();
	if (!Movement)
	{
		return;
	}
	AFGLocomotive* Locomotive = Cast<AFGLocomotive>(&Vehicle);
	if (UFGLocomotiveMovementComponent* LocomotiveMovement = Locomotive ? Locomotive->GetLocomotiveMovementComponent() : nullptr)
	{
		// Traction scaled by Scale at each speed and stretched over SpeedScale times the speeds; the power draw follows
		// the traction power, force times speed.
		FRuntimeFloatCurve& Curve = LocomotiveMovement->mTractiveEffortCurve;
		FRichCurve Traction = Base.Traction;
		for (FRichCurveKey& Key : Traction.Keys)
		{
			Key.Time *= SpeedScale;
			Key.Value *= Scale;
			Key.ArriveTangent *= Scale / SpeedScale;
			Key.LeaveTangent *= Scale / SpeedScale;
		}
		Curve.EditorCurveData = Traction;
		// The changed curve is this locomotive's own; the shared asset goes back once both scales are 1 again.
		Curve.ExternalCurve = Scale == 1.f && SpeedScale == 1.f ? Base.TractionAsset.Get() : nullptr;
		LocomotiveMovement->mMaxTractiveEffort = Base.MaxTractiveEffort * Scale;
		Locomotive->mPowerConsumption = FFloatInterval(Base.PowerConsumption.Min * Scale * SpeedScale, Base.PowerConsumption.Max * Scale * SpeedScale);
	}
	Movement->mMaxVelocity = MaxVelocity;
	Base.Scale = Scale;
	Base.SpeedScale = SpeedScale;
	Base.MaxVelocitySet = MaxVelocity;
}

void AATTrafficSubsystem::RestoreLocomotivePower()
{
	TSet<AFGTrain*> Trains;
	for (TPair<TWeakObjectPtr<AFGRailroadVehicle>, FATVehicleBase>& Pair : mVehicleBase)
	{
		if (AFGRailroadVehicle* Vehicle = Pair.Key.Get())
		{
			SetVehicleScale(*Vehicle, Pair.Value, 1.f, 1.f, Pair.Value.MaxVelocity);
			Trains.Add(Vehicle->GetTrain());
		}
	}
	mVehicleBase.Reset();
	for (AFGTrain* Train : Trains)
	{
		if (IsValid(Train))
		{
			Train->RecalculateConsistData();
		}
	}
}

void AATTrafficSubsystem::ApplySpeedSettings(AFGTrain* Train, float DeltaSeconds)
{
	// Above the locomotives' own top speed their traction is stretched to the setting; the game's driver then aims
	// for the consist's top speed, at least the setting, so the setting caps it.
	if (mEnabled && GTopSpeed > 0.f && Train->IsSelfDrivingEnabled() && mTrains.Contains(Train))
	{
		Train->mAtcData.TargetSpeed = FMath::Min(Train->mAtcData.TargetSpeed, GTopSpeed);
	}
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
