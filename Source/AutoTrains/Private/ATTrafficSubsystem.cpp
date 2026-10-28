#include "ATTrafficSubsystem.h"
#include "ATConfig.h"
#include "ATControlSubsystem.h"
#include "ATRouter.h"
#include "ATSipp.h"
#include "Algo/AnyOf.h"
#include "AutoTrains.h"
#include "Buildables/FGBuildableRailroadStation.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "EngineUtils.h"
#include "Curves/CurveFloat.h"
#include "FGLocomotive.h"
#include "FGLocomotiveMovementComponent.h"
#include "FGRailroadSubsystem.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "FGRailroadVehicle.h"
#include "FGTrain.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Subsystem/SubsystemActorManager.h"

namespace
{
	// From the Mods menu: tighter margins, and trains passing through each other.
	bool GAggressive = true;
	bool GPhaseThrough = false;
	// Some train is passing through a deadlock.
	bool GGhosting = false;
	// Seconds of slack for following and going first. [s]
	float GSafetyMargin = 0.f;
	// Top speed of every self-driving train, 0 for each train's own [cm/s], and the acceleration multiplier.
	float GTopSpeed = 0.f;
	float GAccelerationScale = 1.f;
	int32 GPhaseThroughCount = 0;
	// Tracks the two vehicles of the last contact let through were on, and the trains on them at the next update.
	TWeakObjectPtr<AFGBuildableRailroadTrack> GPhaseTrack;
	TWeakObjectPtr<AFGBuildableRailroadTrack> GPhaseOtherTrack;
	bool GPhaseNamesPending = false;
	FString GPhaseNames;

	constexpr float SettingsInterval = 1.f;
	// Trains stop at least this far before a switch. [cm]
	constexpr float SwitchStopMargin = 2500.f;
	// Distance along each track from a switch point within which a vehicle fouls the switch, at least; branches leaving
	// a switch the same way foul it until their centre lines are JointClearance apart, measured up to JointFoulMax. [cm]
	constexpr float JointFoulDistance = 2000.f;
	constexpr float JointClearance = 550.f;
	constexpr float JointFoulMax = 15000.f;
	constexpr float JointFoulStep = 200.f;
	// A vehicle counts as on the next track only once it reaches this far past the end of its own; station tracks
	// are exactly one vehicle long. [cm]
	constexpr float OverhangTolerance = 50.f;
	// Planning deceleration: shares of the measured full-brake deceleration and of the air brakes' force over the
	// train's mass with its cargo; the dynamic brakes come on top, strong at low speed and weaker above.
	float MeasuredBrakeShare() { return GAggressive ? 0.95f : 0.85f; }
	float RatedBrakeShare() { return GAggressive ? 0.8f : 0.6f; }
	// Seconds the brakes take to build up, counted as travel at the starting speed; about 1 s in the measurements.
	constexpr float BrakeLagSeconds = 0.75f;
	constexpr float MinPlanDeceleration = 50.f;
	// Forced braking plans the stop with at least this distance left, so the last tick doesn't divide by nothing. [cm]
	constexpr float ForcedBrakeMinDistance = 100.f;
	// A brake sample counts for at most this multiple of the consist's rating; stronger ones came from catches or
	// contact with another train.
	constexpr float MaxRatedBrakeMultiple = 1.25f;
	// A train over its braking curve by more than this gets full brake; its target speed is the curve's this long
	// ahead. [cm/s] [s]
	constexpr float OverspeedTolerance = 100.f;
	constexpr float TargetLeadSeconds = 0.5f;
	constexpr double ConflictRebuildSeconds = 30.0;
	constexpr double ConflictRebuildMinSeconds = 2.0;

	// Pacing behind a leader crossing ahead: only a leader at least FollowMinSpeed fast, cleared the safety margin before
	// the follower gets there. [s] [cm/s]
	float FollowTimeMargin() { return GSafetyMargin; }
	constexpr float FollowMinSpeed = 500.f;
	// A train behind a leader crossing its route drives so it gets there this much later than the time the leader
	// clears it plus the safety margin. [s]
	constexpr float CrossingPaceSlackSeconds = 1.f;
	// How far along its path a follower looks for the vehicles of the trains it follows. [cm]
	constexpr float GapSearchDistance = 300000.f;
	// Returned by GetDistanceToTrackStart for a track that isn't on the train's path.
	constexpr float TrackNotOnPath = -1e9f;

	// A train passing through a deadlock does so until it has driven its length and this much further, or for at most
	// GhostMaxSeconds. [cm] [s]
	constexpr float GhostClearDistance = 30000.f;
	constexpr double GhostMaxSeconds = 120.0;
	// A train that waited this long counts for deadlock detection; a deadlocked one slower than this shows the game's
	// long-wait warning. [s] [cm/s]
	constexpr float DeadlockWaitSeconds = 20.f;
	constexpr float DeadlockShowSpeed = 50.f;
	// A deadlock not seen for this long is over. [s]
	constexpr double DeadlockForgetSeconds = 10.0;
	// A blocked train slower than this counts as waiting. [cm/s]
	constexpr float PlannerWaitSpeed = 300.f;

	// Learning how a train drives: per update, its cruising speed moves this share towards a higher speed and this share
	// towards a lower one; its acceleration this share towards each sample. Plans use at least these. [cm/s] [cm/s^2]
	constexpr float CruiseRiseRate = 0.1f;
	constexpr float CruiseFallRate = 0.005f;
	constexpr float AccelSampleRate = 0.05f;
	constexpr float MinPlanCruise = 833.f;
	constexpr float MinPlanAcceleration = 10.f;

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
	// A planning cycle's plans as soon as they are in.
	if (mCycle.IsValid() && mCycle->bDone.load(std::memory_order_acquire))
	{
		ApplyPlanCycle();
	}
	mUpdateTimer -= DeltaSeconds;
	mSinceUpdate += DeltaSeconds;
	if (mUpdateTimer <= 0.f)
	{
		mUpdateTimer = mSettings.UpdateIntervalSeconds;
		UpdateTraffic(mSinceUpdate);
		mSinceUpdate = 0.f;
		if (GPhaseNamesPending)
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
			GPhaseNames = FString::Printf(TEXT("%s on %s and %s on %s"), *NamesOn(GPhaseTrack.Get()), *TrackName(GPhaseTrack.Get()), *NamesOn(GPhaseOtherTrack.Get()),
				*TrackName(GPhaseOtherTrack.Get()));
			GPhaseNamesPending = false;
		}
	}
	mPhaseReportTimer -= DeltaSeconds;
	if (mPhaseReportTimer <= 0.f)
	{
		mPhaseReportTimer = 10.f;
		if (GPhaseThroughCount > 0)
		{
			UE_LOG(LogAutoTrains, Log, TEXT("traffic: trains passed through each other %d times in the last 10 s; last %s"), GPhaseThroughCount, *GPhaseNames);
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
	mSettings = Settings;
}

bool AATTrafficSubsystem::ShouldPhaseThrough()
{
	return GPhaseThrough || GGhosting;
}

void AATTrafficSubsystem::CountPhaseThrough(AFGBuildableRailroadTrack* Track, AFGBuildableRailroadTrack* OtherTrack)
{
	++GPhaseThroughCount;
	GPhaseTrack = Track;
	GPhaseOtherTrack = OtherTrack;
	GPhaseNamesPending = true;
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
	mBookings.Reset();
	mOwnerBookings.Reset();
	mCycle.Reset();
	mPlanSummaryAt = GetWorld() ? GetWorld()->GetTimeSeconds() + 60.0 : 0.0;
	mUpdateTimer = 0.f;
	mSinceUpdate = 0.f;
	mDeadlocks.Reset();
	if (bEnabled)
	{
		RebuildConflicts();
	}
	const FString Message = bEnabled ? TEXT("Traffic control on: self-driving trains ignore signals and book track and switches in time.") : TEXT("Traffic control off.");
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
	return Message;
}

float AATTrafficSubsystem::GetSafetyMarginSeconds()
{
	return GSafetyMargin;
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

	const double WorldNow = GetWorld()->GetTimeSeconds();
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
		TrackDocking(State, Train, WorldNow);
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
				const float Force = Deceleration * State.Mass;
				float& Stored = State.BrakeForce[Band];
				const bool bFirst = Stored <= 0.f;
				// Weaker samples count more, so the plan errs on the safe side.
				Stored = bFirst ? Force : Force < Stored ? Stored * 0.5f + Force * 0.5f : Stored * 0.85f + Force * 0.15f;
				if (bFirst)
				{
					UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s brakes at %.2f m/s2 at %.0f-%.0f km/h (rated %.2f, %.0f t); planning with %.2f"), *TrainName(Train),
						Deceleration / 100.f, Band * ATBrakeBandWidth * 0.036f, (Band + 1) * ATBrakeBandWidth * 0.036f, Train->mConsistData.MaxBrakeDeceleration / 100.f,
						State.Mass / 1000.f, GetPlanningDeceleration(State) / 100.f);
				}
			}
		}
		// Motion as driven: the speed it runs at when nothing holds it back (rising quickly, falling slowly), and its
		// acceleration while the driver is speeding up.
		const float Target = Train->mAtcData.TargetSpeed;
		const bool bFree = State.bManaged && !State.bFullBrake && !State.bWasFullBrake && !State.bForcedSinceUpdate && Target >= State.Speed - 50.f;
		if (bFree && State.Speed > 500.f)
		{
			const float Rate = State.Speed > State.CruiseSeen ? CruiseRiseRate : CruiseFallRate;
			State.CruiseSeen = State.CruiseSeen <= 0.f ? State.Speed : State.CruiseSeen + (State.Speed - State.CruiseSeen) * Rate;
		}
		if (bFree && DeltaSeconds > 0.f && Target > State.Speed + 100.f && State.Speed > State.LastSpeed)
		{
			const float Sample = (State.Speed - State.LastSpeed) / DeltaSeconds;
			if (Sample > 5.f && Sample < 500.f)
			{
				State.AccelSeen = State.AccelSeen <= 0.f ? Sample : State.AccelSeen + (Sample - State.AccelSeen) * AccelSampleRate;
			}
		}
		State.bWasFullBrake = State.bFullBrake;
		State.bForcedSinceUpdate = State.bForcedBrake;
		State.LastSpeed = State.Speed;
		RefreshBrakePlan(State);
		State.StoppedSeconds = State.Speed < 100.f ? State.StoppedSeconds + DeltaSeconds : 0.f;
		if (State.GhostUntil >= 0.0)
		{
			State.GhostLeft -= State.Speed * DeltaSeconds;
			if (State.GhostLeft <= 0.f || WorldNow > State.GhostUntil || !State.bManaged)
			{
				State.GhostUntil = -1.0;
				UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s has passed through the deadlock"), *TrainName(Train));
			}
		}
		State.RerouteCooldown = FMath::Max(0.f, State.RerouteCooldown - DeltaSeconds);
	}
	for (auto It = mTrains.CreateIterator(); It; ++It)
	{
		if (!Seen.Contains(It.Key()))
		{
			It.RemoveCurrent();
		}
	}
	GGhosting = false;
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		GGhosting = GGhosting || Pair.Value.GhostUntil >= 0.0;
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

	// Reasons carry times: logged again only when the train or place waited at changes.
	auto LogKey = [](const FATTrafficTrain* State) { return State->BlockedKey; };
	TMap<const FATTrafficTrain*, FString> PreviousReasons;
	for (const FATTrafficTrain* State : Managed)
	{
		PreviousReasons.Add(State, LogKey(State));
	}
	UpdatePlannedTraffic(DeltaSeconds);
	for (FATTrafficTrain* State : Managed)
	{
		if (LogKey(State) != PreviousReasons.FindRef(State) && !State->BlockedReason.IsEmpty())
		{
			UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s: %s"), *TrainName(State->Train.Get()), *State->BlockedReason);
		}

		// Waiting for the train it names, even while it rolls slowly up to its stop.
		State->WaitingSeconds = State->BlockedBy.IsValid() && State->Speed < PlannerWaitSpeed ? State->WaitingSeconds + DeltaSeconds : 0.f;
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
	State.Mass = 0.f;

	const FTrainAtcData& Atc = Train->mAtcData;
	State.bManaged = Train->IsSelfDrivingEnabled() && !Train->IsDocked() && Atc.Path.IsValid()
		&& Atc.Path->PathPoints.IsValidIndex(Atc.CurrentPathSegment);
	if (State.bManaged)
	{
		State.RoutePath = Atc.Path.Get();
		State.RouteFirstSegment = Atc.CurrentPathSegment;
		State.GoalStation = Atc.Path->Station;
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
		if (const UFGRailroadVehicleMovementComponent* Movement = Vehicle->GetRailroadVehicleMovementComponent())
		{
			State.Mass += Movement->GetMass();
		}
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
			if (Joint != INDEX_NONE && ToEnd < HalfLength + GetFoulDistance(Connection))
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

	// The consist's own mass leaves out cargo loaded since it was last worked out.
	State.Mass = FMath::Max3(State.Mass, Train->mConsistData.Mass, 1.f);

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

void AATTrafficSubsystem::UpdateAuthority(FATTrafficTrain& State) const
{
	State.AuthorityTrack = nullptr;
	State.AuthorityPath = nullptr;
	State.AuthoritySegment = INDEX_NONE;
	State.bAuthorityToGoal = false;
	State.StopMargin = GetStopDistance();
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

float AATTrafficSubsystem::GetLatestClearSeconds(const FATTrafficTrain& Holder, float ClearDistance, float Speed) const
{
	const float Reach = Holder.bAuthorityToGoal ? (Holder.Route.Num() > 0 ? Holder.Route.Last().EndDistance - Holder.FrontAhead : 0.f)
		: GetAuthorityDistance(Holder.Train.Get()) - Holder.StopMargin;
	if (Reach < ClearDistance || Speed <= 0.f)
	{
		return -1.f;
	}
	// Slowing evenly to stop at the end of its authority, which it may do whatever its speed now.
	const float Deceleration = FMath::Square(Speed) / (2.f * FMath::Max(Reach, 1.f));
	return (Speed - FMath::Sqrt(FMath::Max(0.f, FMath::Square(Speed) - 2.f * Deceleration * ClearDistance))) / Deceleration;
}

float AATTrafficSubsystem::GetClearDistance(const FATTrafficTrain& Holder, const AFGBuildableRailroadTrack* Track, int32 Joint) const
{
	// Past the end of the track or the switch's zone; a track behind the holder's route is one its tail is on.
	for (const FATRouteStep& Step : Holder.Route)
	{
		const bool bThere = Joint != INDEX_NONE ? GetJoint(Step.Exit) == Joint : Step.Track == Track;
		if (bThere)
		{
			return Step.EndDistance - Holder.FrontAhead + Holder.Length + (Joint != INDEX_NONE ? GetJointFoulDistance(Joint) : 0.f);
		}
	}
	const float FromVehicles = GetTailClearDistance(Holder, Track, Joint);
	return FromVehicles >= 0.f ? FromVehicles : Holder.Length + (Joint != INDEX_NONE ? GetJointFoulDistance(Joint) : JointFoulDistance);
}

float AATTrafficSubsystem::GetTailClearDistance(const FATTrafficTrain& Holder, const AFGBuildableRailroadTrack* Track, int32 Joint) const
{
	// The end of a track the train leaves it through: its route exit, or the end attached to a track a vehicle of the
	// train is on or its route starts with.
	auto GetForwardEnd = [&Holder](const AFGBuildableRailroadTrack* VehicleTrack) -> const UFGRailroadTrackConnectionComponent*
	{
		for (const FATRouteStep& Step : Holder.Route)
		{
			if (Step.Track == VehicleTrack)
			{
				return Step.Exit;
			}
		}
		for (int32 End = 0; End < 2; ++End)
		{
			const UFGRailroadTrackConnectionComponent* Connection = VehicleTrack->GetConnection(End);
			for (const UFGRailroadTrackConnectionComponent* Other : Connection ? Connection->GetConnections() : TArray<UFGRailroadTrackConnectionComponent*>())
			{
				const AFGBuildableRailroadTrack* OtherTrack = Other ? Other->GetTrack() : nullptr;
				if (OtherTrack && OtherTrack != VehicleTrack && ((Holder.Route.Num() > 0 && Holder.Route[0].Track == OtherTrack)
					|| Holder.Vehicles.ContainsByPredicate([OtherTrack](const FATVehicleSpot& Spot) { return Spot.Track == OtherTrack; })))
				{
					return Connection;
				}
			}
		}
		return nullptr;
	};
	float Distance = -1.f;
	for (const FATVehicleSpot& Vehicle : Holder.Vehicles)
	{
		const UFGRailroadTrackConnectionComponent* Forward = GetForwardEnd(Vehicle.Track);
		if (!Forward)
		{
			continue;
		}
		const float ToForward = FMath::Abs(Forward->GetTrackPosition().Offset - Vehicle.Offset);
		if (Joint != INDEX_NONE)
		{
			// Within the switch's fouling zone at one of its own track's ends: until its rear is out of it.
			for (int32 End = 0; End < 2; ++End)
			{
				const UFGRailroadTrackConnectionComponent* Connection = Vehicle.Track->GetConnection(End);
				if (!Connection || GetJoint(Connection) != Joint)
				{
					continue;
				}
				const float ToEnd = FMath::Abs(Connection->GetTrackPosition().Offset - Vehicle.Offset);
				const float Foul = GetFoulDistance(Connection);
				if (ToEnd >= Vehicle.HalfLength + Foul)
				{
					continue;
				}
				Distance = FMath::Max(Distance, Connection == Forward ? ToEnd + Vehicle.HalfLength + GetJointFoulDistance(Joint) : Vehicle.HalfLength + Foul - ToEnd);
			}
			continue;
		}
		if (Vehicle.Track == Track)
		{
			// Its rear has to pass the forward end.
			Distance = FMath::Max(Distance, ToForward + Vehicle.HalfLength + OverhangTolerance);
			continue;
		}
		// Overhanging onto Track attached at one of its own track's ends: until it no longer reaches past that end.
		for (int32 End = 0; End < 2; ++End)
		{
			const UFGRailroadTrackConnectionComponent* Connection = Vehicle.Track->GetConnection(End);
			const bool bAttached = Connection && Connection->GetConnections().ContainsByPredicate([Track](const UFGRailroadTrackConnectionComponent* Other)
			{
				return Other && Other->GetTrack() == Track;
			});
			const float ToEnd = Connection ? FMath::Abs(Connection->GetTrackPosition().Offset - Vehicle.Offset) : 0.f;
			if (!bAttached || ToEnd >= Vehicle.HalfLength - OverhangTolerance)
			{
				continue;
			}
			Distance = FMath::Max(Distance, Connection == Forward ? ToEnd + Vehicle.HalfLength : Vehicle.HalfLength - ToEnd);
		}
	}
	return Distance;
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
		// A leader merging in or crossing ahead: stop before the first switch along this train's route it holds, or the
		// first track it holds (or crosses).
		for (int32 Index = 0; Index < State.Route.Num(); ++Index)
		{
			const FATRouteStep& Step = State.Route[Index];
			const UFGRailroadTrackConnectionComponent* JointEntry = Index > 0 ? State.Route[Index - 1].Exit : nullptr;
			const int32 Joint = GetJoint(JointEntry);
			const bool bHeldJoint = Joint != INDEX_NONE && HoldsJoint(*Leader, Joint);
			const AFGBuildableRailroadTrack* Held = nullptr;
			for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Step.Track))
			{
				if (!Held && HoldsTrack(*Leader, Conflict))
				{
					Held = Conflict;
				}
			}
			if (!bHeldJoint && !Held)
			{
				continue;
			}
			float Start = GetDistanceToTrackStart(Train, State, Step.Track);
			if (Start <= TrackNotOnPath)
			{
				break;
			}
			if (bHeldJoint)
			{
				Start -= GetFoulDistance(JointEntry);
			}
			// A leader merging the same way moves on ahead; one crossing leaves the conflict only once it has passed:
			// while it clears the crossing before this train, at its speed now, gets there, this train only keeps to a
			// speed that gets it there after the leader.
			const bool bSameWay = Leader->Route.ContainsByPredicate([&Step](const FATRouteStep& LeaderStep)
			{
				return LeaderStep.Track == Step.Track && LeaderStep.Exit == Step.Exit;
			});
			const float Pace = bSameWay ? -1.f : GetCrossingPaceSpeed(Train, *Leader, bHeldJoint ? nullptr : Held, bHeldJoint ? Joint : INDEX_NONE, Start);
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

float AATTrafficSubsystem::GetCrossingPaceSpeed(const AFGTrain* Train, const FATTrafficTrain& Leader, const AFGBuildableRailroadTrack* Track, int32 Joint,
	float Start) const
{
	const AFGTrain* LeaderTrain = Leader.Train.Get();
	const float LeaderSpeed = LeaderTrain ? FMath::Abs(LeaderTrain->mSimulationData.Velocity) : 0.f;
	const float Speed = FMath::Abs(Train->mSimulationData.Velocity);
	if (!LeaderTrain || LeaderSpeed < FollowMinSpeed || Start <= 0.f)
	{
		return -1.f;
	}
	const float ClearSeconds = GetLatestClearSeconds(Leader, GetClearDistance(Leader, Track, Joint), LeaderSpeed);
	if (ClearSeconds < 0.f)
	{
		return -1.f;
	}
	const float Seconds = ClearSeconds + FollowTimeMargin();
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

void AATTrafficSubsystem::GhostThrough(const TArray<const FATTrafficTrain*>& Chain, double Seconds)
{
	const double Now = GetWorld()->GetTimeSeconds();
	if (Chain.ContainsByPredicate([](const FATTrafficTrain* Member) { return Member->GhostUntil >= 0.0; }))
	{
		return;
	}
	TArray<FString> Lines;
	for (const FATTrafficTrain* Member : Chain)
	{
		if (FATTrafficTrain* Stuck = mTrains.Find(Member->Train))
		{
			Stuck->GhostUntil = Now + GhostMaxSeconds;
			Stuck->GhostLeft = Stuck->Length + GhostClearDistance;
			Lines.Add(FString::Printf(TEXT("%s: %s"), *TrainName(Stuck->Train.Get()), *Stuck->BlockedReason));
		}
	}
	GGhosting = true;
	const FString Fault = FString::Printf(TEXT("%s deadlock of %d trains passed through after %.0f s:\n  %s\n"), *FDateTime::Now().ToString(TEXT("%Y-%m-%d %H:%M:%S")), Chain.Num(),
		Seconds, *FString::Join(Lines, TEXT("\n  ")));
	UE_LOG(LogAutoTrains, Warning, TEXT("traffic: DEADLOCK passed through: %s"), *Fault);
	const FString Path = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("AutoTrains") / TEXT("faults.log"));
	if (!FFileHelper::SaveStringToFile(Fault, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append))
	{
		UE_LOG(LogAutoTrains, Error, TEXT("Could not write the fault to %s"), *Path);
	}
}

void AATTrafficSubsystem::LogDeadlocks()
{
	const double Now = GetWorld()->GetTimeSeconds();
	// Follow each long-waiting train's chain of trains it waits for; a chain that comes back is a deadlock. Every train
	// in one is marked; a cycle is logged when it first shows up. One that drops out of sight for a moment and comes back
	// within DeadlockForgetSeconds is the same deadlock still going on.
	TSet<const FATTrafficTrain*> InCycle;
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const FATTrafficTrain& Start = Pair.Value;
		if (!Start.bManaged || Start.WaitingSeconds < DeadlockWaitSeconds || InCycle.Contains(&Start))
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
			InCycle.Add(Member);
		}
		TArray<FString> Sorted = Names;
		Sorted.Sort();
		const FString Signature = FString::Join(Sorted, TEXT("|"));
		FDeadlockSeen* Seen = mDeadlocks.Find(Signature);
		const bool bNew = !Seen;
		if (bNew)
		{
			Seen = &mDeadlocks.Add(Signature, {Now, Now});
		}
		Seen->Last = Now;
		if (mSettings.bGhostDeadlocks && Now - Seen->First >= mSettings.GhostAfterSeconds)
		{
			GhostThrough(Chain, Now - Seen->First);
		}
		if (!bNew)
		{
			continue;
		}
		UE_LOG(LogAutoTrains, Warning, TEXT("traffic: DEADLOCK: %s -> %s"), *FString::Join(Names, TEXT(" -> ")), *Names[0]);
		for (const FATTrafficTrain* Member : Chain)
		{
			UE_LOG(LogAutoTrains, Warning, TEXT("traffic: DEADLOCK:   %s: %s"), *TrainName(Member->Train.Get()), *Member->BlockedReason);
			// Every train in the cycle looks for another way at once; a train waiting for it counts as impassable.
			if (FATTrafficTrain* Stuck = mTrains.Find(Member->Train))
			{
				Stuck->RerouteCooldown = 0.f;
				Stuck->RerouteTimer = 0.f;
			}
		}
	}
	for (auto It = mDeadlocks.CreateIterator(); It; ++It)
	{
		if (Now - It.Value().Last > DeadlockForgetSeconds)
		{
			It.RemoveCurrent();
		}
	}
	for (TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		Pair.Value.bDeadlocked = InCycle.Contains(&Pair.Value);
	}
}

AATTrafficSubsystem::FRouteSpeed AATTrafficSubsystem::GetRouteSpeed(const FATTrafficTrain& State) const
{
	FRouteSpeed Speed;
	// As the train has been seen to drive, within its top speed; until then its top speed and half its rated acceleration.
	const float Top = FMath::Max(GetTopSpeed(State.Train.Get()), 500.f);
	Speed.Cruise = State.CruiseSeen > 0.f ? FMath::Clamp(State.CruiseSeen, FMath::Min(MinPlanCruise, Top), Top) : Top;
	Speed.Length = State.Length;
	const float Acceleration = State.AccelSeen > 0.f ? FMath::Max(State.AccelSeen, MinPlanAcceleration) : GetAcceleration(State, 0.5f);
	const float Deceleration = GetPlanningDeceleration(State, Speed.Cruise) * GetNormalBrakingShare();
	Speed.StopLoss = Speed.Cruise / (2.f * Acceleration) + Speed.Cruise / (2.f * FMath::Max(Deceleration, MinPlanDeceleration));
	Speed.Accel = Acceleration;
	Speed.Brake = FMath::Max(Deceleration, MinPlanDeceleration);
	return Speed;
}

void AATTrafficSubsystem::RebuildConflicts()
{
	const double Begin = FPlatformTime::Seconds();
	mConflicts.Reset();
	mJointOf.Reset();
	mFoulOf.Reset();
	mJointFoul.Reset();

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

	// Fouling zones: two branches leaving a switch the same way foul it until their centre lines are far enough apart.
	auto PointAt = [](const UFGRailroadTrackConnectionComponent* Connection, float Distance, FVector& OutLocation)
	{
		const AFGBuildableRailroadTrack* Track = Connection->GetTrack();
		const float Length = Track->GetLength();
		const float Along = FMath::Clamp(Connection->GetTrackPosition().Offset < Length * 0.5f ? Distance : Length - Distance, 0.f, Length);
		FVector Direction;
		Track->GetWorldLocationAndDirectionAtDistance(Along, OutLocation, Direction);
	};
	TMap<int32, TArray<const UFGRailroadTrackConnectionComponent*>> JointMembers;
	for (const TPair<const UFGRailroadTrackConnectionComponent*, int32>& Pair : mJointOf)
	{
		if (Pair.Key->GetTrack())
		{
			JointMembers.FindOrAdd(Pair.Value).Add(Pair.Key);
		}
	}
	float LongestFoul = 0.f;
	for (const TPair<int32, TArray<const UFGRailroadTrackConnectionComponent*>>& Pair : JointMembers)
	{
		const TArray<const UFGRailroadTrackConnectionComponent*>& Members = Pair.Value;
		for (int32 A = 0; A < Members.Num(); ++A)
		{
			for (int32 B = A + 1; B < Members.Num(); ++B)
			{
				const AFGBuildableRailroadTrack* TrackA = Members[A]->GetTrack();
				const AFGBuildableRailroadTrack* TrackB = Members[B]->GetTrack();
				if (TrackA == TrackB)
				{
					continue;
				}
				// Only branches leaving the switch point the same way; the track on the other side runs away from both.
				FVector Origin, NearA, NearB;
				PointAt(Members[A], 0.f, Origin);
				PointAt(Members[A], 500.f, NearA);
				PointAt(Members[B], 500.f, NearB);
				if (FVector::DotProduct((NearA - Origin).GetSafeNormal(), (NearB - Origin).GetSafeNormal()) < 0.5f)
				{
					continue;
				}
				const float Reach = FMath::Min3(TrackA->GetLength(), TrackB->GetLength(), JointFoulMax);
				float Foul = Reach;
				for (float Distance = JointFoulStep; Distance <= Reach; Distance += JointFoulStep)
				{
					FVector OnA, OnB;
					PointAt(Members[A], Distance, OnA);
					PointAt(Members[B], Distance, OnB);
					if (FVector::Dist(OnA, OnB) >= JointClearance)
					{
						Foul = Distance;
						break;
					}
				}
				for (const UFGRailroadTrackConnectionComponent* Member : {Members[A], Members[B]})
				{
					float& Stored = mFoulOf.FindOrAdd(Member, JointFoulDistance);
					Stored = FMath::Max(Stored, Foul);
				}
				float& JointFoul = mJointFoul.FindOrAdd(Pair.Key, JointFoulDistance);
				JointFoul = FMath::Max(JointFoul, Foul);
				LongestFoul = FMath::Max(LongestFoul, Foul);
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
	BuildSearchGraph(Tracks, JointCount);
	mConflictsBuiltAt = FPlatformTime::Seconds();
	mConflictsStale = false;
	UE_LOG(LogAutoTrains, Log, TEXT("traffic: conflict model rebuilt: %d tracks, %d switch joints (longest fouling zone %.0f m), %d crossing tracks, %.1f ms"),
		Tracks.Num(), JointCount, LongestFoul / 100.f, Crossings, (mConflictsBuiltAt - Begin) * 1000.0);
}

void AATTrafficSubsystem::BuildSearchGraph(const TArray<AFGBuildableRailroadTrack*>& Tracks, int32 JointCount)
{
	const TSharedRef<FATSippGraph, ESPMode::ThreadSafe> Graph = MakeShared<FATSippGraph, ESPMode::ThreadSafe>();
	for (AFGBuildableRailroadTrack* Track : Tracks)
	{
		if (!Track || !Track->GetConnection(0) || !Track->GetConnection(1))
		{
			continue;
		}
		const int32 TrackIndex = Graph->Length.Add(Track->GetLength());
		Graph->Tracks.Add(Track);
		Graph->TrackIndex.Add(Track, TrackIndex);
		for (int32 End = 0; End < 2; ++End)
		{
			UFGRailroadTrackConnectionComponent* Connection = Track->GetConnection(End);
			Graph->ConnectionIndex.Add(Connection, Graph->Connections.Add(Connection));
			Graph->TrackOf.Add(TrackIndex);
		}
	}
	const int32 Count = Graph->Connections.Num();
	Graph->OppositeOf.Init(INDEX_NONE, Count);
	Graph->JointOf.Init(INDEX_NONE, Count);
	Graph->FoulOf.Init(0.f, Count);
	Graph->StopMarginOf.Init(0.f, Count);
	Graph->JointFoul.Init(JointFoulDistance, JointCount);
	Graph->OneWay.Init(false, Count);
	Graph->Attached.SetNum(Count);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const UFGRailroadTrackConnectionComponent* Connection = Graph->Connections[Index].Get();
		const int32* Opposite = Graph->ConnectionIndex.Find(Connection->GetOpposite());
		Graph->OppositeOf[Index] = Opposite ? *Opposite : INDEX_NONE;
		const int32 Joint = GetJoint(Connection);
		Graph->JointOf[Index] = Joint;
		Graph->FoulOf[Index] = GetFoulDistance(Connection);
		Graph->StopMarginOf[Index] = GetStopMargin(Connection);
		if (Graph->JointFoul.IsValidIndex(Joint))
		{
			Graph->JointFoul[Joint] = GetJointFoulDistance(Joint);
			Graph->MaxJointFoul = FMath::Max(Graph->MaxJointFoul, Graph->JointFoul[Joint]);
		}
		// The game's one-way rule: no passing a signal from behind.
		Graph->OneWay[Index] = Connection->GetTrailingSignal() && !Connection->GetFacingSignal();
		for (const UFGRailroadTrackConnectionComponent* Other : Connection->GetConnections())
		{
			if (const int32* OtherIndex = Graph->ConnectionIndex.Find(Other))
			{
				Graph->Attached[Index].Add(*OtherIndex);
			}
		}
	}
	Graph->Conflicts.SetNum(Graph->Length.Num());
	for (const TPair<const AFGBuildableRailroadTrack*, int32>& Pair : Graph->TrackIndex)
	{
		for (const AFGBuildableRailroadTrack* Conflict : GetConflicts(Pair.Key))
		{
			if (const int32* ConflictIndex = Graph->TrackIndex.Find(Conflict))
			{
				Graph->Conflicts[Pair.Value].Add(*ConflictIndex);
			}
		}
	}
	mSearchGraph = Graph;
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

float AATTrafficSubsystem::GetFoulDistance(const UFGRailroadTrackConnectionComponent* Connection) const
{
	const float* Foul = Connection ? mFoulOf.Find(Connection) : nullptr;
	return Foul ? *Foul : JointFoulDistance;
}

float AATTrafficSubsystem::GetJointFoulDistance(int32 Joint) const
{
	const float* Foul = mJointFoul.Find(Joint);
	return Foul ? *Foul : JointFoulDistance;
}

int32 AATTrafficSubsystem::GetJoint(const UFGRailroadTrackConnectionComponent* Connection) const
{
	const int32* Joint = Connection ? mJointOf.Find(Connection) : nullptr;
	return Joint ? *Joint : INDEX_NONE;
}

float AATTrafficSubsystem::GetStopMargin(const UFGRailroadTrackConnectionComponent* Exit) const
{
	return GetJoint(Exit) != INDEX_NONE ? FMath::Max(SwitchStopMargin, GetFoulDistance(Exit) + GetStopDistance()) : GetStopDistance();
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
	const float Mass = FMath::Max(State.Mass, 1.f);
	const float AirBrakes = Train ? Train->mConsistData.MaxAirBrakingEffort / Mass * RatedBrakeShare() : 100.f;
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
		if (Highest == INDEX_NONE)
		{
			Plan = AirBrakes;
		}
		else if (Band > Highest)
		{
			Plan = FMath::Min(State.PlanDecel[Highest], AirBrakes);
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

float AATTrafficSubsystem::GetAllowedSpeed(const FATTrafficTrain& State, float Distance, float Share)
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
		const float Deceleration = FMath::Max(State.PlanDecel[Band], MinPlanDeceleration) * Share;
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

float AATTrafficSubsystem::GetAcceleration(const FATTrafficTrain& State, float Share)
{
	const AFGTrain* Train = State.Train.Get();
	const float Tractive = Train ? Train->mConsistData.MaxTractiveEffort : 0.f;
	return FMath::Clamp(Tractive / FMath::Max(State.Mass, 1.f) * Share, 20.f, 1200.f);
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
	// Passing through a deadlock: the trains in its way don't hold it back.
	const bool bGhost = State->GhostUntil >= 0.0;
	const float FollowLimit = bGhost ? -1.f : GetFollowLimit(Train, *State, HardFollowLimit, PaceSpeed, &FollowInfo);
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
	// The target follows a service-brake curve that far ahead, so the game's driver, braking gently and a little
	// behind its target, stays under the full-brake curve that decides full brake.
	const float StopDistance = GetStopDistance();
	auto GetAllowed = [State, Distance, Usable, StopDistance](float Follow, float Lead, float Share)
	{
		float Result = MAX_flt;
		if (Distance >= 0.f)
		{
			Result = GetAllowedSpeed(*State, Usable - Lead, Share);
		}
		if (Follow >= 0.f)
		{
			Result = FMath::Min(Result, GetAllowedSpeed(*State, Follow - StopDistance - Lead, Share));
		}
		return Result;
	};
	const float Deceleration = GetPlanningDeceleration(*State, Speed);
	const float Allowed = GetAllowed(FollowLimit, Speed * TargetLeadSeconds, GetNormalBrakingShare());
	const float AllowedHard = GetAllowed(HardFollowLimit, 0.f, 1.f);
	float Target = FMath::Min(Allowed, PaceSpeed);
	// Waiting for a train that will clear: drive at the speed that reaches the braking point as it clears, so the
	// train rolls on instead of stopping.
	if (Distance >= 0.f && Usable > 0.f && State->ClearSeconds > 0.f)
	{
		const float Seconds = State->ClearSeconds;
		const float Service = Deceleration * GetNormalBrakingShare();
		Target = FMath::Min(Target, Service * (FMath::Sqrt(FMath::Square(Seconds) + 2.f * Usable / Service) - Seconds));
	}
	// Forced braking: when its own brakes won't stop the train in time, take off the rest of the speed directly, at
	// the deceleration that stops it at the stop point, the way the game's block signals hold a train. Once started
	// it holds, with full brake, until the train's own brakes suffice.
	const float StopWithin = FMath::Min(Distance >= 0.f ? Usable : MAX_flt, HardFollowLimit >= 0.f ? HardFollowLimit - StopDistance : MAX_flt);
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
		if (HardFollowLimit >= 0.f && (Distance < 0.f || HardFollowLimit - StopDistance < Usable))
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

	// A deadlocked train standing still shows the game's long-wait warning over it: its driver raises it while the
	// restrictive point says Stop, once its wait passes the threshold (the deadlock has lasted long enough already).
	if (State->bDeadlocked && Speed < DeadlockShowSpeed && Next)
	{
		if (!bArm)
		{
			Atc.NextRestrictiveSignalConnection = const_cast<UFGRailroadTrackConnectionComponent*>(Next);
			Atc.NextRestrictiveSignalSpeed = 0.f;
			Atc.NextRestrictiveSignalDistance = Atc.CurrentConnectionDistance;
			Atc.NextRestrictiveSignalGrade = 0.f;
			Atc.NextRestrictiveSignalStopOffset = Atc.CurrentConnectionDistance;
		}
		Atc.NextRestrictiveSignalAspect = ERailroadSignalAspect::RSA_Stop;
		Train->mSelfDrivingData.TimeWaitingAtSignal = FMath::Max(Train->mSelfDrivingData.TimeWaitingAtSignal, Train->mLongWaitAtSignalThreshold);
	}
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
		Lines.Add(FString::Printf(TEXT("%s: %.0f km/h (cruises %.0f, accelerates %.2f m/s2), %d claims, may drive %s, brakes at %.1f m/s2, waited %.0f s, rerouted %d times%s%s%s%s%s"),
			*TrainName(State.Train.Get()), State.Speed * 0.036f, State.CruiseSeen * 0.036f, State.AccelSeen / 100.f, State.Claims.Num(),
			Distance < 0.f ? TEXT("to its stop") : *FString::Printf(TEXT("%.0f m"), FMath::Max(0.f, Distance - State.StopMargin) / 100.f),
			GetPlanningDeceleration(State) / 100.f, State.WaitingSeconds, State.Reroutes, State.BlockedReason.IsEmpty() ? TEXT("") : TEXT(", "), *State.BlockedReason,
			State.ClearSeconds > 0.f ? *FString::Printf(TEXT(" (clears in %.0f s)"), State.ClearSeconds) : TEXT(""),
			State.PlanNote.IsEmpty() ? TEXT("") : TEXT("; "), *State.PlanNote));
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
