#include "ATTrafficSubsystem.h"
#include "ATConfig.h"
#include "ATControlSubsystem.h"
#include "ATRouter.h"
#include "ATSipp.h"
#include "Algo/AnyOf.h"
#include "AutoTrains.h"
#include "Buildables/FGBuildableRailroadSignal.h"
#include "Buildables/FGBuildableRailroadStation.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "EngineUtils.h"
#include "FGRailroadTimeTable.h"
#include "FGTrainStationIdentifier.h"
#include "RailroadNavigation.h"
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
	// Branches leaving a switch the same way foul it until their centre lines are the track clearance setting apart,
	// measured up to JointFoulMax; every track fouls it at least that clearance from the switch point. [cm]
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
	constexpr double ConflictRebuildMinSeconds = 2.0;

	// How far along its path a follower looks for the vehicles of the trains it follows. [cm]
	constexpr float GapSearchDistance = 300000.f;
	// Returned by GetDistanceToTrackStart for a track that isn't on the train's path.
	constexpr float TrackNotOnPath = -1e9f;

	// A deadlocked train slower than this shows the game's long-wait warning. [cm/s]
	constexpr float DeadlockShowSpeed = 50.f;
	// A deadlock not seen for this long is over. [s]
	constexpr double DeadlockForgetSeconds = 10.0;
	// A blocked train slower than this counts as waiting. [cm/s]
	constexpr float PlannerWaitSpeed = 300.f;
	// A wait ends once the train is let go for this long without a break, or once it moves faster than PlannerWaitSpeed. [s]
	constexpr float WaitEndFreeSeconds = 3.f;

	// Learning how a train drives: per update, its cruising speed moves this share towards a higher speed and this share
	// towards a lower one; its acceleration this share towards each sample. Plans use at least these. [cm/s] [cm/s^2]
	constexpr float CruiseRiseRate = 0.1f;
	constexpr float CruiseFallRate = 0.005f;
	constexpr float AccelSampleRate = 0.05f;
	// Share of its locomotives' traction a train is planned to use until some has been measured.
	constexpr float UnseenTractionShare = 0.5f;
	constexpr float MinPlanCruise = 833.f;
	constexpr float MinPlanAcceleration = 10.f;
	// Hill help: starts below this speed when the slope's pull is over this share of its traction and it wants to go
	// this much faster; gives this much more traction than the pull; ends once the pull is under this share. [cm/s]
	constexpr float HillHelpSpeed = 417.f;
	constexpr float HillStallShare = 0.9f;
	constexpr float HillWantMargin = 100.f;
	constexpr float HillHelpHeadroom = 1.3f;
	constexpr float HillHelpEndShare = 0.8f;
	// A train told to go counts as standing under this speed; each stall time raises its help by this much. [cm/s]
	constexpr float HillStandSpeed = 56.f;
	constexpr float HillStallStep = 1.25f;
	// A train counts as told to go only with at least this much room to where it has to stop. [cm]
	constexpr float HillStallRoom = 500.f;
	// A switch is set for a train this far ahead of it, on top of this many updates of its driving. [cm]
	constexpr float SwitchThrowMargin = 2000.f;
	constexpr float SwitchThrowUpdates = 2.f;

	void WriteFault(const FString& Fault)
	{
		const FString Path = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("AutoTrains") / TEXT("faults.log"));
		if (!FFileHelper::SaveStringToFile(Fault, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append))
		{
			UE_LOG(LogAutoTrains, Error, TEXT("Could not write the fault to %s"), *Path);
		}
	}

	// What the train's driver is doing and the forces on the train, for the log.
	FString DescribeDrive(const AFGTrain& Train)
	{
		const AFGLocomotive* Master = Train.GetMultipleUnitMaster();
		const UFGLocomotiveMovementComponent* Movement = Master ? Master->GetLocomotiveMovementComponent() : nullptr;
		const FTrainSimulationData& Simulation = Train.mSimulationData;
		return FString::Printf(TEXT("target %.0f km/h, throttle %.2f, air brake %.2f, dynamic brake %.2f, traction %.0f kN, brakes %.0f kN, resistance %.0f kN, gradient %.0f kN, power %d, %d target points, %s"),
			Train.mAtcData.TargetSpeed * 0.036f, Movement ? Movement->GetThrottle() : -1.f, Movement ? Movement->GetAirBrake() : -1.f, Movement ? Movement->GetDynamicBrake() : -1.f,
			Simulation.TractiveForce / 100000.f, (Simulation.AirBrakingForce + Simulation.DynamicBrakingForce) / 100000.f, Simulation.ResistiveForce / 100000.f, Simulation.GradientForce / 100000.f,
			Master && Master->HasPower() ? 1 : 0, Train.mAtcData.TargetPoints.Num(), *StaticEnum<ESelfDrivingLocomotiveError>()->GetNameStringByValue(static_cast<int64>(Train.GetSelfDrivingError())));
	}

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

	// Gravity. [cm/s^2]
	constexpr float GravityAcceleration = 980.f;

	// How much Connection's track rises per cm towards it. [cm/cm]
	float GetGrade(const UFGRailroadTrackConnectionComponent* Connection)
	{
		const UFGRailroadTrackConnectionComponent* Opposite = Connection ? Connection->GetOpposite() : nullptr;
		const AFGBuildableRailroadTrack* Track = Connection ? Connection->GetTrack() : nullptr;
		const float Length = Track ? Track->GetLength() : 0.f;
		return Opposite && Length > 0.f ? (Connection->GetComponentLocation().Z - Opposite->GetComponentLocation().Z) / Length : 0.f;
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

	// The signals the train would pass from behind on its shortest path to its timetable's current stop with signals
	// ignored (the game's pathfinder doesn't leave a track end that has only a signal facing the other way), for the log.
	FString DescribeSignalsInWay(AFGTrain* Train)
	{
		const AFGRailroadTimeTable* TimeTable = Train ? Train->GetTimeTable() : nullptr;
		TArray<FTimeTableStop> Stops;
		if (TimeTable)
		{
			TimeTable->GetStops(Stops);
		}
		const FTimeTableStop* Stop = Stops.IsValidIndex(TimeTable ? TimeTable->GetCurrentStop() : INDEX_NONE) ? &Stops[TimeTable->GetCurrentStop()] : nullptr;
		AFGBuildableRailroadStation* Station = Stop && Stop->Station ? Stop->Station->GetStation() : nullptr;
		if (!Station)
		{
			return TEXT("no next stop to check");
		}
		FRailroadGraphAStarFilter Filter;
		Filter.IgnoreSignals = true;
		TArray<FString> Best;
		int32 BestPoints = MAX_int32;
		for (AFGRailroadSubsystem::TTrainIterator It(Train->GetFirstVehicle()); It; ++It)
		{
			AFGLocomotive* Locomotive = Cast<AFGLocomotive>(*It);
			if (!Locomotive)
			{
				continue;
			}
			const FRailroadPathFindingResult Result = FRailroadNavigation::FindPathSync(Locomotive, Station, Filter);
			if (Result.Result != ERailroadPathFindingResult::RPFR_Success || !Result.Path.IsValid() || Result.Path->PathPoints.Num() >= BestPoints)
			{
				continue;
			}
			BestPoints = Result.Path->PathPoints.Num();
			Best.Reset();
			for (const FRailroadPathPoint& Point : Result.Path->PathPoints)
			{
				const UFGRailroadTrackConnectionComponent* Connection = Point.TrackConnection.Get();
				const AFGBuildableRailroadSignal* Signal = Connection && !Connection->GetFacingSignal() ? Connection->GetTrailingSignal() : nullptr;
				if (Signal)
				{
					const FVector Location = Signal->GetActorLocation();
					Best.Add(FString::Printf(TEXT("%s at the end of %s (%.0f, %.0f, %.0f)"), *GetNameSafe(Signal), *TrackName(Connection->GetTrack()),
						Location.X / 100.f, Location.Y / 100.f, Location.Z / 100.f));
				}
			}
		}
		const FString StationText = Stop->Station->GetStationName().ToString();
		if (BestPoints == MAX_int32)
		{
			return FString::Printf(TEXT("no path to %s even with signals ignored"), *StationText);
		}
		return FString::Printf(TEXT("to %s it would pass %d signal(s) from behind: %s"), *StationText, Best.Num(), *FString::Join(Best, TEXT("; ")));
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
	if (Settings.SwitchClearanceMetres != mSettings.SwitchClearanceMetres)
	{
		UE_LOG(LogAutoTrains, Log, TEXT("Settings: track clearance %.1f m; the conflict model is rebuilt"), Settings.SwitchClearanceMetres);
		mConflictsStale = true;
	}
	AATControlSubsystem* Control = AATControlSubsystem::Get(this);
	if (Control && (!mConfigDetailedLog.IsSet() || mConfigDetailedLog.GetValue() != Settings.bDetailedLog))
	{
		mConfigDetailedLog = Settings.bDetailedLog;
		LogAutoTrains.SetVerbosity(Settings.bDetailedLog ? ELogVerbosity::Verbose : ELogVerbosity::Log);
		UE_LOG(LogAutoTrains, Log, TEXT("Settings: detailed log %s; %s"), Settings.bDetailedLog ? TEXT("on") : TEXT("off"), *Control->SetTelemetry(Settings.bDetailedLog));
	}
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
	mPlannedOnce = false;
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

void AATTrafficSubsystem::UpdateTraffic(float DeltaSeconds)
{
	const double Now = FPlatformTime::Seconds();
	if (Now - mConflictsBuiltAt > mSettings.TrackRefreshSeconds || (mConflictsStale && Now - mConflictsBuiltAt > ConflictRebuildMinSeconds))
	{
		RebuildConflicts();
	}
	// Rebuilt since the last update, here or when traffic control came on: the crossings are numbered anew.
	const bool bRenumbered = mConflictsBuiltAt != mConflictsSeenAt;
	mConflictsSeenAt = mConflictsBuiltAt;

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
		const TSet<int32> WasOnCrossings = State.OccupiedCrossings;
		ReadTrain(State, Train);
		if (!bRenumbered)
		{
			LogCrossingEntries(State, WasOnCrossings, WorldNow);
		}
		TrackDocking(State, Train, WorldNow);
		UpdateHillHelp(State, WorldNow);
		WatchLostPath(State, WorldNow);
		ApplyLocomotivePower(Train, (Train->IsSelfDrivingEnabled() ? GAccelerationScale : 1.f) * State.HillBoost, Train->IsSelfDrivingEnabled() ? GTopSpeed * 0.036f : 0.f);

		// Measure full-brake deceleration per speed band between updates with full brake on at both; a catch ends at
		// standstill, and docking catches and crashes are far above 20 m/s^2.
		if (State.bFullBrake && State.bWasFullBrake && !State.bForcedSinceUpdate && State.LastSpeed > 500.f && State.Speed > 300.f && DeltaSeconds > 0.f)
		{
			// Measured with the slope's pull taken out, so it is the brakes' alone.
			const float Raw = (State.LastSpeed - State.Speed) / DeltaSeconds;
			if (Raw > 50.f && Raw < 2000.f)
			{
				const float Deceleration = FMath::Clamp(Raw - GravityAcceleration * State.Grade, 1.f, Train->mConsistData.MaxBrakeDeceleration * MaxRatedBrakeMultiple);
				const int32 Band = GetBrakeBand((State.LastSpeed + State.Speed) * 0.5f);
				const float Force = Deceleration * State.Mass;
				float& Stored = State.BrakeForce[Band];
				const bool bFirst = Stored <= 0.f;
				// Weaker samples count more, so the plan errs on the safe side.
				Stored = bFirst ? Force : Force < Stored ? Stored * 0.5f + Force * 0.5f : Stored * 0.85f + Force * 0.15f;
				if (bFirst)
				{
					UE_LOG(LogAutoTrains, Verbose, TEXT("traffic: %s brakes at %.2f m/s2 at %.0f-%.0f km/h (rated %.2f, %.0f t); planning with %.2f"), *TrainName(Train),
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
		if (bFree && State.HillBoost == 1.f && DeltaSeconds > 0.f && Target > State.Speed + 100.f && State.Speed > State.LastSpeed)
		{
			const float Sample = (State.Speed - State.LastSpeed) / DeltaSeconds + GravityAcceleration * State.Grade;
			if (Sample > 5.f && Sample < 500.f)
			{
				const int32 Band = GetBrakeBand((State.LastSpeed + State.Speed) * 0.5f);
				float& Stored = State.TractionForce[Band];
				const float Force = Sample * State.Mass;
				if (Stored <= 0.f)
				{
					UE_LOG(LogAutoTrains, Verbose, TEXT("traffic: %s speeds up at %.2f m/s2 at %.0f-%.0f km/h (%.0f t, slope %.1f %%)"), *TrainName(Train), Sample / 100.f,
						Band * ATBrakeBandWidth * 0.036f, (Band + 1) * ATBrakeBandWidth * 0.036f, State.Mass / 1000.f, State.Grade * 100.f);
				}
				Stored = Stored <= 0.f ? Force : Stored + (Force - Stored) * AccelSampleRate;
			}
		}
		State.bWasFullBrake = State.bFullBrake;
		State.bForcedSinceUpdate = State.bForcedBrake;
		State.LastSpeed = State.Speed;
		RefreshBrakePlan(State);
		RefreshTractionPlan(State);
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

	ThrowSwitches(DeltaSeconds);

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
		// Held up with no train named: the first train standing in its way ahead is the one it waits for.
		if (!State->BlockedReason.IsEmpty() && !State->BlockedBy.IsValid())
		{
			FString Where;
			if (const FATTrafficTrain* InWay = FindTrainInWay(*State, Where))
			{
				State->BlockedBy = InWay->Train;
				State->BlockedReason += FString::Printf(TEXT("; %s stands in its way on %s"), *TrainName(InWay->Train.Get()), *Where);
			}
		}
		if (LogKey(State) != PreviousReasons.FindRef(State) && !State->BlockedReason.IsEmpty())
		{
			UE_LOG(LogAutoTrains, Verbose, TEXT("traffic: %s: %s"), *TrainName(State->Train.Get()), *State->BlockedReason);
		}

		// Held up, even while it rolls slowly up to its stop; with or without a train to wait for. Let go for moments
		// without moving off, it is still waiting.
		const bool bHeldUp = !State->BlockedReason.IsEmpty();
		State->FreeSeconds = bHeldUp ? 0.f : State->FreeSeconds + DeltaSeconds;
		const bool bWaitGoesOn = bHeldUp || (State->WaitingSeconds > 0.f && State->FreeSeconds < WaitEndFreeSeconds);
		State->WaitingSeconds = bWaitGoesOn && State->Speed < PlannerWaitSpeed ? State->WaitingSeconds + DeltaSeconds : 0.f;
		State->UnnamedWaitSeconds = bHeldUp && State->WaitingSeconds > 0.f && !State->BlockedBy.IsValid() ? State->UnnamedWaitSeconds + DeltaSeconds : 0.f;
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
	State.OccupiedCrossings.Reset();
	State.FrontAhead = 0.f;
	State.Grade = 0.f;
	State.GradeAhead = 0.f;
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

	TArray<const UFGRailroadTrackConnectionComponent*> Overhangs;
	// Track ends a vehicle reaches past, with how far it reaches. [cm]
	TArray<TPair<const UFGRailroadTrackConnectionComponent*, float>> ZoneOverhangs;
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
			if (ToEnd < HalfLength - OverhangTolerance)
			{
				Overhangs.Add(Connection);
			}
			if (ToEnd < HalfLength)
			{
				ZoneOverhangs.Add({Connection, HalfLength - ToEnd});
			}
			const int32 Joint = GetJoint(Connection);
			if (Joint != INDEX_NONE && ToEnd < HalfLength + GetFoulDistance(Connection))
			{
				State.OccupiedJoints.Add(Joint);
			}
		}
		for (const FATZone& Zone : GetZones(Track))
		{
			if (Position.Offset - HalfLength < Zone.To && Position.Offset + HalfLength > Zone.From)
			{
				State.OccupiedCrossings.Add(Zone.Crossing);
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

	// A vehicle reaching past a track end overhangs onto the track it moves on to there: the only one attached, else (a
	// switch) the one the train's route or another of its vehicles is on; the switch's fouling zone covers the rest.
	const TSet<AFGBuildableRailroadTrack*> VehicleTracks = State.Occupied;
	const auto MovesOnTo = [&State, &VehicleTracks](const TArray<UFGRailroadTrackConnectionComponent*>& Attached, AFGBuildableRailroadTrack* Track)
	{
		return Track && (Attached.Num() == 1 || VehicleTracks.Contains(Track)
			|| State.Route.ContainsByPredicate([Track](const FATRouteStep& Step) { return Step.Track == Track; }));
	};
	for (const UFGRailroadTrackConnectionComponent* Connection : Overhangs)
	{
		const TArray<UFGRailroadTrackConnectionComponent*> Attached = Connection->GetConnections();
		for (const UFGRailroadTrackConnectionComponent* Other : Attached)
		{
			AFGBuildableRailroadTrack* Track = Other ? Other->GetTrack() : nullptr;
			if (MovesOnTo(Attached, Track))
			{
				State.Occupied.Add(Track);
			}
		}
	}
	// The overhanging part stands on that track's crossing zones near its end.
	for (const TPair<const UFGRailroadTrackConnectionComponent*, float>& Overhang : ZoneOverhangs)
	{
		const TArray<UFGRailroadTrackConnectionComponent*> Attached = Overhang.Key->GetConnections();
		for (const UFGRailroadTrackConnectionComponent* Other : Attached)
		{
			AFGBuildableRailroadTrack* Track = Other ? Other->GetTrack() : nullptr;
			if (!MovesOnTo(Attached, Track))
			{
				continue;
			}
			const float End = Other->GetTrackPosition().Offset;
			for (const FATZone& Zone : GetZones(Track))
			{
				if (Zone.From < End + Overhang.Value && Zone.To > End - Overhang.Value)
				{
					State.OccupiedCrossings.Add(Zone.Crossing);
				}
			}
		}
	}

	// Slope under the body: from the rear car's centre to the front car's, the way the train goes (gravity pulls on an
	// evenly loaded train as on that mean slope); a single car takes its route's first track.
	const AFGRailroadVehicle* Front = Train->mSimulationData.FirstVehicle ? Train->mSimulationData.FirstVehicle.Get() : Train->GetFirstVehicle();
	const AFGRailroadVehicle* Rear = Front == Train->GetFirstVehicle() ? Train->GetLastVehicle() : Train->GetFirstVehicle();
	if (Front && Rear)
	{
		State.FrontZ = Front->GetActorLocation().Z;
		State.RearZ = Rear->GetActorLocation().Z;
		State.FrontZAt = -Front->GetLength() * 0.5f;
		State.RearZAt = -(State.Length - Rear->GetLength() * 0.5f);
		const float Span = State.FrontZAt - State.RearZAt;
		State.Grade = Span > 100.f ? (State.FrontZ - State.RearZ) / Span : State.Route.Num() > 0 ? GetGrade(State.Route[0].Exit) : 0.f;
	}
	// Slope from the rear car to the route ahead.
	State.GradeAhead = State.Grade;
	for (int32 Index = 0; Index < State.Route.Num(); ++Index)
	{
		const FATRouteStep& Step = State.Route[Index];
		const float Ahead = Step.EndDistance - State.FrontAhead;
		if (Ahead >= mSettings.SlopeLookaheadMetres * 100.f || Index == State.Route.Num() - 1)
		{
			if (Ahead > 0.f && Step.Exit)
			{
				State.GradeAhead = (Step.Exit->GetComponentLocation().Z - State.RearZ) / (Ahead - State.RearZAt);
			}
			break;
		}
	}

	// The consist's own mass leaves out cargo loaded since it was last worked out.
	State.Mass = FMath::Max3(State.Mass, Train->mConsistData.Mass, 1.f);

	if (!State.bManaged)
	{
		State.Claims.Reset();
		State.ClaimedJoints.Reset();
		State.ClaimedCrossings.Reset();
		State.AuthorityCut = -1.f;
		State.FollowLeaders.Reset();
		State.FollowedTrains.Reset();
		State.AuthorityTrack = nullptr;
		State.bAuthorityToGoal = false;
		State.BlockedReason.Reset();
		State.BlockedBy = nullptr;
		State.WaitingSeconds = 0.f;
		State.UnnamedWaitSeconds = 0.f;
		State.bFullBrake = false;
		State.bForcedBrake = false;
		State.bCatchArmed = false;
		State.bHasPlan = false;
	}
}

const FATTrafficTrain* AATTrafficSubsystem::FindTrainInWay(const FATTrafficTrain& State, FString& OutWhere) const
{
	for (int32 Index = 1; Index < State.Route.Num(); ++Index)
	{
		const FATRouteStep& Step = State.Route[Index];
		if (State.Occupied.Contains(Step.Track))
		{
			continue;
		}
		const int32 Joint = GetJoint(State.Route[Index - 1].Exit);
		for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
		{
			const FATTrafficTrain& Other = Pair.Value;
			if (&Other != &State && Other.Train.IsValid() && (Other.Occupied.Contains(Step.Track) || (Joint != INDEX_NONE && Other.OccupiedJoints.Contains(Joint))))
			{
				OutWhere = TrackName(Step.Track);
				return &Other;
			}
		}
	}
	return nullptr;
}

bool AATTrafficSubsystem::HasToLeaveThrough(const FATTrafficTrain& Other, const UFGRailroadTrackConnectionComponent* Exit, float Within) const
{
	const AFGBuildableRailroadTrack* Track = Exit->GetTrack();
	const auto HasVehicleOn = [&Other](const AFGBuildableRailroadTrack* On)
	{
		return Other.Vehicles.ContainsByPredicate([On](const FATVehicleSpot& Spot) { return Spot.Track == On; });
	};
	const bool bOnTrack = HasVehicleOn(Track);
	for (const FATRouteStep& Step : Other.Route)
	{
		if (Step.Track == Track)
		{
			return Step.Exit == Exit && (bOnTrack || Step.EndDistance - Other.FrontAhead < Within);
		}
	}
	if (!bOnTrack)
	{
		return false;
	}
	// Its front is past the track or it has no route: it is leaving through Exit if it reaches past it.
	for (const UFGRailroadTrackConnectionComponent* Attached : Exit->GetConnections())
	{
		const AFGBuildableRailroadTrack* Next = Attached ? Attached->GetTrack() : nullptr;
		if (HasVehicleOn(Next) || Other.Route.ContainsByPredicate([Next](const FATRouteStep& Step) { return Step.Track == Next; }))
		{
			return true;
		}
	}
	return Other.Route.Num() == 0 && Other.Speed >= HillStandSpeed;
}

void AATTrafficSubsystem::ThrowSwitches(float DeltaSeconds)
{
	for (TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		FATTrafficTrain& State = Pair.Value;
		AFGTrain* Train = State.Train.Get();
		if (!Train || !State.bManaged)
		{
			continue;
		}
		const float Drives = State.Speed * DeltaSeconds * SwitchThrowUpdates;
		for (int32 Index = 0; Index + 1 < State.Route.Num(); ++Index)
		{
			UFGRailroadTrackConnectionComponent* Exit = State.Route[Index].Exit;
			const float Ahead = State.Route[Index].EndDistance - State.FrontAhead;
			if (Ahead > Drives + SwitchThrowMargin)
			{
				break;
			}
			if (Ahead < 0.f || !Exit || !Exit->IsFacingSwitch())
			{
				continue;
			}
			const int32 Position = Exit->GetSwitchPositionForTrack(State.Route[Index + 1].Track);
			if (Position == INDEX_NONE || Position == Exit->GetSwitchPosition())
			{
				continue;
			}
			const FATTrafficTrain* Leaving = nullptr;
			for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& OtherPair : mTrains)
			{
				if (&OtherPair.Value != &State && HasToLeaveThrough(OtherPair.Value, Exit, Ahead))
				{
					Leaving = &OtherPair.Value;
					break;
				}
			}
			if (!Leaving)
			{
				if (!Exit->IsSwitchClear(Train))
				{
					UE_LOG(LogAutoTrains, Verbose, TEXT("traffic: %s sets the switch at the end of %s, %.0f m ahead, which the game wouldn't: another train is within 12 m of it"), *TrainName(Train),
						*TrackName(State.Route[Index].Track), Ahead / 100.f);
				}
				Exit->SetSwitchPosition(Position);
			}
			else if (Ahead <= Drives && State.WrongSwitch != Exit)
			{
				State.WrongSwitch = Exit;
				UE_LOG(LogAutoTrains, Warning, TEXT("traffic: WRONG SWITCH: %s reaches the switch at the end of %s, %.0f m ahead, set for %s, which still has to go through it"), *TrainName(Train),
					*TrackName(State.Route[Index].Track), Ahead / 100.f, *TrainName(Leaving->Train.Get()));
			}
		}
	}
}

void AATTrafficSubsystem::WatchLostPath(FATTrafficTrain& State, double Now) const
{
	const AFGTrain* Train = State.Train.Get();
	if (State.bManaged && State.bLostReported)
	{
		UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s has a path again after %.0f s"), *TrainName(Train), Now - State.LostSince);
	}
	if (State.bManaged || !Train || !Train->IsSelfDrivingEnabled() || Train->IsDocked())
	{
		State.bHadPath = State.bManaged;
		State.LostSince = -1.0;
		State.bLostReported = false;
		return;
	}
	if (!State.bHadPath)
	{
		return;
	}
	State.LostSince = State.LostSince < 0.0 ? Now : State.LostSince;
	if (State.bLostReported || Now - State.LostSince < mSettings.DeadlockWaitSeconds)
	{
		return;
	}
	State.bLostReported = true;
	const FString Where = State.Vehicles.Num() > 0 ? TrackName(State.Vehicles[0].Track) : TEXT("no track");
	const bool bSignals = Train->GetSelfDrivingError() == ESelfDrivingLocomotiveError::SDLE_StationUnreachableWithSignals;
	const FString Fault = FString::Printf(TEXT("%s %s has had no path for %.0f s (on %s, %.0f km/h): %s%s%s\n"), *FDateTime::Now().ToString(TEXT("%Y-%m-%d %H:%M:%S")), *TrainName(Train),
		Now - State.LostSince, *Where, State.Speed * 0.036f, *DescribeDrive(*Train), bSignals ? TEXT("; ") : TEXT(""),
		bSignals ? *DescribeSignalsInWay(const_cast<AFGTrain*>(Train)) : TEXT(""));
	UE_LOG(LogAutoTrains, Warning, TEXT("traffic: LOST: %s"), *Fault.TrimEnd());
	WriteFault(Fault);
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
	// Crossing zones on its held tracks that its body hasn't passed yet; on the last one, only those before where its
	// grant ends.
	State.ClaimedCrossings.Reset();
	for (int32 Index = 0; Index < State.Route.Num() && HoldsTrack(State, State.Route[Index].Track); ++Index)
	{
		const FATRouteStep& Step = State.Route[Index];
		const bool bLast = Index + 1 == State.Route.Num() || !HoldsTrack(State, State.Route[Index + 1].Track);
		const float Begin = Step.EndDistance - Step.Track->GetLength() - State.FrontAhead;
		for (const FATZone& Zone : GetZones(Step.Track))
		{
			float From = 0.f;
			float To = 0.f;
			GetZoneAlong(Zone, Step.Track, Step.Exit, From, To);
			if (Begin + To + State.Length <= 0.f)
			{
				continue;
			}
			if (!bLast || State.AuthorityCut < 0.f || Step.Track->GetLength() - From > State.AuthorityCut)
			{
				State.ClaimedCrossings.Add(Zone.Crossing);
			}
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
		State.StopMargin = FMath::Max(GetStopMargin(State.Route[Last].Exit), State.AuthorityCut);
	}
	RefreshClaimedJoints(State);
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
			return Track && HoldsTrack(Other, Track);
		}) || State.ClaimedJoints.Array().ContainsByPredicate([&Other](int32 Joint) { return HoldsJoint(Other, Joint); })
			|| State.ClaimedCrossings.Array().ContainsByPredicate([&Other](int32 Crossing) { return HoldsCrossing(Other, Crossing); });
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

float AATTrafficSubsystem::GetFollowLimit(const AFGTrain* Train, const FATTrafficTrain& State, float& OutHardLimit, FATFollowInfo* OutInfo) const
{
	float Limit = -1.f;
	OutHardLimit = -1.f;
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
	// The start of the first switch or crossing zone ahead of the front that the train would stand on if it stopped behind
	// a leader at Limit; -1 if none. [cm]
	auto JunctionBefore = [this, Train, &State](float Limit)
	{
		const float Front = Limit - GetStopDistance();
		const float Rear = Front - State.Length;
		for (int32 Index = 0; Index < State.Route.Num(); ++Index)
		{
			const FATRouteStep& Step = State.Route[Index];
			const float Start = GetDistanceToTrackStart(Train, State, Step.Track);
			if (Start <= TrackNotOnPath || Start >= Front + GetClearance())
			{
				break;
			}
			const int32 Joint = Index > 0 ? GetJoint(State.Route[Index - 1].Exit) : INDEX_NONE;
			if (Joint != INDEX_NONE)
			{
				const float Foul = GetJointFoulDistance(Joint);
				if (Start - Foul > 0.f && Start - Foul < Front && Start + Foul > Rear)
				{
					return Start - Foul;
				}
			}
			for (const FATZone& Zone : GetZones(Step.Track))
			{
				float From = 0.f;
				float To = 0.f;
				GetZoneAlong(Zone, Step.Track, Step.Exit, From, To);
				if (Start + From > 0.f && Start + From < Front && Start + To > Rear)
				{
					return Start + From;
				}
			}
		}
		return -1.f;
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
			// It doesn't stop on a switch or crossing behind the leader: it waits before it until there's room past it, if it
			// can still stop there.
			const float Junction = JunctionBefore(Gap + (bMovesAway ? HardBrake : 0.f));
			if (Junction >= 0.f && Junction - GetStopDistance() >= GetStoppingDistance(State, State.Speed))
			{
				Add(Junction, 0.f, 0.f, true);
			}
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
			const bool bHeldTrack = HoldsTrack(*Leader, Step.Track);
			// The first crossing zone along the track that the leader holds, from the track's start. [cm]
			float ZoneAt = -1.f;
			for (const FATZone& Zone : GetZones(Step.Track))
			{
				float From = 0.f;
				float To = 0.f;
				GetZoneAlong(Zone, Step.Track, Step.Exit, From, To);
				if (HoldsCrossing(*Leader, Zone.Crossing) && (ZoneAt < 0.f || From < ZoneAt))
				{
					ZoneAt = From;
				}
			}
			if (!bHeldJoint && !bHeldTrack && ZoneAt < 0.f)
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
			else if (!bHeldTrack)
			{
				Start += ZoneAt;
			}
			// A leader merging the same way moves on ahead; one crossing leaves the conflict only once it has passed.
			const bool bSameWay = Leader->Route.ContainsByPredicate([&Step](const FATRouteStep& LeaderStep)
			{
				return LeaderStep.Track == Step.Track && LeaderStep.Exit == Step.Exit;
			});
			Add(FMath::Max(Start, 0.f), bSameWay ? SoftBrake : 0.f, bSameWay ? HardBrake : 0.f, false);
			break;
		}
	}
	return Limit;
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
	// One train at a time, so no two run onto the same switch together: the first whose way is free of the train it
	// waits for, else the first.
	const FATTrafficTrain* const* Free = Chain.FindByPredicate([this](const FATTrafficTrain* Member) { return !IsWayTaken(*Member); });
	const FATTrafficTrain* Picked = Free ? *Free : Chain[0];
	FATTrafficTrain* Stuck = mTrains.Find(Picked->Train);
	if (!Stuck)
	{
		return;
	}
	Stuck->GhostUntil = Now + mSettings.GhostMaxSeconds;
	Stuck->GhostLeft = Stuck->Length + mSettings.GhostDistanceMetres * 100.f;
	TArray<FString> Lines;
	for (const FATTrafficTrain* Member : Chain)
	{
		Lines.Add(FString::Printf(TEXT("%s%s: %s"), Member == Picked ? TEXT("passes through: ") : TEXT(""), *TrainName(Member->Train.Get()), *Member->BlockedReason));
	}
	GGhosting = true;
	const FString Fault = FString::Printf(TEXT("%s deadlock of %d trains passed through after %.0f s:\n  %s\n"), *FDateTime::Now().ToString(TEXT("%Y-%m-%d %H:%M:%S")), Chain.Num(),
		Seconds, *FString::Join(Lines, TEXT("\n  ")));
	UE_LOG(LogAutoTrains, Warning, TEXT("traffic: DEADLOCK passed through: %s"), *Fault);
	WriteFault(Fault);
}

bool AATTrafficSubsystem::IsWayTaken(const FATTrafficTrain& State) const
{
	const FATTrafficTrain* Blocker = mTrains.Find(State.BlockedBy);
	if (!Blocker)
	{
		return true;
	}
	if (State.BlockedAtJoint != INDEX_NONE)
	{
		return HoldsJoint(*Blocker, State.BlockedAtJoint);
	}
	if (State.BlockedAtCrossing != INDEX_NONE)
	{
		return HoldsCrossing(*Blocker, State.BlockedAtCrossing);
	}
	return !State.BlockedAtTrack || HoldsTrack(*Blocker, State.BlockedAtTrack);
}

void AATTrafficSubsystem::LogDeadlocks()
{
	const double Now = GetWorld()->GetTimeSeconds();
	// Follow each long-waiting train's chain of trains it waits for; a chain that comes back is a deadlock. Every train
	// in one is marked; a cycle is logged when it first shows up. One that drops out of sight for a moment and comes back
	// within DeadlockForgetSeconds is the same deadlock still going on.
	for (TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		Pair.Value.bInDeadlock = false;
	}
	TSet<const FATTrafficTrain*> InCycle;
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const FATTrafficTrain& Start = Pair.Value;
		if (!Start.bManaged || Start.WaitingSeconds < mSettings.DeadlockWaitSeconds || InCycle.Contains(&Start))
		{
			continue;
		}
		TArray<const FATTrafficTrain*> Chain;
		const FATTrafficTrain* Current = &Start;
		while (Current && !Chain.Contains(Current) && Chain.Num() < 64)
		{
			Chain.Add(Current);
			Current = mTrains.Find(Current->BlockedBy);
			if (Current && (!Current->bManaged || Current->WaitingSeconds < mSettings.DeadlockWaitSeconds))
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
			if (FATTrafficTrain* Marked = mTrains.Find(Member->Train))
			{
				Marked->bInDeadlock = true;
			}
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
			UE_LOG(LogAutoTrains, Warning, TEXT("traffic: DEADLOCK:   %s: %s; that train %s the place"), *TrainName(Member->Train.Get()), *Member->BlockedReason,
				IsWayTaken(*Member) ? TEXT("stands on or holds") : TEXT("neither stands on nor holds"));
			LogWaitDump(*Member, Now);
			// Every train in the cycle looks for another way at once; a train waiting for it counts as impassable.
			if (FATTrafficTrain* Stuck = mTrains.Find(Member->Train))
			{
				Stuck->RerouteCooldown = 0.f;
				Stuck->RerouteTimer = 0.f;
			}
		}
	}
	// A train held up for long with no train to wait for all that time: nothing ahead will let it go, so it and every
	// train whose chain of waits ends at it are stuck. Logged when it first shows up; it looks for another way at once.
	auto IsWaiting = [this](const FATTrafficTrain* State) { return State && State->bManaged && State->WaitingSeconds >= mSettings.DeadlockWaitSeconds; };
	TSet<const FATTrafficTrain*> InStall;
	for (TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		FATTrafficTrain& Root = Pair.Value;
		if (!IsWaiting(&Root) || Root.UnnamedWaitSeconds < mSettings.DeadlockWaitSeconds || InCycle.Contains(&Root))
		{
			continue;
		}
		TArray<FString> Behind;
		InStall.Add(&Root);
		for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Other : mTrains)
		{
			const FATTrafficTrain* Current = &Other.Value;
			for (int32 Steps = 0; Current != &Root && IsWaiting(Current) && Steps < 64; ++Steps)
			{
				Current = mTrains.Find(Current->BlockedBy);
			}
			if (Current == &Root && &Other.Value != &Root)
			{
				Behind.Add(TrainName(Other.Value.Train.Get()));
				InStall.Add(&Other.Value);
			}
		}
		const FString Signature = TEXT("stuck|") + TrainName(Root.Train.Get());
		FDeadlockSeen* Seen = mDeadlocks.Find(Signature);
		const bool bNew = !Seen;
		if (bNew)
		{
			Seen = &mDeadlocks.Add(Signature, {Now, Now});
		}
		Seen->Last = Now;
		if (!bNew)
		{
			continue;
		}
		Behind.Sort();
		const FString Fault = FString::Printf(TEXT("%s %s has waited %.0f s with no train to wait for: %s; %d waiting behind it%s%s\n"),
			*FDateTime::Now().ToString(TEXT("%Y-%m-%d %H:%M:%S")), *TrainName(Root.Train.Get()), Root.UnnamedWaitSeconds, *Root.BlockedReason, Behind.Num(),
			Behind.Num() > 0 ? TEXT(": ") : TEXT(""), *FString::Join(Behind, TEXT(", ")));
		UE_LOG(LogAutoTrains, Warning, TEXT("traffic: STUCK: %s"), *Fault.TrimEnd());
		WriteFault(Fault);
		Root.RerouteCooldown = 0.f;
		Root.RerouteTimer = 0.f;
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
		Pair.Value.bDeadlocked = InCycle.Contains(&Pair.Value) || InStall.Contains(&Pair.Value);
	}
}

AATTrafficSubsystem::FRouteSpeed AATTrafficSubsystem::GetRouteSpeed(const FATTrafficTrain& State) const
{
	FRouteSpeed Speed;
	// As the train has been seen to drive, within its top speed; until then its top speed.
	const float Top = FMath::Max(GetTopSpeed(State.Train.Get()), 500.f);
	Speed.Cruise = State.CruiseSeen > 0.f ? FMath::Clamp(State.CruiseSeen, FMath::Min(MinPlanCruise, Top), Top) : Top;
	Speed.Length = State.Length;
	// The planned acceleration's time-equivalent mean from standstill up to cruising speed, on level track.
	double Seconds = 0.0;
	for (int32 Band = 0; Band < ATBrakeBands && Band * ATBrakeBandWidth < Speed.Cruise; ++Band)
	{
		const float Low = Band * ATBrakeBandWidth;
		Seconds += (FMath::Min(Low + ATBrakeBandWidth, Speed.Cruise) - Low) / FMath::Max(State.PlanAccel[Band], MinPlanAcceleration);
	}
	const float Acceleration = Seconds > 0.0 ? Speed.Cruise / Seconds : GetAcceleration(State, UnseenTractionShare);
	// On level track: the planner adds each stretch's slope itself.
	const float Deceleration = FMath::Max(State.PlanDecel[GetBrakeBand(Speed.Cruise)], MinPlanDeceleration) * GetNormalBrakingShare();
	Speed.Accel = Acceleration;
	Speed.Brake = FMath::Max(Deceleration, MinPlanDeceleration);
	return Speed;
}

void AATTrafficSubsystem::RebuildConflicts()
{
	const float Clearance = GetClearance();
	const double Begin = FPlatformTime::Seconds();
	mZonesOf.Reset();
	mCrossingCount = 0;
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
					if (FVector::Dist(OnA, OnB) >= Clearance)
					{
						Foul = FMath::Max(Distance, Clearance);
						break;
					}
				}
				for (const UFGRailroadTrackConnectionComponent* Member : {Members[A], Members[B]})
				{
					float& Stored = mFoulOf.FindOrAdd(Member, Clearance);
					Stored = FMath::Max(Stored, Foul);
				}
				float& JointFoul = mJointFoul.FindOrAdd(Pair.Key, Clearance);
				JointFoul = FMath::Max(JointFoul, Foul);
				LongestFoul = FMath::Max(LongestFoul, Foul);
			}
		}
	}

	// Crossings: the game's overlapping tracks, both ways, except branches of the same switch, which the joint covers.
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
	// The game counts tracks whose outlines overlap; they cross only where their centre lines come within the clearance.
	// Each crossing is a zone on either track: the stretch of it whose centre line comes that close to the other's.
	auto Sample = [](const AFGBuildableRailroadTrack* Track, TArray<TPair<float, FVector>>& Out)
	{
		const float Length = Track->GetLength();
		for (float Along = 0.f;; Along = FMath::Min(Along + JointFoulStep, Length))
		{
			FVector Location, Direction;
			Track->GetWorldLocationAndDirectionAtDistance(Along, Location, Direction);
			Out.Add({Along, Location});
			if (Along >= Length)
			{
				break;
			}
		}
	};
	auto Stretch = [Clearance](const TArray<TPair<float, FVector>>& Of, const TArray<TPair<float, FVector>>& Other, float& OutFrom, float& OutTo)
	{
		OutFrom = MAX_flt;
		OutTo = -MAX_flt;
		float Closest = MAX_flt;
		for (const TPair<float, FVector>& Point : Of)
		{
			float Nearest = MAX_flt;
			for (const TPair<float, FVector>& OtherPoint : Other)
			{
				Nearest = FMath::Min(Nearest, FVector::DistSquared(Point.Value, OtherPoint.Value));
			}
			Nearest = FMath::Sqrt(Nearest);
			Closest = FMath::Min(Closest, Nearest);
			if (Nearest < Clearance)
			{
				OutFrom = FMath::Min(OutFrom, Point.Key);
				OutTo = FMath::Max(OutTo, Point.Key);
			}
		}
		return Closest;
	};
	TSet<TPair<const AFGBuildableRailroadTrack*, const AFGBuildableRailroadTrack*>> Checked;
	int32 Clear = 0;
	for (AFGBuildableRailroadTrack* Track : Tracks)
	{
		for (AFGBuildableRailroadTrack* Overlapping : Track->GetOverlappingTracks())
		{
			if (!Overlapping || Overlapping == Track || SharesJoint(Track, Overlapping)
				|| Checked.Contains(TPair<const AFGBuildableRailroadTrack*, const AFGBuildableRailroadTrack*>(Overlapping, Track)))
			{
				continue;
			}
			Checked.Add(TPair<const AFGBuildableRailroadTrack*, const AFGBuildableRailroadTrack*>(Track, Overlapping));
			TArray<TPair<float, FVector>> PointsA, PointsB;
			Sample(Track, PointsA);
			Sample(Overlapping, PointsB);
			float FromA = 0.f, ToA = 0.f, FromB = 0.f, ToB = 0.f;
			const float Distance = Stretch(PointsA, PointsB, FromA, ToA);
			if (Distance >= Clearance)
			{
				++Clear;
				const FString Pair = TrackName(Track) + TEXT("|") + TrackName(Overlapping);
				if (!mLoggedClearPairs.Contains(Pair))
				{
					mLoggedClearPairs.Add(Pair);
					UE_LOG(LogAutoTrains, Verbose, TEXT("traffic: %s and %s overlap for the game but their centre lines stay %.1f m apart: not a crossing"),
						*TrackName(Track), *TrackName(Overlapping), Distance / 100.f);
				}
				continue;
			}
			Stretch(PointsB, PointsA, FromB, ToB);
			const int32 Crossing = mCrossingCount++;
			// A sample step either side, as the stretch is only known to that.
			mZonesOf.FindOrAdd(Track).Add({Crossing, FMath::Max(FromA - JointFoulStep, 0.f), FMath::Min(ToA + JointFoulStep, Track->GetLength())});
			mZonesOf.FindOrAdd(Overlapping).Add({Crossing, FMath::Max(FromB - JointFoulStep, 0.f), FMath::Min(ToB + JointFoulStep, Overlapping->GetLength())});
		}
	}
	BuildSearchGraph(Tracks, JointCount);
	mConflictsBuiltAt = FPlatformTime::Seconds();
	mConflictsStale = false;
	UE_LOG(LogAutoTrains, Log, TEXT("traffic: conflict model rebuilt: %d tracks, %d switch joints (longest fouling zone %.0f m), %d crossings, %d overlapping pairs that don't cross, %.1f ms"),
		Tracks.Num(), JointCount, LongestFoul / 100.f, mCrossingCount, Clear, (mConflictsBuiltAt - Begin) * 1000.0);
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
	Graph->ZOf.Init(0.f, Count);
	Graph->StopMarginOf.Init(0.f, Count);
	Graph->JointFoul.Init(GetClearance(), JointCount);
	Graph->AtEnd.Init(false, Count);
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
		Graph->ZOf[Index] = Connection->GetComponentLocation().Z;
		Graph->AtEnd[Index] = Connection->GetTrackPosition().Offset > Graph->Length[Graph->TrackOf[Index]] * 0.5f;
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
	TArray<bool> Platform;
	for (const AFGBuildableRailroadTrack* Track : Graph->Tracks)
	{
		Platform.Add(Track && Track->GetIsOwnedByPlatform());
	}
	Graph->NumCrossings = mCrossingCount;
	Graph->StopDistance = GetStopDistance();
	Graph->ZonesOf.SetNum(Graph->Length.Num());
	for (const TPair<const AFGBuildableRailroadTrack*, int32>& Pair : Graph->TrackIndex)
	{
		for (const FATZone& Zone : GetZones(Pair.Key))
		{
			Graph->ZonesOf[Pair.Value].Add({Graph->CrossingResource(Zone.Crossing), Zone.From, Zone.To});
		}
	}
	// Nothing is decided where the only way on is a plain track: no switch, no one-way signal, no platform, no crossing.
	Graph->Through.Init(INDEX_NONE, Count);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		if (Graph->Attached[Index].Num() != 1 || Graph->JointOf[Index] != INDEX_NONE || Graph->OneWay[Index] || Platform[Graph->TrackOf[Index]])
		{
			continue;
		}
		const int32 Entry = Graph->Attached[Index][0];
		const int32 NextTrack = Graph->TrackOf[Entry];
		if (Graph->OppositeOf[Entry] != INDEX_NONE && Graph->JointOf[Entry] == INDEX_NONE && !Platform[NextTrack] && Graph->ZonesOf[NextTrack].Num() == 0)
		{
			Graph->Through[Index] = Graph->OppositeOf[Entry];
		}
	}
	mSearchGraph = Graph;
}

float AATTrafficSubsystem::GetFoulDistance(const UFGRailroadTrackConnectionComponent* Connection) const
{
	const float* Foul = Connection ? mFoulOf.Find(Connection) : nullptr;
	return Foul ? *Foul : GetClearance();
}

float AATTrafficSubsystem::GetJointFoulDistance(int32 Joint) const
{
	const float* Foul = mJointFoul.Find(Joint);
	return Foul ? *Foul : GetClearance();
}

int32 AATTrafficSubsystem::GetJoint(const UFGRailroadTrackConnectionComponent* Connection) const
{
	const int32* Joint = Connection ? mJointOf.Find(Connection) : nullptr;
	return Joint ? *Joint : INDEX_NONE;
}

float AATTrafficSubsystem::GetStopMargin(const UFGRailroadTrackConnectionComponent* Exit) const
{
	return GetJoint(Exit) != INDEX_NONE ? GetFoulDistance(Exit) + GetStopDistance() : GetStopDistance();
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

bool AATTrafficSubsystem::HoldsCrossing(const FATTrafficTrain& State, int32 Crossing)
{
	return State.OccupiedCrossings.Contains(Crossing) || State.ClaimedCrossings.Contains(Crossing);
}

const TArray<FATZone>& AATTrafficSubsystem::GetZones(const AFGBuildableRailroadTrack* Track) const
{
	static const TArray<FATZone> None;
	const TArray<FATZone>* Zones = mZonesOf.Find(Track);
	return Zones ? *Zones : None;
}

void AATTrafficSubsystem::GetZoneAlong(const FATZone& Zone, const AFGBuildableRailroadTrack* Track, const UFGRailroadTrackConnectionComponent* Exit, float& OutFrom, float& OutTo)
{
	const float Length = Track->GetLength();
	const bool bAtEnd = Exit && Exit->GetTrackPosition().Offset > Length * 0.5f;
	OutFrom = bAtEnd ? Zone.From : Length - Zone.To;
	OutTo = bAtEnd ? Zone.To : Length - Zone.From;
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

void AATTrafficSubsystem::RefreshTractionPlan(FATTrafficTrain& State) const
{
	const AFGTrain* Train = State.Train.Get();
	const float Mass = FMath::Max(State.Mass, 1.f);
	float Curve[ATBrakeBands] = {};
	for (AFGRailroadSubsystem::TTrainIterator It(Train ? Train->GetFirstVehicle() : nullptr); It; ++It)
	{
		const AFGLocomotive* Locomotive = Cast<AFGLocomotive>(*It);
		const UFGLocomotiveMovementComponent* Movement = Locomotive ? Locomotive->GetLocomotiveMovementComponent() : nullptr;
		for (int32 Band = 0; Movement && Band < ATBrakeBands; ++Band)
		{
			Curve[Band] += Movement->CalcTractiveEffort((Band + 0.5f) * ATBrakeBandWidth);
		}
	}
	float Share = 0.f;
	int32 Measured = 0;
	for (int32 Band = 0; Band < ATBrakeBands; ++Band)
	{
		if (State.TractionForce[Band] > 0.f && Curve[Band] > 0.f)
		{
			Share += State.TractionForce[Band] / Curve[Band];
			++Measured;
		}
	}
	Share = Measured > 0 ? Share / Measured : UnseenTractionShare;
	// Curve as applied, the hill help included.
	State.FullTraction = Curve[0] / FMath::Max(State.HillBoost, 1.f);
	const float Fallback = GetAcceleration(State, UnseenTractionShare);
	for (int32 Band = 0; Band < ATBrakeBands; ++Band)
	{
		const float Force = State.TractionForce[Band] > 0.f ? State.TractionForce[Band] : Curve[Band] * Share;
		State.PlanAccel[Band] = FMath::Clamp(Force > 0.f ? Force / Mass : Fallback, MinPlanAcceleration, 1200.f);
	}
}

void AATTrafficSubsystem::UpdateHillHelp(FATTrafficTrain& State, double Now) const
{
	const AFGTrain* Train = State.Train.Get();
	const float Pull = GravityAcceleration * State.Grade * State.Mass;
	const bool bOn = mSettings.bHillHelp && Train && Train->IsSelfDrivingEnabled() && State.bManaged && State.FullTraction > 0.f;
	// Braking with the throttle shut: more traction can't move it. Held up by its plan, braking or not on a climb, it
	// doesn't count as wanting to go.
	const AFGLocomotive* Master = Train ? Train->GetMultipleUnitMaster() : nullptr;
	const UFGLocomotiveMovementComponent* Movement = Master ? Master->GetLocomotiveMovementComponent() : nullptr;
	const bool bBraking = Movement && Movement->GetThrottle() <= 0.f && Movement->GetAirBrake() + Movement->GetDynamicBrake() > 0.f;
	const bool bWants = bOn && Train->mAtcData.TargetSpeed > State.Speed + HillWantMargin && !State.bFullBrake && State.Room >= HillStallRoom
		&& !(!State.BlockedReason.IsEmpty() && (bBraking || Pull <= State.FullTraction * HillStallShare));
	if (!bWants || State.Speed >= HillStandSpeed || State.StandSince < 0.0)
	{
		State.StandSince = Now;
		State.bStallReported = false;
	}
	// Help ends over the climb, or when switched off; not when a player takes over on the climb.
	const bool bCanHelp = mSettings.bHillHelp && Train && State.FullTraction > 0.f;
	if (State.HillBoost > 1.f && (!bCanHelp || (Pull < State.FullTraction * HillHelpEndShare && (State.Speed >= HillHelpSpeed || !bWants))))
	{
		UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s is over the climb at %.0f km/h (slope %.1f %%); its own traction again after %.0f s of hill help"), *TrainName(Train),
			State.Speed * 0.036f, State.Grade * 100.f, Now - State.HillSince);
		State.HillBoost = 1.f;
		return;
	}
	if (!bOn)
	{
		return;
	}
	// Help starts once it is slow and wants to go on, or stands though told to go on any slope; it is raised, never
	// lowered, while it lasts.
	const bool bStalled = Now - State.StandSince >= mSettings.HillStallSeconds;
	const float Needed = FMath::Min(FMath::Max(Pull * HillHelpHeadroom / State.FullTraction, bStalled && !bBraking ? State.HillBoost * HillStallStep : 0.f), mSettings.HillHelpMax);
	if (State.HillBoost == 1.f && !bStalled && !(bWants && State.Speed < HillHelpSpeed && Pull > State.FullTraction * HillStallShare))
	{
		return;
	}
	if (bStalled)
	{
		State.StandSince = Now;
	}
	if (Needed > State.HillBoost + 0.05f)
	{
		UE_LOG(LogAutoTrains, Log, TEXT("traffic: %s %s on a %.1f %% climb at %.0f km/h (slope pulls %.0f kN, its traction %.0f kN): hill help x%.2f traction%s%s"), *TrainName(Train),
			bStalled ? TEXT("stands though told to go") : State.HillBoost == 1.f ? TEXT("stalls") : TEXT("still stalls"), State.Grade * 100.f, State.Speed * 0.036f, Pull / 100000.f,
			State.FullTraction / 100000.f, Needed, bStalled ? TEXT("; ") : TEXT(""), bStalled ? *DescribeDrive(*Train) : TEXT(""));
		if (State.HillBoost == 1.f)
		{
			State.HillSince = Now;
		}
		State.HillBoost = FMath::Max(Needed, 1.f);
	}
	else if (bStalled && !State.bStallReported)
	{
		State.bStallReported = true;
		const FString Fault = FString::Printf(TEXT("%s %s stands at x%.2f traction though told to go (slope %.1f %%, last driven %.1f s ago): %s\n"), *FDateTime::Now().ToString(TEXT("%Y-%m-%d %H:%M:%S")),
			*TrainName(Train), State.HillBoost, State.Grade * 100.f, Now - State.DrivenAt, *DescribeDrive(*Train));
		UE_LOG(LogAutoTrains, Warning, TEXT("traffic: STALLED: %s"), *Fault.TrimEnd());
		WriteFault(Fault);
	}
}

float AATTrafficSubsystem::GetPlanningDeceleration(const FATTrafficTrain& State, float Speed)
{
	return FMath::Max(State.PlanDecel[GetBrakeBand(Speed >= 0.f ? Speed : State.Speed)] + GravityAcceleration * State.GradeAhead, MinPlanDeceleration);
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
		Distance += (FMath::Square(High) - FMath::Square(Low)) / (2.f * FMath::Max(State.PlanDecel[Band] + GravityAcceleration * State.GradeAhead, MinPlanDeceleration));
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
		const float Deceleration = FMath::Max(State.PlanDecel[Band] + GravityAcceleration * State.GradeAhead, MinPlanDeceleration) * Share;
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
	State->DrivenAt = GetWorld()->GetTimeSeconds();
	const float Distance = GetAuthorityDistance(Train);
	// Behind a leader on shared track: keep able to stop where the leader could stop.
	float HardFollowLimit = -1.f;
	FATFollowInfo FollowInfo;
	// Passing through a deadlock: the trains in its way don't hold it back.
	const bool bGhost = State->GhostUntil >= 0.0;
	const float FollowLimit = bGhost ? -1.f : GetFollowLimit(Train, *State, HardFollowLimit, &FollowInfo);
	if (Distance < 0.f && FollowLimit < 0.f)
	{
		State->Room = MAX_flt;
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
	// Forced braking: when its own brakes won't stop the train in time, take off the rest of the speed directly, at
	// the deceleration that stops it at the stop point, the way the game's block signals hold a train. Once started
	// it holds, with full brake, until the train's own brakes suffice: what it slowed by itself since the last tick
	// (its brakes and the slope) counts, else its planned braking.
	const float StopWithin = FMath::Min(Distance >= 0.f ? Usable : MAX_flt, HardFollowLimit >= 0.f ? HardFollowLimit - StopDistance : MAX_flt);
	State->Room = StopWithin;
	const float Needed = StopWithin < MAX_flt ? FMath::Square(Speed) / (2.f * FMath::Max(StopWithin, ForcedBrakeMinDistance)) : 0.f;
	const bool bOverCurve = Speed > AllowedHard + OverspeedTolerance;
	const bool bWasFullBrake = State->bFullBrake;
	const bool bWasForced = State->bForcedBrake;
	const float Own = bWasForced && DeltaSeconds > 0.f ? (State->ForcedSpeed - Speed) / DeltaSeconds : Deceleration;
	State->bForcedBrake = DeltaSeconds > 0.f && Speed > 0.f && Needed > Own && (bOverCurve || bWasForced);
	State->bFullBrake = bOverCurve || State->bForcedBrake;
	Atc.TargetSpeed = State->bFullBrake ? 0.f : FMath::Min(Atc.TargetSpeed, Allowed);
	if (State->bForcedBrake)
	{
		const float Velocity = Train->mSimulationData.Velocity;
		const float NewSpeed = FMath::Max(0.f, Speed - (Needed - Own) * DeltaSeconds);
		Train->mSimulationData.Velocity = Velocity < 0.f ? -NewSpeed : NewSpeed;
		State->ForcedSpeed = NewSpeed;
		State->bForcedSinceUpdate = true;
		if (!bWasForced)
		{
			UE_LOG(LogAutoTrains, Verbose, TEXT("traffic: %s forced braking at %.1f m/s2 (%.0f km/h, %.0f m to stop, its brakes %.1f m/s2)"), *TrainName(Train),
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
		// The game's grade weakens its braking when positive (downhill); ours is positive uphill.
		Atc.NextRestrictiveSignalGrade = -State->GradeAhead;
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
			Atc.NextRestrictiveSignalGrade = -State->GradeAhead;
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
		Lines.Add(FString::Printf(TEXT("%s: %.0f km/h (cruises %.0f, accelerates %.2f m/s2), %d claims, may drive %s, brakes at %.1f m/s2, waited %.0f s, rerouted %d times%s%s%s%s"),
			*TrainName(State.Train.Get()), State.Speed * 0.036f, State.CruiseSeen * 0.036f, State.PlanAccel[GetBrakeBand(State.Speed)] / 100.f, State.Claims.Num(),
			Distance < 0.f ? TEXT("to its stop") : *FString::Printf(TEXT("%.0f m"), FMath::Max(0.f, Distance - State.StopMargin) / 100.f),
			GetPlanningDeceleration(State) / 100.f, State.WaitingSeconds, State.Reroutes, State.BlockedReason.IsEmpty() ? TEXT("") : TEXT(", "), *State.BlockedReason,
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
