#include "ATControlSubsystem.h"
#include "ATRouter.h"
#include "AutoTrains.h"
#include "Buildables/FGBuildableRailroadSignal.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "EngineUtils.h"
#include "FGDismantleInterface.h"
#include "FGLocomotive.h"
#include "FGRailroadSubsystem.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "FGRailroadVehicle.h"
#include "FGTrain.h"
#include "HAL/PlatformTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Subsystem/SubsystemActorManager.h"

namespace
{
	// How far short of the stop point the braking curve ends. [cm]
	constexpr float StopMargin = 100.f;
	// Share of the consist's brake deceleration the stop curve plans with.
	constexpr float StopBrakeShare = 0.7f;
	constexpr float CmPerSecondToKmh = 0.036f;
	constexpr float TelemetryInterval = 0.2f;

	template <typename TEnum>
	FString EnumName(TEnum Value)
	{
		const UEnum* Enum = StaticEnum<TEnum>();
		return Enum ? Enum->GetNameStringByValue(static_cast<int64>(Value)) : FString::FromInt(static_cast<int32>(Value));
	}

	const TCHAR* ModeName(EATCommandMode Mode)
	{
		switch (Mode)
		{
		case EATCommandMode::SpeedCap: return TEXT("speed");
		case EATCommandMode::StopAt: return TEXT("stopat");
		default: return TEXT("none");
		}
	}

	// Returns the remaining track distance along the train's path to its goal, or -1. [cm]
	float GetDistanceToGoal(const AFGTrain* Train)
	{
		const FTrainAtcData& Atc = Train->mAtcData;
		if (!Atc.Path.IsValid() || !Atc.Path->PathPoints.IsValidIndex(Atc.CurrentPathSegment))
		{
			return -1.f;
		}
		return Atc.Path->PathPoints[Atc.CurrentPathSegment].Distance + Atc.CurrentConnectionDistance;
	}

	FString DescribePosition(const AFGRailroadVehicle* Vehicle)
	{
		if (!Vehicle)
		{
			return TEXT(",");
		}
		const FRailroadTrackPosition& Position = Vehicle->GetTrackPosition();
		return FString::Printf(TEXT("%s,%.2f"), *GetNameSafe(Position.Track.Get()), Position.Offset / 100.f);
	}
}

AATControlSubsystem::AATControlSubsystem()
{
	ReplicationPolicy = ESubsystemReplicationPolicy::SpawnOnServer;
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
}

AATControlSubsystem* AATControlSubsystem::Get(const UObject* WorldContext)
{
	UWorld* World = WorldContext ? WorldContext->GetWorld() : nullptr;
	USubsystemActorManager* Manager = World ? World->GetSubsystem<USubsystemActorManager>() : nullptr;
	return Manager ? Manager->GetSubsystemActor<AATControlSubsystem>() : nullptr;
}

void AATControlSubsystem::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!mLogging)
	{
		return;
	}
	mLogTime += DeltaSeconds;
	mSampleTimer -= DeltaSeconds;
	if (mSampleTimer <= 0.f)
	{
		mSampleTimer += TelemetryInterval;
		SampleTelemetry(TelemetryInterval);
	}
	mFlushTimer -= DeltaSeconds;
	if (mFlushTimer <= 0.f)
	{
		mFlushTimer = 2.f;
		FlushTelemetry();
	}
	mHookReportTimer -= DeltaSeconds;
	if (mHookReportTimer <= 0.f)
	{
		mHookReportTimer = 10.f;
		UE_LOG(LogAutoTrains, Log, TEXT("Control hook: %d calls in 10 s, %.2f us each, %.3f ms per second"),
			mHookCalls, mHookCalls > 0 ? mHookSeconds * 1e6 / mHookCalls : 0.0, mHookSeconds * 100.0);
		mHookSeconds = 0.0;
		mHookCalls = 0;
	}
}

void AATControlSubsystem::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	FlushTelemetry();
	Super::EndPlay(EndPlayReason);
}

void AATControlSubsystem::AfterTickAtc(AFGTrain* Train, float DeltaSeconds)
{
	const double Start = FPlatformTime::Seconds();
	FATTrainState& State = mStates.FindOrAdd(Train);
	FTrainAtcData& Atc = Train->mAtcData;
	const float Speed = FMath::Abs(Train->mSimulationData.Velocity);
	State.Odometer += Speed * DeltaSeconds;
	State.GameTargetSpeed = Atc.TargetSpeed;

	if (State.bIgnoreSignals && Train->IsSelfDrivingEnabled())
	{
		IgnoreSignalStops(Train);
	}
	if (Train->IsSelfDrivingEnabled() && State.Mode != EATCommandMode::None)
	{
		float Allowed = Atc.TargetSpeed;
		if (State.Mode == EATCommandMode::SpeedCap)
		{
			Allowed = FMath::Min(Allowed, State.CapSpeed);
		}
		else
		{
			const float Remaining = State.StopOdometer - State.Odometer;
			const float Deceleration = FMath::Max(Train->mConsistData.MaxBrakeDeceleration * StopBrakeShare, 10.f);
			const float CurveSpeed = Remaining > StopMargin ? FMath::Sqrt(2.f * Deceleration * (Remaining - StopMargin)) : 0.f;
			Allowed = FMath::Min(Allowed, CurveSpeed);
			if (!State.bStopReported && Speed < 5.f && CurveSpeed <= 0.f)
			{
				State.bStopReported = true;
				UE_LOG(LogAutoTrains, Log, TEXT("%s stopped for stopat: %.1f m %s the stop point"), *Train->GetTrainName().ToString(),
					FMath::Abs(Remaining) / 100.f, Remaining >= 0.f ? TEXT("short of") : TEXT("past"));
			}
		}
		Atc.TargetSpeed = Allowed;
	}
	State.FinalTargetSpeed = Atc.TargetSpeed;

	if (State.AppliedPath.IsValid() && !State.bPathReplacedReported && Atc.Path != State.AppliedPath.Pin())
	{
		State.bPathReplacedReported = true;
		UE_LOG(LogAutoTrains, Log, TEXT("%s: the path this mod set was replaced (now %d points, current segment %d)"), *Train->GetTrainName().ToString(),
			Atc.Path.IsValid() ? Atc.Path->PathPoints.Num() : 0, Atc.CurrentPathSegment);
	}

	mHookSeconds += FPlatformTime::Seconds() - Start;
	++mHookCalls;
}

void AATControlSubsystem::IgnoreSignalStops(AFGTrain* Train)
{
	FTrainAtcData& Atc = Train->mAtcData;
	for (FTrainAtcPoint& Point : Atc.TargetPoints)
	{
		if (Point.SignalAspect == ERailroadSignalAspect::RSA_Stop && Point.TrackConnection && Point.TrackConnection->GetFacingSignal())
		{
			Point.SignalAspect = ERailroadSignalAspect::RSA_Clear;
		}
	}
	if (Atc.NextRestrictiveSignalAspect != ERailroadSignalAspect::RSA_Stop)
	{
		return;
	}

	// Pick the restrictive point as FTrainAtcData::UpdateNextRestrictiveSignal does, now that signal stops are clear.
	const FTrainAtcPoint* Restrictive = nullptr;
	for (const FTrainAtcPoint& Point : Atc.TargetPoints)
	{
		if (!Point.TrackConnection || Point.Distance >= Train->mConsistData.MaxBrakeDistance)
		{
			break;
		}
		if (Point.SignalAspect == ERailroadSignalAspect::RSA_Stop || Point.SignalAspect == ERailroadSignalAspect::RSA_Dock)
		{
			Restrictive = &Point;
			break;
		}
	}
	Atc.NextRestrictiveSignalConnection = Restrictive ? Restrictive->TrackConnection.Get() : nullptr;
	Atc.NextRestrictiveSignalSpeed = 0.f;
	Atc.NextRestrictiveSignalDistance = Restrictive ? Restrictive->Distance : 0.f;
	Atc.NextRestrictiveSignalGrade = Restrictive ? Restrictive->AverageGrade : 0.f;
	Atc.NextRestrictiveSignalAspect = Restrictive ? Restrictive->SignalAspect : ERailroadSignalAspect::RSA_None;
	Atc.NextRestrictiveSignalStopOffset = !Restrictive ? 0.f
		: Restrictive->SignalAspect == ERailroadSignalAspect::RSA_Dock ? TrainConstants::DOCK_STOP_OFFSET : TrainConstants::SIGNAL_STOP_OFFSET;

	// Target speed as AFGTrain::TickAtc computes it from the restrictive point.
	float Target = Train->mConsistData.MaxSpeed;
	if (Restrictive)
	{
		const float Distance = Atc.NextRestrictiveSignalDistance;
		if (Distance > TrainConstants::RESTRICTED_SPEED_DISTANCE)
		{
			Atc.NextRestrictiveSignalSpeed = TrainConstants::RESTRICTED_SPEED;
			const float Mass = FMath::Max(Train->mConsistData.Mass, 1.f);
			const float Deceleration = FMath::Max(0.f, Train->mConsistData.MaxBrakeDeceleration - Train->mSimulationData.GravitationalForce * Atc.NextRestrictiveSignalGrade / Mass);
			Target = FMath::Min(Target, FMath::Sqrt(Deceleration * 2.f * (Distance - TrainConstants::RESTRICTED_SPEED_DISTANCE) + FMath::Square(TrainConstants::RESTRICTED_SPEED)));
		}
		else
		{
			Target = Distance >= Atc.NextRestrictiveSignalStopOffset + 1e-4f ? TrainConstants::RESTRICTED_SPEED : 0.f;
		}
	}
	const AFGLocomotive* Master = Train->GetMultipleUnitMaster();
	Atc.TargetSpeed = Master && Master->HasPower() ? Target : 0.f;
}

bool AATControlSubsystem::IsIgnoringSignals(const AFGTrain* Train) const
{
	const FATTrainState* State = mStates.Find(Train);
	return State && State->bIgnoreSignals;
}

AFGTrain* AATControlSubsystem::FindTrain(const FString& Token) const
{
	const TArray<AFGTrain*> Trains = GetSortedTrains();
	if (Token.IsNumeric())
	{
		const int32 Index = FCString::Atoi(*Token) - 1;
		return Trains.IsValidIndex(Index) ? Trains[Index] : nullptr;
	}
	for (AFGTrain* Train : Trains)
	{
		if (Train->GetTrainName().ToString().Contains(Token, ESearchCase::IgnoreCase))
		{
			return Train;
		}
	}
	return nullptr;
}

TArray<FString> AATControlSubsystem::ListTrains() const
{
	TArray<FString> Lines;
	const TArray<AFGTrain*> Trains = GetSortedTrains();
	for (int32 Index = 0; Index < Trains.Num(); ++Index)
	{
		AFGTrain* Train = Trains[Index];
		const FATTrainState* State = mStates.Find(Train);
		const float DistanceToGoal = GetDistanceToGoal(Train);
		Lines.Add(FString::Printf(TEXT("%d. %s: %s, %.0f km/h, %.0f m long, command %s%s, %s to goal"), Index + 1,
			*Train->GetTrainName().ToString(), Train->IsSelfDrivingEnabled() ? TEXT("self-driving") : *EnumName(Train->GetTrainStatus()),
			FMath::Abs(Train->mSimulationData.Velocity) * CmPerSecondToKmh, Train->mConsistData.Length / 100.f,
			State ? ModeName(State->Mode) : TEXT("none"), State && State->bIgnoreSignals ? TEXT(", ignoring signals") : TEXT(""),
			DistanceToGoal >= 0.f ? *FString::Printf(TEXT("%.0f m"), DistanceToGoal / 100.f) : TEXT("no path")));
	}
	if (Lines.Num() == 0)
	{
		Lines.Add(TEXT("No trains."));
	}
	for (const FString& Line : Lines)
	{
		UE_LOG(LogAutoTrains, Log, TEXT("list: %s"), *Line);
	}
	return Lines;
}

FString AATControlSubsystem::SetSpeedCap(AFGTrain* Train, float Kmh)
{
	FATTrainState& State = mStates.FindOrAdd(Train);
	State.Mode = EATCommandMode::SpeedCap;
	State.CapSpeed = FMath::Max(0.f, Kmh) / CmPerSecondToKmh;
	const FString Message = FString::Printf(TEXT("%s: speed capped at %.0f km/h."), *Train->GetTrainName().ToString(), Kmh);
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
	return Message;
}

FString AATControlSubsystem::SetStopAhead(AFGTrain* Train, float Metres)
{
	FATTrainState& State = mStates.FindOrAdd(Train);
	const float Speed = FMath::Abs(Train->mSimulationData.Velocity);
	const float Deceleration = FMath::Max(Train->mConsistData.MaxBrakeDeceleration * StopBrakeShare, 10.f);
	const float BrakingDistance = Speed * Speed / (2.f * Deceleration) + StopMargin;
	State.Mode = EATCommandMode::StopAt;
	State.StopOdometer = State.Odometer + Metres * 100.f;
	State.bStopReported = false;
	const FString Message = FString::Printf(TEXT("%s: stopping %.0f m ahead (at %.0f km/h it needs about %.0f m)."), *Train->GetTrainName().ToString(),
		Metres, Speed * CmPerSecondToKmh, BrakingDistance / 100.f);
	UE_LOG(LogAutoTrains, Log, TEXT("%s Consist: %.0f m, %.0f t, brake deceleration %.2f m/s2, brake distance %.0f m"), *Message,
		Train->mConsistData.Length / 100.f, Train->mConsistData.Mass / 1000.f, Train->mConsistData.MaxBrakeDeceleration / 100.f,
		Train->mConsistData.MaxBrakeDistance / 100.f);
	return Message;
}

FString AATControlSubsystem::Release(AFGTrain* Train)
{
	FATTrainState& State = mStates.FindOrAdd(Train);
	State.Mode = EATCommandMode::None;
	const FString Message = FString::Printf(TEXT("%s: released."), *Train->GetTrainName().ToString());
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
	return Message;
}

FString AATControlSubsystem::ReleaseAll()
{
	int32 Count = 0;
	for (TPair<TWeakObjectPtr<AFGTrain>, FATTrainState>& Pair : mStates)
	{
		Count += Pair.Value.Mode != EATCommandMode::None || Pair.Value.bIgnoreSignals ? 1 : 0;
		Pair.Value.Mode = EATCommandMode::None;
		Pair.Value.bIgnoreSignals = false;
	}
	const FString Message = FString::Printf(TEXT("Released %d trains; all obey signals again."), Count);
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
	return Message;
}

FString AATControlSubsystem::SetIgnoreSignals(AFGTrain* Train, bool bIgnore)
{
	mStates.FindOrAdd(Train).bIgnoreSignals = bIgnore;
	const FString Message = FString::Printf(TEXT("%s: %s signals."), *Train->GetTrainName().ToString(), bIgnore ? TEXT("ignoring") : TEXT("obeying"));
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
	return Message;
}

FString AATControlSubsystem::CheckRoute(AFGTrain* Train)
{
	const FTrainAtcData& Atc = Train->mAtcData;
	UFGRailroadTrackConnectionComponent* Start = FATRouter::GetConnectionAhead(Train);
	UFGRailroadTrackConnectionComponent* Goal = Atc.Path.IsValid() && Atc.Path->PathPoints.Num() > 0 ? Atc.Path->PathPoints.Last().TrackConnection.Get() : nullptr;
	if (!Start || !Goal)
	{
		const FString Message = FString::Printf(TEXT("%s: needs a path (self-driving, heading to a stop). Ahead %s, goal %s."), *Train->GetTrainName().ToString(),
			*FATRouter::Describe(Start), *FATRouter::Describe(Goal));
		UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
		return Message;
	}

	const double Begin = FPlatformTime::Seconds();
	const TArray<UFGRailroadTrackConnectionComponent*> Route = FATRouter::FindRoute(Start, Goal, !IsIgnoringSignals(Train));
	const double Milliseconds = (FPlatformTime::Seconds() - Begin) * 1000.0;
	if (Route.Num() == 0)
	{
		const FString Message = FString::Printf(TEXT("%s: this mod's router found no route from %s to %s (%.2f ms)."), *Train->GetTrainName().ToString(),
			*FATRouter::Describe(Start), *FATRouter::Describe(Goal), Milliseconds);
		UE_LOG(LogAutoTrains, Warning, TEXT("%s"), *Message);
		return Message;
	}
	const FRailroadPathSharedPtr Ours = FATRouter::BuildPath(Route, Atc.Path->Station.Get());
	const FString Message = FString::Printf(TEXT("%s: %s (%.2f ms)"), *Train->GetTrainName().ToString(), *FATRouter::CompareWithCurrentPath(Train, Ours), Milliseconds);
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
	for (int32 Index = 0; Index < FMath::Min(Route.Num(), 8); ++Index)
	{
		const int32 TheirIndex = Atc.CurrentPathSegment + Index;
		UE_LOG(LogAutoTrains, Log, TEXT("  %d: ours %s %.1f m | game %s %.1f m"), Index, *FATRouter::Describe(Route[Index]), Ours->PathPoints[Index].Distance / 100.f,
			Atc.Path->PathPoints.IsValidIndex(TheirIndex) ? *FATRouter::Describe(Atc.Path->PathPoints[TheirIndex].TrackConnection.Get()) : TEXT("-"),
			Atc.Path->PathPoints.IsValidIndex(TheirIndex) ? Atc.Path->PathPoints[TheirIndex].Distance / 100.f : 0.f);
	}
	return Message;
}

FString AATControlSubsystem::RouteVia(AFGTrain* Train, const FVector& Location)
{
	const FTrainAtcData& Atc = Train->mAtcData;
	UFGRailroadTrackConnectionComponent* Start = FATRouter::GetConnectionAhead(Train);
	UFGRailroadTrackConnectionComponent* Goal = Atc.Path.IsValid() && Atc.Path->PathPoints.Num() > 0 ? Atc.Path->PathPoints.Last().TrackConnection.Get() : nullptr;
	if (!Start || !Goal)
	{
		return FString::Printf(TEXT("%s: needs a path (self-driving, heading to a stop)."), *Train->GetTrainName().ToString());
	}

	AFGBuildableRailroadTrack* Nearest = nullptr;
	float NearestDistanceSquared = FMath::Square(5000.f);
	for (TActorIterator<AFGBuildableRailroadTrack> It(GetWorld()); It; ++It)
	{
		for (int32 Index = 0; Index < 2; ++Index)
		{
			const UFGRailroadTrackConnectionComponent* Connection = It->GetConnection(Index);
			const float DistanceSquared = Connection ? FVector::DistSquared(Connection->GetConnectorLocation(), Location) : MAX_flt;
			if (DistanceSquared < NearestDistanceSquared)
			{
				NearestDistanceSquared = DistanceSquared;
				Nearest = *It;
			}
		}
	}
	if (!Nearest)
	{
		return TEXT("No track within 50 m.");
	}

	const bool bObeyOneWay = !IsIgnoringSignals(Train);
	TArray<UFGRailroadTrackConnectionComponent*> Best;
	for (int32 Index = 0; Index < 2; ++Index)
	{
		UFGRailroadTrackConnectionComponent* Via = Nearest->GetConnection(Index);
		TArray<UFGRailroadTrackConnectionComponent*> First = FATRouter::FindRoute(Start, Via, bObeyOneWay);
		const TArray<UFGRailroadTrackConnectionComponent*> Second = FATRouter::FindRoute(Via, Goal, bObeyOneWay);
		if (First.Num() == 0 || Second.Num() == 0)
		{
			continue;
		}
		First.Append(&Second[1], Second.Num() - 1);
		if (Best.Num() == 0 || FATRouter::GetRouteLength(First) < FATRouter::GetRouteLength(Best))
		{
			Best = MoveTemp(First);
		}
	}
	if (Best.Num() == 0)
	{
		const FString Message = FString::Printf(TEXT("%s: no route to its stop passes %s."), *Train->GetTrainName().ToString(), *GetNameSafe(Nearest));
		UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
		return Message;
	}

	TSet<UFGRailroadTrackConnectionComponent*> Seen;
	int32 Repeats = 0;
	for (UFGRailroadTrackConnectionComponent* Connection : Best)
	{
		bool bAlreadySeen = false;
		Seen.Add(Connection, &bAlreadySeen);
		Repeats += bAlreadySeen ? 1 : 0;
	}
	const FRailroadPathSharedPtr Path = FATRouter::BuildPath(Best, Atc.Path->Station.Get());
	const bool bApplied = FATRouter::ApplyPath(Train, Path, Goal);
	FATTrainState& State = mStates.FindOrAdd(Train);
	State.AppliedPath = Path;
	State.bPathReplacedReported = false;
	const FString Message = FString::Printf(TEXT("%s: route via %s, %d points, %.0f m, %d repeated connections; applied: %s, now at segment %d."),
		*Train->GetTrainName().ToString(), *GetNameSafe(Nearest), Best.Num(), FATRouter::GetRouteLength(Best) / 100.f, Repeats,
		bApplied ? TEXT("yes") : TEXT("NO"), Train->mAtcData.CurrentPathSegment);
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
	return Message;
}

FString AATControlSubsystem::SetLogging(bool bLog)
{
	if (bLog == mLogging)
	{
		return bLog ? FString::Printf(TEXT("Already logging to %s"), *mTelemetryPath) : TEXT("Not logging.");
	}
	mLogging = bLog;
	if (!bLog)
	{
		FlushTelemetry();
		UE_LOG(LogAutoTrains, Log, TEXT("Telemetry stopped: %s"), *mTelemetryPath);
		return FString::Printf(TEXT("Telemetry saved to %s"), *mTelemetryPath);
	}

	mTelemetryPath = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("AutoTrains") / FString::Printf(TEXT("telemetry-%s.csv"), *FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S"))));
	mLogTime = 0.f;
	mSampleTimer = 0.f;
	mFlushTimer = 2.f;
	mHookReportTimer = 10.f;
	mHookSeconds = 0.0;
	mHookCalls = 0;
	mPendingRows.Reset();
	for (AFGTrain* Train : GetSortedTrains())
	{
		const FTrainConsist& Consist = Train->mConsistData;
		mPendingRows.Add(FString::Printf(TEXT("# consist,%s,length_m=%.1f,mass_t=%.1f,max_speed_kmh=%.0f,tractive_kN=%.1f,air_brake_kN=%.1f,dynamic_brake_kN=%.1f,brake_decel_ms2=%.3f,brake_distance_m=%.0f"),
			*Train->GetTrainName().ToString(), Consist.Length / 100.f, Consist.Mass / 1000.f, Consist.MaxSpeed * CmPerSecondToKmh,
			Consist.MaxTractiveEffort / 100000.f, Consist.MaxAirBrakingEffort / 100000.f, Consist.MaxDynamicBrakingEffort / 100000.f,
			Consist.MaxBrakeDeceleration / 100.f, Consist.MaxBrakeDistance / 100.f));
	}
	mPendingRows.Add(TEXT("time_s,train,self_driving,status,command,ignore_signals,speed_kmh,game_target_kmh,final_target_kmh,accel_ms2,odometer_m,stop_remaining_m,goal_distance_m,restrictive_aspect,restrictive_distance_m,next_signal_aspect,next_signal_distance_m,front_track,front_offset_m,rear_track,rear_offset_m,docking,error"));
	FlushTelemetry();
	UE_LOG(LogAutoTrains, Log, TEXT("Telemetry started: %s"), *mTelemetryPath);
	return FString::Printf(TEXT("Logging telemetry to %s"), *mTelemetryPath);
}

FString AATControlSubsystem::SetAutopilot(AFGTrain* Train, bool bEnabled)
{
	Train->SetSelfDrivingEnabled(bEnabled);
	const FString Message = FString::Printf(TEXT("%s: autopilot %s."), *Train->GetTrainName().ToString(), Train->IsSelfDrivingEnabled() ? TEXT("on") : TEXT("off"));
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
	return Message;
}

FString AATControlSubsystem::SetAutopilotAll(bool bEnabled)
{
	int32 Changed = 0;
	for (AFGTrain* Train : GetSortedTrains())
	{
		if (Train->IsSelfDrivingEnabled() != bEnabled)
		{
			Train->SetSelfDrivingEnabled(bEnabled);
			Changed += Train->IsSelfDrivingEnabled() == bEnabled ? 1 : 0;
		}
	}
	const FString Message = FString::Printf(TEXT("Autopilot turned %s on %d trains."), bEnabled ? TEXT("on") : TEXT("off"), Changed);
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);
	return Message;
}

FString AATControlSubsystem::RemoveAllSignals()
{
	TArray<AFGBuildableRailroadSignal*> Signals;
	for (TActorIterator<AFGBuildableRailroadSignal> It(GetWorld()); It; ++It)
	{
		if (IsValid(*It))
		{
			Signals.Add(*It);
		}
	}
	int32 PathSignals = 0;
	int32 Forced = 0;
	for (AFGBuildableRailroadSignal* Signal : Signals)
	{
		PathSignals += Signal->IsPathSignal() ? 1 : 0;
		IFGDismantleInterface::Execute_Dismantle(Signal);
		// Dismantle outside the build gun can leave the actor in place (seen in 0.2.2); destroy what's left.
		if (IsValid(Signal) && !Signal->IsActorBeingDestroyed())
		{
			Signal->Destroy();
			++Forced;
		}
	}
	const int32 Remaining = CountSignals();
	const FString Message = FString::Printf(TEXT("Removed %d signals (%d path, %d block; %d needed a forced destroy); %d left."), Signals.Num(), PathSignals,
		Signals.Num() - PathSignals, Forced, Remaining);
	UE_LOG(LogAutoTrains, Log, TEXT("%s"), *Message);

	FTimerHandle Handle;
	GetWorldTimerManager().SetTimer(Handle, FTimerDelegate::CreateWeakLambda(this, [this]()
	{
		UE_LOG(LogAutoTrains, Log, TEXT("Signals in the world one second after removal: %d"), CountSignals());
	}), 1.f, false);
	return Message;
}

int32 AATControlSubsystem::CountSignals() const
{
	int32 Count = 0;
	for (TActorIterator<AFGBuildableRailroadSignal> It(GetWorld()); It; ++It)
	{
		Count += IsValid(*It) && !It->IsActorBeingDestroyed() ? 1 : 0;
	}
	return Count;
}

TArray<AFGTrain*> AATControlSubsystem::GetSortedTrains() const
{
	TArray<AFGTrain*> Trains;
	if (AFGRailroadSubsystem* Railroad = AFGRailroadSubsystem::Get(GetWorld()))
	{
		Railroad->GetAllTrains(Trains);
	}
	Trains.RemoveAll([](const AFGTrain* Train) { return !IsValid(Train); });
	Trains.Sort([](const AFGTrain& A, const AFGTrain& B) { return A.GetTrainName().ToString() < B.GetTrainName().ToString(); });
	return Trains;
}

void AATControlSubsystem::SampleTelemetry(float SampleSeconds)
{
	for (AFGTrain* Train : GetSortedTrains())
	{
		FATTrainState& State = mStates.FindOrAdd(Train);
		const FTrainAtcData& Atc = Train->mAtcData;
		const float Speed = FMath::Abs(Train->mSimulationData.Velocity);
		const float Acceleration = (Speed - State.LastSampleSpeed) / SampleSeconds / 100.f;
		State.LastSampleSpeed = Speed;
		const float GoalDistance = GetDistanceToGoal(Train);
		mPendingRows.Add(FString::Printf(TEXT("%.2f,%s,%d,%s,%s,%d,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.1f,%s,%.1f,%s,%.1f,%s,%s,%s,%s"),
			mLogTime, *Train->GetTrainName().ToString(), Train->IsSelfDrivingEnabled() ? 1 : 0, *EnumName(Train->GetTrainStatus()),
			ModeName(State.Mode), State.bIgnoreSignals ? 1 : 0, Speed * CmPerSecondToKmh, State.GameTargetSpeed * CmPerSecondToKmh,
			State.FinalTargetSpeed * CmPerSecondToKmh, Acceleration, State.Odometer / 100.f,
			State.Mode == EATCommandMode::StopAt ? (State.StopOdometer - State.Odometer) / 100.f : 0.f,
			GoalDistance >= 0.f ? GoalDistance / 100.f : -1.f,
			*EnumName(Atc.NextRestrictiveSignalAspect), Atc.NextRestrictiveSignalDistance / 100.f,
			*EnumName(Atc.NextSignalAspect), Atc.NextSignalDistance / 100.f,
			*DescribePosition(Train->GetFirstVehicle()), *DescribePosition(Train->GetLastVehicle()),
			*EnumName(Train->GetDockingState()), *EnumName(Train->GetSelfDrivingError())));
	}
}

void AATControlSubsystem::FlushTelemetry()
{
	if (mPendingRows.Num() == 0 || mTelemetryPath.IsEmpty())
	{
		return;
	}
	const FString Text = FString::Join(mPendingRows, TEXT("\n")) + TEXT("\n");
	if (!FFileHelper::SaveStringToFile(Text, *mTelemetryPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append))
	{
		UE_LOG(LogAutoTrains, Error, TEXT("Could not write telemetry to %s"), *mTelemetryPath);
	}
	mPendingRows.Reset();
}
