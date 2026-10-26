#include "ASTestSignalSubsystem.h"
#include "ASHooks.h"
#include "AutoSignals.h"
#include "Buildables/FGBuildableRailroadSignal.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "EngineUtils.h"
#include "FGBuildableSubsystem.h"
#include "FGRailroadSubsystem.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "FGTrain.h"
#include "Subsystem/SubsystemActorManager.h"

const FName AASTestSignalSubsystem::SignalTag(TEXT("AutoSignals.TestSignal"));

namespace
{
	const TCHAR* BlockSignalClassPath = TEXT("/Game/FactoryGame/Buildable/Factory/Train/Signal/Build_RailroadBlockSignal.Build_RailroadBlockSignal_C");
	const TCHAR* PathSignalClassPath = TEXT("/Game/FactoryGame/Buildable/Factory/Train/Signal/Build_RailroadPathSignal.Build_RailroadPathSignal_C");

	template <typename TEnum>
	FString EnumName(TEnum Value)
	{
		const UEnum* Enum = StaticEnum<TEnum>();
		return Enum ? Enum->GetNameStringByValue(static_cast<int64>(Value)) : FString::FromInt(static_cast<int32>(Value));
	}

	// Writes Connections into the signal's connection array property Name, as a save load would.
	bool SetConnectionArray(AFGBuildableRailroadSignal* Signal, FName Name, const TArray<UFGRailroadTrackConnectionComponent*>& Connections)
	{
		FArrayProperty* Property = FindFProperty<FArrayProperty>(AFGBuildableRailroadSignal::StaticClass(), Name);
		if (!Property)
		{
			UE_LOG(LogAutoSignals, Error, TEXT("Signal property %s not found"), *Name.ToString());
			return false;
		}
		TArray<TObjectPtr<UFGRailroadTrackConnectionComponent>>* Array =
			Property->ContainerPtrToValuePtr<TArray<TObjectPtr<UFGRailroadTrackConnectionComponent>>>(Signal);
		Array->Reset();
		for (UFGRailroadTrackConnectionComponent* Connection : Connections)
		{
			Array->Add(Connection);
		}
		return true;
	}

	FString DescribeConnection(const UFGRailroadTrackConnectionComponent* Connection)
	{
		return Connection ? FString::Printf(TEXT("%s.%s"), *GetNameSafe(Connection->GetOwner()), *Connection->GetName()) : TEXT("none");
	}
}

AASTestSignalSubsystem::AASTestSignalSubsystem()
{
	ReplicationPolicy = ESubsystemReplicationPolicy::SpawnOnServer;
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
}

AASTestSignalSubsystem* AASTestSignalSubsystem::Get(const UObject* WorldContext)
{
	UWorld* World = WorldContext ? WorldContext->GetWorld() : nullptr;
	USubsystemActorManager* Manager = World ? World->GetSubsystem<USubsystemActorManager>() : nullptr;
	return Manager ? Manager->GetSubsystemActor<AASTestSignalSubsystem>() : nullptr;
}

bool AASTestSignalSubsystem::IsTestSignal(const AActor* Actor)
{
	return Actor && Actor->Tags.Contains(SignalTag);
}

void AASTestSignalSubsystem::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!mWatching)
	{
		return;
	}
	mWatchTimer -= DeltaSeconds;
	if (mWatchTimer <= 0.f)
	{
		mWatchTimer = 1.f;
		LogChanges();
	}
}

FString AASTestSignalSubsystem::PlaceSignals(const FVector& Location, const FVector& Direction, bool bPathSignal, bool bBothWays)
{
	UFGRailroadTrackConnectionComponent* Nearest = FindNearestConnection(Location, 5000.f);
	if (!Nearest)
	{
		return TEXT("No track joint within 50 m.");
	}

	// Trains moving along Direction leave their track through the near side of the joint.
	TArray<UFGRailroadTrackConnectionComponent*> NearSide;
	TArray<UFGRailroadTrackConnectionComponent*> FarSide;
	if (FVector::DotProduct(Nearest->GetConnectorNormal(), Direction) >= 0.f)
	{
		NearSide.Add(Nearest);
		FarSide = Nearest->GetConnections();
	}
	else
	{
		NearSide = Nearest->GetConnections();
		FarSide.Add(Nearest);
	}
	if (NearSide.Num() == 0 || FarSide.Num() == 0)
	{
		return TEXT("That joint is the end of a track. Stand near a joint between two track pieces.");
	}

	FString Message;
	FString Error;
	if (AFGBuildableRailroadSignal* Signal = SpawnSignal(NearSide, bPathSignal, Error))
	{
		Message = FString::Printf(TEXT("Placed %s."), *DescribeSignal(Signal));
	}
	else
	{
		Message = FString::Printf(TEXT("Not placed: %s"), *Error);
	}
	if (bBothWays)
	{
		Error.Reset();
		if (AFGBuildableRailroadSignal* Opposite = SpawnSignal(FarSide, bPathSignal, Error))
		{
			Message += FString::Printf(TEXT(" Placed %s."), *DescribeSignal(Opposite));
		}
		else
		{
			Message += FString::Printf(TEXT(" Opposite not placed: %s"), *Error);
		}
	}
	return Message;
}

int32 AASTestSignalSubsystem::ClearSignals()
{
	int32 Count = 0;
	for (const TWeakObjectPtr<AFGBuildableRailroadSignal>& Signal : mSignals)
	{
		if (Signal.IsValid())
		{
			Signal->Destroy();
			++Count;
		}
	}
	mSignals.Reset();
	mLastAspects.Reset();
	UE_LOG(LogAutoSignals, Log, TEXT("Removed %d test signals"), Count);
	return Count;
}

void AASTestSignalSubsystem::SetSignalsHidden(bool bHide)
{
	mHidden = bHide;
	int32 Count = 0;
	for (const TWeakObjectPtr<AFGBuildableRailroadSignal>& Signal : mSignals)
	{
		if (Signal.IsValid())
		{
			Signal->SetActorHiddenInGame(bHide);
			Signal->SetActorEnableCollision(!bHide);
			++Count;
		}
	}
	UE_LOG(LogAutoSignals, Log, TEXT("%s %d test signals"), bHide ? TEXT("Hid") : TEXT("Showed"), Count);
}

void AASTestSignalSubsystem::SetWatching(bool bWatch)
{
	mWatching = bWatch;
	mWatchTimer = 0.f;
	mLastTrainKeys.Reset();
	mLastAspects.Reset();
	UE_LOG(LogAutoSignals, Log, TEXT("Watching %s"), bWatch ? TEXT("on") : TEXT("off"));
}

TArray<FString> AASTestSignalSubsystem::LogState()
{
	TArray<FString> Chat;
	AFGRailroadSubsystem* Railroad = AFGRailroadSubsystem::Get(GetWorld());

	UE_LOG(LogAutoSignals, Log, TEXT("==== /sigtest info ===="));
	int32 SignalCount = 0;
	for (const TWeakObjectPtr<AFGBuildableRailroadSignal>& Signal : mSignals)
	{
		if (Signal.IsValid())
		{
			const FString Line = DescribeSignal(Signal.Get());
			UE_LOG(LogAutoSignals, Log, TEXT("%s"), *Line);
			Chat.Add(Line);
			++SignalCount;
		}
	}
	UE_LOG(LogAutoSignals, Log, TEXT("%d test signals"), SignalCount);

	for (const FString& Line : FASHooks::DescribeTrackGraphs(Railroad))
	{
		UE_LOG(LogAutoSignals, Log, TEXT("%s"), *Line);
	}

	TArray<AFGTrain*> Trains;
	if (Railroad)
	{
		Railroad->GetAllTrains(Trains);
	}
	for (AFGTrain* Train : Trains)
	{
		FString Key;
		const FString Line = DescribeTrain(Train, Key);
		UE_LOG(LogAutoSignals, Log, TEXT("%s"), *Line);
		Chat.Add(Line);
		for (const FString& Booking : FASHooks::DescribeBookings(Train))
		{
			UE_LOG(LogAutoSignals, Log, TEXT("  %s"), *Booking);
		}
	}
	Chat.Add(FString::Printf(TEXT("%d test signals, %d trains. Details in FactoryGame.log (LogAutoSignals)."), SignalCount, Trains.Num()));
	return Chat;
}

UFGRailroadTrackConnectionComponent* AASTestSignalSubsystem::FindNearestConnection(const FVector& Location, float MaxDistance) const
{
	UFGRailroadTrackConnectionComponent* Best = nullptr;
	float BestDistanceSquared = MaxDistance * MaxDistance;
	for (TActorIterator<AFGBuildableRailroadTrack> It(GetWorld()); It; ++It)
	{
		for (int32 Index = 0; Index < 2; ++Index)
		{
			UFGRailroadTrackConnectionComponent* Connection = It->GetConnection(Index);
			if (!Connection)
			{
				continue;
			}
			const float DistanceSquared = FVector::DistSquared(Connection->GetConnectorLocation(), Location);
			if (DistanceSquared < BestDistanceSquared)
			{
				BestDistanceSquared = DistanceSquared;
				Best = Connection;
			}
		}
	}
	return Best;
}

AFGBuildableRailroadSignal* AASTestSignalSubsystem::SpawnSignal(const TArray<UFGRailroadTrackConnectionComponent*>& NearSide, bool bPathSignal, FString& OutError)
{
	// Guarded connections are on the near side of the joint, observed ones on the far side; widen both until every
	// track meeting at the joint is covered (merges have several guarded, facing switches several observed).
	TArray<UFGRailroadTrackConnectionComponent*> Guarded = NearSide;
	TArray<UFGRailroadTrackConnectionComponent*> Observed;
	for (int32 Pass = 0; Pass < 8; ++Pass)
	{
		const int32 Before = Guarded.Num() + Observed.Num();
		for (UFGRailroadTrackConnectionComponent* Connection : Guarded)
		{
			for (UFGRailroadTrackConnectionComponent* Other : Connection->GetConnections())
			{
				if (Other)
				{
					Observed.AddUnique(Other);
				}
			}
		}
		for (UFGRailroadTrackConnectionComponent* Connection : Observed)
		{
			for (UFGRailroadTrackConnectionComponent* Other : Connection->GetConnections())
			{
				if (Other)
				{
					Guarded.AddUnique(Other);
				}
			}
		}
		if (Guarded.Num() + Observed.Num() == Before)
		{
			break;
		}
	}
	for (const UFGRailroadTrackConnectionComponent* Connection : Guarded)
	{
		if (Connection->GetFacingSignal())
		{
			OutError = FString::Printf(TEXT("%s already has a signal for that direction"), *DescribeConnection(Connection));
			return nullptr;
		}
	}
	for (const UFGRailroadTrackConnectionComponent* Connection : Observed)
	{
		if (Connection->GetTrailingSignal())
		{
			OutError = FString::Printf(TEXT("%s already has a signal behind it"), *DescribeConnection(Connection));
			return nullptr;
		}
	}

	UClass* Class = LoadClass<AFGBuildableRailroadSignal>(nullptr, bPathSignal ? PathSignalClassPath : BlockSignalClassPath);
	AFGBuildableSubsystem* Buildables = AFGBuildableSubsystem::Get(GetWorld());
	if (!Class || !Buildables)
	{
		OutError = TEXT("signal class or buildable subsystem missing");
		UE_LOG(LogAutoSignals, Error, TEXT("Spawn failed: class %s, buildable subsystem %s"), *GetNameSafe(Class), *GetNameSafe(Buildables));
		return nullptr;
	}

	const FVector TravelDirection = Guarded[0]->GetConnectorNormal();
	const FTransform Transform(TravelDirection.Rotation(), Guarded[0]->GetConnectorLocation());
	AFGBuildableRailroadSignal* Signal = Cast<AFGBuildableRailroadSignal>(Buildables->BeginSpawnBuildable(Class, Transform));
	if (!Signal)
	{
		OutError = TEXT("spawn failed");
		UE_LOG(LogAutoSignals, Error, TEXT("BeginSpawnBuildable returned no signal for %s"), *Class->GetName());
		return nullptr;
	}
	if (!SetConnectionArray(Signal, TEXT("mGuardedConnections"), Guarded) || !SetConnectionArray(Signal, TEXT("mObservedConnections"), Observed))
	{
		Signal->Destroy();
		OutError = TEXT("signal connection properties missing");
		return nullptr;
	}
	Signal->Tags.Add(SignalTag);
	Signal->FinishSpawning(Transform);
	if (mHidden)
	{
		Signal->SetActorHiddenInGame(true);
		Signal->SetActorEnableCollision(false);
	}
	mSignals.Add(Signal);

	bool bHooked = true;
	for (const UFGRailroadTrackConnectionComponent* Connection : Guarded)
	{
		bHooked &= Connection->GetFacingSignal() == Signal;
	}
	for (const UFGRailroadTrackConnectionComponent* Connection : Observed)
	{
		bHooked &= Connection->GetTrailingSignal() == Signal;
	}
	UE_LOG(LogAutoSignals, Log, TEXT("Spawned %s %s at %s: guards %d (%s), observes %d (%s), valid connections %s, attached to track %s"),
		bPathSignal ? TEXT("path signal") : TEXT("block signal"), *Signal->GetName(), *Transform.GetLocation().ToCompactString(),
		Guarded.Num(), *DescribeConnection(Guarded[0]), Observed.Num(), *DescribeConnection(Observed.Num() > 0 ? Observed[0] : nullptr),
		Signal->HasValidConnections() ? TEXT("yes") : TEXT("no"), bHooked ? TEXT("yes") : TEXT("NO"));
	return Signal;
}

FString AASTestSignalSubsystem::DescribeSignal(AFGBuildableRailroadSignal* Signal)
{
	if (!Signal)
	{
		return TEXT("none");
	}
	const TSharedPtr<FFGRailroadSignalBlock> Block = Signal->GetObservedBlock().Pin();
	return FString::Printf(TEXT("%s %s: %s, block %s, %s%s"),
		Signal->IsPathSignal() ? TEXT("path signal") : TEXT("block signal"), *Signal->GetName(),
		*EnumName(Signal->GetAspect()),
		Block ? *FString::Printf(TEXT("%d (%s%s)"), Block->ID, Block->IsPathBlock() ? TEXT("path") : TEXT("normal"), Block->IsOccupied() ? TEXT(", occupied") : TEXT("")) : TEXT("none yet"),
		*EnumName(Signal->GetBlockValidation()),
		Signal->IsBiDirectional() ? TEXT(", two-way") : TEXT(""));
}

FString AASTestSignalSubsystem::DescribeTrain(AFGTrain* Train, FString& OutKey)
{
	if (!Train)
	{
		return TEXT("none");
	}
	const FTrainAtcData& Atc = Train->mAtcData;
	const UFGRailroadTrackConnectionComponent* NextSignalConnection = Atc.NextSignalConnection.Get();
	const AFGBuildableRailroadSignal* NextSignal = NextSignalConnection ? NextSignalConnection->GetFacingSignal() : nullptr;
	const float SpeedKmh = FMath::Abs(Train->mSimulationData.Velocity) * 0.036f;
	OutKey = FString::Printf(TEXT("%s|%d|%s|%s|%s|%s|%s|%s"),
		*EnumName(Train->GetTrainStatus()), Train->IsSelfDrivingEnabled() ? 1 : 0, *GetNameSafe(NextSignal),
		*EnumName(Atc.NextSignalAspect), *EnumName(Atc.NextRestrictiveSignalAspect),
		*EnumName(Train->GetSelfDrivingError()), *EnumName(Train->GetDockingState()), SpeedKmh < 1.f ? TEXT("stopped") : TEXT("moving"));
	return FString::Printf(TEXT("train %s: %s%s, %.0f km/h, next signal %s %s at %.0f m, restrictive %s at %.0f m, waited %.0f s, error %s, docking %s"),
		*Train->GetTrainName().ToString(), *EnumName(Train->GetTrainStatus()), Train->IsSelfDrivingEnabled() ? TEXT(" (self-driving)") : TEXT(""),
		SpeedKmh, *GetNameSafe(NextSignal), *EnumName(Atc.NextSignalAspect), Atc.NextSignalDistance / 100.f,
		*EnumName(Atc.NextRestrictiveSignalAspect), Atc.NextRestrictiveSignalDistance / 100.f,
		Train->mSelfDrivingData.TimeWaitingAtSignal, *EnumName(Train->GetSelfDrivingError()), *EnumName(Train->GetDockingState()));
}

void AASTestSignalSubsystem::LogChanges()
{
	for (const TWeakObjectPtr<AFGBuildableRailroadSignal>& Signal : mSignals)
	{
		if (!Signal.IsValid())
		{
			continue;
		}
		const uint8 Aspect = static_cast<uint8>(Signal->GetAspect());
		const uint8* Last = mLastAspects.Find(Signal);
		if (!Last || *Last != Aspect)
		{
			mLastAspects.Add(Signal, Aspect);
			UE_LOG(LogAutoSignals, Log, TEXT("watch: %s"), *DescribeSignal(Signal.Get()));
		}
	}

	TArray<AFGTrain*> Trains;
	if (AFGRailroadSubsystem* Railroad = AFGRailroadSubsystem::Get(GetWorld()))
	{
		Railroad->GetAllTrains(Trains);
	}
	for (AFGTrain* Train : Trains)
	{
		FString Key;
		const FString Line = DescribeTrain(Train, Key);
		const FString* Last = mLastTrainKeys.Find(Train);
		if (!Last || *Last != Key)
		{
			mLastTrainKeys.Add(Train, Key);
			UE_LOG(LogAutoSignals, Log, TEXT("watch: %s"), *Line);
			for (const FString& Booking : FASHooks::DescribeBookings(Train))
			{
				UE_LOG(LogAutoSignals, Log, TEXT("watch:   %s"), *Booking);
			}
		}
	}
}
