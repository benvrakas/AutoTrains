#include "AutoTrains.h"
#include "EngineUtils.h"
#include "FGTrain.h"

DEFINE_LOG_CATEGORY(LogAutoTrains);

namespace
{
	// How many trains carry each name, counted once per frame (on the game thread).
	TMap<FString, int32> GTrainNameCounts;
	uint64 GTrainNameFrame = MAX_uint64;
	TWeakObjectPtr<UWorld> GTrainNameWorld;
}

FString ATTrainName(const AFGTrain* Train)
{
	if (!Train)
	{
		return TEXT("?");
	}
	const FString Name = Train->GetTrainName().ToString();
	UWorld* World = Train->GetWorld();
	if (!World || !IsInGameThread())
	{
		return Name;
	}
	if (GTrainNameFrame != GFrameCounter || GTrainNameWorld.Get() != World)
	{
		GTrainNameCounts.Reset();
		GTrainNameFrame = GFrameCounter;
		GTrainNameWorld = World;
		for (TActorIterator<AFGTrain> It(World); It; ++It)
		{
			++GTrainNameCounts.FindOrAdd(It->GetTrainName().ToString());
		}
	}
	return GTrainNameCounts.FindRef(Name) > 1 ? FString::Printf(TEXT("%s #%s"), *Name, *Train->GetName().Right(4)) : Name;
}

void FAutoTrainsModule::StartupModule()
{
	UE_LOG(LogAutoTrains, Log, TEXT("Auto Trains module starting"));
}

void FAutoTrainsModule::ShutdownModule()
{
}

IMPLEMENT_MODULE(FAutoTrainsModule, AutoTrains)
