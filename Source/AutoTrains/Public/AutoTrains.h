#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

DECLARE_LOG_CATEGORY_EXTERN(LogAutoTrains, Log, All);

class AFGTrain;

// A train's name for the log. Trains sharing a name (every train nobody named is "Train") get the end of their actor's
// name, which the save keeps, so log lines tell them apart; SmartTrains names them the same way.
FString ATTrainName(const AFGTrain* Train);

// Module entry point for Auto Trains.
class FAutoTrainsModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
