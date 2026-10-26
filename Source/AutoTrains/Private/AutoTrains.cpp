#include "AutoTrains.h"

DEFINE_LOG_CATEGORY(LogAutoTrains);

void FAutoTrainsModule::StartupModule()
{
	UE_LOG(LogAutoTrains, Log, TEXT("Auto Trains module starting"));
}

void FAutoTrainsModule::ShutdownModule()
{
}

IMPLEMENT_MODULE(FAutoTrainsModule, AutoTrains)
