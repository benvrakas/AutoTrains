#include "AutoSignals.h"

DEFINE_LOG_CATEGORY(LogAutoSignals);

void FAutoSignalsModule::StartupModule()
{
	UE_LOG(LogAutoSignals, Log, TEXT("AutoSignals module starting"));
}

void FAutoSignalsModule::ShutdownModule()
{
}

IMPLEMENT_MODULE(FAutoSignalsModule, AutoSignals)
