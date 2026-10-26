#include "ASModules.h"
#include "ASChatCommand.h"
#include "ASHooks.h"
#include "ASTestSignalSubsystem.h"

UASGameInstanceModule::UASGameInstanceModule()
{
	bRootModule = true;
}

void UASGameInstanceModule::DispatchLifecycleEvent(ELifecyclePhase Phase)
{
	if (Phase == ELifecyclePhase::INITIALIZATION)
	{
		FASHooks::InstallHooks();
	}
	Super::DispatchLifecycleEvent(Phase);
}

UASGameWorldModule::UASGameWorldModule()
{
	bRootModule = true;
	ModSubsystems.Add(AASTestSignalSubsystem::StaticClass());
	mChatCommands.Add(AASChatCommand::StaticClass());
}
