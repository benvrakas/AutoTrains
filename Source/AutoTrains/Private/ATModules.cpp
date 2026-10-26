#include "ATModules.h"
#include "ATChatCommand.h"
#include "ATControlSubsystem.h"
#include "ATHooks.h"
#include "ATTrafficSubsystem.h"

UATGameInstanceModule::UATGameInstanceModule()
{
	bRootModule = true;
}

void UATGameInstanceModule::DispatchLifecycleEvent(ELifecyclePhase Phase)
{
	if (Phase == ELifecyclePhase::INITIALIZATION)
	{
		FATHooks::InstallHooks();
	}
	Super::DispatchLifecycleEvent(Phase);
}

UATGameWorldModule::UATGameWorldModule()
{
	bRootModule = true;
	ModSubsystems.Add(AATControlSubsystem::StaticClass());
	ModSubsystems.Add(AATTrafficSubsystem::StaticClass());
	mChatCommands.Add(AATChatCommand::StaticClass());
}
