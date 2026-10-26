#include "ATModules.h"
#include "ATChatCommand.h"
#include "ATControlSubsystem.h"
#include "ATHooks.h"

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
	mChatCommands.Add(AATChatCommand::StaticClass());
}
