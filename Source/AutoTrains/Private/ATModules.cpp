#include "ATModules.h"
#include "ATChatCommand.h"
#include "ATConfig.h"
#include "ATControlSubsystem.h"
#include "ATHooks.h"
#include "ATTrafficSubsystem.h"

UATGameInstanceModule::UATGameInstanceModule()
{
	bRootModule = true;
	ModConfigurations.Add(UATConfig::StaticClass());
}

void UATGameInstanceModule::DispatchLifecycleEvent(ELifecyclePhase Phase)
{
	// SML registers the configuration during initialization; its editor classes must be in place first.
	if (Phase == ELifecyclePhase::INITIALIZATION)
	{
		UATConfig::UseSMLEditorClasses();
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
