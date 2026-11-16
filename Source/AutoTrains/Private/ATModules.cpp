#include "ATModules.h"
#include "ATChatCommand.h"
#include "ATClientSettingsSubsystem.h"
#include "ATConfig.h"
#include "ATControlSubsystem.h"
#include "ATHooks.h"
#include "ATRemoteCallObject.h"
#include "ATTrafficSubsystem.h"

UATGameInstanceModule::UATGameInstanceModule()
{
	bRootModule = true;
	ModConfigurations.Add(UATConfig::StaticClass());
	RemoteCallObjects.Add(UATRemoteCallObject::StaticClass());
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
	ModSubsystems.Add(AATClientSettingsSubsystem::StaticClass());
	mChatCommands.Add(AATChatCommand::StaticClass());
}
