#pragma once

#include "CoreMinimal.h"
#include "Module/GameInstanceModule.h"
#include "Module/GameWorldModule.h"
#include "ATModules.generated.h"

// Auto Trains' root game instance module; installs the hooks and registers the configuration and remote call object.
UCLASS()
class AUTOTRAINS_API UATGameInstanceModule : public UGameInstanceModule
{
	GENERATED_BODY()

public:
	UATGameInstanceModule();

	virtual void DispatchLifecycleEvent(ELifecyclePhase Phase) override;
};

// Auto Trains' root game world module; registers the subsystems and the chat command.
UCLASS()
class AUTOTRAINS_API UATGameWorldModule : public UGameWorldModule
{
	GENERATED_BODY()

public:
	UATGameWorldModule();
};
