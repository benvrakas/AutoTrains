#pragma once

#include "CoreMinimal.h"
#include "Module/GameInstanceModule.h"
#include "Module/GameWorldModule.h"
#include "ATModules.generated.h"

// Auto Trains' root game instance module; installs the hooks.
UCLASS()
class AUTOTRAINS_API UATGameInstanceModule : public UGameInstanceModule
{
	GENERATED_BODY()

public:
	UATGameInstanceModule();

	virtual void DispatchLifecycleEvent(ELifecyclePhase Phase) override;
};

// Auto Trains' root game world module; registers the control subsystem and the chat command.
UCLASS()
class AUTOTRAINS_API UATGameWorldModule : public UGameWorldModule
{
	GENERATED_BODY()

public:
	UATGameWorldModule();
};
