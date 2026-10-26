#pragma once

#include "CoreMinimal.h"
#include "Module/GameInstanceModule.h"
#include "Module/GameWorldModule.h"
#include "ASModules.generated.h"

// AutoSignals' root game instance module; installs the hooks.
UCLASS()
class AUTOSIGNALS_API UASGameInstanceModule : public UGameInstanceModule
{
	GENERATED_BODY()

public:
	UASGameInstanceModule();

	virtual void DispatchLifecycleEvent(ELifecyclePhase Phase) override;
};

// AutoSignals' root game world module; registers the test signal subsystem and the chat command.
UCLASS()
class AUTOSIGNALS_API UASGameWorldModule : public UGameWorldModule
{
	GENERATED_BODY()

public:
	UASGameWorldModule();
};
