#pragma once

#include "CoreMinimal.h"

class UWidgetBlueprintHookData;

// Adds Auto Trains' section to the vanilla locomotive menu and starts the ride HUD (clients only; not in the editor).
namespace ATUIHooks
{
	void Register(UObject* Outer, TArray<UWidgetBlueprintHookData*>& OutHooks);
}
