#pragma once

#include "CoreMinimal.h"

// Installs Auto Trains' hook on the train AI: after AFGTrain::TickAtc the control subsystem adjusts the train's
// stop point and target speed.
class FATHooks
{
public:
	static void InstallHooks();
};
