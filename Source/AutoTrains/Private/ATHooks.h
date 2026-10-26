#pragma once

#include "CoreMinimal.h"

// Installs Auto Trains' hooks on the train AI: after AFGTrain::TickAtc the control subsystem adjusts the train's
// target speed, and while a train's ATC refreshes its target points, signal stops can be cleared for that train.
class FATHooks
{
public:
	static void InstallHooks();
};
