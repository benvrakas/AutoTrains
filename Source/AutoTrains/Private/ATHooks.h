#pragma once

#include "CoreMinimal.h"

// Installs Auto Trains' hooks on the train AI and physics: after AFGTrain::TickAtc the control subsystem adjusts the
// train's stop point and target speed, and train-against-train sweeps report no hit while trains should pass
// through each other. Friend of AFGRailroadSubsystem.
class FATHooks
{
public:
	static void InstallHooks();
};
