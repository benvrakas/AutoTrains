#pragma once

#include "CoreMinimal.h"

class AFGRailroadSubsystem;
class AFGTrain;

// Installs AutoSignals' hooks and reads railroad internals for diagnostics. Friend of AFGRailroadSubsystem and
// AFGTrainScheduler.
class FASHooks
{
public:
	static void InstallHooks();

	// Returns one line per block booking the train scheduler holds for Train. Server only.
	static TArray<FString> DescribeBookings(AFGTrain* Train);

	// Returns one line per track graph and one per signal block in it.
	static TArray<FString> DescribeTrackGraphs(AFGRailroadSubsystem* Railroad);
};
