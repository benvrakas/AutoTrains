#pragma once

#include "CoreMinimal.h"
#include "RailroadNavigation.h"

class AFGBuildableRailroadStation;
class AFGTrain;
class UFGRailroadTrackConnectionComponent;

// Routes in the game's path format. A route is the chain of connections a train leaves each track through, from the
// connection ahead of the train to the goal; the route search itself is in ATRouteSearch.
class FATRouter
{
public:
	// Returns the summed track length of Route after its first connection. [cm]
	static float GetRouteLength(const TArray<UFGRailroadTrackConnectionComponent*>& Route);

	// Builds a game path from Route; each point's distance is the track length from it to the end of the route.
	static FRailroadPathSharedPtr BuildPath(const TArray<UFGRailroadTrackConnectionComponent*>& Route, AFGBuildableRailroadStation* Station);

	// Gives the train Path to Goal; returns what FTrainAtcData::SetPath returned.
	static bool ApplyPath(AFGTrain* Train, const FRailroadPathSharedPtr& Path, UFGRailroadTrackConnectionComponent* Goal);
};
