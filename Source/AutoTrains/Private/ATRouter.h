#pragma once

#include "CoreMinimal.h"
#include "RailroadNavigation.h"

class AFGBuildableRailroadStation;
class AFGTrain;
class UFGRailroadTrackConnectionComponent;

// Route finding over the track graph in the game's path format. A route is the chain of connections a train leaves
// each track through, from the connection ahead of the train to the goal; the next hop from a connection is the far
// end of each track attached to it.
class FATRouter
{
public:
	// Returns the fastest route from Start, reached StartSeconds from now, to Goal, both included, or an empty array;
	// signals only set the directions track may be used in, as in the game. StepSeconds (track, exit connection, seconds from now
	// the track is reached) gives the seconds to get over each track.
	static TArray<UFGRailroadTrackConnectionComponent*> FindFastestRoute(UFGRailroadTrackConnectionComponent* Start, UFGRailroadTrackConnectionComponent* Goal,
		float StartSeconds, TFunctionRef<float(const AFGBuildableRailroadTrack*, const UFGRailroadTrackConnectionComponent*, float)> StepSeconds);

	// Returns the summed track length of Route after its first connection. [cm]
	static float GetRouteLength(const TArray<UFGRailroadTrackConnectionComponent*>& Route);

	// Builds a game path from Route; each point's distance is the track length from it to the end of the route.
	static FRailroadPathSharedPtr BuildPath(const TArray<UFGRailroadTrackConnectionComponent*>& Route, AFGBuildableRailroadStation* Station);

	// Gives the train Path to Goal; returns what FTrainAtcData::SetPath returned.
	static bool ApplyPath(AFGTrain* Train, const FRailroadPathSharedPtr& Path, UFGRailroadTrackConnectionComponent* Goal);
};
