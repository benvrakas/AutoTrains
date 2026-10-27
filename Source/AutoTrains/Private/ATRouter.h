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
	// Returns the shortest route from Start to Goal, both included, or an empty array. With bObeyOneWay a connection
	// that has only a trailing signal can't be left, as in the game's pathfinding.
	static TArray<UFGRailroadTrackConnectionComponent*> FindRoute(UFGRailroadTrackConnectionComponent* Start, UFGRailroadTrackConnectionComponent* Goal, bool bObeyOneWay);

	// Returns the fastest route from Start, reached StartSeconds from now, to Goal, ignoring one-way signals: StepSeconds
	// (track, exit connection, seconds from now the track is reached) gives the seconds to get over each track.
	static TArray<UFGRailroadTrackConnectionComponent*> FindFastestRoute(UFGRailroadTrackConnectionComponent* Start, UFGRailroadTrackConnectionComponent* Goal,
		float StartSeconds, TFunctionRef<float(const AFGBuildableRailroadTrack*, const UFGRailroadTrackConnectionComponent*, float)> StepSeconds);

	// Returns the summed track length of Route after its first connection. [cm]
	static float GetRouteLength(const TArray<UFGRailroadTrackConnectionComponent*>& Route);

	// Builds a game path from Route; each point's distance is the track length from it to the end of the route.
	static FRailroadPathSharedPtr BuildPath(const TArray<UFGRailroadTrackConnectionComponent*>& Route, AFGBuildableRailroadStation* Station);

	// Returns the connection ahead of the train's lead locomotive, or nullptr.
	static UFGRailroadTrackConnectionComponent* GetConnectionAhead(AFGTrain* Train);

	// Compares OurPath, which starts at the connection ahead of the train, with the train's current path; returns a
	// log line.
	static FString CompareWithCurrentPath(AFGTrain* Train, const FRailroadPathSharedPtr& OurPath);

	// Gives the train Path to Goal; returns what FTrainAtcData::SetPath returned.
	static bool ApplyPath(AFGTrain* Train, const FRailroadPathSharedPtr& Path, UFGRailroadTrackConnectionComponent* Goal);

	// Returns "<track>.<connection>" for logs.
	static FString Describe(const UFGRailroadTrackConnectionComponent* Connection);

private:
	// Lowest-cost search from Start, at StartCost, to Goal; StepCost(track, exit connection, cost so far) is the cost of
	// each track. With bObeyOneWay a connection that has only a trailing signal can't be left.
	static TArray<UFGRailroadTrackConnectionComponent*> Search(UFGRailroadTrackConnectionComponent* Start, UFGRailroadTrackConnectionComponent* Goal, bool bObeyOneWay,
		float StartCost, TFunctionRef<float(const AFGBuildableRailroadTrack*, const UFGRailroadTrackConnectionComponent*, float)> StepCost);
};
