#include "ATRouter.h"
#include "Buildables/FGBuildableRailroadStation.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "FGLocomotive.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "FGTrain.h"

namespace
{
	struct FOpenNode
	{
		UFGRailroadTrackConnectionComponent* Connection;
		float Cost;
	};

	bool operator<(const FOpenNode& A, const FOpenNode& B)
	{
		return A.Cost < B.Cost;
	}
}

TArray<UFGRailroadTrackConnectionComponent*> FATRouter::FindFastestRoute(UFGRailroadTrackConnectionComponent* Start, UFGRailroadTrackConnectionComponent* Goal,
	float StartSeconds, TFunctionRef<float(const AFGBuildableRailroadTrack*, const UFGRailroadTrackConnectionComponent*, float)> StepSeconds)
{
	TArray<UFGRailroadTrackConnectionComponent*> Route;
	if (!Start || !Goal)
	{
		return Route;
	}

	TMap<UFGRailroadTrackConnectionComponent*, float> BestCost;
	TMap<UFGRailroadTrackConnectionComponent*, UFGRailroadTrackConnectionComponent*> Previous;
	TArray<FOpenNode> Open;
	BestCost.Add(Start, StartSeconds);
	Open.HeapPush({Start, StartSeconds});
	while (Open.Num() > 0)
	{
		FOpenNode Node;
		Open.HeapPop(Node);
		if (Node.Cost > BestCost.FindChecked(Node.Connection))
		{
			continue;
		}
		if (Node.Connection == Goal)
		{
			break;
		}
		// The game's one-way rule: no passing a signal from behind.
		if (Node.Connection->GetTrailingSignal() && !Node.Connection->GetFacingSignal())
		{
			continue;
		}
		for (UFGRailroadTrackConnectionComponent* Entry : Node.Connection->GetConnections())
		{
			UFGRailroadTrackConnectionComponent* Next = Entry ? Entry->GetOpposite() : nullptr;
			const AFGBuildableRailroadTrack* Track = Entry ? Entry->GetTrack() : nullptr;
			if (!Next || !Track)
			{
				continue;
			}
			const float Cost = Node.Cost + StepSeconds(Track, Next, Node.Cost);
			const float* Known = BestCost.Find(Next);
			if (!Known || Cost < *Known)
			{
				BestCost.Add(Next, Cost);
				Previous.Add(Next, Node.Connection);
				Open.HeapPush({Next, Cost});
			}
		}
	}

	if (!BestCost.Contains(Goal))
	{
		return Route;
	}
	for (UFGRailroadTrackConnectionComponent* Connection = Goal; Connection; Connection = Previous.FindRef(Connection))
	{
		Route.Add(Connection);
		if (Connection == Start)
		{
			break;
		}
	}
	Algo::Reverse(Route);
	return Route;
}

float FATRouter::GetRouteLength(const TArray<UFGRailroadTrackConnectionComponent*>& Route)
{
	float Length = 0.f;
	for (int32 Index = 1; Index < Route.Num(); ++Index)
	{
		const AFGBuildableRailroadTrack* Track = Route[Index] ? Route[Index]->GetTrack() : nullptr;
		Length += Track ? Track->GetLength() : 0.f;
	}
	return Length;
}

FRailroadPathSharedPtr FATRouter::BuildPath(const TArray<UFGRailroadTrackConnectionComponent*>& Route, AFGBuildableRailroadStation* Station)
{
	FRailroadPathSharedPtr Path = MakeShared<FRailroadPath>();
	Path->Station = Station;
	Path->PathPoints.SetNum(Route.Num());
	float DistanceToEnd = 0.f;
	for (int32 Index = Route.Num() - 1; Index >= 0; --Index)
	{
		Path->PathPoints[Index].TrackConnection = Route[Index];
		Path->PathPoints[Index].Distance = DistanceToEnd;
		const AFGBuildableRailroadTrack* Track = Route[Index] ? Route[Index]->GetTrack() : nullptr;
		DistanceToEnd += Track ? Track->GetLength() : 0.f;
	}
	return Path;
}

bool FATRouter::ApplyPath(AFGTrain* Train, const FRailroadPathSharedPtr& Path, UFGRailroadTrackConnectionComponent* Goal)
{
	FRailroadPathFindingResult Result;
	Result.Locomotive = Train->GetMultipleUnitMaster();
	Result.Goal = Goal;
	Result.Path = Path;
	Result.Result = ERailroadPathFindingResult::RPFR_Success;
	return Train->mAtcData.SetPath(Result);
}
