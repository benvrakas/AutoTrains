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

TArray<UFGRailroadTrackConnectionComponent*> FATRouter::FindRoute(UFGRailroadTrackConnectionComponent* Start, UFGRailroadTrackConnectionComponent* Goal, bool bObeyOneWay)
{
	TArray<UFGRailroadTrackConnectionComponent*> Route;
	if (!Start || !Goal)
	{
		return Route;
	}

	TMap<UFGRailroadTrackConnectionComponent*, float> BestCost;
	TMap<UFGRailroadTrackConnectionComponent*, UFGRailroadTrackConnectionComponent*> Previous;
	TArray<FOpenNode> Open;
	BestCost.Add(Start, 0.f);
	Open.HeapPush({Start, 0.f});
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
		if (bObeyOneWay && !Node.Connection->GetFacingSignal() && Node.Connection->GetTrailingSignal())
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
			const float Cost = Node.Cost + Track->GetLength();
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

UFGRailroadTrackConnectionComponent* FATRouter::GetConnectionAhead(AFGTrain* Train)
{
	const AFGLocomotive* Master = Train ? Train->GetMultipleUnitMaster() : nullptr;
	return Master ? Master->GetTrackPosition().GetForwardConnection() : nullptr;
}

FString FATRouter::CompareWithCurrentPath(AFGTrain* Train, const FRailroadPathSharedPtr& OurPath)
{
	const FTrainAtcData& Atc = Train->mAtcData;
	if (!Atc.Path.IsValid() || !OurPath.IsValid() || OurPath->PathPoints.Num() == 0)
	{
		return TEXT("no path to compare");
	}
	const TArray<FRailroadPathPoint>& Theirs = Atc.Path->PathPoints;
	int32 Offset = INDEX_NONE;
	for (int32 Index = 0; Index < Theirs.Num(); ++Index)
	{
		if (Theirs[Index].TrackConnection.Get() == OurPath->PathPoints[0].TrackConnection.Get())
		{
			Offset = Index;
			break;
		}
	}
	if (Offset == INDEX_NONE)
	{
		return FString::Printf(TEXT("the connection ahead (%s) isn't on the game's path (%d points, current segment %d)"),
			*Describe(OurPath->PathPoints[0].TrackConnection.Get()), Theirs.Num(), Atc.CurrentPathSegment);
	}

	int32 Matching = 0;
	float MaxDistanceError = 0.f;
	while (Offset + Matching < Theirs.Num() && Matching < OurPath->PathPoints.Num()
		&& Theirs[Offset + Matching].TrackConnection.Get() == OurPath->PathPoints[Matching].TrackConnection.Get())
	{
		MaxDistanceError = FMath::Max(MaxDistanceError, FMath::Abs(Theirs[Offset + Matching].Distance - OurPath->PathPoints[Matching].Distance));
		++Matching;
	}
	const bool bSame = Matching == OurPath->PathPoints.Num() && Offset + Matching == Theirs.Num();
	return FString::Printf(TEXT("%s: game path has %d points (current segment %d, ahead connection at %d), ours %d, %d match in order, largest distance difference %.1f m, game distance at start %.1f m, ours %.1f m"),
		bSame ? TEXT("SAME ROUTE") : TEXT("DIFFERENT ROUTE"), Theirs.Num(), Atc.CurrentPathSegment, Offset, OurPath->PathPoints.Num(), Matching,
		MaxDistanceError / 100.f, Theirs[Offset].Distance / 100.f, OurPath->PathPoints[0].Distance / 100.f);
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

FString FATRouter::Describe(const UFGRailroadTrackConnectionComponent* Connection)
{
	return Connection ? FString::Printf(TEXT("%s.%s"), *GetNameSafe(Connection->GetOwner()), *Connection->GetName()) : TEXT("none");
}
