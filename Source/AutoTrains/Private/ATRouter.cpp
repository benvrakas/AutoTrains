#include "ATRouter.h"
#include "Buildables/FGBuildableRailroadStation.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "FGLocomotive.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "FGTrain.h"

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
