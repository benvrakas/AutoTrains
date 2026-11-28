// What the UI shows of Auto Trains' traffic (Docs/UI.md).
#include "ATTrafficSubsystem.h"
#include "ATChatCommand.h"
#include "ATRemoteCallObject.h"
#include "AutoTrains.h"
#include "Buildables/FGBuildableRailroadStation.h"
#include "EngineUtils.h"
#include "FGPlayerController.h"
#include "FGTrain.h"
#include "FGTrainStationIdentifier.h"

namespace
{
	// A train standing this long with a plan wait counts as waiting on the monitor. [s]
	constexpr float MonitorWaitSeconds = 120.f;
	// Most rows the monitor lists.
	constexpr int32 MonitorRows = 40;
	// A wait reason is cut to this length for the UI; the log has it in full.
	constexpr int32 ReasonLength = 110;
	constexpr float ViewKmhPerCmPerSecond = 0.036f;

	FString StationNameOf(const AFGBuildableRailroadStation* Station)
	{
		const AFGTrainStationIdentifier* Identifier = Station ? Station->GetStationIdentifier() : nullptr;
		return Identifier ? Identifier->GetStationName().ToString() : FString();
	}

	FString Shorten(const FString& Reason)
	{
		return Reason.Len() <= ReasonLength ? Reason : Reason.Left(ReasonLength - 3) + TEXT("...");
	}
}

FATTrainView AATTrafficSubsystem::MakeTrainView(AFGTrain* Train) const
{
	FATTrainView View;
	View.Train = Train;
	View.bTraffic = mEnabled;
	const FATTrafficTrain* State = Train ? mTrains.Find(Train) : nullptr;
	const UWorld* World = GetWorld();
	const double Now = World ? World->GetTimeSeconds() : 0.0;
	if (!State || !Train || !Train->IsSelfDrivingEnabled())
	{
		return View;
	}
	View.Speed = State->Speed * ViewKmhPerCmPerSecond;
	View.TargetSpeed = Train->mAtcData.TargetSpeed * ViewKmhPerCmPerSecond;
	const bool bPlanStuck = State->PlanFailedSince >= 0.0 && Now - State->PlanFailedSince >= 60.0;
	View.State = Now < State->GhostUntil ? 5 : Train->IsDocked() ? 3 : (State->bInDeadlock || bPlanStuck) ? 4 : State->WaitingSeconds > 2.f ? 2 : State->bManaged ? 1 : 0;
	const AFGBuildableRailroadStation* Goal = State->bManaged ? State->GoalStation.Get() : State->PredictedStation.Get();
	View.Goal = StationNameOf(Goal);
	const double Arrival = State->bManaged ? State->PlanArrival : State->PredictedArrival;
	View.ArrivalSeconds = Arrival >= 0.0 && Arrival < 1.0e8 ? static_cast<float>(FMath::Max(0.0, Arrival - Now)) : -1.f;
	const AFGTrain* WaitFor = State->BlockedBy.IsValid() ? State->BlockedBy.Get() : State->PlanWaitFor.Get();
	if (WaitFor && State->WaitingSeconds > 0.f)
	{
		View.WaitingFor = WaitFor->GetTrainName().ToString();
	}
	View.WaitedSeconds = State->WaitingSeconds;
	View.Reason = State->WaitingSeconds > 0.f ? Shorten(State->BlockedReason) : FString();
	return View;
}

FATMonitorView AATTrafficSubsystem::MakeMonitorView() const
{
	FATMonitorView View;
	View.bTraffic = mEnabled;
	View.PassThroughs = mPassThroughs;
	View.CycleMs = static_cast<float>(mLastCycleMs);
	View.BudgetMs = mSettings.UpdateIntervalSeconds * 1000.f;
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const AFGTrain* Train = Pair.Value.Train.Get();
		if (!Train || !Train->IsSelfDrivingEnabled())
		{
			continue;
		}
		++View.Trains;
		View.Docked += Train->IsDocked() ? 1 : 0;
		View.Driving += !Train->IsDocked() && Pair.Value.Speed > 50.f ? 1 : 0;
		View.Waiting += !Train->IsDocked() && Pair.Value.WaitingSeconds > 2.f ? 1 : 0;
	}
	TArray<AFGTrain*> Trains;
	TArray<FString> Kinds;
	TArray<float> Seconds;
	const_cast<AATTrafficSubsystem*>(this)->ExternalGetTroubledTrains(Trains, Kinds, Seconds);
	TSet<const AFGTrain*> Listed;
	for (int32 Index = 0; Index < Trains.Num() && View.Rows.Num() < MonitorRows; ++Index)
	{
		const FATTrafficTrain* State = mTrains.Find(Trains[Index]);
		FATMonitorRow& Row = View.Rows.AddDefaulted_GetRef();
		Row.Train = Trains[Index];
		Row.Name = Trains[Index]->GetTrainName().ToString();
		Row.Kind = Kinds[Index];
		Row.Seconds = Seconds[Index];
		Row.Reason = State ? Shorten(State->BlockedReason) : FString();
		Listed.Add(Trains[Index]);
	}
	View.Troubled = View.Rows.Num();
	// Long waits that aren't troubles (yet): the longest first.
	TArray<const FATTrafficTrain*> Long;
	for (const TPair<TWeakObjectPtr<AFGTrain>, FATTrafficTrain>& Pair : mTrains)
	{
		const AFGTrain* Train = Pair.Value.Train.Get();
		if (Train && !Listed.Contains(Train) && Pair.Value.bManaged && Pair.Value.WaitingSeconds >= MonitorWaitSeconds)
		{
			Long.Add(&Pair.Value);
		}
	}
	Long.Sort([](const FATTrafficTrain& A, const FATTrafficTrain& B) { return A.WaitingSeconds > B.WaitingSeconds; });
	for (const FATTrafficTrain* State : Long)
	{
		if (View.Rows.Num() >= MonitorRows)
		{
			break;
		}
		FATMonitorRow& Row = View.Rows.AddDefaulted_GetRef();
		Row.Train = State->Train.Get();
		Row.Name = Row.Train->GetTrainName().ToString();
		Row.Kind = TEXT("Waiting");
		Row.Seconds = State->WaitingSeconds;
		Row.Reason = Shorten(State->BlockedReason);
	}
	return View;
}

FString AATTrafficSubsystem::MarkForPlayer(AFGTrain* Train, APlayerController* Player)
{
	const FATTrafficTrain* State = Train ? mTrains.Find(Train) : nullptr;
	AFGTrain* Target = State && State->BlockedBy.IsValid() ? State->BlockedBy.Get() : State && State->PlanWaitFor.IsValid() ? State->PlanWaitFor.Get() : Train;
	const AFGRailroadVehicle* Vehicle = Target ? Target->GetFirstVehicle() : nullptr;
	if (!Vehicle || !Player)
	{
		return TEXT("Nothing to mark.");
	}
	return AATChatCommand::MarkLocation(GetWorld(), Player, Vehicle->GetActorLocation(), Target->GetTrainName().ToString());
}
