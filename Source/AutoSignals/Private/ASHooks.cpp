#include "ASHooks.h"
#include "ASTestSignalSubsystem.h"
#include "AutoSignals.h"
#include "Buildables/FGBuildableRailroadSignal.h"
#include "FGRailroadSignalBlock.h"
#include "FGRailroadSubsystem.h"
#include "FGTrain.h"
#include "FGTrainScheduler.h"
#include "Patching/NativeHookManager.h"

namespace
{
	FString DescribeSignal(const AFGBuildableRailroadSignal* Signal)
	{
		if (!Signal)
		{
			return TEXT("none");
		}
		return FString::Printf(TEXT("%s%s"), *Signal->GetName(), AASTestSignalSubsystem::IsTestSignal(Signal) ? TEXT(" (test)") : TEXT(""));
	}
}

void FASHooks::InstallHooks()
{
#if WITH_EDITOR
	UE_LOG(LogAutoSignals, Log, TEXT("Editor build: hooks not installed"));
#else
	static bool bInstalled = false;
	if (bInstalled)
	{
		return;
	}
	bInstalled = true;

	AFGBuildableRailroadSignal* Sample = GetMutableDefault<AFGBuildableRailroadSignal>();
	SUBSCRIBE_METHOD_VIRTUAL(AFGBuildable::ShouldSave_Implementation, Sample, [](auto& Scope, const AFGBuildable* Self)
	{
		if (AASTestSignalSubsystem::IsTestSignal(Self))
		{
			Scope.Override(false);
		}
	});

	SUBSCRIBE_METHOD_AFTER(AFGRailroadSubsystem::AddSignal, [](AFGRailroadSubsystem* Self, AFGBuildableRailroadSignal* Signal)
	{
		const bool bTest = AASTestSignalSubsystem::IsTestSignal(Signal);
		UE_LOG(LogAutoSignals, Verbose, TEXT("AddSignal: %s"), *DescribeSignal(Signal));
		if (bTest)
		{
			UE_LOG(LogAutoSignals, Log, TEXT("Railroad subsystem registered %s"), *DescribeSignal(Signal));
		}
	});
	SUBSCRIBE_METHOD_AFTER(AFGRailroadSubsystem::RemoveSignal, [](AFGRailroadSubsystem* Self, AFGBuildableRailroadSignal* Signal)
	{
		UE_LOG(LogAutoSignals, Verbose, TEXT("RemoveSignal: %s"), *DescribeSignal(Signal));
	});
	SUBSCRIBE_METHOD_AFTER(AFGRailroadSubsystem::RebuildSignalBlocks, [](AFGRailroadSubsystem* Self, int32 GraphID)
	{
		const FTrackGraph* Graph = Self ? Self->mTrackGraphs.Find(GraphID) : nullptr;
		UE_LOG(LogAutoSignals, Log, TEXT("Signal blocks rebuilt for track graph %d: %d blocks"), GraphID, Graph ? Graph->SignalBlocks.Num() : -1);
	});

	UE_LOG(LogAutoSignals, Log, TEXT("Hooks installed"));
#endif
}

TArray<FString> FASHooks::DescribeBookings(AFGTrain* Train)
{
	TArray<FString> Lines;
	AFGRailroadSubsystem* Railroad = Train ? AFGRailroadSubsystem::Get(Train->GetWorld()) : nullptr;
	AFGTrainScheduler* Scheduler = Railroad ? Railroad->GetTrainScheduler() : nullptr;
	const TUniquePtr<FTrainSchedulerInfo>* Info = Scheduler ? Scheduler->mSchedulerInfos.Find(Train) : nullptr;
	if (!Info || !Info->IsValid())
	{
		Lines.Add(TEXT("not tracked by the train scheduler"));
		return Lines;
	}

	const FTrainSchedulerInfo& SchedulerInfo = **Info;
	Lines.Add(FString::Printf(TEXT("scheduler: dependency level %d, priority %d, needs approval %s, depends on %d trains, %d bookings"),
		SchedulerInfo.DependencyLevel, SchedulerInfo.Priority, SchedulerInfo.NeedApproval ? TEXT("yes") : TEXT("no"),
		SchedulerInfo.TrainDependencies.Num(), SchedulerInfo.BlockReservations.Num()));
	for (const TWeakPtr<FFGRailroadBlockReservation>& Weak : SchedulerInfo.BlockReservations)
	{
		const TSharedPtr<FFGRailroadBlockReservation> Reservation = Weak.Pin();
		if (!Reservation)
		{
			Lines.Add(TEXT("  expired booking"));
			continue;
		}
		const TSharedPtr<FFGRailroadSignalBlock> Block = Reservation->Block.Pin();
		const TCHAR* Status = Reservation->Status == ERailroadBlockReservationStatus::RBRS_Approved ? TEXT("approved")
			: Reservation->Status == ERailroadBlockReservationStatus::RBRS_Pending ? TEXT("pending") : TEXT("cancelled");
		Lines.Add(FString::Printf(TEXT("  block %d (%s, %s): %s booking %s, entry %s, exit %s, path segments %d..%d, %d tracks left"),
			Block ? Block->ID : -1,
			Block && Block->IsPathBlock() ? TEXT("path block") : TEXT("normal block"),
			Block && Block->IsOccupied() ? TEXT("occupied") : TEXT("free"),
			Reservation->Type == ERailroadBlockReservationType::RBRT_Path ? TEXT("path") : TEXT("exclusive"),
			Status,
			*DescribeSignal(Reservation->EntrySignal.Get()),
			*DescribeSignal(Reservation->ExitSignal.Get()),
			Reservation->EntryPathSegment, Reservation->ExitPathSegment, Reservation->RemainingTracks.Num()));
	}
	return Lines;
}

TArray<FString> FASHooks::DescribeTrackGraphs(AFGRailroadSubsystem* Railroad)
{
	TArray<FString> Lines;
	if (!Railroad)
	{
		Lines.Add(TEXT("no railroad subsystem"));
		return Lines;
	}
	const UEnum* ValidationEnum = StaticEnum<ERailroadBlockValidation>();
	for (const TPair<int32, FTrackGraph>& Pair : Railroad->mTrackGraphs)
	{
		Lines.Add(FString::Printf(TEXT("track graph %d: %d tracks, %d signal blocks"), Pair.Key, Pair.Value.Tracks.Num(), Pair.Value.SignalBlocks.Num()));
		for (const TSharedPtr<FFGRailroadSignalBlock>& Block : Pair.Value.SignalBlocks)
		{
			if (!Block)
			{
				continue;
			}
			Lines.Add(FString::Printf(TEXT("  block %d: %s, %s, %s"), Block->ID,
				Block->IsPathBlock() ? TEXT("path block") : TEXT("normal block"),
				Block->IsOccupied() ? TEXT("occupied") : TEXT("free"),
				ValidationEnum ? *ValidationEnum->GetNameStringByValue(static_cast<int64>(Block->GetBlockValidation())) : TEXT("?")));
		}
	}
	return Lines;
}
