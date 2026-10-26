#include "ATHooks.h"
#include "ATControlSubsystem.h"
#include "AutoTrains.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "FGTrain.h"
#include "Patching/NativeHookManager.h"

namespace
{
	// The train whose TickAtc is running; TickAtc and the ATC functions it calls run on the game thread.
	AFGTrain* GTickingTrain = nullptr;
}

void FATHooks::InstallHooks()
{
#if WITH_EDITOR
	UE_LOG(LogAutoTrains, Log, TEXT("Editor build: hooks not installed"));
#else
	static bool bInstalled = false;
	if (bInstalled)
	{
		return;
	}
	bInstalled = true;

	SUBSCRIBE_METHOD(AFGTrain::TickAtc, [](auto& Scope, AFGTrain* Self, float DeltaSeconds)
	{
		AATControlSubsystem* Control = Self && Self->HasAuthority() ? AATControlSubsystem::Get(Self) : nullptr;
		GTickingTrain = Self;
		Scope(Self, DeltaSeconds);
		GTickingTrain = nullptr;
		if (Control)
		{
			Control->AfterTickAtc(Self, DeltaSeconds);
		}
	});

	SUBSCRIBE_METHOD_AFTER(FTrainAtcData::RefreshTargetPointsDistanceAndAspect, [](const bool& bResult, FTrainAtcData* Self, float CurrentPathSegmentOffset)
	{
		if (!bResult || !GTickingTrain || &GTickingTrain->mAtcData != Self)
		{
			return;
		}
		const AATControlSubsystem* Control = AATControlSubsystem::Get(GTickingTrain);
		if (!Control || !Control->IsIgnoringSignals(GTickingTrain))
		{
			return;
		}
		for (FTrainAtcPoint& Point : Self->TargetPoints)
		{
			if (Point.SignalAspect == ERailroadSignalAspect::RSA_Stop && Point.TrackConnection && Point.TrackConnection->GetFacingSignal())
			{
				Point.SignalAspect = ERailroadSignalAspect::RSA_Clear;
			}
		}
	});

	UE_LOG(LogAutoTrains, Log, TEXT("Hooks installed"));
#endif
}
