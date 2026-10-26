#include "ATHooks.h"
#include "ATControlSubsystem.h"
#include "AutoTrains.h"
#include "FGTrain.h"
#include "Patching/NativeHookManager.h"

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

	SUBSCRIBE_METHOD_AFTER(AFGTrain::TickAtc, [](AFGTrain* Self, float DeltaSeconds)
	{
		if (!Self || !Self->HasAuthority())
		{
			return;
		}
		if (AATControlSubsystem* Control = AATControlSubsystem::Get(Self))
		{
			Control->AfterTickAtc(Self, DeltaSeconds);
		}
	});

	UE_LOG(LogAutoTrains, Log, TEXT("Hooks installed"));
#endif
}
