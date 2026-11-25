#include "UI/ATUIHooks.h"
#include "AutoTrains.h"
#include "Engine/Engine.h"
#include "Patching/WidgetBlueprintHookManager.h"
#include "UI/ATTrainPanel.h"

namespace
{
	const TCHAR* LocomotiveMenuClass = TEXT("/Game/FactoryGame/Buildable/Vehicle/Train/Locomotive/UI/Widget_Locomotive_Menu.Widget_Locomotive_Menu_C");
}

void ATUIHooks::Register(UObject* Outer, TArray<UWidgetBlueprintHookData*>& OutHooks)
{
#if WITH_EDITOR
	UE_LOG(LogAutoTrains, Log, TEXT("UI: editor build, no hooks"));
#else
	if (IsRunningDedicatedServer())
	{
		return;
	}
	UWidgetBlueprintHookManager* Manager = GEngine ? GEngine->GetEngineSubsystem<UWidgetBlueprintHookManager>() : nullptr;
	if (!Manager)
	{
		UE_LOG(LogAutoTrains, Error, TEXT("UI: SML widget hook manager missing; no locomotive menu section"));
		return;
	}
	UWidgetBlueprintHookData* Hook = NewObject<UWidgetBlueprintHookData>(Outer);
	Hook->WidgetClass = TSoftClassPtr<UUserWidget>(FSoftObjectPath(LocomotiveMenuClass));
	Hook->NewWidgetClass = UATTrainPanel::StaticClass();
	Hook->NewWidgetName = TEXT("AT_TrainPanel");
	// The Timetable column; at its end, under the vanilla buttons (and SmartTrains' section).
	Hook->ParentWidgetName = TEXT("VerticalBox_1");
	Hook->ParentWidgetType = EWidgetBlueprintHookParentType::Direct;
	if (!Hook->WidgetClass.LoadSynchronous() || !Hook->ResolveParentWidgetOnArchetype())
	{
		UE_LOG(LogAutoTrains, Error, TEXT("UI: locomotive menu or its Timetable column not found; no section"));
	}
	else
	{
		Manager->RegisterWidgetBlueprintHook(Hook);
		OutHooks.Add(Hook);
		UE_LOG(LogAutoTrains, Log, TEXT("UI: locomotive menu section added"));
	}
	UATRideHud::Install();
#endif
}
