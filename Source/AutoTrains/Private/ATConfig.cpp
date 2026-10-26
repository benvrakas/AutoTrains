#include "ATConfig.h"
#include "AutoTrains.h"
#include "Configuration/ConfigManager.h"
#include "Configuration/Properties/ConfigPropertyBool.h"
#include "Configuration/Properties/ConfigPropertyFloat.h"
#include "Configuration/Properties/WidgetExtension/CP_Float.h"
#include "Configuration/Properties/ConfigPropertySection.h"
#include "Configuration/Properties/WidgetExtension/CP_Section.h"
#include "Engine/GameInstance.h"
#include "Engine/World.h"

#define LOCTEXT_NAMESPACE "AutoTrains"

const FString UATConfig::TrafficControlKey = TEXT("TrafficControl");
const FString UATConfig::ReroutingKey = TEXT("Rerouting");
const FString UATConfig::PhaseThroughKey = TEXT("PhaseThrough");
const FString UATConfig::AggressiveKey = TEXT("AggressiveTiming");
const FString UATConfig::SafetyMarginKey = TEXT("SafetyMarginSeconds");

namespace
{
	FConfigId MakeConfigId()
	{
		FConfigId Id;
		Id.ModReference = TEXT("AutoTrains");
		return Id;
	}

	const UConfigPropertySection* FindRoot(const UObject* WorldContext)
	{
		const UWorld* World = WorldContext ? WorldContext->GetWorld() : nullptr;
		const UGameInstance* GameInstance = World ? World->GetGameInstance() : nullptr;
		const UConfigManager* Manager = GameInstance ? GameInstance->GetSubsystem<UConfigManager>() : nullptr;
		return Manager ? Manager->GetConfigurationRootSection(MakeConfigId()) : nullptr;
	}

	bool ReadBool(const UConfigPropertySection* Root, const FString& Key, bool bDefault)
	{
		const TObjectPtr<UConfigProperty>* Found = Root ? Root->SectionProperties.Find(Key) : nullptr;
		const UConfigPropertyBool* Property = Found ? Cast<UConfigPropertyBool>(Found->Get()) : nullptr;
		return Property ? Property->Value : bDefault;
	}

	float ReadFloat(const UConfigPropertySection* Root, const FString& Key, float Default)
	{
		const TObjectPtr<UConfigProperty>* Found = Root ? Root->SectionProperties.Find(Key) : nullptr;
		const UConfigPropertyFloat* Property = Found ? Cast<UConfigPropertyFloat>(Found->Get()) : nullptr;
		return Property ? Property->Value : Default;
	}

	const TCHAR* SMLPropertyPath = TEXT("/SML/Interface/UI/Menu/Mods/ConfigProperties/");

	// Loads SML's Blueprint subclass of a config property class, e.g. BP_ConfigPropertyBool for UConfigPropertyBool.
	UClass* LoadEditorClass(const UClass* NativeClass)
	{
		const FString Name = TEXT("BP_") + NativeClass->GetName();
		const FString Path = FString::Printf(TEXT("%s%s.%s_C"), SMLPropertyPath, *Name, *Name);
		UClass* Loaded = LoadClass<UConfigProperty>(nullptr, *Path);
		if (!Loaded || !Loaded->IsChildOf(NativeClass))
		{
			UE_LOG(LogAutoTrains, Warning, TEXT("Config: SML editor class %s not found; that setting won't be editable in the Mods menu"), *Path);
			return nullptr;
		}
		return Loaded;
	}

	// Creates an instance of EditorClass under Outer with every property value of Source.
	UConfigProperty* CloneAs(const UConfigProperty* Source, UClass* EditorClass, UObject* Outer)
	{
		UConfigProperty* Clone = NewObject<UConfigProperty>(Outer, EditorClass, NAME_None, RF_Public);
		for (TFieldIterator<FProperty> It(Source->GetClass()); It; ++It)
		{
			It->CopyCompleteValue_InContainer(Clone, Source);
		}
		return Clone;
	}
}

UATConfig::UATConfig()
{
	ConfigId = MakeConfigId();
	DisplayName = LOCTEXT("ConfigName", "Auto Trains");
	Description = LOCTEXT("ConfigDescription", "Train traffic without block or path signals.");

	RootSection = CreateDefaultSubobject<UConfigPropertySection>(TEXT("RootSection"));
	auto AddBool = [this](const FString& Key, const TCHAR* ObjectName, const FText& Name, const FText& Tooltip, bool bDefault)
	{
		UConfigPropertyBool* Property = CreateDefaultSubobject<UConfigPropertyBool>(ObjectName);
		Property->DisplayName = Name;
		Property->Tooltip = Tooltip;
		Property->DefaultValue = bDefault;
		Property->Value = bDefault;
		RootSection->SectionProperties.Add(Key, Property);
	};
	const FATSettings Defaults;
	AddBool(TrafficControlKey, TEXT("TrafficControl"), LOCTEXT("TrafficControl", "Traffic control"),
		LOCTEXT("TrafficControlTip", "Auto Trains drives every self-driving train: trains ignore block and path signals and keep out of each other's way by claiming the track ahead. Off gives the game's own signals and scheduler back."),
		Defaults.bTrafficControl);
	AddBool(ReroutingKey, TEXT("Rerouting"), LOCTEXT("Rerouting", "Congestion rerouting"),
		LOCTEXT("ReroutingTip", "Trains take another route, even a longer one, when theirs is busy with standing or oncoming trains."),
		Defaults.bRerouting);
	AddBool(PhaseThroughKey, TEXT("PhaseThrough"), LOCTEXT("PhaseThrough", "Trains pass through each other"),
		LOCTEXT("PhaseThroughTip", "While traffic control is on, trains that touch pass through each other instead of crashing and derailing, so a misjudged gap only looks like a clip."),
		Defaults.bPhaseThrough);
	AddBool(AggressiveKey, TEXT("AggressiveTiming"), LOCTEXT("Aggressive", "Aggressive timing"),
		LOCTEXT("AggressiveTip", "Tighter margins: trains plan braking closer to what they really manage, claim less track ahead, and squeeze through junctions ahead of other trains more readily. Best with trains passing through each other. Off is more cautious."),
		Defaults.bAggressive);

	UConfigPropertyFloat* Margin = CreateDefaultSubobject<UConfigPropertyFloat>(TEXT("SafetyMarginSeconds"));
	Margin->DisplayName = LOCTEXT("SafetyMargin", "Safety margin (seconds)");
	Margin->Tooltip = LOCTEXT("SafetyMarginTip", "Extra time, 0 to 10 seconds, a train leaves when it follows another out of a junction or goes through ahead of another train's claim. 0 starts trains as soon as the timing allows; raise it if trains clip each other.");
	Margin->DefaultValue = Defaults.SafetyMargin;
	Margin->Value = Defaults.SafetyMargin;
	RootSection->SectionProperties.Add(SafetyMarginKey, Margin);
}

void UATConfig::UseSMLEditorClasses()
{
	UATConfig* Defaults = GetMutableDefault<UATConfig>();
	UConfigPropertySection* OldRoot = Defaults->RootSection;
	UClass* SectionClass = OldRoot ? LoadEditorClass(UConfigPropertySection::StaticClass()) : nullptr;
	if (!SectionClass || OldRoot->GetClass() == SectionClass)
	{
		return;
	}

	UConfigPropertySection* NewRoot = Cast<UConfigPropertySection>(CloneAs(OldRoot, SectionClass, Defaults));
	NewRoot->SectionProperties.Reset();
	if (UCP_Section* EditorSection = Cast<UCP_Section>(NewRoot))
	{
		EditorSection->WidgetType = ECP_SectionWidgetType::CPS_Vertical;
		EditorSection->HasHeader = false;
	}
	int32 Converted = 0;
	for (const TPair<FString, TObjectPtr<UConfigProperty>>& Pair : OldRoot->SectionProperties)
	{
		UConfigProperty* Old = Pair.Value;
		if (!Old)
		{
			continue;
		}
		if (UClass* EditorClass = LoadEditorClass(Old->GetClass()))
		{
			UConfigProperty* Clone = CloneAs(Old, EditorClass, NewRoot);
			// SML's number widget accepts 0 to 1 unless told otherwise.
			if (UCP_Float* Float = Cast<UCP_Float>(Clone))
			{
				Float->MinValue = MinSafetyMargin;
				Float->MaxValue = MaxSafetyMargin;
			}
			NewRoot->SectionProperties.Add(Pair.Key, Clone);
			++Converted;
			continue;
		}
		NewRoot->SectionProperties.Add(Pair.Key, Old);
	}
	Defaults->RootSection = NewRoot;
	UE_LOG(LogAutoTrains, Log, TEXT("Config: %d of %d settings use SML's editor widgets"), Converted, OldRoot->SectionProperties.Num());
}

bool UATConfig::Read(const UObject* WorldContext, FATSettings& OutSettings)
{
	const UConfigPropertySection* Root = FindRoot(WorldContext);
	if (!Root)
	{
		return false;
	}
	const FATSettings Defaults;
	OutSettings.bTrafficControl = ReadBool(Root, TrafficControlKey, Defaults.bTrafficControl);
	OutSettings.bRerouting = ReadBool(Root, ReroutingKey, Defaults.bRerouting);
	OutSettings.bPhaseThrough = ReadBool(Root, PhaseThroughKey, Defaults.bPhaseThrough);
	OutSettings.bAggressive = ReadBool(Root, AggressiveKey, Defaults.bAggressive);
	OutSettings.SafetyMargin = FMath::Clamp(ReadFloat(Root, SafetyMarginKey, Defaults.SafetyMargin), MinSafetyMargin, MaxSafetyMargin);
	return true;
}

#undef LOCTEXT_NAMESPACE
