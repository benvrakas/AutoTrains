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
const FString UATConfig::GhostDeadlocksKey = TEXT("GhostDeadlocks");
const FString UATConfig::HillHelpKey = TEXT("HillHelp");
const FString UATConfig::DetailedLogKey = TEXT("DetailedLog");

namespace
{
	// A number setting: its key, field, accepted range, Mods menu name and tooltip.
	struct FNumberSetting
	{
		const TCHAR* Key;
		float FATSettings::* Field;
		float Min;
		float Max;
		FText Name;
		FText Tooltip;
	};

	const TArray<FNumberSetting>& GetNumberSettings()
	{
		static const TArray<FNumberSetting> Settings = {
			{TEXT("SafetyMarginSeconds"), &FATSettings::SafetyMargin, 0.f, 10.f, LOCTEXT("SafetyMargin", "Safety margin (seconds)"),
				LOCTEXT("SafetyMarginTip", "Time, 0 to 10 seconds, kept between two trains using the same track or switch. 0 lets a train in as soon as the one before it has cleared; raise it if trains clip each other.")},
			{TEXT("RerouteMinGainSeconds"), &FATSettings::RerouteMinGainSeconds, 0.f, 600.f, LOCTEXT("RerouteGain", "Reroute: minimum time saved (seconds)"),
				LOCTEXT("RerouteGainTip", "A train only takes another route if it gets there at least this much sooner, 0 to 600 seconds. Higher keeps trains on their usual route more often.")},
			{TEXT("LookaheadMetres"), &FATSettings::LookaheadMetres, 0.f, 2000.f, LOCTEXT("Lookahead", "Lookahead (m)"),
				LOCTEXT("LookaheadTip", "Track a train is given past its braking distance and its own length, 0 to 2000 m. More lets trains run faster towards traffic; less keeps track free for others.")},
			{TEXT("SwitchClearanceMetres"), &FATSettings::SwitchClearanceMetres, 1.f, 10.f, LOCTEXT("SwitchClearance", "Track clearance (m)"),
				LOCTEXT("SwitchClearanceTip", "How close, 1 to 10 m, the centre lines of two tracks may come before trains on them count as in each other's way. A switch's branches block each other only until they are this far apart, and where two tracks cross only the stretch where they come this close counts. Smaller lets trains slip past each other closer to switches (they may clip); larger keeps them further apart.")},
			{TEXT("DefaultStaySeconds"), &FATSettings::DefaultStaySeconds, 0.f, 600.f, LOCTEXT("DefaultStay", "Station stay estimate (seconds)"),
				LOCTEXT("DefaultStayTip", "How long a train is expected to stay at a stop set to fully load or unload, 0 to 600 seconds, until it has been seen there; after that its own last stay there counts. Stops that load or unload once take the freight platforms' fixed time, or the stop's set duration when there's no cargo to move.")},
			{TEXT("GhostAfterSeconds"), &FATSettings::GhostAfterSeconds, 10.f, 600.f, LOCTEXT("GhostAfter", "Pass through a deadlock after (seconds)"),
				LOCTEXT("GhostAfterTip", "How long trains stay deadlocked, 10 to 600 seconds, before one of them passes through the others (when that's on).")},
			{TEXT("RouteSearchSeconds"), &FATSettings::RouteSearchSeconds, 1.f, 60.f, LOCTEXT("RouteSearch", "Route search interval (seconds)"),
				LOCTEXT("RouteSearchTip", "How often each train looks for a faster route, 1 to 60 seconds. Shorter finds better routes sooner; longer costs less planning time on big networks.")},
			{TEXT("PlanSearchLimit"), &FATSettings::PlanSearchLimit, 10000.f, 1000000.f, LOCTEXT("PlanSearchLimit", "Plan search limit (states)"),
				LOCTEXT("PlanSearchLimitTip", "Most steps one train's plan may search, 10000 to 1000000. Lower uses less CPU and memory on huge networks; long or busy routes may then find no way more often.")},
			{TEXT("UpdateIntervalSeconds"), &FATSettings::UpdateIntervalSeconds, 0.05f, 2.f, LOCTEXT("UpdateInterval", "Update interval (seconds)"),
				LOCTEXT("UpdateIntervalTip", "How often trains are planned and given track, 0.05 to 2 seconds. Shorter reacts faster and costs more time per second on big networks; longer is lighter but trains react later.")},
		};
		return Settings;
	}

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
		LOCTEXT("TrafficControlTip", "Auto Trains drives every self-driving train: trains ignore block and path signals and keep out of each other's way by booking the track and switches ahead in time. Off gives the game's own signals and scheduler back."),
		Defaults.bTrafficControl);
	AddBool(ReroutingKey, TEXT("Rerouting"), LOCTEXT("Rerouting", "Congestion rerouting"),
		LOCTEXT("ReroutingTip", "Trains take another route, even a longer one, when it gets them there sooner than waiting for standing or oncoming trains on theirs."),
		Defaults.bRerouting);
	AddBool(PhaseThroughKey, TEXT("PhaseThrough"), LOCTEXT("PhaseThrough", "Trains pass through each other"),
		LOCTEXT("PhaseThroughTip", "While traffic control is on, trains that touch pass through each other instead of crashing and derailing, so a misjudged gap only looks like a clip."),
		Defaults.bPhaseThrough);
	AddBool(GhostDeadlocksKey, TEXT("GhostDeadlocks"), LOCTEXT("GhostDeadlocks", "Clear deadlocks by passing through (beta)"),
		LOCTEXT("GhostDeadlocksTip", "A safeguard while Auto Trains is in beta: when trains have been stuck waiting for each other in a circle for the set time, one of them drives on through the others until it is clear of the jam. Each case is written to the log and to FactoryGame/Saved/AutoTrains/faults.log so it can be reported."),
		Defaults.bGhostDeadlocks);
	AddBool(HillHelpKey, TEXT("HillHelp"), LOCTEXT("HillHelp", "Help stalled trains up hills"),
		LOCTEXT("HillHelpTip", "A train that has slowed down on a climb too steep for its locomotives to get going again gets extra traction, up to four times its own, until it is moving and the slope under it is one it can manage. Trains are planned not to wait on such climbs; this is for when one has to anyway."),
		Defaults.bHillHelp);
	AddBool(DetailedLogKey, TEXT("DetailedLog"), LOCTEXT("DetailedLog", "Detailed log"),
		LOCTEXT("DetailedLogTip", "Writes much more to FactoryGame.log: what each train waits for, its station stays and next legs, how it speeds up and brakes, and every plan that finds no way. Also records every train's speed and control 5 times a second to a CSV file in FactoryGame/Saved/AutoTrains. Turn it on to report a problem."),
		Defaults.bDetailedLog);

	for (const FNumberSetting& Setting : GetNumberSettings())
	{
		UConfigPropertyFloat* Property = CreateDefaultSubobject<UConfigPropertyFloat>(Setting.Key);
		Property->DisplayName = Setting.Name;
		Property->Tooltip = Setting.Tooltip;
		Property->DefaultValue = Defaults.*Setting.Field;
		Property->Value = Defaults.*Setting.Field;
		RootSection->SectionProperties.Add(Setting.Key, Property);
	}
}

bool UATConfig::GetRange(const FString& Key, float& OutMin, float& OutMax)
{
	for (const FNumberSetting& Setting : GetNumberSettings())
	{
		if (Key == Setting.Key)
		{
			OutMin = Setting.Min;
			OutMax = Setting.Max;
			return true;
		}
	}
	return false;
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
			UCP_Float* Float = Cast<UCP_Float>(Clone);
			float Min = 0.f;
			float Max = 1.f;
			if (Float && GetRange(Pair.Key, Min, Max))
			{
				Float->MinValue = Min;
				Float->MaxValue = Max;
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

TArray<FString> UATConfig::DescribeForChat(const UObject* WorldContext)
{
	TArray<FString> Out;
	const UConfigPropertySection* Root = FindRoot(WorldContext);
	if (!Root)
	{
		Out.Add(TEXT("Auto Trains' configuration isn't available on this machine."));
		return Out;
	}
	for (const TPair<FString, TObjectPtr<UConfigProperty>>& Pair : Root->SectionProperties)
	{
		if (const UConfigPropertyBool* Bool = Cast<UConfigPropertyBool>(Pair.Value.Get()))
		{
			Out.Add(FString::Printf(TEXT("%s%s = %s   (%s)"), Bool->Value != Bool->DefaultValue ? TEXT("* ") : TEXT(""), *Pair.Key,
				Bool->Value ? TEXT("on") : TEXT("off"), *Bool->DisplayName.ToString()));
		}
		else if (const UConfigPropertyFloat* Float = Cast<UConfigPropertyFloat>(Pair.Value.Get()))
		{
			float Min = 0.f;
			float Max = 0.f;
			GetRange(Pair.Key, Min, Max);
			Out.Add(FString::Printf(TEXT("%s%s = %g   (%s, %g to %g)"), !FMath::IsNearlyEqual(Float->Value, Float->DefaultValue) ? TEXT("* ") : TEXT(""), *Pair.Key,
				Float->Value, *Float->DisplayName.ToString(), Min, Max));
		}
	}
	return Out;
}

bool UATConfig::Write(const UObject* WorldContext, const FString& Key, const FString& Value, FString& OutMessage)
{
	const UWorld* World = WorldContext ? WorldContext->GetWorld() : nullptr;
	const UGameInstance* GameInstance = World ? World->GetGameInstance() : nullptr;
	UConfigManager* Manager = GameInstance ? GameInstance->GetSubsystem<UConfigManager>() : nullptr;
	UConfigPropertySection* Root = Manager ? Manager->GetConfigurationRootSection(MakeConfigId()) : nullptr;
	if (!Root)
	{
		OutMessage = TEXT("Auto Trains' configuration isn't available on this machine.");
		UE_LOG(LogAutoTrains, Warning, TEXT("Settings: write of %s failed: configuration not found"), *Key);
		return false;
	}

	// The setting: an exact key, else the only key or name containing Key.
	const FString Wanted = Key.TrimStartAndEnd();
	FString Chosen;
	TArray<FString> Matches;
	for (const TPair<FString, TObjectPtr<UConfigProperty>>& Pair : Root->SectionProperties)
	{
		const UConfigProperty* Property = Pair.Value.Get();
		if (!Property)
		{
			continue;
		}
		if (Pair.Key.Equals(Wanted, ESearchCase::IgnoreCase))
		{
			Chosen = Pair.Key;
			break;
		}
		if (Pair.Key.Contains(Wanted, ESearchCase::IgnoreCase) || Property->DisplayName.ToString().Contains(Wanted, ESearchCase::IgnoreCase))
		{
			Matches.Add(Pair.Key);
		}
	}
	if (Chosen.IsEmpty() && Matches.Num() == 1)
	{
		Chosen = Matches[0];
	}
	if (Chosen.IsEmpty())
	{
		OutMessage = Matches.Num() == 0 ? FString::Printf(TEXT("No setting called '%s'; /at settings lists them."), *Wanted)
			: FString::Printf(TEXT("'%s' matches %s; use the full key."), *Wanted, *FString::Join(Matches, TEXT(", ")));
		return false;
	}

	UConfigProperty* Property = Root->SectionProperties.FindChecked(Chosen).Get();
	FString Shown;
	if (UConfigPropertyBool* Bool = Cast<UConfigPropertyBool>(Property))
	{
		const FString Word = Value.TrimStartAndEnd();
		const bool bOn = Word.Equals(TEXT("on"), ESearchCase::IgnoreCase) || Word.Equals(TEXT("true"), ESearchCase::IgnoreCase) || Word.Equals(TEXT("yes"), ESearchCase::IgnoreCase) || Word == TEXT("1");
		const bool bOff = Word.Equals(TEXT("off"), ESearchCase::IgnoreCase) || Word.Equals(TEXT("false"), ESearchCase::IgnoreCase) || Word.Equals(TEXT("no"), ESearchCase::IgnoreCase) || Word == TEXT("0");
		if (!bOn && !bOff)
		{
			OutMessage = FString::Printf(TEXT("%s takes on or off."), *Chosen);
			return false;
		}
		Bool->Value = bOn;
		Shown = bOn ? TEXT("on") : TEXT("off");
	}
	else if (UConfigPropertyFloat* Float = Cast<UConfigPropertyFloat>(Property))
	{
		float Min = 0.f;
		float Max = 0.f;
		const FString Number = Value.TrimStartAndEnd();
		const bool bRange = GetRange(Chosen, Min, Max);
		const float Parsed = FCString::Atof(*Number);
		if (!bRange || !Number.IsNumeric() || Parsed < Min || Parsed > Max)
		{
			OutMessage = FString::Printf(TEXT("%s takes a number from %g to %g."), *Chosen, Min, Max);
			return false;
		}
		Float->Value = Parsed;
		Shown = FString::Printf(TEXT("%g"), Parsed);
	}
	else
	{
		OutMessage = FString::Printf(TEXT("%s can't be set from chat."), *Chosen);
		UE_LOG(LogAutoTrains, Warning, TEXT("Settings: write of %s failed: property of an unexpected class"), *Chosen);
		return false;
	}
	Property->MarkDirty();
	Manager->MarkConfigurationDirty(MakeConfigId());
	Manager->FlushPendingSaves();
	OutMessage = FString::Printf(TEXT("%s is now %s."), *Chosen, *Shown);
	return true;
}

bool UATConfig::ReadValues(const UObject* WorldContext, TMap<FString, FString>& OutValues)
{
	const UConfigPropertySection* Root = FindRoot(WorldContext);
	if (!Root)
	{
		return false;
	}
	for (const TPair<FString, TObjectPtr<UConfigProperty>>& Pair : Root->SectionProperties)
	{
		if (const UConfigPropertyBool* Bool = Cast<UConfigPropertyBool>(Pair.Value.Get()))
		{
			OutValues.Add(Pair.Key, Bool->Value ? TEXT("on") : TEXT("off"));
		}
		else if (const UConfigPropertyFloat* Float = Cast<UConfigPropertyFloat>(Pair.Value.Get()))
		{
			OutValues.Add(Pair.Key, FString::SanitizeFloat(Float->Value));
		}
	}
	return true;
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
	OutSettings.bGhostDeadlocks = ReadBool(Root, GhostDeadlocksKey, Defaults.bGhostDeadlocks);
	OutSettings.bHillHelp = ReadBool(Root, HillHelpKey, Defaults.bHillHelp);
	OutSettings.bDetailedLog = ReadBool(Root, DetailedLogKey, Defaults.bDetailedLog);
	for (const FNumberSetting& Setting : GetNumberSettings())
	{
		OutSettings.*Setting.Field = FMath::Clamp(ReadFloat(Root, Setting.Key, Defaults.*Setting.Field), Setting.Min, Setting.Max);
	}
	return true;
}

#undef LOCTEXT_NAMESPACE
