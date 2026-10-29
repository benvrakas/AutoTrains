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
const FString UATConfig::GhostDeadlocksKey = TEXT("GhostDeadlocks");
const FString UATConfig::HillHelpKey = TEXT("HillHelp");

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
			{TEXT("TopSpeedKmh"), &FATSettings::TopSpeedKmh, 0.f, 500.f, LOCTEXT("TopSpeed", "Top speed (km/h)"),
				LOCTEXT("TopSpeedTip", "Top speed of every self-driving train, 0 to 500 km/h, while traffic control is on. 0 keeps each train's own top speed. Above the speed at which a locomotive's traction runs out (about 120 km/h on flat track), its traction is stretched up to this speed and it draws that much more power.")},
			{TEXT("AccelerationMultiplier"), &FATSettings::Acceleration, 0.25f, 5.f, LOCTEXT("Acceleration", "Acceleration (x)"),
				LOCTEXT("AccelerationTip", "Traction of self-driving trains' locomotives, 0.25 to 5 times their own, while traffic control is on: they accelerate that much faster and draw that much more power. Braking isn't changed.")},
			{TEXT("RerouteMinGainSeconds"), &FATSettings::RerouteMinGainSeconds, 0.f, 600.f, LOCTEXT("RerouteGain", "Reroute: minimum time saved (seconds)"),
				LOCTEXT("RerouteGainTip", "A train only takes another route if it gets there at least this much sooner, 0 to 600 seconds. Higher keeps trains on their usual route more often.")},
			{TEXT("RerouteMinGainPercent"), &FATSettings::RerouteMinGainPercent, 0.f, 100.f, LOCTEXT("RerouteShare", "Reroute: minimum share saved (%)"),
				LOCTEXT("RerouteShareTip", "A train only takes another route if it saves at least this share of the trip's time as well, 0 to 100%.")},
			{TEXT("RerouteHoldSeconds"), &FATSettings::RerouteHoldSeconds, 0.f, 600.f, LOCTEXT("RerouteHold", "Reroute: keep a new route for (seconds)"),
				LOCTEXT("RerouteHoldTip", "After taking another route, a train doesn't change route again for this long, 0 to 600 seconds.")},
			{TEXT("LookaheadMetres"), &FATSettings::LookaheadMetres, 0.f, 2000.f, LOCTEXT("Lookahead", "Lookahead (m)"),
				LOCTEXT("LookaheadTip", "Track a train is given past its braking distance and its own length, 0 to 2000 m. More lets trains run faster towards traffic; less keeps track free for others.")},
			{TEXT("StopDistanceMetres"), &FATSettings::StopDistanceMetres, 1.f, 100.f, LOCTEXT("StopDistance", "Stop distance (m)"),
				LOCTEXT("StopDistanceTip", "How far a waiting train stops short of track it hasn't been given, 1 to 100 m. Before a switch it keeps the switch's fouling zone plus this.")},
			{TEXT("SwitchClearanceMetres"), &FATSettings::SwitchClearanceMetres, 1.f, 10.f, LOCTEXT("SwitchClearance", "Track clearance (m)"),
				LOCTEXT("SwitchClearanceTip", "How close, 1 to 10 m, the centre lines of two tracks may come before trains on them count as in each other's way. A switch's branches block each other only until they are this far apart, and where two tracks cross only the stretch where they come this close counts. Smaller lets trains slip past each other closer to switches (they may clip); larger keeps them further apart.")},
			{TEXT("NormalBrakingPercent"), &FATSettings::NormalBrakingPercent, 30.f, 100.f, LOCTEXT("NormalBraking", "Normal braking (%)"),
				LOCTEXT("NormalBrakingTip", "How hard trains brake for normal stops, 30 to 100% of the braking they plan with. Higher stops later and gets there sooner; lower is gentler and leaves room for error.")},
			{TEXT("DefaultStaySeconds"), &FATSettings::DefaultStaySeconds, 0.f, 600.f, LOCTEXT("DefaultStay", "Station stay estimate (seconds)"),
				LOCTEXT("DefaultStayTip", "How long a train is expected to stay at a stop set to fully load or unload, 0 to 600 seconds, until it has been seen there; after that its own last stay there counts. Stops that load or unload once take the freight platforms' fixed time, or the stop's set duration when there's no cargo to move.")},
			{TEXT("GhostAfterSeconds"), &FATSettings::GhostAfterSeconds, 10.f, 600.f, LOCTEXT("GhostAfter", "Pass through a deadlock after (seconds)"),
				LOCTEXT("GhostAfterTip", "How long trains stay deadlocked, 10 to 600 seconds, before they pass through each other (when that's on).")},
			{TEXT("PlanRetries"), &FATSettings::PlanRetries, 0.f, 10.f, LOCTEXT("PlanRetries", "Planning retries"),
				LOCTEXT("PlanRetriesTip", "When a train finds no way because a train planned before it is in the way, it is planned before that train and the round is planned again, up to this many times per round, 0 to 10. 0 leaves the planning order as it is.")},
			{TEXT("RouteSearchSeconds"), &FATSettings::RouteSearchSeconds, 1.f, 60.f, LOCTEXT("RouteSearch", "Route search interval (seconds)"),
				LOCTEXT("RouteSearchTip", "How often each train looks for a faster route, 1 to 60 seconds. Shorter finds better routes sooner; longer costs less planning time on big networks.")},
			{TEXT("RouteSearchLateSeconds"), &FATSettings::RouteSearchLateSeconds, 1.f, 300.f, LOCTEXT("RouteSearchLate", "Route search when late (seconds)"),
				LOCTEXT("RouteSearchLateTip", "A train looks for another route at once when its planned arrival has got this much later since its last search, 1 to 300 seconds.")},
			{TEXT("RouteSearchStuckSeconds"), &FATSettings::RouteSearchStuckSeconds, 0.25f, 30.f, LOCTEXT("RouteSearchStuck", "Route search while stuck (seconds)"),
				LOCTEXT("RouteSearchStuckTip", "How often a train whose plan finds no way, or a docked train whose departure finds none, looks for another route, 0.25 to 30 seconds. Shorter finds a way round sooner; longer costs less planning time.")},
			{TEXT("TrackRefreshSeconds"), &FATSettings::TrackRefreshSeconds, 5.f, 600.f, LOCTEXT("TrackRefresh", "Track refresh (seconds)"),
				LOCTEXT("TrackRefreshTip", "How often the whole track network is read again, 5 to 600 seconds. Track built or removed is usually picked up within 2 seconds anyway; this catches anything missed.")},
			{TEXT("DeadlockWaitSeconds"), &FATSettings::DeadlockWaitSeconds, 5.f, 300.f, LOCTEXT("DeadlockWait", "Deadlock: count trains waiting longer than (seconds)"),
				LOCTEXT("DeadlockWaitTip", "How long a train has to wait, 5 to 300 seconds, before it counts when looking for deadlocks. The pass-through time starts once a deadlock is found.")},
			{TEXT("HillHelpMax"), &FATSettings::HillHelpMax, 1.f, 10.f, LOCTEXT("HillHelpMax", "Hill help: most extra traction (x)"),
				LOCTEXT("HillHelpMaxTip", "The most traction a stalled train gets on a climb, 1 to 10 times its own. Raise it if heavy trains still stall on very steep ramps.")},
			{TEXT("GhostDistanceMetres"), &FATSettings::GhostDistanceMetres, 0.f, 2000.f, LOCTEXT("GhostDistance", "Deadlock pass-through distance (m)"),
				LOCTEXT("GhostDistanceTip", "A train passing through a deadlock does so until it has driven its own length and this much further, 0 to 2000 m.")},
			{TEXT("GhostMaxSeconds"), &FATSettings::GhostMaxSeconds, 10.f, 600.f, LOCTEXT("GhostMax", "Deadlock pass-through time limit (seconds)"),
				LOCTEXT("GhostMaxTip", "A train passes through a deadlock for at most this long, 10 to 600 seconds, even if it hasn't driven the distance.")},
			{TEXT("PlanSearchLimit"), &FATSettings::PlanSearchLimit, 10000.f, 1000000.f, LOCTEXT("PlanSearchLimit", "Plan search limit (states)"),
				LOCTEXT("PlanSearchLimitTip", "Most steps one train's plan may search, 10000 to 1000000. Lower uses less CPU and memory on huge networks; long or busy routes may then find no way more often.")},
			{TEXT("SlopeLookaheadMetres"), &FATSettings::SlopeLookaheadMetres, 0.f, 2000.f, LOCTEXT("SlopeLookahead", "Slope lookahead (m)"),
				LOCTEXT("SlopeLookaheadTip", "How far ahead of a train the slope is averaged for its braking, 0 to 2000 m. Shorter reacts to short dips and crests; longer evens them out.")},
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
	AddBool(AggressiveKey, TEXT("AggressiveTiming"), LOCTEXT("Aggressive", "Aggressive timing"),
		LOCTEXT("AggressiveTip", "Tighter margins: trains plan braking closer to what they really manage. Best with trains passing through each other. Off is more cautious."),
		Defaults.bAggressive);
	AddBool(GhostDeadlocksKey, TEXT("GhostDeadlocks"), LOCTEXT("GhostDeadlocks", "Clear deadlocks by passing through (beta)"),
		LOCTEXT("GhostDeadlocksTip", "When trains have been stuck waiting for each other in a circle for the set time, they drive on and pass through each other until they're clear of the jam. Each case is written to FactoryGame/Saved/AutoTrains/faults.log so it can be reported."),
		Defaults.bGhostDeadlocks);
	AddBool(HillHelpKey, TEXT("HillHelp"), LOCTEXT("HillHelp", "Help stalled trains up hills"),
		LOCTEXT("HillHelpTip", "A train that has slowed down on a climb too steep for its locomotives to get going again gets extra traction, up to the set limit, until it is moving and the slope under it is one it can manage. Trains are planned not to wait on such climbs; this is for when one has to anyway."),
		Defaults.bHillHelp);

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
	OutSettings.bGhostDeadlocks = ReadBool(Root, GhostDeadlocksKey, Defaults.bGhostDeadlocks);
	OutSettings.bHillHelp = ReadBool(Root, HillHelpKey, Defaults.bHillHelp);
	for (const FNumberSetting& Setting : GetNumberSettings())
	{
		OutSettings.*Setting.Field = FMath::Clamp(ReadFloat(Root, Setting.Key, Defaults.*Setting.Field), Setting.Min, Setting.Max);
	}
	return true;
}

#undef LOCTEXT_NAMESPACE
