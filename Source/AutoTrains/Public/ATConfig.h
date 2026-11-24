#pragma once

#include "CoreMinimal.h"
#include "Configuration/ModConfiguration.h"
#include "ATConfig.generated.h"

// Auto Trains settings, read live from SML's Mods menu (the host's apply in multiplayer; an admin's Mods menu changes
// are sent to a server).
struct FATSettings
{
	bool bTrafficControl = true;
	bool bRerouting = true;
	bool bPhaseThrough = true;
	// Trains deadlocked for GhostAfterSeconds pass through each other to clear the jam (a beta safeguard).
	bool bGhostDeadlocks = true;
	float GhostAfterSeconds = 60.f;
	// A train too slow to climb the slope under it gets extra traction, up to HillHelpMax times its own, until it can.
	bool bHillHelp = true;
	float HillHelpMax = 4.f;
	// A train told to go that stands this long, on any slope, gets more hill help, again each time this passes. [s]
	float HillStallSeconds = 4.f;
	// Writes every train's waits, departures, acceleration and braking to FactoryGame.log, and telemetry to a CSV file.
	bool bDetailedLog = false;
	// Time kept between two trains' bookings of the same track or switch. [s]
	float SafetyMargin = 0.f;
	// Rerouting: a new route must save at least this much time and this share of the trip, and a train keeps a new
	// route this long. [s] [%] [s]
	float RerouteMinGainSeconds = 30.f;
	float RerouteMinGainPercent = 20.f;
	float RerouteHoldSeconds = 60.f;
	// Track a train is granted past its braking distance and its length. [m]
	float LookaheadMetres = 100.f;
	// How close the centre lines of two tracks may come before trains on them would touch: a switch's branches foul it
	// until they are this far apart, and crossing tracks conflict only where they come this close. [m]
	float SwitchClearanceMetres = 3.f;
	// Stay assumed at a fully load or unload stop until one there has been measured. [s]
	float DefaultStaySeconds = 40.f;
	// How often trains are read, planned and given track. [s]
	float UpdateIntervalSeconds = 0.25f;
	// Route search: every train this often, at once when its planned arrival has got this much later since its last
	// search, and this often while its plan finds no way. [s]
	float RouteSearchSeconds = 5.f;
	float RouteSearchLateSeconds = 15.f;
	float RouteSearchStuckSeconds = 1.f;
	// How often the track network is read again in full. [s]
	float TrackRefreshSeconds = 30.f;
	// How long a train waits before it counts for deadlock detection. [s]
	float DeadlockWaitSeconds = 20.f;
	// Most search states one plan may use.
	float PlanSearchLimit = 200000.f;
	// How far ahead a train's slope is averaged for its braking. [m]
	float SlopeLookaheadMetres = 300.f;
};

// Auto Trains' mod configuration: switches and numbers shown in the Mods menu and Satisfactory Mod Manager.
UCLASS()
class AUTOTRAINS_API UATConfig : public UModConfiguration
{
	GENERATED_BODY()

public:
	UATConfig();

	static const FString TrafficControlKey;
	static const FString ReroutingKey;
	static const FString PhaseThroughKey;
	static const FString GhostDeadlocksKey;
	static const FString HillHelpKey;
	static const FString DetailedLogKey;

	// Sets OutMin and OutMax to the accepted range of the number setting Key; returns false if Key isn't one.
	static bool GetRange(const FString& Key, float& OutMin, float& OutMax);

	// Rebuilds the default configuration with SML's Blueprint property classes, which carry the editor widgets
	// SML's Mods menu shows. Call before SML registers the configuration.
	static void UseSMLEditorClasses();

	// Fills OutSettings from the registered configuration; returns false, leaving the defaults, if it isn't available.
	static bool Read(const UObject* WorldContext, FATSettings& OutSettings);

	// One line per setting of the registered configuration for chat: key, value, range and name; settings off their
	// default are marked.
	static TArray<FString> DescribeForChat(const UObject* WorldContext);

	// Sets the setting Key (any case; a unique part of a key or name also works) to Value ("on"/"off" or a number) in
	// the registered configuration, which SML saves to this machine's config file. OutMessage says what happened.
	// Returns false if Key or Value isn't valid or the configuration isn't available.
	static bool Write(const UObject* WorldContext, const FString& Key, const FString& Value, FString& OutMessage);

	// Fills OutValues with every setting of the registered configuration as Write takes it: key to "on"/"off" or a
	// number. Returns false if the configuration isn't available.
	static bool ReadValues(const UObject* WorldContext, TMap<FString, FString>& OutValues);
};
