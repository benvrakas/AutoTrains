#pragma once

#include "CoreMinimal.h"
#include "Configuration/ModConfiguration.h"
#include "ATConfig.generated.h"

// Auto Trains settings, read live from SML's Mods menu (the host's apply in multiplayer).
struct FATSettings
{
	bool bTrafficControl = true;
	bool bRerouting = true;
	bool bPhaseThrough = true;
	bool bAggressive = true;
	// Time kept between two trains' bookings of the same track or switch. [s]
	float SafetyMargin = 0.f;
	// Top speed of every self-driving train, 0 for each train's own. [km/h]
	float TopSpeedKmh = 0.f;
	// Multiplier on how fast self-driving trains gain speed.
	float Acceleration = 1.f;
	// Rerouting: a new route must save at least this much time and this share of the trip, and a train keeps a new
	// route this long. [s] [%] [s]
	float RerouteMinGainSeconds = 30.f;
	float RerouteMinGainPercent = 20.f;
	float RerouteHoldSeconds = 60.f;
	// Track a train is granted past its braking distance and its length. [m]
	float LookaheadMetres = 100.f;
	// How far a train stops short of the end of its granted track; before a switch at least 25 m and its fouling zone
	// plus this. [m]
	float StopDistanceMetres = 10.f;
	// Share of its planned braking a train's normal stops use. [%]
	float NormalBrakingPercent = 85.f;
	// Stay assumed at a station until one there has been measured. [s]
	float DefaultStaySeconds = 40.f;
	// How often trains are read, planned and given track. [s]
	float UpdateIntervalSeconds = 0.25f;
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
	static const FString AggressiveKey;

	// Sets OutMin and OutMax to the accepted range of the number setting Key; returns false if Key isn't one.
	static bool GetRange(const FString& Key, float& OutMin, float& OutMax);

	// Rebuilds the default configuration with SML's Blueprint property classes, which carry the editor widgets
	// SML's Mods menu shows. Call before SML registers the configuration.
	static void UseSMLEditorClasses();

	// Fills OutSettings from the registered configuration; returns false, leaving the defaults, if it isn't available.
	static bool Read(const UObject* WorldContext, FATSettings& OutSettings);
};
