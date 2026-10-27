#pragma once

#include "CoreMinimal.h"
#include "Configuration/ModConfiguration.h"
#include "ATConfig.generated.h"

// Auto Trains settings, read live from SML's Mods menu (the host's apply in multiplayer).
struct FATSettings
{
	bool bTrafficControl = true;
	bool bRerouting = true;
	// The space-time planner drives instead of the claim system.
	bool bPlannerControl = true;
	bool bPhaseThrough = true;
	bool bAggressive = true;
	// Extra time a train leaves when following another out of a junction or going ahead of another's claim. [s]
	float SafetyMargin = 0.f;
	// Top speed of every self-driving train, 0 for each train's own. [km/h]
	float TopSpeedKmh = 0.f;
	// Multiplier on how fast self-driving trains gain speed.
	float Acceleration = 1.f;
};

// Auto Trains' mod configuration: on/off switches shown in the Mods menu and Satisfactory Mod Manager.
UCLASS()
class AUTOTRAINS_API UATConfig : public UModConfiguration
{
	GENERATED_BODY()

public:
	UATConfig();

	static const FString TrafficControlKey;
	static const FString ReroutingKey;
	static const FString PlannerKey;
	static const FString PhaseThroughKey;
	static const FString AggressiveKey;
	static const FString SafetyMarginKey;
	static const FString TopSpeedKey;
	static const FString AccelerationKey;

	// Accepted ranges: safety margin [s], top speed [km/h], acceleration multiplier.
	static constexpr float MinSafetyMargin = 0.f;
	static constexpr float MaxSafetyMargin = 10.f;
	static constexpr float MinTopSpeed = 0.f;
	static constexpr float MaxTopSpeed = 500.f;
	static constexpr float MinAcceleration = 0.25f;
	static constexpr float MaxAcceleration = 5.f;

	// Sets OutMin and OutMax to the accepted range of the number setting Key; returns false if Key isn't one.
	static bool GetRange(const FString& Key, float& OutMin, float& OutMax);

	// Rebuilds the default configuration with SML's Blueprint property classes, which carry the editor widgets
	// SML's Mods menu shows. Call before SML registers the configuration.
	static void UseSMLEditorClasses();

	// Fills OutSettings from the registered configuration; returns false, leaving the defaults, if it isn't available.
	static bool Read(const UObject* WorldContext, FATSettings& OutSettings);
};
