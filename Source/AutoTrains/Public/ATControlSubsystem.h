#pragma once

#include "CoreMinimal.h"
#include "RailroadNavigation.h"
#include "Subsystem/ModSubsystem.h"
#include "ATControlSubsystem.generated.h"

class AFGTrain;

// What the test build is making a train do.
enum class EATCommandMode : uint8
{
	None,
	SpeedCap,
	StopAt
};

// Per-train control and measurement state.
struct FATTrainState
{
	EATCommandMode Mode = EATCommandMode::None;
	// Speed cap for SpeedCap. [cm/s]
	float CapSpeed = 0.f;
	// Odometer reading to stop at for StopAt. [cm]
	float StopOdometer = 0.f;
	bool bStopReported = false;
	bool bIgnoreSignals = false;
	// Distance travelled since the train was first seen. [cm]
	float Odometer = 0.f;
	// Target speed the game computed this tick, and what it was after this mod's change. [cm/s]
	float GameTargetSpeed = 0.f;
	float FinalTargetSpeed = 0.f;
	// Speed at the previous telemetry sample. [cm/s]
	float LastSampleSpeed = 0.f;
	// Path this mod gave the train, to notice when something replaces it.
	FRailroadPathWeakPtr AppliedPath;
	bool bPathReplacedReported = false;
};

// Server only. Applies the test commands to trains after their ATC tick and writes telemetry for /at.
UCLASS()
class AUTOTRAINS_API AATControlSubsystem : public AModSubsystem
{
	GENERATED_BODY()

public:
	AATControlSubsystem();

	static AATControlSubsystem* Get(const UObject* WorldContext);

	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// Called after the game's AFGTrain::TickAtc; integrates the odometer and lowers the target speed per the train's command.
	void AfterTickAtc(AFGTrain* Train, float DeltaSeconds);

	// Returns true if Train should drive through stop signals.
	bool IsIgnoringSignals(const AFGTrain* Train) const;

	// Clears the train's signal stops and recomputes its restrictive point and target speed without them. Call after
	// AFGTrain::TickAtc.
	static void IgnoreSignalStops(AFGTrain* Train);

	// Returns the train for a /at list number or a name, or nullptr.
	AFGTrain* FindTrain(const FString& Token) const;

	// The test commands; each returns a message for the player and logs what it did.
	TArray<FString> ListTrains() const;
	FString SetSpeedCap(AFGTrain* Train, float Kmh);
	FString SetStopAhead(AFGTrain* Train, float Metres);
	FString Release(AFGTrain* Train);
	FString ReleaseAll();
	FString SetIgnoreSignals(AFGTrain* Train, bool bIgnore);
	FString CheckRoute(AFGTrain* Train);
	FString RouteVia(AFGTrain* Train, const FVector& Location);
	FString SetLogging(bool bLog);
	FString SetAutopilot(AFGTrain* Train, bool bEnabled);
	FString SetAutopilotAll(bool bEnabled);
	FString RemoveAllSignals();
	FString SetSessionName(const FString& Name);

private:
	// Returns the number of block and path signals in the world that aren't being destroyed.
	int32 CountSignals() const;

	// Returns all trains, sorted by name, as numbered by /at list.
	TArray<AFGTrain*> GetSortedTrains() const;

	// Appends one telemetry row per train.
	void SampleTelemetry(float SampleSeconds);
	void FlushTelemetry();

	TMap<TWeakObjectPtr<AFGTrain>, FATTrainState> mStates;

	bool mLogging = false;
	float mSampleTimer = 0.f;
	float mFlushTimer = 0.f;
	float mLogTime = 0.f;
	FString mTelemetryPath;
	TArray<FString> mPendingRows;

	// Time spent in AfterTickAtc since the last report.
	double mHookSeconds = 0.0;
	int32 mHookCalls = 0;
	float mHookReportTimer = 0.f;
};
