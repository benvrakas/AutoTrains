#pragma once

#include "CoreMinimal.h"
#include "Subsystem/ModSubsystem.h"
#include "ATControlSubsystem.generated.h"

class AFGTrain;

// Per-train measurement state for telemetry.
struct FATTrainState
{
	// Distance travelled since the train was first seen. [cm]
	float Odometer = 0.f;
	// Target speed the game computed this tick, and what it was after this mod's change. [cm/s]
	float GameTargetSpeed = 0.f;
	float FinalTargetSpeed = 0.f;
	// Speed at the previous telemetry sample. [cm/s]
	float LastSampleSpeed = 0.f;
};

// Server only. Hands each train to the traffic subsystem after its ATC tick, carries out /at commands and writes
// telemetry.
UCLASS()
class AUTOTRAINS_API AATControlSubsystem : public AModSubsystem
{
	GENERATED_BODY()

public:
	AATControlSubsystem();

	static AATControlSubsystem* Get(const UObject* WorldContext);

	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// Called after the game's AFGTrain::TickAtc; integrates the odometer and applies the traffic subsystem's control.
	void AfterTickAtc(AFGTrain* Train, float DeltaSeconds);

	// Clears the train's signal stops and recomputes its restrictive point and target speed without them. Call after
	// AFGTrain::TickAtc.
	static void IgnoreSignalStops(AFGTrain* Train);

	// Returns the train for a /at list number or a name, or nullptr.
	AFGTrain* FindTrain(const FString& Token) const;

	// The /at commands; each returns a message for the player and logs what it did.
	TArray<FString> ListTrains() const;
	FString SetLogging(bool bLog);
	FString SetAutopilot(AFGTrain* Train, bool bEnabled);
	FString SetAutopilotAll(bool bEnabled);
	FString RemoveAllSignals();

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
