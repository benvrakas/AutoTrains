#pragma once

#include "CoreMinimal.h"
#include "Subsystem/ModSubsystem.h"
#include "ASTestSignalSubsystem.generated.h"

class AFGBuildableRailroadSignal;
class AFGTrain;
class UFGRailroadTrackConnectionComponent;

// Server only. Places, hides and removes the test build's signals, which are never saved, and logs train and block
// state for /sigtest.
UCLASS()
class AUTOSIGNALS_API AASTestSignalSubsystem : public AModSubsystem
{
	GENERATED_BODY()

public:
	AASTestSignalSubsystem();

	static AASTestSignalSubsystem* Get(const UObject* WorldContext);

	// Returns true if Actor is a signal placed by this mod.
	static bool IsTestSignal(const AActor* Actor);

	virtual void Tick(float DeltaSeconds) override;

	// Places a signal at the track joint nearest Location for trains moving along Direction, plus the opposite one if
	// bBothWays. Returns a message for the player.
	FString PlaceSignals(const FVector& Location, const FVector& Direction, bool bPathSignal, bool bBothWays);

	// Removes every test signal; returns how many there were.
	int32 ClearSignals();

	// Hides or shows every test signal; hidden ones also have no collision.
	void SetSignalsHidden(bool bHide);

	// Turns the once-a-second log of train and signal changes on or off.
	void SetWatching(bool bWatch);

	// Logs the test signals, track graphs, trains and scheduler bookings; returns a short summary for chat.
	TArray<FString> LogState();

	static const FName SignalTag;

private:
	// Returns the track connection nearest Location within MaxDistance, or nullptr.
	UFGRailroadTrackConnectionComponent* FindNearestConnection(const FVector& Location, float MaxDistance) const;

	// Spawns one signal at the joint, read by trains leaving through NearSide; returns nullptr and sets OutError on failure.
	AFGBuildableRailroadSignal* SpawnSignal(const TArray<UFGRailroadTrackConnectionComponent*>& NearSide, bool bPathSignal, FString& OutError);

	// Returns a one-line description of a test signal.
	static FString DescribeSignal(AFGBuildableRailroadSignal* Signal);

	// Returns a one-line description of Train's driving state; OutKey gets the parts that matter for change detection.
	static FString DescribeTrain(AFGTrain* Train, FString& OutKey);

	void LogChanges();

	TArray<TWeakObjectPtr<AFGBuildableRailroadSignal>> mSignals;
	TMap<TWeakObjectPtr<AFGTrain>, FString> mLastTrainKeys;
	TMap<TWeakObjectPtr<AFGBuildableRailroadSignal>, uint8> mLastAspects;
	bool mHidden = false;
	bool mWatching = false;
	float mWatchTimer = 0.f;
};
