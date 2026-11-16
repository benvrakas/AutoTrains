#pragma once

#include "CoreMinimal.h"
#include "Subsystem/ModSubsystem.h"
#include "ATClientSettingsSubsystem.generated.h"

// On a client connected to a server: sends settings the player changes in their Mods menu to the server, which applies
// them if the player is a server admin.
UCLASS()
class AUTOTRAINS_API AATClientSettingsSubsystem : public AModSubsystem
{
	GENERATED_BODY()

public:
	AATClientSettingsSubsystem();

	virtual void Tick(float DeltaSeconds) override;

private:
	// The Mods menu values last read, key to value as UATConfig::ReadValues gives them.
	TMap<FString, FString> mLocalValues;
	bool mHasLocalValues = false;

	// Changed values not sent yet, and when the last of them changed. [s, world time]
	TMap<FString, FString> mPendingEdits;
	double mLastEditTime = 0.0;

	float mReadTimer = 0.f;
};
