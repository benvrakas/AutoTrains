#include "ATClientSettingsSubsystem.h"
#include "ATConfig.h"
#include "ATRemoteCallObject.h"
#include "AutoTrains.h"

namespace
{
	// How often the Mods menu values are read. [s]
	constexpr float ReadInterval = 0.5f;
	// Edits are sent once none has followed for this long, so dragging a slider sends one change. [s]
	constexpr double EditSettleSeconds = 2.0;
}

AATClientSettingsSubsystem::AATClientSettingsSubsystem()
{
	ReplicationPolicy = ESubsystemReplicationPolicy::SpawnOnClient;
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
}

void AATClientSettingsSubsystem::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	mReadTimer -= DeltaSeconds;
	if (mReadTimer > 0.f || GetNetMode() != NM_Client)
	{
		return;
	}
	mReadTimer = ReadInterval;

	TMap<FString, FString> Values;
	if (!UATConfig::ReadValues(this, Values))
	{
		return;
	}
	const double Now = GetWorld()->GetTimeSeconds();
	if (!mHasLocalValues)
	{
		mLocalValues = MoveTemp(Values);
		mHasLocalValues = true;
		return;
	}
	for (const TPair<FString, FString>& Pair : Values)
	{
		const FString* Old = mLocalValues.Find(Pair.Key);
		if (!Old || *Old != Pair.Value)
		{
			mPendingEdits.Add(Pair.Key, Pair.Value);
			mLastEditTime = Now;
		}
	}
	mLocalValues = MoveTemp(Values);
	if (mPendingEdits.Num() == 0 || Now - mLastEditTime < EditSettleSeconds)
	{
		return;
	}

	TArray<FATSettingChange> Changes;
	for (const TPair<FString, FString>& Pair : mPendingEdits)
	{
		FATSettingChange& Change = Changes.AddDefaulted_GetRef();
		Change.Key = Pair.Key;
		Change.Value = Pair.Value;
	}
	mPendingEdits.Reset();
	if (UATRemoteCallObject* RCO = UATRemoteCallObject::GetLocal(this))
	{
		UE_LOG(LogAutoTrains, Log, TEXT("Settings: sending %d Mods menu changes to the server"), Changes.Num());
		RCO->Server_SetSettings(Changes);
	}
}
