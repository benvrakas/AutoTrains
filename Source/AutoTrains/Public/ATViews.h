#pragma once

#include "CoreMinimal.h"
#include "ATViews.generated.h"

class AFGTrain;

// What the server sends a player's locomotive menu and ride HUD about one train (Docs/UI.md).
USTRUCT()
struct AUTOTRAINS_API FATTrainView
{
	GENERATED_BODY()

	UPROPERTY()
	TObjectPtr<AFGTrain> Train = nullptr;

	// 0 not driven by Auto Trains, 1 driving, 2 waiting, 3 docked, 4 stuck (no plan for a minute or in a deadlock), 5
	// passing through a deadlock (the beta safeguard).
	UPROPERTY()
	uint8 State = 0;

	// Now and what the plan asks for. [km/h]
	UPROPERTY()
	float Speed = 0.f;

	UPROPERTY()
	float TargetSpeed = 0.f;

	// Its next stop and when its plan gets it there, -1 if not known. [s]
	UPROPERTY()
	FString Goal;

	UPROPERTY()
	float ArrivalSeconds = -1.f;

	// The train it waits for (empty: none), for how long [s], and why, in a few words.
	UPROPERTY()
	FString WaitingFor;

	UPROPERTY()
	float WaitedSeconds = 0.f;

	UPROPERTY()
	FString Reason;

	// Traffic control is on.
	UPROPERTY()
	bool bTraffic = true;
};

// One train in trouble on the monitor.
USTRUCT()
struct AUTOTRAINS_API FATMonitorRow
{
	GENERATED_BODY()

	UPROPERTY()
	TObjectPtr<AFGTrain> Train = nullptr;

	UPROPERTY()
	FString Name;

	// "Deadlock", "Stuck", "Blocked" or "Waiting".
	UPROPERTY()
	FString Kind;

	UPROPERTY()
	float Seconds = 0.f;

	UPROPERTY()
	FString Reason;
};

// The traffic monitor ("/at monitor").
USTRUCT()
struct AUTOTRAINS_API FATMonitorView
{
	GENERATED_BODY()

	UPROPERTY()
	int32 Trains = 0;

	UPROPERTY()
	int32 Driving = 0;

	UPROPERTY()
	int32 Waiting = 0;

	UPROPERTY()
	int32 Docked = 0;

	UPROPERTY()
	int32 Troubled = 0;

	// Planner round on a worker, averaged over the last minute, and the most it may take. [ms]
	UPROPERTY()
	float CycleMs = 0.f;

	UPROPERTY()
	float BudgetMs = 0.f;

	// Deadlocks passed through since the world loaded (each a bug to report).
	UPROPERTY()
	int32 PassThroughs = 0;

	UPROPERTY()
	bool bTraffic = true;

	UPROPERTY()
	TArray<FATMonitorRow> Rows;
};
