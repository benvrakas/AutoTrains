#pragma once

#include "CoreMinimal.h"
#include "Subsystem/ModSubsystem.h"
#include "ATTrafficSubsystem.generated.h"

class AFGBuildableRailroadTrack;
class AFGTrain;
class UFGRailroadTrackConnectionComponent;

// One track on a train's route, from the track its front is on to its goal.
struct FATRouteStep
{
	AFGBuildableRailroadTrack* Track = nullptr;
	// Connection the train leaves the track through.
	UFGRailroadTrackConnectionComponent* Exit = nullptr;
	// Distance from the train's front to the end of this track when the route was read. [cm]
	float EndDistance = 0.f;
};

// Traffic state of one train, rebuilt every update except its claims.
struct FATTrafficTrain
{
	TWeakObjectPtr<AFGTrain> Train;
	// Self-driving, not docked, following a path: this mod drives it.
	bool bManaged = false;
	TArray<FATRouteStep> Route;
	TSet<AFGBuildableRailroadTrack*> Occupied;
	// Tracks ahead granted to this train, in route order; kept between updates until the train is past them.
	TArray<TWeakObjectPtr<AFGBuildableRailroadTrack>> Claims;
	float Length = 0.f;
	float Speed = 0.f;
	// How far the train's front is ahead of its MU master, which the ATC distances are measured from. [cm]
	float FrontAhead = 0.f;
	float WaitingSeconds = 0.f;
	// Last track the train may drive to the end of; null means no limit beyond its route.
	TWeakObjectPtr<AFGBuildableRailroadTrack> AuthorityTrack;
	bool bAuthorityToGoal = false;
	// Why the claims stop where they do, for the log and /at traffic.
	FString BlockedReason;
	TWeakObjectPtr<AFGTrain> BlockedBy;
};

// Server only. Train traffic without signals: every self-driving train claims the track ahead of it, a track is
// free only if no other train occupies or claims it or a track overlapping it, and a claim may only end where the
// whole train stands off junctions and out of the way of trains coming the other way or across. Each train's
// speed is limited so it can always stop before the end of its claims.
UCLASS()
class AUTOTRAINS_API AATTrafficSubsystem : public AModSubsystem
{
	GENERATED_BODY()

public:
	AATTrafficSubsystem();

	static AATTrafficSubsystem* Get(const UObject* WorldContext);

	virtual void Tick(float DeltaSeconds) override;

	bool IsEnabled() const { return mEnabled; }
	FString SetEnabled(bool bEnabled);

	// Returns true if this mod is driving Train.
	bool IsManaged(const AFGTrain* Train) const;

	// Called after the game's AFGTrain::TickAtc for managed trains; caps the target speed to the train's authority.
	void ApplyAuthority(AFGTrain* Train);

	// Returns the distance from Train's front to the end of its authority, or -1 for no limit. [cm]
	float GetAuthorityDistance(const AFGTrain* Train) const;

	// Returns a one-line traffic summary of Train for telemetry.
	FString DescribeForTelemetry(const AFGTrain* Train) const;

	// Logs every train's traffic state; returns lines for chat.
	TArray<FString> DescribeAll() const;

private:
	void UpdateTraffic(float DeltaSeconds);
	void ReadTrain(FATTrafficTrain& State, AFGTrain* Train) const;
	void ReleasePassedClaims(FATTrafficTrain& State) const;
	void ExtendClaims(FATTrafficTrain& State);
	void UpdateAuthority(FATTrafficTrain& State) const;

	// Returns false and sets OutBlocker if another train occupies or claims Track or a track overlapping it.
	bool IsTrackFree(const AFGBuildableRailroadTrack* Track, const FATTrafficTrain& Self, const FATTrafficTrain*& OutBlocker) const;

	// Returns true if Self may stop with its front at the end of route step EndIndex.
	bool IsValidStop(const FATTrafficTrain& Self, int32 EndIndex, FString& OutReason) const;

	// Returns true if no other moving train needs to pass Track, against or across Self's direction, before Self
	// moves on; trains behind Self going the same way only queue.
	bool IsOutOfOthersWay(const FATTrafficTrain& Self, const FATRouteStep& Step, FString& OutReason) const;

	// Returns Track and the tracks overlapping it (switch branches, crossings).
	const TArray<AFGBuildableRailroadTrack*>& GetConflicts(AFGBuildableRailroadTrack* Track);
	bool IsJunction(AFGBuildableRailroadTrack* Track);

	bool HoldsTrack(const FATTrafficTrain& State, const AFGBuildableRailroadTrack* Track) const;

	// Planning deceleration for Train. [cm/s^2]
	static float GetPlanningDeceleration(const AFGTrain* Train);

	bool mEnabled = false;
	float mUpdateTimer = 0.f;
	TMap<TWeakObjectPtr<AFGTrain>, FATTrafficTrain> mTrains;

	struct FTrackConflicts
	{
		TArray<AFGBuildableRailroadTrack*> Conflicts;
		double ReadTime = 0.0;
	};
	TMap<TWeakObjectPtr<AFGBuildableRailroadTrack>, FTrackConflicts> mConflictCache;
};
