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
	// Full-brake deceleration measured on this train, 0 until measured. [cm/s^2]
	float MeasuredBrake = 0.f;
	// Speed at the previous update, and whether the train was on full brake since. [cm/s]
	float LastSpeed = 0.f;
	bool bFullBrake = false;
	// Whether the game's catch is set to hold the train at its stop point.
	bool bCatchArmed = false;
	// Seconds until this train's route is next checked for congestion, and how often it was rerouted.
	float RerouteTimer = -1.f;
	int32 Reroutes = 0;
	// Last track the train may drive to the end of; null means no limit beyond its route.
	TWeakObjectPtr<AFGBuildableRailroadTrack> AuthorityTrack;
	bool bAuthorityToGoal = false;
	// Why the claims stop where they do, for the log and /at traffic.
	FString BlockedReason;
	TWeakObjectPtr<AFGTrain> BlockedBy;
};

// Server only. Train traffic without signals: every self-driving train claims the track ahead of it, a track is
// free only if no other train occupies or claims it or a conflicting track (overlapping it, or meeting it at a
// switch), and a claim may only end where the whole train stands off junctions and out of the way of trains coming
// the other way. Each train's speed is limited so it can always stop before the end of its claims, with the game's
// catch as the backstop at the last connection.
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

	// Turns congestion rerouting on or off (on by default while traffic control is on).
	FString SetRerouteEnabled(bool bEnabled);

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

	// Returns false if another train occupies or claims Track or a track conflicting with it; sets OutBlocker, the
	// conflicting track it holds, and whether it stands on it.
	bool IsTrackFree(const AFGBuildableRailroadTrack* Track, const FATTrafficTrain& Self, const FATTrafficTrain*& OutBlocker,
		AFGBuildableRailroadTrack*& OutConflict, bool& bOutStandsOnIt) const;

	// Returns true if Self may stop with its front at the end of route step EndIndex.
	bool IsValidStop(const FATTrafficTrain& Self, int32 EndIndex, FString& OutReason) const;

	// Returns true if no other moving train needs to pass Track, against or across Self's direction, before Self
	// moves on; trains behind Self going the same way only queue.
	bool IsOutOfOthersWay(const FATTrafficTrain& Self, const FATRouteStep& Step, FString& OutReason) const;

	// Returns Track first, then every track conflicting with it.
	const TArray<AFGBuildableRailroadTrack*>& GetConflicts(const AFGBuildableRailroadTrack* Track) const;
	bool IsJunction(const AFGBuildableRailroadTrack* Track) const;

	// Rebuilds the conflict graph for every track in the world: the game's overlapping tracks, made symmetric, and
	// all tracks that meet at a joint of three or more tracks (switches and merges).
	void RebuildConflicts();

	bool HoldsTrack(const FATTrafficTrain& State, const AFGBuildableRailroadTrack* Track) const;

	// Congestion other trains than Self put on each track: standing trains, claims, and their routes ahead.
	struct FCongestion
	{
		float Penalty = 0.f;
		// Exit connections of other trains' upcoming passes over the track, to tell same and opposite direction apart.
		TArray<const UFGRailroadTrackConnectionComponent*> UpcomingExits;
	};
	void BuildCongestion(const FATTrafficTrain& Self, TMap<const AFGBuildableRailroadTrack*, FCongestion>& Out) const;

	// Extra cost of driving over Track and leaving through Exit, given the congestion map. [cm]
	static float GetCongestionCost(const TMap<const AFGBuildableRailroadTrack*, FCongestion>& Congestion, const AFGBuildableRailroadTrack* Track,
		const UFGRailroadTrackConnectionComponent* Exit);

	// Looks for a route to State's goal, from the end of its claims, that's clearly cheaper than its current one once
	// congestion is counted, and switches to it.
	void ConsiderReroute(FATTrafficTrain& State);

	// Deceleration to plan stops with: 85% of the measured full-brake deceleration, or 60% of the consist's rating
	// until measured. [cm/s^2]
	static float GetPlanningDeceleration(const FATTrafficTrain& State);

	bool mEnabled = false;
	bool mRerouteEnabled = true;
	float mUpdateTimer = 0.f;
	TMap<TWeakObjectPtr<AFGTrain>, FATTrafficTrain> mTrains;

	TMap<const AFGBuildableRailroadTrack*, TArray<AFGBuildableRailroadTrack*>> mConflicts;
	double mConflictsBuiltAt = -1e9;
	// Set when a track missing from the conflict graph was looked up.
	mutable bool mConflictsStale = false;
};
