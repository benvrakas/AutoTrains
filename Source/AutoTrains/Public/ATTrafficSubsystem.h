#pragma once

#include "CoreMinimal.h"
#include "Subsystem/ModSubsystem.h"
#include "ATTrafficSubsystem.generated.h"

class AFGBuildableRailroadTrack;
class AFGTrain;
class UFGRailroadTrackConnectionComponent;

// Full-brake braking is measured per speed band of this width, up to ATBrakeBands bands. [cm/s]
constexpr float ATBrakeBandWidth = 1000.f;
constexpr int32 ATBrakeBands = 8;

// One track on a train's route, from the track its front is on to its goal.
struct FATRouteStep
{
	AFGBuildableRailroadTrack* Track = nullptr;
	// Connection the train leaves the track through.
	UFGRailroadTrackConnectionComponent* Exit = nullptr;
	// Distance from the train's MU master to the end of this track when the route was read. [cm]
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
	// Switch joints with a vehicle of this train within their fouling zone.
	TSet<int32> OccupiedJoints;
	// Tracks ahead granted to this train, in route order; kept between updates until the train is past them.
	TArray<TWeakObjectPtr<AFGBuildableRailroadTrack>> Claims;
	// Switch joints the train's held tracks pass through.
	TSet<int32> ClaimedJoints;
	// Trains holding track this train has claimed too: it follows them and keeps its distance from them.
	TArray<TWeakObjectPtr<AFGTrain>> FollowLeaders;
	// Trains this one counted as moving out of its way when it claimed track they hold. Apart from trains ahead of it
	// the same way, only these lead it.
	TSet<TWeakObjectPtr<AFGTrain>> FollowedTrains;
	float Length = 0.f;
	float Speed = 0.f;
	// How far the train's front is ahead of its MU master, which the ATC distances are measured from. [cm]
	float FrontAhead = 0.f;
	float WaitingSeconds = 0.f;
	// Full-brake braking force measured on this train per speed band, as deceleration times mass so a train that
	// takes on cargo plans with less; 0 where not measured. [cm/s^2 kg]
	float BrakeForce[ATBrakeBands] = {};
	// Speed at the previous update. [cm/s]
	float LastSpeed = 0.f;
	// Whether the train is on full brake, and whether it was at the previous update.
	bool bFullBrake = false;
	bool bWasFullBrake = false;
	// Seconds until the moving train that blocks this one has cleared where it blocks; -1 if unknown.
	float ClearSeconds = -1.f;
	// Seconds the train has stood still.
	float StoppedSeconds = 0.f;
	// Whether the game's catch is set to hold the train at its stop point.
	bool bCatchArmed = false;
	// Seconds until this train's route is next checked for congestion, and how often it was rerouted.
	float RerouteTimer = -1.f;
	int32 Reroutes = 0;
	// Seconds before this train may reroute again after a reroute.
	float RerouteCooldown = 0.f;
	// Last track the train may drive to the end of; null means no limit beyond its route.
	TWeakObjectPtr<AFGBuildableRailroadTrack> AuthorityTrack;
	bool bAuthorityToGoal = false;
	// How far before the end of the authority track the train stops: more when a switch is there. [cm]
	float StopMargin = 1000.f;
	// Why the claims stop where they do, for the log and /at traffic.
	FString BlockedReason;
	TWeakObjectPtr<AFGTrain> BlockedBy;
};

// Server only. Train traffic without signals. Every self-driving train claims the track ahead of it. A track is
// free unless another train stands on it, has claimed it, or holds a track crossing it; a switch is free unless
// another train has a vehicle within its fouling zone or holds track through it. A claim may only end where the whole
// train stands clear of crossings and switches and out of the way of trains coming the other way. Each train's
// speed is limited so it can always stop before the end of its claims, with the game's catch as the backstop.
// A train may take track another train has only claimed if it clears it well before that train could arrive.
UCLASS()
class AUTOTRAINS_API AATTrafficSubsystem : public AModSubsystem
{
	GENERATED_BODY()

public:
	AATTrafficSubsystem();

	static AATTrafficSubsystem* Get(const UObject* WorldContext);

	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	bool IsEnabled() const { return mEnabled; }
	FString SetEnabled(bool bEnabled);

	// Turns congestion rerouting on or off (on by default while traffic control is on).
	FString SetRerouteEnabled(bool bEnabled);

	// Returns true if trains should pass through each other instead of colliding (traffic control and the setting on).
	static bool ShouldPhaseThrough();

	// Called by the collision hook each time a train contact was let through.
	static void CountPhaseThrough();

	// Returns true if this mod is driving Train.
	bool IsManaged(const AFGTrain* Train) const;

	// Called after the game's AFGTrain::TickAtc for managed trains; caps the target speed to the train's authority.
	void ApplyAuthority(AFGTrain* Train);

	// Returns the distance from Train's front to the end of its authority track, or -1 for no limit. [cm]
	float GetAuthorityDistance(const AFGTrain* Train) const;

	// Returns a one-line traffic summary of Train for telemetry.
	FString DescribeForTelemetry(const AFGTrain* Train) const;

	// Logs every train's traffic state; returns lines for chat.
	TArray<FString> DescribeAll() const;

private:
	// Another train's claim Self wants to take over, at a track or a switch on Self's route.
	struct FPreemption
	{
		FATTrafficTrain* Holder = nullptr;
		const AFGBuildableRailroadTrack* Track = nullptr;
		int32 Joint = INDEX_NONE;
	};

	// Applies the Mods menu settings; a changed switch overrides what /at set for this session.
	void ApplySettings();
	void UpdateTraffic(float DeltaSeconds);
	void ReadTrain(FATTrafficTrain& State, AFGTrain* Train) const;
	void ReleasePassedClaims(FATTrafficTrain& State) const;
	void ExtendClaims(FATTrafficTrain& State);
	void UpdateAuthority(FATTrafficTrain& State) const;
	void RefreshClaimedJoints(FATTrafficTrain& State) const;
	void LogDeadlocks();

	// Returns false if another train (not in Ignore) stands on, claims or holds a track crossing Track; sets the
	// holder, the track it holds, and whether it stands on it.
	bool IsTrackFree(const AFGBuildableRailroadTrack* Track, const FATTrafficTrain& Self, FATTrafficTrain*& OutHolder,
		AFGBuildableRailroadTrack*& OutConflict, bool& bOutStandsOnIt, const TSet<const FATTrafficTrain*>* Ignore = nullptr);

	// Returns false if another train (not in Ignore) has a vehicle in Joint's fouling zone or holds track through it.
	bool IsJointFree(int32 Joint, const FATTrafficTrain& Self, FATTrafficTrain*& OutHolder, bool& bOutStandsAtIt,
		const TSet<const FATTrafficTrain*>* Ignore = nullptr);

	// Returns true if every preemption can be done safely with Self clearing the conflicts within ClearDistance;
	// sets OutReason otherwise.
	bool CanPreempt(const FATTrafficTrain& Self, const TArray<FPreemption>& Preemptions, float ClearDistance, FString& OutReason) const;
	void ApplyPreemptions(const TArray<FPreemption>& Preemptions);

	// Returns the route index at which Holder's claims meet the preempted item, or INDEX_NONE; for a switch it's the
	// step whose exit is the switch.
	int32 FindOnRoute(const FATTrafficTrain& Holder, const FPreemption& Preemption) const;

	// Returns true if another moving train's route soon passes Track or a track crossing it (Joint: through the
	// switch), so Self mustn't stand there.
	bool IsNeededByOthers(const FATTrafficTrain& Self, const AFGBuildableRailroadTrack* Track, int32 Joint) const;

	// Returns true if Holder, standing on Track (or at Joint), is moving away and clears it before Self, SelfDistance
	// away, could get there; Self may then count it as free and start moving early.
	bool IsClearingInTime(const FATTrafficTrain& Self, const FATTrafficTrain& Holder, const AFGBuildableRailroadTrack* Track, int32 Joint,
		float SelfDistance) const;

	// Returns how far Holder's front still travels until its tail has cleared Track (or Joint's fouling zone). [cm]
	float GetClearDistance(const FATTrafficTrain& Holder, const AFGBuildableRailroadTrack* Track, int32 Joint) const;

	// Returns the seconds until the train blocking State has cleared the first track (or switch) beyond State's
	// claims that it holds, or -1 if it may stop before then.
	float GetBlockerClearSeconds(const FATTrafficTrain& State) const;

	// Returns 1 if Other is ahead on Self's route going the same way, -1 if it's on Self's route the other way or
	// behind Self on its track, 0 if the track Other's master is on isn't on Self's route.
	int32 GetRouteOrder(const FATTrafficTrain& Self, const FATTrafficTrain& Other) const;

	// Returns true if Other leads Self: it's ahead on Self's route going the same way, or Self followed it when
	// claiming. A train following Self is never its leader.
	bool IsLeaderOf(const FATTrafficTrain& Self, const FATTrafficTrain& Other) const;

	// Returns true if Holder runs ahead of Self the same way: the track Holder's master is on lies on Self's route at
	// or after FromIndex, left through the same connection, and Holder hasn't stood still for long.
	bool IsSameWayLeader(const FATTrafficTrain& Self, const FATTrafficTrain& Holder, int32 FromIndex) const;

	// Drops Self's claims from the first one shared with a train that has stopped.
	void RevalidateFollowClaims(FATTrafficTrain& State);

	// Sets State's follow leaders: other trains holding track it has claimed.
	void UpdateFollowLeaders(FATTrafficTrain& State) const;

	// Returns how far Train may go behind its follow leaders: to the first track along its path one of them holds,
	// plus that leader's braking distance; -1 if it follows none. [cm]
	float GetFollowLimit(const AFGTrain* Train, const FATTrafficTrain& State) const;

	// Returns the distance from Train's front to the start of Track along its path (negative once on it), or a large
	// negative number if Track isn't on it. [cm]
	float GetDistanceToTrackStart(const AFGTrain* Train, const FATTrafficTrain& State, const AFGBuildableRailroadTrack* Track) const;

	// Returns true if Self may stop with its front StopMargin before the end of route step EndIndex.
	bool IsValidStop(const FATTrafficTrain& Self, int32 EndIndex, FString& OutReason) const;

	// Returns true if no other moving train needs to pass Track, against Self's direction, before Self moves on;
	// trains behind Self going the same way only queue.
	bool IsOutOfOthersWay(const FATTrafficTrain& Self, const FATRouteStep& Step, FString& OutReason) const;

	// Returns Track first, then every track crossing it.
	const TArray<AFGBuildableRailroadTrack*>& GetConflicts(const AFGBuildableRailroadTrack* Track) const;
	bool IsCrossing(const AFGBuildableRailroadTrack* Track) const;

	// Returns the switch joint Connection belongs to, or INDEX_NONE.
	int32 GetJoint(const UFGRailroadTrackConnectionComponent* Connection) const;

	// Stop margin before a track end leaving through Exit. [cm]
	float GetStopMargin(const UFGRailroadTrackConnectionComponent* Exit) const;

	// Rebuilds the conflict model for every track in the world: switch joints (three or more tracks meeting) and the
	// game's overlapping tracks, made symmetric, that don't share a switch joint (crossings).
	void RebuildConflicts();

	bool HoldsTrack(const FATTrafficTrain& State, const AFGBuildableRailroadTrack* Track) const;
	static bool HoldsJoint(const FATTrafficTrain& State, int32 Joint);

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

	// Deceleration to plan a stop from Speed (the train's own speed if negative) with: a share of the weakest
	// full-brake deceleration measured at or below that speed, or a share of the consist's rating until measured.
	// [cm/s^2]
	static float GetPlanningDeceleration(const FATTrafficTrain& State, float Speed = -1.f);

	// Claim order score, lower first: seconds until the train's tail has passed the first track it still needs, minus
	// a share of the time it has waited.
	float GetClaimPriority(const FATTrafficTrain& State) const;

	// Acceleration estimate from the consist's tractive effort and mass, times Share. [cm/s^2]
	static float GetAcceleration(const FATTrafficTrain& State, float Share);

	// Time to cover Distance from Speed accelerating at Acceleration up to MaxSpeed. [s]
	static float TimeToCover(float Distance, float Speed, float Acceleration, float MaxSpeed);

	bool mEnabled = false;
	bool mRerouteEnabled = true;
	float mUpdateTimer = 0.f;
	// Seconds since the last traffic update.
	float mSinceUpdate = 0.f;
	float mSettingsTimer = 0.f;
	float mPhaseReportTimer = 0.f;
	// Settings as last read, to notice changes; unset until first read.
	TOptional<bool> mConfigTraffic;
	TOptional<bool> mConfigRerouting;
	TMap<TWeakObjectPtr<AFGTrain>, FATTrafficTrain> mTrains;

	TMap<const AFGBuildableRailroadTrack*, TArray<AFGBuildableRailroadTrack*>> mConflicts;
	TMap<const UFGRailroadTrackConnectionComponent*, int32> mJointOf;
	double mConflictsBuiltAt = -1e9;
	// Set when a track missing from the conflict model was looked up.
	mutable bool mConflictsStale = false;
	// The last deadlock logged, to log each one once.
	FString mLastDeadlock;
};
