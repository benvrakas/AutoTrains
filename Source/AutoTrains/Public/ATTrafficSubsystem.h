#pragma once

#include "CoreMinimal.h"
#include "Curves/RichCurve.h"
#include "Subsystem/ModSubsystem.h"
#include "UObject/ObjectKey.h"
#include "ATTrafficSubsystem.generated.h"

class AFGBuildableRailroadStation;
class AFGBuildableRailroadTrack;
class AFGRailroadVehicle;
class AFGTrain;
class UCurveFloat;
class UFGRailroadTrackConnectionComponent;

// Full-brake braking is measured and planned per speed band of this width, up to ATBrakeBands bands; the top band
// also covers every speed above it. [cm/s]
constexpr float ATBrakeBandWidth = 250.f;
constexpr int32 ATBrakeBands = 32;

// Where one vehicle of a train stands.
struct FATVehicleSpot
{
	AFGBuildableRailroadTrack* Track = nullptr;
	// Position of the vehicle's centre along the track, and half the vehicle's length. [cm]
	float Offset = 0.f;
	float HalfLength = 0.f;
};

// A vehicle's own top speed and, for a locomotive, its traction curve, maximum tractive effort and power draw, kept
// while the acceleration and top speed settings change them.
struct FATVehicleBase
{
	FRichCurve Traction;
	TWeakObjectPtr<UCurveFloat> TractionAsset;
	float MaxTractiveEffort = 0.f;
	FFloatInterval PowerConsumption;
	// Top speed the game allows the vehicle, and the speed at which its traction runs out. [km/h]
	float MaxVelocity = 0.f;
	float TractionTop = 0.f;
	// Traction force scale, traction speed scale, and the top speed set. [km/h]
	float Scale = 1.f;
	float SpeedScale = 1.f;
	float MaxVelocitySet = 0.f;
};

// One track on a train's route, from the track its front is on to its goal.
struct FATRouteStep
{
	AFGBuildableRailroadTrack* Track = nullptr;
	// Connection the train leaves the track through.
	UFGRailroadTrackConnectionComponent* Exit = nullptr;
	// Distance from the train's MU master to the end of this track when the route was read. [cm]
	float EndDistance = 0.f;
};

// The follow leader that sets a train's speed cap, for the log.
struct FATFollowInfo
{
	TWeakObjectPtr<AFGTrain> Leader;
	// Distance to the leader's nearest vehicle on the path, or to the first track it holds; the leader's braking
	// distance added to it. [cm]
	float Gap = -1.f;
	float LeaderBrake = 0.f;
	bool bVehicleOnPath = false;
};

// Traffic state of one train, rebuilt every update except its claims.
struct FATTrafficTrain
{
	TWeakObjectPtr<AFGTrain> Train;
	// Self-driving, not docked, following a path: this mod drives it.
	bool bManaged = false;
	TArray<FATRouteStep> Route;
	TSet<AFGBuildableRailroadTrack*> Occupied;
	TArray<FATVehicleSpot> Vehicles;
	// Switch joints with a vehicle of this train within their fouling zone.
	TSet<int32> OccupiedJoints;
	// Tracks ahead granted to this train, in route order; kept between updates until the train is past them.
	TArray<TWeakObjectPtr<AFGBuildableRailroadTrack>> Claims;
	// Switch joints the train's held tracks pass through.
	TSet<int32> ClaimedJoints;
	// Trains holding track this train has claimed too: it follows them and keeps its distance from them.
	TArray<TWeakObjectPtr<AFGTrain>> FollowLeaders;
	// World time each current follow leader became one, for the log. [s]
	TMap<TWeakObjectPtr<AFGTrain>, double> FollowLeaderSince;
	// Follow leaders whose vehicles on this train's path move away along it: ahead the same way, or turning off it.
	TSet<TWeakObjectPtr<AFGTrain>> LeadersMovingAway;
	// Trains this one counted as moving out of its way when it claimed track they hold. Apart from trains ahead of it
	// the same way, only these lead it.
	TSet<TWeakObjectPtr<AFGTrain>> FollowedTrains;
	float Length = 0.f;
	float Speed = 0.f;
	// Mass with cargo, from the vehicles. [kg]
	float Mass = 0.f;
	// How far the train's front is ahead of its MU master, which the ATC distances are measured from. [cm]
	float FrontAhead = 0.f;
	// The ATC path the route was read from and the path segment of its first step.
	const void* RoutePath = nullptr;
	int32 RouteFirstSegment = 0;
	float WaitingSeconds = 0.f;
	// Full-brake braking force measured on this train per speed band, as deceleration times mass so a train that
	// takes on cargo plans with less; 0 where not measured. [cm/s^2 kg]
	float BrakeForce[ATBrakeBands] = {};
	// Deceleration planned with in each speed band, from BrakeForce, the consist's mass and rating. [cm/s^2]
	float PlanDecel[ATBrakeBands] = {};
	// Speed at the previous update. [cm/s]
	float LastSpeed = 0.f;
	// Whether the train is on full brake, and whether it was at the previous update.
	bool bFullBrake = false;
	bool bWasFullBrake = false;
	// Whether the mod is taking speed off the train directly, and whether it did since the previous update.
	bool bForcedBrake = false;
	bool bForcedSinceUpdate = false;
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
	// ATC path and path segment of the authority track.
	const void* AuthorityPath = nullptr;
	int32 AuthoritySegment = INDEX_NONE;
	bool bAuthorityToGoal = false;
	// How far before the end of the authority track the train stops: more when a switch is there. [cm]
	float StopMargin = 1000.f;
	// While the train has waited long: the tracks and switches it needs to its next safe stop, which other trains leave
	// free for it.
	TSet<const AFGBuildableRailroadTrack*> RequestTracks;
	TSet<int32> RequestJoints;
	// Why the claims stop where they do, for the log and /at traffic.
	FString BlockedReason;
	TWeakObjectPtr<AFGTrain> BlockedBy;

	// Space-time planner. The station the route leads to.
	TWeakObjectPtr<AFGBuildableRailroadStation> GoalStation;
	// When the plan has the front enter each route step's track and the switch before it (world seconds; a large value
	// where nothing is booked).
	TArray<double> PlanTrackEnter;
	TArray<double> PlanJointEnter;
	// The plan's first wait still ahead: where the front stands [cm], when it moves off (world seconds, -1 for none), and
	// the train it waits for.
	float PlanWaitAt = -1.f;
	double PlanDepart = -1.0;
	TWeakObjectPtr<AFGTrain> PlanWaitFor;
	// When the plan has the front reach the goal's track, and what it was at the last route search (world seconds, -1
	// unknown).
	double PlanArrival = -1.0;
	double SearchedArrival = -1.0;
	// Why the last plan failed and which train it is committed against, empty if neither; a summary for /at traffic.
	FString PlanFailure;
	FString PlanCommitted;
	FString PlanNote;
	// Docking: whether the train was docked at the previous update, whether it was seen undocked since traffic control
	// started (so a dock's start is known), when the current dock started (world seconds), and the station.
	bool bWasDocked = false;
	bool bSeenUndocked = false;
	double DockedAt = -1.0;
	TWeakObjectPtr<AFGBuildableRailroadStation> DockStation;
	// The next leg from the station the train is docked at, as the game's pathfinder finds it from the locomotive the
	// train would leave with: route, front ahead of that locomotive [cm], the station it leads to, and when it was worked
	// out (world seconds).
	TArray<FATRouteStep> PredictedRoute;
	float PredictedFrontAhead = 0.f;
	TWeakObjectPtr<AFGBuildableRailroadStation> PredictedStation;
	double PredictedAt = -1e9;
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

	// Turns the space-time planner on or off (off by default): it drives every self-driving train instead of the claim
	// rules.
	FString SetPlannerControl(bool bEnabled);
	bool IsPlannerControl() const { return mPlannerControl; }

	// Returns true if trains should pass through each other instead of colliding (traffic control and the setting on).
	static bool ShouldPhaseThrough();

	// Called by the collision hook each time a train contact was let through, with the tracks the two vehicles are on.
	static void CountPhaseThrough(AFGBuildableRailroadTrack* Track, AFGBuildableRailroadTrack* OtherTrack);

	// Returns true if this mod is driving Train.
	bool IsManaged(const AFGTrain* Train) const;

	// Called after the game's AFGTrain::TickAtc for managed trains; caps the target speed to the train's authority.
	void ApplyAuthority(AFGTrain* Train, float DeltaSeconds);

	// Called after ApplyAuthority: applies the top speed setting to a self-driving train.
	void ApplySpeedSettings(AFGTrain* Train, float DeltaSeconds);

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

	// Sets State's request: the tracks and switches from route step First to its next safe stop, while it has waited
	// long for another train.
	void UpdateRequest(FATTrafficTrain& State, int32 First) const;

	// Returns the train that has waited longer than State and needs Track (or a track crossing it), or Joint, to get
	// to its next safe stop, unless State is in its way or it waits for or follows State; null if none.
	const FATTrafficTrain* FindRequester(const FATTrafficTrain& State, const AFGBuildableRailroadTrack* Track, int32 Joint) const;
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

	// Returns the latest Holder, at Speed, has travelled ClearDistance if it never stops before the end of its
	// authority (or its goal), or -1 if that comes first. [s]
	float GetLatestClearSeconds(const FATTrafficTrain& Holder, float ClearDistance, float Speed) const;

	// Returns how far Holder's front still travels until its tail has cleared Track (or Joint's fouling zone). [cm]
	float GetClearDistance(const FATTrafficTrain& Holder, const AFGBuildableRailroadTrack* Track, int32 Joint) const;

	// Returns how far Holder still travels until none of its vehicles is on Track, a track off its route, or reaches
	// onto it past an end (or is in Joint's fouling zone); -1 if none does. [cm]
	float GetTailClearDistance(const FATTrafficTrain& Holder, const AFGBuildableRailroadTrack* Track, int32 Joint) const;

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
	bool IsSameWayLeader(const FATTrafficTrain& Self, const FATTrafficTrain& Holder, int32 FromIndex, const AFGBuildableRailroadTrack* Track, int32 Joint) const;

	// Returns true if Holder's authority (or its goal) takes its tail past Track (or Joint's fouling zone), so a train
	// following it there never has to stop behind it on the way.
	bool IsClearedPast(const FATTrafficTrain& Holder, const AFGBuildableRailroadTrack* Track, int32 Joint) const;

	// Drops Self's claims from the first one shared with a train that has stopped.
	void RevalidateFollowClaims(FATTrafficTrain& State);

	// Sets State's follow leaders: leaders holding track it has claimed, and trains it doesn't drive standing on it.
	void UpdateFollowLeaders(FATTrafficTrain& State) const;

	// Returns how far Train may go behind its follow leaders: to the nearest vehicle of one along its path (or the
	// first track along it one holds), plus the braking distance of a leader moving on ahead the same way, counted
	// with the strongest braking a sample counts for; -1 if it follows none. OutHardLimit is the same with the
	// leaders' planning braking, for full brake. OutPaceSpeed is the highest speed at which Train reaches the
	// crossings of leaders clearing them first only after they have, MAX_flt if none. [cm]
	float GetFollowLimit(const AFGTrain* Train, const FATTrafficTrain& State, float& OutHardLimit, float& OutPaceSpeed, FATFollowInfo* OutInfo = nullptr) const;

	// Returns the speed at which Train, Start away from where Leader crosses its route on Track (or passes Joint),
	// gets there with some slack after the latest Leader, able to get past it, can have cleared it; -1 if Leader may
	// not clear it or Train at its speed now could get there first. [cm/s]
	float GetCrossingPaceSpeed(const AFGTrain* Train, const FATTrafficTrain& Leader, const AFGBuildableRailroadTrack* Track, int32 Joint, float Start) const;

	// Returns Braking, but no further than Leader may still travel: the end of its authority or its goal. [cm]
	float GetLeaderTravel(const FATTrafficTrain& Leader, float Braking) const;

	// Returns the distance from Train's front along its path to the nearest vehicle of Other ahead of it, or -1 if
	// none is within GapSearchDistance. [cm]
	float GetGapToVehicles(const AFGTrain* Train, const FATTrafficTrain& State, const FATTrafficTrain& Other) const;

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

	// Fouling zone of the switch at Connection along its track, and the longest of a joint's tracks. [cm]
	float GetFoulDistance(const UFGRailroadTrackConnectionComponent* Connection) const;
	float GetJointFoulDistance(int32 Joint) const;

	// Stop margin before a track end leaving through Exit. [cm]
	float GetStopMargin(const UFGRailroadTrackConnectionComponent* Exit) const;

	// Rebuilds the conflict model for every track in the world: switch joints (three or more tracks meeting) and the
	// game's overlapping tracks, made symmetric, that don't share a switch joint (crossings).
	void RebuildConflicts();

	bool HoldsTrack(const FATTrafficTrain& State, const AFGBuildableRailroadTrack* Track) const;
	static bool HoldsJoint(const FATTrafficTrain& State, int32 Joint);

	// When another train is on a track or one crossing it, seen from now: from Start to End, leaving it through Exit
	// (null if unknown); whether on the track itself, standing there, and waiting for the train being routed. [s]
	struct FOccupancy
	{
		float Start = 0.f;
		float End = 0.f;
		const UFGRailroadTrackConnectionComponent* Exit = nullptr;
		bool bOnTrack = false;
		bool bStanding = false;
		bool bWaitsForSelf = false;
	};
	// What other trains than Self do on each track: where they stand, where they pass when, and where they hold it.
	struct FCongestion
	{
		TArray<FOccupancy> Occupancies;
		// Exit connections of other trains that hold the track on their route.
		TArray<const UFGRailroadTrackConnectionComponent*> HeldExits;
	};
	void BuildCongestion(const FATTrafficTrain& Self, TMap<const AFGBuildableRailroadTrack*, FCongestion>& Out) const;

	// How a train is timed along a route: cruising speed and length [cm/s] [cm], the time a stop costs over driving
	// on [s], and the acceleration and service braking it plans with [cm/s^2].
	struct FRouteSpeed
	{
		float Cruise = 500.f;
		float Length = 0.f;
		float StopLoss = 0.f;
		float Accel = 100.f;
		float Brake = 100.f;
	};
	FRouteSpeed GetRouteSpeed(const FATTrafficTrain& State) const;

	// Seconds a train needs over Track, leaving through Exit, getting there Arrival seconds from now: driving it, the
	// wait for other trains on it, and the time stopping costs; bNewRoute adds driving against another train's held
	// track.
	static float GetTrackSeconds(const TMap<const AFGBuildableRailroadTrack*, FCongestion>& Congestion, const AFGBuildableRailroadTrack* Track,
		const UFGRailroadTrackConnectionComponent* Exit, float Arrival, const FRouteSpeed& Speed, bool bNewRoute);

	// Looks for a route to State's goal, from the end of its claims, that's clearly faster than its current one once
	// other trains are counted, and switches to it.
	void ConsiderReroute(FATTrafficTrain& State);

	// Sets State.PlanDecel: a share of the deceleration measured in each band; a band without a measurement takes the
	// nearest measured band above it (braking is weaker at higher speed), or above every measured band the highest
	// one's but no more than a share of the air brakes' force over the train's mass with cargo; a train never
	// measured plans with that share of the air brakes.
	void RefreshBrakePlan(FATTrafficTrain& State) const;

	// Deceleration planned with at Speed (the train's own speed if negative). [cm/s^2]
	static float GetPlanningDeceleration(const FATTrafficTrain& State, float Speed = -1.f);

	// Distance State needs to stop from Speed: the brakes building up, then each band's planned deceleration. [cm]
	static float GetStoppingDistance(const FATTrafficTrain& State, float Speed);

	// Highest speed from which State stops within Distance, braking at Share of its planned deceleration. [cm/s]
	static float GetAllowedSpeed(const FATTrafficTrain& State, float Distance, float Share = 1.f);

	// Claim order score, lower first: seconds until the train's tail has passed the first track it still needs, minus
	// a share of the time it has waited.
	float GetClaimPriority(const FATTrafficTrain& State) const;

	// Acceleration estimate from the consist's tractive effort and mass, times Share. [cm/s^2]
	static float GetAcceleration(const FATTrafficTrain& State, float Share);

	// Time to cover Distance from Speed accelerating at Acceleration up to MaxSpeed. [s]
	static float TimeToCover(float Distance, float Speed, float Acceleration, float MaxSpeed);

	// Scales the traction force and power draw of every locomotive of Train by Scale and, for a TopSpeed above the
	// speed at which a locomotive's traction runs out, stretches its traction curve to TopSpeed, with the power draw
	// scaled the same, and lets every vehicle run that fast (Scale 1 and TopSpeed 0 give them back their own); then
	// has the train recalculate its consist. [km/h]
	void ApplyLocomotivePower(AFGTrain* Train, float Scale, float TopSpeed);
	static void SetVehicleScale(AFGRailroadVehicle& Vehicle, FATVehicleBase& Base, float Scale, float SpeedScale, float MaxVelocity);
	// Gives every changed vehicle its own traction, power and top speed back.
	void RestoreLocomotivePower();

	// Space-time planner (ATPlanner.cpp). A resource is a track (conflicting with the tracks crossing it) or a switch
	// joint's fouling zone; a booking holds it for a train from Enter to Clear (world seconds).
	struct FResourceKey
	{
		const AFGBuildableRailroadTrack* Track = nullptr;
		int32 Joint = INDEX_NONE;
		bool operator==(const FResourceKey& Other) const { return Track == Other.Track && Joint == Other.Joint; }
		friend uint32 GetTypeHash(const FResourceKey& Key) { return HashCombine(::GetTypeHash(Key.Track), ::GetTypeHash(Key.Joint)); }
	};
	enum class EBookingKind : uint8
	{
		// A self-driving train's plan.
		Moving,
		// A train this mod doesn't drive, where it stands.
		Standing,
		// A docked train's stay and its next leg.
		Predicted,
	};
	struct FBooking
	{
		// Compared only, never followed: the train may be gone.
		const AFGTrain* Owner = nullptr;
		double Enter = 0.0;
		double Clear = 0.0;
		// A track's exit connection; for a switch, the exit of the track after it and the connection it is entered from.
		// Null if unknown.
		const UFGRailroadTrackConnectionComponent* Exit = nullptr;
		const UFGRailroadTrackConnectionComponent* Via = nullptr;
		EBookingKind Kind = EBookingKind::Moving;
		// The owner stood on it or couldn't stop short of it when booking.
		bool bHeld = false;
	};
	// One resource a train passes on its route: the front position at which it enters it, the one at which its tail
	// has cleared it [cm along the route from the front], the times [world s], and the route step it belongs to
	// (INDEX_NONE for what the tail holds behind the route).
	struct FPass
	{
		FResourceKey Key;
		const UFGRailroadTrackConnectionComponent* Exit = nullptr;
		const UFGRailroadTrackConnectionComponent* Via = nullptr;
		float EnterAt = 0.f;
		float ClearAt = 0.f;
		double Enter = 0.0;
		double Clear = 0.0;
		int32 RouteIndex = INDEX_NONE;
	};
	// A wait in a plan: the front stands at At [cm along the route] until Until.
	struct FWait
	{
		float At = 0.f;
		double Until = 0.0;
	};
	struct FPlan
	{
		TArray<FWait> Waits;
		TArray<FPass> Passes;
		// The train whose booking caused the first wait; why planning failed, if it did; a train booked on a resource
		// this one can no longer stop short of, if any, and that resource.
		const AFGTrain* WaitFor = nullptr;
		FString Failure;
		const AFGTrain* CommittedWith = nullptr;
		FString CommittedAt;
		// Distance within which the train can't stop any more. [cm]
		float Committed = 0.f;
	};
	// The resources an owner has bookings on, to remove them again.
	struct FOwnerBookings
	{
		const AFGTrain* Owner = nullptr;
		TSet<FResourceKey> Keys;
	};

	static float GetSafetyMarginSeconds();
	// Plans and books every train in turn, then gives each self-driving train its movement authority from the bookings.
	void UpdatePlannedTraffic(float DeltaSeconds);
	// Lists the resources along State's route with the front positions at which it enters and clears them, plus those
	// its tail still holds behind the front (entered already).
	void CollectPasses(const FATTrafficTrain& State, TArray<FPass>& Out) const;
	// Times Passes for State's motion with Waits, from Now; the train stays GoalDwell at its goal. [s]
	void TimePasses(const FATTrafficTrain& State, const FRouteSpeed& Speed, const TArray<FWait>& Waits, double Now, double GoalDwell, TArray<FPass>& Passes) const;
	// Plans State around the other trains' bookings (with bHeldOnly, of self-driving trains only what they hold),
	// starting from FixedWaits: each conflict adds a wait at the last place to stand before it; with no such place the
	// train stands for good before it.
	void PlanTrain(const FATTrafficTrain& State, double Now, const TArray<FWait>& FixedWaits, double GoalDwell, bool bHeldOnly, FPlan& Plan) const;
	// Returns another train's booking that Pass conflicts with on its resource (or a track crossing it), or null,
	// and the time Pass would have to enter instead: after it has cleared, or, following it the same way, a headway
	// after it both enters and clears. With bHeldOnly a self-driving train's booking counts only where it is held.
	const FBooking* FindConflict(const AFGTrain* Self, const FPass& Pass, double Headway, bool bHeldOnly, double& OutNeedEnter) const;
	// Books Passes for State; those entered within HeldWithin of the front count as held. [cm]
	void Book(const FATTrafficTrain& State, const TArray<FPass>& Passes, EBookingKind Kind, float HeldWithin);
	// Books what State stands on, from Now until Until.
	void BookStanding(const FATTrafficTrain& State, double Now, double Until, EBookingKind Kind);
	void RemoveBookings(const AFGTrain* Owner);
	// Plans and books a self-driving train, and keeps its booked times per route step.
	void PlanMoving(FATTrafficTrain& State, double Now);
	// Books a docked train's stay until its expected departure, then its predicted next leg.
	void PlanDocked(FATTrafficTrain& State, double Now);
	// Works out State's next leg from the station it is docked at, from its timetable and the game's pathfinder.
	void PredictDeparture(FATTrafficTrain& State, double Now);
	// Notes docking and departure; a dock seen from its start teaches the dwell.
	void TrackDocking(FATTrafficTrain& State, AFGTrain* Train, double Now);
	// Expected stay of Train at Station: its own last one there, else any train's last one there, else a default. [s]
	float GetExpectedDwell(const AFGTrain* Train, const AFGBuildableRailroadStation* Station) const;
	// How far State's front is ahead of the start of Route, from its vehicles. [cm]
	float ComputeFrontAhead(const FATTrafficTrain& State, const TArray<FATRouteStep>& Route) const;
	// Sets State's claims: the route resources from its front up to its stopping distance plus lookahead that it may
	// enter now: booked for it, no other train booked there first or standing there, except one it follows the same way.
	void GrantFromPlan(FATTrafficTrain& State, double Now);
	// Returns true if Self may enter Key now with its booking entering at Enter; otherwise sets the train in the way and
	// why. A train it follows the same way is added to its followed trains.
	bool CanEnter(FATTrafficTrain& Self, const FResourceKey& Key, double Enter, const UFGRailroadTrackConnectionComponent* Via,
		const UFGRailroadTrackConnectionComponent* Exit, double Now, const FATTrafficTrain*& OutBlocker, FString& OutWhy) const;
	// Searches State's fastest route through the bookings when due, and takes it if clearly faster.
	void ConsiderPlannedReroute(FATTrafficTrain& State, double Now, float DeltaSeconds);
	// Seconds the route search counts for getting over Track, leaving through Exit, reached Arrival seconds from Now:
	// the wait for the earliest entry no other train's booking of it, a track crossing it or the switch before it
	// conflicts with, the time stopping costs, and driving it.
	float GetSearchStepSeconds(const AFGTrain* Self, const AFGBuildableRailroadTrack* Track, const UFGRailroadTrackConnectionComponent* Exit, double Now,
		float Arrival, const FRouteSpeed& Speed) const;

	TMap<FResourceKey, TArray<FBooking>> mBookings;
	TMap<FObjectKey, FOwnerBookings> mOwnerBookings;
	// The traffic state of each train by its pointer, for the current update.
	TMap<const AFGTrain*, FATTrafficTrain*> mStateOf;
	// Last dwell of each train at each station, and the last one by any train at each station. [s]
	TMap<TPair<FObjectKey, FObjectKey>, float> mDwell;
	TMap<FObjectKey, float> mStationDwell;
	// Planner counters since the last summary.
	int32 mPlans = 0;
	int32 mPlanWaits = 0;
	int32 mPlanFailures = 0;
	int32 mPlanCommitted = 0;
	int32 mPlanReroutes = 0;
	int32 mPlanPredictions = 0;
	int32 mPlanUpdates = 0;
	double mPlanSeconds = 0.0;
	double mPlanSummaryAt = 0.0;

	// Vehicles the acceleration and top speed settings have changed.
	TMap<TWeakObjectPtr<AFGRailroadVehicle>, FATVehicleBase> mVehicleBase;

	bool mEnabled = false;
	bool mRerouteEnabled = true;
	bool mPlannerControl = false;
	float mUpdateTimer = 0.f;
	// Seconds since the last traffic update.
	float mSinceUpdate = 0.f;
	float mSettingsTimer = 0.f;
	float mPhaseReportTimer = 0.f;
	// Settings as last read, to notice changes; unset until first read.
	TOptional<bool> mConfigTraffic;
	TOptional<bool> mConfigRerouting;
	TOptional<bool> mConfigPlanner;
	TMap<TWeakObjectPtr<AFGTrain>, FATTrafficTrain> mTrains;

	TMap<const AFGBuildableRailroadTrack*, TArray<AFGBuildableRailroadTrack*>> mConflicts;
	TMap<const UFGRailroadTrackConnectionComponent*, int32> mJointOf;
	TMap<const UFGRailroadTrackConnectionComponent*, float> mFoulOf;
	TMap<int32, float> mJointFoul;
	double mConflictsBuiltAt = -1e9;
	// Set when a track missing from the conflict model was looked up.
	mutable bool mConflictsStale = false;
	// The last deadlock logged, to log each one once.
	FString mLastDeadlock;
};
