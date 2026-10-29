#pragma once

#include "CoreMinimal.h"
#include "ATConfig.h"
#include "FGTrainDockingRules.h"
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
struct FATSippCycle;
struct FATSippGraph;
struct FATSippResult;
struct FATSippTrain;

// Full-brake braking is measured and planned per speed band of this width, up to ATBrakeBands bands; the top band
// also covers every speed above it. [cm/s]
constexpr float ATBrakeBandWidth = 250.f;
constexpr int32 ATBrakeBands = 32;

// Where one vehicle of a train stands.
// A crossing zone on a track: the crossing and the stretch of the track it covers, from the end of the track's
// connection 0. [cm]
struct FATZone
{
	int32 Crossing = INDEX_NONE;
	float From = 0.f;
	float To = 0.f;
};

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
	// Switch joints with a vehicle of this train within their fouling zone, and crossing zones with one on them.
	TSet<int32> OccupiedJoints;
	TSet<int32> OccupiedCrossings;
	// Tracks ahead granted to this train, in route order; kept between updates until the train is past them.
	TArray<TWeakObjectPtr<AFGBuildableRailroadTrack>> Claims;
	// Switch joints the train's held tracks pass through, and crossing zones on them up to where its grant ends.
	TSet<int32> ClaimedJoints;
	TSet<int32> ClaimedCrossings;
	// Where its grant ends on its last held track, as a distance before that track's end; -1 at the track's own stop
	// margin. [cm]
	float AuthorityCut = -1.f;
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
	// Slope the way it drives, positive uphill: under its body (from its rear car's centre to its front car's), and from
	// its rear car to its route a few hundred metres ahead. [cm/cm]
	float Grade = 0.f;
	float GradeAhead = 0.f;
	// Heights of its front and rear cars' centres, and how far ahead of its front they are (negative). [cm]
	float FrontZ = 0.f;
	float RearZ = 0.f;
	float FrontZAt = 0.f;
	float RearZAt = 0.f;
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
	// Speed the train runs at when nothing holds it back, as seen; 0 until seen. [cm/s]
	float CruiseSeen = 0.f;
	// Traction measured per speed band, as acceleration with the slope taken out times mass, 0 where not measured; and
	// the acceleration planned with in each band on level track. [cm/s^2 kg] [cm/s^2]
	float TractionForce[ATBrakeBands] = {};
	float PlanAccel[ATBrakeBands] = {};
	// Its locomotives' full traction at low speed without the hill help, 0 until known [cm/s^2 kg]; the traction
	// multiple the hill help gives it (1 for none), and since when (world s).
	float FullTraction = 0.f;
	float HillBoost = 1.f;
	double HillSince = 0.0;
	// Blocked key of the last wait before a climb, so each is logged once.
	FString ClimbWaitKey;
	// Whether the train is on full brake, and whether it was at the previous update.
	bool bFullBrake = false;
	bool bWasFullBrake = false;
	// Whether the mod is taking speed off the train directly, and whether it did since the previous update.
	bool bForcedBrake = false;
	bool bForcedSinceUpdate = false;
	// Speed forced braking left the train at, the last tick it acted. [cm/s]
	float ForcedSpeed = 0.f;
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
	// Why the claims stop where they do, for the log and /at traffic, and what of it is logged on change (the planner's
	// reasons carry times).
	FString BlockedReason;
	FString BlockedKey;
	TWeakObjectPtr<AFGTrain> BlockedBy;

	// Space-time planner. The station the route leads to.
	TWeakObjectPtr<AFGBuildableRailroadStation> GoalStation;
	// When the train set off for the station of its current leg (world seconds, -1 before), and that station.
	double LegStart = -1.0;
	TWeakObjectPtr<AFGBuildableRailroadStation> LegGoal;
	// Place in this update's planning order of self-driving trains, first 0; INDEX_NONE for other trains.
	int32 PlanRank = INDEX_NONE;
	// Whether a planning cycle has planned it since it was last driven by the game or a player.
	bool bHasPlan = false;
	// Trains it was planned before on this leg after its plan failed behind them; they don't go back before it.
	TSet<TWeakObjectPtr<AFGTrain>> MovedAheadOf;
	// When the plan has the front enter each route track and switch (world seconds), and which of those bookings are
	// locked.
	TMap<const AFGBuildableRailroadTrack*, double> PlanTrackEnter;
	TMap<int32, double> PlanJointEnter;
	TSet<const AFGBuildableRailroadTrack*> PlanTrackLocked;
	TSet<int32> PlanJointLocked;
	TMap<int32, double> PlanCrossingEnter;
	TSet<int32> PlanCrossingLocked;
	// The plan's first wait still ahead: PlanWaitShort short of the end of the track left through PlanWaitExit, or
	// where the train stands (bPlanWaitHere); when it moves off (world seconds, -1 for none), and the train it waits
	// for. PlanWaitAt: how far ahead of the front that is, as of the last grant [cm].
	TWeakObjectPtr<UFGRailroadTrackConnectionComponent> PlanWaitExit;
	float PlanWaitShort = 0.f;
	bool bPlanWaitHere = false;
	float PlanWaitAt = -1.f;
	double PlanDepart = -1.0;
	TWeakObjectPtr<AFGTrain> PlanWaitFor;
	// When the plan has the front reach the goal's track end, and what it was at the last route search (world seconds,
	// -1 unknown).
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
	// The current dock's rules, and whether its freight platforms move cargo: 1 one has, 0 all finished without, -1 not
	// known yet.
	FTrainDockingRuleSet DockRules;
	int32 DockTransfer = -1;
	// The next leg from the station the train is docked at, as the game's pathfinder finds it from the locomotive the
	// train would leave with: route, front ahead of that locomotive [cm], the station it leads to, and when it was worked
	// out (world seconds).
	TArray<FATRouteStep> PredictedRoute;
	float PredictedFrontAhead = 0.f;
	TWeakObjectPtr<AFGBuildableRailroadStation> PredictedStation;
	int32 PredictedStop = INDEX_NONE;
	double PredictedAt = -1e9;
	// The route step of each track on the route (first time) and of each switch, by track and by joint.
	TMap<const AFGBuildableRailroadTrack*, int32> RouteIndexOf;
	TMap<int32, int32> JointIndexOf;
	// In a cycle of trains waiting for each other.
	bool bDeadlocked = false;
	// Passing through a deadlock: until when at most (world seconds, -1 if not) and how far it still drives so. [cm]
	double GhostUntil = -1.0;
	float GhostLeft = 0.f;
	// The track or switch the train waits at.
	const AFGBuildableRailroadTrack* BlockedAtTrack = nullptr;
	int32 BlockedAtJoint = INDEX_NONE;
	int32 BlockedAtCrossing = INDEX_NONE;
};

// Server only. Train traffic without signals, planned in space and time (ATPlanner.cpp): every train books the tracks
// and switch fouling zones on its way as time slots around the others' bookings and routes itself the fastest way
// through them; a train is granted (claims) a track or switch only when nobody else stands on or holds it and no other
// train is booked there before it. Each train's speed is limited so it can always stop before the end of its claims,
// with the game's catch as the backstop.
UCLASS()
class AUTOTRAINS_API AATTrafficSubsystem : public AModSubsystem
{
	GENERATED_BODY()

public:
	AATTrafficSubsystem();

	static AATTrafficSubsystem* Get(const UObject* WorldContext);

	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	FString SetEnabled(bool bEnabled);

	// Turns congestion rerouting on or off (on by default while traffic control is on).
	FString SetRerouteEnabled(bool bEnabled);


	// Returns true if trains should pass through each other instead of colliding (traffic control and the setting on).
	static bool ShouldPhaseThrough();

	// Called by the collision hook each time a train contact was let through, with the tracks the two vehicles are on.
	static void CountPhaseThrough(AFGBuildableRailroadTrack* Track, AFGBuildableRailroadTrack* OtherTrack);

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
	// Applies the Mods menu settings; a changed switch overrides what /at set for this session.
	void ApplySettings();
	// From the settings: how far trains stop short of track they haven't been given [cm], and the share of their planned
	// braking normal stops use.
	float GetStopDistance() const { return mSettings.StopDistanceMetres * 100.f; }
	float GetNormalBrakingShare() const { return mSettings.NormalBrakingPercent / 100.f; }
	void UpdateTraffic(float DeltaSeconds);
	void ReadTrain(FATTrafficTrain& State, AFGTrain* Train) const;
	void ReleasePassedClaims(FATTrafficTrain& State) const;

	void UpdateAuthority(FATTrafficTrain& State) const;
	void RefreshClaimedJoints(FATTrafficTrain& State) const;
	void LogDeadlocks();
	// Lets the trains of a deadlock that lasted Seconds pass through each other, and records the fault.
	void GhostThrough(const TArray<const FATTrafficTrain*>& Chain, double Seconds);
	// Logs what a long-waiting train waits for: its booking and every booking at that resource, with their trains.
	void LogWaitDump(const FATTrafficTrain& State, double Now) const;

	// Returns how far Holder still travels until none of its vehicles is on Track, a track off its route, or reaches
	// onto it past an end (or is in Joint's fouling zone); -1 if none does. [cm]
	float GetTailClearDistance(const FATTrafficTrain& Holder, const AFGBuildableRailroadTrack* Track, int32 Joint) const;

	// Returns 1 if Other is ahead on Self's route going the same way, -1 if it's on Self's route the other way or
	// behind Self on its track, 0 if the track Other's master is on isn't on Self's route.
	int32 GetRouteOrder(const FATTrafficTrain& Self, const FATTrafficTrain& Other) const;

	// Returns true if Other leads Self: it's ahead on Self's route going the same way, or Self followed it when
	// claiming. A train following Self is never its leader.
	bool IsLeaderOf(const FATTrafficTrain& Self, const FATTrafficTrain& Other) const;

	// Sets State's follow leaders: leaders holding track it has claimed, and trains it doesn't drive standing on it.
	void UpdateFollowLeaders(FATTrafficTrain& State) const;

	// Returns how far Train may go behind its follow leaders: to the nearest vehicle of one along its path (or the
	// first track along it one holds), plus the braking distance of a leader moving on ahead the same way, counted
	// with the strongest braking a sample counts for; -1 if it follows none. OutHardLimit is the same with the
	// leaders' planning braking, for full brake. [cm]
	float GetFollowLimit(const AFGTrain* Train, const FATTrafficTrain& State, float& OutHardLimit, FATFollowInfo* OutInfo = nullptr) const;

	// Returns Braking, but no further than Leader may still travel: the end of its authority or its goal. [cm]
	float GetLeaderTravel(const FATTrafficTrain& Leader, float Braking) const;

	// Returns the distance from Train's front along its path to the nearest vehicle of Other ahead of it, or -1 if
	// none is within GapSearchDistance. [cm]
	float GetGapToVehicles(const AFGTrain* Train, const FATTrafficTrain& State, const FATTrafficTrain& Other) const;

	// Returns the distance from Train's front to the start of Track along its path (negative once on it), or a large
	// negative number if Track isn't on it. [cm]
	float GetDistanceToTrackStart(const AFGTrain* Train, const FATTrafficTrain& State, const AFGBuildableRailroadTrack* Track) const;

	// The crossing zones on Track.
	const TArray<FATZone>& GetZones(const AFGBuildableRailroadTrack* Track) const;
	// Where Zone lies along Track driven towards Exit, from the track's start that way. [cm]
	static void GetZoneAlong(const FATZone& Zone, const AFGBuildableRailroadTrack* Track, const UFGRailroadTrackConnectionComponent* Exit, float& OutFrom, float& OutTo);

	// Returns the switch joint Connection belongs to, or INDEX_NONE.
	int32 GetJoint(const UFGRailroadTrackConnectionComponent* Connection) const;

	// Fouling zone of the switch at Connection along its track, and the longest of a joint's tracks. [cm]
	float GetFoulDistance(const UFGRailroadTrackConnectionComponent* Connection) const;
	float GetJointFoulDistance(int32 Joint) const;
	// The track clearance setting. [cm]
	float GetClearance() const { return mSettings.SwitchClearanceMetres * 100.f; }

	// Stop margin before a track end leaving through Exit. [cm]
	float GetStopMargin(const UFGRailroadTrackConnectionComponent* Exit) const;

	// Rebuilds the conflict model for every track in the world: switch joints (three or more tracks meeting) and the
	// game's overlapping tracks, made symmetric, that don't share a switch joint (crossings).
	void RebuildConflicts();

	bool HoldsTrack(const FATTrafficTrain& State, const AFGBuildableRailroadTrack* Track) const;
	static bool HoldsJoint(const FATTrafficTrain& State, int32 Joint);
	static bool HoldsCrossing(const FATTrafficTrain& State, int32 Crossing);

	// How a train is timed along a route: cruising speed and length [cm/s] [cm], and the acceleration and service
	// braking it plans with [cm/s^2].
	struct FRouteSpeed
	{
		float Cruise = 500.f;
		float Length = 0.f;
		float Accel = 100.f;
		float Brake = 100.f;
	};
	FRouteSpeed GetRouteSpeed(const FATTrafficTrain& State) const;

	// Sets State.PlanDecel: a share of the deceleration measured in each band; a band without a measurement takes the
	// nearest measured band above it (braking is weaker at higher speed), or above every measured band the highest
	// one's but no more than a share of the air brakes' force over the train's mass with cargo; a train never
	// measured plans with that share of the air brakes.
	void RefreshBrakePlan(FATTrafficTrain& State) const;
	// Sets State.PlanAccel: measured bands as measured; the others from its locomotives' traction curves at that speed,
	// scaled by the share of them the train was seen to use in measured bands (half until measured).
	void RefreshTractionPlan(FATTrafficTrain& State) const;
	// Sets State.HillBoost: extra traction while the train, wanting to go on and slow, has less traction than the slope
	// under it pulls back with, until that slope is one it manages on its own.
	void UpdateHillHelp(FATTrafficTrain& State, double Now) const;

	// Deceleration planned with at Speed (the train's own speed if negative). [cm/s^2]
	static float GetPlanningDeceleration(const FATTrafficTrain& State, float Speed = -1.f);

	// Distance State needs to stop from Speed: the brakes building up, then each band's planned deceleration. [cm]
	static float GetStoppingDistance(const FATTrafficTrain& State, float Speed);

	// Highest speed from which State stops within Distance, braking at Share of its planned deceleration. [cm/s]
	static float GetAllowedSpeed(const FATTrafficTrain& State, float Distance, float Share = 1.f);

	// Acceleration estimate from the consist's tractive effort and mass, times Share. [cm/s^2]
	static float GetAcceleration(const FATTrafficTrain& State, float Share);

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
		int32 Crossing = INDEX_NONE;
		bool operator==(const FResourceKey& Other) const { return Track == Other.Track && Joint == Other.Joint && Crossing == Other.Crossing; }
		friend uint32 GetTypeHash(const FResourceKey& Key) { return HashCombine(HashCombine(::GetTypeHash(Key.Track), ::GetTypeHash(Key.Joint)), ::GetTypeHash(Key.Crossing)); }
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
		// Locked: the owner stands on it or couldn't stop short of it at service braking; nobody books ahead of it, and it
		// goes first. Held: the owner holds it from its last grant; others may still fit in ahead of it.
		bool bLocked = false;
		bool bHeld = false;
		// Committed: on the owner's way from what it holds or can't stop short of to the next place it can stand without
		// holding up a train planned before it; it can't give way inside that stretch.
		bool bCommitted = false;
	};
	// One resource a train passes on its route: the front position at which it enters it, the one at which its tail
	// has cleared it [cm along the route from the front], and the route step it belongs to (INDEX_NONE for what the tail
	// holds behind the route).
	struct FPass
	{
		FResourceKey Key;
		const UFGRailroadTrackConnectionComponent* Exit = nullptr;
		const UFGRailroadTrackConnectionComponent* Via = nullptr;
		float EnterAt = 0.f;
		float ClearAt = 0.f;
		int32 RouteIndex = INDEX_NONE;
	};
	// The resources an owner has bookings on, to remove them again.
	struct FOwnerBookings
	{
		const AFGTrain* Owner = nullptr;
		TSet<FResourceKey> Keys;
	};

	static float GetSafetyMarginSeconds();
	// Applies a finished planning cycle, gives each self-driving train its movement authority from the bookings, and
	// starts the next cycle.
	void UpdatePlannedTraffic(float DeltaSeconds);
	// Lists the resources along State's route with the front positions at which it enters and clears them, plus those
	// its tail still holds behind the front (entered already).
	void CollectPasses(const FATTrafficTrain& State, TArray<FPass>& Out) const;
	// Starts a planning cycle (ATSipp) on worker threads: every train in turn, with the bookings as they are now;
	// bWait runs it on this thread and applies it at once.
	void StartPlanCycle(double Now, float DeltaSeconds, bool bWait);
	// Takes the finished cycle's bookings and plans.
	void ApplyPlanCycle();
	// Fills a cycle train from State's route: motion, the resources it holds, can't stop short of or has behind it, and
	// which trains go first or come after it on its line.
	void FillSippTrain(const FATTrafficTrain& State, const FATSippGraph& Graph, const TMap<const AFGTrain*, int32>& SlotOf, FATSippTrain& Out) const;
	// Keeps a self-driving train's plan from a cycle for its grants, and takes its new route if the cycle rerouted it.
	void ApplyPlan(FATTrafficTrain& State, const FATSippCycle& Cycle, int32 Slot);
	void ApplyReroute(FATTrafficTrain& State, const FATSippCycle& Cycle, int32 Slot);
	// Returns true when State's route is due for a search for a faster one.
	bool IsSearchDue(FATTrafficTrain& State, float DeltaSeconds);
	// Indexes State's route: the route step of each track (first time on the route) and of the switch before it.
	void IndexRoute(FATTrafficTrain& State) const;
	// Returns the step of Of's route at which Other's front stands, the same way and ahead of Of's front, or INDEX_NONE.
	int32 FrontIndexOn(const FATTrafficTrain& Of, const FATTrafficTrain& Other) const;
	// Sets PlanRank of the self-driving trains in Trains and sorts them by it: a train ahead of another on that one's
	// route first, else the one that set off on its leg first.
	void RankTrains(TArray<FATTrafficTrain*>& Trains) const;
	// Returns true if Other has to pass Self before it gets to Key: Self's front stands on Other's route, the same way,
	// before Key, and both routes run over the same tracks from there up to Key.
	bool IsBehind(const FATTrafficTrain& Other, const FATTrafficTrain& Self, const FResourceKey& Key) const;
	void RemoveBookings(const AFGTrain* Owner);
	// Works out State's next leg from the station it is docked at, from its timetable and the game's pathfinder.
	void PredictDeparture(FATTrafficTrain& State, double Now);
	// Notes docking and departure; a dock seen from its start teaches the dwell.
	void TrackDocking(FATTrafficTrain& State, AFGTrain* Train, double Now);
	// Expected stay of Train at Station under the stop's Rules (null if unknown), with Transfer as in DockTransfer: at a
	// station without freight platforms, the train's last stay there, else none; a load or unload once is the freight
	// platforms' transfer if there is cargo to move (as at the train's last stay there while not known), else the stop's
	// duration; a full load or unload, or unknown rules, as long as the train's last such stay there, else the default. [s]
	float GetExpectedStay(const AFGTrain* Train, const AFGBuildableRailroadStation* Station, const FTrainDockingRuleSet* Rules, int32 Transfer) const;
	// The stay of a load or unload once that moves cargo: as measured, else the platforms' transfer time plus the
	// docking itself, else the default. [s]
	float GetTransferStay() const;
	// Whether a freight or fluid platform is in the platform chain on either side of Station, up to the next station.
	bool HasCargoPlatforms(const AFGBuildableRailroadStation* Station) const;
	// The docking rules of the stop at Index of Train's timetable if it is at Station, else null.
	static const FTrainDockingRuleSet* GetStopRules(const AFGTrain* Train, int32 Index, const AFGBuildableRailroadStation* Station, FTrainDockingRuleSet& Storage);
	// How far State's front is ahead of the start of Route, from its vehicles. [cm]
	float ComputeFrontAhead(const FATTrafficTrain& State, const TArray<FATRouteStep>& Route) const;
	// Sets State's claims: the route resources from its front up to its stopping distance plus lookahead that it may
	// enter now: booked for it, no other train booked there first or standing there, except one it follows the same way.
	void GrantFromPlan(FATTrafficTrain& State, double Now);
	// Returns true if Self may enter Key, at its route step Index, now with its booking entering at Enter (locked or not;
	// held from its last grant or not); otherwise sets the train in the way and why. A train it follows the same way is added to its followed
	// trains.
	bool CanEnter(FATTrafficTrain& Self, const FResourceKey& Key, int32 Index, double Enter, bool bLocked, bool bHeldBefore, const UFGRailroadTrackConnectionComponent* Via,
		const UFGRailroadTrackConnectionComponent* Exit, double Now, const FATTrafficTrain*& OutBlocker, FString& OutWhy) const;
	// Returns the last route step the train can no longer change: what it holds and what it can't stop short of.
	int32 GetRouteCommit(const FATTrafficTrain& State) const;
	// Rebuilds the network copy the planner runs on.
	void BuildSearchGraph(const TArray<AFGBuildableRailroadTrack*>& Tracks, int32 JointCount);

	TMap<FResourceKey, TArray<FBooking>> mBookings;
	TMap<FObjectKey, FOwnerBookings> mOwnerBookings;
	// The network as the route searches see it, from the last conflict model rebuild.
	TSharedPtr<const FATSippGraph, ESPMode::ThreadSafe> mSearchGraph;
	// The planning cycle running on a worker, if any, and the time since the last one started. [s]
	TSharedPtr<FATSippCycle, ESPMode::ThreadSafe> mCycle;
	float mSinceCycle = 0.f;
	// Whether the first cycle since traffic control started has been planned.
	bool mPlannedOnce = false;
	// The traffic state of each train by its pointer, for the current update.
	TMap<const AFGTrain*, FATTrafficTrain*> mStateOf;
	// Last full load or unload stay, or stay at a station without freight platforms, of each train at each station [s],
	// and whether its last stay there moved cargo.
	TMap<TPair<FObjectKey, FObjectKey>, float> mDwell;
	// Stations without freight platforms already logged.
	mutable TSet<FObjectKey> mNoCargoLogged;
	TMap<TPair<FObjectKey, FObjectKey>, bool> mLastTransfer;
	// Stay of a load or unload once that moves cargo, as measured, and the freight platforms' transfer time (-1 until
	// known). [s]
	float mTransferStay = -1.f;
	float mPlatformSeconds = -1.f;
	// Planner counters since the last summary.
	int32 mPlans = 0;
	int32 mPlanWaits = 0;
	int32 mPlanFailures = 0;
	int32 mPlanCommitted = 0;
	int32 mPlanReroutes = 0;
	int32 mPlanPredictions = 0;
	int32 mPlanUpdates = 0;
	int32 mCycles = 0;
	int32 mPlanReorders = 0;
	// The last train logged as held up past the planning retry limit.
	FString mLastRetryLimited;
	int32 mCyclesLate = 0;
	double mCycleMilliseconds = 0.0;
	double mPlanSeconds = 0.0;
	double mPlanSummaryAt = 0.0;
	double mPlanSlowLoggedAt = -1e9;
	double mWaitDumpAt = 0.0;
	// The last pair of trains logged holding conflicting resources, and the trains and tracks logged standing on a track
	// crossing one another train holds.
	FString mLastDoubleHold;
	TSet<FString> mLoggedCrossings;
	// Pairs of tracks the game counts as overlapping that were logged as not crossing.
	TSet<FString> mLoggedClearPairs;

	// Vehicles the acceleration and top speed settings have changed.
	TMap<TWeakObjectPtr<AFGRailroadVehicle>, FATVehicleBase> mVehicleBase;

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
	// The Mods menu settings as last read.
	FATSettings mSettings;
	TMap<TWeakObjectPtr<AFGTrain>, FATTrafficTrain> mTrains;

	// Crossing zones by track, and how many crossings there are.
	TMap<const AFGBuildableRailroadTrack*, TArray<FATZone>> mZonesOf;
	int32 mCrossingCount = 0;
	TMap<const UFGRailroadTrackConnectionComponent*, int32> mJointOf;
	TMap<const UFGRailroadTrackConnectionComponent*, float> mFoulOf;
	TMap<int32, float> mJointFoul;
	double mConflictsBuiltAt = -1e9;
	// Set when a track missing from the conflict model was looked up.
	mutable bool mConflictsStale = false;
	// Deadlocks going on, by their trains' names: when each was first and last seen (world seconds).
	struct FDeadlockSeen
	{
		double First = 0.0;
		double Last = 0.0;
	};
	TMap<FString, FDeadlockSeen> mDeadlocks;
};
