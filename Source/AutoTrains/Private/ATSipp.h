#pragma once

#include "CoreMinimal.h"
#include <atomic>

class AFGBuildableRailroadTrack;
class AFGTrain;
class UFGRailroadTrackConnectionComponent;

// An open-ended time: a stay nobody knows the end of, or a resource held for good. [world s]
constexpr double ATSippNever = 1e12;

// The track network as plain data, so planning runs off the game thread without touching a UObject. Built on the game
// thread with the conflict model; never changed after. A resource is a track (0 .. tracks - 1) or a switch joint's
// fouling zone (tracks + joint).
struct FATSippGraph
{
	// Per connection (a track end; as a search node: the front at that end, about to leave the track through it): its
	// track, the connection at the track's other end, its switch joint (INDEX_NONE if none) and how far the joint's
	// fouling zone reaches along the track, how far short of it a train stops [cm], whether the game's one-way rule
	// forbids passing it (a signal seen from behind), and the connections of the tracks attached to it.
	TArray<int32> TrackOf;
	TArray<int32> OppositeOf;
	TArray<int32> JointOf;
	TArray<float> FoulOf;
	TArray<float> StopMarginOf;
	TArray<bool> OneWay;
	TArray<TArray<int32>> Attached;
	// Per track: its length [cm] and the tracks it conflicts with, itself first.
	TArray<float> Length;
	TArray<TArray<int32>> Conflicts;
	// Per joint: the longest fouling zone of its tracks, and the longest of all. [cm]
	TArray<float> JointFoul;
	float MaxJointFoul = 0.f;
	// Game thread only: the objects behind the indices.
	TArray<TWeakObjectPtr<UFGRailroadTrackConnectionComponent>> Connections;
	TArray<const AFGBuildableRailroadTrack*> Tracks;
	TMap<const UFGRailroadTrackConnectionComponent*, int32> ConnectionIndex;
	TMap<const AFGBuildableRailroadTrack*, int32> TrackIndex;

	int32 NumResources() const { return Length.Num() + JointFoul.Num(); }
	int32 JointResource(int32 Joint) const { return Length.Num() + Joint; }
};

enum class EATSippKind : uint8
{
	// A self-driving train's plan.
	Moving,
	// A train this mod doesn't drive, where it stands.
	Standing,
	// A docked train's stay and its next leg.
	Predicted,
};

// A booking as the planner holds it: the owner's slot in the cycle, when it holds the resource (world seconds), the
// connection it leaves a track through and, at a joint, the one it comes from (INDEX_NONE if unknown). Locked: nobody
// books ahead of it. Held: from the owner's last grant. Committed: the owner can't give way there.
struct FATSippBooking
{
	int32 Owner = INDEX_NONE;
	double Enter = 0.0;
	double Clear = 0.0;
	int32 Exit = INDEX_NONE;
	int32 Via = INDEX_NONE;
	EATSippKind Kind = EATSippKind::Moving;
	bool bLocked = false;
	bool bHeld = false;
	bool bCommitted = false;
};

// Every booking by resource, and the resources each owner has bookings on.
struct FATSippTable
{
	TArray<TArray<FATSippBooking>> ByResource;
	TArray<TArray<int32>> OwnerResources;

	void Init(int32 Resources, int32 Owners);
	void Add(int32 Resource, const FATSippBooking& Booking);
	void Remove(int32 Owner);
};

// One resource a plan passes: the front positions at which it enters it (<= 0: entered already) and its tail has
// cleared it [cm from where the front is now], the times [world s], the route step it belongs to (INDEX_NONE for what
// the tail holds behind the route), and its connections as in a booking.
struct FATSippPass
{
	int32 Resource = INDEX_NONE;
	int32 Exit = INDEX_NONE;
	int32 Via = INDEX_NONE;
	float EnterAt = 0.f;
	float ClearAt = 0.f;
	double Enter = 0.0;
	double Clear = 0.0;
	int32 Step = INDEX_NONE;
	bool bLocked = false;
	bool bHeld = false;
	bool bCommitted = false;
};

enum class EATSippTurn : uint8
{
	// Books what it stands on until StandUntil.
	Standing,
	// Plans and books its route.
	Plan,
};

// One train's turn in a planning cycle.
struct FATSippTrain
{
	EATSippTurn Turn = EATSippTurn::Standing;
	EATSippKind Kind = EATSippKind::Standing;
	// Place in the planning order of self-driving trains (INDEX_NONE for others), and whether its plan keeps clear of
	// the uncommitted bookings of self-driving trains planned before it only (else of none of them).
	int32 Rank = INDEX_NONE;
	bool bRanked = true;
	// Standing: the resources it stands on, booked locked until StandUntil.
	TArray<int32> Standing;
	double StandUntil = ATSippNever;
	// Plan: the connection it leaves each route track through, from the track its front is on, and how far its front
	// is from the end of that first track [cm]; whether the last step is its goal (else the route was cut short where
	// the network copy ends).
	TArray<int32> Route;
	float FirstEnd = 0.f;
	bool bGoal = true;
	// What its tail holds behind the route (entered, locked), and which route resources are locked or held.
	TArray<FATSippPass> Tail;
	TSet<int32> Locked;
	TSet<int32> Held;
	// Slots whose bookings go first wherever they meet this train's way (trains ahead of it on its route), and, per slot,
	// the resources that train only gets to after this one (it is behind this one on the same line).
	TSet<int32> First;
	TMap<int32, TSet<int32>> Behind;
	// Its length [cm], cruising speed [cm/s], planned acceleration and service braking [cm/s^2], speed now [cm/s],
	// whether it stands, the earliest it may move off and its stay at the goal (world s, ATSippNever if unknown).
	float Length = 0.f;
	float Cruise = 500.f;
	float Accel = 100.f;
	float Brake = 100.f;
	float Speed = 0.f;
	bool bStanding = false;
	double MinDepart = 0.0;
	double GoalDwell = 0.0;
	// Route search: whether to look for a faster route beyond route step Commit, and the gain a new one needs. [s] [%]
	bool bSearch = false;
	int32 Commit = 0;
	float MinGainSeconds = 0.f;
	float MinGainPercent = 0.f;
};

// A train's plan from a cycle.
struct FATSippResult
{
	bool bPlanned = false;
	// The route planned (connections, from the track the front is on) and what it passes, timed.
	TArray<int32> Route;
	TArray<FATSippPass> Passes;
	// When the front reaches the goal's track end (ATSippNever if not).
	double Arrival = ATSippNever;
	// The first wait still ahead: at which route step's end it stands (-1 where it is now, INDEX_NONE for none), until
	// when, and for whose booking (slot).
	int32 WaitStep = INDEX_NONE;
	double WaitUntil = 0.0;
	int32 WaitFor = INDEX_NONE;
	// Failed: no way to the goal; it stands for good at the end of route step StandStep (-1 where it is now), held up by
	// BlockedBy's booking on BlockedAt (INDEX_NONE if unknown).
	bool bFailed = false;
	int32 StandStep = -1;
	int32 BlockedBy = INDEX_NONE;
	int32 BlockedAt = INDEX_NONE;
	// A train booked first on a resource this one can't stop short of any more, and that resource.
	int32 CommittedWith = INDEX_NONE;
	int32 CommittedAt = INDEX_NONE;
	// Rerouted: the arrival on the route it had, and the route search's effort.
	bool bRerouted = false;
	double OldArrival = ATSippNever;
	int32 SearchStates = 0;
	double SearchMilliseconds = 0.0;
	// The plan's search: its states, and the moves it left out because the train couldn't reach the speed, its tail
	// couldn't keep to a window, or the stay at the goal didn't fit.
	int32 States = 0;
	int32 RejectedMotion = 0;
	int32 RejectedTail = 0;
	int32 RejectedStay = 0;
};

// One planning cycle: every train in turn (slot order) removes its own bookings and books its new plan around the
// others' in its scope. Filled on the game thread, run on a worker, read back on the game thread once bDone.
struct FATSippCycle
{
	TSharedPtr<const FATSippGraph, ESPMode::ThreadSafe> Graph;
	double Now = 0.0;
	double Headway = 0.0;
	TArray<FATSippTrain> Trains;
	FATSippTable Table;
	TArray<FATSippResult> Results;
	double Milliseconds = 0.0;
	std::atomic<bool> bDone{false};
	// Game thread only: the train of each slot.
	TArray<TWeakObjectPtr<AFGTrain>> Owners;
};

namespace ATSipp
{
	// Plans every train of the cycle on worker threads (route searches in parallel first, then the turns in order) and
	// sets bDone when finished.
	void Launch(const TSharedRef<FATSippCycle, ESPMode::ThreadSafe>& Cycle);

	// The cycle itself, on the calling thread.
	void Run(FATSippCycle& Cycle);
}
