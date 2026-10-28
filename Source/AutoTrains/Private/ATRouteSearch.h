#pragma once

#include "CoreMinimal.h"
#include <atomic>

class AFGBuildableRailroadTrack;
class UFGRailroadTrackConnectionComponent;

// The track network as plain data, so route searches run off the game thread without touching a UObject. Built on the
// game thread with the conflict model; never changed after.
struct FATSearchGraph
{
	// Per connection (a track end; as a search node: the front at that end, about to leave the track through it): its
	// track, the connection at the track's other end, its switch joint (INDEX_NONE if none) and how far the joint's
	// fouling zone reaches along the track [cm], whether the game's one-way rule forbids passing it (a signal seen from
	// behind), and the connections of the tracks attached to it.
	TArray<int32> TrackOf;
	TArray<int32> OppositeOf;
	TArray<int32> JointOf;
	TArray<float> FoulOf;
	TArray<bool> OneWay;
	TArray<TArray<int32>> Attached;
	// Per track: its length [cm] and the tracks it conflicts with, itself first.
	TArray<float> Length;
	TArray<TArray<int32>> Conflicts;
	// Longest fouling zone of each joint's tracks. [cm]
	TMap<int32, float> JointFoul;
	// Game thread only: the objects behind the indices.
	TArray<TWeakObjectPtr<UFGRailroadTrackConnectionComponent>> Connections;
	TMap<const UFGRailroadTrackConnectionComponent*, int32> ConnectionIndex;
	TMap<const AFGBuildableRailroadTrack*, int32> TrackIndex;
};

// Another train's booking as a search sees it: when it holds the resource (world seconds), the connection it leaves a
// track through and, at a joint, the one it comes from (INDEX_NONE if unknown), whose it is (an index into the
// request's Owners) and whether nobody may go ahead of it.
struct FATSearchBooking
{
	double Enter = 0.0;
	double Clear = 0.0;
	int32 Exit = INDEX_NONE;
	int32 Via = INDEX_NONE;
	int32 Owner = INDEX_NONE;
	bool bLocked = false;
};

// Something under the train's body when its front stands at the start that another train is booked onto later: the
// latest the front may move off from the start, and how much further it has to go to leave it. [world s] [cm]
struct FATSearchHold
{
	double Latest = 0.0;
	float Reach = 0.f;
	int32 Owner = INDEX_NONE;
};

// Everything one search needs, copied on the game thread.
struct FATSearchRequest
{
	TSharedPtr<const FATSearchGraph, ESPMode::ThreadSafe> Graph;
	// The front at the end of Start's track at StartTime (world seconds), heading for Goal's track end; Current is the
	// route the train has now after Start, timed the same way for comparison.
	int32 Start = INDEX_NONE;
	int32 Goal = INDEX_NONE;
	double StartTime = 0.0;
	TArray<int32> Current;
	// The train's length and cruising speed, the time a stop costs over driving on, and the headway. [cm] [cm/s] [s] [s]
	float Length = 0.f;
	float Cruise = 500.f;
	float StopLoss = 0.f;
	double Headway = 0.0;
	// Bookings the train has to keep clear of, by track and by joint, and what its body holds at the start.
	TMap<int32, TArray<FATSearchBooking>> TrackBookings;
	TMap<int32, TArray<FATSearchBooking>> JointBookings;
	TArray<FATSearchHold> Holds;
	// The names of the bookings' owners, for the log.
	TArray<FString> Owners;
};

// Why the current route can't be driven: the step of Current the front gets no further than the start of, when it gets
// there and the latest it may move off (both ATSearchNever if it never got there), the step whose window or the hold
// that sets that latest time (INDEX_NONE: none; -1 with LimitOwner set: a hold), and the booking keeping the step from
// being entered then: when it lets the front in, whose it is and how it meets the train.
struct FATSearchBlock
{
	enum class EWay : uint8
	{
		None,
		SameWay,
		Crossing,
		Switch,
		Signal,
		NotAttached,
	};
	int32 Step = INDEX_NONE;
	double Reached = 0.0;
	double Latest = 0.0;
	int32 LimitStep = INDEX_NONE;
	int32 LimitOwner = INDEX_NONE;
	double Opens = 0.0;
	int32 Owner = INDEX_NONE;
	EWay Way = EWay::None;
	bool bLocked = false;
};

struct FATSearchResult
{
	// The fastest route's connections after Start, and when it and the current route have the front at the goal's track
	// end (world seconds; ATSearchNever if not reachable).
	TArray<int32> Route;
	double Arrival = 0.0;
	double CurrentArrival = 0.0;
	FATSearchBlock CurrentBlock;
	int32 States = 0;
	double Milliseconds = 0.0;
};

// A search in flight: the worker fills Result, then sets bDone.
struct FATSearchTask
{
	FATSearchRequest Request;
	FATSearchResult Result;
	std::atomic<bool> bDone{false};
};

constexpr double ATSearchNever = 1e18;

namespace ATRouteSearch
{
	// Safe-interval search over the whole network for a train of the request's length: a state is a track end the front
	// reaches and a window of entry times in which the next track is free of every booking for as long as the train
	// takes to pass it; the train may wait at a track end, or at the start, only as long as everything under its body stays
// free. Runs
	// the task on a worker thread and sets bDone when finished.
	void Launch(const TSharedRef<FATSearchTask, ESPMode::ThreadSafe>& Task);

	// The search itself, on the calling thread.
	void Run(FATSearchTask& Task);
}
