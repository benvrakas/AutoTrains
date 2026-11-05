# Auto Trains API for other mods

Auto Trains is optional to other mods and they to it. Call it by reflection, the way Freight Car Control's
ExternalSetStops is called: find the actor of class `ATTrafficSubsystem` (an SML mod subsystem, one per world; for
example `TActorIterator<AActor>` with `GetClass()->GetName() == TEXT("ATTrafficSubsystem")`, or the subsystem actor
manager), then `FindFunction(TEXT("..."))`, fill an `FStructOnScope` and `ProcessEvent`. Everything runs on the
server's game thread. Times are seconds.

| Function | Parameters | Returns |
|---|---|---|
| `ExternalGetWaitingFor` | `AFGTrain* Train`, out `TArray<AFGTrain*> OutWaitingFor`, out `float OutWaitedSeconds`, out `FString OutReason` | `bool`: true if Auto Trains drives the train now. OutWaitingFor: the trains it waits for, the one holding it up first; parked trains and trains Auto Trains doesn't drive are included. OutWaitedSeconds: how long it has been held up. OutReason: why, as in the log; empty if it isn't held up. |
| `ExternalGetPlannedArrival` | `AFGTrain* Train`, out `AFGBuildableRailroadStation* OutGoal` | `float`: seconds until the train reaches OutGoal by its plan, around every other train's booking and queue. For a docked train: its next stop, the stay included. -1 without a plan that gets there. |
| `ExternalEstimateTravelSeconds` | `AFGTrain* Train`, `AFGBuildableRailroadStation* From`, `AFGBuildableRailroadStation* To` | `float`: seconds from From to To on the shortest way the signals allow (leaving either way), at the train's cruising speed and acceleration, with no other train in the way; -1 if there is no such way. Train may be null (default speeds). Costs a search over the whole network (a few ms on large networks). |
| `ExternalGetStationQueue` | `AFGBuildableRailroadStation* Station`, out `TArray<AFGTrain*> OutTrains` | `float`: seconds until the last train docked there or heading there for its current stop is expected to have left; 0 if none, -1 if one of them has no plan. OutTrains: the docked ones, then the others by planned arrival. |
| `ExternalGetTroubledTrains` | out `TArray<AFGTrain*> OutTrains`, out `TArray<FString> OutKinds`, out `TArray<float> OutSeconds` | `int32`: how many entries. Kinds: `Deadlock` (in a cycle of trains waiting for each other), `Stuck` (held up with no train to wait for), `Lost` (self-driving without a path), `Stalled` (standing at the full hill help though told to go). A train can appear more than once. |
| `ExternalTimeTableChanged` | `AFGTrain* Train` | nothing. Call after changing a train's timetable: Auto Trains predicts its next leg again from the new timetable instead of the old one. |

A deadlock is passed through (the trains drive through each other) after the "Pass through a deadlock after" setting;
poll `ExternalGetTroubledTrains` more often than that to act first.
