# Changelog

## 1.0.1

- **Update interval** setting (0.05 to 2 seconds, default 0.25): how often trains are planned and given track.

## 1.0.0

First release.

- **Train traffic without block or path signals.** Every self-driving train books the tracks and switches on its way
  as time slots around the other trains, and slows down, waits or goes so no two trains ever need the same track at
  the same time. Existing signals are ignored while traffic control is on.
- **A train moves only onto track that is its turn and that nobody else holds,** and only stops where it leaves the
  crossings and switches other trains are booked on free. It's given enough track ahead to reach its cruising speed.
- **Going first:** a train may go through a crossing or switch ahead of a train still far enough away to stop, when
  it's clear before that train arrives.
- **Following** through junctions and roundabouts the same way, keeping enough distance to stop behind the train
  ahead; trains on one line never overtake each other in their plans.
- **Fastest-route search** through everyone's bookings: trains take another route, even a longer one, when it gets
  them there clearly sooner.
- **Docked trains' departures are booked ahead,** from how long each train stayed at that station last time and the
  next stop in its timetable, so nobody queues in the way it needs to leave.
- **Deadlocks shown:** trains waiting for each other in a circle get the game's warning over them.
- **Braking and acceleration learned per train,** per speed band and allowing for cargo, so plans use what the train
  really manages.
- **Trains pass through each other** instead of crashing (setting, on by default).
- **Forced braking:** a train that can't stop in time with its own brakes is stopped anyway, at the point it must
  stop.
- Mods menu settings: Traffic control, Congestion rerouting, Trains pass through each other, Aggressive timing,
  Safety margin, Top speed, Acceleration, and for fine-tuning: rerouting thresholds, lookahead, stop distance, normal
  braking and the station stay estimate.
- `/at removesignals confirm` dismantles every block and path signal in the world.
- Works with DynamicTimeTable and Freight Car Control.

**Good to know**

- Back up your save before trying it. Uninstalling gives trains back to the game's signals; if you removed your
  signals, place them again first.
- Remove DynamicTrainRoutes and other mods that set train routes.
