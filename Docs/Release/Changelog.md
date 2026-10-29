# Changelog

## 1.3.3

- **Fixed: push-pull trains at dead-end stations meeting head-on on single track.** A train stopping at a station
  with no way on past it holds the track back to the last switch until it has backed out past that switch, so other
  trains heading in wait at the passing place instead of driving onto the single track in front of it. This replaces
  1.3.2's booking of every train's next leg, which didn't prevent it and made bookings flip back and forth every
  planning round.
- **Fixed: trains crawling at walking pace before switches** (often right before a straight). A train held up ahead
  no longer creeps along to arrive just as its way clears; it drives to where its plan waits, stops, and leaves on
  time. Planning also takes half the time it did in 1.3.2.

## 1.3.2

- **Fixed: push-pull trains on single track waiting for each other forever.** A train heading for a station now also
  books its next leg after the stay: on to its next stop, the way it will leave (backwards for a push-pull train if
  that is shorter). Other trains plan around that departure, so none drives onto the single track a train has to
  turn back over; they wait at the passing place instead.
- **Fixed: jams where a train waited without naming the train in its way**, which left the deadlock detection and the
  planning order unable to free it.
- **Fixed: sudden hard stops when a plan briefly failed.** A train whose plan found no way for a moment lost the track
  just ahead that it had already been given, and was stopped on the spot even at full speed. It now keeps track it
  holds; only a train standing there or holding it stops it.
- **Fixed: trains running past their stop under forced braking** downhill or while their brakes were still building
  up. Forced braking now counts what the train actually slowed by itself, not what its brakes should manage.
- **Acceleration per train, per speed and per load:** a train's pulling power is learned at each speed as a force, so
  a loaded train plans slower acceleration than the same train empty, and speeds it hasn't driven yet follow its
  locomotives' traction curves (strong at low speed, fading towards top speed). More locomotives or fewer cars plan
  faster acceleration from the start.
- **Slopes over the whole train:** the slope a train brakes and accelerates on is taken over its whole length, from
  its rear car to its front car, so a long train cresting a hill or entering a dip is planned as gravity really pulls
  on it.

## 1.3.1

- **Station stays known from the timetable:** a stop set to load or unload once takes the freight platforms' fixed
  transfer time when there is cargo to move, and the stop's set duration when there isn't, so a docked train's
  departure is known as soon as its platforms start (or don't). Only stops set to fully load or unload, which take
  as long as the cargo does, still go by the train's last stay there; the Station stay estimate setting now only
  applies to those.
- **Slopes:** each train's measured braking and acceleration now leave out the slope it was on, and plans, stopping
  distances and the game's stop point add the slope of the track ahead, so trains stop where planned downhill and
  aren't held back uphill.
- Removed the `/at session` chat command.

## 1.3.0

- **One planner for route, speed and waits (multi-train safe-interval planning):** each train's plan now comes out of
  one search over the free time windows of every track and switch, with the train's own acceleration and braking and
  its whole length, planned in turn around the trains before it. A train can slow down to reach a crossing just as it
  clears instead of stopping, and only plans to stand where its body holds nobody up. This replaces the old timetable
  pass with its corrections ("no plan after 32 waits" is gone) and the separate route search, so the route a search
  finds is exactly the plan the train then drives.
- **Planning runs on worker threads:** the whole planning round runs off the game thread, with route searches side by
  side, so planning no longer costs frame time.
- **Straights planned as one stretch:** speed and stops are only decided at switches, crossings, platforms and
  one-way signals, so planning has far less to look at on lines built from many short pieces; trains on a straight
  simply queue one behind the other.
- **A train that finds no way goes first:** when a train's plan fails because a train planned before it is in the
  way, it is planned before that train for the rest of its trip and the round is planned again (new setting Planning
  retries, 3 by default). A train is never put before one ahead of it on the same line.
- **Fixed: trains slowing for each other at junctions.** A train standing over a switch held every branch of it, so
  an unused branch crossing the other line (a crossover) held up trains there.
- **Fixed: tracks that only run close counted as crossing.** Two tracks count as crossing only where their centre
  lines come within 5.5 m, not wherever the game sees their outlines overlap (curves a foundation apart).

## 1.2.0

- **Fixed: deadlocks on track used both ways and where two lines cross twice close together.** A train that has
  started into such a stretch can't give way until it is through, so every train now keeps clear of its plan up to
  the next place it can stand without holding anyone up, whatever the planning order. This also stops the sudden hard
  stops in front of level crossings when the other train hadn't cleared them yet.
- **Signals set direction:** rerouting keeps the game's one-way rule, so no train is sent past a signal from behind.
  Place a block signal facing the way trains should go to keep them on the right side of a double track or to make a
  link one-way. Signals still never stop trains.
- The log no longer fills with check warnings while trains pass through a deadlock.

## 1.1.1

- **Fixed: two trains with their fronts on the same track each took the other to be ahead** and waited for it, a
  deadlock. Only the one nearer the track's end is ahead now.
- **Fixed: trains crawling or waiting with a clear way ahead.** Two trains could each plan to wait for the other, and
  the waits kept slipping later, so both crept along for a minute or more. Trains now plan in a fixed order (one ahead
  on another's route first, else whichever set off first); each plans around the trains before it and only around
  what later ones stand on or are already driving onto, so the plans always fit together.

## 1.1.0

- **Timetables by safe-interval search (SIPP):** each train's waits are now found by one search over the free time
  windows at every place along its route it could stand, with its whole length clear of other trains' bookings. Waits
  land where they cost least, and a train no longer waits for a train that is still far away when it could have
  gone through first.
- A train is given track up to the place its plan waits and no further, so it never stands where others are booked.
- **Deadlocks cleared by passing through (beta, on by default):** trains deadlocked for longer than a set time (60 s)
  drive on through each other until they're clear of the jam; each case is written to
  `FactoryGame/Saved/AutoTrains/faults.log`.
- New settings: Update interval, Clear deadlocks by passing through, Pass through a deadlock after.

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
