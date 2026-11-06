# Changelog

## 1.4.3

- **Fixed: two trains could wait on each other forever without the deadlock rescue stepping in.** One of them was
  let go and held up again many times a second without ever moving off, and each moment it was let go counted as
  the end of its wait, so it never looked stuck. Seen with two push-pull trains sharing a switch, standing for over
  15 minutes. A wait now only ends once the train actually moves off or is let go for 3 seconds, so such a pair is
  passed through like any other deadlock.

## 1.4.2

- **A train queueing behind another one no longer stops on a switch or crossing.** It stopped right behind the train
  ahead, wherever that was, so a queue into a busy station could stand across a junction, hold up the line that
  crosses it and tie itself into a deadlock. Now it waits before the junction until there's room for it past it. A
  train already too close to stop before the junction goes on as before.

## 1.4.1

- **Fixed: trains queueing for a station where a train stays longer than expected could block its way out.** A train
  still docked after its expected stay counted as leaving within a second, again and again. A train waiting for that
  station moved up onto the junction in front of it for what looked like a short wait, and when the docked train
  finally left, its way out was blocked. Trains whose route passed through that station also never took a way around
  it. A train that has overstayed now counts as staying as long again as it has overstayed (up to 10 minutes), so
  others wait before the junction or take another route.
- **No more hill help for trains that are only held up**, unless they're on a climb too steep for them. A train waiting
  for another one (in a deadlock, say, or creeping up to its stop in a queue) counted as standing though told to go,
  got up to 4 times its traction and was reported as STALLED. A train braking with its throttle shut that isn't held
  up is still reported, but no longer gets more traction, which can't move it.
- The Detailed log telemetry file names each train by a unique id as well, so trains with the same name are told
  apart, and commas in train names no longer break its columns.

## 1.4.0

- **Change settings on a dedicated server from in game:** `/at settings` lists the server's Auto Trains settings, and
  `/at set <setting> <value>` changes one, e.g. `/at set DetailedLog on`. Changing them is for the host or a server
  admin, like the other `/at` commands that change something. Before, a dedicated server's settings could only be
  changed in its config file: your own Mods menu doesn't reach the server.
- A player signed in to a dedicated server with its admin password now always counts as a server admin for `/at`.
- Refused `/at` commands are logged with the player's name.

**Good to know**

- Everyone in a multiplayer session needs Auto Trains 1.4.0.

## 1.3.10

- New setting: **Detailed log**, off by default. FactoryGame.log now keeps to what matters: deadlocks, stuck,
  stalled and lost trains, hill help, reroutes and the planner's summaries. With Detailed log on it also gets what
  each train waits for, its station stays and next legs, how it speeds up and brakes, and every plan that finds no
  way, and every train's speed and control is recorded to a CSV file. Turn it on to report a problem.
- `/at log on|off` is gone: the Detailed log setting replaces it.

## 1.3.9

- **Fixed: a train could run into the tail of another one at a merge.** A train that is ahead on the same route can be
  followed closely through a switch or crossing. When its front was already past the merge but its tail was still on
  the other branch, the train behind was let in anyway and ran into that tail. Now it waits until none of the other
  train is on that switch or crossing off its own route.
- **Trains no longer wait across a junction.** A train queueing for a station, or for anything else, could stop with its
  tail across the switch or crossing of the other line, so trains coming the other way had to wait for it as well,
  which could end in a deadlock. Now it waits before the junction instead.
- **Better stay estimates for "fully load/unload" stops with a time.** With "or", a train stays at most that long; with
  "and", at least that long. Before, the time was left out, so trains that waited the whole time made others plan
  with a far too short stay.
- **Fixed: two trains facing each other on a single track could wait for good.** Each one's plan found no way past the other without naming it, so the deadlock was never found and never passed through. A train held up with no train named now waits for the first train standing in its way, so such a pair counts as a deadlock.
- Fewer false "stands though told to go" reports and less needless hill help: a train standing right at the point it
  has to stop at no longer counts.
- Other mods can now ask Auto Trains what a train waits for, when it will arrive, how long a trip takes, which trains
  queue for a station and which trains are in trouble (see Docs/API.md in the source).
- New setting: **Longest wait across a junction**.

## 1.3.8

- **Fixed: a train stopped on a climb could hold up the network for good.** Hill help only came on when the slope alone
  was too much for the train, so a train that lost its speed on a slightly gentler part of a climb could stand there
  for hours with the way ahead clear, and the trains behind it queued up with no warning. A train that stands while it
  is told to go now gets more and more traction, on any slope, until it moves. If it still stands at the full hill
  help, the log and faults.log say so, with what its driver and brakes are doing.
- **Fixed: a train could take the wrong branch at a switch and run onto a track the wrong way.** The game won't set a
  switch while any other train is within 12 m of it, and a train that gets there anyway just follows the switch as it
  stands. That could put it off its route, sometimes facing a signal from behind with no way on, holding up every train
  behind it. The mod now sets the switches just ahead of a train itself, unless another train still has to go through
  the same switch first.
- A self-driving train that loses its path is now reported in the log and faults.log. It used to stand there with no
  warning.
- New setting: **Hill help: stall time**.

## 1.3.7

- **Fixed: trains braking hard where 1.3.6 had them stop for their plan.** 1.3.6 made a train stop at every wait its
  plan had on the track ahead, also waits a plan made a moment earlier had put too close to stop for, so some trains
  braked hard from full speed, and two trains could end up planned to wait for each other until one found another
  way. Trains keep going as before 1.3.6; the crossings past such a wait are now checked before a train is given
  them, which was the real fault behind the crossing given to two trains.
- No more false crossing warnings in the log right after loading a save.

## 1.3.6

- **Fixed: a train stopping for good just past a crossing**, which could hold up the whole network. A train that
  rolled over a crossing while braking was then refused that crossing, which was already behind it. Trains also stop
  holding a crossing as soon as they have cleared it, rather than when they leave that piece of track.
- **Stuck trains show up in the log.** A train held up for longer than the deadlock threshold with no train to wait
  for is logged as STUCK (also in faults.log), with the trains waiting behind it, and shows the game's long-wait
  warning like a deadlocked train.
- **Fixed: a train waiting for the train behind it at a crossing.** Where a crossing lay just ahead of a train on the
  track it was on, the train following it could be given the crossing first, and both then stood still until the
  deadlock was passed through. The train in front now always gets it first.
- **Fixed: a train's rear cars not always keeping a crossing to themselves.** A car reaching past the end of the
  track it was on didn't count as standing on a crossing just beyond that end, so a long train's rear could briefly
  leave a crossing it was still driving over free for another train.
- **Fixed: trains braking hard at full speed where their plan had them wait.** A train with room enough to stop on
  the track where its plan waited was given that whole track, and the crossings on it past the wait, which nobody had
  checked. It then drove up to the wait at full speed and had to brake hard, or held a crossing another train had too.
  The grant now ends at the wait.
- **More detail in the log for crossing warnings**: a train coming onto a crossing it wasn't given is logged, and
  crossing warnings name where each train is, what it holds and what is booked there.

## 1.3.5

- **Fixed: trains queued for a crossing holding each other up.** A train waiting behind another on the way to a
  crossing could be given the crossing first, and the train in front then waited for it for good. Crossings now
  follow the same rule as track and switches: a train behind on the same line only gets a crossing after the one in
  front of it.

## 1.3.4

- **Crossings book only the crossing itself.** Where two tracks cross, only the stretch where they come within the
  track clearance counts, not both whole track pieces. Trains on a double roundabout's outer ring and trains crossing
  over to the inner ring can now use the junction at the same time, and only meet in time at the crossing point.
- **Trains wait right before a crossing** instead of before the whole junction: a train can stop partway along a
  track, just short of the crossing it has to wait for.
- **Fixed: trains planned a long stay at stations without freight platforms.** The game lets a train leave such a
  station almost at once, but the planner expected the usual 15-30 s and kept the track booked for it, so other
  trains (push-pull pairs in particular) waited for a train that had already left. Stations without freight or fluid
  platforms now count as no stay, or as long as the train was last seen staying there.
- **Fixed: trains stalling for good on steep climbs.** A heavy train held up halfway up a ramp steeper than its
  locomotives can start on rolled back and forth at full throttle forever, and every train behind it waited for it
  (it never counted as a deadlock, since it was trying to move). Now:
  - trains are planned with what their locomotives can really climb: they don't plan to wait on such a climb, and a
    train that is held up before one waits at its foot until its way to the top is clear;
  - a train that stalls on a climb anyway gets extra traction until it is over the steep part (new settings **Help
    stalled trains up hills**, on by default, and **Hill help: most extra traction (x)**, 4 by default).

## 1.3.3

- **Fixed: push-pull trains at dead-end stations meeting head-on on single track.** A train stopping at a station
  with no way on past it holds the track back to the last switch until it has backed out past that switch, so other
  trains heading in wait at the passing place instead of driving onto the single track in front of it. This replaces
  1.3.2's booking of every train's next leg, which didn't prevent it and made bookings flip back and forth every
  planning round.
- **Fixed: trains crawling at walking pace before switches** (often right before a straight). A train held up ahead
  no longer creeps along to arrive just as its way clears; it drives to where its plan waits, stops, and leaves on
  time. Planning also takes half the time it did in 1.3.2.
- **Trains slip past each other much closer to switches.** A switch's branches now only block each other until they
  are 3 m apart (was 5.5 m, and at least 20 m along every track), and trains no longer stop at least 25 m before a
  switch. New setting **Track clearance (m)** to set how close trains may pass.
- **A train that is in another's way finds another route** even when it already has track granted to its goal, so
  in a passing section it takes the free track instead of waiting on the one the other train needs.
- **A train with no way on never stops on a switch:** it waits where it is clear of every switch instead.
- **Fixed: a train standing just before a switch couldn't change which branch it takes**, so a push-pull train
  leaving a station kept heading for the track the oncoming train waited on instead of taking the free one.
- **New settings:** route search interval, route search when late, route search while stuck, track refresh, the
  deadlock wait and pass-through distance and time limit, plan search limit and slope lookahead (all at the values
  used so far).

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
