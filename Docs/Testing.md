# How the traffic works, and how to test it

With traffic control on (the default), every self-driving train ignores signals, and the space-time planner drives:

- Bookings: every 0.25 s each train in turn books the tracks and switch fouling zones on its route as time slots,
  around everyone else's bookings (nobody's existing booking is taken): self-driving trains along their route to the
  goal plus the stay there; docked trains their stay until the expected departure, then their next leg from the
  timetable as the game's pathfinder finds it; trains the mod doesn't drive where they stand, open-ended. A conflict
  puts a wait at the last track end before it. Same way on the same track (or through a switch the same way) is
  allowed with a headway (safety margin + 2 s) at both ends. Stays are learned per train and station from the last
  dock (40 s until one is seen). Plans use each train's cruising speed and acceleration as seen.
- Locks: what a train stands on, can't stop short of, or holds within reach of service braking is locked: nobody books
  ahead of it. What it holds beyond that is held: another train may fit in ahead, and the holder then gives it up with
  ordinary braking.
- Moving: a train is granted a track or switch only when nobody else stands on it or holds it, and no other train is
  booked there before it (locked bookings first); a train it follows the same way is the exception, kept apart by the
  follow limit (it can always stop behind the leader's tail plus the leader's braking distance). A train keeps what it
  holds and has locked whatever was booked since. It always keeps the speed at which it can stop before the end of its
  grants, planning with 80% of its rated braking until it has braked hard once, then 95% of what it actually managed
  (60% and 85% when not aggressive), per speed band and per tonne; over that curve it gets full brake, and the game's
  own catch stops it dead at its stop point. Trains stop 25 m before a switch (or the switch's fouling zone plus 10 m)
  and 10 m before other track ends. Held where its plan waits, a train rolls up to arrive as the slot opens.
- Rerouting (`/at reroute on|off`, on by default): every 5 s, at once when its planned arrival got 15 s worse or its
  plan failed, a train searches the fastest route through the bookings, and takes it if 10 s and 10% faster; it
  doesn't go back to a route it left in the last 2 minutes.
- Deadlocks: trains waiting for each other in a circle for 20 s are logged (`DEADLOCK`) and get the game's long-wait
  warning ("!") over them; if one only waits for the booking order it goes first for 30 s
  (`goes first ... to break the deadlock`). Trains blocking each other where they stand need moving by hand.
- Log lines: `traffic: <train>: waits for ...` when the train or place waited for changes; `planner:` lines for
  reroutes, failed plans, committed conflicts, stays, departure predictions, `WAIT` descriptions of trains waiting over
  a minute, and a summary every 60 s.

Player driven trains aren't controlled; the others treat them as obstacles.

## Mods menu settings (all on by default)

- **Traffic control**: Auto Trains drives every self-driving train. Off gives the game's signals back.
- **Congestion rerouting**
- **Trains pass through each other**: while traffic control is on, trains that touch don't crash or derail; the
  log counts how often it happened (`passed through each other N times`).
- **Aggressive timing**: plans braking with 95% of measured (80% of rated until measured), and counts each second a
  train has waited as half a second sooner in planning order. Off: 85%, 60%, a full second.

- **Safety margin (seconds)**: 0 to 10, default 0: extra time kept between two trains' bookings of the same track or
  switch (on top of 2 s).
- **Top speed (km/h)**: 0 to 500, default 0 (each train's own). Above the speed at which a locomotive's traction runs
  out (about 120 km/h on flat track), its traction curve is stretched up to the setting, and its power draw with it;
  every vehicle's own speed limit is raised to the setting.
- **Acceleration (x)**: 0.25 to 5, default 1: scales each self-driving train's locomotives' traction and their power
  draw (idle and full) by the same factor, so they accelerate and consume that much more. Braking isn't changed.

`/at traffic` and `/at reroute` switch for the session; changing the setting in the Mods menu applies again.

Back up the save first.

## Setup

1. Load a copy of your world. A copied save file is still named after the original session in game, because the
   game keeps the session name inside the save data.
2. `/at session <new name>`: renames the running session, so this and later saves and autosaves stop going to the
   original world's session.
3. `/at autopilot all off`, then `/at removesignals confirm`.
4. Save, and check the save list shows it under the new session.
5. `/at log on`, then `/at traffic on`.
6. Turn autopilot on for two trains whose routes cross or share track: `/at autopilot <train> on`. Add more once those
   behave.

## Watch for

1. **Following.** Two trains on the same track in the same direction: does the second keep its distance and stop
   behind the first?
2. **Junctions.** Trains meeting at a switch or crossing: does one wait before it until the other has cleared it,
   never stopping on it?
3. **Single track.** Two trains on one track from opposite ends: does one wait at the end (or a passing place)
   until the other is through?
4. **Long and short trains.** Does a long train wait where a short one would already go, because there's no room
   for its whole length past a junction?
5. **Stations.** Do trains still dock normally, with DynamicTimeTable and Freight Car Control doing their part?
6. **Rerouting.** Where there's another way round: does a train take it when the usual way is blocked or busy, and
   does it still reach its stop?
7. **Stuck trains.** Any train that waits forever: `/at traffic status` says why.

`/at traffic status` lists every moving self-driving train with its grants, how far it may drive, what it's waiting
for and its plan. The log also gets a `traffic:` line whenever a train's reason for waiting changes.

When you're done: `/at log off`. The telemetry CSV and `FactoryGame.log` (`LogAutoTrains` lines) show what happened.

## All commands

`<train>` is the number from `/at list`, or part of the train's name. In multiplayer only the host (or a dedicated
server admin) can use them, except `/at list` and `/at traffic status`.

- `/at traffic on|off|status`
- `/at reroute on|off`
- `/at list`: every train with its number, speed, length and distance to its goal
- `/at autopilot <train> on|off` or `/at autopilot all on|off`
- `/at removesignals confirm`: dismantles every block and path signal in the loaded world
- `/at session <new name>`: renames the running session; saves and autosaves go to the new name
- `/at log on|off`: telemetry every 0.2 s to `%LOCALAPPDATA%\FactoryGame\Saved\AutoTrains\telemetry-<time>.csv`
