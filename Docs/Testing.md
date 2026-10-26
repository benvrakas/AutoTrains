# Test build 0.5.3: traffic without signals

This build adds the first traffic control. With it on, every self-driving train ignores signals and claims the
track ahead of it:

- A track is free only if no other train stands on it, has claimed it, or holds a track crossing it. A switch or
  merge is free only if no other train has a vehicle within 20 m of the switch point or holds track through it.
  Trains stop 25 m before a switch (10 m elsewhere), so a waiting train never fouls one.
- A train always keeps the speed at which it can stop before the end of its claims, so it slows down and stops for
  traffic ahead and speeds up as track frees. It plans with 60% of its rated braking until it has braked hard once,
  then with 85% of what it actually managed. If it's ever over that curve it gets full brake, and at the last
  connection the game's own catch stops it dead at its stop point, whatever its speed.
- A claim may only end where the whole train stands clear of crossings and switches that another train's route
  passes within 1.5 km (3 km when not aggressive), and where no other train needs to come the other way. So a
  train never stops inside a junction someone needs, or halfway along single track that another train needs, but
  it may wait on a crossing or switch nobody is heading for.
- Following: a train may count track or a switch another train stands on or has claimed as free if that train is
  moving at 5 m/s or more, isn't held up itself, and clears it before this train could get there at full
  acceleration, plus the safety margin setting (0 s by default). So a train follows another through a roundabout instead of waiting until it has left
  the whole path. A follower never takes track its leader still needs and hasn't claimed (`follows X, which still
  needs`). It's rechecked every update; if the leader slows, the follower gives the claims back and brakes
  (`gives up ... claims`).
- Claims run unbroken from the train's position; a claim left past a gap (after a reroute or giving way) is dropped.
- Claim order: the train that would get through the next track it needs soonest claims first; each second waited
  counts as half a second sooner (a full second when not aggressive), so no train waits forever.
- Going first: a train may take a track or switch that another train has only claimed if it clears it before the other
  train could get there at full acceleration, plus the safety margin, and the other train can still stop short of it.
  The other train gives up its claims from there on. The log says `goes first`.
- Deadlocks: if trains wait for each other in a circle for 20 s, the log says `DEADLOCK` with each train's reason.
- Congestion rerouting (`/at reroute on|off`, on by default): every 5 s each moving train prices its route as track
  length plus congestion: +1.5 km for a track with a standing train, +400 m for a moving one, +300 m for track
  another train has claimed, +800 m for track another train will cross the other way, +50 m the same way. If a route
  from the end of its claims is at least 15% and 300 m cheaper, it switches to it, even when that route is longer.
  The log says `rerouted` with the costs and lengths.

Trains don't yet adjust their speed ahead of time to arrive just as track frees; they brake when they must. Player
driven trains aren't controlled; the others treat them as obstacles.

## Mods menu settings (all on by default)

- **Traffic control**: Auto Trains drives every self-driving train. Off gives the game's signals back.
- **Congestion rerouting**
- **Trains pass through each other**: while traffic control is on, trains that touch don't crash or derail; the
  log counts how often it happened (`passed through each other N times`).
- **Aggressive timing**: plans braking with 95% of measured (80% of rated until measured), a 10 m buffer for the train giving way and claims 100 m past the braking distance instead
  of 200 m. Off is the cautious values.

- **Safety margin (seconds)**: 0 to 10, default 0: extra time a train leaves when following another out of a junction
  or going ahead of another train's claim.

`/at traffic` and `/at reroute` switch for the session; changing the setting in the Mods menu applies again.

Back up the save first.

## Setup

1. Load the copy of your world ("Bens CSE No Signals"). In game it's still named after the original session, because
   the game keeps the session name inside the save data.
2. `/at session Bens CSE No Signals`: renames the running session, so this and later saves and autosaves stop going
   to your main world's session.
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

`/at traffic status` lists every moving self-driving train with its claims, how far it may drive and what it's
waiting for. The log also gets a `traffic:` line whenever a train's reason for waiting changes.

When you're done: `/at log off`, then tell me. I read the telemetry CSV and `FactoryGame.log` myself.

## All commands

`<train>` is the number from `/at list`, or part of the train's name.

- `/at traffic on|off|status`
- `/at reroute on|off`
- `/at list`: every train with its number, speed, length, command and distance to its goal
- `/at autopilot <train> on|off` or `/at autopilot all on|off`
- `/at removesignals confirm`: dismantles every block and path signal in the loaded world
- `/at session <new name>`: renames the running session; saves and autosaves go to the new name
- `/at log on|off`: telemetry every 0.2 s to `%LOCALAPPDATA%\FactoryGame\Saved\AutoTrains\telemetry-<time>.csv`
- Control tests from 0.2: `/at speed <train> <km/h>`, `/at stopat <train> <m>`, `/at release <train>|all`,
  `/at signals <train> ignore|obey`, `/at route <train> check|via`
