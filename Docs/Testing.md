# Test build 0.3.1: traffic without signals

This build adds the first traffic control. With it on, every self-driving train ignores signals and claims the
track ahead of it:

- A track is free only if no other train stands on it or has claimed it, or a track overlapping it (switch branches,
  crossings).
- A train always keeps the speed at which it can stop before the end of its claims, so it slows down and stops for
  traffic ahead and speeds up as track frees.
- A claim may only end where the whole train stands off junctions and where no other train needs to come the other
  way. So a train never stops inside a junction or halfway along single track that another train needs. It waits
  before the junction, or goes on until it's clear of it.
- The train that has waited longest gets to claim first.

Trains don't yet adjust their speed ahead of time to arrive just as track frees; they brake when they must. Player
driven trains aren't controlled; the others treat them as obstacles.

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
6. **Stuck trains.** Any train that waits forever: `/at traffic status` says why.

`/at traffic status` lists every moving self-driving train with its claims, how far it may drive and what it's
waiting for. The log also gets a `traffic:` line whenever a train's reason for waiting changes.

When you're done: `/at log off`, then tell me. I read the telemetry CSV and `FactoryGame.log` myself.

## All commands

`<train>` is the number from `/at list`, or part of the train's name.

- `/at traffic on|off|status`
- `/at list`: every train with its number, speed, length, command and distance to its goal
- `/at autopilot <train> on|off` or `/at autopilot all on|off`
- `/at removesignals confirm`: dismantles every block and path signal in the loaded world
- `/at session <new name>`: renames the running session; saves and autosaves go to the new name
- `/at log on|off`: telemetry every 0.2 s to `%LOCALAPPDATA%\FactoryGame\Saved\AutoTrains\telemetry-<time>.csv`
- Control tests from 0.2: `/at speed <train> <km/h>`, `/at stopat <train> <m>`, `/at release <train>|all`,
  `/at signals <train> ignore|obey`, `/at route <train> check|via`
