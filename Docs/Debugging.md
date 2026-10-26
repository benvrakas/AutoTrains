# Debugging

## Logs

The game log is `%LOCALAPPDATA%\FactoryGame\Saved\Logs\FactoryGame.log`. Filter for `LogAutoTrains`.

- startup: `Auto Trains module starting`, `Hooks installed`
- every `/at` command and its result
- `stopat`: the consist's length, mass, brake deceleration and brake distance, then `<train> stopped for stopat: <n> m
  short of|past the stop point`
- `route check`: `SAME ROUTE` or `DIFFERENT ROUTE` with point counts and distance differences, then the first 8 points
  of both routes side by side
- `route via`: the route's length and point count, whether the train accepted it, and later `the path this mod set was
  replaced` if the game or another mod gave the train a different path
- while logging: `Control hook: <n> calls in 10 s, <us> us each`

## Traffic

- `Traffic control on|off` when switched; `traffic: <train>: <reason>` whenever the reason a train's claims stop
  changes: `waits for <train>, which holds <track> or a track crossing it, <n> m ahead`, or `no safe place to stop
  beyond <track>: <why>` (the train would stand on a junction, or another train comes the other way over a track).
- `/at traffic status` logs `traffic status:` lines: speed, number of claims, how far the train may drive, how long
  it has waited, and its reason.
- Telemetry columns `traffic_managed`, `authority_m` (distance from the front to where it must be able to stop;
  -1 means to its station), `claims`, `waiting_s`, `blocked_by`.
- Distances from the game's ATC are from the MU master locomotive; the mod corrects them by how far the train's
  front is ahead of it (`FrontAhead`).

### Fixed in 0.5.5 (from the 0.5.4 session log)

- 240 of 241 `over its braking curve` lines said `behind a leader`. Two causes: leaders had no direction, so a
  train coming up behind (or a follower) counted as a leader, and the follower capped itself on track the "leader"
  stood on behind it; and a follower already on its leader's track was capped at the leader's braking distance
  alone, not the gap to its tail, so the cap collapsed whenever the leader slowed. Now only a train ahead going the
  same way is a leader (`IsLeaderOf`), and the cap is the gap to its tail plus its braking distance.
- Mutual `gives up` pairs (each train treating the other as its stopped leader) came from the same directionless
  leaders.
- `goes first ... gives up 0 claims` repeated: the cut didn't remove anything (claims under the holder's own
  vehicles stay). It's logged only when claims are actually given up. A switch spanned by a train's vehicles on
  both sides now counts as occupied, so it can't be preempted at all.
- Going first used to take the other trains' claims before checking the rest of the track the train needed; when
  that check failed, the other trains had lost their claims for nothing and had to stop and reclaim. Preemptions
  are now applied tentatively and rolled back if the check fails.
- A train rerouted back and forth every 5 s between two near-equal routes (`at test1`); reroutes now have a 30 s
  cooldown.
- A train following a moving train the same way (AI Limiter RSC behind Heavy Oil Residue) waited for the leader's
  timing check; with aggressive timing a same-way leader on the route is followed directly, with the moving-block
  cap keeping the distance.

## Telemetry

`/at log on` writes `%LOCALAPPDATA%\FactoryGame\Saved\AutoTrains\telemetry-<time>.csv`: one `# consist` line per
train, then a row per train every 0.2 s:

- `speed_kmh`; `game_target_kmh`, the speed the game's driver wanted; `final_target_kmh`, after this mod's command
- `accel_ms2`, from the speed change between samples
- `odometer_m`, distance travelled since the train was first seen; `stop_remaining_m` for `stopat`
- `goal_distance_m`, the track distance left to the train's goal along its path
- `restrictive_*` and `next_signal_*`, what the game's driver is braking for
- `front_track`/`front_offset_m` and `rear_track`/`rear_offset_m`, where the first and last vehicle are

## How the control works

- The mod hooks `AFGTrain::TickAtc`. After the game computes the train's target speed, the mod lowers it for a speed
  cap or a braking curve to a stop point (70% of the consist's brake deceleration, ending 1 m short).
  `TickSelfDriving_FollowPath` then drives to that target speed with the game's own throttle and brake logic.
- Ignoring signals happens in the same hook: the mod clears the stop aspect from the train's target points that have
  a facing signal, then picks the restrictive point and target speed the way the game does, without those stops.
  Stations (dock points) are untouched. (0.2.0 hooked `FTrainAtcData::RefreshTargetPointsDistanceAndAspect` for
  this, which crashed: the hook's call-through stub for that function was corrupt, likely because it starts with a
  short conditional jump. Don't hook it.)
- Routes are chains of track connections, built by the mod's own shortest-path search and given to the train with
  `FTrainAtcData::SetPath`.
