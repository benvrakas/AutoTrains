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

### Fixed before 1.0.0 (from the 0.5.5 session log)

- Deadlock, Rubber SNK and at test 2 each standing on the other's crossing. at test 2 planned with 3.65 m/s2 but
  managed 2.4 at 100 km/h, so it overran its stop at 60 km/h onto a switch set the other way; the game re-pathed it
  across Rubber SNK's claim. Braking is speed dependent (2.4 m/s2 at 100 km/h, 3.5 at 60 km/h for that train) and
  the single running average picked up the low-speed end of every stop. Now braking is measured per 10 m/s band, as
  force (deceleration x consist mass) so loading cargo counts, weaker samples weigh more, and a stop plans with the
  weakest band at or below its speed (75% of the weakest for an unmeasured band above all measured ones). Samples
  need full brake at both ends of the interval, and the interval is the real time since the last update, not the
  nominal 0.25 s (frame hitches inflated the measurement).
- The catch never armed for that train: it only armed with the authority track under the MU master, and on a long
  train the front reaches the stop point first. It now arms whenever the stop point lies before the master's next
  connection, with the stop offset set to put the stop there.
- at test 3 waited ~7 s twice for trains crossing in front with full speed up to its stop point. A train blocked by
  a moving train that will clear (GetBlockerClearSeconds) now caps its speed at the one that reaches the braking
  point as the blocker clears: v = a (sqrt(T^2 + 2D/a) - T).
- Full brakes "behind a leader" at 0 m with the path clear (Circuit Board Gather Train at 121 km/h): crossing
  traffic counted as a leader whenever its route didn't run over the follower's track, so a train queued behind at
  a crossing (Heavy Oil Residue, which had started because Circuit Board was clearing in time) became the leader of
  the train it was waiting for, and the follow cap stopped Circuit Board on the crossing. The same made AI Limiter
  RSC give up its claims and go first again four times a second. Now a train leads another only if it's ahead the
  same way, or the other recorded following it when it claimed (`FollowedTrains`, dropped once nothing is shared);
  two trains never follow each other; and a crossing leader caps the follower at the crossing, without adding the
  leader's braking distance (that only applies to a leader moving on ahead the same way).
- Brake samples up to 12.65 m/s2 against a rating of 2.6 (catches, contact with other trains): a sample now counts
  for at most 1.25x the rating.
- Next session: 12 full brakes in 8 min (from 70), no deadlocks, one pass-through burst. at test 2 and Heavy Oil
  Residue (120) crept up to 9-14 m behind a stopped train with no follow cap, then gave up their claims at speed
  (and at test 2 overlapped AI Limiter RSC: 565 pass-throughs). The train in front had counted the one behind it
  as "clearing in time" (IsClearingInTime didn't check order), so it recorded the follower as its leader, which in
  turn kept the real follower from following. `GetRouteOrder` now decides who is ahead on a shared route; a holder
  behind Self on its own way is never clearing in time, and IsSameWayLeader needs the holder ahead.
- Remaining "behind a leader" brakes: the moving-block cap gave the leader its own (weakest) planning braking
  distance from a 0.25 s old speed, so a leader braking normally (harder at lower speed) shrank the cap faster than
  the follower could follow. The leader's distance now uses its live speed and the strongest braking a sample can
  count for (1.25x rated).
- Two full brakes right after a 57 s pause (the game didn't tick): trains moved past their authority in one step.
  Not a logic bug.
- Next session (15 min): no deadlocks, 22 full brakes, 4 pass-through bursts. Both overlaps were at stations: EBeams
  GTHR Train followed MF Resource Train into Encased Pipe SRC, and Steel Pipe Xtra Steel followed Train into
  Encased Pipe Conk/Steel SNK, while the first train docked. IsSameWayLeader followed a train into its own station,
  and a docked train is unmanaged, so it stopped being anyone's leader: the follow cap vanished and the follower's
  authority already ran to its goal. Now a train isn't followed the same way when its goal track is on the
  follower's route; a train this mod doesn't drive (docked, player driven) standing on claimed track counts as a
  stopped leader (claims dropped from there) and as an obstacle for the follow cap; and the follow cap measures the
  gap to the nearest vehicle of a leader on the follower's path (`Vehicles`), not the leader's snapshot tail.
- The follow cap is now two-level: the speed cap gives a leader moving on ahead its braking distance with the
  strongest braking a sample counts for, and full brake only comes when over the curve with the leader's planning
  braking. The strict cap alone put followers over the curve whenever they followed closer than it allowed.
- Next session (12 min, no telemetry): no deadlocks, no pass-throughs, 11 full brakes. The previous session's
  telemetry shows the "behind a leader" ones aren't a slowly tightening cap: MF Resource Train at 107 km/h had a cap
  of 141 km/h, then a leader appeared ~150 m ahead inside its own claims within one tick. Rerouting isn't it (75
  reroutes, authority never dropped). The full-brake line now names the binding leader, the gap (to its nearest
  vehicle on the path, or to the first track it holds), the leader's braking distance and how long it has been a
  leader (`FollowLeaderSince`), to find where such leaders come from.
- at test 2 (112 m) waited at the roundabout for Adaptive Control Unit GTHR Train A to leave the same platform:
  correct, as the ramp between the junction and the station is ~85 m. But the log said ACU A stood on the ramp track:
  a vehicle within HalfLength + 2 m of a track end counted as on every attached track, and station tracks are exactly
  one vehicle long, so every docked train's ends spilled onto the next track. A vehicle now counts on the next track
  only once it reaches 0.5 m past the end; switches stay covered by the 20 m fouling zone.
- With the new full-brake detail (9 full brakes, 1 pass-through burst in 13 min): every "behind a leader" one had
  "its braking 0 m". (1) The leader's braking distance counted only when its master's track was on the follower's
  route, so a leader turning off the route (its tail still ahead on the follower's path) counted as standing still;
  now every managed leader whose route doesn't run over the follower's route the other way moves away
  (`LeadersMovingAway`). (2) A leader travels no further than its authority (or goal) before it stops, whatever its
  braking, and the catch can stop it there dead; its distance is now capped at that (`GetLeaderTravel`). (3) A leader
  by route order can at the same time be following this train through a crossing (Heavy Oil Residue [Coke Steel] and
  Coated Plates RSC B); its claims there only trail this train, so the held-track fallback skips leaders that follow
  this train and only their vehicles count.
- Cable RSC into AI Limiter RSC: Cable planned with 2.60 m/s2 at 100 km/h but managed 1.2-1.5 there (5-6 below 40
  km/h), and its brakes took about a second to build up. Its 72-108 km/h band had been measured at ~70 km/h. Braking
  is now measured in 2.5 m/s bands; a stop's distance adds up each band's planned deceleration plus 0.75 s of brake
  build-up at the starting speed (`GetStoppingDistance`, inverted by `GetAllowedSpeed`); a band above every measured
  one plans with the highest scaled down in proportion to speed (not below 0.4x rated); a train never measured plans
  with 0.8x rated up to 50 km/h, scaled down in proportion above.
- Forced braking (the user's idea, like vanilla block signals): when a train is over its full-brake curve and the
  deceleration it needs to stop at its stop point exceeds its plan, ApplyAuthority takes the difference off
  `mSimulationData.Velocity` each tick. Brake samples from intervals with forced braking are skipped.
- Settings Top speed (0 = own) and Acceleration (x) in ApplySpeedSettings, after ApplyAuthority: TargetSpeed capped
  at the top speed, or raised to it when the driver runs flat out at the locomotive's own top speed; above that, or
  scaled by the multiplier, speed is added to `mSimulationData.Velocity`. Timing uses the top speed and scaled
  acceleration.
- Acceleration now scales the locomotives themselves (`ApplyLocomotivePower`, every update): the tractive effort curve
  (copied into the component's own curve, keys and tangents scaled), `mMaxTractiveEffort` and `mPowerConsumption`
  (min and max), then `AFGTrain::RecalculateConsistData`. Power draw follows throttle as a share of the scaled
  maximum, so it scales with traction. A locomotive whose `mMaxTractiveEffort` no longer matches what was set is
  taken as reset by the game and captured again. Restored when traffic control goes off, self-driving goes off, or
  the world ends; nothing is saved. Log: `locomotives at xN traction and power draw`.
- Forced braking flapped every tick: it took off just enough to fall under the full-brake curve, switched off, and
  came back the next tick (321 log lines). It now holds, with full brake, until the train's own brakes suffice.
- Remaining full brakes all had "first track it holds, its braking 0 m": a crossing leader the train followed held
  the crossing ahead (its claim) until it had passed it, so the follower braked to stop there although the leader
  clears it long before. A crossing leader is now ignored while it clears the crossing before the follower, at its
  speed now, gets there (`IsCrossingClearedFirst`); if it stops, the stopped-leader rule drops the claims.
- Acceleration x2 applied (tractive effort 2000 -> 4000 kN per train), but measured acceleration rose only x1.5 at
  20-60 km/h, x1.4 at 60-80, x1.2 at 80-100 (resistance is unchanged, but that doesn't explain low speed). Telemetry now
  has `throttle` (master locomotive), `tractive_kN`, `brake_kN`, `resistive_kN`, `gradient_kN` to see whether the
  game's driver holds the throttle back or the force is capped.
- Deadlock Coated Plates RSC Train -> Coated Plates RSC B -> MF Resource Train: MF followed CP Train (same way) over
  the crossing CP B needed to leave its station, CP Train's authority ended just past it (waiting for CP B), so MF
  stopped behind it on the crossing. Most remaining full brakes were the same: a train followed a leader whose
  authority didn't take it past the shared track, the leader stopped, the follower braked hard. Following (same way
  or clearing in time) now needs the leader's authority, or goal, to take its tail past that track or switch
  (`IsClearedPast`).

- Session 2026-10-27 (IsClearedPast build), 50 min: 2 deadlocks, 44 full brakes. 35 of the full brakes were behind a
  crossing leader at 100+ km/h: `IsCrossingClearedFirst` ignored the leader while the follower, at its speed then,
  got there after it, the follower sped up, the test failed and the follower braked to the crossing, then slowed and
  the test passed again (full brake flipping every tick, e.g. Cable RSC behind Coated Plates RSC Train). Now a leader
  clearing the crossing first sets a pace instead (`GetCrossingPaceSpeed`): the follower keeps to the speed that gets
  it there 1 s after the leader has cleared it plus the safety margin, so the test keeps passing.
- Deadlock MF Resource Train <-> AI Limiter RSC ("-287 m ahead", forced braking at 333 m/s2 from 93 km/h): AI
  Limiter's route came back later over a track its tail was on, `ReadTrain` measured the front from that later pass
  (`FrontAhead` 639 m), so its authority fell to 0. `FrontAhead` now only counts route tracks within the train's
  length. `GetAuthorityDistance` finds the authority track at its own path segment, not an earlier pass over it.
- Deadlock Coated Plates RSC Train <-> Coated Plates RSC B: CP B rerouted onto a route that came back over the track
  where CP Train stood waiting for it. A train waiting for the train being routed now costs 100 km
  (`WaitingForSelfPenalty`), so a route through it is only taken if there's no other.
- Top speed 200 km/h had no effect on flat track: the vanilla traction curve is 4000 kN (at x2) up to 50 km/h and
  zero from about 120 km/h, while `mConsistData.MaxSpeed` is 200, so the old speed boost (above MaxSpeed) never ran;
  trains above 120 km/h were all going downhill. The setting now stretches each locomotive's traction curve over
  setting x 1.05 / the speed where it runs out (`GetTractionTop`), scales its power draw by the same factor (traction
  power is force times speed), and raises every vehicle's `mMaxVelocity` to the setting; the game's driver aims for
  the consist's top speed and the setting caps it. Nothing adds speed directly any more.
- Acceleration x2: tractive force is 4000 kN x throttle at low speed, so the scaling applies. The game's driver opens
  the throttle at about 0.2 per second and holds it around 0.5-0.75 while the target is near, which is why measured
  acceleration rose less than x2.
- Pass-through reports now name the trains on the tracks of the last contact. The burst at 04:45:20-30 had no two
  trains on the same track (switch or crossing).
- Session 2026-10-27 05:19: MF Resource Train and AI Limiter RSC were nose to nose on a short bidirectional line
  from load: the save held the deadlock of the previous session (04:20), which never cleared (trains can't back
  out) and was cleared by hand. It formed when AI Limiter rerouted (04:20:22) onto a route running the other way over
  track MF held, its stop, and MF rerouted 2 s later onto one running the other way into AI Limiter's claims; each
  was committed into the line from its own end. Reroute congestion counted another train's claim as 300 m whichever
  way it ran. A new route now costs 100 km per track it runs against another train's held route
  (`HeadOnPenalty`, `FCongestion::HeldExits`); the current route doesn't, so single track with passing loops keeps
  working. The pass-through at 05:29:33-53 (Coated Plates RSC Train / at test 3) was at test 3 driven by hand.
- Same session: forced braking 15 times in 13 min, mostly at 150-180 km/h needing only 1.1-1.5 m/s2, against plans
  of 0.6-0.7 m/s2. Those were the never-measured fallback, 0.8x rated scaled down in proportion above 50 km/h. Measured
  in telemetry, full brake gives 2100-2500 kN at every speed (air brakes, rated force, plus dynamic brakes, about
  2000 kN below 40 km/h and 400-600 above 120) and 2.3-3.4 m/s2 at 120-180 km/h. The rating
  (`MaxBrakeDeceleration`) is the air brakes' force over the consist mass, and the consist mass leaves out cargo
  loaded later: at test 4 braked at 2.4 m/s2 with 2500 kN, about 1050 t, against a consist of 730 t. That, not speed,
  is why loaded trains measured 1.2 m/s2 at 100 km/h earlier. The plan now uses the vehicles' own mass with cargo
  (`FATTrafficTrain::Mass`, also for acceleration and brake samples), and plans an unmeasured band with 0.8x the air
  brakes' force over that mass, at every speed (dynamic brakes are left as margin); above every measured band no
  more than the highest measured. Telemetry has `mass_t` (vehicles with cargo).
- Reinforced Plates Train was forced from 92 km/h to a stop in a second (181 m/s2) while pacing behind Cable RSC
  crossing at 150 km/h: Cable slowed, its clear time (distance / speed now) grew, the pace became too fast and the
  test failed 27 m from the crossing. The pace and following in time (`IsClearingInTime`) now take the latest the
  leader can clear it: slowing evenly to stop at the end of its authority (`GetLatestClearSeconds`), which only
  gets sooner as time passes.
- Session 2026-10-27 06:04 correction: `mass_t` (vehicles with cargo) equals the consist mass for every train, so the
  consist mass does include cargo and the cargo explanation above is wrong; the plan's air-brake share is still
  right (steady full-brake deceleration after 1 s is 2.1-3.2 m/s2 at every speed). The low first samples (0.6-2.0)
  are the brakes building up in the first 0.5 s, which `BrakeLagSeconds` already covers.
- Adaptive Control Unit GTHR Train A waited 31 s at a busy junction: its next safe stop was 590 m (32 tracks) on,
  and Rubber SNK, Coated Plates RSC B and AI Limiter RSC kept claiming parts of that stretch in turn; waiting time
  only ordered the claims within each update. Fair turns: a train that has waited 12 s for another train records the
  tracks and switches to its next safe stop (`UpdateRequest`); trains that have waited less don't claim them, or
  tracks crossing them (`FindRequester`), unless they hold one of them already, or the waiting train waits for or
  follows them. The request lapses after 90 s of waiting, in case it is what holds the train up. Log: `has waited N
  s; other trains leave its way free`, and `lets X through first`.
- at test 2 full brake at 98 km/h behind Cable RSC, which had turned off into its station: Cable's tail overhung a
  switch onto at test 2's branch, a track off Cable's route, whose clear distance was taken as Cable's length plus
  20 m (84 m) against 60 m left to its stop, so Cable "might stop on it". The clear distance of a track off the
  route now comes from the vehicles: the rear of the last one on it past its forward end, or the overhang ending
  (`GetTailClearDistance`).
- Most of the 40 forced brakes were trains driving on the full-brake curve itself: the target was the curve, the
  game's driver brakes slightly behind its target, went 1 m/s over, and got full brake with forced braking needing
  about the planned deceleration. The target now runs 0.5 s ahead of the curve (`TargetLeadSeconds`).
- Session 2026-10-27 06:26, 12 min: no deadlocks, no full brakes over the curve. Fair turns held 18 times (e.g. Heavy
  Oil Residue [Coke Steel] and Circuit Board Gather Train for at test1, Train and at test 2 for Coated Plates RSC
  Train); the longest wait, Coated Plates RSC B 42 s, queued for Coated Plates RSC Train at their shared station.
  Forced braking still came on and off within one stop (at test 3 five times from 67 to 16 km/h, needing 1.4-1.7
  m/s2, its plan): the target followed the full-brake curve, so the game's driver braking gently drifted over it.
  The target now follows a service-brake curve at 0.75 of the planned deceleration (`ServiceBrakeShare`, also for
  pacing behind a clearing blocker); the full-brake curve only decides full brake.
- Pass-through reports named "?": the trains were looked up at the 10 s report, after they had moved. The names are
  now taken at the traffic update after the contact.
- Rerouting is timed instead of priced in metres (the user: optimise everything for time). `ConsiderReroute` times the
  current route and searches the fastest one (`FATRouter::FindFastestRoute`, Dijkstra whose step cost gets the arrival
  time): each track at the train's top speed, plus the wait for other trains on it or on a track crossing it while
  this train would be (`BuildCongestion` gives each one's occupancy in seconds from now: standing until expected to
  move off, moving until its tail is off, its route ahead timed at its speed picking up to its top speed), plus the
  stop cost (braking and accelerating again, `GetRouteSpeed`) up to the wait. A train ahead the same way costs a 2 s
  headway. A train waiting for this one, and for a new route track held the other way, count 100000 s. A reroute
  needs 10% and 10 s.
- Session 2026-10-27 07:01 (time-based rerouting), 34 min: dockings per train-hour 29.3 against 29.6-30.9 before, so
  timing routes didn't change throughput, with 6 reroutes in 34 min against 80-90 per 12 min priced in metres. Mean
  speed off stations fell from 114 to 98 km/h already in the session before, with the 0.75 service-brake target
  (earlier braking into every stop); `ServiceBrakeShare` is now 0.85.
- Five forced stops at 50-130 km/h within 0-10 m ("first track it holds 0-10 m", up to 669 m/s2), e.g. Coated Plates
  RSC B behind at test 3: the two routes shared a switch, not a track. The follow limit's held-track fallback only
  looked at tracks the leader holds, so nothing paced the follower until the leader's vehicle reached the switch and
  overhung onto its track a few metres ahead. The fallback now takes the first switch on the route the leader holds
  too, paced the same way (the leader's clear distance through the switch, from its route or from its vehicles in the
  fouling zone, `GetTailClearDistance` with a joint), stopping before the switch's fouling zone.
- Adaptive Control Unit GTHR Train B stood 25 m before a switch (a valid stop) while Coated Plates RSC B passed at 150
  km/h on the other branch: 82 contacts in 10 s. Near a shallow switch the branches stay closer than a vehicle's
  width well past the fixed 20 m fouling zone. `RebuildConflicts` now measures each switch: for every pair of
  branches leaving it the same way, the distance along them until their centre lines are 5.5 m apart (up to 150 m,
  at least 20 m) (`GetFoulDistance` per connection, `GetJointFoulDistance` per joint). It's used for vehicles fouling a
  switch, stop margins before a switch (fouling zone + 10 m, at least 25 m), clear distances and valid stops. Log:
  `longest fouling zone N m`.
- Review after the 07:01 session: (1) `RevalidateFollowClaims` dropped shared claims the follower could no longer
  stop before (AI Limiter RSC gave up 4 claims behind a stopped at test 3 and was over its curve 11 ms later); it now
  keeps claims up to the first stop beyond its stopping distance, the follow limit holding it behind the leader.
  (2) A same-way leader counts for following only if the gap to its tail plus its travel before it could stop covers
  the follower's stop (`IsSameWayLeader`), as the follow limit then requires: Cable RSC followed Heavy Oil Residue
  [Coke Steel] pulling away from standstill 0 m ahead and was forced to stop. (3) Fair turns also exempt a train that
  a train in the waiting train's way waits for, so that one can clear the way.
