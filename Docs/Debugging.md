# Debugging

## Logs

The game log is `%LOCALAPPDATA%\FactoryGame\Saved\Logs\FactoryGame.log`. Filter for `LogAutoTrains`.

- startup: `Auto Trains module starting`, `Hooks installed`
- every `/at` command and its result
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

- `speed_kmh`; `game_target_kmh`, the speed the game's driver wanted; `final_target_kmh`, after this mod's control
- `accel_ms2`, from the speed change between samples
- `odometer_m`, distance travelled since the train was first seen
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
- Milestone 2 started (space-time planner, see the plan file): `ATPlanner.cpp`, shadow mode only. Resources are
  tracks (with crossing tracks as conflicts) and switch joints (geometric fouling zones); a pass is the front
  position where a train enters a resource and where its tail has cleared it, timed by a motion profile (legs between
  stops: accelerate at half the tractive acceleration to top speed, brake at the service deceleration; the goal is
  booked open-ended because the dwell is unknown). Update: trains not driven or docked book what they stand on
  open-ended; moving trains book what lies within their stopping distance (committed); then in claim-priority order
  each train is planned: the first conflict (overlap plus the safety margin and 2 s drift) puts a wait at the last
  track end (stop margin short) before the resource so it enters after the booking clears; up to 32 waits.
- A detected deadlock now has every train in the cycle check for another route at once (cooldown reset); routes
  through a train waiting for it count as impassable, so a cycle with an alternative way out can unlock itself.
- Planner shadow: two passes the same way on the same track are compatible when the later one enters and clears a
  headway after the earlier one (following without overtaking); opposite ways, crossings and switches stay
  exclusive.
- Planner step 3 (user, 2026-10-27: a rigid foundation instead of the conditional claim rules; space-time SIPP
  routing, updated continuously; dwell times and timetables are known): shadow mode was never run in game and is
  gone. The planner now drives behind a toggle. Why each piece:
  - The reservation table persists between updates and each train only replaces its own bookings, so a booking is
    never taken by someone else. That keeps the order on every resource consistent with one timeline, which is what
    rules out precedence cycles (deadlocks) and starvation; times may drift, order doesn't.
  - Execution follows booking order, not times: a train enters a track or switch only once every train booked there
    earlier has passed, and nobody stands there (same-way following excepted, kept apart by the follow limit). Late
    or early trains then only wait; they never meet.
  - Docked trains book their predicted departure (timetable next stop, `FRailroadNavigation::FindPathSync` from each
    locomotive, shortest) ahead of moving trains' unheld bookings, so no train queues in a throat the departing
    train needs (the station-throat deadlock risk of 1.0). Stays are learned per train and station.
  - The goal is booked until arrival + expected stay + pulling the train's length out, not open-ended, so others can
  plan through stations.
  - Rerouting uses the same table: time-dependent earliest arrival with waits, comparing both routes with the same
    estimator.

- Planner on by default (user, 2026-10-27); the setting's key is now PlannerControl so saved "off" values of the first
  key don't carry over. Updates slower than 20 ms are logged.
- First planner run (16:49, 6 min): 2.5-3.4 ms per update, no deadlocks, but 118 forced brakings (claims: 74 in 46 min)
  and a 320 m/s2 stop. Cause: a train could book into a gap ahead of another train's booking even after that train was
  granted it or committed to it, flipping the order (Reinforced Plates Train lost a crossing it had been granted to at
  test1 10 m before it). Fix: locked bookings (stood on, held from the last grant, or within the stopping distance):
  nobody books ahead of one, the owner's locked passes aren't re-checked against others' unlocked bookings, and at
  grant time locked bookings go first. The route search treats them the same way.
- Second planner run (17:15, 11 min): 2-3.4 ms per update, no deadlocks, the user saw no emergency stops, but the log
  had 164 forced brakings (11 over 50 m/s2, up to 834) and trains not taking gaps they could have. (1) A booking was
  locked by the owner's stopping distance even where it had no grant, so two trains could be locked onto crossing
  tracks and the one with the earlier time won over the one already granted (Heavy Oil Residue (120) lost a crossing
  8 m before it to at test 3). Now a train is only granted a resource nobody else holds (stands on or was granted),
  apart from following the same way: holding excludes, the bookings only order. A check logs `CHECK FAILED` if two
  trains ever hold conflicting resources. (2) Locks were too broad: everything held from a grant was locked, so no train
  could fit in ahead of a far-off holder. Now only what the owner stands on or couldn't stop short of at service
  braking is locked (1.2 x stopping distance + 1 s + 60 m); what it holds beyond is held: others may fit in ahead and
  the holder then gives it up with ordinary braking. (3) Plans assumed the 200 km/h top speed setting while trains
  average about 92 km/h, so far-off trains looked much closer in time and others held back for them. Plans now use
  each train's cruising speed and acceleration as seen. Wait reasons show the blocking booking's lock and entry time
  and are logged only when the blocker or place changes.
- Cleanup (2026-10-27): the 0.2 control-test commands (`/at speed`, `stopat`, `release`, `signals`, `route`) and what
  only they used (per-train command modes, the route check, `FATRouter::FindRoute`, `GetConnectionAhead`,
  `CompareWithCurrentPath`, `Describe`) are gone; telemetry drops the `command`, `ignore_signals` and
  `stop_remaining_m` columns.
- Third planner run (17:30): followers crept behind trains on open track ("waits for AI Limiter RSC ... locked, clear in
  0 s" with the leader moving away). The tail's bookings behind a train's route had no direction, so following never
  matched there and a follower waited for each track to be left entirely. CollectPasses now walks the tail back from
  the route's first track and books each track and switch with the way it is left. Heavy Oil Residue (120) stood after
  a 45 m/s2 stop (AI Limiter RSC held a track 34 m ahead of it at 167 km/h), its plan failing at 32 waits and
  rerouting between three routes every 20 s: waits are now capped at 64, a route left in the last 2 minutes isn't taken
  again, a train the planner names a blocker for counts as waiting for deadlock detection wherever it stands, and a
  path the game gives a driving train is logged (`got a new path from the game`), the suspected cause of the stop.
- Fourth planner run (17:49): gridlock in the double-roundabout interchange, DEADLOCK cycles (EBeams GTHR Train <->
  AI Limiter RSC; Rubber SNK [conc] -> Cable RSC -> Steel Pipe Xtra Steel), and Coated Plates RSC Train waiting 10 min
  for Adaptive Control Unit GTHR Train B's booking that always said "enters in 34 s". Cause: everything within the
  lock distance (60 m for a standing train) was locked, held or not, and a train's locked passes skip the conflict
  check; so a standing train planned straight through resources it didn't hold, booked what lay beyond far too early,
  and others yielded to those phantom times. Now only what a train stands on, can't stop short of, or holds within
  reach of service braking is locked.
- Deadlocked trains show the game's long-wait warning ("!"): RE'd `AFGTrain::TickSelfDriving_FollowPath` counts
  `mSelfDrivingData.TimeWaitingAtSignal` while `NextRestrictiveSignalAspect` is Stop and the ATC speed is about 0, and
  reports `SDLE_LongWaitAtSignal` past `mLongWaitAtSignalThreshold`; `TickSelfDriving` writes the error every tick, so
  it can't be set directly. `AFGRailroadSubsystem`'s tick runs `TickAtc` (the mod's hook) before `TickSelfDriving`.
  A standing train in a detected cycle gets a Stop restrictive point where it stands and the wait timer at the
  threshold; when the cycle breaks the aspect goes back and the game clears the warning.
- Fifth planner run (18:06): no DEADLOCK logged, yet Steel Pipe Xtra Steel stood 31 m before switch 179 for 7 min
  waiting for Cable RSC ("held, enters in 7 s") and Rubber SNK [conc] ("planned, enters in 35 s"), times that never
  moved, while Cable RSC waited for Steel Pipe Xtra Steel at the same switch (the trio of the earlier deadlock). Waiting
  time reset whenever a train rolled faster than 1 m/s (slowly creeping up to its stop), so the cycle was never
  detected. Now, with the planner, a blocked train slower than 3 m/s counts as waiting. A detected cycle with a member
  that only waits for the booking order gets that member going first for 30 s: its route ahead (stopping distance,
  length and 400 m) is locked, still kept clear of other locked bookings, so the others plan behind it
  (`goes first ... to break the deadlock`). Trains that waited over 60 s are described every 30 s (`planner: WAIT`):
  their booking at the resource, and every booking there with its train's state.
- Sixth planner run (18:24): no DEADLOCK although 8 trains waited 2-5 min; the dumps showed the trains they waited for
  standing at 0 km/h with 7-11 granted tracks and no reason of their own, so no waiting chain closed. (1) MF Resource
  Train lost a track it held at 144 km/h (799 m/s2) to EBeams GTHR Train, whose booking was locked only because it
  couldn't stop short of it, and both ended in switch 9's fouling zone holding each other back by their follow limits.
  A train now keeps what it held from its last grant and has locked; no booking outranks that, only a train standing
  there or holding it. (2) A train held back only by its follow limit now names its leader (`stays behind`), so such
  cycles are detected and shown. (3) Following onto shared track is granted only while the leader moves (3 m/s) or
  isn't blocked: followers granted behind a standing leader held crossings and switches for others while standing.
  The wait dump shows each train's authority, target speed and follow limit.
- The claim system is gone (user, 2026-10-27: "strip it ... keep it in the GitHub history"); the planner is the only
  mode, without its toggle or `/at planner`. Removed: claim extension, valid stops, preemption, fair turns, follow
  revalidation, congestion rerouting and their settings' effects; Aggressive timing now sets the braking shares and the
  waiting weight in the planning order, and the safety margin adds to the bookings' headway. The last claim-system
  build is commit 612b43d.
- Seventh planner run (18:40): trains crept and stopped in turn: at test 2 flipped every update between `stays behind`
  and `waits for ... stands on` its leader 10 m ahead, because following was granted only while the leader moved or
  wasn't blocked, and the leader creeping around those thresholds flipped it. Following the same way is granted again
  whatever the leader does (the follow limit keeps the distance). The deadlock breaker no longer promotes a train whose
  blocker has the resource locked (it stands there or can't avoid it): Rubber SNK [conc] and Cable RSC stood in each
  other's way, Rubber's tail across Cable's path, which only moving one of them by hand can solve.
- Root cause of the repeated deadlocks (eighth run, 18:56): nearly every cycle was a pair on one line, A behind B: B
  "waits for A before track X, 9 m ahead: holds track X / its plan has no slot" and A "stays behind B, 10 m ahead"
  (Rubber SNK [conc]/Cable RSC, Reinforced Plates Train/at test 3, ACU GTHR B/at test 2, Train/Circuit Board Gather
  Train, Coated Plates RSC Train/at test1, at test1/at test 3). The plan times a train's motion ignoring the trains
  ahead of it on its line, so while B had a wait ahead, A's plan reached B's next track first (overtaking it in time),
  A was booked first there, and the following rule let A be granted it behind B: B then waited for A's hold, and A
  for B physically. Fix: along one line the order is fixed. Planning: another train whose front stands on this route,
  the same way, comes first on every booking it has at that point of the route or later (nobody books ahead of it);
  one whose route has this train's front on it before a resource has to pass this train first, so its booking there
  doesn't count. Granting: the same, and a hold by a train behind on the same line doesn't block. Routes are indexed
  (track and switch -> route step) every update.
- Review of rules added for misdiagnosed problems, removed: the deadlock breaker ("goes first", 90 promotions across
  the logs, the same trains every 30 s, none freed: the cycles were the physical overtaking jams, and with the order
  fixed along each line the bookings can't form order-only cycles); not going back to a route left in the last
  2 minutes (Heavy Oil Residue (120) rerouting in circles was a symptom of it being stuck); the log of the game giving a
  driving train a new path (never fired in 10 sessions: wrong guess); waits per plan back to 32 (failures still happened
  at 64).
- Ninth run (19:13): at test1 at 127 km/h was granted past a crossing (500 m) and lost it again every update or two
  (187 m), forced braking each time; Coated Plates RSC B had been granted the crossing track and then had to stop on
  it, waiting for at test1's tail to clear the track it needed next, which at test1 couldn't clear without crossing.
  Track a train holds isn't re-planned, so once something ahead stalled it drove on and stood on the crossing. A
  merge switch held by a stopped train's tail is the same thing. Fix (safe stops): a held-up train's grant ends only
  where it stands clear of every track (and crossing track) and switch zone another train is booked on before the
  train can move on (trains queueing behind it the same way, or that have to pass it first, don't count); otherwise
  it is cut back to the last such place it can still stop at, and the train waits before the junction.
- The safety margin is the whole headway between bookings (user, 2026-10-27): no extra seconds on top.
- Tenth run (19:25, build c2c98d5): hard stops again (Rubber SNK [conc] 455 m/s2 at 147 km/h, Heavy Oil Residue [Coke
  Steel] 369, AI Limiter RSC 52 at 192 km/h), all at the crossing cluster around track 2146621121, each waiting for a
  train that "stands on" a crossing track, and CHECK FAILED every ~20 s. The line-order rule's "behind" test only asked
  whether this train's front lies on the other's route before the resource; where routes share a stretch and then
  cross further on, a crossing train counted as behind, its hold on the crossing was ignored, and both were granted
  it. Now a train is behind only when both routes run over the same tracks from this train's front up to the
  resource.
- Rerouting made conservative (user: prefer the usual, shorter route): a new route must save 30 s and 20%, and is
  kept 60 s. These, the lookahead past the braking distance, the stop distance, the normal braking share and the
  station stay estimate are now Mods menu settings (number settings are one table in ATConfig.cpp).
- After 1.0.0 (telemetry 14:22-14:49, build c2c98d5): with the game wanting >= 60 km/h, the mod held trains below 80%
  of the game's target 70% of the time. (1) Grants reached the stopping distance at the current speed plus length
  plus 100 m, and the speed was capped to stop within that at normal braking: at 200 km/h the cap was about 1% over
  the speed, so the game's driver barely accelerated (AI Limiter RSC at 138 km/h, 529 m granted, capped at 139). Grants
  now reach the stopping distance at the speed the train is heading for (its cruise). (2) Everything held within
  1.2 x the stopping distance + 1 s + 60 m was locked, about 640 m at 150 km/h, so no crossing train could go first
  even when long clear before the fast train arrived. Locked is now only within the full-brake stopping distance plus
  the stop distance; beyond, held track can be gone ahead of, and the holder brakes if it must.
- 1.1.0 (user, after the 1.0.0 log showed 1,026 of 1,127 planned-booking waits were for trains at least 10 s away):
  the timetable comes from a safe-interval search (`PlanSafeIntervals`): stops are the front's position, each route
  track's end short of its stop margin, and the goal; standing at a stop must keep every resource the body covers there
  clear of other bookings (`CollectBlocks`, same rules as the conflict check: locked and line-order trains first,
  trains behind on the line skipped), and each move every resource it touches; labels per (stop, free window) keep the
  earliest arrival. The old greedy loop only repairs what the simplified timing (cruise between stops, a stop loss
  after a wait) missed. Removed as superseded: safe-stop trimming; a train is now granted no further than its plan's
  next wait, which the search only puts where the body stands clear. Kept: line order, locks/holds, the follow
  exception, tail directions, wait dumps and checks.
- Ghosting (beta, user): a deadlock cycle lasting longer than the setting has its trains granted their way regardless,
  follow limit off and pass-through on, until each drives its length + 300 m (2 min at most); `faults.log` gets the
  trains and their reasons. `planner: CROSSING:` logs any train standing on a track crossing one another train holds,
  to find the remaining hard-stop cause.
- 1.0.0 session (20:05-21:15) deadlocks: (1) Steel Pipe Xtra Steel/EBeams GTHR Train and Coated Plates RSC
  Train/ACU GTHR Train A: the rear train, 0 m behind, had been granted (through the follow exception) a track past the
  front one, its own booking there 45+ min away; the front one waited for that hold. 1.1.0 grants only up to the
  plan's next wait, which is behind the front train. (2) AI Limiter RSC standing in merge switch 123 wasn't allowed
  on ("would stand on ... which that train is booked on"): safe-stop trimming counted every future booking when the
  train had no planned departure, including bookings that waited for it. Removed in 1.1.0; SIPP only avoids a
  standing place while another booking there overlaps in time. (3) Cycles dropped out of detection for single updates
  and were logged again each time (708 times) and, in 1.1.0, would have restarted the passing-through timer; a cycle
  seen again within 10 s is now the same deadlock.
- 1.1.0 session (16:52-17:13 local, SMR build): the user saw a train waiting with a clear way ahead. Circuit Board
  Gather Train crept at 1-17 km/h for over a minute (17:11:42 on), its reason alternating between a docked train's
  predicted departure ("booked first ... planned") and its own plan's wait for ACU GTHR Train A, whose plan waited for
  it in turn; the "plan moves off in" times kept growing instead of counting down. `at_mutual.py` (scratchpad): 175
  of 623 consecutive plan waits receded > 2 s. Cause: every train planned around all others' bookings, those of trains
  later in the turn order a whole update old, and the turn order (`GetClaimPriority`: time to clear the next track
  minus waited time) changed every update, so two plans could each wait for the other. Fixed in 1.1.1 with
  prioritized planning: a stable rank (`RankTrains`: ahead on another's route first, else `LegStart`), and
  `EConflictScope::Ranked` skips lower-ranked trains' neither locked nor held bookings (planning and router step cost).
  The one deadlock (Circuit Board/Heavy Oil Residue [Coke Steel], ghosted after 61 s) had both fronts on track
  2146636522; `FrontIndexOn` found each on the other's route at step 0 and so each treated the other as ahead and
  first. It now checks which front is nearer the track's end, as `GetRouteOrder` already did.
- 1.1.1 session (18:09-20:04 local): receding mutual waits mostly gone; 3 deadlocks ghosted. 18:12 was inherited from
  the save: under 1.1.0 at 17:45 Reinforced Plates and MF Resource each drove onto one of two crossings
  (2143118901/2142548751) and needed the other, then sat "waiting for ?" (no cycle seen, no ghosting) for 22 minutes.
  19:47 (MF Resource leaving J-Spires R-Plates SRC, Reinforced Plates heading in, single track 2147407495): once MF's
  grant stopped short of 2147407495 its booking there was unheld, so the higher-ranked Reinforced Plates ignored it,
  planned to drive onto 2147407495 and wait at its end until MF left the track MF stood on, which MF could only leave
  onto 2147407495; both waits receded to 2669 s. Same family: hard stops (~20/h, at 100+ km/h with 0 to -30 m left,
  `at_overrun_why.py`): 20 of 22 "stands on" a crossing track whose other train was late. Fixed in 1.2.0 with
  committed bookings (`MarkCommitted`, respected under every scope). Ghosting trains no longer log CHECK FAILED or
  "can't stop short". Not yet explained: 19:34 four-train cycle of plan waits (Coated Plates RSC Train, Coated Plates
  RSC B, ACU GTHR A, Steel Pipe Xtra Steel), "no plan after 32 waits" repeats for Reinforced Plates and Rubber SNK.
- 1.3.0 test build (0abd703), 2026-10-28 01:19-01:36 local: searches took 0.3-2 ms on a worker (265-2232 states), but
  most reroutes said `no way` for the current route and were applied, up to 61 -> 128 tracks. Causes in the search
  model: the current route was timed greedily (first window at each track, a dead end where a later window with a
  later end would have worked); same-way trains through a switch counted as crossing; trains behind on one line
  (`IsBehind`) were counted and trains ahead on the route didn't go first, unlike `FindConflict`; and the start had
  no body deadline, so a waiting train could be sent onto a route that kept it standing where another train needed
  to pass. 01:21:21 Heavy Oil Residue [Coke Steel], waiting at the end of 2147206932 with a plan, was rerouted that
  way into Heavy Oil Residue (120) [Gunpowder]: deadlock, ghosted after 60 s. Fixed: the current route is searched
  like any other (restricted to its steps), joints get the same-way rule (`Exit`/`Via`), bookings are filtered as in
  `FindConflict`, `Request.Holds` limits the wait at the start (any rank; their `Enter - H - reach / cruise`), a
  train whose plan has a finite arrival keeps its route when the search finds none (`keeps its route` line, once
  per place/train), and `no way` lines say where and why (`FATSearchBlock`).
- 1.3.0 planner (full multi-train SIPP, ATSipp): the per-update PlanTrain/PlanSafeIntervals pair (cruise + stop-loss
  timetable, then up to 32 corrective waits with FProfile) and the separate graph route search were two models that
  disagreed (false "no way", reroute storms, "no plan after 32 waits"). Now one search with the train's own speed
  profile is both: planning cycles run on a worker (`planner: last 60 s: ... N cycles, X ms each on a worker, N
  updates waited for one`), the cycle's table replaces `mBookings` when it lands (Tick), grants stay on the game
  thread. Failures read `no way on past <resource> for <train>`; reroutes `rerouted after ...: <old> -> <new> s to
  its goal, a -> b tracks ... (route search: n states, m ms on a worker)`.
- First ATSipp test (a9d7d4a, 02:43-02:58 local): every train came to a standstill; almost every plan "no way to its
  goal" with no train named, 187 bookings, cycles 2.4 ms. Causes: (1) the search never braked harder than service
  braking, so a train already too fast for its next stop (approaching its station) had no plan, stood for good, and
  its followers (same way, open Clear) failed after it; the old FProfile braked harder where it had to. Moves ending
  within the service stopping distance from the speed now may now brake as hard as needed. (2) A braking train under
  0.5 m/s at a track end too close to stop at had no move; passing speeds go down to a crawl. (3) 8 speed bands let a
  slightly faster state that can't stop prune a slower one that can; now 32. (4) Before the first cycle landed no
  train had a plan, so none was granted anything (forced braking at up to 358 m/s^2): a driven train without a plan
  is now planned at once on the game thread. Failure lines now count the moves left out (out of reach, tail window,
  stay at the goal).
- MT-SIPP paper (Docs/machines-12-00482.pdf, Miao et al. 2024) compared at the request of the sfmod-08 session.
  Station stop conflicts (sec. 5.3, status 1-2) are covered by the existing goal stay check (StayFits: every resource
  under the body, joints' fouling zones included, kept free of every booking in scope from arrival through dwell and
  pull-out) and the tail-clear windows of every wait; the paper's permanent GOALS/CARS lists correspond to stays booked
  open-ended here (trains not driven, failed plans standing for good), which later plans already avoid. Status 3 and
  the paper's priority adjustment (sec. 6, Fig. 11) are new: ATSipp::Run plans the turns again with a failed train
  moved before the train whose booking blocked it, when that booking counted only by planning order
  (`bBlockedByRank`), never before a train ahead on its route nor back before one that went before it this leg
  (`MovedAheadOf`); bounded by the Planning retries setting and half the update interval. The move sticks for the leg
  (LegStart just before the other train's). Logs: `is planned before ... from now on`, `planning retry limit
  reached`, and `N trains moved up after a failed plan` in the 60 s summary.
- Second standstill (c2b684a, 03:16-03:25 local): every train at a station "no way to its goal (1 states; left out:
  0 ...)". The real cause of both standstills: a train standing with its front at its track's end (trains stand so
  at stations) had no move at all, since passing a track end needed at least 0.5 m/s and speeding up from standstill
  over the last centimetre reaches about 0.08 m/s; trains waiting for it then never moved either. Passing a track
  end is now allowed at any speed reached. The squared-speed slack added for the first standstill was no cause and is
  reverted; harder braking when already too fast, crawling past a track end too close to stop at, 32 speed bands
  and planning a train without a plan at once stay (each closes a real gap).
- 6e1071b session (03:31-03:46 local): Cable RSC waited for AI Limiter RSC at a junction on track 2146916052 vs
  2146621121, a pair from AFGBuildableRailroadTrack::GetOverlappingTracks that never meets (the game counts curves a
  foundation apart as overlapping, as the user confirmed for vanilla blocks). Crossings now need the centre lines
  within JointClearance (sampled every JointFoulStep, 3D); dropped pairs are logged once. Also: the queue-behind stop
  shift of the run merging (QueueAhead) counted same-way trains merely passing the run's last track, pushed the stop
  behind where a follower already was and made plans fail every other cycle ("no way on past switch 130", many tail
  windows missed, cycles up to 78 ms with retries); removed, the same-way rule already lets followers plan to wait at a
  run's end behind the leader.
- Junction slowdowns, real cause (user's idea): ReadTrain counted every track attached at a track end a vehicle
  overhangs as occupied, including switch branches the train doesn't use; an unused branch overlapping the other line
  (a crossover) then held the other train up ("stands on track ..."), and was booked as a tail pass. A vehicle now
  occupies only the track it moves on to: the only one attached, or at a switch the one its route or another of its
  vehicles is on; the joint's fouling zone covers the rest. The clearance measurement (7b8576d) was reverted.
- 1.3.1: station stays from 3600+ logged stays were two values per train, ~15 s (no cargo to move: the stop's
  DockForDuration, default 15 s) and ~27-28 s (a freight platform transfer, mTimeToCompleteLoad/Unload plus docking).
  GetExpectedStay now uses the timetable stop's FTrainDockingRuleSet: load/unload once = transfer stay if cargo moves
  (DockTransfer from the docked cargo platforms' status: Loading/Unloading -> 1, all Complete/Idle -> 0), else the
  duration; IsDurationAndRule takes the larger; fully load/unload and unknown rules use the train's last such stay
  (mDwell) or the setting. The transfer stay is learned globally (mTransferStay, starts at the platforms' time + 2 s).
  Slopes: GetGrade(connection) = rise per cm towards it; brake and acceleration samples have g * grade (front track)
  taken out, GetPlanningDeceleration/GetStoppingDistance/GetAllowedSpeed add g * GradeAhead (300 m), the planner adds
  each run's length-weighted grade to its accel/brake (FATSippGraph::GradeOf), and the catch gets -GradeAhead (the
  game's restrictive grade weakens braking when positive). Watch for "freight platforms move cargo in" and the new
  "(expected ..; load/unload once, cargo moved)" stay lines.
- 1.3.1 session (12:21-12:45 local, 17 trains): station stays predicted within 0.5 s (27.1 s with cargo, 15.0 s
  without); no deadlocks, crossings, pass-throughs or slow updates. Bug: Reinforced Plates Train at 120 km/h lost the
  track 3 m ahead it held from its last grant when its plan failed for a cycle ("its plan has no slot there yet":
  CanEnter refused any resource without a plan entry), and forced braking stopped it at 552 m/s^2. Held resources
  are now kept without a plan slot, like locked ones (only a train standing there or holding it stops it). Open:
  1-15 brief plan failures a minute, ~90% "tail windows missed" (no wait place where the body clears the next
  booking), each gone the next cycle.
- 1.3.1 session (17:37-18:17 local, 18 trains, network being rebuilt): no deadlocks, crossings or pass-throughs.
  Heavy Oil Residue (120) [Gunpowder] stopped at 134-498 m/s^2 three times after "its plan has no slot there yet"
  (fixed in 1.3.2). Nitric Acid RSC sped up from 88 to 99 km/h under forced braking toward a stop 190 m ahead,
  forced braking engaging anew every update: it took off only Needed - the planned braking (1.5 m/s^2), but the
  train's own braking was far less (brakes building up, downhill, fluid cargo). Forced braking now measures what the
  train slowed by itself since its last tick (State.ForcedSpeed) and takes off the rest.
- 1.3.2 additions: traction is learned per speed band as force (TractionForce = acceleration sample + g * grade, times
  mass), planned per band as force / current mass; unmeasured bands use the locomotives' summed CalcTractiveEffort
  at the band's middle speed times the share seen in measured bands (0.5 until any); the planner uses the
  time-equivalent mean acceleration from 0 to cruise. Grade is the body's mean slope: (front car Z - rear car Z) /
  their distance, front = mSimulationData.FirstVehicle (first in the direction of travel); GradeAhead from the rear
  car to the route ~300 m ahead. The planner's per-move slope is half the front's rise plus half the tail's over the
  move, interpolated over track-end heights (FATSippGraph::ZOf) and the cars at the start. Log: "<train> speeds up at
  X m/s2 at A-B km/h (t, slope %)" per first sample in a band. Not handled: a docked train leaving with a locomotive
  at the other end starts its plan with the body's slope reversed for its first train length.
- 1.3.2 test (19:04-19:46 local, 19 trains): a jam from 1 min after load to the end (40 min, ~2000 plan waits a
  minute): Circuit Board Gather Train and Coated Plates RSC B each stood where the other's way led; both plans failed
  "no way to its goal" (tail windows missed) without naming a train, so they waited for "?" - invisible to the
  deadlock detector (no edge), never waiting by WaitingSeconds (needs BlockedBy), never moved up (needs BlockedBy) -
  and eleven trains queued behind them. FGap now records the booking that sets ClearHi (HiBy, bHiByRank) and ClearLo
  (LoBy); a tail window missed or a goal stay that doesn't fit notes that train, so failures name who is in the way.
- Bug report (player, telemetry-20261029-000557.csv in D:\SFMod\Friends logs, not in git): two 32 m push-pull trains, 360 m
  between two dead-end stations on single track with a passing place. One docked and turned back while the other,
  already heading there, had entered the single track: the docked train's goal booking only covered its stay plus a
  forward pull-out, so the approaching train planned in right after it. Both then waited for each other ("Train"
  blocked by "Train"). Fix: a self-driving train with a timetable also books its next leg (ATSipp MakeNextLeg): from
  where its body stands at the goal, Dijkstra by length to the next stop's station track, on through the goal's
  track end or (bCanReverse: a locomotive faces the other way from the master) backwards from the rear car, whichever
  is shorter; planned as a standing EATSippKind::Predicted train from leg 1's arrival (FPlanner with an explicit train
  and time base), departing at arrival + GoalDwell, staying NextDwell at the next station. Booked as Predicted, so
  every train keeps clear of it; planned around Predicted bookings first, else around only locked/held/committed/
  Standing ones (bFixedOnly) so the others yield at their next turn. Not booked when the next stay is unknown. Log:
  "<train>: next leg after its stay at <station> booked, leaving backwards/forwards" on change, and "N next legs
  booked" in the planner summary.
- 1.3.2 sessions (21:02-21:57 local, 19 trains) and the friend's second report (3 push-pull trains, two dead-end
  stations Aix-en-Provence / Hamburg, 3-track passing section, video + telemetry in D:\SFMod\Friends logs): the next
  leg flipped booked / "not booked: no way fits" every cycle (~21000 log lines an hour), e.g. Cable RSC and Crystal
  Oscilator RSC both bound for Blue Crater Rubber A: one cycle Crystal's leg was booked and Cable's plan waited 236 s,
  the next neither was. Cause: next legs were booked as Predicted (everyone yields) while planned around nobody's
  current leg, and skipped when they failed. Grant flips 30/min (1.3.1) -> 224-248/min; time under 12 km/h 7-8% ->
  25-27% of moving time; cycles 70 -> 140-240 ms. The friend's trains still deadlocked at Hamburg. Reverted in 1.3.3
  (a4729bf's source), replaced by the dead-end rule: at a goal whose way on ends without a switch (IsDeadEnd: no
  attachment, a one-way signal, or single attachments to the end), StayFits checks and Build books every resource
  from the last switch on the way (LastSwitchAt, FOpenResource::StartAt) until the train has backed out past it:
  arrival + GoalDwell + RestTime(goal - resource start). Part of the train's own plan, so ranks and priority
  adjustment decide who goes first. Also removed the pre-planner pacing (ClearSeconds: drive at the speed that
  reaches the wait point as the plan moves off): with long plan waits it made trains crawl at 1-8 km/h for minutes
  before the switch where they wait, and each flip applied it anew.
- Audit after the user's rule (revert fixes that weren't the fix), 1.3.3: 1df9278 was the fix for "every train stood
  still" but the cause was 5cd9dfc; of its parts, passes planned down to a crawl within ForcedReach (Slowest = 0 there,
  floored at MinPassSpeed) are reverted (Slowest = what braking reaches, floored at MinPassSpeed); harder braking for
  moves ending within ForcedReach stays (a train inside its service stopping distance has to brake harder; without
  it plans failed outright), as do the 32 speed bands, the rejection counts and planning a new train at once (made
  first-cycle-only in e7d5303). Kept as real fixes: 5cd9dfc, 5682b21, 6264747, e067e4a. c873748 (keep held track
  without a plan slot) and d1673d5 (forced braking counts achieved deceleration) to be re-checked on a 1.3.3 log:
  the 1.3.2 logs had 2560 forced brakes an hour (80 over 20 m/s^2) and 1635 "no slot" lines, mostly from the
  next-leg grant flips.
