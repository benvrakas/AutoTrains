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

The Detailed log setting (before 1.3.10 `/at log on`) writes `%LOCALAPPDATA%\FactoryGame\Saved\AutoTrains\telemetry-<time>.csv`: one `# consist` line per
train (name, then its object name), then a row per train every 0.2 s (`train` with commas made spaces, `train_id`
the AFGTrain object name, unique where names repeat):

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
- 1.3.3 first push-pull test (user's own two 32 m push-pull trains, two dead-end stations, a passing section of one
  long straight and an S-curve bypass): crawling gone (7.7% of moving time under 12 km/h, cycles 45-95 ms), but the
  pair deadlocked: Test 1's plan failed ("no way to its goal") and it stood for good at its furthest stop, front
  locomotive 18 m from switch 254's point, inside that switch's zone (20 m minimum then), the switch Test 2 needed to
  reach the bypass; 0 reroutes for either (IsSearchDue skipped trains whose authority reached the goal, true on a
  360 m line). Changes: fouling zones from a Track clearance setting (default 3 m, was the fixed 5.5 m, with a 20 m
  minimum along every track); the 25 m minimum stop margin before a switch removed (zone + stop distance); crossings
  use the same clearance; bAuthorityToGoal no longer blocks route searches (a pre-planner rule; the planner freezes
  the route up to Commit anyway); a failed plan stands at the furthest stop with no switch joint under the body
  (FurthestClear) before the furthest stop of any kind. Not done: a docked train's departure still uses the game's
  shortest path (the arriving train is expected to reroute around it).
- 1.3.3 second push-pull test: still deadlocked, with the leaving train undocked and standing in its station (Moving),
  0 reroutes. Cause: GetRouteCommit counted a route step as committed unless the train could stop the far end's stop
  margin before its start, even standing; the leaving train stood 8 m before the switch, so the branch after it (and
  any switch's branch within zone + stop distance) was fixed and the route search couldn't change it. Now a standing
  train is committed only to what it holds, and a moving one to what it can't stop short of with the stop margin of
  the switch before the step. A docked-departure route search tried before this (2964257) never ran and is
  reverted.
- 1.3.4 crossing zones (user's report: on a junction a train going straight waited before the whole junction, not
  just before the curve crossing its track; question: can trains use both rings of a double roundabout at once). Before,
  crossing tracks conflicted as whole pieces (mConflicts) and stops were only at track ends. Now each crossing is a zone
  resource (tracks + joints + crossing): on each track, the stretch whose centre line comes within the clearance of the
  other's (sampled every 2 m, padded a step), FATZone / FATSippZone, mZonesOf / Graph.ZonesOf. Tracks conflict only with
  themselves. Planner: a move books the zones the front reaches by its end (the rest on the next move); Expand offers a
  stop StopDistance short of each zone on the run (not on the goal's track); stopped states are told apart by
  position; Build adds zone passes; WaitShort gives where a wait stands short of its track end. Grants: zones in turn
  per claimed track; a zone it can't have ends the grant AuthorityCut before the track end (StopMargin takes it);
  ClaimedCrossings = zones on held tracks before the cut; OccupiedCrossings from vehicle spans; follow limits stop
  before a zone a leader holds; checks and the wait dump name "crossing N". Untested in game.
- 1.3.4 platformless stations (user: a push-pull train's dock was nearly instant; game log of 1.3.3, 2.5 h): the two
  push-pull trains never logged "docked at" or "left", so their docks ended within one update (0.25 s) and TrackDocking
  never saw them; GetExpectedStay still gave the unknown-transfer stay (15-29 s) as GoalDwell, and both trains failed
  "no way to its goal" on nearly every leg, then rerouted 3 -> 7 tracks. Now HasCargoPlatforms walks the station's
  platform chain on both sides (up to the next station, 64 platforms) for an AFGBuildableTrainPlatformCargo; without
  one the stay is the train's last seen stay there (mDwell), else 0. Same log, not addressed yet: two 3-train
  deadlocks ghosted through after 60 s (05:56, 06:09). In the second, Heavy Oil [Coke Steel] stood 7 m behind Coated
  Plates RSC Train but still held its grant to its goal (9 claims, locked "enters in 0 s, clear in 20 s"), so Train
  waited for it on track 2145430117, while Coated Plates waited 80 s for Train at switch 180; the numbers stayed the
  same for minutes. Real cause (telemetry): Coated Plates RSC Train (700 t, 2000 kN traction) was stalled on a climb on
  track 2147329636 from about 05:54 to the end of the session, at full throttle and rolling back and forth to where the
  slope's pull (up to 2223 kN) equals its traction (front about 51 m in). It had come up at about 70 km/h; its authority
  then dropped from 188 m to 34 m (blocked by Steel Pipe Xtra Steel), the throttle was cut on the grade, and when the
  track came back it was too slow to climb on. The planner kept planning it as able to drive ("may drive 351 m, target
  133 km/h"), so trains behind and around it waited on its plan for up to 2534 s, and the deadlock detector (wait
  cycles) never saw it; ghosting through at 05:57 and 06:12 left it on the hill.
- 1.3.4 climbs (fix for the stall above). Planner: FATSippTrain.Climb is the train's full low-speed traction
  (FullTraction: its locomotives' curve at band 0, without the hill help) over its mass; StartClimb is that times the
  hill help limit when it's on, used for moves from where the train stands now. Expand used to floor the acceleration
  on a slope at MinSlopeAccel, so a train could always speed up; now a move is left out (RejectedClimb, in the failure
  log) when, at full traction against the mean body slope over the move, the train would stall before its end, and a
  pass is capped at the speed that leaves it. So no wait is planned where the train couldn't move off again. Grants: a
  route track whose end-to-end slope pulls back with over 90% of FullTraction is a hard climb; the grant runs through a
  hard climb rather than ending on it, and a train held up on one it hasn't reached yet (and can still stop before)
  waits at its foot ("waits before the climb at ..."). Hill help (UpdateHillHelp): a managed self-driving train under
  15 km/h that wants to go faster, with the pull under its body over 90% of FullTraction, gets traction x1.3 the pull
  (at most the setting, default 4), raised but never lowered, until the pull is under 80% of FullTraction; traction
  isn't learned while it lasts. Logs "stalls on a N % climb" / "is over the climb". Untested in game.
- 1.3.5 (1.3.4 log, user: "Nitric Acid RSC is causing a holdup for no reason"): Nitric Acid RSC waited 8 min 11 m
  before crossing 93 because Crystal Oscilator RSC, two trains behind it on the same line (behind Rubber SNK [conc]),
  held crossing 93 (granted with 18 claims, 337 m). IsBehind looked a crossing key up by Key.Track (null), so the
  "holder behind me on the same line" exception never applied to crossings; FillSippTrain's Behind sets left crossings
  out too. Now routes keep CrossingIndexOf (IndexRoute), IsBehind uses it, and Behind adds the crossings on the shared
  stretch. The deadlock detector missed this one (Crystal Oscilator's blocker was a train 337 m ahead). Not done: 29
  CROSSING check warnings in 11 min, mostly pairs following each other at speed (Copper Sheet / Circuit Board Gather
  over crossings 29-41), likely followers the check doesn't exclude.
- 1.3.6 (1.3.5 log, user: "still getting deadlocks that aren't registering as deadlocks"): from 17:22 (log time) to
  the end, 13 trains waited 25-40 min and no DEADLOCK was logged. Root: Rubber SNK [conc] overran crossing 22 while
  braking from 100 km/h without having been granted it ("-60 m to stop"), and stood there for good: 10.5 m of
  authority, full brakes, `waits for ? before crossing 22, 0 m ahead: its plan has no slot there yet` (the 0 m is
  clamped; the zone was behind it). GrantFromPlan checked every crossing zone on the front's track, also zones the
  body had passed; the plan leaves those out (ClearAt <= 0), so a zone it never held had no slot and was refused
  forever. Train stayed behind it, Reinforced Plates waited for Train (on crossing 22), and the rest queued behind.
  Not seen as a deadlock: Rubber had no BlockedBy, so its WaitingSeconds stayed 0 and no chain closed.
  Fix: GrantFromPlan and RefreshClaimedJoints skip zones the whole body has passed (Begin + To + Length <= 0, the
  plan's own rule), so a train also stops holding a crossing it has cleared while its front is still on that track
  (some of the CROSSING "was granted" warnings may be this). WaitingSeconds now counts any held-up standstill, and
  LogDeadlocks reports `STUCK:` (log + faults.log) for a train waiting longer than the deadlock threshold with no
  train to wait for, naming the trains whose wait chain ends at it; they all get the long-wait icon and the root
  looks for another way at once.
- 1.3.6, second round (8a1d56e session, 34 min): no Rubber-style stall (no standstill with an unnamed blocker for 20 s
  in telemetry), but 39 deadlocks (rate as before). 29 of their members: `X: waits for Y before crossing N, ~5 m
  ahead: holds crossing N` with `Y: stays behind X` (56 in the 1.3.5 hour too). IsBehind required the key past
  Self's front track (`*OtherKeyAt <= SelfAt` -> false); a crossing zone can be on that track ahead of the front, so
  the follower got it through the follow exemption and the leader then waited for its own follower. Now crossings on
  Self's front track count (FirstAt = SelfAt), FillSippTrain's Behind sets add the zones of the front track, and the
  crossing double-hold check skips same-line pairs like the track check. IsBehind also returns false when Other
  stands on the key: route indices don't see what the tail stands on, so a route coming back over a crossing let ACU A
  be granted crossing 72 while Steel Pipe Xtra Steel stood on it.
  STUCK fired 17 times on momentary "waits for ?" lines (a failed plan between named blockers) because it used
  WaitingSeconds; it now needs UnnamedWaitSeconds (held up with no named train, without a break) past the threshold.
  CHECK FAILED right after "conflict model rebuilt" (crossing count changed while building) is one update of claims
  under the old crossing numbers; it clears on the next update.
- CROSSING diagnostics (after 706e968): `planner: CROSSING ENTRY:` logs a driven, non-ghosting train coming to stand
  on a crossing zone it doesn't hold (speed, authority, full/forced brake, seconds since the conflict model rebuild,
  last wait reason). The CROSSING warning and crossing CHECK FAILED lines now carry, per train: stands on/holds, each
  route step with the zone (metres from its front, track held or not), its vehicles over the zone, its plan's entry
  time, its claims and grant cut; then route order both ways, IsBehind both ways, FollowedTrains both ways, ranks, and
  the zone's bookings. Read them as: "was granted" with the granter's zone ahead and no IsBehind/follow = a grant
  that ignored the stander; a CROSSING ENTRY at speed with forced brake = an overrun; right after a rebuild = renumbering.
- CROSSING ENTRY results (bb0db24 session, 47 min): no deadlocks, STUCK, CROSSING warnings or CHECK FAILED; longest
  wait in telemetry 37 s. 660 CROSSING ENTRY lines: 644 from the body behind the MU master (no route step, no plan
  slot; the route starts at the master, so only OccupiedCrossings covers those tracks) at full speed, repeating every
  ~0.5 s per crossing, one per 16 m car. Cause: ReadTrain tested zones only on each vehicle's own track (its centre's),
  and the zones were within a car half-length of a track end (0-2 m, 32-34 m on a 34 m track), so cover dropped each
  time a car centre crossed the end: other trains could be granted the crossing in those gaps. Now a vehicle reaching
  past a track end also stands on the next track's zones within its overhang (same next-track rule as Occupied, no
  OverhangTolerance). The other 16 were the update right after a conflict model rebuild (crossings renumbered, 0
  claims); LogCrossingEntries is skipped on that update.
- 4ba0e65 session (2.4 h): 1 CROSSING ENTRY (a rebuild outside UpdateTraffic's own), 0 deadlocks, 1 STUCK (Rubber, 23 s
  at a departure after a 2-state search failure with no cause recorded). CHECK FAILED x2 at 22:37:03: Circuit Board
  Gather held crossing 22 (394 m ahead, plan enters in 48 s, empty BlockedReason) while Train 2 held it 77 m ahead
  (locked, 3 s). GrantFromPlan's zone loop skips zones past PlanWaitAt unchecked, relying on the plan-wait cut that
  follows; but the `End >= Need` break came first, so on the step holding the wait the whole track was claimed and
  RefreshClaimedJoints (no AuthorityCut) added those zones. Same cause behind the worst "over its braking curve" lines
  (AI Limiter 113 km/h allowed 0, MF Resource 99 allowed 0): no cut, so the train ran at speed up to the wait, where
  PlanWaitAt = 0 and the "where it stands" branch left it only the tracks it stood on. Now the plan-wait cut is checked
  before the Need break.
- 1.3.6 (43ccc1b) session, 1 h: worse, not better. Severe braking-curve overruns 7/h (was ~3/h); e.g. Reinforced Plates
  97 km/h with 96 m to its plan's wait, Coated Plates 127 km/h with 63 m. The plan-wait cut now applied on the step
  where the Need break used to end the grant, so waits a plan (made from a snapshot ~1 cycle old, possibly with
  forced-braking reach) put within stopping distance were enforced; before, the train drove on and the next plan
  moved the wait. Also Train 2 and Steel Pipe Xtra Steel planned to wait for each other (plan waits growing ~130 s per
  s, DEADLOCK + STUCK at 23:39:29) until Steel Pipe rerouted 11 s later. 43ccc1b reverted; instead, when the Need break
  ends the grant on a step, the zones past PlanWaitAt are checked with CanEnter there (GrantZones(true)); a refusal
  cuts the grant at that zone. "where it stands" hard stops at speed (Train 2 103 km/h, allowed 0) predate 43ccc1b:
  PlanWaitAt clamps to 0 when the train has already passed its plan's wait point.
- a1647c4 session (46 min): 0 deadlocks/STUCK/CHECK FAILED/CROSSING warnings, longest wait 37-40 s, severe overruns
  3.9/h (pre-1.3.6 level), "where it stands" 12/h, no runaway plan waits. The 3 CROSSING ENTRY lines followed the
  rebuild SetEnabled does when traffic control comes on (not seen by UpdateTraffic's own check), so the entry check is
  now skipped whenever mConflictsBuiltAt changed since the last update (mConflictsSeenAt).
- 1.3.8 (stall with no warning): the 1.3.7 session left running 16 h ended in a 12-train queue, screenshot of a junction
  on a hump; the user: looks gravity related. Head of the queue: AI Limiter RSC (700 t, 2000 kN), never in a WAIT dump
  or STUCK report, only as a booking line: "0 km/h, waited 0 s, 10 claims, may drive 330 m, target 136 km/h" from
  08:44 UTC for 15 h. It came up the climb past switch 66 at 32 km/h, slowed, and stood with its front 30 m before
  switch 101 and its tail on switch 66, 245 m short of what it was waiting for, so no authority stopped it. No
  BlockedReason, so WaitingSeconds stayed 0 and neither the deadlock nor the stuck check saw it; everything behind it
  waited on its bookings ("waits for ?": Rubber SNK [conc]'s failed plan had 3 states, its block noted on moving states
  that aren't the one it stands at). Hill help never logged: it needs the body-chord pull over 90% of traction (26.2 %
  for this train) and the stand was gentler; after a reload the same train moved off at once, reached 14 km/h at
  26.5 % and hill help took it over a 34.4 % chord. NOT known: why it didn't move in the old session. A driver at full
  throttle should have moved it at that pull, and there was no telemetry; the driver (throttle up to 1.00 in telemetry)
  may have idled with a stale target, or the real gradient may exceed the chord (cars aren't evenly loaded; the 1.3.4
  stall also sat at full throttle where gradient equals traction). Fix: UpdateHillHelp also counts a train standing
  (under 2 km/h) that wants 3.6 km/h more for HillStallSeconds as stalled on any slope and raises the help x1.25 each
  time; help ends only once it is also moving or no longer wants to. At HillHelpMax it reports STALLED (log and
  faults.log) with DescribeDrive (throttle, brakes, forces, power, error) and how long ago ApplyAuthority last ran
  (DrivenAt), which tells the two theories apart. Untested in game.
- 1.3.8 (wrong branch): the user saw a train from a single track head onto a double track the wrong way, screenshots of
  a junction with red signals. Session 2026-10-31 19:09 local (log is UTC). MF Resource Train and AI Limiter RSC met
  nose to nose at the joint of tracks 2145780312 and 2147407495 at 02:01:53, both on failed plans ("no way on past ...
  for" each other, each driving up to the end of what it held); deadlock ghosted at 02:03:12. 02:03:19 MF, ghosting at
  90 km/h: `LogRailroad: UpdatePathSegment, failed to find our location along the path, aborting`, then "forced braking
  at 309 m/s2 (-10 m to stop)". MF is then in no WAIT dump, STUCK report or blocker line until 03:29:02: unmanaged, no
  path (bManaged needs one). AI Limiter stood 28 m short of switch 126/128 for 5000 s ("no way to its goal", waits for
  ?), Reinforced Plates beside it; both moved within seconds of MF at 03:29, a few seconds after the user's message:
  probably the user moving MF by hand (not confirmed). That warning always sits next to a train past its stop point:
  7 this session, 1129 in the 15 h gridlock (Adaptive Control Unit GTHR Train B, "-24 m to stop", 2-4 a minute from
  08:48, i.e. after AI Limiter stood at 08:44, so it did not cause that stall).
  RE (AFGTrain::TickSelfDriving_FollowPath, 0xa0b616): each tick, if Atc.CurrentConnection IsFacingSwitch, it takes
  PathPoints[CurrentPathSegment + 1]'s track and, only if CurrentConnection->IsSwitchClear(this), sets the position. On
  not clear it does nothing and the train drives on. IsSwitchClear collects every track of the switch on both sides and
  fails if IsConnectionOccupied(connection, 1200, ignored): another train's vehicle within 12 m of the joint, whichever
  way it moves. SetSwitchPosition just forwards to the switch control, no check. The planner's own limit is the fouling
  zone (track clearance, 3 m by default), and ghosting ignores the switch, so a train can legally reach a switch the
  game won't set: MF reached switch 140 with AI Limiter's body on it, so it stayed set the way AI Limiter came and MF
  ran onto that lane against the signals. Position only matters to a vehicle leaving through the facing connection: a
  branch connection has one attached connection, so a train coming in from a branch ignores it, and vehicles past the
  joint ignore it. Fix: ThrowSwitches sets each facing switch within two updates of driving plus 20 m ahead of the
  train's front, unless HasToLeaveThrough says another train has a vehicle leaving through it or gets there sooner. Such
  a train is ahead on the same line, so following keeps this one behind it and no hold is needed; ghosting past one
  would log WRONG SWITCH. Game logic is left running (same route, same result). WatchLostPath reports LOST for a
  self-driving train without a path for DeadlockWaitSeconds, since these trains were invisible. NOT fixed: how the two
  trains got nose to nose (failed plans each drive up to the end of what they hold, from both ends of a single-track
  section), and a train stranded the wrong way still needs help by hand. The user's temporary signal (placed wrong way
  to split a spline, deleted seconds later, far from here) is unlikely: the track count stayed 4929 from 01:21 to 02:29
  UTC, so no split in the hour before; a wrong-way signal only counts if a 30 s rebuild lands while it stands.
  Untested in game.
- 1.3.9 (SmartTrains session, 2026-11-05 13:07 local, log 19:07+ UTC, ~40 trains, 1.3.8 installed from SMR): the
  SmartTrains session forwarded "trains drive through each other at junctions" with 21 "can't stop short of ...
  booked at the same time" warnings. Those are planning conflicts only: the grant still gives a resource to one train
  at a time, the other brakes hard (forced braking). Real contacts: 19:19:12 (27 sweeps, MF train on track 2146360615,
  Copper Ingot train on 2146357810, crossing 83) and three deadlock pass-throughs of the water shuttles (19:21, 19:29,
  19:32). 19:19: MF stood 11 m before crossing 83 waiting for the Copper Ingot train (booked first, locked, clear in
  7 s), then got target 100 km/h at 19:19:05 while that train had stopped behind a queue with its tail on the
  crossing. CanEnter's "nobody else holds it" loop lets a followed train (GetRouteOrder > 0: its master's track is on
  Self's route, same exit) through without asking which side it occupies the resource from; at a merge its front is
  on Self's route while its rear cars are on the other branch in the joint or crossing zone, and GetFollowLimit's gap
  only counts vehicles on Self's path. Fix: HasVehicleOffRoute; a followed train with a vehicle in the zone on a track
  not on Self's route or under it blocks. Not proven that the 19:19 contact was this (no geometry in the log), but
  it is the only grant path that fits.
  Water shuttles 19:20: train A left Water SRC B for SNK B while B was docked at SNK B about to leave for SRC B, the
  same 10 tracks the other way: no way past each other, so each plan waited for the other's booking ("moves off in
  22719 s" is that, not a stay estimate), deadlock, pass-through. A dispatch problem; SmartTrains can now ask
  ExternalEstimateTravelSeconds / ExternalGetStationQueue / ExternalGetTroubledTrains. Separately, fully load/unload
  stops ignored DockForDuration in GetExpectedStay (B: expected 26.9 s, stayed 120.7 s at its "or 120 s" stop with no
  cargo to move); now max with "and", min with "or".
  User (same day): even before Auto Trains, trains queueing for a station would stand across the oncoming main line
  and deadlock. SIPP waits are allowed wherever the windows fit, tail over a junction included, and trains planned
  later queue behind that. Fix: Move rejects a wait longer than JunctionWaitSeconds (setting, 5 s) at a state whose
  body is on any joint or crossing resource (IsOnJunction: open resources with ClearAt past the stop), except at the
  start; counted as "waits across a junction" in failure lines. Risk: tight back-to-back switches may leave no legal
  waiting place, giving more failed plans; the setting can be raised.
  Hill help: 55 "stands though told to go" were mostly trains at their stop point with a creep target (target 4 km/h,
  throttle 0.00, ~1 m of room). bWants now needs State.Room (room to the stop point as last driven) >= 5 m.
  Untested in game.
- 1.3.9 (head-on pair, SmartTrains log 13:55 local, 20:16 UTC): the two push-pull water trains left opposite ends
  of their shared single track 4 s apart and stood nose to nose, both "waits for ? ... no way to its goal", STUCK at
  20:16:41/45, never DEADLOCK (the cycle needs BlockedBy and the failed plans name nobody: the block is noted on moving
  states the stand state doesn't see). Fix: after planning, a held-up train with no BlockedBy gets the first train with
  a vehicle on its route ahead of its body (or on a switch there) as BlockedBy (FindTrainInWay), so the pair is a
  cycle and gets passed through. Side effect: such waits are no longer "unnamed", so they leave STUCK; a train held by
  a parked train is named but no deadlock (the chain stops at an unmanaged train). Also ExternalCheckLeg for
  SmartTrains to hold a train in its station: a probe turn (Kind 3, bProbe, ranked last, never moved up) planned after
  every self-driving train, with all their bookings in scope and invisible to them; its bookings are replaced by the
  train's own standing/predicted turn next cycle, which comes before the self-driving trains. ExternalGetPlannedArrival
  for docked trains returned the old leg's PlanArrival (0): now PredictedArrival from the predicted-leg result.
- 1.3.10 (Detailed log): a 7-minute session with 38 trains wrote 1205 Auto Trains lines, about 390 "speeds up" and
  390 wait reasons. Those, brake samples, station stays and next legs, timetable changes, leg checks, forced braking,
  switches set for the game, plans finding no way, planning-order moves and non-crossing overlaps are Verbose now;
  the Detailed log setting sets LogAutoTrains to Verbose and starts the telemetry CSV (replacing /at log). Faults,
  hill help, reroutes, full brakes and the 60 s planner summary (which counts failed plans) stay at Log. Non-crossing
  overlaps are only logged at the first rebuild after the setting is on if that pair wasn't seen before.
- 1.4.1 (IB server log 2026-11-06 04:07-04:10 UTC, via the SmartTrains session): Amuminum Ingot RSC docked at
  Encased Unranium Cell RSC 1 for 220 s under "fully unload or 120 s" (the stop rule, into a nearly full platform: so the "or" time
  plus docking runs far past the min(stay, 120 s) estimate; SmartTrains' hold only withholds the next trip).
  Train (job 60, goal that station) stood 38 m before switch 578 with its body on crossing 336: its plans said 3 s, as
  an overdue docked train's departure was Now + OverdueDepartSeconds (1 s) every cycle, under JunctionWaitSeconds.
  Amuminum's way out crossed 336: DEADLOCK 04:09:15, passed through 04:10:36. Same cause: Motor RSC 1 waited 62-92 s
  behind Cable RSC docked 227 s at Grass Fields Crater Conc (Rubber SNK), on its route, and never rerouted (a 1-5 s
  wait never gains RerouteMinGain). Fix: overdue departure = Now + clamp(overdue, 1 s, 600 s). Open: a docked train
  with no predicted leg (SmartTrains writes the next job only as it leaves) books no way out, so a queue can still
  stand in it when the stay ends sooner than the overdue estimate.
- 1.4.1 (same server log, 03:11-04:57 UTC): 3 STALLED and ~40 "stands though told to go" lines, all with throttle 0
  and air or dynamic brake on, mostly on flat track (target 10-16 km/h, 6-16 target points). The 04:29 Train was a member of
  the 3-train deadlock reported a second before ("waits for Motor RSC 1 before the switch ..., 42 m ahead"), so Room
  (authority) passed HillStallRoom while the plan held it. Fix: bWants is false for a braking train (throttle <= 0, any
  brake) with a BlockedReason; a braking train without one gets no stall steps (STALLED still reported once).
  Baseline unchanged: ~1.8 "can't stop short" per minute (190 in 105 min, 39 trains), 7 deadlocks passed through.
- 1.4.1 detailed-log session (server 05:05-05:16 UTC, 45 trains): 0 deadlocks, STUCK, STALLED, LOST or crossing
  entries; 7 "can't stop short". 13 hill help lines: held trains creeping up to their stop in a queue (2 km/h, throttle
  0.04-0.07, flat) or braking down a 7-18 % slope, all with a named blocker in telemetry. bWants now also excludes a held
  train unless the slope pulls over HillStallShare of its traction. Planning cycles 226-497 ms on the worker, 49-99
  updates a minute waited for one. Telemetry rows are keyed by train name, and SmartTrains names repeat (several
  "ST: Nitrogen Gas: Jungle Spires Nitrogen SRC (2100) -> RockPow- Nitrogen SNK"), so rows of same-named trains mix.
- 1.4.2 (server log 06:38-08:13 UTC, AT 1.4.1 / ST 0.6.3): 13 deadlocks, 6 passed through, all at track 2147310803 and
  crossings 167/168 on the way into Aluminum Ingot RSC (13 trips through it in 15 min, at capacity). At 08:09:13 ranks
  29-41 stood nose to tail through switch 52, each "stays behind" the next 10 m ahead. The follow limit stops a train
  behind its leader's nearest vehicle wherever that is; the SIPP junction-wait rule doesn't cover it. Fix: in
  GetFollowLimit, if stopping behind the leader (with its braking when it moves away) would leave the body on a
  switch's fouling zone (GetJointFoulDistance either side of the joint) or a crossing zone ahead of the front, the limit
  is that zone's start, if the train can still stop StopDistance short of it. SmartTrains 0.6.4 adds QueueLimitSeconds
  (rejects trips queueing over 60 s at any stop) and keeps repeats out of queued stations.
- 1.4.3 (server log 16:04-18:07 UTC, AT 1.4.2 / ST 0.6.4, telemetry with train_id): the two push-pull water shuttles
  (both named "Train": BP_Train_C_2146978540 Water SRC A <-> SNK A, BP_Train_C_2147084676 Water SRC B <-> SNK B, 10
  tracks / 245 m legs sharing switch 514 and track 2147100206) stood each other off from 17:50 for 17+ min with no
  DEADLOCK line. One waited "where it stands" for the other (its plans alternated between "no way on past switch 514"
  and a locked booking on 514); the other, 21 m before 514, flapped every update between 2 claims (blocked: "booked
  first on switch 514 (locked)") and 10 claims (authority to its goal, empty BlockedReason), never moving off
  (0-1 km/h, 3 m in 13 min). The empty reason on alternate updates reset WaitingSeconds (waiting_s 0.0/0.3), so it
  never reached DeadlockWaitSeconds and LogDeadlocks never ran the chain or ghosted it through. Fix: a wait ends only
  after WaitEndFreeSeconds (3 s) let go without a break, or moving over PlannerWaitSpeed. Open: why the grant flaps
  (the shuttle that waits where it stands books 514 locked; the other is granted 514 on the updates its plan failed).
  The 16:05-16:08 Copper Sheet RSC 2 / R-Plates deadlock at the climb on track 2145780312 passed through after 60 s.
- 1.4.4 (same server logs): (1) The 16:08 deadlock at the climb (track 2145780312, 38 % with hill help) and the 16:00
  one by Rotor RSC 1 (tracks 2147216935/2144004608) formed the same way. The R-Plates train, docked at the top with its
  leg down booked, kept the Copper train at switch 147 until 16:04:48, when its plan failed on a third train (Iron
  Plate). A failed plan books nothing past where its train stands, so the next cycle the Copper train was granted the
  single stretch and stopped at the foot of the climb (climb wait), face to face. Also 11:01 and 14:17 in faults.log.
  Fix: a failed Moving plan adds bReserve bookings (Enter now, Clear never) on the route's tracks past its stand point
  up to ReserveAheadDistance (1 km), unless a train holds or is locked on one of them the other way. GapAt ignores
  them for the same exit; CanEnter ignores them for the same exit and for trains ranked before the owner (no mutual
  block between two failed trains); HoldsUp and FindCommittedConflict ignore them. (2) Switch 514: both water trains
  stood about 21 m short of it, inside its fouling zone (up to 42 m here: branches under SwitchClearanceMetres apart),
  so each one's joint pass had EnterAt 0 ("entering now"), was Locked, and was booked from now. Fix: standing
  (CollectPasses: Speed < StandingSpeed; SIPP: bStanding) with the front short of the switch point, the joint is
  entered at the switch point; accepted cost: a train through the switch may overlap the standing one's nose.
- 1.4.5 (server run 2026-11-06 21:30-23:34 UTC, AT 1.4.4 / ST 0.6.6): from 23:08:17 the Iron Ingot SRC C train
  (BP_Train_C_2147262886) stood 1 m short of switch 188's point (front on track 2145993630, 2.35 m), and a Coal
  train (BP_Train_C_2147143945) on the other branch 22 m short of it, for 25+ min with 10 trains queued behind. The
  Iron Ingot train's grant flapped every second (about 450 m / 25 claims, then 20-100 m); the last cut came at 32 km/h
  and forced braking stopped it inside the fouling zone. Its vehicle counted as on 188 (OccupiedJoints), so
  CollectPasses kept the joint pass at EnterAt <= 0 and Locked; but SIPP's Output still applied the 1.4.4 standing
  rule (bStanding, front short of the switch point) and put the joint at the switch point, which its failed plan
  never reached, so it booked nothing on 188. The Coal train's plan then took 188 and track 2146765859 first
  (committed), while CanEnter's holder check ("stands on switch 188") kept the Coal train back: the Iron Ingot train
  waited for the Coal train's booking, the Coal train for the Iron Ingot train standing there. The Iron Ingot train
  waited for "?", so no deadlock was found (STUCK warnings only). Fix: the standing rule applies only when the joint
  isn't in Train.Locked. Open: why the grant flapped; a lock on the joint doesn't lock the track after it.
  The water trains cycled normally (switch 514), and there were no head-on deadlocks or LogDeadlocks.
- 1.4.6 (server run 2026-11-07 00:52-01:47 UTC, AT 1.4.5 / ST 0.6.8, Detailed log): the switch 188 standoff was there
  from load to the end (Iron Ingot SRC C train "waits for ? before track 2146765859, 1 m ahead", 9-12 behind it).
  Rank 0 or not, the train had no booking on switch 188: the whole Coal queue booked 188 and 2146765859 from 3 s on.
  Cause: Build makes passes only over Out.Route = RouteTo(Final), and a failed plan's route ends at the step it
  stands on, so the joint after it (in Train.Locked: its vehicle counts as on it) never got a pass or a booking. The
  1.4.5 change (standing rule only when the joint isn't locked) never ran here; it stays, since it keeps a locked
  joint entered now once the plan's route reaches it. The same gap made 1.4.4's reservations a no-op: they took
  passes with Enter never, and a failed plan has none past its stand point (0 "reserved the other way" in 55 min of
  Detailed log, 28 trains). Fix: past a failed plan's route, Build adds the route steps whose joint or track is in
  Train.Locked (contiguous from the stand point) and makes passes there only for locked resources, entered now and
  clear never. The reservations (bReserve, ReserveAheadDistance) are removed. Also this run: 3 DEADLOCK cycles by
  Coated Plates RSC SNK (tracks 2147238907 / 2146785416), passed through.
- 1.4.7 (server run 2026-11-07 from 07:09 UTC, AT 1.4.6 / ST 0.6.8, Detailed log off until 07:14:55): the two water
  push-pull shuttles (BP_Train_C_2146978540 SNK B <-> SRC B, BP_Train_C_2147084676 SNK A <-> SRC A) share only track
  2147100206 and the merge switches at its ends; each has its own platforms. Both stood side by side at the SRC end,
  20 m from the merge (switch 531), after load. 07:13:12 DEADLOCK: rank 14 waited at switch 531 for rank 15's booking
  there (locked, enters in 10 s); rank 15 waited at track 2147100206 for rank 14's earlier planned booking. Rank 15's
  531 booking was locked though it stood 20 m short (SIPP enter time 10 s, so its front wasn't in the zone): the only
  rule left is CollectPasses' "held and within LockedWithin" (its booking on 2147100206 was held). A standing train
  can wait anywhere, so that rule now applies to moving trains only. 07:14:14 the deadlock was passed through with
  both trains ghosting at once: both ran onto 2147100206 together, the far merge switch was set for 2146978540
  (WRONG SWITCH 07:14:26), 2147084676 followed onto SNK B's branch, the game gave it SDLE_StationUnreachable (07:14:48
  LOST), it went unmanaged and 2146978540 waited behind it for good (not a cycle, so never passed through again).
  GhostThrough now ghosts one train per deadlock: the first whose blocker neither stands on nor holds the place it
  waits at (IsWayTaken), else the first. BlockedAt* are reset each grant and set for plan waits, so they're current.
  A new DEADLOCK now also logs each member's WAIT dump and whether its blocker stands on or holds the place.
- 1.4.8 (server run 2026-11-08 from 22:06 UTC, AT 1.4.7 / ST 0.7.5, Detailed log): "ST: Sulfur: Crater Lake Sulfur
  SRC (1200) -> RockPow- Sulfur SNK" stalled at 14 km/h on a 33.4 % climb (22:34:38, hill help x1.49), then stood 1 m
  short of track 2147217081 with "its plan has no slot there yet" (STUCK 22:35:07, a Coal train queued behind). The
  SIPP search uses Train.StartClimb (traction x HillHelpMax) only for the first move from standing (Step < 0, Pos <
  MinMove); every later move on the climb used Train.Climb (own traction), which can't climb 33 %, so every move up
  was a RejectedClimb and the plan failed. Fix: Out.Climb = StartClimb while HillBoost > 1 or the slope under the
  body pulls more than HillStartShare of its own traction (help lasts until over the climb). 22:36:11 a player sat in
  and switched autopilot off; UpdateHillHelp ended the help because bOn needs self-driving, and ApplyLocomotivePower
  applied the boost only to self-driving trains, so at x1.00 (2000 kN against 2240 kN of slope) it rolled back
  through the Coal and an Iron Plate train. Help now ends only over the climb or when switched off, and the boost
  applies whoever drives. Also: "ST: to Depot Sink 1/2" LOST with SDLE_StationUnreachableWithSignals. Disassembly of
  FRailroadGraphAStarFilter::IsTraversalAllowed (PDB public at section 1 + 0xb602f0): allowed if IgnoreSignals, or the
  connection has a facing signal, or it has no trailing signal; block validation isn't checked. So these are
  direction blocks. The LOST report now reruns FindPathSync with IgnoreSignals and lists the trailing-only signals on
  the path.
- 1.4.9: /at where and /at tp, so players can find what the log names (Satisfactory shows no coordinates). Compass:
  DefaultGame.ini [FGMapCompassSettings] mCompassCardinalDirections puts North at (0, -1) and East at (1, 0), so
  bearing = atan2(dx, -dy), and view yaw 0 (+X) is bearing 90. The marker is one per player, found again by its name
  "AT <player>" so it survives restarts without saved state. Its icon is copied from any existing map marker, else
  mDefaultStampIconID (656): the map marker icon IDs live in the cooked icon library, so this is a guess to check in
  game. /at tp uses ACharacter::TeleportTo on the server (encroachment check, 3 m above the place); untested on the
  dedicated server.
