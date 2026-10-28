<!--
Text for the Auto Trains page on ficsit.app (Satisfactory Mod Repository).
- "Short description" field: the line under SHORT DESCRIPTION (at most 128 characters).
- "Full description" field: everything under FULL DESCRIPTION.
- The two transparency fields: the text under NETWORK ACTIVITY TRANSPARENCY and AI USAGE TRANSPARENCY.
- Logo: Docs/Images/icon-1080.png (full resolution; icon-512.png is the same image smaller).
-->

# SHORT DESCRIPTION

Train traffic without block or path signals: trains slow down, wait or take another route so they never collide.

# FULL DESCRIPTION

## Auto Trains

No more signals. Auto Trains takes over the traffic of every self-driving train. Each train plans its trip through
the whole network ahead of time, booking every track and switch it will use for the moments it will be there, and
speeds up, slows down, waits or takes another route so that no two trains ever need the same piece of track at the
same time. Build junctions, roundabouts, crossings and single track however you like, and let the trains sort it out.

### How it works: safe interval path planning

The planner is built on **safe interval path planning (SIPP)**, a method from robotics for moving many vehicles through
a shared map without collisions. The idea is small enough to follow in four steps.

**1. Every piece of track has a timeline.** Each track and each switch has its own booking calendar. Free time is
open; time when a train will be on it is booked. A train is on a track from the moment its front enters until its tail
has left, so a long train books a place for longer than a short one.

```
Junction A    time ->
              0s        20s       40s       60s       80s
  Train 1     ░░░░░░░░░░██████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░    <- already planned
  Train 2     ░░░░░░░░░░░░░░░░░░░░░░░░░░██████░░░░░░░░░░░░░    <- planned next
  Free:       [ safe 0-10 s ]            [ safe 30-... s ] ...
```

**2. Free stretches of time are the "safe intervals".** The gaps between bookings are windows in which a train can
use the track without meeting anyone. A small safety margin is kept on both sides of every booking, and it's the only
margin: there are no hidden extra seconds.

**3. Each train searches for the earliest way through the gaps.** The search doesn't just look for the shortest path.
It looks for the earliest arrival through a sequence of track *and* time windows, using the train's real acceleration,
braking distance, speed and length. Three things fall out of that for free:

- A train may **slow down to reach a crossing just as the other train clears it**, instead of stopping and starting.
- A train may **go first** through a junction ahead of a train that is still far away.
- Waits **land where they cost least**, and a waiting train only stands where its whole body leaves every crossing and
  switch that other trains are booked on free.

**4. Trains are planned one after another.** The first train plans around nothing. Each next train plans around the
bookings of the trains before it, then books its own plan. Because every plan is made around the earlier ones, they
always fit together: no two bookings overlap, so no two trains meet. A fixed order stops trains from each waiting for
the other, and a train that is already ahead of another on its route goes first.

Plans are redone continuously, several times a second. When a train runs late, brakes harder than expected or a new
train joins, everyone's plan is updated and the bookings move with them. The planning runs on worker threads, so it
doesn't cost frame time.

**What the trains get from it**

- **Fastest route.** The search covers the whole network, so a train that would wait a long time on its usual route
  takes another one, even a longer one, when that gets it there clearly sooner. Rerouting is conservative: a new route
  must save a set time and share of the trip (settings below), and it's kept for a while.
- **Timetables known ahead.** A docked train's departure is booked ahead from how long it stayed there last time and
  where its timetable sends it next, so no train queues in the way another needs to leave.
- **Never stuck in a junction.** A train only drives into a stretch it can get through; it won't stop with its body
  across a crossing or switch someone else is booked on.
- **Brakes it can trust.** Each train measures how hard it really brakes at each speed and plans with that. If a
  train still can't stop in time, the mod brakes it harder than its brakes could, the way block signals do.
- **Deadlocks cleared.** Trains waiting for each other in a circle get the game's warning over them. With the beta
  setting on (the default), after a minute they pass through each other to clear the jam, and the case is written to
  a log you can send in.

The method follows the safe interval planning of Phillips and Likhachev (2011) and its extension to long vehicles,
[MT-SIPP](https://doi.org/10.3390/machines12070482) (Miao et al., 2024), which books the body of a train and not just
its head. Auto Trains adds real train motion, whole-network route choice, learned station stays and continuous
replanning.

### Settings (Mods menu)

| Setting | Default | What it does |
|---|---|---|
| **Traffic control** | On | Auto Trains drives every self-driving train. Off gives the game's signals back. |
| **Congestion rerouting** | On | Trains take another route when it gets them there sooner than waiting on theirs. |
| **Trains pass through each other** | On | While traffic control is on, trains that touch pass through each other instead of crashing. A misjudged gap then only looks like a clip. |
| **Aggressive timing** | On | Tighter margins: trains plan braking closer to what they really manage. Off is more cautious. |
| **Safety margin (seconds)** | 0 | Time, 0 to 10 s, kept between two trains using the same track or switch. 0 lets a train in as soon as the one before it has cleared; raise it if trains clip each other. |
| **Top speed (km/h)** | 0 | Top speed of every self-driving train, up to 500 km/h. 0 keeps each train's own. Above the speed at which a locomotive's traction runs out (about 120 km/h on flat track), its traction is stretched up to the setting and it draws that much more power. |
| **Acceleration (x)** | 1 | Traction of self-driving trains' locomotives, 0.25 to 5 times their own: they accelerate that much faster and draw that much more power from the grid. Braking isn't changed. |
| **Reroute: minimum time saved (seconds)** | 30 | A train only takes another route if it gets there at least this much sooner. Higher keeps trains on their usual route. |
| **Reroute: minimum share saved (%)** | 20 | ...and if it saves at least this share of the trip's time. |
| **Reroute: keep a new route for (seconds)** | 60 | After taking another route, a train doesn't change again for this long. |
| **Lookahead (m)** | 100 | Track a train is given past its braking distance. More lets trains run faster towards traffic; less keeps track free for others. |
| **Stop distance (m)** | 10 | How far a waiting train stops short of track it hasn't been given (before a switch at least 25 m and the switch's fouling zone plus this). |
| **Normal braking (%)** | 85 | How hard trains brake for normal stops, as a share of the braking they plan with. |
| **Station stay estimate (seconds)** | 40 | How long a train is expected to stay at a station before it has been seen there; after that its own last stay counts. |
| **Clear deadlocks by passing through (beta)** | On | Trains stuck waiting for each other in a circle drive on through each other after the set time; each case is written to FactoryGame/Saved/AutoTrains/faults.log. |
| **Pass through a deadlock after (seconds)** | 60 | How long a deadlock lasts before its trains pass through each other. |
| **Planning retries** | 3 | When a train finds no way because a train planned before it is in the way, it is planned before that train and the round is planned again, up to this many times. 0 keeps the planning order as it is. |
| **Update interval (seconds)** | 0.25 | How often trains are planned and given track. Shorter reacts faster but costs more on big networks. |

### Getting started

1. **Back up your save.**
2. Install the mod and load your world. Existing signals stay where they are but no longer stop trains while traffic
   control is on, so you can try the mod without removing anything.
3. To get rid of signals for good, type `/at removesignals confirm` in chat. It dismantles every block and path
   signal in the world. There's no refund and no undo.

### One-way track

Signals still set direction, as in the game: no train passes a signal from behind. To keep trains on the right side of
a double track, or to make a link one-way, place one block signal facing the way trains should go. A signal on each
side of the track (back to back) allows both ways.

### Works with

- **[DynamicTimeTable](https://ficsit.app/mod/DynamicTimeTable)** and
  **[Freight Car Control](https://ficsit.app/mod/FreightCarControl)**: stations, docking and timetables work as
  usual.
- **Not** with mods that set train routes themselves, such as DynamicTrainRoutes. Auto Trains does its own routing,
  so remove them.
- Multiplayer: the host (or dedicated server) runs all traffic control, using the host's settings. Everyone needs the
  mod.

### Good to know

- **Player-driven trains** aren't controlled. Self-driving trains treat them as obstacles, but a player can still
  drive into another train.
- **Uninstalling** returns trains to the game's signals. The mod stores nothing in your save. If you removed your
  signals, place them again before uninstalling, or self-driving trains will run into each other.
- `/at traffic status` lists every moving self-driving train, how far it may drive and what it's waiting for. It's
  the first place to look when a train seems stuck.

### Chat commands

`/at traffic on|off|status`, `/at reroute on|off`, `/at list`, `/at removesignals confirm`, `/at autopilot <train|all>
on|off`, `/at session <new name>` (renames the running session, for a copy of a save), `/at log on|off` (writes
telemetry for bug reports). In multiplayer only the host or a server admin can change anything; everyone can use
`/at list` and `/at traffic status`.

### Bugs and ideas

Report them on [GitHub](https://github.com/benvrakas/AutoTrains/issues), ideally with your `FactoryGame.log`. The
source code is there too.

# NETWORK ACTIVITY TRANSPARENCY

Auto Trains doesn't connect to the internet. In multiplayer all traffic control runs on the host; players see the
trains through the game's own connection. There's no telemetry or analytics: `/at log on` writes a CSV file to your
own computer only, and deadlocks cleared by passing through are written to `faults.log` there.

# AI USAGE TRANSPARENCY

This mod was built with the help of an AI coding assistant (Anthropic's Claude):

- **Source code:** the mod's C++ source code was written by the AI, directed, reviewed and tested in game by the
  author.
- **Mod page text:** this description was drafted by the AI and edited by the author.
- **Developer documentation** in the source folder was written by the AI.
- **Icon:** drawn by a Python script written by the AI (Tools/make_icon.py).
- **In game:** the mod doesn't use or give access to generative AI.
