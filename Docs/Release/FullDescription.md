## Auto Trains

No more signals. Auto Trains takes over the traffic of every self-driving train. Each train plans its trip through
the whole network ahead of time, booking every track and switch it will use for the moments it will be there, and
speeds up, slows down, waits or takes another route so that no two trains ever need the same piece of track at the
same time. Build junctions, roundabouts, crossings and single track however you like, and let the trains sort it out.

### How it works: safe interval path planning

The planner is built on **safe interval path planning (SIPP)**, a method from robotics for moving many vehicles through
a shared map without collisions. The idea is small enough to follow in four steps.

**1. Every piece of track has a timeline.** Each track, each switch and each crossing has its own booking calendar. Free time is
open; time when a train will be on it is booked. A train is on a track from the moment its front enters until its tail
has left, so a long train books a place for longer than a short one.

<div align="center">
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 720 256" width="720" role="img" aria-label="Booking calendars of two tracks and a junction: train 1 books the junction first, train 2 is planned around it and enters just after train 1 clears">
<rect width="720" height="256" rx="10" fill="#1b1f27"/>
<defs><pattern id="atsafe" width="8" height="8" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><rect width="8" height="8" fill="#16301f"/><line x1="0" y1="0" x2="0" y2="8" stroke="#3fb96b" stroke-width="2"/></pattern></defs>
<text x="16" y="26" fill="#e8ecf2" font-family="sans-serif" font-size="15" font-weight="bold">Every piece of track has a booking calendar</text>
<text x="16" y="45" fill="#9aa5b5" font-family="sans-serif" font-size="12">Two trains meet at a junction. Train 1 is planned first; train 2 plans around it.</text>
<g font-family="sans-serif" font-size="12" fill="#c5cdd9"><text x="16" y="75">Track A</text><text x="16" y="125">Track B</text><text x="16" y="175">Junction</text></g>
<g stroke="#2d3441"><line x1="130" y1="60" x2="690" y2="60"/><line x1="130" y1="110" x2="690" y2="110"/><line x1="130" y1="160" x2="690" y2="160"/><line x1="130" y1="190" x2="690" y2="190"/></g>
<rect x="130" y="146" width="84" height="28" fill="url(#atsafe)"/>
<rect x="340" y="146" width="350" height="28" fill="url(#atsafe)"/>
<rect x="214" y="146" width="14" height="28" fill="#3b82f6" fill-opacity="0.35"/>
<rect x="326" y="146" width="14" height="28" fill="#3b82f6" fill-opacity="0.35"/>
<rect x="228" y="146" width="98" height="28" rx="4" fill="#3b82f6"/>
<text x="277" y="164" fill="#fff" font-family="sans-serif" font-size="12" text-anchor="middle" font-weight="bold">train 1</text>
<rect x="242" y="146" width="98" height="28" rx="4" fill="none" stroke="#ef4444" stroke-width="2" stroke-dasharray="5 4"/>
<text x="291" y="140" fill="#ef6a6a" font-family="sans-serif" font-size="11" text-anchor="middle">train 2 unplanned: overlap</text>
<rect x="340" y="146" width="98" height="28" rx="4" fill="#f59e0b"/>
<text x="389" y="164" fill="#1b1f27" font-family="sans-serif" font-size="12" text-anchor="middle" font-weight="bold">train 2</text>
<rect x="130" y="46" width="126" height="28" rx="4" fill="#3b82f6"/>
<text x="193" y="64" fill="#fff" font-family="sans-serif" font-size="12" text-anchor="middle" font-weight="bold">train 1</text>
<rect x="130" y="96" width="224" height="28" rx="4" fill="#f59e0b"/>
<text x="242" y="114" fill="#1b1f27" font-family="sans-serif" font-size="12" text-anchor="middle" font-weight="bold">train 2 slows down</text>
<text x="172" y="186" fill="#3fb96b" font-family="sans-serif" font-size="11" text-anchor="middle">safe interval</text>
<text x="277" y="186" fill="#7aa7f0" font-family="sans-serif" font-size="11" text-anchor="middle">safety margin each side</text>
<text x="565" y="186" fill="#3fb96b" font-family="sans-serif" font-size="11" text-anchor="middle">safe interval, open ended</text>
<g stroke="#9aa5b5"><line x1="130" y1="204" x2="690" y2="204"/><line x1="130" y1="200" x2="130" y2="208"/><line x1="214" y1="200" x2="214" y2="208"/><line x1="298" y1="200" x2="298" y2="208"/><line x1="382" y1="200" x2="382" y2="208"/><line x1="466" y1="200" x2="466" y2="208"/><line x1="550" y1="200" x2="550" y2="208"/><line x1="634" y1="200" x2="634" y2="208"/></g>
<g font-family="sans-serif" font-size="11" fill="#9aa5b5" text-anchor="middle"><text x="130" y="222">0 s</text><text x="214" y="222">10</text><text x="298" y="222">20</text><text x="382" y="222">30</text><text x="466" y="222">40</text><text x="550" y="222">50</text><text x="634" y="222">60</text></g>
<text x="690" y="244" fill="#9aa5b5" font-family="sans-serif" font-size="11" text-anchor="end">time</text>
</svg>
</div>

<p align="center"><sub>Figure 1: bookings on a junction, with the free time between them</sub></p>


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

<div align="center">
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 720 270" width="720" role="img" aria-label="Distance against time for train 2 approaching a crossing that train 1 has booked: the unplanned path runs into the booking, the planned path slows down and arrives just after it ends">
<rect width="720" height="270" rx="10" fill="#1b1f27"/>
<defs><marker id="atarr" viewBox="0 0 8 8" refX="7" refY="4" markerWidth="7" markerHeight="7" orient="auto"><path d="M0 0L8 4L0 8z" fill="#f59e0b"/></marker></defs>
<text x="16" y="26" fill="#e8ecf2" font-family="sans-serif" font-size="15" font-weight="bold">Planning in space and time: slow down instead of stopping</text>
<text x="16" y="45" fill="#9aa5b5" font-family="sans-serif" font-size="12">Distance to the crossing against time. The blue block is train 1's booking of the crossing.</text>
<g stroke="#2d3441"><line x1="70" y1="215" x2="690" y2="215"/><line x1="70" y1="70" x2="70" y2="215"/></g>
<rect x="250" y="70" width="200" height="145" fill="#3b82f6" fill-opacity="0.10"/>
<rect x="270" y="70" width="160" height="145" fill="#3b82f6" fill-opacity="0.22"/>
<rect x="70" y="70" width="620" height="16" fill="#2d3441"/>
<rect x="270" y="70" width="160" height="16" fill="#3b82f6"/>
<text x="78" y="82" fill="#c5cdd9" font-family="sans-serif" font-size="11">crossing</text>
<text x="350" y="82" fill="#fff" font-family="sans-serif" font-size="11" text-anchor="middle" font-weight="bold">booked by train 1</text>
<line x1="70" y1="215" x2="270" y2="86" stroke="#3b82f6" stroke-width="3"/>
<text x="135" y="168" fill="#7aa7f0" font-family="sans-serif" font-size="12" font-weight="bold" transform="rotate(-33 135 168)">train 1</text>
<line x1="130" y1="215" x2="330" y2="86" stroke="#ef4444" stroke-width="2.5" stroke-dasharray="6 5"/>
<circle cx="330" cy="86" r="5" fill="#ef4444"/>
<text x="470" y="112" fill="#ef6a6a" font-family="sans-serif" font-size="12">unplanned: reaches the</text>
<text x="470" y="128" fill="#ef6a6a" font-family="sans-serif" font-size="12">crossing while it is booked</text>
<path d="M130 215 Q 300 196 450 86" fill="none" stroke="#f59e0b" stroke-width="3.5" marker-end="url(#atarr)"/>
<circle cx="450" cy="86" r="5" fill="#3fb96b"/>
<text x="470" y="170" fill="#f59e0b" font-family="sans-serif" font-size="12" font-weight="bold">train 2 as planned:</text>
<text x="470" y="186" fill="#f59e0b" font-family="sans-serif" font-size="12">eases off, then arrives as</text>
<text x="470" y="202" fill="#f59e0b" font-family="sans-serif" font-size="12">train 1 clears. It never stops.</text>
<g stroke="#9aa5b5"><line x1="70" y1="211" x2="70" y2="219"/><line x1="170" y1="211" x2="170" y2="219"/><line x1="270" y1="211" x2="270" y2="219"/><line x1="370" y1="211" x2="370" y2="219"/><line x1="470" y1="211" x2="470" y2="219"/><line x1="570" y1="211" x2="570" y2="219"/><line x1="670" y1="211" x2="670" y2="219"/></g>
<g font-family="sans-serif" font-size="11" fill="#9aa5b5" text-anchor="middle"><text x="70" y="233">0 s</text><text x="170" y="233">10</text><text x="270" y="233">20</text><text x="370" y="233">30</text><text x="470" y="233">40</text><text x="570" y="233">50</text><text x="670" y="233">60</text></g>
<text x="690" y="254" fill="#9aa5b5" font-family="sans-serif" font-size="11" text-anchor="end">time</text>
<text x="12" y="150" fill="#9aa5b5" font-family="sans-serif" font-size="11" transform="rotate(-90 12 150)" text-anchor="middle">distance</text>
</svg>
</div>

<p align="center"><sub>Figure 2: the same idea as a path through space and time</sub></p>

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
- **Timetables known ahead.** A docked train's departure is booked ahead, so no train queues in the way another needs
  to leave. A train stopping at a dead-end station holds the track back to the last switch until it has backed out,
  so push-pull trains never meet head-on on single track. A stop set to load or unload once takes the freight platforms' fixed transfer time when there is cargo to
  move, and the stop's set duration when there isn't, so the departure is known as soon as the platforms start (or
  don't). Only stops set to fully load or unload go by how long the train stayed there last time. At a station
  without freight platforms, where the game lets a train go at once, no stay is planned.
- **Never stuck in a junction.** A train only drives into a stretch it can get through; it won't stop with its body
  across a crossing or switch someone else is booked on.
- **Brakes it can trust.** Each train measures how hard it really brakes and accelerates at each speed, with the
  slope taken out, and plans with that. Stopping distances and stop points add the slope of the track ahead, so trains
  stop where planned downhill and aren't held back uphill. If a train still can't stop in time, the mod brakes it
  harder than its brakes could, the way block signals do.
- **No stalls on steep ramps.** Trains are planned with what their locomotives can really climb: none is made to wait
  on a climb it couldn't start on again, and one held up before such a climb waits at its foot until its way to the
  top is clear. If a train stalls on one anyway, it gets extra traction until it is over the steep part.
- **Deadlocks cleared.** Trains waiting for each other in a circle get the game's warning over them. With the beta
  setting on (the default), after a minute they pass through each other to clear the jam, and the case is written to
  a log you can send in. A train held up that long with no train to wait for also gets the warning and is logged as
  stuck.

The method follows the safe interval planning of Phillips and Likhachev (2011) and its extension to long vehicles,
[MT-SIPP](https://doi.org/10.3390/machines12070482) (Miao et al., 2024), which books the body of a train and not just
its head. Auto Trains adds real train motion, whole-network route choice, learned station stays and continuous
replanning.

### Settings (Mods menu)

<details>
<summary><b>All settings and their defaults</b> (click to open)</summary>

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
| **Stop distance (m)** | 10 | How far a waiting train stops short of track it hasn't been given (before a switch, the switch's fouling zone plus this). |
| **Track clearance (m)** | 3 | How close the centre lines of two tracks may come before trains on them count as in each other's way. A switch's branches block each other only until they are this far apart, and where two tracks cross only the stretch where they come this close counts, so a train can wait right before a crossing. Smaller lets trains slip past each other closer to switches (they may clip); larger keeps them further apart. |
| **Normal braking (%)** | 85 | How hard trains brake for normal stops, as a share of the braking they plan with. |
| **Station stay estimate (seconds)** | 40 | How long a train is expected to stay at a stop set to fully load or unload, until it has been seen there; after that its own last stay there counts. Stops that load or unload once take the freight platforms' fixed time, or the stop's set duration when there's no cargo to move. |
| **Clear deadlocks by passing through (beta)** | On | Trains stuck waiting for each other in a circle drive on through each other after the set time; each case is written to FactoryGame/Saved/AutoTrains/faults.log. |
| **Pass through a deadlock after (seconds)** | 60 | How long a deadlock lasts before its trains pass through each other. |
| **Help stalled trains up hills** | On | A train that has slowed down on a climb too steep for its locomotives to get going again gets extra traction until it is over the steep part. Trains are planned not to wait on such climbs; this is for when one has to anyway. |
| **Hill help: most extra traction (x)** | 4 | The most traction a stalled train gets on a climb, 1 to 10 times its own. Raise it if heavy trains still stall on very steep ramps. |
| **Planning retries** | 3 | When a train finds no way because a train planned before it is in the way, it is planned before that train and the round is planned again, up to this many times. 0 keeps the planning order as it is. |
| **Route search interval (seconds)** | 5 | How often each train looks for a faster route. Shorter finds better routes sooner; longer costs less planning time on big networks. |
| **Route search when late (seconds)** | 15 | A train looks for another route at once when its planned arrival has got this much later since its last search. |
| **Route search while stuck (seconds)** | 1 | How often a train whose plan finds no way looks for another route. |
| **Track refresh (seconds)** | 30 | How often the whole track network is read again. Track built or removed is usually picked up within 2 seconds anyway. |
| **Deadlock: count trains waiting longer than (seconds)** | 20 | How long a train has to wait before it counts when looking for deadlocks. |
| **Deadlock pass-through distance (m)** | 300 | A train passing through a deadlock does so until it has driven its own length and this much further. |
| **Deadlock pass-through time limit (seconds)** | 120 | A train passes through a deadlock for at most this long. |
| **Plan search limit (states)** | 200000 | Most steps one train's plan may search. Lower uses less CPU and memory on huge networks; long or busy routes may then find no way more often. |
| **Slope lookahead (m)** | 300 | How far ahead of a train the slope is averaged for its braking. |
| **Update interval (seconds)** | 0.25 | How often trains are planned and given track. Shorter reacts faster but costs more on big networks. |

</details>

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

### Chat commands

Type them in the game chat. A `<train>` is its number from `/at list`, or part of its name.

**Look at what the trains are doing** (anyone can use these)

| Command | What it does |
|---|---|
| `/at list` | Every train with its number, speed, length and distance to its goal. |
| `/at traffic status` | Every moving self-driving train, how far it may drive and what it is waiting for. The first place to look when a train seems stuck. |

**Switch things on and off** (host or server admin)

| Command | What it does |
|---|---|
| `/at traffic on` / `off` | Auto Trains drives every self-driving train, or gives the game's signals back. Same as the Traffic control setting. |
| `/at reroute on` / `off` | Congestion rerouting on or off. Same as the Congestion rerouting setting. |
| `/at autopilot <train> on` / `off` | Self-driving for one train. |
| `/at autopilot all on` / `off` | Self-driving for every train. |

**One-off jobs** (host or server admin)

| Command | What it does |
|---|---|
| `/at removesignals confirm` | Dismantles every block and path signal in the world, with no refund and no undo. Without `confirm` it only explains what it would do. |
| `/at log on` / `off` | Writes telemetry for bug reports to a CSV file in `%LOCALAPPDATA%\FactoryGame\Saved\AutoTrains`. |

In multiplayer only the host or a server admin can change anything.

### Bugs and ideas

Report them on [GitHub](https://github.com/benvrakas/AutoTrains/issues), ideally with your `FactoryGame.log`. The
source code is there too.
