<!--
Text for the Auto Trains page on ficsit.app (Satisfactory Mod Repository).
- "Short description" field: the line under SHORT DESCRIPTION (at most 128 characters).
- "Full description" field: everything under FULL DESCRIPTION.
- The two transparency fields: the text under NETWORK ACTIVITY TRANSPARENCY and AI USAGE TRANSPARENCY.
- Logo: Docs/Images/icon-1080.png (full resolution; icon-512.png is the same image smaller).
Replace the [bracketed] placeholder before publishing.
-->

# SHORT DESCRIPTION

Train traffic without block or path signals: trains slow down, wait or take another route so they never collide.

# FULL DESCRIPTION

## Auto Trains

No more signals. Auto Trains takes over the traffic of every self-driving train: each train reserves the track
ahead of it, and speeds up, slows down, waits or reroutes so that no two trains ever need the same track at the same
time. Build junctions, roundabouts, crossings and single track however you like, and let the trains sort it out.

### How trains share the track

- **Reservations instead of blocks.** A train reserves the track it is about to drive over and always keeps a speed
  at which it can stop before the end of its reservation. As track frees up ahead, it reserves more and speeds up.
- **Never stuck in a junction.** A train only stops where its whole length stands clear of the switches and crossings
  other trains need, and never halfway along single track another train needs to use the other way.
- **Following.** A train can follow another through a junction or roundabout instead of waiting until the whole way
  is clear, keeping enough distance to stop if the train ahead does.
- **Going first.** A short or fast train may slip through a junction ahead of a train that has only reserved it, if
  it is clear again before the other train could get there and that train can still stop in time.
- **Fair turns.** The train that would get through soonest goes first, and time spent waiting counts in a train's
  favour, so nobody waits forever.
- **Congestion rerouting.** Every few seconds a train compares its route with the alternatives, counting standing,
  moving and oncoming trains. If another route is clearly less busy it takes that one, even when it's longer.
- **Brakes it can trust.** Each train measures how hard it really brakes at each speed and plans with that. If a
  train still can't stop in time, the mod brakes it harder than its brakes could, the way block signals do.

### Settings (Mods menu)

| Setting | Default | What it does |
|---|---|---|
| **Traffic control** | On | Auto Trains drives every self-driving train. Off gives the game's signals back. |
| **Congestion rerouting** | On | Trains take another route when theirs is busy. |
| **Trains pass through each other** | On | While traffic control is on, trains that touch pass through each other instead of crashing. A misjudged gap then only looks like a clip. |
| **Aggressive timing** | On | Tighter margins: trains follow closer and slip through junctions more readily. Off is more cautious. |
| **Safety margin (seconds)** | 0 | Extra time, 0 to 10 s, a train leaves when it follows another out of a junction or goes ahead of another train. Raise it if trains clip each other. |
| **Top speed (km/h)** | 0 | Top speed of every self-driving train, up to 500 km/h. 0 keeps each train's own. Above the speed at which a locomotive's traction runs out (about 120 km/h on flat track), its traction is stretched up to the setting and it draws that much more power. |
| **Acceleration (x)** | 1 | Traction of self-driving trains' locomotives, 0.25 to 5 times their own: they accelerate that much faster and draw that much more power from the grid. Braking isn't changed. |

### Getting started

1. **Back up your save.**
2. Install the mod and load your world. Existing signals stay where they are but are ignored while traffic control is
   on, so you can try the mod without removing anything.
3. To get rid of signals for good, type `/at removesignals confirm` in chat. It dismantles every block and path
   signal in the world. There's no refund and no undo.

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
own computer only.

# AI USAGE TRANSPARENCY

[Edit to match how you made the mod.]

This mod was built with the help of an AI coding assistant (Anthropic's Claude):

- **Source code:** the mod's C++ source code was written by the AI, directed, reviewed and tested in game by the
  author.
- **Mod page text:** this description was drafted by the AI and edited by the author.
- **Developer documentation** in the source folder was written by the AI.
- **Icon:** drawn by a Python script written by the AI (Tools/make_icon.py).
- **In game:** the mod doesn't use or give access to generative AI.
