# Test build 0.2.1: can the mod drive trains itself?

The finished mod plans every train's speed along its route so trains never meet, without signals. That needs the
mod to command speed and stop points precisely, ignore signals, choose routes, and know where every train is. This
build tests those controls on their own; there's no automatic traffic planning in it yet.

Back up the save first. Only command trains on track where nothing else is in their way: a train told to ignore
signals, or routed somewhere new, has no collision protection from this build.

## Commands

`<train>` is the number from `/at list`, or part of the train's name.

- `/at list`: every train with its number, speed, length, command and distance to its goal
- `/at speed <train> <km/h>`: cap its speed (`0` stops it)
- `/at stopat <train> <m>`: stop it that many metres ahead; the log says how far short of or past the point it stopped
- `/at release <train>` or `/at release all`
- `/at signals <train> ignore|obey`
- `/at route <train> check`: this mod's router against the game's route to the train's current stop
- `/at route <train> via`: send the train to its stop through the track nearest you
- `/at log on|off`: telemetry every 0.2 s to `%LOCALAPPDATA%\FactoryGame\Saved\AutoTrains\telemetry-<time>.csv`

## Steps

Run `/at log on` first and `/at log off` at the end.

1. **Speed.** On a self-driving train at full speed: `/at speed <train> 30`, wait, `/at speed <train> 0`, wait,
   `/at release <train>`. Does it hold 30 km/h, stop, then carry on to its stop normally?
2. **Stop points.** At full speed: `/at stopat <train> 300`. Repeat with an empty and a loaded train, and on a slope
   if you have one. Note roughly where each stopped.
3. **Signals.** With a train waiting at a red signal and nothing ahead of it: `/at signals <train> ignore`. Does it
   go? Then `/at signals <train> obey`.
4. **Router.** On a few trains heading to a stop: `/at route <train> check`. Chat should say `SAME ROUTE`.
5. **Custom route.** Stand next to a track on a different way to a train's next stop: `/at route <train> via`. Does
   the train take that way, set the switches, and dock at the station normally?
6. **Your other mods.** Repeat 1 and 5 with a train that uses a DynamicTimeTable group and has Freight Car Control
   car rules. Do its stops and freight car rules still work?

Then send the telemetry CSV and `%LOCALAPPDATA%\FactoryGame\Saved\Logs\FactoryGame.log`.

## What each step answers

| Step | Question |
|---|---|
| 1 | Can the mod set any train's speed directly, and how closely does the train follow it? |
| 2 | How precisely can it stop a train at a chosen point, and how do braking distances vary by load and slope? |
| 3 | Can it make a train disregard the game's signals? |
| 4 | Does its own router match the game's routes, so it can replace the routing? |
| 5 | Will a train follow a route the mod chose, switches and docking included? |
| 6 | Do DynamicTimeTable and Freight Car Control keep working? |
| all | What does the control cost per tick (the log reports it every 10 s while logging)? |
