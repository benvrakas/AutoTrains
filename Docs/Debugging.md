# Debugging

## Logs

The game log is `%LOCALAPPDATA%\FactoryGame\Saved\Logs\FactoryGame.log`. Filter for `LogAutoSignals`.

At the default level the mod logs:

- startup: `AutoSignals module starting`, `Hooks installed`
- every `/sigtest` command, and what it did
- each test signal spawned: `Spawned block|path signal <name> at <location>: guards <n> (...), observes <n> (...),
  valid connections yes|no, attached to track yes|NO`, then `Railroad subsystem registered <signal>`
- `Signal blocks rebuilt for track graph <id>: <n> blocks` whenever the game rebuilds blocks
- `/sigtest info`: every test signal (aspect, block, validation), every track graph and its blocks, every train
  (status, speed, next signal, wait time, error) with the scheduler's bookings for it
- `/sigtest watch on`: once a second, a `watch:` line for each test signal whose aspect changed and each train whose
  status, next signal, aspect, error, docking state or moving/stopped changed, with its bookings

Every `AddSignal`/`RemoveSignal` call, the player's signals included, is logged at `Verbose`. To raise the level, add
to `%LOCALAPPDATA%\FactoryGame\Saved\Config\Windows\Engine.ini`:

```
[Core.Log]
LogAutoSignals=Verbose
```

## Reading bookings

A booking line reads `block <id> (path block|normal block, occupied|free): path|exclusive booking
approved|pending|cancelled, entry <signal>, exit <signal>, path segments <a>..<b>, <n> tracks left`.

- A path booking with an exit signal followed by an exclusive booking of the next normal block means the scheduler
  books the block after a junction's exit. A path booking alone means it doesn't.
- `depends on <n> trains` and `dependency level` come from the scheduler's deadlock ordering.

## Test signals

- They're the game's own block and path signal actors, spawned by the mod with the tag `AutoSignals.TestSignal`.
- They're never written to the save (the mod hooks `AFGBuildable::ShouldSave_Implementation` for that tag), so a
  reload removes them.
- Dismantling one with the build gun works like any signal while it's visible.
