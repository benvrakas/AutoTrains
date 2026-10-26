# AutoSignals (working name)

Satisfactory mod: self-driving trains share track without player-placed signals. Every track joint acts as a block or
path signal depending on what's beyond it, using the game's own signal blocks and train scheduler.

Current state: test build 0.1.0, which only places signals on command (`/sigtest`) to check that signals created by
code work. See `Docs/Testing.md` and `Docs/Debugging.md`.

Build with the SatisfactoryModLoader starter project; package with
`D:\SFMod\Tools\BuildScripts\package-autosignals.bat local` (Windows client only).
