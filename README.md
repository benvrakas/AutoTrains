# Auto Trains

A Satisfactory mod for train traffic without block or path signals. Every self-driving train books the track and
switches on its way as time slots around the other trains (space-time planning), and speeds up, slows down, waits or
reroutes so that no two trains ever need the same track at the same time. Trains follow each other through
roundabouts, take the fastest route through everyone's bookings, and docked trains' departures are booked ahead from
their timetables.

- Existing signals don't stop trains while traffic control is on; they only keep the game's one-way rule (no passing
  a signal from behind), so a signal marks track as one-way. `/at removesignals confirm` removes them all.
- Optional: colliding trains pass through each other instead of derailing.
- Works with DynamicTimeTable and Freight Car Control; replaces DynamicTrainRoutes.
- Nothing is written into save data; turning traffic control off (or uninstalling) gives trains back to signals.

![Icon](Docs/Images/icon-512.png)

- Mod page text and changelog: [Docs/Release](Docs/Release)
- How the traffic works, a test checklist and every `/at` command: [Docs/Testing.md](Docs/Testing.md)
- Logs, telemetry and fix history: [Docs/Debugging.md](Docs/Debugging.md)
- Icon: `python Tools/make_icon.py`
- Build: place the folder in the SML starter project's `Mods` folder and package it with Alpakit (Windows, Windows
  server and Linux server targets)
