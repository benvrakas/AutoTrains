<!-- Paste into the Changelog field when uploading 1.4.14 to ficsit.app (published there: 1.4.5). Covers 1.4.6 to 1.4.14. -->

- Fixed: a train stopped on a switch could block it forever (for real this time). A held-up train now keeps everything
  it can't get off, so trains on the other branch aren't sent through its switch. 1.4.4's track reservations, which
  never took effect, are removed.
- Fixed: two trains side by side before a merge, or at the same switch, could block each other or take turns starting
  for a minute or more. A train now keeps a switch it holds unless the other train can't stop any more or is ahead of
  it on the same line.
- Fixed: passing through a deadlock could strand a train. One train passes through at a time now, preferring one whose
  way is clear.
- Fixed: a train stopped part way up a steep climb could stay there for good, and taking over a train by hand on a
  climb took its hill help away.
- Fixed: a train whose timetable changed while driving (SmartTrains does this) was stopped dead from full speed. It
  now keeps its room for up to 2 s until the game finds the new path.
- Fewer "can't stop short" conflicts: a train that can no longer stop short of something an earlier train booked now
  goes before that train, and is planned again.
- Fixed: a fast train's crossing could look free under its own body, and a train waiting at a crossing zone's edge could
  stop with its nose on the crossing.
- New: `/at where` tells you where you are in the log's metres; `/at where <place>` says how far and which way a place
  (coordinates, a signal's name, a train) is and puts a marker on your compass. `/at tp <place>` moves you there
  (host or admin only).
- For other mods: ask how busy the way into a station is. Nothing changes in how trains drive.
- The log names signals and backwards stations as the cause when a train finds no path, and trains sharing a name get a
  short id (`Train #5678`).
