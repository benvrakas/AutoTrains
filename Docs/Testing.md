# Test build checklist (0.1.0)

## Setup

- A copy of a world or a fresh save. "No Build Cost" in Advanced Game Settings makes building quick.
- A loop of track with two stations, one junction (a branch that rejoins the loop, or a crossing), and two
  self-driving trains with timetables between the stations. No player signals on this network.
- The full mod profile, DynamicTrainRoutes included. Single player.

## Commands

- `/sigtest place block` or `/sigtest place path`: a test signal at the track joint nearest you, for trains moving
  the way you look. Add `both` for the opposite direction too.
- `/sigtest hide`, `/sigtest show`, `/sigtest clear`
- `/sigtest info`: logs everything; `/sigtest watch on|off`: logs changes every second

## Checks

1. Two block signals splitting the loop into two blocks: the trains keep their distance and stop for each other.
2. Path signals in front of the junction: trains wait before it instead of stopping inside it.
3. DynamicTrainRoutes routes around the new blocks, or misbehaves.
4. `/sigtest hide`: everything keeps working.
5. Save and reload: the save loads and the test signals are gone.

Run `/sigtest watch on` before the checks and `/sigtest info` at each step, then send `FactoryGame.log`.
