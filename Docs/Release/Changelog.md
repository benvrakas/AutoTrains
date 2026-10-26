# Changelog

## 1.0.0

First release.

- **Train traffic without block or path signals.** Self-driving trains reserve the track ahead and slow down, wait
  or go so they never need the same track at the same time. Existing signals are ignored while traffic control is
  on.
- **Trains never stop inside a junction** or on single track another train needs.
- **Following** through junctions and roundabouts, with a moving safe distance to the train ahead.
- **Going first:** a train that clears a junction in time may go ahead of a train that has only reserved it.
- **Congestion rerouting:** trains take another route, even a longer one, when theirs is busy.
- **Trains roll on instead of stopping** when the train in their way will be clear in time: they slow down early and
  arrive just as the way frees.
- **Braking learned per train,** per speed band and allowing for cargo, so stops are planned with what the train
  really manages.
- **Trains pass through each other** instead of crashing (setting, on by default).
- Mods menu settings: Traffic control, Congestion rerouting, Trains pass through each other, Aggressive timing,
  Safety margin.
- `/at removesignals confirm` dismantles every block and path signal in the world.
- Works with DynamicTimeTable and Freight Car Control.

**Good to know**

- Back up your save before trying it. Uninstalling gives trains back to the game's signals; if you removed your
  signals, place them again first.
- Remove DynamicTrainRoutes and other mods that set train routes.
