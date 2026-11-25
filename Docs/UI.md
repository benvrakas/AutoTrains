# UI

All on the client; the server answers each request from `UATRemoteCallObject` (ATRemoteCallObject.h) with a view
(`ATViews.h`) made by `AATTrafficSubsystem::MakeTrainView` / `MakeMonitorView` (ATReadouts.cpp). Widgets ask once a
second while shown and refresh when a view arrives (`UATPanel`, ATPanel.h).

- **Locomotive menu** (`UATTrainPanel`): hooked at the end of `Widget_Locomotive_Menu`'s Timetable column
  (`VerticalBox_1`) by `ATUIHooks::Register` (game instance post-init; not in the editor or on a dedicated server).
  State badge, speed against `mAtcData.TargetSpeed`, goal and planned arrival, what it waits for (`BlockedBy`, else
  `PlanWaitFor`) and `BlockedReason` cut to 110 characters. **Show me**: `Server_MarkTrain`, which puts the player's
  `/at where` marker on the train waited for (`AATChatCommand::MarkLocation`).
- **Ride HUD** (`UATRideHud`): a core ticker every 0.5 s finds the train the local player sits in (the possessed
  `AFGLocomotive`, or the character's driven vehicle) and shows the strip at the top right, 60 units down (SmartTrains'
  card sits at 170 under it). Hidden while the train isn't self-driving. It never takes input.
- **Monitor** (`UATMonitorWindow`, `/at monitor`): tiles and rows of `ExternalGetTroubledTrains` plus trains waiting
  120 s or more; the planner round is the last cycle's worker time (`mLastCycleMs`) against the update interval.
- **Pass-through alert**: `GhostThrough` counts `mPassThroughs` and sends a chat line to every player who may change
  the world (`AlertAdmins`).

Style: `ATStyle` colours (FICSIT orange on dark panels), rounded brushes, the hexagon logo (`UATLogo`), the game's font
copied from the menu the panel sits in (or a hidden vanilla label for the HUD), a 0.22 s fade and rise on showing.
