<!--
Text for the Auto Trains page on ficsit.app (Satisfactory Mod Repository).
- "Short description" field: the line under SHORT DESCRIPTION (at most 128 characters).
- "Full description" field: the whole of FullDescription.md (this folder), nothing else.
- The two transparency fields: the text under NETWORK ACTIVITY TRANSPARENCY and AI USAGE TRANSPARENCY.
- Logo: Docs/Images/icon-1080.png (full resolution; icon-512.png is the same image smaller).
-->

# SHORT DESCRIPTION

Train traffic without block or path signals: trains slow down, wait or take another route so they never collide.

# FULL DESCRIPTION

In FullDescription.md.

# NETWORK ACTIVITY TRANSPARENCY

Auto Trains doesn't connect to the internet. In multiplayer all traffic control runs on the host; players see the
trains through the game's own connection. There's no telemetry or analytics: `/at log on` writes a CSV file to your
own computer only, and deadlocks cleared by passing through are written to `faults.log` there.

# AI USAGE TRANSPARENCY

This mod was built with the help of an AI coding assistant (Anthropic's Claude):

- **Source code:** the mod's C++ source code was written by the AI, directed, reviewed and tested in game by the
  author.
- **Mod page text:** this description was drafted by the AI and edited by the author.
- **Developer documentation** in the source folder was written by the AI.
- **Icon:** drawn by a Python script written by the AI (Tools/make_icon.py).
- **In game:** the mod doesn't use or give access to generative AI.
