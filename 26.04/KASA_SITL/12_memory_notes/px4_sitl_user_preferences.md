---
name: px4-sitl-user-preferences
description: How the user wants GPS/EKF param changes and debugging handled on the PX4 SITL tarot680pro project
metadata:
  node_type: memory
  type: feedback
  originSessionId: c262037b-e3a4-43c3-9299-846565ccd7e0
  modified: 2026-11-04T13:55:51.902Z
---

Never change GPS-related EKF2/sensor parameters (e.g. `EKF2_GPS_CTRL`, `EKF2_OF_CTRL`, `COM_RC_IN_MODE`) proactively — always ask first before touching them.
**Why:** GPS/estimator behavior on this project has already caused a lot of confusing debugging (see [[px4_sitl_tarot680pro_state]] for the settled conclusions on GPS_CTRL=0 vs 3); the user wants control over this specific knob.
**How to apply:** If a task seems to call for adjusting these params, propose the change and wait for confirmation rather than editing and testing it directly.

Minimize exploratory/trial-and-error debugging loops on this project — the user is credit-sensitive.
**Why:** stated explicitly during handoff; repeated broad "try things and see" loops burn credits without guaranteed progress.
**How to apply:** Do one targeted, specific change/test at a time rather than branching into multiple speculative fixes. Before re-debugging something, check [[px4_sitl_tarot680pro_state]] for findings already marked settled.
