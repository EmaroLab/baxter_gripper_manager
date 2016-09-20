# Baxter Moveit Gripper Manager

This package provides a simplified interface for gripper control.
when started, it automatically calibrates the grippers.

You can publish commands on this topic:

`baxter_gripper_control`

`command: "open"/"close"`
`arm: "right"/"left"/"both"`