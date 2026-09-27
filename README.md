# Surgical Arm Research Code

This repository contains the independently added surgical-arm research scripts and VLM service. The local Desktop folder also contains a Unity 6000.6.0f1 project copied from the working prototype to preserve its current scene and asset references. Those inherited Unity files are excluded from Git; this repository alone is **not** a complete Unity project.

To run the local prototype, open this folder as a project in Unity Hub with editor 6000.6.0f1. The `RobotControl` scene in `Assets/Scenes` is the configured start scene. The editor script automatically opens and plays it after import. Do not delete the locally retained scene, model, `RobotMove` component, package manifest, or project settings: the research scripts still depend on them.

Publishing a reproducible standalone Unity project requires replacing the inherited scene, models, scripts, and other third-party assets with original or appropriately licensed alternatives, or obtaining permission and retaining required attribution. Do not publish the local ignored files as your own.