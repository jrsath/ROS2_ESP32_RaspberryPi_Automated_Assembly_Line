# CAD reconstruction validation

The simulation geometry in this package was reconstructed from the supplied `Conveyor Robot Rig` SolidWorks archive rather than from hand-built placeholder primitives.

- 18 supplied `.SLDPRT` files produced 18 binary STL meshes from their saved display tessellation.
- The top-level `Conveyor Robot Rig.SLDASM` component tree supplies 4 `Robot Assembly` occurrences, 11 non-robot static component occurrences and 16 `Box Lower` occurrences.
- The four robot base placements in the reconstructed Gazebo frame are approximately X = -0.56254, -0.21254, +0.21254 and +0.56254 m, matching their four occurrences in the supplied assembly.
- The supplied SolidWorks assembly bounding box is X [-0.620, 0.826] m, Y [-0.014, 0.41183] m, Z [-0.588, 0.12265] m in its native assembly frame. The extracted meshes and transforms reproduce the dominant assembly extents; the package applies one global coordinate conversion so SolidWorks vertical maps to Gazebo Z-up.
- Each SO101 visual link uses the corresponding provided CAD part (`Base`, `Shoulder_Rotation_Pitch`, `Upper_Arm`, `Lower_Arm`, `Wrist_Pitch_Roll`, `Fixed_Gripper`, `Moving_Jaw`). At simulated q=0 the link transforms reproduce the saved `Robot Assembly.SLDASM` pose.
- Source and generated-mesh SHA-256 hashes and triangle counts are stored in `source_manifest.json`.

The STL geometry is the saved SolidWorks display tessellation, not a re-evaluation of the native Parasolid B-rep. It is therefore appropriate as the provided visual/physics mesh but should not be treated as a manufacturing export with guaranteed CAD tolerance.

Static package validation performed during repair: Python syntax compilation, XML parsing, YAML parsing, binary-STL size/triangle consistency, source-reference cleanup and ZIP integrity. ROS 2 Humble/Gazebo could not be executed in the repair environment, so the first launch on the project Ubuntu machine remains the runtime integration test.

## ROS integration repairs

- Removed all stale `group4_project2`/`project2_cell` references and installed all Python executables under `lib/group4_assembly_line`.
- Controller parameters are explicit for `/assembly_left`, `/assembly_right`, `/inspection` and `/packaging`, so each Gazebo plugin receives its own `update_rate` and controller `type` values.
- `joint_state_broadcaster` uses local topics for each robot to prevent four identical SO101 joint names colliding on a shared `/joint_states` topic; the state publisher and arm abstraction are wired to each robot's local broadcaster topic.
- Simulation and real hardware share the same logical SO101 API; only the backend/topic mapping changes.
- `fake_io.py` provides the micro-ROS-facing state/command contract with no ESP32 attached, while the serial micro-ROS agent can be started optionally when a board is connected.

## 2026-10-01 runtime startup repair

- Added `wait_for_sim.py` readiness gate for all four arm/gripper trajectory actions plus complete six-joint feedback.
- RViz and the cell FSM now start only after the simulated robot interfaces are ready.
- Increased controller-manager spawn timeout to 180 s.
- `joint_control.py` now waits up to 120 s by default and exposes `--wait-timeout`.
- Corrected RViz robot TF prefixes to include the namespace separator.
