# Group 4 assembly line

Use **group4_project2_AL**. Its robot dependency is included under
`vendor/group4_assembly_line`.

**Current status:** Full starts and runs the available paths. If a movement fails,
that station holds its measured position and shows the reason in **Open log**;
other independent work continues. Edit the path, then Reset to retry. Failed
movements are not skipped and unfinished products are not counted as complete.

The V6 carrier pickup still needs clearance adjustment in Gazebo, and the five
closed-box stacking paths remain unsolved. The complete five-toy cycle has not
passed. Physical motor commands still need the lab driver mapping and calibration.

## Install and run

Use Ubuntu 22.04 / ROS 2 Humble. On Windows, first run
`wsl --install -d Ubuntu-22.04` and `wsl --update` in Administrator PowerShell.
Extract the project into Ubuntu, then:

```bash
cd ~/ROS2/group4_project2_AL
bash install_build.sh
bash run.sh sim
```

The installer builds an isolated workspace and ignores inherited ROS overlays.
Other workspaces remain intact. Rerun after code changes or a missing executable.
Windows users can use `install_build.cmd` and `run.cmd`; shell scripts run in
Ubuntu. Do not run inside a ZIP. A new run closes earlier Project 2 sessions,
not unrelated apps. Ctrl+C closes the run.

## Simulation

Choose **Task**, **No. of Products**, and **Faulty products**, then **Start**.
Start applies changed selections and waits for setup/planning. After Pause it
resumes the existing run. **Reset** restores stock and clears the run.
A later unsolved box path does not prevent Start. The affected station waits before
that action. E-stop, lost feedback and excessive drift still stop the run.

Fault positions can be random, first, last, or a comma-separated list such as
`2,4`. Zero faults gives an all-good batch. These are simulated test defects;
the camera still decides pass/fail.

| Control | Action |
|---|---|
| Full | Assemble, inspect and package |
| Assembly | Assemble/convey with simulated unloading |
| Inspection | Prebuilt toys for inspection and sorting |
| Packaging | Prebuilt toys delivered by conveyor, inspected and packaged |
| Start / Pause | Start or hold motion; active buttons are highlighted |
| E-stop | Disable simulated actuators; release and Reset before restarting |
| Reset | Restore stock/arms and clear counters |

The final short conveyor advance and inspection-holder spawning changes remain
pending. The panel fits without scrolling; paths and logs open in separate,
non-modal windows. The micro-ROS agent runs in the launch terminal.

The three checkboxes beside E-stop are optional **simulation tests**:

- **Sensor occupied:** force an occupied conveyor output; clear it to release the override.
- **Interrupt command heartbeat:** stop commands to test the watchdog; clear it and Reset afterward.
- **Master/slave:** copy arm 1 in manual mode. Production controls each arm separately.

Leave these unchecked for a normal run.

```bash
bash run.sh sim 0 gui:=false   # Hide Gazebo's display.
bash run.sh sim 0 rviz:=true   # Also show RViz.
bash run.sh qa 4              # Four good toys + two defects; random seed 4.
bash run.sh manual            # Manual simulated arm controls.
bash run.sh shell             # Isolated simulation ROS terminal.
```

Green results mean pass, red reject, amber camera bypass. Bypass is not an
inspection pass. Rejects retain the prepared box. Logs:
`~/ROS2/project2_data/results`.

## Laptop and lab PC

Choose **Physical ROS connection → Connect feedback** in the panel.

| Selection | ESP32 connection | Agent location |
|---|---|---|
| Wi-Fi / laptop ROS | ESP32 Wi-Fi to laptop IPv4 | Laptop; Connect starts it if the UDP port is free |
| USB / lab PC ROS | ESP32 USB plugged into lab PC | Lab PC; start its agent there |

Copy [ROS_Environment](ROS_Environment/README.md) to the lab PC. Follow its setup,
then leave `bash ROS_Environment/run_agent.sh usb` running there.

Start the university arm drivers on the PC connected to the arms. Both computers
need the same hardware ROS domain and network access, even with USB selected.
USB does not carry ROS between computers. Simulation uses a separate domain.

Run `ros2 topic list -t` in the university driver terminal. Confirm command
types, joint order, units, limits and calibration before enabling an adapter.
Enter feedback topics under `hardware.arms` in the main `ID.yaml`.

**The panel currently reads physical telemetry; it does not command the real
arms or merge physical buttons into simulation controls.** ESP32 connection
and arm feedback are displayed separately.

WSL NAT can prevent cross-PC discovery. Use native Ubuntu or suitable
[mirrored networking](https://learn.microsoft.com/en-us/windows/wsl/networking).
An ESP32 UDP relay forwards micro-ROS only, not general ROS joint traffic.

## ESP32 setup

1. Edit `ID.yaml`: `transport: serial` for USB or `transport: wifi`.
   Hardware and simulation domains must differ.
2. For Wi-Fi set SSID/password, reachable laptop `agent_ip` and port 8888.
3. Run `python3 tools/apply_id.py`. Open the whole `firmware/project2_io`
   folder in Arduino IDE. Install `micro_ros_arduino`, select **ESP32 Dev Module**
   and flash. Changing the panel selection does not reflash the board.
4. Close Serial Monitor. Set USB `serial_device` from
   `ls -l /dev/serial/by-id/`; choose the ESP32, not an arm adapter.
5. Native Ubuntu uses USB directly. WSL needs Windows USB forwarding:

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File tools/connect_usb.ps1 -Port COM7
   ```

   Approve binding if prompted. Ubuntu uses `/dev/ttyUSB*`, not COM7.
   Replugging may require reattaching. See [Microsoft's USB guide](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).
6. Test with `bash run.sh doctor`, then
   `bash run.sh hardware io_only:=true`. This reads telemetry without actuator
   commands. Use one agent per serial port/UDP endpoint.

USB firmware was flashed and telemetry received during development. The latest
check found no attached serial device; reconnect/attach before testing.
Keep `io.verified: false` until voltages/polarities, E-stop feedback and conveyor
rate are checked. The E-stop must cut power independently. GPIOs are 3.3V;
GPIO35 needs external bias.

The one-contact selector supports Full/Packaging. In simulation its edges select
and reset a section; its steady position permits GUI changes.

## Camera

The camera connects separately from the main ESP32. Run `bash run.sh camera`.

| Camera | ID.yaml settings |
|---|---|
| Local USB webcam | `camera.source: usb`, correct `camera.device` |
| Lab ROS camera | `camera.source: ros`, correct `camera.topic` |
| ESP32-CAM | `camera.source: url`, `camera.url: http://CAMERA_IP:81/stream` |

ESP32-CAM uses Espressif's CameraWebServer sketch for the correct board.
Confirm the stream in a browser. The viewer shows Waiting, Verifying, Pass/Fail
or No camera connected. Calibrate the arm-3 camera mount and defect views.

Use **Open camera** in the control panel to reopen the feed. The window's X,
Escape, Q or **Close camera** stops the feed and releases the camera. It stays
closed until reopened. An inspection requested while it is closed returns a
failure, even when the no-camera bypass is enabled. `camera_window:=false`
instead keeps inspection running without a preview window.

## Edit movements

Select **Plan a path** after Reset. Click a task in one of the four arm columns.
**▼ Tasks** returns to the list. Right-click to create/delete a task or
overwrite/delete a movement. Delete asks for confirmation. Drag to reorder replay.

**Save coordinate** captures stationary measured joints. **Complete selected
movement/task** replays one/all points. Manual entry is below these buttons.
Replay waits for feedback and checks other-arm clearance; it does not attach
parts or release physical servo torque.

Overwriting a named production target also updates its joint override. New taught
tasks do not replace production dependencies: add automatic actions in the
sequence functions below. Return to Run line and Reset to apply waypoint edits.

| Edit | Location |
|---|---|
| Network, camera, I/O, arm topics | `ID.yaml` |
| Fixture, conveyor, feeders and output poses | `config/production.yaml` |
| Contacts, speed, drift | Same file: `contacts`, `speed_rad_s`, `position_hold` |
| Solved simulation joint positions (40 movements) | `config/solved_joint_paths.yaml` |
| Production points / taught paths | Same file: `joint_overrides`, `path_points`, `teaching_tasks` |
| Carrier/component positions | `config/assembly_run.yaml`: `tray` |
| Automatic actions/interlocks | `scripts/production_run.py`: `assembly`, `conveyor`, `packaging` |
| Grip/approach solving | `project/production_plan.py`: `make_plans` |
| V6 part import and camera reference | `tools/import_v6.py`, then `tools/activate_v6.py` |
| Report/results | `report/report.tex`, `report/results.tex`, trial CSV files |

`solved_joint_paths.yaml` stores the ten solved tasks. Each point has six joint
angles in the listed joint order. Its `missing_paths` list names the remaining
stack transfers. The editor loads these references immediately; changing the CAD
or station configuration invalidates them. They are not hardware calibration.
Regenerate them from the project shell with:

```bash
python3 tools/export_joint_paths.py --products 5 --seconds 120
```

Exit code 2 means some paths still need solving/capture; valid candidates remain
saved. The script never invents coordinates for a failed path.

Positions are metres; angles are radians. Moves require 0.003rad/0.5mm in the
model. Small idle drift is corrected; stale feedback or excessive drift stops
the run. Encoders cannot measure flex or slipping parts.

## Report, sharing and tests

The [four-page ACRA draft](report/report.pdf) separates evidence from pending work.
Enter measured lab results after physical trials.

```bash
python3 tools/package_project.py ~/Downloads/group4_project2_AL.zip
```

The ZIP excludes private settings and build output. The recipient runs the
installer. The source is a diagnostic revision; see the current limits above before running it.

```bash
bash run.sh test            # Unit tests, then all geometry checks.
bash run.sh test assembly   # Unit tests, then assembly geometry.
```

- Controls, selector edges, Reset and product counts.
- Joint limits, taught paths, drift and stale feedback.
- Part contact, gear outside grip, lid/stack clearances.
- Conveyor sensing, rejection and waiting-box reuse.
- Camera classification, connection profiles and installation isolation.

The five-product integration result is saved in
`docs/validation/latest_process.json`, including any blocking reason.
106 unit/UI tests pass. The live Full diagnostic test confirms Start, Pause,
resume and station-local holding while another station continues; its result is
`docs/validation/latest_diagnostic.json`. It is not a completed production cycle.
The full-cycle test does not yet pass. Attachments and prescribed motion do not validate friction, snap-fit force
or 1mm lab accuracy.

To repeat the live diagnostic test, start the simulation without its panel:

```bash
bash run.sh sim 0 panel:=false rviz:=false
```

In another terminal in this project:

```bash
bash run.sh shell
python3 test/integration_process.py --mode full --products 5 --diagnostic
```

The test presses Start/Pause/Start and checks the first blocked station. Omit
`--diagnostic` to require all five finished products instead. Closing the test
panel stops its simulated I/O heartbeat.
