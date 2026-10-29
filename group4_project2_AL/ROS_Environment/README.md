# Lab PC ROS environment

This folder can be copied on its own to the lab PC. The university's existing
driver workspace continues to control the arms. The laptop runs the control
panel and simulation. USB is the ESP32-to-PC connection; the two PCs still
communicate over the same network.

## USB on the lab PC

1. Install ROS 2 Humble on Ubuntu 22.04. Keep the university driver installation.
2. Copy `ID.example.yaml` to `ID.yaml` here. Set `ros_domain_id` to the same
   hardware domain as the laptop's `group4_project2_AL/ID.yaml`.
3. Plug the ESP32 directly into the lab PC with a data cable. Flash the project's
   USB firmware first. Close Serial Monitor.
4. Run `ls -l /dev/serial/by-id/`. Put the ESP32's full path in `serial_device`.
   With multiple arm adapters connected, automatic serial selection will refuse
   to guess. If permission is denied, run `sudo usermod -aG dialout "$USER"`,
   then log out and back in.
5. If the agent is not installed, run `bash install_agent.sh` once.
6. Run `bash run_agent.sh usb` and leave that terminal open.
7. Start the university arm drivers in their usual sourced terminal, with the
   same `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY=0` set before launch.
8. On the laptop, run the simulation and choose **USB / lab PC ROS → Connect
   feedback**. The panel shows ESP32 and arm feedback separately.

## Wi-Fi on the laptop

1. Set the ESP32 firmware to Wi-Fi, using the laptop agent's reachable IPv4
   address, port 8888 and the hardware ROS domain in the main `ID.yaml`.
2. On the laptop run `bash ROS_Environment/run_agent.sh wifi` from the project
   folder. Choose **Wi-Fi / laptop ROS → Connect feedback** in the panel.
3. If the motors remain attached to the lab PC, its drivers still run there;
   they must be reachable on the same ROS domain and network.

Changing the panel selection does not flash the ESP32 or start a program on a
different computer. It selects and records the intended connection arrangement.
Start the matching agent once on the named host. Do not run two agents on the
same ESP32 serial port or UDP endpoint.

## Check before enabling the arms

In the lab driver terminal, run `ros2 topic list -t` and
`ros2 topic echo /YOUR_ARM_JOINT_STATE_TOPIC --once`. Record the command message
types, joint order, units, calibration and limits. Set the actual feedback topics
in the laptop's main `ID.yaml`. The panel connection is currently read-only:
it does not assume that simulation trajectory messages match the lab drivers.

Both computers need ROS network discovery. Use native Ubuntu or a suitably
configured WSL mirrored network. An ESP32 UDP relay forwards micro-ROS packets
only; it does not forward the complete ROS graph. Check `ros2 topic list` on
both machines before troubleshooting the arm code.

The USB camera connects separately to a computer. Plugging in the ESP32 does
not connect the rig camera. Select the camera device, ROS image topic or network
stream in the main `ID.yaml`.
