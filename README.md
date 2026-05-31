# vive_libsurvive_ros

Steam-free HTC **VIVE Tracker / Lighthouse** tracking for **ROS1 Noetic** using
[libsurvive](https://github.com/collabora/libsurvive). No SteamVR, no OpenVR, no
Steam runtime — and **no HMD required**.

Target setup:

- 3x VIVE Tracker (2 via wireless dongle, 1 via direct USB / "serial" connection)
- 2x SteamVR **2.0** base stations
- Ubuntu 20.04 + ROS Noetic

## What it publishes

For every device found (matched to a friendly name by serial in
`config/trackers.yaml`):

| Topic | Type | Notes |
|-------|------|-------|
| `~<name>/pose`  | `geometry_msgs/PoseStamped` | in `world_frame` |
| `~<name>/twist` | `geometry_msgs/TwistStamped` | if pysurvive exposes velocity |
| TF `world_frame -> <name>` | tf2 | when `publish_tf: true` |

Quaternions are converted from libsurvive `(w,x,y,z)` to ROS `(x,y,z,w)`; units
are meters / radians-per-second in a right-handed frame.

## Install

```bash
# 1) build libsurvive + pysurvive + install udev rules (one time)
rosrun vive_libsurvive_ros install_libsurvive.sh   # or run scripts/install_libsurvive.sh

# 2) build the catkin package
cd ~/ros_ws/catkin_ws
catkin build vive_libsurvive_ros   # or: catkin_make
source devel/setup.bash
```

If `install_libsurvive.sh` cannot install `pysurvive`, build it manually from the
cloned `~/libsurvive` tree (`pip3 install --user .`) — see the libsurvive README.

## Bring-up

```bash
# verify libsurvive sees the trackers + both base stations (prints serials)
survive-cli

# one-time room calibration (writes ~/.config/libsurvive/config.json)
#   keep a tracker still and visible to BOTH base stations until it converges
survive-cli

# run the node
roslaunch vive_libsurvive_ros libsurvive_tracking.launch
```

Check it:

```bash
rostopic list | grep vive_libsurvive
rostopic echo /vive_libsurvive_ros/tracker_mobile_front/pose
rosrun tf tf_echo libsurvive_world tracker_fixed
rosrun rviz rviz   # add TF display, fixed frame = libsurvive_world
```

## Configuration (`config/trackers.yaml`)

- `devices`: maps each device **serial** to a `name` (TF frame + topic namespace).
  The serials shipped here come from the existing SteamVR config; run `survive-cli`
  to confirm the serials libsurvive actually reports and edit as needed. Remember
  to set the **2nd base station serial** (`lighthouse_1`).
- `world_frame` (default `libsurvive_world`), `publish_tf`, `publish_twist`,
  `publish_lighthouses`, `publish_rate`.
- `survive_args`: extra CLI args forwarded to libsurvive. Defaults force gen-2
  base stations and pin a persistent calibration file:
  `--lighthouse-gen 2 -c /home/robot/.config/libsurvive/config.json`.

## Notes & gotchas

- **Direct-USB ("serial") tracker**: libsurvive auto-detects it like any other
  device — no separate driver. It only needs the udev rule for product `28de:2300`
  (already in `udev/83-vive-libsurvive.rules`).
- **Base stations 2.0** are RF-synced; set distinct channels (1..16) on each. With
  libsurvive there is **no 4-lighthouse SteamVR cap** — you can add more later.
- **World origin** is established by calibration; it is stable across restarts only
  while you keep the same `config.json`. Move the base stations → recalibrate.
- **No HMD**: nothing here needs one. (The old `vive_tracking_ros` package in this
  workspace uses SteamVR/OpenVR and a `null` HMD driver — this package replaces that
  path entirely.)
- If `survive-cli` works but ROS can't open the device, you almost always still
  need to replug after installing the udev rules.
```
