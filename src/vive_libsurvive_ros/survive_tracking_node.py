#!/usr/bin/env python3
"""Steam-free VIVE tracking node backed by libsurvive (pysurvive) -- ROS2 port.

Discovers every device libsurvive can see (trackers connected via dongle *or*
direct USB, plus the SteamVR 2.0 base stations), maps each one to a friendly
frame name via a YAML config, and republishes its pose as:

  * geometry_msgs/PoseStamped on  ~/<name>/pose   (or <ns>/<name>/pose)
  * a TF from <world_frame> to <name>
  * geometry_msgs/TwistStamped on ~/<name>/twist  (if the pysurvive build
    exposes velocity)

No SteamVR / OpenVR / Steam runtime is required.

ROS2 note: the rich ``devices`` config (a list of maps) cannot be expressed as
native ROS2 parameters, so the node takes a single ``config`` string parameter
pointing at the YAML file and loads it itself with PyYAML -- the same semantics
as Noetic's ``rosparam load file``. The YAML keys are unchanged.
"""

import sys
import time

import rclpy
from rclpy.node import Node
import tf2_ros
import yaml

from vive_libsurvive_ros import survive_utils as su

try:
    import pysurvive
except ImportError:
    pysurvive = None


class SurviveTrackingNode(Node):
    def __init__(self):
        super().__init__("vive_libsurvive_ros")

        if pysurvive is None:
            self.get_logger().fatal(
                "pysurvive not found. Install libsurvive's python bindings, e.g.\n"
                "  ros2 run vive_libsurvive_ros install_libsurvive.sh\n"
                "or  pip3 install pysurvive")
            raise ImportError("pysurvive")

        # Single 'config' parameter = path to the YAML file (Noetic rosparam-load
        # equivalent). All settings are read from it with the original defaults.
        config_path = self.declare_parameter("config", "").value
        cfg = {}
        if config_path:
            with open(config_path, "r") as f:
                cfg = yaml.safe_load(f) or {}
            self.get_logger().info("loaded config: %s" % config_path)
        else:
            self.get_logger().warn("no 'config' parameter set; using defaults")
        self._cfg = cfg

        self.world_frame = cfg.get("world_frame", "libsurvive_world")
        self.publish_tf = bool(cfg.get("publish_tf", True))
        self.publish_twist = bool(cfg.get("publish_twist", True))
        self.publish_lighthouses = bool(cfg.get("publish_lighthouses", True))
        self.rate_hz = float(cfg.get("publish_rate", 120.0))

        # --- vive_tracking_ros compatibility options ---------------------
        self.topic_namespace = cfg.get("topic_namespace", "")
        self.dash_to_underscore = bool(cfg.get("dash_to_underscore", False))
        world_rot = cfg.get("world_rotation",
                            cfg.get("world_rotation_rpy", [0.0, 0.0, 0.0]))
        self.q_world = su.quat_from_param(world_rot)
        self.apply_tracker_local = bool(cfg.get("apply_tracker_local", True))
        self.q_tracker_local = su.quat_from_param(
            cfg.get("tracker_local_rotation",
                    [-0.70710454, 0.70710902, 0.0, 0.0]))

        # Optional low-pass filtering of the published tracker pose/twist. 0 = off.
        self.pose_filter_alpha = float(cfg.get("pose_filter_alpha", 0.0))
        self.twist_filter_alpha = float(cfg.get("twist_filter_alpha", 0.0))
        self._pose_filters = {}
        self._twist_filters = {}

        survive_args = cfg.get("survive_args", [])
        if isinstance(survive_args, str):
            survive_args = survive_args.split()

        # devices: list of {serial, name, type}. Matched by serial (preferred)
        # or libsurvive codename.
        self.devices = cfg.get("devices", [])
        self._build_lookup()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # publishers + resolved frame name, created lazily as objects appear
        self._pose_pubs = {}
        self._twist_pubs = {}
        self._resolved = {}        # libsurvive codename -> friendly frame name
        self._frames_in_use = {}   # friendly frame name -> codename that owns it
        self._warned_unmapped = set()

        self.get_logger().info("Starting libsurvive context (Steam-free)...")
        argv = ["survive_tracking_node"] + list(survive_args)
        self.ctx = pysurvive.SimpleContext(argv)

        # libsurvive registers devices asynchronously, so rescan periodically.
        self._scan_objects()

    # ------------------------------------------------------------------ setup
    def _build_lookup(self):
        """Index config entries by libsurvive codename (and serial, best-effort)."""
        self.by_survive_name = {}
        self.by_serial = {}
        for dev in self.devices:
            friendly = dev.get("name")
            if not friendly:
                continue
            sname = dev.get("survive_name")
            if sname:
                self.by_survive_name[sname.upper()] = friendly
            serial = dev.get("serial")
            if serial:
                self.by_serial[serial.upper()] = friendly

    def _resolve(self, obj):
        """Map a libsurvive object to a configured frame name (or its codename)."""
        codename = su.object_name(obj)
        serial = su.object_serial(obj)

        # 1) match on SERIAL -- stable and unique.
        if serial:
            s = serial.upper()
            if s in self.by_serial:
                return self.by_serial[s]
            for cfg_serial, friendly in self.by_serial.items():
                if cfg_serial in s:
                    return friendly
        # 2) fall back to codename ONLY when no serial was read.
        if codename.upper() in self.by_survive_name:
            return self.by_survive_name[codename.upper()]
        # 3) fall back to the raw codename so the device is still published
        if codename.upper() not in self._warned_unmapped:
            self.get_logger().warn(
                "Device '%s' (serial=%s) not in config; publishing under its "
                "libsurvive codename." % (codename, serial))
            self._warned_unmapped.add(codename.upper())
        return codename

    def _scan_objects(self):
        """Register any devices libsurvive has added since the last scan."""
        try:
            curr = pysurvive.simple_get_first_object(self.ctx.ptr)
            while curr:
                obj = pysurvive.SimpleObject(curr)
                self._register(obj)
                # Lighthouses are static, so publish their pose directly per scan.
                if su.is_lighthouse(obj) and su.object_name(obj) in self._resolved:
                    self._publish(obj)
                curr = pysurvive.simple_get_next_object(self.ctx.ptr, curr)
        except Exception as exc:
            self.get_logger().warn("object scan failed: %s" % exc,
                                   throttle_duration_sec=10.0)

    def _register(self, obj):
        codename = su.object_name(obj)
        if codename in self._resolved:
            return
        if su.is_lighthouse(obj) and not self.publish_lighthouses:
            return

        frame = self._resolve(obj)
        if self.dash_to_underscore:
            frame = frame.replace("-", "_")

        # Guard against two devices mapping to the same frame.
        owner = self._frames_in_use.get(frame)
        if owner is not None and owner != codename:
            self.get_logger().warn(
                "Frame '%s' already owned by '%s'; device '%s' (serial=%s) maps "
                "to the same name and will be published under '%s_%s' instead. "
                "Fix the device serial/connection to disambiguate."
                % (frame, owner, codename, su.object_serial(obj), frame, codename))
            frame = "%s_%s" % (frame, codename)
        self._frames_in_use[frame] = codename
        self._resolved[codename] = frame

        pose_topic = self._topic(frame, "pose")
        self._pose_pubs[codename] = self.create_publisher(
            su.PoseStamped, pose_topic, 10)
        if self.publish_twist:
            self._twist_pubs[codename] = self.create_publisher(
                su.TwistStamped, self._topic(frame, "twist"), 10)
        kind = "lighthouse" if su.is_lighthouse(obj) else "tracker"
        self.get_logger().info(
            "Registered %s '%s' -> frame '%s' (serial=%s) topic=%s"
            % (kind, codename, frame, su.object_serial(obj), pose_topic))

    def _topic(self, frame, suffix):
        """Build the topic name. With topic_namespace set, use an absolute
        <ns>/<frame>/<suffix>. Otherwise publish privately under ~/<frame>/<suffix>."""
        if self.topic_namespace:
            return "%s/%s/%s" % (self.topic_namespace.rstrip("/"), frame, suffix)
        return "~/%s/%s" % (frame, suffix)

    # -------------------------------------------------------------------- run
    def _publish(self, obj):
        codename = su.object_name(obj)
        if codename not in self._resolved:
            self._register(obj)
            if codename not in self._resolved:
                return
        frame = self._resolved[codename]

        pose = su.read_pose(obj)
        if pose is None:
            return
        position, quat = pose
        # Reproduce vive_tracking_ros: world rotation on every device, plus the
        # tracker body rotation on trackers only (not lighthouses).
        q_local = self.q_tracker_local
        if not self.apply_tracker_local or su.is_lighthouse(obj):
            q_local = None
        position, quat = su.transform_pose(position, quat, self.q_world, q_local)

        # Optional pose low-pass (trackers only; lighthouses are static).
        if self.pose_filter_alpha > 0.0 and not su.is_lighthouse(obj):
            pf = self._pose_filters.get(codename)
            if pf is None:
                pf = self._pose_filters[codename] = su.PoseLowPass(self.pose_filter_alpha)
            position, quat = pf.update(position, quat)

        stamp = self.get_clock().now().to_msg()

        self._pose_pubs[codename].publish(
            su.make_pose_stamped(stamp, self.world_frame, position, quat))

        if self.publish_tf:
            self.tf_broadcaster.sendTransform(
                su.make_transform(stamp, self.world_frame, frame, position, quat))

        if (self.publish_twist and codename in self._twist_pubs
                and not su.is_lighthouse(obj)):
            vel = su.read_velocity(obj)
            if vel is not None:
                linear, angular = vel
                # Convert the world-frame velocity to the tracker's body frame.
                linear, angular = su.body_twist(
                    linear, angular, self.q_world, position, quat)
                if self.twist_filter_alpha > 0.0:
                    tf_ = self._twist_filters.get(codename)
                    if tf_ is None:
                        tf_ = self._twist_filters[codename] = su.TwistLowPass(self.twist_filter_alpha)
                    linear, angular = tf_.update(linear, angular)
                self._twist_pubs[codename].publish(
                    su.make_twist_stamped(stamp, frame, linear, angular))

    def spin(self):
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.0
        next_scan = time.monotonic()
        while rclpy.ok() and self.ctx.Running():
            # Periodically pick up devices that appeared (or aren't tracking yet).
            now = time.monotonic()
            if now >= next_scan:
                self._scan_objects()
                next_scan = now + 1.0

            # Drain ALL pending updates each wake (bounded for safety).
            drained = 0
            updated = self.ctx.NextUpdated()
            while updated is not None and drained < 256:
                try:
                    self._publish(updated)
                except Exception as exc:  # never let one bad frame kill the node
                    self.get_logger().warn("publish error: %s" % exc,
                                           throttle_duration_sec=5.0)
                drained += 1
                updated = self.ctx.NextUpdated()
            if drained == 0 and period > 0.0:
                # Nothing pending; yield so we don't busy-spin a core.
                time.sleep(period)
        self.get_logger().info("libsurvive context stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SurviveTrackingNode()
    except Exception as exc:
        print("failed to start libsurvive node: %s" % exc, file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
