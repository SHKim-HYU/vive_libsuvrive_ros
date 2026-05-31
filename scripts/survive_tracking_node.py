#!/usr/bin/env python3
"""Steam-free VIVE tracking node backed by libsurvive (pysurvive).

Discovers every device libsurvive can see (trackers connected via dongle *or*
direct USB, plus the SteamVR 2.0 base stations), maps each one to a friendly
frame name via a YAML config, and republishes its pose as:

  * geometry_msgs/PoseStamped on  ~<name>/pose
  * a TF from <world_frame> to <name>
  * geometry_msgs/TwistStamped on ~<name>/twist   (if the pysurvive build
    exposes velocity)

No SteamVR / OpenVR / Steam runtime is required.
"""

import sys

import rospy
import tf2_ros

from vive_libsurvive_ros import survive_utils as su

try:
    import pysurvive
except ImportError:
    rospy.logfatal(
        "pysurvive not found. Install libsurvive's python bindings, e.g.\n"
        "  rosrun vive_libsurvive_ros install_libsurvive.sh\n"
        "or  pip3 install pysurvive")
    raise


class SurviveTrackingNode(object):
    def __init__(self):
        rospy.init_node("vive_libsurvive_ros")

        self.world_frame = rospy.get_param("~world_frame", "libsurvive_world")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.publish_twist = rospy.get_param("~publish_twist", True)
        self.publish_lighthouses = rospy.get_param("~publish_lighthouses", True)
        self.rate_hz = float(rospy.get_param("~publish_rate", 120.0))

        # --- vive_tracking_ros compatibility options ---------------------
        # If set (e.g. "/vive"), poses publish to <ns>/<name>/pose (absolute)
        # instead of ~<name>/pose, matching the SteamVR package's topic names.
        self.topic_namespace = rospy.get_param("~topic_namespace", "")
        # Replace '-' with '_' in frame/topic names (vive_tracking_ros does this).
        self.dash_to_underscore = rospy.get_param("~dash_to_underscore", False)
        # Pose transform reproducing vive_tracking_ros (see survive_utils):
        #  * world rotation: [r,p,y] (3) or [x,y,z,w] (4). Rotates position and
        #    pre-multiplies orientation. vive_tracking_ros uses +90deg X to make
        #    OpenVR Y-up into Z-up; libsurvive is already Z-up so default is
        #    identity -- set a yaw here to align heading with vive_world.
        world_rot = rospy.get_param("~world_rotation",
                                    rospy.get_param("~world_rotation_rpy",
                                                    [0.0, 0.0, 0.0]))
        self.q_world = su.quat_from_param(world_rot)
        #  * tracker local rotation: post-multiplied onto each tracker's
        #    orientation to match vive_tracking_ros's body-axis convention.
        #    Default = its exact constant; set [] to disable.
        self.apply_tracker_local = rospy.get_param("~apply_tracker_local", True)
        self.q_tracker_local = su.quat_from_param(
            rospy.get_param("~tracker_local_rotation",
                            [-0.70710454, 0.70710902, 0.0, 0.0]))

        # Optional low-pass filtering of the published tracker pose/twist. 0 = off
        # (libsurvive output is already Kalman-smoothed). A value in (0,1) is the
        # EMA/SLERP factor: smaller = smoother but more lag. Applied to trackers
        # only (not lighthouses). State is kept per device.
        self.pose_filter_alpha = float(rospy.get_param("~pose_filter_alpha", 0.0))
        self.twist_filter_alpha = float(rospy.get_param("~twist_filter_alpha", 0.0))
        self._pose_filters = {}
        self._twist_filters = {}

        # NOTE: this package is the BACKEND/driver only. It publishes each device's
        # pose/twist/TF in its own world frame (self.world_frame) with the axis/
        # body conventions above. It does NOT define an application "vive_world"
        # or anchor to a reference device -- that belongs to the application layer
        # (the hyumm vive_world anchor package).

        # Extra command line arguments forwarded verbatim to libsurvive, e.g.
        # ["--lighthouse-gen", "2", "-c", "/home/robot/.config/libsurvive/config.json"].
        survive_args = rospy.get_param("~survive_args", [])
        if isinstance(survive_args, str):
            survive_args = survive_args.split()

        # devices: list of {serial, name, type}. We match libsurvive objects to
        # these entries by serial number (preferred) or codename.
        self.devices = rospy.get_param("~devices", [])
        self._build_lookup()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # publishers + resolved frame name, created lazily as objects appear
        self._pose_pubs = {}
        self._twist_pubs = {}
        self._resolved = {}   # libsurvive codename -> friendly frame name
        self._frames_in_use = {}   # friendly frame name -> codename that owns it
        self._warned_unmapped = set()

        rospy.loginfo("Starting libsurvive context (Steam-free)...")
        argv = ["survive_tracking_node"] + list(survive_args)
        self.ctx = pysurvive.SimpleContext(argv)

        # libsurvive registers devices asynchronously a second or two after
        # init (and a device that is not yet tracking never shows up via
        # NextUpdated), so we rescan Objects() periodically rather than once.
        self._scan_objects()

    # ------------------------------------------------------------------ setup
    def _build_lookup(self):
        """Index config entries by libsurvive codename (and serial, best-effort).

        With the simple pysurvive API, an object only exposes its codename
        (e.g. 'T20', 'KN0', 'WM0', 'LH0'), not its serial -- libsurvive assigns
        those codenames deterministically per device and persists the mapping in
        ~/.config/libsurvive/config.json, so they are stable across runs. Match
        primarily on 'survive_name'; 'serial' is kept only as an optional hint.
        """
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

        # 1) match on SERIAL -- stable and unique. libsurvive codenames
        #    (T20, T21, WM0, ...) are assigned by enumeration order and CHANGE
        #    between runs / when connections change, so serial is the reliable
        #    key whenever the device exposes one.
        if serial:
            s = serial.upper()
            if s in self.by_serial:
                return self.by_serial[s]
            for cfg_serial, friendly in self.by_serial.items():
                if cfg_serial in s:
                    return friendly
        # 2) fall back to codename ONLY when no serial was read (e.g. a wireless
        #    device whose config blob hasn't fully loaded). Less reliable because
        #    codenames are not stable -- fix the link so the serial reads.
        if codename.upper() in self.by_survive_name:
            return self.by_survive_name[codename.upper()]
        # 3) fall back to the raw codename so the device is still published
        if codename.upper() not in self._warned_unmapped:
            rospy.logwarn("Device '%s' (serial=%s) not in config; publishing "
                          "under its libsurvive codename.", codename, serial)
            self._warned_unmapped.add(codename.upper())
        return codename

    def _scan_objects(self):
        """Register any devices libsurvive has added since the last scan.

        SimpleContext.Objects() only returns the list captured at construction
        time, so devices libsurvive adds a second or two later never show up
        there. Walk the live linked list directly via the C API instead.
        """
        try:
            curr = pysurvive.simple_get_first_object(self.ctx.ptr)
            while curr:
                obj = pysurvive.SimpleObject(curr)
                self._register(obj)
                # Lighthouses are static, so they almost never arrive via
                # NextUpdated(); publish their pose directly on each scan tick.
                if su.is_lighthouse(obj) and su.object_name(obj) in self._resolved:
                    self._publish(obj)
                curr = pysurvive.simple_get_next_object(self.ctx.ptr, curr)
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "object scan failed: %s", exc)

    def _register(self, obj):
        codename = su.object_name(obj)
        if codename in self._resolved:
            return
        if su.is_lighthouse(obj) and not self.publish_lighthouses:
            return

        frame = self._resolve(obj)
        if self.dash_to_underscore:
            frame = frame.replace("-", "_")

        # Guard against two devices mapping to the same frame (e.g. an unstable
        # codename fallback) -- they would otherwise clobber the same topic.
        owner = self._frames_in_use.get(frame)
        if owner is not None and owner != codename:
            rospy.logwarn("Frame '%s' already owned by '%s'; device '%s' "
                          "(serial=%s) maps to the same name and will be "
                          "published under '%s_%s' instead. Fix the device "
                          "serial/connection to disambiguate.",
                          frame, owner, codename, su.object_serial(obj),
                          frame, codename)
            frame = "%s_%s" % (frame, codename)
        self._frames_in_use[frame] = codename
        self._resolved[codename] = frame

        pose_topic = self._topic(frame, "pose")
        self._pose_pubs[codename] = rospy.Publisher(
            pose_topic, su.PoseStamped, queue_size=10)
        if self.publish_twist:
            self._twist_pubs[codename] = rospy.Publisher(
                self._topic(frame, "twist"), su.TwistStamped, queue_size=10)
        kind = "lighthouse" if su.is_lighthouse(obj) else "tracker"
        rospy.loginfo("Registered %s '%s' -> frame '%s' (serial=%s) topic=%s",
                      kind, codename, frame, su.object_serial(obj), pose_topic)

    def _topic(self, frame, suffix):
        """Build the topic name. With topic_namespace set, use an absolute
        <ns>/<frame>/<suffix> (matches vive_tracking_ros's /vive/...). Otherwise
        publish privately under ~<frame>/<suffix>."""
        if self.topic_namespace:
            return "%s/%s/%s" % (self.topic_namespace.rstrip("/"), frame, suffix)
        return "~%s/%s" % (frame, suffix)

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

        stamp = rospy.Time.now()

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
                # Convert the world-frame velocity to the tracker's body frame
                # (Adjoint), exactly like vive_tracking_ros.
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
        rate = rospy.Rate(self.rate_hz)
        next_scan = rospy.Time.now()
        while not rospy.is_shutdown() and self.ctx.Running():
            # Periodically pick up devices that appeared (or aren't tracking yet).
            now = rospy.Time.now()
            if now >= next_scan:
                self._scan_objects()
                next_scan = now + rospy.Duration(1.0)

            # Drain ALL pending updates each wake. NextUpdated() returns at most
            # one object per call in fixed registration order and clears its
            # update flag, so servicing only one per loop favours earlier-
            # registered devices and skews per-tracker rates when 2-3 trackers
            # update between wakes. Loop until drained (bounded for safety).
            drained = 0
            updated = self.ctx.NextUpdated()
            while updated is not None and drained < 256:
                try:
                    self._publish(updated)
                except Exception as exc:  # never let one bad frame kill the node
                    rospy.logwarn_throttle(5.0, "publish error: %s", exc)
                drained += 1
                updated = self.ctx.NextUpdated()
            if drained == 0:
                # Nothing pending; yield so we don't busy-spin a core.
                rate.sleep()
        rospy.loginfo("libsurvive context stopped.")


def main():
    try:
        node = SurviveTrackingNode()
    except Exception as exc:
        rospy.logfatal("failed to start libsurvive node: %s", exc)
        sys.exit(1)
    node.spin()


if __name__ == "__main__":
    main()
