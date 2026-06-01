"""Helpers for bridging libsurvive (pysurvive) objects to ROS messages.

libsurvive uses a quaternion order of (w, x, y, z) and SI units (meters) in a
right-handed world frame.  ROS geometry_msgs use (x, y, z, w).  These helpers
keep all of the format-juggling in one place so the node stays readable.
"""

from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped


def rpy_to_quat(rpy):
    """(roll, pitch, yaw) radians -> (x, y, z, w). Identity for [0,0,0]."""
    from .transforms import quaternion_from_euler
    r, p, y = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
    return tuple(quaternion_from_euler(r, p, y))


def quat_from_param(value):
    """Accept either a 4-element [x,y,z,w] quaternion or a 3-element
    [roll,pitch,yaw] euler (radians) and return a normalised (x,y,z,w) tuple.
    An empty / None value means 'no rotation' -> None."""
    if not value:
        return None
    seq = list(value)
    if len(seq) == 4:
        q = [float(v) for v in seq]
    elif len(seq) == 3:
        from .transforms import quaternion_from_euler
        q = list(quaternion_from_euler(*[float(v) for v in seq]))
    else:
        raise ValueError("rotation must be 4 (quat) or 3 (rpy) numbers, got %r"
                         % (value,))
    import numpy as np
    n = np.linalg.norm(q)
    if n > 0:
        q = [c / n for c in q]
    return tuple(q)


def _is_identity(q):
    return q is None or (abs(q[0]) < 1e-9 and abs(q[1]) < 1e-9
                         and abs(q[2]) < 1e-9 and abs(q[3] - 1.0) < 1e-9)


def transform_pose(position, quat, q_world, q_local):
    """Reproduce vive_tracking_ros's pose mapping.

    q_world : world-frame rotation. Position is rotated by it and orientation is
              PRE-multiplied (q_world * quat) -- aligns the whole frame onto
              'vive_world'. In vive_tracking_ros this is +90 deg about X to turn
              OpenVR's Y-up frame into Z-up; libsurvive is already Z-up so the
              default here is identity (calibrate yaw/origin separately).
    q_local : device body rotation, POST-multiplied (quat * q_local) -- re-orients
              the tracker's local axes. Defaults to the vive_tracking_ros tracker
              constant. Position is unaffected. Pass None to skip.
    """
    import numpy as np
    from .transforms import quaternion_matrix, quaternion_multiply

    if _is_identity(q_world):
        pos = tuple(float(c) for c in position)
        q = list(quat)
    else:
        rot = quaternion_matrix(q_world)[:3, :3]
        pos = tuple(rot.dot(np.asarray(position, dtype=float)))
        q = quaternion_multiply(q_world, quat)

    if not _is_identity(q_local):
        q = quaternion_multiply(q, q_local)

    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n > 0:
        q = q / n
    return pos, tuple(q)


def _as_str(value):
    """libsurvive returns names/serials as bytes; normalise to str."""
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return str(value)


def object_name(obj):
    """Codename libsurvive assigns to a device, e.g. 'T20', 'LH0', 'WM0'."""
    return _as_str(obj.Name())


def object_serial(obj):
    """Serial number (e.g. 'LHR-CBB722E9'), or None.

    The simple pysurvive SimpleObject only exposes Name()/Pose(), but the C
    function survive_simple_serial_number() is available in the generated
    bindings and takes the object pointer (SimpleObject.ptr). Use it so we can
    match devices on the same LHR-/LHB- serials the SteamVR config uses. Fall
    back to a Serial() method if a future build adds one.
    """
    try:
        import pysurvive
        fn = getattr(pysurvive, "survive_simple_serial_number", None)
        ptr = getattr(obj, "ptr", None)
        if fn is not None and ptr:
            serial = fn(ptr)
            if serial:
                return _as_str(serial)
    except Exception:
        pass
    getter = getattr(obj, "Serial", None)
    if getter is not None:
        try:
            serial = getter()
            if serial:
                return _as_str(serial)
        except Exception:
            pass
    return None


def is_lighthouse(obj):
    """True if the object is a base station rather than a tracked device."""
    # Prefer the typed API when available.
    type_getter = getattr(obj, "ObjectType", None)
    if type_getter is not None:
        try:
            import pysurvive
            lh_enum = getattr(pysurvive, "SurviveSimpleObject_LIGHTHOUSE", None)
            if lh_enum is not None:
                return type_getter() == lh_enum
        except Exception:
            pass
    # Fall back to the naming convention libsurvive uses for base stations.
    name = object_name(obj).upper()
    return name.startswith("LH") or name.startswith("LHB")


def read_pose(obj):
    """Return ((x, y, z), (qx, qy, qz, qw)) in ROS order, or None if no fix yet.

    pysurvive's SimpleObject.Pose() returns either a SurvivePose or a
    (SurvivePose, timecode) tuple depending on the build; handle both.
    """
    try:
        raw = obj.Pose()
    except Exception:
        return None
    if raw is None:
        return None
    pose = raw[0] if isinstance(raw, tuple) else raw
    try:
        pos = pose.Pos
        rot = pose.Rot  # (w, x, y, z)
    except Exception:
        return None

    position = (float(pos[0]), float(pos[1]), float(pos[2]))
    # libsurvive (w, x, y, z) -> ROS (x, y, z, w)
    quat = (float(rot[1]), float(rot[2]), float(rot[3]), float(rot[0]))

    # An object that has never been seen reports an all-zero pose; treat the
    # degenerate (zero quaternion) case as "no valid fix yet".
    if quat == (0.0, 0.0, 0.0, 0.0):
        return None
    return position, quat


def read_velocity(obj):
    """Return ((vx, vy, vz), (wx, wy, wz)) in the libsurvive world frame, or None.

    Uses the C function survive_simple_object_get_latest_velocity(sao, *vel).
    libsurvive reports angular velocity as an axis-angle rate (rad/s), which maps
    directly onto Twist.angular.
    """
    try:
        import ctypes
        import pysurvive
        fn = getattr(pysurvive, "survive_simple_object_get_latest_velocity", None)
        ptr = getattr(obj, "ptr", None)
        if fn is None or not ptr:
            return None
        vel = pysurvive.SurviveVelocity()
        fn(ptr, ctypes.byref(vel))
        lin = (float(vel.Pos[0]), float(vel.Pos[1]), float(vel.Pos[2]))
        ang = (float(vel.AxisAngleRot[0]), float(vel.AxisAngleRot[1]),
               float(vel.AxisAngleRot[2]))
        return lin, ang
    except Exception:
        return None


def body_twist(linear, angular, q_world, position, quat):
    """Reproduce vive_tracking_ros's twist mapping.

    The velocity comes from libsurvive in the world frame. vive_tracking_ros:
      1) rotates linear & angular by the world rotation (q_world),
      2) stacks the spatial twist V_s = [linear; angular],
      3) converts it to the BODY (tracker) frame with
         V_b = Adjoint(TransInv(T)) @ V_s, where T is the device's transformed
         pose, so the published twist is expressed in the tracker's own frame.
    Returns ((vx,vy,vz), (wx,wy,wz)) in the body frame.
    """
    import numpy as np
    import modern_robotics as mr
    from .transforms import quaternion_matrix

    lin = np.asarray(linear, dtype=float)
    ang = np.asarray(angular, dtype=float)
    if not _is_identity(q_world):
        rot = quaternion_matrix(q_world)[:3, :3]
        lin = rot.dot(lin)
        ang = rot.dot(ang)

    v_s = np.append(lin, ang)                 # [v; w] -- same order as vive_tracking_ros
    T = quaternion_matrix(quat)
    T[0:3, 3] = position
    v_b = mr.Adjoint(mr.TransInv(T)) @ v_s
    return tuple(v_b[:3]), tuple(v_b[3:])


class _EmaVec(object):
    """Exponential-moving-average low-pass for a vector.
    alpha in (0,1): smaller = smoother (more lag). alpha>=1 = passthrough."""
    def __init__(self, alpha):
        self.alpha = float(alpha)
        self.state = None

    def update(self, v):
        import numpy as np
        v = np.asarray(v, dtype=float)
        if self.state is None or self.alpha >= 1.0:
            self.state = v
        else:
            self.state = self.alpha * v + (1.0 - self.alpha) * self.state
        return tuple(self.state)


class _QuatSlerp(object):
    """SLERP-based low-pass for a quaternion (x,y,z,w).
    fraction in (0,1): smaller = smoother. fraction>=1 = passthrough."""
    def __init__(self, fraction):
        self.fraction = float(fraction)
        self.last = None

    def update(self, q):
        import numpy as np
        from .transforms import quaternion_slerp
        q = np.asarray(q, dtype=float)
        if self.last is None or self.fraction >= 1.0:
            self.last = q
            return tuple(q)
        if float(np.dot(self.last, q)) < 0.0:   # shortest-path
            q = -q
        out = quaternion_slerp(self.last, q, self.fraction)
        self.last = np.asarray(out, dtype=float)
        return tuple(out)


class PoseLowPass(object):
    """Optional low-pass for a pose: EMA on position, SLERP on orientation.
    alpha<=0 disables (returns input unchanged)."""
    def __init__(self, alpha):
        self.enabled = float(alpha) > 0.0
        if self.enabled:
            self._pos = _EmaVec(alpha)
            self._quat = _QuatSlerp(alpha)

    def update(self, position, quat):
        if not self.enabled:
            return position, quat
        return self._pos.update(position), self._quat.update(quat)


class TwistLowPass(object):
    """Optional EMA low-pass for a twist (linear, angular). alpha<=0 disables."""
    def __init__(self, alpha):
        self.enabled = float(alpha) > 0.0
        if self.enabled:
            self._lin = _EmaVec(alpha)
            self._ang = _EmaVec(alpha)

    def update(self, linear, angular):
        if not self.enabled:
            return linear, angular
        return self._lin.update(linear), self._ang.update(angular)


def make_pose_stamped(stamp, frame_id, position, quat):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = quat
    return msg


def make_transform(stamp, parent_frame, child_frame, position, quat):
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent_frame
    tf.child_frame_id = child_frame
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = position
    (tf.transform.rotation.x, tf.transform.rotation.y,
     tf.transform.rotation.z, tf.transform.rotation.w) = quat
    return tf


def make_twist_stamped(stamp, frame_id, linear, angular):
    msg = TwistStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = linear
    msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular
    return msg
