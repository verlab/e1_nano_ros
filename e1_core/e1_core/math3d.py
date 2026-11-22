"""Small quaternion helpers (w, x, y, z), body->world rotations, FLU / ENU."""
import math
import numpy as np


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qrot(q, v):
    """Rotate vector v by quaternion q (body->world when q is the attitude)."""
    p = np.concatenate([[0.0], v])
    return qmul(qmul(q, p), qconj(q))[1:]


def qrot_inv(q, v):
    """Rotate world vector into the body frame."""
    return qrot(qconj(q), v)


def from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])


def to_euler(q):
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rotation_vector_between(a, b):
    """Rotation vector that rotates a onto b (same as CF-Drone Vector::rotationVectorBetween)."""
    an, bn = np.linalg.norm(a), np.linalg.norm(b)
    if an < 1e-6 or bn < 1e-6:
        return np.zeros(3)
    d = np.cross(a, b)
    dn = np.linalg.norm(d)
    if dn < 1e-6:
        if np.dot(a, b) > 0:
            return np.zeros(3)
        perp = np.cross(a, [1.0, 0, 0])
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(a, [0, 1.0, 0])
        return perp / np.linalg.norm(perp) * math.pi
    ang = math.atan2(dn, np.dot(a, b))
    return d / dn * ang


def quat_from_thrust_dir(z_world, yaw):
    """Attitude whose body z axis is z_world (unit) with the given heading."""
    z = np.asarray(z_world, float)
    z = z / np.linalg.norm(z)
    xc = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    y = np.cross(z, xc)
    if np.linalg.norm(y) < 1e-6:
        y = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    R = np.column_stack([x, y, z])
    return mat_to_quat(R)


def mat_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q / np.linalg.norm(q) * (1 if q[0] >= 0 else -1)
