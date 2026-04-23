import numpy as np


def _quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


def _quat_norm(q):
    norm = np.linalg.norm(q)
    if norm <= 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    return [value / norm for value in q]


def enu_vec_to_nwu(x, y, z):
    return [y, -x, z]


def nwu_vec_to_enu(x, y, z):
    return [-y, x, z]


def enu_quat_to_nwu(qw, qx, qy, qz):
    frame_rot = [np.sqrt(0.5), 0.0, 0.0, -np.sqrt(0.5)]
    return _quat_norm(_quat_mul(frame_rot, [qw, qx, qy, qz]))


def nwu_quat_to_enu(qw, qx, qy, qz):
    frame_rot = [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)]
    return _quat_norm(_quat_mul(frame_rot, [qw, qx, qy, qz]))