"""sim.utils.math3d — Các hàm toán học 3D: quaternion, ma trận xoay, skew-symmetric.

Quy ước quaternion: q = [w, x, y, z]  (Hamilton, scalar-first).
Quy ước frame: body → NED khi xoay bằng quat_rotate(q, v_body).
"""
import numpy as np
from numpy.typing import NDArray

# ---------------------------------------------------------------------------
# Quaternion cơ bản
# ---------------------------------------------------------------------------

def quat_mult(q: NDArray, r: NDArray) -> NDArray:
    """Nhân Hamilton hai quaternion q ⊗ r."""
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1,
    ])


def quat_conj(q: NDArray) -> NDArray:
    """Liên hợp quaternion (đảo chiều phần vector)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q: NDArray) -> NDArray:
    """Chuẩn hoá quaternion về đơn vị. Trả [1,0,0,0] nếu norm ≈ 0."""
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_rotate(q: NDArray, v: NDArray) -> NDArray:
    """Xoay vector v bằng quaternion q: v' = q ⊗ [0,v] ⊗ q*.

    Nếu q là quaternion body→NED thì quat_rotate(q, v_body) → v_NED.
    """
    v_q = np.array([0.0, v[0], v[1], v[2]])
    return quat_mult(quat_mult(q, v_q), quat_conj(q))[1:]


def quat_from_rotvec(phi: NDArray) -> NDArray:
    """Tạo quaternion từ rotation vector (small angle OK, big angle cũng đúng).

    phi: vector 3D, hướng = trục xoay, độ lớn = góc (rad).
    """
    angle = np.linalg.norm(phi)
    if angle < 1e-12:
        # Xấp xỉ bậc 1 cho góc nhỏ
        return quat_normalize(np.array([1.0, phi[0]*0.5, phi[1]*0.5, phi[2]*0.5]))
    half = angle * 0.5
    s = np.sin(half) / angle
    return np.array([np.cos(half), phi[0]*s, phi[1]*s, phi[2]*s])


def quat_to_euler(q: NDArray) -> NDArray:
    """Chuyển quaternion → Euler angles [roll, pitch, yaw] (rad).

    Thứ tự ZYX (aerospace convention).
    """
    w, x, y, z = q
    # Roll (φ)
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    # Pitch (θ)
    sinp = 2.0 * (w*y - z*x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    # Yaw (ψ)
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw])


def quat_from_euler(roll: float, pitch: float, yaw: float) -> NDArray:
    """Tạo quaternion từ Euler angles (ZYX convention)."""
    cr, sr = np.cos(roll*0.5),  np.sin(roll*0.5)
    cp, sp = np.cos(pitch*0.5), np.sin(pitch*0.5)
    cy, sy = np.cos(yaw*0.5),   np.sin(yaw*0.5)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])


# ---------------------------------------------------------------------------
# Ma trận xoay
# ---------------------------------------------------------------------------

def rot_from_quat(q: NDArray) -> NDArray:
    """Ma trận xoay 3×3 từ quaternion (body → NED)."""
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


# ---------------------------------------------------------------------------
# Skew-symmetric
# ---------------------------------------------------------------------------

def skew(v: NDArray) -> NDArray:
    """Ma trận skew-symmetric [v]× từ vector 3D."""
    return np.array([
        [ 0.0, -v[2],  v[1]],
        [ v[2],  0.0, -v[0]],
        [-v[1],  v[0],  0.0],
    ])
