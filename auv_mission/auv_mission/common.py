"""Bagian murni (tanpa ROS/py_trees) dari misi: matematika + state bersama.

Dipisah supaya bisa dites di mana saja tanpa perlu ROS terpasang.
"""


def wrap180(d):
    """Bungkus sudut ke rentang (-180, 180]."""
    while d > 180.0:
        d -= 360.0
    while d <= -180.0:
        d += 360.0
    return d


def clamp(v, lo, hi):
    """Batasi v ke [lo, hi]."""
    return lo if v < lo else hi if v > hi else v


def depth_reached(cur, target, tol):
    """True kalau kedalaman sudah dalam toleransi (meter)."""
    return abs(cur - target) <= tol


def heading_reached(cur, target, tol):
    """True kalau arah kompas sudah dalam toleransi (derajat), lewat sisi pendek."""
    return abs(wrap180(target - cur)) <= tol


class State:
    """Kondisi robot terkini, diisi dari telemetry ESP, dibaca daun-daun BT."""

    def __init__(self):
        self.heading = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.depth = 0.0
        self.armed = False
        self.last_rx = 0.0   # waktu (monotonic) terima telemetry terakhir
