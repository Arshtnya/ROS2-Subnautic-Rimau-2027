"""Daun (leaf) behaviour tree untuk misi AUV.

Semua daun hanya bicara lewat topik yang sudah ada di auv_bridge:
    /auv/arm (Bool), /auv/target_depth (Float32), /auv/target_heading (Float32),
    /cmd_vel (Twist), dan membaca /auv/state (Float32MultiArray).
"""
import time

import py_trees
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, Vector3

from .common import depth_reached, heading_reached


class AuvLeaf(py_trees.behaviour.Behaviour):
    """Basis semua daun: menyimpan node ROS + state bersama."""

    def __init__(self, name, node, state):
        super().__init__(name)
        self.node = node
        self.state = state


class SetArmed(AuvLeaf):
    """Arm/disarm lewat /auv/arm, tunggu sampai terkonfirmasi di telemetry."""

    def __init__(self, node, state, value, timeout=5.0):
        super().__init__("Arm" if value else "Disarm", node, state)
        self.value = value
        self.timeout = timeout
        self.t0 = 0.0
        self.pub = node.create_publisher(Bool, "/auv/arm", 10)

    def initialise(self):
        self.pub.publish(Bool(data=self.value))
        self.t0 = time.monotonic()

    def update(self):
        if self.state.armed == self.value:
            return py_trees.common.Status.SUCCESS
        if time.monotonic() - self.t0 > self.timeout:
            return py_trees.common.Status.FAILURE
        self.pub.publish(Bool(data=self.value))   # kirim ulang kalau belum kena
        return py_trees.common.Status.RUNNING


class SetDepth(AuvLeaf):
    """Perintah tahan kedalaman; SUCCESS saat |depth - target| <= tol."""

    def __init__(self, node, state, target, tol=0.1, timeout=20.0):
        super().__init__("SetDepth(%.2fm)" % target, node, state)
        self.target = target
        self.tol = tol
        self.timeout = timeout
        self.t0 = 0.0
        self.pub = node.create_publisher(Float32, "/auv/target_depth", 10)

    def initialise(self):
        self.pub.publish(Float32(data=self.target))
        self.t0 = time.monotonic()

    def update(self):
        if depth_reached(self.state.depth, self.target, self.tol):
            return py_trees.common.Status.SUCCESS
        if time.monotonic() - self.t0 > self.timeout:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING


class SetHeading(AuvLeaf):
    """Perintah tahan arah kompas; SUCCESS saat error <= tol derajat."""

    def __init__(self, node, state, target, tol=10.0, timeout=20.0):
        super().__init__("SetHeading(%.0fdeg)" % target, node, state)
        self.target = target
        self.tol = tol
        self.timeout = timeout
        self.t0 = 0.0
        self.pub = node.create_publisher(Float32, "/auv/target_heading", 10)

    def initialise(self):
        self.pub.publish(Float32(data=self.target))
        self.t0 = time.monotonic()

    def update(self):
        if heading_reached(self.state.heading, self.target, self.tol):
            return py_trees.common.Status.SUCCESS
        if time.monotonic() - self.t0 > self.timeout:
            return py_trees.common.Status.FAILURE
        self.pub.publish(Float32(data=self.target))
        return py_trees.common.Status.RUNNING


class MoveSurge(AuvLeaf):
    """Maju/mundur selama `duration` detik lalu berhenti dan SUCCESS."""

    def __init__(self, node, state, speed, duration):
        super().__init__("MoveSurge(%.2f x %.1fs)" % (speed, duration), node, state)
        self.speed = speed
        self.duration = duration
        self.t0 = 0.0
        self.pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def initialise(self):
        self.t0 = time.monotonic()

    def update(self):
        self.pub.publish(Twist(linear=Vector3(x=self.speed)))
        if time.monotonic() - self.t0 >= self.duration:
            self.pub.publish(Twist())
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class Stop(AuvLeaf):
    """Hentikan gerak manual (Twist nol)."""

    def __init__(self, node, state):
        super().__init__("Stop", node, state)
        self.pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def update(self):
        self.pub.publish(Twist())
        return py_trees.common.Status.SUCCESS
