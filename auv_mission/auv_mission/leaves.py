"""Daun-daun behaviour tree untuk misi AUV."""
import time

import py_trees
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, Vector3

from .common import depth_reached, heading_reached

SUKSES = py_trees.common.Status.SUCCESS
GAGAL = py_trees.common.Status.FAILURE
JALAN = py_trees.common.Status.RUNNING


class SetArmed(py_trees.behaviour.Behaviour):
    def __init__(self, node, state, value):
        super().__init__("Arm" if value else "Disarm")
        self.state = state
        self.value = value
        self.pub = node.create_publisher(Bool, "/auv/arm", 10)

    def initialise(self):
        self.mulai = time.monotonic()

    def update(self):
        if self.state.armed == self.value:
            return SUKSES
        if time.monotonic() - self.mulai > 5.0:
            return GAGAL
        self.pub.publish(Bool(data=self.value))
        return JALAN


class SetDepth(py_trees.behaviour.Behaviour):
    def __init__(self, node, state, target):
        super().__init__("SetDepth(%.2fm)" % target)
        self.state = state
        self.target = target
        self.pub = node.create_publisher(Float32, "/auv/target_depth", 10)

    def initialise(self):
        self.mulai = time.monotonic()

    def update(self):
        if depth_reached(self.state.depth, self.target, 0.1):
            return SUKSES
        if time.monotonic() - self.mulai > 20.0:
            return GAGAL
        self.pub.publish(Float32(data=self.target))
        return JALAN


class SetHeading(py_trees.behaviour.Behaviour):
    def __init__(self, node, state, target):
        super().__init__("SetHeading(%.0fdeg)" % target)
        self.state = state
        self.target = target
        self.pub = node.create_publisher(Float32, "/auv/target_heading", 10)

    def initialise(self):
        self.mulai = time.monotonic()

    def update(self):
        if heading_reached(self.state.heading, self.target, 10.0):
            return SUKSES
        if time.monotonic() - self.mulai > 20.0:
            return GAGAL
        self.pub.publish(Float32(data=self.target))
        return JALAN


class Gerak(py_trees.behaviour.Behaviour):
    def __init__(self, node, duration, surge=0.0, sway=0.0):
        super().__init__("Gerak")
        self.duration = duration
        self.surge = surge
        self.sway = sway
        self.pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def initialise(self):
        self.mulai = time.monotonic()

    def update(self):
        if time.monotonic() - self.mulai >= self.duration:
            return SUKSES
        self.pub.publish(Twist(linear=Vector3(x=self.surge, y=self.sway)))
        return JALAN

    def terminate(self, new_status):
        self.pub.publish(Twist())   # berhenti


class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super().__init__("Stop")
        self.pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def update(self):
        self.pub.publish(Twist())
        return SUKSES