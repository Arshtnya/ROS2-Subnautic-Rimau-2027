"""Tiruan ESP32 supaya misi bisa diuji tanpa robot.

Meniru perilaku auv_bridge + ESP: terima perintah, gerakkan depth & heading
menuju target secara perlahan, lalu kabarkan lewat /auv/state & /auv/armed.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray
from geometry_msgs.msg import Twist

from .common import clamp, wrap180

RATE_HZ = 20.0
DEPTH_RATE = 0.1    # m/s
HEAD_RATE = 20.0     # deg/s


class FakeFc(Node):
    def __init__(self):
        super().__init__("fake_fc")
        self.armed = False
        self.depth = 0.0
        self.heading = 0.0
        self.target_depth = None
        self.target_heading = None
        self.surge = 0.0

        self.create_subscription(Bool, "/auv/arm", self.on_arm, 10)
        self.create_subscription(Float32, "/auv/target_depth", self.on_td, 10)
        self.create_subscription(Float32, "/auv/target_heading", self.on_th, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.pub_state = self.create_publisher(Float32MultiArray, "/auv/state", 10)
        self.pub_armed = self.create_publisher(Bool, "/auv/armed", 10)
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info("fake_fc jalan (tiruan ESP)")

    def on_arm(self, m):
        self.armed = m.data
        if not m.data:
            self.target_depth = None
            self.target_heading = None

    def on_td(self, m):
        self.target_depth = m.data

    def on_th(self, m):
        self.target_heading = m.data

    def on_cmd(self, m):
        self.surge = m.linear.x

    def tick(self):
        dt = 1.0 / RATE_HZ
        if self.armed:
            if self.target_depth is not None:
                self.depth += clamp(self.target_depth - self.depth, -DEPTH_RATE * dt, DEPTH_RATE * dt)
            if self.target_heading is not None:
                err = wrap180(self.target_heading - self.heading)
                self.heading = (self.heading + clamp(err, -HEAD_RATE * dt, HEAD_RATE * dt)) % 360.0
        self.pub_state.publish(Float32MultiArray(data=[self.heading, 0.0, 0.0, self.depth]))
        self.pub_armed.publish(Bool(data=self.armed))


def main():
    rclpy.init()
    node = FakeFc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
