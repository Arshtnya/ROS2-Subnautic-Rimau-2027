# Bridge ROS 2 <-> auv-fc (ESP32) + odometri sederhana.
#
#   masuk dari ROS : /cmd_vel          (Twist : gerak manual)
#                    /auv/arm           (Bool  : arm/disarm)
#                    /auv/target_depth  (Float32: tahan kedalaman ini, meter)
#                    /auv/target_heading(Float32: tahan arah ini, derajat kompas)
#   keluar ke ROS  : /auv/state (Float32MultiArray: heading, pitch, roll, depth)
#                    /auv/armed (Bool: status armed sungguhan dari ESP)
#                    /odom     (nav_msgs/Odometry: posisi + orientasi + kecepatan)
#                    TF odom -> base_link (posisi robot, hasil hitung sendiri)
#
# ESP tidak diubah. UART Serial1 115200, protokol frame AA 55 (lihat link_proto.py).
#
# Gaya fungsi: satu node + fungsi-fungsi (tak ada class). State bersama di dict `s`.
import math

import rclpy
from std_msgs.msg import Bool, Float32, Float32MultiArray
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, Point, Vector3
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial

from .link_proto import Link, setpoint
from .odom_math import integrate

PORT = "/dev/ttyACM0"   # port UART ke ESP
BAUD = 115200
RATE = 20.0             # Hz loop (harus < failsafe ESP 500ms)


def _quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def main():
    rclpy.init()
    node = rclpy.create_node("auv_bridge")
    node.declare_parameter("port", PORT)
    node.declare_parameter("speed_scale", 0.5)   # meter/detik saat perintah gerak = 1
    port = node.get_parameter("port").value
    speed_scale = float(node.get_parameter("speed_scale").value)

    ser = serial.Serial(port, BAUD, timeout=0)   # buka port UART
    link = Link()                                # parser frame dari ESP
    tf = TransformBroadcaster(node)              # kirim TF odom -> base_link

    # catatan: perintah dari ROS + kondisi dari ESP + posisi hasil hitung sendiri
    s = {"armed": False, "surge": 0.0, "sway": 0.0, "heave": 0.0, "yaw": 0.0,
         "depth_hold": False, "heading_hold": False,
         "target_depth": 0.0, "target_heading": 0.0,
         "heading": 0.0, "pitch": 0.0, "roll": 0.0, "depth": 0.0,
         "x": 0.0, "y": 0.0, "last_t": node.get_clock().now()}

    pub_state = node.create_publisher(Float32MultiArray, "/auv/state", 10)
    pub_odom = node.create_publisher(Odometry, "/odom", 10)
    pub_armed = node.create_publisher(Bool, "/auv/armed", 10)

    # ---------- telinga ----------
    def on_cmd(m):
        s["surge"] = m.linear.x
        s["sway"] = m.linear.y
        s["heave"] = m.linear.z
        s["yaw"] = m.angular.z
        if m.linear.z != 0.0:        # gerak manual naik/turun mematikan mode tahan
            s["depth_hold"] = False
        if m.angular.z != 0.0:       # putar manual mematikan mode tahan arah
            s["heading_hold"] = False

    def on_arm(m):
        s["armed"] = m.data

    def on_target_depth(m):
        s["target_depth"] = m.data
        s["depth_hold"] = True       # minta tahan -> nyalakan mode tahan

    def on_target_heading(m):
        s["target_heading"] = m.data
        s["heading_hold"] = True

    node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)
    node.create_subscription(Bool, "/auv/arm", on_arm, 10)
    node.create_subscription(Float32, "/auv/target_depth", on_target_depth, 10)
    node.create_subscription(Float32, "/auv/target_heading", on_target_heading, 10)

    # ---------- odometri ----------
    def odometry():
        now = node.get_clock().now()
        dt = (now - s["last_t"]).nanoseconds * 1e-9   # detik sejak tick lalu
        s["last_t"] = now
        if dt <= 0.0 or dt > 0.5:                     # lewatkan kalau aneh/glitch
            return
        s["x"], s["y"] = integrate(s["x"], s["y"], s["heading"],
                                   s["surge"], s["sway"], speed_scale, dt)

        qx, qy, qz, qw = _quat(math.radians(s["roll"]), math.radians(s["pitch"]),
                               math.radians(-s["heading"]))
        stamp = now.to_msg()

        # TF odom -> base_link : dipakai rviz / nav2 / EKF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = s["x"]
        t.transform.translation.y = s["y"]
        t.transform.translation.z = -s["depth"]       # z ke atas, depth ke bawah
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        tf.sendTransform(t)

        # /odom : bentuk standar ROS. Kecepatan = taksiran dari perintah gerak.
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position = Point(x=s["x"], y=s["y"], z=-s["depth"])
        od.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        od.twist.twist.linear = Vector3(x=s["surge"] * speed_scale,
                                        y=s["sway"] * speed_scale, z=0.0)
        od.twist.twist.angular = Vector3(x=0.0, y=0.0, z=s["yaw"])
        pub_odom.publish(od)

    # ---------- loop utama ----------
    def tick():
        # 1) kirim setpoint ke ESP (biar failsafe tidak trip)
        ser.write(setpoint(
            armed=s["armed"], depth_hold=s["depth_hold"], heading_hold=s["heading_hold"],
            surge=s["surge"], sway=s["sway"], heave=s["heave"], yaw_rate=s["yaw"],
            target_depth=s["target_depth"], target_heading=s["target_heading"]))

        # 2) baca balasan ESP, publish telemetry-nya
        n = ser.in_waiting
        if n:
            for kind, d in link.feed(ser.read(n)):
                if kind == "telemetry":
                    s["heading"] = float(d["heading"])
                    s["pitch"] = float(d["pitch"])
                    s["roll"] = float(d["roll"])
                    s["depth"] = float(d["depth"])
                    pub_state.publish(Float32MultiArray(
                        data=[s["heading"], s["pitch"], s["roll"], s["depth"]]))
                    pub_armed.publish(Bool(data=bool(d["armed"])))

        # 3) hitung posisi sendiri lalu broadcast
        odometry()

    node.create_timer(1.0 / RATE, tick)
    node.get_logger().info("auv_bridge jalan di " + port)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
