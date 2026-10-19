import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

def main():
    rclpy.init()
    node = rclpy.create_node('mecanum')
    pub = node.create_publisher(Float32MultiArray, 'wheel_cmd', 10)

    jari_roda = 0.05   # meter
    L = 0.30           # setengah panjang + setengah lebar robot (meter)

    def terima(pesan):
        vx = pesan.linear.x      # maju (+) mundur (-)
        vy = pesan.linear.y      # geser kiri (+) kanan (-)
        wz = pesan.angular.z     # putar kiri (+) kanan (-)

        # rumus mecanum (hasil dalam rad/s)
        fl = (vx - vy - L * wz) / jari_roda   # depan kiri
        fr = (vx + vy + L * wz) / jari_roda   # depan kanan
        rl = (vx + vy - L * wz) / jari_roda   # belakang kiri
        rr = (vx - vy + L * wz) / jari_roda   # belakang kanan

        hasil = Float32MultiArray()
        hasil.data = [fl, fr, rl, rr]
        pub.publish(hasil)
        print('roda:', hasil.data)

    node.create_subscription(Twist, 'cmd_vel', terima, 10)
    rclpy.spin(node)