import rclpy
from std_msgs.msg import String



def main():
    rclpy.init()
    node = rclpy.create_node('denger')
    pub = node.create_publisher(String, 'chatter', 10)

    def kirim():
        angka = 0
        angka+=1
        pesan = String()
        pesan.data = angka
        pub.publish(pesan)
        print('kirim:', pesan.data)

    node.create_timer(1.0, kirim)
    rclpy.spin(node)