import rclpy
from std_msgs.msg import String



def main():
    rclpy.init()
    node = rclpy.create_node('denger')

    def terima(pesan):
        print('terima:', pesan.data)

    node.create_subscription(String, 'chatter', terima, 10)
    rclpy.spin(node)