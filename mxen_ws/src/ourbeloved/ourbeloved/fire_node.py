import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy


class fire_node(Node):

    def __init__(self):
        super().__init__("fire_node")
        self.subscription = self.create_subscription(Joy, "joy", self.listener_callback, 10)
        self.publisher = self.create_publisher(Bool, "fire", 10)
   #      timer_period = 0.05
   #      self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pressed = False

    def listener_callback(self, msg):
        if msg.buttons[5] and self.pressed == False:
            msg = Bool()
            msg.data = True
            self.publisher.publish(msg)
            self.pressed = True
        elif not msg.buttons[5] and self.pressed == True:
            msg = Bool()
            msg.data = False
            self.publisher.publish(msg)
            self.pressed = False
        


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = fire_node()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
