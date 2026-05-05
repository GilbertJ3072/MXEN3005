import rclpy
import time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from xarmclient import XArm

from sensor_msgs.msg import Joy
from std_msgs.msg import String


class ControllerSubscriber(Node):

    def __init__(self):
        super().__init__("controller_subscriber")
        self.subscription = self.create_subscription(Joy, "joy", self.listener_callback, 10)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.xarm = XArm()
        self.newJoints = self.xarm.get_joints()
        self.grip = 0

    def timer_callback(self):
        self.xarm.set_joints(self.newJoints)

    def listener_callback(self, msg):
        if msg.buttons[5]:
            self.xarm.grip(1)
        else:
            self.xarm.grip(0)

        if msg.buttons[10]:
            self.xarm.home()
            time.sleep(2)
            self.newJoints = self.xarm.get_joints()

        joint0 = msg.axes[0]+self.newJoints[0]
        joint1 = msg.axes[4]+self.newJoints[1]
        joint2 = msg.axes[1]+self.newJoints[2]
        joint3 = msg.axes[3]+self.newJoints[3]
        joint4 = msg.axes[7]+self.newJoints[4]
        joint5 = msg.axes[6]+self.newJoints[5]
        
        joints = (joint0, joint1, joint2, joint3, joint4, joint5)

        if self.xarm.is_goal_valid(joints) == 0:
            self.newJoints = (joint0, joint1, joint2, joint3, joint4, joint5)

        self.get_logger().info(f"X: {msg.axes[0]}, Y: {msg.axes[1]}")


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = ControllerSubscriber()

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
