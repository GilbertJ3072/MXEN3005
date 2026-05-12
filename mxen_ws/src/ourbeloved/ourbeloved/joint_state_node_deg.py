import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from xarmclient import XArm
from sensor_msgs.msg import JointState
import numpy as np

class joint_state_node_deg(Node):

    def __init__(self):
        super().__init__("joint_state_node_deg")
        self.publisher = self.create_publisher(JointState, "joint_state_deg", 10) 
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.xarm = XArm()

    def timer_callback(self):
        p = self.xarm.get_joints()
        P = [p[0], p[1], p[2], p[3], p[4], p[5]]
        msg = JointState()
        msg.position = np.array(P)
        self.publisher.publish(msg)
        #self.get_logger().info(f"Position: {msg.position}")


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = joint_state_node_deg()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()

