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
        self.control_mode = 'joint'

    def timer_callback(self):
        self.xarm.set_joints(self.newJoints, motion_mode='high_acc')

    def listener_callback(self, msg):
        #resting
        #rest
        if msg.buttons[3]: #square
            self.xarm.rest()
            time.sleep(2)
            self.newJoints = self.xarm.get_joints()

        #gripper toggle
        if msg.buttons[5]: #right bunmper
            self.xarm.grip(1)
        else:
            self.xarm.grip(0)

        
        #homing button
        if msg.buttons[10]: #domer
            self.xarm.home()
            time.sleep(2)
            self.newJoints = self.xarm.get_joints()

        if msg.buttons[8]:
            self.control_mode = 'cartesian'
        
        if msg.buttons[9]:
            self.control_mode = 'joint'

        # Joint control mode
        if self.control_mode == 'joint':
            joint0 = msg.axes[0]+self.newJoints[0]
            joint1 = msg.axes[1]+self.newJoints[1]
            joint2 = msg.axes[4]+self.newJoints[2]
            joint3 = msg.axes[3]+self.newJoints[3]
            joint4 = msg.axes[7]+self.newJoints[4]
            joint5 = msg.axes[6]+self.newJoints[5]
            
            joints = (joint0, joint1, joint2, joint3, joint4, joint5)
        
            if self.xarm.is_goal_valid(joints) == 0:
                self.newJoints = (joint0, joint1, joint2, joint3, joint4, joint5)

        if self.control_mode == 'cartesian':
            self.get_logger().info(f"diddy booty jackson")
            diff_x = msg.axes[3]
            diff_y = msg.axes[4]
            diff_z = msg.axes[1]

            ik_joints = self.cartesian_mode(diff_x, diff_y, diff_z) 

            if ik_joints is not None and self.xarm.is_goal_valid(joints) == 0:
                self.newJoints = ik_joints
            
            


        # self.get_logger().info(f"X: {msg.axes[0]}, Y: {msg.axes[1]}")

    def cartesian_mode(self, diff_x, diff_y, diff_z):
        #log that goal is being executed
        prev_joints = [1000, 1000, 1000, 1000, 1000, 1000]
        prev_dist = 1000
        stuck_count = 0
        #actuate goal request 
        present_joints = self.xarm.get_joints()
        present_htm, _ = fk(present_joints)
        x = goal_htm[0,3]
        y = goal_htm[1,3]
        z = goal_htm[2,3] 
        goal_htm = present_htm.copy()
        goal_htm[0,3] = x + diff_x
        goal_htm[1,3] = y + diff_y
        goal_htm[2,3] = z + diff_z
        
        goal_joints = self.ik_intermediate_step(present_htm, goal_htm)

        return goal_joints  
    
    def ik_intermediate_step(self, present_htm, goal_htm):
            """break path from current pos to end goal into smaller steps for numerical ik. returns None for impossible inverse kinematics step"""
            #intialise goal x,y,zs from htm
            final_x = goal_htm[0,3]
            final_y = goal_htm[1,3]
            final_z = goal_htm[2,3]
            initial_x = present_htm[0,3]
            initial_y = present_htm[1,3]
            initial_z = present_htm[2,3]
    
            #these will be important later
            intermediate_htms = []
            intermediate_angles = self.xarm.get_joints()
            no_more_intermediate_frames_flag = False
    
    
            #while distance between goal and current pos is greater than 0.5 in any cartesian direction, inch the x, y, z closer by 0.5 until in range; save all intermediate frames in array
            #maybe change from 0.5?
            while not no_more_intermediate_frames_flag:
                self.get_logger().info(f"{initial_x}, {initial_y}, {initial_z}")
                no_more_intermediate_frames_flag = True
    
                if(abs(final_x - initial_x) > 2):
                    no_more_intermediate_frames_flag = False
                    initial_x = initial_x + 2 * (final_x - initial_x)/abs(final_x - initial_x)
                if(abs(final_y - initial_y) > 2):
                    no_more_intermediate_frames_flag = False
                    initial_y = initial_y + 2 * (final_y - initial_y)/abs(final_y - initial_y)
                if(abs(final_z - initial_z) > 2):
                    no_more_intermediate_frames_flag = False
                    initial_z = initial_z + 2 * (final_z - initial_z)/abs(final_z - initial_z)
      
                if(not no_more_intermediate_frames_flag):
                    intermediate_htm = goal_htm.copy()
                    intermediate_htm[0,3] = initial_x
                    intermediate_htm[1,3] = initial_y
                    intermediate_htm[2,3] = initial_z
                    intermediate_htms.append(intermediate_htm)
    
    
            intermediate_htms.append(goal_htm.copy()) #goal htm is the last fk function to call so goes at end of list
    
            self.get_logger().info(f"HTMS: {intermediate_htms}\nnumber:{len(intermediate_htms)}")
    
    
            for i in range(len(intermediate_htms)):
                intermediate_angles = ik(intermediate_angles, intermediate_htms[i])
                self.get_logger().info(f"IK{i}/{len(intermediate_htms)}: {intermediate_angles}")
                self.get_logger().info(f"after")
    
    
                if intermediate_angles is None: #if impossible kinematics step
                    self.get_logger().info(f"blabla")
    
                    return None
    
    
            return intermediate_angles
    

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = ControllerSubscriber()

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
