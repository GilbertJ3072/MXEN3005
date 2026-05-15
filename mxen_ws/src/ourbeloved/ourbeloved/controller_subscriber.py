import copy
import rclpy
import time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from xarmclient import XArm

from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
from ourbeloved.wx250s_kinematics import fk, ik
import numpy as np


class ControllerSubscriber(Node):
    def __init__(self):
        super().__init__("controller_subscriber")
        self.subscription1 = self.create_subscription(Joy, "joy", self.listener_callback, 10)
        self.subscription2 = self.create_subscription(JointState, "precise_joints", self.precise_callback, 10)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.xarm = XArm()
        self.newJoints = self.xarm.get_joints()
        self.grip = 0
        self.control_mode = 'joint'
        self.stick = [0,0,0,0,0,0,0,0]
        self.stickFlag = False
        self.driftFlag = False
        self.tune_vel = (45,)*6
        # precise movement
        self.within_tolerance = True
        self.within_count = 0
        self.tolerance = 0.1
        self.pi_threshold = 0.5
        self.goal_joints = list(self.xarm.get_joints())
        self.actual_goal_joints = list(self.xarm.get_joints())
        self.prev_joints = list(self.xarm.get_joints())
        # PI
        self.integral = [0.0] * 6
        self.prev_errors = [0.0] * 6  # add this line to __init__
        self.kp = 0.5
        self.ki = 0.2
        self.integral_clamp = 50.0
        self.phase = 'initial'  # 'initial' | 'moving' | 'controller'
        # Cartesian
        self.ref_htm, _ = fk(self.xarm.get_joints())

    def precise_callback(self, msg):
        goal_validity = self.xarm.is_goal_valid(tuple(msg.position))
        if(goal_validity==0):
            self.within_tolerance = False
            self.within_count = 0
            self.integral = [0.0] * 6
            self.prev_errors = [0.0] * 6
            self.actual_goal_joints = list(msg.position)
            self.goal_joints = list(msg.position)
            self.prev_joints = list(msg.position)
            self.settled_count = 0
            self.phase = 'initial'
            self.control_mode = 'joints'
        else:
            self.get_logger().info(f"Recieved invalid joint request {msg.position}\n Error State {goal_validity}")

    def timer_callback(self):
        if not self.within_tolerance:
            cur_joints = self.xarm.get_joints()
            errors = [self.actual_goal_joints[i] - cur_joints[i] for i in range(6)]
            problem_joints = [i for i in range(6) if abs(errors[i]) > self.tolerance]
            self.get_logger().info(f"errors: {[f'{e:.3f}' for e in errors]} | pjs: {problem_joints}")

            if len(problem_joints) == 0:
                self.within_count += 1
                if self.within_count >= 5:
                    self.within_tolerance = True
                    self.get_logger().info("Goal reached within tolerance")
            elif self.stickFlag:
                self.within_tolerance = True
            else:
                self.within_count = 0

                if self.phase == 'initial':
                    self.xarm.set_joints(tuple(self.actual_goal_joints), motion_mode='high_acc', velocities=[30]*6)
                    self.phase = 'moving'

                elif self.phase == 'moving':
                    movement = [abs(cur_joints[i] - self.prev_joints[i]) for i in range(6)]
                    if all(m <= 0.2 for m in movement):
                        self.settled_count += 1
                        if self.settled_count >= 5:
                            self.phase = 'pi'
                            self.integral = [0.0] * 6
                            self.settled_count = 0
                    else:
                        self.settled_count = 0
                    self.prev_joints = cur_joints

                elif self.phase == 'pi':
                    for i in range(6):
                        self.integral[i] = np.clip(
                            self.integral[i] + errors[i],
                            -self.integral_clamp,
                            self.integral_clamp
                        )
                    for i in problem_joints:
                        correction = self.kp * errors[i] + self.ki * self.integral[i]
                        self.goal_joints[i] = self.actual_goal_joints[i] + correction
                    self.xarm.set_joints(tuple(self.goal_joints), motion_mode='high_acc', velocities=[30]*6)


        self.stickFlag = any(self.stick[i] != 0.0 for i in [0, 1, 3, 4, 6, 7])
        if self.stickFlag:
            if self.control_mode == 'cartesian':
                self.xarm.set_joints(self.newJoints)
            else:
                self.xarm.set_joints(self.newJoints, motion_mode='high_acc', velocities=self.tune_vel)
            self.driftFlag = True
        if not self.stickFlag:
            self.newJoints = self.xarm.get_joints()
            if self.driftFlag:
                self.xarm.set_joints(self.newJoints)
                self.driftFlag = False

    def listener_callback(self, msg):
        #resting
        self.stick = msg.axes
        #rest
        if msg.buttons[3]: #square
            self.xarm.rest()
            time.sleep(2)
            self.newJoints = self.xarm.get_joints()

        #gripper toggle
      #  if msg.buttons[5]: #right bunmper
      #      self.xarm.grip(1)
      #  else:
      #      self.xarm.grip(0)

        
        #homing button
        if msg.buttons[10]: #domer
            self.xarm.home()
            time.sleep(2)
            self.newJoints = self.xarm.get_joints()

        if msg.buttons[8]:
            self.control_mode = 'cartesian'
            self.ref_htm, _ = fk(self.xarm.get_joints())
            
        
        if msg.buttons[9]:
            self.control_mode = 'joint'

        # Joint control mode
        if self.control_mode == 'joint':
            self.tune_vel = (40,)*6
            self.get_logger().info(f"Joint Mode")

            js0 = 1
            js1 = 1
            js2 = 1
            js3 = 1
            js4 = 1
            js5 = 1

            joint0 = js0*msg.axes[0]+self.newJoints[0]
            joint1 = js1*msg.axes[7]+self.newJoints[1]
            joint2 = js2*msg.axes[4]+self.newJoints[2]
            joint3 = js3*msg.axes[3]+self.newJoints[3]
            joint4 = js4*msg.axes[1]+self.newJoints[4]
            joint5 = js5*msg.axes[6]+self.newJoints[5]
            
            joints = (joint0, joint1, joint2, joint3, joint4, joint5)
        
            if self.xarm.is_goal_valid(joints) == 0:
                self.newJoints = (joint0, joint1, joint2, joint3, joint4, joint5)

        if self.control_mode == 'cartesian':
            self.tune_vel = (30,)*6
            self.get_logger().info(f"Cartesian Mode")
            diff_x = msg.axes[4]
            diff_y = msg.axes[3]
            diff_z = msg.axes[1]

            ik_joints = self.cartesian_mode(diff_x, diff_y, diff_z) 

            if ik_joints is not None and self.xarm.is_goal_valid(ik_joints) == 0:
                self.newJoints = ik_joints
            
            


        # self.get_logger().info(f"X: {msg.axes[0]}, Y: {msg.axes[1]}")

    def cartesian_mode(self, diff_x, diff_y, diff_z):

        rotation_htm = self.ref_htm.copy()
        present_htm, _ = fk(self.newJoints)
        goal_htm = present_htm.copy()

        rot_mat = rotation_htm[0:3,0:3]

        diff_xyz = np.array([diff_x, diff_y, diff_z])

        rot_diff_xyz = rot_mat @ diff_xyz

        goal_htm[0:3,3] += rot_diff_xyz
        
        goal_joints = self.ik_intermediate_step(
            present_htm,
            goal_htm
        )

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
            intermediate_angles = self.newJoints
            no_more_intermediate_frames_flag = False
    
    
            #while distance between goal and current pos is greater than 0.5 in any cartesian direction, inch the x, y, z closer by 0.5 until in range; save all intermediate frames in array
            #maybe change from 0.5?
            while not no_more_intermediate_frames_flag:
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
    
    
    
            for i in range(len(intermediate_htms)):
                intermediate_angles = ik(intermediate_angles, intermediate_htms[i])
    
    
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
