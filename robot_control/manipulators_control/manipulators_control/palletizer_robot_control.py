#!/usr/bin/env python3
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from .pymoveit2.moveit2 import MoveIt2
from typing import List
from manipulator_interfaces.srv import JointPosition
from manipulator_interfaces.srv import GripperDownPose
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
import math
import time

def action_availiable(node):
    return (node.moveit2._MoveIt2__move_action_client.server_is_ready() and
            node.moveit2._plan_kinematic_path_service.service_is_ready() and
            node.moveit2._plan_cartesian_path_service.service_is_ready() and
            node.moveit2._MoveIt2__follow_joint_trajectory_action_client.server_is_ready()) 

class RobotControl(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('palletizer_control')

        self.allow_execution = True
        self.allow_execution_general = True

        callback_group = ReentrantCallbackGroup()

        joint_names = ["rotate_column", "base_to_tr","first_cont_1", "first_cont_2", "first_cont_3",
                       "gripper_rot", "first_cont_to_tr", "from_tr_to_last", "second_cont_1", "second_cont_2"]

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name="base_link",
            end_effector_name="ee_link",
            group_name="palletizer_arm",
            callback_group=callback_group,
            execute_via_moveit=False,
            follow_joint_trajectory_action_name="/palletizer/palletizer_joint_trajectory_controller/follow_joint_trajectory"
        )        

        self.moveJ_srv = self.create_service(GripperDownPose, '/palletizer/MoveJ', self.moveJ_callback, callback_group=callback_group)
        self.moveL_srv = self.create_service(GripperDownPose, '/palletizer/MoveL', self.moveL_callback, callback_group=callback_group)
        self.moveJoint_srv = self.create_service(JointPosition, '/palletizer/MoveJoint', self.moveJoint_callback, callback_group=callback_group)
        self.moveJ_cyl_srv = self.create_service(GripperDownPose, '/palletizer/MoveJCyl', self.moveJCyl_callback, callback_group=callback_group)
        self.moveL_cyl_srv = self.create_service(GripperDownPose, '/palletizer/MoveLCyl', self.moveLCyl_callback, callback_group=callback_group)

        self.allow_sub_1 = self.create_subscription(Bool, '/palletizer/AllowExecution', self.allow_execution_callback, 10, callback_group=callback_group)
        self.allow_sub_2 = self.create_subscription(Bool, '/AllowExecution', self.allow_execution_callback_general, 10, callback_group=callback_group)

        self.vacuum_gripper_srv = self.create_service(SetBool, '/palletizer/vacuum_gripper/turn_on', self.vacuum_gripper_callback, callback_group=callback_group)
        self.suction_cup_pub = self.create_publisher(Bool, '/palletizer/palletizer/suction_cup/turn_on', 1)

    def moveJ_callback(self, request, response):
        if self.allow_execution:
            goal_pose = request
            goal_position = [goal_pose.position[0], goal_pose.position[1], goal_pose.position[2]]
            goal_orientation = [0, 0, -math.sin((goal_pose.gripper_angle)/2), math.cos((goal_pose.gripper_angle)/2)]
            print(goal_orientation)
            self.moveit2.move_to_pose(position=goal_position, quat_xyzw=goal_orientation, cartesian=False, tolerance_position=0.001, tolerance_orientation=0.001)
            response.success = self.moveit2.wait_until_executed()
        else:
            response.success = False
        return response
        
    def moveL_callback(self, request, response):
        if self.allow_execution and self.allow_execution_general:
            goal_pose = request
            goal_position = [goal_pose.position[0], goal_pose.position[1], goal_pose.position[2]]
            goal_orientation = [0, 0, -math.sin((goal_pose.gripper_angle)/2), math.cos((goal_pose.gripper_angle)/2)]
            self.moveit2.move_to_pose(position=goal_position, quat_xyzw=goal_orientation, cartesian=True, tolerance_position=0.001, tolerance_orientation=0.001)
            response.success = self.moveit2.wait_until_executed()
        else:
            response.success = False
        return response
    
    def moveJoint_callback(self, request, response):
        if self.allow_execution and self.allow_execution_general:
            conf = request.joint_position
            goal_position = [conf[0], conf[1], conf[1], -conf[1]+conf[2], -conf[2], conf[3], -conf[1], conf[2], conf[2], conf[1]-conf[2]]
            self.moveit2.move_to_configuration(joint_positions=goal_position, tolerance=0.001)
            response.success = self.moveit2.wait_until_executed()
        else:
            response.success = False
        return response

    def moveJCyl_callback(self, request, response):
        if self.allow_execution and self.allow_execution_general:
            goal_pose = request
            goal_position = [math.cos(goal_pose.position[0])*goal_pose.position[1], math.sin(goal_pose.position[0])*goal_pose.position[1], goal_pose.position[2]]
            goal_orientation = [0, 0, -math.sin((goal_pose.gripper_angle)/2), math.cos((goal_pose.gripper_angle)/2)]
            self.moveit2.move_to_pose(position=goal_position, quat_xyzw=goal_orientation, cartesian=False, tolerance_position=0.001, tolerance_orientation=0.001)
            response.success = self.moveit2.wait_until_executed()
        else:
            response.success = False
        return response

    def moveLCyl_callback(self, request, response):
        if self.allow_execution and self.allow_execution_general:
            goal_pose = request
            goal_position = [math.cos(goal_pose.position[0])*goal_pose.position[1], math.sin(goal_pose.position[0])*goal_pose.position[1], goal_pose.position[2]]
            goal_orientation = [0, 0, -math.sin((goal_pose.gripper_angle)/2), math.cos((goal_pose.gripper_angle)/2)]
            self.moveit2.move_to_pose(position=goal_position, quat_xyzw=goal_orientation, cartesian=True)
            response.success = self.moveit2.wait_until_executed()
        else:
            response.success = False
        return response
    
    def allow_execution_callback(self, msg):
        self.reset_execution(msg)
        self.allow_execution = msg.data

    def allow_execution_callback_general(self, msg):
        self.reset_execution(msg)
        self.allow_execution_general = msg.data

    def reset_execution(self, msg):
        if not msg.data and (self.moveit2._MoveIt2__is_motion_requested or self.moveit2._MoveIt2__is_executing):
            self.moveit2.goal_handle.cancel_goal_async()
            self.moveit2.force_reset_executing_state()

    def vacuum_gripper_callback(self, request : SetBool.Request, response : SetBool.Response):
        if self.allow_execution and self.allow_execution_general:
            msg = Bool()
            msg.data = request.data
            self.suction_cup_pub.publish(msg)
            response.success = True
        else:
            response.success = False
        return response


def main():
    robot_control_node = RobotControl()
    while not action_availiable(robot_control_node):
        continue

    executor = MultiThreadedExecutor(2)

    executor.add_node(robot_control_node)

    executor_thread = Thread(target=executor.spin, daemon=True)

    executor_thread.start()
    robot_control_node.create_rate(1.0).sleep()
    robot_control_node.moveit2.move_to_configuration(joint_positions=[-0.785, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], tolerance=0.001)
    robot_control_node.moveit2.wait_until_executed()


    rate = robot_control_node.create_rate(5)
    while rclpy.ok() and 'controller_manager' in robot_control_node.get_node_names():
        rate.sleep()

    rclpy.shutdown()

    executor_thread.join()


if __name__ == "__main__":
    main()




