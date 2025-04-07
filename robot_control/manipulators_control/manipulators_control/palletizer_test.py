#!/usr/bin/env python3
from threading import Thread
import copy
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from typing import List
from manipulator_interfaces.srv import GripperDownPose
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import time
from std_srvs.srv import SetBool


class SimpleClient(Node):
    def __init__(self):
        super().__init__('palletizer_simple_client')
        self.cli_joint = self.create_client(GripperDownPose, '/palletizer/MoveJ')
        self.vacuum_gripper_turn = self.create_client(SetBool, '/palletizer/vacuum_gripper/turn_on')

        while not self.cli_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")

        self.cli_linear = self.create_client(GripperDownPose, '/palletizer/MoveL')

        while not self.cli_linear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")

        self.req = GripperDownPose.Request()

        self.init_req = GripperDownPose.Request()
        self.init_req.position = [0.20995, 0.0, 0.1486]
        self.init_req.gripper_angle = 0.0

    def move_to_init(self):
        self.future = self.cli_joint.call_async(self.init_req)
        rclpy.spin_until_future_complete(self, self.future)
        if not self.future.result():
            self.get_logger().error("Failed to move initial pose")
    
    def send_movej(self, pose):
        self.req.position = pose[0]
        self.req.gripper_angle = pose[1]
        self.future = self.cli_joint.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_movel(self, pose):
        self.req.position = pose[0]
        self.req.gripper_angle = pose[1]
        self.future = self.cli_linear.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def gripper_turn(self, state):
        msg = SetBool.Request()
        msg.data = state
        self.vacuum_gripper_turn.call_async(msg)

    def grasp_and_move(self, obj_pose, target_pose):
        z_offset = 0.05
        start_pose = copy.deepcopy(obj_pose)
        start_pose[0][2] += z_offset

        response = False

        response = self.send_movej(obj_pose)

        if not response:
            print("Start pose not allowed")
            return
        
        self.gripper_turn(True)

        if not self.send_movel(obj_pose):
            print("Object pose not allowed")
            return
        start_pose[0][2] += 0.05
        if not self.send_movel(start_pose):
            return
        
        start_target_pose = copy.deepcopy(target_pose)
        start_target_pose[0][2] += z_offset + 0.01

        if not self.send_movej(start_target_pose):
            print("Start target pose not allowed")
            return 
        
        if not self.send_movel(target_pose):
            print("Target pose not allowed")
            return

        self.gripper_turn(False) 

        time.sleep(0.5)

        self.send_movel(start_target_pose)

        # self.move_to_init()


def Pose_msg_create(position, gripper_angle):
    return [position, float(gripper_angle)]


def main():
    rclpy.init()
    client = SimpleClient()

    first_cube_pose = Pose_msg_create([0.269,-0.0961, 0.0245], 0)
    second_cube_pose = Pose_msg_create([0.269, 0.01, 0.0245], 0)
    third_cube_pose = Pose_msg_create([0.269, 0.114, 0.0245], 0)

    first_cube_target_pose = Pose_msg_create([0.0, 0.294, 0.026], 0)
    second_cube_target_pose = Pose_msg_create([0.0, 0.294, 0.055], 0)
    third_cube_target_pose = Pose_msg_create([0.0, 0.294, 0.08], 0)

    obj_poses = [first_cube_pose, second_cube_pose, third_cube_pose]
    target_poses = [first_cube_target_pose, second_cube_target_pose, third_cube_target_pose]

    # while rclpy.ok():
    #     response = client.send_movel(pose1)
    #     response = client.send_movel(pose2)

    for i in range(3):
        client.grasp_and_move(obj_poses[i], target_poses[i])


    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()