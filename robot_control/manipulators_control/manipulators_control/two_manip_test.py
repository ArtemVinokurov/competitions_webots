#!/usr/bin/env python3
from threading import Thread
import copy
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from typing import List
from manipulator_interfaces.srv import GoalPose
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import time



class SimpleClient(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('simple_client')
        self.cli_joint = self.create_client(GoalPose, '/palletizer/MoveJ')
        self.vacuum_gripper_pub = self.create_publisher(Bool, '/palletizer/suction_cup/turn_on', qos_profile=1)

        while not self.cli_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")

        self.cli_linear = self.create_client(GoalPose, '/palletizer/MoveL')

        while not self.cli_linear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")

        self.req = GoalPose.Request()


        self.initial_joint_pose = Pose()
        self.initial_joint_pose.position.x = 0.0
        self.initial_joint_pose.position.y = 0.20995
        self.initial_joint_pose.position.z = 0.1486

        self.initial_joint_pose.orientation.x = 0.0
        self.initial_joint_pose.orientation.y = 0.0
        self.initial_joint_pose.orientation.z = 0.0
        self.initial_joint_pose.orientation.w = 1.0





    def move_to_init(self):
        if not self.send_movej(self.initial_joint_pose):
            self.get_logger().error("Failed to move initial pose")


    
    def send_movej(self, pose : Pose):
        self.req.position = [pose.position.x,pose.position.y,pose.position.z]
        self.req.orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        self.future = self.cli_joint.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()
    

    def send_movel(self, pose):
        self.req.position = [pose.position.x,pose.position.y,pose.position.z]
        self.req.orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        self.future = self.cli_linear.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def gripper_turn(self, state):
        msg = Bool()
        msg.data = state
        self.vacuum_gripper_pub.publish(msg)

    def grasp_and_move(self, obj_pose : Pose, target_pose : Pose):
        z_offset = 0.05
        start_pose = copy.deepcopy(obj_pose)
        start_pose.position.z += z_offset

        response = False

        response = self.send_movej(obj_pose)

        if not response:
            print("Start pose not allowed")
            return
        
        self.gripper_turn(True)

        if not self.send_movel(obj_pose):
            print("Object pose not allowed")
            return
        start_pose.position.z += 0.05
        if not self.send_movel(start_pose):
            return
        
        start_target_pose = copy.deepcopy(target_pose)
        start_target_pose.position.z += z_offset + 0.01

        if not self.send_movej(start_target_pose):
            print("Start target pose not allowed")
            return 
        
        if not self.send_movel(target_pose):
            print("Target pose not allowed")
            return

        self.gripper_turn(False) 

        time.sleep(0.5)

        self.send_movel(start_target_pose)

        self.move_to_init()






def Pose_msg_create(position, orientation):
    msg = Pose()
    msg.position.x = position[0]
    msg.position.y = position[1]
    msg.position.z = position[2]

    msg.orientation.x = orientation[0]
    msg.orientation.y = orientation[1]
    msg.orientation.z = orientation[2]
    msg.orientation.w = orientation[3]

    return msg


    
def main():
    client = SimpleClient()

    first_cube_pose = Pose_msg_create([0.2296, -0.09, 0.0245], [0.0, 0.0, 0.0, 1.0])
    second_cube_pose = Pose_msg_create([0.2296, -0.02, 0.0245], [0.0, 0.0, 0.0, 1.0])
    third_cube_pose = Pose_msg_create([0.2296, 0.05, 0.0245], [0.0, 0.0, 0.0, 1.0])

    first_cube_target_pose = Pose_msg_create([0.0, 0.27, 0.026], [0.0, 0.0, 0.0, 1.0])
    second_cube_target_pose = Pose_msg_create([0.0, 0.27, 0.055], [0.0, 0.0, 0.0, 1.0])
    third_cube_target_pose = Pose_msg_create([0.0, 0.27, 0.08], [0.0, 0.0, 0.0, 1.0])

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