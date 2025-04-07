#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from manipulator_interfaces.srv import GripperDownPose, GoalPose, GripperCmd
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int8MultiArray, Float32

# Импортируем класс для движения паллетайзера из palletizer_test.py
from .palletizer_test import SimpleClient as PalSimpleClient
# Импортируем класс для движения углового из angle_test.py
from .angle_test import SimpleClient as AngSimpleClient
import copy


# Наследуем все методы и свойства класса SimpleClient.
class PalletizerClient(PalSimpleClient):
    ''' Класс для управления паллетайзером. '''
    def __init__(self):
        super().__init__() # rclpy.init() вызывается в родительском классе SimpleClient
        self.move_to_init()


class AngleClient(AngSimpleClient, Node):
    ''' Класс для управления угловым манипулятором. '''
    def __init__(self):
        Node.__init__(self, 'simple_angle_client')

        # Создаем клиентов для сервисов управления манипулятором:
        self.cli_joint = self.create_client(GoalPose, '/angle/MoveJ')
        self.gripper_cli = self.create_client(GripperCmd, 'angle/gripper_cmd')

        while not self.cli_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")

        self.cli_linear = self.create_client(GoalPose, '/angle/MoveL')

        while not self.cli_linear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")

        # Задаем начальную позицию углового манипулятора:
        init_point = {      'x': 0.19522,   'y': 0.0, 'z': 0.1936}
        init_quaternion = { 'x': 0.0,       'y': 1.0, 'z': 0.0,     'w': 0.0}
        self.initial_joint_pose = Pose(position=Point(**init_point), orientation=Quaternion(**init_quaternion))

        self.req = GoalPose.Request()

        self.move_to_init()


class LampsConveyor(Node):
    ''' Класс для управления лампами и конвейером. '''
    def __init__(self):
        super().__init__('simple_client_lamps_conveyor')
        # Создаем объекты для передачи массива для ламп через топик:
        self.left_lamp_pub = self.create_publisher(Int8MultiArray, '/Left_Lamp_driver/lamp_topic', 1)
        self.right_lamp_pub = self.create_publisher(Int8MultiArray, '/Right_Lamp_driver/lamp_topic', 1)
        # Создаем объект для передачи скорости конвейера через топик:
        self.conveyor_pub = self.create_publisher(Float32, '/Conveyor/cmd_vel', 1)
    
    def lamp_turn_on(self, lamp_object, state):
        ''' Передача в ROS-топик массива значений state лампы. '''
        msg = Int8MultiArray()
        msg.data = state
        lamp_object.publish(msg)
    
    def move_conveyor(self, velocity):
        ''' Передача в ROS-топик скорости для конвейера. '''
        msg = Float32()
        msg.data = velocity
        self.conveyor_pub.publish(msg)
    
    def wait_for_secs(self, time_secs: float):
        ''' Ожидание заданного количества секунд. '''
        timer = self.create_timer(time_secs, self.__timer_callback, clock=self.get_clock())
        while not timer.is_ready():
            pass
    
    def __timer_callback(self):
        self.get_logger().info(f'Timer expired!')


def pose_relative_to_palletizer(cube_size, xyz, pall_xyz):
    # Учитываем разницу в координатах x,y,z между паллетайзером и кубиками:
    new_coords = [x - p for x, p in zip(xyz, pall_xyz)]
    # Преобразуем оси кубиков к осям паллетайзера, чтобы он правильно перемещал кубики:
    new_coords[0], new_coords[1] = -new_coords[1], new_coords[0]
    # Прибавляем половину высоты кубиков, т.к. координата z кубика в сцене - это центр кубика:
    new_coords[2] += cube_size/2 + 0.002

    return [new_coords, 0.0]


def main():
    palletizer = PalletizerClient()
    angle = AngleClient()
    client = LampsConveyor()

    ##### Паллетайзер перетаскивает кубики:
    palletizer_xyz  = [0.02, 0.3, 0.74]

    first_cube_xyz  = [-0.03, 0.1, 0.829]
    second_cube_xyz = [-0.08,  0.1, 0.829]
    third_cube_xyz = [-0.13, 0.1, 0.829]

    cube_size = 0.025 # Высота кубиков в сцене cube_size единиц
    obj_poses = [pose_relative_to_palletizer(cube_size, xyz, palletizer_xyz) for xyz in [first_cube_xyz,
                                                                                         second_cube_xyz,
                                                                                         third_cube_xyz]]

    target_poses = copy.deepcopy(obj_poses)
    for p in target_poses:
        p[0][1] += 0.15         # перемещаем кубики вдоль оси Y паллетайзера
        p[0][2] += cube_size    # запас по вертикали

    # Перемещаем кубики между заданными точками с помощью присоски (реализовано в функции grasp_and_move):
    green_blue_red_leds = {0: [0,1,0,0], 1: [1,0,0,0], 2: [0,0,0,1]}
    for i in range(len(obj_poses)):
        # Включаем светодиод под цвет перемещаемого кубика.
        client.lamp_turn_on(client.left_lamp_pub, green_blue_red_leds[i])

        palletizer.grasp_and_move(obj_poses[i], target_poses[i])

        client.lamp_turn_on(client.left_lamp_pub, [0,0,0,0])
    
    ##### Конвейер передвигает кубики:
    client.move_conveyor(-0.05)
    client.wait_for_secs(2.5)
    client.move_conveyor(0.0)
        
    palletizer.destroy_node()
    angle.destroy_node()
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
