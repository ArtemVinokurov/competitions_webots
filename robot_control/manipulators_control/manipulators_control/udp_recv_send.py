import rclpy as ros
from rclpy.node import Node

from std_msgs.msg import Int8MultiArray, Int32, Int32MultiArray, Float32, Bool
from manipulator_interfaces.srv import GripperDownPose, GoalPose, GripperCmd, SetInt32, SetInt32Array
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState

from os import path
import csv
import socket
from threading import Thread
import time
from math import pi, isnan

import PySimpleGUI as sg

from .icons import *

devices = {
    'palletizer':       'p',
    'angle':            'g',
    'lamp_1':           'l1',
    'lamp_2':           'l2',
    'conveyor':         's',
    'pall_grip':        'p1',
    'angle_grip':       'g1',
    'angle_button':     'ba',
    'pall_button':      'bp',
    'smart1_button':    'bm1',
    'smart2_button':    'bm2',
    'remote_term_btn':  'br',
}
feedback_str = {
    'movements':    'm',
    'load':         'l',
    'temperatures': 't',
}
FEEDBACK_TIMEOUT = 1.0 # в секундах
GRIPPER_JNAME = 'left_finger_joint_1'

NUM_SERVOS = 6
MAX_POS =   16383         # диапазон сервопривода AR-S430-01
MIN_TEMP = -5
MAX_TEMP =  72
MIN_LOAD = -1000
MAX_LOAD =  1000

CONFIG_CSV = '_PolygonData.csv'
CONFIG_FOLDER = 'lib/manipulators_control'
UDP_COL = 'UDP_KEY'
NAME_COL = 'NAME'

# *************************************************************
COORDS_COEFF = 1.0 / 1000
# *************************************************************


class UdpHandler(Node):
    ''' Класс для управления симуляцией через UDP. '''

    # MAX_UDP_PACKET = 1024
    MAX_UDP_PACKET = 512
    UDP_PORT_IN = 8080
    UDP_PORT_OUT = 8090

    def __init__(self):
        super().__init__('udp_client')

        # UDP socket
        self.sock = None
        self.ip_feedback = '<broadcast>'

        self.servo_def_pos = 0
        self.servo_def_load = 0
        self.servo_def_temp = 30

        self.is_send_user_data = False
        self.is_angle_gripper_on = True
        self.is_pall_gripper_on = False

        self.pall_prev_values = []
        self.angle_prev_values = []
        
        self.pall_cur_joints = None
        self.pall_cur_effort = None
        self.pall_cur_temps = None
        self.pall_user_joints = None
        self.pall_user_effort = None
        self.pall_user_temps = None
        self.pall_idx_servos =          (5, 6, 2, 4)
        self.pall_grip_idx_servos =     (9, 1, 2, 13, -1, 15) # (9, 1, 2, 13, None, 15)
        self.pall_vacuum_pos = '0'
        self.pall_gripper_pos = 0.0
        self.pall_cmd_count = 0
        self.palletizer_name = devices['palletizer']
        # self.pall_grip_name = devices['pall_grip']

        self.angle_cur_joints = None
        self.angle_cur_effort = None
        self.angle_cur_temps = None
        self.angle_user_joints = None
        self.angle_user_effort = None
        self.angle_user_temps = None
        self.angle_idx_servos =         (0, 3, 1, 2, 4)
        self.angle_grip_idx_servos =    (7, 2, 0, 1, 3, 4)
        self.angle_vacuum_pos = '0'
        self.angle_gripper_pos = 0.0
        self.angle_cmd_count = 0
        self.angle_name = devices['angle']
        # self.angle_grip_name = devices['angle_grip']

        self.conveyor_cur_vel = 0.0
        self.conveyor_cmd_count = 0
        self.conveyor_name = devices['conveyor']

        self.left_lamp_state = None
        self.left_lamp_cmd_count = 0
        self.left_lamp_name = devices['lamp_1']
        self.right_lamp_state = None
        self.right_lamp_cmd_count = 0
        self.right_lamp_name = devices['lamp_2']

        self.angle_button_value = 0
        self.pall_button_value = 0
        self.smart1_button_value = 0
        self.smart2_button_value = 0
        self.remote_term_btn_array = [0, 0, 0, 0]
        self.angle_btn_cmd_count = 0
        self.pall_btn_cmd_count = 0
        self.smart1_btn_cmd_count = 0
        self.smart2_btn_cmd_count = 0
        self.remote_term_btn_cmd_count = 0
        self.realtime_btn_count_done = False
        
        self.future_pall = None
        self.future_pall_grip = None
        self.future_pall_gripper = None
        self.future_angle = None
        self.future_angle_grip = None
        self.future_angle_gripper = None

        # Создаем объекты для передачи массива для ламп через топик:
        self.left_lamp_pub = self.create_publisher(Int8MultiArray, '/Left_Lamp_driver/lamp_topic', 10)
        self.right_lamp_pub = self.create_publisher(Int8MultiArray, '/Right_Lamp_driver/lamp_topic', 10)
        # Создаем объект для передачи скорости конвейера через топик:
        self.conveyor_pub = self.create_publisher(Float32, '/Conveyor/cmd_vel', 10)

        # Объекты для управления кнопками через сервисы:
        self.angle_button = self.create_client(SetInt32, '/angle_controller/Button_node/SetState')
        self.pall_button = self.create_client(SetInt32, '/palletizer_controller/Button_node/SetState')
        self.smart1_button = self.create_client(SetInt32, '/smart_1/Button_node/SetState')
        self.smart2_button = self.create_client(SetInt32, '/smart_2/Button_node/SetState')
        self.remote_term_btn = self.create_client(SetInt32Array, '/remote_terminal/Button_node/SetState')

        self.button_request = SetInt32.Request()
        self.button_request_array = SetInt32Array.Request()

        # Создаем клиентов для сервисов управления паллетайзером:
        self.pall_joint = self.create_client(GripperDownPose, '/palletizer/MoveJ')
        self.pall_vacuum_on_off = self.create_client(SetBool, '/palletizer/vacuum_gripper/turn_on')
        while not self.pall_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service is not available")

        # Создаем клиентов для сервисов управления угловым манипулятором:
        self.angle_grip_joint = self.create_client(GoalPose, '/angle/MoveJ')
        self.angle_gripper = self.create_client(GripperCmd, 'angle/gripper_cmd')
        while not self.angle_grip_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service is not available")
        
        self.pall_request = GripperDownPose.Request()
        self.angle_request = GoalPose.Request()

        # Создаем подписки на топики для обратной связи:
        self.create_subscription(JointState, '/palletizer/joint_states',
                                 self.update_pall_joint_state, 10)
        self.create_subscription(Bool, '/palletizer/palletizer/suction_cup/turn_on',
                                 self.update_pall_vacuum, 10)
        self.create_subscription(JointState, '/angle/joint_states',
                                 self.update_angle_joint_state, 10)
        self.create_subscription(Bool, '/angle/Black5dof/suction_cup/turn_on',
                                 self.update_angle_vacuum, 10)
        
        self.create_subscription(Float32, '/Conveyor/cmd_vel',
                                 self.update_conveyor_vel, 10)
        self.create_subscription(Int8MultiArray, '/Left_Lamp_driver/lamp_topic',
                                 self.update_left_lamp, 10)
        self.create_subscription(Int8MultiArray, '/Right_Lamp_driver/lamp_topic',
                                 self.update_right_lamp, 10)
        
        self.create_subscription(Int32, '/angle_controller/Button_node/State',
                                 self.update_angle_button, 10)
        self.create_subscription(Int32, '/palletizer_controller/Button_node/State',
                                 self.update_pall_button, 10)
        self.create_subscription(Int32, '/smart_1/Button_node/State',
                                 self.update_smart1_button, 10)
        self.create_subscription(Int32, '/smart_2/Button_node/State',
                                 self.update_smart2_button, 10)
        self.create_subscription(Int32MultiArray, '/remote_terminal/Button_node/State',
                                 self.update_remote_term_btn, 10)
        
        self.load_param(CONFIG_CSV)
    
    def load_param(self, config_fname: str):
        ''' Загрузка параметров для связи с устройствами по UDP. '''

        # STATE;    KEY;  IP;             PORT;   NAME;      ENTRANCE;   UDP_KEY
        # 10;       M;    192.168.42.10;  8888;   Robot_1;   InOut;      g
        try:
            folder, _ = path.dirname(path.abspath(__file__)).split('/lib/')
            config_file = path.join(folder, CONFIG_FOLDER, config_fname)

            with open(config_file, newline='') as config:
                reader = csv.DictReader(config, delimiter=';')
                # columns = reader.fieldnames
                for row in reader:
                    if row[UDP_COL] in (devices['palletizer'], devices['pall_grip']):
                        self.palletizer_name = row[NAME_COL]
                    elif row[UDP_COL] in (devices['angle'], devices['angle_grip']):
                        self.angle_name = row[NAME_COL]
                    elif row[UDP_COL] == devices['conveyor']:
                        self.conveyor_name = row[NAME_COL]
                    elif row[UDP_COL] == devices['lamp_1']:
                        self.left_lamp_name = row[NAME_COL]
                    elif row[UDP_COL] == devices['lamp_2']:
                        self.right_lamp_name = row[NAME_COL]
        except Exception as e:
            print(e)

    def parse_and_send(self, data: str):
        ''' Прием UDP команды и отправка ее на устройства. '''
        e = 'Неправильный формат!'
        if not data or ':' not in data:
            return
        
        device_type, *cmd = data.split(':')
        if not cmd[-1][-1] or cmd[-1][-1] != '#':
            return
        cmd[-1] = cmd[-1].removesuffix('#')
        
        if device_type in (devices['lamp_1'], devices['lamp_2']):
            if len(cmd) == 4:
                state = list(map(int, cmd))
                if device_type == devices['lamp_1']:
                    self.left_lamp_cmd_count += 1
                    self.lamp_turn_on(self.left_lamp_pub, state)
                else:
                    self.right_lamp_cmd_count += 1
                    self.lamp_turn_on(self.right_lamp_pub, state)
            else:
                print(e, 'UDP команда для Лампы (снизу вверх - синий:зеленый:оранжевый:красный):',
                      f'{devices["lamp_1"]}:0:1:0:1# ({devices["lamp_2"]}:0:0:0:1#)')
        
        elif device_type == devices['conveyor']:
            if len(cmd) == 1:
                velocity = float(cmd[0])
                self.conveyor_cmd_count += 1
                self.move_conveyor(velocity)
            else:
                print(e, f'UDP команда для Конвейера (скорость): {devices["conveyor"]}:-0.05#')
        
        elif device_type == devices['angle_button']:
            if len(cmd) == 1:
                value = int(cmd[0])
                self.angle_btn_cmd_count += 1
                self.set_button(self.angle_button, value)
            else:
                print(e, f'UDP команда для Кнопки Углового манипулятора (значение): {devices["angle_button"]}:0#')
        
        elif device_type == devices['pall_button']:
            if len(cmd) == 1:
                value = int(cmd[0])
                self.pall_btn_cmd_count += 1
                self.set_button(self.pall_button, value)
            else:
                print(e, f'UDP команда для Кнопки Паллетайзера (значение): {devices["pall_button"]}:0#')
        
        elif device_type == devices['smart1_button']:
            if len(cmd) == 1:
                value = int(cmd[0])
                self.smart1_btn_cmd_count += 1
                self.set_button(self.smart1_button, value)
            else:
                print(e, f'UDP команда для Смарт-кнопки 1 (значение): {devices["smart1_button"]}:0#')
        
        elif device_type == devices['smart2_button']:
            if len(cmd) == 1:
                value = int(cmd[0])
                self.smart2_btn_cmd_count += 1
                self.set_button(self.smart2_button, value)
            else:
                print(e, f'UDP команда для Смарт-кнопки 2 (значение): {devices["smart2_button"]}:0#')
        
        elif device_type == devices['remote_term_btn']:
            if len(cmd) == 4:
                state = list(map(int, cmd))
                self.remote_term_btn_cmd_count += 1
                self.set_button_array(self.remote_term_btn, state)
            else:
                print(e, 'UDP команда для Удаленного терминала (останов:неизв.:неизв.:неизв.):',
                      f'{devices["remote_term_btn"]}:0:0:0:0#')
        
        elif device_type == devices['palletizer']:
            if len(cmd) == 4:
                pose = list(map(float, cmd))
                self.pall_cmd_count += 1
                self.pall_send_movej(pose[:-1])
                self.pall_set_vacuum_cmd(pose[-1])
            else:
                print(e, 'UDP команда для Паллетайзера с присоской (x:y:z:насос):',
                      f'{devices["palletizer"]}:0:0:0:0#')
        
        elif device_type == devices['angle_grip']:
            if len(cmd) == 5:
                pose = list(map(float, cmd))
                self.angle_cmd_count += 1
                self.angle_send_movej(pose[:-1])
                self.angle_set_gripper_pos(bool(pose[-1]))
            else:
                print(e, 'UDP команда для Углового манипулятора: (x:y:поворот_схвата:z:схват):',
                      f'{devices["angle_grip"]}:0:0:0:0:0#')
                # print(e, 'UDP команда для Углового манипулятора: (x:y:z:x_orient:y_orient:z_orient:w_orient:схват):',
                #       f'{devices["angle_grip"]}:0:0:0:1:0:0:0:1#')
        
        # Angle UDP command     (x:y:z:насос): "g:0:0:0:0#"
        # Pall_grip UDP command (x:y:z:схват): p1:0:0:0:0#
    
    def pall_send_movej(self, pose: list):
        '''
        Отправка команды движения к целевой позиции на паллетайзер.
        pose = [x, y, z, gripper_angle]
        '''
        if self.future_pall and not self.future_pall.done():
            return
        
        self.pall_request.position = [p * COORDS_COEFF for p in pose]
        # self.pall_request.gripper_angle = pose[-1]
        self.future_pall = self.pall_joint.call_async(self.pall_request)

        return self.future_pall.result()
    
    def pall_set_vacuum_cmd(self, state):
        ''' Отправка команды включения/отключения присоски паллетайзера. '''
        msg = SetBool.Request()
        msg.data = bool(state)
        self.pall_vacuum_on_off.call_async(msg)
    
    def angle_send_movej(self, pose: list):
        '''
        Отправка команды движения к целевой позиции на угловой манипулятор.
        pose = [x, y, поворот_схвата, z]
        '''
        if self.future_angle_grip and not self.future_angle_grip.done():
            return
        
        self.angle_request.position = [pose[i] * COORDS_COEFF for i in [0, 1, 3]]
        self.angle_request.orientation = [0.0, 1.0, 0.0, 0.0] if pose[2]==0 else [1.0, 0.0, 0.0, 0.0]
        # g1:0.1:0.1:0.2:0:1:0:0:0# - движение (поворот схвата по часовой, y_orient=1)
        # g1:0.1:0.1:0.2:1:0:0:0:0# - поворот схвата против часовой, x_orient=1
        self.future_angle_grip = self.angle_grip_joint.call_async(self.angle_request)

        return self.future_angle_grip.result()
    
    def angle_set_gripper_pos(self, state: bool):
        ''' Отправка команды для схвата углового манипулятора. '''
        if self.future_angle_gripper and not self.future_angle_gripper.done():
            return
        
        request = GripperCmd.Request()

        request.goal_pose = 0.9 if state else 0.0
        self.future_angle_gripper = self.angle_gripper.call_async(request)

        return self.future_angle_gripper.result()
    
    def lamp_turn_on(self, lamp_object, state: list):
        ''' Передача в ROS-топик массива значений state лампы. '''
        msg = Int8MultiArray()
        msg.data = state
        lamp_object.publish(msg)
    
    def move_conveyor(self, velocity: float):
        ''' Передача в ROS-топик скорости для конвейера. '''
        msg = Float32()
        msg.data = velocity
        self.conveyor_pub.publish(msg)
    
    def set_button(self, button_client, value: int):
        ''' Передача в ROS-сервис значения кнопки. '''
        self.button_request.data = value

        future_button = button_client.call_async(self.button_request)

        return future_button.result()
    
    def set_button_array(self, button_client, array: Int32MultiArray):
        ''' Передача в ROS-сервис значений кнопки. '''
        self.button_request_array.data = array

        future_button = button_client.call_async(self.button_request_array)

        return future_button.result()
    
    def update_pall_joint_state(self, msg):
        ''' Обновление позиций паллетайзера. '''
        self.pall_cur_joints = msg.position
        self.pall_cur_jnames = msg.name
        # TEMP:
        self.pall_cur_temps = [self.servo_def_temp for _ in range(len(self.pall_cur_joints))]
        # LOAD:
        self.pall_cur_effort = [self.servo_def_load if isnan(x) else x for x in msg.effort]
    
    def update_pall_vacuum(self, msg):
        ''' Обновление состояния присоски паллетайзера. '''
        self.pall_vacuum_pos = '1' if msg.data else '0'
    
    def update_angle_joint_state(self, msg):
        ''' Обновление позиций углового манипулятора. '''
        self.angle_cur_joints = msg.position
        self.angle_cur_jnames = msg.name
        # TEMP:
        self.angle_cur_temps = [self.servo_def_temp for _ in range(len(self.angle_cur_joints))]
        # LOAD:
        self.angle_cur_effort = [self.servo_def_load if isnan(x) else x for x in msg.effort]
        # self.angle_cur_effort = [self.servo_def_load for _ in range(len(self.angle_cur_joints))]
    
    def update_angle_vacuum(self, msg):
        ''' Обновление состояния присоски углового манипулятора. '''
        self.angle_vacuum_pos = '1' if msg.data else '0'
    
    def update_conveyor_vel(self, msg):
        ''' Обновление значения скорости конвейера. '''
        self.conveyor_cur_vel = msg.data
    
    def update_left_lamp(self, msg):
        ''' Обновление состояний светодиодов лампы. '''
        self.left_lamp_state = msg.data
    
    def update_right_lamp(self, msg):
        ''' Обновление состояний светодиодов лампы. '''
        self.right_lamp_state = msg.data
    
    def update_angle_button(self, msg):
        ''' Обновление состояния кнопки. '''
        self.angle_button_value = msg.data
    
    def update_pall_button(self, msg):
        ''' Обновление состояния кнопки. '''
        self.pall_button_value = msg.data
    
    def update_smart1_button(self, msg):
        ''' Обновление состояния кнопки. '''
        self.smart1_button_value = msg.data
    
    def update_smart2_button(self, msg):
        ''' Обновление состояния кнопки. '''
        self.smart2_button_value = msg.data
    
    def update_remote_term_btn(self, msg):
        ''' Обновление состояния кнопки. '''
        self.remote_term_btn_array = msg.data
    
    def set_user_inputs(self, gui_object, robot_name):
        '''
        Ограничение введенных положений, температур и нагрузок для сервоприводов.
        Запись/установка значений для отправки по UDP.
        '''
        if self.pall_cur_joints and robot_name==gui_object.robot_list[0]: # 'Паллетайзер'
            self.pall_user_joints = [None for _ in range(NUM_SERVOS)]
            self.pall_user_temps = [None for _ in range(NUM_SERVOS)]
            self.pall_user_effort = [None for _ in range(NUM_SERVOS)]
        elif self.angle_cur_joints and robot_name==gui_object.robot_list[1]: # 'Угловой'
            self.angle_user_joints = [None for _ in range(NUM_SERVOS)]
            self.angle_user_temps = [None for _ in range(NUM_SERVOS)]
            self.angle_user_effort = [None for _ in range(NUM_SERVOS)]
        
        for name, (vmin, vmax) in zip(('POS', 'TEMP', 'LOAD'),
                                      ((0, MAX_POS), (MIN_TEMP, MAX_TEMP), (MIN_LOAD, MAX_LOAD))):
            for i in range(NUM_SERVOS):
                if gui_object.window[f'-{name}{i}-'].Widget['state'] == 'readonly':
                    continue

                str_input = gui_object.window[f'-{name}{i}-'].get()
                try:
                    val = int(str_input)
                    value = vmin if val<vmin else vmax if val>vmax else val
                    gui_object.window[f'-{name}{i}-'].update(value)
                    if robot_name == gui_object.robot_list[0]: # 'Паллетайзер'
                        if name == 'POS':
                            self.pall_user_joints[i] = value
                        elif name == 'TEMP':
                            self.pall_user_temps[i] = value
                        elif name == 'LOAD':
                            self.pall_user_effort[i] = value
                    elif robot_name == gui_object.robot_list[1]: # 'Угловой'
                        if name == 'POS':
                            self.angle_user_joints[i] = value
                        elif name == 'TEMP':
                            self.angle_user_temps[i] = value
                        elif name == 'LOAD':
                            self.angle_user_effort[i] = value
                except ValueError:
                    pass
    
    def load_prev_values(self, gui_object, robot_name):
        ''' Обновление полей ввода до предыдущих значений при переключении выпадающего списка. '''
        # Save current input values:
        saved_values = self.angle_prev_values if robot_name==gui_object.robot_list[0] else self.pall_prev_values
        if not saved_values:
            for name in ('POS', 'TEMP', 'LOAD'):
                saved_values.append([gui_object.window[f'-{name}{i}-'].get() for i in range(NUM_SERVOS)])
        else:
            for idx, name in enumerate(('POS', 'TEMP', 'LOAD')):
                for i in range(NUM_SERVOS):
                    saved_values[idx][i] = gui_object.window[f'-{name}{i}-'].get()
        # Load previous input values for robot_name:
        values_to_load = self.pall_prev_values if robot_name==gui_object.robot_list[0] else self.angle_prev_values
        if values_to_load:
            for idx, name in enumerate(('POS', 'TEMP', 'LOAD')):
                for i in range(NUM_SERVOS):
                    gui_object.window[f'-{name}{i}-'].update(values_to_load[idx][i])

    def show_cur_values(self, gui_object, robot_name):
        ''' Отображение текущих значений сервоприводов в полях ввода. '''
        pall_idx_send = self.pall_grip_idx_servos if GRIPPER_JNAME in self.pall_cur_jnames else self.pall_idx_servos
        angle_idx_send = self.angle_grip_idx_servos if GRIPPER_JNAME in self.angle_cur_jnames else self.angle_idx_servos
        joints = []
        # Паллетайзер:
        if self.pall_cur_joints and robot_name==gui_object.robot_list[0]:
            joints = self.joint2tick(self.pall_cur_joints[i] for i in pall_idx_send)
        # Угловой:
        elif self.angle_cur_joints and robot_name==gui_object.robot_list[1]:
            joints = self.joint2tick(self.angle_cur_joints[i] for i in angle_idx_send)
        
        for name, (vmin, vmax) in zip(('POS', 'TEMP', 'LOAD'),
                                      ((0, MAX_POS), (MIN_TEMP, MAX_TEMP), (MIN_LOAD, MAX_LOAD))):
            for i in range(NUM_SERVOS):
                if gui_object.window[f'-{name}{i}-'].Widget['state']=='readonly' or i>=len(joints):
                    continue

                value = None
                if self.pall_cur_joints and robot_name==gui_object.robot_list[0]: # 'Паллетайзер'
                    if name == 'POS':
                        value = joints[i]
                    elif name == 'TEMP':
                        value = self.pall_cur_temps[pall_idx_send[i]]
                    elif name == 'LOAD':
                        value = self.pall_cur_effort[pall_idx_send[i]]
                elif self.angle_cur_joints and robot_name==gui_object.robot_list[1]: # 'Угловой'
                    if name == 'POS':
                        value = joints[i]
                    elif name == 'TEMP':
                        value = self.angle_cur_temps[angle_idx_send[i]]
                    elif name == 'LOAD':
                        value = self.angle_cur_effort[angle_idx_send[i]]
                
                if value is not None: # we check for None, because load=0 can be
                    gui_object.window[f'-{name}{i}-'].update(value)
    
    def print_udp_cmd(self, addr_port, in_out, msg):
        ''' Печать в консоль адреса/порта, времени, направления и команды. '''
        time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(time_str, addr_port, in_out, msg)
    
    def joint2tick(self, joints):
        ''' Перевод радиан в единицы положения сервоприводов. '''
        return [int(i * MAX_POS/2/pi + MAX_POS/2) for i in joints]
    
    def create_and_send_udp(self, _type, name, count, status='', data=[]):
        ''' Формирует строку из текущих положений устройства и отправляет по UDP. '''
        if status:
            msg = ':'.join(str(i) for i in [_type, name, status, count, *data]) + '#'
        else:
            msg = ':'.join(str(i) for i in [_type, name, count, *data]) + '#'
        
        self.sock.sendto(msg.encode(), (self.ip_feedback, self.UDP_PORT_OUT))
        # self.print_udp_cmd(self.UDP_PORT_OUT, '<<', msg) # DEBUG

    def send_feedback(self):
        ''' Отправка положений устройств по UDP в заданном формате. '''
        # 'left/right_finger_joint_1' = схват
        # Palletizer: rotate_column = 1, first_cont_1 = 2, first_cont_2 = 3, gripper_rot = 4

        # pall_cur_jnames = 5,6,2,4:
        #     ['base_to_tr', 'first_cont_3', 'first_cont_2', 'from_tr_to_last', 'gripper_rot', 'rotate_column',
        #      'first_cont_1', 'first_cont_to_tr', 'second_cont_1', 'second_cont_2']

        # pall_grip_cur_jnames = 9,1,2,13,_,15:
        #     ['base_to_tr', 'first_cont_1', 'first_cont_2', 'first_cont_to_tr', 'right_support_joint',
        #      'right_finger_joint_3', 'right_finger_joint_2', 'first_cont_3', 'second_cont_2', 'rotate_column',
        #      'right_finger_joint_1', 'left_support_joint', 'left_finger_joint_3', 'gripper_rot',
        #      'left_finger_joint_2', 'left_finger_joint_1', 'second_cont_1', 'from_tr_to_last']


        # angle_grip_cur_jnames = 7,2,0,1,3,4:
        #     ['Joint_3', 'Joint_4', 'Joint_2', 'Joint_5', 'left_finger_joint_1', 'left_finger_joint_2',
        #      'left_finger_joint_3', 'Joint_1', 'left_support_joint', 'right_finger_joint_1',
        #      'right_finger_joint_2', 'right_finger_joint_3', 'right_support_joint']

        # angle_vacuum_cur_jnames = 0,3,1,2,4:
        #     ['Joint_1', 'Joint_3', 'Joint_4', 'Joint_2', 'Joint_5']
        
        pall_idx_send = self.pall_grip_idx_servos if GRIPPER_JNAME in self.pall_cur_jnames else self.pall_idx_servos
        angle_idx_send = self.angle_grip_idx_servos if GRIPPER_JNAME in self.angle_cur_jnames else self.angle_idx_servos
        
        ##### Паллетайзер:
        if self.pall_cur_joints:
            moving = '0'
            if self.future_pall and not self.future_pall.done():
                moving = '1'
            
            # Позиции в тиках диапазона сервопривода:
            joints = self.joint2tick(self.pall_cur_joints)
            joints = [joints[i] for i in pall_idx_send]
            if self.is_send_user_data:
                for i in range(len(joints)):
                    if self.pall_user_joints and self.pall_user_joints[i]:
                        joints[i] = self.pall_user_joints[i]
            if GRIPPER_JNAME in self.pall_cur_jnames:
                udp_name = devices['pall_grip']
                joints[-2] = joints[-1]
                joints = joints[:-1]
                if not self.is_pall_gripper_on:
                    joints = joints[:-1]
            else:
                udp_name = devices['palletizer']
                # Добавляем состояние присоски
                joints += [self.pall_vacuum_pos]
            self.create_and_send_udp(udp_name + feedback_str['movements'],
                                     udp_name,
                                     self.pall_cmd_count,
                                     moving,
                                     joints)
            
            temps = [self.pall_cur_temps[i] for i in pall_idx_send]
            if self.is_send_user_data:
                for i in range(len(temps)):
                    if self.pall_user_temps and self.pall_user_temps[i]:
                        temps[i] = self.pall_user_temps[i]
            if GRIPPER_JNAME in self.pall_cur_jnames:
                temps[-2] = temps[-1]
                temps = temps[:-1]
                if not self.is_pall_gripper_on:
                    temps = temps[:-1]
            self.create_and_send_udp(udp_name + feedback_str['temperatures'],
                                     udp_name,
                                     self.pall_cmd_count,
                                     moving,
                                     temps)
            
            loads = [self.pall_cur_effort[i] for i in pall_idx_send]
            if self.is_send_user_data:
                for i in range(len(loads)):
                    if self.pall_user_effort and self.pall_user_effort[i]:
                        loads[i] = self.pall_user_effort[i]
            if GRIPPER_JNAME in self.pall_cur_jnames:
                loads[-2] = loads[-1]
                loads = loads[:-1]
                if not self.is_pall_gripper_on:
                    loads = loads[:-1]
            self.create_and_send_udp(udp_name + feedback_str['load'],
                                     udp_name,
                                     self.pall_cmd_count,
                                     moving,
                                     loads)

        ##### Угловой со схватом:
        if self.angle_cur_joints:
            moving = '0'
            if self.future_angle_grip and not self.future_angle_grip.done():
                moving = '1'

            # Позиции в тиках диапазона сервопривода:
            joints = self.joint2tick(self.angle_cur_joints)
            # joints = [joints[i] for i in self.angle_grip_idx_servos] + [self.angle_gripper_pos]
            joints = [joints[i] for i in angle_idx_send]
            if self.is_send_user_data:
                for i in range(len(joints)):
                    if self.angle_user_joints and self.angle_user_joints[i]:
                        joints[i] = self.angle_user_joints[i]
            if GRIPPER_JNAME in self.angle_cur_jnames:
                udp_name = devices['angle_grip']
                if not self.is_angle_gripper_on:
                    joints = joints[:-1]
            else:
                udp_name = devices['angle']
                # Добавляем состояние присоски
                joints += [self.angle_vacuum_pos]
            self.create_and_send_udp(udp_name + feedback_str['movements'],
                                     udp_name,
                                     self.angle_cmd_count,
                                     moving,
                                     joints)
            
            temps = [self.angle_cur_temps[i] for i in self.angle_grip_idx_servos]
            if self.is_send_user_data:
                for i in range(len(temps)):
                    if self.angle_user_temps and self.angle_user_temps[i]:
                        temps[i] = self.angle_user_temps[i]
            if GRIPPER_JNAME in self.angle_cur_jnames and not self.is_angle_gripper_on:
                temps = temps[:-1]
            self.create_and_send_udp(udp_name + feedback_str['temperatures'],
                                     udp_name,
                                     self.angle_cmd_count,
                                     moving,
                                     temps)
            
            loads = [self.angle_cur_effort[i] for i in self.angle_grip_idx_servos]
            if self.is_send_user_data:
                for i in range(len(loads)):
                    if self.angle_user_effort and self.angle_user_effort[i]:
                        loads[i] = self.angle_user_effort[i]
            if GRIPPER_JNAME in self.angle_cur_jnames and not self.is_angle_gripper_on:
                loads = loads[:-1]
            self.create_and_send_udp(udp_name + feedback_str['load'],
                                     udp_name,
                                     self.angle_cmd_count,
                                     moving,
                                     loads)

        self.create_and_send_udp(devices['conveyor'], devices['conveyor'],
                                 self.conveyor_cmd_count,
                                 data=[round(self.conveyor_cur_vel, 3)])

        if self.left_lamp_state:
            self.create_and_send_udp(devices['lamp_1'], devices['lamp_1'],
                                     self.left_lamp_cmd_count,
                                     data=self.left_lamp_state)

        if self.right_lamp_state:
            self.create_and_send_udp(devices['lamp_2'], devices['lamp_2'],
                                     self.right_lamp_cmd_count,
                                     data=self.right_lamp_state)
        
        # Buttons:
        self.create_and_send_udp(devices['angle_button'], devices['angle_button'],
                                 self.angle_btn_cmd_count,
                                 data=[self.angle_button_value])
        self.create_and_send_udp(devices['pall_button'], devices['pall_button'],
                                 self.pall_btn_cmd_count,
                                 data=[self.pall_button_value])
        self.create_and_send_udp(devices['smart1_button'], devices['smart1_button'],
                                 self.smart1_btn_cmd_count,
                                 data=[self.smart1_button_value])
        self.create_and_send_udp(devices['smart2_button'], devices['smart2_button'],
                                 self.smart2_btn_cmd_count,
                                 data=[self.smart2_button_value])
        self.create_and_send_udp(devices['remote_term_btn'], devices['remote_term_btn'],
                                 self.remote_term_btn_cmd_count,
                                 data=self.remote_term_btn_array)

    def read_udp(self):
        ''' Считывание UDP данных и отправка на устройства симуляции. '''
        while True:
            udp_data, udp_client = self.sock.recvfrom(self.MAX_UDP_PACKET)
            self.ip_feedback = udp_client[0] # (IP, port)
            data = udp_data.decode('utf-8')
            self.print_udp_cmd(udp_client, '>>', data)
            self.parse_and_send(data)

            time.sleep(0.001)

    def udp_mainloop(self, ip, port: int):
        '''
        Принимает UDP команды. Управляет устройствами полигона в отдельном процессе.
        Отсылает положения устройств по UDP.
        '''
        # Создаем UDP сокет и связываем его с портом port для прослушивания:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((ip, port))
        
        # Считываем UDP команды в отдельном процессе. Он завершится при выходе из main(), т.к. daemon=True 
        read_thread = Thread(target=self.read_udp, daemon=True)
        read_thread.start()

        gui = ButtonsGUI()
        # default_color = gui.window['-ANGLE-'].Widget['background']

        start_gui_time = time.time()

        self.timer = self.create_timer(FEEDBACK_TIMEOUT, self.send_feedback)

        try:
            # Обработка событий в цикле
            while True:
                ros.spin_once(self, timeout_sec=0.1)

                if not gui.show_cur_values_done and time.time()-start_gui_time > 1:
                    self.show_cur_values(gui, gui.window['-COMBO-'].get())
                    gui.show_cur_values_done = True

                event = ''
                
                if not gui.window_closed:
                    event, values = gui.window.read(timeout=50)

                if event in (sg.WIN_CLOSED, '-QUIT-'):
                    # break
                    gui.window_closed = True
                    gui.window.close()
                
                elif event == '-ANGLE-':
                    new_value = int(not self.angle_button_value)
                    self.angle_button_value = new_value
                    self.set_button(self.angle_button, new_value)
                    self.angle_btn_cmd_count += 1
                    # color = ('black', 'orange red') if new_value==1 else default_color
                    # gui.window['-ANGLE-'].update(button_color=color)
                    gui.window['-ANGLE-'].update(image_data=icon_btn_red120 if new_value==1 else icon_btn_grey_red120)
                
                elif event == '-PALL-':
                    new_value = int(not self.pall_button_value)
                    self.pall_button_value = new_value
                    self.set_button(self.pall_button, new_value)
                    self.pall_btn_cmd_count += 1
                    gui.window['-PALL-'].update(image_data=icon_btn_red120 if new_value==1 else icon_btn_grey_red120)
                
                elif event == '-SMART_1-':
                    if self.smart1_button_value != 1:
                        self.set_button(self.smart1_button, 1)
                    if not self.realtime_btn_count_done:
                        self.smart1_btn_cmd_count += 1
                        self.realtime_btn_count_done = True
                    gui.window['-SMART_1-'].update(image_data=icon_btn_green80)
                
                elif event == '-SMART_2-':
                    if self.smart2_button_value != 1:
                        self.set_button(self.smart2_button, 1)
                    if not self.realtime_btn_count_done:
                        self.smart2_btn_cmd_count += 1
                        self.realtime_btn_count_done = True
                    gui.window['-SMART_2-'].update(image_data=icon_btn_green80)
                
                elif event == '-REMOTE_STOP-':
                    new_value = int(not self.remote_term_btn_array[0])
                    self.remote_term_btn_array[0] = new_value
                    self.set_button_array(self.remote_term_btn, self.remote_term_btn_array)
                    self.remote_term_btn_cmd_count += 1
                    gui.window['-REMOTE_STOP-'].update(image_data=icon_btn_red120 if new_value==1 else icon_btn_grey_red120)
                
                elif event in ('-REMOTE_2-', '-REMOTE_3-', '-REMOTE_4-'):
                    index = 1 + ('-REMOTE_2-', '-REMOTE_3-', '-REMOTE_4-').index(event)
                    if self.remote_term_btn_array[index] != 1:
                        self.remote_term_btn_array[index] = 1
                        self.set_button_array(self.remote_term_btn, self.remote_term_btn_array)
                    if not self.realtime_btn_count_done:
                        self.remote_term_btn_cmd_count += 1
                        self.realtime_btn_count_done = True
                    gui.window[event].update(image_data=[icon_btn_yellow64, icon_btn_red64, icon_btn_green64][index-1])
                
                elif event == '-COMBO-':
                    true_if_palletizer = values[event]==gui.robot_list[0]
                    for i in range(4, NUM_SERVOS):
                        for name in ('POS', 'TEMP', 'LOAD'):
                            gui.window[f'-{name}{i}-'].update(disabled = true_if_palletizer)
                    if values[event] == gui.robot_list[0]: # 'Паллетайзер'
                        gui.window['-GRIP-'].update(value = self.is_pall_gripper_on)
                    elif values[event] == gui.robot_list[1]:
                        gui.window['-GRIP-'].update(value = self.is_angle_gripper_on)
                    # Изменение активности поля ввода GRIP:
                    gripper_on = self.is_pall_gripper_on if true_if_palletizer else self.is_angle_gripper_on
                    for name in ('POS', 'TEMP', 'LOAD'):
                        gui.window[f'-{name}{NUM_SERVOS-1}-'].update(disabled = not gripper_on)
                    # self.show_cur_values(gui, values[event])
                    self.load_prev_values(gui, values[event])
                
                elif event == '-GRIP-':
                    value = values[event]
                    cur_robot = gui.window['-COMBO-'].get()
                    if cur_robot == gui.robot_list[0]: # 'Паллетайзер'
                        self.is_pall_gripper_on = value
                    elif cur_robot == gui.robot_list[1]:
                        self.is_angle_gripper_on = value
                    # Изменение активности поля ввода GRIP:
                    for name in ('POS', 'TEMP', 'LOAD'):
                        gui.window[f'-{name}{NUM_SERVOS-1}-'].update(disabled = not value)

                elif event[:4] == '-POS':
                    value = values[event]
                    if len(value) > 5:
                        val = value[:5] # [0, 16383]
                        gui.window[event].update(val)
                
                elif event[:5] == '-TEMP':
                    value = values[event]
                    if len(value) > 2:
                        val = value[:2] # [-5, 72]
                        gui.window[event].update(val)
                
                elif event[:5] == '-LOAD':
                    value = values[event]
                    if len(value.replace('-', '')) > 4:
                        val = value[:5 if value[0]=='-' else 4] # [-1000, 1000]
                        gui.window[event].update(val)
                
                elif event == '-APPLY-':
                    if self.pall_cur_joints or self.angle_cur_joints:
                        self.set_user_inputs(gui, gui.window['-COMBO-'].get())
                        self.is_send_user_data = True
                
                elif event == '-CUR_VALUES-':
                    self.is_send_user_data = False
                    self.show_cur_values(gui, gui.window['-COMBO-'].get())
                
                if not gui.window_closed:
                    # Возвращаем кнопки без фиксации в "ноль":
                    if event not in ('-SMART_1-', '-SMART_2-', '-REMOTE_2-', '-REMOTE_3-', '-REMOTE_4-'):
                        self.realtime_btn_count_done = False
                    if event != '-SMART_1-':
                        gui.window['-SMART_1-'].update(image_data=icon_btn_grey_green80)
                        if self.smart1_button_value != 0:
                            self.set_button(self.smart1_button, 0)
                        self.smart1_button_value = 0
                    if event != '-SMART_2-':
                        gui.window['-SMART_2-'].update(image_data=icon_btn_grey_green80)
                        if self.smart2_button_value != 0:
                            self.set_button(self.smart2_button, 0)
                        self.smart2_button_value = 0
                    if self.remote_term_btn_array:
                        for idx, icon in zip(range(2, 5), [icon_btn_grey_yellow64,
                                                           icon_btn_grey_red64,
                                                           icon_btn_grey_green64]):
                            if event != f'-REMOTE_{idx}-':
                                gui.window[f'-REMOTE_{idx}-'].update(image_data=icon)
                                if self.remote_term_btn_array[idx-1] != 0:
                                    self.remote_term_btn_array[idx-1] = 0
                                    self.set_button_array(self.remote_term_btn, self.remote_term_btn_array)
                
                time.sleep(0.001)
        
        except KeyboardInterrupt:
            print('\nFinished.\n')
        
        except Exception as e:
            print(e)
        
        gui.window.close()
        self.sock.close()


class ButtonsGUI:
    ''' Класс интерфейса для управления кнопками. '''

    sg.theme('default 1')
    
    GUI_WIDTH =  1100
    GUI_HEIGHT = 540
    MARGIN_X = 20
    MARGIN_Y = 20

    POS_INPUT_WIDTH =  5
    TEMP_INPUT_WIDTH = 5
    LOAD_INPUT_WIDTH = 5

    robot_list = ['Паллетайзер', 'Угловой']

    def __init__(self):
        self.layout = [[]]
        self.window_closed = False
        self.show_cur_values_done = False

        self.create_widgets()

        self.window = sg.Window(title = 'Управление кнопками полигона',
                                layout = self.layout,
                                size = (self.GUI_WIDTH, self.GUI_HEIGHT),
                                margins = (2*self.MARGIN_X, self.MARGIN_Y),
                                icon = ICON,
                                resizable = False,
                                finalize = True)
        
        self.window['-BLANK_TEXT_TEMP-'].update(visible=False)
    
    def create_widgets(self):
        ''' Наполнение окна виджетами. '''
        labels = {
            'POS':  'Положение:',
            'TEMP': 'Температура, C:',
            'LOAD': 'Нагрузка, 0.1%:',
        }
        maxlen_labels = max(map(len, labels.values()))
        
        self.layout = [
            [sg.Push(),
             sg.Frame('', pad=(self.MARGIN_X, (0, self.MARGIN_Y)), layout=[
                 [sg.Button('Угловой',
                            image_data = icon_btn_grey_red120,
                            border_width = 0,
                            pad = (self.MARGIN_X/2, self.MARGIN_Y/2),
                            font = ('Arial', 12, 'bold'),
                            key = '-ANGLE-')]]),
             sg.Frame('', pad=(self.MARGIN_X, (0, self.MARGIN_Y)), layout=[
                 [sg.Button('Паллетайзер',
                            image_data = icon_btn_grey_red120,
                            border_width = 0,
                            pad = (self.MARGIN_X/2, self.MARGIN_Y/2),
                            font = ('Arial', 11, 'bold'),
                            key = '-PALL-')]]),
             sg.Frame('', pad=(self.MARGIN_X, (0, self.MARGIN_Y)), layout=[
                 [sg.RealtimeButton('Смарт 1',
                                    image_data = icon_btn_grey_green80,
                                    border_width = 0,
                                    pad = (self.MARGIN_X/2, self.MARGIN_Y/2),
                                    font = ('Arial', 10, 'bold'),
                                    key = '-SMART_1-')]]),
             sg.Frame('', pad=(0, (0, self.MARGIN_Y)), layout=[
                 [sg.RealtimeButton('Смарт 2',
                                    image_data = icon_btn_grey_green80,
                                    border_width = 0,
                                    pad = (self.MARGIN_X/2, self.MARGIN_Y/2),
                                    font = ('Arial', 10, 'bold'),
                                    key = '-SMART_2-')]]),
             sg.Push(),],
            
            [sg.Frame('Удаленный терминал:',
                      pad = (self.MARGIN_X, 0),
                      layout = [
                        [sg.Button('Останов',
                                   image_data = icon_btn_grey_red120,
                                   border_width = 0,
                                   pad = (self.MARGIN_X, (self.MARGIN_Y/2, 0)),
                                   font = ('Arial', 12, 'bold'),
                                   key = '-REMOTE_STOP-'),],
                        [sg.RealtimeButton('2',
                                           image_data = icon_btn_grey_yellow64,
                                           border_width = 0,
                                           pad = (2*self.MARGIN_X, 0),
                                           font = ('Arial', 14, 'bold'),
                                           key = '-REMOTE_2-'),],
                        [sg.RealtimeButton('3',
                                           image_data = icon_btn_grey_red64,
                                           border_width = 0,
                                           pad = ((5*self.MARGIN_X, self.MARGIN_X/2), 0),
                                           font = ('Arial', 14, 'bold'),
                                           key = '-REMOTE_3-'),
                         sg.RealtimeButton('4',
                                           image_data = icon_btn_grey_green64,
                                           border_width = 0,
                                           pad = (0, 0),
                                           font = ('Arial', 14, 'bold'),
                                           key = '-REMOTE_4-'),],
                      ]),

             sg.Frame('Сервоприводы:',
                      pad = ((self.MARGIN_X, 0), 0),
                      layout=[
                        [sg.Push(),
                         sg.Combo(self.robot_list,
                                  default_value = self.robot_list[1],
                                  pad = (0, self.MARGIN_Y),
                                  size = (20, 1),
                                  font = ('Arial', 11, 'bold'),
                                  auto_size_text = True,
                                  key = '-COMBO-',
                                  readonly = True,
                                  enable_events = True),
                         sg.Push(),],
                        
                        [sg.Push(),
                         sg.Checkbox('Схват',
                                     pad = (self.MARGIN_X/2, 0),
                                     key = '-GRIP-',
                                     default = True if self.robot_list[1]=='Угловой' else False,
                                     enable_events = True)],

                        [sg.pin(sg.Text(max(labels.values(), key=len),
                                        pad = ((self.MARGIN_X, 0), (self.MARGIN_Y/3, 0)),
                                        font = ('Arial', 10),
                                        key = '-BLANK_TEXT_TEMP-'),
                                        shrink = False)
                        ] + [sg.Text(str(i+1),
                                     pad = (((2.8-0.6*(i==0))*self.MARGIN_X, 0), (self.MARGIN_Y/3, 0)),
                                     font = ('Arial', 8))
                             for i in range(NUM_SERVOS)],
                        
                        [sg.Text(labels['POS'] + ' '*(maxlen_labels-len(labels['POS'])),
                                 pad = ((self.MARGIN_X, self.MARGIN_X/2), (0, self.MARGIN_Y/2)),
                                 font = ('Arial', 9),)
                        ] + [sg.Input('',
                                      size = (self.POS_INPUT_WIDTH, 1),
                                      justification = 'right',
                                      pad = (self.MARGIN_X/2, (0, self.MARGIN_Y/2)),
                                      font = ('Arial', 9, 'bold'),
                                      key = f'-POS{i}-',
                                      enable_events = True)
                             for i in range(NUM_SERVOS)],
                        
                        [sg.Text(labels['TEMP'] + ' '*(maxlen_labels-len(labels['TEMP'])),
                                 pad = ((self.MARGIN_X, 0), (0, self.MARGIN_Y/2)),
                                 font = ('Arial', 9),)
                        ] + [sg.Input('',
                                      size = (self.TEMP_INPUT_WIDTH, 1),
                                      justification = 'right',
                                      pad = (self.MARGIN_X/2, (0, self.MARGIN_Y/2)),
                                      font = ('Arial', 9, 'bold'),
                                      key = f'-TEMP{i}-',
                                      enable_events = True)
                             for i in range(NUM_SERVOS)],
                        
                        [sg.Text(labels['LOAD'] + ' '*(maxlen_labels-len(labels['LOAD'])),
                                 pad = ((self.MARGIN_X, self.MARGIN_X/4), 0),
                                 font = ('Arial', 9),)
                        ] + [sg.Input('',
                                      size = (self.LOAD_INPUT_WIDTH, 1),
                                      justification = 'right',
                                      pad = (self.MARGIN_X/2, 0),
                                      font = ('Arial', 9, 'bold'),
                                      key = f'-LOAD{i}-',
                                      enable_events = True)
                             for i in range(NUM_SERVOS)],
                        
                        [sg.Push(),
                         sg.Button('Текущие\nзначения',
                                   pad = (0, (self.MARGIN_Y/2, 0)),
                                   font = ('Arial', 5),
                                   key = '-CUR_VALUES-')],
                        [sg.Push(),
                         sg.Button('Применить',
                                   pad = (0, (self.MARGIN_Y/2, self.MARGIN_Y)),
                                   font = ('Arial', 11, 'bold'),
                                   key = '-APPLY-'),
                         sg.Push(),],
                      ])],
        ]


def main():
    if not ros.ok():
        ros.init()
    
    app = UdpHandler()
    
    app.udp_mainloop('', app.UDP_PORT_IN) # bind to all interfaces at given port
        
    app.destroy_node()
    # ros.try_shutdown()

if __name__ == '__main__':
    main()
