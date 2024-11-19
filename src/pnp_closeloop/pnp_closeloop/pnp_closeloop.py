import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray
from std_msgs.msg import String
from ros2_data.action import MoveXYZW
from ros2_data.action import MoveXYZ
from ros2_data.action import MoveG
from ros2_grasping.action import Attacher 
import ast
import time
# from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from rclpy.executors import MultiThreadedExecutor
import os
import json



class PNP_Closeloop(Node):
    def __init__(self):
        super().__init__('pnp_closeloop')
        self.get_logger().info("PNP Closeloop Node is started")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.image_sub

        self.control_pub = self.create_publisher(String, '/robotaction', 10)

        self.home_pose = {'action': 'MoveXYZW', 'value': {'positionx': 0.5, 'positiony': 0.05, 'positionz': 0.6, 'yaw': 45.00, 'pitch': 180.00, 'roll': 0.00}, 'speed': 1.0}
        self.bin_pose = {'action': 'MoveXYZW', 'value': {'positionx': 0.0, 'positiony': -0.5, 'positionz': 0.6, 'yaw': 45.00, 'pitch': 180.00, 'roll': 0.00}, 'speed': 1.0}
        self.attach = {'action': 'Attach', 'value': {'object': 'box', 'endeffector': 'end_effector_frame'}}
        self.dettach = {'action': 'Detach', 'value': {'object': 'box'}}
        self.gripper_open = {'action': 'GripperOpen'}
        self.gripper_close = {'action': 'GripperClose'}

    
    def move_to_home(self):
        self.control_pub.publish(String(data=json.dumps(self.home_pose)))

    def move_to_bin(self):
        self.control_pub.publish(String(data=json.dumps(self.bin_pose)))
    
    def attach(self):
        self.control_pub.publish(String(data=json.dumps(self.attach)))

    def dettach(self):
        self.control_pub.publish(String(data=json.dumps(self.dettach)))

    def gripper_open(self):
        self.control_pub.publish(String(data=json.dumps(self.gripper_open)))

    def gripper_close(self):
        self.control_pub.publish(String(data=json.dumps(self.gripper_close)))

