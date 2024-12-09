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
from gazebo_msgs.msg import ModelStates
import sys
from threading import Thread
from collections import deque
from multiprocessing import Process
from grasp_gen_interface.srv import GraspGen

sys.path.append('/home/vishwas/Conveyor/src/pandarobotmove/pandarobotmove')

# from panda_actions import MoveXYZWclient, MoveGclient, MoveXYZclient, ATTACHERclient, DetacherPUB, CubePoseSub
from panda_actions import CloseLoopRobot
from conveyorbelt_msgs.srv import ConveyorBeltControl


RES = "null"
CENTERS = None
CAMERA = 'overhead'
object_queue = deque()
COMPLETED = []


class OverheadCamSub(Node):
    def __init__(self):
        super().__init__('overhead_cam_subscriber')
        self.subscription = self.create_subscription(
            String,
            'object_center/overhead',
            self.listener_callback,
            10)
        self.subscription
    
    def listener_callback(self, msg):
        global object_queue
        # print("Received message")
        data = msg.data

        # convert the json string to dictionary
        data_dict = json.loads(data)

        exists = any(obj.get('id') == data_dict['id'] for obj in object_queue)

        if not exists and data_dict['id'] not in COMPLETED:
            object_queue.append(data_dict)
            print(f"Object {data_dict['id']} detected at center {data_dict['center']}")
            global CENTERS
            CENTERS = data_dict['center']

class OverHeadCamYoloSub(Node):
    def __init__(self):
        super().__init__('overhead_cam_yolo_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/yolo/overhead',
            self.listener_callback,
            10)
        self.subscription
    
    def listener_callback(self, msg):
        global object_queue
        # print("Received message")
        data = msg.data

        # convert the json string to dictionary
        data_dict = json.loads(data)
        data_dict = data_dict['detections']

        """
        Message format:
        [detection = {
                        "id": str(track_id),  # Assign unique ID for each object
                        "class": cls,
                        "bbox": [x_min, y_min, x_max, y_max],
                        "info": {'center':(world_x, world_y), 'width': world_w, 'height': world_h, 'angle': 0},
                    },
       {
                        "id": str(track_id),  # Assign unique ID for each object
                        "class": cls,
                        "bbox": [x_min, y_min, x_max, y_max],
                        "info": {'center':(world_x, world_y), 'width': world_w, 'height': world_h, 'angle': 0},
                    }]
        """

        for obj in data_dict:
            exists = any(obj.get('id') == obj['id'] for obj in object_queue)

            if not exists and obj['id'] not in COMPLETED:
                object_queue.append(obj)
                print(f"Object {obj['id']} detected at center {obj['info']['center']}")

class InHandCamSub(Node):
    def __init__(self):
        super().__init__('inhand_cam_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/yolo/inhand',
            self.listener_callback,
            10)
        self.subscription
        self.data = None
    
    def listener_callback(self, msg):
        # print("Received message")
        data = msg.data

        # convert the json string to dictionary
        data_dict = json.loads(data)
        self.data = data_dict
    
    def get_object_data(self):
        return self.data
    
class InHandCamEdgeSub(Node):
    def __init__(self):
        super().__init__('inhand_cam_edge_subscriber')
        self.subscription = self.create_subscription(String, 'object_center/inhand', self.listener_callback, 10)
        self.subscription
        self.data = None
    
    def listener_callback(self, msg):
        data = msg.data
        data_dict = json.loads(data)
        self.data = data_dict
    
    def get_object_data(self):
        return self.data

class ConveyorPowerClient(Node):
    def __init__(self):
        super().__init__('conveyor_power_client')
        self.client = self.create_client(ConveyorBeltControl, '/CONVEYORPOWER')
        
        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Service is now available, sending request...')
        
        # Create the request
        request = ConveyorBeltControl.Request()
        request.power = 5.0  # Set the desired power level
        
        # Send the request and get the response
        # future = self.client.call_async(request)
        # future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Conveyor power changed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def send_request(self, power):
        request = ConveyorBeltControl.Request()
        request.power = power
        future = self.client.call_async(request)
        return future


class GraspClient(Node):
    def __init__(self):
        super().__init__('grasp_client')
        self.client = self.create_client(GraspGen, 'generate_grasp')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to be available...')
        
        self.get_logger().info('Service is now available.')

        self.gs = None
        self.request = GraspGen.Request()
    
    def send_request(self, grasp_type):
        
        self.request.input = grasp_type
        # Call the service asynchronously
        future = self.client.call_async(self.request)
        return future
    
    def response_callback(self, future):
        try:
            # print("Received response")
            response = future.result()
            grasp_rectangle = response.grasp.data
            self.get_logger().info(f'Received grasp rectangle: {grasp_rectangle}')
        except Exception as e:
            self.get_logger().error(f'Error while calling service: {str(e)}')

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.timer = self.create_timer(1, self.timer_callback)

        self.execution = True
    
    def timer_callback(self):
        if self.execution:
            self.controls()
            self.execution = False
            self.timer.cancel()
        else:
            print("Execution stopped")
        

    def controls(self):
        global CENTERS
        global CAMERA
        global object_queue
        global COMPLETED

        node = CloseLoopRobot()

        print("Control Started")

        home_pose = {'action': 'MoveXYZW', 'value': {'positionx': 0.4, 'positiony': 0.05, 'positionz': 0.6, 'yaw': 45.00, 'pitch': 180.00, 'roll': 0.00}, 'speed': 1.0}
        bin_pose = {'action': 'MoveXYZW', 'value': {'positionx': 0.3, 'positiony': 0.45, 'positionz': 0.6, 'yaw': 45.00, 'pitch': 180.00, 'roll': 0.00}, 'speed': 1.0}
        attach = {'action': 'Attach', 'value': {'object': 'box', 'endeffector': 'end_effector_frame'}}
        dettach = {'action': 'Detach', 'value': {'object': 'box'}}
        gripper_open = {'action': 'GripperOpen'}
        gripper_close = {'action': 'GripperClose'}
        generate_grasp = {'action': 'GenerateGrasp', 'value': {'grasp_type': 'generate_grasp_grconvnet'}}
        action_format = {'action': 'MoveXYZW', 'value': {'positionx': 0.5, 'positiony': 0.05, 'positionz': 0.6, 'yaw': 20.00, 'pitch': 180.00, 'roll': 0.00}, 'speed': 1.0}

        # Object data:
        object_data = {}
        object_id = -1

        while True:

            conveyorpower = ConveyorPowerClient()
            # provide power 3.0 to the conveyor belt
            conveyorpower.send_request(3.0)

            conveyorpower.destroy_node()

            # 1. Move to HOME:
            node.control(home_pose)
            print("STEP 1 Moved to home")
            
            # 2. Open the gripper:
            node.control(gripper_open)
            print("STEP 2 Opened the gripper")
            
            # 3. Get image center from overhead camera:
            while len(object_queue) == 0:
                print("No objects detected")
                time.sleep(1)
                continue
            # print(len(object_queue))
            # gs = node.control(generate_grasp)
            object = object_queue.popleft()
            # Add to completed list
            COMPLETED.append(object['id'])
            action_format['value']['positionx'] = object['info']['center'][0] * 1.1
            # action_format['value']['yaw'] = gs[4] + 90

            node.control(action_format)
            # time.sleep(5)
            print("STEP 3 Moved to object center")
            
            # 4. Pose Correction from In-hand camera
            
            while rclpy.ok():
                inhandcamnode = InHandCamSub()
                rclpy.spin_once(inhandcamnode)
                if inhandcamnode.get_object_data() is not None:
                    object_data = inhandcamnode.get_object_data()
                    inhandcamnode.destroy_node()
                    object_data = object_data['detections'][0]
                    center = object_data['info']['center']
                    print("HI")
                    print(center)
                    if 0.05 - center[1] > 0.01:
                        continue
                    conveyorpower = ConveyorPowerClient()
                    # provide power 0.0 to the conveyor belt
                    conveyorpower.send_request(1.0)
                    conveyorpower.destroy_node()
                    break
                inhandcamnode.destroy_node()
            inhandcamnode.destroy_node()
      
            print(f"Conveyor slowed down.")
            # object_data = object_data['detections'][0]
            # center = object_data['info']['center']
            edge_data_node = InHandCamEdgeSub()
            rclpy.spin_once(edge_data_node)
            edge_data = edge_data_node.get_object_data()
            # print(edge_data)
            edge_data_node.destroy_node()
            # print(float(edge_data['angle']))
            anlge = float(edge_data['angle'])
            # print(float(edge_data['center'][0]))
            action_format['value']['positionx'] = center[0]
            # action_format['value']['positionx'] = float(edge_data['center'][0])
            action_format['value']['positiony'] = 0.05
            action_format['value']['yaw'] = anlge
            
            # action_format['value']['yaw'] = 45.00

            node.control(action_format)
            print("STEP 4 Moved to object center")

            # 5. Move down to grasp the object:
            action_format['value']['positionz'] = 0.5
            node.control(action_format)
            print("STEP 5 Moved down to grasp object")

            # 6. Attach the object:
            # object_id = int(object_center['id']) + 1
            object_id += 1
            attach['value']['object'] = f"gelatin_box_1"
            node.control(attach)

            # 7. Close the gripper:
            node.control(gripper_close)

            # 8. Pick the object:
            action_format['value']['positionz'] = 0.6
            node.control(action_format)
            print("STEP 8 Picked the object")

            # 9. Move to BIN:
            node.control(bin_pose)
            print("STEP 9 Moved to bin")

            # 10. Open the gripper:
            node.control(gripper_open)
            print("STEP 10 Opened the gripper")

            # 11. Detach the object:
            node.control(dettach)
            print("STEP 11 Detached the object")


def spin_node():
    # rclpy.init()
    overheadcamnode = OverHeadCamYoloSub()
    try:
        rclpy.spin(overheadcamnode)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

def main(args=None):

    rclpy.init(args=args)

    global CENTERS
    global CAMERA

    # node = CloseLoopRobot()
    overheadcamnode = OverHeadCamYoloSub()
    control_node = RobotControlNode()

    executor = MultiThreadedExecutor()

    executor.add_node(control_node)
    executor.add_node(overheadcamnode)

    executor.spin()

    executor.shutdown()


if __name__ == '__main__':
    main()

