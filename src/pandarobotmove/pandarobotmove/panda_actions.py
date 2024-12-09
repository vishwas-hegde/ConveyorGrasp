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



RES = "null"
CENTERS = None
CAMERA = 'overhead'
object_queue = deque()

class CloseLoopRobot(Node):

    def __init__(self):
        super().__init__("close_loop_robot")

        self.MoveG_CLIENT = MoveGclient()
        self.Attach_Client = ATTACHERclient()
        self.Detach_Client = DetacherPUB()

        self.MoveXYZW_CLIENT = MoveXYZWclient()
        self.MoveXYZ_CLIENT = MoveXYZclient()

        # self.grasp_client = GraspClient()

        self.nodeLOG = rclpy.create_node('node_LOG')

    def future_grasp(self, position, velocity=0.05):
        y = position

        graspy = 0.05
        diff = abs(y - graspy)

        # calcuate the time to reach the object:
        time = diff / velocity

        return time

    def control(self, msg):
        global RES
        
        # Log number of steps:
        # self.nodeLOG.get_logger().info("Number of steps -> " + str(1))
        # time.sleep(1)

        #Defining final cube pose for grasp
        grasp_pose = [0.5, 0.05, 0.4]

        trigger = msg

        # print(f"Received message: {trigger}")

        # trigger = json.loads(trigger)
        i = 0

        # ================================= 4. SEQUENCE ================================= #
        if (trigger['action'] == 'MoveXYZW'): 
            
            # print("")
            # print("STEP NUMBER " + str(i) + " -> MoveXYZW:")
            # print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            # else:
            #     print("Joint speed -> " + str(JointSPEED))

            positionx = trigger['value']['positionx']
            positiony = trigger['value']['positiony']
            positionz = trigger['value']['positionz']
            yaw = trigger['value']['yaw']
            pitch = trigger['value']['pitch']
            roll = trigger['value']['roll']
            
            self.MoveXYZW_CLIENT.send_goal(positionx,positiony,positionz,yaw,pitch,roll, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(self.MoveXYZW_CLIENT)
                if (RES != "null"):
                    break
            
            # print ("Result of MoveXYZW ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveXYZW:SUCCESS"):
                # print("MoveXYZW ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
            #     print("MoveXYZW ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
            #     self.nodeLOG.get_logger().info("ERROR: Program finished since MoveXYZW ACTION in step number -> " + str(i) + " failed.")

        elif (trigger['action'] == 'MoveXYZ'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveXYZ:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            positionx = trigger['value']['positionx']
            positiony = trigger['value']['positiony']
            positionz = trigger['value']['positionz']
            self.MoveXYZ_CLIENT.send_goal(positionx,positiony,positionz, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(self.MoveXYZ_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveXYZ ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveXYZ:SUCCESS"):
                print("MoveXYZ ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveXYZ ACTION in step number -> " + str(i) + " failed.")
                self.nodeLOG.get_logger().info("ERROR: Program finished since MoveXYZ ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")

        elif (trigger['action'] == 'Attach'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> ATTACH OBJECT:")
            print(trigger['value'])

            OBJ = trigger['value']['object']
            EE = trigger['value']['endeffector']
            
            self.Attach_Client.send_goal(OBJ,EE)
            rclpy.spin_once(self.Attach_Client)
            
            print("Object ATTACHED successfully.")

        elif (trigger['action'] == 'Detach'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> DETACH OBJECT:")
            print(trigger['value'])

            OBJ = trigger['value']['object']
            
            MSG = String()
            MSG.data = "True"

            t_end = time.time() + 1
            while time.time() < t_end:
                self.Detach_Client.publisher_.publish(MSG) # Publish repeatedly for a second to make sure that the ATTACHER SERVER receives the message.
        
            print("Object DETACHED successfully.")

        elif (trigger['action'] == 'GripperOpen'):
            
            # print("")
            # print("STEP NUMBER " + str(i) + " -> GripperOpen (MoveG).")

            GP = 0.04
            self.MoveG_CLIENT.send_goal(GP)
            
            while rclpy.ok():
                rclpy.spin_once(self.MoveG_CLIENT)
                if (RES != "null"):
                    break
            
            # print ("Result of MoveG ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveG:SUCCESS"):
                # print("MoveG ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            # else:
            #     print("MoveG ACTION in step number -> " + str(i) + " failed.")
            #     print("The program will be closed. Bye!")

        elif (trigger['action'] == 'GripperClose'):
            
            # print("")
            # print("STEP NUMBER " + str(i) + " -> GripperClose (MoveG).")

            GP = 0.01
            self.MoveG_CLIENT.send_goal(GP)
            
            while rclpy.ok():
                rclpy.spin_once(self.MoveG_CLIENT)
                if (RES != "null"):
                    break
            
            # print("Result of MoveG ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveG:SUCCESS"):
                # print("MoveG ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            # else:
            #     print("MoveG ACTION in step number -> " + str(i) + " failed.")
            #     print("The program will be closed. Bye!")
        
        elif(trigger['action'] == 'GenerateGrasp'):
            print("")
            print("STEP NUMBER " + str(i) + " -> GenerateGrasp:")
            print(trigger['value'])

            # Extract grasp type from the message
            grasp_type = trigger['value']['grasp_type']
            grasp_dict = {'grasp_type':'generate_grasp_grconvnet'}

            # convert to json string
            grasp_type = json.dumps(grasp_dict)

            # Create an instance of GraspClient
            

            # Send the service request with grasp_type
            future = self.grasp_client.send_request(grasp_type)

            rclpy.spin_until_future_complete(self.grasp_client, future)
            response = future.result()
            gs = response.grasp.data
            print(f'Grasp rectangle: {gs}')

            # Spin until response is received
            # while rclpy.ok():
            #     rclpy.spin_once(self.grasp_client)
            #     # if RES != "null":  # You can set a global flag for response if needed
            #     break

            print("Grasp generation completed.")
            RES = "null"  # Reset the global response flag
            return gs

        else:
            print("Step number " + str(i) + " -> Action type not identified. Please check.")
            print("The program will be closed. Bye!")
            self.nodeLOG.get_logger().info("ERROR: Program finished since ACTION NAME in step number -> " + str(i) + " was not identified.")

            #time.sleep(1)

        # print("")
        # print("SEQUENCE EXECUTION FINISHED!")
        # # print("Program will be closed. Bye!")
        # self.nodeLOG.get_logger().info("SUCESS: Program execution sucessfully finished.")
        # nodeLOG.destroy_node()
        # print("Closing... BYE!")
        # time.sleep(1)


class MoveXYZWclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveXYZW_client')
        self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')
        # 2. Wait for MoveXYZW server to be available:
        print ("Waiting for MoveXYZW action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveXYZW ACTION SERVER detected.")
    
    def send_goal(self, GoalXYZWx, GoalXYZWy, GoalXYZWz, GoalXYZWyaw, GoalXYZWpitch, GoalXYZWroll, JointSPEED):
        # 1. Assign variables:
        goal_msg = MoveXYZW.Goal()
        goal_msg.positionx = GoalXYZWx
        goal_msg.positiony = GoalXYZWy
        goal_msg.positionz = GoalXYZWz
        goal_msg.yaw = GoalXYZWyaw
        goal_msg.pitch = GoalXYZWpitch
        goal_msg.roll = GoalXYZWroll
        goal_msg.speed = JointSPEED
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        # self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveXYZW ACTION CALL finished.")     

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveXYZW ACTION CALL.

# 5. MoveXYZ:
class MoveXYZclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveXYZ_client')
        self._action_client = ActionClient(self, MoveXYZ, 'MoveXYZ')
        # 2. Wait for MoveXYZ server to be available:
        print ("Waiting for MoveXYZ action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveXYZ ACTION SERVER detected.")
    
    def send_goal(self, GoalXYZx, GoalXYZy, GoalXYZz, JointSPEED):
        # 1. Assign variables:
        goal_msg = MoveXYZ.Goal()
        goal_msg.positionx = GoalXYZx
        goal_msg.positiony = GoalXYZy
        goal_msg.positionz = GoalXYZz
        goal_msg.speed = JointSPEED         
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        # self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveXYZ ACTION CALL finished.")    

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveXYZ ACTION CALL.

# 9. MoveG:
class MoveGclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveG_client')
        self._action_client = ActionClient(self, MoveG, 'MoveG')
        # 2. Wait for MoveG server to be available:
        print ("Waiting for MoveG action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveG ACTION SERVER detected.")
    
    def send_goal(self, GP):
        # 1. Assign variables:
        goal_msg = MoveG.Goal()
        goal_msg.goal = GP
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        # self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveG ACTION CALL finished.")

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveG ACTION CALL.

# 11. ATTACHER - Action Client:
class ATTACHERclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('Attacher_client')
        self._action_client = ActionClient(self, Attacher, 'Attacher')
        # 2. Wait for ATTACHER server to be available:
        print ("Waiting for ATTACHER action server to be available...")
        self._action_client.wait_for_server()
        print ("Attacher ACTION SERVER detected.")
    
    def send_goal(self, object, endeffector):
        # 1. Assign variables:
        goal_msg = Attacher.Goal()
        goal_msg.object = object
        goal_msg.endeffector = endeffector
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

# 12. DETACHER - Publihser:
class DetacherPUB(Node):
    
    def __init__(self):
        # Declare NODE:
        super().__init__("ros2_PUBLISHER")
        # Declare PUBLISHER:
        self.publisher_ = self.create_publisher(String, "ros2_Detach", 5) #(msgType, TopicName, QueueSize)

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

        if data_dict['id'] not in object_queue:
            object_queue.append(data_dict['id'])
            print(f"Object {data_dict['id']} detected at center {data_dict['center']}")
            global CENTERS
            CENTERS = data_dict['center']

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
        # response = future.result()
        # gs = response.grasp.data
        # self.gs = gs
        # print(f'Grasp rectangle: {gs}')
        # future.add_done_callback(self.response_callback)
        return future
    
    def response_callback(self, future):
        try:
            # print("Received response")
            response = future.result()
            grasp_rectangle = response.grasp.data
            self.get_logger().info(f'Received grasp rectangle: {grasp_rectangle}')
        except Exception as e:
            self.get_logger().error(f'Error while calling service: {str(e)}')
