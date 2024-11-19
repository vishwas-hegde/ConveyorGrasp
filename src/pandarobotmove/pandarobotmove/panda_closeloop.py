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

RES = "null"
CENTERS = None
CAMERA = 'overhead'

class CloseLoopRobot(Node):

    def __init__(self):
        super().__init__("close_loop_robot")

        self.MoveG_CLIENT = MoveGclient()
        self.Attach_Client = ATTACHERclient()
        self.Detach_Client = DetacherPUB()

        self.MoveXYZW_CLIENT = MoveXYZWclient()
        self.MoveXYZ_CLIENT = MoveXYZclient()

        self.nodeLOG = rclpy.create_node('node_LOG')

        # self.image_processing = ImageProcessing()

    def get_object_center(self, camera='overhead'):
        # print('hi')
        rclpy.init()
        self.image_processing = ImageProcessing()
        rclpy.spin_once(self.image_processing)
            
        centers = self.image_processing.get_object_center(camera=camera)
        return centers
    
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
        self.nodeLOG.get_logger().info("Number of steps -> " + str(1))
        # time.sleep(1)

        #Defining final cube pose for grasp
        grasp_pose = [0.5, 0.05, 0.4]

        trigger = msg

        print(f"Received message: {trigger}")

        # trigger = json.loads(trigger)
        i = 0

        # ================================= 4. SEQUENCE ================================= #
        if (trigger['action'] == 'MoveXYZW'): 
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveXYZW:")
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
            yaw = trigger['value']['yaw']
            pitch = trigger['value']['pitch']
            roll = trigger['value']['roll']
            
            self.MoveXYZW_CLIENT.send_goal(positionx,positiony,positionz,yaw,pitch,roll, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(self.MoveXYZW_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveXYZW ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveXYZW:SUCCESS"):
                print("MoveXYZW ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveXYZW ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                self.nodeLOG.get_logger().info("ERROR: Program finished since MoveXYZW ACTION in step number -> " + str(i) + " failed.")

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
            
            print("")
            print("STEP NUMBER " + str(i) + " -> GripperOpen (MoveG).")

            GP = 0.04
            self.MoveG_CLIENT.send_goal(GP)
            
            while rclpy.ok():
                rclpy.spin_once(self.MoveG_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveG ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveG:SUCCESS"):
                print("MoveG ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveG ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")

        elif (trigger['action'] == 'GripperClose'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> GripperClose (MoveG).")

            GP = 0.02
            self.MoveG_CLIENT.send_goal(GP)
            
            while rclpy.ok():
                rclpy.spin_once(self.MoveG_CLIENT)
                if (RES != "null"):
                    break
            
            print("Result of MoveG ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveG:SUCCESS"):
                print("MoveG ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveG ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")

        else:
            print("Step number " + str(i) + " -> Action type not identified. Please check.")
            print("The program will be closed. Bye!")
            self.nodeLOG.get_logger().info("ERROR: Program finished since ACTION NAME in step number -> " + str(i) + " was not identified.")

            #time.sleep(1)

        print("")
        print("SEQUENCE EXECUTION FINISHED!")
        # print("Program will be closed. Bye!")
        self.nodeLOG.get_logger().info("SUCESS: Program execution sucessfully finished.")
        # nodeLOG.destroy_node()
        # print("Closing... BYE!")
        # time.sleep(1)

class ImageProcessing(Node):
    def __init__(self):
        super().__init__("image_processing")
        self.cv_bridge = CvBridge()
        self.overhead_rgb = None
        self.overhead_depth = None
        self.inhand_rgb = None
        self.inhand_depth = None
        self.object_world_coords = None
        self.overhead_rgb_sub = self.create_subscription(Image, '/realsense/image_raw', self.overhead_rgb_callback, 10)
        self.overhead_depth_sub = self.create_subscription(Image, '/realsense/depth/image_raw', self.overhead_depth_callback, 10)
        self.inhand_rgb_sub = self.create_subscription(Image, '/panda_camera/image_raw', self.inhand_rgb_callback, 10)
        self.inhand_depth_sub = self.create_subscription(Image, '/panda_camera/depth/image_raw', self.inhand_depth_callback, 10)

        self.get_object_center()

    def overhead_rgb_callback(self, msg):
        self.overhead_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        # print(self.overhead_rgb)
        self.get_object_center()

    def overhead_depth_callback(self, msg):
        self.overhead_depth = self.cv_bridge.imgmsg_to_cv2(msg, '32FC1')

    def inhand_rgb_callback(self, msg):
        self.inhand_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

    def inhand_depth_callback(self, msg):
        self.inhand_depth = self.cv_bridge.imgmsg_to_cv2(msg, '32FC1')
    
    def return_center(self):
        return self.object_world_coords

    def segment_cube(self, rgb_image):
        if rgb_image is not None:
            hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([10, 100, 100])
            upper_orange = np.array([25, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)

            # publish mask image
            # mask_image = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            # self.image_publisher.publish(mask_image)
            return mask
    
    def get_object_center(self, camera=None):
        rgb_image = None
        global CAMERA
        camera = CAMERA
        if camera == 'overhead':
            rgb_image = self.overhead_rgb
            # print(rgb_image)
        elif camera == 'inhand':
            rgb_image = self.inhand_rgb
        if rgb_image is not None:
            mask = self.segment_cube(rgb_image=rgb_image)
            if mask is None:
                self.center = None
                return
            M = cv2.moments(mask)
            if M["m00"] == 0:
                self.center = None
                return
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            self.center = (center_x, center_y)
            # print(self.center)

            self.convert_to_world_coordinates(rgb_image=rgb_image)

    def convert_to_world_coordinates(self, rgb_image=None):
        if rgb_image is not None:
            global CENTERS
            # Get depth value at the center
            depth_value = 0.76

            # Intrinsic parameters (update as per your camera)
            self.fx = 554.256  # Focal length in x
            self.fy = 554.256  # Focal length in y
            self.cx = rgb_image.shape[1] // 2  # Image center x-coordinate
            self.cy = rgb_image.shape[0] // 2  # Image center y-coordinate

            # Calculate 3D position in the camera frame
            y = -(self.center[0] - self.cx) * depth_value / self.fx
            x = (self.center[1] - self.cy) * depth_value / self.fy
            z = depth_value
            # print("Object Camera Coordinates:", x, y, z)

            R_c_w = np.array([[-1, 0, 0],
                          [0, 1, 0],
                          [0, 0, -1]])
            T_c_w = np.array([0.5 , -0.7 , 1.12])
            T_w = np.array([0, 0, 0])

            object_coordinates_wrto_camera = [x,y,z]
            camera_coords_wrto_world = R_c_w @ T_w + T_c_w
            object_coords_wrto_world = R_c_w @ (object_coordinates_wrto_camera) + camera_coords_wrto_world

            self.object_world_coords = np.array(object_coords_wrto_world)
            # print("camera_coords_wrto_world",camera_coords_wrto_world)
            # print(object_coords_wrto_world)
            CENTERS = object_coords_wrto_world
            # print(CENTERS)
            
        return object_coords_wrto_world

# 4. MoveXYZW:
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
        self.get_logger().info('Goal accepted :)')
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
        self.get_logger().info('Goal accepted :)')
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
        self.get_logger().info('Goal accepted :)')
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

class CubePoseSub(Node):
    def __init__(self):
        super().__init__('cube_position_subscriber')
        self.subscription = self.create_subscription(
            ModelStates,
            'ros2_grasp/model_states',
            self.listener_callback,
            10)
        self.subscription
        
        # Initialize cube position as None to indicate no data received yet
        self.cube_pose = None
        # self.get_logger().info('Cube State Subscriber initialized')

    def listener_callback(self, msg):
        try:
            box_index = msg.name.index('box')
            pose = msg.pose[box_index].position
            self.cube_pose = [pose.x, pose.y, pose.z]
            # print(f'Cube Position: x={self.cube_pose[0]:.3f}, y={self.cube_pose[1]:.3f}, z={self.cube_pose[2]:.3f}')
            # self.get_logger().debug(  # Changed to debug to reduce spam
            #     f'Cube Position: x={self.cube_pose[0]:.3f}, '
            #     f'y={self.cube_pose[1]:.3f}, '
            #     f'z={self.cube_pose[2]:.3f}'
            # )
            
        except ValueError:
            self.get_logger().warn(f"'box' model not found in model_states")

    def get_current_pose(self):
        """Returns current pose or None if no data received yet"""
        return self.cube_pose

    def wait_for_position(self, target_pose, tolerance=0.01, timeout=None):
        """
        Wait until cube reaches near target position within tolerance
        
        Args:
            target_pose: List[float] - [x, y, z] target position
            tolerance: float - How close to target is considered "arrived"
            timeout: float or None - Max seconds to wait, None for no timeout
            
        Returns:
            bool - True if position reached, False if timed out
        """
        if timeout:
            start_time = self.get_clock().now()
            
        while True:
            if self.cube_pose is None:
                self.get_logger().info("Waiting for first cube position data...")
                time.sleep(0.01)
                continue
                
            distance = np.linalg.norm(np.array(self.cube_pose) - np.array(target_pose))

            print(distance)
            
            if distance <= tolerance:
                return True
                
            if timeout:
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > timeout:
                    return False
                    
            time.sleep(0.1)

def main(args=None):

    rclpy.init(args=args)

    global CENTERS
    global CAMERA

    node = CloseLoopRobot()

    home_pose = {'action': 'MoveXYZW', 'value': {'positionx': 0.4, 'positiony': 0.2, 'positionz': 0.6, 'yaw': 45.00, 'pitch': 180.00, 'roll': 0.00}, 'speed': 1.0}
    bin_pose = {'action': 'MoveXYZW', 'value': {'positionx': 0.0, 'positiony': -0.5, 'positionz': 0.6, 'yaw': 45.00, 'pitch': 180.00, 'roll': 0.00}, 'speed': 1.0}
    attach = {'action': 'Attach', 'value': {'object': 'box', 'endeffector': 'end_effector_frame'}}
    dettach = {'action': 'Detach', 'value': {'object': 'box'}}
    gripper_open = {'action': 'GripperOpen'}
    gripper_close = {'action': 'GripperClose'}

    action_format = {'action': 'MoveXYZW', 'value': {'positionx': 0.5, 'positiony': 0.05, 'positionz': 0.6, 'yaw': 45.00, 'pitch': 180.00, 'roll': 0.00}, 'speed': 1.0}

    while True:
        # 1. Move to HOME:
        node.control(home_pose)
        # time.sleep(0.5)

        # 2. Open the gripper:
        node.control(gripper_open)

        # 3. Get image center from overhead camera:
        
        while rclpy.ok():
            CAMERA = 'overhead'
            image_processing = ImageProcessing()
            rclpy.spin_once(image_processing)
            # print(CENTERS)
            image_processing.destroy_node()
            if CENTERS is not None:
                break
        # print(center)
        center = CENTERS
        yaw = 45.0

        # 4. Move the end-effector to match the object center:
        action_format['value']['positionx'] = center[0]
        action_format['value']['positiony'] = 0.05
        action_format['value']['yaw'] = yaw
        node.control(action_format)
        # time.sleep(0.5)

        action_format['value']['positionz'] = 0.46

        node.control(action_format)

        # 5. Move down to grasp the object:
        grasp_pose = [0.6, 0.05, 0.4]
        while rclpy.ok():
            cube_subscriber = CubePoseSub()
            rclpy.spin_once(cube_subscriber)
            if cube_subscriber.get_current_pose() is not None:
                distance = np.linalg.norm(np.array(cube_subscriber.get_current_pose()) - np.array(grasp_pose))
                print(distance)
                if distance < 0.05:
                    break
            cube_subscriber.destroy_node()
        # correct the end-effector position:
        # center = node.get_object_center(camera='inhand')
        # pos_y = center[1]
        # time_to_grasp = node.future_grasp(pos_y)
        # time.sleep(time_to_grasp-0.5)
        # # center = [0.5, 0.05]
        # yaw = 45.0
        # action_format['value']['positionx'] = center[0]
        # action_format['value']['positiony'] = center[1]
        # action_format['value']['yaw'] = yaw
        # reduce Z
       

        # 6. Close the gripper:
        node.control(gripper_close)

        # 7. Attach the object:
        node.control(attach) 

        # 8. Move to home:
        node.control(home_pose)

        # 9. Move to bin:
        node.control(bin_pose)

        # 10. Open the gripper:
        node.control(gripper_open)

        # 11. Detach the object:
        node.control(dettach)


    # rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

