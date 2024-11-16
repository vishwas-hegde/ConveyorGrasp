#!/usr/bin/env python3
#!/usr/bin/python3
# Import required libraries:
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
import os
import ast
import time
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist

# Import ACTIONS:
from ros2_data.action import MoveJ
from ros2_data.action import MoveJs
from ros2_data.action import MoveL
from ros2_data.action import MoveR
from ros2_data.action import MoveXYZW
from ros2_data.action import MoveXYZ
from ros2_data.action import MoveYPR
from ros2_data.action import MoveROT
from ros2_data.action import MoveRP
from ros2_data.action import MoveG
from ros2_grasping.action import Attacher 

import numpy as np

# Import MESSAGES:
from ros2_data.msg import JointPose
from ros2_data.msg import JointPoseS

# Import MultiThreadedExecutor:
from rclpy.executors import MultiThreadedExecutor

# Define GLOBAL VARIABLE -> RES:
RES = "null"

# Define CLASSES for EACH ACTION:

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

# 10. ABB-RWS I/O Service Client:
class abbRWS_IO(Node):

    def __init__(self):
        super().__init__('abbRWS_IO_client')
        self.cli = self.create_client(SetIOSignal, '/rws_client/set_io_signal')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print('Waiting for ABB-RWS I/O Service Server to be available...')
        print('ABB-RWS I/O Service Server detected.')
        self.req = SetIOSignal.Request()

    def send_request(self, signal, value):
        global RES
        self.req.signal = signal
        self.req.value = value
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        RES = self.future.result() 

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

# ===== INPUT PARAMETERS ===== #

# CLASS: Input program (.txt) as ROS2 PARAMETER:
PARAM_PROGRAM = "cubeirb"
P_CHECK_PROGRAM = False
class ProgramPARAM(Node):
    def __init__(self):
        global PARAM_PROGRAM
        global P_CHECK_PROGRAM
        
        super().__init__('ros2_program_param')
        self.declare_parameter('PROGRAM_FILENAME', "default")
        PARAM_PROGRAM = self.get_parameter('PROGRAM_FILENAME').get_parameter_value().string_value
        if (PARAM_PROGRAM == "default"):
            self.get_logger().info('PROGRAM_FILENAME ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:    
            self.get_logger().info('PROGRAM_FILENAME ROS2 Parameter received: ' + PARAM_PROGRAM)
        P_CHECK_PROGRAM = True

# CLASS: Input ROBOT MODEL as ROS2 PARAMETER:
PARAM_ROBOT = "panda"
P_CHECK_ROBOT = False
class RobotPARAM(Node):
    def __init__(self):
        global PARAM_ROBOT
        global P_CHECK_ROBOT
        
        super().__init__('ros2_robot_param')
        self.declare_parameter('ROBOT_MODEL', "default")
        PARAM_ROBOT = self.get_parameter('ROBOT_MODEL').get_parameter_value().string_value
        if (PARAM_ROBOT == "default"):
            self.get_logger().info('ROBOT_MODEL ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:
            self.get_logger().info('ROBOT_MODEL ROS2 Parameter received: ' + PARAM_ROBOT)

            # Check value:
            if (PARAM_ROBOT == "irb120" or 
                PARAM_ROBOT == "irb1200"or
                PARAM_ROBOT == "irb6640" or
                PARAM_ROBOT == "cr35ia" or 
                PARAM_ROBOT == "ur3" or 
                PARAM_ROBOT == "ur5" or 
                PARAM_ROBOT == "ur10" or
                PARAM_ROBOT == "panda" or
                PARAM_ROBOT == "iiwa"):
                None # do nothing.
            else:
                self.get_logger().info('ERROR: The Robot model defined is not in the system.')
                CloseProgram.CLOSE()
        
        P_CHECK_ROBOT = True

# CLASS: Input ROBOT End-Effector MODEL as ROS2 PARAMETER:
PARAM_EE = "panda_hand"
P_CHECK_EE = False
class eePARAM(Node):
    def __init__(self):
        global PARAM_EE
        global P_CHECK_EE
        
        super().__init__('ros2_ee_param')
        self.declare_parameter('EE_MODEL', "default")
        PARAM_EE = self.get_parameter('EE_MODEL').get_parameter_value().string_value
        if (PARAM_EE == "default"):
            self.get_logger().info('EE_MODEL ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:
            self.get_logger().info('EE_MODEL ROS2 Parameter received: ' + PARAM_EE)
            
            if (PARAM_EE == "schunk" or 
                PARAM_EE == "panda_hand" or
                PARAM_EE == "none"):
                None # do nothing.
            else:
                self.get_logger().info('ERROR: The End-Effector model defined is not in the system.')
                CloseProgram.CLOSE()
        
        P_CHECK_EE = True

# CLASS: WARNING + CLOSE:
class CloseProgram():
    def CLOSE():
        print("")
        print("Please execute the program and input all ROS2 parameters in the Ubuntu Terminal as stated below:")
        print('COMMAND -> ros2 run ros2_execution ros2_execution.py --ros-args -p PROGRAM_FILENAME:="---" -p ROBOT_MODEL:="---" -p EE_MODEL:="---"')
        print("Closing... BYE!")
        time.sleep(5)
        exit()


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
        self.get_logger().info('Cube State Subscriber initialized')

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

# ==================================================================================================================================== #
# ==================================================================================================================================== #
# =============================================================== MAIN =============================================================== #
# ==================================================================================================================================== #
# ==================================================================================================================================== #

def main(args=None):
    
    # Import global variable RES:
    global RES
    
    # 1. INITIALISE ROS NODE:
    rclpy.init(args=args)

    
    

    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")

    print("ros2_RobotSimulation --> SEQUENCE EXECUTION")
    print("Python script -> ros2_execution.py")
    print("")

    # 2. INITIALISE RECEIVED ROS2 PARAMETERS:
    global PARAM_PROGRAM
    global P_CHECK_PROGRAM
    global PARAM_ROBOT
    global P_CHECK_ROBOT
    global PARAM_EE
    global P_CHECK_EE
    

    paramNODE = ProgramPARAM()
    while (P_CHECK_PROGRAM == False):
        rclpy.spin_once(paramNODE)
    paramNODE.destroy_node()

    robotNODE = RobotPARAM()
    while (P_CHECK_ROBOT == False):
        rclpy.spin_once(robotNODE)
    robotNODE.destroy_node()

    eeNODE = eePARAM()
    while (P_CHECK_EE == False):
        rclpy.spin_once(eeNODE)
    eeNODE.destroy_node()

    # Load components --> According to input ROS2 Parameters:

    print ("")

    # If parallel gripper: MoveG and attacher plugin activated.
    if (PARAM_EE == "schunk" or 
        PARAM_EE == "panda_hand"):
        MoveG_CLIENT = MoveGclient()
        Attach_Client = ATTACHERclient()
        Detach_Client = DetacherPUB()

    MoveXYZW_CLIENT = MoveXYZWclient()
    MoveXYZ_CLIENT = MoveXYZclient()

    #  3. GET PROGRAM FILENAME:
    print("")
    print("All checks complete!")
    print("")

    # Create NODE for LOGGING:
    nodeLOG = rclpy.create_node('node_LOG')

    EXISTS = True
    PR_NAME = PARAM_PROGRAM
    # filepath = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'ros2_RobotSimulation', 'ros2_execution', 'programs', PR_NAME + ".txt")
    filepath = "/home/agbhat/conveyorGrasp_ws/ConveyorGrasp/src/ros2_execution/programs/cubeirb.txt"
    # EXISTS = os.path.exists(filepath)
    if (EXISTS == True):
        print(PR_NAME + " file found! Executing program...")
        nodeLOG.get_logger().info("SUCCESS: " + PR_NAME + " file (program) found.")
        time.sleep(1)
    elif (EXISTS == False):
        print(PR_NAME + " file not found. Please input the PROGRAM FILENAME correctly as a ROS2 parameter in the Ubuntu Terminal:")
        nodeLOG.get_logger().info("ERROR: " + PR_NAME + " file (program) not found. Please try again.")
        print('COMMAND -> ros2 run ros2_execution ros2_execution.py --ros-args -p PROGRAM_FILENAME:="---" -p ROBOT_MODEL:="---" -p EE_MODEL:="---"')
        print("Closing... BYE!")
        time.sleep(5)
        exit()

    # OPEN PR_NAME.txt FILE:
    with open(filepath) as file:
        f = file.readlines()
        i = 1
        seq = dict()
        for line in f:
            seq[str(i)] = ast.literal_eval(line)
            i = i + 1
        file.close()

    # Log number of steps:
    nodeLOG.get_logger().info(PR_NAME + ": Number of steps -> " + str(len(seq)))
    time.sleep(1)

    #Defining final cube pose for grasp
    grasp_pose = [0.5, 0.05, 0.4]

    # ================================= 4. SEQUENCE ================================= #

    for i in range (1, len(seq)+1):
        
        trigger = seq[str(i)]
        print(i)

        if i == 4:  # wait for cube to arrive
            grasp_pose = [0.5, 0.05, 0.4]
            while rclpy.ok():
                cube_subscriber = CubePoseSub()
                rclpy.spin_once(cube_subscriber)
                if cube_subscriber.get_current_pose() is not None:
                    distance = np.linalg.norm(np.array(cube_subscriber.get_current_pose()) - np.array(grasp_pose))
                    print(distance)
                    if distance < 0.05:
                        break
    



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
            
            MoveXYZW_CLIENT.send_goal(positionx,positiony,positionz,yaw,pitch,roll, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveXYZW_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveXYZW ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveXYZW:SUCCESS"):
                print("MoveXYZW ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveXYZW ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveXYZW ACTION in step number -> " + str(i) + " failed.")
                break

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
            MoveXYZ_CLIENT.send_goal(positionx,positiony,positionz, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveXYZ_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveXYZ ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveXYZ:SUCCESS"):
                print("MoveXYZ ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveXYZ ACTION in step number -> " + str(i) + " failed.")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveXYZ ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                break

        elif (trigger['action'] == 'Attach'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> ATTACH OBJECT:")
            print(trigger['value'])

            OBJ = trigger['value']['object']
            EE = trigger['value']['endeffector']
            
            Attach_Client.send_goal(OBJ,EE)
            rclpy.spin_once(Attach_Client)
            
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
                Detach_Client.publisher_.publish(MSG) # Publish repeatedly for a second to make sure that the ATTACHER SERVER receives the message.
        
            print("Object DETACHED successfully.")

        elif (trigger['action'] == 'GripperOpen'):

            if (PARAM_EE == "panda_hand"):
            
                print("")
                print("STEP NUMBER " + str(i) + " -> GripperOpen (MoveG).")

                GP = 0.04
                MoveG_CLIENT.send_goal(GP)
                
                while rclpy.ok():
                    rclpy.spin_once(MoveG_CLIENT)
                    if (RES != "null"):
                        break
                
                print ("Result of MoveG ACTION CALL is -> { " + RES + " }")
                
                if (RES == "MoveG:SUCCESS"):
                    print("MoveG ACTION in step number -> " + str(i) + " successfully executed.")
                    RES = "null"
                else:
                    print("MoveG ACTION in step number -> " + str(i) + " failed.")
                    print("The program will be closed. Bye!")
                    break

        elif (trigger['action'] == 'GripperClose'):

            if (PARAM_EE == "panda_hand"):
            
                print("")
                print("STEP NUMBER " + str(i) + " -> GripperClose (MoveG).")

                GP = 0.02
                MoveG_CLIENT.send_goal(GP)
                
                while rclpy.ok():
                    rclpy.spin_once(MoveG_CLIENT)
                    if (RES != "null"):
                        break
                
                print ("Result of MoveG ACTION CALL is -> { " + RES + " }")
                
                if (RES == "MoveG:SUCCESS"):
                    print("MoveG ACTION in step number -> " + str(i) + " successfully executed.")
                    RES = "null"
                else:
                    print("MoveG ACTION in step number -> " + str(i) + " failed.")
                    print("The program will be closed. Bye!")
                    break

        else:
            print("Step number " + str(i) + " -> Action type not identified. Please check.")
            print("The program will be closed. Bye!")
            nodeLOG.get_logger().info("ERROR: Program finished since ACTION NAME in step number -> " + str(i) + " was not identified.")
            break

        #time.sleep(1)

    print("")
    print("SEQUENCE EXECUTION FINISHED!")
    print("Program will be closed. Bye!")
    nodeLOG.get_logger().info("SUCESS: Program execution sucessfully finished.")
    nodeLOG.destroy_node()
    print("Closing... BYE!")
    time.sleep(5)
        

if __name__ == '__main__':
    main()