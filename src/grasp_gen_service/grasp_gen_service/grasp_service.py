"""
Service for Grasp Generation using GRConvNet, GGCNN and GGCNN2
"""
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from grasp_gen_interface.srv import GraspGen
from . import ggcnn_process
from . import grconvnet_process
from . import ggcnn2_process
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray, Float64MultiArray

class GenerativeGraspService(Node):
    def __init__(self):
        super().__init__('generative_grasp_service')
        self.srv = self.create_service(GraspGen, 'generate_grasp', self.generate_grasp_callback)

        # Subscriber to rgb image
        self.image_subscriber = self.create_subscription(
            Image,
            '/panda_camera/image_raw',             # get the rgb image from realsense camera
            self.rgb_listener_callback,
            10)
        
        # Subscriber to depth image
        self.depth_subscriber = self.create_subscription(
            Image,
            '/panda_camera/depth/image_raw',        # get the depth image from realsense camera
            self.depth_listener_callback,
            10) 
        
        # Publisher for grasp rectangle
        self.grasp_rectangle_publisher = self.create_publisher(Float32MultiArray, '/grasp_rectangle', 10)

        # Publisher for processed depth image to be used for pose generation
        self.depth_image_processed_publisher = self.create_publisher(Image, '/vbm/depth/image_raw', 10)
        
        self.grconvnet = grconvnet_process.GRConvNet_Grasp()   # Initialize GRConvNet
        self.ggcnn = ggcnn_process.GGCNN_Grasp()               # Initialize GGCNN
        self.ggcnn2 = ggcnn2_process.GGCNN_Grasp()              # Initialize GGCNN2
        
        self.rgb_image = None
        self.depth_image = None

        self.br = CvBridge()                            # CvBridge object for image conversion
        self.get_logger().info('Generative Grasp Service has been started')

    def generate_grasp_callback(self, request, response):
        """
        Callback function for grasp generation service
        """

        if request.input == "generate_grasp_grconvnet":
            plt.imshow(self.depth_image)
            plt.show()
            gs, depth_img_processed, gs_copy = self.grconvnet.process_data(self.rgb_image, self.depth_image)   # Process data using GRConvNet
            gs = Float32MultiArray(data=gs)
            self.get_logger().info('Grasp generated')
            response.grasp = gs
            self.grasp_rectangle_publisher.publish(gs)     # Publish grasp rectangle
            self.depth_image_processed_publisher.publish(self.br.cv2_to_imgmsg(depth_img_processed))   # Publish processed depth image

            # self.plot_grasp(gs_copy)    # Plot the grasp rectangle on the original image

            # plt.imshow(depth_img_processed)
            # plt.show()

        elif request.input == "generate_grasp_ggcnn":
            gs, depth_img_processed, gs_copy = self.ggcnn.process_data(self.rgb_image, self.depth_image)        # Process data using GGCNN
            gs = Float32MultiArray(data=gs)
            self.get_logger().info('Grasp generated')
            response.grasp = gs
            self.grasp_rectangle_publisher.publish(gs)    # Publish grasp rectangle
            self.depth_image_processed_publisher.publish(self.br.cv2_to_imgmsg(depth_img_processed))    # Publish processed depth image

            # self.plot_grasp(gs_copy)    # Plot the grasp rectangle on the original image

            # plt.imshow(depth_img_processed)
            # plt.show()

        elif request.input == "generate_grasp_ggcnn2":
            gs, depth_img_processed, gs_copy = self.ggcnn2.process_data(self.rgb_image, self.depth_image)   # Process data using GGCNN2
            gs = Float32MultiArray(data=gs)
            self.get_logger().info('Grasp generated')
            response.grasp = gs
            self.grasp_rectangle_publisher.publish(gs)     # Publish grasp rectangle
            self.depth_image_processed_publisher.publish(self.br.cv2_to_imgmsg(depth_img_processed))   # Publish processed depth image

            self.plot_grasp(gs_copy)    # Plot the grasp rectangle on the original image
        
        else:
            self.get_logger().info('Invalid input')
            response.grasp = Float32MultiArray(data=[0, 0, 0, 0, 0])

        return response
    
    def rgb_listener_callback(self, msg):
        """
        Callback function for rgb image subscriber
        """
        frame = self.br.imgmsg_to_cv2(msg)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.rgb_image = frame

    def depth_listener_callback(self, msg):
        """
        Callback function for depth image subscriber
        """
        frame = self.br.imgmsg_to_cv2(msg, "32FC1")

        self.depth_image = frame
    
    def plot_grasp(self, gs):
        """
        Plot the grasp rectangle on the original image
        """
        ax = plt.subplot(111)
        ax.imshow(self.rgb_image)
        for g in gs:
            g.plot(ax)
        ax.set_title('Grasp')
        ax.axis('off')
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    generative_grasp_service = GenerativeGraspService()

    rclpy.spin(generative_grasp_service)

    generative_grasp_service.destroy_node()
    rclpy.shutdown()


    