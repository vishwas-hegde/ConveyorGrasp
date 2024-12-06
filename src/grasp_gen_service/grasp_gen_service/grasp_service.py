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
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import json
import matplotlib.patches as patches

class GenerativeGraspService(Node):
    def __init__(self):
        super().__init__('generative_grasp_service')
        self.srv = self.create_service(GraspGen, 'generate_grasp', self.generate_grasp_callback)

        # Subscriber to rgb image
        self.image_subscriber = self.create_subscription(
            Image,
            '/realsense/image_raw',             # get the rgb image from realsense camera
            self.rgb_listener_callback,
            10)
        
        # Subscriber to depth image
        self.depth_subscriber = self.create_subscription(
            Image,
            '/realsense/depth/image_raw',        # get the depth image from realsense camera
            self.depth_listener_callback,
            10) 
        
        self.detection_sub = self.create_subscription(
                  String, '/yolo/overhead', self.detection_callback, 10)
            
        self.grasp_detections_pub = self.create_publisher(String, '/grasp_detections', 10)
        
        # Publisher for grasp rectangle
        self.grasp_rectangle_publisher = self.create_publisher(Float32MultiArray, '/grasp_rectangle', 10)

        # Publisher for processed depth image to be used for pose generation
        self.depth_image_processed_publisher = self.create_publisher(Image, '/vbm/depth/image_raw', 10)
        
        self.grconvnet = grconvnet_process.GRConvNet_Grasp()   # Initialize GRConvNet
        self.ggcnn = ggcnn_process.GGCNN_Grasp()               # Initialize GGCNN
        self.ggcnn2 = ggcnn2_process.GGCNN_Grasp()              # Initialize GGCNN2
        
        self.rgb_image = None
        self.depth_image = None
        self.latest_detections = None

        self.br = CvBridge()                            # CvBridge object for image conversion
        self.get_logger().info('Generative Grasp Service has been started')

    def generate_grasp_callback(self, request, response):
        """
        Callback function for grasp generation service
        """
        try:
            self.get_logger().debug(f'Received request input: {request.input}')
            
            try:
                data = json.loads(request.input)
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Failed to parse JSON input: {str(e)}')
                response.grasp = Float32MultiArray(data=[0, 0, 0, 0, 0])
                return response
            
            if self.latest_detections is None:
                self.get_logger().error("No detections received yet.")
                response.grasp = Float32MultiArray(data=[0, 0, 0, 0, 0])
                return response

            if not isinstance(data, dict):
                self.get_logger().error('Input must be a JSON object')
                response.grasp = Float32MultiArray(data=[0, 0, 0, 0, 0])
                return response

            if 'grasp_type' not in data or 'crop' not in data:
                self.get_logger().error('Missing required fields: grasp_type or crop')
                response.grasp = Float32MultiArray(data=[0, 0, 0, 0, 0])
                return response

            grasp_type = data['grasp_type']
            crop = data['crop']

            if not isinstance(crop, list) or len(crop) != 4:
                self.get_logger().error('Crop must be a list of 4 values [x_min, y_min, x_max, y_max]')
                response.grasp = Float32MultiArray(data=[0, 0, 0, 0, 0])
                return response
            
            if grasp_type == "generate_grasp_grconvnet":
                gs, depth_img_processed, gs_copy = self.grconvnet.process_data(self.rgb_image, self.depth_image, crop, self.latest_detections)  
                flat_gs = []
                for grasp in gs:
                    flat_gs.extend([
                        grasp['center_x'], grasp['center_y'],
                        grasp['width'], grasp['height'],
                        grasp['angle']
                    ])

                gs = Float32MultiArray(data=flat_gs)
                self.get_logger().info('Grasp generated')
                # response.grasp = gs
                self.grasp_rectangle_publisher.publish(gs)     
                self.depth_image_processed_publisher.publish(self.br.cv2_to_imgmsg(depth_img_processed)) 
                closest_grasp=self.plot_grasp(gs, self.latest_detections)  
                response.grasp = closest_grasp  
            
            else:
                self.get_logger().info('Invalid input')
                response.grasp = Float32MultiArray(data=[0, 0, 0, 0, 0])

            return response
        
        except Exception as e:
            self.get_logger().error(f'Unexpected error in grasp generation: {str(e)}')
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

    def detection_callback(self, msg):
            """
            Callback function for receiving detections from YOLO node
            """
            try:
                # self.get_logger().info(f"Detection callback received data: {msg.data}")
                self.latest_detections = json.loads(msg.data)  # Parse and store the detections
                # self.get_logger().info(f"Updated latest detections: {self.latest_detections}")
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to parse detection message: {str(e)}")
    
    
    def plot_grasp(self, grasps, detections):
        """
        Plot the closest grasp rectangle to each YOLO detection bounding box on the RGB image.
        """
        if self.rgb_image is None:
            self.get_logger().error("No RGB image available for plotting.")
            return
        print("inside the plot grasp function")

        ax = plt.subplot(111)
        ax.imshow(self.rgb_image)

        grasps = grasps.data
        num_elements_per_grasp = 5
        grasp_list = [grasps[i:i + num_elements_per_grasp] for i in range(0, len(grasps), num_elements_per_grasp)]

        for detection in detections.get("detections", []):
            bbox = detection["bbox"]
            bbox_color = "red"
            center_color = "blue"

            x_min, y_min, x_max, y_max = bbox
            bbox_center_x = (x_min + x_max) / 2
            bbox_center_y = (y_min + y_max) / 2

            rect = plt.Rectangle(
                (x_min, y_min),
                x_max - x_min,
                y_max - y_min,
                linewidth=2,
                edgecolor=bbox_color,
                facecolor="none",
                linestyle="--",
            )
            ax.add_patch(rect)
            ax.plot(bbox_center_x, bbox_center_y, marker="o", color=center_color, markersize=5)

            ax.text(
                x_min,
                y_min - 10,
                f"ID: {detection['id']} Class: {detection['class']}",
                color=bbox_color,
                fontsize=8,
                bbox=dict(facecolor="white", alpha=0.5),
            )

            min_distance = float("inf")
            closest_grasp = None
            for k, g in enumerate(grasp_list):
                print(f"Grasp {k}: {g}")
                centerx, centery, length, width, angle = g
                distance = ((centerx - bbox_center_x) ** 2 + (centery - bbox_center_y) ** 2) ** 0.5
                if distance < min_distance:
                    min_distance = distance
                    closest_grasp = g

            if closest_grasp:
                centerx, centery, length, width, angle = closest_grasp
                print(f"Closest Grasp to BBox Center ({bbox_center_x}, {bbox_center_y}): ({centerx}, {centery})")
                rectangle = patches.Rectangle(
                    (centerx - length / 2, centery - width / 2),
                    width,
                    length,
                    angle=np.degrees(angle),
                    linewidth=1,
                    edgecolor='r',
                    facecolor='none'
                )
                ax.add_patch(rectangle)

        ax.set_aspect('equal', adjustable='box')
        ax.set_title("Grasp and Detection Visualization")
        plt.show()
        return closest_grasp



def main(args=None):
    rclpy.init(args=args)

    generative_grasp_service = GenerativeGraspService()

    rclpy.spin(generative_grasp_service)

    generative_grasp_service.destroy_node()
    rclpy.shutdown()


    