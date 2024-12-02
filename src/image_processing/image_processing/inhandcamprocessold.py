import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import uuid  # To generate unique IDs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from matplotlib import pyplot as plt
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped

class InHandCamNode(Node):
    def __init__(self):
        super().__init__('inhand_camera_node')
        self.publisher = self.create_publisher(String, 'object_center/inhand', 10)
        self.timer = self.create_timer(0.5, self.detect_and_publish)
        self.tracked_objects = {}  # Dictionary to track objects with their IDs

        self.cv_bridge = CvBridge()
        self.inhand_rgb = None
        self.inhand_depth = None
        self.object_world_coords = None
        self.inhand_rgb_sub = self.create_subscription(Image, '/panda_camera/image_raw', self.inhand_rgb_callback, 10)
        self.inhand_depth_sub = self.create_subscription(Image, '/panda_camera/depth/image_raw', self.inhand_depth_callback, 10)
        self.id = 0
        self.prev_msg = None
        self.output_image_pub = self.create_publisher(Image, 'inhand_camera_process/output_image', 10)

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def inhand_rgb_callback(self, msg):
        self.inhand_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

    def inhand_depth_callback(self, msg):
        self.inhand_depth = self.cv_bridge.imgmsg_to_cv2(msg, '32FC1')

    def detect_and_publish(self):
        data = self.get_object_center_and_dimensions()
        if data is None:
            return
        world_center, dimensions = data[0], data[1]
        if world_center is None or dimensions is None:
            return
        
        detected_center = world_center
        length, breadth, angle = dimensions

        if breadth > 600:
            return

        message = {
                "center": detected_center,
                "length": length,
                "breadth": breadth,
                "angle": angle
            }
        json_message = String()
        json_message.data = json.dumps(message)
        self.publisher.publish(json_message)
        self.get_logger().info(f'Published from In-Hand Camera: {json_message.data}')
        
        
        # if not self.is_existing_object(detected_center):
        #     self.id += 1
        #     self.tracked_objects[self.id] = detected_center
            
        #     # Create the JSON message
        #     message = {
        #         "id": str(self.id),
        #         "center": detected_center,
        #         "length": length,
        #         "breadth": breadth,
        #         "angle": angle
        #     }
        #     json_message = String()
        #     json_message.data = json.dumps(message)
        #     self.publisher.publish(json_message)
        #     self.get_logger().info(f'Published: {json_message.data}')
        #     self.prev_msg = message
        # else:
        #     if self.prev_msg is not None:
        #         json_message = String()
        #         json_message.data = json.dumps(self.prev_msg)
        #         self.publisher.publish(json_message)

    
    def is_existing_object(self, center):
        """
        Check if the detected center matches an existing tracked object.
        Return True if it matches, False otherwise.
        """
        for obj_center in self.tracked_objects.values():
            if self.calculate_distance(center, obj_center) < 0.1:
                return True
        return False
    
    def calculate_distance(self, c1, c2):
        """
        Calculate the Euclidean distance between two centers.
        """
        return ((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2) ** 0.5
        
    def segment_cube(self, rgb_image):
        if rgb_image is not None:
            # Convert the image to grayscale
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

            _, edges = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

            return edges

    def get_object_center_and_dimensions(self):
        rgb_image = self.inhand_rgb
        if rgb_image is not None:
            # Detect edges
            edges = self.segment_cube(rgb_image)
            # edges = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            if edges is None:
                return None
            
            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None
            
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Calculate the minimum area bounding rectangle
            rect = cv2.minAreaRect(largest_contour)
            (center_x, center_y), (length, breadth), angle = rect
            
            # Correct the angle to ensure it is in the range [0, 180)
            if length < breadth:
                angle = angle + 90  # Ensure the angle is relative to the longer side
            
            # Optionally, visualize
            # debug_image = rgb_image.copy()
            # box = cv2.boxPoints(rect)
            # box = np.int0(box)  # Convert to integer coordinates
            # cv2.drawContours(debug_image, [box], 0, (0, 255, 0), 2)  # Draw rectangle
            # cv2.circle(debug_image, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)  # Draw center
            # # cv2.imshow('Debug View', debug_image)
            # # cv2.waitKey(1)
            # plt.imshow(debug_image)
            # plt.show()
            # publish debug image
            # output_image_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            # self.output_image_pub.publish(output_image_msg)
            
            # Convert center to world coordinates
            self.center = (int(center_x), int(center_y))
            world_center = self.convert_to_world_coordinates()
            # print('hi')
            
            return [world_center, (length, breadth, angle)]

        return None
        
    # def convert_to_world_coordinates(self):
    #     rgb_image = self.inhand_rgb
    #     if rgb_image is not None:
    #         # Get depth value at the center
    #         depth_value = 0.76

    #         # Intrinsic parameters (update as per your camera)
    #         self.fx = 554.256  # Focal length in x
    #         self.fy = 554.256  # Focal length in y
    #         self.cx = rgb_image.shape[1] // 2  # Image center x-coordinate
    #         self.cy = rgb_image.shape[0] // 2  # Image center y-coordinate

    #         # Calculate 3D position in the camera frame
    #         y = -(self.center[0] - self.cx) * depth_value / self.fx
    #         x = (self.center[1] - self.cy) * depth_value / self.fy
    #         z = depth_value
    #         # print("Object Camera Coordinates:", x, y, z)

    #         R_c_w = np.array([[-1, 0, 0],
    #                       [0, 1, 0],
    #                       [0, 0, -1]])
    #         T_c_w = np.array([0.5 , -0.7 , 1.12])
    #         T_w = np.array([0, 0, 0])

    #         object_coordinates_wrto_camera = [x,y,z]
    #         camera_coords_wrto_world = R_c_w @ T_w + T_c_w
    #         object_coords_wrto_world = R_c_w @ (object_coordinates_wrto_camera) + camera_coords_wrto_world

    #         self.object_world_coords = np.array(object_coords_wrto_world)
    #         # print("camera_coords_wrto_world",camera_coords_wrto_world)
    #         # print(object_coords_wrto_world)
    #         # convert to list
    #         object_coords_wrto_world = object_coords_wrto_world.tolist()
            
    #     return object_coords_wrto_world
    
    def convert_to_world_coordinates(self):
        rgb_image = self.inhand_rgb
        if self.center is not None and rgb_image is not None:

            R_c_w = np.array([[0, 0, 1],
                              [-0.001, 1, 0.001],
                              [1, 0, 0]])
            try:
                H_c_w = self.tf_buffer.lookup_transform(
                        'panda_camera_optical_link',  # Target frame
                        'panda_link0',  # Target frame
                         rclpy.time.Time(),
                        timeout=rclpy.time.Duration(seconds=5.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f'TF Lookup Failed: {str(e)}')
                return None
            
            # T_c_w = np.array([0.52, -0.03 , 0.5])
            T_c_w = np.array([H_c_w.transform.translation.x, H_c_w.transform.translation.y, H_c_w.transform.translation.z])
            # print("T_c_w: ", T_c_w)


            # Get depth value at the center
            depth_value = 0.34 - T_c_w[0] #box is at 34cm in world frame


            # Intrinsic parameters (update as per your camera)
            self.fx = 554.256  # Focal length in x
            self.fy = 554.256  # Focal length in y
            self.cx = rgb_image.shape[1] // 2  # Image center x-coordinate
            self.cy = rgb_image.shape[0] // 2  # Image center y-coordinate

            # Calculate 3D position in the camera frame
            y = (self.center[0] - self.cx) * depth_value / self.fx
            x = (self.center[1] - self.cy) * depth_value / self.fy
            z = depth_value
            # print("Object Camera Coordinates:", x, y, z)

            P_camera = np.array([x, y, z])  # Point in camera frame
            P_world = np.dot(R_c_w, P_camera) + T_c_w

            self.object_world_coords = np.array([P_world[2], P_world[1], P_world[0]])
            object_coords_wrto_world = self.object_world_coords.tolist()
            
        return object_coords_wrto_world
    

def main(args=None):
    rclpy.init(args=args)
    node = InHandCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

