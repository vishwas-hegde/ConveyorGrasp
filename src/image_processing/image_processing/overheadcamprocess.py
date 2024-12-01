import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import uuid  # To generate unique IDs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class OverHeadCamNode(Node):
    def __init__(self):
        super().__init__('overhead_camera_node')
        self.publisher = self.create_publisher(String, 'object_center/overhead', 10)
        self.timer = self.create_timer(0.5, self.detect_and_publish)
        self.tracked_objects = {}  # Dictionary to track objects with their IDs

        self.cv_bridge = CvBridge()
        self.overhead_rgb = None
        self.overhead_depth = None
        self.object_world_coords = None
        self.overhead_rgb_sub = self.create_subscription(Image, '/realsense/image_raw', self.overhead_rgb_callback, 10)
        self.overhead_depth_sub = self.create_subscription(Image, '/realsense/depth/image_raw', self.overhead_depth_callback, 10)
        self.id = 0
        self.prev_msg = None

    def overhead_rgb_callback(self, msg):
        self.overhead_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

    def overhead_depth_callback(self, msg):
        self.overhead_depth = self.cv_bridge.imgmsg_to_cv2(msg, '32FC1')

    def detect_and_publish(self):
        # Simulate detection of object centers
        detected_centers = [self.get_object_center()]
        # print(detected_centers)

        # print(self.tracked_objects)
        for detected_center in detected_centers:
            if detected_center is None:
                continue
            if not self.is_existing_object(detected_center):
                # Treat as a new object
                # new_id = str(uuid.uuid4())
                # new_id = str(self.id + 1)
                self.id += 1
                self.tracked_objects = {}
                self.tracked_objects[self.id] = detected_center  # Add to tracked objects
                
                # Create the JSON message
                message = {
                    "id": str(self.id),
                    "center": detected_center
                }
                json_message = String()
                json_message.data = json.dumps(message)
                self.publisher.publish(json_message)
                self.get_logger().info(f'Published from Over head camera: {json_message.data}')
                self.prev_msg = message
            
            else:
                # update the center of the existing object
                for obj_id, obj_center in self.tracked_objects.items():
                    if self.calculate_distance(detected_center, obj_center) < 0.1:
                        self.tracked_objects[obj_id] = detected_center
                        break
                # publish the previous message
                if self.prev_msg is not None:
                    json_message = String()
                    json_message.data = json.dumps(self.prev_msg)
                    self.publisher.publish(json_message)
                    # self.get_logger().info(f'Published: {json_message.data}')
    
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
            hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([10, 100, 100])
            upper_orange = np.array([25, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)
            return mask
    
    def get_object_center(self):
        rgb_image = self.overhead_rgb
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

            return self.convert_to_world_coordinates()

    def convert_to_world_coordinates(self):
        rgb_image = self.overhead_rgb
        if rgb_image is not None:
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
            # convert to list
            object_coords_wrto_world = object_coords_wrto_world.tolist()
            
        return object_coords_wrto_world

def main(args=None):
    rclpy.init(args=args)
    node = OverHeadCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
