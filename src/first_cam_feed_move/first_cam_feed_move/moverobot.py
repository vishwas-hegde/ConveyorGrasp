import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped


class MoveRobot(Node):
    
    def __init__(self):
        super().__init__('move_robot')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/panda_camera/image_raw', self.image_callback, 10)
        self.depth_image_sub = self.create_subscription(Image, '/realsense/depth/image_raw', self.depth_image_callback, 10)
        self.rgb_image = None
        self.center = None
        self.move = True
        self.object_world_coords = None

        self.image_publisher = self.create_publisher(
            Image,
            'output_image',
            10)

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.rgb_image = cv_image

        img = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        # self.image_publisher.publish(img)

        self.get_object_center()

    def depth_image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.depth_image = cv_image
        

    def segment_cube(self):
        if self.rgb_image is not None:
            hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([0, 100, 100])
            upper_orange = np.array([25, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)

            # publish mask image
            mask_image = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            self.image_publisher.publish(mask_image)
            return mask

    def get_object_center(self):
        if self.rgb_image is not None and self.move:
            mask = self.segment_cube()
            M = cv2.moments(mask)
            if M["m00"] == 0:
                self.center = None
                return None
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            self.center = (center_x, center_y)
            # print(self.center)

            self.convert_to_world_coordinates()

    def convert_to_world_coordinates(self):
        if self.center is not None and self.rgb_image is not None:
            # Get depth value at the center
            depth_value = 0.76

            # Intrinsic parameters (update as per your camera)
            self.fx = 554.256  # Focal length in x
            self.fy = 554.256  # Focal length in y
            self.cx = self.rgb_image.shape[1] // 2  # Image center x-coordinate
            self.cy = self.rgb_image.shape[0] // 2  # Image center y-coordinate

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
            print(self.object_world_coords)
            
        return object_coords_wrto_world

# Object World Coordinates: 1.1999273659065954 -0.8022992985190958 1.1914901096603978
# Object Camera Coordinates: -0.09093270979475189 -0.10229929851909586 0.7

    def move_robot(self):
        # Assuming you are using some Move action here
        if self.object_world_coords is not None:
            move = MoveXYZ.Goal()
            move.positionx = self.object_world_coords[0]
            move.positiony = self.object_world_coords[1]
            move.positionz = self.object_world_coords[2]
            self._action_client.send_goal_async(move)
            self.move = False

def main(args=None):
    rclpy.init(args=args)
    move_robot = MoveRobot()
    rclpy.spin(move_robot)
    move_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
