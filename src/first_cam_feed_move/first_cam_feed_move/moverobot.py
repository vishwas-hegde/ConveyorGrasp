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
        self.image_sub = self.create_subscription(Image, '/realsense/image_raw', self.image_callback, 10)
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

    def depth_image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.depth_image = cv_image
        self.get_object_center()

    def segment_cube(self):
        if self.rgb_image is not None:
            hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([10, 100, 100])
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
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            self.center = (center_x, center_y)
            #show the image with a point at the center
            # cv2.circle(self.rgb_image, (center_x, center_y), 5, (255, 255, 255), -1)
            # cv2.imshow("Image window", self.rgb_image)
            # cv2.waitKey(3)

            self.convert_to_world_coordinates()

    def convert_to_world_coordinates(self):
        if self.center is not None and self.rgb_image is not None:
            # convert pixel coordinates to world coordinates
            image_dim_x, image_dim_y, _ = self.rgb_image.shape

            self.fx = 554.256
            self.fy = 554.256
            self.cx = image_dim_x // 2 
            self.cy = image_dim_y // 2

            # Calculate 3D position
            depth_value = 0.7
            x = (self.center[0] - self.cx) * depth_value / self.fx
            y = (self.center[1] - self.cy) * depth_value / self.fy
            z = depth_value

            print("Object Camera Coordinates:", x, y, z)

            # Create the PointStamped object for the camera coordinates
            object_coords_camera = PointStamped()
            object_coords_camera.header.frame_id = 'camera_link_optical' 
            object_coords_camera.point.x = x
            object_coords_camera.point.y = y
            object_coords_camera.point.z = z

            try:
                transform = self.tf_buffer.lookup_transform('world', 'camera_link_optical', rclpy.time.Time())
                transformed_point = tf2_geometry_msgs.do_transform_point(object_coords_camera, transform)
                
                # Print the transformed coordinates in world frame
                print("Object World Coordinates:", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)
                self.object_world_coords = np.array([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])

            except (tf2_ros.TransformException, Exception) as e:
                self.get_logger().error(f"Error transforming coordinates: {e}")
                self.object_world_coords = None
                return None

            return self.object_world_coords

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
