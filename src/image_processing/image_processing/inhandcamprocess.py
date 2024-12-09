import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import uuid  # To generate unique IDs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import tf2_ros


class PandaCamNode(Node):
    def __init__(self):
        super().__init__('panda_camera_node')

        # YOLO model setup
        package_name = 'yolo'
        ros2_execution_package_share_path = get_package_share_directory(package_name)
        path = ros2_execution_package_share_path.split('/')
        index = path.index(package_name)
        path[index - 1] = "src"
        yolo_package_path = '/'.join(path[:index + 1])
        model_path = os.path.join(yolo_package_path, 'model', 'yolo_ycb.pt')

        self.detection_model = YOLO(model_path)
        folder_path = os.path.join(os.getcwd(), "src", "yolo", "yolo", "yolo_finetune", "ycb_foods")
        yaml_path = os.path.join(folder_path, "data.yaml")
        with open(yaml_path, 'r') as file:
            self.yaml_data = yaml.safe_load(file)
        self.ycb_names = self.yaml_data['names']
        self.names = self.detection_model.names

        # Publishers and subscriptions
        self.json_pub = self.create_publisher(String, "/yolo/inhand", 10)
        self.annotated_frame = None
        self.annotated_frame_pub = self.create_publisher(Image, '/yolo/inhand_annotated', 10)
        self.timer = self.create_timer(0.5, self.run)
        self.get_logger().info("YOLO node with JSON publisher is up and running!")


        self.publisher = self.create_publisher(String, 'object_center/panda', 10)
        self.tracked_objects = set()

        self.cv_bridge = CvBridge()
        self.panda_rgb = None
        self.panda_depth = None
        self.object_world_coords = None
        self.panda_rgb_sub = self.create_subscription(Image, '/panda_camera/image_raw', self.panda_rgb_callback, 10)
        self.panda_depth_sub = self.create_subscription(Image, '/panda_camera/depth/image_raw', self.panda_depth_callback, 10)
        self.prev_msg = None

        self.confidence_threshold = 0.2

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    def panda_rgb_callback(self, msg):
        self.panda_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

    def panda_depth_callback(self, msg):
        self.panda_depth = self.cv_bridge.imgmsg_to_cv2(msg, '32FC1')
    
    def publish_json(self, detections):
        """
        Publishes the detections in JSON format.
        """
        json_message = String()
        json_message.data = json.dumps(detections)
        self.json_pub.publish(json_message)
        self.get_logger().info(f"Published JSON: {json_message.data}")
    
    def calculate_distance(self, c1, c2):
        """
        Calculate the Euclidean distance between two centers.
        """
        return ((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2) ** 0.5
    
    def calculate_angle(self, c1, c2):
        """
        Calculate the angle between two centers.
        """
        angle = np.arctan2(c2[1] - c1[1], c2[0] - c1[0])
        # convert to degrees
        return np.degrees(angle)

    def transform_to_matrix(self, quaternion):
        """Convert translation and quaternion to a 4x4 transformation matrix."""
        from scipy.spatial.transform import Rotation as R

        # Create a rotation matrix from quaternion
        rotation_matrix = R.from_quat(quaternion).as_matrix()

        # Construct the 4x4 transformation matrix
        # transform_matrix = np.eye(4)
        # transform_matrix[:3, :3] = rotation_matrix
        # transform_matrix[:3, 3] = translation

        return rotation_matrix

    def convert_to_world_coordinates(self, x, y):
        rgb_image = self.panda_rgb
        center = (x, y)
        if center is not None and rgb_image is not None:

            # R_c_w = np.array([[0, 0, 1],
            #                   [-0.001, 1, 0.001],
            #                   [1, 0, 0]])
            R_c_w = np.array([[0, 1, 0],
                               [-0.001, 0, -1.001],
                               [-1, 0, 0]])
            try:
                H_c_w = self.tf_buffer.lookup_transform(
                        'panda_camera_optical_link',  # Target frame
                        'panda_link0',  # Target frame
                         rclpy.time.Time(),
                        timeout=rclpy.time.Duration(seconds=5.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f'TF Lookup Failed: {str(e)}')
                return None
            
            T_c_w = np.array([H_c_w.transform.translation.x, H_c_w.transform.translation.y, H_c_w.transform.translation.z])

            # Get depth value at the center
            depth_value = 0.34 - T_c_w[0] #box is at 34cm in world frame
            quaternion = [H_c_w.transform.rotation.x, H_c_w.transform.rotation.y, H_c_w.transform.rotation.z, H_c_w.transform.rotation.w]
            R_c_w = np.array(self.transform_to_matrix(quaternion))
            

            # Intrinsic parameters (update as per your camera)
            self.fx = 554.256  # Focal length in x
            self.fy = 554.256  # Focal length in y
            self.cx = rgb_image.shape[1] // 2  # Image center x-coordinate
            self.cy = rgb_image.shape[0] // 2  # Image center y-coordinate

            # Calculate 3D position in the camera frame
            y = (center[0] - self.cx) * depth_value / self.fx
            x = (center[1] - self.cy) * depth_value / self.fy
            z = depth_value
            # print("Object Camera Coordinates:", x, y, z)

            P_camera = np.array([x, y, z])  # Point in camera frame
            P_world = np.dot(R_c_w, P_camera) + T_c_w

            self.object_world_coords = np.array([P_world[2], P_world[1], P_world[0]])
            object_coords_wrto_world = self.object_world_coords.tolist()
            
        return object_coords_wrto_world
    
    
    def run(self):
        if self.panda_rgb is not None:
            # results = self.detection_model(self.panda_rgb, verbose=False)
            results = self.detection_model.track(self.panda_rgb, persist=True)
            
            if results[0].boxes is None or len(results[0].boxes) == 0:
                return
            if results[0].boxes.id is None:
                return
            boxes = results[0].boxes.xywh.cpu()
            track_ids = results[0].boxes.id.int().cpu().tolist()
            confidences = results[0].boxes.conf.cpu().tolist()
            classes = results[0].boxes.cls.cpu().tolist()

            filtered_boxes = [box for box, conf in zip(boxes, confidences) if conf >= self.confidence_threshold]
            filtered_ids = [track_id for track_id, conf in zip(track_ids, confidences) if conf >= self.confidence_threshold]
            filtered_classes = [cls for cls, conf in zip(classes, confidences) if conf >= self.confidence_threshold]

            detections = {"detections": []}  # Initialize JSON structure
            print("HI")
            for box, track_id, cls in zip(filtered_boxes, filtered_ids, filtered_classes):
                print("HI")
                if track_id is not None:# and track_id not in self.tracked_objects:
                    # print(box)
                    x, y, w, h = map(float, box.tolist())
                    world_x, world_y, _ = self.convert_to_world_coordinates(x, y)
                    print(world_x, world_y)
                    #get xmin, ymin, xmax, ymax
                    x_min = int(x - w / 2)
                    y_min = int(y - h / 2)
                    x_max = int(x + w / 2)
                    y_max = int(y + h / 2)

                    world_x_min, world_y_min, _ = self.convert_to_world_coordinates(x_min, y_min)
                    world_x_max, world_y_max, _ = self.convert_to_world_coordinates(x_max, y_max)
                    #get world width and height
                    world_w = abs(world_x_max - world_x_min)
                    world_h = abs(world_y_max - world_y_min)

                    angle = self.calculate_angle((x_min, y_min), (x_max, y_max))
                    cls = int(cls)
                    # cls_name = self.names[cls]
                    detection = {
                        "id": str(track_id),  # Assign unique ID for each object
                        "class": cls,
                        "bbox": [x_min, y_min, x_max, y_max],
                        "info": {'center':(world_x, world_y), 'width': world_w, 'height': world_h, 'angle': angle},
                    }
                    detections["detections"].append(detection)
                    # self.tracked_objects.add(track_id)
                    self.prev_msg = detections
                    # self.publish_json(self.prev_msg)

                # else:
                if self.prev_msg is not None:
                    # json_message = String()
                    # json_message.data = json.dumps(self.prev_msg)
                    self.publish_json(self.prev_msg)

            # Annotate and display the image
            annotated_frame = results[0].plot()
            self.annotated_frame = annotated_frame
            self.annotated_frame_pub.publish(self.cv_bridge.cv2_to_imgmsg(annotated_frame, 'bgr8'))
            # cv2.imshow('YOLO Live Predictions', annotated_frame)
            # cv2.waitKey(1)
   
    

def main(args=None):
    rclpy.init(args=args)
    node = PandaCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

