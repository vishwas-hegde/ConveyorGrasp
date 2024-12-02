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


class OverHeadCamNode(Node):
    def __init__(self):
        super().__init__('overhead_camera_node')

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
        self.json_pub = self.create_publisher(String, "/yolo/prediction/json", 10)
        self.annotated_frame = None
        self.timer = self.create_timer(0.1, self.run)
        self.get_logger().info("YOLO node with JSON publisher is up and running!")


        self.publisher = self.create_publisher(String, 'object_center/overhead', 10)
        self.tracked_objects = set()

        self.cv_bridge = CvBridge()
        self.overhead_rgb = None
        self.overhead_depth = None
        self.object_world_coords = None
        self.overhead_rgb_sub = self.create_subscription(Image, '/realsense/image_raw', self.overhead_rgb_callback, 10)
        self.overhead_depth_sub = self.create_subscription(Image, '/realsense/depth/image_raw', self.overhead_depth_callback, 10)
        self.prev_msg = None

        self.confidence_threshold = 0.75


    def overhead_rgb_callback(self, msg):
        self.overhead_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

    def overhead_depth_callback(self, msg):
        self.overhead_depth = self.cv_bridge.imgmsg_to_cv2(msg, '32FC1')
    
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


    def convert_to_world_coordinates(self, x,y):
        rgb_image = self.overhead_rgb
        center = (x, y)
        if rgb_image is not None:
            # Get depth value at the center
            depth_value = 0.76

            # Intrinsic parameters (update as per your camera)
            self.fx = 554.256  # Focal length in x
            self.fy = 554.256  # Focal length in y
            self.cx = rgb_image.shape[1] // 2  # Image center x-coordinate
            self.cy = rgb_image.shape[0] // 2  # Image center y-coordinate

            # Calculate 3D position in the camera frame
            y = -(center[0] - self.cx) * depth_value / self.fx
            x = (center[1] - self.cy) * depth_value / self.fy
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
    
    
    def run(self):
        if self.overhead_rgb is not None:
            # results = self.detection_model(self.overhead_rgb, verbose=False)
            results = self.detection_model.track(self.overhead_rgb, persist=True)
            print("Results: ")

            boxes = results[0].boxes.xywh.cpu()
            track_ids = results[0].boxes.id.int().cpu().tolist()
            confidences = results[0].boxes.conf.cpu().tolist()
            classes = results[0].boxes.cls.cpu().tolist()

            filtered_boxes = [box for box, conf in zip(boxes, confidences) if conf >= self.confidence_threshold]
            filtered_ids = [track_id for track_id, conf in zip(track_ids, confidences) if conf >= self.confidence_threshold]
            filtered_classes = [cls for cls, conf in zip(classes, confidences) if conf >= self.confidence_threshold]

            detections = {"detections": []}  # Initialize JSON structure

            for box, track_id, cls in zip(filtered_boxes, filtered_ids, filtered_classes):

                if track_id not in self.tracked_objects:
                    # print(box)
                    x, y, w, h = map(float, box.tolist())
                    world_x, world_y, _ = self.convert_to_world_coordinates(x, y)
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


                    cls = int(cls)
                    # cls_name = self.names[cls]
                    detection = {
                        "id": str(track_id),  # Assign unique ID for each object
                        "class": cls,
                        "bbox": [x_min, y_min, x_max, y_max],
                        "info": {'center':(world_x, world_y), 'width': world_w, 'height': world_h, 'angle': 0},
                    }
                    detections["detections"].append(detection)
                    self.tracked_objects.add(track_id)
                    self.prev_msg = detections

                else:
                    if self.prev_msg is not None:
                        # json_message = String()
                        # json_message.data = json.dumps(self.prev_msg)
                        self.publish_json(self.prev_msg)

            # Annotate and display the image
            annotated_frame = results[0].plot()
            self.annotated_frame = annotated_frame
            cv2.imshow('YOLO Live Predictions', annotated_frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = OverHeadCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
