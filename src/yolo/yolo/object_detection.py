import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
import json
import cv2 as cv
import sys
sys.path.append('/home/vishwas/Conveyor/src/yolo/yolo')
from sort import Sort  # Install SORT: https://github.com/abewley/sort


class RealSenseSubscriber(Node):

    def __init__(self):
        super().__init__('yolo_json_tracker')

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
        self.annotated_frame = None
        self.annotated_frame_pub = self.create_publisher(Image, "/yolo/annotated_frame", 10)

        # Publishers and subscriptions
        self.json_pub = self.create_publisher(String, "/yolo/prediction/json", 10)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/realsense/image_raw', self.realsense_callback, 1)
        self.image = None
        self.annotated_frame = None
        self.tracker = Sort()  # Initialize the SORT tracker
        self.timer = self.create_timer(0.01, self.run)
        self.get_logger().info("YOLO with centroid-based object tracking is up and running!")

    def realsense_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def publish_json(self, detections):
        """
        Publishes the detections in JSON format.
        """
        json_message = String()
        json_message.data = json.dumps(detections)
        self.json_pub.publish(json_message)
        self.get_logger().info(f"Published JSON: {json_message.data}")

    def run(self):
        if self.image is not None:
            results = self.detection_model(self.image, verbose=False)

            detections = {"detections": []}  # Initialize JSON structure
            centroids = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Parse bounding box
                    x_min, y_min, x_max, y_max = map(float, box.xyxy[0].tolist())
                    confidence = float(box.conf[0])
                    cls = int(box.cls[0])

                    # Compute centroid
                    x_center = (x_min + x_max) / 2
                    y_center = (y_min + y_max) / 2

                    centroids.append([x_center, y_center, confidence, cls])
                    print(centroids)

            # Update tracker with centroids
            tracked_objects = self.tracker.update(np.array(centroids))
            print(tracked_objects)
            for tracked_obj in tracked_objects:
                x_center, y_center, track_id, cls = tracked_obj
                cls_name = self.names[int(cls)]
                detection = {
                    "id": int(track_id),  # Assign unique tracker ID
                    "class": cls_name,
                    "bbox": {'center':(x, y), 'width': w, 'height': h, 'angle': 0}
                }
                detections["detections"].append(detection)

                # Annotate tracked object on the frame
                cv.circle(self.image, (int(x_center), int(y_center)), 5, (0, 255, 0), -1)
                cv.putText(
                    self.image,
                    f"ID {int(track_id)}: {cls_name}",
                    (int(x_center) + 10, int(y_center)),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

            # Publish annotated frame
            annotated_frame_msg = self.bridge.cv2_to_imgmsg(self.image, 'bgr8')
            self.annotated_frame_pub.publish(annotated_frame_msg)

            # Publish detections as JSON
            self.publish_json(detections)

            # Show the annotated frame
            cv.imshow('YOLO Centroid Tracking', self.image)
            cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    realsense_sub = RealSenseSubscriber()

    rclpy.spin(realsense_sub)

    realsense_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
