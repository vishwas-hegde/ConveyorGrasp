#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
import numpy as np

class GetObjectPos(Node):
    def __init__(self):
        super().__init__('get_object_pose_node')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
    def get_object_world_coordinates(self, camera_point):
        """
        Transform object coordinates from camera frame to robot base frame
        
        Args:
            camera_point: Point in camera frame (x, y, z in meters)
            
        Returns:
            transformed_point: Point in robot base frame
        """
        # Create a PointStamped message
        image_coords = PointStamped()
        image_coords.header.frame_id = "camera_link_optical" 
        image_coords.header.stamp = self.get_clock().now().to_msg()
        image_coords.point.x = camera_point[0]
        image_coords.point.y = camera_point[1]
        image_coords.point.z = camera_point[2]
        
        try:
            # Look up the transform between camera and robot base
            transform = self.tf_buffer.lookup_transform(
                "panda_link0",  # target 
                "camera_link_optical",  # source
                rclpy.time.Time(),  
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Apply transform to point
            transformed_point = do_transform_point(image_coords, transform)
            
            return [
                transformed_point.point.x,
                transformed_point.point.y,
                transformed_point.point.z
            ]
            
        except TransformException as e:
            self.get_logger().error(f"Transform failed: {e}")
            return None
        
    def get_world_coords_manually(self, camera_point):

        transform_matrix = np.array([
            [0.001, 0.000, 1.000, 0.500],
            [0.000, 1.000, 0.000, -0.700],
            [-1.000, 0.000, 0.001, 1.100],
            [0.000, 0.000, 0.000, 1.000]
        ])

        x, y, z = camera_point

        object_coords_camera = np.array([x, y, z, 1])  
        world_coords = transform_matrix @ object_coords_camera
        x_world, y_world, z_world = world_coords[:3]

        return [x_world, y_world, z_world]

def main():
    rclpy.init()
    
    # Create and use the localizer
    localizer = GetObjectPos()
    
    try:
        # Example object centroid in camera frame
        object_pos_camera = [0.3, 0.2, 0.5]  # x, y, z in meters
        
        # Get world coordinates
        world_coords = localizer.get_object_world_coordinates(object_pos_camera)
        
        if world_coords:
            localizer.get_logger().info(f"Object world coordinates: {world_coords}")
            
    except KeyboardInterrupt:
        pass
    
    finally:
        localizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()