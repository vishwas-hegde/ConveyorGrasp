#!/usr/bin/env python3

import argparse
import os
import random
import time
import xacro
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy

def main():
    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Continuously spawn objects into our Gazebo world at a fixed location.')
    parser.add_argument('--package', type=str, default='ros2_grasping', help='Package where URDF/XACRO files are located.')
    parser.add_argument('--name', type=str, default='box', help='Base name of the objects to spawn.')
    parser.add_argument('--x', type=float, default=0.5, help='the x component of the initial position [meters].')
    parser.add_argument('--y', type=float, default=-0.5, help='the y component of the initial position [meters].')
    parser.add_argument('--z', type=float, default=1.5, help='the z component of the initial position [meters].')
    
    args, unknown = parser.parse_known_args()

    # URDF file paths for orange and green boxes
    urdf_paths = {
        'orange': "/home/vishwas/VBM_project/test3/src/ros2_grasping/urdf/orange_box.urdf",
        'green': "/home/vishwas/VBM_project/test3/src/ros2_grasping/urdf/green_box.urdf"
    }

    # Start node
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    # Create client to communicate with `/spawn_entity` service
    node.get_logger().info('Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')
    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service not available! Exiting...')
        return

    spawn_count = 0
    try:
        while rclpy.ok():
            # Randomly choose which box to spawn
            # color_choice = random.choice(['orange', 'green'])
            # For 80% orange, 20% green:
            color_choice = random.choices(['orange', 'green'], weights=[0.8, 0.2])[0]
            urdf_file_path = urdf_paths[color_choice]

            # Set up spawn request
            request = SpawnEntity.Request()
            request.name = f"{args.name}_{color_choice}_{spawn_count}"  # Unique name for each spawn
            
            # Read and process the URDF file
            with open(urdf_file_path, 'r') as file:
                request.xml = file.read()

            # Set fixed position without randomization 
            request.initial_pose.position.x = args.x
            request.initial_pose.position.y = args.y
            request.initial_pose.position.z = args.z

            # Spawn the object
            node.get_logger().info(f'Spawning `{request.name}` as a `{color_choice}` box at ({request.initial_pose.position.x}, {request.initial_pose.position.y}, {request.initial_pose.position.z})')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)

            if future.result() is not None:
                node.get_logger().info(f'Successfully spawned `{request.name}`')
            else:
                node.get_logger().error(f'Failed to spawn `{request.name}`')

            # Increment counter and pause before next spawn
            spawn_count += 1
            time.sleep(20)  # Delay of 20 secs before spawning the next box

    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown
        node.get_logger().info('Done! Shutting down node.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()