# ConveyorGrasp
A simulation to Detect, Track, Synthesize Grasp and execute grasp for objects on a moving conveyor belt


ros2 topic pub /robotaction std_msgs/msg/String "data: '{\"action\": \"MoveXYZW\", \"value\": {\"positionx\": 0.5, \"positiony\": 0.05, \"positionz\": 0.6, \"yaw\": 45.00, \"pitch\": 180.00, \"roll\": 0.00}, \"speed\": 1.0}'" --once

To spawn the ycb object:
ros2 run ros2_grasping spawn_ycb.py --name "sugar_box" --x 0.5 --y -0.7

To run yolo for overhead:
ros2 run image_processing overhead_cam_process 

To run grasp service:
ros2 run grasp_gen_service grasp_gen_service 

### To get the grasp:
```bash
ros2 service call /generate_grasp grasp_gen_interface/srv/GraspGen "{input: '{\"grasp_type\": \"generate_grasp_grconvnet\", \"crop\": [230, 191, 391, 262]}'}"




