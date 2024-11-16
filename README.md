# ConveyorGrasp
A simulation to Detect, Track, Synthesize Grasp and execute grasp for objects on a moving conveyor belt


ros2 topic pub /robotaction std_msgs/msg/String "data: '{\"action\": \"MoveXYZW\", \"value\": {\"positionx\": 0.5, \"positiony\": 0.05, \"positionz\": 0.6, \"yaw\": 45.00, \"pitch\": 180.00, \"roll\": 0.00}, \"speed\": 1.0}'" --once

