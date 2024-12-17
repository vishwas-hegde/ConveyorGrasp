# ConveyorGrasp
A simulation to Detect, Track, Synthesize Grasp and execute grasp for objects on a moving conveyor belt.

## Workflow Steps

### 1. **Object Detection and Center Extraction**
- When an object is spawned in the workspace, the YOLO server processes the overhead camera feed.
- The YOLO server provides the center coordinates of the bounding box for the detected object.

### 2. **Object Prioritization**
- The center values of the detected objects (from the overhead camera) are added to a priority queue.
- Objects are prioritized for processing based on the queue order (e.g., first detected).

### 3. **Robot End-Effector Movement**
- The first element (highest priority object) is extracted from the queue.
- The robot end-effector moves to the predefined grasp window based on the object's center coordinates provided by the overhead camera.

### 4. **In-Hand Pose Correction**
- When the object appears under the in-hand camera feed, the YOLO server processes the feed and provides the object's center coordinates again.
- These updated center values are used to correct the pose of the robot end-effector.

### 5. **Grasp Execution**
- The robot end-effector moves down to grasp the object.

### 6. **Object Placement**
- The object is dropped into the designated bin.

### 7. **Home Position or Next Object**
- After placing the object, the robot moves:
  - To the home position, or
  - To the center of the next object in the queue.


### To launch the environment:
```bash
ros2 launch panda_ros2_moveit2 panda_interface.launch.py
```

### To start the open loop grasping:
```bash
ros2 run pandarobotmove pandarobotcloseloopyolo
```

### To spawn the YCB object in our environment:
```bash
ros2 run ros2_grasping spawn_ycb.py --x 0.5 --y -0.7  --name "gelatin_box"
```

### To power the conveyor belt:
```bash
ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 2.5}"
```

### Error Correction with In-hand camera
![](https://github.com/vishwas-hegde/ConveyorGrasp/blob/main/videos/error_corrction_gif.gif)

### Grasping in Cluttered Environment
![](https://github.com/vishwas-hegde/ConveyorGrasp/blob/main/videos/clutter_grasp-giff.gif)

