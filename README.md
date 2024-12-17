# ConveyorGrasp
A simulation to Detect, Track, Synthesize Grasp and execute grasp for objects on a moving conveyor belt.

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

### To power the conveyor belt and control it:
```bash
ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 2.5}"
```

![Demo Video](path/to/your-gif.gif)

