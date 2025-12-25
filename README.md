# Robot-Automation

**우선 해당 링크에 들어가서 튜토리얼을 진행한다**
https://github.com/ykkimhgu/HGU_IAIA/tree/main/Tutorial/TU_ROS_v2

## 1. Structure

```
catkin_ws
	|- build
	|- devel
	|- packages_from_git
	|- src
	|----|- CMakeLists.txt
	|----|- 22000167_Kiminyeop_ur_python
	|----|----|- msg
	|----|----|----| grip_command.msg
	|----|----|----| grip_state.msg
	|----|----|----| object_info.msg
	|----|----|----| pet_info.msg
	|----|----|----| robot_state.msg
	|----|----|- src
	|----|----|----| camera.py
	|----|----|----| cell_detection.py
	|----|----|----| composite_detection.py
	|----|----|----| empty_cell_box_detection_node.py
	|----|----|----| empty_composite_cellbox_detection_node.py
	|----|----|----| filled_cellbox_detection_node.py
	|----|----|----| filled_composite_cellbox_detection_node.py
	|----|----|----| main.py
	|----|----|----| move_group_python_interface.py
	|----|----|----| winding_detect_node.py
	|----|----|----| yellow_bracket_detection_node.py
```
`my_robot_calibration.yaml` will add to the home

## 2. Command

1. If you saved my_robot_calibration.yaml to your home directory, run the following. If the path doesn't exist, you'll need to reset it to match your computer's path.
```
chmod+x/~my_robot_calibration.yaml
```

2. Since the project is based on ROS environment, every command line is done under catkin workspace directory.
```
cd~/catkin_ws
catkin_make
roscore
```

3. To activate the system, the robot should be connected to a computer. Communication protocol is TCP/IP based therefore, IP should be set in proper manner
```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2
roslaunch ur5e_rg2_moveit_config move_group.launch
```

4. Connect camara in UR5e
```
chmod +x ~/catkin_ws/src/ur_python/src/camera.py
rosrun 22000167_Kiminyeop_ur_python camera.py
```

5. Follow the steps below

- Empty Bracket Detection: This detection node detects yellow bracket holes.
```
chmod +x ~/catkin_ws/src/ur_python/src/yellow_bracket_detection_node.py
rosrun 22000167_Kiminyeop_ur_python yellow_bracket_detection_node.py
```

- Cell Box Cell Detection: This detection node detects cells in empty cell boxes.
```
chmod +x ~/catkin_ws/src/ur_python/src/filled_cellbox_detection_node.py
rosrun 22000167_Kiminyeop_ur_python filled_cellbox_detection_node.py
```

- Empty Completed Box Detection: This detection node detects empty boxes in completed cell boxes. 
```
chmod +x ~/catkin_ws/src/ur_python/src/empty_composite_cellbox_detection_node.py
rosrun 22000167_Kiminyeop_ur_python empty_composite_cellbox_detection_node.py
```

- Completed Cell Box Detection: This detection node detects cells in a completed cell box.
```
chmod +x ~/catkin_ws/src/ur_python/src/filled_composite_cellbox_detection_node.py
rosrun 22000167_Kiminyeop_ur_python filled_composite_cellbox_detection_node.py
```

- Empty Cell Box Detection: This detection node detects empty boxes in an empty cell box. 
```
chmod +x ~/catkin_ws/src/ur_python/src/empty_cell_cellbox_detection_node.py
rosrun 22000167_Kiminyeop_ur_python empty_cell_cellbox_detection_node.py
```

- Orange Winding Detection: This detection node detects orange windings.
```
chmod +x ~/catkin_ws/src/ur_python/src/winding_detect_node.py
rosrun 22000167_Kiminyeop_ur_python winding_detect_node.py
```

- Completed Cell Detection: This detection node detects completed cells. 
```
chmod +x ~/catkin_ws/src/ur_python/src/composite_detection.py
rosrun 22000167_Kiminyeop_ur_python composite_detection.py
```

8. Cell Detection: This detection node detects empty cells.
```
chmod +x ~/catkin_ws/src/ur_python/src/cell_detection.py
rosrun 22000167_Kiminyeop_ur_python cell_detection.py
```

- Run Arduino Stopper
	After installing Arduino 1.8.19, confirm that the board is Arduino Uno in the tool bar and that the port is dev/tty/ACM0. Confirm and upload, then close the Arduino window.

10. Enter VScode and run main.py.

## 3 Troubleshooting

1. If the robot movement doesn't work, it's almost always due to a problem with 
```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2
``` 
Therefore, undoing and re-running this terminal should resolve the issue.

2. If you run the code 
```
chmod +x ~/catkin_ws/src/ur_python/src/camera.py
rosrun 22000167_Kiminyeop_ur_python camera.py
```
the camera may not be connected to the ur5e robot but to the laptop. In this case, adjust the numeric value of `self.cap = cv2.VideoCapture(2)`
from 0 to 5 and then run the program.
