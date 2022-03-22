# mate_ros2_projects

ROS2 projects.

## install

Assuming the ROS2 workspace is ```~/ros2_ws``` and the ```~/ros2_ws/install/local_setup.bash``` is set up (sourced) in the ```~/.bashrc```.
The package can be downloaded and installed by:
```
cd ~/ros2_ws/src
git clone https://github.com/g0mb4/mate_ros2_projects.git
cd ..
colcon build
source ~/.bashrc
```

## dummy_camera

A virtual camera that can output images into the ROS network emulating a real camera.

### qucikstart
```
ros2 launch dummy_camera start.launch.py
```

## turtle_catcher

A position control demonstartion using turtlesim. The *game_master* will spawn a target turtle at random position and the *turtle1_controller* drives the turtle to reach the target.

### qucikstart
```
ros2 launch turtle_catcher start.launch.py
```

## qt_turtle

Qt5 integration demo, it will show the position of the turtle.

### qucikstart
```
ros2 launch qt_turtle start.launch.py
```

## web_turtle

Webbrowser (JavaScript) integration demo, it will start a webserver and allows the user to drive the turtle via a browser.

### qucikstart
```
ros2 launch web_turtle start.launch.py
```

## rviz_turtle

Rviz 3D visualization tool demonstration, shows a simple URDF model of the turtle in Rviz.

### qucikstart
```
ros2 launch rviz_turtle start.launch.py
```

## gazebo_demo

gazebo simulation demo using a differential drive robot with a lidar.

### qucikstart
```
ros2 launch gazebo_demo start.launch.py
```

