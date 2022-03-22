# mate_ros2_projects

ROS2 projects.

## install

The assumbed operating system is Ubuntu 20.04. The required packages are the extra packages besides the ```ros-foxy-desktop``` version.

Assuming the ROS2 workspace is ```~/ros2_ws``` and the ```~/ros2_ws/install/local_setup.bash``` is set up (sourced) in the ```~/.bashrc```,
the package can be downloaded and installed by:
```
cd ~/ros2_ws/src
git clone https://github.com/g0mb4/mate_ros2_projects.git
cd ..
colcon build
source ~/.bashrc
```

## dummy_camera

A virtual camera that can output PNG images into the ROS network emulating a real camera.

### qucikstart
```
ros2 launch dummy_camera start.launch.py
```

### dependencies
```
sudo add-apt-repository ppa:linuxuprising/libpng12
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt install -y locate libpng12-0 libjasper1 libjasper-dev
```

### new topics
+ using external C++ libraries (OpenCV)
+ ```RCLCPP FATAL()``` macro
+ topic remapping
+ change the name of the node
+ ```--symlink-install```

## turtle_catcher

A position control demonstartion using turtlesim. The *game_master* will spawn a target turtle at random position and the *turtle1_controller* drives the turtle to reach the target.

### qucikstart
```
ros2 launch turtle_catcher start.launch.py
```

### new topics
+ position control
+ intarface within a package
+ chaining async service calls

## qt_turtle

Qt5 integration demo, it will show the position of the turtle.

### qucikstart
```
ros2 launch qt_turtle start.launch.py
```

### dependencies
```
sudo apt update
sudo apt install -y qt5-default qtcreator
```

### new topics
+ Qt5 framework

## web_turtle

Browser (JavaScript) integration demo, it will start a webserver and allows the user to drive the turtle using a browser.

### qucikstart
```
ros2 launch web_turtle start.launch.py
```

### dependencies
```
pip3 install tornado
cd ros2_ws/src
git clone https://github.com/RobotWebTools/rosbridge_suite.git
cd ..
colcon build
```

### new topics
+ ```rosbridge```
+ JavaScript
+ andvanced launch files

## rviz_turtle

Rviz 3D visualization tool demonstration, shows a simple URDF model of the turtle in Rviz.

### qucikstart
```
ros2 launch rviz_turtle start.launch.py
```

### new topics
+ ```rviz```
+ URDF format
+ robot state
+ transform functions
+ TF tree

## gazebo_demo

gazebo simulation demo using an SDF model differential drive robot with a lidar, it will start the gazebo simulator and the Rviz tool.

### qucikstart
```
ros2 launch gazebo_demo start.launch.py
```

### dependencies
```
sudo apt update
sudo apt install ros-foxy-gazebo-ros-pkgs
```

### new topics
+ ```gazebo```
+ SDF format

