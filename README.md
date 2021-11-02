# dag_case

# 1. Install ROS foxy and other libraries
```
sudo apt-get install ros-foxy-pluginlib
sudo apt-get install ros-foxy-rcl-interfaces
```
# 2. Building
## About parking node
This node was modified in C++ based on the python demo project https://github.com/ROBOTIS-GIT/turtlebot3_applications/tree/ros2-devel/turtlebot3_automatic_parking.

## Build filter
```
cd your_workspace/src
git clone -b foxy https://github.com/ros/filters.git
```

After cloning to your workspace, delete the line `<filters plugin="default_plugins.xml"/>` in package.xml file. And add this line 'pluginlib_export_plugin_description_file(filters default_plugins.xml)' to CMakeLists.txt file after `ament_target_dependencies` command.

```
cd your_workspace
colcon build --packages-select filters --symlink-install
```
## Add and build the laser filter plugin
```
cd your_workspace/src
git clone https://github.com/ruoxianglee/laser_filters.git
cd your_workspace
colcon build --packages-select laser_filters --symlink-install
```
## Build this project
```
cd your_workspace
colcon build --packages-select dag_case --symlink-install
```
# 3. Run
```
ros2 run dag_case dag_case
```

# Github

```
git add .
git commit -m "your comment"
git push -u origin main
```

# CPU 
```
su
cat /sys/devices/system/cpu/online
echo 0 > /sys/devices/system/cpu/cpu1/online
cat /sys/devices/system/cpu/online
su - username
```