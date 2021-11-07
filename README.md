# dag_case using image corner detection

Note: Source your ROS crystal packages before using this package
```
source /opt/ros/crystal/setup.bash
```

# 1. Install ROS crystal & other packages
```
sudo apt install ros-crystal-cv-bridge
sudo apt install ros-crystal-image-transport
```

# 2. Building
## Modify file path for your own usage
1. Modify the source image path (line 130 & 131 in timer_publisher.h):
```
string image_file = "/media/eric/ubuntu/DATA/Event/ETH/shapes_translation/Text/shapes_translation/images.txt";
string image_path = "/media/eric/ubuntu/DATA/Event/ETH/shapes_translation/Text/shapes_translation/";
```

2. Modify the test results path (line 24 in harris_node.h):
```
string result_path = "/home/eric/crystal_ws/src/corner_detection/results/";
```

3. Modify the test results path (in shi_tomasi_node.h)

## Build this project
```
cd crystal_ws
colcon build --packages-select corner_detection --symlink-install
```
# 3. Run
```
ros2 run corner_detection corner_detection
```

# Github

```
git branch -a
git checkout crystal
git add .
git commit -m "your comment"
git push origin crystal
```

# CPU 
```
su
cat /sys/devices/system/cpu/online
echo 0 > /sys/devices/system/cpu/cpu1/online
cat /sys/devices/system/cpu/online
su - username
```