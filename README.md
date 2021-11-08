# Dag_case Using Image Corner Detection

**Note: Remember to source your ROS crystal packages before using this package**
```
source /opt/ros/crystal/setup.bash
```

# Step 1. Install ROS crystal & other dependent packages
```
sudo apt install ros-crystal-cv-bridge
sudo apt install ros-crystal-image-transport
```

# Step 2. Building
## Initialize your own workspace
```
cd
mkdir -p ~/crystal_ws/src
cd ~/crystal_ws/src
git clone https://github.com/ruoxianglee/corner_detection.git -b crystal
```

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
source /opt/ros/crystal/setup.bash
cd crystal_ws
colcon build --packages-select corner_detection --symlink-install
```
# Step 3. Run
**Note: Remeber to set your CPU environment before running.**
```
source ~/crystal_ws/install/setup.bash
ros2 run corner_detection corner_detection
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
