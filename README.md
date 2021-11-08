# Dag_case Using Image Corner Detection

**Note: Remember to source your ROS eloquent packages before using this package**
```
source /opt/ros/eloquent/setup.bash
```

# Step 1. Install ROS eloquent & other dependent packages
```
sudo apt install ros-eloquent-cv-bridge
sudo apt install ros-eloquent-image-transport
```

# Step 2. Building
## Initialize your own workspace
```
cd
mkdir -p ~/eloquent_ws/src
cd ~/eloquent_ws/src
git clone https://github.com/ruoxianglee/corner_detection.git
```

## Modify file path for your own usage
1. Modify the source image path (line 130 & 131 in timer_publisher.h):
```
string image_file = "/media/eric/ubuntu/DATA/Event/ETH/shapes_translation/Text/shapes_translation/images.txt";
string image_path = "/media/eric/ubuntu/DATA/Event/ETH/shapes_translation/Text/shapes_translation/";
```

2. Modify the test results path (line 24 in harris_node.h):
```
string result_path = "/home/eric/eloquent_ws/src/corner_detection/results/";
```

3. Modify the test results path (in shi_tomasi_node.h)

## Build this project
```
source /opt/ros/eloquent/setup.bash
cd eloquent_ws
colcon build --packages-select corner_detection --symlink-install
```
# Step 3. Run
**Note: Remeber to set your CPU environment before running.**
```
source ~/eloquent_ws/install/setup.bash
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
