# dag_case using image corner detection

# 1. ROS crystal
--- Test on ROS crystal

# 2. Building
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