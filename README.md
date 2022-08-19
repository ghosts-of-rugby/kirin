# introduction
## clone repo & rosdep install
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive git@gitlab.com:ghosts-of-rugby/kirin.git
cd ~/ros2_ws
sudo rosdep install -i --from-path src --rosdistro <distro> -y
```
Please replace <distro> for distro name you are using such as foxy (my environment)

## install apt requirements
```
sudo xargs apt install -y < requirements/apt.txt
```

# build
```
cd ~/ros2_ws && colcon build --symlink-install
```

# test execution
## test joystick (without ros2)
jstest /dev/input/js0
