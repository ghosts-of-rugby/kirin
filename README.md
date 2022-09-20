# introduction
## clone repo & rosdep install
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive git@github.com:ghosts-of-rugby/kirin.git
cd ~/ros2_ws
sudo rosdep install -i --from-path src --rosdistro <distro> -y
```
Please replace <distro> for distro name you are using such as foxy (my environment)

## install apt requirements
```
sudo xargs apt install -y < requirements/apt.txt
```
# bashrc
add source command in bashrc
``
echo "~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "/opt/ros/foxy/setup.bash" >> ~/.bashrc
``

# build
```
cd ~/ros2_ws && colcon build --symlink-install
```


# test execution
## test joystick (without ros2)
```shell
jstest /dev/input/js0
```


## test motor input
in terminal1
```shell
ros2 run kirin test_motor_controller --ros-args --params-file (ros2 pkg prefix --share kirin)/launch/params.yaml
```

in terminal2
```shell
ros2 topic pub -r 50 motor/reference kirin_msgs/msg/MotorStateVector "{angle: {theta: 0.0, left: 7.0, right: -7.0, z: 0.0}, velocity: {theta: 0.0, left:0.0, right: 0.0, z: 0.0} }"
```

## test hand tool manager
in terminal 1
```shell
ros2 run kirin test_hand_tool_manager
```

in terminal 2
```shell
ros2 service call /tool/set_air_state kirin_msgs/srv/SetAirState "{ air_state: {left: 0, right: 0, top: 0, ex_left: 0, ex_right: 0, release: 0} }"
```
