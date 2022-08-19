# introduction
## clone repo
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@gitlab.com:ghosts-of-rugby/kirin.git
```

## install requirements
```
sudo xargs apt install -y < requirements/apt.txt
```

# build
```
cd ~/ros2_ws && colcon build
```

# test execution
## test joystick (without ros2)
jstest /dev/input/js0
