# Create Package

```
source /opt/ros/humble/setup.bash
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

# Import package 

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ros/ros_tutorials.git -b humble
# cd if you're still in the ``src`` directory with the ``ros_tutorials`` clone
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```

# Build Package

```
colcon build --symlink-install
source ~/Code/ros_playground/src/planning/install/setup.sh
ros2 run planning node
```

# Source Config and launch
```
source ~/Code/panda_arm_config/install/setup.sh
source ~/Code/ros_playground/src/planning/install/setup.sh
ros2 launch planning.launch.py
```

# Launch Moveit Setup Assistant
```
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
######################
Isaac Sim
######################

# python script
~/.local/share/ov/pkg/isaac-sim-4.0.0/python.sh

# examples
~/.local/share/ov/pkg/isaac-sim-4.0.0/exts/omni.isaac.examples/omni/isaac/examples

# standalone examples
~/.local/share/ov/pkg/isaac-sim-4.0.0/standalone_examples/api/omni.isaac.core