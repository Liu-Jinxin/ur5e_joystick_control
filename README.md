# How to run the code in this repository
1. Clone the repository
2. Open the terminal and navigate to the repository folder
3. Run the following command to install the required packages:
```bash
colcon build
source install/setup.bash
ros2 launch ur5e_joystick_control ur5e_joystick_control.launch.py
```