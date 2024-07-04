# How to run the code in this repository
1. Clone the repository

2. Open the terminal and navigate to the repository folder

3. Run the following command to install the required packages:
```bash
catkin_make
source devel/setup.bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.102
```

4. Run the UR control panel to control the robot:
```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_vel_controller']
stop_controllers: ['scaled_pos_joint_traj_controller']
strictness: 2
start_asap: false
timeout: 0.0"
```

5. Check the list of controllers:
```bash
rosservice call /controller_manager/list_controllers
```

6. Run the joystick control node:
```bash
roslaunch ur5e_joystick_control ur5e_joystick_control.launch
```

7. Generate database:
```bash
roslaunch ur5e_joystick_control generate_database.launch
```

8. Copy data:
```bash
cp -r /home/jinxin/ur_ws/rosbag /media/jinxin/Data_ubuntu
```