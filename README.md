# How to run the code in this repository
1. Clone the repository

2. Open the terminal and navigate to the repository folder

3. Run the following command to install the required packages:
```bash
catkin_make
source devel/setup.bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.104
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

9. calibration:
```bash
➜  ur_ws roslaunch ur_calibration calibration_correction.launch robot_ip:=
192.168.1.104 target_filename:="/home/jinxin/ur_ws/ur20calibration.yaml"
... logging to /home/jinxin/.ros/log/f6105e60-3eba-11ef-bb07-218cac334720/roslaunch-jinxin-ur20-7294.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://jinxin-ur20:36083/

SUMMARY
========

PARAMETERS
 * /calibration_correction/output_filename: /home/jinxin/ur_w...
 * /calibration_correction/robot_ip: 192.168.1.104
 * /rosdistro: noetic
 * /rosversion: 1.16.0

NODES
  /
    calibration_correction (ur_calibration/calibration_correction)

auto-starting new master
process[master]: started with pid [7302]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to f6105e60-3eba-11ef-bb07-218cac334720
process[rosout-1]: started with pid [7312]
started core service [/rosout]
process[calibration_correction-2]: started with pid [7315]
[ WARN] [1720615818.873099507]: Failed to read from stream, reconnecting in 1 seconds...
[ INFO] [1720615820.426767821]: checksum: [3042159723 2825233598 3038656413 2837741972 2836111355 2833694156 ]
dh_theta: [1.4910176998964e-07 -1.36660647780966 1.04825235932089 0.318363234964337 4.02378880735854e-08 -3.86843341552103e-07 ]
dh_a: [0.000159694994835582 -0.0862491988383858 -0.372580390862978 1.65704245972927e-05 5.37906639089847e-06 0 ]
dh_d: [0.16257063182108 315.829375096859 -342.810589282497 27.1153529744042 0.0998816266727 0.0995218985320186 ]
dh_alpha: [1.5700052415988 -0.00131879150548935 0.00455096395056942 1.57044157216112 -1.57135106820315 0 ]
calibration_status: 2

[ INFO] [1720615820.527327375]: Writing calibration data to "/home/jinxin/ur_ws/ur20calibration.yaml"
[ INFO] [1720615820.528029814]: Wrote output.
[ INFO] [1720615820.528144463]: Calibration correction done
[calibration_correction-2] process has finished cleanly
log file: /home/jinxin/.ros/log/f6105e60-3eba-11ef-bb07-218cac334720/calibration_correction-2*.log
```