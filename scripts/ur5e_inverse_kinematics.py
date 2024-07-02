#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from RobotiqGripper import RobotiqHand
import numpy as np

class UR5eInverseKinematics:
    def __init__(self):
        rospy.init_node('ur5e_inverse_kinematics', anonymous=True)

        # self.gripper = RobotiqHand()
        # self.gripper.connect("192.168.0.121", 54321)
        # self.gripper.reset()
        # self.gripper.activate()
        # self.gripper.wait_activate_complete()
        # self.gripper_position = 0

        self.dh_d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]
        self.dh_a = [0, -0.425, -0.3922, 0, 0, 0]
        self.dh_alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
        self.joint_angles = [0, 0, 0, 0, 0, 0]  # Initial joint angles

        self.joint_angles_received = False  # Flag to check if joint angles are received
        self.subscription_joy = rospy.Subscriber('/joy', Joy, self.joystick_callback)
        self.subscription_joint_states = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self.publisher = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
        

    def joint_states_callback(self, msg):
        joint_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        joint_positions = dict(zip(msg.name, msg.position))
        self.joint_angles = [joint_positions[joint] for joint in joint_order]
        self.joint_angles_received = True

    def joystick_callback(self, msg):
        if not self.joint_angles_received:
            rospy.loginfo('Joint angles not received yet, skipping inverse kinematics calculation.')
            return

        x_velocity = msg.axes[0]
        y_velocity = -msg.axes[1]
        z_velocity = msg.axes[4]
        roll_velocity = msg.axes[3]
        pitch_velocity = msg.axes[6]
        yaw_velocity = msg.axes[7]

        if msg.buttons[4] == 1:
            rospy.loginfo('Opening gripper')
            self.gripper_position += 10
            if self.gripper_position > 255:
                self.gripper_position = 255
            self.gripper.move(self.gripper_position, 0, 100)
        
        if msg.buttons[5] == 1:
            rospy.loginfo('Closing gripper')
            self.gripper_position -= 10
            if self.gripper_position < 0:
                self.gripper_position = 0
            self.gripper.move(self.gripper_position, 0, 100)

        # Base to end-effector velocity transformation
        eef_base_velocity = np.array([0.05 * x_velocity, 0.05 * y_velocity, 0.05 * z_velocity, 0.05 * roll_velocity, 0.05 * pitch_velocity, 0.05 * yaw_velocity])
        rospy.loginfo('End-effector velocity: {}'.format(eef_base_velocity))
        joint_velocities = self.inverse_kinematics(self.joint_angles, eef_base_velocity)
        self.publish_joint_velocities(joint_velocities)

    def compute_jacobian(self, theta):
        J = np.zeros((6, 6))  # Jacobian matrix
        T = np.eye(4)  # Transformation matrix from the base to the current joint

        # Lists to store z axes and origin points for each joint
        z_vectors = []
        p_vectors = []

        # Start with the base z-axis, which is [0, 0, 1] for the initial configuration
        z_base = np.array([0, 0, 1])  # This is z0
        p_base = np.array([0, 0, 0])  # This is p0

        # Initial base z-axis and origin point
        z_vectors.append(z_base)
        p_vectors.append(p_base)

        for i in range(6):
            alpha = self.dh_alpha[i]  # Twist angle
            a = self.dh_a[i]          # Link length
            d = self.dh_d[i]          # Link offset

            # Compute transformation matrix for joint i
            Ti = np.array([
                [np.cos(theta[i]), -np.sin(theta[i]) * np.cos(alpha), np.sin(theta[i]) * np.sin(alpha), a * np.cos(theta[i])],
                [np.sin(theta[i]), np.cos(theta[i]) * np.cos(alpha), -np.cos(theta[i]) * np.sin(alpha), a * np.sin(theta[i])],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]
            ])

            # Update transformation matrix from base to current joint
            T = np.dot(T, Ti)
            
            # Update and store z axis and origin point for the current joint
            z_vectors.append(T[:3, 2])
            p_vectors.append(T[:3, 3])

        p_end = p_vectors[-1]  # Position of the end effector

        for i in range(6):
            # Compute columns of the Jacobian matrix
            J[:3, i] = np.cross(z_vectors[i], p_end - p_vectors[i])  # Linear velocity component
            J[3:, i] = z_vectors[i]  # Angular velocity component

        return J

    def inverse_kinematics(self, theta, end_effector_velocity):
        J = self.compute_jacobian(theta)
        J_pinv = np.linalg.pinv(J)  # Pseudoinverse of Jacobian
        joint_velocities = np.dot(J_pinv, end_effector_velocity)
        return joint_velocities

    def publish_joint_velocities(self, joint_velocities):
        msg = Float64MultiArray()
        msg.data = joint_velocities.tolist()
        self.publisher.publish(msg)
        return

def main():
    try:
        ur5e_inverse_kinematics = UR5eInverseKinematics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
