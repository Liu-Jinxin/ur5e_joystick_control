#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class UR5eInverseKinematics(Node):
    def __init__(self):
        super().__init__('ur5e_inverse_kinematics')
        self.subscription_joy = self.create_subscription(Joy, '/joy', self.joystick_callback, 10)
        self.subscription_joint_states = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 1)

        self.dh_d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]
        self.dh_a = [0, -0.425, -0.3922, 0, 0, 0]
        self.dh_alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
        self.joint_angles = [0, 0, 0, 0, 0, 0]  # Initial joint angles
        self.joint_angles_received = False  # Flag to check if joint angles are received

    def joint_states_callback(self, msg):
        joint_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        joint_positions = dict(zip(msg.name, msg.position))
        self.joint_angles = [joint_positions[joint] for joint in joint_order]
        self.joint_angles_received = True  # Set the flag to True once joint angles are received

    def joystick_callback(self, msg):
        if not self.joint_angles_received:
            self.get_logger().info('Joint angles not received yet, skipping inverse kinematics calculation.')
            return
        
        x_velocity = msg.axes[0]        # X velocity
        y_velocity = msg.axes[1]        # Y velocity
        z_velocity = msg.axes[2]        # Z velocity
        roll_velocity = msg.axes[3]     # Roll velocity
        pitch_velocity = 0.0            # Pitch velocity
        yaw_velocity = 0.0              # Yaw velocity

        if msg.buttons[4] == 1:
            pitch_velocity = msg.axes[0]
            yaw_velocity = msg.axes[1]
            x_velocity = 0
            y_velocity = 0
        elif msg.buttons[5] == 1:
            pitch_velocity = msg.axes[2]
            yaw_velocity = msg.axes[3]
            z_velocity = 0
            roll_velocity = 0

        # Adding logging to debug the input velocities
        self.get_logger().info(f'Velocities - X: {x_velocity}, Y: {y_velocity}, Z: {z_velocity}, Roll: {roll_velocity}, Pitch: {pitch_velocity}, Yaw: {yaw_velocity}')

        end_effector_velocity = np.array([0.1 * x_velocity, 0.1 * y_velocity, 0.1 * z_velocity, roll_velocity, pitch_velocity, yaw_velocity])
        joint_velocities = self.inverse_kinematics(self.joint_angles, end_effector_velocity)
        self.publish_joint_velocities(joint_velocities)

    def compute_jacobian(self, theta):
        jacobian = np.zeros((6, 6))
        t = np.eye(4)
        z = np.array([0, 0, 1])
        p = np.zeros(3)

        for i in range(6):
            ti = np.array([
                [np.cos(theta[i]), -np.sin(theta[i]) * np.cos(self.dh_alpha[i]), np.sin(theta[i]) * np.sin(self.dh_alpha[i]), self.dh_a[i] * np.cos(theta[i])],
                [np.sin(theta[i]), np.cos(theta[i]) * np.cos(self.dh_alpha[i]), -np.cos(theta[i]) * np.sin(self.dh_alpha[i]), self.dh_a[i] * np.sin(theta[i])],
                [0, np.sin(self.dh_alpha[i]), np.cos(self.dh_alpha[i]), self.dh_d[i]],
                [0, 0, 0, 1]
            ])
            t = np.dot(t, ti)
            r = t[:3, :3]
            new_p = t[:3, 3]
            jacobian[:3, i] = np.cross(z, (new_p - p))
            jacobian[3:, i] = z
            z = r @ np.array([0, 0, 1])
            p = new_p

        return jacobian

    def inverse_kinematics(self, theta, end_effector_velocity):
        jacobian = self.compute_jacobian(theta)
        jacobian_inv = np.linalg.pinv(jacobian)
        joint_velocities = np.dot(jacobian_inv, end_effector_velocity)
        return joint_velocities

    def publish_joint_velocities(self, joint_velocities):
        msg = Float64MultiArray()
        msg.data = joint_velocities.tolist()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ur5e_inverse_kinematics = UR5eInverseKinematics()
    rclpy.spin(ur5e_inverse_kinematics)
    ur5e_inverse_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
