#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import math
from message_filters import ApproximateTimeSynchronizer, Subscriber


def dh_matrix(theta, d, a, alpha):
    """Create the Denavit-Hartenberg transformation matrix."""
    return np.array(
        [
            [
                math.cos(theta),
                -math.sin(theta) * math.cos(alpha),
                math.sin(theta) * math.sin(alpha),
                a * math.cos(theta),
            ],
            [
                math.sin(theta),
                math.cos(theta) * math.cos(alpha),
                -math.cos(theta) * math.sin(alpha),
                a * math.sin(theta),
            ],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1],
        ]
    )


def forward_kinematics(joint_angles):
    """Compute the forward kinematics for all joints to get the transformation matrix from base to wrist."""
    dh_params = np.array(
        [
            [joint_angles[0], 0.1625, 0, math.pi / 2],
            [joint_angles[1], 0, -0.425, 0],
            [joint_angles[2], 0, -0.3922, 0],
            [joint_angles[3], 0.1333, 0, math.pi / 2],
            [joint_angles[4], 0.0997, 0, -math.pi / 2],
            [joint_angles[5], 0.0996, 0, 0],
        ]
    )
    T = np.eye(4)
    for params in dh_params:
        T = np.dot(T, dh_matrix(*params))
    return T


def static_transform_wrist_to_camera():
    """Provide the static transformation matrix from the wrist link to the camera frame."""
    return np.array(
        [
            [-0.500, -0.500, 0.707, 0.047],
            [0.866, -0.288, 0.408, 0.040],
            [0.000, 0.817, 0.577, 0.049],
            [0.000, 0.000, 0.000, 1.000],
        ]
    )


class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)
        
        self.image_topic = rospy.get_param('~image_topic', 'camera/color/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', 'camera/aligned_depth_to_color/image_raw')
        self.confi_topic = rospy.get_param('~confi_topic', 'camera/confidence/image_rect_raw')
        self.joint_topic = rospy.get_param('~joint_topic', 'joint_states')
        self.result_path = rospy.get_param('~result_path', '/workspace/Ros1_bag/L515_2024-07-03-21-09-35_results')
        self.traj_file_path = os.path.join(os.path.dirname(self.result_path), 'traj.txt')
        os.makedirs(self.result_path, exist_ok=True)
        self.traj_file = open(self.traj_file_path, "a")

        self.bridge = CvBridge()
        self.frame_number = 0
        self.desired_order = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        color_sub = Subscriber(self.image_topic, Image)
        depth_sub = Subscriber(self.depth_topic, Image)
        confi_sub = Subscriber(self.confi_topic, Image)
        joint_sub = Subscriber(self.joint_topic, JointState)
        self.ts = ApproximateTimeSynchronizer([color_sub, depth_sub, confi_sub, joint_sub], 30, 0.002)
        self.ts.registerCallback(self.callback)
        

    def __del__(self):
        if self.traj_file:
            self.traj_file.close()

    def callback(self, color_msg, depth_msg, confi_msg, joint_msg):
        rospy.loginfo("Received images and joint states")
        joint_angles = self.get_joint_angles(joint_msg)
        base_to_wrist_transform = forward_kinematics(joint_angles)
        wrist_to_camera_transform = static_transform_wrist_to_camera()
        base_to_camera_transform = np.dot(base_to_wrist_transform, wrist_to_camera_transform)
        flat_transform = base_to_camera_transform.flatten()
        rospy.loginfo('Writing this matrix to traj.txt: {}'.format(flat_transform))
        np.savetxt(self.traj_file, [flat_transform], fmt='%.18e')
        cv_color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        frame_filename = os.path.join(self.result_path, 'frame{:06d}.jpg'.format(self.frame_number))
        cv2.imwrite(frame_filename, cv_color_image)
        rospy.loginfo("Saved color image to {}".format(frame_filename))
        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth_filename = os.path.join(self.result_path, 'depth{:06d}.png'.format(self.frame_number))
        cv2.imwrite(depth_filename, cv_depth_image)
        rospy.loginfo("Saved depth image to {}".format(depth_filename))
        cv_confi_image = self.bridge.imgmsg_to_cv2(confi_msg, desired_encoding="passthrough")
        confi_filename = os.path.join(self.result_path, 'confi{:06d}.png'.format(self.frame_number))
        cv2.imwrite(confi_filename, cv_confi_image)
        rospy.loginfo("Saved confi image to {}".format(confi_filename))
        self.frame_number += 1

    def get_joint_angles(self, joint_msg):
        index_map = {name: idx for idx, name in enumerate(joint_msg.name)}
        joint_angles = [joint_msg.position[index_map[joint]] for joint in self.desired_order]
        return joint_angles

def main():
    try:
        image_saver = ImageSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()