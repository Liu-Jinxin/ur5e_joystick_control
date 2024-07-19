import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import tf_transformations

class TF2Listener(Node):
    def __init__(self):
        super().__init__('tf2_listener')

        # 初始化 tf2 buffer 和 listener
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

        self.source_frame = 'camera_color_optical_frame'
        self.target_frame = 'wrist_3_link'

        # 定时器，每秒查询一次变换
        self.timer = self.create_timer(1.0, self.handle_timer)

    def handle_timer(self):
        try:
            # 现在的时间
            now = rclpy.time.Time()
            # 使用 tf2 来查询最新的变换关系
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                now)
            # 转换为 4x4 矩阵并打印
            translation = [trans.transform.translation.x, 
                           trans.transform.translation.y, 
                           trans.transform.translation.z]
            rotation = [trans.transform.rotation.x, 
                        trans.transform.rotation.y, 
                        trans.transform.rotation.z, 
                        trans.transform.rotation.w]
            matrix = tf_transformations.compose_matrix(translate=translation, angles=tf_transformations.euler_from_quaternion(rotation))
            print("4x4 Transformation Matrix:")
            print(matrix)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info(str(e))

def main():
    rclpy.init()
    tf2_listener = TF2Listener()
    rclpy.spin(tf2_listener)
    tf2_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
