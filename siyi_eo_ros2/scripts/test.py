# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# import cv2
# from cv_bridge import CvBridge
# import time

# class UpscaleViewer(Node):
#     def __init__(self):
#         super().__init__('upscale_viewer')
#         self.bridge = CvBridge()
#         self.subscription = self.create_subscription(
#             Image,
#             '/cui/siyi_eo_ros2/image_raw',  # 네 ROS2 토픽 이름에 맞게 수정
#             self.listener_callback,
#             10)
#         self.get_logger().info('✅ Subscribed to /cui/siyi_eo_ros2/image_raw')

#         self.prev_time = time.time()
#         self.fps = 0.0

#     def listener_callback(self, msg):
#         current_time = time.time()
#         dt = current_time - self.prev_time
#         self.prev_time = current_time

#         if dt > 0:
#             self.fps = 1.0 / dt

#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # FPS 텍스트 표시
#         cv2.putText(frame, f'FPS: {self.fps:.2f}', (10, 30),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

#         # VGA → FHD 업스케일 (필요하면)
#         upscaled_frame = cv2.resize(frame, (1920, 1080), interpolation=cv2.INTER_CUBIC)

#         # 화면 출력
#         cv2.imshow('Upscaled to FHD', frame)
#         if cv2.waitKey(1) == 27:
#             rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = UpscaleViewer()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     cv2.destroyAllWindows()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import time

class UpscaleViewer(Node):
    def __init__(self):
        super().__init__('upscale_viewer')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/cui/siyi_eo_ros2/image_raw/compressed',  # 압축 이미지 토픽 이름으로 수정
            self.listener_callback,
            10)
        self.get_logger().info('✅ Subscribed to /cui/siyi_eo_ros2/image_raw/compressed')

        self.prev_time = time.time()
        self.fps = 0.0

    def listener_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt > 0:
            self.fps = 1.0 / dt

        # 압축된 이미지 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # FPS 텍스트 표시
        cv2.putText(frame, f'FPS: {self.fps:.2f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        # VGA → FHD 업스케일 (필요하면)
        upscaled_frame = cv2.resize(frame, (1920, 1080), interpolation=cv2.INTER_CUBIC)

        # 화면 출력
        cv2.imshow('Upscaled to FHD', upscaled_frame)
        if cv2.waitKey(1) == 27:
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = UpscaleViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
