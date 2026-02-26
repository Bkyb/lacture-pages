#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO

class HipPoseDetectorNode(Node):
    def __init__(self):
        super().__init__('hip_pose_detector_node')
        
        # 모델 설정
        self.declare_parameter('model_name', 'yolov8n-pose.pt')
        self.model = YOLO(self.get_parameter('model_name').value)
        self.conf_threshold = 0.5 

        # ROS2 통신
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.result_pub = self.create_publisher(Image, '/pose/visualized_image', 10)
        self.coord_pub = self.create_publisher(Point, '/pose/human_center_2d', 10)

        # 키포인트 인덱스 정의
        self.L_HIP, self.R_HIP = 11, 12
        self.L_KNEE, self.R_KNEE = 13, 14
        self.L_ANKLE, self.R_ANKLE = 15, 16

        self.get_logger().info("하체 중심점 검출기: 무릎 부재 시 발목 대체 로직 활성화")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return

        results = self.model(cv_image, verbose=False, conf=self.conf_threshold)
        annotated_frame = results[0].plot()

        for result in results:
            if result.keypoints is None or len(result.keypoints.xy) == 0:
                continue
            
            all_xy = result.keypoints.xy.cpu().numpy()
            all_conf = result.keypoints.conf.cpu().numpy()

            for i in range(len(all_xy)):
                xy = all_xy[i]
                conf = all_conf[i]

                # --- 계층적 포인트 수집 로직 ---
                valid_points = []

                # 1. 엉덩이는 기본적으로 추가 (검출 시)
                if conf[self.L_HIP] > self.conf_threshold:
                    valid_points.append(xy[self.L_HIP])
                if conf[self.R_HIP] > self.conf_threshold:
                    valid_points.append(xy[self.R_HIP])

                # 2. 왼쪽 다리: 무릎 우선, 없으면 발목
                if conf[self.L_KNEE] > self.conf_threshold:
                    valid_points.append(xy[self.L_KNEE])
                    l_status = "Knee"
                elif conf[self.L_ANKLE] > self.conf_threshold:
                    valid_points.append(xy[self.L_ANKLE])
                    l_status = "Ankle"
                else:
                    l_status = "None"

                # 3. 오른쪽 다리: 무릎 우선, 없으면 발목
                if conf[self.R_KNEE] > self.conf_threshold:
                    valid_points.append(xy[self.R_KNEE])
                    r_status = "Knee"
                elif conf[self.R_ANKLE] > self.conf_threshold:
                    valid_points.append(xy[self.R_ANKLE])
                    r_status = "Ankle"
                else:
                    r_status = "None"

                # 4. 최소 2개 포인트 이상일 때 중심점 계산
                if len(valid_points) >= 2:
                    final_center = np.mean(valid_points, axis=0)
                    cx, cy = int(final_center[0]), int(final_center[1])

                    # 좌표 전송
                    point_msg = Point()
                    point_msg.x = float(cx)
                    point_msg.y = float(cy)
                    point_msg.z = 0.0
                    self.coord_pub.publish(point_msg)

                    # --- 시각화 ---
                    # 다리 상태에 따른 색상 변경 (Knee: 녹색, Ankle 활용 시: 노란색)
                    use_ankle = "Ankle" in [l_status, r_status]
                    color = (0, 255, 255) if use_ankle else (0, 255, 0)

                    cv2.circle(annotated_frame, (cx, cy), 15, color, -1)
                    cv2.circle(annotated_frame, (cx, cy), 15, (0, 0, 0), 2)
                    
                    info_text = f"C({len(valid_points)}p) L:{l_status} R:{r_status}"
                    cv2.putText(annotated_frame, info_text, (cx - 80, cy - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 결과 이미지 발행
        try:
            res_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            res_msg.header = msg.header
            self.result_pub.publish(res_msg)
        except CvBridgeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = HipPoseDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()