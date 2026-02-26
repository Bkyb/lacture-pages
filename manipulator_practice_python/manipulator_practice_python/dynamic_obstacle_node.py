#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import numpy as np
import cv2

class DynamicObstacleNode(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_node')
        
        # Subscriber: 이미지 상의 픽셀 좌표 수신
        self.sub = self.create_subscription(
            Point,
            '/pose/human_center_2d',
            self.pose_callback,
            10
        )
        
        # Publisher: RViz 시각화를 위한 마커 발행
        self.marker_pub = self.create_publisher(Marker, '/human_obstacle_marker', 10)

        # 1. 호모그래피 행렬 계산 (초기화 시 1회만 수행)
        # 이미지 픽셀 좌표
        self.pts_pixel = np.array([
            [574, 327],
            [668, 696],
            [1164, 626],
            [868, 309]
        ], dtype=np.float32)

        # 매칭되는 물리적 좌표 (ROS 규격에 맞게 mm -> m 단위로 변환)
        self.pts_physical = np.array([
            [1.3, 0.875],
            [-0.2, 0.875],
            [-0.2, -0.125],
            [1.3, -0.125]
        ], dtype=np.float32)

        # 변환 행렬 도출
        self.H = cv2.getPerspectiveTransform(self.pts_pixel, self.pts_physical)
        self.get_logger().info("Homography Matrix initialized successfully.")

    def pose_callback(self, msg):
        u, v = msg.x, msg.y

        # 2. 픽셀 좌표를 물리적 좌표로 투영
        # cv2.perspectiveTransform을 위해 shape를 (1, 1, 2)로 맞춤
        pt_pixel = np.array([[[u, v]]], dtype=np.float32)
        pt_physical = cv2.perspectiveTransform(pt_pixel, self.H)
        
        # 변환된 X, Y 좌표 (미터 단위)
        X, Y = pt_physical[0][0]

        # 3. 터미널 출력 (원점 좌표계 기준)
        self.get_logger().info(f"Pixel: ({u:.1f}, {v:.1f}) -> Physical (base_link): X={X:.3f}m, Y={Y:.3f}m")

        # 4. RViz 마커 생성 및 발행
        self.publish_marker(X, Y)

    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "human_obstacle"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # 위치 설정 (Z 중심점: -0.7m ~ 1.0m의 중간값인 0.15m)
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.15
        
        # 방향 (기본 회전 없음)
        marker.pose.orientation.w = 1.0

        # 크기 설정 (지름 600mm -> 0.6m, 높이 1700mm -> 1.7m)
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 1.7

        # 색상 설정 (빨간색, 반투명)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.6

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()