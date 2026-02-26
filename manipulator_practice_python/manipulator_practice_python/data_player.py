#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.qos 

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import numpy as np
import glob

class DataPlayerNode(Node):
    def __init__(self):
        super().__init__('data_player_node')
        
        # 1. ROS 파라미터로 데이터 루트 디렉토리 경로 받기
        self.declare_parameter('directory_path', '')
        self.directory_path = self.get_parameter('directory_path').get_parameter_value().string_value
        
        if not self.directory_path or not os.path.isdir(self.directory_path):
            self.get_logger().fatal(f"유효하지 않은 디렉토리 경로: {self.directory_path}")
            self.is_finished = True 
            return

        self.get_logger().info(f"데이터 재생 디렉토리: {self.directory_path}")

        # 2. 퍼블리셔 설정 (로봇 포즈 관련 제거)
        self.color_pub = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/camera/aligned_depth_to_color/image_raw', 10)
        
        latching_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.cam_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/camera/color/camera_info', 
            qos_profile=latching_qos 
        )
        
        self.bridge = CvBridge()

        # 3. 데이터 버퍼
        self.color_frames = []
        self.depth_frames = []

        # 4. 재생 상태 변수
        self.current_index = 0
        self.total_frames = 0
        self.loop_count = 0
        self.target_loops = 50  # 반복 횟수
        self.is_finished = False 
        self.timer = None
        
        self.height = 0
        self.width = 0

        # 5. 데이터 로드 및 타이머 시작
        try:
            self.load_data_sequences() 
            
            if self.total_frames > 0:
                self.get_logger().info(f"총 {self.total_frames}개의 시퀀스 로드 완료. 30Hz 재생 시작.")
                self.publish_camera_info()
                self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
            else:
                self.get_logger().error("로드할 이미지 데이터가 없습니다.")
                self.is_finished = True
        except Exception as e:
            self.get_logger().fatal(f"데이터 로드 중 오류 발생: {e}")
            self.is_finished = True

    def load_data_sequences(self):
        """color/ 및 depth/ 폴더에서 PNG 파일들을 동적으로 로드"""
        
        # 경로 설정 (사용자 구조: directory_path/color/*.png, directory_path/depth/*.png)
        color_dir = os.path.join(self.directory_path, "color")
        depth_dir = os.path.join(self.directory_path, "depth")

        # 파일 목록 획득 및 정렬 (000000.png, 000001.png 순서 보장)
        color_files = sorted(glob.glob(os.path.join(color_dir, "*.png")))
        depth_files = sorted(glob.glob(os.path.join(depth_dir, "*.png")))

        if not color_files or not depth_files:
            raise FileNotFoundError(f"이미지 파일을 찾을 수 없습니다. 경로 확인: {color_dir}, {depth_dir}")

        # 1. 컬러 이미지 로드
        self.get_logger().info("컬러 이미지 로드 중...")
        for c_file in color_files:
            img = cv2.imread(c_file)
            if img is not None:
                if self.height == 0:
                    self.height, self.width = img.shape[:2]
                self.color_frames.append(img)
        
        # 2. 깊이 이미지 로드 (16비트 원본 유지)
        self.get_logger().info("깊이 이미지 로드 중...")
        for d_file in depth_files:
            # IMREAD_UNCHANGED 필수 (16bit raw 데이터)
            img = cv2.imread(d_file, cv2.IMREAD_UNCHANGED)
            if img is not None:
                self.depth_frames.append(img)

        # 3. 개수 정합성 확인
        c_len = len(self.color_frames)
        d_len = len(self.depth_frames)
        
        if c_len != d_len:
            self.get_logger().warn(f"프레임 개수 불일치! Color: {c_len}, Depth: {d_len}")
            self.total_frames = min(c_len, d_len)
        else:
            self.total_frames = c_len

    def publish_camera_info(self):
        """CameraInfo 발행 (고정 파라미터 사용)"""
        msg = CameraInfo()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "camera_color_optical_frame"
        
        msg.height = self.height
        msg.width = self.width
        
        # RealSense D435 등 일반적인 기본값 (필요시 수정)
        fx = 910.72; fy = 910.86; cx = 634.24; cy = 355.96
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
                 
        self.cam_info_pub.publish(msg)

    def timer_callback(self):
        if self.is_finished:
            return
            
        color_frame = self.color_frames[self.current_index]
        depth_frame = self.depth_frames[self.current_index]
        
        now = self.get_clock().now().to_msg()
        
        try:
            # Color: BGR8
            color_msg = self.bridge.cv2_to_imgmsg(color_frame, "bgr8")
            color_msg.header.stamp = now
            color_msg.header.frame_id = "camera_color_optical_frame" 
            
            # Depth: 16UC1 (16비트 단일 채널)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, "16UC1")
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = "camera_depth_optical_frame" 
            
            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge 오류: {e}")
        
        self.current_index += 1
        
        if self.current_index >= self.total_frames:
            self.current_index = 0
            self.loop_count += 1
            self.get_logger().info(f"▶️ 루프 완료 ({self.loop_count} / {self.target_loops})")
            
            if self.loop_count >= self.target_loops:
                self.is_finished = True 
                self.get_logger().info("재생 종료.")
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DataPlayerNode()
    if not node.is_finished:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()