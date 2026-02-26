import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

class MapRepeater(Node):
    def __init__(self):
        super().__init__('map_repeater')

        # 1. octomap_server가 보내는 토픽 이름 (받을 것)
        self.input_topic = '/octomap_point_cloud_centers'
        # 2. MoveIt/RViz에 쏴줄 새로운 토픽 이름 (보낼 것)
        self.output_topic = '/octomap_continuous'

        # [중요] 받는 쪽 QoS: 'Transient Local'로 설정해야 1번 보낸 데이터를 놓치지 않고 잡음
        qos_in = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        
        # [중요] 보내는 쪽 QoS: 일반적인 설정 (계속 쏠 거니까 Volatile이어도 됨)
        qos_out = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)

        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.callback, qos_in)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos_out)
        
        self.cached_msg = None
        
        # 1초마다 재발행하는 타이머
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(f'Waiting for map on {self.input_topic}...')

    def callback(self, msg):
        # 메시지가 들어오면 캐시에 저장
        if self.cached_msg is None:
            self.get_logger().info('Map received! Starting continuous republishing...')
        self.cached_msg = msg

    def timer_callback(self):
        # 저장된 메시지가 있으면 현재 시간으로 갱신해서 재발행
        if self.cached_msg:
            self.cached_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.cached_msg)

def main():
    rclpy.init()
    node = MapRepeater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()