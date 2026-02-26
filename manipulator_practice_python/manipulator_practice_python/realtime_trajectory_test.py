import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# 주의: dsr_msgs2 패키지 및 메시지 이름은 실제 환경에 맞게 확인해주세요.
from dsr_msgs2.msg import ServojRtStream 

class ReverseTrajectoryNode(Node):
    def __init__(self):
        super().__init__('reverse_trajectory_node')
        
        # 1. Publisher & Subscriber 설정
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.pub = self.create_publisher(ServojRtStream, '/servoj_rt_stream', 10)
        
        # 2. 30Hz 타이머 설정 (1초 / 30 = 약 0.0333초)
        self.timer_period = 1.0 / 30.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 3. 상태 관리 및 제어 변수
        self.state = 'RECORDING'  # 상태: 'RECORDING' -> 'PUBLISHING' -> 'DONE'
        self.recorded_positions = []
        self.record_limit = 100   # 기록할 프레임 수 (약 3.3초 분량)
        self.publish_index = 0    # 퍼블리시할 인덱스 추적
        
        self.get_logger().info("조인트 궤적 기록을 시작합니다... (상태: RECORDING)")

    def joint_state_callback(self, msg):
        # 기록 상태가 아니면 무시
        if self.state != 'RECORDING':
            return

        try:
            # joint_states의 순서가 1,2,4,5,3,6 처럼 섞여 있으므로 1~6 순서로 명시적 정렬
            joint_indices = {name: i for i, name in enumerate(msg.name)}
            ordered_pos = [
                msg.position[joint_indices['joint_1']],
                msg.position[joint_indices['joint_2']],
                msg.position[joint_indices['joint_3']],
                msg.position[joint_indices['joint_4']],
                msg.position[joint_indices['joint_5']],
                msg.position[joint_indices['joint_6']]
            ]
            self.recorded_positions.append(ordered_pos)
            
        except KeyError as e:
            self.get_logger().warn(f"조인트 이름을 찾을 수 없습니다: {e}")
            return

        # 설정한 한계치까지 기록했다면 상태를 PUBLISHING으로 전환
        if len(self.recorded_positions) >= self.record_limit:
            self.state = 'PUBLISHING'
            # 궤적을 역순으로 미리 뒤집어 놓음
            self.recorded_positions.reverse()
            self.get_logger().info("기록 완료. 역순 퍼블리시를 시작합니다. (상태: PUBLISHING)")

    def timer_callback(self):
        # 타이머는 30Hz 주기로 계속 돌지만, PUBLISHING 상태일 때만 메시지를 쏩니다.
        if self.state == 'PUBLISHING':
            if self.publish_index < len(self.recorded_positions):
                pos = self.recorded_positions[self.publish_index]
                
                stream_msg = ServojRtStream()
                stream_msg.pos = pos
                # 속도와 가속도는 필요에 따라 계산하여 넣을 수 있으나, 우선 0으로 둡니다.
                stream_msg.vel = [0.0] * 6
                stream_msg.acc = [0.0] * 6
                stream_msg.time = self.timer_period
                
                self.pub.publish(stream_msg)
                self.publish_index += 1
                self.get_logger().info("쏠수있어")
            else:
                # 모든 기록을 쏘고 나면 상태 변경 및 타이머 종료
                self.state = 'DONE'
                self.get_logger().info("역순 궤적 퍼블리시가 완료되었습니다. (상태: DONE)")
                self.timer.cancel()  # 더 이상 불필요한 타이머 콜백 호출 방지

def main(args=None):
    rclpy.init(args=args)
    node = ReverseTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()