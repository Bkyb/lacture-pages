import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import math
import numpy as np

# 사용자 정의 서비스 임포트 (패키지명 확인 필요)
# safety_guard_core/srv/MoveToPose.srv 파일이 있다고 가정
from manipulator_practice_cpp.srv import MoveToPose

class RobotActionGateway(Node):
    def __init__(self):
        super().__init__('robot_action_gateway')

        # 1. 설정 (파라미터로 관리하면 더 좋습니다)
        self.declare_parameter('group_name', 'manipulator')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_link', 'link_6')
        self.declare_parameter('keep_orientation', False) # 기본값: 각도 유지 안 함

        self.group_name = self.get_parameter('group_name').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_link = self.get_parameter('ee_link').value

        # 2. 콜백 그룹 설정 (중첩된 비동기 호출을 위해 필수)
        self.callback_group = ReentrantCallbackGroup()

        # 3. 서비스 서버 생성 (요청을 받는 곳)
        self.srv = self.create_service(
            MoveToPose,
            'move_to_pose',
            self.handle_move_request,
            callback_group=self.callback_group
        )

        # 4. 액션 클라이언트 생성 (MoveIt으로 명령을 보내는 곳)
        self._action_client = ActionClient(
            self, 
            MoveGroup, 
            '/move_action', 
            callback_group=self.callback_group
        )

        self.get_logger().info('Robot Action Gateway Server Started. Waiting for requests...')

    def rotation_matrix_to_quaternion(self, R):
        """3x3 회전 행렬 -> 쿼터니언 변환"""
        R = np.array(R).reshape(3, 3)
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw, qx, qy, qz = 0.25 * S, (R[2,1]-R[1,2])/S, (R[0,2]-R[2,0])/S, (R[1,0]-R[0,1])/S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            qw, qx, qy, qz = (R[2,1]-R[1,2])/S, 0.25 * S, (R[0,1]+R[1,0])/S, (R[0,2]+R[2,0])/S
        elif R[1,1] > R[2,2]:
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            qw, qx, qy, qz = (R[0,2]-R[2,0])/S, (R[0,1]+R[1,0])/S, 0.25 * S, (R[1,2]+R[2,1])/S
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            qw, qx, qy, qz = (R[1,0]-R[0,1])/S, (R[0,2]+R[2,0])/S, (R[1,2]+R[2,1])/S, 0.25 * S
        return [float(qx), float(qy), float(qz), float(qw)]

    async def handle_move_request(self, request, response):
        """서비스 요청이 들어오면 실행되는 함수"""
        self.get_logger().info('서비스 요청 수신!')

        # 액션 서버 연결 확인
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = "MoveIt 액션 서버를 찾을 수 없습니다."
            self.get_logger().error(response.message)
            return response

        try:
            # 1. 입력 데이터 파싱 (기존 로직 유지)
            # request.pose_matrix는 float[16] 이라고 가정
            mat = request.pose_matrix 
            
            # 회전 행렬 추출 (Row-major: 0,1,2 / 4,5,6 / 8,9,10)
            rotation_flat = [
                mat[0], mat[1], mat[2],
                mat[4], mat[5], mat[6],
                mat[8], mat[9], mat[10]
            ]
            
            # 위치 추출 (Row-major: 3, 7, 11) 
            tx = float(mat[3])
            ty = float(mat[7])
            tz = float(mat[11])



            q = self.rotation_matrix_to_quaternion(rotation_flat)
            
            # 파라미터 값 실시간 확인 (동적 변경 가능)
            keep_orientation = self.get_parameter('keep_orientation').value
            
            self.get_logger().info(f'목표: [{tx:.3f}, {ty:.3f}, {tz:.3f}], 각도유지: {keep_orientation}')

            # 2. Action Goal 생성
            goal_msg = MoveGroup.Goal()
            goal_msg.request.workspace_parameters.header.frame_id = self.base_frame
            goal_msg.request.workspace_parameters.min_corner.x = -2.0
            goal_msg.request.workspace_parameters.min_corner.y = -2.0
            goal_msg.request.workspace_parameters.min_corner.z = -2.0
            goal_msg.request.workspace_parameters.max_corner.x = 2.0
            goal_msg.request.workspace_parameters.max_corner.y = 2.0
            goal_msg.request.workspace_parameters.max_corner.z = 2.0
            goal_msg.request.start_state.is_diff = True
            goal_msg.request.group_name = self.group_name
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = 5.0
            goal_msg.request.max_velocity_scaling_factor = 0.1
            goal_msg.request.max_acceleration_scaling_factor = 0.1

            # [Goal Constraints] 최종 목표점
            goal_con = Constraints()
            goal_con.name = "goal_pose"
            
            pos_con = PositionConstraint()
            pos_con.header.frame_id = self.base_frame
            pos_con.link_name = self.ee_link
            pos_con.weight = 1.0
            target_box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.001, 0.001, 0.001])
            pos_con.constraint_region.primitives.append(target_box)
            box_pose = Pose()
            box_pose.position.x, box_pose.position.y, box_pose.position.z = tx, ty, tz
            pos_con.constraint_region.primitive_poses.append(box_pose)
            goal_con.position_constraints.append(pos_con)

            ori_con = OrientationConstraint()
            ori_con.header.frame_id = self.base_frame
            ori_con.link_name = self.ee_link
            ori_con.orientation.x, ori_con.orientation.y, ori_con.orientation.z, ori_con.orientation.w = q[0], q[1], q[2], q[3]
            ori_con.absolute_x_axis_tolerance = 0.01
            ori_con.absolute_y_axis_tolerance = 0.01
            ori_con.absolute_z_axis_tolerance = 0.01
            ori_con.weight = 1.0
            goal_con.orientation_constraints.append(ori_con)
            
            goal_msg.request.goal_constraints.append(goal_con)

            # [Path Constraints] 이동 중 자세 유지 (옵션)
            if keep_orientation:
                path_con = Constraints()
                path_con.name = "keep_orientation"
                path_ori = OrientationConstraint()
                path_ori.header.frame_id = self.base_frame
                path_ori.link_name = self.ee_link
                path_ori.orientation = ori_con.orientation # 목표 자세를 유지하며 이동
                path_ori.absolute_x_axis_tolerance = 0.1
                path_ori.absolute_y_axis_tolerance = 0.1
                path_ori.absolute_z_axis_tolerance = 3.14 
                path_ori.weight = 1.0
                path_con.orientation_constraints.append(path_ori)
                goal_msg.request.path_constraints = path_con

            # 3. 액션 전송 및 대기 (await 사용)
            self.get_logger().info('MoveIt으로 액션 전송 중...')
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            goal_handle = await send_goal_future

            if not goal_handle.accepted:
                response.success = False
                response.message = "MoveIt이 경로 계획 요청을 거절했습니다."
                self.get_logger().error(response.message)
                return response

            self.get_logger().info('이동 시작! 완료 대기 중...')
            get_result_future = goal_handle.get_result_async()
            result_wrapper = await get_result_future
            action_result = result_wrapper.result

            # 4. 결과 처리
            if action_result.error_code.val == 1: # SUCCESS
                response.success = True
                response.message = "이동 성공 (MoveIt Success)"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f"이동 실패 ErrorCode: {action_result.error_code.val}"
                self.get_logger().error(response.message)

        except Exception as e:
            response.success = False
            response.message = f"서버 내부 에러: {str(e)}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotActionGateway()

    # 비동기 처리를 위해 MultiThreadedExecutor 사용 필수
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Server stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()