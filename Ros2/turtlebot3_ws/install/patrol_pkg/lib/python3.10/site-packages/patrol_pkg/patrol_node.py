import math

import rclpy
import requests
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# ──────────────────────────────
# patrol8.py를 "정식 ROS2 노드"로 변환한 버전 (사운드 센서 미사용)
#
# 기존 방식 (patrol8.py):
#   - time.sleep()으로 블로킹, nav2_simple_commander로 결과를 while문에서 폴링
#   - ros2 node list 에 안 뜨고, 다른 노드와 한 executor에서 같이 돌리기 어려움
#
# 이 버전:
#   - rclpy.node.Node를 상속받는 정식 노드
#   - NavigateToPose 액션 클라이언트를 직접 써서 콜백 기반(논블로킹)으로 동작
#   - 타이머 하나로 "이동 -> 대기 -> 다음 이동" 상태를 관리 (상태 기계)
#   - ros2 run 으로 실행 가능한 정식 노드 (패키지화 방법은 안내 참고)
# ──────────────────────────────

PAUSE_SECONDS = 3.0
ROUND_TRIP_PAUSE_SECONDS = 10.0
SERVER_URL = "http://192.168.0.2:5000"

# 상태 정의
STATE_NAVIGATING = 'navigating'
STATE_PAUSING = 'pausing'
STATE_ROUND_TRIP_PAUSING = 'round_trip_pausing'


def make_pose(node, x, y, yaw=0.0):
    """
    x, y: 목표 좌표
    yaw: 도착했을 때 로봇이 바라볼 방향 (라디안)
         0.0   = +x 방향 (정면, 기본)
         1.57  = +y 방향 (왼쪽으로 90도)
         3.14  = -x 방향 (뒤쪽, 180도)
         -1.57 = -y 방향 (오른쪽으로 90도)
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # Nav2 액션 클라이언트 (BasicNavigator 대신 직접 사용 - 콜백 기반)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 웨이포인트 준비 (amcl_pose로 실측한 좌표, 시작 위치 포함 9개 지점)
        self.waypoints = self._build_waypoints()
        self.num_points = len(self.waypoints)  # 예: 16개 (정방향 9 + 역방향 7)
        self.current_i = 0

        # 상태 기계 변수
        self.state = None
        self.pause_timer = None

        self.get_logger().info('patrol_node 시작. Nav2 액션 서버 대기 중...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 액션 서버 연결됨. 순찰 시작.')

        self._notify_patrol_start()
        self._send_next_goal()

    def _build_waypoints(self):
        # 칸막이 물체 인식을 위해 홀수 지점은 오른쪽, 짝수 지점(1번 제외)은 왼쪽을 바라보도록 설정
        p1 = make_pose(self, 0.10165125674164602, -0.05073359956159341, yaw=0.0)      # 시작 위치 - 정면
        p2 = make_pose(self, 0.34164818768996846, 1.1977723311292336, yaw=-1.57)      # 왼쪽
        p3 = make_pose(self, 0.7443546017221914, 1.1606690109190736, yaw=1.57)        # 오른쪽
        p4 = make_pose(self, 0.9720431379366761, 0.33897203992241903, yaw=-1.57)      # 왼쪽
        p5 = make_pose(self, 1.2140891297116543, 1.2011701385908178, yaw=1.57)        # 오른쪽
        p6 = make_pose(self, 1.3507324382874042, 0.38942217704970494, yaw=-1.57)      # 왼쪽
        p7 = make_pose(self, 1.5658655966192034, 1.2440643100650892, yaw=1.57)        # 오른쪽
        p8 = make_pose(self, 1.8126727979489197, 0.14460677061396246, yaw=-1.57)      # 왼쪽
        p9 = make_pose(self, 2.0434873863022585, 1.1071957958898084, yaw=1.57)        # 오른쪽

        forward = [p1, p2, p3, p4, p5, p6, p7, p8, p9]
        # 마지막 지점(9) 찍고 나서 역순으로 8->7->...->2까지만 돌아옴
        # (시작점 1은 다음 바퀴 시작으로 자동 포함되므로 여기서 중복시키지 않음)
        backward = list(reversed(forward[1:-1]))  # [8, 7, ..., 3, 2]
        return forward + backward

    def _notify_patrol_start(self):
        try:
            requests.post(f"{SERVER_URL}/api/patrol/start", timeout=2)
        except Exception as e:
            self.get_logger().warn(f"순찰 시작 알림 실패: {e}")

    def _notify_patrol_end(self):
        try:
            requests.post(f"{SERVER_URL}/api/patrol/end", timeout=2)
        except Exception as e:
            self.get_logger().warn(f"순찰 종료 알림 실패: {e}")

    def _send_next_goal(self):
        """현재 인덱스의 웨이포인트로 이동 명령 전송 (논블로킹)"""
        self.state = STATE_NAVIGATING
        target = self.waypoints[self.current_i % self.num_points]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target

        idx = self.current_i % self.num_points
        self.get_logger().info(f"{idx + 1}번째 웨이포인트로 이동 시작")
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('목표가 거부됨, 재시도')
            self._send_next_goal()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        status = future.result().status
        idx = self.current_i % self.num_points
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"{idx + 1}번째 웨이포인트 도착 (성공)")
        else:
            self.get_logger().warn(f"{idx + 1}번째 웨이포인트 도착 실패 (status={status})")

        # current_index == 0 은 waypoints[0] = 시작점(1번).
        # current_i == 0(맨 처음 출발)은 제외하고, 그 이후에 다시 1번에 도착했다면
        # 정방향+역방향을 다 돌고 시작점으로 돌아온 것 = 한 바퀴(왕복) 완료
        if idx == 0 and self.current_i > 0:
            self.state = STATE_ROUND_TRIP_PAUSING
            self._notify_patrol_end()
            self.get_logger().info(f"한 바퀴(왕복) 완료! 다음 바퀴 시작 전 {ROUND_TRIP_PAUSE_SECONDS}초 대기...")
            self.pause_timer = self.create_timer(ROUND_TRIP_PAUSE_SECONDS, self._on_pause_done)
        else:
            self.state = STATE_PAUSING
            self.get_logger().info(f"{PAUSE_SECONDS}초 정지, 객체 인식 대기 중...")
            self.pause_timer = self.create_timer(PAUSE_SECONDS, self._on_pause_done)

    def _on_pause_done(self):
        # 타이머는 한 번만 쓰고 없앰
        self.pause_timer.cancel()
        self.pause_timer.destroy()

        was_round_trip = (self.state == STATE_ROUND_TRIP_PAUSING)
        self.current_i += 1

        if was_round_trip:
            self.get_logger().info("새 바퀴 시작 -> 서버에 순찰 시작 알림")
            self._notify_patrol_start()

        self._send_next_goal()

    def destroy_node(self):
        # 순찰 도중에 꺼졌어도, 지금까지 진행된 바퀴는 "종료"로 마무리해줌
        self._notify_patrol_end()
        super().destroy_node()


def main():
    rclpy.init()
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
