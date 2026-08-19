import math
import time

import rclpy
import requests
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

# ──────────────────────────────
# ROS2 Nav2 웨이포인트 순찰 (PC에서 실행)
# 카메라 스트리밍은 pi02에서 camera_stream.py로 별도 실행
#
# - 각 지점 도착 후 PAUSE_SECONDS만큼 정지 (YOLO 등 객체 인식 시간 확보)
# - 각 지점마다 도착 시 바라볼 방향(yaw, 라디안)을 지정 가능
# - 1->...->9->...->2->1 로 다시 시작점(1번)에 돌아오면 그게 "한 바퀴(왕복)" 완료
#   그 시점에 ROUND_TRIP_PAUSE_SECONDS만큼 추가로 대기한 뒤 다음 바퀴 시작
# - [추가] 한 바퀴가 시작/끝날 때마다 studycam 서버에 알려줌
#   (서버는 이걸로 순찰 횟수/로그를 기록함. 실패해도 순찰 자체엔 지장 없음)
# ──────────────────────────────

# 지점 도착 후 대기 시간(초) - YOLO 인식 시간 확보용
PAUSE_SECONDS = 3.0

# 한 바퀴(왕복) 완료 후(=다시 1번에 도착했을 때), 다음 바퀴 시작 전 추가로 대기하는 시간(초)
ROUND_TRIP_PAUSE_SECONDS = 10.0

# studycam 서버 주소 - 실제 PC(WSL) IP로 교체 필요
SERVER_URL = "http://192.168.0.2:5000"


def notify_patrol_start():
    try:
        requests.post(f"{SERVER_URL}/api/patrol/start", timeout=2)
    except Exception as e:
        print(f"[알림 실패] 순찰 시작 알림 전송 안 됨: {e}")


def notify_patrol_end():
    try:
        requests.post(f"{SERVER_URL}/api/patrol/end", timeout=2)
    except Exception as e:
        print(f"[알림 실패] 순찰 종료 알림 전송 안 됨: {e}")


def make_pose(nav, x, y, yaw=0.0):
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
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    # yaw(라디안)를 쿼터니언으로 변환 (2D 평면 회전이라 z, w만 계산하면 됨)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()  # amcl, 컨트롤러 등 준비될 때까지 대기

    # amcl_pose로 실측한 좌표 (시작 위치 포함 9개 지점)
    # 칸막이 물체 인식을 위해 홀수 지점은 오른쪽, 짝수 지점(1번 제외)은 왼쪽을 바라보도록 설정
    point_1 = make_pose(nav, 0.10165125674164602, -0.05073359956159341, yaw=0.0)      # 시작 위치 - 정면
    point_2 = make_pose(nav, 0.34164818768996846, 1.1977723311292336, yaw=-1.57)      # 왼쪽
    point_3 = make_pose(nav, 0.7443546017221914, 1.1606690109190736, yaw=1.57)        # 오른쪽
    point_4 = make_pose(nav, 0.9720431379366761, 0.33897203992241903, yaw=-1.57)      # 왼쪽
    point_5 = make_pose(nav, 1.2140891297116543, 1.2011701385908178, yaw=1.57)        # 오른쪽
    point_6 = make_pose(nav, 1.3507324382874042, 0.38942217704970494, yaw=-1.57)      # 왼쪽
    point_7 = make_pose(nav, 1.5658655966192034, 1.2440643100650892, yaw=1.57)        # 오른쪽
    point_8 = make_pose(nav, 1.8126727979489197, 0.14460677061396246, yaw=-1.57)      # 왼쪽
    point_9 = make_pose(nav, 2.0434873863022585, 1.1071957958898084, yaw=1.57)        # 오른쪽

    forward = [point_1, point_2, point_3, point_4, point_5,
               point_6, point_7, point_8, point_9]
    # 마지막 지점(9) 찍고 나서 역순으로 8->7->...->2까지만 돌아옴
    # (시작점 1은 다음 바퀴 시작으로 자동 포함되므로 여기서 중복시키지 않음)
    backward = list(reversed(forward[1:-1]))  # [8, 7, ..., 3, 2]
    waypoints = forward + backward
    num_points = len(waypoints)  # 예: 16개 (1~9 정방향 9개 + 8~2 역방향 7개)

    i = 0
    print("첫 바퀴 시작 -> 서버에 순찰 시작 알림")
    notify_patrol_start()

    try:
        while rclpy.ok():
            current_index = i % num_points
            target = waypoints[current_index]
            nav.goToPose(target)
            while not nav.isTaskComplete():
                feedback = nav.getFeedback()
                # 나중에 여기서 카메라(객체 인식 결과) 체크해서
                # 객체 인식되면 nav.cancelTask() 하고 정지시키면 됨
                pass
            result = nav.getResult()
            print(f"{current_index + 1}번째 웨이포인트 도착, 결과: {result}")

            # 도착 후 잠깐 정지 (YOLO 등 객체 인식 시간 확보)
            print(f"  → {PAUSE_SECONDS}초 정지, 객체 인식 대기 중...")
            time.sleep(PAUSE_SECONDS)

            # current_index == 0 은 waypoints[0] = point_1(시작점).
            # i == 0(맨 처음 출발)은 제외하고, 그 이후에 다시 1번에 도착했다면
            # 정방향+역방향을 다 돌고 시작점으로 돌아온 것 = 한 바퀴(왕복) 완료
            if current_index == 0 and i > 0:
                print("  → 한 바퀴(왕복) 완료! 서버에 순찰 종료 알림")
                notify_patrol_end()

                print(f"  → 다음 바퀴 시작 전 {ROUND_TRIP_PAUSE_SECONDS}초 대기...")
                time.sleep(ROUND_TRIP_PAUSE_SECONDS)

                print("  → 새 바퀴 시작 -> 서버에 순찰 시작 알림")
                notify_patrol_start()

            i += 1
    except KeyboardInterrupt:
        pass
    finally:
        # 순찰 도중에 꺼졌어도, 지금까지 진행된 바퀴는 "종료"로 마무리해줌
        notify_patrol_end()
        nav.lifecycleShutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
