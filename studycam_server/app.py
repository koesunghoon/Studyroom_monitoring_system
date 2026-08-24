"""
STUDYCAM 관제 서버
------------------
WSL(ROS2 Humble) 안에서 돌아가는 Flask 서버.
파이(터틀봇)가 8000번 포트로 올리는 원본 웹캠 스트림을 받아 YOLO 추론 후 재전송하고,
같은 프로세스 안에서 백그라운드 rclpy 노드가 /battery_state, /map, /tf(map->base_footprint),
/odom을 구독해서 배터리 잔량, SLAM 지도, 순찰 여부를 실데이터로 채웁니다.
"""

import os
import re
import time
import math
import socket
import sqlite3
import threading
from datetime import datetime
from collections import deque

from flask import Flask, Response, render_template, jsonify, request

import cv2
import numpy as np
import requests
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf2_ros import TransformException

APP_DIR = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.join(APP_DIR, "studycam.db")
MODEL_PATH = os.path.join(APP_DIR, "best.pt")

app = Flask(__name__)

# ============================================================
# 파이(터틀봇) 웹캠 스트림 주소
# ============================================================
PI_STREAM_URL = "http://192.168.0.4:8000/video_feed"   # 실제 파이 IP:포트로 교체

# 단위는 mm — DEPTH_SCALE_M으로 미터 변환 (팀원 카메라 실측값 확인 완료)
PI_DEPTH_URL = "http://192.168.0.4:8000/depth_feed"
DEPTH_SCALE_M = 0.0010000000474974513  # raw depth 값(정수) * 이 값 = 미터 (팀원 실측값)

# ============================================================
# YOLO 모델 (sit / fallen / empty / item 4클래스)
# ============================================================
yolo_model = YOLO(MODEL_PATH)
CLASS_NAMES = yolo_model.names
print("모델 클래스 확인:", CLASS_NAMES)

# 박스를 화면에 "그리는" 기준 (관대하게 - 시각적 확인용)
CONF_THRESHOLD = 0.5

# 실제로 DB에 경고를 기록하고 알림을 띄우는 기준 (더 엄격하게)
# - 신뢰도가 ALERT_CONF_THRESHOLD 이상인 프레임이
#   ALERT_MIN_DURATION_SEC 만큼 "연속으로" 나와야 진짜 이벤트로 인정 (단발성/회전중 오탐 방지)
ALERT_CONF_THRESHOLD = 0.6
ALERT_MIN_DURATION_SEC = 2.5  # 이 시간(초) 이상 "연속으로" 감지돼야 진짜 이벤트로 인정
ALERT_COOLDOWN_SEC = 10
ALERT_TRIGGER_CLASSES = {"fallen"}

# 분실물 감지 클래스 (best.pt)
LOST_ITEM_CLASSES = {"item"}

BOX_COLORS = {
    "sit": (0, 200, 0),
    "fallen": (0, 0, 255),
    "item": (200, 80, 160),   # 보라 계열 (BGR) - sit(초록)/fallen(빨강)과 구분되는 색
    "empty": (255, 255, 255),  # 흰색 (명시적으로 지정 - 기본값과 같지만 의도를 분명히 함)
}
_last_alert_ts = {}
_streak_start_ts = {}  # 클래스명 -> 지금 연속 감지가 "언제부터" 시작됐는지 (끊기면 None)
_alert_active = {}     # 클래스명 -> 지금 "미해결 상태로 활성화된" 경고가 있는지 (조치 전까지 재발동 안 함)

# ============================================================
# 카메라 내부 파라미터 (거리 -> 맵 좌표 변환에 필요)
# pyrealsense2로 아래처럼 뽑아서 정확한 값으로 교체
#   profile = pipeline.get_active_profile()
#   intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
#   print(intr.fx, intr.fy, intr.ppx, intr.ppy)
# 지금은 D435 640x480 기준 일반적인 근사값으로 임시 세팅해둠.
# ============================================================
DEPTH_FX = 381.6277160644531
DEPTH_FY = 381.6277160644531
DEPTH_PPX = 317.9629211425781
DEPTH_PPY = 238.81799707093125

# 로봇 중심 기준 카메라 장착 위치 (m) - 실측값(cm)을 미터로 변환해서 반영
# x: 로봇 중심에서 전방으로 얼마나 떨어졌는지, y: 좌우 오프셋
# 실측: x=7.5cm, y=8cm  (y의 좌/우 방향(부호)은 실제로 돌려보고 반대면 부호만 뒤집으면 됨)
CAM_OFFSET_X = 0.075
CAM_OFFSET_Y = 0.08

# ============================================================
# 7개 방의 맵 좌표 "중심점" (x, y), 단위: m
# patrol_node.py(자율주행 웨이포인트)에서 실측한 좌표를 그대로 가져다 씀.
# ============================================================
MAX_ROOM_DISTANCE = 1.5  # 가장 가까운 방 중심점이라도 이보다 멀면 "미확인 구역" 처리 (m)

# patrol.py 웨이포인트 좌표 (point_3 ~ point_9 -> 1~7번 좌석, 1인실 7개)
# point_1은 시작 위치, point_2는 방과 무관한 경유점이라 매핑에서 제외
ROOM_CENTERS = {
    1: (0.7443546017221914, 1.1606690109190736),    # point_3
    2: (0.9720431379366761, 0.33897203992241903),   # point_4
    3: (1.2140891297116543, 1.2011701385908178),    # point_5
    4: (1.3507324382874042, 0.38942217704970494),   # point_6
    5: (1.5658655966192034, 1.2440643100650892),    # point_7
    6: (1.8126727979489197, 0.14460677061396246),   # point_8
    7: (2.0434873863022585, 1.1071957958898084),    # point_9
}


def pixel_depth_to_map_xy(cx, cy, depth_m, robot_pose):
    """
    카메라 픽셀 좌표(cx, cy) + 거리(depth_m)를 맵 좌표(x, y)로 변환.
    로봇 위치(tf)가 아직 유효하지 않거나 거리 정보가 없으면 None.
    """
    if depth_m is None or not robot_pose.get("valid"):
        return None

    # 카메라 광학 좌표계(x=오른쪽, y=아래, z=전방) -> 로봇 기준(x=전방, y=좌측)
    cam_x = (cx - DEPTH_PPX) * depth_m / DEPTH_FX   # 오른쪽(+)
    cam_z = depth_m                                  # 전방(+)

    robot_x = CAM_OFFSET_X + cam_z
    robot_y = CAM_OFFSET_Y - cam_x

    yaw = robot_pose["yaw"]
    map_x = robot_pose["x"] + robot_x * math.cos(yaw) - robot_y * math.sin(yaw)
    map_y = robot_pose["y"] + robot_x * math.sin(yaw) + robot_y * math.cos(yaw)
    return map_x, map_y


def find_room(map_xy):
    """맵 좌표에서 가장 가까운 방 중심점을 찾음. 너무 멀거나 캘리브레이션 안 됐으면 None."""
    if map_xy is None:
        return None
    map_x, map_y = map_xy

    best_room, best_dist = None, None
    for room_no, center in ROOM_CENTERS.items():
        if center is None:
            continue
        dx, dy = map_x - center[0], map_y - center[1]
        dist = math.hypot(dx, dy)
        if best_dist is None or dist < best_dist:
            best_room, best_dist = room_no, dist

    if best_room is None or best_dist > MAX_ROOM_DISTANCE:
        return None
    return best_room


# ============================================================
# 파이 스트림에서 "항상 최신 프레임만" 읽어오는 백그라운드 리더
# ============================================================
class LatestFrameReader:
    """
    /video_feed 는 컬러(JPEG) 멀티파트 스트림.
    cv2.VideoCapture(HTTP MJPEG)는 내부적으로 FFmpeg를 쓰는데, 서버 구현(waitress 등)에 따라
    청크 전송 방식이 조금만 달라져도 스트림을 못 여는 경우가 있어서,
    뎁스 리더와 동일하게 requests로 직접 멀티파트를 파싱하는 방식으로 통일함.
    """
    def __init__(self, url):
        self.url = url
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        placeholder_set = False
        while self.running:
            try:
                resp = requests.get(self.url, stream=True, timeout=5)
                buf = b""
                for chunk in resp.iter_content(chunk_size=4096):
                    if not self.running:
                        break
                    buf += chunk
                    start = buf.find(b"\xff\xd8")  # JPEG SOI 마커
                    end = buf.find(b"\xff\xd9")    # JPEG EOI 마커
                    if start != -1 and end != -1 and end > start:
                        end_full = end + 2
                        jpg_bytes = buf[start:end_full]
                        buf = buf[end_full:]
                        arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if img is not None:
                            with self.lock:
                                self.frame = img
                            placeholder_set = False
            except Exception:
                if not placeholder_set:
                    blank = 128 * np.ones((480, 640, 3), dtype="uint8")
                    cv2.putText(
                        blank, "PI STREAM NOT REACHABLE", (30, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
                    )
                    with self.lock:
                        self.frame = blank
                    placeholder_set = True
                time.sleep(1)

    def get_frame(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()


class LatestDepthReader:
    """
    /depth_feed 는 컬러 스트림과 다르게 16비트 PNG 프레임이라
    cv2.VideoCapture(HTTP MJPEG용)로는 제대로 못 읽어서, 직접 멀티파트 스트림을 파싱함.
    엔드포인트가 아직 없거나 꺼져있어도 앱이 죽지 않고 get_frame()이 None을 반환하도록 처리.
    """
    def __init__(self, url):
        self.url = url
        self.frame = None  # uint16 numpy array (H, W), 단위: raw depth (보통 mm)
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    @staticmethod
    def _find_png_end(buf, start):
        """
        PNG 청크 구조(길이+타입+데이터+CRC)를 따라가면서 진짜 IEND 청크의 끝 위치를 찾음.
        압축된 픽셀 데이터 안에 우연히 "IEND" 텍스트가 섞여 나오는 오탐(false positive)을 막기 위해,
        단순 텍스트 검색 대신 각 청크 길이만큼 정확히 건너뛰며 확인함.
        반환값: 다음 청크가 아직 다 안 왔으면 None, 찾았으면 그 끝 위치(int)
        """
        pos = start + 8  # PNG 시그니처(8바이트) 다음부터 청크가 시작됨
        while pos + 8 <= len(buf):
            length = int.from_bytes(buf[pos:pos + 4], "big")
            chunk_type = buf[pos + 4:pos + 8]
            chunk_end = pos + 8 + length + 4  # 길이(4)+타입(4) + 데이터(length) + CRC(4)
            if chunk_end > len(buf):
                return None  # 이 청크가 아직 다 도착 안 함 -> 더 받아야 함
            if chunk_type == b"IEND":
                return chunk_end
            pos = chunk_end
        return None

    def _reader(self):
        while self.running:
            try:
                resp = requests.get(self.url, stream=True, timeout=5)
                buf = b""
                for chunk in resp.iter_content(chunk_size=4096):
                    if not self.running:
                        break
                    buf += chunk
                    start = buf.find(b"\x89PNG")
                    if start == -1:
                        continue
                    end_full = self._find_png_end(buf, start)
                    if end_full is None:
                        continue  # 아직 프레임이 다 안 왔음, 다음 청크 더 받기

                    png_bytes = buf[start:end_full]
                    buf = buf[end_full:]
                    arr = np.frombuffer(png_bytes, dtype=np.uint8)
                    depth_img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
                    if depth_img is not None:
                        with self.lock:
                            self.frame = depth_img
            except Exception:
                # 아직 뎁스 엔드포인트가 안 켜져 있거나 접속 실패 -> 잠깐 쉬고 재시도
                time.sleep(2)

    def get_frame(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()


def get_distance_m(depth_frame, cx, cy, color_w, color_h):
    """컬러 프레임 좌표(cx, cy) 기준으로 뎁스 프레임에서 거리(m)를 조회."""
    if depth_frame is None:
        return None
    dh, dw = depth_frame.shape[:2]
    # 컬러/뎁스 해상도가 다를 수 있으니 비율로 좌표 변환
    dx = int(cx * dw / color_w)
    dy = int(cy * dh / color_h)
    dx = max(0, min(dw - 1, dx))
    dy = max(0, min(dh - 1, dy))

    raw = int(depth_frame[dy, dx])
    if raw == 0:
        # 0은 "측정 안 됨" 구멍인 경우가 많아서, 주변 3x3에서 유효값 찾아 대체
        region = depth_frame[max(0, dy - 2):dy + 3, max(0, dx - 2):dx + 3]
        valid = region[region > 0]
        if valid.size == 0:
            return None
        raw = int(np.median(valid))

    return round(raw * DEPTH_SCALE_M, 2)


frame_reader = LatestFrameReader(PI_STREAM_URL)
depth_reader = LatestDepthReader(PI_DEPTH_URL)


# ============================================================
# 공유 상태 (rclpy 노드가 씀 <-> Flask 라우트가 읽음)
# ============================================================
_state_lock = threading.Lock()

_robot_state = {
    "robot_state": "순찰 중",
    "zone": "열람실",
    "battery": 0,
    "patrol_count_today": 0,   # 서버 시작 시점부터 카운트 (DB에 날짜 컬럼이 없어 "오늘" 필터링은 추후 보완)
    "patrol_active": False,    # 지금 한 바퀴(전체 웨이포인트) 순찰이 진행 중인지
}

_map_state = {
    "width": 0,
    "height": 0,
    "resolution": 0.05,
    "origin_x": 0.0,
    "origin_y": 0.0,
    "data": [],
}

_robot_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "valid": False}
_trail = deque(maxlen=300)


# ============================================================
# DB (SQLite) — 출결 관리 / 순찰 로그 / 경고 이력
# ============================================================
def get_db():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


def init_db():
    conn = get_db()
    conn.executescript(
        """
        CREATE TABLE IF NOT EXISTS students (
            student_id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            fingerprint_slot INTEGER UNIQUE,
            seat_no INTEGER
        );
        CREATE TABLE IF NOT EXISTS attendance (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ts TEXT NOT NULL,
            student_id TEXT NOT NULL,
            seat TEXT,
            action TEXT NOT NULL,
            status TEXT DEFAULT '정상'
        );
        CREATE TABLE IF NOT EXISTS patrol_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ts TEXT NOT NULL,
            zone TEXT NOT NULL,
            result TEXT NOT NULL,
            duration TEXT,
            status TEXT DEFAULT '완료'
        );
        CREATE TABLE IF NOT EXISTS warning_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ts TEXT NOT NULL,
            seat TEXT,
            wtype TEXT NOT NULL,
            confidence REAL,
            distance_m REAL,
            handled TEXT DEFAULT '확인 대기'
        );
        """
    )
    conn.commit()

    # 기존에 이미 만들어진 DB 파일에 distance_m 컬럼이 없으면 추가 (마이그레이션)
    cols = [row["name"] for row in conn.execute("PRAGMA table_info(warning_log)")]
    if "distance_m" not in cols:
        conn.execute("ALTER TABLE warning_log ADD COLUMN distance_m REAL")
        conn.commit()

    # 학생 - 지문 슬롯 - 좌석 매핑 (지문 슬롯 번호를 좌석 번호와 동일하게 배정)
    if conn.execute("SELECT COUNT(*) c FROM students").fetchone()["c"] == 0:
        students_seed = [
            ("S01", "김준혁", 1, 1),
            ("S02", "고경민", 2, 2),
            ("S03", "김재민", 3, 3),
            ("S04", "고성훈", 4, 4),
            ("S05", "고지훈", 5, 5),
            ("S06", "이경한", 6, 6),
            ("S07", "고롱롱", 7, 7),
        ]
        conn.executemany(
            "INSERT INTO students (student_id, name, fingerprint_slot, seat_no) VALUES (?,?,?,?)",
            students_seed,
        )

    # 출결 기록은 실제 지문 인식 연동 전까지는 비워둠 (가짜 데이터 넣지 않음)
    if conn.execute("SELECT COUNT(*) c FROM warning_log").fetchone()["c"] == 0:
        seed = [
            ("07:52:33", "05번", "물품 방치", 0.79, None, "확인됨"),
        ]
        conn.executemany(
            "INSERT INTO warning_log (ts, seat, wtype, confidence, distance_m, handled) VALUES (?,?,?,?,?,?)",
            seed,
        )
    conn.commit()
    conn.close()


def insert_warning(seat, wtype, confidence, distance_m=None, handled="확인 대기"):
    """경고 이력을 DB에 적재하고, 방금 만든 행의 id를 반환 (프론트가 중복 알림 방지에 사용)."""
    now = datetime.now().strftime("%H:%M:%S")
    conn = get_db()
    cur = conn.execute(
        "INSERT INTO warning_log (ts, seat, wtype, confidence, distance_m, handled) VALUES (?,?,?,?,?,?)",
        (now, seat, wtype, confidence, distance_m, handled),
    )
    row_id = cur.lastrowid
    conn.commit()
    conn.close()
    return row_id


def insert_patrol_start(zone):
    """로봇이 움직이기 시작한 순간 순찰 기록을 하나 생성하고, 그 row id를 반환."""
    now = datetime.now().strftime("%H:%M:%S")
    conn = get_db()
    cur = conn.execute(
        "INSERT INTO patrol_log (ts, zone, result, duration, status) VALUES (?,?,?,?,?)",
        (now, zone, "진행 중", "-", "진행중"),
    )
    row_id = cur.lastrowid
    conn.commit()
    conn.close()
    return row_id


def update_patrol_end(row_id, duration_sec):
    """로봇이 멈췄을 때, 아까 만든 순찰 기록에 소요시간/결과를 채워넣음."""
    mm = int(duration_sec // 60)
    ss = int(duration_sec % 60)
    duration_str = f"{mm:02d}:{ss:02d}"
    conn = get_db()
    conn.execute(
        "UPDATE patrol_log SET result=?, duration=?, status=? WHERE id=?",
        ("이상없음", duration_str, "완료", row_id),
    )
    conn.commit()
    conn.close()


def parse_ts_today(ts_str):
    """DB에 저장된 'HH:MM:SS' 문자열을 오늘 날짜 기준 datetime으로 변환."""
    t = datetime.strptime(ts_str, "%H:%M:%S").time()
    return datetime.combine(datetime.now().date(), t)


# ============================================================
# rclpy 백그라운드 노드 — 배터리 / SLAM 맵 / 로봇 위치(tf) / 순찰 감지(odom)
# ============================================================
class RosBridge(Node):
    def __init__(self):
        super().__init__("studycam_bridge")

        self.create_subscription(BatteryState, "/battery_state", self.on_battery, 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.5, self.update_pose)

        self.get_logger().info("studycam_bridge 시작 - battery/map/tf 구독 중...")

    def on_battery(self, msg: BatteryState):
        with _state_lock:
            _robot_state["battery"] = round(msg.percentage)  # 터틀봇3는 0~100 그대로 들어옴

    def on_map(self, msg: OccupancyGrid):
        with _state_lock:
            _map_state["width"] = msg.info.width
            _map_state["height"] = msg.info.height
            _map_state["resolution"] = msg.info.resolution
            _map_state["origin_x"] = msg.info.origin.position.x
            _map_state["origin_y"] = msg.info.origin.position.y
            _map_state["data"] = list(msg.data)

    def update_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
        except TransformException:
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        with _state_lock:
            _robot_pose.update({"x": x, "y": y, "yaw": yaw, "valid": True})
            _trail.append((x, y))


def start_ros_bridge():
    rclpy.init()
    node = RosBridge()
    rclpy.spin(node)


ros_thread = threading.Thread(target=start_ros_bridge, daemon=True)
ros_thread.start()


# ============================================================
# 지문인식(STM32 + ESP01) 출결 수신 - 순수 TCP 소켓 서버
# ESP01은 AT+CIPSEND로 순수 텍스트만 던지는 방식이라 HTTP(Flask, 5000번)와는
# 별도 포트에서 받아야 함. 기대하는 메시지 형식: "FP,<지문 슬롯 번호>\n"
# ============================================================
FINGERPRINT_TCP_PORT = 5001


def record_attendance(slot):
    """지문 슬롯 번호를 받아서 학생을 조회하고, 입실/퇴실을 자동 판별해 출결 기록."""
    conn = get_db()
    student = conn.execute(
        "SELECT * FROM students WHERE fingerprint_slot=?", (slot,)
    ).fetchone()

    if student is None:
        conn.close()
        print(f"[지문 출결] 슬롯 {slot} - 등록되지 않은 학생")
        return

    # 이 학생의 마지막 기록이 "입실"이면 이번엔 "퇴실"로, 아니면 "입실"로 토글
    last = conn.execute(
        "SELECT action FROM attendance WHERE student_id=? ORDER BY id DESC LIMIT 1",
        (student["student_id"],),
    ).fetchone()
    action = "퇴실" if (last is not None and last["action"] == "입실") else "입실"

    seat_text = f"{student['seat_no']}번 좌석" if student["seat_no"] is not None else "-"
    now = datetime.now().strftime("%H:%M:%S")
    conn.execute(
        "INSERT INTO attendance (ts, student_id, seat, action, status) VALUES (?,?,?,?,?)",
        (now, student["student_id"], seat_text, action, "정상"),
    )
    conn.commit()
    conn.close()

    if student["seat_no"] is not None:
        set_seat_state_internal(student["seat_no"], "occupied" if action == "입실" else "empty")

    print(f"[지문 출결] {student['name']}({student['student_id']}) - {action}")


def handle_fingerprint_line(line):
    line = line.strip()
    if not line.startswith("FP,"):
        return
    try:
        slot = int(line.split(",", 1)[1])
    except (IndexError, ValueError):
        print(f"[지문 출결] 형식 이상: {line!r}")
        return
    record_attendance(slot)


# 지금 연결돼 있는 STM32 소켓 (문열림 신호를 되돌려 보낼 때 씀)
_fingerprint_conn = None
_fingerprint_conn_lock = threading.Lock()


def send_door_open_command():
    """조치 완료 시 STM32에 문열림 신호를 보냄. 연결 안 돼있으면 조용히 실패."""
    with _fingerprint_conn_lock:
        conn = _fingerprint_conn
    if conn is None:
        print("[문열림 신호] STM32가 연결되어 있지 않아 전송하지 못함")
        return False
    try:
        conn.sendall(b"DOOR_OPEN\n")
        print("[문열림 신호] STM32로 전송함")
        return True
    except Exception as e:
        print(f"[문열림 신호] 전송 실패: {e}")
        return False


def fingerprint_server_loop():
    global _fingerprint_conn

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", FINGERPRINT_TCP_PORT))
    srv.listen(1)
    print(f"지문 출결 TCP 서버 시작: 0.0.0.0:{FINGERPRINT_TCP_PORT}")

    while True:
        conn, addr = srv.accept()
        print(f"[지문 출결] STM32 연결됨: {addr}")
        with _fingerprint_conn_lock:
            _fingerprint_conn = conn

        buf = b""
        try:
            with conn:
                while True:
                    chunk = conn.recv(64)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        handle_fingerprint_line(line.decode(errors="ignore"))
        except Exception as e:
            print(f"[지문 출결] 연결 처리 중 오류: {e}")
        finally:
            with _fingerprint_conn_lock:
                if _fingerprint_conn is conn:
                    _fingerprint_conn = None


fingerprint_thread = threading.Thread(target=fingerprint_server_loop, daemon=True)
fingerprint_thread.start()


# ============================================================
# 페이지
# ============================================================
@app.route("/")
def index():
    return render_template("index.html")


# ============================================================
# API: 로봇 상태 / KPI
# ============================================================
@app.route("/api/status")
def api_status():
    with _state_lock:
        return jsonify(dict(_robot_state))


@app.route("/api/robot_state", methods=["POST"])
def update_robot_state():
    data = request.get_json(force=True)
    with _state_lock:
        _robot_state.update({k: v for k, v in data.items() if k in _robot_state})
    return jsonify({"ok": True})


# ============================================================
# API: 순찰 시작/종료 (patrol_node.py가 "한 바퀴"의 시작/끝 시점에 직접 호출)
# /odom 움직임만으로는 "웨이포인트 사이 잠깐 정지"와 "한 바퀴 다 돌고 끝"을 구분할 수
# 없어서, 순찰 스크립트 자신이 정확한 시점을 알려주는 방식으로 처리함.
# ============================================================
@app.route("/api/patrol/start", methods=["POST"])
def api_patrol_start():
    zone = _robot_state.get("zone", "-")
    row_id = insert_patrol_start(zone)
    with _state_lock:
        _robot_state["patrol_count_today"] += 1
        _robot_state["patrol_active"] = True
    return jsonify({"ok": True, "id": row_id})


@app.route("/api/patrol/end", methods=["POST"])
def api_patrol_end():
    conn = get_db()
    row = conn.execute(
        "SELECT * FROM patrol_log WHERE status='진행중' ORDER BY id DESC LIMIT 1"
    ).fetchone()
    conn.close()

    if row is None:
        return jsonify({"ok": False, "error": "진행중인 순찰 기록이 없습니다"}), 400

    duration_sec = (datetime.now() - parse_ts_today(row["ts"])).total_seconds()
    if duration_sec < 0:
        duration_sec += 24 * 3600  # 자정을 넘긴 경우 보정

    update_patrol_end(row["id"], duration_sec)
    with _state_lock:
        _robot_state["patrol_active"] = False
    return jsonify({"ok": True})


# ============================================================
# API: SLAM 맵 (실데이터)
# ============================================================
@app.route("/api/map")
def api_map():
    with _state_lock:
        return jsonify({
            "width": _map_state["width"],
            "height": _map_state["height"],
            "resolution": _map_state["resolution"],
            "origin_x": _map_state["origin_x"],
            "origin_y": _map_state["origin_y"],
            "data": _map_state["data"],
            "robot": dict(_robot_pose),
            "trail": list(_trail),
        })


# ============================================================
# API: 좌석 현황 (YOLO 결과 - 아직 목업, 좌석 수만 7개로 고정)
# YOLO 연동: 추론 파이프라인에서 좌석별 상태를 계산해 이 캐시를 갱신
# 지문인식 연동: 출결(attendance)은 센서 붙으면 실데이터로 교체
# ============================================================
SEAT_COUNT = 7
_seat_cache = None


def get_seats():
    global _seat_cache
    if _seat_cache is None:
        _seat_cache = []
        for seat_no in range(1, SEAT_COUNT + 1):
            _seat_cache.append({"no": seat_no, "state": get_seat_state_from_attendance(seat_no)})
    return _seat_cache


def set_seat_state_internal(seat_no, state):
    """서버 내부(YOLO 감지, 조치 처리 등)에서 좌석 상태를 직접 바꿀 때 쓰는 헬퍼."""
    seats = get_seats()
    for s in seats:
        if s["no"] == seat_no:
            s["state"] = state


def get_seat_state_from_attendance(seat_no):
    """이 좌석에 배정된 학생이 지금 입실 상태인지 DB 기준으로 확인."""
    conn = get_db()
    student = conn.execute(
        "SELECT student_id FROM students WHERE seat_no=?", (seat_no,)
    ).fetchone()
    if student is None:
        conn.close()
        return "empty"

    last = conn.execute(
        "SELECT action FROM attendance WHERE student_id=? ORDER BY id DESC LIMIT 1",
        (student["student_id"],),
    ).fetchone()
    conn.close()
    return "occupied" if (last is not None and last["action"] == "입실") else "empty"


def force_checkout_seat(seat_no):
    """위급상황 조치 완료 시, 그 좌석 학생을 강제로 퇴실 처리 (실제 출결 기록도 남김)."""
    conn = get_db()
    student = conn.execute(
        "SELECT * FROM students WHERE seat_no=?", (seat_no,)
    ).fetchone()

    if student is None:
        conn.close()
        set_seat_state_internal(seat_no, "empty")
        return

    last = conn.execute(
        "SELECT action FROM attendance WHERE student_id=? ORDER BY id DESC LIMIT 1",
        (student["student_id"],),
    ).fetchone()

    if last is not None and last["action"] == "입실":
        now = datetime.now().strftime("%H:%M:%S")
        conn.execute(
            "INSERT INTO attendance (ts, student_id, seat, action, status) VALUES (?,?,?,?,?)",
            (now, student["student_id"], f"{seat_no}번 좌석", "퇴실", "정상"),
        )
        conn.commit()

    conn.close()
    set_seat_state_internal(seat_no, "empty")


@app.route("/api/seats")
def api_seats():
    return jsonify(get_seats())


@app.route("/api/seats/<int:no>", methods=["POST"])
def set_seat(no):
    data = request.get_json(force=True)
    seats = get_seats()
    for s in seats:
        if s["no"] == no:
            s["state"] = data.get("state", s["state"])
    return jsonify({"ok": True})


# ============================================================
# API: 로그 조회
# ============================================================
_LOG_TABLES = {
    "attendance": "attendance",
    "patrol": "patrol_log",
    "warning": "warning_log",
}


@app.route("/api/logs/<kind>")
def api_logs(kind):
    table = _LOG_TABLES.get(kind)
    if not table:
        return jsonify({"error": "unknown log kind"}), 404

    conn = get_db()
    if kind == "attendance":
        # student_id(예: S01)는 그대로 두고, 화면 표시용 이름만 조인해서 같이 내려줌
        rows = conn.execute(
            """
            SELECT a.*, s.name AS student_name
            FROM attendance a
            LEFT JOIN students s ON a.student_id = s.student_id
            ORDER BY a.id DESC LIMIT 50
            """
        ).fetchall()
    else:
        rows = conn.execute(f"SELECT * FROM {table} ORDER BY id DESC LIMIT 50").fetchall()
    conn.close()
    return jsonify([dict(r) for r in rows])


# ============================================================
# API: 경고 조치 완료 처리
# - DB 행 상태 업데이트
# - 해당 좌석을 다시 빈 좌석으로 되돌림
# - 쓰러짐 감지 상태를 리셋해서, 다음번 진짜 상황은 다시 감지되게 함
# ============================================================
@app.route("/api/warning/<int:warning_id>/resolve", methods=["POST"])
def resolve_warning(warning_id):
    data = request.get_json(silent=True) or {}
    status = data.get("status", "완료")

    conn = get_db()
    row = conn.execute("SELECT * FROM warning_log WHERE id=?", (warning_id,)).fetchone()
    if row is None:
        conn.close()
        return jsonify({"ok": False, "error": "해당 경고 기록을 찾을 수 없습니다"}), 404

    conn.execute("UPDATE warning_log SET handled=? WHERE id=?", (status, warning_id))
    conn.commit()
    conn.close()

    # "N번 좌석" 형식에서 번호 추출해서 강제 퇴실 처리 (위급상황 조치 완료 = 퇴실로 간주)
    match = re.match(r"(\d+)번 좌석", row["seat"] or "")
    if match:
        force_checkout_seat(int(match.group(1)))

    # "조치 완료"(오탐 아님)일 때만 STM32 문열림 신호 전송
    if status == "완료":
        send_door_open_command()

    # 쓰러짐 감지였다면, 조치 완료됐으니 다시 새로운 상황으로 감지 가능하게 리셋
    if row["wtype"] == "쓰러짐 의심":
        _alert_active["fallen"] = False
        _streak_start_ts["fallen"] = None

    return jsonify({"ok": True})


# ============================================================
# API: 분실물 이력 조회 / 조치 완료
# (별도 테이블 안 만들고 warning_log를 wtype='분실물 의심'으로 필터링해서 재사용)
# ============================================================
@app.route("/api/logs/lost_items")
def api_logs_lost_items():
    conn = get_db()
    rows = conn.execute(
        "SELECT * FROM warning_log WHERE wtype='분실물 의심' ORDER BY id DESC LIMIT 50"
    ).fetchall()
    conn.close()
    return jsonify([dict(r) for r in rows])


@app.route("/api/lost_item/<int:item_id>/resolve", methods=["POST"])
def resolve_lost_item(item_id):
    conn = get_db()
    row = conn.execute(
        "SELECT * FROM warning_log WHERE id=? AND wtype='분실물 의심'", (item_id,)
    ).fetchone()

    if row is None:
        conn.close()
        return jsonify({"ok": False, "error": "해당 분실물 기록을 찾을 수 없습니다"}), 404

    conn.execute("UPDATE warning_log SET handled=? WHERE id=?", ("완료", item_id))
    conn.commit()
    conn.close()

    # 조치 완료됐으니, 같은 자리에 또 새 분실물이 생기면 다시 감지되도록 리셋
    for cls in LOST_ITEM_CLASSES:
        _alert_active[cls] = False
        _streak_start_ts[cls] = None

    return jsonify({"ok": True})


# ============================================================
# 웹캠 스트리밍 (MJPEG) — 파이 스트림 최신 프레임을 YOLO 처리 후 재전송
# ============================================================
def gen_frames():
    while True:
        frame = frame_reader.get_frame()
        if frame is None:
            time.sleep(0.1)
            continue

        depth_frame = depth_reader.get_frame()  # 뎁스 엔드포인트 없으면 None, 그래도 안 죽음
        frame_h, frame_w = frame.shape[:2]

        results = yolo_model(frame, verbose=False)[0]

        # 이번 프레임에서 각 클래스별로 감지된 "최고 신뢰도 박스" 하나씩만 기록
        # (연속 프레임 판정 + 위치 계산에 이 박스의 cx, cy, 거리를 그대로 씀)
        best_detection_this_frame = {}
        detected_labels_this_frame = set()  # empty 직접 감지 확인 및 분실물 판정용

        for box in results.boxes:
            label = CLASS_NAMES[int(box.cls[0])]
            conf = float(box.conf[0])
            if conf < CONF_THRESHOLD:
                continue

            detected_labels_this_frame.add(label)

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            color = BOX_COLORS.get(label, (255, 255, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            # 박스 중심점 기준으로 거리 조회 (뎁스 스트림 없으면 None -> 라벨에서 생략)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            dist_m = get_distance_m(depth_frame, cx, cy, frame_w, frame_h)
            label_text = f"{label} {conf:.2f}"
            if dist_m is not None:
                label_text += f" | {dist_m}m"

            cv2.putText(
                frame, label_text, (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2,
            )

            if label in (ALERT_TRIGGER_CLASSES | LOST_ITEM_CLASSES) and conf >= ALERT_CONF_THRESHOLD:
                cur = best_detection_this_frame.get(label)
                if cur is None or conf > cur["conf"]:
                    best_detection_this_frame[label] = {
                        "conf": conf, "cx": cx, "cy": cy, "dist_m": dist_m,
                    }

        # "좌석이 비어있다"를 sit 부재로 간접 추론하지 않고, empty 클래스 직접 감지로 확인
        # (카메라 각도 등으로 sit을 못 잡아내는 경우에 "없음"으로 오판하는 걸 방지)
        empty_confirmed_this_frame = "empty" in detected_labels_this_frame

        # 연속 감지 "지속 시간" 판정: 이번 프레임에 감지 안 됐으면 스트릭 초기화
        # (프레임 개수가 아니라 실제 경과 시간(초)으로 재서, 처리 속도가 빨라져도
        #  로봇이 회전하는 짧은 순간에 오발동하지 않도록 함)
        now_ts = time.time()

        # ---- 쓰러짐 판정 ----
        for cls in ALERT_TRIGGER_CLASSES:
            if cls in best_detection_this_frame:
                if _streak_start_ts.get(cls) is None:
                    _streak_start_ts[cls] = now_ts  # 이번이 연속 감지의 시작점
            else:
                _streak_start_ts[cls] = None
                _alert_active[cls] = False
                continue

            streak_duration = now_ts - _streak_start_ts[cls]
            if streak_duration >= ALERT_MIN_DURATION_SEC:
                # 이미 미해결 상태로 떠있는 경고가 있으면, 계속 감지되고 있어도 다시 DB에 안 넣음
                # (조치 완료되기 전까지는 같은 상황을 반복 기록하지 않음)
                if not _alert_active.get(cls, False):
                    det = best_detection_this_frame[cls]

                    with _state_lock:
                        pose_snapshot = dict(_robot_pose)
                    map_xy = pixel_depth_to_map_xy(det["cx"], det["cy"], det["dist_m"], pose_snapshot)
                    room_no = find_room(map_xy)
                    seat_label = f"{room_no}번 좌석" if room_no is not None else "미확인 구역"

                    print(
                        f"[방 판정 디버그] 연속감지 {streak_duration:.2f}초 | robot_pose={pose_snapshot} | "
                        f"det(cx,cy,dist_m)=({det['cx']},{det['cy']},{det['dist_m']}) | "
                        f"map_xy={map_xy} | room_no={room_no}"
                    )

                    insert_warning(
                        seat_label, "쓰러짐 의심", det["conf"],
                        distance_m=det["dist_m"], handled="확인 대기",
                    )
                    if room_no is not None:
                        set_seat_state_internal(room_no, "alert")

                    _last_alert_ts[cls] = now_ts
                    _alert_active[cls] = True

        # ---- 분실물 판정 (사람이 없을 때만) ----
        for cls in LOST_ITEM_CLASSES:
            if cls in best_detection_this_frame and empty_confirmed_this_frame:
                if _streak_start_ts.get(cls) is None:
                    _streak_start_ts[cls] = now_ts
            else:
                _streak_start_ts[cls] = None
                _alert_active[cls] = False
                continue

            streak_duration = now_ts - _streak_start_ts[cls]
            if streak_duration >= ALERT_MIN_DURATION_SEC:
                if not _alert_active.get(cls, False):
                    det = best_detection_this_frame[cls]

                    with _state_lock:
                        pose_snapshot = dict(_robot_pose)
                    map_xy = pixel_depth_to_map_xy(det["cx"], det["cy"], det["dist_m"], pose_snapshot)
                    room_no = find_room(map_xy)

                    # 방을 특정 못 하면 "공석 여부" 자체를 확인할 수 없으니 발동 보류
                    # (streak는 유지되니 다음 프레임에 조건 맞으면 바로 재시도됨)
                    seat_confirmed_empty = (
                        room_no is not None
                        and get_seat_state_from_attendance(room_no) == "empty"
                    )

                    if seat_confirmed_empty:
                        seat_label = f"{room_no}번 좌석"
                        print(
                            f"[분실물 판정 디버그] 연속감지 {streak_duration:.2f}초 | room_no={room_no}"
                        )
                        insert_warning(
                            seat_label, "분실물 의심", det["conf"],
                            distance_m=det["dist_m"], handled="확인 대기",
                        )
                        _last_alert_ts[cls] = now_ts
                        _alert_active[cls] = True

        ok, buf = cv2.imencode(".jpg", frame)
        if not ok:
            continue
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")


@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    init_db()
    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)