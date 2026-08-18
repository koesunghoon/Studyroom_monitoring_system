import os
import time
import math
import random
import sqlite3
import threading
from datetime import datetime
from collections import deque

from flask import Flask, Response, render_template, jsonify, request
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.time import Time
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
PI_STREAM_URL = "http://192.168.0.4:8000/video_feed"   # 실제 파이 IP:포트로 ros2 topic list --no-daemon | grep map교체

# ============================================================
# YOLO 모델 (sit / fallen 2클래스)
# ============================================================
yolo_model = YOLO(MODEL_PATH)
CLASS_NAMES = yolo_model.names
print("모델 클래스 확인:", CLASS_NAMES)

ALERT_CLASSES = {"fallen"}
CONF_THRESHOLD = 0.5
ALERT_COOLDOWN_SEC = 10
BOX_COLORS = {
    "sit": (0, 200, 0),
    "fallen": (0, 0, 255),
}
_last_alert_ts = {}


# ============================================================
# 파이 스트림에서 "항상 최신 프레임만" 읽어오는 백그라운드 리더
# ============================================================
class LatestFrameReader:
    def __init__(self, url):
        self.url = url
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.cap = cv2.VideoCapture(self.url)
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        while self.running:
            if not self.cap.isOpened():
                self.cap.release()
                time.sleep(1)
                self.cap = cv2.VideoCapture(self.url)
                continue
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self.lock:
                self.frame = frame

    def get_frame(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()


frame_reader = LatestFrameReader(PI_STREAM_URL)


# ============================================================
# 공유 상태 (rclpy 노드가 씀 <-> Flask 라우트가 읽음)
# ============================================================
_state_lock = threading.Lock()

_robot_state = {
    "robot_state": "순찰 중",
    "zone": "2F 열람실",
    "battery": 0,
    "patrol_count_today": 6,
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
# rclpy 백그라운드 노드 — 배터리 / SLAM 맵 / 로봇 위치(tf) 구독
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
            return  # 아직 맵/tf가 안 잡혔으면 그냥 넘어감 (에러 아님)

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
            handled TEXT DEFAULT '확인 대기'
        );
        """
    )
    conn.commit()

    if conn.execute("SELECT COUNT(*) c FROM attendance").fetchone()["c"] == 0:
        seed = [
            ("09:02:14", "S24-018", "07번", "입실", "정상"),
            ("09:14:41", "S24-005", "12번", "입실", "정상"),
            ("10:03:02", "S24-011", "-", "퇴실", "정상"),
        ]
        conn.executemany(
            "INSERT INTO attendance (ts, student_id, seat, action, status) VALUES (?,?,?,?,?)",
            seed,
        )
    if conn.execute("SELECT COUNT(*) c FROM patrol_log").fetchone()["c"] == 0:
        seed = [
            ("08:00:00", "2F 열람실 A", "이상없음", "04:12", "완료"),
            ("09:00:00", "2F 열람실 B", "이석 1건", "03:58", "주의"),
            ("10:00:00", "2F 열람실 A", "이상없음", "04:05", "완료"),
        ]
        conn.executemany(
            "INSERT INTO patrol_log (ts, zone, result, duration, status) VALUES (?,?,?,?,?)",
            seed,
        )
    if conn.execute("SELECT COUNT(*) c FROM warning_log").fetchone()["c"] == 0:
        seed = [
            ("08:41:09", "12번", "장시간 이석", 0.88, "확인됨"),
            ("07:52:33", "05번", "물품 방치", 0.79, "확인됨"),
        ]
        conn.executemany(
            "INSERT INTO warning_log (ts, seat, wtype, confidence, handled) VALUES (?,?,?,?,?)",
            seed,
        )
    conn.commit()
    conn.close()


def insert_warning(seat, wtype, confidence, handled="확인 대기"):
    now = datetime.now().strftime("%H:%M:%S")
    conn = get_db()
    conn.execute(
        "INSERT INTO warning_log (ts, seat, wtype, confidence, handled) VALUES (?,?,?,?,?)",
        (now, seat, wtype, confidence, handled),
    )
    conn.commit()
    conn.close()


# ============================================================
# 페이지
# ============================================================
@app.route("/")
def index():
    return render_template("index.html")


# ============================================================
# API: 로봇 상태 / KPI (battery는 이제 rclpy 노드가 실시간으로 채움)
# ============================================================
@app.route("/api/status")
def api_status():
    with _state_lock:
        return jsonify(dict(_robot_state))


@app.route("/api/robot_state", methods=["POST"])
def update_robot_state():
    """수동으로 상태를 덮어쓰고 싶을 때(예: 순찰 구역 변경)만 사용."""
    data = request.get_json(force=True)
    with _state_lock:
        _robot_state.update({k: v for k, v in data.items() if k in _robot_state})
    return jsonify({"ok": True})


# ============================================================
# API: SLAM 맵 (실데이터) — /map, 로봇 위치, 이동 궤적
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
# API: 좌석 현황 (YOLO 결과)
# ============================================================
SEAT_COUNT = 24
_seat_cache = None


def get_seats():
    global _seat_cache
    if _seat_cache is None:
        _seat_cache = []
        for i in range(SEAT_COUNT):
            r = random.random()
            state = "occupied" if r < 0.62 else ("empty" if r < 0.85 else "away")
            _seat_cache.append({"no": i + 1, "state": state})
    return _seat_cache


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
    rows = conn.execute(f"SELECT * FROM {table} ORDER BY id DESC LIMIT 50").fetchall()
    conn.close()
    return jsonify([dict(r) for r in rows])


# ============================================================
# API: 이벤트 트리거
# ============================================================
@app.route("/api/event", methods=["POST"])
def api_event():
    data = request.get_json(force=True)
    etype = data.get("type")

    if etype == "fall":
        insert_warning(data.get("seat", "07번"), "쓰러짐 의심", 0.94, "출동중")
    elif etype == "away":
        insert_warning(data.get("seat", "16번"), "장시간 이석", 0.86, "확인 대기")
    elif etype == "intrusion":
        insert_warning("-", "비인가 침입", 0.81, "확인 대기")
    else:
        return jsonify({"error": "unknown event type"}), 400

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

        results = yolo_model(frame, verbose=False)[0]
        for box in results.boxes:
            label = CLASS_NAMES[int(box.cls[0])]
            conf = float(box.conf[0])
            if conf < CONF_THRESHOLD:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            color = BOX_COLORS.get(label, (255, 255, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame, f"{label} {conf:.2f}", (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2,
            )

            if label in ALERT_CLASSES:
                last_ts = _last_alert_ts.get(label, 0)
                if time.time() - last_ts > ALERT_COOLDOWN_SEC:
                    insert_warning("웹캠", "쓰러짐 의심", conf, "확인 대기")
                    _last_alert_ts[label] = time.time()

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