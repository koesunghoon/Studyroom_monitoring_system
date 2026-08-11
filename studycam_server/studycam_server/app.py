"""
STUDYCAM 관제 서버
------------------
로컬 PC에서 여는 Flask 서버. 지금은 웹캠 스트리밍 + SQLite 로그 DB까지만 실제로 동작하고,
좌석 현황/SLAM 맵은 아직 YOLO·ROS2가 안 붙어서 목업 데이터를 내려줍니다.
각 자리에 "TODO"로 나중에 뭘 실데이터로 바꿔야 하는지 표시해 뒀습니다.
"""

import os
import time
import random
import sqlite3
from datetime import datetime

from flask import Flask, Response, render_template, jsonify, request

import cv2
import numpy as np
from ultralytics import YOLO

APP_DIR = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.join(APP_DIR, "studycam.db")
MODEL_PATH = os.path.join(APP_DIR, "best.pt")

app = Flask(__name__)

# ============================================================
# YOLO 모델 (쓰러짐 감지: bending / fallen / falling / standing)
# ============================================================
yolo_model = YOLO(MODEL_PATH)
CLASS_NAMES = yolo_model.names
ALERT_CLASSES = {"fallen", "falling"}
CONF_THRESHOLD = 0.5
ALERT_COOLDOWN_SEC = 10
BOX_COLORS = {
    "fallen": (0, 0, 255),
    "falling": (0, 0, 255),
    "bending": (0, 200, 255),
    "standing": (0, 200, 0),
}
_last_alert_ts = {}


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
            action TEXT NOT NULL,     -- 입실 / 퇴실
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
            wtype TEXT NOT NULL,      -- 쓰러짐 의심 / 장시간 이석 / 비인가 침입 ...
            confidence REAL,
            handled TEXT DEFAULT '확인 대기'
        );
        """
    )
    conn.commit()

    # 최초 실행 시에만 샘플 데이터 시딩
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
# API: 로봇 상태 / KPI
# TODO(터틀봇 연동): 아래 값들을 rosbridge_suite(websocket) 또는
#   별도 브릿지 스크립트가 /api/robot_state 로 POST 해주는 실제 값으로 교체
# ============================================================
_robot_state = {
    "robot_state": "순찰 중",
    "zone": "2F 열람실",
    "battery": 78,
    "patrol_count_today": 6,
}


@app.route("/api/status")
def api_status():
    return jsonify(_robot_state)


@app.route("/api/robot_state", methods=["POST"])
def update_robot_state():
    """터틀봇/STM32 쪽에서 주기적으로 호출해 상태를 갱신하는 용도 (추후 사용)."""
    data = request.get_json(force=True)
    _robot_state.update({k: v for k, v in data.items() if k in _robot_state})
    return jsonify({"ok": True})


# ============================================================
# API: 좌석 현황 (YOLO 결과)
# TODO(YOLO 연동): 추론 파이프라인에서 좌석별 상태를 계산해 이 캐시를 갱신하거나,
#   이 엔드포인트 자체를 추론 서버 프록시로 교체
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
    """데모 트리거나 실제 YOLO 파이프라인이 개별 좌석 상태를 갱신할 때 사용."""
    data = request.get_json(force=True)
    seats = get_seats()
    for s in seats:
        if s["no"] == no:
            s["state"] = data.get("state", s["state"])
    return jsonify({"ok": True})


# ============================================================
# API: 로그 조회 (실제 SQLite)
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
# API: 이벤트 트리거 → DB 적재 (데모 버튼이 호출)
# 실제 YOLO 감지가 붙으면 추론 코드에서 이 함수를 그대로 재사용하면 됨
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
# 웹캠 스트리밍 (MJPEG)
# 지금은 PC 웹캠(index 0)을 그대로 스트리밍합니다.
# TODO(터틀봇 연동): cv2.VideoCapture(0) 자리를
#   - 라즈베리파이가 image_transport로 올리는 RTSP/HTTP 스트림 URL
#   - 또는 ROS2 이미지 토픽을 구독해 프레임을 넘겨주는 별도 브릿지
#   로 교체하면 됨. 이 함수의 나머지 로직(JPEG 인코딩 후 스트리밍)은 그대로 재사용 가능.
# ============================================================
def gen_frames():
    # index 0은 이 PC에서 Intel RealSense가 선점하고 있어서 실패함.
    # 실제 USB 웹캠은 index 1에 잡힘 (장치 재연결/재부팅 시 순서 바뀔 수 있음).
    cam = cv2.VideoCapture(1)
    # isOpened()만으로는 Windows에서 카메라 없을 때도 True가 나올 수 있어
    # 실제로 첫 프레임을 읽어봐서 진짜 사용 가능한지 확인한다.
    camera_ok = cam.isOpened() and cam.read()[0]

    if not camera_ok:
        cam.release()
        blank = 128 * np.ones((480, 640, 3), dtype="uint8")
        cv2.putText(
            blank, "NO CAMERA CONNECTED", (60, 240),
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2,
        )
        ok, buf = cv2.imencode(".jpg", blank)
        frame = buf.tobytes()
        while True:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(1)

    try:
        while True:
            ok, frame = cam.read()
            if not ok:
                break

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
    finally:
        cam.release()


@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    init_db()
    app.run(host="0.0.0.0", port=5000, debug=True)
