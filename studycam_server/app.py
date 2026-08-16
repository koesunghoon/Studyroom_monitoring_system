"""
STUDYCAM 관제 서버
------------------
로컬 PC에서 여는 Flask 서버.
파이(터틀봇)가 8000번 포트로 올리는 원본 웹캠 스트림을 받아서,
PC에서 YOLO 추론(sit/fallen) 후 박스를 그려 /video_feed 로 다시 내보냅니다.

지연(딜레이) 방지: 수신 스레드가 항상 최신 프레임으로 덮어쓰고,
스트리밍 쪽은 그 순간의 최신 프레임만 꺼내 쓰기 때문에 처리(YOLO)가 느려도
지연이 계속 쌓이지 않습니다 (LatestFrameReader).
"""

import os
import time
import random
import sqlite3
import threading
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
# 파이(터틀봇) 웹캠 스트림 주소
# ============================================================
PI_STREAM_URL = "http://192.168.0.4:8000/video_feed"   # 실제 파이 IP:포트로 교체

# ============================================================
# YOLO 모델 (sit / fallen 2클래스)
# ============================================================
yolo_model = YOLO(MODEL_PATH)
CLASS_NAMES = yolo_model.names
print("모델 클래스 확인:", CLASS_NAMES)   # 실행 시 콘솔에서 실제 라벨명 꼭 확인

ALERT_CLASSES = {"fallen"}
CONF_THRESHOLD = 0.5
ALERT_COOLDOWN_SEC = 10
BOX_COLORS = {
    "sit": (0, 200, 0),        # 초록 (BGR)
    "fallen": (0, 0, 255),     # 빨강
}
_last_alert_ts = {}


# ============================================================
# 파이 스트림에서 "항상 최신 프레임만" 읽어오는 백그라운드 리더
# 처리(YOLO) 속도가 수신 속도보다 느려도 지연이 누적되지 않도록 함
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
                self.frame = frame   # 최신 프레임으로 계속 덮어씀 -> 밀린 프레임은 자동 폐기

    def get_frame(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self.running = False
        self.cap.release()


frame_reader = LatestFrameReader(PI_STREAM_URL)   # 앱 시작 시 한 번만 생성 (연결 재사용)


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
# API: 로봇 상태 / KPI
# TODO(터틀봇 연동): rosbridge_suite(websocket) 또는 별도 브릿지 스크립트가
#   /api/robot_state 로 POST 해주는 실제 값으로 교체
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
# TODO(YOLO 연동): 추론 파이프라인에서 좌석별 상태를 계산해 이 캐시를 갱신
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
# 실제 YOLO 감지가 붙으면 insert_warning()을 그대로 재사용하면 됨
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
# 웹캠 스트리밍 (MJPEG) — 파이 스트림(LatestFrameReader) 최신 프레임을
# YOLO 처리 후 재전송. 처리 속도가 느려도 지연이 쌓이지 않음.
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