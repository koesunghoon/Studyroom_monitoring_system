import pyrealsense2 as rs
import numpy as np
import cv2
import os
import sys
import tty
import termios
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ===== 설정 =====
SAVE_DIR = "./dataset"
IMG_W, IMG_H, FPS = 640, 480, 30
SAVE_DEPTH = False
STREAM_PORT = 8000

CLASSES = {
    '1': "standing",
    '2': "fallen",
    '3': "sitdown",
}

os.makedirs(SAVE_DIR, exist_ok=True)
for cls_name in CLASSES.values():
    os.makedirs(os.path.join(SAVE_DIR, cls_name), exist_ok=True)
    if SAVE_DEPTH:
        os.makedirs(os.path.join(SAVE_DIR, cls_name, "depth"), exist_ok=True)

# 최신 프레임을 여러 스레드(스트리밍 서버 + 캡처 로직)가 공유하기 위한 버퍼
latest_frame_lock = threading.Lock()
latest_color_img = None
latest_frames = None  # depth 저장용 원본 frameset


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


# ===== MJPEG 스트리밍 서버 =====
class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # 접속 로그로 터미널 어지럽히지 않기

    def do_GET(self):
        if self.path != "/":
            self.send_response(404)
            self.end_headers()
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        try:
            while True:
                with latest_frame_lock:
                    img = None if latest_color_img is None else latest_color_img.copy()
                if img is None:
                    time.sleep(0.05)
                    continue

                ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    continue

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                self.wfile.write(jpg.tobytes())
                self.wfile.write(b"\r\n")
                time.sleep(1 / 15)  # 스트리밍은 15fps 정도면 충분
        except (BrokenPipeError, ConnectionResetError):
            pass


def run_stream_server():
    server = ThreadingHTTPServer(("0.0.0.0", STREAM_PORT), StreamHandler)
    server.serve_forever()


# ===== 카메라 파이프라인 =====
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.bgr8, FPS)
if SAVE_DEPTH:
    config.enable_stream(rs.stream.depth, IMG_W, IMG_H, rs.format.z16, FPS)

pipeline.start(config)

for _ in range(15):  # 워밍업
    pipeline.wait_for_frames()

counts = {}
for cls_name in CLASSES.values():
    cls_dir = os.path.join(SAVE_DIR, cls_name)
    counts[cls_name] = len([f for f in os.listdir(cls_dir) if f.endswith(".jpg")])

# 프레임을 계속 갱신하는 백그라운드 스레드
stop_flag = threading.Event()


def frame_updater():
    global latest_color_img, latest_frames
    while not stop_flag.is_set():
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        img = np.asanyarray(color_frame.get_data())
        with latest_frame_lock:
            latest_color_img = img
            latest_frames = frames


updater_thread = threading.Thread(target=frame_updater, daemon=True)
updater_thread.start()

stream_thread = threading.Thread(target=run_stream_server, daemon=True)
stream_thread.start()

# 라파이 IP 안내용으로 hostname -I 결과 대신 안내 문구만 출력 (IP는 알고 있으니)
print(f"실시간 화면 보기: 브라우저에서 http://<라파이IP>:{STREAM_PORT} 접속")
print(f"준비 완료. 1:standing 2:fallen 3:sitdown / q: 종료")
print("자세 취한 뒤 해당 키를 누르면 바로 촬영됩니다 (엔터 불필요).")
print(f"현재까지 저장된 장수: {counts}")

try:
    while True:
        key = get_key()

        if key == 'q':
            break
        if key not in CLASSES:
            continue

        with latest_frame_lock:
            img = None if latest_color_img is None else latest_color_img.copy()
            frames_snapshot = latest_frames

        if img is None:
            print("아직 프레임 준비 중입니다. 잠시 후 다시 시도하세요.")
            continue

        cls_name = CLASSES[key]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"{cls_name}_{counts[cls_name]:05d}_{timestamp}.jpg"
        cv2.imwrite(os.path.join(SAVE_DIR, cls_name, filename), img)
        print(f"[{cls_name}] #{counts[cls_name]} 저장됨: {filename}")

        if SAVE_DEPTH and frames_snapshot is not None:
            depth_frame = frames_snapshot.get_depth_frame()
            if depth_frame:
                depth_img = np.asanyarray(depth_frame.get_data())
                np.save(os.path.join(SAVE_DIR, cls_name, "depth", f"depth_{counts[cls_name]:05d}.npy"), depth_img)

        counts[cls_name] += 1

finally:
    stop_flag.set()
    pipeline.stop()
    total = sum(counts.values())
    summary = ", ".join([f"{k}: {v}장" for k, v in counts.items()])
    print(f"\n총 {total}장 저장 완료 ({summary}) -> {SAVE_DIR}")
