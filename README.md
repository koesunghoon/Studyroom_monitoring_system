# Studyroom_monitoring_system
**스터디카페 무인 순찰 로봇 — TurtleBot3 기반 실시간 모니터링 & 출결 관리 시스템**

TurtleBot3가 스터디카페(1인실 7개)를 자율주행으로 순찰하면서, 카메라로 좌석 상황(쓰러짐/분실물)을 감지하고, 지문인식으로 출결을 자동 관리하는 통합 시스템입니다.

---

## 주요 기능

### 🗺️ 실시간 SLAM 지도
- ROS2 Nav2/Cartographer 기반 SLAM 맵을 웹 대시보드에 실시간 시각화
- 로봇 현재 위치·이동 궤적을 지도 위에 실시간 표시

### 📹 실시간 카메라 스트리밍 + 객체 인식
- Intel RealSense D435(컬러+뎁스) 스트림을 웹에서 실시간 확인
- YOLOv8 기반 객체 인식(착석/쓰러짐)을 프레임에 오버레이
- 뎁스 카메라로 거리 계산 → 로봇 위치(tf)와 조합해 정확한 좌석(방) 번호 판정

### 🚨 위급상황 자동 감지
- 쓰러짐이 일정 시간(연속 감지) 이상 지속되면 자동으로 경고 발생
- 감지 즉시 DB 기록, 해당 좌석 UI 변경, 실시간 알림 팝업
- "조치 완료" 처리 시 자동으로 퇴실 처리(출결 기록 포함)

### 🎒 분실물 감지 
- 좌석이 공석인데 물건(가방)만 감지되는 경우를 자동으로 판별해 경고

### 🖐️ 지문인식 출결 관리
- STM32 + AS608 지문 센서로 학생별 입실/퇴실 자동 기록
- 같은 학생이 재인식하면 입실 ↔ 퇴실 자동 토글
- 좌석 현황 UI가 실제 출결 데이터를 그대로 반영

### 🚶 순찰 관리
- TurtleBot이 전체 웨이포인트를 한 바퀴 돌 때마다 순찰 횟수 자동 기록
- 순찰 시작/종료 실시간 이벤트 알림

---

## 시스템 아키텍처

```
┌─────────────────────────┐
│   라즈베리파이 (pi02)     │
│  ─────────────────────  │
│  RealSense D435          │
│  - 컬러/뎁스 MJPEG 스트리밍 │
│  (camera_stream.py)      │
└───────────┬──────────────┘
            │ HTTP (MJPEG)
            ▼
┌──────────────────────────────────────────────┐
│           PC (WSL2, Ubuntu 22.04)              │
│  ┌────────────────────────────────────────┐  │
│  │  Flask 서버 (app.py)                     │  │
│  │  ─────────────────────────────────────  │  │
│  │  · YOLO 추론 (sit/fallen)                │  │
│  │  · 거리 계산 + 방 번호 판정                │  │
│  │  · rclpy 백그라운드 노드                  │  │
│  │    (/map, /battery_state, tf 구독)       │  │
│  │  · SQLite DB (출결/순찰/경고/분실물)       │  │
│  │  · 지문 출결 TCP 서버 (5001)              │  │
│  │  · 순찰 시작/종료 API                     │  │
│  │  · 웹 대시보드 (실시간 폴링)               │  │
│  └────────────────────────────────────────┘  │
└───────────┬────────────────────┬──────────────┘
            │ ROS2 (DDS)          │ TCP (5001)
            ▼                     ▼
┌─────────────────────┐   ┌──────────────────────┐
│  TurtleBot3 + Nav2    │   │  STM32 NUCLEO-F411RE  │
│  ───────────────────  │   │  ────────────────────  │
│  · RPLIDAR C1 SLAM    │   │  · AS608 지문 센서      │
│  · Cartographer/Nav2  │   │  · RC522 RFID(관리자)   │
│  · 웨이포인트 순찰      │   │  · ESP-01 WiFi 모듈     │
│  (patrol8.py)         │   │  · CLI(ST-Link VCP)     │
└─────────────────────┘   └──────────────────────┘
```

### 핵심 설계 원칙

- **하드웨어 I/O는 파이/STM32, 지능적 처리(YOLO, DB, 로직)는 PC** — 역할을 명확히 분리
- **DDS(ROS2) vs HTTP vs TCP** — 각 통신 방식의 특성에 맞게 선택
  - SLAM/배터리: ROS2 DDS 자동 발견 (IP 하드코딩 불필요)
  - 카메라 스트리밍: HTTP MJPEG (직접 파싱)
  - 지문 출결: 순수 TCP 소켓 (ESP01 AT 커맨드 특성상 HTTP 불가)
  - 순찰 이벤트: 일반 HTTP POST

---

## 기술 스택

| 분류 | 기술 |
|---|---|
| 로봇 | TurtleBot3 Burger, RPLIDAR C1, Intel RealSense D435 |
| 자율주행 | ROS2 Humble, Nav2, Cartographer |
| 임베디드 | STM32 NUCLEO-F411RE (HAL), AS608, MFRC522, ESP-01 |
| AI | YOLOv8 (Ultralytics), pyrealsense2 |
| 백엔드 | Python, Flask, SQLite, rclpy |
| 프론트엔드 | HTML/CSS/JavaScript (Vanilla) |
| 개발 환경 | WSL2 (Ubuntu 22.04) + Windows |

---

## 폴더 구조

```
Studyroom_monitoring_system/
├── studycam_server/
│   ├── app.py                 # Flask 메인 서버
│   ├── best.pt                 # YOLO 학습 모델
│   ├── requirements.txt
│   ├── templates/
│   │   └── index.html
│   └── static/
│       ├── css/style.css
│       └── js/dashboard.js
├── STM32_Studyroom/            # STM32 펌웨어 (CubeIDE 프로젝트)
│   ├── Core/Src/
│   │   ├── main.c
│   │   ├── cli.c               # 시리얼 CLI (지문 등록/인식 등)
│   │   ├── rfid.c              # RC522 RFID 드라이버
│   │   ├── esp01.c             # ESP-01 WiFi 드라이버
│   │   └── fingerprint.c       # AS608 지문 센서 드라이버
│   └── ...
├── patrol8.py                  # Nav2 웨이포인트 순찰 스크립트
└── camera_stream.py             # 파이 쪽 카메라 스트리밍 서버
```

---

## 실행 방법

### 1. 사전 준비
- WSL2(Ubuntu 22.04) + ROS2 Humble 설치
- `.wslconfig`에 미러 네트워킹 모드 설정
- Windows 방화벽에 필요한 포트 인바운드 규칙 추가 (ROS2 DDS UDP, Flask 5000, 지문 TCP 5001)

### 2. 저장소 클론 및 환경 설정
```bash
git clone https://github.com/koesunghoon/Studyroom_monitoring_system.git
cd Studyroom_monitoring_system/studycam_server

python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt
```

### 3. 터틀봇 쪽 실행
```bash
# 파이: bringup
ros2 launch <bringup launch 파일>

# 파이: 카메라 스트리밍
python3 camera_stream.py

# Nav2/AMCL 실행 후 순찰 시작
python3 patrol8.py
```

### 4. 웹 서버 실행
```bash
cd studycam_server
source venv/bin/activate
python app.py
```
브라우저에서 `http://localhost:5000` 접속

---

## 팀 구성

| 담당 | 역할 |
|---|---|
| _(고성훈)_ |프로젝트 총괄|
| _(고경민, 김재민, 고성훈)_ | TurtleBot3 SLAM/Nav2, 자율주행 순찰 로직, RealSense 카메라 스트리밍 |
| _(고성훈)_ | STM32 임베디드 (지문인식 AS608, RFID 관리자 인증, WiFi 통신) |
| _(김준혁, 고지훈)_ |YOLO 연동|
| _(김준혁)_ | 웹 대시보드, Flask서버, DB설계 및 연동 |
---
