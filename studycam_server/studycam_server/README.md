# STUDYCAM 관제 서버 (로컬 PC용 Flask)

## 실행 방법

```bash
cd studycam_server
python -m venv venv
venv\Scripts\activate        # Windows
# source venv/bin/activate   # macOS/Linux
Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
pip install -r requirements.txt
python app.py
```

브라우저에서 `http://localhost:5000` 접속.

첫 실행 시 `studycam.db`(SQLite)가 자동 생성되고 샘플 로그가 몇 줄 채워집니다.

## 지금 실제로 동작하는 것

- **웹캠 스트리밍** (`/video_feed`): PC에 연결된 웹캠(index 0)을 MJPEG로 실시간 스트리밍합니다.
  카메라가 없으면 "NO CAMERA CONNECTED" 안내 프레임이 대신 나옵니다.
- **출결 관리 / 순찰 로그 / 경고 이력**: SQLite(`studycam.db`)에 실제로 저장·조회됩니다.
  우측 하단 데모 버튼(쓰러짐! / 장시간 이석 / 낯선 사람)을 누르면 실제로 DB에 행이 추가되고,
  화면이 새로고침 없이 갱신됩니다.

## 아직 목업인 것 (연동 전)

- **SLAM 맵**: `static/js/dashboard.js`의 `drawMap()`이 가짜 경로를 그립니다.
- **좌석 현황**: `app.py`의 `get_seats()`가 랜덤 값을 캐싱해서 내려줍니다.
- **카메라 바운딩 박스**: YOLO 추론이 없어서 실제 영상 위에 겹치는 건 아직 없고,
  데모 버튼을 누를 때만 위치를 하드코딩해서 잠깐 보여줍니다.

각 파일에 `TODO`로 표시해 뒀습니다.

## 다음 단계 (터틀봇 / STM32 연동 시)

1. **카메라**: `app.py`의 `gen_frames()`에서 `cv2.VideoCapture(0)` 부분을
   터틀봇 라즈베리파이가 올리는 스트림(RTSP 주소, 또는 ROS2 이미지 토픽을
   구독해 프레임을 넘겨주는 별도 브릿지 스크립트)으로 교체하면 됩니다.
   나머지(JPEG 인코딩 → MJPEG 스트리밍) 로직은 그대로 재사용 가능합니다.

2. **로봇 상태(배터리/순찰 상태) / 좌석 / SLAM 맵**: 지금은 폴링(주기적 fetch) 구조인데,
   실시간성이 중요해지면 `Flask-SocketIO`로 바꿔서 ROS2 → Flask로 서버 푸시하는 편이 낫습니다.
   당장은 이 프로젝트 규모에서 폴링으로도 충분합니다.

3. **ROS2 브릿지**: `rosbridge_suite`(websocket)를 터틀봇 쪽에서 띄우고,
   PC의 별도 파이썬 스크립트(또는 이 Flask 앱 안의 백그라운드 스레드)가 구독해서
   `POST /api/robot_state`, `POST /api/seats/<no>` 로 이 서버에 값을 밀어넣는 구조를 추천합니다.
   (이미 두 엔드포인트를 만들어 뒀습니다.)

4. **STM32**: `pyserial`로 시리얼 포트를 읽어서, 마찬가지로 `/api/robot_state` 같은
   엔드포인트로 값을 밀어넣거나, DB에 바로 적재하는 별도 함수를 `app.py`에 추가하면 됩니다.

5. **YOLO**: 추론 결과(좌석별 상태)를 `get_seats()`가 반환하는 캐시에 그대로 덮어쓰거나,
   감지 이벤트가 발생하면 지금 데모 버튼이 호출하는 `/api/event`를 그대로 재사용해서
   POST 하면 경고 이력 DB 적재 → 화면 반영까지 코드 변경 없이 동작합니다.
