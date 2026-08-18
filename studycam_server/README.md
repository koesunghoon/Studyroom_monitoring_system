
## 실행 방법

```
# wsl 환경
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install flask opencv-python-headless numpy ultralytics
```
브라우저에서 `http://localhost:5000` 접속.


cd ~/Studyroom_monitoring_system/studycam_server
source /opt/ros/humble/setup.bash    # ROS2 환경 먼저
source venv/bin/activate              # 그다음 venv
python app.py
