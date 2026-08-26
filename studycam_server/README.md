
## 실행 방법

```
# Ubuntu(22.04)
sudo apt install ros-humble-desktop
python3 -m venv --system-site-packages venv   # rclpy 접근용
source venv/bin/activate
pip install -r requirements.txt
python app.py
```
브라우저에서 `http://localhost:5000` 접속.

cd ~/Studyroom_monitoring_system/studycam_server
source /opt/ros/humble/setup.bash    # ROS2 환경 먼저
source venv/bin/activate              # 그다음 venv
python app.py
# 웹에서 map이 뜨지 않을때
1. 방화벽 규칙 재확인 (Windows PowerShell, 관리자 권한) — 안 되면 이것부터
powershell
Get-NetFirewallRule -DisplayName "ROS2 DDS*", "Flask Studycam*", "Allow ICMP*" | Select DisplayName, Enabled

규칙이 없거나 Enabled: False면:

powershell
New-NetFirewallRule -DisplayName "Allow ICMP In" -Protocol ICMPv4 -IcmpType 8 -Direction Inbound -Action Allow
New-NetFirewallRule -DisplayName "ROS2 DDS UDP All In" -Direction Inbound -Protocol UDP -Action Allow
New-NetFirewallRule -DisplayName "ROS2 DDS UDP All Out" -Direction Outbound -Protocol UDP -Action Allow
New-NetFirewallRule -DisplayName "Flask Studycam TCP" -Direction Inbound -Protocol TCP -LocalPort 5000 -Action Allow
3. 터미널에서 환경 확인
bash
echo $ROS_DOMAIN_ID          # 31 나와야 함
echo $RMW_IMPLEMENTATION     # 비어있어야 함
ip a                          # 192.168.0.x 대역 확인
4. 파이 SSH 접속 후 bringup + Nav2 실행
bash
ros2 launch ~/ros2_ws/src/tb3_c1_bringup/launch/robot_c1.launch.py

(새 터미널)

bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map5.yaml
5. 파이 ↔ ubuntu 통신 확인
bash
# WSL에서
ping <파이IP>
bash
# 파이에서
ping <WSL IP>

둘 다 안 되면 → 2번(방화벽)부터 다시.

6. 데몬 정리 (필요시)
bash
pkill -9 -f _ros2_daemon
rm -rf /tmp/ros2cli_daemon_*
ros2 daemon start
7. 저장소로 이동, 서버 실행
bash
cd ~/Studyroom_monitoring_system/studycam_server
source venv/bin/activate
python app.py
8. 최종 확인
bash
curl http://localhost:5000/api/map
curl http://localhost:5000/api/status

브라우저:

http://localhost:5000
