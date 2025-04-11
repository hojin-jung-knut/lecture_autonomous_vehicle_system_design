### Term Project 1
Map: R_KR_PG_C-track_outdoor(tutorials/path/C-track_outdoor.txt)
Model: 2023_Hyundai_Ioniq5
센서 세팅 조건(장착 위치 및 개수, 노이즈 수준 등) 변경 불가
날씨 및 시간: Sunny/13:00
pure_pursuit_test.launch 파일 사용(tutorials/launch/pure_pursuit_test.launch)
출발/도착 시간: 출발 waypoint x,y 좌표로부터 1m 벗어난 시간과 도착 waypoint x,y 좌표 1m 이내로 들어온 시간을 계산   
평가기준: Waypoint 좌표와 차량 gps 위치 좌표의 RMS 오차(m) * 주행 기록(초)의 값을 최소화
평가일시: 중간고사 이후 첫번째 주

#### Vehicle Specification
Minimum turning radius(m): 5.97   
Max. wheel steer angle(deg): 40   
Length(m): 4.635   
Width(m): 1.892   
Height(m): 2.434   
Wheelbase(m): 3   
Front Overhang(m): 0.845   
Rear Overhang(m): 0.7   

#### Simulator Network
- Ego Ctrl Cmd    
  Network Settings > Ego-0 > Cmd Control   
  /Ctrl_cmd 제어시, longi type 1번 (accel, brake) 제어를 사용해야 함   
- Event Cmd Srv   
  차량 기어 변경   
  Network Settings > Ego-0 > Publisher, Subscriber, Service   
- CollisionData   
  충돌 여부 확인   
  Network Settings > Ego-0 > Publisher, Subscriber, Service   
- Competition Vehicle Status   
  주행 기록계 : 현재 종방향 속도,  wheel_angle, Accel, Brake 값을 제공   
  Network Settings > Ego-0 > Publisher, Subscriber, Service   

### Term Project2
To be determined
