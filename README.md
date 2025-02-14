[Term Project 1]
Map: R_KR_PG_C-track_outdoor   
Model: 2023_Hyundai_Ioniq5   
센서 세팅 조건(장착 위치 및 개수, 노이즈 수준 등) 변경 불가
날씨 및 시간: Sunny/13:00
평가기준: Waypoint와 차량 gps 위치의 RMS 오차(m) * 주행 기록(초)의 값을 최소화

Vehicle Specification
Minimum turning radius(m): 5.97
Max. wheel steer angle(deg): 40
Length(m): 4.635
Width(m): 1.892
Height(m): 2.434
Wheelbase(m): 3
Front Overhang(m): 0.845
Rear Overhang(m): 0.7

Simulator Network
1. Ego Ctrl Cmd 
  Network Settings > Ego-0 > Cmd Control
  /Ctrl_cmd 제어시, longi type 1번 (accel, brake) 제어를 사용해야 함
2. Event Cmd Srv
  차량 기어 변경
  Network Settings > Ego-0 > Publisher, Subscriber, Service
3. CollisionData
  충돌 여부 확인
  Network Settings > Ego-0 > Publisher, Subscriber, Service
4. Competition Vehicle Status
  주행 기록계 : 현재 종방향 속도,  wheel_angle, Accel, Brake 값을 제공
  Network Settings > Ego-0 > Publisher, Subscriber, Service

[Term Project2]
To be determined
