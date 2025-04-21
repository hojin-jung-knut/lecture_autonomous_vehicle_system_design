### Term Project 1
Map: R_KR_PG_C-track_outdoor(tutorials/path/C-track_outdoor.txt)   
Model: 2023_Hyundai_Ioniq5   
센서 세팅 조건(장착 위치 및 개수, 노이즈 수준 등) 변경 불가   
날씨 및 시간: Sunny/13:00   
pure_pursuit_test.launch 파일 사용(tutorials/launch/pure_pursuit_test.launch)   
출발/도착시간: 출발 waypoint x,y 좌표로부터 1m 벗어난 순간을 출발시간, 도착 waypoint x,y 좌표가 1m 이내로 들어온 순간을 도착시간으로 계산   
평가기준: 주행 기록의 값을 최소화    
  * Waypoint 좌표와 차량 gps 위치 좌표의 오차가 기준값 이상 벗어날 시 시간 패널티 부과   
  * 미완주 차량은 완주 차량보다 순위가 낮으며 출발지로부터 목적지까지 가까이 갈수록 순위가 높음
  * 텀프로젝트 발표시 인당 3번의 프로그램 실행기회 부여
  * 텀프로젝트 발표 시간에 실행 오류시 시뮬레이션 결과 별도 제출 가능(주행기록 + 설계코드). 순위는 본 발표 시간에 완주한 결과 보다 낮음.
평가방법: data_logger.py 파일로 주행기록 취득 후 정량적 평가 실시   
주요 변경 파라미터: 속도제어 pid값, 전방주시거리   
배점 및 감점   
  * 20점(완주 시 15점 + 1위 5점부터 등수대로 0.5점씩 차감). 미완주시 0점(추후 완주 코드 및 기록 제출시 완주 점수 획득 가능)   
  * 주행로그 분석 중 위치 오차가 0.75m보다 큰 지점이 1점 이상 발생한 경우 1초씩 패널티

#### Vehicle Specification(Hyundai_Ioniq5)
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
  Network Settings > Ego-0 > Cmd Control (morai_msgs/CtrlCmd)    
  /ctrl_cmd(또는 ctrl_cmd_0) 제어시, longCmdType = 1 (accel, brake) 제어를 사용해야 함   
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
