### MORAI Sim 통신 및 lidar 센서 연결
roslaunch rosbridge_server rosbridge_websocket.launch   
roslaunch velodyne_pointcloud VLP16_points.launch

### ROS 기본 실행 명령어
rosrun tutorials [python_file_name].py   
roslaunch tutorials [launch_file_name].launch

### Term Project I
Map: R_KR_PG_C-track_outdoor(tutorials/path/c-track.txt)   
Model: 2023_Hyundai_Ioniq5   
센서 세팅 조건(장착 위치 및 개수, 노이즈 수준 등) 변경 불가   
pure_pursuit_test.launch 파일 사용(tutorials/launch/pure_pursuit_test.launch)   
날씨 및 시간: Sunny/13:00   
출발/도착시간: 출발 waypoint x,y 좌표로부터 1m 벗어난 순간을 출발시간, 도착 waypoint x,y 좌표가 1m 이내로 들어온 순간을 도착시간으로 계산   
평가방법: data_logger.py 파일로 주행기록 취득 후 정량적 평가 실시   
주요 변경 파라미터: 속도제어 pid값, 전방주시거리    
평가기준: 주행 기록의 값을 최소화    
- 총점 20점(완주 시 15점 + 1위 5점부터 등수대로 0.5점씩 차감). 미완주시 0점(추후 완주 코드 및 기록 제출시 완주 점수 획득 가능)   
- 미완주 차량은 완주 차량보다 순위가 낮으며 출발지로부터 목적지까지 가까이 갈수록 순위가 높음   
- 텀프로젝트 발표시 인당 3번의 프로그램 실행기회 부여   
- 텀프로젝트 발표 시간에 실행 오류시 시뮬레이션 결과 별도 제출 가능(주행기록 + 설계코드). 순위는 본 발표 시간에 완주한 결과 보다 낮음.   

#### Vehicle Specification(Hyundai_Ioniq5)
Minimum turning radius(m): 5.97   
Max. wheel steer angle(deg): 40   
Length(m): 4.635   
Width(m): 1.892   
Height(m): 2.434   
Wheelbase(m): 3   
Front Overhang(m): 0.845   
Rear Overhang(m): 0.7   

#### 주요 Simulator Network Info.
##### Vehicle Status    
- Network Settings > Ego-0 > Publisher, Subscriber, Service  (morai_msgs/Ego_topic)    
- 주행 기록계 : 현재 종방향 속도,  wheel_angle, Accel, Brake 값을 제공    
##### Ego Ctrl Cmd    
- Network Settings > Ego-0 > Cmd Control (morai_msgs/CtrlCmd)    
- /ctrl_cmd(또는 ctrl_cmd_0) 제어시, longCmdType = 1 (accel, brake) 방식의 제어를 사용    
##### Event Cmd Srv   
- Network Settings > Ego-0 > Publisher, Subscriber, Service (morai_msgs/EventCmdSrv)    
- 차량 기어 변경(P,R,N,D) 명령어     
##### CollisionData     
- Network Settings > Ego-0 > Publisher, Subscriber, Service (morai_msgs/CollisionData)     
- 차량과 장애물 및 주변 지형/지물 간 충돌 여부 확인     

### Term Project II
Map: R_KR_PR_Sangam_NoBuildings(tutorials/path/sangam.txt)   
Scenario 파일 사용(tutorials/scenario/Sangam_Term_Project2.json)   
- C:\Users\Administrator\MoraiLauncher_Win\MoraiLauncher_Win_Data\SaveFile\Scenario\R_KR_PR_Sangam_NoBuildings 위치에 json 파일 저장   
- MORAI Sim -> Edit -> Scenario -> Load Scenario에서 저장한 json 파일 불러오기   
lattice_driving.launch 파일 사용(tutorials/launch/lattice_driving.launch)   
pure_pursuit.py or stanley.py 파일의 /local_path를 lattice_planner 파일의 /lattice_path로 대체하여 사용   

Model: Term Project1과 동일
센서 세팅 조건: Term Project1과 동일   
날씨 및 시간: Term Project1과 동일
출발/도착시간: Term Project1과 동일   
평가방법: Term Project1과 동일   

- 주행경로   
![image](https://github.com/user-attachments/assets/902ada86-1239-4bfb-bdc7-1b0f0714c2f5)      
- 차선변경   
![image](https://github.com/user-attachments/assets/2845996a-528b-4583-b887-e1df590189ae)   
제공된 미션 경로에 차선 변경이 필요한 부분이 있음
미션 경로를 최대한 벗어나지 않도록 끊긴 경로에 대한 최적화된 경로 계획을 하여 자율주행제어 완성
- 정적 장애물   
주행 경로상에 concrete barricade 배치
- 센서 고장   
GPS blackout 구간 존재: 카메라 이미지의 차선 검출 정보 or dead-reckoning 기반 제어 알고리즘 설계
