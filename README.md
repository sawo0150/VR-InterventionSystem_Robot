# **VR Intervention Delivery Robot (ROS 2 Project)**

rf2o_laser_odometry
robot-localization

**VR 개입 기반 배달 로봇 시스템**의 ROS 2 워크스페이스입니다.

이 프로젝트는 \*\*자율주행(Nav2)\*\*을 기본으로 하되, 난관 봉착 시 **VR 원격 제어**로 부드럽게 전환되는 하이브리드 제어 시스템을 구현합니다.

## **📂 Repository Structure & Development Guide**

이 리포지토리는 **확장성**과 **모듈화**를 위해 기능별로 패키지가 분리되어 있습니다.

각 패키지별 구현 목표와 생성해야 할 주요 파일은 다음과 같습니다.

### **1\. 🧱 Interfaces & Description (기반)**

#### **vris\_interfaces**

**역할:** 프로젝트 전용 메시지/서비스/액션 정의 (의존성 순환 방지)

* \[ \] msg/InterventionState.msg: 현재 제어 모드(Auto/VR), VR 연결 상태 등 정의  
* \[ \] msg/VRControl.msg: VR 컨트롤러 입력 데이터 (조이스틱, 버튼)  
* \[ \] action/DeliveryMission.action: 배달 미션 목표 및 피드백 정의

#### **vris\_robot\_description**

**역할:** 로봇의 물리적 형상 및 TF 트리 정의

* \[ \] urdf/robot.urdf.xacro: 로봇 베이스, Lidar 위치, IMU 위치 정의 (정확한 TF 필수)  
* \[ \] launch/display.launch.py: RViz에서 모델 확인용  
* \[ \] **Tip:** Lidar와 IMU의 xyz, rpy 값을 실제 하드웨어 장착 위치와 정확히 일치시킬 것.

### **2\. ⚙️ Hardware Abstraction (하드웨어)**

#### **vris\_sensors**

**역할:** Lidar, IMU 등 센서 드라이버 실행 및 파라미터 설정

* \[ \] launch/rplidar.launch.py: RPLidar A3 실행 (vendor 패키지 include)  
* \[ \] launch/imu.launch.py: HFI-A9 IMU 실행 및 데이터 필터링  
* \[ \] launch/sensors\_all.launch.py: 모든 센서 \+ robot\_localization(EKF) 노드 실행

#### **vris\_base\_hw**

**역할:** 아두이노 모터 제어 및 Odometry 계산

* \[ \] src/base\_serial\_node.cpp: 아두이노와 시리얼 통신. /cmd\_vel 구독 \-\> PWM 변환, 엔코더 수신 \-\> /odom 발행.  
* \[ \] config/motor\_params.yaml: 바퀴 반지름, 휠 간 거리(Track width) 설정.

#### **vris\_teleop**

**역할:** 조이스틱/Xbox 패드를 통한 수동 제어 (매핑 및 비상용)

* \[ \] config/xbox\_mapping.yaml: Xbox 컨트롤러 버튼 매핑  
* \[ \] launch/teleop\_joy.launch.py: teleop\_twist\_joy 노드 실행

### **3\. 🔀 Control & Mode Management (핵심 로직)**

#### **vris\_vr\_bridge**

**역할:** WebRTC 서버(외부)와 ROS 2 내부 통신 연결

* \[ \] src/vr\_subscriber.cpp: WebRTC에서 받은 JSON/String을 geometry\_msgs/Twist로 변환하여 **/cmd\_vel\_vr** 토픽 발행.  
* \[ \] src/robot\_status\_publisher.cpp: 배터리, 위치, 카메라 상태 등을 WebRTC로 전송.

#### **vris\_mode\_manager**

**역할:** 자율주행 신호와 VR 신호 중재 (TwistMux)

* \[ \] config/twist\_mux.yaml:  
  * 우선순위 1: Lock (E-Stop)  
  * 우선순위 2: **/cmd\_vel\_vr** (VR 개입)  
  * 우선순위 3: /cmd\_vel\_joy (Xbox)  
  * 우선순위 4: **/cmd\_vel\_auto** (Nav2)  
* \[ \] src/mode\_switch\_node.cpp: 개입 요청 시 Mux의 Lock을 풀거나 우선순위 조정 트리거 관리.

### **4\. 🗺️ Navigation & AI (자율주행)**

#### **vris\_slam**

**역할:** 지도 작성 (SLAM Toolbox)

* \[ \] config/mapper\_params\_online\_async.yaml: SLAM Toolbox 파라미터  
* \[ \] launch/mapping.launch.py: vris\_bringup/robot\_minimal \+ SLAM 노드 \+ RViz 실행  
* \[ \] **Goal:** Xbox 패드로 로봇을 운전하며 정밀한 지도(.pgm, .yaml) 생성 및 저장.

#### **vris\_navigation**

**역할:** Nav2 스택 설정

* \[ \] config/nav2\_params.yaml: Costmap(Inflation, Obstacle layer), Planner(Smac/Theta\*), Controller(DWB/MPPI) 설정.  
* \[ \] config/keepout\_zones.yaml: 자율주행 금지구역 설정 (가상 벽).  
* \[ \] launch/navigation.launch.py: 맵 로드 \+ AMCL \+ Nav2 Lifecycle Manager 실행.

#### **vris\_bt\_trees & vris\_bt\_plugins**

**역할:** 배달 시나리오 및 VR 개입 판단 로직 (Behavior Tree)

* \[ \] vris\_bt\_plugins/src/:  
  * Condition: IsInterventionNeeded: 로봇이 곤란한 상황인지 판단.  
  * Action: RequestVRControl: VR 사용자에게 알림 전송.  
* \[ \] vris\_bt\_trees/trees/delivery\_w\_intervention.xml:  
  * 메인: \[배달 지점 이동\] \-\> (실패/조건 만족 시) \-\> \[VR 개입 요청\] \-\> \[VR 조종 대기\] \-\> \[자율주행 재개\]

### **5\. 🎬 Scenarios & Bringup (통합 실행)**

#### **vris\_scenarios**

**역할:** 실험 및 데모용 시나리오 스크립트

* \[ \] src/random\_delivery\_node.cpp: 맵 상의 좌표 중 하나를 랜덤 선택해 Nav2 Action으로 전송.

#### **vris\_bringup**

**역할:** 전체 시스템 원클릭 실행

* \[ \] launch/robot\_minimal.launch.py: 센서 \+ 하드웨어 \+ Teleop (기본 구동)  
* \[ \] launch/robot\_mapping.launch.py: 기본 구동 \+ SLAM  
* \[ \] launch/robot\_system\_integration.launch.py: 기본 구동 \+ Nav2 \+ VR Bridge \+ BT \+ Scenario

## **🚀 Development Workflow (Step-by-Step)**

아래 순서대로 개발을 진행하며 체크박스를 채워나가세요.

### **Phase 1: Hardware & Low-level Control**

* \[ \] **센서 확인:** vris\_sensors 런치 파일로 Lidar, IMU 토픽(/scan, /imu)이 정상적으로 나오는지 확인.  
* \[ \] **모터 제어:** vris\_base\_hw 구현 후 아두이노 연결. /cmd\_vel 발행 시 바퀴가 도는지 확인.  
* \[ \] **TF 트리 확립:** vris\_robot\_description 설정 후 ros2 run tf2\_tools view\_frames로 트리 끊김 없는지 확인.  
* \[ \] **수동 주행:** vris\_teleop \+ Xbox 패드로 로봇을 부드럽게 운전할 수 있어야 함.

### **Phase 2: Mapping (SLAM)**

* \[ \] **지도 작성:** vris\_slam 실행. Xbox 패드로 실험 공간 전체를 돌아다니며 지도 완성.  
* \[ \] **지도 저장:** ros2 run nav2\_map\_server map\_saver\_cli \-f my\_map 으로 저장.

### **Phase 3: Navigation (Basic)**

* \[ \] **자율 주행:** 저장된 지도를 띄우고 vris\_navigation 실행.  
* \[ \] **RViz 테스트:** 2D Goal Pose로 목표를 찍었을 때 장애물을 피해 이동하는지 확인.

### **Phase 4: VR Intervention System Integration**

* \[ \] **Bridge 연결:** WebRTC 서버를 켜고 vris\_vr\_bridge 실행. VR 컨트롤러 입력이 /cmd\_vel\_vr로 들어오는지 확인.  
* \[ \] **Twist Mux:** Nav2가 켜진 상태에서 VR 입력을 주었을 때, Nav2 명령이 무시되고 VR 명령이 우선시되는지 확인 (twist\_mux 설정 검증).  
* \[ \] **BT 시나리오:** 랜덤 배달 중 강제로 길을 막았을 때, 로봇이 멈추고 VR 개입을 요청하는지 테스트.

## **📝 Notes**

* **Topic Naming Convention:**  
  * Auto Command: /cmd\_vel\_auto  
  * VR Command: /cmd\_vel\_vr  
  * Joy Command: /cmd\_vel\_joy  
  * Final Output: /cmd\_vel  
* **Coordinate Frames:**  
  * map \-\> odom \-\> base\_link \-\> laser, imu\_link