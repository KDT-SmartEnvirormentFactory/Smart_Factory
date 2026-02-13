# Smart_Factory_KDT_4_1팀

##시스템개요
<img width="1024" height="565" alt="image" src="https://github.com/user-attachments/assets/b2083e32-99e5-4b3a-b97c-102ea72f700b" />

##🏭 AIoT-based Autonomous Resource Circulation System
OneM2M 표준 플랫폼(Mobius 4.0) 기반의 자율 주행 수거 및 스마트 분류 시스템 > From Automation to Autonomy: Smart Factory Logistics Solution

##📖 프로젝트 개요 (Overview)
본 프로젝트는 제조 및 건설 현장의 자원 수거/분류 과정에서 발생하는 인력 의존성과 안전 사고 문제를 해결하기 위해 개발되었습니다.

단순한 하드웨어 제어를 넘어, **국제 표준 IoT 플랫폼(Mobius)**을 활용한 Hub & Spoke 아키텍처를 구축하여 이기종 기기(Robot, Conveyor, Sensor) 간의 M2M 연동과 **실시간 관제(Digital Twin)**를 구현했습니다.

##🏗️ 시스템 아키텍처 (System Architecture)
전체 시스템은 중앙의 Mobius 4.0 서버를 허브(Hub)로 하여 모든 엣지 디바이스가 연결된 구조입니다.

Core Server (Hub): Node.js 기반의 Mobius 4.0 플랫폼과 PostgreSQL 17 데이터베이스를 사용하여 대용량 센서 데이터를 안정적으로 처리.

Edge Device (Spoke):

AGV Robot: Raspberry Pi 5 기반, Line Tracing 및 RFID 정밀 위치 제어.

Smart Sorter: ESP32/Arduino 기반, 셔틀(Feeder) 및 컨베이어 제어.

Protocol:

MQTT: 비상 정지(STOP), 셔틀 트리거 등 초저지연(Low-Latency) 제어.

HTTP (REST): 데이터 로깅, 상태 조회 및 웹/앱 대시보드 시각화.

UART: 내부 MCU(RPi ↔ Arduino) 간 노이즈 없는 신뢰성 통신.

##⚙️ 핵심 기술 (Key Technologies)
🔹 1. 정밀 도킹 및 하역 (Precision Docking)
문제 (Pain Point): 이동 관성(Inertia)으로 인한 오버슈트(Overshoot) 발생 및 덤핑 위치 이탈.

해결 (Solution): RFID 태그 인식 즉시 통신 버퍼 플러시(Serial.flush) 및 인터럽트 제동 로직을 적용하여 오차 범위 ±1cm 이내 정밀 정차 구현.

🔹 2. 스마트 분류 및 전력 최적화 (Singulation & Power Mgmt)
기능: 로봇이 하역한 다량의 폐기물을 셔틀(Shuttle) 시스템이 하나씩 순차 투입(Singulation)하여 YOLO AI의 인식률을 극대화.

Troubleshooting: 고토크 모터 기동 시 발생한 전압 강하(Voltage Drop, 10.5V → 3V) 및 MCU 리셋 문제를 해결하기 위해, DC-DC 컨버터(LM2596) 적용 및 비동기 시차 제어(Asynchronous Control) 알고리즘을 도입하여 전력 안정성을 확보함.

🔹 3. 통합 관제 및 디지털 트윈 (Monitoring & Control)
구현: React/HTML 기반 웹 대시보드 및 Android Native App(Java) 구축.

특징: **Mobius 리소스 트리(OneM2M)**를 시각화하여 로봇의 실시간 위치를 맵에 표시하고, 비상 상황 시 0.1초 이내 즉각적인 제어(Emergency Stop) 지원.

##🛠️ 기술 스택 (Tech Stack)
Category,Technology
Server & DB,
Hardware / Embedded,
Connectivity,
Frontend / Mobile,
AI / Vision,
