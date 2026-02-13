🏗️ Smart Factory System
스마트 팩토리 시스템 구축 및 아키텍처 보고서

본 프로젝트는 로봇 · 컨베이어 · 서버 · 모바일 앱을 하나의 흐름으로 연결한 스마트 팩토리 시스템이다.
IoT 플랫폼, 백엔드, 프론트엔드, 모바일 앱을 단계적으로 구축하여 확장성과 유지보수성이 높은 구조를 목표로 설계했다.

1️⃣ 🌐 IoT 플랫폼 구축 (Mobius 4)

“모든 장치와 데이터가 만나는 중앙 정거장”

국제 표준 oneM2M 기반 오픈소스 IoT 플랫폼 Mobius 4를 도입하여
로봇과 서버 간 통신을 표준화하였다.

🔧 구축 환경

Main Server: Raspberry Pi

IP: 192.168.0.5

Port: 7579 (HTTP)

📌 구축 과정
1. Node.js 기반 설치

Mobius 구동을 위한 런타임 환경 구성

2. 리소스 트리(Resource Tree) 설계

데이터를 구조적으로 관리하기 위한 oneM2M 리소스 설계

AE (Application Entity)

Robot_Final : 로봇이라는 주체 등록

CNT (Container)

/status : 로봇 위치(RFID), 주행 상태

/command : 정지 / 재개 명령

/battery : 배터리 잔량

/conveyor/log : 컨베이어 분류 결과

3. 데이터 처리 방식

CIN (Content Instance)

실제 데이터가 누적 저장되는 단위

la (latest) 조회를 통해 실시간 상태 확인

💡 핵심 포인트
장치끼리 직접 연결하지 않고 Mobius를 중간에 두는 구조로 설계하여
로봇이나 앱이 변경되어도 서로의 코드를 수정할 필요 없는
👉 느슨한 결합(Decoupling) 구조를 완성했다.

2️⃣ 🧠 백엔드 시스템 구축 (Backend)

“데이터를 처리하고 판단하는 두뇌”

Python 기반 Flask 프레임워크를 사용하여
웹 서버 + AI 연산 기능을 하나로 통합하였다.

🔑 핵심 파일

conveyor_main.py

(robot_main.py와 연동)

📌 구축 과정
1. 웹 서버 호스팅 (Flask)
app.run(host='0.0.0.0', port=8001)


외부 접근 가능한 웹 서버 개방

render_template()로 HTML 대시보드 제공

2. AI 추론 엔진 탑재 (YOLOv8)

실시간 카메라 영상 입력

객체(캔, 배터리 등) 인식

최적화: 5프레임당 1회 추론 → 실시간성 확보

3. 데이터 영속성 관리 (Data Persistence)

실시간 데이터를 production_data.json 파일에 저장

서버 재시작 후에도 당일 생산량 유지

4. API 엔드포인트 구축

/api/stats

/api/download_log
→ 프론트엔드(JS)에서 데이터 요청 가능

3️⃣ 🖥️ 프론트엔드 & 모바일 앱 구축

“사용자와 시스템이 소통하는 얼굴”

웹 대시보드를 기반으로
**안드로이드 앱(WebView)**으로 감싸는 하이브리드 구조 채택

A. 🌐 웹 대시보드 (HTML / JS / CSS)
1. UI 디자인

Dark Mode

Industrial UI (현장 가독성 최적화)

2. 실시간 통신

setInterval() 기반 Polling (1초 주기)

새로고침 없이 실시간 데이터 반영

3. 시각화

Chart.js: 시간대별 생산량 그래프

SVG Map: 공장 트랙 & 로봇 위치 좌표 표시

B. 📱 안드로이드 앱 (Java / WebView)
1. WebView 컨테이너

Flask 서버 로드
http://192.168.0.5:8001

2. 브릿지(Bridge) 구축

JavascriptInterface

웹 → 네이티브(Java) 데이터 전달

3. 네이티브 기능 확장

전달받은 데이터를 기반으로
AI 요약 한줄평 UI 표시

📊 전체 데이터 흐름 요약 (Data Flow)

감지 (Sensor)
로봇 / 컨베이어가 객체 인식 또는 RFID 감지

전송 (Publish)
Mobius 리소스(/status, /command 등)에 데이터 업로드

처리 (Process)
Flask 백엔드에서 데이터 집계 및 JSON 저장

시각화 (Subscribe / Poll)
웹 대시보드가 Mobius & Backend에서 최신 데이터 조회

관제 (Control)
앱에서 버튼 입력 → Mobius → 로봇 제어
