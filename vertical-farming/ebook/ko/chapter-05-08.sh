#!/bin/bash

# Chapter 05: IoT, Sensors & Automation
cat > /home/user/wia-standards/vertical-farming/ebook/ko/chapter-05.html << 'CH05EOF'
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>챕터 05 - WIA 수직농업 표준</title>
    <style>
        body {font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Apple SD Gothic Neo', 'Malgun Gothic', sans-serif;background: #0f172a;color: #f8fafc;line-height: 1.9;max-width: 900px;margin: 0 auto;padding: 40px 20px;}
        h1, h2, h3, h4 {color: #84CC16;margin-top: 2em;}h1 {border-bottom: 3px solid #84CC16;padding-bottom: 10px;}a {color: #60a5fa;text-decoration: none;}a:hover {text-decoration: underline;}
        code {background: #1e293b;padding: 2px 6px;border-radius: 4px;color: #10b981;}pre {background: #1e293b;padding: 20px;border-radius: 8px;overflow-x: auto;border-left: 4px solid #84CC16;}
        .info-box {background: #1e293b;padding: 20px;border-radius: 8px;margin: 20px 0;border-left: 4px solid #60a5fa;}
        table {width: 100%;border-collapse: collapse;margin: 20px 0;background: #1e293b;}th, td {padding: 12px;text-align: left;border-bottom: 1px solid #334155;}
        th {background: #334155;color: #84CC16;font-weight: bold;}.nav {margin: 40px 0;padding: 20px 0;border-top: 1px solid #334155;border-bottom: 1px solid #334155;display: flex;justify-content: space-between;}
        footer {margin-top: 60px;padding-top: 20px;border-top: 1px solid #334155;text-align: center;color: #94a3b8;}ul {line-height: 2.2;}li {margin: 12px 0;}
    </style>
</head>
<body>
    <div class="nav"><a href="chapter-04.html">← 챕터 4</a><a href="chapter-06.html">챕터 6 →</a></div>
    <h1>챕터 05: IoT, 센서 및 자동화</h1>
    
    <h2>5.1 IoT 개론</h2>
    <p>사물 인터넷(IoT)은 센서, 소프트웨어 및 네트워크 연결로 강화된 물리적 장치로 구성되어 데이터를 수집하고 교환할 수 있습니다. 수직농업에서 IoT는 정밀 농업을 가능하게 하여 운영자가 환경 조건, 식물 건강 및 시스템 성능을 실시간으로 모니터링하고 제어할 수 있습니다.</p>
    
    <h3>5.1.1 수직농업에서 IoT의 이점</h3>
    <ul>
        <li><strong>실시간 모니터링:</strong> 언제 어디서나 조건에 즉시 액세스</li>
        <li><strong>데이터 기반 의사 결정:</strong> 직감 대신 분석 사용</li>
        <li><strong>원격 제어:</strong> 현장에 없어도 시스템 조정</li>
        <li><strong>조기 경보:</strong> 문제가 심각해지기 전에 감지</li>
        <li><strong>자동화:</strong> 수동 개입 없이 일상적인 작업 처리</li>
        <li><strong>최적화:</strong> 최고 효율을 위한 지속적인 개선</li>
        <li><strong>확장성:</strong> 수백 또는 수천 개의 센서/장치 관리</li>
    </ul>
    
    <h3>5.1.2 IoT 시스템 아키텍처</h3>
    <p><strong>레이어 1: 센서 및 액추에이터:</strong> 물리적 장치가 데이터 수집 및 조치 수행 (온도 센서, pH 프로브, 밸브, 팬).</p>
    <p><strong>레이어 2: 연결:</strong> 센서를 시스템에 연결하는 네트워크 (Wi-Fi, 이더넷, Zigbee, LoRaWAN, 셀룰러).</p>
    <p><strong>레이어 3: 에지 컴퓨팅:</strong> 로컬 데이터 처리 (Raspberry Pi, Arduino, 산업용 게이트웨이, PLCs).</p>
    <p><strong>레이어 4: 클라우드/데이터베이스:</strong> 중앙 집중식 데이터 스토리지 및 분석 (AWS IoT, Azure IoT, Google Cloud IoT, 자체 호스팅 서버).</p>
    <p><strong>레이어 5: 애플리케이션:</strong> 사용자 인터페이스 및 제어 (웹 대시보드, 모바일 앱, SCADA 시스템).</p>
    
    <h2>5.2 센서 기술</h2>
    
    <h3>5.2.1 환경 센서</h3>
    <p><strong>온도 센서:</strong> DHT22 (±0.5°C, 저렴, 디지털); DS18B20 (±0.5°C, 방수, 1-Wire 프로토콜); BME280 (온도 + 습도 + 압력, I²C/SPI); 열전대/RTD (고정밀, 산업용).</p>
    
    <p><strong>습도 센서:</strong> DHT22/SHT3x (디지털 RH 센서); HIH-4000 (아날로그 출력); BME680 (습도 + 온도 + 압력 + VOC 가스).</p>
    
    <p><strong>CO₂ 센서:</strong> MH-Z19 (NDIR, 400-5000 ppm); SCD30 (NDIR, ±30 ppm 정확도, I²C); K30 (산업용 NDIR, 0-10,000 ppm).</p>
    
    <p><strong>광 센서:</strong> BH1750 (디지털 광도 센서, lux); Apogee SQ-500 (양자 PAR 센서, μmol/m²/s); 스펙트럼 센서 (AS7341, 파장별 강도).</p>
    
    <h3>5.2.2 용액 센서 (수경재배)</h3>
    <p><strong>pH 센서:</strong> 유리 전극 pH 프로브 (정확도 ±0.01 pH); 고체 상태 pH 센서 (낮은 유지 관리, 덜 정확); 매월 보정 필요 (pH 4, 7, 10 완충액).</p>
    
    <p><strong>EC/TDS 센서:</strong> 전도도 프로브 (EC mS/cm 측정); TDS 미터 (ppm, EC에서 변환); 온도 보상 필수.</p>
    
    <p><strong>용존 산소 (DO) 센서:</strong> 전류계 센서 (빠른 응답, 멤브레인 교체 필요); 광학 DO 센서 (낮은 유지 관리, 더 비쌈); 목표 범위: 6-10 mg/L.</p>
    
    <p><strong>수위 센서:</strong> 초음파 거리 센서 (비접촉); 플로트 스위치 (간단, 기계적); 정전용량식 센서 (비접촉, 디지털).</p>
    
    <h3>5.2.3 식물 건강 센서</h3>
    <p><strong>식물 카메라:</strong> RGB 카메라 (시각적 점검, 성장 추적); 다중 스펙트럼 카메라 (식물 건강 지수 - NDVI, GNDVI); 열 카메라 (스트레스 감지, 물 상태).</p>
    
    <p><strong>엽록소 미터:</strong> SPAD 미터 (엽록소 함량, 질소 상태); 비파괴적, 휴대용; 영양 결핍 조기 감지.</p>
    
    <p><strong>줄기 직경 센서:</strong> 덴드로미터 (성장 속도 측정); 물 스트레스 감지 (줄기 수축/팽창); 연구 및 고가 작물.</p>
    
    <h2>5.3 통신 프로토콜</h2>
    
    <h3>5.3.1 유선 프로토콜</h3>
    <p><strong>I²C (Inter-Integrated Circuit):</strong> 용도: 단거리 센서 통신 (수 미터); 장점: 간단, 2선식, 여러 장치 공유; 단점: 짧은 범위, 속도 제한; 일반적: BME280, SCD30, 많은 센서.</p>
    
    <p><strong>SPI (Serial Peripheral Interface):</strong> 용도: 빠른 데이터 전송 (ADC, 디스플레이); 장점: I²C보다 빠름, 전이중; 단점: 더 많은 와이어(최소 4개), 장거리 아님; 일반적: SD 카드, TFT 디스플레이.</p>
    
    <p><strong>Modbus RTU/TCP:</strong> 용도: 산업 자동화, PLC, SCADA; 장점: 산업 표준, 견고함; 단점: 더 복잡한 구현; 일반적: 상업용 센서, 액추에이터.</p>
    
    <p><strong>이더넷/TCP/IP:</strong> 용도: 백본 네트워크, 인터넷 연결; 장점: 빠름, 신뢰성, 장거리; 단점: 케이블 필요, 설치 복잡; 일반적: 게이트웨이, 서버, IP 카메라.</p>
    
    <h3>5.3.2 무선 프로토콜</h3>
    <p><strong>Wi-Fi (802.11):</strong> 장점: 높은 대역폭, 익숙함, 인터넷 직접 액세스; 단점: 높은 전력 소비, 제한된 범위; 최적: 게이트웨이, 카메라, 고대역폭 장치.</p>
    
    <p><strong>Zigbee/Z-Wave:</strong> 장점: 저전력, 메시 네트워크, 수백 개 장치 지원; 단점: 낮은 대역폭, 허브 필요; 최적: 센서 네트워크, 스마트 홈 통합.</p>
    
    <p><strong>LoRaWAN:</strong> 장점: 매우 긴 범위 (수 km), 낮은 전력; 단점: 매우 낮은 대역폭, 게이트웨이 필요; 최적: 대규모 농장, 원격 모니터링.</p>
    
    <p><strong>Bluetooth/BLE:</strong> 장점: 낮은 전력(BLE), 편재성, 간단; 단점: 짧은 범위 (~10-100m), 장치 제한; 최적: 모바일 앱, 근접 센서.</p>
    
    <h2>5.4 데이터 수집 및 스토리지</h2>
    
    <h3>5.4.1 데이터 수집 빈도</h3>
    <p><strong>환경 데이터:</strong> 온도/습도: 1-5분마다; CO₂: 5-10분마다; 광 강도: 10-30분마다 (변동 느림).</p>
    <p><strong>용액 데이터:</strong> pH/EC: 10-30분마다; DO: 15-60분마다; 수위: 1-5분마다.</p>
    <p><strong>식물 이미지:</strong> RGB: 매일 1-3회; 다중 스펙트럼: 주당 1-3회; 열 이미징: 필요 시/연구용.</p>
    <p><strong>에너지/시스템:</strong> 전력 소비: 1-15분마다; 펌프/팬 상태: 1-5분마다; 경보/이벤트: 즉시 (이벤트 기반).</p>
    
    <h3>5.4.2 데이터베이스 솔루션</h3>
    <p><strong>시계열 데이터베이스 (TSDB):</strong> InfluxDB: 오픈 소스, IoT에 인기; Prometheus: 메트릭 및 경보에 적합; TimescaleDB: PostgreSQL 기반, SQL 쿼리; 이유: 센서 데이터는 시계열 (타임스탬프 값).</p>
    
    <p><strong>관계형 데이터베이스 (RDBMS):</strong> PostgreSQL/MySQL: 범용, 성숙; SQLite: 경량, 로컬 에지 스토리지; 용도: 메타데이터, 사용자 계정, 작물 레시피.</p>
    
    <p><strong>NoSQL 데이터베이스:</strong> MongoDB: 문서 스토리지, 유연한 스키마; Cassandra: 분산, 고가용성; 용도: 비정형 데이터, 대규모 확장.</p>
    
    <h2>5.5 자동화 전략</h2>
    
    <h3>5.5.1 규칙 기반 자동화</h3>
    <p><strong>IF-THEN 논리:</strong> 예: 온도 > 26°C이면 냉각 팬 켜기; RH > 70%이면 제습기 활성화; pH < 5.5이면 경보 전송 + pH Up 도징.</p>
    <p><strong>구현:</strong> Node-RED (시각적 흐름 기반 프로그래밍); Home Assistant (자동화 YAML); 사용자 정의 Python/JavaScript 스크립트.</p>
    <p><strong>장점:</strong> 간단하고 이해하기 쉬움; 예측 가능한 동작; 빠른 개발. <strong>단점:</strong> 복잡한 상호 작용 처리 제한; 경험에서 학습하지 않음; 모든 시나리오에 대한 규칙 필요.</p>
    
    <h3>5.5.2 PID 제어 자동화</h3>
    <p><strong>PID (비례-적분-미분):</strong> 비례(P): 현재 오류에 반응; 적분(I): 과거 오류 축적 수정; 미분(D): 미래 오류 예측 및 오버슈트 방지.</p>
    <p><strong>애플리케이션:</strong> 온도 제어 (HVAC 조정); pH/EC 도징 (영양 주입); CO₂ 수준 (밸브 조절).</p>
    <p><strong>튜닝:</strong> 수동 조정 (Ziegler-Nichols 방법); 자동 조정 알고리즘; 최적 성능을 위해 중요.</p>
    
    <h3>5.5.3 AI/ML 기반 자동화</h3>
    <p><strong>기계 학습 모델:</strong> 회귀: 성장 속도, 수확량 예측; 분류: 질병 감지, 식물 단계 식별; 클러스터링: 최적 작물 그룹, 이상 감지; 강화 학습: 자체 최적화 제어 시스템.</p>
    
    <p><strong>애플리케이션:</strong> 예측 유지 관리 (장비 고장 예측); 수확량 예측 (시장 계획); 동적 환경 조정 (조건 최적화); 컴퓨터 비전 (해충, 질병, 성장 단계 감지).</p>
    
    <p><strong>구현:</strong> Python (TensorFlow, PyTorch, scikit-learn); 에지 AI (NVIDIA Jetson, Google Coral); 클라우드 ML (AWS SageMaker, Google AI Platform).</p>
    
    <h2>5.6 대시보드 및 시각화</h2>
    
    <h3>5.6.1 오픈 소스 대시보드</h3>
    <p><strong>Grafana:</strong> 시계열 데이터 시각화; InfluxDB/Prometheus와 통합; 아름다운 차트, 그래프, 게이지; 경보 및 알림 지원.</p>
    <p><strong>Node-RED 대시보드:</strong> Node-RED 흐름에 통합; 빠른 프로토타이핑; 간단한 UI 요소 (버튼, 슬라이더, 차트).</p>
    <p><strong>ThingsBoard:</strong> 전체 IoT 플랫폼; 장치 관리 + 시각화; 오픈 소스 및 클라우드 버전.</p>
    
    <h3>5.6.2 사용자 정의 웹 대시보드</h3>
    <p><strong>프론트엔드:</strong> React/Vue.js (동적 UI); Chart.js/D3.js (데이터 시각화); 반응형 디자인 (모바일 친화적).</p>
    <p><strong>백엔드:</strong> Node.js/Express (API 서버); Django/Flask (Python 웹 프레임워크); RESTful API (데이터 액세스).</p>
    <p><strong>실시간 업데이트:</strong> WebSocket (양방향 통신); MQTT (경량 메시징); Server-Sent Events (SSE, 단방향 푸시).</p>
    
    <h3>5.6.3 모바일 앱</h3>
    <p><strong>크로스 플랫폼:</strong> React Native (JavaScript); Flutter (Dart); Ionic (웹 기술). 장점: 한 번 작성, iOS + Android 배포.</p>
    <p><strong>네이티브:</strong> Swift (iOS); Kotlin/Java (Android). 장점: 최고 성능, 플랫폼별 기능.</p>
    <p><strong>기능:</strong> 실시간 센서 판독; 원격 제어 (펌프, 조명, 팬); 푸시 알림 (경보, 수확 시간); 카메라 피드 (실시간 식물 보기); 데이터 분석 (차트, 추세).</p>
    
    <h2>5.7 보안 및 개인 정보 보호</h2>
    
    <h3>5.7.1 IoT 보안 위협</h3>
    <ul>
        <li><strong>무단 액세스:</strong> 해커가 시스템 제어 획득</li>
        <li><strong>데이터 유출:</strong> 독점 작물 레시피, 비즈니스 데이터 도난</li>
        <li><strong>서비스 거부 (DoS):</strong> 공격자가 시스템 충돌 유발</li>
        <li><strong>중간자 (MitM) 공격:</strong> 센서와 서버 간 데이터 가로채기</li>
        <li><strong>펌웨어 조작:</strong> 악성 코드 삽입</li>
    </ul>
    
    <h3>5.7.2 보안 모범 사례</h3>
    <p><strong>네트워크 보안:</strong> 방화벽 (운영/관리 네트워크 분리); VPN (원격 액세스); 네트워크 세분화 (VLAN으로 IoT 장치 격리).</p>
    <p><strong>인증 및 암호화:</strong> 강력한 암호 (기본값 변경!); 다단계 인증 (MFA); SSL/TLS 암호화 (전송 중 데이터); SSH 키 (암호 대신).</p>
    <p><strong>장치 보안:</strong> 펌웨어 업데이트 (정기적으로 패치); 미사용 서비스 비활성화; 물리적 보안 (액세스 제어, 잠긴 인클로저).</p>
    <p><strong>모니터링 및 로깅:</strong> 침입 감지 시스템 (IDS); 액세스 로그 (누가 무엇을 언제); 정기 감사 (보안 검토).</p>
    
    <h2>5.8 결론</h2>
    <p>IoT, 센서 및 자동화는 현대 수직농업의 중추입니다. 이러한 기술은 정밀 제어, 실시간 모니터링, 데이터 기반 최적화를 가능하게 하여 수동 농업을 능가하는 수확량과 효율성을 달성합니다. 강력한 센서 네트워크, 신뢰할 수 있는 통신 및 지능형 자동화에 투자하면 수직 농장은 일관되게 고품질 작물을 생산하고 비용을 절감하며 상업적 성공을 달성할 수 있습니다. 다음 챕터에서는 작물 선택 및 생산 계획을 살펴보고 수익성 있는 작물 선택 및 연중 생산 최적화에 대한 전략을 다룹니다.</p>
    
    <div class="nav"><a href="chapter-04.html">← 챕터 4</a><a href="chapter-06.html">챕터 6 →</a></div>
    <footer><p style="color: #ffd700;">弘益人間 · 널리 인간을 이롭게 하라</p><p>© 2025 WIA Standards · MIT License</p></footer>
</body>
</html>
CH05EOF

# Chapter 06: Crop Selection & Production Planning
cat > /home/user/wia-standards/vertical-farming/ebook/ko/chapter-06.html << 'CH06EOF'
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>챕터 06 - WIA 수직농업 표준</title>
    <style>
        body {font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Apple SD Gothic Neo', 'Malgun Gothic', sans-serif;background: #0f172a;color: #f8fafc;line-height: 1.9;max-width: 900px;margin: 0 auto;padding: 40px 20px;}
        h1, h2, h3, h4 {color: #84CC16;margin-top: 2em;}h1 {border-bottom: 3px solid #84CC16;padding-bottom: 10px;}a {color: #60a5fa;text-decoration: none;}a:hover {text-decoration: underline;}
        code {background: #1e293b;padding: 2px 6px;border-radius: 4px;color: #10b981;}pre {background: #1e293b;padding: 20px;border-radius: 8px;overflow-x: auto;border-left: 4px solid #84CC16;}
        .info-box {background: #1e293b;padding: 20px;border-radius: 8px;margin: 20px 0;border-left: 4px solid #60a5fa;}
        table {width: 100%;border-collapse: collapse;margin: 20px 0;background: #1e293b;}th, td {padding: 12px;text-align: left;border-bottom: 1px solid #334155;}
        th {background: #334155;color: #84CC16;font-weight: bold;}.nav {margin: 40px 0;padding: 20px 0;border-top: 1px solid #334155;border-bottom: 1px solid #334155;display: flex;justify-content: space-between;}
        footer {margin-top: 60px;padding-top: 20px;border-top: 1px solid #334155;text-align: center;color: #94a3b8;}ul {line-height: 2.2;}li {margin: 12px 0;}
    </style>
</head>
<body>
    <div class="nav"><a href="chapter-05.html">← 챕터 5</a><a href="chapter-07.html">챕터 7 →</a></div>
    <h1>챕터 06: 작물 선택 및 생산 계획</h1>
    
    <h2>6.1 수익성 있는 작물 선택</h2>
    <p>올바른 작물을 선택하는 것은 수직농업 성공의 핵심입니다. 모든 작물이 수직 시스템에 적합한 것은 아니며, 수익성은 시장 수요, 성장 속도, 수확량 및 판매 가격에 따라 크게 달라집니다.</p>
    
    <h3>6.1.1 수직농업에 이상적인 작물 특성</h3>
    <ul>
        <li><strong>빠른 성장 주기:</strong> 더 짧은 터닝 타임 = 더 많은 연간 수확</li>
        <li><strong>컴팩트한 크기:</strong> 공간 효율성을 위한 작은 발자국</li>
        <li><strong>높은 시장 가격:</strong> 운영 비용 정당화를 위한 프리미엄 가격</li>
        <li><strong>짧은 유통 기한:</strong> 현지 생산이 경쟁 우위 제공</li>
        <li><strong>연중 수요:</strong> 일관된 생산 및 매출</li>
        <li><strong>낮은 노동 강도:</strong> 자동화된 수확 또는 간단한 관리</li>
    </ul>
    
    <h3>6.1.2 상위 수직농업 작물</h3>
    <table>
        <thead><tr><th>작물</th><th>성장 주기</th><th>수확량 (kg/m²/년)</th><th>시장 가격</th><th>난이도</th></tr></thead>
        <tbody>
            <tr><td>상추</td><td>25-35일</td><td>50-80</td><td>중간-높음</td><td>쉬움</td></tr>
            <tr><td>바질</td><td>30-45일</td><td>40-70</td><td>높음</td><td>쉬움</td></tr>
            <tr><td>마이크로그린</td><td>7-14일</td><td>60-120</td><td>매우 높음</td><td>쉬움</td></tr>
            <tr><td>케일/시금치</td><td>30-40일</td><td>45-75</td><td>중간</td><td>쉬움</td></tr>
            <tr><td>딸기</td><td>연중 (처음 수확까지 90일)</td><td>30-50</td><td>매우 높음</td><td>중간</td></tr>
            <tr><td>토마토</td><td>70-90일</td><td>40-70</td><td>중간</td><td>중간-어려움</td></tr>
            <tr><td>허브 (민트, 파슬리)</td><td>40-60일</td><td>30-60</td><td>높음</td><td>쉬움</td></tr>
            <tr><td>팩 초이</td><td>30-45일</td><td>40-65</td><td>중간</td><td>쉬움</td></tr>
        </tbody>
    </table>
    
    <h2>6.2 생산 계획 및 스케줄링</h2>
    
    <h3>6.2.1 연속 수확 시스템</h3>
    <p>일관된 생산을 위해 성숙 단계를 엇갈리게 합니다. 예: 10-구역 시스템에서 상추를 35일 주기로 재배: 구역 1: 0-3일 (파종); 구역 2: 4-7일 (초기 성장); ... 구역 10: 32-35일 (수확). 매주 구역 하나 수확 → 구역 청소 → 재파종. 결과: 일주일마다 동일한 수확량, 연중 일관된 생산.</p>
    
    <h3>6.2.2 작물 회전 전략</h3>
    <p>동일한 작물을 연속으로 재배하면 다음이 발생할 수 있습니다: 영양분 고갈 (일부 영양소는 더 빨리 소모됨); 해충 축적 (특정 작물 해충이 증가); 질병 압력 (병원균 축적). 회전 이점: 시스템 "휴식" (영양분 균형 재조정); 해충/질병 주기 중단; 시장 수요에 맞춘 다양화.</p>
    
    <p><strong>예시 회전:</strong> 1-3월: 상추 (겨울 수요 높음); 4-6월: 바질 (봄/여름 인기); 7-9월: 토마토 (여름 프리미엄 가격); 10-12월: 케일/시금치 (가을/겨울 수요).</p>
    
    <h2>6.3 품종 선택</h2>
    
    <h3>6.3.1 상추 품종</h3>
    <p><strong>버터헤드:</strong> 부드러운 잎, 달콤한 맛. 인기: Boston, Bibb. 성장 시간: 28-35일.</p>
    <p><strong>로메인:</strong> 바삭한 질감, 견고한 갈비. 인기: Parris Island, Jericho. 성장 시간: 30-40일.</p>
    <p><strong>리프 레터스:</strong> 느슨한 잎, 머리 형성 없음. 인기: Red Sails, Green Oak. 성장 시간: 25-30일 (가장 빠름).</p>
    <p><strong>크리스프헤드:</strong> 아이스버그 유형, 단단한 머리. 상업적이지만 느린 성장. 성장 시간: 40-50일.</p>
    
    <h3>6.3.2 허브 품종</h3>
    <p><strong>바질:</strong> 제노베제 (클래식 이탈리안, 높은 수요); 타이 바질 (감초 향, 아시아 요리); 레몬 바질 (감귤 노트, 전문 시장).</p>
    <p><strong>민트:</strong> 스피어민트 (음료, 디저트); 페퍼민트 (차, 약용); 초콜릿 민트 (전문, 프리미엄 가격).</p>
    <p><strong>파슬리:</strong> 곱슬 (장식); 평평한 잎/이탈리안 (요리, 더 많은 맛).</p>
    
    <h3>6.3.3 과채류 품종</h3>
    <p><strong>토마토:</strong> 체리 토마토 (빠른 성장, 높은 수확량, 인기); 비프스테이크 (큰, 프리미엄, 느린 성장); 산 마르자노 (페이스트, 요리용).</p>
    <p><strong>고추:</strong> 벨 페퍼 (다색, 높은 수요); 할라피뇨 (매운, 인기); 미니 스위트 페퍼 (스낵 크기, 프리미엄).</p>
    <p><strong>딸기:</strong> Albion (주중성, 연중 결실); Seascape (내열성, 좋은 풍미); Monterey (높은 수확량, 질병 저항성).</p>
    
    <h2>6.4 수확량 최적화</h2>
    
    <h3>6.4.1 식물 밀도</h3>
    <p>m²당 더 많은 식물 = 더 높은 수확량... 한 지점까지. 최적 밀도 찾기: 너무 조밀: 광 경쟁, 공기 흐름 감소, 질병 위험; 너무 희박: 공간 낭비, 수확량 감소.</p>
    
    <p><strong>권장 밀도:</strong> 상추: 50-80 식물/m²; 바질: 40-60 식물/m²; 마이크로그린: 1500-2500 g 종자/m²; 토마토: 2-4 식물/m² (수직 재배); 딸기: 8-12 식물/m².</p>
    
    <h3>6.4.2 다층 재배</h3>
    <p>수직 공간 활용: 통상적: 2-5미터 높이에 5-10층. 각 층: 30-50cm 간격 (작물 크기에 따라). 이점: 5-10배 생산량 (단일 층 대비); 동일한 발자국에서 더 많은 수확량. 과제: 각 층에 균일한 조명; 각 층으로의 공기 흐름; 유지 관리 접근성 (상부 층).</p>
    
    <h3>6.4.3 성장 단계 최적화</h3>
    <p><strong>발아/파종:</strong> 높은 습도 (70-85%), 온화한 광 (100-150 PPFD); 목표: 빠른 균일한 발아.</p>
    <p><strong>묘목:</strong> 중간 광 (150-250 PPFD), 온도 낮춤; 목표: 강한 컴팩트 묘목.</p>
    <p><strong>영양 생장:</strong> 최대 광 (200-400 PPFD), 최적 온도/영양; 목표: 빠른 바이오매스 축적.</p>
    <p><strong>수확 전:</strong> 조명 약간 감소 (비용 절약), 습도 낮춤; 목표: 품질 개선, 병 감소.</p>
    
    <h2>6.5 재정 계획</h2>
    
    <h3>6.5.1 비용 구조</h3>
    <p><strong>자본 지출 (CapEx):</strong> 시설 건설/개조; HVAC, 조명, 수경재배 시스템; IoT, 센서, 자동화; 총 CapEx: $500-2000/m² (재배 면적).</p>
    <p><strong>운영 비용 (OpEx):</strong> 전기 (조명, HVAC) - 40-60%; 노동 - 20-30%; 종자, 영양소 - 10-15%; 유지 관리, 수리 - 5-10%; 마케팅, 분배 - 5-10%.</p>
    
    <h3>5.2 수익 잠재력</h3>
    <p><strong>예시 계산 (100m² 시스템):</strong> 작물: 상추 (35일 주기); 밀도: 60 식물/m²; 수확량: 6000 머리/주기; 연간 주기: 10; 총 연간 생산: 60,000 머리/년.</p>
    <p><strong>매출:</strong> 도매 가격: $2/머리; 연간 매출: $120,000; 소매 직접 판매: $3-5/머리 → $180,000-300,000.</p>
    <p><strong>OpEx (연간):</strong> 전기: $30,000; 노동: $25,000; 소모품: $10,000; 기타: $10,000; 총: $75,000/년.</p>
    <p><strong>총 이익:</strong> 도매: $45,000/년; 소매: $105,000-225,000/년; 이익률: 38-75%.</p>
    
    <h3>6.5.3 손익분기점 분석</h3>
    <p>손익분기점 = CapEx + 누적 OpEx = 누적 수익. 예 (위에서): CapEx: $150,000 (100m² × $1500/m²); OpEx/년: $75,000; 수익/년: $120,000 (도매). 손익분기점: $150k / ($120k - $75k) = 3.3년.</p>
    <p>소매로 더 빠른 손익분기점: $150k / ($180k - $75k) = 1.4년.</p>
    
    <h2>6.6 시장 전략</h2>
    
    <h3>6.6.1 판매 채널</h3>
    <p><strong>소매 직접 (농민 시장, CSA):</strong> 장점: 최고 가격, 고객 관계; 단점: 시간 집약적, 대량 제한.</p>
    <p><strong>레스토랑/요리사:</strong> 장점: 프리미엄 가격, 일관된 수요, 브랜드 구축; 단점: 엄격한 품질, 배송 일정.</p>
    <p><strong>식료품점/슈퍼마켓:</strong> 장점: 대량, 일관된 판매; 단점: 가격 압력, 인증 요구 사항 (GAP).</p>
    <p><strong>식사 배달 키트:</strong> 장점: 예측 가능한 주문, 성장 시장; 단점: 엄격한 물류, 포장 요구 사항.</p>
    <p><strong>도매 유통업체:</strong> 장점: 대량, 간단한 물류; 단점: 가장 낮은 가격, 마진 압박.</p>
    
    <h3>6.6.2 차별화 전략</h3>
    <ul>
        <li><strong>지역/초신선:</strong> 수확 후 몇 시간 안에, 수입품보다 더 신선</li>
        <li><strong>유기농/무농약:</strong> 제어된 환경 = 살충제 없음, 프리미엄</li>
        <li><strong>희귀 품종:</strong> 전통 또는 특수 품종 (자주색 바질, 마이크로그린)</li>
        <li><strong>연중 가용성:</strong> 계절 작물이 제철 아닐 때</li>
        <li><strong>지속 가능성:</strong> 물 90% 적음, 수송 없음, 환경 스토리</li>
        <li><strong>추적 가능성:</strong> QR 코드로 농장에서 테이블까지, 투명성</li>
    </ul>
    
    <h2>6.7 품질 관리</h2>
    
    <h3>6.7.1 수확 후 처리</h3>
    <p><strong>수확 타이밍:</strong> 아침 일찍 (당 함량 높음); 최적 크기/성숙도; 손상 방지.</p>
    <p><strong>청소:</strong> 오염 제거를 위한 세척 (필요 시); 부드러운 취급 (타박상 방지); 위생적인 포장.</p>
    <p><strong>냉각:</strong> 수확 후 즉시 4-8°C로 냉각; 저온실 또는 냉장 보관; 유통 기한 2-3배 연장.</p>
    <p><strong>포장:</strong> 통기성 포장 (에틸렌 축적 방지); 브랜딩 라벨 (로고, 정보); 포장 날짜, 유통 기한.</p>
    
    <h3>6.7.2 식품 안전 인증</h3>
    <p><strong>GAP (우수 농업 관행):</strong> USDA 인증; 식품 안전 절차, 기록 보관; 슈퍼마켓이 종종 요구.</p>
    <p><strong>유기농 인증:</strong> USDA 유기농, EU 유기농; 엄격한 요구 사항, 높은 비용; 수경재배는 USDA 유기농 허용 (논란이 있지만).</p>
    <p><strong>HACCP (위해 요소 분석 및 중요 관리점):</strong> 식품 안전 관리 시스템; 위험 식별 및 통제; 상업 운영에 권장.</p>
    
    <h2>6.8 결론</h2>
    <p>작물 선택 및 생산 계획은 수직농업 수익성의 핵심 결정 요인입니다. 올바른 작물 선택, 생산 일정 최적화, 품질 보장 및 효과적인 마케팅은 성공적인 상업 운영을 만듭니다. 빠르게 성장하는 고가 작물에 집중하고, 연속 수확 시스템을 구현하며, 강력한 시장 관계를 구축하면 수직 농장은 수익성을 달성하고 지속할 수 있습니다. 다음 챕터에서는 지속 가능성과 자원 관리를 살펴보고 환경 영향을 최소화하고 장기 생존 가능성을 보장하는 전략을 다룹니다.</p>
    
    <div class="nav"><a href="chapter-05.html">← 챕터 5</a><a href="chapter-07.html">챕터 7 →</a></div>
    <footer><p style="color: #ffd700;">弘益人間 · 널리 인간을 이롭게 하라</p><p>© 2025 WIA Standards · MIT License</p></footer>
</body>
</html>
CH06EOF

# Continue with chapters 07-08 in next segment due to length...

echo "Chapters 05-06 created successfully"
