# WIA-CITY-014: 보안 시스템 표준 🔒

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/Version-1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/Standard-WIA--CITY--014-red.svg)](https://wia.org/standards/city-014)

## 개요

WIA-CITY-014 보안 시스템 표준은 도시 및 건물의 안전과 보안을 보장하기 위한 통합 보안 시스템의 국제 표준입니다.

본 표준은 CCTV 감시, 침입 탐지, 경비 시스템, 비상 호출, AI 영상 분석을 통합하여 포괄적인 보안 솔루션을 제공합니다.

### 주요 기능

- 🎥 **CCTV 감시 시스템**: IP 카메라, PTZ, 열화상 카메라 통합 관리
- 🚨 **침입 탐지 시스템**: PIR, 도어/창문 센서, 유리 파손 감지
- 👮 **경비 시스템**: 순찰 관리, 경비원 위치 추적, 체크포인트
- 📞 **비상 호출**: 패닉 버튼, 비디오 인터폰, 긴급 알림
- 🤖 **AI 영상 분석**: 객체 탐지, 행동 분석, 얼굴 인식, 번호판 인식
- 📊 **통합 관제**: 실시간 모니터링, 통합 대시보드, 비디오 월
- 🔍 **사고 조사**: 증거 수집, 조사 관리, 보고서 작성
- 🌐 **개방형 표준**: 다양한 제조사 및 시스템 간 호환

## 빠른 시작

### 1. 설치

```bash
# 저장소 클론
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/security-system-city

# 의존성 설치
./install.sh
```

### 2. TypeScript SDK 사용

```bash
# npm으로 설치
npm install @wia/city-security-system

# 또는 yarn
yarn add @wia/city-security-system
```

```typescript
import { SecuritySystemSDK } from '@wia/city-security-system';

const sdk = new SecuritySystemSDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/city-014/v1'
});

// 카메라 목록 조회
const cameras = await sdk.cameras.list();

// 라이브 스트림 가져오기
const stream = await sdk.cameras.getStream('camera-001', 'rtsp');
console.log('스트림 URL:', stream.data.stream_url);

// 센서 무장
await sdk.sensors.arm('sensor-pir-001');

// 보안 존 무장
await sdk.zones.arm('zone-floor-1', 'away');

// 경보 목록 조회
const alerts = await sdk.alerts.list({ severity: 'high' });

// 긴급 상황 알림
await sdk.emergency.trigger({
  type: 'intrusion',
  priority: 'critical',
  location: { building: 'Main Office', floor: '3F' },
  details: { description: '침입 감지됨' },
  reporter: { type: 'sensor', id: 'sensor-001' }
});
```

### 3. CLI 도구 사용

```bash
# API 키 설정
export WIA_SECURITY_SYSTEM_API_KEY="your-api-key"

# 카메라 목록 조회
./cli/security-system-city.sh list-cameras

# 특정 존의 카메라만 조회
./cli/security-system-city.sh list-cameras zone-entrance

# 라이브 스트림 URL 가져오기
./cli/security-system-city.sh get-stream camera-001 rtsp

# 스냅샷 캡처
./cli/security-system-city.sh snapshot camera-001

# 센서 무장
./cli/security-system-city.sh arm-sensor sensor-pir-001

# 보안 존 무장
./cli/security-system-city.sh arm-zone zone-floor-1 away

# 경보 확인
./cli/security-system-city.sh ack-alert alert-001 operator-kim

# 긴급 상황 알림
./cli/security-system-city.sh trigger-emergency fire "Building A" "화재 감지됨"

# 통합 대시보드
./cli/security-system-city.sh dashboard
```

## 주요 기능 상세

### 1. CCTV 감시 시스템

#### 1.1 지원 카메라 유형

- **고정형 카메라 (Fixed Camera)**: 일반 감시 용도
- **PTZ 카메라**: 원격 제어로 회전, 틸트, 줌 가능
- **돔 카메라**: 반달 방지, 실내외 겸용
- **열화상 카메라**: 완전 암흑 환경, 화재 감지
- **어안 카메라**: 360도 전방위 감시
- **파노라마 카메라**: 광역 감시

#### 1.2 카메라 제어

```typescript
// PTZ 카메라 제어
await sdk.cameras.ptzControl('camera-ptz-001', {
  action: 'preset',
  value: 3  // 프리셋 3번 위치로 이동
});

// 줌 인
await sdk.cameras.ptzControl('camera-ptz-001', {
  action: 'zoom',
  value: 10,
  speed: 5
});

// 스냅샷 캡처
const snapshot = await sdk.cameras.snapshot('camera-001');
console.log('스냅샷 URL:', snapshot.data.image_url);

// 녹화 영상 조회
const recordings = await sdk.cameras.getRecordings('camera-001', {
  start: '2025-12-25T00:00:00Z',
  end: '2025-12-25T23:59:59Z'
});
```

#### 1.3 영상 녹화 및 저장

- **연속 녹화**: 24시간 연속 녹화 (중요 구역)
- **모션 감지 녹화**: 움직임 감지 시 자동 녹화
- **스케줄 녹화**: 특정 시간대만 녹화 (야간/주말)
- **이벤트 녹화**: 경보 발생 시 자동 녹화
- **스마트 녹화**: AI 분석 기반 선택적 녹화

### 2. 침입 탐지 시스템

#### 2.1 센서 유형

- **PIR 모션 센서**: 인체 열 감지 (12m 범위)
- **도어/창문 센서**: 개폐 감지 (자석 센서)
- **유리 파손 센서**: 유리 깨지는 소리 감지 (9m 범위)
- **진동 센서**: 충격 및 진동 감지 (벽/천장/바닥)
- **레이저/빔 센서**: 외곽 경계 침입 감지

#### 2.2 보안 존 관리

```typescript
// 보안 존 생성
await sdk.zones.create({
  zone_name: '1층 로비',
  zone_type: 'interior',
  sensors: ['sensor-pir-001', 'sensor-door-001'],
  cameras: ['camera-lobby-001', 'camera-lobby-002'],
  arming_schedule: {
    weekdays: { start: '18:00', end: '08:00' },
    weekends: { start: '00:00', end: '23:59' },
    holidays: ['2025-01-01', '2025-12-25']
  },
  alarm_settings: {
    entry_delay: 30,
    exit_delay: 60,
    alarm_duration: 120,
    chime: true,
    notification: {
      sms: ['+82-10-1234-5678'],
      email: ['security@company.com'],
      push: true,
      patrol: true
    }
  },
  response_plan: {
    priority: 'high',
    auto_actions: ['lock_doors', 'turn_on_lights', 'sound_siren'],
    dispatch_security: true,
    notify_police: false
  }
});

// 보안 존 무장
await sdk.zones.arm('zone-floor-1', 'away');

// 보안 존 해제 (코드 입력)
await sdk.zones.disarm('zone-floor-1', '1234');
```

#### 2.3 다층 방어 (Defense in Depth)

```
1층 - 외곽 경계 (Perimeter):
  - 레이저/빔 센서
  - 울타리 진동 센서
  - 외부 PIR 센서
  - 외곽 CCTV

2층 - 건물 외벽 (Building Envelope):
  - 도어/창문 센서
  - 유리 파손 감지
  - 진동 센서
  - 외벽 카메라

3층 - 내부 공간 (Interior):
  - 내부 PIR 센서
  - 복도 카메라
  - 내부 도어 센서

4층 - 중요 구역 (Critical Areas):
  - 고감도 PIR
  - 고해상도 카메라
  - 출입 통제
  - 온도/습도 센서
```

### 3. 경비 시스템

#### 3.1 순찰 관리

```typescript
// 순찰 시작
const patrol = await sdk.patrols.start('route-night-001', 'guard-kim');

// 체크포인트 스캔
await sdk.patrols.scanCheckpoint('checkpoint-001', 'guard-kim', {
  scan_code: 'NFC-CP001',
  tasks_completed: ['check_doors', 'check_lights'],
  notes: '모든 이상 없음'
});

// 순찰 완료
const report = await sdk.patrols.complete(patrol.data.patrol_id);

// 순찰 이력 조회
const history = await sdk.patrols.getHistory({
  guard_id: 'guard-kim',
  start: '2025-12-01',
  end: '2025-12-31'
});
```

#### 3.2 경비원 위치 추적

```typescript
// 경비원 위치 업데이트
await sdk.guards.updateLocation('guard-kim', {
  latitude: 37.5665,
  longitude: 126.9780,
  building: 'Main Office',
  floor: '2F'
});

// 경비원 상태 변경
await sdk.guards.updateStatus('guard-kim', 'on_patrol');

// 패닉 버튼 (긴급 상황)
await sdk.guards.panic('guard-kim', {
  latitude: 37.5665,
  longitude: 126.9780
});
```

### 4. 비상 호출 시스템

#### 4.1 긴급 상황 유형

- **의료 응급 (medical)**: 부상, 질병, 응급처치 필요
- **화재 (fire)**: 화재 발생, 연기 감지
- **침입 (intrusion)**: 무단 침입, 도난
- **폭행 (assault)**: 폭력, 위협
- **자연재해 (natural_disaster)**: 지진, 홍수, 태풍
- **폭탄 위협 (bomb_threat)**: 폭발물 신고
- **총기 난사 (active_shooter)**: 총기 사고
- **위험물질 (hazmat)**: 화학물질 유출
- **긴급 대피 (evacuation)**: 건물 긴급 대피

#### 4.2 긴급 상황 대응

```typescript
// 긴급 상황 알림
await sdk.emergency.trigger({
  type: 'fire',
  priority: 'critical',
  location: {
    building: 'Building A',
    floor: '5F',
    room: '501'
  },
  reporter: {
    type: 'employee',
    name: '김철수'
  },
  details: {
    description: '5층 501호에서 화재 발생',
    casualties: 0,
    hazard_level: 'high',
    spread_risk: true
  },
  response: {
    dispatched: ['fire_dept', 'guard_team_1'],
    eta: [5, 2],
    status: 'dispatched'
  },
  communications: {
    pa_announcement: true,
    sms_broadcast: true,
    app_notification: true,
    email_notification: true
  }
});

// 활성 긴급 상황 조회
const activeEmergencies = await sdk.emergency.getActive();

// 긴급 상황 상태 업데이트
await sdk.emergency.updateStatus('emergency-001', 'on_scene');

// 긴급 상황 해결
await sdk.emergency.resolve('emergency-001', {
  resolved_by: 'chief-park',
  outcome: '화재 진압 완료, 인명 피해 없음',
  report: '소화기로 초기 진화 성공'
});

// 긴급 방송
await sdk.emergency.broadcast(
  '전 직원은 즉시 건물을 대피하시기 바랍니다.',
  ['zone-all']
);
```

### 5. AI 영상 분석

#### 5.1 객체 탐지 및 추적

```typescript
// 실시간 객체 탐지
const detections = await sdk.analytics.detectObjects('camera-001');

// 결과 처리
detections.data.detections.forEach(detection => {
  console.log(`${detection.object.type} 감지: 신뢰도 ${detection.object.confidence}`);
  if (detection.object.type === 'person') {
    console.log('위치:', detection.object.bounding_box);
    if (detection.tracking) {
      console.log('추적 ID:', detection.tracking.track_id);
      console.log('이동 경로:', detection.tracking.path);
    }
  }
});
```

#### 5.2 행동 분석

```typescript
// 이상 행동 분석
const behavior = await sdk.analytics.analyzeBehavior('camera-entrance-001');

behavior.data.detected_behaviors.forEach(b => {
  if (b.behavior_type === 'loitering' && b.duration > 300) {
    console.log(`⚠️ 배회 감지: ${b.duration}초 경과`);
    if (b.alert.triggered) {
      console.log('경보 발생!');
    }
  }
});
```

#### 5.3 얼굴 인식

```typescript
// 얼굴 인식
const faces = await sdk.analytics.recognizeFaces('camera-entrance-001');

faces.data.faces.forEach(face => {
  if (face.recognition?.matched) {
    console.log(`✅ 인식: ${face.recognition.person_id}`);
    if (face.recognition.watchlist_match) {
      console.log(`🚨 워치리스트 일치: ${face.recognition.watchlist_match.reason}`);
    }
  } else {
    console.log('⚠️ 미등록 인물');
  }
});
```

#### 5.4 차량 번호판 인식 (ANPR/LPR)

```typescript
// 번호판 인식
const plates = await sdk.analytics.recognizePlates('camera-gate-001');

plates.data.plates.forEach(plate => {
  console.log(`차량 번호: ${plate.license_plate.number}`);
  console.log(`신뢰도: ${plate.license_plate.confidence}`);

  if (plate.access_control) {
    if (plate.access_control.authorized) {
      console.log('✅ 출입 승인');
    } else if (plate.access_control.blacklist_match) {
      console.log('🚨 블랙리스트 차량!');
    }
  }
});
```

#### 5.5 군중 분석

```typescript
// 군중 밀도 분석
const crowd = await sdk.analytics.analyzeCrowd('camera-plaza-001', 'zone-plaza');

console.log(`인원 수: ${crowd.data.crowd_metrics.people_count}명`);
console.log(`밀도: ${crowd.data.crowd_metrics.density} 명/m²`);
console.log(`수용 현황: ${crowd.data.crowd_metrics.occupancy.percentage}%`);

if (crowd.data.alerts.overcrowding) {
  console.log('⚠️ 과밀 경고!');
}

if (crowd.data.alerts.stampede_risk) {
  console.log('🚨 압사 위험!');
}

// 추천 조치
crowd.data.recommendations.forEach(rec => {
  console.log(`추천: ${rec}`);
});
```

### 6. 경보 및 이벤트 관리

#### 6.1 경보 관리

```typescript
// 경보 목록 조회
const alerts = await sdk.alerts.list({
  severity: 'high',
  status: 'active'
});

// 경보 확인
await sdk.alerts.acknowledge('alert-001', 'operator-kim');

// 경보 해결
await sdk.alerts.resolve('alert-001', 'operator-kim', '침입 탐지 오경보 - 청소 직원');

// 경보 종료
await sdk.alerts.close('alert-001');
```

#### 6.2 이벤트 조회

```typescript
// 이벤트 목록 조회
const events = await sdk.events.list({
  type: 'alarm',
  severity: 'critical',
  start: '2025-12-25T00:00:00Z',
  end: '2025-12-25T23:59:59Z'
});

// 이벤트 검색
const searchResults = await sdk.events.search({
  keyword: '화재',
  filters: {
    severity: ['high', 'critical']
  },
  date_range: {
    startDate: '2025-12-01',
    endDate: '2025-12-31'
  }
});

// 이벤트 내보내기
const exportResult = await sdk.events.export({
  format: 'pdf',
  start: '2025-12-01',
  end: '2025-12-31',
  filters: {
    severity: ['high', 'critical']
  }
});

console.log('보고서 다운로드:', exportResult.data.download_url);
```

### 7. 통합 관제 센터

#### 7.1 대시보드

```typescript
// 통합 대시보드
const dashboard = await sdk.control.getDashboard();

console.log(`활성 경보: ${dashboard.data.active_alerts}개`);
console.log(`온라인 카메라: ${dashboard.data.cameras_online}대`);
console.log(`온라인 센서: ${dashboard.data.sensors_online}개`);
console.log(`근무 중 경비원: ${dashboard.data.guards_on_duty}명`);

console.log('최근 이벤트:');
dashboard.data.recent_events.forEach(event => {
  console.log(`  [${event.severity}] ${event.event_type}: ${event.details.description}`);
});
```

#### 7.2 비디오 월

```typescript
// 비디오 월 설정 조회
const videoWall = await sdk.control.getVideoWall('video-wall-001');

// 프리셋 적용
await sdk.control.setVideoWallPreset('video-wall-001', 'preset-all-entrances');
```

### 8. 사고 조사

```typescript
// 사고 조사 생성
await sdk.investigations.create({
  status: 'open',
  incident_details: {
    date: '2025-12-25',
    time: '14:30',
    location: {
      building: 'Main Office',
      floor: '3F',
      room: '301'
    },
    type: '절도',
    severity: 'high',
    description: '노트북 도난 사건'
  },
  involved_parties: {
    victims: [{
      name: '김철수',
      contact: '+82-10-1234-5678'
    }]
  },
  investigation: {
    lead_investigator: 'chief-park',
    team_members: ['investigator-lee', 'investigator-choi'],
    findings: '',
    conclusion: '',
    recommendations: []
  },
  reporting: {
    police_report_filed: true,
    police_report_number: '2025-12345'
  }
});

// 증거 추가
await sdk.investigations.addEvidence('incident-001', {
  type: 'video',
  data: {
    camera_id: 'camera-301',
    start_time: '2025-12-25T14:00:00Z',
    end_time: '2025-12-25T15:00:00Z',
    file_url: 'https://storage.example.com/evidence/video-001.mp4',
    notes: '용의자가 301호에 진입하는 장면'
  }
});

// 조사 종료
await sdk.investigations.close('incident-001');
```

## API 명세

### 엔드포인트 목록

#### 카메라 관리
```
GET    /api/v1/cameras              - 카메라 목록 조회
GET    /api/v1/cameras/{id}         - 카메라 상세 조회
POST   /api/v1/cameras              - 카메라 등록
PUT    /api/v1/cameras/{id}         - 카메라 정보 수정
DELETE /api/v1/cameras/{id}         - 카메라 삭제
GET    /api/v1/cameras/{id}/stream  - 실시간 스트림
POST   /api/v1/cameras/{id}/ptz     - PTZ 제어
POST   /api/v1/cameras/{id}/snapshot - 스냅샷 캡처
GET    /api/v1/cameras/{id}/recordings - 녹화 영상 조회
```

#### 센서 관리
```
GET    /api/v1/sensors              - 센서 목록 조회
GET    /api/v1/sensors/{id}         - 센서 상세 조회
POST   /api/v1/sensors              - 센서 등록
PUT    /api/v1/sensors/{id}         - 센서 정보 수정
DELETE /api/v1/sensors/{id}         - 센서 삭제
POST   /api/v1/sensors/{id}/arm     - 센서 무장
POST   /api/v1/sensors/{id}/disarm  - 센서 해제
POST   /api/v1/sensors/{id}/test    - 센서 테스트
```

#### 보안 존 관리
```
GET    /api/v1/zones                - 보안 존 목록 조회
GET    /api/v1/zones/{id}           - 보안 존 상세 조회
POST   /api/v1/zones                - 보안 존 생성
PUT    /api/v1/zones/{id}           - 보안 존 수정
DELETE /api/v1/zones/{id}           - 보안 존 삭제
POST   /api/v1/zones/{id}/arm       - 보안 존 무장
POST   /api/v1/zones/{id}/disarm    - 보안 존 해제
```

#### 경보 관리
```
GET    /api/v1/alerts               - 경보 목록 조회
GET    /api/v1/alerts/{id}          - 경보 상세 조회
POST   /api/v1/alerts/{id}/acknowledge - 경보 확인
POST   /api/v1/alerts/{id}/resolve  - 경보 해결
POST   /api/v1/alerts/{id}/close    - 경보 종료
```

#### 긴급 상황 관리
```
POST   /api/v1/emergency/alert      - 긴급 상황 알림
GET    /api/v1/emergency/active     - 활성 긴급 상황 조회
GET    /api/v1/emergency/{id}       - 긴급 상황 상세 조회
POST   /api/v1/emergency/{id}/status - 상태 업데이트
POST   /api/v1/emergency/{id}/resolve - 긴급 상황 해결
POST   /api/v1/emergency/broadcast  - 긴급 방송
```

## 보안 및 개인정보보호

### 데이터 보안

- **암호화**: TLS 1.3, AES-256
- **인증**: OAuth 2.0, JWT 토큰
- **권한 관리**: RBAC (Role-Based Access Control)
- **감사 로그**: 모든 접근 및 조작 기록

### 개인정보보호

- **영상 보관 기간**: 법적 요구사항 준수 (일반적으로 30일)
- **접근 권한**: 최소 권한 원칙 적용
- **익명화 처리**: 필요 시 얼굴 블러링
- **촬영 안내**: 명확한 CCTV 촬영 안내 표지판
- **GDPR 준수**: EU 지역 운영 시 GDPR 완전 준수

## 시스템 요구사항

### 서버 요구사항

- **OS**: Linux (Ubuntu 20.04+), Windows Server 2019+
- **CPU**: 8코어 이상 (영상 분석 시 16코어 권장)
- **RAM**: 16GB 이상 (영상 분석 시 32GB 권장)
- **GPU**: NVIDIA GPU (CUDA 지원) - AI 분석용
- **저장공간**: 카메라당 1TB 권장 (30일 보관 기준)
- **네트워크**: 1Gbps 이상

### 클라이언트 요구사항

- **브라우저**: Chrome 90+, Firefox 88+, Edge 90+
- **모바일 OS**: iOS 14+, Android 10+
- **네트워크**: 안정적인 인터넷 연결

## 성능 지표 (KPI)

| KPI | 목표 | 측정 방법 |
|-----|------|----------|
| 시스템 가용성 | 99.9% | 월간 다운타임 측정 |
| 경보 대응 시간 | 1분 이내 | 평균 대응 시간 |
| 오경보율 | 5% 이하 | 오경보 / 총 경보 |
| 영상 손실률 | 0.1% 이하 | 손실 시간 / 전체 시간 |
| 센서 온라인율 | 98% 이상 | 온라인 센서 / 전체 센서 |
| AI 탐지 정확도 | 95% 이상 | 정확한 탐지 / 전체 탐지 |

## 문제 해결

### 카메라 연결 안 됨

```bash
# 카메라 네트워크 연결 확인
ping <camera-ip>

# 포트 열림 확인
telnet <camera-ip> 554

# 카메라 재부팅
# (카메라 관리 페이지에서 재부팅)
```

### 센서 오동작

```bash
# 센서 테스트
./cli/security-system-city.sh test-sensor sensor-001

# 센서 재설정
./cli/security-system-city.sh disarm-sensor sensor-001
./cli/security-system-city.sh arm-sensor sensor-001
```

### API 연결 실패

```bash
# API 연결 테스트
curl -H "Authorization: Bearer $WIA_SECURITY_SYSTEM_API_KEY" \
  https://api.wia.org/city-014/v1/control/status

# 응답이 없으면 엔드포인트 확인
echo $WIA_SECURITY_SYSTEM_ENDPOINT
```

## 라이선스

MIT License - 자유롭게 사용, 수정, 배포 가능

## 기여하기

기여를 환영합니다! Pull Request를 보내주세요.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 지원

- 📖 문서: https://wia.org/standards/city-014
- 💬 커뮤니티: https://github.com/WIA-Official/wia-standards/discussions
- 📧 이메일: standards@wia-official.org
- 🐛 버그 리포트: https://github.com/WIA-Official/wia-standards/issues

## 관련 표준

- [WIA-CITY-013: 접근 제어 시스템](../access-control-system)
- [WIA-CITY-015: 화재 안전 시스템](../fire-safety-system)
- [WIA-AI-XXX: AI 영상 분석](../../ai/)
- [WIA-SEC-XXX: 암호화 및 보안](../../security/)

## 변경 이력

### v1.0.0 (2025-12-25)
- 초기 릴리스
- CCTV 감시 시스템
- 침입 탐지 시스템
- 경비 시스템 통합
- 비상 호출 시스템
- AI 영상 분석
- 통합 관제 센터

---

**홍익인간 (弘益人間) (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
