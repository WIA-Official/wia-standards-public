# WIA-CITY-015: 출입 통제 시스템 표준 🔐

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/Version-1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/Standard-WIA--CITY--015-sky.svg)](https://wia.org/standards/city-015)

## 개요

WIA-CITY-015 출입 통제 시스템 표준은 건물, 시설, 구역의 물리적 접근을 체계적으로 관리하고 제어하기 위한 통합 표준입니다. 다양한 인증 방법(카드키, 생체인식, PIN), 방문자 관리, 주차장 연동, 비상 대응을 포괄하는 완전한 출입 통제 솔루션을 제공합니다.

### 주요 기능

- 🔐 **다중 인증 방법**: RFID/NFC 카드, 생체인식(지문/얼굴/홍채), PIN, 모바일 크리덴셜
- 👥 **사용자 관리**: 직원, 계약자, 방문자 등 다양한 사용자 유형 지원
- 🚪 **출입구 제어**: 실시간 모니터링, 원격 개방/잠금, Anti-Passback
- 📅 **시간/구역 기반 접근**: 시간대별, 구역별 세밀한 접근 권한 설정
- 🎫 **방문자 관리**: 사전 등록, QR 코드 발급, 체크인/체크아웃
- 🚗 **주차장 통합**: 차량 번호 인식, 정기권 관리, 입출차 자동 기록
- 🏢 **엘리베이터 제어**: 층별 접근 권한에 따른 자동 호출
- 📊 **출입 로그**: 모든 출입 기록 저장, 검색, 감사 추적
- 🚨 **비상 모드**: 화재, 재난 시 자동 개방 또는 폐쇄
- 📈 **실시간 분석**: 재실 현황, 출입 통계, 이상 탐지

## 빠른 시작

### 1. 설치

```bash
# 저장소 클론
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/access-control-system

# 의존성 설치
./install.sh
```

### 2. TypeScript SDK 사용

```typescript
import { AccessControlSDK } from '@wia/city-access-control';

const sdk = new AccessControlSDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/city-015/v1'
});

// 실시간 대시보드 조회
const dashboard = await sdk.getRealtimeDashboard('BLD-001');
console.log('현재 재실 인원:', dashboard.data.occupancy.current, '명');
console.log('오늘 총 출입:', dashboard.data.accessActivity.totalToday, '회');

// 사용자 정보 조회
const user = await sdk.getUserInfo('EMP-001');
console.log('사용자:', user.data.name);
console.log('부서:', user.data.department);
console.log('접근 권한:', user.data.accessZones);

// 접근 권한 부여
await sdk.grantAccess({
  userId: 'EMP-001',
  accessZones: ['SERVER-ROOM', 'DATA-CENTER'],
  validFrom: '2025-12-25T00:00:00+09:00',
  validUntil: '2025-12-31T23:59:59+09:00',
  timeRestrictions: {
    weekdays: { start: '09:00', end: '18:00' }
  }
});

// 출입구 원격 개방
await sdk.unlockDoor({
  accessPointId: 'AP-MAIN-001',
  duration: 10,  // 10초 동안 개방
  reason: '긴급 상황',
  authorizedBy: 'admin'
});

// 방문자 사전 등록
const visitor = await sdk.registerVisitor({
  name: '홍길동',
  company: 'ABC 주식회사',
  phone: '+82-10-1234-5678',
  purpose: '업무 미팅',
  host: {
    employeeId: 'EMP-001',
    name: '김철수',
    department: '영업부',
    email: 'kim@company.com',
    phone: '+82-10-9876-5432'
  },
  visitDate: '2025-12-25',
  timeSlot: {
    start: '14:00',
    end: '16:00'
  },
  accessZones: ['LOBBY', 'MEETING-ROOM-A']
});

console.log('방문자 ID:', visitor.data.visitorId);
console.log('QR 코드:', visitor.data.credential.code);

// 방문자 체크인
await sdk.checkInVisitor('VIS-20251225-001', {
  photoUrl: '/photos/visitor-001.jpg'
});

// 주차장 현황 조회
const parkingStatus = await sdk.getParkingLotStatus('PL-001');
console.log('주차 가능:', parkingStatus.data.availableSlots, '면');
console.log('주차율:', parkingStatus.data.occupancyRate, '%');

// 입차 기록
await sdk.recordParkingEntry({
  vehicleId: 'VEH-001',
  licensePlate: '12가3456',
  gateId: 'GATE-ENTRY-001',
  parkingZone: 'B1',
  photoUrl: '/parking/entry-001.jpg'
});

// 출입 로그 조회
const logs = await sdk.getAccessLogs({
  userId: 'EMP-001',
  dateRange: {
    startDate: '2025-12-25T00:00:00+09:00',
    endDate: '2025-12-25T23:59:59+09:00'
  },
  page: 1,
  limit: 20
});

console.log('출입 기록:', logs.data.items.length, '건');

// 비상 모드 활성화
await sdk.activateEmergency({
  emergencyType: 'fire',
  actions: ['unlock-all', 'sound-alarm'],
  reason: '화재 발생',
  activatedBy: 'admin'
});
```

### 3. CLI 도구 사용

```bash
# API 설정
./cli/access-control-system.sh config

# 실시간 대시보드
./cli/access-control-system.sh dashboard BLD-001

# 사용자 목록
./cli/access-control-system.sh users list

# 사용자 정보 조회
./cli/access-control-system.sh users get EMP-001

# 접근 권한 부여
./cli/access-control-system.sh grant-access EMP-001 SERVER-ROOM

# 접근 권한 철회
./cli/access-control-system.sh revoke-access EMP-001 SERVER-ROOM

# 출입구 원격 개방 (10초)
./cli/access-control-system.sh unlock AP-MAIN-001 10

# 출입 로그 조회
./cli/access-control-system.sh access-logs

# 특정 사용자의 출입 로그
./cli/access-control-system.sh access-logs EMP-001

# 방문자 목록
./cli/access-control-system.sh visitors list

# 방문자 체크인
./cli/access-control-system.sh check-in VIS-20251225-001

# 방문자 체크아웃
./cli/access-control-system.sh check-out VIS-20251225-001

# 주차장 현황
./cli/access-control-system.sh parking-status PL-001

# 비상 모드 활성화
./cli/access-control-system.sh emergency-activate fire

# 비상 모드 해제
./cli/access-control-system.sh emergency-clear EMG-001

# 활성 알람 조회
./cli/access-control-system.sh alerts BLD-001
```

### 4. 상세 사양 확인

- **스펙 문서**: [`spec/WIA-CITY-015-v1.0.md`](spec/WIA-CITY-015-v1.0.md)

## 저장소 구조

```
access-control-system/
├── README.md              # 본 문서
├── install.sh             # 설치 스크립트
├── spec/
│   └── WIA-CITY-015-v1.0.md  # 상세 스펙 (한국어)
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts    # 타입 정의
│       │   └── index.ts    # SDK 구현
│       └── package.json    # npm 패키지 설정
└── cli/
    └── access-control-system.sh  # CLI 도구
```

## 기술 범위

### 인증 방법

#### 1. 카드키 시스템

**RFID (Radio Frequency Identification)**:
- **저주파 (125kHz)**: EM4100, HID Prox - 기본 출입 통제
- **고주파 (13.56MHz)**: MIFARE Classic, MIFARE DESFire - 보안 강화, 결제 연동
- **초고주파 (860-960MHz)**: 장거리 인식, 주차장 차량 태그

**NFC (Near Field Communication)**:
- 스마트폰 지원
- 모바일 크리덴셜
- QR 코드와 병행

#### 2. 생체인식

**지문 인식**:
- 기술: 광학식, 정전식, 초음파식
- FAR: ≤ 0.001% (1/100,000)
- FRR: ≤ 1%
- 인식 시간: ≤ 1초
- Liveness Detection (위조 방지)

**얼굴 인식**:
- 기술: 2D, 3D (Structured Light, ToF), 열화상
- FAR: ≤ 0.01% (1/10,000)
- FRR: ≤ 3%
- 인식 거리: 0.5-2m
- 마스크 착용 시에도 인식 가능

**홍채 인식**:
- 최고 정확도
- FAR: ≤ 0.0001% (1/1,000,000)
- FRR: ≤ 0.5%
- 쌍둥이도 구분 가능

**정맥 인식**:
- 손가락 정맥, 손바닥 정맥
- 위조 불가능 (체내 혈관)
- 높은 위생성

#### 3. PIN 및 비밀번호

- 4-8자리 숫자 PIN
- 영문+숫자+특수문자 비밀번호
- bcrypt 해싱
- 실패 횟수 제한
- 정기 변경 정책

#### 4. 다중 인증 (MFA)

고보안 구역에서는 2개 이상의 인증 방법 조합:

| 보안 수준 | 인증 방법 | 적용 예 |
|----------|----------|---------|
| 낮음 | 카드 | 일반 사무실 |
| 중간 | 카드 + PIN | 회의실 |
| 높음 | 생체인식 + PIN | 연구실 |
| 매우 높음 | 카드 + 생체인식 + PIN | 데이터센터, 금고 |

### 방문자 관리 시스템

#### 프로세스

```
1. 사전 등록
   └─ 직원이 방문자 정보 입력
   └─ 호스트 지정, 방문 일시, 목적 입력

2. 초대장 발송
   └─ 이메일/SMS로 QR 코드 발급
   └─ 방문 일시, 장소, 주차 정보 포함

3. 체크인
   └─ QR 코드 스캔 또는 신분증 제시
   └─ 사진 촬영 (선택)
   └─ 임시 출입증 발급

4. 출입
   └─ 지정된 구역만 접근 가능
   └─ 시간 제한 적용
   └─ 호스트 동반 규칙 (선택)

5. 체크아웃
   └─ 퇴실 시 방문증 반납
   └─ 체크아웃 시간 기록
```

#### 방문자 유형

- **업무 방문**: 거래처, 협력사
- **면접 방문**: 채용 후보자
- **배송**: 택배, 물품 반입
- **유지보수**: 외주 기술자
- **VIP**: 임원 방문객, 정부 관계자

### 주차장 출입 통제

#### 차량 인식

**LPR (License Plate Recognition)**:
- 카메라: 1080p 이상, IR 야간 촬영
- OCR 엔진: 한글, 영문, 숫자 인식
- 인식률: ≥ 98% (주간), ≥ 95% (야간)
- 처리 시간: ≤ 1초

**RFID 태그**:
- 주파수: 2.45GHz (능동형), 860-960MHz (수동형)
- 읽기 거리: 5-10m
- 차량 부착: 앞유리 상단

#### 주차 관리

```json
{
  "parkingPass": {
    "type": "monthly",
    "validFrom": "2025-12-01",
    "validUntil": "2025-12-31",
    "assignedSlot": "B1-A-15"
  },
  "billing": {
    "entryTime": "2025-12-25T08:30:00+09:00",
    "exitTime": "2025-12-25T18:45:00+09:00",
    "duration": 615,
    "fee": 8000,
    "paymentMethod": "credit-card"
  }
}
```

### 엘리베이터 층별 접근 제어

#### 시스템 연동

```typescript
// 사용자 층별 권한 설정
const elevatorRules = {
  userId: 'EMP-001',
  allowedFloors: ['1F', '2F', '5F', '10F', '15F'],
  timeRestrictions: {
    weekdays: { start: '07:00', end: '22:00' },
    weekends: { start: '09:00', end: '18:00' }
  }
};

// 엘리베이터 호출
await sdk.requestElevatorCall({
  userId: 'EMP-001',
  credentialId: 'CARD-001',
  currentFloor: '1F',
  destinationFloor: '15F'
});
// → 허가된 층이면 엘리베이터 자동 호출
// → 엘리베이터 내 15층 버튼 자동 활성화
```

### 출입 로그 및 감사

#### 로그 형식

```json
{
  "logId": "LOG-20251225-123456",
  "timestamp": "2025-12-25T09:15:32+09:00",
  "accessPointId": "AP-MAIN-ENTRANCE",
  "accessPointName": "본관 정문",
  "userId": "EMP-001",
  "userName": "김철수",
  "credentialType": "rfid-card",
  "action": "entry",
  "result": "granted",
  "doorOpenDuration": 3.5,
  "photoCapture": "/logs/photos/20251225-123456.jpg"
}
```

#### 로그 보존 및 감사

| 로그 유형 | 보존 기간 | 백업 |
|----------|----------|------|
| 일반 출입 로그 | 3년 | 일일 |
| 거부된 출입 시도 | 5년 | 일일 |
| 비상 출입 로그 | 영구 | 실시간 |
| 감사 로그 (시스템 변경) | 10년 | 실시간 |

#### 감사 리포트

**일일 리포트**:
- 총 출입 횟수
- 거부된 출입 시도
- 시간외 출입
- 미퇴실 인원

**월간 리포트**:
- 출입 패턴 분석
- 구역별 이용 현황
- 이상 징후 탐지
- 컴플라이언스 리포트

### 비상시 출입 통제

#### 비상 모드 유형

**화재 비상**:
```javascript
{
  emergencyType: 'fire',
  actions: [
    'unlock-all-doors',      // 모든 도어 자동 개방
    'disable-elevators',     // 엘리베이터 1층 강제 이동
    'activate-emergency-lighting',
    'sound-evacuation-alarm',
    'notify-fire-department'
  ]
}
```

**보안 비상 (침입, 테러)**:
```javascript
{
  emergencyType: 'intrusion',
  actions: [
    'lock-all-doors',        // 모든 도어 자동 폐쇄
    'lock-perimeter',        // 외부 출입구 차단
    'activate-cctv-recording',
    'sound-security-alarm',
    'notify-police'
  ]
}
```

**재난 비상 (지진, 홍수)**:
```javascript
{
  emergencyType: 'natural-disaster',
  actions: [
    'unlock-exits',          // 비상구만 개방
    'provide-safe-route',    // 안전 경로 안내
    'disable-automatic-locks',
    'sound-evacuation-alarm'
  ]
}
```

## 데이터 모델

### 사용자

```typescript
interface UserInfo {
  userId: string;
  employeeId?: string;
  name: string;
  department?: string;
  position?: string;
  email: string;
  phone: string;
  userType: 'employee' | 'contractor' | 'visitor' | 'vip' | 'admin';
  status: 'active' | 'suspended' | 'expired';
  credentials: Credential[];
  accessLevel: string;
  accessZones: string[];
  schedule?: AccessSchedule;
}
```

### 크리덴셜

```typescript
interface AccessCard {
  credentialId: string;
  userId: string;
  type: 'rfid-card' | 'nfc-card';
  cardNumber: string;
  facilityCode?: string;
  technology: 'EM4100' | 'HID-Prox' | 'MIFARE-Classic' | 'MIFARE-DESFire';
  status: 'active' | 'suspended' | 'lost' | 'expired';
  validFrom: string;
  validUntil?: string;
}

interface BiometricCredential {
  credentialId: string;
  userId: string;
  type: 'fingerprint' | 'face' | 'iris' | 'vein';
  templateHash: string;
  enrolledDate: string;
  quality: number;  // 0-100
}
```

### 출입구

```typescript
interface AccessPoint {
  accessPointId: string;
  name: string;
  location: {
    building: string;
    floor: string;
    zone: string;
  };
  type: 'main-entrance' | 'side-entrance' | 'employee-entrance' | 'parking-gate' | 'elevator';
  devices: {
    reader: ReaderInfo;
    lock: LockInfo;
    doorSensor: DoorSensor;
    camera?: CameraInfo;
  };
  accessControl: {
    normallyOpen: boolean;
    unlockDuration: number;
    antiPassback: boolean;
    twoPersonRule: boolean;
  };
}
```

### 출입 로그

```typescript
interface AccessLog {
  logId: string;
  timestamp: string;
  accessPointId: string;
  userId?: string;
  credentialType?: string;
  action: 'entry' | 'exit' | 'denied';
  result: 'granted' | 'denied-invalid-credential' | 'denied-time-restriction' | 'denied-zone-restriction';
  photoCapture?: string;
}
```

## API 명세

### 인증

```bash
# API Key 발급
POST /api/v1/auth/register
{
  "organizationName": "ABC 주식회사",
  "email": "admin@abc.com",
  "buildingId": "BLD-001"
}

# 헤더
Authorization: Bearer {apiKey}
Content-Type: application/json
X-WIA-Standard: CITY-015
X-WIA-Version: 1.0.0
```

### 주요 엔드포인트

| 분류 | 엔드포인트 | 메서드 | 설명 |
|------|-----------|--------|------|
| **사용자** | `/api/v1/users` | GET | 사용자 목록 |
| | `/api/v1/users` | POST | 사용자 등록 |
| | `/api/v1/users/{id}` | GET | 사용자 조회 |
| | `/api/v1/users/{id}` | PUT | 사용자 수정 |
| | `/api/v1/users/{id}` | DELETE | 사용자 삭제 |
| **크리덴셜** | `/api/v1/credentials` | POST | 크리덴셜 발급 |
| | `/api/v1/credentials/{id}` | GET | 크리덴셜 조회 |
| | `/api/v1/credentials/{id}/suspend` | POST | 크리덴셜 일시 정지 |
| | `/api/v1/credentials/{id}/reactivate` | POST | 크리덴셜 재활성화 |
| | `/api/v1/credentials/{id}` | DELETE | 크리덴셜 폐기 |
| **접근 제어** | `/api/v1/access/grant` | POST | 접근 권한 부여 |
| | `/api/v1/access/revoke` | POST | 접근 권한 철회 |
| | `/api/v1/access/verify` | POST | 크리덴셜 검증 |
| **출입구** | `/api/v1/access-points` | GET | 출입구 목록 |
| | `/api/v1/access-points/{id}` | GET | 출입구 조회 |
| | `/api/v1/access-points/unlock` | POST | 출입구 원격 개방 |
| | `/api/v1/access-points/lock` | POST | 출입구 원격 잠금 |
| **로그** | `/api/v1/access-logs` | GET | 출입 로그 조회 |
| | `/api/v1/access-logs/{id}` | GET | 로그 상세 조회 |
| | `/api/v1/access-logs/export` | POST | 로그 내보내기 |
| **방문자** | `/api/v1/visitors` | POST | 방문자 등록 |
| | `/api/v1/visitors` | GET | 방문자 목록 |
| | `/api/v1/visitors/{id}` | GET | 방문자 조회 |
| | `/api/v1/visitors/{id}/check-in` | POST | 체크인 |
| | `/api/v1/visitors/{id}/check-out` | POST | 체크아웃 |
| **주차** | `/api/v1/parking/vehicles` | POST | 차량 등록 |
| | `/api/v1/parking/entry` | POST | 입차 기록 |
| | `/api/v1/parking/exit` | POST | 출차 기록 |
| | `/api/v1/parking/lots/{id}/status` | GET | 주차장 현황 |
| **비상** | `/api/v1/emergency/activate` | POST | 비상 모드 활성화 |
| | `/api/v1/emergency/deactivate` | POST | 비상 모드 해제 |
| | `/api/v1/emergency/status` | GET | 비상 상태 조회 |
| **모니터링** | `/api/v1/dashboard/{buildingId}` | GET | 실시간 대시보드 |
| | `/api/v1/alerts` | GET | 활성 알람 |
| | `/api/v1/alerts/{id}/acknowledge` | POST | 알람 확인 |

## 보안 요구사항

### 데이터 보안

- **전송 암호화**: TLS 1.3 이상
- **저장 암호화**: AES-256
- **생체 템플릿**: 원본 저장 금지, 해시/암호화된 템플릿만 저장
- **개인정보**: GDPR, 개인정보보호법 준수

### 물리적 보안

- **컨트롤러**: 방화/방수 케이스, 잠금 장치
- **배선**: 은폐 배선, 차단 방지
- **리더기**: 변조 방지 센서 (Tamper Switch)

### 네트워크 보안

- **세그먼테이션**: OT/IT 네트워크 분리
- **방화벽**: 출입 시스템 전용 VLAN
- **침입 탐지**: IDS/IPS
- **접근 제어**: MAC 주소 필터링

## 적용 효과

### 보안 강화

- **무단 침입 방지**: 비인가 출입 차단 99% 이상
- **실시간 모니터링**: 출입 현황 실시간 확인
- **완벽한 감사 추적**: 모든 출입 기록 저장

### 운영 효율

- **무인 보안**: 24/7 자동 출입 관리
- **인력 절감**: 보안 인력 30-50% 감축
- **신속한 대응**: 비상 상황 자동 처리

### 사용자 편의

- **다양한 인증**: 카드, 모바일, 생체인식 선택
- **빠른 인증**: 1초 이내 출입 가능
- **모바일 크리덴셜**: 스마트폰으로 출입

## 용례 (Use Cases)

### 오피스 빌딩

**규모**: 25층, 2,000명 재직

**시스템 구성**:
- 정문: 얼굴 인식 + 카드 리더
- 주차장: 차량 번호 인식
- 엘리베이터: 층별 접근 제어
- 서버실: 카드 + 생체인식 (2-Factor)

**효과**:
- 무인 보안 체계 구축
- 비인가 출입 99% 차단
- 방문자 관리 자동화

### 데이터센터

**보안 등급**: Tier 3

**시스템 구성**:
- Mantrap (이중 도어)
- 카드 + 생체인식 + PIN (3-Factor)
- Anti-Passback
- 24/7 CCTV 녹화

**효과**:
- 물리적 보안 ISO 27001 인증 취득
- 출입 감사 완벽 추적

### 병원

**특수 요구사항**:
- 격리 병동 출입 통제
- 약품 보관소 접근 제어
- 신생아실 보안

**시스템 구성**:
- RFID 팔찌 (환자)
- 카드 + PIN (의료진)
- 비상 개방 (화재 시)

**효과**:
- 환자 안전 향상
- 약품 도난 방지
- 감염 통제

## 성능 기준

| 지표 | 목표 |
|------|------|
| 인증 응답 시간 | ≤ 1초 |
| 도어 개방 시간 | ≤ 2초 (인증 후) |
| 시스템 가용성 | ≥ 99.9% |
| 로그 저장 성공률 | ≥ 99.99% |
| 생체인식 FAR | ≤ 0.01% |
| 생체인식 FRR | ≤ 3% |

## 참조 표준

- ISO/IEC 19794: 생체정보 데이터 교환 형식
- ISO/IEC 24745: 생체정보 템플릿 보호
- ANSI/INCITS 378: 지문 템플릿 형식
- Wiegand Interface Standard
- OSDP (Open Supervised Device Protocol) v2.2
- BACnet (ISO 16484-5): 빌딩 자동화 연동
- GDPR (General Data Protection Regulation)
- 개인정보보호법 (한국)

## 기여 방법

WIA-CITY-015 표준 개선에 기여하실 수 있습니다:

1. **기술 피드백**: 표준 개선 제안
2. **사례 연구**: 출입 통제 시스템 성공 사례 공유
3. **번역**: 다양한 언어로 문서 번역
4. **도구 개발**: 오픈소스 분석 도구 개발
5. **교육**: 교육 자료 및 튜토리얼 제작

## 라이선스

WIA-CITY-015 출입 통제 시스템 표준은 MIT License로 공개됩니다.

자유롭게:
- **공유** - 복사 및 재배포
- **수정** - 리믹스, 변형, 2차 저작물 제작

조건:
- **저작자 표시** - WIA에 적절한 크레딧 제공

## 문의

### 표준 개발

- **기관**: WIA (World Certification Industry Association)
- **발행**: SmileStory Inc.
- **이메일**: standards@wia.org
- **홈페이지**: https://wia.org/standards/city-015

### 기술 지원

- **이메일**: support@wia.org
- **문서**: [기술 스펙](spec/WIA-CITY-015-v1.0.md)
- **커뮤니티**: https://forum.wia.org/access-control

### 인증 문의

- **이메일**: certification@wia.org
- **비용**: [인증 수수료 안내](https://wia.org/certification/fees)
- **신청**: https://wia.org/certification/apply

## 버전 이력

| 버전 | 날짜 | 변경 내용 |
|------|------|----------|
| 1.0.0 | 2025-12-25 | 초판 발행 |

---

**발행 기관**: WIA (World Certification Industry Association)
**라이선스**: MIT License
**문의**: standards@wia.org
**홈페이지**: https://wia.org/standards/city-015

**홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**
