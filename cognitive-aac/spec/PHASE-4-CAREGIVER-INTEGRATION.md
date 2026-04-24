# WIA Cognitive AAC - Phase 4: Caregiver Integration Specification

## 케어기버/전문가 통합 시스템 명세

**홍익인간 (弘益人間) - 널리 인간을 이롭게 하라**

---

## 1. 개요

### 1.1 목적

인지 장애 AAC 사용자를 지원하는 케어기버, 전문가, 가족 간의 원활한 협업을 위한 통합 시스템.

### 1.2 핵심 원칙

1. **사용자 중심**: 모든 기능은 AAC 사용자의 이익을 위해
2. **프라이버시 우선**: 명시적 동의 기반 데이터 공유
3. **접근성**: 다양한 역할에 맞는 인터페이스
4. **규정 준수**: HIPAA, GDPR 완전 준수

---

## 2. 시스템 아키텍처

### 2.1 전체 구조

```
┌─────────────────────────────────────────────────────────────┐
│                      클라이언트 앱들                        │
├─────────────┬─────────────┬─────────────┬──────────────────┤
│  AAC 앱     │ 케어기버 앱 │ 전문가 웹   │ 가족 포털        │
│  (사용자)   │ (모바일)    │ (대시보드)  │ (웹/모바일)      │
└──────┬──────┴──────┬──────┴──────┬──────┴────────┬─────────┘
       │             │             │               │
       └─────────────┴──────┬──────┴───────────────┘
                            │
                   ┌────────▼────────┐
                   │   API Gateway   │
                   │  (인증/라우팅)  │
                   └────────┬────────┘
                            │
       ┌────────────────────┼────────────────────┐
       │                    │                    │
┌──────▼──────┐     ┌───────▼──────┐     ┌──────▼──────┐
│  REST API   │     │  WebSocket   │     │  Compliance │
│  (CRUD)     │     │  (실시간)    │     │  (감사/동의)│
└──────┬──────┘     └───────┬──────┘     └──────┬──────┘
       │                    │                    │
       └────────────────────┼────────────────────┘
                            │
                   ┌────────▼────────┐
                   │    Database     │
                   │  (암호화 저장)  │
                   └─────────────────┘
```

### 2.2 마이크로서비스 구성

| 서비스 | 역할 |
|--------|------|
| **CaregiverService** | 케어기버 대시보드, 모니터링 |
| **ProfessionalService** | 전문가 평가, 치료 계획 |
| **FamilyService** | 가족 참여, 회상 기능 |
| **ComplianceService** | HIPAA/GDPR 준수 |
| **RealtimeService** | WebSocket 실시간 통신 |

---

## 3. API 명세

### 3.1 케어기버 API

```typescript
// 실시간 모니터링
GET  /api/v1/caregiver/clients/:clientId/live
Response: LiveMonitoringData

// 일간 요약
GET  /api/v1/caregiver/clients/:clientId/daily-summary
Query: { date?: string }
Response: DailySummary

// 주간 진행상황
GET  /api/v1/caregiver/clients/:clientId/weekly-progress
Query: { weekStart?: string }
Response: WeeklyProgress

// 활동 기록
POST /api/v1/caregiver/clients/:clientId/activities
Body: { type, data }
Response: Activity

// UI 원격 조정
POST /api/v1/caregiver/clients/:clientId/adjust-ui
Body: UIConfiguration
Response: { success, message }

// 기기 잠금
POST /api/v1/caregiver/clients/:clientId/lock
Body: { locked: boolean }
Response: { success, locked }
```

### 3.2 전문가 API

```typescript
// 클라이언트 목록
GET  /api/v1/professional/clients
Response: ClientSummary[]

// 평가 실시
POST /api/v1/professional/clients/:clientId/assessments
Body: { type, scores, notes }
Response: AssessmentResult

// 목표 설정
POST /api/v1/professional/clients/:clientId/goals
Body: Goal
Response: Goal

// 목표 진행 업데이트
PATCH /api/v1/professional/goals/:goalId/progress
Body: { progress: number }
Response: GoalProgress

// 임상 보고서 생성
POST /api/v1/professional/clients/:clientId/reports
Body: { type, period }
Response: ClinicalReport
```

### 3.3 가족 API

```typescript
// 사진 업로드
POST /api/v1/family/clients/:clientId/photos
Body: Photo
Response: Photo

// 메시지 전송
POST /api/v1/family/clients/:clientId/messages
Body: { content, type, senderName }
Response: FamilyMessage

// 인생 이야기 추가
POST /api/v1/family/clients/:clientId/life-story
Body: LifeStoryEntry
Response: LifeStoryEntry

// 회상 세션 시작
POST /api/v1/family/clients/:clientId/reminiscence
Body: { topic: string }
Response: ReminiscenceSession
```

### 3.4 규정 준수 API

```typescript
// 동의 기록
POST /api/v1/compliance/consents
Body: ConsentRecord
Response: ConsentRecord

// GDPR 데이터 접근
GET  /api/v1/compliance/gdpr/access
Response: PersonalData

// GDPR 데이터 삭제
DELETE /api/v1/compliance/gdpr/erasure
Response: { success, message }

// 규정 준수 보고서
GET  /api/v1/compliance/reports
Query: { type, startDate, endDate }
Response: ComplianceReport
```

---

## 4. 실시간 통신

### 4.1 WebSocket 프로토콜

```typescript
// 연결
ws://api.wia-aac.org/ws?clientId=xxx&token=yyy

// 메시지 유형
type WSMessageType =
  | 'activity'        // 활동 업데이트
  | 'mood_change'     // 기분 변화
  | 'alert'           // 알림
  | 'config_change'   // 설정 변경
  | 'message'         // 메시지
  | 'session_start'   // 세션 시작
  | 'session_end';    // 세션 종료

// 메시지 형식
interface WebSocketMessage {
  type: WSMessageType;
  clientId: string;
  timestamp: Date;
  payload: unknown;
}
```

### 4.2 이벤트 흐름

```
AAC 앱                  서버                케어기버 앱
  │                      │                      │
  │ ── activity ──────► │                      │
  │                      │ ── broadcast ──────►│
  │                      │                      │ (화면 업데이트)
  │                      │                      │
  │ ── mood_change ───► │                      │
  │                      │ ── broadcast ──────►│
  │                      │                      │ (알림 표시)
```

---

## 5. 데이터 모델

### 5.1 핵심 엔티티

```typescript
// 클라이언트 (AAC 사용자)
interface Client {
  id: string;
  name: string;
  type: 'autism' | 'dementia' | 'general';
  dateOfBirth?: Date;
  profileId: string;
  caregivers: CaregiverLink[];
  professionals: ProfessionalLink[];
  family: FamilyLink[];
}

// 활동
interface Activity {
  id: string;
  clientId: string;
  type: 'communication' | 'navigation' | 'selection';
  timestamp: Date;
  data: ActivityData;
}

// 목표
interface Goal {
  id: string;
  clientId: string;
  title: string;
  category: GoalCategory;
  status: GoalStatus;
  progress: number;
  milestones: Milestone[];
}
```

### 5.2 관계도

```
┌──────────┐       ┌──────────────┐       ┌──────────┐
│  Client  │──1:N──│  Activity    │       │   Goal   │
└────┬─────┘       └──────────────┘       └────┬─────┘
     │                                         │
     │ 1:N                                     │ 1:N
     ▼                                         ▼
┌──────────┐       ┌──────────────┐       ┌──────────┐
│ Caregiver│       │ Professional │       │Milestone │
│   Link   │       │    Link      │       │          │
└──────────┘       └──────────────┘       └──────────┘
```

---

## 6. 보안 및 규정 준수

### 6.1 인증/인가

| 항목 | 구현 |
|------|------|
| **인증** | JWT + Refresh Token |
| **인가** | RBAC (역할 기반 접근 제어) |
| **세션** | 30분 타임아웃, 재인증 |
| **MFA** | 전문가/관리자 필수 |

### 6.2 역할별 권한

| 역할 | 활동 보기 | UI 조정 | 평가 | 데이터 내보내기 |
|------|----------|--------|------|----------------|
| 케어기버 | ✅ | ✅ | ❌ | ❌ |
| 전문가 | ✅ | ✅ | ✅ | ✅ |
| 가족 | 제한적 | ❌ | ❌ | ❌ |
| 관리자 | ✅ | ✅ | ✅ | ✅ |

### 6.3 HIPAA 준수

| 요구사항 | 구현 |
|----------|------|
| PHI 암호화 | AES-256 (저장), TLS 1.3 (전송) |
| 접근 제어 | 역할 기반 + 동의 기반 |
| 감사 추적 | 모든 PHI 접근 로깅 |
| 위반 통지 | 자동 감지 및 알림 |

### 6.4 GDPR 준수

| 권리 | API |
|------|-----|
| 접근권 | `GET /gdpr/access` |
| 정정권 | `PATCH /gdpr/rectification` |
| 삭제권 | `DELETE /gdpr/erasure` |
| 이동권 | `POST /gdpr/portability` |

---

## 7. 알림 시스템

### 7.1 알림 유형

| 유형 | 트리거 | 수신자 |
|------|--------|--------|
| **긴급** | 도움 요청, 응급 | 모든 케어기버 |
| **경고** | 비활동, 기분 급변 | 주 케어기버 |
| **정보** | 세션 시작/종료 | 구독자 |
| **일정** | 예약된 알림 | 지정 수신자 |

### 7.2 전달 채널

```
┌─────────────────────────────────────────┐
│              알림 서비스                │
├─────────────────────────────────────────┤
│                                         │
│  ┌─────────┐  ┌────────┐  ┌─────────┐  │
│  │  푸시   │  │  SMS   │  │ 이메일  │  │
│  │ (FCM)  │  │(Twilio)│  │(SendGrid)│ │
│  └─────────┘  └────────┘  └─────────┘  │
│                                         │
│  우선순위: 긴급 → 모든 채널             │
│           경고 → 푸시 + SMS             │
│           정보 → 푸시만                 │
└─────────────────────────────────────────┘
```

---

## 8. 배포 구성

### 8.1 인프라

```yaml
# Docker Compose (개발)
services:
  api:
    image: wia-cognitive-aac-api
    ports:
      - "3001:3001"
    environment:
      - NODE_ENV=development

  database:
    image: postgres:15
    volumes:
      - pgdata:/var/lib/postgresql/data

  redis:
    image: redis:7
```

### 8.2 확장성

| 컴포넌트 | 확장 전략 |
|----------|----------|
| API 서버 | 수평 확장 (로드 밸런서) |
| WebSocket | Sticky 세션 + Redis Pub/Sub |
| 데이터베이스 | 읽기 복제본 |
| 파일 저장소 | S3 호환 스토리지 |

---

## 9. 구현 체크리스트

### 9.1 백엔드 API

- [x] 타입 정의 (`types/index.ts`)
- [x] 케어기버 서비스 (`CaregiverService.ts`)
- [x] 전문가 서비스 (`ProfessionalService.ts`)
- [x] 가족 서비스 (`FamilyService.ts`)
- [x] 규정 준수 서비스 (`ComplianceService.ts`)
- [x] API 라우트 (`routes/index.ts`)
- [x] WebSocket 서버 (`index.ts`)

### 9.2 문서화

- [x] 케어기버 가이드 (`CAREGIVER-GUIDE.md`)
- [x] 전문가 가이드 (`PROFESSIONAL-GUIDE.md`)
- [x] Phase 4 명세 (이 문서)

### 9.3 향후 구현

- [ ] 프론트엔드 앱 (React Native / React)
- [ ] 푸시 알림 서비스
- [ ] 데이터베이스 마이그레이션
- [ ] E2E 테스트

---

## 10. 결론

Phase 4는 WIA Cognitive AAC 표준의 마지막 단계로, 사용자-케어기버-전문가-가족 간의 연결을 완성합니다.

**핵심 가치:**

> "인지 능력과 관계없이 모든 사람이 사랑하는 사람들과 연결될 수 있도록"

---

*WIA Cognitive AAC Phase 4 Specification v1.0*
*Last Updated: 2024*

**홍익인간 (弘益人間) - 널리 인간을 이롭게 하라** 🌏
