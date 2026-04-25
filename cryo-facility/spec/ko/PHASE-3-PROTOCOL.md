# WIA-CRYO-FACILITY: PHASE 3 - Protocol 명세서

**버전:** 1.0.0
**상태:** Draft
**날짜:** 2025-12-18
**카테고리:** 극저온 보존 시설 운영
**색상 코드:** #06B6D4 (Cyan)

---

## 1. 소개

### 1.1 목적
본 명세서는 극저온 보존 시설 관리를 위한 운영 프로토콜을 정의하며, 모니터링 절차, 긴급 대응, 유지보수 일정, 규정 준수 워크플로우 및 시설 간 통신을 포함합니다.

### 1.2 프로토콜 카테고리
- 환경 모니터링 프로토콜
- Dewar 유지보수 및 보충 프로토콜
- 긴급 대응 및 사고 관리
- 직원 자격 및 교육 프로토콜
- 보안 및 접근 제어 프로토콜
- 데이터 백업 및 재난 복구
- 시설 간 이송 프로토콜
- 품질 보증 및 규정 준수

### 1.3 프로토콜 준수
WIA-CRYO-FACILITY 표준을 구현하는 모든 시설은 인증 상태를 유지하기 위해 이러한 프로토콜을 준수해야 합니다.

---

## 2. 환경 모니터링 프로토콜 (Environmental Monitoring Protocol)

### 2.1 지속적 모니터링 요구사항

#### 2.1.1 온도 모니터링 (Temperature Monitoring)

**프로토콜 ID:** PROTO-ENV-TEMP-001

**목적:** 모든 중요 구역의 지속적인 온도 모니터링 보장

**모니터링 빈도:**
- Dewar 내부: 1분마다
- 저장실 주변: 5분마다
- 장비실: 10분마다
- 사무실 구역: 30분마다

**구현 단계:**

1. **센서 설치**
   - 중복 온도 센서 설치 (Dewar당 최소 3개)
   - 90일마다 센서 교정
   - 센서 위치 및 일련번호 문서화
   - 센서에 배터리 백업 보장

2. **데이터 수집**
   - 모든 온도 판독값을 중앙 모니터링 시스템에 기록
   - 각 판독값에 UTC 시간으로 타임스탬프 지정
   - 최소 7년간 원시 센서 데이터 저장
   - 이동 평균 계산 (1시간, 24시간)

3. **경보 구성**
   ```json
   {
     "alertRules": [
       {
         "metric": "dewar_temperature",
         "condition": "greater_than",
         "threshold": 80.0,
         "severity": "warning",
         "action": "notify_supervisor"
       },
       {
         "metric": "dewar_temperature",
         "condition": "greater_than",
         "threshold": 85.0,
         "severity": "critical",
         "action": "activate_emergency_protocol"
       },
       {
         "metric": "temperature_sensor_failure",
         "condition": "no_data",
         "duration": 300,
         "severity": "critical",
         "action": "immediate_technician_dispatch"
       }
     ]
   }
   ```

4. **대응 절차**
   - 경고 알림: 5분 이내에 당직 감독자에게 알림
   - 치명적 알림: 즉시 긴급 대응팀 활성화
   - 센서 고장: 30분 이내에 기술자 파견

**규정 준수 체크리스트:**
- [ ] 모든 Dewar에 최소 3개의 기능성 온도 센서 보유
- [ ] 지난 90일 이내에 센서 교정됨
- [ ] 경보 규칙이 구성되고 테스트됨
- [ ] 대응팀 연락처 목록이 최신 상태
- [ ] 백업 모니터링 시스템이 작동 중

#### 2.1.2 액체 질소 레벨 모니터링 (Liquid Nitrogen Level Monitoring)

**프로토콜 ID:** PROTO-ENV-LN2-002

**목적:** 지속적인 레벨 모니터링을 통한 질소 고갈 방지

**모니터링 빈도:**
- 레벨 측정: 15분마다
- 소비율 계산: 시간마다
- 추세 분석: 매일

**구현 단계:**

1. **레벨 센서 설정**
   - 초음파 또는 용량성 레벨 센서 설치
   - 0-100% 범위로 센서 구성
   - 표준 Dewar의 경우 30%에서 임계 임계값 설정
   - 중복 측정 방법 구현

2. **예측 보충 일정**
   ```python
   def calculate_refill_schedule(current_level, consumption_rate, critical_threshold):
       """
       소비 패턴을 기반으로 다음 보충 날짜 계산
       """
       available_volume = current_level - critical_threshold
       days_until_critical = available_volume / consumption_rate
       safety_margin_days = 2

       refill_date = datetime.now() + timedelta(days=(days_until_critical - safety_margin_days))

       return {
           "current_level": current_level,
           "consumption_rate": consumption_rate,
           "days_until_critical": days_until_critical,
           "recommended_refill_date": refill_date,
           "urgency": "high" if days_until_critical < 5 else "normal"
       }
   ```

3. **자동 주문**
   - 레벨이 40%에 도달하면 보충 주문 생성
   - 공급업체 가용성 확인
   - 48시간 리드 타임으로 배송 일정 잡기
   - 긴급 공급업체 연락처 목록 유지

**품질 메트릭:**
- 질소 레벨이 임계 임계값 이하로 떨어진 사례 0건
- 98% 이상의 정시 보충 완료율
- 평균 보충 일정 정확도 1일 이내

#### 2.1.3 압력 모니터링 (Pressure Monitoring)

**프로토콜 ID:** PROTO-ENV-PRESS-003

**목적:** 안전을 보장하기 위해 Dewar 및 시설 압력 모니터링

**모니터링 요구사항:**

| 위치 | 정상 범위 | 경고 임계값 | 치명적 임계값 | 조치 |
|----------|--------------|-------------------|-------------------|--------|
| Dewar 내부 | 0.05-0.25 kPa | 0.30 kPa | 0.40 kPa | 압력 릴리프 활성화 |
| 저장실 | 100-102 kPa | 98 kPa 또는 104 kPa | 95 kPa 또는 106 kPa | HVAC 조정 |
| 장비실 | 100-102 kPa | 98 kPa 또는 104 kPa | 95 kPa 또는 106 kPa | 원인 조사 |

**압력 릴리프 프로토콜:**
```
IF dewar_pressure > critical_threshold THEN
  1. 자동 압력 릴리프 밸브 활성화
  2. 긴급 대응팀에 경고
  3. 즉각적인 구역에서 인원 대피
  4. 압력 하강률 모니터링
  5. 원인 조사 (과충전, 온도 급등, 밸브 고장)
  6. 사고 문서화
END IF
```

### 2.2 환경 데이터 보고

**프로토콜 ID:** PROTO-ENV-REPORT-004

**일일 보고서 생성:**
```json
{
  "reportType": "daily_environmental",
  "facilityId": "CRYO-FAC-A7B3C9D2",
  "reportDate": "2025-12-18",
  "summary": {
    "dewars": {
      "total": 12,
      "normal": 12,
      "warning": 0,
      "critical": 0
    },
    "temperatureMetrics": {
      "dewarAverage": 77.3,
      "dewarMin": 77.1,
      "dewarMax": 77.6,
      "ambientAverage": 22.4
    },
    "nitrogenMetrics": {
      "totalConsumption": 82.5,
      "refillsCompleted": 0,
      "lowLevelAlerts": 0
    },
    "alertsSummary": {
      "totalAlerts": 3,
      "criticalAlerts": 0,
      "warningAlerts": 2,
      "infoAlerts": 1
    }
  },
  "generatedAt": "2025-12-18T23:59:00Z",
  "generatedBy": "automated_reporting_system"
}
```

---

## 3. Dewar 유지보수 프로토콜 (Dewar Maintenance Protocol)

### 3.1 정기 유지보수 일정

**프로토콜 ID:** PROTO-MAINT-ROUTINE-001

#### 3.1.1 일일 유지보수 작업

| 작업 | 빈도 | 할당된 역할 | 소요 시간 | 체크리스트 |
|------|-----------|---------------|----------|-----------|
| 육안 검사 | 매일 (오전 8시) | Cryonics Technician | 15분 | 서리 패턴, 얼음 축적, 눈에 보이는 손상 |
| 레벨 확인 | 매일 (오전 8시, 오후 8시) | Cryonics Technician | 10분 | 센서 vs 딥스틱 측정 비교 |
| 경보 테스트 | 매일 (오전 9시) | Senior Technician | 5분 | 모든 경보 시스템 테스트 |
| 로그 검토 | 매일 (오전 10시) | Supervisor | 20분 | 야간 모니터링 로그 검토 |

**일일 검사 체크리스트:**
```
DEWAR 일일 검사 - 날짜: _______ Dewar ID: _______

육안 검사:
[ ] 비정상적인 서리 패턴 없음
[ ] 외부에 얼음 축적 없음
[ ] 단열재에 눈에 보이는 손상 없음
[ ] 압력 릴리프 밸브 깨끗함
[ ] 비정상적인 소음이나 진동 없음

레벨 확인:
[ ] 센서 판독값: _____%
[ ] 딥스틱 판독값: _____%
[ ] 허용 범위 내 편차 (<2%)
[ ] 소비율 정상

안전 시스템:
[ ] 모든 센서 응답 중
[ ] 경보 시스템 테스트 및 기능 정상
[ ] 백업 전원 사용 가능
[ ] 긴급 연락처 최신 상태

검사자: _____________ 시간: _______ 서명: _____________
```

#### 3.1.2 주간 유지보수 작업

**실행 시간:** 매주 월요일 오전 10:00

1. **Dewar 성능 분석**
   ```python
   def analyze_dewar_performance(dewar_id, days=7):
       """
       지난 주 동안 Dewar 성능 분석
       """
       data = get_monitoring_data(dewar_id, days)

       analysis = {
           "dewarId": dewar_id,
           "analysisPeriod": f"지난 {days}일",
           "metrics": {
               "averageTemperature": calculate_average(data.temperatures),
               "temperatureStability": calculate_std_dev(data.temperatures),
               "averageConsumptionRate": calculate_average(data.consumption),
               "consumptionTrend": calculate_trend(data.consumption),
               "alertsTriggered": count_alerts(data.alerts),
               "uptimePercentage": calculate_uptime(data.status)
           },
           "recommendations": generate_recommendations(data),
           "anomaliesDetected": detect_anomalies(data)
       }

       return analysis
   ```

2. **센서 교정 확인**
   - 중복 센서 간 판독값 비교
   - 1K 이상 편차가 있는 센서 플래그 지정
   - 필요한 경우 재교정 일정 잡기

3. **질소 소비 추세 분석**
   - 7일 이동 평균 계산
   - 과거 기준과 비교
   - 편차가 15% 이상인 경우 조사

#### 3.1.3 월간 유지보수 작업

**실행 시간:** 매월 첫 번째 월요일

| 작업 | 소요 시간 | 요구사항 |
|------|----------|--------------|
| 심층 검사 | 2시간 | 시설 오프라인 일정 |
| 센서 재교정 | 1시간 | 교정 표준 |
| 밸브 테스트 | 30분 | 안전 프로토콜 활성 |
| 단열 무결성 점검 | 1시간 | 열화상 장비 |
| 문서 감사 | 1시간 | 모든 로그 최신 상태 |

### 3.2 질소 보충 프로토콜 (Nitrogen Refill Protocol)

**프로토콜 ID:** PROTO-MAINT-REFILL-002

**표준 운영 절차:**

```
액체 질소 보충 절차

보충 전 체크리스트:
1. Dewar 식별 확인
2. 현재 질소 레벨 확인
3. 필요한 보충 수량 확인
4. 최근 온도 로그 검토
5. PPE 가용성 확인 (단열 장갑, 안면 보호대, 안전 부츠)
6. 공급업체 배송 문서 확인
7. 압력 릴리프 밸브 테스트

보충 실행:
1. 배송 트럭을 지정된 구역에 배치
2. Dewar 및 배송 장비 접지
3. Dewar 충전 포트에 이송 호스 연결
4. Dewar 벤트 밸브 열기
5. 제어된 속도로 질소 이송 시작 (최대 50 L/분)
6. 모니터링:
   - 충전 레벨 (용량의 95%에서 중지)
   - 온도 안정성
   - 압력 레벨
   - 벤트 가스 흐름
7. 이송 완료
8. 호스 분리 (먼저 압력 평형화 허용)
9. 모든 밸브 닫고 고정

보충 후 문서화:
1. 최종 레벨 백분율 기록
2. 추가된 부피 문서화
3. 공급업체 배치 번호 기록
4. 기술자 ID 기록
5. 보충 일정 업데이트
6. 레벨 표시기 사진 촬영
7. 디지털 보충 보고서 제출

품질 확인:
1. 30분 이내에 온도 안정성 확인
2. 비정상적인 압력 판독값 없음 확인
3. 누출 또는 서리 이상 확인
4. 센서 판독값이 시각적 표시기와 일치하는지 확인

기술자: _____________ 감독자: _____________ 날짜: _______
```

**보충 문서화 예제:**
```json
{
  "refillId": "REFILL-2025-1234",
  "dewarId": "DEWAR-BF01XL2025",
  "facilityId": "CRYO-FAC-A7B3C9D2",
  "refillDate": "2025-12-18T08:30:00Z",
  "preFill": {
    "nitrogenLevel": 35.2,
    "temperature": 77.4,
    "pressure": 0.12
  },
  "refillDetails": {
    "volumeAdded": 650,
    "supplier": "AirLiquide",
    "batchNumber": "LN2-2025-12-18-001",
    "deliveryTruck": "TRK-089",
    "transferRate": 45,
    "duration": 14.5
  },
  "postFill": {
    "nitrogenLevel": 97.8,
    "temperature": 77.2,
    "pressure": 0.14
  },
  "performedBy": "STAFF-TC0012ABC",
  "supervisedBy": "STAFF-SV0003XYZ",
  "qualityChecks": {
    "temperatureStable": true,
    "pressureNormal": true,
    "noLeaksDetected": true,
    "sensorValidation": "passed"
  },
  "nextScheduledRefill": "2025-12-25T08:00:00Z",
  "photos": [
    "refill-2025-1234-before.jpg",
    "refill-2025-1234-during.jpg",
    "refill-2025-1234-after.jpg"
  ]
}
```

### 3.3 예방 유지보수 프로토콜 (Preventive Maintenance Protocol)

**프로토콜 ID:** PROTO-MAINT-PREVENT-003

**연간 유지보수 일정:**

| 월 | 활동 | 범위 | 필요한 다운타임 |
|-------|----------|-------|-------------------|
| 1월 | 포괄적 시설 감사 | 모든 시스템 | 2일 |
| 3월 | Dewar 심층 청소 (유닛 1-4) | 부분 | 1일 |
| 5월 | 백업 전원 시스템 테스트 | 전기 | 4시간 |
| 6월 | Dewar 심층 청소 (유닛 5-8) | 부분 | 1일 |
| 7월 | 화재 진압 시스템 테스트 | 안전 | 4시간 |
| 9월 | Dewar 심층 청소 (유닛 9-12) | 부분 | 1일 |
| 10월 | 보안 시스템 업그레이드 | 접근 제어 | 8시간 |
| 12월 | 연말 규정 준수 검토 | 문서화 | 1일 |

**Dewar 심층 청소 절차:**
```
연간 DEWAR 심층 청소 프로토콜

안전 요구사항:
- 시설 감독자 승인 필요
- 대기 중인 긴급 대응팀
- 환자 이송 계획 승인
- 백업 Dewar 용량 확인

청소 전 (1주차):
1. 30일 전에 모든 이해관계자에게 알림
2. 환자 이송 문서 준비
3. 백업 Dewar 가용성 확인
4. 전문 청소팀 일정 잡기
5. 질소 폐기 준비

환자 이송 (2주차):
1. 이송 프로토콜에 따라 백업 Dewar로 환자 이송
2. 새로운 환자 위치 문서화
3. 시설 기록 업데이트
4. 모든 환자가 설명되는지 확인

청소 프로세스 (2주차):
1. 질소 완전 제거
2. Dewar가 실온으로 따뜻해지도록 허용 (3-5일)
3. 내시경으로 내부 검사
4. 청소 및 살균
5. 구조 무결성 평가
6. 단열 검사 및 수리
7. 센서 교체 또는 재교정

청소 후 (3주차):
1. Dewar를 작동 온도로 재냉각
2. 액체 질소 보충
3. 시스템 테스트 및 검증
4. 환자를 원래 위치로 다시 이송
5. 48시간 모니터링 기간
6. 최종 문서화 및 인증

품질 보증:
- 모든 단계의 사진 문서화
- 제3자 검사 보고서
- 센서 교정 인증서
- 구조 무결성 인증
```

---

## 4. 긴급 대응 프로토콜 (Emergency Response Protocol)

### 4.1 긴급 분류 시스템 (Emergency Classification System)

**프로토콜 ID:** PROTO-EMERG-CLASS-001

| 레벨 | 분류 | 대응 시간 | 팀 규모 | 권한 |
|-------|----------------|---------------|-----------|-----------|
| 1 | 정보성 | 24시간 | 1 | Technician |
| 2 | 경미한 사고 | 4시간 | 2-3 | Supervisor |
| 3 | 주요 사고 | 1시간 | 4-6 | Facility Manager |
| 4 | 치명적 긴급상황 | 15분 | 전체 팀 | Emergency Coordinator |
| 5 | 재난급 | 즉시 | 모든 인원 + 외부 | Executive Director |

### 4.2 치명적 온도 긴급상황 (Critical Temperature Emergency)

**프로토콜 ID:** PROTO-EMERG-TEMP-002

**트리거 조건:**
- Dewar 온도가 85K 초과
- 여러 온도 센서 고장
- 급격한 온도 상승 (시간당 >1K)

**대응 절차:**

```
치명적 온도 긴급상황 대응

즉각적 조치 (0-5분):
1. 긴급 경보 활성화
   - 모든 시설 경보 발동
   - 긴급 대응팀에 알림
   - 시설 관리자 및 이사에게 경고
   - 긴급 질소 공급업체 연락

2. Dewar 평가
   - 영향받은 Dewar 식별
   - 환자 수 결정
   - 온도 변화율 평가
   - 질소 레벨 평가

3. 초기 안정화
   - 백업 냉각 시스템 활성화
   - 레벨이 낮은 경우 긴급 질소 충전 시작
   - 주변 실내 온도 감소
   - 인접 Dewar 모니터링

단기 조치 (5-30분):
4. 근본 원인 분석
   - 질소 공급 확인
   - 센서 정확도 확인
   - 단열 무결성 검사
   - 기계적 고장 평가

5. 격리 조치
   - 영향받은 장비 격리
   - 환자 이송 장비 준비
   - 백업 Dewar 준비
   - 이송팀 브리핑

6. 커뮤니케이션
   - 환자의 법적 대리인에게 알림
   - 보험 제공자 연락
   - 사고 보고서 준비
   - 필요한 경우 규제 기관에 경고

중기 조치 (30분 - 4시간):
7. 환자 이송 결정
   IF 온도 계속 상승:
     - 환자 이송 프로토콜 시작
     - 모든 환자 이동 문서화
     - 백업 Dewar 준비 상태 확인
   ELSE:
     - 강화된 모니터링 지속
     - 중복 냉각 구현

8. 장비 수리
   - 고장 지점 진단
   - 교체 부품 소싱
   - 수리 기술자 일정 잡기
   - 임시 수정 구현

장기 조치 (4시간 이상):
9. 시스템 복원
   - 수리 완료
   - 온도 안정성 확인
   - 정상 운영으로 복귀
   - 이동한 경우 환자 다시 이송

10. 사고 후 검토
    - 사고 보고서 작성
    - 대응 효과 분석
    - 필요한 경우 프로토콜 업데이트
    - 직원 디브리핑
```

**의사결정 트리:**
```
온도 > 85K 감지됨
    |
    ├─> 질소 레벨 < 30%
    |   └─> 즉시 보충
    |
    ├─> 질소 레벨 정상
    |   ├─> 단열 고장 의심
    |   |   └─> 환자 이송 + 수리
    |   |
    |   └─> 센서 오작동 의심
    |       └─> 백업 센서로 확인
    |
    └─> 여러 시스템 고장
        └─> 시설 전체 긴급상황
            └─> 재난 복구 활성화
```

### 4.3 정전 프로토콜 (Power Outage Protocol)

**프로토콜 ID:** PROTO-EMERG-POWER-003

**대응 타임라인:**

| 시간 | 조치 | 담당자 |
|------|--------|-------------------|
| 0분 | 백업 발전기 자동 시작 | 자동 시스템 |
| 1분 | 백업 전원 활성 확인 | 당직 기술자 |
| 2분 | 긴급 조정자에게 알림 | 모니터링 시스템 |
| 5분 | 정전 범위 및 기간 평가 | Facility Manager |
| 10분 | 전력 회사 연락 | 운영 직원 |
| 15분 | 연료 레벨 확인 | 유지보수 직원 |
| 30분 | 모든 직원에게 상태 업데이트 | Emergency Coordinator |

**백업 전원 시스템:**

```json
{
  "powerSystems": {
    "primary": {
      "source": "grid_utility",
      "capacity": "무제한",
      "status": "active"
    },
    "backup": {
      "source": "diesel_generator",
      "capacity": "500 kVA",
      "runtime": "전체 부하에서 72시간",
      "fuelCapacity": "2000 리터",
      "autoStartDelay": "10초",
      "maintenanceSchedule": "월간"
    },
    "emergency": {
      "source": "battery_ups",
      "capacity": "100 kVA",
      "runtime": "4시간",
      "criticalSystemsOnly": true,
      "testSchedule": "주간"
    }
  },
  "criticalLoadPriority": [
    "dewar_monitoring_systems",
    "temperature_sensors",
    "alarm_systems",
    "security_systems",
    "communication_systems",
    "emergency_lighting",
    "hvac_systems",
    "office_systems"
  ]
}
```

### 4.4 질소 공급 중단 (Nitrogen Supply Disruption)

**프로토콜 ID:** PROTO-EMERG-N2-004

**시나리오:** 주요 질소 공급업체가 배송 불가

**대응 조치:**

1. **즉각적 평가 (0-15분)**
   ```python
   def assess_nitrogen_emergency(facility_id):
       """
       치명적 레벨까지의 시간 계산
       """
       dewars = get_facility_dewars(facility_id)

       assessment = {
           "facilityId": facility_id,
           "assessmentTime": datetime.now(),
           "dewarStatus": []
       }

       for dewar in dewars:
           current_level = dewar.nitrogen_level
           consumption_rate = dewar.consumption_rate
           critical_level = dewar.critical_threshold

           hours_to_critical = ((current_level - critical_level) / consumption_rate) * 24

           assessment["dewarStatus"].append({
               "dewarId": dewar.id,
               "currentLevel": current_level,
               "hoursToCritical": hours_to_critical,
               "urgency": "critical" if hours_to_critical < 24 else "warning"
           })

       # 긴급도별 정렬
       assessment["dewarStatus"].sort(key=lambda x: x["hoursToCritical"])
       assessment["mostUrgent"] = assessment["dewarStatus"][0]

       return assessment
   ```

2. **공급업체 활성화 (15-30분)**
   - 백업 공급업체 #1 연락
   - 백업 공급업체 #2 연락
   - 공급업체 네트워크 조정자 연락
   - 긴급 배송 요청

3. **절약 조치 (30분 이후)**
   - 시설 주변 온도 감소
   - Dewar 접근 최소화
   - 안전한 경우 환자 통합
   - 소비율 면밀히 모니터링

4. **긴급 조달 (병렬)**
   - 프리미엄 가격으로 긴급 구매 승인
   - 필요한 경우 대체 운송 준비
   - 임시 공급을 위해 다른 시설과 조정

**백업 공급업체 연락처 목록:**
```json
{
  "suppliers": [
    {
      "priority": 1,
      "name": "AirLiquide Emergency Services",
      "phone": "+1-800-555-7890",
      "email": "emergency@airliquide.com",
      "deliveryCapability": "24/7",
      "minimumOrder": "500 리터",
      "averageResponseTime": "4시간"
    },
    {
      "priority": 2,
      "name": "Praxair Cryogenics",
      "phone": "+1-800-555-7891",
      "email": "emergency@praxair.com",
      "deliveryCapability": "24/7",
      "minimumOrder": "300 리터",
      "averageResponseTime": "6시간"
    },
    {
      "priority": 3,
      "name": "Matheson Tri-Gas",
      "phone": "+1-800-555-7892",
      "email": "service@matheson-trigas.com",
      "deliveryCapability": "업무 시간 + 대기",
      "minimumOrder": "250 리터",
      "averageResponseTime": "8시간"
    }
  ]
}
```

---

## 5. 직원 자격 프로토콜 (Staff Qualification Protocol)

### 5.1 신입 직원 온보딩 (New Employee Onboarding)

**프로토콜 ID:** PROTO-STAFF-ONBOARD-001

**1단계: 오리엔테이션 (1주차)**

| 일 | 활동 | 소요 시간 | 산출물 |
|-----|----------|----------|-------------|
| 1 | 시설 투어, 안전 브리핑 | 4시간 | 안전 확인서 서명 |
| 2 | 극저온 보존 원리 소개 | 8시간 | 이해도 퀴즈 (>80%) |
| 3 | 장비 숙지 | 8시간 | 장비 체크리스트 완료 |
| 4 | 모니터링 시스템 교육 | 8시간 | 시스템 액세스 자격 증명 |
| 5 | 선임 기술자 섀도잉 | 8시간 | 일일 관찰 로그 |

**2단계: 기술 교육 (2-4주차)**

```
2주차: Dewar 운영
- 질소 보충 절차 (감독 하에)
- 온도 모니터링 시스템
- 경보 대응 프로토콜
- 일일 유지보수 작업

3주차: 긴급 절차
- 긴급 대응 교육
- 응급 처치 및 CPR 인증
- 화재 안전 및 대피
- 사고 보고 절차

4주차: 전문 시스템
- 환경 모니터링 시스템
- 접근 제어 및 보안
- 데이터 관리 및 문서화
- 품질 보증 절차
```

**3단계: 인증 (5-6주차)**

1. 필기 시험 (100문제, 최소 90% 필요)
2. 실기 평가
3. 긴급 시나리오 시뮬레이션
4. 시설 관리자와 최종 검토
5. 인증서 발급

**지속적 요구사항:**
- 연간 40시간 계속 교육
- 분기별 긴급 훈련
- 연간 재인증
- 필요에 따라 전문 교육

### 5.2 교육 기록 프로토콜 (Training Records Protocol)

**프로토콜 ID:** PROTO-STAFF-TRAINING-002

**교육 문서화 요구사항:**

```json
{
  "trainingRecord": {
    "trainingId": "TRAIN-2025-0234",
    "staffId": "STAFF-TC0034DEF",
    "trainingDetails": {
      "trainingName": "Advanced Dewar Maintenance 2025",
      "trainingType": "advanced",
      "category": "technical_skills",
      "provider": "WIA Cryonics Training Institute",
      "format": "in_person"
    },
    "schedule": {
      "startDate": "2025-12-10T09:00:00Z",
      "endDate": "2025-12-15T17:00:00Z",
      "totalHours": 40,
      "attendanceHours": 40
    },
    "assessment": {
      "writtenExam": {
        "score": 92,
        "passingScore": 80,
        "attemptNumber": 1
      },
      "practicalExam": {
        "score": 95,
        "passingScore": 85,
        "evaluator": "Dr. Emily Chen"
      },
      "overallResult": "pass"
    },
    "certification": {
      "certificateNumber": "CERT-ADV-DEWAR-2025-0234",
      "issueDate": "2025-12-15T17:00:00Z",
      "expiryDate": "2027-12-15T17:00:00Z",
      "certificateUrl": "https://certificates.cryo-facility.wia.org/CERT-ADV-DEWAR-2025-0234.pdf"
    },
    "competenciesGained": [
      "고급 질소 시스템 진단",
      "Dewar 이송 절차",
      "단열 수리 기술",
      "긴급 냉각 시스템 활성화"
    ]
  }
}
```

---

## 6. 보안 및 접근 제어 프로토콜 (Security and Access Control Protocol)

### 6.1 물리적 접근 제어 (Physical Access Control)

**프로토콜 ID:** PROTO-SEC-ACCESS-001

**접근 레벨:**

| 레벨 | 직함 | 승인된 구역 | 요구사항 |
|-------|-------|------------------|--------------|
| 0 | 방문자 | 로비, 회의실 | 에스코트 필요 |
| 1 | 행정 직원 | 사무실, 휴게실 | 배지 + PIN |
| 2 | 초급 기술자 | 위 + 모니터링 센터 | 배지 + PIN + 생체인식 |
| 3 | 선임 기술자 | 위 + Dewar실 | 배지 + PIN + 생체인식 + 교육 인증 |
| 4 | 감독자 | 위 + 장비실 | 배지 + PIN + 생체인식 + 감독자 인증 |
| 5 | Facility Manager | 모든 구역 | 배지 + PIN + 생체인식 + 관리 인증 |

**접근 제어 구현:**

```python
class AccessControlSystem:
    def __init__(self):
        self.access_rules = self.load_access_rules()
        self.audit_log = []

    def verify_access(self, staff_id, area_id, access_method):
        """
        직원이 요청된 구역에 접근 권한이 있는지 확인
        """
        staff = self.get_staff_profile(staff_id)
        area = self.get_area_profile(area_id)

        # 접근 레벨 확인
        if staff.access_level < area.required_level:
            self.log_access_denied(staff_id, area_id, "insufficient_access_level")
            return False

        # 시간 제한 확인
        if area.time_restricted and not self.check_time_window(area.allowed_hours):
            self.log_access_denied(staff_id, area_id, "outside_allowed_hours")
            return False

        # 다중 인증 확인
        if area.requires_biometric and access_method != "biometric":
            self.log_access_denied(staff_id, area_id, "biometric_required")
            return False

        # 활성 인증 확인
        if area.requires_certification:
            if not self.verify_certification(staff.certifications, area.required_certs):
                self.log_access_denied(staff_id, area_id, "certification_expired")
                return False

        # 접근 허용
        self.log_access_granted(staff_id, area_id, access_method)
        return True

    def log_access_event(self, staff_id, area_id, action, reason=None):
        """
        감사 추적을 위해 모든 접근 이벤트 기록
        """
        event = {
            "timestamp": datetime.now(),
            "staffId": staff_id,
            "areaId": area_id,
            "action": action,
            "reason": reason,
            "ipAddress": self.get_request_ip(),
            "deviceId": self.get_device_id()
        }

        self.audit_log.append(event)
        self.store_audit_event(event)
```

---

## 7. 버전 이력

| 버전 | 날짜 | 작성자 | 변경사항 |
|---------|------|--------|---------|
| 1.0.0 | 2025-12-18 | WIA 표준위원회 | 초기 프로토콜 명세서 |

---

## 8. 참조

- NFPA 55: Compressed Gases and Cryogenic Fluids Code
- OSHA 29 CFR 1910: Occupational Safety and Health Standards
- ISO 9001:2015: Quality Management Systems
- ISO 45001:2018: Occupational Health and Safety Management
- ANSI Z535: Safety Signs and Tags Standards

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
