# WIA Cognitive AAC - Phase 1 Summary
# 인지 AAC 표준 Phase 1 완료 요약

---

## 철학
**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

---

## 1. Phase 1 개요

### 1.1 목표
인지장애(자폐 스펙트럼, 치매, 지적장애) 사용자를 위한 AAC 시스템의 **인지 프로파일 표준**을 정의합니다.

### 1.2 완료 일자
2025-01-14

### 1.3 상태
**Phase 1 완료** - Phase 2 (적응형 UI 엔진) 진행 준비 완료

---

## 2. 산출물

### 2.1 연구 보고서

| 문서 | 위치 | 설명 |
|------|------|------|
| **RESEARCH-PHASE-1.md** | `/spec/RESEARCH-PHASE-1.md` | 기존 평가 도구 조사 및 분석 |

### 2.2 표준 명세서

| 문서 | 위치 | 설명 |
|------|------|------|
| **COGNITIVE-PROFILE-SPEC.md** | `/spec/COGNITIVE-PROFILE-SPEC.md` | 핵심 인지 프로파일 표준 |
| **AUTISM-PROFILE-SPEC.md** | `/spec/AUTISM-PROFILE-SPEC.md` | 자폐 스펙트럼 특화 프로파일 |
| **DEMENTIA-PROFILE-SPEC.md** | `/spec/DEMENTIA-PROFILE-SPEC.md` | 치매 특화 프로파일 |
| **ASSESSMENT-METRICS.md** | `/spec/ASSESSMENT-METRICS.md` | AAC 효과 평가 메트릭 |

### 2.3 JSON Schema

| 스키마 | 위치 | 용도 |
|--------|------|------|
| **cognitive-profile.schema.json** | `/schemas/` | 기본 인지 프로파일 검증 |
| **autism-profile.schema.json** | `/schemas/` | 자폐 프로파일 검증 |
| **dementia-profile.schema.json** | `/schemas/` | 치매 프로파일 검증 |

---

## 3. 주요 결과

### 3.1 인지 영역 모델

6개의 핵심 인지 영역을 정의했습니다:

```
CognitiveDomains
├── Memory (기억)
│   ├── 즉시 회상, 작업 기억
│   └── 장기 기억 (일화/의미/절차)
├── Attention (주의)
│   ├── 지속, 선택적, 분할
│   └── 주의 전환
├── Language (언어)
│   ├── 수용, 표현
│   └── 이름대기, 따라말하기
├── Executive (실행 기능)
│   ├── 계획, 억제
│   └── 유연성, 문제 해결
├── VisualSpatial (시공간)
│   └── 지각, 공간 지남력, 구성
└── SocialCognition (사회 인지)
    └── 감정 인식, 마음 이론, 사회적 판단
```

### 3.2 인지 수준 척도

5단계 인지 수준 척도를 정의했습니다:

| 수준 | 값 | 설명 | 권장 AAC 복잡도 |
|------|:--:|------|-----------------|
| PROFOUND | 1 | 심각한 손상 | 2-4 항목 |
| SEVERE | 2 | 중증 손상 | 4-6 항목 |
| MODERATE | 3 | 중등도 손상 | 6-9 항목 |
| MILD | 4 | 경미한 손상 | 9-16 항목 |
| TYPICAL | 5 | 정상 범위 | 16+ 항목 |

### 3.3 자폐 스펙트럼 특화 영역

DSM-5 기반 특화 영역:
- 사회적 의사소통 (공동 주의, 상호성, 비언어적 의사소통)
- 제한적/반복적 행동 (상동행동, 루틴, 특별 관심사)
- 감각 처리 (7개 감각 영역)
- 강점 프로파일 (시각적, 체계적, 기억 강점)

### 3.4 치매 특화 영역

단계 기반 접근:
- 보존된 능력 (절차 기억, 정서, 음악, 원격 기억)
- 손상된 영역 (기억, 언어, 실행 기능, 시공간, 주의)
- 일상생활 기능 (ADL/IADL)
- 행동/심리 증상 (BPSD)
- 개인사/선호 (회상 요소)

### 3.5 AAC 효과 메트릭

7개 메트릭 범주:
1. 의사소통 효율성 (속도, 오류율)
2. 의사소통 효과성 (성공률, 이해도)
3. 언어 발달 (어휘, 구문)
4. 사회적 참여 (상호작용 빈도/품질)
5. 독립성 (촉진 수준, 일반화)
6. 인지 부하 (주관적/행동적 지표)
7. 사용자 만족도 (사용성, 삶의 질)

---

## 4. 기존 도구 연동

### 4.1 조사된 평가 도구

| 도구 | 대상 | 연동 영역 |
|------|------|-----------|
| **ADOS-2** | 자폐 | 사회적 의사소통, 제한적 행동 |
| **Vineland-3** | 적응 행동 | 의사소통, 일상생활, 사회화 |
| **MMSE** | 치매 | 인지 단계 (중등도-중증) |
| **MoCA** | 치매 | 인지 단계 (MCI-경도) |
| **Communication Matrix** | 의사소통 | 발달 단계 (7레벨) |

### 4.2 점수 → 프로파일 매핑

기존 평가 도구 점수를 WIA 인지 프로파일로 변환하는 알고리즘을 명세했습니다.

---

## 5. 윤리/프라이버시

### 5.1 핵심 원칙

- **최소화**: 필요한 정보만 수집
- **익명화**: 직접 식별 정보 제거
- **암호화**: 전송/저장 시 암호화
- **동의**: 명시적 동의 획득
- **접근 제어**: 권한 기반 접근

### 5.2 대리 결정 프로토콜

인지 능력이 저하된 사용자를 위한 동의 프로토콜을 정의했습니다.

---

## 6. 디렉토리 구조

```
cognitive-aac/
├── prompts/
│   ├── MASTER-PROMPT.md
│   ├── PHASE-1-PROMPT.md
│   ├── PHASE-2-PROMPT.md
│   ├── PHASE-3-PROMPT.md
│   └── PHASE-4-PROMPT.md
├── spec/
│   ├── RESEARCH-PHASE-1.md        ← NEW
│   ├── COGNITIVE-PROFILE-SPEC.md  ← NEW
│   ├── AUTISM-PROFILE-SPEC.md     ← NEW
│   ├── DEMENTIA-PROFILE-SPEC.md   ← NEW
│   └── ASSESSMENT-METRICS.md      ← NEW
├── schemas/
│   ├── cognitive-profile.schema.json  ← NEW
│   ├── autism-profile.schema.json     ← NEW
│   └── dementia-profile.schema.json   ← NEW
└── docs/
    └── PHASE-1-SUMMARY.md         ← NEW
```

---

## 7. 다음 단계: Phase 2

### 7.1 Phase 2 목표
**적응형 UI 엔진** - 인지 프로파일에 따라 AAC 인터페이스를 동적으로 적응시키는 시스템

### 7.2 Phase 2 주요 과제

1. **프로파일 → UI 매핑 알고리즘**
   - 인지 수준에 따른 복잡도 조절
   - 자폐/치매 특화 UI 규칙

2. **적응형 레이아웃 시스템**
   - 동적 그리드 조절
   - 상황별 어휘 세트

3. **실시간 적응**
   - 인지 부하 모니터링
   - 자동 단순화/확대

4. **API 설계**
   - TypeScript/Python API
   - REST/WebSocket 인터페이스

### 7.3 Phase 2 시작 방법

```bash
cd cognitive-aac
cat prompts/PHASE-2-PROMPT.md
```

---

## 8. 참고 자료

### 8.1 연구 참고
- ADOS-2: [WPS Publish](https://www.wpspublish.com/ados-2)
- Vineland-3: [Pearson](https://www.pearsonassessments.com/vineland-3)
- MoCA/MMSE: [SimplePractice Guide](https://www.simplepractice.com/blog/moca-vs-mmse/)
- Communication Matrix: [communicationmatrix.org](https://www.communicationmatrix.org/)

### 8.2 표준 참고
- DSM-5: 자폐 스펙트럼 진단 기준
- ISO 21801-1:2020: 인지 접근성
- WCAG 2.2: 웹 콘텐츠 접근성 가이드라인

---

## 9. 기여자

| 역할 | 담당 |
|------|------|
| 표준 설계 | WIA / Claude Code |
| 검토 | SmileStory Inc. |
| 후원 | WIA (World Inclusive Accessibility) |

---

<div align="center">

**WIA Cognitive AAC Standard - Phase 1 Complete**

*"인지를 이해하고, 접근성을 확장하고, 소통을 연결하다"*

**홍익인간** - 널리 인간을 이롭게 하라

**Phase 2로 계속 →**

</div>
