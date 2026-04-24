# Phase 4: Ecosystem Integration
## Claude Code 작업 프롬프트

---

**Standard**: WIA Space
**Phase**: 4 of 4
**목표**: 우주 기술 데이터를 외부 시스템과 연동
**난이도**: ★★★★☆
**예상 작업량**: 스펙 문서 1개 + 출력 모듈 구현 + 예제

---

## 🎯 Phase 4 목표

### 핵심 질문
```
"Phase 1에서 Data Format을 정의하고,
 Phase 2에서 API Interface를 만들고,
 Phase 3에서 Communication Protocol을 정의했다.

 이제 WIA Space 데이터를 외부 시스템과 어떻게 연동할 것인가?

 - NASA GMAT로 궤도 분석?
 - CesiumJS로 3D 시각화?
 - CCSDS 표준 형식으로 내보내기?

 모든 출력 방식에서 동일한 인터페이스를 사용할 수 있을까?"
```

### 목표
```
WIA Space 데이터 → 외부 시스템 연동

출력 경로:
├─ Visualization: 3D 시각화 (CesiumJS, Three.js)
├─ Export: 표준 형식 내보내기 (CCSDS OEM/OPM, JSON, CSV)
├─ Dashboard: 실시간 대시보드 (NASA OpenMCT)
└─ Alert: 알림 시스템 (WebHook, Email)

단일 API로 모든 출력 방식 지원
```

---

## 📋 Phase 1-3 결과물 활용

| 이전 Phase 산출물 | Phase 4 활용 |
|-----------------|-------------|
| Phase 1: Data Format | 내보내기 데이터 소스 |
| Phase 2: Rust API | 데이터 처리 API |
| Phase 3: Protocol | 실시간 데이터 스트리밍 |

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 시각화 도구 조사

| 도구 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **CesiumJS** | 3D 지구/우주 시각화 | "CesiumJS satellite visualization CZML" |
| **Three.js** | WebGL 3D 렌더링 | "Three.js orbit visualization" |
| **NASA OpenMCT** | 미션 제어 대시보드 | "NASA OpenMCT telemetry dashboard" |
| **GMAT** | 궤도 분석 도구 | "NASA GMAT script format" |

### 2단계: 데이터 내보내기 형식 조사

| 형식 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **CCSDS OEM** | 궤도 시계열 형식 | "CCSDS OEM orbit ephemeris message format" |
| **CCSDS OPM** | 궤도 파라미터 형식 | "CCSDS OPM orbit parameter message" |
| **TLE** | 두 줄 요소 세트 | "TLE two-line element format" |
| **CZML** | CesiumJS 데이터 형식 | "CZML format specification" |

### 3단계: 대시보드/모니터링 조사

| 시스템 | 조사 대상 | 웹서치 키워드 |
|--------|----------|--------------|
| **OpenMCT** | 원격측정 대시보드 | "OpenMCT plugin development" |
| **Grafana** | 메트릭 시각화 | "Grafana satellite telemetry" |
| **Prometheus** | 시계열 데이터베이스 | "Prometheus space telemetry" |

### 4단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-4.md`에 다음을 정리:

```markdown
# Phase 4 사전 조사 결과

## 1. 시각화 도구 비교

### CesiumJS
- 기능: [조사 내용]
- CZML 형식: [조사 내용]
- WIA Space 적용: [분석]

### NASA OpenMCT
- 기능: [조사 내용]
- 플러그인 구조: [조사 내용]
- WIA Space 적용: [분석]

## 2. 데이터 내보내기 형식

### CCSDS OEM/OPM
- 형식 구조: [조사 내용]
- 사용 사례: [조사 내용]

### CZML
- 형식 구조: [조사 내용]
- CesiumJS 연동: [조사 내용]

## 3. 결론
- 권장 시각화 방식: [제안]
- 내보내기 형식 설계: [제안]
- 대시보드 연동 설계: [제안]
```

---

## 🏗️ 출력 연동 설계

### 1. 출력 인터페이스 (Output Interface)

#### 기본 출력 인터페이스
```rust
#[async_trait]
pub trait OutputAdapter: Send + Sync {
    /// 출력 유형
    fn output_type(&self) -> OutputType;

    /// 어댑터 이름
    fn name(&self) -> &str;

    /// 초기화
    async fn initialize(&mut self, config: &OutputConfig) -> Result<(), OutputError>;

    /// 출력
    async fn output(&self, data: &OutputData) -> Result<OutputResult, OutputError>;

    /// 사용 가능 여부
    fn is_available(&self) -> bool;

    /// 정리
    async fn dispose(&mut self) -> Result<(), OutputError>;
}

pub enum OutputType {
    Visualization,  // 3D 시각화
    Export,         // 데이터 내보내기
    Dashboard,      // 대시보드
    Alert,          // 알림
    Logger,         // 로깅
    Custom(String), // 사용자 정의
}
```

### 2. 내보내기 어댑터

#### CCSDS OEM Exporter
```rust
pub struct CcsdsOemExporter {
    name: String,
    originator: String,
    object_name: String,
    object_id: String,
}

impl CcsdsOemExporter {
    pub fn export_oem(&self, data: &OutputData) -> Result<String, OutputError>;
}
```

#### CCSDS OPM Exporter
```rust
pub struct CcsdsOpmExporter {
    name: String,
    originator: String,
}

impl CcsdsOpmExporter {
    pub fn export_opm(&self, data: &OutputData) -> Result<String, OutputError>;
}
```

#### CZML Exporter (CesiumJS용)
```rust
pub struct CzmlExporter {
    name: String,
    document_name: String,
}

impl CzmlExporter {
    pub fn export_czml(&self, data: &OutputData) -> Result<String, OutputError>;
}
```

### 3. 통합 출력 매니저

```rust
pub struct OutputManager {
    adapters: HashMap<String, Box<dyn OutputAdapter>>,
}

impl OutputManager {
    /// 어댑터 등록
    pub async fn register(&self, name: &str, adapter: Box<dyn OutputAdapter>);

    /// 특정 어댑터로 출력
    pub async fn output_to(&self, name: &str, data: &OutputData) -> Result<OutputResult, OutputError>;

    /// 모든 어댑터로 브로드캐스트
    pub async fn broadcast(&self, output_type: OutputType, data: &OutputData) -> Vec<Result<OutputResult, OutputError>>;

    /// 활성 어댑터 조회
    pub async fn get_available_by_type(&self, output_type: OutputType) -> Vec<String>;
}
```

---

## 📁 산출물 목록

Phase 4 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-4.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-4-INTEGRATION.md

내용:
1. 개요 (Overview)
2. 출력 계층 아키텍처 (Output Layer Architecture)
3. 출력 인터페이스 (Output Interface)
4. 시각화 연동 (Visualization Integration)
   - CesiumJS (CZML)
   - Three.js
5. 데이터 내보내기 (Data Export)
   - CCSDS OEM/OPM
   - JSON/CSV
   - GMAT Script
6. 대시보드 연동 (Dashboard Integration)
7. 알림 시스템 (Alert System)
8. 통합 출력 매니저 (Output Manager)
9. 에러 처리 (Error Handling)
10. 예제 (Examples)
11. 참고문헌 (References)
```

### 3. Rust 출력 모듈
```
/api/rust/src/
├── output/
│   ├── mod.rs
│   ├── adapter.rs           # 출력 인터페이스
│   ├── manager.rs           # 통합 매니저
│   ├── exporter.rs          # 내보내기 구현
│   │   - CcsdsOemExporter
│   │   - CcsdsOpmExporter
│   │   - JsonExporter
│   │   - CsvExporter
│   │   - GmatScriptExporter
│   │   - CzmlExporter
│   ├── data.rs              # 출력 데이터 타입
│   └── error.rs             # 에러 타입
└── ...
```

### 4. 예제 코드
```
/api/rust/examples/
├── output_demo.rs           # 출력 계층 데모
├── ccsds_export.rs          # CCSDS 내보내기 예제
└── visualization.rs         # 시각화 데이터 생성
```

---

## ✅ 완료 체크리스트

Phase 4 완료 전 확인:

```
□ 웹서치로 시각화/내보내기 기술 조사 완료
□ /spec/RESEARCH-PHASE-4.md 작성 완료
□ /spec/PHASE-4-INTEGRATION.md 작성 완료
□ Rust output 모듈 구현 완료
□ CCSDS OEM/OPM Exporter 구현 완료
□ JSON/CSV Exporter 구현 완료
□ GMAT Script Exporter 구현 완료
□ CZML Exporter (CesiumJS) 구현 완료
□ OutputManager 구현 완료
□ 단위 테스트 작성 완료
□ 테스트 통과
□ 예제 코드 완료
□ README 업데이트 (Phase 4 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 시각화/내보내기 기술 조사
   ↓
2. /spec/RESEARCH-PHASE-4.md 작성
   ↓
3. 출력 인터페이스 설계
   ↓
4. /spec/PHASE-4-INTEGRATION.md 작성
   ↓
5. Rust OutputAdapter trait 정의
   ↓
6. CCSDS OEM/OPM Exporter 구현
   ↓
7. JSON/CSV Exporter 구현
   ↓
8. GMAT Script Exporter 구현
   ↓
9. CZML Exporter 구현
   ↓
10. OutputManager 구현
   ↓
11. 테스트 작성 및 실행
   ↓
12. 예제 코드 작성
   ↓
13. 완료 체크리스트 확인
   ↓
14. WIA Space Standard 완료! 🎉
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1-3 결과물과 연동 가능하도록 설계
✅ 출력 어댑터 추상화 (새로운 출력 방식 쉽게 추가)
✅ 표준 형식 지원 (CCSDS, CZML)
✅ 비동기 처리 (async/await)
✅ Mock 구현으로 테스트 가능하게
✅ 에러 처리 포함
```

### DON'T (하지 말 것)

```
❌ 특정 시각화 도구에만 종속
❌ 실제 외부 서비스 필수 의존 (Mock 필요)
❌ 동기 블로킹 처리
❌ Phase 1-3 형식과 불일치
```

---

## 🔗 WIA Space 전체 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                     우주 기술 데이터                         │
│        (Dyson Sphere, Mars, Warp Drive, Asteroid...)       │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 1: Data Format Standard                   │
│                    표준 JSON 스키마                          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 2: API Interface Standard                 │
│                     Rust API 구현                            │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 3: Communication Protocol                 │
│                 WIA Space Protocol (WSP)                     │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 4: Ecosystem Integration                  │
│                     OutputManager                            │
├─────────────┬─────────────┬─────────────┬───────────────────┤
│ CzmlExporter│ CcsdsExporter│ JsonExporter│ GmatExporter      │
└──────┬──────┴──────┬──────┴──────┬──────┴──────┬────────────┘
       │             │             │             │
       ▼             ▼             ▼             ▼
   ┌───────┐    ┌───────┐    ┌───────┐    ┌───────┐
   │CesiumJS│    │ CCSDS │    │ JSON/ │    │ NASA  │
   │ 시각화 │    │ 표준  │    │  CSV  │    │ GMAT  │
   └───────┘    └───────┘    └───────┘    └───────┘
```

---

## 🚀 작업 시작

이제 Phase 4 작업을 시작하세요.

첫 번째 단계: **웹서치로 시각화 및 내보내기 기술 조사**

```
검색 키워드: "CesiumJS CZML satellite visualization tutorial"
```

화이팅! 🚀

WIA Space Standard의 마지막 Phase입니다.
완료되면 데이터 정의부터 외부 시스템 연동까지 전체 파이프라인이 완성됩니다!

---

<div align="center">

**Phase 4 of 4**

Ecosystem Integration

🎯 최종 목표: 데이터 → 시각화/내보내기/대시보드

弘益人間 - Benefit All Humanity 🌟

</div>
