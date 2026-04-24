# Phase 4: WIA Ecosystem Integration
## Claude Code 작업 프롬프트

---

**Standard**: WIA Climate
**Phase**: 4 of 4
**목표**: Climate 데이터를 WIA 생태계 및 외부 서비스와 연동
**난이도**: ★★★★☆
**예상 작업량**: 스펙 문서 1개 + 연동 모듈 구현 + 예제

---

## 🎯 Phase 4 목표

### 핵심 질문
```
"Phase 1에서 Data Format을 정의하고,
 Phase 2에서 API Interface를 만들고,
 Phase 3에서 Communication Protocol을 정의했다.

 이제 Climate 데이터를 어디로 보내고 어떻게 활용할 것인가?

 - 대시보드로? (WIA Earth/Grafana)
 - 클라우드 저장소로? (ESGF/S3)
 - 알림 서비스로? (Webhook/SMS)
 - 다른 WIA 표준으로? (상호 연동)

 모든 연동에서 동일한 인터페이스를 사용할 수 있을까?"
```

### 목표
```
Climate 센서 데이터 → 처리 → WIA 생태계 연동

출력 경로:
├─ Dashboard: 데이터 → 시각화 (Grafana, WIA Earth)
├─ Storage: 데이터 → 저장 (S3, ESGF, NetCDF)
├─ Alert: 데이터 → 알림 (Webhook, Email, SMS)
└─ API: 데이터 → 공개 API (REST, GraphQL)

단일 API로 모든 연동 방식 지원
```

---

## 📋 Phase 1-3 결과물 활용

| 이전 Phase 산출물 | Phase 4 활용 |
|-----------------|-------------|
| Phase 1: Data Format | 표준 JSON 데이터 포맷 |
| Phase 2: Rust API | 데이터 생성/처리 API |
| Phase 3: Protocol | WebSocket/MQTT 전송 |
| Phase 4 | 출력 연동 및 생태계 통합 |

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 시각화 플랫폼 조사

| 서비스 | 조사 대상 | 웹서치 키워드 |
|-------|----------|--------------|
| **Grafana** | 시계열 대시보드 | "Grafana data source plugin development" |
| **Apache ECharts** | 차트 라이브러리 | "ECharts climate data visualization" |
| **Cesium/Mapbox** | 지리공간 시각화 | "Cesium JS climate data visualization" |
| **Streamlit** | Python 대시보드 | "Streamlit real-time dashboard" |

### 2단계: 데이터 저장소 조사

| 기술 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **InfluxDB** | 시계열 DB | "InfluxDB climate sensor data" |
| **TimescaleDB** | PostgreSQL 확장 | "TimescaleDB IoT time series" |
| **AWS S3** | 객체 스토리지 | "AWS S3 climate data storage" |
| **NetCDF** | 과학 데이터 형식 | "NetCDF Python write climate data" |

### 3단계: 알림/통합 서비스 조사

| 기술 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **Webhook** | HTTP 콜백 | "Webhook notification service best practices" |
| **Slack/Discord** | 메시징 통합 | "Slack incoming webhook API" |
| **Twilio** | SMS 알림 | "Twilio SMS API alerts" |
| **PagerDuty** | 온콜 알림 | "PagerDuty API integration" |

### 4단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-4.md`에 다음을 정리:

```markdown
# Phase 4 사전 조사 결과

## 1. 시각화 플랫폼

### Grafana
- 장점: [조사 내용]
- 단점: [조사 내용]
- Climate 적용: [분석]

### 지리공간 시각화
- Cesium/Mapbox 현황: [조사 내용]
- 적용 방향: [제안]

## 2. 데이터 저장소

### 시계열 데이터베이스
- InfluxDB vs TimescaleDB: [비교]
- 권장 옵션: [제안]

### 클라우드 스토리지
- S3/GCS 연동: [조사 내용]
- NetCDF 변환: [방법]

## 3. 알림 서비스

### Webhook 패턴
- 설계: [조사 내용]
- 보안: [고려사항]

### 메시징 통합
- Slack/Discord: [적용 방법]

## 4. 결론
- 권장 시각화 방식: [제안]
- 권장 저장소: [제안]
- 알림 설계: [제안]
```

---

## 🏗️ 연동 인터페이스 설계

### 1. 출력 인터페이스 (Output Interface)

#### Rust 기본 출력 인터페이스
```rust
use async_trait::async_trait;
use crate::ClimateMessage;
use crate::error::Result;

/// Output adapter trait for ecosystem integration
#[async_trait]
pub trait OutputAdapter: Send + Sync {
    /// Adapter type identifier
    fn adapter_type(&self) -> OutputType;

    /// Adapter name
    fn name(&self) -> &str;

    /// Initialize the adapter
    async fn initialize(&mut self, options: &OutputOptions) -> Result<()>;

    /// Output climate data
    async fn output(&self, message: &ClimateMessage) -> Result<()>;

    /// Output batch of climate data
    async fn output_batch(&self, messages: &[ClimateMessage]) -> Result<()>;

    /// Check if adapter is available
    fn is_available(&self) -> bool;

    /// Cleanup resources
    async fn dispose(&mut self) -> Result<()>;
}

/// Output adapter types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputType {
    /// Dashboard/Visualization
    Dashboard,
    /// Data storage
    Storage,
    /// Alert/Notification
    Alert,
    /// External API
    Api,
    /// Custom adapter
    Custom,
}

/// Output configuration options
#[derive(Debug, Clone, Default)]
pub struct OutputOptions {
    /// Endpoint URL
    pub endpoint: Option<String>,
    /// Authentication token
    pub auth_token: Option<String>,
    /// Batch size for bulk operations
    pub batch_size: Option<usize>,
    /// Retry configuration
    pub retry_config: Option<RetryConfig>,
    /// Custom options
    pub custom: HashMap<String, serde_json::Value>,
}

/// Retry configuration
#[derive(Debug, Clone)]
pub struct RetryConfig {
    /// Maximum retry attempts
    pub max_attempts: u32,
    /// Initial delay in milliseconds
    pub initial_delay_ms: u64,
    /// Maximum delay in milliseconds
    pub max_delay_ms: u64,
    /// Backoff multiplier
    pub backoff_multiplier: f64,
}
```

### 2. Dashboard 어댑터

```rust
/// Dashboard output adapter (e.g., Grafana, WIA Earth)
#[async_trait]
pub trait DashboardAdapter: OutputAdapter {
    /// Push data to dashboard
    async fn push_data(&self, message: &ClimateMessage) -> Result<()>;

    /// Create or update panel
    async fn create_panel(&self, config: PanelConfig) -> Result<String>;

    /// Get dashboard URL
    fn dashboard_url(&self) -> Option<&str>;
}

/// Panel configuration for dashboards
#[derive(Debug, Clone)]
pub struct PanelConfig {
    pub title: String,
    pub data_type: DataType,
    pub visualization: VisualizationType,
    pub refresh_interval_ms: Option<u64>,
}

#[derive(Debug, Clone)]
pub enum VisualizationType {
    TimeSeries,
    Gauge,
    Map,
    Table,
    Heatmap,
}
```

### 3. Storage 어댑터

```rust
/// Storage output adapter (e.g., InfluxDB, S3)
#[async_trait]
pub trait StorageAdapter: OutputAdapter {
    /// Write data to storage
    async fn write(&self, message: &ClimateMessage) -> Result<()>;

    /// Write batch of data
    async fn write_batch(&self, messages: &[ClimateMessage]) -> Result<()>;

    /// Query data (optional)
    async fn query(&self, query: &StorageQuery) -> Result<Vec<ClimateMessage>>;

    /// Export to file format
    async fn export(&self, format: ExportFormat, path: &str) -> Result<()>;
}

#[derive(Debug, Clone)]
pub struct StorageQuery {
    pub data_type: Option<DataType>,
    pub time_range: Option<TimeRange>,
    pub location_bounds: Option<LocationBounds>,
    pub limit: Option<usize>,
}

#[derive(Debug, Clone)]
pub enum ExportFormat {
    Json,
    Csv,
    NetCDF,
    Parquet,
}
```

### 4. Alert 어댑터

```rust
/// Alert output adapter (e.g., Webhook, Slack)
#[async_trait]
pub trait AlertAdapter: OutputAdapter {
    /// Send alert notification
    async fn send_alert(&self, alert: &Alert) -> Result<()>;

    /// Register alert rule
    async fn register_rule(&self, rule: AlertRule) -> Result<String>;

    /// Unregister alert rule
    async fn unregister_rule(&self, rule_id: &str) -> Result<()>;

    /// Check if message triggers any rules
    async fn check_rules(&self, message: &ClimateMessage) -> Vec<Alert>;
}

#[derive(Debug, Clone)]
pub struct Alert {
    pub severity: AlertSeverity,
    pub title: String,
    pub message: String,
    pub source: Option<ClimateMessage>,
    pub timestamp: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlertSeverity {
    Info,
    Warning,
    Error,
    Critical,
}

#[derive(Debug, Clone)]
pub struct AlertRule {
    pub id: String,
    pub name: String,
    pub condition: AlertCondition,
    pub severity: AlertSeverity,
}

#[derive(Debug, Clone)]
pub enum AlertCondition {
    /// Threshold exceeded
    Threshold { field: String, operator: ComparisonOp, value: f64 },
    /// Rate of change
    RateOfChange { field: String, threshold: f64, window_seconds: u64 },
    /// Missing data
    MissingData { timeout_seconds: u64 },
}
```

### 5. 통합 출력 매니저

```rust
/// Output manager for coordinating multiple adapters
pub struct OutputManager {
    adapters: HashMap<String, Box<dyn OutputAdapter>>,
    default_adapters: Vec<String>,
}

impl OutputManager {
    /// Create a new output manager
    pub fn new() -> Self;

    /// Register an adapter
    pub fn register(&mut self, name: &str, adapter: Box<dyn OutputAdapter>) -> Result<()>;

    /// Unregister an adapter
    pub fn unregister(&mut self, name: &str) -> Result<()>;

    /// Output to all default adapters
    pub async fn broadcast(&self, message: &ClimateMessage) -> Result<()>;

    /// Output to specific adapter
    pub async fn output_to(&self, adapter_name: &str, message: &ClimateMessage) -> Result<()>;

    /// Get adapter by name
    pub fn get_adapter(&self, name: &str) -> Option<&dyn OutputAdapter>;

    /// List all registered adapters
    pub fn list_adapters(&self) -> Vec<&str>;
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
2. 연동 아키텍처 (Integration Architecture)
3. 출력 인터페이스 (Output Interface)
4. Dashboard 연동 (Visualization Integration)
5. Storage 연동 (Data Storage Integration)
6. Alert 연동 (Alert/Notification Integration)
7. 통합 출력 매니저 (Output Manager)
8. 인증 및 보안 (Authentication & Security)
9. 에러 처리 (Error Handling)
10. 예제 (Examples)
```

### 3. Rust 출력 모듈
```
/api/rust/src/
├── integration/
│   ├── mod.rs
│   ├── output_adapter.rs    # 출력 인터페이스
│   ├── output_manager.rs    # 통합 매니저
│   ├── dashboard/
│   │   ├── mod.rs
│   │   ├── grafana.rs       # Grafana 어댑터
│   │   └── mock.rs          # 테스트용
│   ├── storage/
│   │   ├── mod.rs
│   │   ├── influxdb.rs      # InfluxDB 어댑터
│   │   ├── file.rs          # 파일 저장
│   │   └── mock.rs          # 테스트용
│   └── alert/
│       ├── mod.rs
│       ├── webhook.rs       # Webhook 어댑터
│       ├── console.rs       # 콘솔 출력
│       └── mock.rs          # 테스트용
└── ...
```

### 4. 예제 코드
```
/api/rust/examples/
├── integration_demo.rs      # 전체 연동 데모
├── grafana_output.rs        # Grafana 출력 예제
├── influxdb_storage.rs      # InfluxDB 저장 예제
├── webhook_alert.rs         # Webhook 알림 예제
└── multi_output.rs          # 다중 출력 예제
```

### 5. JSON Schema
```
/spec/schemas/
├── alert-rule.schema.json   # 알림 규칙 스키마
└── output-config.schema.json # 출력 설정 스키마
```

---

## ✅ 완료 체크리스트

Phase 4 완료 전 확인:

```
□ 웹서치로 시각화/저장소/알림 기술 조사 완료
□ /spec/RESEARCH-PHASE-4.md 작성 완료
□ /spec/PHASE-4-INTEGRATION.md 작성 완료
□ OutputAdapter 트레잇 정의 완료
□ DashboardAdapter 구현 완료 (Mock)
□ StorageAdapter 구현 완료 (File + Mock)
□ AlertAdapter 구현 완료 (Console + Webhook + Mock)
□ OutputManager 구현 완료
□ 단위 테스트 작성 완료
□ 테스트 통과
□ 전체 통합 데모 예제 완료
□ README 업데이트 (Phase 4 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 시각화/저장소/알림 기술 조사
   ↓
2. /spec/RESEARCH-PHASE-4.md 작성
   ↓
3. 출력 인터페이스 설계
   ↓
4. /spec/PHASE-4-INTEGRATION.md 작성
   ↓
5. Rust OutputAdapter 트레잇 정의
   ↓
6. Rust DashboardAdapter 구현 (Mock)
   ↓
7. Rust StorageAdapter 구현 (File)
   ↓
8. Rust AlertAdapter 구현 (Console/Webhook)
   ↓
9. Rust OutputManager 구현
   ↓
10. 테스트 작성 및 실행
   ↓
11. 전체 통합 데모 예제 작성
   ↓
12. 완료 체크리스트 확인
   ↓
13. WIA Climate Standard 완료! 🎉
```

---

## 🔗 WIA 생태계 연동 다이어그램

```
┌─────────────────────────────────────────────────────────────┐
│                     Climate Sensors                          │
│          (DAC, Weather Station, Ocean Buoy, etc.)           │
└─────────────────────────────────────────────────────────────┘
                              │
                         [센서 데이터]
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 1: Data Format Standard                   │
│                   센서 신호 → 표준 JSON                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 2: Rust API Interface                     │
│                  표준 API → 데이터 처리                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 3: Communication Protocol                 │
│                  WebSocket/MQTT 전송                         │
└─────────────────────────────────────────────────────────────┘
                              │
                        [Climate Data]
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 4: WIA Ecosystem Integration              │
│                     OutputManager                            │
├─────────────────┬─────────────────┬─────────────────────────┤
│ DashboardAdapter│  StorageAdapter │     AlertAdapter        │
└────────┬────────┴────────┬────────┴──────────┬──────────────┘
         │                 │                   │
         ▼                 ▼                   ▼
    ┌─────────┐      ┌─────────┐         ┌─────────┐
    │ Grafana │      │InfluxDB│         │ Webhook │
    │   WIA   │      │   S3   │         │  Slack  │
    │  Earth  │      │ NetCDF │         │  Email  │
    └─────────┘      └─────────┘         └─────────┘
         │                 │                   │
         ▼                 ▼                   ▼
   시각화/모니터링     장기 저장        실시간 알림
```

---

## 🚀 작업 시작

이제 Phase 4 작업을 시작하세요.

첫 번째 단계: **웹서치로 연동 기술 조사**

```
검색 키워드: "Grafana data source plugin climate data"
```

화이팅! 🌍

WIA Climate Standard의 마지막 Phase입니다.
완료되면 센서 입력부터 출력까지 전체 파이프라인이 완성됩니다!

---

<div align="center">

**Phase 4 of 4**

WIA Ecosystem Integration

🎯 최종 목표: 센서 → 처리 → 출력 (Dashboard/Storage/Alert)

弘益人間 - Benefit All Humanity

</div>
