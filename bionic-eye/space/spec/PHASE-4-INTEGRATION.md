# WIA Space Ecosystem Integration Specification

**Version**: 1.0.0
**Status**: Draft
**Phase**: 4 of 4

---

## 1. 개요 (Overview)

WIA Space Ecosystem Integration은 WIA Space Standard의 데이터를 외부 시스템과 연동하기 위한 출력 계층을 정의합니다.

### 1.1 목표

- Phase 1-3 결과물과 통합된 출력 계층 제공
- 다양한 시각화 도구 (CesiumJS, Three.js, GMAT) 연동
- 표준 데이터 형식 (CCSDS, NASA) 내보내기
- 대시보드 및 모니터링 시스템 통합
- 알림 및 이벤트 시스템 지원

### 1.2 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                   WIA Space Standard                         │
├─────────────────────────────────────────────────────────────┤
│  Phase 1: Data Format │ Phase 2: API │ Phase 3: Protocol    │
└─────────────────────────────────────────────────────────────┘
                              │
                      [OutputManager]
                              │
    ┌──────────┬──────────┬───┴───┬──────────┬──────────┐
    ▼          ▼          ▼       ▼          ▼          ▼
┌────────┐┌────────┐┌────────┐┌────────┐┌────────┐┌────────┐
│Visualize││Export ││Dashboard││ Alert ││ Logger ││ Custom │
│ Adapter ││Adapter││ Adapter ││Adapter││Adapter ││Adapter │
└────────┘└────────┘└────────┘└────────┘└────────┘└────────┘
```

---

## 2. 출력 계층 아키텍처 (Output Layer Architecture)

### 2.1 계층 구조

| 계층 | 설명 |
|-----|------|
| **Application** | 사용자 애플리케이션 |
| **OutputManager** | 통합 출력 관리자 |
| **Adapters** | 개별 출력 어댑터 |
| **Backends** | 외부 시스템 연동 |

### 2.2 데이터 흐름

```
SpaceProject
     │
     ▼
OutputData (serialize)
     │
     ▼
OutputManager.output()
     │
     ├─► VisualizerAdapter ─► CesiumJS / Three.js
     │
     ├─► ExporterAdapter ─► CCSDS OEM / JSON / CSV
     │
     ├─► DashboardAdapter ─► OpenMCT / Custom
     │
     └─► AlertAdapter ─► WebHook / Email / Slack
```

---

## 3. 출력 인터페이스 (Output Interface)

### 3.1 OutputAdapter Trait

```rust
/// Output type enumeration
pub enum OutputType {
    Visualization,
    Export,
    Dashboard,
    Alert,
    Logger,
    Custom(String),
}

/// Output adapter trait
#[async_trait]
pub trait OutputAdapter: Send + Sync {
    /// Get output type
    fn output_type(&self) -> OutputType;

    /// Get adapter name
    fn name(&self) -> &str;

    /// Initialize adapter
    async fn initialize(&mut self, config: &OutputConfig) -> Result<()>;

    /// Output data
    async fn output(&self, data: &OutputData) -> Result<OutputResult>;

    /// Check availability
    fn is_available(&self) -> bool;

    /// Dispose adapter
    async fn dispose(&mut self) -> Result<()>;
}
```

### 3.2 OutputData

```rust
/// Output data container
pub struct OutputData {
    /// Data source (project, mission, etc.)
    pub source: DataSource,

    /// Data payload
    pub payload: OutputPayload,

    /// Metadata
    pub metadata: OutputMetadata,

    /// Timestamp
    pub timestamp: DateTime<Utc>,
}

/// Data source types
pub enum DataSource {
    Project(SpaceProject),
    Mission(MissionData),
    Telemetry(TelemetryData),
    Simulation(SimulationData),
    Custom(serde_json::Value),
}

/// Output payload types
pub enum OutputPayload {
    Specification(TechnologySpec),
    Trajectory(TrajectoryData),
    Status(StatusData),
    Alert(AlertData),
    Raw(Vec<u8>),
}
```

### 3.3 OutputConfig

```rust
/// Output configuration
pub struct OutputConfig {
    /// Output format
    pub format: OutputFormat,

    /// Target path or URL
    pub target: String,

    /// Additional options
    pub options: HashMap<String, serde_json::Value>,
}

/// Output formats
pub enum OutputFormat {
    // WIA formats
    WiaJson,
    WiaCompact,

    // CCSDS formats
    CcsdsOem,
    CcsdsOpm,
    CcsdsOmm,

    // NASA formats
    NasaSpice,
    GmatScript,

    // Generic formats
    Json,
    Csv,
    Xml,

    // Visualization formats
    Czml,          // Cesium
    GeoJson,       // Geographic
    Kml,           // Google Earth
}
```

---

## 4. 시각화 연동 (Visualization Integration)

### 4.1 VisualizerAdapter

```rust
pub struct VisualizerAdapter {
    pub backend: VisualizerBackend,
    pub config: VisualizerConfig,
}

pub enum VisualizerBackend {
    Cesium,      // CesiumJS
    ThreeJs,     // Three.js
    Gmat,        // NASA GMAT
    OpenMct,     // NASA OpenMCT
    Custom(String),
}

pub struct VisualizerConfig {
    /// API endpoint or file path
    pub endpoint: String,

    /// Scene configuration
    pub scene: SceneConfig,

    /// Animation settings
    pub animation: AnimationConfig,
}
```

### 4.2 CZML 출력 (Cesium)

```json
[
  {
    "id": "document",
    "name": "WIA Space Mission",
    "version": "1.0"
  },
  {
    "id": "mars-orbiter-01",
    "name": "Mars Orbiter",
    "availability": "2035-01-01T00:00:00Z/2035-12-31T23:59:59Z",
    "position": {
      "referenceFrame": "INERTIAL",
      "cartographicDegrees": [
        "2035-01-01T00:00:00Z", -122.0, 37.0, 400000,
        "2035-01-01T01:00:00Z", -120.0, 38.0, 400000
      ]
    },
    "point": {
      "color": { "rgba": [255, 0, 0, 255] },
      "pixelSize": 10
    },
    "path": {
      "material": {
        "polylineGlow": {
          "color": { "rgba": [255, 128, 0, 255] },
          "glowPower": 0.2
        }
      },
      "width": 2,
      "leadTime": 3600,
      "trailTime": 3600
    }
  }
]
```

### 4.3 Three.js Scene 데이터

```json
{
  "scene": {
    "name": "Mars Terraforming Simulation",
    "objects": [
      {
        "id": "mars",
        "type": "sphere",
        "radius": 3389.5,
        "texture": "mars_texture.jpg",
        "position": [0, 0, 0]
      },
      {
        "id": "orbital-mirror-01",
        "type": "mesh",
        "geometry": "mirror_array.glb",
        "position": [0, 3500, 0],
        "animation": {
          "type": "orbit",
          "period": 5400
        }
      }
    ],
    "camera": {
      "position": [0, 5000, 10000],
      "target": [0, 0, 0]
    }
  }
}
```

---

## 5. 데이터 내보내기 (Data Export)

### 5.1 ExporterAdapter

```rust
pub struct ExporterAdapter {
    pub format: OutputFormat,
    pub config: ExporterConfig,
}

pub struct ExporterConfig {
    /// Output path
    pub output_path: PathBuf,

    /// Overwrite existing
    pub overwrite: bool,

    /// Compression
    pub compression: Option<CompressionType>,

    /// Include metadata
    pub include_metadata: bool,
}
```

### 5.2 CCSDS OEM 형식

```xml
<?xml version="1.0" encoding="UTF-8"?>
<oem xmlns="urn:ccsds:recommendation:navigation:schema:oem">
  <header>
    <CREATION_DATE>2035-03-15T12:00:00Z</CREATION_DATE>
    <ORIGINATOR>WIA Space Standard</ORIGINATOR>
  </header>
  <body>
    <segment>
      <metadata>
        <OBJECT_NAME>Mars Orbiter 01</OBJECT_NAME>
        <OBJECT_ID>2035-001A</OBJECT_ID>
        <CENTER_NAME>MARS</CENTER_NAME>
        <REF_FRAME>EME2000</REF_FRAME>
        <TIME_SYSTEM>UTC</TIME_SYSTEM>
        <START_TIME>2035-03-15T00:00:00Z</START_TIME>
        <STOP_TIME>2035-03-16T00:00:00Z</STOP_TIME>
      </metadata>
      <data>
        <stateVector>
          <EPOCH>2035-03-15T00:00:00Z</EPOCH>
          <X>-2500.000</X>
          <Y>3500.000</Y>
          <Z>1000.000</Z>
          <X_DOT>1.500</X_DOT>
          <Y_DOT>-2.000</Y_DOT>
          <Z_DOT>0.500</Z_DOT>
        </stateVector>
      </data>
    </segment>
  </body>
</oem>
```

### 5.3 GMAT Script 형식

```matlab
%----------------------------------------
%---------- Spacecraft
%----------------------------------------
Create Spacecraft MarsOrbiter;
GMAT MarsOrbiter.DateFormat = UTCGregorian;
GMAT MarsOrbiter.Epoch = '15 Mar 2035 00:00:00.000';
GMAT MarsOrbiter.CoordinateSystem = MarsFixed;
GMAT MarsOrbiter.SMA = 3896.2;
GMAT MarsOrbiter.ECC = 0.01;
GMAT MarsOrbiter.INC = 93.0;
GMAT MarsOrbiter.RAAN = 0.0;
GMAT MarsOrbiter.AOP = 0.0;
GMAT MarsOrbiter.TA = 0.0;

%----------------------------------------
%---------- Mission Sequence
%----------------------------------------
BeginMissionSequence;
Propagate DefaultProp(MarsOrbiter) {MarsOrbiter.ElapsedSecs = 86400};
```

### 5.4 WIA JSON 형식

WIA Space Standard Phase 1 형식 그대로 내보내기:

```json
{
  "$schema": "https://wia.live/schemas/space/mars-terraforming.schema.json",
  "technology_id": "terraform-mars-001",
  "category": "mars_terraforming",
  "trl": 3,
  "target_planet": {
    "name": "Mars",
    "current_atmosphere": {
      "pressure_pa": 636,
      "co2_percent": 95.3
    }
  },
  "intervention_methods": [
    {
      "method": "solar_mirrors",
      "temperature_effect_kelvin": 10.0
    }
  ],
  "metadata": {
    "exported_at": "2035-03-15T12:00:00Z",
    "exporter": "WIA Space Standard v1.0.0"
  }
}
```

---

## 6. 대시보드 연동 (Dashboard Integration)

### 6.1 DashboardAdapter

```rust
pub struct DashboardAdapter {
    pub backend: DashboardBackend,
    pub config: DashboardConfig,
}

pub enum DashboardBackend {
    OpenMct,      // NASA OpenMCT
    Grafana,      // Grafana
    Custom(String),
}

pub struct DashboardConfig {
    /// Dashboard URL
    pub url: String,

    /// Authentication
    pub auth: Option<DashboardAuth>,

    /// Update interval
    pub update_interval_ms: u64,
}
```

### 6.2 OpenMCT 텔레메트리 형식

```json
{
  "id": "wia.space.mars-orbiter.telemetry",
  "key": "atmospheric_processor",
  "name": "Atmospheric Processor",
  "values": [
    {
      "key": "timestamp",
      "source": "timestamp",
      "name": "Timestamp",
      "format": "utc"
    },
    {
      "key": "co2_concentration",
      "name": "CO2 Concentration",
      "units": "%",
      "format": "float"
    },
    {
      "key": "temperature_k",
      "name": "Temperature",
      "units": "K",
      "format": "float"
    },
    {
      "key": "pressure_pa",
      "name": "Pressure",
      "units": "Pa",
      "format": "integer"
    }
  ]
}
```

### 6.3 WebSocket 텔레메트리 스트림

Phase 3 프로토콜을 사용한 실시간 텔레메트리:

```json
{
  "protocol": "wia-space",
  "version": "1.0.0",
  "messageId": "...",
  "type": "telemetry",
  "source": { "id": "mars-orbiter-01", "type": "spacecraft" },
  "destination": { "id": "mission-control", "type": "ground_station" },
  "payload": {
    "missionId": "mars-terraform-2035",
    "subsystem": "atmospheric_processor",
    "readings": {
      "co2_concentration": 0.953,
      "temperature_k": 210.5,
      "pressure_pa": 636
    },
    "status": "nominal"
  }
}
```

---

## 7. 알림 시스템 (Alert System)

### 7.1 AlertAdapter

```rust
pub struct AlertAdapter {
    pub backend: AlertBackend,
    pub config: AlertConfig,
}

pub enum AlertBackend {
    WebHook,
    Email,
    Slack,
    Discord,
    Sms,
    Custom(String),
}

pub struct AlertConfig {
    /// Alert endpoint
    pub endpoint: String,

    /// Alert levels to send
    pub levels: Vec<AlertLevel>,

    /// Rate limiting
    pub rate_limit: Option<RateLimit>,
}

pub enum AlertLevel {
    Critical,
    Warning,
    Info,
    Debug,
}
```

### 7.2 Alert 메시지 형식

```json
{
  "alert": {
    "id": "alert-2035-001",
    "level": "warning",
    "source": "mars-orbiter-01",
    "subsystem": "atmospheric_processor",
    "message": "Processing rate below threshold",
    "details": {
      "current_rate_kg_per_day": 1200,
      "threshold_kg_per_day": 1500,
      "deviation_percent": -20
    },
    "timestamp": "2035-03-15T14:30:00Z",
    "recommended_action": "Check processor intake filters"
  }
}
```

### 7.3 WebHook 형식

```json
{
  "type": "wia_space_alert",
  "version": "1.0.0",
  "timestamp": "2035-03-15T14:30:00Z",
  "alert": {
    "level": "warning",
    "title": "Atmospheric Processor Alert",
    "message": "Processing rate below threshold",
    "source": {
      "mission": "mars-terraform-2035",
      "spacecraft": "mars-orbiter-01",
      "subsystem": "atmospheric_processor"
    },
    "data": {
      "metric": "processing_rate",
      "value": 1200,
      "unit": "kg/day",
      "threshold": 1500
    }
  }
}
```

---

## 8. OutputManager

### 8.1 통합 출력 관리자

```rust
pub struct OutputManager {
    adapters: HashMap<String, Box<dyn OutputAdapter>>,
    default_adapters: HashSet<OutputType>,
}

impl OutputManager {
    /// Register adapter
    pub fn register(&mut self, name: &str, adapter: Box<dyn OutputAdapter>);

    /// Unregister adapter
    pub fn unregister(&mut self, name: &str);

    /// Output to specific adapter
    pub async fn output_to(
        &self,
        adapter_name: &str,
        data: &OutputData,
    ) -> Result<OutputResult>;

    /// Broadcast to all adapters of type
    pub async fn broadcast(
        &self,
        output_type: OutputType,
        data: &OutputData,
    ) -> Vec<Result<OutputResult>>;

    /// Output to all default adapters
    pub async fn output(&self, data: &OutputData) -> Vec<Result<OutputResult>>;

    /// Get registered adapters
    pub fn get_adapters(&self) -> Vec<&dyn OutputAdapter>;
}
```

### 8.2 사용 예시

```rust
// Create output manager
let mut manager = OutputManager::new();

// Register adapters
manager.register("cesium", Box::new(CesiumAdapter::new()));
manager.register("ccsds", Box::new(CcsdsExporter::new()));
manager.register("webhook", Box::new(WebHookAdapter::new()));

// Create output data
let data = OutputData::from_project(&project);

// Output to specific adapter
manager.output_to("cesium", &data).await?;

// Broadcast to all exporters
manager.broadcast(OutputType::Export, &data).await;

// Output to all defaults
manager.output(&data).await;
```

---

## 9. 에러 처리 (Error Handling)

### 9.1 에러 코드

| 코드 | 이름 | 설명 |
|-----|------|------|
| `6001` | `ADAPTER_NOT_FOUND` | 어댑터 없음 |
| `6002` | `ADAPTER_NOT_AVAILABLE` | 어댑터 사용 불가 |
| `6003` | `OUTPUT_FAILED` | 출력 실패 |
| `6004` | `FORMAT_ERROR` | 형식 오류 |
| `6005` | `CONNECTION_FAILED` | 연결 실패 |
| `6006` | `AUTH_FAILED` | 인증 실패 |
| `6007` | `RATE_LIMITED` | 속도 제한 |
| `6008` | `TIMEOUT` | 시간 초과 |

### 9.2 OutputResult

```rust
pub enum OutputResult {
    Success {
        adapter: String,
        output_path: Option<String>,
        bytes_written: Option<usize>,
    },
    PartialSuccess {
        adapter: String,
        warnings: Vec<String>,
    },
    Failure {
        adapter: String,
        error: OutputError,
    },
}
```

---

## 10. 예제 (Examples)

### 10.1 전체 파이프라인

```rust
use wia_space::prelude::*;
use wia_space::output::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Create project (Phase 1)
    let project = SpaceProject::mars_terraforming("terraform-001");

    // Create output manager (Phase 4)
    let mut manager = OutputManager::new();

    // Register adapters
    manager.register("cesium", Box::new(CesiumAdapter::default()));
    manager.register("ccsds-oem", Box::new(CcsdsOemExporter::new("output/")));
    manager.register("webhook", Box::new(WebHookAdapter::new("https://...")));

    // Create output data
    let data = OutputData::from_project(&project)
        .with_metadata(OutputMetadata {
            author: "WIA Space".to_string(),
            version: "1.0.0".to_string(),
            ..Default::default()
        });

    // Output to all adapters
    let results = manager.output(&data).await;

    // Check results
    for result in results {
        match result {
            Ok(OutputResult::Success { adapter, .. }) => {
                println!("✓ {} succeeded", adapter);
            }
            Err(e) => {
                eprintln!("✗ Error: {}", e);
            }
        }
    }

    Ok(())
}
```

### 10.2 실시간 텔레메트리 스트리밍

```rust
use wia_space::protocol::*;
use wia_space::output::*;

async fn stream_telemetry(manager: &OutputManager) {
    // Create telemetry data
    let telemetry = TelemetryPayload {
        mission_id: "mars-terraform-2035".to_string(),
        subsystem: "atmospheric_processor".to_string(),
        readings: serde_json::json!({
            "co2_concentration": 0.953,
            "temperature_k": 210.5
        }),
        status: "nominal".to_string(),
        timestamp: None,
    };

    // Create output data
    let data = OutputData::from_telemetry(&telemetry);

    // Broadcast to dashboards
    manager.broadcast(OutputType::Dashboard, &data).await;
}
```

---

## 11. 구현 체크리스트

### 필수 구현

- [ ] OutputAdapter trait
- [ ] OutputManager
- [ ] WIA JSON 내보내기
- [ ] CSV 내보내기
- [ ] Mock 어댑터

### 선택 구현

- [ ] CCSDS OEM/OPM 내보내기
- [ ] GMAT Script 내보내기
- [ ] CZML (Cesium) 내보내기
- [ ] OpenMCT 연동
- [ ] WebHook 알림
- [ ] Slack/Discord 연동

---

## 12. 참조 (References)

- [NASA OpenMCT](https://github.com/nasa/openmct) - Mission Control Framework
- [CesiumJS](https://cesium.com/platform/cesiumjs/) - 3D Globe Visualization
- [CCSDS Orbit Data Messages](https://ccsds.org/Pubs/502x0b3e1.pdf) - OEM/OPM Specification
- [NASA GMAT](https://opensource.gsfc.nasa.gov/projects/GMAT/) - Mission Analysis Tool
- [WIA Space Phase 1](./PHASE-1-DATA-FORMAT.md) - Data Format Standard
- [WIA Space Phase 3](./PHASE-3-PROTOCOL.md) - Communication Protocol

---

*WIA Space Ecosystem Integration Specification v1.0.0*
*Phase 4 of 4*
