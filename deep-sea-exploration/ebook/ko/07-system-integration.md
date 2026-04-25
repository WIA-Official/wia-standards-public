# 제7장: 시스템 통합 (Phase 4)

## 차량, 센서, 육상 기반 시스템의 종단간 통합

---

## 7.1 참조 아키텍처

### 시스템 개요

WIA 심해 탐사 참조 아키텍처는 수중 차량, 선상 시스템, 육상 기반 인프라를 통합된 운영 플랫폼으로 통합하기 위한 완전한 프레임워크를 제공합니다.

```
┌──────────────────────────────────────────────────────────────────────┐
│                        육상 측 시스템                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │ 데이터 아카이브 │  │   미션     │  │   공공      │              │
│  │  & 포털      │  │   계획     │  │   포털      │              │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘              │
│         └─────────────────┴─────────────────┘                       │
│                           │                                          │
│                    ┌──────┴──────┐                                  │
│                    │ 클라우드 API │                                  │
│                    └──────┬──────┘                                  │
└──────────────────────────┼───────────────────────────────────────────┘
                           │ 위성 / 인터넷
┌──────────────────────────┼───────────────────────────────────────────┐
│                    선상 시스템                                       │
│  ┌───────────────────────┴────────────────────────┐                │
│  │              WIA 데이터 허브                    │                │
│  └─┬───────────┬───────────┬───────────┬─────────┘                │
│    │           │           │           │                            │
│  ┌─┴──────┐  ┌─┴────────┐ ┌─┴────────┐ ┌─┴────────────┐           │
│  │미션    │  │원격측정   │ │ 저장소   │ │ 측위        │           │
│  │제어    │  │대시보드   │ │  서버   │ │   (USBL)    │           │
│  └────────┘  └──────────┘ └──────────┘ └──────────────┘           │
│                           │                                          │
└───────────────────────────┼──────────────────────────────────────────┘
                           │ 광섬유 / 음향
┌───────────────────────────┼──────────────────────────────────────────┐
│                   수중 시스템                                        │
│    ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐     │
│    │   ROV    │  │   AUV    │  │ 센서     │  │ 트랜스폰더   │     │
│    └──────────┘  └──────────┘  └──────────┘  └──────────────┘     │
└──────────────────────────────────────────────────────────────────────┘
```

### 구성 요소 책임

| 구성 요소 | 주요 기능 |
|-----------|----------|
| 데이터 아카이브 | 장기 저장, 데이터 검색, 공개 접근 |
| 미션 계획 | 크루즈 전 계획, 웨이포인트 설계, 시뮬레이션 |
| 클라우드 API | 외부 통합, 제3자 접근 |
| WIA 데이터 허브 | 중앙 데이터 집계, 프로토콜 변환 |
| 미션 제어 | 실시간 차량 제어, 조종사 인터페이스 |
| 원격측정 대시보드 | 상태 시각화, 경보 |
| 저장소 서버 | 로컬 데이터 저장, 버퍼링 |
| USBL | 수중 측위 |
| ROV/AUV | 데이터 수집, 샘플 회수 |
| 센서 | 환경 측정 |

### 데이터 흐름 패턴

**실시간 원격측정 흐름**:
```
차량 → 테더/음향 → 데이터 허브 → 대시보드
                              → 저장소
                              → 클라우드 (일부)
```

**명령 흐름**:
```
조종사 UI → 미션 제어 → 데이터 허브 → 차량
```

**데이터 아카이브 흐름**:
```
저장소 → 후처리 → 아카이브 → 포털
```

---

## 7.2 센서 통합 프레임워크

### 센서 추상화 계층

WIA 센서 통합 프레임워크는 다양한 해양학 센서를 위한 통합 인터페이스를 제공합니다:

```python
from abc import ABC, abstractmethod

class WIASensor(ABC):
    """모든 WIA 호환 센서의 기본 클래스"""

    def __init__(self, sensor_id: str, config: dict):
        self.sensor_id = sensor_id
        self.config = config
        self.calibration = None
        self.last_reading = None

    @abstractmethod
    def initialize(self) -> bool:
        """센서 하드웨어 초기화"""
        pass

    @abstractmethod
    def read(self) -> dict:
        """현재 센서 값 읽기"""
        pass

    @abstractmethod
    def get_metadata(self) -> dict:
        """WIA 형식을 위한 센서 메타데이터 반환"""
        pass

    def to_wia_format(self, reading: dict) -> dict:
        """읽기 값을 WIA 메시지 형식으로 변환"""
        return {
            "value": reading["value"],
            "unit": self.config["unit"],
            "sensorId": self.sensor_id,
            "calibrationDate": self.calibration["date"],
            "accuracy": self.calibration["accuracy"],
            "qualityFlag": self.assess_quality(reading)
        }
```

### 센서 드라이버 구현

**CTD 센서 예제**:

```python
class SeaBirdCTD(WIASensor):
    """Sea-Bird CTD 센서용 드라이버"""

    def initialize(self) -> bool:
        self.serial = Serial(self.config["port"], 9600)
        self.serial.write(b"*INIT\r\n")
        response = self.serial.readline()
        return b"OK" in response

    def read(self) -> dict:
        self.serial.write(b"*DATA\r\n")
        raw = self.serial.readline().decode()
        values = raw.split(",")
        return {
            "temperature": float(values[0]),
            "conductivity": float(values[1]),
            "pressure": float(values[2]),
            "timestamp": datetime.utcnow().isoformat()
        }

    def get_metadata(self) -> dict:
        return {
            "manufacturer": "Sea-Bird Scientific",
            "model": "SBE 49 FastCAT",
            "serialNumber": self.config["serial"],
            "firmwareVersion": self.query_firmware(),
            "sampleRate": 16  # Hz
        }
```

### 센서 레지스트리

```yaml
# sensor-registry.yaml
sensors:
  - id: "CTD-001"
    type: "SeaBirdCTD"
    config:
      port: "/dev/ttyUSB0"
      serial: "SBE49-2023-001"
      unit: "celsius"
    calibration:
      date: "2024-12-01"
      accuracy: 0.001
      certificate: "CAL-2024-001"

  - id: "DO-001"
    type: "AanderaOptode"
    config:
      port: "/dev/ttyUSB1"
      serial: "OPT-2022-042"
    calibration:
      date: "2024-11-15"
      accuracy: 0.01
```

### 플러그 앤 플레이 검색

```python
class SensorManager:
    def __init__(self):
        self.sensors = {}
        self.drivers = self.load_drivers()

    def discover_sensors(self):
        """연결된 센서 자동 검색"""
        for port in serial.tools.list_ports.comports():
            for driver in self.drivers:
                if driver.probe(port):
                    sensor = driver(port)
                    self.register_sensor(sensor)
                    break

    def register_sensor(self, sensor):
        sensor.initialize()
        self.sensors[sensor.sensor_id] = sensor
        self.emit_event("sensor_registered", sensor)

    def collect_all(self) -> list:
        """모든 센서에서 읽기 값 수집"""
        readings = []
        for sensor in self.sensors.values():
            try:
                reading = sensor.read()
                wia_reading = sensor.to_wia_format(reading)
                readings.append(wia_reading)
            except Exception as e:
                self.log_error(sensor.sensor_id, e)
        return readings
```

### 한국 해양 센서 표준

한국해양과학기술원(KIOST)에서 사용하는 센서 시스템:

| 센서 유형 | 모델 | 측정 항목 | 정확도 |
|----------|------|----------|--------|
| CTD | SBE 911plus | 수온, 염분, 수심 | ±0.001°C |
| 용존산소 | Aanderaa 4831 | DO | ±1% |
| 유속계 | ADCP RDI | 해류 속도/방향 | ±0.1% |
| 형광계 | WET Labs ECO | 엽록소-a | ±0.01 μg/L |
| 탁도계 | Seapoint | 탁도 | ±0.01 NTU |

---

## 7.3 실시간 데이터 처리 파이프라인

### 파이프라인 아키텍처

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   수집      │───>│   처리      │───>│   분석      │───>│   출력      │
│   단계      │    │   단계      │    │   단계      │    │   단계      │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
     │                   │                   │                   │
     ▼                   ▼                   ▼                   ▼
  원시 데이터        검증 및           파생            저장 및
   수집             정제             제품            배포
```

### 스트림 처리

```python
from dataclasses import dataclass
from typing import Callable

@dataclass
class PipelineStage:
    name: str
    processor: Callable
    next_stages: list

class DataPipeline:
    def __init__(self):
        self.stages = {}

    def add_stage(self, stage: PipelineStage):
        self.stages[stage.name] = stage

    async def process(self, data: dict):
        """파이프라인을 통해 데이터 처리"""
        stage = self.stages.get("ingest")
        results = []

        while stage:
            try:
                data = await stage.processor(data)
                results.append((stage.name, data))

                if stage.next_stages:
                    stage = self.stages.get(stage.next_stages[0])
                else:
                    stage = None
            except Exception as e:
                self.handle_error(stage, e, data)
                break

        return results

# 파이프라인 구성
pipeline = DataPipeline()

pipeline.add_stage(PipelineStage(
    name="ingest",
    processor=validate_message,
    next_stages=["process"]
))

pipeline.add_stage(PipelineStage(
    name="process",
    processor=apply_calibration,
    next_stages=["analyze"]
))

pipeline.add_stage(PipelineStage(
    name="analyze",
    processor=derive_parameters,
    next_stages=["output"]
))

pipeline.add_stage(PipelineStage(
    name="output",
    processor=distribute_results,
    next_stages=[]
))
```

### 파생 매개변수

| 입력 매개변수 | 파생 매개변수 | 공식 |
|--------------|--------------|------|
| 온도, 전도도, 압력 | 염분 | UNESCO 1983 |
| 온도, 염분, 압력 | 음속 | Chen-Millero |
| 온도, 염분, 압력 | 밀도 | EOS-80 |
| 용존 산소, 온도 | 산소 포화도 | Weiss 1970 |

```python
def derive_salinity(temp_c: float, cond_mS: float, press_dbar: float) -> float:
    """CTD 데이터에서 실용 염분 계산 (UNESCO 1983)"""
    # 전도도 비율
    R = cond_mS / 42.914

    # 온도 보정
    rt = 0.6766097 + temp_c * (0.0200564 + temp_c * (1.104259e-4 +
         temp_c * (-6.9698e-7 + temp_c * 1.0031e-9)))

    # 압력 보정
    Rp = 1 + press_dbar * (2.070e-5 + press_dbar * (-6.370e-10 +
         press_dbar * 3.989e-15))

    Rt = R / (Rp * rt)

    # 염분 계산
    S = (0.008 - 0.1692 * Rt**0.5 + 25.3851 * Rt +
         14.0941 * Rt**1.5 - 7.0261 * Rt**2 + 2.7081 * Rt**2.5)

    return round(S, 4)
```

### 실시간 데이터 품질 관리

```python
class RealTimeQC:
    """
    실시간 데이터 품질 관리 시스템
    QARTOD (Quality Assurance of Real-Time Oceanographic Data) 기반
    """

    def __init__(self, config: dict):
        self.config = config
        self.history = {}  # 센서별 이력 데이터

    def run_tests(self, data: dict) -> dict:
        """
        QARTOD 품질 테스트 실행

        Returns:
            QC 플래그가 추가된 데이터
        """
        sensor_id = data['sensorId']
        value = data['value']

        flags = {
            'gross_range': self.gross_range_test(value, data['type']),
            'spike': self.spike_test(sensor_id, value),
            'rate_of_change': self.rate_of_change_test(sensor_id, value),
            'flat_line': self.flat_line_test(sensor_id, value)
        }

        # 종합 플래그 계산
        aggregate_flag = self._compute_aggregate(flags)

        data['qc'] = {
            'flags': flags,
            'aggregate': aggregate_flag,
            'timestamp': datetime.utcnow().isoformat()
        }

        # 이력 업데이트
        self._update_history(sensor_id, value)

        return data

    def gross_range_test(self, value: float, data_type: str) -> int:
        """
        총 범위 테스트: 물리적으로 불가능한 값 탐지

        Returns:
            1: PASS, 3: SUSPECT, 4: FAIL
        """
        ranges = {
            'temperature': (-2.5, 40),
            'salinity': (0, 42),
            'pressure': (0, 12000),
            'dissolved_oxygen': (0, 500)
        }

        min_val, max_val = ranges.get(data_type, (-float('inf'), float('inf')))

        if min_val <= value <= max_val:
            return 1  # PASS
        else:
            return 4  # FAIL

    def spike_test(self, sensor_id: str, value: float) -> int:
        """
        스파이크 테스트: 갑작스러운 변화 탐지
        """
        history = self.history.get(sensor_id, [])

        if len(history) < 3:
            return 1  # 이력 부족, PASS

        # 스파이크 계산
        prev = history[-1]
        prev_prev = history[-2]

        spike = abs(value - prev) - abs(prev - prev_prev) / 2
        threshold = self.config.get('spike_threshold', 3.0)

        if spike > threshold:
            return 3  # SUSPECT
        return 1  # PASS
```

---

## 7.4 클라우드 통합 및 저장

### 클라우드 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                     클라우드 플랫폼                          │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐│
│  │ API 게이트웨이│  │  메시지    │  │   인증              ││
│  │             │  │   큐       │  │    서비스           ││
│  └──────┬──────┘  └──────┬──────┘  └─────────────────────┘│
│         │                │                                  │
│  ┌──────┴────────────────┴─────────────────────────────┐  │
│  │              처리 계층                               │  │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │  │
│  │  │ 수집     │  │ 변환     │  │ 품질 관리        │   │  │
│  │  │ 워커     │  │ 워커     │  │ 워커             │   │  │
│  │  └──────────┘  └──────────┘  └──────────────────┘   │  │
│  └──────────────────────────────────────────────────────┘  │
│                           │                                  │
│  ┌───────────────────────────────────────────────────────┐ │
│  │                  저장 계층                             │ │
│  │  ┌─────────┐  ┌─────────┐  ┌──────────┐  ┌────────┐ │ │
│  │  │시계열   │  │ 객체    │  │ 메타데이터│  │ 검색   │ │ │
│  │  │DB      │  │ 저장소  │  │ 데이터베이스│  │ 인덱스 │ │ │
│  │  └─────────┘  └─────────┘  └──────────┘  └────────┘ │ │
│  └───────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 데이터 동기화

```python
class CloudSync:
    def __init__(self, config):
        self.api_url = config["cloud_api_url"]
        self.api_key = config["api_key"]
        self.queue = asyncio.Queue()
        self.batch_size = 100
        self.sync_interval = 60  # 초

    async def upload_batch(self, messages: list):
        """클라우드에 메시지 배치 업로드"""
        async with aiohttp.ClientSession() as session:
            response = await session.post(
                f"{self.api_url}/ingest/batch",
                headers={"Authorization": f"Bearer {self.api_key}"},
                json={"messages": messages}
            )
            return response.status == 200

    async def sync_loop(self):
        """클라우드와 지속적 동기화"""
        batch = []
        while True:
            try:
                # 메시지 수집
                while len(batch) < self.batch_size:
                    msg = await asyncio.wait_for(
                        self.queue.get(),
                        timeout=self.sync_interval
                    )
                    batch.append(msg)
            except asyncio.TimeoutError:
                pass

            if batch:
                success = await self.upload_batch(batch)
                if success:
                    batch = []
                else:
                    # 지수 백오프로 재시도 로직
                    await self.handle_upload_failure(batch)
```

### 오프라인 운영

```python
class OfflineBuffer:
    """클라우드 연결이 끊어졌을 때 데이터 버퍼"""

    def __init__(self, db_path: str):
        self.db = sqlite3.connect(db_path)
        self.create_tables()

    def buffer_message(self, message: dict):
        self.db.execute(
            "INSERT INTO buffer (timestamp, message, synced) VALUES (?, ?, ?)",
            (datetime.utcnow(), json.dumps(message), False)
        )
        self.db.commit()

    def get_unsynced(self, limit: int = 1000) -> list:
        cursor = self.db.execute(
            "SELECT id, message FROM buffer WHERE synced = 0 LIMIT ?",
            (limit,)
        )
        return [(row[0], json.loads(row[1])) for row in cursor.fetchall()]

    def mark_synced(self, ids: list):
        self.db.execute(
            f"UPDATE buffer SET synced = 1 WHERE id IN ({','.join('?' * len(ids))})",
            ids
        )
        self.db.commit()
```

### 한국 해양 데이터 아카이브

한국의 주요 해양 데이터 아카이브 시스템과의 통합:

| 시스템 | 운영 기관 | 데이터 유형 | API 형식 |
|--------|----------|-----------|----------|
| KODC | KIOST | 해양관측 데이터 | REST/NetCDF |
| KHOA | 국립해양조사원 | 조석/해류 | REST/JSON |
| KOOS | 해양환경관리공단 | 해양환경 | SOAP/XML |
| K-IOOS | 해양수산부 | 통합 해양관측 | OGC SOS |

```python
class KODCIntegration:
    """
    KIOST 해양데이터센터(KODC) 통합
    """

    BASE_URL = "https://kodc.kiost.ac.kr/api/v1"

    def __init__(self, api_key: str):
        self.api_key = api_key

    async def submit_cruise_data(self, cruise_data: dict) -> str:
        """
        탐사 데이터 제출

        Returns:
            등록된 데이터셋 ID
        """
        async with aiohttp.ClientSession() as session:
            response = await session.post(
                f"{self.BASE_URL}/datasets",
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json"
                },
                json=cruise_data
            )
            result = await response.json()
            return result['datasetId']

    async def register_ctd_profile(self, profile: dict) -> str:
        """
        CTD 프로파일 등록
        """
        # KODC 형식으로 변환
        kodc_format = {
            'station': profile['stationId'],
            'date': profile['timestamp'],
            'latitude': profile['position']['latitude'],
            'longitude': profile['position']['longitude'],
            'measurements': [
                {
                    'depth': m['depth'],
                    'temperature': m['temperature'],
                    'salinity': m['salinity'],
                    'pressure': m['pressure']
                }
                for m in profile['data']
            ]
        }

        return await self._submit('ctd', kodc_format)
```

---

## 7.5 시각화 및 미션 제어

### 대시보드 구성 요소

| 구성 요소 | 목적 | 업데이트 속도 |
|-----------|------|--------------|
| 3D 위치 | 수중에서의 차량 위치 | 1 Hz |
| 원격측정 게이지 | 깊이, 방위, 속도, 고도 | 1 Hz |
| 센서 플롯 | 환경 데이터 시계열 | 0.1-1 Hz |
| 비디오 피드 | 실시간 카메라 스트림 | 30 fps |
| 미션 맵 | 웨이포인트, 트랙, 조사 영역 | 이벤트 기반 |
| 시스템 상태 | 경보, 경고, 건강 상태 | 이벤트 기반 |

### 미션 제어 인터페이스

```typescript
// 미션 제어 대시보드용 React 컴포넌트
interface MissionControlProps {
  vehicleId: string;
  webSocketUrl: string;
}

const MissionControl: React.FC<MissionControlProps> = ({ vehicleId, webSocketUrl }) => {
  const [telemetry, setTelemetry] = useState<VehicleTelemetry | null>(null);
  const [alerts, setAlerts] = useState<Alert[]>([]);

  useEffect(() => {
    const ws = new WebSocket(webSocketUrl);

    ws.onmessage = (event) => {
      const message = JSON.parse(event.data);

      switch (message.channel) {
        case 'navigation':
          setTelemetry(prev => ({ ...prev, position: message.data }));
          break;
        case 'alerts':
          setAlerts(prev => [...prev, message.data]);
          break;
      }
    };

    return () => ws.close();
  }, [webSocketUrl]);

  return (
    <div className="mission-control">
      <PositionDisplay position={telemetry?.position} />
      <TelemetryGauges data={telemetry} />
      <VideoGrid vehicleId={vehicleId} />
      <AlertPanel alerts={alerts} />
      <CommandPanel vehicleId={vehicleId} />
    </div>
  );
};
```

### 3D 시각화 컴포넌트

```typescript
interface DeepSeaVisualizerProps {
  telemetry: VehicleTelemetry;
  bathymetry: BathymetryGrid;
}

const DeepSeaVisualizer: React.FC<DeepSeaVisualizerProps> = ({
  telemetry,
  bathymetry
}) => {
  const sceneRef = useRef<THREE.Scene>();

  useEffect(() => {
    // Three.js 장면 초기화
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight);

    // 해저 지형 렌더링
    const geometry = createBathymetryGeometry(bathymetry);
    const material = new THREE.MeshPhongMaterial({
      vertexColors: true,
      side: THREE.DoubleSide
    });
    const terrain = new THREE.Mesh(geometry, material);
    scene.add(terrain);

    // ROV 모델 추가
    const rovModel = loadROVModel();
    scene.add(rovModel);

    sceneRef.current = scene;

    return () => {
      // 정리
    };
  }, []);

  // 원격측정 업데이트 시 ROV 위치 갱신
  useEffect(() => {
    if (sceneRef.current && telemetry) {
      updateROVPosition(sceneRef.current, telemetry);
    }
  }, [telemetry]);

  return <canvas id="deep-sea-canvas" />;
};
```

---

## 7.6 제3자 시스템 통합

### 통합 패턴

| 패턴 | 사용 사례 | 복잡도 |
|------|----------|--------|
| REST API | 데이터 쿼리, 명령 | 낮음 |
| WebSocket | 실시간 스트리밍 | 중간 |
| 메시지 큐 | 비동기 처리 | 중간 |
| 파일 내보내기 | 배치 데이터 교환 | 낮음 |
| 데이터베이스 복제 | 데이터 웨어하우스 | 높음 |

### 웹훅 통합

```python
class WebhookManager:
    def __init__(self):
        self.subscriptions = []

    def subscribe(self, url: str, events: list, secret: str):
        self.subscriptions.append({
            "url": url,
            "events": events,
            "secret": secret
        })

    async def notify(self, event: str, data: dict):
        for sub in self.subscriptions:
            if event in sub["events"]:
                payload = {
                    "event": event,
                    "timestamp": datetime.utcnow().isoformat(),
                    "data": data
                }
                signature = hmac.new(
                    sub["secret"].encode(),
                    json.dumps(payload).encode(),
                    hashlib.sha256
                ).hexdigest()

                await self.send_webhook(sub["url"], payload, signature)
```

### OGC 표준 지원

```python
class OGCSensorThingsAPI:
    """
    OGC SensorThings API 통합
    국제 표준 기반 데이터 공유
    """

    def __init__(self, base_url: str):
        self.base_url = base_url

    def create_thing(self, vehicle: dict) -> dict:
        """
        SensorThings Thing 생성 (차량)
        """
        thing = {
            "@iot.id": vehicle['id'],
            "name": vehicle['name'],
            "description": f"WIA Deep Sea Vehicle: {vehicle['type']}",
            "properties": {
                "vehicleType": vehicle['type'],
                "operator": vehicle['operator'],
                "maxDepth": vehicle['maxDepth']
            },
            "Locations": [{
                "name": "Current Location",
                "encodingType": "application/geo+json",
                "location": {
                    "type": "Point",
                    "coordinates": [
                        vehicle['longitude'],
                        vehicle['latitude'],
                        -vehicle['depth']
                    ]
                }
            }]
        }

        return self._post("Things", thing)

    def create_datastream(self, sensor: dict, thing_id: str) -> dict:
        """
        SensorThings Datastream 생성
        """
        datastream = {
            "name": f"{sensor['type']} measurements",
            "description": f"Observations from {sensor['id']}",
            "observationType": "http://www.opengis.net/def/observationType/OGC-OM/2.0/OM_Measurement",
            "unitOfMeasurement": {
                "name": sensor['unit']['name'],
                "symbol": sensor['unit']['symbol'],
                "definition": sensor['unit']['definition']
            },
            "Thing": {"@iot.id": thing_id},
            "Sensor": self._get_or_create_sensor(sensor),
            "ObservedProperty": self._get_observed_property(sensor['type'])
        }

        return self._post("Datastreams", datastream)
```

---

## 7.7 테스트 및 검증

### 테스트 범주

| 범주 | 범위 | 자동화 |
|------|------|--------|
| 단위 테스트 | 개별 구성 요소 | 완전 자동화 |
| 통합 테스트 | 구성 요소 상호작용 | 자동화 |
| 시스템 테스트 | 종단간 흐름 | 반자동화 |
| 성능 테스트 | 부하, 지연 시간 | 자동화 |
| 인수 테스트 | 사용자 시나리오 | 수동 + 자동화 |

### 검증 체크리스트

- [ ] 모든 WIA 메시지 유형이 스키마에 대해 검증됨
- [ ] API 엔드포인트가 올바른 상태 코드 반환
- [ ] WebSocket 연결이 재연결 처리
- [ ] 데이터 파이프라인이 초당 1000 메시지 처리
- [ ] 클라우드 동기화가 연결 손실에서 복구
- [ ] 비상 메시지가 100ms 이내에 전달됨
- [ ] 비디오 스트림이 부하에서 30 fps 유지
- [ ] 대시보드가 데이터 수신 후 500ms 이내에 업데이트

### 시뮬레이션 테스트 환경

```python
class DeepSeaSimulator:
    """
    심해 환경 시뮬레이터
    통합 테스트 및 운영자 훈련용
    """

    def __init__(self, config: dict):
        self.config = config
        self.vehicles = {}
        self.environment = self._create_environment()

    def _create_environment(self) -> dict:
        """
        시뮬레이션 환경 생성
        """
        return {
            'bathymetry': self._load_bathymetry(),
            'currents': self._generate_current_field(),
            'temperature_profile': self._create_temp_profile(),
            'salinity_profile': self._create_sal_profile()
        }

    def add_vehicle(self, vehicle_id: str, vehicle_type: str,
                    initial_position: dict):
        """
        시뮬레이션에 차량 추가
        """
        self.vehicles[vehicle_id] = {
            'type': vehicle_type,
            'position': initial_position,
            'velocity': {'x': 0, 'y': 0, 'z': 0},
            'sensors': self._create_virtual_sensors(vehicle_type)
        }

    def simulate_step(self, dt: float):
        """
        시뮬레이션 한 스텝 진행
        """
        for vid, vehicle in self.vehicles.items():
            # 물리 시뮬레이션
            self._update_physics(vehicle, dt)

            # 센서 시뮬레이션
            telemetry = self._simulate_sensors(vehicle)

            # 통신 시뮬레이션 (지연, 손실)
            if self._simulate_comm_success(vehicle['position']):
                self._emit_telemetry(vid, telemetry)

    def inject_fault(self, fault_type: str, vehicle_id: str):
        """
        장애 주입 테스트
        """
        faults = {
            'comm_loss': self._simulate_comm_loss,
            'sensor_failure': self._simulate_sensor_failure,
            'thruster_malfunction': self._simulate_thruster_fault,
            'leak_detected': self._simulate_leak
        }

        fault_handler = faults.get(fault_type)
        if fault_handler:
            fault_handler(vehicle_id)
```

---

## 장 요약

Phase 4는 시스템 통합에 대한 포괄적인 지침을 제공하여 WIA 심해 탐사 표준을 완성합니다. 참조 아키텍처는 명확한 구성 요소 책임과 데이터 흐름을 설정하고, 센서 통합 프레임워크는 플러그 앤 플레이 센서 지원을 가능하게 합니다.

실시간 데이터 처리 파이프라인은 원시 센서 데이터를 검증된 파생 제품으로 변환합니다. 클라우드 통합은 전역 데이터 접근을 가능하게 하고 오프라인 버퍼링은 연결 중단 시 데이터 손실을 방지합니다. 시각화 및 미션 제어 인터페이스는 운영자에게 안전하고 효과적인 운용에 필요한 상황 인식을 제공합니다.

---

## 핵심 요점

1. **참조 아키텍처가 구성 요소 책임과 데이터 흐름 정의**
2. **센서 추상화 계층이 플러그 앤 플레이 센서 통합 가능**
3. **데이터 파이프라인이 원시 데이터를 검증된 제품으로 변환**
4. **오프라인 버퍼링이 있는 클라우드 동기화로 데이터 복원력 보장**
5. **포괄적인 테스트가 종단간 기능 검증**

---

## 복습 질문

1. CTD 센서에서 육상 아카이브까지의 데이터 흐름을 그리세요.
2. 새 계측기용 WIA 센서 드라이버를 구현하세요.
3. 파생 염분 계산을 위한 데이터 파이프라인을 설계하세요.
4. 시스템이 클라우드 연결 손실을 어떻게 처리합니까?
5. ROV 운용에 필수적인 대시보드 구성 요소는 무엇입니까?

---

## 이사부호 시스템 통합 사례

대한민국의 대표적 해양연구선 '이사부호'의 시스템 통합 사례:

| 시스템 | 구성 요소 | WIA 통합 상태 |
|--------|----------|--------------|
| 다중 빔 음향측심기 | Kongsberg EM122 | 완료 |
| CTD 시스템 | SBE 911plus | 완료 |
| USBL 측위 | Sonardyne Ranger 2 | 완료 |
| ROV HEMIRE | 6000m급 작업용 | 진행 중 |
| 선상 데이터 서버 | 분산 저장 클러스터 | 완료 |
| KODC 연동 | 실시간 데이터 전송 | 완료 |

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 하라
