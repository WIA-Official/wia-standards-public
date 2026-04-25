# Chapter 7: System Integration (Phase 4)

## End-to-End Integration of Vehicles, Sensors, and Shore-Based Systems

---

## 7.1 Reference Architecture

### System Overview

The WIA Deep Sea Exploration reference architecture provides a complete framework for integrating underwater vehicles, shipboard systems, and shore-based infrastructure into a cohesive operational platform.

```
┌──────────────────────────────────────────────────────────────────────┐
│                        SHORE-SIDE SYSTEMS                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │ Data Archive │  │   Mission    │  │   Public     │              │
│  │  & Portal    │  │   Planning   │  │   Portal     │              │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘              │
│         └─────────────────┴─────────────────┘                       │
│                           │                                          │
│                    ┌──────┴──────┐                                  │
│                    │ Cloud APIs  │                                  │
│                    └──────┬──────┘                                  │
└──────────────────────────┼───────────────────────────────────────────┘
                           │ Satellite / Internet
┌──────────────────────────┼───────────────────────────────────────────┐
│                    SHIPBOARD SYSTEMS                                 │
│  ┌───────────────────────┴────────────────────────┐                │
│  │              WIA Data Hub                       │                │
│  └─┬───────────┬───────────┬───────────┬─────────┘                │
│    │           │           │           │                            │
│  ┌─┴──────┐  ┌─┴────────┐ ┌─┴────────┐ ┌─┴────────────┐           │
│  │Mission │  │Telemetry │ │ Storage  │ │ Positioning  │           │
│  │Control │  │Dashboard │ │  Server  │ │   (USBL)     │           │
│  └────────┘  └──────────┘ └──────────┘ └──────────────┘           │
│                           │                                          │
└───────────────────────────┼──────────────────────────────────────────┘
                           │ Fiber / Acoustic
┌───────────────────────────┼──────────────────────────────────────────┐
│                   UNDERWATER SYSTEMS                                 │
│    ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐     │
│    │   ROV    │  │   AUV    │  │ Sensors  │  │ Transponders │     │
│    └──────────┘  └──────────┘  └──────────┘  └──────────────┘     │
└──────────────────────────────────────────────────────────────────────┘
```

### Component Responsibilities

| Component | Primary Functions |
|-----------|------------------|
| Data Archive | Long-term storage, data discovery, public access |
| Mission Planning | Pre-cruise planning, waypoint design, simulation |
| Cloud APIs | External integration, third-party access |
| WIA Data Hub | Central data aggregation, protocol conversion |
| Mission Control | Real-time vehicle control, pilot interface |
| Telemetry Dashboard | Status visualization, alerts |
| Storage Server | Local data storage, buffering |
| USBL | Underwater positioning |
| ROV/AUV | Data collection, sample retrieval |
| Sensors | Environmental measurements |

### Data Flow Patterns

**Real-Time Telemetry Flow**:
```
Vehicle → Tether/Acoustic → Data Hub → Dashboard
                                    → Storage
                                    → Cloud (subset)
```

**Command Flow**:
```
Pilot UI → Mission Control → Data Hub → Vehicle
```

**Data Archive Flow**:
```
Storage → Post-processing → Archive → Portal
```

---

## 7.2 Sensor Integration Framework

### Sensor Abstraction Layer

The WIA Sensor Integration Framework provides a unified interface for diverse oceanographic sensors:

```python
from abc import ABC, abstractmethod

class WIASensor(ABC):
    """Base class for all WIA-compliant sensors"""

    def __init__(self, sensor_id: str, config: dict):
        self.sensor_id = sensor_id
        self.config = config
        self.calibration = None
        self.last_reading = None

    @abstractmethod
    def initialize(self) -> bool:
        """Initialize sensor hardware"""
        pass

    @abstractmethod
    def read(self) -> dict:
        """Read current sensor value"""
        pass

    @abstractmethod
    def get_metadata(self) -> dict:
        """Return sensor metadata for WIA format"""
        pass

    def to_wia_format(self, reading: dict) -> dict:
        """Convert reading to WIA message format"""
        return {
            "value": reading["value"],
            "unit": self.config["unit"],
            "sensorId": self.sensor_id,
            "calibrationDate": self.calibration["date"],
            "accuracy": self.calibration["accuracy"],
            "qualityFlag": self.assess_quality(reading)
        }
```

### Sensor Driver Implementation

**CTD Sensor Example**:

```python
class SeaBirdCTD(WIASensor):
    """Driver for Sea-Bird CTD sensors"""

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

### Sensor Registry

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

### Plug-and-Play Discovery

```python
class SensorManager:
    def __init__(self):
        self.sensors = {}
        self.drivers = self.load_drivers()

    def discover_sensors(self):
        """Auto-discover connected sensors"""
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
        """Collect readings from all sensors"""
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

---

## 7.3 Real-Time Data Processing Pipelines

### Pipeline Architecture

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Ingest    │───>│   Process   │───>│   Analyze   │───>│   Output    │
│   Stage     │    │   Stage     │    │   Stage     │    │   Stage     │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
     │                   │                   │                   │
     ▼                   ▼                   ▼                   ▼
  Raw data          Validated           Derived            Stored &
  collection        & cleaned           products           distributed
```

### Stream Processing

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
        """Process data through pipeline"""
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

# Pipeline configuration
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

### Derived Parameters

| Input Parameters | Derived Parameter | Formula |
|-----------------|-------------------|---------|
| Temperature, Conductivity, Pressure | Salinity | UNESCO 1983 |
| Temperature, Salinity, Pressure | Sound Velocity | Chen-Millero |
| Temperature, Salinity, Pressure | Density | EOS-80 |
| Dissolved Oxygen, Temperature | Oxygen Saturation | Weiss 1970 |

```python
def derive_salinity(temp_c: float, cond_mS: float, press_dbar: float) -> float:
    """Calculate practical salinity from CTD data (UNESCO 1983)"""
    # Conductivity ratio
    R = cond_mS / 42.914

    # Temperature correction
    rt = 0.6766097 + temp_c * (0.0200564 + temp_c * (1.104259e-4 +
         temp_c * (-6.9698e-7 + temp_c * 1.0031e-9)))

    # Pressure correction
    Rp = 1 + press_dbar * (2.070e-5 + press_dbar * (-6.370e-10 +
         press_dbar * 3.989e-15))

    Rt = R / (Rp * rt)

    # Salinity calculation
    S = (0.008 - 0.1692 * Rt**0.5 + 25.3851 * Rt +
         14.0941 * Rt**1.5 - 7.0261 * Rt**2 + 2.7081 * Rt**2.5)

    return round(S, 4)
```

---

## 7.4 Cloud Integration and Storage

### Cloud Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Cloud Platform                          │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐│
│  │ API Gateway │  │  Message    │  │   Authentication    ││
│  │             │  │   Queue     │  │    Service          ││
│  └──────┬──────┘  └──────┬──────┘  └─────────────────────┘│
│         │                │                                  │
│  ┌──────┴────────────────┴─────────────────────────────┐  │
│  │              Processing Layer                         │  │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │  │
│  │  │ Ingest   │  │ Transform│  │ Quality Control  │   │  │
│  │  │ Workers  │  │ Workers  │  │ Workers          │   │  │
│  │  └──────────┘  └──────────┘  └──────────────────┘   │  │
│  └──────────────────────────────────────────────────────┘  │
│                           │                                  │
│  ┌───────────────────────────────────────────────────────┐ │
│  │                  Storage Layer                         │ │
│  │  ┌─────────┐  ┌─────────┐  ┌──────────┐  ┌────────┐ │ │
│  │  │Time-    │  │ Object  │  │ Metadata │  │ Search │ │ │
│  │  │Series DB│  │ Storage │  │ Database │  │ Index  │ │ │
│  │  └─────────┘  └─────────┘  └──────────┘  └────────┘ │ │
│  └───────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### Data Synchronization

```python
class CloudSync:
    def __init__(self, config):
        self.api_url = config["cloud_api_url"]
        self.api_key = config["api_key"]
        self.queue = asyncio.Queue()
        self.batch_size = 100
        self.sync_interval = 60  # seconds

    async def upload_batch(self, messages: list):
        """Upload batch of messages to cloud"""
        async with aiohttp.ClientSession() as session:
            response = await session.post(
                f"{self.api_url}/ingest/batch",
                headers={"Authorization": f"Bearer {self.api_key}"},
                json={"messages": messages}
            )
            return response.status == 200

    async def sync_loop(self):
        """Continuous sync with cloud"""
        batch = []
        while True:
            try:
                # Collect messages
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
                    # Retry logic with exponential backoff
                    await self.handle_upload_failure(batch)
```

### Offline Operation

```python
class OfflineBuffer:
    """Buffer data when cloud connectivity is lost"""

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

---

## 7.5 Visualization and Mission Control

### Dashboard Components

| Component | Purpose | Update Rate |
|-----------|---------|-------------|
| 3D Position | Vehicle location in water column | 1 Hz |
| Telemetry Gauges | Depth, heading, speed, altitude | 1 Hz |
| Sensor Plots | Time-series of environmental data | 0.1-1 Hz |
| Video Feeds | Live camera streams | 30 fps |
| Mission Map | Waypoints, track, survey area | Event-driven |
| System Status | Alerts, warnings, health | Event-driven |

### Mission Control Interface

```typescript
// React component for mission control dashboard
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

---

## 7.6 Third-Party System Integration

### Integration Patterns

| Pattern | Use Case | Complexity |
|---------|----------|------------|
| REST API | Data queries, commands | Low |
| WebSocket | Real-time streaming | Medium |
| Message Queue | Async processing | Medium |
| File Export | Batch data exchange | Low |
| Database Replication | Data warehouse | High |

### Webhook Integration

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

---

## 7.7 Testing and Validation

### Test Categories

| Category | Scope | Automation |
|----------|-------|------------|
| Unit Tests | Individual components | Fully automated |
| Integration Tests | Component interactions | Automated |
| System Tests | End-to-end flows | Semi-automated |
| Performance Tests | Load, latency | Automated |
| Acceptance Tests | User scenarios | Manual + automated |

### Validation Checklist

- [ ] All WIA message types validated against schema
- [ ] API endpoints return correct status codes
- [ ] WebSocket connections handle reconnection
- [ ] Data pipeline processes 1000 messages/second
- [ ] Cloud sync recovers from connectivity loss
- [ ] Emergency messages delivered within 100ms
- [ ] Video streams maintain 30 fps under load
- [ ] Dashboard updates within 500ms of data receipt

---

## Chapter Summary

Phase 4 completes the WIA Deep Sea Exploration Standard by providing comprehensive guidance for system integration. The reference architecture establishes clear component responsibilities and data flows, while the sensor integration framework enables plug-and-play sensor support.

Real-time data processing pipelines transform raw sensor data into validated, derived products. Cloud integration enables global data access while offline buffering ensures no data is lost during connectivity outages. Visualization and mission control interfaces provide operators with the situational awareness needed for safe, effective operations.

---

## Key Takeaways

1. **Reference architecture defines component responsibilities** and data flows
2. **Sensor abstraction layer enables plug-and-play** sensor integration
3. **Data pipelines transform raw data** into validated products
4. **Cloud sync with offline buffering** ensures data resilience
5. **Comprehensive testing validates** end-to-end functionality

---

## Review Questions

1. Draw the data flow from a CTD sensor to the shore-side archive.
2. Implement a WIA sensor driver for a new instrument.
3. Design a data pipeline for calculating derived salinity.
4. How does the system handle cloud connectivity loss?
5. What dashboard components are essential for ROV operations?

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
