# 제7장: UTM 및 시스템 통합

## Phase 4: 공역 통합, 교통 관리 및 다중 시스템 조정

---

## 7.1 UTM 아키텍처 개요

### UTM이란?

무인항공교통관리(UTM)는 저고도 공역에서 안전한 드론 운영을 가능하게 하는 서비스 생태계입니다. 직접적인 관제사-조종사 통신을 통해 유인 항공기를 관리하는 전통적인 항공교통관제(ATC)와 달리, UTM은 수천 개의 동시 드론 운영을 조정하기 위해 자동화된 시스템을 사용합니다.

```
UTM 생태계 아키텍처:

┌─────────────────────────────────────────────────────────────────────┐
│                    규제 당국 (FAA/EASA/국토부)                        │
│  - 공역 규칙                                                         │
│  - 운영 요구사항                                                     │
│  - 인증 표준                                                         │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    UTM 서비스 제공자 (USS)                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                │
│  │   USS #1    │  │   USS #2    │  │   USS #3    │                │
│  │  (AirMap)   │  │  (Unifly)   │  │  (OneSky)   │                │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘                │
│         │                │                │                        │
│         └────────────────┼────────────────┘                        │
│                          │                                          │
│                    USS 네트워크                                      │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    드론 운영자                                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                │
│  │ 배송 회사   │  │ 측량 회사   │  │ 점검 회사   │                │
│  │   함대      │  │   함대      │  │   함대      │                │
│  └─────────────┘  └─────────────┘  └─────────────┘                │
└─────────────────────────────────────────────────────────────────────┘
```

### UTM 서비스

| 서비스 | 설명 | 제공자 |
|--------|------|--------|
| 비행 인가 | 비행 계획 요청/승인 | USS |
| 전략적 분리 | 비행 전 충돌 감지 | USS |
| 전술적 분리 | 비행 중 충돌 해결 | USS |
| 공역 관리 | 동적 공역 제약 | 당국 |
| Remote ID | 드론 식별 브로드캐스트 | 드론 |
| 보충 데이터 | 날씨, 장애물, NOTAM | 다중 |

### 분리 요구사항

```
표준 분리 최소값:

수평 분리:
┌─────────────────────────────────────────┐
│                                         │
│     드론 A      50m 최소      드론 B    │
│        ●─────────────────────────────●  │
│                                         │
└─────────────────────────────────────────┘

수직 분리:
┌─────────────────────────────────────────┐
│                                         │
│  드론 A ●                               │
│          │                              │
│          │  30m 최소                    │
│          │                              │
│  드론 B ●                               │
│                                         │
└─────────────────────────────────────────┘

결합 (안전 볼륨):
       50m
    ←───────→
    ┌───────┐  ↑
    │       │  │ 30m
    │   ●   │  │
    │       │  ↓
    └───────┘
```

---

## 7.2 비행 계획 제출

### 비행 계획 구조

```json
{
  "operationId": "OP-20250101-1234",
  "operatorId": "WIA-OP-001",
  "pilotId": "PILOT-001",
  "droneId": "WIA-DRN-X1-0042",
  "submitTime": "2025-01-01T09:00:00Z",
  "operationType": "DELIVERY",
  "priority": "NORMAL",
  "flightGeography": {
    "type": "Polygon",
    "coordinates": [[
      [126.95, 37.55],
      [127.05, 37.55],
      [127.05, 37.48],
      [126.95, 37.48],
      [126.95, 37.55]
    ]]
  },
  "operationVolumes": [
    {
      "volumeId": "VOL-001",
      "timeStart": "2025-01-01T10:00:00Z",
      "timeEnd": "2025-01-01T10:30:00Z",
      "altitudeLower": 0,
      "altitudeUpper": 120,
      "altitudeReference": "MSL",
      "geography": {
        "type": "Polygon",
        "coordinates": [[...]]
      }
    }
  ],
  "trajectory": {
    "waypoints": [
      {"lat": 37.5665, "lng": 126.9780, "alt": 50, "time": "2025-01-01T10:00:00Z"},
      {"lat": 37.5300, "lng": 127.0100, "alt": 120, "time": "2025-01-01T10:05:00Z"},
      {"lat": 37.5012, "lng": 127.0396, "alt": 50, "time": "2025-01-01T10:12:00Z"}
    ]
  },
  "contingencyPlans": {
    "lostLink": "RETURN_TO_HOME",
    "lostGps": "HOVER_AND_WAIT",
    "lowBattery": "LAND_NEAREST",
    "alternativeLandingSites": [
      {"lat": 37.5300, "lng": 127.0100, "name": "비상 착륙장 A"}
    ]
  },
  "droneDetails": {
    "class": "LIGHT",
    "mtow": 8.5,
    "maxSpeed": 22,
    "communicationType": "4G_LTE",
    "remoteIdType": "BROADCAST"
  }
}
```

### 인가 워크플로우

```python
class UTMClient:
    """
    UTM 서비스 제공자와 상호작용하는 클라이언트.
    """

    def __init__(self, api_url: str, api_key: str):
        self.api_url = api_url
        self.api_key = api_key

    async def submit_flight_plan(self, flight_plan: dict) -> dict:
        """
        인가를 위한 비행 계획 제출.

        Returns:
            운영 ID와 상태가 포함된 인가 응답
        """
        async with aiohttp.ClientSession() as session:
            response = await session.post(
                f"{self.api_url}/operations",
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json"
                },
                json=flight_plan
            )

            result = await response.json()

            if response.status == 201:
                return {
                    "status": "AUTHORIZED",
                    "operationId": result["operationId"],
                    "validFrom": result["validFrom"],
                    "validTo": result["validTo"],
                    "constraints": result.get("constraints", [])
                }
            elif response.status == 409:
                return {
                    "status": "CONFLICT",
                    "conflicts": result["conflicts"],
                    "suggestions": result.get("suggestions", [])
                }
            else:
                return {
                    "status": "REJECTED",
                    "reason": result.get("reason", "알 수 없음")
                }

    async def activate_operation(self, operation_id: str) -> bool:
        """
        비행 준비 시 인가된 운영 활성화.
        """
        async with aiohttp.ClientSession() as session:
            response = await session.put(
                f"{self.api_url}/operations/{operation_id}/activate",
                headers={"Authorization": f"Bearer {self.api_key}"}
            )
            return response.status == 200

    async def end_operation(self, operation_id: str) -> bool:
        """
        활성 운영 종료.
        """
        async with aiohttp.ClientSession() as session:
            response = await session.put(
                f"{self.api_url}/operations/{operation_id}/end",
                headers={"Authorization": f"Bearer {self.api_key}"}
            )
            return response.status == 200
```

---

## 7.3 실시간 위치 보고

### 위치 보고 형식

드론은 비행 중 매초 위치를 보고해야 합니다:

```json
{
  "operationId": "OP-20250101-1234",
  "droneId": "WIA-DRN-X1-0042",
  "timestamp": "2025-01-01T10:05:30.123Z",
  "position": {
    "lat": 37.5300,
    "lng": 127.0100,
    "altMsl": 125.5,
    "altAgl": 35.2,
    "accuracy": {
      "horizontal": 1.5,
      "vertical": 2.0
    }
  },
  "velocity": {
    "speed": 15.5,
    "track": 45.0,
    "verticalSpeed": -0.3
  },
  "status": {
    "operationalStatus": "ACTIVE",
    "remoteIdBroadcasting": true,
    "battery": 72,
    "gpsStatus": "RTK_FIXED"
  }
}
```

### 위치 보고 구현

```python
class PositionReporter:
    """
    UTM에 실시간 위치 보고.
    """

    def __init__(self, utm_client: UTMClient, operation_id: str):
        self.client = utm_client
        self.operation_id = operation_id
        self.report_interval = 1.0  # 초
        self.running = False

    async def start_reporting(self, telemetry_source):
        """
        연속 위치 보고 시작.
        """
        self.running = True

        while self.running:
            try:
                # 현재 원격측정 가져오기
                telemetry = await telemetry_source.get_current()

                # 위치 보고 포맷
                report = self._format_report(telemetry)

                # UTM에 전송
                await self.client.report_position(self.operation_id, report)

                # 다음 간격까지 대기
                await asyncio.sleep(self.report_interval)

            except Exception as e:
                logging.error(f"위치 보고 실패: {e}")
                # 오류에도 보고 계속
                await asyncio.sleep(self.report_interval)

    def stop_reporting(self):
        self.running = False

    def _format_report(self, telemetry: dict) -> dict:
        return {
            "operationId": self.operation_id,
            "droneId": telemetry["droneId"],
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "position": {
                "lat": telemetry["position"]["latitude"],
                "lng": telemetry["position"]["longitude"],
                "altMsl": telemetry["position"]["altitudeMSL"],
                "altAgl": telemetry["position"]["altitudeAGL"],
                "accuracy": telemetry["position"]["accuracy"]
            },
            "velocity": {
                "speed": telemetry["velocity"]["groundSpeed"],
                "track": telemetry["attitude"]["heading"],
                "verticalSpeed": telemetry["velocity"]["verticalSpeed"]
            },
            "status": {
                "operationalStatus": "ACTIVE",
                "remoteIdBroadcasting": True,
                "battery": telemetry["battery"]["percentage"],
                "gpsStatus": telemetry["gps"]["status"]
            }
        }
```

---

## 7.4 충돌 감지 및 해결

### 충돌 유형

| 유형 | 설명 | 대응 시간 |
|------|------|----------|
| 전략적 | 다른 계획과의 비행 전 중복 | 비행 전 |
| 전술적 | 비행 중 예측 충돌 | 30-60초 |
| 긴급 | 높은 충돌 확률 | <10초 |

### 전략적 분리

```python
class StrategicDeconfliction:
    """
    비행 전 충돌 감지 및 해결.
    """

    def __init__(self, separation_horizontal: float = 50.0,
                 separation_vertical: float = 30.0,
                 time_buffer: float = 60.0):
        self.sep_h = separation_horizontal
        self.sep_v = separation_vertical
        self.time_buffer = time_buffer

    def check_conflicts(self, new_operation: dict,
                       existing_operations: list) -> list:
        """
        기존 운영과의 충돌 확인.

        Returns:
            충돌 운영 리스트
        """
        conflicts = []

        for existing in existing_operations:
            if self._operations_conflict(new_operation, existing):
                conflicts.append({
                    "operationId": existing["operationId"],
                    "conflictType": "VOLUME_OVERLAP",
                    "conflictTime": self._find_conflict_time(new_operation, existing),
                    "suggestedResolution": self._suggest_resolution(new_operation, existing)
                })

        return conflicts

    def _operations_conflict(self, op1: dict, op2: dict) -> bool:
        """
        두 운영이 중복되는 볼륨을 가지는지 확인.
        """
        for vol1 in op1["operationVolumes"]:
            for vol2 in op2["operationVolumes"]:
                # 시간 중복 확인
                if not self._times_overlap(vol1, vol2):
                    continue

                # 공간 중복 확인
                if self._volumes_overlap(vol1, vol2):
                    return True

        return False

    def _times_overlap(self, vol1: dict, vol2: dict) -> bool:
        """시간 창이 버퍼와 함께 중복되는지 확인."""
        start1 = datetime.fromisoformat(vol1["timeStart"].rstrip("Z"))
        end1 = datetime.fromisoformat(vol1["timeEnd"].rstrip("Z"))
        start2 = datetime.fromisoformat(vol2["timeStart"].rstrip("Z"))
        end2 = datetime.fromisoformat(vol2["timeEnd"].rstrip("Z"))

        buffer = timedelta(seconds=self.time_buffer)

        return (start1 - buffer < end2 + buffer and
                start2 - buffer < end1 + buffer)

    def _suggest_resolution(self, new_op: dict, existing: dict) -> dict:
        """
        충돌에 대한 해결 제안.
        """
        # 먼저 시간적 해결 시도
        suggested_start = datetime.fromisoformat(
            existing["operationVolumes"][0]["timeEnd"].rstrip("Z")
        ) + timedelta(minutes=5)

        return {
            "type": "TEMPORAL",
            "suggestedStart": suggested_start.isoformat() + "Z",
            "reason": "기존 운영 이후까지 지연"
        }
```

### 전술적 분리

```python
class TacticalDeconfliction:
    """
    비행 중 실시간 충돌 감지.
    """

    def __init__(self, prediction_horizon: float = 60.0,
                 update_interval: float = 1.0):
        self.horizon = prediction_horizon
        self.interval = update_interval

    def predict_conflicts(self, own_state: dict, traffic: list) -> list:
        """
        다음 예측 기간 내 충돌 예측.

        Args:
            own_state: 현재 드론 상태 (위치, 속도)
            traffic: 근처 교통 상태 리스트

        Returns:
            예측 충돌 리스트
        """
        conflicts = []

        own_trajectory = self._predict_trajectory(own_state, self.horizon)

        for other in traffic:
            other_trajectory = self._predict_trajectory(other, self.horizon)

            conflict = self._check_trajectory_conflict(own_trajectory, other_trajectory)

            if conflict:
                conflicts.append({
                    "trafficId": other["droneId"],
                    "timeToConflict": conflict["time"],
                    "closestApproach": conflict["distance"],
                    "conflictPoint": conflict["point"],
                    "urgency": self._calculate_urgency(conflict)
                })

        return sorted(conflicts, key=lambda c: c["timeToConflict"])

    def resolve_conflict(self, own_state: dict, conflict: dict,
                        current_waypoint: dict) -> dict:
        """
        회피 기동 생성.

        Returns:
            수정된 웨이포인트 또는 속도 조정
        """
        ttc = conflict["timeToConflict"]

        if ttc > 30:
            # 충분한 시간 - 속도 조정
            return {
                "type": "SPEED_ADJUSTMENT",
                "newSpeed": own_state["speed"] * 0.7,
                "duration": 20
            }

        elif ttc > 15:
            # 중간 긴급성 - 고도 변경
            current_alt = own_state["position"]["altitude"]
            other_alt = conflict["conflictPoint"]["altitude"]

            if current_alt > other_alt:
                new_alt = current_alt + 50
            else:
                new_alt = current_alt - 50

            return {
                "type": "ALTITUDE_CHANGE",
                "newAltitude": max(30, min(120, new_alt)),
                "climbRate": 3.0 if new_alt > current_alt else -3.0
            }

        else:
            # 높은 긴급성 - 즉시 회피
            return {
                "type": "IMMEDIATE_AVOIDANCE",
                "action": "HOVER",
                "duration": 30
            }
```

---

## 7.5 Remote ID (원격 식별)

### 브로드캐스트 Remote ID

```python
class RemoteIDTransmitter:
    """
    ASTM F3411 / ASD-STAN prEN 4709-002에 따른 Remote ID 브로드캐스트.
    """

    def __init__(self, serial_number: str, operator_id: str):
        self.serial_number = serial_number
        self.operator_id = operator_id
        self.session_id = self._generate_session_id()

    def generate_broadcast_message(self, position: dict, velocity: dict,
                                   pilot_location: dict) -> bytes:
        """
        Remote ID 브로드캐스트 메시지 생성.

        메시지는 Bluetooth 5.0 및/또는 WiFi NaN을 통해 브로드캐스트.
        """
        message = {
            "protocolVersion": 2,
            "messageType": "BASIC_ID",
            "idType": "SERIAL_NUMBER",
            "uasId": self.serial_number,
            "timestamp": time.time(),
            "position": {
                "latitude": position["latitude"],
                "longitude": position["longitude"],
                "geodetic_altitude": position["altitudeMSL"],
                "height_agl": position["altitudeAGL"],
                "horizontal_accuracy": position["accuracy"]["horizontal"],
                "vertical_accuracy": position["accuracy"]["vertical"],
                "speed_accuracy": 0.5
            },
            "velocity": {
                "speed": velocity["groundSpeed"],
                "vertical_speed": velocity["verticalSpeed"],
                "direction": velocity["track"]
            },
            "operator": {
                "operator_id": self.operator_id,
                "pilot_location": {
                    "latitude": pilot_location["latitude"],
                    "longitude": pilot_location["longitude"]
                }
            },
            "status": {
                "operational_status": "AIRBORNE",
                "height_type": "AGL",
                "classification": "DELIVERY"
            }
        }

        return self._encode_message(message)

    def _encode_message(self, message: dict) -> bytes:
        """
        ASTM F3411-19 형식에 따라 메시지 인코딩.

        실제 인코딩은 Bluetooth/WiFi 브로드캐스트를 위한
        특정 바이너리 형식을 따름.
        """
        import struct

        # 바이너리 형식으로 패킹
        # 간소화된 표현
        packed = struct.pack(
            '!B',  # 프로토콜 버전
            message["protocolVersion"]
        )

        # 위치 추가 (압축된 위경도)
        lat_enc = int((message["position"]["latitude"] + 90) * 1e7)
        lon_enc = int((message["position"]["longitude"] + 180) * 1e7)

        packed += struct.pack('!II', lat_enc, lon_enc)

        # 속도 추가
        packed += struct.pack('!HhH',
            int(message["velocity"]["speed"] * 100),
            int(message["velocity"]["vertical_speed"] * 100),
            int(message["velocity"]["direction"] * 100)
        )

        return packed
```

### 네트워크 Remote ID

```python
class NetworkRemoteID:
    """
    USS에 대한 네트워크 기반 Remote ID 보고.
    """

    def __init__(self, uss_endpoint: str, api_key: str):
        self.endpoint = uss_endpoint
        self.api_key = api_key

    async def report(self, telemetry: dict, operator_info: dict):
        """
        네트워크 서비스에 Remote ID 정보 보고.

        브로드캐스트 Remote ID를 보완:
        - 브로드캐스트 범위를 넘어선 운영
        - 이력 추적
        - 법 집행 조회
        """
        report = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "aircraft": {
                "serial_number": telemetry["droneId"],
                "registration": telemetry.get("registration"),
                "aircraft_type": "MULTIROTOR"
            },
            "position": {
                "lat": telemetry["position"]["latitude"],
                "lng": telemetry["position"]["longitude"],
                "alt": telemetry["position"]["altitudeMSL"],
                "accuracy_h": telemetry["position"]["accuracy"]["horizontal"],
                "accuracy_v": telemetry["position"]["accuracy"]["vertical"]
            },
            "velocity": {
                "speed": telemetry["velocity"]["groundSpeed"],
                "track": telemetry["attitude"]["heading"],
                "vertical": telemetry["velocity"]["verticalSpeed"]
            },
            "operator": {
                "id": operator_info["operatorId"],
                "location": operator_info.get("location")
            },
            "operation": {
                "id": telemetry.get("operationId"),
                "category": "DELIVERY",
                "status": "ACTIVE"
            }
        }

        async with aiohttp.ClientSession() as session:
            await session.post(
                f"{self.endpoint}/rid/flights",
                headers={"Authorization": f"Bearer {self.api_key}"},
                json=report
            )
```

---

## 7.6 지상 관제 스테이션 통합

### GCS 아키텍처

```
지상 관제 스테이션 아키텍처:

┌─────────────────────────────────────────────────────────────────────┐
│                    지상 관제 스테이션                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐    │
│  │   지도 표시     │  │   원격측정      │  │   비디오 피드   │    │
│  │                 │  │    대시보드     │  │                 │    │
│  │  - 드론 위치   │  │  - 배터리      │  │  - 실시간 카메라│    │
│  │  - 웨이포인트  │  │  - GPS 상태    │  │  - 녹화        │    │
│  │  - 지오펜스    │  │  - 모터 상태   │  │  - 스냅샷      │    │
│  │  - 교통        │  │  - 고도        │  │                 │    │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘    │
│                                                                     │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐    │
│  │ 미션 제어      │  │  알림 패널     │  │  함대 상태      │    │
│  │                 │  │                 │  │                 │    │
│  │  - 미션 계획   │  │  - 경고        │  │  - 전체 드론   │    │
│  │  - 명령 전송   │  │  - 오류        │  │  - 할당        │    │
│  │  - 진행 모니터 │  │  - UTM 알림    │  │  - 가용성      │    │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘    │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │                    통신 레이어                               │  │
│  │  주: 4G/5G  │  백업: 900MHz  │  UTM: HTTPS/WebSocket        │  │
│  └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### GCS 소프트웨어 구현

```typescript
// React 기반 GCS 컴포넌트
interface DroneStatus {
  droneId: string;
  position: Position;
  velocity: Velocity;
  battery: BatteryStatus;
  missionProgress: number;
  alerts: Alert[];
}

const GroundControlStation: React.FC = () => {
  const [drones, setDrones] = useState<Map<string, DroneStatus>>(new Map());
  const [selectedDrone, setSelectedDrone] = useState<string | null>(null);
  const [alerts, setAlerts] = useState<Alert[]>([]);

  // 실시간 업데이트를 위한 WebSocket 연결
  useEffect(() => {
    const ws = new WebSocket('wss://api.wia.com/v1/gcs/stream');

    ws.onmessage = (event) => {
      const message = JSON.parse(event.data);

      switch (message.type) {
        case 'TELEMETRY':
          updateDroneStatus(message.droneId, message.data);
          break;
        case 'ALERT':
          addAlert(message.data);
          break;
        case 'UTM_UPDATE':
          handleUTMUpdate(message.data);
          break;
      }
    };

    return () => ws.close();
  }, []);

  const sendCommand = async (droneId: string, command: Command) => {
    const response = await fetch(`/api/v1/drones/${droneId}/commands`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(command)
    });

    if (!response.ok) {
      addAlert({
        severity: 'ERROR',
        message: `명령 실패: ${command.type}`,
        droneId
      });
    }
  };

  return (
    <div className="gcs-layout">
      <MapDisplay
        drones={drones}
        selectedDrone={selectedDrone}
        onDroneSelect={setSelectedDrone}
      />
      <TelemetryPanel
        drone={selectedDrone ? drones.get(selectedDrone) : null}
      />
      <MissionControl
        drone={selectedDrone}
        onCommand={sendCommand}
      />
      <AlertPanel alerts={alerts} />
      <FleetOverview drones={drones} />
    </div>
  );
};
```

---

## 7.7 다중 드론 조정

### 함대 배차 최적화

```python
class FleetDispatcher:
    """
    배송 미션을 위한 드론 할당 최적화.
    """

    def __init__(self, fleet: list):
        self.fleet = {d["droneId"]: d for d in fleet}

    def assign_mission(self, mission: dict) -> str:
        """
        미션에 최적의 드론 선택.

        고려사항:
        - 배터리 수준
        - 픽업까지 거리
        - 현재 작업량
        - 드론 능력
        """
        pickup = mission["pickup"]["location"]
        package_weight = mission["package"]["weight"]

        candidates = []

        for drone_id, drone in self.fleet.items():
            # 이용 불가 드론 건너뛰기
            if drone["status"] != "AVAILABLE":
                continue

            # 페이로드 용량 확인
            if package_weight > drone["maxPayload"]:
                continue

            # 배터리 확인
            if drone["battery"] < 50:
                continue

            # 점수 계산
            distance = self._calculate_distance(
                drone["location"], pickup
            )

            score = self._calculate_score(drone, distance, mission)
            candidates.append((drone_id, score))

        if not candidates:
            raise NoAvailableDroneError("적합한 드론 없음")

        # 최고 점수 드론 반환
        candidates.sort(key=lambda x: x[1], reverse=True)
        return candidates[0][0]

    def _calculate_score(self, drone: dict, distance: float,
                        mission: dict) -> float:
        """
        할당 점수 계산 (높을수록 좋음).
        """
        # 근접성 점수 (가까울수록 좋음)
        proximity = 1.0 / (1.0 + distance / 1000)

        # 배터리 점수
        battery = drone["battery"] / 100

        # 효율성 점수 (오늘 성과 기준)
        efficiency = drone.get("todayEfficiency", 0.9)

        # 미션 우선순위 가중치
        priority_weights = {"EXPRESS": 1.5, "STANDARD": 1.0, "ECONOMY": 0.7}
        priority = priority_weights.get(mission["priority"], 1.0)

        return (0.4 * proximity + 0.3 * battery + 0.3 * efficiency) * priority
```

---

## 한국 UTM 시스템: K-드론

### K-드론 시스템 개요

한국은 국토교통부 주관으로 K-드론 시스템을 운영하고 있습니다:

| 구성요소 | 기능 |
|----------|------|
| 드론 원스톱 민원서비스 | 비행 승인 신청 |
| K-드론 시스템 | 실시간 교통 관리 |
| 드론 식별 시스템 | Remote ID 관리 |
| 공역 정보 시스템 | 비행금지구역 정보 |

### K-드론 API 연동

```python
class KDroneClient:
    """K-드론 시스템 연동 클라이언트."""

    def __init__(self, api_key: str):
        self.api_key = api_key
        self.base_url = "https://api.kdrone.go.kr/v1"

    async def submit_flight_approval(self, plan: dict) -> dict:
        """비행 승인 신청."""
        return await self._post("/flight-approvals", {
            "registrationNumber": plan["droneRegistration"],
            "pilotLicense": plan["pilotLicense"],
            "flightArea": plan["area"],
            "flightTime": plan["scheduledTime"],
            "purpose": "DELIVERY"
        })

    async def check_airspace(self, lat: float, lon: float, alt: float) -> dict:
        """공역 상태 확인."""
        return await self._get("/airspace/check", {
            "latitude": lat,
            "longitude": lon,
            "altitude": alt
        })

    async def get_no_fly_zones(self, bounds: dict) -> list:
        """비행금지구역 조회."""
        return await self._get("/no-fly-zones", bounds)
```

---

## 장 요약

UTM 통합은 안전하고 확장 가능한 드론 배송 운영에 필수적입니다. UTM 생태계는 비행 계획을 조정하고, 실시간 분리를 제공하며, 드론이 운영 전반에 걸쳐 식별되고 추적될 수 있도록 보장합니다.

USS를 통한 비행 계획 제출은 이륙 전 전략적 분리를 가능하게 하고, 실시간 위치 보고는 비행 중 전술적 충돌 감지를 지원합니다. Remote ID 브로드캐스트 및 네트워크 보고는 당국 및 다른 공역 사용자에게 드론 식별을 보장합니다.

지상 관제 스테이션은 운영자에게 상황 인식 및 제어 기능을 제공하며, 함대 배차 최적화는 배송 미션에 대한 효율적인 드론 할당을 보장합니다.

---

## 핵심 요약

1. **UTM은 저고도 공역으로의 안전한 드론 통합** 가능
2. **전략적 분리**는 비행 전 충돌 방지
3. **실시간 위치 보고**는 전술적 충돌 감지 가능
4. **Remote ID**는 당국 및 다른 공역 사용자에게 식별 제공
5. **함대 최적화**로 배송 효율성 극대화

---

## 복습 문제

1. 전략적 분리와 전술적 분리의 차이점은?
2. 드론은 비행 중 얼마나 자주 UTM에 위치를 보고해야 합니까?
3. Remote ID 브로드캐스트에 포함되어야 하는 정보는?
4. 교차점에 접근하는 두 드론에 대한 충돌 해결 전략을 설계하시오.
5. 시간에 민감한 의료 배송을 위해 함대 배차를 어떻게 최적화하겠습니까?

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 하라
