# 제6장: 통신 프로토콜

## Phase 3: MQTT, CoAP, LoRaWAN 및 HTTP 프로토콜

---

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

1. MQTT 토픽 구조와 QoS 수준 이해하기
2. 제약된 장치를 위한 CoAP 프로토콜 구현하기
3. LoRaWAN을 위한 컴팩트 바이너리 페이로드 인코딩하기
4. HTTP/REST 통신으로 센서 데이터 제출하기
5. TLS/DTLS를 사용한 전송 보안 구성하기
6. 다층 데이터 검증 및 품질 관리 적용하기
7. 프로토콜 변환을 위한 엣지 컴퓨팅 패턴 설계하기

---

## 6.1 MQTT 프로토콜 사양

MQTT (Message Queuing Telemetry Transport)는 연속 전원과 안정적인 네트워크 연결을 가진 연결된 센서에 이상적입니다.

### 토픽 구조

```
wia/{standard}/{region}/{location}/{deviceId}/{messageType}
```

**예:**
```
wia/env027/us-west/seattle/ENV-AIR-001/data
wia/env027/kr-seoul/gangnam/WATER-001/status
wia/env027/eu-london/zone3/SOIL-FARM-A-001/calibration
```

### 메시지 유형

| 토픽 접미사 | 목적 | QoS | 보존 |
|--------------|---------|-----|----------|
| `/data` | 센서 측정 | 1 | 아니오 |
| `/status` | 장치 상태 | 1 | 예 |
| `/calibration` | 보정 데이터 | 2 | 예 |
| `/command` | 제어 명령 | 2 | 아니오 |
| `/alert` | 임계값 경보 | 1 | 아니오 |

### 센서 데이터 게시

**토픽:** `wia/env027/us-west/seattle/ENV-AIR-001/data`

**페이로드 (JSON):**
```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "pm2_5": {"value": 15.3, "unit": "μg/m³"}
  }
}
```

**Python 예제:**
```python
import paho.mqtt.client as mqtt
import json
from datetime import datetime

client = mqtt.Client(client_id="wia-ENV-AIR-001")
client.username_pw_set("device_user", "device_password")
client.tls_set(ca_certs="/path/to/ca.crt")
client.connect("broker.example.com", 8883, 60)

data = {
    "version": "1.0.0",
    "standard": "WIA-ENE-027",
    "deviceId": "ENV-AIR-001",
    "timestamp": datetime.utcnow().isoformat() + "Z",
    "sensorType": "air_quality",
    "readings": {
        "pm2_5": {"value": 15.3, "unit": "μg/m³"},
        "pm10": {"value": 22.8, "unit": "μg/m³"}
    },
    "quality": {"overall": "good", "flags": []}
}

topic = "wia/env027/us-west/seattle/ENV-AIR-001/data"
client.publish(topic, json.dumps(data), qos=1)
```

### 연결 관리

**클라이언트 ID:** `wia-{deviceId}-{randomString}`

**Keep-Alive:** 60-300초 (보고 빈도 기준)

**Last Will and Testament:**
```json
{
  "topic": "wia/env027/us-west/seattle/ENV-AIR-001/status",
  "payload": {
    "deviceId": "ENV-AIR-001",
    "status": "offline",
    "timestamp": "2025-01-09T10:30:00Z"
  },
  "qos": 1,
  "retain": true
}
```

---

## 6.2 제약된 장치를 위한 CoAP

CoAP (Constrained Application Protocol)는 UDP 전송을 사용하는 리소스 제한 장치용으로 설계되었습니다.

### 리소스 구조

```
/wia/env/{resource}
```

**표준 리소스:**
```
/wia/env/data          - 현재 측정
/wia/env/status        - 장치 상태
/wia/env/config        - 구성
/wia/env/calibration   - 보정 데이터
```

### 리소스 탐색

```
GET coap://sensor.local/.well-known/core

응답:
</wia/env/data>;ct=50;rt="wia.env.data";obs,
</wia/env/status>;ct=50;rt="wia.env.status";obs,
</wia/env/config>;ct=50;rt="wia.env.config"
```

### GET 데이터

```
GET coap://sensor.local/wia/env/data
Accept: application/json

응답 (2.05 Content):
{
  "deviceId": "ENV-SOIL-001",
  "timestamp": "2025-01-09T10:30:00Z",
  "readings": {
    "moisture": {"value": 28.5, "unit": "%VWC"}
  }
}
```

### POST 데이터를 클라우드로

```
POST coap://api.example.com/api/v1/sensors/ENV-SOIL-001/data
Content-Format: application/json

{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-SOIL-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "soil",
  "readings": {
    "moisture": {"value": 28.5, "unit": "%VWC"}
  }
}

응답 (2.01 Created)
```

### Observe 패턴

```
GET coap://sensor.local/wia/env/data
Observe: 0

응답 시퀀스:
2.05 Content (Observe: 0)  {"moisture": 28.5, ...}
2.05 Content (Observe: 1)  {"moisture": 28.7, ...}
2.05 Content (Observe: 2)  {"moisture": 28.9, ...}
```

---

## 6.3 장거리 센서를 위한 LoRaWAN

LoRaWAN은 원격 센서를 위한 장거리, 저전력 통신을 가능하게 합니다.

### 페이로드 인코딩

**컴팩트 바이너리 형식 (17바이트):**

```
바이트 0:      메시지 유형 (0x01 = 데이터)
바이트 1-4:    장치 ID 해시 (4바이트)
바이트 5-8:    Unix 타임스탬프 (4바이트)
바이트 9-10:   PM2.5 (uint16, 스케일 0.1 μg/m³)
바이트 11-12:  PM10 (uint16, 스케일 0.1 μg/m³)
바이트 13-14:  온도 (int16, 스케일 0.01 °C)
바이트 15:     습도 (uint8, 스케일 1 %RH)
바이트 16:     배터리 (uint8, 스케일 1 %)
```

**인코딩 예제 (Python):**
```python
import struct
import hashlib
from datetime import datetime

def encode_air_quality_lora(device_id, pm25, pm10, temp, humidity, battery):
    # 메시지 유형
    msg_type = 0x01

    # 장치 ID 해시 (처음 4바이트)
    device_hash = hashlib.sha256(device_id.encode()).digest()[:4]

    # Unix 타임스탬프
    timestamp = int(datetime.utcnow().timestamp())

    # 스케일된 값
    pm25_scaled = int(pm25 * 10)      # 15.3 → 153
    pm10_scaled = int(pm10 * 10)      # 22.8 → 228
    temp_scaled = int(temp * 100)     # 20.5 → 2050

    # 바이트로 패킹
    payload = struct.pack(
        '>B4sIHHhBB',  # 형식: byte, 4bytes, uint32, 2*uint16, int16, 2*byte
        msg_type,
        device_hash,
        timestamp,
        pm25_scaled,
        pm10_scaled,
        temp_scaled,
        humidity,
        battery
    )

    return payload

# 예제 사용
payload = encode_air_quality_lora(
    device_id="ENV-AIR-001",
    pm25=15.3,
    pm10=22.8,
    temp=20.5,
    humidity=65,
    battery=85
)
print(f"Payload ({len(payload)} bytes): {payload.hex()}")
```

**디코딩 예제:**
```python
def decode_air_quality_lora(payload, device_lookup):
    # 바이트 언패킹
    msg_type, device_hash, timestamp, pm25_raw, pm10_raw, temp_raw, humidity, battery = struct.unpack(
        '>B4sIHHhBB',
        payload
    )

    # 장치 ID 조회
    device_id = device_lookup.get(device_hash.hex())

    # 값 다시 스케일링
    pm25 = pm25_raw / 10.0
    pm10 = pm10_raw / 10.0
    temp = temp_raw / 100.0

    # WIA 형식으로 변환
    wia_data = {
        "version": "1.0.0",
        "standard": "WIA-ENE-027",
        "deviceId": device_id,
        "timestamp": datetime.utcfromtimestamp(timestamp).isoformat() + "Z",
        "sensorType": "air_quality",
        "readings": {
            "pm2_5": {"value": pm25, "unit": "μg/m³"},
            "pm10": {"value": pm10, "unit": "μg/m³"},
            "temperature": {"value": temp, "unit": "°C"},
            "humidity": {"value": humidity, "unit": "%RH"}
        },
        "metadata": {"battery": battery}
    }

    return wia_data
```

### 적응형 데이터 속도

| 데이터 속도 | 확산 인자 | 최대 페이로드 | 권장 용도 |
|-----------|------------------|-------------|-----------------|
| DR0 | SF12 | 51 바이트 | 핵심 데이터만, 최대 범위 |
| DR2 | SF10 | 115 바이트 | 표준 배포 |
| DR4 | SF8 | 222 바이트 | 메타데이터가 포함된 전체 데이터 |

---

## 6.4 HTTP/REST 통신

### 데이터 제출

```http
POST /api/v1/sensors/ENV-AIR-001/data
Content-Type: application/json
Authorization: Bearer {token}

{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "pm2_5": {"value": 15.3, "unit": "μg/m³"}
  }
}
```

**cURL 예제:**
```bash
curl -X POST https://api.example.com/api/v1/sensors/ENV-AIR-001/data \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -d '{
    "version": "1.0.0",
    "standard": "WIA-ENE-027",
    "deviceId": "ENV-AIR-001",
    "timestamp": "2025-01-09T10:30:00.000Z",
    "sensorType": "air_quality",
    "readings": {
      "pm2_5": {"value": 15.3, "unit": "μg/m³"}
    }
  }'
```

---

## 6.5 보안 및 암호화

### 전송 보안

| 프로토콜 | 보안 메커니즘 | 포트 |
|----------|-------------------|------|
| MQTT | TLS 1.3 (MQTTS) | 8883 |
| CoAP | DTLS 1.3 (CoAPS) | 5684 |
| HTTP | TLS 1.3 (HTTPS) | 443 |
| LoRaWAN | AES-128 암호화 | 내장 |

### 인증서 기반 인증

```python
# TLS를 사용한 MQTT
import ssl

client = mqtt.Client()
client.tls_set(
    ca_certs="/path/to/ca.crt",
    certfile="/path/to/client.crt",
    keyfile="/path/to/client.key",
    tls_version=ssl.PROTOCOL_TLSv1_3
)
client.connect("broker.example.com", 8883)
```

---

## 6.6 데이터 검증 및 품질 관리

### 검증 단계

1. **스키마 검증**: JSON을 스키마와 비교
2. **범위 검증**: 값이 그럴듯한 범위 내에 있는지 확인
3. **시간 검증**: 타임스탬프 유효성 확인
4. **교차 매개변수 검증**: 관계 확인

**예제 검증:**
```python
def validate_sensor_data(data):
    errors = []

    # 스키마 검증
    if not all(k in data for k in ['version', 'deviceId', 'timestamp', 'readings']):
        errors.append("Missing required fields")

    # 범위 검증
    if 'pm2_5' in data.get('readings', {}):
        pm25 = data['readings']['pm2_5']['value']
        if pm25 < 0 or pm25 > 500:
            errors.append(f"PM2.5 value {pm25} out of range [0-500]")

    # 시간 검증
    timestamp = datetime.fromisoformat(data['timestamp'].replace('Z', '+00:00'))
    if timestamp > datetime.now(timezone.utc):
        errors.append("Future timestamp not allowed")

    return len(errors) == 0, errors
```

---

## 6.7 엣지 컴퓨팅 패턴

### 게이트웨이 책임

- 프로토콜 변환 (MQTT/CoAP/LoRaWAN → HTTP)
- 데이터 집계 및 필터링
- 로컬 이상 감지
- 연결 손실 시 버퍼링
- 엣지 분석

**예제 게이트웨이 로직:**
```python
class SensorGateway:
    def __init__(self):
        self.buffer = []
        self.mqtt_client = mqtt.Client()
        self.api_endpoint = "https://api.example.com/api/v1/sensors"

    def on_mqtt_message(self, client, userdata, msg):
        # MQTT를 통해 센서로부터 수신
        sensor_data = json.loads(msg.payload)

        # 검증
        valid, errors = validate_sensor_data(sensor_data)
        if not valid:
            print(f"Invalid data: {errors}")
            return

        # 엣지 처리: 이상 감지
        if self.detect_anomaly(sensor_data):
            sensor_data['quality']['flags'].append('rapid_change')

        # 클라우드 API로 전달
        self.forward_to_cloud(sensor_data)

    def forward_to_cloud(self, data):
        try:
            response = requests.post(
                f"{self.api_endpoint}/{data['deviceId']}/data",
                json=data,
                headers={"Authorization": f"Bearer {API_KEY}"}
            )
            response.raise_for_status()
        except requests.exceptions.RequestException:
            # 네트워크 실패 시 재시도를 위해 버퍼링
            self.buffer.append(data)
```

---

## 6.8 복습 문제 및 핵심 요점

### 복습 문제

1. 3개 도시에 걸친 100개 센서를 위한 MQTT 토픽 구조를 설계하세요. 도시의 모든 센서를 구독하기 위한 와일드카드를 포함하세요.

2. 수분, 온도, EC 및 NPK를 측정하는 토양 센서를 위한 LoRaWAN 페이로드 크기를 계산하세요. DR0 (51바이트)에 맞습니까?

3. 5분마다 보고하는 센서의 전력 소비를 비교하세요: (a) WiFi를 통한 MQTT, (b) 4G를 통한 CoAP, (c) LoRaWAN. 어느 것이 가장 전력 효율적입니까?

4. PM2.5 센서 고장을 감지하기 위한 엣지 처리 로직을 설계하세요. 어떤 품질 플래그를 설정하시겠습니까?

### 핵심 요점

1. **MQTT**: 연속 전원과 네트워크 연결을 가진 연결된 센서에 가장 적합합니다. QoS는 신뢰할 수 있는 전달을 보장합니다.

2. **CoAP**: UDP 전송을 사용하는 제약된 장치용으로 설계되었습니다. Observe 패턴은 효율적인 업데이트를 가능하게 합니다.

3. **LoRaWAN**: 원격, 배터리 구동 센서에 이상적입니다. 제한된 페이로드를 위해 컴팩트한 바이너리 인코딩이 필요합니다.

4. **HTTP/REST**: 데이터 제출을 위한 범용 프로토콜입니다. 간단하지만 MQTT/CoAP보다 오버헤드가 높습니다.

5. **보안**: MQTT/HTTP용 TLS 1.3, CoAP용 DTLS 1.3, LoRaWAN용 AES-128. 인증서 기반 인증 권장.

6. **검증**: 다층 검증 (스키마, 범위, 시간, 교차 매개변수)은 데이터 품질을 보장합니다.

7. **엣지 컴퓨팅**: 게이트웨이는 프로토콜 변환, 버퍼링 및 로컬 처리를 처리하여 클라우드 부하를 줄입니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity

**다음 장: [제7장: 시스템 통합](07-system-integration.md)**
