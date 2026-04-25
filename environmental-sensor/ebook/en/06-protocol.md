# Chapter 6: Communication Protocols

## Phase 3: MQTT, CoAP, LoRaWAN, and HTTP Protocols

---

## 6.1 MQTT Protocol Specification

MQTT (Message Queuing Telemetry Transport) is ideal for connected sensors with continuous power and reliable network connectivity.

### Topic Structure

```
wia/{standard}/{region}/{location}/{deviceId}/{messageType}
```

**Examples:**
```
wia/env027/us-west/seattle/ENV-AIR-001/data
wia/env027/kr-seoul/gangnam/WATER-001/status
wia/env027/eu-london/zone3/SOIL-FARM-A-001/calibration
```

### Message Types

| Topic Suffix | Purpose | QoS | Retained |
|--------------|---------|-----|----------|
| `/data` | Sensor measurements | 1 | No |
| `/status` | Device health | 1 | Yes |
| `/calibration` | Calibration data | 2 | Yes |
| `/command` | Control commands | 2 | No |
| `/alert` | Threshold alerts | 1 | No |

### Publishing Sensor Data

**Topic:** `wia/env027/us-west/seattle/ENV-AIR-001/data`

**Payload (JSON):**
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

**Python Example:**
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

### Connection Management

**Client ID:** `wia-{deviceId}-{randomString}`

**Keep-Alive:** 60-300 seconds (based on reporting frequency)

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

## 6.2 CoAP for Constrained Devices

CoAP (Constrained Application Protocol) is designed for resource-limited devices with UDP transport.

### Resource Structure

```
/wia/env/{resource}
```

**Standard Resources:**
```
/wia/env/data          - Current measurements
/wia/env/status        - Device status
/wia/env/config        - Configuration
/wia/env/calibration   - Calibration data
```

### Resource Discovery

```
GET coap://sensor.local/.well-known/core

Response:
</wia/env/data>;ct=50;rt="wia.env.data";obs,
</wia/env/status>;ct=50;rt="wia.env.status";obs,
</wia/env/config>;ct=50;rt="wia.env.config"
```

### GET Data

```
GET coap://sensor.local/wia/env/data
Accept: application/json

Response (2.05 Content):
{
  "deviceId": "ENV-SOIL-001",
  "timestamp": "2025-01-09T10:30:00Z",
  "readings": {
    "moisture": {"value": 28.5, "unit": "%VWC"}
  }
}
```

### POST Data to Cloud

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

Response (2.01 Created)
```

### Observe Pattern

```
GET coap://sensor.local/wia/env/data
Observe: 0

Response sequence:
2.05 Content (Observe: 0)  {"moisture": 28.5, ...}
2.05 Content (Observe: 1)  {"moisture": 28.7, ...}
2.05 Content (Observe: 2)  {"moisture": 28.9, ...}
```

---

## 6.3 LoRaWAN for Long-Range Sensors

LoRaWAN enables long-range, low-power communication for remote sensors.

### Payload Encoding

**Compact Binary Format (17 bytes):**

```
Byte 0:      Message Type (0x01 = data)
Byte 1-4:    Device ID hash (4 bytes)
Byte 5-8:    Unix timestamp (4 bytes)
Byte 9-10:   PM2.5 (uint16, scale 0.1 μg/m³)
Byte 11-12:  PM10 (uint16, scale 0.1 μg/m³)
Byte 13-14:  Temperature (int16, scale 0.01 °C)
Byte 15:     Humidity (uint8, scale 1 %RH)
Byte 16:     Battery (uint8, scale 1 %)
```

**Encoding Example (Python):**
```python
import struct
import hashlib
from datetime import datetime

def encode_air_quality_lora(device_id, pm25, pm10, temp, humidity, battery):
    # Message type
    msg_type = 0x01

    # Device ID hash (first 4 bytes)
    device_hash = hashlib.sha256(device_id.encode()).digest()[:4]

    # Unix timestamp
    timestamp = int(datetime.utcnow().timestamp())

    # Scaled values
    pm25_scaled = int(pm25 * 10)      # 15.3 → 153
    pm10_scaled = int(pm10 * 10)      # 22.8 → 228
    temp_scaled = int(temp * 100)     # 20.5 → 2050

    # Pack into bytes
    payload = struct.pack(
        '>B4sIHHhBB',  # Format: byte, 4bytes, uint32, 2*uint16, int16, 2*byte
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

# Example usage
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

**Decoding Example:**
```python
def decode_air_quality_lora(payload, device_lookup):
    # Unpack bytes
    msg_type, device_hash, timestamp, pm25_raw, pm10_raw, temp_raw, humidity, battery = struct.unpack(
        '>B4sIHHhBB',
        payload
    )

    # Lookup device ID
    device_id = device_lookup.get(device_hash.hex())

    # Scale values back
    pm25 = pm25_raw / 10.0
    pm10 = pm10_raw / 10.0
    temp = temp_raw / 100.0

    # Convert to WIA format
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

### Adaptive Data Rate

| Data Rate | Spreading Factor | Max Payload | Recommended Use |
|-----------|------------------|-------------|-----------------|
| DR0 | SF12 | 51 bytes | Core data only, maximum range |
| DR2 | SF10 | 115 bytes | Standard deployment |
| DR4 | SF8 | 222 bytes | Full data with metadata |

---

## 6.4 HTTP/REST Communication

### Data Submission

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

**cURL Example:**
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

## 6.5 Security and Encryption

### Transport Security

| Protocol | Security Mechanism | Port |
|----------|-------------------|------|
| MQTT | TLS 1.3 (MQTTS) | 8883 |
| CoAP | DTLS 1.3 (CoAPS) | 5684 |
| HTTP | TLS 1.3 (HTTPS) | 443 |
| LoRaWAN | AES-128 encryption | Built-in |

### Certificate-Based Authentication

```python
# MQTT with TLS
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

## 6.6 Data Validation and Quality Control

### Validation Steps

1. **Schema Validation**: Check JSON against schema
2. **Range Validation**: Verify values within plausible ranges
3. **Temporal Validation**: Check timestamp validity
4. **Cross-Parameter Validation**: Verify relationships

**Example Validation:**
```python
def validate_sensor_data(data):
    errors = []

    # Schema validation
    if not all(k in data for k in ['version', 'deviceId', 'timestamp', 'readings']):
        errors.append("Missing required fields")

    # Range validation
    if 'pm2_5' in data.get('readings', {}):
        pm25 = data['readings']['pm2_5']['value']
        if pm25 < 0 or pm25 > 500:
            errors.append(f"PM2.5 value {pm25} out of range [0-500]")

    # Temporal validation
    timestamp = datetime.fromisoformat(data['timestamp'].replace('Z', '+00:00'))
    if timestamp > datetime.now(timezone.utc):
        errors.append("Future timestamp not allowed")

    return len(errors) == 0, errors
```

---

## 6.7 Edge Computing Patterns

### Gateway Responsibilities

- Protocol translation (MQTT/CoAP/LoRaWAN → HTTP)
- Data aggregation and filtering
- Local anomaly detection
- Buffering during connectivity loss
- Edge analytics

**Example Gateway Logic:**
```python
class SensorGateway:
    def __init__(self):
        self.buffer = []
        self.mqtt_client = mqtt.Client()
        self.api_endpoint = "https://api.example.com/api/v1/sensors"

    def on_mqtt_message(self, client, userdata, msg):
        # Receive from sensor via MQTT
        sensor_data = json.loads(msg.payload)

        # Validate
        valid, errors = validate_sensor_data(sensor_data)
        if not valid:
            print(f"Invalid data: {errors}")
            return

        # Edge processing: detect anomalies
        if self.detect_anomaly(sensor_data):
            sensor_data['quality']['flags'].append('rapid_change')

        # Forward to cloud API
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
            # Buffer for retry if network fails
            self.buffer.append(data)
```

---

## 6.8 Review Questions and Key Takeaways

### Review Questions

1. Design an MQTT topic structure for 100 sensors across 3 cities. Include wildcards for subscribing to all sensors in a city.

2. Calculate LoRaWAN payload size for soil sensor measuring moisture, temperature, EC, and NPK. Can it fit in DR0 (51 bytes)?

3. Compare power consumption for sensor reporting every 5 minutes via: (a) MQTT over WiFi, (b) CoAP over 4G, (c) LoRaWAN. Which is most power-efficient?

4. Design edge processing logic to detect PM2.5 sensor failures. What quality flags would you set?

### Key Takeaways

1. **MQTT**: Best for connected sensors with continuous power and network connectivity. QoS ensures reliable delivery.

2. **CoAP**: Designed for constrained devices with UDP transport. Observe pattern enables efficient updates.

3. **LoRaWAN**: Ideal for remote, battery-powered sensors. Requires compact binary encoding for limited payload.

4. **HTTP/REST**: Universal protocol for data submission. Simple but higher overhead than MQTT/CoAP.

5. **Security**: TLS 1.3 for MQTT/HTTP, DTLS 1.3 for CoAP, AES-128 for LoRaWAN. Certificate-based authentication recommended.

6. **Validation**: Multi-layer validation (schema, range, temporal, cross-parameter) ensures data quality.

7. **Edge Computing**: Gateways handle protocol translation, buffering, and local processing to reduce cloud load.

---

© 2025 WIA Standards Committee. 弘익인간 (홍익인간) - Benefit All Humanity

**Next Chapter: [Chapter 7: System Integration](07-system-integration.md)**
