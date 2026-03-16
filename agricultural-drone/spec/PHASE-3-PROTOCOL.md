# WIA-AGRI-004: Agricultural Drone Standard
## Phase 3: Communication Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines communication protocols for agricultural drones, including MAVLink extensions, ground control station (GCS) protocols, and spray control systems.

### 1.1 Supported Protocols

- **MAVLink 2.0**: Primary flight control protocol
- **DroneKit Extensions**: Agricultural operation extensions
- **MQTT**: IoT sensor data and telemetry
- **WebSocket**: Real-time web client communication
- **LoRaWAN**: Long-range rural area communication

---

## 2. MAVLink Protocol Extensions

### 2.1 Agricultural MAVLink Messages

#### Message: AGRI_SPRAY_STATUS (ID: 12000)

Spray system status and control.

```c
// MAVLink Message Definition
typedef struct __mavlink_agri_spray_status_t {
    uint64_t time_usec;          // Timestamp (microseconds since UNIX epoch)
    float tank_level_percent;    // Tank level (0-100%)
    float flow_rate_lpm;         // Flow rate (liters per minute)
    float spray_rate_lpha;       // Spray rate (liters per hectare)
    float swath_width_m;         // Spray swath width (meters)
    uint8_t spray_active;        // Spray active (0=off, 1=on)
    uint8_t nozzle_count;        // Number of active nozzles
    uint8_t pump_status;         // Pump status (0=off, 1=on, 2=error)
    uint8_t nozzle_status[8];    // Individual nozzle status
} mavlink_agri_spray_status_t;
```

**Python Example:**
```python
from pymavlink import mavutil

# Send spray status
msg = connection.mav.agri_spray_status_send(
    time_usec=int(time.time() * 1e6),
    tank_level_percent=75.5,
    flow_rate_lpm=1.5,
    spray_rate_lpha=15.0,
    swath_width_m=4.5,
    spray_active=1,
    nozzle_count=4,
    pump_status=1,
    nozzle_status=[1, 1, 1, 1, 0, 0, 0, 0]
)
```

#### Message: AGRI_FIELD_INFO (ID: 12001)

Field information and crop data.

```c
typedef struct __mavlink_agri_field_info_t {
    uint64_t time_usec;
    char field_id[32];           // Field identifier
    float area_hectares;         // Field area
    uint8_t crop_type;           // Crop type enum
    uint8_t crop_stage;          // Growth stage (0-100%)
    float expected_yield;        // Expected yield (tons/ha)
    uint8_t soil_moisture;       // Soil moisture (0-100%)
    float soil_temperature;      // Soil temperature (°C)
} mavlink_agri_field_info_t;
```

#### Message: AGRI_NDVI_DATA (ID: 12002)

Real-time NDVI data from multispectral camera.

```c
typedef struct __mavlink_agri_ndvi_data_t {
    uint64_t time_usec;
    int32_t lat;                 // Latitude (degrees * 1e7)
    int32_t lon;                 // Longitude (degrees * 1e7)
    float ndvi_value;            // NDVI value (-1.0 to 1.0)
    uint16_t red_band;           // Red band value (0-65535)
    uint16_t nir_band;           // NIR band value (0-65535)
    uint8_t quality;             // Data quality (0-100%)
} mavlink_agri_ndvi_data_t;
```

### 2.2 MAVLink Command Extensions

#### CMD: MAV_CMD_AGRI_SPRAY_START (31000)

Start spray operation.

**Parameters:**
- `param1`: Spray rate (L/ha)
- `param2`: Swath width (meters)
- `param3`: Nozzle configuration (bitmask)
- `param4`: Reserved
- `param5`: Reserved
- `param6`: Reserved
- `param7`: Reserved

**Example:**
```python
# Start spraying at 15 L/ha with 4.5m swath
connection.mav.command_long_send(
    target_system,
    target_component,
    mavutil.mavlink.MAV_CMD_AGRI_SPRAY_START,
    0,  # confirmation
    15.0,  # param1: spray rate
    4.5,   # param2: swath width
    0b1111,  # param3: nozzles 1-4 active
    0, 0, 0, 0
)
```

#### CMD: MAV_CMD_AGRI_SPRAY_STOP (31001)

Stop spray operation.

#### CMD: MAV_CMD_AGRI_CALIBRATE_FLOW (31002)

Calibrate flow rate sensor.

---

## 3. DroneKit Agricultural Extensions

### 3.1 Spray Control Module

```python
from dronekit import connect, VehicleMode
import time

# Connect to vehicle
vehicle = connect('/dev/ttyUSB0', baud=57600)

# Agricultural drone extension
class AgriDrone:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.spray_active = False
        self.tank_level = 100.0

    def start_spray(self, rate_lpha=15.0, swath_m=4.5):
        """Start spray operation"""
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            31000,  # MAV_CMD_AGRI_SPRAY_START
            0,
            rate_lpha, swath_m, 0b1111,
            0, 0, 0, 0
        )
        self.vehicle.send_mavlink(msg)
        self.spray_active = True

    def stop_spray(self):
        """Stop spray operation"""
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            31001,  # MAV_CMD_AGRI_SPRAY_STOP
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.vehicle.send_mavlink(msg)
        self.spray_active = False

    def get_spray_status(self):
        """Get current spray status"""
        return {
            'active': self.spray_active,
            'tank_level': self.tank_level,
            'flow_rate': 1.5
        }

# Usage
agri = AgriDrone(vehicle)
agri.start_spray(rate_lpha=15.0)
time.sleep(60)  # Spray for 60 seconds
agri.stop_spray()
```

### 3.2 NDVI Mission Planner

```python
class NDVIMission:
    def __init__(self, vehicle, field_boundary):
        self.vehicle = vehicle
        self.field_boundary = field_boundary

    def create_survey_mission(self, altitude=50, overlap=0.7):
        """Create NDVI survey mission with camera triggers"""
        from pymavlink import mavutil

        waypoints = []
        # Generate lawnmower pattern
        for i, point in enumerate(self._generate_waypoints()):
            wp = mavutil.mavlink.MAVLink_mission_item_message(
                0, 0,
                i,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0,
                0, 0, 0, 0,
                point[0], point[1], altitude
            )
            waypoints.append(wp)

            # Add camera trigger
            trigger = mavutil.mavlink.MAVLink_mission_item_message(
                0, 0,
                i + 0.5,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
                0, 0,
                1, 0, 0, 0, 0, 0, 0
            )
            waypoints.append(trigger)

        return waypoints
```

---

## 4. MQTT Protocol for IoT Integration

### 4.1 Topic Structure

```
wia/agri-drone/{droneId}/telemetry          # Real-time telemetry
wia/agri-drone/{droneId}/spray/status       # Spray status
wia/agri-drone/{droneId}/battery            # Battery status
wia/agri-drone/{droneId}/command            # Commands to drone
wia/agri-drone/{droneId}/alert              # Alerts and warnings

wia/farm/{farmId}/weather                   # Farm weather data
wia/farm/{farmId}/field/{fieldId}/ndvi      # Field NDVI data
```

### 4.2 Message Payloads

**Telemetry Message:**
```json
{
  "timestamp": 1704110445123,
  "location": {
    "lat": 37.5665,
    "lon": 126.9780,
    "alt": 5.2
  },
  "velocity": {
    "speed": 3.5,
    "heading": 87.4
  },
  "battery": {
    "voltage": 22.4,
    "current": 15.2,
    "percentage": 78
  },
  "status": "SPRAYING"
}
```

**Command Message:**
```json
{
  "command": "START_SPRAY",
  "parameters": {
    "sprayRate": 15.0,
    "swathWidth": 4.5
  },
  "timestamp": 1704110445123
}
```

### 4.3 MQTT Client Example

```python
import paho.mqtt.client as mqtt
import json

class AgriDroneMQTT:
    def __init__(self, broker="mqtt.wia-agri.org", port=1883):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(broker, port)

    def on_connect(self, client, userdata, flags, rc):
        # Subscribe to drone commands
        client.subscribe("wia/agri-drone/+/command")

    def on_message(self, client, userdata, msg):
        data = json.loads(msg.payload)
        if data['command'] == 'START_SPRAY':
            self.handle_spray_command(data['parameters'])

    def publish_telemetry(self, drone_id, telemetry):
        topic = f"wia/agri-drone/{drone_id}/telemetry"
        self.client.publish(topic, json.dumps(telemetry))

    def publish_spray_status(self, drone_id, status):
        topic = f"wia/agri-drone/{drone_id}/spray/status"
        self.client.publish(topic, json.dumps(status))

# Usage
mqtt_client = AgriDroneMQTT()
mqtt_client.publish_telemetry("AGRI-DRONE-001", {
    "timestamp": time.time(),
    "location": {"lat": 37.5665, "lon": 126.9780}
})
```

---

## 5. LoRaWAN Protocol for Rural Areas

### 5.1 LoRaWAN Device Profile

**DevEUI:** Unique drone identifier
**AppEUI:** WIA Agricultural Network
**AppKey:** Encryption key

### 5.2 Uplink Messages

**Telemetry (Port 1):**
```
Byte 0-3:  Latitude (signed 32-bit, degrees * 1e7)
Byte 4-7:  Longitude (signed 32-bit, degrees * 1e7)
Byte 8-9:  Altitude (unsigned 16-bit, meters * 10)
Byte 10:   Battery percentage (0-100)
Byte 11:   Status byte (bit flags)
```

**Spray Status (Port 2):**
```
Byte 0:    Tank level (0-100%)
Byte 1:    Flow rate (L/min * 10)
Byte 2:    Spray active (0/1)
Byte 3:    Nozzle status (bitmask)
```

### 5.3 Downlink Commands

**Start Spray (Port 10):**
```
Byte 0:    Command: 0x01
Byte 1:    Spray rate (L/ha)
Byte 2:    Swath width (meters)
```

**Stop Spray (Port 10):**
```
Byte 0:    Command: 0x02
```

### 5.4 Python LoRaWAN Example

```python
import struct
from lorawan import LoRaWAN

class AgriDroneLoRa:
    def __init__(self, dev_eui, app_eui, app_key):
        self.lora = LoRaWAN(dev_eui, app_eui, app_key)

    def send_telemetry(self, lat, lon, alt, battery, status):
        # Pack data
        payload = struct.pack(
            '>iiHBB',
            int(lat * 1e7),
            int(lon * 1e7),
            int(alt * 10),
            battery,
            status
        )
        # Send on port 1
        self.lora.send(payload, port=1)

    def send_spray_status(self, tank, flow, active, nozzles):
        payload = struct.pack(
            'BBBB',
            tank,
            int(flow * 10),
            1 if active else 0,
            nozzles
        )
        self.lora.send(payload, port=2)
```

---

## 6. WebSocket Protocol for Real-Time Clients

### 6.1 Connection & Authentication

```javascript
const ws = new WebSocket('wss://api.wia-agri.org/v1/ws');

ws.onopen = () => {
  // Authenticate
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'YOUR_JWT_TOKEN'
  }));

  // Subscribe to drone telemetry
  ws.send(JSON.stringify({
    type: 'subscribe',
    channel: 'drone.AGRI-DRONE-001.telemetry'
  }));
};

ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);

  switch(msg.type) {
    case 'telemetry':
      handleTelemetry(msg.data);
      break;
    case 'alert':
      handleAlert(msg.data);
      break;
  }
};
```

### 6.2 Message Types

**Telemetry Update:**
```json
{
  "type": "telemetry",
  "channel": "drone.AGRI-DRONE-001.telemetry",
  "timestamp": 1704110445123,
  "data": {
    "location": {"lat": 37.5665, "lon": 126.9780},
    "battery": 78,
    "status": "SPRAYING"
  }
}
```

**Command:**
```json
{
  "type": "command",
  "target": "AGRI-DRONE-001",
  "command": "START_SPRAY",
  "params": {
    "sprayRate": 15.0
  }
}
```

**Alert:**
```json
{
  "type": "alert",
  "severity": "WARNING",
  "message": "Battery below 30%",
  "droneId": "AGRI-DRONE-001",
  "timestamp": 1704110445123
}
```

---

## 7. Spray System Communication Protocol

### 7.1 Serial Protocol (RS-485)

**Baud Rate:** 115200
**Data Bits:** 8
**Parity:** None
**Stop Bits:** 1

### 7.2 Command Structure

```
Start Byte: 0xAA
Command ID: 1 byte
Data Length: 1 byte
Data: N bytes
Checksum: 2 bytes (CRC16)
End Byte: 0x55
```

### 7.3 Commands

**Start Spray (0x01):**
```
AA 01 04 [rate_lpha][swath_m][nozzles] [CRC16] 55
```

**Stop Spray (0x02):**
```
AA 02 00 [CRC16] 55
```

**Get Status (0x03):**
```
AA 03 00 [CRC16] 55
```

**Status Response:**
```
AA 83 06 [tank_level][flow_rate][pump_status][nozzles] [CRC16] 55
```

### 7.4 Arduino Example

```cpp
#include <CRC16.h>

class SprayController {
public:
    void sendStartSpray(float rate, float swath, byte nozzles) {
        byte cmd[] = {0xAA, 0x01, 0x04};
        byte data[] = {
            (byte)(rate * 10),
            (byte)(swath * 10),
            nozzles
        };

        uint16_t crc = CRC16.calculate(cmd, sizeof(cmd), data, sizeof(data));

        Serial.write(cmd, sizeof(cmd));
        Serial.write(data, sizeof(data));
        Serial.write((byte)(crc >> 8));
        Serial.write((byte)(crc & 0xFF));
        Serial.write(0x55);
    }

    void sendStopSpray() {
        byte packet[] = {0xAA, 0x02, 0x00, 0x00, 0x00, 0x55};
        uint16_t crc = CRC16.calculate(packet, 3);
        packet[3] = crc >> 8;
        packet[4] = crc & 0xFF;
        Serial.write(packet, sizeof(packet));
    }
};
```

---

## 8. Camera Trigger Protocol

### 8.1 PWM Trigger

**Frequency:** 50 Hz (20ms period)
**Trigger Pulse:** 1500μs high pulse
**Idle:** 1000μs high pulse

### 8.2 USB Trigger (PTP Protocol)

```python
import gphoto2 as gp

class CameraTrigger:
    def __init__(self):
        self.camera = gp.Camera()
        self.camera.init()

    def trigger(self):
        """Trigger camera capture"""
        file_path = self.camera.capture(gp.GP_CAPTURE_IMAGE)
        return file_path

    def set_interval(self, interval_sec):
        """Set interval capture mode"""
        config = self.camera.get_config()
        interval = config.get_child_by_name('interval')
        interval.set_value(str(interval_sec))
        self.camera.set_config(config)
```

---

## 9. Weather Station Integration Protocol

### 9.1 Modbus RTU

**Slave Address:** 1
**Baud Rate:** 9600
**Function Code:** 0x03 (Read Holding Registers)

**Registers:**
```
0x0000: Temperature (°C * 10)
0x0001: Humidity (%)
0x0002: Wind Speed (m/s * 10)
0x0003: Wind Direction (degrees)
0x0004: Pressure (hPa * 10)
0x0005: Rainfall (mm * 10)
```

### 9.2 Python Example

```python
from pymodbus.client.sync import ModbusSerialClient

class WeatherStation:
    def __init__(self, port='/dev/ttyUSB1'):
        self.client = ModbusSerialClient(
            method='rtu',
            port=port,
            baudrate=9600,
            timeout=1
        )

    def read_weather(self):
        result = self.client.read_holding_registers(0, 6, unit=1)

        return {
            'temperature': result.registers[0] / 10.0,
            'humidity': result.registers[1],
            'windSpeed': result.registers[2] / 10.0,
            'windDirection': result.registers[3],
            'pressure': result.registers[4] / 10.0,
            'rainfall': result.registers[5] / 10.0
        }
```

---

## 10. Ground Control Station Protocol

### 10.1 GCS Connection Flow

```
1. GCS connects via MAVLink (TCP/UDP)
2. Heartbeat exchange
3. Request parameter list
4. Request mission items
5. Monitor telemetry stream
6. Send commands as needed
```

### 10.2 Mission Upload Protocol

```python
from pymavlink import mavutil

def upload_mission(connection, waypoints):
    # Send mission count
    connection.mav.mission_count_send(
        connection.target_system,
        connection.target_component,
        len(waypoints)
    )

    # Wait for mission request
    for i, wp in enumerate(waypoints):
        msg = connection.recv_match(type='MISSION_REQUEST', blocking=True)
        if msg.seq == i:
            connection.mav.send(wp)

    # Wait for ACK
    ack = connection.recv_match(type='MISSION_ACK', blocking=True)
    return ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
```

---

## 11. Protocol Security

### 11.1 Encryption

- **MAVLink 2.0**: AES-128 encryption
- **MQTT**: TLS 1.3
- **WebSocket**: WSS (WebSocket Secure)
- **LoRaWAN**: AES-128 end-to-end encryption

### 11.2 Authentication

```python
# MAVLink signing
from pymavlink import mavutil

connection = mavutil.mavlink_connection(
    'tcp:127.0.0.1:5760',
    signing=True,
    secret_key=b'YOUR_SECRET_KEY'
)
```

---

**© 2025 WIA Standards | MIT License**
**弘益人間 · Benefit All Humanity**
