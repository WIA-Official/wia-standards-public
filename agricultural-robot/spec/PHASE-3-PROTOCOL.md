# WIA-AGRI-003: Agricultural Robot Standard
## Phase 3: Protocol Specification

**Version:** 1.2.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26
**Category:** Agriculture (AGRI)

---

## 1. Overview

### 1.1 Purpose
This specification defines communication protocols for agricultural robots, including ROS2 topics, CAN bus messaging, MQTT broker configuration, and industrial automation protocols. These protocols enable real-time control, sensor data streaming, and inter-robot communication.

### 1.2 Protocol Stack

```
┌────────────────────────────────────┐
│   Application Layer (Farm Apps)   │
├────────────────────────────────────┤
│   WIA API Layer (REST/GraphQL)    │
├────────────────────────────────────┤
│  Middleware (ROS2, MQTT, DDS)     │
├────────────────────────────────────┤
│   Transport (TCP, UDP, CAN)       │
├────────────────────────────────────┤
│   Physical (Ethernet, WiFi, LoRa) │
└────────────────────────────────────┘
```

---

## 2. ROS2 Integration

### 2.1 ROS2 Topics

#### 2.1.1 Telemetry Publishing

**Topic:** `/agri_robot/telemetry`
**Message Type:** `wia_agri_msgs/Telemetry`

```python
# wia_agri_msgs/Telemetry.msg
Header header
string robot_id
string robot_type

# Location
sensor_msgs/NavSatFix gps_position
float64 heading

# Status
string work_mode
uint8 battery_level
float32 speed
float32 temperature

# Sensors
float32 soil_moisture
float32 soil_temperature
float32 air_temperature
float32 humidity
float32 crop_health
```

**Publishing Example:**
```python
import rclpy
from wia_agri_msgs.msg import Telemetry
from sensor_msgs.msg import NavSatFix

node = rclpy.create_node('agri_robot_telemetry')
pub = node.create_publisher(Telemetry, '/agri_robot/telemetry', 10)

msg = Telemetry()
msg.robot_id = 'AGRI-ROBOT-2025-001'
msg.robot_type = 'autonomous-tractor'
msg.work_mode = 'plowing'
msg.battery_level = 85
msg.speed = 5.2

pub.publish(msg)
```

---

#### 2.1.2 Command Subscription

**Topic:** `/agri_robot/command`
**Message Type:** `wia_agri_msgs/Command`

```python
# wia_agri_msgs/Command.msg
Header header
string command_id
string robot_id
string command_type  # start, stop, pause, resume, emergency_stop
string task_id
geometry_msgs/PoseStamped target_pose
```

**Subscription Example:**
```python
def command_callback(msg):
    if msg.command_type == 'emergency_stop':
        # Immediate halt
        stop_all_motors()
    elif msg.command_type == 'pause':
        # Pause current task
        pause_task(msg.task_id)

sub = node.create_subscription(
    Command,
    '/agri_robot/command',
    command_callback,
    10
)
```

---

#### 2.1.3 Task Assignment

**Topic:** `/agri_robot/task`
**Message Type:** `wia_agri_msgs/Task`

```python
# wia_agri_msgs/Task.msg
Header header
string task_id
string robot_id
string field_id
string task_type  # plowing, harvesting, weeding, spraying, seeding
uint8 priority  # 0=low, 1=medium, 2=high, 3=urgent
time scheduled_start
uint32 estimated_duration  # minutes
TaskParameters parameters
```

---

#### 2.1.4 Sensor Data Streams

**Topic:** `/agri_robot/sensors/multispectral`
**Message Type:** `sensor_msgs/Image`

```python
# Multispectral camera for NDVI calculation
pub_ms = node.create_publisher(
    Image,
    '/agri_robot/sensors/multispectral',
    10
)
```

**Topic:** `/agri_robot/sensors/lidar`
**Message Type:** `sensor_msgs/PointCloud2`

```python
# 3D LIDAR for obstacle detection
pub_lidar = node.create_publisher(
    PointCloud2,
    '/agri_robot/sensors/lidar',
    10
)
```

---

### 2.2 ROS2 Services

#### 2.2.1 Field Map Request

**Service:** `/agri_robot/get_field_map`
**Type:** `wia_agri_srvs/GetFieldMap`

```python
# wia_agri_srvs/GetFieldMap.srv
# Request
string field_id
---
# Response
bool success
string message
geometry_msgs/Polygon boundary
Obstacle[] obstacles
```

**Client Example:**
```python
client = node.create_client(GetFieldMap, '/agri_robot/get_field_map')
request = GetFieldMap.Request()
request.field_id = 'FIELD-KR-2025-042'

future = client.call_async(request)
response = future.result()
```

---

#### 2.2.2 Path Planning

**Service:** `/agri_robot/plan_path`
**Type:** `wia_agri_srvs/PlanPath`

```python
# wia_agri_srvs/PlanPath.srv
# Request
geometry_msgs/PoseStamped start
geometry_msgs/PoseStamped goal
string algorithm  # astar, dijkstra, rrt, dwa
---
# Response
bool success
nav_msgs/Path path
float32 estimated_distance
float32 estimated_time
```

---

### 2.3 ROS2 Actions

#### 2.3.1 Execute Task

**Action:** `/agri_robot/execute_task`
**Type:** `wia_agri_actions/ExecuteTask`

```python
# wia_agri_actions/ExecuteTask.action
# Goal
string task_id
string task_type
geometry_msgs/Polygon work_area
---
# Result
bool success
string message
float32 area_completed
uint32 duration
---
# Feedback
uint8 progress  # 0-100
geometry_msgs/PoseStamped current_pose
string current_status
```

**Client Example:**
```python
from wia_agri_actions.action import ExecuteTask
from rclpy.action import ActionClient

action_client = ActionClient(node, ExecuteTask, '/agri_robot/execute_task')

goal = ExecuteTask.Goal()
goal.task_id = 'TASK-2025-001-H42'
goal.task_type = 'harvesting'

future = action_client.send_goal_async(goal, feedback_callback)
```

---

## 3. CAN Bus Protocol

### 3.1 CAN Frame Structure

```
┌─────┬─────┬─────┬──────────┬─────┐
│ SOF │ ID  │ RTR │   DATA   │ CRC │
└─────┴─────┴─────┴──────────┴─────┘
  1bit 11bit  1bit   0-64bit   15bit
```

### 3.2 CAN Identifier Allocation

| CAN ID Range | Purpose | Priority |
|--------------|---------|----------|
| 0x000-0x0FF | Emergency stop, safety | Highest |
| 0x100-0x1FF | Motor control | High |
| 0x200-0x2FF | Sensor data | Medium |
| 0x300-0x3FF | Navigation | Medium |
| 0x400-0x4FF | Diagnostics | Low |
| 0x500-0x5FF | Configuration | Lowest |

### 3.3 Standard Messages

#### 3.3.1 Emergency Stop (CAN ID: 0x001)

```
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ 0xFF │ 0x00 │ 0x00 │ 0x00 │ 0x00 │ 0x00 │ 0x00 │ 0x00 │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
  CMD    Reason Reserved
```

#### 3.3.2 Motor Control (CAN ID: 0x100)

```
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ 0x01 │ SpH  │ SpL  │ Dir  │ Pwr  │ 0x00 │ 0x00 │ 0x00 │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
  CMD   Speed(H) Speed(L) Direction Power
```

- Speed: 16-bit unsigned int (0-65535) in cm/s
- Direction: 0=forward, 1=reverse, 2=left, 3=right
- Power: 0-100%

#### 3.3.3 GPS Position (CAN ID: 0x300)

```
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ LatH │ LatM │ LatL │ 0x00 │ LonH │ LonM │ LonL │ 0x00 │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
  Latitude (24-bit)       Longitude (24-bit)
```

Latitude/Longitude: Signed 24-bit integer × 10^-6 degrees

#### 3.3.4 Battery Status (CAN ID: 0x250)

```
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ SOC  │ VltH │ VltL │ CurH │ CurL │ Temp │ 0x00 │ 0x00 │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
  SOC   Voltage    Current    Temp
```

- SOC: State of Charge (0-100%)
- Voltage: 16-bit unsigned int in mV
- Current: 16-bit signed int in mA
- Temp: Signed 8-bit in °C

---

### 3.4 CAN Bus Configuration

```ini
[CAN0]
interface = can0
bitrate = 500000  # 500 kbps
sample_point = 0.875
sjw = 1
tseg1 = 6
tseg2 = 1
```

**Linux Configuration:**
```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

---

## 4. MQTT Protocol

### 4.1 Broker Configuration

```yaml
mqtt:
  broker: mqtt.wiastandards.com
  port: 8883  # TLS
  protocol: MQTTv5
  keepalive: 60
  qos: 1  # At least once delivery
  retain: false
```

### 4.2 Topic Hierarchy

```
wia/
├── agri/
│   ├── {farm-id}/
│   │   ├── fleet/
│   │   │   ├── status
│   │   │   └── coordination
│   │   ├── robots/
│   │   │   ├── {robot-id}/
│   │   │   │   ├── telemetry
│   │   │   │   ├── status
│   │   │   │   ├── command
│   │   │   │   └── sensors/
│   │   │   │       ├── gps
│   │   │   │       ├── imu
│   │   │   │       └── camera
│   │   └── fields/
│   │       ├── {field-id}/
│   │       │   ├── map
│   │       │   ├── health
│   │       │   └── weather
```

### 4.3 Message Formats

#### 4.3.1 Telemetry (QoS 1)

**Topic:** `wia/agri/{farm-id}/robots/{robot-id}/telemetry`

```json
{
  "timestamp": "2025-12-26T15:45:22Z",
  "robotId": "AGRI-ROBOT-2025-001",
  "location": {
    "lat": 37.5665,
    "lon": 126.9780,
    "alt": 120.5
  },
  "status": {
    "mode": "plowing",
    "battery": 85,
    "speed": 5.2
  }
}
```

#### 4.3.2 Command (QoS 2)

**Topic:** `wia/agri/{farm-id}/robots/{robot-id}/command`

```json
{
  "commandId": "CMD-2025-042",
  "type": "pause",
  "timestamp": "2025-12-26T15:46:00Z",
  "ttl": 30
}
```

#### 4.3.3 Fleet Coordination (QoS 1)

**Topic:** `wia/agri/{farm-id}/fleet/coordination`

```json
{
  "timestamp": "2025-12-26T15:50:00Z",
  "assignments": [
    {
      "robotId": "AGRI-ROBOT-2025-001",
      "zone": "ZONE-A",
      "priority": "high"
    },
    {
      "robotId": "AGRI-ROBOT-2025-002",
      "zone": "ZONE-B",
      "priority": "medium"
    }
  ]
}
```

---

### 4.4 Last Will and Testament (LWT)

```json
{
  "topic": "wia/agri/{farm-id}/robots/{robot-id}/status",
  "payload": {
    "status": "offline",
    "reason": "connection-lost",
    "timestamp": "2025-12-26T15:55:00Z"
  },
  "qos": 1,
  "retain": true
}
```

---

## 5. Industrial Protocols

### 5.1 Modbus TCP

**Connection:** `{robot-ip}:502`

#### Register Mapping

| Address | Type | Description | Unit |
|---------|------|-------------|------|
| 40001 | Holding | Battery Level | % |
| 40002 | Holding | Speed | cm/s |
| 40003 | Holding | Work Mode | enum |
| 40004-40005 | Holding | GPS Latitude | deg × 10^6 |
| 40006-40007 | Holding | GPS Longitude | deg × 10^6 |
| 40008 | Holding | Soil Moisture | % |
| 40009 | Holding | Temperature | °C × 10 |

#### Function Codes

- **0x03:** Read Holding Registers (status read)
- **0x06:** Write Single Register (command)
- **0x10:** Write Multiple Registers (configuration)

---

### 5.2 OPC UA

**Endpoint:** `opc.tcp://{robot-ip}:4840`

#### Node Hierarchy

```
Objects/
├── AgriRobot/
│   ├── Identification/
│   │   ├── RobotId
│   │   ├── Manufacturer
│   │   └── SerialNumber
│   ├── Status/
│   │   ├── WorkMode
│   │   ├── Battery
│   │   └── Operational
│   ├── Location/
│   │   ├── Latitude
│   │   ├── Longitude
│   │   └── Heading
│   ├── Sensors/
│   │   ├── SoilMoisture
│   │   ├── Temperature
│   │   └── CropHealth
│   └── Control/
│       ├── Start()
│       ├── Stop()
│       └── EmergencyStop()
```

---

### 5.3 ISOBUS (ISO 11783)

**Physical:** CAN bus @ 250 kbps

#### Task Controller (TC) Messages

```
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ PGN  │ Prio │ SA   │ Data (0-1785 bytes)              │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
```

**PGN 0xCF00:** Process Data (work rate, area)
**PGN 0xE700:** Task Controller Status
**PGN 0xFE09:** Vehicle Position

---

## 6. Network Topology

### 6.1 Farm Network Architecture

```
                 Internet
                    │
              ┌─────▼─────┐
              │  Gateway  │
              │  Router   │
              └─────┬─────┘
                    │
        ┌───────────┼───────────┐
        │           │           │
    ┌───▼───┐   ┌──▼──┐    ┌───▼───┐
    │ WiFi  │   │LoRa │    │4G/LTE │
    │ AP    │   │ GW  │    │Modem  │
    └───┬───┘   └──┬──┘    └───┬───┘
        │          │           │
    ┌───▼──────────▼───────────▼───┐
    │        Farm Network          │
    │   (192.168.100.0/24)        │
    └──┬──────┬──────┬──────┬─────┘
       │      │      │      │
    Robot1 Robot2 Robot3 Sensors
```

### 6.2 Communication Matrix

| Source | Destination | Protocol | Latency | Bandwidth |
|--------|-------------|----------|---------|-----------|
| Robot | Cloud | MQTT/TLS | <500ms | 10 kbps |
| Robot | Robot | CAN/Ethernet | <100ms | 1 Mbps |
| Robot | Gateway | WiFi/LoRa | <200ms | 100 kbps |
| Sensors | Robot | CAN/I2C | <50ms | 1 Mbps |

---

## 7. Time Synchronization

### 7.1 NTP Configuration

```yaml
ntp:
  servers:
    - time.nist.gov
    - time.google.com
    - ntp.wiastandards.com
  max_drift: 100ms
  sync_interval: 3600s
```

### 7.2 PTP (Precision Time Protocol)

For high-precision applications (cm-level GPS):

```ini
[global]
domain_number = 0
priority1 = 128
priority2 = 128
clock_servo = linreg
```

---

## 8. Security Protocols

### 8.1 TLS Configuration

```yaml
tls:
  version: 1.3
  cipher_suites:
    - TLS_AES_256_GCM_SHA384
    - TLS_CHACHA20_POLY1305_SHA256
  certificate_validation: strict
  client_auth: required
```

### 8.2 Message Authentication

```json
{
  "message": {...},
  "signature": {
    "algorithm": "Ed25519",
    "publicKey": "did:wia:robot:key-1",
    "value": "base64-encoded-signature"
  }
}
```

---

## 9. Protocol Testing

### 9.1 ROS2 Testing

```bash
# Publish test telemetry
ros2 topic pub /agri_robot/telemetry wia_agri_msgs/Telemetry \
  "{robot_id: 'TEST-001', battery_level: 85}"

# Echo commands
ros2 topic echo /agri_robot/command
```

### 9.2 CAN Bus Testing

```bash
# Send emergency stop
cansend can0 001#FF00000000000000

# Monitor CAN traffic
candump can0
```

### 9.3 MQTT Testing

```bash
# Subscribe to telemetry
mosquitto_sub -h mqtt.wiastandards.com -p 8883 \
  -t "wia/agri/+/robots/+/telemetry" --cafile ca.crt

# Publish command
mosquitto_pub -h mqtt.wiastandards.com -p 8883 \
  -t "wia/agri/FARM-001/robots/ROBOT-001/command" \
  -m '{"type":"pause"}'
```

---

## 10. Integration Examples

### 10.1 Multi-Protocol Bridge

```python
class AgriRobotBridge:
    def __init__(self):
        self.ros_node = rclpy.create_node('bridge')
        self.mqtt_client = mqtt.Client()
        self.can_bus = can.interface.Bus(channel='can0')

    def ros_to_mqtt(self, ros_msg):
        mqtt_payload = {
            'robotId': ros_msg.robot_id,
            'battery': ros_msg.battery_level
        }
        self.mqtt_client.publish(
            f'wia/agri/FARM-001/robots/{ros_msg.robot_id}/telemetry',
            json.dumps(mqtt_payload)
        )

    def can_to_ros(self, can_frame):
        if can_frame.arbitration_id == 0x250:  # Battery status
            msg = Telemetry()
            msg.battery_level = can_frame.data[0]
            self.ros_publisher.publish(msg)
```

---

**© 2025 WIA (World Certification Industry Association)**
**弘益人間 · Benefit All Humanity**
**License:** MIT
