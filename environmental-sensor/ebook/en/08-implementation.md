# Chapter 8: Implementation Guide

## Deploying WIA-ENE-027 Compliant Environmental Sensor Systems

---

## 8.1 Implementation Roadmap

### Phase-Based Deployment

**Phase 1: Planning (Weeks 1-4)**
- Define requirements and use cases
- Select sensor types and locations
- Choose cloud platform and database
- Design system architecture
- Budget and resource allocation

**Phase 2: Procurement (Weeks 5-8)**
- Purchase sensors and gateways
- Set up cloud accounts
- Obtain domain and SSL certificates
- Establish development environment

**Phase 3: Development (Weeks 9-16)**
- Implement data ingestion pipeline
- Develop API endpoints
- Configure database schemas
- Build dashboards and visualizations
- Set up monitoring and alerting

**Phase 4: Testing (Weeks 17-20)**
- Unit testing (data validation, API endpoints)
- Integration testing (end-to-end data flow)
- Load testing (simulate peak traffic)
- Security testing (penetration testing)
- User acceptance testing

**Phase 5: Deployment (Weeks 21-24)**
- Install sensors
- Configure gateways
- Deploy cloud infrastructure
- Migrate to production
- Train operators

**Phase 6: Operations (Ongoing)**
- Monitor system health
- Calibrate sensors
- Handle maintenance
- Optimize performance
- Expand deployment

---

## 8.2 Hardware Selection and Deployment

### Sensor Selection Criteria

| Criteria | Questions to Ask | Importance |
|----------|------------------|------------|
| **Accuracy** | Does it meet measurement requirements? | Critical |
| **Cost** | Fits budget for planned scale? | High |
| **Power** | Battery life or grid power needed? | High |
| **Connectivity** | WiFi, cellular, LoRaWAN available? | Critical |
| **Durability** | IP rating for environmental protection? | Medium |
| **Calibration** | How often? Can be done in-field? | High |
| **WIA Compliance** | Can output Phase 1 format? | Critical |

### Example Sensor Selections

**Urban Air Quality Network:**
- Sensor: PurpleAir PA-II ($280)
- Connectivity: WiFi
- Power: AC adapter
- Measurements: PM1.0, PM2.5, PM10, temperature, humidity
- Deployment: Outdoor, IP65 enclosure
- Quantity: 100 sensors
- Total: $28,000

**Agricultural Soil Monitoring:**
- Sensor: METER TEROS 12 ($250)
- Connectivity: LoRaWAN
- Power: Battery (2-year life)
- Measurements: Moisture, temperature, EC
- Deployment: Buried 15cm depth
- Quantity: 200 sensors, 4 gateways
- Total: $52,000

### Deployment Best Practices

**Sensor Placement:**
- Air quality: 2-3m height, away from direct emissions
- Water quality: Representative locations, avoid stagnant areas
- Soil: Multiple depths, representative field locations
- Weather: Open area, 2m height for temperature

**Physical Installation:**
- Secure mounting (prevent theft/vandalism)
- Weatherproof enclosures
- Proper grounding (lightning protection)
- Cable management and strain relief
- Clear labeling with device ID

**Power Considerations:**
- Grid power: GFCI protection, weatherproof outlets
- Solar: Size for worst-case month, 5-10W panels
- Battery: 2× capacity for safety margin
- Backup power for critical sensors

---

## 8.3 Sensor Calibration and Validation

### Initial Calibration

**Air Quality Sensors:**
```python
def calibrate_pm_sensor(sensor_id, reference_pm25, measured_pm25):
    """
    Co-location calibration with reference monitor

    Args:
        sensor_id: Device identifier
        reference_pm25: Array of reference measurements
        measured_pm25: Array of sensor measurements

    Returns:
        Calibration parameters (slope, intercept)
    """
    from scipy.stats import linregress

    slope, intercept, r_value, p_value, std_err = linregress(measured_pm25, reference_pm25)

    calibration = {
        "deviceId": sensor_id,
        "lastCalibration": datetime.utcnow().isoformat() + "Z",
        "nextCalibration": (datetime.utcnow() + timedelta(days=180)).isoformat() + "Z",
        "method": "reference_colocated",
        "parameters": {
            "slope": round(slope, 4),
            "intercept": round(intercept, 4),
            "r_squared": round(r_value**2, 4)
        },
        "certificateId": f"CAL-{datetime.utcnow().strftime('%Y%m%d')}-{sensor_id}"
    }

    print(f"Calibration equation: PM2.5_corrected = {slope:.4f} * PM2.5_raw + {intercept:.4f}")
    print(f"R² = {r_value**2:.4f}")

    return calibration

# Example usage
reference = [10.2, 15.3, 22.1, 30.5, 45.2]  # Reference monitor readings
measured = [8.5, 13.2, 20.3, 28.1, 42.3]    # Low-cost sensor readings

cal = calibrate_pm_sensor("ENV-AIR-001", reference, measured)
```

**Water Quality Sensors:**
- pH: 2-point calibration (pH 4.0 and 7.0 buffers)
- Conductivity: 1-point calibration (1413 μS/cm standard)
- Turbidity: 3-point calibration (0, 100, 1000 NTU standards)
- DO: Zero and saturation calibration

**Soil Sensors:**
- Moisture: Gravimetric comparison in known soil
- EC: Standard solution calibration
- Nutrients: Laboratory analysis comparison

### Calibration Schedule

| Sensor Type | Frequency | Method |
|-------------|-----------|--------|
| PM sensors | 6 months | Reference co-location |
| Gas sensors | 3-6 months | Zero/span calibration |
| pH | 1 month | Buffer calibration |
| DO | 3 months | Zero/saturation |
| Soil moisture | 12 months | Gravimetric check |

---

## 8.4 Network Configuration

### MQTT Broker Setup

**Mosquitto Configuration:**
```conf
# /etc/mosquitto/mosquitto.conf

listener 1883
listener 8883

# TLS configuration
cafile /etc/mosquitto/ca_certificates/ca.crt
certfile /etc/mosquitto/certs/server.crt
keyfile /etc/mosquitto/certs/server.key
require_certificate false
tls_version tlsv1.3

# Authentication
allow_anonymous false
password_file /etc/mosquitto/passwd

# Logging
log_dest file /var/log/mosquitto/mosquitto.log
log_type all
```

**Create User:**
```bash
mosquitto_passwd -c /etc/mosquitto/passwd sensor_user
sudo systemctl restart mosquitto
```

### LoRaWAN Network Server

**ChirpStack Configuration:**
```yaml
# chirpstack.toml
[network_server]
net_id = "000000"
band = "US915"

[network_server.gateway]
stats_interval = "30s"

[application_server]
enabled = true

[application_server.integration.mqtt]
server = "tcp://localhost:1883"
username = "chirpstack"
password = "password"

# Uplink topic
uplink_topic_template = "application/{{ .ApplicationID }}/device/{{ .DevEUI }}/rx"
```

**Device Provisioning:**
```python
import grpc
from chirpstack_api.as_pb import device_pb2
from chirpstack_api.as_pb import device_pb2_grpc

# Configure ChirpStack API
channel = grpc.insecure_channel('localhost:8080')
client = device_pb2_grpc.DeviceServiceStub(channel)

# Create device
device = device_pb2.Device(
    dev_eui="0000000000000001",
    name="ENV-SOIL-001",
    application_id=1,
    description="Soil moisture sensor - Field A",
    device_profile_id="abc123"
)

request = device_pb2.CreateDeviceRequest(device=device)
response = client.Create(request)
```

---

## 8.5 Data Pipeline Setup

### Complete Pipeline Architecture

```
Sensors → MQTT Broker → Python Processor → Database → API → Dashboard
```

**Python Data Processor:**
```python
import paho.mqtt.client as mqtt
import json
import psycopg2
from datetime import datetime

# Database connection
conn = psycopg2.connect("dbname=sensors user=postgres password=password")

def on_message(client, userdata, msg):
    try:
        # Parse WIA message
        data = json.loads(msg.payload)

        # Validate
        if not validate_wia_message(data):
            print(f"Invalid message from {data.get('deviceId')}")
            return

        # Store in database
        store_in_database(data)

        # Check thresholds
        check_thresholds(data)

    except Exception as e:
        print(f"Error processing message: {e}")

def store_in_database(data):
    cur = conn.cursor()

    for param, reading in data['readings'].items():
        cur.execute("""
            INSERT INTO sensor_data (
                time, device_id, sensor_type, parameter, value, unit
            ) VALUES (%s, %s, %s, %s, %s, %s)
        """, (
            data['timestamp'],
            data['deviceId'],
            data['sensorType'],
            param,
            reading['value'],
            reading['unit']
        ))

    conn.commit()
    cur.close()

def check_thresholds(data):
    """Alert if PM2.5 > 55 (unhealthy)"""
    if 'pm2_5' in data.get('readings', {}):
        pm25 = data['readings']['pm2_5']['value']
        if pm25 > 55:
            send_alert(data['deviceId'], 'PM2.5', pm25, 55)

def send_alert(device_id, parameter, value, threshold):
    # Send email/SMS alert
    print(f"ALERT: {device_id} {parameter}={value} exceeds threshold {threshold}")

# MQTT client
client = mqtt.Client()
client.username_pw_set("sensor_user", "password")
client.tls_set(ca_certs="/path/to/ca.crt")
client.on_message = on_message
client.connect("broker.example.com", 8883, 60)
client.subscribe("wia/env027/#")
client.loop_forever()
```

---

## 8.6 Dashboard Development

### Grafana Setup

**Install Grafana:**
```bash
sudo apt-get install -y software-properties-common
sudo add-apt-repository "deb https://packages.grafana.com/oss/deb stable main"
sudo apt-get update
sudo apt-get install grafana
sudo systemctl start grafana-server
sudo systemctl enable grafana-server
```

**Configure Data Source (InfluxDB):**
1. Navigate to Configuration → Data Sources
2. Add InfluxDB data source
3. Configure:
   - URL: http://localhost:8086
   - Database: sensors
   - User: admin
   - Password: password

**Create Dashboard:**
- Add panel: Time series graph
- Query: `SELECT mean("value") FROM "pm2_5" WHERE $timeFilter GROUP BY time(1h)`
- Visualization: Line graph
- Thresholds: Green (0-12), Yellow (12-35), Orange (35-55), Red (55+)

---

## 8.7 Testing and Quality Assurance

### Test Plan

**Unit Tests:**
```python
import pytest
from sensor_processor import validate_wia_message, process_air_quality

def test_valid_message():
    message = {
        "version": "1.0.0",
        "standard": "WIA-ENE-027",
        "deviceId": "TEST-001",
        "timestamp": "2025-01-09T10:30:00Z",
        "sensorType": "air_quality",
        "readings": {"pm2_5": {"value": 15.3, "unit": "μg/m³"}}
    }
    assert validate_wia_message(message) == True

def test_missing_fields():
    message = {"deviceId": "TEST-001"}
    assert validate_wia_message(message) == False

def test_out_of_range():
    message = {
        "version": "1.0.0",
        "standard": "WIA-ENE-027",
        "deviceId": "TEST-001",
        "timestamp": "2025-01-09T10:30:00Z",
        "sensorType": "air_quality",
        "readings": {"pm2_5": {"value": 1500, "unit": "μg/m³"}}
    }
    result = process_air_quality(message)
    assert "out_of_range" in result['quality']['flags']
```

**Load Testing:**
```python
import concurrent.futures
import requests
import time

def submit_sensor_data(sensor_id, count):
    url = f"https://api.example.com/api/v1/sensors/{sensor_id}/data"
    headers = {"Authorization": "Bearer API_KEY"}

    for i in range(count):
        data = {
            "version": "1.0.0",
            "standard": "WIA-ENE-027",
            "deviceId": sensor_id,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "sensorType": "air_quality",
            "readings": {"pm2_5": {"value": 15.0 + i*0.1, "unit": "μg/m³"}}
        }
        response = requests.post(url, json=data, headers=headers)
        assert response.status_code == 201

# Simulate 100 sensors sending data simultaneously
with concurrent.futures.ThreadPoolExecutor(max_workers=100) as executor:
    futures = [executor.submit(submit_sensor_data, f"TEST-{i:03d}", 10) for i in range(100)]
    concurrent.futures.wait(futures)
```

---

## 8.8 Operations and Maintenance

### Monitoring Checklist

**Daily:**
- Check sensor data freshness (all sensors reporting?)
- Review alert logs
- Check API uptime and response times
- Monitor database size and performance

**Weekly:**
- Review data quality flags
- Check calibration status
- Update firmware if needed
- Review system logs for errors

**Monthly:**
- Generate compliance reports
- Review and optimize database queries
- Backup and archival
- Calibrate sensors on schedule

**Quarterly:**
- System performance review
- Security audit
- User feedback review
- Plan capacity upgrades

### Maintenance Procedures

**Battery Replacement:**
1. Check battery level via API
2. Schedule field visit when < 20%
3. Replace with fresh batteries
4. Update metadata in system
5. Verify sensor resumes operation

**Sensor Calibration:**
1. Co-locate with reference monitor (7-14 days)
2. Collect paired measurements
3. Calculate calibration parameters
4. Update sensor firmware/configuration
5. Record calibration certificate
6. Schedule next calibration

---

## 8.9 Review Questions and Key Takeaways

### Review Questions

1. Create a 6-month implementation plan for 50-sensor air quality network including milestones and deliverables.

2. Calculate total cost of ownership for 100 sensors over 3 years including hardware, cloud, maintenance, and calibration.

3. Design calibration schedule for mixed deployment: 20 air quality, 30 water quality, 50 soil sensors.

4. Write unit tests for WIA message validation covering required fields, data types, and range checks.

### Key Takeaways

1. **Phased Approach**: 24-week implementation roadmap from planning through deployment ensures systematic execution.

2. **Hardware Selection**: Balance accuracy, cost, power, and connectivity based on deployment requirements.

3. **Calibration Critical**: Regular calibration (3-12 months depending on sensor type) maintains data quality.

4. **Network Configuration**: MQTT broker and LoRaWAN server setup enables reliable sensor connectivity.

5. **Data Pipeline**: Python processor validates, stores, and monitors sensor data in real-time.

6. **Testing**: Unit, integration, and load testing ensures system reliability before production.

7. **Operations**: Daily/weekly/monthly/quarterly checklists maintain system health.

8. **Documentation**: Comprehensive documentation accelerates troubleshooting and knowledge transfer.

---

## Chapter Summary

This chapter provided practical guidance for deploying WIA-ENE-027 compliant environmental sensor systems. The implementation roadmap spans 24 weeks from planning through operations, with clear phases and deliverables.

Hardware selection balances accuracy, cost, and operational requirements. Proper sensor placement, secure mounting, and appropriate power solutions ensure reliable long-term operation. Initial calibration and ongoing calibration schedules maintain measurement accuracy.

Network configuration establishes MQTT brokers or LoRaWAN servers for sensor connectivity. The data pipeline validates incoming data, stores it in time-series databases, and checks thresholds for alerting. Dashboard development provides visualization through Grafana or custom applications.

Comprehensive testing including unit tests, integration tests, and load tests ensures system reliability. Operations and maintenance procedures with daily/weekly/monthly/quarterly checklists keep the system running smoothly.

Following this implementation guide, organizations can successfully deploy scalable, reliable environmental sensor networks that provide actionable insights for air quality, water quality, soil monitoring, and meteorological applications.

---

© 2025 WIA Standards Committee. 弘益인간 (홍익인간) - Benefit All Humanity

**Congratulations! You have completed the WIA-ENE-027 Environmental Sensor Standard ebook.**

**For continued learning:**
- Review phase specifications in detail
- Explore code examples on GitHub
- Join the WIA community forum
- Consider WIA certification for your deployment
