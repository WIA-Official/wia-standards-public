# WIA-AGRI-007: Smart Irrigation Standard
## Phase 4: System Integration Specification

### 4.1 Overview

Phase 4 defines the integration points with external systems including weather services, soil sensors, water utilities, farm management software, and certification platforms.

**Duration**: 6-12 months
**Key Outcome**: Fully integrated smart irrigation ecosystem

### 4.2 Weather API Integration

#### 4.2.1 Supported Weather Services

- **NOAA/NWS** (USA): National Weather Service data
- **OpenWeatherMap**: Global weather API
- **Weather Underground**: Hyperlocal weather data
- **CIMIS** (California): California Irrigation Management Information System
- **Meteostat**: Historical weather data
- **Custom weather stations**: On-farm weather monitoring

#### 4.2.2 Weather Data Endpoints

**NOAA/NWS Integration**:
```json
{
  "provider": "NOAA",
  "endpoint": "https://api.weather.gov/points/{lat},{lon}",
  "update_frequency_minutes": 60,
  "data_fields": [
    "temperature",
    "humidity",
    "wind_speed",
    "precipitation",
    "solar_radiation"
  ],
  "api_key_required": false
}
```

**OpenWeatherMap Integration**:
```json
{
  "provider": "OpenWeatherMap",
  "endpoint": "https://api.openweathermap.org/data/3.0/onecall",
  "params": {
    "lat": 38.5,
    "lon": -121.5,
    "appid": "{API_KEY}",
    "units": "metric",
    "exclude": "minutely,alerts"
  },
  "update_frequency_minutes": 15,
  "rate_limit": "1000/day"
}
```

#### 4.2.3 Weather Data Mapping

```python
def map_weather_data(provider, raw_data):
    """Map provider-specific data to WIA standard format"""

    if provider == "OpenWeatherMap":
        return {
            "timestamp": datetime.fromtimestamp(raw_data['current']['dt']),
            "temperature_c": raw_data['current']['temp'],
            "humidity_percent": raw_data['current']['humidity'],
            "wind_speed_ms": raw_data['current']['wind_speed'],
            "precipitation_mm": raw_data['current'].get('rain', {}).get('1h', 0),
            "solar_radiation_mj_m2": raw_data['current'].get('uvi', 0) * 0.36,  # Approximation
            "conditions": raw_data['current']['weather'][0]['description']
        }

    elif provider == "NOAA":
        return {
            "timestamp": raw_data['properties']['timestamp'],
            "temperature_c": fahrenheit_to_celsius(raw_data['properties']['temperature']['value']),
            "humidity_percent": raw_data['properties']['relativeHumidity']['value'],
            "wind_speed_ms": raw_data['properties']['windSpeed']['value'] * 0.44704,  # mph to m/s
            # ... additional mappings
        }
```

### 4.3 Soil Sensor Integration

#### 4.3.1 Supported Sensor Protocols

- **Modbus RTU**: Industrial sensor networks
- **SDI-12**: Standard for environmental sensors
- **LoRaWAN**: Long-range wireless sensors
- **Zigbee**: Low-power mesh networks
- **Proprietary**: Vendor-specific protocols (with adapters)

#### 4.3.2 SDI-12 Protocol Implementation

```python
# SDI-12 sensor reading
class SDI12Sensor:
    def __init__(self, port, address):
        self.port = port
        self.address = address

    def read_measurement(self):
        # Send measurement command
        command = f"{self.address}M!"
        self.port.write(command.encode())

        # Wait for sensor response
        response = self.port.readline()
        # Parse: '0013' means 0 seconds wait, 1 measurement, 3 values

        # Request data
        command = f"{self.address}D0!"
        self.port.write(command.encode())

        data = self.port.readline().decode()
        # Parse: '0+25.4+0.35+22.1' (address, moisture%, VWC, temp)

        values = data.split('+')[1:]  # Skip address
        return {
            "soil_moisture_percent": float(values[0]),
            "volumetric_water_content": float(values[1]),
            "soil_temperature_c": float(values[2])
        }
```

#### 4.3.3 LoRaWAN Sensor Integration

```json
{
  "sensor_network": {
    "protocol": "LoRaWAN",
    "frequency": "915MHz (US) / 868MHz (EU)",
    "gateway_id": "gw-farm-001",
    "sensors": [
      {
        "device_eui": "70B3D5499123ABCD",
        "sensor_type": "soil_moisture",
        "zone_id": "zone-1",
        "depth_cm": 30,
        "transmit_interval_minutes": 60,
        "battery_type": "2xAA Lithium",
        "expected_battery_life_years": 5
      }
    ]
  }
}
```

### 4.4 Water Utility Integration

#### 4.4.1 Water Meter Reading

**Pulse Counter Integration**:
```json
{
  "meter_id": "wm-main-001",
  "meter_type": "pulse_counter",
  "pulse_per_liter": 1,
  "gpio_pin": 23,
  "debounce_ms": 50,
  "cumulative_volume_m3": 125634.5,
  "last_reset": "2025-01-01T00:00:00Z"
}
```

**Modbus Water Meter**:
```python
from pymodbus.client import ModbusSerialClient

def read_water_meter(meter_address):
    client = ModbusSerialClient(
        port='/dev/ttyUSB0',
        baudrate=9600,
        parity='N',
        stopbits=1,
        bytesize=8
    )

    # Read cumulative volume (registers 0-1, 32-bit)
    result = client.read_holding_registers(
        address=0,
        count=2,
        slave=meter_address
    )

    cumulative_liters = (result.registers[0] << 16) | result.registers[1]
    cumulative_m3 = cumulative_liters / 1000

    # Read instantaneous flow (register 2)
    result = client.read_holding_registers(address=2, count=1, slave=meter_address)
    flow_rate_lpm = result.registers[0] / 10  # Scaled by 10

    return {
        "cumulative_volume_m3": cumulative_m3,
        "flow_rate_lpm": flow_rate_lpm,
        "timestamp": datetime.now().isoformat()
    }
```

#### 4.4.2 Water Billing Integration

```json
{
  "utility_provider": "Aqua District 123",
  "account_number": "1234567890",
  "billing_cycle": "monthly",
  "rate_structure": {
    "tier_1": {
      "max_volume_m3": 500,
      "rate_per_m3": 0.50
    },
    "tier_2": {
      "max_volume_m3": 1500,
      "rate_per_m3": 0.75
    },
    "tier_3": {
      "max_volume_m3": null,
      "rate_per_m3": 1.00
    }
  },
  "api_endpoint": "https://api.aquadistrict.com/v1/usage",
  "auto_report_usage": true,
  "report_frequency": "daily"
}
```

### 4.5 Farm Management Software Integration

#### 4.5.1 Supported Platforms

- **John Deere Operations Center**
- **Climate FieldView**
- **Trimble Ag Software**
- **FarmLogs**
- **AgWorld**
- **Generic REST API**

#### 4.5.2 Data Exchange Format

```json
{
  "integration": {
    "platform": "Generic FMS",
    "sync_direction": "bidirectional",
    "sync_frequency_hours": 24,
    "data_types": [
      "field_boundaries",
      "crop_types",
      "planting_dates",
      "harvest_dates",
      "irrigation_events",
      "water_usage"
    ],
    "api_config": {
      "endpoint": "https://fms.example.com/api/v2",
      "auth_type": "oauth2",
      "client_id": "{CLIENT_ID}",
      "client_secret": "{CLIENT_SECRET}"
    }
  }
}
```

#### 4.5.3 Field Boundary Import

```python
def import_field_boundaries(fms_api_client):
    """Import field boundaries from FMS and create irrigation zones"""

    # Fetch fields from FMS
    fields = fms_api_client.get_fields()

    for field in fields:
        # Create zone if doesn't exist
        zone = {
            "zone_id": f"zone-{field['id']}",
            "zone_name": field['name'],
            "area_ha": field['area_hectares'],
            "crop_type": field['current_crop'],
            "boundary": field['geometry'],  # GeoJSON polygon
            "external_id": field['id'],
            "external_system": "FMS"
        }

        create_or_update_zone(zone)

        # Import planting date to adjust crop coefficient
        if field.get('planting_date'):
            update_crop_stage(zone['zone_id'], field['planting_date'])
```

### 4.6 Satellite & Remote Sensing Integration

#### 4.6.1 NDVI & Crop Health Monitoring

```json
{
  "satellite_provider": "Sentinel-2",
  "resolution_m": 10,
  "revisit_days": 5,
  "data_products": ["NDVI", "NDMI", "RGB"],
  "api_endpoint": "https://scihub.copernicus.eu/dhus",
  "zone_monitoring": [
    {
      "zone_id": "zone-1",
      "ndvi_threshold_low": 0.4,
      "ndvi_threshold_high": 0.8,
      "action_on_low_ndvi": "increase_irrigation",
      "action_on_high_ndvi": "maintain_current"
    }
  ]
}
```

#### 4.6.2 NDVI-Based Irrigation Adjustment

```python
def adjust_irrigation_by_ndvi(zone_id, ndvi_value):
    """Adjust irrigation based on vegetation index"""

    zone = get_zone(zone_id)

    if ndvi_value < zone.ndvi_threshold_low:
        # Low vegetation health - possible water stress
        adjustment_factor = 1.2  # Increase irrigation by 20%
        priority = "high"
    elif ndvi_value > zone.ndvi_threshold_high:
        # Healthy vegetation
        adjustment_factor = 0.9  # Reduce irrigation by 10%
        priority = "normal"
    else:
        # Normal range
        adjustment_factor = 1.0
        priority = "normal"

    # Update zone irrigation schedule
    update_zone_irrigation_factor(zone_id, adjustment_factor, priority)

    return {
        "zone_id": zone_id,
        "ndvi": ndvi_value,
        "adjustment_factor": adjustment_factor,
        "new_priority": priority
    }
```

### 4.7 Energy Management Integration

#### 4.7.1 Solar Power Integration

```json
{
  "power_source": "solar_with_battery",
  "solar_panel_kw": 10,
  "battery_capacity_kwh": 20,
  "grid_connection": true,
  "irrigation_scheduling": {
    "prefer_solar_hours": true,
    "solar_hours_start": "08:00",
    "solar_hours_end": "16:00",
    "battery_reserve_percent": 30,
    "grid_power_allowed": true,
    "max_grid_power_kw": 5
  }
}
```

#### 4.7.2 Time-of-Use Rate Optimization

```python
def optimize_irrigation_for_tou_rates(zones, electricity_rates):
    """Schedule irrigation during low-rate periods"""

    # Get TOU rate schedule
    off_peak_hours = [hour for hour, rate in electricity_rates.items()
                     if rate == min(electricity_rates.values())]

    # Reschedule zones to off-peak hours
    for zone in zones:
        current_schedule = get_zone_schedule(zone.zone_id)

        # Find next available off-peak slot
        next_off_peak = find_next_hour_in_list(off_peak_hours)

        if next_off_peak != current_schedule.start_hour:
            # Reschedule
            update_zone_schedule(
                zone.zone_id,
                start_time=f"{next_off_peak:02d}:00:00",
                reason="TOU_optimization"
            )
```

### 4.8 Water Efficiency Certification

#### 4.8.1 WIA Certification Platform

```json
{
  "certification_api": "https://certification.wia.org/api/v1",
  "standard": "WIA-AGRI-007",
  "farm_id": "farm-12345",
  "certification_level": "silver",
  "auto_reporting": true,
  "report_frequency": "monthly",
  "metrics_tracked": [
    "total_water_usage_m3",
    "water_savings_vs_baseline_percent",
    "irrigation_efficiency_percent",
    "sensor_coverage_percent",
    "weather_integration_active"
  ]
}
```

#### 4.8.2 Certification Data Submission

```python
def submit_certification_report(system_id, period_start, period_end):
    """Generate and submit water efficiency report for certification"""

    # Calculate metrics
    water_usage = get_water_usage(system_id, period_start, period_end)
    baseline_usage = get_baseline_usage(system_id)
    savings_percent = ((baseline_usage - water_usage) / baseline_usage) * 100

    efficiency = calculate_irrigation_efficiency(system_id, period_start, period_end)
    sensor_coverage = calculate_sensor_coverage(system_id)

    # Prepare certification report
    report = {
        "farm_id": get_farm_id(system_id),
        "report_period": {
            "start": period_start.isoformat(),
            "end": period_end.isoformat()
        },
        "metrics": {
            "total_water_usage_m3": water_usage,
            "baseline_usage_m3": baseline_usage,
            "water_savings_percent": savings_percent,
            "irrigation_efficiency_percent": efficiency,
            "sensor_coverage_percent": sensor_coverage,
            "weather_integration_active": is_weather_integration_active(system_id)
        },
        "compliance_status": "compliant" if savings_percent >= 35 else "non_compliant"
    }

    # Submit to WIA certification platform
    response = requests.post(
        "https://certification.wia.org/api/v1/reports",
        json=report,
        headers={"Authorization": f"Bearer {CERT_API_TOKEN}"}
    )

    return response.json()
```

#### 4.8.3 Verifiable Credentials (VC) Generation

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wia.org/credentials/irrigation/v1"
  ],
  "type": ["VerifiableCredential", "WaterEfficiencyCredential"],
  "issuer": "did:wia:certification-authority",
  "issuanceDate": "2025-01-15T00:00:00Z",
  "expirationDate": "2026-01-15T00:00:00Z",
  "credentialSubject": {
    "id": "did:wia:farm:green-valley-farms",
    "farmName": "Green Valley Farms",
    "certificationLevel": "silver",
    "waterSavingsPercent": 42,
    "annualWaterUsage_m3": 150000,
    "irrigationEfficiency": 87.5,
    "standard": "WIA-AGRI-007",
    "auditDate": "2025-01-10"
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-01-15T00:00:00Z",
    "proofPurpose": "assertionMethod",
    "verificationMethod": "did:wia:certification-authority#key-1",
    "proofValue": "z5e4Qw7xK...signature...Zy8P3mN"
  }
}
```

### 4.9 Mobile App Integration

#### 4.9.1 Mobile API Endpoints

```
GET  /mobile/v1/dashboard - Overview of all systems
GET  /mobile/v1/systems/{id}/status - Real-time status
POST /mobile/v1/systems/{id}/zones/{zone_id}/irrigate - Start irrigation
POST /mobile/v1/systems/{id}/zones/{zone_id}/stop - Stop irrigation
GET  /mobile/v1/notifications - Get alerts and notifications
POST /mobile/v1/notifications/{id}/acknowledge - Acknowledge alert
```

#### 4.9.2 Push Notifications

```json
{
  "notification_service": "Firebase Cloud Messaging",
  "notification_types": [
    {
      "type": "irrigation_started",
      "priority": "normal",
      "message_template": "Irrigation started in {zone_name} for {duration} minutes"
    },
    {
      "type": "alert_critical",
      "priority": "high",
      "message_template": "ALERT: {alert_message} in {zone_name}",
      "sound": "critical_alert.wav"
    },
    {
      "type": "daily_summary",
      "priority": "low",
      "message_template": "Today: {total_water_m3} m³ used across {active_zones} zones",
      "scheduled_time": "20:00"
    }
  ]
}
```

### 4.10 Cloud Platform Integration

#### 4.10.1 AWS IoT Core Integration

```python
import boto3
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

def setup_aws_iot_connection(thing_name, cert_path, key_path):
    """Connect irrigation system to AWS IoT Core"""

    mqtt_client = AWSIoTMQTTClient(thing_name)
    mqtt_client.configureEndpoint("xxxxxx.iot.us-west-2.amazonaws.com", 8883)
    mqtt_client.configureCredentials(
        "root-ca.pem",
        key_path,
        cert_path
    )

    mqtt_client.connect()

    # Subscribe to command topic
    mqtt_client.subscribe(f"irrigation/{thing_name}/commands", 1, command_callback)

    # Publish telemetry
    def publish_telemetry(sensor_data):
        mqtt_client.publish(
            f"irrigation/{thing_name}/telemetry",
            json.dumps(sensor_data),
            1
        )

    return mqtt_client, publish_telemetry
```

### 4.11 Data Analytics & Reporting

#### 4.11.1 Business Intelligence Integration

```json
{
  "bi_platform": "Tableau / Power BI / Custom",
  "data_warehouse": {
    "type": "PostgreSQL / TimescaleDB",
    "connection_string": "postgresql://user:pass@host:5432/irrigation_db",
    "tables": [
      "irrigation_events",
      "sensor_readings",
      "water_usage_daily",
      "weather_history",
      "zone_performance"
    ]
  },
  "update_frequency_minutes": 15,
  "retention_policy": {
    "raw_data_days": 90,
    "aggregated_data_years": 5
  }
}
```

### 4.12 Integration Testing

#### 4.12.1 Integration Test Scenarios

1. **Weather API Failover**: Primary weather source fails, system switches to backup
2. **Sensor Communication Loss**: Handle gracefully, use last known values or ET-based scheduling
3. **Utility Meter Mismatch**: Detect and alert on discrepancies between system flow totals and utility meter
4. **FMS Sync Conflict**: Handle conflicting crop data from multiple sources
5. **Certification Upload Failure**: Queue reports for retry when connection restored

### 4.13 Integration Checklist

- [ ] Configure weather API integration
- [ ] Connect all soil sensors and verify readings
- [ ] Integrate water utility meter
- [ ] Set up FMS data exchange (if applicable)
- [ ] Configure satellite data feeds (optional)
- [ ] Integrate energy management system (if applicable)
- [ ] Register for WIA certification platform
- [ ] Set up mobile app connectivity
- [ ] Configure cloud platform connection
- [ ] Set up data warehouse for analytics
- [ ] Test all integration points
- [ ] Document API keys and credentials securely
- [ ] Set up monitoring and alerting for integration failures

---

**Previous Phase**: [Phase 3: Protocol](PHASE-3-PROTOCOL.md)

© 2025 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity
