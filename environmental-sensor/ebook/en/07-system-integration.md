# Chapter 7: System Integration

## Phase 4: Cloud Platforms, Analytics, and Ecosystem Integration

---

## 7.1 Cloud Platform Integration (AWS, Azure, Google Cloud)

### AWS IoT Core Integration

```
WIA Sensors → MQTT → AWS IoT Core → Rules Engine →
  ├→ S3 (long-term storage)
  ├→ Timestream (time-series database)
  ├→ Lambda (processing)
  └→ SNS (alerting)
```

**AWS IoT Core Rule:**
```sql
SELECT * FROM 'wia/env027/+/+/+/data'
```

**Lambda Function (Data Processing):**
```python
import json
import boto3
from datetime import datetime

timestream = boto3.client('timestream-write')

def lambda_handler(event, context):
    # Parse WIA sensor data
    sensor_data = json.loads(event['payload'])

    # Write to Timestream
    records = []
    for param, reading in sensor_data['readings'].items():
        records.append({
            'Time': sensor_data['timestamp'],
            'Dimensions': [
                {'Name': 'deviceId', 'Value': sensor_data['deviceId']},
                {'Name': 'parameter', 'Value': param}
            ],
            'MeasureName': 'value',
            'MeasureValue': str(reading['value']),
            'MeasureValueType': 'DOUBLE'
        })

    timestream.write_records(
        DatabaseName='environmental-sensors',
        TableName='sensor-data',
        Records=records
    )

    return {'statusCode': 200}
```

### Azure IoT Hub Integration

```
WIA Sensors → MQTT → IoT Hub → Message Routing →
  ├→ Azure Data Lake (storage)
  ├→ Time Series Insights (analytics)
  └→ Stream Analytics (real-time)
```

**Azure Function (C#):**
```csharp
using Microsoft.Azure.WebJobs;
using Microsoft.Extensions.Logging;
using Newtonsoft.Json;

public static class ProcessSensorData
{
    [FunctionName("ProcessSensorData")]
    public static void Run(
        [IoTHubTrigger("messages/events", Connection = "IoTHubConnection")] string message,
        [CosmosDB(
            databaseName: "SensorDB",
            collectionName: "SensorData",
            ConnectionStringSetting = "CosmosDBConnection")] out dynamic document,
        ILogger log)
    {
        var sensorData = JsonConvert.DeserializeObject<dynamic>(message);

        document = new
        {
            id = Guid.NewGuid().ToString(),
            deviceId = sensorData.deviceId,
            timestamp = sensorData.timestamp,
            sensorType = sensorData.sensorType,
            readings = sensorData.readings
        };

        log.LogInformation($"Processed data from {sensorData.deviceId}");
    }
}
```

### Google Cloud IoT Integration

```
WIA Sensors → MQTT → Cloud IoT Core → Pub/Sub →
  ├→ BigQuery (data warehouse)
  ├→ Dataflow (stream processing)
  └→ Cloud Functions (event processing)
```

**Cloud Function (Node.js):**
```javascript
const {BigQuery} = require('@google-cloud/bigquery');
const bigquery = new BigQuery();

exports.processSensorData = async (message, context) => {
  const data = JSON.parse(Buffer.from(message.data, 'base64').toString());

  const rows = Object.entries(data.readings).map(([param, reading]) => ({
    deviceId: data.deviceId,
    timestamp: data.timestamp,
    parameter: param,
    value: reading.value,
    unit: reading.unit
  }));

  await bigquery
    .dataset('environmental_sensors')
    .table('sensor_data')
    .insert(rows);
};
```

---

## 7.2 Time-Series Database Integration

### InfluxDB

**Python Integration:**
```python
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

client = InfluxDBClient(url="http://localhost:8086", token="my-token", org="my-org")
write_api = client.write_api(write_options=SYNCHRONOUS)

def wia_to_influx(wia_data):
    points = []

    for param, reading in wia_data['readings'].items():
        point = Point("environmental_sensor") \
            .tag("deviceId", wia_data["deviceId"]) \
            .tag("sensorType", wia_data["sensorType"]) \
            .tag("parameter", param) \
            .field("value", reading["value"]) \
            .field("unit", reading["unit"]) \
            .time(wia_data["timestamp"])

        if "uncertainty" in reading:
            point.field("uncertainty", reading["uncertainty"])

        points.append(point)

    write_api.write(bucket="sensors", record=points)

# Example usage
sensor_data = {
    "deviceId": "ENV-AIR-001",
    "timestamp": "2025-01-09T10:30:00Z",
    "sensorType": "air_quality",
    "readings": {
        "pm2_5": {"value": 15.3, "unit": "μg/m³", "uncertainty": 3.0},
        "pm10": {"value": 22.8, "unit": "μg/m³", "uncertainty": 4.5}
    }
}

wia_to_influx(sensor_data)

# Query example
query_api = client.query_api()
query = '''
from(bucket: "sensors")
  |> range(start: -24h)
  |> filter(fn: (r) => r["deviceId"] == "ENV-AIR-001")
  |> filter(fn: (r) => r["parameter"] == "pm2_5")
'''
result = query_api.query(query=query)
```

### TimescaleDB

**SQL Schema:**
```sql
CREATE TABLE sensor_data (
    time TIMESTAMPTZ NOT NULL,
    device_id TEXT NOT NULL,
    sensor_type TEXT,
    parameter TEXT,
    value DOUBLE PRECISION,
    unit TEXT,
    uncertainty DOUBLE PRECISION,
    quality_overall TEXT,
    quality_flags JSONB
);

SELECT create_hypertable('sensor_data', 'time');

CREATE INDEX ON sensor_data (device_id, time DESC);
CREATE INDEX ON sensor_data (parameter, time DESC);
```

**Python Integration:**
```python
import psycopg2
import json

conn = psycopg2.connect("dbname=sensors user=postgres")
cur = conn.cursor()

def wia_to_timescale(wia_data):
    for param, reading in wia_data['readings'].items():
        cur.execute("""
            INSERT INTO sensor_data (
                time, device_id, sensor_type, parameter,
                value, unit, uncertainty, quality_overall, quality_flags
            ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)
        """, (
            wia_data['timestamp'],
            wia_data['deviceId'],
            wia_data['sensorType'],
            param,
            reading['value'],
            reading['unit'],
            reading.get('uncertainty'),
            wia_data.get('quality', {}).get('overall'),
            json.dumps(wia_data.get('quality', {}).get('flags', []))
        ))

    conn.commit()

# Continuous aggregate for hourly averages
cur.execute("""
    CREATE MATERIALIZED VIEW hourly_avg
    WITH (timescaledb.continuous) AS
    SELECT
        device_id,
        parameter,
        time_bucket('1 hour', time) AS hour,
        avg(value) AS avg_value,
        min(value) AS min_value,
        max(value) AS max_value,
        count(*) AS sample_count
    FROM sensor_data
    GROUP BY device_id, parameter, hour
""")
```

---

## 7.3 Analytics and Visualization Platforms

### Grafana Dashboard

**Data Source Configuration:**
```yaml
apiVersion: 1
datasources:
  - name: WIA-InfluxDB
    type: influxdb
    url: http://influxdb:8086
    jsonData:
      version: Flux
      organization: my-org
      defaultBucket: sensors
    secureJsonData:
      token: my-influx-token
```

**Dashboard Panel (JSON):**
```json
{
  "title": "PM2.5 Levels - Last 24 Hours",
  "type": "graph",
  "datasource": "WIA-InfluxDB",
  "targets": [
    {
      "query": "from(bucket: \"sensors\")\n  |> range(start: -24h)\n  |> filter(fn: (r) => r[\"parameter\"] == \"pm2_5\")\n  |> aggregateWindow(every: 1h, fn: mean)"
    }
  ],
  "fieldConfig": {
    "defaults": {
      "unit": "conμgm3",
      "thresholds": {
        "steps": [
          {"value": 0, "color": "green"},
          {"value": 12, "color": "yellow"},
          {"value": 35, "color": "orange"},
          {"value": 55, "color": "red"}
        ]
      }
    }
  }
}
```

### Custom React Dashboard

```javascript
import React, { useEffect, useState } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend } from 'recharts';

function SensorDashboard({ deviceId }) {
  const [data, setData] = useState([]);

  useEffect(() => {
    const fetchData = async () => {
      const response = await fetch(
        `https://api.example.com/api/v1/sensors/${deviceId}/data?start=-24h&aggregation=hourly`,
        { headers: { 'Authorization': `Bearer ${API_KEY}` } }
      );
      const result = await response.json();
      setData(result.data);
    };

    fetchData();
    const interval = setInterval(fetchData, 300000); // Every 5 minutes
    return () => clearInterval(interval);
  }, [deviceId]);

  return (
    <div>
      <h2>PM2.5 Levels - {deviceId}</h2>
      <LineChart width={800} height={400} data={data}>
        <CartesianGrid strokeDasharray="3 3" />
        <XAxis dataKey="timestamp" />
        <YAxis label={{ value: 'PM2.5 (μg/m³)', angle: -90 }} />
        <Tooltip />
        <Legend />
        <Line type="monotone" dataKey="pm2_5.mean" stroke="#8884d8" />
      </LineChart>
    </div>
  );
}
```

---

## 7.4 Regulatory Reporting Systems

### EPA Air Quality System (AQS) Reporting

```python
class WIAtoAQSReporter:
    def __init__(self, api_client):
        self.api = api_client

    def generate_aqs_report(self, device_id, start_date, end_date):
        # Fetch WIA data
        wia_data = self.api.get_sensor_data(
            device_id=device_id,
            start=start_date,
            end=end_date,
            aggregation='daily'
        )

        # Transform to AQS format
        aqs_records = []
        for record in wia_data:
            aqs_records.append({
                'State_Code': self.extract_state_code(device_id),
                'County_Code': self.extract_county_code(device_id),
                'Site_ID': device_id,
                'Parameter_Code': '88101',  # PM2.5 LC
                'POC': '1',  # Parameter Occurrence Code
                'Date': record['timestamp'][:10],
                'Sample_Duration': '24',  # 24-hour
                'Sample_Value': record['pm2_5']['mean'],
                'Units': 'Micrograms/cubic meter',
                'Method_Code': 'SENSOR'  # Custom for WIA sensors
            })

        return aqs_records

    def export_aqs_csv(self, records, filename):
        import csv
        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=records[0].keys())
            writer.writeheader()
            writer.writerows(records)
```

---

## 7.5 Integration with WIA Standards

### WIA-OMNI-API Discovery

```json
{
  "standardId": "WIA-ENE-027",
  "version": "1.0.0",
  "endpoints": {
    "discovery": "/api/v1/sensors",
    "data": "/api/v1/sensors/{id}/data",
    "realtime": "wss://api.example.com/v1/stream"
  },
  "authentication": ["OAuth2", "APIKey", "JWT"],
  "capabilities": {
    "sensorTypes": ["air_quality", "water_quality", "soil", "meteorological"],
    "protocols": ["MQTT", "CoAP", "LoRaWAN", "HTTP"],
    "features": ["streaming", "aggregation", "alerts", "calibration"]
  }
}
```

### WIA-INTENT Integration

```javascript
// Intent: "Show me areas with unhealthy air quality in Seoul"
{
  "intent": "query_environmental_data",
  "parameters": {
    "parameter": "air_quality",
    "condition": "unhealthy",
    "location": "Seoul",
    "timeframe": "current"
  }
}

// System translates to WIA API:
// GET /api/v1/sensors?type=air_quality&location=seoul
// Filter where aqi.category in ["unhealthy", "very_unhealthy", "hazardous"]
```

---

## 7.6 Federated Monitoring Networks

### Cross-Organization Data Sharing

```python
class FederatedSensorNetwork:
    def __init__(self):
        self.participants = {
            'org-a': 'https://api.org-a.com',
            'org-b': 'https://api.org-b.com',
            'org-c': 'https://api.org-c.com'
        }

    def federated_query(self, sensor_type, location_bbox, start, end):
        results = []

        for org_id, api_url in self.participants.items():
            try:
                response = requests.get(
                    f"{api_url}/api/v1/sensors",
                    params={
                        'type': sensor_type,
                        'location': location_bbox,
                        'start': start,
                        'end': end
                    }
                )
                results.extend(response.json()['sensors'])
            except Exception as e:
                print(f"Failed to query {org_id}: {e}")

        return results

    def aggregate_air_quality(self, results):
        # Aggregate PM2.5 across all organizations
        total_pm25 = sum(r['readings']['pm2_5']['value'] for r in results)
        avg_pm25 = total_pm25 / len(results)
        return avg_pm25
```

---

## 7.7 Reference Architectures

### Smart City Air Quality Network

```
Architecture Components:
├─ Sensor Layer: 200 air quality sensors
├─ Edge Layer: 20 gateways with local processing
├─ Cloud Layer: AWS IoT Core + Timestream
├─ API Layer: WIA Phase 2 compliant REST API
├─ Storage: S3 (raw), Timestream (processed)
├─ Analytics: Lambda (anomaly detection), SageMaker (forecasting)
├─ Applications:
│  ├─ Public dashboard (React + Grafana)
│  ├─ Mobile app (React Native)
│  ├─ Alert system (SNS → SMS/Email)
│  └─ Regulatory reporting (automated AQS)
└─ Integration: WIA-SOCIAL for public engagement

Deployment:
- 200 sensors @ $300 = $60,000
- 20 gateways @ $500 = $10,000
- Cloud: ~$2,000/month
- Development: $150,000
- Total first year: $244,000
```

### Agricultural Soil Monitoring

```
Architecture:
├─ Sensors: 500 LoRaWAN soil probes (5 farms)
├─ Gateways: 10 LoRaWAN gateways
├─ Platform: Azure IoT Hub + Time Series Insights
├─ API: WIA-compliant REST API
├─ Applications:
│  ├─ Farm dashboard (web)
│  ├─ Mobile app for farmers
│  ├─ Irrigation automation
│  └─ Crop modeling integration
└─ Analytics: Yield prediction models

Benefits:
- 30% water savings
- 15% yield improvement
- $500/acre additional revenue
- ROI: 18 months
```

---

## 7.8 Review Questions and Key Takeaways

### Review Questions

1. Design AWS IoT Core integration for 1,000 sensors. What services would you use and why?

2. Create TimescaleDB continuous aggregate for daily min/max/avg of PM2.5 by location.

3. Compare costs for storing 1 year of data (1000 sensors, 5-minute intervals) in: (a) InfluxDB Cloud, (b) AWS Timestream, (c) self-hosted TimescaleDB.

4. Design federated query to find highest PM2.5 readings across 3 organizations in last 24 hours.

### Key Takeaways

1. **Cloud Integration**: AWS, Azure, and Google Cloud provide managed IoT platforms with seamless WIA integration.

2. **Time-Series Databases**: InfluxDB and TimescaleDB optimized for sensor data with efficient querying and aggregation.

3. **Visualization**: Grafana provides powerful dashboards; custom React apps enable tailored user experiences.

4. **Regulatory Compliance**: Automated reporting transforms WIA data to regulatory formats (AQS, etc.).

5. **WIA Ecosystem**: Integration with OMNI-API, INTENT, and SOCIAL standards creates comprehensive solutions.

6. **Federated Networks**: Cross-organization data sharing enables regional/national environmental monitoring.

7. **Reference Architectures**: Proven patterns for smart cities and agriculture accelerate deployment.

---

© 2025 WIA Standards Committee. 弘益인간 (홍익인간) - Benefit All Humanity

**Next Chapter: [Chapter 8: Implementation Guide](08-implementation.md)**
