# 제7장: 시스템 통합

## Phase 4: 클라우드 플랫폼, 분석 및 생태계 통합

---

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

1. AWS, Azure, Google Cloud를 사용한 클라우드 플랫폼 통합 구현하기
2. InfluxDB와 TimescaleDB를 위한 시계열 데이터베이스 구성하기
3. Grafana 대시보드 및 맞춤형 시각화 개발하기
4. EPA AQS 및 기타 형식으로 규제 보고 자동화하기
5. WIA-OMNI-API 및 WIA-INTENT와 통합하기
6. 조직 간 연합 모니터링 네트워크 설계하기
7. 스마트시티 및 농업을 위한 참조 아키텍처 적용하기

---

## 7.1 클라우드 플랫폼 통합 (AWS, Azure, Google Cloud)

### AWS IoT Core 통합

```
WIA 센서 → MQTT → AWS IoT Core → 규칙 엔진 →
  ├→ S3 (장기 저장)
  ├→ Timestream (시계열 데이터베이스)
  ├→ Lambda (처리)
  └→ SNS (경보)
```

**AWS IoT Core 규칙:**
```sql
SELECT * FROM 'wia/env027/+/+/+/data'
```

**Lambda 함수 (데이터 처리):**
```python
import json
import boto3
from datetime import datetime

timestream = boto3.client('timestream-write')

def lambda_handler(event, context):
    # WIA 센서 데이터 파싱
    sensor_data = json.loads(event['payload'])

    # Timestream에 쓰기
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

### Azure IoT Hub 통합

```
WIA 센서 → MQTT → IoT Hub → 메시지 라우팅 →
  ├→ Azure Data Lake (저장)
  ├→ Time Series Insights (분석)
  └→ Stream Analytics (실시간)
```

**Azure 함수 (C#):**
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

### Google Cloud IoT 통합

```
WIA 센서 → MQTT → Cloud IoT Core → Pub/Sub →
  ├→ BigQuery (데이터 웨어하우스)
  ├→ Dataflow (스트림 처리)
  └→ Cloud Functions (이벤트 처리)
```

**Cloud 함수 (Node.js):**
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

## 7.2 시계열 데이터베이스 통합

### InfluxDB

**Python 통합:**
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

# 예제 사용
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

# 쿼리 예제
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

**SQL 스키마:**
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

**Python 통합:**
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

# 시간당 평균을 위한 연속 집계
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

## 7.3 분석 및 시각화 플랫폼

### Grafana 대시보드

**데이터 소스 구성:**
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

**대시보드 패널 (JSON):**
```json
{
  "title": "PM2.5 수준 - 지난 24시간",
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

### 맞춤형 React 대시보드

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
    const interval = setInterval(fetchData, 300000); // 5분마다
    return () => clearInterval(interval);
  }, [deviceId]);

  return (
    <div>
      <h2>PM2.5 수준 - {deviceId}</h2>
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

## 7.4 규제 보고 시스템

### EPA 대기질 시스템 (AQS) 보고

```python
class WIAtoAQSReporter:
    def __init__(self, api_client):
        self.api = api_client

    def generate_aqs_report(self, device_id, start_date, end_date):
        # WIA 데이터 가져오기
        wia_data = self.api.get_sensor_data(
            device_id=device_id,
            start=start_date,
            end=end_date,
            aggregation='daily'
        )

        # AQS 형식으로 변환
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
                'Method_Code': 'SENSOR'  # WIA 센서용 맞춤형
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

## 7.5 WIA 표준과의 통합

### WIA-OMNI-API 탐색

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

### WIA-INTENT 통합

```javascript
// 의도: "서울에서 건강에 해로운 대기질 지역 표시"
{
  "intent": "query_environmental_data",
  "parameters": {
    "parameter": "air_quality",
    "condition": "unhealthy",
    "location": "Seoul",
    "timeframe": "current"
  }
}

// 시스템이 WIA API로 변환:
// GET /api/v1/sensors?type=air_quality&location=seoul
// aqi.category in ["unhealthy", "very_unhealthy", "hazardous"]로 필터링
```

---

## 7.6 연합 모니터링 네트워크

### 조직 간 데이터 공유

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
        # 모든 조직의 PM2.5 집계
        total_pm25 = sum(r['readings']['pm2_5']['value'] for r in results)
        avg_pm25 = total_pm25 / len(results)
        return avg_pm25
```

---

## 7.7 참조 아키텍처

### 스마트시티 대기질 네트워크

```
아키텍처 구성요소:
├─ 센서 계층: 200개 대기질 센서
├─ 엣지 계층: 20개 게이트웨이 (로컬 처리)
├─ 클라우드 계층: AWS IoT Core + Timestream
├─ API 계층: WIA Phase 2 준수 REST API
├─ 저장: S3 (원시), Timestream (처리됨)
├─ 분석: Lambda (이상 감지), SageMaker (예측)
├─ 애플리케이션:
│  ├─ 공개 대시보드 (React + Grafana)
│  ├─ 모바일 앱 (React Native)
│  ├─ 경보 시스템 (SNS → SMS/이메일)
│  └─ 규제 보고 (자동화된 AQS)
└─ 통합: 공공 참여를 위한 WIA-SOCIAL

배포:
- 200개 센서 @ $300 = $60,000
- 20개 게이트웨이 @ $500 = $10,000
- 클라우드: 월 ~$2,000
- 개발: $150,000
- 첫 해 총계: $244,000
```

### 농업 토양 모니터링

```
아키텍처:
├─ 센서: 500개 LoRaWAN 토양 프로브 (5개 농장)
├─ 게이트웨이: 10개 LoRaWAN 게이트웨이
├─ 플랫폼: Azure IoT Hub + Time Series Insights
├─ API: WIA 준수 REST API
├─ 애플리케이션:
│  ├─ 농장 대시보드 (웹)
│  ├─ 농부용 모바일 앱
│  ├─ 관개 자동화
│  └─ 작물 모델링 통합
└─ 분석: 수확량 예측 모델

혜택:
- 30% 물 절약
- 15% 수확량 개선
- 에이커당 $500 추가 수익
- ROI: 18개월
```

---

## 7.8 복습 문제 및 핵심 요점

### 복습 문제

1. 1,000개 센서를 위한 AWS IoT Core 통합을 설계하세요. 어떤 서비스를 사용하고 왜 사용하시겠습니까?

2. 위치별 PM2.5의 일일 최소/최대/평균을 위한 TimescaleDB 연속 집계를 생성하세요.

3. 1년 데이터 저장 비용을 비교하세요 (1000개 센서, 5분 간격): (a) InfluxDB Cloud, (b) AWS Timestream, (c) 자체 호스팅 TimescaleDB.

4. 지난 24시간 동안 3개 조직에서 가장 높은 PM2.5 읽기를 찾기 위한 연합 쿼리를 설계하세요.

### 핵심 요점

1. **클라우드 통합**: AWS, Azure 및 Google Cloud는 원활한 WIA 통합을 제공하는 관리형 IoT 플랫폼을 제공합니다.

2. **시계열 데이터베이스**: InfluxDB 및 TimescaleDB는 효율적인 쿼리 및 집계로 센서 데이터에 최적화되어 있습니다.

3. **시각화**: Grafana는 강력한 대시보드를 제공하고 맞춤형 React 앱은 맞춤형 사용자 경험을 가능하게 합니다.

4. **규제 준수**: 자동화된 보고는 WIA 데이터를 규제 형식(AQS 등)으로 변환합니다.

5. **WIA 생태계**: OMNI-API, INTENT 및 SOCIAL 표준과의 통합으로 포괄적인 솔루션을 만듭니다.

6. **연합 네트워크**: 조직 간 데이터 공유로 지역/국가 환경 모니터링이 가능합니다.

7. **참조 아키텍처**: 스마트시티 및 농업을 위한 검증된 패턴이 배포를 가속화합니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity

**다음 장: [제8장: 구현 가이드](08-implementation.md)**
