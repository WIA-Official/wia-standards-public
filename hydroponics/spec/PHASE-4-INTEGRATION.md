# WIA-AGRI-027: Hydroponics Standard
## Phase 4: System Integration Guidelines

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26
**Emoji:** 💧

---

## 1. Overview

This document provides comprehensive guidelines for integrating WIA-AGRI-027 Hydroponics Standard with existing agricultural systems, enterprise software, IoT platforms, and third-party applications.

### 1.1 Philosophy

弘益人間 (홍익인간) - Benefit All Humanity

Our integration framework is designed to be flexible, extensible, and compatible with diverse ecosystems while maintaining data integrity and security.

### 1.2 Integration Scope

- ERP (Enterprise Resource Planning) systems
- Farm Management Software (FMS)
- IoT platforms and device management
- Cloud storage and analytics platforms
- Business intelligence and reporting tools
- Mobile and web applications

---

## 2. Integration Architecture

### 2.1 Reference Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Application Layer                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │ ERP/FMS  │  │Dashboard │  │Mobile App│  │Analytics │        │
│  └─────┬────┘  └─────┬────┘  └─────┬────┘  └─────┬────┘        │
└────────┼─────────────┼─────────────┼─────────────┼──────────────┘
         │             │             │             │
         └─────────────┼─────────────┼─────────────┘
                       │             │
┌──────────────────────┼─────────────┼──────────────────────────────┐
│                  API Gateway Layer                                │
│      ┌─────────────────────────────────────────────┐             │
│      │  WIA-AGRI-027 API Gateway                   │             │
│      │  - Authentication & Authorization            │             │
│      │  - Rate Limiting & Throttling               │             │
│      │  - Request Routing & Load Balancing         │             │
│      └──────────────────┬──────────────────────────┘             │
└─────────────────────────┼───────────────────────────────────────┘
                          │
┌─────────────────────────┼───────────────────────────────────────┐
│                  Service Layer                                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │ System   │  │ Sensor   │  │ Nutrient │  │ Alert    │        │
│  │ Service  │  │ Service  │  │ Service  │  │ Service  │        │
│  └─────┬────┘  └─────┬────┘  └─────┬────┘  └─────┬────┘        │
└────────┼─────────────┼─────────────┼─────────────┼──────────────┘
         │             │             │             │
┌────────┼─────────────┼─────────────┼─────────────┼──────────────┐
│        │         Data Layer         │             │              │
│  ┌─────┴────┐  ┌─────┴────┐  ┌─────┴────┐  ┌─────┴────┐        │
│  │PostgreSQL│  │TimescaleDB│ │  Redis   │  │  S3      │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
└───────────────────────────────────────────────────────────────────┘
         │             │             │             │
┌────────┼─────────────┼─────────────┼─────────────┼──────────────┐
│        │      Device Layer          │             │              │
│  ┌─────┴────┐  ┌─────┴────┐  ┌─────┴────┐  ┌─────┴────┐        │
│  │ Gateway  │  │ Sensors  │  │Actuators │  │Controllers│       │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
└───────────────────────────────────────────────────────────────────┘
```

---

## 3. ERP Integration

### 3.1 Supported ERP Systems

- SAP Business One
- Microsoft Dynamics 365
- Oracle NetSuite
- Odoo
- Custom ERP systems

### 3.2 Integration Patterns

**Pattern 1: API-Based Integration**

```javascript
// Sync harvest data to ERP
async function syncHarvestToERP(harvestData) {
  // Get harvest log from WIA-AGRI-027
  const harvest = await wiaClient.getHarvestLog(harvestData.harvestId);

  // Transform to ERP format
  const erpInventoryItem = {
    productCode: harvest.cropType,
    quantity: harvest.quantity,
    unit: harvest.unit,
    quality: harvest.quality,
    harvestDate: harvest.harvestDate,
    batchNumber: harvest.batchNumber,
    location: harvest.location,
    cost: calculateProductionCost(harvest),
    metadata: {
      systemId: harvest.systemId,
      nutrientFormula: harvest.nutrientHistory.formulaId
    }
  };

  // Create inventory entry in ERP
  await erpClient.createInventoryItem(erpInventoryItem);
}
```

**Pattern 2: Webhook-Based Integration**

```javascript
// ERP webhook endpoint
app.post('/webhooks/wia-harvest', async (req, res) => {
  const event = req.body;

  if (event.event === 'harvest.logged') {
    const harvestData = event.data;

    // Process harvest data
    await processHarvestForERP(harvestData);

    // Update production tracking
    await updateProductionMetrics(harvestData);
  }

  res.status(200).json({ received: true });
});
```

### 3.3 Data Mapping

| WIA-AGRI-027 Field | ERP Field | Transformation |
|-------------------|-----------|----------------|
| harvestId | productionOrderId | Direct mapping |
| cropType | productCode | Lookup from master data |
| quantity | inventoryQuantity | Direct mapping |
| weight | netWeight | Direct mapping |
| quality | qualityGrade | Map premium→A, grade_a→B |
| harvestDate | completionDate | ISO 8601 to ERP date format |
| operator | operator | Direct mapping |
| systemId | workCenter | Lookup from system registry |

---

## 4. Farm Management Software Integration

### 4.1 Integration Workflow

```
┌─────────────┐      ┌─────────────┐      ┌─────────────┐
│   Sensors   │─────▶│ WIA-AGRI-027│─────▶│     FMS     │
│             │      │   Gateway   │      │             │
└─────────────┘      └─────────────┘      └─────────────┘
       │                    │                     │
       │                    ▼                     │
       │             ┌─────────────┐              │
       │             │  Analytics  │              │
       │             │   Engine    │              │
       │             └─────────────┘              │
       │                    │                     │
       │                    ▼                     │
       │             ┌─────────────┐              │
       └────────────▶│  Dashboard  │◀─────────────┘
                     │   & Alerts  │
                     └─────────────┘
```

### 4.2 FMS Integration SDK

**Installation:**

```bash
npm install @wia/agri027-fms-adapter
```

**Usage Example:**

```javascript
const { FMSAdapter } = require('@wia/agri027-fms-adapter');

const adapter = new FMSAdapter({
  wiaApiKey: 'YOUR_WIA_API_KEY',
  fmsProvider: 'farmOS',
  fmsConfig: {
    baseUrl: 'https://farm.example.com',
    username: 'admin',
    password: 'secret'
  }
});

// Sync system status to FMS
await adapter.syncSystemStatus('HYDRO-SYS-001');

// Sync sensor data to FMS
await adapter.syncSensorData({
  systemId: 'HYDRO-SYS-001',
  startTime: '2025-12-25T00:00:00Z',
  endTime: '2025-12-26T00:00:00Z'
});

// Create FMS activity log from harvest
await adapter.createActivityLog({
  type: 'harvest',
  harvestId: 'HARVEST-20251226-001'
});
```

---

## 5. IoT Platform Integration

### 5.1 AWS IoT Core

**Architecture:**

```
Sensors → MQTT → AWS IoT Core → AWS IoT Rules → Lambda
                                                   ├─→ DynamoDB
                                                   ├─→ S3
                                                   └─→ CloudWatch
```

**IoT Core Rule:**

```json
{
  "sql": "SELECT * FROM 'wia/agri027/+/+/sensors/#'",
  "ruleDisabled": false,
  "awsIotSqlVersion": "2016-03-23",
  "actions": [
    {
      "lambda": {
        "functionArn": "arn:aws:lambda:us-east-1:123456789:function:ProcessSensorData"
      }
    },
    {
      "dynamoDBv2": {
        "roleArn": "arn:aws:iam::123456789:role/IoTDynamoDBRole",
        "putItem": {
          "tableName": "SensorReadings"
        }
      }
    }
  ]
}
```

**Lambda Function:**

```javascript
exports.handler = async (event) => {
  const sensorData = JSON.parse(event.payload);

  // Process sensor data
  if (sensorData.parameter === 'pH' && sensorData.value > 7.0) {
    // Trigger alert
    await sns.publish({
      TopicArn: 'arn:aws:sns:us-east-1:123456789:HydroponicsAlerts',
      Message: `High pH detected: ${sensorData.value}`,
      Subject: 'WIA-AGRI-027 Alert'
    }).promise();
  }

  return { statusCode: 200, body: 'Processed' };
};
```

### 5.2 Azure IoT Hub

**Connection String:**

```
HostName=wia-agri027.azure-devices.net;DeviceId=HYDRO-SYS-001;SharedAccessKey=...
```

**Device Twin:**

```json
{
  "deviceId": "HYDRO-SYS-001",
  "properties": {
    "desired": {
      "targetPH": 6.0,
      "targetEC": 2.0,
      "alertThresholds": {
        "ph": {"min": 5.5, "max": 6.5},
        "ec": {"min": 1.8, "max": 2.2}
      }
    },
    "reported": {
      "currentPH": 6.2,
      "currentEC": 2.1,
      "status": "operational",
      "lastUpdate": "2025-12-26T10:30:00Z"
    }
  }
}
```

### 5.3 Google Cloud IoT Core

**Device Configuration:**

```yaml
deviceId: HYDRO-SYS-001
cloudRegion: us-central1
registryId: hydroponics-registry
projectId: wia-agri027

deviceConfig:
  targetPH: 6.0
  targetEC: 2.0
  samplingInterval: 60
```

**Pub/Sub Integration:**

```javascript
const {PubSub} = require('@google-cloud/pubsub');
const pubsub = new PubSub();

// Subscribe to sensor data
const subscription = pubsub.subscription('sensor-data-sub');

subscription.on('message', message => {
  const data = JSON.parse(message.data.toString());
  console.log('Received sensor data:', data);

  // Process data
  processSensorData(data);

  message.ack();
});
```

---

## 6. Cloud Analytics Integration

### 6.1 Time-Series Databases

**InfluxDB Integration:**

```javascript
const Influx = require('influx');

const influx = new Influx.InfluxDB({
  host: 'localhost',
  database: 'wia_agri027',
  schema: [
    {
      measurement: 'sensor_readings',
      fields: {
        value: Influx.FieldType.FLOAT,
        quality: Influx.FieldType.STRING
      },
      tags: [
        'systemId',
        'deviceId',
        'deviceType',
        'parameter'
      ]
    }
  ]
});

// Write sensor data
await influx.writePoints([
  {
    measurement: 'sensor_readings',
    tags: {
      systemId: 'HYDRO-SYS-001',
      deviceId: 'SENSOR-PH-001',
      deviceType: 'ph_sensor',
      parameter: 'pH'
    },
    fields: {
      value: 6.2,
      quality: 'excellent'
    },
    timestamp: new Date()
  }
]);

// Query historical data
const results = await influx.query(`
  SELECT mean("value") AS "avg_ph"
  FROM "sensor_readings"
  WHERE "systemId" = 'HYDRO-SYS-001'
    AND "parameter" = 'pH'
    AND time > now() - 24h
  GROUP BY time(1h)
`);
```

**TimescaleDB Integration:**

```sql
-- Create hypertable
CREATE TABLE sensor_readings (
  time TIMESTAMPTZ NOT NULL,
  system_id VARCHAR(50),
  device_id VARCHAR(50),
  parameter VARCHAR(50),
  value DECIMAL(10,2),
  quality VARCHAR(20)
);

SELECT create_hypertable('sensor_readings', 'time');

-- Create continuous aggregate for hourly averages
CREATE MATERIALIZED VIEW sensor_hourly_avg
WITH (timescaledb.continuous) AS
SELECT
  time_bucket('1 hour', time) AS hour,
  system_id,
  parameter,
  AVG(value) AS avg_value,
  MIN(value) AS min_value,
  MAX(value) AS max_value
FROM sensor_readings
GROUP BY hour, system_id, parameter;
```

### 6.2 Data Warehousing

**BigQuery Integration:**

```javascript
const {BigQuery} = require('@google-cloud/bigquery');
const bigquery = new BigQuery();

async function loadSensorData(sensorReadings) {
  const datasetId = 'wia_agri027';
  const tableId = 'sensor_readings';

  const rows = sensorReadings.map(reading => ({
    timestamp: reading.timestamp,
    system_id: reading.systemId,
    device_id: reading.deviceId,
    parameter: reading.parameter,
    value: reading.value,
    quality: reading.quality
  }));

  await bigquery
    .dataset(datasetId)
    .table(tableId)
    .insert(rows);

  console.log(`Inserted ${rows.length} rows`);
}

// Analyze data
async function analyzeSystemPerformance(systemId, startDate, endDate) {
  const query = `
    SELECT
      parameter,
      AVG(value) as avg_value,
      MIN(value) as min_value,
      MAX(value) as max_value,
      STDDEV(value) as std_dev
    FROM \`wia_agri027.sensor_readings\`
    WHERE system_id = @systemId
      AND timestamp BETWEEN @startDate AND @endDate
    GROUP BY parameter
  `;

  const options = {
    query: query,
    params: {systemId, startDate, endDate}
  };

  const [rows] = await bigquery.query(options);
  return rows;
}
```

---

## 7. Business Intelligence Integration

### 7.1 Power BI

**Data Connector:**

```m
// Power Query M script
let
  Source = Web.Contents("https://api.wia.org/agri027/v1/system/HYDRO-SYS-001/sensors/history", [
    Headers=[
      #"Authorization"="Bearer YOUR_API_KEY",
      #"Content-Type"="application/json"
    ],
    Query=[
      startTime="2025-12-01T00:00:00Z",
      endTime="2025-12-26T23:59:59Z",
      aggregation="hourly"
    ]
  ]),
  JsonData = Json.Document(Source),
  DataPoints = JsonData[dataPoints],
  ToTable = Table.FromList(DataPoints, Splitter.SplitByNothing(), null, null, ExtraValues.Error),
  ExpandedColumns = Table.ExpandRecordColumn(ToTable, "Column1", {"timestamp", "value", "quality"})
in
  ExpandedColumns
```

### 7.2 Tableau

**Web Data Connector:**

```javascript
(function() {
  var myConnector = tableau.makeConnector();

  myConnector.getSchema = function(schemaCallback) {
    var cols = [
      {id: "timestamp", dataType: tableau.dataTypeEnum.datetime},
      {id: "systemId", dataType: tableau.dataTypeEnum.string},
      {id: "parameter", dataType: tableau.dataTypeEnum.string},
      {id: "value", dataType: tableau.dataTypeEnum.float},
      {id: "quality", dataType: tableau.dataTypeEnum.string}
    ];

    var tableSchema = {
      id: "wiaAgri027SensorData",
      alias: "WIA-AGRI-027 Sensor Data",
      columns: cols
    };

    schemaCallback([tableSchema]);
  };

  myConnector.getData = function(table, doneCallback) {
    fetch('https://api.wia.org/agri027/v1/system/HYDRO-SYS-001/sensors/history', {
      headers: {
        'Authorization': 'Bearer YOUR_API_KEY'
      }
    })
    .then(res => res.json())
    .then(data => {
      var tableData = data.dataPoints.map(point => ({
        timestamp: point.timestamp,
        systemId: data.systemId,
        parameter: data.parameter,
        value: point.value,
        quality: point.quality
      }));

      table.appendRows(tableData);
      doneCallback();
    });
  };

  tableau.registerConnector(myConnector);
})();
```

---

## 8. Mobile Application Integration

### 8.1 React Native Integration

**Installation:**

```bash
npm install @wia/agri027-react-native
```

**Usage:**

```javascript
import React, { useEffect, useState } from 'react';
import { View, Text } from 'react-native';
import { WIAClient } from '@wia/agri027-react-native';

const HydroponicsMonitor = () => {
  const [sensorData, setSensorData] = useState(null);
  const client = new WIAClient({ apiKey: 'YOUR_API_KEY' });

  useEffect(() => {
    // Get real-time sensor data
    const subscription = client.subscribeToSensors('HYDRO-SYS-001', (data) => {
      setSensorData(data);
    });

    return () => subscription.unsubscribe();
  }, []);

  return (
    <View>
      <Text>pH: {sensorData?.ph || 'Loading...'}</Text>
      <Text>EC: {sensorData?.ec || 'Loading...'} mS/cm</Text>
      <Text>Temp: {sensorData?.temperature || 'Loading...'} °C</Text>
    </View>
  );
};
```

### 8.2 Flutter Integration

```dart
import 'package:wia_agri027/wia_agri027.dart';

class HydroponicsMonitor extends StatefulWidget {
  @override
  _HydroponicsMonitorState createState() => _HydroponicsMonitorState();
}

class _HydroponicsMonitorState extends State<HydroponicsMonitor> {
  final WIAClient client = WIAClient(apiKey: 'YOUR_API_KEY');
  SensorData? sensorData;

  @override
  void initState() {
    super.initState();
    _subscribeToSensors();
  }

  void _subscribeToSensors() {
    client.subscribeToSensors('HYDRO-SYS-001', (data) {
      setState(() {
        sensorData = data;
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Text('pH: ${sensorData?.ph ?? "Loading..."}'),
        Text('EC: ${sensorData?.ec ?? "Loading..."} mS/cm'),
        Text('Temp: ${sensorData?.temperature ?? "Loading..."} °C'),
      ],
    );
  }
}
```

---

## 9. Microservices Integration

### 9.1 Docker Deployment

**docker-compose.yml:**

```yaml
version: '3.8'

services:
  wia-agri027-gateway:
    image: wia/agri027-gateway:latest
    ports:
      - "8080:8080"
    environment:
      - DATABASE_URL=postgresql://user:pass@db:5432/wia_agri027
      - REDIS_URL=redis://redis:6379
      - MQTT_BROKER=mqtt://mosquitto:1883
    depends_on:
      - db
      - redis
      - mosquitto

  db:
    image: timescale/timescaledb:latest-pg14
    environment:
      - POSTGRES_DB=wia_agri027
      - POSTGRES_USER=user
      - POSTGRES_PASSWORD=pass
    volumes:
      - pgdata:/var/lib/postgresql/data

  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"

  mosquitto:
    image: eclipse-mosquitto:latest
    ports:
      - "1883:1883"
      - "8883:8883"
    volumes:
      - ./mosquitto.conf:/mosquitto/config/mosquitto.conf

  grafana:
    image: grafana/grafana:latest
    ports:
      - "3000:3000"
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=admin
    depends_on:
      - db

volumes:
  pgdata:
```

### 9.2 Kubernetes Deployment

**deployment.yaml:**

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-agri027-gateway
spec:
  replicas: 3
  selector:
    matchLabels:
      app: wia-agri027-gateway
  template:
    metadata:
      labels:
        app: wia-agri027-gateway
    spec:
      containers:
      - name: gateway
        image: wia/agri027-gateway:latest
        ports:
        - containerPort: 8080
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: db-secrets
              key: url
        - name: API_KEY
          valueFrom:
            secretKeyRef:
              name: api-secrets
              key: key
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
        livenessProbe:
          httpGet:
            path: /health
            port: 8080
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 8080
          initialDelaySeconds: 5
          periodSeconds: 5
---
apiVersion: v1
kind: Service
metadata:
  name: wia-agri027-gateway
spec:
  selector:
    app: wia-agri027-gateway
  ports:
  - protocol: TCP
    port: 80
    targetPort: 8080
  type: LoadBalancer
```

---

## 10. Testing & Validation

### 10.1 Integration Testing

```javascript
const { WIAClient } = require('@wia/agri027-sdk');
const assert = require('assert');

describe('WIA-AGRI-027 Integration Tests', () => {
  let client;

  before(() => {
    client = new WIAClient({ apiKey: process.env.WIA_API_KEY });
  });

  it('should retrieve system status', async () => {
    const status = await client.getSystemStatus('HYDRO-SYS-001');
    assert(status.systemId === 'HYDRO-SYS-001');
    assert(status.status === 'operational');
  });

  it('should get current sensor data', async () => {
    const sensors = await client.getSensorData('HYDRO-SYS-001');
    assert(Array.isArray(sensors));
    assert(sensors.length > 0);
  });

  it('should adjust nutrients', async () => {
    const adjustment = await client.adjustNutrients('HYDRO-SYS-001', {
      targetPH: 6.0,
      targetEC: 2.0
    });
    assert(adjustment.status === 'initiated');
  });
});
```

### 10.2 Load Testing

```javascript
const autocannon = require('autocannon');

autocannon({
  url: 'https://api.wia.org/agri027/v1/system/HYDRO-SYS-001/status',
  connections: 100,
  duration: 60,
  headers: {
    'Authorization': 'Bearer YOUR_API_KEY'
  }
}, (err, result) => {
  console.log('Requests/sec:', result.requests.average);
  console.log('Latency (ms):', result.latency.mean);
});
```

---

## 11. Best Practices

### 11.1 Error Handling

```javascript
async function robustIntegration() {
  try {
    const data = await wiaClient.getSensorData('HYDRO-SYS-001');
    await processData(data);
  } catch (error) {
    if (error.code === 'RATE_LIMIT_EXCEEDED') {
      // Wait and retry
      await sleep(60000);
      return robustIntegration();
    } else if (error.code === 'NETWORK_ERROR') {
      // Log and alert
      logger.error('Network error:', error);
      await alertOps(error);
    } else {
      // Unexpected error
      throw error;
    }
  }
}
```

### 11.2 Caching Strategy

```javascript
const NodeCache = require('node-cache');
const cache = new NodeCache({ stdTTL: 300 }); // 5 minutes

async function getCachedSystemStatus(systemId) {
  const cacheKey = `system:${systemId}:status`;
  let status = cache.get(cacheKey);

  if (!status) {
    status = await wiaClient.getSystemStatus(systemId);
    cache.set(cacheKey, status);
  }

  return status;
}
```

---

## 12. Conclusion

The WIA-AGRI-027 Phase 4 integration guidelines provide a comprehensive framework for connecting hydroponic systems with enterprise software, IoT platforms, and analytics tools. By following these best practices, organizations can build robust, scalable, and maintainable integrations.

---

**Document Information:**

- **Standard ID:** WIA-AGRI-027
- **Phase:** 4 - System Integration Guidelines
- **Status:** Active
- **Maintained by:** WIA (World Certification Industry Association)
- **Contact:** integration@wia.org

弘益人間 (홍익인간) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
