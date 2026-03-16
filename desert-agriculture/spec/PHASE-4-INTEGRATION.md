# WIA-AGRI-021: Desert Agriculture
## Phase 4 - Integration Specification

**Version:** 1.0
**Status:** Official Standard
**Last Updated:** 2025-12-26
**Standard ID:** WIA-AGRI-021

---

## 1. Overview

This specification defines integration patterns, best practices, and implementation guidelines for connecting WIA-AGRI-021 Desert Agriculture systems with existing agricultural infrastructure, IoT platforms, enterprise systems, and third-party services.

### 1.1 Integration Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    External Systems                         │
├─────────────────────────────────────────────────────────────┤
│  ERP  │  Weather  │  Market  │  Satellite  │  Supply Chain │
└────┬───────┬────────┬──────────┬────────────┬───────────────┘
     │       │        │          │            │
     └───────┴────────┴──────────┴────────────┘
                      │
         ┌────────────▼────────────┐
         │   Integration Layer     │
         │  - API Gateway          │
         │  - Message Queue        │
         │  - ETL Pipeline         │
         └────────────┬────────────┘
                      │
         ┌────────────▼────────────┐
         │  WIA-AGRI-021 Platform  │
         │  - Core Services        │
         │  - Data Storage         │
         │  - Analytics Engine     │
         └────────────┬────────────┘
                      │
     ┌────────────────┼────────────────┐
     │                │                │
┌────▼────┐    ┌─────▼──────┐   ┌────▼────┐
│ Sensors │    │ Controllers │   │ Devices │
└─────────┘    └────────────┘   └─────────┘
```

### 1.2 Integration Patterns

- **API Integration**: RESTful APIs for synchronous communication
- **Event-Driven**: Webhooks and message queues for asynchronous events
- **Batch Processing**: Scheduled data synchronization
- **Real-time Streaming**: WebSocket and MQTT for live data
- **File-Based**: CSV/JSON exports for legacy systems

---

## 2. IoT Platform Integration

### 2.1 AWS IoT Core

#### 2.1.1 Device Configuration

```javascript
const AWS = require('aws-sdk');
const iot = new AWS.Iot({region: 'us-east-1'});

// Create thing for each sensor
const createSensor = async (farmId, sensorId) => {
  // Create IoT thing
  const thing = await iot.createThing({
    thingName: `${farmId}_${sensorId}`,
    attributePayload: {
      attributes: {
        farmId: farmId,
        sensorType: 'soil_moisture',
        standard: 'WIA-AGRI-021'
      }
    }
  }).promise();

  // Attach policy
  await iot.attachThingPrincipal({
    thingName: thing.thingName,
    principal: certificateArn
  }).promise();

  return thing;
};
```

#### 2.1.2 Data Ingestion

```javascript
// Publish sensor data to AWS IoT
const AWS_IOT = require('aws-iot-device-sdk');

const device = AWS_IOT.device({
  keyPath: './certs/private.pem.key',
  certPath: './certs/certificate.pem.crt',
  caPath: './certs/root-CA.crt',
  clientId: 'farm_sahara_001_sensor_001',
  host: 'xxxxx.iot.us-east-1.amazonaws.com'
});

device.on('connect', () => {
  console.log('Connected to AWS IoT');

  // Publish sensor data
  device.publish(
    'wia-agri-021/farm_sahara_001/sensors/data',
    JSON.stringify({
      standard: 'WIA-AGRI-021',
      timestamp: new Date().toISOString(),
      sensorId: 'sensor_sm_001',
      data: {
        soilMoisture: 12.8,
        temperature: 38.5
      }
    })
  );
});
```

#### 2.1.3 Rules Engine

```javascript
// AWS IoT Rule to process sensor data
{
  "sql": "SELECT * FROM 'wia-agri-021/+/sensors/data' WHERE data.soilMoisture < 12",
  "actions": [
    {
      "lambda": {
        "functionArn": "arn:aws:lambda:us-east-1:123456789:function:handleLowMoisture"
      }
    },
    {
      "sns": {
        "targetArn": "arn:aws:sns:us-east-1:123456789:irrigation-alerts",
        "roleArn": "arn:aws:iam::123456789:role/iot-sns-role"
      }
    }
  ]
}
```

### 2.2 Azure IoT Hub

#### 2.2.1 Device Registration

```javascript
const iothub = require('azure-iothub');
const connectionString = 'HostName=...;SharedAccessKeyName=...;SharedAccessKey=...';

const registry = iothub.Registry.fromConnectionString(connectionString);

const device = {
  deviceId: 'farm_sahara_001_sensor_001',
  status: 'enabled',
  tags: {
    farmId: 'farm_sahara_001',
    standard: 'WIA-AGRI-021',
    location: 'zone_a1'
  }
};

registry.create(device, (err, deviceInfo) => {
  if (err) {
    console.error('Failed to create device:', err);
  } else {
    console.log('Device created:', deviceInfo);
  }
});
```

#### 2.2.2 Telemetry Ingestion

```javascript
const { Client } = require('azure-iot-device');
const { Mqtt } = require('azure-iot-device-mqtt');

const deviceConnectionString = 'HostName=...;DeviceId=...;SharedAccessKey=...';
const client = Client.fromConnectionString(deviceConnectionString, Mqtt);

client.open(err => {
  if (err) {
    console.error('Could not connect:', err);
  } else {
    console.log('Client connected');

    // Send telemetry
    const message = new Message(JSON.stringify({
      standard: 'WIA-AGRI-021',
      timestamp: new Date().toISOString(),
      farmId: 'farm_sahara_001',
      data: {
        temperature: 38.5,
        humidity: 15.2
      }
    }));

    client.sendEvent(message, (err) => {
      if (err) {
        console.error('Failed to send message:', err);
      }
    });
  }
});
```

### 2.3 Google Cloud IoT

#### 2.3.1 Device Management

```python
from google.cloud import iot_v1

client = iot_v1.DeviceManagerClient()
parent = f"projects/{project_id}/locations/{region}/registries/{registry_id}"

device = {
    'id': 'farm_sahara_001_sensor_001',
    'credentials': [
        {
            'public_key': {
                'format': 'RSA_X509_PEM',
                'key': public_key
            }
        }
    ],
    'metadata': {
        'farmId': 'farm_sahara_001',
        'standard': 'WIA-AGRI-021',
        'sensorType': 'soil_moisture'
    }
}

response = client.create_device(parent=parent, device=device)
print(f"Device created: {response.id}")
```

#### 2.3.2 Pub/Sub Integration

```python
from google.cloud import pubsub_v1
import json

publisher = pubsub_v1.PublisherClient()
topic_path = publisher.topic_path(project_id, topic_id)

# Publish sensor data
data = {
    'standard': 'WIA-AGRI-021',
    'timestamp': datetime.utcnow().isoformat(),
    'farmId': 'farm_sahara_001',
    'sensorId': 'sensor_sm_001',
    'data': {
        'soilMoisture': 12.8,
        'temperature': 38.5
    }
}

message_bytes = json.dumps(data).encode('utf-8')
future = publisher.publish(topic_path, message_bytes)
print(f"Published message ID: {future.result()}")
```

---

## 3. Analytics Platform Integration

### 3.1 Grafana Dashboard

#### 3.1.1 Data Source Configuration

```yaml
apiVersion: 1
datasources:
  - name: WIA-AGRI-021
    type: prometheus
    url: https://metrics.wia-agri-021.org
    access: proxy
    isDefault: true
    jsonData:
      httpMethod: GET
      customQueryParameters: 'farmId=farm_sahara_001'
    secureJsonData:
      apiKey: 'your_api_key'
```

#### 3.1.2 Dashboard JSON

```json
{
  "dashboard": {
    "title": "Desert Agriculture Monitoring",
    "panels": [
      {
        "id": 1,
        "title": "Soil Moisture",
        "type": "graph",
        "targets": [
          {
            "expr": "wia_agri_021_soil_moisture{farmId=\"farm_sahara_001\"}",
            "legendFormat": "{{zoneId}}"
          }
        ],
        "yaxes": [
          {
            "label": "Moisture (%)",
            "min": 0,
            "max": 100
          }
        ]
      },
      {
        "id": 2,
        "title": "Temperature",
        "type": "graph",
        "targets": [
          {
            "expr": "wia_agri_021_temperature{farmId=\"farm_sahara_001\",type=\"air\"}",
            "legendFormat": "Air Temp"
          }
        ]
      }
    ]
  }
}
```

### 3.2 Tableau Integration

#### 3.2.1 Web Data Connector

```javascript
(function() {
  var myConnector = tableau.makeConnector();

  myConnector.getSchema = function(schemaCallback) {
    var cols = [
      { id: "timestamp", dataType: tableau.dataTypeEnum.datetime },
      { id: "farmId", dataType: tableau.dataTypeEnum.string },
      { id: "soilMoisture", dataType: tableau.dataTypeEnum.float },
      { id: "temperature", dataType: tableau.dataTypeEnum.float }
    ];

    var tableSchema = {
      id: "wiaAgri021SensorData",
      alias: "WIA-AGRI-021 Sensor Data",
      columns: cols
    };

    schemaCallback([tableSchema]);
  };

  myConnector.getData = function(table, doneCallback) {
    fetch('https://api.wia-agri-021.org/v1/sensors/data?farmId=farm_sahara_001', {
      headers: {
        'X-API-Key': tableau.password
      }
    })
    .then(response => response.json())
    .then(data => {
      var tableData = data.data.map(item => ({
        timestamp: item.timestamp,
        farmId: item.farmId,
        soilMoisture: item.data.soilMoisture,
        temperature: item.data.temperature
      }));

      table.appendRows(tableData);
      doneCallback();
    });
  };

  tableau.registerConnector(myConnector);
})();
```

### 3.3 Power BI Integration

#### 3.3.1 Custom Connector

```powerquery
let
    Source = (farmId as text) =>
    let
        apiKey = "your_api_key",
        url = "https://api.wia-agri-021.org/v1/sensors/data?farmId=" & farmId,
        headers = [
            #"X-API-Key" = apiKey,
            #"Content-Type" = "application/json"
        ],
        response = Web.Contents(url, [Headers=headers]),
        json = Json.Document(response),
        data = json[data],
        table = Table.FromList(data, Splitter.SplitByNothing(), null, null, ExtraValues.Error),
        expanded = Table.ExpandRecordColumn(table, "Column1",
            {"timestamp", "farmId", "sensorId", "data"},
            {"Timestamp", "FarmId", "SensorId", "Data"})
    in
        expanded
in
    Source
```

### 3.4 Elasticsearch Integration

#### 3.4.1 Index Mapping

```json
{
  "mappings": {
    "properties": {
      "timestamp": {
        "type": "date",
        "format": "strict_date_optional_time"
      },
      "farmId": {
        "type": "keyword"
      },
      "sensorId": {
        "type": "keyword"
      },
      "location": {
        "type": "geo_point"
      },
      "data": {
        "properties": {
          "temperature": {
            "type": "float"
          },
          "soilMoisture": {
            "type": "float"
          },
          "humidity": {
            "type": "float"
          }
        }
      }
    }
  }
}
```

#### 3.4.2 Data Ingestion

```javascript
const { Client } = require('@elastic/elasticsearch');
const client = new Client({ node: 'https://elasticsearch:9200' });

const indexSensorData = async (data) => {
  await client.index({
    index: 'wia-agri-021-sensors',
    body: {
      timestamp: data.timestamp,
      farmId: data.farmId,
      sensorId: data.sensorId,
      location: {
        lat: data.location.latitude,
        lon: data.location.longitude
      },
      data: data.data
    }
  });
};
```

---

## 4. ERP System Integration

### 4.1 SAP Integration

#### 4.1.1 OData Service

```xml
<?xml version="1.0" encoding="utf-8"?>
<edmx:Edmx Version="4.0" xmlns:edmx="http://docs.oasis-open.org/odata/ns/edmx">
  <edmx:DataServices>
    <Schema Namespace="WIA.AGRI.021" xmlns="http://docs.oasis-open.org/odata/ns/edm">
      <EntityType Name="SensorData">
        <Key>
          <PropertyRef Name="Id"/>
        </Key>
        <Property Name="Id" Type="Edm.String" Nullable="false"/>
        <Property Name="FarmId" Type="Edm.String"/>
        <Property Name="Timestamp" Type="Edm.DateTimeOffset"/>
        <Property Name="SoilMoisture" Type="Edm.Decimal"/>
        <Property Name="Temperature" Type="Edm.Decimal"/>
      </EntityType>
      <EntityContainer Name="Container">
        <EntitySet Name="SensorData" EntityType="WIA.AGRI.021.SensorData"/>
      </EntityContainer>
    </Schema>
  </edmx:DataServices>
</edmx:Edmx>
```

#### 4.1.2 BAPI Integration

```javascript
const sapRfc = require('node-rfc');

const client = new sapRfc.Client({
  user: 'USERNAME',
  passwd: 'PASSWORD',
  ashost: 'sap.example.com',
  sysnr: '00',
  client: '100'
});

client.connect((err) => {
  if (err) return console.error('Connection failed:', err);

  client.invoke('Z_WIA_AGRI_UPDATE', {
    IV_FARM_ID: 'farm_sahara_001',
    IT_SENSOR_DATA: [
      {
        SENSOR_ID: 'sensor_sm_001',
        TIMESTAMP: '2025-12-26T10:30:00',
        SOIL_MOISTURE: 12.8,
        TEMPERATURE: 38.5
      }
    ]
  }, (err, result) => {
    if (err) return console.error('Invoke failed:', err);
    console.log('Result:', result);
  });
});
```

### 4.2 Oracle ERP Cloud

#### 4.2.1 REST API Integration

```java
import oracle.apps.fnd.cp.request.Run;
import oracle.apps.financials.commonModules.shared.api.PublicRestClient;

public class WIAAgriIntegration {
    public static void updateAgriData(String farmId, SensorData data) {
        PublicRestClient client = new PublicRestClient();
        client.setBaseUrl("https://oracle-erp.example.com/fscmRestApi");
        client.setUsername("USERNAME");
        client.setPassword("PASSWORD");

        String payload = String.format(
            "{\"FarmId\":\"%s\",\"SensorData\":{\"SoilMoisture\":%.2f,\"Temperature\":%.2f}}",
            farmId, data.getSoilMoisture(), data.getTemperature()
        );

        Response response = client.post("/resources/latest/wiaAgriData", payload);
        System.out.println("Response: " + response.getBody());
    }
}
```

### 4.3 Microsoft Dynamics 365

#### 4.3.1 Web API Integration

```csharp
using Microsoft.Xrm.Sdk;
using Microsoft.Xrm.Sdk.Client;

public class WIAAgriIntegration
{
    public static void CreateSensorReading(IOrganizationService service, SensorData data)
    {
        Entity sensorReading = new Entity("wia_sensorreading");
        sensorReading["wia_farmid"] = data.FarmId;
        sensorReading["wia_sensorid"] = data.SensorId;
        sensorReading["wia_timestamp"] = data.Timestamp;
        sensorReading["wia_soilmoisture"] = data.SoilMoisture;
        sensorReading["wia_temperature"] = data.Temperature;

        Guid recordId = service.Create(sensorReading);
        Console.WriteLine($"Created record: {recordId}");
    }
}
```

---

## 5. Weather Service Integration

### 5.1 OpenWeatherMap

```javascript
const axios = require('axios');

class WeatherIntegration {
  constructor(apiKey) {
    this.apiKey = apiKey;
    this.baseUrl = 'https://api.openweathermap.org/data/2.5';
  }

  async getCurrentWeather(lat, lon) {
    const response = await axios.get(`${this.baseUrl}/weather`, {
      params: {
        lat: lat,
        lon: lon,
        appid: this.apiKey,
        units: 'metric'
      }
    });

    return {
      standard: 'WIA-AGRI-021',
      timestamp: new Date().toISOString(),
      source: 'openweathermap',
      data: {
        temperature: response.data.main.temp,
        humidity: response.data.main.humidity,
        pressure: response.data.main.pressure,
        windSpeed: response.data.wind.speed,
        windDirection: response.data.wind.deg,
        clouds: response.data.clouds.all,
        description: response.data.weather[0].description
      }
    };
  }

  async getForecast(lat, lon, days = 7) {
    const response = await axios.get(`${this.baseUrl}/forecast`, {
      params: {
        lat: lat,
        lon: lon,
        appid: this.apiKey,
        units: 'metric',
        cnt: days * 8 // 3-hour intervals
      }
    });

    return response.data.list.map(item => ({
      timestamp: new Date(item.dt * 1000).toISOString(),
      temperature: item.main.temp,
      humidity: item.main.humidity,
      precipitation: item.rain ? item.rain['3h'] : 0
    }));
  }
}
```

### 5.2 WeatherAPI.com

```python
import requests
from datetime import datetime

class WeatherAPIIntegration:
    def __init__(self, api_key):
        self.api_key = api_key
        self.base_url = 'http://api.weatherapi.com/v1'

    def get_current_weather(self, location):
        response = requests.get(
            f'{self.base_url}/current.json',
            params={
                'key': self.api_key,
                'q': location,
                'aqi': 'yes'
            }
        )

        data = response.json()
        return {
            'standard': 'WIA-AGRI-021',
            'timestamp': datetime.utcnow().isoformat(),
            'source': 'weatherapi',
            'data': {
                'temperature': data['current']['temp_c'],
                'humidity': data['current']['humidity'],
                'windSpeed': data['current']['wind_kph'],
                'solarRadiation': data['current'].get('uv', 0) * 100,
                'conditions': data['current']['condition']['text']
            }
        }

    def get_agriculture_forecast(self, location, days=7):
        response = requests.get(
            f'{self.base_url}/forecast.json',
            params={
                'key': self.api_key,
                'q': location,
                'days': days,
                'aqi': 'no',
                'alerts': 'yes'
            }
        )

        data = response.json()
        forecast = []

        for day in data['forecast']['forecastday']:
            forecast.append({
                'date': day['date'],
                'temperature': {
                    'min': day['day']['mintemp_c'],
                    'max': day['day']['maxtemp_c'],
                    'avg': day['day']['avgtemp_c']
                },
                'humidity': day['day']['avghumidity'],
                'precipitation': day['day']['totalprecip_mm'],
                'conditions': day['day']['condition']['text']
            })

        return forecast
```

---

## 6. Satellite Imagery Integration

### 6.1 Sentinel Hub

```python
from sentinelhub import (
    SHConfig,
    MimeType,
    CRS,
    BBox,
    SentinelHubRequest,
    DataCollection
)

class SatelliteImageryIntegration:
    def __init__(self, client_id, client_secret):
        self.config = SHConfig()
        self.config.sh_client_id = client_id
        self.config.sh_client_secret = client_secret

    def get_ndvi_data(self, bbox, date_range):
        evalscript = """
        //VERSION=3
        function setup() {
            return {
                input: ["B04", "B08"],
                output: { bands: 1, sampleType: "FLOAT32" }
            };
        }

        function evaluatePixel(sample) {
            let ndvi = (sample.B08 - sample.B04) / (sample.B08 + sample.B04);
            return [ndvi];
        }
        """

        request = SentinelHubRequest(
            evalscript=evalscript,
            input_data=[
                SentinelHubRequest.input_data(
                    data_collection=DataCollection.SENTINEL2_L2A,
                    time_interval=date_range,
                )
            ],
            responses=[
                SentinelHubRequest.output_response('default', MimeType.TIFF)
            ],
            bbox=bbox,
            size=[512, 512],
            config=self.config
        )

        return request.get_data()[0]

    def analyze_crop_health(self, ndvi_data):
        avg_ndvi = ndvi_data.mean()

        if avg_ndvi > 0.6:
            health_status = 'excellent'
        elif avg_ndvi > 0.4:
            health_status = 'good'
        elif avg_ndvi > 0.2:
            health_status = 'fair'
        else:
            health_status = 'poor'

        return {
            'standard': 'WIA-AGRI-021',
            'source': 'sentinel-2',
            'analysis': {
                'ndvi_average': float(avg_ndvi),
                'health_status': health_status,
                'vegetation_cover': float(avg_ndvi * 100)
            }
        }
```

---

## 7. Supply Chain Integration

### 7.1 Blockchain Integration

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract WIAAgriSupplyChain {
    struct Harvest {
        string farmId;
        string cropType;
        uint256 quantity;
        uint256 harvestDate;
        string qualityGrade;
        bool organic;
        bytes32 dataHash;
    }

    mapping(uint256 => Harvest) public harvests;
    uint256 public harvestCount;

    event HarvestRecorded(
        uint256 indexed harvestId,
        string farmId,
        string cropType,
        uint256 quantity
    );

    function recordHarvest(
        string memory _farmId,
        string memory _cropType,
        uint256 _quantity,
        string memory _qualityGrade,
        bool _organic,
        bytes32 _dataHash
    ) public returns (uint256) {
        harvestCount++;

        harvests[harvestCount] = Harvest({
            farmId: _farmId,
            cropType: _cropType,
            quantity: _quantity,
            harvestDate: block.timestamp,
            qualityGrade: _qualityGrade,
            organic: _organic,
            dataHash: _dataHash
        });

        emit HarvestRecorded(harvestCount, _farmId, _cropType, _quantity);

        return harvestCount;
    }

    function verifyHarvest(uint256 _harvestId, bytes32 _dataHash)
        public
        view
        returns (bool)
    {
        return harvests[_harvestId].dataHash == _dataHash;
    }
}
```

### 7.2 Smart Contract Integration

```javascript
const Web3 = require('web3');
const web3 = new Web3('https://mainnet.infura.io/v3/YOUR_INFURA_KEY');

class SupplyChainIntegration {
  constructor(contractAddress, abi) {
    this.contract = new web3.eth.Contract(abi, contractAddress);
  }

  async recordHarvest(farmId, cropData, privateKey) {
    const account = web3.eth.accounts.privateKeyToAccount(privateKey);

    // Hash the harvest data
    const dataHash = web3.utils.soliditySha3(
      { type: 'string', value: farmId },
      { type: 'string', value: cropData.type },
      { type: 'uint256', value: cropData.quantity },
      { type: 'string', value: JSON.stringify(cropData.sensorData) }
    );

    // Create transaction
    const tx = this.contract.methods.recordHarvest(
      farmId,
      cropData.type,
      cropData.quantity,
      cropData.qualityGrade,
      cropData.organic,
      dataHash
    );

    const gas = await tx.estimateGas({ from: account.address });
    const gasPrice = await web3.eth.getGasPrice();

    const signedTx = await account.signTransaction({
      to: this.contract.options.address,
      data: tx.encodeABI(),
      gas,
      gasPrice
    });

    const receipt = await web3.eth.sendSignedTransaction(
      signedTx.rawTransaction
    );

    return receipt;
  }
}
```

---

## 8. Legacy System Integration

### 8.1 CSV Export

```javascript
const { Parser } = require('json2csv');
const fs = require('fs');

class CSVExporter {
  async exportSensorData(farmId, startDate, endDate) {
    // Fetch data from API
    const response = await fetch(
      `https://api.wia-agri-021.org/v1/sensors/data?farmId=${farmId}&startTime=${startDate}&endTime=${endDate}`,
      {
        headers: { 'X-API-Key': process.env.API_KEY }
      }
    );

    const data = await response.json();

    // Define CSV fields
    const fields = [
      'timestamp',
      'farmId',
      'sensorId',
      'sensorType',
      'value',
      'unit',
      'location'
    ];

    // Flatten data
    const flatData = data.data.map(item => ({
      timestamp: item.timestamp,
      farmId: item.farmId,
      sensorId: item.sensorId,
      sensorType: item.type,
      value: item.data.value,
      unit: item.data.unit,
      location: `${item.location.latitude},${item.location.longitude}`
    }));

    // Convert to CSV
    const parser = new Parser({ fields });
    const csv = parser.parse(flatData);

    // Save to file
    fs.writeFileSync(`sensor_data_${farmId}_${Date.now()}.csv`, csv);

    return csv;
  }
}
```

### 8.2 FTP Integration

```javascript
const FtpClient = require('ftp');

class FTPIntegration {
  constructor(config) {
    this.client = new FtpClient();
    this.config = config;
  }

  async uploadData(localFile, remotePath) {
    return new Promise((resolve, reject) => {
      this.client.on('ready', () => {
        this.client.put(localFile, remotePath, (err) => {
          if (err) reject(err);
          this.client.end();
          resolve();
        });
      });

      this.client.connect(this.config);
    });
  }

  async scheduleDataExport(farmId, schedule) {
    const cron = require('node-cron');

    cron.schedule(schedule, async () => {
      // Export data
      const exporter = new CSVExporter();
      const csv = await exporter.exportSensorData(
        farmId,
        new Date(Date.now() - 86400000), // Last 24 hours
        new Date()
      );

      // Upload via FTP
      const filename = `wia_agri_${farmId}_${Date.now()}.csv`;
      fs.writeFileSync(`/tmp/${filename}`, csv);

      await this.uploadData(`/tmp/${filename}`, `/data/${filename}`);
      console.log(`Data exported and uploaded: ${filename}`);
    });
  }
}
```

---

## 9. Best Practices

### 9.1 API Integration Guidelines

1. **Use API Keys Securely**: Store in environment variables or secrets manager
2. **Implement Retry Logic**: Handle transient failures gracefully
3. **Rate Limit Compliance**: Respect API rate limits
4. **Error Handling**: Implement comprehensive error handling
5. **Logging**: Log all API calls for debugging
6. **Caching**: Cache responses when appropriate
7. **Versioning**: Always specify API version

### 9.2 Data Synchronization

1. **Idempotency**: Ensure operations can be safely retried
2. **Timestamps**: Use UTC timestamps consistently
3. **Conflict Resolution**: Implement conflict resolution strategy
4. **Batch Processing**: Process data in batches for efficiency
5. **Data Validation**: Validate all incoming data

### 9.3 Security Best Practices

1. **TLS/SSL**: Always use encrypted connections
2. **Authentication**: Implement proper authentication
3. **Authorization**: Use role-based access control
4. **Data Encryption**: Encrypt sensitive data
5. **Audit Logging**: Log all access and changes
6. **Regular Updates**: Keep dependencies updated

---

## 10. Testing Integration

### 10.1 Integration Testing

```javascript
const chai = require('chai');
const expect = chai.expect;

describe('WIA-AGRI-021 Integration Tests', () => {
  it('should fetch sensor data from API', async () => {
    const response = await fetch(
      'https://api.wia-agri-021.org/v1/sensors/data?farmId=farm_sahara_001',
      {
        headers: { 'X-API-Key': process.env.API_KEY }
      }
    );

    expect(response.status).to.equal(200);

    const data = await response.json();
    expect(data.success).to.be.true;
    expect(data.data).to.be.an('array');
  });

  it('should integrate with weather API', async () => {
    const weather = new WeatherIntegration(process.env.WEATHER_API_KEY);
    const data = await weather.getCurrentWeather(31.7917, -7.0926);

    expect(data).to.have.property('standard', 'WIA-AGRI-021');
    expect(data.data).to.have.property('temperature');
    expect(data.data).to.have.property('humidity');
  });
});
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA · All Rights Reserved
