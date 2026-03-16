# WIA-AGRI-001: Smart Farm Standard
## Phase 4 - Integration Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines integration patterns for smart farm systems with external platforms, services, and legacy agricultural equipment.

### 1.1 Integration Architecture

```
┌────────────────────────────────────────────────────────────┐
│                    Smart Farm Platform                     │
├────────────────────────────────────────────────────────────┤
│  IoT Platform   │  Weather API  │  Market Data │  AI/ML   │
│  Integration    │  Integration  │  Integration │ Services │
├────────────────────────────────────────────────────────────┤
│              Integration Gateway Layer                     │
│  (Protocol Translation, Data Mapping, Auth)                │
├────────────────────────────────────────────────────────────┤
│  AWS IoT │ Azure │ Google │ OpenWeather │ USDA │ TensorFlow│
│   Core   │  IoT  │  IoT   │     API     │  API │    Hub    │
└────────────────────────────────────────────────────────────┘
```

---

## 2. IoT Platform Integrations

### 2.1 AWS IoT Core

**Overview:** Managed cloud service for IoT device connectivity

#### 2.1.1 Connection Setup

```javascript
const awsIot = require('aws-iot-device-sdk');

const device = awsIot.device({
  keyPath: './certs/private.pem.key',
  certPath: './certs/certificate.pem.crt',
  caPath: './certs/AmazonRootCA1.pem',
  clientId: 'smart-farm-soil-001',
  host: 'a1b2c3d4e5f6g7.iot.us-east-1.amazonaws.com'
});

device.on('connect', () => {
  console.log('Connected to AWS IoT');
  device.subscribe('wia/smart-farm/farm-001/commands/+');
});

device.on('message', (topic, payload) => {
  const message = JSON.parse(payload.toString());
  handleCommand(message);
});

// Publish sensor data
const sensorData = {
  sensorId: 'soil-001',
  timestamp: Date.now(),
  measurements: { moisture: 68.5, temperature: 22.3 }
};
device.publish('wia/smart-farm/farm-001/sensors/data/soil-001',
               JSON.stringify(sensorData));
```

#### 2.1.2 Device Shadow (Digital Twin)

**Shadow Document:**
```json
{
  "state": {
    "reported": {
      "moisture": 68.5,
      "temperature": 22.3,
      "ph": 6.4,
      "battery": 85,
      "lastUpdated": 1704096600000
    },
    "desired": {
      "samplingInterval": 300,
      "alertThresholds": {
        "moistureMin": 60,
        "moistureMax": 80
      }
    }
  },
  "metadata": {
    "reported": {
      "moisture": {"timestamp": 1704096600}
    }
  },
  "version": 42
}
```

#### 2.1.3 Rules Engine

**Rule SQL:**
```sql
SELECT
  sensorId,
  measurements.soil.moisture as moisture,
  measurements.soil.temperature as temperature
FROM 'wia/smart-farm/+/sensors/data/+'
WHERE moisture < 60
```

**Actions:**
- Send SNS notification
- Store in DynamoDB
- Trigger Lambda function
- Forward to IoT Analytics

---

### 2.2 Azure IoT Hub

**Overview:** Cloud-based IoT solution accelerator

#### 2.2.1 Connection Setup

```javascript
const iothub = require('azure-iot-device-mqtt').Mqtt;
const Message = require('azure-iot-device').Message;
const Client = require('azure-iot-device').Client;

const connectionString = 'HostName=smart-farm-hub.azure-devices.net;DeviceId=soil-001;SharedAccessKey=...';
const client = Client.fromConnectionString(connectionString, iothub);

client.open(err => {
  if (err) {
    console.error('Connection error:', err);
  } else {
    console.log('Connected to Azure IoT Hub');

    // Receive cloud-to-device messages
    client.on('message', msg => {
      console.log('Message received:', msg.getData().toString());
      client.complete(msg, console.log);
    });
  }
});

// Send device-to-cloud message
const telemetry = {
  sensorId: 'soil-001',
  moisture: 68.5,
  temperature: 22.3
};
const message = new Message(JSON.stringify(telemetry));
client.sendEvent(message, console.log);
```

#### 2.2.2 Device Twin

**Update Twin:**
```javascript
client.getTwin((err, twin) => {
  if (err) {
    console.error('Get twin error:', err);
  } else {
    // Update reported properties
    twin.properties.reported.update({
      moisture: 68.5,
      temperature: 22.3,
      battery: 85
    }, console.log);

    // Listen for desired property changes
    twin.on('properties.desired', delta => {
      console.log('Desired properties changed:', delta);
      // Apply new configuration
    });
  }
});
```

#### 2.2.3 Direct Methods

```javascript
client.onDeviceMethod('irrigate', (request, response) => {
  const duration = request.payload.duration;

  // Execute irrigation
  startIrrigation(duration).then(() => {
    response.send(200, 'Irrigation completed', err => {
      if (err) console.error('Response error:', err);
    });
  });
});
```

---

### 2.3 Google Cloud IoT

**Overview:** Fully managed service for device connectivity

#### 2.3.1 Connection Setup

```javascript
const jwt = require('jsonwebtoken');
const mqtt = require('mqtt');
const fs = require('fs');

// Create JWT token
const createJwt = (projectId, privateKeyFile, algorithm) => {
  const token = {
    iat: parseInt(Date.now() / 1000),
    exp: parseInt(Date.now() / 1000) + 20 * 60, // 20 minutes
    aud: projectId
  };
  const privateKey = fs.readFileSync(privateKeyFile);
  return jwt.sign(token, privateKey, { algorithm: algorithm });
};

const mqttClient = mqtt.connect({
  host: 'mqtt.googleapis.com',
  port: 8883,
  protocol: 'mqtts',
  clientId: `projects/${projectId}/locations/${region}/registries/${registryId}/devices/${deviceId}`,
  username: 'unused',
  password: createJwt(projectId, privateKeyFile, 'RS256')
});

// Publish telemetry
const topic = `/devices/${deviceId}/events`;
mqttClient.publish(topic, JSON.stringify({ moisture: 68.5 }));

// Subscribe to config updates
const configTopic = `/devices/${deviceId}/config`;
mqttClient.subscribe(configTopic);
```

---

## 3. Weather API Integrations

### 3.1 OpenWeatherMap

**API Key:** Required (free tier: 60 calls/minute)

#### 3.1.1 Current Weather

```javascript
const axios = require('axios');

async function getCurrentWeather(lat, lon, apiKey) {
  const url = `https://api.openweathermap.org/data/2.5/weather`;
  const response = await axios.get(url, {
    params: {
      lat: lat,
      lon: lon,
      appid: apiKey,
      units: 'metric'
    }
  });

  return {
    temperature: response.data.main.temp,
    humidity: response.data.main.humidity,
    pressure: response.data.main.pressure,
    windSpeed: response.data.wind.speed,
    windDirection: response.data.wind.deg,
    cloudCover: response.data.clouds.all,
    description: response.data.weather[0].description
  };
}
```

#### 3.1.2 5-Day Forecast

```javascript
async function getForecast(lat, lon, apiKey) {
  const url = `https://api.openweathermap.org/data/2.5/forecast`;
  const response = await axios.get(url, {
    params: { lat, lon, appid: apiKey, units: 'metric' }
  });

  // Parse 3-hour intervals into daily forecasts
  const dailyForecasts = response.data.list.reduce((acc, item) => {
    const date = item.dt_txt.split(' ')[0];
    if (!acc[date]) {
      acc[date] = { temps: [], humidity: [], rain: [] };
    }
    acc[date].temps.push(item.main.temp);
    acc[date].humidity.push(item.main.humidity);
    acc[date].rain.push(item.rain ? item.rain['3h'] : 0);
    return acc;
  }, {});

  return Object.entries(dailyForecasts).map(([date, data]) => ({
    date: date,
    tempMin: Math.min(...data.temps),
    tempMax: Math.max(...data.temps),
    avgHumidity: data.humidity.reduce((a,b) => a+b) / data.humidity.length,
    totalRain: data.rain.reduce((a,b) => a+b)
  }));
}
```

#### 3.1.3 Agricultural Weather Index API

```javascript
async function getAgroWeather(lat, lon, apiKey) {
  const url = `https://api.openweathermap.org/data/2.5/agro/weather`;
  const response = await axios.get(url, {
    params: {
      lat, lon, appid: apiKey,
      // Additional agro-specific parameters
      polyid: 'polygon_id' // For specific field polygons
    }
  });

  return {
    soilTemperature: response.data.main.temp,
    soilMoisture: response.data.soil.moisture,
    precipitation: response.data.rain
  };
}
```

---

### 3.2 Weather.com (IBM Weather Company)

```javascript
async function getWeatherComData(lat, lon, apiKey) {
  const url = `https://api.weather.com/v3/wx/forecast/daily/5day`;
  const response = await axios.get(url, {
    params: {
      geocode: `${lat},${lon}`,
      format: 'json',
      units: 'm',
      language: 'en-US',
      apiKey: apiKey
    }
  });

  return response.data;
}
```

---

## 4. Agricultural Market Data Integration

### 4.1 USDA Market News API

**Endpoint:** `https://marsapi.ams.usda.gov/services/v1.2/reports`

```javascript
async function getUSDAMarketData(commodity) {
  const url = 'https://marsapi.ams.usda.gov/services/v1.2/reports';
  const response = await axios.get(url, {
    params: {
      q: commodity, // e.g., "tomatoes"
      reportType: 'daily_prices'
    }
  });

  return response.data.results.map(report => ({
    commodity: report.commodity,
    variety: report.variety,
    priceMin: report.lowPrice,
    priceMax: report.highPrice,
    priceAvg: report.avgPrice,
    unit: report.unit,
    market: report.marketName,
    date: report.reportDate
  }));
}
```

---

### 4.2 FAO GIEWS (Global Food Price Monitor)

```javascript
async function getFAOFoodPrices(country, commodity) {
  // FAO GIEWS data API
  const url = `http://www.fao.org/giews/food-prices/api/data`;
  const response = await axios.get(url, {
    params: {
      country: country, // ISO 3166-1 alpha-3
      commodity: commodity,
      format: 'json'
    }
  });

  return response.data;
}
```

---

## 5. AI/ML Service Integration

### 5.1 TensorFlow Serving (Disease Detection)

**Setup:**
```bash
docker run -p 8501:8501 \
  --mount type=bind,source=/path/to/model,target=/models/disease_detection \
  -e MODEL_NAME=disease_detection \
  tensorflow/serving
```

**Inference Request:**
```javascript
async function detectDisease(imageBuffer) {
  const imageArray = Array.from(new Uint8Array(imageBuffer));

  const response = await axios.post(
    'http://localhost:8501/v1/models/disease_detection:predict',
    {
      instances: [{
        input_image: imageArray
      }]
    }
  );

  const predictions = response.data.predictions[0];
  return {
    disease: predictions.class_name,
    confidence: predictions.confidence,
    severity: predictions.severity
  };
}
```

---

### 5.2 AWS SageMaker (Yield Prediction)

**Invoke Endpoint:**
```javascript
const AWS = require('aws-sdk');
const sagemaker = new AWS.SageMakerRuntime();

async function predictYield(farmData) {
  const params = {
    EndpointName: 'yield-prediction-endpoint',
    ContentType: 'application/json',
    Body: JSON.stringify({
      crop_type: farmData.cropType,
      area: farmData.area,
      soil_moisture: farmData.soilMoisture,
      temperature: farmData.temperature,
      humidity: farmData.humidity,
      growth_stage: farmData.growthStage
    })
  };

  const response = await sagemaker.invokeEndpoint(params).promise();
  const prediction = JSON.parse(response.Body.toString());

  return {
    estimatedYield: prediction.yield_kg,
    confidence: prediction.confidence,
    harvestDate: prediction.optimal_harvest_date
  };
}
```

---

### 5.3 Azure Cognitive Services (Image Analysis)

```javascript
const { ComputerVisionClient } = require('@azure/cognitiveservices-computervision');
const { CognitiveServicesCredentials } = require('@azure/ms-rest-azure-js');

async function analyzeCropImage(imageUrl) {
  const credentials = new CognitiveServicesCredentials(subscriptionKey);
  const client = new ComputerVisionClient(credentials, endpoint);

  const result = await client.analyzeImage(imageUrl, {
    visualFeatures: ['Color', 'Tags', 'Objects', 'Description']
  });

  return {
    description: result.description.captions[0].text,
    tags: result.tags.map(tag => ({ name: tag.name, confidence: tag.confidence })),
    dominantColors: result.color.dominantColors
  };
}
```

---

## 6. Drone System Integration

### 6.1 DJI SDK Integration

```javascript
const dji = require('dji-sdk');

async function captureFarmImage(drone, coordinates) {
  // Navigate to location
  await drone.navigation.flyTo({
    latitude: coordinates.lat,
    longitude: coordinates.lon,
    altitude: 50 // meters
  });

  // Capture multispectral image
  const image = await drone.camera.capturePhoto({
    mode: 'multispectral',
    bands: ['red', 'green', 'blue', 'nir', 're'] // Near-infrared, Red-edge
  });

  // Calculate NDVI
  const ndvi = calculateNDVI(image.nir, image.red);

  return {
    imageUrl: image.url,
    ndvi: ndvi,
    timestamp: new Date(),
    location: coordinates
  };
}

function calculateNDVI(nir, red) {
  // NDVI = (NIR - Red) / (NIR + Red)
  return (nir - red) / (nir + red);
}
```

---

### 6.2 ArduPilot MAVLink Integration

```javascript
const mavlink = require('node-mavlink');

const connection = new mavlink.MAVLink(null, 1, 1); // sysid, compid

// Send mission waypoints
function uploadMission(waypoints) {
  const mission = waypoints.map((wp, index) => ({
    seq: index,
    frame: mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    command: mavlink.MAV_CMD_NAV_WAYPOINT,
    param1: 0, // Hold time
    param2: 0, // Acceptance radius
    param3: 0, // Pass through
    param4: 0, // Yaw
    x: wp.latitude,
    y: wp.longitude,
    z: wp.altitude
  }));

  // Send MISSION_COUNT
  connection.sendMessage(new mavlink.messages.mission_count(
    mission.length,
    1, // target_system
    1  // target_component
  ));
}
```

---

## 7. Blockchain Integration (Traceability)

### 7.1 Farm-to-Table Traceability

**Smart Contract (Solidity):**
```solidity
pragma solidity ^0.8.0;

contract FarmTraceability {
    struct Harvest {
        string farmId;
        string cropType;
        uint256 quantity;
        uint256 harvestDate;
        string certifications;
        bytes32 dataHash;
    }

    mapping(bytes32 => Harvest) public harvests;

    event HarvestRecorded(bytes32 indexed harvestId, string farmId, string cropType);

    function recordHarvest(
        string memory farmId,
        string memory cropType,
        uint256 quantity,
        string memory certifications,
        bytes32 dataHash
    ) public returns (bytes32) {
        bytes32 harvestId = keccak256(abi.encodePacked(farmId, block.timestamp));

        harvests[harvestId] = Harvest({
            farmId: farmId,
            cropType: cropType,
            quantity: quantity,
            harvestDate: block.timestamp,
            certifications: certifications,
            dataHash: dataHash
        });

        emit HarvestRecorded(harvestId, farmId, cropType);
        return harvestId;
    }

    function verifyHarvest(bytes32 harvestId) public view returns (Harvest memory) {
        return harvests[harvestId];
    }
}
```

**Integration Code:**
```javascript
const { ethers } = require('ethers');

async function recordHarvestOnChain(harvestData) {
  const provider = new ethers.providers.JsonRpcProvider(rpcUrl);
  const wallet = new ethers.Wallet(privateKey, provider);
  const contract = new ethers.Contract(contractAddress, abi, wallet);

  // Hash the sensor data for integrity
  const dataHash = ethers.utils.keccak256(JSON.stringify(harvestData.sensorData));

  const tx = await contract.recordHarvest(
    harvestData.farmId,
    harvestData.cropType,
    harvestData.quantity,
    harvestData.certifications,
    dataHash
  );

  const receipt = await tx.wait();
  return receipt.events[0].args.harvestId;
}
```

---

## 8. ERP System Integration

### 8.1 SAP Integration

**SAP Plant Maintenance (PM) Module:**
```javascript
const axios = require('axios');

async function createSAPWorkOrder(equipmentId, maintenanceType) {
  const sapEndpoint = 'https://sap-server.com/sap/opu/odata/sap/API_MAINTENANCEORDER';

  const workOrder = {
    OrderType: maintenanceType,
    Equipment: equipmentId,
    Priority: '2',
    PlanningPlant: 'FARM001',
    MaintenanceActivityType: 'REPAIR'
  };

  const response = await axios.post(sapEndpoint, workOrder, {
    auth: {
      username: 'sap_user',
      password: 'sap_password'
    }
  });

  return response.data.OrderID;
}
```

---

### 8.2 Oracle NetSuite Integration

```javascript
async function syncInventoryToNetSuite(harvestData) {
  const netsuite = require('netsuite-rest');

  const inventoryItem = {
    itemId: `HARVEST_${harvestData.cropType}_${Date.now()}`,
    displayName: `${harvestData.cropType} - ${harvestData.farmId}`,
    quantity: harvestData.quantity,
    location: harvestData.farmId,
    unitPrice: harvestData.marketPrice,
    itemType: 'inventoryItem'
  };

  const response = await netsuite.createRecord({
    recordType: 'inventoryItem',
    bodyFields: inventoryItem
  });

  return response.id;
}
```

---

## 9. Legacy Equipment Integration

### 9.1 ISOBUS (ISO 11783) Integration

**Overview:** Agricultural machinery communication standard

```javascript
const isobus = require('isobus-connector');

// Connect to tractor CAN bus
const bus = new isobus.CANBus({
  interface: 'can0',
  baudRate: 250000
});

// Read fuel consumption
bus.on('message', (msg) => {
  if (msg.pgn === 65266) { // Fuel Consumption PGN
    const fuelRate = msg.data.readUInt16LE(0) * 0.05; // L/h
    console.log(`Fuel consumption: ${fuelRate} L/h`);

    // Send to cloud
    publishToCloud({
      equipmentId: 'tractor-001',
      metric: 'fuel_consumption',
      value: fuelRate,
      timestamp: Date.now()
    });
  }
});
```

---

### 9.2 Retrofit Kit for Analog Sensors

**Setup:**
```
Analog Sensor (4-20mA) → ADC (ADS1115) → Raspberry Pi → MQTT
```

**Code (Python):**
```python
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import paho.mqtt.client as mqtt

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
chan = AnalogIn(ads, ADS.P0)

# 4-20mA sensor (e.g., soil moisture)
# 4mA = 0%, 20mA = 100%
def read_sensor():
    voltage = chan.voltage
    current_ma = (voltage / 250) * 1000  # Using 250Ω resistor
    percentage = (current_ma - 4) / 16 * 100
    return percentage

mqtt_client = mqtt.Client()
mqtt_client.connect("mqtt.wiastandards.com", 8883)

while True:
    moisture = read_sensor()
    mqtt_client.publish("wia/smart-farm/farm-001/sensors/data/soil-analog",
                        f'{{"moisture": {moisture}}}')
    time.sleep(300)  # 5 minutes
```

---

## 10. Certification & Compliance Integration

### 10.1 Organic Certification Data Export

```javascript
function generateOrganicCertReport(farmId, startDate, endDate) {
  // Query database for certification-relevant data
  const report = {
    farmInfo: getFarmInfo(farmId),
    period: { start: startDate, end: endDate },
    inputs: {
      fertilizers: getFertilizerApplications(farmId, startDate, endDate),
      pesticides: getPesticideApplications(farmId, startDate, endDate),
      seeds: getSeedSources(farmId, startDate, endDate)
    },
    practices: {
      cropRotation: getCropRotationHistory(farmId),
      soilManagement: getSoilManagementPractices(farmId),
      waterManagement: getWaterUsage(farmId, startDate, endDate)
    },
    harvest: getHarvestRecords(farmId, startDate, endDate)
  };

  // Export to PDF for certification body
  return generatePDF(report);
}
```

---

### 10.2 GAP (Good Agricultural Practices) Compliance

```javascript
function checkGAPCompliance(farmId) {
  const checks = {
    waterQuality: checkWaterQuality(farmId), // Must meet drinking water standards
    pesticideRecords: checkPesticideRecordKeeping(farmId), // Complete records required
    workerSafety: checkWorkerSafetyTraining(farmId), // Training documentation
    traceability: checkTraceabilitySystem(farmId), // Lot tracking
    facilityHygiene: checkFacilityInspections(farmId) // Regular inspections
  };

  const compliant = Object.values(checks).every(check => check.passed);

  return {
    compliant: compliant,
    checks: checks,
    nextAuditDate: calculateNextAuditDate(),
    recommendations: generateRecommendations(checks)
  };
}
```

---

## 11. Implementation Checklist

### 11.1 IoT Platform Integration
- [ ] Choose IoT platform (AWS IoT, Azure IoT, Google Cloud IoT)
- [ ] Set up device registry
- [ ] Configure authentication (X.509 certificates, SAS tokens)
- [ ] Implement device twin/shadow synchronization
- [ ] Set up rules engine for data routing
- [ ] Configure alarms and notifications

### 11.2 External API Integration
- [ ] Obtain API keys (Weather, Market Data)
- [ ] Implement rate limiting and caching
- [ ] Set up error handling and retries
- [ ] Configure data refresh schedules
- [ ] Map external data to internal schema

### 11.3 AI/ML Integration
- [ ] Train or obtain pre-trained models
- [ ] Deploy models (TensorFlow Serving, SageMaker)
- [ ] Create inference endpoints
- [ ] Implement image preprocessing pipeline
- [ ] Set up model monitoring and retraining

### 11.4 Legacy System Integration
- [ ] Identify legacy protocols (Modbus, ISOBUS)
- [ ] Deploy protocol gateways
- [ ] Implement data mapping
- [ ] Test bidirectional communication
- [ ] Document API bridges

---

## 12. Reference Integrations

| System Type | Examples | Protocol | Difficulty |
|-------------|----------|----------|------------|
| IoT Platform | AWS IoT, Azure IoT | MQTT, HTTPS | Medium |
| Weather API | OpenWeatherMap | REST | Easy |
| Market Data | USDA, FAO | REST | Easy |
| AI/ML | TensorFlow, SageMaker | gRPC, REST | Hard |
| Drones | DJI, ArduPilot | Proprietary, MAVLink | Hard |
| Blockchain | Ethereum, Hyperledger | JSON-RPC, gRPC | Medium |
| ERP | SAP, Oracle | OData, REST | Hard |
| Legacy | ISOBUS, Modbus | CAN, TCP | Medium |

---

**Previous Phase:** [PHASE-3-PROTOCOL.md](./PHASE-3-PROTOCOL.md)

---

© 2025 WIA (World Certification Industry Association)
弘益人間 · Benefit All Humanity
