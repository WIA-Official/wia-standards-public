# WIA-AGRI-018: Vertical Farming Standard
## Phase 4: Integration Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines integration patterns for vertical farming systems with external platforms including smart buildings, supply chains, marketplaces, AI/ML services, and enterprise systems.

### 1.1 Integration Scope

- **Building Management Systems (BMS)**: HVAC, lighting, energy monitoring
- **Supply Chain Platforms**: Traceability, blockchain, logistics
- **E-Commerce/Marketplaces**: Digital sales channels
- **AI/ML Services**: Predictive analytics, optimization
- **Enterprise Systems**: ERP, CRM, accounting
- **Government Systems**: Food safety reporting, certifications

---

## 2. Building Management System Integration

### 2.1 BACnet Integration

**Overview:** BACnet (Building Automation and Control Networks) is the standard protocol for building automation.

**Connection Configuration:**
```json
{
  "protocol": "BACnet/IP",
  "deviceInstance": 1001,
  "network": {
    "port": 47808,
    "broadcastAddress": "192.168.1.255"
  },
  "deviceObjects": [
    {
      "objectType": "ANALOG_INPUT",
      "objectInstance": 1,
      "objectName": "VF-Tier3-Temperature",
      "presentValue": 22.5,
      "units": "degrees-celsius"
    },
    {
      "objectType": "ANALOG_INPUT",
      "objectInstance": 2,
      "objectName": "VF-Tier3-Humidity",
      "presentValue": 65,
      "units": "percent"
    },
    {
      "objectType": "ANALOG_OUTPUT",
      "objectInstance": 10,
      "objectName": "VF-Tier3-HVAC-Setpoint",
      "presentValue": 22.0,
      "units": "degrees-celsius"
    }
  ]
}
```

**Data Mapping:**
```javascript
// Vertical Farm → BACnet BMS
const bacnetMapping = {
  'farm.tier3.temperature': {
    objectType: 'ANALOG_INPUT',
    objectInstance: 1,
    updateInterval: 30000 // 30 seconds
  },
  'farm.tier3.humidity': {
    objectType: 'ANALOG_INPUT',
    objectInstance: 2,
    updateInterval: 30000
  },
  'farm.tier3.hvac.setpoint': {
    objectType: 'ANALOG_OUTPUT',
    objectInstance: 10,
    bidirectional: true
  }
};
```

### 2.2 Modbus Integration

**Overview:** Modbus RTU/TCP for industrial sensors and actuators.

**Configuration:**
```json
{
  "protocol": "Modbus TCP",
  "host": "192.168.1.100",
  "port": 502,
  "slaveId": 1,
  "registers": [
    {
      "name": "temperature_tier3",
      "address": 30001,
      "type": "HOLDING_REGISTER",
      "dataType": "FLOAT32",
      "scale": 0.1,
      "unit": "celsius"
    },
    {
      "name": "humidity_tier3",
      "address": 30003,
      "type": "HOLDING_REGISTER",
      "dataType": "FLOAT32",
      "scale": 0.1,
      "unit": "percent"
    }
  ]
}
```

**Python Example:**
```python
from pymodbus.client import ModbusTcpClient

client = ModbusTcpClient('192.168.1.100', port=502)

# Read temperature from Modbus
result = client.read_holding_registers(30001, 2, unit=1)
temperature = struct.unpack('>f',
  struct.pack('>HH', result.registers[0], result.registers[1]))[0]

print(f"Temperature: {temperature}°C")

# Write HVAC setpoint
setpoint = 23.0
setpoint_bytes = struct.pack('>f', setpoint)
registers = struct.unpack('>HH', setpoint_bytes)
client.write_registers(40001, list(registers), unit=1)
```

### 2.3 KNX Integration

**Overview:** KNX for European building automation.

**Group Addresses:**
```
1/0/1 - Tier 3 Temperature Sensor
1/0/2 - Tier 3 Humidity Sensor
1/0/3 - Tier 3 CO2 Sensor
2/0/1 - Tier 3 HVAC Control
2/0/2 - Tier 3 LED Lighting Control
```

**Telegram Example:**
```
Source: 1.1.1 (Temperature Sensor)
Destination: 1/0/1 (Group Address)
Value: 22.5°C (Encoded: 0x08 0xCA)
```

---

## 3. Supply Chain Integration

### 3.1 Blockchain Traceability

**Platform:** Ethereum, Hyperledger Fabric, or custom blockchain

**Smart Contract Structure:**
```solidity
// Ethereum Smart Contract Example
pragma solidity ^0.8.0;

contract VerticalFarmTraceability {
    struct HarvestRecord {
        string harvestId;
        string farmId;
        string batchId;
        uint256 harvestDate;
        uint256 weight;
        string cropType;
        string grade;
        address farmer;
        bool organic;
        bool pesticideFree;
    }

    mapping(string => HarvestRecord) public harvests;

    event HarvestRecorded(
        string harvestId,
        string farmId,
        uint256 timestamp
    );

    function recordHarvest(
        string memory harvestId,
        string memory farmId,
        string memory batchId,
        uint256 weight,
        string memory cropType,
        string memory grade
    ) public {
        harvests[harvestId] = HarvestRecord({
            harvestId: harvestId,
            farmId: farmId,
            batchId: batchId,
            harvestDate: block.timestamp,
            weight: weight,
            cropType: cropType,
            grade: grade,
            farmer: msg.sender,
            organic: true,
            pesticideFree: true
        });

        emit HarvestRecorded(harvestId, farmId, block.timestamp);
    }

    function getHarvest(string memory harvestId)
        public view returns (HarvestRecord memory) {
        return harvests[harvestId];
    }
}
```

**Integration Flow:**
```javascript
// Record harvest on blockchain
const Web3 = require('web3');
const web3 = new Web3('https://mainnet.infura.io/v3/YOUR_KEY');

const contract = new web3.eth.Contract(ABI, CONTRACT_ADDRESS);

async function recordHarvest(harvestData) {
  const tx = await contract.methods.recordHarvest(
    harvestData.harvestId,
    harvestData.farmId,
    harvestData.batchId,
    harvestData.weight,
    harvestData.cropType,
    harvestData.grade
  ).send({
    from: FARMER_ADDRESS,
    gas: 300000
  });

  console.log('Blockchain TX:', tx.transactionHash);
  return tx.transactionHash;
}
```

### 3.2 QR Code Generation

**Implementation:**
```javascript
const QRCode = require('qrcode');

async function generateTraceabilityQR(harvestId) {
  const traceabilityData = {
    standard: 'WIA-AGRI-018',
    version: '1.0.0',
    harvestId: harvestId,
    verificationUrl: `https://trace.verticalfarm.io/${harvestId}`,
    blockchainTx: '0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb',
    farmId: 'VF-SEOUL-001',
    cropType: 'Green Leaf Lettuce',
    harvestDate: '2025-01-15',
    organic: true,
    pesticideFree: true,
    carbonFootprint: '0.058 kg CO2e/kg'
  };

  const qrString = JSON.stringify(traceabilityData);
  const qrImage = await QRCode.toDataURL(qrString);

  return qrImage; // Base64 encoded PNG
}
```

### 3.3 GS1 Standards Integration

**GTIN (Global Trade Item Number):**
```
Product: VF-SEOUL-001 Green Lettuce 200g
GTIN-13: 8801234567890
```

**GS1 DataMatrix:**
```
(01)08801234567890  # GTIN
(10)BATCH-2025-001  # Batch/Lot
(17)250115          # Expiration Date (YYMMDD)
(21)HRV-2025-001    # Serial Number
```

**EDI Integration (EDIFACT):**
```
UNH+1+ORDERS:D:96A:UN'
BGM+220+ORDER-2025-001+9'
DTM+137:20250115:102'
NAD+BY+BUYER001::9++Seoul Market+123 Market St++Seoul++12345+KR'
LIN+1++8801234567890:EN'
QTY+21:1080'  # 1080 packages
PRI+AAA:2.50:EA'  # Price per unit
UNT+8+1'
```

---

## 4. E-Commerce Integration

### 4.1 Shopify Integration

**API Configuration:**
```javascript
const Shopify = require('shopify-api-node');

const shopify = new Shopify({
  shopName: 'vertical-farm-seoul',
  apiKey: 'your_api_key',
  password: 'your_password'
});

// Create product from harvest
async function createProductFromHarvest(harvest) {
  const product = await shopify.product.create({
    title: `Fresh ${harvest.cropType} - Pesticide Free`,
    body_html: `
      <strong>Farm-fresh vertical farming produce</strong>
      <ul>
        <li>Harvest ID: ${harvest.harvestId}</li>
        <li>Harvest Date: ${harvest.harvestDate}</li>
        <li>Grade: ${harvest.grade}</li>
        <li>Carbon Footprint: ${harvest.carbonFootprint}</li>
      </ul>
    `,
    vendor: 'Seoul Vertical Farm',
    product_type: 'Fresh Vegetables',
    tags: ['organic', 'pesticide-free', 'vertical-farm', 'sustainable'],
    variants: [
      {
        price: '3.50',
        sku: `VF-${harvest.harvestId}`,
        inventory_quantity: harvest.packageCount,
        weight: 200,
        weight_unit: 'g'
      }
    ],
    images: [
      { src: harvest.productImageUrl }
    ]
  });

  return product;
}
```

### 4.2 WooCommerce Integration

**REST API:**
```php
<?php
require 'vendor/autoload.php';

use Automattic\WooCommerce\Client;

$woocommerce = new Client(
    'https://vertical-farm-seoul.com',
    'ck_XXXX',
    'cs_XXXX',
    ['version' => 'wc/v3']
);

// Create product
$data = [
    'name' => 'Fresh Green Lettuce - Pesticide Free',
    'type' => 'simple',
    'regular_price' => '3.50',
    'description' => 'Harvested fresh from our vertical farm',
    'short_description' => 'Pesticide-free, sustainable',
    'sku' => 'VF-HRV-2025-001',
    'stock_quantity' => 1080,
    'manage_stock' => true,
    'categories' => [
        ['id' => 9] // Fresh Vegetables
    ],
    'meta_data' => [
        ['key' => 'harvest_id', 'value' => 'HRV-2025-001'],
        ['key' => 'farm_id', 'value' => 'VF-SEOUL-001'],
        ['key' => 'harvest_date', 'value' => '2025-01-15'],
        ['key' => 'blockchain_tx', 'value' => '0x742d35...']
    ]
];

$product = $woocommerce->post('products', $data);
?>
```

---

## 5. AI/ML Integration

### 5.1 TensorFlow Serving

**Model Deployment:**
```bash
# Deploy crop health prediction model
docker run -p 8501:8501 \
  -v /models/crop-health:/models/crop-health \
  -e MODEL_NAME=crop-health \
  tensorflow/serving
```

**Inference Request:**
```python
import requests
import json

def predict_harvest_time(sensor_data):
    url = 'http://localhost:8501/v1/models/crop-health:predict'

    payload = {
        'instances': [
            {
                'temperature': sensor_data['temperature'],
                'humidity': sensor_data['humidity'],
                'light_intensity': sensor_data['light_intensity'],
                'ec': sensor_data['ec'],
                'ph': sensor_data['ph'],
                'growth_days': sensor_data['growth_days']
            }
        ]
    }

    response = requests.post(url, json=payload)
    prediction = response.json()['predictions'][0]

    return {
        'optimal_harvest_date': prediction['harvest_date'],
        'expected_yield': prediction['yield_kg'],
        'quality_score': prediction['quality'],
        'confidence': prediction['confidence']
    }
```

### 5.2 AWS SageMaker Integration

**Endpoint Invocation:**
```python
import boto3
import json

sagemaker = boto3.client('sagemaker-runtime')

def optimize_nutrient_mix(current_params):
    payload = {
        'current_ec': current_params['ec'],
        'current_ph': current_params['ph'],
        'crop_type': current_params['crop_type'],
        'growth_stage': current_params['growth_stage'],
        'target_yield': current_params['target_yield']
    }

    response = sagemaker.invoke_endpoint(
        EndpointName='vertical-farm-nutrient-optimizer',
        ContentType='application/json',
        Body=json.dumps(payload)
    )

    result = json.loads(response['Body'].read())

    return {
        'nitrogen_ppm': result['nitrogen'],
        'phosphorus_ppm': result['phosphorus'],
        'potassium_ppm': result['potassium'],
        'target_ec': result['ec'],
        'target_ph': result['ph']
    }
```

### 5.3 Google Cloud AI Platform

**Vision API - Crop Health Detection:**
```python
from google.cloud import vision

def analyze_crop_health(image_path):
    client = vision.ImageAnnotatorClient()

    with open(image_path, 'rb') as image_file:
        content = image_file.read()

    image = vision.Image(content=content)
    response = client.label_detection(image=image)

    # Custom model for crop disease detection
    custom_response = client.annotate_image({
        'image': image,
        'features': [
            {'type_': vision.Feature.Type.LABEL_DETECTION},
            {'type_': vision.Feature.Type.IMAGE_PROPERTIES}
        ]
    })

    return {
        'health_score': calculate_health_score(response),
        'detected_issues': extract_issues(response),
        'recommendations': generate_recommendations(response)
    }
```

---

## 6. Enterprise System Integration

### 6.1 ERP Integration (SAP)

**SAP OData API:**
```javascript
const axios = require('axios');

async function syncHarvestToSAP(harvestData) {
  const sapEndpoint = 'https://sap.company.com/sap/opu/odata/sap/';

  const productionOrder = {
    Material: harvestData.cropType,
    ProductionPlant: 'VF-SEOUL',
    MfgOrderPlannedTotalQty: harvestData.totalWeight,
    ProductionUnit: 'KG',
    MfgOrderPlannedStartDate: harvestData.seedingDate,
    MfgOrderPlannedEndDate: harvestData.harvestDate,
    MfgOrderActualReleaseDate: harvestData.harvestDate
  };

  const response = await axios.post(
    `${sapEndpoint}API_PRODUCTION_ORDER_2_SRV/A_ProductionOrder_2`,
    productionOrder,
    {
      headers: {
        'Authorization': 'Basic ' + btoa('user:pass'),
        'Content-Type': 'application/json'
      }
    }
  );

  return response.data;
}
```

### 6.2 Accounting Integration (QuickBooks)

**QuickBooks API:**
```javascript
const QuickBooks = require('node-quickbooks');

const qbo = new QuickBooks(
  consumerKey,
  consumerSecret,
  accessToken,
  accessTokenSecret,
  realmId,
  useProductionEnvironment,
  debug
);

async function createSalesReceipt(orderData) {
  const receipt = {
    Line: [
      {
        DetailType: 'SalesItemLineDetail',
        Amount: orderData.totalAmount,
        SalesItemLineDetail: {
          ItemRef: {
            value: '1', // Product ID in QuickBooks
            name: orderData.productName
          },
          Qty: orderData.quantity,
          UnitPrice: orderData.unitPrice
        }
      }
    ],
    CustomerRef: {
      value: orderData.customerId
    },
    TxnDate: orderData.orderDate,
    PrivateNote: `Harvest ID: ${orderData.harvestId}`
  };

  qbo.createSalesReceipt(receipt, function(err, salesReceipt) {
    if (err) console.error(err);
    else console.log('Receipt created:', salesReceipt.Id);
  });
}
```

---

## 7. Government & Compliance Integration

### 7.1 Food Safety Reporting

**FDA FSMA Integration (US):**
```json
{
  "reportType": "HARVEST_NOTIFICATION",
  "farmId": "VF-SEOUL-001",
  "fei": "1234567890",
  "harvestId": "HRV-2025-001",
  "commodity": "LETTUCE",
  "harvestDate": "2025-01-15",
  "quantity": {
    "value": 216.5,
    "unit": "KG"
  },
  "destination": {
    "facilityName": "Seoul Distribution Center",
    "address": "123 Logistics Rd, Seoul",
    "fei": "0987654321"
  },
  "foodSafety": {
    "gwp_certified": true,
    "haccp_compliant": true,
    "pesticide_test": "PASS",
    "microbiology_test": "PASS"
  }
}
```

### 7.2 Organic Certification Integration

**USDA Organic Portal:**
```xml
<?xml version="1.0"?>
<OrganicCertificationReport>
  <FarmID>VF-SEOUL-001</FarmID>
  <CertificationNumber>USDA-ORG-2024-001</CertificationNumber>
  <ReportingPeriod>2025-Q1</ReportingPeriod>
  <Production>
    <Crop>
      <Name>Lettuce</Name>
      <Variety>Green Leaf</Variety>
      <Quantity>3248</Quantity>
      <Unit>KG</Unit>
    </Crop>
  </Production>
  <Inputs>
    <Nutrient>
      <Name>Organic Nitrogen</Name>
      <Source>Plant-based</Source>
      <Quantity>45</Quantity>
      <Certified>true</Certified>
    </Nutrient>
  </Inputs>
  <PestControl>
    <Method>None Required (Controlled Environment)</Method>
    <ChemicalsUsed>None</ChemicalsUsed>
  </PestControl>
</OrganicCertificationReport>
```

---

## 8. IoT Platform Integration

### 8.1 AWS IoT Core

**Thing Configuration:**
```json
{
  "thingName": "VF-SEOUL-001-TEMP-T3-01",
  "thingTypeName": "TemperatureSensor",
  "attributes": {
    "farmId": "VF-SEOUL-001",
    "tier": "3",
    "zone": "A",
    "manufacturer": "SensorTech"
  }
}
```

**MQTT Topics:**
```
Publish: $aws/things/VF-SEOUL-001-TEMP-T3-01/shadow/update
Subscribe: $aws/things/VF-SEOUL-001-TEMP-T3-01/shadow/update/delta
```

**Shadow Document:**
```json
{
  "state": {
    "reported": {
      "temperature": 22.5,
      "humidity": 65,
      "timestamp": 1640995845
    },
    "desired": {
      "sampleRate": 30,
      "alertThreshold": 27.0
    }
  }
}
```

### 8.2 Azure IoT Hub

**Device Twin:**
```json
{
  "deviceId": "VF-SEOUL-001-TEMP-T3-01",
  "etag": "AAAAAAAAAAE=",
  "properties": {
    "desired": {
      "telemetryInterval": 30000,
      "temperatureThreshold": {
        "min": 20,
        "max": 25
      }
    },
    "reported": {
      "firmwareVersion": "1.2.5",
      "batteryLevel": 85,
      "lastSeen": "2025-01-01T10:30:00Z"
    }
  }
}
```

---

## 9. Data Lake Integration

### 9.1 Apache Kafka

**Producer Configuration:**
```javascript
const { Kafka } = require('kafkajs');

const kafka = new Kafka({
  clientId: 'vertical-farm-producer',
  brokers: ['kafka1:9092', 'kafka2:9092']
});

const producer = kafka.producer();

async function publishSensorData(data) {
  await producer.send({
    topic: 'vertical-farm-sensors',
    messages: [
      {
        key: data.farmId,
        value: JSON.stringify(data),
        partition: data.tier % 8, // Partition by tier
        timestamp: Date.now()
      }
    ]
  });
}
```

### 9.2 Apache Spark Streaming

**Stream Processing:**
```python
from pyspark.sql import SparkSession
from pyspark.sql.functions import *

spark = SparkSession.builder \
    .appName("VerticalFarmAnalytics") \
    .getOrCreate()

# Read from Kafka
df = spark.readStream \
    .format("kafka") \
    .option("kafka.bootstrap.servers", "localhost:9092") \
    .option("subscribe", "vertical-farm-sensors") \
    .load()

# Parse JSON and aggregate
sensor_data = df.select(
    from_json(col("value").cast("string"), schema).alias("data")
).select("data.*")

# Calculate hourly averages
hourly_avg = sensor_data \
    .withWatermark("timestamp", "10 minutes") \
    .groupBy(
        window("timestamp", "1 hour"),
        "farmId",
        "tier"
    ).agg(
        avg("temperature").alias("avg_temp"),
        avg("humidity").alias("avg_humidity"),
        avg("co2").alias("avg_co2")
    )

# Write to data lake
query = hourly_avg.writeStream \
    .format("parquet") \
    .option("path", "s3://vertical-farm-datalake/hourly") \
    .option("checkpointLocation", "s3://checkpoints/") \
    .start()
```

---

## 10. Integration Testing

### 10.1 Test Scenarios

**Building Integration Test:**
```javascript
describe('BACnet Integration', () => {
  it('should read temperature from BACnet device', async () => {
    const bacnet = new BACnetClient('192.168.1.100');
    const temp = await bacnet.readProperty({
      objectType: 'ANALOG_INPUT',
      objectInstance: 1,
      property: 'PRESENT_VALUE'
    });

    expect(temp).toBeGreaterThan(15);
    expect(temp).toBeLessThan(30);
  });

  it('should write HVAC setpoint via BACnet', async () => {
    const bacnet = new BACnetClient('192.168.1.100');
    const result = await bacnet.writeProperty({
      objectType: 'ANALOG_OUTPUT',
      objectInstance: 10,
      property: 'PRESENT_VALUE',
      value: 23.0
    });

    expect(result.success).toBe(true);
  });
});
```

**Blockchain Integration Test:**
```javascript
describe('Blockchain Traceability', () => {
  it('should record harvest on blockchain', async () => {
    const harvestData = {
      harvestId: 'HRV-TEST-001',
      farmId: 'VF-SEOUL-001',
      weight: 100,
      cropType: 'Lettuce'
    };

    const txHash = await blockchain.recordHarvest(harvestData);
    expect(txHash).toMatch(/^0x[a-fA-F0-9]{64}$/);

    // Verify on blockchain
    const record = await blockchain.getHarvest('HRV-TEST-001');
    expect(record.farmId).toBe('VF-SEOUL-001');
  });
});
```

---

## 11. Best Practices

### 11.1 Error Handling

**Retry Logic:**
```javascript
async function withRetry(fn, maxRetries = 3, delay = 1000) {
  for (let i = 0; i < maxRetries; i++) {
    try {
      return await fn();
    } catch (error) {
      if (i === maxRetries - 1) throw error;
      await new Promise(resolve => setTimeout(resolve, delay * (i + 1)));
    }
  }
}

// Usage
const result = await withRetry(() =>
  externalAPI.sendData(sensorData)
);
```

### 11.2 Data Validation

**Schema Validation:**
```javascript
const Joi = require('joi');

const harvestSchema = Joi.object({
  harvestId: Joi.string().required(),
  farmId: Joi.string().required(),
  totalWeight: Joi.number().min(0).required(),
  cropType: Joi.string().required(),
  grade: Joi.string().valid('PREMIUM', 'GRADE_A', 'GRADE_B')
});

function validateHarvest(data) {
  const { error, value } = harvestSchema.validate(data);
  if (error) throw new Error(`Validation failed: ${error.message}`);
  return value;
}
```

---

## 12. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-01 | Initial integration specification |

---

**© 2025 WIA Standards · MIT License**
**弘익人間 (Benefit All Humanity)**
