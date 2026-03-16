# WIA-IND-007 Phase 4: Integration Specification
## Food Safety Traceability Standard v1.0

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

---

## Overview

Phase 4 defines integration pathways connecting WIA-IND-007 traceability systems with enterprise resource planning (ERP), blockchain networks, IoT sensor platforms, and regulatory reporting systems. These integrations enable automated data flow, reduce manual effort, and ensure comprehensive traceability.

## 1. ERP System Integration

### 1.1 Supported ERP Platforms

**Major ERP Systems:**
- SAP S/4HANA
- Oracle ERP Cloud
- Microsoft Dynamics 365
- Infor CloudSuite
- NetSuite
- Sage X3
- Epicor ERP

### 1.2 Integration Architecture

**Integration Patterns:**
- **Real-time:** API-based synchronous integration
- **Near real-time:** Message queue (AMQP, Kafka)
- **Batch:** Scheduled ETL processes
- **Hybrid:** Critical events real-time, bulk data batch

**Reference Architecture:**
```
┌─────────────┐         ┌──────────────┐         ┌─────────────┐
│   ERP       │◄───────►│  Integration │◄───────►│ Traceability│
│   System    │   API   │    Layer     │   API   │   System    │
└─────────────┘         └──────────────┘         └─────────────┘
                              │
                              ▼
                        ┌──────────────┐
                        │   Message    │
                        │    Queue     │
                        └──────────────┘
```

### 1.3 Data Mapping

**ERP to Traceability Mapping:**

| ERP Entity | WIA-IND-007 Entity | Sync Direction |
|------------|-------------------|----------------|
| Material Master | Product | Bidirectional |
| Production Order | Batch Record | ERP → Trace |
| Goods Receipt | Supply Chain Event | ERP → Trace |
| Goods Issue | Supply Chain Event | ERP → Trace |
| Quality Inspection | Quality Test | ERP → Trace |
| Sales Order | Distribution Event | ERP → Trace |

### 1.4 SAP S/4HANA Integration

**OData Service Configuration:**
```xml
<edmx:Edmx Version="4.0">
  <edmx:DataServices>
    <Schema Namespace="WIA.Traceability">
      <EntityType Name="ProductBatch">
        <Key>
          <PropertyRef Name="MaterialNumber"/>
          <PropertyRef Name="BatchNumber"/>
        </Key>
        <Property Name="MaterialNumber" Type="Edm.String" Nullable="false"/>
        <Property Name="BatchNumber" Type="Edm.String" Nullable="false"/>
        <Property Name="ProductionDate" Type="Edm.DateTime"/>
        <Property Name="ExpirationDate" Type="Edm.DateTime"/>
        <Property Name="OriginLocation" Type="Edm.String"/>
        <Property Name="TraceabilityURL" Type="Edm.String"/>
      </EntityType>
    </Schema>
  </edmx:DataServices>
</edmx:Edmx>
```

**ABAP Function Module:**
```abap
FUNCTION Z_WIA_SUBMIT_BATCH_EVENT.
  IMPORTING
    IV_BATCH_NUMBER TYPE CHARG_D
    IV_EVENT_TYPE TYPE STRING
    IV_LOCATION TYPE STRING
    IS_EVENT_DATA TYPE ZSTR_EVENT_DATA
  EXPORTING
    EV_EVENT_ID TYPE STRING
    EV_BLOCKCHAIN_TX TYPE STRING
  EXCEPTIONS
    COMMUNICATION_ERROR
    DATA_VALIDATION_ERROR.
```

## 2. Blockchain Integration

### 2.1 Supported Blockchain Networks

**Public Blockchains:**
- Ethereum (Mainnet, Sepolia Testnet)
- Polygon (MATIC)
- Binance Smart Chain
- Avalanche

**Permissioned Blockchains:**
- Hyperledger Fabric
- Hyperledger Besu
- R3 Corda
- IBM Food Trust

### 2.2 Smart Contract Interface

**Food Traceability Smart Contract (Solidity):**
```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

contract FoodTraceability {
    struct BatchRecord {
        string batchNumber;
        string productGtin;
        uint256 productionDate;
        string originLocation;
        address producer;
        bool exists;
    }

    struct SupplyChainEvent {
        uint256 timestamp;
        string eventType;
        string location;
        address submitter;
        string dataHash;
    }

    mapping(string => BatchRecord) public batches;
    mapping(string => SupplyChainEvent[]) public events;

    event BatchCreated(string indexed batchNumber, string productGtin, address producer);
    event EventRecorded(string indexed batchNumber, string eventType, uint256 timestamp);

    function createBatch(
        string memory batchNumber,
        string memory productGtin,
        uint256 productionDate,
        string memory originLocation
    ) public {
        require(!batches[batchNumber].exists, "Batch already exists");

        batches[batchNumber] = BatchRecord({
            batchNumber: batchNumber,
            productGtin: productGtin,
            productionDate: productionDate,
            originLocation: originLocation,
            producer: msg.sender,
            exists: true
        });

        emit BatchCreated(batchNumber, productGtin, msg.sender);
    }

    function recordEvent(
        string memory batchNumber,
        string memory eventType,
        string memory location,
        string memory dataHash
    ) public {
        require(batches[batchNumber].exists, "Batch does not exist");

        events[batchNumber].push(SupplyChainEvent({
            timestamp: block.timestamp,
            eventType: eventType,
            location: location,
            submitter: msg.sender,
            dataHash: dataHash
        }));

        emit EventRecorded(batchNumber, eventType, block.timestamp);
    }

    function getBatchEvents(string memory batchNumber)
        public
        view
        returns (SupplyChainEvent[] memory)
    {
        return events[batchNumber];
    }
}
```

### 2.3 Web3 Integration Layer

**JavaScript/TypeScript Integration:**
```typescript
import { ethers } from 'ethers';

class BlockchainTraceability {
  private provider: ethers.Provider;
  private contract: ethers.Contract;
  private wallet: ethers.Wallet;

  constructor(rpcUrl: string, contractAddress: string, privateKey: string) {
    this.provider = new ethers.JsonRpcProvider(rpcUrl);
    this.wallet = new ethers.Wallet(privateKey, this.provider);

    const abi = [...]; // Contract ABI
    this.contract = new ethers.Contract(contractAddress, abi, this.wallet);
  }

  async createBatch(
    batchNumber: string,
    productGtin: string,
    productionDate: number,
    originLocation: string
  ): Promise<string> {
    const tx = await this.contract.createBatch(
      batchNumber,
      productGtin,
      productionDate,
      originLocation
    );

    const receipt = await tx.wait();
    return receipt.transactionHash;
  }

  async recordEvent(
    batchNumber: string,
    eventType: string,
    location: string,
    eventData: any
  ): Promise<string> {
    // Hash event data
    const dataHash = ethers.keccak256(
      ethers.toUtf8Bytes(JSON.stringify(eventData))
    );

    const tx = await this.contract.recordEvent(
      batchNumber,
      eventType,
      location,
      dataHash
    );

    const receipt = await tx.wait();
    return receipt.transactionHash;
  }
}
```

## 3. IoT Sensor Integration

### 3.1 Supported Protocols

**Communication Protocols:**
- MQTT (Message Queuing Telemetry Transport)
- CoAP (Constrained Application Protocol)
- HTTP/HTTPS REST APIs
- LoRaWAN (Long Range Wide Area Network)
- NB-IoT (Narrowband IoT)

### 3.2 Temperature Sensor Integration

**MQTT Topic Structure:**
```
wia/traceability/{organizationId}/{facilityId}/{sensorId}/temperature
```

**Message Format:**
```json
{
  "sensorId": "TEMP-SENSOR-001",
  "timestamp": "2025-12-27T14:00:00Z",
  "batchNumber": "BATCH-2025-001",
  "temperature": 4.2,
  "unit": "celsius",
  "batteryLevel": 87,
  "signalStrength": -65,
  "location": {
    "facilityId": "WAREHOUSE-001",
    "zone": "Cold Storage A"
  }
}
```

**Node.js MQTT Subscriber:**
```javascript
const mqtt = require('mqtt');
const client = mqtt.connect('mqtts://mqtt.wia.org:8883', {
  username: 'sensor-gateway',
  password: process.env.MQTT_PASSWORD,
  clientId: 'facility-001-gateway'
});

client.on('connect', () => {
  client.subscribe('wia/traceability/+/+/+/temperature', (err) => {
    if (!err) console.log('Subscribed to temperature sensors');
  });
});

client.on('message', async (topic, message) => {
  const data = JSON.parse(message.toString());

  // Validate temperature is within acceptable range
  if (data.temperature < 2.0 || data.temperature > 6.0) {
    await triggerTemperatureAlert(data);
  }

  // Store reading in database
  await storeTemperatureReading(data);

  // Submit to blockchain if configured
  if (process.env.BLOCKCHAIN_ENABLED === 'true') {
    await submitToBlockchain(data);
  }
});
```

### 3.3 GPS Tracker Integration

**Tracking Device Integration:**
```python
import asyncio
from aiohttp import ClientSession

class GPSTrackerIntegration:
    def __init__(self, api_key: str):
        self.api_key = api_key
        self.base_url = "https://api.traceability.wia.org/v1"

    async def submit_location(self, device_id: str, batch_number: str,
                             latitude: float, longitude: float):
        async with ClientSession() as session:
            payload = {
                "eventType": "LOCATION_UPDATE",
                "timestamp": datetime.utcnow().isoformat() + "Z",
                "batchNumber": batch_number,
                "deviceId": device_id,
                "location": {
                    "latitude": latitude,
                    "longitude": longitude
                },
                "philosophy": "弘益人間"
            }

            headers = {"X-API-Key": self.api_key}

            async with session.post(
                f"{self.base_url}/events",
                json=payload,
                headers=headers
            ) as response:
                return await response.json()
```

## 4. Regulatory Reporting Integration

### 4.1 FDA FSMA 204 Reporting

**Automated Report Generation:**
```json
{
  "fsma204Report": {
    "reportType": "Food Traceability List",
    "submissionDate": "2025-12-27",
    "firm": {
      "ffrNumber": "12345678901",
      "firmName": "ABC Food Company",
      "contactEmail": "regulatory@abcfood.com"
    },
    "products": [
      {
        "traceabilityLot": "BATCH-2025-001",
        "productDescription": "Organic Romaine Lettuce",
        "quantity": 10000,
        "unit": "lbs",
        "harvestDate": "2025-12-20",
        "location": {
          "farm": "ABC Farms",
          "coordinates": "36.7783,-119.4179"
        },
        "cooling": {
          "initialCoolingDate": "2025-12-20",
          "temperature": "34-38F"
        },
        "receivers": [
          {
            "receiverName": "XYZ Distribution",
            "receiveDate": "2025-12-21",
                "quantity": 10000
          }
        ]
      }
    ]
  }
}
```

### 4.2 EU Food Safety Authority (EFSA) Integration

**RASFF Portal Integration:**
```xml
<RASFF_Notification>
  <NotificationNumber>2025.0001</NotificationNumber>
  <NotificationType>Alert</NotificationType>
  <Product>
    <ProductName>Organic Lettuce</ProductName>
    <BatchNumbers>
      <Batch>BATCH-2025-001</Batch>
    </BatchNumbers>
  </Product>
  <Hazard>
    <HazardCategory>Pathogenic microorganisms</HazardCategory>
    <HazardSubstance>Salmonella</HazardSubstance>
  </Hazard>
  <Origin>
    <Country>US</Country>
    <Producer>ABC Farms</Producer>
  </Origin>
  <Distribution>
    <DistributionCountries>
      <Country>DE</Country>
      <Country>FR</Country>
      <Country>NL</Country>
    </DistributionCountries>
  </Distribution>
  <Actions>
    <Action>Product recall</Action>
    <Action>Public warning</Action>
  </Actions>
</RASFF_Notification>
```

## 5. Warehouse Management System (WMS) Integration

### 5.1 Inventory Tracking

**WMS Event Synchronization:**
- Goods receipt → Supply chain event
- Stock transfer → Location update
- Picking → Distribution event
- Cycle count → Inventory verification

### 5.2 RFID Integration

**RFID Reader Integration:**
```java
import com.impinj.octane.*;

public class RFIDTraceabilityIntegration {
    private ImpinjReader reader;
    private TraceabilityAPI traceabilityAPI;

    public void startReading() throws Exception {
        reader = new ImpinjReader();
        reader.connect("192.168.1.100");

        Settings settings = reader.queryDefaultSettings();
        settings.getReport().setIncludeAntennaPortNumber(true);
        settings.getReport().setIncludeFirstSeenTime(true);

        reader.setTagReportListener((reader, report) -> {
            for (Tag tag : report.getTags()) {
                processTag(tag);
            }
        });

        reader.applySettings(settings);
        reader.start();
    }

    private void processTag(Tag tag) {
        String epc = tag.getEpc().toString();
        String batchNumber = extractBatchFromEPC(epc);

        // Submit event to traceability system
        traceabilityAPI.submitEvent(new SupplyChainEvent()
            .setBatchNumber(batchNumber)
            .setEventType("RFID_SCAN")
            .setLocation("WAREHOUSE-GATE-A")
            .setTimestamp(tag.getFirstSeenTime())
        );
    }
}
```

## 6. Transportation Management System (TMS) Integration

### 6.1 Shipment Tracking

**TMS Integration Points:**
- Shipment creation → Transport event
- Route optimization → Planned journey
- Delivery confirmation → Custody transfer
- Temperature monitoring → Cold chain verification

### 6.2 Telematics Integration

**Fleet Management Integration:**
```python
class TelematicsIntegration:
    async def process_vehicle_telemetry(self, vehicle_data):
        """Process real-time vehicle telemetry data"""

        # Extract relevant information
        vehicle_id = vehicle_data['vehicleId']
        temperature = vehicle_data['trailerTemperature']
        location = vehicle_data['gpsCoordinates']

        # Find associated shipments
        shipments = await self.get_active_shipments(vehicle_id)

        for shipment in shipments:
            # Submit location update
            await self.traceability_api.submit_event({
                "eventType": "LOCATION_UPDATE",
                "batchNumbers": shipment['batchNumbers'],
                "location": location,
                "temperature": temperature,
                "timestamp": vehicle_data['timestamp']
            })

            # Check for temperature violations
            if not self.is_temperature_acceptable(temperature, shipment['requirements']):
                await self.trigger_alert(shipment, temperature)
```

## 7. Quality Management System (QMS) Integration

### 7.1 Test Result Integration

**Laboratory Information Management System (LIMS):**
```json
{
  "testResult": {
    "testId": "TEST-2025-12345",
    "batchNumber": "BATCH-2025-001",
    "testType": "Microbiological Analysis",
    "testDate": "2025-12-20T14:00:00Z",
    "laboratory": {
      "labId": "LAB-001",
      "name": "Food Safety Laboratory",
      "accreditation": "ISO 17025"
    },
    "parameters": [
      {
        "parameter": "Salmonella",
        "result": "Not Detected",
        "unit": "in 25g",
        "method": "ISO 6579",
        "status": "pass"
      },
      {
        "parameter": "E. coli",
        "result": "< 10",
        "unit": "CFU/g",
        "method": "ISO 16649",
        "status": "pass"
      }
    ],
    "overallStatus": "APPROVED",
    "approvedBy": "Quality Manager",
    "approvalDate": "2025-12-20T16:00:00Z"
  }
}
```

## 8. Consumer-Facing Integration

### 8.1 Mobile App SDK

**React Native Integration:**
```typescript
import { WIATraceability } from '@wia/traceability-sdk';

const TraceabilityScanner: React.FC = () => {
  const [productData, setProductData] = useState(null);

  const handleScan = async (qrCode: string) => {
    try {
      const sdk = new WIATraceability({
        apiKey: 'your-api-key',
        environment: 'production'
      });

      const data = await sdk.getProductInfo(qrCode);
      setProductData(data);
    } catch (error) {
      console.error('Failed to fetch traceability data:', error);
    }
  };

  return (
    <QRCodeScanner
      onScan={handleScan}
      renderResult={() => (
        <ProductTraceabilityView data={productData} />
      )}
    />
  );
};
```

### 8.2 E-commerce Platform Integration

**Shopify App Integration:**
```javascript
// Shopify app integration for product traceability
app.post('/webhooks/products/create', async (req, res) => {
  const product = req.body;

  // Create traceability record for new product
  await traceabilityAPI.createProduct({
    gtin: product.variants[0].barcode,
    productName: product.title,
    category: mapShopifyToWIA(product.product_type),
    metafields: {
      batchNumber: product.metafields.batch_number,
      origin: product.metafields.origin
    }
  });

  res.status(200).send('OK');
});
```

## 9. Integration Testing

### 9.1 Test Environment

**Sandbox Endpoints:**
- API: `https://api.sandbox.wia.org/v1`
- Blockchain: Sepolia Testnet
- MQTT: `mqtts://mqtt.sandbox.wia.org:8883`

### 9.2 Integration Test Suite

**Sample Test Cases:**
```javascript
describe('ERP Integration Tests', () => {
  test('Batch creation syncs to traceability system', async () => {
    const batch = await erp.createProductionOrder({
      material: 'LETTUCE-001',
      quantity: 10000,
      productionDate: '2025-12-20'
    });

    // Wait for async sync
    await delay(2000);

    const traceData = await traceabilityAPI.getBatch(batch.batchNumber);
    expect(traceData).toBeDefined();
    expect(traceData.productGtin).toBe('01234567890128');
  });
});
```

---

## Implementation Checklist

- [ ] ERP system integration completed
- [ ] Blockchain smart contracts deployed
- [ ] IoT sensors configured and transmitting
- [ ] Regulatory reporting automation enabled
- [ ] WMS/TMS integration functional
- [ ] QMS test results synchronized
- [ ] Consumer-facing applications deployed
- [ ] Integration tests passing
- [ ] Monitoring and alerting configured
- [ ] Documentation updated

---

**Document Version:** 1.0
**Last Updated:** 2025-12-27
**Status:** Official Release
**弘益人間 - Benefit All Humanity through Seamless Integration**

© 2025 SmileStory Inc. / WIA (World Certification Industry Association)
