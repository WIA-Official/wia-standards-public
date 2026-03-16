# WIA-AGRI-014: Agricultural Supply Chain Standard
## Phase 4: WIA Integration Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines integration patterns between the Agricultural Supply Chain Standard and other WIA ecosystem standards, creating a comprehensive farm-to-fork traceability system.

---

## 2. WIA Standards Ecosystem Integration

### 2.1 Integration with WIA-BLOCKCHAIN

**Provenance Recording:**

```javascript
// Record shipment on WIA-BLOCKCHAIN
const WIABlockchain = require('@wia/blockchain');

async function recordShipmentProvenance(shipmentData) {
  const blockchain = new WIABlockchain({
    network: 'WIA-SUPPLY-CHAIN',
    nodeUrl: 'https://blockchain.wia.org'
  });

  const provenanceRecord = {
    type: 'AGRICULTURAL_SHIPMENT',
    productId: shipmentData.productInfo.productId,
    farmId: shipmentData.origin.farmId,
    timestamp: new Date().toISOString(),
    hash: generateHash(shipmentData),
    metadata: {
      coldChainVerified: true,
      qualityChecked: true,
      certifications: shipmentData.productInfo.certifications
    }
  };

  const tx = await blockchain.submitTransaction(provenanceRecord);
  return {
    transactionId: tx.id,
    blockNumber: tx.blockNumber,
    blockHash: tx.blockHash
  };
}
```

### 2.2 Integration with WIA-VC (Verifiable Credentials)

**Product Provenance Credential:**

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wia.org/credentials/v1"
  ],
  "type": ["VerifiableCredential", "AgriculturalProvenanceCredential"],
  "issuer": "did:wia:agricultural-authority",
  "issuanceDate": "2025-01-01T08:00:00Z",
  "credentialSubject": {
    "id": "did:wia:product:PROD-TOMATO-001",
    "productName": "Organic Tomatoes",
    "farmName": "Green Valley Farm",
    "farmDID": "did:wia:farm:FARM-KR-001",
    "harvestDate": "2025-01-01",
    "certifications": [
      {
        "type": "OrganicCertification",
        "certifier": "Korea Organic Certification",
        "certificateNumber": "ORG-2025-001",
        "validUntil": "2025-12-31"
      }
    ],
    "traceability": {
      "farmToRetail": true,
      "coldChainMonitored": true,
      "blockchainVerified": true,
      "blockchainTx": "TX-0x1234567890abcdef"
    },
    "qualityAssurance": {
      "pesticideTest": "NEGATIVE",
      "microbiologicalTest": "PASSED",
      "grade": "GRADE_A"
    }
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-01-01T08:00:00Z",
    "verificationMethod": "did:wia:agricultural-authority#key-1",
    "proofPurpose": "assertionMethod",
    "proofValue": "z3MvGcVxzRbVo..."
  }
}
```

### 2.3 Integration with WIA-IoT Standard

**IoT Device Registration:**

```javascript
const WIAIoT = require('@wia/iot');

async function registerColdChainSensor(deviceInfo) {
  const iot = new WIAIoT({
    endpoint: 'https://iot.wia.org'
  });

  const device = await iot.registerDevice({
    deviceType: 'TEMPERATURE_HUMIDITY_SENSOR',
    deviceId: 'SENSOR-TEMP-001',
    manufacturer: 'IoT Sensors Inc.',
    model: 'TS-2000',
    capabilities: ['temperature', 'humidity', 'gps'],
    certifications: ['WIA-IoT-L2', 'IP67'],
    communicationProtocol: 'MQTT',
    securityLevel: 'TLS_1_3',
    assignedTo: {
      type: 'SHIPMENT',
      id: 'SHIP-2025-001'
    }
  });

  return device;
}
```

### 2.4 Integration with WIA-CARBON Standard

**Carbon Footprint Tracking:**

```javascript
const WIACarbon = require('@wia/carbon');

async function calculateTransportationCarbon(shipmentData) {
  const carbon = new WIACarbon();

  const footprint = await carbon.calculate({
    activity: 'FREIGHT_TRANSPORT',
    transportMode: 'REFRIGERATED_TRUCK',
    distance: shipmentData.route.totalDistance, // km
    weight: shipmentData.productInfo.quantity,  // kg
    refrigerationRequired: true,
    temperatureRange: '2-8C',
    emissionFactor: 0.85 // kg CO2e per ton-km
  });

  return {
    totalEmissions: footprint.co2e, // kg CO2e
    emissionsPerKg: footprint.co2e / shipmentData.productInfo.quantity,
    offset: await carbon.suggestOffset(footprint.co2e)
  };
}
```

---

## 3. External System Integration

### 3.1 ERP System Integration (SAP)

**SAP S/4HANA Integration:**

```javascript
const SAPClient = require('node-rfc');

async function syncToSAP(shipmentData) {
  const client = new SAPClient({
    dest: 'SAP_EWM',
    user: 'WIA_USER',
    passwd: 'password',
    ashost: 'sap.company.com',
    sysnr: '00'
  });

  await client.connect();

  // Create Inbound Delivery
  const result = await client.call('BAPI_INBOUND_DELIVERY_CREATE_SLS', {
    HEADER_DATA: {
      DELIV_DATE: shipmentData.timestamp.split('T')[0],
      DOC_DATE: new Date().toISOString().split('T')[0],
      DELIV_TYPE: 'ZLF'
    },
    HEADER_PARTNER: [{
      PARTNER_ROLE: 'SP',
      PARTNER_NUMB: shipmentData.origin.farmId
    }],
    ITEM_DATA: [{
      MATERIAL: shipmentData.productInfo.productId,
      DELIV_QTY: shipmentData.productInfo.quantity,
      DELIV_QTY_UNIT: shipmentData.productInfo.unit,
      BATCH: shipmentData.productInfo.batchNumber
    }]
  });

  await client.close();
  return result.DELIVERY;
}
```

### 3.2 Oracle WMS Integration

**Warehouse Receipt:**

```javascript
async function createWarehouseReceipt(shipmentData, warehouseInfo) {
  const response = await fetch('https://oracle-wms.company.com/api/v1/receipts', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': 'Bearer ' + oracleToken
    },
    body: JSON.stringify({
      receiptNumber: shipmentData.shipmentId,
      shipmentNumber: shipmentData.shipmentId,
      expectedArrivalDate: shipmentData.route.waypoints[shipmentData.route.waypoints.length-1].timestamp,
      supplier: shipmentData.origin.farmId,
      items: [{
        itemId: shipmentData.productInfo.productId,
        quantity: shipmentData.productInfo.quantity,
        uom: shipmentData.productInfo.unit,
        lotNumber: shipmentData.productInfo.batchNumber,
        expirationDate: calculateExpirationDate(shipmentData),
        storageTemperature: warehouseInfo.temperatureZone
      }],
      customFields: {
        coldChainVerified: true,
        blockchainTx: shipmentData.blockchain.transactionId,
        provenanceVC: shipmentData.provenanceCredential
      }
    })
  });

  return await response.json();
}
```

### 3.3 GS1 EPCIS Integration

**EPCIS Event Publishing:**

```javascript
const EPCIS = require('epcis2.js');

async function publishEPCISEvent(shipmentData, eventType) {
  const event = new EPCIS.ObjectEvent({
    eventTime: new Date().toISOString(),
    eventTimeZoneOffset: '+09:00',
    action: 'OBSERVE',
    bizStep: getBizStep(eventType), // shipping, receiving, etc.
    disposition: 'in_transit',
    readPoint: {
      id: `urn:epc:id:sgln:${shipmentData.currentLocation.gln}`
    },
    bizLocation: {
      id: `urn:epc:id:sgln:${shipmentData.origin.gln}`
    },
    epcList: [
      `urn:epc:id:sgtin:${shipmentData.productInfo.gtin}.${shipmentData.productInfo.batchNumber}`
    ],
    extensions: {
      'wia:temperature': shipmentData.coldChain.temperature,
      'wia:humidity': shipmentData.coldChain.humidity,
      'wia:blockchainTx': shipmentData.blockchain.transactionId
    }
  });

  await EPCIS.captureEvent(event, {
    endpoint: 'https://epcis.company.com/capture'
  });
}
```

### 3.4 Retailer POS Integration

**Walmart Retail Link:**

```javascript
async function sendToWalmartRetailLink(shipmentData) {
  const response = await fetch('https://retaillink.walmart.com/api/v1/asn', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'WM-CONSUMER.ID': 'your-consumer-id',
      'WM-SEC.ACCESS_TOKEN': accessToken
    },
    body: JSON.stringify({
      asn: {
        asnNumber: shipmentData.shipmentId,
        shipDate: shipmentData.timestamp,
        expectedDeliveryDate: shipmentData.estimatedArrival,
        vendor: {
          vendorNumber: shipmentData.origin.farmId,
          vendorName: shipmentData.origin.farmName
        },
        shipTo: {
          storeNumber: shipmentData.destination.facilityId,
          dcNumber: shipmentData.destination.dcNumber
        },
        items: [{
          itemNumber: shipmentData.productInfo.upc,
          description: shipmentData.productInfo.productName,
          quantity: shipmentData.productInfo.quantity,
          uom: 'EA',
          lotNumber: shipmentData.productInfo.batchNumber,
          expirationDate: calculateExpirationDate(shipmentData),
          wiaProvenance: {
            blockchainVerified: true,
            coldChainMonitored: true,
            organicCertified: shipmentData.productInfo.certifications.includes('ORGANIC')
          }
        }]
      }
    })
  });

  return await response.json();
}
```

---

## 4. Government & Regulatory Integration

### 4.1 Korea Customs Integration

**Import/Export Declaration:**

```javascript
async function submitCustomsDeclaration(shipmentData) {
  const response = await fetch('https://unipass.customs.go.kr/api/v1/declaration', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': 'Bearer ' + customsToken
    },
    body: JSON.stringify({
      declarationType: 'IMPORT',
      shipmentId: shipmentData.shipmentId,
      hsCode: shipmentData.productInfo.hsCode,
      origin: {
        country: shipmentData.origin.country,
        farm: shipmentData.origin.farmName,
        coordinates: shipmentData.origin.location
      },
      productDetails: {
        name: shipmentData.productInfo.productName,
        quantity: shipmentData.productInfo.quantity,
        value: shipmentData.productInfo.value,
        currency: 'KRW'
      },
      phytosanitary: {
        certificateNumber: shipmentData.phytosanitaryCert,
        issueDate: shipmentData.certIssueDate,
        verified: true
      },
      blockchain: {
        provenanceVerified: true,
        transactionId: shipmentData.blockchain.transactionId
      }
    })
  });

  return await response.json();
}
```

### 4.2 FDA FSMA Compliance (US)

**Food Safety Traceability:**

```javascript
async function submitFSMATraceability(shipmentData) {
  const response = await fetch('https://fda.gov/api/fsma/traceability', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'FDA-API-Key': fdaApiKey
    },
    body: JSON.stringify({
      traceabilityLotCode: shipmentData.productInfo.batchNumber,
      productDescription: shipmentData.productInfo.productName,
      quantity: shipmentData.productInfo.quantity,
      unitOfMeasure: shipmentData.productInfo.unit,
      originInformation: {
        harvestLocation: shipmentData.origin.farmName,
        harvestDate: shipmentData.productInfo.harvestDate,
        growerName: shipmentData.origin.operator,
        growerContact: shipmentData.origin.contactPhone
      },
      cooling: {
        initialCoolingDate: shipmentData.coldChain.startDate,
        coolingMethod: 'REFRIGERATED_TRANSPORT',
        temperatureLog: shipmentData.coldChain.history
      },
      shipping: {
        shipperName: shipmentData.logistics.carrier,
        shipDate: shipmentData.timestamp,
        receiverName: shipmentData.destination.facilityName,
        receiverLocation: shipmentData.destination.location
      },
      digitalTraceability: {
        blockchainVerified: true,
        blockchainNetwork: 'WIA-SUPPLY-CHAIN',
        transactionHash: shipmentData.blockchain.transactionId
      }
    })
  });

  return await response.json();
}
```

---

## 5. Consumer-Facing Integration

### 5.1 QR Code for Consumer Traceability

**Generate Consumer QR Code:**

```javascript
const QRCode = require('qrcode');

async function generateConsumerQRCode(productData) {
  const consumerData = {
    productName: productData.productInfo.productName,
    farmName: productData.origin.farmName,
    harvestDate: productData.productInfo.harvestDate,
    certifications: productData.productInfo.certifications,
    verifyUrl: `https://verify.wia.org/product/${productData.productInfo.productId}`,
    blockchain: {
      verified: true,
      network: 'WIA-SUPPLY-CHAIN',
      txId: productData.blockchain.transactionId
    }
  };

  const qrCode = await QRCode.toDataURL(JSON.stringify(consumerData));
  return qrCode; // Base64 image
}
```

### 5.2 Mobile App Integration

**Consumer Verification App:**

```javascript
// Mobile app can scan QR and verify provenance
async function verifyProductProvenance(scannedData) {
  const response = await fetch(`https://api.wia.org/v1/provenance/verify/${scannedData.productId}`, {
    method: 'GET',
    headers: {
      'Content-Type': 'application/json'
    }
  });

  const verification = await response.json();

  return {
    verified: verification.verified,
    farmToForkJourney: verification.traceability.map(event => ({
      stage: event.stage,
      location: event.location,
      timestamp: event.timestamp,
      operator: event.operator
    })),
    coldChainIntegrity: verification.coldChain.violations === 0,
    certifications: verification.certifications,
    blockchainProof: verification.blockchainHash
  };
}
```

---

## 6. Analytics & Business Intelligence Integration

### 6.1 Data Warehouse Integration

**Export to Analytics Platform:**

```javascript
async function exportToDataWarehouse(shipmentData) {
  const BigQuery = require('@google-cloud/bigquery');
  const bigquery = new BigQuery();

  const dataset = bigquery.dataset('wia_supply_chain');
  const table = dataset.table('shipments');

  await table.insert({
    shipment_id: shipmentData.shipmentId,
    product_id: shipmentData.productInfo.productId,
    origin_farm: shipmentData.origin.farmId,
    destination: shipmentData.destination.facilityId,
    ship_date: shipmentData.timestamp,
    delivery_date: shipmentData.deliveryTimestamp,
    duration_minutes: calculateDuration(shipmentData),
    cold_chain_violations: shipmentData.coldChain.violations.length,
    avg_temperature: shipmentData.coldChain.avgTemperature,
    quality_grade: shipmentData.quality.grade,
    blockchain_verified: shipmentData.blockchain.verified,
    carbon_footprint: shipmentData.carbon.totalEmissions
  });
}
```

---

## 7. WIA Certification Requirements

### 7.1 Compliance Checklist

**For WIA-AGRI-014 Certification:**

- ✅ Implement all Phase 1 data formats
- ✅ Support REST API endpoints (Phase 2)
- ✅ IoT sensor integration (MQTT/CoAP)
- ✅ Blockchain provenance recording
- ✅ Cold chain monitoring with alerts
- ✅ Quality control integration
- ✅ Consumer-facing QR code verification
- ✅ Integration with at least 2 external systems
- ✅ WIA-VC credential issuance
- ✅ Security: TLS 1.3, message signing

### 7.2 Certification Levels

| Level | Requirements | Use Case |
|-------|--------------|----------|
| **Bronze** | Phase 1-2, Basic IoT | Small farms, local distribution |
| **Silver** | Phase 1-3, Full IoT, Blockchain | Regional distributors |
| **Gold** | All Phases, Multi-system integration | National/International supply chains |

---

**弘益人間 (Benefit All Humanity)**
*WIA - World Certification Industry Association*
*© 2025 MIT License*
