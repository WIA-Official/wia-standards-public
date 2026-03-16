# WIA Edible Algae Integration Standard
## Phase 4 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [SCADA Integration](#scada-integration)
4. [ERP Integration](#erp-integration)
5. [Quality Management System](#quality-management-system)
6. [Supply Chain Integration](#supply-chain-integration)
7. [Blockchain Traceability](#blockchain-traceability)
8. [Third-Party Platforms](#third-party-platforms)
9. [Data Exchange Formats](#data-exchange-formats)
10. [Compliance & Certifications](#compliance--certifications)

---

## Overview

### 1.1 Purpose

The WIA Edible Algae Integration Standard defines how algae cultivation systems integrate with SCADA, ERP, quality management, supply chain, blockchain, and external platforms to enable end-to-end traceability and operational excellence.

### 1.2 Integration Architecture

```
┌────────────────────────────────────────────────────────────┐
│                    Cloud Platform Layer                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │ Analytics│  │   API    │  │Blockchain│  │ Dashboards│  │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘  │
└────────────────────────────────────────────────────────────┘
                           ▲
                           │ HTTPS/MQTT
                           ▼
┌────────────────────────────────────────────────────────────┐
│                  Integration Middleware                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │  SCADA   │  │   ERP    │  │   QMS    │  │  Supply  │  │
│  │  Bridge  │  │  Adapter │  │Connector │  │  Chain   │  │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘  │
└────────────────────────────────────────────────────────────┘
                           ▲
                           │ Modbus/OPC UA/REST
                           ▼
┌────────────────────────────────────────────────────────────┐
│               Facility Automation Layer                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │   PBR    │  │ Sensors  │  │   PLC    │  │  Pumps/  │  │
│  │Controllers│ │ Network  │  │ Control  │  │  Valves  │  │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘  │
└────────────────────────────────────────────────────────────┘
```

### 1.3 Integration Principles

1. **Interoperability**: Standards-based integration (OPC UA, Modbus, REST)
2. **Real-time**: Sub-second data exchange for critical parameters
3. **Traceability**: Blockchain-based immutable record
4. **Scalability**: Support 1 to 1000+ reactors
5. **Security**: End-to-end encryption, role-based access
6. **Resilience**: Graceful degradation, offline operation

---

## System Architecture

### 2.1 Integration Patterns

| Pattern | Use Case | Protocol |
|---------|----------|----------|
| Request-Response | API queries | HTTP/REST |
| Publish-Subscribe | Real-time sensor data | MQTT |
| Batch Transfer | Daily production reports | SFTP/S3 |
| Event-Driven | Alerts, harvest triggers | WebHooks |
| Stream Processing | Time-series analytics | Apache Kafka |

### 2.2 Data Flow Diagram

```
[Photobioreactor] --MQTT--> [Edge Gateway] --HTTPS--> [Cloud]
       │                           │
       └── Modbus TCP ──> [PLC] ───┘

[Cloud] --REST--> [ERP System (SAP/Oracle)]
[Cloud] --SOAP--> [QMS (TraceGains/FoodLogiQ)]
[Cloud] --GraphQL--> [Supply Chain Platform]
[Cloud] --Web3--> [Blockchain (Ethereum/Hyperledger)]
```

---

## SCADA Integration

### 3.1 OPC UA Server

**Endpoint**: `opc.tcp://scada.oceanfarms.com:4840`

**Node Structure**:
```
Root
├── Facilities
│   ├── FAC-OCEAN-01
│   │   ├── Reactors
│   │   │   ├── PBR-001
│   │   │   │   ├── Status (String)
│   │   │   │   ├── Temperature (Double)
│   │   │   │   ├── pH (Double)
│   │   │   │   ├── BiomassDensity (Double)
│   │   │   │   └── AlarmStatus (Boolean)
│   │   │   └── PBR-002
│   │   └── Systems
│   │       ├── WaterSupply
│   │       ├── CO2Injection
│   │       └── LightingControl
```

**Read Variable**:
```python
from opcua import Client

client = Client("opc.tcp://scada.oceanfarms.com:4840")
client.connect()

node = client.get_node("ns=2;s=FAC-OCEAN-01.PBR-001.Temperature")
temperature = node.get_value()
print(f"Temperature: {temperature}°C")

client.disconnect()
```

**Write Variable**:
```python
setpoint_node = client.get_node("ns=2;s=FAC-OCEAN-01.PBR-001.TempSetpoint")
setpoint_node.set_value(26.0)
```

### 3.2 Modbus TCP Integration

**Holding Registers Map** (Reactor PBR-001):

| Register | Address | Type | Description | Unit |
|----------|---------|------|-------------|------|
| Temperature | 40001 | INT16 | Current temp × 10 | 0.1°C |
| pH | 40002 | INT16 | pH × 100 | 0.01 pH |
| BiomassOD | 40003 | INT16 | OD680 × 1000 | 0.001 OD |
| TempSetpoint | 40011 | INT16 | Setpoint × 10 | 0.1°C |
| LightIntensity | 40012 | UINT16 | μmol/m²/s | 1 |

**Python Example**:
```python
from pymodbus.client import ModbusTcpClient

client = ModbusTcpClient('192.168.1.100', port=502)
client.connect()

# Read temperature (address 40001 -> 0-indexed = 0)
result = client.read_holding_registers(0, 1, unit=1)
temperature = result.registers[0] / 10.0
print(f"Temperature: {temperature}°C")

# Write setpoint (address 40011 -> 0-indexed = 10)
client.write_register(10, int(26.0 * 10), unit=1)
client.close()
```

---

## ERP Integration

### 4.1 SAP Integration

**IDoc Message** (Production Batch):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<LOIPRO01>
  <IDOC BEGIN="1">
    <EDI_DC40>
      <TABNAM>EDI_DC40</TABNAM>
      <MANDT>100</MANDT>
      <DOCNUM>0000000001</DOCNUM>
      <DOCREL>750</DOCREL>
      <STATUS>30</STATUS>
      <MESTYP>LOIPRO</MESTYP>
      <IDOCTYP>LOIPRO01</IDOCTYP>
    </EDI_DC40>
    <E1AFKOL>
      <AUFNR>1000001</AUFNR>  <!-- Batch ID -->
      <MATNR>SPIRULINA-POWDER</MATNR>  <!-- Material -->
      <GAMNG>450</GAMNG>  <!-- Quantity (kg) -->
      <GMEIN>KG</GMEIN>  <!-- Unit -->
      <BUDAT>20250115</BUDAT>  <!-- Harvest Date -->
    </E1AFKOL>
  </IDOC>
</LOIPRO01>
```

**REST API Alternative**:

```http
POST /sap/opu/odata/sap/API_PRODUCTION_ORDER_2_SRV/A_ProductionOrder
Authorization: Basic base64(username:password)
Content-Type: application/json

{
  "ManufacturingOrder": "1000001",
  "Material": "SPIRULINA-POWDER",
  "ProductionPlant": "OCEAN-FARMS-01",
  "TotalQuantity": "450",
  "ProductionUnit": "KG",
  "PlannedStartDate": "2025-01-15",
  "ActualReleaseDate": "2025-01-15"
}
```

### 4.2 Oracle ERP Cloud

```http
POST /fscmRestApi/resources/11.13.18.05/productionOrders
Authorization: Bearer <access_token>
Content-Type: application/vnd.oracle.adf.resourceitem+json

{
  "OrganizationCode": "OCEAN-FARMS",
  "ItemNumber": "SPIRULINA-POWDER",
  "OrderQuantity": 450,
  "UOMCode": "KG",
  "StartDate": "2025-01-15T00:00:00Z",
  "CompletionDate": "2025-01-18T00:00:00Z",
  "StatusCode": "RELEASED"
}
```

---

## Quality Management System

### 5.1 TraceGains Integration

**Certificate of Analysis (CoA) Upload**:

```http
POST /api/v1/coa
Authorization: Bearer <tracegains_token>
Content-Type: application/json

{
  "supplier_id": "OCEAN-FARMS-01",
  "product_name": "Spirulina Powder",
  "lot_number": "ALGAE-2025-001",
  "production_date": "2025-01-18",
  "expiration_date": "2027-01-18",
  "test_results": [
    {
      "test_name": "Protein Content",
      "result": "65.2",
      "unit": "%",
      "specification": "≥60%",
      "status": "PASS"
    },
    {
      "test_name": "Lead",
      "result": "0.04",
      "unit": "ppm",
      "specification": "≤0.5 ppm",
      "status": "PASS"
    },
    {
      "test_name": "E. coli",
      "result": "ABSENT",
      "specification": "ABSENT",
      "status": "PASS"
    }
  ],
  "overall_status": "APPROVED",
  "approved_by": "QC Manager",
  "approval_date": "2025-01-19"
}
```

### 5.2 FoodLogiQ Integration

**Product Record**:

```http
POST /api/v3/products
Authorization: ApiKey <foodlogiq_key>
Content-Type: application/json

{
  "gtin": "00012345678905",
  "product_name": "Organic Spirulina Powder",
  "brand": "Ocean Algae Farms",
  "lot_code": "ALGAE-2025-001",
  "harvest_date": "2025-01-18",
  "certifications": [
    {"type": "ORGANIC", "certifier": "USDA"},
    {"type": "ISO_22000", "certifier": "SGS"}
  ],
  "nutritional_info": {
    "serving_size": "5g",
    "protein_g": 3.26,
    "vitamin_b12_ug": 12.5
  },
  "traceability": {
    "cultivation_id": "CULT-2025-ALG-001",
    "facility": "FAC-OCEAN-01",
    "blockchain_hash": "0x1234...abcd"
  }
}
```

---

## Supply Chain Integration

### 6.1 EDI 856 - Advance Ship Notice

```edi
ISA*00*          *00*          *ZZ*OCEANFARMS     *ZZ*DISTRIBUTOR    *250119*1430*U*00401*000000001*0*P*>~
GS*SH*OCEANFARMS*DISTRIBUTOR*20250119*1430*1*X*004010~
ST*856*0001~
BSN*00*SHIP001*20250119*1430~
HL*1**S~
TD5*2*FEDEX*M***GROUND~
REF*BM*ALGAE-2025-001~
N1*SF*Ocean Algae Farms*92*OCEANFARMS~
N1*ST*Healthy Foods Distributor*92*DISTRIBUTOR~
HL*2*1*O~
PRF*PO12345***20250115~
HL*3*2*I~
LIN**UP*00012345678905~
SN1**450*KG~
PID*F****Organic Spirulina Powder~
SE*15*0001~
GE*1*1~
IEA*1*000000001~
```

### 6.2 GS1 EPCIS Events

**Harvest Event**:

```xml
<epcis:EPCISDocument schemaVersion="2.0">
  <epcis:EPCISBody>
    <epcis:EventList>
      <ObjectEvent>
        <eventTime>2025-01-18T14:00:00Z</eventTime>
        <eventTimeZoneOffset>+00:00</eventTimeZoneOffset>
        <epcList>
          <epc>urn:epc:id:sgtin:0012345.067890.001</epc>
        </epcList>
        <action>ADD</action>
        <bizStep>urn:epcglobal:cbv:bizstep:commissioning</bizStep>
        <disposition>urn:epcglobal:cbv:disp:active</disposition>
        <readPoint>
          <id>urn:epc:id:sgln:0012345.00001.0</id>
        </readPoint>
        <bizLocation>
          <id>urn:epc:id:sgln:0012345.00001.0</id>
        </bizLocation>
        <extension>
          <quantityList>
            <quantityElement>
              <epcClass>urn:epc:class:lgtin:0012345.067890.Lot001</epcClass>
              <quantity>450</quantity>
              <uom>KGM</uom>
            </quantityElement>
          </quantityList>
          <ilmd>
            <cultivation_id>CULT-2025-ALG-001</cultivation_id>
            <harvest_date>2025-01-18</harvest_date>
            <protein_content>65.2</protein_content>
          </ilmd>
        </extension>
      </ObjectEvent>
    </epcis:EventList>
  </epcis:EPCISBody>
</epcis:EPCISDocument>
```

---

## Blockchain Traceability

### 7.1 Ethereum Smart Contract

**Solidity Contract**:

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract EdibleAlgaeTraceability {
    struct Batch {
        string cultivationId;
        string species;
        uint256 harvestDate;
        uint256 biomassKg;
        string facilityId;
        string certifications;
        bool isOrganic;
    }

    mapping(string => Batch) public batches;

    event BatchRegistered(
        string indexed batchId,
        string cultivationId,
        uint256 harvestDate
    );

    function registerBatch(
        string memory batchId,
        string memory cultivationId,
        string memory species,
        uint256 biomassKg,
        string memory facilityId,
        bool isOrganic
    ) public {
        batches[batchId] = Batch({
            cultivationId: cultivationId,
            species: species,
            harvestDate: block.timestamp,
            biomassKg: biomassKg,
            facilityId: facilityId,
            certifications: "USDA_ORGANIC,ISO_22000",
            isOrganic: isOrganic
        });

        emit BatchRegistered(batchId, cultivationId, block.timestamp);
    }

    function verifyBatch(string memory batchId)
        public
        view
        returns (Batch memory)
    {
        return batches[batchId];
    }
}
```

**Web3 Integration**:

```javascript
const Web3 = require('web3');
const web3 = new Web3('https://mainnet.infura.io/v3/YOUR_KEY');

const contractAddress = '0x1234...abcd';
const abi = [...]; // Contract ABI

const contract = new web3.eth.Contract(abi, contractAddress);

// Register batch
await contract.methods.registerBatch(
  'ALGAE-2025-001',
  'CULT-2025-ALG-001',
  'Spirulina platensis',
  450,
  'FAC-OCEAN-01',
  true
).send({ from: accountAddress });

// Verify batch
const batch = await contract.methods.verifyBatch('ALGAE-2025-001').call();
console.log('Batch:', batch);
```

### 7.2 Hyperledger Fabric

**Chaincode (Go)**:

```go
package main

import (
    "encoding/json"
    "github.com/hyperledger/fabric-contract-api-go/contractapi"
)

type AlgaeBatch struct {
    CultivationID  string  `json:"cultivation_id"`
    Species        string  `json:"species"`
    HarvestDate    string  `json:"harvest_date"`
    BiomassKg      float64 `json:"biomass_kg"`
    ProteinPercent float64 `json:"protein_percent"`
    Certifications []string `json:"certifications"`
}

type SmartContract struct {
    contractapi.Contract
}

func (s *SmartContract) RegisterBatch(ctx contractapi.TransactionContextInterface,
    batchID string, cultivationID string, species string, biomassKg float64) error {

    batch := AlgaeBatch{
        CultivationID:  cultivationID,
        Species:        species,
        BiomassKg:      biomassKg,
        Certifications: []string{"USDA_ORGANIC", "ISO_22000"},
    }

    batchJSON, err := json.Marshal(batch)
    if err != nil {
        return err
    }

    return ctx.GetStub().PutState(batchID, batchJSON)
}

func (s *SmartContract) QueryBatch(ctx contractapi.TransactionContextInterface,
    batchID string) (*AlgaeBatch, error) {

    batchJSON, err := ctx.GetStub().GetState(batchID)
    if err != nil {
        return nil, err
    }

    var batch AlgaeBatch
    err = json.Unmarshal(batchJSON, &batch)
    if err != nil {
        return nil, err
    }

    return &batch, nil
}
```

---

## Third-Party Platforms

### 8.1 Weather API Integration

**OpenWeather API**:

```http
GET https://api.openweathermap.org/data/2.5/weather?lat=21.3&lon=-157.8&appid=YOUR_KEY

Response:
{
  "coord": {"lon": -157.8, "lat": 21.3},
  "weather": [{"main": "Clear", "description": "clear sky"}],
  "main": {
    "temp": 298.15,
    "humidity": 65
  },
  "wind": {"speed": 5.5},
  "dt": 1642249845
}
```

**Integration Logic**:
- Ambient temperature affects PBR cooling requirements
- Rainfall affects open pond water levels
- Cloud cover affects natural light supplementation needs

### 8.2 Laboratory Testing Services

**Eurofins API** (Certificate of Analysis):

```http
POST /api/v1/test-results
Authorization: Bearer <eurofins_token>
Content-Type: application/json

{
  "client_id": "OCEAN-FARMS-01",
  "sample_id": "SAMP-2025-001",
  "product": "Spirulina Powder",
  "lot_number": "ALGAE-2025-001",
  "tests_requested": [
    "HEAVY_METALS_PANEL",
    "MICROBIAL_COUNT",
    "NUTRITIONAL_ANALYSIS"
  ]
}
```

---

## Data Exchange Formats

### 9.1 CSV Export (Daily Production Report)

```csv
Date,Facility,Reactor,Species,BiomassHarvested_kg,ProteinContent_%,Status
2025-01-18,FAC-OCEAN-01,PBR-001,Spirulina,22.5,65.2,COMPLETE
2025-01-18,FAC-OCEAN-01,PBR-002,Chlorella,18.3,58.7,COMPLETE
2025-01-18,FAC-OCEAN-01,PBR-003,Spirulina,21.8,64.9,COMPLETE
```

### 9.2 Parquet (Big Data Analytics)

```python
import pyarrow.parquet as pq
import pandas as pd

df = pd.DataFrame({
    'timestamp': [...],
    'reactor_id': [...],
    'biomass_density': [...],
    'temperature': [...],
    'ph': [...]
})

df.to_parquet('cultivation_timeseries_2025-01.parquet',
              engine='pyarrow',
              compression='snappy')
```

---

## Compliance & Certifications

### 10.1 FDA FSMA Compliance

**Hazard Analysis and Risk-Based Preventive Controls (HARPC)**:

```json
{
  "facility_id": "FAC-OCEAN-01",
  "fsma_registration": "12345678901",
  "preventive_controls": [
    {
      "hazard": "BIOLOGICAL_CONTAMINATION",
      "preventive_control": "UV_STERILIZATION",
      "monitoring": "Daily UV lamp intensity check",
      "corrective_action": "Replace UV lamps, re-sterilize system"
    },
    {
      "hazard": "HEAVY_METAL_CONTAMINATION",
      "preventive_control": "WATER_FILTRATION",
      "monitoring": "Quarterly water testing",
      "corrective_action": "Replace filters, test source water"
    }
  ]
}
```

### 10.2 ISO 22000 Integration

**HACCP Critical Control Point**:

```json
{
  "ccp": "DRYING_TEMPERATURE",
  "critical_limit": {
    "min": 50,
    "max": 60,
    "unit": "celsius"
  },
  "monitoring_procedure": "Continuous temperature logging",
  "corrective_action": "Reject batch if temp exceeds 65°C",
  "verification": "Monthly calibration of temperature sensors",
  "records": "Temperature logs retained 2 years"
}
```

---

**弘益人間 (Hongik Ingan)** · Benefit All Humanity

© 2025 WIA Standards - MIT License
