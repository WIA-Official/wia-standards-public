# WIA-FOOD_SAFETY Specification - PHASE 1: Core Architecture & Standards

**Version:** 1.0.0
**Last Updated:** 2026-01-11

## Executive Summary

WIA-FOOD_SAFETY is a blockchain-based food traceability and safety management system implementing HACCP (Hazard Analysis Critical Control Points), ISO 22000:2018, FSSC 22000, and BRC Global Standards. The system provides end-to-end farm-to-table tracking with immutable audit trails, real-time temperature monitoring, pathogen detection, and instant recall capabilities.

**Problem Statement**: Traditional food safety systems require 7+ days for traceability, leading to widespread contamination during outbreaks. In 2018, the E. coli outbreak in romaine lettuce took weeks to trace, affecting 36 states and causing 5 deaths. Current systems lack real-time visibility and immutable records.

**Solution**: WIA-FOOD_SAFETY reduces traceability time to < 2.2 seconds using blockchain technology, IoT sensors, and automated HACCP compliance monitoring.

## Detailed Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    User Interface Layer                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ Web Dashboard│  │ Mobile App   │  │ QR Scanner   │     │
│  │ (React)      │  │ (React Native)│  │ (Camera API) │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
└────────────────────────┬────────────────────────────────────┘
                         │ HTTPS/REST/GraphQL
┌────────────────────────▼────────────────────────────────────┐
│                    Application Layer                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ Traceability │  │ HACCP Engine │  │ Recall Mgmt  │     │
│  │ Service      │  │ (CCP Monitor)│  │ System       │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ Analytics    │  │ Alert System │  │ Compliance   │     │
│  │ Engine       │  │ (Real-time)  │  │ Validator    │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
└────────────────────────┬────────────────────────────────────┘
                         │
    ┌────────────────────┼────────────────────┐
    │                    │                    │
┌───▼──────────────┐ ┌──▼──────────────┐ ┌──▼──────────────┐
│ Blockchain Layer │ │ IoT Data Layer  │ │ Lab Data Layer  │
│                  │ │                 │ │                 │
│ • Ethereum       │ │ • LoRaWAN       │ │ • Pathogen Test │
│ • Hyperledger    │ │ • Temp Sensors  │ │ • Allergen Test │
│   Fabric         │ │ • GPS Trackers  │ │ • Chemical Test │
│ • Smart Contract │ │ • Humidity      │ │ • Nutritional   │
│ • IPFS (Docs)    │ │ • Light Sensors │ │   Analysis      │
└──────────────────┘ └─────────────────┘ └─────────────────┘
```

### Blockchain Architecture

#### Ethereum Implementation
- **Network**: Ethereum mainnet + Layer-2 (Polygon) for cost efficiency
- **Smart Contracts**: Solidity 0.8+ for food tracking, batch management, recall execution
- **Storage**: On-chain (hashes), off-chain (IPFS) for documents/photos
- **Consensus**: Proof-of-Stake (post-Merge)
- **Transaction Cost**: ~$0.01-0.05 per transaction (Layer-2)

#### Hyperledger Fabric Implementation
- **Network**: Private permissioned network for enterprise deployments
- **Channels**: Separate channels for suppliers, manufacturers, distributors, retailers
- **Chaincode**: Go-based smart contracts for HACCP compliance
- **Consensus**: Raft ordering service (crash fault tolerant)
- **Privacy**: Private data collections for sensitive supplier information

#### Smart Contract: FoodTraceability.sol
```solidity
contract FoodTraceability {
    struct FoodProduct {
        bytes32 batchId;
        string productName;
        uint256 harvestDate;
        address farmer;
        string origin;
        bytes32[] ccpRecords;  // HACCP CCP records
        bool isRecalled;
    }

    mapping(bytes32 => FoodProduct) public products;
    mapping(bytes32 => TemperatureLog[]) public tempLogs;

    event ProductRegistered(bytes32 batchId, address farmer);
    event CCPRecorded(bytes32 batchId, string ccpType, bool passed);
    event RecallIssued(bytes32 batchId, string reason);
}
```

## HACCP 7 Principles Implementation

### Principle 1: Conduct Hazard Analysis
**Implementation**:
- Automated hazard database (biological, chemical, physical)
- AI-powered risk assessment based on product type, origin, season
- Historical contamination pattern analysis
- Allergen identification and tracking

**Example Hazards**:
- **Biological**: E. coli O157:H7, Salmonella, Listeria monocytogenes
- **Chemical**: Pesticide residues, heavy metals (lead, mercury)
- **Physical**: Glass fragments, metal shavings, plastic pieces

### Principle 2: Determine Critical Control Points (CCPs)
**CCPs Monitored**:
1. **Receiving Temperature**: Raw materials must arrive at ≤ 4°C (40°F)
2. **Cooking Temperature**: Poultry minimum 74°C (165°F) for 15 seconds
3. **Cold Storage**: ≤ 4°C (40°F) for refrigerated, ≤ -18°C (0°F) for frozen
4. **Hot Holding**: ≥ 60°C (140°F) for ready-to-eat foods
5. **pH Monitoring**: pH ≤ 4.6 for acidified foods
6. **Water Activity**: aw ≤ 0.85 to prevent microbial growth

### Principle 3: Establish Critical Limits
**Automated Monitoring**:
- IoT sensors record every 60 seconds
- Blockchain logs every reading with timestamp
- Alert triggered if out-of-spec for > 4 minutes (FDA requirement)

### Principle 4: Establish Monitoring Procedures
**System Features**:
- Real-time dashboard showing all CCPs
- Automated sensor readings (no manual logging)
- Photo documentation via mobile app
- GPS verification of transport routes

### Principle 5: Establish Corrective Actions
**Automated Workflows**:
```
IF temperature > 4°C for > 4 minutes:
  1. Alert supervisor via SMS/push notification
  2. Log incident on blockchain (immutable)
  3. Quarantine affected batch in system
  4. Initiate product evaluation protocol
  5. Document corrective action taken
```

### Principle 6: Establish Verification Procedures
**Blockchain Verification**:
- Smart contract validates all CCP records
- Third-party auditors can verify on blockchain
- Cryptographic proof of data integrity
- Historical trend analysis

### Principle 7: Establish Record-Keeping
**Immutable Records**:
- All records stored on blockchain (tamper-proof)
- Retention: 2 years (FDA requirement) to forever (blockchain)
- Audit trail with timestamp + digital signature
- Export to PDF for regulatory submission

## ISO 22000:2018 Compliance

### Clause 4: Context of the Organization
- Documented food safety policy
- Stakeholder mapping (farmers, processors, distributors, retailers, consumers)
- Risk assessment for internal/external factors

### Clause 5: Leadership
- Food safety team with defined roles
- Management review dashboard
- KPI tracking (CCP compliance rate, traceability time, recall effectiveness)

### Clause 6: Planning
- Hazard analysis and risk assessment
- Food safety objectives (99.99% CCP compliance target)
- Change management process

### Clause 7: Support
- Training module for system users
- Calibration schedule for IoT sensors (±0.1°C accuracy requirement)
- Document management via IPFS

### Clause 8: Operation
- Operational prerequisite programs (PRPs)
- HACCP plan execution
- Traceability implementation (one-up, one-back + one-step)

### Clause 9: Performance Evaluation
- Real-time KPI dashboard
- Monthly management review
- Internal audit checklist automation

### Clause 10: Improvement
- Corrective action tracking
- Root cause analysis (5 Whys, Fishbone)
- Continuous improvement metrics

## Additional Standards Compliance

### FSSC 22000 (Food Safety System Certification)
- ISO 22000:2018 + ISO/TS 22002-1 (Prerequisite Programs)
- Additional requirements: Food defense, food fraud mitigation
- Blockchain provides transparent supply chain for fraud prevention

### BRC Global Standards (Issue 9)
- Senior management commitment
- HACCP-based food safety plan
- Product traceability and recall procedures
- Supplier approval and performance monitoring

### FDA FSMA (Food Safety Modernization Act)
- **Preventive Controls**: Automated hazard analysis and CCP monitoring
- **Foreign Supplier Verification**: Blockchain-verified supplier credentials
- **Produce Safety Rule**: Temperature monitoring for fresh produce
- **Traceability Records**: < 2.2 seconds vs. 7 days traditional

## Cold Chain Monitoring

### IoT Sensor Network
**Hardware**:
- Temperature sensors: ±0.1°C accuracy (Pt1000 RTD)
- Humidity sensors: ±2% RH accuracy
- GPS trackers: 5-meter accuracy
- Battery life: 5 years (LoRaWAN low power)

**Communication**:
- Protocol: LoRaWAN (Long Range Wide Area Network)
- Range: Up to 15 km in rural areas
- Data transmission: Every 60 seconds
- Encryption: AES-128

**Installation Locations**:
1. Refrigerated trucks (3 sensors: front, middle, rear)
2. Cold storage warehouses (1 sensor per 100 m²)
3. Retail display cases (1 sensor per case)
4. Transportation containers (2 sensors per 40-foot container)

### Temperature Monitoring Algorithm
```python
def monitor_cold_chain(sensor_data):
    """
    Monitor temperature compliance per FDA requirements
    Alert if temp > critical limit for > 4 minutes
    """
    critical_limit = 4.0  # °C
    alert_threshold = 4 * 60  # 4 minutes in seconds

    out_of_spec_duration = 0
    for reading in sensor_data:
        if reading.temp > critical_limit:
            out_of_spec_duration += reading.interval
            if out_of_spec_duration >= alert_threshold:
                issue_alert(reading.batch_id, reading.temp, reading.location)
                quarantine_batch(reading.batch_id)
        else:
            out_of_spec_duration = 0  # Reset counter
```

## Traceability Performance

### Speed Comparison
| System | Traceability Time | Method |
|--------|------------------|---------|
| **Traditional** | 7-14 days | Paper records, manual phone calls |
| **Barcode** | 1-3 days | Centralized database, manual scanning |
| **WIA-FOOD_SAFETY** | **< 2.2 seconds** | Blockchain query, QR code scan |

### Real-World Case Study: Walmart + IBM Food Trust
- **2018 Pilot**: Tracing mangoes from farm to store
- **Result**: 2.2 seconds (blockchain) vs. 6 days 18 hours (traditional)
- **Impact**: Now deployed for 25+ products, 100+ suppliers

### Traceability Requirements
1. **One-Up**: Record where product came from (supplier ID, batch ID)
2. **One-Back**: Record where product was sent (customer ID, shipment ID)
3. **One-Step**: Full supply chain visibility (farmer → consumer)

## Key Features Detailed

### 1. Farm-to-Table Traceability
- **QR Code Generation**: ECC-200 Data Matrix, 50x50 modules
- **RFID Tags**: ISO 18000-63 (860-960 MHz), read range 5-10 meters
- **NFC Tags**: ISO 14443 Type A, read range 4 cm (for retail)
- **Blockchain Records**: Farmer name, GPS coordinates, harvest date, certifications

### 2. Contamination Alerts
**Pathogen Detection Integration**:
- Lab results API: LIMS (Laboratory Information Management System)
- Test types: PCR (Polymerase Chain Reaction), ATP (Adenosine Triphosphate), immunoassay
- Detection time: 24-48 hours (traditional) → 6 hours (rapid PCR)
- Positive result triggers instant blockchain alert + recall workflow

**Common Pathogens**:
- E. coli O157:H7 (beef, lettuce)
- Salmonella (poultry, eggs)
- Listeria monocytogenes (deli meats, soft cheese)
- Campylobacter (raw chicken)

### 3. Recall Management
**FDA Recall Classification**:
- **Class I**: Dangerous or defective products, high risk (e.g., botulism)
- **Class II**: May cause temporary health problems (e.g., undeclared allergen)
- **Class III**: Unlikely to cause harm (e.g., labeling error)

**WIA-FOOD_SAFETY Recall Workflow**:
1. Identify contaminated batch via blockchain query (< 2.2 sec)
2. Notify all downstream recipients (retailers, distributors) via SMS/email
3. Generate recall report with affected locations
4. Track product return via QR code scanning
5. Document destruction with photo proof on blockchain

## Technology Stack

### Blockchain
- **Primary**: Ethereum (public traceability) + Polygon (Layer-2 for cost)
- **Enterprise**: Hyperledger Fabric (private enterprise deployments)
- **Storage**: IPFS (photos, lab reports, certifications)
- **Identity**: DIDs (Decentralized Identifiers) for suppliers

### IoT & Sensors
- **Protocol**: LoRaWAN (long-range, low-power)
- **Sensors**: Temperature (Pt1000 RTD), humidity (capacitive), GPS
- **Gateways**: Multi-channel LoRaWAN gateways (8-channel minimum)
- **Platform**: AWS IoT Core, Azure IoT Hub

### APIs & Integration
- **REST API**: OpenAPI 3.0 specification
- **GraphQL**: Real-time queries for dashboard
- **Webhooks**: Instant alerts for CCP violations
- **EDI**: Electronic Data Interchange for legacy systems (X12, EDIFACT)

### Database
- **Blockchain**: Immutable records (transactions, CCP logs)
- **PostgreSQL**: Relational data (users, products, locations)
- **TimescaleDB**: Time-series sensor data (optimized for IoT)
- **Redis**: Caching for fast API responses

## Performance Metrics

### System Performance
- **Traceability Query**: < 2.2 seconds (99th percentile)
- **Blockchain Write**: < 5 seconds (Polygon Layer-2)
- **API Response**: < 100ms (95th percentile)
- **Uptime**: 99.95% SLA (4 hours downtime/year)

### Sensor Accuracy
- **Temperature**: ±0.1°C (Pt1000 RTD sensor)
- **Humidity**: ±2% RH (capacitive sensor)
- **GPS**: ±5 meters (with clear sky)
- **Battery Life**: 5 years (LoRaWAN, 60-second intervals)

### Blockchain Performance
- **Ethereum Mainnet**: 15-30 TPS (transactions per second)
- **Polygon Layer-2**: 1000+ TPS (sufficient for global deployment)
- **Hyperledger Fabric**: 3500+ TPS (private network)
- **Transaction Cost**: $0.01-0.05 (Polygon) vs. $1-5 (Ethereum mainnet)

## Deployment Models

### Cloud-Native (SaaS)
- Multi-tenant architecture
- Pay-per-transaction pricing
- Managed blockchain nodes
- Auto-scaling for peak loads

### On-Premises (Enterprise)
- Hyperledger Fabric private network
- Kubernetes deployment
- Integration with existing ERP (SAP, Oracle)
- Air-gapped option for sensitive data

### Hybrid
- Public blockchain for traceability (transparency)
- Private database for confidential data (pricing, contracts)
- API gateway for controlled access

---

**© 2026 WIA | 弘益人間 (Benefit All Humanity)**
