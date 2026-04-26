# WIA-ENE-050: Direct Air Capture Standard
## PHASE 4: INTEGRATION

**Version:** 1.0
**Status:** Draft
**Date:** 2025-12-25
**Category:** Energy (ENE)

---

## 1. Overview

This document specifies integration requirements for WIA-ENE-050 compliant systems. It covers APIs, protocols, and interfaces for connecting DAC facilities with storage infrastructure, energy systems, carbon registries, and public platforms.

### 1.1 Integration Domains

1. **Storage Integration:** Connection to geological storage, mineralization sites, utilization facilities
2. **Energy Integration:** Coordination with renewable energy providers and grids
3. **Registry Integration:** Carbon credit registries and blockchain networks
4. **MRV Integration:** Third-party verification systems
5. **Public Integration:** Open data platforms, dashboards, research databases
6. **Enterprise Integration:** Corporate carbon accounting and reporting systems

---

## 2. API Specification

### 2.1 RESTful API

**Base URL:** `https://api.wia.org/dac/v1`

#### 2.1.1 Authentication

**Method:** OAuth 2.0 Bearer Token

```http
POST /auth/token
Content-Type: application/json

{
  "client_id": "your-client-id",
  "client_secret": "your-client-secret",
  "grant_type": "client_credentials",
  "scope": "read:facility write:capture"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "read:facility write:capture"
}
```

#### 2.1.2 Endpoints

##### GET /facilities

List all DAC facilities.

**Request:**
```http
GET /facilities?country=IS&status=operational
Authorization: Bearer {token}
```

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "facilityId": "dac-climeworks-orca-001",
      "name": "Orca DAC Facility",
      "location": {
        "country": "IS",
        "latitude": 64.0685,
        "longitude": -21.9479
      },
      "capacity": {
        "annualTonsCO2": 4000
      },
      "status": "operational"
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 50,
    "total": 1
  }
}
```

##### GET /facilities/{facilityId}

Get detailed facility information.

**Request:**
```http
GET /facilities/dac-climeworks-orca-001
Authorization: Bearer {token}
```

**Response:** Complete facility object (see PHASE-2-DATA.md)

##### GET /facilities/{facilityId}/capture

Get real-time capture data.

**Request:**
```http
GET /facilities/dac-climeworks-orca-001/capture?start=2025-12-25T00:00:00Z&end=2025-12-25T23:59:59Z&interval=hourly
Authorization: Bearer {token}
```

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "timestamp": "2025-12-25T00:00:00Z",
      "capture": {
        "currentRate": 0.5,
        "hourlyTotal": 0.5,
        "efficiency": 0.96
      },
      "energy": {
        "currentConsumption": 125,
        "hourlyTotal": 0.125
      }
    }
  ]
}
```

##### POST /facilities/{facilityId}/capture

Submit capture data (for facility operators).

**Request:**
```http
POST /facilities/dac-climeworks-orca-001/capture
Authorization: Bearer {token}
Content-Type: application/json

{
  "timestamp": "2025-12-25T10:30:00Z",
  "capture": {
    "currentRate": 0.48,
    "dailyTotal": 3.84
  },
  "energy": {
    "currentConsumption": 120,
    "dailyTotal": 2.88
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "recordId": "rec-20251225-001",
    "timestamp": "2025-12-25T10:30:00Z",
    "status": "accepted"
  }
}
```

##### POST /storage/events

Submit CO2 storage event.

**Request:**
```http
POST /storage/events
Authorization: Bearer {token}
Content-Type: application/json

{
  "facilityId": "dac-climeworks-orca-001",
  "amount": 10.5,
  "storageMethod": "mineralization",
  "storageSite": {
    "siteId": "carbfix-hellisheidi-001"
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "eventId": "storage-orca-20251225-001",
    "status": "pending-verification"
  }
}
```

##### GET /verification/reports

List verification reports.

**Request:**
```http
GET /verification/reports?facilityId=dac-climeworks-orca-001&year=2025
Authorization: Bearer {token}
```

**Response:** Array of verification report objects

##### POST /credits/issue

Issue carbon removal credits (after verification).

**Request:**
```http
POST /credits/issue
Authorization: Bearer {token}
Content-Type: application/json

{
  "facilityId": "dac-climeworks-orca-001",
  "verificationReportId": "verify-2025-q4-001",
  "amount": 1000,
  "vintage": 2025,
  "registry": "Puro.earth"
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "creditId": "credit-2025-001",
    "serialNumber": "PURO-DAC-2025-12-001",
    "amount": 1000,
    "status": "issued"
  }
}
```

### 2.2 GraphQL API

**Endpoint:** `https://api.wia.org/graphql`

#### 2.2.1 Schema

```graphql
type Query {
  facility(facilityId: ID!): Facility
  facilities(filter: FacilityFilter, pagination: Pagination): FacilityConnection
  captureData(facilityId: ID!, timeRange: TimeRange): [CaptureData]
  storageEvents(facilityId: ID): [StorageEvent]
  verificationReports(facilityId: ID): [VerificationReport]
  carbonCredits(filter: CreditFilter): [CarbonCredit]
}

type Mutation {
  submitCaptureData(facilityId: ID!, data: CaptureDataInput!): CaptureDataResult
  submitStorageEvent(event: StorageEventInput!): StorageEventResult
  issueCredits(request: CreditIssuanceInput!): CreditIssuanceResult
}

type Subscription {
  captureDataUpdated(facilityId: ID!): CaptureData
  storageEventCreated(facilityId: ID): StorageEvent
}

type Facility {
  facilityId: ID!
  name: String!
  location: Location!
  technology: Technology!
  capacity: Capacity!
  operational: OperationalStatus!
  realtimeData: CaptureData
}

type CaptureData {
  timestamp: DateTime!
  capture: CaptureMetrics!
  energy: EnergyMetrics!
  environmental: EnvironmentalConditions
}

# Additional types...
```

#### 2.2.2 Example Query

```graphql
query GetFacilityWithRealtimeData {
  facility(facilityId: "dac-climeworks-orca-001") {
    facilityId
    name
    location {
      latitude
      longitude
      country
    }
    capacity {
      annualTonsCO2
    }
    realtimeData {
      timestamp
      capture {
        currentRate
        efficiency
      }
      energy {
        currentConsumption
        specificEnergy
      }
    }
  }
}
```

### 2.3 WebSocket API (Real-Time)

**Endpoint:** `wss://api.wia.org/ws`

#### 2.3.1 Connection

```javascript
const ws = new WebSocket('wss://api.wia.org/ws');

ws.on('open', () => {
  // Authenticate
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'your-bearer-token'
  }));

  // Subscribe to facility updates
  ws.send(JSON.stringify({
    type: 'subscribe',
    channel: 'facility:dac-climeworks-orca-001:capture'
  }));
});

ws.on('message', (data) => {
  const message = JSON.parse(data);
  console.log('Capture update:', message);
});
```

#### 2.3.2 Message Format

**Capture Data Update:**
```json
{
  "type": "capture_update",
  "facilityId": "dac-climeworks-orca-001",
  "timestamp": "2025-12-25T10:30:00Z",
  "data": {
    "capture": {
      "currentRate": 0.48,
      "efficiency": 0.96
    },
    "energy": {
      "currentConsumption": 120
    }
  }
}
```

---

## 3. Storage Integration

### 3.1 Pipeline Network Integration

**Standard:** ISO 27913 (CO2 Pipeline Transportation)

#### 3.1.1 Pipeline API

**Endpoint:** `https://pipeline.example.com/api/v1`

**Operations:**
- `POST /injection/request` - Request pipeline capacity
- `POST /injection/schedule` - Schedule CO2 injection
- `GET /injection/status/{id}` - Check injection status
- `POST /injection/confirm` - Confirm CO2 delivered

**Example:**
```json
POST /injection/request
{
  "facilityId": "dac-climeworks-orca-001",
  "amount": 10.5,
  "targetDate": "2025-12-26",
  "destinationSite": "carbfix-hellisheidi-001"
}

Response:
{
  "requestId": "pipe-req-001",
  "status": "approved",
  "scheduledDate": "2025-12-26T08:00:00Z",
  "pipelineSegment": "iceland-southwest-001"
}
```

#### 3.1.2 Data Exchange

**Shared Data:**
- Flow rate (tons/hour)
- Pressure (bar)
- Temperature (°C)
- CO2 purity (%)
- Metering data (cumulative tons)

**Communication:**
- Real-time: MQTT or WebSocket
- Batch: REST API (hourly summaries)
- Security: TLS 1.3, mutual authentication

### 3.2 Geological Storage Site Integration

**Standard:** ISO 27914 (Geological Storage)

#### 3.2.1 Storage Site API

**Operations:**
- `POST /injection/reserve` - Reserve injection capacity
- `POST /injection/execute` - Begin injection
- `GET /injection/monitoring/{id}` - Get monitoring data
- `GET /verification/{id}` - Get verification certificate

**Monitoring Data:**
```json
{
  "injectionId": "inj-2025-12-001",
  "siteId": "carbfix-hellisheidi-001",
  "monitoring": {
    "pressure": {
      "current": 85,
      "max": 120,
      "unit": "bar"
    },
    "seismic": {
      "events": 0,
      "lastSurvey": "2025-12-01"
    },
    "groundwater": {
      "pH": 7.2,
      "lastSample": "2025-12-20"
    }
  },
  "status": "stable"
}
```

### 3.3 CarbFix Integration (Mineralization)

**Specific Integration for Basalt Mineralization**

#### 3.3.1 Injection Protocol

**Pre-Injection:**
1. Send CO2 amount and schedule to CarbFix API
2. Receive water allocation (CO2:water = 1:25)
3. Coordinate delivery logistics

**During Injection:**
1. Real-time data exchange:
   - Dissolved CO2 concentration
   - pH and conductivity
   - Injection pressure and temperature
2. Adjust injection rate based on feedback

**Post-Injection:**
1. Receive mineralization timeline
2. Access monitoring data
3. Obtain verification certificate (after 2 years)

---

## 4. Energy System Integration

### 4.1 Renewable Energy Coordination

**Objective:** Optimize DAC operations for renewable energy availability

#### 4.1.1 Energy Forecasting Integration

**Data Sources:**
- Solar irradiance forecasts
- Wind speed predictions
- Geothermal availability
- Grid carbon intensity

**API Integration:**
```javascript
// Fetch renewable energy forecast
const forecast = await fetch('https://energy-grid.example.com/api/forecast', {
  method: 'POST',
  body: JSON.stringify({
    location: { lat: 64.0685, lon: -21.9479 },
    timeRange: { start: '2025-12-26T00:00:00Z', hours: 24 }
  })
});

// Adjust DAC schedule based on forecast
const schedule = optimizeDACSchedule(forecast, dacCapacity);
```

#### 4.1.2 Load Shifting

**Strategy:**
- Run adsorption cycles when renewable energy abundant
- Schedule regeneration (high energy) during peak renewable generation
- Delay non-critical operations during low renewable periods

**Implementation:**
```javascript
function optimizeDACSchedule(energyForecast, dacParams) {
  const schedule = [];

  energyForecast.forEach(hour => {
    if (hour.renewablePercentage > 80) {
      // High renewable availability - run regeneration
      schedule.push({
        operation: 'regeneration',
        modules: dacParams.totalModules,
        energyDemand: dacParams.regenerationEnergy
      });
    } else if (hour.renewablePercentage > 50) {
      // Moderate renewable - run adsorption
      schedule.push({
        operation: 'adsorption',
        modules: dacParams.totalModules,
        energyDemand: dacParams.adsorptionEnergy
      });
    } else {
      // Low renewable - minimal operations
      schedule.push({
        operation: 'idle',
        energyDemand: dacParams.baseloadEnergy
      });
    }
  });

  return schedule;
}
```

### 4.2 Grid Integration

**Standard:** IEEE 2030.5 (Smart Energy Profile)

#### 4.2.1 Demand Response

**Capabilities:**
- Curtailment: Reduce load within 5 minutes
- Load shifting: Reschedule operations up to 24 hours
- Frequency regulation: Adjust load for grid stability

**API:**
```json
POST /grid/demand-response/event
{
  "eventId": "dr-2025-12-25-001",
  "type": "load-reduction",
  "requestedReduction": 500,
  "unit": "kW",
  "start": "2025-12-25T14:00:00Z",
  "duration": 2,
  "compensation": 50,
  "unit": "$/MWh"
}

Response:
{
  "status": "accepted",
  "actualReduction": 480,
  "modules": ["module-1", "module-3", "module-5"]
}
```

---

## 5. Carbon Registry Integration

### 5.1 Puro.earth Integration

**Standard:** CO2 Removal Certificate (CORC) Methodology

#### 5.1.1 Project Registration

**Endpoint:** `https://api.puro.earth/v1`

```json
POST /projects/register
{
  "projectName": "Orca DAC Carbon Removal",
  "methodology": "DAC-Mineralization-v1.0",
  "facilityId": "dac-climeworks-orca-001",
  "location": { "country": "IS", "coordinates": [64.0685, -21.9479] },
  "annualCapacity": 4000,
  "storageMethod": "basalt-mineralization",
  "startDate": "2021-09-08"
}

Response:
{
  "projectId": "PURO-DAC-IS-001",
  "status": "registered",
  "nextStep": "validation"
}
```

#### 5.1.2 Credit Issuance

**Workflow:**
1. Submit verification report
2. Puro.earth validation
3. Credits issued to project account
4. Credits available for sale/retirement

**API:**
```json
POST /credits/issue
{
  "projectId": "PURO-DAC-IS-001",
  "verificationReportId": "verify-2025-q4-001",
  "amount": 1000,
  "vintage": 2025,
  "verifier": "DNV GL"
}

Response:
{
  "serialNumbers": [
    "PURO-DAC-2025-12-001-001",
    "PURO-DAC-2025-12-001-002",
    ...
  ],
  "amount": 1000,
  "status": "issued"
}
```

### 5.2 Blockchain Integration

**Networks:** Ethereum, Polygon, Hyperledger

#### 5.2.1 Smart Contract Interface

**Carbon Credit Token (ERC-721 NFT):**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

import "@openzeppelin/contracts/token/ERC721/ERC721.sol";

contract DACCarbonCredit is ERC721 {
    struct CreditData {
        string facilityId;
        uint256 amountTonsCO2;
        uint256 vintage;
        string storageMethod;
        string verificationReportURL;
        bool retired;
    }

    mapping(uint256 => CreditData) public credits;
    uint256 public nextTokenId;

    constructor() ERC721("DAC Carbon Removal Credit", "DACRC") {}

    function mintCredit(
        address to,
        string memory facilityId,
        uint256 amountTonsCO2,
        uint256 vintage,
        string memory storageMethod,
        string memory verificationReportURL
    ) external returns (uint256) {
        uint256 tokenId = nextTokenId++;
        _mint(to, tokenId);

        credits[tokenId] = CreditData({
            facilityId: facilityId,
            amountTonsCO2: amountTonsCO2,
            vintage: vintage,
            storageMethod: storageMethod,
            verificationReportURL: verificationReportURL,
            retired: false
        });

        return tokenId;
    }

    function retireCredit(uint256 tokenId) external {
        require(ownerOf(tokenId) == msg.sender, "Not credit owner");
        require(!credits[tokenId].retired, "Already retired");

        credits[tokenId].retired = true;
        // Transfer to burn address or lock permanently
    }

    function getCreditData(uint256 tokenId) external view returns (CreditData memory) {
        return credits[tokenId];
    }
}
```

#### 5.2.2 Web3 Integration

```javascript
const Web3 = require('web3');
const web3 = new Web3('https://polygon-rpc.com');

const contractAddress = '0x...';
const contractABI = [...];
const contract = new web3.eth.Contract(contractABI, contractAddress);

// Mint carbon credit
async function issueCredit(creditData) {
  const accounts = await web3.eth.getAccounts();

  const receipt = await contract.methods.mintCredit(
    accounts[0], // recipient
    creditData.facilityId,
    creditData.amountTonsCO2,
    creditData.vintage,
    creditData.storageMethod,
    creditData.verificationReportURL
  ).send({ from: accounts[0] });

  console.log('Credit issued:', receipt.events.Transfer.returnValues.tokenId);
  return receipt;
}

// Retire carbon credit
async function retireCredit(tokenId) {
  const accounts = await web3.eth.getAccounts();

  const receipt = await contract.methods.retireCredit(tokenId)
    .send({ from: accounts[0] });

  console.log('Credit retired:', tokenId);
  return receipt;
}
```

---

## 6. MRV System Integration

### 6.1 Third-Party Verifier Integration

**Verifiers:** DNV GL, Bureau Veritas, SCS Global, etc.

#### 6.1.1 Data Submission

**Automated Data Export:**
```javascript
async function submitToVerifier(facilityId, period) {
  // Gather all required data
  const data = {
    facilityInfo: await getFacilityInfo(facilityId),
    captureData: await getCaptureData(facilityId, period),
    energyData: await getEnergyData(facilityId, period),
    storageEvents: await getStorageEvents(facilityId, period),
    calibrationRecords: await getCalibrationRecords(facilityId, period),
    maintenanceLogs: await getMaintenanceLogs(facilityId, period)
  };

  // Generate verification package
  const package = {
    version: '1.0',
    standard: 'WIA-ENE-050',
    period: period,
    data: data,
    checksum: calculateChecksum(data)
  };

  // Submit to verifier portal
  const response = await fetch('https://verifier.example.com/api/submit', {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${verifierAPIKey}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify(package)
  });

  return response.json();
}
```

### 6.2 Verifiable Credentials

**Standard:** W3C Verifiable Credentials

#### 6.2.1 Issuance

```javascript
const { createVerifiableCredential } = require('did-jwt-vc');

async function issueVerificationCredential(verificationData) {
  const credentialSubject = {
    id: `did:wia:capture:${verificationData.eventId}`,
    type: 'CarbonRemoval',
    amount: {
      value: verificationData.amountTonsCO2,
      unit: 'tons CO2'
    },
    facility: verificationData.facilityId,
    period: verificationData.period,
    verificationStandard: 'ISO 27916',
    verifier: verificationData.verifierId
  };

  const vc = await createVerifiableCredential({
    credential: {
      '@context': [
        'https://www.w3.org/2018/credentials/v1',
        'https://wia.org/credentials/dac/v1'
      ],
      type: ['VerifiableCredential', 'CarbonRemovalCertificate'],
      issuer: { id: 'did:wia:verifier:dnv-gl' },
      credentialSubject: credentialSubject
    }
  }, { issuer: issuerDID, signer: issuerSigner });

  return vc;
}
```

#### 6.2.2 Verification

```javascript
const { verifyCredential } = require('did-jwt-vc');

async function verifyCredential(vcJWT) {
  const verified = await verifyCredential(vcJWT, resolver);

  if (verified.verified) {
    console.log('Credential is valid');
    console.log('Subject:', verified.verifiableCredential.credentialSubject);
  } else {
    console.log('Credential verification failed');
  }

  return verified;
}
```

---

## 7. Public Data Integration

### 7.1 Open Data API

**Endpoint:** `https://open-data.wia.org/dac/v1`

**No Authentication Required** (public data only)

#### 7.1.1 Available Data

- Aggregated capture statistics (by country, technology, month)
- Facility locations and capacity (if facility opts in)
- Industry benchmarks (average efficiency, energy intensity)
- Historical trends

**Example:**
```http
GET /statistics/global?year=2025

Response:
{
  "year": 2025,
  "totalFacilities": 42,
  "totalCapacity": 85000,
  "totalCaptured": 72000,
  "averageEfficiency": 0.89,
  "byCountry": [
    { "country": "IS", "facilities": 3, "captured": 15000 },
    { "country": "US", "facilities": 12, "captured": 28000 },
    ...
  ]
}
```

### 7.2 Dashboard Integration

**Real-Time Dashboard Widget:**

```html
<!-- Embed WIA-ENE-050 Dashboard Widget -->
<div id="wia-dac-widget"></div>
<script src="https://widgets.wia.org/dac/v1/widget.js"></script>
<script>
  WIADAC.init({
    container: '#wia-dac-widget',
    facilityId: 'dac-climeworks-orca-001',
    metrics: ['capture-rate', 'efficiency', 'energy'],
    theme: 'dark',
    refreshInterval: 60 // seconds
  });
</script>
```

### 7.3 Research Data Access

**For Academic Researchers:**

**Application Process:**
1. Submit research proposal
2. Sign data use agreement
3. Receive API credentials with higher rate limits
4. Access anonymized detailed data

**Available Data:**
- Minute-level operational data (anonymized)
- Sorbent performance data
- Energy optimization strategies
- Environmental condition correlations

---

## 8. Enterprise Integration

### 8.1 Corporate Carbon Accounting

**Integration with:** Microsoft Sustainability Manager, Salesforce Net Zero Cloud, etc.

#### 8.1.1 API Integration

```javascript
// Export carbon removal data to corporate system
async function exportToMSM(facilityId, period) {
  const captureData = await getVerifiedCapture(facilityId, period);

  const msmPayload = {
    activityType: 'CarbonRemoval',
    subType: 'DirectAirCapture',
    quantity: captureData.amount,
    unit: 'tonsCO2',
    date: captureData.date,
    facility: facilityId,
    verificationStandard: 'ISO 27916',
    certificateURL: captureData.certificateURL
  };

  // Submit to Microsoft Sustainability Manager
  const response = await fetch('https://msm-api.microsoft.com/api/activities', {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${msmToken}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify(msmPayload)
  });

  return response.json();
}
```

### 8.2 Reporting Integration

**Standards:** GRI, CDP, TCFD, SASB

**Data Export Formats:**
- CSV for spreadsheet import
- JSON for programmatic access
- PDF for official reports
- XBRL for regulatory filings

---

## 9. Integration Security

### 9.1 Authentication & Authorization

**Methods:**
- OAuth 2.0 for user delegation
- API keys for machine-to-machine
- mTLS for high-security connections
- JWT tokens for stateless authentication

### 9.2 Rate Limiting

**Default Limits:**
- Public API: 100 requests/hour
- Authenticated: 1,000 requests/hour
- Partner tier: 10,000 requests/hour
- Real-time WebSocket: Unlimited (fair use)

### 9.3 Audit Logging

**All API calls logged:**
- Timestamp
- Client ID
- Endpoint accessed
- Request parameters
- Response status
- IP address

---

## 10. Integration Testing

### 10.1 Conformance Testing

**Test Suite:** Available at `https://github.com/WIA-Official/wia-ene-050-tests`

**Tests:**
- API endpoint compliance
- Data format validation
- Authentication flows
- Error handling
- Rate limiting
- WebSocket connections

### 10.2 Interoperability Testing

**Annual Plugfest:**
- DAC facilities
- Storage providers
- Registry systems
- MRV platforms
- Test all integrations in realistic scenarios

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / World Internet Alliance (WIA)


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
