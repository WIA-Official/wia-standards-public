# WIA-FOOD_SAFETY Specification - PHASE 2: API Design & Data Models

**Version:** 1.0.0
**Last Updated:** 2026-01-11

## Overview

This document defines the API endpoints, data models, and integration interfaces for WIA-FOOD_SAFETY. The API follows RESTful principles with OpenAPI 3.0 specification and provides GraphQL for real-time queries.

## API Architecture

### Base URLs
```
Production:  https://api.wia-food-safety.org/v1
Staging:     https://api-staging.wia-food-safety.org/v1
Development: http://localhost:3000/v1
```

### Authentication
- **Method**: JWT (JSON Web Tokens) + API Keys
- **Token Lifetime**: 1 hour (access), 7 days (refresh)
- **Rate Limiting**: 1000 requests/hour (standard), 10,000 requests/hour (enterprise)
- **Security**: HTTPS only, TLS 1.3+

### API Standards
- **REST**: OpenAPI 3.0 specification
- **GraphQL**: Apollo Server v4
- **Versioning**: URI versioning (/v1, /v2)
- **Response Format**: JSON (default), XML (optional)
- **Error Handling**: RFC 7807 (Problem Details for HTTP APIs)

## Core API Endpoints

### 1. Product Registration & Traceability

#### POST /products/register
Register a new food product on the blockchain.

**Request Body:**
```json
{
  "productName": "Organic Romaine Lettuce",
  "category": "FRESH_PRODUCE",
  "batchId": "FARM-2026-001234",
  "origin": {
    "farmName": "Green Valley Farms",
    "location": {
      "latitude": 36.1699,
      "longitude": -115.1398,
      "address": "123 Farm Road, Salinas, CA 93901"
    },
    "certifications": ["USDA_ORGANIC", "GAP_CERTIFIED"]
  },
  "harvestDate": "2026-01-10T08:30:00Z",
  "quantity": {
    "value": 500,
    "unit": "kg"
  },
  "expirationDate": "2026-01-17T23:59:59Z",
  "haccp": {
    "planId": "HACCP-GVF-2026",
    "ccps": [
      {
        "type": "RECEIVING_TEMP",
        "criticalLimit": "4.0°C",
        "monitoringFrequency": "EVERY_60_SECONDS"
      }
    ]
  }
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "productId": "0x1234567890abcdef",
    "batchId": "FARM-2026-001234",
    "blockchainTxHash": "0xabcdef1234567890...",
    "qrCode": "data:image/png;base64,iVBORw0KGgoAAAANS...",
    "rfidTag": "E2801170000001234567890",
    "createdAt": "2026-01-11T10:15:30Z"
  }
}
```

#### GET /products/trace/{batchId}
Trace complete supply chain history in < 2.2 seconds.

**Response (200 OK):**
```json
{
  "success": true,
  "traceabilityTime": "0.87s",
  "data": {
    "batchId": "FARM-2026-001234",
    "currentStatus": "IN_TRANSIT",
    "supplyChain": [
      {
        "stage": "FARM",
        "actor": "Green Valley Farms",
        "location": "Salinas, CA",
        "timestamp": "2026-01-10T08:30:00Z",
        "temperature": "12.5°C",
        "blockchainTxHash": "0xabc..."
      },
      {
        "stage": "PROCESSING",
        "actor": "FreshCut Processing Inc",
        "location": "San Jose, CA",
        "timestamp": "2026-01-10T14:20:00Z",
        "temperature": "3.2°C",
        "blockchainTxHash": "0xdef..."
      },
      {
        "stage": "DISTRIBUTION",
        "actor": "ColdChain Logistics",
        "location": "En route to Los Angeles",
        "timestamp": "2026-01-11T06:45:00Z",
        "temperature": "2.8°C",
        "gps": {
          "latitude": 34.0522,
          "longitude": -118.2437
        },
        "blockchainTxHash": "0xghi..."
      }
    ],
    "totalDistance": "520 km",
    "totalTime": "22h 15m"
  }
}
```

#### GET /products/qr-scan/{qrCode}
Scan QR code and retrieve product information.

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "productName": "Organic Romaine Lettuce",
    "batchId": "FARM-2026-001234",
    "origin": "Green Valley Farms, Salinas, CA",
    "harvestDate": "2026-01-10",
    "expirationDate": "2026-01-17",
    "certifications": ["USDA Organic", "GAP Certified"],
    "safetyStatus": "PASS",
    "lastTemperature": "2.8°C",
    "blockchainVerified": true
  }
}
```

### 2. HACCP & Critical Control Points

#### POST /haccp/ccp-record
Log Critical Control Point measurement.

**Request Body:**
```json
{
  "batchId": "FARM-2026-001234",
  "ccpType": "COLD_STORAGE",
  "measurement": {
    "temperature": 3.5,
    "unit": "CELSIUS",
    "humidity": 85,
    "timestamp": "2026-01-11T10:30:00Z"
  },
  "criticalLimit": {
    "min": 0,
    "max": 4,
    "unit": "CELSIUS"
  },
  "sensorId": "TEMP-SENSOR-001",
  "location": "Warehouse A, Zone 3"
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "recordId": "CCP-2026-001234-5678",
    "status": "PASS",
    "deviationDetected": false,
    "blockchainTxHash": "0xjkl...",
    "timestamp": "2026-01-11T10:30:05Z"
  }
}
```

#### GET /haccp/ccp-violations
Retrieve CCP violations requiring corrective action.

**Query Parameters:**
- `startDate`: ISO 8601 date (e.g., 2026-01-01)
- `endDate`: ISO 8601 date
- `ccpType`: RECEIVING_TEMP, COOKING_TEMP, COLD_STORAGE, etc.
- `severity`: CRITICAL, MAJOR, MINOR

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "totalViolations": 3,
    "violations": [
      {
        "violationId": "VIO-2026-001",
        "batchId": "FARM-2026-001234",
        "ccpType": "COLD_STORAGE",
        "criticalLimit": "≤ 4.0°C",
        "actualValue": "6.2°C",
        "duration": "8 minutes",
        "timestamp": "2026-01-11T03:15:00Z",
        "severity": "CRITICAL",
        "correctiveAction": {
          "status": "COMPLETED",
          "action": "Product quarantined, temperature adjusted",
          "completedBy": "John Supervisor",
          "completedAt": "2026-01-11T03:30:00Z"
        }
      }
    ]
  }
}
```

### 3. Temperature Monitoring & IoT

#### POST /iot/temperature-log
Bulk upload temperature sensor data.

**Request Body:**
```json
{
  "sensorId": "TEMP-SENSOR-001",
  "batchId": "FARM-2026-001234",
  "readings": [
    {
      "temperature": 3.2,
      "humidity": 82,
      "timestamp": "2026-01-11T10:00:00Z",
      "location": {
        "latitude": 34.0522,
        "longitude": -118.2437
      }
    },
    {
      "temperature": 3.4,
      "humidity": 83,
      "timestamp": "2026-01-11T10:01:00Z",
      "location": {
        "latitude": 34.0525,
        "longitude": -118.2440
      }
    }
  ]
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "recordsCreated": 2,
    "alertsTriggered": 0,
    "blockchainTxHash": "0xmno..."
  }
}
```

#### GET /iot/temperature-history/{batchId}
Retrieve complete temperature history for a batch.

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "batchId": "FARM-2026-001234",
    "sensorId": "TEMP-SENSOR-001",
    "totalReadings": 1440,
    "timeRange": {
      "start": "2026-01-10T08:30:00Z",
      "end": "2026-01-11T08:30:00Z"
    },
    "statistics": {
      "avgTemperature": 3.1,
      "minTemperature": 2.5,
      "maxTemperature": 4.0,
      "violations": 0
    },
    "readings": [
      {
        "temperature": 3.2,
        "timestamp": "2026-01-11T10:00:00Z"
      }
    ]
  }
}
```

### 4. Recall Management

#### POST /recalls/initiate
Initiate product recall.

**Request Body:**
```json
{
  "batchIds": ["FARM-2026-001234", "FARM-2026-001235"],
  "reason": "E. coli O157:H7 detected in lab testing",
  "severity": "CLASS_I",
  "recallScope": "NATIONWIDE",
  "contactInfo": {
    "company": "Green Valley Farms",
    "phone": "+1-555-RECALL",
    "email": "recall@greenvalleyfarms.com"
  },
  "labReport": {
    "testDate": "2026-01-11",
    "pathogen": "E_COLI_O157H7",
    "labName": "FDA Regional Lab",
    "reportUrl": "https://docs.wia-food-safety.org/lab/report-001.pdf"
  }
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "recallId": "RECALL-2026-001",
    "affectedProducts": 2,
    "distributionPoints": 47,
    "estimatedQuantity": "1,250 kg",
    "notificationsSent": {
      "sms": 15,
      "email": 47,
      "pushNotifications": 32
    },
    "pressRelease": "https://wia-food-safety.org/recalls/2026-001",
    "blockchainTxHash": "0xpqr...",
    "initiatedAt": "2026-01-11T11:00:00Z"
  }
}
```

#### GET /recalls/{recallId}/status
Track recall progress and effectiveness.

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "recallId": "RECALL-2026-001",
    "status": "IN_PROGRESS",
    "effectiveness": {
      "totalUnits": 1250,
      "unitsRecovered": 987,
      "recoveryRate": "78.96%",
      "unitsDestroyed": 850
    },
    "timeline": [
      {
        "event": "RECALL_INITIATED",
        "timestamp": "2026-01-11T11:00:00Z"
      },
      {
        "event": "FDA_NOTIFIED",
        "timestamp": "2026-01-11T11:05:00Z"
      },
      {
        "event": "PRESS_RELEASE_PUBLISHED",
        "timestamp": "2026-01-11T11:30:00Z"
      }
    ]
  }
}
```

### 5. Laboratory Testing Integration

#### POST /lab/test-results
Submit laboratory test results.

**Request Body:**
```json
{
  "batchId": "FARM-2026-001234",
  "labName": "FDA Regional Laboratory",
  "testType": "PATHOGEN_SCREENING",
  "sampleDate": "2026-01-10",
  "testDate": "2026-01-11",
  "results": [
    {
      "pathogen": "E_COLI_O157H7",
      "detected": false,
      "detectionLimit": "< 10 CFU/g",
      "method": "PCR"
    },
    {
      "pathogen": "SALMONELLA",
      "detected": false,
      "detectionLimit": "< 1 CFU/25g",
      "method": "ELISA"
    },
    {
      "pathogen": "LISTERIA_MONOCYTOGENES",
      "detected": false,
      "detectionLimit": "< 10 CFU/g",
      "method": "PCR"
    }
  ],
  "overallResult": "PASS",
  "certifiedBy": "Dr. Jane Microbiologist",
  "reportUrl": "https://lab.example.com/reports/2026-001234.pdf"
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "testId": "LAB-2026-001234",
    "batchId": "FARM-2026-001234",
    "overallResult": "PASS",
    "blockchainTxHash": "0xstu...",
    "certificateUrl": "https://wia-food-safety.org/certificates/LAB-2026-001234.pdf",
    "timestamp": "2026-01-11T12:00:00Z"
  }
}
```

## Data Models

### FoodProduct
Complete product definition with traceability metadata.

```typescript
interface FoodProduct {
  productId: string;              // UUID or blockchain address
  batchId: string;                // Unique batch identifier
  productName: string;            // "Organic Romaine Lettuce"
  category: ProductCategory;      // FRESH_PRODUCE, MEAT, DAIRY, etc.

  origin: Origin;
  packaging: Packaging;
  quantity: Quantity;

  dates: {
    harvestDate: DateTime;
    processingDate?: DateTime;
    expirationDate: DateTime;
    bestBeforeDate?: DateTime;
  };

  haccp: HACCPPlan;
  certifications: Certification[];
  blockchain: BlockchainMetadata;

  status: ProductStatus;          // ACTIVE, RECALLED, EXPIRED, DESTROYED
  createdAt: DateTime;
  updatedAt: DateTime;
}

enum ProductCategory {
  FRESH_PRODUCE = "FRESH_PRODUCE",
  MEAT = "MEAT",
  POULTRY = "POULTRY",
  SEAFOOD = "SEAFOOD",
  DAIRY = "DAIRY",
  EGGS = "EGGS",
  PROCESSED_FOODS = "PROCESSED_FOODS",
  BEVERAGES = "BEVERAGES",
  BAKERY = "BAKERY"
}

interface Origin {
  farmName?: string;
  manufacturerName?: string;
  location: Location;
  certifications: CertificationType[];
  supplierId: string;
}

interface Location {
  latitude: number;               // -90 to 90
  longitude: number;              // -180 to 180
  address: string;
  city: string;
  state: string;
  country: string;                // ISO 3166-1 alpha-2 (e.g., "US")
  postalCode: string;
}
```

### BatchRecord
Supply chain event tracking for one-up/one-back traceability.

```typescript
interface BatchRecord {
  recordId: string;
  batchId: string;

  stage: SupplyChainStage;
  actor: {
    name: string;
    companyId: string;
    role: ActorRole;              // FARMER, PROCESSOR, DISTRIBUTOR, RETAILER
  };

  location: Location;
  timestamp: DateTime;

  // One-up traceability
  receivedFrom?: {
    companyId: string;
    batchId: string;
    shipmentId: string;
  };

  // One-back traceability
  sentTo?: {
    companyId: string;
    shipmentId: string;
  };

  temperature?: number;           // °C
  humidity?: number;              // % RH
  gps?: {
    latitude: number;
    longitude: number;
  };

  blockchainTxHash: string;
  createdAt: DateTime;
}

enum SupplyChainStage {
  FARM = "FARM",
  HARVEST = "HARVEST",
  PROCESSING = "PROCESSING",
  PACKAGING = "PACKAGING",
  STORAGE = "STORAGE",
  DISTRIBUTION = "DISTRIBUTION",
  RETAIL = "RETAIL",
  CONSUMER = "CONSUMER"
}
```

### TemperatureLog
IoT sensor readings for cold chain monitoring.

```typescript
interface TemperatureLog {
  logId: string;
  sensorId: string;
  batchId: string;

  temperature: number;            // °C
  humidity?: number;              // % RH
  lightExposure?: number;         // Lux

  timestamp: DateTime;
  location?: {
    latitude: number;
    longitude: number;
  };

  withinSpec: boolean;
  criticalLimit: {
    min: number;
    max: number;
  };

  alertTriggered: boolean;
  blockchainTxHash?: string;      // Logged every 10 minutes to reduce cost
}

interface SensorCalibration {
  sensorId: string;
  calibratedDate: DateTime;
  nextCalibrationDate: DateTime;
  accuracy: string;               // "±0.1°C"
  calibratedBy: string;
  certificateUrl: string;
}
```

### HACCPPlan
HACCP 7 principles implementation.

```typescript
interface HACCPPlan {
  planId: string;
  companyName: string;
  productType: string;

  hazardAnalysis: Hazard[];
  ccps: CriticalControlPoint[];

  verificationProcedures: VerificationProcedure[];
  recordKeeping: RecordKeepingRequirement[];

  approvedBy: string;
  approvalDate: DateTime;
  nextReviewDate: DateTime;
}

interface Hazard {
  type: HazardType;               // BIOLOGICAL, CHEMICAL, PHYSICAL
  description: string;
  likelihood: RiskLevel;          // LOW, MEDIUM, HIGH
  severity: RiskLevel;
  controlMeasures: string[];
}

enum HazardType {
  BIOLOGICAL = "BIOLOGICAL",      // E. coli, Salmonella, Listeria
  CHEMICAL = "CHEMICAL",          // Pesticides, allergens, toxins
  PHYSICAL = "PHYSICAL"           // Glass, metal, plastic
}

interface CriticalControlPoint {
  ccpId: string;
  type: CCPType;
  description: string;
  criticalLimit: CriticalLimit;
  monitoringProcedure: MonitoringProcedure;
  correctiveAction: CorrectiveAction;
}

enum CCPType {
  RECEIVING_TEMP = "RECEIVING_TEMP",
  COOKING_TEMP = "COOKING_TEMP",
  COOLING_TIME = "COOLING_TIME",
  COLD_STORAGE = "COLD_STORAGE",
  HOT_HOLDING = "HOT_HOLDING",
  PH_CONTROL = "PH_CONTROL",
  WATER_ACTIVITY = "WATER_ACTIVITY",
  METAL_DETECTION = "METAL_DETECTION"
}

interface CriticalLimit {
  parameter: string;              // "Temperature", "pH", "Time"
  min?: number;
  max?: number;
  target?: number;
  unit: string;                   // "°C", "pH", "minutes"
}

interface MonitoringProcedure {
  frequency: string;              // "Every 60 seconds", "Every batch"
  method: string;                 // "IoT sensor", "Visual inspection"
  responsiblePerson: string;
}

interface CorrectiveAction {
  trigger: string;                // "Temperature > 4°C for > 4 minutes"
  actions: string[];              // ["Alert supervisor", "Quarantine batch"]
  responsiblePerson: string;
  documentationRequired: boolean;
}
```

### RecallNotice
Product recall management.

```typescript
interface RecallNotice {
  recallId: string;
  batchIds: string[];

  reason: string;
  severity: RecallClass;
  scope: RecallScope;

  initiatedBy: {
    company: string;
    contact: ContactInfo;
  };

  affectedProducts: {
    totalQuantity: number;
    unit: string;
    distributionPoints: number;
    estimatedConsumerReach: number;
  };

  labReport?: LabTestResult;

  status: RecallStatus;
  effectiveness: RecallEffectiveness;

  notifications: {
    fdaNotified: boolean;
    pressReleaseUrl?: string;
    distributorsNotified: number;
    retailersNotified: number;
  };

  timeline: RecallEvent[];

  blockchainTxHash: string;
  initiatedAt: DateTime;
  completedAt?: DateTime;
}

enum RecallClass {
  CLASS_I = "CLASS_I",            // Serious health hazard (death, injury)
  CLASS_II = "CLASS_II",          // Temporary health problems
  CLASS_III = "CLASS_III"         // Unlikely to cause health problems
}

enum RecallScope {
  NATIONWIDE = "NATIONWIDE",
  REGIONAL = "REGIONAL",
  LOCAL = "LOCAL",
  INTERNATIONAL = "INTERNATIONAL"
}

enum RecallStatus {
  INITIATED = "INITIATED",
  IN_PROGRESS = "IN_PROGRESS",
  COMPLETED = "COMPLETED",
  TERMINATED = "TERMINATED"
}

interface RecallEffectiveness {
  totalUnits: number;
  unitsRecovered: number;
  recoveryRate: number;           // Percentage
  unitsDestroyed: number;
  unitsReturned: number;
}

interface RecallEvent {
  event: string;                  // "RECALL_INITIATED", "FDA_NOTIFIED"
  description?: string;
  timestamp: DateTime;
  blockchainTxHash?: string;
}
```

### LabTestResult
Laboratory testing integration.

```typescript
interface LabTestResult {
  testId: string;
  batchId: string;

  labName: string;
  labAccreditation: string[];     // "ISO/IEC 17025", "FDA Registered"

  testType: TestType;
  sampleDate: DateTime;
  testDate: DateTime;

  results: TestResult[];
  overallResult: TestOutcome;

  certifiedBy: string;
  reportUrl: string;              // PDF certificate
  blockchainTxHash: string;

  createdAt: DateTime;
}

enum TestType {
  PATHOGEN_SCREENING = "PATHOGEN_SCREENING",
  ALLERGEN_TESTING = "ALLERGEN_TESTING",
  CHEMICAL_RESIDUE = "CHEMICAL_RESIDUE",
  NUTRITIONAL_ANALYSIS = "NUTRITIONAL_ANALYSIS",
  HEAVY_METALS = "HEAVY_METALS"
}

interface TestResult {
  parameter: string;              // "E. coli O157:H7", "Salmonella"
  detected: boolean;
  value?: string;                 // "< 10 CFU/g"
  detectionLimit: string;
  method: string;                 // "PCR", "ELISA", "ICP-MS"
  unit?: string;
}

enum TestOutcome {
  PASS = "PASS",
  FAIL = "FAIL",
  INCONCLUSIVE = "INCONCLUSIVE"
}
```

## GraphQL Schema

### Queries

```graphql
type Query {
  # Product traceability
  product(batchId: String!): FoodProduct
  traceSupplyChain(batchId: String!): SupplyChainTrace

  # HACCP monitoring
  ccpViolations(
    startDate: DateTime!
    endDate: DateTime!
    severity: Severity
  ): [CCPViolation!]!

  # Temperature monitoring
  temperatureHistory(
    batchId: String!
    startDate: DateTime
    endDate: DateTime
  ): [TemperatureLog!]!

  # Recall management
  recall(recallId: String!): RecallNotice
  activeRecalls: [RecallNotice!]!

  # Lab testing
  labTestResult(testId: String!): LabTestResult
}

type Subscription {
  # Real-time temperature monitoring
  temperatureAlert(batchId: String!): TemperatureLog!

  # CCP violations
  ccpViolation(companyId: String!): CCPViolation!

  # Recall notifications
  recallNotification: RecallNotice!
}

type Mutation {
  registerProduct(input: ProductInput!): FoodProduct!
  recordCCP(input: CCPInput!): CCPRecord!
  initiateRecall(input: RecallInput!): RecallNotice!
  submitLabResults(input: LabResultInput!): LabTestResult!
}
```

## Blockchain Smart Contract Integration

### Event Emissions
Smart contracts emit events for critical operations:

```solidity
event ProductRegistered(
    bytes32 indexed batchId,
    address indexed supplier,
    string productName,
    uint256 timestamp
);

event CCPRecorded(
    bytes32 indexed batchId,
    string ccpType,
    bool passed,
    uint256 timestamp
);

event RecallIssued(
    bytes32 indexed batchId,
    string reason,
    uint8 severity,
    uint256 timestamp
);

event TemperatureLogged(
    bytes32 indexed batchId,
    int16 temperature,  // Temperature * 10 (e.g., 35 = 3.5°C)
    uint256 timestamp
);
```

### Read Operations
```typescript
// Query blockchain for product verification
async function verifyProduct(batchId: string): Promise<boolean> {
  const product = await contract.products(batchId);
  return product.isRegistered && !product.isRecalled;
}

// Get complete CCP history
async function getCCPHistory(batchId: string): Promise<CCPRecord[]> {
  const events = await contract.queryFilter(
    contract.filters.CCPRecorded(batchId)
  );
  return events.map(event => ({
    ccpType: event.args.ccpType,
    passed: event.args.passed,
    timestamp: event.args.timestamp
  }));
}
```

## Error Handling

### Standard Error Response
```json
{
  "success": false,
  "error": {
    "code": "PRODUCT_NOT_FOUND",
    "message": "Product with batch ID 'FARM-2026-999999' not found",
    "details": {
      "batchId": "FARM-2026-999999",
      "searchedAt": "2026-01-11T12:00:00Z"
    },
    "timestamp": "2026-01-11T12:00:00.123Z",
    "requestId": "req-abc123"
  }
}
```

### Error Codes
| Code | HTTP Status | Description |
|------|-------------|-------------|
| `PRODUCT_NOT_FOUND` | 404 | Batch ID does not exist |
| `INVALID_BATCH_ID` | 400 | Batch ID format invalid |
| `CCP_VIOLATION` | 422 | Critical Control Point exceeded |
| `BLOCKCHAIN_ERROR` | 503 | Blockchain transaction failed |
| `UNAUTHORIZED` | 401 | Invalid or expired token |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `RECALL_IN_PROGRESS` | 423 | Product is recalled |

---

**© 2026 WIA | 弘益人間 (Benefit All Humanity)**
