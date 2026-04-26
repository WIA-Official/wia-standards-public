# Chapter 5: API Interface Specifications

## Learning Objectives

After completing this chapter, you will be able to:

1. Implement WIA E-Waste API authentication and authorization
2. Design device registration and tracking endpoints
3. Build collection and processing event APIs
4. Create material recovery reporting interfaces
5. Develop compliance and reporting API integrations

---

## 5.1 API Architecture Overview

### 5.1.1 RESTful API Design

```
WIA E-Waste API Architecture:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  CLIENT APPLICATIONS                                                │
│  ├─ Producer Systems                                               │
│  ├─ Collection Point Apps                                          │
│  ├─ Processing Facility ERP                                        │
│  ├─ Regulatory Portals                                             │
│  └─ Consumer Mobile Apps                                           │
│                                                                     │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    API GATEWAY                               │   │
│  │  ├─ Rate Limiting                                           │   │
│  │  ├─ Authentication                                          │   │
│  │  ├─ Request Routing                                         │   │
│  │  └─ Logging                                                 │   │
│  └─────────────────────────────────────────────────────────────┘   │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    API SERVICES                              │   │
│  │                                                              │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │   │
│  │  │ Device   │ │Collection│ │Processing│ │Compliance│       │   │
│  │  │ Registry │ │ Service  │ │ Service  │ │ Service  │       │   │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘       │   │
│  │                                                              │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │   │
│  │  │ Material │ │ Facility │ │ Chain of │ │ Analytics│       │   │
│  │  │ Recovery │ │ Registry │ │ Custody  │ │ Service  │       │   │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘       │   │
│  │                                                              │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.1.2 Base URL and Versioning

```typescript
// API base configuration
const apiConfig = {
  production: "https://api.wia-ewaste.org/v1",
  sandbox: "https://sandbox.wia-ewaste.org/v1",

  versioning: {
    current: "v1",
    supported: ["v1"],
    deprecated: [],
    sunset: []
  },

  contentType: "application/json",
  acceptedFormats: ["application/json", "application/xml"],

  rateLimit: {
    basic: { requests: 100, period: "minute" },
    standard: { requests: 1000, period: "minute" },
    premium: { requests: 10000, period: "minute" }
  }
};
```

### 5.1.3 Common Response Format

```typescript
// Standard API response wrapper
interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  meta: {
    requestId: string;
    timestamp: string;
    version: string;
  };
  pagination?: {
    page: number;
    pageSize: number;
    totalItems: number;
    totalPages: number;
    hasNext: boolean;
    hasPrevious: boolean;
  };
}

// Example success response
{
  "success": true,
  "data": {
    "deviceId": "WIA-SAMSUNG-SM-2024-A1B2C3D4-7X",
    "status": "registered"
  },
  "meta": {
    "requestId": "req-abc123",
    "timestamp": "2025-01-15T10:30:00Z",
    "version": "v1"
  }
}

// Example error response
{
  "success": false,
  "error": {
    "code": "DEVICE_NOT_FOUND",
    "message": "Device with specified ID does not exist",
    "details": {
      "deviceId": "WIA-INVALID-ID"
    }
  },
  "meta": {
    "requestId": "req-xyz789",
    "timestamp": "2025-01-15T10:30:00Z",
    "version": "v1"
  }
}
```

---

## 5.2 Authentication and Authorization

### 5.2.1 OAuth 2.0 Implementation

```typescript
// OAuth 2.0 configuration
interface AuthConfig {
  authorizationEndpoint: "https://auth.wia-ewaste.org/oauth/authorize";
  tokenEndpoint: "https://auth.wia-ewaste.org/oauth/token";

  grantTypes: ["client_credentials", "authorization_code", "refresh_token"];

  scopes: {
    "device:read": "Read device information";
    "device:write": "Register and update devices";
    "collection:read": "Read collection events";
    "collection:write": "Submit collection events";
    "processing:read": "Read processing data";
    "processing:write": "Submit processing events";
    "material:read": "Read material recovery data";
    "material:write": "Submit material recovery data";
    "compliance:read": "Read compliance reports";
    "compliance:write": "Submit compliance reports";
    "admin": "Administrative access";
  };
}

// Token request example
async function getAccessToken(clientId: string, clientSecret: string): Promise<TokenResponse> {
  const response = await fetch("https://auth.wia-ewaste.org/oauth/token", {
    method: "POST",
    headers: {
      "Content-Type": "application/x-www-form-urlencoded",
      "Authorization": `Basic ${btoa(`${clientId}:${clientSecret}`)}`
    },
    body: new URLSearchParams({
      grant_type: "client_credentials",
      scope: "device:read device:write collection:write"
    })
  });

  return response.json();
}

// Token response
interface TokenResponse {
  access_token: string;
  token_type: "Bearer";
  expires_in: number;      // seconds
  refresh_token?: string;
  scope: string;
}
```

### 5.2.2 API Key Authentication

```typescript
// For simpler integrations
interface ApiKeyAuth {
  header: "X-WIA-API-Key";
  format: "wia_live_[32-character-key]";

  // Example request
  headers: {
    "X-WIA-API-Key": "wia_live_abc123def456ghi789jkl012mno345pq",
    "Content-Type": "application/json"
  };
}
```

### 5.2.3 Role-Based Access Control

| Role | Scopes | Description |
|------|--------|-------------|
| Producer | device:write, device:read, compliance:read | Register devices, view compliance |
| Collector | collection:write, device:read | Submit collection events |
| Processor | processing:write, material:write, device:read | Submit processing data |
| Regulator | *:read, compliance:write | Full read access, compliance verification |
| Consumer | device:read (own) | View recycling certificate |
| Auditor | *:read | Read-only audit access |

---

## 5.3 Device Registry API

### 5.3.1 Device Registration Endpoints

```typescript
// POST /devices - Register new device
interface RegisterDeviceRequest {
  producer: {
    id: string;
    name: string;
    model: string;
    modelNumber: string;
  };
  category: {
    wiaCategory: string;
    weeeCategory: number;
  };
  production: {
    date: string;
    facility: string;
    batchNumber?: string;
    serialNumber?: string;
  };
  physical?: {
    weightKg: number;
    dimensions?: object;
  };
  materials: MaterialComposition;
  hazardous?: HazardousSubstances;
}

// Response
interface RegisterDeviceResponse {
  deviceId: string;              // Assigned WDID
  registrationDate: string;
  status: "registered";
  qrCode: string;                // URL to QR code image
  verificationUrl: string;       // Consumer verification URL
}

// GET /devices/{deviceId} - Retrieve device information
// GET /devices?producer={producerId}&category={category}&from={date}&to={date}

// PUT /devices/{deviceId} - Update device information
// PATCH /devices/{deviceId}/status - Update device status
```

### 5.3.2 Device Lookup Examples

```bash
# Register a new device
curl -X POST https://api.wia-ewaste.org/v1/devices \
  -H "Authorization: Bearer {token}" \
  -H "Content-Type: application/json" \
  -d '{
    "producer": {
      "id": "SAMSUNG-KR",
      "name": "Samsung Electronics",
      "model": "Galaxy S24",
      "modelNumber": "SM-S921B"
    },
    "category": {
      "wiaCategory": "SM",
      "weeeCategory": 3
    },
    "production": {
      "date": "2024-01-15",
      "facility": "Samsung Gumi Plant",
      "batchNumber": "2024-01-001"
    },
    "physical": {
      "weightKg": 0.187
    },
    "materials": {
      "totalWeightKg": 0.187,
      "aggregates": {
        "metals": {"ferrous": 0.025, "nonFerrousBase": 0.042}
      }
    }
  }'

# Lookup device by ID
curl -X GET https://api.wia-ewaste.org/v1/devices/WIA-SAMSUNG-SM-2024-A1B2C3D4-7X \
  -H "Authorization: Bearer {token}"

# Search devices by producer
curl -X GET "https://api.wia-ewaste.org/v1/devices?producer=SAMSUNG-KR&from=2024-01-01&to=2024-12-31&page=1&pageSize=100" \
  -H "Authorization: Bearer {token}"
```

### 5.3.3 Batch Device Registration

```typescript
// POST /devices/batch - Register multiple devices
interface BatchRegisterRequest {
  batchId: string;
  producer: ProducerInfo;
  devices: {
    serialNumber: string;
    model: string;
    productionDate: string;
    // Individual overrides
  }[];
  commonAttributes: {
    category: CategoryInfo;
    materials: MaterialComposition;
    facility: string;
  };
}

// Response
interface BatchRegisterResponse {
  batchId: string;
  totalSubmitted: number;
  successful: number;
  failed: number;
  devices: {
    serialNumber: string;
    deviceId?: string;           // WDID if successful
    status: "registered" | "failed";
    error?: string;
  }[];
}
```

---

## 5.4 Collection API

### 5.4.1 Collection Event Endpoints

```typescript
// POST /collections - Submit collection event
interface CollectionEventRequest {
  collectionPoint: {
    id: string;
    type: "retail" | "municipal" | "producer_program" | "private";
  };

  // For individual device tracking
  device?: {
    deviceId?: string;           // WDID if known
    serialNumber?: string;       // Alternative lookup
    imei?: string;               // For phones
  };

  // For batch collection
  batch?: {
    category: string;
    quantity?: number;
    weightKg: number;
    description?: string;
  };

  condition: {
    functional: boolean;
    physicalCondition: "excellent" | "good" | "fair" | "poor" | "damaged";
    completeness: "complete" | "missing_accessories" | "missing_parts";
    batteryPresent?: boolean;
    dataWiped?: boolean;
  };

  consumer?: {
    anonymousId?: string;
    email?: string;              // For certificate delivery
    certificateRequested: boolean;
  };

  routing: {
    recommendation: "reuse" | "refurbishment" | "recycling";
    destinationFacility?: string;
  };

  verification: {
    method: "visual" | "functional_test" | "automated";
    operatorId: string;
    photos?: string[];           // URLs to evidence photos
  };
}

// Response
interface CollectionEventResponse {
  eventId: string;
  collectionId: string;          // Public reference number
  deviceId?: string;
  timestamp: string;

  status: "accepted" | "pending_verification";

  certificate?: {
    id: string;
    url: string;
    qrCode: string;
  };

  nextSteps: {
    routing: string;
    estimatedProcessingDate?: string;
  };
}
```

### 5.4.2 Collection Point Management

```typescript
// GET /collection-points - List collection points
// GET /collection-points/{id} - Get collection point details
// POST /collection-points - Register new collection point

interface CollectionPoint {
  id: string;
  name: string;
  type: "retail" | "municipal" | "producer_program" | "private" | "mobile";

  location: {
    address: string;
    city: string;
    postalCode: string;
    country: string;
    coordinates: {
      latitude: number;
      longitude: number;
    };
  };

  operator: {
    entityId: string;
    name: string;
    contact: string;
  };

  capabilities: {
    acceptedCategories: string[];
    maxWeightKg?: number;
    hasScanning: boolean;
    hasFunctionalTesting: boolean;
    hasSecureDataWipe: boolean;
  };

  operatingHours: {
    [day: string]: { open: string; close: string } | "closed";
  };

  certifications: string[];
  status: "active" | "inactive" | "suspended";
}

// GET /collection-points/nearby?lat={lat}&lng={lng}&radius={km}
```

### 5.4.3 Collection Statistics

```typescript
// GET /collections/statistics
interface CollectionStatisticsRequest {
  period: "day" | "week" | "month" | "year";
  from?: string;
  to?: string;
  groupBy?: "category" | "location" | "collector" | "producer";
  collectorId?: string;
  region?: string;
}

// Response
interface CollectionStatisticsResponse {
  period: { from: string; to: string };

  summary: {
    totalEvents: number;
    totalWeightKg: number;
    totalItems: number;
    averageCondition: object;
  };

  breakdown: {
    key: string;
    events: number;
    weightKg: number;
    items?: number;
  }[];

  trends: {
    date: string;
    value: number;
  }[];

  routing: {
    reuse: { count: number; percentage: number };
    refurbishment: { count: number; percentage: number };
    recycling: { count: number; percentage: number };
  };
}
```

---

## 5.5 Processing API

### 5.5.1 Processing Event Endpoints

```typescript
// POST /processing/events - Submit processing event
interface ProcessingEventRequest {
  facilityId: string;
  processType: "receiving" | "sorting" | "disassembly" | "shredding" | "separation" | "refining";

  input: {
    batchIds?: string[];
    deviceIds?: string[];
    fromEventId?: string;        // Link to previous event
    totalWeightKg: number;
    categoryBreakdown?: { category: string; weightKg: number }[];
  };

  output: {
    fractions: OutputFraction[];
    waste?: { type: string; weightKg: number; disposalMethod: string }[];
  };

  processMetrics?: {
    startTime: string;
    endTime: string;
    energyKwh?: number;
    waterLiters?: number;
  };

  qualityControl?: {
    samplingDone: boolean;
    contaminationPercent?: number;
    notes?: string;
  };
}

interface OutputFraction {
  fractionId: string;
  materialType: string;
  weightKg: number;
  purity?: number;
  destination: "internal" | "sold" | "downstream" | "disposal";
  downstreamProcessorId?: string;
}
```

### 5.5.2 Material Recovery Reporting

```typescript
// POST /processing/material-recovery - Submit material recovery data
interface MaterialRecoveryRequest {
  facilityId: string;
  reportingPeriod: { from: string; to: string };

  input: {
    totalWeightKg: number;
    byCategory: { category: string; weightKg: number }[];
    sourceBatches: string[];
  };

  recovery: {
    materials: {
      materialCode: string;
      materialName: string;
      weightKg: number;
      purity: number;
      qualityCertification?: string;
      buyerId?: string;
      revenue?: number;
    }[];

    reusedComponents: {
      type: string;
      quantity: number;
      estimatedValue?: number;
    }[];
  };

  residuals: {
    hazardous: { type: string; weightKg: number; treatment: string }[];
    nonHazardous: { type: string; weightKg: number; disposal: string }[];
  };

  metrics: {
    overallRecoveryRate: number;
    recyclingRate: number;
    reuseRate: number;
    disposalRate: number;
    co2Avoided?: number;
  };
}

// GET /processing/material-recovery?facility={id}&from={date}&to={date}
// GET /processing/material-recovery/summary - Aggregated statistics
```

### 5.5.3 Facility API

```typescript
// POST /facilities - Register facility
interface FacilityRegistration {
  name: string;
  type: "collection" | "processing" | "refurbishment" | "final_treatment";

  location: {
    address: string;
    coordinates: object;
  };

  operator: {
    entityId: string;
    name: string;
  };

  capabilities: {
    processTypes: string[];
    acceptedCategories: string[];
    hazardousHandling: boolean;
    annualCapacityKg: number;
  };

  permits: {
    permitId: string;
    type: string;
    issuingAuthority: string;
    validUntil: string;
  }[];

  certifications: {
    standard: string;          // "R2", "e-Stewards", "ISO14001"
    certificateNumber: string;
    validUntil: string;
  }[];
}

// GET /facilities/{id}
// GET /facilities?type={type}&capability={capability}&region={region}
// PUT /facilities/{id}/certifications - Update certifications
// GET /facilities/{id}/performance - Facility performance metrics
```

---

## 5.6 Compliance API

### 5.6.1 Compliance Report Submission

```typescript
// POST /compliance/reports - Submit compliance report
interface ComplianceReportRequest {
  reportType: "producer_annual" | "processor_annual" | "quarterly" | "ad_hoc";
  jurisdiction: string;
  reportingPeriod: { from: string; to: string };

  entityId: string;
  entityType: "producer" | "collector" | "processor";

  // Type-specific content
  producerData?: ProducerComplianceData;
  processorData?: ProcessorComplianceData;

  certification: {
    certifiedBy: string;
    title: string;
    date: string;
    digitalSignature?: string;
  };

  attachments?: {
    type: string;
    filename: string;
    url: string;
  }[];
}

// Response
interface ComplianceReportResponse {
  reportId: string;
  status: "submitted" | "under_review" | "approved" | "rejected";
  submissionDate: string;
  acknowledgementNumber: string;

  validation: {
    valid: boolean;
    warnings?: string[];
    errors?: string[];
  };

  complianceStatus: {
    overall: "compliant" | "partial" | "non_compliant";
    details: {
      metric: string;
      target: number;
      achieved: number;
      status: string;
    }[];
  };

  nextActions?: {
    action: string;
    dueDate: string;
  }[];
}
```

### 5.6.2 Compliance Verification

```typescript
// GET /compliance/verify/{entityId} - Check entity compliance status
interface ComplianceStatusResponse {
  entityId: string;
  entityName: string;
  entityType: string;

  currentStatus: "compliant" | "conditional" | "non_compliant" | "pending";
  statusDate: string;

  certifications: {
    certification: string;
    status: "active" | "expired" | "suspended";
    validUntil: string;
  }[];

  metrics: {
    period: string;
    collectionRate?: number;
    recoveryRate?: number;
    targetMet: boolean;
  }[];

  issues?: {
    type: string;
    description: string;
    dueDate?: string;
    status: "open" | "resolved";
  }[];

  publicVerificationUrl: string;
}

// POST /compliance/verify/device/{deviceId} - Verify device recycling
// Returns certificate if device lifecycle is closed
```

### 5.6.3 Regulatory Reporting

```typescript
// GET /compliance/reports/generate - Generate regulatory report
interface ReportGenerationRequest {
  reportFormat: "weee_eu" | "california" | "japan_hal" | "generic";
  jurisdiction: string;
  entityId: string;
  reportingPeriod: { from: string; to: string };
  outputFormat: "json" | "xml" | "pdf";
}

// POST /compliance/reports/submit - Submit to regulatory authority
interface RegulatorySubmission {
  reportId: string;
  authority: string;
  submissionMethod: "api" | "portal" | "manual";

  // Returns
  submissionId: string;
  status: "submitted" | "acknowledged" | "processing";
  trackingUrl?: string;
}
```

---

## 5.7 Webhook Integration

### 5.7.1 Event Notifications

```typescript
// Webhook configuration
interface WebhookConfig {
  url: string;
  events: WebhookEvent[];
  secret: string;              // For signature verification
  active: boolean;
  retryPolicy: {
    maxRetries: 3;
    backoffSeconds: [10, 60, 300];
  };
}

type WebhookEvent =
  | "device.registered"
  | "device.collected"
  | "device.processed"
  | "device.lifecycle_closed"
  | "batch.created"
  | "batch.transferred"
  | "material.recovered"
  | "compliance.due"
  | "compliance.submitted"
  | "certification.expiring"
  | "alert.triggered";

// Webhook payload
interface WebhookPayload {
  id: string;
  event: WebhookEvent;
  timestamp: string;
  data: object;
  signature: string;           // HMAC-SHA256
}

// POST /webhooks - Register webhook
// GET /webhooks - List webhooks
// PUT /webhooks/{id} - Update webhook
// DELETE /webhooks/{id} - Remove webhook
// POST /webhooks/{id}/test - Send test event
```

### 5.7.2 Webhook Security

```typescript
// Verify webhook signature
function verifyWebhookSignature(
  payload: string,
  signature: string,
  secret: string
): boolean {
  const computed = crypto
    .createHmac("sha256", secret)
    .update(payload)
    .digest("hex");

  return crypto.timingSafeEqual(
    Buffer.from(signature),
    Buffer.from(computed)
  );
}

// Example webhook handler
app.post("/webhooks/wia", (req, res) => {
  const signature = req.headers["x-wia-signature"];

  if (!verifyWebhookSignature(req.body, signature, WEBHOOK_SECRET)) {
    return res.status(401).send("Invalid signature");
  }

  const event = req.body;

  switch (event.event) {
    case "device.collected":
      handleDeviceCollected(event.data);
      break;
    case "compliance.due":
      handleComplianceDue(event.data);
      break;
    // ... handle other events
  }

  res.status(200).send("OK");
});
```

---

## 5.8 SDK Examples

### 5.8.1 TypeScript SDK

```typescript
// WIA E-Waste SDK usage
import { WiaEwasteClient } from "@wia/ewaste-sdk";

const client = new WiaEwasteClient({
  apiKey: process.env.WIA_API_KEY,
  environment: "production"
});

// Register a device
async function registerDevice() {
  const device = await client.devices.create({
    producer: {
      id: "SAMSUNG-KR",
      name: "Samsung Electronics",
      model: "Galaxy S24"
    },
    category: {
      wiaCategory: "SM",
      weeeCategory: 3
    },
    production: {
      date: "2024-01-15",
      facility: "Samsung Gumi"
    },
    materials: {
      totalWeightKg: 0.187
    }
  });

  console.log(`Registered device: ${device.deviceId}`);
  return device;
}

// Submit collection event
async function submitCollection(deviceId: string) {
  const collection = await client.collections.create({
    collectionPoint: { id: "CP-001", type: "retail" },
    device: { deviceId },
    condition: {
      functional: false,
      physicalCondition: "fair"
    },
    routing: { recommendation: "recycling" }
  });

  return collection;
}

// Check device lifecycle
async function getDeviceHistory(deviceId: string) {
  const history = await client.devices.getLifecycle(deviceId);

  console.log(`Device ${deviceId} events:`);
  history.events.forEach(event => {
    console.log(`  ${event.timestamp}: ${event.eventType}`);
  });

  return history;
}
```

### 5.8.2 Python SDK

```python
from wia_ewaste import WiaClient

client = WiaClient(api_key="wia_live_...")

# Register device
device = client.devices.create(
    producer={"id": "SAMSUNG-KR", "name": "Samsung"},
    category={"wia_category": "SM", "weee_category": 3},
    production={"date": "2024-01-15", "facility": "Gumi"},
    materials={"total_weight_kg": 0.187}
)

print(f"Device ID: {device.device_id}")

# Submit collection
collection = client.collections.create(
    collection_point={"id": "CP-001", "type": "retail"},
    device={"device_id": device.device_id},
    condition={"functional": False, "physical_condition": "fair"},
    routing={"recommendation": "recycling"}
)

# Generate compliance report
report = client.compliance.generate_report(
    entity_id="PRODUCER-001",
    report_format="weee_eu",
    period={"from": "2024-01-01", "to": "2024-12-31"}
)
```

---

## 5.9 Review Questions

### Question 1
Design the OAuth 2.0 scope set for a recycling facility that needs to receive devices from collectors and submit material recovery reports. What scopes are needed and why?

### Question 2
Write the API request to register a batch of 1,000 identical laptop computers from the same production run. Include all required fields.

### Question 3
A collection point wants to receive real-time notifications when devices are routed to their facility. Design the webhook configuration and handler.

### Question 4
Create the API call sequence to track a device from collection through final material recovery, including all status checks.

### Question 5
Design an API endpoint for public device verification that returns recycling certificate information without exposing sensitive business data.

---

## 5.10 Key Takeaways

| API Category | Key Endpoints | Primary Users |
|--------------|---------------|---------------|
| Device Registry | /devices, /devices/{id} | Producers |
| Collection | /collections, /collection-points | Collectors |
| Processing | /processing/events, /material-recovery | Processors |
| Compliance | /compliance/reports, /verify | All, Regulators |
| Webhooks | /webhooks | Integrators |

### API Best Practices
- **Use batch endpoints** for high-volume operations
- **Implement webhooks** for real-time integration
- **Cache responses** where appropriate (TTL varies by endpoint)
- **Handle rate limits** gracefully with exponential backoff
- **Verify signatures** on all webhook payloads

### Next Chapter Preview

Chapter 6 covers the operational protocols for chain of custody, material handling, hazardous substance management, and compliance verification procedures.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
