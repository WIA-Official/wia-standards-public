# WIA-FOOD_SAFETY Specification - PHASE 3: Security, Compliance & Performance

**Version:** 1.0.0
**Last Updated:** 2026-01-11

## Overview

This document defines the security architecture, regulatory compliance requirements, and performance optimization strategies for WIA-FOOD_SAFETY. The system must protect sensitive supply chain data while maintaining transparency for food safety verification.

## Security Architecture

### Multi-Layer Security Model

```
┌─────────────────────────────────────────────┐
│  Layer 5: Application Security              │
│  • Input validation • Output encoding       │
│  • Business logic security                  │
└──────────────────┬──────────────────────────┘
┌──────────────────▼──────────────────────────┐
│  Layer 4: API Security                      │
│  • JWT authentication • API key management  │
│  • Rate limiting • OAuth 2.0                │
└──────────────────┬──────────────────────────┘
┌──────────────────▼──────────────────────────┐
│  Layer 3: Network Security                  │
│  • TLS 1.3 encryption • WAF (Web App FW)    │
│  • DDoS protection • VPN for IoT sensors    │
└──────────────────┬──────────────────────────┘
┌──────────────────▼──────────────────────────┐
│  Layer 2: Data Security                     │
│  • Encryption at rest (AES-256)             │
│  • Encryption in transit (TLS 1.3)          │
│  • Key management (AWS KMS, HashiCorp Vault)│
└──────────────────┬──────────────────────────┘
┌──────────────────▼──────────────────────────┐
│  Layer 1: Blockchain Security               │
│  • Cryptographic hashing (SHA-256)          │
│  • Digital signatures (ECDSA)               │
│  • Consensus mechanism (PoS, Raft)          │
└─────────────────────────────────────────────┘
```

### Authentication & Authorization

#### JWT (JSON Web Token)
**Token Structure:**
```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT"
  },
  "payload": {
    "sub": "user-123456",
    "name": "John Farmer",
    "email": "john@greenvalleyfarms.com",
    "role": "FARMER",
    "companyId": "green-valley-farms",
    "permissions": [
      "product:register",
      "product:read",
      "ccp:record"
    ],
    "iat": 1704988800,
    "exp": 1704992400
  },
  "signature": "..."
}
```

**Token Lifecycle:**
- **Access Token**: 1 hour lifetime, used for API requests
- **Refresh Token**: 7 days lifetime, used to obtain new access tokens
- **Revocation**: Redis-based token blacklist for instant logout

#### Role-Based Access Control (RBAC)

| Role | Permissions | Use Case |
|------|-------------|----------|
| **FARMER** | Register products, record harvest, update CCP | Farm-level data entry |
| **PROCESSOR** | Update processing stage, record CCP, view traceability | Food processing facilities |
| **DISTRIBUTOR** | Update location, record temperature, view traceability | Logistics companies |
| **RETAILER** | View traceability, scan QR codes, consumer queries | Stores, restaurants |
| **INSPECTOR** | View all data, audit trails, compliance reports | FDA, USDA, third-party auditors |
| **ADMIN** | Full access, user management, system configuration | System administrators |
| **CONSUMER** | Scan QR codes, view public product info | End consumers |

**Permission Matrix:**
```typescript
const permissions = {
  FARMER: [
    "product:register",
    "product:read",
    "product:update:own",
    "ccp:record:own",
    "temperature:log:own"
  ],
  INSPECTOR: [
    "product:read:all",
    "ccp:read:all",
    "temperature:read:all",
    "audit:generate",
    "compliance:validate"
  ],
  CONSUMER: [
    "product:read:public",
    "qr:scan",
    "traceability:view:public"
  ]
};
```

### API Security

#### Rate Limiting
```yaml
Rate Limits:
  Standard Tier:
    - 1,000 requests/hour
    - 100 requests/minute
    - Burst: 20 requests/second

  Enterprise Tier:
    - 10,000 requests/hour
    - 500 requests/minute
    - Burst: 50 requests/second

  Public Tier (QR scans):
    - 100 requests/hour per IP
    - 10 requests/minute per IP
```

**Rate Limit Response:**
```json
HTTP/1.1 429 Too Many Requests
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Try again in 45 seconds.",
    "retryAfter": 45,
    "limit": 1000,
    "remaining": 0,
    "resetAt": "2026-01-11T13:00:00Z"
  }
}
```

#### API Key Management
- **Generation**: Cryptographically secure random strings (256 bits)
- **Storage**: Hashed with bcrypt (cost factor: 12)
- **Rotation**: Automated 90-day rotation with 30-day overlap
- **Scope**: Granular permissions per API key

#### Input Validation
```typescript
// Example: Batch ID validation
function validateBatchId(batchId: string): boolean {
  // Must be alphanumeric, 8-20 characters, specific format
  const regex = /^[A-Z]{3,4}-\d{4}-\d{6,12}$/;
  return regex.test(batchId);
}

// Temperature validation
function validateTemperature(temp: number): boolean {
  return temp >= -50 && temp <= 100; // °C, reasonable range
}

// SQL injection prevention
import { escape } from 'sqlstring';
const query = `SELECT * FROM products WHERE batchId = ${escape(batchId)}`;
```

### Data Encryption

#### Encryption at Rest
- **Algorithm**: AES-256-GCM (Galois/Counter Mode)
- **Key Management**: AWS KMS (Key Management Service) or HashiCorp Vault
- **Key Rotation**: Automated 90-day rotation
- **Sensitive Fields**: Supplier pricing, contract terms, personal data

**Encryption Example:**
```typescript
import { createCipheriv, createDecipheriv, randomBytes } from 'crypto';

function encryptSensitiveData(plaintext: string, key: Buffer): string {
  const iv = randomBytes(16);
  const cipher = createCipheriv('aes-256-gcm', key, iv);
  let encrypted = cipher.update(plaintext, 'utf8', 'hex');
  encrypted += cipher.final('hex');
  const authTag = cipher.getAuthTag();

  return JSON.stringify({
    iv: iv.toString('hex'),
    encryptedData: encrypted,
    authTag: authTag.toString('hex')
  });
}
```

#### Encryption in Transit
- **Protocol**: TLS 1.3 (Transport Layer Security)
- **Ciphers**: ECDHE-RSA-AES256-GCM-SHA384 (forward secrecy)
- **Certificates**: Let's Encrypt or commercial CA
- **HSTS**: HTTP Strict Transport Security enabled (max-age: 31536000)

**TLS Configuration:**
```nginx
ssl_protocols TLSv1.3;
ssl_ciphers 'ECDHE-RSA-AES256-GCM-SHA384:ECDHE-RSA-AES128-GCM-SHA256';
ssl_prefer_server_ciphers on;
ssl_session_cache shared:SSL:10m;
ssl_session_timeout 10m;
```

### Blockchain Security

#### Cryptographic Hashing
- **Algorithm**: SHA-256 (Secure Hash Algorithm)
- **Use Cases**: Product IDs, batch IDs, document fingerprints
- **Collision Resistance**: 2^256 possible hashes (practically impossible)

**Hash Generation:**
```typescript
import { createHash } from 'crypto';

function generateBatchId(product: FoodProduct): string {
  const data = `${product.farmName}:${product.harvestDate}:${product.productName}`;
  const hash = createHash('sha256').update(data).digest('hex');
  return `BATCH-${hash.substring(0, 12)}`;
}
```

#### Digital Signatures
- **Algorithm**: ECDSA (Elliptic Curve Digital Signature Algorithm)
- **Curve**: secp256k1 (Ethereum standard)
- **Use Cases**: Product registration, CCP records, recall notices

**Signature Verification:**
```typescript
import { verifyMessage } from 'ethers';

function verifyProductSignature(
  message: string,
  signature: string,
  expectedAddress: string
): boolean {
  const recoveredAddress = verifyMessage(message, signature);
  return recoveredAddress.toLowerCase() === expectedAddress.toLowerCase();
}
```

#### Smart Contract Security
**Security Best Practices:**
1. **Reentrancy Protection**: Use OpenZeppelin's ReentrancyGuard
2. **Access Control**: Role-based permissions (Ownable, AccessControl)
3. **Integer Overflow**: Use Solidity 0.8+ (built-in overflow checks)
4. **External Call Safety**: Check return values, limit gas
5. **Audit**: Third-party security audits (ConsenSys Diligence, Trail of Bits)

**Example Secure Contract:**
```solidity
pragma solidity ^0.8.0;

import "@openzeppelin/contracts/security/ReentrancyGuard.sol";
import "@openzeppelin/contracts/access/AccessControl.sol";

contract FoodTraceability is ReentrancyGuard, AccessControl {
    bytes32 public constant FARMER_ROLE = keccak256("FARMER_ROLE");
    bytes32 public constant INSPECTOR_ROLE = keccak256("INSPECTOR_ROLE");

    mapping(bytes32 => FoodProduct) private products;

    function registerProduct(
        bytes32 batchId,
        string memory productName
    ) external onlyRole(FARMER_ROLE) nonReentrant {
        require(products[batchId].timestamp == 0, "Product already exists");
        products[batchId] = FoodProduct({
            batchId: batchId,
            productName: productName,
            farmer: msg.sender,
            timestamp: block.timestamp,
            isRecalled: false
        });
        emit ProductRegistered(batchId, msg.sender, productName, block.timestamp);
    }
}
```

### Privacy & Data Protection

#### GDPR Compliance (EU General Data Protection Regulation)
- **Right to Access**: API endpoint to retrieve personal data
- **Right to Erasure**: Mark records as "deleted" (blockchain immutability constraint)
- **Data Minimization**: Only store necessary data on-chain
- **Consent Management**: Explicit opt-in for personal data collection

**Personal Data Handling:**
```typescript
// Store personal data off-chain, only hash on-chain
const personalData = {
  name: "John Farmer",
  email: "john@example.com",
  phone: "+1-555-0123"
};

const personalDataHash = createHash('sha256')
  .update(JSON.stringify(personalData))
  .digest('hex');

// Store hash on blockchain
await contract.registerFarmer(farmerId, personalDataHash);

// Store actual data in encrypted database
await database.storeFarmer(farmerId, encrypt(personalData));
```

#### Confidential Data (Trade Secrets)
- **Problem**: Suppliers don't want to reveal pricing, contracts
- **Solution**: Hyperledger Fabric Private Data Collections

**Private Data Example:**
```go
// Hyperledger Fabric chaincode
type SupplierContract struct {
  SupplierID string `json:"supplierId"`
  ProductID  string `json:"productId"`
  Price      float64 `json:"price"`      // PRIVATE
  Volume     int     `json:"volume"`     // PRIVATE
  StartDate  string  `json:"startDate"`  // PUBLIC
  EndDate    string  `json:"endDate"`    // PUBLIC
}

// Store private data
err := stub.PutPrivateData("supplierContracts", contractId, contractJSON)
```

### IoT Sensor Security

#### Device Authentication
- **Method**: X.509 certificates per sensor
- **Provisioning**: Secure factory programming
- **Renewal**: Automatic certificate renewal (Let's Encrypt ACME protocol)

#### Communication Security
- **Protocol**: LoRaWAN with AES-128 encryption
- **Network Server**: Authenticated via AppKey
- **Replay Protection**: Frame counters (monotonically increasing)

**LoRaWAN Security:**
```yaml
Security Keys:
  AppKey:    128-bit AES key (device-specific)
  NwkSKey:   Network Session Key (per-session)
  AppSKey:   Application Session Key (per-session)

Message Integrity:
  MIC:       32-bit Message Integrity Code
  Algorithm: AES-128-CMAC
```

#### Sensor Tampering Detection
- **GPS Verification**: Alert if sensor moves unexpectedly
- **Calibration Drift**: Alert if readings deviate from expected range
- **Battery Monitoring**: Alert if battery drains faster than expected (potential attack)

## Regulatory Compliance

### FDA FSMA (Food Safety Modernization Act)

#### Traceability Rule (Final Rule: 2022)
**Key Data Elements (KDEs):**
1. **Traceability Lot Code**: Unique identifier (batch ID)
2. **Product Description**: Name, variety, packaging
3. **Quantity**: Amount shipped/received
4. **Location**: Physical address (farm, facility)
5. **Date**: Harvest, shipping, receiving dates
6. **Reference Document**: Invoice, BOL (Bill of Lading)

**WIA-FOOD_SAFETY Implementation:**
```typescript
interface FSMATraceabilityRecord {
  // Required KDEs
  traceabilityLotCode: string;      // = batchId
  productDescription: string;
  quantity: { value: number; unit: string };
  location: Location;
  date: {
    harvest?: DateTime;
    shipping?: DateTime;
    receiving?: DateTime;
  };
  referenceDocument: {
    type: "INVOICE" | "BOL" | "PURCHASE_ORDER";
    number: string;
    url: string;                    // Stored on IPFS
  };

  // Optional but recommended
  grower: string;
  shipper: string;
  receiver: string;
  blockchainTxHash: string;         // Immutable proof
}
```

#### Preventive Controls Rule
**Requirements:**
1. Written food safety plan
2. Hazard analysis
3. Preventive controls (CCPs)
4. Monitoring procedures
5. Corrective actions
6. Verification activities
7. Record-keeping

**Automated Compliance:**
- System automatically generates food safety plan from HACCP template
- Real-time CCP monitoring via IoT sensors
- Corrective action workflows triggered automatically
- Blockchain records provide tamper-proof audit trail

### ISO 22000:2018 Certification

**Audit Readiness Features:**
1. **Document Control**: All procedures version-controlled on IPFS
2. **Training Records**: Employee training completion tracked
3. **Internal Audits**: Automated monthly internal audit checklists
4. **Management Review**: KPI dashboard for quarterly reviews
5. **Traceability Testing**: One-up/one-back verification in < 2.2 seconds

**Audit Trail Export:**
```typescript
// Generate audit package for ISO 22000 certification
async function generateAuditPackage(
  companyId: string,
  startDate: DateTime,
  endDate: DateTime
): Promise<AuditPackage> {
  return {
    companyInfo: await getCompanyInfo(companyId),
    foodSafetyPolicy: await getDocument("food-safety-policy.pdf"),
    haccpPlans: await getHACCPPlans(companyId),
    ccpRecords: await getCCPRecords(companyId, startDate, endDate),
    temperatureLogs: await getTemperatureLogs(companyId, startDate, endDate),
    correctiveActions: await getCorrectiveActions(companyId, startDate, endDate),
    calibrationRecords: await getSensorCalibrations(companyId),
    trainingRecords: await getTrainingRecords(companyId),
    blockchainProof: await getBlockchainProof(companyId, startDate, endDate)
  };
}
```

### USDA Organic Certification

**Organic Traceability Requirements:**
1. Audit trail from certified organic farm to retail
2. Prevention of commingling with non-organic products
3. Documentation of organic status at each stage
4. Annual inspection by USDA-accredited certifier

**Blockchain Implementation:**
```solidity
struct OrganicCertification {
    string certificationNumber;     // "USDA-ORG-123456"
    address certifyingAgent;        // Blockchain address
    uint256 issueDate;
    uint256 expirationDate;
    bool isValid;
}

mapping(bytes32 => OrganicCertification) public organicCerts;

function verifyOrganic(bytes32 batchId) external view returns (bool) {
    OrganicCertification memory cert = organicCerts[batchId];
    return cert.isValid && cert.expirationDate > block.timestamp;
}
```

### EU Regulations

#### General Food Law (EC) No 178/2002
- **Traceability**: One-up/one-back (Article 18)
- **Food Safety**: Responsibility of food business operators
- **Rapid Alert System**: RASFF (Rapid Alert System for Food and Feed)

#### HACCP Regulation (EC) No 852/2004
- Mandatory HACCP-based procedures for food businesses
- Hazard analysis and CCP monitoring required
- Record-keeping for official controls

**WIA-FOOD_SAFETY meets all EU requirements** with blockchain-verified traceability and automated HACCP compliance.

## Performance Optimization

### Database Optimization

#### Indexing Strategy
```sql
-- PostgreSQL indexes for fast queries
CREATE INDEX idx_products_batch_id ON products(batch_id);
CREATE INDEX idx_products_company_id ON products(company_id);
CREATE INDEX idx_products_status ON products(status);
CREATE INDEX idx_products_created_at ON products(created_at DESC);

-- TimescaleDB hypertable for time-series sensor data
SELECT create_hypertable('temperature_logs', 'timestamp');
CREATE INDEX idx_temp_logs_batch_id ON temperature_logs(batch_id);
CREATE INDEX idx_temp_logs_sensor_id ON temperature_logs(sensor_id);

-- Composite index for common query pattern
CREATE INDEX idx_temp_logs_batch_timestamp
  ON temperature_logs(batch_id, timestamp DESC);
```

#### Query Optimization
```typescript
// Bad: N+1 query problem
for (const product of products) {
  product.temperatureLogs = await db.query(
    'SELECT * FROM temperature_logs WHERE batch_id = $1',
    [product.batchId]
  );
}

// Good: Single query with JOIN
const productsWithLogs = await db.query(`
  SELECT p.*,
         json_agg(tl.*) as temperature_logs
  FROM products p
  LEFT JOIN temperature_logs tl ON p.batch_id = tl.batch_id
  WHERE p.company_id = $1
  GROUP BY p.id
`, [companyId]);
```

#### Caching Strategy
```yaml
Cache Layers:
  L1 - Application Cache (Node.js):
    - Library: node-cache
    - TTL: 5 minutes
    - Use: Frequently accessed data

  L2 - Redis Cache:
    - TTL: 1 hour (product data), 5 minutes (temperature data)
    - Use: API responses, session data
    - Eviction: LRU (Least Recently Used)

  L3 - CDN (CloudFront, Cloudflare):
    - TTL: 24 hours
    - Use: Static assets, public product info, QR code images
```

**Redis Caching Example:**
```typescript
import Redis from 'ioredis';
const redis = new Redis();

async function getProduct(batchId: string): Promise<FoodProduct> {
  // Check cache first
  const cached = await redis.get(`product:${batchId}`);
  if (cached) {
    return JSON.parse(cached);
  }

  // Cache miss: query database
  const product = await db.products.findOne({ batchId });

  // Store in cache (1 hour TTL)
  await redis.setex(`product:${batchId}`, 3600, JSON.stringify(product));

  return product;
}
```

### API Performance

#### Response Time Targets
```yaml
Performance SLA:
  P50 (Median):      < 50ms
  P95:               < 200ms
  P99:               < 500ms
  P99.9:             < 2s

Traceability Query:
  Target:            < 2.2 seconds (end-to-end)
  Blockchain Query:  < 1 second
  Database Query:    < 500ms
  API Response:      < 500ms
```

#### Pagination & Limiting
```typescript
// Limit result sets to prevent memory issues
interface PaginationParams {
  page: number;        // Default: 1
  limit: number;       // Default: 20, Max: 100
  sortBy?: string;     // Default: 'createdAt'
  sortOrder?: 'asc' | 'desc';  // Default: 'desc'
}

// Cursor-based pagination for large datasets
interface CursorPaginationParams {
  cursor?: string;     // Opaque cursor token
  limit: number;       // Max: 100
}

// Example response
{
  "data": [...],
  "pagination": {
    "page": 1,
    "limit": 20,
    "totalPages": 50,
    "totalRecords": 1000,
    "nextCursor": "eyJpZCI6MTIzNDU2fQ=="
  }
}
```

#### Async Processing
```typescript
// Use message queues for heavy operations
import Bull from 'bull';

const recallQueue = new Bull('recall-processing');

// Producer: Initiate recall (returns immediately)
app.post('/recalls/initiate', async (req, res) => {
  const job = await recallQueue.add({
    batchIds: req.body.batchIds,
    reason: req.body.reason
  });

  res.json({
    success: true,
    jobId: job.id,
    status: "PROCESSING"
  });
});

// Consumer: Process recall in background
recallQueue.process(async (job) => {
  const { batchIds, reason } = job.data;

  // 1. Identify all downstream recipients
  const recipients = await traceForwardSupplyChain(batchIds);

  // 2. Send notifications (SMS, email, push)
  await notificationService.sendRecallAlerts(recipients, reason);

  // 3. Log on blockchain
  await blockchainService.recordRecall(batchIds, reason);

  // 4. Generate FDA report
  await generateFDARecallReport(batchIds, reason);
});
```

### Blockchain Performance

#### Layer-2 Scaling
**Problem**: Ethereum mainnet is expensive ($1-5 per transaction) and slow (15 TPS)

**Solution**: Polygon (Matic) Layer-2
- **Speed**: 1000+ TPS
- **Cost**: $0.01-0.05 per transaction
- **Finality**: ~2 seconds
- **Security**: Secured by Ethereum mainnet

**Transaction Batching:**
```typescript
// Instead of 1 transaction per temperature reading (expensive)
// Batch 10 readings into 1 transaction

const tempReadings: TemperatureReading[] = [];

// Collect readings for 10 minutes
setInterval(async () => {
  if (tempReadings.length >= 10) {
    // Submit batch to blockchain
    const merkleRoot = calculateMerkleRoot(tempReadings);
    await contract.logTemperatureBatch(batchId, merkleRoot);

    // Store individual readings off-chain (database)
    await db.temperatureLogs.insertMany(tempReadings);

    tempReadings.length = 0; // Clear array
  }
}, 60000); // Every 1 minute
```

#### IPFS for Large Documents
**Problem**: Storing PDFs, photos on blockchain is expensive

**Solution**: Store on IPFS, only store hash on-chain
```typescript
import IPFS from 'ipfs-http-client';
const ipfs = IPFS.create({ url: 'https://ipfs.infura.io:5001' });

// Upload lab report PDF to IPFS
async function uploadLabReport(pdfBuffer: Buffer): Promise<string> {
  const { cid } = await ipfs.add(pdfBuffer);
  const ipfsUrl = `https://ipfs.io/ipfs/${cid}`;

  // Store only the CID (hash) on blockchain
  await contract.recordLabTest(batchId, cid.toString());

  return ipfsUrl;
}
```

### Monitoring & Alerting

#### Performance Metrics
```yaml
Key Metrics (Prometheus + Grafana):
  API Metrics:
    - Request rate (requests/second)
    - Response time (P50, P95, P99)
    - Error rate (4xx, 5xx)
    - Throughput (MB/s)

  Blockchain Metrics:
    - Transaction success rate
    - Gas price (Gwei)
    - Confirmation time
    - Failed transactions

  Database Metrics:
    - Query time (P50, P95, P99)
    - Connection pool utilization
    - Cache hit rate
    - Disk I/O

  IoT Metrics:
    - Sensor uptime
    - Data transmission success rate
    - Battery level
    - Signal strength (RSSI)
```

#### Alerting Rules
```yaml
Alerts (PagerDuty, OpsGenie):
  Critical (Page immediately):
    - API error rate > 5% for 5 minutes
    - Database connection pool exhausted
    - Blockchain node unreachable
    - CCP violation detected

  Warning (Slack notification):
    - API P99 latency > 2 seconds for 10 minutes
    - Cache hit rate < 70%
    - Sensor battery < 20%
    - Disk usage > 80%

  Info (Log only):
    - New product registered
    - Recall completed successfully
    - Daily backup completed
```

---

**© 2026 WIA | 弘익人間 (Benefit All Humanity)**
