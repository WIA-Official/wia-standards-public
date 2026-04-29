# WIA-FINTECH_INNOVATION Specification - PHASE 3: Security, Compliance & Performance

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2026-01-11

## Table of Contents
1. [Security Architecture](#security-architecture)
2. [Authentication & Authorization](#authentication--authorization)
3. [Regulatory Compliance](#regulatory-compliance)
4. [Privacy & Data Protection](#privacy--data-protection)
5. [Performance & Scalability](#performance--scalability)
6. [Reliability & Resilience](#reliability--resilience)

---

## 1. Security Architecture

### 1.1 Defense in Depth Strategy

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 7: Application Security (WAF, DDoS Protection)       │
├─────────────────────────────────────────────────────────────┤
│  Layer 6: API Gateway (Rate Limiting, Auth, Throttling)     │
├─────────────────────────────────────────────────────────────┤
│  Layer 5: Identity & Access (OAuth 2.1, mTLS, FAPI)        │
├─────────────────────────────────────────────────────────────┤
│  Layer 4: Transport Security (TLS 1.3, Certificate Pinning) │
├─────────────────────────────────────────────────────────────┤
│  Layer 3: Network Security (VPC, Security Groups, Firewall) │
├─────────────────────────────────────────────────────────────┤
│  Layer 2: Data Security (Encryption at Rest, Tokenization)  │
├─────────────────────────────────────────────────────────────┤
│  Layer 1: Infrastructure Security (HSM, Secure Boot, TPM)   │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Cryptographic Standards

**Encryption at Rest:**
- **Algorithm**: AES-256-GCM
- **Key Management**: AWS KMS, Google Cloud KMS, Azure Key Vault
- **Key Rotation**: Automatic 90-day rotation
- **Envelope Encryption**: Data Encryption Keys (DEK) encrypted by Key Encryption Keys (KEK)

**Encryption in Transit:**
- **Protocol**: TLS 1.3 (mandatory), TLS 1.2 (fallback only)
- **Cipher Suites** (TLS 1.3):
  ```
  TLS_AES_256_GCM_SHA384
  TLS_CHACHA20_POLY1305_SHA256
  TLS_AES_128_GCM_SHA256
  ```
- **Certificate Authority**: DigiCert, Let's Encrypt (ACME protocol)
- **Certificate Pinning**: Mobile apps pin intermediate CA certificates
- **Perfect Forward Secrecy (PFS)**: ECDHE key exchange

**Hashing & Signing:**
- **Payment Signatures**: HMAC-SHA256, RSA-PSS (4096-bit)
- **Password Hashing**: Argon2id (memory-hard function)
  ```
  Argon2id parameters:
  - Memory: 64 MiB
  - Iterations: 3
  - Parallelism: 4
  - Output: 32 bytes
  ```
- **API Request Signing**: Ed25519 (Edwards-curve Digital Signature Algorithm)

### 1.3 PCI DSS Compliance

**PCI DSS v4.0.1 Requirements:**

| Requirement | Implementation |
|-------------|----------------|
| **1. Network Security** | Firewall rules, network segmentation, DMZ |
| **2. Default Credentials** | Mandatory password change, MFA |
| **3. Stored Cardholder Data** | Tokenization (never store PAN), vault |
| **4. Encryption in Transit** | TLS 1.3, certificate pinning |
| **5. Malware Protection** | ClamAV, YARA rules, sandboxing |
| **6. Secure Systems** | Patch management, vulnerability scanning |
| **7. Access Control** | RBAC, principle of least privilege |
| **8. Identification** | Unique user IDs, MFA, SSO |
| **9. Physical Access** | Biometric access, video surveillance |
| **10. Logging & Monitoring** | Centralized logging (Splunk, ELK) |
| **11. Security Testing** | Quarterly ASV scans, annual penetration test |
| **12. Security Policy** | InfoSec policy, incident response plan |

**Tokenization Architecture:**
```
Card Number (PAN): 4111 1111 1111 1111
         ↓
  Tokenization Vault
         ↓
Token: tok_visa_7k2m8n4p9q1s
         ↓
   Stored in Database

# Detokenization (authorized systems only)
Token → Vault → Original PAN (for payment processing)
```

**PCI Scope Reduction:**
- **Zero PAN Storage**: All card data tokenized immediately
- **Isolated Cardholder Data Environment (CDE)**
- **Out-of-Scope Systems**: Application servers, databases (token only)
- **Third-Party Processors**: Stripe, Adyen (PCI Level 1 certified)

---

## 2. Authentication & Authorization

### 2.1 OAuth 2.1 with PKCE

**Authorization Code Flow (Recommended):**
```
┌────────┐                                      ┌──────────────┐
│ Client │                                      │ Auth Server  │
└───┬────┘                                      └──────┬───────┘
    │                                                  │
    │ 1. Generate code_verifier + code_challenge      │
    │                                                  │
    │ 2. GET /authorize?                              │
    │    response_type=code&                          │
    │    client_id=xxx&                               │
    │    redirect_uri=xxx&                            │
    │    scope=payments accounts&                     │
    │    code_challenge=xxx&                          │
    │    code_challenge_method=S256                   │
    ├─────────────────────────────────────────────────>│
    │                                                  │
    │ 3. User authenticates + consents                │
    │                                                  │
    │ 4. Redirect: /callback?code=AUTH_CODE           │
    │<─────────────────────────────────────────────────┤
    │                                                  │
    │ 5. POST /token                                  │
    │    grant_type=authorization_code&               │
    │    code=AUTH_CODE&                              │
    │    redirect_uri=xxx&                            │
    │    code_verifier=xxx                            │
    ├─────────────────────────────────────────────────>│
    │                                                  │
    │ 6. Response:                                    │
    │    {                                            │
    │      access_token: "...",                       │
    │      refresh_token: "...",                      │
    │      expires_in: 3600,                          │
    │      token_type: "Bearer"                       │
    │    }                                            │
    │<─────────────────────────────────────────────────┤
    │                                                  │
    │ 7. API Request:                                 │
    │    Authorization: Bearer {access_token}         │
    │                                                  │
```

**PKCE (Proof Key for Code Exchange):**
```javascript
// Step 1: Generate code verifier (43-128 characters)
const codeVerifier = base64UrlEncode(randomBytes(32));
// Example: "dBjftJeZ4CVP-mB92K27uhbUJU1p1r_wW1gFWFOEjXk"

// Step 2: Generate code challenge
const codeChallenge = base64UrlEncode(sha256(codeVerifier));
// Example: "E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGJSstw-cM"

// Step 3: Authorization request includes code_challenge
// Step 4: Token request includes code_verifier (server validates)
```

**Token Security:**
- **Access Token Lifetime**: 1 hour (short-lived)
- **Refresh Token Lifetime**: 90 days (with rotation)
- **Token Binding**: Tokens bound to client via DPoP (Demonstrating Proof of Possession)
- **Token Storage**: HttpOnly, Secure, SameSite cookies (web), Keychain/Keystore (mobile)

### 2.2 Strong Customer Authentication (SCA)

**PSD2 SCA Requirements (Two-Factor Authentication):**
1. **Knowledge**: Something you know (password, PIN)
2. **Possession**: Something you have (phone, hardware token)
3. **Inherence**: Something you are (fingerprint, face ID)

**SCA Exemptions:**
- **Low Value**: Transactions < €30
- **Trusted Beneficiaries**: User-whitelisted payees
- **Recurring Payments**: After initial SCA
- **Secure Corporate Payments**: B2B transactions
- **Transaction Risk Analysis (TRA)**: Low-risk transactions (< €500, fraud rate < 0.13%)

**Implementation:**
```javascript
// SCA Challenge Flow
async function initiateSCAChallenge(paymentId, scaMethod) {
  const challenge = await api.post('/v1/sca/challenge', {
    payment_id: paymentId,
    method: scaMethod  // 'sms', 'push', 'biometric', 'hardware_token'
  });

  // Example response
  return {
    challenge_id: 'sca_chg_abc123',
    method: 'push',
    status: 'pending',
    expires_at: '2026-01-11T17:35:45Z',
    push_notification_sent: true
  };
}

// Verify SCA Response
async function verifySCAResponse(challengeId, responseCode) {
  const result = await api.post('/v1/sca/verify', {
    challenge_id: challengeId,
    response_code: responseCode,
    device_fingerprint: getDeviceFingerprint()
  });

  return {
    verified: true,
    authentication_value: 'AAIBBZIkFlIAAAAAUUAAAAAAAAA=',  // CAVV
    eci: '05',  // Electronic Commerce Indicator
    transaction_id: 'c3RyaXBlX3RyYW5zYWN0aW9uX2lk'
  };
}
```

**Dynamic Linking:**
```
# Authentication code cryptographically bound to transaction
SCA_CODE = HMAC-SHA256(
  key: user_secret_key,
  message: payment_amount || payee_account || timestamp
)

# Example
Amount: $250.00
Payee: IBAN GB29 NWBK 6016 1331 9268 19
Timestamp: 2026-01-11T17:30:45Z
SCA Code: 827491 (6-digit TOTP bound to transaction)
```

### 2.3 Mutual TLS (mTLS) for Open Banking

**Certificate-Based Authentication:**
```
┌─────────┐                           ┌──────────────┐
│  TPP    │                           │  Bank API    │
│ (Client)│                           │   Server     │
└────┬────┘                           └──────┬───────┘
     │                                       │
     │ 1. TLS ClientHello                   │
     │    (Client Certificate Offered)       │
     ├──────────────────────────────────────>│
     │                                       │
     │ 2. TLS ServerHello                   │
     │    (Server Certificate)               │
     │    (Request Client Certificate)       │
     │<──────────────────────────────────────┤
     │                                       │
     │ 3. Client Certificate                 │
     │    (Signed by eIDAS QTSP)            │
     ├──────────────────────────────────────>│
     │                                       │
     │ 4. Certificate Validation:            │
     │    - Check QTSP signature            │
     │    - Verify not revoked (OCSP)       │
     │    - Check authorization number       │
     │                                       │
     │ 5. TLS Handshake Complete            │
     │<──────────────────────────────────────┤
     │                                       │
     │ 6. API Request (mutually authenticated)│
     ├──────────────────────────────────────>│
```

**eIDAS Qualified Trust Service Provider (QTSP) Certificates:**
- **Required for EU Open Banking**: PSD2 compliance
- **Providers**: Aruba PEC, InfoCert, GlobalSign, DigiCert
- **Certificate Types**:
  - QWAC (Qualified Website Authentication Certificate)
  - QSealC (Qualified Seal Certificate)
- **Organization Identifier**: PSD2 authorization number embedded in certificate

**Certificate Revocation:**
- **OCSP (Online Certificate Status Protocol)**: Real-time revocation checks
- **CRL (Certificate Revocation List)**: Fallback mechanism
- **OCSP Stapling**: Server provides OCSP response (reduces latency)

---

## 3. Regulatory Compliance

### 3.1 PSD2 (EU Payment Services Directive 2)

**Technical Standards (RTS):**
- **Strong Customer Authentication**: Two-factor authentication required
- **Secure Communication**: FAPI security profile, mTLS
- **Common API Standards**: Berlin Group, STET, Polish API, UK Open Banking

**Key Obligations:**
```
For Account Servicing Payment Service Providers (ASPSPs):
1. Provide dedicated interface (API) for TPPs
2. Implement SCA for customer authentication
3. Allow Account Information Service Providers (AISPs) access
4. Allow Payment Initiation Service Providers (PISPs) access
5. 99.5% uptime requirement for production APIs
6. Fallback mechanism if API unavailable

For Third-Party Providers (TPPs):
1. Register with national competent authority
2. Obtain eIDAS certificates (QWAC, QSealC)
3. Implement mTLS for API communication
4. Do not store PSU credentials
5. Display clear consent screens
```

**Access-To-Account (XS2A) Framework:**
```typescript
interface ConsentRequest {
  access: {
    accounts: Array<{
      iban: string;
      currency?: string;
    }>;
    balances: Array<{ iban: string }>;
    transactions: Array<{ iban: string }>;
  };
  recurringIndicator: boolean;  // One-time or recurring access
  validUntil: string;  // ISO 8601 date (max 90 days)
  frequencyPerDay: number;  // Max 4 for recurring
  combinedServiceIndicator: boolean;  // AIS + PIS
}
```

### 3.2 US Personal Financial Data Rights (Section 1033)

**CFPB Final Rule (October 2024):**

**Phased Implementation:**
| Institution Type | Compliance Deadline |
|------------------|---------------------|
| Large Banks (> $500B assets) | April 2026 |
| Mid-Size ($50B - $500B) | April 2027 |
| Small ($850M - $50B) | April 2029 |
| Non-Bank Fintechs | April 2030 |

**Core Requirements:**
1. **Data Access**: Share consumer-authorized financial data at no charge
2. **Standardized APIs**: Technical specifications by FDX
3. **Developer Access**: Qualified third parties can access data
4. **Revocable Consent**: Consumers can revoke authorization anytime
5. **Data Minimization**: Only share data necessary for service
6. **Retention Limits**: Delete data when no longer needed

**FDX API Standards:**
```javascript
// Example: Account retrieval (FDX 5.1)
GET /fdx/v5/accounts
Authorization: Bearer {access_token}
Fdx-Version: 5.1

Response:
{
  "accounts": [
    {
      "accountId": "acc_123456",
      "accountType": "DEPOSIT_ACCOUNT",
      "accountNumberDisplay": "****1234",
      "nickname": "My Checking",
      "balanceAsOf": "2026-01-11T17:30:45Z",
      "currentBalance": 2543.21,
      "availableBalance": 2543.21,
      "currency": "USD"
    }
  ]
}
```

**Security Requirements:**
- OAuth 2.1 with PKCE
- Redirect URIs must use HTTPS
- Token expiration: access tokens ≤ 10 minutes
- Refresh tokens require user re-authentication after 1 year

### 3.3 GDPR (General Data Protection Regulation)

**Personal Data Processing:**

**Legal Basis:**
- **Consent**: Explicit, freely given, specific, informed
- **Contract**: Necessary for contract performance (e.g., payment processing)
- **Legal Obligation**: AML/KYC compliance
- **Legitimate Interest**: Fraud prevention (with balancing test)

**Data Subject Rights:**
```typescript
interface DataSubjectRights {
  right_to_access: () => PersonalData;  // Art. 15
  right_to_rectification: (corrections: any) => void;  // Art. 16
  right_to_erasure: () => void;  // Art. 17 ("right to be forgotten")
  right_to_restrict_processing: () => void;  // Art. 18
  right_to_data_portability: () => StructuredData;  // Art. 20
  right_to_object: () => void;  // Art. 21
  right_not_subject_to_automated_decisions: () => void;  // Art. 22
}
```

**Data Protection by Design:**
- **Pseudonymization**: Separate personal identifiers from data
- **Encryption**: AES-256 for data at rest, TLS 1.3 in transit
- **Access Controls**: RBAC, principle of least privilege
- **Data Minimization**: Collect only necessary data
- **Retention Limits**:
  - Transaction data: 7 years (AML compliance)
  - Marketing consent: 2 years inactive
  - Logs: 90 days

**Data Breach Notification:**
- **To Supervisory Authority**: Within 72 hours of awareness
- **To Data Subjects**: Without undue delay (if high risk)
- **Documentation**: Nature, consequences, measures taken

### 3.4 KYC/AML Compliance

**Know Your Customer (KYC):**
```javascript
const kycVerification = {
  identity_verification: {
    document_types: ['passport', 'drivers_license', 'national_id'],
    verification_methods: [
      'onfido',  // ID document + selfie
      'jumio',
      'veriff'
    ],
    liveness_detection: true,  // Prevent spoofing
    document_expiry_check: true
  },
  address_verification: {
    methods: ['utility_bill', 'bank_statement', 'tax_document'],
    max_age_days: 90,
    verification_service: 'trulioo'
  },
  pep_screening: {
    // Politically Exposed Persons
    databases: ['world-check', 'dow_jones', 'refinitiv'],
    continuous_monitoring: true
  },
  sanctions_screening: {
    lists: [
      'OFAC SDN',  // US Treasury
      'UN Consolidated List',
      'EU Sanctions',
      'UK HMT'
    ],
    real_time: true
  },
  adverse_media_screening: {
    sources: ['news', 'regulatory_actions', 'court_records'],
    risk_categories: ['fraud', 'money_laundering', 'terrorism']
  }
};
```

**Anti-Money Laundering (AML):**
- **Customer Due Diligence (CDD)**: Standard risk assessment
- **Enhanced Due Diligence (EDD)**: High-risk customers (PEPs, high-value)
- **Suspicious Activity Reports (SARs)**: File with FinCEN (US) or FIU (EU)
- **Transaction Monitoring Rules**:
  ```
  Rule 1: Structuring - Multiple transactions < $10,000 to avoid CTR
  Rule 2: Rapid Movement - Funds in/out within 24 hours
  Rule 3: Round Dollar - Unusual round amounts ($10,000, $50,000)
  Rule 4: High-Risk Jurisdictions - Transactions to/from FATF blacklist
  Rule 5: Velocity - Transaction count exceeds baseline by 200%
  ```

---

## 4. Privacy & Data Protection

### 4.1 Data Classification

| Classification | Examples | Protection Requirements |
|----------------|----------|-------------------------|
| **Critical** | Card PAN, CVV, PIN | Tokenization, HSM storage, PCI DSS SAQ-D |
| **Sensitive** | SSN, bank account, passport | Encryption at rest/transit, RBAC, audit logs |
| **Confidential** | Email, phone, address | Encryption, access controls |
| **Internal** | Transaction IDs, metadata | Access controls |
| **Public** | API documentation | None |

### 4.2 Privacy-Enhancing Technologies

**Differential Privacy:**
```python
# Example: Aggregate BNPL statistics without revealing individual data
def compute_average_bnpl_amount_with_dp(amounts, epsilon=1.0):
    """
    Compute average with differential privacy guarantee.
    epsilon: Privacy budget (lower = more private)
    """
    true_average = sum(amounts) / len(amounts)

    # Laplace noise for differential privacy
    sensitivity = (max(amounts) - min(amounts)) / len(amounts)
    noise = np.random.laplace(0, sensitivity / epsilon)

    noisy_average = true_average + noise
    return noisy_average

# Usage
average_bnpl = compute_average_bnpl_amount_with_dp(
    amounts=[299, 599, 1299, 450, 899],
    epsilon=1.0
)
# Result: 709.2 (true: 709.2, noise prevents exact reconstruction)
```

**Homomorphic Encryption (Future):**
```
# Compute on encrypted data without decryption
encrypted_balance_1 = encrypt(5000)
encrypted_balance_2 = encrypt(3000)

# Server computes on encrypted data
encrypted_total = add(encrypted_balance_1, encrypted_balance_2)

# Client decrypts result
total = decrypt(encrypted_total)  # = 8000
```

### 4.3 Consent Management

**Granular Consent:**
```typescript
interface ConsentPreferences {
  user_id: string;
  consents: {
    essential: {
      payment_processing: true;  // Required for service
      security: true;
    };
    functional: {
      save_payment_methods: boolean;
      transaction_history: boolean;
      auto_pay: boolean;
    };
    analytics: {
      usage_statistics: boolean;
      performance_monitoring: boolean;
    };
    marketing: {
      email: boolean;
      sms: boolean;
      push_notifications: boolean;
      personalized_offers: boolean;
    };
    third_party: {
      credit_bureaus: boolean;
      open_banking_data_sharing: boolean;
      affiliate_partners: boolean;
    };
  };
  consent_date: string;
  ip_address: string;
  user_agent: string;
  consent_text_version: string;  // Audit trail
}
```

**Consent Audit Trail:**
```sql
CREATE TABLE consent_audit_log (
  id BIGSERIAL PRIMARY KEY,
  user_id UUID NOT NULL,
  action VARCHAR(50) NOT NULL,  -- 'granted', 'revoked', 'modified'
  consent_type VARCHAR(100) NOT NULL,
  previous_value JSONB,
  new_value JSONB,
  ip_address INET,
  user_agent TEXT,
  timestamp TIMESTAMP DEFAULT NOW(),
  INDEX idx_user_timestamp (user_id, timestamp DESC)
);
```

---

## 5. Performance & Scalability

### 5.1 Performance Targets

| Operation | Target Latency (P50) | P95 | P99 |
|-----------|---------------------|-----|-----|
| Payment Authorization | 150ms | 300ms | 500ms |
| Card Tokenization | 80ms | 150ms | 250ms |
| Open Banking Account Fetch | 120ms | 250ms | 400ms |
| BNPL Eligibility Check | 100ms | 200ms | 350ms |
| 3DS Challenge | 800ms | 1500ms | 2500ms |
| Webhook Delivery | 2s | 5s | 10s |

**Throughput Requirements:**
- **Payment API**: 50,000 TPS (peak), 10,000 TPS (sustained)
- **Open Banking**: 20,000 TPS
- **BNPL**: 5,000 TPS

### 5.2 Caching Strategy

**Multi-Level Cache:**
```
┌──────────────────────────────────────────────────────────┐
│  L1: Application Cache (in-memory, per service)          │
│  - Customer metadata: 60s TTL                            │
│  - Merchant info: 300s TTL                               │
│  Implementation: Caffeine (Java), LRU Cache (Node.js)    │
└───────────────┬──────────────────────────────────────────┘
                │
┌───────────────▼──────────────────────────────────────────┐
│  L2: Distributed Cache (Redis Cluster)                   │
│  - Payment methods: 3600s TTL                            │
│  - BNPL plans: 1800s TTL                                 │
│  - Open banking consents: 300s TTL                       │
│  Configuration: 6 nodes (3 masters, 3 replicas)          │
└───────────────┬──────────────────────────────────────────┘
                │
┌───────────────▼──────────────────────────────────────────┐
│  L3: Database with Read Replicas (PostgreSQL)            │
│  - Primary: writes only                                  │
│  - Replicas (3x): read-heavy queries                     │
└──────────────────────────────────────────────────────────┘
```

**Cache Invalidation:**
```javascript
// Event-driven cache invalidation
eventBus.on('payment_method.updated', async (event) => {
  const { customer_id, payment_method_id } = event.data;

  // Invalidate L1 cache (local)
  appCache.delete(`payment_method:${payment_method_id}`);

  // Invalidate L2 cache (Redis)
  await redis.del(`customer:${customer_id}:payment_methods`);

  // Publish to other service instances
  await pubsub.publish('cache.invalidate', {
    keys: [`payment_method:${payment_method_id}`]
  });
});
```

### 5.3 Database Optimization

**Read/Write Splitting:**
```
Payment Write → Primary DB (synchronous replication) → Replicas (for reads)

# Connection pool configuration
const writePool = new Pool({
  host: 'primary.db.wia-fintech.io',
  max: 50,
  connectionTimeoutMillis: 2000
});

const readPool = new Pool({
  host: 'replica.db.wia-fintech.io',  // Load-balanced across 3 replicas
  max: 200,
  connectionTimeoutMillis: 1000
});
```

**Indexing Strategy:**
```sql
-- Payment table indexes
CREATE INDEX idx_payments_customer_created ON payments(customer_id, created_at DESC);
CREATE INDEX idx_payments_merchant_status ON payments(merchant_id, status, created_at DESC);
CREATE INDEX idx_payments_status_created ON payments(status, created_at) WHERE status IN ('pending', 'processing');

-- Partial index for active BNPL orders
CREATE INDEX idx_bnpl_orders_active ON bnpl_orders(customer_id, status, created_at DESC)
  WHERE status IN ('active', 'pending_approval');

-- GIN index for JSONB metadata search
CREATE INDEX idx_payments_metadata_gin ON payments USING GIN (metadata jsonb_path_ops);
```

**Partitioning:**
```sql
-- Time-series partitioning for transaction logs
CREATE TABLE transactions (
  id BIGSERIAL,
  customer_id UUID,
  amount NUMERIC(12,2),
  created_at TIMESTAMP NOT NULL,
  ...
) PARTITION BY RANGE (created_at);

-- Monthly partitions
CREATE TABLE transactions_2026_01 PARTITION OF transactions
  FOR VALUES FROM ('2026-01-01') TO ('2026-02-01');

CREATE TABLE transactions_2026_02 PARTITION OF transactions
  FOR VALUES FROM ('2026-02-01') TO ('2026-03-01');
```

### 5.4 API Rate Limiting

**Tiered Rate Limits:**
```typescript
const rateLimits = {
  free_tier: {
    requests_per_second: 10,
    requests_per_day: 1000,
    burst: 20
  },
  standard_tier: {
    requests_per_second: 100,
    requests_per_day: 100000,
    burst: 200
  },
  enterprise_tier: {
    requests_per_second: 1000,
    requests_per_day: 10000000,
    burst: 2000
  }
};

// Token bucket algorithm
class TokenBucket {
  constructor(capacity, refillRate) {
    this.capacity = capacity;  // Max tokens
    this.tokens = capacity;
    this.refillRate = refillRate;  // Tokens per second
    this.lastRefill = Date.now();
  }

  consume(tokens = 1) {
    this.refill();
    if (this.tokens >= tokens) {
      this.tokens -= tokens;
      return true;
    }
    return false;  // Rate limit exceeded
  }

  refill() {
    const now = Date.now();
    const elapsedSeconds = (now - this.lastRefill) / 1000;
    const tokensToAdd = elapsedSeconds * this.refillRate;
    this.tokens = Math.min(this.capacity, this.tokens + tokensToAdd);
    this.lastRefill = now;
  }
}
```

**Rate Limit Headers:**
```
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1736616000

# If exceeded
HTTP/1.1 429 Too Many Requests
Retry-After: 42
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 0
X-RateLimit-Reset: 1736616042
```

---

## 6. Reliability & Resilience

### 6.1 High Availability Architecture

**Multi-Region Deployment:**
```
┌─────────────────────┐    ┌─────────────────────┐    ┌─────────────────────┐
│   US-East Region    │    │   EU-West Region    │    │  Asia-Pacific Reg   │
│                     │    │                     │    │                     │
│  ┌──────────────┐   │    │  ┌──────────────┐   │    │  ┌──────────────┐   │
│  │ API Gateway  │   │    │  │ API Gateway  │   │    │  │ API Gateway  │   │
│  └──────┬───────┘   │    │  └──────┬───────┘   │    │  └──────┬───────┘   │
│         │           │    │         │           │    │         │           │
│  ┌──────▼───────┐   │    │  ┌──────▼───────┐   │    │  ┌──────▼───────┐   │
│  │  Services    │   │    │  │  Services    │   │    │  │  Services    │   │
│  │  (K8s pods)  │   │    │  │  (K8s pods)  │   │    │  │  (K8s pods)  │   │
│  └──────┬───────┘   │    │  └──────┬───────┘   │    │  └──────┬───────┘   │
│         │           │    │         │           │    │         │           │
│  ┌──────▼───────┐   │    │  ┌──────▼───────┐   │    │  ┌──────▼───────┐   │
│  │   Database   │◄──┼────┼──┤   Database   │◄──┼────┼──┤   Database   │   │
│  │   (Primary)  │───┼────┼─►│  (Replica)   │───┼────┼─►│  (Replica)   │   │
│  └──────────────┘   │    │  └──────────────┘   │    │  └──────────────┘   │
└─────────────────────┘    └─────────────────────┘    └─────────────────────┘
          ▲                          ▲                          ▲
          │                          │                          │
          └──────────────────────────┴──────────────────────────┘
                   Global Load Balancer (GeoDNS)
                   (Route 53, Cloudflare)
```

**Availability Target: 99.99%** (52.56 minutes downtime/year)

### 6.2 Circuit Breaker Pattern

```javascript
class CircuitBreaker {
  constructor(options) {
    this.failureThreshold = options.failureThreshold || 5;
    this.timeout = options.timeout || 60000;  // 60s
    this.resetTimeout = options.resetTimeout || 30000;  // 30s

    this.state = 'CLOSED';  // CLOSED, OPEN, HALF_OPEN
    this.failureCount = 0;
    this.nextAttempt = Date.now();
  }

  async execute(fn) {
    if (this.state === 'OPEN') {
      if (Date.now() < this.nextAttempt) {
        throw new Error('Circuit breaker is OPEN');
      }
      this.state = 'HALF_OPEN';
    }

    try {
      const result = await Promise.race([
        fn(),
        this.timeoutPromise()
      ]);

      this.onSuccess();
      return result;
    } catch (error) {
      this.onFailure();
      throw error;
    }
  }

  onSuccess() {
    this.failureCount = 0;
    if (this.state === 'HALF_OPEN') {
      this.state = 'CLOSED';
    }
  }

  onFailure() {
    this.failureCount++;
    if (this.failureCount >= this.failureThreshold) {
      this.state = 'OPEN';
      this.nextAttempt = Date.now() + this.resetTimeout;
    }
  }

  timeoutPromise() {
    return new Promise((_, reject) => {
      setTimeout(() => reject(new Error('Timeout')), this.timeout);
    });
  }
}

// Usage
const bankAPICircuitBreaker = new CircuitBreaker({
  failureThreshold: 3,
  timeout: 5000,
  resetTimeout: 30000
});

const accounts = await bankAPICircuitBreaker.execute(() =>
  fetch('https://bank-api.example.com/accounts')
);
```

### 6.3 Disaster Recovery

**Recovery Time Objective (RTO):** 15 minutes
**Recovery Point Objective (RPO):** 5 minutes

**Backup Strategy:**
```yaml
database_backups:
  continuous_archiving:
    method: WAL (Write-Ahead Logging) streaming
    frequency: real-time
    retention: 30 days

  snapshots:
    frequency: every 6 hours
    retention: 7 days

  cross_region_replication:
    destination: secondary region
    lag: < 5 seconds

  point_in_time_recovery:
    granularity: 1 second
    window: 30 days
```

**Failover Procedure:**
1. **Detection**: Health checks fail (3 consecutive failures in 15s)
2. **Validation**: Automated tests confirm primary region unavailability
3. **DNS Update**: GeoDNS routes traffic to secondary region (TTL: 60s)
4. **Database Promotion**: Replica promoted to primary (< 30s)
5. **Application Failover**: K8s workloads scale up in secondary region
6. **Verification**: Smoke tests confirm functionality
7. **Notification**: Incident declared, stakeholders notified

---

## Compliance Checklist

- [ ] PCI DSS SAQ-D certification (annual)
- [ ] ISO/IEC 27001:2022 certification
- [ ] SOC 2 Type II audit report
- [ ] PSD2 technical standards compliance
- [ ] Section 1033 readiness (US financial institutions)
- [ ] GDPR Data Protection Impact Assessment (DPIA)
- [ ] Penetration testing (annual, quarterly for PCI)
- [ ] Vulnerability scanning (monthly)
- [ ] Third-party security audits
- [ ] Incident response plan tested (quarterly)

---

## References

1. **PCI Security Standards Council** - PCI DSS v4.0.1
2. **EBA** - RTS on Strong Customer Authentication (Regulation EU 2018/389)
3. **OpenID Foundation** - Financial-grade API (FAPI) 2.0
4. **NIST** - SP 800-63B Digital Identity Guidelines
5. **OWASP** - API Security Top 10
6. **ISO/IEC** - 27001:2022, 27017:2015, 27018:2019

---

**© 2026 WIA | 弘益人間**
