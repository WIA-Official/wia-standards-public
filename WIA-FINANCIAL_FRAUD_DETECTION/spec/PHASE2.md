# WIA-FINANCIAL_FRAUD_DETECTION - PHASE 2: API Specification & Data Models
**Version**: 1.0
**Status**: Production Ready
**Last Updated**: 2026-01-11

---

## 📋 Table of Contents

1. [API Overview](#api-overview)
2. [Authentication & Authorization](#authentication--authorization)
3. [Core API Endpoints](#core-api-endpoints)
4. [Data Models](#data-models)
5. [Feature Engineering Pipeline](#feature-engineering-pipeline)
6. [ML Model Specifications](#ml-model-specifications)
7. [Webhook Events](#webhook-events)
8. [Error Handling](#error-handling)
9. [Rate Limiting & Quotas](#rate-limiting--quotas)

---

## 1. API Overview

### 1.1 Base URL

**Production**: `https://api.wia-fraud.io/v1`
**Sandbox**: `https://sandbox-api.wia-fraud.io/v1`

### 1.2 API Design Principles

- **RESTful**: Standard HTTP methods (GET, POST, PUT, DELETE)
- **JSON**: All request/response bodies use JSON
- **Idempotent**: POST requests support idempotency keys
- **Versioned**: API version in URL path (`/v1`, `/v2`)
- **HATEOAS**: Hypermedia links for resource navigation
- **Pagination**: Cursor-based pagination for list endpoints

### 1.3 Request/Response Format

**Request Headers**:
```http
Content-Type: application/json
Authorization: Bearer {access_token}
X-API-Key: {api_key}
X-Idempotency-Key: {uuid} (optional, for POST/PUT)
X-Request-ID: {uuid} (optional, for tracing)
```

**Response Format**:
```json
{
  "status": "success" | "error",
  "data": { ... },
  "meta": {
    "request_id": "req_abc123",
    "timestamp": "2026-01-11T10:30:00Z",
    "processing_time_ms": 87
  },
  "errors": [ ... ] (only if status=error)
}
```

### 1.4 Supported Protocols

- **REST API**: Primary interface (HTTPS)
- **GraphQL**: Advanced querying (Beta)
- **gRPC**: High-performance internal services
- **WebSockets**: Real-time fraud alerts

### 1.5 SDKs

Official SDKs available for:
- Python (pip install wia-fraud-detection)
- TypeScript/Node.js (npm install @wia/fraud-detection)
- Java (Maven: com.wia:fraud-detection-sdk)
- Go (go get github.com/wia-official/fraud-detection-go)
- Ruby (gem install wia-fraud-detection)

---

## 2. Authentication & Authorization

### 2.1 Authentication Methods

#### 2.1.1 API Key Authentication (Simple)
```http
GET /v1/fraud/score HTTP/1.1
Host: api.wia-fraud.io
X-API-Key: EXAMPLE_API_KEY_REPLACE_ME
```

**Use Case**: Server-to-server communication, backend services

#### 2.1.2 OAuth 2.0 (Recommended)
```http
POST /v1/oauth/token HTTP/1.1
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=client_abc123
&client_secret=secret_def456
&scope=fraud:read fraud:write
```

**Response**:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "fraud:read fraud:write"
}
```

**Scopes**:
- `fraud:read` - Read fraud scores and reports
- `fraud:write` - Submit transactions for analysis
- `fraud:admin` - Manage rules, models, and configurations
- `fraud:feedback` - Submit feedback (confirm/dispute fraud)

#### 2.1.3 JWT Token (Mobile/Web Apps)
```http
GET /v1/fraud/score HTTP/1.1
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**JWT Claims**:
```json
{
  "sub": "user_123",
  "iss": "https://auth.wia-fraud.io",
  "aud": "api.wia-fraud.io",
  "exp": 1736596800,
  "iat": 1736593200,
  "scope": ["fraud:read"],
  "merchant_id": "merch_abc123"
}
```

### 2.2 Authorization Model (RBAC)

**Roles**:
| Role | Permissions | Use Case |
|------|-------------|----------|
| **Viewer** | Read fraud scores, view reports | Analysts, support staff |
| **Operator** | Submit transactions, update feedback | Integration services |
| **Admin** | Configure rules, manage models | Fraud team, DevOps |
| **Auditor** | Read-only access to all data | Compliance, security audit |

**Permission Matrix**:
```
┌─────────────────┬────────┬──────────┬───────┬─────────┐
│ Resource        │ Viewer │ Operator │ Admin │ Auditor │
├─────────────────┼────────┼──────────┼───────┼─────────┤
│ fraud:score     │   R    │   R/W    │  R/W  │    R    │
│ fraud:report    │   R    │   R      │  R/W  │    R    │
│ fraud:rules     │   -    │   R      │  R/W  │    R    │
│ fraud:models    │   -    │   -      │  R/W  │    R    │
│ fraud:feedback  │   -    │   W      │  R/W  │    R    │
│ fraud:audit_log │   -    │   -      │  R    │    R    │
└─────────────────┴────────┴──────────┴───────┴─────────┘
```

### 2.3 IP Whitelisting

For enhanced security, restrict API access to specific IP ranges:

```bash
# Configure via API
POST /v1/settings/ip-whitelist
{
  "merchant_id": "merch_abc123",
  "allowed_ips": [
    "203.0.113.0/24",
    "198.51.100.42"
  ]
}
```

---

## 3. Core API Endpoints

### 3.1 Transaction Analysis

#### **POST /v1/fraud/analyze**

Submit a transaction for real-time fraud analysis.

**Request**:
```json
{
  "transaction": {
    "id": "txn_1234567890",
    "amount": 149.99,
    "currency": "USD",
    "timestamp": "2026-01-11T10:30:00Z",
    "type": "purchase",
    "merchant": {
      "id": "merch_abc123",
      "name": "TechStore Inc",
      "mcc": "5732",
      "country": "US"
    },
    "customer": {
      "id": "cust_xyz789",
      "email": "john.doe@example.com",
      "phone": "+1-555-123-4567",
      "name": "John Doe",
      "account_age_days": 365
    },
    "payment_method": {
      "type": "card",
      "card_bin": "424242",
      "card_last4": "4242",
      "card_brand": "visa",
      "card_funding": "credit",
      "card_country": "US"
    },
    "shipping_address": {
      "line1": "123 Main St",
      "city": "San Francisco",
      "state": "CA",
      "postal_code": "94105",
      "country": "US"
    },
    "billing_address": {
      "line1": "123 Main St",
      "city": "San Francisco",
      "state": "CA",
      "postal_code": "94105",
      "country": "US"
    },
    "device": {
      "fingerprint": "fp_abc123def456",
      "ip_address": "203.0.113.42",
      "user_agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)...",
      "accept_language": "en-US,en;q=0.9",
      "screen_resolution": "1920x1080",
      "timezone_offset": -480
    },
    "session": {
      "id": "sess_123456",
      "duration_seconds": 420,
      "pages_viewed": 5,
      "referrer": "https://google.com"
    }
  }
}
```

**Response** (95ms):
```json
{
  "status": "success",
  "data": {
    "fraud_assessment": {
      "transaction_id": "txn_1234567890",
      "risk_score": 0.23,
      "risk_level": "low",
      "decision": "approve",
      "decision_reasons": [
        "Customer has 365-day account history with clean record",
        "Device previously seen with this customer (5 times)",
        "Shipping address matches billing address",
        "Transaction amount within normal spending pattern"
      ],
      "model_scores": {
        "xgboost": 0.18,
        "random_forest": 0.25,
        "deep_learning": 0.22,
        "isolation_forest": 0.31,
        "autoencoder": 0.19
      },
      "rule_evaluations": [
        {
          "rule_id": "velocity_check",
          "rule_name": "Transaction Velocity",
          "triggered": false,
          "details": "2 transactions in last hour (threshold: 3)"
        },
        {
          "rule_id": "geolocation_check",
          "rule_name": "IP Geolocation",
          "triggered": false,
          "details": "IP location matches billing address"
        }
      ],
      "feature_importance": [
        {"feature": "customer_account_age_days", "importance": 0.18, "value": 365},
        {"feature": "device_frequency", "importance": 0.15, "value": 5},
        {"feature": "amount_deviation_from_avg", "importance": 0.12, "value": 0.08},
        {"feature": "address_match", "importance": 0.11, "value": 1},
        {"feature": "hour_of_day", "importance": 0.09, "value": 10}
      ],
      "recommended_actions": [
        {
          "action": "approve",
          "confidence": 0.95,
          "reasoning": "Low risk score with strong positive signals"
        }
      ]
    }
  },
  "meta": {
    "request_id": "req_abc123",
    "timestamp": "2026-01-11T10:30:00.095Z",
    "processing_time_ms": 95,
    "model_version": "v2.3.1"
  }
}
```

**Decision Values**:
- `approve` - Proceed with transaction (risk_score < 0.50)
- `challenge` - Request additional authentication (0.50 ≤ risk_score < 0.70)
- `review` - Manual review required (0.70 ≤ risk_score < 0.90)
- `block` - Reject transaction (risk_score ≥ 0.90)

**HTTP Status Codes**:
- `200 OK` - Analysis completed successfully
- `400 Bad Request` - Invalid request format
- `401 Unauthorized` - Authentication failed
- `429 Too Many Requests` - Rate limit exceeded
- `500 Internal Server Error` - Server error
- `503 Service Unavailable` - Temporary overload

### 3.2 Batch Transaction Analysis

#### **POST /v1/fraud/batch-analyze**

Analyze multiple transactions in a single request (up to 100 transactions).

**Request**:
```json
{
  "transactions": [
    { "id": "txn_001", "amount": 50.00, ... },
    { "id": "txn_002", "amount": 75.50, ... },
    ...
  ]
}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "results": [
      {
        "transaction_id": "txn_001",
        "fraud_assessment": { ... }
      },
      {
        "transaction_id": "txn_002",
        "fraud_assessment": { ... }
      }
    ],
    "summary": {
      "total_transactions": 100,
      "approved": 85,
      "challenged": 8,
      "reviewed": 5,
      "blocked": 2
    }
  },
  "meta": {
    "processing_time_ms": 1250,
    "batch_id": "batch_abc123"
  }
}
```

### 3.3 Feedback Submission

#### **POST /v1/fraud/feedback**

Submit feedback on fraud decisions to improve model accuracy.

**Request**:
```json
{
  "transaction_id": "txn_1234567890",
  "feedback_type": "confirmed_fraud" | "false_positive" | "true_negative",
  "fraud_type": "account_takeover" | "stolen_card" | "synthetic_identity" | null,
  "notes": "Customer confirmed unauthorized transaction via phone support",
  "submitted_by": "agent_123",
  "evidence": {
    "chargeback_id": "cb_abc123",
    "chargeback_reason": "10.4 - Fraud - Card Absent Environment"
  }
}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "feedback_id": "fb_xyz789",
    "transaction_id": "txn_1234567890",
    "feedback_accepted": true,
    "model_update_scheduled": true,
    "estimated_retrain_date": "2026-01-14T00:00:00Z"
  }
}
```

**Feedback Types**:
- `confirmed_fraud` - Transaction was actually fraudulent (true positive)
- `false_positive` - Legitimate transaction incorrectly flagged
- `true_negative` - Correctly approved legitimate transaction
- `false_negative` - Fraudulent transaction that was approved (missed)

### 3.4 Fraud Report Retrieval

#### **GET /v1/fraud/reports/{transaction_id}**

Retrieve detailed fraud analysis report for a specific transaction.

**Response**:
```json
{
  "status": "success",
  "data": {
    "report": {
      "transaction_id": "txn_1234567890",
      "created_at": "2026-01-11T10:30:00Z",
      "fraud_assessment": { ... },
      "historical_context": {
        "customer_lifetime_transactions": 47,
        "customer_fraud_history": 0,
        "customer_dispute_history": 1,
        "device_seen_count": 5,
        "device_fraud_history": 0
      },
      "similar_transactions": [
        {
          "transaction_id": "txn_0987654321",
          "similarity_score": 0.89,
          "outcome": "approved",
          "fraud_occurred": false
        }
      ],
      "network_analysis": {
        "connected_accounts": 0,
        "shared_devices": 1,
        "shared_addresses": 0,
        "risk_cluster": null
      }
    }
  }
}
```

### 3.5 Statistics & Analytics

#### **GET /v1/fraud/statistics**

Retrieve fraud statistics for a date range.

**Query Parameters**:
- `start_date`: ISO 8601 date (e.g., 2026-01-01)
- `end_date`: ISO 8601 date
- `group_by`: `day`, `week`, `month`
- `merchant_id`: Filter by merchant (optional)

**Response**:
```json
{
  "status": "success",
  "data": {
    "statistics": {
      "date_range": {
        "start": "2026-01-01",
        "end": "2026-01-11"
      },
      "total_transactions": 125000,
      "fraud_detected": 1250,
      "fraud_rate": 0.01,
      "false_positive_rate": 0.0045,
      "false_negative_rate": 0.0012,
      "chargebacks": 45,
      "chargeback_rate": 0.00036,
      "amount_saved_usd": 187500.00,
      "by_decision": {
        "approved": 120000,
        "challenged": 2500,
        "reviewed": 1500,
        "blocked": 1000
      },
      "by_fraud_type": {
        "stolen_card": 450,
        "account_takeover": 320,
        "synthetic_identity": 180,
        "friendly_fraud": 150,
        "other": 150
      },
      "top_countries_fraud": [
        {"country": "US", "fraud_count": 400},
        {"country": "UK", "fraud_count": 180},
        {"country": "CA", "fraud_count": 120}
      ]
    }
  }
}
```

### 3.6 Rule Management

#### **GET /v1/fraud/rules**

List all fraud detection rules.

**Response**:
```json
{
  "status": "success",
  "data": {
    "rules": [
      {
        "rule_id": "rule_velocity_001",
        "name": "Transaction Velocity Check",
        "type": "velocity",
        "enabled": true,
        "priority": 1,
        "conditions": {
          "max_transactions_per_minute": 3,
          "max_transactions_per_day": 50
        },
        "action": "review",
        "last_triggered": "2026-01-11T09:15:00Z",
        "trigger_count_today": 12
      },
      {
        "rule_id": "rule_geo_001",
        "name": "Impossible Travel Detection",
        "type": "geolocation",
        "enabled": true,
        "priority": 2,
        "conditions": {
          "min_distance_km": 500,
          "max_time_hours": 2
        },
        "action": "block",
        "last_triggered": "2026-01-11T08:30:00Z",
        "trigger_count_today": 3
      }
    ]
  }
}
```

#### **POST /v1/fraud/rules**

Create a new fraud detection rule.

**Request**:
```json
{
  "name": "High Value Transaction Review",
  "type": "threshold",
  "enabled": true,
  "priority": 3,
  "conditions": {
    "amount_usd": {
      "operator": ">=",
      "value": 1000
    },
    "customer_account_age_days": {
      "operator": "<",
      "value": 30
    }
  },
  "action": "review"
}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "rule_id": "rule_threshold_002",
    "created_at": "2026-01-11T10:45:00Z"
  }
}
```

---

## 4. Data Models

### 4.1 Transaction Schema

```typescript
interface Transaction {
  // Core identifiers
  id: string;                    // Unique transaction ID
  merchant_id: string;           // Merchant identifier
  customer_id: string;           // Customer identifier

  // Financial details
  amount: number;                // Transaction amount
  currency: string;              // ISO 4217 currency code (USD, EUR, etc.)
  timestamp: string;             // ISO 8601 timestamp
  type: TransactionType;         // purchase, refund, authorization, etc.

  // Merchant information
  merchant: {
    id: string;
    name: string;
    mcc: string;                 // Merchant Category Code (e.g., 5732)
    country: string;             // ISO 3166-1 alpha-2 country code
    url?: string;
    phone?: string;
  };

  // Customer information
  customer: {
    id: string;
    email: string;
    phone?: string;
    name?: string;
    account_created_at: string;
    account_age_days: number;
    loyalty_tier?: string;       // bronze, silver, gold, platinum
    total_lifetime_value?: number;
  };

  // Payment method
  payment_method: CardPayment | BankPayment | WalletPayment;

  // Addresses
  shipping_address?: Address;
  billing_address?: Address;

  // Device & session
  device: DeviceInfo;
  session?: SessionInfo;

  // Additional metadata
  metadata?: Record<string, any>;
}

type TransactionType =
  | 'purchase'
  | 'authorization'
  | 'capture'
  | 'refund'
  | 'chargeback'
  | 'withdrawal'
  | 'deposit';

interface CardPayment {
  type: 'card';
  card_bin: string;              // First 6 digits of card (BIN)
  card_last4: string;            // Last 4 digits
  card_brand: 'visa' | 'mastercard' | 'amex' | 'discover' | 'other';
  card_funding: 'credit' | 'debit' | 'prepaid' | 'unknown';
  card_country: string;          // ISO 3166-1 alpha-2
  cardholder_name?: string;
  expiry_month?: number;         // 1-12
  expiry_year?: number;          // 4-digit year
  cvv_provided: boolean;
  three_d_secure?: {
    version: '1.0' | '2.0';
    authenticated: boolean;
    eci: string;                 // Electronic Commerce Indicator
    cavv?: string;               // Cardholder Authentication Verification Value
  };
}

interface BankPayment {
  type: 'bank_transfer' | 'ach' | 'wire';
  account_type: 'checking' | 'savings';
  routing_number?: string;
  account_last4?: string;
  bank_name?: string;
  bank_country: string;
}

interface WalletPayment {
  type: 'wallet';
  wallet_provider: 'paypal' | 'apple_pay' | 'google_pay' | 'venmo' | 'other';
  wallet_id: string;
  wallet_email?: string;
}

interface Address {
  line1: string;
  line2?: string;
  city: string;
  state?: string;                // State/province/region
  postal_code: string;
  country: string;               // ISO 3166-1 alpha-2
  latitude?: number;
  longitude?: number;
}

interface DeviceInfo {
  fingerprint: string;           // Unique device identifier
  ip_address: string;
  ip_geolocation?: {
    country: string;
    region: string;
    city: string;
    latitude: number;
    longitude: number;
    isp: string;
  };
  user_agent: string;
  accept_language: string;
  screen_resolution?: string;    // e.g., "1920x1080"
  timezone_offset: number;       // Minutes from UTC
  platform?: string;             // iOS, Android, Windows, macOS, etc.
  browser?: string;              // Chrome, Safari, Firefox, etc.
  is_vpn: boolean;
  is_proxy: boolean;
  is_tor: boolean;
}

interface SessionInfo {
  id: string;
  started_at: string;
  duration_seconds: number;
  pages_viewed: number;
  referrer?: string;
  utm_source?: string;
  utm_medium?: string;
  utm_campaign?: string;
}
```

### 4.2 Fraud Assessment Schema

```typescript
interface FraudAssessment {
  transaction_id: string;
  risk_score: number;            // 0.0 (no risk) to 1.0 (certain fraud)
  risk_level: RiskLevel;
  decision: Decision;
  decision_reasons: string[];

  // Model scores
  model_scores: {
    xgboost: number;
    random_forest: number;
    deep_learning: number;
    isolation_forest: number;
    autoencoder: number;
  };

  // Rule evaluations
  rule_evaluations: RuleEvaluation[];

  // Feature importance (top N features)
  feature_importance: FeatureImportance[];

  // Recommended actions
  recommended_actions: RecommendedAction[];

  // Timestamps
  analyzed_at: string;
  model_version: string;
}

type RiskLevel = 'very_low' | 'low' | 'medium' | 'high' | 'critical';

type Decision = 'approve' | 'challenge' | 'review' | 'block';

interface RuleEvaluation {
  rule_id: string;
  rule_name: string;
  triggered: boolean;
  details: string;
  confidence?: number;
}

interface FeatureImportance {
  feature: string;
  importance: number;            // 0.0 to 1.0
  value: any;                    // Actual feature value
  contribution: 'increases_risk' | 'decreases_risk';
}

interface RecommendedAction {
  action: Decision;
  confidence: number;
  reasoning: string;
  alternative_actions?: {
    action: Decision;
    confidence: number;
  }[];
}
```

### 4.3 Feedback Schema

```typescript
interface FraudFeedback {
  feedback_id: string;
  transaction_id: string;
  feedback_type: FeedbackType;
  fraud_type?: FraudType;
  notes?: string;
  submitted_by: string;
  submitted_at: string;
  evidence?: {
    chargeback_id?: string;
    chargeback_reason?: string;
    customer_confirmation?: boolean;
    law_enforcement_report?: string;
  };
}

type FeedbackType =
  | 'confirmed_fraud'      // True positive
  | 'false_positive'       // Legitimate incorrectly flagged
  | 'true_negative'        // Correctly approved
  | 'false_negative';      // Fraud that was missed

type FraudType =
  | 'stolen_card'
  | 'account_takeover'
  | 'synthetic_identity'
  | 'friendly_fraud'
  | 'refund_fraud'
  | 'card_testing'
  | 'money_laundering'
  | 'other';
```

---

## 5. Feature Engineering Pipeline

### 5.1 Feature Categories

#### 5.1.1 Transaction Features (15 features)

```python
# Amount-based features
- transaction_amount
- amount_z_score (deviation from user's average)
- amount_percentile (relative to all transactions)
- currency_conversion_rate (if not USD)

# Temporal features
- hour_of_day (0-23)
- day_of_week (0-6, Monday=0)
- is_weekend (boolean)
- is_business_hours (boolean)
- time_since_last_transaction_seconds

# Merchant features
- merchant_category_code (MCC)
- merchant_country_risk_score (0-1)
- merchant_fraud_rate_30d

# Transaction type
- transaction_type (encoded)
```

#### 5.1.2 User Behavioral Features (25 features)

```python
# Historical spending
- user_avg_transaction_amount_30d
- user_median_transaction_amount_30d
- user_std_transaction_amount_30d
- user_max_transaction_amount_30d
- user_total_transactions_30d
- user_total_amount_30d

# Account characteristics
- account_age_days
- account_age_hours (for new accounts)
- loyalty_tier_encoded (0=none, 1=bronze, 2=silver, 3=gold)
- lifetime_value

# Behavioral patterns
- avg_transactions_per_day_30d
- avg_transactions_per_week_30d
- unique_merchants_30d
- merchant_repeat_rate (transactions with same merchant / total)
- avg_days_between_transactions

# Geographic patterns
- unique_countries_30d
- unique_cities_30d
- home_country_indicator (boolean)
- travel_frequency_score

# Velocity features (rolling windows)
- transactions_last_1h
- transactions_last_6h
- transactions_last_24h
- amount_spent_last_24h
```

#### 5.1.3 Device & Network Features (20 features)

```python
# Device characteristics
- device_age_days (first seen)
- device_transaction_count_30d
- device_unique_users (how many users used this device)
- device_fraud_rate_30d

# IP address
- ip_country
- ip_region
- ip_city
- ip_reputation_score (0-1, from threat intelligence)
- ip_is_vpn (boolean)
- ip_is_proxy (boolean)
- ip_is_tor (boolean)
- ip_transaction_count_30d
- ip_unique_users_30d

# User agent
- browser_type (encoded)
- os_type (encoded)
- platform_type (encoded)
- is_mobile (boolean)
- is_bot_suspected (boolean)

# Session
- session_duration_seconds
- session_pages_viewed
```

#### 5.1.4 Address Features (10 features)

```python
# Address matching
- billing_shipping_address_match (boolean)
- billing_address_verified (boolean)
- shipping_address_verified (boolean)

# Address characteristics
- billing_country_risk_score
- shipping_country_risk_score
- address_age_days (how long customer used this address)

# Geographic distance
- billing_ip_distance_km (distance between billing address and IP)
- shipping_billing_distance_km
- impossible_travel_indicator (boolean)
- cross_border_indicator (billing country != IP country)
```

#### 5.1.5 Graph Features (10 features)

```python
# Network analysis
- connected_users_count (shared device, IP, address)
- connected_users_fraud_rate
- shared_device_count
- shared_ip_count
- shared_email_domain_indicator
- shared_phone_prefix_indicator

# Cluster analysis
- risk_cluster_id (from graph clustering)
- risk_cluster_size
- risk_cluster_fraud_rate
- community_detection_score
```

**Total Features**: 80

### 5.2 Feature Store Architecture

**Technology**: Feast (Feature Store)

**Components**:
1. **Offline Store**: Historical features for model training (Parquet on S3)
2. **Online Store**: Low-latency features for real-time inference (Redis)
3. **Feature Registry**: Metadata about features (YAML definitions)

**Example Feature Definition** (YAML):
```yaml
feature_view:
  name: user_transaction_statistics_30d
  entities:
    - customer
  ttl: 86400s  # 24 hours
  features:
    - name: user_avg_transaction_amount_30d
      dtype: FLOAT
    - name: user_total_transactions_30d
      dtype: INT64
    - name: user_unique_merchants_30d
      dtype: INT64
  batch_source:
    type: BigQuerySource
    query: |
      SELECT
        customer_id,
        timestamp,
        AVG(amount) as user_avg_transaction_amount_30d,
        COUNT(*) as user_total_transactions_30d,
        COUNT(DISTINCT merchant_id) as user_unique_merchants_30d
      FROM transactions
      WHERE timestamp >= TIMESTAMP_SUB(CURRENT_TIMESTAMP(), INTERVAL 30 DAY)
      GROUP BY customer_id, DATE(timestamp)
  stream_source:
    type: KafkaSource
    topic: transactions
    bootstrap_servers: kafka:9092
```

### 5.3 Real-Time Feature Computation

**Architecture**:
```
Kafka (transaction events)
  ↓
Flink (stream processing)
  ↓
Feature calculations (windowed aggregations)
  ↓
Redis (online feature store)
  ↓
ML inference service
```

**Flink Job Example** (velocity features):
```java
DataStream<Transaction> transactions = env
    .addSource(new FlinkKafkaConsumer<>("transactions", ...))
    .keyBy(Transaction::getCustomerId)
    .window(TumblingEventTimeWindows.of(Time.hours(1)))
    .aggregate(new TransactionCountAggregator());

// Write to Redis
transactions.addSink(new RedisSink<>(redisConfig));
```

---

## 6. ML Model Specifications

### 6.1 XGBoost Model

**Algorithm**: Gradient Boosted Decision Trees

**Hyperparameters**:
```python
{
    'objective': 'binary:logistic',
    'eval_metric': 'auc',
    'max_depth': 6,
    'learning_rate': 0.1,
    'n_estimators': 200,
    'subsample': 0.8,
    'colsample_bytree': 0.8,
    'scale_pos_weight': 50,  # Address class imbalance (1:50 fraud ratio)
    'min_child_weight': 3,
    'gamma': 0.1,
    'reg_alpha': 0.1,  # L1 regularization
    'reg_lambda': 1.0,  # L2 regularization
    'random_state': 42
}
```

**Training Data**:
- **Size**: 10M transactions (200K fraud, 9.8M legitimate)
- **Time Range**: Last 6 months
- **Sampling**: SMOTE (Synthetic Minority Oversampling) for training set
- **Validation**: Stratified 5-fold cross-validation

**Performance**:
- **F1-Score**: 0.947
- **AUC-ROC**: 0.994
- **Precision**: 0.92
- **Recall**: 0.95
- **Training Time**: 45 minutes on 8-core CPU

**Model File**: `xgboost_fraud_v2.3.1.json` (15 MB)

### 6.2 Random Forest Model

**Algorithm**: Ensemble of Decision Trees

**Hyperparameters**:
```python
{
    'n_estimators': 100,
    'max_depth': 10,
    'min_samples_split': 5,
    'min_samples_leaf': 2,
    'max_features': 'sqrt',  # sqrt(80) ≈ 9 features per tree
    'class_weight': 'balanced',
    'bootstrap': True,
    'oob_score': True,
    'random_state': 42,
    'n_jobs': -1
}
```

**Performance**:
- **F1-Score**: 0.925
- **AUC-ROC**: 0.988
- **Precision**: 0.89
- **Recall**: 0.93
- **Out-of-Bag Score**: 0.982

**Model File**: `random_forest_fraud_v2.3.1.pkl` (120 MB)

### 6.3 Deep Neural Network

**Framework**: TensorFlow 2.15

**Architecture**:
```python
model = keras.Sequential([
    keras.layers.Input(shape=(80,)),
    keras.layers.BatchNormalization(),
    keras.layers.Dense(128, activation='relu', kernel_regularizer='l2'),
    keras.layers.Dropout(0.3),
    keras.layers.Dense(64, activation='relu', kernel_regularizer='l2'),
    keras.layers.Dropout(0.3),
    keras.layers.Dense(32, activation='relu', kernel_regularizer='l2'),
    keras.layers.Dropout(0.2),
    keras.layers.Dense(1, activation='sigmoid')
])
```

**Training Configuration**:
```python
{
    'loss': focal_loss(alpha=0.25, gamma=2.0),  # Handle class imbalance
    'optimizer': keras.optimizers.Adam(learning_rate=0.001),
    'metrics': ['AUC', 'Precision', 'Recall'],
    'batch_size': 512,
    'epochs': 50,
    'early_stopping': {
        'monitor': 'val_auc',
        'patience': 5,
        'restore_best_weights': True
    }
}
```

**Performance**:
- **F1-Score**: 0.935
- **AUC-ROC**: 0.991
- **Precision**: 0.91
- **Recall**: 0.94
- **Training Time**: 2 hours on GPU (NVIDIA A100)

**Model File**: `dnn_fraud_v2.3.1.h5` (8 MB)

### 6.4 Isolation Forest (Unsupervised)

**Algorithm**: Anomaly Detection

**Hyperparameters**:
```python
{
    'n_estimators': 100,
    'max_samples': 256,
    'contamination': 0.01,  # Expected fraud rate
    'max_features': 1.0,
    'bootstrap': False,
    'random_state': 42
}
```

**Use Case**: Detect novel fraud patterns not seen during training

**Anomaly Score Interpretation**:
- Score < -0.5: High anomaly (potential fraud)
- -0.5 ≤ Score < 0: Moderate anomaly
- Score ≥ 0: Normal behavior

### 6.5 Autoencoder (Unsupervised)

**Architecture**:
```python
# Encoder
encoder = keras.Sequential([
    keras.layers.Input(shape=(80,)),
    keras.layers.Dense(40, activation='relu'),
    keras.layers.Dense(20, activation='relu')
])

# Decoder
decoder = keras.Sequential([
    keras.layers.Input(shape=(20,)),
    keras.layers.Dense(40, activation='relu'),
    keras.layers.Dense(80, activation='sigmoid')
])

# Autoencoder
autoencoder = keras.Model(inputs=encoder.input, outputs=decoder(encoder.output))
```

**Training**:
- **Loss**: Mean Squared Error (reconstruction error)
- **Training Data**: Only legitimate transactions
- **Threshold**: 95th percentile of reconstruction error on validation set

**Anomaly Detection**:
```python
reconstruction_error = mse(input, autoencoder(input))
if reconstruction_error > threshold:
    flag_as_anomaly()
```

### 6.6 Model Serving Infrastructure

**Technology**: TensorFlow Serving + Seldon Core

**Deployment**:
```yaml
apiVersion: v1
kind: Service
metadata:
  name: fraud-detection-model
spec:
  predictor:
    replicas: 3
    graph:
      name: ensemble
      type: ENSEMBLE
      children:
        - name: xgboost
          type: MODEL
          endpoint:
            type: REST
          modelUri: s3://models/xgboost_fraud_v2.3.1
        - name: random-forest
          type: MODEL
          modelUri: s3://models/random_forest_fraud_v2.3.1
        - name: dnn
          type: MODEL
          modelUri: s3://models/dnn_fraud_v2.3.1
```

**Load Balancing**: Round-robin across 3 replicas
**Auto-Scaling**: Based on CPU (70% threshold) and request latency (>100ms)

---

## 7. Webhook Events

### 7.1 Webhook Configuration

Register webhook URLs to receive real-time fraud events.

**POST /v1/webhooks**:
```json
{
  "url": "https://yourapp.com/webhooks/fraud",
  "events": ["fraud.high_risk_detected", "fraud.blocked"],
  "secret": "whsec_abc123def456"
}
```

### 7.2 Event Types

| Event | Description | Trigger Condition |
|-------|-------------|-------------------|
| `fraud.high_risk_detected` | High-risk transaction detected | risk_score ≥ 0.70 |
| `fraud.blocked` | Transaction blocked | decision = "block" |
| `fraud.review_required` | Manual review needed | decision = "review" |
| `fraud.challenge_issued` | Step-up auth requested | decision = "challenge" |
| `fraud.feedback_received` | Feedback submitted | POST /feedback |

### 7.3 Webhook Payload

```json
{
  "id": "evt_abc123",
  "type": "fraud.high_risk_detected",
  "created": 1736593800,
  "data": {
    "transaction_id": "txn_1234567890",
    "risk_score": 0.87,
    "risk_level": "high",
    "decision": "review",
    "reasons": [
      "First transaction from this device",
      "Amount 3x higher than average"
    ]
  }
}
```

### 7.4 Webhook Security

**Signature Verification** (HMAC-SHA256):
```python
import hmac
import hashlib

def verify_webhook(payload, signature, secret):
    expected_signature = hmac.new(
        secret.encode(),
        payload.encode(),
        hashlib.sha256
    ).hexdigest()
    return hmac.compare_digest(expected_signature, signature)
```

**Headers**:
```http
X-WIA-Signature: sha256=abc123def456...
X-WIA-Event-ID: evt_abc123
X-WIA-Timestamp: 1736593800
```

---

## 8. Error Handling

### 8.1 Error Response Format

```json
{
  "status": "error",
  "errors": [
    {
      "code": "invalid_request",
      "message": "Missing required field: transaction.amount",
      "field": "transaction.amount",
      "docs_url": "https://docs.wia-fraud.io/errors#invalid_request"
    }
  ],
  "meta": {
    "request_id": "req_abc123",
    "timestamp": "2026-01-11T10:30:00Z"
  }
}
```

### 8.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `invalid_request` | 400 | Malformed request (missing fields, invalid format) |
| `authentication_failed` | 401 | Invalid API key or expired token |
| `permission_denied` | 403 | Insufficient permissions for resource |
| `resource_not_found` | 404 | Transaction, rule, or report not found |
| `rate_limit_exceeded` | 429 | Too many requests (see Rate Limiting) |
| `internal_error` | 500 | Unexpected server error |
| `service_unavailable` | 503 | Temporary overload or maintenance |
| `model_error` | 500 | ML model inference failed |
| `timeout` | 504 | Request exceeded processing time limit (10s) |

### 8.3 Retry Strategy

**Exponential Backoff**:
```python
import time
import random

def retry_with_backoff(func, max_retries=3):
    for attempt in range(max_retries):
        try:
            return func()
        except (RateLimitError, ServiceUnavailable) as e:
            if attempt == max_retries - 1:
                raise
            wait_time = (2 ** attempt) + random.uniform(0, 1)
            time.sleep(wait_time)
```

**Idempotency**:
Use `X-Idempotency-Key` header to safely retry POST requests:
```http
POST /v1/fraud/analyze
X-Idempotency-Key: uuid-abc-123-def-456
```

---

## 9. Rate Limiting & Quotas

### 9.1 Rate Limits (Per API Key)

| Tier | Requests/Minute | Requests/Day | Burst Limit |
|------|-----------------|--------------|-------------|
| **Free** | 60 | 1,000 | 10 |
| **Starter** | 600 | 50,000 | 100 |
| **Professional** | 6,000 | 500,000 | 1,000 |
| **Enterprise** | Unlimited | Unlimited | Unlimited |

### 9.2 Rate Limit Headers

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 600
X-RateLimit-Remaining: 587
X-RateLimit-Reset: 1736594400
```

### 9.3 Rate Limit Exceeded Response

```http
HTTP/1.1 429 Too Many Requests
Retry-After: 45

{
  "status": "error",
  "errors": [{
    "code": "rate_limit_exceeded",
    "message": "Rate limit exceeded. Retry after 45 seconds."
  }]
}
```

---

**Document Status**: ✅ **Approved for PHASE 3 Development**
**Next Phase**: [PHASE3.md](./PHASE3.md) - Security, Performance & Compliance

---

© 2026 WIA (World Certification Industry Association)
**Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity
