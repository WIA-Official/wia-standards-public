# WIA Carbon Credit Micro API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Core Interfaces](#core-interfaces)
4. [API Endpoints](#api-endpoints)
5. [Footprint Tracking](#footprint-tracking)
6. [Credit Management](#credit-management)
7. [Trading Platform](#trading-platform)
8. [Authentication](#authentication)
9. [Error Handling](#error-handling)
10. [Usage Examples](#usage-examples)
11. [References](#references)

---

## Overview

### 1.1 Purpose

The WIA Carbon Credit Micro API Interface Standard defines a comprehensive programmatic interface for tracking individual carbon footprints, managing micro carbon credits, and facilitating blockchain-verified trading. This Phase 2 specification builds upon the Phase 1 Data Format, providing developers with standardized APIs to build carbon-conscious applications.

**Core Objectives**:
- Provide unified API for carbon footprint tracking and credit management
- Enable real-time emission monitoring and credit trading
- Support multi-platform integration (mobile, web, IoT)
- Facilitate corporate carbon offset programs
- Ensure blockchain transparency and verification

### 1.2 Scope

This standard defines:

| Component | Description |
|-----------|-------------|
| **Core API** | Main CarbonCreditMicro class interface |
| **REST Endpoints** | HTTP API for all carbon operations |
| **WebSocket Events** | Real-time market data and notifications |
| **SDK Support** | Python, TypeScript, JavaScript libraries |
| **Authentication** | OAuth 2.0 and API key authentication |

### 1.3 Phase 1 Compatibility

Phase 2 API is fully compatible with Phase 1 Data Format:

```
Phase 1: Data Format (JSON structure)
    ↓
Phase 2: API Interface (programming interface)
    ↓
Phase 3: Protocol (communication protocol)
    ↓
Phase 4: Integration (ecosystem integration)
```

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **CarbonCreditMicro** | Main API class for carbon operations |
| **FootprintTracker** | Component for emission tracking |
| **CreditWallet** | User's carbon credit balance manager |
| **TradingEngine** | Marketplace trading interface |
| **BlockchainVerifier** | On-chain verification service |
| **EmissionCalculator** | CO2 calculation engine |

### 2.2 API Types

| Type | Description |
|------|-------------|
| **Synchronous API** | Immediate response operations |
| **Asynchronous API** | Background processing with callbacks |
| **Streaming API** | Real-time data streams (WebSocket) |
| **Batch API** | Bulk operations for efficiency |

---

## Core Interfaces

### 3.1 CarbonCreditMicro Class

Main API entry point for all carbon credit operations.

#### TypeScript

```typescript
class CarbonCreditMicro {
  // Constructor
  constructor(options?: CarbonCreditMicroOptions);

  // Footprint Tracking
  trackEmission(activity: EmissionActivity): Promise<EmissionRecord>;
  getDailyFootprint(date: Date): Promise<DailyFootprint>;
  getFootprintHistory(startDate: Date, endDate: Date): Promise<FootprintHistory>;
  calculateEmission(params: EmissionParams): Promise<CO2Amount>;

  // Credit Management
  getCreditBalance(): Promise<CreditBalance>;
  earnCredit(action: OffsetAction): Promise<CreditEarned>;
  retireCredits(amount: number, reason: string): Promise<RetirementRecord>;
  transferCredits(recipient: string, amount: number): Promise<TransferRecord>;

  // Trading Platform
  listCreditsForSale(amount: number, price: number): Promise<Listing>;
  buyCredits(listingId: string, amount: number): Promise<Purchase>;
  sellCredits(buyerId: string, amount: number, price: number): Promise<Sale>;
  getMarketPrice(): Promise<MarketPrice>;
  getMarketHistory(period: TimePeriod): Promise<PriceHistory>;

  // Blockchain Operations
  mintCreditToken(creditId: string): Promise<TokenMinted>;
  verifyOnChain(recordId: string): Promise<VerificationResult>;
  getTransactionHistory(): Promise<BlockchainTransaction[]>;

  // Corporate Integration
  createOffsetProgram(config: OffsetProgramConfig): Promise<OffsetProgram>;
  purchaseBulkCredits(amount: number): Promise<BulkPurchase>;
  generateOffsetReport(period: TimePeriod): Promise<OffsetReport>;

  // Analytics
  getEmissionInsights(): Promise<EmissionInsights>;
  getCreditStatistics(): Promise<CreditStatistics>;
  compareToPeers(category?: string): Promise<PeerComparison>;

  // Event Handling
  on<T extends EventType>(event: T, handler: EventHandler<T>): void;
  off<T extends EventType>(event: T, handler: EventHandler<T>): void;
  once<T extends EventType>(event: T, handler: EventHandler<T>): void;
}
```

#### Python

```python
class CarbonCreditMicro:
    def __init__(self, options: Optional[CarbonCreditMicroOptions] = None):
        ...

    # Footprint Tracking
    async def track_emission(self, activity: EmissionActivity) -> EmissionRecord: ...
    async def get_daily_footprint(self, date: datetime.date) -> DailyFootprint: ...
    async def get_footprint_history(self, start_date: datetime.date, end_date: datetime.date) -> FootprintHistory: ...
    async def calculate_emission(self, params: EmissionParams) -> CO2Amount: ...

    # Credit Management
    async def get_credit_balance(self) -> CreditBalance: ...
    async def earn_credit(self, action: OffsetAction) -> CreditEarned: ...
    async def retire_credits(self, amount: float, reason: str) -> RetirementRecord: ...
    async def transfer_credits(self, recipient: str, amount: float) -> TransferRecord: ...

    # Trading Platform
    async def list_credits_for_sale(self, amount: float, price: float) -> Listing: ...
    async def buy_credits(self, listing_id: str, amount: float) -> Purchase: ...
    async def sell_credits(self, buyer_id: str, amount: float, price: float) -> Sale: ...
    async def get_market_price(self) -> MarketPrice: ...
    async def get_market_history(self, period: TimePeriod) -> PriceHistory: ...

    # Blockchain Operations
    async def mint_credit_token(self, credit_id: str) -> TokenMinted: ...
    async def verify_on_chain(self, record_id: str) -> VerificationResult: ...
    async def get_transaction_history(self) -> List[BlockchainTransaction]: ...

    # Event Handling
    def on(self, event: EventType, handler: EventHandler) -> None: ...
    def off(self, event: EventType, handler: EventHandler) -> None: ...
```

### 3.2 CarbonCreditMicroOptions

```typescript
interface CarbonCreditMicroOptions {
  // API Configuration
  apiKey?: string;
  apiSecret?: string;
  baseUrl?: string;

  // Authentication
  authMethod?: 'oauth2' | 'api_key' | 'jwt';
  oauthToken?: string;

  // Blockchain Configuration
  blockchainNetwork?: 'ethereum' | 'polygon' | 'avalanche';
  walletAddress?: string;
  privateKey?: string;

  // Calculation Settings
  emissionFactorRegion?: 'US' | 'EU' | 'CN' | 'IN' | 'KR';
  defaultCurrency?: 'USD' | 'EUR' | 'KRW';

  // Features
  enableAutoTracking?: boolean;
  enableBlockchainVerification?: boolean;
  enableRealTimeMarket?: boolean;

  // Logging
  logLevel?: 'debug' | 'info' | 'warn' | 'error' | 'none';
}
```

---

## API Endpoints

### 4.1 REST API Overview

All endpoints follow RESTful conventions with JSON payloads.

**Base URL**: `https://api.wia.live/carbon-credit-micro/v1`

**Authentication**: OAuth 2.0 Bearer token or API Key

**Content-Type**: `application/json`

### 4.2 Footprint Endpoints

#### POST /footprint/track

Track a new emission activity.

```http
POST /footprint/track
Authorization: Bearer <token>
Content-Type: application/json

{
  "category": "transportation",
  "subcategory": "car",
  "distance": 25,
  "distanceUnit": "km",
  "fuelType": "gasoline",
  "timestamp": "2025-01-15T08:30:00Z"
}
```

**Response** (201 Created):
```json
{
  "activityId": "ACT-2025-0001",
  "co2Amount": 5.2,
  "unit": "kg CO2e",
  "verified": true,
  "verificationMethod": "gps_tracking"
}
```

#### GET /footprint/daily/:date

Get daily carbon footprint.

```http
GET /footprint/daily/2025-01-15
Authorization: Bearer <token>
```

**Response** (200 OK):
```json
{
  "date": "2025-01-15",
  "totalEmissions": 18.5,
  "categories": {
    "transportation": 5.2,
    "energy": 6.9,
    "food": 3.4,
    "consumption": 2.5,
    "waste": 0.5
  },
  "activities": [...],
  "comparison": {
    "previousDay": -2.1,
    "nationalAverage": -6.5
  }
}
```

#### GET /footprint/history

Get footprint history over a period.

```http
GET /footprint/history?start=2025-01-01&end=2025-01-31&granularity=daily
Authorization: Bearer <token>
```

**Response**:
```json
{
  "period": {
    "start": "2025-01-01",
    "end": "2025-01-31"
  },
  "totalEmissions": 575.5,
  "averageDaily": 18.6,
  "data": [
    {
      "date": "2025-01-01",
      "emissions": 22.3
    },
    {
      "date": "2025-01-02",
      "emissions": 15.8
    }
  ],
  "trends": {
    "direction": "decreasing",
    "changePercent": -5.2
  }
}
```

#### POST /footprint/calculate

Calculate CO2 for a hypothetical activity.

```http
POST /footprint/calculate
Authorization: Bearer <token>
Content-Type: application/json

{
  "category": "transportation",
  "subcategory": "flight",
  "distance": 500,
  "distanceUnit": "km",
  "flightClass": "economy"
}
```

**Response**:
```json
{
  "co2Amount": 125.0,
  "unit": "kg CO2e",
  "emissionFactor": 0.25,
  "calculationMethod": "IATA_standard",
  "breakdown": {
    "directEmissions": 100.0,
    "indirectEmissions": 25.0
  }
}
```

### 4.3 Credit Endpoints

#### GET /credits/balance

Get current credit balance.

```http
GET /credits/balance
Authorization: Bearer <token>
```

**Response**:
```json
{
  "balance": 2.5,
  "unit": "tCO2e",
  "breakdown": {
    "earned": 1.2,
    "purchased": 1.5,
    "sold": -0.2
  },
  "value": {
    "usd": 63.75,
    "marketPrice": 25.5
  }
}
```

#### POST /credits/earn

Earn credits from an offset action.

```http
POST /credits/earn
Authorization: Bearer <token>
Content-Type: application/json

{
  "source": "tree_planting",
  "description": "Planted 10 oak trees",
  "quantity": 10,
  "location": {
    "lat": 37.5665,
    "lng": 126.9780
  },
  "proof": "ipfs://Qm...",
  "timestamp": "2025-01-15T10:00:00Z"
}
```

**Response**:
```json
{
  "creditId": "CRD-2025-0001",
  "amount": 0.5,
  "status": "pending_verification",
  "estimatedVerificationTime": "24h",
  "verifierAssigned": "WIA-Verifier-001"
}
```

#### POST /credits/retire

Permanently retire credits.

```http
POST /credits/retire
Authorization: Bearer <token>
Content-Type: application/json

{
  "amount": 0.5,
  "reason": "personal_offset_2025_jan",
  "note": "Offsetting January emissions"
}
```

**Response**:
```json
{
  "retirementId": "RET-2025-0001",
  "amount": 0.5,
  "retiredAt": "2025-01-15T12:00:00Z",
  "blockchainTx": "0xabc123...",
  "certificate": "https://wia.live/certificates/RET-2025-0001.pdf"
}
```

#### POST /credits/transfer

Transfer credits to another user.

```http
POST /credits/transfer
Authorization: Bearer <token>
Content-Type: application/json

{
  "recipient": "USER-2025-500",
  "amount": 0.3,
  "note": "Gift for birthday"
}
```

**Response**:
```json
{
  "transferId": "TRF-2025-0001",
  "amount": 0.3,
  "recipient": "USER-2025-500",
  "status": "completed",
  "blockchainTx": "0xdef456..."
}
```

### 4.4 Trading Endpoints

#### POST /trading/list

List credits for sale on marketplace.

```http
POST /trading/list
Authorization: Bearer <token>
Content-Type: application/json

{
  "amount": 1.0,
  "pricePerCredit": 26.0,
  "currency": "USD",
  "minPurchase": 0.1,
  "expiresAt": "2025-02-15T00:00:00Z"
}
```

**Response**:
```json
{
  "listingId": "LST-2025-0001",
  "amount": 1.0,
  "pricePerCredit": 26.0,
  "status": "active",
  "listedAt": "2025-01-15T10:00:00Z"
}
```

#### POST /trading/buy

Purchase credits from marketplace.

```http
POST /trading/buy
Authorization: Bearer <token>
Content-Type: application/json

{
  "listingId": "LST-2025-0001",
  "amount": 0.5
}
```

**Response**:
```json
{
  "purchaseId": "PUR-2025-0001",
  "amount": 0.5,
  "totalPrice": 13.0,
  "seller": "USER-2025-100",
  "status": "completed",
  "blockchainTx": "0xghi789..."
}
```

#### GET /trading/market/price

Get current market price.

```http
GET /trading/market/price
Authorization: Bearer <token>
```

**Response**:
```json
{
  "currentPrice": 25.5,
  "currency": "USD",
  "unit": "tCO2e",
  "change24h": 0.5,
  "changePercent24h": 2.0,
  "volume24h": 1250.5,
  "timestamp": "2025-01-15T12:00:00Z"
}
```

#### GET /trading/market/history

Get historical market data.

```http
GET /trading/market/history?period=30d&interval=1d
Authorization: Bearer <token>
```

**Response**:
```json
{
  "period": "30d",
  "interval": "1d",
  "data": [
    {
      "timestamp": "2025-01-01T00:00:00Z",
      "open": 24.5,
      "high": 25.2,
      "low": 24.0,
      "close": 25.0,
      "volume": 850.3
    },
    {
      "timestamp": "2025-01-02T00:00:00Z",
      "open": 25.0,
      "high": 25.8,
      "low": 24.8,
      "close": 25.5,
      "volume": 920.7
    }
  ]
}
```

#### GET /trading/listings

Get active marketplace listings.

```http
GET /trading/listings?sort=price&order=asc&limit=20
Authorization: Bearer <token>
```

**Response**:
```json
{
  "listings": [
    {
      "listingId": "LST-2025-0001",
      "seller": "USER-2025-100",
      "amount": 1.0,
      "pricePerCredit": 24.5,
      "listedAt": "2025-01-14T10:00:00Z"
    },
    {
      "listingId": "LST-2025-0002",
      "seller": "USER-2025-200",
      "amount": 2.5,
      "pricePerCredit": 25.0,
      "listedAt": "2025-01-15T08:00:00Z"
    }
  ],
  "pagination": {
    "total": 150,
    "page": 1,
    "pageSize": 20
  }
}
```

### 4.5 Blockchain Endpoints

#### POST /blockchain/mint

Mint blockchain token for credits.

```http
POST /blockchain/mint
Authorization: Bearer <token>
Content-Type: application/json

{
  "creditId": "CRD-2025-0001",
  "network": "polygon"
}
```

**Response**:
```json
{
  "tokenId": "CCM-2025-0001",
  "contractAddress": "0x742d35Cc...",
  "network": "polygon",
  "transactionHash": "0xabc123...",
  "status": "pending"
}
```

#### GET /blockchain/verify/:recordId

Verify record on blockchain.

```http
GET /blockchain/verify/CCR-2025-000001
Authorization: Bearer <token>
```

**Response**:
```json
{
  "recordId": "CCR-2025-000001",
  "verified": true,
  "blockchainTx": "0xdef456...",
  "blockNumber": 18234567,
  "timestamp": "2025-01-15T10:00:00Z",
  "confirmations": 150
}
```

#### GET /blockchain/transactions

Get blockchain transaction history.

```http
GET /blockchain/transactions?limit=50
Authorization: Bearer <token>
```

**Response**:
```json
{
  "transactions": [
    {
      "transactionHash": "0xabc123...",
      "type": "mint",
      "amount": 0.5,
      "timestamp": "2025-01-15T10:00:00Z",
      "status": "confirmed",
      "gasUsed": 65000
    },
    {
      "transactionHash": "0xdef456...",
      "type": "transfer",
      "amount": 0.2,
      "from": "0x742d35Cc...",
      "to": "0x8ba1f109...",
      "timestamp": "2025-01-15T14:00:00Z",
      "status": "confirmed",
      "gasUsed": 45000
    }
  ]
}
```

### 4.6 Corporate Endpoints

#### POST /corporate/offset-program

Create corporate offset program.

```http
POST /corporate/offset-program
Authorization: Bearer <token>
Content-Type: application/json

{
  "name": "2025 Q1 Carbon Neutrality",
  "targetAmount": 1000.0,
  "budget": 25000.0,
  "currency": "USD",
  "startDate": "2025-01-01",
  "endDate": "2025-03-31",
  "autoExecute": true
}
```

**Response**:
```json
{
  "programId": "PROG-2025-0001",
  "status": "active",
  "purchased": 0,
  "remaining": 1000.0,
  "spent": 0,
  "averagePrice": 0
}
```

#### POST /corporate/bulk-purchase

Purchase credits in bulk.

```http
POST /corporate/bulk-purchase
Authorization: Bearer <token>
Content-Type: application/json

{
  "amount": 100.0,
  "maxPricePerCredit": 26.0,
  "currency": "USD"
}
```

**Response**:
```json
{
  "purchaseId": "BULK-2025-0001",
  "amount": 100.0,
  "totalPrice": 2550.0,
  "averagePrice": 25.5,
  "sellers": 25,
  "status": "completed",
  "transactions": [...]
}
```

#### GET /corporate/report/:period

Generate offset report.

```http
GET /corporate/report/2025-q1
Authorization: Bearer <token>
```

**Response**:
```json
{
  "period": "2025-Q1",
  "totalPurchased": 850.0,
  "totalSpent": 21675.0,
  "averagePrice": 25.5,
  "creditsRetired": 850.0,
  "offsetPercentage": 100,
  "breakdown": {
    "tree_planting": 400.0,
    "renewable_energy": 300.0,
    "other": 150.0
  },
  "reportUrl": "https://wia.live/reports/2025-q1.pdf"
}
```

### 4.7 Analytics Endpoints

#### GET /analytics/insights

Get emission insights and recommendations.

```http
GET /analytics/insights
Authorization: Bearer <token>
```

**Response**:
```json
{
  "summary": {
    "monthlyAverage": 560.5,
    "trend": "decreasing",
    "changePercent": -8.5
  },
  "topCategories": [
    {
      "category": "transportation",
      "percentage": 35,
      "recommendation": "Consider using public transit or electric vehicle"
    },
    {
      "category": "energy",
      "percentage": 28,
      "recommendation": "Switch to renewable energy provider"
    }
  ],
  "achievements": [
    {
      "id": "low_carbon_week",
      "achieved": true,
      "date": "2025-01-15"
    }
  ]
}
```

#### GET /analytics/compare

Compare to peers.

```http
GET /analytics/compare?category=transportation
Authorization: Bearer <token>
```

**Response**:
```json
{
  "userEmissions": 135.5,
  "peerAverage": 180.2,
  "percentile": 75,
  "comparison": "better_than_average",
  "savingsVsAverage": 44.7,
  "rank": 1523,
  "totalUsers": 10000
}
```

---

## Footprint Tracking

### 5.1 Emission Tracking Example

```typescript
import { CarbonCreditMicro } from 'wia-carbon-credit-micro';

const carbon = new CarbonCreditMicro({
  apiKey: 'your-api-key',
  emissionFactorRegion: 'US'
});

// Track car commute
const emission = await carbon.trackEmission({
  category: 'transportation',
  subcategory: 'car',
  distance: 25,
  distanceUnit: 'km',
  fuelType: 'gasoline',
  timestamp: new Date()
});

console.log('CO2 emitted:', emission.co2Amount, 'kg');
```

### 5.2 Auto-Tracking Integration

```python
from wia_carbon_credit_micro import CarbonCreditMicro

carbon = CarbonCreditMicro(
    api_key='your-api-key',
    enable_auto_tracking=True
)

# Connect to smart home devices
await carbon.connect_smart_meter('meter-id-123')
await carbon.connect_vehicle_telematics('vehicle-vin')

# Automatic tracking happens in background
# Retrieve daily summary
footprint = await carbon.get_daily_footprint(date.today())
print(f"Today's emissions: {footprint.total_emissions} kg CO2e")
```

---

## Credit Management

### 6.1 Earning Credits

```typescript
// Earn credits from tree planting
const earned = await carbon.earnCredit({
  source: 'tree_planting',
  description: 'Planted 10 oak trees in community park',
  quantity: 10,
  location: { lat: 37.5665, lng: 126.9780 },
  proof: 'ipfs://Qm...',
  timestamp: new Date()
});

console.log('Credits earned:', earned.amount, 'tCO2e');
console.log('Status:', earned.status);
```

### 6.2 Credit Portfolio Management

```python
# Get credit balance
balance = await carbon.get_credit_balance()
print(f"Total balance: {balance.balance} tCO2e")
print(f"Portfolio value: ${balance.value.usd}")

# Retire credits for carbon neutrality
retirement = await carbon.retire_credits(
    amount=0.5,
    reason='personal_offset_january_2025'
)
print(f"Certificate: {retirement.certificate}")
```

---

## Trading Platform

### 7.1 Selling Credits

```typescript
// List credits for sale
const listing = await carbon.listCreditsForSale({
  amount: 1.0,
  pricePerCredit: 26.0,
  currency: 'USD',
  minPurchase: 0.1
});

console.log('Listed:', listing.listingId);

// Monitor sales
carbon.on('credit:sold', (sale) => {
  console.log('Sold:', sale.amount, 'tCO2e for $', sale.totalPrice);
});
```

### 7.2 Buying Credits

```python
# Get current market price
price = await carbon.get_market_price()
print(f"Current price: ${price.current_price} per tCO2e")

# Purchase credits
if price.current_price < 26.0:
    purchase = await carbon.buy_credits(
        listing_id='LST-2025-0001',
        amount=0.5
    )
    print(f"Purchase complete: {purchase.purchase_id}")
```

### 7.3 Market Analysis

```typescript
// Get 30-day price history
const history = await carbon.getMarketHistory({ period: '30d' });

// Calculate average price
const avgPrice = history.data.reduce((sum, day) =>
  sum + day.close, 0) / history.data.length;

console.log('30-day average price:', avgPrice);

// Set up price alerts
carbon.on('price:changed', (data) => {
  if (data.changePercent > 5) {
    console.log('Significant price movement:', data.changePercent, '%');
  }
});
```

---

## Authentication

### 8.1 OAuth 2.0 Flow

```typescript
// Initialize with OAuth
const carbon = new CarbonCreditMicro({
  authMethod: 'oauth2',
  clientId: 'your-client-id',
  clientSecret: 'your-client-secret'
});

// Obtain access token
const authUrl = carbon.getAuthorizationUrl({
  redirectUri: 'https://yourapp.com/callback',
  scope: ['footprint:read', 'credits:write', 'trading:execute']
});

// After user authorization
const token = await carbon.exchangeAuthCode(authCode);
carbon.setAccessToken(token.accessToken);
```

### 8.2 API Key Authentication

```python
# Simple API key authentication
carbon = CarbonCreditMicro(
    api_key='wccm_live_abc123...',
    api_secret='wccm_secret_xyz789...'
)

# All requests automatically authenticated
balance = await carbon.get_credit_balance()
```

### 8.3 JWT Token Authentication

```typescript
// JWT token for server-to-server
const carbon = new CarbonCreditMicro({
  authMethod: 'jwt',
  privateKey: fs.readFileSync('private-key.pem')
});

// Token auto-refreshed on expiration
```

---

## Error Handling

### 9.1 Error Codes

```typescript
enum CarbonCreditErrorCode {
  // Footprint errors (1xxx)
  INVALID_EMISSION_DATA = 1001,
  CALCULATION_FAILED = 1002,
  TRACKING_DISABLED = 1003,

  // Credit errors (2xxx)
  INSUFFICIENT_CREDITS = 2001,
  CREDIT_VERIFICATION_FAILED = 2002,
  CREDIT_ALREADY_RETIRED = 2003,
  INVALID_CREDIT_SOURCE = 2004,

  // Trading errors (3xxx)
  LISTING_NOT_FOUND = 3001,
  INSUFFICIENT_FUNDS = 3002,
  PRICE_CHANGED = 3003,
  MARKET_CLOSED = 3004,

  // Blockchain errors (4xxx)
  BLOCKCHAIN_UNAVAILABLE = 4001,
  TRANSACTION_FAILED = 4002,
  INSUFFICIENT_GAS = 4003,
  WALLET_NOT_CONNECTED = 4004,

  // Authentication errors (5xxx)
  INVALID_API_KEY = 5001,
  TOKEN_EXPIRED = 5002,
  INSUFFICIENT_PERMISSIONS = 5003
}
```

### 9.2 Error Handling Example

```typescript
try {
  await carbon.buyCredits('LST-2025-0001', 10.0);
} catch (error) {
  if (error instanceof CarbonCreditError) {
    switch (error.code) {
      case CarbonCreditErrorCode.INSUFFICIENT_CREDITS:
        console.log('Seller has insufficient credits');
        break;
      case CarbonCreditErrorCode.INSUFFICIENT_FUNDS:
        console.log('Insufficient funds for purchase');
        break;
      case CarbonCreditErrorCode.PRICE_CHANGED:
        console.log('Price changed, please retry');
        break;
      default:
        console.error('Unknown error:', error.message);
    }
  }
}
```

---

## Usage Examples

### 10.1 Complete Daily Tracking Flow

```typescript
import { CarbonCreditMicro } from 'wia-carbon-credit-micro';

async function trackDailyActivities() {
  const carbon = new CarbonCreditMicro({
    apiKey: 'your-api-key',
    emissionFactorRegion: 'US'
  });

  // Morning commute
  await carbon.trackEmission({
    category: 'transportation',
    subcategory: 'car',
    distance: 25,
    fuelType: 'gasoline'
  });

  // Work lunch
  await carbon.trackEmission({
    category: 'food',
    subcategory: 'restaurant',
    mealType: 'lunch',
    foodItems: ['chicken', 'rice', 'vegetables']
  });

  // Evening shopping
  await carbon.trackEmission({
    category: 'consumption',
    subcategory: 'clothing',
    items: [{ type: 'shirt', quantity: 2 }]
  });

  // Get daily summary
  const footprint = await carbon.getDailyFootprint(new Date());
  console.log('Total emissions:', footprint.totalEmissions, 'kg CO2e');

  // Compare to average
  const insights = await carbon.getEmissionInsights();
  console.log('vs National average:', insights.vsNationalAverage);
}
```

### 10.2 Automated Carbon Neutrality

```python
from wia_carbon_credit_micro import CarbonCreditMicro
from datetime import date, timedelta

async def achieve_carbon_neutrality():
    carbon = CarbonCreditMicro(api_key='your-api-key')

    # Get last month's footprint
    last_month_start = date.today().replace(day=1) - timedelta(days=1)
    last_month_start = last_month_start.replace(day=1)
    last_month_end = date.today().replace(day=1) - timedelta(days=1)

    footprint = await carbon.get_footprint_history(
        last_month_start,
        last_month_end
    )

    # Convert kg to tonnes
    total_emissions_tonnes = footprint.total_emissions / 1000

    # Check credit balance
    balance = await carbon.get_credit_balance()

    # Purchase needed credits
    if balance.balance < total_emissions_tonnes:
        needed = total_emissions_tonnes - balance.balance
        purchase = await carbon.buy_credits_market_price(needed)
        print(f"Purchased {needed} tCO2e for ${purchase.total_price}")

    # Retire credits to offset emissions
    retirement = await carbon.retire_credits(
        amount=total_emissions_tonnes,
        reason=f'offset_{last_month_start.strftime("%Y_%m")}'
    )

    print(f"Carbon neutral! Certificate: {retirement.certificate}")
```

### 10.3 Corporate Bulk Trading

```typescript
async function corporateOffsetProgram() {
  const carbon = new CarbonCreditMicro({
    apiKey: 'corporate-api-key',
    authMethod: 'oauth2'
  });

  // Create annual offset program
  const program = await carbon.createOffsetProgram({
    name: '2025 Carbon Neutrality Initiative',
    targetAmount: 10000.0,  // 10,000 tCO2e
    budget: 250000.0,       // $250,000
    currency: 'USD',
    startDate: '2025-01-01',
    endDate: '2025-12-31',
    autoExecute: true,
    maxPricePerCredit: 26.0
  });

  // Monitor program progress
  carbon.on('program:purchase', (purchase) => {
    console.log('Purchased:', purchase.amount, 'tCO2e');
    console.log('Program progress:', purchase.progress, '%');
  });

  // Generate quarterly report
  const report = await carbon.generateOffsetReport('2025-Q1');
  console.log('Q1 Report:', report.reportUrl);
  console.log('Credits purchased:', report.totalPurchased);
  console.log('Average price:', report.averagePrice);
}
```

### 10.4 Real-Time Market Trading

```python
from wia_carbon_credit_micro import CarbonCreditMicro

async def real_time_trading():
    carbon = CarbonCreditMicro(
        api_key='your-api-key',
        enable_real_time_market=True
    )

    # Set up WebSocket connection
    await carbon.connect_market_stream()

    # Trading strategy: Buy when price drops below $25
    def on_price_update(price_data):
        if price_data.current_price < 25.0:
            # Buy 1 tCO2e
            carbon.buy_credits_market_price(1.0)
            print(f"Bought 1 tCO2e at ${price_data.current_price}")

    carbon.on('market:price', on_price_update)

    # Trading strategy: Sell when price goes above $27
    def on_price_high(price_data):
        if price_data.current_price > 27.0:
            balance = carbon.get_credit_balance_sync()
            if balance.balance > 0.5:
                carbon.list_credits_for_sale(
                    amount=0.5,
                    price_per_credit=27.0
                )
                print(f"Listed 0.5 tCO2e at ${27.0}")

    carbon.on('market:price', on_price_high)

    # Keep connection alive
    await carbon.keep_alive()
```

---

## References

### Related Standards

- [WIA Carbon Credit Micro Data Format (Phase 1)](/carbon-credit-micro/spec/PHASE-1-DATA-FORMAT.md)
- [WIA Carbon Credit Micro Protocol (Phase 3)](/carbon-credit-micro/spec/PHASE-3-PROTOCOL.md)
- [GHG Protocol](https://ghgprotocol.org/)
- [ISO 14064 - Greenhouse Gas Accounting](https://www.iso.org/standard/66453.html)

### Blockchain Standards

- [ERC-20 Token Standard](https://eips.ethereum.org/EIPS/eip-20)
- [ERC-1155 Multi-Token Standard](https://eips.ethereum.org/EIPS/eip-1155)
- [W3C Verifiable Credentials](https://www.w3.org/TR/vc-data-model/)

### Carbon Standards

- [Verified Carbon Standard (VCS)](https://verra.org/programs/verified-carbon-standard/)
- [Gold Standard](https://www.goldstandard.org/)

---

<div align="center">

**WIA Carbon Credit Micro Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
