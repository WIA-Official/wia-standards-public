# WIA-UNI-013: Currency Integration Standard v1.1

**Status:** Official Standard
**Version:** 1.1.0
**Date:** March 15, 2026
**Authors:** WIA Standards Committee

---

## Changes from v1.0

### New Features
- **Webhook Support:** Real-time event notifications for transaction status changes
- **Batch Conversion API:** Support for processing multiple currency conversions in a single request
- **Enhanced Analytics:** Detailed transaction analytics and reporting endpoints
- **Mobile SDK:** Official mobile SDK for iOS and Android

### Improvements
- Reduced API response times (average 30% faster)
- Enhanced error messages with detailed troubleshooting guidance
- Improved rate limiting with burst capacity
- Extended transaction history retention (from 1 year to 3 years)

### Bug Fixes
- Fixed edge case in exchange rate calculation during market volatility
- Resolved timezone handling issues in historical rate queries
- Corrected fee calculation rounding errors for micro-transactions

---

## New API Endpoints

### Webhook Management

#### Register Webhook
```
POST /webhooks
Content-Type: application/json

Request Body:
{
  "url": "https://example.com/webhook",
  "events": ["conversion.completed", "conversion.failed", "rate.updated"],
  "secret": "webhook_secret_key"
}

Response:
{
  "success": true,
  "data": {
    "webhookId": "WH-2026-001",
    "url": "https://example.com/webhook",
    "events": ["conversion.completed", "conversion.failed", "rate.updated"],
    "status": "ACTIVE"
  }
}
```

### Batch Conversion

```
POST /conversions/batch
Content-Type: application/json

Request Body:
{
  "conversions": [
    {
      "fromCurrency": "KRW",
      "toCurrency": "KPW",
      "amount": 1000000,
      "reference": "REF-001"
    },
    {
      "fromCurrency": "KRW",
      "toCurrency": "KPW",
      "amount": 500000,
      "reference": "REF-002"
    }
  ]
}

Response:
{
  "success": true,
  "data": {
    "batchId": "BATCH-2026-001",
    "status": "PROCESSING",
    "totalConversions": 2,
    "estimatedCompletionTime": "2026-03-15T10:35:00Z"
  }
}
```

### Analytics API

```
GET /analytics/transaction-summary
Query Parameters:
  - startDate: ISO 8601 date
  - endDate: ISO 8601 date
  - currency: KRW | KPW | ALL

Response:
{
  "success": true,
  "data": {
    "totalVolume": 500000000,
    "totalTransactions": 15000,
    "averageTransactionSize": 33333,
    "conversionsByType": {
      "INDIVIDUAL_TRANSFER": 12000,
      "BUSINESS_PAYMENT": 2500,
      "TRADE_SETTLEMENT": 500
    },
    "dailyBreakdown": [...]
  }
}
```

---

## Mobile SDK

### iOS Installation
```swift
// CocoaPods
pod 'WIA-CurrencyIntegration', '~> 1.1.0'

// Swift Package Manager
.package(url: "https://github.com/WIA-Official/currency-integration-ios", from: "1.1.0")
```

### Android Installation
```gradle
// build.gradle
implementation 'org.wia:currency-integration:1.1.0'
```

---

All other specifications remain unchanged from v1.0. See [WIA-UNI-013-spec-v1.0.md](./WIA-UNI-013-spec-v1.0.md) for full documentation.

**© 2026 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
