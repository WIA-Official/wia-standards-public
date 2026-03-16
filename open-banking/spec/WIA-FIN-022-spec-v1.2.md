# WIA-FIN-022: Open Banking Standard - Specification v1.2

**Status:** Stable
**Published:** 2025-11-01
**Category:** Finance/Economy
**Emoji:** 🏦
**Extends:** v1.1

## Changes from v1.1

This version adds advanced features including international payments, account aggregation optimizations, and AI-powered fraud detection.

## 1. International Payments

### 1.1 Cross-Border Payments

**Endpoint:** `POST /international-payment-consents`

**Request:**
```json
{
  "Data": {
    "Initiation": {
      "InstructionIdentification": "INTL-PAY-001",
      "InstructedAmount": {
        "Amount": "1000.00",
        "Currency": "USD"
      },
      "CurrencyOfTransfer": "USD",
      "ExchangeRateInformation": {
        "RateType": "Indicative"
      },
      "CreditorAccount": {
        "SchemeName": "IBAN",
        "Identification": "DE89370400440532013000",
        "Name": "International Supplier"
      },
      "CreditorAgent": {
        "SchemeName": "BICFI",
        "Identification": "DEUTDEFF"
      },
      "ChargeBearer": "Shared"
    }
  }
}
```

### 1.2 FX Rate Information

**Endpoint:** `GET /international-payment-consents/{ConsentId}/funds-confirmation`

Returns real-time FX rates and total charges.

## 2. Enhanced Transaction Data

### 2.1 Transaction Categorization

Transactions include AI-powered categorization:

**Response:**
```json
{
  "TransactionId": "123",
  "Amount": { "Amount": "25.50", "Currency": "GBP" },
  "MerchantDetails": {
    "MerchantName": "Coffee Shop",
    "MerchantCategoryCode": "5814"
  },
  "EnrichedData": {
    "Category": "Food & Dining",
    "Subcategory": "Coffee Shops",
    "Logo": "https://cdn.example.com/logos/coffeeshop.png",
    "RecurringPrediction": {
      "IsRecurring": true,
      "Frequency": "Weekly",
      "Confidence": 0.92
    }
  }
}
```

### 2.2 Carbon Footprint Data

Optional carbon tracking for transactions:

```json
{
  "TransactionId": "456",
  "EnvironmentalData": {
    "CarbonEmissions": {
      "Amount": 2.5,
      "Unit": "kg_co2e"
    },
    "SustainabilityScore": 65
  }
}
```

## 3. Account Aggregation Enhancements

### 3.1 Bulk Account Requests

**Endpoint:** `POST /bulk-accounts`

Request data from multiple accounts in single call:

**Request:**
```json
{
  "AccountIds": ["ACC-001", "ACC-002", "ACC-003"],
  "DataTypes": ["balances", "transactions"],
  "TransactionPeriod": {
    "FromDateTime": "2025-01-01T00:00:00Z",
    "ToDateTime": "2025-12-31T23:59:59Z"
  }
}
```

### 3.2 Delta Synchronization

Efficient sync with `since` parameter:

**Endpoint:** `GET /accounts/{AccountId}/transactions?since={timestamp}`

Returns only transactions modified since timestamp.

## 4. AI-Powered Features

### 4.1 Fraud Detection Signals

ASPSPs MAY include fraud risk scores:

**Response Headers:**
```
x-fraud-risk-score: 0.05
x-fraud-indicators: unusual_time,new_payee
```

### 4.2 Spending Insights

**Endpoint:** `GET /accounts/{AccountId}/insights`

**Response:**
```json
{
  "Data": {
    "Period": "Month",
    "Insights": [
      {
        "Type": "SpendingPattern",
        "Category": "Dining",
        "AverageAmount": "150.00",
        "Trend": "Increasing",
        "Recommendation": "You're spending 20% more on dining this month"
      },
      {
        "Type": "UnusualActivity",
        "Description": "Large transaction detected",
        "Amount": "500.00",
        "Confidence": 0.85
      }
    ]
  }
}
```

## 5. Premium API Tiers

### 5.1 Tier Levels

- **Basic**: Standard rate limits (10 req/sec)
- **Premium**: Higher limits (50 req/sec), webhooks
- **Enterprise**: Dedicated infrastructure (200 req/sec), SLA

### 5.2 Tier Selection

**Header:**
```
x-api-tier: premium
```

## 6. Advanced Security

### 6.1 Behavioral Biometrics

Support for behavioral biometric signals:

**Request Header:**
```
x-biometric-signature: {behavioral_hash}
```

### 6.2 Device Binding

Bind consents to specific devices:

**Consent Parameter:**
```json
{
  "DeviceBinding": {
    "DeviceId": "device-fingerprint-123",
    "DeviceType": "mobile_ios",
    "BindingRequired": true
  }
}
```

## 7. Open Finance Extensions

### 7.1 Investment Accounts

**Endpoint:** `GET /investment-accounts/{AccountId}`

**Response:**
```json
{
  "Data": {
    "InvestmentAccountId": "INV-123",
    "AccountName": "Investment Portfolio",
    "Holdings": [
      {
        "SecurityId": "US0378331005",
        "SecurityName": "Apple Inc.",
        "Quantity": 10,
        "CurrentValue": {
          "Amount": "1750.00",
          "Currency": "USD"
        }
      }
    ],
    "TotalValue": {
      "Amount": "25000.00",
      "Currency": "USD"
    }
  }
}
```

### 7.2 Pension Data

**Endpoint:** `GET /pension-accounts/{AccountId}`

Returns pension pot value, contributions, and projections.

## 8. Compliance Updates

### 8.1 GDPR Enhancements

- Data portability in JSON and CSV formats
- Automated data deletion on consent revocation
- Detailed audit logs accessible to PSU

### 8.2 Accessibility

APIs MUST support:
- Clear error messages suitable for screen readers
- Alternative text for all visual elements
- WCAG 2.1 Level AA compliance in UI flows

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
