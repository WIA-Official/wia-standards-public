# WIA-FIN-013 Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Stable  
**Last Updated:** December 2025

---

## Table of Contents

1. [Overview](#overview)
2. [Core Data Types](#core-data-types)
3. [Transaction Format](#transaction-format)
4. [Wallet Format](#wallet-format)
5. [Token Format](#token-format)
6. [QR Code Format](#qr-code-format)
7. [Biometric Data Format](#biometric-data-format)
8. [Error Format](#error-format)
9. [Validation Rules](#validation-rules)
10. [Examples](#examples)

---

## Overview

The WIA-FIN-013 Phase 1 specification defines standardized JSON schemas for all mobile payment data structures. This ensures interoperability across platforms, languages, and implementations.

### Design Principles

- **Extensibility:** Support for custom fields without breaking compatibility
- **Validation:** Strict schema validation for security
- **Versioning:** Backward-compatible schema evolution
- **Efficiency:** Minimal data size for mobile networks
- **Security:** No sensitive data in plain text

---

## Core Data Types

### Amount

```json
{
  "value": "99.99",
  "currency": "USD",
  "minorUnits": 2
}
```

**Fields:**
- `value` (string, required): Decimal amount as string to avoid floating-point errors
- `currency` (string, required): ISO 4217 3-letter currency code
- `minorUnits` (integer, required): Number of decimal places (2 for USD, 0 for JPY)

### Timestamp

```json
{
  "timestamp": "2025-12-25T14:30:00.000Z",
  "timezone": "UTC"
}
```

**Format:** ISO 8601 UTC timestamp

### Address

```json
{
  "line1": "123 Main St",
  "line2": "Apt 4B",
  "city": "San Francisco",
  "state": "CA",
  "postalCode": "94102",
  "country": "US"
}
```

### Device Info

```json
{
  "deviceId": "dev_abc123...",
  "platform": "iOS",
  "osVersion": "17.2",
  "appVersion": "2.5.0",
  "manufacturer": "Apple",
  "model": "iPhone 15 Pro"
}
```

---

## Transaction Format

### Payment Transaction

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-013",
  "transactionId": "tx_1234567890abcdef",
  "type": "payment",
  "method": "nfc",
  "status": "completed",
  "amount": {
    "value": "99.99",
    "currency": "USD",
    "minorUnits": 2
  },
  "merchant": {
    "id": "merchant_xyz789",
    "name": "Coffee Shop",
    "category": "5812",
    "location": {
      "latitude": 37.7749,
      "longitude": -122.4194,
      "address": {
        "line1": "456 Market St",
        "city": "San Francisco",
        "state": "CA",
        "postalCode": "94102",
        "country": "US"
      }
    }
  },
  "customer": {
    "customerId": "cust_abc123",
    "email": "user@example.com",
    "phone": "+1-555-0100"
  },
  "paymentMethod": {
    "type": "card",
    "brand": "visa",
    "last4": "4242",
    "expiryMonth": 12,
    "expiryYear": 2028,
    "tokenId": "tok_secure_abc123..."
  },
  "authentication": {
    "method": "biometric",
    "type": "face_id",
    "timestamp": "2025-12-25T14:30:00.000Z",
    "deviceCVM": true
  },
  "device": {
    "deviceId": "dev_abc123...",
    "platform": "iOS",
    "osVersion": "17.2",
    "appVersion": "2.5.0"
  },
  "network": {
    "acquirer": "Bank of America",
    "processor": "First Data",
    "cardNetwork": "Visa"
  },
  "cryptogram": {
    "type": "EMV_3DS",
    "value": "AgAAAAAAAIIAAAAAVJECAA==",
    "transactionCounter": 42
  },
  "timestamps": {
    "initiated": "2025-12-25T14:30:00.000Z",
    "authorized": "2025-12-25T14:30:01.234Z",
    "completed": "2025-12-25T14:30:02.567Z"
  },
  "fees": {
    "processing": {
      "value": "0.29",
      "currency": "USD"
    },
    "service": {
      "value": "0.00",
      "currency": "USD"
    }
  },
  "metadata": {
    "orderId": "ORDER-12345",
    "receiptUrl": "https://example.com/receipts/tx_123",
    "custom": {}
  }
}
```

### Transaction Status Values

- `initiated`: Transaction started but not authorized
- `authorizing`: Awaiting authorization
- `authorized`: Approved by issuer
- `completed`: Successfully completed
- `failed`: Transaction failed
- `declined`: Declined by issuer
- `cancelled`: Cancelled by user
- `refunded`: Payment refunded
- `partially_refunded`: Partial refund issued

### Payment Method Types

- `nfc`: NFC contactless payment
- `qr_code`: QR code payment
- `card`: Card-not-present (online)
- `wallet`: Mobile wallet balance
- `bank_transfer`: Direct bank transfer
- `crypto`: Cryptocurrency payment

---

## Wallet Format

### Digital Wallet

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-013",
  "walletId": "wallet_abc123...",
  "userId": "user_xyz789",
  "type": "mobile_wallet",
  "status": "active",
  "balance": {
    "available": {
      "value": "1250.50",
      "currency": "USD"
    },
    "pending": {
      "value": "50.00",
      "currency": "USD"
    },
    "reserved": {
      "value": "0.00",
      "currency": "USD"
    }
  },
  "paymentMethods": [
    {
      "id": "pm_card_123",
      "type": "card",
      "brand": "visa",
      "last4": "4242",
      "expiryMonth": 12,
      "expiryYear": 2028,
      "isDefault": true,
      "tokenId": "tok_abc123...",
      "billingAddress": {
        "line1": "123 Main St",
        "city": "San Francisco",
        "state": "CA",
        "postalCode": "94102",
        "country": "US"
      }
    },
    {
      "id": "pm_bank_456",
      "type": "bank_account",
      "bankName": "Chase",
      "accountType": "checking",
      "last4": "6789",
      "routingNumber": "021000021",
      "isDefault": false
    }
  ],
  "settings": {
    "defaultCurrency": "USD",
    "allowNFC": true,
    "allowQRCode": true,
    "biometricEnabled": true,
    "notificationsEnabled": true,
    "autoReload": {
      "enabled": false,
      "threshold": "50.00",
      "amount": "100.00",
      "sourcePaymentMethodId": "pm_card_123"
    }
  },
  "limits": {
    "daily": {
      "transaction": {
        "value": "5000.00",
        "currency": "USD"
      },
      "withdrawal": {
        "value": "1000.00",
        "currency": "USD"
      }
    },
    "monthly": {
      "transaction": {
        "value": "50000.00",
        "currency": "USD"
      }
    }
  },
  "created": "2025-01-01T00:00:00.000Z",
  "updated": "2025-12-25T14:30:00.000Z"
}
```

---

## Token Format

### Payment Token

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-013",
  "tokenId": "tok_1234567890abcdef",
  "type": "payment_token",
  "status": "active",
  "deviceBinding": {
    "deviceId": "dev_abc123...",
    "deviceFingerprint": "fp_xyz789...",
    "bindingMethod": "device_account_number"
  },
  "cardDetails": {
    "tokenPAN": "4900000000001234",
    "brand": "visa",
    "last4DigitsRealPAN": "4242",
    "expiryMonth": 12,
    "expiryYear": 2028,
    "cardholderName": "JOHN DOE"
  },
  "tokenization": {
    "requestor": "apple_pay",
    "requestorId": "40010000000",
    "tokenProvider": "visa_token_service",
    "tokenReferenceId": "DWSPMC00000000001234567890"
  },
  "securityCodes": {
    "cvv": "encrypted_cvv_value",
    "dynamicCVV": true
  },
  "restrictions": {
    "singleUse": false,
    "maxAmount": {
      "value": "10000.00",
      "currency": "USD"
    },
    "allowedMerchantCategories": ["5812", "5411"],
    "allowedCountries": ["US", "CA", "GB"],
    "expiresAt": "2028-12-31T23:59:59.999Z"
  },
  "created": "2025-01-15T10:00:00.000Z",
  "lastUsed": "2025-12-25T14:30:00.000Z"
}
```

### Token Lifecycle Events

```json
{
  "tokenId": "tok_1234567890abcdef",
  "event": "token_activated",
  "timestamp": "2025-01-15T10:00:00.000Z",
  "metadata": {
    "activatedBy": "user",
    "device": "iPhone 15 Pro"
  }
}
```

**Event Types:**
- `token_created`: Token provisioned
- `token_activated`: Token activated for use
- `token_suspended`: Temporarily suspended
- `token_resumed`: Reactivated after suspension
- `token_deleted`: Permanently deleted
- `token_expired`: Token expired

---

## QR Code Format

### Static QR Code

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-013",
  "qrType": "static",
  "merchantId": "merchant_xyz789",
  "merchantName": "Coffee Shop",
  "qrCodeId": "qr_static_abc123",
  "paymentDetails": {
    "amount": {
      "value": "5.00",
      "currency": "USD",
      "editable": false
    },
    "description": "Coffee",
    "reference": "MENU-ITEM-001"
  },
  "expiresAt": null,
  "created": "2025-01-01T00:00:00.000Z"
}
```

### Dynamic QR Code

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-013",
  "qrType": "dynamic",
  "transactionId": "tx_1234567890abcdef",
  "merchantId": "merchant_xyz789",
  "qrCodeId": "qr_dynamic_xyz123",
  "paymentDetails": {
    "amount": {
      "value": "125.50",
      "currency": "USD",
      "editable": false
    },
    "description": "Restaurant bill #4231",
    "items": [
      {
        "name": "Burger",
        "quantity": 2,
        "unitPrice": "15.00",
        "total": "30.00"
      },
      {
        "name": "Fries",
        "quantity": 2,
        "unitPrice": "5.00",
        "total": "10.00"
      }
    ],
    "tax": "10.05",
    "tip": "18.00",
    "total": "125.50"
  },
  "expiresAt": "2025-12-25T15:00:00.000Z",
  "created": "2025-12-25T14:30:00.000Z",
  "maxScans": 1,
  "scannedCount": 0
}
```

---

## Biometric Data Format

### Biometric Authentication

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-013",
  "authenticationId": "auth_abc123...",
  "type": "biometric",
  "method": "fingerprint",
  "result": "success",
  "confidence": 98.7,
  "deviceInfo": {
    "deviceId": "dev_abc123...",
    "biometricType": "touch_id",
    "sensorVersion": "gen3"
  },
  "biometricData": {
    "templateHash": "hash_xyz789...",
    "encryptedData": "encrypted_bio_data...",
    "algorithm": "SHA-256"
  },
  "liveness": {
    "detected": true,
    "score": 99.2
  },
  "timestamp": "2025-12-25T14:30:00.000Z",
  "expiresAt": "2025-12-25T14:35:00.000Z"
}
```

**Biometric Methods:**
- `fingerprint`: Fingerprint scan
- `face_recognition`: Facial recognition
- `iris_scan`: Iris scanning
- `voice_recognition`: Voice biometrics
- `palm_print`: Palm print recognition

---

## Error Format

### Error Response

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-013",
  "error": {
    "code": "insufficient_funds",
    "message": "Insufficient funds in account",
    "type": "payment_error",
    "details": {
      "availableBalance": "45.00",
      "requestedAmount": "99.99",
      "currency": "USD"
    },
    "timestamp": "2025-12-25T14:30:00.000Z",
    "requestId": "req_abc123...",
    "documentation": "https://docs.wiastandards.com/errors/insufficient_funds"
  }
}
```

**Error Types:**
- `validation_error`: Invalid input data
- `authentication_error`: Authentication failed
- `payment_error`: Payment processing error
- `network_error`: Network/connectivity issue
- `system_error`: Internal system error
- `security_error`: Security violation detected

**Common Error Codes:**
- `invalid_amount`: Amount is invalid
- `invalid_currency`: Currency not supported
- `insufficient_funds`: Not enough balance
- `card_declined`: Card declined by issuer
- `expired_token`: Token has expired
- `biometric_failed`: Biometric authentication failed
- `rate_limit_exceeded`: Too many requests

---

## Validation Rules

### Amount Validation

- Must be positive decimal number
- Maximum 2 decimal places for most currencies
- Minimum: 0.01 (or currency equivalent)
- Maximum: 999999.99

### Currency Validation

- Must be valid ISO 4217 code
- Must be in supported currency list

### Card Validation

- PAN: 13-19 digits (Luhn algorithm)
- Expiry: Month 1-12, Year >= current year
- CVV: 3-4 digits

### Token Validation

- Token ID: Unique, alphanumeric, 16-64 characters
- Must have valid expiry date
- Device binding required

---

## Examples

### Complete Payment Flow

```json
{
  "request": {
    "version": "1.0.0",
    "standard": "WIA-FIN-013",
    "type": "payment",
    "method": "nfc",
    "amount": {
      "value": "99.99",
      "currency": "USD"
    },
    "merchantId": "merchant_xyz789",
    "deviceId": "dev_abc123..."
  },
  "response": {
    "success": true,
    "transactionId": "tx_1234567890abcdef",
    "status": "completed",
    "timestamp": "2025-12-25T14:30:02.567Z"
  }
}
```

---

## Compliance

This specification complies with:
- ISO 20022 (Financial Services Messaging)
- EMVCo 3.0 Specification
- PCI DSS 4.0 Requirements
- GDPR Data Protection Standards

---

© 2025 WIA (World Certification Industry Association)  
License: MIT

## Advanced Data Structures

### P2P Transfer Format

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-013",
  "transferId": "p2p_1234567890abcdef",
  "type": "p2p_transfer",
  "status": "completed",
  "sender": {
    "userId": "user_abc123",
    "walletId": "wallet_sender_xyz",
    "name": "Alice Johnson",
    "email": "alice@example.com",
    "phone": "+1-555-0100"
  },
  "recipient": {
    "userId": "user_def456",
    "walletId": "wallet_recipient_abc",
    "name": "Bob Smith",
    "email": "bob@example.com",
    "phone": "+1-555-0200",
    "verificationStatus": "verified"
  },
  "amount": {
    "value": "50.00",
    "currency": "USD",
    "minorUnits": 2
  },
  "fees": {
    "sender": {
      "value": "0.00",
      "currency": "USD"
    },
    "recipient": {
      "value": "0.00",
      "currency": "USD"
    }
  },
  "message": "Lunch money 🍕",
  "memo": "Split bill from restaurant",
  "timestamps": {
    "initiated": "2025-12-25T14:30:00.000Z",
    "completed": "2025-12-25T14:30:02.000Z"
  },
  "metadata": {
    "referenceId": "REF-12345",
    "groupId": "group_restaurant_split",
    "tags": ["lunch", "split_bill"]
  }
}
```

### Subscription Payment

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-013",
  "subscriptionId": "sub_1234567890abcdef",
  "status": "active",
  "customer": {
    "customerId": "cust_abc123",
    "email": "user@example.com"
  },
  "plan": {
    "planId": "plan_premium",
    "name": "Premium Plan",
    "interval": "month",
    "intervalCount": 1,
    "amount": {
      "value": "9.99",
      "currency": "USD"
    }
  },
  "paymentMethod": {
    "type": "card",
    "tokenId": "tok_abc123...",
    "last4": "4242"
  },
  "billing": {
    "nextBillingDate": "2026-01-25T00:00:00.000Z",
    "billingDayOfMonth": 25,
    "prorationBehavior": "create_prorations"
  },
  "trialPeriod": {
    "active": false,
    "endDate": "2025-02-25T00:00:00.000Z"
  },
  "created": "2025-01-25T00:00:00.000Z",
  "currentPeriodStart": "2025-12-25T00:00:00.000Z",
  "currentPeriodEnd": "2026-01-25T00:00:00.000Z"
}
```

### Refund Format

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-013",
  "refundId": "ref_1234567890abcdef",
  "originalTransactionId": "tx_original_123",
  "status": "completed",
  "type": "full_refund",
  "amount": {
    "value": "99.99",
    "currency": "USD"
  },
  "reason": "customer_request",
  "reasonCode": "CUST_REQ_001",
  "reasonDescription": "Customer requested refund within return period",
  "initiatedBy": {
    "type": "merchant",
    "userId": "merchant_admin_123"
  },
  "refundMethod": {
    "type": "original_payment_method",
    "expectedArrival": "3-5_business_days"
  },
  "timestamps": {
    "requested": "2025-12-26T10:00:00.000Z",
    "approved": "2025-12-26T10:05:00.000Z",
    "completed": "2025-12-26T10:10:00.000Z"
  }
}
```

### Dispute/Chargeback Format

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-013",
  "disputeId": "dis_1234567890abcdef",
  "transactionId": "tx_disputed_123",
  "status": "under_review",
  "type": "chargeback",
  "reason": "fraudulent",
  "amount": {
    "value": "99.99",
    "currency": "USD"
  },
  "customer": {
    "customerId": "cust_abc123",
    "evidence": {
      "description": "Unauthorized transaction",
      "documents": [
        {
          "type": "police_report",
          "url": "https://example.com/docs/police_report.pdf"
        }
      ]
    }
  },
  "merchant": {
    "merchantId": "merchant_xyz789",
    "evidence": {
      "description": "Transaction was authorized with biometric",
      "documents": [
        {
          "type": "authentication_log",
          "url": "https://example.com/logs/auth_log.json"
        },
        {
          "type": "delivery_confirmation",
          "url": "https://example.com/docs/delivery.pdf"
        }
      ]
    }
  },
  "timeline": {
    "filed": "2025-12-30T00:00:00.000Z",
    "responseDeadline": "2026-01-15T23:59:59.999Z",
    "resolution": null
  }
}
```

## Field Specifications

### Required vs Optional Fields

#### Transaction Object
- **Required:** version, standard, transactionId, type, method, status, amount, timestamps
- **Optional:** merchant, customer, metadata, fees, device, authentication

#### Wallet Object
- **Required:** version, standard, walletId, userId, type, status, balance
- **Optional:** paymentMethods, settings, limits

#### Token Object
- **Required:** version, standard, tokenId, type, status, deviceBinding, cardDetails
- **Optional:** restrictions, metadata

### Data Type Specifications

#### String Fields
- **Maximum length:** 255 characters (unless specified)
- **Encoding:** UTF-8
- **Special characters:** Allowed but must be properly escaped in JSON

#### Numeric Fields
- **Amount values:** Stored as strings to preserve precision
- **Integer fields:** 32-bit signed integers unless specified
- **Floating point:** Avoided for monetary values

#### Boolean Fields
- **Values:** true or false (lowercase)
- **No null:** Must be explicitly true or false

#### Array Fields
- **Maximum elements:** 1000 (unless specified)
- **Empty arrays:** Allowed

#### Object Fields
- **Nesting depth:** Maximum 5 levels
- **Custom fields:** Allowed in metadata objects

### Timestamp Standards

All timestamps must follow ISO 8601 format in UTC:
- Format: YYYY-MM-DDTHH:mm:ss.SSSZ
- Example: 2025-12-25T14:30:00.000Z
- Precision: Milliseconds
- Timezone: Always UTC (Z suffix)

## Schema Versioning

### Version Format
- Format: MAJOR.MINOR.PATCH
- Example: 1.0.0

### Version Compatibility
- **MAJOR:** Breaking changes, requires code updates
- **MINOR:** New features, backward compatible
- **PATCH:** Bug fixes, fully compatible

### Migration Strategy
1. New version released alongside existing version
2. 6-month deprecation period for old version
3. Clear migration guide provided
4. Automated conversion tools available

## Security Considerations

### Sensitive Data Handling
- **PAN (Primary Account Number):** Never stored or transmitted in plain text
- **CVV:** Never stored, even encrypted
- **Biometric data:** Only hashed templates, never raw data
- **Passwords:** Never included in any data structure

### Encryption Requirements
- **In Transit:** TLS 1.3 minimum
- **At Rest:** AES-256 encryption
- **Key Management:** Hardware Security Module (HSM) recommended

### Data Minimization
- Only collect necessary data
- Redact sensitive fields in logs
- Automatic data expiration policies

## Performance Optimization

### Data Size Guidelines
- **Transaction object:** ~2-5 KB
- **Wallet object:** ~5-10 KB
- **Token object:** ~1-3 KB
- **QR code object:** ~500 bytes - 2 KB

### Caching Strategies
- **Static data:** Cache for 24 hours
- **Dynamic data:** No caching
- **Tokens:** Cache until expiry

### Compression
- **Recommended:** gzip compression for API responses
- **Expected ratio:** 60-70% size reduction

## Testing Data

### Test Card Numbers
- **Visa:** 4111111111111111
- **Mastercard:** 5555555555554444
- **Amex:** 378282246310005
- **Discover:** 6011111111111117

### Test Scenarios
1. **Successful payment:** Amount < $100
2. **Declined payment:** Amount = $100.01
3. **Insufficient funds:** Amount = $100.02
4. **Card error:** Amount = $100.03
5. **Network timeout:** Amount = $100.04

---

END OF PHASE 1 DATA FORMAT SPECIFICATION

Total Pages: 25+
Last Updated: December 25, 2025
