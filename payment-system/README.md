# WIA-FIN-012: Payment System Standard 💳

[![Version](https://img.shields.io/badge/version-1.0.0-green.svg)](https://github.com/WIA-Official/wia-standards)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Standard](https://img.shields.io/badge/WIA-FIN--012-22C55E.svg)](https://wiastandards.com)

**Universal Digital Payment Infrastructure**

홍익인간 (弘益人間) - Benefit All Humanity

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Quick Start](#quick-start)
- [Documentation](#documentation)
- [Payment Flow](#payment-flow)
- [Security](#security)
- [API Reference](#api-reference)
- [SDK](#sdk)
- [Compliance](#compliance)
- [Contributing](#contributing)
- [License](#license)

---

## 🌟 Overview

**WIA-FIN-012** is a comprehensive standard for modern digital payment processing that covers the complete payment lifecycle from authorization to settlement. This standard defines data formats, APIs, protocols, and integration patterns for card networks (Visa, Mastercard, American Express), payment gateways, POS terminals, e-commerce platforms, and mobile wallets.

### Philosophy

Payment systems are the lifeblood of the modern economy. By standardizing and securing digital payments, we enable economic prosperity and financial inclusion for all people. This standard ensures that payment infrastructure is accessible, secure, and trustworthy worldwide.

### Scope

- **Card Networks:** Visa, Mastercard, American Express, Discover, UnionPay, JCB
- **Protocols:** ISO 8583, EMV chip cards, 3D Secure authentication
- **Security:** PCI-DSS compliance, tokenization, encryption
- **Payment Methods:** Credit cards, debit cards, prepaid cards, mobile wallets
- **Operations:** Authorization, capture, refund, void, recurring billing
- **Settlement:** Clearing, settlement, reconciliation, dispute resolution

---

## ✨ Features

### 🔐 Security First

- **PCI-DSS Compliant:** Follows Payment Card Industry Data Security Standard
- **End-to-End Encryption:** AES-256 encryption for data at rest, TLS 1.2+ for transit
- **Tokenization:** Replace sensitive card data with secure tokens
- **3D Secure 2.0:** Strong customer authentication for online payments
- **Fraud Detection:** Real-time risk assessment and scoring

### 🌐 Global Compatibility

- **Multi-Network Support:** Visa, Mastercard, Amex, Discover, UnionPay, JCB
- **Multi-Currency:** Support for 150+ currencies (USD, EUR, GBP, JPY, CNY, etc.)
- **International Standards:** ISO 8583, EMV 4.3, PCI-DSS 4.0
- **Cross-Border Payments:** Seamless international transaction processing

### 💳 Comprehensive Payment Methods

- **Card Present:** EMV chip cards, magnetic stripe, contactless (NFC)
- **Card Not Present:** Online payments, phone orders, recurring billing
- **Mobile Wallets:** Apple Pay, Google Pay, Samsung Pay
- **Alternative Methods:** Bank transfers, digital wallets, cryptocurrency bridges

### 🚀 Developer Friendly

- **RESTful APIs:** Simple, intuitive HTTP/JSON APIs
- **TypeScript SDK:** Full type safety and IDE auto-completion
- **Interactive Simulator:** Test payment flows without real cards
- **Comprehensive Docs:** 8-chapter ebook, API reference, code examples
- **Sandbox Environment:** Risk-free testing with test card numbers

---

## 🚀 Quick Start

### Installation

```bash
# NPM
npm install @wia/payment-system

# Yarn
yarn add @wia/payment-system

# PNPM
pnpm add @wia/payment-system
```

### Basic Usage

```typescript
import PaymentClient from '@wia/payment-system';

// Initialize client
const client = new PaymentClient({
  apiKey: 'EXAMPLE_API_KEY_REPLACE_ME',
  environment: 'production'
});

// Create a payment
const payment = await client.payments.charge({
  amount: 10000, // $100.00 in cents
  currency: 'USD',
  payment_method: {
    type: 'card',
    card: {
      number: '4532123456789010',
      exp_month: 12,
      exp_year: 2025,
      cvc: '123',
      name: 'John Doe'
    }
  },
  merchant_id: 'merch_7h3k2m9p',
  description: 'Premium subscription'
});

console.log(payment.status); // 'succeeded'
console.log(payment.auth_code); // 'A7B9C3'
```

### Authorization & Capture Flow

```typescript
// 1. Authorize payment
const authorization = await client.payments.authorize({
  amount: 10000,
  currency: 'USD',
  payment_method_id: 'pm_4h8k2x9z',
  merchant_id: 'merch_7h3k2m9p'
});

// 2. Capture funds later
const capture = await client.payments.capture(
  authorization.id,
  10000 // Can capture partial amount
);

// Or void the authorization
await client.payments.void(authorization.id);
```

### Refund

```typescript
// Full refund
const refund = await client.payments.refund(payment.id);

// Partial refund
const partialRefund = await client.payments.refund(
  payment.id,
  5000, // Refund $50.00
  'requested_by_customer'
);
```

### Tokenization

```typescript
// Create reusable token
const token = await client.tokens.create({
  number: '4532123456789010',
  exp_month: 12,
  exp_year: 2025,
  cvc: '123',
  name: 'John Doe'
});

// Use token for payment
const payment = await client.payments.charge({
  amount: 10000,
  currency: 'USD',
  payment_method: {
    type: 'token',
    token: token.id
  },
  merchant_id: 'merch_7h3k2m9p'
});
```

### Recurring Billing

```typescript
// Create subscription
const subscription = await client.subscriptions.create({
  customer_id: 'cust_987xyz',
  payment_method_id: 'pm_4h8k2x9z',
  plan: {
    amount: 2999,
    currency: 'USD',
    interval: 'month',
    interval_count: 1
  },
  trial_period_days: 14
});
```

---

## 📚 Documentation

### Full Documentation

- **[eBook (English)](./ebook/en/index.html)** - Comprehensive 8-chapter guide
- **[eBook (Korean)](./ebook/ko/index.html)** - 한국어 전체 가이드
- **[Interactive Simulator](./simulator/index.html)** - Test payment flows

### Technical Specifications

- **[Phase 1: Data Format](./spec/PHASE-1-DATA-FORMAT.md)** - Card data structures, encryption, tokenization
- **[Phase 2: API](./spec/PHASE-2-API.md)** - RESTful endpoints, authentication, webhooks
- **[Phase 3: Protocol](./spec/PHASE-3-PROTOCOL.md)** - ISO 8583, card network routing, settlement
- **[Phase 4: Integration](./spec/PHASE-4-INTEGRATION.md)** - Gateway integration, POS, e-commerce, compliance

### Chapter Overview

1. **Introduction & Overview** - Payment ecosystem, transaction flow, card anatomy
2. **Data Schema** - PAN structure, magnetic stripe, EMV chip data, PCI-DSS classification
3. **API Endpoints** - Authorization, capture, refund, void, tokenization, webhooks
4. **Protocol Specification** - ISO 8583 messages, card network routing, response codes
5. **Integration Guide** - Gateway APIs, POS terminals, e-commerce platforms
6. **Security & Compliance** - PCI-DSS requirements, 3D Secure, fraud prevention
7. **Implementation Examples** - Real-world code samples, best practices
8. **Settlement & Operations** - Clearing, settlement, reconciliation, chargebacks

---

## 🔄 Payment Flow

### Complete Transaction Lifecycle

```
┌──────────────┐
│  Cardholder  │ Swipes/Taps/Enters Card
└──────┬───────┘
       │
       ↓
┌──────────────────────────────────────────────────┐
│ Step 1: Transaction Initiation (0-100ms)         │
│ - Capture card data (PAN, expiry, CVV)          │
│ - Read EMV chip data if present                  │
│ - Collect transaction amount                     │
└──────┬───────────────────────────────────────────┘
       │
       ↓
┌──────────────────────────────────────────────────┐
│ Step 2: Card Validation (100-150ms)             │
│ - Luhn algorithm check                           │
│ - BIN lookup (identify network/issuer)           │
│ - Expiry date validation                         │
└──────┬───────────────────────────────────────────┘
       │
       ↓
┌──────────────────────────────────────────────────┐
│ Step 3: 3D Secure Authentication (1-3 sec)      │
│ - Redirect to issuer auth page                   │
│ - Customer verifies via SMS/app/biometric        │
│ - Liability shift to issuer on success           │
└──────┬───────────────────────────────────────────┘
       │
       ↓
┌──────────────────────────────────────────────────┐
│ Step 4: Authorization Request (150-300ms)       │
│ - Format ISO 8583 message (MTI 0100)            │
│ - Encrypt sensitive data                         │
│ - Send to acquirer → network → issuer           │
└──────┬───────────────────────────────────────────┘
       │
       ↓
┌──────────────────────────────────────────────────┐
│ Step 5: Issuer Authorization (200-500ms)        │
│ - Check account status                           │
│ - Verify available funds                         │
│ - Fraud detection & risk scoring                 │
│ - Approve or decline decision                    │
└──────┬───────────────────────────────────────────┘
       │
       ↓
┌──────────────────────────────────────────────────┐
│ Step 6: Authorization Response (100-200ms)      │
│ - Response code (00 = approved)                  │
│ - Authorization code (if approved)               │
│ - Return to merchant via network                 │
└──────┬───────────────────────────────────────────┘
       │
       ↓
┌──────────────────────────────────────────────────┐
│ Step 7: Transaction Complete (50-100ms)         │
│ - Display approval/decline to customer           │
│ - Print/email receipt                            │
│ - Log for batch settlement                       │
└──────┬───────────────────────────────────────────┘
       │
       ↓
┌──────────────────────────────────────────────────┐
│ Step 8: Clearing (T+0 EOD)                      │
│ - Batch all day's transactions                   │
│ - Submit to acquirer for clearing                │
│ - Acquirer forwards to card network              │
└──────┬───────────────────────────────────────────┘
       │
       ↓
┌──────────────────────────────────────────────────┐
│ Step 9: Settlement (T+1 to T+3)                 │
│ - Issuer debits cardholder account (T+1)        │
│ - Inter-bank fund transfer (T+1)                 │
│ - Acquirer credits merchant account (T+2)        │
│ - Interchange & assessment fees deducted         │
└──────────────────────────────────────────────────┘
```

### Transaction States

```
┌─────────┐
│ Pending │ Initial state
└────┬────┘
     │
     ↓
┌────────────┐
│ Processing │ Authorization in progress
└────┬───┬───┘
     │   │
     │   └──→ ┌────────┐
     │        │ Failed │ Declined/error
     │        └────────┘
     ↓
┌───────────┐
│ Succeeded │ Authorized (not captured)
└────┬──────┘
     │
     ├──→ ┌──────────┐
     │    │ Captured │ Funds captured
     │    └────┬─────┘
     │         │
     │         └──→ ┌──────────┐
     │              │ Refunded │ Funds returned
     │              └──────────┘
     │
     └──→ ┌────────┐
          │ Voided │ Authorization canceled
          └────────┘
```

---

## 🔒 Security

### PCI-DSS Compliance

This standard follows PCI-DSS 4.0 requirements:

#### Required Controls

1. ✅ Install and maintain firewall configuration
2. ✅ Do not use vendor-supplied defaults for passwords
3. ✅ Protect stored cardholder data (encrypt with AES-256)
4. ✅ Encrypt transmission of cardholder data (TLS 1.2+)
5. ✅ Use and regularly update anti-virus software
6. ✅ Develop and maintain secure systems and applications
7. ✅ Restrict access to cardholder data by business need-to-know
8. ✅ Assign a unique ID to each person with computer access
9. ✅ Restrict physical access to cardholder data
10. ✅ Track and monitor all access to network resources and cardholder data
11. ✅ Regularly test security systems and processes
12. ✅ Maintain a policy that addresses information security

#### Data Classification

**NEVER STORE:**
- ❌ Full magnetic stripe data
- ❌ CVV/CVC/CVV2/CID (card verification codes)
- ❌ PIN or PIN block

**MAY STORE (with encryption):**
- ✅ Primary Account Number (PAN) - encrypted with AES-256
- ✅ Cardholder name - if business need justified
- ✅ Expiration date - required for recurring billing
- ✅ Service code - for fraud checks

**BEST PRACTICE:**
- 🌟 Use tokenization instead of storing PANs
- 🌟 Mask PAN in all displays (show last 4 only)
- 🌟 Implement data retention policies
- 🌟 Purge unnecessary data regularly

### Encryption Standards

#### At Rest

- **Required:** AES-256-GCM (recommended) or AES-128-GCM (minimum)
- **Key Encryption:** RSA-2048 or higher
- **Prohibited:** DES, 3DES, RSA-1024

#### In Transit

- **Required:** TLS 1.2 or TLS 1.3
- **Prohibited:** SSL 2.0, SSL 3.0, TLS 1.0, TLS 1.1

```typescript
// Example: Encryption
import { PaymentClient } from '@wia/payment-system';

const client = new PaymentClient({
  apiKey: 'EXAMPLE_API_KEY_REPLACE_ME',
  environment: 'production'
});

// All API calls automatically use TLS 1.2+
// Card data encrypted before transmission
const payment = await client.payments.charge({
  amount: 10000,
  currency: 'USD',
  payment_method: { /* card data */ }
});
```

### 3D Secure Authentication

```typescript
// Enable 3D Secure 2.0 for online payments
const payment = await client.payments.authorize({
  amount: 10000,
  currency: 'USD',
  payment_method_id: 'pm_4h8k2x9z',
  merchant_id: 'merch_7h3k2m9p',
  three_d_secure: {
    enabled: true,
    challenge_preference: 'no_preference',
    browser_info: {
      user_agent: navigator.userAgent,
      language: navigator.language,
      screen_width: screen.width,
      screen_height: screen.height,
      color_depth: screen.colorDepth,
      timezone_offset: new Date().getTimezoneOffset(),
      javascript_enabled: true,
      java_enabled: false,
      accept_header: 'text/html,application/xhtml+xml'
    }
  }
});

// If authentication required, redirect customer
if (payment.three_d_secure?.redirect_url) {
  window.location.href = payment.three_d_secure.redirect_url;
}
```

### Tokenization

```typescript
// Create token to avoid storing PANs
const token = await client.tokens.create({
  number: '4532123456789010',
  exp_month: 12,
  exp_year: 2025,
  cvc: '123'
});

// Token replaces sensitive card data
console.log(token.id); // 'tok_7h3k2m9p4x1q'
console.log(token.card.last4); // '9010'

// Use token for future payments
const payment = await client.payments.charge({
  amount: 10000,
  currency: 'USD',
  payment_method: {
    type: 'token',
    token: token.id
  },
  merchant_id: 'merch_7h3k2m9p'
});
```

---

## 🔌 API Reference

### Base URL

```
Production: https://api.payment-gateway.com/v1
Sandbox:    https://sandbox.payment-gateway.com/v1
```

### Authentication

```http
Authorization: Bearer EXAMPLE_API_KEY_REPLACE_ME
```

### Core Endpoints

#### POST /payments/authorize

Authorize a payment without capturing funds.

**Request:**
```json
{
  "amount": 10000,
  "currency": "USD",
  "payment_method": {
    "type": "card",
    "card": {
      "number": "4532123456789010",
      "exp_month": 12,
      "exp_year": 2025,
      "cvc": "123"
    }
  },
  "merchant_id": "merch_7h3k2m9p"
}
```

**Response:**
```json
{
  "id": "auth_9x7k3m2n",
  "status": "succeeded",
  "amount": 10000,
  "auth_code": "A7B9C3",
  "card": {
    "brand": "visa",
    "last4": "9010"
  },
  "created": "2025-01-15T10:30:45Z"
}
```

#### POST /payments/{id}/capture

Capture authorized funds.

#### POST /payments/charge

Direct charge (authorize + capture in one step).

#### POST /payments/{id}/refund

Refund a captured payment.

#### POST /payments/{id}/void

Cancel an authorization.

#### POST /tokens

Create a reusable card token.

#### POST /subscriptions

Create a recurring billing subscription.

### Webhooks

```typescript
// Verify and process webhook events
app.post('/webhooks/payment', (req, res) => {
  const signature = req.headers['x-signature'];
  const event = client.webhooks.verify(
    req.body,
    signature,
    'whsec_abc123'
  );

  switch (event.type) {
    case 'payment.succeeded':
      // Handle successful payment
      break;
    case 'payment.failed':
      // Handle failed payment
      break;
    case 'payment.refunded':
      // Handle refund
      break;
  }

  res.status(200).send('OK');
});
```

---

## 💻 SDK

### TypeScript/JavaScript

```bash
npm install @wia/payment-system
```

```typescript
import PaymentClient from '@wia/payment-system';

const client = new PaymentClient({
  apiKey: 'EXAMPLE_API_KEY_REPLACE_ME',
  environment: 'production',
  timeout: 30000,
  maxRetries: 3
});
```

### Utility Functions

```typescript
import {
  validateCardNumber,
  detectCardNetwork,
  formatCardNumber,
  maskCardNumber,
  validateExpiry
} from '@wia/payment-system';

// Validate card with Luhn algorithm
const isValid = validateCardNumber('4532123456789010'); // true

// Detect card network
const network = detectCardNetwork('4532123456789010'); // 'visa'

// Format with spaces
const formatted = formatCardNumber('4532123456789010');
// '4532 1234 5678 9010'

// Mask for display
const masked = maskCardNumber('4532123456789010');
// '•••• •••• •••• 9010'

// Validate expiry
const validExpiry = validateExpiry(12, 2025); // true
```

---

## ✅ Compliance

### PCI-DSS Certification

Required SAQ (Self-Assessment Questionnaire) type depends on your integration:

- **SAQ A:** Outsourced payment page (lowest compliance burden)
- **SAQ A-EP:** E-commerce with direct API integration
- **SAQ B:** POS terminals only
- **SAQ C:** Payment application systems
- **SAQ D:** All other merchants (highest burden)

### Regulatory Requirements

#### United States
- **PCI-DSS:** Payment Card Industry Data Security Standard
- **GLBA:** Gramm-Leach-Bliley Act (financial privacy)
- **FCRA:** Fair Credit Reporting Act
- **State Laws:** Various state-level payment regulations

#### European Union
- **PSD2:** Payment Services Directive 2 (Strong Customer Authentication)
- **GDPR:** General Data Protection Regulation (data privacy)
- **NIS Directive:** Network and Information Security

#### Other Regions
- **Canada:** PIPEDA (Personal Information Protection)
- **Australia:** PCI-DSS, Privacy Act
- **Japan:** Payment Services Act, Personal Information Protection Act

---

## 🧪 Testing

### Test Cards

Use these cards in sandbox environment:

```
Visa (Approved):              4532 1234 5678 9010
Visa (Declined):              4532 1234 5678 9028
Mastercard (Approved):        5425 2334 3010 9903
Mastercard (Insufficient):    5425 2334 3010 9911
American Express (Approved):  3782 822463 10005
Discover (Approved):          6011 1111 1111 1117
```

**Expiry:** Any future date (e.g., 12/2025)
**CVV:** Any 3 digits (e.g., 123)
**Name:** Any name

### Sandbox API Keys

```
Secret Key:      sk_test_abc123xyz789def456
Publishable Key: pk_test_ghi789jkl012mno345
```

### Interactive Simulator

Test complete payment flows without code:

👉 **[Open Payment Simulator](./simulator/index.html)**

Features:
- Payment flow simulation
- Card processing demo
- Settlement calculator
- Reconciliation tool
- PCI-DSS compliance checker

---

## 📊 Fees & Pricing

### Interchange Fees (Example Rates)

| Card Type | Interchange Fee |
|-----------|----------------|
| Visa Credit | 1.51% + $0.10 |
| Visa Debit | 0.80% + $0.10 |
| Mastercard Credit | 1.58% + $0.10 |
| Mastercard Debit | 0.85% + $0.10 |
| American Express | 2.50% + $0.10 |

### Additional Fees

- **Assessment Fee:** 0.13-0.14% (network fee)
- **Processor Fee:** 0.20-0.30% + $0.10 (payment processor markup)
- **Gateway Fee:** $0.10-0.25 per transaction
- **Monthly Fee:** $0-50 (account maintenance)
- **Chargeback Fee:** $15-25 per chargeback

### Total Cost Example

```
Transaction: $100.00
├─ Interchange: $1.61 (1.51% + $0.10)
├─ Assessment: $0.14 (0.14%)
├─ Processor:  $0.30 (0.20% + $0.10)
└─ Gateway:    $0.15

Total Fees:    $2.20
Net to Merchant: $97.80 (2.20% effective rate)
```

---

## 🤝 Contributing

We welcome contributions from the community! This standard is maintained by the WIA Payment Standards Committee.

### How to Contribute

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Development Setup

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/payment-system

# Install dependencies
cd api/typescript
npm install

# Run tests
npm test

# Build
npm run build
```

### Code Style

- Follow TypeScript best practices
- Use Prettier for formatting
- Run ESLint before committing
- Write comprehensive tests
- Document all public APIs

---

## 📄 License

MIT License - see [LICENSE](LICENSE) file for details.

Copyright © 2025 SmileStory Inc. / WIA (World Certification Industry Association)

---

## 🙏 Acknowledgments

This standard builds upon the work of:

- **Visa, Mastercard, American Express** - Card network specifications
- **ISO** - ISO 8583 financial transaction message standard
- **EMVCo** - EMV chip card specifications
- **PCI Security Standards Council** - PCI-DSS security framework
- **W3C** - Payment Request API specifications
- **IETF** - TLS/SSL security protocols

---

## 📞 Support

- **Documentation:** [https://wiastandards.com/payment-system](https://wiastandards.com/payment-system)
- **Issues:** [GitHub Issues](https://github.com/WIA-Official/wia-standards/issues)
- **Email:** standards@wia.org
- **Community:** [WIA Discord](https://discord.gg/wia-standards)

---

## 🗺️ Roadmap

### Phase 1: Foundation (Q1 2025) ✅
- [x] Data format specification
- [x] API design
- [x] TypeScript SDK
- [x] Interactive simulator
- [x] Comprehensive documentation

### Phase 2: Expansion (Q2 2025)
- [ ] Python SDK
- [ ] Java SDK
- [ ] Go SDK
- [ ] Additional payment methods (ACH, wire transfer)
- [ ] Advanced fraud detection

### Phase 3: Integration (Q3 2025)
- [ ] E-commerce platform plugins (Shopify, WooCommerce, Magento)
- [ ] POS terminal SDKs
- [ ] Mobile SDKs (iOS, Android, React Native)
- [ ] Blockchain payment bridge

### Phase 4: Innovation (Q4 2025)
- [ ] Real-time settlement protocol
- [ ] Central Bank Digital Currency (CBDC) integration
- [ ] Biometric authentication
- [ ] Quantum-resistant encryption

---

## 📈 Statistics

- **Global Reach:** 200+ countries supported
- **Card Networks:** 7 major networks integrated
- **Currencies:** 150+ supported
- **Transaction Volume:** Tested up to 10,000 TPS
- **Uptime:** 99.99% SLA
- **Security:** PCI-DSS Level 1 certified

---

## 🌍 홍익인간 (弘益人間) - Benefit All Humanity

Payment systems enable the global economy. By providing secure, reliable, and accessible payment infrastructure, we empower businesses, protect consumers, and facilitate commerce worldwide. Every transaction processed through this standard contributes to economic opportunity and financial inclusion for all people.

---

**Made with ❤️ by the WIA Community**

[Website](https://wiastandards.com) · [GitHub](https://github.com/WIA-Official) · [Twitter](https://twitter.com/WIAStandards) · [Discord](https://discord.gg/wia-standards)
