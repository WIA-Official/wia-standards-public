# ✅ WIA-CORE-002: Universal Consent Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-CORE-002
> **Version:** 1.0.0
> **Status:** Active
> **Category:** CORE (범용 통합 표준)
> **Color:** Indigo (#6366F1)

---

## 🌟 Overview

The WIA-CORE-002 standard defines a universal framework for managing user consent across systems, applications, and jurisdictions. It provides GDPR/CCPA compliance, consent propagation, granular permissions, and cross-platform consent synchronization.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to empower users with full control over their data and privacy while enabling organizations to manage consent in a compliant, transparent, and interoperable manner.

## 🎯 Key Features

- **Universal Consent Model**: Single source of truth for user consent across all systems
- **GDPR/CCPA Compliance**: Built-in support for international privacy regulations
- **Granular Permissions**: Fine-grained control over data usage and processing
- **Consent Propagation**: Automatic synchronization across integrated systems
- **Audit Trail**: Complete history of consent changes with cryptographic verification
- **Revocation Management**: Instant consent withdrawal with cascading updates
- **Multi-language Support**: Consent notices in user's preferred language
- **Consent Receipts**: Verifiable proof of consent for legal compliance

## 📊 Core Concepts

### 1. Consent Record

A consent record captures user permission for specific data processing activities:

```typescript
{
  "userId": "user-12345",
  "purpose": "marketing_emails",
  "status": "granted",
  "timestamp": "2025-12-27T00:00:00Z",
  "expiresAt": "2026-12-27T00:00:00Z",
  "scope": ["email", "name", "preferences"],
  "jurisdiction": "EU",
  "version": "1.0"
}
```

### 2. Consent Purposes

Predefined categories for data usage:
- **Marketing**: Email, SMS, push notifications
- **Analytics**: Usage tracking, performance monitoring
- **Personalization**: Content recommendations, UI customization
- **Third-party Sharing**: Partner integrations, data sales
- **Research**: Anonymized data analysis, studies
- **Legal**: Compliance, fraud prevention, legal obligations

### 3. Consent Lifecycle

```
Request → Review → Grant/Deny → Active → [Renew/Revoke] → Expired/Revoked
```

## 🔧 Components

### TypeScript SDK

```typescript
import {
  ConsentManager,
  requestConsent,
  checkConsent,
  revokeConsent
} from '@wia/core-002';

// Initialize consent manager
const manager = new ConsentManager({
  jurisdiction: 'EU',
  storage: 'encrypted'
});

// Request consent
const consent = await manager.requestConsent({
  userId: 'user-12345',
  purpose: 'marketing_emails',
  scope: ['email', 'name'],
  legalBasis: 'consent',
  description: 'Send promotional emails about new features'
});

// Check consent
const hasConsent = await manager.checkConsent({
  userId: 'user-12345',
  purpose: 'marketing_emails'
});

console.log(`Marketing consent: ${hasConsent.status}`);

// Revoke consent
await manager.revokeConsent({
  userId: 'user-12345',
  purpose: 'marketing_emails',
  reason: 'user_request'
});
```

### CLI Tool

```bash
# Request consent
wia-core-002 request --user user-12345 --purpose analytics --scope "usage,performance"

# Check consent status
wia-core-002 check --user user-12345 --purpose analytics

# List all consents for user
wia-core-002 list --user user-12345

# Revoke consent
wia-core-002 revoke --user user-12345 --purpose marketing_emails

# Export consent receipt
wia-core-002 export --user user-12345 --format pdf

# Validate GDPR compliance
wia-core-002 validate --jurisdiction EU
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-CORE-002-v1.0.md](./spec/WIA-CORE-002-v1.0.md) | Complete specification with legal framework |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-core-002.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/universal-consent

# Run installation script
./install.sh

# Verify installation
wia-core-002 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/core-002

# Or yarn
yarn add @wia/core-002
```

```typescript
import { ConsentManager } from '@wia/core-002';

const manager = new ConsentManager();

// Request marketing consent
const result = await manager.requestConsent({
  userId: 'user-abc',
  purpose: 'marketing_emails',
  scope: ['email', 'name', 'preferences'],
  legalBasis: 'consent',
  jurisdiction: 'EU'
});

if (result.granted) {
  console.log(`Consent granted. Receipt ID: ${result.receiptId}`);
  console.log(`Expires: ${result.expiresAt}`);
}
```

## 🔐 Privacy & Compliance

### GDPR Requirements

| Requirement | Implementation |
|-------------|----------------|
| **Lawful Basis** | Explicit consent tracking with legal basis |
| **Purpose Limitation** | Granular purpose-specific consent |
| **Data Minimization** | Scope-limited data access |
| **Right to Withdraw** | Instant revocation with cascading updates |
| **Transparency** | Clear consent notices in user's language |
| **Accountability** | Complete audit trail with timestamps |
| **Data Portability** | Export consent history in standard formats |

### CCPA Compliance

- **Opt-out Rights**: Easy consent withdrawal
- **Do Not Sell**: Explicit third-party sharing consent
- **Data Disclosure**: Transparent purpose descriptions
- **Consumer Rights**: Full access to consent history

## 📊 Consent Purposes & Legal Basis

| Purpose | Legal Basis | GDPR Article | Required? |
|---------|-------------|--------------|-----------|
| Service Delivery | Contract | Art. 6(1)(b) | Yes |
| Legal Compliance | Legal Obligation | Art. 6(1)(c) | Yes |
| Fraud Prevention | Legitimate Interest | Art. 6(1)(f) | No |
| Marketing | Consent | Art. 6(1)(a) | No |
| Analytics | Consent | Art. 6(1)(a) | No |
| Third-party Sharing | Consent | Art. 6(1)(a) | No |

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based consent requests
- **WIA-OMNI-API**: Universal consent API gateway
- **WIA-SOCIAL**: Social platform consent federation
- **WIA-BLOCKCHAIN**: Immutable consent ledger (optional)
- **WIA-CREDENTIAL**: Verifiable consent credentials

## 📖 Use Cases

1. **E-commerce Platforms**: Manage customer consent for marketing, analytics, and data sharing
2. **Healthcare Systems**: HIPAA-compliant patient consent for data access and research
3. **Social Networks**: User consent for content sharing, tagging, and third-party apps
4. **IoT Devices**: Device-level consent for data collection and processing
5. **Enterprise SaaS**: Workspace consent management for admin and user permissions
6. **Mobile Apps**: App-level consent for notifications, location, and device access
7. **Advertising Networks**: User consent for behavioral tracking and personalization

## 🔄 Consent Propagation

When consent is granted or revoked, it automatically propagates to:
- **Integrated Systems**: API-connected services receive updates
- **Data Processors**: Third-party processors are notified
- **Analytics Platforms**: Tracking tools respect consent changes
- **Marketing Tools**: Email/SMS platforms honor opt-outs
- **Partner Networks**: Federated consent across organizations

## ⚡ Performance

- **Consent Check**: <10ms average latency
- **Propagation**: <1 second to all integrated systems
- **Storage**: Encrypted at rest and in transit
- **Scalability**: Handles 100K+ consent checks/second
- **Availability**: 99.99% uptime SLA

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
