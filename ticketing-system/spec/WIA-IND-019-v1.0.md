# WIA-IND-019: Ticketing System Standard - Complete Specification

> **Standard ID:** WIA-IND-019
> **Version:** 1.0.0
> **Status:** Active
> **Category:** IND (Industry)
> **Last Updated:** 2025-12-27
> **Authors:** WIA Industry Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Ticket Data Format](#5-ticket-data-format)
6. [QR Code and Barcode Standards](#6-qr-code-and-barcode-standards)
7. [Seat Reservation System](#7-seat-reservation-system)
8. [Dynamic Pricing Algorithms](#8-dynamic-pricing-algorithms)
9. [Fraud Prevention](#9-fraud-prevention)
10. [Ticket Transfer and Resale](#10-ticket-transfer-and-resale)
11. [Multi-Venue Support](#11-multi-venue-support)
12. [Season Pass Management](#12-season-pass-management)
13. [Access Control Integration](#13-access-control-integration)
14. [API Specification](#14-api-specification)
15. [Security Requirements](#15-security-requirements)
16. [Privacy and Data Protection](#16-privacy-and-data-protection)
17. [Interoperability](#17-interoperability)
18. [Testing and Certification](#18-testing-and-certification)
19. [Appendices](#19-appendices)

---

## 1. Introduction

### 1.1 Purpose

The WIA-IND-019 standard establishes a comprehensive framework for digital and physical ticketing systems. This standard addresses the growing need for interoperability, security, and innovation in the global ticketing industry, serving events ranging from concerts and sports to transportation and attractions.

### 1.2 Background

Traditional ticketing systems suffer from fragmentation, lack of standardization, and security vulnerabilities. This standard provides:

- **Universal Format**: Cross-platform ticket compatibility
- **Enhanced Security**: Multi-layer fraud prevention
- **Dynamic Pricing**: Market-responsive pricing algorithms
- **Consumer Protection**: Fair resale and transfer mechanisms
- **Accessibility**: Support for diverse user needs
- **Sustainability**: Reduced paper waste through digital tickets

### 1.3 Philosophy: 弘益人間 (Benefit All Humanity)

This standard embodies the principle of benefiting all stakeholders:
- **Event Organizers**: Increased revenue and operational efficiency
- **Venue Operators**: Streamlined access control and analytics
- **Attendees**: Fair pricing, security, and convenience
- **Society**: Reduced fraud, environmental benefits, and economic fairness

### 1.4 Design Principles

1. **Interoperability**: Work seamlessly across platforms and vendors
2. **Security**: Multi-layer protection against fraud and counterfeiting
3. **Privacy**: Strong data protection and user consent
4. **Accessibility**: Support for users with disabilities
5. **Scalability**: Handle events from 10 to 100,000+ attendees
6. **Flexibility**: Adapt to various event types and business models
7. **Transparency**: Clear pricing, fees, and terms
8. **Sustainability**: Minimize environmental impact

---

## 2. Scope

### 2.1 Applicability

This standard applies to:

- **Event Types**
  - Concerts and music festivals
  - Sports events (professional, collegiate, amateur)
  - Theater and performing arts
  - Conferences and exhibitions
  - Museums and attractions
  - Transportation (train, bus, airline, ferry)
  - Cinemas and entertainment venues
  - Virtual and hybrid events

- **Ticket Types**
  - Single entry tickets
  - Multi-day passes
  - Season passes and memberships
  - VIP and premium tickets
  - Group and package tickets
  - Accessible seating tickets
  - Virtual admission
  - Flexible and exchangeable tickets

- **Stakeholders**
  - Event organizers and promoters
  - Venue operators and management
  - Ticketing platforms and marketplaces
  - Primary ticket issuers
  - Secondary market operators
  - Payment processors
  - Access control system providers
  - Regulatory authorities

### 2.2 Out of Scope

The following are not covered by this standard:
- Gift cards and vouchers (covered by WIA-FIN-XXX)
- Membership cards without event access
- Employee credentials (covered by WIA-SEC-XXX)
- General admission without seating

---

## 3. Normative References

The following standards and specifications are referenced:

- **ISO/IEC 18004:2015** - QR Code barcode symbology specification
- **ISO/IEC 15417:2007** - Code 128 barcode specification
- **ISO/IEC 27001:2013** - Information security management
- **WCAG 2.1** - Web Content Accessibility Guidelines
- **PCI DSS 3.2** - Payment Card Industry Data Security Standard
- **GDPR** - General Data Protection Regulation (EU)
- **RFC 3339** - Date and Time on the Internet
- **RFC 4122** - UUID specification
- **IEEE 802.11** - Wireless LAN standards (for NFC)
- **Blockchain Standards**: ERC-721 (NFT), ERC-1155 (Multi-Token)

---

## 4. Terms and Definitions

### 4.1 Core Terms

**Ticket**: A digital or physical credential granting admission to an event or service.

**Event**: A scheduled occurrence with defined time, location, and capacity constraints.

**Venue**: A physical or virtual location where an event takes place.

**Seat**: A specific, reserved location within a venue section.

**Holder**: The person or entity authorized to use a ticket.

**Issuer**: The organization or platform that creates and distributes tickets.

**Validator**: A system or person that verifies ticket authenticity at entry.

**QR Code**: A two-dimensional barcode encoding ticket validation data.

**Blockchain Hash**: A cryptographic identifier linking a ticket to distributed ledger.

**Dynamic Pricing**: Algorithmic price adjustment based on demand, time, and market conditions.

**Resale**: The transfer of a ticket from original purchaser to a new holder for value.

**Transfer**: The change of ticket ownership, with or without payment.

**Check-in**: The validation and redemption of a ticket at venue entry.

**Season Pass**: A ticket granting multiple entries over a defined period.

**Accessible Ticket**: A ticket with accommodations for persons with disabilities.

### 4.2 Technical Terms

**TOTP**: Time-based One-Time Password algorithm for rotating codes.

**Geofencing**: Location-based validation using GPS or network positioning.

**NFC**: Near Field Communication for contactless validation.

**Biometric Binding**: Association of a ticket with holder's biometric data.

**Smart Contract**: Self-executing contract on blockchain for ticket operations.

**Capacity Management**: Real-time inventory tracking and allocation.

**Scalper**: A person or bot purchasing tickets for resale at inflated prices.

**Anti-Scalping**: Mechanisms to prevent bulk purchasing and price gouging.

---

## 5. Ticket Data Format

### 5.1 Core Schema

All tickets MUST conform to the following JSON schema:

```json
{
  "$schema": "https://json-schema.org/draft-07/schema#",
  "title": "WIA-IND-019 Ticket",
  "type": "object",
  "required": [
    "ticketId",
    "eventId",
    "eventName",
    "venue",
    "validity",
    "security",
    "status"
  ],
  "properties": {
    "ticketId": {
      "type": "string",
      "pattern": "^TKT-[0-9]{4}-[A-Z0-9]+-[A-Z0-9]+-[0-9]{4}$",
      "description": "Unique ticket identifier"
    },
    "eventId": {
      "type": "string",
      "pattern": "^EVT-[0-9]{4}-[0-9]{3,}$",
      "description": "Event identifier"
    },
    "eventName": {
      "type": "string",
      "minLength": 1,
      "maxLength": 200,
      "description": "Human-readable event name"
    },
    "venue": {
      "type": "object",
      "required": ["id", "name", "location"],
      "properties": {
        "id": {
          "type": "string",
          "description": "Venue identifier"
        },
        "name": {
          "type": "string",
          "description": "Venue name"
        },
        "location": {
          "type": "object",
          "required": ["address"],
          "properties": {
            "address": {
              "type": "string",
              "description": "Physical address"
            },
            "coordinates": {
              "type": "object",
              "properties": {
                "lat": {
                  "type": "number",
                  "minimum": -90,
                  "maximum": 90
                },
                "lon": {
                  "type": "number",
                  "minimum": -180,
                  "maximum": 180
                }
              }
            },
            "timezone": {
              "type": "string",
              "description": "IANA timezone identifier"
            }
          }
        }
      }
    },
    "seat": {
      "type": "object",
      "description": "Seat assignment (optional for general admission)",
      "properties": {
        "section": {
          "type": "string",
          "description": "Venue section"
        },
        "row": {
          "type": "string",
          "description": "Seat row"
        },
        "seat": {
          "type": "string",
          "description": "Seat number"
        },
        "type": {
          "type": "string",
          "enum": ["standard", "VIP", "accessible", "premium", "standing"],
          "description": "Seat type"
        },
        "accessible": {
          "type": "boolean",
          "description": "ADA/accessibility compliant"
        },
        "companion": {
          "type": "string",
          "description": "Associated companion seat ID"
        }
      }
    },
    "holder": {
      "type": "object",
      "required": ["name"],
      "properties": {
        "name": {
          "type": "string",
          "description": "Ticket holder name"
        },
        "email": {
          "type": "string",
          "format": "email",
          "description": "Contact email"
        },
        "phone": {
          "type": "string",
          "pattern": "^\\+?[1-9]\\d{1,14}$",
          "description": "Phone number (E.164 format)"
        },
        "verified": {
          "type": "boolean",
          "description": "Identity verification status"
        },
        "biometric": {
          "type": "object",
          "description": "Optional biometric binding",
          "properties": {
            "type": {
              "type": "string",
              "enum": ["fingerprint", "face", "iris"]
            },
            "hash": {
              "type": "string",
              "description": "Hashed biometric template"
            }
          }
        }
      }
    },
    "pricing": {
      "type": "object",
      "required": ["finalPrice", "currency"],
      "properties": {
        "originalPrice": {
          "type": "number",
          "minimum": 0,
          "description": "Base price before adjustments"
        },
        "finalPrice": {
          "type": "number",
          "minimum": 0,
          "description": "Final price paid"
        },
        "currency": {
          "type": "string",
          "pattern": "^[A-Z]{3}$",
          "description": "ISO 4217 currency code"
        },
        "taxes": {
          "type": "number",
          "minimum": 0,
          "description": "Tax amount"
        },
        "fees": {
          "type": "number",
          "minimum": 0,
          "description": "Service and processing fees"
        },
        "discounts": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "type": {
                "type": "string",
                "enum": ["early-bird", "student", "senior", "group", "promo"]
              },
              "amount": {
                "type": "number",
                "minimum": 0
              },
              "code": {
                "type": "string"
              }
            }
          }
        }
      }
    },
    "validity": {
      "type": "object",
      "required": ["eventDate"],
      "properties": {
        "issueDate": {
          "type": "string",
          "format": "date-time",
          "description": "RFC 3339 timestamp"
        },
        "eventDate": {
          "type": "string",
          "format": "date-time",
          "description": "Event start time"
        },
        "expiryDate": {
          "type": "string",
          "format": "date-time",
          "description": "Ticket expiration time"
        },
        "timezone": {
          "type": "string",
          "description": "IANA timezone"
        },
        "entryWindow": {
          "type": "object",
          "description": "Allowed entry time range",
          "properties": {
            "start": {
              "type": "string",
              "format": "date-time"
            },
            "end": {
              "type": "string",
              "format": "date-time"
            }
          }
        }
      }
    },
    "security": {
      "type": "object",
      "required": ["qrCode", "signature"],
      "properties": {
        "qrCode": {
          "type": "string",
          "description": "QR code payload (encrypted)"
        },
        "barcode": {
          "type": "string",
          "description": "1D barcode (Code 128)"
        },
        "nfc": {
          "type": "object",
          "description": "NFC chip data",
          "properties": {
            "uid": {
              "type": "string"
            },
            "ndef": {
              "type": "string"
            }
          }
        },
        "blockchain": {
          "type": "object",
          "description": "Blockchain verification",
          "properties": {
            "hash": {
              "type": "string",
              "description": "Transaction hash"
            },
            "network": {
              "type": "string",
              "enum": ["ethereum", "polygon", "solana", "avalanche"]
            },
            "contractAddress": {
              "type": "string"
            },
            "tokenId": {
              "type": "string"
            },
            "verified": {
              "type": "boolean"
            }
          }
        },
        "signature": {
          "type": "string",
          "description": "Digital signature (ECDSA/RSA)"
        },
        "totp": {
          "type": "object",
          "description": "Time-based OTP for rotating codes",
          "properties": {
            "secret": {
              "type": "string",
              "description": "Shared secret (encrypted)"
            },
            "interval": {
              "type": "number",
              "default": 30,
              "description": "Refresh interval in seconds"
            }
          }
        }
      }
    },
    "status": {
      "type": "string",
      "enum": [
        "active",
        "pending",
        "used",
        "expired",
        "cancelled",
        "suspended",
        "transferred"
      ],
      "description": "Current ticket status"
    },
    "transferable": {
      "type": "boolean",
      "default": true,
      "description": "Can be transferred to another person"
    },
    "resellable": {
      "type": "boolean",
      "default": true,
      "description": "Can be resold on secondary market"
    },
    "checkInStatus": {
      "type": "string",
      "enum": ["not-checked-in", "checked-in", "rejected"],
      "default": "not-checked-in"
    },
    "checkInHistory": {
      "type": "array",
      "description": "Check-in attempt history",
      "items": {
        "type": "object",
        "properties": {
          "timestamp": {
            "type": "string",
            "format": "date-time"
          },
          "location": {
            "type": "string",
            "description": "Gate/entry point"
          },
          "result": {
            "type": "string",
            "enum": ["success", "failure", "duplicate"]
          },
          "validator": {
            "type": "string",
            "description": "Scanner/validator ID"
          }
        }
      }
    },
    "metadata": {
      "type": "object",
      "properties": {
        "issuer": {
          "type": "string",
          "description": "Issuing organization"
        },
        "platform": {
          "type": "string",
          "default": "WIA-IND-019"
        },
        "version": {
          "type": "string",
          "pattern": "^\\d+\\.\\d+\\.\\d+$"
        },
        "customFields": {
          "type": "object",
          "description": "Event-specific metadata"
        }
      }
    }
  }
}
```

### 5.2 Ticket ID Format

Ticket IDs MUST follow this structure:

```
TKT-{YEAR}-{EVENT_CODE}-{SECTION}-{SEQUENCE}

Examples:
- TKT-2025-EVT001-A12-0001
- TKT-2025-CON789-GA-5432
- TKT-2025-SPT456-VIP-0099
```

Components:
- **YEAR**: 4-digit year (2025)
- **EVENT_CODE**: Alphanumeric event identifier (EVT001, CON789)
- **SECTION**: Seat section or ticket type (A12, GA, VIP)
- **SEQUENCE**: 4-digit sequence number (0001-9999)

### 5.3 Event ID Format

Event IDs MUST follow this structure:

```
EVT-{YEAR}-{SEQUENCE}

Examples:
- EVT-2025-001
- EVT-2025-12345
```

### 5.4 Venue ID Format

Venue IDs SHOULD follow this structure:

```
VEN-{LOCATION_CODE}-{SEQUENCE}

Examples:
- VEN-NYC-001 (New York City venue #1)
- VEN-LA-042 (Los Angeles venue #42)
- VEN-LON-007 (London venue #7)
```

---

## 6. QR Code and Barcode Standards

### 6.1 QR Code Specification

#### 6.1.1 ISO Compliance

All QR codes MUST comply with **ISO/IEC 18004:2015**.

#### 6.1.2 QR Code Payload

The QR code MUST encode a JSON Web Token (JWT) containing:

```json
{
  "header": {
    "alg": "ES256",
    "typ": "JWT"
  },
  "payload": {
    "tid": "TKT-2025-EVT001-A12-0001",
    "eid": "EVT-2025-001",
    "exp": 1751328000,
    "iat": 1735129600,
    "sub": "john@example.com",
    "iss": "wia-ticketing",
    "totp": "123456",
    "ver": "1.0.0"
  },
  "signature": "ECDSA_SIGNATURE_HERE"
}
```

Fields:
- **tid**: Ticket ID
- **eid**: Event ID
- **exp**: Expiration timestamp (Unix epoch)
- **iat**: Issued at timestamp
- **sub**: Subject (ticket holder email)
- **iss**: Issuer identifier
- **totp**: Current time-based OTP (6 digits)
- **ver**: Standard version

#### 6.1.3 QR Code Generation

```typescript
import { generateQRCode } from '@wia/ind-019';

const qrCode = await generateQRCode({
  ticketId: 'TKT-2025-EVT001-A12-0001',
  eventId: 'EVT-2025-001',
  holderEmail: 'john@example.com',
  expiryDate: new Date('2025-06-21T02:00:00Z'),
  totpSecret: 'BASE32_SECRET_HERE',
  format: 'png', // or 'svg', 'base64'
  size: 512, // pixels
  errorCorrection: 'H' // High (30% correction)
});
```

#### 6.1.4 Error Correction

QR codes MUST use error correction level **H (High)** allowing 30% data recovery.

#### 6.1.5 Rotation Prevention

To prevent replay attacks, QR codes SHOULD rotate every 30 seconds using TOTP:

```javascript
const totp = generateTOTP(secret, Date.now(), 30); // 30-second interval
```

### 6.2 Barcode Specification

#### 6.2.1 Code 128 Standard

1D barcodes MUST use **Code 128** format (ISO/IEC 15417:2007).

#### 6.2.2 Barcode Payload

The barcode MUST encode:

```
128-{TICKET_ID_HASH}-{CHECKSUM}

Example: 128-7A3F9B2E-C7
```

Components:
- **TICKET_ID_HASH**: 8-character hex hash of ticket ID (truncated SHA256)
- **CHECKSUM**: 2-character verification code (modulo 97)

#### 6.2.3 Barcode Generation

```bash
# CLI
wia-ind-019 generate-barcode --ticket-id "TKT-2025-EVT001-A12-0001" --format code128
```

### 6.3 NFC Integration

#### 6.3.1 NFC Data Exchange Format (NDEF)

NFC-enabled tickets MUST support NDEF records:

```json
{
  "uid": "04:68:2A:B2:C3:4D:80",
  "ndef": {
    "type": "text/plain",
    "payload": "wia://ticket/TKT-2025-EVT001-A12-0001",
    "encoding": "UTF-8"
  },
  "encryption": "AES-256-GCM",
  "authentication": true
}
```

#### 6.3.2 NFC Read Range

NFC validation SHOULD occur within 4cm (1.5 inches) of reader.

---

## 7. Seat Reservation System

### 7.1 Seat Numbering

#### 7.1.1 Standard Format

Seats MUST be identified using:

```
{SECTION}-{ROW}-{SEAT}

Examples:
- A-12-15 (Section A, Row 12, Seat 15)
- ORCH-F-101 (Orchestra, Row F, Seat 101)
- GA-FLOOR-001 (General Admission Floor, Position 1)
```

#### 7.1.2 Accessible Seating

Accessible seats MUST include metadata:

```json
{
  "seat": "ACC-A-5",
  "accessible": true,
  "features": [
    "wheelchair-space",
    "companion-seat",
    "transfer-seat",
    "aisle-access"
  ],
  "width": 90, // cm
  "clearance": 120, // cm
  "sightlines": "unobstructed",
  "companion": "ACC-A-6"
}
```

### 7.2 Capacity Management

#### 7.2.1 Real-Time Inventory

The system MUST track capacity in real-time:

```typescript
interface CapacityStatus {
  total: number;
  sold: number;
  reserved: number;
  available: number;
  blocked: number; // maintenance, VIP holds
  utilization: number; // percentage (0-1)
}

// Example
{
  total: 50000,
  sold: 32450,
  reserved: 1250, // pending payment
  available: 15800,
  blocked: 500,
  utilization: 0.675 // 67.5%
}
```

#### 7.2.2 Seat Locking

During purchase, seats MUST be locked for **10 minutes**:

```typescript
const lock = await lockSeat({
  seatId: 'A-12-15',
  duration: 600, // seconds
  userId: 'USER-123',
  sessionId: 'SESSION-ABC'
});

// Auto-release after timeout
setTimeout(() => releaseSeat(lock.id), lock.duration * 1000);
```

#### 7.2.3 Overbooking Prevention

The system MUST prevent overbooking using distributed locks:

```typescript
// Atomic seat reservation
const reservation = await atomicReserve({
  seats: ['A-12-15', 'A-12-16'],
  timeout: 600,
  consistency: 'strong' // require all or none
});
```

### 7.3 Seat Maps

#### 7.3.1 SVG Format

Venue seat maps SHOULD be provided in SVG format:

```xml
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 600">
  <g id="section-A">
    <rect id="seat-A-12-15"
          x="300" y="200"
          width="20" height="20"
          class="seat available"
          data-price="150.00"
          data-type="standard" />
  </g>
</svg>
```

Seat classes:
- `available`: Green (#10B981)
- `sold`: Gray (#6B7280)
- `selected`: Blue (#3B82F6)
- `blocked`: Red (#EF4444)
- `accessible`: Purple (#8B5CF6)

---

## 8. Dynamic Pricing Algorithms

### 8.1 Pricing Model

#### 8.1.1 Base Formula

The dynamic price at time `t` is calculated as:

```
P(t) = B × D(t) × T(t) × M(t) × S(t) × C(t)

Where:
- P(t) = Price at time t
- B = Base price (fixed)
- D(t) = Demand factor (0.5 - 3.0)
- T(t) = Time decay factor (0.6 - 1.5)
- M(t) = Market condition multiplier (0.8 - 1.3)
- S(t) = Supply scarcity factor (0.7 - 2.5)
- C(t) = Competition factor (0.9 - 1.1)
```

#### 8.1.2 Demand Factor D(t)

```typescript
function calculateDemandFactor(metrics: DemandMetrics): number {
  const {
    pageViews,        // Last 24h page views
    searchVolume,     // Search queries for event
    socialMentions,   // Social media mentions
    historyRate       // Similar event sell-through
  } = metrics;

  // Weighted scoring
  const score =
    (pageViews / 10000) * 0.3 +
    (searchVolume / 5000) * 0.3 +
    (socialMentions / 1000) * 0.2 +
    historyRate * 0.2;

  // Clamp to [0.5, 3.0]
  return Math.max(0.5, Math.min(3.0, score));
}
```

#### 8.1.3 Time Decay Factor T(t)

```typescript
function calculateTimeDecay(daysUntilEvent: number): number {
  if (daysUntilEvent > 90) {
    // Early bird: 40% discount
    return 0.6;
  } else if (daysUntilEvent > 60) {
    // Moderate early: 20% discount
    return 0.8;
  } else if (daysUntilEvent > 30) {
    // Normal pricing
    return 1.0;
  } else if (daysUntilEvent > 7) {
    // Rising: 10% increase
    return 1.1;
  } else if (daysUntilEvent > 1) {
    // Last week: 20% increase
    return 1.2;
  } else {
    // Last minute: 50% increase
    return 1.5;
  }
}
```

#### 8.1.4 Supply Scarcity Factor S(t)

```typescript
function calculateScarcityFactor(capacityRemaining: number): number {
  // capacityRemaining is percentage (0-1)

  if (capacityRemaining > 0.70) {
    // Plenty available: 30% discount
    return 0.7;
  } else if (capacityRemaining > 0.50) {
    // Half full: 10% discount
    return 0.9;
  } else if (capacityRemaining > 0.25) {
    // Getting scarce: normal price
    return 1.0;
  } else if (capacityRemaining > 0.10) {
    // Very limited: 50% increase
    return 1.5;
  } else {
    // Almost sold out: 150% increase
    return 2.5;
  }
}
```

#### 8.1.5 Market Condition Multiplier M(t)

```typescript
function calculateMarketMultiplier(conditions: MarketConditions): number {
  const {
    economicIndex,    // Economic indicator (0-100)
    seasonality,      // Season factor (0.8-1.2)
    competitorPrices, // Average competitor pricing
    weatherForecast   // Weather impact (outdoor events)
  } = conditions;

  let multiplier = 1.0;

  // Economic conditions
  if (economicIndex > 70) multiplier *= 1.1;
  else if (economicIndex < 40) multiplier *= 0.9;

  // Seasonality
  multiplier *= seasonality;

  // Competition (adjust to market)
  multiplier *= (competitorPrices / basePrice);

  // Weather (outdoor events only)
  if (weatherForecast === 'poor') multiplier *= 0.85;

  // Clamp to [0.8, 1.3]
  return Math.max(0.8, Math.min(1.3, multiplier));
}
```

### 8.2 Pricing Constraints

#### 8.2.1 Price Floor and Ceiling

```typescript
interface PricingConstraints {
  basePrice: number;
  minPrice: number;        // Never below this
  maxPrice: number;        // Never above this
  minPriceRatio: number;   // e.g., 0.5 (50% of base)
  maxPriceRatio: number;   // e.g., 2.5 (250% of base)
}

function applyConstraints(
  calculatedPrice: number,
  constraints: PricingConstraints
): number {
  const { basePrice, minPrice, maxPrice, minPriceRatio, maxPriceRatio } = constraints;

  const floor = Math.max(minPrice, basePrice * minPriceRatio);
  const ceiling = Math.min(maxPrice, basePrice * maxPriceRatio);

  return Math.max(floor, Math.min(ceiling, calculatedPrice));
}
```

#### 8.2.2 Price Change Rate Limiting

Prices MUST NOT change more than **10% per hour**:

```typescript
function rateLimitPriceChange(
  currentPrice: number,
  newPrice: number,
  timeDelta: number // seconds
): number {
  const maxChangePerHour = 0.10; // 10%
  const maxChange = currentPrice * maxChangePerHour * (timeDelta / 3600);

  if (newPrice > currentPrice + maxChange) {
    return currentPrice + maxChange;
  } else if (newPrice < currentPrice - maxChange) {
    return currentPrice - maxChange;
  }

  return newPrice;
}
```

### 8.3 Tiered Pricing

Different seat sections MAY have different base prices:

```typescript
const pricingTiers = {
  'VIP': { basePrice: 500, multiplier: 1.0 },
  'Premium': { basePrice: 300, multiplier: 1.0 },
  'Standard': { basePrice: 150, multiplier: 1.0 },
  'Balcony': { basePrice: 75, multiplier: 1.0 },
  'General': { basePrice: 50, multiplier: 1.0 }
};
```

---

## 9. Fraud Prevention

### 9.1 Multi-Layer Security

#### 9.1.1 Security Layers

1. **QR Code Encryption**: AES-256-GCM
2. **Digital Signature**: ECDSA (secp256k1)
3. **Blockchain Verification**: Ethereum/Polygon
4. **Biometric Binding**: Face ID / Fingerprint
5. **Geofencing**: GPS location validation
6. **TOTP Rotation**: 30-second code refresh
7. **Device Fingerprinting**: Browser/app identification
8. **AI Anomaly Detection**: Behavioral analysis

#### 9.1.2 Duplicate Detection

Real-time duplicate scan prevention:

```typescript
async function validateTicket(validation: TicketValidation): Promise<ValidationResult> {
  const { ticketId, qrCode, location, timestamp } = validation;

  // Check if already used
  const checkInRecord = await getCheckInHistory(ticketId);
  if (checkInRecord.status === 'checked-in') {
    return {
      isValid: false,
      error: 'DUPLICATE_SCAN',
      message: 'Ticket already used',
      originalCheckIn: checkInRecord.timestamp
    };
  }

  // Verify QR signature
  const qrValid = await verifyQRSignature(qrCode);
  if (!qrValid) {
    return {
      isValid: false,
      error: 'INVALID_QR',
      message: 'QR code verification failed'
    };
  }

  // Check geofence
  const withinVenue = await checkGeofence(location, validation.venueId);
  if (!withinVenue) {
    return {
      isValid: false,
      error: 'LOCATION_MISMATCH',
      message: 'Not within venue geofence'
    };
  }

  // Verify TOTP
  const totpValid = verifyTOTP(validation.totp, timestamp);
  if (!totpValid) {
    return {
      isValid: false,
      error: 'TOTP_EXPIRED',
      message: 'Time-based code expired'
    };
  }

  // Blockchain verification
  const blockchainValid = await verifyBlockchain(ticketId);
  if (!blockchainValid) {
    return {
      isValid: false,
      error: 'BLOCKCHAIN_INVALID',
      message: 'Blockchain ownership verification failed'
    };
  }

  // Mark as used
  await recordCheckIn(ticketId, location, timestamp);

  return {
    isValid: true,
    message: 'Ticket validated successfully',
    holder: await getTicketHolder(ticketId)
  };
}
```

### 9.2 Anti-Scalping Measures

#### 9.2.1 Purchase Limits

```typescript
interface PurchaseLimits {
  maxPerUser: number;          // e.g., 6 tickets
  maxPerTransaction: number;   // e.g., 10 tickets
  maxPerIP: number;            // e.g., 20 tickets
  maxPerCard: number;          // e.g., 12 tickets
  cooldownPeriod: number;      // seconds between purchases
}
```

#### 9.2.2 Bot Detection

```typescript
async function detectBot(request: PurchaseRequest): Promise<BotScore> {
  const signals = {
    // Request characteristics
    userAgent: analyzeUserAgent(request.userAgent),
    ipReputation: await checkIPReputation(request.ip),
    browserFingerprint: request.fingerprint,

    // Behavioral signals
    mouseMovements: request.mouseData?.entropy || 0,
    keystrokeDynamics: request.keystrokes?.variance || 0,
    pageNavigation: request.navigationPattern,

    // Time-based signals
    requestSpeed: calculateRequestSpeed(request),
    formFillTime: request.formCompletionTime
  };

  // ML model prediction
  const botProbability = await mlModel.predict(signals);

  return {
    probability: botProbability,
    signals,
    action: botProbability > 0.7 ? 'block' :
            botProbability > 0.4 ? 'challenge' :
            'allow'
  };
}
```

#### 9.2.3 CAPTCHA Integration

```typescript
if (botScore.action === 'challenge') {
  const captchaResult = await verifyCaptcha({
    token: request.captchaToken,
    action: 'ticket_purchase',
    expectedHostname: config.hostname
  });

  if (captchaResult.score < 0.5) {
    throw new Error('CAPTCHA verification failed');
  }
}
```

### 9.3 Blockchain Integration

#### 9.3.1 NFT Ticket Minting

```solidity
// Smart contract (Solidity)
pragma solidity ^0.8.0;

import "@openzeppelin/contracts/token/ERC721/ERC721.sol";

contract WIATicketNFT is ERC721 {
    struct Ticket {
        string ticketId;
        string eventId;
        uint256 expiryTime;
        bool used;
    }

    mapping(uint256 => Ticket) public tickets;
    uint256 private _tokenIdCounter;

    function mintTicket(
        address holder,
        string memory ticketId,
        string memory eventId,
        uint256 expiryTime
    ) public returns (uint256) {
        uint256 tokenId = _tokenIdCounter++;
        _safeMint(holder, tokenId);

        tickets[tokenId] = Ticket({
            ticketId: ticketId,
            eventId: eventId,
            expiryTime: expiryTime,
            used: false
        });

        return tokenId;
    }

    function validateTicket(uint256 tokenId) public returns (bool) {
        require(_exists(tokenId), "Ticket does not exist");
        require(block.timestamp < tickets[tokenId].expiryTime, "Ticket expired");
        require(!tickets[tokenId].used, "Ticket already used");

        tickets[tokenId].used = true;
        return true;
    }
}
```

#### 9.3.2 Blockchain Verification

```typescript
async function verifyBlockchain(ticketId: string): Promise<boolean> {
  const ticket = await getTicket(ticketId);
  const { hash, network, contractAddress, tokenId } = ticket.security.blockchain;

  // Connect to blockchain
  const provider = getProvider(network);
  const contract = new ethers.Contract(contractAddress, ABI, provider);

  // Verify ownership
  const owner = await contract.ownerOf(tokenId);
  const expectedOwner = await getHolderWalletAddress(ticket.holder.email);

  if (owner.toLowerCase() !== expectedOwner.toLowerCase()) {
    return false;
  }

  // Verify not used
  const ticketData = await contract.tickets(tokenId);
  if (ticketData.used) {
    return false;
  }

  return true;
}
```

---

## 10. Ticket Transfer and Resale

### 10.1 Transfer Mechanism

#### 10.1.1 Initiate Transfer

```typescript
async function transferTicket(transfer: TicketTransfer): Promise<TransferResult> {
  const {
    ticketId,
    fromEmail,
    toEmail,
    requireApproval,
    message
  } = transfer;

  // Verify ownership
  const ticket = await getTicket(ticketId);
  if (ticket.holder.email !== fromEmail) {
    throw new Error('Not ticket owner');
  }

  // Check if transferable
  if (!ticket.transferable) {
    throw new Error('Ticket is non-transferable');
  }

  // Create transfer request
  const transferRequest = await createTransferRequest({
    ticketId,
    from: fromEmail,
    to: toEmail,
    status: requireApproval ? 'pending' : 'auto-approved',
    createdAt: new Date(),
    message
  });

  // Send notifications
  await sendTransferNotification(toEmail, transferRequest);

  if (!requireApproval) {
    await executeTransfer(transferRequest.id);
  }

  return {
    success: true,
    transferId: transferRequest.id,
    status: transferRequest.status
  };
}
```

#### 10.1.2 Accept Transfer

```typescript
async function acceptTransfer(transferId: string, recipientEmail: string): Promise<void> {
  const transfer = await getTransferRequest(transferId);

  if (transfer.to !== recipientEmail) {
    throw new Error('Not the intended recipient');
  }

  if (transfer.status !== 'pending') {
    throw new Error('Transfer not pending');
  }

  await executeTransfer(transferId);
}

async function executeTransfer(transferId: string): Promise<void> {
  const transfer = await getTransferRequest(transferId);
  const ticket = await getTicket(transfer.ticketId);

  // Update ticket holder
  ticket.holder.email = transfer.to;
  ticket.holder.name = transfer.toName;
  ticket.holder.verified = false; // Require re-verification
  ticket.status = 'transferred';

  // Update blockchain ownership (if NFT)
  if (ticket.security.blockchain) {
    await transferNFT(
      ticket.security.blockchain.tokenId,
      transfer.from,
      transfer.to
    );
  }

  // Regenerate QR code
  ticket.security.qrCode = await generateNewQRCode(ticket);

  await updateTicket(ticket);
  await updateTransferStatus(transferId, 'completed');

  // Notify both parties
  await sendTransferCompletedNotification(transfer);
}
```

### 10.2 Resale Marketplace

#### 10.2.1 List for Resale

```typescript
async function listForResale(listing: ResaleListing): Promise<ListingResult> {
  const {
    ticketId,
    askingPrice,
    sellerId,
    expiryDate
  } = listing;

  const ticket = await getTicket(ticketId);

  // Verify resellable
  if (!ticket.resellable) {
    throw new Error('Ticket is non-resellable');
  }

  // Check price constraints
  const rules = await getResaleRules(ticket.eventId);
  if (askingPrice > ticket.pricing.finalPrice * rules.maxMarkup) {
    throw new Error(`Price exceeds maximum markup of ${rules.maxMarkup * 100}%`);
  }

  if (askingPrice < ticket.pricing.finalPrice * rules.minPriceRatio) {
    throw new Error(`Price below minimum of ${rules.minPriceRatio * 100}%`);
  }

  // Create listing
  const marketplaceListing = await createListing({
    ticketId,
    sellerId,
    askingPrice,
    originalPrice: ticket.pricing.finalPrice,
    listedAt: new Date(),
    expiryDate,
    status: 'active',
    transferFee: rules.transferFee
  });

  // Lock ticket
  await lockTicket(ticketId, 'resale-listing');

  return {
    success: true,
    listingId: marketplaceListing.id,
    url: `https://marketplace.wia.com/listing/${marketplaceListing.id}`
  };
}
```

#### 10.2.2 Purchase from Resale

```typescript
async function purchaseResaleTicket(
  listingId: string,
  buyerId: string
): Promise<PurchaseResult> {
  const listing = await getListing(listingId);

  if (listing.status !== 'active') {
    throw new Error('Listing not active');
  }

  // Process payment
  const payment = await processPayment({
    amount: listing.askingPrice + listing.transferFee,
    buyerId,
    sellerId: listing.sellerId,
    description: `Resale ticket ${listing.ticketId}`
  });

  if (!payment.success) {
    throw new Error('Payment failed');
  }

  // Transfer ticket
  const transfer = await transferTicket({
    ticketId: listing.ticketId,
    fromEmail: listing.sellerEmail,
    toEmail: listing.buyerEmail,
    requireApproval: false
  });

  // Update listing
  await updateListing(listingId, {
    status: 'sold',
    soldAt: new Date(),
    buyerId,
    finalPrice: listing.askingPrice
  });

  // Distribute payment
  await distributePayment({
    seller: listing.askingPrice,
    platform: listing.transferFee,
    originalIssuer: listing.transferFee * 0.1 // 10% to original issuer
  });

  return {
    success: true,
    ticketId: listing.ticketId,
    transferId: transfer.transferId,
    amountPaid: listing.askingPrice + listing.transferFee
  };
}
```

#### 10.2.3 Anti-Scalping Rules

```typescript
interface ResaleRules {
  enabled: boolean;
  maxMarkup: number;        // e.g., 1.2 (120% of face value)
  minPriceRatio: number;    // e.g., 0.8 (80% of face value)
  transferFee: number;      // e.g., 5.00 USD
  maxListingsPerUser: number; // e.g., 4 tickets
  verificationRequired: boolean;
  cooldownPeriod: number;   // seconds before relist
}
```

---

## 11. Multi-Venue Support

### 11.1 Venue Registry

```typescript
interface Venue {
  id: string;
  name: string;
  type: 'stadium' | 'arena' | 'theater' | 'festival' | 'cinema' | 'museum' | 'virtual';
  location: {
    address: string;
    city: string;
    country: string;
    coordinates: { lat: number; lon: number };
    timezone: string;
  };
  capacity: {
    total: number;
    sections: {
      [sectionId: string]: {
        capacity: number;
        type: 'seated' | 'standing' | 'accessible';
      };
    };
  };
  amenities: string[];
  accessibility: {
    wheelchairAccess: boolean;
    elevators: boolean;
    assistiveListening: boolean;
    signLanguage: boolean;
  };
  contactInfo: {
    phone: string;
    email: string;
    website: string;
  };
}
```

### 11.2 Multi-Venue Events

```typescript
// Concert tour across multiple venues
const tour = await createTourEvent({
  name: 'World Tour 2025',
  artist: 'Global Superstar',
  dates: [
    { venueId: 'VEN-NYC-001', date: '2025-06-01T20:00:00Z' },
    { venueId: 'VEN-LA-042', date: '2025-06-10T20:00:00Z' },
    { venueId: 'VEN-CHI-015', date: '2025-06-20T20:00:00Z' },
    { venueId: 'VEN-LON-007', date: '2025-07-01T19:00:00Z' }
  ],
  ticketTypes: [
    { type: 'single-show', venues: 'any' },
    { type: 'vip-package-all', venues: 'all', price: 2000 },
    { type: 'regional-pass', venues: ['VEN-NYC-001', 'VEN-CHI-015'], price: 400 }
  ]
});
```

### 11.3 Venue Synchronization

```typescript
// Real-time inventory sync across venues
const syncSystem = new MultiVenueSyncSystem({
  venues: ['VEN-001', 'VEN-002', 'VEN-003'],
  syncInterval: 5000, // 5 seconds
  conflictResolution: 'last-write-wins'
});

await syncSystem.start();
```

---

## 12. Season Pass Management

### 12.1 Season Pass Structure

```typescript
interface SeasonPass {
  passId: string;
  passType: 'full-season' | 'partial-season' | 'flex-pass' | 'unlimited';
  holder: TicketHolder;
  validity: {
    startDate: Date;
    endDate: Date;
    timezone: string;
  };
  allocation: {
    totalEvents: number;
    usedEvents: number;
    remainingEvents: number;
    includedEvents: string[]; // Event IDs
  };
  benefits: {
    priorityAccess: boolean;
    discounts: { type: string; amount: number }[];
    exclusiveContent: boolean;
    transferable: boolean;
  };
  pricing: {
    totalPrice: number;
    perEventPrice: number;
    currency: string;
    paymentPlan?: {
      installments: number;
      frequency: 'monthly' | 'quarterly';
    };
  };
  status: 'active' | 'suspended' | 'expired' | 'cancelled';
  renewalDate?: Date;
  autoRenew: boolean;
}
```

### 12.2 Event Redemption

```typescript
async function redeemSeasonPass(
  passId: string,
  eventId: string,
  seatPreference?: string
): Promise<Ticket> {
  const pass = await getSeasonPass(passId);

  // Verify validity
  if (pass.status !== 'active') {
    throw new Error('Season pass not active');
  }

  if (!pass.allocation.includedEvents.includes(eventId)) {
    throw new Error('Event not included in season pass');
  }

  if (pass.allocation.remainingEvents <= 0) {
    throw new Error('No events remaining');
  }

  // Check if already redeemed for this event
  const existingTicket = await findTicket({ passId, eventId });
  if (existingTicket) {
    throw new Error('Already redeemed for this event');
  }

  // Allocate seat
  const seat = await allocateSeat({
    eventId,
    preference: seatPreference,
    priority: pass.benefits.priorityAccess
  });

  // Create ticket
  const ticket = await createTicket({
    eventId,
    seat,
    holder: pass.holder,
    pricing: { finalPrice: 0, currency: pass.pricing.currency },
    metadata: {
      seasonPassId: passId,
      redemptionDate: new Date()
    }
  });

  // Update pass allocation
  pass.allocation.usedEvents++;
  pass.allocation.remainingEvents--;
  await updateSeasonPass(pass);

  return ticket;
}
```

---

## 13. Access Control Integration

### 13.1 Supported Access Control Systems

- **Turnstiles**: Automatic gate control
- **Handheld Scanners**: Mobile validation devices
- **NFC Readers**: Contactless validation
- **Biometric Kiosks**: Fingerprint/face recognition
- **Mobile Apps**: Self-service validation
- **QR Scanners**: Camera-based validation

### 13.2 Validation Protocol

```typescript
interface ValidationRequest {
  ticketId: string;
  validationMethod: 'qr' | 'barcode' | 'nfc' | 'biometric';
  validationData: string; // QR payload, NFC UID, etc.
  location: {
    venueId: string;
    gate: string;
    coordinates?: { lat: number; lon: number };
  };
  timestamp: Date;
  deviceId: string;
  validatorId?: string; // Staff member ID
}

interface ValidationResponse {
  isValid: boolean;
  ticketId: string;
  holder: {
    name: string;
    verified: boolean;
  };
  seat?: {
    section: string;
    row: string;
    seat: string;
  };
  warnings: string[];
  errors: string[];
  action: 'allow' | 'deny' | 'manual-check';
  accessLevel: 'standard' | 'vip' | 'staff' | 'accessible';
}
```

### 13.3 Offline Validation

```typescript
// Generate offline validation bundle
const offlineBundle = await generateOfflineBundle({
  eventId: 'EVT-2025-001',
  validUntil: new Date('2025-06-21T02:00:00Z'),
  includeTickets: 'all',
  encryptionKey: 'SECURE_KEY'
});

// Validate offline
const validator = new OfflineValidator(offlineBundle);
const result = validator.validate({
  qrCode: scannedQR,
  timestamp: new Date()
});
```

---

## 14. API Specification

### 14.1 RESTful API Endpoints

```
POST   /api/v1/tickets              - Create ticket
GET    /api/v1/tickets/:id          - Get ticket details
PUT    /api/v1/tickets/:id          - Update ticket
DELETE /api/v1/tickets/:id          - Cancel ticket

POST   /api/v1/tickets/:id/validate - Validate ticket
POST   /api/v1/tickets/:id/transfer - Transfer ticket
POST   /api/v1/tickets/:id/checkin  - Check-in ticket

GET    /api/v1/events/:id           - Get event details
GET    /api/v1/events/:id/seats     - Get seat availability
POST   /api/v1/events/:id/reserve   - Reserve seats

GET    /api/v1/pricing/dynamic      - Calculate dynamic price
GET    /api/v1/pricing/forecast     - Price forecast

POST   /api/v1/resale/listings      - Create resale listing
GET    /api/v1/resale/listings/:id  - Get listing details
POST   /api/v1/resale/purchase      - Purchase resale ticket

GET    /api/v1/season-passes/:id    - Get season pass
POST   /api/v1/season-passes/:id/redeem - Redeem for event
```

### 14.2 Authentication

All API requests MUST include authentication:

```
Authorization: Bearer <JWT_TOKEN>
X-API-Key: <API_KEY>
```

### 14.3 Rate Limiting

- **Standard tier**: 100 requests/minute
- **Premium tier**: 1000 requests/minute
- **Enterprise tier**: 10000 requests/minute

---

## 15. Security Requirements

### 15.1 Encryption

- **Data in transit**: TLS 1.3
- **Data at rest**: AES-256
- **QR codes**: AES-256-GCM
- **Private keys**: Hardware Security Module (HSM)

### 15.2 Key Management

```typescript
const keyManagement = {
  rotation: '90-days',
  storage: 'HSM',
  backup: 'encrypted-offline',
  access: 'multi-party-computation'
};
```

---

## 16. Privacy and Data Protection

### 16.1 GDPR Compliance

- **Right to Access**: Provide ticket data export
- **Right to Erasure**: Delete user data upon request
- **Data Minimization**: Collect only necessary information
- **Consent**: Explicit opt-in for marketing
- **Data Portability**: JSON export format

### 16.2 PII Protection

Personally Identifiable Information MUST be:
- Encrypted at rest
- Masked in logs
- Redacted in exports
- Deleted after retention period

---

## 17. Interoperability

### 17.1 WIA Ecosystem Integration

- **WIA-INTENT**: Intent-based ticket discovery
- **WIA-OMNI-API**: Universal API gateway
- **WIA-SOCIAL**: Social sharing
- **WIA-PAYMENT**: Payment processing
- **WIA-IDENTITY**: Identity verification

### 17.2 Third-Party Integration

Support for:
- **Ticketmaster** API
- **Eventbrite** API
- **StubHub** API
- **AXS** API
- **SeatGeek** API

---

## 18. Testing and Certification

### 18.1 Compliance Testing

Implementations MUST pass:
1. QR code generation and validation
2. Dynamic pricing calculations
3. Fraud detection scenarios
4. Transfer and resale workflows
5. Multi-venue synchronization
6. Offline validation
7. Accessibility compliance (WCAG 2.1 AA)

### 18.2 WIA Certification

Contact: cert.wiastandards.com

---

## 19. Appendices

### Appendix A: Sample Implementations

See `/api/typescript/` for reference implementation.

### Appendix B: Test Vectors

See `/spec/test-vectors.json` for validation test cases.

### Appendix C: Migration Guide

For migrating from legacy ticketing systems to WIA-IND-019.

### Appendix D: Glossary

Complete glossary of technical terms used in this specification.

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-27 | Initial release |

---

## Copyright and License

© 2025 SmileStory Inc. / WIA

This specification is licensed under the MIT License.

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
