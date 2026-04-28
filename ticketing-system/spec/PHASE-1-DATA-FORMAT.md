# WIA-IND-019 — Phase 1: Data Format

> Ticketing-system canonical Phase 1: ticket + barcode + event + seat-map envelopes.

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




---

## A.1 Ticket envelope (canonical)

The canonical ticket envelope carries: ticket identifier (UUID v4 or the venue's local convention), event identifier, venue identifier, seat identifier (section, row, seat), face value with currency (ISO 4217), holder identifier (optional, anonymous tickets supported), valid-from / valid-until timestamps (RFC 3339), barcode payload (Phase 1 §A.2), digital-signature block (Ed25519 by default), transfer-history block (Phase 3), and metadata (purchase channel, payment-token reference, fraud-score at issuance).

## A.2 Barcode and QR-code payload

QR-code payloads follow ISO/IEC 18004 (Model 2 QR, error correction H for outdoor durability). The payload is a base64url-encoded JSON Web Token (JWT) per IETF RFC 7519 with claims: ticket_id, event_id, venue_id, seat_id, valid_from, valid_until, holder_id, issued_at. The signature uses Ed25519 (preferred) or RS256 (legacy). Aztec Code (ISO/IEC 24778) is supported for IATA-style legacy compatibility with airline boarding passes.

## A.3 Event and venue descriptor

Event descriptors carry: event_id, name, type (concert, sports, theatre, conference, exhibition), date_range, venue_id, capacity, section_map_url (referencing a SVG or GeoJSON map per Phase 4 §A.3), pricing_tiers, and a status (`scheduled`, `on_sale`, `sold_out`, `cancelled`, `postponed`). Venue descriptors carry: venue_id, name, address, geo_coordinates, total_capacity, accessibility_features, section list with seat-map URI.

## A.4 Seat-map representation

Seat maps are stored as SVG with stable IDs per seat (`seat-{section}-{row}-{seat}`) so seat-selection front-ends can highlight available seats by ID lookup. Alternative representations: GeoJSON for venue-level (large stadium) maps, HTML5 canvas + JSON for compact mobile renders. The canonical representation is SVG; converters live in the reference container.

## A.5 Pricing-tier and dynamic-price envelope

Pricing-tier descriptors carry: tier_id, name, base_price (Money), seat_set (reference to SVG IDs), and the dynamic-price policy reference (Phase 3 §A.2). Dynamic-price envelopes carry the policy_id, the inputs (demand, time-to-event, weather where applicable), the computed price, the currency, and the timestamp.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/ticketing-system/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-ticketing-system-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/ticketing-system-host:1.0.0` ships every ticketing-system envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/ticketing-system.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Ticketing-system deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
