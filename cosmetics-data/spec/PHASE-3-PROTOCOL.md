# WIA-IND-005: Phase 3 - Protocol Specification

**Version:** 1.0
**Status:** Active
**Philosophy:** 弘益人間 - Benefit All Humanity

## Overview

Phase 3 defines protocols for data exchange between cosmetic systems including manufacturers, regulatory bodies, retailers, and consumer applications. This specification ensures reliable, secure, and auditable communication across the cosmetics ecosystem.

## Data Exchange Protocol

### Protocol Stack

```
┌─────────────────────────────────┐
│  Application Layer (JSON/XML)  │
├─────────────────────────────────┤
│  Transport Layer (HTTPS/WSS)   │
├─────────────────────────────────┤
│  Security Layer (TLS 1.3+)     │
├─────────────────────────────────┤
│  Network Layer (TCP/IP)         │
└─────────────────────────────────┘
```

### Message Format

#### Standard Message Envelope

```json
{
  "protocol": "WIA-IND-005-EXCHANGE",
  "version": "1.0",
  "message_id": "msg_abc123def456",
  "timestamp": "2025-01-15T10:30:00Z",
  "source": {
    "system_id": "manufacturer_erp_001",
    "organization": "Acme Cosmetics Inc",
    "country": "US",
    "contact": "api@acmecosmetics.com"
  },
  "destination": {
    "system_id": "regulatory_db_eu",
    "organization": "EU Commission",
    "country": "EU"
  },
  "payload": {
    "type": "product_registration",
    "data": {...}
  },
  "security": {
    "encryption": "AES-256-GCM",
    "signature": "SHA-256-RSA",
    "signature_value": "base64_encoded_signature",
    "certificate_fingerprint": "sha256_fingerprint"
  },
  "philosophy": "弘益人間 - Secure exchange for global benefit"
}
```

## Regulatory Synchronization Protocol

### Product Registration Flow

#### 1. Registration Request

**Manufacturer → Regulatory Authority**

```json
{
  "protocol": "WIA-IND-005-REGULATORY-SYNC",
  "sync_type": "product_registration",
  "payload": {
    "product": {
      "product_id": "PROD-12345",
      "name": "Hydrating Serum",
      "gtin": "1234567890123",
      "ingredients": [...],
      "formulation": {...},
      "safety_assessment": {...},
      "manufacturing": {
        "facility_id": "FAC-001",
        "gmp_certified": true
      }
    },
    "target_market": "EU",
    "responsible_person": {
      "name": "John Doe",
      "address": "123 Main St, Brussels, Belgium",
      "email": "john.doe@acmecosmetics.com"
    },
    "pif_reference": "https://storage.acmecosmetics.com/pif/PROD-12345.pdf"
  }
}
```

#### 2. Registration Acknowledgment

**Regulatory Authority → Manufacturer**

```json
{
  "protocol": "WIA-IND-005-REGULATORY-SYNC",
  "sync_type": "registration_acknowledgment",
  "status": "received",
  "registration_id": "REG-EU-2025-001234",
  "payload": {
    "submission_date": "2025-01-15T10:30:00Z",
    "estimated_review_time": "30 days",
    "required_documents": [
      "Safety Assessment Report",
      "Manufacturing Process Validation"
    ],
    "next_steps": [
      "Submit additional documentation within 14 days",
      "Await compliance review"
    ]
  }
}
```

#### 3. Compliance Decision

```json
{
  "protocol": "WIA-IND-005-REGULATORY-SYNC",
  "sync_type": "compliance_decision",
  "registration_id": "REG-EU-2025-001234",
  "decision": "approved",
  "payload": {
    "approval_date": "2025-02-15T10:30:00Z",
    "valid_until": "2030-02-15",
    "conditions": [],
    "cpnp_number": "CPNP-123456",
    "restrictions": []
  },
  "philosophy": "弘益人間 - Transparent regulatory process"
}
```

### Safety Alert Propagation

#### Alert Broadcasting

```json
{
  "protocol": "WIA-IND-005-SAFETY-ALERT",
  "alert_id": "ALERT-2025-001",
  "severity": "high",
  "alert_type": "ingredient_restriction",
  "payload": {
    "ingredient": {
      "inci_name": "Methylisothiazolinone",
      "cas_number": "2682-20-4"
    },
    "change": {
      "type": "max_concentration_reduced",
      "old_limit": "0.01%",
      "new_limit": "0.0015%",
      "effective_date": "2025-03-01",
      "regulation": "EU Regulation 1223/2009 Amendment"
    },
    "affected_products": "All leave-on cosmetics",
    "required_action": "Reformulate or relabel products exceeding new limit",
    "deadline": "2025-06-01"
  },
  "broadcast_to": ["all_manufacturers", "all_retailers"],
  "philosophy": "弘益人間 - Rapid safety communication"
}
```

## Supply Chain Data Exchange

### Ingredient Sourcing Protocol

#### Certificate of Analysis (CoA) Exchange

```json
{
  "protocol": "WIA-IND-005-SUPPLY-CHAIN",
  "exchange_type": "certificate_of_analysis",
  "payload": {
    "ingredient": {
      "inci_name": "Hyaluronic Acid",
      "batch_number": "HA-2025-001",
      "quantity": "100 kg",
      "supplier": "Global Ingredients Co"
    },
    "analysis": {
      "purity": "≥95%",
      "molecular_weight": "1,000,000-1,500,000 Da",
      "appearance": "White powder",
      "microbiology": {
        "total_aerobic_count": "<100 CFU/g",
        "yeast_mold": "<10 CFU/g",
        "ecoli": "Negative",
        "salmonella": "Negative"
      },
      "heavy_metals": {
        "lead": "<0.5 ppm",
        "arsenic": "<0.5 ppm",
        "mercury": "<0.1 ppm"
      }
    },
    "certifications": [
      {
        "type": "Halal",
        "certificate_number": "HAL-2024-12345",
        "valid_until": "2026-01-15"
      },
      {
        "type": "Kosher",
        "certificate_number": "KOS-2024-67890",
        "valid_until": "2026-01-15"
      }
    ],
    "blockchain_anchor": {
      "network": "Ethereum",
      "tx_hash": "0x1234567890abcdef",
      "block_number": 12345678
    }
  },
  "philosophy": "弘益人間 - Transparent supply chain"
}
```

### Batch Tracking Protocol

#### Batch Transfer

```json
{
  "protocol": "WIA-IND-005-BATCH-TRACKING",
  "transfer_type": "manufacturer_to_distributor",
  "payload": {
    "product": {
      "product_id": "PROD-12345",
      "batch_number": "B2025-001234",
      "manufacture_date": "2025-01-10",
      "expiry_date": "2027-01-10",
      "quantity": 10000,
      "unit": "units"
    },
    "transfer": {
      "from": {
        "organization": "Acme Cosmetics Inc",
        "facility": "Main Factory",
        "location": "Seoul, South Korea"
      },
      "to": {
        "organization": "Global Distributors LLC",
        "facility": "Distribution Center A",
        "location": "Los Angeles, USA"
      },
      "transfer_date": "2025-01-15",
      "shipping_info": {
        "carrier": "DHL",
        "tracking_number": "DHL123456789",
        "conditions": "Store at 15-25°C"
      }
    },
    "blockchain_anchor": {
      "network": "VeChain",
      "tx_hash": "0xabcdef123456",
      "timestamp": "2025-01-15T10:30:00Z"
    }
  },
  "philosophy": "弘益人間 - End-to-end traceability"
}
```

## Consumer Application Protocol

### Product Scan Protocol

#### QR Code Scan Request

**Consumer App → WIA System**

```json
{
  "protocol": "WIA-IND-005-CONSUMER",
  "request_type": "product_scan",
  "payload": {
    "scan_data": {
      "barcode_type": "QR",
      "data": "https://cosmetics.wia-standards.org/p/PROD-12345",
      "timestamp": "2025-01-15T10:30:00Z",
      "location": {
        "country": "US",
        "language": "en"
      }
    },
    "user_preferences": {
      "language": "en",
      "allergen_alerts": true,
      "detailed_info": true
    },
    "user_profile": {
      "allergens": ["Limonene", "Nuts"],
      "preferences": ["Vegan", "Cruelty-Free"]
    }
  }
}
```

#### Product Information Response

```json
{
  "protocol": "WIA-IND-005-CONSUMER",
  "response_type": "product_info",
  "payload": {
    "product": {
      "name": "Hydrating Serum",
      "brand": "Acme Cosmetics",
      "category": "Skin Care - Serum"
    },
    "safety_summary": {
      "overall_rating": 9.2,
      "allergen_alert": false,
      "user_safe": true
    },
    "ingredients": [
      {
        "inci": "Aqua",
        "common_name": "Water",
        "percentage": "60-70%",
        "function": "Solvent",
        "safety_score": 10
      },
      {
        "inci": "Hyaluronic Acid",
        "common_name": "Hyaluronic Acid",
        "percentage": "2-5%",
        "function": "Hydration Booster",
        "safety_score": 9
      }
    ],
    "certifications": ["Vegan", "Cruelty-Free", "Dermatologist Tested"],
    "sustainability": {
      "packaging_recyclable": true,
      "carbon_footprint": "Low",
      "certifications": ["EcoCert"]
    }
  },
  "philosophy": "弘익人間 - Empowering consumer choices"
}
```

## Protocol Security

### Encryption Requirements

- **Transport:** TLS 1.3 or higher
- **Message Payload:** AES-256-GCM
- **Key Exchange:** ECDHE (Elliptic Curve Diffie-Hellman Ephemeral)
- **Digital Signatures:** RSA-4096 or ECDSA P-384

### Authentication

#### Mutual TLS (mTLS)

Both parties must present valid certificates:

```
Client Certificate:
  Subject: CN=acmecosmetics.com, O=Acme Cosmetics Inc, C=US
  Issuer: CN=WIA Certificate Authority

Server Certificate:
  Subject: CN=api.wia-standards.org, O=WIA, C=INTL
  Issuer: CN=WIA Root CA
```

### Message Integrity

#### HMAC Verification

```json
{
  "message": {...},
  "hmac": {
    "algorithm": "HMAC-SHA256",
    "value": "base64_encoded_hmac",
    "key_id": "key_identifier"
  }
}
```

## Error Handling and Retry Logic

### Error Response Format

```json
{
  "protocol": "WIA-IND-005-ERROR",
  "error": {
    "code": "SYNC_FAILED",
    "message": "Regulatory synchronization failed",
    "details": {
      "reason": "Missing required field: safety_assessment",
      "field": "payload.safety_assessment"
    },
    "retry_after": 3600,
    "support_contact": "support@wia-standards.org"
  }
}
```

### Retry Strategy

- **Exponential Backoff:** 1s, 2s, 4s, 8s, 16s
- **Max Retries:** 5
- **Circuit Breaker:** After 10 consecutive failures
- **Idempotency:** Use message_id for deduplication

## Audit Trail

### Audit Log Format

```json
{
  "audit_id": "AUDIT-2025-001234",
  "timestamp": "2025-01-15T10:30:00Z",
  "event_type": "regulatory_sync",
  "actor": {
    "system_id": "manufacturer_erp_001",
    "user": "john.doe@acmecosmetics.com"
  },
  "action": "product_registration_submitted",
  "result": "success",
  "blockchain_anchor": {
    "network": "Hyperledger Fabric",
    "channel": "cosmetics-regulatory",
    "tx_id": "abc123def456"
  },
  "philosophy": "弘益人間 - Transparent and accountable"
}
```

## Philosophy Statement

**弘益人間 (Benefit All Humanity):** These protocols are designed for interoperability and transparency. By standardizing data exchange across the cosmetics ecosystem, we enable seamless collaboration between manufacturers, regulators, retailers, and consumers. Open protocols prevent vendor lock-in and promote innovation while maintaining safety and compliance.

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Status:** Active
**Philosophy:** 弘益人間 - Benefit All Humanity
