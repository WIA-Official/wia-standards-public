# WIA-AI-023 NLP Standard - Phase 1: Data Format

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-25

---

## 1. Overview

Phase 1 defines standardized data formats for Natural Language Processing inputs, outputs, and intermediate representations. Consistent data formats ensure interoperability between different NLP systems and enable seamless data exchange.

### 1.1 Goals

- Define JSON schemas for all NLP tasks
- Standardize text encoding and representation
- Specify metadata requirements
- Enable language-agnostic processing
- Support multilingual applications

## 2. Core Data Schema

### 2.1 Base Request Format

All NLP requests follow this base schema:

```json
{
  "standard": "WIA-AI-023",
  "version": "1.0",
  "task": "string",
  "input": {
    "text": "string",
    "language": "string (ISO 639-1)",
    "encoding": "UTF-8"
  },
  "config": {},
  "metadata": {
    "request_id": "uuid",
    "timestamp": "ISO 8601",
    "client_info": {}
  }
}
```

### 2.2 Base Response Format

```json
{
  "standard": "WIA-AI-023",
  "version": "1.0",
  "task": "string",
  "output": {},
  "metadata": {
    "request_id": "uuid",
    "timestamp": "ISO 8601",
    "processing_time_ms": "number",
    "model": "string",
    "confidence": "number (0-1)"
  },
  "status": {
    "code": "number",
    "message": "string"
  }
}
```

## 3. Task-Specific Data Formats

### 3.1 Tokenization

**Request:**
```json
{
  "task": "tokenization",
  "input": {
    "text": "Natural language processing is essential.",
    "language": "en"
  },
  "config": {
    "method": "word|subword|character",
    "preserve_case": true,
    "include_offsets": true
  }
}
```

**Response:**
```json
{
  "output": {
    "tokens": ["Natural", "language", "processing", "is", "essential", "."],
    "token_count": 6,
    "offsets": [
      {"start": 0, "end": 7},
      {"start": 8, "end": 16},
      {"start": 17, "end": 27},
      {"start": 28, "end": 30},
      {"start": 31, "end": 40},
      {"start": 40, "end": 41}
    ]
  }
}
```

### 3.2 Named Entity Recognition (NER)

**Request:**
```json
{
  "task": "named_entity_recognition",
  "input": {
    "text": "Apple CEO Tim Cook visited Cupertino on January 15th.",
    "language": "en"
  },
  "config": {
    "entity_types": ["PERSON", "ORGANIZATION", "LOCATION", "DATE"],
    "aggregation": "simple"
  }
}
```

**Response:**
```json
{
  "output": {
    "entities": [
      {
        "text": "Apple",
        "type": "ORGANIZATION",
        "start": 0,
        "end": 5,
        "confidence": 0.98
      },
      {
        "text": "Tim Cook",
        "type": "PERSON",
        "start": 10,
        "end": 18,
        "confidence": 0.99
      },
      {
        "text": "Cupertino",
        "type": "LOCATION",
        "start": 27,
        "end": 36,
        "confidence": 0.97
      },
      {
        "text": "January 15th",
        "type": "DATE",
        "start": 40,
        "end": 52,
        "confidence": 0.95
      }
    ],
    "entity_count": 4
  }
}
```

### 3.3 Sentiment Analysis

**Request:**
```json
{
  "task": "sentiment_analysis",
  "input": {
    "text": "This product is absolutely amazing!",
    "language": "en"
  },
  "config": {
    "granularity": "document|sentence",
    "return_scores": true
  }
}
```

**Response:**
```json
{
  "output": {
    "sentiment": "positive",
    "confidence": 0.96,
    "scores": {
      "positive": 0.96,
      "neutral": 0.03,
      "negative": 0.01
    },
    "polarity": 0.92,
    "subjectivity": 0.85
  }
}
```

### 3.4 Text Classification

**Request:**
```json
{
  "task": "text_classification",
  "input": {
    "text": "Scientists discovered a new exoplanet orbiting a distant star.",
    "language": "en"
  },
  "config": {
    "categories": ["Technology", "Science", "Business", "Sports", "Politics"],
    "top_k": 3,
    "threshold": 0.1
  }
}
```

**Response:**
```json
{
  "output": {
    "predictions": [
      {
        "category": "Science",
        "confidence": 0.94,
        "rank": 1
      },
      {
        "category": "Technology",
        "confidence": 0.28,
        "rank": 2
      },
      {
        "category": "Business",
        "confidence": 0.05,
        "rank": 3
      }
    ],
    "primary_category": "Science"
  }
}
```

### 3.5 Text Generation

**Request:**
```json
{
  "task": "text_generation",
  "input": {
    "prompt": "Natural language processing enables",
    "language": "en"
  },
  "config": {
    "max_length": 100,
    "temperature": 0.8,
    "top_p": 0.95,
    "top_k": 50,
    "num_return_sequences": 3,
    "do_sample": true,
    "stop_sequences": ["\n\n"]
  }
}
```

**Response:**
```json
{
  "output": {
    "generated_texts": [
      "Natural language processing enables computers to understand and generate human language effectively.",
      "Natural language processing enables advanced chatbots and virtual assistants.",
      "Natural language processing enables sentiment analysis and text classification at scale."
    ],
    "generation_count": 3
  }
}
```

### 3.6 Text Summarization

**Request:**
```json
{
  "task": "summarization",
  "input": {
    "text": "Long document text here...",
    "language": "en"
  },
  "config": {
    "method": "abstractive|extractive",
    "max_length": 150,
    "min_length": 50,
    "num_sentences": null
  }
}
```

**Response:**
```json
{
  "output": {
    "summary": "Concise summary of the document...",
    "summary_length": 87,
    "compression_ratio": 0.15,
    "key_points": [
      "First key point",
      "Second key point",
      "Third key point"
    ]
  }
}
```

## 4. Language Codes

All language specifications use ISO 639-1 two-letter codes:

| Code | Language      |
|------|---------------|
| en   | English       |
| ko   | Korean        |
| es   | Spanish       |
| fr   | French        |
| de   | German        |
| zh   | Chinese       |
| ja   | Japanese      |
| ar   | Arabic        |
| hi   | Hindi         |
| pt   | Portuguese    |

## 5. Error Handling

### 5.1 Error Response Format

```json
{
  "standard": "WIA-AI-023",
  "version": "1.0",
  "status": {
    "code": 400,
    "message": "Invalid input format",
    "details": "Text field is required"
  },
  "metadata": {
    "request_id": "uuid",
    "timestamp": "ISO 8601"
  }
}
```

### 5.2 Standard Error Codes

| Code | Meaning                  | Description                        |
|------|--------------------------|------------------------------------|
| 200  | Success                  | Request completed successfully     |
| 400  | Bad Request              | Invalid input format or parameters |
| 401  | Unauthorized             | Invalid or missing authentication  |
| 429  | Too Many Requests        | Rate limit exceeded                |
| 500  | Internal Server Error    | Server-side processing error       |
| 503  | Service Unavailable      | Service temporarily unavailable    |

## 6. Validation Rules

### 6.1 Text Input Validation

- **Encoding:** Must be valid UTF-8
- **Length:** Maximum 100,000 characters (configurable per implementation)
- **Empty text:** Returns error code 400
- **Special characters:** Preserved unless explicitly configured otherwise

### 6.2 Language Code Validation

- Must be valid ISO 639-1 code
- Case-insensitive (normalized to lowercase)
- Unknown languages return error code 400

### 6.3 Confidence Scores

- Range: 0.0 to 1.0 (inclusive)
- Precision: At least 2 decimal places
- Missing confidence defaults to null

## 7. Metadata Requirements

### 7.1 Mandatory Metadata

Every response must include:
- `request_id`: Unique identifier (UUID v4)
- `timestamp`: ISO 8601 format
- `standard`: "WIA-AI-023"
- `version`: Semantic version string

### 7.2 Optional Metadata

- `processing_time_ms`: Processing duration
- `model`: Model identifier
- `confidence`: Overall confidence score
- `warnings`: Array of warning messages
- `debug_info`: Debugging information (development only)

## 8. Compliance Checklist

- [ ] All requests include standard and version fields
- [ ] Text encoding is UTF-8
- [ ] Language codes use ISO 639-1
- [ ] Response includes request_id and timestamp
- [ ] Error responses follow standard format
- [ ] Confidence scores are in range [0, 1]
- [ ] Entity offsets are character-based (not byte-based)
- [ ] All JSON is valid and well-formed

---

**Next:** [Phase 2: API Interface](PHASE-2-API.md)

**弘益人間** · Benefit All Humanity

© 2025 WIA - World Certification Industry Association

---

## Annex A — Conformance Tier Matrix

WIA conformance for nlp-standard is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/nlp-standard/api/` — TypeScript SDK skeleton
- `wia-standards/standards/nlp-standard/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/nlp-standard/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

