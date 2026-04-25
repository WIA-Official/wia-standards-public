# WIA-FINANCIAL_INCLUSION: Phase 1 - Data Format Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the data format requirements for FINANCIAL INCLUSION. All implementations MUST follow these specifications to ensure interoperability across systems.

## 2. Data Structures

### 2.1 Primary Record Format

```json
{
  "type": "WIA-FINANCIAL_INCLUSIONRecord",
  "version": "1.0",
  "id": "string (UUID v4)",
  "timestamp": "ISO 8601",
  "data": {
    "category": "string",
    "value": "any",
    "metadata": {}
  },
  "signature": "string (Ed25519)"
}
```

### 2.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| type | string | Yes | Record type identifier |
| version | string | Yes | Specification version |
| id | UUID | Yes | Unique record identifier |
| timestamp | ISO 8601 | Yes | Record creation time |
| data | object | Yes | Primary data payload |
| signature | string | No | Cryptographic signature |

## 3. Data Types

### 3.1 Core Types

| Type | Format | Example |
|------|--------|---------|
| String | UTF-8 | "Hello World" |
| Number | IEEE 754 | 3.14159 |
| Boolean | true/false | true |
| Date | ISO 8601 | "2025-01-01T00:00:00Z" |
| UUID | RFC 4122 | "550e8400-e29b-41d4-a716-446655440000" |

### 3.2 Extended Types

```typescript
interface ExtendedData {
  binaryData: ArrayBuffer;
  coordinates: GeoCoordinate;
  range: NumberRange;
  collection: DataCollection;
}

interface GeoCoordinate {
  latitude: number;   // -90 to 90
  longitude: number;  // -180 to 180
  altitude?: number;  // meters
}

interface NumberRange {
  min: number;
  max: number;
  step?: number;
}
```

## 4. Encoding Requirements

### 4.1 Character Encoding
- All text MUST be UTF-8 encoded
- No BOM (Byte Order Mark) allowed
- Line endings: LF (Unix style)

### 4.2 Binary Encoding
- Use Base64 for binary data in JSON
- Use Protocol Buffers for high-performance scenarios
- Big-endian byte order for network transmission

## 5. Validation Rules

### 5.1 Schema Validation
```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["type", "version", "id", "timestamp", "data"],
  "properties": {
    "type": { "type": "string", "const": "WIA-FINANCIAL_INCLUSIONRecord" },
    "version": { "type": "string", "pattern": "^\\d+\\.\\d+$" },
    "id": { "type": "string", "format": "uuid" },
    "timestamp": { "type": "string", "format": "date-time" }
  }
}
```

### 5.2 Validation Errors

| Code | Message | Resolution |
|------|---------|------------|
| E001 | Invalid type | Use correct record type |
| E002 | Missing required field | Add all required fields |
| E003 | Invalid format | Check field format specifications |
| E004 | Schema mismatch | Validate against JSON schema |

## 6. Versioning

### 6.1 Version Format
- Major.Minor (e.g., 1.0, 2.1)
- Major version: Breaking changes
- Minor version: Backward-compatible additions

### 6.2 Migration Path
- v1.0 → v1.1: Add optional fields
- v1.x → v2.0: Schema restructuring

## 7. Examples

### 7.1 Basic Record
```json
{
  "type": "WIA-FINANCIAL_INCLUSIONRecord",
  "version": "1.0",
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-01T00:00:00Z",
  "data": {
    "category": "standard",
    "value": "example data",
    "metadata": {
      "source": "system",
      "priority": "normal"
    }
  }
}
```

---

## 8. Reference Standards Alignment

The Phase 1 data formats are layered above well-established financial-services and IT primitives so that conformant implementations interoperate with the global financial system.

| Concern | Reference |
|---------|-----------|
| Financial-services messaging | ISO 20022 (Financial Services — Universal financial industry message scheme) |
| Merchant category codes | ISO 18245:2003 |
| Bank account identifiers | ISO 13616 (IBAN) |
| Currency codes | ISO 4217 |
| Card-originated transaction messages | ISO 8583 |
| Transaction-time encoding | ISO 8601:2019 |
| Risk-data aggregation | BCBS 239 (Basel Committee on Banking Supervision) |
| AML/CFT recommendations | FATF Recommendations (40+9, current revision) |
| Numeric encoding | IEEE 754-2019 |
| JSON | RFC 8259 |
| JSON Schema | JSON Schema Draft 2020-12 |
| UUID | RFC 9562 |
| Hash | FIPS 180-4 (SHA-2), FIPS 202 (SHA-3) |
| Digital signatures | RFC 8032 (EdDSA), NIST FIPS 186-5 (ECDSA) |
| Verifiable credentials | W3C VC Data Model 2.0 |
| Decentralised identity | W3C DID Core 1.0 |
| Privacy framework | ISO/IEC 29100:2011 |
| Information security | ISO/IEC 27001:2022 |
| Quality management | ISO 9001:2015 |
| Locale | BCP 47 (RFC 5646), Unicode CLDR |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

## 9. Conformance

A Phase 1 implementation is conformant when:

1. Records validate against the v1.0 JSON Schema in §5.1.
2. Currency amounts use ISO 4217 currency codes.
3. Bank account identifiers (where applicable) use ISO 13616 IBAN format.
4. Transaction timestamps follow ISO 8601:2019.
5. Where the deployment interoperates with traditional financial-services infrastructure, ISO 20022 message types are produced or consumed under the deployment's documented profile.
6. Hash and signature algorithms match §8 with explicit algorithm identifiers.

## 10. Implementation Appendix

### 10.1 Identity for the Underbanked

Financial inclusion requires identity solutions that work for populations without traditional government-issued credentials. The reference programme supports:

- **W3C Verifiable Credentials** issued by community organisations, civil-society institutions, or employers.
- **W3C DID Core** with did:web, did:key, and did:peer methods for self-sovereign identity.
- **Mobile-network identity** where mobile-network operators are recognised by the deploying jurisdiction.
- **Biometric templates** stored locally (never centrally) and verified at the edge per ISO/IEC 24745:2022 (biometric template protection).

### 10.2 Transaction Records for Micropayments

Micropayment records use the same JSON envelope as larger transactions but with optimised batching: up to 1,000 micropayments may be aggregated into a single signed batch record. Batch records preserve per-transaction integrity through Merkle-tree aggregation, allowing individual transactions to be selectively disclosed without revealing the rest of the batch.

### 10.3 Cross-Border Considerations

Cross-border financial-inclusion records adhere to the deploying jurisdictions' AML/CFT rules. The data format includes an optional `wia.compliance` block carrying regulatory tags so that downstream regulatory-reporting systems can produce the rule-required reports without re-deriving the underlying data classification.

### 10.4 Currency Handling

Currency codes follow ISO 4217. Amounts are represented in minor units (e.g., cents, satoshis, paisa) using arbitrary-precision integers to avoid floating-point representational errors. Each amount carries an explicit `decimal_places` field aligning with the ISO 4217 minor-unit count for the currency, so that display formatting is unambiguous.

For currencies not yet listed in ISO 4217 (community currencies, local mutual-credit systems), the deployment publishes a private namespace under `wia.private.currency.<scheme>.<code>` and documents the conversion rate to a recognised reference currency in a separately versioned conversion table.

### 10.5 Identity Records for the Underbanked

Identity records are stored as W3C Verifiable Credentials (VC Data Model 2.0) issued by recognised parties (community organisations, employers, mobile-network operators, civil-society institutions). The data format records the credential reference rather than the credential payload directly, preserving the holder's ability to selectively disclose attributes without revealing the full credential.

When biometric identifiers are used as a binding factor, the biometric template never leaves the holder's device. The reference programme follows ISO/IEC 24745:2022 (biometric template protection) so that even leaked templates are useless without the holder's device.

### 10.6 Record Linkage and Deduplication

Records can be linked across systems using the WIA-FINANCIAL_INCLUSION identifier as the canonical anchor. Records produced by different operating organisations that refer to the same underlying transaction reconcile through:

- A canonical hash over the transaction's parties, amount, currency, and timestamp.
- Idempotency keys carried in the originating system's metadata.
- Operational reconciliation procedures documented by each operating organisation.

The reconciliation procedures are part of the operating organisation's compliance documentation.

### 10.7 Localisation Considerations

Underbanked populations are frequently multilingual or low-literacy. The data format supports localised string fields with explicit BCP 47 (RFC 5646) language tags, multiple translations per string, and an explicit primary language indicator. Display formatting follows the Unicode CLDR locale data for the user's preferred locale, including bidi handling for right-to-left scripts (Unicode UAX #9).

### 10.8 Conformance Reporting

Operating organisations publish a conformance report listing the supported transaction categories, the supported currencies, the integrated regulatory frameworks, and the supported identity-credential issuers. The report is reviewed annually and is part of the deployment's compliance artefacts.

---

**弘益人間 (Benefit All Humanity)**



## 11. Glossary

- **Underbanked** — A population segment with limited access to traditional banking services; the precise definition follows the deploying jurisdiction's national-statistics office.
- **Mobile money** — A digital store of value held in an account that is accessed through a mobile phone; distinguished from a traditional bank account.
- **Remittance** — A transfer of money by a foreign worker to an individual in their home country.
- **Microfinance** — Financial services targeted at low-income individuals or those who lack access to traditional banking.
- **Mutual credit** — A community-based credit system where members extend short-term credit to each other.
- **Verifiable Credential (VC)** — A tamper-evident credential whose authorship can be cryptographically verified, per W3C VC Data Model 2.0.
- **DID (Decentralised Identifier)** — A globally unique identifier that resolves to a DID document, per W3C DID Core 1.0.

## 12. Conformance Statement Template

```yaml
wia_financial_inclusion_v1_conformance:
  spec_version: "1.0"
  operating_organisation:
    name: "<organisation name>"
    jurisdiction_iso3166: "<country code>"
  supported_currencies_iso4217:
    - USD
    - KRW
    - "<other ISO 4217 codes>"
  supported_transaction_categories:
    - remittance
    - micropayment
    - "<other categories>"
  identity_credential_issuers:
    - did:web:example.org
    - "<other recognised issuers>"
  reference_standards_compliance:
    iso_20022: true
    iso_4217: true
    iso_13616: true
    iso_27001: true
    fatf_recommendations: true
  conformance_evidence_uri: "https://example.org/wia-fi/conformance.html"
```

### Cross-Cutting Considerations

The Phase 1 data format anchors the rest of the standard. Operating organisations document any extensions to the canonical schema in their published conformance statement so that interoperability across deployments is preserved even when individual deployments add fields specific to their operating context. Extensions follow the `wia.private.<vendor>.<field>` naming convention to prevent collision with future canonical fields, and the deployment publishes the schema for each private field so that auditors and partners can interpret it consistently.

---

*© 2025 WIA - World Certification Industry Association*
*MIT License*
