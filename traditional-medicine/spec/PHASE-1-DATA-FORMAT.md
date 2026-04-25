# WIA-TRADITIONAL-MEDICINE: Phase 1 - Data Format

## 弘益人間 (Benefit All Humanity)

**Version:** 1.0.0
**Status:** Official
**Last Updated:** 2025-01

---

## 1. Overview

Phase 1 establishes standardized data formats for representing traditional medicine concepts in modern healthcare information systems. These formats enable interoperability between different traditional medicine systems while preserving their unique terminology and concepts.

### 1.1 Design Principles

- **Comprehensive**: Capture the full richness of traditional diagnostic information
- **Interoperable**: Compatible with HL7 FHIR and other healthcare standards
- **Multilingual**: Preserve original terminology alongside translations
- **Extensible**: Allow addition of new traditional medicine systems
- **Validated**: Include JSON Schema validation for data quality

---

## 2. Constitutional Profile Schema

The constitutional profile captures an individual's constitution assessment across multiple traditional medicine systems.

### 2.1 Schema Definition

```json
{
  "$schema": "https://wia.live/schemas/traditional-medicine/v1.0.0/constitutional-profile",
  "type": "object",
  "required": ["profile_id", "patient_id", "timestamp"],
  "properties": {
    "profile_id": {
      "type": "string",
      "format": "uuid",
      "description": "Unique identifier for this profile"
    },
    "patient_id": {
      "type": "string",
      "description": "Reference to patient record"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp of assessment"
    },
    "practitioner_id": {
      "type": "string",
      "description": "Reference to practitioner record"
    },
    "tcm_constitution": {
      "$ref": "#/definitions/TCMConstitution"
    },
    "ayurveda_prakriti": {
      "$ref": "#/definitions/AyurvedaPrakriti"
    },
    "sasang_constitution": {
      "$ref": "#/definitions/SasangConstitution"
    },
    "genomic_correlates": {
      "$ref": "#/definitions/GenomicCorrelates"
    },
    "metabolomic_profile": {
      "$ref": "#/definitions/MetabolomicProfile"
    },
    "microbiome_profile": {
      "$ref": "#/definitions/MicrobiomeProfile"
    }
  }
}
```

### 2.2 TCM Constitution Types

| Type | Chinese | Pinyin | Characteristics |
|------|---------|--------|-----------------|
| qi_deficiency | 氣虛質 | Qì xū zhì | Fatigue, weak voice, susceptibility to colds |
| yang_deficiency | 陽虛質 | Yáng xū zhì | Cold intolerance, cold extremities |
| yin_deficiency | 陰虛質 | Yīn xū zhì | Heat sensations, dry skin, night sweats |
| phlegm_dampness | 痰濕質 | Tán shī zhì | Overweight, greasy skin, heavy sensation |
| damp_heat | 濕熱質 | Shī rè zhì | Oily skin, irritability |
| blood_stasis | 血瘀質 | Xuě yū zhì | Dark complexion, pain tendency |
| qi_stagnation | 氣郁質 | Qì yù zhì | Emotional sensitivity, sighing |
| special_diathesis | 特稟質 | Tè bǐng zhì | Allergic tendencies |
| balanced | 平和質 | Píng hé zhì | Optimal health state |

### 2.3 Sasang Constitution Types

| Type | Korean | Hanja | Organ Tendency | Prevalence |
|------|--------|-------|----------------|------------|
| taeyang | 태양인 | 太陽人 | Strong Lung, Weak Liver | ~0.1% |
| taeeum | 태음인 | 太陰人 | Strong Liver, Weak Lung | ~50% |
| soyang | 소양인 | 少陽人 | Strong Spleen, Weak Kidney | ~25% |
| soeum | 소음인 | 少陰人 | Strong Kidney, Weak Spleen | ~25% |

---

## 3. Traditional Diagnosis Schema

### 3.1 Four Examinations Structure

```json
{
  "four_examinations": {
    "inspection": {
      "complexion": "string",
      "spirit": "string",
      "tongue": {
        "body_color": "enum[pale|light_red|red|dark_red|purple]",
        "body_shape": "enum[thin|normal|swollen|teeth_marked]",
        "coating_color": "enum[white|yellow|gray|black]",
        "coating_thickness": "enum[thin|normal|thick|peeled]",
        "moisture": "enum[dry|normal|wet]",
        "image_url": "string",
        "ai_analysis": {
          "model": "string",
          "confidence": "number",
          "findings": "array[string]"
        }
      }
    },
    "auscultation_olfaction": {
      "voice": { "quality": "string", "volume": "string" },
      "breathing": { "pattern": "string" },
      "body_odor": "string"
    },
    "inquiry": {
      "chief_complaint": "string",
      "onset": "string",
      "ten_questions": {
        "cold_heat": "string",
        "perspiration": "string",
        "appetite_thirst": "string",
        "urination_defecation": "string",
        "sleep": "string",
        "pain": "string",
        "menstruation": "string",
        "emotions": "string"
      }
    },
    "palpation": {
      "pulse": {
        "rate": "number",
        "rhythm": "enum[regular|irregular]",
        "quality": "array[string]",
        "positions": {
          "left_cun": { "quality": "string" },
          "left_guan": { "quality": "string" },
          "left_chi": { "quality": "string" },
          "right_cun": { "quality": "string" },
          "right_guan": { "quality": "string" },
          "right_chi": { "quality": "string" }
        }
      }
    }
  }
}
```

### 3.2 Pulse Qualities (28 Classical Types)

| Category | Qualities |
|----------|-----------|
| Depth | Floating (浮), Deep (沉) |
| Speed | Slow (遲), Rapid (數) |
| Strength | Forceless (虛), Forceful (實) |
| Width | Thin (細), Big (大) |
| Length | Short (短), Long (長) |
| Rhythm | Intermittent (結代), Irregular (促) |
| Quality | Slippery (滑), Wiry (弦), Choppy (澀) |

---

## 4. Herbal Medicine Database Schema

### 4.1 Herb Entity Structure

```json
{
  "herbal_medicine": {
    "herb_id": "string",
    "names": {
      "scientific": "string",
      "chinese": "string",
      "pinyin": "string",
      "korean": "string",
      "japanese": "string",
      "sanskrit": "string",
      "common_en": "string"
    },
    "taxonomy": {
      "kingdom": "string",
      "family": "string",
      "genus": "string",
      "species": "string",
      "dna_barcode": "string"
    },
    "traditional_properties": {
      "taste": "array[enum]",
      "temperature": "enum[hot|warm|neutral|cool|cold]",
      "meridians": "array[string]",
      "actions": "array[string]",
      "indications": "array[string]",
      "contraindications": "array[string]"
    },
    "modern_pharmacology": {
      "active_compounds": "array[CompoundObject]",
      "network_pharmacology": {
        "targets": "array[string]",
        "pathways": "array[string]",
        "diseases": "array[string]"
      }
    },
    "safety": {
      "toxicity_class": "enum[non_toxic|low|moderate|high]",
      "max_daily_dose_g": "number",
      "pregnancy_category": "enum[A|B|C|D|X]",
      "drug_interactions": "array[InteractionObject]",
      "adverse_reactions": "array[string]"
    },
    "quality_standards": {
      "pharmacopoeia": "array[string]",
      "marker_compounds": "array[string]",
      "authentication_method": "array[string]"
    }
  }
}
```

### 4.2 Taste Classifications

| Taste | Chinese | Effect |
|-------|---------|--------|
| Sweet | 甘 | Tonifying, harmonizing |
| Bitter | 苦 | Clearing heat, drying dampness |
| Sour | 酸 | Astringent, consolidating |
| Pungent | 辛 | Dispersing, moving qi |
| Salty | 鹹 | Softening, descending |
| Bland | 淡 | Draining dampness |
| Astringent | 澀 | Consolidating, stopping leakage |

---

## 5. ICD-11 Traditional Medicine Codes

### 5.1 Pattern Codes (TM1:S)

| Code | Pattern (English) | Pattern (Chinese) |
|------|-------------------|-------------------|
| TM1:SA00 | Yin deficiency pattern | 陰虛證 |
| TM1:SB00 | Yang deficiency pattern | 陽虛證 |
| TM1:SC00 | Qi deficiency pattern | 氣虛證 |
| TM1:SD00 | Blood deficiency pattern | 血虛證 |
| TM1:SE00 | Qi stagnation pattern | 氣滯證 |
| TM1:SF00 | Blood stasis pattern | 血瘀證 |
| TM1:SG00 | Phlegm pattern | 痰證 |
| TM1:SH00 | Heat pattern | 熱證 |
| TM1:SI00 | Cold pattern | 寒證 |
| TM1:SJ00 | Dampness pattern | 濕證 |

---

## 6. Validation and Quality

### 6.1 Schema Validation

All data must pass JSON Schema validation before submission:

```bash
npm install @wia/tm-validator
wia-tm-validate constitution.json
```

### 6.2 Required Fields

| Data Type | Required Fields |
|-----------|-----------------|
| Constitutional Profile | profile_id, patient_id, timestamp |
| Traditional Diagnosis | diagnosis_id, patient_id, pattern_diagnosis |
| Herbal Medicine | herb_id, names.scientific, traditional_properties |

---

## References

- WHO ICD-11 Traditional Medicine Module
- ISO/TC 249 Traditional Chinese Medicine Standards
- Constitution in Chinese Medicine Questionnaire (CCMQ)
- Questionnaire for Sasang Constitution Classification II (QSCCII)

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association

## P.1 Data Format Cross-References

This Phase defines the canonical data types referenced by the API surface (Phase 2),
the wire protocol (Phase 3), and integration scenarios (Phase 4). Implementations
MUST round-trip every canonical type through serialization and deserialization
without loss of precision or semantics.

### P.1.1 Canonical Encoding Rules

1. UTF-8 is the required character encoding for textual fields.
2. Numeric fields use IEEE 754 binary64 unless explicitly marked as fixed-point.
3. Timestamps use RFC 3339 with timezone offset; durations use ISO 8601.
4. UUIDs follow RFC 4122 v4 unless deterministic IDs are required.
5. Binary payloads are encoded as Base64 (RFC 4648 §4) in JSON contexts and as
   raw octet strings in Protocol Buffers / CBOR contexts.

### P.1.2 Schema Evolution

Schema changes follow these compatibility classes:

| Class | Allowed Changes | Wire-Compat |
|-------|-----------------|-------------|
| Patch | Doc fixes, examples, validator tightening within existing range | Forward & backward |
| Minor | New optional fields, new enum values with default fallback        | Forward |
| Major | Field rename, type change, removal, semantics change              | None |

### P.1.3 Validation Order

Validators MUST apply checks in this order: (1) syntactic well-formedness,
(2) schema conformance, (3) cross-field invariants, (4) external referential
integrity, (5) policy / authorization. A failure short-circuits subsequent
checks; the response message identifies the first failing rule by ID.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of traditional-medicine so that conformance claims at any
Phase remain unambiguous.*

