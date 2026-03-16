# WIA-AI-017 Specification: PHASE 1
## Core Authentication Framework

**Version:** 1.0
**Status:** Published
**Date:** 2025-12-25
**Philosophy:** 弘益人間 (홍익인간) · Benefit All Humanity

---

## 1. Introduction

This specification defines Phase 1 of the WIA-AI-017 AI Content Authentication standard, establishing the core framework for verifying and authenticating AI-generated or AI-modified digital content.

### 1.1 Scope

Phase 1 covers:
- Digital signature requirements
- Content fingerprinting methods
- Basic watermarking capabilities
- Provenance metadata structure
- Authentication APIs

### 1.2 Normative References

- RFC 5652: Cryptographic Message Syntax (CMS)
- RFC 3161: Time-Stamp Protocol (TSP)
- RFC 8032: Edwards-Curve Digital Signature Algorithm (EdDSA)
- C2PA Technical Specification v1.3
- ISO/IEC 19794: Biometric Data Interchange Formats

---

## 2. Digital Signatures

### 2.1 Supported Algorithms

Implementations MUST support at least one of the following signature algorithms:

| Algorithm | Key Size | Security Level | Status |
|-----------|----------|----------------|--------|
| Ed25519 | 256 bits | 128-bit | **RECOMMENDED** |
| ECDSA P-256 | 256 bits | 128-bit | REQUIRED |
| ECDSA P-384 | 384 bits | 192-bit | OPTIONAL |
| RSA-PSS 3072 | 3072 bits | 128-bit | OPTIONAL |

### 2.2 Signature Format

Signatures MUST use one of:
- **CMS SignedData** (RFC 5652) - RECOMMENDED
- **JSON Web Signature (JWS)** (RFC 7515) - OPTIONAL

Example CMS structure:
```
ContentInfo ::= SEQUENCE {
    contentType ContentType,
    content [0] EXPLICIT ANY DEFINED BY contentType
}

SignedData ::= SEQUENCE {
    version CMSVersion,
    digestAlgorithms DigestAlgorithmIdentifiers,
    encapContentInfo EncapsulatedContentInfo,
    certificates [0] IMPLICIT CertificateSet OPTIONAL,
    signerInfos SignerInfos
}
```

### 2.3 Hash Functions

Implementations MUST support:
- **SHA-256** (REQUIRED)
- SHA-3-256 (RECOMMENDED)
- SHA-512 (OPTIONAL)

MD5 and SHA-1 MUST NOT be used.

### 2.4 Timestamping

All signatures SHOULD include an RFC 3161 timestamp token from a trusted Time Stamping Authority (TSA).

---

## 3. Content Fingerprinting

### 3.1 Image Fingerprinting

Implementations MUST support at least one perceptual hash algorithm:

**pHash (Perceptual Hash)**
- Algorithm: DCT-based perceptual hashing
- Output: 64-bit binary hash
- Use: Primary image fingerprinting

**dHash (Difference Hash)**
- Algorithm: Gradient-based hashing
- Output: 64-bit binary hash
- Use: Fast similarity detection

### 3.2 Similarity Threshold

For image matching, the following Hamming distance thresholds SHOULD be used:

- **Identical/Near-identical:** ≤ 5 bits
- **Very similar:** 6-10 bits
- **Similar:** 11-20 bits
- **Different:** > 20 bits

### 3.3 Video Fingerprinting

Video fingerprinting MUST:
1. Extract keyframes at regular intervals (recommended: 1 frame per second)
2. Generate perceptual hash for each keyframe
3. Compute temporal hash from frame sequence

### 3.4 Audio Fingerprinting

Audio fingerprinting SHOULD use:
- **Chromaprint** (RECOMMENDED)
- Acoustic fingerprinting based on spectral features

---

## 4. Watermarking

### 4.1 Requirements

Digital watermarks MUST be:
- **Imperceptible:** PSNR > 40dB for images
- **Robust:** Survive JPEG compression (quality ≥ 75)
- **Blind:** Extractable without original content
- **Capacity:** Minimum 64 bits of payload

### 4.2 Supported Techniques

**For Images:**
- DCT-based watermarking (RECOMMENDED)
- DWT-based watermarking (OPTIONAL)
- Spread spectrum watermarking (OPTIONAL)

**For Video:**
- Frame-based DCT watermarking (RECOMMENDED)
- 3D DWT watermarking (OPTIONAL)

**For Audio:**
- Echo hiding (RECOMMENDED)
- Spread spectrum (OPTIONAL)

### 4.3 Watermark Payload Format

```
WatermarkPayload ::= SEQUENCE {
    version INTEGER (1),
    contentId UTF8String,
    creator UTF8String OPTIONAL,
    timestamp GeneralizedTime,
    additionalData OCTET STRING OPTIONAL
}
```

---

## 5. Provenance Metadata

### 5.1 C2PA Integration

All authenticated content SHOULD include a C2PA manifest containing:

```json
{
    "claim_generator": "WIA-AI-017/1.0",
    "title": "Content Title",
    "format": "image/jpeg",
    "instance_id": "urn:uuid:...",
    "assertions": [
        {
            "label": "c2pa.actions",
            "data": { ... }
        },
        {
            "label": "c2pa.hash.data",
            "data": {
                "hash": "...",
                "alg": "sha256"
            }
        }
    ]
}
```

### 5.2 Required Assertions

The following C2PA assertions MUST be included:
- `c2pa.actions` - Creation and modification history
- `c2pa.hash.data` - Content hash
- `stds.schema-org.CreativeWork` - Creator information

### 5.3 AI-Specific Assertions

When content involves AI:
- `c2pa.ai_generative_training` - AI model information (REQUIRED)
- `c2pa.ai_inference` - AI processing details (OPTIONAL)

---

## 6. Authentication API

### 6.1 Sign Content

**Endpoint:** `POST /api/v1/authenticate`

Request:
```json
{
    "content": "base64EncodedContent",
    "metadata": {
        "title": "string",
        "creator": "string",
        "ai_model": "string",
        "timestamp": "ISO8601"
    },
    "options": {
        "embed_watermark": true,
        "include_fingerprint": true
    }
}
```

Response:
```json
{
    "authenticated_content": "base64EncodedAuthenticatedContent",
    "manifest": { ... },
    "signature": "base64Signature",
    "fingerprints": [
        {
            "algorithm": "pHash",
            "hash": "hex..."
        }
    ]
}
```

### 6.2 Verify Content

**Endpoint:** `POST /api/v1/verify`

Request:
```json
{
    "content": "base64EncodedContent"
}
```

Response:
```json
{
    "authentic": true,
    "signature_valid": true,
    "watermark_detected": true,
    "manifest": { ... },
    "confidence": 0.98
}
```

---

## 7. Security Considerations

### 7.1 Key Management

- Private signing keys MUST be stored in Hardware Security Modules (HSM)
- Key rotation SHOULD occur annually
- Certificate validity MUST NOT exceed 3 years

### 7.2 Attack Resistance

Implementations MUST be resilient to:
- Signature forgery attempts
- Watermark removal attacks
- Metadata stripping
- Replay attacks (via timestamp validation)

### 7.3 Privacy

- Personal data in provenance SHOULD be redactable
- Zero-knowledge proofs MAY be used for selective disclosure

---

## 8. Conformance

An implementation conforming to WIA-AI-017 Phase 1 MUST:
1. Support Ed25519 or ECDSA P-256 signatures
2. Support SHA-256 content hashing
3. Implement at least one perceptual fingerprinting algorithm
4. Support C2PA manifest embedding and extraction
5. Provide the authentication and verification APIs

---

## 9. Test Vectors

[Test vectors available at: https://github.com/WIA-Official/wia-standards/tree/main/test-vectors]

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
