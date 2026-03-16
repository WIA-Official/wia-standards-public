# WIA-SOC-013 PHASE 1: DATA FORMAT SPECIFICATION

**Public Document Standard - Data Formats**

Version: 1.0
Date: 2025-01-15
Status: Final

---

## 1. Overview

This specification defines data formats for public document digitization, including image formats, OCR output, metadata schemas, and archival formats compliant with international standards.

## 2. Document Structure

### 2.1 JSON Document Schema

```json
{
  "documentId": "doc:wia:soc013:{type}:{country}:{year}:{uniqueId}",
  "version": "1.0",
  "type": "birthCertificate | passport | nationalID | driverLicense | diploma | deed",
  "issuer": "did:gov:{country}:{agency}",
  "issuedAt": "ISO 8601 timestamp",
  "expiresAt": "ISO 8601 timestamp (optional)",
  "subject": {
    "name": "string",
    "nameLocal": "string (native script)",
    "nameRomanized": "string",
    "dateOfBirth": "YYYY-MM-DD",
    "placeOfBirth": "string",
    "nationality": "ISO 3166-1 alpha-3"
  },
  "images": [{
    "pageNumber": "integer",
    "format": "image/jpeg | image/png | image/tiff",
    "resolution": "integer (DPI)",
    "url": "string",
    "sha256": "string"
  }],
  "metadata": {},
  "signatures": [],
  "blockchainAnchors": []
}
```

### 2.2 XML Document Schema

```xml
<document xmlns="https://wiastandards.org/soc-013/v1"
          documentId="doc:wia:soc013:birth:US:2025:abc123"
          type="birthCertificate"
          version="1.0">
  <issuer>did:gov:us:california:registry</issuer>
  <issuedAt>2025-01-15T10:30:00Z</issuedAt>
  <subject>
    <name>John Alexander Doe</name>
    <dateOfBirth>1990-05-20</dateOfBirth>
    <placeOfBirth>San Francisco, CA, USA</placeOfBirth>
  </subject>
  <images>
    <image pageNumber="1" format="image/jpeg" resolution="600" 
           url="..." sha256="..."/>
  </images>
</document>
```

## 3. Image Formats

### 3.1 Master Archive Format: TIFF

- **Compression**: LZW (lossless) or Uncompressed
- **Color Depth**: 8-bit grayscale or 24-bit RGB
- **Resolution**: 600-1200 DPI minimum
- **Photometric**: Min-is-White for grayscale
- **Byte Order**: Little Endian (Intel)

### 3.2 Distribution Format: PDF/A

- **Version**: PDF/A-2b or PDF/A-3b (ISO 19005-2/3)
- **Conformance**: Level B minimum, Level A for accessibility
- **Compression**: JPEG2000 for images (quality 85)
- **Fonts**: All fonts embedded
- **Color Space**: sRGB

### 3.3 Derivative Formats

- **JPEG**: 150-300 DPI, quality 85, for web delivery
- **PNG**: 300 DPI, lossless, for transparency support
- **WebP**: 150-300 DPI, for modern web browsers

## 4. OCR Output Format

### 4.1 hOCR (HTML-based OCR)

```xml
<span class="ocr_line" title="bbox 120 80 520 140; baseline 0.015 -12; x_size 48; x_descenders 12; x_ascenders 36">
  <span class="ocrx_word" title="bbox 120 80 280 140; x_wconf 98">United</span>
  <span class="ocrx_word" title="bbox 290 80 420 140; x_wconf 99">States</span>
  <span class="ocrx_word" title="bbox 430 80 520 140; x_wconf 97">Passport</span>
</span>
```

### 4.2 ALTO XML (Analyzed Layout and Text Object)

```xml
<TextBlock ID="TB1" HPOS="120" VPOS="80" WIDTH="400" HEIGHT="60">
  <TextLine ID="TL1" HPOS="120" VPOS="80" WIDTH="400" HEIGHT="60">
    <String CONTENT="United States Passport" WC="0.98"/>
  </TextLine>
</TextBlock>
```

## 5. Metadata Schemas

### 5.1 Dublin Core

Required elements: `dc:title`, `dc:creator`, `dc:date`, `dc:identifier`, `dc:format`, `dc:language`

Optional elements: `dc:subject`, `dc:description`, `dc:publisher`, `dc:rights`, `dc:coverage`

### 5.2 PREMIS

```json
{
  "premis": {
    "objectIdentifier": "doc:wia:soc013:abc123",
    "preservationLevel": "full",
    "significantProperties": ["authenticity", "integrity"],
    "environment": {
      "software": "WIA-SOC-013 Scanner v1.0",
      "hardware": "Fujitsu fi-7600 Scanner"
    },
    "events": [],
    "agents": [],
    "rights": {}
  }
}
```

## 6. Cryptographic Formats

### 6.1 Digital Signature

```json
{
  "signature": {
    "algorithm": "ECDSA-SHA256 | RSA-SHA256 | EdDSA-Ed25519",
    "value": "base64-encoded signature",
    "publicKey": "PEM-encoded public key",
    "certificateChain": ["base64-cert-1", "base64-cert-2", "base64-root"],
    "timestamp": "RFC 3161 timestamp token"
  }
}
```

### 6.2 Blockchain Anchor

```json
{
  "blockchainAnchor": {
    "network": "ethereum-mainnet | polygon | solana",
    "transactionHash": "0x...",
    "blockNumber": 12345678,
    "timestamp": "2025-01-15T10:30:00Z",
    "documentHash": "sha256 hash",
    "smartContract": "0x..."
  }
}
```

## 7. Quality Metrics

| Metric | Standard | Measurement |
|--------|----------|-------------|
| Image Resolution | ≥ 300 DPI | EXIF metadata |
| OCR Confidence | ≥ 95% | Per-word confidence scores |
| Metadata Completeness | 100% required fields | Field validation |
| Color Accuracy | ΔE < 3 | CIE Lab color difference |

## 8. Validation Rules

1. Document ID must follow pattern: `doc:wia:soc013:{type}:{country}:{year}:{uid}`
2. Timestamps must be ISO 8601 format in UTC
3. Language codes must be ISO 639-1 or ISO 639-2
4. Country codes must be ISO 3166-1 alpha-2 or alpha-3
5. SHA-256 hashes must be lowercase hexadecimal

## 9. Extensibility

Custom metadata may be added under `extensions` namespace:

```json
{
  "extensions": {
    "custom:securityLevel": "confidential",
    "custom:retentionPeriod": "99 years"
  }
}
```

## 10. Compliance

This specification complies with:
- ISO 15489 (Records Management)
- ISO 19005 (PDF/A)
- ISO 15836 (Dublin Core)
- W3C Verifiable Credentials
- ICAO 9303 (Machine Readable Travel Documents)

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
