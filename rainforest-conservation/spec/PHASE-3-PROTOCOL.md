# WIA Rainforest Conservation Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red)

---

## Table of Contents

1. [Overview](#overview)
2. [Communication Protocols](#communication-protocols)
3. [Data Exchange Protocols](#data-exchange-protocols)
4. [Alert Protocols](#alert-protocols)
5. [Verification Protocols](#verification-protocols)
6. [Security Protocols](#security-protocols)
7. [Compliance](#compliance)

---

## Overview

### 1.1 Purpose

Define communication protocols for rainforest monitoring systems, ensuring interoperability between satellite systems, ground sensors, conservation NGOs, indigenous communities, and government agencies.

### 1.2 Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer (REST APIs)    │
├─────────────────────────────────────┤
│   Transport Layer (HTTPS/WSS)      │
├─────────────────────────────────────┤
│   Security Layer (TLS 1.3)         │
├─────────────────────────────────────┤
│   Network Layer (IPv4/IPv6)        │
└─────────────────────────────────────┘
```

---

## Communication Protocols

### 2.1 Real-time Monitoring Protocol

**WebSocket Connection**:
```
wss://ws.wia.rainforest.org/v1/monitor
```

**Connection Handshake**:
```json
{
  "type": "subscribe",
  "forestId": "FOREST-2025-AMZ-001",
  "metrics": ["deforestation", "fire", "ndvi"],
  "interval": 3600
}
```

**Server Response**:
```json
{
  "type": "subscribed",
  "forestId": "FOREST-2025-AMZ-001",
  "sessionId": "WS-2025-001"
}
```

**Real-time Updates**:
```json
{
  "type": "update",
  "forestId": "FOREST-2025-AMZ-001",
  "metric": "deforestation",
  "value": 12.5,
  "timestamp": "2025-01-15T12:00:00Z",
  "severity": "warning"
}
```

### 2.2 Batch Data Protocol

**Batch Upload Endpoint**: `POST /batch/upload`

**Request Format**:
```json
{
  "batchId": "BATCH-2025-001",
  "format": "geojson",
  "compression": "gzip",
  "records": [
    { ... },
    { ... }
  ]
}
```

---

## Data Exchange Protocols

### 3.1 Satellite Data Protocol

**Satellite Provider Integration**:

```json
{
  "protocol": "WIA-SATELLITE-v1",
  "provider": "sentinel-2",
  "dataType": "multispectral",
  "coverage": {
    "type": "Polygon",
    "coordinates": [[...]]
  },
  "resolution": "10m",
  "bands": ["B4", "B8", "B11"],
  "cloudCover": 5,
  "acquisitionDate": "2025-01-15T10:30:00Z"
}
```

**NDVI Calculation Protocol**:
```
NDVI = (NIR - Red) / (NIR + Red)
where:
  NIR = Band 8 (Near-Infrared)
  Red = Band 4 (Red)
```

### 3.2 Ground Sensor Protocol

**IoT Sensor Data**:
```json
{
  "protocol": "WIA-SENSOR-v1",
  "sensorId": "SENSOR-2025-001",
  "type": "temperature-humidity",
  "location": {
    "type": "Point",
    "coordinates": [-60.123, -3.456]
  },
  "readings": {
    "temperature": 26.5,
    "humidity": 85,
    "soilMoisture": 72
  },
  "timestamp": "2025-01-15T12:00:00Z"
}
```

**MQTT Topic Structure**:
```
wia/rainforest/{forestId}/sensor/{sensorId}/{metric}

Examples:
wia/rainforest/FOREST-2025-AMZ-001/sensor/SENSOR-001/temperature
wia/rainforest/FOREST-2025-AMZ-001/sensor/SENSOR-001/humidity
```

### 3.3 Biodiversity Data Protocol

**Species Observation Protocol**:
```json
{
  "protocol": "WIA-BIODIVERSITY-v1",
  "observationId": "OBS-2025-001",
  "species": {
    "scientificName": "Panthera onca",
    "commonName": "jaguar",
    "iucnStatus": "NT"
  },
  "location": {
    "type": "Point",
    "coordinates": [-60.123, -3.456]
  },
  "method": "camera-trap",
  "confidence": 0.98,
  "observedBy": "researcher-id-123",
  "timestamp": "2025-01-15T08:30:00Z"
}
```

---

## Alert Protocols

### 4.1 Deforestation Alert Protocol

**Alert Severity Levels**:
- `info` - Minor changes detected
- `warning` - Moderate deforestation (100-500 ha)
- `critical` - Severe deforestation (>500 ha)

**Alert Message Format**:
```json
{
  "protocol": "WIA-ALERT-v1",
  "alertId": "ALERT-2025-001",
  "type": "deforestation",
  "severity": "critical",
  "forestId": "FOREST-2025-AMZ-001",
  "area": 750,
  "location": {
    "type": "Polygon",
    "coordinates": [[...]]
  },
  "detectionMethod": "satellite-sentinel-2",
  "confidence": 0.96,
  "causes": ["logging", "agriculture"],
  "timestamp": "2025-01-15T12:00:00Z",
  "recipients": [
    "forest-authority@example.org",
    "conservation-ngo@example.org",
    "indigenous-community@example.org"
  ]
}
```

**Alert Escalation Protocol**:
```
1. Auto-detect deforestation via satellite
2. Verify with secondary data sources
3. Calculate severity level
4. Notify primary stakeholders (< 15 min)
5. Escalate to authorities if critical
6. Track response and resolution
```

### 4.2 Fire Detection Protocol

**Fire Alert**:
```json
{
  "protocol": "WIA-FIRE-v1",
  "alertId": "FIRE-2025-001",
  "forestId": "FOREST-2025-AMZ-001",
  "location": {
    "type": "Point",
    "coordinates": [-60.234, -3.567]
  },
  "hotspots": 15,
  "area": 125,
  "intensity": "high",
  "windSpeed": 12,
  "windDirection": 45,
  "timestamp": "2025-01-15T14:20:00Z"
}
```

---

## Verification Protocols

### 5.1 Carbon Credit Verification Protocol

**MRV (Monitoring, Reporting, Verification)**:

```json
{
  "protocol": "WIA-CARBON-MRV-v1",
  "projectId": "REDD-2025-001",
  "baseline": {
    "period": "2020-2024",
    "deforestationRate": 500,
    "carbonEmissions": 125000
  },
  "monitoring": {
    "period": "2024-2025",
    "deforestationRate": 150,
    "carbonEmissions": 37500,
    "reduction": 70
  },
  "verification": {
    "method": "satellite-mRV",
    "verifier": "verra-vcs",
    "confidence": 0.95,
    "verified": true
  },
  "credits": {
    "issued": 87500,
    "registry": "verra-vcs",
    "serialNumber": "VCS-2025-001"
  }
}
```

### 5.2 Indigenous Rights Verification

**Free, Prior, and Informed Consent (FPIC)**:
```json
{
  "protocol": "WIA-FPIC-v1",
  "consentId": "FPIC-2025-001",
  "project": "Conservation Area Expansion",
  "community": "Yanomami People",
  "territory": "INDIG-2025-001",
  "process": {
    "informationProvided": "2024-12-01",
    "consultationPeriod": 90,
    "decisionDate": "2025-03-01",
    "outcome": "approved-with-conditions"
  },
  "conditions": [
    "50% revenue sharing",
    "Community-led monitoring",
    "Traditional practices protected"
  ],
  "signatures": {
    "communityLeader": "sig-1",
    "projectManager": "sig-2",
    "witness": "sig-3"
  }
}
```

---

## Security Protocols

### 6.1 Authentication Protocol

**OAuth 2.0 + DID (Decentralized Identifiers)**:

```json
{
  "did": "did:wia:rainforest:org:conservation-ngo-123",
  "publicKey": {
    "id": "did:wia:rainforest:org:conservation-ngo-123#key-1",
    "type": "Ed25519VerificationKey2020",
    "controller": "did:wia:rainforest:org:conservation-ngo-123",
    "publicKeyMultibase": "z6Mk..."
  }
}
```

### 6.2 Data Encryption Protocol

**In-Transit Encryption**:
- TLS 1.3 for all HTTPS communications
- WSS (WebSocket Secure) for real-time streams

**At-Rest Encryption**:
- AES-256-GCM for database storage
- Field-level encryption for sensitive data

**Key Management**:
```json
{
  "keyId": "KEY-2025-001",
  "algorithm": "AES-256-GCM",
  "rotation": "90-days",
  "kms": "aws-kms"
}
```

### 6.3 Data Integrity Protocol

**Hash-based Verification**:
```json
{
  "dataId": "FOREST-2025-AMZ-001",
  "hash": "sha256:a3b2c1d4e5f6...",
  "timestamp": "2025-01-15T12:00:00Z",
  "signature": {
    "algorithm": "Ed25519",
    "value": "sig:9a8b7c6d..."
  }
}
```

---

## Compliance

### 7.1 REDD+ Compliance

**Requirements**:
- Baseline establishment
- Additionality proof
- Leakage assessment
- Permanence guarantee
- Co-benefits (biodiversity, social)

### 7.2 CBD (Convention on Biological Diversity)

**Nagoya Protocol Compliance**:
```json
{
  "protocol": "WIA-ABS-v1",
  "resourceAccess": {
    "geneticResource": "tree-species-samples",
    "location": "FOREST-2025-AMZ-001",
    "traditionalKnowledge": true
  },
  "benefitSharing": {
    "monetary": "50% royalties",
    "nonMonetary": ["capacity building", "technology transfer"],
    "community": "Yanomami People"
  }
}
```

### 7.3 GDPR & Data Privacy

**Personal Data Protection**:
```json
{
  "dataType": "indigenous-community-info",
  "classification": "sensitive",
  "consent": {
    "obtained": true,
    "purpose": "conservation-monitoring",
    "retention": "5-years"
  },
  "rights": {
    "access": true,
    "rectification": true,
    "erasure": true,
    "portability": true
  }
}
```

---

## Protocol Evolution

### 8.1 Versioning

Protocol versions follow semantic versioning:
- `WIA-PROTOCOL-v{major}.{minor}.{patch}`

### 8.2 Deprecation Policy

- New version announced: 6 months notice
- Parallel support: 12 months
- Old version sunset: After 12 months

## Appendix — Indigenous Community Custodian Verification

The community custodian DID is the legal authority that signs consent
envelopes governing data flow within community-claimed territory. The
custodian is typically a designated community council, traditional
leader, or recognised representative entity rather than an individual.

WIA-OMNI-API issues `community_custodian_link` claims that bind a
custodian DID to a community-claim DID with documented authority
sources (traditional law recognition by national government, registered
indigenous organisation membership, etc.). Hosts MUST verify the
custodian_link claim before accepting a consent envelope, and MUST
refuse consent envelopes signed by an expired or revoked custodian.

Custodian succession (when a community changes its representative
council) follows a chain-of-trust pattern: the prior custodian signs
a transition envelope endorsing the new custodian, OR the community's
authority source (traditional law body or registered organisation)
signs a fresh custodian_link claim for the new custodian. Either path
preserves continuity of the consent envelopes; previously-issued
consent envelopes remain valid until their declared `valid_until`.

## Appendix — Replay Cache Sizing

For a typical satellite vendor cloud receiving 100 deforestation alert
envelopes per second across all customers (peak during dry-season
fire activity), the seen-nonce cache must hold roughly
`100 × 600 = 60 000` entries. With 16-byte nonce keys plus a 4-byte
timestamp, the strict cache footprint is approximately
`60 000 × 24 ≈ 1.5 MiB`. Hosts SHOULD provision at least double this
to absorb fire-season bursts. Persistent caches across restart are
RECOMMENDED for hosts that page on critical alerts; volatile caches
risk re-emitting alerts after a failover and triggering duplicate
clinician/policy-maker pages.

---

**© 2025 WIA (World Certification Industry Association)**
**License**: MIT
**Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity
