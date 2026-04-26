# WIA Fire Safety System Standard
## PHASE 1 Specification v1.0

**Status:** Official Release  
**Date:** 2025-12-27  
**Organization:** World Certification Industry Association (WIA)  
**Philosophy:** 弘益人間 (홍익인간) · Benefit All Humanity

---

## Executive Summary

This document specifies Phase 1 of the WIA Fire Safety System Standard, providing comprehensive technical requirements for data format standardization in modern fire safety infrastructure. The specification enables interoperability between products from different manufacturers while supporting innovation and competition.

**Key Objectives:**
- Establish standardized data schemas
- Enable seamless multi-vendor system deployment
- Reduce total cost of ownership through competition
- Improve system reliability and maintainability
- Support advanced features like analytics and predictive maintenance

**Scope:** This specification applies to all fire safety system components including fire alarm control panels, detection devices, notification appliances, suppression systems, and integration interfaces.

---

## 1. Introduction

### 1.1 Purpose

Phase 1 of the WIA Fire Safety System Standard addresses the fundamental data representation requirements that enable all other aspects of interoperability. Without standardized data formats, systems cannot exchange information effectively regardless of protocol or API design.

### 1.2 Normative References

This specification builds upon and references the following standards:

- **NFPA 72:** National Fire Alarm and Signaling Code (2022 Edition)
- **EN 54 Series:** Fire Detection and Fire Alarm Systems
- **ISO/IEC 27001:** Information Security Management
- **RFC 8259:** JSON Data Interchange Format
- **RFC 7540:** HTTP/2 Protocol
- **RFC 6455:** WebSocket Protocol
- **TLS 1.3:** Transport Layer Security
- **IEEE 802.3:** Ethernet Standards

### 1.3 Terminology

**SHALL/MUST:** Indicates mandatory requirement for conformance  
**SHOULD:** Indicates strong recommendation  
**MAY:** Indicates permission or optional feature  
**Control Panel:** Fire alarm control and indicating equipment (FACP)  
**Addressable Device:** Device with unique network address  
**Notification Appliance:** Devices that alert occupants (horns, strobes, speakers)  
**Initiating Device:** Devices that detect fire (smoke detectors, heat detectors, pull stations)

---

## 2. Architecture Overview

### 2.1 Layered Architecture

The WIA standard employs a layered architecture separating concerns:

```
┌─────────────────────────────────────────┐
│   Application Layer                     │  User interfaces, analytics
├─────────────────────────────────────────┤
│   API Layer                             │  RESTful + WebSocket APIs
├─────────────────────────────────────────┤
│   Protocol Layer                        │  Fire safety protocols
├─────────────────────────────────────────┤
│   Data Layer                            │  Standardized data formats
├─────────────────────────────────────────┤
│   Transport Layer                       │  TCP/IP, UDP, TLS
├─────────────────────────────────────────┤
│   Physical Layer                        │  Ethernet, WiFi, RS-485
└─────────────────────────────────────────┘
```

Phase 1 focuses on the Data Layer specifications.

### 2.2 Design Principles

1. **Simplicity:** Prefer simple proven approaches over complex novel ones
2. **Security:** Security by design, mandatory encryption, strong authentication
3. **Reliability:** Deterministic behavior, comprehensive error handling
4. **Interoperability:** Work correctly with any compliant implementation
5. **Performance:** Meet latency and throughput requirements for life safety
6. **Maintainability:** Clear specifications, comprehensive documentation
7. **Extensibility:** Support future enhancements while maintaining compatibility

---

## 3. Technical Specifications

### 3.1 Data Format Requirements

#### 3.1.1 Sensor Data Schema

All sensor data SHALL be represented in JSON format conforming to the following schema:

```json
{
  "sensorId": "string (UUID v4)",
  "type": "string (smoke|heat|flame|co|multi)",
  "location": {
    "building": "string",
    "floor": "integer",
    "zone": "string",
    "coordinates": {
      "x": "number (meters)",
      "y": "number (meters)",
      "z": "number (meters, optional)"
    }
  },
  "status": "string (normal|alarm|trouble|disabled)",
  "readings": {
    "value": "number",
    "unit": "string",
    "timestamp": "string (ISO 8601)"
  },
  "metadata": {
    "manufacturer": "string",
    "model": "string",
    "firmwareVersion": "string",
    "installationDate": "string (ISO 8601)",
    "lastMaintenance": "string (ISO 8601)"
  }
}
```

**Requirements:**
- Sensor IDs MUST be globally unique UUID v4
- Timestamps MUST use ISO 8601 format with UTC timezone
- Coordinates MUST use WGS84 datum, metric units
- Status values MUST be from enumerated set
- All numeric readings MUST include units

#### 3.1.2 Alarm Event Schema

Alarm events SHALL conform to the following structure:

```json
{
  "eventId": "string (UUID v4)",
  "eventType": "string (fire|supervisory|trouble|test)",
  "priority": "string (critical|high|medium|low)",
  "source": {
    "deviceId": "string (UUID v4)",
    "deviceType": "string",
    "location": { /* Location object */ }
  },
  "timestamp": "string (ISO 8601)",
  "description": "string",
  "notifications": ["string (array of notification targets)"],
  "status": "string (active|acknowledged|resolved)",
  "acknowledgedBy": "string (user ID, optional)",
  "acknowledgedAt": "string (ISO 8601, optional)",
  "resolvedAt": "string (ISO 8601, optional)"
}
```

---

## 4. Security Requirements

### 4.1 Encryption

All network communications MUST use TLS 1.3 or later with:
- Minimum 2048-bit RSA or 256-bit ECC certificates
- Perfect forward secrecy (PFS) cipher suites
- Certificate pinning for critical connections
- Automatic certificate rotation before expiration

### 4.2 Authentication

Multi-factor authentication REQUIRED for:
- Administrative access
- System configuration changes
- Alarm silence/acknowledge operations
- Emergency override functions

### 4.3 Audit Logging

All security-relevant events SHALL be logged:
- User authentication attempts (success/failure)
- Configuration changes with before/after values
- Alarm acknowledgment and silence operations
- Emergency override activations
- Certificate/key updates
- Network connection establishment/termination

Logs MUST be:
- Tamper-evident using cryptographic hashing
- Retained for minimum 1 year
- Backed up to separate secure storage
- Available for compliance audits

---

## 5. Performance Requirements

### 5.1 Latency

Maximum end-to-end latency requirements:

| Operation | Maximum Latency |
|-----------|----------------|
| Sensor to Panel | 1 second |
| Panel Processing | 500 milliseconds |
| Alarm Notification | 1 second |
| Total Detection to Alert | 3 seconds |
| API Response (read) | 100 milliseconds |
| API Response (write) | 500 milliseconds |

### 5.2 Reliability

System availability requirements:
- Control panels: 99.99% uptime (52 minutes downtime/year)
- Network infrastructure: 99.9% availability
- Power backup: Minimum 24 hours battery capacity
- Redundant panels: Automatic failover <5 seconds

### 5.3 Scalability

Systems MUST support:
- Minimum 10,000 addressable devices per panel
- 100 networked panels per system
- 1,000,000 total devices in enterprise deployments
- 1,000 concurrent API connections
- 10,000 events per second

---

## 6. Testing & Certification

### 6.1 Conformance Testing

Products seeking WIA certification MUST pass:
- Protocol compliance tests
- Interoperability tests with reference implementation
- Performance benchmark tests
- Security penetration tests
- Stress and reliability tests

### 6.2 Certification Levels

**Level 1 - Basic Conformance:**
- Mandatory requirements only
- Single-vendor deployment

**Level 2 - Multi-Vendor:**
- Interoperability demonstrated
- Mixed vendor device support

**Level 3 - Advanced Features:**
- Optional features implemented
- Enhanced integration capabilities
- Analytics and predictive maintenance

---

## 7. Implementation Guidance

### 7.1 Migration Strategies

Organizations can adopt Phase 1 through:
- **Gateway Translation:** Interface devices translate legacy protocols
- **Phased Replacement:** Incremental device replacement over time
- **New Construction:** Full WIA compliance from initial installation
- **Retrofit:** Upgrade existing systems with WIA-compatible components

### 7.2 Best Practices

1. **Start with Reference Implementation:** Use WIA-provided reference code
2. **Comprehensive Testing:** Automated test suites for all requirements
3. **Security First:** Implement security requirements before features
4. **Documentation:** Maintain complete system documentation
5. **Training:** Ensure staff trained on WIA standard compliance

---

## 8. Appendices

### Appendix A: JSON Schema Definitions

Complete JSON Schema files available at:  
https://github.com/WIA-Official/fire-safety-system/schemas/

### Appendix B: Reference Implementations

Reference implementations in multiple languages:
- TypeScript/Node.js
- Python 3.x
- Java 11+
- C++17
- Go 1.19+

Available at: https://github.com/WIA-Official/fire-safety-system/

### Appendix C: Conformance Test Suite

Comprehensive test suite for certification testing:  
https://github.com/WIA-Official/fire-safety-system/tests/

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-27 | Initial official release |

---

**Copyright © 2025 World Certification Industry Association**  
**License:** Creative Commons Attribution 4.0 International (CC BY 4.0)  
**Philosophy:** 弘益人間 (홍익인간) · Benefit All Humanity


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

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
