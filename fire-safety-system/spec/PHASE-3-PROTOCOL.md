# WIA Fire Safety System Standard
## PHASE 3 Specification v1.0

**Status:** Official Release  
**Date:** 2025-12-27  
**Organization:** World Certification Industry Association (WIA)  
**Philosophy:** 弘益人間 (홍익인간) · Benefit All Humanity

---

## Executive Summary

This document specifies Phase 3 of the WIA Fire Safety System Standard, providing comprehensive technical requirements for communication protocols in modern fire safety infrastructure. The specification enables interoperability between products from different manufacturers while supporting innovation and competition.

**Key Objectives:**
- Establish standardized communication protocols
- Enable seamless multi-vendor system deployment
- Reduce total cost of ownership through competition
- Improve system reliability and maintainability
- Support advanced features like analytics and predictive maintenance

**Scope:** This specification applies to all fire safety system components including fire alarm control panels, detection devices, notification appliances, suppression systems, and integration interfaces.

---

## 1. Introduction

### 1.1 Purpose

Phase 3 of the WIA Fire Safety System Standard addresses the communication protocols enabling reliable fire safety system operation across diverse network infrastructures. Protocol specifications ensure consistent behavior during normal operation and emergency conditions when reliability is paramount.

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

Phase 3 focuses on the Protocol Layer specifications.

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

### 3.1 Protocol Message Formats

#### 3.1.1 Fire Detection Protocol

State machine for fire detection and alarm:

```
┌──────────┐
│  NORMAL  │ ◄─────────────────────────┐
└────┬─────┘                            │
     │ Sensor trigger                   │
     │                                  │
     ▼                                  │
┌──────────┐                            │
│PRE-ALARM │                            │
└────┬─────┘                            │
     │ Verification (multi-sensor)     │
     │                                  │
     ▼                                  │
┌──────────┐    Silenced/Resolved      │
│  ALARM   │──────────────────────────►│
└────┬─────┘                            │
     │ Acknowledged                     │
     │                                  │
     ▼                                  │
┌──────────┐    Investigation complete │
│  ACK     │──────────────────────────►│
└──────────┘
```

**Message Format:**
```
HEADER | TYPE | PRIORITY | SOURCE | TIMESTAMP | DATA | CHECKSUM
```

**Timing Requirements:**
- Sensor response: <1s from fire detection to signal
- Panel processing: <500ms from signal to alarm
- Notification activation: <1s from alarm to audible/visual
- Network propagation: <2s end-to-end for distributed systems

#### 3.1.2 Evacuation Coordination Protocol

Phased evacuation messaging:

1. **Alert Phase (T+0):** Silent notification to security/management
2. **Zone Evacuation (T+30s):** Alert affected zones
3. **Floor Evacuation (T+60s):** Evacuate entire floor
4. **Building Evacuation (T+90s):** Full building evacuation

Voice evacuation messages SHALL:
- Use clear, calm, authoritative tone
- Provide specific instructions
- Repeat at 10-20 second intervals
- Support multiple languages (configurable)

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

Organizations can adopt Phase 3 through:
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


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
