# WIA-ENE-050: Direct Air Capture Standard
## PHASE 1: FOUNDATION

**Version:** 1.0
**Status:** Draft
**Date:** 2025-12-25
**Category:** Energy (ENE)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the World Internet Alliance (WIA) standard for Direct Air Capture (DAC) technology. WIA-ENE-050 establishes standardized data formats, protocols, and integration methods for DAC facilities that remove CO2 directly from the atmosphere.

### 1.2 Scope

This standard covers:
- DAC facility data structures and metadata
- CO2 capture measurement and reporting
- Sorbent efficiency tracking
- Energy consumption metrics
- Storage and sequestration protocols
- Verification and certification processes
- Carbon credit generation and tracking
- Integration with CCS infrastructure

### 1.3 Background

Direct Air Capture is a critical carbon dioxide removal (CDR) technology that extracts CO2 directly from ambient air. Unlike point-source capture at industrial facilities, DAC can be deployed anywhere and addresses legacy emissions. As DAC technology scales globally, standardization becomes essential for:

- **Transparency:** Verifiable reporting of captured CO2
- **Interoperability:** Integration between facilities and storage sites
- **Credibility:** Standardized verification for carbon credits
- **Efficiency:** Sharing of best practices and optimization methods

### 1.4 Philosophy

弘益人間 (Hongik Ingan) - Benefit All Humanity

This standard embodies WIA's commitment to creating technologies that serve humanity and the planet. By standardizing DAC systems, we enable a global network of carbon removal facilities working together to reverse climate change.

---

## 2. Terminology

### 2.1 Key Terms

**Direct Air Capture (DAC):** Technology that removes CO2 directly from ambient air using chemical processes.

**Sorbent:** Material that captures CO2 through chemical or physical adsorption. Can be solid (e.g., amine-functionalized materials) or liquid (e.g., alkaline solutions).

**Adsorption:** The capture phase where CO2 binds to the sorbent material.

**Regeneration:** The release phase where heat or other processes free the CO2 from the sorbent, producing a concentrated CO2 stream.

**Specific Energy:** Energy required per ton of CO2 captured, measured in MWh/ton CO2 or GJ/ton CO2.

**Carbon Dioxide Removal (CDR):** Activities that remove CO2 from the atmosphere and durably store it.

**CCS (Carbon Capture and Storage):** The process of capturing CO2 and storing it underground or in other permanent reservoirs.

**Mineralization:** Converting CO2 into solid carbonate minerals through chemical reactions with rocks (e.g., basalt).

**MRV (Monitoring, Reporting, Verification):** Framework for tracking and verifying carbon removal claims.

### 2.2 DAC Technology Types

**Solid Sorbent DAC:**
- Uses solid materials (typically amine-based) on porous supports
- Lower temperature regeneration (80-120°C)
- Examples: Climeworks, Global Thermostat
- Advantages: Modular, lower energy requirements

**Liquid Solvent DAC:**
- Uses liquid alkaline solutions (e.g., potassium hydroxide)
- Higher temperature regeneration (>900°C for calcination)
- Examples: Carbon Engineering
- Advantages: Higher throughput, mature chemistry

**Electrochemical DAC:**
- Uses electrochemical cells to capture and release CO2
- Voltage-driven process
- Examples: Emerging technologies
- Advantages: Potentially lower energy, no thermal swing

### 2.3 Abbreviations

- **DAC:** Direct Air Capture
- **CDR:** Carbon Dioxide Removal
- **CCS:** Carbon Capture and Storage
- **MRV:** Monitoring, Reporting, Verification
- **VC:** Verifiable Credential
- **DID:** Decentralized Identifier
- **API:** Application Programming Interface
- **IoT:** Internet of Things
- **GHG:** Greenhouse Gas

---

## 3. System Architecture

### 3.1 Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA-ENE-050 ECOSYSTEM                     │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ DAC Facility │  │   Storage    │  │ Verification │     │
│  │     Data     │──│   Systems    │──│   Services   │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
│         │                  │                  │             │
│         └──────────────────┴──────────────────┘             │
│                           │                                 │
│                  ┌────────▼────────┐                        │
│                  │   WIA-ENE-050   │                        │
│                  │   Data Layer    │                        │
│                  └────────┬────────┘                        │
│                           │                                 │
│         ┌─────────────────┼─────────────────┐              │
│         │                 │                 │              │
│  ┌──────▼──────┐  ┌──────▼──────┐  ┌──────▼──────┐       │
│  │  Monitoring │  │   Carbon    │  │    Public   │       │
│  │  Dashboard  │  │   Credit    │  │   Registry  │       │
│  │             │  │  Registry   │  │             │       │
│  └─────────────┘  └─────────────┘  └─────────────┘       │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 Core Components

#### 3.2.1 DAC Facility Layer
- **Sensors & IoT Devices:** Measure CO2, temperature, humidity, airflow, energy
- **Process Control Systems:** Manage adsorption/regeneration cycles
- **Data Collection:** Real-time capture metrics and operational data
- **Local Storage:** Time-series database for historical data

#### 3.2.2 Data Standardization Layer
- **Data Formatters:** Convert proprietary formats to WIA-ENE-050 schema
- **Validation Services:** Ensure data integrity and completeness
- **Aggregation Services:** Combine data from multiple modules/facilities
- **API Gateways:** Expose standardized data endpoints

#### 3.2.3 Integration Layer
- **Storage Integration:** Connect to geological storage, mineralization sites
- **Pipeline Networks:** Coordinate CO2 transport infrastructure
- **Energy Systems:** Interface with renewable energy providers
- **MRV Systems:** Submit data for verification

#### 3.2.4 Verification Layer
- **Third-Party Auditors:** Independent verification of capture claims
- **Blockchain/DLT:** Immutable record of verified captures
- **Credential Issuance:** Generate verifiable credentials (VCs)
- **Registry Integration:** Submit to carbon credit registries

#### 3.2.5 Application Layer
- **Public Dashboards:** Real-time transparency of capture operations
- **Carbon Markets:** Trading platforms for carbon removal credits
- **Compliance Reporting:** Regulatory and voluntary reporting
- **Research Data:** Anonymized data for scientific research

### 3.3 Data Flow

```
1. CAPTURE
   DAC Facility → Sensors → Process Data
   ↓
2. STANDARDIZE
   Raw Data → WIA-ENE-050 Format → Validation
   ↓
3. STORE
   Validated Data → Time-Series DB → Historical Archive
   ↓
4. VERIFY
   Capture Data → MRV System → Third-Party Audit
   ↓
5. CERTIFY
   Verified Data → VC Issuance → Blockchain Record
   ↓
6. REGISTER
   Certificate → Carbon Registry → Credits Issued
   ↓
7. PUBLISH
   Aggregated Data → Public API → Dashboards/Apps
```

---

## 4. Core Principles

### 4.1 Transparency

All DAC operations must be transparently reported using standardized formats. This includes:
- Real-time capture rates
- Energy consumption per ton CO2
- Sorbent efficiency and degradation
- Storage method and location
- Verification status

### 4.2 Verifiability

All carbon removal claims must be independently verifiable through:
- Third-party audits (ISO 27916, ISO 14064-2)
- Continuous monitoring systems
- Blockchain-based immutable records
- Open data APIs for public scrutiny

### 4.3 Interoperability

WIA-ENE-050 systems must integrate seamlessly with:
- Other DAC facilities (data sharing, best practices)
- Storage infrastructure (geological, mineralization, utilization)
- Carbon registries (Verra, Gold Standard, Puro.earth)
- Energy systems (renewable energy coordination)
- Climate accounting (national inventories, corporate reporting)

### 4.4 Accuracy

Measurements must meet strict accuracy requirements:
- CO2 capture: ±5% uncertainty
- Energy consumption: ±3% uncertainty
- Storage verification: >95% confidence
- Temporal resolution: Hourly or better

### 4.5 Permanence

Carbon removal must be durable:
- Geological storage: >1000 years permanence
- Mineralization: Permanent (>10,000 years)
- Utilization: Case-by-case assessment
- Monitoring: Long-term verification plans

### 4.6 Additionality

DAC operations must demonstrate:
- Removal beyond business-as-usual
- Project would not occur without carbon revenue
- No double-counting with other programs
- Clear baseline and project scenarios

---

## 5. Technical Requirements

### 5.1 Data Collection Requirements

#### 5.1.1 Temporal Resolution
- **Operational Data:** Collected every 1-5 minutes
- **Aggregated Data:** Reported hourly
- **Verification Data:** Daily summaries required
- **Public Reporting:** Monthly aggregates

#### 5.1.2 Measurement Accuracy

| Parameter | Accuracy | Uncertainty |
|-----------|----------|-------------|
| CO2 Captured | ±5% | 95% confidence |
| Energy Consumption | ±3% | 95% confidence |
| Sorbent Mass | ±2% | 99% confidence |
| Ambient CO2 | ±10 ppm | 90% confidence |
| Temperature | ±1°C | 95% confidence |
| Pressure | ±5% | 95% confidence |

#### 5.1.3 Data Retention
- **Raw Data:** 7 years minimum
- **Aggregated Data:** 30 years minimum
- **Verification Records:** Permanent
- **Blockchain Hashes:** Permanent (immutable)

### 5.2 Communication Requirements

#### 5.2.1 API Standards
- **Protocol:** RESTful HTTP/HTTPS, GraphQL
- **Authentication:** OAuth 2.0, API keys
- **Data Format:** JSON, JSON-LD
- **Rate Limiting:** Configurable per client
- **Versioning:** Semantic versioning (v1.0, v1.1, etc.)

#### 5.2.2 Real-Time Data
- **WebSocket Support:** For live monitoring
- **Update Frequency:** 1-60 seconds (configurable)
- **Latency:** <5 seconds from measurement to API
- **Availability:** 99.9% uptime SLA

#### 5.2.3 Data Security
- **Encryption:** TLS 1.3 for all communications
- **Authentication:** Multi-factor authentication for admin
- **Authorization:** Role-based access control (RBAC)
- **Audit Logs:** All API access logged

### 5.3 Storage Requirements

#### 5.3.1 Database
- **Type:** Time-series database (e.g., InfluxDB, TimescaleDB)
- **Backup:** Daily incremental, weekly full
- **Redundancy:** Geographic replication
- **Retention:** As per data retention policy

#### 5.3.2 Blockchain Integration
- **Networks:** Ethereum, Polygon, or permissioned chains
- **Data Stored:** Hashes of verified capture events
- **Smart Contracts:** Carbon credit issuance logic
- **Verification:** Merkle proofs for data integrity

---

## 6. Compliance & Standards

### 6.1 International Standards

#### 6.1.1 Carbon Capture & Storage
- **ISO 27916:** Carbon dioxide capture, transportation and geological storage
- **ISO 27914:** Geological storage of CO2
- **ISO 27913:** Pipeline transportation systems

#### 6.1.2 GHG Accounting
- **ISO 14064-1:** GHG emissions and removals at organizational level
- **ISO 14064-2:** GHG emissions and removals at project level
- **ISO 14064-3:** Validation and verification of GHG statements

#### 6.1.3 Life Cycle Assessment
- **ISO 14040:** LCA principles and framework
- **ISO 14044:** LCA requirements and guidelines

### 6.2 Carbon Crediting Standards

#### 6.2.1 Registries
- **Puro.earth:** CO2 Removal Certificate (CORC) methodology
- **Verra:** VCS (Verified Carbon Standard)
- **Gold Standard:** Renewable energy and climate security
- **American Carbon Registry (ACR):** Carbon offset projects

#### 6.2.2 Methodology Requirements
- Baseline scenario definition
- Additionality demonstration
- Leakage assessment
- Permanence guarantee
- Monitoring plan
- Verification frequency (annual minimum)

### 6.3 Regulatory Compliance

#### 6.3.1 Environmental Regulations
- Clean Air Act (USA)
- EU ETS (Emissions Trading System)
- National carbon pricing mechanisms
- Local air quality standards

#### 6.3.2 Safety Regulations
- Occupational safety (OSHA, EU-OSHA)
- Chemical handling (REACH, TSCA)
- Pressure vessel codes (ASME, PED)
- Emergency response plans

---

## 7. Security & Privacy

### 7.1 Data Security

#### 7.1.1 Confidentiality
- Proprietary operational details protected
- Trade secrets (e.g., sorbent formulations) encrypted
- Competitive information access-controlled
- Public data clearly designated

#### 7.1.2 Integrity
- Cryptographic hashing of all records
- Blockchain anchoring for immutability
- Version control for data updates
- Audit trails for all modifications

#### 7.1.3 Availability
- Redundant systems and backups
- DDoS protection for public APIs
- Disaster recovery procedures
- 99.9% uptime guarantee

### 7.2 Privacy

#### 7.2.1 Personal Data
- Minimize collection of personal information
- GDPR/CCPA compliance for employee data
- Anonymization of research datasets
- Clear privacy policies

#### 7.2.2 Commercial Sensitivity
- Tiered access levels (public, partner, private)
- Non-disclosure agreements for detailed data
- Aggregation to protect individual facility data
- Competitive benchmarking anonymized

---

## 8. Governance

### 8.1 Standard Governance

**Maintainer:** World Internet Alliance (WIA)
**Technical Committee:** WIA-ENE-050 Working Group
**Review Cycle:** Annual
**Amendment Process:** RFC (Request for Comments) → Community Review → Vote → Update

### 8.2 Certification

#### 8.2.1 Facility Certification
- Self-assessment questionnaire
- Documentation review
- Third-party audit
- Annual recertification

#### 8.2.2 Software Certification
- API conformance testing
- Data format validation
- Security audit
- Interoperability testing

### 8.3 Dispute Resolution

- Technical disputes → Technical Committee review
- Certification disputes → Independent arbitration
- Data disputes → Third-party verification
- Appeal process clearly defined

---

## 9. Roadmap

### Phase 1: Foundation (Current)
- Core data schema definition
- Basic API specification
- Pilot implementations
- Community feedback

### Phase 2: Data Formats (Next)
- Detailed data models
- Validation rules
- Reference implementations
- Conformance test suite

### Phase 3: Protocols
- Capture protocols
- Storage protocols
- Verification workflows
- MRV integration

### Phase 4: Integration
- CCS infrastructure integration
- Carbon registry connectors
- Energy system coordination
- Global network deployment

### Phase 5: Ecosystem
- Public dashboards
- Carbon markets integration
- Research data platform
- Policy integration

---

## 10. References

### 10.1 Technical References

1. National Academies (2019). *Negative Emissions Technologies and Reliable Sequestration*
2. IEA (2022). *Direct Air Capture: A key technology for net zero*
3. Climeworks (2020). *Technical Specifications - Orca Plant*
4. Carbon Engineering (2018). *A Process for Capturing CO2 from the Atmosphere*

### 10.2 Standards References

1. ISO 27916:2019 - Carbon dioxide capture, transportation and geological storage
2. ISO 14064-2:2019 - Greenhouse gases, Part 2: Project level
3. IPCC (2018). *Special Report on Global Warming of 1.5°C*
4. Puro.earth (2021). *CO2 Removal Certificate Methodology*

### 10.3 Related WIA Standards

- **WIA-INTENT:** Intent representation (foundation for DAC goals)
- **WIA-OMNI-API:** Universal API framework
- **WIA-SOCIAL:** Social integration for transparency
- **WIA-ENE-XXX:** Other energy standards

---

## 11. Contact & Support

**Website:** https://wia.org/standards/ene-050
**Email:** ene-050@wia.org
**GitHub:** https://github.com/WIA-Official/wia-standards/direct-air-capture
**Forum:** https://forum.wia.org/c/ene-050

---

## License

This specification is licensed under **CC BY 4.0** (Creative Commons Attribution 4.0 International).

You are free to:
- Share: copy and redistribute the material
- Adapt: remix, transform, and build upon the material

Under the following terms:
- Attribution: You must give appropriate credit to WIA

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / World Internet Alliance (WIA)
