# Chapter 3: WIA Standard Overview

## Architecture and Design Principles of the WIA Deep Sea Exploration Standard

---

## 3.1 Standard Scope and Objectives

### Defining the Boundaries

The WIA Deep Sea Exploration Standard provides comprehensive specifications for the collection, formatting, transmission, and archival of data from underwater vehicles and ocean observation systems. The standard addresses the complete data lifecycle—from sensor to archive—ensuring interoperability and long-term value.

**In Scope**:

| Category | Included Elements |
|----------|------------------|
| Data Formats | Telemetry, oceanographic, bathymetric, biological, geological, video metadata |
| APIs | Vehicle control, data access, mission management, streaming |
| Protocols | Acoustic communication, file transfer, real-time streaming |
| Integration | Sensor frameworks, cloud connectivity, visualization |
| Certification | Compliance levels, testing procedures, certification process |

**Out of Scope**:

| Category | Reason | Reference Standard |
|----------|--------|-------------------|
| Vehicle mechanical design | Varies by application | Classification societies |
| Pressure vessel certification | Safety-critical, regulated | ASME, DNV |
| Electrical safety | Covered by existing standards | IEC 60335 |
| Life support (HOVs) | Specialized domain | Classification societies |
| Acoustic modem hardware | Manufacturer-specific | ITU-T |

### Strategic Objectives

The WIA Deep Sea Exploration Standard aims to achieve:

1. **Interoperability**: Enable seamless data exchange between institutions, vehicles, and systems worldwide

2. **Efficiency**: Reduce the 20-30% of research time currently spent on data conversion and reformatting

3. **Quality**: Ensure data includes necessary metadata for scientific validity and long-term usability

4. **Innovation**: Provide a stable foundation that encourages technology development without lock-in

5. **Accessibility**: Make deep-sea data available to researchers regardless of institutional resources

6. **Sustainability**: Enable long-term archival and discovery of ocean data

### Success Metrics

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Adoption rate | 50+ institutions by 2027 | Registry of compliant systems |
| Data conversion reduction | 80% less time | User surveys |
| Dataset discoverability | 95% findable | Metadata catalog statistics |
| Cross-institution sharing | 10x increase | Data repository metrics |
| Format validation success | 99%+ | Automated testing |

---

## 3.2 Four-Phase Implementation

The WIA Deep Sea Exploration Standard is organized into four phases, each building on the previous:

### Phase 1: Data Format Specification

**Purpose**: Define the structure and content of all data types

**Key Deliverables**:
- JSON-based message formats with validation schemas
- Binary format for bandwidth-constrained scenarios
- File organization and archival structures
- Metadata requirements for all data types

**Core Data Types**:

| Data Type | Primary Use | Update Rate |
|-----------|-------------|-------------|
| Vehicle Telemetry | Real-time monitoring | 1-10 Hz |
| Oceanographic Data | Environmental conditions | 0.1-1 Hz |
| Bathymetric Data | Seafloor mapping | Survey-based |
| Sample Metadata | Biological/geological samples | Event-based |
| Navigation Data | Position and orientation | 1-10 Hz |
| Video Metadata | Media indexing | Frame-based |
| Mission Logs | Event documentation | Event-based |

**Example: Base Message Format**:

```json
{
  "wiaVersion": "1.0",
  "messageType": "OCEANOGRAPHIC_DATA",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sequenceNumber": 12345,
  "sourceId": "ROV-ATLANTIS-001",
  "priority": "NORMAL",
  "payload": { ... },
  "metadata": { ... },
  "checksum": "SHA256:abc123..."
}
```

### Phase 2: API Interface Specification

**Purpose**: Define programmatic interfaces for data access and vehicle control

**Key Deliverables**:
- RESTful API specification (OpenAPI 3.0)
- WebSocket streaming protocol
- GraphQL query interface (optional)
- Authentication and authorization framework
- SDK reference implementations

**API Categories**:

| Category | Endpoints | Methods |
|----------|-----------|---------|
| Telemetry | /telemetry | GET, WebSocket |
| Data Access | /data, /samples, /bathymetry | GET, POST |
| Mission | /missions, /waypoints | GET, POST, PUT, DELETE |
| Vehicle Control | /commands, /configuration | POST, GET |
| System | /health, /version, /calibration | GET |

**Example: Telemetry Stream Subscription**:

```javascript
const ws = new WebSocket('wss://vehicle.example.com/api/v1/telemetry/stream');

ws.onmessage = (event) => {
  const telemetry = JSON.parse(event.data);
  console.log(`Depth: ${telemetry.payload.position.depth}m`);
  console.log(`Temperature: ${telemetry.payload.environment.temperature.value}°C`);
};

ws.send(JSON.stringify({
  action: 'subscribe',
  channels: ['navigation', 'environment'],
  rate: 1 // Hz
}));
```

### Phase 3: Communication Protocol

**Purpose**: Specify protocols for reliable data transmission in underwater environments

**Key Deliverables**:
- Acoustic communication packet formats
- Error correction and retry mechanisms
- Priority-based data transmission
- Multi-vehicle networking
- Store-and-forward protocols

**Protocol Stack**:

```
┌─────────────────────────────────────┐
│       Application (WIA Data)       │
├─────────────────────────────────────┤
│     Transport (Reliable/Best-Effort)│
├─────────────────────────────────────┤
│    Network (Acoustic Addressing)    │
├─────────────────────────────────────┤
│  Physical (Acoustic/Optical/Fiber)  │
└─────────────────────────────────────┘
```

**Priority Classes**:

| Priority | Examples | Delivery Guarantee |
|----------|----------|-------------------|
| CRITICAL | Emergency abort, safety alerts | Immediate, retry until ACK |
| HIGH | Navigation updates, commands | Reliable with timeout |
| NORMAL | Scientific data, telemetry | Best effort with buffering |
| LOW | Video frames, bulk data | Opportunistic |

### Phase 4: System Integration

**Purpose**: Provide guidance for end-to-end system integration

**Key Deliverables**:
- Reference architecture
- Sensor integration framework
- Cloud connectivity patterns
- Visualization requirements
- Testing and validation procedures

**Reference Architecture**:

```
┌─────────────────────────────────────────────────────────────┐
│                     Shore-Side Systems                      │
│  ┌─────────┐  ┌──────────┐  ┌─────────┐  ┌──────────────┐ │
│  │  Data   │  │ Mission  │  │ Archive │  │ Visualization│ │
│  │ Archive │  │ Planning │  │  Portal │  │    Tools     │ │
│  └────┬────┘  └────┬─────┘  └────┬────┘  └──────┬───────┘ │
│       └────────────┴──────────────┴──────────────┘         │
│                           │                                 │
└───────────────────────────┼─────────────────────────────────┘
                            │ Internet/Satellite
┌───────────────────────────┼─────────────────────────────────┐
│                    Ship Systems                             │
│  ┌─────────────┐  ┌───────┴───────┐  ┌──────────────────┐ │
│  │   Mission   │  │  Data Server  │  │   USBL/Acoustic  │ │
│  │   Control   │  │   & Storage   │  │    Positioning   │ │
│  └──────┬──────┘  └───────┬───────┘  └────────┬─────────┘ │
│         └─────────────────┴───────────────────┘             │
│                           │                                 │
└───────────────────────────┼─────────────────────────────────┘
                            │ Fiber/Acoustic
┌───────────────────────────┼─────────────────────────────────┐
│                Underwater Vehicles                          │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌──────────────┐  │
│  │   ROV   │  │   AUV   │  │ Sensors │  │  Transponders │ │
│  └─────────┘  └─────────┘  └─────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

---

## 3.3 Compliance Levels

The WIA standard defines three compliance levels, allowing organizations to implement according to their capabilities and needs:

### Level 1: Basic Compliance

**Requirements**:
- Implement core message format (base header fields)
- Support at least three data types (vehicle telemetry, navigation, one scientific)
- Validate messages against WIA JSON Schema
- Provide basic metadata (source, timestamp, checksum)

**Suitable For**:
- Small research vessels
- Educational institutions
- Prototype systems
- Legacy system integration

**Certification Process**:
- Self-certification with test suite
- Online validation tool
- No formal audit required

### Level 2: Standard Compliance

**Requirements**:
- All Level 1 requirements
- Implement all Phase 1 data types
- Implement core Phase 2 APIs (telemetry, data access)
- Full metadata according to specification
- Binary format support for low-bandwidth scenarios
- Error handling and retry mechanisms

**Suitable For**:
- University research vessels
- Commercial survey operations
- National research institutions
- Industry-standard equipment manufacturers

**Certification Process**:
- Formal test suite execution
- Sample data submission
- Documentation review
- Certificate issued by WIA

### Level 3: Advanced Compliance

**Requirements**:
- All Level 2 requirements
- Full Phase 2 API implementation
- Phase 3 protocol implementation
- Phase 4 integration with reference architecture
- Real-time streaming with guaranteed delivery
- Cloud integration capabilities
- Extended metadata and provenance tracking

**Suitable For**:
- Major research institutions (WHOI, MBARI, JAMSTEC)
- Deep-sea mining operations
- Government agencies
- Platform manufacturers

**Certification Process**:
- Full audit by WIA certification body
- Interoperability testing with reference implementation
- Annual recertification
- Listed in WIA certified products registry

### Compliance Matrix

| Requirement | Level 1 | Level 2 | Level 3 |
|-------------|---------|---------|---------|
| Base message format | ✓ | ✓ | ✓ |
| JSON Schema validation | ✓ | ✓ | ✓ |
| Three data types | ✓ | ✓ | ✓ |
| All data types | - | ✓ | ✓ |
| Core APIs | - | ✓ | ✓ |
| Full APIs | - | - | ✓ |
| Binary format | - | ✓ | ✓ |
| Protocol implementation | - | - | ✓ |
| Cloud integration | - | - | ✓ |
| Self-certification | ✓ | - | - |
| Formal certification | - | ✓ | ✓ |
| Annual audit | - | - | ✓ |

---

## 3.4 Relationship to Existing Standards

### Harmonization Approach

The WIA standard does not seek to replace existing oceanographic standards but to provide a unified framework that harmonizes and extends them:

**Design Principles**:
1. **Adoption**: Incorporate proven elements from existing standards
2. **Extension**: Add capabilities not covered by existing standards
3. **Mapping**: Provide clear conversion pathways
4. **Coexistence**: Support parallel use during transition

### Related Standards

| Standard/Organization | Focus Area | WIA Relationship |
|----------------------|------------|------------------|
| NOAA Data Standards | US ocean data | Compliant, extended |
| OOI Data Products | Observatory data | Compatible formats |
| MBARI VARS | Video annotation | Metadata alignment |
| ISO 19115 | Geographic metadata | Profile incorporated |
| CF Conventions | NetCDF attributes | Mapping provided |
| OGC SensorML | Sensor description | Compatible |
| IEEE 802.11bb | Underwater optical | Physical layer reference |
| JANUS | Acoustic comms | Protocol inspiration |

### NOAA Standards Alignment

The WIA standard aligns with NOAA's oceanographic data standards:

**Bathymetric Data**:
- Compatible with BAG (Bathymetric Attributed Grid) format
- Supports NOAA's NCEI archival requirements
- Includes multibeam-specific metadata

**Water Column Data**:
- Aligned with NOAA's WCD format requirements
- CTD data compatible with SeaDataNet formats
- Supports NOAA's ERDDAP data server

**Biological Observations**:
- Mapping to Darwin Core standard
- OBIS (Ocean Biodiversity Information System) compatible
- GBIF-ready export option

### OOI (Ocean Observatories Initiative) Compatibility

The WIA standard supports OOI data products and workflows:

- Compatible timestamp and coordinate formats
- Aligned quality control flag values
- Interoperable with OOI data portal
- Supports OOI sensor type nomenclature

### Example: Standard Mapping

WIA Oceanographic Data → CF-compliant NetCDF:

| WIA Field | CF Standard Name | Unit |
|-----------|-----------------|------|
| environment.temperature.value | sea_water_temperature | degrees_C |
| environment.salinity.value | sea_water_practical_salinity | 1 (PSU) |
| environment.dissolvedOxygen.value | mass_concentration_of_oxygen_in_sea_water | kg m-3 |
| location.depth | depth | m |
| location.latitude | latitude | degrees_north |
| location.longitude | longitude | degrees_east |

---

## 3.5 Certification Process

### Overview

WIA certification demonstrates that a system complies with the standard and can interoperate with other certified systems. The certification process varies by compliance level.

### Level 1 Self-Certification

**Process**:
1. Download WIA validation toolkit
2. Run automated tests against your system
3. Submit passing test report to WIA registry
4. Receive Level 1 certification badge

**Timeline**: 1-2 days
**Cost**: Free

### Level 2 Formal Certification

**Process**:

1. **Application** (Week 1)
   - Submit application form
   - Provide system documentation
   - Pay certification fee

2. **Documentation Review** (Week 2-3)
   - WIA reviewers assess documentation
   - Identify any gaps or issues
   - Provide feedback for correction

3. **Test Execution** (Week 4-5)
   - Execute full test suite
   - Submit test results and sample data
   - WIA validates results

4. **Certification Decision** (Week 6)
   - Review board assesses compliance
   - Issue certificate or remediation list
   - Publish in certified products registry

**Timeline**: 6-8 weeks
**Cost**: $2,500 USD

### Level 3 Advanced Certification

**Process**:

1. **Pre-Assessment** (Month 1)
   - Initial system assessment
   - Identify integration requirements
   - Plan interoperability testing

2. **Implementation Review** (Month 2)
   - On-site or remote audit
   - Code and architecture review
   - Documentation assessment

3. **Interoperability Testing** (Month 3)
   - Test against WIA reference implementation
   - Multi-system data exchange testing
   - Real-world scenario validation

4. **Certification** (Month 4)
   - Final report and recommendations
   - Certificate issuance
   - Annual recertification schedule

**Timeline**: 3-4 months
**Cost**: $10,000-25,000 USD (based on system complexity)

### Certification Maintenance

| Level | Validity | Recertification |
|-------|----------|-----------------|
| Level 1 | 2 years | Re-run test suite |
| Level 2 | 2 years | Abbreviated review |
| Level 3 | 1 year | Annual audit |

---

## 3.6 Version Control and Updates

### Versioning Scheme

The WIA standard uses semantic versioning (SemVer):

```
MAJOR.MINOR.PATCH
  │      │     │
  │      │     └── Bug fixes, clarifications
  │      └──────── New features, backward-compatible
  └─────────────── Breaking changes
```

**Current Version**: 1.0.0

**Version History**:

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |
| 0.9.0 | 2024-09 | Release candidate |
| 0.8.0 | 2024-06 | Beta, Phase 4 complete |
| 0.5.0 | 2024-03 | Alpha, Phase 2 complete |

### Update Process

**Annual Review Cycle**:
1. January-March: Collect feedback and change requests
2. April-June: Working group review and drafting
3. July-August: Public comment period
4. September: Final revisions
5. October-November: Interoperability testing
6. December: Version release

**Change Categories**:

| Category | Example | Process |
|----------|---------|---------|
| Errata | Typo fixes | Published immediately |
| Clarification | Ambiguous text | Quarterly update |
| Minor enhancement | New optional field | Annual cycle |
| New feature | New data type | Annual cycle, major feedback |
| Breaking change | Field removal | Major version, 2-year transition |

### Backward Compatibility

The WIA standard prioritizes backward compatibility:

**Guaranteed**:
- Level 2 systems will read Level 1 data
- Level 3 systems will read Level 1 and Level 2 data
- Minor version updates are backward compatible
- Required fields never removed in minor versions

**Transition Period**:
- Breaking changes announced 12 months in advance
- Both old and new formats supported during transition
- Tools provided for data migration
- Certified systems given extension for re-certification

### Extension Mechanism

The WIA standard allows extensions without modifying the core specification:

**Custom Fields**:
```json
{
  "wiaVersion": "1.0",
  "messageType": "CUSTOM_DATA",
  "payload": {
    "wiaExtension": {
      "vendor": "MBARI",
      "extensionId": "midwater-respiration-chamber",
      "version": "2.1",
      "data": {
        "chamberVolume": 5.2,
        "oxygenFlux": 0.042
      }
    }
  }
}
```

**Rules for Extensions**:
- Extension namespace required (vendor identifier)
- Must not conflict with standard field names
- Should be documented and published
- May be candidates for future standard inclusion

---

## Chapter Summary

The WIA Deep Sea Exploration Standard provides a comprehensive framework for oceanographic data interoperability through its four-phase approach: Data Format, API Interface, Communication Protocol, and System Integration. Three compliance levels (Basic, Standard, Advanced) allow organizations to implement according to their capabilities, with clear certification pathways for each level.

The standard harmonizes with existing oceanographic standards (NOAA, OOI, CF Conventions) while extending capabilities to address gaps in underwater vehicle data, real-time streaming, and system integration. Semantic versioning and a transparent update process ensure stability while allowing the standard to evolve.

Organizations implementing the WIA standard benefit from reduced data conversion effort, improved collaboration with partners, and future-proof data archival. The philosophy of 弘益人間 ensures that these benefits extend to the global oceanographic community.

---

## Key Takeaways

1. **Four phases build progressively**: Data Format → API → Protocol → Integration
2. **Three compliance levels accommodate different needs**: Basic (self-cert), Standard (formal), Advanced (audit)
3. **The standard harmonizes with existing standards** rather than replacing them
4. **Certification demonstrates interoperability** and is required for Level 2/3
5. **Semantic versioning ensures stability** with clear backward compatibility guarantees
6. **Extensions allow vendor-specific data** without fragmenting the standard

---

## Review Questions

1. What are the four phases of the WIA Deep Sea Exploration Standard?
2. What is the minimum requirement for Level 1 compliance?
3. How does the WIA standard relate to NOAA oceanographic data standards?
4. What is the difference between Level 2 and Level 3 certification?
5. Explain the semantic versioning scheme used by the WIA standard.
6. How are breaking changes handled to ensure backward compatibility?
7. What is the purpose of the extension mechanism?

---

## Compliance Checklist

### Level 1 Quick Check

- [ ] Base message format implemented with all required fields
- [ ] At least three data types supported
- [ ] JSON Schema validation passing
- [ ] Timestamps in ISO8601 UTC format
- [ ] Checksums calculated correctly
- [ ] WIA validation toolkit executed successfully

### Level 2 Additional Requirements

- [ ] All Phase 1 data types implemented
- [ ] Core REST APIs functional
- [ ] Binary format for low-bandwidth supported
- [ ] Full metadata per specification
- [ ] Sample data submitted to WIA
- [ ] Documentation complete

### Level 3 Additional Requirements

- [ ] Full API implementation
- [ ] Protocol layer operational
- [ ] Cloud integration demonstrated
- [ ] Interoperability test passed
- [ ] Architecture review completed
- [ ] Annual audit scheduled

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
