# WIA-SPACE-018: Urban Air Mobility (UAM) Standard
## Version 1.0

**Status:** Active
**Published:** 2025-01-15
**Organization:** WIA - World Certification Industry Association
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## Executive Summary

The WIA-SPACE-018 Urban Air Mobility (UAM) standard establishes a comprehensive framework for electric vertical takeoff and landing (eVTOL) aircraft operations in urban environments. This standard addresses aircraft design, vertiport infrastructure, air traffic management, regulatory compliance, passenger experience, and operational procedures necessary for safe, efficient, and sustainable UAM deployment.

UAM represents a transformative approach to urban transportation, leveraging three-dimensional airspace to reduce congestion, lower emissions, and provide rapid point-to-point mobility. This standard guides stakeholders—including aircraft manufacturers, operators, infrastructure developers, regulators, and cities—toward interoperable, safe, and scalable UAM systems that benefit all humanity.

---

## 1. Scope

### 1.1 Applicability

This standard applies to:

- **eVTOL Aircraft:** Electric vertical takeoff and landing aircraft designed for urban passenger and cargo operations
- **Vertiports:** Ground infrastructure facilities supporting UAM operations
- **UTM Systems:** UAM Traffic Management systems coordinating airspace operations
- **Operators:** Entities conducting commercial UAM passenger or cargo services
- **Infrastructure Providers:** Organizations developing and operating vertiport facilities
- **Technology Suppliers:** Providers of batteries, propulsion, avionics, and other critical systems

### 1.2 Out of Scope

This standard does not address:

- Conventional helicopters (covered by existing aviation standards)
- Long-range eVTOL aircraft primarily serving regional routes >200 miles
- Unmanned cargo drones <200 lbs (covered by small UAS regulations)
- Military or defense UAM applications

---

## 2. Normative References

- FAA 14 CFR Part 23 - Airworthiness Standards: Normal Category Airplanes
- FAA 14 CFR Part 27 - Airworthiness Standards: Normal Category Rotorcraft
- FAA 14 CFR Part 135 - Operating Requirements: Commuter and On Demand Operations
- EASA CS-23 - Certification Specifications for Normal, Utility, Aerobatic, and Commuter Category Aeroplanes
- NASA UAM Maturity Levels (UML) Framework
- RTCA DO-178C - Software Considerations in Airborne Systems
- RTCA DO-254 - Design Assurance Guidance for Airborne Electronic Hardware
- SAE ARP 4761 - Guidelines and Methods for Conducting Safety Assessment Process
- NFPA 418 - Standard for Heliports
- ICAO Annex 14 - Aerodromes (Heliport Design)

---

## 3. Terms and Definitions

**Advanced Air Mobility (AAM):** Air transportation system using revolutionary aircraft for previously underserved routes, including UAM, regional air mobility, and cargo delivery.

**eVTOL:** Electric Vertical Takeoff and Landing aircraft using electric propulsion and vertical flight capability.

**FATO:** Final Approach and Takeoff Area - area over which final approach and takeoff occur.

**TLOF:** Touchdown and Liftoff Area - load-bearing surface where aircraft touch down and lift off.

**UAM:** Urban Air Mobility - air transportation of passengers and cargo within and around metropolitan areas using eVTOL aircraft.

**UTM:** UAM Traffic Management - system coordinating safe, efficient UAM operations through digital infrastructure and automation.

**UML:** UAM Maturity Level - NASA framework defining operational sophistication from nascent (UML-1) to full autonomy (UML-6).

**Vertiport:** Facility designed for eVTOL aircraft takeoff, landing, charging, passenger boarding, and integration with ground transportation.

**Vertistop:** Simplified facility with basic landing and charging capabilities.

**弘益人間 (Hongik Ingan):** Korean philosophy meaning "widely benefiting all people" - guiding principle for UAM development ensuring equitable access and community benefit.

---

## 4. UAM System Architecture

### 4.1 System Components

Complete UAM systems comprise five primary components:

1. **eVTOL Aircraft:** Electric aircraft providing vertical takeoff/landing with range 50-200 miles
2. **Vertiport Infrastructure:** Ground facilities for operations, charging, and passenger services
3. **UTM Systems:** Digital airspace management coordinating safe operations
4. **Ground Transportation:** Multimodal connections completing door-to-door journeys
5. **Operations Support:** Maintenance, training, customer service, and emergency response

### 4.2 System Integration Requirements

UAM systems SHALL integrate seamlessly across components:

- Aircraft SHALL interface with vertiport charging systems per Section 6.3
- Aircraft SHALL communicate with UTM systems per Section 7.2
- Vertiports SHALL integrate with ground transportation per Section 5.4
- All systems SHALL maintain interoperability across manufacturers and operators

---

## 5. Vertiport Infrastructure Standards

### 5.1 Site Selection Criteria

Vertiport sites SHALL meet the following minimum criteria:

**Airspace Access:**
- Clear approach/departure paths with obstacle clearance per ICAO Annex 14
- Coordination with local air traffic control
- Compliance with airspace classifications and restrictions

**Ground Access:**
- Maximum 1-mile from public transportation OR dedicated ground transport
- Pedestrian access with ADA-compliant pathways
- Vehicle pickup/dropoff areas

**Physical Requirements:**
- Minimum area: 5,000 sq ft per landing pad
- Structural capacity: 2.5x maximum aircraft weight
- Utilities: Electrical service supporting megawatt-scale charging

**Environmental:**
- Noise impact assessment showing <65 dB at property boundaries during normal operations
- Community engagement and acceptance

### 5.2 Airside Design Standards

**TLOF (Touchdown and Liftoff Area):**
- Diameter: Minimum 1.0x aircraft overall dimension
- Load capacity: 2.5x maximum aircraft gross weight with safety factor
- Surface: Non-slip, rapid drainage, clearly marked

**FATO (Final Approach and Takeoff Area):**
- Diameter: Minimum 1.5x TLOF diameter
- Obstacle clearance: Slopes per ICAO Annex 14 Section 3.5

**Pad Spacing:**
- Simultaneous operations: Minimum 1.5x FATO diameter between active pads
- Parking positions: May use reduced spacing with safety analysis

### 5.3 Charging Infrastructure

**Power Requirements:**
- Minimum 500 kW per pad for initial operations
- Scalable to 2 MW per pad for rapid turnaround (<15 minutes)
- Grid connection: Transmission or sub-transmission voltage with redundancy

**Charging Technology:**
- Conductive or wireless/inductive charging
- Automated connection/activation
- Thermal management and safety interlocks
- Compliance with aviation electrical standards

**Energy Storage:**
- Optional battery energy storage systems for peak shaving and grid services
- Backup power for critical operations

### 5.4 Passenger Facilities

**Check-in and Waiting:**
- Mobile app-based check-in with biometric verification
- Waiting areas: Minimum 15 sq ft per peak passenger
- Restrooms, Wi-Fi, device charging

**Boarding:**
- Direct aircraft access minimizing weather exposure
- Accessible boarding for mobility-impaired passengers
- Baggage handling for carry-on sized items

**Ground Transportation Integration:**
- Dedicated pickup/dropoff zones
- Bicycle and micro-mobility parking
- Real-time transit information

---

## 6. eVTOL Aircraft Standards

### 6.1 Configuration Types

This standard recognizes three primary eVTOL configurations:

**Multicopter:** Multiple fixed rotors (8-18) operating continuously in all flight phases
- Advantages: Mechanical simplicity, high redundancy
- Applications: Short-range urban operations (<30 miles)

**Lift + Cruise:** Separate lift and cruise propulsion systems
- Advantages: Cruise efficiency, optimized systems
- Applications: Medium-range operations (50-150 miles)

**Vectored Thrust:** Tilting propulsion units or wings
- Advantages: High cruise efficiency and speed
- Applications: Longer-range operations (100-200 miles)

### 6.2 Propulsion Requirements

**Electric Motors:**
- Power density: Minimum 5 kW/kg
- Efficiency: Minimum 95% at cruise conditions
- Redundancy: No single motor failure prevents safe landing

**Battery Systems:**
- Energy density: Minimum 200 Wh/kg at pack level (initial); 300+ Wh/kg recommended
- Discharge capability: Minimum 2C continuous
- Cycle life: Minimum 500 cycles to 80% capacity
- Thermal management: Active cooling maintaining 20-40°C optimal range
- Safety: Thermal runaway protection, crash resistance, venting systems

### 6.3 Flight Control Systems

**Architecture:**
- Fly-by-wire with no mechanical backup
- Triple or quadruple redundant flight computers
- Dissimilar redundancy recommended for critical functions

**Capabilities:**
- Stability augmentation across flight envelope
- Carefree handling preventing dangerous conditions
- Autonomous modes per operator requirements and UML level

**Human-Machine Interface:**
- Intuitive controls reducing pilot workload
- Clear status information and alerts
- Degraded mode operations with failures

### 6.4 Safety Requirements

**Catastrophic Failure Rate:**
- Target: <10⁻⁹ per flight hour (less than 1 in 1 billion hours)
- Analysis: Quantitative safety assessment per SAE ARP 4761

**Redundancy:**
- No single failure prevents continued safe flight and landing
- Propulsion: Ability to complete flight with multiple motor/battery failures
- Flight controls: Triple redundant with automatic reconfiguration
- Power distribution: Independent paths preventing common-mode failures

---

## 7. Air Traffic Management (UTM)

### 7.1 UTM System Architecture

**Core Components:**
- Flight Information Management System (FIMS)
- UTM Service Suppliers (USS)
- Supplemental Data Service Providers (SDSP)
- Public Safety USS for emergency operations

**Communication:**
- Cellular (4G/5G) primary
- Satellite backup
- Dedicated aviation spectrum for safety-critical functions

### 7.2 Operational Concepts by UML

**UML-1 (Nascent):** 1-2 aircraft/vertiport, VFR, pilot-centric, traditional ATC

**UML-2 (Limited):** 2-10 aircraft/vertiport, some IFR, basic UTM tracking

**UML-3 (Initial UTM):** 10-30 aircraft/vertiport, core UTM services, conflict detection

**UML-4 (Mature UTM):** 30-60 aircraft/vertiport, automated tactical resolution, V2V communication

**UML-5 (Advanced Autonomy):** 60-100+ aircraft/vertiport, supervised autonomy, predictive systems

**UML-6 (Full Autonomy):** 100+ aircraft/vertiport, coordinated autonomy, system-level oversight

### 7.3 Conflict Management

**Strategic Deconfliction:**
- Flight planning analysis before departure
- Conflict resolution required for authorization
- Coordination between USS providers

**Tactical Resolution:**
- Real-time monitoring and prediction
- Automated or semi-automated resolution
- 2-10 minute lookahead typical

**Collision Avoidance:**
- Aircraft-based detect-and-avoid systems
- Independent of UTM (defense-in-depth)
- Automatic avoidance maneuvers if necessary

---

## 8. Regulatory Compliance

### 8.1 Aircraft Certification

**Type Certification Basis:**
- FAA Part 23/27/29 or EASA CS-23/27 with Special Conditions
- Special Conditions addressing novel aspects (electric propulsion, distributed motors, etc.)

**Testing Requirements:**
- Ground testing: Structures, systems, environmental
- Flight testing: 500-2,000+ hours depending on novelty
- Production approval for manufacturing quality

### 8.2 Operational Certification

**Air Carrier Operations:**
- FAA Part 135 or equivalent
- Operations manual, training program, maintenance program
- Safety Management System (SMS)

**Pilot Requirements:**
- Commercial pilot license with instrument rating
- Type rating for specific aircraft
- Medical certification (2nd class minimum)
- Recurrent training per operator program

### 8.3 Vertiport Certification

**Design Approval:**
- Compliance with Section 5 standards
- Structural certification
- Environmental review and approval

**Operations Approval:**
- Operational procedures manual
- Emergency response procedures
- Coordination with local authorities

---

## 9. Passenger Experience Standards

### 9.1 Journey Time Requirements

**Target Pre-Flight Time:** 10-20 minutes from vertiport arrival to departure

**Process Components:**
- Check-in: <2 minutes (returning passengers), <5 minutes (first-time)
- Security: Minimal for domestic point-to-point operations
- Waiting: 5-15 minutes typical
- Boarding: 3-5 minutes

### 9.2 Cabin Standards

**Seating:**
- Legroom comparable to business class (36-40 inches)
- 4-6 passengers typical capacity

**Environment:**
- Cabin noise: Maximum 70 dB during cruise
- Climate control: 20-25°C maintained
- Large windows for exterior views

**Connectivity:**
- USB charging at each seat
- Wi-Fi connectivity recommended

### 9.3 Accessibility

**Universal Design:**
- Wheelchair-accessible vertiport facilities
- Boarding assistance procedures
- Visual, auditory, and tactile wayfinding
- Service animal accommodation

---

## 10. Environmental Standards

### 10.1 Emissions

**Operational Emissions:**
- Zero direct emissions (all-electric operations)
- Renewable energy for charging encouraged

**Lifecycle Emissions:**
- Lifecycle assessment recommended
- Battery recycling program required

### 10.2 Noise

**Aircraft Noise:**
- Maximum 65 dB at 500 feet distance during normal operations
- Noise testing per ICAO Annex 16

**Vertiport Operations:**
- Maximum 65 dB at property boundaries during normal operations
- Noise monitoring and community reporting

---

## 11. Conformance and Certification

### 11.1 Conformance Levels

**Bronze:** Basic compliance with core safety requirements
- Aircraft type certified to applicable standards
- Basic vertiport infrastructure (single pad minimum)
- UML-1 or UML-2 operations
- Part 135 or equivalent operational certification

**Silver:** Enhanced compliance with full standard
- All Bronze requirements
- Multi-pad vertiport with passenger amenities
- UML-3 or UML-4 operations capability
- Advanced safety management and data analytics

**Gold:** Complete compliance with aspirational goals
- All Silver requirements
- High-capacity vertiport (4+ pads) with multimodal integration
- UML-5 or UML-6 readiness
- Zero-emission energy sources
- Comprehensive community integration and equitable access programs

### 11.2 Certification Process

Organizations seeking WIA-SPACE-018 certification SHALL:

1. Submit application with conformance documentation
2. Undergo technical review by WIA assessment team
3. Demonstrate operational capability through testing/demonstration
4. Receive certification decision and level designation
5. Maintain compliance through annual audits

---

## 12. Implementation Timeline

### Phase 1 (2024-2027): Initial Operations
- UML-1/UML-2 operations
- Select cities and routes
- Bronze certification sufficient
- Premium pricing, limited market

### Phase 2 (2028-2032): Growth
- UML-3/UML-4 operations
- 30-50 cities globally
- Silver certification recommended
- Expanding market segments

### Phase 3 (2033-2040): Mainstream
- UML-4/UML-5 operations
- 100+ cities globally
- Gold certification for competitive advantage
- Mass market viability

### Phase 4 (2041-2050): Maturity
- UML-5/UML-6 operations
- Global networks
- Universal access and affordability

---

## Appendix A: 弘益人間 (Hongik Ingan) - Benefit All Humanity

The principle of 弘益人間, meaning "widely benefiting all people," SHALL guide UAM development and deployment. Stakeholders SHALL consider:

**Equitable Access:**
- Progressive pricing making UAM accessible across income levels
- Public-private partnerships serving underserved communities
- Integration with public transportation extending benefits

**Environmental Justice:**
- Noise and visual impacts distributed equitably
- Zero-emission operations improving air quality for all
- Climate benefits through reduced ground congestion

**Economic Opportunity:**
- High-quality job creation in manufacturing, operations, and services
- Local workforce development and training
- Economic development in underserved areas

**Safety and Security:**
- Highest safety standards protecting all passengers and communities
- Emergency medical services benefiting all regardless of ability to pay
- Disaster response and evacuation capabilities

UAM success SHALL be measured not only by market size but by breadth of access and community benefit.

---

## Document Control

**Version:** 1.0
**Effective Date:** 2025-01-15
**Review Cycle:** Annual
**Next Review:** 2026-01-15

**Maintained by:** WIA Technical Committee on Advanced Air Mobility
**Contact:** uam-standards@wia.org

**弘益人間 · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA - World Certification Industry Association
