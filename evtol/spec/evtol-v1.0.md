# WIA-SPACE-019: eVTOL Technology Standard v1.0

## 弘益人間 - Benefit All Humanity

---

## Executive Summary

The WIA-SPACE-019 eVTOL (Electric Vertical Takeoff and Landing) standard provides a comprehensive framework for the development, certification, and deployment of electric vertical takeoff and landing aircraft. This standard addresses all aspects of eVTOL technology from propulsion systems and airframe design to autonomous flight controls, certification requirements, and commercial operations.

**Version:** 1.0
**Release Date:** January 2025
**Status:** Published
**Maintainer:** WIA (World Certification Industry Association)

---

## 1. Scope and Purpose

### 1.1 Scope

This standard applies to:
- Electric vertical takeoff and landing aircraft for passenger and cargo transport
- Propulsion systems including electric motors, batteries, and power management
- Autonomous and piloted flight control systems
- Manufacturing processes and quality assurance
- Certification and regulatory compliance
- Operational procedures and infrastructure

### 1.2 Purpose

弘益人間 (Benefit All Humanity) - This standard aims to:
- Establish common technical requirements for eVTOL aircraft
- Promote safety, sustainability, and accessibility
- Enable interoperability between manufacturers and operators
- Support regulatory harmonization globally
- Advance technology while protecting communities and environment

---

## 2. Normative References

- **FAA Part 23:** Airworthiness Standards for Normal Category Airplanes
- **EASA SC-VTOL:** Special Condition for VTOL Aircraft
- **DO-178C:** Software Considerations in Airborne Systems
- **DO-254:** Design Assurance Guidance for Airborne Electronic Hardware
- **AS9100:** Quality Management Systems for Aviation
- **ISO 9001:** Quality Management Systems
- **IEC 62304:** Medical Device Software Lifecycle Processes (adapted for aviation)

---

## 3. Aircraft Configurations

### 3.1 Configuration Types

#### 3.1.1 Multicopter
- **Definition:** Multiple fixed rotors providing lift and control
- **Example:** Volocopter 2X, EHang 216
- **Characteristics:**
  - Simple mechanical design
  - High redundancy through distributed propulsion
  - Limited cruise efficiency
  - Typical range: 15-30 miles

#### 3.1.2 Lift+Cruise
- **Definition:** Separate propulsion for vertical lift and horizontal cruise
- **Example:** Joby S4, Archer Midnight
- **Characteristics:**
  - Moderate mechanical complexity
  - Good cruise efficiency
  - Transition phase requires sophisticated control
  - Typical range: 60-150 miles

#### 3.1.3 Tilt-Wing
- **Definition:** Entire wing structure tilts from vertical to horizontal
- **Example:** Lilium Jet
- **Characteristics:**
  - Complex mechanical and control systems
  - Excellent cruise efficiency
  - Challenging transition dynamics
  - Typical range: 100-200 miles

#### 3.1.4 Tilt-Rotor
- **Definition:** Rotors tilt from vertical to horizontal configuration
- **Example:** Wisk Cora
- **Characteristics:**
  - Proven concept from V-22 Osprey
  - High cruise efficiency
  - Mechanical complexity
  - Typical range: 150-300 miles

### 3.2 Configuration Selection Criteria

Selection based on mission requirements:
- **Urban Air Mobility (short trips):** Multicopter or Lift+Cruise
- **Regional Connectivity:** Lift+Cruise or Tilt-Wing
- **Long-Range Operations:** Tilt-Wing or Tilt-Rotor
- **Autonomous Operations:** Any configuration with appropriate sensors and controls

---

## 4. Propulsion System Requirements

### 4.1 Electric Motors

#### 4.1.1 Specifications
- **Type:** Brushless Permanent Magnet Synchronous Motors (PMSM)
- **Efficiency:** Minimum 93% at rated power
- **Power Density:** Minimum 4 kW/kg
- **Cooling:** Active liquid or forced-air cooling required
- **Redundancy:** Each motor operates independently
- **Operating Voltage:** 400-800V DC typical

#### 4.1.2 Testing Requirements
- Continuous operation at rated power for 30 minutes minimum
- Peak power operation (110% rated) for 5 minutes
- Thermal cycling validation
- Vibration testing per DO-160
- Electromagnetic compatibility (EMC) per DO-160

### 4.2 Battery Systems

#### 4.2.1 Energy Density
- **Minimum:** 180 Wh/kg at pack level
- **Target:** 220-250 Wh/kg
- **Future (solid-state):** 400+ Wh/kg

#### 4.2.2 Power Density
- Capable of 3C continuous discharge
- Peak discharge 5C for takeoff/landing (2 minutes)

#### 4.2.3 Safety Requirements
- Thermal runaway propagation barriers between cells
- Battery Management System (BMS) monitoring all cells
- Dual-redundant BMS architecture
- Automatic fire suppression system
- Crash-resistant housing with energy absorption

#### 4.2.4 Cycle Life
- Minimum 2,000 cycles to 80% capacity
- Calendar life: 10 years minimum
- Operating temperature: -20°C to 60°C

### 4.3 Power Management

- High-voltage DC architecture (400-800V)
- Redundant power distribution buses
- Motor controllers (inverters) with 98%+ efficiency
- Emergency power reserves: 20% minimum battery capacity
- Battery state monitoring at 10 Hz minimum

---

## 5. Structural Requirements

### 5.1 Materials

#### 5.1.1 Primary Structure
- Carbon fiber composite with 60% fiber volume fraction minimum
- Ultimate tensile strength: 3,000 MPa minimum
- Modulus of elasticity: 200 GPa minimum
- Quality assurance per AS9100

#### 5.1.2 Secondary Structure
- Aluminum alloys (7075-T6 or equivalent)
- Titanium alloys for high-stress components
- Aramid fiber for impact protection

### 5.2 Load Factors

- **Limit Load Factor:** +3.8g / -1.5g minimum
- **Ultimate Load Factor:** 1.5 x Limit Load
- **Gust Loads:** Per FAA Part 23 requirements
- **Emergency Landing:** 16g forward, 12g vertical (seats and restraints)

### 5.3 Crashworthiness

- Energy-absorbing landing gear
- Crushable structure zones
- Fire-resistant materials in cabin
- Emergency egress: All occupants exit within 90 seconds

---

## 6. Autonomous Flight Systems

### 6.1 Sensor Requirements

#### 6.1.1 Navigation Sensors
- **GPS/GNSS:** Triple-redundant with RTK capability
- **IMU:** 6 independent units minimum
- **Air Data:** 6 independent pitot-static systems
- **Magnetometer:** Triple-redundant

#### 6.1.2 Perception Sensors
- **Radar:** 360° coverage, 2 km range minimum
- **LiDAR:** 3 units, 300m range minimum
- **Cameras:** 12 cameras, 360° visual coverage
- **Ultrasonic:** 8 sensors for proximity detection

### 6.2 Flight Control Architecture

- Triple-redundant flight computers
- Diverse software implementations
- 100 Hz control loop minimum
- Automatic failure detection and isolation
- Envelope protection (prevent departure from safe flight regime)

### 6.3 Detect and Avoid

- ADS-B In/Out for cooperative aircraft
- Non-cooperative detect and avoid using radar, LiDAR, cameras
- Collision avoidance maneuvers within 5 seconds of detection
- Minimum separation: 500 ft horizontal, 250 ft vertical

### 6.4 Software Certification

- Flight-critical software: DO-178C Design Assurance Level A (DAL-A)
- Mission-critical software: DAL-B
- Non-critical software: DAL-C or lower
- Formal verification for safety-critical algorithms

---

## 7. Noise and Environmental Standards

### 7.1 Noise Limits

- **Maximum Noise Level:** 70 dB at 500 feet AGL
- **Community Noise:** 65 dB maximum at property boundaries
- **Frequency Content:** Minimize low-frequency (< 200 Hz) components
- **Measurement:** Per FAA Part 36 methodology adapted for eVTOL

### 7.2 Emissions

- **Direct Emissions:** Zero during flight (all-electric)
- **Lifecycle Emissions:** 80% reduction vs. equivalent helicopter
- **Grid Carbon Intensity:** Report emissions based on electricity source
- **Battery Recycling:** 95% material recovery target

### 7.3 Sustainability Requirements

- Ethical sourcing of battery materials (lithium, cobalt, nickel)
- Labor practices audit for entire supply chain
- Renewable energy for manufacturing preferred
- End-of-life aircraft recycling plan

---

## 8. Certification Framework

### 8.1 Type Certification

Follow FAA Part 23 with Special Conditions for:
- Powered-lift category airworthiness
- Electric propulsion systems
- Distributed propulsion
- Fly-by-wire flight controls
- Transition flight regime

### 8.2 Production Certification

- AS9100 or equivalent quality management system
- Production flight testing: 100% of aircraft
- Supplier qualification and auditing
- Configuration management
- Non-conformance tracking and resolution

### 8.3 Operational Certification

- Part 135 Air Carrier certificate (US)
- Equivalent EASA AOC (Europe)
- Pilot training programs and type ratings
- Maintenance programs and intervals
- Safety Management System (SMS)

---

## 9. Infrastructure Standards

### 9.1 Vertiport Requirements

#### 9.1.1 Landing Pad
- Minimum dimensions: 1.5 x rotor diameter
- Load capacity: 1.5 x maximum takeoff weight
- Surface: Non-skid, fire-resistant
- Lighting: Per heliport standards (FAA AC 150/5390-2C)

#### 9.1.2 Charging Infrastructure
- **Power:** 150-400 kW per charging station
- **Voltage:** Compatible with aircraft (400-800V DC)
- **Connectors:** Standardized (CCS or CHAdeMO adapted for aviation)
- **Charging Time:** 80% charge in 15-30 minutes target

#### 9.1.3 Passenger Facilities
- Weather protection
- Fire suppression systems
- Emergency medical equipment
- Accessibility (ADA compliant)

### 9.2 Air Traffic Management

- Integration with existing ATC systems
- UAS Traffic Management (UTM) for low-altitude operations
- Automated separation assurance
- Geofencing and airspace restrictions
- Weather information systems

---

## 10. Operational Procedures

### 10.1 Preflight Procedures

- Battery state of charge verification (minimum 20% reserve + mission)
- Motor and propeller inspection
- Flight control system test
- Weather assessment and go/no-go decision
- Weight and balance calculation

### 10.2 Normal Operations

- Takeoff: Vertical climb to 100 ft AGL minimum before transition
- Cruise: Maintain minimum 500 ft AGL over populated areas
- Landing: Approach to 100 ft AGL before transition to hover
- Emergency procedures for all failure modes

### 10.3 Maintenance Requirements

- **Pre-flight:** Visual inspection, system status checks
- **Post-flight:** Flight log review, defect reporting
- **25 flight hours:** Motor inspection, battery health check
- **100 flight hours:** Detailed inspection, calibration verification
- **1000 flight hours:** Major inspection, component replacements

---

## 11. 弘益人間 Implementation Guidelines

### 11.1 Accessibility

- Pricing strategy must target mass market (< $5 per passenger mile)
- Service areas include underserved communities, not only affluent areas
- Accessibility accommodations for passengers with disabilities
- Multi-language support in all user interfaces

### 11.2 Community Engagement

- Public notification before new service areas
- Community noise monitoring and reporting
- Regular public safety briefings
- Transparent incident reporting

### 11.3 Environmental Stewardship

- Renewable energy charging infrastructure priority
- Battery lifecycle management and recycling
- Noise abatement procedures
- Wildlife protection measures

### 11.4 Workforce Development

- Local hiring and training programs
- Apprenticeships and technical education partnerships
- Fair labor practices throughout supply chain
- Diversity and inclusion in workforce

---

## 12. Compliance and Certification

### 12.1 Self-Declaration

Manufacturers may self-declare compliance for non-safety-critical aspects:
- User interface design
- Interior aesthetics
- Non-structural components

### 12.2 Third-Party Verification

Independent verification required for:
- Safety-critical systems (propulsion, flight controls, structures)
- Software (DO-178C DAL-A)
- Quality management systems (AS9100)
- Environmental claims

### 12.3 Continuing Airworthiness

- Service bulletin compliance
- Airworthiness directive (AD) compliance
- Fleet-wide monitoring for trends
- Safety reporting (incidents, accidents, near-misses)

---

## 13. Future Developments

### 13.1 Technology Roadmap

- **2025-2027:** Initial certification and service entry
- **2027-2030:** Autonomous operations, improved batteries (solid-state)
- **2030-2035:** Long-range operations (300+ miles), hydrogen fuel cells
- **2035+:** Full urban air mobility networks, air taxi infrastructure

### 13.2 Standard Evolution

This standard will be updated regularly to incorporate:
- Lessons learned from operational experience
- Technological advancements
- Regulatory developments
- Community feedback

**Next Update:** v1.1 planned for Q4 2026

---

## 14. Definitions and Acronyms

**eVTOL:** Electric Vertical Takeoff and Landing
**PMSM:** Permanent Magnet Synchronous Motor
**BMS:** Battery Management System
**IMU:** Inertial Measurement Unit
**RTK:** Real-Time Kinematic (GPS)
**ADS-B:** Automatic Dependent Surveillance-Broadcast
**DAL:** Design Assurance Level
**UAM:** Urban Air Mobility
**UTM:** UAS Traffic Management
**弘益人間:** Hongik Ingan - Benefit All Humanity

---

## 15. Contact and Support

**Website:** https://wia.org/standards/SPACE-019
**Email:** evtol-standard@wia.org
**GitHub:** https://github.com/WIA-Official/wia-standards/tree/main/standards/evtol

---

**弘益人間** - This standard exists to benefit all humanity through safe, sustainable, accessible air transportation.

© 2025 SmileStory Inc. / WIA
Licensed under Creative Commons Attribution 4.0 International (CC BY 4.0)
