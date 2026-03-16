# WIA-SPACE-018: Urban Air Mobility (UAM) Standard

**Version:** 1.0
**Status:** Published
**Date:** 2025-01-26
**Category:** Space Transportation / Urban Aviation

---

## Abstract

WIA-SPACE-018 defines the comprehensive standard for Urban Air Mobility (UAM) systems, operations, and documentation. This standard covers eVTOL aircraft design, propulsion systems, infrastructure (vertiports), air traffic management (UTM), safety and certification, business models, and future development of urban air transportation.

## 1. Scope

This standard applies to:
- eVTOL aircraft design and specifications
- Electric propulsion systems and battery management
- Vertiport infrastructure and charging facilities
- UAM Traffic Management (UTM) systems
- Safety certification and airworthiness standards
- Noise regulations and environmental compliance
- Business models and operational frameworks
- Integration with existing transportation networks

## 2. Normative References

- FAA Part 23 (Small Aircraft Certification)
- FAA Special Conditions for Powered Lift
- EASA Special Condition VTOL
- ICAO Annex 16 (Environmental Protection - Aircraft Noise)
- International Docking System Standard (IDSS)
- UTM Framework and Architecture Documents
- IEEE 1547 (Distributed Energy Resources)

## 3. Terms and Definitions

### 3.1 UAM (Urban Air Mobility)
A transportation system using eVTOL aircraft for moving people and cargo within urban and suburban environments.

### 3.2 eVTOL (electric Vertical Take-Off and Landing)
Aircraft powered by electric motors that can take off and land vertically without requiring a runway.

### 3.3 Vertiport
A facility designed for eVTOL operations including takeoff, landing, passenger boarding, and battery charging.

### 3.4 UTM (UAM Traffic Management)
Digital system managing safe and efficient operation of multiple eVTOL aircraft in urban airspace.

### 3.5 FATO (Final Approach and Take-Off area)
Defined area for final phase of approach to landing and initial phase of takeoff.

### 3.6 Multicopter
eVTOL configuration using multiple fixed rotors for thrust generation and control.

### 3.7 Tilt-Rotor
eVTOL configuration where rotors tilt between vertical (hover) and horizontal (cruise) orientations.

### 3.8 Distributed Electric Propulsion (DEP)
Use of multiple electric motors distributed across the aircraft for redundancy and efficiency.

## 4. eVTOL Aircraft Standards

### 4.1 Aircraft Categories

#### 4.1.1 Multicopter
- **Configuration:** 4-36 fixed rotors
- **Advantages:** Simple design, excellent hover capability
- **Limitations:** Lower cruise speed (80-120 km/h), shorter range
- **Applications:** Short urban hops, tourism

#### 4.1.2 Tilt-Rotor
- **Configuration:** 2-8 tilting rotors with wings
- **Advantages:** High cruise speed (200-300 km/h), longer range
- **Limitations:** Complex transition mechanics
- **Applications:** Airport shuttles, intercity routes

#### 4.1.3 Lift+Cruise
- **Configuration:** Separate lift and cruise propulsors
- **Advantages:** Optimized for each flight phase
- **Limitations:** Additional weight and drag
- **Applications:** Balanced urban operations

### 4.2 Structural Requirements
- **Materials:** Carbon fiber composites (CFRP) preferred
- **Load Factors:** Minimum +3.8g / -1.5g
- **Crashworthiness:** Comply with Part 23 requirements
- **Lightning Protection:** Full aircraft protection required

### 4.3 Performance Requirements
- **Minimum Range:** 25 km (urban) to 250 km (regional)
- **Cruise Speed:** Minimum 100 km/h
- **Ceiling:** Minimum 1,500 m AGL
- **Endurance:** Minimum 30 minutes + 20% reserve

## 5. Propulsion Systems

### 5.1 Electric Motors

#### 5.1.1 Motor Types
- **Brushless DC (BLDC):** Primary choice for most applications
- **Permanent Magnet Synchronous Motor (PMSM):** High precision applications
- **Efficiency:** Minimum 90% at rated power
- **Power Density:** Target 10 kW/kg

#### 5.1.2 Motor Requirements
- **Redundancy:** N+1 or N+2 configuration (one or two motor failure tolerance)
- **Cooling:** Liquid cooling systems for motors >50 kW
- **Control:** Independent control for each motor
- **Monitoring:** Real-time temperature, current, RPM monitoring

### 5.2 Battery Systems

#### 5.2.1 Battery Chemistry
- **Current:** Lithium-ion NMC or NCA (250-300 Wh/kg)
- **Near-term:** High-energy NMC (350-400 Wh/kg)
- **Future:** Solid-state batteries (400-500 Wh/kg)

#### 5.2.2 Battery Requirements
- **Energy Density:** Minimum 250 Wh/kg at pack level
- **Power Density:** Minimum 1,000 W/kg (5C discharge capability)
- **Cycle Life:** Minimum 3,000 cycles to 80% capacity
- **Safety:** Pass nail penetration, crush, and thermal runaway tests
- **Temperature Range:** -20°C to +60°C operation

#### 5.2.3 Battery Management System (BMS)
- **Cell Monitoring:** Individual cell voltage, temperature, current
- **SOC Estimation:** ±5% accuracy
- **SOH Estimation:** Predict remaining useful life
- **Protection:** Overcharge, over-discharge, overcurrent, thermal protection
- **Balancing:** Active or passive cell balancing

### 5.3 Charging Systems
- **Fast Charging:** 80% in 15-30 minutes (500-1,000 kW)
- **Standard Charging:** Full charge in 1-2 hours (50-150 kW)
- **Connector:** Standardized high-power connector (CCS or proprietary)
- **Communication:** CAN bus or similar for charge management

## 6. Vertiport Infrastructure

### 6.1 Vertiport Classification

#### 6.1.1 Vertihub
- **Capacity:** 4+ landing pads
- **Throughput:** 50+ operations/hour
- **Facilities:** Full passenger terminal, maintenance, charging

#### 6.1.2 Vertiport
- **Capacity:** 1-3 landing pads
- **Throughput:** 10-30 operations/hour
- **Facilities:** Basic passenger amenities, charging

#### 6.1.3 Vertistop
- **Capacity:** 1 landing pad
- **Throughput:** <10 operations/hour
- **Facilities:** Minimal - pickup/dropoff only

### 6.2 Design Requirements
- **FATO Size:** Minimum 30m x 30m for 4-passenger eVTOL
- **TLOF Size:** 1.0 x rotor diameter (minimum)
- **Safety Area:** 15m around FATO
- **Clearway:** 1:8 slope for approach/departure

### 6.3 Charging Infrastructure
- **Power Supply:** Minimum 500 kW per pad
- **Charging Stations:** Automated high-power DC charging
- **Energy Storage:** Optional ESS for peak shaving
- **Renewable Integration:** Target 50% renewable energy by 2030

## 7. UAM Traffic Management (UTM)

### 7.1 UTM Architecture
- **Cloud-Based:** Scalable, distributed architecture
- **Real-Time:** <100ms latency for critical functions
- **Availability:** 99.999% uptime requirement

### 7.2 Core Functions
- **Flight Planning:** Automated route generation and optimization
- **Traffic Coordination:** Conflict detection and resolution
- **Weather Integration:** Real-time meteorological data
- **Geofencing:** Automated restricted area avoidance
- **Emergency Management:** Automated emergency landing site identification

### 7.3 Communication Systems
- **Primary:** 4G/5G cellular networks
- **Backup:** Satellite communication
- **Aircraft-to-Aircraft:** ADS-B for collision avoidance
- **Data Rate:** Minimum 1 Mbps uplink/downlink

### 7.4 Safety Protocols
- **Separation Standards:**
  - Horizontal: Minimum 500m
  - Vertical: Minimum 100m
- **Collision Avoidance:** Multi-layer system (strategic, tactical, emergency)
- **Contingency Planning:** Automated safe landing procedures

## 8. Safety and Certification

### 8.1 Airworthiness Certification
- **Type Certification:** FAA/EASA approval required
- **Production Certification:** Quality management system approval
- **Continuing Airworthiness:** Ongoing maintenance and monitoring

### 8.2 Safety Requirements
- **Target Safety Level:** 10^-7 fatal accidents per flight hour
- **Redundancy:** Critical systems must have backup
- **Emergency Systems:** Whole aircraft parachute for catastrophic failures
- **Fire Protection:** Battery fire suppression systems

### 8.3 Noise Standards
- **Maximum Noise:** 65 dB at 100m distance during takeoff
- **Cruise Noise:** 60 dB at 100m distance
- **Community Noise:** Comply with local regulations

### 8.4 Pilot Requirements
- **Licensing:** Powered Lift rating or equivalent
- **Training:** Minimum 40-60 hours eVTOL-specific training
- **Medical:** Class 2 medical certificate minimum
- **Recurrent Training:** Every 6 months

## 9. Operational Standards

### 9.1 Operating Conditions
- **Visual Meteorological Conditions (VMC):**
  - Visibility: Minimum 5 km
  - Cloud Ceiling: Minimum 300m (1,000 ft)
- **Wind Limits:** Maximum 25 kt (46 km/h)
- **Precipitation:** No heavy rain or thunderstorms

### 9.2 Flight Operations
- **Altitude:** 300-1,500m AGL for urban operations
- **Speed:** Maximum 150 kt (278 km/h) in urban corridors
- **Night Operations:** Requires enhanced lighting and navigation systems

### 9.3 Maintenance Requirements
- **Daily:** Pre-flight inspection, system checks
- **Weekly:** Battery health check, motor inspection
- **Monthly:** Comprehensive system test
- **Annual:** Major overhaul, component replacement

## 10. Business Models

### 10.1 Service Categories
- **Air Taxi:** On-demand passenger transport
- **Cargo Delivery:** Medical supplies, urgent parts, e-commerce
- **Emergency Medical:** Air ambulance, organ transport
- **Tourism:** Sightseeing tours, resort shuttles
- **B2B:** Corporate shuttles, infrastructure inspection

### 10.2 Pricing Framework
- **Initial (2025-2027):** $5-8 per mile
- **Growth (2028-2030):** $3-5 per mile
- **Mature (2031+):** $2-4 per mile
- **Target:** Competitive with premium ground transport

## 11. Environmental Compliance

### 11.1 Emissions
- **Direct Emissions:** Zero during operation (electric)
- **Lifecycle Emissions:** Calculate well-to-wheel emissions
- **Carbon Offset:** Renewable energy charging encouraged

### 11.2 Noise Management
- **Time Restrictions:** Possible night flight limitations
- **Route Planning:** Avoid residential areas when possible
- **Monitoring:** Real-time noise measurement at vertiports

## 12. Data and Privacy

### 12.1 Data Collection
- **Flight Data:** Position, speed, altitude, system status
- **Passenger Data:** Booking information, payment details
- **Operational Data:** Weather, traffic, performance metrics

### 12.2 Privacy Protection
- **Encryption:** All data encrypted in transit and at rest
- **Access Control:** Role-based access to sensitive data
- **Compliance:** GDPR, CCPA, and local regulations
- **Retention:** Data retention policies aligned with regulations

## 13. Future Development

### 13.1 Technology Roadmap
- **2025-2027:** Initial commercialization with pilots
- **2028-2030:** Partial autonomy, improved batteries
- **2031-2035:** Full autonomy, solid-state batteries
- **2036-2040:** Hydrogen fuel cells, long-range operations

### 13.2 Advanced Features
- **Autonomous Flight:** Transition from piloted to autonomous
- **Hydrogen Propulsion:** For long-range routes (>500 km)
- **Urban Air Corridors:** Dedicated 3D airspace for UAM
- **MaaS Integration:** Seamless integration with ground transport

## 14. Conformance

Systems claiming conformance to WIA-SPACE-018 shall:
1. Implement eVTOL safety requirements per Section 4-5
2. Provide vertiport infrastructure per Section 6
3. Integrate with UTM systems per Section 7
4. Achieve airworthiness certification per Section 8
5. Meet noise and environmental standards per Section 11
6. Provide comprehensive documentation per Section 13

## 15. Bibliography

- FAA Engineering Brief No. 105 (Vertiport Design)
- NASA UAM Maturity Level Framework
- EASA SC-VTOL Certification Specifications
- Uber Elevate White Papers
- K-UAM Roadmap (Korea Ministry of Land, Infrastructure and Transport)
- Morgan Stanley UAM Market Study
- Roland Berger Urban Air Mobility Study

---

## Appendices

### Appendix A: eVTOL Comparison Table

| Aircraft | Type | Passengers | Speed | Range | Status |
|----------|------|------------|-------|-------|--------|
| Joby Aviation | Tilt-Rotor | 4+1 | 320 km/h | 240 km | FAA Stage 4 |
| Volocopter VoloCity | Multicopter | 1+1 | 110 km/h | 35 km | EASA Cert |
| Lilium Jet | Multicopter (Ducted) | 6+1 | 280 km/h | 250 km | Development |
| Archer Maker | Lift+Cruise | 4+1 | 240 km/h | 160 km | Testing |
| Wisk Aero Cora | Lift+Cruise | 2 | 177 km/h | 40 km | Autonomous |

### Appendix B: Vertiport Design Examples

1. **Rooftop Vertiport:** Urban building integration
2. **Ground-Level Vertihub:** Dedicated facility with terminals
3. **Highway Interchange:** Transport hub integration
4. **Airport Vertiport:** Seamless air-to-air transfer

### Appendix C: UTM Communication Protocols

- **ADS-B:** 1090 MHz Extended Squitter
- **5G V2X:** 5G New Radio for vehicle communication
- **ASTERIX Cat 129:** UAM data exchange format
- **JSON/REST API:** Cloud UTM interface standard

---

**Standard Maintained By:** WIA Technical Committee
**Contact:** standards@wia-official.org
**License:** Published under 弘益人間 (Benefit All Humanity) philosophy

**Revision History:**
- v1.0 (2025-01-26): Initial publication

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
