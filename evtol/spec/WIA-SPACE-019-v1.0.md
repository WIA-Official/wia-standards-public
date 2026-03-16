# WIA-SPACE-019: Electric Vertical Take-Off and Landing (eVTOL) Standard

**Version:** 1.0
**Status:** Published
**Date:** 2025-01-26
**Category:** Space Transportation / Urban Aviation / eVTOL Technology

---

## Abstract

WIA-SPACE-019 defines the comprehensive standard for Electric Vertical Take-Off and Landing (eVTOL) aircraft, covering design configurations, electric propulsion systems, flight control, safety systems, airworthiness certification, and future development. This standard provides technical specifications and best practices for manufacturers, operators, regulators, and researchers in the emerging eVTOL industry.

## 1. Scope

This standard applies to:
- eVTOL aircraft design and specifications (multicopter, tilt-rotor, lift+cruise, vectored thrust)
- Electric propulsion systems (motors, batteries, power electronics)
- Flight control systems (fly-by-wire, autonomous flight, AI-based control)
- Safety systems (redundancy, emergency landing, ballistic parachutes)
- Airworthiness certification (EASA SC-VTOL, FAA Part 23, global standards)
- Manufacturer specifications and industry ecosystem
- Future technology roadmap and market development

## 2. Normative References

- FAA Part 23 (Small Aircraft Certification)
- FAA Special Conditions for Powered Lift
- EASA Special Condition VTOL (SC-VTOL)
- ICAO Annex 16 (Environmental Protection - Aircraft Noise)
- ISO 9001 (Quality Management Systems)
- AS9100 (Aerospace Quality Management)
- IEEE 1547 (Distributed Energy Resources)

## 3. Terms and Definitions

### 3.1 eVTOL (electric Vertical Take-Off and Landing)
Aircraft powered by electric motors that can take off and land vertically without requiring a runway.

### 3.2 Distributed Electric Propulsion (DEP)
Use of multiple electric motors distributed across the aircraft for redundancy, efficiency, and safety.

### 3.3 Multicopter
eVTOL configuration using multiple fixed rotors (4-36) for thrust generation and control.

### 3.4 Tilt-Rotor
eVTOL configuration where rotors tilt between vertical (hover) and horizontal (cruise) orientations.

### 3.5 Lift+Cruise
eVTOL configuration with separate vertical lift rotors and forward cruise propellers.

### 3.6 Fly-By-Wire (FBW)
Flight control system where pilot inputs are converted to electronic signals processed by computers that control actuators.

### 3.7 Battery Management System (BMS)
Electronic system that monitors and controls battery charging, discharging, and safety.

### 3.8 Ballistic Recovery System (BRS)
Whole-aircraft parachute system deployed via rocket to lower aircraft safely in emergency.

## 4. eVTOL Design Configurations

### 4.1 Multicopter

#### 4.1.1 Configuration
- **Rotor Count:** 4-36 fixed rotors
- **Control:** Individual rotor RPM adjustment
- **Advantages:** Simple design, excellent hover, high redundancy
- **Limitations:** Lower cruise speed (80-130 km/h), shorter range (30-50 km)
- **Applications:** Short urban trips, tourism, emergency medical

#### 4.1.2 Performance Requirements
- Minimum hover efficiency: 5-7 kg/kW
- Disk loading: 50-150 N/m²
- Noise: <65 dB @ 100m during takeoff/landing

### 4.2 Tilt-Rotor

#### 4.2.1 Configuration
- **Rotor Count:** 2-8 tilting rotors with wings
- **Transition:** Gradual tilt from vertical to horizontal (15-45 seconds)
- **Advantages:** High cruise speed (200-350 km/h), long range (150-300 km)
- **Limitations:** Complex mechanics, difficult transition
- **Applications:** Airport shuttles, intercity routes, premium air taxi

#### 4.2.2 Performance Requirements
- Cruise speed: Minimum 200 km/h
- Transition altitude: 50-150m AGL
- Wing loading: 150-300 kg/m²

### 4.3 Lift+Cruise

#### 4.3.1 Configuration
- **Lift System:** 8-16 vertical rotors
- **Cruise System:** 2-6 forward propellers
- **Advantages:** Optimized for each flight phase
- **Limitations:** Additional weight, complexity
- **Applications:** Medium-range urban aviation, cargo

#### 4.3.2 Performance Requirements
- Combined efficiency: >80% in cruise
- Power distribution flexibility: <3 seconds mode transition
- Cruise speed: 180-280 km/h

## 5. Electric Propulsion Systems

### 5.1 Electric Motors

#### 5.1.1 Motor Types
- **Brushless DC (BLDC):** Primary choice - 90-95% efficiency
- **Permanent Magnet Synchronous Motor (PMSM):** High precision applications
- **Power Density:** Target 10-20 kW/kg
- **Efficiency:** Minimum 90% at rated power, 85%+ at partial load

#### 5.1.2 Motor Requirements
- **Redundancy:** N+1 or N+2 configuration
- **Cooling:** Liquid cooling for motors >50 kW
- **Control:** Independent ESC for each motor
- **Monitoring:** Real-time temperature, current, RPM, vibration
- **Lifespan:** Minimum 10,000 hours

### 5.2 Battery Systems

#### 5.2.1 Current Technology (2025-2027)
- **Chemistry:** Lithium-ion NMC or NCA
- **Energy Density:** 250-300 Wh/kg (pack level)
- **Power Density:** Minimum 1,000 W/kg (5C discharge)
- **Cycle Life:** Minimum 3,000 cycles to 80% capacity
- **Safety:** Pass nail penetration, crush, thermal runaway tests

#### 5.2.2 Near-Term Technology (2028-2030)
- **Chemistry:** High-energy NMC, early solid-state
- **Energy Density:** 350-400 Wh/kg
- **Charging:** 80% in 15-30 minutes
- **Operating Temperature:** -20°C to +60°C

#### 5.2.3 Future Technology (2031+)
- **Solid-State:** 400-500 Wh/kg, zero fire risk
- **Lithium-Sulfur:** 600-800 Wh/kg
- **Lithium-Air:** 1,000+ Wh/kg (long-term)

### 5.3 Battery Management System (BMS)

#### 5.3.1 Core Functions
- **Cell Monitoring:** Individual cell voltage, temperature, current
- **SOC Estimation:** ±3% accuracy
- **SOH Estimation:** Remaining useful life prediction
- **Protection:** Overcharge, over-discharge, overcurrent, thermal protection
- **Balancing:** Active or passive cell balancing

#### 5.3.2 Safety Requirements
- **Thermal Isolation:** Ceramic/aerogel between cells
- **Fire Suppression:** Inert gas or coolant injection system
- **Multiple Fuses:** Module and pack level overcurrent protection
- **Real-Time Monitoring:** Temperature, voltage, swelling detection

### 5.4 Power Electronics

#### 5.4.1 Inverters
- **Topology:** Three-phase bridge inverter
- **Semiconductors:** SiC or GaN preferred for >95% efficiency
- **Power Density:** Target 15-20 kW/kg
- **Cooling:** Integrated with motor cooling loop

#### 5.4.2 DC-DC Converters
- **Input Voltage:** 400-800V (battery pack)
- **Output Voltages:** 12V, 28V, 48V (avionics)
- **Efficiency:** Minimum 92%
- **Redundancy:** Dual converters for critical loads

## 6. Flight Control Systems

### 6.1 Fly-By-Wire (FBW)

#### 6.1.1 Architecture
- **Redundancy:** Dual or triple flight control computers
- **Cross-Check:** Majority voting for fault detection
- **Diversity:** Different processor manufacturers to prevent common-mode failures
- **Update Rate:** Minimum 100 Hz control loop

#### 6.1.2 Flight Control Modes
- **Attitude Hold:** Maintain orientation when controls released
- **Position Hold:** GPS-based hover at fixed location
- **Altitude Hold:** Barometric altitude maintenance
- **Heading Hold:** Magnetic compass direction holding
- **Velocity Control:** Commanded speed achievement
- **Path Following:** Waypoint navigation

### 6.2 Sensor Suite

#### 6.2.1 Required Sensors
- **IMU:** 6-axis (3-axis accelerometer + 3-axis gyroscope), dual redundant
- **GPS/GNSS:** Dual receiver, RTK capability for precision
- **Barometer:** Dual altimeter, static pressure compensation
- **Magnetometer:** Dual compass, calibration capability
- **Airspeed:** Pitot-static system for forward flight

#### 6.2.2 Autonomous Flight Sensors
- **LiDAR:** 360-degree scanning, 100m+ range
- **Cameras:** Stereo vision, object detection (AI-based)
- **Radar:** Weather radar, collision avoidance
- **ADS-B:** Traffic awareness, TCAS functionality
- **Ultrasonic:** Ground proximity during landing

### 6.3 Autonomous Flight Levels

#### 6.3.1 Level Classification
- **Level 0:** Fully manual, basic stabilization
- **Level 1:** Assisted automation (altitude/speed hold)
- **Level 2:** Partial automation (auto takeoff/landing)
- **Level 3:** Conditional automation (specific conditions only)
- **Level 4:** High automation (pilot optional, ground monitoring)
- **Level 5:** Full automation (no human intervention)

#### 6.3.2 Requirements by Level
- **Level 2-3:** Pilot always present and monitoring
- **Level 4:** Remote pilot/operator monitoring multiple aircraft
- **Level 5:** AI-only operation with system redundancy

## 7. Safety Systems

### 7.1 Redundancy

#### 7.1.1 Propulsion Redundancy
- **Minimum:** N+1 motor failure tolerance
- **Recommended:** N+2 simultaneous motor failures
- **Best Practice:** N/2 configuration (50% motors can fail)

#### 7.1.2 Battery Redundancy
- **Multiple Modules:** 4-12 independent battery modules
- **Isolation:** Electrical and thermal isolation between modules
- **Graceful Degradation:** Continue flight with reduced modules

#### 7.1.3 Flight Control Redundancy
- **Dual/Triple Computers:** 2-3 independent FCC
- **Sensor Redundancy:** All critical sensors duplicated
- **Dissimilar Redundancy:** Different hardware/software architecture

### 7.2 Emergency Systems

#### 7.2.1 Emergency Landing
- **Automatic Site Selection:** AI-based safe landing location identification
- **Controlled Descent:** Optimized thrust distribution with degraded systems
- **Priority Locations:** Vertiports > Airports > Open spaces > Roads > Water

#### 7.2.2 Ballistic Recovery System (BRS)
- **Deployment Altitude:** Minimum 100-150m AGL
- **Descent Rate:** 6-8 m/s with full parachute
- **Parachute Size:** 2-3x aircraft weight capacity
- **Activation:** Manual (pilot) or automatic (system)

### 7.3 Fire Safety

#### 7.3.1 Prevention
- **Thermal Barriers:** Ceramic/aerogel cell isolation
- **Temperature Monitoring:** Real-time cell-level monitoring
- **Current Limiting:** Immediate overcurrent shutoff
- **Pressure Relief:** Safe gas venting when overheated

#### 7.3.2 Suppression
- **Inert Gas:** N₂ or CO₂ injection system
- **Coolant Spray:** Liquid cooling for thermal runaway cells
- **Module Isolation:** Disconnect and isolate affected module
- **Exhaust System:** Toxic gas venting outside aircraft

## 8. Airworthiness Certification

### 8.1 Certification Authorities

#### 8.1.1 FAA (United States)
- **Category:** Powered Lift
- **Basis:** Part 23 + Special Conditions
- **Process:** 5-stage certification (Basis → Compliance → Verification → Approval → TC)
- **Timeline:** 5-7 years average

#### 8.1.2 EASA (Europe)
- **Standard:** SC-VTOL (Special Condition for VTOL)
- **Categories:** Basic (<2 pax) and Enhanced (commercial)
- **Process:** Similar to FAA 5-stage
- **Timeline:** 5-7 years average

#### 8.1.3 CAAC (China)
- **Fast-Track:** Streamlined for domestic industry
- **First Certified:** EHang EH216 (2021)
- **Timeline:** 3-5 years

### 8.2 Testing Requirements

#### 8.2.1 Ground Testing
- **Static Structural:** Ultimate load testing (1.5x limit load)
- **Fatigue:** Lifecycle structural testing (10,000+ cycles)
- **Battery Safety:** Overcharge, nail penetration, crush, fire tests
- **EMC:** Lightning strike, radio interference testing
- **Noise:** Sound level measurement at 100m

#### 8.2.2 Flight Testing
- **Performance:** Speed, climb, range, ceiling verification
- **Handling:** Control response, stability characteristics
- **Transition:** Vertical-to-horizontal mode change (tilt-rotor)
- **Failure Modes:** Motor/battery failure simulation
- **Environmental:** Temperature, wind, precipitation limits

#### 8.2.3 Safety Analysis
- **FHA:** Functional Hazard Assessment
- **FTA:** Fault Tree Analysis
- **FMEA:** Failure Modes and Effects Analysis
- **SSA:** System Safety Assessment
- **Target:** 10⁻⁷ fatal accidents per flight hour

### 8.3 Noise Standards

#### 8.3.1 Requirements
- **Takeoff/Landing:** Maximum 65 dB @ 100m
- **Cruise:** Maximum 60 dB @ 100m
- **Ground Operations:** Maximum 70 dB @ vertiport boundary
- **Night Operations:** Additional 5-10 dB reduction

## 9. Manufacturers and Industry

### 9.1 Leading Manufacturers

#### 9.1.1 United States
- **Joby Aviation:** Tilt-rotor, 320 km/h, FAA Stage 4
- **Archer Aviation:** Lift+cruise, 240 km/h, United Airlines partnership
- **Wisk Aero:** Autonomous lift+cruise, Boeing-backed

#### 9.1.2 Europe
- **Volocopter (Germany):** Multicopter, 110 km/h, EASA certification
- **Lilium (Germany):** Ducted fan, 280 km/h, 7-seater
- **Vertical Aerospace (UK):** Lift+cruise, 320 km/h

#### 9.1.3 Asia
- **EHang (China):** Multicopter, 130 km/h, first certified (CAAC 2021)
- **Hyundai Supernal (Korea):** Tilt-rotor, K-UAM initiative

### 9.2 Market Projections

#### 9.2.1 Timeline
- **2025-2027:** Initial commercialization, 1M passengers/year
- **2028-2030:** Growth phase, 150M passengers/year
- **2031-2035:** Popularization, 1.5B passengers/year
- **2036-2040:** Maturity, 2.3B passengers/year

#### 9.2.2 Market Size
- **Morgan Stanley:** $1.5T by 2040
- **McKinsey:** $500B by 2035
- **ARK Invest:** $800B by 2030

## 10. Future Development

### 10.1 Technology Roadmap

#### 10.1.1 Battery Evolution
- **2025-2027:** Li-ion NMC/NCA, 250-300 Wh/kg
- **2028-2030:** Early solid-state, 350-400 Wh/kg
- **2031-2035:** Solid-state mainstream, 450-500 Wh/kg
- **2036-2040:** Li-S, 600-800 Wh/kg
- **2041+:** Li-Air, 1,000+ Wh/kg

#### 10.1.2 Autonomous Flight
- **2025-2027:** Level 2-3 (pilot required)
- **2028-2030:** Level 3-4 (pilot optional)
- **2031-2035:** Level 4-5 (fully autonomous)
- **2036+:** Level 5 standard, AI-only operations

#### 10.1.3 Alternative Propulsion
- **Hydrogen Fuel Cells:** 2030+ for long-range (500+ km)
- **Hybrid-Electric:** Bridge technology for extended range
- **Advanced Batteries:** Graphene, aluminum-air (research)

### 10.2 Infrastructure Development

#### 10.2.1 Vertiport Network
- **2025:** 50-100 vertiports globally
- **2030:** 1,000+ vertiports in major cities
- **2040:** 50,000+ including suburban areas
- **2050:** 100,000+ ubiquitous coverage

#### 10.2.2 Charging Infrastructure
- **Standard Charging:** 50-150 kW, 1-2 hours
- **Fast Charging:** 500-1,000 kW, 15-30 minutes
- **Battery Swap:** 5-10 minutes (cargo eVTOL)
- **Wireless Charging:** Future automatic charging pads

## 11. Conformance

Systems claiming conformance to WIA-SPACE-019 shall:

1. Implement eVTOL design per Section 4 (multicopter, tilt-rotor, or lift+cruise)
2. Utilize electric propulsion systems per Section 5 (motors, batteries, power electronics)
3. Incorporate fly-by-wire control per Section 6 (redundant computers, sensor fusion)
4. Implement safety systems per Section 7 (redundancy, emergency landing, BRS)
5. Pursue airworthiness certification per Section 8 (FAA, EASA, or equivalent)
6. Document compliance with manufacturer specifications per Section 9
7. Plan for future development per Section 10

## 12. Bibliography

- FAA Engineering Brief No. 105A (eVTOL Design and Airworthiness)
- EASA SC-VTOL Certification Specifications
- NASA eVTOL Aircraft Technology Working Group Reports
- Uber Elevate White Papers
- K-UAM Technology Roadmap (Korea MOLIT)
- Morgan Stanley UAM Market Analysis
- Roland Berger Urban Air Mobility Study
- McKinsey Urban Air Mobility Report

---

## Appendices

### Appendix A: eVTOL Comparison Table

| Manufacturer | Aircraft | Type | Pax | Speed | Range | Status |
|-------------|----------|------|-----|-------|-------|--------|
| Joby Aviation | S4 | Tilt-Rotor | 4+1 | 320 km/h | 240 km | FAA Stage 4 |
| Archer Aviation | Midnight | Lift+Cruise | 4+1 | 240 km/h | 160 km | FAA Early Stage |
| Volocopter | VoloCity | Multicopter | 1+1 | 110 km/h | 35 km | EASA Certification |
| Lilium | Jet | Ducted Fan | 6+1 | 280 km/h | 250 km | EASA Early Stage |
| Wisk Aero | 6th Gen | Lift+Cruise | 4 | 220 km/h | 140 km | FAA Autonomous Path |
| EHang | EH216 | Multicopter | 2 | 130 km/h | 30 km | CAAC Certified (2021) |
| Hyundai | Supernal | Tilt-Rotor | 4-5 | 240+ km/h | 100+ km | Development |
| Vertical | VX4 | Lift+Cruise | 4+1 | 320 km/h | 160 km | FAA Early Stage |

### Appendix B: Battery Technology Comparison

| Technology | Wh/kg | W/kg | Cycle Life | Safety | Timeline |
|-----------|-------|------|-----------|---------|----------|
| Li-ion NMC | 250-300 | 1,000+ | 3,000 | Moderate | Current |
| Li-ion NCA | 260-300 | 1,200+ | 2,500 | Moderate | Current |
| High-Ni NMC | 350-400 | 1,500+ | 2,000 | Moderate | 2028-2030 |
| Solid-State | 400-500 | 2,000+ | 5,000+ | High | 2028-2035 |
| Lithium-Sulfur | 600-800 | 800-1,000 | 500-1,000 | Moderate | 2032+ |
| Lithium-Air | 1,000+ | 500-800 | Unknown | Research | 2040+ |

### Appendix C: Noise Level Comparison

| Source | Noise Level | Distance | Notes |
|--------|-------------|----------|-------|
| Normal Conversation | 60 dB | 1m | Baseline |
| eVTOL (Multicopter) | 60-65 dB | 100m | Takeoff/landing |
| eVTOL (Tilt-Rotor) | 65-70 dB | 100m | Takeoff/landing |
| eVTOL Cruise | 45-60 dB | 100m | All types |
| Helicopter | 90-105 dB | 100m | 30-45 dB louder |
| Jet Aircraft | 120-140 dB | 100m | Commercial |

---

**Standard Maintained By:** WIA Technical Committee
**Contact:** standards@wia-official.org
**License:** Published under 弘益人間 (Benefit All Humanity) philosophy

**Revision History:**
- v1.0 (2025-01-26): Initial publication

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
