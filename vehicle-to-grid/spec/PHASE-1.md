# WIA-AUTO-029 PHASE 1: Foundation

## Overview

**Phase:** 1 - Foundation
**Status:** Implementation Ready
**Duration:** 6-12 months
**Complexity:** Moderate
**Dependencies:** None

## Objective

Establish the foundational infrastructure and capabilities for Vehicle-to-Grid (V2G) operations, including basic bidirectional charging, grid connection, and safety systems.

## Technical Requirements

### 1.1 Bidirectional Charging Hardware

#### Onboard Charger (OBC)
- **Power Rating:** 3.3 kW - 22 kW bidirectional
- **Topology:** Full-bridge or three-level inverter
- **Efficiency:** ≥ 95% (both directions)
- **Power Factor:** ≥ 0.95
- **THD:** < 5% at rated power
- **Voltage Range:** 230V AC (single-phase) or 400V AC (three-phase)
- **DC Voltage:** 200V - 800V (battery side)

#### Charging Equipment (EVSE)
- **Connector Type:** Type 2 (IEC 62196) or J1772
- **Communication:** ISO 15118-2 (basic level)
- **Power Metering:** Bidirectional energy measurement (± 0.5% accuracy)
- **Safety:** Ground fault detection, arc fault protection, overcurrent protection
- **Anti-Islanding:** Per IEEE 1547 requirements
- **Response Time:** < 2 seconds for anti-islanding detection

### 1.2 Battery Management System

#### Core Functions
- Cell voltage monitoring (± 10 mV accuracy)
- Temperature monitoring (± 1°C accuracy, minimum 4 sensors per module)
- State of Charge (SoC) estimation (± 3% accuracy)
- State of Health (SoH) estimation (± 5% accuracy)
- Cell balancing (passive or active)
- Thermal management control

#### Safety Limits
- Overcharge protection: Cutoff at 4.2V per cell (NMC/NCA) or 3.65V (LFP)
- Overdischarge protection: Cutoff at 2.5V per cell
- Overtemperature protection: Cutoff at 55°C (charging) or 60°C (discharging)
- Overcurrent protection: 2C charging, 3C discharging (continuous)

#### V2G-Specific Requirements
- Bidirectional power flow support
- Configurable SoC limits for V2G (default: 20% - 80%)
- Cycle counting and degradation tracking
- Communication interface with OBC (CAN bus, 500 kbps minimum)

### 1.3 Communication Protocols

#### ISO 15118-2 (Basic)
- Plug & Charge: Vehicle identification and authentication
- Power delivery control: Charge and discharge power negotiation
- Session management: Connection, charging session, disconnection
- Basic billing information exchange

#### OCPP 1.6 or 2.0.1
- Charger-to-network communication
- Remote start/stop
- Reservation handling
- Transaction management
- Firmware updates

#### Data Exchange
- **Update Rate:** Minimum 1 Hz for power control signals
- **Latency:** < 500 ms end-to-end
- **Reliability:** 99.5% message delivery
- **Security:** TLS 1.2 or higher encryption

### 1.4 Grid Connection Requirements

#### Electrical Interconnection
- Compliance with IEEE 1547-2018 (North America) or VDE-AR-N 4100 (Europe)
- Voltage range: 0.88 - 1.10 pu (per unit)
- Frequency range: 59.3 - 60.5 Hz (60 Hz systems) or 49.0 - 51.0 Hz (50 Hz systems)
- Power factor: 0.95 leading to 0.95 lagging
- Voltage unbalance: < 2% (three-phase systems)

#### Protection Systems
- Undervoltage/overvoltage protection (trip within 2 seconds)
- Underfrequency/overfrequency protection (trip within 0.16 seconds)
- Ground fault detection (30 mA threshold, trip within 0.3 seconds)
- Arc fault detection (5A threshold)
- Reverse power flow protection

#### Metering and Monitoring
- Bidirectional energy meter (IEC 62053 Class 1)
- Real-time power monitoring (±1% accuracy)
- Logging of all charging/discharging sessions
- Remote meter reading capability

### 1.5 Safety and Certification

#### Vehicle Safety
- ISO 6469-3: Electrical safety requirements for traction batteries
- UN ECE R100: Safety requirements for electric vehicles
- SAE J2929: Safety standards for electric vehicle propulsion batteries

#### Charging Safety
- IEC 61851-1: Conductive charging system requirements
- IEC 62196: Connectors and couplers
- UL 2202: Electric vehicle charging system equipment
- UL 2231: Personnel protection systems for EV supply equipment

#### Grid Integration Safety
- IEEE 1547.1: Interconnection conformance testing
- UL 1741: Inverters, converters, and charge controllers
- IEC 62109: Safety of power converters for photovoltaic systems (applicable sections)

### 1.6 User Interface and Controls

#### Vehicle Display
- Current power flow direction and magnitude
- State of charge (SoC) percentage
- Estimated charging/discharging time
- V2G session status
- Revenue earned (current session and cumulative)

#### Mobile Application
- Remote start/stop of V2G session
- SoC and power flow monitoring
- Departure time and target SoC setting
- V2G participation preferences (enable/disable, SoC limits)
- Revenue tracking and history
- Notifications (session complete, grid events, errors)

#### Web Portal
- Detailed session history
- Energy flow visualization
- Revenue reports
- Billing and payment management
- Firmware update notifications

## Implementation Steps

### Step 1: Hardware Installation (Weeks 1-4)
1. Install bidirectional onboard charger (if not factory-equipped)
2. Install/upgrade bidirectional EVSE
3. Upgrade electrical panel if needed (50A+ breaker recommended)
4. Install bidirectional energy meter
5. Verify grounding and bonding
6. Install communication equipment

### Step 2: Software Configuration (Weeks 5-6)
1. Configure BMS for V2G operation
2. Set initial SoC limits (20% - 80% default)
3. Configure ISO 15118 communication
4. Set up OCPP connection to network backend
5. Install mobile application
6. Create user account and preferences

### Step 3: Testing and Commissioning (Weeks 7-8)
1. Perform pre-operational checks
2. Conduct charging functionality test (G2V)
3. Conduct discharging functionality test (V2G)
4. Verify anti-islanding protection
5. Test communication protocols
6. Perform power quality measurements
7. Verify metering accuracy
8. Conduct thermal management test

### Step 4: Grid Interconnection (Weeks 9-12)
1. Submit interconnection application to utility
2. Provide technical documentation
3. Schedule interconnection inspection
4. Utility approval and permission to operate
5. Final commissioning with utility monitoring

## Performance Metrics

### Technical Performance
- **Round-trip Efficiency:** ≥ 85%
- **Availability:** ≥ 95%
- **Power Quality:** THD < 5%, PF > 0.95
- **Response Time:** < 5 seconds to grid signals
- **Communication Uptime:** ≥ 99%

### Safety Performance
- **Fault Detection Time:** < 2 seconds
- **Ground Fault Trip Time:** < 0.3 seconds
- **Anti-Islanding Response:** < 2 seconds
- **Zero Safety Incidents:** Target for first 12 months

### User Experience
- **App Responsiveness:** < 1 second for status updates
- **Session Initiation:** < 10 seconds from plug-in
- **Revenue Display:** Real-time updates (< 5 second delay)

## Success Criteria

- ✅ Bidirectional power flow operational
- ✅ All safety systems functional and tested
- ✅ Grid interconnection approved and active
- ✅ Communication protocols operational
- ✅ User interface functional and responsive
- ✅ Minimum 10 successful V2G sessions completed
- ✅ All performance metrics met
- ✅ No safety incidents or grid violations

## Risk Mitigation

### Technical Risks
- **Battery degradation:** Implement conservative SoC limits (20-80%), moderate C-rates (< 0.5C)
- **Hardware failure:** Use certified equipment, comprehensive testing before deployment
- **Communication failure:** Implement fallback modes, local control capability
- **Grid incompatibility:** Thorough pre-installation site assessment, utility coordination

### Operational Risks
- **User error:** Clear documentation, intuitive user interface, training materials
- **Regulatory changes:** Monitor regulations, flexible system design for updates
- **Interconnection delays:** Early utility engagement, complete documentation

## Cost Estimate

### Hardware Costs
- Bidirectional OBC (if not factory): $3,000 - $6,000
- Bidirectional EVSE: $2,000 - $5,000
- Electrical panel upgrade: $500 - $2,000
- Bidirectional meter: $200 - $500
- Communication equipment: $200 - $500
- **Total Hardware:** $5,900 - $14,000

### Installation Costs
- Electrical work: $1,000 - $3,000
- Permitting: $200 - $500
- Interconnection fees: $100 - $500
- **Total Installation:** $1,300 - $4,000

### Software/Service Costs
- Mobile app subscription: $0 - $10/month
- Communication service: $10 - $30/month
- Platform fees: 10% - 30% of V2G revenue
- **Annual Software/Service:** $120 - $1,500

### Total Phase 1 Investment
- **Initial:** $7,200 - $18,000
- **Annual Operating:** $120 - $1,500

### Expected ROI
- **Annual V2G Revenue:** $2,000 - $5,000
- **Payback Period:** 2-4 years
- **10-Year NPV:** $15,000 - $40,000 (at 5% discount rate)

## Next Steps

Upon successful completion of Phase 1, proceed to:

**PHASE 2: Grid Integration** - Advanced grid services, smart charging algorithms, market participation

---

## Appendix A: Required Equipment List

1. Bidirectional Onboard Charger (compatible with vehicle)
2. Level 2 Bidirectional EVSE (UL 2202 certified)
3. Bidirectional Energy Meter (IEC 62053 Class 1)
4. Communication Gateway (4G/LTE or Wi-Fi)
5. Electrical Panel and Breaker (50A minimum)
6. Grounding and Bonding Equipment
7. Surge Protection Device
8. Emergency Disconnect Switch

## Appendix B: Testing Checklist

- [ ] Visual inspection of all equipment
- [ ] Electrical continuity and resistance tests
- [ ] Ground fault test
- [ ] Insulation resistance test (>1 MΩ)
- [ ] Polarity verification
- [ ] Communication link test
- [ ] Charging power test (all power levels)
- [ ] Discharging power test (all power levels)
- [ ] Power quality measurement (THD, PF)
- [ ] Anti-islanding test
- [ ] Emergency stop test
- [ ] Thermal performance test (1-hour duration)
- [ ] Metering accuracy verification
- [ ] User interface functionality test
- [ ] Documentation review and completion

## Appendix C: Compliance Matrix

| Standard | Requirement | Compliance Method | Verification |
|----------|-------------|-------------------|--------------|
| IEEE 1547 | Anti-islanding | Hardware protection | Testing |
| UL 1741 | Inverter safety | Certified equipment | Documentation |
| ISO 15118-2 | Communication | Software implementation | Testing |
| IEC 61851-1 | Charging safety | Certified EVSE | Documentation |
| IEC 62196 | Connectors | Type 2/J1772 | Visual inspection |
| UN ECE R100 | Vehicle safety | OEM certification | Documentation |

---

**弘益人間 (Benefit All Humanity)**
© 2025 SmileStory Inc. / WIA - World Certification Industry Association
WIA-AUTO-029 Vehicle-to-Grid V2G Standard - Phase 1 Specification
