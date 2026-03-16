# WIA-ENE-007: Hydrogen Energy Standard  
## PHASE 3: Integration (Months 19-30)

**弘益人間 (홍익인간) - Benefit All Humanity**

---

## 3.1 Phase Overview

Phase 3 focuses on commissioning, startup, performance verification, and full integration of the hydrogen energy system into operational service. This phase transitions responsibility from construction to operations while ensuring all performance guarantees are met and the facility operates safely and reliably.

### Phase Objectives
- Safely commission all systems from nitrogen to hydrogen operation
- Achieve stable, continuous hydrogen production
- Verify performance guarantees (capacity, efficiency, purity)
- Integrate with external systems (grid, offtakers, utilities)
- Optimize operations for efficiency and reliability
- Complete handover to operations team

### Success Criteria
- ✓ Successful first hydrogen production and purity verification
- ✓ Performance guarantees met or exceeded (capacity ≥98%, efficiency ≥95% of guarantee)
- ✓ 30-day reliability test passed (availability >95%)
- ✓ Zero safety incidents during commissioning and startup
- ✓ All training and documentation complete
- ✓ Final acceptance certificate issued

---

## 3.2 Commissioning Strategy

### Commissioning Phases

**Phase 3.1: Systems Integration Testing (Months 19-21)**
- Individual system commissioning (utilities, control systems, safety systems)
- Integration testing (control logic, interlocks, alarms)
- Nitrogen commissioning (purging, leak testing, functional checks)

**Phase 3.2: First Hydrogen Introduction (Month 22)**
- Safe hydrogen introduction procedures
- Low-pressure operation and leak surveys
- System pressure ramp-up
- Hydrogen quality baseline

**Phase 3.3: Performance Testing (Months 23-24)**
- Capacity testing at various loads
- Efficiency measurements
- Purity verification (ISO 14687 compliance)
- Environmental compliance testing

**Phase 3.4: Reliability Demonstration (Months 25-26)**
- 30-day continuous operation test
- Load-following and turndown testing
- Emergency shutdown and restart testing
- Integration with grid and offtaker systems

**Phase 3.5: Optimization and Tuning (Months 27-30)**
- Process optimization (efficiency improvements)
- Control system tuning (response time, stability)
- Predictive maintenance program implementation
- Operator proficiency development

### Commissioning Team Structure

```
Commissioning Manager
(Overall commissioning leadership)
        |
    ----|----
    |       |
Process Lead    Safety Lead
    |               |
- Electrolyzer  - Permits
- Compression   - Safety systems
- Purification  - Environmental
- Storage       - Emergency response
    |
Controls Lead
    |
- SCADA
- PLC/DCS
- Interlocks
- Cybersecurity
```

**Commissioning Authority:**
- Commissioning Manager has stop-work authority for safety concerns
- No deviation from approved commissioning procedures without written approval
- Daily safety briefings mandatory for all personnel

**Deliverable:** Commissioning Plan
- Detailed commissioning procedures for each system
- Sequence and schedule (critical path analysis)
- Hold points requiring approval before proceeding
- Resource requirements (personnel, utilities, consumables)
- Safety protocols and emergency response procedures

---

## 3.3 Systems Commissioning (Nitrogen Phase)

### Utility Systems Commissioning

**Electrical Power:**
- [ ] Energize electrical distribution from utility connection
- [ ] Verify voltage, frequency, phase balance at all panels
- [ ] Functional test of emergency power system (if applicable)
- [ ] Load bank testing of critical circuits
- [ ] Protective relay testing and coordination

**Water Systems:**
- [ ] Commission water treatment system (RO, EDI)
- [ ] Verify DI water quality (resistivity >10 MΩ·cm)
- [ ] Cooling water system flow and temperature testing
- [ ] Fire protection system hydrostatic and flow testing

**Compressed Air:**
- [ ] Compressor commissioning and performance testing
- [ ] Air dryer and filtration verification (dew point, particulate)
- [ ] Distribution system leak testing
- [ ] Instrument air quality verification (ISO 8573-1)

**Nitrogen Supply:**
- [ ] Connect nitrogen supply (tube trailers or on-site generator)
- [ ] Verify purity (>99.9%, O₂ <100 ppm)
- [ ] Pressure regulation and distribution testing

### Control System Commissioning

**SCADA and HMI:**
- [ ] Power up and boot sequence verification
- [ ] Network connectivity testing (field devices, enterprise systems)
- [ ] Graphics and alarm configuration verification
- [ ] Historical data logging functionality
- [ ] User access control and audit trail testing

**PLC/DCS:**
- [ ] Controller power-up and program loading
- [ ] I/O mapping verification (actual vs. configured)
- [ ] Analog signal calibration check (4-20mA loops)
- [ ] Discrete signal testing (digital inputs/outputs)

**Interlock and Safety Logic:**
- [ ] Logic testing using forcing functions (no actual trips)
- [ ] Shutdown sequence verification (orderly vs. emergency)
- [ ] Permissive testing (start-up conditions)
- [ ] Alarm priority and response verification

**Cybersecurity Validation:**
- [ ] Firewall configuration and rule testing
- [ ] Intrusion detection system (IDS) functionality
- [ ] Access control and authentication testing
- [ ] Backup and recovery procedures validation

### Safety Systems Commissioning

**Gas Detection:**
- [ ] Sensor installation verification (location, orientation)
- [ ] Calibration with known gas concentrations
- [ ] Alarm setpoint verification (10% LEL warning, 20% LEL alarm)
- [ ] Response time measurement (<30 seconds T90)
- [ ] Interlock testing (alarm → valve closure, ventilation activation)

**Fire Protection:**
- [ ] Fire detection system testing (smoke, heat, flame detectors)
- [ ] Suppression system functional test (dry run, no agent discharge)
- [ ] Manual pull stations and notification devices
- [ ] Integration with fire alarm monitoring

**Emergency Shutdown (ESD):**
- [ ] ESD logic functional testing
- [ ] Individual ESD button testing (local and remote)
- [ ] Automatic ESD triggers (gas detection, high pressure, etc.)
- [ ] Valve closure verification (timing, full closure confirmation)
- [ ] Reset and restart procedures

**Deliverable:** System Commissioning Reports
- Commissioning procedure completion checklists
- Test results and acceptance criteria verification
- Non-conformances and resolutions
- System acceptance signatures (commissioning lead, operations, owner)

---

## 3.4 Nitrogen Commissioning and Leak Testing

### Nitrogen Purging

**Objective:** Displace air from all process lines and equipment with nitrogen to eliminate oxygen before hydrogen introduction.

**Procedure:**
1. Close all manual isolation valves except purge inlet and outlet
2. Introduce nitrogen at low flow rate (displacement purge)
3. Monitor oxygen concentration at outlet (target: <1% O₂)
4. Minimum purge volume: 5× system volume
5. For critical systems (electrolyzer, fuel cells): <0.5% O₂, <100 ppm moisture

**Purge Verification:**
- Oxygen measurement at multiple sample points
- Documentation of purge time, flow rate, and final O₂ concentration
- Hold pressure after purge to verify no air in-leakage

### Pressure Testing

**Test Pressure Selection:**
- Design pressure: Specified in vessel/piping design
- Test pressure: 1.5× design pressure (per ASME codes)
- Test medium: Nitrogen (for hydrogen systems)

**Procedure (Pneumatic Test):**
1. Isolate section to be tested (double block and bleed)
2. Slowly pressurize to 25% of test pressure (check for gross leaks)
3. Increase to 50%, 75%, then full test pressure
4. Hold at test pressure for 4 hours minimum
5. Acceptance: <2% pressure drop (temperature-corrected)

**Safety Measures:**
- Personnel exclusion zone during pressurization
- Pressure relief device installed at test pressure +10%
- Slow depressurization (risk of brittle fracture if too fast)
- PPE: Safety glasses, hearing protection

**Leak Detection Survey:**
- Portable hydrogen detectors at all flanges, valves, fittings
- Soap solution testing of suspect areas
- Acceptance: <500 ppm H₂ equivalent at any point

**Deliverable:** Pressure Test Reports
- Pressure test data sheets (pressure vs. time, temperature)
- Leak survey results with location photos
- Non-conformance reports for any leaks found and repairs made
- Acceptance signatures

---

## 3.5 First Hydrogen Introduction

### Pre-Start Safety Review

**Checklist:**
- [ ] All nitrogen commissioning complete and accepted
- [ ] Leak testing passed with no unresolved leaks
- [ ] Safety systems functional (gas detection, ESD, fire protection)
- [ ] Operations team briefed on hydrogen introduction procedure
- [ ] Emergency response plan reviewed with local fire department
- [ ] Hydrogen supply available (tube trailers, pipeline, or on-site production)
- [ ] Commissioning Manager approval to proceed

### Hydrogen Introduction Procedure

**Step 1: Atmospheric Purge (Nitrogen → Hydrogen)**
- Slowly introduce hydrogen at purge outlet while venting nitrogen
- Monitor O₂ and N₂ concentration in effluent gas
- Criteria: H₂ >98%, O₂ <1%, N₂ <2%
- Vent to safe location (above roof level, away from ignition sources)

**Step 2: Leak Detection Survey**
- Portable H₂ detectors survey all flanges, valves, connections
- Soap solution bubble testing
- Fixed gas detectors functional verification
- Any leak >100 ppm: stop introduction, repair, re-test

**Step 3: Low-Pressure Operation (24-hour hold)**
- Pressurize to 25% of normal operating pressure
- Monitor for pressure decay (<1% in 24 hours acceptable)
- Verify no alarms or gas detection events
- Temperature monitoring for any hot spots

**Step 4: Pressure Ramp-Up**
- Increase to 50% pressure (24-hour hold)
- Increase to 75% pressure (24-hour hold)
- Increase to 100% pressure
- Repeat leak survey at each pressure step

**Step 5: Hydrogen Quality Baseline**
- Sample hydrogen from multiple locations
- Laboratory analysis per ISO 14687 (all parameters)
- Establish baseline purity from initial fill (before production starts)

**Safety Briefing:**
- Daily briefings during hydrogen introduction
- Review of hydrogen properties, hazards, and controls
- Emergency response procedures (evacuation, communication)
- Hydrogen flame characteristics (invisible, use thermal imaging)

**Deliverable:** First Hydrogen Introduction Report
- Purge verification data (gas composition vs. time)
- Leak survey results
- Pressure test data at each step
- Hydrogen quality baseline analysis
- Safety briefing attendance records

---

## 3.6 Electrolyzer First Start and Performance Testing

### Electrolyzer Commissioning

**Pre-Start Checks (Electrolyzer-Specific):**
- [ ] DI water system operational (resistivity verified)
- [ ] Electrolyte condition verified (alkaline only; PEM uses solid membrane)
- [ ] Stack resistance measurement (compare to baseline from vendor)
- [ ] Cooling system operational (flow, temperature control)
- [ ] Gas drying and purification systems ready
- [ ] Hydrogen storage or vent capacity available

**First Energization (Low Current Start):**

1. **10% Rated Current (1-2 hours):**
   - Monitor cell voltages (uniformity ±50 mV)
   - Verify hydrogen production (compare to theoretical)
   - No leaks (internal or external)
   - Temperature within specification

2. **25% Rated Current (2-4 hours):**
   - Efficiency verification (kWh/kg H₂)
   - Purity sampling and analysis (online and lab)
   - Thermal balance check (cooling capacity adequate)

3. **50% Rated Current (8 hours):**
   - Control system tuning (response to setpoint changes)
   - Load response testing (ramp up/down)

4. **75% Rated Current (16 hours):**
   - Extended operation observation
   - Gas processing system performance (drying, purification)

5. **100% Rated Current (24-hour continuous test):**
   - Performance guarantee verification
   - Hydrogen production rate ≥98% of nameplate
   - Efficiency ≥95% of guaranteed value (typically 65-75% HHV)
   - Purity meets ISO 14687 (all parameters)

### Performance Testing Protocol

**Capacity Test:**
- Operate at rated power for 24 hours continuous
- Measure: Electricity input (kW), hydrogen output (Nm³/h or kg/h), oxygen output
- Calculate: Specific energy consumption (kWh/Nm³ or kWh/kg)
- Acceptance: Production rate ≥98% of nameplate capacity

**Efficiency Test:**
- Steady-state operation at 100%, 75%, 50%, 25% load
- Measure: Input power, hydrogen output, temperatures
- Calculate: System efficiency (HHV or LHV basis)
- Acceptance: Efficiency ≥95% of guarantee at rated capacity

**Purity Test:**
- Sample hydrogen at delivery point (after all purification)
- Laboratory analysis per ISO 14687
- Parameters: Total non-H₂ gases, H₂O, O₂, N₂, CO, CO₂, hydrocarbons, sulfur, ammonia
- Acceptance: All parameters within ISO 14687 limits

**Turndown Test:**
- Demonstrate operation at minimum load (typically 10-20% of rated)
- Ramp from minimum to maximum and back
- Measure: Ramp rate, efficiency at part-load, purity stability

**Load-Following Test (if applicable):**
- Simulate renewable energy variability
- Step changes and ramp changes in power input
- Measure: Response time, efficiency impact, purity impact

**Deliverable:** Performance Test Report
- Test procedures and data sheets
- Measured performance vs. guaranteed values
- Purity analysis certificates
- Acceptance statement (pass/fail with deviations noted)

---

## 3.7 Integration with External Systems

### Grid Integration

**Interconnection Testing:**
- [ ] Utility interconnection agreement executed
- [ ] Protection relay settings coordinated and tested
- [ ] Power quality testing (harmonics, power factor, flicker)
- [ ] SCADA integration (real-time data to utility)
- [ ] Demand response signals (if participating in DR programs)

**Grid Services (if applicable):**
- [ ] Frequency regulation response testing
- [ ] Ramp rate capability demonstration
- [ ] Voltage support (reactive power capability)
- [ ] Communication protocol validation (IEC 61850, DNP3, Modbus)

**Renewable Energy Correlation:**
- [ ] Metering of renewable energy generation and electrolyzer consumption
- [ ] Data logging for green hydrogen certification
- [ ] REC (Renewable Energy Certificate) procurement and tracking
- [ ] Carbon intensity calculation and reporting

### Hydrogen Offtake Integration

**Pipeline Connection (if applicable):**
- [ ] Pipeline interconnection complete and pressure-tested
- [ ] Flow metering calibrated and verified
- [ ] Quality monitoring (online and periodic lab analysis)
- [ ] Custody transfer agreement and billing system operational

**Tube Trailer Filling (if applicable):**
- [ ] Fill station commissioned (compression, cooling, metering)
- [ ] Safety protocols for trailer connection/disconnection
- [ ] Trailer inventory management system
- [ ] Delivery scheduling and logistics

**Fuel Cell or End-Use Integration:**
- [ ] End-use equipment hydrogen quality verification
- [ ] Supply pressure and flow rate verification
- [ ] Backup supply or redundancy testing
- [ ] Communication protocols for supply/demand coordination

**Deliverable:** Integration Test Reports
- Grid interconnection test results
- Offtake system commissioning and performance data
- Integration agreements and approvals
- Operational protocols for integrated operation

---

## 3.8 Reliability Demonstration and Optimization

### 30-Day Reliability Test

**Objective:** Demonstrate sustained, reliable operation meeting availability and performance targets.

**Test Protocol:**
- Continuous operation for 30 days (720 hours)
- Operate at variable loads simulating actual operating profile
- Planned downtime (maintenance, shift changes): excluded from availability calculation
- Unplanned downtime (trips, equipment failure): counts against availability

**Performance Metrics:**
- **Availability:** = (Operating hours / (Total hours - Planned downtime)) × 100%
  - Target: >95% (industry best practice)
- **Capacity Factor:** = (Actual H₂ production / Theoretical max production) × 100%
  - Target: >80% (depends on operational profile)
- **Efficiency:** Average efficiency over test period
  - Target: Within 5% of guaranteed efficiency
- **Purity Compliance:** = (Samples meeting ISO 14687 / Total samples) × 100%
  - Target: 100%

**Daily Monitoring:**
- Production logs (hydrogen output, electricity consumption)
- Purity sampling (daily minimum)
- Equipment status (run hours, alarms, trips)
- Environmental compliance (water usage, discharge quality)

**Incident Management:**
- All trips and unplanned downtime logged with root cause analysis
- Corrective actions implemented before restart
- Lessons learned documented

**Deliverable:** Reliability Test Report
- 30-day production summary (daily and cumulative)
- Availability and capacity factor calculations
- Incident log with root cause analysis
- Performance trend charts
- Acceptance statement

### Process Optimization

**Optimization Opportunities:**

1. **Efficiency Improvements:**
   - Operating temperature optimization (electrolyzer operates more efficiently at higher temperature, within limits)
   - Load scheduling (operate when renewable energy is abundant and cheap)
   - Waste heat recovery (for building heating, preheating water)

2. **Availability Improvements:**
   - Predictive maintenance (vibration analysis, oil analysis, thermography)
   - Spare parts inventory optimization (critical spares on-site)
   - Operator proficiency (training on troubleshooting, faster response)

3. **Cost Reduction:**
   - Energy management (minimize auxiliary power consumption)
   - Water usage optimization (recycling, cascading use)
   - Oxygen monetization (sell or use on-site)

**Continuous Improvement Program:**
- Weekly operations review meetings
- Monthly KPI tracking and trend analysis
- Quarterly benchmarking against industry standards
- Annual process review and optimization study

**Deliverable:** Optimization Report
- Baseline performance metrics (before optimization)
- Improvement initiatives with business case
- Implementation results (measured improvements)
- Recommendations for ongoing optimization

---

## 3.9 Environmental and Regulatory Compliance

### Environmental Monitoring

**Air Quality:**
- Verification that operations meet air permit conditions (if applicable)
- Emissions testing (minimal for electrolysis; may apply to ancillary equipment)
- Fugitive emissions monitoring (leak detection and repair program)

**Water:**
- Discharge water quality monitoring (pH, temperature, dissolved solids)
- Compliance with discharge permit limits
- Water usage tracking and reporting

**Waste Management:**
- Characterization of any waste streams (spent catalysts, filters, oils)
- Proper disposal per hazardous waste regulations
- Waste minimization and recycling initiatives

**Noise:**
- Boundary noise monitoring (ensure compliance with local ordinances)
- Mitigation measures if needed (enclosures, barriers)

**Deliverable:** Environmental Compliance Report
- Monitoring data (air, water, noise)
- Compliance status (permit conditions vs. actual)
- Non-compliance events and corrective actions
- Annual environmental report (if required by permits)

### Safety and Regulatory Inspections

**Regulatory Inspections:**
- Fire marshal final inspection
- OSHA inspection (if triggered)
- Pressure vessel inspection (Authorized Inspector)
- Environmental agency inspection (air, water permits)

**Internal Audits:**
- Process Safety Management (PSM) audit (if applicable)
- Safety system functional testing (annual)
- Emergency response drill (semi-annual)

**Deliverable:** Inspection and Audit Log
- Inspection dates, agencies, findings
- Corrective action plans and completion status
- Ongoing inspection schedule (annual pressure vessel, etc.)

---

## 3.10 Phase 3 Completion and Handover to Operations

### Final Acceptance Criteria

- [ ] Performance tests passed (capacity, efficiency, purity)
- [ ] 30-day reliability test passed (availability >95%)
- [ ] All regulatory inspections passed
- [ ] Environmental compliance demonstrated
- [ ] Operations team fully trained and competent
- [ ] As-built documentation complete and accurate
- [ ] O&M manuals and spare parts inventory finalized
- [ ] No outstanding safety or critical quality issues

### Handover to Operations

**Transition Activities:**
- Formal handover meeting (construction → operations)
- Transfer of as-built drawings, O&M manuals, training records
- Spare parts inventory and warehouse setup
- Maintenance schedule and work order system activation
- Shift staffing and supervision plan implementation

**Performance Guarantee Period:**
- Typically 1 year after final acceptance
- Vendor remains responsible for defects and performance shortfalls
- Owner operates facility with vendor technical support
- Performance monitoring and quarterly reviews

**Post-Startup Support:**
- Vendor technical support (on-call, periodic site visits)
- Operator mentoring (advanced troubleshooting, optimization)
- Performance benchmarking and improvement initiatives

### Final Acceptance Certificate

**Issued by:** Owner (after verification of all acceptance criteria)

**Contents:**
- Project summary (scope, schedule, budget)
- Performance test results (met or exceeded)
- Reliability demonstration results
- Outstanding items (punch list, warranty items)
- Acceptance date and signatures

**Trigger Events:**
- Final payment to EPC contractor
- Warranty period commencement
- Insurance policy adjustments (construction → operational)
- Release of retainage and performance bonds

**Deliverable:** Final Acceptance Package
- Final Acceptance Certificate
- Performance test reports
- Reliability test report
- As-built documentation (drawings, manuals)
- Training records and certifications
- Lessons learned and recommendations

---

**End of Phase 3 Specification**

**Next Steps:** Phase 4 (Optimization) will focus on continuous improvement, cost reduction, and maximizing long-term performance and profitability of the hydrogen facility.

---

© 2025 SmileStory Inc. / WIA  
WIA-ENE-007: Hydrogen Energy Standard v1.0  
弘익人間 (홍익인간) · Benefit All Humanity
