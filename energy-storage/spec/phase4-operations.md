# WIA-ENE-010: Energy Storage Standard
## Phase 4 - Operations & Maintenance

### Document Information
- **Version:** 1.0
- **Status:** Active
- **Last Updated:** 2025-12-25
- **Standard ID:** WIA-ENE-010

---

## 1. Operational Procedures

### 1.1 Normal Operations

#### 1.1.1 Startup Procedure

**Daily Startup (Automated):**
```
1. System wake from standby
2. Perform self-diagnostics (5 minutes)
   - BMS health check
   - Communication verification
   - Sensor validation
   - Safety system check
3. Resume programmed operating mode
4. Send status update to operators
```

**Manual Startup (After Maintenance):**
```
1. Visual inspection of system
2. Verify all disconnects closed
3. Verify cooling system operational
4. Enable BMS
5. Monitor DC bus pre-charge
6. Close battery contactors
7. Enable inverter
8. Synchronize with grid
9. Transition to standby mode
10. Enable EMS control
```

#### 1.1.2 Operating Modes

**Peak Shaving Mode:**
```
Operation:
- Monitor grid power
- Discharge when power > setpoint
- Charge when power < setpoint
- Maintain SOC in operating range

Parameters:
- Peak setpoint: User-defined
- Min SOC: 20%
- Max SOC: 90%
- Power ramp rate: 10%/second
```

**Load Leveling Mode:**
```
Operation:
- Charge during off-peak hours
- Discharge during peak hours
- Follow time-of-use schedule

Parameters:
- Schedule: User-defined
- Target SOC levels by time
- Power limits by time period
```

**Frequency Regulation Mode:**
```
Operation:
- Monitor grid frequency continuously
- Respond to frequency deviations
- Provide automatic generation control (AGC)

Parameters:
- Droop: 5% (configurable)
- Deadband: ±0.036 Hz
- Response time: <100ms
- Sustained response: 30 minutes
```

**Backup Power Mode:**
```
Operation:
- Monitor grid status
- Detect grid failure
- Island local loads
- Maintain voltage/frequency
- Reconnect when grid restored

Parameters:
- Transfer time: <10ms
- Island voltage: ±5%
- Island frequency: ±0.1 Hz
```

#### 1.1.3 Shutdown Procedure

**Normal Shutdown:**
```
1. Ramp power to zero (60 seconds)
2. Open grid contactor
3. Discharge to safe SOC (50%)
4. Open battery contactors
5. Disable inverter
6. Disable BMS (keep monitoring active)
7. Enter standby mode
8. Log shutdown event
```

**Emergency Shutdown:**
```
1. Activate emergency stop (E-stop)
2. Open all contactors immediately
3. Disable all power electronics
4. Activate fire suppression if needed
5. Alert operators
6. Log event with full diagnostics
7. Require manual reset
```

### 1.2 Monitoring & Alerting

#### 1.2.1 Key Performance Indicators (KPIs)

**Energy KPIs:**
```
Daily:
- Energy charged (kWh)
- Energy discharged (kWh)
- Round-trip efficiency (%)
- Self-consumption (kWh)

Monthly:
- Total throughput (MWh)
- Average efficiency (%)
- Availability (%)
- Revenue/savings ($)

Annual:
- Cumulative cycles
- Capacity retention (%)
- ROI progress (%)
```

#### 1.2.2 Alarm Management

**Alarm Levels:**
```
Level 1 - Information:
- Mode changes
- Scheduled maintenance due
- Non-critical status updates
Action: Log only

Level 2 - Warning:
- Approaching limits (80% threshold)
- Minor deviations from optimal
- Predictive maintenance alerts
Action: Notify operator, log event

Level 3 - Alarm:
- Exceeded safe operating limits
- Component degradation detected
- Communication loss
Action: Reduce power, notify operator immediately

Level 4 - Critical:
- Safety system activation
- Thermal runaway detection
- Ground fault detected
Action: Emergency shutdown, notify emergency contacts
```

**Alarm Response Procedures:**
```
Warning Response (Level 2):
1. Acknowledge alarm
2. Review system status
3. Assess root cause
4. Schedule corrective action if needed
5. Document resolution

Alarm Response (Level 3):
1. Immediate acknowledgment
2. Remote assessment
3. Dispatch technician if needed
4. Implement temporary mitigation
5. Schedule permanent fix

Critical Response (Level 4):
1. Verify emergency shutdown
2. Dispatch emergency response team
3. Assess safety of personnel/equipment
4. Secure site
5. Investigate root cause
6. Implement corrective actions
7. Perform comprehensive testing before restart
```

---

## 2. Preventive Maintenance

### 2.1 Maintenance Schedule

#### Monthly Maintenance (1 hour)
```
Inspections:
□ Visual inspection of all components
□ Check for leaks (cooling system)
□ Verify cooling system operation
□ Check for unusual sounds/odors
□ Inspect cable connections
□ Review alarm history

Measurements:
□ Battery system voltage
□ Insulation resistance (if accessible)
□ Ambient temperature/humidity

Documentation:
□ Log all observations
□ Update maintenance records
□ Schedule any needed repairs
```

#### Quarterly Maintenance (4 hours)
```
Inspections:
□ Torque check on all connections
□ Filter replacement/cleaning
□ Thermal imaging of electrical connections
□ Physical security check

Tests:
□ Capacity test (partial discharge)
□ Efficiency measurement
□ Communication link test
□ Backup power test (if applicable)

Software:
□ Firmware update check
□ Configuration backup
□ Log file archival

Documentation:
□ Performance trend analysis
□ Update predictive models
□ Schedule annual maintenance
```

#### Annual Maintenance (8 hours)
```
Comprehensive Inspection:
□ Full visual inspection (with access panels removed)
□ Structural inspection
□ Ventilation system inspection
□ Fire suppression system inspection

Electrical Tests:
□ Full capacity test
□ Insulation resistance test
□ Ground resistance test
□ Protection system verification

Mechanical:
□ Torque verification of all connections
□ Cooling system service
□ Filter replacement
□ Lubrication as needed

Software/Calibration:
□ Sensor calibration verification
□ Software updates
□ Backup/restore test
□ Cybersecurity assessment

Performance:
□ Full charge/discharge cycle
□ Efficiency measurement
□ Power quality assessment
□ Response time verification

Safety Systems:
□ Fire suppression test
□ Emergency stop test
□ Alarm system test
□ Safety interlock verification

Documentation:
□ Annual performance report
□ Remaining useful life assessment
□ Budget forecast for next year
□ Update maintenance procedures
```

### 2.2 Condition-Based Maintenance

#### 2.2.1 Capacity Monitoring

**Trending:**
```
Monitor quarterly capacity tests
Fit degradation curve
Predict capacity at future date

Alert when:
- Degradation rate increases
- Projected capacity < 80% before warranty end
- Capacity < 80% of rated
```

**Intervention Triggers:**
```
Capacity 90-80%: Increase monitoring frequency
Capacity 80-70%: Assess cell replacement options
Capacity <70%: Plan system upgrade/replacement
```

#### 2.2.2 Resistance Monitoring

**Tracking:**
```
Monitor internal resistance trend
Compare to baseline and specifications

Alert when:
- Resistance > 150% of baseline
- Rapid increase detected (>10% per month)
```

#### 2.2.3 Temperature Monitoring

**Pattern Analysis:**
```
Monitor temperature distribution
Identify hot spots
Track seasonal variations

Alert when:
- Temperature gradient > 5°C
- Hot spots develop (>10°C above average)
- Cooling system performance degrades
```

### 2.3 Corrective Maintenance

#### 2.3.1 Common Issues & Solutions

**Issue: Capacity Degradation**
```
Symptoms:
- Reduced runtime
- Lower than expected SOC
- Increased charging time

Root Causes:
- Normal aging
- Thermal stress
- Deep cycling
- Calendar aging

Solutions:
- Optimize operating strategy
- Improve thermal management
- Reduce depth of discharge
- Consider cell/module replacement
```

**Issue: High Cell Voltage Imbalance**
```
Symptoms:
- Large voltage spread between cells
- Frequent balancing activity
- Premature charge termination

Root Causes:
- Failed balancing circuit
- Weak cell(s)
- SOC estimation error

Solutions:
- Verify balancing function
- Identify and replace weak cells
- Recalibrate SOC algorithm
```

**Issue: Thermal Issues**
```
Symptoms:
- High operating temperatures
- Thermal derating
- Thermal shutdowns

Root Causes:
- Cooling system failure
- Blocked airflow
- High ambient temperature
- Excessive internal resistance

Solutions:
- Service cooling system
- Clear ventilation paths
- Improve facility cooling
- Assess battery health
```

**Issue: Communication Failures**
```
Symptoms:
- Lost data
- Control failures
- Alarm generation

Root Causes:
- Network issues
- Cable damage
- Software bugs
- Electromagnetic interference

Solutions:
- Check network connectivity
- Inspect/replace cables
- Update firmware
- Improve shielding/grounding
```

---

## 3. Performance Optimization

### 3.1 Operating Strategy Optimization

#### 3.1.1 Peak Shaving Optimization

**Analysis:**
```
Review historical load data
Identify peak demand patterns
Calculate optimal setpoint

Optimization:
setpoint = average_load + 0.8 × (peak_load - average_load)

Benefits:
- Maximizes demand charge savings
- Minimizes battery cycling
- Extends battery life
```

#### 3.1.2 Energy Arbitrage Optimization

**Price Forecasting:**
```
Collect historical price data
Train machine learning model
Predict next-day prices

Strategy:
- Charge when predicted price < threshold_low
- Discharge when predicted price > threshold_high
- Account for efficiency losses and degradation costs
```

#### 3.1.3 Multi-Service Stacking

**Combined Revenue Streams:**
```
Services:
1. Peak shaving (demand charge reduction)
2. Energy arbitrage (time-of-use optimization)
3. Frequency regulation (grid services)
4. Backup power (resilience value)

Optimization:
- Prioritize by value ($/kWh)
- Ensure SOC reserve for backup
- Coordinate multiple objectives
- Maximize total value
```

### 3.2 SOC Management

#### 3.2.1 Operating Window Optimization

**Standard Strategy:**
```
Min SOC: 20%
Max SOC: 90%
Operating Range: 70%

Benefits:
- Extended cycle life
- Thermal management
- Reserve capacity
```

**Aggressive Strategy (when needed):**
```
Min SOC: 10%
Max SOC: 95%
Operating Range: 85%

Use Cases:
- Emergency backup
- High-value events
- Rare usage
```

#### 3.2.2 Seasonal Adjustments

**Summer (High Cooling Load):**
```
- Increase min SOC for backup
- Limit discharge power to reduce heat
- Schedule charging for cooler hours
```

**Winter (Low Temperature):**
```
- Pre-heat battery if available
- Limit charge power at low temp
- Adjust SOC algorithm for temperature
```

---

## 4. Troubleshooting

### 4.1 Diagnostic Procedures

#### 4.1.1 Systematic Approach

**Step 1: Gather Information**
```
- Review alarm history
- Check system logs
- Interview operators
- Observe system behavior
```

**Step 2: Analyze Data**
```
- Identify patterns
- Correlate events
- Review trends
- Compare to baseline
```

**Step 3: Hypothesize**
```
- List possible causes
- Rank by probability
- Consider recent changes
```

**Step 4: Test Hypothesis**
```
- Perform targeted tests
- Isolate variables
- Verify root cause
```

**Step 5: Implement Solution**
```
- Plan corrective action
- Implement fix
- Test system
- Verify resolution
```

**Step 6: Document**
```
- Record root cause
- Document solution
- Update procedures
- Share lessons learned
```

#### 4.1.2 Diagnostic Tools

**Software Tools:**
```
- BMS diagnostic software
- Inverter configuration tools
- Network analyzers
- Data analysis software (Python, MATLAB)
```

**Hardware Tools:**
```
- Multimeter (Fluke 87V or similar)
- Clamp meter (AC/DC)
- Thermal camera (FLIR or similar)
- Power quality analyzer
- Insulation tester
- Oscilloscope
```

### 4.2 Common Diagnostic Scenarios

**Scenario 1: System Not Charging**
```
Check:
1. Grid voltage present?
   - No: Check utility supply
   - Yes: Continue

2. Inverter enabled?
   - No: Check control signals
   - Yes: Continue

3. SOC < max SOC?
   - No: Normal behavior
   - Yes: Continue

4. BMS allowing charge?
   - No: Check BMS alarms
   - Yes: Continue

5. Battery contactors closed?
   - No: Check control logic
   - Yes: Check power flow path
```

**Scenario 2: Poor Efficiency**
```
Analysis:
1. Measure AC input/output
2. Measure DC battery power
3. Calculate losses:
   - Inverter losses
   - Battery losses
   - Auxiliary losses

Compare to specifications
Identify highest loss contributor
Investigate root cause
```

**Scenario 3: Frequent Alarms**
```
Analysis:
1. Review alarm frequency
2. Identify alarm types
3. Look for patterns:
   - Time of day
   - Operating mode
   - Environmental conditions

Common Causes:
- Setpoints too tight
- Sensor drift
- Control instability
- External disturbances
```

---

## 5. End of Life Management

### 5.1 Performance Assessment

#### When to Consider Replacement:
```
Indicators:
- Capacity < 70% of rated
- Round-trip efficiency < 75%
- Frequent failures (>5 per month)
- Maintenance costs > operating savings
- Safety concerns
```

### 5.2 Decommissioning Procedure

**Planning:**
```
1. Schedule outage
2. Notify stakeholders
3. Arrange disposal/recycling
4. Plan replacement (if applicable)
```

**Execution:**
```
1. Discharge to safe SOC (0-10%)
2. Disconnect from grid
3. Open all circuit breakers
4. Disconnect battery cables
5. Drain cooling system
6. Remove modules/packs
7. Package for transport
8. Clean and restore site
```

**Disposal:**
```
- Contact certified recycler
- Provide battery chemistry data
- Ship according to regulations (UN 3480/3481)
- Obtain disposal certificate
- Update asset records
```

### 5.3 Second-Life Applications

**Assessment:**
```
Criteria for Second Life:
- Capacity > 70%
- No safety concerns
- Reasonable remaining cycle life

Potential Applications:
- Residential storage
- Small commercial systems
- Off-grid applications
- Renewable integration
```

---

## 6. Continuous Improvement

### 6.1 Performance Reviews

**Monthly Review:**
```
Metrics:
- Availability
- Energy throughput
- Efficiency
- Revenue/savings
- Alarm frequency

Actions:
- Identify anomalies
- Trend analysis
- Optimization opportunities
```

**Annual Review:**
```
Assessment:
- Overall system performance
- ROI achievement
- Maintenance costs
- Degradation rate

Strategic Planning:
- Warranty utilization
- Upgrade opportunities
- Replacement timing
- Lessons learned
```

### 6.2 Technology Updates

**Firmware/Software:**
```
- Review release notes
- Test in non-production environment
- Schedule update window
- Backup current configuration
- Perform update
- Verify functionality
- Monitor for issues
```

**Hardware Upgrades:**
```
Opportunities:
- Improved BMS modules
- Higher efficiency inverters
- Advanced sensors
- Enhanced safety systems

Process:
- Evaluate cost/benefit
- Plan installation
- Minimize downtime
- Update documentation
```

---

## 7. Conclusion

Phase 4 provides comprehensive operational and maintenance procedures to ensure WIA-ENE-010 compliant systems deliver optimal performance, safety, and longevity throughout their operational lifetime. Proactive maintenance and continuous optimization maximize value and minimize total cost of ownership.

**End of WIA-ENE-010 Specification Series**

---

© 2025 SmileStory Inc. / WIA  
弘益人間 (Hongik Ingan) · Benefit All Humanity
