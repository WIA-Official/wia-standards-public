# WIA-ENE-023 PHASE-2: Core Technologies

**Version**: 1.0  
**Status**: Active  
**Category**: Technical Specification  
**Theme Color**: #22C55E  
**Prerequisites**: PHASE-1 Compliance

---

## Philosophy

**弘益人間 (Hongik Ingan)** - Technology Serves Humanity's Sustainable Future

Phase 2 implements core recycling technologies enabling automated material separation, contamination detection, and quality assurance at scale.

---

## 1. Executive Summary

WIA-ENE-023 Phase 2 specifies technical requirements for core recycling technologies including optical sorting, eddy current separation, contamination detection, and quality control systems. Facilities implementing Phase 2 achieve significantly improved recovery rates, material purity, and operational efficiency compared to basic mechanical sorting.

### 1.1 Scope

Phase 2 covers:
- Optical sorting technologies (NIR, RGB, hyperspectral)
- Electromagnetic separation systems
- Contamination detection equipment
- Quality control instrumentation
- Process control systems
- Data acquisition and SCADA integration
- Equipment specifications and performance requirements
- Installation and commissioning protocols

### 1.2 Performance Targets

Facilities completing Phase 2 implementation shall achieve:
- Recovery Rate: ≥85% (vs. 60-75% baseline)
- Material Purity: ≥95% (vs. 85-90% baseline)
- Sorting Accuracy: ≥98% (optical sorters)
- Throughput Increase: 30-50% vs. manual sorting
- Contamination Detection: ≥95% accuracy for critical contaminants
- Equipment Uptime: ≥92%
- Energy Efficiency: <45 kWh per tonne processed

---

## 2. Optical Sorting Technologies

### 2.1 Near-Infrared (NIR) Spectroscopy

#### 2.1.1 Technical Requirements

**Wavelength Range**: 1000-2500 nanometers minimum

**Detection Rate**: ≥300,000 objects per hour per meter of belt width

**Belt Speed Compatibility**: 2-4 meters per second

**Sorting Width Options**: 1.2m, 1.6m, 2.0m, 2.4m, 3.2m

**Accuracy**: ≥98% for dry, clean materials; ≥95% for wet/contaminated materials

**Response Time**: ≤5 milliseconds from detection to ejection signal

**Air System**: 
- Pressure: 6-8 bar
- Flow: 15-25 cubic meters per minute per meter width
- Valve density: ≥32 valves per meter
- Valve response: <3 milliseconds

**Recognition Capabilities**: Minimum identification of PET, HDPE, PP, PS, PVC, paper, cardboard, metals

#### 2.1.2 Installation Requirements

**Mounting**: Adjustable height 0.8-1.5 meters above belt

**Illumination**: High-intensity halogen or LED (minimum 50,000 lux at material surface)

**Environmental**: Operating temperature 0-45°C, humidity 10-90% non-condensing

**Electrical**: 15-25 kW per unit, 400V 3-phase or per local standards

**Compressed Air**: Dedicated supply with filtration (5 micron), drying (<-40°C dewpoint), regulation

**Networking**: Gigabit Ethernet connectivity, SCADA integration capability

#### 2.1.3 Calibration Protocols

**Initial Calibration**:
1. Dark current calibration (sensors in dark)
2. White reference calibration (99% reflectance standard)
3. Material library creation (scan 20+ samples per material type)
4. Spectral signature verification
5. Ejection timing optimization (test objects at multiple belt positions)
6. Multi-material mix testing (verify no cross-contamination)

**Ongoing Calibration**:
- Daily: White reference check
- Weekly: Material library verification (spot checks)
- Monthly: Full spectral calibration
- Quarterly: Complete material library update

### 2.2 RGB Vision Systems

#### 2.2.1 Technical Specifications

**Camera Resolution**: Minimum 4096 pixels line-scan

**Frame Rate**: 30-60 frames per second

**Color Depth**: 24-bit color (8 bits per RGB channel)

**Field of View**: Matches belt width with 10% overlap margins

**Lighting**: Uniform LED illumination (6500K color temperature, CRI ≥90)

**Recognition Algorithms**: Support for shape, color, texture, brand mark recognition

#### 2.2.2 Applications

**Glass Color Sorting**:
- Separation of clear, green, amber glass
- Accuracy: ≥96% single-pass
- Throughput: 4-6 tonnes per hour per meter width

**Container Recognition**:
- Bottle vs. cup identification
- Brand recognition for deposit return
- Damaged container detection
- Label coverage assessment

**Quality Inspection**:
- Visual defect detection
- Color uniformity verification
- Foreign object identification

### 2.3 X-Ray Transmission (XRT) Systems

#### 2.3.1 Technical Requirements

**X-Ray Source**: 50-160 kV adjustable

**Detection**: Linear detector array, minimum 0.5mm spatial resolution

**Throughput**: 40-120 tonnes per hour (depends on material and particle size)

**Identification**: Material density differentiation, heavy metal detection

**Safety**: Fully enclosed system, interlocks, radiation monitoring, compliance with local radiation safety regulations

#### 2.3.2 Applications

**Heavy Material Separation**:
- Glass from mixed containers
- Stones/concrete from organics
- Dense plastics from light plastics

**Contaminant Detection**:
- Lead-containing materials
- Dense metals in waste streams
- Batteries and electronic components

---

## 3. Electromagnetic Separation

### 3.1 Magnetic Separation

#### 3.1.1 Equipment Types

**Overhead Magnets**:
- Permanent or electromagnetic
- Suspended over conveyor belt
- Self-cleaning or manual cleaning
- Typical field strength: 2000-4000 gauss at material surface

**Magnetic Drum Separators**:
- Rotating drum with internal magnets
- Continuous separation of ferrous materials
- Drum diameter: 300-600mm typical
- Rotational speed: 30-60 RPM

**Cross-Belt Magnets**:
- Perpendicular belt captures and removes ferrous metals
- Continuous automatic discharge
- Belt width matches process conveyor

#### 3.1.2 Performance Requirements

**Ferrous Recovery**: ≥99% of ferrous materials ≥25mm

**Purity**: ≥98% ferrous in output (≤2% non-ferrous contamination)

**Throughput**: No significant reduction of main line throughput

**Maintenance**: Cleaning/discharge operation without process shutdown

### 3.2 Eddy Current Separation

#### 3.2.1 Technical Specifications

**Rotor Design**:
- Rare-earth permanent magnets (neodymium-iron-boron)
- Rotor diameter: 250-500mm (larger = more separation force)
- Pole arrangement: optimized for aluminum can size range
- Rotational speed: 2000-3500 RPM

**Belt Configuration**:
- Belt speed: 2.5-4.0 m/s
- Material layer depth: 25-50mm (monolayer preferred)
- Belt tension monitoring and automatic adjustment

**Splitter Positioning**:
- Adjustable aluminum collection splitter
- Position optimized for material trajectory
- Typical throw distance: 0.8-1.5 meters for aluminum cans

**Power Requirements**:
- 8-15 kW per unit
- VFD control for speed adjustment
- Soft start capability

#### 3.2.2 Performance Targets

**Aluminum Recovery**: ≥98% of aluminum containers and fragments ≥25mm

**Copper Recovery**: ≥95% of copper materials

**Brass Recovery**: ≥90% of brass materials

**Purity**: ≥96% non-ferrous metals in output

**Material Size Range**: Effective separation for 15-200mm particles

#### 3.2.3 Optimization Parameters

**Rotor Speed**:
- Higher speed = greater throwing force
- Optimize for target material (2200-2800 RPM typical for aluminum cans)

**Belt Speed**:
- Match to material presentation requirements
- Typically 2.8-3.5 m/s for mixed containers

**Material Layer**:
- Monolayer ideal (best separation)
- Maximum 50mm depth (performance degrades with excessive depth)

**Splitter Position**:
- Adjust based on material testing
- Separate optimization for aluminum, copper, other non-ferrous

---

## 4. Contamination Detection Systems

### 4.1 X-Ray Fluorescence (XRF)

#### 4.1.1 System Specifications

**X-Ray Source**: 40-50 kV tube voltage, 100-1000 μA tube current

**Detection Range**: Elements from Mg (atomic number 12) to U (atomic number 92)

**Measurement Time**: 0.5-3 seconds per measurement point (handheld) or continuous scanning (inline)

**Detection Limits**: 10-100 ppm for heavy metals (matrix dependent)

**Accuracy**: ±0.1-0.5% for major elements, ±10-50 ppm for trace elements

**Safety**: Shielded design, operator exposure <1 μSv per measurement

#### 4.1.2 Applications

**PVC Detection in PET**:
- Chlorine identification (Cl peak at 2.62 keV)
- Detection limit: <50 ppm chlorine
- Inline scanning: 2000-3000 bottles per hour

**Heavy Metal Screening**:
- Lead (Pb) in paint, electronics
- Cadmium (Cd) in plastics, batteries
- Mercury (Hg) in fluorescent lamps, switches
- Chromium (Cr) in treated wood, leather

**Alloy Sorting**:
- Aluminum alloy differentiation (2xxx, 5xxx, 6xxx series)
- Stainless steel grade identification (304, 316, etc.)
- Copper alloy sorting (brass, bronze, copper)

### 4.2 Advanced NIR for Contamination

#### 4.2.1 Hyperspectral Imaging

**Wavelength Bands**: 100-200+ spectral channels across 900-2500 nm range

**Spectral Resolution**: 5-10 nm per band

**Applications**:
- Multi-layer packaging detection
- Food contamination identification
- UV-degraded plastic detection
- Additive and filler identification

**Processing**: GPU-accelerated spectral analysis, <20ms processing latency

#### 4.2.2 Detection Capabilities

**Multi-Layer Materials**:
- Aseptic packaging (aluminum layer detection)
- Laminated films (multiple polymer layers)
- Metallized plastics

**Contamination Types**:
- Food residue on containers
- Oil/grease contamination
- Labels and adhesives exceeding thresholds
- Degraded/discolored materials unsuitable for recycling

### 4.3 Computer Vision and AI

#### 4.3.1 System Architecture

**Cameras**: High-resolution industrial cameras (5+ megapixel)

**Illumination**: Structured LED lighting (multiple angles, wavelengths)

**Computing**: Edge computing (GPU-accelerated) or cloud processing

**Software**: Deep learning models (CNN architectures: ResNet, EfficientNet, custom)

**Training Data**: 5-50 million labeled images per application

**Inference Speed**: 30-120 frames per second, <50ms latency

#### 4.3.2 Recognition Capabilities

**Object Classification**:
- Bottle types (water, juice, milk, etc.)
- Container shapes (bottle, jar, tub, cup)
- Material types (supplement to NIR)
- Brand recognition (for deposit return systems)

**Contamination Detection**:
- Food residue (visual indicators)
- Motor oil bottles (distinctive shape/labeling)
- Household hazmat (cleaners, pesticides, paint)
- Damaged/crushed containers unsuitable for processing

**Quality Assessment**:
- Label coverage (affects washing efficiency)
- Cap/closure presence
- Fill level (containers should be empty)
- Deformation/damage assessment

#### 4.3.3 Performance Requirements

**Accuracy**: ≥96% for trained object classes

**False Positive Rate**: <3% (avoid removing good material)

**False Negative Rate**: <2% for critical hazards (batteries, hazmat)

**Processing Latency**: <20ms from image capture to classification result

**Continuous Learning**: Support for model updates, active learning from operator corrections

---

## 5. Process Control and Automation

### 5.1 SCADA Integration

#### 5.1.1 System Architecture

**Sensors and Instrumentation**:
- Belt scales (load cells, accuracy ±0.5%)
- Optical sensors (material presence detection)
- Encoders (belt speed monitoring)
- Proximity sensors (equipment position)
- Temperature sensors (bearing monitoring)
- Vibration sensors (predictive maintenance)
- Pressure sensors (air system monitoring)
- Current sensors (motor load monitoring)

**PLCs and Controllers**:
- Industrial PLCs (Allen-Bradley, Siemens, or equivalent)
- Distributed I/O for remote sensor connection
- Redundant controllers for critical functions
- Real-time operating systems

**HMI (Human-Machine Interface)**:
- Touchscreen workstations at key control points
- Real-time process visualization
- Alarm management and acknowledgment
- Trend charting and historical data review
- Recipe/configuration management

**Networking**:
- Industrial Ethernet (EtherNet/IP, PROFINET, or equivalent)
- Redundant network architecture
- Secure segmentation (process network, business network)
- Cybersecurity per IEC 62443

#### 5.1.2 Control Functions

**Equipment Coordination**:
- Belt speed synchronization across process
- Start/stop sequencing (prevent jams)
- Load balancing (distribute material flow)
- Automatic bypass of failed equipment

**Quality Control**:
- Real-time purity monitoring
- Automatic sorting adjustments (reject/accept thresholds)
- Material flow routing (high quality vs. reprocessing)
- Contamination alarms and automatic responses

**Safety Systems**:
- Emergency stop circuits
- Interlock management (guards, access doors)
- Fire detection and suppression coordination
- Evacuation alarms

**Performance Monitoring**:
- Throughput tracking (hourly, daily, monthly)
- Recovery rate calculation
- Equipment efficiency (OEE - Overall Equipment Effectiveness)
- Energy consumption monitoring
- Predictive maintenance alerts

### 5.2 Data Acquisition Requirements

#### 5.2.1 Real-Time Data Collection

**Sample Rates**:
- Critical processes: 1-10 Hz
- Standard monitoring: 0.1-1 Hz
- Slow processes: 0.01-0.1 Hz

**Data Points**: Minimum 300-500 tags for medium facility

**Storage**: Local buffering (minimum 7 days), historical database (minimum 2 years)

**Bandwidth**: Minimum 100 Mbps network infrastructure

#### 5.2.2 Data Types

**Analog Measurements**:
- Weights, flows, temperatures, pressures, speeds
- Resolution: 12-bit minimum (4096 discrete values)
- Accuracy: ±0.5% of full scale typical

**Digital States**:
- Equipment on/off, valve positions, alarm states
- Boolean (true/false) representation
- Timestamp resolution: ±100 milliseconds

**Event Logs**:
- Operator actions, alarms, batch records
- Audit trail (who, what, when)
- Non-editable logging (regulatory compliance)

---

## 6. Equipment Specifications

### 6.1 Conveyor Systems

#### 6.1.1 Belt Conveyors

**Belt Material**: 
- Rubber (standard duty)
- PVC/PU (food-safe, easy cleaning)
- Steel mesh (drainage, drying applications)

**Belt Width**: 600mm, 800mm, 1000mm, 1200mm, 1600mm standard sizes

**Belt Speed**: 0.5-4.0 m/s variable (VFD controlled)

**Capacity**: 10-150 tonnes per hour (depends on width, speed, material density)

**Components**:
- Heavy-duty rollers (sealed bearings, minimum L10 life 30,000 hours)
- Belt cleaning systems (primary and secondary scrapers)
- Belt tracking systems (automatic alignment)
- Emergency stops every 15 meters maximum

#### 6.1.2 Disc Screens

**Function**: Separate 2D materials (paper, cardboard) from 3D materials (containers)

**Disc Spacing**: Adjustable 40-120mm

**Shaft Speed**: 30-60 RPM

**Throughput**: 15-40 tonnes per hour per meter width

**Separation Efficiency**: ≥90% for materials with significant 2D/3D difference

### 6.2 Air Classification

#### 6.2.1 Zigzag Classifiers

**Air Velocity**: 3-8 m/s (adjustable for material characteristics)

**Throughput**: 10-30 tonnes per hour

**Separation Efficiency**: ≥92% for materials with density difference >0.3 g/cm³

**Power**: 30-60 kW (fan motor)

**Ductwork**: Minimum 800mm diameter, sealed construction, inspection ports

#### 6.2.2 Ballistic Separators

**Paddle Design**: Oscillating fingers, 50-80mm spacing

**Frequency**: 60-90 cycles per minute

**Inclination**: 15-30 degrees (adjustable)

**Throughput**: 15-35 tonnes per hour

**Fractions**: Simultaneous separation into 2D, 3D, and fines (<50mm)

---

## 7. Installation and Commissioning

### 7.1 Site Preparation

**Structural Requirements**:
- Floor loading: minimum 5 kN/m² for equipment areas
- Elevated platforms: steel frame, anti-slip flooring, guardrails per OSHA
- Vibration isolation: springs or pads under heavy rotating equipment

**Electrical Infrastructure**:
- Transformer capacity: 500-2000 kVA (depends on facility size)
- Distribution: 400V 3-phase 50/60 Hz (or local standard)
- Emergency power: backup generator for critical systems
- Grounding: per NEC Article 250 or local codes

**Compressed Air**:
- Compressor capacity: 200-800 CFM at 125 PSI (depends on optical sorter count)
- Air treatment: filtration (5 micron), drying (-40°C dewpoint), oil removal
- Distribution: aluminum or stainless steel piping, pressure regulation at each user

**HVAC**:
- Equipment rooms: 15-25°C temperature control
- Processing area: dust collection, general ventilation (6-10 air changes per hour)
- Control room: precision HVAC (20±2°C, 50±10% RH)

### 7.2 Commissioning Protocol

**Phase 1: Mechanical (Weeks 1-2)**
1. Equipment installation verification (alignment, leveling, anchoring)
2. Belt tensioning and tracking
3. Lubrication systems check
4. Manual operation tests (jogging, slow speed)
5. Safety systems verification (e-stops, guards, interlocks)

**Phase 2: Electrical (Weeks 2-3)**
1. Power supply verification (voltage, phase, grounding)
2. Motor rotation direction
3. Sensor wiring and calibration
4. Network connectivity
5. HMI functionality

**Phase 3: Integration (Weeks 3-4)**
1. PLC programming verification
2. Sequential startup/shutdown testing
3. Interlock verification
4. Alarm testing
5. SCADA integration

**Phase 4: Process (Weeks 4-6)**
1. Material flow testing (no load, light load, full load)
2. Sorting algorithm calibration
3. Throughput testing
4. Quality verification (purity, recovery rate)
5. Optimization and fine-tuning

**Phase 5: Performance (Week 6-8)**
1. Extended run testing (8+ hour continuous operation)
2. Performance metric verification vs. specifications
3. Operator training
4. Documentation review and turnover
5. Final acceptance

---

## 8. Performance Verification

### 8.1 Acceptance Testing

**Throughput Test**:
- Process design throughput for 4 continuous hours
- Verify no equipment failures or jams
- Confirm output meets quality specifications

**Quality Test**:
- Sample output bales (minimum 10 samples)
- Laboratory analysis of purity and composition
- Compare to specification requirements
- Accept if ≥90% of samples meet specifications

**Reliability Test**:
- 40-hour continuous operation
- Maximum 2 hours total downtime allowed
- No critical equipment failures
- Document all maintenance activities

### 8.2 Key Performance Indicators (KPIs)

**Technical KPIs**:
- Recovery Rate ≥85%
- Purity ≥95%
- Sorting Accuracy ≥98%
- Equipment Uptime ≥92%

**Operational KPIs**:
- Throughput (tonnes per hour)
- Energy per tonne (<45 kWh/t)
- Labor hours per tonne
- Maintenance cost per tonne

**Quality KPIs**:
- Customer rejection rate (<5%)
- Rework rate (<10%)
- Contamination incidents (zero critical)

---

## 9. Maintenance Requirements

### 9.1 Preventive Maintenance Schedule

**Daily**:
- Visual inspection of belts, pulleys, bearings
- Cleaning of optical sensors
- Compressed air system checks (pressure, moisture)
- Lubrication of daily-lube points

**Weekly**:
- Belt tension and tracking verification
- Safety system testing (e-stops, interlocks)
- NIR white reference calibration
- Data backup verification

**Monthly**:
- Bearing temperature and vibration monitoring
- Belt splice inspection
- Air valve testing and cleaning
- Complete system functional test

**Quarterly**:
- NIR full calibration and material library update
- Motor megger testing (insulation resistance)
- Alignment checks (shafts, pulleys)
- Safety system comprehensive test

**Annual**:
- Major component overhaul (bearings, belts, seals)
- Electrical system thermography
- Comprehensive safety audit
- Calibration of all instrumentation

### 9.2 Predictive Maintenance

**Vibration Monitoring**:
- Baseline vibration signatures for all rotating equipment
- Weekly or continuous monitoring
- Alarm thresholds: ±20% from baseline
- Predictive alerts 30-60 days before failure

**Thermal Monitoring**:
- Infrared thermography quarterly
- Continuous bearing temperature monitoring for critical equipment
- Alarm: temperature >20°C above ambient or >10°C increase rate

**Oil Analysis**:
- Quarterly sampling for gearboxes, hydraulic systems
- Measure: viscosity, particle count, water content, acid number
- Trending to predict component wear

---

## 10. Training Requirements

### 10.1 Operator Training

**Duration**: Minimum 80 hours (2 weeks)

**Topics**:
- Process overview and material flow
- Equipment operation and controls
- Quality standards and sampling procedures
- Safety procedures and PPE requirements
- Emergency response protocols
- Troubleshooting common issues
- Documentation and reporting

**Certification**: Written test (≥80% score) + practical demonstration

### 10.2 Maintenance Training

**Duration**: Minimum 120 hours (3 weeks)

**Topics**:
- Mechanical systems and components
- Electrical systems and controls
- Pneumatic systems
- Preventive maintenance procedures
- Troubleshooting and diagnostics
- SCADA and PLC basics
- Calibration procedures
- Safety and lockout/tagout

**Certification**: Written test (≥85% score) + hands-on evaluation

---

## 11. Phase 2 Compliance Checklist

### 11.1 Equipment Installation
- [ ] Optical sorting systems installed and operational
- [ ] Electromagnetic separation installed
- [ ] Contamination detection systems operational
- [ ] Conveyor systems upgraded/installed
- [ ] Air classification systems installed
- [ ] All equipment properly commissioned

### 11.2 Process Control
- [ ] SCADA system installed and integrated
- [ ] Real-time data acquisition functional
- [ ] Process control algorithms implemented
- [ ] HMI workstations operational
- [ ] Historical data storage configured

### 11.3 Performance
- [ ] Recovery rate ≥85% achieved
- [ ] Purity ≥95% achieved
- [ ] Sorting accuracy ≥98% verified
- [ ] Throughput targets met
- [ ] Energy efficiency targets met

### 11.4 Training and Documentation
- [ ] Operator training completed and certified
- [ ] Maintenance training completed and certified
- [ ] Operating procedures documented
- [ ] Maintenance procedures documented
- [ ] As-built drawings completed

### 11.5 Verification
- [ ] Performance testing completed
- [ ] KPIs documented and meeting targets
- [ ] Third-party audit passed (if applicable)
- [ ] Management review and approval

---

**Document Control**

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-01-15 | WIA Standards Committee | Initial release |

**Copyright** © 2025 World Industry Association  
**License**: WIA Open Standard License v1.0  
**弘益人間** · Benefit All Humanity
