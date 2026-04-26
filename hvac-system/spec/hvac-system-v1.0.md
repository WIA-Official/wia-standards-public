# WIA-CITY-010: Smart HVAC System Standard v1.0

## Executive Summary

The WIA-CITY-010 standard defines requirements for intelligent Heating, Ventilation, and Air Conditioning (HVAC) systems in smart city environments. This standard enables energy-efficient, adaptive climate control through IoT sensors, AI optimization, and integrated building automation.

**Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity

## 1. Scope and Purpose

### 1.1 Scope
This standard applies to:
- Commercial buildings (offices, retail, hospitality)
- Residential multi-unit buildings
- Industrial facilities
- Healthcare and educational institutions
- Smart city infrastructure

### 1.2 Purpose
- Reduce HVAC energy consumption by 30-50%
- Improve indoor air quality and occupant comfort
- Enable predictive maintenance and fault detection
- Integrate with smart city energy grids
- Support renewable energy integration

## 2. Normative References

- ASHRAE Standard 62.1: Ventilation for Acceptable Indoor Air Quality
- ASHRAE Standard 90.1: Energy Standard for Buildings
- ISO 16484: Building Automation and Control Systems (BACS)
- BACnet (ISO 16484-5): Communication Protocol
- WELL Building Standard v2.0
- LEED v4.1 Building Design and Construction

## 3. System Architecture

###  3.1 Control Hierarchy
- **Level 1**: Field Devices (sensors, actuators, thermostats)
- **Level 2**: Zone Controllers (VAV boxes, fan coil units)
- **Level 3**: Building Automation System (BAS)
- **Level 4**: Enterprise/Cloud Management

### 3.2 Required Components
1. Temperature sensors (±0.5°C accuracy)
2. Humidity sensors (±3% RH accuracy)
3. CO2 sensors (±50 ppm accuracy)
4. Occupancy sensors (95%+ detection rate)
5. Particulate matter sensors (PM2.5)
6. Smart thermostats with remote control
7. Variable speed drives on fans and pumps
8. Automated dampers and actuators

## 4. Certification Levels

### Bronze Level
- Basic IoT sensors and programmable thermostats
- Remote monitoring capabilities
- Scheduled occupancy modes
- **Energy Savings**: 10-20%

### Silver Level
- Zone-based control with occupancy sensing
- Demand-controlled ventilation (CO2-based)
- Basic air quality monitoring
- **Energy Savings**: 20-35%

### Gold Level
- AI-driven optimization algorithms
- Predictive maintenance with AFDD
- Advanced IAQ management (VOCs, PM2.5)
- Building Management System integration
- **Energy Savings**: 35-50%

### Platinum Level
- Machine learning and reinforcement learning control
- Demand response participation
- Full ecosystem integration (lighting, security, energy)
- Pathogen detection and air disinfection
- **Energy Savings**: 50%+

## 5. Indoor Air Quality Requirements

### 5.1 Mandatory Parameters
| Parameter | Target Range | Measurement Frequency |
|-----------|--------------|----------------------|
| CO2 | <1000 ppm | Continuous |
| Temperature | 20-24°C (68-75°F) | Continuous |
| Relative Humidity | 40-60% | Continuous |
| PM2.5 | <12 µg/m³ | Continuous |
| VOCs | <500 µg/m³ | Hourly |

### 5.2 Ventilation
- Minimum outdoor air per ASHRAE 62.1
- Demand-controlled ventilation based on CO2
- Filtration: Minimum MERV 13 (Gold/Platinum)

## 6. Energy Efficiency Requirements

### 6.1 Optimization Strategies (Mandatory for Silver+)
- Optimal start/stop algorithms
- Economizer control (free cooling)
- Supply air temperature reset
- Static pressure reset
- Chilled/hot water temperature reset

### 6.2 Performance Metrics
- Chiller efficiency: <0.65 kW/ton (Gold+)
- Fan power: <0.40 W/CFM (Silver+)
- Boiler efficiency: >85% (condensing preferred)

## 7. Communication Protocols

### 7.1 Required Support (Silver+)
- BACnet/IP or BACnet MS/TP
- Modbus TCP or Modbus RTU
- RESTful API for cloud connectivity

### 7.2 Data Exchange
- Real-time sensor data (15-minute intervals minimum)
- Equipment status and alarms
- Energy consumption metrics
- Performance analytics

## 8. Predictive Maintenance (Gold+)

### 8.1 Automated Fault Detection
- Simultaneous heating and cooling
- Economizer stuck/failed
- Sensor calibration drift
- Scheduling errors
- Excessive outdoor air intake
- Short cycling detection

### 8.2 Monitoring Techniques
- Vibration analysis (rotating equipment)
- Thermography (electrical and thermal)
- Power quality monitoring
- Refrigerant analysis
- Performance trending

## 9. Cybersecurity Requirements

### 9.1 Network Security
- Network segmentation (HVAC isolated from IT)
- Encrypted communications (TLS 1.2+)
- Strong authentication (multi-factor for remote access)
- Regular security updates

### 9.2 Data Protection
- Occupancy data anonymization
- Privacy-by-design principles
- Transparent data collection policies

## 10. Implementation and Testing

### 10.1 Commissioning Requirements
- Functional performance testing
- Sensor calibration verification
- Control sequence validation
- Energy baseline establishment
- IAQ compliance verification

### 10.2 Continuous Commissioning
- Quarterly performance reviews
- Annual sensor calibration
- Ongoing fault detection monitoring

## 11. Documentation Requirements

### 11.1 Mandatory Documentation
- System architecture diagrams
- Control sequence narratives
- Sensor placement plans
- Maintenance schedules
- Commissioning reports
- Energy performance tracking

## 12. Compliance and Certification

### 12.1 Certification Process
1. Submit system design for review
2. Install and commission system
3. Operate for 3-month baseline period
4. Third-party performance verification
5. Annual recertification

### 12.2 Verification Metrics
- Energy savings vs. baseline
- IAQ compliance percentage
- System uptime
- Occupant satisfaction surveys

## Appendix A: API Specification

See `api/typescript/` for reference implementation.

## Appendix B: Typical Use Cases

1. **Commercial Office Building**: 50,000 sq ft, Gold certification, 35% energy savings
2. **Hospital**: Platinum certification, advanced IAQ, pathogen control
3. **School**: Silver certification, CO2-based ventilation, 25% savings
4. **Residential Tower**: Bronze/Silver, zone control, 20% savings

---

**Version**: 1.0
**Published**: 2025
**Maintained by**: WIA (World Certification Industry Association)
**Philosophy**: 弘익人間 · Benefit All Humanity

© 2025 SmileStory Inc. / WIA
