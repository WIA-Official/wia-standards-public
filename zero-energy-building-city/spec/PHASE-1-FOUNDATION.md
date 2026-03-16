# WIA-CITY-005: Zero Energy Building - PHASE 1 FOUNDATION

**Version:** 1.0
**Status:** Active
**Category:** CITY
**Last Updated:** 2025-12-25

---

## 1. Overview

### 1.1 Purpose

The WIA-CITY-005 standard defines a comprehensive framework for designing, implementing, certifying, and operating Net Zero Energy Buildings (NZEB). This standard establishes:

- Energy production and consumption data formats
- ZEB certification protocols and grading systems
- Renewable energy integration specifications
- Carbon neutrality verification methods
- Building performance optimization algorithms

### 1.2 Scope

This standard applies to:

- **Residential Buildings**: Single-family homes, apartments, condominiums
- **Commercial Buildings**: Offices, retail spaces, hotels
- **Industrial Buildings**: Manufacturing facilities, warehouses
- **Institutional Buildings**: Schools, hospitals, government buildings
- **Mixed-Use Developments**: Combined residential and commercial spaces

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)**

Zero energy buildings represent a fundamental shift towards sustainable architecture that benefits humanity by:
- Reducing carbon emissions and combating climate change
- Lowering energy costs for occupants
- Improving indoor air quality and occupant health
- Demonstrating viable pathways to carbon neutrality
- Creating resilient, self-sufficient communities

---

## 2. Key Concepts

### 2.1 Net Zero Energy Building (NZEB)

A building that produces as much energy as it consumes over a year through on-site renewable energy generation.

**Energy Balance Formula:**
```
Annual Energy Production ≥ Annual Energy Consumption
```

### 2.2 ZEB Grades (1-5)

| Grade | Balance Ratio | Description | Characteristics |
|-------|---------------|-------------|-----------------|
| **1** | ≥ 120% | Premium | Energy positive, exports surplus to grid |
| **2** | 100-120% | Excellent | Net zero energy balance achieved |
| **3** | 80-100% | Good | Near zero, minimal grid dependency |
| **4** | 60-80% | Fair | Significant renewable integration |
| **5** | < 60% | Needs Improvement | Requires optimization |

**Balance Ratio Calculation:**
```
Balance Ratio = (Annual Production / Annual Consumption) × 100%
```

### 2.3 Energy Sources

#### On-Site Renewable Energy
- **Solar Photovoltaic (PV)**: Rooftop and facade-mounted panels
- **Solar Thermal**: Hot water and space heating
- **Wind**: Small-scale turbines for suitable locations
- **Geothermal**: Ground-source heat pumps
- **Biomass**: Sustainable fuel sources (where applicable)

#### Energy Storage
- **Battery Energy Storage Systems (ESS)**: Lithium-ion, flow batteries
- **Thermal Storage**: Hot water tanks, phase change materials
- **Mechanical Storage**: Compressed air, flywheels (large buildings)

### 2.4 Passive Design Principles

ZEB certification requires integration of passive design strategies:

1. **Building Orientation**: Optimal solar exposure and natural ventilation
2. **Insulation**: High-performance envelope (walls, roof, foundation)
3. **Windows**: Triple-glazed, low-E coatings, proper sizing
4. **Thermal Mass**: Strategic use of materials to regulate temperature
5. **Natural Ventilation**: Cross-ventilation, stack effect
6. **Daylighting**: Maximize natural light to reduce artificial lighting
7. **Shading**: Overhangs, louvers, vegetation to prevent overheating

---

## 3. System Architecture

### 3.1 Core Components

```
┌─────────────────────────────────────────────────────────────┐
│                    ZERO ENERGY BUILDING                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐      ┌──────────────┐                   │
│  │   PRODUCTION │      │  CONSUMPTION │                   │
│  ├──────────────┤      ├──────────────┤                   │
│  │ • Solar PV   │      │ • HVAC       │                   │
│  │ • Wind       │◄────►│ • Lighting   │                   │
│  │ • Geothermal │      │ • Equipment  │                   │
│  │ • Solar Thml │      │ • Appliances │                   │
│  └──────────────┘      └──────────────┘                   │
│         │                      │                           │
│         └──────────┬───────────┘                           │
│                    │                                       │
│         ┌──────────▼──────────┐                           │
│         │   ENERGY STORAGE    │                           │
│         │      (ESS/BMS)      │                           │
│         └──────────┬──────────┘                           │
│                    │                                       │
│         ┌──────────▼──────────┐                           │
│         │  SMART GRID         │                           │
│         │  (Import/Export)    │                           │
│         └─────────────────────┘                           │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│              MONITORING & CERTIFICATION                     │
│  • Real-time Energy Monitoring                             │
│  • Performance Analytics                                   │
│  • ZEB Grade Calculation                                   │
│  • Carbon Footprint Tracking                               │
│  • Verifiable Credentials (VC)                             │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 Data Flow

1. **Production Metering**: Real-time monitoring of renewable energy generation
2. **Consumption Metering**: Detailed tracking of building energy use
3. **Storage Management**: ESS charge/discharge optimization
4. **Grid Interaction**: Import/export balancing
5. **Data Aggregation**: Centralized energy management system
6. **Analytics**: Performance calculation and optimization
7. **Certification**: ZEB grade assignment and VC issuance

---

## 4. Technical Requirements

### 4.1 Metering Infrastructure

#### Production Meters
- **Accuracy**: ±2% or better
- **Sampling Rate**: Minimum 1 minute intervals
- **Data Storage**: Minimum 2 years historical data
- **Protocol Support**: Modbus, BACnet, MQTT, or RESTful API

#### Consumption Meters
- **Granularity**: Sub-metering by major load categories
- **Real-time Monitoring**: < 5 second latency
- **Power Quality**: Voltage, current, power factor monitoring
- **Integration**: Building Management System (BMS) compatible

### 4.2 Energy Storage Systems

#### Minimum Requirements
- **Capacity**: ≥ 2 hours of average building load
- **Round-trip Efficiency**: ≥ 85%
- **Cycle Life**: ≥ 5,000 cycles
- **Safety**: UL 9540, IEC 62619 compliance
- **Warranty**: Minimum 10 years

#### Management Features
- **Peak Shaving**: Load leveling during high-demand periods
- **Load Shifting**: Time-of-use optimization
- **Backup Power**: Emergency power during outages
- **Grid Services**: Frequency regulation, demand response

### 4.3 Building Envelope Performance

#### Thermal Performance
- **Walls**: R-value ≥ R-30 (5.3 m²·K/W)
- **Roof**: R-value ≥ R-50 (8.8 m²·K/W)
- **Foundation**: R-value ≥ R-20 (3.5 m²·K/W)
- **Windows**: U-value ≤ 0.20 W/m²·K
- **Air Tightness**: ≤ 0.6 ACH50 (air changes per hour at 50 Pa)

#### Ventilation
- **Heat Recovery**: ≥ 85% efficiency (ERV/HRV)
- **Fresh Air**: Minimum as per ASHRAE 62.1 or equivalent
- **Filtration**: MERV 13 or higher

---

## 5. Certification Process

### 5.1 Pre-Certification Phase

1. **Design Review**
   - Energy modeling and simulation
   - Passive design strategy verification
   - Renewable energy system sizing
   - Expected performance calculation

2. **Documentation Submission**
   - Building plans and specifications
   - Energy system designs
   - Expected annual production/consumption
   - Passive design features

### 5.2 Construction Phase

1. **Quality Assurance**
   - Insulation installation verification
   - Air tightness testing (blower door test)
   - HVAC system commissioning
   - Window and door installation QA

2. **System Installation**
   - Renewable energy system installation
   - ESS installation and testing
   - Metering infrastructure setup
   - BMS/EMS configuration

### 5.3 Performance Verification Phase

**Duration**: 12 continuous months

1. **Data Collection**
   - Monthly energy production data
   - Monthly energy consumption data
   - Grid import/export records
   - System performance metrics

2. **Analysis**
   - Energy balance calculation
   - Carbon footprint assessment
   - System efficiency verification
   - Performance degradation monitoring

3. **Third-Party Audit**
   - Independent verification of data
   - On-site inspection
   - System testing and validation
   - Documentation review

### 5.4 Certificate Issuance

Upon successful verification:

1. **ZEB Grade Assignment** (1-5 based on performance)
2. **Energy Performance Certificate** (digital and physical)
3. **Verifiable Credential** (blockchain-anchored)
4. **QR Code** (links to public dashboard)
5. **Performance Report** (detailed analysis)

### 5.5 Ongoing Monitoring

- **Annual Reporting**: Performance data submission
- **Re-certification**: Every 3 years
- **Continuous Monitoring**: Real-time dashboard access
- **Grade Updates**: Based on ongoing performance

---

## 6. Compliance Standards

### 6.1 International Standards

- **ISO 52000**: Energy performance of buildings
- **EN 15603**: Energy performance calculation methodology
- **ASHRAE 90.1**: Energy standard for buildings
- **Passive House**: PHIUS+ or PHI certification compatible
- **LEED**: ZEB credit contribution
- **BREEAM**: Ene 01 energy performance credit

### 6.2 Safety Standards

- **Electrical**: IEC 60364, NEC (National Electrical Code)
- **Fire Safety**: NFPA 855 (ESS), local building codes
- **Structural**: Load calculations for solar panels, wind turbines
- **Environmental**: ISO 14001 compatible

### 6.3 Data Standards

- **Energy Data**: IEEE 1547 (grid interconnection)
- **BMS Protocols**: BACnet, Modbus, LonWorks
- **IoT**: MQTT, CoAP for sensor networks
- **Verifiable Credentials**: W3C VC Data Model

---

## 7. Performance Metrics

### 7.1 Primary Metrics

| Metric | Formula | Unit | Target |
|--------|---------|------|--------|
| Energy Balance Ratio | Production / Consumption | % | ≥ 100% |
| Self-Sufficiency | (1 - Grid Import / Total Consumption) × 100 | % | ≥ 80% |
| Energy Use Intensity (EUI) | Annual Consumption / Floor Area | kWh/m²/year | ≤ 50 |
| Renewable Fraction | Renewable Production / Total Consumption | % | ≥ 100% |
| Carbon Intensity | Net CO₂ Emissions / Floor Area | kg CO₂/m²/year | ≤ 0 |

### 7.2 Secondary Metrics

- **Peak Load Reduction**: Percentage decrease from baseline
- **ESS Utilization**: Charge/discharge cycles per year
- **Grid Independence**: Hours per year on ESS backup
- **Solar Performance Ratio**: Actual / Expected production
- **HVAC Efficiency**: COP/EER of heating/cooling systems

---

## 8. Example Use Cases

### 8.1 Single-Family Residence

**Building Profile:**
- Floor Area: 200 m²
- Occupants: 4
- Location: Temperate climate

**Energy Systems:**
- 10 kW Solar PV (40 panels × 250W)
- 20 kWh ESS (Lithium-ion battery)
- 5 kW Geothermal heat pump
- High-efficiency appliances (ENERGY STAR)

**Performance:**
- Annual Production: 14,000 kWh
- Annual Consumption: 13,500 kWh
- Balance Ratio: 103.7%
- **ZEB Grade: 2 (Excellent)**

### 8.2 Office Building

**Building Profile:**
- Floor Area: 5,000 m²
- Occupants: 250
- Location: Urban setting

**Energy Systems:**
- 250 kW Rooftop Solar PV
- 500 kWh ESS
- VRF HVAC system with heat recovery
- LED lighting with occupancy sensors
- Smart building controls

**Performance:**
- Annual Production: 312,500 kWh
- Annual Consumption: 250,000 kWh
- Balance Ratio: 125%
- **ZEB Grade: 1 (Premium)**

### 8.3 Mixed-Use Development

**Building Profile:**
- Floor Area: 15,000 m² (60% residential, 40% commercial)
- Occupants: 500 residents + 200 workers
- Location: Suburban

**Energy Systems:**
- 500 kW Solar PV (rooftop + facade)
- 50 kW Wind turbines (2 × 25 kW)
- 1 MWh Community ESS
- District heating/cooling with geothermal
- EV charging infrastructure (solar-powered)

**Performance:**
- Annual Production: 780,000 kWh
- Annual Consumption: 750,000 kWh
- Balance Ratio: 104%
- **ZEB Grade: 2 (Excellent)**

---

## 9. Benefits

### 9.1 Environmental

- **Carbon Neutrality**: Net zero or negative carbon emissions
- **Resource Conservation**: Reduced fossil fuel dependence
- **Habitat Preservation**: Lower environmental impact
- **Climate Resilience**: Distributed, renewable energy sources

### 9.2 Economic

- **Energy Cost Savings**: 70-100% reduction in utility bills
- **Property Value**: 10-20% premium for ZEB-certified buildings
- **Incentives**: Access to tax credits, grants, rebates
- **Long-term ROI**: Payback period typically 7-15 years
- **Grid Services Revenue**: Income from energy exports

### 9.3 Social

- **Health**: Improved indoor air quality and thermal comfort
- **Energy Security**: Resilience during grid outages
- **Community**: Demonstration of sustainable practices
- **Education**: Learning opportunities for occupants and visitors
- **Employment**: Green jobs in design, construction, and maintenance

---

## 10. Implementation Roadmap

### Phase 1: Foundation (Months 1-3)
- ✅ Standard development and publication
- ✅ Certification body establishment
- ✅ Training program for auditors
- ✅ Software tools for energy modeling

### Phase 2: Pilot Projects (Months 4-12)
- 🚀 10-20 pilot buildings across different typologies
- 🚀 Data collection and analysis
- 🚀 Standard refinement based on feedback
- 🚀 Case study publication

### Phase 3: Scale-Up (Year 2-3)
- 📈 100+ certified buildings
- 📈 Integration with local building codes
- 📈 Industry partnerships (manufacturers, utilities)
- 📈 Public awareness campaigns

### Phase 4: Mainstream Adoption (Year 3+)
- 🌍 1,000+ certified buildings globally
- 🌍 Policy advocacy for ZEB requirements
- 🌍 Continuous standard updates
- 🌍 International harmonization

---

## 11. Conclusion

WIA-CITY-005 provides a comprehensive, practical framework for achieving net zero energy buildings at scale. By combining rigorous performance standards with flexible implementation pathways, this standard enables:

- **Measurable Impact**: Clear metrics for energy balance and carbon neutrality
- **Technology Neutrality**: Support for diverse renewable energy sources
- **Economic Viability**: Balance between performance and cost-effectiveness
- **Scalability**: Applicable to buildings of all sizes and types
- **Transparency**: Public verification through QR codes and VCs

**Together, we can transform the built environment into a positive force for climate action.**

---

**弘益人間 · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
WIA-CITY-005 v1.0
https://wia.official/standards/CITY/005
