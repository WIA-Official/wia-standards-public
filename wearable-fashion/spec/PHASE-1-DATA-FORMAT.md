# PHASE 1 — Data Format

> Wearable-fashion canonical envelopes: garment, sub-system,
> sensor, actuator, and the architecture map plus technical
> requirements that frame every other phase. The appendices
> defining material catalogs, certification schedules, and
> reference patterns also live here because they are reference
> data, not behavior.

## Document Information

| Field | Value |
|-------|-------|
| **Title** | Wearable Fashion Standard |
| **Standard ID** | WIA-IND-003 |
| **Version** | 1.0.0 |
| **Authors** | WIA Fashion Technology Research Group |
| **Contributors** | Fashion designers, electronics engineers, textile specialists |
| **Status** | Active |
| **License** | MIT |
| **Copyright** | © 2025 SmileStory Inc. / WIA |

---


## Abstract

**弘익人間 (홍익인간) - Benefit All Humanity**

The WIA-IND-003 Wearable Fashion Standard establishes a comprehensive technical framework for integrating electronic components, sensors, displays, and interactive elements into clothing and accessories. This standard addresses the design, manufacturing, power management, user interaction, safety, and sustainability aspects of wearable fashion technology.

As technology becomes increasingly wearable, this standard aims to democratize tech-integrated fashion, making it accessible, safe, sustainable, and beneficial for all. From LED-embedded evening gowns to health-monitoring athletic wear, from heated jackets for extreme climates to solar-powered accessories, this standard provides the technical foundation for the future of fashion.

---


## 1. Introduction

### 1.1 Purpose

This standard provides a unified technical framework for wearable fashion technology, enabling:
- Consistent design and manufacturing practices
- Interoperability between components and systems
- Safety and reliability standards
- Sustainable and ethical production methods
- Innovation in fashion technology

### 1.2 Philosophy

**弘益人間 (홍익인간)** - "Broadly benefiting humanity"

Wearable fashion technology should:
- Enhance personal expression and creativity
- Improve quality of life through smart functionality
- Be accessible to people of all backgrounds
- Respect environmental sustainability
- Prioritize user safety and comfort

### 1.3 Target Audience

- Fashion designers and brands
- Electronics engineers
- Textile manufacturers
- Product developers
- Quality assurance teams
- Certification bodies

---


## 2. Scope

### 2.1 Included

This standard covers:
- Smart jewelry (rings, bracelets, necklaces, earrings)
- LED and light-emitting garments
- Interactive clothing (touch-sensitive, gesture-controlled)
- Thermal regulation clothing (heating/cooling)
- Health monitoring fashion
- Energy harvesting garments
- Fashion accessories with integrated technology

### 2.2 Excluded

This standard does not cover:
- Purely functional wearable devices (smartwatches, fitness trackers)
- Medical devices requiring regulatory approval
- Military/tactical equipment
- Industrial protective clothing
- Virtual/augmented reality fashion

---


## 3. Normative References

The following standards are referenced:
- IEC 60529: IP Code (Ingress Protection)
- ISO 105: Textiles - Tests for color fastness
- OEKO-TEX Standard 100: Textile safety
- ISO 12402: Personal flotation devices
- IEC 62368-1: Audio/video equipment safety
- Bluetooth SIG specifications
- USB-IF specifications
- Qi wireless charging standard
- FCC Part 15: Radio frequency devices

---


## 4. Terms and Definitions

### 4.1 General Terms

**Wearable Fashion**: Clothing or accessories that integrate electronic components for aesthetic, functional, or interactive purposes.

**Smart Jewelry**: Jewelry items (rings, bracelets, necklaces) with embedded electronics, sensors, or displays.

**E-Textile**: Fabric with integrated electronic components or conductive materials.

**Conductive Thread**: Thread made with or coated with conductive materials for carrying electrical current.

**LED Garment**: Clothing item incorporating light-emitting diodes for illumination or display.

### 4.2 Technical Terms

**Duty Cycle**: Percentage of time a component is active vs. total time.

**IP Rating**: Ingress Protection rating indicating resistance to dust and water.

**mAh**: Milliampere-hour, unit of battery capacity.

**PWM**: Pulse Width Modulation, method for controlling LED brightness.

**Washability**: Ability of a garment to withstand cleaning processes.

**SAR**: Specific Absorption Rate, measure of electromagnetic field absorption.

---


## 5. Wearable Fashion Architecture

### 5.1 System Components

```
┌─────────────────────────────────────────────────────────┐
│                 Wearable Fashion System                 │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐            │
│  │  Power   │  │  Control │  │ Display/ │            │
│  │  System  │──│   Unit   │──│  Output  │            │
│  └──────────┘  └──────────┘  └──────────┘            │
│       │              │              │                  │
│       │              │              │                  │
│  ┌────▼────┐   ┌────▼────┐   ┌────▼────┐            │
│  │ Battery │   │ Sensors │   │   LED/  │            │
│  │Charging │   │  Input  │   │ Haptic  │            │
│  └─────────┘   └─────────┘   └─────────┘            │
│                                                         │
│  ┌──────────────────────────────────────────┐         │
│  │       Textile/Fashion Integration        │         │
│  └──────────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────┘
```

### 5.2 Component Layers

1. **Fashion Layer**: Visible fabric, design elements
2. **Functional Layer**: Sensors, LEDs, displays
3. **Power Layer**: Battery, charging, power distribution
4. **Control Layer**: Microcontroller, wireless communication
5. **Structural Layer**: Support, mounting, protection

### 5.3 Design Principles

- **Modularity**: Components should be replaceable
- **Flexibility**: Design should accommodate body movement
- **Washability**: Electronics should be removable or waterproof
- **Comfort**: No sharp edges, proper weight distribution
- **Aesthetics**: Technology should enhance, not detract from design

---


## 6. Technical Requirements

### 6.1 Electrical Specifications

#### 6.1.1 Voltage Ranges

| Component Type | Nominal Voltage | Operating Range | Maximum |
|----------------|----------------|-----------------|---------|
| Low Power LEDs | 3.3V | 2.8-3.6V | 5V |
| Microcontroller | 3.3V | 2.5-3.6V | 5V |
| High Power LEDs | 5V | 4.5-5.5V | 12V |
| Heating Elements | 5-12V | 4-13V | 24V |
| Motors/Actuators | 3.3-5V | 3-6V | 12V |

#### 6.1.2 Current Specifications

- Maximum current per conductive thread: 1A
- Maximum current per LED: 60mA (RGB), 20mA (single color)
- Total garment current: <5A for wearable items
- Standby current: <1mA

#### 6.1.3 Power Consumption Classes

| Class | Power Range | Application | Battery Life |
|-------|-------------|-------------|--------------|
| Ultra Low | 0-100mW | Smart jewelry, sensors | 7-30 days |
| Low | 100mW-1W | Notification devices | 1-7 days |
| Medium | 1-5W | LED accessories | 4-24 hours |
| High | 5-20W | LED clothing | 2-8 hours |
| Very High | 20-100W | Heated garments | 1-4 hours |

### 6.2 Mechanical Requirements

#### 6.2.1 Flexibility

- Bend radius: Minimum 10mm for rigid components
- Flex cycles: >10,000 for conductive traces
- Stretch: Up to 30% for elastic integration

#### 6.2.2 Durability

- Abrasion resistance: ISO 12947 >10,000 cycles
- Tear resistance: ISO 13937 >25N
- Seam strength: ISO 13935 >300N

### 6.3 Environmental Requirements

#### 6.3.1 Operating Conditions

- Temperature: -10°C to +50°C
- Humidity: 10% to 90% non-condensing
- Altitude: Up to 3000m

#### 6.3.2 Storage Conditions

- Temperature: -20°C to +60°C
- Humidity: 5% to 95% non-condensing
- Battery storage: 40-60% charge level

---


## 17. Appendices

### Appendix A: Material Properties

**Conductive Materials**

| Material | Resistivity (Ω·m) | Conductivity (%IACS) | Flexibility | Cost |
|----------|-------------------|---------------------|-------------|------|
| Silver | 1.59×10⁻⁸ | 105 | Poor (pure) | Very High |
| Copper | 1.68×10⁻⁸ | 100 | Moderate | Moderate |
| Gold | 2.44×10⁻⁸ | 70 | Poor | Extremely High |
| Aluminum | 2.82×10⁻⁸ | 61 | Good | Low |
| Stainless Steel | 6.9×10⁻⁷ | 2.5 | Excellent | Low |
| Conductive Polymer | 10⁻³-10⁻⁵ | <0.01 | Excellent | Moderate |

**Textile Properties**

| Property | Cotton | Polyester | Nylon | Wool | Silk |
|----------|--------|-----------|-------|------|------|
| Tensile Strength | 3-5 g/den | 4-6 g/den | 4-7 g/den | 1-2 g/den | 3-5 g/den |
| Moisture Regain | 8.5% | 0.4% | 4% | 14% | 11% |
| Melting Point | Decomposes | 260°C | 220°C | Decomposes | Decomposes |
| Elasticity | Low | Medium | High | High | Low |
| Washability | Excellent | Excellent | Excellent | Delicate | Delicate |

### Appendix B: Power Calculations Reference

**Battery Capacity Conversion**
```
1 Ah = 1000 mAh
Wh = V × Ah
Example: 3.7V 1000mAh = 3.7Wh
```

**LED Power Calculation**
```
Power per LED (W) = Voltage (V) × Current (A)
Total Power (W) = Power per LED × Number of LEDs × Duty Cycle × Brightness

Example:
50 LEDs, 5V, 20mA each, 50% brightness, 30% duty cycle
Power = (5 × 0.02) × 50 × 0.5 × 0.3 = 0.75W
```

**Heating Power Calculation**
```
Power (W) = Voltage² / Resistance
Power (W) = Current² × Resistance
Power (W) = Voltage × Current

Example:
12V, 20Ω heating element
Power = 12² / 20 = 7.2W
```

### Appendix C: Sizing Guidelines

**Battery Sizing**
```
Required Capacity (mAh) = (Average Current × Operating Time) / (DoD × Efficiency)

Example:
100mA average, 10 hour target, 0.8 DoD, 0.9 efficiency
Capacity = (100 × 10) / (0.8 × 0.9) = 1389 mAh
Select: 1500-2000 mAh battery
```

**Conductive Thread Sizing**
```
Minimum Strands = Required Current / Current per Strand

Example:
500mA required, thread rated 100mA per strand
Strands = 500 / 100 = 5 strands minimum
Use: 6-8 strands for safety margin
```

### Appendix D: Common Failure Modes

**Electrical Failures**
```
1. Thread breakage: Excessive flexing, poor routing
   Solution: Larger bend radius, strain relief

2. Corrosion: Moisture, sweat, washing
   Solution: Better waterproofing, corrosion-resistant materials

3. Short circuit: Frayed threads, damaged insulation
   Solution: Proper insulation, protective routing

4. Battery degradation: Overcharge, deep discharge, heat
   Solution: BMS, proper charging, thermal management
```

**Mechanical Failures**
```
1. Component detachment: Weak bonding, mechanical stress
   Solution: Stronger attachment, stress distribution

2. Fabric tearing: Concentrated stress, poor reinforcement
   Solution: Reinforcement patches, load distribution

3. Waterproofing failure: Seal degradation, mechanical damage
   Solution: Robust sealing method, protection from wear
```

### Appendix E: Troubleshooting Guide

**LED Issues**
```
Problem: LEDs not lighting
- Check power voltage and current
- Verify data line connection
- Test with simple single-color pattern
- Check LED orientation

Problem: Flickering
- Insufficient power supply
- Poor ground connection
- EMI interference
- Code timing issues

Problem: Wrong colors
- Incorrect LED type in software
- Damaged LEDs
- Voltage drop in long runs
- Color calibration needed
```

**Power Issues**
```
Problem: Short battery life
- Excessive current draw (measure with ammeter)
- Poor sleep mode implementation
- Battery degradation (check capacity)
- Inefficient code

Problem: Not charging
- Check charger voltage/current
- Verify charging circuit
- Test battery with multimeter
- Check temperature (charging disabled if too hot/cold)
```

### Appendix F: Regulatory Compliance Checklist

**Pre-Market Requirements**
```
☐ Electrical safety testing (IEC 62368-1)
☐ RF certification (FCC, CE, IC if wireless)
☐ Battery safety (IEC 62133, UN 38.3 for shipping)
☐ Textile safety (OEKO-TEX or equivalent)
☐ Flammability testing (16 CFR 1610 if applicable)
☐ Labeling requirements (care instructions, warnings)
☐ User manual (safety instructions, specifications)
☐ Declaration of Conformity (CE)
☐ Country-specific requirements (e.g., CCC for China)
```

### Appendix G: Glossary

**IACS**: International Annealed Copper Standard, reference for electrical conductivity

**DoD**: Depth of Discharge, percentage of battery capacity used

**EMI**: Electromagnetic Interference

**ESD**: Electrostatic Discharge

**GOTS**: Global Organic Textile Standard

**IMU**: Inertial Measurement Unit (accelerometer + gyroscope + magnetometer)

**IP Code**: Ingress Protection rating for dust and water resistance

**LED**: Light Emitting Diode

**LiPo**: Lithium Polymer battery

**MCU**: Microcontroller Unit

**NFC**: Near Field Communication

**NTC**: Negative Temperature Coefficient (thermistor)

**OEKO-TEX**: International textile testing and certification standard

**PCM**: Phase Change Material

**PPG**: Photoplethysmography (optical heart rate sensing)

**PWM**: Pulse Width Modulation

**RoHS**: Restriction of Hazardous Substances

**SAR**: Specific Absorption Rate (RF energy absorption)

**SOC**: State of Charge (battery level)

**SOH**: State of Health (battery degradation)

**TEG**: Thermoelectric Generator

---


