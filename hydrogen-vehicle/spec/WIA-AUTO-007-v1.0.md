# WIA-AUTO-007: Hydrogen Vehicle Specification v1.0

> **Standard ID:** WIA-AUTO-007
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Fuel Cell Technology](#2-fuel-cell-technology)
3. [Hydrogen Storage Systems](#3-hydrogen-storage-systems)
4. [Power Electronics](#4-power-electronics)
5. [Electric Motor Systems](#5-electric-motor-systems)
6. [Refueling Infrastructure](#6-refueling-infrastructure)
7. [Efficiency Calculations](#7-efficiency-calculations)
8. [Data Formats](#8-data-formats)
9. [API Interface](#9-api-interface)
10. [Safety Protocols](#10-safety-protocols)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the technical framework for hydrogen fuel cell vehicles (FCVs), based on proven automotive engineering principles, electrochemical science, and hydrogen energy systems.

### 1.2 Scope

The standard covers:
- Proton Exchange Membrane Fuel Cell (PEMFC) technology
- Solid Oxide Fuel Cell (SOFC) technology
- High-pressure hydrogen storage (Type III & IV tanks)
- Power electronics and motor control systems
- Refueling protocols and infrastructure
- Efficiency metrics and calculations
- Safety systems and protocols

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to accelerate the adoption of zero-emission hydrogen vehicles that benefit all of humanity through clean transportation while ensuring safety, efficiency, and interoperability.

### 1.4 Terminology

- **PEMFC**: Proton Exchange Membrane Fuel Cell
- **SOFC**: Solid Oxide Fuel Cell
- **LHV**: Lower Heating Value (120 MJ/kg for H2)
- **HHV**: Higher Heating Value (142 MJ/kg for H2)
- **H70**: 700 bar hydrogen storage standard
- **H35**: 350 bar hydrogen storage standard
- **SOC**: State of Charge
- **BOL**: Beginning of Life
- **EOL**: End of Life

---

## 2. Fuel Cell Technology

### 2.1 Proton Exchange Membrane Fuel Cell (PEMFC)

#### 2.1.1 Operating Principle

The PEMFC operates through electrochemical reactions:

**Anode Reaction:**
```
H2 → 2H⁺ + 2e⁻
```

**Cathode Reaction:**
```
½O2 + 2H⁺ + 2e⁻ → H2O
```

**Overall Reaction:**
```
H2 + ½O2 → H2O + Energy (Electrical + Heat)
```

#### 2.1.2 Cell Voltage

The theoretical open-circuit voltage:

```
E_cell = E⁰ + (RT / 2F) × ln[(P_H2 × √P_O2) / P_H2O]
```

Where:
- `E⁰` = Standard cell potential (1.23 V at 25°C)
- `R` = Universal gas constant (8.314 J/mol·K)
- `T` = Temperature (K)
- `F` = Faraday constant (96,485 C/mol)
- `P_H2`, `P_O2`, `P_H2O` = Partial pressures

Typical operating voltage: 0.6 - 0.7 V per cell

#### 2.1.3 Stack Power Output

```
P_stack = N_cells × V_cell × I_stack
```

Where:
- `P_stack` = Stack power output (W)
- `N_cells` = Number of cells in series
- `V_cell` = Average cell voltage (V)
- `I_stack` = Stack current (A)

#### 2.1.4 PEMFC Efficiency

```
η_PEMFC = (V_cell / E_tn) × 100%
```

Where:
- `η_PEMFC` = Fuel cell efficiency (%)
- `V_cell` = Actual cell voltage (V)
- `E_tn` = Thermoneutral voltage (1.48 V at 25°C)

Typical PEMFC efficiency: 50-60% (electrical)

#### 2.1.5 Operating Conditions

| Parameter | Minimum | Optimal | Maximum |
|-----------|---------|---------|---------|
| Temperature | 60°C | 80°C | 90°C |
| Pressure | 1.5 bar | 2.5 bar | 3.0 bar |
| Humidity | 40% RH | 80% RH | 100% RH |
| Current Density | 0.2 A/cm² | 0.8 A/cm² | 1.5 A/cm² |

### 2.2 Solid Oxide Fuel Cell (SOFC)

#### 2.2.1 Operating Principle

SOFC operates at high temperatures (600-1000°C) with oxygen ion conduction.

**Anode Reaction:**
```
H2 + O²⁻ → H2O + 2e⁻
```

**Cathode Reaction:**
```
½O2 + 2e⁻ → O²⁻
```

#### 2.2.2 SOFC Efficiency

SOFC achieves higher electrical efficiency: 60-70%

Combined heat and power (CHP): up to 90% total efficiency

### 2.3 Fuel Cell Stack Design

#### 2.3.1 Stack Configuration

- **Bipolar Plates**: Graphite or metallic plates for current collection
- **Membrane Electrode Assembly (MEA)**: Catalyst-coated membrane
- **Gas Diffusion Layer (GDL)**: Porous carbon paper/cloth
- **Cooling Channels**: Integrated thermal management

#### 2.3.2 Stack Sizing

```
A_cell = P_target / (N_cells × V_cell × J_avg)
```

Where:
- `A_cell` = Active cell area (cm²)
- `P_target` = Target power output (W)
- `J_avg` = Average current density (A/cm²)

#### 2.3.3 Power Density

Typical power density: 2-4 kW/L (stack level)

Target: >4 kW/L for automotive applications

---

## 3. Hydrogen Storage Systems

### 3.1 Storage Tank Types

#### 3.1.1 Type III Tanks

- **Construction**: Aluminum liner with carbon fiber composite overwrap
- **Operating Pressure**: 350 bar (H35)
- **Weight**: Higher than Type IV
- **Cost**: Lower than Type IV
- **Applications**: Commercial vehicles, buses

#### 3.1.2 Type IV Tanks

- **Construction**: Polymer liner (HDPE) with carbon fiber composite overwrap
- **Operating Pressure**: 700 bar (H70)
- **Weight**: Lighter than Type III
- **Cost**: Higher than Type III
- **Applications**: Passenger vehicles

### 3.2 Storage Capacity

#### 3.2.1 Hydrogen Mass Calculation

```
m_H2 = (P × V × MW_H2) / (Z × R × T)
```

Where:
- `m_H2` = Hydrogen mass (kg)
- `P` = Pressure (Pa)
- `V` = Tank volume (m³)
- `MW_H2` = Molecular weight (2.016 g/mol)
- `Z` = Compressibility factor
- `R` = Gas constant (8.314 J/mol·K)
- `T` = Temperature (K)

#### 3.2.2 Gravimetric Density

```
ρ_grav = m_H2 / (m_H2 + m_tank) × 100%
```

Where:
- `ρ_grav` = Gravimetric density (%)
- `m_tank` = Tank system weight (kg)

Target: >5.5% (DOE 2025 target)

#### 3.2.3 Volumetric Density

```
ρ_vol = m_H2 / V_tank
```

Where:
- `ρ_vol` = Volumetric density (kg/m³)
- `V_tank` = Total tank volume (m³)

Target: >40 kg/m³ (DOE 2025 target)

### 3.3 Tank Pressure Ratings

| Standard | Nominal Pressure | Working Pressure | Test Pressure |
|----------|------------------|------------------|---------------|
| H35 | 350 bar | 350 bar | 525 bar (1.5×) |
| H70 | 700 bar | 700 bar | 1,050 bar (1.5×) |

### 3.4 Storage Energy Density

Energy stored in tank:

```
E_stored = m_H2 × LHV_H2
```

Where:
- `E_stored` = Total energy (MJ)
- `LHV_H2` = Lower heating value (120 MJ/kg)

For 5.6 kg H2:
```
E_stored = 5.6 × 120 = 672 MJ ≈ 187 kWh
```

---

## 4. Power Electronics

### 4.1 DC/DC Converter

#### 4.1.1 Boost Converter

Converts fuel cell output voltage to motor bus voltage:

```
V_out = V_in × D / (1 - D)
```

Where:
- `V_out` = Output voltage (V)
- `V_in` = Input voltage from fuel cell (V)
- `D` = Duty cycle (0-1)

#### 4.1.2 Converter Efficiency

```
η_conv = P_out / P_in × 100%
```

Typical efficiency: 95-98%

### 4.2 Motor Controller (Inverter)

#### 4.2.1 Three-Phase Inverter

Converts DC to three-phase AC for motor drive:

**Power Calculation:**
```
P_motor = √3 × V_line × I_line × cos(φ) × η_motor
```

Where:
- `P_motor` = Motor power (kW)
- `V_line` = Line voltage (V)
- `I_line` = Line current (A)
- `cos(φ)` = Power factor
- `η_motor` = Motor efficiency

#### 4.2.2 Inverter Efficiency

Typical efficiency: 94-96%

### 4.3 Battery Buffer System

Small lithium-ion battery (1-2 kWh) for:
- Peak power assist
- Regenerative braking energy capture
- Cold start support

---

## 5. Electric Motor Systems

### 5.1 Motor Types

#### 5.1.1 Permanent Magnet Synchronous Motor (PMSM)

- **Power Range**: 80-150 kW
- **Efficiency**: 92-96%
- **Power Density**: 5-8 kW/kg
- **Advantages**: High efficiency, compact size
- **Disadvantages**: Rare earth magnet cost

#### 5.1.2 Induction Motor (IM)

- **Power Range**: 100-200 kW
- **Efficiency**: 88-94%
- **Advantages**: Lower cost, robust
- **Disadvantages**: Lower efficiency

### 5.2 Motor Power Calculation

```
P_motor = T × ω / 1000
```

Where:
- `P_motor` = Motor power (kW)
- `T` = Torque (N·m)
- `ω` = Angular velocity (rad/s)

### 5.3 Motor Efficiency Map

Efficiency varies with torque and speed:

```
η_motor = f(T, ω)
```

Peak efficiency zone: 50-80% of rated torque, 2000-4000 rpm

---

## 6. Refueling Infrastructure

### 6.1 Refueling Protocols

#### 6.1.1 SAE J2601 Standard

**Pre-cooling Requirements:**
- H70: -40°C to -33°C
- H35: -20°C to -10°C

**Fill Rate:**
```
ṁ_H2 = (m_target - m_current) / t_fill
```

Where:
- `ṁ_H2` = Mass flow rate (kg/min)
- `m_target` = Target mass (kg)
- `m_current` = Current mass (kg)
- `t_fill` = Fill time (min)

Target fill time: 3-5 minutes for passenger vehicles

#### 6.1.2 Communication Protocol

**Vehicle-to-Dispenser Communication:**
- Tank volume
- Current pressure
- Current temperature
- Maximum fill rate
- Maximum pressure

#### 6.1.3 Temperature Rise During Refueling

```
ΔT = (T_final - T_initial)
```

Limit: ΔT ≤ 85°C (to prevent tank damage)

### 6.2 Dispenser Requirements

| Parameter | H35 | H70 |
|-----------|-----|-----|
| Supply Pressure | 450-500 bar | 900-950 bar |
| Pre-cooling | -20 to -10°C | -40 to -33°C |
| Flow Rate | 60 g/s | 60 g/s |
| Nozzle Type | TIR-22 | TIR-21 |

### 6.3 Station Capacity

```
Cap_station = N_dispensers × (Cap_dispenser / t_fill) × η_util
```

Where:
- `Cap_station` = Station capacity (kg/day)
- `N_dispensers` = Number of dispensers
- `Cap_dispenser` = Dispenser capacity (kg)
- `η_util` = Utilization factor (0.7-0.8)

---

## 7. Efficiency Calculations

### 7.1 Fuel Cell System Efficiency

```
η_system = η_fc × η_conv × η_aux
```

Where:
- `η_system` = Overall system efficiency
- `η_fc` = Fuel cell stack efficiency (0.50-0.60)
- `η_conv` = DC/DC converter efficiency (0.95-0.98)
- `η_aux` = Auxiliary systems efficiency (0.90-0.95)

Typical system efficiency: 45-55%

### 7.2 Powertrain Efficiency

```
η_powertrain = η_system × η_inverter × η_motor × η_transmission
```

Where:
- `η_inverter` = Inverter efficiency (0.94-0.96)
- `η_motor` = Motor efficiency (0.92-0.96)
- `η_transmission` = Transmission efficiency (0.95-0.98)

Typical powertrain efficiency: 38-48%

### 7.3 Vehicle Range Calculation

```
Range = (m_H2 × LHV_H2 × η_powertrain) / E_consumption
```

Where:
- `Range` = Vehicle range (km)
- `m_H2` = Hydrogen capacity (kg)
- `LHV_H2` = 120 MJ/kg
- `η_powertrain` = Powertrain efficiency (0.38-0.48)
- `E_consumption` = Energy consumption (MJ/km)

Example for 5.6 kg H2 capacity:
```
Range = (5.6 × 120 × 0.45) / 0.95
Range ≈ 318 km
```

### 7.4 Well-to-Wheel Efficiency

```
η_WtW = η_production × η_compression × η_transport × η_powertrain
```

Where:
- `η_production` = H2 production efficiency
  - Electrolysis: 60-80%
  - Steam methane reforming: 70-85%
- `η_compression` = Compression to 700 bar (0.88-0.92)
- `η_transport` = Distribution efficiency (0.95-0.98)
- `η_powertrain` = Vehicle efficiency (0.38-0.48)

**Green H2 (Electrolysis):**
```
η_WtW = 0.70 × 0.90 × 0.97 × 0.45 ≈ 27.5%
```

**Gray H2 (SMR with CCS):**
```
η_WtW = 0.75 × 0.90 × 0.97 × 0.45 ≈ 29.5%
```

### 7.5 Energy Consumption

#### 7.5.1 NEDC Cycle

Average energy consumption: 0.85-1.05 MJ/km (0.24-0.29 kWh/km)

#### 7.5.2 WLTC Cycle

Average energy consumption: 0.95-1.15 MJ/km (0.26-0.32 kWh/km)

#### 7.5.3 Highway Driving

Average energy consumption: 1.10-1.40 MJ/km (0.31-0.39 kWh/km)

---

## 8. Data Formats

### 8.1 Vehicle Status Data

```json
{
  "vehicle_id": "WIA-AUTO-007-12345",
  "timestamp": "2025-12-26T10:30:00Z",
  "fuel_cell": {
    "power_output": 85.5,
    "voltage": 385.2,
    "current": 222.0,
    "efficiency": 0.58,
    "temperature": 78.5,
    "status": "active"
  },
  "hydrogen_storage": {
    "pressure": 580.0,
    "temperature": 18.5,
    "mass_remaining": 3.2,
    "soc": 0.57,
    "tank_type": "Type IV H70"
  },
  "power_electronics": {
    "dc_bus_voltage": 650.0,
    "converter_efficiency": 0.97,
    "inverter_efficiency": 0.95
  },
  "motor": {
    "power_output": 78.0,
    "torque": 245.0,
    "rpm": 3040,
    "efficiency": 0.94
  },
  "battery_buffer": {
    "soc": 0.75,
    "voltage": 380.0,
    "power": 5.5,
    "mode": "assist"
  },
  "vehicle": {
    "speed": 85.0,
    "range_remaining": 185.0,
    "odometer": 12540.5,
    "energy_consumption": 0.98
  }
}
```

### 8.2 Refueling Session Data

```json
{
  "session_id": "REF-2025-12-26-001",
  "timestamp_start": "2025-12-26T10:15:00Z",
  "timestamp_end": "2025-12-26T10:18:30Z",
  "vehicle_id": "WIA-AUTO-007-12345",
  "dispenser_id": "H70-STN-001-DSP-1",
  "protocol": "SAE J2601",
  "initial_state": {
    "pressure": 150.0,
    "temperature": 22.0,
    "mass": 1.2
  },
  "final_state": {
    "pressure": 695.0,
    "temperature": 45.0,
    "mass": 5.6
  },
  "refueling_params": {
    "pre_cool_temp": -38.0,
    "peak_flow_rate": 0.068,
    "average_flow_rate": 0.062,
    "total_mass": 4.4,
    "duration": 210.0,
    "pressure_ramp_rate": 2.6
  },
  "station_info": {
    "location": "Seoul Gangnam Station",
    "supply_pressure": 925.0,
    "ambient_temp": 15.0
  }
}
```

### 8.3 Fuel Cell Performance Map

```json
{
  "map_id": "PEMFC-100kW-v1.0",
  "rated_power": 100,
  "operating_points": [
    {
      "current_density": 0.2,
      "voltage": 0.85,
      "power_density": 0.17,
      "efficiency": 0.62
    },
    {
      "current_density": 0.6,
      "voltage": 0.72,
      "power_density": 0.43,
      "efficiency": 0.58
    },
    {
      "current_density": 1.0,
      "voltage": 0.65,
      "power_density": 0.65,
      "efficiency": 0.52
    },
    {
      "current_density": 1.4,
      "voltage": 0.58,
      "power_density": 0.81,
      "efficiency": 0.45
    }
  ]
}
```

---

## 9. API Interface

### 9.1 Core Functions

#### 9.1.1 Calculate Fuel Cell Efficiency

```typescript
interface FuelCellParams {
  powerOutput: number;        // kW
  hydrogenFlowRate: number;   // kg/h
  stackVoltage: number;       // V
  stackCurrent: number;       // A
}

interface EfficiencyResult {
  efficiency: number;         // 0-1
  powerDensity: number;       // kW/L
  currentDensity: number;     // A/cm²
  voltageEfficiency: number;  // 0-1
}
```

#### 9.1.2 Calculate Vehicle Range

```typescript
interface RangeParams {
  hydrogenCapacity: number;   // kg
  fuelCellEfficiency: number; // 0-1
  systemEfficiency: number;   // 0-1
  energyConsumption: number;  // MJ/km
}

interface RangeResult {
  range: number;              // km
  energyAvailable: number;    // MJ
  energyUsable: number;       // MJ
  rangeBuffer: number;        // km (reserve)
}
```

#### 9.1.3 Validate Tank Pressure

```typescript
interface TankValidation {
  pressure: number;           // bar
  temperature: number;        // °C
  tankType: 'Type III' | 'Type IV';
  standard: 'H35' | 'H70';
}

interface TankValidationResult {
  isValid: boolean;
  warnings: string[];
  errors: string[];
  safetyMargin: number;       // %
  maxAllowedPressure: number; // bar
}
```

#### 9.1.4 Optimize Refueling

```typescript
interface RefuelingParams {
  targetPressure: number;     // bar
  ambientTemp: number;        // °C
  currentPressure: number;    // bar
  currentTemp: number;        // °C
  tankVolume: number;         // L
}

interface RefuelingPlan {
  preCoolTemp: number;        // °C
  flowRate: number;           // kg/min
  estimatedTime: number;      // min
  finalTemp: number;          // °C
  safetyChecks: SafetyCheck[];
}
```

---

## 10. Safety Protocols

### 10.1 Hydrogen Leak Detection

#### 10.1.1 Detection Threshold

Lower Explosive Limit (LEL) for H2: 4% by volume in air

Detection threshold: 1% LEL (0.04% H2)

#### 10.1.2 Sensor Requirements

- **Type**: Catalytic bead or electrochemical
- **Response Time**: <5 seconds
- **Location**: Near tank, fuel cell, refueling port
- **Redundancy**: Minimum 2 sensors per critical area

### 10.2 Pressure Relief Systems

#### 10.2.1 Thermally Activated Pressure Relief Device (TPRD)

**Activation Temperature**: 110°C ± 10°C

**Flow Capacity**: Discharge all H2 in <60 seconds

**Location**: On each storage tank

#### 10.2.2 Pressure Relief Valve (PRV)

**Set Pressure**: 125% of nominal working pressure
- H70: 875 bar
- H35: 437.5 bar

### 10.3 Crash Safety

#### 10.3.1 Tank Protection

- Tanks located in protected zone
- Front and rear crumple zones
- Side impact protection bars
- Underbody protection plate

#### 10.3.2 Emergency Shut-off

Automatic H2 shut-off in case of:
- Collision detection (>10g deceleration)
- Rollover detection (>60° tilt)
- Fire detection
- Manual trigger

**Shut-off Time**: <100 milliseconds

### 10.4 Ventilation Requirements

#### 10.4.1 Enclosed Spaces

Minimum air exchange: 4 ACH (air changes per hour)

Hydrogen rises, so ventilation at ceiling level

#### 10.4.2 Garage/Parking

Natural or mechanical ventilation required

No ignition sources within 3 meters of vehicle

### 10.5 Fire Safety

#### 10.5.1 Fire Classification

Hydrogen fires are Class C (flammable gas)

Do not extinguish unless supply can be shut off

#### 10.5.2 Safety Distances

In case of TPRD activation:
- 10 meters minimum safe distance
- 25 meters for fire/rescue personnel approach

### 10.6 Maintenance Safety

#### 10.6.1 Pre-maintenance Checks

- Verify zero pressure in H2 system
- Purge system with nitrogen
- Ground vehicle to prevent static
- Use non-sparking tools

#### 10.6.2 Personal Protective Equipment

- Safety glasses
- Fire-resistant clothing
- Grounded anti-static footwear
- H2 gas detector (personal)

---

## 11. References

### 11.1 Technical Standards

1. SAE J2601: "Fueling Protocols for Light Duty Gaseous Hydrogen Surface Vehicles"
2. ISO 19881: "Gaseous hydrogen — Land vehicle fuel containers"
3. ISO 19882: "Gaseous hydrogen — Thermally activated pressure relief devices"
4. SAE J2719: "Hydrogen Fuel Quality for Fuel Cell Vehicles"
5. GTR No. 13: "Global Technical Regulation on Hydrogen Fuel Cell Vehicles"

### 11.2 Fuel Cell Technology

1. Larminie, J., Dicks, A. (2003). "Fuel Cell Systems Explained"
2. 선행 연구. "Fuel Cell Fundamentals"
3. Zhang, J. (2008). "PEM Fuel Cell Electrocatalysts and Catalyst Layers"

### 11.3 Hydrogen Storage

1. 선행 연구. "Hydrogen as a Future Energy Carrier"
2. DOE (2020). "Hydrogen Storage Technical Team Roadmap"
3. 선행 연구. "Metal hydride materials for solid hydrogen storage"

### 11.4 Vehicle Integration

1. Toyota (2015). "Mirai Technical Overview"
2. Hyundai (2020). "NEXO Fuel Cell Vehicle Technology"
3. Honda (2016). "Clarity Fuel Cell Technical Guide"

### 11.5 Safety Standards

1. ISO/TR 15916: "Basic considerations for the safety of hydrogen systems"
2. NFPA 2: "Hydrogen Technologies Code"
3. SAE J2578: "Recommended Practice for General Fuel Cell Vehicle Safety"

### 11.6 Physical Constants

| Constant | Symbol | Value |
|----------|--------|-------|
| Lower Heating Value (H2) | LHV_H2 | 120 MJ/kg |
| Higher Heating Value (H2) | HHV_H2 | 142 MJ/kg |
| Hydrogen Density (STP) | ρ_H2 | 0.0899 kg/m³ |
| H2 Specific Heat | c_p | 14.3 kJ/kg·K |
| Faraday Constant | F | 96,485 C/mol |
| Universal Gas Constant | R | 8.314 J/mol·K |

### 11.7 WIA Standards

- WIA-INTENT: Intent-based vehicle control
- WIA-OMNI-API: Universal automotive API
- WIA-ENERGY: Smart energy systems
- WIA-SOCIAL: Vehicle communication protocols

---

## Appendix A: Example Calculations

### A.1 Fuel Cell Efficiency

```
Given:
- Stack Power: 100 kW
- Stack Voltage: 400 V
- Stack Current: 250 A
- H2 Flow Rate: 0.9 kg/h

Calculation:
- Power Output: P = V × I = 400 × 250 = 100,000 W = 100 kW
- H2 Energy Input: E_in = 0.9 kg/h × 120 MJ/kg = 108 MJ/h = 30 kW
- Efficiency: η = 100 / 30 = 0.60 or 60%

Result: Fuel cell operating at 60% efficiency (typical for PEMFC)
```

### A.2 Vehicle Range

```
Given:
- H2 Capacity: 5.6 kg
- FC Efficiency: 60%
- System Efficiency: 90%
- Motor Efficiency: 94%
- Energy Consumption: 0.95 MJ/km

Calculation:
- Total Energy: E = 5.6 × 120 = 672 MJ
- Usable Energy: E_usable = 672 × 0.60 × 0.90 × 0.94 = 341 MJ
- Range: R = 341 / 0.95 = 359 km

Result: Vehicle range approximately 359 km (similar to Toyota Mirai)
```

### A.3 Refueling Time

```
Given:
- Target Mass: 5.6 kg
- Current Mass: 0.5 kg
- Flow Rate: 0.060 kg/s

Calculation:
- Mass to Fill: Δm = 5.6 - 0.5 = 5.1 kg
- Fill Time: t = 5.1 / 0.060 = 85 seconds ≈ 1.4 minutes

Plus pre-cooling and pressurization ramp: Total ≈ 3-4 minutes
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-007 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
