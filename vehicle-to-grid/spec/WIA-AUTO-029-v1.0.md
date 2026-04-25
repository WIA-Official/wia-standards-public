# WIA-AUTO-029: Vehicle-to-Grid (V2G) Specification v1.0

> **Standard ID:** WIA-AUTO-029
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive & Energy Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Bidirectional Charging Technology](#2-bidirectional-charging-technology)
3. [Grid Services](#3-grid-services)
4. [ISO 15118 V2G Communication](#4-iso-15118-v2g-communication)
5. [Energy Management Algorithms](#5-energy-management-algorithms)
6. [Revenue Models](#6-revenue-models)
7. [Battery Degradation Considerations](#7-battery-degradation-considerations)
8. [Smart Grid Integration](#8-smart-grid-integration)
9. [Data Formats](#9-data-formats)
10. [API Interface](#10-api-interface)
11. [Safety and Standards](#11-safety-and-standards)
12. [References](#12-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the technical framework for Vehicle-to-Grid (V2G) systems, enabling electric vehicles to serve as distributed energy resources that can both consume and supply electricity to the power grid.

### 1.2 Scope

The standard covers:
- Bidirectional power electronics and charging infrastructure
- Communication protocols for grid coordination
- Energy management and optimization algorithms
- Battery health preservation strategies
- Revenue generation and compensation mechanisms
- Safety protocols and grid compliance

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - V2G technology transforms transportation into a key enabler of renewable energy integration, grid stability, and energy democracy. By treating EVs as mobile energy storage, we create a more resilient, sustainable, and equitable energy system that benefits vehicle owners, grid operators, and society.

### 1.4 Terminology

- **V2G (Vehicle-to-Grid)**: Bidirectional energy flow between EV and power grid
- **G2V (Grid-to-Vehicle)**: Conventional charging (grid to vehicle)
- **V2H (Vehicle-to-Home)**: Energy flow from vehicle to home
- **V2B (Vehicle-to-Building)**: Energy flow from vehicle to building
- **SoC (State of Charge)**: Current battery energy level (0-100%)
- **SoH (State of Health)**: Battery capacity vs. new (0-100%)
- **DoD (Depth of Discharge)**: Percentage of capacity discharged
- **EVSE (Electric Vehicle Supply Equipment)**: Charging station
- **PCS (Power Conversion System)**: Bidirectional inverter/charger
- **ISO 15118**: International V2G communication standard
- **Ancillary Services**: Grid support services (frequency regulation, etc.)

---

## 2. Bidirectional Charging Technology

### 2.1 Power Electronics Architecture

#### 2.1.1 Onboard Charger (OBC)

Modern V2G-capable vehicles require bidirectional onboard chargers:

```
Grid AC ↔ AC/DC Converter ↔ Battery DC
```

Key specifications:
- **AC Input**: 208-240V, 50/60 Hz, single or three-phase
- **DC Output**: 200-800V (vehicle dependent)
- **Power Rating**: 3.3 - 22 kW (bidirectional)
- **Efficiency**: > 92% (both directions)
- **Power Factor**: > 0.95
- **THD (Total Harmonic Distortion)**: < 5%

#### 2.1.2 Bidirectional Inverter

The core component enabling V2G:

```
Topology: Full-bridge or interleaved boost converter
Switching frequency: 20-100 kHz
Control method: Dual-loop (voltage outer, current inner)
Modulation: Space Vector PWM (SVPWM) or Sine PWM
```

Transfer function for current control:
```
G_i(s) = K_p + K_i/s
```

Where:
- `K_p` = Proportional gain
- `K_i` = Integral gain
- `s` = Laplace variable

#### 2.1.3 Power Flow Control

**Charging Mode (G2V):**
```
P_charge = V_grid × I_grid × PF × η_charge
```

**Discharging Mode (V2G):**
```
P_discharge = V_battery × I_battery × η_discharge / PF
```

Where:
- `V_grid` = Grid voltage (V)
- `I_grid` = Grid current (A)
- `PF` = Power factor (0.95-1.0)
- `η_charge` = Charging efficiency (0.90-0.94)
- `η_discharge` = Discharging efficiency (0.88-0.92)
- `V_battery` = Battery voltage (V)
- `I_battery` = Battery current (A)

### 2.2 State of Charge Management

#### 2.2.1 SoC Estimation

Coulomb counting with voltage correction:

```
SoC(t) = SoC(t-1) + (η × ∫I(t)dt) / C_nominal + ΔSoC_voltage
```

Where:
- `I(t)` = Battery current (A, positive = charging)
- `C_nominal` = Nominal capacity (Ah)
- `η` = Coulombic efficiency (0.98-0.99)
- `ΔSoC_voltage` = Voltage-based correction

#### 2.2.2 SoC Operating Windows

Recommended operating ranges:

| Application | Min SoC | Max SoC | Usable Range |
|-------------|---------|---------|--------------|
| Daily Commute | 20% | 80% | 60% |
| V2G + Reserve | 30% | 85% | 55% |
| Maximum Revenue | 20% | 90% | 70% |
| Battery Longevity | 30% | 70% | 40% |

### 2.3 Power Ramp Rates

To protect battery and grid stability:

**Maximum ramp rate:**
```
dP/dt ≤ P_max / t_ramp
```

Typical values:
- **Frequency regulation**: 20-100% per second
- **Load following**: 10-50% per minute
- **Energy arbitrage**: 5-20% per minute

---

## 3. Grid Services

### 3.1 Frequency Regulation

#### 3.1.1 Frequency Response

Primary frequency response maintains grid frequency at 60 Hz (or 50 Hz):

```
ΔP = K_f × Δf
```

Where:
- `ΔP` = Power adjustment (kW)
- `K_f` = Frequency droop coefficient (kW/Hz)
- `Δf` = Frequency deviation (Hz)

For a 10 kW V2G system with 5% droop:
```
K_f = 10 kW / (0.05 × 60 Hz) = 3.33 kW/Hz
```

#### 3.1.2 Response Time Requirements

| Regulation Type | Response Time | Compensation |
|----------------|---------------|--------------|
| Primary (FCR) | < 2 seconds | $40-60/MW/h |
| Secondary (aFRR) | < 30 seconds | $25-40/MW/h |
| Tertiary (mFRR) | < 15 minutes | $15-25/MW/h |

#### 3.1.3 Control Algorithm

```python
def frequency_regulation(current_freq, target_freq=60.0, soc_current, soc_limits):
    """
    Frequency regulation control algorithm
    """
    # Calculate frequency error
    freq_error = current_freq - target_freq

    # Determine power response with SoC limits
    if freq_error > 0.05:  # Frequency too high
        # Increase charging (absorb excess energy)
        if soc_current < soc_limits['max']:
            power_adjustment = min(freq_error * K_f, P_max_charge)
        else:
            power_adjustment = 0
    elif freq_error < -0.05:  # Frequency too low
        # Increase discharging (inject energy)
        if soc_current > soc_limits['min']:
            power_adjustment = max(freq_error * K_f, -P_max_discharge)
        else:
            power_adjustment = 0
    else:
        power_adjustment = 0

    return power_adjustment
```

### 3.2 Peak Shaving

#### 3.2.1 Peak Demand Reduction

Reduce facility peak demand to lower demand charges:

```
Demand_Charge = Peak_Power × Rate × Days_per_month
```

Savings calculation:
```
Savings = (Peak_baseline - Peak_with_V2G) × Rate × Days
```

Example:
- Baseline peak: 200 kW
- With V2G: 180 kW (10 EVs × 2 kW each)
- Demand charge: $15/kW/month
- Savings: (200 - 180) × $15 = $300/month

#### 3.2.2 Peak Prediction

Machine learning model for peak prediction:

```
Peak(t+1) = f(Load(t), Load(t-1), Temperature(t), Day_of_week, Hour)
```

Using neural network or gradient boosting:
- **Input features**: Historical load, weather, time
- **Output**: Predicted peak in next hour
- **Trigger**: Discharge when P_predicted > P_threshold

### 3.3 Load Balancing

#### 3.3.1 Load Following

Track slow variations in grid demand:

```
P_vehicle(t) = α × (P_demand(t) - P_supply(t))
```

Where:
- `α` = Participation factor (0-1)
- `P_demand(t)` = Total grid demand
- `P_supply(t)` = Total grid supply

#### 3.3.2 Aggregated Control

For fleet of N vehicles:

```
P_total = Σ(i=1 to N) P_vehicle_i(t)
```

With constraints:
```
P_min_i ≤ P_vehicle_i ≤ P_max_i
SoC_min_i ≤ SoC_i(t) ≤ SoC_max_i
```

### 3.4 Voltage Support

#### 3.4.1 Reactive Power Control

V2G inverters can provide reactive power (Q) without battery:

```
S = √(P² + Q²) ≤ S_max
Q_max = √(S_max² - P²)
```

Where:
- `S` = Apparent power (kVA)
- `P` = Active power (kW)
- `Q` = Reactive power (kVAR)
- `S_max` = Inverter rating (kVA)

#### 3.4.2 Voltage Regulation

```
Q_injection = K_v × (V_target - V_measured)
```

Where:
- `K_v` = Voltage droop coefficient (kVAR/V)
- `V_target` = Target voltage (typically 240V)
- `V_measured` = Measured grid voltage

---

## 4. ISO 15118 V2G Communication

### 4.1 Protocol Overview

ISO 15118 enables high-level communication between EV and EVSE:

**Protocol Stack:**
```
Application Layer: V2G Messages (EXI encoded)
Presentation Layer: EXI (Efficient XML Interchange)
Session Layer: V2GTP (V2G Transfer Protocol)
Transport Layer: TCP
Network Layer: IPv6
Data Link Layer: Ethernet (PLC or Wi-Fi)
Physical Layer: HomePlug Green PHY or Wi-Fi
```

### 4.2 Message Sequences

#### 4.2.1 Session Establishment

```
EV                              EVSE
│                                │
├──── SessionSetupReq ──────────>│
│<─── SessionSetupRes ───────────┤
│                                │
├──── ServiceDiscoveryReq ──────>│
│<─── ServiceDiscoveryRes ───────┤
│                                │
├──── ServiceDetailReq ─────────>│
│<─── ServiceDetailRes ──────────┤
│                                │
├──── PaymentServiceSelectionReq>│
│<─── PaymentServiceSelectionRes─┤
```

#### 4.2.2 Charging Loop

```
EV                              EVSE
│                                │
├──── ChargeParameterDiscoveryReq>│
│<─── ChargeParameterDiscoveryRes│
│                                │
├──── PowerDeliveryReq (Start) ─>│
│<─── PowerDeliveryRes ──────────┤
│                                │
├──── CurrentDemandReq ─────────>│  (Repeated every 250ms)
│<─── CurrentDemandRes ──────────┤
│                                │
├──── PowerDeliveryReq (Stop) ──>│
│<─── PowerDeliveryRes ──────────┤
```

### 4.3 V2G-Specific Parameters

#### 4.3.1 Discharge Parameters

```xml
<DC_EVSEChargeParameter>
  <EVSEMaximumPowerLimit>10000</EVSEMaximumPowerLimit>  <!-- 10 kW -->
  <EVSEMaximumCurrentLimit>32</EVSEMaximumCurrentLimit>  <!-- 32 A -->
  <EVSEMaximumVoltageLimit>450</EVSEMaximumVoltageLimit> <!-- 450 V -->
  <EVSEMinimumCurrentLimit>-32</EVSEMinimumCurrentLimit> <!-- -32 A (discharge) -->
  <EVSEMinimumVoltageLimit>200</EVSEMinimumVoltageLimit> <!-- 200 V -->
  <EVSEPowerRampLimitation>100</EVSEPowerRampLimitation> <!-- 100 W/s -->
</DC_EVSEChargeParameter>
```

#### 4.3.2 Energy Transfer Mode

```xml
<ChargeService>
  <ServiceID>1</ServiceID>
  <ServiceName>AC_BPT</ServiceName>  <!-- Bidirectional Power Transfer -->
  <ServiceCategory>EVCharging</ServiceCategory>
  <FreeService>false</FreeService>
  <SupportedEnergyTransferMode>
    <EnergyTransferMode>AC_three_phase_core_BPT</EnergyTransferMode>
  </SupportedEnergyTransferMode>
</ChargeService>
```

### 4.4 Grid Services Signaling

```xml
<EVPowerProfile>
  <TimeAnchor>2025-12-26T14:30:00Z</TimeAnchor>
  <EVPowerProfileEntry>
    <StartPeriod>0</StartPeriod>  <!-- Immediately -->
    <Duration>900</Duration>       <!-- 15 minutes -->
    <Power>-8000</Power>           <!-- -8 kW (discharge to grid) -->
  </EVPowerProfileEntry>
  <EVPowerProfileEntry>
    <StartPeriod>900</StartPeriod>
    <Duration>1800</Duration>
    <Power>5000</Power>            <!-- 5 kW (charge from grid) -->
  </EVPowerProfileEntry>
</EVPowerProfile>
```

---

## 5. Energy Management Algorithms

### 5.1 Optimal Charging Schedule

#### 5.1.1 Optimization Problem

Minimize cost while meeting departure requirements:

```
minimize: Σ(t=0 to T) [P_charge(t) × Price(t) × Δt]

subject to:
  SoC(T_departure) ≥ SoC_target
  SoC_min ≤ SoC(t) ≤ SoC_max
  -P_max_discharge ≤ P(t) ≤ P_max_charge
  SoC(t+1) = SoC(t) + (η × P(t) × Δt) / C
```

#### 5.1.2 Dynamic Programming Solution

```python
def optimal_schedule(prices, soc_initial, soc_target, departure_time):
    """
    Dynamic programming for optimal V2G schedule
    """
    T = len(prices)
    V = np.zeros((T+1, 101))  # Value function (time, SoC%)
    policy = np.zeros((T, 101))  # Optimal power at each state

    # Initialize: high cost if target not met at departure
    for soc in range(101):
        if soc < soc_target:
            V[T][soc] = 1e6  # Penalty
        else:
            V[T][soc] = 0

    # Backward induction
    for t in range(T-1, -1, -1):
        for soc in range(101):
            min_cost = float('inf')
            best_power = 0

            # Try different power levels
            for power in np.linspace(-P_max_discharge, P_max_charge, 50):
                # Calculate next SoC
                soc_next = soc + (efficiency * power * dt) / capacity
                soc_next = int(np.clip(soc_next, SoC_min, SoC_max))

                # Cost = energy cost + future cost
                cost = power * prices[t] * dt + V[t+1][soc_next]

                if cost < min_cost:
                    min_cost = cost
                    best_power = power

            V[t][soc] = min_cost
            policy[t][soc] = best_power

    return policy
```

### 5.2 Energy Arbitrage Strategy

#### 5.2.1 Buy-Low, Sell-High Algorithm

```python
def arbitrage_decision(current_price, price_forecast, soc_current):
    """
    Decide whether to charge, discharge, or idle
    """
    # Calculate price percentiles
    p10 = np.percentile(price_forecast, 10)  # Low price threshold
    p90 = np.percentile(price_forecast, 90)  # High price threshold

    # Decision logic
    if current_price < p10 and soc_current < SoC_max:
        # Buy (charge) - prices are low
        power = P_max_charge
        action = "CHARGE"
    elif current_price > p90 and soc_current > SoC_min:
        # Sell (discharge) - prices are high
        power = -P_max_discharge
        action = "DISCHARGE"
    else:
        # Hold - prices are moderate
        power = 0
        action = "IDLE"

    return power, action
```

#### 5.2.2 Revenue Calculation

Daily arbitrage revenue:

```
Revenue_daily = Σ(P_sell × Price_high × η_discharge - P_buy × Price_low / η_charge) × Δt
```

Example:
- Buy: 10 kWh at $0.08/kWh = $0.80
- Sell: 10 × 0.88 = 8.8 kWh at $0.35/kWh = $3.08
- Net revenue: $3.08 - $0.80 = $2.28/day
- Monthly: $2.28 × 30 = $68.40

### 5.3 Multi-Objective Optimization

Balance multiple objectives:

```
minimize: w1 × Cost + w2 × Degradation + w3 × Inconvenience

subject to:
  Grid service commitments
  Departure time requirements
  Battery health constraints
```

Where:
- `w1, w2, w3` = Weighting factors (user preferences)
- `Cost` = Energy cost ($)
- `Degradation` = Battery wear cost ($)
- `Inconvenience` = Unmet driving needs penalty

---

## 6. Revenue Models

### 6.1 Compensation Structures

#### 6.1.1 Energy Market Participation

**Day-Ahead Market:**
```
Revenue_DA = Σ(E_sell(h) × LMP(h))
```

Where:
- `E_sell(h)` = Energy sold in hour h (kWh)
- `LMP(h)` = Locational Marginal Price ($/kWh)

**Real-Time Market:**
```
Revenue_RT = Σ(P_discharge(t) × Price_RT(t) × Δt)
```

#### 6.1.2 Ancillary Services

**Capacity Payment:**
```
Revenue_capacity = P_committed × Rate_capacity × Hours
```

**Performance Payment:**
```
Revenue_performance = Energy_delivered × Rate_performance
```

Total ancillary revenue:
```
Revenue_ancillary = Revenue_capacity + Revenue_performance
```

### 6.2 Revenue Sharing Models

#### 6.2.1 Owner-Aggregator Split

Common splits:
- **70/30**: 70% vehicle owner, 30% aggregator
- **60/40**: With aggregator providing equipment
- **80/20**: Owner owns charger, aggregator provides software

#### 6.2.2 Revenue Example

Monthly revenue breakdown for 10 kW V2G system:

| Revenue Stream | Amount | Owner (70%) | Aggregator (30%) |
|----------------|--------|-------------|------------------|
| Energy Arbitrage | $45 | $31.50 | $13.50 |
| Frequency Regulation | $80 | $56.00 | $24.00 |
| Demand Response | $25 | $17.50 | $7.50 |
| Peak Shaving | $35 | $24.50 | $10.50 |
| **Total** | **$185** | **$129.50** | **$55.50** |

### 6.3 Cost Considerations

#### 6.3.1 Battery Degradation Cost

```
Cost_degradation = Cycles × Capacity × (Cost_battery / Cycle_life) × DoD
```

Example:
- Battery cost: $150/kWh
- Capacity: 75 kWh
- Cycle life: 2,000 cycles (80% depth)
- V2G cycles/day: 0.5
- Average DoD: 0.4

```
Cost_per_cycle = (75 × $150) / 2000 × 0.4 = $2.25
Daily cost = 0.5 × $2.25 = $1.13
Monthly cost = $1.13 × 30 = $33.90
```

#### 6.3.2 Net Revenue

```
Net_Revenue = Gross_Revenue - Degradation_Cost - Service_Fee
```

Using example above:
```
Net_Revenue = $129.50 - $33.90 - $5.00 = $90.60/month
Annual = $90.60 × 12 = $1,087/year
```

---

## 7. Battery Degradation Considerations

### 7.1 Degradation Mechanisms

#### 7.1.1 Calendar Aging

Time-dependent capacity loss even when not in use:

```
C_loss_calendar(t) = α × √t × exp(-E_a / (R × T))
```

Where:
- `α` = Pre-exponential factor
- `t` = Time (months)
- `E_a` = Activation energy (J/mol)
- `R` = Gas constant (8.314 J/mol·K)
- `T` = Temperature (K)

#### 7.1.2 Cycle Aging

Capacity loss from charge/discharge cycles:

```
C_loss_cycle(N) = β × N^γ × DoD^δ × C_rate^ε
```

Where:
- `N` = Number of cycles
- `DoD` = Depth of discharge (0-1)
- `C_rate` = Charge/discharge rate (C)
- `β, γ, δ, ε` = Empirical coefficients

Typical values:
- `γ` = 0.5 - 0.6 (square root dependency)
- `δ` = 1.2 - 1.5 (superlinear with DoD)
- `ε` = 0.3 - 0.5 (moderate rate dependency)

#### 7.1.3 Combined Model

Total capacity loss:

```
SoH(t, N) = 1 - (C_loss_calendar + C_loss_cycle)
```

### 7.2 V2G Impact on Battery Life

#### 7.2.1 Increased Cycle Count

Without V2G:
- Cycles/day: 0.3 (occasional deep discharge)
- Annual cycles: 110
- Lifetime (to 80% SoH): 2,500 cycles / 110 = 22.7 years

With V2G (aggressive):
- Cycles/day: 1.0
- Annual cycles: 365
- Lifetime: 2,500 / 365 = 6.8 years

With V2G (optimized):
- Cycles/day: 0.5
- Annual cycles: 182
- Lifetime: 2,500 / 182 = 13.7 years

#### 7.2.2 Reduced DoD Impact

Operating in 40-80% SoC window (40% DoD) vs. 20-90% (70% DoD):

```
Lifetime_ratio = (DoD_baseline / DoD_V2G)^δ
                = (0.70 / 0.40)^1.3 ≈ 2.2×
```

**Result**: Limiting DoD can more than double cycle life.

### 7.3 Optimization Strategies

#### 7.3.1 Degradation-Aware Scheduling

Include degradation cost in optimization:

```
Cost_total(t) = Energy_cost(t) + λ × Degradation_cost(t)
```

Where `λ` is degradation cost weight (user-adjustable).

Degradation cost per cycle:
```
Deg_cost = (Battery_cost / Cycle_life) × DoD × C_rate_factor
```

#### 7.3.2 Adaptive SoC Windows

Adjust operating range based on battery age:

| Battery SoH | SoC Min | SoC Max | Usable Range |
|-------------|---------|---------|--------------|
| 100-90% | 25% | 85% | 60% |
| 90-80% | 30% | 80% | 50% |
| 80-70% | 35% | 75% | 40% |
| < 70% | 40% | 70% | 30% |

#### 7.3.3 Temperature Management

Maintain optimal battery temperature:

```python
def thermal_management(T_battery, T_ambient, P_discharge):
    """
    Adjust power to maintain safe battery temperature
    """
    T_optimal = 25  # °C
    T_max = 40      # °C

    if T_battery > T_max:
        # Reduce power to cool down
        P_derate = P_discharge * (T_max - T_battery) / (T_max - T_optimal)
        return max(0, P_derate)
    elif T_battery < 10:
        # Low temperature - reduce to prevent Li plating
        return P_discharge * 0.5
    else:
        return P_discharge
```

---

## 8. Smart Grid Integration

### 8.1 Grid Operator Interface

#### 8.1.1 System Architecture

```
Grid Operator (ISO/RTO)
    ↓
Aggregator Platform
    ↓
EV Charging Network
    ↓
Individual EVs
```

Communication flow:
1. Grid operator sends demand signal
2. Aggregator distributes to EV fleet
3. EVs respond with available capacity
4. Aggregator coordinates collective response
5. Grid operator measures performance

#### 8.1.2 OpenADR Integration

Automated Demand Response protocol:

```xml
<oadrDistributeEvent>
  <eiEvent>
    <eventDescriptor>
      <eventID>EVT-2025-001</eventID>
      <modificationNumber>0</modificationNumber>
      <eventStatus>active</eventStatus>
    </eventDescriptor>
    <eiActivePeriod>
      <startDateTime>2025-12-26T17:00:00Z</startDateTime>
      <duration>PT2H</duration>  <!-- 2 hours -->
    </eiActivePeriod>
    <eiEventSignals>
      <eiEventSignal>
        <signalName>LOAD_DISPATCH</signalName>
        <signalType>level</signalType>
        <intervals>
          <interval>
            <duration>PT15M</duration>
            <signalPayload>-8.5</signalPayload>  <!-- -8.5 kW per EV -->
          </interval>
        </intervals>
      </eiEventSignal>
    </eiEventSignals>
  </eiEvent>
</oadrDistributeEvent>
```

### 8.2 Aggregation Strategies

#### 8.2.1 Portfolio Management

Manage diverse fleet with varying characteristics:

```python
class FleetAggregator:
    def __init__(self, vehicles):
        self.vehicles = vehicles

    def get_available_capacity(self):
        """Calculate total available V2G capacity"""
        capacity_up = 0    # Charging capacity
        capacity_down = 0  # Discharging capacity

        for vehicle in self.vehicles:
            if vehicle.is_connected:
                # Discharge capacity (constrained by SoC min)
                energy_available = vehicle.soc - vehicle.soc_min
                capacity_down += min(vehicle.max_discharge_power,
                                     energy_available / dt)

                # Charge capacity (constrained by SoC max)
                energy_needed = vehicle.soc_max - vehicle.soc
                capacity_up += min(vehicle.max_charge_power,
                                   energy_needed / dt)

        return capacity_up, capacity_down
```

#### 8.2.2 Fairness Allocation

Distribute grid service requests fairly across fleet:

```python
def allocate_power_fair(total_power_request, vehicles):
    """
    Allocate power request proportionally to available capacity
    """
    allocations = {}

    # Calculate each vehicle's share of total capacity
    total_capacity = sum(v.max_power for v in vehicles if v.is_connected)

    for vehicle in vehicles:
        if vehicle.is_connected:
            # Proportional allocation
            share = vehicle.max_power / total_capacity
            allocated_power = total_power_request * share

            # Respect vehicle constraints
            allocated_power = np.clip(allocated_power,
                                      vehicle.min_power,
                                      vehicle.max_power)

            allocations[vehicle.id] = allocated_power

    return allocations
```

### 8.3 Renewable Energy Integration

#### 8.3.1 Solar + V2G

Optimal pairing for homes with solar:

**Daytime (Solar Production):**
```
PV_production → Battery_charge
Grid_export = PV - Home_load - Battery_charge
```

**Evening (Peak Demand):**
```
Battery_discharge → Home_load + Grid_export
Revenue = Grid_export × Peak_price
```

#### 8.3.2 Wind + V2G

Absorb excess wind energy at night:

```python
def wind_integration_strategy(wind_forecast, ev_fleet):
    """
    Charge EVs when wind production is high, discharge during low wind
    """
    for hour in range(24):
        wind_excess = wind_forecast[hour] - base_load[hour]

        if wind_excess > 0:
            # Excess wind - charge EVs
            charge_power = min(wind_excess, fleet.total_charge_capacity)
            fleet.distribute_charging(charge_power)
        elif wind_excess < -threshold:
            # Wind deficit - discharge EVs to compensate
            discharge_power = min(-wind_excess, fleet.total_discharge_capacity)
            fleet.distribute_discharging(discharge_power)
```

---

## 9. Data Formats

### 9.1 Vehicle State Message

```json
{
  "vehicleId": "EV-12345-ABC",
  "timestamp": "2025-12-26T14:30:00Z",
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194
  },
  "connection": {
    "connected": true,
    "evseId": "EVSE-001",
    "connectionTime": "2025-12-26T14:15:00Z",
    "plannedDeparture": "2025-12-26T18:00:00Z"
  },
  "battery": {
    "soc": 65.5,
    "soh": 94.2,
    "capacity": 75.0,
    "temperature": 28.5,
    "voltage": 385.2,
    "current": 15.3
  },
  "power": {
    "currentPower": 5.8,
    "maxChargePower": 11.0,
    "maxDischargePower": 10.0,
    "mode": "charging"
  },
  "preferences": {
    "targetSoC": 80,
    "minSoC": 30,
    "maxDoD": 60,
    "allowV2G": true,
    "degradationLimit": 0.5
  }
}
```

### 9.2 Grid Service Request

```json
{
  "requestId": "GSR-2025-12-26-001",
  "timestamp": "2025-12-26T14:30:00Z",
  "serviceType": "frequency_regulation",
  "duration": 3600,
  "startTime": "2025-12-26T15:00:00Z",
  "endTime": "2025-12-26T16:00:00Z",
  "powerProfile": {
    "baseline": 0,
    "maxCharge": 20,
    "maxDischarge": -15,
    "rampRate": 5.0
  },
  "compensation": {
    "capacityRate": 35.0,
    "energyRate": 0.25,
    "currency": "USD",
    "unit": "per_kWh"
  },
  "requirements": {
    "responseTime": 4,
    "availability": 0.95,
    "accuracyTolerance": 0.10
  }
}
```

### 9.3 Revenue Report

```json
{
  "vehicleId": "EV-12345-ABC",
  "reportPeriod": {
    "start": "2025-11-01T00:00:00Z",
    "end": "2025-11-30T23:59:59Z"
  },
  "summary": {
    "totalRevenue": 127.85,
    "totalCost": 45.60,
    "netRevenue": 82.25,
    "energyDelivered": 125.5,
    "energyConsumed": 198.3
  },
  "breakdown": {
    "energyArbitrage": {
      "revenue": 38.50,
      "transactions": 22
    },
    "frequencyRegulation": {
      "revenue": 65.20,
      "hours": 78.5
    },
    "demandResponse": {
      "revenue": 18.15,
      "events": 5
    },
    "peakShaving": {
      "revenue": 6.00,
      "savings": 12.5
    }
  },
  "costs": {
    "energyPurchase": 35.80,
    "degradation": 8.20,
    "serviceFee": 1.60
  },
  "batteryHealth": {
    "cyclesThisPeriod": 15.3,
    "averageDoD": 0.42,
    "estimatedDegradation": 0.08
  }
}
```

---

## 10. API Interface

### 10.1 RESTful API Endpoints

#### 10.1.1 Vehicle Registration

```
POST /api/v1/vehicles

Request:
{
  "vehicleId": "EV-12345",
  "make": "Tesla",
  "model": "Model 3",
  "year": 2024,
  "battery": {
    "capacity": 75,
    "chemistry": "NMC811"
  },
  "charger": {
    "maxChargePower": 11,
    "maxDischargePower": 10,
    "connector": "Type2"
  }
}

Response:
{
  "status": "success",
  "vehicleId": "EV-12345",
  "apiKey": "sk_live_abc123...",
  "registered": "2025-12-26T14:30:00Z"
}
```

#### 10.1.2 Start V2G Session

```
POST /api/v1/sessions

Request:
{
  "vehicleId": "EV-12345",
  "evseId": "EVSE-001",
  "service": "frequency_regulation",
  "duration": 7200,
  "preferences": {
    "targetSoC": 80,
    "minSoC": 30
  }
}

Response:
{
  "sessionId": "SESS-2025-12-26-001",
  "status": "active",
  "startTime": "2025-12-26T15:00:00Z",
  "estimatedRevenue": 12.50
}
```

#### 10.1.3 Get Current Status

```
GET /api/v1/vehicles/{vehicleId}/status

Response:
{
  "vehicleId": "EV-12345",
  "soc": 68.5,
  "power": -8.2,
  "mode": "discharging",
  "revenue": {
    "current_session": 5.80,
    "today": 18.50,
    "month": 127.85
  }
}
```

### 10.2 WebSocket Real-Time Updates

```javascript
// Connect to WebSocket
const ws = new WebSocket('wss://api.wia.com/v2g/stream');

// Authenticate
ws.send(JSON.stringify({
  type: 'auth',
  apiKey: 'sk_live_abc123...'
}));

// Subscribe to vehicle updates
ws.send(JSON.stringify({
  type: 'subscribe',
  channel: 'vehicle:EV-12345'
}));

// Receive real-time updates
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);

  if (data.type === 'power_update') {
    console.log(`Power: ${data.power} kW`);
    console.log(`SoC: ${data.soc}%`);
  }

  if (data.type === 'revenue_update') {
    console.log(`Revenue: $${data.revenue}`);
  }
};
```

---

## 11. Safety and Standards

### 11.1 Electrical Safety

#### 11.1.1 Protection Mechanisms

**Ground Fault Detection:**
```
I_leakage = I_grid - I_vehicle
If I_leakage > 30 mA → Disconnect
```

**Over-Current Protection:**
```
If I_grid > I_rated × 1.1 for t > 0.5s → Disconnect
```

**Over/Under Voltage:**
```
If V_grid < 0.85 × V_nominal or V_grid > 1.1 × V_nominal → Disconnect
```

**Frequency Deviation:**
```
If |f_grid - f_nominal| > 0.5 Hz → Disconnect
```

#### 11.1.2 Anti-Islanding

Prevent energizing de-energized grid:

```python
def anti_islanding_detection():
    """
    Detect grid disconnection and prevent islanding
    """
    # Active frequency drift method
    f_grid = measure_grid_frequency()

    # Inject small frequency perturbation
    inject_perturbation(0.1)  # 0.1 Hz

    # Measure response
    f_response = measure_grid_frequency()

    if abs(f_response - f_grid) > threshold:
        # Grid is weak/disconnected - stop V2G
        disconnect_immediately()
        return "ISLANDING_DETECTED"
    else:
        return "GRID_CONNECTED"
```

### 11.2 Cybersecurity

#### 11.2.1 Authentication

All V2G communications must use:
- **TLS 1.3** for transport encryption
- **X.509 certificates** for device authentication
- **OAuth 2.0** or **API keys** for user authentication

#### 11.2.2 Message Integrity

```
HMAC-SHA256(message || nonce || timestamp, shared_key)
```

Prevent:
- Replay attacks (nonce + timestamp)
- Man-in-the-middle (TLS)
- Unauthorized control (authentication)

### 11.3 Standards Compliance

#### 11.3.1 Required Standards

| Standard | Description | Scope |
|----------|-------------|-------|
| ISO 15118-20 | V2G communication (AC & DC) | Protocol |
| IEC 61851-1 | EV charging system | Hardware |
| IEC 61851-23 | DC charging stations | Hardware |
| IEEE 2030.1.1 | V2G interface | Grid integration |
| UL 1741 | Inverters and grid-tied converters | Safety |
| SAE J2847/2 | V2G communication | Protocol |
| SAE J3072 | V2G AC level 2 | Hardware |
| IEEE 1547 | Distributed energy resources | Grid connection |

#### 11.3.2 Testing Requirements

**Hardware Testing:**
- Insulation resistance: > 1 MΩ
- Ground continuity: < 0.1 Ω
- Dielectric strength: 2× rated voltage + 1000V for 1 minute
- Temperature rise: < 50°C above ambient

**Performance Testing:**
- Efficiency: > 88% (both directions)
- Power factor: > 0.95
- THD: < 5%
- Response time: Per service requirement

---

## 12. References

### 12.1 Technical Standards

1. **ISO 15118-20** (2022). "Road vehicles - Vehicle to grid communication interface - Part 20: 2nd generation network layer and application layer requirements"
2. **IEC 61851-1** (2017). "Electric vehicle conductive charging system - Part 1: General requirements"
3. **IEEE 2030.1.1** (2021). "Technical Specifications of a DC Quick Charger for Use with Electric Vehicles"
4. **SAE J2847/2** (2022). "Communication between Plug-in Vehicles and the Utility Grid"
5. **UL 1741** (2021). "Inverters, Converters, Controllers and Interconnection System Equipment"

### 12.2 Scientific Literature

1. Kempton, W., & Tomić, J. (2005). "Vehicle-to-grid power fundamentals: Calculating capacity and net revenue." *Journal of Power Sources*, 144(1), 268-279.
2. Peterson, S. B., Whitacre, J. F., & Apt, J. (2010). "The economics of using plug-in hybrid electric vehicle battery packs for grid storage." *Journal of Power Sources*, 195(8), 2377-2384.

### 12.3 Grid Operator Resources

1. **CAISO** - California ISO V2G Participation Model
2. **PJM** - Frequency Regulation Market Rules
3. **ERCOT** - Ancillary Services Protocols
4. **FERC Order 841** - Electric Storage Participation in Markets

### 12.4 Battery Research

2. Dubarry, M., & Baure, G. (2020). "Perspective on commercial Li-ion battery testing, best practices for simple and effective protocols." *Electronics*, 9(1), 152.

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-029 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
