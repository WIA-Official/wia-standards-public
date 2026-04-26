# Chapter 5: Control Protocols and Optimization (Phase 3)

## 5.1 Intelligent Building Control Overview

### 5.1.1 From Monitoring to Control

Phase 3 of WIA-BEMS transitions from data collection and access (Phases 1-2) to intelligent automated control. This phase defines the control sequences, optimization algorithms, and operational protocols that enable buildings to operate autonomously while maximizing efficiency and occupant comfort.

**Control Architecture Layers:**

```
┌───────────────────────────────────────────────────────────────────┐
│                    SUPERVISORY OPTIMIZATION                        │
│  Global Optimization │ Demand Response │ Predictive Scheduling    │
│                      (Minutes to Hours)                            │
└────────────────────────────────┬──────────────────────────────────┘
                                 │
┌────────────────────────────────┴──────────────────────────────────┐
│                    SEQUENCE COORDINATION                           │
│  Setpoint Optimization │ Mode Selection │ Equipment Staging       │
│                      (Seconds to Minutes)                          │
└────────────────────────────────┬──────────────────────────────────┘
                                 │
┌────────────────────────────────┴──────────────────────────────────┐
│                      LOCAL CONTROL LOOPS                           │
│  PID Control │ Modulating Control │ On/Off Control                │
│                     (Milliseconds to Seconds)                      │
└────────────────────────────────┬──────────────────────────────────┘
                                 │
┌────────────────────────────────┴──────────────────────────────────┐
│                     EQUIPMENT INTERFACE                            │
│  BACnet │ Modbus │ MQTT │ Native Protocols                        │
└───────────────────────────────────────────────────────────────────┘
```

### 5.1.2 Control Mode Framework

**Control Modes:**

```typescript
interface ControlModes {
  occupied: {
    description: "Normal occupied operation";
    triggers: ["schedule", "occupancy_sensor", "manual"];
    behavior: "Full comfort control, normal setpoints";
  };

  unoccupied: {
    description: "Setback mode for unoccupied periods";
    triggers: ["schedule", "occupancy_timeout", "manual"];
    behavior: "Wide temperature bands, reduced ventilation";
  };

  standby: {
    description: "Ready for quick return to occupied";
    triggers: ["pre_schedule", "occupancy_approaching"];
    behavior: "Maintain moderate conditions";
  };

  warmup: {
    description: "Morning warmup period";
    triggers: ["scheduled_start", "optimal_start"];
    behavior: "Aggressive heating to reach setpoint by occupancy";
  };

  cooldown: {
    description: "Morning cooldown period";
    triggers: ["scheduled_start", "optimal_start"];
    behavior: "Pre-cooling to reach setpoint by occupancy";
  };

  demand_response: {
    description: "Grid demand response event active";
    triggers: ["dr_signal", "price_threshold"];
    behavior: "Shed loads, widen setpoints, pre-condition";
  };

  emergency: {
    description: "Emergency or fault condition";
    triggers: ["fire_alarm", "equipment_fault", "power_failure"];
    behavior: "Safety-first operation";
  };

  nightFlush: {
    description: "Night purge ventilation";
    triggers: ["outdoor_temp_favorable", "building_thermal_mass"];
    behavior: "High ventilation to pre-cool with outdoor air";
  };
}
```

## 5.2 HVAC Control Sequences

### 5.2.1 Air Handling Unit Control

**AHU Control Sequence (ASHRAE Guideline 36 Compatible):**

```python
class AHUController:
    """
    WIA-BEMS AHU Control Sequence
    Based on ASHRAE Guideline 36-2021
    """

    def __init__(self, config: AHUConfig):
        self.config = config
        self.mode = ControlMode.STANDBY
        self.economizer = EconomizerController(config.economizer)

    def execute_control_loop(self, inputs: AHUInputs) -> AHUOutputs:
        """
        Main control loop executed every scan (typically 1-5 seconds)
        """
        outputs = AHUOutputs()

        # 1. Determine operating mode
        self.mode = self._determine_mode(inputs)

        # 2. Supply air temperature control
        outputs.supply_air_temp_setpoint = self._calculate_sat_setpoint(inputs)

        # 3. Economizer control
        econ_outputs = self.economizer.control(
            outdoor_temp=inputs.outdoor_air_temp,
            return_temp=inputs.return_air_temp,
            supply_setpoint=outputs.supply_air_temp_setpoint,
            mode=self.mode
        )
        outputs.outdoor_air_damper = econ_outputs.oa_damper
        outputs.return_air_damper = econ_outputs.ra_damper

        # 4. Cooling/Heating valve control
        cooling_heating = self._control_coils(
            supply_temp=inputs.supply_air_temp,
            setpoint=outputs.supply_air_temp_setpoint,
            mode=self.mode
        )
        outputs.cooling_valve = cooling_heating.cooling
        outputs.heating_valve = cooling_heating.heating

        # 5. Fan control
        outputs.supply_fan_speed = self._control_supply_fan(inputs)
        outputs.return_fan_speed = self._control_return_fan(inputs, outputs)

        # 6. Minimum outdoor air
        outputs.outdoor_air_damper = max(
            outputs.outdoor_air_damper,
            self._minimum_oa_position(inputs)
        )

        return outputs

    def _calculate_sat_setpoint(self, inputs: AHUInputs) -> float:
        """
        Supply Air Temperature Reset based on zone demands
        """
        if self.mode == ControlMode.COOLING:
            # Reset SAT based on cooling requests
            max_cooling_request = max(z.cooling_request for z in inputs.zones)

            # 10°C minimum, 16°C maximum
            sat = 16 - (max_cooling_request / 100) * 6
            return max(10, min(16, sat))

        elif self.mode == ControlMode.HEATING:
            # Reset SAT based on heating requests
            max_heating_request = max(z.heating_request for z in inputs.zones)

            # 20°C minimum, 35°C maximum
            sat = 20 + (max_heating_request / 100) * 15
            return max(20, min(35, sat))

        return 14  # Default for ventilating mode

    def _control_supply_fan(self, inputs: AHUInputs) -> float:
        """
        Supply fan speed control with static pressure reset
        """
        # Static pressure setpoint reset
        # If most VAV boxes are nearly closed, reduce pressure
        # If any VAV box is fully open, increase pressure

        vav_positions = [z.damper_position for z in inputs.zones]
        max_position = max(vav_positions)
        avg_position = sum(vav_positions) / len(vav_positions)

        # Trim and respond logic
        if max_position > 95:  # At least one box nearly full open
            self.static_pressure_setpoint = min(
                self.static_pressure_setpoint + 25,  # Pa
                self.config.max_static_pressure
            )
        elif avg_position < 50:  # Boxes generally closed
            self.static_pressure_setpoint = max(
                self.static_pressure_setpoint - 15,
                self.config.min_static_pressure
            )

        # PID control on static pressure
        error = self.static_pressure_setpoint - inputs.duct_static_pressure
        return self.fan_pid.calculate(error)

    def _control_coils(self, supply_temp: float, setpoint: float,
                       mode: ControlMode) -> CoilOutputs:
        """
        Cooling and heating coil control with deadband
        """
        outputs = CoilOutputs()
        error = setpoint - supply_temp

        if mode == ControlMode.COOLING:
            if error < -0.5:  # Need more cooling
                outputs.cooling = min(100, self.cooling_valve + 5)
            elif error > 0.5:  # Too much cooling
                outputs.cooling = max(0, self.cooling_valve - 5)
            outputs.heating = 0

        elif mode == ControlMode.HEATING:
            if error > 0.5:  # Need more heating
                outputs.heating = min(100, self.heating_valve + 5)
            elif error < -0.5:  # Too much heating
                outputs.heating = max(0, self.heating_valve - 5)
            outputs.cooling = 0

        return outputs
```

### 5.2.2 VAV Box Control

**Terminal Unit Control:**

```typescript
interface VAVBoxController {
  // Configuration
  config: {
    zone_id: string;
    box_type: 'cooling_only' | 'reheat' | 'dual_duct';
    design_cfm: number;
    min_cfm_occupied: number;
    min_cfm_unoccupied: number;
    reheat_type?: 'hot_water' | 'electric';
    has_occupancy_sensor: boolean;
    has_co2_sensor: boolean;
  };

  // Control sequence
  control(inputs: VAVInputs): VAVOutputs;
}

function vavControl(inputs: VAVInputs, config: VAVConfig): VAVOutputs {
  const outputs: VAVOutputs = {};

  // Determine occupancy
  const occupied = inputs.occupied ||
                   inputs.occupancy_sensor ||
                   (inputs.schedule === 'occupied');

  // Calculate ventilation requirement
  let minCFM = occupied ? config.min_cfm_occupied : config.min_cfm_unoccupied;

  // CO2-based DCV if sensor present
  if (config.has_co2_sensor && inputs.co2_ppm !== null) {
    const co2_setpoint = 800;  // ppm
    const co2_max = 1000;      // ppm

    if (inputs.co2_ppm > co2_setpoint) {
      // Increase ventilation
      const ratio = (inputs.co2_ppm - co2_setpoint) / (co2_max - co2_setpoint);
      minCFM = minCFM + ratio * (config.design_cfm - minCFM);
    }
  }

  // Cooling control
  const coolingError = inputs.zone_temp - inputs.cooling_setpoint;

  if (coolingError > 0) {
    // Zone needs cooling
    const coolingRequest = Math.min(100, coolingError * 20);  // 5% per 0.25°C

    // Increase airflow
    outputs.damper_position = mapRange(
      coolingRequest,
      0, 100,
      minCFM / config.design_cfm * 100,
      100
    );
    outputs.reheat_valve = 0;
  }

  // Heating control (if reheat)
  const heatingError = inputs.heating_setpoint - inputs.zone_temp;

  if (heatingError > 0 && config.box_type !== 'cooling_only') {
    // Zone needs heating
    const heatingRequest = Math.min(100, heatingError * 20);

    // Minimum airflow for heating
    outputs.damper_position = minCFM / config.design_cfm * 100;

    // Modulate reheat
    outputs.reheat_valve = heatingRequest;
  }

  // Deadband - neither heating nor cooling
  if (coolingError <= 0 && heatingError <= 0) {
    outputs.damper_position = minCFM / config.design_cfm * 100;
    outputs.reheat_valve = 0;
  }

  // Apply limits
  outputs.damper_position = clamp(outputs.damper_position, 0, 100);
  outputs.reheat_valve = clamp(outputs.reheat_valve || 0, 0, 100);

  // Calculate zone requests for AHU reset
  outputs.cooling_request = Math.max(0, coolingError * 20);
  outputs.heating_request = Math.max(0, heatingError * 20);

  return outputs;
}
```

### 5.2.3 Chiller Plant Control

**Chiller Staging and Optimization:**

```python
class ChillerPlantController:
    """
    Chiller plant optimization including staging and setpoint reset
    """

    def __init__(self, config: ChillerPlantConfig):
        self.config = config
        self.chillers = [ChillerController(c) for c in config.chillers]
        self.running_chillers: List[ChillerController] = []

    def optimize(self, inputs: PlantInputs) -> PlantOutputs:
        """
        Main plant optimization loop
        """
        outputs = PlantOutputs()

        # 1. Calculate total cooling load
        total_load_tons = self._calculate_load(inputs)

        # 2. Determine optimal number of chillers
        num_chillers = self._optimal_staging(total_load_tons)

        # 3. Stage chillers up/down
        self._stage_chillers(num_chillers, total_load_tons)

        # 4. Chilled water supply temperature reset
        outputs.chwst_setpoint = self._reset_chwst(inputs)

        # 5. Condenser water temperature reset
        outputs.cwst_setpoint = self._reset_cwst(inputs)

        # 6. Pump and tower control
        outputs.chw_pump_speed = self._control_chw_pumps(inputs)
        outputs.cw_pump_speed = self._control_cw_pumps(inputs)
        outputs.tower_fan_speed = self._control_towers(inputs, outputs.cwst_setpoint)

        return outputs

    def _optimal_staging(self, load_tons: float) -> int:
        """
        Determine optimal number of chillers based on load and efficiency
        """
        best_efficiency = float('inf')
        best_num = 1

        for num in range(1, len(self.chillers) + 1):
            # Calculate part load ratio
            capacity = sum(c.capacity_tons for c in self.chillers[:num])
            plr = load_tons / capacity

            # Skip if PLR too high (need more chillers)
            if plr > 0.95:
                continue

            # Calculate weighted efficiency at this PLR
            efficiency = self._calculate_plant_efficiency(num, plr)

            if efficiency < best_efficiency:
                best_efficiency = efficiency
                best_num = num

        return best_num

    def _calculate_plant_efficiency(self, num_chillers: int, plr: float) -> float:
        """
        Calculate kW/ton at given staging and part load ratio
        """
        # Typical centrifugal chiller IPLV curve
        # Most efficient around 50-70% load
        if plr < 0.25:
            modifier = 1.4  # Low efficiency at low loads
        elif plr < 0.50:
            modifier = 0.9
        elif plr < 0.75:
            modifier = 0.85  # Sweet spot
        else:
            modifier = 1.0

        base_efficiency = 0.55  # kW/ton at design

        return base_efficiency * modifier

    def _reset_chwst(self, inputs: PlantInputs) -> float:
        """
        Chilled water supply temperature reset based on demand
        """
        # Higher CHWST = higher efficiency but less capacity
        # Lower CHWST = lower efficiency but more capacity

        # Check AHU cooling valve positions
        max_valve = max(ahu.cooling_valve for ahu in inputs.ahus)

        if max_valve > 90:
            # Valves nearly full open, lower temperature
            new_setpoint = self.chwst_setpoint - 0.3
        elif max_valve < 70:
            # Valves have room, raise temperature
            new_setpoint = self.chwst_setpoint + 0.2
        else:
            new_setpoint = self.chwst_setpoint

        # Limit to 5-8°C range
        return clamp(new_setpoint, 5.0, 8.0)

    def _reset_cwst(self, inputs: PlantInputs) -> float:
        """
        Condenser water setpoint reset based on wet bulb
        """
        # Minimum approach to wet bulb
        min_approach = 3.0  # °C

        # Target CWST based on outdoor conditions
        target = inputs.outdoor_wet_bulb + min_approach

        # But not below minimum for chiller operation
        min_cwst = 18.0  # °C, typical minimum for centrifugal

        return max(target, min_cwst)
```

## 5.3 Optimization Algorithms

### 5.3.1 Model Predictive Control

**MPC for Building HVAC:**

```python
import numpy as np
from scipy.optimize import minimize

class BuildingMPC:
    """
    Model Predictive Control for building thermal optimization
    """

    def __init__(self, config: MPCConfig):
        self.horizon = config.prediction_horizon  # e.g., 24 hours
        self.timestep = config.timestep  # e.g., 15 minutes
        self.steps = self.horizon * 60 // self.timestep

        self.model = ThermalModel(config.building_params)

    def optimize(self, state: BuildingState,
                 forecasts: Forecasts) -> ControlTrajectory:
        """
        Solve MPC optimization problem
        """
        # Initial guess: current setpoints maintained
        x0 = self._initial_guess(state)

        # Bounds for decision variables
        bounds = self._get_bounds()

        # Constraints
        constraints = [
            {'type': 'ineq', 'fun': self._comfort_constraint, 'args': (state, forecasts)},
            {'type': 'ineq', 'fun': self._equipment_constraint}
        ]

        # Optimize
        result = minimize(
            self._objective,
            x0,
            args=(state, forecasts),
            method='SLSQP',
            bounds=bounds,
            constraints=constraints
        )

        return self._decode_solution(result.x)

    def _objective(self, x: np.ndarray, state: BuildingState,
                   forecasts: Forecasts) -> float:
        """
        Minimize energy cost while maintaining comfort
        """
        trajectory = self._decode_solution(x)
        total_cost = 0.0

        # Simulate forward
        sim_state = state.copy()

        for t in range(self.steps):
            # Apply controls
            controls = trajectory.get_controls(t)

            # Simulate one step
            energy_kwh, new_state = self.model.step(
                sim_state, controls,
                forecasts.outdoor_temp[t],
                forecasts.occupancy[t],
                forecasts.solar_gain[t]
            )

            # Energy cost
            price = forecasts.electricity_price[t]
            total_cost += energy_kwh * price

            # Comfort penalty
            for zone in new_state.zones:
                if zone.occupied:
                    deviation = max(0, abs(zone.temp - zone.setpoint) - 0.5)
                    total_cost += deviation * 10  # Penalty weight

            sim_state = new_state

        return total_cost

    def _comfort_constraint(self, x: np.ndarray, state: BuildingState,
                           forecasts: Forecasts) -> np.ndarray:
        """
        Ensure zone temperatures stay within bounds
        """
        trajectory = self._decode_solution(x)
        violations = []

        sim_state = state.copy()

        for t in range(self.steps):
            controls = trajectory.get_controls(t)
            _, new_state = self.model.step(
                sim_state, controls,
                forecasts.outdoor_temp[t],
                forecasts.occupancy[t],
                forecasts.solar_gain[t]
            )

            for zone in new_state.zones:
                if zone.occupied:
                    # Temperature must be within 19-25°C when occupied
                    violations.append(zone.temp - 19)  # Lower bound
                    violations.append(25 - zone.temp)  # Upper bound

            sim_state = new_state

        return np.array(violations)
```

### 5.3.2 Optimal Start/Stop

**Optimal Start Algorithm:**

```typescript
class OptimalStartController {
  private learningRate = 0.1;
  private heatingRate: number;  // °C per hour of operation
  private coolingRate: number;

  calculateStartTime(
    currentTemp: number,
    targetTemp: number,
    occupancyTime: Date,
    outdoorTemp: number,
    mode: 'heating' | 'cooling'
  ): Date {
    // Get current time
    const now = new Date();

    // Calculate temperature difference
    const deltaT = Math.abs(targetTemp - currentTemp);

    // Adjust rate based on outdoor conditions
    const rate = mode === 'heating' ?
      this.adjustedHeatingRate(outdoorTemp) :
      this.adjustedCoolingRate(outdoorTemp);

    // Calculate required run time
    const runTimeHours = deltaT / rate;

    // Add safety margin (10%)
    const adjustedRunTime = runTimeHours * 1.1;

    // Calculate start time
    const startTime = new Date(
      occupancyTime.getTime() - adjustedRunTime * 60 * 60 * 1000
    );

    // Don't start before current time
    return startTime > now ? startTime : now;
  }

  updateLearning(
    actualStartTime: Date,
    occupancyTime: Date,
    targetTemp: number,
    actualTempAtOccupancy: number
  ): void {
    // Learn from performance
    const error = targetTemp - actualTempAtOccupancy;

    if (Math.abs(error) > 0.5) {
      // Adjust rate based on actual performance
      const duration = (occupancyTime.getTime() - actualStartTime.getTime()) / 3600000;

      if (error > 0) {
        // Didn't reach target, need faster rate (more time)
        this.heatingRate *= (1 - this.learningRate * error);
      } else {
        // Overshot, need slower rate (less time)
        this.heatingRate *= (1 + this.learningRate * Math.abs(error));
      }
    }
  }

  private adjustedHeatingRate(outdoorTemp: number): number {
    // Lower outdoor temp = slower heating rate
    const baseFactor = 1.0;
    const tempFactor = 1 - 0.02 * (20 - outdoorTemp);
    return this.heatingRate * baseFactor * Math.max(0.5, tempFactor);
  }

  private adjustedCoolingRate(outdoorTemp: number): number {
    // Higher outdoor temp = slower cooling rate
    const baseFactor = 1.0;
    const tempFactor = 1 - 0.02 * (outdoorTemp - 25);
    return this.coolingRate * baseFactor * Math.max(0.5, tempFactor);
  }
}
```

### 5.3.3 Demand Response Control

**Grid-Interactive Control:**

```python
class DemandResponseController:
    """
    Grid demand response and load management
    """

    def __init__(self, config: DRConfig):
        self.config = config
        self.active_events: List[DREvent] = []
        self.pre_conditioning_active = False

    def handle_dr_signal(self, signal: GridSignal) -> List[DRAction]:
        """
        Handle incoming demand response signal
        """
        actions = []

        if signal.type == 'PRICE':
            actions = self._handle_price_signal(signal)
        elif signal.type == 'LOAD_SHED':
            actions = self._handle_shed_signal(signal)
        elif signal.type == 'LOAD_SHIFT':
            actions = self._handle_shift_signal(signal)

        return actions

    def _handle_price_signal(self, signal: PriceSignal) -> List[DRAction]:
        """
        Respond to time-of-use or real-time pricing
        """
        actions = []
        price_ratio = signal.price / signal.baseline_price

        if price_ratio > 2.0:  # Critical peak
            actions.extend([
                DRAction('SETPOINT_OFFSET', {'cooling': +2.0, 'heating': -2.0}),
                DRAction('LIGHTING_REDUCE', {'percent': 30}),
                DRAction('EQUIPMENT_SHED', {'types': ['non_critical']}),
            ])

        elif price_ratio > 1.5:  # High price
            actions.extend([
                DRAction('SETPOINT_OFFSET', {'cooling': +1.0, 'heating': -1.0}),
                DRAction('LIGHTING_REDUCE', {'percent': 15}),
            ])

        elif price_ratio < 0.5:  # Low price - pre-condition
            actions.extend([
                DRAction('PRE_COOL', {'target_offset': -1.0}),
                DRAction('THERMAL_STORAGE_CHARGE', {}),
            ])

        return actions

    def _handle_shed_signal(self, signal: ShedSignal) -> List[DRAction]:
        """
        Handle load shed request from utility
        """
        actions = []

        # Calculate shed capacity
        shed_needed_kw = signal.shed_request_kw
        available_shed_kw = 0

        # Priority order for shedding
        shed_priorities = [
            ('lighting_dim', self._estimate_lighting_shed()),
            ('hvac_setback', self._estimate_hvac_shed()),
            ('non_critical_off', self._estimate_non_critical_shed()),
            ('ev_charging_pause', self._estimate_ev_shed()),
        ]

        for shed_type, capacity in shed_priorities:
            if available_shed_kw < shed_needed_kw:
                actions.append(DRAction(shed_type, {'capacity_kw': capacity}))
                available_shed_kw += capacity

        return actions

    def pre_condition(self, upcoming_event: DREvent) -> List[DRAction]:
        """
        Pre-condition building before DR event
        """
        # Start pre-cooling/heating 1-2 hours before event
        lead_time = upcoming_event.start_time - datetime.now()

        if lead_time < timedelta(hours=2):
            return [
                DRAction('PRE_COOL' if upcoming_event.season == 'summer' else 'PRE_HEAT',
                        {'offset': -1.5, 'duration_minutes': lead_time.total_seconds() / 60})
            ]

        return []
```

## 5.4 Fault Detection and Diagnostics

### 5.4.1 Rule-Based FDD

**Fault Detection Rules:**

```python
class RuleBasedFDD:
    """
    Rule-based fault detection for HVAC equipment
    """

    def __init__(self):
        self.rules = self._initialize_rules()
        self.fault_history: List[Fault] = []

    def _initialize_rules(self) -> List[FDDRule]:
        return [
            # Supply fan rules
            FDDRule(
                id='SF-001',
                name='Supply Fan Not Running During Occupied',
                condition=lambda s: s.mode == 'occupied' and not s.supply_fan_running,
                severity='critical',
                probable_causes=['Motor failure', 'VFD fault', 'Control issue']
            ),

            # Temperature rules
            FDDRule(
                id='SAT-001',
                name='Supply Air Temp Too High for Cooling',
                condition=lambda s: (
                    s.mode == 'cooling' and
                    s.cooling_valve > 90 and
                    s.supply_air_temp > s.supply_air_setpoint + 3
                ),
                severity='high',
                probable_causes=['Low chilled water flow', 'Dirty coil', 'Stuck valve']
            ),

            FDDRule(
                id='SAT-002',
                name='Supply Air Temp Too Low for Heating',
                condition=lambda s: (
                    s.mode == 'heating' and
                    s.heating_valve > 90 and
                    s.supply_air_temp < s.supply_air_setpoint - 3
                ),
                severity='high',
                probable_causes=['Low hot water temp', 'Air in coil', 'Stuck valve']
            ),

            # Economizer rules
            FDDRule(
                id='ECON-001',
                name='Economizer Should Be Active But Is Not',
                condition=lambda s: (
                    s.mode == 'cooling' and
                    s.outdoor_temp < s.return_temp - 2 and
                    s.outdoor_air_damper < 50 and
                    s.cooling_valve > 50
                ),
                severity='medium',
                probable_causes=['Damper stuck', 'Actuator failed', 'Control logic error']
            ),

            FDDRule(
                id='ECON-002',
                name='Economizer Active When Outdoor Conditions Unfavorable',
                condition=lambda s: (
                    s.outdoor_air_damper > 80 and
                    s.outdoor_temp > s.return_temp + 2
                ),
                severity='medium',
                probable_causes=['Stuck damper', 'Failed OA sensor', 'Control logic error']
            ),

            # Simultaneous heating and cooling
            FDDRule(
                id='HC-001',
                name='Simultaneous Heating and Cooling',
                condition=lambda s: (
                    s.cooling_valve > 20 and
                    s.heating_valve > 20
                ),
                severity='high',
                probable_causes=['Control sequence error', 'Stuck valve', 'Sensor issue']
            ),

            # Sensor rules
            FDDRule(
                id='SENS-001',
                name='Mixed Air Temp Sensor Error',
                condition=lambda s: (
                    abs(
                        s.mixed_air_temp -
                        (s.outdoor_air_temp * s.outdoor_air_damper/100 +
                         s.return_air_temp * (100-s.outdoor_air_damper)/100)
                    ) > 5
                ),
                severity='medium',
                probable_causes=['Sensor drift', 'Poor sensor location', 'Stratification']
            ),
        ]

    def evaluate(self, state: EquipmentState) -> List[Fault]:
        """
        Evaluate all rules against current state
        """
        faults = []

        for rule in self.rules:
            try:
                if rule.condition(state):
                    fault = Fault(
                        rule_id=rule.id,
                        name=rule.name,
                        severity=rule.severity,
                        probable_causes=rule.probable_causes,
                        detected_at=datetime.now(),
                        equipment_id=state.equipment_id
                    )
                    faults.append(fault)
            except Exception as e:
                # Rule evaluation error
                pass

        return faults
```

### 5.4.2 Statistical FDD

**Anomaly Detection:**

```python
from sklearn.ensemble import IsolationForest
import numpy as np

class StatisticalFDD:
    """
    Machine learning based fault detection
    """

    def __init__(self):
        self.models: Dict[str, IsolationForest] = {}
        self.baselines: Dict[str, np.ndarray] = {}

    def train_baseline(self, equipment_id: str, historical_data: pd.DataFrame):
        """
        Train anomaly detection model on normal operation data
        """
        # Select features
        features = [
            'supply_air_temp', 'return_air_temp', 'outdoor_temp',
            'cooling_valve', 'heating_valve', 'supply_fan_speed',
            'power_kw'
        ]

        X = historical_data[features].values

        # Remove known fault periods
        X = X[historical_data['fault_flag'] == False]

        # Train isolation forest
        model = IsolationForest(
            contamination=0.01,
            random_state=42,
            n_estimators=100
        )
        model.fit(X)

        self.models[equipment_id] = model
        self.baselines[equipment_id] = {
            'mean': np.mean(X, axis=0),
            'std': np.std(X, axis=0)
        }

    def detect_anomaly(self, equipment_id: str,
                       current_data: np.ndarray) -> AnomalyResult:
        """
        Detect if current state is anomalous
        """
        model = self.models.get(equipment_id)
        if model is None:
            return AnomalyResult(is_anomaly=False, score=0)

        # Predict
        score = model.decision_function([current_data])[0]
        is_anomaly = model.predict([current_data])[0] == -1

        # Calculate feature contributions
        baseline = self.baselines[equipment_id]
        z_scores = (current_data - baseline['mean']) / baseline['std']
        top_contributors = np.argsort(np.abs(z_scores))[-3:]

        return AnomalyResult(
            is_anomaly=is_anomaly,
            score=score,
            contributing_features=top_contributors.tolist()
        )
```

## 5.5 Predictive Maintenance

### 5.5.1 Degradation Monitoring

**Equipment Health Scoring:**

```python
class EquipmentHealthMonitor:
    """
    Monitor equipment health and predict maintenance needs
    """

    def __init__(self, equipment_id: str, equipment_type: str):
        self.equipment_id = equipment_id
        self.equipment_type = equipment_type
        self.health_indicators: Dict[str, HealthIndicator] = {}

    def calculate_health_score(self, current_data: Dict) -> HealthScore:
        """
        Calculate overall equipment health score (0-100)
        """
        indicators = []

        if self.equipment_type == 'ahu':
            # Filter differential pressure
            indicators.append(self._filter_health(current_data))

            # Belt/drive health
            indicators.append(self._drive_health(current_data))

            # Coil health
            indicators.append(self._coil_health(current_data))

            # Damper health
            indicators.append(self._damper_health(current_data))

        elif self.equipment_type == 'chiller':
            # Compressor health
            indicators.append(self._compressor_health(current_data))

            # Condenser/evaporator
            indicators.append(self._heat_exchanger_health(current_data))

            # Refrigerant
            indicators.append(self._refrigerant_health(current_data))

        # Weighted average
        weights = [ind.weight for ind in indicators]
        scores = [ind.score for ind in indicators]
        overall = np.average(scores, weights=weights)

        return HealthScore(
            overall=overall,
            indicators=indicators,
            recommendations=self._generate_recommendations(indicators)
        )

    def _filter_health(self, data: Dict) -> HealthIndicator:
        """
        Assess filter condition based on differential pressure
        """
        current_dp = data.get('filter_dp_pa', 0)
        clean_dp = self.config.clean_filter_dp
        max_dp = self.config.max_filter_dp

        # Calculate remaining life
        ratio = (current_dp - clean_dp) / (max_dp - clean_dp)
        score = 100 * (1 - ratio)

        if score < 20:
            status = 'replace_now'
        elif score < 40:
            status = 'replace_soon'
        else:
            status = 'ok'

        return HealthIndicator(
            name='filter',
            score=max(0, score),
            weight=0.2,
            status=status,
            details={'dp_pa': current_dp, 'remaining_life_pct': score}
        )

    def predict_failure(self, historical_data: pd.DataFrame) -> FailurePrediction:
        """
        Predict time to failure based on degradation trends
        """
        # Fit degradation model
        X = (historical_data['timestamp'] - historical_data['timestamp'].min()).dt.days.values
        y = historical_data['health_score'].values

        # Linear regression for degradation rate
        slope, intercept = np.polyfit(X, y, 1)

        # Predict days until health score reaches threshold
        threshold = 30  # Critical health score
        current_score = y[-1]

        if slope >= 0:
            # Not degrading
            days_to_failure = float('inf')
        else:
            days_to_failure = (threshold - intercept) / slope - X[-1]

        confidence = self._calculate_confidence(X, y, slope, intercept)

        return FailurePrediction(
            days_to_failure=max(0, days_to_failure),
            confidence=confidence,
            degradation_rate=slope,
            recommended_maintenance_date=datetime.now() + timedelta(days=max(0, days_to_failure * 0.8))
        )
```

## 5.6 Implementation Best Practices

### 5.6.1 Control Loop Tuning

**PID Tuning Guidelines:**

| Parameter | Typical Range | Effect |
|-----------|---------------|--------|
| Kp (Proportional) | 1-10 | Faster response, may overshoot |
| Ki (Integral) | 0.01-1 | Eliminates steady-state error |
| Kd (Derivative) | 0-5 | Reduces overshoot, noise sensitive |

### 5.6.2 Commissioning Checklist

```markdown
## Phase 3 Commissioning Checklist

### Sequence Verification
- [ ] AHU sequences operate per specification
- [ ] VAV boxes respond correctly to load
- [ ] Chiller staging matches design
- [ ] Economizer operates in correct range
- [ ] Optimal start achieves setpoint on time

### Optimization Validation
- [ ] Supply air reset responds to zones
- [ ] Static pressure reset reduces fan energy
- [ ] Chilled water reset maintains coil capacity
- [ ] Demand response curtails expected load

### FDD Activation
- [ ] All rules tested with simulated faults
- [ ] Alert routing configured
- [ ] Baseline models trained
- [ ] Historical data validated
```

---

**Chapter Summary:**

This chapter covered the Control Protocols (Phase 3):

- Hierarchical control architecture from local loops to optimization
- HVAC control sequences for AHUs, VAVs, and chillers
- Model predictive control for building optimization
- Demand response and grid integration
- Rule-based and statistical fault detection
- Predictive maintenance with health scoring

In the next chapter, we will explore System Integration (Phase 4), covering smart grid, renewable energy, and external system connections.

---

© 2025 World Certification Industry Association (WIA)
弘益人間 (Hongik Ingan) - Benefit All Humanity
