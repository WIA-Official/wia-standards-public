/**
 * WIA-AUTO-004: Electric Vehicle Standard - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  EV_CONSTANTS,
  RangeCalculationRequest,
  RangeCalculationResponse,
  EnergyConsumptionRequest,
  EnergyConsumptionResponse,
  RegenerativeBrakingRequest,
  RegenerativeBrakingResponse,
  ChargingTimeRequest,
  ChargingTimeResponse,
  VehicleConfiguration,
  BatteryState,
  MotorState,
  DriveProfileData,
  EVErrorCode,
  ElectricVehicleError,
} from './types';

// ============================================================================
// Range Calculation
// ============================================================================

/**
 * Calculate estimated vehicle range based on current conditions
 *
 * @param request - Range calculation parameters
 * @returns Range estimate with confidence intervals
 */
export function calculateRange(
  request: RangeCalculationRequest
): RangeCalculationResponse {
  const {
    battery_capacity_kwh,
    current_soc_percent,
    average_speed_kmh,
    ambient_temp_c,
    drive_profile,
    hvac_enabled,
    terrain_type,
    driving_style_factor = 1.0,
    usable_dod = EV_CONSTANTS.TYPICAL_DOD,
    powertrain_efficiency = EV_CONSTANTS.TYPICAL_POWERTRAIN_EFFICIENCY,
  } = request;

  // Available energy
  const available_energy_kwh =
    battery_capacity_kwh * (current_soc_percent / 100) * usable_dod;

  // Base consumption (depends on drive profile and speed)
  let base_consumption_kwh_per_100km: number;

  if (drive_profile === 'urban') {
    // Urban: lower speeds, more stop-and-go
    base_consumption_kwh_per_100km = 12 + (average_speed_kmh - 40) * 0.05;
  } else if (drive_profile === 'highway') {
    // Highway: higher speeds, aerodynamic drag dominant
    const speed_factor = Math.pow(average_speed_kmh / 100, 2.5);
    base_consumption_kwh_per_100km = 14 * speed_factor;
  } else if (drive_profile === 'sport') {
    // Sport: aggressive driving
    base_consumption_kwh_per_100km = 20 + (average_speed_kmh - 60) * 0.08;
  } else if (drive_profile === 'eco') {
    // Eco: efficient driving
    base_consumption_kwh_per_100km = 10 + (average_speed_kmh - 50) * 0.04;
  } else {
    // Mixed: balanced
    base_consumption_kwh_per_100km = 16 + (average_speed_kmh - 70) * 0.06;
  }

  // Temperature impact
  let temp_impact_percent = 0;
  if (ambient_temp_c < 0) {
    temp_impact_percent = Math.min(50, Math.abs(ambient_temp_c) * 2);
  } else if (ambient_temp_c > 35) {
    temp_impact_percent = Math.min(20, (ambient_temp_c - 35) * 1.5);
  }

  // HVAC impact
  let hvac_impact_percent = 0;
  if (hvac_enabled) {
    if (ambient_temp_c < 0) {
      // Heating (resistive or heat pump)
      hvac_impact_percent = 30 - ambient_temp_c; // Worse in cold
    } else if (ambient_temp_c > 30) {
      // Cooling (AC)
      hvac_impact_percent = 5 + (ambient_temp_c - 30) * 0.5;
    }
  }

  // Terrain impact
  const terrain_factors = {
    flat: 0,
    rolling: 15,
    mountainous: 30,
  };
  const terrain_impact_percent = terrain_factors[terrain_type];

  // Style impact
  const style_impact_percent = (driving_style_factor - 1.0) * 100;

  // Total consumption
  const total_factor =
    1 +
    temp_impact_percent / 100 +
    hvac_impact_percent / 100 +
    terrain_impact_percent / 100 +
    style_impact_percent / 100;

  const energy_consumption_kwh_per_100km =
    (base_consumption_kwh_per_100km * total_factor) / powertrain_efficiency;

  // Range calculation
  const estimated_range_km =
    (available_energy_kwh / energy_consumption_kwh_per_100km) * 100;

  // Confidence and min/max ranges
  const uncertainty = 0.15; // ±15%
  const range_min_km = estimated_range_km * (1 - uncertainty);
  const range_max_km = estimated_range_km * (1 + uncertainty);
  const confidence = 0.85; // 85% confidence

  return {
    estimated_range_km,
    range_min_km,
    range_max_km,
    energy_consumption_kwh_per_100km,
    confidence,
    factors: {
      base_consumption_kwh_per_100km,
      temperature_impact_percent: temp_impact_percent,
      hvac_impact_percent,
      terrain_impact_percent,
      style_impact_percent,
    },
    available_energy_kwh,
  };
}

// ============================================================================
// Energy Consumption
// ============================================================================

/**
 * Calculate energy consumption for a given drive scenario
 *
 * @param request - Energy consumption parameters
 * @returns Detailed energy breakdown
 */
export function calculateEnergyConsumption(
  request: EnergyConsumptionRequest
): EnergyConsumptionResponse {
  const {
    distance_km,
    average_speed_kmh,
    vehicle_mass_kg,
    drag_coefficient,
    frontal_area_m2,
    rolling_resistance,
    grade_percent = 0,
    acceleration_mps2 = 0,
    ambient_temp_c = 20,
    hvac_power_kw = 0,
    powertrain_efficiency = EV_CONSTANTS.TYPICAL_POWERTRAIN_EFFICIENCY,
  } = request;

  const speed_ms = average_speed_kmh * EV_CONSTANTS.KMH_TO_MS;
  const time_hours = distance_km / average_speed_kmh;
  const time_seconds = time_hours * 3600;

  // Aerodynamic drag force
  const F_aero =
    0.5 *
    EV_CONSTANTS.AIR_DENSITY *
    drag_coefficient *
    frontal_area_m2 *
    speed_ms *
    speed_ms;

  // Rolling resistance force
  const F_roll =
    rolling_resistance *
    vehicle_mass_kg *
    EV_CONSTANTS.GRAVITY *
    Math.cos((grade_percent / 100) * Math.atan(1) * 4); // approximate

  // Grade resistance force
  const F_grade =
    vehicle_mass_kg *
    EV_CONSTANTS.GRAVITY *
    Math.sin((grade_percent / 100) * Math.atan(1) * 4);

  // Acceleration force
  const rotational_factor = 1.08; // Account for rotational inertia
  const F_accel = vehicle_mass_kg * rotational_factor * acceleration_mps2;

  // Power requirements at wheels (W)
  const P_aero_w = F_aero * speed_ms;
  const P_roll_w = F_roll * speed_ms;
  const P_grade_w = F_grade * speed_ms;
  const P_accel_w = F_accel * speed_ms;

  // Energy at wheels (kWh)
  const aerodynamic_wheel_kwh = (P_aero_w * time_seconds) * EV_CONSTANTS.J_TO_KWH;
  const rolling_wheel_kwh = (P_roll_w * time_seconds) * EV_CONSTANTS.J_TO_KWH;
  const grade_wheel_kwh = (P_grade_w * time_seconds) * EV_CONSTANTS.J_TO_KWH;
  const acceleration_wheel_kwh = (P_accel_w * time_seconds) * EV_CONSTANTS.J_TO_KWH;

  // Total mechanical energy
  const mechanical_energy_kwh =
    Math.max(0, aerodynamic_wheel_kwh) +
    Math.max(0, rolling_wheel_kwh) +
    Math.max(0, grade_wheel_kwh) +
    Math.max(0, acceleration_wheel_kwh);

  // Drivetrain loss
  const drivetrain_loss_kwh =
    mechanical_energy_kwh * ((1 - powertrain_efficiency) / powertrain_efficiency);

  // HVAC energy
  const hvac_kwh = hvac_power_kw * time_hours;

  // Accessories (estimated)
  const accessories_kwh = 0.3 * time_hours; // ~300W baseline

  // Total energy from battery
  const total_energy_kwh =
    mechanical_energy_kwh +
    drivetrain_loss_kwh +
    hvac_kwh +
    accessories_kwh;

  const energy_per_km = total_energy_kwh / distance_km;
  const energy_per_100km = energy_per_km * 100;

  const efficiency_percent =
    (mechanical_energy_kwh / total_energy_kwh) * 100;

  return {
    total_energy_kwh,
    energy_per_km,
    energy_per_100km,
    breakdown: {
      aerodynamic_kwh: aerodynamic_wheel_kwh,
      rolling_kwh: rolling_wheel_kwh,
      grade_kwh: grade_wheel_kwh,
      acceleration_kwh: acceleration_wheel_kwh,
      hvac_kwh,
      accessories_kwh,
      drivetrain_loss_kwh,
    },
    efficiency_percent,
  };
}

// ============================================================================
// Regenerative Braking
// ============================================================================

/**
 * Calculate energy recovered through regenerative braking
 *
 * @param request - Regenerative braking parameters
 * @returns Energy recovery breakdown
 */
export function calculateRegenerativeEnergy(
  request: RegenerativeBrakingRequest
): RegenerativeBrakingResponse {
  const {
    vehicle_mass_kg,
    initial_speed_kmh,
    final_speed_kmh,
    motor_efficiency,
    inverter_efficiency,
    battery_efficiency,
    current_soc_percent,
    battery_temp_c,
    max_regen_power_kw,
  } = request;

  // Convert speeds to m/s
  const v_initial_ms = initial_speed_kmh * EV_CONSTANTS.KMH_TO_MS;
  const v_final_ms = final_speed_kmh * EV_CONSTANTS.KMH_TO_MS;

  // Kinetic energy available
  const KE_initial_j = 0.5 * vehicle_mass_kg * v_initial_ms * v_initial_ms;
  const KE_final_j = 0.5 * vehicle_mass_kg * v_final_ms * v_final_ms;
  const kinetic_energy_j = KE_initial_j - KE_final_j;
  const kinetic_energy_kwh = kinetic_energy_j * EV_CONSTANTS.J_TO_KWH;

  // Overall recovery efficiency
  const overall_efficiency =
    motor_efficiency * inverter_efficiency * battery_efficiency;
  const recoverable_energy_kwh = kinetic_energy_kwh * overall_efficiency;

  // Check limitations
  const limitations: string[] = [];
  let actual_recovered_kwh = recoverable_energy_kwh;

  // SoC limitation (can't charge if battery is full)
  if (current_soc_percent > 95) {
    limitations.push('battery_soc_high');
    actual_recovered_kwh *= 0.5; // Reduce recovery at high SoC
  } else if (current_soc_percent > 90) {
    actual_recovered_kwh *= 0.8;
  }

  // Temperature limitation
  if (battery_temp_c < 0) {
    limitations.push('battery_temp_low');
    actual_recovered_kwh *= 0.6; // Reduced charge acceptance in cold
  } else if (battery_temp_c < 10) {
    actual_recovered_kwh *= 0.85;
  }

  if (battery_temp_c > 45) {
    limitations.push('battery_temp_high');
    actual_recovered_kwh *= 0.9;
  }

  // Power limitation
  if (max_regen_power_kw) {
    const delta_v_ms = v_initial_ms - v_final_ms;
    const decel_time_s = delta_v_ms / request.deceleration_mps2;
    const average_power_kw = (kinetic_energy_kwh / decel_time_s) * 3600;

    if (average_power_kw > max_regen_power_kw) {
      limitations.push('power_limit');
      actual_recovered_kwh *= max_regen_power_kw / average_power_kw;
    }
  }

  // Speed limitation (low speed regen is limited)
  if (final_speed_kmh < 5) {
    limitations.push('speed_too_low');
    actual_recovered_kwh *= 0.7;
  }

  const recovery_efficiency_percent =
    (actual_recovered_kwh / kinetic_energy_kwh) * 100;

  // Estimate range extension (assume 18 kWh/100km consumption)
  const range_extension_km = (actual_recovered_kwh / 0.18) * 100;

  const average_regen_power_kw =
    (kinetic_energy_kwh / (delta_v_ms / request.deceleration_mps2)) * 3600;

  return {
    kinetic_energy_kwh,
    recoverable_energy_kwh,
    actual_recovered_kwh,
    recovery_efficiency_percent,
    limitations,
    range_extension_km,
    average_regen_power_kw,
  };
}

// ============================================================================
// Charging Time
// ============================================================================

/**
 * Calculate charging time for given conditions
 *
 * @param request - Charging time parameters
 * @returns Charging time estimate with curve
 */
export function calculateChargingTime(
  request: ChargingTimeRequest
): ChargingTimeResponse {
  const {
    battery_capacity_kwh,
    current_soc_percent,
    target_soc_percent,
    charger_power_kw,
    charger_type,
    battery_temp_c,
    ambient_temp_c = 20,
  } = request;

  // Energy to deliver
  const energy_to_deliver_kwh =
    (battery_capacity_kwh * (target_soc_percent - current_soc_percent)) / 100;

  // Charging efficiency
  let charging_efficiency = 0.95;
  if (charger_type === 'AC_L1' || charger_type === 'AC_L2') {
    charging_efficiency = 0.90; // OBC losses
  } else if (charger_type === 'DC_FAST') {
    charging_efficiency = 0.95;
  } else {
    charging_efficiency = 0.85; // Wireless
  }

  // Temperature-based limitations
  let temp_power_factor = 1.0;
  let precondition_time_minutes = 0;

  if (battery_temp_c < 15) {
    // Cold battery: slow charging initially
    temp_power_factor = 0.5;
    precondition_time_minutes = Math.max(0, (15 - battery_temp_c) * 2);
  } else if (battery_temp_c > 40) {
    // Hot battery: reduced charging
    temp_power_factor = 0.7;
    precondition_time_minutes = Math.max(0, (battery_temp_c - 40) * 1.5);
  }

  // Build charging curve
  const charging_curve: Array<{
    time_minutes: number;
    soc_percent: number;
    power_kw: number;
  }> = [];

  let current_soc = current_soc_percent;
  let elapsed_minutes = precondition_time_minutes;
  let energy_delivered = 0;

  // Preconditioning phase
  if (precondition_time_minutes > 0) {
    charging_curve.push({
      time_minutes: 0,
      soc_percent: current_soc,
      power_kw: charger_power_kw * 0.3, // Low power during precondition
    });
    charging_curve.push({
      time_minutes: precondition_time_minutes,
      soc_percent: current_soc,
      power_kw: charger_power_kw * temp_power_factor,
    });
  }

  // Charging phases
  const time_step_minutes = 5;

  while (current_soc < target_soc_percent && elapsed_minutes < 300) {
    // Determine power based on SoC (simplified charging curve)
    let power_kw: number;

    if (charger_type === 'DC_FAST') {
      // DC fast charging curve
      if (current_soc < 50) {
        power_kw = charger_power_kw * temp_power_factor;
      } else if (current_soc < 80) {
        power_kw = charger_power_kw * temp_power_factor * (1 - (current_soc - 50) / 60);
      } else {
        power_kw = charger_power_kw * temp_power_factor * 0.3 * Math.exp(-(current_soc - 80) / 10);
      }
    } else {
      // AC charging: more constant
      if (current_soc < 90) {
        power_kw = charger_power_kw * temp_power_factor;
      } else {
        power_kw = charger_power_kw * temp_power_factor * 0.5 * Math.exp(-(current_soc - 90) / 5);
      }
    }

    // Energy delivered in this time step
    const energy_step_kwh = (power_kw * time_step_minutes) / 60;
    energy_delivered += energy_step_kwh * charging_efficiency;

    // Update SoC
    current_soc = current_soc_percent + (energy_delivered / battery_capacity_kwh) * 100;
    elapsed_minutes += time_step_minutes;

    charging_curve.push({
      time_minutes: elapsed_minutes,
      soc_percent: Math.min(current_soc, target_soc_percent),
      power_kw,
    });

    if (current_soc >= target_soc_percent) break;
  }

  const estimated_time_minutes = elapsed_minutes;
  const average_power_kw = energy_to_deliver_kwh / (estimated_time_minutes / 60);

  const warnings: string[] = [];
  if (battery_temp_c < 10) {
    warnings.push('Battery temperature is low, charging will be slower');
  }
  if (battery_temp_c > 45) {
    warnings.push('Battery temperature is high, charging power may be limited');
  }
  if (target_soc_percent > 80 && charger_type === 'DC_FAST') {
    warnings.push('Charging slows significantly above 80% SoC for DC fast charging');
  }

  return {
    estimated_time_minutes,
    energy_to_deliver_kwh,
    average_power_kw,
    charging_curve,
    precondition_time_minutes: precondition_time_minutes > 0 ? precondition_time_minutes : undefined,
    warnings: warnings.length > 0 ? warnings : undefined,
  };
}

// ============================================================================
// SDK Class
// ============================================================================

/**
 * Main Electric Vehicle SDK class
 */
export class ElectricVehicleSDK {
  private vehicle: VehicleConfiguration;

  constructor(vehicle: VehicleConfiguration) {
    this.vehicle = vehicle;
  }

  /**
   * Calculate range for current vehicle configuration
   */
  public calculateRange(params: {
    currentSoC: number;
    averageSpeed: number;
    ambientTemp?: number;
    profile?: 'urban' | 'highway' | 'mixed' | 'sport' | 'eco';
    hvacEnabled?: boolean;
    terrain?: TerrainType;
  }): RangeCalculationResponse {
    return calculateRange({
      battery_capacity_kwh: this.vehicle.battery.capacity_kwh,
      current_soc_percent: params.currentSoC,
      average_speed_kmh: params.averageSpeed,
      ambient_temp_c: params.ambientTemp ?? 20,
      drive_profile: params.profile ?? 'mixed',
      hvac_enabled: params.hvacEnabled ?? true,
      terrain_type: params.terrain ?? 'flat',
      vehicle_specs: this.vehicle.vehicle_specs,
      powertrain_efficiency: this.vehicle.powertrain.gearbox_efficiency_percent / 100,
    });
  }

  /**
   * Calculate energy consumption for a trip
   */
  public calculateTripEnergy(params: {
    distance: number;
    averageSpeed: number;
    grade?: number;
    ambientTemp?: number;
    hvacPower?: number;
  }): EnergyConsumptionResponse {
    return calculateEnergyConsumption({
      distance_km: params.distance,
      average_speed_kmh: params.averageSpeed,
      vehicle_mass_kg: this.vehicle.vehicle_specs.mass_kg,
      drag_coefficient: this.vehicle.vehicle_specs.drag_coefficient,
      frontal_area_m2: this.vehicle.vehicle_specs.frontal_area_m2,
      rolling_resistance: this.vehicle.vehicle_specs.rolling_resistance,
      grade_percent: params.grade,
      ambient_temp_c: params.ambientTemp,
      hvac_power_kw: params.hvacPower,
      powertrain_efficiency: this.vehicle.powertrain.gearbox_efficiency_percent / 100,
    });
  }

  /**
   * Calculate regenerative braking energy recovery
   */
  public calculateRegenEnergy(params: {
    initialSpeed: number;
    finalSpeed: number;
    decelerationRate: number;
    currentSoC: number;
    batteryTemp: number;
  }): RegenerativeBrakingResponse {
    return calculateRegenerativeEnergy({
      vehicle_mass_kg: this.vehicle.vehicle_specs.mass_kg,
      initial_speed_kmh: params.initialSpeed,
      final_speed_kmh: params.finalSpeed,
      deceleration_mps2: params.decelerationRate,
      motor_efficiency: this.vehicle.motor.efficiency_percent / 100,
      inverter_efficiency: this.vehicle.inverter.efficiency_percent / 100,
      battery_efficiency: 0.96,
      current_soc_percent: params.currentSoC,
      battery_temp_c: params.batteryTemp,
      max_regen_power_kw: this.vehicle.motor.power_rated_kw * 0.7,
    });
  }

  /**
   * Calculate charging time
   */
  public calculateChargingTime(params: {
    currentSoC: number;
    targetSoC: number;
    chargerPower: number;
    chargerType: 'AC_L1' | 'AC_L2' | 'DC_FAST';
    batteryTemp: number;
  }): ChargingTimeResponse {
    return calculateChargingTime({
      battery_capacity_kwh: this.vehicle.battery.capacity_kwh,
      current_soc_percent: params.currentSoC,
      target_soc_percent: params.targetSoC,
      charger_power_kw: params.chargerPower,
      charger_type: params.chargerType,
      battery_temp_c: params.batteryTemp,
    });
  }

  /**
   * Get vehicle configuration
   */
  public getVehicle(): VehicleConfiguration {
    return this.vehicle;
  }

  /**
   * Update vehicle configuration
   */
  public updateVehicle(vehicle: Partial<VehicleConfiguration>): void {
    this.vehicle = { ...this.vehicle, ...vehicle };
  }
}

// ============================================================================
// Exports
// ============================================================================

export {
  calculateRange,
  calculateEnergyConsumption,
  calculateRegenerativeEnergy,
  calculateChargingTime,
  ElectricVehicleSDK,
};

export * from './types';
