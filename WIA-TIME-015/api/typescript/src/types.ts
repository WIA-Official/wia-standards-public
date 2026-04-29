/**
 * WIA-TIME-015: Time Machine Hardware - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Temporal Hardware Engineering Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Hardware Types
// ============================================================================

/**
 * Hardware component status
 */
export type ComponentStatus =
  | 'offline'
  | 'initializing'
  | 'standby'
  | 'ready'
  | 'active'
  | 'warning'
  | 'error'
  | 'critical'
  | 'emergency-shutdown';

/**
 * Hardware health indicator
 */
export interface HealthIndicator {
  /** Component name */
  component: string;

  /** Current status */
  status: ComponentStatus;

  /** Health percentage (0-100) */
  health: number;

  /** Operating hours */
  operatingHours: number;

  /** Time until next maintenance (hours) */
  maintenanceDue?: number;

  /** Active warnings */
  warnings: string[];

  /** Active alarms */
  alarms: string[];

  /** Last diagnostic timestamp */
  lastDiagnostic: Date;
}

// ============================================================================
// Flux Capacitor
// ============================================================================

/**
 * Flux capacitor configuration
 */
export interface FluxCapacitorConfig {
  /** Peak power in watts (1.21 GW typical) */
  power: number;

  /** Operating voltage in kilovolts */
  voltage: number;

  /** Temporal conversion efficiency (0-1) */
  efficiency: number;

  /** Cooling system type */
  coolingSystem: 'liquid-nitrogen' | 'liquid-helium' | 'cryocooler';

  /** Operating temperature in Kelvin */
  operatingTemperature: number;

  /** Superconductor material */
  material: 'YBCO' | 'BSCCO' | 'NbTi' | 'Nb3Sn';

  /** Electrode configuration */
  electrodeConfig: 'Y-shaped' | 'toroidal' | 'helical';

  /** Enable automatic shutdown on overheat */
  autoShutdown: boolean;
}

/**
 * Flux capacitor state
 */
export interface FluxCapacitorState {
  /** Current status */
  status: ComponentStatus;

  /** Current charge level (0-1) */
  chargeLevel: number;

  /** Current temperature in Kelvin */
  temperature: number;

  /** Current voltage in kilovolts */
  voltage: number;

  /** Current power output in watts */
  powerOutput: number;

  /** Temporal efficiency (actual vs theoretical) */
  efficiency: number;

  /** Coolant level (0-1) */
  coolantLevel: number;

  /** Coolant flow rate in L/min */
  coolantFlowRate: number;

  /** Time until fully charged (seconds) */
  timeToFullCharge?: number;

  /** Discharge capacity remaining (joules) */
  dischargeCapacity: number;

  /** Number of discharge cycles */
  dischargeCycles: number;

  /** Health indicators */
  health: HealthIndicator;
}

/**
 * Flux capacitor discharge parameters
 */
export interface DischargeParameters {
  /** Target power level (0-1) */
  powerLevel: number;

  /** Discharge duration in seconds */
  duration: number;

  /** Pulse mode or continuous */
  mode: 'pulse' | 'continuous';

  /** Pulse frequency in Hz (for pulse mode) */
  pulseFrequency?: number;

  /** Safety limit override (requires authorization) */
  overrideSafety?: boolean;
}

/**
 * Flux capacitor diagnostics result
 */
export interface FluxCapacitorDiagnostics {
  /** Overall status */
  status: 'pass' | 'warning' | 'fail';

  /** Electrode integrity (0-1) */
  electrodeIntegrity: number;

  /** Insulation resistance in megohms */
  insulationResistance: number;

  /** Cooling system efficiency (0-1) */
  coolingEfficiency: number;

  /** Temporal conversion accuracy (0-1) */
  conversionAccuracy: number;

  /** Superconductor critical current (amperes) */
  criticalCurrent: number;

  /** Detected issues */
  issues: {
    severity: 'info' | 'warning' | 'critical';
    code: string;
    description: string;
    recommendation: string;
  }[];

  /** Next calibration due */
  nextCalibrationDue: Date;

  /** Estimated remaining service life (hours) */
  remainingServiceLife: number;
}

// ============================================================================
// Temporal Field Generator
// ============================================================================

/**
 * Temporal field generator configuration
 */
export interface TemporalFieldGeneratorConfig {
  /** Field strength in Tesla */
  strength: number;

  /** Field uniformity (0-1, 0.999 typical) */
  uniformity: number;

  /** Stabilizer type */
  stabilizerType: 'active-feedback' | 'passive' | 'hybrid';

  /** Field geometry */
  geometry: 'toroidal' | 'spherical' | 'cylindrical';

  /** Magnet type */
  magnetType: 'superconducting' | 'permanent' | 'electromagnet';

  /** Cooling system */
  coolingSystem: 'liquid-helium' | 'cryocooler' | 'liquid-nitrogen';

  /** Field rotation frequency in Hz */
  rotationFrequency: number;

  /** Coverage angle in degrees (360 for full sphere) */
  coverage: number;

  /** Active zone radius in meters */
  activeZoneRadius: number;
}

/**
 * Temporal field generator state
 */
export interface TemporalFieldGeneratorState {
  /** Current status */
  status: ComponentStatus;

  /** Field strength (actual) in Tesla */
  fieldStrength: number;

  /** Field uniformity (actual, 0-1) */
  fieldUniformity: number;

  /** Field stability (0-1) */
  fieldStability: number;

  /** Rotation frequency in Hz */
  rotationFrequency: number;

  /** Phase coherence (0-1) */
  phaseCoherence: number;

  /** Magnet temperature in Kelvin */
  magnetTemperature: number;

  /** Cooling system status */
  coolingStatus: ComponentStatus;

  /** Power consumption in watts */
  powerConsumption: number;

  /** Plasma current in amperes (for toroidal) */
  plasmaCurrent?: number;

  /** Stabilization corrections per second */
  stabilizationRate: number;

  /** Health indicators */
  health: HealthIndicator;
}

/**
 * Field mapping data
 */
export interface FieldMapping {
  /** Timestamp of measurement */
  timestamp: Date;

  /** 3D grid of field measurements */
  fieldData: {
    position: { x: number; y: number; z: number };
    strength: number;
    direction: { x: number; y: number; z: number };
  }[];

  /** Field uniformity analysis */
  uniformityAnalysis: {
    average: number;
    stdDev: number;
    min: number;
    max: number;
    uniformity: number;
  };

  /** Detected anomalies */
  anomalies: {
    position: { x: number; y: number; z: number };
    type: 'hotspot' | 'void' | 'distortion';
    severity: number;
  }[];
}

// ============================================================================
// Chrono-Navigation System
// ============================================================================

/**
 * Navigation target coordinates
 */
export interface NavigationTarget {
  /** Target time (ISO 8601) */
  targetTime: Date | string;

  /** Spatial coordinates */
  location: {
    /** Latitude in degrees */
    latitude: number;

    /** Longitude in degrees */
    longitude: number;

    /** Altitude in meters (MSL) */
    altitude: number;
  };

  /** Reference frame */
  referenceFrame: 'ICRF' | 'GCRF' | 'ECEF' | 'ECI';

  /** Spatial lock enabled */
  spatialLock: boolean;

  /** Allow temporal drift (for safety) */
  allowDrift?: {
    temporal: number; // seconds
    spatial: number; // meters
  };
}

/**
 * Navigation system configuration
 */
export interface NavigationSystemConfig {
  /** Temporal accuracy in seconds per century */
  temporalAccuracy: number;

  /** Spatial accuracy in meters */
  spatialAccuracy: number;

  /** Processing power in FLOPS */
  processingPower: number;

  /** Quantum processor qubits */
  quantumQubits?: number;

  /** Memory capacity in bytes */
  memoryCapacity: number;

  /** Update rate in Hz */
  updateRate: number;

  /** Enable paradox detection */
  paradoxDetection: boolean;

  /** Enable timeline integrity monitoring */
  timelineMonitoring: boolean;

  /** Reference time source */
  timeSource: 'TAI' | 'UTC' | 'GPS' | 'atomic';
}

/**
 * Navigation system state
 */
export interface NavigationSystemState {
  /** Current status */
  status: ComponentStatus;

  /** Current position and time */
  current: {
    time: Date;
    location: {
      latitude: number;
      longitude: number;
      altitude: number;
    };
    velocity: {
      x: number;
      y: number;
      z: number;
    };
    attitude: {
      roll: number;
      pitch: number;
      yaw: number;
    };
  };

  /** Active target */
  target?: NavigationTarget;

  /** Trajectory calculation status */
  trajectoryStatus: 'none' | 'calculating' | 'ready' | 'executing';

  /** Time drift from reference (seconds) */
  timeDrift: number;

  /** Position accuracy (meters) */
  positionAccuracy: number;

  /** Velocity accuracy (m/s) */
  velocityAccuracy: number;

  /** Processor utilization (0-1) */
  processorLoad: number;

  /** Quantum coherence time (seconds) */
  quantumCoherence?: number;

  /** Health indicators */
  health: HealthIndicator;
}

/**
 * Trajectory plan
 */
export interface TrajectoryPlan {
  /** Unique trajectory ID */
  id: string;

  /** Origin coordinates */
  origin: NavigationTarget;

  /** Destination coordinates */
  destination: NavigationTarget;

  /** Temporal displacement in seconds */
  displacement: number;

  /** Spatial distance in meters */
  spatialDistance: number;

  /** Travel method */
  method: 'wormhole' | 'ctc' | 'alcubierre' | 'field' | 'natural';

  /** Energy requirement in joules */
  energyRequired: number;

  /** Estimated duration (proper time) in seconds */
  duration: number;

  /** Safety score (0-1) */
  safetyScore: number;

  /** Paradox risk assessment */
  paradoxRisk: 'none' | 'low' | 'medium' | 'high' | 'critical';

  /** Timeline integrity impact */
  timelineImpact: 'none' | 'minimal' | 'moderate' | 'significant';

  /** Waypoints */
  waypoints: {
    time: Date;
    location: { latitude: number; longitude: number; altitude: number };
    fieldStrength: number;
    energyLevel: number;
  }[];

  /** Contingency plans */
  contingencies: {
    trigger: string;
    action: string;
    abortTarget: NavigationTarget;
  }[];

  /** Approval status */
  approved: boolean;

  /** Approval authority */
  approvedBy?: string;

  /** Approval timestamp */
  approvedAt?: Date;
}

// ============================================================================
// Power Coupling System
// ============================================================================

/**
 * Power coupling configuration
 */
export interface PowerCouplingConfig {
  /** Input voltage in kilovolts */
  inputVoltage: number;

  /** Input frequency in Hz */
  inputFrequency: number;

  /** Total capacity in watts */
  totalCapacity: number;

  /** Number of transformer stages */
  transformerStages: number;

  /** Energy storage type */
  storageType: 'capacitor' | 'flywheel' | 'battery' | 'supercapacitor';

  /** Storage capacity in joules */
  storageCapacity: number;

  /** Backup power enabled */
  backupEnabled: boolean;

  /** Power quality monitoring */
  qualityMonitoring: boolean;
}

/**
 * Power coupling state
 */
export interface PowerCouplingState {
  /** Current status */
  status: ComponentStatus;

  /** Input power in watts */
  inputPower: number;

  /** Output power in watts */
  outputPower: number;

  /** Power factor (0-1) */
  powerFactor: number;

  /** Efficiency (0-1) */
  efficiency: number;

  /** Energy storage level (0-1) */
  storageLevel: number;

  /** Voltage regulation quality */
  voltageRegulation: {
    nominal: number;
    actual: number;
    deviation: number;
    thd: number; // Total Harmonic Distortion
  };

  /** Frequency regulation */
  frequencyRegulation: {
    nominal: number;
    actual: number;
    deviation: number;
  };

  /** Load distribution */
  loadDistribution: {
    fluxCapacitor: number;
    fieldGenerator: number;
    navigation: number;
    cooling: number;
    control: number;
    other: number;
  };

  /** Backup power status */
  backupStatus: 'offline' | 'standby' | 'active';

  /** Health indicators */
  health: HealthIndicator;
}

/**
 * Power quality metrics
 */
export interface PowerQuality {
  /** Timestamp */
  timestamp: Date;

  /** Voltage (RMS) */
  voltage: {
    L1: number;
    L2: number;
    L3: number;
    average: number;
  };

  /** Current (RMS) */
  current: {
    L1: number;
    L2: number;
    L3: number;
    neutral: number;
  };

  /** Power factor per phase */
  powerFactor: {
    L1: number;
    L2: number;
    L3: number;
    total: number;
  };

  /** Total Harmonic Distortion */
  thd: {
    voltage: number;
    current: number;
  };

  /** Frequency */
  frequency: number;

  /** Voltage unbalance (%) */
  voltageUnbalance: number;

  /** Transient events detected */
  transients: {
    type: 'sag' | 'swell' | 'interruption' | 'spike';
    magnitude: number;
    duration: number;
    timestamp: Date;
  }[];
}

// ============================================================================
// Shielding System
// ============================================================================

/**
 * Shielding configuration
 */
export interface ShieldingConfig {
  /** Radiation shielding layers */
  radiationShielding: {
    material: 'lead' | 'tungsten' | 'concrete' | 'polyethylene' | 'borated-polyethylene';
    thickness: number; // mm
    attenuation: number; // factor
  }[];

  /** Electromagnetic shielding */
  emShielding: {
    material: 'mu-metal' | 'copper' | 'aluminum';
    layers: number;
    effectiveness: number; // dB
  };

  /** Temporal field shielding */
  temporalShielding: {
    type: 'active' | 'passive' | 'hybrid';
    attenuation: number; // factor
    powerRequirement: number; // watts
  };

  /** Acoustic shielding */
  acousticShielding: {
    enabled: boolean;
    attenuation: number; // dB
  };
}

/**
 * Shielding system state
 */
export interface ShieldingSystemState {
  /** Current status */
  status: ComponentStatus;

  /** Radiation levels */
  radiationLevels: {
    inside: number; // μSv/hr
    outside: number; // μSv/hr
    shieldingEffectiveness: number; // %
  };

  /** Magnetic field levels */
  magneticFields: {
    inside: number; // gauss
    outside: number; // gauss
    shieldingEffectiveness: number; // dB
  };

  /** Temporal field leakage */
  temporalLeakage: {
    gradient: number; // seconds per meter
    shieldingEffectiveness: number; // %
  };

  /** Acoustic levels */
  acousticLevels: {
    inside: number; // dBA
    outside: number; // dBA
    attenuation: number; // dB
  };

  /** Health indicators */
  health: HealthIndicator;
}

// ============================================================================
// Control Interface
// ============================================================================

/**
 * Control interface configuration
 */
export interface ControlInterfaceConfig {
  /** Number of displays */
  displayCount: number;

  /** Display resolution */
  displayResolution: {
    width: number;
    height: number;
    refreshRate: number;
  };

  /** Automation level (0-5) */
  automationLevel: 0 | 1 | 2 | 3 | 4 | 5;

  /** Voice control enabled */
  voiceControl: boolean;

  /** Haptic feedback enabled */
  hapticFeedback: boolean;

  /** Accessibility features */
  accessibility: {
    screenReader: boolean;
    highContrast: boolean;
    largeText: boolean;
    colorBlindMode: boolean;
  };

  /** Alarm management */
  alarmConfig: {
    audioEnabled: boolean;
    visualEnabled: boolean;
    prioritization: 'ISA-18.2' | 'custom';
  };
}

/**
 * Control interface state
 */
export interface ControlInterfaceState {
  /** Current status */
  status: ComponentStatus;

  /** Active operator */
  operator?: {
    id: string;
    name: string;
    role: string;
    loginTime: Date;
  };

  /** Current view/screen */
  currentView: string;

  /** Active alarms */
  activeAlarms: {
    level: 'info' | 'warning' | 'alarm' | 'emergency';
    code: string;
    message: string;
    timestamp: Date;
    acknowledged: boolean;
  }[];

  /** System mode */
  systemMode: 'manual' | 'assisted' | 'semi-auto' | 'auto';

  /** Interlock status */
  interlocks: {
    name: string;
    active: boolean;
    reason?: string;
  }[];

  /** Communication status */
  communications: {
    voice: ComponentStatus;
    data: ComponentStatus;
    emergency: ComponentStatus;
  };

  /** Health indicators */
  health: HealthIndicator;
}

// ============================================================================
// Complete Time Machine
// ============================================================================

/**
 * Complete time machine configuration
 */
export interface TimeMachineConfig {
  /** Machine identification */
  id: string;
  name: string;
  location: string;

  /** Component configurations */
  fluxCapacitor: FluxCapacitorConfig;
  fieldGenerator: TemporalFieldGeneratorConfig;
  navigation: NavigationSystemConfig;
  powerCoupling: PowerCouplingConfig;
  shielding: ShieldingConfig;
  controlInterface: ControlInterfaceConfig;

  /** Safety settings */
  safetySettings: {
    maxEnergy: number; // joules
    maxDisplacement: number; // seconds
    requireApproval: boolean;
    emergencyShutdownEnabled: boolean;
    redundancyLevel: number;
  };

  /** Maintenance schedule */
  maintenanceSchedule: {
    dailyChecks: boolean;
    weeklyMaintenance: boolean;
    monthlyProcedures: boolean;
    annualCertification: boolean;
  };
}

/**
 * Complete time machine state
 */
export interface TimeMachineState {
  /** Overall status */
  status: ComponentStatus;

  /** Component states */
  fluxCapacitor: FluxCapacitorState;
  fieldGenerator: TemporalFieldGeneratorState;
  navigation: NavigationSystemState;
  powerCoupling: PowerCouplingState;
  shielding: ShieldingSystemState;
  controlInterface: ControlInterfaceState;

  /** Overall health */
  overallHealth: {
    percentage: number;
    status: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';
    readiness: 'ready' | 'not-ready' | 'degraded';
  };

  /** Active mission */
  activeMission?: {
    id: string;
    name: string;
    startTime: Date;
    trajectory: TrajectoryPlan;
    phase: 'preparation' | 'charging' | 'displacement' | 'transit' | 'arrival' | 'return';
  };

  /** Operational metrics */
  metrics: {
    totalDisplacements: number;
    totalEnergyConsumed: number; // joules
    totalOperatingHours: number;
    lastMaintenance: Date;
    nextMaintenance: Date;
    certificationExpiry: Date;
  };
}

/**
 * Pre-flight check result
 */
export interface PreFlightCheckResult {
  /** Overall go/no-go decision */
  allSystemsGo: boolean;

  /** Timestamp of check */
  timestamp: Date;

  /** Checks performed */
  checks: {
    category: string;
    item: string;
    status: 'pass' | 'warning' | 'fail';
    details?: string;
  }[];

  /** Critical issues */
  criticalIssues: string[];

  /** Warnings */
  warnings: string[];

  /** Recommendations */
  recommendations: string[];

  /** Approval required */
  approvalRequired: boolean;

  /** Approved by */
  approvedBy?: {
    name: string;
    role: string;
    timestamp: Date;
  };
}

/**
 * Diagnostics options
 */
export interface DiagnosticsOptions {
  /** Run full diagnostic suite */
  full: boolean;

  /** Specific components to test */
  components?: ('flux-capacitor' | 'field-generator' | 'navigation' | 'power' | 'shielding' | 'control')[];

  /** Include stress testing */
  stressTest: boolean;

  /** Generate detailed report */
  detailedReport: boolean;

  /** Save results to file */
  saveResults: boolean;
}

/**
 * Diagnostics result
 */
export interface DiagnosticsResult {
  /** Overall result */
  status: 'pass' | 'warning' | 'fail';

  /** Timestamp */
  timestamp: Date;

  /** Duration of diagnostics (seconds) */
  duration: number;

  /** Component results */
  components: {
    fluxCapacitor?: FluxCapacitorDiagnostics;
    fieldGenerator?: {
      status: 'pass' | 'warning' | 'fail';
      fieldMapping: FieldMapping;
      coolingEfficiency: number;
      magneticIntegrity: number;
      issues: Array<{
        severity: 'info' | 'warning' | 'critical';
        description: string;
      }>;
    };
    navigation?: {
      status: 'pass' | 'warning' | 'fail';
      accuracy: { temporal: number; spatial: number };
      processorHealth: number;
      quantumCoherence?: number;
      issues: Array<{
        severity: 'info' | 'warning' | 'critical';
        description: string;
      }>;
    };
    power?: {
      status: 'pass' | 'warning' | 'fail';
      quality: PowerQuality;
      efficiency: number;
      storageHealth: number;
      issues: Array<{
        severity: 'info' | 'warning' | 'critical';
        description: string;
      }>;
    };
    shielding?: {
      status: 'pass' | 'warning' | 'fail';
      effectiveness: {
        radiation: number;
        electromagnetic: number;
        temporal: number;
      };
      issues: Array<{
        severity: 'info' | 'warning' | 'critical';
        description: string;
      }>;
    };
    control?: {
      status: 'pass' | 'warning' | 'fail';
      responseTime: number;
      alarmSystem: boolean;
      interlocks: boolean;
      issues: Array<{
        severity: 'info' | 'warning' | 'critical';
        description: string;
      }>;
    };
  };

  /** Summary */
  summary: {
    totalChecks: number;
    passed: number;
    warnings: number;
    failed: number;
    criticalIssues: number;
  };

  /** Recommendations */
  recommendations: string[];

  /** Next diagnostic due */
  nextDiagnosticDue: Date;
}

/**
 * Maintenance record
 */
export interface MaintenanceRecord {
  /** Record ID */
  id: string;

  /** Maintenance type */
  type: 'daily' | 'weekly' | 'monthly' | 'annual' | 'corrective';

  /** Timestamp */
  timestamp: Date;

  /** Performed by */
  performedBy: {
    name: string;
    role: string;
    certification: string;
  };

  /** Components serviced */
  components: string[];

  /** Work performed */
  workPerformed: {
    description: string;
    duration: number; // minutes
    parts: Array<{
      partNumber: string;
      description: string;
      quantity: number;
      serialNumber?: string;
    }>;
  }[];

  /** Issues found */
  issuesFound: Array<{
    component: string;
    severity: 'info' | 'warning' | 'critical';
    description: string;
    resolved: boolean;
    resolution?: string;
  }>;

  /** Post-maintenance test results */
  testResults: {
    status: 'pass' | 'fail';
    details: string;
  };

  /** Next maintenance due */
  nextMaintenanceDue: Date;

  /** Certification renewed */
  certificationRenewed?: {
    type: string;
    validUntil: Date;
  };

  /** Notes */
  notes?: string;
}

// ============================================================================
// Events and Callbacks
// ============================================================================

/**
 * Event types
 */
export type TimeMachineEvent =
  | 'status-change'
  | 'alarm'
  | 'warning'
  | 'component-failure'
  | 'mission-start'
  | 'mission-complete'
  | 'displacement-start'
  | 'displacement-complete'
  | 'emergency-shutdown'
  | 'maintenance-due'
  | 'certification-expiring';

/**
 * Event callback
 */
export type EventCallback<T = any> = (event: {
  type: TimeMachineEvent;
  timestamp: Date;
  data: T;
  severity: 'info' | 'warning' | 'critical' | 'emergency';
  message: string;
}) => void;

// ============================================================================
// Export all types
// ============================================================================

export {
  // Re-export for convenience
};
