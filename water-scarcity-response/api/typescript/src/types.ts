/**
 * WIA Water Scarcity Response Types
 * Comprehensive type definitions for drought management and water emergency response
 *
 * @module @wia/water-scarcity-response
 * @version 1.0.0
 */

/**
 * Drought severity levels based on standardized indices
 */
export enum DroughtLevel {
  NORMAL = 'normal',
  ABNORMALLY_DRY = 'abnormally_dry',
  MODERATE_DROUGHT = 'moderate_drought',
  SEVERE_DROUGHT = 'severe_drought',
  EXTREME_DROUGHT = 'extreme_drought',
  EXCEPTIONAL_DROUGHT = 'exceptional_drought'
}

/**
 * Water restriction phases
 */
export enum RestrictionPhase {
  NONE = 'none',
  VOLUNTARY = 'voluntary',
  PHASE_1 = 'phase_1',
  PHASE_2 = 'phase_2',
  PHASE_3 = 'phase_3',
  EMERGENCY = 'emergency'
}

/**
 * Alternative water supply types
 */
export enum AlternativeSupplyType {
  GROUNDWATER = 'groundwater',
  DESALINATION = 'desalination',
  RECYCLED_WATER = 'recycled_water',
  RAINWATER_HARVESTING = 'rainwater_harvesting',
  WATER_IMPORT = 'water_import',
  EMERGENCY_RESERVES = 'emergency_reserves'
}

/**
 * Drought monitoring metrics
 */
export interface DroughtMetrics {
  /** Current drought level */
  droughtLevel: DroughtLevel;
  /** Standardized Precipitation Index (SPI) */
  spi: number;
  /** Palmer Drought Severity Index (PDSI) */
  pdsi: number;
  /** Soil moisture percentage */
  soilMoisture: number;
  /** Reservoir storage percentage */
  reservoirStorage: number;
  /** Groundwater level in meters */
  groundwaterLevel: number;
  /** Precipitation deficit in mm */
  precipitationDeficit: number;
  /** Temperature anomaly in °C */
  temperatureAnomaly: number;
  /** Affected area in km² */
  affectedArea: number;
  /** Affected population count */
  affectedPopulation: number;
  /** Measurement timestamp */
  timestamp: Date;
}

/**
 * Water restriction measures
 */
export interface WaterRestriction {
  /** Restriction ID */
  id: string;
  /** Current restriction phase */
  phase: RestrictionPhase;
  /** Target water reduction percentage */
  reductionTarget: number;
  /** Allowed outdoor watering days */
  allowedWateringDays: number[];
  /** Watering hours restrictions */
  wateringHours?: {
    start: string;
    end: string;
  };
  /** Car washing restrictions */
  carWashingAllowed: boolean;
  /** Pool filling restrictions */
  poolFillingAllowed: boolean;
  /** Lawn watering restrictions */
  lawnWateringAllowed: boolean;
  /** Commercial restrictions */
  commercialRestrictions: string[];
  /** Agricultural restrictions */
  agriculturalRestrictions: string[];
  /** Penalties for violations */
  penalties: {
    firstViolation: number;
    secondViolation: number;
    thirdViolation: number;
  };
  /** Effective date */
  effectiveDate: Date;
  /** Expiration date (if applicable) */
  expirationDate?: Date;
}

/**
 * Emergency response plan
 */
export interface EmergencyResponse {
  /** Response ID */
  id: string;
  /** Emergency severity level */
  severity: 'low' | 'medium' | 'high' | 'critical';
  /** Response status */
  status: 'planned' | 'activated' | 'in_progress' | 'completed' | 'cancelled';
  /** Target regions */
  targetRegions: string[];
  /** Emergency actions */
  actions: EmergencyAction[];
  /** Alternative supplies activated */
  alternativeSupplies: AlternativeSupply[];
  /** Resource allocations */
  resourceAllocations: ResourceAllocation[];
  /** Communication plan */
  communicationPlan: CommunicationPlan;
  /** Activation timestamp */
  activatedAt?: Date;
  /** Estimated duration in days */
  estimatedDuration: number;
}

/**
 * Emergency action definition
 */
export interface EmergencyAction {
  /** Action ID */
  id: string;
  /** Action type */
  type: 'supply_activation' | 'rationing' | 'transport' | 'conservation' | 'infrastructure';
  /** Action description */
  description: string;
  /** Priority level (1-5) */
  priority: number;
  /** Responsible agency */
  responsibleAgency: string;
  /** Status */
  status: 'pending' | 'in_progress' | 'completed' | 'failed';
  /** Start date */
  startDate?: Date;
  /** Completion date */
  completionDate?: Date;
  /** Resources required */
  resourcesRequired: {
    personnel: number;
    equipment: string[];
    budget: number;
  };
}

/**
 * Alternative water supply source
 */
export interface AlternativeSupply {
  /** Supply ID */
  id: string;
  /** Supply type */
  type: AlternativeSupplyType;
  /** Supply capacity in liters/day */
  capacity: number;
  /** Current production in liters/day */
  currentProduction: number;
  /** Location coordinates */
  location: {
    latitude: number;
    longitude: number;
  };
  /** Operational status */
  status: 'operational' | 'standby' | 'maintenance' | 'offline';
  /** Activation time in hours */
  activationTime: number;
  /** Cost per liter */
  costPerLiter: number;
  /** Water quality parameters */
  waterQuality: {
    ph: number;
    tds: number;
    turbidity: number;
    certified: boolean;
  };
}

/**
 * Resource allocation for emergency response
 */
export interface ResourceAllocation {
  /** Resource type */
  resourceType: 'water' | 'personnel' | 'equipment' | 'funding' | 'infrastructure';
  /** Amount allocated */
  amount: number;
  /** Unit of measurement */
  unit: string;
  /** Recipient region/agency */
  recipient: string;
  /** Allocation date */
  allocationDate: Date;
  /** Priority level */
  priority: number;
}

/**
 * Water rationing plan
 */
export interface RationingPlan {
  /** Rationing ID */
  id: string;
  /** Rationing type */
  type: 'per_capita' | 'rotating_schedule' | 'sector_based' | 'priority_based';
  /** Per capita allocation in liters/day */
  perCapitaAllocation: number;
  /** Rotation schedule if applicable */
  rotationSchedule?: {
    [zone: string]: number[];
  };
  /** Sector priorities */
  sectorPriorities: {
    sector: string;
    priority: number;
    allocationPercentage: number;
  }[];
  /** Essential services exemptions */
  exemptions: string[];
  /** Enforcement measures */
  enforcementMeasures: string[];
  /** Effective date */
  effectiveDate: Date;
}

/**
 * Early warning system
 */
export interface EarlyWarning {
  /** Warning ID */
  id: string;
  /** Warning level */
  level: 'watch' | 'advisory' | 'warning' | 'emergency';
  /** Drought forecast */
  forecast: {
    droughtLevel: DroughtLevel;
    confidence: number;
    timeframe: number; // days
  };
  /** Affected regions */
  affectedRegions: string[];
  /** Recommended actions */
  recommendedActions: string[];
  /** Issue date */
  issueDate: Date;
  /** Valid until */
  validUntil: Date;
  /** Alert channels */
  alertChannels: ('sms' | 'email' | 'radio' | 'tv' | 'app' | 'siren')[];
}

/**
 * Communication plan for emergency response
 */
export interface CommunicationPlan {
  /** Target audiences */
  targetAudiences: {
    audience: string;
    channels: string[];
    frequency: string;
  }[];
  /** Key messages */
  keyMessages: string[];
  /** Spokesperson information */
  spokespersons: {
    name: string;
    role: string;
    contact: string;
  }[];
  /** Update schedule */
  updateSchedule: string;
}

/**
 * Recovery plan
 */
export interface RecoveryPlan {
  /** Recovery ID */
  id: string;
  /** Recovery phases */
  phases: RecoveryPhase[];
  /** Success criteria */
  successCriteria: {
    metric: string;
    targetValue: number;
    currentValue: number;
  }[];
  /** Estimated recovery time in days */
  estimatedRecoveryTime: number;
  /** Post-drought assessment */
  postDroughtAssessment?: {
    economicImpact: number;
    environmentalImpact: string;
    socialImpact: string;
    lessonsLearned: string[];
  };
}

/**
 * Recovery phase definition
 */
export interface RecoveryPhase {
  /** Phase name */
  name: string;
  /** Phase description */
  description: string;
  /** Phase order */
  order: number;
  /** Actions in this phase */
  actions: string[];
  /** Duration in days */
  duration: number;
  /** Status */
  status: 'pending' | 'in_progress' | 'completed';
}

/**
 * Event types for water scarcity response
 */
export type WaterScarcityEvent =
  | DroughtLevelChangeEvent
  | RestrictionPhaseChangeEvent
  | EmergencyActivatedEvent
  | SupplyActivatedEvent
  | RationingImplementedEvent
  | EarlyWarningIssuedEvent;

/**
 * Drought level change event
 */
export interface DroughtLevelChangeEvent {
  type: 'drought_level_change';
  previousLevel: DroughtLevel;
  newLevel: DroughtLevel;
  metrics: DroughtMetrics;
  timestamp: Date;
}

/**
 * Restriction phase change event
 */
export interface RestrictionPhaseChangeEvent {
  type: 'restriction_phase_change';
  previousPhase: RestrictionPhase;
  newPhase: RestrictionPhase;
  restriction: WaterRestriction;
  timestamp: Date;
}

/**
 * Emergency activated event
 */
export interface EmergencyActivatedEvent {
  type: 'emergency_activated';
  response: EmergencyResponse;
  timestamp: Date;
}

/**
 * Alternative supply activated event
 */
export interface SupplyActivatedEvent {
  type: 'supply_activated';
  supply: AlternativeSupply;
  timestamp: Date;
}

/**
 * Rationing implemented event
 */
export interface RationingImplementedEvent {
  type: 'rationing_implemented';
  plan: RationingPlan;
  timestamp: Date;
}

/**
 * Early warning issued event
 */
export interface EarlyWarningIssuedEvent {
  type: 'early_warning_issued';
  warning: EarlyWarning;
  timestamp: Date;
}
