/**
 * WIA-SPACE-009: Space Radiation Protection Standard
 * TypeScript Type Definitions
 *
 * @standard WIA-SPACE-009
 * @version 1.0.0
 * @organization WIA (World Certification Industry Association)
 */

/**
 * Alert levels for radiation warnings
 */
export enum AlertLevel {
  GREEN = 'GREEN',
  YELLOW = 'YELLOW',
  ORANGE = 'ORANGE',
  RED = 'RED'
}

/**
 * Types of radiation sources
 */
export enum RadiationSource {
  GCR = 'GCR',         // Galactic Cosmic Rays
  SAA = 'SAA',         // South Atlantic Anomaly
  SPE = 'SPE',         // Solar Particle Event
  VAN_ALLEN = 'VAN_ALLEN'  // Van Allen Belts
}

/**
 * Mission types for dose calculation
 */
export enum MissionType {
  LEO = 'LEO',
  LUNAR = 'LUNAR',
  MARS = 'MARS',
  DEEP_SPACE = 'DEEP_SPACE'
}

/**
 * Solar activity levels
 */
export enum SolarActivity {
  MINIMUM = 'MINIMUM',
  AVERAGE = 'AVERAGE',
  MAXIMUM = 'MAXIMUM'
}

/**
 * Location in space
 */
export interface Location {
  /** Orbit type (LEO, GEO, Lunar, etc.) */
  orbit: string;
  /** Altitude in kilometers */
  altitude_km: number;
  /** Latitude in degrees */
  latitude: number;
  /** Longitude in degrees */
  longitude: number;
}

/**
 * Radiation measurement data
 */
export interface RadiationData {
  /** Daily dose rate in mSv/day */
  dose_rate_mSv_day: number;
  /** Accumulated dose in mSv */
  accumulated_dose_mSv: number;
  /** Breakdown by source */
  sources: Record<string, number>;
  /** Particle type percentages */
  particle_types: {
    protons: number;
    helium: number;
    HZE: number;  // High-Z and Energy particles
  };
}

/**
 * Alert information
 */
export interface AlertInfo {
  /** Alert level */
  level: AlertLevel;
  /** SPE risk assessment */
  spe_risk: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  /** Whether EVA is permitted */
  eva_permitted: boolean;
  /** Additional warnings */
  warnings?: string[];
}

/**
 * Complete radiation measurement
 */
export interface RadiationMeasurement {
  /** Measurement timestamp */
  timestamp: string;
  /** Location of measurement */
  location: Location;
  /** Radiation data */
  radiation: RadiationData;
  /** Alert status */
  alerts: AlertInfo;
}

/**
 * Solar flare classification
 */
export type FlareClass = 'A' | 'B' | 'C' | 'M' | 'X';

/**
 * SPE source information
 */
export interface SPESource {
  /** Solar flare class and magnitude */
  flare_class: string;
  /** Location on Sun (e.g., N15W45) */
  location: string;
  /** Peak time of flare */
  peak_time: string;
}

/**
 * SPE arrival prediction
 */
export interface SPEPrediction {
  /** Expected arrival time */
  arrival_time: string;
  /** Expected duration in hours */
  duration_hours: number;
  /** Peak particle flux */
  peak_flux: string;
  /** Energy range in MeV */
  energy_range_MeV: string;
}

/**
 * Solar Particle Event alert
 */
export interface SPEAlert {
  /** Unique alert ID */
  id: string;
  /** Alert timestamp */
  timestamp: string;
  /** Alert level */
  level: AlertLevel;
  /** Source information */
  source: SPESource;
  /** Prediction data */
  prediction: SPEPrediction;
  /** Recommended actions */
  recommended_actions: string[];
}

/**
 * Dosimetry reading from personal device
 */
export interface DosimetryReading {
  /** Crew member ID */
  crew_id: string;
  /** Measured dose in mSv */
  dose_mSv: number;
  /** Reading timestamp */
  timestamp: string;
  /** Device identifier */
  device_id: string;
  /** Optional location */
  location?: string;
}

/**
 * Career dose limit information
 */
export interface CareerLimit {
  /** Age of crew member */
  age: number;
  /** Gender */
  gender: 'male' | 'female';
  /** Career dose limit in mSv */
  limit_mSv: number;
  /** Currently accumulated dose */
  accumulated_mSv: number;
  /** Remaining allowable dose */
  remaining_mSv: number;
  /** Percentage of limit used */
  percentage_used: number;
}

/**
 * Mission dose calculation parameters
 */
export interface MissionDoseParams {
  /** Mission type */
  mission_type: MissionType;
  /** Duration in days */
  duration_days: number;
  /** Solar activity level */
  solar_activity: SolarActivity;
  /** Optional shielding factor (0-1) */
  shielding_factor?: number;
}

/**
 * Mission dose calculation result
 */
export interface MissionDoseResult {
  /** Estimated daily dose rate */
  daily_dose_rate_mSv: number;
  /** Total mission dose */
  total_dose_mSv: number;
  /** Confidence interval (95%) */
  confidence_interval: [number, number];
  /** Risk assessment */
  risk_level: 'LOW' | 'MODERATE' | 'HIGH' | 'VERY_HIGH';
  /** Recommendations */
  recommendations: string[];
}

/**
 * Shielding effectiveness data
 */
export interface ShieldingData {
  /** Material type */
  material: string;
  /** Thickness in g/cm² */
  thickness_g_cm2: number;
  /** Dose reduction factor */
  reduction_factor: number;
  /** Effective for particle types */
  effective_against: RadiationSource[];
}

/**
 * Storm shelter specification
 */
export interface StormShelter {
  /** Shelter ID */
  id: string;
  /** Location/module */
  location: string;
  /** Maximum capacity */
  capacity: number;
  /** Shielding thickness */
  shielding: ShieldingData;
  /** Currently occupied */
  occupied: boolean;
  /** Operational status */
  status: 'READY' | 'OCCUPIED' | 'MAINTENANCE' | 'OFFLINE';
}

/**
 * API Response wrapper
 */
export interface APIResponse<T> {
  /** Response status */
  status: 'success' | 'error';
  /** Response data */
  data?: T;
  /** Error message if applicable */
  error?: string;
  /** Timestamp of response */
  timestamp: string;
}

/**
 * Configuration options for radiation monitoring
 */
export interface MonitoringConfig {
  /** Update interval in seconds */
  update_interval_sec: number;
  /** Alert thresholds */
  thresholds: {
    daily_dose_mSv: number;
    spe_flux_pfu: number;
  };
  /** Auto-alert enabled */
  auto_alert: boolean;
  /** Notification endpoints */
  notification_endpoints: string[];
}

/**
 * Historical dose data point
 */
export interface HistoricalDoseData {
  /** Timestamp */
  timestamp: string;
  /** Dose value */
  dose_mSv: number;
  /** Location */
  location: string;
  /** Source breakdown */
  sources?: Record<string, number>;
}

/**
 * Dose statistics over a time period
 */
export interface DoseStatistics {
  /** Time period start */
  period_start: string;
  /** Time period end */
  period_end: string;
  /** Average daily dose */
  avg_daily_dose_mSv: number;
  /** Minimum dose */
  min_dose_mSv: number;
  /** Maximum dose */
  max_dose_mSv: number;
  /** Total accumulated */
  total_dose_mSv: number;
  /** Number of measurements */
  measurement_count: number;
}
