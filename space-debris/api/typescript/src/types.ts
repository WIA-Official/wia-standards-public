/**
 * WIA-SPACE-010: Space Debris Management Standard
 * TypeScript Type Definitions
 */

export enum DebrisType {
  PAYLOAD = 'PAYLOAD',
  ROCKET_BODY = 'ROCKET_BODY',
  DEBRIS = 'DEBRIS',
  UNKNOWN = 'UNKNOWN'
}

export enum CollisionRisk {
  LOW = 'LOW',
  MEDIUM = 'MEDIUM',
  HIGH = 'HIGH',
  CRITICAL = 'CRITICAL'
}

export enum ManeuverAction {
  MONITOR = 'MONITOR',
  ANALYZE = 'ANALYZE',
  CONSIDER_MANEUVER = 'CONSIDER_MANEUVER',
  EXECUTE_MANEUVER = 'EXECUTE_MANEUVER'
}

export interface OrbitalElements {
  altitude_km: number;
  inclination_deg: number;
  eccentricity: number;
  period_min: number;
  semi_major_axis_km?: number;
  raan_deg?: number;
  arg_perigee_deg?: number;
}

export interface DebrisObject {
  norad_id: string;
  name: string;
  type: DebrisType;
  mass_kg?: number;
  size_cm: number;
  orbit: OrbitalElements;
  tracking: {
    last_update: string;
    position_accuracy_m: number;
    velocity_mps: number;
  };
  collision_risk: CollisionRisk;
  country_of_origin?: string;
}

export interface ConjunctionDataMessage {
  id: string;
  timestamp: string;
  primary: string;
  secondary: string;
  tca: string; // Time of Closest Approach
  miss_distance_m: number;
  probability: number;
  recommendation: ManeuverAction;
  radial_miss_m?: number;
  in_track_miss_m?: number;
  cross_track_miss_m?: number;
}

export interface ManeuverPlan {
  satellite_id: string;
  planned_time: string;
  delta_v_mps: number;
  direction: 'RADIAL' | 'IN_TRACK' | 'CROSS_TRACK';
  fuel_cost_kg: number;
  collision_avoidance_id?: string;
}

export interface DebrisRemovalMission {
  mission_id: string;
  target_object: string;
  capture_method: 'NET' | 'HARPOON' | 'ROBOTIC_ARM' | 'MAGNETIC';
  deorbit_method: 'DRAG_SAIL' | 'TETHER' | 'DIRECT_REENTRY';
  status: 'PLANNED' | 'ACTIVE' | 'COMPLETED' | 'FAILED';
  launch_date?: string;
  completion_date?: string;
}

export interface DebrisStatistics {
  total_tracked: number;
  by_size: {
    large_10cm_plus: number;
    medium_1_10cm: number;
    small_1mm_1cm: number;
  };
  by_type: Record<DebrisType, number>;
  by_orbit: {
    LEO: number;
    MEO: number;
    GEO: number;
    HEO: number;
  };
  collision_events_last_year: number;
}

export interface APIResponse<T> {
  status: 'success' | 'error';
  data?: T;
  error?: string;
  timestamp: string;
}
