/**
 * WIA-AUTO-024: Fleet Management - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geographic Types
// ============================================================================

/**
 * Geographic coordinates (WGS 84)
 */
export interface GeoCoordinates {
  /** Latitude in decimal degrees */
  latitude: number;

  /** Longitude in decimal degrees */
  longitude: number;

  /** Altitude in meters (optional) */
  altitude?: number;

  /** Accuracy in meters (optional) */
  accuracy?: number;
}

/**
 * Location with address information
 */
export interface Location extends GeoCoordinates {
  /** Human-readable address */
  address?: string;

  /** City name */
  city?: string;

  /** State/province code */
  state?: string;

  /** Postal/ZIP code */
  postalCode?: string;

  /** Country code (ISO 3166-1 alpha-2) */
  country?: string;
}

/**
 * Geofence definition
 */
export interface Geofence {
  /** Unique geofence identifier */
  id: string;

  /** Geofence name */
  name: string;

  /** Geofence type */
  type: 'circle' | 'polygon' | 'rectangle';

  /** Coordinates defining the geofence */
  coordinates: GeoCoordinates[];

  /** Radius for circle geofence (meters) */
  radius?: number;

  /** Event triggers */
  triggers?: {
    onEntry?: string[];
    onExit?: string[];
    onDwell?: {
      duration: number;
      actions: string[];
    };
  };
}

// ============================================================================
// Vehicle Types
// ============================================================================

/**
 * Vehicle identification and specifications
 */
export interface Vehicle {
  /** Unique vehicle identifier */
  vehicleId: string;

  /** Vehicle Identification Number */
  vin?: string;

  /** Manufacturer (e.g., "Toyota", "Ford") */
  make: string;

  /** Model name */
  model: string;

  /** Manufacturing year */
  year: number;

  /** Vehicle type */
  type: 'sedan' | 'suv' | 'truck' | 'van' | 'bus' | 'motorcycle' | 'other';

  /** Fuel type */
  fuelType: 'gasoline' | 'diesel' | 'electric' | 'hybrid' | 'cng' | 'lng' | 'hydrogen';

  /** Vehicle capacity */
  capacity: {
    passengers: number;
    cargo: number; // cubic meters or liters
    weight: number; // kg
  };

  /** Technical specifications */
  specifications: {
    engineSize?: number; // liters
    transmission?: 'manual' | 'automatic' | 'cvt';
    fuelTankCapacity?: number; // liters
    batteryCapacity?: number; // kWh (for electric/hybrid)
    odometerReading: number; // km or miles
  };

  /** Current status */
  status: 'active' | 'inactive' | 'maintenance' | 'retired';

  /** Current location */
  location?: GeoCoordinates;

  /** Last location update timestamp */
  lastUpdate?: Date | string;

  /** Assigned driver */
  assignedDriver?: string;

  /** Home base/depot */
  homeBase?: string;

  /** Purchase information */
  purchaseDate?: Date | string;
  purchasePrice?: number;

  /** Current estimated value */
  currentValue?: number;

  /** License plate */
  licensePlate?: string;

  /** Registration expiration */
  registrationExpiry?: Date | string;

  /** Insurance information */
  insurance?: {
    provider: string;
    policyNumber: string;
    expiryDate: Date | string;
  };
}

/**
 * Real-time vehicle telemetry data
 */
export interface VehicleTelemetry {
  /** Vehicle identifier */
  vehicleId: string;

  /** Timestamp of telemetry data */
  timestamp: Date | string;

  /** Location data */
  location: GeoCoordinates;

  /** Heading in degrees (0-360) */
  heading?: number;

  /** Speed in km/h or mph */
  speed: number;

  /** Odometer reading in km or miles */
  odometer?: number;

  /** Engine metrics */
  engine?: {
    rpm?: number;
    temperature?: number;
    load?: number;
    throttlePosition?: number;
  };

  /** Fuel metrics */
  fuel?: {
    level: number; // percentage or liters
    pressure?: number;
    rate?: number; // consumption rate
  };

  /** Battery metrics (for electric/hybrid) */
  battery?: {
    level: number; // percentage
    voltage?: number;
    temperature?: number;
    range?: number; // estimated range in km
  };

  /** Diagnostic information */
  diagnostics?: {
    mil: boolean; // Malfunction Indicator Lamp
    dtcCount: number; // Diagnostic Trouble Code count
    codes?: string[];
  };
}

/**
 * Vehicle health score
 */
export interface VehicleHealth {
  /** Vehicle identifier */
  vehicleId: string;

  /** Overall health score (0-100) */
  overallScore: number;

  /** Component health scores */
  components: {
    engine: number;
    transmission: number;
    brakes: number;
    tires: number;
    battery: number;
    suspension: number;
  };

  /** Active warnings */
  warnings: string[];

  /** Recommended actions */
  recommendations: string[];

  /** Last assessment date */
  assessmentDate: Date | string;
}

// ============================================================================
// Route and Trip Types
// ============================================================================

/**
 * Waypoint or stop in a route
 */
export interface Waypoint {
  /** Location coordinates */
  location: GeoCoordinates;

  /** Waypoint name or description */
  name?: string;

  /** Address */
  address?: string;

  /** Arrival time (actual or estimated) */
  arrivalTime?: Date | string;

  /** Departure time (actual or estimated) */
  departureTime?: Date | string;

  /** Stop duration in minutes */
  duration?: number;

  /** Stop purpose */
  purpose?: 'pickup' | 'delivery' | 'break' | 'refuel' | 'other';

  /** Sequence number in route */
  sequence?: number;
}

/**
 * Optimized route
 */
export interface Route {
  /** Unique route identifier */
  routeId: string;

  /** Route name */
  name?: string;

  /** Vehicle assigned to route */
  vehicleId?: string;

  /** Driver assigned to route */
  driverId?: string;

  /** Waypoints in order */
  waypoints: Waypoint[];

  /** Total distance in km or miles */
  totalDistance: number;

  /** Estimated duration in minutes */
  estimatedDuration: number;

  /** Estimated fuel consumption */
  estimatedFuel?: number;

  /** Route status */
  status: 'planned' | 'in_progress' | 'completed' | 'cancelled';

  /** Optimization parameters */
  optimization?: {
    objective: 'minimize_distance' | 'minimize_time' | 'minimize_cost' | 'balanced';
    constraints: RouteConstraints;
  };

  /** Route geometry (encoded polyline) */
  geometry?: string;
}

/**
 * Route optimization constraints
 */
export interface RouteConstraints {
  /** Maximum total distance */
  maxDistance?: number;

  /** Maximum total time */
  maxTime?: number;

  /** Time windows for deliveries */
  timeWindows?: boolean;

  /** Traffic optimization */
  trafficOptimization?: boolean;

  /** Avoid tolls */
  avoidTolls?: boolean;

  /** Avoid highways */
  avoidHighways?: boolean;

  /** Vehicle capacity constraints */
  capacityConstraints?: boolean;
}

/**
 * Complete trip record
 */
export interface Trip {
  /** Unique trip identifier */
  tripId: string;

  /** Vehicle used for trip */
  vehicleId: string;

  /** Driver */
  driverId?: string;

  /** Start time */
  startTime: Date | string;

  /** End time */
  endTime?: Date | string;

  /** Start location */
  startLocation: Location;

  /** End location */
  endLocation?: Location;

  /** Total distance traveled */
  distance: number;

  /** Total duration in minutes */
  duration?: number;

  /** Fuel consumed */
  fuelConsumed?: number;

  /** Stops during trip */
  stops?: Waypoint[];

  /** Driving events */
  events?: DrivingEvent[];

  /** Trip cost breakdown */
  cost?: {
    fuel: number;
    tolls: number;
    parking: number;
    other?: number;
    total: number;
  };

  /** Trip status */
  status: 'in_progress' | 'completed' | 'cancelled';
}

// ============================================================================
// Driver Types
// ============================================================================

/**
 * Driver information
 */
export interface Driver {
  /** Unique driver identifier */
  driverId: string;

  /** First name */
  firstName: string;

  /** Last name */
  lastName: string;

  /** Email address */
  email?: string;

  /** Phone number */
  phone?: string;

  /** Driver's license information */
  license: {
    number: string;
    class: string;
    expiryDate: Date | string;
    state: string;
  };

  /** Employment status */
  status: 'active' | 'inactive' | 'suspended' | 'terminated';

  /** Hire date */
  hireDate?: Date | string;

  /** Certifications */
  certifications?: string[];

  /** Assigned vehicle */
  assignedVehicle?: string;

  /** Driver safety score */
  safetyScore?: number;

  /** Home location */
  homeLocation?: Location;
}

/**
 * Driving event (harsh braking, speeding, etc.)
 */
export interface DrivingEvent {
  /** Event identifier */
  eventId: string;

  /** Driver identifier */
  driverId: string;

  /** Vehicle identifier */
  vehicleId: string;

  /** Event timestamp */
  timestamp: Date | string;

  /** Event type */
  type: 'harsh_braking' | 'harsh_acceleration' | 'sharp_cornering' |
        'speeding' | 'rapid_lane_change' | 'phone_use' | 'seatbelt_violation' | 'other';

  /** Severity level */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Event location */
  location: GeoCoordinates;

  /** Event metrics */
  metrics?: {
    deceleration?: number;
    acceleration?: number;
    lateralG?: number;
    speedBefore?: number;
    speedAfter?: number;
    duration?: number;
  };

  /** Environmental context */
  context?: {
    weather?: string;
    traffic?: string;
    roadType?: string;
  };
}

/**
 * Driver safety score
 */
export interface DriverSafetyScore {
  /** Driver identifier */
  driverId: string;

  /** Overall safety score (0-100) */
  overallScore: number;

  /** Component scores */
  scores: {
    speedingAvoidance: number;
    smoothBraking: number;
    cornering: number;
    idleTimeManagement: number;
    seatbeltCompliance: number;
  };

  /** Event counts by type */
  eventCounts: Record<string, number>;

  /** Scoring period */
  period: {
    start: Date | string;
    end: Date | string;
  };

  /** Trend */
  trend: 'improving' | 'stable' | 'declining';
}

/**
 * Hours of Service (HOS) log entry
 */
export interface HOSEntry {
  /** Driver identifier */
  driverId: string;

  /** Date of entry */
  date: Date | string;

  /** Duty status changes */
  dutyStatus: Array<{
    status: 'off_duty' | 'sleeper_berth' | 'driving' | 'on_duty_not_driving';
    startTime: string; // HH:MM:SS
    endTime: string;
    duration: number; // minutes
    location?: GeoCoordinates;
  }>;

  /** HOS violations */
  violations: string[];

  /** Remaining driving time (minutes) */
  remainingDrivingTime: number;

  /** Remaining on-duty time (minutes) */
  remainingOnDutyTime: number;

  /** Next required break */
  nextBreakRequired?: Date | string;

  /** Certification status */
  certified: boolean;

  /** Certification timestamp */
  certifiedAt?: Date | string;
}

// ============================================================================
// Maintenance Types
// ============================================================================

/**
 * Maintenance schedule item
 */
export interface MaintenanceSchedule {
  /** Vehicle identifier */
  vehicleId: string;

  /** Maintenance items */
  items: Record<string, {
    interval: number;
    unit: 'km' | 'miles' | 'hours' | 'days' | 'months' | 'years';
    lastService: number | Date | string;
    nextService: number | Date | string;
    status: 'ok' | 'upcoming' | 'overdue';
  }>;
}

/**
 * Maintenance work order
 */
export interface WorkOrder {
  /** Work order identifier */
  workOrderId: string;

  /** Vehicle identifier */
  vehicleId: string;

  /** Work order type */
  type: 'preventive' | 'corrective' | 'inspection' | 'recall';

  /** Priority level */
  priority: 'low' | 'medium' | 'high' | 'critical';

  /** Current status */
  status: 'pending' | 'scheduled' | 'in_progress' | 'completed' | 'cancelled';

  /** Scheduled date */
  scheduledDate?: Date | string;

  /** Estimated duration in minutes */
  estimatedDuration?: number;

  /** Tasks to be performed */
  tasks: Array<{
    taskId: string;
    description: string;
    parts?: Array<{
      partId: string;
      name: string;
      quantity: number;
      cost?: number;
    }>;
    labor?: {
      hours: number;
      rate: number;
    };
  }>;

  /** Assigned technician */
  assignedTechnician?: string;

  /** Estimated cost */
  estimatedCost?: number;

  /** Actual cost */
  actualCost?: number;

  /** Completion date */
  completionDate?: Date | string;

  /** Notes */
  notes?: string;
}

/**
 * Predictive maintenance result
 */
export interface MaintenancePrediction {
  /** Vehicle identifier */
  vehicleId: string;

  /** Component being predicted */
  component: string;

  /** Predicted time to failure (days) */
  timeToFailure: number;

  /** Recommended maintenance date */
  recommendedMaintenanceDate: Date | string;

  /** Priority level */
  priority: 'low' | 'medium' | 'high' | 'critical';

  /** Confidence level (0-1) */
  confidence: number;

  /** Contributing factors */
  factors?: string[];
}

// ============================================================================
// Fuel Management Types
// ============================================================================

/**
 * Fuel transaction
 */
export interface FuelTransaction {
  /** Transaction identifier */
  transactionId: string;

  /** Vehicle identifier */
  vehicleId: string;

  /** Driver identifier */
  driverId?: string;

  /** Transaction timestamp */
  timestamp: Date | string;

  /** Location of fueling */
  location: Location;

  /** Station name/ID */
  station?: string;

  /** Fuel quantity */
  quantity: number;

  /** Unit of measurement */
  unit: 'liters' | 'gallons';

  /** Total cost */
  cost: number;

  /** Price per unit */
  pricePerUnit: number;

  /** Fuel type */
  fuelType: string;

  /** Odometer reading at time of fueling */
  odometer: number;

  /** Payment method */
  paymentMethod: 'fleet_card' | 'cash' | 'credit_card' | 'invoice';

  /** Card/transaction reference */
  reference?: string;
}

/**
 * Fuel efficiency metrics
 */
export interface FuelEfficiency {
  /** Vehicle identifier */
  vehicleId: string;

  /** Time period */
  period: {
    start: Date | string;
    end: Date | string;
  };

  /** Average fuel efficiency */
  averageEfficiency: number;

  /** Unit (km/L or mpg) */
  unit: 'km/L' | 'mpg' | 'L/100km';

  /** Total distance */
  totalDistance: number;

  /** Total fuel consumed */
  totalFuel: number;

  /** Total fuel cost */
  totalCost: number;

  /** Trend */
  trend: 'improving' | 'stable' | 'declining';

  /** Comparison to fleet average */
  vsFleetAverage?: number; // percentage
}

// ============================================================================
// Fleet Analytics Types
// ============================================================================

/**
 * Fleet efficiency score
 */
export interface FleetEfficiencyScore {
  /** Fleet identifier */
  fleetId: string;

  /** Overall efficiency score (0-100) */
  score: number;

  /** Component scores */
  components: {
    routeOptimization: number;
    fuelManagement: number;
    vehicleMaintenance: number;
    driverSafety: number;
  };

  /** Calculation weights */
  weights?: {
    routeOptimization: number;
    fuelManagement: number;
    vehicleMaintenance: number;
    driverSafety: number;
  };

  /** Period */
  period: {
    start: Date | string;
    end: Date | string;
  };
}

/**
 * Key Performance Indicators
 */
export interface FleetKPIs {
  /** Fleet identifier */
  fleetId: string;

  /** Reporting period */
  period: {
    start: Date | string;
    end: Date | string;
  };

  /** Utilization metrics */
  utilization: {
    overall: number; // percentage
    target: number;
    status: 'above_target' | 'on_target' | 'below_target';
  };

  /** Fuel efficiency */
  fuelEfficiency: {
    average: number;
    target: number;
    unit: string;
    status: 'above_target' | 'on_target' | 'below_target';
  };

  /** Maintenance cost ratio */
  maintenanceCostRatio: {
    value: number; // percentage
    target: number;
    status: 'above_target' | 'on_target' | 'below_target';
  };

  /** On-time delivery rate */
  onTimeDelivery: {
    value: number; // percentage
    target: number;
    status: 'above_target' | 'on_target' | 'below_target';
  };

  /** Safety score */
  safetyScore: {
    average: number;
    target: number;
    status: 'above_target' | 'on_target' | 'below_target';
  };

  /** Vehicle downtime */
  vehicleDowntime?: {
    value: number; // percentage
    target: number;
    status: 'above_target' | 'on_target' | 'below_target';
  };
}

/**
 * Cost analytics
 */
export interface FleetCostAnalytics {
  /** Fleet identifier */
  fleetId: string;

  /** Period */
  period: {
    start: Date | string;
    end: Date | string;
  };

  /** Cost breakdown */
  costs: {
    fuel: {
      amount: number;
      percentage: number;
      trend: 'increasing' | 'stable' | 'decreasing';
    };
    maintenance: {
      amount: number;
      percentage: number;
      trend: 'increasing' | 'stable' | 'decreasing';
    };
    insurance: {
      amount: number;
      percentage: number;
      trend: 'increasing' | 'stable' | 'decreasing';
    };
    drivers: {
      amount: number;
      percentage: number;
      trend: 'increasing' | 'stable' | 'decreasing';
    };
    other: {
      amount: number;
      percentage: number;
      trend: 'increasing' | 'stable' | 'decreasing';
    };
    total: number;
  };

  /** Cost per mile/km */
  costPerMile: number;

  /** Cost per vehicle */
  costPerVehicle: number;
}

// ============================================================================
// Compliance Types
// ============================================================================

/**
 * IFTA (International Fuel Tax Agreement) report
 */
export interface IFTAReport {
  /** Reporting period */
  reportingPeriod: string;

  /** Jurisdiction code */
  jurisdiction: string;

  /** Vehicle identifier */
  vehicleId: string;

  /** Fuel data */
  fuelData: {
    totalMiles: number;
    taxableMiles: number;
    fuelPurchased: number;
    averageMpg: number;
    fuelConsumed: number;
    taxRate: number;
    taxOwed: number;
  };

  /** Cross-border miles */
  crossBorderMiles: Record<string, number>;
}

/**
 * Vehicle inspection record
 */
export interface InspectionRecord {
  /** Vehicle identifier */
  vehicleId: string;

  /** Inspection date */
  inspectionDate: Date | string;

  /** Inspector identifier */
  inspector: string;

  /** Inspection type */
  type: 'annual' | 'pre_trip' | 'post_trip' | 'roadside' | 'random';

  /** Inspection items */
  items: Array<{
    category: string;
    items: Array<{
      name: string;
      status: 'pass' | 'fail' | 'warning';
      measurement?: number;
      notes?: string;
    }>;
  }>;

  /** Overall status */
  overallStatus: 'pass' | 'fail' | 'conditional';

  /** Next inspection due */
  nextInspectionDue?: Date | string;

  /** Defects found */
  defects?: string[];
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Route optimization request
 */
export interface RouteOptimizationRequest {
  /** Vehicles to use */
  vehicles: string[];

  /** Destinations to visit */
  destinations: Location[];

  /** Optimization constraints */
  constraints?: RouteConstraints;

  /** Optimization objective */
  objective?: 'minimize_distance' | 'minimize_time' | 'minimize_cost' | 'balanced';

  /** Consider real-time traffic */
  useTraffic?: boolean;

  /** Start time for route */
  startTime?: Date | string;
}

/**
 * Route optimization response
 */
export interface RouteOptimizationResponse {
  /** Optimized routes */
  routes: Route[];

  /** Total distance across all routes */
  totalDistance: number;

  /** Total estimated time */
  totalTime: number;

  /** Estimated total cost */
  estimatedCost?: number;

  /** Optimization metrics */
  metrics: {
    vehiclesUsed: number;
    stopsCompleted: number;
    averageDistance: number;
    averageTime: number;
  };
}

/**
 * Fleet analytics request
 */
export interface FleetAnalyticsRequest {
  /** Fleet identifier */
  fleetId: string;

  /** Time period */
  period: {
    start: Date | string;
    end: Date | string;
  } | 'last-7-days' | 'last-30-days' | 'last-90-days' | 'ytd';

  /** Metrics to include */
  metrics?: Array<'efficiency' | 'fuel' | 'maintenance' | 'safety' | 'costs' | 'utilization'>;
}

/**
 * Fleet analytics response
 */
export interface FleetAnalyticsResponse {
  /** Fleet identifier */
  fleetId: string;

  /** Period */
  period: {
    start: Date | string;
    end: Date | string;
  };

  /** Efficiency score */
  efficiencyScore?: FleetEfficiencyScore;

  /** KPIs */
  kpis?: FleetKPIs;

  /** Cost analytics */
  costs?: FleetCostAnalytics;

  /** Summary statistics */
  summary: {
    totalVehicles: number;
    activeVehicles: number;
    totalDistance: number;
    totalFuelConsumed: number;
    avgFuelEconomy: number;
    totalTrips: number;
    maintenanceAlerts: number;
    safetyEvents: number;
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-024 error codes
 */
export enum FleetErrorCode {
  VEHICLE_NOT_FOUND = 'FM001',
  DRIVER_NOT_FOUND = 'FM002',
  ROUTE_OPTIMIZATION_FAILED = 'FM003',
  INVALID_COORDINATES = 'FM004',
  GEOFENCE_VIOLATION = 'FM005',
  HOS_VIOLATION = 'FM006',
  MAINTENANCE_OVERDUE = 'FM007',
  INVALID_FUEL_DATA = 'FM008',
  TELEMETRY_ERROR = 'FM009',
  UNAUTHORIZED_ACCESS = 'FM010',
}

/**
 * Fleet management error
 */
export class FleetManagementError extends Error {
  constructor(
    public code: FleetErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'FleetManagementError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Pagination parameters
 */
export interface PaginationParams {
  /** Page number (1-indexed) */
  page?: number;

  /** Items per page */
  pageSize?: number;

  /** Sort field */
  sortBy?: string;

  /** Sort order */
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Items in current page */
  items: T[];

  /** Pagination metadata */
  pagination: {
    currentPage: number;
    pageSize: number;
    totalItems: number;
    totalPages: number;
    hasNext: boolean;
    hasPrevious: boolean;
  };
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Fleet management constants
 */
export const FLEET_CONSTANTS = {
  /** Default GPS update interval (seconds) */
  DEFAULT_GPS_INTERVAL: 30,

  /** Maximum allowed speed (km/h) */
  MAX_SPEED_THRESHOLD: 120,

  /** Harsh braking threshold (m/s²) */
  HARSH_BRAKING_THRESHOLD: -8.0,

  /** Harsh acceleration threshold (m/s²) */
  HARSH_ACCELERATION_THRESHOLD: 7.0,

  /** Minimum vehicle health score */
  MIN_HEALTH_SCORE: 70,

  /** Default fuel efficiency unit */
  DEFAULT_FUEL_UNIT: 'km/L',

  /** Hours of service daily driving limit (minutes) */
  HOS_DAILY_DRIVING_LIMIT: 11 * 60,

  /** Hours of service daily on-duty limit (minutes) */
  HOS_DAILY_ON_DUTY_LIMIT: 14 * 60,

  /** Required rest break duration (minutes) */
  HOS_REST_BREAK_DURATION: 30,
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Geographic
  GeoCoordinates,
  Location,
  Geofence,

  // Vehicle
  Vehicle,
  VehicleTelemetry,
  VehicleHealth,

  // Route and Trip
  Waypoint,
  Route,
  RouteConstraints,
  Trip,

  // Driver
  Driver,
  DrivingEvent,
  DriverSafetyScore,
  HOSEntry,

  // Maintenance
  MaintenanceSchedule,
  WorkOrder,
  MaintenancePrediction,

  // Fuel
  FuelTransaction,
  FuelEfficiency,

  // Analytics
  FleetEfficiencyScore,
  FleetKPIs,
  FleetCostAnalytics,

  // Compliance
  IFTAReport,
  InspectionRecord,

  // API
  RouteOptimizationRequest,
  RouteOptimizationResponse,
  FleetAnalyticsRequest,
  FleetAnalyticsResponse,

  // Utility
  PaginationParams,
  PaginatedResponse,
};

export { FLEET_CONSTANTS, FleetErrorCode, FleetManagementError };
