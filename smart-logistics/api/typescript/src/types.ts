/**
 * WIA-AUTO-016: Smart Logistics - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive & Logistics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Logistics Types
// ============================================================================

/**
 * Geographic coordinates
 */
export interface GeoLocation {
  lat: number;
  lng: number;
  altitude?: number;
  accuracy?: number;
}

/**
 * Physical address
 */
export interface Address {
  /** Street address line 1 */
  street1: string;

  /** Street address line 2 (optional) */
  street2?: string;

  /** City name */
  city: string;

  /** State/Province code */
  state: string;

  /** Postal/ZIP code */
  postalCode: string;

  /** Country code (ISO 3166-1 alpha-2) */
  country: string;

  /** Geographic coordinates */
  location?: GeoLocation;
}

/**
 * Time window for delivery or pickup
 */
export interface TimeWindow {
  /** Start time (HH:MM format or ISO 8601) */
  start: string;

  /** End time (HH:MM format or ISO 8601) */
  end: string;

  /** Timezone (e.g., "America/Los_Angeles") */
  timezone?: string;
}

/**
 * Dimensions in 3D space
 */
export interface Dimensions {
  length: number;
  width: number;
  height: number;
  unit: 'cm' | 'in' | 'm' | 'ft';
}

/**
 * Weight measurement
 */
export interface Weight {
  value: number;
  unit: 'kg' | 'lb' | 'g' | 'oz' | 'ton';
}

// ============================================================================
// Shipment Types
// ============================================================================

/**
 * Package information
 */
export interface Package {
  /** Unique package identifier */
  packageId: string;

  /** Barcode or tracking number */
  barcode?: string;

  /** Package dimensions */
  dimensions: Dimensions;

  /** Package weight */
  weight: Weight;

  /** Package contents */
  contents?: PackageItem[];

  /** Declared value */
  value?: {
    amount: number;
    currency: string;
  };

  /** Special handling requirements */
  specialHandling?: (
    | 'fragile'
    | 'this_side_up'
    | 'keep_dry'
    | 'temperature_controlled'
    | 'hazmat'
    | 'perishable'
  )[];

  /** Temperature requirements (for cold chain) */
  temperatureRange?: {
    min: number;
    max: number;
    unit: 'C' | 'F';
  };
}

/**
 * Item inside a package
 */
export interface PackageItem {
  /** Stock Keeping Unit */
  sku: string;

  /** Item description */
  description: string;

  /** Quantity */
  quantity: number;

  /** Unit price */
  unitPrice?: number;

  /** HS code for customs */
  hsCode?: string;
}

/**
 * Shipment origin or destination
 */
export interface ShipmentLocation {
  /** Facility ID (for warehouses) */
  facilityId?: string;

  /** Contact name */
  name: string;

  /** Physical address */
  address: Address;

  /** Contact phone */
  phone?: string;

  /** Contact email */
  email?: string;

  /** Delivery instructions */
  instructions?: string;
}

/**
 * Shipment status
 */
export type ShipmentStatus =
  | 'created'
  | 'label_created'
  | 'picked_up'
  | 'in_transit'
  | 'at_facility'
  | 'out_for_delivery'
  | 'delivered'
  | 'exception'
  | 'returned'
  | 'cancelled';

/**
 * Shipment event
 */
export interface ShipmentEvent {
  /** Event timestamp */
  timestamp: Date | string;

  /** Event status */
  status: ShipmentStatus;

  /** Location where event occurred */
  location?: string;

  /** Facility ID */
  facilityId?: string;

  /** Event description */
  description: string;

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Complete shipment information
 */
export interface Shipment {
  /** Unique shipment identifier */
  shipmentId: string;

  /** Carrier tracking number */
  trackingNumber?: string;

  /** Current status */
  status: ShipmentStatus;

  /** Origin location */
  origin: ShipmentLocation;

  /** Destination location */
  destination: ShipmentLocation;

  /** Packages in shipment */
  packages: Package[];

  /** Service type */
  service: {
    type: 'standard' | 'express' | 'overnight' | 'same_day' | 'economy';
    carrierServiceCode?: string;
    guaranteedDelivery?: boolean;
    insuranceValue?: number;
  };

  /** Timeline information */
  timeline: {
    created: Date | string;
    shipped?: Date | string;
    estimatedDelivery?: Date | string;
    actualDelivery?: Date | string;
  };

  /** Event history */
  events: ShipmentEvent[];

  /** Current location */
  currentLocation?: GeoLocation & {
    timestamp: Date | string;
    facility?: string;
  };

  /** Assigned vehicle */
  assignedVehicle?: string;

  /** Assigned driver */
  assignedDriver?: string;

  /** Proof of delivery */
  proofOfDelivery?: ProofOfDelivery;
}

/**
 * Proof of delivery
 */
export interface ProofOfDelivery {
  /** Recipient name */
  signedBy: string;

  /** Digital signature (base64) */
  signature?: string;

  /** Photo of delivery (base64 or URL) */
  photo?: string;

  /** Delivery timestamp */
  timestamp: Date | string;

  /** GPS location of delivery */
  location?: GeoLocation;

  /** Notes from driver */
  notes?: string;
}

// ============================================================================
// Route Optimization Types
// ============================================================================

/**
 * Vehicle information
 */
export interface Vehicle {
  /** Vehicle identifier */
  vehicleId: string;

  /** Vehicle type */
  type: 'car' | 'van' | 'truck' | 'motorcycle' | 'bicycle' | 'drone';

  /** Capacity in kg */
  capacity: number;

  /** Current load in kg */
  currentLoad?: number;

  /** Fuel type */
  fuelType: 'gasoline' | 'diesel' | 'electric' | 'hybrid' | 'cng';

  /** Fuel efficiency (km per liter or kWh) */
  fuelEfficiency?: number;

  /** Maximum range in km */
  maxRange?: number;

  /** Current fuel/battery level (0-1) */
  fuelLevel?: number;

  /** License plate */
  licensePlate?: string;

  /** Special equipment */
  equipment?: ('lift_gate' | 'refrigeration' | 'hazmat_certified')[];
}

/**
 * Driver information
 */
export interface Driver {
  /** Driver identifier */
  driverId: string;

  /** Driver name */
  name: string;

  /** Contact phone */
  phone: string;

  /** Email address */
  email?: string;

  /** License number */
  licenseNumber?: string;

  /** Hours of service remaining */
  hoursOfService?: number;

  /** Performance metrics */
  metrics?: {
    safetyScore: number;
    efficiencyScore: number;
    customerRating: number;
  };
}

/**
 * Delivery stop
 */
export interface Stop {
  /** Stop sequence number */
  sequence: number;

  /** Stop type */
  type: 'pickup' | 'delivery' | 'service';

  /** Associated shipment ID */
  shipmentId?: string;

  /** Location coordinates */
  location: GeoLocation;

  /** Address */
  address: string;

  /** Time window for stop */
  timeWindow?: TimeWindow;

  /** Service time in minutes */
  serviceTime: number;

  /** Number of packages */
  packages?: number;

  /** Total weight */
  weight?: number;

  /** Stop status */
  status: 'pending' | 'in_progress' | 'completed' | 'failed' | 'skipped';

  /** Arrival time */
  arrival?: Date | string;

  /** Departure time */
  departure?: Date | string;

  /** Estimated arrival time */
  estimatedArrival?: Date | string;

  /** Proof of delivery (for delivery stops) */
  pod?: ProofOfDelivery;

  /** Stop priority (1-10, higher = more urgent) */
  priority?: number;
}

/**
 * Route information
 */
export interface Route {
  /** Route identifier */
  routeId: string;

  /** Assigned vehicle */
  vehicle: Vehicle;

  /** Assigned driver */
  driver: Driver;

  /** Route date */
  date: Date | string;

  /** Route status */
  status: 'planned' | 'in_progress' | 'completed' | 'cancelled';

  /** Depot/starting location */
  depot: {
    facilityId: string;
    location: GeoLocation;
  };

  /** List of stops */
  stops: Stop[];

  /** Route metrics */
  metrics: RouteMetrics;

  /** Optimization details */
  optimization?: OptimizationDetails;
}

/**
 * Route performance metrics
 */
export interface RouteMetrics {
  /** Total distance in km */
  totalDistance: number;

  /** Total time in minutes */
  totalTime: number;

  /** Completed stops */
  completedStops: number;

  /** Remaining stops */
  remainingStops: number;

  /** On-time performance (0-1) */
  onTimePerformance: number;

  /** Fuel consumption */
  fuelConsumption?: number;

  /** CO2 emissions in kg */
  emissions?: number;

  /** Total cost */
  totalCost?: number;
}

/**
 * Route optimization details
 */
export interface OptimizationDetails {
  /** Algorithm used */
  algorithm: 'genetic_algorithm' | 'ant_colony' | 'simulated_annealing' | 'greedy' | 'custom';

  /** Optimization time in seconds */
  optimizationTime: number;

  /** Cost savings compared to original route */
  costSavings?: number;

  /** Emission reduction in kg CO2 */
  emissionReduction?: number;

  /** Time savings in minutes */
  timeSavings?: number;

  /** Number of iterations */
  iterations?: number;
}

/**
 * Route optimization request
 */
export interface RouteOptimizationRequest {
  /** Origin point */
  origin: GeoLocation;

  /** List of destination points */
  destinations: (GeoLocation & {
    timeWindow?: TimeWindow;
    serviceTime?: number;
    priority?: number;
  })[];

  /** Vehicle constraints */
  vehicle: {
    type: Vehicle['type'];
    capacity: number;
    fuelType: Vehicle['fuelType'];
    currentLoad?: number;
  };

  /** Optimization constraints */
  constraints?: {
    maxDistance?: number;
    maxDuration?: number;
    timeWindows?: boolean;
    trafficAware?: boolean;
    avoidTolls?: boolean;
    avoidHighways?: boolean;
  };

  /** Optimization goals */
  optimization?: {
    goal: 'minimize_cost' | 'minimize_time' | 'minimize_distance' | 'minimize_emissions';
    weights?: {
      distance?: number;
      time?: number;
      emissions?: number;
    };
  };
}

/**
 * Route optimization result
 */
export interface RouteOptimizationResult {
  /** Generated route ID */
  routeId: string;

  /** Optimized route */
  optimizedRoute: {
    stops: Stop[];
    totalDistance: number;
    totalTime: number;
    estimatedCost: number;
    estimatedEmissions: number;
  };

  /** Alternative routes */
  alternatives?: RouteOptimizationResult['optimizedRoute'][];

  /** Optimization metadata */
  metadata: OptimizationDetails;
}

// ============================================================================
// Warehouse Types
// ============================================================================

/**
 * Warehouse facility
 */
export interface Warehouse {
  /** Warehouse identifier */
  warehouseId: string;

  /** Warehouse name */
  name: string;

  /** Physical address */
  address: Address;

  /** Warehouse type */
  type: 'distribution_center' | 'fulfillment_center' | 'cross_dock' | 'cold_storage';

  /** Total capacity in cubic meters */
  capacity: number;

  /** Current utilization (0-1) */
  utilization: number;

  /** Automation level (0-1) */
  automationLevel: number;

  /** Operating hours */
  hours?: {
    [day: string]: TimeWindow;
  };

  /** Certifications */
  certifications?: ('iso9001' | 'iso14001' | 'haccp' | 'gmp' | 'fssc22000')[];
}

/**
 * Inventory item
 */
export interface InventoryItem {
  /** Stock Keeping Unit */
  sku: string;

  /** Product description */
  description: string;

  /** Quantity on hand */
  quantity: number;

  /** Storage location */
  location: string;

  /** Item status */
  status: 'available' | 'reserved' | 'quarantine' | 'damaged' | 'expired';

  /** Reorder point */
  reorderPoint: number;

  /** Economic order quantity */
  economicOrderQuantity: number;

  /** Last inventory count date */
  lastCounted: Date | string;

  /** Next reorder date */
  nextReorder?: Date | string;

  /** Unit cost */
  unitCost?: number;

  /** Expiration date (for perishables) */
  expirationDate?: Date | string;

  /** Lot/batch number */
  lotNumber?: string;
}

/**
 * Pick list for warehouse operations
 */
export interface PickList {
  /** Pick list identifier */
  pickListId: string;

  /** Associated order ID */
  orderId: string;

  /** Priority level */
  priority: 'standard' | 'high' | 'urgent';

  /** Pick list status */
  status: 'pending' | 'in_progress' | 'completed' | 'cancelled';

  /** Items to pick */
  picks: PickItem[];

  /** Assigned picker */
  picker?: string;

  /** Estimated completion time */
  estimatedCompletionTime?: Date | string;

  /** Actual completion time */
  actualCompletionTime?: Date | string;
}

/**
 * Individual pick item
 */
export interface PickItem {
  /** Line number */
  lineNumber: number;

  /** SKU to pick */
  sku: string;

  /** Storage location */
  location: string;

  /** Quantity to pick */
  quantity: number;

  /** Quantity actually picked */
  pickedQuantity?: number;

  /** Pick status */
  status?: 'pending' | 'picked' | 'short' | 'damaged';

  /** Pick time */
  pickTime?: Date | string;

  /** Time taken to pick (seconds) */
  pickDuration?: number;
}

/**
 * Warehouse efficiency metrics
 */
export interface WarehouseMetrics {
  /** Warehouse Efficiency Score */
  efficiencyScore: number;

  /** Products processed per hour */
  throughput: number;

  /** Average processing time per unit (seconds) */
  avgProcessingTime: number;

  /** Order fulfillment rate (0-1) */
  fulfillmentRate: number;

  /** Inventory accuracy (0-1) */
  inventoryAccuracy: number;

  /** Space utilization (0-1) */
  spaceUtilization: number;

  /** Operating cost per unit */
  costPerUnit: number;

  /** Quality rate (0-1) */
  qualityRate: number;
}

// ============================================================================
// Tracking Types
// ============================================================================

/**
 * Real-time tracking update
 */
export interface TrackingUpdate {
  /** Shipment or vehicle ID */
  id: string;

  /** Update type */
  type: 'location' | 'status' | 'event' | 'eta';

  /** Timestamp */
  timestamp: Date | string;

  /** Location data */
  location?: GeoLocation & {
    speed?: number;
    heading?: number;
  };

  /** Status change */
  status?: ShipmentStatus;

  /** Event information */
  event?: ShipmentEvent;

  /** Updated ETA */
  eta?: Date | string;
}

/**
 * Vehicle telemetry data
 */
export interface VehicleTelemetry {
  /** Vehicle identifier */
  vehicleId: string;

  /** Timestamp */
  timestamp: Date | string;

  /** Current position */
  position: GeoLocation;

  /** Vehicle status */
  status: {
    speed: number;
    heading: number;
    odometer: number;
    fuelLevel: number;
    engineStatus: 'off' | 'idle' | 'running';
    batteryLevel?: number;
  };

  /** Driver information */
  driver?: {
    id: string;
    hoursOfService: number;
    breakRequired: boolean;
  };

  /** Sensor data */
  sensors?: {
    temperature?: number;
    doorOpen?: boolean;
    acceleration?: { x: number; y: number; z: number };
  };
}

/**
 * ETA calculation parameters
 */
export interface ETAParameters {
  /** Current location */
  currentLocation: GeoLocation;

  /** Destination */
  destination: GeoLocation;

  /** Remaining stops */
  remainingStops?: number;

  /** Current traffic conditions */
  trafficConditions?: 'light' | 'moderate' | 'heavy' | 'severe';

  /** Weather conditions */
  weather?: {
    condition: 'clear' | 'rain' | 'snow' | 'fog';
    severity?: 'light' | 'moderate' | 'heavy';
  };

  /** Average velocity (km/h) */
  averageVelocity?: number;

  /** Historical route data */
  historicalData?: {
    averageTime: number;
    variance: number;
  };
}

/**
 * ETA calculation result
 */
export interface ETAResult {
  /** Estimated time of arrival */
  eta: Date | string;

  /** Remaining time in minutes */
  remainingTime: number;

  /** Remaining distance in km */
  remainingDistance: number;

  /** Confidence interval */
  confidence: {
    min: Date | string;
    max: Date | string;
    probability: number;
  };

  /** Factors affecting ETA */
  factors?: {
    traffic: number;
    weather: number;
    stops: number;
    historical: number;
  };
}

// ============================================================================
// Analytics Types
// ============================================================================

/**
 * Performance analytics
 */
export interface PerformanceAnalytics {
  /** Time period */
  period: {
    start: Date | string;
    end: Date | string;
  };

  /** Delivery metrics */
  delivery: {
    totalDeliveries: number;
    onTimeDeliveries: number;
    onTimeRate: number;
    averageDeliveryTime: number;
    customerSatisfaction?: number;
  };

  /** Route metrics */
  routing: {
    totalRoutes: number;
    totalDistance: number;
    totalTime: number;
    averageStopsPerRoute: number;
    routeEfficiency: number;
  };

  /** Cost metrics */
  costs: {
    totalCost: number;
    costPerDelivery: number;
    costPerKm: number;
    fuelCost: number;
    laborCost: number;
  };

  /** Environmental metrics */
  environmental: {
    totalEmissions: number;
    emissionsPerDelivery: number;
    fuelConsumption: number;
  };

  /** Warehouse metrics */
  warehouse?: WarehouseMetrics;
}

// ============================================================================
// Physical Constants & Configuration
// ============================================================================

/**
 * Logistics constants
 */
export const LOGISTICS_CONSTANTS = {
  /** Average walking speed (m/s) */
  WALKING_SPEED: 1.4,

  /** Average service time per delivery (minutes) */
  AVG_SERVICE_TIME: 3,

  /** Time per floor in multi-story building (seconds) */
  TIME_PER_FLOOR: 15,

  /** Maximum weight per package (kg) */
  MAX_PACKAGE_WEIGHT: 30,

  /** Maximum dimensions per package (cm) */
  MAX_PACKAGE_DIMENSION: 150,

  /** Speed of light (for distance calculations, m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Earth radius (for distance calculations, km) */
  EARTH_RADIUS: 6371,
} as const;

/**
 * Optimization weights
 */
export interface OptimizationWeights {
  distance: number;
  time: number;
  fuel: number;
  emissions: number;
  violations: number;
}

/**
 * Default optimization weights
 */
export const DEFAULT_WEIGHTS: OptimizationWeights = {
  distance: 0.3,
  time: 0.3,
  fuel: 0.2,
  emissions: 0.1,
  violations: 0.1,
};

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
  page: number;
  pageSize: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalItems: number;
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-016 error codes
 */
export enum LogisticsErrorCode {
  INVALID_ADDRESS = 'L001',
  ROUTE_NOT_FOUND = 'L002',
  CAPACITY_EXCEEDED = 'L003',
  TIME_WINDOW_VIOLATION = 'L004',
  OPTIMIZATION_FAILED = 'L005',
  TRACKING_NOT_FOUND = 'L006',
  WAREHOUSE_FULL = 'L007',
  INVENTORY_INSUFFICIENT = 'L008',
  VEHICLE_UNAVAILABLE = 'L009',
  DRIVER_UNAVAILABLE = 'L010',
}

/**
 * Logistics error
 */
export class LogisticsError extends Error {
  constructor(
    public code: LogisticsErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'LogisticsError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  GeoLocation,
  Address,
  TimeWindow,
  Dimensions,
  Weight,

  // Shipment types
  Package,
  PackageItem,
  ShipmentLocation,
  ShipmentEvent,
  Shipment,
  ProofOfDelivery,

  // Route types
  Vehicle,
  Driver,
  Stop,
  Route,
  RouteMetrics,
  OptimizationDetails,
  RouteOptimizationRequest,
  RouteOptimizationResult,

  // Warehouse types
  Warehouse,
  InventoryItem,
  PickList,
  PickItem,
  WarehouseMetrics,

  // Tracking types
  TrackingUpdate,
  VehicleTelemetry,
  ETAParameters,
  ETAResult,

  // Analytics types
  PerformanceAnalytics,
  OptimizationWeights,

  // Utility types
  PaginationParams,
  PaginatedResponse,
};

export {
  LOGISTICS_CONSTANTS,
  DEFAULT_WEIGHTS,
  LogisticsErrorCode,
  LogisticsError,
};

export type { ShipmentStatus };
