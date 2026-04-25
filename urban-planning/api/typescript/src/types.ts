/**
 * WIA-CITY-016: Urban Planning Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 timestamp string
 */
export type Timestamp = string;

/**
 * Geographic coordinates
 */
export interface Coordinates {
  latitude: number;
  longitude: number;
  altitude?: number;
}

/**
 * Area measurement in square meters
 */
export type Area = number;

/**
 * Distance in meters
 */
export type Distance = number;

// ============================================================================
// Land Use Types
// ============================================================================

/**
 * Zoning types
 */
export enum ZoningType {
  RESIDENTIAL_LOW = 'residential_low',
  RESIDENTIAL_MEDIUM = 'residential_medium',
  RESIDENTIAL_HIGH = 'residential_high',
  COMMERCIAL_NEIGHBORHOOD = 'commercial_neighborhood',
  COMMERCIAL_CENTRAL = 'commercial_central',
  OFFICE = 'office',
  INDUSTRIAL_LIGHT = 'industrial_light',
  INDUSTRIAL_HEAVY = 'industrial_heavy',
  MIXED_USE = 'mixed_use',
  GREEN_SPACE = 'green_space',
  PARK = 'park',
  INSTITUTIONAL = 'institutional',
  INFRASTRUCTURE = 'infrastructure',
}

/**
 * Land use types
 */
export enum LandUseType {
  RESIDENTIAL = 'residential',
  COMMERCIAL = 'commercial',
  INDUSTRIAL = 'industrial',
  AGRICULTURAL = 'agricultural',
  RECREATIONAL = 'recreational',
  INSTITUTIONAL = 'institutional',
  TRANSPORTATION = 'transportation',
  UTILITIES = 'utilities',
  VACANT = 'vacant',
}

/**
 * Land use parcel
 */
export interface LandUse {
  // Identifiers
  parcelId: string;
  address?: string;
  district?: string;

  // Area and geometry
  area: Area;                        // m²
  perimeter: Distance;               // m
  coordinates: Coordinates;
  polygon?: Array<Coordinates>;      // Boundary polygon

  // Zoning and regulation
  zoning: ZoningType;
  landUseType: LandUseType;
  designatedUse: string[];
  prohibitedUse: string[];

  // Building regulations
  buildingRegulation: BuildingRegulation;

  // Ownership and status
  ownership: {
    type: 'public' | 'private' | 'mixed';
    owner?: string;
  };
  developmentStatus: 'undeveloped' | 'under_development' | 'developed' | 'redevelopment';

  // Metadata
  lastUpdated: Timestamp;
  notes?: string;
}

/**
 * Building regulation parameters
 */
export interface BuildingRegulation {
  // Density controls
  far: number;                       // Floor Area Ratio (용적률)
  bcr: number;                       // Building Coverage Ratio (건폐율)

  // Height controls
  maxHeight: number;                 // meters
  maxFloors: number;

  // Setback requirements
  setback: {
    front: number;                   // meters
    rear: number;
    side: number;
    street?: number;
  };

  // Other controls
  greenSpaceRatio: number;           // Required green space %
  parkingRatio: number;              // Parking spaces per unit area
  openSpaceRatio?: number;           // Public open space %

  // Bonus provisions
  bonusFAR?: {
    publicSpace?: number;
    greenBuilding?: number;
    affordableHousing?: number;
    transitOriented?: number;
    total: number;
  };
}

/**
 * Zoning area
 */
export interface ZoningArea {
  zoneId: string;
  name: string;
  type: ZoningType;

  // Area information
  totalArea: Area;                   // m²
  parcels: string[];                 // Parcel IDs

  // Boundary
  boundary: Array<Coordinates>;

  // Regulations
  regulation: BuildingRegulation;

  // Compatibility
  compatibleWith: ZoningType[];
  bufferRequired?: Map<ZoningType, Distance>;

  // Statistics
  statistics?: {
    population?: number;
    households?: number;
    buildings?: number;
    developmentRate?: number;        // %
  };
}

// ============================================================================
// Green Space and Parks
// ============================================================================

/**
 * Green space types
 */
export enum GreenSpaceType {
  REGIONAL_GREEN = 'regional_green',
  URBAN_PARK = 'urban_park',
  NEIGHBORHOOD_PARK = 'neighborhood_park',
  POCKET_PARK = 'pocket_park',
  LINEAR_PARK = 'linear_park',
  ECOLOGICAL_CORRIDOR = 'ecological_corridor',
  GREEN_BELT = 'green_belt',
  ROOF_GARDEN = 'roof_garden',
}

/**
 * Green space
 */
export interface GreenSpace {
  // Identifiers
  greenSpaceId: string;
  name: string;
  type: GreenSpaceType;

  // Location and area
  location: Coordinates;
  area: Area;                        // m²
  boundary: Array<Coordinates>;

  // Accessibility
  accessibility: {
    walkingDistance: Distance;       // meters from nearest residential area
    publicTransit: boolean;
    parkingSpaces?: number;
  };

  // Facilities
  facilities: {
    playground?: boolean;
    sportsField?: boolean;
    walkingTrails?: boolean;
    restrooms?: boolean;
    lighting?: boolean;
    wifi?: boolean;
  };

  // Ecological value
  ecological: {
    treeCount?: number;
    biodiversityIndex?: number;
    nativeSpecies?: string[];
  };

  // Usage statistics
  usage?: {
    dailyVisitors?: number;
    peakHours?: string[];
    satisfaction?: number;             // 0-10 scale
  };

  // Maintenance
  maintenanceSchedule?: string;
  condition?: 'excellent' | 'good' | 'fair' | 'poor';
}

// ============================================================================
// Infrastructure
// ============================================================================

/**
 * Road types
 */
export enum RoadType {
  HIGHWAY = 'highway',
  ARTERIAL = 'arterial',
  COLLECTOR = 'collector',
  LOCAL = 'local',
  PEDESTRIAN = 'pedestrian',
  BICYCLE = 'bicycle',
}

/**
 * Road network
 */
export interface RoadNetwork {
  roadId: string;
  name: string;
  type: RoadType;

  // Geometry
  length: Distance;                  // meters
  width: Distance;                   // meters
  lanes: number;
  route: Array<Coordinates>;

  // Characteristics
  speedLimit: number;                // km/h
  surface: 'asphalt' | 'concrete' | 'gravel' | 'unpaved';
  condition: 'excellent' | 'good' | 'fair' | 'poor';

  // Capacity
  capacity: {
    vehiclesPerHour: number;
    currentVolume?: number;
    congestionLevel?: number;        // V/C ratio
  };

  // Features
  features: {
    sidewalk: boolean;
    bikelane: boolean;
    streetLighting: boolean;
    drainage: boolean;
  };
}

/**
 * Transit types
 */
export enum TransitType {
  METRO = 'metro',
  LIGHT_RAIL = 'light_rail',
  BRT = 'brt',
  BUS = 'bus',
  TRAM = 'tram',
}

/**
 * Public transit
 */
export interface PublicTransit {
  transitId: string;
  name: string;
  type: TransitType;

  // Route
  route: Array<Coordinates>;
  stations: TransitStation[];

  // Service
  operatingHours: {
    start: string;
    end: string;
  };
  frequency: {
    peak: number;                    // minutes
    offPeak: number;
  };

  // Capacity
  capacity: {
    vehicleCapacity: number;
    dailyRidership?: number;
  };
}

/**
 * Transit station
 */
export interface TransitStation {
  stationId: string;
  name: string;
  location: Coordinates;

  // Catchment area
  walkingRadius: Distance;           // meters (typically 500-800m)

  // Facilities
  facilities: {
    parkAndRide?: number;            // parking spaces
    bikeParking?: number;
    accessibility: boolean;          // wheelchair accessible
    elevator?: boolean;
  };

  // Ridership
  dailyBoardings?: number;
  dailyAlightings?: number;
}

/**
 * Water supply infrastructure
 */
export interface WaterSupply {
  facilityId: string;
  type: 'treatment_plant' | 'reservoir' | 'pump_station' | 'pipe';

  // Capacity
  capacity: {
    daily: number;                   // m³/day
    storage?: number;                // m³
  };

  // Service area
  serviceArea: string[];             // Zone IDs
  population: number;

  // Quality
  waterQuality?: {
    ph: number;
    turbidity: number;
    residualChlorine: number;
  };
}

/**
 * Sewerage infrastructure
 */
export interface Sewerage {
  facilityId: string;
  type: 'treatment_plant' | 'pump_station' | 'sewer_line';

  // Capacity
  capacity: {
    daily: number;                   // m³/day
  };

  // Treatment level
  treatmentLevel?: 'primary' | 'secondary' | 'tertiary' | 'advanced';

  // Effluent quality
  effluentQuality?: {
    bod: number;                     // mg/L
    cod: number;
    ss: number;
    tn: number;
    tp: number;
  };
}

/**
 * Power infrastructure
 */
export interface PowerInfrastructure {
  facilityId: string;
  type: 'substation' | 'distribution_line' | 'solar_farm' | 'wind_farm';

  // Capacity
  capacity: number;                  // kW or MW
  voltage?: number;                  // kV

  // Service area
  serviceArea: string[];             // Zone IDs

  // Renewable energy
  renewableRatio?: number;           // %
}

// ============================================================================
// Population and Demographics
// ============================================================================

/**
 * Population data
 */
export interface PopulationData {
  // Identifiers
  zoneId: string;
  zoneName: string;

  // Population
  population: number;
  households: number;
  averageHouseholdSize: number;

  // Density
  density: {
    population: number;              // persons/km²
    household: number;               // households/ha
    gross: number;                   // including roads and parks
    net: number;                     // residential land only
  };

  // Demographics
  demographics: {
    age_0_14: number;                // %
    age_15_64: number;               // %
    age_65_plus: number;             // %
    maleRatio: number;               // %
  };

  // Socioeconomic
  socioeconomic?: {
    medianIncome?: number;
    unemploymentRate?: number;       // %
    educationLevel?: {
      highSchool: number;            // %
      bachelor: number;
      graduate: number;
    };
  };

  // Projections
  projection?: {
    year: number;
    population: number;
    growth: number;                  // %
  }[];
}

// ============================================================================
// Urban Plan
// ============================================================================

/**
 * Urban development plan
 */
export interface UrbanPlan {
  // Identifiers
  planId: string;
  name: string;
  type: 'master_plan' | 'detailed_plan' | 'district_plan' | 'redevelopment';

  // Area
  planningArea: {
    totalArea: Area;                 // m²
    boundary: Array<Coordinates>;
    districts: string[];             // District IDs
  };

  // Timeline
  timeline: {
    startDate: Timestamp;
    completionDate: Timestamp;
    phases: PlanPhase[];
  };

  // Land use allocation
  landUseAllocation: {
    residential: number;             // %
    commercial: number;
    industrial: number;
    greenSpace: number;
    infrastructure: number;
    other: number;
  };

  // Targets
  targets: {
    population: number;
    households: number;
    jobs: number;
    greenSpacePerCapita: number;     // m²/person
    publicTransitShare: number;      // %
  };

  // Infrastructure plan
  infrastructure: {
    roads: RoadNetwork[];
    transit: PublicTransit[];
    water: WaterSupply[];
    sewerage: Sewerage[];
    power: PowerInfrastructure[];
  };

  // Sustainability goals
  sustainability?: {
    greenBuildingTarget: number;     // %
    renewableEnergyTarget: number;   // %
    wasteRecyclingTarget: number;    // %
    carbonReduction: number;         // % from baseline
  };

  // Status
  status: 'draft' | 'review' | 'approved' | 'implementation' | 'completed';
  approvalDate?: Timestamp;
}

/**
 * Plan phase
 */
export interface PlanPhase {
  phaseNumber: number;
  name: string;
  startDate: Timestamp;
  endDate: Timestamp;

  // Deliverables
  deliverables: {
    residentialUnits?: number;
    commercialArea?: Area;           // m²
    infrastructure?: string[];
    publicFacilities?: string[];
  };

  // Budget
  budget?: {
    total: number;
    spent?: number;
    currency: string;
  };

  status: 'planned' | 'ongoing' | 'completed' | 'delayed';
}

// ============================================================================
// Simulation
// ============================================================================

/**
 * Growth simulation parameters
 */
export interface GrowthSimulationParams {
  // Time period
  startYear: number;
  endYear: number;
  timeStep: number;                  // years

  // Scenario
  scenario: 'bau' | 'compact' | 'sprawl' | 'custom';

  // Growth rates
  populationGrowthRate: number;      // % per year
  economicGrowthRate: number;        // % per year

  // Constraints
  constraints: {
    developableLand?: Area;          // m²
    infrastructure?: string[];
    environmental?: string[];
  };

  // Model parameters
  modelParams?: {
    accessibility_weight: number;
    proximity_weight: number;
    slope_weight: number;
    zoning_weight: number;
  };
}

/**
 * Simulation result
 */
export interface SimulationResult {
  simulationId: string;
  params: GrowthSimulationParams;

  // Results by year
  results: {
    year: number;
    population: number;
    developedArea: Area;             // m²
    landUseDistribution: {
      residential: Area;
      commercial: Area;
      industrial: Area;
      greenSpace: Area;
      infrastructure: Area;
    };

    // Infrastructure demand
    infrastructureDemand: {
      roadLength: Distance;          // km
      waterSupply: number;           // m³/day
      powerCapacity: number;         // MW
      sewerageCapacity: number;      // m³/day
    };

    // Environmental impacts
    environmentalImpact?: {
      greenSpaceRatio: number;       // %
      carbonEmissions: number;       // tons CO2/year
      airQuality: number;            // AQI
    };
  }[];

  // Metadata
  createdAt: Timestamp;
  computeTime: number;               // seconds
}

/**
 * Traffic simulation parameters
 */
export interface TrafficSimulationParams {
  // Network
  roadNetwork: RoadNetwork[];

  // Demand
  tripGeneration: {
    zoneId: string;
    trips: number;                   // trips per day
  }[];

  // Analysis period
  timeOfDay: 'peak' | 'offPeak' | 'allDay';

  // Model
  modelType: 'static' | 'dynamic';
}

/**
 * Traffic simulation result
 */
export interface TrafficSimulationResult {
  simulationId: string;

  // Road performance
  roadPerformance: {
    roadId: string;
    volume: number;                  // vehicles/hour
    capacity: number;
    vcRatio: number;                 // Volume/Capacity
    speed: number;                   // km/h
    levelOfService: 'A' | 'B' | 'C' | 'D' | 'E' | 'F';
  }[];

  // Network performance
  networkPerformance: {
    averageSpeed: number;            // km/h
    totalDelay: number;              // vehicle-hours
    avgCongestion: number;           // V/C ratio
  };

  // Recommendations
  recommendations?: {
    congestionPoints: string[];      // Road IDs
    suggestedImprovements: string[];
  };
}

// ============================================================================
// Analysis
// ============================================================================

/**
 * Zoning compatibility analysis
 */
export interface ZoningCompatibilityAnalysis {
  zone1: ZoningType;
  zone2: ZoningType;

  compatible: boolean;
  compatibilityScore: number;        // 0-100

  issues?: string[];
  recommendations?: {
    bufferWidth?: Distance;          // meters
    transitionalUse?: ZoningType;
    mitigationMeasures?: string[];
  };
}

/**
 * Accessibility analysis
 */
export interface AccessibilityAnalysis {
  zoneId: string;

  // Transit accessibility
  transitAccess: {
    nearestStation: string;
    distance: Distance;              // meters
    walkTime: number;                // minutes
    serviceFrequency: number;        // trips per hour
  };

  // Amenity accessibility
  amenityAccess: {
    schools: { count: number; avgDistance: Distance };
    parks: { count: number; avgDistance: Distance };
    shopping: { count: number; avgDistance: Distance };
    healthcare: { count: number; avgDistance: Distance };
  };

  // Overall score
  accessibilityScore: number;        // 0-100
  walkabilityScore: number;          // 0-100
}

/**
 * Density analysis
 */
export interface DensityAnalysis {
  zoneId: string;

  // Current density
  current: {
    population: number;              // persons/km²
    residential: number;             // units/ha
    far: number;
    bcr: number;
  };

  // Allowed density
  allowed: {
    maxFAR: number;
    maxBCR: number;
    maxHeight: number;               // meters
  };

  // Development potential
  potential: {
    additionalUnits: number;
    additionalPopulation: number;
    additionalFloorArea: Area;       // m²
  };

  // Comparison
  comparison: {
    utilizationRate: number;         // % of allowed density used
    comparedToAverage: number;       // % difference from city average
  };
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Generic API response
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: Timestamp;
    requestId: string;
  };
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  pagination: {
    total: number;
    page: number;
    pageSize: number;
    totalPages: number;
  };
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page?: number;
  pageSize?: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Filter parameters
 */
export interface FilterParams {
  zoning?: ZoningType[];
  landUseType?: LandUseType[];
  minArea?: Area;
  maxArea?: Area;
  district?: string[];
  developmentStatus?: string[];
}

/**
 * Date range filter
 */
export interface DateRangeFilter {
  start: Timestamp;
  end: Timestamp;
}
