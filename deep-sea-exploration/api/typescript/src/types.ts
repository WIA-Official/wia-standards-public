/**
 * WIA-ENE-040: Deep Sea Exploration Standard - TypeScript Type Definitions
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
  depth?: number;
}

/**
 * GeoJSON LineString for tracks
 */
export type GeoJSONLineString = {
  type: 'LineString';
  coordinates: number[][];
};

// ============================================================================
// Depth Zones
// ============================================================================

/**
 * Ocean depth zones
 */
export enum DepthZone {
  EPIPELAGIC = 'epipelagic',           // 0-200m (광합성 가능)
  MESOPELAGIC = 'mesopelagic',         // 200-1,000m (박명대)
  BATHYPELAGIC = 'bathypelagic',       // 1,000-4,000m (심해대)
  ABYSSOPELAGIC = 'abyssopelagic',     // 4,000-6,000m (초심해대)
  HADOPELAGIC = 'hadopelagic',         // 6,000-11,000m (해덕대)
}

/**
 * Seafloor habitat types
 */
export enum HabitatType {
  ABYSSAL_PLAIN = 'abyssal_plain',               // 심해 평원
  SEAMOUNT = 'seamount',                         // 해저산
  TRENCH = 'trench',                             // 해구
  MID_OCEAN_RIDGE = 'mid_ocean_ridge',           // 중앙 해령
  CONTINENTAL_SLOPE = 'continental_slope',       // 대륙 사면
  HYDROTHERMAL_VENT = 'hydrothermal_vent',       // 열수 분출공
  COLD_SEEP = 'cold_seep',                       // 냉수 용출대
  WHALE_FALL = 'whale_fall',                     // 고래 사체
  CANYON = 'canyon',                             // 해저 협곡
  VOLCANIC = 'volcanic',                         // 화산 지대
}

// ============================================================================
// Submersible Types
// ============================================================================

/**
 * Submersible vehicle types
 */
export enum SubmersibleType {
  HOV = 'hov',           // Human-Occupied Vehicle (유인 잠수정)
  ROV = 'rov',           // Remotely Operated Vehicle (원격 조종 무인 잠수정)
  AUV = 'auv',           // Autonomous Underwater Vehicle (자율 무인 잠수정)
  LANDER = 'lander',     // Free-fall Lander (자유낙하 착륙선)
  CRAWLER = 'crawler',   // Benthic Crawler (해저면 이동 로봇)
}

/**
 * HOV (Human-Occupied Vehicle) specification
 */
export interface HOVSpecification {
  vehicleId: string;
  name: string;
  type: SubmersibleType.HOV;

  // Technical specs
  maxDepth: number;                      // meters
  crew: number;                          // number of people
  endurance: number;                     // hours
  speed: number;                         // knots

  // Hull
  hull: {
    material: 'titanium' | 'steel' | 'aluminum';
    shape: 'sphere' | 'cylinder';
    viewportDiameter: number;            // cm
    pressureRating: number;              // atm
  };

  // Systems
  propulsion: {
    thrusters: number;
    batteryCapacity: number;             // kWh
    batteryType: string;
  };

  lifeSupport: {
    oxygenDuration: number;              // hours (normal)
    emergencyDuration: number;           // hours
    co2Scrubber: string;
  };

  // Payload
  payload: {
    manipulatorArms: number;
    cameras: string[];
    sensors: string[];
    sampleCapacity: number;              // liters
  };

  // Safety
  safety: {
    ballastType: string;
    buoyancyMaterial: string;
    emergencyBeacon: boolean;
    acousticCommunication: boolean;
  };

  // Operational
  operator: string;
  certificationDate: Timestamp;
  lastMaintenance: Timestamp;
}

/**
 * ROV (Remotely Operated Vehicle) specification
 */
export interface ROVSpecification {
  vehicleId: string;
  name: string;
  type: SubmersibleType.ROV;

  // Class
  rovClass: 'work_class' | 'observation' | 'light_work';

  // Technical specs
  maxDepth: number;                      // meters
  dimensions: {
    length: number;                      // meters
    width: number;                       // meters
    height: number;                      // meters
    weight: number;                      // kg
  };

  // Tether
  tether: {
    length: number;                      // meters
    type: 'fiber_optic' | 'coaxial' | 'hybrid';
    bandwidth: number;                   // Gbps
  };

  // Propulsion
  thrusters: {
    horizontal: number;
    vertical: number;
    maxSpeed: number;                    // knots
  };

  // Manipulators
  manipulators: {
    count: number;
    functions: number;                   // degrees of freedom
    maxReach: number;                    // meters
    graspForce: number;                  // kg
  };

  // Sensors (detailed in separate interface)
  sensors: ROVSensorSuite;

  // Sampling
  sampling: {
    biobox: boolean;
    pushCores: number;
    niskinBottles: number;
    suctionSampler: boolean;
  };
}

/**
 * ROV sensor suite
 */
export interface ROVSensorSuite {
  navigation: {
    dvl: boolean;                        // Doppler Velocity Log
    usbl: boolean;                       // Ultra-Short Baseline
    ins: boolean;                        // Inertial Navigation System
    compass: boolean;
    altimeter: boolean;
  };

  imaging: {
    mainCamera: {
      resolution: string;                // e.g., "4K", "HD"
      frameRate: number;                 // fps
      zoom: boolean;
    };
    stillCamera?: {
      megapixels: number;
    };
    laserScaler: boolean;                // Parallel lasers for scale
  };

  environmental: {
    ctd: boolean;                        // Conductivity-Temperature-Depth
    dissolvedOxygen: boolean;
    ph: boolean;
    orp: boolean;                        // Oxidation-Reduction Potential
    turbidity: boolean;
  };

  sonar: {
    forwardLooking?: {
      frequency: number;                 // kHz
      range: number;                     // meters
    };
    imaging?: {
      frequency: number;                 // kHz
    };
    profiler?: boolean;                  // Sub-bottom profiler
  };

  lighting: {
    ledLights: number;
    totalLumens: number;
  };
}

/**
 * AUV (Autonomous Underwater Vehicle) specification
 */
export interface AUVSpecification {
  vehicleId: string;
  name: string;
  type: SubmersibleType.AUV;

  maxDepth: number;                      // meters
  endurance: number;                     // hours
  range: number;                         // km
  speed: number;                         // knots

  navigation: {
    gps: boolean;                        // Surface only
    ins: boolean;
    dvl: boolean;
    usblBeacons: boolean;
  };

  sensors: {
    multibeamSonar: {
      frequency: number;                 // kHz
      swathWidth: number;                // meters
    };
    sideScanSonar?: {
      frequency: number;                 // kHz
      range: number;                     // meters
    };
    subBottomProfiler?: boolean;
    camera?: boolean;
    ctd: boolean;
    magnetometer?: boolean;
  };

  communication: {
    acoustic: boolean;
    satellite: boolean;                  // When surfaced
  };

  missionPlanning: {
    autonomyLevel: 'full' | 'supervised' | 'remote';
    surveyPattern: 'lawn_mower' | 'adaptive' | 'waypoint';
  };
}

/**
 * Lander specification
 */
export interface LanderSpecification {
  landerId: string;
  name: string;
  type: SubmersibleType.LANDER;

  maxDepth: number;                      // meters

  structure: {
    frameMaterial: 'aluminum' | 'titanium';
    buoyancyType: 'syntactic_foam' | 'glass_spheres';
    ballastWeight: number;               // kg
  };

  deployment: {
    descentRate: number;                 // m/s
    freefall: boolean;
  };

  recovery: {
    acousticRelease: boolean;
    ascentRate: number;                  // m/s
    beaconType: string;
  };

  instruments: {
    camera?: {
      type: 'time_lapse' | 'video';
      resolution: string;
      duration: number;                  // hours
    };
    sedimentTrap?: boolean;
    waterSampler?: {
      niskinBottles: number;
      volume: number;                    // liters each
    };
    sensors: {
      ctd: boolean;
      currentMeter?: boolean;
      seismometer?: boolean;
    };
  };

  power: {
    batteryType: string;
    capacity: number;                    // Wh
    duration: number;                    // days
  };
}

// ============================================================================
// Expedition Management
// ============================================================================

/**
 * Research expedition
 */
export interface Expedition {
  expeditionId: string;

  // Basic info
  name: string;
  chiefScientist: string;
  vessel: {
    name: string;
    operator: string;
    imo?: string;                        // IMO number
    length: number;                      // meters
    classification: 'research_vessel' | 'commercial';
  };

  // Location
  location: {
    region: string;
    area: string;
    coordinates: Coordinates;
    eez?: string;                        // Exclusive Economic Zone
  };

  // Duration
  duration: {
    startDate: Timestamp;
    endDate: Timestamp;
    daysAtSea: number;
  };

  // Objectives
  objectives: string[];
  targetDepthRange: {
    min: number;
    max: number;
  };

  // Equipment
  submersibles: string[];                // List of submersible IDs
  sensors: string[];

  // Permits
  permits: {
    type: string;
    issuedBy: string;
    permitNumber: string;
    validUntil: Timestamp;
  }[];

  // Team
  scientists: {
    name: string;
    affiliation: string;
    role: string;
  }[];

  // Metadata
  metadata: {
    createdAt: Timestamp;
    updatedAt: Timestamp;
    status: 'planning' | 'active' | 'completed' | 'cancelled';
  };
}

// ============================================================================
// Dive Operations
// ============================================================================

/**
 * Dive log entry
 */
export interface DiveLog {
  diveId: string;
  expeditionId: string;

  // Submersible
  submersible: {
    vehicleId: string;
    type: SubmersibleType;
    crew?: {
      name: string;
      role: 'pilot' | 'scientist' | 'observer';
    }[];
  };

  // Timeline
  timeline: {
    launch: Timestamp;
    onBottom: Timestamp;
    offBottom: Timestamp;
    recovery: Timestamp;
    bottomTime: number;                  // seconds
  };

  // Navigation
  navigation: {
    launchPosition: Coordinates;
    bottomTrack: GeoJSONLineString | string; // GeoJSON or file path
    maxDepth: number;
    averageDepth: number;
    areaExplored: number;                // km²
  };

  // Observations
  observations: {
    organisms: number;
    newSpecies: number;
    samples: number;
    rocks: number;
    videos: number;
    photos: number;
  };

  // Environmental
  environmental: {
    bottomTemperature: number;           // °C
    bottomSalinity: number;              // PSU
    bottomOxygen: number;                // mg/L
    bottomPH?: number;
  };

  // Notes
  notes: string;

  // Safety
  incidents: {
    type: string;
    severity: 'minor' | 'moderate' | 'major';
    description: string;
    resolved: boolean;
  }[];
}

// ============================================================================
// Biological Observations
// ============================================================================

/**
 * Organism observation
 */
export interface OrganismObservation {
  observationId: string;
  diveId: string;
  timestamp: Timestamp;

  // Location
  location: {
    latitude: number;
    longitude: number;
    depth: number;
    depthZone: DepthZone;
    habitat: HabitatType;
  };

  // Taxonomy
  organism: {
    taxonId?: string;                    // WoRMS AphiaID or similar
    scientificName?: string;
    commonName?: string;
    phylum?: string;
    class?: string;
    order?: string;
    family?: string;
    genus?: string;
    species?: string;
    identificationLevel: 'phylum' | 'class' | 'order' | 'family' | 'genus' | 'species' | 'subspecies';
    identifiedBy: string;
    identificationMethod: 'visual' | 'genetic' | 'morphological';
  };

  // Morphology
  morphology: {
    length?: number;                     // mm
    width?: number;                      // mm
    height?: number;                     // mm
    weight?: number;                     // g
    color: string;
    notes: string;
  };

  // Behavior
  behavior: 'feeding' | 'resting' | 'swimming' | 'scavenging' | 'burrowing' | 'attached' | 'unknown';

  // Abundance
  abundance: string;                     // e.g., "1 individual", "5-10 individuals", "dense aggregation"

  // Associated species
  associatedSpecies: string[];

  // Media
  media: {
    type: 'video' | 'photo';
    file: string;
    timecode?: string;
    resolution?: string;
  }[];

  // Metadata
  quality: 'excellent' | 'good' | 'fair' | 'poor';
  verified: boolean;
}

/**
 * Biological sample
 */
export interface BiologicalSample {
  sampleId: string;
  expeditionId: string;
  diveId: string;
  timestamp: Timestamp;

  // Location
  location: {
    latitude: number;
    longitude: number;
    depth: number;
    habitat: HabitatType;
  };

  // Sample type
  sampleType: 'whole_organism' | 'tissue' | 'genetic' | 'environmental_dna';

  // Organism
  organism?: {
    scientificName?: string;
    taxonId?: string;
    lifestage: 'adult' | 'juvenile' | 'larva' | 'egg' | 'unknown';
  };

  // Collection method
  method: 'slurp_gun' | 'scoop_net' | 'manipulator' | 'box_core' | 'niskin_bottle';

  // Preservation
  preservation: {
    type: 'frozen' | 'ethanol' | 'formalin' | 'rnalater' | 'live';
    temperature?: number;                // °C
    concentration?: number;              // % for ethanol/formalin
    pressurePreserved?: boolean;
  };

  // Storage
  storage: {
    facility: string;
    repository: string;
    catalogNumber: string;
    freezer?: string;
    box?: string;
    position?: string;
  };

  // Analyses
  analyses: string[];                    // e.g., ["DNA barcoding", "Transcriptomics", "Morphology"]

  // Availability
  curator: string;
  availability: 'available' | 'restricted' | 'depleted';

  // Permits
  nagoyadCompliant: boolean;             // Nagoya Protocol
  materialTransferAgreement?: string;
}

// ============================================================================
// Geological & Geochemical Data
// ============================================================================

/**
 * Hydrothermal vent
 */
export interface HydrothermalVent {
  ventId: string;
  ventField: string;
  discoveryDate: Timestamp;

  // Location
  location: {
    latitude: number;
    longitude: number;
    depth: number;
    region: string;
    tectonicSetting: 'mid_ocean_ridge' | 'back_arc_basin' | 'volcanic_arc' | 'intraplate';
  };

  // Type
  ventType: 'black_smoker' | 'white_smoker' | 'diffuse_flow' | 'lost_city_type';

  // Structure
  structure: {
    height: number;                      // meters
    diameter: number;                    // meters
    material: string;                    // e.g., "Sulfide (FeS, CuS, ZnS)"
    chimneyType?: 'active' | 'inactive' | 'collapsed';
  };

  // Fluid chemistry
  fluidChemistry: {
    temperature: number;                 // °C
    ph: number;
    h2s: number;                         // mmol/kg
    ch4: number;                         // mmol/kg
    h2?: number;                         // mmol/kg
    metals: {
      fe?: number;                       // μmol/kg
      cu?: number;
      zn?: number;
      mn?: number;
    };
  };

  // Biology
  biology: {
    dominantSpecies: string;
    abundance: 'low' | 'medium' | 'high';
    community: string[];
  };

  // Monitoring
  monitoring: {
    lastVisit: Timestamp;
    status: 'active' | 'waning' | 'inactive';
    changesSince?: string;
  };
}

/**
 * Rock sample
 */
export interface RockSample {
  sampleId: string;
  expeditionId: string;
  diveId: string;
  timestamp: Timestamp;

  // Location
  location: {
    latitude: number;
    longitude: number;
    depth: number;
    geologicalContext: string;
  };

  // Sample type
  rockType: 'basalt' | 'gabbro' | 'peridotite' | 'sulfide' | 'sedimentary' | 'manganese_nodule' | 'crust';

  // Description
  description: {
    color: string;
    texture: string;
    grainSize: string;
    weathering: 'fresh' | 'slightly_weathered' | 'heavily_weathered';
    notes: string;
  };

  // Dimensions
  dimensions: {
    length: number;                      // cm
    width: number;                       // cm
    height: number;                      // cm
    weight: number;                      // g
  };

  // Collection
  method: 'manipulator' | 'rock_drill' | 'grab_sampler';

  // Storage
  storage: {
    facility: string;
    catalogNumber: string;
    location: string;
  };

  // Analyses
  analyses: string[];                    // e.g., ["XRF", "Thin section", "Isotope dating"]
}

/**
 * Sediment core
 */
export interface SedimentCore {
  coreId: string;
  expeditionId: string;
  diveId?: string;
  timestamp: Timestamp;

  // Location
  location: {
    latitude: number;
    longitude: number;
    waterDepth: number;
    habitat: HabitatType;
  };

  // Core type
  coreType: 'push_core' | 'gravity_core' | 'piston_core' | 'box_core';

  // Dimensions
  dimensions: {
    length: number;                      // cm (recovered length)
    penetration: number;                 // cm (attempted penetration)
    diameter: number;                    // cm
    recovery: number;                    // % (length/penetration)
  };

  // Description
  description: {
    sedimentType: 'clay' | 'silt' | 'sand' | 'ooze' | 'mixed';
    color: string;
    layers: {
      topDepth: number;                  // cm
      bottomDepth: number;               // cm
      description: string;
    }[];
    bioturbation: 'none' | 'low' | 'moderate' | 'high';
  };

  // Storage
  storage: {
    facility: string;
    repository: string;
    archiveHalf: string;
    workingHalf: string;
  };

  // Subsamples
  subsamples: {
    depth: number;                       // cm
    analysis: string;
    status: 'available' | 'analyzed' | 'depleted';
  }[];
}

// ============================================================================
// Sonar Mapping
// ============================================================================

/**
 * Multibeam bathymetry survey
 */
export interface MultibeamSurvey {
  surveyId: string;
  expeditionId: string;

  // Survey details
  surveyDate: Timestamp;
  area: {
    name: string;
    extent: {
      minLat: number;
      maxLat: number;
      minLon: number;
      maxLon: number;
    };
    totalArea: number;                   // km²
  };

  // System
  sonarSystem: {
    manufacturer: string;
    model: string;
    frequency: number;                   // kHz
    beamCount: number;
    swathWidth: number;                  // meters
  };

  // Processing
  processing: {
    software: string;
    gridResolution: number;              // meters
    verticalUncertainty: number;         // meters
    horizontalUncertainty: number;       // meters
    soundVelocityProfile: boolean;
    tideCorrection: boolean;
  };

  // Deliverables
  deliverables: {
    bathymetryGrid: string;              // File path or URL
    backscatterMosaic?: string;
    format: 'geotiff' | 'netcdf' | 'xyz' | 'bag';
    crs: string;                         // e.g., "EPSG:4326"
  };

  // Statistics
  statistics: {
    minDepth: number;
    maxDepth: number;
    meanDepth: number;
    stdDevDepth: number;
  };
}

// ============================================================================
// Environmental DNA
// ============================================================================

/**
 * eDNA sample
 */
export interface EnvironmentalDNASample {
  sampleId: string;
  expeditionId: string;
  diveId?: string;
  timestamp: Timestamp;

  // Location
  location: {
    latitude: number;
    longitude: number;
    depth: number;
    depthZone: DepthZone;
  };

  // Water sample
  waterSample: {
    volume: number;                      // liters
    collectionMethod: 'niskin_bottle' | 'rov_sampler' | 'ctd_rosette';
    filtrationOnsite: boolean;
    filterSize: number;                  // μm
  };

  // DNA extraction
  extraction: {
    date: Timestamp;
    kit: string;
    concentration: number;               // ng/μL
    quality: {
      a260_280: number;
      a260_230: number;
    };
  };

  // Sequencing
  sequencing?: {
    platform: 'illumina' | 'nanopore' | 'pacbio';
    target: '18S' | '16S' | 'COI' | 'shotgun';
    readCount: number;
    averageLength: number;               // bp
  };

  // Analysis
  analysis?: {
    otuCount: number;
    taxonomicAssignments: {
      phylum: string;
      count: number;
    }[];
    novelSpecies: number;
  };

  // Storage
  storage: {
    facility: string;
    freezer: string;
    box: string;
    position: string;
    remainingVolume: number;             // μL
  };
}

// ============================================================================
// Pressure Systems
// ============================================================================

/**
 * Pressure measurement
 */
export interface PressureMeasurement {
  depth: number;                         // meters
  pressure: {
    atm: number;                         // atmospheres
    bar: number;                         // bars
    mpa: number;                         // megapascals
    psi: number;                         // pounds per square inch
  };
  temperature: number;                   // °C
  timestamp: Timestamp;
}

/**
 * Isobaric sampler
 */
export interface IsobaricSampler {
  samplerId: string;

  // Specifications
  maxPressure: number;                   // MPa
  volume: number;                        // mL
  material: 'titanium' | 'stainless_steel';

  // Sample
  sample: {
    sampleId: string;
    collectionDepth: number;
    collectionPressure: number;          // MPa
    maintainedPressure: number;          // MPa
    temperature: number;                 // °C
  };

  // Status
  status: 'sealed' | 'opened' | 'failed';
  integrity: 'intact' | 'compromised';
}

// ============================================================================
// API Request/Response Types
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
    version: string;
  };
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  limit: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  limit: number;
  hasMore: boolean;
}

/**
 * Date range filter
 */
export interface DateRangeFilter {
  startDate: Timestamp;
  endDate: Timestamp;
}

/**
 * Depth range filter
 */
export interface DepthRangeFilter {
  minDepth: number;
  maxDepth: number;
}

/**
 * Geographic filter
 */
export interface GeographicFilter {
  minLat: number;
  maxLat: number;
  minLon: number;
  maxLon: number;
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  GeoJSONLineString,
};
