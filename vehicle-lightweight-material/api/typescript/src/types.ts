/**
 * WIA-AUTO-021: Vehicle Lightweight Material - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Materials Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Material Categories and Types
// ============================================================================

/**
 * Material category classification
 */
export type MaterialCategory =
  | 'Ferrous'
  | 'Non-Ferrous'
  | 'Composite'
  | 'Polymer'
  | 'Ceramic'
  | 'Hybrid';

/**
 * Specific material types
 */
export type MaterialType =
  | 'Steel-Mild'
  | 'Steel-HSS'
  | 'Steel-AHSS'
  | 'Steel-UHSS'
  | 'Aluminum'
  | 'Magnesium'
  | 'Titanium'
  | 'CFRP'
  | 'GFRP'
  | 'Natural-Fiber-Composite'
  | 'MMC'
  | 'Sandwich'
  | 'Thermoplastic'
  | 'Thermoset';

/**
 * Steel subcategories
 */
export type SteelType =
  | 'Mild'
  | 'DP' // Dual Phase
  | 'CP' // Complex Phase
  | 'TRIP' // Transformation-Induced Plasticity
  | 'Martensitic'
  | 'Boron' // Hot-stamped
  | 'Stainless';

/**
 * Aluminum alloy series
 */
export type AluminumSeries =
  | '1xxx' // Pure aluminum
  | '2xxx' // Al-Cu
  | '3xxx' // Al-Mn
  | '4xxx' // Al-Si
  | '5xxx' // Al-Mg
  | '6xxx' // Al-Mg-Si
  | '7xxx' // Al-Zn
  | '8xxx'; // Other

/**
 * Composite fiber types
 */
export type FiberType =
  | 'Carbon-Standard'
  | 'Carbon-Intermediate'
  | 'Carbon-High-Modulus'
  | 'Glass-E'
  | 'Glass-S'
  | 'Glass-C'
  | 'Aramid'
  | 'Natural-Flax'
  | 'Natural-Hemp'
  | 'Natural-Jute';

/**
 * Resin matrix types
 */
export type MatrixType =
  | 'Epoxy'
  | 'Polyester'
  | 'Vinyl-Ester'
  | 'Phenolic'
  | 'Polyurethane'
  | 'Polypropylene'
  | 'Polyamide'
  | 'PEEK';

// ============================================================================
// Material Properties
// ============================================================================

/**
 * Mechanical properties of a material
 */
export interface MechanicalProperties {
  /** Density in kg/m³ */
  density: number;

  /** Tensile strength in MPa */
  tensileStrength: number;

  /** Yield strength in MPa */
  yieldStrength: number;

  /** Elongation at break in % */
  elongation: number;

  /** Young's modulus in GPa */
  youngsModulus: number;

  /** Poisson's ratio (dimensionless) */
  poissonsRatio: number;

  /** Shear modulus in GPa */
  shearModulus?: number;

  /** Hardness (various scales) */
  hardness?: {
    value: number;
    scale: 'Brinell' | 'Vickers' | 'Rockwell-A' | 'Rockwell-B' | 'Rockwell-C';
  };

  /** Specific strength in kN·m/kg */
  specificStrength?: number;

  /** Specific modulus in MN·m/kg */
  specificModulus?: number;

  /** Fatigue strength at 10^7 cycles in MPa */
  fatigueStrength?: number;

  /** Fracture toughness in MPa·√m */
  fractureToughness?: number;

  /** Test standard used */
  testStandard?: string;

  /** Test temperature in °C */
  temperature?: number;
}

/**
 * Thermal properties
 */
export interface ThermalProperties {
  /** Thermal conductivity in W/m·K */
  thermalConductivity: number;

  /** Specific heat capacity in J/kg·K */
  specificHeat: number;

  /** Coefficient of thermal expansion in 10⁻⁶/K */
  thermalExpansion: number;

  /** Melting point in °C */
  meltingPoint?: number;

  /** Maximum service temperature in °C */
  maxServiceTemp?: number;

  /** Minimum service temperature in °C */
  minServiceTemp?: number;
}

/**
 * Electrical properties
 */
export interface ElectricalProperties {
  /** Electrical conductivity in MS/m */
  electricalConductivity?: number;

  /** Electrical resistivity in nΩ·m */
  electricalResistivity?: number;

  /** Dielectric constant (for insulators) */
  dielectricConstant?: number;
}

/**
 * Forming and manufacturing properties
 */
export interface FormingProperties {
  /** Formability rating */
  formability: 'Poor' | 'Fair' | 'Good' | 'Excellent';

  /** Minimum bend radius in mm (for 1mm thickness) */
  minBendRadius?: number;

  /** Deep drawing ratio */
  deepDrawingRatio?: number;

  /** Spring-back tendency */
  springBack?: 'Low' | 'Medium' | 'High';

  /** Weldability rating */
  weldability?: 'Poor' | 'Fair' | 'Good' | 'Excellent';

  /** Machinability rating */
  machinability?: 'Poor' | 'Fair' | 'Good' | 'Excellent';
}

/**
 * Environmental and lifecycle properties
 */
export interface EnvironmentalProperties {
  /** Corrosion resistance rating */
  corrosionResistance: 'Poor' | 'Fair' | 'Good' | 'Excellent';

  /** Recyclability percentage */
  recyclability: number; // 0-100

  /** Recycling energy savings vs. primary production (%) */
  recyclingEnergySavings?: number;

  /** UV resistance */
  uvResistance?: 'Poor' | 'Fair' | 'Good' | 'Excellent';

  /** Moisture absorption (%) */
  moistureAbsorption?: number;

  /** Biodegradable */
  biodegradable?: boolean;
}

/**
 * Cost and availability
 */
export interface CostProperties {
  /** Relative cost (Mild steel = 1.0) */
  relativeCost: number;

  /** Typical price range in $/kg */
  priceRange?: {
    min: number;
    max: number;
    currency: string;
  };

  /** Availability rating */
  availability?: 'Limited' | 'Moderate' | 'Good' | 'Excellent';

  /** Lead time in days */
  leadTime?: number;
}

/**
 * Complete material specification
 */
export interface MaterialSpecification {
  /** Unique material identifier */
  id: string;

  /** Material name */
  name: string;

  /** Category */
  category: MaterialCategory;

  /** Material type */
  type: MaterialType;

  /** Subcategory (e.g., specific alloy) */
  subcategory?: string;

  /** Chemical composition (element: percentage) */
  composition?: Record<string, number>;

  /** Mechanical properties */
  mechanical: MechanicalProperties;

  /** Thermal properties */
  thermal?: ThermalProperties;

  /** Electrical properties */
  electrical?: ElectricalProperties;

  /** Forming properties */
  forming?: FormingProperties;

  /** Environmental properties */
  environmental?: EnvironmentalProperties;

  /** Cost properties */
  cost?: CostProperties;

  /** Typical applications */
  applications?: string[];

  /** Advantages */
  advantages?: string[];

  /** Limitations */
  limitations?: string[];

  /** Standards and specifications */
  standards?: string[];

  /** Data source */
  source?: string;

  /** Last updated */
  lastUpdated?: Date;
}

// ============================================================================
// Component and Geometry
// ============================================================================

/**
 * Component types in a vehicle
 */
export type ComponentType =
  | 'Body-Panel'
  | 'Hood'
  | 'Door'
  | 'Fender'
  | 'Roof'
  | 'Trunk-Lid'
  | 'Bumper'
  | 'Chassis'
  | 'Subframe'
  | 'Suspension-Arm'
  | 'Wheel'
  | 'Engine-Block'
  | 'Transmission-Case'
  | 'Battery-Enclosure'
  | 'Floor-Pan'
  | 'Pillar-A'
  | 'Pillar-B'
  | 'Pillar-C'
  | 'Door-Beam'
  | 'Roof-Rail'
  | 'Interior-Trim'
  | 'Seat-Frame'
  | 'Custom';

/**
 * Geometric configuration types
 */
export type GeometryType =
  | 'Sheet' // Flat or formed sheet
  | 'Tube' // Hollow cylinder
  | 'Extrusion' // Constant cross-section
  | 'Casting' // Cast component
  | 'Forging' // Forged component
  | 'Composite-Layup' // Laminated composite
  | 'Custom'; // Custom geometry

/**
 * Sheet geometry
 */
export interface SheetGeometry {
  type: 'Sheet';
  /** Surface area in m² */
  surfaceArea: number;
  /** Thickness in mm */
  thickness: number;
  /** Curvature (for formed sheets) */
  curvature?: number;
}

/**
 * Tube geometry
 */
export interface TubeGeometry {
  type: 'Tube';
  /** Outer diameter in mm */
  outerDiameter: number;
  /** Wall thickness in mm */
  wallThickness: number;
  /** Length in mm */
  length: number;
  /** Cross-section shape */
  crossSection?: 'Circular' | 'Square' | 'Rectangular' | 'Hexagonal';
}

/**
 * Extrusion geometry
 */
export interface ExtrusionGeometry {
  type: 'Extrusion';
  /** Cross-sectional area in mm² */
  crossSectionArea: number;
  /** Perimeter in mm */
  perimeter: number;
  /** Length in mm */
  length: number;
}

/**
 * Custom geometry
 */
export interface CustomGeometry {
  type: 'Custom';
  /** Volume in m³ */
  volume: number;
  /** Mass in kg (if known) */
  mass?: number;
}

/**
 * Union type for all geometry types
 */
export type Geometry =
  | SheetGeometry
  | TubeGeometry
  | ExtrusionGeometry
  | CustomGeometry;

/**
 * Component specification
 */
export interface ComponentSpecification {
  /** Component name */
  name: string;

  /** Component type */
  type: ComponentType;

  /** Geometry */
  geometry: Geometry;

  /** Current material */
  currentMaterial: string;

  /** Load cases */
  loadCases?: LoadCase[];

  /** Safety requirements */
  safetyRequirements?: SafetyRequirements;

  /** Manufacturing constraints */
  manufacturingConstraints?: ManufacturingConstraints;
}

// ============================================================================
// Load Cases and Analysis
// ============================================================================

/**
 * Load case type
 */
export type LoadType =
  | 'Tensile'
  | 'Compressive'
  | 'Bending'
  | 'Torsion'
  | 'Shear'
  | 'Combined'
  | 'Impact'
  | 'Fatigue';

/**
 * Load case specification
 */
export interface LoadCase {
  /** Load case identifier */
  id: string;

  /** Load type */
  type: LoadType;

  /** Applied force in N */
  force?: number;

  /** Applied moment in N·m */
  moment?: number;

  /** Applied pressure in MPa */
  pressure?: number;

  /** Safety factor required */
  safetyFactor: number;

  /** Frequency of occurrence */
  frequency?: 'Rare' | 'Occasional' | 'Frequent' | 'Continuous';

  /** Temperature during load in °C */
  temperature?: number;
}

/**
 * Structural analysis result
 */
export interface StructuralAnalysis {
  /** Maximum stress in MPa */
  maxStress: number;

  /** Maximum strain (dimensionless) */
  maxStrain: number;

  /** Maximum deflection in mm */
  maxDeflection: number;

  /** Stress concentration factor */
  stressConcentration?: number;

  /** Factor of safety */
  factorOfSafety: number;

  /** Is design safe? */
  isSafe: boolean;

  /** Failure mode (if unsafe) */
  failureMode?: 'Yield' | 'Fracture' | 'Buckling' | 'Fatigue';

  /** Recommendations */
  recommendations?: string[];
}

// ============================================================================
// Weight Reduction
// ============================================================================

/**
 * Weight reduction calculation request
 */
export interface WeightReductionRequest {
  /** Component specification */
  component: ComponentSpecification;

  /** New (lightweight) material */
  newMaterial: string;

  /** Maintain equivalent stiffness? */
  equivalentStiffness?: boolean;

  /** Maintain equivalent strength? */
  equivalentStrength?: boolean;

  /** Quantity of components */
  quantity?: number;
}

/**
 * Weight reduction calculation result
 */
export interface WeightReductionResult {
  /** Original mass in kg */
  originalMass: number;

  /** New mass in kg */
  newMass: number;

  /** Weight saved in kg */
  weightSaved: number;

  /** Percentage reduction */
  percentReduction: number;

  /** Total weight saved (if quantity > 1) in kg */
  totalWeightSaved?: number;

  /** New thickness required in mm (if geometry changed) */
  newThickness?: number;

  /** Structural integrity assessment */
  structuralIntegrity?: StructuralAnalysis;

  /** Cost implications */
  costImplication?: CostAnalysis;

  /** Fuel efficiency impact */
  fuelEfficiency?: FuelEfficiencyImpact;

  /** Environmental impact */
  environmentalImpact?: EnvironmentalImpact;
}

/**
 * Cost analysis
 */
export interface CostAnalysis {
  /** Original material cost in $ */
  originalMaterialCost: number;

  /** New material cost in $ */
  newMaterialCost: number;

  /** Tooling cost difference in $ */
  toolingCostDelta?: number;

  /** Manufacturing cost difference in $ */
  manufacturingCostDelta?: number;

  /** Total cost delta in $ */
  totalCostDelta: number;

  /** Payback period in years (based on fuel savings) */
  paybackPeriod?: number;

  /** Break-even quantity */
  breakEvenQuantity?: number;
}

/**
 * Fuel efficiency impact
 */
export interface FuelEfficiencyImpact {
  /** Vehicle total weight in kg */
  vehicleWeight: number;

  /** Weight reduction percentage */
  weightReductionPercent: number;

  /** Efficiency coefficient (typically 0.6-0.7) */
  efficiencyCoefficient: number;

  /** Fuel consumption reduction in % */
  fuelConsumptionReduction: number;

  /** Fuel savings in L/100km */
  fuelSavingsL100km?: number;

  /** CO₂ reduction in g/km */
  co2Reduction?: number;

  /** Annual fuel cost savings in $ (based on typical usage) */
  annualFuelCostSavings?: number;
}

/**
 * Environmental impact assessment
 */
export interface EnvironmentalImpact {
  /** Lifetime CO₂ savings in kg */
  lifetimeCO2Savings: number;

  /** Production energy difference in MJ */
  productionEnergyDelta: number;

  /** Recycling energy savings in MJ */
  recyclingEnergySavings: number;

  /** Overall environmental score (0-100, higher is better) */
  environmentalScore: number;

  /** Carbon footprint reduction (%) */
  carbonFootprintReduction: number;
}

// ============================================================================
// Manufacturing
// ============================================================================

/**
 * Manufacturing process types
 */
export type ManufacturingProcess =
  | 'Stamping'
  | 'Deep-Drawing'
  | 'Hot-Stamping'
  | 'Hydroforming'
  | 'Superplastic-Forming'
  | 'Roll-Forming'
  | 'Extrusion'
  | 'Casting-HPDC'
  | 'Casting-Sand'
  | 'Forging'
  | 'RTM'
  | 'Autoclave'
  | 'Compression-Molding'
  | 'HP-RTM'
  | 'Hand-Layup'
  | 'Pultrusion'
  | 'Additive-Manufacturing'
  | 'Machining';

/**
 * Manufacturing constraints
 */
export interface ManufacturingConstraints {
  /** Preferred manufacturing processes */
  preferredProcesses?: ManufacturingProcess[];

  /** Excluded processes */
  excludedProcesses?: ManufacturingProcess[];

  /** Maximum production volume per year */
  maxProductionVolume?: number;

  /** Minimum production volume per year */
  minProductionVolume?: number;

  /** Required cycle time in seconds */
  requiredCycleTime?: number;

  /** Tolerances in mm */
  tolerances?: {
    dimensional: number;
    surface: number;
  };

  /** Surface finish requirement */
  surfaceFinish?: 'Class-A' | 'Class-B' | 'Class-C' | 'Functional';
}

/**
 * Manufacturing process recommendation
 */
export interface ProcessRecommendation {
  /** Process name */
  process: ManufacturingProcess;

  /** Suitability score (0-100) */
  suitabilityScore: number;

  /** Cycle time in seconds */
  cycleTime: number;

  /** Tooling cost in $ */
  toolingCost: number;

  /** Unit cost in $ */
  unitCost: number;

  /** Minimum economical volume */
  minEconomicalVolume: number;

  /** Advantages */
  advantages: string[];

  /** Disadvantages */
  disadvantages: string[];

  /** Process parameters */
  parameters?: Record<string, number | string>;
}

// ============================================================================
// Joining Technologies
// ============================================================================

/**
 * Joining method types
 */
export type JoiningMethod =
  | 'RSW' // Resistance Spot Welding
  | 'Laser-Welding'
  | 'FSW' // Friction Stir Welding
  | 'MIG-Welding'
  | 'TIG-Welding'
  | 'SPR' // Self-Piercing Rivet
  | 'FDS' // Flow Drill Screw
  | 'Clinching'
  | 'Mechanical-Fastener'
  | 'Adhesive-Bonding'
  | 'Rivet-Bonding'
  | 'Weld-Bonding'
  | 'Brazing'
  | 'Soldering';

/**
 * Joint specification
 */
export interface JointSpecification {
  /** Joint identifier */
  id: string;

  /** Material A */
  materialA: string;

  /** Material B */
  materialB: string;

  /** Thickness A in mm */
  thicknessA: number;

  /** Thickness B in mm */
  thicknessB: number;

  /** Recommended joining methods */
  recommendedMethods?: JoiningMethod[];

  /** Required joint strength in N */
  requiredStrength?: number;

  /** Disassembly requirement */
  disassemblyRequired?: boolean;

  /** Corrosion protection requirement */
  corrosionProtection?: boolean;
}

/**
 * Joint strength properties
 */
export interface JointStrength {
  /** Tensile-shear strength in N */
  tensileShear: number;

  /** Cross-tension strength in N */
  crossTension: number;

  /** Peel strength in N/mm (for adhesives) */
  peelStrength?: number;

  /** Fatigue strength at 10^7 cycles in N */
  fatigueStrength?: number;
}

/**
 * Joining method recommendation
 */
export interface JoiningRecommendation {
  /** Joining method */
  method: JoiningMethod;

  /** Compatibility score (0-100) */
  compatibilityScore: number;

  /** Joint strength */
  strength: JointStrength;

  /** Cost per joint in $ */
  costPerJoint: number;

  /** Cycle time in seconds */
  cycleTime: number;

  /** Advantages */
  advantages: string[];

  /** Disadvantages */
  disadvantages: string[];

  /** Special requirements */
  specialRequirements?: string[];
}

// ============================================================================
// Crash Safety
// ============================================================================

/**
 * Crash test type
 */
export type CrashTestType =
  | 'Frontal-Full'
  | 'Frontal-Offset'
  | 'Side-Pole'
  | 'Side-Barrier'
  | 'Rear-Impact'
  | 'Roof-Crush'
  | 'Rollover'
  | 'Pedestrian';

/**
 * Safety requirements
 */
export interface SafetyRequirements {
  /** Crash test types that apply */
  applicableTests?: CrashTestType[];

  /** Minimum specific energy absorption in kJ/kg */
  minSEA?: number;

  /** Maximum intrusion allowed in mm */
  maxIntrusion?: number;

  /** Peak deceleration limit in g */
  maxDeceleration?: number;

  /** Energy absorption target in kJ */
  energyAbsorptionTarget?: number;

  /** NCAP rating target */
  ncapTarget?: 1 | 2 | 3 | 4 | 5;
}

/**
 * Crash simulation request
 */
export interface CrashSimulationRequest {
  /** Component or structure */
  component: ComponentSpecification;

  /** Material */
  material: string;

  /** Impact conditions */
  impactConditions: {
    /** Impact velocity in m/s */
    velocity: number;
    /** Impact mass in kg */
    mass: number;
    /** Impact angle in degrees */
    angle: number;
  };

  /** Boundary conditions */
  boundaryConditions?: 'Fixed' | 'Simply-Supported' | 'Free';
}

/**
 * Crash simulation result
 */
export interface CrashSimulationResult {
  /** Total energy absorbed in J */
  energyAbsorbed: number;

  /** Specific energy absorption in kJ/kg */
  specificEnergyAbsorption: number;

  /** Peak force in N */
  peakForce: number;

  /** Mean crushing force in N */
  meanForce: number;

  /** Crush distance in mm */
  crushDistance: number;

  /** Deformation mode */
  deformationMode: 'Progressive' | 'Global-Buckling' | 'Fracture' | 'Mixed';

  /** Force efficiency (mean/peak) */
  forceEfficiency: number;

  /** Timeline data */
  timeline?: {
    /** Time in ms */
    time: number[];
    /** Force in N */
    force: number[];
    /** Displacement in mm */
    displacement: number[];
    /** Energy in J */
    energy: number[];
  };

  /** Safety assessment */
  safetyAssessment?: {
    meetsRequirements: boolean;
    issues: string[];
    recommendations: string[];
  };
}

// ============================================================================
// Material Selection
// ============================================================================

/**
 * Material selection criteria
 */
export interface MaterialSelectionCriteria {
  /** Application description */
  application: string;

  /** Component type */
  componentType?: ComponentType;

  /** Load cases */
  loadCases?: LoadCase[];

  /** Constraints */
  constraints?: {
    /** Maximum weight in kg */
    maxWeight?: number;
    /** Maximum cost in $ */
    maxCost?: number;
    /** Minimum tensile strength in MPa */
    minStrength?: number;
    /** Minimum Young's modulus in GPa */
    minStiffness?: number;
    /** Corrosion resistance requirement */
    corrosionResistance?: 'Low' | 'Medium' | 'High';
    /** Formability requirement */
    formability?: 'Low' | 'Medium' | 'High';
    /** Temperature range in °C */
    temperatureRange?: [number, number];
  };

  /** Preferences (0-1 scale) */
  preferences?: {
    /** Weight preference */
    lightWeight?: number;
    /** Cost preference (lower cost) */
    lowCost?: number;
    /** Recyclability preference */
    recyclability?: number;
    /** Sustainability preference */
    sustainability?: number;
    /** Availability preference */
    availability?: number;
  };

  /** Excluded materials */
  excludedMaterials?: string[];
}

/**
 * Material selection result
 */
export interface MaterialSelectionResult {
  /** Ranked material recommendations */
  ranking: MaterialRecommendation[];

  /** Analysis charts */
  analysis?: {
    /** Ashby chart data points */
    ashbyChart?: {
      material: string;
      strength: number;
      density: number;
      specificStrength: number;
    }[];
    /** Cost vs. performance */
    costPerformance?: {
      material: string;
      cost: number;
      performance: number;
    }[];
  };

  /** Selection rationale */
  rationale?: string;
}

/**
 * Material recommendation
 */
export interface MaterialRecommendation {
  /** Material ID */
  materialId: string;

  /** Material name */
  materialName: string;

  /** Overall score (0-100) */
  overallScore: number;

  /** Performance metrics */
  metrics?: {
    weight: number;
    strength: number;
    stiffness: number;
    cost: number;
    manufacturability: number;
    sustainability: number;
  };

  /** Pros */
  pros: string[];

  /** Cons */
  cons: string[];

  /** Estimated weight in kg */
  estimatedWeight?: number;

  /** Estimated cost in $ */
  estimatedCost?: number;

  /** Confidence level */
  confidence?: 'Low' | 'Medium' | 'High';
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for automotive materials
 */
export const MATERIAL_CONSTANTS = {
  /** Standard gravity in m/s² */
  GRAVITY: 9.81,

  /** Typical efficiency coefficient for fuel consumption vs. weight */
  FUEL_EFFICIENCY_COEFFICIENT: 0.65,

  /** CO₂ per liter of gasoline in g */
  CO2_PER_LITER_GASOLINE: 2392,

  /** CO₂ per liter of diesel in g */
  CO2_PER_LITER_DIESEL: 2640,

  /** Average vehicle lifetime in km */
  AVERAGE_VEHICLE_LIFETIME_KM: 250000,

  /** Minimum safety factor for structural components */
  MIN_SAFETY_FACTOR: 1.5,

  /** Minimum safety factor for crash-critical components */
  MIN_CRASH_SAFETY_FACTOR: 2.0,

  /** Minimum specific energy absorption for crash structures in kJ/kg */
  MIN_SEA: 50,
} as const;

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
 * Comparison result
 */
export interface ComparisonResult<T> {
  items: T[];
  winner?: T;
  analysis: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-021 error codes
 */
export enum MaterialErrorCode {
  MATERIAL_NOT_FOUND = 'M001',
  INVALID_PROPERTIES = 'M002',
  INSUFFICIENT_STRENGTH = 'M003',
  EXCESSIVE_WEIGHT = 'M004',
  MANUFACTURING_NOT_FEASIBLE = 'M005',
  JOINING_NOT_COMPATIBLE = 'M006',
  COST_EXCEEDS_BUDGET = 'M007',
  SAFETY_REQUIREMENTS_NOT_MET = 'M008',
  CORROSION_RISK = 'M009',
  INVALID_GEOMETRY = 'M010',
}

/**
 * Material error
 */
export class MaterialError extends Error {
  constructor(
    public code: MaterialErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MaterialError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Categories
  MaterialCategory,
  MaterialType,
  SteelType,
  AluminumSeries,
  FiberType,
  MatrixType,

  // Properties
  MechanicalProperties,
  ThermalProperties,
  ElectricalProperties,
  FormingProperties,
  EnvironmentalProperties,
  CostProperties,
  MaterialSpecification,

  // Components
  ComponentType,
  GeometryType,
  SheetGeometry,
  TubeGeometry,
  ExtrusionGeometry,
  CustomGeometry,
  Geometry,
  ComponentSpecification,

  // Analysis
  LoadType,
  LoadCase,
  StructuralAnalysis,

  // Weight Reduction
  WeightReductionRequest,
  WeightReductionResult,
  CostAnalysis,
  FuelEfficiencyImpact,
  EnvironmentalImpact,

  // Manufacturing
  ManufacturingProcess,
  ManufacturingConstraints,
  ProcessRecommendation,

  // Joining
  JoiningMethod,
  JointSpecification,
  JointStrength,
  JoiningRecommendation,

  // Safety
  CrashTestType,
  SafetyRequirements,
  CrashSimulationRequest,
  CrashSimulationResult,

  // Selection
  MaterialSelectionCriteria,
  MaterialSelectionResult,
  MaterialRecommendation,

  // Utility
  ComparisonResult,
};

export { MATERIAL_CONSTANTS, MaterialErrorCode, MaterialError };
