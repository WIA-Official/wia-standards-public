/**
 * WIA-SEMI-020: Semiconductor Packaging Standard - Type Definitions
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

// ============================================================================
// Package Types
// ============================================================================

export enum PackageType {
  INTERPOSER_2_5D = '2.5D_INTERPOSER',
  STACKED_3D = '3D_STACKED',
  FAN_OUT_WLP = 'FAN_OUT_WLP',
  FAN_OUT_PANEL = 'FAN_OUT_PANEL',
  SYSTEM_IN_PACKAGE = 'SYSTEM_IN_PACKAGE',
  CHIPLET_BASED = 'CHIPLET_BASED',
}

export enum InterposerMaterial {
  SILICON = 'SILICON',
  ORGANIC = 'ORGANIC',
  GLASS = 'GLASS',
}

export enum BondingType {
  MICROBUMP = 'MICROBUMP',
  HYBRID_BONDING = 'HYBRID_BONDING',
  COPPER_PILLAR = 'COPPER_PILLAR',
  THERMOCOMPRESSION = 'THERMOCOMPRESSION',
}

// ============================================================================
// Dimensional Specifications
// ============================================================================

export interface Dimensions {
  length: number; // mm
  width: number; // mm
  height: number; // mm
  tolerance: number; // mm
}

export interface TSVSpecification {
  diameter: number; // μm
  depth: number; // μm
  pitch: number; // μm
  aspectRatio: number; // depth:diameter
  density: number; // per mm²
  resistance: number; // mΩ
  capacitance: number; // fF
}

export interface MicrobumpSpecification {
  pitch: number; // μm
  height: number; // μm
  diameter: number; // μm
  coplanarity: number; // μm
  standoffHeight: number; // μm
  material: string;
  shearStrength: number; // MPa
}

export interface RDLSpecification {
  layers: number;
  lineWidth: number; // μm
  lineSpacing: number; // μm
  metalThickness: number; // μm
  viaDiameter: number; // μm
  metal: 'COPPER' | 'ALUMINUM';
  dielectric: string;
}

// ============================================================================
// Thermal Specifications
// ============================================================================

export interface ThermalProperties {
  junctionToCaseResistance: number; // °C/W (θJC)
  junctionToBoardResistance: number; // °C/W (θJB)
  junctionToAmbientResistance?: number; // °C/W (θJA)
  maxJunctionTemp: number; // °C
  operatingTempMin: number; // °C
  operatingTempMax: number; // °C
  thermalConductivity: number; // W/m·K
}

export interface ThermalInterfaceMaterial {
  type: 'GREASE' | 'PAD' | 'PHASE_CHANGE' | 'SOLDER' | 'LIQUID_METAL';
  thermalConductivity: number; // W/m·K
  bondLineThickness: number; // μm
  thermalResistance: number; // °C·cm²/W
  operatingTempRange: [number, number]; // [min, max] in °C
}

export interface CoolingSpecification {
  method: 'NATURAL_CONVECTION' | 'FORCED_AIR' | 'LIQUID_COOLING' | 'IMMERSION';
  heatTransferCoefficient: number; // W/m²·K
  maxPowerDissipation: number; // W
  airflowRate?: number; // CFM
  liquidFlowRate?: number; // ml/min
}

// ============================================================================
// Electrical Specifications
// ============================================================================

export interface ElectricalProperties {
  resistance: number; // Ω
  capacitance: number; // F
  inductance: number; // H
  impedance: number; // Ω
  insertionLoss: number; // dB
  returnLoss: number; // dB
  crosstalk: number; // dB
}

export interface PowerIntegrity {
  targetImpedance: number; // mΩ
  dcResistance: number; // mΩ
  irDrop: number; // V
  voltageRipple: number; // mV peak-to-peak
  decouplingCapacitance: number; // F
}

export interface SignalIntegrity {
  impedance: number; // Ω (single-ended or differential)
  insertionLoss: number; // dB at Nyquist frequency
  returnLoss: number; // dB
  nearEndCrosstalk: number; // dB
  farEndCrosstalk: number; // dB
  isi: number; // UI (Inter-Symbol Interference)
}

export interface HighSpeedInterface {
  protocol: 'PCIE' | 'ETHERNET' | 'UCIE' | 'CXL' | 'SERDES';
  dataRate: number; // Gbps
  lanes: number;
  signaling: 'SINGLE_ENDED' | 'DIFFERENTIAL';
  ber: number; // Bit Error Rate
  jitterBudget: number; // UI
}

// ============================================================================
// Chiplet and UCIe Specifications
// ============================================================================

export interface ChipletSpecification {
  id: string;
  dieSize: Dimensions;
  processNode: number; // nm
  function: 'COMPUTE' | 'IO' | 'MEMORY' | 'ACCELERATOR' | 'CUSTOM';
  powerDomain: string;
  interconnectStandard: 'UCIE' | 'BOW' | 'AIB' | 'CUSTOM';
}

export interface UCIeConfiguration {
  version: string; // '1.0', '1.1', etc.
  packageType: 'STANDARD' | 'ADVANCED'; // UCIe-S or UCIe-A
  bumpPitch: number; // μm (25, 55, or 9 for advanced)
  laneCount: number; // 16, 32, 64, etc.
  dataRate: number; // Gbps per lane
  signaling: 'SINGLE_ENDED' | 'DIFFERENTIAL';
  protocol: 'RAW' | 'PCIE' | 'CXL';
  bandwidth: number; // GB/s total
  latency: number; // ns
  powerEfficiency: number; // pJ/bit
}

// ============================================================================
// HBM Specifications
// ============================================================================

export interface HBMSpecification {
  generation: 'HBM' | 'HBM2' | 'HBM2E' | 'HBM3' | 'HBM3E';
  stackHeight: number; // number of DRAM dies
  dieThickness: number; // μm
  totalHeight: number; // μm
  interfaceWidth: number; // bits (typically 1024)
  dataRate: number; // Gbps per pin
  bandwidth: number; // GB/s per stack
  capacity: number; // GB per stack
  channels: number; // 8 or 16
  tsvCount: number;
}

// ============================================================================
// Material Properties
// ============================================================================

export interface MaterialProperties {
  name: string;
  thermalConductivity: number; // W/m·K
  electricalResistivity?: number; // μΩ·cm
  cte: number; // ppm/°C (Coefficient of Thermal Expansion)
  youngsModulus?: number; // GPa
  poissonRatio?: number;
  density?: number; // g/cm³
  dielectricConstant?: number;
  lossTangent?: number;
}

export interface SolderMaterial extends MaterialProperties {
  alloy: 'SAC305' | 'SAC405' | 'SnBi' | 'SnAg' | 'CUSTOM';
  meltingPoint: number; // °C
  yieldStrength: number; // MPa
  tensileStrength: number; // MPa
}

export interface MoldingCompound extends MaterialProperties {
  tg: number; // °C (Glass Transition Temperature)
  fillerContent: number; // percentage
  flexuralModulus: number; // GPa
  moistureAbsorption: number; // percentage
}

// ============================================================================
// Reliability Specifications
// ============================================================================

export interface ReliabilityTest {
  testName: string;
  standard: string; // e.g., 'JEDEC JESD22-A104'
  conditions: ReliabilityConditions;
  duration: number | string; // hours or cycles
  sampleSize: number;
  acceptanceCriteria: string;
}

export interface ReliabilityConditions {
  temperatureMin?: number; // °C
  temperatureMax?: number; // °C
  humidity?: number; // % RH
  bias?: boolean;
  pressure?: number; // atm
  dwellTime?: number; // minutes
  rampRate?: number; // °C/min
}

export enum ReliabilityTestType {
  TEMPERATURE_CYCLING = 'TCT',
  HTSL = 'HTSL',
  THB = 'THB',
  HAST = 'HAST',
  AUTOCLAVE = 'PCT',
  PRECONDITIONING = 'MSL',
  BOARD_LEVEL_RELIABILITY = 'BLR',
  DROP_TEST = 'DROP',
  VIBRATION = 'VIBRATION',
  HTOL = 'HTOL',
}

export interface FailureMetrics {
  mttf: number; // hours (Mean Time To Failure)
  failureRate: number; // FIT (Failures In Time per billion hours)
  characteristicLifetime: number; // hours or cycles
  weibullBeta: number; // shape parameter
  weibullEta: number; // scale parameter
}

export interface AccelerationFactors {
  arrhenius: {
    activationEnergy: number; // eV
    useTemperature: number; // °C
    testTemperature: number; // °C
    accelerationFactor: number;
  };
  coffinManson?: {
    exponent: number;
    deltaT: number; // °C
    accelerationFactor: number;
  };
}

// ============================================================================
// Package Design
// ============================================================================

export interface PackageDesign {
  designId: string;
  type: PackageType;
  dimensions: Dimensions;
  dies: ChipletSpecification[];
  interconnect: InterconnectDesign;
  thermal: ThermalDesign;
  electrical: ElectricalDesign;
  materials: MaterialStack;
  reliability: ReliabilityRequirements;
}

export interface InterconnectDesign {
  interposer?: InterposerSpecification;
  tsv?: TSVSpecification;
  microbumps?: MicrobumpSpecification;
  rdl?: RDLSpecification;
  chipletInterconnects?: UCIeConfiguration[];
  hbm?: HBMSpecification[];
}

export interface InterposerSpecification {
  material: InterposerMaterial;
  dimensions: Dimensions;
  tsv: TSVSpecification;
  rdl: RDLSpecification;
  thickness: number; // μm
  warpageLimit: number; // μm
}

export interface ThermalDesign {
  properties: ThermalProperties;
  tim?: ThermalInterfaceMaterial;
  cooling: CoolingSpecification;
  hotspots: HotspotAnalysis[];
}

export interface HotspotAnalysis {
  location: [number, number, number]; // x, y, z in mm
  temperature: number; // °C
  powerDensity: number; // W/cm²
  mitigation?: string;
}

export interface ElectricalDesign {
  power: PowerIntegrity;
  signal: SignalIntegrity;
  highSpeedInterfaces: HighSpeedInterface[];
}

export interface MaterialStack {
  substrate: MaterialProperties;
  molding?: MoldingCompound;
  solder: SolderMaterial;
  underfill?: MaterialProperties;
  tim?: ThermalInterfaceMaterial;
}

export interface ReliabilityRequirements {
  qualification: ReliabilityTest[];
  targetMTTF: number; // hours
  targetFailureRate: number; // FIT
  operatingLifetime: number; // years
  application: 'CONSUMER' | 'INDUSTRIAL' | 'AUTOMOTIVE' | 'AEROSPACE';
}

// ============================================================================
// Simulation and Analysis
// ============================================================================

export interface ThermalSimulation {
  toolName: string;
  meshSize: number; // elements
  powerMap: PowerDistribution[];
  ambientTemp: number; // °C
  coolingConfig: CoolingSpecification;
  results: ThermalSimulationResults;
}

export interface PowerDistribution {
  dieId: string;
  power: number; // W
  distribution: 'UNIFORM' | 'FUNCTIONAL_BLOCK' | 'DETAILED';
  map?: number[][]; // 2D power density map if detailed
}

export interface ThermalSimulationResults {
  maxTemperature: number; // °C
  avgTemperature: number; // °C
  temperatureGradient: number; // °C/mm
  junctionToCaseResistance: number; // °C/W
  hotspots: HotspotAnalysis[];
}

export interface ElectricalSimulation {
  type: 'SIGNAL_INTEGRITY' | 'POWER_INTEGRITY' | 'EMI';
  toolName: string;
  frequency: number; // GHz
  results: ElectricalSimulationResults;
}

export interface ElectricalSimulationResults {
  sParameters?: SParameters;
  impedance?: number; // Ω
  insertionLoss?: number; // dB
  returnLoss?: number; // dB
  crosstalk?: number; // dB
  eyeDiagram?: EyeDiagramMetrics;
}

export interface SParameters {
  s11: number; // dB (return loss)
  s21: number; // dB (insertion loss)
  s31?: number; // dB (near-end crosstalk)
  s41?: number; // dB (far-end crosstalk)
  frequency: number; // GHz
}

export interface EyeDiagramMetrics {
  eyeHeight: number; // mV
  eyeWidth: number; // UI
  jitter: number; // UI
  ber: number; // Bit Error Rate
}

// ============================================================================
// Manufacturing Process
// ============================================================================

export interface ManufacturingProcess {
  processName: string;
  steps: ProcessStep[];
  yield: number; // percentage
  cycleTime: number; // hours
  cost: number; // $ per unit
}

export interface ProcessStep {
  stepNumber: number;
  operation: string;
  equipment: string;
  parameters: Record<string, any>;
  duration: number; // minutes
  criticalParameters: string[];
}

export interface QualityMetrics {
  defectDensity: number; // defects per cm²
  yield: number; // percentage
  cpk: number; // Process Capability Index
  outgoingQualityLevel: number; // PPM
}

// ============================================================================
// Test and Measurement
// ============================================================================

export interface TestSpecification {
  testType: 'ELECTRICAL' | 'THERMAL' | 'MECHANICAL' | 'RELIABILITY';
  testName: string;
  equipment: string;
  procedure: string;
  measurements: Measurement[];
  limits: TestLimits;
}

export interface Measurement {
  parameter: string;
  value: number;
  unit: string;
  timestamp: Date;
  equipment: string;
}

export interface TestLimits {
  parameter: string;
  minimum?: number;
  maximum?: number;
  nominal?: number;
  unit: string;
}

// ============================================================================
// Documentation
// ============================================================================

export interface PackageDocumentation {
  designSpecification: PackageDesign;
  thermalAnalysis: ThermalSimulation;
  electricalAnalysis: ElectricalSimulation;
  reliabilityData: ReliabilityReport;
  manufacturingData: ManufacturingProcess;
  qualificationReport: QualificationReport;
}

export interface ReliabilityReport {
  tests: ReliabilityTest[];
  results: ReliabilityTestResults[];
  failureAnalysis: FailureAnalysis[];
  metrics: FailureMetrics;
}

export interface ReliabilityTestResults {
  testName: string;
  testType: ReliabilityTestType;
  sampleSize: number;
  failures: number;
  failureMode: string[];
  passed: boolean;
}

export interface FailureAnalysis {
  failureMode: string;
  rootCause: string;
  correctiveAction: string;
  preventiveAction: string;
}

export interface QualificationReport {
  packageType: PackageType;
  application: string;
  reliabilityTests: ReliabilityTestResults[];
  electricalTests: TestSpecification[];
  thermalTests: TestSpecification[];
  mechanicalTests: TestSpecification[];
  approved: boolean;
  approvalDate: Date;
  approver: string;
}

// ============================================================================
// Utility Types
// ============================================================================

export type Temperature = number; // °C
export type Distance = number; // mm or μm depending on context
export type Resistance = number; // Ω or mΩ
export type Capacitance = number; // F
export type Inductance = number; // H
export type Power = number; // W
export type Frequency = number; // Hz, MHz, GHz

export interface Range<T> {
  min: T;
  max: T;
  typical?: T;
}

export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

// ============================================================================
// Constants
// ============================================================================

export const PHYSICAL_CONSTANTS = {
  BOLTZMANN_CONSTANT: 8.617e-5, // eV/K
  AMBIENT_TEMPERATURE: 25, // °C
  ROOM_TEMPERATURE: 25, // °C
  SILICON_THERMAL_CONDUCTIVITY: 150, // W/m·K
  COPPER_THERMAL_CONDUCTIVITY: 400, // W/m·K
  SILICON_CTE: 2.6, // ppm/°C
} as const;

export const JEDEC_STANDARDS = {
  JEP95: '2.5D Silicon Interposer Design Guide',
  JESD235: 'High Bandwidth Memory (HBM) DRAM',
  JESD22_A104: 'Temperature Cycling',
  JESD22_A103: 'High Temperature Storage Life',
  JESD22_A101: 'Steady-State Temperature Humidity Bias Life Test',
  JESD22_A110: 'Highly Accelerated Temperature and Humidity Stress Test (HAST)',
  JSTD020: 'Moisture/Reflow Sensitivity Classification',
} as const;
