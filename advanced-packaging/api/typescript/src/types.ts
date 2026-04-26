/**
 * WIA Advanced Packaging Standard - Type Definitions
 * WIA-SEMI-003 v1.0.0
 *
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

// ============================================================================
// Base Types
// ============================================================================

export interface Position2D {
  x: number; // mm
  y: number; // mm
}

export interface Position3D extends Position2D {
  z: number; // mm or layer index
}

export interface Dimensions2D {
  width: number;  // mm
  height: number; // mm
}

export interface Dimensions3D extends Dimensions2D {
  thickness: number; // μm
}

// ============================================================================
// Material Types
// ============================================================================

export type SubstrateType = 'silicon' | 'organic' | 'glass';
export type InterposerType = 'silicon' | 'organic';
export type TechnologyNode = '3nm' | '5nm' | '7nm' | '10nm' | '14nm' | '28nm' | 'legacy';
export type CoolingType = 'passive' | 'active' | 'liquid';

export interface Material {
  name: string;
  thermalConductivity: number;    // W/m·K
  electricalResistivity: number;  // Ω·m
  cte: number;                    // ppm/°C (Coefficient of Thermal Expansion)
  density: number;                // kg/m³
}

// ============================================================================
// Die Types
// ============================================================================

export interface Die {
  id: string;
  position: Position3D;
  dimensions: Dimensions2D;
  technology: TechnologyNode;
  power: number;          // W
  maxTemperature: number; // °C
  function: string;       // e.g., 'CPU', 'GPU', 'Memory', 'IO'
  vendor?: string;
  partNumber?: string;
}

// ============================================================================
// Interposer Types
// ============================================================================

export interface Interposer {
  type: InterposerType;
  dimensions: Dimensions3D;
  layers: number;
  minFeatureSize: number;  // μm (line/space)
  viaDiameter: number;     // μm
  material: Material;
}

// ============================================================================
// TSV Types
// ============================================================================

export type TSVType = 'via-first' | 'via-middle' | 'via-last';

export interface TSV {
  id: string;
  type: TSVType;
  position: Position2D;
  diameter: number;        // μm
  depth: number;          // μm
  aspectRatio: number;    // depth/diameter
  keepOutZone: number;    // μm (radius)
  resistance: number;     // Ω
  capacitance: number;    // fF
}

// ============================================================================
// Interconnect Types
// ============================================================================

export type InterconnectType = 'microbump' | 'tsv' | 'hybrid-bonding' | 'wire-bond';

export interface Interconnect {
  id: string;
  type: InterconnectType;
  source: string;         // Die ID
  target: string;         // Die ID or substrate
  count: number;
  pitch: number;          // μm
  diameter?: number;      // μm (for bumps)
  dataRate?: number;      // Gbps per pin
  protocol?: string;      // e.g., 'PCIe', 'CXL', 'UCIe'
}

export interface MicroBump {
  pitch: number;          // μm
  diameter: number;       // μm
  height: number;         // μm
  material: 'solder' | 'copper-pillar';
  underfill: boolean;
}

export interface HybridBonding {
  pitch: number;          // μm
  padSize: number;        // μm
  surfaceRoughness: number; // nm Ra
  bondingForce: number;   // kN
  bondingTemp: number;    // °C
}

// ============================================================================
// Package 2.5D Types
// ============================================================================

export interface Package2DConfig {
  id: string;
  name: string;
  version: string;
  interposer: Interposer;
  dies: Die[];
  interconnects: Interconnect[];
  microBumps: MicroBump;
  substrate?: {
    type: SubstrateType;
    dimensions: Dimensions3D;
    layers: number;
  };
}

export interface Package2D {
  config: Package2DConfig;

  // Methods
  validate(): ValidationResult;
  simulate(): SimulationResult;
  optimize(options: OptimizationOptions): OptimizationResult;
  thermalAnalysis(): ThermalResult;
  powerAnalysis(): PowerResult;
  signalIntegrityAnalysis(): SignalIntegrityResult;
  export(format: ExportFormat): Buffer | string;
}

// ============================================================================
// Package 3D Types
// ============================================================================

export interface Package3DConfig {
  id: string;
  name: string;
  version: string;
  dies: Die[];
  tsvs: TSV[];
  bonding: HybridBonding | MicroBump;
  maxStackHeight: number;  // μm
}

export interface Package3D {
  config: Package3DConfig;

  // Methods
  validate(): ValidationResult;
  stack(): StackResult;
  thermalAnalysis(): ThermalResult;
  mechanicalAnalysis(): MechanicalResult;
  tsvAnalysis(): TSVAnalysisResult;
  export(format: ExportFormat): Buffer | string;
}

// ============================================================================
// Chiplet Types
// ============================================================================

export interface ChipletInterface {
  id: string;
  type: 'UCIe' | 'AIB' | 'BoW' | 'custom';
  position: 'north' | 'south' | 'east' | 'west';
  width: number;          // mm
  lanes: number;
  dataRate: number;       // Gbps per lane
  protocol: string[];     // ['PCIe', 'CXL', 'AXI']
  power: {
    active: number;       // mW
    idle: number;         // mW
    sleep: number;        // mW
  };
}

export interface Chiplet extends Die {
  interfaces: ChipletInterface[];
  testability: {
    boundaryScanlength: number;
    bistCoverage: number; // percentage
  };
}

// ============================================================================
// HBM Types
// ============================================================================

export interface HBMStack {
  id: string;
  generation: 'HBM2' | 'HBM2e' | 'HBM3' | 'HBM3e';
  channels: 8 | 16;
  stackHeight: 4 | 8 | 12;  // number of DRAM dies
  capacity: number;          // GB
  bandwidth: number;         // GB/s
  dataRate: number;          // Gbps
  interfaceWidth: number;    // bits per channel
  power: number;             // W
}

// ============================================================================
// Thermal Types
// ============================================================================

export interface ThermalNode {
  id: string;
  position: Position3D;
  temperature: number;       // °C
  powerDensity: number;      // W/mm²
}

export interface Hotspot {
  location: Position3D;
  temperature: number;       // °C
  area: number;             // mm²
  severity: 'low' | 'medium' | 'high' | 'critical';
}

export interface ThermalDesign {
  ambientTemperature: number;     // °C
  maxJunctionTemperature: number; // °C
  coolingType: CoolingType;
  heatSpreader?: {
    material: Material;
    thickness: number;  // mm
    area: number;      // mm²
  };
  thermalInterfaceMaterial: {
    conductivity: number;    // W/m·K
    thickness: number;       // μm
    bondLineThickness: number; // μm
  };
}

export interface ThermalResult {
  timestamp: Date;
  ambient: number;              // °C
  maxTemperature: number;       // °C
  avgTemperature: number;       // °C
  thermalGradient: number;      // °C
  nodes: ThermalNode[];
  hotspots: Hotspot[];
  thermalResistance: number;    // °C/W
  coolingCapacity: number;      // W
  warnings: string[];
}

// ============================================================================
// Power Delivery Types
// ============================================================================

export interface PowerDomain {
  name: string;
  voltage: number;        // V
  current: number;        // A
  power: number;         // W
  tolerance: number;     // %
  ripple: number;        // mV
}

export interface PowerDistributionNetwork {
  domains: PowerDomain[];
  onDieCapacitance: number;   // nF
  packageCapacitance: number; // nF
  boardCapacitance: number;   // μF
  targetImpedance: number;    // mΩ
}

export interface PowerResult {
  totalPower: number;         // W
  irDrop: number;            // mV
  impedance: number;         // mΩ
  efficiency: number;        // %
  domains: PowerDomain[];
  warnings: string[];
}

// ============================================================================
// Signal Integrity Types
// ============================================================================

export interface SignalPath {
  id: string;
  source: string;
  destination: string;
  length: number;           // mm
  impedance: number;        // Ω
  dataRate: number;         // Gbps
}

export interface SignalIntegrityResult {
  paths: SignalPath[];
  insertionLoss: number;    // dB
  returnLoss: number;       // dB
  crosstalk: number;        // dB
  jitter: number;           // ps
  eyeHeight: number;        // mV
  eyeWidth: number;         // ps
  ber: number;              // Bit Error Rate
  passed: boolean;
  warnings: string[];
}

// ============================================================================
// Simulation & Analysis Types
// ============================================================================

export interface ValidationResult {
  valid: boolean;
  errors: string[];
  warnings: string[];
  timestamp: Date;
}

export interface SimulationResult {
  success: boolean;
  runtime: number;          // seconds
  thermal: ThermalResult;
  power: PowerResult;
  signalIntegrity: SignalIntegrityResult;
  timestamp: Date;
}

export interface StackResult {
  totalHeight: number;      // μm
  alignment: number;        // μm
  warpage: number;         // μm
  success: boolean;
  warnings: string[];
}

export interface MechanicalResult {
  stress: number;          // MPa
  strain: number;          // %
  warpage: number;         // μm
  thermalCyclingResult: {
    cycles: number;
    passed: boolean;
  };
  warnings: string[];
}

export interface TSVAnalysisResult {
  totalTSVs: number;
  resistance: number;       // Ω (average)
  capacitance: number;      // fF (average)
  bandwidth: number;        // TB/s
  latency: number;          // ns
  reliability: number;      // %
  warnings: string[];
}

// ============================================================================
// Optimization Types
// ============================================================================

export type OptimizationTarget = 'performance' | 'power' | 'thermal' | 'cost' | 'balanced';

export interface OptimizationOptions {
  target: OptimizationTarget;
  maxIterations?: number;
  tolerance?: number;
  constraints?: {
    maxPower?: number;        // W
    maxTemperature?: number;  // °C
    maxCost?: number;         // USD
    maxArea?: number;         // mm²
  };
}

export interface OptimizationResult {
  success: boolean;
  iterations: number;
  improvement: number;      // %
  originalScore: number;
  optimizedScore: number;
  changes: string[];
  warnings: string[];
}

// ============================================================================
// Export Types
// ============================================================================

export type ExportFormat = 'JSON' | 'GDSII' | 'LEF' | 'DEF' | 'OASIS';

export interface ExportOptions {
  format: ExportFormat;
  includeMetadata?: boolean;
  compression?: boolean;
  version?: string;
}

// ============================================================================
// Manufacturing Types
// ============================================================================

export interface ManufacturingRules {
  minFeatureSize: number;      // μm
  minSpacing: number;          // μm
  minViaSize: number;          // μm
  maxAspectRatio: number;
  alignmentTolerance: number;  // μm
  waferThickness: number;      // μm
  dieYield: number;           // %
}

export interface DFMCheck {
  passed: boolean;
  violations: {
    rule: string;
    severity: 'error' | 'warning' | 'info';
    location: Position3D;
    description: string;
  }[];
}

// ============================================================================
// Testing & Reliability Types
// ============================================================================

export interface ReliabilityTest {
  name: string;
  type: 'thermal-cycling' | 'thb' | 'hts' | 'mechanical' | 'electrical';
  conditions: {
    temperature?: number | [number, number];  // °C or range
    humidity?: number;                        // %
    duration?: number;                        // hours
    cycles?: number;
  };
  result: {
    passed: boolean;
    failureMode?: string;
    mtbf?: number;  // hours (Mean Time Between Failures)
  };
}

export interface TestResult {
  continuity: boolean;
  resistance: { min: number; max: number; avg: number }; // Ω
  capacitance: { min: number; max: number; avg: number }; // pF
  highSpeedTest: {
    dataRate: number;  // Gbps
    ber: number;       // Bit Error Rate
    passed: boolean;
  };
  reliability: ReliabilityTest[];
  timestamp: Date;
}
