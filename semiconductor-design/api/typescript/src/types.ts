/**
 * WIA-SEMI-001: Semiconductor Design Standard
 * TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

// ============================================================================
// RTL Design Types
// ============================================================================

/**
 * RTL Design configuration
 */
export interface DesignConfig {
  name: string;
  standard: 'WIA-SEMI-001';
  version: string;
  description?: string;
  author?: string;
  date?: Date;
}

/**
 * Hardware Description Language
 */
export type HDL = 'verilog' | 'vhdl' | 'systemverilog';

/**
 * Signal direction
 */
export type Direction = 'input' | 'output' | 'inout';

/**
 * Signal definition
 */
export interface Signal {
  name: string;
  direction: Direction;
  width: number;
  signed?: boolean;
  description?: string;
}

/**
 * Port definition
 */
export interface Port extends Signal {
  clock?: boolean;
  reset?: boolean;
  active_low?: boolean;
}

/**
 * Parameter definition
 */
export interface Parameter {
  name: string;
  type: 'integer' | 'string' | 'real';
  default_value: number | string;
  description?: string;
}

/**
 * RTL Module
 */
export interface Module {
  name: string;
  ports: Port[];
  parameters: Parameter[];
  instances?: ModuleInstance[];
  language: HDL;
  source_file?: string;
}

/**
 * Module instance
 */
export interface ModuleInstance {
  instance_name: string;
  module_name: string;
  parameter_overrides?: Record<string, number | string>;
  port_connections?: Record<string, string>;
}

/**
 * Complete RTL Design
 */
export interface RTLDesign {
  config: DesignConfig;
  top_module: Module;
  modules: Module[];
  metadata?: DesignMetadata;
}

/**
 * Design metadata
 */
export interface DesignMetadata {
  total_modules: number;
  total_instances: number;
  total_signals: number;
  hierarchy_depth: number;
  estimated_gates?: number;
  critical_path?: string;
}

// ============================================================================
// Verification Types
// ============================================================================

/**
 * Verification methodology
 */
export type VerificationMethod =
  | 'directed'
  | 'random'
  | 'constrained_random'
  | 'uvm'
  | 'formal';

/**
 * Verification configuration
 */
export interface VerificationConfig {
  design: RTLDesign;
  methodology: VerificationMethod;
  coverage: CoverageConfig;
  assertions?: boolean;
  seed?: number;
}

/**
 * Coverage configuration
 */
export interface CoverageConfig {
  functional: boolean;
  code: boolean;
  toggle: boolean;
  fsm: boolean;
  assertion?: boolean;
}

/**
 * Testbench configuration
 */
export interface TestBench {
  dut: Module;
  clock_period: number; // in ns
  reset_cycles: number;
  stimulus: 'directed' | 'directed_random' | 'constrained_random';
  timeout_cycles?: number;
}

/**
 * Simulation results
 */
export interface SimulationResult {
  passed: boolean;
  total_tests: number;
  passed_tests: number;
  failed_tests: number;
  coverage: CoverageReport;
  errors?: string[];
  warnings?: string[];
  simulation_time: number; // in seconds
}

/**
 * Coverage report
 */
export interface CoverageReport {
  code_coverage: number; // percentage
  functional_coverage: number;
  toggle_coverage: number;
  fsm_coverage: number;
  assertion_coverage?: number;
  details?: CoverageDetails;
}

/**
 * Detailed coverage information
 */
export interface CoverageDetails {
  total_lines: number;
  covered_lines: number;
  total_branches: number;
  covered_branches: number;
  total_toggles: number;
  covered_toggles: number;
}

// ============================================================================
// Synthesis Types
// ============================================================================

/**
 * Target technology
 */
export type Technology =
  | 'TSMC_5nm'
  | 'TSMC_7nm'
  | 'TSMC_28nm'
  | 'Samsung_5nm'
  | 'Samsung_7nm'
  | 'Intel_14nm'
  | 'generic';

/**
 * Optimization goal
 */
export type OptimizationGoal = 'area' | 'power' | 'performance' | 'balanced';

/**
 * Synthesis constraints
 */
export interface SynthesisConstraints {
  clock_frequency: number; // MHz
  clock_uncertainty?: number; // ns
  input_delay?: number; // ns
  output_delay?: number; // ns
  power_budget?: number; // mW
  area_constraint?: number; // mm²
  false_paths?: string[];
  multicycle_paths?: MulticyclePath[];
}

/**
 * Multi-cycle path definition
 */
export interface MulticyclePath {
  from: string;
  to: string;
  cycles: number;
}

/**
 * Synthesis configuration
 */
export interface SynthesisConfig {
  design: RTLDesign;
  technology: Technology;
  constraints: SynthesisConstraints;
  optimization: OptimizationGoal;
  clock_gating?: boolean;
  retiming?: boolean;
  resource_sharing?: boolean;
}

/**
 * Gate-level netlist
 */
export interface Netlist {
  design_name: string;
  technology: Technology;
  modules: NetlistModule[];
  metadata: NetlistMetadata;
}

/**
 * Netlist module
 */
export interface NetlistModule {
  name: string;
  cells: Cell[];
  nets: Net[];
}

/**
 * Standard cell instance
 */
export interface Cell {
  instance_name: string;
  cell_type: string;
  library: string;
  area?: number;
  power?: number;
}

/**
 * Net (wire connection)
 */
export interface Net {
  name: string;
  driver: string;
  loads: string[];
  capacitance?: number;
}

/**
 * Netlist metadata
 */
export interface NetlistMetadata {
  total_cells: number;
  total_nets: number;
  area: number; // µm²
  power: number; // mW
  max_frequency: number; // MHz
  timing_met: boolean;
}

/**
 * Synthesis results
 */
export interface SynthesisResult {
  netlist: Netlist;
  timing_report: TimingReport;
  power_report: PowerReport;
  area_report: AreaReport;
  qor: QualityOfResults;
}

/**
 * Timing report
 */
export interface TimingReport {
  setup_slack: number; // ns
  hold_slack: number; // ns
  max_frequency: number; // MHz
  critical_path: TimingPath;
  worst_paths: TimingPath[];
}

/**
 * Timing path
 */
export interface TimingPath {
  startpoint: string;
  endpoint: string;
  delay: number; // ns
  slack: number; // ns
  path_type: 'setup' | 'hold';
}

/**
 * Power report
 */
export interface PowerReport {
  total_power: number; // mW
  dynamic_power: number; // mW
  static_power: number; // mW
  breakdown: PowerBreakdown;
}

/**
 * Power breakdown by component
 */
export interface PowerBreakdown {
  clock_network: number;
  registers: number;
  combinational: number;
  io: number;
  leakage: number;
}

/**
 * Area report
 */
export interface AreaReport {
  total_area: number; // µm²
  combinational_area: number;
  sequential_area: number;
  net_area: number;
  cell_count: number;
  breakdown: AreaBreakdown;
}

/**
 * Area breakdown by cell type
 */
export interface AreaBreakdown {
  logic: number;
  registers: number;
  buffers: number;
  clock_gates: number;
  other: number;
}

/**
 * Quality of Results metrics
 */
export interface QualityOfResults {
  ppa_score: number; // Combined metric (0-100)
  timing_score: number;
  power_score: number;
  area_score: number;
  congestion: number; // percentage
  utilization: number; // percentage
}

// ============================================================================
// Physical Design Types
// ============================================================================

/**
 * Floorplan configuration
 */
export interface FloorplanConfig {
  die_width: number; // µm
  die_height: number; // µm
  core_utilization: number; // 0.0 - 1.0
  aspect_ratio: number;
  io_ring?: boolean;
  power_stripes?: PowerStripesConfig;
}

/**
 * Power stripes configuration
 */
export interface PowerStripesConfig {
  vdd_stripes: number;
  vss_stripes: number;
  stripe_width: number; // µm
  spacing: number; // µm
}

/**
 * Placement configuration
 */
export interface PlacementConfig {
  timing_driven: boolean;
  congestion_driven: boolean;
  power_driven: boolean;
  effort: 'low' | 'medium' | 'high';
}

/**
 * Routing configuration
 */
export interface RoutingConfig {
  layers: number;
  min_layer?: number;
  max_layer?: number;
  via_optimization: boolean;
  crosstalk_optimization: boolean;
}

/**
 * Physical design result
 */
export interface PhysicalDesignResult {
  floorplan: FloorplanData;
  placement: PlacementData;
  routing: RoutingData;
  sign_off: SignOffResults;
}

/**
 * Floorplan data
 */
export interface FloorplanData {
  die_area: number; // µm²
  core_area: number; // µm²
  utilization: number;
  macros: MacroPlacement[];
  io_pads: IOPad[];
}

/**
 * Macro placement
 */
export interface MacroPlacement {
  name: string;
  x: number;
  y: number;
  width: number;
  height: number;
  orientation: 'N' | 'S' | 'E' | 'W' | 'FN' | 'FS' | 'FE' | 'FW';
}

/**
 * IO pad
 */
export interface IOPad {
  name: string;
  type: 'input' | 'output' | 'power' | 'ground';
  location: 'north' | 'south' | 'east' | 'west';
  position: number;
}

/**
 * Placement data
 */
export interface PlacementData {
  total_cells: number;
  placed_cells: number;
  overflow: number;
  hpwl: number; // Half-Perimeter Wire Length
}

/**
 * Routing data
 */
export interface RoutingData {
  total_nets: number;
  routed_nets: number;
  vias: number;
  wire_length: number; // µm
  overflow: number;
  drc_violations: number;
}

/**
 * Sign-off results
 */
export interface SignOffResults {
  sta: StaticTimingAnalysis;
  power: PowerAnalysis;
  drc: DesignRuleCheck;
  lvs: LayoutVsSchematic;
  antenna: AntennaCheck;
  erc: ElectricalRuleCheck;
  ready_for_tapeout: boolean;
}

/**
 * Static Timing Analysis
 */
export interface StaticTimingAnalysis {
  setup_violations: number;
  hold_violations: number;
  worst_slack: number; // ns
  total_endpoints: number;
  met: boolean;
}

/**
 * Power Analysis
 */
export interface PowerAnalysis {
  ir_drop_max: number; // percentage
  ir_drop_violations: number;
  em_violations: number;
  met: boolean;
}

/**
 * Design Rule Check
 */
export interface DesignRuleCheck {
  total_violations: number;
  spacing_violations: number;
  width_violations: number;
  enclosure_violations: number;
  met: boolean;
}

/**
 * Layout vs Schematic
 */
export interface LayoutVsSchematic {
  devices_match: boolean;
  nets_match: boolean;
  total_errors: number;
  met: boolean;
}

/**
 * Antenna Check
 */
export interface AntennaCheck {
  violations: number;
  critical_nets: string[];
  met: boolean;
}

/**
 * Electrical Rule Check
 */
export interface ElectricalRuleCheck {
  total_violations: number;
  short_violations: number;
  open_violations: number;
  met: boolean;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Design format for export
 */
export type ExportFormat =
  | 'verilog'
  | 'vhdl'
  | 'systemverilog'
  | 'json'
  | 'gdsii'
  | 'def'
  | 'lef';

/**
 * Log level for reporting
 */
export type LogLevel = 'debug' | 'info' | 'warning' | 'error';

/**
 * Error type
 */
export interface DesignError {
  level: LogLevel;
  message: string;
  location?: SourceLocation;
  timestamp: Date;
}

/**
 * Source code location
 */
export interface SourceLocation {
  file: string;
  line: number;
  column?: number;
}

/**
 * Design statistics
 */
export interface DesignStatistics {
  modules: number;
  instances: number;
  signals: number;
  gates: number;
  registers: number;
  area: number;
  power: number;
  frequency: number;
}
