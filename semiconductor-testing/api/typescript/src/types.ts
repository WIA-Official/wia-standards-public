/**
 * WIA-SEMI-002: Semiconductor Testing Standard
 * TypeScript Type Definitions
 *
 * @module @wia/semiconductor-testing/types
 * @version 1.0.0
 */

// ============================================================================
// Wafer Testing Types
// ============================================================================

/**
 * Wafer test configuration
 */
export interface WaferTest {
  /** Unique wafer identifier */
  waferId: string;

  /** Lot identifier */
  lotId: string;

  /** Device type being tested */
  deviceType: string;

  /** Wafer size in millimeters */
  waferSize: 200 | 300 | 450;

  /** Wafer orientation notch/flat */
  orientation: 'flat' | 'notch';

  /** Die dimensions */
  dieSize: {
    width: number;  // in mm
    height: number; // in mm
  };

  /** Probe card identifier */
  probeCard: string;

  /** Test program identifier */
  testProgram: string;

  /** Test mode */
  mode: 'parametric' | 'functional' | 'full';

  /** Optional environmental conditions */
  conditions?: TestConditions;
}

/**
 * Wafer test results
 */
export interface WaferTestResults {
  /** Wafer identifier */
  waferId: string;

  /** Test timestamp */
  timestamp: Date;

  /** Test duration in milliseconds */
  duration: number;

  /** Array of die results */
  dies: DieResult[];

  /** Overall statistics */
  statistics: WaferStatistics;

  /** Wafer map data */
  waferMap: WaferMap;
}

/**
 * Individual die test result
 */
export interface DieResult {
  /** Die X coordinate */
  x: number;

  /** Die Y coordinate */
  y: number;

  /** Bin classification */
  binCode: number;

  /** Test time in milliseconds */
  testTime: number;

  /** Parametric test results */
  parametricResults?: ParametricResult[];

  /** Functional test results */
  functionalResults?: FunctionalResult[];

  /** Failure information if bin 0 */
  failures?: FailureInfo[];
}

/**
 * Parametric test measurement
 */
export interface ParametricResult {
  /** Test name */
  name: string;

  /** Measured value */
  value: number;

  /** Unit of measurement */
  unit: string;

  /** Lower specification limit */
  lowerLimit?: number;

  /** Upper specification limit */
  upperLimit?: number;

  /** Pass/fail status */
  pass: boolean;
}

/**
 * Functional test result
 */
export interface FunctionalResult {
  /** Test name */
  name: string;

  /** Pattern count */
  patternCount: number;

  /** Failing patterns */
  failingPatterns: number;

  /** Pass/fail status */
  pass: boolean;
}

/**
 * Failure information
 */
export interface FailureInfo {
  /** Failure category */
  category: 'parametric' | 'functional' | 'continuity' | 'other';

  /** Test name that failed */
  testName: string;

  /** Failure description */
  description: string;

  /** Failure code */
  code?: number;
}

/**
 * Wafer statistics
 */
export interface WaferStatistics {
  /** Total dies tested */
  totalDies: number;

  /** Good dies (bin 1) */
  goodDies: number;

  /** Partial dies (bin 2-8) */
  partialDies: number;

  /** Failed dies (bin 0) */
  failedDies: number;

  /** Yield percentage */
  yield: number;

  /** Average test time per die in milliseconds */
  averageTestTime: number;

  /** Bin distribution */
  binDistribution: Record<number, number>;
}

/**
 * Wafer map representation
 */
export interface WaferMap {
  /** Wafer identifier */
  waferId: string;

  /** Wafer size in mm */
  waferSize: number;

  /** Die size */
  dieSize: { width: number; height: number };

  /** 2D array of die bin codes */
  map: number[][];

  /** Metadata */
  metadata: {
    createdAt: Date;
    testProgram: string;
    equipment: string;
  };
}

// ============================================================================
// Package Testing Types
// ============================================================================

/**
 * Package test configuration
 */
export interface PackageTest {
  /** Package type */
  packageType: 'QFN' | 'BGA' | 'FCBGA' | 'WLCSP' | 'DIP' | 'QFP' | 'other';

  /** Device serial number */
  serialNumber: string;

  /** Lot identifier */
  lotId: string;

  /** Device type */
  deviceType: string;

  /** Test program */
  testProgram: string;

  /** Test conditions */
  conditions: TestConditions;

  /** Burn-in configuration */
  burnIn?: BurnInConfig;
}

/**
 * Package test results
 */
export interface PackageTestResults {
  /** Serial number */
  serialNumber: string;

  /** Test timestamp */
  timestamp: Date;

  /** Test duration */
  duration: number;

  /** Overall pass/fail */
  pass: boolean;

  /** Bin code */
  binCode: number;

  /** Continuity test results */
  continuity: ContinuityResult;

  /** Parametric results */
  parametric: ParametricResult[];

  /** Functional results */
  functional: FunctionalResult[];

  /** Burn-in results if applicable */
  burnInResults?: BurnInResults;
}

/**
 * Continuity test results
 */
export interface ContinuityResult {
  /** All pins connected */
  allPinsConnected: boolean;

  /** Failed pins */
  failedPins: number[];

  /** Pin resistance measurements */
  pinResistance: Record<number, number>;

  /** Isolation test results */
  isolation: boolean;
}

/**
 * Burn-in configuration
 */
export interface BurnInConfig {
  /** Temperature in Celsius */
  temperature: number;

  /** Voltage (percentage of nominal) */
  voltage: number;

  /** Duration in hours */
  duration: number;

  /** Test pattern */
  pattern: string;

  /** Monitoring interval in minutes */
  monitoringInterval: number;
}

/**
 * Burn-in test results
 */
export interface BurnInResults {
  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Pass/fail status */
  pass: boolean;

  /** Failure time if failed */
  failureTime?: Date;

  /** Monitoring snapshots */
  snapshots: BurnInSnapshot[];
}

/**
 * Burn-in monitoring snapshot
 */
export interface BurnInSnapshot {
  /** Snapshot timestamp */
  timestamp: Date;

  /** Temperature reading */
  temperature: number;

  /** Voltage reading */
  voltage: number;

  /** Current consumption */
  current: number;

  /** Device status */
  status: 'operating' | 'failed' | 'degraded';
}

// ============================================================================
// Test Conditions
// ============================================================================

/**
 * Environmental test conditions
 */
export interface TestConditions {
  /** Temperature in Celsius */
  temperature: number;

  /** Supply voltage in volts */
  voltage: number;

  /** Test frequency in MHz */
  frequency?: number;

  /** Temperature corner */
  tempCorner?: 'min' | 'nominal' | 'max';

  /** Voltage corner */
  voltageCorner?: 'min' | 'nominal' | 'max';
}

// ============================================================================
// ATE Configuration
// ============================================================================

/**
 * ATE equipment configuration
 */
export interface ATEConfig {
  /** Equipment identifier */
  equipmentId: string;

  /** Equipment model */
  model: 'ADVANTEST-T2000' | 'ADVANTEST-V93000' | 'TERADYNE-ULTRAFLEX' | 'TERADYNE-J750' | string;

  /** Network address */
  ipAddress: string;

  /** Network port */
  port: number;

  /** Number of test sites */
  sites: number;

  /** Authentication credentials */
  authentication?: {
    username: string;
    password: string;
    token?: string;
  };
}

/**
 * ATE status information
 */
export interface ATEStatus {
  /** Equipment ID */
  equipmentId: string;

  /** Connection status */
  connected: boolean;

  /** Operational status */
  status: 'idle' | 'testing' | 'calibrating' | 'error' | 'offline';

  /** Active test sites */
  activeSites: number[];

  /** Equipment temperature */
  temperature: number;

  /** Last calibration date */
  lastCalibration?: Date;

  /** Uptime percentage */
  uptime: number;

  /** Error messages if any */
  errors?: string[];
}

/**
 * ATE session
 */
export interface ATESession {
  /** Session identifier */
  sessionId: string;

  /** Equipment configuration */
  equipment: ATEConfig;

  /** Session start time */
  startTime: Date;

  /** Current status */
  status: ATEStatus;

  /** Disconnect function */
  disconnect(): Promise<void>;

  /** Load test program */
  loadProgram(programId: string): Promise<void>;

  /** Execute test */
  executeTest(config: WaferTest | PackageTest): Promise<WaferTestResults | PackageTestResults>;

  /** Get real-time status */
  getStatus(): Promise<ATEStatus>;
}

// ============================================================================
// Quality Metrics
// ============================================================================

/**
 * Quality metrics summary
 */
export interface QualityMetrics {
  /** Time period for metrics */
  period: {
    start: Date;
    end: Date;
  };

  /** Wafer probe yield */
  waferYield: number;

  /** Final test yield */
  finalYield: number;

  /** Cumulative yield */
  cumulativeYield: number;

  /** Defects per million */
  dpm: number;

  /** Equipment uptime */
  uptime: number;

  /** Throughput (wafers/day) */
  throughput: number;

  /** Bin distribution */
  binDistribution: Record<number, number>;

  /** Trend analysis */
  trend: 'improving' | 'stable' | 'degrading';
}

/**
 * Analytics configuration
 */
export interface AnalyticsConfig {
  /** Lot ID to analyze */
  lotId?: string;

  /** Wafer IDs to analyze */
  waferIds?: string[];

  /** Date range */
  dateRange?: {
    start: Date;
    end: Date;
  };

  /** Metrics to calculate */
  metrics: ('yield' | 'dpm' | 'uptime' | 'throughput' | 'bin_distribution')[];

  /** Include trend analysis */
  includeTrend?: boolean;
}

// ============================================================================
// Data Streaming
// ============================================================================

/**
 * Real-time test result event
 */
export interface TestResultEvent {
  /** Event type */
  type: 'die_complete' | 'wafer_complete' | 'device_complete';

  /** Timestamp */
  timestamp: Date;

  /** Result data */
  data: DieResult | WaferTestResults | PackageTestResults;
}

/**
 * Streaming callback function
 */
export type StreamCallback = (event: TestResultEvent) => void;

// ============================================================================
// Export format options
// ============================================================================

/**
 * Data export formats
 */
export type ExportFormat = 'STDF' | 'CSV' | 'JSON' | 'XML';

/**
 * Export configuration
 */
export interface ExportConfig {
  /** Output format */
  format: ExportFormat;

  /** Include raw data */
  includeRaw?: boolean;

  /** Include statistics */
  includeStats?: boolean;

  /** Compression */
  compress?: boolean;
}
