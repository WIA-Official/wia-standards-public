/**
 * WIA-RUST-ADVANCED: Advanced Rust Programming Standard - TypeScript Type Definitions
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Lifetime Management Types
// ============================================================================

export enum LifetimeBound {
  STATIC = 'STATIC',
  NAMED = 'NAMED',
  ANONYMOUS = 'ANONYMOUS',
  ELIDED = 'ELIDED'
}

export interface LifetimeScope {
  name: string;
  bound: LifetimeBound;
  parentScope?: string;
  startLine: number;
  endLine: number;
  references: LifetimeReference[];
}

export interface LifetimeReference {
  variableName: string;
  lifetime: string;
  isMutable: boolean;
  location: CodeLocation;
}

export interface BorrowChecker {
  borrowId: string;
  borrowType: 'SHARED' | 'MUTABLE' | 'OWNERSHIP_TRANSFER';
  lifetime: string;
  startLocation: CodeLocation;
  endLocation: CodeLocation;
  violations: BorrowViolation[];
}

export interface BorrowViolation {
  violationType: 'MULTIPLE_MUTABLE_BORROWS' | 'MUTABLE_AND_IMMUTABLE_BORROW' | 'USE_AFTER_MOVE' | 'DANGLING_REFERENCE';
  message: string;
  location: CodeLocation;
  suggestion: string;
}

export interface OwnershipTransfer {
  fromVariable: string;
  toVariable: string;
  transferType: 'MOVE' | 'COPY' | 'CLONE';
  location: CodeLocation;
}

// ============================================================================
// Async/Await Patterns
// ============================================================================

export enum AsyncRuntimeType {
  TOKIO = 'TOKIO',
  ASYNC_STD = 'ASYNC_STD',
  SMOL = 'SMOL',
  CUSTOM = 'CUSTOM'
}

export interface AsyncRuntime {
  runtimeType: AsyncRuntimeType;
  threadPoolSize: number;
  schedulerType: 'MULTI_THREAD' | 'CURRENT_THREAD' | 'CUSTOM';
  enableIO: boolean;
  enableTime: boolean;
  configuration: Record<string, any>;
}

export enum FutureState {
  PENDING = 'PENDING',
  READY = 'READY',
  COMPLETED = 'COMPLETED',
  CANCELLED = 'CANCELLED'
}

export interface FutureHandle {
  futureId: string;
  state: FutureState;
  pollCount: number;
  createdAt: number;
  completedAt?: number;
  wakerCount: number;
}

export interface WakerHandle {
  wakerId: string;
  futureId: string;
  wakeCount: number;
  associatedTask: string;
}

export interface AsyncTask {
  taskId: string;
  name: string;
  priority: number;
  future: FutureHandle;
  spawner: string;
  dependencies: string[];
}

// ============================================================================
// Unsafe Code Patterns
// ============================================================================

export enum UnsafeOperation {
  RAW_POINTER_DEREFERENCE = 'RAW_POINTER_DEREFERENCE',
  FFI_CALL = 'FFI_CALL',
  MUTABLE_STATIC_ACCESS = 'MUTABLE_STATIC_ACCESS',
  UNSAFE_TRAIT_IMPL = 'UNSAFE_TRAIT_IMPL',
  INLINE_ASSEMBLY = 'INLINE_ASSEMBLY',
  UNION_FIELD_ACCESS = 'UNION_FIELD_ACCESS'
}

export interface UnsafeBlock {
  blockId: string;
  operation: UnsafeOperation;
  location: CodeLocation;
  justification: string;
  invariants: string[];
  soundnessProof?: string;
  reviewStatus: 'UNREVIEWED' | 'REVIEWED' | 'APPROVED' | 'FLAGGED';
}

export interface RawPointer {
  pointerId: string;
  pointerType: '*const' | '*mut';
  targetType: string;
  alignment: number;
  isNull: boolean;
  provenance?: string;
}

export interface FFIBinding {
  functionName: string;
  libraryName: string;
  signature: FFISignature;
  safetyInvariants: string[];
  linkage: 'STATIC' | 'DYNAMIC';
  abi: 'C' | 'Rust' | 'System' | 'Cdecl' | 'Stdcall' | 'Fastcall';
}

export interface FFISignature {
  parameters: FFIParameter[];
  returnType: string;
  isVariadic: boolean;
}

export interface FFIParameter {
  name: string;
  type: string;
  isPointer: boolean;
  ownership: 'BORROWED' | 'OWNED' | 'SHARED';
}

// ============================================================================
// Macro System Types
// ============================================================================

export enum MacroType {
  DECLARATIVE = 'DECLARATIVE',
  PROCEDURAL_DERIVE = 'PROCEDURAL_DERIVE',
  PROCEDURAL_ATTRIBUTE = 'PROCEDURAL_ATTRIBUTE',
  PROCEDURAL_FUNCTION_LIKE = 'PROCEDURAL_FUNCTION_LIKE'
}

export interface DeclarativeMacro {
  name: string;
  patterns: MacroPattern[];
  hygiene: 'HYGIENIC' | 'NON_HYGIENIC';
  location: CodeLocation;
}

export interface MacroPattern {
  matcher: string;
  transcriber: string;
  repetitions: MacroRepetition[];
}

export interface MacroRepetition {
  separator?: string;
  operator: '*' | '+' | '?';
  captured: string[];
}

export interface ProcMacro {
  name: string;
  macroType: MacroType;
  inputTokenStream: string;
  outputTokenStream: string;
  dependencies: string[];
}

export interface MacroExpansion {
  expansionId: string;
  macroName: string;
  input: string;
  output: string;
  expansionDepth: number;
  expansionTime: number;
  warnings: MacroWarning[];
}

export interface MacroWarning {
  message: string;
  location: CodeLocation;
  severity: 'WARNING' | 'ERROR';
}

// ============================================================================
// Systems Programming Types
// ============================================================================

export interface MemoryLayout {
  size: number;
  alignment: number;
  fields: FieldLayout[];
  padding: number;
  totalSize: number;
}

export interface FieldLayout {
  name: string;
  offset: number;
  size: number;
  alignment: number;
  type: string;
}

export enum AllocatorStrategy {
  SYSTEM_ALLOCATOR = 'SYSTEM_ALLOCATOR',
  JEMALLOC = 'JEMALLOC',
  MIMALLOC = 'MIMALLOC',
  CUSTOM = 'CUSTOM',
  ARENA = 'ARENA',
  BUMP = 'BUMP'
}

export interface AllocatorConfig {
  strategy: AllocatorStrategy;
  initialCapacity?: number;
  maxCapacity?: number;
  enableProfiling: boolean;
  enableStatistics: boolean;
}

export interface CacheOptimization {
  cacheLineSize: number;
  prefetchDistance: number;
  alignmentStrategy: 'CACHE_LINE' | 'PAGE' | 'NATURAL';
  spatialLocality: number;
  temporalLocality: number;
}

export interface SIMDOperation {
  operationType: 'ADD' | 'SUB' | 'MUL' | 'DIV' | 'FMA' | 'SHUFFLE' | 'BROADCAST';
  vectorWidth: 128 | 256 | 512;
  dataType: 'i8' | 'i16' | 'i32' | 'i64' | 'f32' | 'f64';
  intrinsic: string;
  throughput: number;
}

// ============================================================================
// Concurrency Patterns
// ============================================================================

export enum AtomicOrdering {
  RELAXED = 'RELAXED',
  ACQUIRE = 'ACQUIRE',
  RELEASE = 'RELEASE',
  ACQ_REL = 'ACQ_REL',
  SEQ_CST = 'SEQ_CST'
}

export interface AtomicOperation {
  operationType: 'LOAD' | 'STORE' | 'SWAP' | 'CAS' | 'FETCH_ADD' | 'FETCH_SUB';
  ordering: AtomicOrdering;
  variableName: string;
  location: CodeLocation;
}

export interface LockFreeStructure {
  structureType: 'QUEUE' | 'STACK' | 'LIST' | 'HASH_MAP' | 'TREE';
  algorithm: string;
  progressGuarantee: 'WAIT_FREE' | 'LOCK_FREE' | 'OBSTRUCTION_FREE';
  memoryOrdering: AtomicOrdering[];
  hazardPointers: boolean;
}

export interface ThreadPoolConfig {
  name: string;
  minThreads: number;
  maxThreads: number;
  queueSize: number;
  stackSize: number;
  threadNamePrefix: string;
  panicHandler?: string;
}

export interface MutexAnalysis {
  mutexId: string;
  contentionRate: number;
  averageHoldTime: number;
  maxHoldTime: number;
  deadlockPotential: 'NONE' | 'LOW' | 'MEDIUM' | 'HIGH';
  poisonCount: number;
}

export interface ChannelConfig {
  channelType: 'MPSC' | 'MPMC' | 'SPSC' | 'BROADCAST';
  bounded: boolean;
  capacity?: number;
  blocking: boolean;
}

// ============================================================================
// Performance Profiling Types
// ============================================================================

export interface FlameGraph {
  root: FlameGraphNode;
  totalSamples: number;
  samplingFrequency: number;
  duration: number;
  metadata: Record<string, any>;
}

export interface FlameGraphNode {
  name: string;
  value: number;
  percentage: number;
  children: FlameGraphNode[];
  selfTime: number;
  totalTime: number;
}

export interface MemoryProfile {
  peakUsage: number;
  currentUsage: number;
  allocations: AllocationProfile[];
  deallocations: number;
  leaks: MemoryLeak[];
  fragmentationRatio: number;
}

export interface AllocationProfile {
  allocator: string;
  size: number;
  count: number;
  stackTrace: string[];
  timestamp: number;
}

export interface MemoryLeak {
  size: number;
  allocatedAt: number;
  stackTrace: string[];
  type: string;
}

export interface BenchmarkResult {
  benchmarkName: string;
  iterations: number;
  meanTime: number;
  stdDev: number;
  minTime: number;
  maxTime: number;
  throughput?: number;
  allocations?: number;
  cpuCycles?: number;
}

export interface PerformanceMetrics {
  cpuUsage: number;
  memoryUsage: number;
  throughput: number;
  latency: LatencyMetrics;
  cacheMetrics: CacheMetrics;
  branchPrediction: BranchPredictionMetrics;
}

export interface LatencyMetrics {
  p50: number;
  p95: number;
  p99: number;
  p999: number;
  max: number;
}

export interface CacheMetrics {
  l1Hits: number;
  l1Misses: number;
  l2Hits: number;
  l2Misses: number;
  l3Hits: number;
  l3Misses: number;
  missRate: number;
}

export interface BranchPredictionMetrics {
  predictedBranches: number;
  mispredictedBranches: number;
  mispredictionRate: number;
}

// ============================================================================
// Code Analysis Types
// ============================================================================

export interface CodeLocation {
  file: string;
  line: number;
  column: number;
  length?: number;
}

export interface CompilationUnit {
  crateName: string;
  version: string;
  edition: '2015' | '2018' | '2021' | '2024';
  features: string[];
  dependencies: CrateDependency[];
}

export interface CrateDependency {
  name: string;
  version: string;
  features: string[];
  optional: boolean;
}

// ============================================================================
// Error Types
// ============================================================================

export class RustAdvancedError extends Error {
  constructor(
    message: string,
    public code: string,
    public location?: CodeLocation,
    public context?: Record<string, any>
  ) {
    super(message);
    this.name = 'RustAdvancedError';
  }
}

export class LifetimeError extends RustAdvancedError {
  constructor(message: string, location?: CodeLocation) {
    super(message, 'LIFETIME_ERROR', location);
    this.name = 'LifetimeError';
  }
}

export class BorrowCheckError extends RustAdvancedError {
  constructor(message: string, location?: CodeLocation) {
    super(message, 'BORROW_CHECK_ERROR', location);
    this.name = 'BorrowCheckError';
  }
}

export class UnsafetyError extends RustAdvancedError {
  constructor(message: string, location?: CodeLocation) {
    super(message, 'UNSAFETY_ERROR', location);
    this.name = 'UnsafetyError';
  }
}

export class ConcurrencyError extends RustAdvancedError {
  constructor(message: string, location?: CodeLocation) {
    super(message, 'CONCURRENCY_ERROR', location);
    this.name = 'ConcurrencyError';
  }
}

/**
 * 弘益人間 (Benefit All Humanity)
 *
 * These type definitions support the development of high-performance,
 * safe, and concurrent systems using advanced Rust programming patterns.
 */
