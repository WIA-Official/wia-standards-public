/**
 * WIA-RUST-INTERMEDIATE - TypeScript Type Definitions
 * Comprehensive types for intermediate Rust programming concepts
 */

/**
 * Error handling types
 */

/** Represents Rust's Result<T, E> type */
export interface Result<T, E> {
  ok?: T;
  err?: E;
  isOk: boolean;
  isErr: boolean;
}

/** Represents Rust's Option<T> type */
export interface Option<T> {
  some?: T;
  none: boolean;
  isSome: boolean;
  isNone: boolean;
}

/** Error kind enumeration */
export enum ErrorKind {
  IO = "IO",
  Parse = "Parse",
  NotFound = "NotFound",
  PermissionDenied = "PermissionDenied",
  ConnectionRefused = "ConnectionRefused",
  Other = "Other",
  Custom = "Custom"
}

/** Custom error configuration */
export interface CustomError {
  kind: ErrorKind;
  message: string;
  source?: Error;
  backtrace?: string;
  context?: Record<string, any>;
}

/**
 * Trait system types
 */

/** Trait definition */
export interface TraitDefinition {
  name: string;
  methods: TraitMethod[];
  associatedTypes?: AssociatedType[];
  supertraits?: string[];
  documentation?: string;
  isUnsafe?: boolean;
}

/** Trait method signature */
export interface TraitMethod {
  name: string;
  parameters: Parameter[];
  returnType: string;
  isDefault?: boolean;
  whereClause?: WhereClause[];
  documentation?: string;
}

/** Trait implementation */
export interface TraitImpl {
  traitName: string;
  targetType: string;
  methods: MethodImpl[];
  whereClause?: WhereClause[];
  isUnsafe?: boolean;
}

/** Method implementation details */
export interface MethodImpl {
  name: string;
  body: string;
  visibility?: Visibility;
}

/** Trait bound specification */
export interface TraitBound {
  type: string;
  bounds: string[];
  lifetime?: string;
}

/** Dynamic trait object */
export interface DynTrait {
  trait: string;
  lifetime?: string;
  additionalBounds?: string[];
}

/**
 * Generics and type parameters
 */

/** Generic type parameter */
export interface TypeParameter {
  name: string;
  bounds?: string[];
  default?: string;
  variance?: "covariant" | "contravariant" | "invariant";
}

/** Generic type definition */
export interface GenericType {
  name: string;
  parameters: TypeParameter[];
  whereClause?: WhereClause[];
  phantomData?: string[];
}

/** Where clause constraint */
export interface WhereClause {
  type: string;
  constraint: string;
  lifetimes?: string[];
}

/** Associated type in traits */
export interface AssociatedType {
  name: string;
  bounds?: string[];
  default?: string;
}

/**
 * Collections and iterators
 */

/** Vector configuration */
export interface VecConfig<T> {
  capacity?: number;
  elements?: T[];
  growthStrategy?: "double" | "incremental";
}

/** HashMap configuration */
export interface HashMapConfig<K, V> {
  capacity?: number;
  entries?: Map<K, V>;
  hasher?: string;
}

/** Iterator pattern */
export interface IteratorPattern {
  itemType: string;
  method: "iter" | "iter_mut" | "into_iter";
  adapters?: IteratorAdapter[];
}

/** Iterator adapter (map, filter, etc.) */
export interface IteratorAdapter {
  kind: "map" | "filter" | "fold" | "take" | "skip" | "chain" | "zip";
  closure?: string;
  argument?: any;
}

/**
 * Module system
 */

/** Module tree structure */
export interface ModuleTree {
  name: string;
  path: string;
  children?: ModuleTree[];
  items?: ModuleItem[];
  visibility: Visibility;
}

/** Module item (function, struct, trait, etc.) */
export interface ModuleItem {
  kind: "fn" | "struct" | "enum" | "trait" | "const" | "static" | "mod";
  name: string;
  visibility: Visibility;
  documentation?: string;
}

/** Visibility scope */
export enum Visibility {
  Public = "pub",
  Crate = "pub(crate)",
  Super = "pub(super)",
  Private = "private",
  PublicIn = "pub(in path)"
}

/** Use statement */
export interface UseStatement {
  path: string;
  items?: string[];
  alias?: string;
  visibility: Visibility;
}

/**
 * Pattern matching
 */

/** Match arm in pattern matching */
export interface MatchArm {
  pattern: Pattern;
  guard?: string;
  body: string;
}

/** Pattern types */
export type Pattern =
  | LiteralPattern
  | IdentifierPattern
  | StructPattern
  | TuplePattern
  | EnumPattern
  | WildcardPattern;

export interface LiteralPattern {
  type: "literal";
  value: any;
}

export interface IdentifierPattern {
  type: "identifier";
  name: string;
  mutable?: boolean;
}

export interface StructPattern {
  type: "struct";
  name: string;
  fields: Record<string, Pattern>;
  restPattern?: boolean;
}

export interface TuplePattern {
  type: "tuple";
  elements: Pattern[];
}

export interface EnumPattern {
  type: "enum";
  variant: string;
  fields?: Pattern[];
}

export interface WildcardPattern {
  type: "wildcard";
}

/** Destructuring pattern */
export interface DestructuringPattern {
  target: string;
  bindings: Binding[];
  isRefutable: boolean;
}

export interface Binding {
  name: string;
  path: string[];
  mutable?: boolean;
}

/**
 * Smart pointers
 */

/** Box<T> configuration */
export interface BoxType<T> {
  value: T;
  heapAllocated: true;
  ownership: "unique";
}

/** Rc<T> configuration */
export interface RcConfig<T> {
  value: T;
  refCount: number;
  ownership: "shared";
  threadSafe: false;
}

/** Arc<T> configuration */
export interface ArcConfig<T> {
  value: T;
  refCount: number;
  ownership: "shared";
  threadSafe: true;
}

/** RefCell<T> configuration */
export interface RefCellConfig<T> {
  value: T;
  borrowState: BorrowState;
  interiorMutability: true;
}

export enum BorrowState {
  Unborrowed = "unborrowed",
  BorrowedShared = "borrowed_shared",
  BorrowedMut = "borrowed_mut"
}

/**
 * Common types
 */

export interface Parameter {
  name: string;
  type: string;
  mutable?: boolean;
  reference?: "shared" | "mutable";
}

export interface Lifetime {
  name: string;
  bound?: string;
  elided?: boolean;
}

/**
 * Error classes
 */

export class RustAnalysisError extends Error {
  constructor(message: string, public readonly code?: string, public readonly context?: any) {
    super(message);
    this.name = "RustAnalysisError";
  }
}

export class TraitResolutionError extends RustAnalysisError {
  constructor(message: string, public readonly traitName: string, public readonly targetType: string) {
    super(message, "TRAIT_RESOLUTION_ERROR", { traitName, targetType });
    this.name = "TraitResolutionError";
  }
}

export class BorrowCheckerError extends RustAnalysisError {
  constructor(message: string, public readonly location: string, public readonly borrowKind: "shared" | "mutable") {
    super(message, "BORROW_CHECKER_ERROR", { location, borrowKind });
    this.name = "BorrowCheckerError";
  }
}

export class LifetimeError extends RustAnalysisError {
  constructor(message: string, public readonly expected: string, public readonly found: string) {
    super(message, "LIFETIME_ERROR", { expected, found });
    this.name = "LifetimeError";
  }
}

/**
 * Configuration and options
 */

export interface RustIntermediateConfig {
  strictMode?: boolean;
  edition?: "2015" | "2018" | "2021" | "2024";
  features?: string[];
  lintLevel?: "forbid" | "deny" | "warn" | "allow";
  optimizationLevel?: 0 | 1 | 2 | 3;
}

export interface AnalysisOptions {
  checkBorrows?: boolean;
  checkLifetimes?: boolean;
  checkTraits?: boolean;
  suggestRefactoring?: boolean;
  maxComplexity?: number;
}

/**
 * Analysis results
 */

export interface TraitAnalysisResult {
  traits: TraitDefinition[];
  implementations: TraitImpl[];
  orphanRules: OrphanRuleViolation[];
  coherence: CoherenceCheck;
}

export interface OrphanRuleViolation {
  trait: string;
  type: string;
  reason: string;
  location: string;
}

export interface CoherenceCheck {
  valid: boolean;
  conflicts?: ConflictingImpl[];
}

export interface ConflictingImpl {
  impl1: string;
  impl2: string;
  reason: string;
}

export interface BorrowCheckResult {
  valid: boolean;
  errors?: BorrowCheckerError[];
  warnings?: string[];
  suggestions?: string[];
}

export interface PatternMatchResult {
  exhaustive: boolean;
  missingPatterns?: Pattern[];
  unreachableArms?: number[];
  suggestions?: string[];
}

export interface LintResult {
  errors: LintIssue[];
  warnings: LintIssue[];
  info: LintIssue[];
  stats: LintStats;
}

export interface LintIssue {
  level: "error" | "warning" | "info";
  message: string;
  code: string;
  location: SourceLocation;
  suggestion?: string;
}

export interface SourceLocation {
  file: string;
  line: number;
  column: number;
  span?: [number, number];
}

export interface LintStats {
  totalIssues: number;
  errorCount: number;
  warningCount: number;
  infoCount: number;
}
