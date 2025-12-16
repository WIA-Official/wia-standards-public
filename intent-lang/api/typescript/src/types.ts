/**
 * WIA-INTENT Type Definitions
 *
 * 의도 기반 프로그래밍 언어의 핵심 타입들
 */

// ============================================================
// Token Types
// ============================================================

export enum TokenType {
  // Keywords
  INTENT = 'INTENT',
  DESIRE = 'DESIRE',
  GIVEN = 'GIVEN',
  WANT = 'WANT',
  CONSTRAINTS = 'CONSTRAINTS',
  CERTAINTY = 'CERTAINTY',
  FALLBACK = 'FALLBACK',
  EVOLVE = 'EVOLVE',
  SEQUENCE = 'SEQUENCE',
  PARALLEL = 'PARALLEL',
  CHOOSE = 'CHOOSE',
  WHEN = 'WHEN',
  DEFAULT = 'DEFAULT',
  IF = 'IF',
  THEN = 'THEN',
  ELSE = 'ELSE',
  WITH = 'WITH',
  USING = 'USING',
  COLLECTIVE = 'COLLECTIVE',
  AGENTS = 'AGENTS',

  // Literals
  NUMBER = 'NUMBER',
  STRING = 'STRING',
  BOOLEAN = 'BOOLEAN',
  PROBABILITY = 'PROBABILITY',

  // Identifiers
  IDENTIFIER = 'IDENTIFIER',

  // Operators
  COLON = 'COLON',
  COMMA = 'COMMA',
  DOT = 'DOT',
  ARROW = 'ARROW',
  GTE = 'GTE',
  LTE = 'LTE',
  EQ = 'EQ',
  NEQ = 'NEQ',
  GT = 'GT',
  LT = 'LT',
  RANGE = 'RANGE',

  // Delimiters
  LBRACE = 'LBRACE',
  RBRACE = 'RBRACE',
  LPAREN = 'LPAREN',
  RPAREN = 'RPAREN',
  LBRACKET = 'LBRACKET',
  RBRACKET = 'RBRACKET',

  // Special
  NEWLINE = 'NEWLINE',
  EOF = 'EOF',
  COMMENT = 'COMMENT',
}

export interface Token {
  type: TokenType;
  value: string;
  line: number;
  column: number;
}

// ============================================================
// AST Node Types
// ============================================================

export type ASTNode =
  | ProgramNode
  | IntentNode
  | DesireNode
  | GivenNode
  | WantNode
  | ConstraintsNode
  | ConstraintNode
  | CertaintyNode
  | FallbackNode
  | EvolveNode
  | SequenceNode
  | ParallelNode
  | ChooseNode
  | WhenNode
  | CollectiveNode
  | AgentNode
  | ExpressionNode;

export interface ProgramNode {
  type: 'Program';
  declarations: (IntentNode | DesireNode | CollectiveNode)[];
}

export interface IntentNode {
  type: 'Intent';
  name: string;
  given?: GivenNode;
  want: WantNode;
  constraints?: ConstraintsNode;
  certainty?: CertaintyNode;
  fallback?: FallbackNode;
  evolve?: EvolveNode;
  body?: (SequenceNode | ParallelNode | ChooseNode)[];
}

export interface DesireNode {
  type: 'Desire';
  name: string;
  ultimateGoal: string;
  indicators: Record<string, ExpressionNode>;
  approach: Record<string, ExpressionNode>;
  ethics: Record<string, ExpressionNode>;
}

export interface GivenNode {
  type: 'Given';
  inputs: Record<string, TypeExpression>;
}

export interface WantNode {
  type: 'Want';
  outputs: Record<string, TypeExpression>;
}

export interface ConstraintsNode {
  type: 'Constraints';
  constraints: ConstraintNode[];
}

export interface ConstraintNode {
  type: 'Constraint';
  name: string;
  operator: '>=' | '<=' | '==' | '!=' | ':';
  value: ExpressionNode;
}

export interface CertaintyNode {
  type: 'Certainty';
  operator: '>=' | '<=';
  threshold: number;
}

export interface FallbackNode {
  type: 'Fallback';
  rules: FallbackRule[];
}

export interface FallbackRule {
  condition: ExpressionNode;
  action: ExpressionNode;
}

export interface EvolveNode {
  type: 'Evolve';
  learnFrom?: string[];
  adaptTo?: string[];
  improve?: {
    metric: string;
    method: string;
    rate: string;
  };
  boundaries?: {
    never: string[];
    always: string[];
  };
}

export interface SequenceNode {
  type: 'Sequence';
  steps: IntentCallNode[];
}

export interface ParallelNode {
  type: 'Parallel';
  branches: IntentCallNode[];
}

export interface ChooseNode {
  type: 'Choose';
  conditions: WhenNode[];
  defaultBranch?: IntentCallNode;
}

export interface WhenNode {
  type: 'When';
  condition: ExpressionNode;
  action: IntentCallNode;
}

export interface CollectiveNode {
  type: 'Collective';
  name: string;
  agents: AgentNode[];
  coordinate: (SequenceNode | ParallelNode)[];
  consensus?: ConsensusNode;
}

export interface AgentNode {
  type: 'Agent';
  name: string;
  role?: string;
  specializesIn?: string;
}

export interface ConsensusNode {
  type: 'Consensus';
  method: string;
  optimize: string;
  constraints: Record<string, ExpressionNode>;
}

export interface IntentCallNode {
  type: 'IntentCall';
  intentName: string;
  args: Record<string, ExpressionNode>;
}

// ============================================================
// Expression Types
// ============================================================

export type ExpressionNode =
  | LiteralNode
  | IdentifierNode
  | BinaryOpNode
  | UnaryOpNode
  | CallNode
  | MemberAccessNode
  | ListNode
  | MapNode
  | ProbabilityNode
  | RangeNode;

export interface LiteralNode {
  type: 'Literal';
  valueType: 'number' | 'string' | 'boolean' | 'null';
  value: number | string | boolean | null;
}

export interface IdentifierNode {
  type: 'Identifier';
  name: string;
}

export interface BinaryOpNode {
  type: 'BinaryOp';
  operator: string;
  left: ExpressionNode;
  right: ExpressionNode;
}

export interface UnaryOpNode {
  type: 'UnaryOp';
  operator: string;
  operand: ExpressionNode;
}

export interface CallNode {
  type: 'Call';
  callee: string;
  args: ExpressionNode[];
}

export interface MemberAccessNode {
  type: 'MemberAccess';
  object: ExpressionNode;
  member: string;
}

export interface ListNode {
  type: 'List';
  elements: ExpressionNode[];
}

export interface MapNode {
  type: 'Map';
  entries: [string, ExpressionNode][];
}

export interface ProbabilityNode {
  type: 'Probability';
  value: ExpressionNode;
  probability: number;
}

export interface RangeNode {
  type: 'Range';
  start: number;
  end: number;
  confidence?: number;
}

// ============================================================
// Type Expressions
// ============================================================

export interface TypeExpression {
  baseType: string;
  isOptional?: boolean;
  isProbabilistic?: boolean;
  genericArgs?: TypeExpression[];
  constraints?: Record<string, any>;
}

// ============================================================
// Runtime Types
// ============================================================

export interface IntentResult {
  success: boolean;
  output?: any;
  certainty: number;
  metadata: {
    executionTime: number;
    resourcesUsed: Record<string, number>;
    evolutionData?: EvolutionData;
  };
  warnings: string[];
  errors: string[];
}

export interface EvolutionData {
  learnedFrom: string[];
  improvements: {
    metric: string;
    before: number;
    after: number;
  }[];
  adaptations: string[];
}

export interface ExecutionContext {
  variables: Map<string, any>;
  intents: Map<string, IntentNode>;
  constraints: GlobalConstraints;
  ethics: EthicsConfig;
}

export interface GlobalConstraints {
  maxTime?: number;
  maxMemory?: number;
  maxCost?: number;
  minCertainty?: number;
}

export interface EthicsConfig {
  principles: string[];
  forbidden: string[];
  required: string[];
}

// ============================================================
// Compilation Types
// ============================================================

export type CompileTarget =
  | 'python'
  | 'javascript'
  | 'typescript'
  | 'rust'
  | 'wasm'
  | 'quantum'  // WIA-QUANTUM
  | 'pseudo';  // 의사코드

export interface CompilationOptions {
  target: CompileTarget;
  optimize: boolean;
  includeComments: boolean;
  strictMode: boolean;
  ethicsEnforcement: 'strict' | 'warn' | 'none';
}

export interface CompilationResult {
  target: CompileTarget;
  code: string;
  sourceMap?: string;
  warnings: CompilerWarning[];
  stats: CompilationStats;
}

export interface CompilerWarning {
  message: string;
  severity: 'info' | 'warning' | 'error';
  line?: number;
  column?: number;
}

export interface CompilationStats {
  intentCount: number;
  desireCount: number;
  constraintCount: number;
  evolutionRulesCount: number;
  estimatedComplexity: number;
}

// ============================================================
// Error Types
// ============================================================

export class WiaIntentError extends Error {
  constructor(
    message: string,
    public code: string,
    public line?: number,
    public column?: number
  ) {
    super(message);
    this.name = 'WiaIntentError';
  }
}

export class ParseError extends WiaIntentError {
  constructor(message: string, line?: number, column?: number) {
    super(message, 'PARSE_ERROR', line, column);
    this.name = 'ParseError';
  }
}

export class ValidationError extends WiaIntentError {
  constructor(message: string, line?: number, column?: number) {
    super(message, 'VALIDATION_ERROR', line, column);
    this.name = 'ValidationError';
  }
}

export class EthicsViolationError extends WiaIntentError {
  constructor(message: string, principle: string) {
    super(`Ethics violation (${principle}): ${message}`, 'ETHICS_VIOLATION');
    this.name = 'EthicsViolationError';
  }
}

export class CertaintyError extends WiaIntentError {
  constructor(required: number, actual: number) {
    super(
      `Certainty requirement not met: required ${required}, got ${actual}`,
      'CERTAINTY_ERROR'
    );
    this.name = 'CertaintyError';
  }
}
