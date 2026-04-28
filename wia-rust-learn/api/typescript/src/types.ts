/**
 * WIA-RUST-LEARN: Rust Learning Platform - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Education Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Learning Types
// ============================================================================

/**
 * Difficulty level for lessons and exercises
 */
export type DifficultyLevel = 'beginner' | 'intermediate' | 'advanced' | 'expert';

/**
 * Status of lesson or exercise completion
 */
export type CompletionStatus = 'not_started' | 'in_progress' | 'completed' | 'mastered';

/**
 * Rust concept categories
 */
export type ConceptCategory =
  | 'basics'
  | 'ownership'
  | 'borrowing'
  | 'lifetimes'
  | 'structs'
  | 'enums'
  | 'traits'
  | 'generics'
  | 'error_handling'
  | 'collections'
  | 'concurrency'
  | 'async'
  | 'macros'
  | 'unsafe'
  | 'testing';

/**
 * Output format for code execution
 */
export type OutputFormat = 'text' | 'json' | 'markdown' | 'html';

/**
 * Test result status
 */
export type TestStatus = 'passed' | 'failed' | 'error' | 'timeout';

// ============================================================================
// Learning Path Types
// ============================================================================

/**
 * A learning path represents a structured curriculum
 */
export interface LearningPath {
  /** Unique identifier for the learning path */
  id: string;

  /** Title of the learning path */
  title: string;

  /** Detailed description */
  description: string;

  /** Difficulty level */
  difficulty: DifficultyLevel;

  /** Estimated completion time in hours */
  estimated_hours: number;

  /** Prerequisites (other learning path IDs) */
  prerequisites: string[];

  /** List of modules in this path */
  modules: Module[];

  /** Tags for categorization */
  tags: string[];

  /** Number of students enrolled */
  enrolled_count?: number;

  /** Average completion rate (0-100%) */
  completion_rate?: number;
}

/**
 * A module contains multiple related lessons
 */
export interface Module {
  /** Unique identifier for the module */
  id: string;

  /** Module title */
  title: string;

  /** Module description */
  description: string;

  /** Order within the learning path */
  order: number;

  /** Lessons in this module */
  lessons: Lesson[];

  /** Estimated completion time in minutes */
  estimated_minutes: number;
}

/**
 * A lesson teaches a specific concept
 */
export interface Lesson {
  /** Unique identifier for the lesson */
  id: string;

  /** Lesson title */
  title: string;

  /** Lesson content in markdown */
  content: string;

  /** Concept category */
  category: ConceptCategory;

  /** Difficulty level */
  difficulty: DifficultyLevel;

  /** Order within the module */
  order: number;

  /** Code examples */
  examples: CodeSnippet[];

  /** Interactive exercises */
  exercises: Exercise[];

  /** Estimated completion time in minutes */
  estimated_minutes: number;

  /** Key concepts covered */
  key_concepts: string[];

  /** Video URL (optional) */
  video_url?: string;
}

// ============================================================================
// Code Execution Types
// ============================================================================

/**
 * A code snippet with metadata
 */
export interface CodeSnippet {
  /** Unique identifier */
  id: string;

  /** Code content */
  code: string;

  /** Programming language (always 'rust' for this SDK) */
  language: string;

  /** Title or description */
  title?: string;

  /** Whether code is editable */
  editable: boolean;

  /** Whether code should run automatically */
  auto_run?: boolean;

  /** Expected output (for validation) */
  expected_output?: string;
}

/**
 * Compiler options for code execution
 */
export interface CompilerOptions {
  /** Optimization level (0, 1, 2, 3, or 's' for size, 'z' for minimum size) */
  opt_level?: string;

  /** Edition (2015, 2018, 2021) */
  edition?: '2015' | '2018' | '2021';

  /** Enable debug symbols */
  debug?: boolean;

  /** Additional compiler flags */
  flags?: string[];

  /** Timeout in milliseconds */
  timeout_ms?: number;

  /** Memory limit in MB */
  memory_limit_mb?: number;
}

/**
 * Result of code execution
 */
export interface ExecutionResult {
  /** Whether execution was successful */
  success: boolean;

  /** Standard output */
  stdout: string;

  /** Standard error */
  stderr: string;

  /** Exit code */
  exit_code: number;

  /** Execution time in milliseconds */
  execution_time_ms: number;

  /** Memory used in KB */
  memory_used_kb?: number;

  /** Compilation errors if any */
  compile_errors?: CompileError[];

  /** Runtime errors if any */
  runtime_errors?: string[];
}

/**
 * Compilation error details
 */
export interface CompileError {
  /** Error message */
  message: string;

  /** Error code (e.g., E0308) */
  code?: string;

  /** File path */
  file?: string;

  /** Line number */
  line?: number;

  /** Column number */
  column?: number;

  /** Severity level */
  level: 'error' | 'warning' | 'note' | 'help';

  /** Suggested fix */
  suggestion?: string;
}

// ============================================================================
// Exercise and Challenge Types
// ============================================================================

/**
 * An interactive exercise
 */
export interface Exercise {
  /** Unique identifier */
  id: string;

  /** Exercise title */
  title: string;

  /** Exercise description/instructions */
  description: string;

  /** Starter code */
  starter_code: string;

  /** Difficulty level */
  difficulty: DifficultyLevel;

  /** Test cases to validate solution */
  test_cases: TestCase[];

  /** Hints for students */
  hints: Hint[];

  /** Reference solution */
  solution?: Solution;

  /** Points awarded for completion */
  points: number;

  /** Maximum attempts allowed */
  max_attempts?: number;
}

/**
 * A coding challenge (more complex than exercise)
 */
export interface Challenge {
  /** Unique identifier */
  id: string;

  /** Challenge title */
  title: string;

  /** Detailed problem statement */
  problem_statement: string;

  /** Difficulty level */
  difficulty: DifficultyLevel;

  /** Concept category */
  category: ConceptCategory;

  /** Starter code template */
  starter_code: string;

  /** Test cases */
  test_cases: TestCase[];

  /** Time limit in minutes */
  time_limit_minutes: number;

  /** Points awarded */
  points: number;

  /** Hints available */
  hints: Hint[];

  /** Solution explanation */
  solution?: Solution;

  /** Success rate (0-100%) */
  success_rate?: number;
}

/**
 * Test case for validating solutions
 */
export interface TestCase {
  /** Test case identifier */
  id: string;

  /** Test name */
  name: string;

  /** Input values */
  input?: string;

  /** Expected output */
  expected_output: string;

  /** Whether this is a hidden test case */
  hidden: boolean;

  /** Points for this test */
  points?: number;
}

/**
 * Test result for a single test case
 */
export interface TestResult {
  /** Test case ID */
  test_id: string;

  /** Test status */
  status: TestStatus;

  /** Actual output */
  actual_output?: string;

  /** Expected output */
  expected_output: string;

  /** Error message if failed */
  error_message?: string;

  /** Execution time in ms */
  execution_time_ms: number;
}

/**
 * Hint for an exercise or challenge
 */
export interface Hint {
  /** Hint identifier */
  id: string;

  /** Hint level (1 is most general, higher is more specific) */
  level: number;

  /** Hint content */
  content: string;

  /** Cost in points (negative) */
  point_penalty?: number;
}

/**
 * Reference solution with explanation
 */
export interface Solution {
  /** Solution code */
  code: string;

  /** Explanation of the solution */
  explanation: string;

  /** Key concepts demonstrated */
  concepts: string[];

  /** Time complexity */
  time_complexity?: string;

  /** Space complexity */
  space_complexity?: string;
}

// ============================================================================
// Progress Tracking Types
// ============================================================================

/**
 * User progress across the platform
 */
export interface UserProgress {
  /** User identifier */
  user_id: string;

  /** Total points earned */
  total_points: number;

  /** Current level */
  level: number;

  /** Points needed for next level */
  points_to_next_level: number;

  /** Lessons completed */
  lessons_completed: string[];

  /** Exercises completed */
  exercises_completed: string[];

  /** Challenges completed */
  challenges_completed: string[];

  /** Achievements earned */
  achievements: Achievement[];

  /** Learning paths in progress */
  active_paths: string[];

  /** Total study time in minutes */
  total_study_minutes: number;

  /** Streak days */
  streak_days: number;

  /** Last activity timestamp */
  last_activity: Date | string;
}

/**
 * Achievement/badge earned by user
 */
export interface Achievement {
  /** Achievement identifier */
  id: string;

  /** Achievement name */
  name: string;

  /** Description */
  description: string;

  /** Icon/badge URL */
  icon_url?: string;

  /** Date earned */
  earned_date: Date | string;

  /** Rarity level */
  rarity: 'common' | 'rare' | 'epic' | 'legendary';

  /** Points awarded */
  points: number;
}

/**
 * Progress for a specific lesson
 */
export interface LessonProgress {
  /** Lesson ID */
  lesson_id: string;

  /** Completion status */
  status: CompletionStatus;

  /** Progress percentage (0-100) */
  progress_percent: number;

  /** Time spent in minutes */
  time_spent_minutes: number;

  /** Number of attempts */
  attempts: number;

  /** Last accessed timestamp */
  last_accessed: Date | string;

  /** Notes taken by user */
  notes?: string;
}

// ============================================================================
// Quiz System Types
// ============================================================================

/**
 * A quiz to test understanding
 */
export interface Quiz {
  /** Quiz identifier */
  id: string;

  /** Quiz title */
  title: string;

  /** Quiz description */
  description: string;

  /** Module or lesson ID this quiz belongs to */
  parent_id: string;

  /** Questions in the quiz */
  questions: Question[];

  /** Time limit in minutes */
  time_limit_minutes: number;

  /** Passing score percentage */
  passing_score: number;

  /** Maximum attempts allowed */
  max_attempts: number;
}

/**
 * Quiz question
 */
export interface Question {
  /** Question identifier */
  id: string;

  /** Question text */
  text: string;

  /** Question type */
  type: 'multiple_choice' | 'multiple_select' | 'true_false' | 'code_output';

  /** Answer options */
  options: Answer[];

  /** Correct answer ID(s) */
  correct_answers: string[];

  /** Points for this question */
  points: number;

  /** Explanation of correct answer */
  explanation?: string;

  /** Code snippet (for code_output type) */
  code_snippet?: string;
}

/**
 * Answer option for a question
 */
export interface Answer {
  /** Answer identifier */
  id: string;

  /** Answer text */
  text: string;

  /** Whether this is correct (should not be exposed to frontend) */
  is_correct?: boolean;
}

/**
 * Quiz attempt result
 */
export interface QuizResult {
  /** Quiz ID */
  quiz_id: string;

  /** User ID */
  user_id: string;

  /** Score percentage */
  score_percent: number;

  /** Points earned */
  points_earned: number;

  /** Whether user passed */
  passed: boolean;

  /** Answers submitted */
  answers: Record<string, string[]>;

  /** Time taken in minutes */
  time_taken_minutes: number;

  /** Attempt number */
  attempt_number: number;

  /** Timestamp */
  completed_at: Date | string;
}

// ============================================================================
// Playground Types
// ============================================================================

/**
 * Interactive playground session
 */
export interface PlaygroundSession {
  /** Session identifier */
  session_id: string;

  /** User identifier */
  user_id: string;

  /** Current code */
  code: string;

  /** Compiler options */
  compiler_options: CompilerOptions;

  /** Session created timestamp */
  created_at: Date | string;

  /** Last modified timestamp */
  last_modified: Date | string;

  /** Session title/name */
  title?: string;

  /** Whether session is shared */
  is_shared: boolean;

  /** Share URL if shared */
  share_url?: string;
}

// ============================================================================
// Rust Concept Types
// ============================================================================

/**
 * A Rust programming concept
 */
export interface RustConcept {
  /** Concept identifier */
  id: string;

  /** Concept name */
  name: string;

  /** Category */
  category: ConceptCategory;

  /** Short description */
  description: string;

  /** Detailed explanation */
  explanation: string;

  /** Code examples */
  examples: CodeSnippet[];

  /** Related concepts */
  related_concepts: string[];

  /** Common pitfalls */
  common_pitfalls?: string[];

  /** Best practices */
  best_practices?: string[];
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-RUST-LEARN error codes
 */
export enum RustLearnErrorCode {
  COMPILATION_FAILED = 'RL001',
  EXECUTION_TIMEOUT = 'RL002',
  MEMORY_LIMIT_EXCEEDED = 'RL003',
  INVALID_CODE = 'RL004',
  TEST_FAILED = 'RL005',
  LESSON_NOT_FOUND = 'RL006',
  EXERCISE_NOT_FOUND = 'RL007',
  INSUFFICIENT_PERMISSIONS = 'RL008',
  PLAYGROUND_ERROR = 'RL009',
  NETWORK_ERROR = 'RL010',
  INVALID_PARAMETERS = 'RL011',
}

/**
 * Rust learning platform error
 */
export class RustLearnError extends Error {
  constructor(
    public code: RustLearnErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'RustLearnError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  DifficultyLevel,
  CompletionStatus,
  ConceptCategory,
  OutputFormat,
  TestStatus,
  LearningPath,
  Module,
  Lesson,
  CodeSnippet,
  CompilerOptions,
  ExecutionResult,
  CompileError,
  Exercise,
  Challenge,
  TestCase,
  TestResult,
  Hint,
  Solution,
  UserProgress,
  Achievement,
  LessonProgress,
  Quiz,
  Question,
  Answer,
  QuizResult,
  PlaygroundSession,
  RustConcept,
};

export { RustLearnErrorCode, RustLearnError };
