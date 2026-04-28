/**
 * WIA-RUST-LEARN: Rust Learning Platform - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Education Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  LearningPath,
  Lesson,
  Exercise,
  Challenge,
  CodeSnippet,
  CompilerOptions,
  ExecutionResult,
  CompileError,
  TestResult,
  TestStatus,
  Hint,
  Solution,
  UserProgress,
  Achievement,
  LessonProgress,
  Quiz,
  QuizResult,
  PlaygroundSession,
  RustConcept,
  ConceptCategory,
  DifficultyLevel,
  CompletionStatus,
  RustLearnErrorCode,
  RustLearnError,
} from './types';

// ============================================================================
// Event Emitter
// ============================================================================

type EventCallback = (...args: any[]) => void;

/**
 * Simple event emitter for SDK events
 */
class EventEmitter {
  private events: Map<string, EventCallback[]> = new Map();

  on(event: string, callback: EventCallback): void {
    if (!this.events.has(event)) {
      this.events.set(event, []);
    }
    this.events.get(event)!.push(callback);
  }

  off(event: string, callback: EventCallback): void {
    const callbacks = this.events.get(event);
    if (callbacks) {
      const index = callbacks.indexOf(callback);
      if (index > -1) {
        callbacks.splice(index, 1);
      }
    }
  }

  emit(event: string, ...args: any[]): void {
    const callbacks = this.events.get(event);
    if (callbacks) {
      callbacks.forEach((callback) => callback(...args));
    }
  }
}

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA Rust Learning Platform SDK
 *
 * Provides comprehensive access to interactive Rust tutorials,
 * code execution, progress tracking, and learning management.
 */
export class WIARustLearnSDK extends EventEmitter {
  private baseUrl: string;
  private apiKey?: string;
  private userId?: string;

  /**
   * Create a new Rust Learning SDK instance
   *
   * @param config - Configuration options
   */
  constructor(config: {
    baseUrl?: string;
    apiKey?: string;
    userId?: string;
  } = {}) {
    super();
    this.baseUrl = config.baseUrl || 'https://api.wia-rust-learn.org/v1';
    this.apiKey = config.apiKey;
    this.userId = config.userId;
  }

  // ============================================================================
  // Learning Path Methods
  // ============================================================================

  /**
   * Get all available learning paths
   *
   * @param filters - Optional filters for difficulty and category
   * @returns Array of learning paths
   */
  async getLearningPaths(filters?: {
    difficulty?: DifficultyLevel;
    category?: ConceptCategory;
  }): Promise<LearningPath[]> {
    try {
      this.emit('learning_paths:fetch', filters);

      // Simulate API call
      const paths: LearningPath[] = [
        {
          id: 'rust-basics',
          title: 'Rust Basics',
          description: 'Introduction to Rust programming fundamentals',
          difficulty: 'beginner',
          estimated_hours: 20,
          prerequisites: [],
          modules: [],
          tags: ['beginner', 'fundamentals'],
          enrolled_count: 15420,
          completion_rate: 78,
        },
        {
          id: 'ownership-mastery',
          title: 'Ownership & Borrowing Mastery',
          description: 'Deep dive into Rust ownership system',
          difficulty: 'intermediate',
          estimated_hours: 15,
          prerequisites: ['rust-basics'],
          modules: [],
          tags: ['ownership', 'borrowing', 'lifetimes'],
          enrolled_count: 8932,
          completion_rate: 65,
        },
      ];

      this.emit('learning_paths:success', paths);
      return paths;
    } catch (error) {
      this.emit('learning_paths:error', error);
      throw new RustLearnError(
        RustLearnErrorCode.NETWORK_ERROR,
        'Failed to fetch learning paths',
        { error }
      );
    }
  }

  /**
   * Start a lesson and track progress
   *
   * @param lessonId - Lesson identifier
   * @returns Lesson details
   */
  async startLesson(lessonId: string): Promise<Lesson> {
    try {
      this.emit('lesson:start', lessonId);

      if (!lessonId) {
        throw new RustLearnError(
          RustLearnErrorCode.INVALID_PARAMETERS,
          'Lesson ID is required'
        );
      }

      // Simulate lesson retrieval
      const lesson: Lesson = {
        id: lessonId,
        title: 'Understanding Ownership',
        content: '# Ownership in Rust\n\nOwnership is Rust\'s most unique feature...',
        category: 'ownership',
        difficulty: 'intermediate',
        order: 1,
        examples: [],
        exercises: [],
        estimated_minutes: 30,
        key_concepts: ['ownership', 'move semantics', 'stack vs heap'],
      };

      this.emit('lesson:loaded', lesson);
      return lesson;
    } catch (error) {
      this.emit('lesson:error', error);
      throw error;
    }
  }

  // ============================================================================
  // Code Execution Methods
  // ============================================================================

  /**
   * Run Rust code in the playground
   *
   * @param code - Rust source code
   * @param options - Compiler options
   * @returns Execution result
   */
  async runCode(
    code: string,
    options: CompilerOptions = {}
  ): Promise<ExecutionResult> {
    try {
      this.emit('code:compile', { code, options });

      if (!code || code.trim().length === 0) {
        throw new RustLearnError(
          RustLearnErrorCode.INVALID_CODE,
          'Code cannot be empty'
        );
      }

      // Simulate compilation and execution
      const defaultOptions: CompilerOptions = {
        edition: '2021',
        opt_level: '0',
        debug: true,
        timeout_ms: 30000,
        memory_limit_mb: 128,
        ...options,
      };

      // Check for common syntax errors
      const compileErrors: CompileError[] = [];
      if (!code.includes('fn main')) {
        compileErrors.push({
          message: 'cannot find `main` function',
          code: 'E0601',
          line: 1,
          column: 1,
          level: 'error',
          suggestion: 'Add a main function: fn main() { ... }',
        });
      }

      if (compileErrors.length > 0) {
        const result: ExecutionResult = {
          success: false,
          stdout: '',
          stderr: 'Compilation failed',
          exit_code: 101,
          execution_time_ms: 0,
          compile_errors: compileErrors,
        };
        this.emit('code:error', result);
        return result;
      }

      // Simulate successful execution
      const result: ExecutionResult = {
        success: true,
        stdout: 'Hello, Rust!\n',
        stderr: '',
        exit_code: 0,
        execution_time_ms: 45,
        memory_used_kb: 2048,
      };

      this.emit('code:success', result);
      return result;
    } catch (error) {
      this.emit('code:error', error);
      throw error;
    }
  }

  /**
   * Submit an exercise solution for validation
   *
   * @param exerciseId - Exercise identifier
   * @param code - User's solution code
   * @returns Test results
   */
  async submitExercise(
    exerciseId: string,
    code: string
  ): Promise<{ passed: boolean; results: TestResult[]; score: number }> {
    try {
      this.emit('exercise:submit', { exerciseId, code });

      // Run code against test cases
      const execution = await this.runCode(code);

      if (!execution.success) {
        return {
          passed: false,
          results: [],
          score: 0,
        };
      }

      // Simulate test results
      const results: TestResult[] = [
        {
          test_id: 'test_1',
          status: 'passed',
          actual_output: 'Hello, Rust!',
          expected_output: 'Hello, Rust!',
          execution_time_ms: 45,
        },
        {
          test_id: 'test_2',
          status: 'passed',
          actual_output: '42',
          expected_output: '42',
          execution_time_ms: 38,
        },
      ];

      const passed = results.every((r) => r.status === 'passed');
      const score = (results.filter((r) => r.status === 'passed').length / results.length) * 100;

      this.emit('exercise:complete', { exerciseId, passed, score });

      return { passed, results, score };
    } catch (error) {
      this.emit('exercise:error', error);
      throw error;
    }
  }

  /**
   * Check if a solution is correct
   *
   * @param code - Solution code
   * @param expectedOutput - Expected output
   * @returns Whether solution is correct
   */
  async checkSolution(code: string, expectedOutput: string): Promise<boolean> {
    const result = await this.runCode(code);
    return result.success && result.stdout.trim() === expectedOutput.trim();
  }

  /**
   * Get a hint for an exercise
   *
   * @param exerciseId - Exercise identifier
   * @param hintLevel - Hint level (1 is most general)
   * @returns Hint object
   */
  async getHint(exerciseId: string, hintLevel: number = 1): Promise<Hint> {
    try {
      this.emit('hint:request', { exerciseId, hintLevel });

      const hint: Hint = {
        id: `hint_${exerciseId}_${hintLevel}`,
        level: hintLevel,
        content: 'Remember that Rust uses ownership to manage memory...',
        point_penalty: -5,
      };

      this.emit('hint:provided', hint);
      return hint;
    } catch (error) {
      this.emit('hint:error', error);
      throw error;
    }
  }

  // ============================================================================
  // Progress Tracking Methods
  // ============================================================================

  /**
   * Track user progress
   *
   * @param lessonId - Lesson identifier
   * @param progressPercent - Progress percentage
   * @returns Updated lesson progress
   */
  async trackProgress(
    lessonId: string,
    progressPercent: number
  ): Promise<LessonProgress> {
    try {
      this.emit('progress:update', { lessonId, progressPercent });

      const progress: LessonProgress = {
        lesson_id: lessonId,
        status: progressPercent === 100 ? 'completed' : 'in_progress',
        progress_percent: progressPercent,
        time_spent_minutes: 25,
        attempts: 1,
        last_accessed: new Date().toISOString(),
      };

      this.emit('progress:saved', progress);
      return progress;
    } catch (error) {
      this.emit('progress:error', error);
      throw error;
    }
  }

  /**
   * Get user's overall progress
   *
   * @returns User progress object
   */
  async getUserProgress(): Promise<UserProgress> {
    try {
      const progress: UserProgress = {
        user_id: this.userId || 'anonymous',
        total_points: 1250,
        level: 8,
        points_to_next_level: 150,
        lessons_completed: ['lesson_1', 'lesson_2', 'lesson_3'],
        exercises_completed: ['ex_1', 'ex_2'],
        challenges_completed: ['challenge_1'],
        achievements: [],
        active_paths: ['rust-basics'],
        total_study_minutes: 420,
        streak_days: 7,
        last_activity: new Date().toISOString(),
      };

      this.emit('progress:fetched', progress);
      return progress;
    } catch (error) {
      this.emit('progress:error', error);
      throw error;
    }
  }

  /**
   * Get user's achievements
   *
   * @returns Array of achievements
   */
  async getAchievements(): Promise<Achievement[]> {
    try {
      this.emit('achievements:fetch');

      const achievements: Achievement[] = [
        {
          id: 'first_lesson',
          name: 'First Steps',
          description: 'Completed your first lesson',
          icon_url: 'https://cdn.wia.org/badges/first-steps.png',
          earned_date: new Date().toISOString(),
          rarity: 'common',
          points: 50,
        },
        {
          id: 'ownership_master',
          name: 'Ownership Master',
          description: 'Mastered Rust ownership concepts',
          icon_url: 'https://cdn.wia.org/badges/ownership-master.png',
          earned_date: new Date().toISOString(),
          rarity: 'rare',
          points: 200,
        },
      ];

      this.emit('achievements:success', achievements);
      return achievements;
    } catch (error) {
      this.emit('achievements:error', error);
      throw error;
    }
  }

  // ============================================================================
  // Quiz Methods
  // ============================================================================

  /**
   * Take a quiz
   *
   * @param quizId - Quiz identifier
   * @param answers - User's answers
   * @returns Quiz result
   */
  async takeQuiz(
    quizId: string,
    answers: Record<string, string[]>
  ): Promise<QuizResult> {
    try {
      this.emit('quiz:submit', { quizId, answers });

      // Simulate quiz grading
      const totalQuestions = Object.keys(answers).length;
      const correctAnswers = Math.floor(totalQuestions * 0.8); // 80% correct
      const scorePercent = (correctAnswers / totalQuestions) * 100;

      const result: QuizResult = {
        quiz_id: quizId,
        user_id: this.userId || 'anonymous',
        score_percent: scorePercent,
        points_earned: Math.floor(scorePercent * 2),
        passed: scorePercent >= 70,
        answers,
        time_taken_minutes: 15,
        attempt_number: 1,
        completed_at: new Date().toISOString(),
      };

      this.emit('quiz:complete', result);
      return result;
    } catch (error) {
      this.emit('quiz:error', error);
      throw error;
    }
  }

  // ============================================================================
  // Playground Methods
  // ============================================================================

  /**
   * Create a new playground session
   *
   * @param code - Initial code
   * @param options - Compiler options
   * @returns Playground session
   */
  async createPlayground(
    code: string = 'fn main() {\n    println!("Hello, Rust!");\n}',
    options: CompilerOptions = {}
  ): Promise<PlaygroundSession> {
    try {
      this.emit('playground:create', { code, options });

      const session: PlaygroundSession = {
        session_id: `session_${Date.now()}`,
        user_id: this.userId || 'anonymous',
        code,
        compiler_options: {
          edition: '2021',
          opt_level: '0',
          debug: true,
          ...options,
        },
        created_at: new Date().toISOString(),
        last_modified: new Date().toISOString(),
        is_shared: false,
      };

      this.emit('playground:created', session);
      return session;
    } catch (error) {
      this.emit('playground:error', error);
      throw error;
    }
  }

  /**
   * Share a playground session
   *
   * @param sessionId - Session identifier
   * @returns Share URL
   */
  async sharePlayground(sessionId: string): Promise<string> {
    try {
      this.emit('playground:share', sessionId);

      const shareUrl = `${this.baseUrl}/playground/share/${sessionId}`;

      this.emit('playground:shared', shareUrl);
      return shareUrl;
    } catch (error) {
      this.emit('playground:error', error);
      throw error;
    }
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Set user ID for tracking
   *
   * @param userId - User identifier
   */
  setUserId(userId: string): void {
    this.userId = userId;
    this.emit('user:set', userId);
  }

  /**
   * Set API key for authentication
   *
   * @param apiKey - API key
   */
  setApiKey(apiKey: string): void {
    this.apiKey = apiKey;
  }
}

// ============================================================================
// Factory Function
// ============================================================================

/**
 * Create a new WIA Rust Learning SDK instance
 *
 * @param config - Configuration options
 * @returns SDK instance
 */
export function createRustLearnSDK(config?: {
  baseUrl?: string;
  apiKey?: string;
  userId?: string;
}): WIARustLearnSDK {
  return new WIARustLearnSDK(config);
}

// ============================================================================
// Exports
// ============================================================================

export { WIARustLearnSDK };
export * from './types';
