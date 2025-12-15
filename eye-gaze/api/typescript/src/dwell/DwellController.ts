/**
 * WIA Eye Gaze Standard - Dwell Controller
 *
 * Implements dwell-based selection for AAC and accessibility.
 *
 * 弘益人間 - 널리 인간을 이롭게
 */

import { IEyeTracker } from '../tracker/IEyeTracker';
import {
  GazePoint,
  GazeTarget,
  DwellConfig,
  DwellFeedbackType,
  DwellState,
  BoundingBox,
  Subscription,
} from '../types';

export interface DwellControllerOptions extends Partial<DwellConfig> {
  /** Eye tracker instance */
  tracker: IEyeTracker;
}

export type DwellEventHandler = (target: GazeTarget, progress?: number) => void;

/**
 * Dwell Controller
 *
 * Manages dwell-based selection for gaze interaction.
 * Tracks when the user's gaze dwells on a target for a specified duration.
 *
 * @example
 * ```typescript
 * const dwellController = new DwellController({
 *   tracker: eyeTracker,
 *   threshold: 800,
 *   visualFeedback: true,
 * });
 *
 * dwellController.registerTarget({
 *   elementId: 'button-1',
 *   boundingBox: { x: 0.4, y: 0.5, width: 0.2, height: 0.1 },
 *   semanticType: 'button',
 *   label: 'OK',
 * });
 *
 * dwellController.onDwellComplete(target => {
 *   console.log(`Selected: ${target.label}`);
 * });
 *
 * dwellController.start();
 * ```
 */
export class DwellController {
  private tracker: IEyeTracker;
  private config: DwellConfig;
  private targets: Map<string, GazeTarget>;
  private dwellState: DwellState;
  private subscription: Subscription | null;
  private progressInterval: ReturnType<typeof setInterval> | null;
  private cooldownUntil: number;
  private enabled: boolean;

  // Event handlers
  private onDwellStartHandlers: Set<DwellEventHandler>;
  private onDwellProgressHandlers: Set<DwellEventHandler>;
  private onDwellCompleteHandlers: Set<DwellEventHandler>;
  private onDwellCancelHandlers: Set<DwellEventHandler>;

  constructor(options: DwellControllerOptions) {
    this.tracker = options.tracker;
    this.config = {
      threshold: options.threshold ?? 800,
      progressInterval: options.progressInterval ?? 50,
      visualFeedback: options.visualFeedback ?? true,
      cooldownPeriod: options.cooldownPeriod ?? 500,
    };

    this.targets = new Map();
    this.dwellState = { active: false, progress: 0 };
    this.subscription = null;
    this.progressInterval = null;
    this.cooldownUntil = 0;
    this.enabled = false;

    this.onDwellStartHandlers = new Set();
    this.onDwellProgressHandlers = new Set();
    this.onDwellCompleteHandlers = new Set();
    this.onDwellCancelHandlers = new Set();
  }

  // ============================================
  // Configuration
  // ============================================

  /**
   * Set the dwell selection time
   * @param ms - Dwell time in milliseconds
   */
  setDwellTime(ms: number): void {
    this.config.threshold = ms;
  }

  /**
   * Get current dwell time
   */
  getDwellTime(): number {
    return this.config.threshold;
  }

  /**
   * Set visual feedback type
   */
  setDwellFeedback(type: DwellFeedbackType): void {
    // Store feedback type for UI rendering
    this.config.visualFeedback = type !== DwellFeedbackType.AUDIO_ONLY;
  }

  /**
   * Set cooldown period after selection
   * @param ms - Cooldown time in milliseconds
   */
  setCooldownPeriod(ms: number): void {
    this.config.cooldownPeriod = ms;
  }

  // ============================================
  // Target Management
  // ============================================

  /**
   * Register a target for dwell selection
   */
  registerTarget(target: GazeTarget): void {
    this.targets.set(target.elementId, target);
  }

  /**
   * Unregister a target
   */
  unregisterTarget(targetId: string): void {
    this.targets.delete(targetId);
  }

  /**
   * Update an existing target
   */
  updateTarget(targetId: string, updates: Partial<GazeTarget>): void {
    const existing = this.targets.get(targetId);
    if (existing) {
      this.targets.set(targetId, { ...existing, ...updates });
    }
  }

  /**
   * Get all registered targets
   */
  getTargets(): GazeTarget[] {
    return Array.from(this.targets.values());
  }

  /**
   * Clear all targets
   */
  clearTargets(): void {
    this.targets.clear();
    this.cancelDwell();
  }

  // ============================================
  // Event Handlers
  // ============================================

  /**
   * Register handler for dwell start
   */
  onDwellStart(handler: DwellEventHandler): void {
    this.onDwellStartHandlers.add(handler);
  }

  /**
   * Register handler for dwell progress
   */
  onDwellProgress(handler: DwellEventHandler): void {
    this.onDwellProgressHandlers.add(handler);
  }

  /**
   * Register handler for dwell complete (selection)
   */
  onDwellComplete(handler: DwellEventHandler): void {
    this.onDwellCompleteHandlers.add(handler);
  }

  /**
   * Register handler for dwell cancel
   */
  onDwellCancel(handler: DwellEventHandler): void {
    this.onDwellCancelHandlers.add(handler);
  }

  /**
   * Remove a handler
   */
  off(event: 'start' | 'progress' | 'complete' | 'cancel', handler: DwellEventHandler): void {
    switch (event) {
      case 'start':
        this.onDwellStartHandlers.delete(handler);
        break;
      case 'progress':
        this.onDwellProgressHandlers.delete(handler);
        break;
      case 'complete':
        this.onDwellCompleteHandlers.delete(handler);
        break;
      case 'cancel':
        this.onDwellCancelHandlers.delete(handler);
        break;
    }
  }

  // ============================================
  // Control
  // ============================================

  /**
   * Start dwell detection
   */
  start(): void {
    if (this.enabled) return;

    this.enabled = true;
    this.subscription = this.tracker.subscribe(this.handleGazeData.bind(this));
  }

  /**
   * Stop dwell detection
   */
  stop(): void {
    if (!this.enabled) return;

    this.enabled = false;
    this.cancelDwell();

    if (this.subscription) {
      this.tracker.unsubscribe(this.subscription);
      this.subscription = null;
    }
  }

  /**
   * Temporarily pause dwell detection
   */
  pause(): void {
    this.enabled = false;
    this.cancelDwell();
  }

  /**
   * Resume dwell detection
   */
  resume(): void {
    this.enabled = true;
  }

  /**
   * Get current dwell state
   */
  getState(): DwellState {
    return { ...this.dwellState };
  }

  /**
   * Check if a specific target is being dwelled on
   */
  isDwellingOn(targetId: string): boolean {
    return this.dwellState.active && this.dwellState.target?.elementId === targetId;
  }

  // ============================================
  // Internal Methods
  // ============================================

  private handleGazeData(gaze: GazePoint): void {
    if (!this.enabled || !gaze.valid) {
      if (this.dwellState.active) {
        this.cancelDwell();
      }
      return;
    }

    // Check cooldown
    if (Date.now() < this.cooldownUntil) {
      return;
    }

    // Find target under gaze
    const hitTarget = this.findTargetAtPoint(gaze.x, gaze.y);

    if (!hitTarget) {
      // No target - cancel if active
      if (this.dwellState.active) {
        this.cancelDwell();
      }
      return;
    }

    if (!this.dwellState.active) {
      // Start new dwell
      this.startDwell(hitTarget);
    } else if (this.dwellState.target?.elementId !== hitTarget.elementId) {
      // Switched to different target
      this.cancelDwell();
      this.startDwell(hitTarget);
    }
    // Otherwise continue current dwell (handled by progress interval)
  }

  private findTargetAtPoint(x: number, y: number): GazeTarget | null {
    for (const target of this.targets.values()) {
      if (this.isPointInBounds(x, y, target.boundingBox)) {
        return target;
      }
    }
    return null;
  }

  private isPointInBounds(x: number, y: number, bounds: BoundingBox): boolean {
    return (
      x >= bounds.x &&
      x <= bounds.x + bounds.width &&
      y >= bounds.y &&
      y <= bounds.y + bounds.height
    );
  }

  private startDwell(target: GazeTarget): void {
    this.dwellState = {
      active: true,
      target,
      startTime: Date.now(),
      progress: 0,
    };

    // Emit start event
    this.emit('start', target);

    // Start progress tracking
    this.progressInterval = setInterval(() => {
      this.updateProgress();
    }, this.config.progressInterval);
  }

  private updateProgress(): void {
    if (!this.dwellState.active || !this.dwellState.startTime || !this.dwellState.target) {
      return;
    }

    const elapsed = Date.now() - this.dwellState.startTime;
    const progress = Math.min(1, elapsed / this.config.threshold);

    this.dwellState.progress = progress;

    // Emit progress event
    this.emit('progress', this.dwellState.target, progress);

    // Check for completion
    if (progress >= 1) {
      this.completeDwell();
    }
  }

  private completeDwell(): void {
    if (!this.dwellState.target) return;

    const target = this.dwellState.target;

    // Clear state
    this.clearProgressInterval();
    this.dwellState = { active: false, progress: 0 };

    // Set cooldown
    this.cooldownUntil = Date.now() + this.config.cooldownPeriod;

    // Emit complete event
    this.emit('complete', target, 1);
  }

  private cancelDwell(): void {
    if (!this.dwellState.active) return;

    const target = this.dwellState.target;
    this.clearProgressInterval();
    this.dwellState = { active: false, progress: 0 };

    if (target) {
      this.emit('cancel', target);
    }
  }

  private clearProgressInterval(): void {
    if (this.progressInterval) {
      clearInterval(this.progressInterval);
      this.progressInterval = null;
    }
  }

  private emit(event: 'start' | 'progress' | 'complete' | 'cancel', target: GazeTarget, progress?: number): void {
    let handlers: Set<DwellEventHandler>;

    switch (event) {
      case 'start':
        handlers = this.onDwellStartHandlers;
        break;
      case 'progress':
        handlers = this.onDwellProgressHandlers;
        break;
      case 'complete':
        handlers = this.onDwellCompleteHandlers;
        break;
      case 'cancel':
        handlers = this.onDwellCancelHandlers;
        break;
    }

    for (const handler of handlers) {
      try {
        handler(target, progress);
      } catch (error) {
        console.error(`Error in dwell ${event} handler:`, error);
      }
    }
  }

  // ============================================
  // Lifecycle
  // ============================================

  /**
   * Dispose of resources
   */
  dispose(): void {
    this.stop();
    this.targets.clear();
    this.onDwellStartHandlers.clear();
    this.onDwellProgressHandlers.clear();
    this.onDwellCompleteHandlers.clear();
    this.onDwellCancelHandlers.clear();
  }
}

/**
 * Create a dwell controller
 */
export function createDwellController(options: DwellControllerOptions): DwellController {
  return new DwellController(options);
}
