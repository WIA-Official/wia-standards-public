/**
 * WIA Haptic Standard - Apple Maps Plugin
 *
 * Integration with Apple MapKit for haptic navigation feedback.
 */

import { IHapticDevice } from '../../api/typescript/src/device';
import { HapticPattern } from '../../api/typescript/src/types';
import {
  NavigationEvent,
  NavigationHapticIntegration,
  NavigationHapticConfig,
  DEFAULT_NAVIGATION_CONFIG,
  Obstacle,
  TurnDirection,
} from './types';
import {
  NAVIGATION_PATTERNS,
  getTurnPattern,
  getApproachPattern,
  getObstaclePattern,
} from './patterns';

/**
 * Apple MapKit step info type
 */
interface MKRouteStep {
  instructions: string;
  distance: number;
  transportType: number;
  notice?: string;
}

/**
 * Apple MapKit directions response
 */
interface MKDirectionsResponse {
  routes: Array<{
    steps: MKRouteStep[];
    distance: number;
    expectedTravelTime: number;
  }>;
}

/**
 * Core Location heading
 */
interface CLHeading {
  trueHeading: number;
  magneticHeading: number;
  headingAccuracy: number;
}

/**
 * Apple Maps navigation state
 */
interface AppleMapsNavigationState {
  currentStepIndex: number;
  distanceToNextStep: number;
  distanceToDestination: number;
  heading?: CLHeading;
  isNavigating: boolean;
}

/**
 * Apple Maps Haptic Plugin
 *
 * Provides haptic feedback for Apple Maps / MapKit navigation.
 *
 * @example
 * ```swift
 * // Swift bridging example
 * let hapticPlugin = AppleMapsHapticPlugin(device: connectedDevice)
 *
 * // Connect to MapKit navigation updates
 * navigator.delegate = HapticNavigationDelegate(plugin: hapticPlugin)
 * ```
 */
export class AppleMapsHapticPlugin implements NavigationHapticIntegration {
  private device: IHapticDevice;
  private _isActive: boolean = false;
  private currentRoute?: MKDirectionsResponse['routes'][0];
  private lastStepIndex: number = -1;
  private lastWarningDistance: number = Infinity;
  private activeObstacles: Map<string, Obstacle> = new Map();

  config: NavigationHapticConfig;

  constructor(device: IHapticDevice, config?: Partial<NavigationHapticConfig>) {
    this.device = device;
    this.config = { ...DEFAULT_NAVIGATION_CONFIG, ...config };
  }

  get isActive(): boolean {
    return this._isActive;
  }

  /**
   * Turn-by-turn haptic callbacks
   */
  turnByTurn = {
    onApproachingTurn: (direction: TurnDirection, distance: number) => {
      if (!this._isActive) return;

      const pattern = getApproachPattern(distance, this.config.turnWarningDistances);
      if (pattern && distance < this.lastWarningDistance) {
        this.playPattern(pattern);
        this.lastWarningDistance = distance;
      }
    },

    onTurn: (direction: TurnDirection) => {
      if (!this._isActive) return;

      const pattern = getTurnPattern(direction);
      this.playPattern(pattern);
      this.lastWarningDistance = Infinity;
    },

    onLaneChange: (direction: 'left' | 'right') => {
      if (!this._isActive) return;

      const pattern = direction === 'left'
        ? NAVIGATION_PATTERNS.LANE_CHANGE_LEFT
        : NAVIGATION_PATTERNS.LANE_CHANGE_RIGHT;
      this.playPattern(pattern);
    },

    onDestinationNear: (distance: number) => {
      if (!this._isActive) return;

      if (distance <= 50) {
        this.playPattern(NAVIGATION_PATTERNS.DESTINATION_APPROACHING);
      }
    },

    onDestinationReached: () => {
      if (!this._isActive) return;

      this.playPattern(NAVIGATION_PATTERNS.DESTINATION_REACHED);
    },

    onOffRoute: () => {
      if (!this._isActive) return;

      this.playPattern(NAVIGATION_PATTERNS.OFF_ROUTE);
    },

    onRerouting: () => {
      if (!this._isActive) return;

      this.playPattern(NAVIGATION_PATTERNS.REROUTING);
    },
  };

  /**
   * Obstacle detection haptic callbacks
   */
  obstacleDetection = {
    onObstacleDetected: (obstacle: Obstacle) => {
      if (!this._isActive || !this.config.obstacleSettings.enabled) return;

      this.activeObstacles.set(obstacle.id, obstacle);

      if (obstacle.distance <= this.config.obstacleSettings.minDistance) {
        const pattern = getObstaclePattern(
          obstacle.distance,
          this.config.obstacleSettings.criticalDistance
        );
        this.playPattern(pattern);
      }
    },

    onObstacleUpdated: (obstacle: Obstacle) => {
      if (!this._isActive || !this.config.obstacleSettings.enabled) return;

      const previous = this.activeObstacles.get(obstacle.id);
      this.activeObstacles.set(obstacle.id, obstacle);

      if (previous &&
          previous.distance > this.config.obstacleSettings.criticalDistance &&
          obstacle.distance <= this.config.obstacleSettings.criticalDistance) {
        this.playPattern(NAVIGATION_PATTERNS.OBSTACLE_CRITICAL);
      }
    },

    onObstacleCleared: (obstacleId: string) => {
      this.activeObstacles.delete(obstacleId);
    },

    onPathClear: () => {
      if (!this._isActive) return;

      this.activeObstacles.clear();
      this.playPattern(NAVIGATION_PATTERNS.PATH_CLEAR);
    },

    onMultipleObstacles: (obstacles: Obstacle[]) => {
      if (!this._isActive || !this.config.obstacleSettings.enabled) return;

      const closest = obstacles.reduce((min, obs) =>
        obs.distance < min.distance ? obs : min
      );

      this.obstacleDetection.onObstacleDetected(closest);
    },
  };

  /**
   * Set the active route
   */
  setRoute(route: MKDirectionsResponse['routes'][0]): void {
    this.currentRoute = route;
    this.lastStepIndex = -1;
    this.lastWarningDistance = Infinity;
  }

  /**
   * Update navigation state from MapKit
   */
  updateNavigationState(state: AppleMapsNavigationState): void {
    if (!this._isActive || !this.currentRoute) return;

    const { currentStepIndex, distanceToNextStep, distanceToDestination } = state;

    // Check for step change (new turn instruction)
    if (currentStepIndex !== this.lastStepIndex && currentStepIndex >= 0) {
      const step = this.currentRoute.steps[currentStepIndex];
      if (step) {
        const direction = this.parseInstructionDirection(step.instructions);
        this.turnByTurn.onTurn(direction);
        this.lastStepIndex = currentStepIndex;
      }
    }

    // Check for approaching turn
    if (currentStepIndex >= 0 && currentStepIndex < this.currentRoute.steps.length - 1) {
      const nextStep = this.currentRoute.steps[currentStepIndex + 1];
      if (nextStep) {
        const direction = this.parseInstructionDirection(nextStep.instructions);
        this.turnByTurn.onApproachingTurn(direction, distanceToNextStep);
      }
    }

    // Check for destination approach
    if (currentStepIndex === this.currentRoute.steps.length - 1) {
      if (distanceToDestination <= 10) {
        this.turnByTurn.onDestinationReached();
      } else {
        this.turnByTurn.onDestinationNear(distanceToDestination);
      }
    }
  }

  /**
   * Parse turn direction from Apple Maps instruction text
   */
  private parseInstructionDirection(instruction: string): TurnDirection {
    const lower = instruction.toLowerCase();

    if (lower.includes('u-turn') || lower.includes('make a u')) {
      return 'u_turn';
    }
    if (lower.includes('sharp left')) {
      return 'sharp_left';
    }
    if (lower.includes('sharp right')) {
      return 'sharp_right';
    }
    if (lower.includes('slight left') || lower.includes('bear left') || lower.includes('keep left')) {
      return 'slight_left';
    }
    if (lower.includes('slight right') || lower.includes('bear right') || lower.includes('keep right')) {
      return 'slight_right';
    }
    if (lower.includes('turn left') || lower.includes('left onto')) {
      return 'left';
    }
    if (lower.includes('turn right') || lower.includes('right onto')) {
      return 'right';
    }
    if (lower.includes('continue') || lower.includes('straight') || lower.includes('head')) {
      return 'straight';
    }

    return 'straight';
  }

  /**
   * Handle off-route detection
   */
  onOffRouteDetected(): void {
    this.turnByTurn.onOffRoute();
  }

  /**
   * Handle rerouting
   */
  onRerouting(): void {
    this.turnByTurn.onRerouting();
  }

  /**
   * Integrate with ARKit LiDAR for obstacle detection
   */
  onARKitObstacles(obstacles: Array<{
    identifier: string;
    classification: string;
    distance: number;
    direction: number;
  }>): void {
    if (!this._isActive || !this.config.obstacleSettings.enabled) return;

    const wiaObstacles: Obstacle[] = obstacles.map(obs => ({
      id: obs.identifier,
      type: this.classifyARKitObstacle(obs.classification),
      direction: obs.direction,
      distance: obs.distance,
      confidence: 0.9,
    }));

    if (wiaObstacles.length > 0) {
      this.obstacleDetection.onMultipleObstacles(wiaObstacles);
    } else {
      this.obstacleDetection.onPathClear();
    }
  }

  /**
   * Map ARKit classification to WIA obstacle type
   */
  private classifyARKitObstacle(classification: string): Obstacle['type'] {
    const mapping: Record<string, Obstacle['type']> = {
      'wall': 'wall',
      'floor': 'unknown',
      'ceiling': 'unknown',
      'table': 'furniture',
      'seat': 'furniture',
      'window': 'wall',
      'door': 'wall',
      'none': 'unknown',
    };

    return mapping[classification] || 'unknown';
  }

  /**
   * Play haptic pattern on device
   */
  private async playPattern(pattern: HapticPattern): Promise<void> {
    try {
      await this.device.playPattern(pattern);
    } catch (error) {
      console.error('Failed to play haptic pattern:', error);
    }
  }

  // Control methods
  start(): void {
    this._isActive = true;
    this.lastStepIndex = -1;
    this.lastWarningDistance = Infinity;
    this.activeObstacles.clear();
  }

  stop(): void {
    this._isActive = false;
    this.device.stop();
    this.currentRoute = undefined;
    this.activeObstacles.clear();
  }

  pause(): void {
    this._isActive = false;
  }

  resume(): void {
    this._isActive = true;
  }
}

export default AppleMapsHapticPlugin;
