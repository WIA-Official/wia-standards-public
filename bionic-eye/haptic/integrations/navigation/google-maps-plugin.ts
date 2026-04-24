/**
 * WIA Haptic Standard - Google Maps Plugin
 *
 * Integration with Google Maps Navigation API for haptic feedback.
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
 * Google Maps navigation event (from Maps SDK)
 */
interface GoogleMapsNavigationEvent {
  type: string;
  step?: {
    maneuver?: string;
    distance?: { value: number };
    instructions?: string;
  };
  route?: {
    legs?: Array<{
      distance: { value: number };
      duration: { value: number };
    }>;
  };
}

/**
 * Google Maps Haptic Plugin
 *
 * Provides haptic feedback for Google Maps navigation events.
 *
 * @example
 * ```typescript
 * const device = await HapticDeviceManager.connect('bluetooth', deviceId);
 * const plugin = new GoogleMapsHapticPlugin(device);
 *
 * // Connect to Google Maps SDK events
 * googleMapsNavigator.addListener('navigation_event', (event) => {
 *   plugin.onNavigationEvent(event);
 * });
 *
 * plugin.start();
 * ```
 */
export class GoogleMapsHapticPlugin implements NavigationHapticIntegration {
  private device: IHapticDevice;
  private _isActive: boolean = false;
  private lastTurnWarningDistance: number = Infinity;
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
      if (pattern && distance < this.lastTurnWarningDistance) {
        this.playPattern(pattern);
        this.lastTurnWarningDistance = distance;
      }
    },

    onTurn: (direction: TurnDirection) => {
      if (!this._isActive) return;

      const pattern = getTurnPattern(direction);
      this.playPattern(pattern);
      this.lastTurnWarningDistance = Infinity;
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

      // Alert if obstacle entered critical zone
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

      // Find the closest obstacle
      const closest = obstacles.reduce((min, obs) =>
        obs.distance < min.distance ? obs : min
      );

      this.obstacleDetection.onObstacleDetected(closest);
    },
  };

  /**
   * Handle Google Maps navigation event
   */
  onNavigationEvent(event: GoogleMapsNavigationEvent): void {
    if (!this._isActive) return;

    const navEvent = this.translateEvent(event);
    if (!navEvent) return;

    switch (navEvent.type) {
      case 'approaching_turn':
        if (navEvent.direction && navEvent.distance !== undefined) {
          this.turnByTurn.onApproachingTurn(navEvent.direction, navEvent.distance);
        }
        break;
      case 'turn_now':
        if (navEvent.direction) {
          this.turnByTurn.onTurn(navEvent.direction);
        }
        break;
      case 'lane_change':
        if (navEvent.direction === 'left' || navEvent.direction === 'right') {
          this.turnByTurn.onLaneChange(navEvent.direction);
        }
        break;
      case 'destination_approaching':
        if (navEvent.distance !== undefined) {
          this.turnByTurn.onDestinationNear(navEvent.distance);
        }
        break;
      case 'destination_reached':
        this.turnByTurn.onDestinationReached();
        break;
      case 'off_route':
        this.turnByTurn.onOffRoute();
        break;
      case 'rerouting':
        this.turnByTurn.onRerouting();
        break;
    }
  }

  /**
   * Translate Google Maps event to WIA navigation event
   */
  private translateEvent(event: GoogleMapsNavigationEvent): NavigationEvent | null {
    const maneuver = event.step?.maneuver;
    const distance = event.step?.distance?.value;

    if (!maneuver) return null;

    const direction = this.maneuverToDirection(maneuver);

    // Determine event type based on distance
    if (distance !== undefined && distance > 10) {
      return {
        type: 'approaching_turn',
        timestamp: Date.now(),
        direction,
        distance,
        instruction: event.step?.instructions,
      };
    }

    return {
      type: 'turn_now',
      timestamp: Date.now(),
      direction,
      distance,
      instruction: event.step?.instructions,
    };
  }

  /**
   * Convert Google Maps maneuver to turn direction
   */
  private maneuverToDirection(maneuver: string): TurnDirection {
    const mapping: Record<string, TurnDirection> = {
      'turn-left': 'left',
      'turn-right': 'right',
      'turn-slight-left': 'slight_left',
      'turn-slight-right': 'slight_right',
      'turn-sharp-left': 'sharp_left',
      'turn-sharp-right': 'sharp_right',
      'uturn-left': 'u_turn',
      'uturn-right': 'u_turn',
      'straight': 'straight',
      'keep-left': 'slight_left',
      'keep-right': 'slight_right',
      'ramp-left': 'slight_left',
      'ramp-right': 'slight_right',
      'fork-left': 'slight_left',
      'fork-right': 'slight_right',
      'merge': 'straight',
      'roundabout-left': 'left',
      'roundabout-right': 'right',
    };

    return mapping[maneuver] || 'straight';
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
    this.lastTurnWarningDistance = Infinity;
    this.activeObstacles.clear();
  }

  stop(): void {
    this._isActive = false;
    this.device.stop();
    this.activeObstacles.clear();
  }

  pause(): void {
    this._isActive = false;
  }

  resume(): void {
    this._isActive = true;
  }
}

export default GoogleMapsHapticPlugin;
