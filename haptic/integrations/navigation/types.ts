/**
 * WIA Haptic Standard - Navigation Integration Types
 *
 * Common types for navigation app integrations.
 */

import { HapticPattern, BodyLocation } from '../../api/typescript/src/types';

/**
 * Cardinal and relative directions
 */
export type TurnDirection = 'left' | 'right' | 'slight_left' | 'slight_right' |
                            'sharp_left' | 'sharp_right' | 'u_turn' | 'straight';

/**
 * Navigation event types
 */
export type NavigationEventType =
  | 'route_started'
  | 'route_updated'
  | 'approaching_turn'
  | 'turn_now'
  | 'lane_change'
  | 'destination_approaching'
  | 'destination_reached'
  | 'off_route'
  | 'rerouting'
  | 'obstacle_detected'
  | 'path_clear';

/**
 * Navigation event data
 */
export interface NavigationEvent {
  type: NavigationEventType;
  timestamp: number;
  direction?: TurnDirection;
  distance?: number;           // meters
  bearing?: number;            // degrees (0-360)
  instruction?: string;
  confidence?: number;         // 0-1
}

/**
 * Obstacle information from sensors
 */
export interface Obstacle {
  id: string;
  type: ObstacleType;
  direction: number;           // degrees (0-360, 0=forward)
  distance: number;            // meters
  velocity?: number;           // m/s (negative = approaching)
  size?: ObstacleSize;
  confidence: number;          // 0-1
}

export type ObstacleType =
  | 'wall'
  | 'person'
  | 'vehicle'
  | 'bicycle'
  | 'animal'
  | 'pole'
  | 'furniture'
  | 'stairs_up'
  | 'stairs_down'
  | 'curb'
  | 'drop'
  | 'unknown';

export type ObstacleSize = 'small' | 'medium' | 'large';

/**
 * Turn-by-turn navigation haptic callbacks
 */
export interface TurnByTurnHaptics {
  onApproachingTurn(direction: TurnDirection, distance: number): void;
  onTurn(direction: TurnDirection): void;
  onLaneChange(direction: 'left' | 'right'): void;
  onDestinationNear(distance: number): void;
  onDestinationReached(): void;
  onOffRoute(): void;
  onRerouting(): void;
}

/**
 * Obstacle detection haptic callbacks
 */
export interface ObstacleDetectionHaptics {
  onObstacleDetected(obstacle: Obstacle): void;
  onObstacleUpdated(obstacle: Obstacle): void;
  onObstacleCleared(obstacleId: string): void;
  onPathClear(): void;
  onMultipleObstacles(obstacles: Obstacle[]): void;
}

/**
 * Navigation haptic integration interface
 */
export interface NavigationHapticIntegration {
  // Configuration
  config: NavigationHapticConfig;

  // Turn-by-turn callbacks
  turnByTurn: TurnByTurnHaptics;

  // Obstacle detection callbacks
  obstacleDetection: ObstacleDetectionHaptics;

  // Control methods
  start(): void;
  stop(): void;
  pause(): void;
  resume(): void;

  // State
  isActive: boolean;
}

/**
 * Configuration for navigation haptics
 */
export interface NavigationHapticConfig {
  // Distance thresholds for turn warnings (meters)
  turnWarningDistances: {
    far: number;      // First warning
    near: number;     // Second warning
    immediate: number; // Turn now
  };

  // Obstacle detection settings
  obstacleSettings: {
    enabled: boolean;
    minDistance: number;      // Ignore obstacles beyond this (meters)
    criticalDistance: number; // High urgency zone (meters)
    updateInterval: number;   // ms between updates
  };

  // Haptic intensity based on urgency
  intensityLevels: {
    info: number;      // 0-1
    warning: number;
    critical: number;
  };

  // Body locations for navigation
  locations: {
    direction: BodyLocation[];
    warning: BodyLocation[];
    confirmation: BodyLocation[];
  };
}

/**
 * Default navigation haptic configuration
 */
export const DEFAULT_NAVIGATION_CONFIG: NavigationHapticConfig = {
  turnWarningDistances: {
    far: 200,
    near: 50,
    immediate: 10,
  },
  obstacleSettings: {
    enabled: true,
    minDistance: 10,
    criticalDistance: 1,
    updateInterval: 100,
  },
  intensityLevels: {
    info: 0.3,
    warning: 0.6,
    critical: 1.0,
  },
  locations: {
    direction: [BodyLocation.WristLeftDorsal, BodyLocation.WristRightDorsal],
    warning: [BodyLocation.WristLeftDorsal, BodyLocation.WristRightDorsal],
    confirmation: [BodyLocation.WristLeftDorsal],
  },
};
