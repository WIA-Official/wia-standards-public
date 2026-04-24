/**
 * WIA Haptic Standard - Navigation Haptic Patterns
 *
 * Pre-defined patterns for navigation events.
 */

import { HapticPattern, HapticPrimitive, WaveformType } from '../../api/typescript/src/types';
import { TurnDirection } from './types';

/**
 * Navigation-specific haptic patterns
 */
export const NAVIGATION_PATTERNS = {
  // Turn approach warnings
  TURN_APPROACHING_FAR: {
    id: 'nav.turn.approaching.far',
    name: 'Turn Approaching (Far)',
    description: 'Gentle notification of upcoming turn',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.3, duration: 100 },
    ],
    totalDuration: 100,
  } as HapticPattern,

  TURN_APPROACHING_NEAR: {
    id: 'nav.turn.approaching.near',
    name: 'Turn Approaching (Near)',
    description: 'Moderate notification of nearby turn',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 175, intensity: 0.5, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 175, intensity: 0.5, duration: 100, delay: 150 },
    ],
    totalDuration: 250,
  } as HapticPattern,

  // Turn direction patterns
  TURN_LEFT: {
    id: 'nav.turn.left',
    name: 'Turn Left',
    description: 'Sharp pulse on left wrist',
    primitives: [
      { waveform: WaveformType.Square, frequency: 200, intensity: 0.8, duration: 150 },
      { waveform: WaveformType.Square, frequency: 200, intensity: 0.6, duration: 100, delay: 50 },
    ],
    totalDuration: 300,
    metadata: { location: 'left' },
  } as HapticPattern,

  TURN_RIGHT: {
    id: 'nav.turn.right',
    name: 'Turn Right',
    description: 'Sharp pulse on right wrist',
    primitives: [
      { waveform: WaveformType.Square, frequency: 200, intensity: 0.8, duration: 150 },
      { waveform: WaveformType.Square, frequency: 200, intensity: 0.6, duration: 100, delay: 50 },
    ],
    totalDuration: 300,
    metadata: { location: 'right' },
  } as HapticPattern,

  TURN_SLIGHT_LEFT: {
    id: 'nav.turn.slight_left',
    name: 'Slight Left',
    description: 'Gentle pulse on left',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 175, intensity: 0.5, duration: 200 },
    ],
    totalDuration: 200,
    metadata: { location: 'left' },
  } as HapticPattern,

  TURN_SLIGHT_RIGHT: {
    id: 'nav.turn.slight_right',
    name: 'Slight Right',
    description: 'Gentle pulse on right',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 175, intensity: 0.5, duration: 200 },
    ],
    totalDuration: 200,
    metadata: { location: 'right' },
  } as HapticPattern,

  TURN_SHARP_LEFT: {
    id: 'nav.turn.sharp_left',
    name: 'Sharp Left',
    description: 'Strong double pulse on left',
    primitives: [
      { waveform: WaveformType.Square, frequency: 220, intensity: 1.0, duration: 100 },
      { waveform: WaveformType.Square, frequency: 220, intensity: 1.0, duration: 100, delay: 50 },
      { waveform: WaveformType.Square, frequency: 220, intensity: 0.8, duration: 100, delay: 50 },
    ],
    totalDuration: 400,
    metadata: { location: 'left' },
  } as HapticPattern,

  TURN_SHARP_RIGHT: {
    id: 'nav.turn.sharp_right',
    name: 'Sharp Right',
    description: 'Strong double pulse on right',
    primitives: [
      { waveform: WaveformType.Square, frequency: 220, intensity: 1.0, duration: 100 },
      { waveform: WaveformType.Square, frequency: 220, intensity: 1.0, duration: 100, delay: 50 },
      { waveform: WaveformType.Square, frequency: 220, intensity: 0.8, duration: 100, delay: 50 },
    ],
    totalDuration: 400,
    metadata: { location: 'right' },
  } as HapticPattern,

  U_TURN: {
    id: 'nav.turn.u_turn',
    name: 'U-Turn',
    description: 'Sweeping pattern indicating reversal',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 180, intensity: 0.7, duration: 150 },
      { waveform: WaveformType.Sine, frequency: 200, intensity: 0.8, duration: 150, delay: 100 },
      { waveform: WaveformType.Sine, frequency: 180, intensity: 0.7, duration: 150, delay: 100 },
    ],
    totalDuration: 650,
  } as HapticPattern,

  GO_STRAIGHT: {
    id: 'nav.straight',
    name: 'Continue Straight',
    description: 'Subtle confirmation to continue',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.3, duration: 100 },
    ],
    totalDuration: 100,
  } as HapticPattern,

  // Lane change
  LANE_CHANGE_LEFT: {
    id: 'nav.lane.left',
    name: 'Lane Change Left',
    description: 'Smooth transition pulse left',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 160, intensity: 0.4, duration: 200 },
    ],
    totalDuration: 200,
    metadata: { location: 'left' },
  } as HapticPattern,

  LANE_CHANGE_RIGHT: {
    id: 'nav.lane.right',
    name: 'Lane Change Right',
    description: 'Smooth transition pulse right',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 160, intensity: 0.4, duration: 200 },
    ],
    totalDuration: 200,
    metadata: { location: 'right' },
  } as HapticPattern,

  // Destination
  DESTINATION_APPROACHING: {
    id: 'nav.destination.approaching',
    name: 'Destination Approaching',
    description: 'Rising pattern indicating arrival',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.4, duration: 150 },
      { waveform: WaveformType.Sine, frequency: 175, intensity: 0.5, duration: 150, delay: 100 },
      { waveform: WaveformType.Sine, frequency: 200, intensity: 0.6, duration: 150, delay: 100 },
    ],
    totalDuration: 650,
  } as HapticPattern,

  DESTINATION_REACHED: {
    id: 'nav.destination.reached',
    name: 'Destination Reached',
    description: 'Triumphant completion pattern',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 200, intensity: 0.8, duration: 200 },
      { waveform: WaveformType.Sine, frequency: 250, intensity: 0.9, duration: 200, delay: 100 },
      { waveform: WaveformType.Sine, frequency: 300, intensity: 1.0, duration: 300, delay: 100 },
    ],
    totalDuration: 900,
  } as HapticPattern,

  // Route status
  OFF_ROUTE: {
    id: 'nav.status.off_route',
    name: 'Off Route',
    description: 'Warning pattern for wrong direction',
    primitives: [
      { waveform: WaveformType.Square, frequency: 100, intensity: 0.7, duration: 200 },
      { waveform: WaveformType.Square, frequency: 100, intensity: 0.7, duration: 200, delay: 100 },
      { waveform: WaveformType.Square, frequency: 100, intensity: 0.7, duration: 200, delay: 100 },
    ],
    totalDuration: 800,
  } as HapticPattern,

  REROUTING: {
    id: 'nav.status.rerouting',
    name: 'Rerouting',
    description: 'Calculating new route pattern',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 120, intensity: 0.4, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 140, intensity: 0.4, duration: 100, delay: 150 },
      { waveform: WaveformType.Sine, frequency: 160, intensity: 0.4, duration: 100, delay: 150 },
    ],
    totalDuration: 600,
  } as HapticPattern,

  // Obstacle warnings
  OBSTACLE_DETECTED: {
    id: 'nav.obstacle.detected',
    name: 'Obstacle Detected',
    description: 'Alert for detected obstacle',
    primitives: [
      { waveform: WaveformType.Square, frequency: 180, intensity: 0.6, duration: 100 },
      { waveform: WaveformType.Square, frequency: 180, intensity: 0.6, duration: 100, delay: 50 },
    ],
    totalDuration: 250,
  } as HapticPattern,

  OBSTACLE_CRITICAL: {
    id: 'nav.obstacle.critical',
    name: 'Obstacle Critical',
    description: 'Urgent warning for close obstacle',
    primitives: [
      { waveform: WaveformType.Square, frequency: 220, intensity: 1.0, duration: 50 },
      { waveform: WaveformType.Square, frequency: 220, intensity: 1.0, duration: 50, delay: 30 },
      { waveform: WaveformType.Square, frequency: 220, intensity: 1.0, duration: 50, delay: 30 },
      { waveform: WaveformType.Square, frequency: 220, intensity: 1.0, duration: 50, delay: 30 },
    ],
    totalDuration: 290,
  } as HapticPattern,

  PATH_CLEAR: {
    id: 'nav.obstacle.clear',
    name: 'Path Clear',
    description: 'All clear confirmation',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 200, intensity: 0.5, duration: 150 },
    ],
    totalDuration: 150,
  } as HapticPattern,
};

/**
 * Get turn pattern by direction
 */
export function getTurnPattern(direction: TurnDirection): HapticPattern {
  switch (direction) {
    case 'left':
      return NAVIGATION_PATTERNS.TURN_LEFT;
    case 'right':
      return NAVIGATION_PATTERNS.TURN_RIGHT;
    case 'slight_left':
      return NAVIGATION_PATTERNS.TURN_SLIGHT_LEFT;
    case 'slight_right':
      return NAVIGATION_PATTERNS.TURN_SLIGHT_RIGHT;
    case 'sharp_left':
      return NAVIGATION_PATTERNS.TURN_SHARP_LEFT;
    case 'sharp_right':
      return NAVIGATION_PATTERNS.TURN_SHARP_RIGHT;
    case 'u_turn':
      return NAVIGATION_PATTERNS.U_TURN;
    case 'straight':
    default:
      return NAVIGATION_PATTERNS.GO_STRAIGHT;
  }
}

/**
 * Get obstacle pattern based on distance
 */
export function getObstaclePattern(distance: number, criticalDistance: number = 1): HapticPattern {
  if (distance <= criticalDistance) {
    return NAVIGATION_PATTERNS.OBSTACLE_CRITICAL;
  }
  return NAVIGATION_PATTERNS.OBSTACLE_DETECTED;
}

/**
 * Get approach pattern based on distance
 */
export function getApproachPattern(
  distance: number,
  thresholds: { far: number; near: number }
): HapticPattern | null {
  if (distance <= thresholds.near) {
    return NAVIGATION_PATTERNS.TURN_APPROACHING_NEAR;
  }
  if (distance <= thresholds.far) {
    return NAVIGATION_PATTERNS.TURN_APPROACHING_FAR;
  }
  return null;
}
