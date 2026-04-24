/**
 * WIA Haptic Standard - Smart Home Haptic Patterns
 *
 * Pre-defined patterns for smart home device interactions.
 */

import { HapticPattern, WaveformType } from '../../api/typescript/src/types';

/**
 * Smart home haptic patterns
 */
export const SMARTHOME_PATTERNS = {
  // Light control
  LIGHT_ON: {
    id: 'home.light.on',
    name: 'Light On',
    description: 'Light turned on - rising brightness',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.3, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 175, intensity: 0.5, duration: 100, delay: 50 },
      { waveform: WaveformType.Sine, frequency: 200, intensity: 0.7, duration: 100, delay: 50 },
    ],
    totalDuration: 400,
  } as HapticPattern,

  LIGHT_OFF: {
    id: 'home.light.off',
    name: 'Light Off',
    description: 'Light turned off - falling pattern',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 200, intensity: 0.6, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.4, duration: 100, delay: 50 },
      { waveform: WaveformType.Sine, frequency: 100, intensity: 0.2, duration: 100, delay: 50 },
    ],
    totalDuration: 400,
  } as HapticPattern,

  LIGHT_DIM: {
    id: 'home.light.dim',
    name: 'Light Dimming',
    description: 'Brightness change feedback',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.4, duration: 50 },
    ],
    totalDuration: 50,
  } as HapticPattern,

  // Door/Lock control
  DOOR_LOCKED: {
    id: 'home.door.locked',
    name: 'Door Locked',
    description: 'Secure locking confirmation',
    primitives: [
      { waveform: WaveformType.Square, frequency: 200, intensity: 0.8, duration: 100 },
      { waveform: WaveformType.Square, frequency: 200, intensity: 0.6, duration: 100, delay: 100 },
    ],
    totalDuration: 300,
  } as HapticPattern,

  DOOR_UNLOCKED: {
    id: 'home.door.unlocked',
    name: 'Door Unlocked',
    description: 'Unlock confirmation',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 180, intensity: 0.6, duration: 150 },
    ],
    totalDuration: 150,
  } as HapticPattern,

  DOOR_OPEN: {
    id: 'home.door.open',
    name: 'Door Opened',
    description: 'Door opening notification',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 120, intensity: 0.4, duration: 200 },
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.5, duration: 200, delay: 50 },
    ],
    totalDuration: 450,
  } as HapticPattern,

  DOOR_CLOSED: {
    id: 'home.door.closed',
    name: 'Door Closed',
    description: 'Door closing notification',
    primitives: [
      { waveform: WaveformType.Square, frequency: 150, intensity: 0.5, duration: 100 },
    ],
    totalDuration: 100,
  } as HapticPattern,

  // Temperature
  TEMP_COLD: {
    id: 'home.temp.cold',
    name: 'Temperature Cold',
    description: 'Low temperature indication',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 80, intensity: 0.5, duration: 300 },
    ],
    totalDuration: 300,
  } as HapticPattern,

  TEMP_COMFORTABLE: {
    id: 'home.temp.comfortable',
    name: 'Temperature Comfortable',
    description: 'Comfortable temperature indication',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.4, duration: 200 },
    ],
    totalDuration: 200,
  } as HapticPattern,

  TEMP_WARM: {
    id: 'home.temp.warm',
    name: 'Temperature Warm',
    description: 'High temperature indication',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 220, intensity: 0.6, duration: 300 },
    ],
    totalDuration: 300,
  } as HapticPattern,

  THERMOSTAT_SET: {
    id: 'home.thermostat.set',
    name: 'Thermostat Set',
    description: 'Temperature target changed',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 175, intensity: 0.5, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 200, intensity: 0.6, duration: 100, delay: 50 },
    ],
    totalDuration: 250,
  } as HapticPattern,

  // Device selection
  DEVICE_HOVER: {
    id: 'home.device.hover',
    name: 'Device Hover',
    description: 'Pointing at device',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 100, intensity: 0.2, duration: 50 },
    ],
    totalDuration: 50,
  } as HapticPattern,

  DEVICE_SELECT: {
    id: 'home.device.select',
    name: 'Device Selected',
    description: 'Device selection confirmed',
    primitives: [
      { waveform: WaveformType.Square, frequency: 180, intensity: 0.7, duration: 80 },
      { waveform: WaveformType.Square, frequency: 180, intensity: 0.5, duration: 80, delay: 50 },
    ],
    totalDuration: 210,
  } as HapticPattern,

  // Alerts
  MOTION_DETECTED: {
    id: 'home.motion.detected',
    name: 'Motion Detected',
    description: 'Motion sensor triggered',
    primitives: [
      { waveform: WaveformType.Square, frequency: 150, intensity: 0.5, duration: 100 },
      { waveform: WaveformType.Square, frequency: 150, intensity: 0.5, duration: 100, delay: 100 },
    ],
    totalDuration: 300,
  } as HapticPattern,

  SMOKE_ALERT: {
    id: 'home.alert.smoke',
    name: 'Smoke Alert',
    description: 'Critical smoke/fire alert',
    primitives: [
      { waveform: WaveformType.Square, frequency: 250, intensity: 1.0, duration: 100 },
      { waveform: WaveformType.Square, frequency: 250, intensity: 1.0, duration: 100, delay: 50 },
      { waveform: WaveformType.Square, frequency: 250, intensity: 1.0, duration: 100, delay: 50 },
      { waveform: WaveformType.Square, frequency: 250, intensity: 1.0, duration: 100, delay: 50 },
    ],
    totalDuration: 450,
  } as HapticPattern,

  WATER_LEAK: {
    id: 'home.alert.water',
    name: 'Water Leak',
    description: 'Water leak detected',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 100, intensity: 0.8, duration: 200 },
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.9, duration: 200, delay: 100 },
      { waveform: WaveformType.Sine, frequency: 100, intensity: 0.8, duration: 200, delay: 100 },
    ],
    totalDuration: 800,
  } as HapticPattern,

  // Blinds/Covers
  BLIND_OPENING: {
    id: 'home.blind.opening',
    name: 'Blind Opening',
    description: 'Blind moving up',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 120, intensity: 0.3, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 140, intensity: 0.4, duration: 100, delay: 100 },
      { waveform: WaveformType.Sine, frequency: 160, intensity: 0.5, duration: 100, delay: 100 },
    ],
    totalDuration: 500,
  } as HapticPattern,

  BLIND_CLOSING: {
    id: 'home.blind.closing',
    name: 'Blind Closing',
    description: 'Blind moving down',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 160, intensity: 0.5, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 140, intensity: 0.4, duration: 100, delay: 100 },
      { waveform: WaveformType.Sine, frequency: 120, intensity: 0.3, duration: 100, delay: 100 },
    ],
    totalDuration: 500,
  } as HapticPattern,

  // Generic
  DEVICE_CONNECTED: {
    id: 'home.device.connected',
    name: 'Device Connected',
    description: 'Device came online',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 180, intensity: 0.5, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 220, intensity: 0.6, duration: 150, delay: 50 },
    ],
    totalDuration: 300,
  } as HapticPattern,

  DEVICE_DISCONNECTED: {
    id: 'home.device.disconnected',
    name: 'Device Disconnected',
    description: 'Device went offline',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.4, duration: 100 },
      { waveform: WaveformType.Sine, frequency: 100, intensity: 0.3, duration: 150, delay: 50 },
    ],
    totalDuration: 300,
  } as HapticPattern,
};

/**
 * Generate temperature haptic pattern
 *
 * Encodes temperature as frequency:
 * - Cold (< 18°C): Low frequency, slow
 * - Comfortable (18-24°C): Medium frequency
 * - Warm (> 24°C): High frequency, fast
 */
export function encodeTemperature(
  celsius: number,
  config: { min: number; max: number } = { min: 10, max: 35 }
): HapticPattern {
  const normalized = Math.max(0, Math.min(1, (celsius - config.min) / (config.max - config.min)));

  // Map to frequency (80-250 Hz)
  const frequency = Math.round(80 + normalized * 170);

  // Map to intensity (higher = stronger)
  const intensity = 0.3 + normalized * 0.4;

  return {
    id: `home.temp.${celsius}`,
    name: `Temperature ${celsius}°C`,
    description: `Haptic encoding of ${celsius}°C`,
    primitives: [
      { waveform: WaveformType.Sine, frequency, intensity, duration: 300 },
    ],
    totalDuration: 300,
  };
}

/**
 * Generate brightness change pattern
 *
 * Quick tick patterns during adjustment
 */
export function encodeBrightness(level: number): HapticPattern {
  // Level 0-100
  const normalized = level / 100;
  const frequency = Math.round(100 + normalized * 100);
  const intensity = 0.2 + normalized * 0.3;

  return {
    id: `home.brightness.${level}`,
    name: `Brightness ${level}%`,
    description: `Haptic encoding of ${level}% brightness`,
    primitives: [
      { waveform: WaveformType.Sine, frequency, intensity, duration: 50 },
    ],
    totalDuration: 50,
  };
}

/**
 * Generate position pattern for blinds/covers
 */
export function encodePosition(position: number): HapticPattern {
  // Position 0 (closed) to 100 (open)
  const pulseCount = Math.max(1, Math.round(position / 25));
  const primitives = [];

  for (let i = 0; i < pulseCount; i++) {
    primitives.push({
      waveform: WaveformType.Sine,
      frequency: 150,
      intensity: 0.4,
      duration: 50,
      delay: i > 0 ? 50 : 0,
    });
  }

  return {
    id: `home.position.${position}`,
    name: `Position ${position}%`,
    description: `Haptic encoding of ${position}% open`,
    primitives,
    totalDuration: pulseCount * 100,
  };
}
