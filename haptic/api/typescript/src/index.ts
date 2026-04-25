/**
 * WIA-SENS-003: Haptic Feedback Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * Hardware-independent haptic feedback API for assistive technology
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';

// Core exports
export * from './types';
export * from './patterns';
export * from './device';
export * from './adapters';

// ============================================================================
// Types
// ============================================================================

export interface WIAHapticConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  debug?: boolean;
}

export interface HapticPattern {
  patternId: string;
  name: string;
  duration: number;
  intensity: number;
  waveform: number[];
  repeat?: number;
}

export interface HapticDevice {
  deviceId: string;
  name: string;
  type: 'vibration' | 'force_feedback' | 'thermal' | 'electrotactile';
  connected: boolean;
  batteryLevel?: number;
  capabilities: DeviceCapabilities;
}

export interface DeviceCapabilities {
  maxIntensity: number;
  minDuration: number;
  maxDuration: number;
  channels: number;
  waveformSupport: boolean;
}

export interface APIResponse<T = unknown> {
  success: boolean;
  data?: T;
  error?: { code: string; message: string };
  timestamp: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIAHapticClient {
  private config: Required<WIAHapticConfig>;
  private eventEmitter = new EventEmitter();
  private connectedDevices: Map<string, HapticDevice> = new Map();

  constructor(config: WIAHapticConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/haptic',
      timeout: 30000,
      debug: false,
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  /**
   * Discover available haptic devices
   */
  async discoverDevices(): Promise<APIResponse<HapticDevice[]>> {
    return this.makeRequest('GET', '/devices');
  }

  /**
   * Connect to a haptic device
   */
  async connectDevice(deviceId: string): Promise<APIResponse<HapticDevice>> {
    const response = await this.makeRequest<HapticDevice>('POST', `/devices/${deviceId}/connect`);
    if (response.success && response.data) {
      this.connectedDevices.set(deviceId, response.data);
      this.eventEmitter.emit('deviceConnected', response.data);
    }
    return response;
  }

  /**
   * Disconnect from a haptic device
   */
  async disconnectDevice(deviceId: string): Promise<APIResponse<void>> {
    const response = await this.makeRequest<void>('POST', `/devices/${deviceId}/disconnect`);
    if (response.success) {
      this.connectedDevices.delete(deviceId);
      this.eventEmitter.emit('deviceDisconnected', deviceId);
    }
    return response;
  }

  /**
   * Play a haptic pattern on a device
   */
  async playPattern(deviceId: string, pattern: HapticPattern): Promise<APIResponse<void>> {
    return this.makeRequest('POST', `/devices/${deviceId}/play`, pattern);
  }

  /**
   * Stop all haptic feedback on a device
   */
  async stopFeedback(deviceId: string): Promise<APIResponse<void>> {
    return this.makeRequest('POST', `/devices/${deviceId}/stop`);
  }

  /**
   * Get predefined patterns
   */
  async getPatterns(): Promise<APIResponse<HapticPattern[]>> {
    return this.makeRequest('GET', '/patterns');
  }

  /**
   * Create a custom pattern
   */
  async createPattern(pattern: Omit<HapticPattern, 'patternId'>): Promise<APIResponse<HapticPattern>> {
    return this.makeRequest('POST', '/patterns', pattern);
  }

  /**
   * Generate a simple vibration pattern
   */
  generateVibrationPattern(duration: number, intensity: number): HapticPattern {
    return {
      patternId: `custom-${Date.now()}`,
      name: 'Custom Vibration',
      duration,
      intensity: Math.min(1, Math.max(0, intensity)),
      waveform: [intensity],
    };
  }

  /**
   * Generate a pulse pattern
   */
  generatePulsePattern(pulseCount: number, pulseDuration: number, intensity: number): HapticPattern {
    const waveform: number[] = [];
    for (let i = 0; i < pulseCount; i++) {
      waveform.push(intensity, 0);
    }
    return {
      patternId: `pulse-${Date.now()}`,
      name: 'Pulse Pattern',
      duration: pulseCount * pulseDuration * 2,
      intensity,
      waveform,
    };
  }

  /**
   * Subscribe to events
   */
  on(event: 'deviceConnected' | 'deviceDisconnected' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  /**
   * Get connected devices
   */
  getConnectedDevices(): HapticDevice[] {
    return Array.from(this.connectedDevices.values());
  }

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<APIResponse<T>> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Haptic] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'SENS-003',
          'X-WIA-Version': '1.0.0',
        },
        body: body ? JSON.stringify(body) : undefined,
      });

      const data = await response.json();
      return {
        success: response.ok,
        data: response.ok ? data : undefined,
        error: response.ok ? undefined : data,
        timestamp: new Date().toISOString(),
      };
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Unknown error';
      this.eventEmitter.emit('error', error);
      return {
        success: false,
        error: { code: 'REQUEST_FAILED', message },
        timestamp: new Date().toISOString(),
      };
    }
  }
}

/**
 * Factory function to create SDK client
 */
export function createClient(config: WIAHapticConfig): WIAHapticClient {
  return new WIAHapticClient(config);
}

export default WIAHapticClient;
