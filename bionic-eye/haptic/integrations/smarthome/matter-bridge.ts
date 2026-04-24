/**
 * WIA Haptic Standard - Matter/Thread Bridge
 *
 * Integration with Matter protocol for smart home haptic feedback.
 */

import { IHapticDevice } from '../../api/typescript/src/device';
import { HapticPattern } from '../../api/typescript/src/types';
import {
  SmartDevice,
  SmartDeviceType,
  SmartDeviceEvent,
  SmartHomeHapticIntegration,
  SmartHomeHapticConfig,
  DEFAULT_SMARTHOME_CONFIG,
  DeviceAction,
} from './types';
import {
  SMARTHOME_PATTERNS,
  encodeTemperature,
  encodeBrightness,
  encodePosition,
} from './patterns';

/**
 * Matter device cluster types
 */
interface MatterEndpoint {
  id: number;
  deviceType: number;
  clusters: Map<number, MatterCluster>;
}

interface MatterCluster {
  id: number;
  attributes: Map<number, unknown>;
  commands: string[];
}

/**
 * Matter Bridge for Haptic Integration
 *
 * Provides haptic feedback for Matter/Thread smart home devices.
 *
 * @example
 * ```typescript
 * const device = await HapticDeviceManager.connect('bluetooth', deviceId);
 * const bridge = new MatterHapticBridge(device);
 *
 * // Discover devices
 * const devices = await bridge.discoverDevices();
 *
 * // Control with haptic feedback
 * await bridge.controlDevice(devices[0].id, { type: 'toggle' });
 *
 * // Spatial device selection
 * bridge.pointToDevice(45); // Point at 45 degrees
 * bridge.confirmSelection();
 * ```
 */
export class MatterHapticBridge implements SmartHomeHapticIntegration {
  private hapticDevice: IHapticDevice;
  private devices: Map<string, SmartDevice> = new Map();
  private selectedDevice: SmartDevice | null = null;
  private selectionTimer: ReturnType<typeof setTimeout> | null = null;
  private lastPointedDevice: SmartDevice | null = null;

  config: SmartHomeHapticConfig;

  constructor(
    hapticDevice: IHapticDevice,
    config?: Partial<SmartHomeHapticConfig>
  ) {
    this.hapticDevice = hapticDevice;
    this.config = { ...DEFAULT_SMARTHOME_CONFIG, ...config };
  }

  /**
   * Discover Matter devices on the network
   */
  async discoverDevices(): Promise<SmartDevice[]> {
    // In real implementation, this would use Matter SDK
    // For now, return mock devices for testing

    const mockDevices: SmartDevice[] = [
      {
        id: 'light-1',
        name: 'Living Room Light',
        type: 'light',
        room: 'Living Room',
        state: { on: false, brightness: 100 },
        direction: 0,
        distance: 3,
        reachable: true,
      },
      {
        id: 'thermostat-1',
        name: 'Thermostat',
        type: 'thermostat',
        room: 'Living Room',
        state: { temperature: 22, targetTemperature: 21 },
        direction: 90,
        distance: 5,
        reachable: true,
      },
      {
        id: 'door-lock-1',
        name: 'Front Door',
        type: 'door_lock',
        room: 'Entrance',
        state: { locked: true },
        direction: 180,
        distance: 8,
        reachable: true,
      },
    ];

    this.devices.clear();
    for (const device of mockDevices) {
      this.devices.set(device.id, device);
    }

    return Array.from(this.devices.values());
  }

  /**
   * Get device by ID
   */
  getDevice(id: string): SmartDevice | undefined {
    return this.devices.get(id);
  }

  /**
   * Get devices by room
   */
  getDevicesByRoom(room: string): SmartDevice[] {
    return Array.from(this.devices.values())
      .filter(d => d.room?.toLowerCase() === room.toLowerCase());
  }

  /**
   * Get devices by type
   */
  getDevicesByType(type: SmartDeviceType): SmartDevice[] {
    return Array.from(this.devices.values())
      .filter(d => d.type === type);
  }

  /**
   * Point to device at given direction
   */
  pointToDevice(direction: number): SmartDevice | null {
    if (!this.config.spatialSelection.enabled) return null;

    // Normalize direction to 0-360
    const normalizedDirection = ((direction % 360) + 360) % 360;

    // Find device within angle threshold
    let closestDevice: SmartDevice | null = null;
    let closestAngle = Infinity;

    for (const device of this.devices.values()) {
      if (device.direction === undefined || !device.reachable) continue;

      const angleDiff = Math.abs(this.angleDifference(normalizedDirection, device.direction));

      if (angleDiff <= this.config.spatialSelection.angleThreshold && angleDiff < closestAngle) {
        closestAngle = angleDiff;
        closestDevice = device;
      }
    }

    // Haptic feedback for device hover
    if (closestDevice && closestDevice !== this.lastPointedDevice) {
      this.playPattern(SMARTHOME_PATTERNS.DEVICE_HOVER);
      this.lastPointedDevice = closestDevice;

      // Start selection timer
      if (this.selectionTimer) {
        clearTimeout(this.selectionTimer);
      }
      this.selectionTimer = setTimeout(() => {
        if (this.lastPointedDevice === closestDevice) {
          this.selectDevice(closestDevice!);
        }
      }, this.config.spatialSelection.confirmationDelay);
    } else if (!closestDevice) {
      this.lastPointedDevice = null;
      if (this.selectionTimer) {
        clearTimeout(this.selectionTimer);
        this.selectionTimer = null;
      }
    }

    return closestDevice;
  }

  /**
   * Select a device
   */
  selectDevice(device: SmartDevice): void {
    this.selectedDevice = device;
    this.playPattern(SMARTHOME_PATTERNS.DEVICE_SELECT);
  }

  /**
   * Confirm current selection
   */
  confirmSelection(): void {
    if (this.selectedDevice) {
      // Default action based on device type
      const action = this.getDefaultAction(this.selectedDevice);
      if (action) {
        this.controlDevice(this.selectedDevice.id, action);
      }
    }
  }

  /**
   * Cancel current selection
   */
  cancelSelection(): void {
    this.selectedDevice = null;
    this.lastPointedDevice = null;
    if (this.selectionTimer) {
      clearTimeout(this.selectionTimer);
      this.selectionTimer = null;
    }
  }

  /**
   * Control a device with haptic feedback
   */
  async controlDevice(deviceId: string, action: DeviceAction): Promise<void> {
    const device = this.devices.get(deviceId);
    if (!device) {
      throw new Error(`Device not found: ${deviceId}`);
    }

    const previousState = { ...device.state };

    // Execute action (in real implementation, send to Matter network)
    this.executeAction(device, action);

    // Provide haptic feedback if enabled
    if (this.config.feedbackOnControl) {
      const pattern = this.getActionPattern(device, action);
      if (pattern) {
        await this.playPattern(pattern);
      }
    }

    // Emit state change event
    this.onDeviceEvent({
      deviceId,
      type: 'state_changed',
      previousState,
      newState: device.state,
      timestamp: Date.now(),
    });
  }

  /**
   * Handle device event
   */
  onDeviceEvent(event: SmartDeviceEvent): void {
    if (!this.config.feedbackOnStateChange) return;

    const device = this.devices.get(event.deviceId);
    if (!device) return;

    switch (event.type) {
      case 'state_changed':
        this.handleStateChange(device, event);
        break;
      case 'connected':
        this.playPattern(SMARTHOME_PATTERNS.DEVICE_CONNECTED);
        break;
      case 'disconnected':
        this.playPattern(SMARTHOME_PATTERNS.DEVICE_DISCONNECTED);
        break;
      case 'triggered':
        this.playPattern(SMARTHOME_PATTERNS.MOTION_DETECTED);
        break;
      case 'alert':
        this.handleAlert(device, event);
        break;
    }
  }

  /**
   * Handle state change event
   */
  private handleStateChange(device: SmartDevice, event: SmartDeviceEvent): void {
    const { previousState, newState } = event;

    switch (device.type) {
      case 'light':
        if (previousState?.on !== newState.on) {
          this.playPattern(newState.on
            ? SMARTHOME_PATTERNS.LIGHT_ON
            : SMARTHOME_PATTERNS.LIGHT_OFF);
        } else if (previousState?.brightness !== newState.brightness && newState.brightness !== undefined) {
          this.playPattern(encodeBrightness(newState.brightness));
        }
        break;

      case 'door_lock':
        if (previousState?.locked !== newState.locked) {
          this.playPattern(newState.locked
            ? SMARTHOME_PATTERNS.DOOR_LOCKED
            : SMARTHOME_PATTERNS.DOOR_UNLOCKED);
        }
        break;

      case 'door_sensor':
        if (previousState?.open !== newState.open) {
          this.playPattern(newState.open
            ? SMARTHOME_PATTERNS.DOOR_OPEN
            : SMARTHOME_PATTERNS.DOOR_CLOSED);
        }
        break;

      case 'thermostat':
        if (previousState?.targetTemperature !== newState.targetTemperature) {
          this.playPattern(SMARTHOME_PATTERNS.THERMOSTAT_SET);
        }
        if (newState.temperature !== undefined) {
          this.playPattern(encodeTemperature(newState.temperature, this.config.temperatureRange));
        }
        break;

      case 'blind':
        if (previousState?.position !== newState.position && newState.position !== undefined) {
          this.playPattern(newState.position > (previousState?.position ?? 0)
            ? SMARTHOME_PATTERNS.BLIND_OPENING
            : SMARTHOME_PATTERNS.BLIND_CLOSING);
        }
        break;
    }
  }

  /**
   * Handle alert event
   */
  private handleAlert(device: SmartDevice, event: SmartDeviceEvent): void {
    // Use high-priority alert patterns
    if (device.name.toLowerCase().includes('smoke') ||
        device.name.toLowerCase().includes('fire')) {
      this.playPattern(SMARTHOME_PATTERNS.SMOKE_ALERT);
    } else if (device.name.toLowerCase().includes('water') ||
               device.name.toLowerCase().includes('leak')) {
      this.playPattern(SMARTHOME_PATTERNS.WATER_LEAK);
    }
  }

  /**
   * Execute device action (mock implementation)
   */
  private executeAction(device: SmartDevice, action: DeviceAction): void {
    switch (action.type) {
      case 'toggle':
        device.state.on = !device.state.on;
        break;
      case 'on':
        device.state.on = true;
        break;
      case 'off':
        device.state.on = false;
        break;
      case 'setBrightness':
        device.state.brightness = action.value;
        break;
      case 'setTemperature':
        device.state.targetTemperature = action.value;
        break;
      case 'setPosition':
        device.state.position = action.value;
        break;
      case 'lock':
        device.state.locked = true;
        break;
      case 'unlock':
        device.state.locked = false;
        break;
      case 'open':
        device.state.open = true;
        break;
      case 'close':
        device.state.open = false;
        break;
    }
  }

  /**
   * Get default action for device type
   */
  private getDefaultAction(device: SmartDevice): DeviceAction | null {
    switch (device.type) {
      case 'light':
      case 'switch':
      case 'outlet':
        return { type: 'toggle' };
      case 'door_lock':
        return device.state.locked ? { type: 'unlock' } : { type: 'lock' };
      default:
        return null;
    }
  }

  /**
   * Get haptic pattern for action
   */
  private getActionPattern(device: SmartDevice, action: DeviceAction): HapticPattern | null {
    switch (action.type) {
      case 'toggle':
      case 'on':
        if (device.type === 'light') return SMARTHOME_PATTERNS.LIGHT_ON;
        break;
      case 'off':
        if (device.type === 'light') return SMARTHOME_PATTERNS.LIGHT_OFF;
        break;
      case 'lock':
        return SMARTHOME_PATTERNS.DOOR_LOCKED;
      case 'unlock':
        return SMARTHOME_PATTERNS.DOOR_UNLOCKED;
      case 'setTemperature':
        return encodeTemperature(action.value, this.config.temperatureRange);
      case 'setBrightness':
        return encodeBrightness(action.value);
      case 'setPosition':
        return encodePosition(action.value);
    }
    return null;
  }

  /**
   * Calculate angle difference
   */
  private angleDifference(a: number, b: number): number {
    let diff = b - a;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
  }

  /**
   * Play haptic pattern
   */
  private async playPattern(pattern: HapticPattern): Promise<void> {
    try {
      await this.hapticDevice.playPattern(pattern);
    } catch (error) {
      console.error('Failed to play haptic pattern:', error);
    }
  }
}

export default MatterHapticBridge;
