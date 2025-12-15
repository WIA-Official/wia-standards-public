/**
 * WIA Eye Gaze Standard - Smart Home Integration
 *
 * Integration with smart home systems for gaze-based control.
 * Supports Matter/Thread protocol and Home Assistant.
 *
 * 弘益人間 - 널리 인간을 이롭게
 *
 * @module @anthropics/wia-eye-gaze/integrations/smarthome
 */

import type { GazePoint, GazeEvent, Vector3D } from '../../api/typescript/src/types';

/**
 * Smart device types
 */
export enum DeviceType {
  Light = 'light',
  Switch = 'switch',
  Thermostat = 'thermostat',
  Lock = 'lock',
  Sensor = 'sensor',
  Camera = 'camera',
  Speaker = 'speaker',
  TV = 'tv',
  Fan = 'fan',
  Blind = 'blind',
  Outlet = 'outlet',
  Unknown = 'unknown',
}

/**
 * Smart device representation
 */
export interface SmartDevice {
  /** Device ID */
  id: string;
  /** Device name */
  name: string;
  /** Device type */
  type: DeviceType;
  /** 3D position in room (meters) */
  position: Vector3D;
  /** Device state */
  state: DeviceState;
  /** Available actions */
  actions: DeviceAction[];
  /** Room/zone */
  room?: string;
  /** Matter node ID */
  matterNodeId?: string;
  /** Home Assistant entity ID */
  hassEntityId?: string;
}

/**
 * Device state
 */
export interface DeviceState {
  /** Is device on/active */
  on?: boolean;
  /** Brightness (0-100) */
  brightness?: number;
  /** Color (HSV or RGB) */
  color?: { h: number; s: number; v: number } | { r: number; g: number; b: number };
  /** Temperature setting (°C) */
  temperature?: number;
  /** Current temperature (°C) */
  currentTemperature?: number;
  /** Lock state */
  locked?: boolean;
  /** Position (0-100, for blinds/curtains) */
  position?: number;
  /** Volume (0-100) */
  volume?: number;
  /** Custom state properties */
  [key: string]: unknown;
}

/**
 * Device action
 */
export interface DeviceAction {
  /** Action name */
  name: string;
  /** Action type */
  type: ActionType;
  /** Parameters */
  params?: ActionParams;
}

export type ActionType =
  | 'toggle'
  | 'turn_on'
  | 'turn_off'
  | 'set_brightness'
  | 'set_color'
  | 'set_temperature'
  | 'lock'
  | 'unlock'
  | 'open'
  | 'close'
  | 'set_position'
  | 'volume_up'
  | 'volume_down'
  | 'play'
  | 'pause'
  | 'custom';

export interface ActionParams {
  brightness?: number;
  color?: { h: number; s: number; v: number };
  temperature?: number;
  position?: number;
  volume?: number;
  [key: string]: unknown;
}

/**
 * Room/zone definition
 */
export interface Room {
  id: string;
  name: string;
  bounds?: {
    min: Vector3D;
    max: Vector3D;
  };
  devices: string[];
}

/**
 * Gaze-device alignment result
 */
export interface GazeDeviceAlignment {
  /** Is gaze aligned with a device */
  aligned: boolean;
  /** Aligned device */
  device?: SmartDevice;
  /** Alignment angle (degrees) */
  angle: number;
  /** Alignment confidence (0-1) */
  confidence: number;
  /** Distance to device (meters) */
  distance?: number;
}

/**
 * Smart home event
 */
export interface SmartHomeEvent {
  type: 'device_selected' | 'action_triggered' | 'state_changed';
  device: SmartDevice;
  action?: DeviceAction;
  previousState?: DeviceState;
  timestamp: number;
}

/**
 * Home Assistant webhook payload
 */
export interface HomeAssistantWebhook {
  event_type: string;
  event_data: {
    entity_id?: string;
    gaze_event?: GazeEvent;
    device_id?: string;
    action?: string;
  };
}

/**
 * Smart Home Integration Configuration
 */
export interface SmartHomeConfig {
  /** Gaze alignment tolerance (degrees) */
  alignmentTolerance: number;
  /** Dwell time to trigger action (ms) */
  dwellTime: number;
  /** User position in room (meters) */
  userPosition: Vector3D;
  /** User facing direction */
  userDirection: Vector3D;
  /** Home Assistant URL */
  hassUrl?: string;
  /** Home Assistant token */
  hassToken?: string;
  /** Matter controller available */
  matterEnabled: boolean;
}

/**
 * Smart Home Integration class
 *
 * Provides gaze-based control for smart home devices.
 */
export class SmartHomeIntegration {
  /** Registered devices */
  private devices: Map<string, SmartDevice> = new Map();

  /** Rooms */
  private rooms: Map<string, Room> = new Map();

  /** Configuration */
  private config: SmartHomeConfig = {
    alignmentTolerance: 15,
    dwellTime: 1000,
    userPosition: { x: 0, y: 1.5, z: 0 },
    userDirection: { x: 0, y: 0, z: 1 },
    matterEnabled: false,
  };

  /** Dwell tracking */
  private dwellDevice: SmartDevice | null = null;
  private dwellStartTime = 0;

  /** Event callbacks */
  private callbacks: ((event: SmartHomeEvent) => void)[] = [];

  /** Matter support flag */
  private _matterSupport = false;

  constructor(config?: Partial<SmartHomeConfig>) {
    if (config) {
      this.config = { ...this.config, ...config };
    }
  }

  /**
   * Detect device that user is looking at
   */
  detectGazedDevice(direction: Vector3D): SmartDevice | null {
    const alignment = this.findAlignedDevice(direction);
    if (!alignment.aligned || !alignment.device) {
      this.resetDwell();
      return null;
    }

    // Handle dwell
    if (alignment.device.id === this.dwellDevice?.id) {
      const elapsed = Date.now() - this.dwellStartTime;
      if (elapsed >= this.config.dwellTime) {
        // Dwell complete - trigger default action
        this.triggerDefaultAction(alignment.device);
        this.resetDwell();
      }
    } else {
      this.dwellDevice = alignment.device;
      this.dwellStartTime = Date.now();
    }

    return alignment.device;
  }

  /**
   * Find device aligned with gaze direction
   */
  private findAlignedDevice(direction: Vector3D): GazeDeviceAlignment {
    const normalizedDirection = this.normalizeVector(direction);
    let bestAlignment: GazeDeviceAlignment = {
      aligned: false,
      angle: 180,
      confidence: 0,
    };

    for (const device of this.devices.values()) {
      // Calculate direction from user to device
      const toDevice = {
        x: device.position.x - this.config.userPosition.x,
        y: device.position.y - this.config.userPosition.y,
        z: device.position.z - this.config.userPosition.z,
      };

      const normalizedToDevice = this.normalizeVector(toDevice);
      const distance = this.vectorMagnitude(toDevice);

      // Calculate angle between gaze and device direction
      const angle = this.angleBetweenVectors(normalizedDirection, normalizedToDevice);

      if (angle < this.config.alignmentTolerance && angle < bestAlignment.angle) {
        const confidence = 1 - angle / this.config.alignmentTolerance;
        bestAlignment = {
          aligned: true,
          device,
          angle,
          confidence,
          distance,
        };
      }
    }

    return bestAlignment;
  }

  /**
   * Control a device
   */
  controlDevice(device: SmartDevice, action: DeviceAction): boolean {
    // Find the action in device's available actions
    const availableAction = device.actions.find((a) => a.type === action.type);
    if (!availableAction) {
      console.warn(`Action ${action.type} not available for device ${device.name}`);
      return false;
    }

    // Emit event
    const previousState = { ...device.state };
    this.emitEvent({
      type: 'action_triggered',
      device,
      action,
      previousState,
      timestamp: Date.now(),
    });

    // Execute action based on integration type
    if (device.hassEntityId && this.config.hassUrl) {
      return this.executeHomeAssistantAction(device, action);
    } else if (device.matterNodeId && this._matterSupport) {
      return this.executeMatterAction(device, action);
    }

    // Simulate local state change
    this.updateDeviceState(device, action);
    return true;
  }

  /**
   * Trigger default action for device type
   */
  private triggerDefaultAction(device: SmartDevice): void {
    let action: DeviceAction;

    switch (device.type) {
      case DeviceType.Light:
      case DeviceType.Switch:
      case DeviceType.Outlet:
        action = { name: 'Toggle', type: 'toggle' };
        break;

      case DeviceType.Lock:
        action = device.state.locked
          ? { name: 'Unlock', type: 'unlock' }
          : { name: 'Lock', type: 'lock' };
        break;

      case DeviceType.Blind:
        action = device.state.position && device.state.position > 50
          ? { name: 'Close', type: 'close' }
          : { name: 'Open', type: 'open' };
        break;

      case DeviceType.TV:
      case DeviceType.Speaker:
        action = device.state.on
          ? { name: 'Pause', type: 'pause' }
          : { name: 'Play', type: 'play' };
        break;

      default:
        action = { name: 'Toggle', type: 'toggle' };
    }

    this.controlDevice(device, action);
  }

  /**
   * Update device state locally
   */
  private updateDeviceState(device: SmartDevice, action: DeviceAction): void {
    const newState = { ...device.state };

    switch (action.type) {
      case 'toggle':
        newState.on = !device.state.on;
        break;
      case 'turn_on':
        newState.on = true;
        break;
      case 'turn_off':
        newState.on = false;
        break;
      case 'set_brightness':
        newState.brightness = action.params?.brightness;
        break;
      case 'set_temperature':
        newState.temperature = action.params?.temperature;
        break;
      case 'lock':
        newState.locked = true;
        break;
      case 'unlock':
        newState.locked = false;
        break;
      case 'open':
        newState.position = 100;
        break;
      case 'close':
        newState.position = 0;
        break;
    }

    device.state = newState;
    this.devices.set(device.id, device);

    this.emitEvent({
      type: 'state_changed',
      device,
      timestamp: Date.now(),
    });
  }

  /**
   * Execute action via Home Assistant
   */
  private executeHomeAssistantAction(device: SmartDevice, action: DeviceAction): boolean {
    if (!this.config.hassUrl || !this.config.hassToken) {
      console.warn('Home Assistant not configured');
      return false;
    }

    const domain = this.getHassDomain(device.type);
    const service = this.getHassService(action.type);

    const payload = {
      entity_id: device.hassEntityId,
      ...action.params,
    };

    // In real implementation: Make HTTP request to Home Assistant API
    console.log(`Home Assistant: ${domain}/${service}`, payload);

    return true;
  }

  /**
   * Execute action via Matter protocol
   */
  private executeMatterAction(device: SmartDevice, action: DeviceAction): boolean {
    if (!this._matterSupport) {
      console.warn('Matter not available');
      return false;
    }

    // In real implementation: Use Matter SDK to send command
    console.log(`Matter: ${device.matterNodeId} ${action.type}`);

    return true;
  }

  /**
   * Send event to Home Assistant webhook
   */
  homeAssistantWebhook(event: GazeEvent): void {
    if (!this.config.hassUrl) return;

    const payload: HomeAssistantWebhook = {
      event_type: 'wia_gaze_event',
      event_data: {
        gaze_event: event,
      },
    };

    // In real implementation: POST to Home Assistant webhook
    console.log('Home Assistant webhook:', payload);
  }

  // Device management

  /**
   * Register a smart device
   */
  registerDevice(device: SmartDevice): void {
    this.devices.set(device.id, device);
  }

  /**
   * Get device by ID
   */
  getDevice(id: string): SmartDevice | undefined {
    return this.devices.get(id);
  }

  /**
   * Get all devices
   */
  getAllDevices(): SmartDevice[] {
    return Array.from(this.devices.values());
  }

  /**
   * Get devices in room
   */
  getDevicesInRoom(roomId: string): SmartDevice[] {
    const room = this.rooms.get(roomId);
    if (!room) return [];

    return room.devices
      .map((id) => this.devices.get(id))
      .filter((d): d is SmartDevice => d !== undefined);
  }

  /**
   * Remove device
   */
  removeDevice(id: string): void {
    this.devices.delete(id);
  }

  // Room management

  /**
   * Add room
   */
  addRoom(room: Room): void {
    this.rooms.set(room.id, room);
  }

  /**
   * Get room
   */
  getRoom(id: string): Room | undefined {
    return this.rooms.get(id);
  }

  // Configuration

  /**
   * Get Matter support status
   */
  get matterSupport(): boolean {
    return this._matterSupport;
  }

  /**
   * Enable Matter support
   */
  enableMatter(): void {
    // In real implementation: Initialize Matter SDK
    this._matterSupport = true;
    this.config.matterEnabled = true;
    console.log('Matter support enabled');
  }

  /**
   * Configure Home Assistant
   */
  configureHomeAssistant(url: string, token: string): void {
    this.config.hassUrl = url;
    this.config.hassToken = token;
  }

  /**
   * Update user position
   */
  setUserPosition(position: Vector3D, direction?: Vector3D): void {
    this.config.userPosition = position;
    if (direction) {
      this.config.userDirection = direction;
    }
  }

  // Event handling

  /**
   * Register event callback
   */
  onEvent(callback: (event: SmartHomeEvent) => void): void {
    this.callbacks.push(callback);
  }

  private emitEvent(event: SmartHomeEvent): void {
    for (const callback of this.callbacks) {
      callback(event);
    }
  }

  // Helpers

  private resetDwell(): void {
    this.dwellDevice = null;
    this.dwellStartTime = 0;
  }

  private normalizeVector(v: Vector3D): Vector3D {
    const mag = this.vectorMagnitude(v);
    if (mag === 0) return { x: 0, y: 0, z: 1 };
    return { x: v.x / mag, y: v.y / mag, z: v.z / mag };
  }

  private vectorMagnitude(v: Vector3D): number {
    return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  }

  private angleBetweenVectors(a: Vector3D, b: Vector3D): number {
    const dot = a.x * b.x + a.y * b.y + a.z * b.z;
    return Math.acos(Math.max(-1, Math.min(1, dot))) * (180 / Math.PI);
  }

  private getHassDomain(type: DeviceType): string {
    const domains: Record<DeviceType, string> = {
      [DeviceType.Light]: 'light',
      [DeviceType.Switch]: 'switch',
      [DeviceType.Thermostat]: 'climate',
      [DeviceType.Lock]: 'lock',
      [DeviceType.Sensor]: 'sensor',
      [DeviceType.Camera]: 'camera',
      [DeviceType.Speaker]: 'media_player',
      [DeviceType.TV]: 'media_player',
      [DeviceType.Fan]: 'fan',
      [DeviceType.Blind]: 'cover',
      [DeviceType.Outlet]: 'switch',
      [DeviceType.Unknown]: 'homeassistant',
    };
    return domains[type];
  }

  private getHassService(actionType: ActionType): string {
    const services: Record<ActionType, string> = {
      toggle: 'toggle',
      turn_on: 'turn_on',
      turn_off: 'turn_off',
      set_brightness: 'turn_on',
      set_color: 'turn_on',
      set_temperature: 'set_temperature',
      lock: 'lock',
      unlock: 'unlock',
      open: 'open_cover',
      close: 'close_cover',
      set_position: 'set_cover_position',
      volume_up: 'volume_up',
      volume_down: 'volume_down',
      play: 'media_play',
      pause: 'media_pause',
      custom: 'custom',
    };
    return services[actionType];
  }

  /**
   * Cleanup
   */
  dispose(): void {
    this.devices.clear();
    this.rooms.clear();
    this.callbacks = [];
    this.resetDwell();
  }
}

/**
 * Create example smart home setup
 */
export function createExampleSetup(): { devices: SmartDevice[]; rooms: Room[] } {
  const devices: SmartDevice[] = [
    {
      id: 'light-living-1',
      name: 'Living Room Light',
      type: DeviceType.Light,
      position: { x: 2, y: 2.5, z: 3 },
      state: { on: true, brightness: 80 },
      actions: [
        { name: 'Toggle', type: 'toggle' },
        { name: 'Turn On', type: 'turn_on' },
        { name: 'Turn Off', type: 'turn_off' },
        { name: 'Set Brightness', type: 'set_brightness' },
      ],
      room: 'living-room',
    },
    {
      id: 'tv-living',
      name: 'Living Room TV',
      type: DeviceType.TV,
      position: { x: 0, y: 1.2, z: 4 },
      state: { on: false, volume: 30 },
      actions: [
        { name: 'Toggle', type: 'toggle' },
        { name: 'Play', type: 'play' },
        { name: 'Pause', type: 'pause' },
      ],
      room: 'living-room',
    },
    {
      id: 'thermostat-main',
      name: 'Main Thermostat',
      type: DeviceType.Thermostat,
      position: { x: -2, y: 1.5, z: 0 },
      state: { temperature: 22, currentTemperature: 21 },
      actions: [{ name: 'Set Temperature', type: 'set_temperature' }],
      room: 'hallway',
    },
  ];

  const rooms: Room[] = [
    {
      id: 'living-room',
      name: 'Living Room',
      devices: ['light-living-1', 'tv-living'],
    },
    {
      id: 'hallway',
      name: 'Hallway',
      devices: ['thermostat-main'],
    },
  ];

  return { devices, rooms };
}

export default SmartHomeIntegration;
