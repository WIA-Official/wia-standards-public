/**
 * WIA Smart Home SDK
 * 弘益人間 - Benefit All Humanity
 *
 * Comprehensive SDK for accessible smart home automation with support for:
 * - Device discovery and control
 * - Scene management
 * - Automation rules
 * - Security monitoring
 * - Energy tracking
 * - Voice assistant integration
 * - Multi-protocol support (Matter/Thread/Zigbee/Z-Wave)
 */

import {
  WIASmartHomeConfig,
  Device,
  DeviceType,
  DeviceControlOptions,
  DeviceStatus,
  Home,
  Zone,
  ZoneType,
  Automation,
  Scene,
  Action,
  Trigger,
  Condition,
  UserProfile,
  Notification,
  NotificationDelivery,
  NotificationType,
  NotificationPriority,
  AccessibilityEvent,
  EventType,
  EventSource,
  SecuritySystem,
  SecurityMode,
  EnergyMonitoring,
  VoiceAssistantConfig,
  VoiceIntentResult,
  ProtocolInfo,
  DiscoveryOptions,
  InputModality,
  OutputModality,
} from './types';

/**
 * Event callback type for real-time updates
 */
export type EventCallback = (event: AccessibilityEvent) => void;

/**
 * Main WIA Smart Home class
 */
export class WIASmartHome {
  private config: WIASmartHomeConfig;
  private eventCallbacks: Map<EventType | 'all', EventCallback[]>;
  private devices: Map<string, Device>;
  private homes: Map<string, Home>;
  private zones: Map<string, Zone>;
  private automations: Map<string, Automation>;
  private scenes: Map<string, Scene>;
  private userProfiles: Map<string, UserProfile>;

  constructor(config: WIASmartHomeConfig) {
    this.config = {
      timeout_ms: 30000,
      enable_local_discovery: true,
      enable_cloud_sync: true,
      default_protocol: 'matter',
      ...config,
    };

    this.eventCallbacks = new Map();
    this.devices = new Map();
    this.homes = new Map();
    this.zones = new Map();
    this.automations = new Map();
    this.scenes = new Map();
    this.userProfiles = new Map();
  }

  // ============================================================================
  // Device Discovery and Management
  // ============================================================================

  /**
   * Discover devices on the network
   */
  async discoverDevices(options?: DiscoveryOptions): Promise<Device[]> {
    const protocol = options?.protocol || this.config.default_protocol;
    const timeout = options?.timeout_ms || this.config.timeout_ms;

    // Simulate device discovery
    const discovered: Device[] = [];

    try {
      // In a real implementation, this would use mDNS, Matter, etc.
      const response = await this.apiCall('POST', '/devices/discover', {
        protocol,
        timeout,
        filter_types: options?.filter_device_types,
      });

      for (const deviceData of response.devices || []) {
        const device = this.parseDevice(deviceData);
        this.devices.set(device.device_id, device);
        discovered.push(device);
      }

      this.emitEvent({
        event_id: this.generateId(),
        event_type: 'system_announcement',
        timestamp: new Date().toISOString(),
        source: { type: 'system', id: 'discovery', name: 'Device Discovery' },
        data: { discovered_count: discovered.length, protocol },
      });

      return discovered;
    } catch (error) {
      throw new Error(`Device discovery failed: ${error}`);
    }
  }

  /**
   * Get device by ID
   */
  async getDevice(deviceId: string): Promise<Device | null> {
    if (this.devices.has(deviceId)) {
      return this.devices.get(deviceId)!;
    }

    try {
      const response = await this.apiCall('GET', `/devices/${deviceId}`);
      const device = this.parseDevice(response);
      this.devices.set(device.device_id, device);
      return device;
    } catch (error) {
      console.error(`Failed to get device ${deviceId}:`, error);
      return null;
    }
  }

  /**
   * Get all devices
   */
  async getDevices(homeId?: string, zoneId?: string): Promise<Device[]> {
    try {
      const params = new URLSearchParams();
      if (homeId) params.set('home_id', homeId);
      if (zoneId) params.set('zone_id', zoneId);

      const response = await this.apiCall('GET', `/devices?${params}`);
      const devices = (response.devices || []).map((d: unknown) => this.parseDevice(d));

      devices.forEach((device: Device) => {
        this.devices.set(device.device_id, device);
      });

      return devices;
    } catch (error) {
      throw new Error(`Failed to get devices: ${error}`);
    }
  }

  /**
   * Control a device
   */
  async controlDevice(
    deviceId: string,
    command: string,
    parameters?: Record<string, unknown>,
    options?: DeviceControlOptions
  ): Promise<boolean> {
    const device = await this.getDevice(deviceId);
    if (!device) {
      throw new Error(`Device ${deviceId} not found`);
    }

    if (options?.confirm) {
      // Emit confirmation request event
      this.emitEvent({
        event_id: this.generateId(),
        event_type: 'device_state_changed',
        timestamp: new Date().toISOString(),
        source: { type: 'user', id: this.config.user_profile_id || 'unknown' },
        target: { type: 'device', id: deviceId },
        data: { command, parameters, awaiting_confirmation: true },
        requires_acknowledgment: true,
      });
    }

    try {
      const response = await this.apiCall(
        'POST',
        `/devices/${deviceId}/control`,
        {
          command,
          parameters,
          timeout_ms: options?.timeout_ms,
        }
      );

      // Update device state
      if (device.current_state && response.state) {
        Object.assign(device.current_state, response.state);
      }

      // Emit state change event
      this.emitEvent({
        event_id: this.generateId(),
        event_type: 'device_state_changed',
        timestamp: new Date().toISOString(),
        source: { type: 'user', id: this.config.user_profile_id || 'unknown' },
        target: { type: 'device', id: deviceId },
        data: { command, parameters, new_state: response.state },
      });

      if (options?.announce) {
        await this.announceDeviceChange(device, command);
      }

      return response.success || false;
    } catch (error) {
      this.emitEvent({
        event_id: this.generateId(),
        event_type: 'device_accessibility_error',
        timestamp: new Date().toISOString(),
        source: { type: 'device', id: deviceId },
        severity: 'alert',
        data: { error: String(error), command },
      });
      throw new Error(`Device control failed: ${error}`);
    }
  }

  /**
   * Turn device on
   */
  async turnOn(deviceId: string, options?: DeviceControlOptions): Promise<boolean> {
    return this.controlDevice(deviceId, 'turn_on', {}, options);
  }

  /**
   * Turn device off
   */
  async turnOff(deviceId: string, options?: DeviceControlOptions): Promise<boolean> {
    return this.controlDevice(deviceId, 'turn_off', {}, options);
  }

  /**
   * Set device brightness (for lights)
   */
  async setBrightness(
    deviceId: string,
    brightness: number,
    options?: DeviceControlOptions
  ): Promise<boolean> {
    return this.controlDevice(deviceId, 'set_brightness', { brightness }, options);
  }

  /**
   * Set device color (for color lights)
   */
  async setColor(
    deviceId: string,
    hue: number,
    saturation: number,
    options?: DeviceControlOptions
  ): Promise<boolean> {
    return this.controlDevice(deviceId, 'set_color', { hue, saturation }, options);
  }

  /**
   * Set thermostat temperature
   */
  async setTemperature(
    deviceId: string,
    temperature: number,
    options?: DeviceControlOptions
  ): Promise<boolean> {
    return this.controlDevice(deviceId, 'set_temperature', { temperature }, options);
  }

  /**
   * Lock/unlock a lock
   */
  async setLockState(
    deviceId: string,
    locked: boolean,
    options?: DeviceControlOptions
  ): Promise<boolean> {
    const command = locked ? 'lock' : 'unlock';
    return this.controlDevice(deviceId, command, {}, options);
  }

  // ============================================================================
  // Scene Management
  // ============================================================================

  /**
   * Create a new scene
   */
  async createScene(scene: Omit<Scene, 'scene_id' | 'created_at' | 'updated_at'>): Promise<Scene> {
    const newScene: Scene = {
      scene_id: this.generateId(),
      created_at: new Date().toISOString(),
      updated_at: new Date().toISOString(),
      ...scene,
    };

    try {
      const response = await this.apiCall('POST', '/scenes', newScene);
      const createdScene = response.scene || newScene;
      this.scenes.set(createdScene.scene_id, createdScene);
      return createdScene;
    } catch (error) {
      throw new Error(`Failed to create scene: ${error}`);
    }
  }

  /**
   * Activate a scene
   */
  async activateScene(sceneId: string, options?: DeviceControlOptions): Promise<boolean> {
    const scene = this.scenes.get(sceneId);
    if (!scene) {
      throw new Error(`Scene ${sceneId} not found`);
    }

    try {
      for (const action of scene.actions) {
        if (action.device_id) {
          await this.controlDevice(
            action.device_id,
            action.command || 'execute',
            action.parameters,
            { ...options, announce: false }
          );

          if (action.delay_ms) {
            await this.delay(action.delay_ms);
          }
        }
      }

      this.emitEvent({
        event_id: this.generateId(),
        event_type: 'scene_activated',
        timestamp: new Date().toISOString(),
        source: { type: 'user', id: this.config.user_profile_id || 'unknown' },
        data: { scene_id: sceneId, scene_name: scene.name },
      });

      if (options?.announce || scene.accessibility_settings?.announce_activation) {
        await this.announce(scene.accessibility_settings?.announcement_text || `Scene ${scene.name} activated`);
      }

      return true;
    } catch (error) {
      throw new Error(`Failed to activate scene: ${error}`);
    }
  }

  /**
   * Get all scenes
   */
  async getScenes(homeId?: string): Promise<Scene[]> {
    try {
      const params = homeId ? `?home_id=${homeId}` : '';
      const response = await this.apiCall('GET', `/scenes${params}`);
      const scenes = response.scenes || [];
      scenes.forEach((scene: Scene) => {
        this.scenes.set(scene.scene_id, scene);
      });
      return scenes;
    } catch (error) {
      throw new Error(`Failed to get scenes: ${error}`);
    }
  }

  // ============================================================================
  // Automation Management
  // ============================================================================

  /**
   * Create a new automation
   */
  async createAutomation(
    automation: Omit<Automation, 'automation_id' | 'created_at' | 'updated_at' | 'trigger_count'>
  ): Promise<Automation> {
    const newAutomation: Automation = {
      automation_id: this.generateId(),
      created_at: new Date().toISOString(),
      updated_at: new Date().toISOString(),
      trigger_count: 0,
      enabled: true,
      ...automation,
    };

    try {
      const response = await this.apiCall('POST', '/automations', newAutomation);
      const created = response.automation || newAutomation;
      this.automations.set(created.automation_id, created);
      return created;
    } catch (error) {
      throw new Error(`Failed to create automation: ${error}`);
    }
  }

  /**
   * Update an automation
   */
  async updateAutomation(automationId: string, updates: Partial<Automation>): Promise<Automation> {
    const automation = this.automations.get(automationId);
    if (!automation) {
      throw new Error(`Automation ${automationId} not found`);
    }

    const updated: Automation = {
      ...automation,
      ...updates,
      updated_at: new Date().toISOString(),
    };

    try {
      const response = await this.apiCall('PUT', `/automations/${automationId}`, updated);
      const updatedAutomation = response.automation || updated;
      this.automations.set(automationId, updatedAutomation);
      return updatedAutomation;
    } catch (error) {
      throw new Error(`Failed to update automation: ${error}`);
    }
  }

  /**
   * Enable/disable an automation
   */
  async setAutomationEnabled(automationId: string, enabled: boolean): Promise<boolean> {
    try {
      await this.updateAutomation(automationId, { enabled });
      return true;
    } catch (error) {
      throw new Error(`Failed to set automation enabled state: ${error}`);
    }
  }

  /**
   * Trigger an automation manually
   */
  async triggerAutomation(automationId: string): Promise<boolean> {
    const automation = this.automations.get(automationId);
    if (!automation) {
      throw new Error(`Automation ${automationId} not found`);
    }

    if (!automation.enabled) {
      throw new Error(`Automation ${automationId} is disabled`);
    }

    try {
      // Check conditions
      const conditionsMet = await this.evaluateConditions(automation.conditions || []);
      if (!conditionsMet) {
        return false;
      }

      // Execute actions
      for (const action of automation.actions) {
        await this.executeAction(action);
      }

      // Update trigger count
      automation.trigger_count = (automation.trigger_count || 0) + 1;
      automation.last_triggered_at = new Date().toISOString();
      this.automations.set(automationId, automation);

      this.emitEvent({
        event_id: this.generateId(),
        event_type: 'automation_triggered',
        timestamp: new Date().toISOString(),
        source: { type: 'automation', id: automationId, name: automation.name },
        data: { automation_name: automation.name, trigger_count: automation.trigger_count },
      });

      return true;
    } catch (error) {
      this.emitEvent({
        event_id: this.generateId(),
        event_type: 'automation_failed',
        timestamp: new Date().toISOString(),
        source: { type: 'automation', id: automationId, name: automation.name },
        severity: 'alert',
        data: { error: String(error) },
      });
      throw new Error(`Failed to trigger automation: ${error}`);
    }
  }

  /**
   * Get all automations
   */
  async getAutomations(homeId?: string): Promise<Automation[]> {
    try {
      const params = homeId ? `?home_id=${homeId}` : '';
      const response = await this.apiCall('GET', `/automations${params}`);
      const automations = response.automations || [];
      automations.forEach((auto: Automation) => {
        this.automations.set(auto.automation_id, auto);
      });
      return automations;
    } catch (error) {
      throw new Error(`Failed to get automations: ${error}`);
    }
  }

  // ============================================================================
  // Security Monitoring
  // ============================================================================

  /**
   * Get security system status
   */
  async getSecuritySystem(homeId: string): Promise<SecuritySystem | null> {
    try {
      const response = await this.apiCall('GET', `/homes/${homeId}/security`);
      return response.security_system || null;
    } catch (error) {
      console.error('Failed to get security system:', error);
      return null;
    }
  }

  /**
   * Set security mode
   */
  async setSecurityMode(homeId: string, mode: SecurityMode, accessCode?: string): Promise<boolean> {
    try {
      const response = await this.apiCall('POST', `/homes/${homeId}/security/mode`, {
        mode,
        access_code: accessCode,
      });

      this.emitEvent({
        event_id: this.generateId(),
        event_type: 'system_announcement',
        timestamp: new Date().toISOString(),
        source: { type: 'system', id: homeId, name: 'Security System' },
        data: { mode, changed: true },
      });

      return response.success || false;
    } catch (error) {
      throw new Error(`Failed to set security mode: ${error}`);
    }
  }

  /**
   * Trigger emergency alert
   */
  async triggerEmergency(homeId: string, type: string, details?: Record<string, unknown>): Promise<boolean> {
    try {
      const response = await this.apiCall('POST', `/homes/${homeId}/emergency`, {
        type,
        details,
      });

      this.emitEvent({
        event_id: this.generateId(),
        event_type: 'emergency_triggered',
        timestamp: new Date().toISOString(),
        source: { type: 'user', id: this.config.user_profile_id || 'unknown' },
        home_id: homeId,
        severity: 'emergency',
        data: { type, details },
        requires_acknowledgment: true,
      });

      return response.success || false;
    } catch (error) {
      throw new Error(`Failed to trigger emergency: ${error}`);
    }
  }

  // ============================================================================
  // Energy Monitoring
  // ============================================================================

  /**
   * Get energy monitoring data for a device
   */
  async getEnergyMonitoring(deviceId: string): Promise<EnergyMonitoring | null> {
    try {
      const response = await this.apiCall('GET', `/devices/${deviceId}/energy`);
      return response.energy_monitoring || null;
    } catch (error) {
      console.error('Failed to get energy monitoring:', error);
      return null;
    }
  }

  /**
   * Get total home energy usage
   */
  async getHomeEnergyUsage(homeId: string, period: 'day' | 'week' | 'month'): Promise<number> {
    try {
      const response = await this.apiCall('GET', `/homes/${homeId}/energy?period=${period}`);
      return response.total_usage_kwh || 0;
    } catch (error) {
      console.error('Failed to get home energy usage:', error);
      return 0;
    }
  }

  // ============================================================================
  // Voice Assistant Integration
  // ============================================================================

  /**
   * Configure voice assistant
   */
  async configureVoiceAssistant(config: VoiceAssistantConfig): Promise<boolean> {
    try {
      const response = await this.apiCall('POST', '/voice/configure', config);
      return response.success || false;
    } catch (error) {
      throw new Error(`Failed to configure voice assistant: ${error}`);
    }
  }

  /**
   * Process voice command
   */
  async processVoiceCommand(transcript: string, language?: string): Promise<VoiceIntentResult> {
    try {
      const response = await this.apiCall('POST', '/voice/process', {
        transcript,
        language: language || 'en-US',
      });

      const result: VoiceIntentResult = response.result || {
        intent: 'unknown',
        confidence: 0,
      };

      this.emitEvent({
        event_id: this.generateId(),
        event_type: 'voice_command_received',
        timestamp: new Date().toISOString(),
        source: { type: 'user', id: this.config.user_profile_id || 'unknown' },
        data: { transcript, result },
        accessibility_context: {
          modalities_used: ['voice'],
        },
      });

      // Execute the command if confidence is high enough
      if (result.confidence > 0.7 && result.action && result.device_id) {
        await this.controlDevice(result.device_id, result.action, result.parameters);

        this.emitEvent({
          event_id: this.generateId(),
          event_type: 'voice_command_executed',
          timestamp: new Date().toISOString(),
          source: { type: 'user', id: this.config.user_profile_id || 'unknown' },
          target: result.device_id ? { type: 'device', id: result.device_id } : undefined,
          data: { transcript, result },
        });
      }

      return result;
    } catch (error) {
      this.emitEvent({
        event_id: this.generateId(),
        event_type: 'voice_command_failed',
        timestamp: new Date().toISOString(),
        source: { type: 'user', id: this.config.user_profile_id || 'unknown' },
        severity: 'warning',
        data: { transcript, error: String(error) },
      });
      throw new Error(`Failed to process voice command: ${error}`);
    }
  }

  // ============================================================================
  // Notification Management
  // ============================================================================

  /**
   * Send a notification
   */
  async sendNotification(notification: Omit<Notification, 'notification_id' | 'timestamp'>): Promise<string> {
    const newNotification: Notification = {
      notification_id: this.generateId(),
      timestamp: new Date().toISOString(),
      ...notification,
    };

    try {
      const response = await this.apiCall('POST', '/notifications', newNotification);
      return response.notification_id || newNotification.notification_id;
    } catch (error) {
      throw new Error(`Failed to send notification: ${error}`);
    }
  }

  /**
   * Dismiss a notification
   */
  async dismissNotification(notificationId: string): Promise<boolean> {
    try {
      const response = await this.apiCall('POST', `/notifications/${notificationId}/dismiss`, {
        dismissed_at: new Date().toISOString(),
      });
      return response.success || false;
    } catch (error) {
      throw new Error(`Failed to dismiss notification: ${error}`);
    }
  }

  // ============================================================================
  // Event Handling
  // ============================================================================

  /**
   * Register event callback
   */
  on(eventType: EventType | 'all', callback: EventCallback): void {
    if (!this.eventCallbacks.has(eventType)) {
      this.eventCallbacks.set(eventType, []);
    }
    this.eventCallbacks.get(eventType)!.push(callback);
  }

  /**
   * Unregister event callback
   */
  off(eventType: EventType | 'all', callback: EventCallback): void {
    const callbacks = this.eventCallbacks.get(eventType);
    if (callbacks) {
      const index = callbacks.indexOf(callback);
      if (index > -1) {
        callbacks.splice(index, 1);
      }
    }
  }

  /**
   * Emit an event
   */
  private emitEvent(event: AccessibilityEvent): void {
    // Call specific event type callbacks
    const specificCallbacks = this.eventCallbacks.get(event.event_type);
    if (specificCallbacks) {
      specificCallbacks.forEach(callback => callback(event));
    }

    // Call 'all' event callbacks
    const allCallbacks = this.eventCallbacks.get('all');
    if (allCallbacks) {
      allCallbacks.forEach(callback => callback(event));
    }
  }

  // ============================================================================
  // Home and Zone Management
  // ============================================================================

  /**
   * Create a new home
   */
  async createHome(home: Omit<Home, 'home_id' | 'created_at' | 'updated_at'>): Promise<Home> {
    const newHome: Home = {
      home_id: this.generateId(),
      created_at: new Date().toISOString(),
      updated_at: new Date().toISOString(),
      zones: [],
      devices: [],
      member_profiles: [],
      ...home,
    };

    try {
      const response = await this.apiCall('POST', '/homes', newHome);
      const created = response.home || newHome;
      this.homes.set(created.home_id, created);
      return created;
    } catch (error) {
      throw new Error(`Failed to create home: ${error}`);
    }
  }

  /**
   * Create a new zone
   */
  async createZone(zone: Omit<Zone, 'zone_id' | 'created_at' | 'updated_at'>): Promise<Zone> {
    const newZone: Zone = {
      zone_id: this.generateId(),
      created_at: new Date().toISOString(),
      updated_at: new Date().toISOString(),
      devices: [],
      ...zone,
    };

    try {
      const response = await this.apiCall('POST', '/zones', newZone);
      const created = response.zone || newZone;
      this.zones.set(created.zone_id, created);
      return created;
    } catch (error) {
      throw new Error(`Failed to create zone: ${error}`);
    }
  }

  // ============================================================================
  // User Profile Management
  // ============================================================================

  /**
   * Get user profile
   */
  async getUserProfile(profileId: string): Promise<UserProfile | null> {
    if (this.userProfiles.has(profileId)) {
      return this.userProfiles.get(profileId)!;
    }

    try {
      const response = await this.apiCall('GET', `/profiles/${profileId}`);
      const profile = response.profile;
      if (profile) {
        this.userProfiles.set(profileId, profile);
      }
      return profile || null;
    } catch (error) {
      console.error('Failed to get user profile:', error);
      return null;
    }
  }

  /**
   * Update user profile
   */
  async updateUserProfile(profileId: string, updates: Partial<UserProfile>): Promise<UserProfile> {
    const profile = await this.getUserProfile(profileId);
    if (!profile) {
      throw new Error(`Profile ${profileId} not found`);
    }

    const updated: UserProfile = {
      ...profile,
      ...updates,
      updated_at: new Date().toISOString(),
    };

    try {
      const response = await this.apiCall('PUT', `/profiles/${profileId}`, updated);
      const updatedProfile = response.profile || updated;
      this.userProfiles.set(profileId, updatedProfile);

      this.emitEvent({
        event_id: this.generateId(),
        event_type: 'accessibility_settings_changed',
        timestamp: new Date().toISOString(),
        source: { type: 'user', id: profileId },
        data: { updates },
      });

      return updatedProfile;
    } catch (error) {
      throw new Error(`Failed to update user profile: ${error}`);
    }
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Make API call
   */
  private async apiCall(method: string, path: string, body?: unknown): Promise<any> {
    const url = `${this.config.api_endpoint || 'https://api.wia.live/smarthome/v1'}${path}`;

    const options: RequestInit = {
      method,
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.config.api_key || ''}`,
      },
    };

    if (body) {
      options.body = JSON.stringify(body);
    }

    const response = await fetch(url, options);

    if (!response.ok) {
      throw new Error(`API call failed: ${response.status} ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Parse device data
   */
  private parseDevice(data: unknown): Device {
    return data as Device;
  }

  /**
   * Evaluate automation conditions
   */
  private async evaluateConditions(conditions: Condition[]): Promise<boolean> {
    // Simplified condition evaluation
    // In a real implementation, this would check actual device states, time, etc.
    return conditions.length === 0 || true;
  }

  /**
   * Execute an action
   */
  private async executeAction(action: Action): Promise<void> {
    switch (action.type) {
      case 'device_control':
        if (action.device_id && action.command) {
          await this.controlDevice(action.device_id, action.command, action.parameters);
        }
        break;
      case 'scene_activate':
        if (action.parameters?.scene_id) {
          await this.activateScene(String(action.parameters.scene_id));
        }
        break;
      case 'voice_announcement':
        if (action.parameters?.text) {
          await this.announce(String(action.parameters.text));
        }
        break;
      case 'delay':
        if (action.delay_ms) {
          await this.delay(action.delay_ms);
        }
        break;
      case 'notification':
        // Send notification
        break;
      default:
        console.warn(`Unknown action type: ${action.type}`);
    }
  }

  /**
   * Announce message via TTS
   */
  private async announce(message: string): Promise<void> {
    // In a real implementation, this would use TTS
    this.emitEvent({
      event_id: this.generateId(),
      event_type: 'system_announcement',
      timestamp: new Date().toISOString(),
      source: { type: 'system', id: 'tts' },
      data: { message },
    });
  }

  /**
   * Announce device change
   */
  private async announceDeviceChange(device: Device, command: string): Promise<void> {
    const message = `${device.name} ${command.replace('_', ' ')}`;
    await this.announce(message);
  }

  /**
   * Delay helper
   */
  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  /**
   * Generate unique ID
   */
  private generateId(): string {
    return `${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }
}

/**
 * Factory function to create WIASmartHome instance
 */
export function createWIASmartHome(config: WIASmartHomeConfig): WIASmartHome {
  return new WIASmartHome(config);
}

/**
 * Export all types
 */
export * from './types';
