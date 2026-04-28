/**
 * WIA-AUTO-010: Vehicle Infotainment SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Technology Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for vehicle infotainment systems including:
 * - Multi-display management
 * - Audio system control
 * - Navigation services
 * - Smartphone integration
 * - Voice assistant
 * - Climate control
 */

import {
  InfotainmentConfig,
  InfotainmentState,
  DisplayConfig,
  DisplayZone,
  AudioProfile,
  PlaybackState,
  Route,
  Location,
  RouteOptions,
  POI,
  SmartphoneConnection,
  VoiceCommand,
  ClimateSettings,
  VehicleStatus,
  SystemEvent,
  EventHandler,
  EventType,
  InfotainmentError,
  InfotainmentErrorCode,
  ContentType,
  AudioSource,
  MediaMetadata,
  GeoLocation,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUTO-010 Vehicle Infotainment System SDK
 */
export class InfotainmentSystem {
  private version = '1.0.0';
  private config: InfotainmentConfig;
  private state: InfotainmentState;
  private eventHandlers: Map<EventType, Set<EventHandler>>;

  // Component managers
  public readonly display: DisplayManager;
  public readonly audio: AudioManager;
  public readonly navigation: NavigationManager;
  public readonly smartphone: SmartphoneManager;
  public readonly voice: VoiceAssistantManager;
  public readonly climate: ClimateManager;
  public readonly vehicle: VehicleManager;

  constructor(config?: Partial<InfotainmentConfig>) {
    // Initialize configuration with defaults
    this.config = {
      displays: config?.displays || {
        primary: {
          id: 'primary',
          type: 'primary',
          size: 15.6,
          resolution: { width: 1920, height: 1080 },
          touchEnabled: true,
          brightness: 'auto',
          mode: 'auto',
        },
      },
      audio: config?.audio || {
        channels: 8,
        speakers: {
          frontLeft: true,
          frontRight: true,
          rearLeft: true,
          rearRight: true,
          subwoofer: true,
        },
      },
      locale: config?.locale || 'en-US',
      timezone: config?.timezone || 'America/Los_Angeles',
    };

    // Initialize state
    this.state = {
      status: 'initializing',
      displayZones: new Map(),
    };

    // Initialize event handlers
    this.eventHandlers = new Map();

    // Initialize component managers
    this.display = new DisplayManager(this);
    this.audio = new AudioManager(this);
    this.navigation = new NavigationManager(this);
    this.smartphone = new SmartphoneManager(this);
    this.voice = new VoiceAssistantManager(this);
    this.climate = new ClimateManager(this);
    this.vehicle = new VehicleManager(this);

    // Mark system as ready
    this.state.status = 'ready';
    this.emit('system-ready', {});
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Get current system state
   */
  getState(): InfotainmentState {
    return { ...this.state };
  }

  /**
   * Get system configuration
   */
  getConfig(): InfotainmentConfig {
    return { ...this.config };
  }

  /**
   * Register event handler
   */
  on<T = unknown>(event: EventType, handler: EventHandler<T>): void {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, new Set());
    }
    this.eventHandlers.get(event)!.add(handler as EventHandler);
  }

  /**
   * Unregister event handler
   */
  off<T = unknown>(event: EventType, handler: EventHandler<T>): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      handlers.delete(handler as EventHandler);
    }
  }

  /**
   * Emit event
   */
  emit<T = unknown>(type: EventType, data: T, source?: string): void {
    const event: SystemEvent<T> = {
      type,
      timestamp: new Date(),
      data,
      source,
    };

    const handlers = this.eventHandlers.get(type);
    if (handlers) {
      handlers.forEach((handler) => handler(event));
    }
  }

  /**
   * Update system state
   */
  protected updateState(updates: Partial<InfotainmentState>): void {
    this.state = { ...this.state, ...updates };
  }
}

// ============================================================================
// Display Manager
// ============================================================================

export class DisplayManager {
  constructor(private system: InfotainmentSystem) {}

  /**
   * Get all displays
   */
  getDisplays(): DisplayConfig[] {
    const config = this.system.getConfig();
    return Object.values(config.displays).filter(
      (d): d is DisplayConfig => d !== undefined
    );
  }

  /**
   * Get specific display
   */
  getDisplay(id: string): DisplayConfig | undefined {
    const displays = this.getDisplays();
    return displays.find((d) => d.id === id);
  }

  /**
   * Set display brightness
   */
  setBrightness(displayId: string, level: number | 'auto'): void {
    const display = this.getDisplay(displayId);
    if (!display) {
      throw new InfotainmentError(
        InfotainmentErrorCode.DISPLAY_NOT_FOUND,
        `Display ${displayId} not found`
      );
    }

    display.brightness = level;
    this.system.emit('display-brightness-changed', { displayId, level });
  }

  /**
   * Set display mode
   */
  setMode(displayId: string, mode: 'day' | 'night' | 'auto'): void {
    const display = this.getDisplay(displayId);
    if (!display) {
      throw new InfotainmentError(
        InfotainmentErrorCode.DISPLAY_NOT_FOUND,
        `Display ${displayId} not found`
      );
    }

    display.mode = mode;
    this.system.emit('display-mode-changed', { displayId, mode });
  }

  /**
   * Set zone content
   */
  setZoneContent(displayId: string, zoneId: string, content: ContentType[]): void {
    const state = this.system.getState();
    const zones = state.displayZones || new Map();

    const zone: DisplayZone = zones.get(zoneId) || {
      id: zoneId,
      name: zoneId,
      bounds: { x: 0, y: 0, width: 1920, height: 1080 },
    };

    zone.content = content;
    zones.set(zoneId, zone);

    this.system['updateState']({ displayZones: zones });
    this.system.emit('display-zone-updated', { displayId, zoneId, content });
  }
}

// ============================================================================
// Audio Manager
// ============================================================================

export class AudioManager {
  private currentProfile: AudioProfile;
  private playback: PlaybackState;

  constructor(private system: InfotainmentSystem) {
    // Initialize default audio profile
    this.currentProfile = {
      name: 'default',
      volume: 50,
      balance: 0,
      fade: 0,
      equalizer: {
        bass: 0,
        mid: 0,
        treble: 0,
      },
      spatialAudio: false,
      anc: false,
      speedCompensation: true,
    };

    // Initialize playback state
    this.playback = {
      state: 'stopped',
      source: 'radio-fm',
      repeat: 'off',
      shuffle: false,
      volume: 50,
    };
  }

  /**
   * Play audio from source
   */
  play(source: AudioSource, metadata?: MediaMetadata): void {
    this.playback = {
      ...this.playback,
      state: 'playing',
      source,
      metadata,
    };

    this.system['updateState']({ playback: this.playback });
    this.system.emit('media-changed', this.playback);
  }

  /**
   * Pause playback
   */
  pause(): void {
    this.playback.state = 'paused';
    this.system['updateState']({ playback: this.playback });
    this.system.emit('media-changed', this.playback);
  }

  /**
   * Stop playback
   */
  stop(): void {
    this.playback.state = 'stopped';
    this.system['updateState']({ playback: this.playback });
    this.system.emit('media-changed', this.playback);
  }

  /**
   * Set volume
   */
  setVolume(level: number): void {
    this.currentProfile.volume = Math.max(0, Math.min(100, level));
    this.playback.volume = this.currentProfile.volume;
    this.system.emit('volume-changed', { volume: level });
  }

  /**
   * Set audio profile
   */
  setProfile(profile: Partial<AudioProfile>): void {
    this.currentProfile = { ...this.currentProfile, ...profile };
    this.system.emit('audio-profile-changed', this.currentProfile);
  }

  /**
   * Get current profile
   */
  getProfile(): AudioProfile {
    return { ...this.currentProfile };
  }

  /**
   * Get playback state
   */
  getPlaybackState(): PlaybackState {
    return { ...this.playback };
  }

  /**
   * Enable/disable spatial audio
   */
  setSpatialAudio(enabled: boolean): void {
    this.currentProfile.spatialAudio = enabled;
    this.system.emit('spatial-audio-changed', { enabled });
  }

  /**
   * Enable/disable active noise cancellation
   */
  setANC(enabled: boolean): void {
    this.currentProfile.anc = enabled;
    this.system.emit('anc-changed', { enabled });
  }
}

// ============================================================================
// Navigation Manager
// ============================================================================

export class NavigationManager {
  private currentRoute: Route | null = null;

  constructor(private system: InfotainmentSystem) {}

  /**
   * Navigate to destination
   */
  navigate(destination: Location | string, options?: RouteOptions): Route {
    // Convert string to location if needed
    const dest: Location =
      typeof destination === 'string'
        ? { address: destination, latitude: 0, longitude: 0 }
        : destination;

    // Create mock route
    const route: Route = {
      id: `route-${Date.now()}`,
      origin: { latitude: 37.7749, longitude: -122.4194 },
      destination: dest,
      summary: {
        distance: 15000,
        duration: 1200,
        trafficDuration: 1380,
      },
      steps: [
        {
          instruction: 'Head north',
          distance: 500,
          duration: 60,
          maneuver: 'straight',
        },
      ],
    };

    this.currentRoute = route;
    this.system['updateState']({ route });
    this.system.emit('route-updated', route);

    return route;
  }

  /**
   * Get current route
   */
  getCurrentRoute(): Route | null {
    return this.currentRoute;
  }

  /**
   * Cancel navigation
   */
  cancelNavigation(): void {
    this.currentRoute = null;
    this.system['updateState']({ route: undefined });
    this.system.emit('route-cancelled', {});
  }

  /**
   * Search for POI
   */
  searchPOI(query: string, location?: GeoLocation): POI[] {
    // Mock POI search results
    return [
      {
        id: 'poi-1',
        name: 'Sample POI',
        category: 'restaurant',
        location: {
          latitude: 37.7849,
          longitude: -122.4294,
          address: '123 Main St',
        },
        distance: 1500,
        rating: 4.5,
      },
    ];
  }

  /**
   * Get estimated time of arrival
   */
  getETA(): Date | null {
    if (!this.currentRoute) return null;

    const now = new Date();
    const eta = new Date(
      now.getTime() + this.currentRoute.summary.trafficDuration! * 1000
    );
    return eta;
  }
}

// ============================================================================
// Smartphone Manager
// ============================================================================

export class SmartphoneManager {
  private connection: SmartphoneConnection | null = null;

  constructor(private system: InfotainmentSystem) {}

  /**
   * Connect smartphone
   */
  async connect(config: SmartphoneConnection): Promise<void> {
    this.connection = {
      ...config,
      status: 'connecting',
    };

    // Simulate connection delay
    await new Promise((resolve) => setTimeout(resolve, 1000));

    this.connection.status = 'connected';
    this.system['updateState']({ smartphone: this.connection });
    this.system.emit('smartphone-connected', this.connection);
  }

  /**
   * Disconnect smartphone
   */
  disconnect(): void {
    if (this.connection) {
      this.connection.status = 'disconnected';
      this.system.emit('smartphone-disconnected', this.connection);
      this.connection = null;
      this.system['updateState']({ smartphone: undefined });
    }
  }

  /**
   * Get connection status
   */
  getConnection(): SmartphoneConnection | null {
    return this.connection;
  }

  /**
   * Start smartphone projection (CarPlay/Android Auto)
   */
  async startProjection(protocol: 'carplay' | 'android-auto'): Promise<void> {
    if (!this.connection) {
      throw new InfotainmentError(
        InfotainmentErrorCode.SMARTPHONE_CONNECTION_FAILED,
        'No smartphone connected'
      );
    }

    if (this.connection.protocol !== protocol) {
      throw new InfotainmentError(
        InfotainmentErrorCode.SMARTPHONE_PROTOCOL_UNSUPPORTED,
        `Connected device does not support ${protocol}`
      );
    }

    // Start projection mode
    this.system.emit('projection-started', { protocol });
  }
}

// ============================================================================
// Voice Assistant Manager
// ============================================================================

export class VoiceAssistantManager {
  constructor(private system: InfotainmentSystem) {}

  /**
   * Execute voice command
   */
  execute(commandText: string): void {
    const command = this.parseCommand(commandText);
    this.system.emit('voice-command', command);
    this.handleCommand(command);
  }

  /**
   * Parse voice command
   */
  private parseCommand(text: string): VoiceCommand {
    const lowerText = text.toLowerCase();

    // Simple intent detection
    let intent: VoiceCommand['intent'] = 'unknown';
    const entities: VoiceCommand['entities'] = {};

    if (lowerText.includes('navigate') || lowerText.includes('directions')) {
      intent = 'navigation';
      entities.destination = text.replace(/navigate to|directions to/i, '').trim();
    } else if (lowerText.includes('play') || lowerText.includes('music')) {
      intent = 'media';
      entities.media = text.replace(/play/i, '').trim();
    } else if (lowerText.includes('call')) {
      intent = 'phone';
      entities.contact = text.replace(/call/i, '').trim();
    } else if (lowerText.includes('temperature') || lowerText.includes('climate')) {
      intent = 'climate';
      const match = text.match(/\d+/);
      if (match) entities.temperature = parseInt(match[0]);
    }

    return {
      text,
      intent,
      entities,
      confidence: 0.85,
      timestamp: new Date(),
    };
  }

  /**
   * Handle parsed command
   */
  private handleCommand(command: VoiceCommand): void {
    switch (command.intent) {
      case 'navigation':
        if (command.entities?.destination) {
          this.system.navigation.navigate(command.entities.destination);
        }
        break;
      case 'media':
        if (command.entities?.media) {
          this.system.audio.play('streaming', {
            title: command.entities.media,
            artist: 'Unknown',
            duration: 0,
            position: 0,
          });
        }
        break;
      case 'climate':
        if (command.entities?.temperature) {
          this.system.climate.setTemperature(command.entities.temperature);
        }
        break;
    }
  }
}

// ============================================================================
// Climate Manager
// ============================================================================

export class ClimateManager {
  private settings: ClimateSettings;

  constructor(private system: InfotainmentSystem) {
    this.settings = {
      temperature: 72,
      unit: 'F',
      fanSpeed: 5,
      airDistribution: 'auto',
      recirculation: false,
      acEnabled: true,
      autoMode: true,
    };
  }

  /**
   * Set temperature
   */
  setTemperature(temp: number, unit?: 'F' | 'C'): void {
    this.settings.temperature = temp;
    if (unit) this.settings.unit = unit;
    this.system.emit('climate-changed', this.settings);
  }

  /**
   * Set fan speed
   */
  setFanSpeed(speed: number): void {
    this.settings.fanSpeed = Math.max(0, Math.min(10, speed)) as any;
    this.system.emit('climate-changed', this.settings);
  }

  /**
   * Toggle AC
   */
  toggleAC(enabled?: boolean): void {
    this.settings.acEnabled = enabled ?? !this.settings.acEnabled;
    this.system.emit('climate-changed', this.settings);
  }

  /**
   * Get climate settings
   */
  getSettings(): ClimateSettings {
    return { ...this.settings };
  }
}

// ============================================================================
// Vehicle Manager
// ============================================================================

export class VehicleManager {
  private status: VehicleStatus;

  constructor(private system: InfotainmentSystem) {
    // Initialize mock vehicle status
    this.status = {
      timestamp: new Date(),
      powertrain: {
        type: 'electric',
        batteryLevel: 85,
        range: 340,
        charging: false,
      },
      speed: 0,
      odometer: 12500,
      doors: {
        driverFront: 'closed',
        passengerFront: 'closed',
        driverRear: 'closed',
        passengerRear: 'closed',
        trunk: 'closed',
        locked: true,
      },
      lights: {
        headlights: 'auto',
        interior: false,
      },
    };
  }

  /**
   * Get vehicle status
   */
  getStatus(): VehicleStatus {
    return { ...this.status };
  }

  /**
   * Update vehicle status
   */
  updateStatus(updates: Partial<VehicleStatus>): void {
    this.status = { ...this.status, ...updates, timestamp: new Date() };
    this.system['updateState']({ vehicle: this.status });
    this.system.emit('vehicle-status-updated', this.status);
  }
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { InfotainmentSystem };
export default InfotainmentSystem;
