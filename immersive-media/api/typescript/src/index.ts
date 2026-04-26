/**
 * WIA-EDU-021: Immersive Media TypeScript SDK
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

/**
 * Main Immersive Media SDK class
 *
 * @example
 * ```typescript
 * const im = new ImmersiveMedia({
 *   apiKey: 'your-api-key',
 *   region: 'us-west-1',
 *   quality: 'high'
 * });
 * ```
 */
export class ImmersiveMedia extends EventEmitter {
  private client: AxiosInstance;
  private config: Types.ImmersiveMediaConfig;
  public readonly version = '1.0.0';
  public readonly analytics: Types.Analytics;

  constructor(config: Types.ImmersiveMediaConfig) {
    super();
    this.config = config;

    // Initialize HTTP client
    this.client = axios.create({
      baseURL: this.getBaseURL(),
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Version': '1.0.0',
      },
    });

    // Initialize analytics
    this.analytics = this.createAnalytics();

    if (config.logging) {
      this.enableLogging();
    }
  }

  private getBaseURL(): string {
    const region = this.config.region || 'us-west-1';
    return `https://api-${region}.wia.edu/v1`;
  }

  private enableLogging(): void {
    this.on('*', (event, ...args) => {
      console.log(`[WIA-IM] ${event}:`, ...args);
    });
  }

  // =========================================================================
  // 360° Video Methods
  // =========================================================================

  /**
   * Create a 360° video experience
   *
   * @param config - Video configuration
   * @returns Video360 instance
   *
   * @example
   * ```typescript
   * const video = im.create360Video({
   *   source: 'ancient-rome-8k.mp4',
   *   projection: 'equirectangular',
   *   stereoMode: 'top-bottom',
   *   controls: true
   * });
   *
   * await video.play();
   * ```
   */
  create360Video(config: Types.Video360Config): Types.Video360 {
    // Implementation would create actual Video360 instance
    // This is a reference implementation
    const video: Types.Video360 = {
      duration: 0,
      currentTime: 0,
      paused: true,

      play: async () => {
        this.emit('video-play', config.source);
      },

      pause: () => {
        this.emit('video-pause', config.source);
      },

      seek: (time: number) => {
        this.emit('video-seek', { source: config.source, time });
      },

      setQuality: (quality) => {
        this.emit('video-quality-change', quality);
      },

      addHotspot: (hotspotConfig): Types.Hotspot => {
        const hotspot: Types.Hotspot = {
          id: `hotspot-${Date.now()}`,
          config: hotspotConfig,
          show: () => {},
          hide: () => {},
          remove: () => {},
        };
        return hotspot;
      },

      removeHotspot: (id: string) => {
        this.emit('hotspot-removed', id);
      },

      on: (event, handler) => this.on(event, handler),
      off: (event, handler) => this.off(event, handler),
    };

    return video;
  }

  // =========================================================================
  // 3D Model Methods
  // =========================================================================

  /**
   * Create an interactive 3D model
   *
   * @param config - Model configuration
   * @returns Model3D instance
   *
   * @example
   * ```typescript
   * const model = im.create3DModel({
   *   source: 'dna-molecule.glb',
   *   position: [0, 1.5, -2],
   *   scale: 2,
   *   animations: ['rotate', 'unwind']
   * });
   *
   * await model.load();
   * model.playAnimation('rotate', true);
   * ```
   */
  create3DModel(config: Types.Model3DConfig): Types.Model3D {
    const model: Types.Model3D = {
      loaded: false,
      boundingBox: { min: [0, 0, 0], max: [0, 0, 0] },

      load: async () => {
        this.emit('model-load-start', config.source);
        // Actual loading implementation would go here
        this.emit('model-loaded', config.source);
      },

      playAnimation: (name, loop = false) => {
        this.emit('animation-play', { name, loop });
      },

      stopAnimation: (name) => {
        this.emit('animation-stop', name);
      },

      addAnnotation: (annotationConfig): Types.Annotation => {
        const annotation: Types.Annotation = {
          id: `annotation-${Date.now()}`,
          config: annotationConfig,
          show: () => {},
          hide: () => {},
          remove: () => {},
        };
        return annotation;
      },

      removeAnnotation: (id) => {
        this.emit('annotation-removed', id);
      },

      setMaterial: (material) => {
        this.emit('material-change', material);
      },

      rotate: (axis, angle) => {
        this.emit('model-rotate', { axis, angle });
      },

      scale: (factor) => {
        this.emit('model-scale', factor);
      },

      moveTo: (position, duration) => {
        this.emit('model-move', { position, duration });
      },

      on: (event, handler) => this.on(event, handler),
      off: (event, handler) => this.off(event, handler),
    };

    return model;
  }

  // =========================================================================
  // Spatial Audio Methods
  // =========================================================================

  /**
   * Create spatial audio environment
   *
   * @param config - Spatial audio configuration
   * @returns SpatialAudio instance
   *
   * @example
   * ```typescript
   * const audio = im.createSpatialAudio({
   *   environment: 'classroom',
   *   reverb: 0.3,
   *   attenuation: 'inverse'
   * });
   *
   * const source = audio.addSource({
   *   id: 'narration',
   *   position: [0, 1.7, 2],
   *   audio: 'narration.mp3',
   *   volume: 0.8
   * });
   * ```
   */
  createSpatialAudio(config: Types.SpatialAudioConfig): Types.SpatialAudio {
    const audio: Types.SpatialAudio = {
      addSource: (sourceConfig): Types.AudioSource => {
        const source: Types.AudioSource = {
          id: sourceConfig.id,
          playing: false,

          play: async () => {
            this.emit('audio-play', sourceConfig.id);
          },

          pause: () => {
            this.emit('audio-pause', sourceConfig.id);
          },

          stop: () => {
            this.emit('audio-stop', sourceConfig.id);
          },

          setVolume: (volume) => {
            this.emit('audio-volume', { id: sourceConfig.id, volume });
          },

          setPosition: (position) => {
            this.emit('audio-position', { id: sourceConfig.id, position });
          },

          remove: () => {
            this.emit('audio-source-removed', sourceConfig.id);
          },
        };
        return source;
      },

      removeSource: (id) => {
        this.emit('audio-source-removed', id);
      },

      setListenerPosition: (position) => {
        this.emit('listener-position', position);
      },

      setListenerOrientation: (forward, up) => {
        this.emit('listener-orientation', { forward, up });
      },

      updateSourcePosition: (id, position) => {
        this.emit('audio-position', { id, position });
      },

      setEnvironment: (environment) => {
        this.emit('environment-change', environment);
      },

      on: (event, handler) => this.on(event, handler),
      off: (event, handler) => this.off(event, handler),
    };

    return audio;
  }

  // =========================================================================
  // Haptic Feedback Methods
  // =========================================================================

  /**
   * Create haptic feedback controller
   *
   * @param config - Haptic configuration
   * @returns Haptic instance
   *
   * @example
   * ```typescript
   * const haptic = im.createHaptic({ hand: 'right' });
   *
   * await haptic.trigger({
   *   type: 'impact',
   *   intensity: 0.8,
   *   duration: 100
   * });
   * ```
   */
  createHaptic(config: Types.HapticConfig = {}): Types.Haptic {
    const haptic: Types.Haptic = {
      trigger: async (effect) => {
        this.emit('haptic-trigger', effect);
      },

      playPattern: async (pattern) => {
        this.emit('haptic-pattern', pattern);
      },

      stop: () => {
        this.emit('haptic-stop');
      },

      on: (event, handler) => this.on(event, handler),
      off: (event, handler) => this.off(event, handler),
    };

    return haptic;
  }

  // =========================================================================
  // XR Device Methods
  // =========================================================================

  /**
   * Get device capabilities
   *
   * @returns Promise with device capabilities
   */
  async getCapabilities(): Promise<Types.DeviceCapabilities> {
    // In real implementation, would query actual device
    return {
      vr: true,
      ar: true,
      handTracking: false,
      eyeTracking: false,
      passthrough: false,
      spatial6DOF: true,
    };
  }

  /**
   * Start XR session
   *
   * @param mode - Session mode (VR, AR, or inline)
   * @returns Promise with XR session
   *
   * @example
   * ```typescript
   * const session = await im.startSession('immersive-vr');
   *
   * session.on('controller-connected', (controller) => {
   *   console.log('Controller:', controller.handedness);
   * });
   * ```
   */
  async startSession(mode: Types.XRSessionMode): Promise<Types.XRSession> {
    this.emit('session-start', mode);

    const session: Types.XRSession = {
      mode,
      active: true,

      addObject: (object) => {
        this.emit('session-object-added', object);
      },

      removeObject: (object) => {
        this.emit('session-object-removed', object);
      },

      enableHandTracking: async () => {
        this.emit('hand-tracking-enabled');
      },

      enablePassthrough: async () => {
        this.emit('passthrough-enabled');
      },

      end: async () => {
        this.emit('session-end');
      },

      on: (event, handler) => this.on(event, handler),
      off: (event, handler) => this.off(event, handler),
    };

    return session;
  }

  // =========================================================================
  // Analytics
  // =========================================================================

  private createAnalytics(): Types.Analytics {
    return {
      trackEvent: (event: Types.AnalyticsEvent) => {
        this.emit('analytics-event', event);
        // Send to analytics backend
      },

      trackProgress: (config: Types.ProgressConfig) => {
        this.emit('analytics-progress', config);
        // Send to learning management system
      },

      getEngagementMetrics: async (): Promise<Types.EngagementMetrics> => {
        // Fetch from backend
        return {
          totalTime: 0,
          interactions: 0,
          completionRate: 0,
          averageScore: 0,
          commonPaths: [],
        };
      },

      on: (event, handler) => this.on(event, handler),
      off: (event, handler) => this.off(event, handler),
    };
  }
}

// Export convenience functions
export function create360Video(config: Types.Video360Config): Types.Video360 {
  const im = new ImmersiveMedia({ apiKey: '' });
  return im.create360Video(config);
}

export function create3DModel(config: Types.Model3DConfig): Types.Model3D {
  const im = new ImmersiveMedia({ apiKey: '' });
  return im.create3DModel(config);
}
