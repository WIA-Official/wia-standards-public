/**
 * WIA-EDU-013: XR Education Standard - TypeScript SDK
 * @version 1.0.0
 * @description SDK for building XR educational experiences
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import {
  XREducationOptions,
  XRSessionConfig,
  VirtualClassroom,
  VirtualLab,
  XRAccessibility,
  XRAnalytics,
  MultiplayerSession,
  APIResponse,
  SessionResponse,
  XREvent,
  Participant,
  LearningAsset
} from './types';

export * from './types';

/**
 * Main XR Education SDK Class
 */
export class XREducation {
  private apiKey: string;
  private baseUrl: string;
  private client: AxiosInstance;
  private config: XRSessionConfig;
  private eventHandlers: Map<string, Function[]>;

  constructor(options: XREducationOptions) {
    this.apiKey = options.apiKey || '';
    this.baseUrl = options.baseUrl || 'https://api.wia.education/xr';
    this.eventHandlers = new Map();

    // Initialize axios client
    this.client = axios.create({
      baseURL: this.baseUrl,
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json'
      }
    });

    // Initialize XR session config
    this.config = {
      mode: options.mode,
      deviceType: options.device || 'webxr',
      trackingMode: '6dof',
      environmentBlending: options.mode === 'immersive-vr' ? 'opaque' : 'alpha-blend',
      capabilities: {
        handTracking: true,
        eyeTracking: false,
        spatialAudio: true,
        haptics: true,
        voiceCommands: false,
        passthrough: options.mode === 'immersive-ar' || options.mode === 'immersive-mr'
      }
    };
  }

  /**
   * Create a virtual classroom environment
   */
  async createVirtualClassroom(config: {
    capacity?: number;
    dimensions?: { width: number; length: number; height: number };
    features?: Partial<VirtualClassroom['features']>;
  }): Promise<VirtualClassroom> {
    try {
      const response = await this.client.post<APIResponse<VirtualClassroom>>('/classrooms', {
        capacity: config.capacity || 30,
        dimensions: config.dimensions || { width: 20, length: 15, height: 5 },
        features: {
          spatialAudio: true,
          whiteboard3D: true,
          screenSharing: true,
          handRaising: true,
          breakoutRooms: true,
          ...config.features
        }
      });

      if (response.data.success && response.data.data) {
        return response.data.data;
      } else {
        throw new Error(response.data.error?.message || 'Failed to create classroom');
      }
    } catch (error) {
      console.error('Error creating virtual classroom:', error);
      throw error;
    }
  }

  /**
   * Create a virtual laboratory
   */
  async createVirtualLab(config: {
    subject: 'chemistry' | 'physics' | 'biology' | 'engineering' | 'medical';
    scenario: string;
    safetyLevel?: 'low' | 'medium' | 'high' | 'critical';
  }): Promise<VirtualLab> {
    try {
      const response = await this.client.post<APIResponse<VirtualLab>>('/labs', {
        subject: config.subject,
        scenario: config.scenario,
        safety: {
          level: config.safetyLevel || 'medium',
          protectiveEquipment: this.getRequiredEquipment(config.subject),
          hazardWarnings: true,
          emergencyProcedures: true
        },
        physics: {
          enabled: true,
          gravity: { x: 0, y: -9.81, z: 0 },
          collisionDetection: true,
          fluidDynamics: config.subject === 'chemistry' || config.subject === 'physics',
          chemicalReactions: config.subject === 'chemistry'
        }
      });

      if (response.data.success && response.data.data) {
        return response.data.data;
      } else {
        throw new Error(response.data.error?.message || 'Failed to create lab');
      }
    } catch (error) {
      console.error('Error creating virtual lab:', error);
      throw error;
    }
  }

  /**
   * Configure accessibility settings
   */
  setAccessibility(settings: Partial<XRAccessibility>): void {
    const defaultSettings: XRAccessibility = {
      visual: {
        colorblindMode: 'none',
        highContrast: false,
        textToSpeech: false,
        subtitles: false,
        fontSize: 'medium'
      },
      motor: {
        seatedMode: false,
        reduceReach: false,
        snapTurning: false,
        teleportMovement: false,
        oneHandedMode: false
      },
      comfort: {
        vignette: 0,
        reducedMotion: false,
        fieldOfViewReduction: 0
      },
      cognitive: {
        simplifiedUI: false,
        guidedMode: false,
        pauseAnytime: true
      }
    };

    // Merge settings
    const mergedSettings = {
      visual: { ...defaultSettings.visual, ...settings.visual },
      motor: { ...defaultSettings.motor, ...settings.motor },
      comfort: { ...defaultSettings.comfort, ...settings.comfort },
      cognitive: { ...defaultSettings.cognitive, ...settings.cognitive }
    };

    // Apply settings (implementation would interact with XR runtime)
    console.log('Accessibility settings applied:', mergedSettings);
  }

  /**
   * Track learning analytics
   */
  async trackAnalytics(sessionId: string): Promise<XRAnalytics> {
    try {
      const response = await this.client.get<APIResponse<XRAnalytics>>(
        `/analytics/${sessionId}`
      );

      if (response.data.success && response.data.data) {
        return response.data.data;
      } else {
        throw new Error(response.data.error?.message || 'Failed to fetch analytics');
      }
    } catch (error) {
      console.error('Error fetching analytics:', error);
      throw error;
    }
  }

  /**
   * Start an XR learning session
   */
  async startSession(config: {
    classroomId?: string;
    labId?: string;
    studentId: string;
  }): Promise<SessionResponse> {
    try {
      const response = await this.client.post<APIResponse<SessionResponse>>('/sessions', {
        ...config,
        mode: this.config.mode,
        deviceType: this.config.deviceType,
        timestamp: new Date().toISOString()
      });

      if (response.data.success && response.data.data) {
        return response.data.data;
      } else {
        throw new Error(response.data.error?.message || 'Failed to start session');
      }
    } catch (error) {
      console.error('Error starting session:', error);
      throw error;
    }
  }

  /**
   * Create a collaborative multiplayer session
   */
  async createMultiplayerSession(config: {
    maxCapacity?: number;
    voiceChat?: boolean;
  }): Promise<MultiplayerSession> {
    try {
      const response = await this.client.post<APIResponse<MultiplayerSession>>('/multiplayer', {
        maxCapacity: config.maxCapacity || 30,
        networking: {
          protocol: 'webrtc',
          latency: 0,
          bandwidth: 0
        },
        sync: {
          positionUpdate: 20,
          physicsSync: true,
          stateSync: true,
          voiceChat: config.voiceChat !== false
        }
      });

      if (response.data.success && response.data.data) {
        return response.data.data;
      } else {
        throw new Error(response.data.error?.message || 'Failed to create multiplayer session');
      }
    } catch (error) {
      console.error('Error creating multiplayer session:', error);
      throw error;
    }
  }

  /**
   * Register event handler
   */
  on(event: string, handler: Function): void {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, []);
    }
    this.eventHandlers.get(event)!.push(handler);
  }

  /**
   * Emit event to registered handlers
   */
  private emit(event: string, data: any): void {
    const handlers = this.eventHandlers.get(event) || [];
    handlers.forEach(handler => handler(data));
  }

  /**
   * Load 3D asset
   */
  async loadAsset(assetId: string): Promise<LearningAsset> {
    try {
      const response = await this.client.get<APIResponse<LearningAsset>>(`/assets/${assetId}`);

      if (response.data.success && response.data.data) {
        return response.data.data;
      } else {
        throw new Error(response.data.error?.message || 'Failed to load asset');
      }
    } catch (error) {
      console.error('Error loading asset:', error);
      throw error;
    }
  }

  /**
   * Add participant to session
   */
  async addParticipant(sessionId: string, participant: Participant): Promise<void> {
    try {
      await this.client.post(`/sessions/${sessionId}/participants`, participant);
    } catch (error) {
      console.error('Error adding participant:', error);
      throw error;
    }
  }

  /**
   * End XR session
   */
  async endSession(sessionId: string): Promise<void> {
    try {
      await this.client.post(`/sessions/${sessionId}/end`);
      this.emit('session-ended', { sessionId });
    } catch (error) {
      console.error('Error ending session:', error);
      throw error;
    }
  }

  /**
   * Helper: Get required safety equipment for lab type
   */
  private getRequiredEquipment(subject: string): string[] {
    const equipmentMap: Record<string, string[]> = {
      chemistry: ['goggles', 'gloves', 'lab-coat'],
      physics: ['goggles'],
      biology: ['goggles', 'gloves'],
      engineering: ['hard-hat', 'goggles', 'gloves'],
      medical: ['gloves', 'mask', 'gown']
    };

    return equipmentMap[subject] || [];
  }
}

/**
 * Factory function for creating XR Education instance
 */
export function createXREducation(options: XREducationOptions): XREducation {
  return new XREducation(options);
}

/**
 * Version info
 */
export const VERSION = '1.0.0';
export const STANDARD = 'WIA-EDU-013';

/**
 * Default export
 */
export default XREducation;
