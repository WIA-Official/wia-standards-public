/**
 * WIA-EDU-025: Entertainment Robot Standard
 * TypeScript SDK v1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 * © 2025 WIA - World Certification Industry Association
 * MIT License
 */

import type {
  WIAEntertainmentRobotConfig,
  InteractiveStory,
  PerformanceProgram,
  TherapeuticSession,
  EdutainmentGame,
  Session,
  EmotionalState,
  RobotProfile,
  APIResponse,
  ConsentRequest,
  PrivacySettings,
} from './types';

export * from './types';

/**
 * Main SDK Class for WIA Entertainment Robot Standard
 */
export class WIAEntertainmentRobot {
  private config: Required<WIAEntertainmentRobotConfig>;
  private baseURL: string;

  constructor(config: WIAEntertainmentRobotConfig) {
    this.config = {
      apiKey: config.apiKey,
      robotId: config.robotId || '',
      baseURL: config.baseURL || 'https://api.wiastandards.com/entertainment-robot/v1',
      environment: config.environment || 'production',
      timeout: config.timeout || 30000,
      retryAttempts: config.retryAttempts || 3,
      privacyMode: config.privacyMode || 'strict',
    };

    this.baseURL = this.config.environment === 'sandbox'
      ? 'https://sandbox-api.wiastandards.com/entertainment-robot/v1'
      : this.config.baseURL;
  }

  /**
   * Stories API - Interactive storytelling
   */
  get stories() {
    return {
      list: async (filters?: {
        genre?: string;
        ageRange?: string;
        language?: string;
        page?: number;
        limit?: number;
      }): Promise<APIResponse<InteractiveStory[]>> => {
        const params = new URLSearchParams(filters as any);
        return this.request(`/stories?${params}`);
      },

      load: async (storyId: string): Promise<InteractiveStory> => {
        const response = await this.request<InteractiveStory>(`/stories/${storyId}`);
        return response.data!;
      },

      startSession: async (
        storyId: string,
        options: {
          userId: string;
          parentalConsent: boolean;
          sessionMetadata?: any;
        }
      ): Promise<Session> => {
        const response = await this.request<Session>(`/stories/${storyId}/sessions`, {
          method: 'POST',
          body: JSON.stringify(options),
        });
        return response.data!;
      },

      makeChoice: async (
        sessionId: string,
        choiceId: string,
        optionId: string
      ): Promise<void> => {
        await this.request(`/sessions/${sessionId}/choices`, {
          method: 'POST',
          body: JSON.stringify({ choiceId, optionId }),
        });
      },
    };
  }

  /**
   * Performances API - Theater and shows
   */
  get performances() {
    return {
      list: async (filters?: {
        type?: string;
        educationalFocus?: string;
        duration?: number;
      }): Promise<APIResponse<PerformanceProgram[]>> => {
        const params = new URLSearchParams(filters as any);
        return this.request(`/performances?${params}`);
      },

      load: async (performanceId: string): Promise<PerformanceProgram> => {
        const response = await this.request<PerformanceProgram>(`/performances/${performanceId}`);
        return response.data!;
      },

      schedule: async (options: {
        performanceId: string;
        startTime: string;
        audienceSize: number;
        educationalFocus?: string;
      }): Promise<Session> => {
        const response = await this.request<Session>(`/performances/${options.performanceId}/schedule`, {
          method: 'POST',
          body: JSON.stringify(options),
        });
        return response.data!;
      },

      coordinateWith: async (sessionId: string, robotIds: string[]): Promise<void> => {
        await this.request(`/sessions/${sessionId}/coordinate`, {
          method: 'POST',
          body: JSON.stringify({ robotIds }),
        });
      },
    };
  }

  /**
   * Emotions API - Emotional intelligence
   */
  get emotions() {
    return {
      detect: async (data: {
        sessionId: string;
        timestamp: string;
        multimodalData: any;
      }): Promise<EmotionalState> => {
        const response = await this.request<EmotionalState>('/emotions/detect', {
          method: 'POST',
          body: JSON.stringify(data),
        });
        return response.data!;
      },

      generateResponse: async (
        emotion: EmotionalState,
        options: {
          strategy: 'encourage' | 'comfort' | 'challenge' | 'calm' | 'celebrate';
          intensity?: 'low' | 'moderate' | 'high';
        }
      ): Promise<any> => {
        const response = await this.request('/emotions/respond', {
          method: 'POST',
          body: JSON.stringify({ emotion, ...options }),
        });
        return response.data;
      },

      createDetector: (options: {
        modes: ('facial' | 'voice' | 'gesture')[];
        processingMode: 'local' | 'cloud';
        privacyLevel: 'strict' | 'standard';
      }): EmotionDetector => {
        return new EmotionDetector(this, options);
      },
    };
  }

  /**
   * Therapy API - Therapeutic sessions (requires professional credentials)
   */
  get therapy() {
    return {
      listProtocols: async (filters?: {
        focus?: string;
        evidenceLevel?: string;
      }): Promise<APIResponse<any[]>> => {
        const params = new URLSearchParams(filters as any);
        return this.request(`/therapeutic-protocols?${params}`);
      },

      createSession: async (options: {
        protocolId: string;
        childProfile: any;
        supervisionMode: string;
        dataRetention: string;
      }): Promise<TherapeuticSession> => {
        const response = await this.request<TherapeuticSession>('/therapeutic-sessions', {
          method: 'POST',
          body: JSON.stringify(options),
        });
        return response.data!;
      },

      trackProgress: async (sessionId: string, metrics: any): Promise<void> => {
        await this.request(`/therapeutic-sessions/${sessionId}/progress`, {
          method: 'POST',
          body: JSON.stringify(metrics),
        });
      },

      generateReport: async (
        sessionId: string,
        options: {
          recipients: string[];
          format: string;
        }
      ): Promise<any> => {
        const response = await this.request(`/therapeutic-sessions/${sessionId}/report`, {
          method: 'POST',
          body: JSON.stringify(options),
        });
        return response.data;
      },
    };
  }

  /**
   * Games API - Edutainment games
   */
  get games() {
    return {
      create: async (options: {
        subject: string;
        difficulty: string;
        format: string;
        learningObjectives: string[];
      }): Promise<EdutainmentGame> => {
        const response = await this.request<EdutainmentGame>('/games', {
          method: 'POST',
          body: JSON.stringify(options),
        });
        return response.data!;
      },

      start: async (gameId: string, userId: string): Promise<Session> => {
        const response = await this.request<Session>(`/games/${gameId}/start`, {
          method: 'POST',
          body: JSON.stringify({ userId }),
        });
        return response.data!;
      },

      submitAnswer: async (
        sessionId: string,
        challengeId: string,
        answer: any
      ): Promise<{ correct: boolean; feedback: string }> => {
        const response = await this.request(`/sessions/${sessionId}/answer`, {
          method: 'POST',
          body: JSON.stringify({ challengeId, answer }),
        });
        return response.data!;
      },
    };
  }

  /**
   * Privacy API - Consent and data management
   */
  get privacy() {
    return {
      requestConsent: async (request: ConsentRequest): Promise<any> => {
        const response = await this.request('/privacy/consent', {
          method: 'POST',
          body: JSON.stringify(request),
        });
        return response.data;
      },

      getSettings: async (userId: string): Promise<PrivacySettings> => {
        const response = await this.request<PrivacySettings>(`/privacy/settings/${userId}`);
        return response.data!;
      },

      updateSettings: async (userId: string, settings: Partial<PrivacySettings>): Promise<void> => {
        await this.request(`/privacy/settings/${userId}`, {
          method: 'PUT',
          body: JSON.stringify(settings),
        });
      },

      exportData: async (userId: string, format: 'JSON' | 'CSV'): Promise<any> => {
        const response = await this.request(`/privacy/export/${userId}?format=${format}`);
        return response.data;
      },

      deleteAllData: async (userId: string): Promise<void> => {
        await this.request(`/privacy/delete/${userId}`, {
          method: 'DELETE',
        });
      },
    };
  }

  /**
   * Robot API - Robot profile and management
   */
  get robot() {
    return {
      getProfile: async (robotId?: string): Promise<RobotProfile> => {
        const id = robotId || this.config.robotId;
        const response = await this.request<RobotProfile>(`/robots/${id}`);
        return response.data!;
      },

      updateProfile: async (robotId: string, updates: Partial<RobotProfile>): Promise<void> => {
        await this.request(`/robots/${robotId}`, {
          method: 'PUT',
          body: JSON.stringify(updates),
        });
      },

      speak: async (text: string, options?: {
        voice?: string;
        emotion?: string;
        speed?: number;
      }): Promise<void> => {
        await this.request('/robot/speak', {
          method: 'POST',
          body: JSON.stringify({ text, ...options }),
        });
      },

      celebrate: async (message: string, options?: { intensity?: 'low' | 'moderate' | 'high' }): Promise<void> => {
        await this.speak(message, { emotion: 'joyful', ...options });
      },

      encourage: async (message: string, options?: { supportive?: boolean }): Promise<void> => {
        await this.speak(message, { emotion: 'encouraging' });
      },
    };
  }

  /**
   * Internal request method
   */
  private async request<T = any>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<APIResponse<T>> {
    const url = `${this.baseURL}${endpoint}`;
    const headers = {
      'Authorization': `Bearer ${this.config.apiKey}`,
      'Content-Type': 'application/json',
      'X-WIA-Standard': 'EDU-025',
      'X-WIA-Version': '1.0.0',
      ...options.headers,
    };

    try {
      const response = await fetch(url, {
        ...options,
        headers,
        signal: AbortSignal.timeout(this.config.timeout),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new WIAError(error);
      }

      const data = await response.json();
      return { data };
    } catch (error) {
      if (error instanceof WIAError) {
        throw error;
      }
      throw new WIAError({
        code: 'NETWORK_ERROR',
        message: 'Failed to communicate with WIA API',
        details: String(error),
        timestamp: new Date().toISOString(),
      });
    }
  }
}

/**
 * Emotion Detector Class
 */
class EmotionDetector {
  private client: WIAEntertainmentRobot;
  private options: any;
  private listeners: Map<string, Function[]> = new Map();

  constructor(client: WIAEntertainmentRobot, options: any) {
    this.client = client;
    this.options = options;
  }

  on(event: 'emotionDetected', callback: (emotion: EmotionalState) => void): void {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, []);
    }
    this.listeners.get(event)!.push(callback);
  }

  async detect(multimodalData: any): Promise<EmotionalState> {
    const emotion = await this.client.emotions.detect({
      sessionId: 'current',
      timestamp: new Date().toISOString(),
      multimodalData,
    });

    // Emit event
    const callbacks = this.listeners.get('emotionDetected') || [];
    callbacks.forEach(cb => cb(emotion));

    return emotion;
  }
}

/**
 * Custom Error Class
 */
export class WIAError extends Error {
  code: string;
  details?: string;
  timestamp: string;
  requestId?: string;

  constructor(error: { code: string; message: string; details?: string; timestamp: string; requestId?: string }) {
    super(error.message);
    this.name = 'WIAError';
    this.code = error.code;
    this.details = error.details;
    this.timestamp = error.timestamp;
    this.requestId = error.requestId;
  }
}

/**
 * Utility Functions
 */
export const utils = {
  generateAnonymousUserId: (): string => {
    return `anonymous-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  },

  validateAgeRange: (ageRange: string): boolean => {
    return /^\d+-\d+$/.test(ageRange);
  },

  formatDuration: (seconds: number): string => {
    const minutes = Math.floor(seconds / 60);
    const remainingSeconds = seconds % 60;
    return `${minutes}:${remainingSeconds.toString().padStart(2, '0')}`;
  },
};

export default WIAEntertainmentRobot;
