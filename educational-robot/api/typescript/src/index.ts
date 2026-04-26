/**
 * WIA-EDU-007: Educational Robot Standard
 * Official TypeScript SDK v2.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 * © 2025 WIA - World Certification Industry Association
 * MIT License
 */

import axios, { AxiosInstance } from 'axios';
import WebSocket from 'ws';
import * as Types from './types';

export * from './types';

const DEFAULT_BASE_URL = 'https://api.wiastandards.com/edu-robot/v1';
const DEFAULT_STREAM_URL = 'wss://stream.wiastandards.com/edu-robot/v1';

/**
 * Main SDK class for WIA Educational Robot Standard
 */
export class WIAEduRobot {
  private client: AxiosInstance;
  private config: Types.WIAEduRobotConfig;
  private ws?: WebSocket;

  constructor(config: Types.WIAEduRobotConfig) {
    this.config = {
      baseURL: DEFAULT_BASE_URL,
      timeout: 30000,
      retryAttempts: 3,
      ...config
    };

    this.client = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/ld+json',
        'WIA-API-Version': '2.0.0'
      }
    });
  }

  // ========================================================================
  // Robot Profile Methods
  // ========================================================================

  /**
   * Get robot profile by ID
   */
  async getRobotProfile(robotId: string): Promise<Types.RobotProfile> {
    const response = await this.client.get<Types.RobotProfile>(`/robots/${robotId}`);
    return response.data;
  }

  /**
   * Update robot configuration
   */
  async updateRobotProfile(robotId: string, updates: Partial<Types.RobotProfile>): Promise<void> {
    await this.client.put(`/robots/${robotId}`, updates);
  }

  // ========================================================================
  // Learning Session Methods
  // ========================================================================

  /**
   * Create a new learning session
   */
  async createSession(session: Omit<Types.LearningSession, '@context' | '@type' | 'sessionId' | 'timestamp'>): Promise<{ sessionId: string; sessionUrl: string }> {
    const response = await this.client.post<{ sessionId: string; sessionUrl: string }>('/sessions', session);
    return response.data;
  }

  /**
   * Get session details by ID
   */
  async getSession(sessionId: string): Promise<Types.LearningSession> {
    const response = await this.client.get<Types.LearningSession>(`/sessions/${sessionId}`);
    return response.data;
  }

  /**
   * Update session (real-time engagement tracking)
   */
  async updateSession(sessionId: string, updates: Partial<Types.LearningSession>): Promise<void> {
    await this.client.patch(`/sessions/${sessionId}`, updates);
  }

  /**
   * End a learning session
   */
  async endSession(sessionId: string): Promise<void> {
    await this.client.delete(`/sessions/${sessionId}`);
  }

  // ========================================================================
  // Student Progress Methods
  // ========================================================================

  /**
   * Get student progress
   */
  async getStudentProgress(
    studentId: string,
    options?: {
      subjectArea?: Types.SubjectArea;
      startDate?: string;
      endDate?: string;
    }
  ): Promise<Types.StudentProgress> {
    const params = new URLSearchParams(options as any);
    const response = await this.client.get<Types.StudentProgress>(
      `/students/${studentId}/progress?${params}`
    );
    return response.data;
  }

  /**
   * Award an achievement to a student
   */
  async awardAchievement(
    studentId: string,
    achievement: {
      achievementId: string;
      type: Types.AchievementType;
      level: Types.AchievementLevel;
      verifiableCredential?: boolean;
    }
  ): Promise<{ achievementId: string; credentialUrl?: string }> {
    const response = await this.client.post<{ achievementId: string; credentialUrl?: string }>(
      `/students/${studentId}/achievements`,
      achievement
    );
    return response.data;
  }

  // ========================================================================
  // Curriculum Methods
  // ========================================================================

  /**
   * List available lesson plans
   */
  async getCurriculum(subjectArea: Types.SubjectArea, grade?: string): Promise<any[]> {
    const params = new URLSearchParams(grade ? { grade } : {});
    const response = await this.client.get(`/curriculum/${subjectArea}?${params}`);
    return response.data.lessons;
  }

  /**
   * Get detailed lesson plan
   */
  async getLessonPlan(subjectArea: Types.SubjectArea, lessonId: string): Promise<any> {
    const response = await this.client.get(`/curriculum/${subjectArea}/${lessonId}`);
    return response.data;
  }

  // ========================================================================
  // Real-Time Interaction Methods
  // ========================================================================

  /**
   * Send student input to robot
   */
  async sendInteraction(interaction: {
    sessionId: string;
    studentInput: {
      type: string;
      content: string;
      timestamp: string;
    };
  }): Promise<{
    interactionId: string;
    robotResponse: any;
    timestamp: string;
  }> {
    const response = await this.client.post('/interactions', interaction);
    return response.data;
  }

  // ========================================================================
  // WebSocket Methods
  // ========================================================================

  /**
   * Connect to real-time WebSocket stream
   */
  connectWebSocket(
    sessionId: string,
    config?: Partial<Types.WebSocketConfig>,
    handlers?: {
      onMessage?: (message: Types.WebSocketMessage) => void;
      onError?: (error: Error) => void;
      onClose?: () => void;
    }
  ): WebSocket {
    const wsConfig: Types.WebSocketConfig = {
      url: DEFAULT_STREAM_URL,
      reconnect: true,
      maxReconnectAttempts: 5,
      reconnectInterval: 3000,
      ...config
    };

    const headers = {
      'Authorization': `Bearer ${this.config.apiKey}`,
      'WIA-Robot-ID': this.config.robotId || '',
      'WIA-Session-ID': sessionId
    };

    this.ws = new WebSocket(wsConfig.url, { headers });

    this.ws.on('message', (data: WebSocket.Data) => {
      try {
        const message = JSON.parse(data.toString()) as Types.WebSocketMessage;
        handlers?.onMessage?.(message);
      } catch (error) {
        handlers?.onError?.(error as Error);
      }
    });

    this.ws.on('error', (error) => {
      handlers?.onError?.(error);
    });

    this.ws.on('close', () => {
      handlers?.onClose?.();

      if (wsConfig.reconnect && wsConfig.maxReconnectAttempts! > 0) {
        setTimeout(() => {
          this.connectWebSocket(sessionId, {
            ...wsConfig,
            maxReconnectAttempts: wsConfig.maxReconnectAttempts! - 1
          }, handlers);
        }, wsConfig.reconnectInterval);
      }
    });

    return this.ws;
  }

  /**
   * Send message through WebSocket
   */
  sendMessage(message: Partial<Types.WebSocketMessage>): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket is not connected');
    }

    const fullMessage: Types.WebSocketMessage = {
      '@context': 'https://wiastandards.com/contexts/edu-robot/message/v1',
      '@type': 'RobotMessage',
      messageId: `msg-${Date.now()}`,
      timestamp: new Date().toISOString(),
      sessionId: '',
      messageType: 'instruction',
      payload: {},
      ...message
    };

    this.ws.send(JSON.stringify(fullMessage));
  }

  /**
   * Close WebSocket connection
   */
  disconnectWebSocket(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = undefined;
    }
  }

  // ========================================================================
  // Verifiable Credentials Methods
  // ========================================================================

  /**
   * Issue a verifiable credential for student achievement
   */
  async issueCredential(
    studentId: string,
    achievement: {
      achievementType: Types.AchievementType;
      subject: Types.SubjectArea;
      skill: string;
      level: Types.AchievementLevel;
      score: number;
    }
  ): Promise<Types.VerifiableCredential> {
    const response = await this.client.post<Types.VerifiableCredential>(
      `/credentials/issue`,
      { studentId, achievement }
    );
    return response.data;
  }

  /**
   * Verify a credential
   */
  async verifyCredential(credential: Types.VerifiableCredential): Promise<{
    verified: boolean;
    issuer: string;
    status: string;
    timestamp: string;
  }> {
    const response = await this.client.post('/credentials/verify', { credential });
    return response.data;
  }
}

/**
 * Create a new WIA Educational Robot SDK instance
 */
export function createClient(config: Types.WIAEduRobotConfig): WIAEduRobot {
  return new WIAEduRobot(config);
}

/**
 * Default export
 */
export default WIAEduRobot;
