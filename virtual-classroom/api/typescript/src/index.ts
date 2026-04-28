/**
 * WIA-EDU-006: Virtual Classroom Standard - TypeScript SDK
 *
 * © 2025 WIA - World Certification Industry Association
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import EventEmitter from 'eventemitter3';
import WebSocket from 'ws';

import * as Types from './types';

export * from './types';

/**
 * Main VirtualClassroom client
 */
export class VirtualClassroom extends EventEmitter {
  private api: AxiosInstance;
  private ws: WebSocket | null = null;
  private config: Types.ClassroomConfig;

  constructor(config: Types.ClassroomConfig) {
    super();
    this.config = config;

    const baseURL =
      config.baseUrl ||
      `https://api-${config.region || 'us-west-1'}.classroom.wia.org/api/v1`;

    this.api = axios.create({
      baseURL,
      timeout: config.timeout || 30000,
      headers: {
        Authorization: `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json'
      }
    });
  }

  // ============================================================================
  // Session Management
  // ============================================================================

  /**
   * Create a new virtual classroom session
   */
  async createSession(
    config: Types.CreateSessionConfig
  ): Promise<Types.Session> {
    try {
      const response = await this.api.post('/sessions', config);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Start a session
   */
  async startSession(sessionId: string): Promise<void> {
    try {
      await this.api.post(`/sessions/${sessionId}/start`);
      this.emit('session:started', { sessionId });
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * End a session
   */
  async endSession(sessionId: string): Promise<void> {
    try {
      await this.api.post(`/sessions/${sessionId}/end`);
      this.emit('session:ended', { sessionId });
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get session details
   */
  async getSession(sessionId: string): Promise<Types.Session> {
    try {
      const response = await this.api.get(`/sessions/${sessionId}`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Breakout Rooms
  // ============================================================================

  /**
   * Create breakout rooms
   */
  async createBreakoutRooms(
    config: Types.CreateBreakoutRoomsConfig
  ): Promise<Types.BreakoutRoom[]> {
    try {
      const response = await this.api.post(
        `/sessions/${config.sessionId}/breakout-rooms`,
        config
      );
      this.emit('breakout:created', { rooms: response.data.rooms });
      return response.data.rooms;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Close all breakout rooms
   */
  async closeBreakoutRooms(sessionId: string): Promise<void> {
    try {
      await this.api.delete(`/sessions/${sessionId}/breakout-rooms`);
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Broadcast message to all breakout rooms
   */
  async broadcastToRooms(config: Types.BroadcastMessage): Promise<void> {
    try {
      await this.api.post(
        `/sessions/${config.sessionId}/breakout-rooms/broadcast`,
        { message: config.message, type: config.type }
      );
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Whiteboard
  // ============================================================================

  /**
   * Create a whiteboard
   */
  async createWhiteboard(
    config: Types.WhiteboardConfig
  ): Promise<Types.Whiteboard> {
    try {
      const response = await this.api.post('/whiteboards', config);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get whiteboard
   */
  async getWhiteboard(whiteboardId: string): Promise<Types.Whiteboard> {
    try {
      const response = await this.api.get(`/whiteboards/${whiteboardId}`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Save whiteboard snapshot
   */
  async saveWhiteboard(whiteboardId: string): Promise<{ url: string }> {
    try {
      const response = await this.api.post(
        `/whiteboards/${whiteboardId}/snapshot`
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Add annotation to whiteboard
   */
  async addAnnotation(
    whiteboardId: string,
    annotation: Types.WhiteboardAnnotation
  ): Promise<void> {
    try {
      await this.api.post(`/whiteboards/${whiteboardId}/draw`, annotation);
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Attendance
  // ============================================================================

  /**
   * Enable attendance tracking
   */
  async enableAttendance(config: {
    sessionId: string;
    faceRecognition?: boolean;
    participationTracking?: boolean;
    attentionMonitoring?: boolean;
  }): Promise<void> {
    try {
      await this.api.post(`/sessions/${config.sessionId}/attendance/enable`, {
        faceRecognition: config.faceRecognition,
        participationTracking: config.participationTracking,
        attentionMonitoring: config.attentionMonitoring
      });
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get attendance data
   */
  async getAttendance(sessionId: string): Promise<Types.Attendance> {
    try {
      const response = await this.api.get(`/sessions/${sessionId}/attendance`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Export attendance report
   */
  async exportAttendance(
    config: Types.ExportAttendanceConfig
  ): Promise<Types.AttendanceReport> {
    try {
      const response = await this.api.post(
        `/sessions/${config.sessionId}/attendance/export`,
        {
          format: config.format,
          includeEngagement: config.includeEngagement
        }
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Quizzes & Polls
  // ============================================================================

  /**
   * Create a quiz
   */
  async createQuiz(config: Types.CreateQuizConfig): Promise<Types.Quiz> {
    try {
      const response = await this.api.post('/quizzes', config);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Launch a quiz
   */
  async launchQuiz(quizId: string): Promise<void> {
    try {
      await this.api.post(`/quizzes/${quizId}/launch`);
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get quiz results
   */
  async getQuizResults(quizId: string): Promise<Types.QuizResults> {
    try {
      const response = await this.api.get(`/quizzes/${quizId}/results`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Create a poll
   */
  async createPoll(config: Types.CreatePollConfig): Promise<Types.Poll> {
    try {
      const response = await this.api.post('/polls', config);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get poll results
   */
  async getPollResults(pollId: string): Promise<Types.Poll> {
    try {
      const response = await this.api.get(`/polls/${pollId}`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Recording
  // ============================================================================

  /**
   * Start recording
   */
  async startRecording(config: Types.StartRecordingConfig): Promise<void> {
    try {
      await this.api.post(`/sessions/${config.sessionId}/recording/start`, {
        settings: config.settings
      });
      this.emit('recording:started', { sessionId: config.sessionId });
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Stop recording
   */
  async stopRecording(sessionId: string): Promise<Types.Recording> {
    try {
      const response = await this.api.post(
        `/sessions/${sessionId}/recording/stop`
      );
      this.emit('recording:stopped', { sessionId });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get recording
   */
  async getRecording(recordingId: string): Promise<Types.Recording> {
    try {
      const response = await this.api.get(`/recordings/${recordingId}`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get transcription
   */
  async getTranscription(recordingId: string): Promise<Types.Transcription> {
    try {
      const response = await this.api.get(
        `/recordings/${recordingId}/transcription`
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Generate subtitles
   */
  async generateSubtitles(
    config: Types.GenerateSubtitlesConfig
  ): Promise<Types.Subtitles> {
    try {
      const response = await this.api.post(
        `/recordings/${config.recordingId}/subtitles`,
        {
          format: config.format,
          language: config.language
        }
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Analytics & Engagement
  // ============================================================================

  /**
   * Get engagement metrics
   */
  async getEngagementMetrics(
    sessionId: string
  ): Promise<Types.EngagementMetrics> {
    try {
      const response = await this.api.get(`/sessions/${sessionId}/engagement`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get session analytics
   */
  async getSessionAnalytics(
    sessionId: string
  ): Promise<Types.SessionAnalytics> {
    try {
      const response = await this.api.get(`/sessions/${sessionId}/analytics`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // WebSocket Connection
  // ============================================================================

  /**
   * Connect to WebSocket for real-time events
   */
  connectWebSocket(): void {
    const wsUrl = `wss://ws-${this.config.region || 'us-west-1'}.classroom.wia.org`;

    this.ws = new WebSocket(wsUrl, {
      headers: {
        Authorization: `Bearer ${this.config.apiKey}`
      }
    });

    this.ws.on('open', () => {
      this.emit('ws:connected');
    });

    this.ws.on('message', (data: WebSocket.Data) => {
      try {
        const event: Types.SessionEvent = JSON.parse(data.toString());
        this.emit(`ws:${event.type}`, event.data);
      } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
      }
    });

    this.ws.on('error', (error) => {
      this.emit('ws:error', error);
    });

    this.ws.on('close', () => {
      this.emit('ws:disconnected');
    });
  }

  /**
   * Disconnect WebSocket
   */
  disconnectWebSocket(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  /**
   * Subscribe to a channel
   */
  subscribe(channel: string, params?: any): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket not connected');
    }

    this.ws.send(
      JSON.stringify({
        action: 'subscribe',
        channel,
        params
      })
    );
  }

  /**
   * Unsubscribe from a channel
   */
  unsubscribe(channel: string): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket not connected');
    }

    this.ws.send(
      JSON.stringify({
        action: 'unsubscribe',
        channel
      })
    );
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Handle API errors
   */
  private handleError(error: any): Types.VirtualClassroomError {
    if (axios.isAxiosError(error) && error.response) {
      const apiError = error.response.data as Types.APIError;
      return new Types.VirtualClassroomError(
        apiError.message,
        apiError.code,
        apiError.details
      );
    }
    return new Types.VirtualClassroomError(
      error.message || 'Unknown error',
      500
    );
  }
}

/**
 * Utility functions
 */
export class Utilities {
  /**
   * Format duration in minutes to human-readable string
   */
  static formatDuration(minutes: number): string {
    if (minutes < 60) {
      return `${minutes} minutes`;
    }
    const hours = Math.floor(minutes / 60);
    const mins = minutes % 60;
    return `${hours}h ${mins}m`;
  }

  /**
   * Validate email address
   */
  static isValidEmail(email: string): boolean {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  }

  /**
   * Generate random session password
   */
  static generatePassword(length: number = 8): string {
    const chars =
      'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
    let password = '';
    for (let i = 0; i < length; i++) {
      password += chars.charAt(Math.floor(Math.random() * chars.length));
    }
    return password;
  }
}

/**
 * Default export
 */
export default VirtualClassroom;
