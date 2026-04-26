/**
 * WIA-EDU-008 Digital Textbook Standard - TypeScript SDK
 * Version: 2.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * @example
 * ```typescript
 * import { DigitalTextbookClient } from '@wia/digital-textbook-sdk';
 *
 * const client = new DigitalTextbookClient({
 *   baseURL: 'https://api.textbooks.com/wia/v1',
 *   accessToken: 'your-access-token'
 * });
 *
 * const textbook = await client.getTextbook('978-3-16-148410-0');
 * ```
 */

import axios, { AxiosInstance } from 'axios';
import WebSocket from 'ws';
import { v4 as uuidv4 } from 'uuid';

// Export all types
export * from './types';

import {
  APIConfig,
  APIError,
  Annotation,
  ReadingProgress,
  ReadingAnalytics,
  TextbookMetadata,
  SyncMessage,
  SyncMessageType,
  XAPIStatement
} from './types';

/**
 * Main Digital Textbook API Client
 */
export class DigitalTextbookClient {
  private api: AxiosInstance;
  private config: APIConfig;
  private ws?: WebSocket;
  private vectorClock: { [deviceId: string]: number } = {};
  private deviceId: string;

  /**
   * Create a new Digital Textbook client
   * @param config - API configuration
   */
  constructor(config: APIConfig) {
    this.config = config;
    this.deviceId = uuidv4();

    this.api = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'WIA-EDU-008',
        'X-WIA-Version': '2.0',
        ...(config.accessToken && {
          Authorization: `Bearer ${config.accessToken}`
        })
      }
    });

    if (config.enableSync && config.syncURL) {
      this.initializeSync();
    }
  }

  /**
   * Get textbook metadata
   * @param textbookId - ISBN or textbook ID
   * @returns Textbook metadata
   */
  async getTextbook(textbookId: string): Promise<TextbookMetadata> {
    const response = await this.api.get(`/textbooks/${textbookId}`);
    return response.data;
  }

  /**
   * List available textbooks
   * @param filters - Optional filters
   * @returns Array of textbooks
   */
  async listTextbooks(filters?: {
    subject?: string;
    level?: string;
    language?: string;
  }): Promise<TextbookMetadata[]> {
    const response = await this.api.get('/textbooks', { params: filters });
    return response.data.results;
  }

  /**
   * Download textbook EPUB
   * @param textbookId - ISBN or textbook ID
   * @returns Download URL
   */
  async downloadTextbook(textbookId: string): Promise<string> {
    const response = await this.api.get(`/textbooks/${textbookId}/download`);
    return response.data.downloadUrl;
  }

  /**
   * Get table of contents
   * @param textbookId - ISBN or textbook ID
   * @returns Table of contents
   */
  async getTableOfContents(textbookId: string): Promise<any> {
    const response = await this.api.get(`/textbooks/${textbookId}/toc`);
    return response.data;
  }

  /**
   * Create annotation
   * @param annotation - Annotation data
   * @returns Created annotation
   */
  async createAnnotation(annotation: Omit<Annotation, 'id' | 'created' | 'modified' | 'syncStatus'>): Promise<Annotation> {
    const response = await this.api.post('/annotations', annotation);

    // If sync is enabled, send via WebSocket
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.sendSyncMessage('sync.annotation.create', response.data);
    }

    return response.data;
  }

  /**
   * Get user annotations
   * @param textbookId - Optional textbook filter
   * @param chapterId - Optional chapter filter
   * @returns Array of annotations
   */
  async getAnnotations(textbookId?: string, chapterId?: string): Promise<Annotation[]> {
    const params: any = {};
    if (textbookId) params.textbookId = textbookId;
    if (chapterId) params.chapterId = chapterId;

    const response = await this.api.get('/annotations', { params });
    return response.data.annotations;
  }

  /**
   * Update annotation
   * @param annotationId - Annotation ID
   * @param updates - Fields to update
   * @returns Updated annotation
   */
  async updateAnnotation(annotationId: string, updates: Partial<Annotation>): Promise<Annotation> {
    const response = await this.api.put(`/annotations/${annotationId}`, updates);

    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.sendSyncMessage('sync.annotation.update', response.data);
    }

    return response.data;
  }

  /**
   * Delete annotation
   * @param annotationId - Annotation ID
   */
  async deleteAnnotation(annotationId: string): Promise<void> {
    await this.api.delete(`/annotations/${annotationId}`);

    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.sendSyncMessage('sync.annotation.delete', { id: annotationId });
    }
  }

  /**
   * Get reading progress
   * @param textbookId - ISBN or textbook ID
   * @returns Reading progress
   */
  async getProgress(textbookId: string): Promise<ReadingProgress> {
    const response = await this.api.get(`/progress/${textbookId}`);
    return response.data;
  }

  /**
   * Update reading progress
   * @param textbookId - ISBN or textbook ID
   * @param progress - Progress data
   * @returns Updated progress
   */
  async updateProgress(textbookId: string, progress: Partial<ReadingProgress>): Promise<ReadingProgress> {
    const response = await this.api.post(`/progress/${textbookId}`, progress);

    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.sendSyncMessage('sync.progress.update', response.data);
    }

    return response.data;
  }

  /**
   * Get reading analytics
   * @param textbookId - ISBN or textbook ID
   * @param period - Time period (e.g., 'week', 'month')
   * @returns Reading analytics
   */
  async getAnalytics(textbookId: string, period: string = 'week'): Promise<ReadingAnalytics> {
    const response = await this.api.get('/analytics/reading', {
      params: { textbookId, period }
    });
    return response.data;
  }

  /**
   * Submit xAPI statement
   * @param statement - xAPI statement
   */
  async submitXAPIStatement(statement: XAPIStatement): Promise<void> {
    await this.api.post('/xapi/statements', statement, {
      headers: {
        'X-Experience-API-Version': '1.0.3'
      }
    });
  }

  /**
   * Initialize WebSocket synchronization
   */
  private initializeSync(): void {
    if (!this.config.syncURL) return;

    this.ws = new WebSocket(this.config.syncURL);

    this.ws.on('open', () => {
      console.log('Sync connection established');

      // Authenticate
      this.ws!.send(JSON.stringify({
        type: 'auth',
        token: this.config.accessToken
      }));
    });

    this.ws.on('message', (data: WebSocket.Data) => {
      const message: SyncMessage = JSON.parse(data.toString());
      this.handleSyncMessage(message);
    });

    this.ws.on('error', (error) => {
      console.error('Sync error:', error);
    });

    this.ws.on('close', () => {
      console.log('Sync connection closed');
      // Implement reconnection logic here
    });
  }

  /**
   * Send sync message via WebSocket
   * @param type - Message type
   * @param data - Message data
   */
  private sendSyncMessage(type: SyncMessageType, data: any): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    // Increment vector clock for this device
    this.vectorClock[this.deviceId] = (this.vectorClock[this.deviceId] || 0) + 1;

    const message: SyncMessage = {
      type,
      messageId: uuidv4(),
      timestamp: new Date().toISOString(),
      deviceId: this.deviceId,
      vectorClock: { ...this.vectorClock },
      data
    };

    this.ws.send(JSON.stringify(message));
  }

  /**
   * Handle incoming sync message
   * @param message - Sync message
   */
  private handleSyncMessage(message: SyncMessage): void {
    // Update vector clock
    Object.keys(message.vectorClock).forEach(deviceId => {
      this.vectorClock[deviceId] = Math.max(
        this.vectorClock[deviceId] || 0,
        message.vectorClock[deviceId]
      );
    });

    // Handle different message types
    switch (message.type) {
      case 'sync.annotation.create':
        // Trigger event or callback for new annotation
        console.log('Remote annotation created:', message.data);
        break;
      case 'sync.annotation.update':
        console.log('Remote annotation updated:', message.data);
        break;
      case 'sync.annotation.delete':
        console.log('Remote annotation deleted:', message.data);
        break;
      case 'sync.progress.update':
        console.log('Remote progress updated:', message.data);
        break;
      case 'sync.ack':
        console.log('Message acknowledged:', message.messageId);
        break;
    }
  }

  /**
   * Close sync connection
   */
  closeSync(): void {
    if (this.ws) {
      this.ws.close();
    }
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Generate WIA-compliant annotation ID
   */
  generateAnnotationId(): string {
    return `anno-${uuidv4()}`;
  },

  /**
   * Validate ISBN-13
   */
  validateISBN(isbn: string): boolean {
    const cleaned = isbn.replace(/[-\s]/g, '');
    if (cleaned.length !== 13) return false;

    const checksum = cleaned
      .split('')
      .map(Number)
      .reduce((sum, digit, i) => sum + digit * (i % 2 === 0 ? 1 : 3), 0);

    return checksum % 10 === 0;
  },

  /**
   * Format duration string
   */
  formatDuration(seconds: number): string {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = seconds % 60;

    return `PT${hours}H${minutes}M${secs}S`;
  },

  /**
   * Parse duration string
   */
  parseDuration(duration: string): number {
    const match = duration.match(/PT(?:(\d+)H)?(?:(\d+)M)?(?:(\d+)S)?/);
    if (!match) return 0;

    const hours = parseInt(match[1] || '0');
    const minutes = parseInt(match[2] || '0');
    const seconds = parseInt(match[3] || '0');

    return hours * 3600 + minutes * 60 + seconds;
  }
};

// Default export
export default DigitalTextbookClient;
