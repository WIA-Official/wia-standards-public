/**
 * WIA-PET-010 Pet Translator SDK
 * Version 2.0.0
 *
 * Official TypeScript SDK for the WIA-PET-010 Pet Translator Standard
 *
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import WebSocket from 'ws';
import EventEmitter from 'eventemitter3';

import {
  PetTranslatorConfig,
  SubscriptionOptions,
  TranslationEvent,
  TranslationResult,
  EmotionTimelineQuery,
  EmotionTimelineResponse,
  HistoryQuery,
  HistoryResponse,
  WebhookConfig,
  TranslationEventData,
  EmotionChangeEventData,
  ConnectionEventData,
  ErrorEventData,
  Species,
  Language,
} from './types';

// Export all types
export * from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

export class PetTranslator extends EventEmitter {
  private config: Required<PetTranslatorConfig>;
  private httpClient: AxiosInstance;
  private ws: WebSocket | null = null;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;
  private reconnectDelay = 1000; // ms

  constructor(config: PetTranslatorConfig) {
    super();

    // Set defaults
    this.config = {
      apiKey: config.apiKey,
      baseUrl: config.baseUrl || 'https://api.wia-pet.org/v2',
      websocketUrl: config.websocketUrl || 'wss://api.wia-pet.org/v2/stream',
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
      languages: config.languages || [Language.English, Language.Korean],
      localProcessing: config.localProcessing || false,
    };

    // Create HTTP client
    this.httpClient = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'User-Agent': 'WIA-PET-Translator-SDK/2.0.0',
      },
    });

    // Setup axios interceptors for retries
    this.setupHttpRetry();
  }

  // ==========================================================================
  // HTTP API Methods
  // ==========================================================================

  /**
   * Translate a single pet communication event
   */
  async translate(event: TranslationEvent): Promise<TranslationResult> {
    try {
      const response = await this.httpClient.post<TranslationResult>(
        '/translate',
        event
      );
      return response.data;
    } catch (error) {
      this.handleError(error);
      throw error;
    }
  }

  /**
   * Translate multiple events in batch
   */
  async translateBatch(events: TranslationEvent[]): Promise<TranslationResult[]> {
    try {
      const response = await this.httpClient.post<TranslationResult[]>(
        '/translate/batch',
        { events }
      );
      return response.data;
    } catch (error) {
      this.handleError(error);
      throw error;
    }
  }

  /**
   * Get emotion timeline for a pet
   */
  async getEmotionTimeline(
    query: EmotionTimelineQuery
  ): Promise<EmotionTimelineResponse> {
    try {
      const response = await this.httpClient.get<EmotionTimelineResponse>(
        `/pets/${query.petId}/emotions`,
        {
          params: {
            startDate: this.formatDate(query.startDate),
            endDate: this.formatDate(query.endDate),
            granularity: query.granularity || 'hour',
          },
        }
      );
      return response.data;
    } catch (error) {
      this.handleError(error);
      throw error;
    }
  }

  /**
   * Get translation history for a pet
   */
  async getHistory(query: HistoryQuery): Promise<HistoryResponse> {
    try {
      const response = await this.httpClient.get<HistoryResponse>(
        `/pets/${query.petId}/history`,
        {
          params: {
            startDate: query.startDate ? this.formatDate(query.startDate) : undefined,
            endDate: query.endDate ? this.formatDate(query.endDate) : undefined,
            limit: query.limit || 100,
            offset: query.offset || 0,
            emotions: query.emotions?.join(','),
            minConfidence: query.minConfidence,
          },
        }
      );
      return response.data;
    } catch (error) {
      this.handleError(error);
      throw error;
    }
  }

  /**
   * Update pet profile
   */
  async updatePetProfile(
    petId: string,
    profile: Partial<TranslationEvent['petProfile']>
  ): Promise<void> {
    try {
      await this.httpClient.post(`/pets/${petId}/profile`, profile);
    } catch (error) {
      this.handleError(error);
      throw error;
    }
  }

  /**
   * Configure webhook notifications
   */
  async configureWebhook(config: WebhookConfig): Promise<void> {
    try {
      await this.httpClient.post('/webhooks', config);
    } catch (error) {
      this.handleError(error);
      throw error;
    }
  }

  // ==========================================================================
  // WebSocket Streaming Methods
  // ==========================================================================

  /**
   * Subscribe to real-time pet communication stream
   */
  async subscribe(options: SubscriptionOptions): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        // Create WebSocket connection
        this.ws = new WebSocket(this.config.websocketUrl, {
          headers: {
            'Authorization': `Bearer ${this.config.apiKey}`,
          },
        });

        this.ws.on('open', () => {
          // Send subscription message
          this.ws!.send(JSON.stringify({
            type: 'subscribe',
            petId: options.petId,
            species: options.species,
            breed: options.breed,
            streamConfig: options.streamConfig || {
              audio: true,
              video: false,
              sensors: false,
            },
            languages: options.languages || this.config.languages,
          }));

          this.emit('connection', {
            status: 'connected',
          } as ConnectionEventData);

          this.reconnectAttempts = 0;
          resolve();
        });

        this.ws.on('message', (data: Buffer) => {
          try {
            const message = JSON.parse(data.toString());
            this.handleWebSocketMessage(message);
          } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
          }
        });

        this.ws.on('error', (error) => {
          this.emit('error', {
            code: 'WEBSOCKET_ERROR',
            message: error.message,
          } as ErrorEventData);
          reject(error);
        });

        this.ws.on('close', () => {
          this.emit('connection', {
            status: 'disconnected',
          } as ConnectionEventData);

          // Attempt reconnection
          this.attemptReconnect(options);
        });

      } catch (error) {
        reject(error);
      }
    });
  }

  /**
   * Unsubscribe from real-time stream
   */
  unsubscribe(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  /**
   * Stream audio data for real-time translation
   */
  streamAudio(audioData: Buffer | string): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket not connected. Call subscribe() first.');
    }

    this.ws.send(JSON.stringify({
      type: 'stream_chunk',
      audio: audioData,
    }));
  }

  /**
   * Stream video data for real-time translation
   */
  streamVideo(videoData: Buffer | string): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket not connected. Call subscribe() first.');
    }

    this.ws.send(JSON.stringify({
      type: 'stream_chunk',
      video: videoData,
    }));
  }

  /**
   * Process audio stream from MediaStream (browser)
   */
  async processAudioStream(stream: MediaStream): Promise<void> {
    if (typeof window === 'undefined') {
      throw new Error('processAudioStream is only available in browser environments');
    }

    const audioContext = new (window.AudioContext || (window as any).webkitAudioContext)();
    const source = audioContext.createMediaStreamSource(stream);
    const processor = audioContext.createScriptProcessor(4096, 1, 1);

    source.connect(processor);
    processor.connect(audioContext.destination);

    processor.onaudioprocess = (e) => {
      const inputData = e.inputBuffer.getChannelData(0);
      const buffer = Buffer.from(new Float32Array(inputData).buffer);
      this.streamAudio(buffer.toString('base64'));
    };
  }

  // ==========================================================================
  // Private Helper Methods
  // ==========================================================================

  private setupHttpRetry(): void {
    this.httpClient.interceptors.response.use(
      (response) => response,
      async (error) => {
        const config = error.config;

        if (!config || !config.retryCount) {
          config.retryCount = 0;
        }

        if (config.retryCount < this.config.retries) {
          config.retryCount += 1;

          // Exponential backoff
          const delay = Math.pow(2, config.retryCount) * 1000;
          await new Promise((resolve) => setTimeout(resolve, delay));

          return this.httpClient(config);
        }

        return Promise.reject(error);
      }
    );
  }

  private handleWebSocketMessage(message: any): void {
    switch (message.type) {
      case 'translation':
        this.emit('translation', message as TranslationEventData);
        break;

      case 'emotion_update':
      case 'emotion_change':
        this.emit('emotionChange', message as EmotionChangeEventData);
        break;

      case 'ack':
        // Acknowledgment received, no action needed
        break;

      case 'error':
        this.emit('error', message as ErrorEventData);
        break;

      default:
        console.warn('Unknown WebSocket message type:', message.type);
    }
  }

  private attemptReconnect(options: SubscriptionOptions): void {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      this.emit('error', {
        code: 'MAX_RECONNECT_ATTEMPTS',
        message: 'Maximum reconnection attempts reached',
      } as ErrorEventData);
      return;
    }

    this.reconnectAttempts++;
    const delay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts - 1);

    setTimeout(() => {
      console.log(`Attempting to reconnect (${this.reconnectAttempts}/${this.maxReconnectAttempts})...`);
      this.subscribe(options).catch((error) => {
        console.error('Reconnection failed:', error);
      });
    }, delay);
  }

  private formatDate(date: Date | string): string {
    if (typeof date === 'string') {
      return date;
    }
    return date.toISOString();
  }

  private handleError(error: any): void {
    let errorData: ErrorEventData;

    if (error.response) {
      // HTTP error response
      errorData = {
        code: error.response.status.toString(),
        message: error.response.data?.message || error.message,
        details: error.response.data,
      };
    } else if (error.request) {
      // Network error
      errorData = {
        code: 'NETWORK_ERROR',
        message: 'Network request failed',
        details: { error: error.message },
      };
    } else {
      // Other error
      errorData = {
        code: 'UNKNOWN_ERROR',
        message: error.message,
      };
    }

    this.emit('error', errorData);
  }

  /**
   * Close all connections and cleanup resources
   */
  destroy(): void {
    this.unsubscribe();
    this.removeAllListeners();
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * Create a PetTranslator instance with configuration
 */
export function createPetTranslator(config: PetTranslatorConfig): PetTranslator {
  return new PetTranslator(config);
}

/**
 * Quick translate function for one-off translations
 */
export async function translate(
  apiKey: string,
  event: TranslationEvent
): Promise<TranslationResult> {
  const translator = new PetTranslator({ apiKey });
  try {
    return await translator.translate(event);
  } finally {
    translator.destroy();
  }
}

// ============================================================================
// Default Export
// ============================================================================

export default PetTranslator;
