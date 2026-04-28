/**
 * WIA-AI-022 Speech Recognition TypeScript SDK
 * Main Implementation
 *
 * @packageDocumentation
 */

import {
  ASREngine,
  ASRConfig,
  TranscriptionOptions,
  TranscriptionResult,
  StreamOptions,
  TranscriptionStream,
  StreamResult,
  LanguageCode,
  Entity,
  Intent,
  Sentiment,
  ASRError,
  ASRErrorCode,
} from './types';

export * from './types';

/**
 * WIA Speech Recognition Engine
 */
export class WIASpeech implements ASREngine {
  private config: Required<ASRConfig>;

  /**
   * Create a new WIA Speech instance
   *
   * @param config - Configuration options
   *
   * @example
   * ```typescript
   * const asr = new WIASpeech({
   *   apiKey: 'your-api-key',
   *   language: 'en-US'
   * });
   *
   * const result = await asr.transcribeFile('audio.wav');
   * console.log(result.text);
   * ```
   */
  constructor(config: ASRConfig = {}) {
    this.config = {
      endpoint: config.endpoint || 'https://api.wia.ai/speech/v1',
      apiKey: config.apiKey || '',
      language: config.language || 'en-US',
      audio: {
        sampleRate: 16000,
        bitDepth: 16,
        channels: 1,
        format: 'wav',
        ...config.audio,
      },
      vad: {
        algorithm: 'webrtc',
        sensitivity: 0.5,
        frameLengthMs: 30,
        ...config.vad,
      },
      logging: config.logging ?? false,
      headers: config.headers || {},
    };
  }

  /**
   * Transcribe audio file
   *
   * @param filePath - Path to audio file
   * @param options - Transcription options
   * @returns Transcription result
   *
   * @example
   * ```typescript
   * const result = await asr.transcribeFile('meeting.wav', {
   *   diarization: true,
   *   numSpeakers: 3,
   *   punctuation: true
   * });
   *
   * console.log(`Text: ${result.text}`);
   * console.log(`Speakers: ${result.diarization?.speakers.length}`);
   * ```
   */
  async transcribeFile(
    filePath: string,
    options?: TranscriptionOptions
  ): Promise<TranscriptionResult> {
    // In Node.js environment
    if (typeof window === 'undefined') {
      const fs = await import('fs');
      const buffer = fs.readFileSync(filePath);
      return this.transcribe(buffer.buffer, options);
    }

    // In browser environment
    throw new ASRError(
      ASRErrorCode.INVALID_AUDIO_FORMAT,
      'File transcription not supported in browser. Use transcribe() with ArrayBuffer.'
    );
  }

  /**
   * Transcribe audio buffer
   *
   * @param audioBuffer - Audio data as ArrayBuffer
   * @param options - Transcription options
   * @returns Transcription result
   *
   * @example
   * ```typescript
   * const audioBuffer = await fetch('audio.wav')
   *   .then(r => r.arrayBuffer());
   *
   * const result = await asr.transcribe(audioBuffer, {
   *   language: 'ko-KR',
   *   alternatives: 3
   * });
   *
   * console.log(result.text);
   * result.alternatives?.forEach((alt, i) => {
   *   console.log(`Alternative ${i+1}: ${alt.text} (${alt.confidence})`);
   * });
   * ```
   */
  async transcribe(
    audioBuffer: ArrayBuffer,
    options?: TranscriptionOptions
  ): Promise<TranscriptionResult> {
    const mergedOptions = {
      language: this.config.language,
      ...options,
    };

    try {
      const response = await fetch(`${this.config.endpoint}/transcribe`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/octet-stream',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-Language': mergedOptions.language,
          ...this.config.headers,
        },
        body: audioBuffer,
      });

      if (!response.ok) {
        throw await this.handleError(response);
      }

      const result: TranscriptionResult = await response.json();

      if (this.config.logging) {
        console.log('[WIA-Speech] Transcription complete:', {
          text: result.text,
          confidence: result.confidence,
          duration: result.duration,
        });
      }

      return result;
    } catch (error) {
      if (error instanceof ASRError) {
        throw error;
      }

      throw new ASRError(
        ASRErrorCode.NETWORK_ERROR,
        `Transcription failed: ${error}`,
        error
      );
    }
  }

  /**
   * Create streaming transcription session
   *
   * @param options - Stream options
   * @returns Transcription stream
   *
   * @example
   * ```typescript
   * const stream = asr.createStream({
   *   interimResults: true,
   *   language: 'en-US'
   * });
   *
   * // Handle interim results
   * stream.on('data', (result) => {
   *   if (!result.isFinal) {
   *     console.log(`Interim: ${result.text}`);
   *   }
   * });
   *
   * // Handle final result
   * stream.on('final', (result) => {
   *   console.log(`Final: ${result.text}`);
   * });
   *
   * // Send audio chunks
   * for (const chunk of audioChunks) {
   *   await stream.write(chunk);
   * }
   *
   * const finalResult = await stream.end();
   * ```
   */
  createStream(options?: StreamOptions): TranscriptionStream {
    return new WIATranscriptionStream(this.config, options);
  }

  /**
   * Detect language in audio
   *
   * @param audioBuffer - Audio data
   * @returns Detected language and confidence
   *
   * @example
   * ```typescript
   * const result = await asr.detectLanguage(audioBuffer);
   * console.log(`Language: ${result.language} (${result.confidence})`);
   * ```
   */
  async detectLanguage(audioBuffer: ArrayBuffer): Promise<{
    language: LanguageCode;
    confidence: number;
  }> {
    const response = await fetch(`${this.config.endpoint}/detect-language`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/octet-stream',
        'Authorization': `Bearer ${this.config.apiKey}`,
        ...this.config.headers,
      },
      body: audioBuffer,
    });

    if (!response.ok) {
      throw await this.handleError(response);
    }

    return response.json();
  }

  /**
   * Extract named entities from text
   *
   * @param text - Input text
   * @returns Array of entities
   *
   * @example
   * ```typescript
   * const entities = await asr.extractEntities(
   *   "Call John Smith at 555-1234 tomorrow"
   * );
   *
   * entities.forEach(e => {
   *   console.log(`${e.type}: ${e.text}`);
   * });
   * ```
   */
  async extractEntities(text: string): Promise<Entity[]> {
    const response = await fetch(`${this.config.endpoint}/entities`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.config.apiKey}`,
        ...this.config.headers,
      },
      body: JSON.stringify({ text }),
    });

    if (!response.ok) {
      throw await this.handleError(response);
    }

    const result = await response.json();
    return result.entities;
  }

  /**
   * Recognize intent from text
   *
   * @param text - Input text
   * @returns Intent classification
   *
   * @example
   * ```typescript
   * const intent = await asr.recognizeIntent("Set alarm for 7 AM");
   * console.log(`Intent: ${intent.name}`);
   * console.log(`Time: ${intent.slots.time}`);
   * ```
   */
  async recognizeIntent(text: string): Promise<Intent> {
    const response = await fetch(`${this.config.endpoint}/intent`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.config.apiKey}`,
        ...this.config.headers,
      },
      body: JSON.stringify({ text }),
    });

    if (!response.ok) {
      throw await this.handleError(response);
    }

    const result = await response.json();
    return result.intent;
  }

  /**
   * Analyze sentiment of text
   *
   * @param text - Input text
   * @returns Sentiment analysis result
   *
   * @example
   * ```typescript
   * const sentiment = await asr.analyzeSentiment("I love this!");
   * console.log(`Sentiment: ${sentiment.label} (${sentiment.score})`);
   * ```
   */
  async analyzeSentiment(text: string): Promise<Sentiment> {
    const response = await fetch(`${this.config.endpoint}/sentiment`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.config.apiKey}`,
        ...this.config.headers,
      },
      body: JSON.stringify({ text }),
    });

    if (!response.ok) {
      throw await this.handleError(response);
    }

    const result = await response.json();
    return result.sentiment;
  }

  /**
   * Handle API errors
   */
  private async handleError(response: Response): Promise<ASRError> {
    let errorCode: ASRErrorCode;
    let message: string;

    switch (response.status) {
      case 400:
        errorCode = ASRErrorCode.INVALID_AUDIO_FORMAT;
        message = 'Invalid audio format or parameters';
        break;
      case 401:
        errorCode = ASRErrorCode.AUTH_FAILED;
        message = 'Authentication failed';
        break;
      case 429:
        errorCode = ASRErrorCode.RATE_LIMIT_EXCEEDED;
        message = 'Rate limit exceeded';
        break;
      case 500:
      case 502:
      case 503:
        errorCode = ASRErrorCode.INTERNAL_ERROR;
        message = 'Internal server error';
        break;
      default:
        errorCode = ASRErrorCode.NETWORK_ERROR;
        message = `HTTP ${response.status}: ${response.statusText}`;
    }

    const details = await response.json().catch(() => null);

    return new ASRError(errorCode, message, details);
  }
}

/**
 * Transcription stream implementation
 */
class WIATranscriptionStream implements TranscriptionStream {
  private ws: WebSocket | null = null;
  private eventHandlers: Map<string, Function[]> = new Map();
  private config: Required<ASRConfig>;
  private options: StreamOptions;

  constructor(config: Required<ASRConfig>, options?: StreamOptions) {
    this.config = config;
    this.options = options || {};
    this.connect();
  }

  private connect(): void {
    const wsUrl = this.config.endpoint.replace(/^http/, 'ws') + '/stream';

    this.ws = new WebSocket(wsUrl, [
      'wia-speech-v1',
      `Bearer ${this.config.apiKey}`,
    ]);

    this.ws.onopen = () => {
      this.ws?.send(JSON.stringify({
        type: 'config',
        config: this.options,
      }));
    };

    this.ws.onmessage = (event) => {
      const data = JSON.parse(event.data);

      if (data.type === 'interim' || data.type === 'final') {
        const result: StreamResult = data.result;
        this.emit('data', result);

        if (result.isFinal) {
          this.emit('final', result);
        }
      }
    };

    this.ws.onerror = (error) => {
      this.emit('error', new Error(`WebSocket error: ${error}`));
    };

    this.ws.onclose = () => {
      this.emit('close');
    };
  }

  async write(audioChunk: ArrayBuffer): Promise<void> {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('Stream not ready');
    }

    this.ws.send(audioChunk);
  }

  async end(): Promise<TranscriptionResult> {
    return new Promise((resolve, reject) => {
      if (!this.ws) {
        reject(new Error('Stream not initialized'));
        return;
      }

      this.once('final', (result: TranscriptionResult) => {
        this.close();
        resolve(result);
      });

      this.once('error', (error: Error) => {
        reject(error);
      });

      this.ws.send(JSON.stringify({ type: 'end' }));
    });
  }

  close(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  on(event: string, callback: Function): void {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, []);
    }
    this.eventHandlers.get(event)!.push(callback);
  }

  private once(event: string, callback: Function): void {
    const wrappedCallback = (...args: any[]) => {
      callback(...args);
      const handlers = this.eventHandlers.get(event);
      if (handlers) {
        const index = handlers.indexOf(wrappedCallback);
        if (index > -1) {
          handlers.splice(index, 1);
        }
      }
    };
    this.on(event, wrappedCallback);
  }

  private emit(event: string, ...args: any[]): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      handlers.forEach(handler => handler(...args));
    }
  }
}

/**
 * Default export
 */
export default WIASpeech;

/**
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * This SDK enables speech recognition technology to serve all people
 * across languages, cultures, and abilities.
 */
