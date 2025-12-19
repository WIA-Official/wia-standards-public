/**
 * WIA Emotion AI WebSocket Streaming
 *
 * Real-time emotion streaming client.
 */

import { EmotionEvent } from './models';

export interface StreamConfig {
    apiKey: string;
    url: string;
    modalities: string[];
    frameRate: number;
    detectActionUnits?: boolean;
    detectMicroExpressions?: boolean;
    confidenceThreshold?: number;
}

export type StreamEventType =
    | 'emotion'
    | 'aggregate'
    | 'error'
    | 'connected'
    | 'disconnected';

export interface EmotionStreamEvent {
    type: 'emotion.event';
    eventId: string;
    timestamp: string;
    sequence: number;
    latencyMs: number;
    emotions: Array<{
        category: string;
        intensity: number;
        confidence: number;
    }>;
    actionUnits?: Array<{
        au: string;
        intensity: number;
        confidence: number;
    }>;
    dimensional?: {
        valence: number;
        arousal: number;
    };
}

export interface AggregateEvent {
    type: 'emotion.aggregate';
    timestamp: string;
    windowStart: string;
    windowEnd: string;
    frameCount: number;
    aggregate: {
        dominantEmotion: string;
        emotionDistribution: Record<string, number>;
        averageValence: number;
        averageArousal: number;
        emotionalStability: number;
        engagementScore: number;
    };
}

type EventHandler<T> = (event: T) => void;

/**
 * Real-time emotion streaming client using WebSocket.
 *
 * @example
 * ```typescript
 * const stream = client.createStream({ modalities: ['facial', 'voice'] });
 *
 * stream.on('emotion', (event) => {
 *     console.log(`Emotion: ${event.emotions[0].category}`);
 * });
 *
 * stream.on('aggregate', (summary) => {
 *     console.log(`Dominant: ${summary.aggregate.dominantEmotion}`);
 * });
 *
 * await stream.start();
 *
 * // Send frames
 * stream.sendFrame(videoFrame);
 * stream.sendAudio(audioChunk);
 *
 * // Stop
 * await stream.stop();
 * ```
 */
export class EmotionStream {
    private config: StreamConfig;
    private handlers: Map<StreamEventType, EventHandler<unknown>[]> = new Map();
    private sessionId: string | null = null;
    private connected = false;
    private sequence = 0;

    constructor(config: StreamConfig) {
        this.config = {
            detectActionUnits: true,
            detectMicroExpressions: false,
            confidenceThreshold: 0.5,
            ...config
        };

        // Initialize handler arrays
        const eventTypes: StreamEventType[] = [
            'emotion', 'aggregate', 'error', 'connected', 'disconnected'
        ];
        eventTypes.forEach(type => this.handlers.set(type, []));
    }

    /**
     * Register event handler.
     */
    on(eventType: 'emotion', handler: EventHandler<EmotionStreamEvent>): this;
    on(eventType: 'aggregate', handler: EventHandler<AggregateEvent>): this;
    on(eventType: 'error', handler: EventHandler<Error>): this;
    on(eventType: 'connected' | 'disconnected', handler: EventHandler<void>): this;
    on(eventType: StreamEventType, handler: EventHandler<unknown>): this {
        const handlers = this.handlers.get(eventType);
        if (handlers) {
            handlers.push(handler);
        }
        return this;
    }

    /**
     * Start the streaming session.
     */
    async start(): Promise<void> {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }

    /**
     * Stop the streaming session.
     */
    async stop(requestSummary = true): Promise<Record<string, unknown> | undefined> {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }

    /**
     * Send a video frame for analysis.
     */
    sendFrame(frame: Buffer | Uint8Array, format: 'jpeg' | 'png' = 'jpeg'): void {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }

    /**
     * Send an audio chunk for analysis.
     */
    sendAudio(audio: Buffer | Uint8Array, encoding = 'LINEAR16'): void {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }

    /**
     * Pause the streaming session.
     */
    pause(): void {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }

    /**
     * Resume the streaming session.
     */
    resume(): void {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }

    /**
     * Check if stream is connected.
     */
    get isConnected(): boolean {
        return this.connected;
    }

    /**
     * Get current session ID.
     */
    getSessionId(): string | null {
        return this.sessionId;
    }
}
