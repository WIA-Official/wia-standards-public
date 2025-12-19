/**
 * WIA Emotion AI Client
 *
 * REST API client for emotion analysis services.
 */

import { AnalysisResult, DimensionalModel, Emotion, ActionUnit } from './models';
import { EmotionStream, StreamConfig } from './stream';

export interface EmotionClientConfig {
    apiKey: string;
    baseUrl?: string;
    timeout?: number;
}

export interface FacialAnalysisOptions {
    detectActionUnits?: boolean;
    detectLandmarks?: boolean;
    detectHeadPose?: boolean;
    culturalContext?: string;
    confidenceThreshold?: number;
}

export interface VoiceAnalysisOptions {
    language?: string;
    extractProsody?: boolean;
    extractVoiceQuality?: boolean;
}

export interface TextAnalysisOptions {
    language?: string;
    detectSarcasm?: boolean;
    analyzeEmoji?: boolean;
}

export interface MultimodalOptions {
    fusionStrategy?: 'early_fusion' | 'late_fusion' | 'hybrid';
    modalityWeights?: {
        facial?: number;
        voice?: number;
        text?: number;
        biosignal?: number;
    };
}

/**
 * WIA Emotion AI API Client
 *
 * @example
 * ```typescript
 * const client = new EmotionClient({ apiKey: 'your-api-key' });
 * const result = await client.analyzeFacialImage('photo.jpg');
 * console.log(result.emotions[0].category);
 * ```
 */
export class EmotionClient {
    private apiKey: string;
    private baseUrl: string;
    private timeout: number;

    constructor(config: EmotionClientConfig) {
        this.apiKey = config.apiKey;
        this.baseUrl = (config.baseUrl || 'https://api.wia.live/wia/emotion/v1').replace(/\/$/, '');
        this.timeout = config.timeout || 30000;
    }

    /**
     * Analyze facial expression in an image.
     */
    async analyzeFacialImage(
        imagePath: string,
        options: FacialAnalysisOptions = {}
    ): Promise<AnalysisResult> {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }

    /**
     * Analyze facial expressions in a video (async job).
     */
    async analyzeFacialVideo(
        videoPath: string,
        options: { frameRate?: number; detectMicroExpressions?: boolean; callbackUrl?: string } = {}
    ): Promise<{ jobId: string; statusUrl: string }> {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }

    /**
     * Analyze voice emotion in audio.
     */
    async analyzeVoice(
        audioPath: string,
        options: VoiceAnalysisOptions = {}
    ): Promise<AnalysisResult> {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }

    /**
     * Analyze text sentiment and emotion.
     */
    async analyzeText(
        text: string,
        options: TextAnalysisOptions = {}
    ): Promise<AnalysisResult> {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }

    /**
     * Analyze biosignal data for emotion.
     */
    async analyzeBiosignal(
        biosignals: {
            heartRate?: { values: number[]; sampleRate: number };
            eda?: { values: number[]; sampleRate: number };
        },
        options: { computeHrv?: boolean; detectStress?: boolean } = {}
    ): Promise<AnalysisResult> {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }

    /**
     * Analyze multiple modalities with fusion.
     */
    async analyzeMultimodal(
        modalities: {
            facial?: { image: string };
            voice?: { audio: string; format: string };
            text?: { content: string };
        },
        options: MultimodalOptions = {}
    ): Promise<AnalysisResult> {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }

    /**
     * Create a real-time emotion streaming session.
     */
    createStream(config: Partial<StreamConfig> = {}): EmotionStream {
        const wsUrl = this.baseUrl
            .replace('https://', 'wss://')
            .replace('/v1', '/v1/stream');

        return new EmotionStream({
            apiKey: this.apiKey,
            url: wsUrl,
            modalities: config.modalities || ['facial'],
            frameRate: config.frameRate || 15
        });
    }

    /**
     * Get status of an async job.
     */
    async getJobStatus(jobId: string): Promise<{
        jobId: string;
        status: 'processing' | 'completed' | 'failed';
        resultUrl?: string;
    }> {
        throw new Error('Install full SDK: npm install @wia/emotion-ai');
    }
}
