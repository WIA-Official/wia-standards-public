/**
 * WIA-AI-026 Generative AI TypeScript SDK
 *
 * A comprehensive SDK for interacting with the WIA Generative AI platform.
 *
 * @package @wia/generative-ai
 * @version 1.0.0
 * @license MIT
 *
 * @example
 * ```typescript
 * import { GenerativeAI } from '@wia/generative-ai';
 *
 * const ai = new GenerativeAI({ apiKey: 'your-api-key' });
 *
 * // Text generation
 * const text = await ai.text.generate({
 *   prompt: 'Write a poem about AI',
 *   temperature: 0.7
 * });
 *
 * // Image generation
 * const images = await ai.image.generate({
 *   prompt: 'A beautiful sunset over mountains',
 *   width: 1024,
 *   height: 1024
 * });
 * ```
 */

import * as types from './types';

export * from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

export class GenerativeAI {
  private config: types.APIConfig;
  private baseUrl: string;

  public text: TextGeneration;
  public image: ImageGeneration;
  public audio: AudioGeneration;
  public video: VideoGeneration;
  public multimodal: MultimodalGeneration;
  public finetune: FineTuning;
  public safety: SafetyModeration;

  constructor(config: types.APIConfig) {
    this.config = {
      baseUrl: 'https://api.wia.org/v1/generative',
      timeout: 60000,
      retries: 3,
      ...config
    };

    this.baseUrl = this.config.baseUrl!;

    // Initialize modules
    this.text = new TextGeneration(this);
    this.image = new ImageGeneration(this);
    this.audio = new AudioGeneration(this);
    this.video = new VideoGeneration(this);
    this.multimodal = new MultimodalGeneration(this);
    this.finetune = new FineTuning(this);
    this.safety = new SafetyModeration(this);
  }

  /**
   * Make an HTTP request to the API
   */
  async request<T>(
    method: string,
    path: string,
    data?: any,
    options?: { stream?: boolean }
  ): Promise<T> {
    const url = `${this.baseUrl}${path}`;

    const headers: Record<string, string> = {
      'Authorization': `Bearer ${this.config.apiKey}`,
      'Content-Type': 'application/json',
      'User-Agent': '@wia/generative-ai/1.0.0',
      ...this.config.headers
    };

    const fetchOptions: RequestInit = {
      method,
      headers,
      body: data ? JSON.stringify(data) : undefined,
    };

    try {
      const response = await fetch(url, fetchOptions);

      if (!response.ok) {
        const error: types.ErrorResponse = await response.json();
        throw new types.GenerativeAIError(
          error.error.message,
          error.error.code,
          response.status,
          error.error.details
        );
      }

      if (options?.stream) {
        return response as any;
      }

      return await response.json();
    } catch (error) {
      if (error instanceof types.GenerativeAIError) {
        throw error;
      }
      throw new types.GenerativeAIError(
        (error as Error).message || 'Unknown error',
        'NETWORK_ERROR'
      );
    }
  }
}

// ============================================================================
// Text Generation Module
// ============================================================================

export class TextGeneration {
  constructor(private client: GenerativeAI) {}

  /**
   * Generate text from a prompt
   */
  async generate(
    request: types.TextGenerationRequest
  ): Promise<types.TextGenerationResponse> {
    if (request.stream) {
      throw new types.GenerativeAIError(
        'Use generateStream() for streaming responses',
        'INVALID_REQUEST'
      );
    }

    return this.client.request<types.TextGenerationResponse>(
      'POST',
      '/text/generate',
      request
    );
  }

  /**
   * Generate text with streaming
   */
  async *generateStream(
    request: types.TextGenerationRequest
  ): AsyncGenerator<types.StreamChunk> {
    const response = await this.client.request<Response>(
      'POST',
      '/text/generate',
      { ...request, stream: true },
      { stream: true }
    );

    const reader = response.body?.getReader();
    if (!reader) {
      throw new types.GenerativeAIError('No response body', 'STREAM_ERROR');
    }

    const decoder = new TextDecoder();
    let buffer = '';

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n');
      buffer = lines.pop() || '';

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          const data = line.slice(6);
          if (data === '[DONE]') return;

          try {
            const chunk: types.StreamChunk = JSON.parse(data);
            yield chunk;
          } catch (e) {
            // Skip invalid JSON
          }
        }
      }
    }
  }

  /**
   * Chat completion (conversational)
   */
  async chat(messages: types.Message[]): Promise<types.TextGenerationResponse> {
    return this.generate({
      prompt: '', // Handled by server from messages
      conversationHistory: messages
    });
  }
}

// ============================================================================
// Image Generation Module
// ============================================================================

export class ImageGeneration {
  constructor(private client: GenerativeAI) {}

  /**
   * Generate images from text prompt
   */
  async generate(
    request: types.ImageGenerationRequest
  ): Promise<types.ImageGenerationResponse> {
    return this.client.request<types.ImageGenerationResponse>(
      'POST',
      '/image/generate',
      request
    );
  }

  /**
   * Edit an existing image
   */
  async edit(
    request: types.ImageEditRequest
  ): Promise<types.ImageGenerationResponse> {
    return this.client.request<types.ImageGenerationResponse>(
      'POST',
      '/image/edit',
      request
    );
  }

  /**
   * Inpaint parts of an image
   */
  async inpaint(
    image: string,
    mask: string,
    prompt: string
  ): Promise<types.ImageGenerationResponse> {
    return this.edit({ image, mask, prompt });
  }

  /**
   * Upscale an image
   */
  async upscale(
    image: string,
    scale: number = 2
  ): Promise<types.ImageGenerationResponse> {
    return this.client.request<types.ImageGenerationResponse>(
      'POST',
      '/image/upscale',
      { image, scale }
    );
  }

  /**
   * Image-to-image transformation
   */
  async imageToImage(
    image: string,
    prompt: string,
    strength: number = 0.8
  ): Promise<types.ImageGenerationResponse> {
    return this.edit({ image, prompt, strength });
  }
}

// ============================================================================
// Audio Generation Module
// ============================================================================

export class AudioGeneration {
  constructor(private client: GenerativeAI) {}

  /**
   * Generate audio from text or prompt
   */
  async generate(
    request: types.AudioGenerationRequest
  ): Promise<types.AudioGenerationResponse> {
    return this.client.request<types.AudioGenerationResponse>(
      'POST',
      '/audio/generate',
      request
    );
  }

  /**
   * Text-to-speech
   */
  async textToSpeech(
    text: string,
    voice?: string,
    language?: string
  ): Promise<types.AudioGenerationResponse> {
    return this.generate({
      text,
      voice,
      language,
      type: 'speech'
    });
  }

  /**
   * Generate music from prompt
   */
  async generateMusic(
    prompt: string,
    duration?: number
  ): Promise<types.AudioGenerationResponse> {
    return this.generate({
      prompt,
      duration,
      type: 'music'
    });
  }

  /**
   * Generate sound effect
   */
  async generateSoundEffect(
    prompt: string
  ): Promise<types.AudioGenerationResponse> {
    return this.generate({
      prompt,
      type: 'sound_effect'
    });
  }
}

// ============================================================================
// Video Generation Module (Phase 2+)
// ============================================================================

export class VideoGeneration {
  constructor(private client: GenerativeAI) {}

  /**
   * Generate video from text prompt
   */
  async generate(
    request: types.VideoGenerationRequest
  ): Promise<types.VideoGenerationResponse> {
    return this.client.request<types.VideoGenerationResponse>(
      'POST',
      '/video/generate',
      request
    );
  }

  /**
   * Check generation status
   */
  async status(id: string): Promise<types.VideoGenerationResponse> {
    return this.client.request<types.VideoGenerationResponse>(
      'GET',
      `/video/${id}`
    );
  }
}

// ============================================================================
// Multimodal Generation Module (Phase 2+)
// ============================================================================

export class MultimodalGeneration {
  constructor(private client: GenerativeAI) {}

  /**
   * Generate output from multimodal inputs
   */
  async generate(
    request: types.MultimodalRequest
  ): Promise<types.MultimodalResponse> {
    return this.client.request<types.MultimodalResponse>(
      'POST',
      '/multimodal/generate',
      request
    );
  }
}

// ============================================================================
// Fine-Tuning Module (Phase 2+)
// ============================================================================

export class FineTuning {
  constructor(private client: GenerativeAI) {}

  /**
   * Create a fine-tuning job
   */
  async create(request: types.FineTuneRequest): Promise<types.FineTuneJob> {
    return this.client.request<types.FineTuneJob>(
      'POST',
      '/finetune',
      request
    );
  }

  /**
   * Get fine-tuning job status
   */
  async get(id: string): Promise<types.FineTuneJob> {
    return this.client.request<types.FineTuneJob>('GET', `/finetune/${id}`);
  }

  /**
   * List fine-tuning jobs
   */
  async list(): Promise<types.FineTuneJob[]> {
    return this.client.request<types.FineTuneJob[]>('GET', '/finetune');
  }

  /**
   * Cancel a fine-tuning job
   */
  async cancel(id: string): Promise<void> {
    await this.client.request('POST', `/finetune/${id}/cancel`);
  }
}

// ============================================================================
// Safety and Moderation Module
// ============================================================================

export class SafetyModeration {
  constructor(private client: GenerativeAI) {}

  /**
   * Check content safety
   */
  async check(content: string): Promise<types.SafetyResult> {
    return this.client.request<types.SafetyResult>(
      'POST',
      '/safety/check',
      { content }
    );
  }

  /**
   * Moderate multiple contents
   */
  async moderateBatch(contents: string[]): Promise<types.SafetyResult[]> {
    return this.client.request<types.SafetyResult[]>(
      'POST',
      '/safety/batch',
      { contents }
    );
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Encode image to base64
 */
export async function encodeImageToBase64(
  file: File | Blob
): Promise<string> {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(reader.result as string);
    reader.onerror = reject;
    reader.readAsDataURL(file);
  });
}

/**
 * Download audio from URL
 */
export async function downloadAudio(url: string): Promise<Blob> {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Failed to download audio: ${response.statusText}`);
  }
  return response.blob();
}

/**
 * Estimate token count
 */
export function estimateTokens(text: string): number {
  // Rough estimation: ~4 characters per token
  return Math.ceil(text.length / 4);
}

/**
 * Calculate generation cost
 */
export function estimateCost(
  type: types.ModelType,
  units: number
): number {
  const pricing = {
    text: 0.001, // per 1000 tokens
    image: 0.02, // per image
    audio: 0.01, // per minute
    video: 0.10, // per second
    multimodal: 0.05 // per request
  };

  return units * pricing[type];
}

// ============================================================================
// Default Export
// ============================================================================

export default GenerativeAI;

/**
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK embodies the principle of Hongik Ingan—using technology
 * to benefit all people. May it empower creativity, amplify human potential,
 * and serve the common good.
 */
