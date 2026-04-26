/**
 * WIA-ROB-013 Companion Robot SDK
 * @version 1.0.0
 * @license MIT
 */

import * as Types from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface SDKConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
}

// ============================================================================
// Companion Robot SDK
// ============================================================================

export class CompanionRobotSDK {
  private config: SDKConfig;
  private baseURL: string;

  constructor(config: SDKConfig) {
    this.config = config;
    this.baseURL = config.baseURL || 'https://api.companion.wia/v1';
  }

  // ==========================================================================
  // Session Management
  // ==========================================================================

  async createSession(
    companionId: string,
    preferences?: Types.SessionPreferences
  ): Promise<Types.Session> {
    return this.request('POST', '/sessions', {
      companionId,
      preferences
    });
  }

  async getSession(sessionId: string): Promise<Types.Session> {
    return this.request('GET', `/sessions/${sessionId}`);
  }

  async endSession(sessionId: string): Promise<void> {
    await this.request('DELETE', `/sessions/${sessionId}`);
  }

  // ==========================================================================
  // Messaging
  // ==========================================================================

  async sendMessage(
    sessionId: string,
    text: string,
    options?: {
      language?: string;
      context?: Partial<Types.MessageContext>;
    }
  ): Promise<Types.MessageResponse> {
    return this.request('POST', `/sessions/${sessionId}/messages`, {
      text,
      language: options?.language || 'en',
      context: options?.context
    });
  }

  async getConversationHistory(
    sessionId: string,
    options?: {
      limit?: number;
      offset?: number;
    }
  ): Promise<{ messages: Types.InteractionMessage[]; pagination: any }> {
    const params = new URLSearchParams();
    if (options?.limit) params.append('limit', options.limit.toString());
    if (options?.offset) params.append('offset', options.offset.toString());

    return this.request('GET', `/sessions/${sessionId}/messages?${params}`);
  }

  // ==========================================================================
  // Emotion Analysis
  // ==========================================================================

  async analyzeEmotion(
    text: string,
    options?: {
      voice?: string;
      language?: string;
    }
  ): Promise<Types.EmotionState> {
    return this.request('POST', '/emotions/analyze', {
      text,
      voice: options?.voice,
      language: options?.language || 'en'
    });
  }

  // ==========================================================================
  // Personality Management
  // ==========================================================================

  async getPersonality(companionId: string): Promise<{
    personalityTraits: Types.PersonalityTraits;
    communicationStyle: string;
    lastUpdated: string;
  }> {
    return this.request('GET', `/companions/${companionId}/personality`);
  }

  async updatePersonality(
    companionId: string,
    traits: Partial<Types.PersonalityTraits>
  ): Promise<{
    updated: boolean;
    personalityTraits: Types.PersonalityTraits;
  }> {
    return this.request('PATCH', `/companions/${companionId}/personality`, {
      personalityTraits: traits
    });
  }

  // ==========================================================================
  // Memory Management
  // ==========================================================================

  async storeMemory(
    type: Types.MemoryType,
    content: string,
    importance: number
  ): Promise<{ memoryId: string; stored: boolean }> {
    return this.request('POST', '/memories', {
      type,
      content,
      importance
    });
  }

  async queryMemories(
    options?: {
      type?: Types.MemoryType;
      limit?: number;
    }
  ): Promise<{ memories: Types.Memory[] }> {
    const params = new URLSearchParams();
    if (options?.type) params.append('type', options.type);
    if (options?.limit) params.append('limit', options.limit.toString());

    return this.request('GET', `/memories?${params}`);
  }

  // ==========================================================================
  // Helper Methods
  // ==========================================================================

  private async request(
    method: string,
    endpoint: string,
    body?: any
  ): Promise<any> {
    const url = `${this.baseURL}${endpoint}`;
    const headers: Record<string, string> = {
      'Authorization': `Bearer ${this.config.apiKey}`,
      'Content-Type': 'application/json'
    };

    const options: RequestInit = {
      method,
      headers,
      ...(body && { body: JSON.stringify(body) })
    };

    const response = await fetch(url, options);

    if (!response.ok) {
      const error: Types.ErrorResponse = await response.json();
      throw new Error(`API Error: ${error.error.message}`);
    }

    return response.json();
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

export function validatePersonalityTraits(traits: Types.PersonalityTraits): boolean {
  const keys: (keyof Types.PersonalityTraits)[] = [
    'openness', 'conscientiousness', 'extraversion', 'agreeableness',
    'neuroticism', 'playfulness', 'formality', 'proactivity'
  ];

  for (const key of keys) {
    const value = traits[key];
    if (typeof value !== 'number' || value < 0 || value > 100) {
      return false;
    }
  }

  return true;
}

export function validateEmotionalDimensions(dimensions: Types.EmotionalDimensions): boolean {
  return (
    dimensions.valence >= -1.0 && dimensions.valence <= 1.0 &&
    dimensions.arousal >= 0.0 && dimensions.arousal <= 1.0 &&
    dimensions.dominance >= -1.0 && dimensions.dominance <= 1.0
  );
}

export function getPrimaryEmotionFromDimensions(
  dimensions: Types.EmotionalDimensions
): Types.PrimaryEmotion {
  const { valence, arousal } = dimensions;

  if (valence > 0.5 && arousal > 0.5) return Types.PrimaryEmotion.JOY;
  if (valence > 0.5 && arousal < 0.3) return Types.PrimaryEmotion.SURPRISE;
  if (valence < -0.5 && arousal > 0.5) return Types.PrimaryEmotion.ANGER;
  if (valence < -0.5 && arousal < 0.3) return Types.PrimaryEmotion.SADNESS;
  if (valence < 0 && arousal > 0.6) return Types.PrimaryEmotion.FEAR;

  return Types.PrimaryEmotion.SURPRISE;
}
