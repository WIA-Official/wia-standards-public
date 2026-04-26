/**
 * WIA-EDU-014 Game-Based Learning Standard - TypeScript SDK
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @standard WIA-EDU-014
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

export interface GameBasedLearningClientConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

/**
 * WIA Game-Based Learning API Client
 *
 * Provides programmatic access to WIA-EDU-014 compliant game platforms
 */
export class GameBasedLearningClient {
  private client: AxiosInstance;

  constructor(config: GameBasedLearningClientConfig) {
    this.client = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        ...(config.apiKey && { 'Authorization': `ApiKey ${config.apiKey}` })
      }
    });
  }

  // ============================================================================
  // Authentication
  // ============================================================================

  /**
   * Authenticate with OAuth 2.0
   */
  async authenticate(credentials: {
    grant_type: string;
    client_id: string;
    client_secret: string;
    code?: string;
  }): Promise<Types.AuthToken> {
    const response = await this.client.post<Types.AuthToken>('/v1/auth/token', credentials);

    // Set token for future requests
    if (response.data.access_token) {
      this.client.defaults.headers.common['Authorization'] = `Bearer ${response.data.access_token}`;
    }

    return response.data;
  }

  // ============================================================================
  // Game Catalog APIs
  // ============================================================================

  /**
   * List available games with filters
   */
  async listGames(params?: {
    subject?: string;
    grade?: number;
    difficulty?: string;
    language?: string;
    limit?: number;
    offset?: number;
  }): Promise<Types.APIResponse<Types.GameMetadata[]>> {
    const response = await this.client.get<Types.APIResponse<Types.GameMetadata[]>>('/v1/games', { params });
    return response.data;
  }

  /**
   * Get detailed game information
   */
  async getGame(gameId: string): Promise<Types.GameMetadata> {
    const response = await this.client.get<Types.GameMetadata>(`/v1/games/${gameId}`);
    return response.data;
  }

  /**
   * Search games
   */
  async searchGames(query: string, params?: {
    language?: string;
    limit?: number;
  }): Promise<Types.APIResponse<Types.GameMetadata[]>> {
    const response = await this.client.get<Types.APIResponse<Types.GameMetadata[]>>('/v1/games/search', {
      params: { q: query, ...params }
    });
    return response.data;
  }

  // ============================================================================
  // Player Management APIs
  // ============================================================================

  /**
   * Create a new player profile
   */
  async createPlayer(profile: Partial<Types.PlayerProfile>): Promise<Types.PlayerProfile> {
    const response = await this.client.post<Types.PlayerProfile>('/v1/players', profile);
    return response.data;
  }

  /**
   * Get player profile
   */
  async getPlayer(playerId: string): Promise<Types.PlayerProfile> {
    const response = await this.client.get<Types.PlayerProfile>(`/v1/players/${playerId}`);
    return response.data;
  }

  /**
   * Update player preferences
   */
  async updatePlayerPreferences(playerId: string, preferences: Partial<Types.PlayerPreferences>): Promise<void> {
    await this.client.patch(`/v1/players/${playerId}/preferences`, preferences);
  }

  // ============================================================================
  // Progress APIs
  // ============================================================================

  /**
   * Save game progress
   */
  async saveProgress(progress: Types.ProgressData): Promise<void> {
    await this.client.post('/v1/progress', progress);
  }

  /**
   * Get player progress for a specific game
   */
  async getProgress(playerId: string, gameId: string): Promise<Types.ProgressData> {
    const response = await this.client.get<Types.ProgressData>(`/v1/progress/${playerId}/${gameId}`);
    return response.data;
  }

  /**
   * List all progress for a player
   */
  async listPlayerProgress(playerId: string): Promise<Types.ProgressData[]> {
    const response = await this.client.get<Types.ProgressData[]>(`/v1/progress/${playerId}`);
    return response.data;
  }

  // ============================================================================
  // Achievement APIs
  // ============================================================================

  /**
   * Award an achievement to a player
   */
  async awardAchievement(award: Omit<Types.AchievementAward, 'awardId'>): Promise<Types.AchievementAward> {
    const response = await this.client.post<Types.AchievementAward>('/v1/achievements', award);
    return response.data;
  }

  /**
   * List player achievements
   */
  async listAchievements(playerId: string): Promise<Types.AchievementAward[]> {
    const response = await this.client.get<Types.AchievementAward[]>(`/v1/achievements/${playerId}`);
    return response.data;
  }

  /**
   * Verify achievement credential
   */
  async verifyAchievement(achievementId: string, credentialId: string): Promise<boolean> {
    const response = await this.client.get<{ verified: boolean }>(
      `/v1/achievements/${achievementId}/verify`,
      { params: { credential: credentialId } }
    );
    return response.data.verified;
  }

  // ============================================================================
  // Analytics APIs
  // ============================================================================

  /**
   * Submit learning analytics events
   */
  async submitAnalyticsEvents(events: any[]): Promise<void> {
    await this.client.post('/v1/analytics/events', { events });
  }
}

/**
 * Create a new Game-Based Learning client instance
 *
 * @example
 * ```typescript
 * const client = createClient({
 *   baseURL: 'https://api.gameplatform.com',
 *   apiKey: 'your-api-key'
 * });
 *
 * const games = await client.listGames({ subject: 'mathematics', grade: 7 });
 * ```
 */
export function createClient(config: GameBasedLearningClientConfig): GameBasedLearningClient {
  return new GameBasedLearningClient(config);
}

/**
 * Validate game metadata against WIA-EDU-014 schema
 */
export function validateGameMetadata(metadata: any): { valid: boolean; errors: string[] } {
  const errors: string[] = [];

  if (!metadata.id || !metadata.id.startsWith('game_')) {
    errors.push('Invalid game ID format. Must start with "game_"');
  }

  if (metadata.standard !== 'WIA-EDU-014') {
    errors.push('Invalid standard. Must be "WIA-EDU-014"');
  }

  if (!metadata.accessibility?.wcagLevel) {
    errors.push('Accessibility WCAG level is required');
  }

  return {
    valid: errors.length === 0,
    errors
  };
}

/**
 * Calculate mastery level from performance data
 */
export function calculateMastery(performance: Types.PerformanceData): number {
  const accuracyWeight = 0.6;
  const completionWeight = 0.4;

  const accuracyScore = performance.accuracy;
  const completionScore = (performance.problemsSolved / 100) * 100; // Normalized

  return Math.min(100, accuracyWeight * accuracyScore + completionWeight * completionScore);
}

/**
 * Generate verifiable credential for achievement
 */
export function generateAchievementCredential(
  award: Types.AchievementAward,
  issuer: { id: string; name: string }
): Partial<Types.VerifiableCredential> {
  return {
    '@context': [
      'https://www.w3.org/2018/credentials/v1',
      'https://w3id.org/openbadges/v3'
    ],
    type: ['VerifiableCredential', 'OpenBadgeCredential'],
    issuer,
    issuanceDate: award.earnedDate,
    credentialSubject: {
      id: `did:wia:student:${award.playerId}`,
      achievement: {
        type: award.achievementId,
        name: award.achievementId,
        description: 'Achievement earned through game-based learning',
        criteria: award.evidence
      }
    },
    // Note: Proof section requires cryptographic signing - implement separately
    proof: {
      type: 'Ed25519Signature2020',
      created: new Date().toISOString(),
      verificationMethod: `${issuer.id}#keys-1`,
      proofPurpose: 'assertionMethod',
      proofValue: '' // To be filled by signing process
    }
  };
}

// Export default client
export default GameBasedLearningClient;
