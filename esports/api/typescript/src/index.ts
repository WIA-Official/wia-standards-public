/**
 * WIA-EDU-022 E-Sports Education Standard - TypeScript SDK
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @standard WIA-EDU-022
 */

import {
  ClientConfig,
  Program,
  Team,
  Player,
  Match,
  CareerPathway,
  ListProgramsRequest,
  ListProgramsResponse,
  ListTeamsRequest,
  ListTeamsResponse,
  ListPlayersRequest,
  ListPlayersResponse,
  ListMatchesRequest,
  ListMatchesResponse,
  CreateProgramRequest,
  UpdateProgramRequest,
  CreateTeamRequest,
  UpdateTeamRequest,
  CreatePlayerRequest,
  UpdatePlayerRequest,
  CreateMatchRequest,
  UpdateMatchRequest,
  APIError,
  WebhookSubscription,
  WebhookEvent,
} from './types';

export * from './types';

// ============================================================================
// Client
// ============================================================================

export class EsportsClient {
  private baseURL: string;
  private apiKey?: string;
  private accessToken?: string;
  private timeout: number;

  constructor(config: ClientConfig) {
    this.apiKey = config.apiKey;
    this.accessToken = config.accessToken;
    this.timeout = config.timeout || 30000;

    if (config.baseURL) {
      this.baseURL = config.baseURL;
    } else if (config.environment === 'sandbox') {
      this.baseURL = 'https://sandbox-api.wia-esports.edu/v1';
    } else {
      this.baseURL = 'https://api.wia-esports.edu/v1';
    }
  }

  // Programs API
  public programs = {
    list: (params?: ListProgramsRequest): Promise<ListProgramsResponse> =>
      this.request('GET', '/programs', { params }),

    get: (programId: string): Promise<Program> =>
      this.request('GET', `/programs/${programId}`),

    create: (data: CreateProgramRequest): Promise<Program> =>
      this.request('POST', '/programs', { data }),

    update: (programId: string, data: UpdateProgramRequest): Promise<Program> =>
      this.request('PATCH', `/programs/${programId}`, { data }),

    delete: (programId: string): Promise<void> =>
      this.request('DELETE', `/programs/${programId}`),
  };

  // Teams API
  public teams = {
    list: (params: ListTeamsRequest): Promise<ListTeamsResponse> =>
      this.request('GET', '/teams', { params }),

    get: (teamId: string): Promise<Team> =>
      this.request('GET', `/teams/${teamId}`),

    create: (data: CreateTeamRequest): Promise<Team> =>
      this.request('POST', '/teams', { data }),

    update: (teamId: string, data: UpdateTeamRequest): Promise<Team> =>
      this.request('PATCH', `/teams/${teamId}`, { data }),

    updateRoster: (teamId: string, roster: { starters: string[]; substitutes: string[] }): Promise<Team> =>
      this.request('PATCH', `/teams/${teamId}/roster`, { data: roster }),

    delete: (teamId: string): Promise<void> =>
      this.request('DELETE', `/teams/${teamId}`),
  };

  // Players API
  public players = {
    list: (params: ListPlayersRequest): Promise<ListPlayersResponse> =>
      this.request('GET', '/players', { params }),

    get: (playerId: string): Promise<Player> =>
      this.request('GET', `/players/${playerId}`),

    create: (data: CreatePlayerRequest): Promise<Player> =>
      this.request('POST', '/players', { data }),

    update: (playerId: string, data: UpdatePlayerRequest): Promise<Player> =>
      this.request('PATCH', `/players/${playerId}`, { data }),

    assignToTeam: (
      playerId: string,
      assignment: { teamId: string; role: string; position: string; joinedDate: string }
    ): Promise<Player> =>
      this.request('POST', `/players/${playerId}/teams`, { data: assignment }),

    delete: (playerId: string): Promise<void> =>
      this.request('DELETE', `/players/${playerId}`),
  };

  // Matches API
  public matches = {
    list: (params: ListMatchesRequest): Promise<ListMatchesResponse> =>
      this.request('GET', '/matches', { params }),

    get: (matchId: string): Promise<Match> =>
      this.request('GET', `/matches/${matchId}`),

    create: (data: CreateMatchRequest): Promise<Match> =>
      this.request('POST', '/matches', { data }),

    update: (matchId: string, data: UpdateMatchRequest): Promise<Match> =>
      this.request('PATCH', `/matches/${matchId}`, { data }),

    updateResult: (
      matchId: string,
      result: { score: { team: number; opponent: number }; outcome: string; duration: number }
    ): Promise<Match> =>
      this.request('PATCH', `/matches/${matchId}/result`, { data: result }),

    delete: (matchId: string): Promise<void> =>
      this.request('DELETE', `/matches/${matchId}`),
  };

  // Career Pathways API
  public career = {
    get: (playerId: string): Promise<CareerPathway> =>
      this.request('GET', `/players/${playerId}/career`),

    updateInterests: (playerId: string, interests: string[]): Promise<CareerPathway> =>
      this.request('PATCH', `/players/${playerId}/career/interests`, { data: { interests } }),

    addExperience: (playerId: string, experience: any): Promise<CareerPathway> =>
      this.request('POST', `/players/${playerId}/career/experiences`, { data: experience }),

    addAchievement: (playerId: string, achievement: any): Promise<CareerPathway> =>
      this.request('POST', `/players/${playerId}/career/achievements`, { data: achievement }),

    updateGoals: (playerId: string, goals: any[]): Promise<CareerPathway> =>
      this.request('PATCH', `/players/${playerId}/career/goals`, { data: { goals } }),
  };

  // Webhooks API
  public webhooks = {
    list: (): Promise<WebhookSubscription[]> =>
      this.request('GET', '/webhooks'),

    create: (data: { url: string; events: WebhookEvent[]; secret: string }): Promise<WebhookSubscription> =>
      this.request('POST', '/webhooks', { data }),

    delete: (webhookId: string): Promise<void> =>
      this.request('DELETE', `/webhooks/${webhookId}`),
  };

  // ============================================================================
  // Private Methods
  // ============================================================================

  private async request<T>(
    method: string,
    path: string,
    options?: { params?: any; data?: any }
  ): Promise<T> {
    const url = new URL(path, this.baseURL);

    // Add query parameters
    if (options?.params) {
      Object.keys(options.params).forEach((key) => {
        if (options.params[key] !== undefined) {
          url.searchParams.append(key, options.params[key]);
        }
      });
    }

    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
    };

    // Add authentication
    if (this.accessToken) {
      headers['Authorization'] = `Bearer ${this.accessToken}`;
    } else if (this.apiKey) {
      headers['X-API-Key'] = this.apiKey;
    }

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      const response = await fetch(url.toString(), {
        method,
        headers,
        body: options?.data ? JSON.stringify(options.data) : undefined,
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        const error: APIError = await response.json();
        throw new Error(`API Error: ${error.code} - ${error.message}`);
      }

      // Handle 204 No Content
      if (response.status === 204) {
        return undefined as T;
      }

      return await response.json();
    } catch (error) {
      clearTimeout(timeoutId);
      if (error instanceof Error && error.name === 'AbortError') {
        throw new Error('Request timeout');
      }
      throw error;
    }
  }
}

// ============================================================================
// Factory Function
// ============================================================================

export function createClient(config: ClientConfig): EsportsClient {
  return new EsportsClient(config);
}

// ============================================================================
// Utility Functions
// ============================================================================

export function verifyWebhookSignature(
  payload: string,
  signature: string,
  secret: string
): boolean {
  // Implementation would use crypto to verify HMAC signature
  // This is a placeholder
  return true;
}

export function formatGamerTag(tag: string): string {
  // Remove special characters and normalize
  return tag.trim().replace(/[^a-zA-Z0-9_-]/g, '');
}

export function calculateWinRate(wins: number, total: number): number {
  if (total === 0) return 0;
  return Math.round((wins / total) * 100 * 10) / 10;
}

export function calculateSkillRating(
  baseRating: number,
  winRate: number,
  gamesPlayed: number
): number {
  // Simple ELO-like calculation (placeholder)
  const confidence = Math.min(gamesPlayed / 50, 1);
  const ratingAdjustment = (winRate - 50) * 10 * confidence;
  return Math.round(baseRating + ratingAdjustment);
}

// ============================================================================
// Constants
// ============================================================================

export const SUPPORTED_GAMES = [
  'League of Legends',
  'Valorant',
  'Rocket League',
  'Overwatch',
  'Minecraft',
  'Fortnite',
  'Super Smash Bros. Ultimate',
  'NBA 2K',
  'Madden NFL',
] as const;

export const GRADE_LEVELS = {
  MIDDLE_SCHOOL: { min: 6, max: 8 },
  HIGH_SCHOOL: { min: 9, max: 12 },
  COLLEGE: { min: 13, max: 16 },
} as const;

export const CERTIFICATION_LEVELS = ['bronze', 'silver', 'gold', 'platinum'] as const;

// ============================================================================
// Default Export
// ============================================================================

export default { createClient, EsportsClient };
