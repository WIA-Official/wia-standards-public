/**
 * WIA-EDU-015: Educational Metaverse TypeScript SDK
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

/**
 * Main Educational Metaverse SDK class
 */
export class EducationalMetaverse extends EventEmitter {
  private client: AxiosInstance;
  private config: Types.MetaverseConfig;

  /**
   * Create a new Educational Metaverse instance
   *
   * @param config - Configuration options
   *
   * @example
   * ```typescript
   * const metaverse = new EducationalMetaverse({
   *   apiKey: 'your-api-key',
   *   region: 'us-west-1',
   *   renderQuality: 'high'
   * });
   * ```
   */
  constructor(config: Types.MetaverseConfig) {
    super();
    this.config = config;

    // Initialize HTTP client
    this.client = axios.create({
      baseURL: this.getBaseURL(),
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Version': '1.0.0',
      },
    });
  }

  private getBaseURL(): string {
    const region = this.config.region || 'us-west-1';
    return `https://api-${region}.wia.edu/v1`;
  }

  // =========================================================================
  // Campus Management
  // =========================================================================

  /**
   * Create a new virtual campus
   *
   * @param config - Campus configuration
   * @returns Promise with campus details
   *
   * @example
   * ```typescript
   * const campus = await metaverse.createCampus({
   *   name: 'Innovation University',
   *   template: 'modern-university',
   *   maxConcurrentUsers: 10000,
   *   physicsEnabled: true
   * });
   * ```
   */
  async createCampus(config: Types.CampusConfig): Promise<Types.Campus> {
    const response = await this.client.post('/campus', config);
    return response.data;
  }

  /**
   * Get campus details
   */
  async getCampus(campusId: string): Promise<Types.Campus> {
    const response = await this.client.get(`/campus/${campusId}`);
    return response.data;
  }

  /**
   * Update campus settings
   */
  async updateCampus(
    campusId: string,
    updates: Partial<Types.CampusConfig>
  ): Promise<Types.Campus> {
    const response = await this.client.patch(`/campus/${campusId}`, updates);
    return response.data;
  }

  /**
   * Delete a campus
   */
  async deleteCampus(campusId: string): Promise<void> {
    await this.client.delete(`/campus/${campusId}`);
  }

  /**
   * Create a new space within a campus
   */
  async createSpace(
    campusId: string,
    config: Types.EnvironmentConfig
  ): Promise<any> {
    const response = await this.client.post(
      `/campus/${campusId}/spaces`,
      config
    );
    return response.data;
  }

  // =========================================================================
  // Avatar Management
  // =========================================================================

  /**
   * Configure avatar system for the campus
   */
  async configureAvatars(
    campusId: string,
    config: Types.AvatarConfig
  ): Promise<void> {
    await this.client.post(`/campus/${campusId}/avatars/config`, config);
  }

  /**
   * Create a user avatar
   */
  async createAvatar(userId: string, customization: any): Promise<Types.Avatar> {
    const response = await this.client.post('/avatars', {
      userId,
      customization,
    });
    return response.data;
  }

  /**
   * Update avatar customization
   */
  async updateAvatar(
    avatarId: string,
    customization: Partial<any>
  ): Promise<Types.Avatar> {
    const response = await this.client.patch(
      `/avatars/${avatarId}`,
      customization
    );
    return response.data;
  }

  // =========================================================================
  // Field Trips
  // =========================================================================

  /**
   * Create a virtual field trip experience
   *
   * @example
   * ```typescript
   * const fieldTrip = await campus.createExperience({
   *   type: 'field-trip',
   *   title: 'Ancient Rome Virtual Tour',
   *   destination: 'historical/rome-colosseum',
   *   duration: 45,
   *   features: {
   *     guidedTour: true,
   *     aiNarrator: 'julius-caesar'
   *   }
   * });
   * ```
   */
  async createFieldTrip(
    campusId: string,
    config: Types.FieldTripConfig
  ): Promise<Types.FieldTrip> {
    const response = await this.client.post(
      `/campus/${campusId}/fieldtrips`,
      config
    );
    return response.data;
  }

  /**
   * Start a field trip
   */
  async startFieldTrip(tripId: string): Promise<void> {
    await this.client.post(`/fieldtrips/${tripId}/start`);
    this.emit('fieldtrip:started', { tripId });
  }

  /**
   * End a field trip
   */
  async endFieldTrip(tripId: string): Promise<void> {
    await this.client.post(`/fieldtrips/${tripId}/end`);
    this.emit('fieldtrip:ended', { tripId });
  }

  // =========================================================================
  // Virtual Labs
  // =========================================================================

  /**
   * Create a virtual learning laboratory
   *
   * @example
   * ```typescript
   * const lab = await campus.createLab({
   *   subject: 'chemistry',
   *   type: 'virtual-laboratory',
   *   equipment: ['beakers', 'bunsen-burners'],
   *   experiments: [
   *     { id: 'exp-001', name: 'Chemical Reactions' }
   *   ]
   * });
   * ```
   */
  async createLab(
    campusId: string,
    config: Types.LabConfig
  ): Promise<any> {
    const response = await this.client.post(
      `/campus/${campusId}/labs`,
      config
    );
    return response.data;
  }

  // =========================================================================
  // Social Features
  // =========================================================================

  /**
   * Enable social features for the campus
   */
  async enableSocialFeatures(
    campusId: string,
    features: Types.SocialFeatures
  ): Promise<void> {
    await this.client.post(`/campus/${campusId}/social`, features);
  }

  // =========================================================================
  // Analytics
  // =========================================================================

  /**
   * Get learning analytics for a campus
   *
   * @example
   * ```typescript
   * const analytics = await campus.getAnalytics({
   *   period: 'last-30-days',
   *   metrics: ['active-users', 'engagement-time']
   * });
   * ```
   */
  async getAnalytics(
    campusId: string,
    config: Types.AnalyticsConfig
  ): Promise<Types.AnalyticsData> {
    const response = await this.client.get(`/campus/${campusId}/analytics`, {
      params: config,
    });
    return response.data;
  }

  /**
   * Export learning data
   */
  async exportLearningData(
    campusId: string,
    config: Types.LearningDataConfig
  ): Promise<Types.LearningData> {
    const response = await this.client.post(
      `/campus/${campusId}/learning-data/export`,
      config
    );
    return response.data;
  }

  // =========================================================================
  // Quests & Gamification
  // =========================================================================

  /**
   * Create an educational quest/game
   *
   * @example
   * ```typescript
   * const quest = await campus.createQuest({
   *   title: 'The Great Science Adventure',
   *   type: 'educational-game',
   *   objectives: [
   *     { task: 'Complete chemistry experiment', points: 100 }
   *   ],
   *   rewards: {
   *     badges: ['scientist'],
   *     leaderboard: true
   *   }
   * });
   * ```
   */
  async createQuest(
    campusId: string,
    config: Types.QuestConfig
  ): Promise<Types.Quest> {
    const response = await this.client.post(
      `/campus/${campusId}/quests`,
      config
    );
    return response.data;
  }

  // =========================================================================
  // Session Management
  // =========================================================================

  /**
   * Join a metaverse session
   */
  async joinSession(config: Types.SessionConfig): Promise<void> {
    const response = await this.client.post('/sessions/join', config);
    this.emit('session:joined', response.data);
  }

  /**
   * Leave current session
   */
  async leaveSession(sessionId: string): Promise<void> {
    await this.client.post(`/sessions/${sessionId}/leave`);
    this.emit('session:left', { sessionId });
  }
}

/**
 * Virtual Campus class for managing a specific campus instance
 */
export class VirtualCampus extends EventEmitter {
  private metaverse: EducationalMetaverse;
  public campusId: string;
  public url: string;

  constructor(
    metaverse: EducationalMetaverse,
    campus: Types.Campus
  ) {
    super();
    this.metaverse = metaverse;
    this.campusId = campus.campusId;
    this.url = campus.url;
  }

  /**
   * Create a new space in this campus
   */
  async createSpace(config: Types.EnvironmentConfig): Promise<any> {
    return this.metaverse.createSpace(this.campusId, config);
  }

  /**
   * Create a field trip experience
   */
  async createExperience(config: Types.FieldTripConfig): Promise<Types.FieldTrip> {
    return this.metaverse.createFieldTrip(this.campusId, config);
  }

  /**
   * Create a virtual lab
   */
  async createLab(config: Types.LabConfig): Promise<any> {
    return this.metaverse.createLab(this.campusId, config);
  }

  /**
   * Enable social features
   */
  async enableSocialFeatures(features: Types.SocialFeatures): Promise<void> {
    return this.metaverse.enableSocialFeatures(this.campusId, features);
  }

  /**
   * Get analytics
   */
  async getAnalytics(config: Types.AnalyticsConfig): Promise<Types.AnalyticsData> {
    return this.metaverse.getAnalytics(this.campusId, config);
  }

  /**
   * Create a quest
   */
  async createQuest(config: Types.QuestConfig): Promise<Types.Quest> {
    return this.metaverse.createQuest(this.campusId, config);
  }

  /**
   * Export learning data
   */
  async exportLearningData(
    config: Types.LearningDataConfig
  ): Promise<Types.LearningData> {
    return this.metaverse.exportLearningData(this.campusId, config);
  }
}

// Export version
export const VERSION = '1.0.0';

// Export default
export default EducationalMetaverse;
