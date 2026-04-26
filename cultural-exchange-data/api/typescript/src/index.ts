/**
 * WIA-UNI-011 Cultural Exchange Data Standard
 * TypeScript SDK v2.0
 */

import axios, { AxiosInstance } from 'axios';
import {
  CulturalEvent,
  HeritageItem,
  MediaAsset,
  Participant,
  APIResponse,
  APIError,
  EventFilter,
  HeritageSearchOptions,
  ClientConfig
} from './types';

export * from './types';

/**
 * WIA-UNI-011 API Client
 */
export class CulturalExchangeClient {
  private client: AxiosInstance;

  constructor(config: ClientConfig) {
    this.client = axios.create({
      baseURL: config.baseURL || 'https://api.cultural-exchange.org/v2',
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
        ...config.headers
      }
    });
  }

  /**
   * Events API
   */
  events = {
    /**
     * List cultural events with optional filtering
     */
    list: async (filter?: EventFilter): Promise<APIResponse<CulturalEvent[]>> => {
      const response = await this.client.get('/events', { params: filter });
      return response.data;
    },

    /**
     * Get specific event by ID
     */
    get: async (eventId: string): Promise<CulturalEvent> => {
      const response = await this.client.get(`/events/${eventId}`);
      return response.data;
    },

    /**
     * Create new cultural event
     */
    create: async (event: Partial<CulturalEvent>): Promise<CulturalEvent> => {
      const response = await this.client.post('/events', event);
      return response.data;
    },

    /**
     * Update existing event
     */
    update: async (eventId: string, event: Partial<CulturalEvent>): Promise<CulturalEvent> => {
      const response = await this.client.put(`/events/${eventId}`, event);
      return response.data;
    },

    /**
     * Delete event (soft delete)
     */
    delete: async (eventId: string): Promise<void> => {
      await this.client.delete(`/events/${eventId}`);
    },

    /**
     * Get event participants
     */
    participants: async (eventId: string): Promise<Participant[]> => {
      const response = await this.client.get(`/events/${eventId}/participants`);
      return response.data;
    },

    /**
     * Get event media
     */
    media: async (eventId: string): Promise<MediaAsset[]> => {
      const response = await this.client.get(`/events/${eventId}/media`);
      return response.data;
    }
  };

  /**
   * Heritage API
   */
  heritage = {
    /**
     * Search heritage items
     */
    search: async (options?: HeritageSearchOptions): Promise<APIResponse<HeritageItem[]>> => {
      const response = await this.client.get('/heritage/search', { params: options });
      return response.data;
    },

    /**
     * Get heritage item by ID
     */
    get: async (heritageId: string): Promise<HeritageItem> => {
      const response = await this.client.get(`/heritage/${heritageId}`);
      return response.data;
    },

    /**
     * Get regional variations
     */
    variations: async (heritageId: string) => {
      const response = await this.client.get(`/heritage/${heritageId}/variations`);
      return response.data;
    }
  };

  /**
   * Media API
   */
  media = {
    /**
     * Request upload URL for media file
     */
    uploadURL: async (metadata: {
      filename: string;
      type: string;
      size: number;
      mimeType: string;
    }): Promise<{ assetId: string; uploadUrl: string; expiresIn: number }> => {
      const response = await this.client.post('/media/upload-url', metadata);
      return response.data;
    },

    /**
     * Get media asset by ID
     */
    get: async (assetId: string): Promise<MediaAsset> => {
      const response = await this.client.get(`/media/${assetId}`);
      return response.data;
    }
  };

  /**
   * Analytics API
   */
  analytics = {
    /**
     * Get event trends
     */
    eventTrends: async (params: { period: string; granularity?: string }) => {
      const response = await this.client.get('/analytics/events/trends', { params });
      return response.data;
    },

    /**
     * Get participation metrics
     */
    participation: async (params: { region?: string; granularity?: string }) => {
      const response = await this.client.get('/analytics/participation', { params });
      return response.data;
    }
  };
}

/**
 * Create WIA-UNI-011 API client
 */
export function createClient(config: ClientConfig): CulturalExchangeClient {
  return new CulturalExchangeClient(config);
}

/**
 * Default export
 */
export default CulturalExchangeClient;
