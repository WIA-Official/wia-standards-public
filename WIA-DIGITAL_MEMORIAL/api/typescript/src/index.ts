/**
 * WIA Digital Memorial Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADigitalMemorial, MemorialResponse, MediaAsset, TributeMessage,
  Story, TimelineEntry, ValidationResult, PaginatedResponse,
} from './types';

export class WIADigitalMemorialClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createMemorial(memorial: WIADigitalMemorial): Promise<MemorialResponse> {
    const response = await this.axios.post<MemorialResponse>('/memorials', memorial);
    return response.data;
  }

  async getMemorial(id: string): Promise<WIADigitalMemorial> {
    const response = await this.axios.get<WIADigitalMemorial>(`/memorials/${id}`);
    return response.data;
  }

  async listMemorials(params?: { status?: string; limit?: number }): Promise<PaginatedResponse<MemorialResponse>> {
    const response = await this.axios.get<PaginatedResponse<MemorialResponse>>('/memorials', { params });
    return response.data;
  }

  async updateMemorial(id: string, updates: Partial<WIADigitalMemorial>): Promise<MemorialResponse> {
    const response = await this.axios.put<MemorialResponse>(`/memorials/${id}`, updates);
    return response.data;
  }

  async publishMemorial(id: string): Promise<void> {
    await this.axios.post(`/memorials/${id}/publish`);
  }

  async uploadMedia(memorialId: string, file: File | Blob, metadata: { type: string; caption?: string }): Promise<MediaAsset> {
    const formData = new FormData();
    formData.append('file', file);
    formData.append('metadata', JSON.stringify(metadata));
    const response = await this.axios.post<MediaAsset>(`/memorials/${memorialId}/media`, formData);
    return response.data;
  }

  async addTimelineEntry(memorialId: string, entry: TimelineEntry): Promise<TimelineEntry> {
    const response = await this.axios.post<TimelineEntry>(`/memorials/${memorialId}/timeline`, entry);
    return response.data;
  }

  async submitTribute(memorialId: string, tribute: Omit<TributeMessage, 'id' | 'createdAt' | 'approved' | 'pinned'>): Promise<TributeMessage> {
    const response = await this.axios.post<TributeMessage>(`/memorials/${memorialId}/tributes`, tribute);
    return response.data;
  }

  async getTributes(memorialId: string): Promise<TributeMessage[]> {
    const response = await this.axios.get<TributeMessage[]>(`/memorials/${memorialId}/tributes`);
    return response.data;
  }

  async submitStory(memorialId: string, story: Omit<Story, 'id' | 'publishedAt' | 'reactions'>): Promise<Story> {
    const response = await this.axios.post<Story>(`/memorials/${memorialId}/stories`, story);
    return response.data;
  }

  async lightCandle(memorialId: string, message?: string): Promise<{ id: string }> {
    const response = await this.axios.post(`/memorials/${memorialId}/candles`, { message });
    return response.data;
  }

  async sendFlower(memorialId: string, type: string, message?: string): Promise<{ id: string }> {
    const response = await this.axios.post(`/memorials/${memorialId}/flowers`, { type, message });
    return response.data;
  }

  async getVisitorStats(memorialId: string): Promise<{ totalVisits: number; uniqueVisitors: number; candlesLit: number; flowersReceived: number }> {
    const response = await this.axios.get(`/memorials/${memorialId}/stats`);
    return response.data;
  }

  async exportMemorial(memorialId: string, format: 'pdf' | 'html' | 'json'): Promise<{ url: string; expiresAt: string }> {
    const response = await this.axios.get(`/memorials/${memorialId}/export`, { params: { format } });
    return response.data;
  }

  validateMemorial(memorial: WIADigitalMemorial): ValidationResult {
    const errors: any[] = [];
    if (memorial.standard !== 'WIA-DIGITAL-MEMORIAL') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!memorial.memorial?.id) errors.push({ path: 'memorial.id', message: 'Memorial ID required' });
    if (!memorial.subject?.fullName) errors.push({ path: 'subject.fullName', message: 'Subject name required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalMemorial(name: string, curatorEmail: string): WIADigitalMemorial {
  return {
    standard: 'WIA-DIGITAL-MEMORIAL',
    version: '1.0.0',
    memorial: {
      id: generateUUID(), type: 'personal', status: 'draft', visibility: 'public',
      createdAt: new Date().toISOString(),
      curator: { id: generateUUID(), name: 'Curator', email: curatorEmail, relationship: 'family', verified: false },
    },
    subject: { id: generateUUID(), fullName: name },
    content: { timeline: [], gallery: { albums: [], totalItems: 0, storageUsed: 0 }, stories: [], tributes: [], milestones: [], relationships: [], legacy: [] },
    access: { invitationOnly: false },
    interactions: { allowTributes: true, allowStories: true, allowPhotoUploads: true, allowVideoUploads: true, virtualCandles: true, virtualFlowers: true, guestbook: true, donations: { enabled: false, charities: [] }, anniversaryReminders: true },
    preservation: { duration: 'indefinite', backup: { frequency: 'weekly', locations: ['cloud'], encryption: true }, succession: { successors: [], inactivityPeriod: 365, autoArchive: true } },
  };
}

export default { WIADigitalMemorialClient, generateUUID, createMinimalMemorial };
