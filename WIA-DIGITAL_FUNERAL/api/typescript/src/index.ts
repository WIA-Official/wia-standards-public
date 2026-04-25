/**
 * WIA Digital Funeral Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADigitalFuneral, FuneralResponse, Tribute, MediaAsset,
  RSVPOption, ValidationResult, PaginatedResponse,
} from './types';

export class WIADigitalFuneralClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createService(service: WIADigitalFuneral): Promise<FuneralResponse> {
    const response = await this.axios.post<FuneralResponse>('/services', service);
    return response.data;
  }

  async getService(id: string): Promise<WIADigitalFuneral> {
    const response = await this.axios.get<WIADigitalFuneral>(`/services/${id}`);
    return response.data;
  }

  async listServices(params?: { status?: string; limit?: number }): Promise<PaginatedResponse<FuneralResponse>> {
    const response = await this.axios.get<PaginatedResponse<FuneralResponse>>('/services', { params });
    return response.data;
  }

  async updateService(id: string, updates: Partial<WIADigitalFuneral>): Promise<FuneralResponse> {
    const response = await this.axios.put<FuneralResponse>(`/services/${id}`, updates);
    return response.data;
  }

  async uploadMedia(serviceId: string, file: File | Blob, type: string): Promise<MediaAsset> {
    const formData = new FormData();
    formData.append('file', file);
    formData.append('type', type);
    const response = await this.axios.post<MediaAsset>(`/services/${serviceId}/media`, formData, {
      headers: { 'Content-Type': 'multipart/form-data' },
    });
    return response.data;
  }

  async submitTribute(serviceId: string, tribute: Omit<Tribute, 'id' | 'approved'>): Promise<Tribute> {
    const response = await this.axios.post<Tribute>(`/services/${serviceId}/tributes`, tribute);
    return response.data;
  }

  async approveTribute(serviceId: string, tributeId: string): Promise<void> {
    await this.axios.post(`/services/${serviceId}/tributes/${tributeId}/approve`);
  }

  async getStreamInfo(serviceId: string): Promise<{ url: string; password?: string; startsAt: string; status: string }> {
    const response = await this.axios.get(`/services/${serviceId}/stream`);
    return response.data;
  }

  async startStream(serviceId: string): Promise<void> {
    await this.axios.post(`/services/${serviceId}/stream/start`);
  }

  async endStream(serviceId: string): Promise<void> {
    await this.axios.post(`/services/${serviceId}/stream/end`);
  }

  async submitRSVP(serviceId: string, rsvp: { name: string; email: string; option: RSVPOption; message?: string }): Promise<{ confirmationId: string }> {
    const response = await this.axios.post(`/services/${serviceId}/rsvp`, rsvp);
    return response.data;
  }

  async getRSVPList(serviceId: string): Promise<{ attending: number; virtual: number; declined: number; pending: number }> {
    const response = await this.axios.get(`/services/${serviceId}/rsvp/summary`);
    return response.data;
  }

  async sendCondolence(serviceId: string, condolence: { name: string; email: string; type: string; content: string }): Promise<{ id: string }> {
    const response = await this.axios.post(`/services/${serviceId}/condolences`, condolence);
    return response.data;
  }

  async getCondolences(serviceId: string): Promise<{ id: string; name: string; type: string; content: string; createdAt: string }[]> {
    const response = await this.axios.get(`/services/${serviceId}/condolences`);
    return response.data;
  }

  async getMemorialUrl(serviceId: string): Promise<{ url: string; qrCode: string }> {
    const response = await this.axios.get(`/services/${serviceId}/memorial`);
    return response.data;
  }

  async getRecording(serviceId: string): Promise<{ url: string; expiresAt: string; duration: number }> {
    const response = await this.axios.get(`/services/${serviceId}/recording`);
    return response.data;
  }

  validateService(service: WIADigitalFuneral): ValidationResult {
    const errors: any[] = [];
    if (service.standard !== 'WIA-DIGITAL-FUNERAL') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!service.service?.id) errors.push({ path: 'service.id', message: 'Service ID required' });
    if (!service.deceased?.name) errors.push({ path: 'deceased.name', message: 'Deceased name required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalService(deceasedName: string, scheduledDate: string): WIADigitalFuneral {
  return {
    standard: 'WIA-DIGITAL-FUNERAL',
    version: '1.0.0',
    service: {
      id: generateUUID(), type: 'hybrid', status: 'planning',
      provider: { id: generateUUID(), name: 'Default Provider', type: 'virtual-platform', contact: { email: 'contact@example.com' } },
      scheduledDate, timezone: 'UTC', createdAt: new Date().toISOString(),
    },
    deceased: {
      id: generateUUID(), name: deceasedName, dateOfBirth: '', dateOfDeath: '', photos: [],
    },
    ceremony: {
      venue: { type: 'hybrid', name: 'Virtual Ceremony' },
      program: [], music: [], readings: [], tributes: [],
    },
    streaming: {
      enabled: true, platform: { provider: 'zoom' }, quality: { resolution: '1080p', bitrate: 4000, adaptiveBitrate: true },
      access: { type: 'invitation-only', requireRegistration: true },
      recording: { enabled: true, retentionDays: 365, downloadable: true, editingAllowed: false },
      interactivity: { chat: true, reactions: true, virtualCandles: true, virtualFlowers: true, guestbook: true },
    },
    memorial: {
      enabled: true, type: 'webpage', visibility: 'unlisted', duration: 'permanent',
      features: [{ type: 'photo-gallery', enabled: true }, { type: 'guestbook', enabled: true }],
      customization: { theme: 'classic', layout: 'classic' },
    },
    guestManagement: {
      invitations: { method: ['email'], template: 'default', reminder: true, reminderDays: 1 },
      rsvp: { enabled: true, options: [], collectDietaryInfo: false, collectAccessibilityNeeds: true },
      attendance: { enabled: true, checkInMethod: 'automatic', reportGeneration: true },
      notifications: { types: ['service-reminder', 'stream-starting'], channels: ['email'] },
    },
    condolences: {
      enabled: true, moderation: { autoApprove: false, filterProfanity: true, reviewQueue: true, moderators: [] },
      types: ['message', 'memory', 'virtual-candle'],
    },
  };
}

export default { WIADigitalFuneralClient, generateUUID, createMinimalService };
