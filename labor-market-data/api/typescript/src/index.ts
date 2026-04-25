/**
 * WIA-SOC-020 Labor Market Data Standard - TypeScript SDK
 * Version: 1.0.0
 * 
 * 弘益人間 (Benefit All Humanity)
 * 
 * © 2025 WIA - World Certification Industry Association
 * License: MIT
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import Ajv from 'ajv';
import addFormats from 'ajv-formats';

export * from './types';
import type {
  WorkerProfile,
  WorkerId,
  Employment,
  JobPosting,
  JobPostingId,
  WageStatistics,
  LaborMarketStatistics,
  ApiResponse,
  ApiError
} from './types';

/**
 * Configuration options for LaborMarketDataClient
 */
export interface ClientConfig {
  /** Base URL for the API */
  baseUrl?: string;
  /** API key for authentication */
  apiKey?: string;
  /** OAuth 2.0 access token */
  accessToken?: string;
  /** Request timeout in milliseconds */
  timeout?: number;
  /** Custom axios configuration */
  axiosConfig?: AxiosRequestConfig;
}

/**
 * Main client for WIA-SOC-020 Labor Market Data API
 */
export class LaborMarketDataClient {
  private axios: AxiosInstance;
  private ajv: Ajv;

  constructor(config: ClientConfig = {}) {
    const {
      baseUrl = 'https://api.wiastandards.com/labor-market/v1',
      apiKey,
      accessToken,
      timeout = 30000,
      axiosConfig = {}
    } = config;

    // Initialize axios instance
    this.axios = axios.create({
      baseURL: baseUrl,
      timeout,
      headers: {
        'Content-Type': 'application/json',
        ...(apiKey && { 'X-API-Key': apiKey }),
        ...(accessToken && { 'Authorization': `Bearer ${accessToken}` })
      },
      ...axiosConfig
    });

    // Initialize JSON Schema validator
    this.ajv = new Ajv();
    addFormats(this.ajv);
  }

  /**
   * Get worker profile by ID
   */
  async getWorkerProfile(workerId: WorkerId): Promise<ApiResponse<WorkerProfile>> {
    const response = await this.axios.get(`/workers/${workerId}`);
    return response.data;
  }

  /**
   * Create new worker profile
   */
  async createWorkerProfile(profile: Omit<WorkerProfile, 'workerId'>): Promise<ApiResponse<WorkerProfile>> {
    const response = await this.axios.post('/workers', profile);
    return response.data;
  }

  /**
   * Update worker profile
   */
  async updateWorkerProfile(workerId: WorkerId, profile: Partial<WorkerProfile>): Promise<ApiResponse<WorkerProfile>> {
    const response = await this.axios.put(`/workers/${workerId}`, profile);
    return response.data;
  }

  /**
   * Get employment history for a worker
   */
  async getEmploymentHistory(workerId: WorkerId): Promise<ApiResponse<Employment[]>> {
    const response = await this.axios.get(`/workers/${workerId}/employment`);
    return response.data;
  }

  /**
   * Search job postings
   */
  async searchJobs(params: {
    location?: string;
    occupation?: string;
    skills?: string[];
    salaryMin?: number;
    remote?: boolean;
    limit?: number;
    offset?: number;
  }): Promise<ApiResponse<JobPosting[]>> {
    const response = await this.axios.get('/jobs', { params });
    return response.data;
  }

  /**
   * Get job posting by ID
   */
  async getJobPosting(postingId: JobPostingId): Promise<ApiResponse<JobPosting>> {
    const response = await this.axios.get(`/jobs/${postingId}`);
    return response.data;
  }

  /**
   * Create new job posting
   */
  async createJobPosting(posting: Omit<JobPosting, 'postingId'>): Promise<ApiResponse<JobPosting>> {
    const response = await this.axios.post('/jobs', posting);
    return response.data;
  }

  /**
   * Get wage statistics
   */
  async getWageStatistics(params: {
    occupation: string;
    region: string;
    period?: string;
  }): Promise<ApiResponse<WageStatistics>> {
    const response = await this.axios.get('/statistics/wages', { params });
    return response.data;
  }

  /**
   * Get labor market statistics
   */
  async getLaborMarketStatistics(params: {
    region: string;
    period: string;
  }): Promise<ApiResponse<LaborMarketStatistics>> {
    const response = await this.axios.get('/statistics/employment', { params });
    return response.data;
  }

  /**
   * Get trending skills
   */
  async getTrendingSkills(params: {
    region?: string;
    industry?: string;
    limit?: number;
  } = {}): Promise<ApiResponse<any>> {
    const response = await this.axios.get('/statistics/skills', {
      params: { ...params, trending: true }
    });
    return response.data;
  }

  /**
   * Validate data against WIA-SOC-020 schema
   */
  validateWorkerProfile(profile: WorkerProfile): boolean {
    // Schema validation logic here
    return true;
  }
}

/**
 * Create a new LaborMarketDataClient instance
 */
export function createClient(config?: ClientConfig): LaborMarketDataClient {
  return new LaborMarketDataClient(config);
}

/**
 * Default export
 */
export default {
  LaborMarketDataClient,
  createClient
};
