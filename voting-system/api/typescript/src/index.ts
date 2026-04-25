/**
 * WIA-SOC-015 Voting System Standard - TypeScript SDK
 * Version: 1.0.0
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 * © 2025 World Certification Industry Association (WIA)
 * License: MIT
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import * as jwt from 'jsonwebtoken';
import * as CryptoJS from 'crypto-js';

// Export all types
export * from './types';

import {
  WIAVotingConfig,
  VoterRegistrationRequest,
  VoterRegistrationResponse,
  Ballot,
  VoteSubmission,
  VoteSubmissionResponse,
  ElectionResult,
  VerificationRequest,
  VerificationResponse,
  AuditEvent,
  PaginationParams,
  APIError,
} from './types';

/**
 * Main SDK class for interacting with WIA-SOC-015 compliant voting systems
 */
export class WIAVotingSDK {
  private client: AxiosInstance;
  private accessToken?: string;

  constructor(config: WIAVotingConfig) {
    this.client = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'User-Agent': 'WIA-SOC-015-SDK/1.0.0',
        ...config.headers,
      },
    });

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error: AxiosError<APIError>) => {
        if (error.response?.data) {
          throw new WIAAPIError(error.response.data);
        }
        throw error;
      }
    );
  }

  /**
   * Set access token for authenticated requests
   */
  setAccessToken(token: string): void {
    this.accessToken = token;
    this.client.defaults.headers.common['Authorization'] = 'Bearer ' + token;
  }

  /**
   * Register a new voter
   */
  async registerVoter(request: VoterRegistrationRequest): Promise<VoterRegistrationResponse> {
    const response = await this.client.post<VoterRegistrationResponse>('/v1/voters/register', request);
    return response.data;
  }

  /**
   * Get ballot for a specific voter and election
   */
  async getBallot(electionId: string, voterId: string): Promise<Ballot> {
    const response = await this.client.get<Ballot>('/v1/elections/' + electionId + '/ballots/' + voterId);
    return response.data;
  }

  /**
   * Submit a completed ballot
   */
  async submitVote(electionId: string, vote: VoteSubmission): Promise<VoteSubmissionResponse> {
    const response = await this.client.post<VoteSubmissionResponse>('/v1/elections/' + electionId + '/votes', vote);
    return response.data;
  }

  /**
   * Get election results
   */
  async getResults(electionId: string, options?: any): Promise<ElectionResult> {
    const params = new URLSearchParams();
    if (options?.contest) params.append('contest', options.contest);
    if (options?.level) params.append('level', options.level);

    const response = await this.client.get<ElectionResult>('/v1/elections/' + electionId + '/results?' + params.toString());
    return response.data;
  }

  /**
   * Verify ballot inclusion using receipt
   */
  async verifyBallot(request: VerificationRequest): Promise<VerificationResponse> {
    const response = await this.client.get<VerificationResponse>('/v1/verification/' + request.receiptId);
    return response.data;
  }

  /**
   * Generate cryptographic hash of ballot
   */
  generateBallotHash(ballot: Ballot): string {
    const ballotString = JSON.stringify(ballot);
    return CryptoJS.SHA3(ballotString, { outputLength: 256 }).toString();
  }
}

/**
 * Custom error class for WIA API errors
 */
export class WIAAPIError extends Error {
  public code: string;
  public details?: string;
  public timestamp: string;
  public requestId: string;

  constructor(error: APIError) {
    super(error.message);
    this.name = 'WIAAPIError';
    this.code = error.code;
    this.details = error.details;
    this.timestamp = error.timestamp;
    this.requestId = error.requestId;

    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, WIAAPIError);
    }
  }
}

/**
 * Helper function to create SDK instance
 */
export function createWIAVotingSDK(config: WIAVotingConfig): WIAVotingSDK {
  return new WIAVotingSDK(config);
}

export default WIAVotingSDK;
