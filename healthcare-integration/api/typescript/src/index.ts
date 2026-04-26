/**
 * WIA-UNI-009 Healthcare Integration SDK
 * Official TypeScript SDK for Inter-Korean Healthcare Integration
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import * as crypto from 'crypto-js';
import { v4 as uuidv4 } from 'uuid';

import {
  HealthcareIntegrationConfig,
  Patient,
  MedicalRecord,
  Prescription,
  DiseaseReport,
  DiseaseAlert,
  Consultation,
  PatientTransfer,
  Hospital,
  DrugInteraction,
  AuthToken,
  AccessAuthorization,
  VerificationProof,
  APIResponse,
  PaginatedResponse,
  PaginationParams,
} from './types';

export * from './types';

/**
 * Main Healthcare Integration Client
 */
export class HealthcareIntegration {
  private client: AxiosInstance;
  private config: HealthcareIntegrationConfig;
  private authToken?: AuthToken;

  constructor(config: HealthcareIntegrationConfig) {
    this.config = {
      apiUrl: 'https://api.healthcare-integration.wia.org/v1',
      environment: 'production',
      fhirVersion: 'R4',
      privacyLevel: 'maximum',
      timeout: 30000,
      ...config,
    };

    this.client = axios.create({
      baseURL: this.config.apiUrl,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-Facility-ID': this.config.facility,
        'X-Region': this.config.region,
        'X-API-Version': '1.0',
      },
    });

    // Request interceptor for authentication
    this.client.interceptors.request.use(
      (config) => {
        if (this.authToken) {
          config.headers.Authorization = `Bearer ${this.authToken.token}`;
        }
        config.headers['X-Request-ID'] = uuidv4();
        return config;
      },
      (error) => Promise.reject(error)
    );

    // Response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error) => {
        if (error.response?.status === 401) {
          // Token expired, clear auth
          this.authToken = undefined;
        }
        return Promise.reject(error);
      }
    );
  }

  // ============================================================================
  // Authentication
  // ============================================================================

  /**
   * Authenticate with the healthcare integration system
   */
  async authenticate(apiKey?: string): Promise<AuthToken> {
    const key = apiKey || this.config.apiKey;

    const response = await this.client.post<APIResponse<AuthToken>>('/auth/token', {
      apiKey: key,
      facility: this.config.facility,
      region: this.config.region,
    });

    if (response.data.success && response.data.data) {
      this.authToken = response.data.data;
      return this.authToken;
    }

    throw new Error('Authentication failed');
  }

  /**
   * Refresh authentication token
   */
  async refreshToken(): Promise<AuthToken> {
    if (!this.authToken?.refreshToken) {
      throw new Error('No refresh token available');
    }

    const response = await this.client.post<APIResponse<AuthToken>>('/auth/refresh', {
      refreshToken: this.authToken.refreshToken,
    });

    if (response.data.success && response.data.data) {
      this.authToken = response.data.data;
      return this.authToken;
    }

    throw new Error('Token refresh failed');
  }

  // ============================================================================
  // Patient Records
  // ============================================================================

  /**
   * Get patient medical record with authorization
   */
  async getPatientRecord(params: {
    patientId: string;
    authorization: AccessAuthorization;
  }): Promise<Patient> {
    const response = await this.client.post<APIResponse<Patient>>(
      `/patients/${params.patientId}`,
      {
        authorization: params.authorization,
      }
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to retrieve patient record');
  }

  /**
   * Get patient medical history
   */
  async getPatientMedicalHistory(
    patientId: string,
    pagination?: PaginationParams
  ): Promise<PaginatedResponse<MedicalRecord>> {
    const response = await this.client.get<APIResponse<PaginatedResponse<MedicalRecord>>>(
      `/patients/${patientId}/records`,
      { params: pagination }
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to retrieve medical history');
  }

  /**
   * Get patient medications
   */
  async getPatientMedications(patientId: string): Promise<Prescription[]> {
    const response = await this.client.get<APIResponse<Prescription[]>>(
      `/patients/${patientId}/medications`
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to retrieve patient medications');
  }

  // ============================================================================
  // Disease Surveillance
  // ============================================================================

  /**
   * Report infectious disease case
   */
  async reportInfectiousDisease(report: Omit<DiseaseReport, 'id'>): Promise<DiseaseReport> {
    const response = await this.client.post<APIResponse<DiseaseReport>>(
      '/surveillance/report',
      report
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to submit disease report');
  }

  /**
   * Get current disease alerts
   */
  async getDiseaseAlerts(region?: string): Promise<DiseaseAlert[]> {
    const response = await this.client.get<APIResponse<DiseaseAlert[]>>(
      '/surveillance/alerts',
      { params: { region } }
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to retrieve disease alerts');
  }

  /**
   * Get disease outbreak information
   */
  async getOutbreakInfo(outbreakId: string): Promise<any> {
    const response = await this.client.get<APIResponse<any>>(
      `/surveillance/outbreaks/${outbreakId}`
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to retrieve outbreak information');
  }

  // ============================================================================
  // Hospital Network
  // ============================================================================

  /**
   * Get list of hospitals
   */
  async getHospitals(region?: string): Promise<Hospital[]> {
    const response = await this.client.get<APIResponse<Hospital[]>>('/hospitals', {
      params: { region },
    });

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to retrieve hospitals');
  }

  /**
   * Get hospital capacity
   */
  async getHospitalCapacity(hospitalId: string): Promise<any> {
    const response = await this.client.get<APIResponse<any>>(
      `/hospitals/${hospitalId}/capacity`
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to retrieve hospital capacity');
  }

  /**
   * Request specialist consultation
   */
  async requestConsultation(
    consultation: Omit<Consultation, 'id' | 'status'>
  ): Promise<Consultation> {
    const response = await this.client.post<APIResponse<Consultation>>(
      '/consultations/request',
      consultation
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to request consultation');
  }

  /**
   * Request patient transfer
   */
  async requestPatientTransfer(
    transfer: Omit<PatientTransfer, 'id' | 'status' | 'requestDate'>
  ): Promise<PatientTransfer> {
    const response = await this.client.post<APIResponse<PatientTransfer>>(
      '/patient-transfers/request',
      transfer
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to request patient transfer');
  }

  // ============================================================================
  // Pharmaceutical
  // ============================================================================

  /**
   * Get drug information
   */
  async getDrugInfo(drugId: string): Promise<any> {
    const response = await this.client.get<APIResponse<any>>(`/drugs/${drugId}`);

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to retrieve drug information');
  }

  /**
   * Check drug interactions
   */
  async checkDrugInteractions(medications: string[]): Promise<DrugInteraction[]> {
    const response = await this.client.post<APIResponse<DrugInteraction[]>>(
      '/drug-interactions/check',
      { medications }
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to check drug interactions');
  }

  /**
   * Create prescription
   */
  async createPrescription(
    prescription: Omit<Prescription, 'id' | 'prescribedDate' | 'status'>
  ): Promise<Prescription> {
    const response = await this.client.post<APIResponse<Prescription>>(
      '/prescriptions',
      prescription
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to create prescription');
  }

  /**
   * Request emergency medication
   */
  async requestEmergencyMedication(params: {
    medicationName: string;
    quantity: string;
    location: string;
    reason: string;
  }): Promise<any> {
    const response = await this.client.post<APIResponse<any>>(
      '/emergency-medication/request',
      params
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to request emergency medication');
  }

  // ============================================================================
  // Blockchain Verification
  // ============================================================================

  /**
   * Get verification proof for a transaction
   */
  async getVerificationProof(transactionId: string): Promise<VerificationProof> {
    const response = await this.client.get<APIResponse<VerificationProof>>(
      `/blockchain/proof/${transactionId}`
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error('Failed to retrieve verification proof');
  }

  /**
   * Verify data integrity
   */
  async verifyDataIntegrity(dataHash: string): Promise<boolean> {
    const response = await this.client.post<APIResponse<{ verified: boolean }>>(
      '/blockchain/verify',
      { dataHash }
    );

    if (response.data.success && response.data.data) {
      return response.data.data.verified;
    }

    return false;
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Encrypt sensitive data for transmission
   */
  private encryptData(data: string, key: string): string {
    return crypto.AES.encrypt(data, key).toString();
  }

  /**
   * Decrypt received data
   */
  private decryptData(encryptedData: string, key: string): string {
    const bytes = crypto.AES.decrypt(encryptedData, key);
    return bytes.toString(crypto.enc.Utf8);
  }

  /**
   * Generate data hash for verification
   */
  generateDataHash(data: any): string {
    const jsonString = JSON.stringify(data);
    return crypto.SHA256(jsonString).toString();
  }

  /**
   * Check if authenticated
   */
  isAuthenticated(): boolean {
    if (!this.authToken) return false;
    return new Date(this.authToken.expiresAt) > new Date();
  }
}

/**
 * Create a new Healthcare Integration client
 */
export function createHealthcareIntegrationClient(
  config: HealthcareIntegrationConfig
): HealthcareIntegration {
  return new HealthcareIntegration(config);
}

/**
 * Default export
 */
export default HealthcareIntegration;
