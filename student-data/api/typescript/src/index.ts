/**
 * WIA-EDU-010 Student Data Standard - TypeScript SDK
 * Version: 2.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import WebSocket from 'ws';
import { v4 as uuidv4 } from 'uuid';

import {
  ClientConfig,
  StudentProfile,
  AcademicRecord,
  AttendanceRecord,
  Consent,
  PrivacySettings,
  TransferRequest,
  TransferResponse,
  ExportRequest,
  ExportResponse,
  APIResponse,
  PaginatedResponse,
  Transcript,
  DegreeCertificate,
  SyncMessage,
  SyncConfig,
  StudentDataError
} from './types';

/**
 * Main client for interacting with WIA-EDU-010 compliant student data systems
 */
export class StudentDataClient {
  private api: AxiosInstance;
  private config: ClientConfig;
  private ws?: WebSocket;

  constructor(config: ClientConfig) {
    this.config = config;

    this.api = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'Accept': 'application/json',
        'X-WIA-Schema-Version': '2.0.0',
        ...(config.accessToken && { 'Authorization': `Bearer ${config.accessToken}` }),
        ...(config.apiKey && { 'X-API-Key': config.apiKey })
      }
    });

    // Initialize WebSocket for real-time sync if enabled
    if (config.enableSync && config.syncURL) {
      this.initializeSync();
    }
  }

  // ============================================================================
  // Student Profile Operations
  // ============================================================================

  /**
   * Get student profile by ID
   */
  async getStudent(studentId: string): Promise<StudentProfile> {
    try {
      const response = await this.api.get<APIResponse<StudentProfile>>(`/students/${studentId}`);
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to fetch student',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Create a new student
   */
  async createStudent(student: Omit<StudentProfile, 'studentId' | 'metadata'>): Promise<StudentProfile> {
    try {
      const response = await this.api.post<APIResponse<StudentProfile>>('/students', student);
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to create student',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Update student profile
   */
  async updateStudent(studentId: string, updates: Partial<StudentProfile>): Promise<StudentProfile> {
    try {
      const response = await this.api.put<APIResponse<StudentProfile>>(`/students/${studentId}`, updates);
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to update student',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Delete student (soft delete)
   */
  async deleteStudent(studentId: string): Promise<void> {
    try {
      const response = await this.api.delete<APIResponse<void>>(`/students/${studentId}`);
      if (!response.data.success) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to delete student',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * List students with pagination
   */
  async listStudents(options?: {
    page?: number;
    limit?: number;
    filters?: Record<string, any>;
  }): Promise<PaginatedResponse<StudentProfile>> {
    try {
      const params = {
        page: options?.page || 1,
        limit: options?.limit || 50,
        ...options?.filters
      };
      const response = await this.api.get<PaginatedResponse<StudentProfile>>('/students', { params });
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Academic Records Operations
  // ============================================================================

  /**
   * Get academic records for a student
   */
  async getAcademicRecords(studentId: string, filters?: {
    semester?: string;
    year?: number;
  }): Promise<AcademicRecord[]> {
    try {
      const response = await this.api.get<APIResponse<AcademicRecord[]>>(
        `/students/${studentId}/records`,
        { params: filters }
      );
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to fetch academic records',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Add a new academic record
   */
  async addAcademicRecord(studentId: string, record: Omit<AcademicRecord, 'recordId'>): Promise<AcademicRecord> {
    try {
      const response = await this.api.post<APIResponse<AcademicRecord>>(
        `/students/${studentId}/records`,
        record
      );
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to add academic record',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Add a grade for a specific course
   */
  async addGrade(studentId: string, grade: {
    courseCode: string;
    courseName: string;
    credits: number;
    grade: string;
    semester?: string;
    year?: number;
  }): Promise<AcademicRecord> {
    try {
      const response = await this.api.post<APIResponse<AcademicRecord>>(
        `/students/${studentId}/grades`,
        grade
      );
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to add grade',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Get official transcript
   */
  async getTranscript(studentId: string, format: 'json' | 'pdf' | 'credential' = 'json'): Promise<any> {
    try {
      const response = await this.api.get(`/students/${studentId}/transcripts`, {
        params: { format }
      });
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Attendance Operations
  // ============================================================================

  /**
   * Get attendance records
   */
  async getAttendance(studentId: string, courseId?: string): Promise<AttendanceRecord[]> {
    try {
      const response = await this.api.get<APIResponse<AttendanceRecord[]>>(
        `/students/${studentId}/attendance`,
        { params: { courseId } }
      );
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to fetch attendance',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Record attendance
   */
  async recordAttendance(studentId: string, attendance: {
    courseId: string;
    date: string;
    status: string;
    notes?: string;
  }): Promise<AttendanceRecord> {
    try {
      const response = await this.api.post<APIResponse<AttendanceRecord>>(
        `/students/${studentId}/attendance`,
        attendance
      );
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to record attendance',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Privacy & Consent Operations
  // ============================================================================

  /**
   * Get privacy settings
   */
  async getPrivacySettings(studentId: string): Promise<PrivacySettings> {
    try {
      const response = await this.api.get<APIResponse<PrivacySettings>>(`/students/${studentId}/privacy`);
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to fetch privacy settings',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Update privacy settings
   */
  async updatePrivacySettings(studentId: string, settings: Partial<PrivacySettings>): Promise<PrivacySettings> {
    try {
      const response = await this.api.put<APIResponse<PrivacySettings>>(
        `/students/${studentId}/privacy`,
        settings
      );
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to update privacy settings',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Get consent log
   */
  async getConsentLog(studentId: string): Promise<Consent[]> {
    try {
      const response = await this.api.get<APIResponse<Consent[]>>(`/students/${studentId}/consent`);
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to fetch consent log',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Data Export Operations (GDPR)
  // ============================================================================

  /**
   * Request data export
   */
  async requestDataExport(studentId: string, request: ExportRequest): Promise<ExportResponse> {
    try {
      const response = await this.api.post<APIResponse<ExportResponse>>(
        `/students/${studentId}/export`,
        request
      );
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to request export',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Download export
   */
  async downloadExport(studentId: string, exportId: string): Promise<any> {
    try {
      const response = await this.api.get(`/students/${studentId}/export/${exportId}/download`);
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Data Transfer Operations
  // ============================================================================

  /**
   * Initiate data transfer to another institution
   */
  async initiateTransfer(request: TransferRequest): Promise<TransferResponse> {
    try {
      const response = await this.api.post<APIResponse<TransferResponse>>('/transfers', request);
      if (!response.data.success || !response.data.data) {
        throw new StudentDataError(
          response.data.error?.message || 'Failed to initiate transfer',
          response.data.error?.code || 'UNKNOWN_ERROR'
        );
      }
      return response.data.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Real-Time Synchronization
  // ============================================================================

  private initializeSync(): void {
    if (!this.config.syncURL) return;

    this.ws = new WebSocket(this.config.syncURL);

    this.ws.on('open', () => {
      console.log('WebSocket connection established');
      this.sendSyncMessage({
        action: 'connect',
        token: this.config.accessToken
      });
    });

    this.ws.on('message', (data: WebSocket.Data) => {
      const message: SyncMessage = JSON.parse(data.toString());
      this.handleSyncMessage(message);
    });

    this.ws.on('error', (error) => {
      console.error('WebSocket error:', error);
    });

    this.ws.on('close', () => {
      console.log('WebSocket connection closed');
    });
  }

  private sendSyncMessage(message: any): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    }
  }

  private handleSyncMessage(message: SyncMessage): void {
    // Handle different sync message types
    console.log('Sync message received:', message);
    // Implement custom event emitter or callback system here
  }

  /**
   * Close WebSocket connection
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
    }
  }

  // ============================================================================
  // Error Handling
  // ============================================================================

  private handleError(error: any): StudentDataError {
    if (error instanceof StudentDataError) {
      return error;
    }

    if (error.response) {
      const { data, status } = error.response;
      return new StudentDataError(
        data.error?.message || error.message,
        data.error?.code || `HTTP_${status}`,
        data.error?.details
      );
    }

    return new StudentDataError(
      error.message || 'Unknown error occurred',
      'UNKNOWN_ERROR'
    );
  }
}

// Export all types
export * from './types';

// Export default client
export default StudentDataClient;
