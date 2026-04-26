/**
 * WIA-MED-007: Hospital Information System Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import {
  Patient,
  Appointment,
  Admission,
  BedAssignment,
  ClinicalOrder,
  APIResponse,
  PaginatedResponse,
  PatientStatus,
  AppointmentStatus,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAHISConfig {
  baseUrl: string;
  apiKey: string;
  timeout?: number;
  debug?: boolean;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIAHIS {
  private config: Required<WIAHISConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAHISConfig) {
    this.config = {
      timeout: 30000,
      debug: false,
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Patient Operations
  // ==========================================================================

  async getPatients(filters?: {
    status?: PatientStatus;
    department?: string;
    page?: number;
    pageSize?: number;
  }): Promise<PaginatedResponse<Patient>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.request(`/api/v1/patients?${params}`);
  }

  async getPatient(mrn: string): Promise<APIResponse<Patient>> {
    return this.request(`/api/v1/patients/${mrn}`);
  }

  async createPatient(patient: Omit<Patient, 'mrn' | 'createdAt'>): Promise<APIResponse<Patient>> {
    return this.request('/api/v1/patients', {
      method: 'POST',
      body: JSON.stringify(patient),
    });
  }

  async updatePatient(mrn: string, updates: Partial<Patient>): Promise<APIResponse<Patient>> {
    return this.request(`/api/v1/patients/${mrn}`, {
      method: 'PATCH',
      body: JSON.stringify(updates),
    });
  }

  async searchPatients(query: string): Promise<PaginatedResponse<Patient>> {
    return this.request(`/api/v1/patients/search?q=${encodeURIComponent(query)}`);
  }

  // ==========================================================================
  // Appointment Operations
  // ==========================================================================

  async getAppointments(filters?: {
    patientMrn?: string;
    providerId?: string;
    status?: AppointmentStatus;
    startDate?: string;
    endDate?: string;
  }): Promise<PaginatedResponse<Appointment>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.request(`/api/v1/appointments?${params}`);
  }

  async getAppointment(appointmentId: string): Promise<APIResponse<Appointment>> {
    return this.request(`/api/v1/appointments/${appointmentId}`);
  }

  async createAppointment(appointment: Partial<Appointment>): Promise<APIResponse<Appointment>> {
    return this.request('/api/v1/appointments', {
      method: 'POST',
      body: JSON.stringify(appointment),
    });
  }

  async updateAppointment(appointmentId: string, updates: Partial<Appointment>): Promise<APIResponse<Appointment>> {
    return this.request(`/api/v1/appointments/${appointmentId}`, {
      method: 'PATCH',
      body: JSON.stringify(updates),
    });
  }

  async cancelAppointment(appointmentId: string, reason?: string): Promise<APIResponse<Appointment>> {
    return this.request(`/api/v1/appointments/${appointmentId}/cancel`, {
      method: 'POST',
      body: JSON.stringify({ reason }),
    });
  }

  // ==========================================================================
  // Admission Operations
  // ==========================================================================

  async getAdmissions(filters?: {
    patientMrn?: string;
    status?: string;
    department?: string;
  }): Promise<PaginatedResponse<Admission>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.request(`/api/v1/admissions?${params}`);
  }

  async getAdmission(admissionId: string): Promise<APIResponse<Admission>> {
    return this.request(`/api/v1/admissions/${admissionId}`);
  }

  async createAdmission(admission: Partial<Admission>): Promise<APIResponse<Admission>> {
    return this.request('/api/v1/admissions', {
      method: 'POST',
      body: JSON.stringify(admission),
    });
  }

  async dischargePatient(admissionId: string, dischargeInfo: {
    dischargeDate: string;
    disposition: string;
    instructions?: string;
  }): Promise<APIResponse<Admission>> {
    return this.request(`/api/v1/admissions/${admissionId}/discharge`, {
      method: 'POST',
      body: JSON.stringify(dischargeInfo),
    });
  }

  // ==========================================================================
  // Bed Management
  // ==========================================================================

  async getBedAssignments(department?: string): Promise<PaginatedResponse<BedAssignment>> {
    const params = department ? `?department=${department}` : '';
    return this.request(`/api/v1/beds${params}`);
  }

  async assignBed(assignment: Partial<BedAssignment>): Promise<APIResponse<BedAssignment>> {
    return this.request('/api/v1/beds/assign', {
      method: 'POST',
      body: JSON.stringify(assignment),
    });
  }

  // ==========================================================================
  // Clinical Orders
  // ==========================================================================

  async createOrder(order: Partial<ClinicalOrder>): Promise<APIResponse<ClinicalOrder>> {
    return this.request('/api/v1/orders', {
      method: 'POST',
      body: JSON.stringify(order),
    });
  }

  async getOrders(patientMrn: string): Promise<PaginatedResponse<ClinicalOrder>> {
    return this.request(`/api/v1/orders?patientMrn=${patientMrn}`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'patientAdmitted' | 'patientDischarged' | 'appointmentCreated' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async request<T = APIResponse>(url: string, options: RequestInit = {}): Promise<T> {
    const fullUrl = `${this.config.baseUrl}${url}`;
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'MED-007',
      'X-WIA-Version': '1.0.0',
      ...options.headers,
    };

    if (this.config.debug) {
      console.log(`[WIA HIS] ${options.method || 'GET'} ${fullUrl}`);
    }

    try {
      const response = await fetch(fullUrl, { ...options, headers });
      const data = await response.json();

      if (!response.ok) {
        this.eventEmitter.emit('error', data);
      }

      return data;
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Unknown error';
      this.eventEmitter.emit('error', { code: 'REQUEST_FAILED', message });
      throw error;
    }
  }
}

export function createClient(config: WIAHISConfig): WIAHIS {
  return new WIAHIS(config);
}

export default WIAHIS;
