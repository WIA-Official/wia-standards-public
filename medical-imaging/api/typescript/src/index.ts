/**
 * WIA-MED-015: Medical Imaging Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import {
  MedicalImage,
  Modality,
  DICOMTags,
  ImageMetadata,
  ImagingStudy,
  SeriesInfo,
  ImageAnnotation,
  RadiologyReport,
  StudyStatus,
  APIResponse,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAMedicalImagingConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  debug?: boolean;
  institutionId?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class MedicalImagingSDK {
  private config: Required<WIAMedicalImagingConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAMedicalImagingConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/imaging',
      timeout: 60000,
      debug: false,
      institutionId: '',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Study Operations
  // ==========================================================================

  async getStudy(studyId: string): Promise<APIResponse<ImagingStudy>> {
    return this.makeRequest('GET', `/studies/${studyId}`);
  }

  async listStudies(filters?: {
    patientId?: string;
    modality?: Modality;
    status?: StudyStatus;
    startDate?: string;
    endDate?: string;
  }): Promise<PaginatedResponse<ImagingStudy>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/studies?${params}`);
  }

  async createStudy(study: Omit<ImagingStudy, 'studyId'>): Promise<APIResponse<ImagingStudy>> {
    return this.makeRequest('POST', '/studies', study);
  }

  async updateStudyStatus(studyId: string, status: StudyStatus): Promise<APIResponse<ImagingStudy>> {
    return this.makeRequest('PATCH', `/studies/${studyId}/status`, { status });
  }

  // ==========================================================================
  // Series Operations
  // ==========================================================================

  async getSeries(studyId: string, seriesId: string): Promise<APIResponse<SeriesInfo>> {
    return this.makeRequest('GET', `/studies/${studyId}/series/${seriesId}`);
  }

  async listSeries(studyId: string): Promise<PaginatedResponse<SeriesInfo>> {
    return this.makeRequest('GET', `/studies/${studyId}/series`);
  }

  // ==========================================================================
  // Image Operations
  // ==========================================================================

  async getImage(imageId: string): Promise<APIResponse<MedicalImage>> {
    return this.makeRequest('GET', `/images/${imageId}`);
  }

  async uploadImage(image: MedicalImage, file: Blob): Promise<APIResponse<{ imageId: string; uploadedAt: string }>> {
    const formData = new FormData();
    formData.append('metadata', JSON.stringify(image));
    formData.append('file', file);

    const response = await fetch(`${this.config.endpoint}/images/upload`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'X-WIA-Standard': 'MED-015',
      },
      body: formData,
    });

    return response.json();
  }

  async downloadImage(imageId: string): Promise<Blob> {
    const response = await fetch(`${this.config.endpoint}/images/${imageId}/download`, {
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
      },
    });
    return response.blob();
  }

  // ==========================================================================
  // Annotation Operations
  // ==========================================================================

  async getAnnotations(imageId: string): Promise<PaginatedResponse<ImageAnnotation>> {
    return this.makeRequest('GET', `/images/${imageId}/annotations`);
  }

  async createAnnotation(imageId: string, annotation: Omit<ImageAnnotation, 'annotationId'>): Promise<APIResponse<ImageAnnotation>> {
    return this.makeRequest('POST', `/images/${imageId}/annotations`, annotation);
  }

  async deleteAnnotation(imageId: string, annotationId: string): Promise<APIResponse<void>> {
    return this.makeRequest('DELETE', `/images/${imageId}/annotations/${annotationId}`);
  }

  // ==========================================================================
  // Report Operations
  // ==========================================================================

  async getReport(studyId: string): Promise<APIResponse<RadiologyReport>> {
    return this.makeRequest('GET', `/studies/${studyId}/report`);
  }

  async createReport(studyId: string, report: Omit<RadiologyReport, 'reportId' | 'createdAt'>): Promise<APIResponse<RadiologyReport>> {
    return this.makeRequest('POST', `/studies/${studyId}/report`, report);
  }

  async signReport(studyId: string, radiologistId: string): Promise<APIResponse<RadiologyReport>> {
    return this.makeRequest('POST', `/studies/${studyId}/report/sign`, { radiologistId });
  }

  // ==========================================================================
  // DICOM Utilities
  // ==========================================================================

  createImageMetadata(studyId: string, patientId: string, modality: Modality, bodyPart: string, options?: Partial<ImageMetadata>): MedicalImage {
    return {
      format: 'WIA-MEDICAL-IMAGING-v1.0',
      imageId: `IMG-${Date.now()}`,
      studyId,
      seriesId: `SER-${Date.now()}`,
      patientId,
      modality,
      bodyPart,
      dicomTags: {
        studyInstanceUID: this.generateUID(),
        seriesInstanceUID: this.generateUID(),
        sopInstanceUID: this.generateUID(),
      },
      imageMetadata: {
        rows: options?.rows || 512,
        columns: options?.columns || 512,
        bitsAllocated: options?.bitsAllocated || 16,
        ...options,
      },
      fileInfo: { sizeBytes: 0, format: 'DICOM' as const, storageLocation: '' },
      acquisitionDate: new Date().toISOString(),
      createdAt: new Date().toISOString(),
    };
  }

  validateDICOM(image: MedicalImage): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (!image.dicomTags.studyInstanceUID) errors.push('Missing Study Instance UID');
    if (!image.dicomTags.seriesInstanceUID) errors.push('Missing Series Instance UID');
    if (!image.dicomTags.sopInstanceUID) errors.push('Missing SOP Instance UID');

    return { valid: errors.length === 0, errors };
  }

  private generateUID(): string {
    return `1.2.840.${Date.now()}.${Math.floor(Math.random() * 1000000)}`;
  }

  static generateStudyId(): string {
    return `STU-${Date.now().toString(36).toUpperCase()}`;
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'studyCreated' | 'imageUploaded' | 'reportSigned' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Medical Imaging] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'MED-015',
          'X-WIA-Version': '1.0.0',
          ...(this.config.institutionId && { 'X-Institution-ID': this.config.institutionId }),
        },
        body: body ? JSON.stringify(body) : undefined,
      });

      return await response.json();
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Unknown error';
      this.eventEmitter.emit('error', { code: 'REQUEST_FAILED', message });
      throw error;
    }
  }
}

export function createClient(config: WIAMedicalImagingConfig): MedicalImagingSDK {
  return new MedicalImagingSDK(config);
}

export default MedicalImagingSDK;
