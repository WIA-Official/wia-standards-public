/**
 * WIA-TRADITIONAL-MEDICINE TypeScript SDK
 * 弘益人間 - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import * as Types from './types';

export * from './types';

export interface WIAClientConfig {
  apiKey: string;
  baseUrl?: string;
  timeout?: number;
}

/**
 * WIA Traditional Medicine API Client
 */
export class WIATraditionalMedicine {
  private client: AxiosInstance;
  private apiKey: string;

  constructor(config: WIAClientConfig) {
    this.apiKey = config.apiKey;
    this.client = axios.create({
      baseURL: config.baseUrl || 'https://api.wia.live/tm/v1',
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
      },
    });
  }

  /**
   * Constitution Assessment APIs
   */
  constitution = {
    /**
     * Submit constitution assessment questionnaire
     */
    assess: async (
      request: Types.ConstitutionAssessmentRequest
    ): Promise<Types.ConstitutionAssessmentResponse> => {
      const response = await this.client.post('/constitution/assess', request);
      return response.data;
    },

    /**
     * Get patient's constitution profile
     */
    getProfile: async (patientId: string): Promise<Types.ConstitutionalProfile> => {
      const response = await this.client.get(`/constitution/profile/${patientId}`);
      return response.data;
    },

    /**
     * Get constitution recommendations based on type
     */
    getRecommendations: async (
      constitutionType: Types.TCMConstitutionType | Types.SasangConstitutionType
    ): Promise<{ diet: string[]; lifestyle: string[]; herbs: string[] }> => {
      const response = await this.client.get(`/constitution/recommendations/${constitutionType}`);
      return response.data;
    },
  };

  /**
   * Diagnosis APIs
   */
  diagnosis = {
    /**
     * Submit traditional diagnosis record
     */
    submit: async (
      diagnosis: Omit<Types.TraditionalDiagnosis, 'diagnosis_id'>
    ): Promise<Types.TraditionalDiagnosis> => {
      const response = await this.client.post('/diagnosis/traditional', diagnosis);
      return response.data;
    },

    /**
     * Analyze tongue image
     */
    analyzeTongue: async (
      imageFile: File | Blob,
      patientId: string
    ): Promise<Types.TongueAnalysisResponse> => {
      const formData = new FormData();
      formData.append('image', imageFile);
      formData.append('patient_id', patientId);

      const response = await this.client.post('/diagnosis/tongue/analyze', formData, {
        headers: { 'Content-Type': 'multipart/form-data' },
      });
      return response.data;
    },

    /**
     * Get patient diagnosis history
     */
    getHistory: async (
      patientId: string,
      options?: { limit?: number; offset?: number }
    ): Promise<Types.TraditionalDiagnosis[]> => {
      const response = await this.client.get(`/diagnosis/history/${patientId}`, {
        params: options,
      });
      return response.data;
    },

    /**
     * Get pattern diagnosis by ICD-11 TM code
     */
    getPattern: async (icd11Code: string): Promise<Types.PatternDiagnosis> => {
      const response = await this.client.get(`/diagnosis/pattern/${icd11Code}`);
      return response.data;
    },
  };

  /**
   * Herbal Medicine APIs
   */
  herbs = {
    /**
     * Search herbal medicine database
     */
    search: async (
      query: string,
      options?: {
        property?: string;
        meridian?: string;
        limit?: number;
      }
    ): Promise<Types.HerbalMedicine[]> => {
      const response = await this.client.get('/herbs/search', {
        params: { query, ...options },
      });
      return response.data.results;
    },

    /**
     * Get herb details by ID
     */
    getById: async (herbId: string): Promise<Types.HerbalMedicine> => {
      const response = await this.client.get(`/herbs/${herbId}`);
      return response.data;
    },

    /**
     * Get formula recommendations based on pattern
     */
    recommendFormula: async (request: {
      pattern: string;
      constitution?: string;
      current_medications?: string[];
      allergies?: string[];
      pregnancy_status?: boolean;
    }): Promise<{ recommendations: Types.FormulaRecommendation[] }> => {
      const response = await this.client.post('/herbs/formula/recommend', request);
      return response.data;
    },

    /**
     * Check drug-herb interactions
     */
    checkInteractions: async (request: {
      herbs: string[];
      medications: string[];
    }): Promise<Types.InteractionCheckResponse> => {
      const response = await this.client.post('/herbs/interactions', request);
      return response.data;
    },

    /**
     * Get formula by ID
     */
    getFormula: async (formulaId: string): Promise<Types.HerbalFormula> => {
      const response = await this.client.get(`/herbs/formula/${formulaId}`);
      return response.data;
    },
  };

  /**
   * Safety Reporting APIs
   */
  safety = {
    /**
     * Report adverse event
     */
    reportAdverseEvent: async (report: {
      patient_id: string;
      reporter_id: string;
      event_date: string;
      products: Array<{
        type: 'herbal_formula' | 'single_herb' | 'acupuncture';
        name: string;
        dose?: string;
        duration_days?: number;
      }>;
      reaction: {
        description: string;
        symptoms: string[];
        onset: string;
        outcome: string;
        severity: 'mild' | 'moderate' | 'severe' | 'life_threatening';
      };
      concomitant_medications?: string[];
    }): Promise<{ report_id: string; status: string }> => {
      const response = await this.client.post('/safety/report', report);
      return response.data;
    },
  };

  /**
   * FHIR Integration APIs
   */
  fhir = {
    /**
     * Convert pattern diagnosis to FHIR Condition resource
     */
    toCondition: (
      diagnosis: Types.PatternDiagnosis,
      patientReference: string
    ): Types.FHIRCondition => {
      return {
        resourceType: 'Condition',
        id: `tm-condition-${Date.now()}`,
        meta: {
          profile: ['http://wia.live/fhir/StructureDefinition/TMPatternDiagnosis'],
        },
        clinicalStatus: {
          coding: [
            {
              system: 'http://terminology.hl7.org/CodeSystem/condition-clinical',
              code: 'active',
            },
          ],
        },
        code: {
          coding: [
            {
              system: 'http://id.who.int/icd/entity',
              code: diagnosis.icd11_tm_code,
              display: diagnosis.pattern_name.en,
            },
          ],
          text: diagnosis.pattern_name.zh || diagnosis.pattern_name.en,
        },
        subject: { reference: patientReference },
        recordedDate: new Date().toISOString(),
      };
    },

    /**
     * Convert constitution score to FHIR Observation
     */
    toObservation: (
      constitution: Types.TCMConstitution,
      patientReference: string
    ): Types.FHIRObservation => {
      return {
        resourceType: 'Observation',
        id: `tm-constitution-${Date.now()}`,
        status: 'final',
        code: {
          coding: [
            {
              system: 'http://wia.live/fhir/CodeSystem/tm-observations',
              code: 'constitution-assessment',
              display: 'TCM Constitution Assessment',
            },
          ],
        },
        subject: { reference: patientReference },
        valueCodeableConcept: {
          coding: [
            {
              system: 'http://wia.live/fhir/CodeSystem/tcm-constitution',
              code: constitution.primary_type,
              display: constitution.primary_type.replace(/_/g, ' '),
            },
          ],
        },
        component: Object.entries(constitution.score).map(([type, score]) => ({
          code: {
            coding: [
              {
                system: 'http://wia.live/fhir/CodeSystem/tcm-constitution',
                code: type,
              },
            ],
          },
          valueQuantity: {
            value: score,
            unit: 'score',
          },
        })),
      };
    },
  };
}

// Default export
export default WIATraditionalMedicine;
