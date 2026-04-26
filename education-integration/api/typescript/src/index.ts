/**
 * WIA-UNI-010: Education Integration Standard
 * TypeScript SDK
 *
 * @package @wia/education-integration
 * @version 1.0.0
 * @license MIT
 *
 * Philosophy: 弘益人間 (홍익인간) · Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import type * as Types from './types';

// Re-export all types
export * from './types';

/**
 * Main SDK class for Education Integration
 */
export class EducationIntegration {
  private client: AxiosInstance;
  private config: Types.EducationIntegrationConfig;

  constructor(config: Types.EducationIntegrationConfig) {
    this.config = config;
    this.client = axios.create({
      baseURL: config.apiEndpoint,
      headers: {
        'Content-Type': 'application/json',
        ...(config.apiKey && { 'Authorization': `Bearer ${config.apiKey}` })
      }
    });
  }

  // =========================================================================
  // Virtual Classroom Methods
  // =========================================================================

  /**
   * Create a new virtual classroom
   */
  async createVirtualClassroom(
    params: Omit<Types.VirtualClassroom, 'id' | 'status' | 'createdAt'>
  ): Promise<Types.APIResponse<Types.VirtualClassroom>> {
    try {
      const response = await this.client.post('/classrooms', {
        ...params,
        status: 'planned',
        createdAt: new Date()
      });
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Get classroom by ID
   */
  async getClassroom(id: string): Promise<Types.APIResponse<Types.VirtualClassroom>> {
    try {
      const response = await this.client.get(`/classrooms/${id}`);
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Schedule a class session
   */
  async scheduleSession(
    classroomId: string,
    params: Omit<Types.ClassSession, 'id' | 'classroomId'>
  ): Promise<Types.APIResponse<Types.ClassSession>> {
    try {
      const response = await this.client.post(`/classrooms/${classroomId}/sessions`, params);
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Record attendance for a session
   */
  async recordAttendance(
    sessionId: string,
    attendance: Types.Attendance[]
  ): Promise<Types.APIResponse<void>> {
    try {
      await this.client.post(`/sessions/${sessionId}/attendance`, { attendance });
      return { success: true };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  // =========================================================================
  // Curriculum Methods
  // =========================================================================

  /**
   * Map curriculum between regions
   */
  async mapCurriculum(params: {
    subject: string;
    gradeRange: [number, number];
    includeVocational?: boolean;
  }): Promise<Types.APIResponse<Types.CurriculumMapping>> {
    try {
      const response = await this.client.post('/curriculum/map', params);
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Get curriculum mapping for a subject
   */
  async getCurriculumMapping(subjectId: string): Promise<Types.APIResponse<Types.CurriculumMapping>> {
    try {
      const response = await this.client.get(`/curriculum/mapping/${subjectId}`);
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Get unified learning objectives
   */
  async getUnifiedObjectives(
    subject: string,
    grade: number
  ): Promise<Types.APIResponse<Types.LearningObjective[]>> {
    try {
      const response = await this.client.get(`/curriculum/objectives`, {
        params: { subject, grade }
      });
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  // =========================================================================
  // Student Exchange Methods
  // =========================================================================

  /**
   * Create exchange program application
   */
  async createExchangeApplication(
    params: Omit<Types.ExchangeProgram, 'id' | 'status'>
  ): Promise<Types.APIResponse<Types.ExchangeProgram>> {
    try {
      const response = await this.client.post('/exchange/apply', {
        ...params,
        status: 'applied'
      });
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Get exchange program status
   */
  async getExchangeProgram(id: string): Promise<Types.APIResponse<Types.ExchangeProgram>> {
    try {
      const response = await this.client.get(`/exchange/${id}`);
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  // =========================================================================
  // Credential Methods
  // =========================================================================

  /**
   * Issue a credential
   */
  async issueCredential(params: {
    studentId?: string;
    teacherId?: string;
    program: string;
    level: string;
    recognitionScope: Types.Credential['recognitionScope'];
    useBlockchain?: boolean;
  }): Promise<Types.APIResponse<Types.Credential>> {
    try {
      const response = await this.client.post('/credentials/issue', params);
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Verify a credential
   */
  async verifyCredential(credentialId: string): Promise<Types.APIResponse<{
    valid: boolean;
    credential?: Types.Credential;
    verifiedAt: Date;
  }>> {
    try {
      const response = await this.client.get(`/credentials/verify/${credentialId}`);
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Get credential by ID
   */
  async getCredential(id: string): Promise<Types.APIResponse<Types.Credential>> {
    try {
      const response = await this.client.get(`/credentials/${id}`);
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  // =========================================================================
  // LMS Course Methods
  // =========================================================================

  /**
   * Create a new course
   */
  async createCourse(
    params: Omit<Types.Course, 'id' | 'enrolled'>
  ): Promise<Types.APIResponse<Types.Course>> {
    try {
      const response = await this.client.post('/lms/courses', {
        ...params,
        enrolled: []
      });
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Enroll student in course
   */
  async enrollStudent(
    courseId: string,
    studentId: string
  ): Promise<Types.APIResponse<void>> {
    try {
      await this.client.post(`/lms/courses/${courseId}/enroll`, { studentId });
      return { success: true };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Get course by ID
   */
  async getCourse(id: string): Promise<Types.APIResponse<Types.Course>> {
    try {
      const response = await this.client.get(`/lms/courses/${id}`);
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Submit assessment
   */
  async submitAssessment(params: {
    assessmentId: string;
    studentId: string;
    answers: any;
    submittedAt?: Date;
  }): Promise<Types.APIResponse<{
    score: number;
    passed: boolean;
    feedback?: string;
  }>> {
    try {
      const response = await this.client.post('/lms/assessments/submit', {
        ...params,
        submittedAt: params.submittedAt || new Date()
      });
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  // =========================================================================
  // Analytics Methods
  // =========================================================================

  /**
   * Get learning analytics for a student
   */
  async getStudentAnalytics(
    studentId: string,
    courseId?: string
  ): Promise<Types.APIResponse<Types.LearningAnalytics>> {
    try {
      const response = await this.client.get(`/analytics/student/${studentId}`, {
        params: { courseId }
      });
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Get program-wide metrics
   */
  async getProgramMetrics(params?: {
    region?: Types.Region;
    startDate?: Date;
    endDate?: Date;
  }): Promise<Types.APIResponse<Types.ProgramMetrics>> {
    try {
      const response = await this.client.get('/analytics/program', {
        params
      });
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  // =========================================================================
  // Teacher Methods
  // =========================================================================

  /**
   * Register teacher for integration program
   */
  async registerTeacher(
    teacher: Omit<Types.Teacher, 'id'>
  ): Promise<Types.APIResponse<Types.Teacher>> {
    try {
      const response = await this.client.post('/teachers/register', teacher);
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Get teacher certification status
   */
  async getTeacherCertification(teacherId: string): Promise<Types.APIResponse<{
    level: number;
    validUntil: Date;
    hoursCompleted: number;
    nextRenewal: Date;
  }>> {
    try {
      const response = await this.client.get(`/teachers/${teacherId}/certification`);
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  // =========================================================================
  // Institution Methods
  // =========================================================================

  /**
   * Get institution details
   */
  async getInstitution(id: string): Promise<Types.APIResponse<Types.Institution>> {
    try {
      const response = await this.client.get(`/institutions/${id}`);
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  /**
   * Search institutions
   */
  async searchInstitutions(params: {
    region?: Types.Region;
    type?: Types.Institution['type'];
    accredited?: boolean;
  }): Promise<Types.APIResponse<Types.Institution[]>> {
    try {
      const response = await this.client.get('/institutions/search', {
        params
      });
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }

  // =========================================================================
  // Utility Methods
  // =========================================================================

  /**
   * Handle API errors
   */
  private handleError(error: any): Types.APIResponse<never> {
    return {
      success: false,
      error: {
        code: error.response?.status?.toString() || 'UNKNOWN_ERROR',
        message: error.response?.data?.message || error.message,
        details: error.response?.data
      }
    };
  }

  /**
   * Get API health status
   */
  async healthCheck(): Promise<Types.APIResponse<{
    status: 'healthy' | 'degraded' | 'down';
    version: string;
    uptime: number;
  }>> {
    try {
      const response = await this.client.get('/health');
      return {
        success: true,
        data: response.data
      };
    } catch (error: any) {
      return this.handleError(error);
    }
  }
}

/**
 * Factory function to create SDK instance
 */
export function createEducationIntegration(
  config: Types.EducationIntegrationConfig
): EducationIntegration {
  return new EducationIntegration(config);
}

/**
 * Default export
 */
export default EducationIntegration;

/**
 * Version information
 */
export const VERSION = '1.0.0';
export const STANDARD = 'WIA-UNI-010';
export const PHILOSOPHY = '弘益人間 (홍익인간) · Benefit All Humanity';
