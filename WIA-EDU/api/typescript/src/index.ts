/**
 * WIA EDU Standard - TypeScript SDK
 * Educational Platform Standard
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIAEDU, InstitutionResponse, EducationalProgram, Course, Learner,
  Instructor, Assessment, Credential, LearningAnalytics, ValidationResult, PaginatedResponse,
} from './types';

export class WIAEDUClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createInstitution(edu: WIAEDU): Promise<InstitutionResponse> {
    const response = await this.axios.post<InstitutionResponse>('/institutions', edu);
    return response.data;
  }

  async getInstitution(id: string): Promise<WIAEDU> {
    const response = await this.axios.get<WIAEDU>(`/institutions/${id}`);
    return response.data;
  }

  async listInstitutions(params?: { type?: string; limit?: number }): Promise<PaginatedResponse<InstitutionResponse>> {
    const response = await this.axios.get<PaginatedResponse<InstitutionResponse>>('/institutions', { params });
    return response.data;
  }

  async updateInstitution(id: string, updates: Partial<WIAEDU>): Promise<InstitutionResponse> {
    const response = await this.axios.put<InstitutionResponse>(`/institutions/${id}`, updates);
    return response.data;
  }

  async createProgram(institutionId: string, program: Omit<EducationalProgram, 'id'>): Promise<EducationalProgram> {
    const response = await this.axios.post<EducationalProgram>(`/institutions/${institutionId}/programs`, program);
    return response.data;
  }

  async listPrograms(institutionId: string, params?: { type?: string; level?: string }): Promise<EducationalProgram[]> {
    const response = await this.axios.get<EducationalProgram[]>(`/institutions/${institutionId}/programs`, { params });
    return response.data;
  }

  async getProgram(institutionId: string, programId: string): Promise<EducationalProgram> {
    const response = await this.axios.get<EducationalProgram>(`/institutions/${institutionId}/programs/${programId}`);
    return response.data;
  }

  async createCourse(institutionId: string, course: Omit<Course, 'id'>): Promise<Course> {
    const response = await this.axios.post<Course>(`/institutions/${institutionId}/courses`, course);
    return response.data;
  }

  async listCourses(institutionId: string, params?: { status?: string; format?: string }): Promise<Course[]> {
    const response = await this.axios.get<Course[]>(`/institutions/${institutionId}/courses`, { params });
    return response.data;
  }

  async getCourse(institutionId: string, courseId: string): Promise<Course> {
    const response = await this.axios.get<Course>(`/institutions/${institutionId}/courses/${courseId}`);
    return response.data;
  }

  async enrollLearner(institutionId: string, courseId: string, learnerId: string): Promise<{ success: boolean; enrollmentId: string }> {
    const response = await this.axios.post(`/institutions/${institutionId}/courses/${courseId}/enrollments`, { learnerId });
    return response.data;
  }

  async registerLearner(institutionId: string, learner: Omit<Learner, 'id' | 'progress' | 'achievements'>): Promise<Learner> {
    const response = await this.axios.post<Learner>(`/institutions/${institutionId}/learners`, learner);
    return response.data;
  }

  async getLearner(institutionId: string, learnerId: string): Promise<Learner> {
    const response = await this.axios.get<Learner>(`/institutions/${institutionId}/learners/${learnerId}`);
    return response.data;
  }

  async updateLearnerProgress(institutionId: string, learnerId: string, progress: any): Promise<Learner> {
    const response = await this.axios.patch<Learner>(`/institutions/${institutionId}/learners/${learnerId}/progress`, progress);
    return response.data;
  }

  async addInstructor(institutionId: string, instructor: Omit<Instructor, 'id' | 'ratings'>): Promise<Instructor> {
    const response = await this.axios.post<Instructor>(`/institutions/${institutionId}/instructors`, instructor);
    return response.data;
  }

  async listInstructors(institutionId: string): Promise<Instructor[]> {
    const response = await this.axios.get<Instructor[]>(`/institutions/${institutionId}/instructors`);
    return response.data;
  }

  async assignInstructor(institutionId: string, courseId: string, instructorId: string): Promise<void> {
    await this.axios.post(`/institutions/${institutionId}/courses/${courseId}/instructors`, { instructorId });
  }

  async createAssessment(institutionId: string, courseId: string, assessment: Omit<Assessment, 'id' | 'status'>): Promise<Assessment> {
    const response = await this.axios.post<Assessment>(`/institutions/${institutionId}/courses/${courseId}/assessments`, assessment);
    return response.data;
  }

  async listAssessments(institutionId: string, courseId: string): Promise<Assessment[]> {
    const response = await this.axios.get<Assessment[]>(`/institutions/${institutionId}/courses/${courseId}/assessments`);
    return response.data;
  }

  async submitAssessment(institutionId: string, assessmentId: string, learnerId: string, submission: any): Promise<{ submissionId: string; status: string }> {
    const response = await this.axios.post(`/institutions/${institutionId}/assessments/${assessmentId}/submissions`, { learnerId, submission });
    return response.data;
  }

  async gradeSubmission(institutionId: string, submissionId: string, grade: { score: number; feedback?: string }): Promise<void> {
    await this.axios.post(`/institutions/${institutionId}/submissions/${submissionId}/grade`, grade);
  }

  async issueCredential(institutionId: string, credential: Omit<Credential, 'id' | 'issuedAt' | 'verification'>): Promise<Credential> {
    const response = await this.axios.post<Credential>(`/institutions/${institutionId}/credentials`, credential);
    return response.data;
  }

  async verifyCredential(credentialId: string): Promise<{ valid: boolean; credential?: Credential; issues?: string[] }> {
    const response = await this.axios.get(`/credentials/${credentialId}/verify`);
    return response.data;
  }

  async getLearningAnalytics(institutionId: string, params?: { courseId?: string; period?: string }): Promise<LearningAnalytics> {
    const response = await this.axios.get<LearningAnalytics>(`/institutions/${institutionId}/analytics`, { params });
    return response.data;
  }

  async getAtRiskLearners(institutionId: string): Promise<{ learners: string[]; interventions: any[] }> {
    const response = await this.axios.get(`/institutions/${institutionId}/analytics/at-risk`);
    return response.data;
  }

  validateEDU(edu: WIAEDU): ValidationResult {
    const errors: { path: string; message: string }[] = [];
    if (edu.standard !== 'WIA-EDU') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!edu.institution?.id) errors.push({ path: 'institution.id', message: 'Institution ID required' });
    if (!edu.institution?.name) errors.push({ path: 'institution.name', message: 'Institution name required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalInstitution(name: string, type: string = 'university'): WIAEDU {
  return {
    standard: 'WIA-EDU',
    version: '1.0.0',
    institution: {
      id: generateUUID(), name, type: type as any, accreditation: [], status: 'active', createdAt: new Date().toISOString(),
      location: { country: '', timezone: 'UTC' },
      contact: { email: 'contact@example.edu' },
    },
    programs: [],
    courses: [],
    learners: [],
    instructors: [],
    assessments: [],
    credentials: [],
    analytics: {
      engagement: { activeUsers: 0, averageSessionDuration: 0, contentInteractions: 0, discussionParticipation: 0 },
      performance: { averageGrade: 0, passRate: 0, completionRate: 0, outcomeAchievement: 0 },
      retention: { enrollmentRetention: 0, courseCompletion: 0, programCompletion: 0, churnRate: 0 },
      predictions: { atRiskLearners: [], successPredictions: [], recommendedInterventions: [] },
    },
  };
}

export default { WIAEDUClient, generateUUID, createMinimalInstitution };
