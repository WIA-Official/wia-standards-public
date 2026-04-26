/**
 * WIA-EDU-005: Educational AI Standard - TypeScript SDK
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

export class EducationalAI {
  private client: AxiosInstance;
  private config: Types.EducationalAIConfig;

  constructor(config: Types.EducationalAIConfig) {
    this.config = {
      baseUrl: 'https://api.wia.org/edu/v1',
      timeout: 30000,
      retries: 3,
      ...config,
    };

    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
      },
    });
  }

  // ===== Student Management =====

  async getStudentProfile(studentId: string): Promise<Types.StudentProfile> {
    const response = await this.client.get<Types.APIResponse<Types.StudentProfile>>(
      `/students/${studentId}`
    );
    return this.handleResponse(response.data);
  }

  async updateStudentProfile(
    studentId: string,
    updates: Partial<Types.StudentProfile>
  ): Promise<Types.StudentProfile> {
    const response = await this.client.patch<Types.APIResponse<Types.StudentProfile>>(
      `/students/${studentId}`,
      updates
    );
    return this.handleResponse(response.data);
  }

  // ===== AI Tutoring =====

  async createTutorSession(config: Types.TutorConfig & { studentId: string }): Promise<Types.TutorSession> {
    const response = await this.client.post<Types.APIResponse<Types.TutorSession>>(
      '/tutor/sessions',
      config
    );
    return this.handleResponse(response.data);
  }

  async askTutor(
    sessionId: string,
    question: string
  ): Promise<Types.TutorResponse> {
    const response = await this.client.post<Types.APIResponse<Types.TutorResponse>>(
      `/tutor/sessions/${sessionId}/ask`,
      { question }
    );
    return this.handleResponse(response.data);
  }

  async getTutorSession(sessionId: string): Promise<Types.TutorSession> {
    const response = await this.client.get<Types.APIResponse<Types.TutorSession>>(
      `/tutor/sessions/${sessionId}`
    );
    return this.handleResponse(response.data);
  }

  async endTutorSession(sessionId: string): Promise<void> {
    await this.client.post(`/tutor/sessions/${sessionId}/end`);
  }

  // ===== Automated Grading =====

  async submitAssignment(request: Types.GradingRequest): Promise<string> {
    const response = await this.client.post<Types.APIResponse<{ submissionId: string }>>(
      '/grading/submit',
      request
    );
    const data = this.handleResponse(response.data);
    return data.submissionId;
  }

  async gradeAssignment(request: Types.GradingRequest): Promise<Types.GradingResponse> {
    const response = await this.client.post<Types.APIResponse<Types.GradingResponse>>(
      '/grading/grade',
      request
    );
    return this.handleResponse(response.data);
  }

  async getGradingResult(submissionId: string): Promise<Types.GradingResponse> {
    const response = await this.client.get<Types.APIResponse<Types.GradingResponse>>(
      `/grading/results/${submissionId}`
    );
    return this.handleResponse(response.data);
  }

  // ===== Personalization =====

  async getRecommendations(request: Types.RecommendationRequest): Promise<Types.RecommendationResponse> {
    const response = await this.client.post<Types.APIResponse<Types.RecommendationResponse>>(
      '/recommendations',
      request
    );
    return this.handleResponse(response.data);
  }

  async getLearningPath(
    studentId: string,
    goal: string
  ): Promise<Types.Topic[]> {
    const response = await this.client.get<Types.APIResponse<Types.Topic[]>>(
      `/learning-paths/${studentId}`,
      { params: { goal } }
    );
    return this.handleResponse(response.data);
  }

  // ===== Content Generation =====

  async generateContent(request: Types.ContentGenerationRequest): Promise<Types.GeneratedContent> {
    const response = await this.client.post<Types.APIResponse<Types.GeneratedContent>>(
      '/content/generate',
      request
    );
    return this.handleResponse(response.data);
  }

  async generatePracticeProblems(
    topic: string,
    difficulty: Types.Difficulty,
    count: number = 10
  ): Promise<Types.ContentItem[]> {
    const result = await this.generateContent({
      type: 'problem',
      topic,
      difficulty,
      count,
    });
    return result.items;
  }

  async generateQuiz(
    topic: string,
    difficulty: Types.Difficulty,
    questionCount: number = 10
  ): Promise<Types.ContentItem[]> {
    const result = await this.generateContent({
      type: 'quiz',
      topic,
      difficulty,
      count: questionCount,
    });
    return result.items;
  }

  // ===== Learning Analytics =====

  async getAnalytics(query: Types.AnalyticsQuery): Promise<Types.AnalyticsResponse> {
    const response = await this.client.post<Types.APIResponse<Types.AnalyticsResponse>>(
      '/analytics/query',
      query
    );
    return this.handleResponse(response.data);
  }

  async getStudentProgress(
    studentId: string,
    timeRange?: Types.DateRange
  ): Promise<Types.AnalyticsResponse> {
    return this.getAnalytics({
      studentId,
      timeRange: timeRange || this.getDefaultTimeRange(),
      metrics: ['mastery', 'engagement', 'progress-rate'],
    });
  }

  async getClassAnalytics(
    classId: string,
    timeRange?: Types.DateRange
  ): Promise<Types.AnalyticsResponse> {
    return this.getAnalytics({
      classId,
      timeRange: timeRange || this.getDefaultTimeRange(),
      metrics: ['performance', 'engagement', 'completion-rate'],
    });
  }

  async identifyAtRiskStudents(classId: string): Promise<Types.Prediction[]> {
    const analytics = await this.getClassAnalytics(classId);
    return analytics.predictions.filter(p => p.type === 'at-risk');
  }

  // ===== Helper Methods =====

  private handleResponse<T>(apiResponse: Types.APIResponse<T>): T {
    if (!apiResponse.success || !apiResponse.data) {
      throw new Error(
        apiResponse.error?.message || 'API request failed'
      );
    }
    return apiResponse.data;
  }

  private getDefaultTimeRange(): Types.DateRange {
    const end = new Date();
    const start = new Date();
    start.setDate(start.getDate() - 30); // Last 30 days
    return { start, end };
  }

  // ===== Utility Methods =====

  async healthCheck(): Promise<boolean> {
    try {
      await this.client.get('/health');
      return true;
    } catch {
      return false;
    }
  }

  setApiKey(apiKey: string): void {
    this.config.apiKey = apiKey;
    this.client.defaults.headers['Authorization'] = `Bearer ${apiKey}`;
  }
}

// ===== Convenience Functions =====

export function createClient(config: Types.EducationalAIConfig): EducationalAI {
  return new EducationalAI(config);
}

export default EducationalAI;
