/**
 * WIA-EDU-002: E-Learning Standard SDK
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * @example
 * ```typescript
 * import { ELearning, CourseType } from '@wia/elearning-sdk';
 *
 * const platform = new ELearning({
 *   apiKey: 'your-api-key',
 *   environment: 'production'
 * });
 *
 * const course = await platform.courses.create({
 *   title: 'Introduction to Web Development',
 *   category: 'Technology',
 *   difficulty: 'Beginner'
 * });
 * ```
 */

import axios, { AxiosInstance } from 'axios';
import { v4 as uuidv4 } from 'uuid';

import {
  Course,
  Student,
  Enrollment,
  Certificate,
  LiveSession,
  Analytics,
  xAPIStatement,
  ELearningConfig,
  CourseType,
  CourseCategory,
  CourseDifficulty,
  ContentFormat,
  EnrollmentStatus,
  Progress
} from './types';

export * from './types';

export class ELearning {
  private client: AxiosInstance;
  private config: ELearningConfig;

  public courses: CourseManager;
  public students: StudentManager;
  public enrollments: EnrollmentManager;
  public live: LiveSessionManager;
  public analytics: AnalyticsManager;
  public xapi: xAPIManager;

  constructor(config: ELearningConfig) {
    this.config = config;

    this.client = axios.create({
      baseURL: config.baseUrl || 'https://api.wia.org/elearning/v1',
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'EDU-002',
        'X-WIA-Philosophy': '弘益人間'
      }
    });

    this.courses = new CourseManager(this.client);
    this.students = new StudentManager(this.client);
    this.enrollments = new EnrollmentManager(this.client);
    this.live = new LiveSessionManager(this.client);
    this.analytics = new AnalyticsManager(this.client);
    this.xapi = new xAPIManager(this.client);
  }
}

export class CourseManager {
  constructor(private client: AxiosInstance) {}

  async create(data: Partial<Course>): Promise<Course> {
    const response = await this.client.post<Course>('/courses', {
      id: uuidv4(),
      ...data,
      createdAt: new Date(),
      updatedAt: new Date()
    });
    return response.data;
  }

  async get(id: string): Promise<Course> {
    const response = await this.client.get<Course>(`/courses/${id}`);
    return response.data;
  }

  async list(filters?: {
    category?: CourseCategory;
    difficulty?: CourseDifficulty;
    type?: CourseType;
  }): Promise<Course[]> {
    const response = await this.client.get<Course[]>('/courses', { params: filters });
    return response.data;
  }

  async update(id: string, data: Partial<Course>): Promise<Course> {
    const response = await this.client.patch<Course>(`/courses/${id}`, {
      ...data,
      updatedAt: new Date()
    });
    return response.data;
  }

  async delete(id: string): Promise<void> {
    await this.client.delete(`/courses/${id}`);
  }

  async addLesson(courseId: string, lesson: any): Promise<void> {
    await this.client.post(`/courses/${courseId}/lessons`, lesson);
  }
}

export class StudentManager {
  constructor(private client: AxiosInstance) {}

  async create(data: Partial<Student>): Promise<Student> {
    const response = await this.client.post<Student>('/students', {
      id: uuidv4(),
      ...data,
      createdAt: new Date()
    });
    return response.data;
  }

  async get(id: string): Promise<Student> {
    const response = await this.client.get<Student>(`/students/${id}`);
    return response.data;
  }

  async getProgress(studentId: string, courseId: string): Promise<Progress> {
    const response = await this.client.get<Progress>(
      `/students/${studentId}/courses/${courseId}/progress`
    );
    return response.data;
  }
}

export class EnrollmentManager {
  constructor(private client: AxiosInstance) {}

  async enroll(studentId: string, courseId: string): Promise<Enrollment> {
    const response = await this.client.post<Enrollment>('/enrollments', {
      id: uuidv4(),
      studentId,
      courseId,
      status: EnrollmentStatus.Active,
      enrolledAt: new Date(),
      progress: {
        percentage: 0,
        lessonsCompleted: 0,
        totalLessons: 0,
        quizScores: [],
        timeSpent: 0
      }
    });
    return response.data;
  }

  async complete(enrollmentId: string): Promise<Enrollment> {
    const response = await this.client.patch<Enrollment>(`/enrollments/${enrollmentId}`, {
      status: EnrollmentStatus.Completed,
      completedAt: new Date()
    });
    return response.data;
  }

  async issueCertificate(enrollmentId: string): Promise<Certificate> {
    const response = await this.client.post<Certificate>(
      `/enrollments/${enrollmentId}/certificate`
    );
    return response.data;
  }
}

export class LiveSessionManager {
  constructor(private client: AxiosInstance) {}

  async create(data: Partial<LiveSession>): Promise<LiveSession> {
    const response = await this.client.post<LiveSession>('/live-sessions', {
      id: uuidv4(),
      ...data,
      joinUrl: `https://live.wia.org/session/${uuidv4()}`
    });
    return response.data;
  }

  async join(sessionId: string, userId: string, role: 'instructor' | 'student'): Promise<string> {
    const response = await this.client.post<{ joinUrl: string }>(
      `/live-sessions/${sessionId}/join`,
      { userId, role }
    );
    return response.data.joinUrl;
  }

  async enableWhiteboard(sessionId: string): Promise<void> {
    await this.client.patch(`/live-sessions/${sessionId}`, {
      'features.whiteboard': true
    });
  }

  async enableScreenSharing(sessionId: string): Promise<void> {
    await this.client.patch(`/live-sessions/${sessionId}`, {
      'features.screenShare': true
    });
  }

  async enableChat(sessionId: string): Promise<void> {
    await this.client.patch(`/live-sessions/${sessionId}`, {
      'features.chat': true
    });
  }
}

export class AnalyticsManager {
  constructor(private client: AxiosInstance) {}

  async getCourseAnalytics(courseId: string): Promise<Analytics> {
    const response = await this.client.get<Analytics>(`/analytics/courses/${courseId}`);
    return response.data;
  }

  async getStudentAnalytics(studentId: string, courseId?: string): Promise<Analytics> {
    const url = courseId
      ? `/analytics/students/${studentId}/courses/${courseId}`
      : `/analytics/students/${studentId}`;

    const response = await this.client.get<Analytics>(url);
    return response.data;
  }
}

export class xAPIManager {
  constructor(private client: AxiosInstance) {}

  async sendStatement(statement: xAPIStatement): Promise<void> {
    await this.client.post('/xapi/statements', {
      ...statement,
      timestamp: statement.timestamp || new Date()
    });
  }

  async getStatements(filters: {
    actor?: string;
    verb?: string;
    since?: Date;
    until?: Date;
  }): Promise<xAPIStatement[]> {
    const response = await this.client.get<xAPIStatement[]>('/xapi/statements', {
      params: filters
    });
    return response.data;
  }
}

export default ELearning;
