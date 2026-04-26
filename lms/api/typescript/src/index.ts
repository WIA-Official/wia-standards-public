/**
 * WIA-EDU-009: Learning Management System Standard
 * TypeScript SDK
 *
 * @package @wia/lms
 * @version 1.0.0
 * @license MIT
 *
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

/**
 * Main LMS SDK Class
 */
export class LMS {
  private client: AxiosInstance;
  private config: Types.LMSConfig;

  constructor(config: Types.LMSConfig) {
    this.config = config;
    this.client = axios.create({
      baseURL: config.apiEndpoint,
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
      },
    });
  }

  // ============================================================================
  // Course Management
  // ============================================================================

  /**
   * Create a new course
   */
  async createCourse(course: Partial<Types.Course>): Promise<Types.Course> {
    const response = await this.client.post<Types.ApiResponse<Types.Course>>('/courses', course);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to create course');
    }
    return response.data.data;
  }

  /**
   * Get course by ID
   */
  async getCourse(courseId: string): Promise<Types.Course> {
    const response = await this.client.get<Types.ApiResponse<Types.Course>>(`/courses/${courseId}`);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Course not found');
    }
    return response.data.data;
  }

  /**
   * Update course
   */
  async updateCourse(courseId: string, updates: Partial<Types.Course>): Promise<Types.Course> {
    const response = await this.client.put<Types.ApiResponse<Types.Course>>(`/courses/${courseId}`, updates);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to update course');
    }
    return response.data.data;
  }

  /**
   * Delete course
   */
  async deleteCourse(courseId: string): Promise<void> {
    await this.client.delete(`/courses/${courseId}`);
  }

  /**
   * List all courses
   */
  async listCourses(params?: { page?: number; perPage?: number }): Promise<Types.ApiResponse<Types.Course[]>> {
    const response = await this.client.get<Types.ApiResponse<Types.Course[]>>('/courses', { params });
    return response.data;
  }

  // ============================================================================
  // Enrollment Management
  // ============================================================================

  /**
   * Enroll students in a course
   */
  async enrollStudents(courseId: string, userIds: string[], role: Types.UserRole = 'student'): Promise<Types.Enrollment[]> {
    const response = await this.client.post<Types.ApiResponse<Types.Enrollment[]>>(`/courses/${courseId}/enrollments`, {
      userIds,
      role,
    });
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to enroll students');
    }
    return response.data.data;
  }

  /**
   * Get course enrollments
   */
  async getCourseEnrollments(courseId: string): Promise<Types.Enrollment[]> {
    const response = await this.client.get<Types.ApiResponse<Types.Enrollment[]>>(`/courses/${courseId}/enrollments`);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get enrollments');
    }
    return response.data.data;
  }

  /**
   * Drop student from course
   */
  async dropEnrollment(enrollmentId: string): Promise<void> {
    await this.client.delete(`/enrollments/${enrollmentId}`);
  }

  // ============================================================================
  // Assignment Management
  // ============================================================================

  /**
   * Create assignment
   */
  async createAssignment(assignment: Partial<Types.Assignment>): Promise<Types.Assignment> {
    const response = await this.client.post<Types.ApiResponse<Types.Assignment>>('/assignments', assignment);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to create assignment');
    }
    return response.data.data;
  }

  /**
   * Get assignment by ID
   */
  async getAssignment(assignmentId: string): Promise<Types.Assignment> {
    const response = await this.client.get<Types.ApiResponse<Types.Assignment>>(`/assignments/${assignmentId}`);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Assignment not found');
    }
    return response.data.data;
  }

  /**
   * Submit assignment
   */
  async submitAssignment(assignmentId: string, submission: Partial<Types.Submission>): Promise<Types.Submission> {
    const response = await this.client.post<Types.ApiResponse<Types.Submission>>(
      `/assignments/${assignmentId}/submissions`,
      submission
    );
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to submit assignment');
    }
    return response.data.data;
  }

  /**
   * Grade submission
   */
  async gradeSubmission(submissionId: string, grade: { score: number; feedback?: string }): Promise<Types.Submission> {
    const response = await this.client.put<Types.ApiResponse<Types.Submission>>(`/submissions/${submissionId}/grade`, grade);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to grade submission');
    }
    return response.data.data;
  }

  // ============================================================================
  // Quiz Management
  // ============================================================================

  /**
   * Create quiz
   */
  async createQuiz(quiz: Partial<Types.Quiz>): Promise<Types.Quiz> {
    const response = await this.client.post<Types.ApiResponse<Types.Quiz>>('/quizzes', quiz);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to create quiz');
    }
    return response.data.data;
  }

  /**
   * Start quiz attempt
   */
  async startQuiz(quizId: string): Promise<Types.QuizSubmission> {
    const response = await this.client.post<Types.ApiResponse<Types.QuizSubmission>>(`/quizzes/${quizId}/attempts`);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to start quiz');
    }
    return response.data.data;
  }

  /**
   * Submit quiz answers
   */
  async submitQuiz(attemptId: string, answers: Types.QuestionAnswer[]): Promise<Types.QuizSubmission> {
    const response = await this.client.post<Types.ApiResponse<Types.QuizSubmission>>(
      `/quiz-attempts/${attemptId}/submit`,
      { answers }
    );
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to submit quiz');
    }
    return response.data.data;
  }

  // ============================================================================
  // Discussion Management
  // ============================================================================

  /**
   * Create discussion topic
   */
  async createDiscussion(discussion: Partial<Types.Discussion>): Promise<Types.Discussion> {
    const response = await this.client.post<Types.ApiResponse<Types.Discussion>>('/discussions', discussion);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to create discussion');
    }
    return response.data.data;
  }

  /**
   * Post to discussion
   */
  async postToDiscussion(discussionId: string, post: Partial<Types.DiscussionPost>): Promise<Types.DiscussionPost> {
    const response = await this.client.post<Types.ApiResponse<Types.DiscussionPost>>(
      `/discussions/${discussionId}/posts`,
      post
    );
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to post to discussion');
    }
    return response.data.data;
  }

  /**
   * Get discussion posts
   */
  async getDiscussionPosts(discussionId: string): Promise<Types.DiscussionPost[]> {
    const response = await this.client.get<Types.ApiResponse<Types.DiscussionPost[]>>(`/discussions/${discussionId}/posts`);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get posts');
    }
    return response.data.data;
  }

  // ============================================================================
  // Grading & Gradebook
  // ============================================================================

  /**
   * Get course gradebook
   */
  async getGradebook(courseId: string): Promise<Types.Gradebook> {
    const response = await this.client.get<Types.ApiResponse<Types.Gradebook>>(`/courses/${courseId}/gradebook`);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get gradebook');
    }
    return response.data.data;
  }

  /**
   * Update student grade
   */
  async updateGrade(courseId: string, userId: string, assignmentId: string, grade: Partial<Types.AssignmentGrade>): Promise<Types.AssignmentGrade> {
    const response = await this.client.put<Types.ApiResponse<Types.AssignmentGrade>>(
      `/courses/${courseId}/students/${userId}/assignments/${assignmentId}/grade`,
      grade
    );
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to update grade');
    }
    return response.data.data;
  }

  // ============================================================================
  // Analytics
  // ============================================================================

  /**
   * Get course analytics
   */
  async getCourseAnalytics(courseId: string): Promise<Types.CourseAnalytics> {
    const response = await this.client.get<Types.ApiResponse<Types.CourseAnalytics>>(`/courses/${courseId}/analytics`);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get analytics');
    }
    return response.data.data;
  }

  /**
   * Get student analytics
   */
  async getStudentAnalytics(courseId: string, userId: string): Promise<Types.StudentAnalytics> {
    const response = await this.client.get<Types.ApiResponse<Types.StudentAnalytics>>(
      `/courses/${courseId}/students/${userId}/analytics`
    );
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get student analytics');
    }
    return response.data.data;
  }

  // ============================================================================
  // Notifications
  // ============================================================================

  /**
   * Send notification
   */
  async sendNotification(notification: Partial<Types.Notification>): Promise<Types.Notification> {
    const response = await this.client.post<Types.ApiResponse<Types.Notification>>('/notifications', notification);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to send notification');
    }
    return response.data.data;
  }

  /**
   * Get user notifications
   */
  async getUserNotifications(userId: string, unreadOnly: boolean = false): Promise<Types.Notification[]> {
    const response = await this.client.get<Types.ApiResponse<Types.Notification[]>>(`/users/${userId}/notifications`, {
      params: { unreadOnly },
    });
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get notifications');
    }
    return response.data.data;
  }

  /**
   * Mark notification as read
   */
  async markNotificationRead(notificationId: string): Promise<void> {
    await this.client.put(`/notifications/${notificationId}/read`);
  }

  // ============================================================================
  // Module Management
  // ============================================================================

  /**
   * Create module
   */
  async createModule(module: Partial<Types.Module>): Promise<Types.Module> {
    const response = await this.client.post<Types.ApiResponse<Types.Module>>('/modules', module);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to create module');
    }
    return response.data.data;
  }

  /**
   * Get course modules
   */
  async getCourseModules(courseId: string): Promise<Types.Module[]> {
    const response = await this.client.get<Types.ApiResponse<Types.Module[]>>(`/courses/${courseId}/modules`);
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get modules');
    }
    return response.data.data;
  }
}

/**
 * Default export
 */
export default LMS;

/**
 * Philosophy
 */
export const PHILOSOPHY = '弘益人間 (홍익인간) · Benefit All Humanity';
