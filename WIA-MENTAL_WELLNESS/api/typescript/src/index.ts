/**
 * WIA-MENTAL_WELLNESS TypeScript SDK
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * Complete SDK for mental wellness platforms including mood tracking,
 * therapy management, assessments, crisis intervention, and mindfulness.
 */

import axios, { AxiosInstance, AxiosError } from "axios";
import * as CryptoJS from "crypto-js";
import {
  MentalWellnessConfig,
  APIResponse,
  PaginatedResponse,
  MoodRecord,
  MoodData,
  MoodAnalytics,
  AssessmentRecord,
  AssessmentType,
  AssessmentHistory,
  TherapySessionRecord,
  SessionDetails,
  SessionContent,
  SessionOutcomes,
  TherapyProgress,
  CrisisInterventionRecord,
  CrisisDetails,
  CrisisAssessment,
  MindfulnessSessionRecord,
  MindfulnessSession,
  MeditationExperience,
  MindfulnessProgress,
  WellnessReport,
  ReportType,
  UserProfile,
  ConsentRecord,
  SafetyPlan,
  BiometricCorrelation,
  WearableData,
  TreatmentPlan,
  UUID,
  ISO8601,
} from "./types";

/**
 * Main SDK Client for WIA Mental Wellness
 */
export class MentalWellnessClient {
  private client: AxiosInstance;
  private config: MentalWellnessConfig;

  constructor(config: MentalWellnessConfig) {
    this.config = {
      baseUrl: "https://api.wia-official.org/mental-wellness/v1",
      timeout: 30000,
      retryAttempts: 3,
      privacyLevel: "high",
      encryptionEnabled: true,
      ...config,
    };

    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        "Content-Type": "application/json",
        "X-API-Key": this.config.apiKey,
        "X-WIA-Version": "1.0",
        "X-Privacy-Level": this.config.privacyLevel,
      },
    });

    this.setupInterceptors();
  }

  private setupInterceptors(): void {
    // Request interceptor
    this.client.interceptors.request.use(
      (config) => {
        if (this.config.userId) {
          config.headers["X-User-ID"] = this.config.userId;
        }
        config.headers["X-Request-ID"] = this.generateUUID();
        return config;
      },
      (error) => Promise.reject(error)
    );

    // Response interceptor
    this.client.interceptors.response.use(
      (response) => response,
      async (error: AxiosError) => {
        if (error.response?.status === 401) {
          // Handle authentication error
          throw new Error("Authentication failed. Please check your API key.");
        }
        return Promise.reject(error);
      }
    );
  }

  // ============================================================================
  // Mood Tracking Methods
  // ============================================================================

  /**
   * Record a mood entry
   */
  async recordMood(
    moodData: MoodData,
    context?: any,
    physiological?: any
  ): Promise<APIResponse<MoodRecord>> {
    try {
      const response = await this.client.post("/mood/record", {
        mood: moodData,
        context,
        physiological,
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get mood history with optional filtering
   */
  async getMoodHistory(
    startDate?: ISO8601,
    endDate?: ISO8601,
    limit: number = 100,
    offset: number = 0
  ): Promise<PaginatedResponse<MoodRecord>> {
    try {
      const response = await this.client.get("/mood/history", {
        params: { startDate, endDate, limit, offset },
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get mood analytics for a specified period
   */
  async getMoodAnalytics(
    period: "day" | "week" | "month" | "year" = "week",
    timezone?: string
  ): Promise<APIResponse<MoodAnalytics>> {
    try {
      const response = await this.client.get("/mood/analytics", {
        params: { period, timezone },
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Delete a specific mood entry
   */
  async deleteMoodEntry(moodId: UUID): Promise<APIResponse<void>> {
    try {
      const response = await this.client.delete(`/mood/${moodId}`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Update an existing mood entry
   */
  async updateMoodEntry(
    moodId: UUID,
    updates: Partial<MoodData>
  ): Promise<APIResponse<MoodRecord>> {
    try {
      const response = await this.client.put(`/mood/${moodId}`, updates);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Assessment Methods
  // ============================================================================

  /**
   * Start a new assessment
   */
  async startAssessment(
    assessmentType: AssessmentType,
    reason?: string
  ): Promise<APIResponse<{ assessmentId: UUID; questions: any[] }>> {
    try {
      const response = await this.client.post("/assessment/start", {
        assessmentType,
        reason: reason || "routine",
        triggeredBy: "self",
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Submit an assessment response
   */
  async submitAssessmentResponse(
    assessmentId: UUID,
    questionId: string,
    response: number | string,
    timeTaken?: number
  ): Promise<APIResponse<any>> {
    try {
      const res = await this.client.post(
        `/assessment/${assessmentId}/response`,
        {
          questionId,
          response,
          timeTaken,
        }
      );
      return res.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Complete an assessment and get results
   */
  async completeAssessment(
    assessmentId: UUID
  ): Promise<APIResponse<AssessmentRecord>> {
    try {
      const response = await this.client.post(
        `/assessment/${assessmentId}/complete`
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get assessment history
   */
  async getAssessmentHistory(): Promise<APIResponse<AssessmentHistory>> {
    try {
      const response = await this.client.get("/assessment/history");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get specific assessment details
   */
  async getAssessment(
    assessmentId: UUID
  ): Promise<APIResponse<AssessmentRecord>> {
    try {
      const response = await this.client.get(`/assessment/${assessmentId}`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get recommended assessments based on current state
   */
  async getRecommendedAssessments(): Promise<
    APIResponse<{ assessments: AssessmentType[]; reasons: string[] }>
  > {
    try {
      const response = await this.client.get("/assessment/recommended");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Therapy Session Methods
  // ============================================================================

  /**
   * Schedule a therapy session
   */
  async scheduleSession(
    therapistId: string,
    sessionDetails: Partial<SessionDetails>
  ): Promise<
    APIResponse<{ sessionId: UUID; scheduledTime: ISO8601; meetingDetails: any }>
  > {
    try {
      const response = await this.client.post("/therapy/session/schedule", {
        therapistId,
        ...sessionDetails,
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Cancel a scheduled session
   */
  async cancelSession(
    sessionId: UUID,
    reason?: string
  ): Promise<APIResponse<void>> {
    try {
      const response = await this.client.post(
        `/therapy/session/${sessionId}/cancel`,
        { reason }
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Reschedule a session
   */
  async rescheduleSession(
    sessionId: UUID,
    newTime: ISO8601
  ): Promise<APIResponse<{ sessionId: UUID; scheduledTime: ISO8601 }>> {
    try {
      const response = await this.client.put(
        `/therapy/session/${sessionId}/reschedule`,
        { newTime }
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Record session notes (therapist only)
   */
  async recordSessionNotes(
    sessionId: UUID,
    notes: {
      subjective?: string;
      objective?: string;
      assessment?: string;
      plan?: string;
      topics?: string[];
      techniques?: string[];
      homework?: string[];
    }
  ): Promise<APIResponse<{ notesId: UUID }>> {
    try {
      const response = await this.client.post(
        `/therapy/session/${sessionId}/notes`,
        notes
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Submit session outcomes (post-session feedback)
   */
  async submitSessionOutcomes(
    sessionId: UUID,
    outcomes: SessionOutcomes
  ): Promise<APIResponse<void>> {
    try {
      const response = await this.client.post(
        `/therapy/session/${sessionId}/outcomes`,
        outcomes
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get session history
   */
  async getSessionHistory(): Promise<
    APIResponse<{ sessions: TherapySessionRecord[]; summary: TherapyProgress }>
  > {
    try {
      const response = await this.client.get("/therapy/session/history");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get specific session details
   */
  async getSession(
    sessionId: UUID
  ): Promise<APIResponse<TherapySessionRecord>> {
    try {
      const response = await this.client.get(`/therapy/session/${sessionId}`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get therapy progress summary
   */
  async getTherapyProgress(): Promise<APIResponse<TherapyProgress>> {
    try {
      const response = await this.client.get("/therapy/progress");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Find therapists based on criteria
   */
  async findTherapists(criteria: {
    specialty?: string;
    location?: { latitude: number; longitude: number; radius: number };
    insurance?: string[];
    modality?: string;
    availability?: string;
  }): Promise<APIResponse<any[]>> {
    try {
      const response = await this.client.post("/therapy/therapists/search", criteria);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Crisis Support Methods
  // ============================================================================

  /**
   * Initiate crisis contact (PRIORITY - IMMEDIATE ROUTING)
   */
  async initiateCrisisContact(
    crisisDetails: {
      severity: "low" | "medium" | "high" | "imminent";
      type: string;
      message: string;
      location?: { latitude: number; longitude: number; consent: boolean };
    }
  ): Promise<
    APIResponse<{
      crisisId: UUID;
      resources: any;
      counselorAssigned?: any;
      responseTime: number;
    }>
  > {
    try {
      const response = await this.client.post("/crisis/contact", {
        ...crisisDetails,
        immediate: true,
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Update crisis status
   */
  async updateCrisisStatus(
    crisisId: UUID,
    status: string,
    assessment?: Partial<CrisisAssessment>
  ): Promise<APIResponse<{ crisisId: UUID; followUp: any }>> {
    try {
      const response = await this.client.put(`/crisis/${crisisId}/status`, {
        status,
        assessment,
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Create or update safety plan
   */
  async createSafetyPlan(
    safetyPlan: SafetyPlan
  ): Promise<APIResponse<{ planId: UUID }>> {
    try {
      const response = await this.client.post("/crisis/safety-plan", safetyPlan);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get current safety plan
   */
  async getSafetyPlan(): Promise<APIResponse<SafetyPlan>> {
    try {
      const response = await this.client.get("/crisis/safety-plan");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get crisis resources (hotlines, emergency contacts)
   */
  async getCrisisResources(
    location?: { latitude: number; longitude: number }
  ): Promise<APIResponse<any>> {
    try {
      const response = await this.client.get("/crisis/resources", {
        params: location,
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Mindfulness & Meditation Methods
  // ============================================================================

  /**
   * Start a meditation session
   */
  async startMeditationSession(
    sessionDetails: MindfulnessSession
  ): Promise<
    APIResponse<{
      sessionId: UUID;
      startedAt: ISO8601;
      audio?: { url: string; duration: number };
    }>
  > {
    try {
      const response = await this.client.post(
        "/mindfulness/session/start",
        sessionDetails
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Complete a meditation session
   */
  async completeMeditationSession(
    sessionId: UUID,
    experience: MeditationExperience,
    biometrics?: any
  ): Promise<
    APIResponse<{
      sessionId: UUID;
      analysis: any;
      streak: any;
      recommendations: string[];
    }>
  > {
    try {
      const response = await this.client.post(
        `/mindfulness/session/${sessionId}/complete`,
        {
          experience,
          biometrics,
        }
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get meditation session history
   */
  async getMeditationHistory(): Promise<
    APIResponse<{
      sessions: MindfulnessSessionRecord[];
      progress: MindfulnessProgress;
    }>
  > {
    try {
      const response = await this.client.get("/mindfulness/history");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get recommended meditation based on current mood/state
   */
  async getRecommendedMeditation(): Promise<
    APIResponse<{
      type: string;
      duration: number;
      reason: string;
      audioUrl?: string;
    }>
  > {
    try {
      const response = await this.client.get("/mindfulness/recommended");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get mindfulness progress statistics
   */
  async getMindfulnessProgress(): Promise<APIResponse<MindfulnessProgress>> {
    try {
      const response = await this.client.get("/mindfulness/progress");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Reports & Analytics Methods
  // ============================================================================

  /**
   * Generate wellness report
   */
  async generateReport(
    reportType: ReportType,
    dateRange: { start: ISO8601; end: ISO8601 },
    options?: { includeRecommendations?: boolean; shareWith?: string[] }
  ): Promise<
    APIResponse<{
      reportId: UUID;
      summary: any;
      downloadUrl: string;
      expiresAt: ISO8601;
    }>
  > {
    try {
      const response = await this.client.post("/reports/generate", {
        reportType,
        dateRange,
        ...options,
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get existing report
   */
  async getReport(reportId: UUID): Promise<APIResponse<WellnessReport>> {
    try {
      const response = await this.client.get(`/reports/${reportId}`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * List all reports
   */
  async listReports(): Promise<APIResponse<WellnessReport[]>> {
    try {
      const response = await this.client.get("/reports");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Delete a report
   */
  async deleteReport(reportId: UUID): Promise<APIResponse<void>> {
    try {
      const response = await this.client.delete(`/reports/${reportId}`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Biometric Integration Methods
  // ============================================================================

  /**
   * Sync wearable device data
   */
  async syncWearableData(
    deviceType: string,
    data: WearableData[]
  ): Promise<APIResponse<{ synced: number; processed: number }>> {
    try {
      const response = await this.client.post("/biometrics/sync", {
        deviceType,
        data,
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get biometric correlations with mood
   */
  async getBiometricCorrelations(): Promise<
    APIResponse<BiometricCorrelation>
  > {
    try {
      const response = await this.client.get("/biometrics/correlations");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // User Profile & Settings Methods
  // ============================================================================

  /**
   * Get user profile
   */
  async getUserProfile(): Promise<APIResponse<UserProfile>> {
    try {
      const response = await this.client.get("/user/profile");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Update user profile
   */
  async updateUserProfile(
    updates: Partial<UserProfile>
  ): Promise<APIResponse<UserProfile>> {
    try {
      const response = await this.client.put("/user/profile", updates);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Manage consent
   */
  async updateConsent(
    consentType: string,
    granted: boolean,
    scope?: string[]
  ): Promise<APIResponse<ConsentRecord>> {
    try {
      const response = await this.client.post("/user/consent", {
        consentType,
        granted,
        scope,
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get all consent records
   */
  async getConsents(): Promise<APIResponse<ConsentRecord[]>> {
    try {
      const response = await this.client.get("/user/consent");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Export all user data (GDPR right to data portability)
   */
  async exportUserData(
    format: "json" | "csv" = "json"
  ): Promise<APIResponse<{ downloadUrl: string; expiresAt: ISO8601 }>> {
    try {
      const response = await this.client.post("/user/export", { format });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Delete all user data (GDPR right to erasure)
   */
  async deleteUserData(confirmation: string): Promise<APIResponse<void>> {
    try {
      const response = await this.client.post("/user/delete", { confirmation });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Treatment Plan Methods
  // ============================================================================

  /**
   * Get current treatment plan
   */
  async getTreatmentPlan(): Promise<APIResponse<TreatmentPlan>> {
    try {
      const response = await this.client.get("/treatment/plan");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Update treatment plan (therapist only)
   */
  async updateTreatmentPlan(
    updates: Partial<TreatmentPlan>
  ): Promise<APIResponse<TreatmentPlan>> {
    try {
      const response = await this.client.put("/treatment/plan", updates);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Generate UUID v4
   */
  private generateUUID(): string {
    return "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx".replace(/[xy]/g, (c) => {
      const r = (Math.random() * 16) | 0;
      const v = c === "x" ? r : (r & 0x3) | 0x8;
      return v.toString(16);
    });
  }

  /**
   * Encrypt sensitive data
   */
  encrypt(data: string, key?: string): string {
    const encryptionKey = key || this.config.apiKey;
    return CryptoJS.AES.encrypt(data, encryptionKey).toString();
  }

  /**
   * Decrypt sensitive data
   */
  decrypt(encryptedData: string, key?: string): string {
    const encryptionKey = key || this.config.apiKey;
    const bytes = CryptoJS.AES.decrypt(encryptedData, encryptionKey);
    return bytes.toString(CryptoJS.enc.Utf8);
  }

  /**
   * Handle API errors
   */
  private handleError(error: any): Error {
    if (axios.isAxiosError(error)) {
      const message =
        error.response?.data?.error?.message || error.message || "Unknown error";
      return new Error(`API Error: ${message}`);
    }
    return error;
  }

  /**
   * Health check
   */
  async healthCheck(): Promise<APIResponse<{ status: string; version: string }>> {
    try {
      const response = await this.client.get("/health");
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }
}

/**
 * Factory function to create a new client instance
 */
export function createClient(config: MentalWellnessConfig): MentalWellnessClient {
  return new MentalWellnessClient(config);
}

/**
 * Export all types
 */
export * from "./types";

/**
 * Default export
 */
export default {
  MentalWellnessClient,
  createClient,
};
