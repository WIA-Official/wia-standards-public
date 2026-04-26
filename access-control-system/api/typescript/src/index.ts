/**
 * WIA-CITY-015: Access Control System Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  UserInfo,
  Credential,
  AccessCard,
  BiometricCredential,
  PINCredential,
  AccessPoint,
  AccessZone,
  AccessLog,
  Visitor,
  VehicleInfo,
  ParkingPass,
  ParkingRecord,
  ParkingLotStatus,
  ElevatorCallRequest,
  EmergencyMode,
  RealtimeDashboard,
  AlertInfo,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  GrantAccessCommand,
  RevokeAccessCommand,
  UnlockDoorCommand,
  ActivateEmergencyCommand,
  UserType,
  CredentialType,
  AccessAction,
  EmergencyType,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface AccessControlSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class AccessControlSDK {
  private config: Required<AccessControlSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: AccessControlSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'CITY-015',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // User Management APIs
  // ==========================================================================

  /**
   * Get user information
   */
  async getUserInfo(userId: string): Promise<ApiResponse<UserInfo>> {
    return this.get<UserInfo>(`/api/v1/users/${userId}`);
  }

  /**
   * List all users
   */
  async listUsers(
    params?: PaginationParams & { userType?: UserType; status?: string }
  ): Promise<ApiResponse<PaginatedResponse<UserInfo>>> {
    const queryParams = this.buildQueryParams(params);
    return this.get<PaginatedResponse<UserInfo>>(
      `/api/v1/users${queryParams}`
    );
  }

  /**
   * Create new user
   */
  async createUser(user: Omit<UserInfo, 'userId'>): Promise<ApiResponse<UserInfo>> {
    return this.post<UserInfo>('/api/v1/users', user);
  }

  /**
   * Update user information
   */
  async updateUser(
    userId: string,
    updates: Partial<UserInfo>
  ): Promise<ApiResponse<UserInfo>> {
    return this.put<UserInfo>(`/api/v1/users/${userId}`, updates);
  }

  /**
   * Delete user
   */
  async deleteUser(userId: string): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/users/${userId}`);
  }

  // ==========================================================================
  // Credential Management APIs
  // ==========================================================================

  /**
   * Issue credential to user
   */
  async issueCredential(
    userId: string,
    credential: Partial<Credential>
  ): Promise<ApiResponse<Credential>> {
    return this.post<Credential>('/api/v1/credentials', {
      userId,
      ...credential,
    });
  }

  /**
   * Get credential details
   */
  async getCredential(credentialId: string): Promise<ApiResponse<Credential>> {
    return this.get<Credential>(`/api/v1/credentials/${credentialId}`);
  }

  /**
   * List user credentials
   */
  async listUserCredentials(userId: string): Promise<ApiResponse<Credential[]>> {
    return this.get<Credential[]>(`/api/v1/users/${userId}/credentials`);
  }

  /**
   * Suspend credential (e.g., lost card)
   */
  async suspendCredential(
    credentialId: string,
    reason: string
  ): Promise<ApiResponse<Credential>> {
    return this.post<Credential>(`/api/v1/credentials/${credentialId}/suspend`, {
      reason,
    });
  }

  /**
   * Reactivate credential
   */
  async reactivateCredential(credentialId: string): Promise<ApiResponse<Credential>> {
    return this.post<Credential>(`/api/v1/credentials/${credentialId}/reactivate`, {});
  }

  /**
   * Revoke credential permanently
   */
  async revokeCredential(
    credentialId: string,
    reason: string
  ): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/credentials/${credentialId}`, { reason });
  }

  // ==========================================================================
  // Access Control APIs
  // ==========================================================================

  /**
   * Grant access to user
   */
  async grantAccess(command: GrantAccessCommand): Promise<ApiResponse<UserInfo>> {
    return this.post<UserInfo>('/api/v1/access/grant', command);
  }

  /**
   * Revoke access from user
   */
  async revokeAccess(command: RevokeAccessCommand): Promise<ApiResponse<UserInfo>> {
    return this.post<UserInfo>('/api/v1/access/revoke', command);
  }

  /**
   * Verify credential for access
   */
  async verifyCredential(
    credentialId: string,
    accessPointId: string
  ): Promise<ApiResponse<{ allowed: boolean; reason?: string }>> {
    return this.post<{ allowed: boolean; reason?: string }>(
      '/api/v1/access/verify',
      { credentialId, accessPointId }
    );
  }

  // ==========================================================================
  // Access Point APIs
  // ==========================================================================

  /**
   * Get access point information
   */
  async getAccessPoint(accessPointId: string): Promise<ApiResponse<AccessPoint>> {
    return this.get<AccessPoint>(`/api/v1/access-points/${accessPointId}`);
  }

  /**
   * List all access points
   */
  async listAccessPoints(
    params?: PaginationParams & { zoneId?: string }
  ): Promise<ApiResponse<PaginatedResponse<AccessPoint>>> {
    const queryParams = this.buildQueryParams(params);
    return this.get<PaginatedResponse<AccessPoint>>(
      `/api/v1/access-points${queryParams}`
    );
  }

  /**
   * Unlock door remotely
   */
  async unlockDoor(command: UnlockDoorCommand): Promise<ApiResponse<void>> {
    return this.post<void>('/api/v1/access-points/unlock', command);
  }

  /**
   * Lock door remotely
   */
  async lockDoor(
    accessPointId: string,
    reason: string,
    authorizedBy: string
  ): Promise<ApiResponse<void>> {
    return this.post<void>('/api/v1/access-points/lock', {
      accessPointId,
      reason,
      authorizedBy,
    });
  }

  // ==========================================================================
  // Access Zone APIs
  // ==========================================================================

  /**
   * Get access zone information
   */
  async getAccessZone(zoneId: string): Promise<ApiResponse<AccessZone>> {
    return this.get<AccessZone>(`/api/v1/zones/${zoneId}`);
  }

  /**
   * List all access zones
   */
  async listAccessZones(
    params?: PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<AccessZone>>> {
    const queryParams = this.buildQueryParams(params);
    return this.get<PaginatedResponse<AccessZone>>(
      `/api/v1/zones${queryParams}`
    );
  }

  // ==========================================================================
  // Access Log APIs
  // ==========================================================================

  /**
   * Get access logs
   */
  async getAccessLogs(
    params: PaginationParams & {
      userId?: string;
      accessPointId?: string;
      zoneId?: string;
      action?: AccessAction;
      dateRange?: DateRangeFilter;
    }
  ): Promise<ApiResponse<PaginatedResponse<AccessLog>>> {
    const queryParams = this.buildQueryParams(params);
    return this.get<PaginatedResponse<AccessLog>>(
      `/api/v1/access-logs${queryParams}`
    );
  }

  /**
   * Get single access log entry
   */
  async getAccessLog(logId: string): Promise<ApiResponse<AccessLog>> {
    return this.get<AccessLog>(`/api/v1/access-logs/${logId}`);
  }

  /**
   * Export access logs (CSV/PDF)
   */
  async exportAccessLogs(
    params: {
      dateRange: DateRangeFilter;
      userId?: string;
      accessPointId?: string;
      format: 'csv' | 'pdf';
    }
  ): Promise<ApiResponse<{ downloadUrl: string }>> {
    return this.post<{ downloadUrl: string }>('/api/v1/access-logs/export', params);
  }

  // ==========================================================================
  // Visitor Management APIs
  // ==========================================================================

  /**
   * Register visitor (pre-registration)
   */
  async registerVisitor(visitor: Partial<Visitor>): Promise<ApiResponse<Visitor>> {
    return this.post<Visitor>('/api/v1/visitors', visitor);
  }

  /**
   * Get visitor information
   */
  async getVisitor(visitorId: string): Promise<ApiResponse<Visitor>> {
    return this.get<Visitor>(`/api/v1/visitors/${visitorId}`);
  }

  /**
   * List visitors
   */
  async listVisitors(
    params?: PaginationParams & { status?: string; visitDate?: string }
  ): Promise<ApiResponse<PaginatedResponse<Visitor>>> {
    const queryParams = this.buildQueryParams(params);
    return this.get<PaginatedResponse<Visitor>>(
      `/api/v1/visitors${queryParams}`
    );
  }

  /**
   * Check-in visitor
   */
  async checkInVisitor(
    visitorId: string,
    data?: { photoUrl?: string; notes?: string }
  ): Promise<ApiResponse<Visitor>> {
    return this.post<Visitor>(`/api/v1/visitors/${visitorId}/check-in`, data || {});
  }

  /**
   * Check-out visitor
   */
  async checkOutVisitor(visitorId: string): Promise<ApiResponse<Visitor>> {
    return this.post<Visitor>(`/api/v1/visitors/${visitorId}/check-out`, {});
  }

  /**
   * Cancel visitor registration
   */
  async cancelVisitor(
    visitorId: string,
    reason: string
  ): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/visitors/${visitorId}/cancel`, { reason });
  }

  // ==========================================================================
  // Parking Management APIs
  // ==========================================================================

  /**
   * Register vehicle
   */
  async registerVehicle(vehicle: Partial<VehicleInfo>): Promise<ApiResponse<VehicleInfo>> {
    return this.post<VehicleInfo>('/api/v1/parking/vehicles', vehicle);
  }

  /**
   * Get vehicle information
   */
  async getVehicle(vehicleId: string): Promise<ApiResponse<VehicleInfo>> {
    return this.get<VehicleInfo>(`/api/v1/parking/vehicles/${vehicleId}`);
  }

  /**
   * List vehicles
   */
  async listVehicles(
    params?: PaginationParams & { ownerId?: string }
  ): Promise<ApiResponse<PaginatedResponse<VehicleInfo>>> {
    const queryParams = this.buildQueryParams(params);
    return this.get<PaginatedResponse<VehicleInfo>>(
      `/api/v1/parking/vehicles${queryParams}`
    );
  }

  /**
   * Issue parking pass
   */
  async issueParkingPass(pass: Partial<ParkingPass>): Promise<ApiResponse<ParkingPass>> {
    return this.post<ParkingPass>('/api/v1/parking/passes', pass);
  }

  /**
   * Record parking entry
   */
  async recordParkingEntry(
    data: Partial<ParkingRecord>
  ): Promise<ApiResponse<ParkingRecord>> {
    return this.post<ParkingRecord>('/api/v1/parking/entry', data);
  }

  /**
   * Record parking exit
   */
  async recordParkingExit(
    data: Partial<ParkingRecord>
  ): Promise<ApiResponse<ParkingRecord>> {
    return this.post<ParkingRecord>('/api/v1/parking/exit', data);
  }

  /**
   * Get parking lot status
   */
  async getParkingLotStatus(
    parkingLotId: string
  ): Promise<ApiResponse<ParkingLotStatus>> {
    return this.get<ParkingLotStatus>(`/api/v1/parking/lots/${parkingLotId}/status`);
  }

  // ==========================================================================
  // Elevator Access Control APIs
  // ==========================================================================

  /**
   * Request elevator call
   */
  async requestElevatorCall(
    request: Partial<ElevatorCallRequest>
  ): Promise<ApiResponse<ElevatorCallRequest>> {
    return this.post<ElevatorCallRequest>('/api/v1/elevators/call', request);
  }

  /**
   * Get user elevator access rules
   */
  async getElevatorAccessRules(userId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/elevators/access-rules/${userId}`);
  }

  // ==========================================================================
  // Emergency Mode APIs
  // ==========================================================================

  /**
   * Activate emergency mode
   */
  async activateEmergency(
    command: ActivateEmergencyCommand
  ): Promise<ApiResponse<EmergencyMode>> {
    return this.post<EmergencyMode>('/api/v1/emergency/activate', command);
  }

  /**
   * Deactivate emergency mode
   */
  async deactivateEmergency(
    emergencyId: string,
    deactivatedBy: string,
    notes?: string
  ): Promise<ApiResponse<EmergencyMode>> {
    return this.post<EmergencyMode>('/api/v1/emergency/deactivate', {
      emergencyId,
      deactivatedBy,
      notes,
    });
  }

  /**
   * Get current emergency status
   */
  async getEmergencyStatus(): Promise<ApiResponse<EmergencyMode[]>> {
    return this.get<EmergencyMode[]>('/api/v1/emergency/status');
  }

  // ==========================================================================
  // Monitoring APIs
  // ==========================================================================

  /**
   * Get real-time dashboard
   */
  async getRealtimeDashboard(buildingId: string): Promise<ApiResponse<RealtimeDashboard>> {
    return this.get<RealtimeDashboard>(`/api/v1/dashboard/${buildingId}`);
  }

  /**
   * Get active alerts
   */
  async getActiveAlerts(
    buildingId: string,
    params?: PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<AlertInfo>>> {
    const queryParams = this.buildQueryParams({ ...params, buildingId });
    return this.get<PaginatedResponse<AlertInfo>>(
      `/api/v1/alerts${queryParams}`
    );
  }

  /**
   * Acknowledge alert
   */
  async acknowledgeAlert(
    alertId: string,
    acknowledgedBy: string,
    notes?: string
  ): Promise<ApiResponse<AlertInfo>> {
    return this.post<AlertInfo>(`/api/v1/alerts/${alertId}/acknowledge`, {
      acknowledgedBy,
      notes,
    });
  }

  // ==========================================================================
  // HTTP Helper Methods
  // ==========================================================================

  private async get<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, body: any): Promise<ApiResponse<T>> {
    return this.request<T>('POST', path, body);
  }

  private async put<T>(path: string, body: any): Promise<ApiResponse<T>> {
    return this.request<T>('PUT', path, body);
  }

  private async delete<T>(path: string, body?: any): Promise<ApiResponse<T>> {
    return this.request<T>('DELETE', path, body);
  }

  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<ApiResponse<T>> {
    try {
      const url = `${this.config.endpoint}${path}`;

      if (this.config.debug) {
        console.log(`[AccessControlSDK] ${method} ${url}`);
        if (body) {
          console.log('[AccessControlSDK] Body:', JSON.stringify(body, null, 2));
        }
      }

      // In a real implementation, use fetch or axios
      // This is a mock implementation
      const response: ApiResponse<T> = {
        success: true,
        data: {} as T,
        metadata: {
          timestamp: new Date().toISOString(),
          version: '1.0.0',
        },
      };

      if (this.config.debug) {
        console.log('[AccessControlSDK] Response:', JSON.stringify(response, null, 2));
      }

      return response;
    } catch (error: any) {
      const errorResponse: ApiResponse<T> = {
        success: false,
        error: {
          code: error.code || 'UNKNOWN_ERROR',
          message: error.message || 'An unknown error occurred',
          details: error,
        },
        metadata: {
          timestamp: new Date().toISOString(),
          version: '1.0.0',
        },
      };

      if (this.config.debug) {
        console.error('[AccessControlSDK] Error:', errorResponse);
      }

      return errorResponse;
    }
  }

  private buildQueryParams(params?: Record<string, any>): string {
    if (!params || Object.keys(params).length === 0) {
      return '';
    }

    const queryString = Object.entries(params)
      .filter(([_, value]) => value !== undefined && value !== null)
      .map(([key, value]) => {
        if (typeof value === 'object' && !Array.isArray(value)) {
          return `${key}=${encodeURIComponent(JSON.stringify(value))}`;
        }
        return `${key}=${encodeURIComponent(value)}`;
      })
      .join('&');

    return queryString ? `?${queryString}` : '';
  }
}

// ============================================================================
// Export SDK
// ============================================================================

export default AccessControlSDK;
