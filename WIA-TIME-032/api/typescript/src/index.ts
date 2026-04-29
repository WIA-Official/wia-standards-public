/**
 * WIA-TIME-032: Time Access Control SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive access control for time travel operations including:
 * - Multi-factor authentication
 * - Authorization and clearance verification
 * - Geographic and temporal restrictions
 * - Real-time monitoring and audit trails
 * - Access revocation mechanisms
 */

import {
  AccessRequest,
  AccessResponse,
  AccessGrant,
  AccessDenial,
  AuthenticationRequest,
  AuthenticationResponse,
  AuthSession,
  RevocationRequest,
  RevocationResponse,
  AccessRevocation,
  TemporalAccess,
  ProtectedEvent,
  RestrictedEra,
  GeographicRestriction,
  AuditLog,
  AccessViolation,
  MonitoringData,
  Anomaly,
  BehaviorBaseline,
  Permission,
  ClearanceLevel,
  EraClass,
  ProtectionLevel,
  ViolationType,
  AccessErrorCode,
  AccessControlError,
  GeoCoordinate,
  AuthCredentials,
  DEFAULT_CONFIG,
  CLEARANCE_REQUIREMENTS,
  PaginationParams,
  PaginatedResponse,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-TIME-032 Time Access Control SDK
 */
export class TimeAccessControlSDK {
  private version = '1.0.0';
  private initialized = false;
  private sessions: Map<string, AuthSession> = new Map();
  private activeAccess: Map<string, TemporalAccess> = new Map();
  private auditLogs: AuditLog[] = [];

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ============================================================================
  // Authentication
  // ============================================================================

  /**
   * Authenticate user with multi-factor authentication
   *
   * @param request - Authentication request with credentials
   * @returns Authentication response with session token
   */
  async authenticate(request: AuthenticationRequest): Promise<AuthenticationResponse> {
    try {
      // Validate request
      if (!request.userId || !request.credentials) {
        throw new AccessControlError(
          AccessErrorCode.INVALID_REQUEST,
          'Invalid authentication request'
        );
      }

      // Count provided factors
      const factors = this.countAuthFactors(request.credentials);

      if (factors < DEFAULT_CONFIG.mfaFactorCount) {
        return {
          authenticated: false,
          mfaRequired: this.getMissingFactors(request.credentials),
          error: `Insufficient authentication factors. Provided: ${factors}, Required: ${DEFAULT_CONFIG.mfaFactorCount}`,
        };
      }

      // Verify each factor
      const passwordValid = await this.verifyPassword(
        request.userId,
        request.credentials.password
      );
      const biometricValid = await this.verifyBiometric(
        request.userId,
        request.credentials.biometric
      );
      const tokenValid = await this.verifyTemporalToken(
        request.userId,
        request.credentials.temporalToken
      );

      // All factors must be valid
      if (!passwordValid || !biometricValid || !tokenValid) {
        this.logAuthFailure(request.userId, request.deviceId);
        return {
          authenticated: false,
          error: 'Authentication failed. One or more factors invalid.',
        };
      }

      // Create session
      const session = this.createSession(request.userId);
      this.sessions.set(session.sessionId, session);

      // Log successful authentication
      this.logAudit({
        userId: request.userId,
        action: 'authenticate',
        actionType: 'AUTHENTICATION',
        granted: true,
        sessionId: session.sessionId,
        clearanceLevel: session.clearanceLevel,
        deviceId: request.deviceId,
        geolocation: request.location,
      });

      return {
        authenticated: true,
        sessionToken: session.sessionId,
        expiresAt: session.expiresAt,
        clearanceLevel: session.clearanceLevel,
      };
    } catch (error) {
      if (error instanceof AccessControlError) {
        throw error;
      }
      throw new AccessControlError(
        AccessErrorCode.AUTHENTICATION_FAILED,
        'Authentication failed',
        { error: String(error) }
      );
    }
  }

  /**
   * Verify user is authorized for specific action
   */
  async authorize(
    userId: string,
    permission: Permission,
    targetTime?: Date
  ): Promise<boolean> {
    const clearance = await this.getUserClearance(userId);
    const userPermissions = this.getPermissionsForClearance(clearance);

    return userPermissions.includes(permission);
  }

  // ============================================================================
  // Access Request
  // ============================================================================

  /**
   * Request temporal access
   *
   * @param request - Access request with target time and purpose
   * @returns Access response (grant or denial)
   */
  async requestAccess(request: AccessRequest): Promise<AccessResponse> {
    try {
      // Validate session
      const session = this.sessions.get(request.sessionToken);
      if (!session || session.expiresAt < new Date()) {
        throw new AccessControlError(
          AccessErrorCode.TOKEN_EXPIRED,
          'Session expired or invalid'
        );
      }

      // Check clearance for era
      const eraClass = this.classifyEra(request.targetTime);
      const minClearance = this.getMinClearanceForEra(eraClass);

      if (session.clearanceLevel < minClearance) {
        const denial: AccessDenial = {
          denialId: this.generateId('DEN'),
          userId: request.userId,
          deniedAt: new Date(),
          reason: `Insufficient clearance. Have: Level ${session.clearanceLevel}, Need: Level ${minClearance}`,
          denialCode: AccessErrorCode.CLEARANCE_INSUFFICIENT,
          appealProcess: 'Contact supervisor for clearance upgrade',
        };

        this.logAudit({
          userId: request.userId,
          action: 'request_access',
          actionType: 'ACCESS_DENY',
          granted: false,
          sessionId: session.sessionId,
          clearanceLevel: session.clearanceLevel,
          targetTime: request.targetTime,
          targetLocation: request.targetLocation,
          denialReason: denial.reason,
        });

        return { granted: false, denial };
      }

      // Check protected events
      const protectedEvents = this.findProtectedEvents(
        request.targetTime,
        request.targetLocation
      );

      for (const event of protectedEvents) {
        if (event.protectionLevel === ProtectionLevel.ABSOLUTE &&
            session.clearanceLevel < ClearanceLevel.GUARDIAN) {
          const denial: AccessDenial = {
            denialId: this.generateId('DEN'),
            userId: request.userId,
            deniedAt: new Date(),
            reason: `Protected event: ${event.name}. Guardian authorization required.`,
            denialCode: AccessErrorCode.GUARDIAN_APPROVAL_REQUIRED,
            appealProcess: 'Submit request to Guardian Council',
          };

          return { granted: false, denial };
        }
      }

      // Check geographic restrictions
      const geoRestrictions = this.checkGeographicRestrictions(
        request.targetLocation,
        request.targetTime,
        request.userId
      );

      if (geoRestrictions.length > 0) {
        const denial: AccessDenial = {
          denialId: this.generateId('DEN'),
          userId: request.userId,
          deniedAt: new Date(),
          reason: `Geographic restriction: ${geoRestrictions[0].name}`,
          denialCode: AccessErrorCode.LOCATION_RESTRICTED,
        };

        return { granted: false, denial };
      }

      // Check for active revocations
      const revocation = this.getActiveRevocation(request.userId);
      if (revocation) {
        const denial: AccessDenial = {
          denialId: this.generateId('DEN'),
          userId: request.userId,
          deniedAt: new Date(),
          reason: `Access revoked: ${revocation.reason}`,
          denialCode: AccessErrorCode.ACCESS_REVOKED,
        };

        return { granted: false, denial };
      }

      // Grant access
      const grant: AccessGrant = {
        grantId: this.generateId('GNT'),
        userId: request.userId,
        granted: new Date(),
        expiresAt: new Date(Date.now() + request.duration * 1000),
        targetTime: request.targetTime,
        targetLocation: request.targetLocation,
        duration: request.duration,
        permissions: request.permissions,
        accessToken: this.generateAccessToken(request),
        restrictions: this.generateRestrictions(request, protectedEvents),
        warnings: this.generateWarnings(request, eraClass),
        monitoringRequired: eraClass !== EraClass.PUBLIC,
        realTimeTracking: session.clearanceLevel <= ClearanceLevel.RESEARCHER,
      };

      // Create active access
      const access: TemporalAccess = {
        accessId: this.generateId('ACC'),
        userId: request.userId,
        grantId: grant.grantId,
        startTime: new Date(),
        targetTime: request.targetTime,
        targetLocation: request.targetLocation,
        expiresAt: grant.expiresAt,
        status: 'active',
        permissions: request.permissions,
        violations: [],
      };

      this.activeAccess.set(access.accessId, access);

      // Log grant
      this.logAudit({
        userId: request.userId,
        action: 'grant_access',
        actionType: 'ACCESS_GRANT',
        granted: true,
        sessionId: session.sessionId,
        clearanceLevel: session.clearanceLevel,
        targetTime: request.targetTime,
        targetLocation: request.targetLocation,
        duration: request.duration,
        permissions: request.permissions,
        purpose: request.purpose,
      });

      return { granted: true, grant };
    } catch (error) {
      if (error instanceof AccessControlError) {
        throw error;
      }
      throw new AccessControlError(
        AccessErrorCode.INVALID_REQUEST,
        'Access request failed',
        { error: String(error) }
      );
    }
  }

  /**
   * Grant access to another user (requires GRANT permission)
   */
  async grantAccess(
    granterId: string,
    targetUserId: string,
    clearanceLevel: ClearanceLevel,
    duration?: number
  ): Promise<AccessGrant> {
    // Verify granter has GRANT permission
    const canGrant = await this.authorize(granterId, Permission.GRANT);
    if (!canGrant) {
      throw new AccessControlError(
        AccessErrorCode.AUTHORIZATION_DENIED,
        'User does not have GRANT permission'
      );
    }

    // Cannot grant higher clearance than granter has
    const granterClearance = await this.getUserClearance(granterId);
    if (clearanceLevel > granterClearance) {
      throw new AccessControlError(
        AccessErrorCode.AUTHORIZATION_DENIED,
        'Cannot grant higher clearance than you possess'
      );
    }

    // Create grant (simplified)
    const grant: AccessGrant = {
      grantId: this.generateId('GNT'),
      userId: targetUserId,
      granted: new Date(),
      expiresAt: new Date(Date.now() + (duration || 86400) * 1000),
      targetTime: new Date(),
      targetLocation: { lat: 0, lon: 0 },
      duration: duration || 86400,
      permissions: this.getPermissionsForClearance(clearanceLevel),
      accessToken: this.generateId('TOK'),
      restrictions: [],
      warnings: [],
      monitoringRequired: true,
      realTimeTracking: false,
    };

    this.logAudit({
      userId: granterId,
      action: 'grant_access_to_user',
      actionType: 'ACCESS_GRANT',
      granted: true,
      sessionId: this.generateId('SES'),
      clearanceLevel: granterClearance,
    });

    return grant;
  }

  // ============================================================================
  // Access Control
  // ============================================================================

  /**
   * Check restrictions for specific time and location
   */
  checkRestrictions(params: {
    time: Date;
    location: GeoCoordinate;
    userId: string;
  }): {
    restricted: boolean;
    reasons: string[];
    eraClass: EraClass;
    protectedEvents: ProtectedEvent[];
    geoRestrictions: GeographicRestriction[];
  } {
    const eraClass = this.classifyEra(params.time);
    const protectedEvents = this.findProtectedEvents(params.time, params.location);
    const geoRestrictions = this.checkGeographicRestrictions(
      params.location,
      params.time,
      params.userId
    );

    const reasons: string[] = [];
    let restricted = false;

    if (eraClass === EraClass.FORBIDDEN) {
      restricted = true;
      reasons.push('Forbidden era - Guardian authorization required');
    }

    if (protectedEvents.some(e => e.protectionLevel === ProtectionLevel.ABSOLUTE)) {
      restricted = true;
      reasons.push('Absolutely protected event in proximity');
    }

    if (geoRestrictions.length > 0) {
      restricted = true;
      reasons.push(...geoRestrictions.map(r => `Geographic restriction: ${r.name}`));
    }

    return {
      restricted,
      reasons,
      eraClass,
      protectedEvents,
      geoRestrictions,
    };
  }

  // ============================================================================
  // Revocation
  // ============================================================================

  /**
   * Revoke user access
   */
  async revokeAccess(request: RevocationRequest): Promise<RevocationResponse> {
    // Verify initiator has REVOKE permission
    const canRevoke = await this.authorize(request.initiatorUserId, Permission.REVOKE);
    if (!canRevoke) {
      throw new AccessControlError(
        AccessErrorCode.AUTHORIZATION_DENIED,
        'User does not have REVOKE permission'
      );
    }

    // Find active sessions and access
    const userSessions = Array.from(this.sessions.values()).filter(
      s => s.userId === request.targetUserId
    );
    const userAccess = Array.from(this.activeAccess.values()).filter(
      a => a.userId === request.targetUserId
    );

    // Terminate sessions
    let sessionsTerminated = 0;
    for (const session of userSessions) {
      this.sessions.delete(session.sessionId);
      sessionsTerminated++;
    }

    // Invalidate active access
    let tokensInvalidated = 0;
    for (const access of userAccess) {
      access.status = 'revoked';
      tokensInvalidated++;
    }

    // Create revocation record
    const revocation: AccessRevocation = {
      revocationId: this.generateId('REV'),
      userId: request.targetUserId,
      initiatedBy: request.initiatorUserId,
      timestamp: new Date(),
      reason: request.reason,
      severity: request.severity,
      scope: request.scope,
      duration: request.duration,
      expiresAt: request.duration === 'temporary' && request.temporaryDuration
        ? new Date(Date.now() + request.temporaryDuration * 1000)
        : undefined,
      status: 'active',
    };

    // Log revocation
    this.logAudit({
      userId: request.initiatorUserId,
      action: 'revoke_access',
      actionType: 'REVOCATION',
      granted: true,
      sessionId: this.generateId('SES'),
      clearanceLevel: await this.getUserClearance(request.initiatorUserId),
    });

    return {
      success: true,
      revocationId: revocation.revocationId,
      sessionsTerminated,
      tokensInvalidated,
      recallInitiated: request.recallRequired,
    };
  }

  // ============================================================================
  // Audit and Monitoring
  // ============================================================================

  /**
   * Get audit logs with optional filters
   */
  async auditAccess(params: {
    userId?: string;
    startTime?: Date;
    endTime?: Date;
    actionType?: string;
    granted?: boolean;
    pagination?: PaginationParams;
  }): Promise<PaginatedResponse<AuditLog>> {
    let filtered = this.auditLogs;

    if (params.userId) {
      filtered = filtered.filter(log => log.userId === params.userId);
    }

    if (params.startTime) {
      filtered = filtered.filter(log => log.timestamp >= params.startTime!);
    }

    if (params.endTime) {
      filtered = filtered.filter(log => log.timestamp <= params.endTime!);
    }

    if (params.actionType) {
      filtered = filtered.filter(log => log.actionType === params.actionType);
    }

    if (params.granted !== undefined) {
      filtered = filtered.filter(log => log.granted === params.granted);
    }

    // Pagination
    const page = params.pagination?.page || 1;
    const pageSize = params.pagination?.pageSize || 50;
    const start = (page - 1) * pageSize;
    const end = start + pageSize;
    const paginated = filtered.slice(start, end);

    return {
      data: paginated,
      total: filtered.length,
      page,
      pageSize,
      totalPages: Math.ceil(filtered.length / pageSize),
    };
  }

  /**
   * Monitor active temporal access in real-time
   */
  monitorAccess(accessId: string): MonitoringData | null {
    const access = this.activeAccess.get(accessId);
    if (!access) return null;

    return {
      userId: access.userId,
      currentTime: new Date(),
      currentLocation: access.currentLocation || access.targetLocation,
      accessGranted: access.startTime,
      expiresAt: access.expiresAt,
      interactions: 0,
      observations: 0,
      modifications: 0,
      withinAuthorizedArea: true,
      withinAuthorizedTime: new Date() < access.expiresAt,
      permissionsRespected: true,
      violations: access.violations,
      warnings: [],
      locationAccuracy: 10,
      deviceBattery: 85,
      novikovCompliance: 1.0,
      timelineStability: 0.99,
      paradoxRisk: 0.01,
    };
  }

  /**
   * Detect anomalies in user behavior
   */
  detectAnomalies(userId: string): Anomaly[] {
    const baseline = this.getBehaviorBaseline(userId);
    const recentActivity = this.getRecentActivity(userId, 7); // Last 7 days

    const anomalies: Anomaly[] = [];

    // Frequency anomaly
    if (recentActivity.length > baseline.frequency * 2) {
      anomalies.push({
        type: 'UNUSUAL_FREQUENCY',
        severity: 'moderate',
        description: `Access frequency ${recentActivity.length} is 2x above baseline ${baseline.frequency}`,
        confidence: 0.8,
        detectedAt: new Date(),
        userId,
        recommendation: 'Review recent access requests for legitimacy',
      });
    }

    return anomalies;
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Count authentication factors provided
   */
  private countAuthFactors(credentials: AuthCredentials): number {
    let count = 0;
    if (credentials.password) count++;
    if (credentials.biometric) count++;
    if (credentials.temporalToken) count++;
    if (credentials.physicalKey) count++;
    if (credentials.mfaCode) count++;
    if (credentials.quantumKey) count++;
    return count;
  }

  /**
   * Get missing authentication factors
   */
  private getMissingFactors(credentials: AuthCredentials): string[] {
    const missing: string[] = [];
    if (!credentials.password) missing.push('password');
    if (!credentials.biometric) missing.push('biometric');
    if (!credentials.temporalToken) missing.push('temporal-token');
    return missing;
  }

  /**
   * Verify password (simplified)
   */
  private async verifyPassword(userId: string, password?: string): Promise<boolean> {
    // In production, this would check against secure storage
    return password !== undefined && password.length >= 8;
  }

  /**
   * Verify biometric data (simplified)
   */
  private async verifyBiometric(userId: string, biometric?: any): Promise<boolean> {
    // In production, this would perform actual biometric matching
    return biometric !== undefined;
  }

  /**
   * Verify temporal token (simplified)
   */
  private async verifyTemporalToken(userId: string, token?: string): Promise<boolean> {
    // In production, this would verify HMAC and timestamp
    return token !== undefined && token.length > 0;
  }

  /**
   * Create authentication session
   */
  private createSession(userId: string): AuthSession {
    return {
      sessionId: this.generateId('SES'),
      userId,
      authenticated: new Date(),
      expiresAt: new Date(Date.now() + DEFAULT_CONFIG.sessionTimeoutSeconds * 1000),
      mfaFactors: ['password', 'biometric', 'temporal-token'],
      clearanceLevel: ClearanceLevel.OPERATOR, // Default, would be fetched from DB
      renewalCount: 0,
      lastActivity: new Date(),
    };
  }

  /**
   * Get user clearance level
   */
  private async getUserClearance(userId: string): Promise<ClearanceLevel> {
    // In production, fetch from database
    return ClearanceLevel.OPERATOR;
  }

  /**
   * Get permissions for clearance level
   */
  private getPermissionsForClearance(clearance: ClearanceLevel): Permission[] {
    switch (clearance) {
      case ClearanceLevel.OBSERVER:
        return [Permission.READ];
      case ClearanceLevel.RESEARCHER:
        return [Permission.READ, Permission.INTERACT];
      case ClearanceLevel.OPERATOR:
        return [Permission.READ, Permission.INTERACT, Permission.MODIFY];
      case ClearanceLevel.ADMINISTRATOR:
        return [Permission.READ, Permission.INTERACT, Permission.MODIFY, Permission.ADMIN, Permission.GRANT, Permission.REVOKE, Permission.AUDIT];
      case ClearanceLevel.GUARDIAN:
        return Object.values(Permission);
      default:
        return [Permission.READ];
    }
  }

  /**
   * Classify era based on time
   */
  private classifyEra(time: Date): EraClass {
    const now = new Date();
    const yearsDiff = (time.getTime() - now.getTime()) / (365.25 * 24 * 3600 * 1000);

    if (Math.abs(yearsDiff) <= 10) return EraClass.FORBIDDEN;
    if (Math.abs(yearsDiff) <= 50) return EraClass.CLASSIFIED;
    if (Math.abs(yearsDiff) <= 100) return EraClass.PROTECTED;
    if (Math.abs(yearsDiff) <= 200) return EraClass.RESTRICTED;
    return EraClass.PUBLIC;
  }

  /**
   * Get minimum clearance for era
   */
  private getMinClearanceForEra(eraClass: EraClass): ClearanceLevel {
    switch (eraClass) {
      case EraClass.PUBLIC: return ClearanceLevel.OBSERVER;
      case EraClass.RESTRICTED: return ClearanceLevel.RESEARCHER;
      case EraClass.PROTECTED: return ClearanceLevel.OPERATOR;
      case EraClass.CLASSIFIED: return ClearanceLevel.ADMINISTRATOR;
      case EraClass.FORBIDDEN: return ClearanceLevel.GUARDIAN;
    }
  }

  /**
   * Find protected events near time and location
   */
  private findProtectedEvents(time: Date, location: GeoCoordinate): ProtectedEvent[] {
    // In production, query from protected events database
    return [];
  }

  /**
   * Check geographic restrictions
   */
  private checkGeographicRestrictions(
    location: GeoCoordinate,
    time: Date,
    userId: string
  ): GeographicRestriction[] {
    // In production, query from geofence database
    return [];
  }

  /**
   * Get active revocation for user
   */
  private getActiveRevocation(userId: string): AccessRevocation | null {
    // In production, query from revocations database
    return null;
  }

  /**
   * Generate access token
   */
  private generateAccessToken(request: AccessRequest): string {
    return `tok_${Date.now()}_${Math.random().toString(36).substr(2, 16)}`;
  }

  /**
   * Generate restrictions for access
   */
  private generateRestrictions(
    request: AccessRequest,
    protectedEvents: ProtectedEvent[]
  ): any[] {
    const restrictions = [];

    if (protectedEvents.length > 0) {
      restrictions.push({
        type: 'interaction',
        description: 'Limited interaction near protected events',
        enforcement: 'warning',
        details: { events: protectedEvents.map(e => e.name) },
      });
    }

    return restrictions;
  }

  /**
   * Generate warnings for access
   */
  private generateWarnings(request: AccessRequest, eraClass: EraClass): string[] {
    const warnings: string[] = [];

    if (eraClass === EraClass.PROTECTED || eraClass === EraClass.CLASSIFIED) {
      warnings.push('High-sensitivity era. All actions are monitored.');
    }

    if (request.duration > 7200) {
      warnings.push('Extended duration access. Periodic check-ins required.');
    }

    return warnings;
  }

  /**
   * Get behavior baseline for user
   */
  private getBehaviorBaseline(userId: string): BehaviorBaseline {
    return {
      userId,
      frequency: 2, // Average accesses per week
      commonEras: ['1900-2000'],
      commonLocations: [],
      averageDuration: 3600,
      typicalPermissions: [Permission.READ, Permission.INTERACT],
      lastUpdated: new Date(),
    };
  }

  /**
   * Get recent activity for user
   */
  private getRecentActivity(userId: string, days: number): AuditLog[] {
    const cutoff = new Date(Date.now() - days * 24 * 3600 * 1000);
    return this.auditLogs.filter(
      log => log.userId === userId && log.timestamp >= cutoff
    );
  }

  /**
   * Log authentication failure
   */
  private logAuthFailure(userId: string, deviceId: string): void {
    // In production, increment failure counter and apply rate limiting
    console.warn(`Authentication failure for user ${userId} on device ${deviceId}`);
  }

  /**
   * Log audit entry
   */
  private logAudit(entry: Partial<AuditLog>): void {
    const log: AuditLog = {
      id: this.generateId('AUD'),
      timestamp: new Date(),
      userId: entry.userId || '',
      sessionId: entry.sessionId || '',
      clearanceLevel: entry.clearanceLevel || ClearanceLevel.OBSERVER,
      action: entry.action || '',
      actionType: entry.actionType || 'ACCESS_REQUEST',
      granted: entry.granted || false,
      violations: entry.violations || [],
      alerts: entry.alerts || [],
      ipAddress: '0.0.0.0',
      deviceId: entry.deviceId || '',
      geolocation: entry.geolocation || { lat: 0, lon: 0 },
      signature: this.generateId('SIG'),
      integrity: this.generateId('HASH'),
      ...entry,
    };

    this.auditLogs.push(log);
  }

  /**
   * Generate unique ID
   */
  private generateId(prefix: string): string {
    return `${prefix}-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Authenticate user (standalone function)
 */
export async function authenticate(
  request: AuthenticationRequest
): Promise<AuthenticationResponse> {
  const sdk = new TimeAccessControlSDK();
  return sdk.authenticate(request);
}

/**
 * Request temporal access (standalone function)
 */
export async function requestAccess(request: AccessRequest): Promise<AccessResponse> {
  const sdk = new TimeAccessControlSDK();
  return sdk.requestAccess(request);
}

/**
 * Revoke access (standalone function)
 */
export async function revokeAccess(
  request: RevocationRequest
): Promise<RevocationResponse> {
  const sdk = new TimeAccessControlSDK();
  return sdk.revokeAccess(request);
}

/**
 * Check restrictions (standalone function)
 */
export function checkRestrictions(params: {
  time: Date;
  location: GeoCoordinate;
  userId: string;
}): ReturnType<TimeAccessControlSDK['checkRestrictions']> {
  const sdk = new TimeAccessControlSDK();
  return sdk.checkRestrictions(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { TimeAccessControlSDK };
export default TimeAccessControlSDK;
