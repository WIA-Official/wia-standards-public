/**
 * WIA-DEF-001: Unmanned Weapon SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for unmanned weapon systems including:
 * - Target acquisition and tracking
 * - Threat assessment
 * - Engagement authorization
 * - Ethical compliance verification
 * - Swarm coordination
 */

import {
  WeaponConfig,
  Target,
  ThreatAssessment,
  EngagementRequest,
  EngagementAuthorization,
  EthicalCheck,
  EthicalViolation,
  RulesOfEngagement,
  Geofence,
  SwarmConfig,
  SwarmTaskAllocation,
  SafetyCheck,
  SystemDiagnostic,
  EngagementLog,
  BattleDamageAssessment,
  Coordinate3D,
  Vector3D,
  DEFENSE_CONSTANTS,
  DefenseErrorCode,
  DefenseError,
  IFFStatus,
  TargetClass,
  AutonomyLevel,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-001 Unmanned Weapon SDK
 */
export class UnmannedWeaponSDK {
  private version = '1.0.0';
  private initialized = false;

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Create weapon system configuration
   *
   * @param params - Weapon configuration parameters
   * @returns Weapon configuration
   */
  createWeaponConfig(params: Partial<WeaponConfig>): WeaponConfig {
    const defaults: WeaponConfig = {
      id: `WPN-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      type: 'UAV',
      role: 'air-defense',
      autonomyLevel: 2,
      payload: {
        type: 'interceptor',
        count: 4,
        range: 10000,
        guidance: 'radar',
        pk: 0.85,
      },
      sensors: [
        {
          type: 'radar',
          range: 50000,
          accuracy: 0.95,
          operational: true,
        },
      ],
      range: 50000,
      maxSpeed: 200,
      endurance: 7200,
      status: 'standby',
    };

    return { ...defaults, ...params } as WeaponConfig;
  }

  /**
   * Assess threat level of a target
   *
   * @param target - Target to assess
   * @param protectedAsset - Position of asset being protected
   * @returns Threat assessment
   */
  assessThreat(
    target: Target,
    protectedAsset?: Coordinate3D
  ): ThreatAssessment {
    // Calculate capability score based on target type
    const capabilityScore = this.calculateCapabilityScore(target.classification);

    // Calculate intent score based on IFF and behavior
    const intentScore = this.calculateIntentScore(target);

    // Calculate proximity score
    const proximityScore = protectedAsset
      ? this.calculateProximityScore(target.position, protectedAsset)
      : 0.5;

    // Calculate velocity score
    const velocityScore = this.calculateVelocityScore(target.velocity);

    // Combined threat score (weighted average)
    const threatLevel =
      0.3 * capabilityScore +
      0.35 * intentScore +
      0.2 * proximityScore +
      0.15 * velocityScore;

    // Classify threat
    let classification: ThreatAssessment['classification'];
    if (threatLevel < 0.3) {
      classification = 'low';
    } else if (threatLevel < 0.6) {
      classification = 'medium';
    } else if (threatLevel < 0.8) {
      classification = 'high';
    } else {
      classification = 'critical';
    }

    // Determine recommended action
    let recommendedAction: ThreatAssessment['recommendedAction'];
    if (classification === 'low') {
      recommendedAction = 'monitor';
    } else if (classification === 'medium') {
      recommendedAction = 'track';
    } else if (classification === 'high') {
      recommendedAction = 'prepare';
    } else {
      recommendedAction = 'engage';
    }

    // Estimate time to impact if target is approaching
    const timeToImpact = this.estimateTimeToImpact(target, protectedAsset);

    return {
      target,
      threatLevel,
      classification,
      scores: {
        capability: capabilityScore,
        intent: intentScore,
        proximity: proximityScore,
        velocity: velocityScore,
      },
      timeToImpact,
      recommendedAction,
      timestamp: new Date(),
    };
  }

  /**
   * Validate engagement authorization
   *
   * @param request - Engagement request
   * @param roe - Rules of engagement
   * @param weapon - Weapon configuration
   * @returns Engagement authorization result
   */
  validateEngagement(
    request: EngagementRequest,
    roe: RulesOfEngagement,
    weapon: WeaponConfig
  ): EngagementAuthorization {
    const errors: string[] = [];
    const warnings: string[] = [];
    const reasoning: string[] = [];

    const { target, threatAssessment, proposedWeapon, humanApproved } = request;

    // Check target certainty
    if (target.certainty < DEFENSE_CONSTANTS.MIN_TARGET_CERTAINTY) {
      errors.push(
        `Target certainty (${target.certainty.toFixed(
          2
        )}) below minimum (${DEFENSE_CONSTANTS.MIN_TARGET_CERTAINTY})`
      );
    } else {
      reasoning.push(`Target certainty acceptable: ${target.certainty.toFixed(2)}`);
    }

    // Check threat level
    if (threatAssessment.threatLevel < DEFENSE_CONSTANTS.MIN_THREAT_LEVEL) {
      warnings.push(
        `Threat level (${threatAssessment.threatLevel.toFixed(
          2
        )}) below typical engagement threshold`
      );
    } else {
      reasoning.push(
        `Threat level sufficient: ${threatAssessment.threatLevel.toFixed(2)}`
      );
    }

    // Check IFF status
    if (target.iffStatus === 'friendly') {
      errors.push('CRITICAL: Target identified as FRIENDLY - engagement prohibited');
    } else if (target.iffStatus === 'unknown' || target.iffStatus === 'neutral') {
      errors.push(
        'Target IFF status uncertain - escalate to human operator'
      );
    } else {
      reasoning.push('IFF status: Hostile');
    }

    // Check ROE compliance
    const roeCheck = this.checkROECompliance(target, roe);
    if (!roeCheck.compliant) {
      errors.push(...roeCheck.violations);
    } else {
      reasoning.push('ROE compliance verified');
    }

    // Check geofencing
    const geofenceCheck = this.checkGeofence(target.position, roe);
    if (!geofenceCheck.allowed) {
      errors.push(`Geofence violation: ${geofenceCheck.reason}`);
    } else {
      reasoning.push(`Geofence check: ${geofenceCheck.zone}`);
    }

    // Estimate collateral damage
    const collateralDamage = this.estimateCollateralDamage(
      target,
      weapon,
      proposedWeapon
    );

    if (collateralDamage > roe.maxCollateralDamage) {
      errors.push(
        `Estimated collateral damage (${(collateralDamage * 100).toFixed(
          1
        )}%) exceeds maximum (${(roe.maxCollateralDamage * 100).toFixed(1)}%)`
      );
    } else if (collateralDamage > 0.01) {
      warnings.push(
        `Collateral damage risk: ${(collateralDamage * 100).toFixed(1)}%`
      );
      reasoning.push('Collateral damage within acceptable limits');
    } else {
      reasoning.push('Minimal collateral damage risk');
    }

    // Ethical compliance check
    const ethicalCompliance = this.checkEthics(target, weapon, collateralDamage);

    if (!ethicalCompliance.compliant) {
      errors.push('Ethical compliance failure');
      errors.push(
        ...ethicalCompliance.violations.map(
          (v) => `${v.type}: ${v.description}`
        )
      );
    } else {
      reasoning.push('Ethical compliance verified');
    }

    // Calculate proportionality
    const proportionality = this.calculateProportionality(
      threatAssessment.threatLevel,
      collateralDamage
    );

    if (proportionality < 8.0) {
      warnings.push(
        `Low proportionality score: ${proportionality.toFixed(2)} (threshold: 8.0)`
      );
    } else {
      reasoning.push(`Proportionality score: ${proportionality.toFixed(2)}`);
    }

    // Calculate engagement score
    const engagementScore =
      threatAssessment.threatLevel *
      target.certainty *
      (proportionality / 10) *
      (target.iffStatus === 'hostile' ? 1 : 0) *
      (roeCheck.compliant ? 1 : 0);

    reasoning.push(`Engagement score: ${engagementScore.toFixed(3)}`);

    // Check autonomy level requirements
    const humanApprovalRequired = this.requiresHumanApproval(
      weapon.autonomyLevel,
      engagementScore,
      collateralDamage
    );

    if (humanApprovalRequired && !humanApproved) {
      errors.push(
        `Human approval required for autonomy level ${weapon.autonomyLevel}`
      );
    }

    // Determine authorization
    const authorized = errors.length === 0;

    return {
      authorized,
      engagementScore,
      reasoning,
      errors,
      warnings,
      weaponSelected: authorized ? proposedWeapon : undefined,
      estimatedCollateralDamage: collateralDamage,
      ethicalCompliance,
      proportionality,
      humanApprovalRequired,
      timestamp: new Date(),
    };
  }

  /**
   * Check ethical compliance
   *
   * @param target - Target being considered
   * @param weapon - Weapon system
   * @param collateralDamage - Estimated collateral damage
   * @returns Ethical check result
   */
  checkEthics(
    target: Target,
    weapon: WeaponConfig,
    collateralDamage: number
  ): EthicalCheck {
    const violations: EthicalViolation[] = [];
    const recommendations: string[] = [];

    // Distinction check
    const distinction = target.certainty >= 0.95;
    if (!distinction) {
      violations.push({
        type: 'indiscriminate-weapon',
        severity: 'critical',
        description: 'Cannot reliably distinguish target type',
        remediation: 'Increase sensor coverage or wait for better identification',
      });
    }

    // Civilian targeting check
    if (
      target.classification === 'civilian' ||
      target.classification === 'unknown'
    ) {
      violations.push({
        type: 'civilian-targeting',
        severity: 'prohibited',
        description: 'Targeting civilians or unknown entities is prohibited',
        affected: [target.id],
        remediation: 'Abort engagement immediately',
      });
    }

    // Proportionality check
    const proportionality =
      collateralDamage <= DEFENSE_CONSTANTS.MAX_COLLATERAL_DAMAGE;
    if (!proportionality) {
      violations.push({
        type: 'proportionality',
        severity: 'critical',
        description: 'Collateral damage exceeds acceptable limits',
        remediation: 'Use more precise weapon or delay engagement',
      });
    }

    // Necessity check
    const necessity = target.classification.startsWith('hostile');
    if (!necessity) {
      violations.push({
        type: 'excessive-force',
        severity: 'warning',
        description: 'Target may not pose immediate threat',
        remediation: 'Verify threat assessment',
      });
    }

    // Humanity check (minimize suffering)
    const humanity = weapon.payload.type !== 'non-lethal' || collateralDamage < 0.01;

    // IHL compliance
    const ihl = {
      distinction,
      proportionality,
      necessity,
      humanity,
    };

    // Determine if compliant
    const compliant =
      violations.filter((v) => v.severity === 'critical' || v.severity === 'prohibited')
        .length === 0;

    // Generate recommendations
    if (!compliant) {
      recommendations.push('Do not proceed with engagement');
      recommendations.push('Escalate to human commander');
    } else if (violations.length > 0) {
      recommendations.push('Proceed with caution');
      recommendations.push('Ensure continuous monitoring');
    }

    // Override allowed only for emergency defensive actions
    const overrideAllowed =
      collateralDamage < 0.1 && target.iffStatus === 'hostile';

    return {
      compliant,
      ihl,
      violations,
      recommendations,
      overrideAllowed,
    };
  }

  /**
   * Allocate tasks to swarm members
   *
   * @param swarm - Swarm configuration
   * @param tasks - Tasks to allocate
   * @returns Task allocations
   */
  allocateSwarmTasks(
    swarm: SwarmConfig,
    tasks: Array<{ location: Coordinate3D; priority: number; type: string }>
  ): SwarmTaskAllocation[] {
    const allocations: SwarmTaskAllocation[] = [];

    // Simple greedy allocation based on proximity
    const availableUnits = [...swarm.memberIds];

    for (const task of tasks.sort((a, b) => b.priority - a.priority)) {
      if (availableUnits.length === 0) break;

      // Find nearest unit
      // For simplicity, assign to first available unit
      const unitId = availableUnits.shift()!;

      allocations.push({
        taskId: `TSK-${Date.now()}-${Math.random().toString(36).substr(2, 6)}`,
        unitId,
        taskType: task.type as any,
        priority: task.priority,
        location: task.location,
        status: 'assigned',
      });
    }

    return allocations;
  }

  /**
   * Perform system diagnostics
   *
   * @param weapon - Weapon configuration
   * @returns Diagnostic result
   */
  runDiagnostics(weapon: WeaponConfig): SystemDiagnostic {
    // Simplified diagnostic - in real implementation would query actual hardware
    return {
      systemId: weapon.id,
      health: 0.95,
      components: {
        propulsion: {
          name: 'Propulsion System',
          operational: true,
          health: 0.98,
          errors: [],
          maintenanceRequired: false,
        },
        weapons: {
          name: 'Weapon System',
          operational: true,
          health: 0.92,
          errors: [],
          maintenanceRequired: false,
        },
        sensors: {
          name: 'Sensor Suite',
          operational: weapon.sensors.every((s) => s.operational),
          health: 0.96,
          errors: [],
          maintenanceRequired: false,
        },
        communications: {
          name: 'Communications',
          operational: true,
          health: 0.99,
          errors: [],
          maintenanceRequired: false,
        },
        navigation: {
          name: 'Navigation',
          operational: true,
          health: 0.97,
          errors: [],
          maintenanceRequired: false,
        },
        power: {
          name: 'Power System',
          operational: true,
          health: 0.94,
          errors: [],
          maintenanceRequired: false,
        },
      },
      faults: [],
      timestamp: new Date(),
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Calculate capability score based on target classification
   */
  private calculateCapabilityScore(classification: TargetClass): number {
    const scores: Record<TargetClass, number> = {
      'hostile-missile': 1.0,
      'hostile-aircraft': 0.9,
      'hostile-uav': 0.7,
      'hostile-vehicle': 0.6,
      'hostile-vessel': 0.7,
      'hostile-personnel': 0.3,
      unknown: 0.5,
      civilian: 0.0,
      friendly: 0.0,
    };

    return scores[classification] || 0.5;
  }

  /**
   * Calculate intent score based on IFF and behavior
   */
  private calculateIntentScore(target: Target): number {
    const iffScores: Record<IFFStatus, number> = {
      hostile: 1.0,
      unknown: 0.6,
      neutral: 0.3,
      friendly: 0.0,
    };

    return iffScores[target.iffStatus] || 0.5;
  }

  /**
   * Calculate proximity score (inverse of distance)
   */
  private calculateProximityScore(
    targetPos: Coordinate3D,
    assetPos: Coordinate3D
  ): number {
    const distance = this.calculateDistance(targetPos, assetPos);

    // Score decreases with distance (inverse relationship)
    // At 100m: 1.0, at 1000m: 0.5, at 10000m: 0.1
    const score = Math.max(0, Math.min(1, 10000 / (distance + 100)));

    return score;
  }

  /**
   * Calculate velocity score (speed toward asset)
   */
  private calculateVelocityScore(velocity: Vector3D): number {
    const speed = Math.sqrt(
      velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2
    );

    // Higher speed = higher threat
    // Normalize to typical threat speeds (0-500 m/s)
    return Math.min(1, speed / 500);
  }

  /**
   * Estimate time to impact
   */
  private estimateTimeToImpact(
    target: Target,
    asset?: Coordinate3D
  ): number | undefined {
    if (!asset) return undefined;

    const distance = this.calculateDistance(target.position, asset);
    const speed = Math.sqrt(
      target.velocity.x ** 2 +
        target.velocity.y ** 2 +
        target.velocity.z ** 2
    );

    if (speed < 1) return undefined; // Not moving

    return distance / speed;
  }

  /**
   * Calculate distance between two coordinates
   */
  private calculateDistance(a: Coordinate3D, b: Coordinate3D): number {
    // Simplified distance calculation (should use haversine for real geo coords)
    const R = 6371000; // Earth radius in meters
    const dLat = ((b.latitude - a.latitude) * Math.PI) / 180;
    const dLon = ((b.longitude - a.longitude) * Math.PI) / 180;
    const lat1 = (a.latitude * Math.PI) / 180;
    const lat2 = (b.latitude * Math.PI) / 180;

    const h =
      Math.sin(dLat / 2) ** 2 +
      Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.asin(Math.sqrt(h));

    const horizontalDist = R * c;
    const verticalDist = Math.abs(b.altitude - a.altitude);

    return Math.sqrt(horizontalDist ** 2 + verticalDist ** 2);
  }

  /**
   * Check ROE compliance
   */
  private checkROECompliance(
    target: Target,
    roe: RulesOfEngagement
  ): { compliant: boolean; violations: string[] } {
    const violations: string[] = [];

    // Check authorized targets
    if (
      roe.authorizedTargets.length > 0 &&
      !roe.authorizedTargets.includes(target.classification)
    ) {
      violations.push(
        `Target type ${target.classification} not in authorized list`
      );
    }

    // Check prohibited targets
    if (roe.prohibitedTargets.includes(target.classification)) {
      violations.push(
        `Target type ${target.classification} is explicitly prohibited`
      );
    }

    // Check time validity
    const now = new Date();
    if (now < roe.validFrom || now > roe.validUntil) {
      violations.push('ROE not valid at current time');
    }

    return {
      compliant: violations.length === 0,
      violations,
    };
  }

  /**
   * Check geofence compliance
   */
  private checkGeofence(
    position: Coordinate3D,
    roe: RulesOfEngagement
  ): { allowed: boolean; reason: string; zone: string } {
    // Check no-fire zones
    for (const zone of roe.noFireZones) {
      if (this.isInGeofence(position, zone)) {
        return {
          allowed: false,
          reason: 'Target in no-fire zone',
          zone: zone.name || zone.id,
        };
      }
    }

    // Check weapons-free zones
    for (const zone of roe.weaponsFreeZones) {
      if (this.isInGeofence(position, zone)) {
        return {
          allowed: true,
          reason: 'Weapons free',
          zone: zone.name || zone.id,
        };
      }
    }

    // Check weapons-tight zones
    for (const zone of roe.weaponsTightZones) {
      if (this.isInGeofence(position, zone)) {
        return {
          allowed: true,
          reason: 'Weapons tight (specific targets only)',
          zone: zone.name || zone.id,
        };
      }
    }

    return {
      allowed: false,
      reason: 'Not in authorized engagement zone',
      zone: 'none',
    };
  }

  /**
   * Check if position is within geofence
   */
  private isInGeofence(position: Coordinate3D, fence: Geofence): boolean {
    // Simplified check - in production would use proper polygon containment
    if (fence.type === 'circle' && fence.center && fence.radius) {
      const distance = this.calculateDistance(position, fence.center);
      return distance <= fence.radius;
    }

    // For polygon, simplified bounding box check
    if (fence.coordinates.length > 0) {
      const lats = fence.coordinates.map((c) => c.latitude);
      const lons = fence.coordinates.map((c) => c.longitude);
      const minLat = Math.min(...lats);
      const maxLat = Math.max(...lats);
      const minLon = Math.min(...lons);
      const maxLon = Math.max(...lons);

      return (
        position.latitude >= minLat &&
        position.latitude <= maxLat &&
        position.longitude >= minLon &&
        position.longitude <= maxLon
      );
    }

    return false;
  }

  /**
   * Estimate collateral damage
   */
  private estimateCollateralDamage(
    target: Target,
    weapon: WeaponConfig,
    proposedWeapon: string
  ): number {
    // Simplified estimation based on weapon type and location
    // In production, would use detailed terrain, population, and weapon effects models

    const payload = weapon.payload;
    const lethalRadius = payload.lethalRadius || 0;

    // Estimate civilian presence (very simplified)
    // In reality, would query population database
    const estimatedCivilians = this.estimateCiviliansNearTarget(target, lethalRadius);

    // Probability of casualty per civilian
    const casualtyProbability = 0.1; // 10% per civilian in area

    // Expected casualties
    const expectedCasualties = estimatedCivilians * casualtyProbability;

    // Normalize to 0-1 scale (>10 casualties = 1.0)
    return Math.min(1, expectedCasualties / 10);
  }

  /**
   * Estimate civilians near target
   */
  private estimateCiviliansNearTarget(
    target: Target,
    radius: number
  ): number {
    // Highly simplified - in production would query actual population data
    // Assume rural area with low population density
    if (target.classification.includes('hostile')) {
      return 0; // Assume hostile targets in unpopulated areas
    }

    // Default conservative estimate
    return 5;
  }

  /**
   * Calculate proportionality score
   */
  private calculateProportionality(
    threatLevel: number,
    collateralDamage: number
  ): number {
    if (collateralDamage < 0.001) {
      return 10.0; // Maximum proportionality
    }

    return (threatLevel * 10) / Math.max(collateralDamage, 0.01);
  }

  /**
   * Check if human approval is required
   */
  private requiresHumanApproval(
    autonomyLevel: AutonomyLevel,
    engagementScore: number,
    collateralDamage: number
  ): boolean {
    // Level 0-1: Always require human
    if (autonomyLevel <= 1) return true;

    // Level 2: Always require human approval for engagement
    if (autonomyLevel === 2) return true;

    // Level 3: Require human if score too low or collateral risk
    if (autonomyLevel === 3) {
      return (
        engagementScore < DEFENSE_CONSTANTS.MIN_ENGAGEMENT_SCORE_L3 ||
        collateralDamage > 0.01
      );
    }

    // Level 4: Require human if score too low or significant collateral risk
    if (autonomyLevel === 4) {
      return (
        engagementScore < DEFENSE_CONSTANTS.MIN_ENGAGEMENT_SCORE_L4 ||
        collateralDamage > 0.05
      );
    }

    return true;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create weapon configuration (standalone function)
 */
export function createWeaponConfig(
  params: Partial<WeaponConfig>
): WeaponConfig {
  const sdk = new UnmannedWeaponSDK();
  return sdk.createWeaponConfig(params);
}

/**
 * Assess threat (standalone function)
 */
export function assessThreat(
  target: Target,
  protectedAsset?: Coordinate3D
): ThreatAssessment {
  const sdk = new UnmannedWeaponSDK();
  return sdk.assessThreat(target, protectedAsset);
}

/**
 * Validate engagement (standalone function)
 */
export function validateEngagement(
  request: EngagementRequest,
  roe: RulesOfEngagement,
  weapon: WeaponConfig
): EngagementAuthorization {
  const sdk = new UnmannedWeaponSDK();
  return sdk.validateEngagement(request, roe, weapon);
}

/**
 * Check ethical compliance (standalone function)
 */
export function checkEthics(
  target: Target,
  weapon: WeaponConfig,
  collateralDamage: number
): EthicalCheck {
  const sdk = new UnmannedWeaponSDK();
  return sdk.checkEthics(target, weapon, collateralDamage);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { UnmannedWeaponSDK };
export default UnmannedWeaponSDK;
