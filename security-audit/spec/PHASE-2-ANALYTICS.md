# WIA-SEC-017: Security Audit
## Phase 2 - Advanced Analytics & Machine Learning

**Standard ID:** WIA-SEC-017
**Category:** Security (SEC)
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

# PHASE 2: Advanced Analytics & Machine Learning

## 1. Anomaly Detection Engine

### 1.1 Behavioral Analysis

The anomaly detection engine uses machine learning to identify suspicious patterns in audit logs:

```javascript
class AuditAnomalyDetector {
  constructor() {
    this.model = new IsolationForest({
      contamination: 0.1,  // Expected anomaly rate
      nEstimators: 100
    });
    this.userProfiles = new Map();
  }

  /**
   * Train model on historical audit data
   */
  async train(historicalLogs) {
    const features = historicalLogs.map(log =>
      this.extractFeatures(log)
    );

    await this.model.fit(features);

    // Build user behavior profiles
    this.buildUserProfiles(historicalLogs);
  }

  /**
   * Extract features from audit log
   */
  extractFeatures(auditLog) {
    return {
      // Temporal features
      hour_of_day: new Date(auditLog.timestamp).getHours(),
      day_of_week: new Date(auditLog.timestamp).getDay(),
      is_weekend: [0, 6].includes(new Date(auditLog.timestamp).getDay()),

      // Location features
      country_code: this.encodeCountry(auditLog.actor.location.country),
      is_vpn: this.detectVPN(auditLog.actor.ip_address),
      distance_from_usual: this.calculateLocationDeviation(
        auditLog.actor.user_id,
        auditLog.actor.location
      ),

      // Access pattern features
      resource_sensitivity: this.classifyResourceSensitivity(auditLog.resource),
      operation_type: this.encodeOperation(auditLog.action.operation),
      failure_rate: this.calculateRecentFailureRate(auditLog.actor.user_id),

      // User features
      user_tenure_days: this.calculateUserTenure(auditLog.actor.user_id),
      privilege_level: this.getUserPrivilegeLevel(auditLog.actor.user_id),
      recent_activity_volume: this.getRecentActivityVolume(auditLog.actor.user_id)
    };
  }

  /**
   * Detect anomalies in real-time
   */
  async detectAnomaly(auditLog) {
    const features = this.extractFeatures(auditLog);
    const anomalyScore = await this.model.predict([features]);

    // Multi-layer anomaly detection
    const detections = {
      ml_score: anomalyScore[0],
      rule_based: this.ruleBasedDetection(auditLog),
      behavioral: this.behavioralDetection(auditLog),
      statistical: this.statisticalDetection(auditLog)
    };

    // Aggregate scores
    const finalScore = this.aggregateScores(detections);

    return {
      is_anomaly: finalScore > 0.7,
      confidence: finalScore,
      detections: detections,
      recommended_actions: this.recommendActions(finalScore, detections)
    };
  }

  /**
   * Rule-based anomaly detection
   */
  ruleBasedDetection(auditLog) {
    let score = 0;
    const reasons = [];

    // Check for impossible travel
    if (this.isImpossibleTravel(auditLog)) {
      score += 0.9;
      reasons.push('Impossible travel detected');
    }

    // Check for unusual access time
    if (this.isUnusualTime(auditLog)) {
      score += 0.3;
      reasons.push('Access outside normal hours');
    }

    // Check for privilege escalation
    if (auditLog.event_type === 'PRIVILEGE_ESCALATION') {
      score += 0.6;
      reasons.push('Privilege escalation event');
    }

    // Check for repeated failures
    const failureRate = this.calculateRecentFailureRate(auditLog.actor.user_id);
    if (failureRate > 0.5) {
      score += 0.8;
      reasons.push(`High failure rate: ${(failureRate * 100).toFixed(1)}%`);
    }

    return { score: Math.min(score, 1.0), reasons };
  }

  /**
   * Behavioral deviation detection
   */
  behavioralDetection(auditLog) {
    const userId = auditLog.actor.user_id;
    const profile = this.userProfiles.get(userId);

    if (!profile) {
      return { score: 0.5, reasons: ['New user - no baseline'] };
    }

    let score = 0;
    const reasons = [];

    // Check location deviation
    const locationDev = this.calculateLocationDeviation(userId, auditLog.actor.location);
    if (locationDev > 1000) { // km
      score += 0.5;
      reasons.push(`Unusual location: ${locationDev.toFixed(0)}km from normal`);
    }

    // Check resource access pattern
    if (!profile.typical_resources.includes(auditLog.resource.id)) {
      score += 0.4;
      reasons.push('Accessing unusual resource');
    }

    // Check time pattern
    const hourDev = Math.abs(
      new Date(auditLog.timestamp).getHours() - profile.typical_hour
    );
    if (hourDev > 4) {
      score += 0.3;
      reasons.push('Unusual time of access');
    }

    return { score: Math.min(score, 1.0), reasons };
  }

  /**
   * Statistical outlier detection
   */
  statisticalDetection(auditLog) {
    const userId = auditLog.actor.user_id;
    const profile = this.userProfiles.get(userId);

    if (!profile || !profile.statistics) {
      return { score: 0.3, reasons: ['Insufficient data'] };
    }

    let score = 0;
    const reasons = [];

    // Check if current activity is outlier
    const currentVolume = this.getRecentActivityVolume(userId, 1); // last hour
    const zscore = (currentVolume - profile.statistics.mean_hourly_activity) /
                   profile.statistics.std_hourly_activity;

    if (Math.abs(zscore) > 3) {
      score += 0.7;
      reasons.push(`Unusual activity volume: ${zscore.toFixed(2)} std devs`);
    }

    return { score: Math.min(score, 1.0), reasons };
  }
}
```

### 1.2 Threat Intelligence Integration

```javascript
class ThreatIntelligenceIntegration {
  constructor() {
    this.feeds = {
      ip_reputation: new IPReputationService(),
      malware_indicators: new MalwareIndicatorService(),
      threat_actors: new ThreatActorService()
    };
  }

  /**
   * Enrich audit log with threat intelligence
   */
  async enrichAuditLog(auditLog) {
    const enrichment = {
      ip_reputation: await this.checkIPReputation(auditLog.actor.ip_address),
      known_threats: await this.checkKnownThreats(auditLog),
      geo_risk: await this.assessGeoRisk(auditLog.actor.location)
    };

    // Calculate threat score
    const threatScore = this.calculateThreatScore(enrichment);

    return {
      ...auditLog,
      threat_intelligence: enrichment,
      threat_score: threatScore,
      risk_level: this.classifyRisk(threatScore)
    };
  }

  async checkIPReputation(ipAddress) {
    const reputation = await this.feeds.ip_reputation.check(ipAddress);

    return {
      score: reputation.score, // 0-100
      categories: reputation.categories, // ['proxy', 'tor', 'vpn', 'malicious']
      last_seen_malicious: reputation.last_seen,
      threat_feeds: reputation.feeds_reporting
    };
  }
}
```

---


## 5. Behavioral Baseline & Drift Detection

### 5.1 Baseline Construction

The audit analyzer maintains rolling baselines per actor across multiple feature dimensions. A baseline is a multi-dimensional descriptor over a sliding window of recent activity, retained per actor identity for the configured retention horizon.

```javascript
class BehavioralBaseline {
  constructor(window_days = 30) {
    this.window_days = window_days;
    this.profiles = new Map();
  }

  update(actor_id, audit_event) {
    const profile = this.profiles.get(actor_id) || this.empty();
    profile.hour_distribution[audit_event.hour] += 1;
    profile.action_distribution[audit_event.action] =
      (profile.action_distribution[audit_event.action] || 0) + 1;
    profile.locations.add(audit_event.location);
    profile.event_count += 1;
    profile.last_seen = audit_event.timestamp;
    this.profiles.set(actor_id, profile);
  }

  drift_score(actor_id, recent_event) {
    const profile = this.profiles.get(actor_id);
    if (!profile) return 1.0; // unseen actor: maximal drift
    const hour_p = profile.hour_distribution[recent_event.hour] / profile.event_count;
    const action_p = (profile.action_distribution[recent_event.action] || 0) / profile.event_count;
    const loc_known = profile.locations.has(recent_event.location) ? 1 : 0;
    return 1 - (0.4 * hour_p + 0.4 * action_p + 0.2 * loc_known);
  }
}
```

### 5.2 Drift Thresholds

| Drift Score | Risk Level | Action |
|-------------|-----------|--------|
| < 0.3 | normal | log only |
| 0.3 – 0.6 | moderate | annotate, raise risk score |
| 0.6 – 0.85 | elevated | challenge with step-up auth |
| ≥ 0.85 | severe | block + incident + analyst review |

Thresholds are configurable per actor class (human, service account, machine identity) since service accounts typically exhibit much narrower distributions and tolerate tighter thresholds.

### 5.3 Pipeline Integration

Drift scoring runs **inline** with audit ingestion before the event is committed to the immutable log so that a high-drift event can carry an annotation in the persisted record. The annotation is part of the signed payload and cannot be retro-edited.

---
