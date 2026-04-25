# WIA-SEC-017: Security Audit
## Phase 3 - Real-time Monitoring & Alerting

**Standard ID:** WIA-SEC-017
**Category:** Security (SEC)
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

# PHASE 3: Real-time Monitoring & Alerting

## 2. Real-time Event Streaming

### 2.1 Event Stream Architecture

```javascript
// WebSocket-based real-time audit streaming
class AuditEventStream {
  constructor(config) {
    this.kafka = new KafkaClient(config.kafka);
    this.subscriptions = new Map();
    this.filters = new Map();
  }

  /**
   * Subscribe to audit events with filters
   */
  subscribe(clientId, filters) {
    const subscription = {
      id: clientId,
      filters: filters,
      socket: null,
      last_sequence: 0
    };

    this.subscriptions.set(clientId, subscription);

    // Create filtered Kafka consumer
    this.createConsumer(clientId, filters);

    return subscription.id;
  }

  /**
   * Create Kafka consumer for filtered audit stream
   */
  async createConsumer(clientId, filters) {
    const consumer = this.kafka.consumer({
      groupId: `audit-stream-${clientId}`,
      sessionTimeout: 30000
    });

    await consumer.connect();
    await consumer.subscribe({
      topic: 'security-audit-events',
      fromBeginning: false
    });

    await consumer.run({
      eachMessage: async ({ topic, partition, message }) => {
        const auditEvent = JSON.parse(message.value.toString());

        // Apply client filters
        if (this.matchesFilters(auditEvent, filters)) {
          this.sendToClient(clientId, auditEvent);
        }
      }
    });
  }

  /**
   * Filter matching logic
   */
  matchesFilters(auditEvent, filters) {
    // Severity filter
    if (filters.severity && !filters.severity.includes(auditEvent.severity)) {
      return false;
    }

    // Event type filter
    if (filters.event_types && !filters.event_types.includes(auditEvent.event_type)) {
      return false;
    }

    // Compliance tags filter
    if (filters.compliance_tags) {
      const hasTag = filters.compliance_tags.some(tag =>
        auditEvent.metadata.compliance_tags.includes(tag)
      );
      if (!hasTag) return false;
    }

    // User filter
    if (filters.users && !filters.users.includes(auditEvent.actor.user_id)) {
      return false;
    }

    // Resource filter
    if (filters.resources && !filters.resources.includes(auditEvent.resource.id)) {
      return false;
    }

    return true;
  }
}
```

### 2.2 Alert Configuration

```json
{
  "alert_rules": [
    {
      "id": "RULE-001",
      "name": "Critical Security Event",
      "description": "Alert on any critical severity event",
      "enabled": true,
      "conditions": {
        "severity": ["CRITICAL"],
        "event_types": ["*"]
      },
      "actions": [
        {
          "type": "email",
          "recipients": ["security@company.com"],
          "template": "critical_security_event"
        },
        {
          "type": "sms",
          "recipients": ["+1-555-0100"],
          "message": "CRITICAL: {{event_type}} by {{actor.user_id}}"
        },
        {
          "type": "webhook",
          "url": "https://incident.company.com/api/create",
          "method": "POST"
        },
        {
          "type": "slack",
          "channel": "#security-alerts",
          "mention": ["@security-oncall"]
        }
      ],
      "throttle": {
        "window_seconds": 300,
        "max_alerts": 5
      }
    },
    {
      "id": "RULE-002",
      "name": "Privilege Escalation Detected",
      "description": "Alert on privilege escalation events",
      "enabled": true,
      "conditions": {
        "event_type": ["PRIVILEGE_ESCALATION"],
        "severity": ["HIGH", "CRITICAL"]
      },
      "actions": [
        {
          "type": "incident",
          "severity": "high",
          "runbook": "https://runbooks.company.com/privilege-escalation"
        }
      ]
    },
    {
      "id": "RULE-003",
      "name": "Anomalous Access Pattern",
      "description": "Alert on detected anomalies",
      "enabled": true,
      "conditions": {
        "anomaly_score": { "gte": 0.8 }
      },
      "actions": [
        {
          "type": "email",
          "recipients": ["soc@company.com"]
        },
        {
          "type": "automatic_response",
          "action": "require_mfa_reverification"
        }
      ]
    }
  ]
}
```

### 2.3 Incident Response Automation

```javascript
class AutomatedIncidentResponse {
  constructor() {
    this.playbooks = new Map();
    this.loadPlaybooks();
  }

  /**
   * Execute automated response to security event
   */
  async respondToEvent(auditEvent, anomalyDetection) {
    // Determine response level
    const responseLevel = this.determineResponseLevel(
      auditEvent.severity,
      anomalyDetection.confidence
    );

    // Select appropriate playbook
    const playbook = this.selectPlaybook(auditEvent.event_type, responseLevel);

    // Execute response actions
    const results = await this.executePlaybook(playbook, auditEvent);

    // Log response actions
    await this.logResponseActions(auditEvent, playbook, results);

    return results;
  }

  /**
   * Example playbook: Suspicious Login
   */
  async executeSuspiciousLoginPlaybook(auditEvent) {
    const actions = [];

    // 1. Require MFA re-verification
    actions.push(
      await this.requireMFAReverification(auditEvent.actor.session_id)
    );

    // 2. Notify user of suspicious activity
    actions.push(
      await this.notifyUser(auditEvent.actor.user_id, {
        type: 'suspicious_login',
        location: auditEvent.actor.location,
        ip: auditEvent.actor.ip_address
      })
    );

    // 3. Limit session permissions
    actions.push(
      await this.limitSessionPermissions(auditEvent.actor.session_id, {
        allowed_operations: ['READ'],
        restricted_resources: ['CONFIDENTIAL', 'RESTRICTED']
      })
    );

    // 4. Monitor subsequent activity
    actions.push(
      await this.enableEnhancedMonitoring(auditEvent.actor.user_id, {
        duration_minutes: 60,
        capture_all_actions: true
      })
    );

    return actions;
  }

  /**
   * Example playbook: Data Breach Attempt
   */
  async executeDataBreachPlaybook(auditEvent) {
    const actions = [];

    // 1. Immediately terminate session
    actions.push(
      await this.terminateSession(auditEvent.actor.session_id)
    );

    // 2. Lock user account
    actions.push(
      await this.lockAccount(auditEvent.actor.user_id, {
        reason: 'Suspected data breach attempt',
        require_security_review: true
      })
    );

    // 3. Block IP address
    actions.push(
      await this.blockIPAddress(auditEvent.actor.ip_address, {
        duration_hours: 24,
        scope: 'GLOBAL'
      })
    );

    // 4. Create critical incident
    actions.push(
      await this.createIncident({
        severity: 'CRITICAL',
        type: 'DATA_BREACH_ATTEMPT',
        details: auditEvent,
        assignee: 'security-oncall'
      })
    );

    // 5. Notify security team
    actions.push(
      await this.notifySecurityTeam({
        priority: 'URGENT',
        event: auditEvent,
        automated_actions: actions
      })
    );

    return actions;
  }
}
```

---


## 5. Alert Fatigue Suppression

### 5.1 Deduplication Window

Alerts originating from the same correlation key within a rolling window collapse into a single alert with an incremented occurrence counter. The correlation key combines event class, actor identity, target resource, and primary indicator-of-compromise so that a burst of identical findings produces one analyst-visible item rather than dozens.

```javascript
class AlertDeduplicator {
  constructor(window_seconds = 300) {
    this.window = window_seconds * 1000;
    this.active = new Map();
  }

  observe(alert) {
    const key = `${alert.class}|${alert.actor}|${alert.target}|${alert.ioc}`;
    const now = Date.now();
    const existing = this.active.get(key);
    if (existing && (now - existing.first_seen) < this.window) {
      existing.occurrences += 1;
      existing.last_seen = now;
      return { suppressed: true, parent: existing.id };
    }
    const fresh = { id: alert.id, first_seen: now, last_seen: now, occurrences: 1 };
    this.active.set(key, fresh);
    return { suppressed: false, parent: null };
  }
}
```

### 5.2 Severity Promotion Rules

A suppressed alert that recurs above a threshold within the dedup window is promoted: a counter exceeding the configured promotion ratio escalates the parent alert one severity tier and re-publishes it to the analyst queue. This protects against an attacker exploiting dedup logic to mask sustained activity.

### 5.3 Closed-Loop Tuning

Each analyst disposition (false positive, true positive, benign-true) feeds back into the rule's effectiveness score. Rules with sustained false-positive rates above the configured cutoff are auto-quarantined: still evaluated and logged but with reduced visibility, awaiting tuning review.
