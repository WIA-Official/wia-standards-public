# WIA-AI-015 Phase 4: Integration Guide

> **Version:** 1.0.0
> **Status:** Stable
> **Last Updated:** 2025-12-25

## Overview

This guide provides patterns and best practices for integrating AI-human collaboration into existing systems and workflows.

## Integration Patterns

### 1. Sidecar Pattern

AI collaboration as auxiliary service alongside existing application:

```
┌─────────────────┐
│   Existing      │
│   Application   │◄─────┐
└─────────────────┘      │
         │               │
         │ API calls     │ Results
         ▼               │
┌─────────────────┐      │
│  Collaboration  │──────┘
│    Service      │
└─────────────────┘
```

**Pros**: Minimal changes to existing app
**Cons**: Network latency, additional infrastructure

### 2. Embedded Pattern

Collaboration logic embedded within application:

```
┌──────────────────────────┐
│     Application          │
│  ┌────────────────────┐  │
│  │  Collaboration     │  │
│  │    Library         │  │
│  └────────────────────┘  │
└──────────────────────────┘
```

**Pros**: Low latency, simple deployment
**Cons**: Tight coupling, version management

### 3. Gateway Pattern

Collaboration gateway routes between AI and humans:

```
Requests
   │
   ▼
┌─────────────┐
│  Gateway    │
└─────────────┘
   │      │
   │      └────────► Human Reviewers
   │
   └─────────────── ► AI Models
```

**Pros**: Centralized routing, easy monitoring
**Cons**: Single point of failure risk

## System Integration

### Integrate with Existing ML Pipeline

```python
# Existing ML pipeline
class ExistingMLPipeline:
    def predict(self, input_data):
        return self.model.predict(input_data)

# Enhanced with collaboration
from wia_ai_015 import CollaborationWrapper

class CollaborativeMLPipeline(ExistingMLPipeline):
    def __init__(self):
        super().__init__()
        self.collaboration = CollaborationWrapper(
            confidence_threshold=0.75,
            escalation_strategy='uncertainty'
        )

    def predict(self, input_data):
        # Get AI prediction
        ai_prediction = super().predict(input_data)

        # Check if human review needed
        if self.collaboration.should_escalate(ai_prediction):
            # Route to human
            human_decision = self.collaboration.request_human_review(
                input_data=input_data,
                ai_prediction=ai_prediction
            )
            return human_decision
        else:
            # Auto-approve
            return ai_prediction
```

### Integrate with Web Application

```typescript
// Frontend integration
import { CollaborationUI } from 'wia-ai-015-react';

function ReviewInterface() {
  return (
    <CollaborationUI
      apiEndpoint="https://api.example.com/v1/collaboration"
      onDecisionSubmit={handleDecision}
      features={{
        showConfidence: true,
        showExplanation: true,
        enableFeedback: true
      }}
    />
  );
}

// Backend integration
import { CollaborationService } from 'wia-ai-015';

const collab = new CollaborationService({
  apiKey: process.env.WIA_API_KEY,
  webhookUrl: 'https://yourapp.com/webhook'
});

app.post('/analyze', async (req, res) => {
  const result = await collab.processTask({
    inputData: req.body,
    priority: 'high'
  });

  res.json(result);
});
```

### Integrate with Database

```sql
-- Extend existing schema
ALTER TABLE decisions ADD COLUMN ai_prediction_id UUID;
ALTER TABLE decisions ADD COLUMN ai_confidence DECIMAL(3,2);
ALTER TABLE decisions ADD COLUMN human_reviewer_id VARCHAR(255);
ALTER TABLE decisions ADD COLUMN escalation_reason VARCHAR(50);

-- Create collaboration audit table
CREATE TABLE collaboration_audit (
  id UUID PRIMARY KEY,
  task_id UUID NOT NULL,
  event_type VARCHAR(50),
  event_data JSONB,
  timestamp TIMESTAMP DEFAULT NOW(),
  INDEX idx_task_id (task_id),
  INDEX idx_timestamp (timestamp)
);
```

## Workflow Integration

### Integrate into Approval Workflow

```yaml
# Existing workflow
approval_workflow:
  - step: submit_request
  - step: manager_review
  - step: final_approval

# Enhanced workflow
collaborative_approval_workflow:
  - step: submit_request
  - step: ai_initial_assessment
    service: wia-ai-015
    confidence_threshold: 0.8
  - step: conditional_routing
    if: ai_confidence < 0.8
    then: manager_review
    else: auto_approve
  - step: quality_assurance
    sample_rate: 0.05  # Random 5% for QA
  - step: final_approval
```

### Integrate into Content Moderation

```javascript
// Existing moderation
async function moderateContent(content) {
  if (containsBlockedKeywords(content)) {
    return { action: 'remove', reason: 'blocked_keyword' };
  }
  return { action: 'approve' };
}

// Enhanced with AI-human collaboration
async function collaborativeModeration(content) {
  // AI analysis
  const aiAnalysis = await aiModerator.analyze(content);

  if (aiAnalysis.confidence > 0.9) {
    // High confidence - auto-decide
    return aiAnalysis.action;
  } else if (aiAnalysis.severity === 'critical') {
    // Critical - immediate human review
    return await requestUrgentReview(content, aiAnalysis);
  } else {
    // Moderate confidence - queue for review
    return await queueForReview(content, aiAnalysis);
  }
}
```

## Data Integration

### Ingest Training Data from Decisions

```python
# Continuous learning pipeline
class FeedbackIngestionPipeline:
    def ingest_human_decisions(self, time_period='1d'):
        # Fetch decisions from last period
        decisions = self.collaboration_api.get_decisions(
            since=time_period
        )

        # Filter high-quality feedback
        quality_decisions = [
            d for d in decisions
            if d.feedback_quality > 0.7 and
               d.decision.confidence > 0.8
        ]

        # Convert to training examples
        training_data = [
            {
                'features': d.task.input_data.features,
                'label': d.decision.final_output,
                'weight': d.decision.confidence
            }
            for d in quality_decisions
        ]

        # Incrementally update model
        self.model.partial_fit(training_data)

        # Evaluate improvement
        metrics = self.evaluate_model()
        return metrics
```

### Export Metrics to BI Tools

```python
# Export to data warehouse
def export_collaboration_metrics():
    metrics = collaboration_service.get_metrics(period='7d')

    # Transform to warehouse schema
    warehouse_records = transform_metrics(metrics)

    # Load to data warehouse
    data_warehouse.bulk_insert(
        table='collaboration_metrics',
        records=warehouse_records
    )

    # Update dashboards
    dashboard_service.refresh('collaboration_overview')
```

## Authentication Integration

### SSO Integration

```typescript
import { CollaborationService } from 'wia-ai-015';
import { OAuthProvider } from 'your-sso-library';

const collab = new CollaborationService({
  auth: {
    provider: 'oauth2',
    tokenEndpoint: 'https://sso.example.com/token',
    clientId: process.env.CLIENT_ID,
    clientSecret: process.env.CLIENT_SECRET
  }
});

// User authentication
const user = await ssoProvider.authenticate(req.headers.authorization);

// Create collaboration session with user context
const session = await collab.createSession({
  user_id: user.id,
  user_email: user.email,
  user_roles: user.roles
});
```

### RBAC Integration

```typescript
// Define roles
const roles = {
  reviewer: ['read_tasks', 'submit_decisions'],
  expert: ['read_tasks', 'submit_decisions', 'train_model'],
  admin: ['read_tasks', 'submit_decisions', 'configure_system', 'view_all_metrics']
};

// Check permissions
function canPerformAction(user, action) {
  const userRoles = user.roles;
  const allowedActions = roles[userRoles].flatMap(r => roles[r]);
  return allowedActions.includes(action);
}

// Protect endpoints
app.post('/tasks/:id/decisions', authorize('submit_decisions'), async (req, res) => {
  // Handle decision submission
});
```

## Monitoring Integration

### Integrate with Prometheus

```yaml
# prometheus.yml
scrape_configs:
  - job_name: 'collaboration_service'
    static_configs:
      - targets: ['localhost:9090']
    metrics_path: '/metrics'
```

```typescript
// Expose metrics
import { register, Counter, Histogram } from 'prom-client';

const taskProcessed = new Counter({
  name: 'collaboration_tasks_processed_total',
  help: 'Total tasks processed',
  labelNames: ['status', 'decision_type']
});

const reviewTime = new Histogram({
  name: 'collaboration_review_time_seconds',
  help: 'Time spent on human review',
  buckets: [1, 5, 15, 60, 300, 900]
});

app.get('/metrics', (req, res) => {
  res.set('Content-Type', register.contentType);
  res.end(register.metrics());
});
```

## Migration Strategy

### Phased Migration

```
Phase 1: Parallel Run (2 weeks)
- Run collaboration system alongside existing system
- Compare results but don't act on collaboration output
- Validate accuracy and performance

Phase 2: Shadow Mode (4 weeks)
- Use collaboration for low-risk cases
- Keep existing system as fallback
- Gradually increase collaboration usage

Phase 3: Primary Mode (4 weeks)
- Collaboration becomes primary system
- Old system available as backup
- Monitor closely for issues

Phase 4: Full Migration (2 weeks)
- Decommission old system
- Collaboration is only system
- Continuous optimization
```

### Data Migration

```sql
-- Migrate historical decisions
INSERT INTO collaboration_decisions (
  task_id,
  decision,
  reviewer_id,
  timestamp,
  confidence,
  migrated_from_legacy
)
SELECT
  id,
  decision_value,
  reviewer,
  created_at,
  1.0, -- Assume high confidence for historical
  true
FROM legacy_decisions
WHERE created_at > '2024-01-01';

-- Create mapping table
CREATE TABLE legacy_id_mapping (
  legacy_id VARCHAR(255),
  collaboration_id UUID,
  entity_type VARCHAR(50),
  PRIMARY KEY (legacy_id, entity_type)
);
```

## Testing Integration

### Integration Tests

```typescript
describe('Collaboration Integration', () => {
  it('should escalate low-confidence predictions', async () => {
    const task = createTestTask();
    const prediction = { class: 'A', confidence: 0.65 };

    const result = await collaboration.process(task, prediction);

    expect(result.escalated).toBe(true);
    expect(result.assigned_reviewer).toBeDefined();
  });

  it('should auto-approve high-confidence predictions', async () => {
    const task = createTestTask();
    const prediction = { class: 'A', confidence: 0.92 };

    const result = await collaboration.process(task, prediction);

    expect(result.escalated).toBe(false);
    expect(result.final_decision).toEqual(prediction.class);
  });
});
```

## Troubleshooting

### Common Integration Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| High latency | Network calls to external service | Use embedded pattern or caching |
| Data inconsistency | Eventual consistency delays | Implement strong consistency for critical data |
| Authentication failures | Token expiration | Implement token refresh logic |
| Rate limiting | Too many API calls | Implement request batching and caching |

---

**弘益人間** (Hongik Ingan) · Benefit All Humanity

© 2025 SmileStory Inc. / WIA

---

## Annex A — Conformance Tier Matrix

WIA conformance for ai-human-collaboration is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/ai-human-collaboration/api/` — TypeScript SDK skeleton
- `wia-standards/standards/ai-human-collaboration/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/ai-human-collaboration/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
