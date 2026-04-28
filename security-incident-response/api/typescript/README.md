# @wia/security-incident-response

TypeScript SDK for WIA Security Incident Response Standard - NIST/SANS Framework Implementation

## Overview

This SDK provides a complete implementation of the NIST 800-61 and SANS incident response frameworks, enabling organizations to systematically detect, respond to, and recover from security incidents.

## Features

- **Complete NIST/SANS Lifecycle Support**
  - Preparation
  - Detection & Analysis
  - Containment, Eradication & Recovery
  - Post-Incident Activity

- **Comprehensive Incident Management**
  - Incident detection and classification
  - Severity-based triage and escalation
  - Multi-role team coordination
  - Real-time incident timeline tracking

- **Forensic Evidence Management**
  - Evidence collection and preservation
  - Chain of custody tracking
  - Multiple evidence type support (logs, memory dumps, network captures, etc.)

- **Response Automation**
  - Automated containment strategies
  - Eradication verification
  - Communication management
  - Escalation workflows

- **Analytics & Reporting**
  - Incident statistics and metrics
  - Post-incident reviews
  - Lessons learned tracking
  - Performance metrics (MTTD, MTTR, etc.)

## Installation

```bash
npm install @wia/security-incident-response
```

## Quick Start

```typescript
import { createIncidentResponse, IncidentType, IncidentSeverity } from '@wia/security-incident-response';

// Initialize the incident response system
const incidentResponse = createIncidentResponse({
  organizationName: 'ACME Corp',
  incidentResponseTeam: [
    {
      id: 'ir-001',
      name: 'Alice Johnson',
      role: 'incident_manager',
      contact: { email: 'alice@acme.com', phone: '+1-555-0100' },
      availability: 'available'
    },
    {
      id: 'ir-002',
      name: 'Bob Smith',
      role: 'security_analyst',
      contact: { email: 'bob@acme.com', phone: '+1-555-0101' },
      availability: 'available'
    }
  ],
  escalationMatrix: [],
  retentionPolicy: {
    incidentData: 365,
    forensicEvidence: 730,
    logs: 90
  },
  automationEnabled: true,
  integrationsEnabled: true
});

// Detect a new incident
const incident = await incidentResponse.detectIncident({
  type: IncidentType.RANSOMWARE,
  severity: IncidentSeverity.CRITICAL,
  title: 'Ransomware detected on file server',
  description: 'File encryption activity detected on FS-PROD-01',
  reportedBy: 'SOC Analyst',
  affectedSystems: ['FS-PROD-01', 'FS-PROD-02'],
  affectedData: ['Financial Records', 'Customer Data'],
  indicators: [
    {
      type: 'hash',
      value: 'a1b2c3d4e5f6...',
      source: 'VirusTotal',
      confidence: 'high',
      firstSeen: new Date()
    }
  ]
});

console.log(`Incident created: ${incident.id}`);
```

## Core Workflows

### 1. Detection & Analysis

```typescript
// Triage the incident
await incidentResponse.triageIncident(incident.id, {
  severity: IncidentSeverity.CRITICAL,
  assignedTo: ['incident_manager', 'security_analyst', 'forensic_investigator'],
  additionalContext: 'Ransomware variant: Conti 2.0'
});

// Perform deep analysis
await incidentResponse.analyzeIncident(incident.id, {
  analyst: 'Bob Smith',
  findings: 'Initial access via phishing email. Lateral movement to 5 systems detected.',
  additionalIOCs: [
    { type: 'ip', value: '192.0.2.100', source: 'Firewall logs', confidence: 'high', firstSeen: new Date() }
  ],
  impactAssessment: 'High - critical production systems affected'
});
```

### 2. Containment

```typescript
// Execute containment strategy
await incidentResponse.executeContainment(incident.id, {
  strategy: 'isolation',
  description: 'Isolated affected servers from network',
  implementedBy: 'Network Engineer',
  affectedSystems: ['FS-PROD-01', 'FS-PROD-02']
});

await incidentResponse.executeContainment(incident.id, {
  strategy: 'credential_reset',
  description: 'Reset all domain admin credentials',
  implementedBy: 'System Administrator',
  affectedSystems: ['AD-DC-01']
});
```

### 3. Evidence Collection

```typescript
// Collect forensic evidence
await incidentResponse.collectEvidence(incident.id, {
  type: 'disk_image',
  description: 'Full disk image of FS-PROD-01',
  collectedBy: 'Forensic Investigator',
  location: '/evidence/fs-prod-01-20250127.dd',
  hash: 'sha256:abc123...',
  size: 524288000000 // 500GB
});

await incidentResponse.collectEvidence(incident.id, {
  type: 'network_logs',
  description: 'Firewall logs for past 7 days',
  collectedBy: 'Security Analyst',
  location: '/evidence/firewall-logs.tar.gz',
  hash: 'sha256:def456...'
});
```

### 4. Eradication & Recovery

```typescript
// Execute eradication
const eradication = await incidentResponse.executeEradication(incident.id, {
  action: 'rebuild_system',
  description: 'Rebuilding FS-PROD-01 from clean backup',
  implementedBy: 'System Administrator'
});

// Verify eradication
await incidentResponse.verifyEradication(incident.id, eradication.id, {
  verifiedBy: 'Security Analyst',
  success: true
});
```

### 5. Communication

```typescript
// Send incident communication
await incidentResponse.sendCommunication(incident.id, {
  priority: 'immediate',
  recipients: ['ciso@acme.com', 'ceo@acme.com'],
  channel: 'email',
  subject: 'CRITICAL: Ransomware Incident Update',
  message: 'Containment complete. Systems isolated. Recovery in progress.',
  sentBy: 'Incident Manager',
  escalation: true
});
```

### 6. Post-Incident Review

```typescript
// Conduct post-incident review
const review = await incidentResponse.conductPostIncidentReview(incident.id, {
  participants: ['Alice Johnson', 'Bob Smith', 'CISO'],
  rootCause: 'User clicked phishing link, leading to credential compromise',
  lessonsLearned: [
    'Need enhanced email filtering',
    'Require MFA for all admin accounts',
    'Improve employee security awareness training'
  ],
  improvements: [
    {
      category: 'technology',
      priority: 'high',
      description: 'Implement advanced email threat protection',
      owner: 'Security Team',
      dueDate: new Date('2025-02-15'),
      status: 'proposed'
    },
    {
      category: 'process',
      priority: 'high',
      description: 'Mandate MFA for all privileged accounts',
      owner: 'IT Operations',
      dueDate: new Date('2025-02-01'),
      status: 'approved'
    }
  ]
});

console.log(`Detection time: ${review.successMetrics.detectionTime} minutes`);
console.log(`Response time: ${review.successMetrics.responseTime} minutes`);
console.log(`Containment time: ${review.successMetrics.containmentTime} minutes`);

// Close the incident
await incidentResponse.closeIncident(incident.id, 'Incident Manager');
```

## Event Handling

```typescript
import { WIAIncidentEventType } from '@wia/security-incident-response';

// Listen for incident events
incidentResponse.on(WIAIncidentEventType.INCIDENT_CREATED, (event) => {
  console.log(`New incident: ${event.data.id} - ${event.data.title}`);
  // Trigger automated workflows, send notifications, etc.
});

incidentResponse.on(WIAIncidentEventType.INCIDENT_ESCALATED, (event) => {
  console.log(`Incident escalated: ${event.data.id}`);
  // Notify senior management
});

incidentResponse.on(WIAIncidentEventType.CONTAINMENT_EXECUTED, (event) => {
  console.log(`Containment executed: ${event.data.action.strategy}`);
  // Update dashboards, alert team
});
```

## Reporting & Analytics

```typescript
// Get incident statistics
const stats = incidentResponse.getStatistics({
  start: new Date('2025-01-01'),
  end: new Date('2025-01-31')
});

console.log(`Total incidents: ${stats.totalIncidents}`);
console.log(`Critical incidents: ${stats.bySeverity.critical}`);
console.log(`Average response time: ${stats.averageResponseTime} minutes`);
console.log(`Average containment time: ${stats.averageContainmentTime} minutes`);

// List active incidents
const activeIncidents = incidentResponse.listIncidents({
  state: 'analyzing'
});

// List critical incidents
const criticalIncidents = incidentResponse.listIncidents({
  severity: 'critical'
});
```

## API Reference

### WIAIncidentResponse Class

#### Detection & Analysis
- `detectIncident(params)` - Detect and create a new security incident
- `triageIncident(incidentId, params)` - Classify and triage an incident
- `analyzeIncident(incidentId, params)` - Perform deep analysis on an incident

#### Containment
- `executeContainment(incidentId, params)` - Execute containment strategy

#### Eradication
- `executeEradication(incidentId, params)` - Execute eradication actions
- `verifyEradication(incidentId, eradicationId, params)` - Verify eradication success

#### Evidence Management
- `collectEvidence(incidentId, params)` - Collect and preserve forensic evidence
- `updateChainOfCustody(incidentId, evidenceId, entry)` - Update chain of custody

#### Communication
- `sendCommunication(incidentId, params)` - Send incident communication

#### Post-Incident
- `conductPostIncidentReview(incidentId, params)` - Conduct post-incident review
- `closeIncident(incidentId, closedBy)` - Close an incident

#### Query & Reporting
- `getIncident(incidentId)` - Get incident by ID
- `listIncidents(filter?)` - List all incidents with optional filtering
- `getStatistics(period)` - Get incident statistics for a time period

#### Event Handling
- `on(eventType, handler)` - Register event handler
- `off(eventType, handler)` - Remove event handler

## TypeScript Types

All types are exported from the package:

```typescript
import {
  SecurityIncident,
  IncidentType,
  IncidentSeverity,
  IncidentState,
  ResponseRole,
  ContainmentStrategy,
  EvidenceType,
  // ... and many more
} from '@wia/security-incident-response';
```

## Best Practices

1. **Always collect evidence early** - Evidence can be lost or tampered with
2. **Document everything** - Use the timeline feature extensively
3. **Follow the chain of custody** - Critical for legal proceedings
4. **Communicate proactively** - Keep stakeholders informed
5. **Conduct post-incident reviews** - Learn from every incident
6. **Automate where possible** - Use event handlers for automation
7. **Practice incident response** - Regular tabletop exercises

## NIST/SANS Framework Alignment

This SDK implements:
- **NIST SP 800-61 Rev. 2** - Computer Security Incident Handling Guide
- **SANS Incident Response** - Six-step process

### NIST Phases
1. ✅ Preparation (Configuration & Team Setup)
2. ✅ Detection & Analysis (detectIncident, triageIncident, analyzeIncident)
3. ✅ Containment, Eradication & Recovery (executeContainment, executeEradication)
4. ✅ Post-Incident Activity (conductPostIncidentReview)

## License

MIT

## Support

For issues and questions:
- GitHub: https://github.com/WIA-Official/wia-standards
- Issues: https://github.com/WIA-Official/wia-standards/issues

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
