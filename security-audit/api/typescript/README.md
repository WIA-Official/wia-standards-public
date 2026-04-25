# @wia/security-audit

> 弘益人間 (홍익인간) - Benefit All Humanity

WIA Security Audit Standard - Comprehensive security audit, compliance, and vulnerability management SDK

## Installation

```bash
npm install @wia/security-audit
```

## Quick Start

```typescript
import { createSecurityAudit } from '@wia/security-audit';

// Create SDK instance
const audit = createSecurityAudit({
  enableLogging: true,
  defaultUser: 'admin@example.com'
});

// Create a new security audit
const myAudit = await audit.createAudit({
  name: 'Q4 2025 Security Audit',
  description: 'Comprehensive security assessment of production environment',
  type: 'internal-audit',
  scope: {
    id: 'scope-1',
    name: 'Production Environment',
    description: 'All production systems and applications',
    targets: [
      {
        id: 'target-1',
        type: 'web-application',
        name: 'Main Web App',
        description: 'Customer-facing web application',
        url: 'https://app.example.com',
        environment: 'production',
        criticality: 'critical',
        metadata: {}
      }
    ],
    exclusions: ['Internal tools', 'Dev environments'],
    frameworks: ['ISO-27001', 'SOC2'],
    startDate: new Date('2025-01-01'),
    endDate: new Date('2025-01-31'),
    objectives: ['Identify security vulnerabilities', 'Assess compliance'],
    constraints: ['No production downtime'],
    assumptions: ['Full access to systems']
  },
  createdBy: 'auditor@example.com'
});

// Start the audit
await audit.startAudit(myAudit.id, 'auditor@example.com');

// Create a security finding
const finding = await audit.createFinding({
  auditId: myAudit.id,
  type: 'vulnerability',
  title: 'SQL Injection in Login Form',
  description: 'User input is not properly sanitized in the login endpoint',
  severity: 'critical',
  category: 'injection',
  location: {
    type: 'code',
    path: '/src/auth/login.ts',
    lineNumber: 45,
    component: 'LoginController'
  },
  recommendation: 'Use parameterized queries or ORM',
  affectedComponents: ['Authentication Service'],
  discoveredBy: 'security-scanner@example.com'
});

// Collect evidence
await audit.collectEvidence(
  finding.id,
  {
    id: 'evidence-1',
    type: 'screenshot',
    title: 'SQL Injection Proof',
    description: 'Screenshot showing successful SQL injection',
    screenshot: 'https://example.com/evidence/screenshot1.png',
    collectedAt: new Date(),
    collectedBy: 'auditor@example.com',
    metadata: {}
  },
  'auditor@example.com'
);

// Create remediation plan
const remediationPlan = await audit.createRemediationPlan({
  findingId: finding.id,
  title: 'Fix SQL Injection Vulnerability',
  description: 'Implement parameterized queries in login endpoint',
  steps: [
    {
      id: 'step-1',
      stepNumber: 1,
      description: 'Review affected code',
      status: 'pending'
    },
    {
      id: 'step-2',
      stepNumber: 2,
      description: 'Implement parameterized queries',
      status: 'pending'
    },
    {
      id: 'step-3',
      stepNumber: 3,
      description: 'Test the fix',
      status: 'pending'
    },
    {
      id: 'step-4',
      stepNumber: 4,
      description: 'Deploy to production',
      status: 'pending'
    }
  ],
  assignedTo: 'developer@example.com',
  priority: 'critical',
  effort: 'medium',
  estimatedHours: 8,
  dueDate: new Date('2025-01-15')
});

// Perform compliance check
await audit.performComplianceCheck({
  auditId: myAudit.id,
  framework: 'ISO-27001',
  controlId: 'A.9.4.1',
  controlTitle: 'Information access restriction',
  controlDescription: 'Access to information and application system functions shall be restricted',
  status: 'compliant',
  assessedBy: 'compliance@example.com',
  notes: 'Access controls properly implemented'
});

// Generate compliance report
const complianceReport = await audit.generateComplianceReport(
  myAudit.id,
  'ISO-27001'
);

console.log(`Compliance Score: ${complianceReport.overallScore}%`);

// Generate audit report
const report = await audit.generateReport({
  auditId: myAudit.id,
  type: 'full-audit',
  title: 'Q4 2025 Security Audit Report',
  generatedBy: 'auditor@example.com',
  format: 'json',
  includeFindings: true,
  includeCompliance: true,
  complianceFramework: 'ISO-27001'
});

// Complete the audit
await audit.completeAudit(myAudit.id, 'auditor@example.com');
```

## Features

### Audit Management
- Create and manage security audits
- Define audit scope and criteria
- Track audit lifecycle (planned, in-progress, completed)
- Support multiple audit types (internal, external, penetration test, etc.)

### Finding Management
- Create and track security findings
- Categorize findings by severity and type
- Assign findings to team members
- Track finding status (open, in-progress, resolved, etc.)
- Filter findings by severity, status, and category

### Evidence Collection
- Attach evidence to findings
- Support multiple evidence types (screenshots, logs, code snippets, etc.)
- Store and organize audit artifacts
- Maintain evidence chain of custody

### Compliance Checking
- Perform compliance checks against multiple frameworks
- Support ISO-27001, SOC2, PCI-DSS, HIPAA, GDPR, and more
- Generate compliance reports
- Track compliance status and scores

### Remediation Tracking
- Create remediation plans for findings
- Break down remediation into steps
- Track remediation progress
- Set priorities and due dates
- Monitor blockers and updates

### Report Generation
- Generate comprehensive audit reports
- Multiple report types (executive summary, technical details, compliance, etc.)
- Calculate audit statistics and risk scores
- Export reports in multiple formats (JSON, PDF, HTML, Markdown)

### Event Handling
- Subscribe to audit events
- Real-time notifications for audit activities
- Custom event handlers
- Audit trail and logging

## API Reference

### Class: WIASecurityAudit

#### Audit Planning and Scoping

- `createAudit(params)` - Create a new security audit
- `startAudit(auditId, startedBy)` - Start an audit
- `completeAudit(auditId, completedBy)` - Complete an audit
- `updateAuditScope(auditId, scope, updatedBy)` - Update audit scope
- `addAuditCriteria(auditId, criteria)` - Add audit criteria

#### Finding Management

- `createFinding(params)` - Create a security finding
- `updateFindingStatus(findingId, status, updatedBy, notes?)` - Update finding status
- `assignFinding(findingId, assignedTo, dueDate?)` - Assign finding to team member
- `getFindingsBySeverity(auditId, severity)` - Get findings by severity
- `getFindingsByStatus(auditId, status)` - Get findings by status

#### Evidence Collection

- `collectEvidence(findingId, evidence, collectedBy)` - Collect and attach evidence
- `addArtifact(auditId, artifact)` - Add artifact to audit
- `getEvidence(findingId)` - Get all evidence for a finding

#### Compliance Checking

- `performComplianceCheck(params)` - Perform compliance check
- `generateComplianceReport(auditId, framework)` - Generate compliance report
- `getComplianceStatus(auditId)` - Get compliance status summary

#### Report Generation

- `generateReport(params)` - Generate audit report
- `calculateStatistics(auditId)` - Calculate audit statistics

#### Remediation Tracking

- `createRemediationPlan(params)` - Create remediation plan for a finding
- `updateRemediationStatus(planId, status, updatedBy)` - Update remediation plan status
- `getRemediationTracking(planId)` - Get remediation tracking status

#### Event Handling

- `on(eventType, handler)` - Register event handler
- `off(eventType, handler)` - Remove event handler

#### Utility Methods

- `getAuditLogs(auditId, limit?)` - Get audit logs
- `exportAudit(auditId)` - Export audit data

## Event Types

- `audit-created` - Audit created
- `audit-started` - Audit started
- `audit-completed` - Audit completed
- `finding-created` - Finding created
- `finding-updated` - Finding updated
- `evidence-added` - Evidence added
- `compliance-checked` - Compliance check performed
- `report-generated` - Report generated
- `remediation-updated` - Remediation plan updated

## Type Definitions

See [types.ts](./src/types.ts) for complete type definitions including:

- Security audit types
- Finding and vulnerability types
- Compliance check types
- Evidence and artifact types
- Remediation tracking types
- Report generation types
- Event types

## Best Practices

### 1. Comprehensive Evidence Collection
Always collect sufficient evidence to support your findings:

```typescript
await audit.collectEvidence(finding.id, {
  id: 'evidence-1',
  type: 'screenshot',
  title: 'Vulnerability Proof',
  description: 'Screenshot demonstrating the vulnerability',
  screenshot: 'https://example.com/evidence.png',
  collectedAt: new Date(),
  collectedBy: 'auditor@example.com',
  hash: 'sha256:abc123...',
  metadata: { tool: 'Burp Suite', version: '2023.1' }
}, 'auditor@example.com');
```

### 2. Detailed Finding Documentation
Provide clear and actionable findings:

```typescript
await audit.createFinding({
  auditId: myAudit.id,
  type: 'vulnerability',
  title: 'Clear and descriptive title',
  description: 'Detailed description of the issue, how it was discovered, and potential impact',
  severity: 'high',
  category: 'authentication',
  location: {
    type: 'code',
    path: '/src/auth/login.ts',
    lineNumber: 45
  },
  recommendation: 'Specific, actionable recommendation for remediation',
  affectedComponents: ['Auth Service', 'User API'],
  discoveredBy: 'security@example.com'
});
```

### 3. Structured Remediation Plans
Break down remediation into clear, manageable steps:

```typescript
await audit.createRemediationPlan({
  findingId: finding.id,
  title: 'Fix Authentication Bypass',
  description: 'Implement proper session validation',
  steps: [
    { id: '1', stepNumber: 1, description: 'Review session handling code', status: 'pending' },
    { id: '2', stepNumber: 2, description: 'Implement session validation', status: 'pending' },
    { id: '3', stepNumber: 3, description: 'Add unit tests', status: 'pending' },
    { id: '4', stepNumber: 4, description: 'Security testing', status: 'pending' },
    { id: '5', stepNumber: 5, description: 'Deploy to production', status: 'pending' }
  ],
  assignedTo: 'dev@example.com',
  priority: 'high',
  effort: 'medium',
  dueDate: new Date('2025-02-01')
});
```

### 4. Regular Compliance Checks
Perform compliance checks throughout the audit:

```typescript
// Check multiple controls
for (const control of controls) {
  await audit.performComplianceCheck({
    auditId: myAudit.id,
    framework: 'ISO-27001',
    controlId: control.id,
    controlTitle: control.title,
    controlDescription: control.description,
    status: assessControlCompliance(control),
    assessedBy: 'compliance@example.com'
  });
}
```

### 5. Event-Driven Workflows
Use event handlers for notifications and automation:

```typescript
audit.on('finding-created', async (event) => {
  const finding = event.data;
  if (finding.severity === 'critical') {
    await notifySecurityTeam(finding);
    await createJiraTicket(finding);
  }
});

audit.on('remediation-updated', async (event) => {
  const plan = event.data;
  if (plan.status === 'completed') {
    await verifyRemediation(plan);
    await updateFindingStatus(plan.findingId, 'resolved');
  }
});
```

## Contributing

Contributions are welcome! Please see the [WIA Standards Repository](https://github.com/WIA-Official/wia-standards) for contribution guidelines.

## License

MIT

## About WIA

World Certification Industry Association (WIA) - Establishing global standards for security, compliance, and best practices.

弘益人間 (홍익인간) - Benefit All Humanity

---

© 2025 SmileStory Inc. / WIA
