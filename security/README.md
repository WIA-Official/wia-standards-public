# WIA Security Standard

**Cybersecurity Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20SECURITY-orange.svg)](https://security.wia.live)

---

<div align="center">

**Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) | [API Portal](https://api.wia.live)

---

**弘益人間** - *Benefit All Humanity*

</div>

---

## Overview

WIA Security is an open standard for cybersecurity event data formats, APIs, and protocols. Built on industry standards including STIX 2.1, OCSF, ECS, and MITRE ATT&CK.

### Key Features

- **Unified Data Format**: Standardized JSON schema for 7 security event types
- **MITRE ATT&CK Integration**: Native support for tactics, techniques, and procedures
- **STIX 2.1 Compatible**: Easy conversion to threat intelligence sharing format
- **Multi-language SDKs**: TypeScript, Python, and Rust implementations
- **SIEM Integration**: Splunk, Elastic, QRadar adapter support

---

## Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | Standard data format for security events | ✅ Complete |
| **2** | API Interface | TypeScript, Python, Rust SDKs | ✅ Complete |
| **3** | Communication Protocol | WebSocket, TAXII 2.1, SIEM Adapters | ✅ Complete |
| **4** | Ecosystem Integration | Threat Intel, SOAR, Cloud Security | ✅ Complete |

---

## Event Types

| Type | Description | Use Case |
|------|-------------|----------|
| `alert` | Security alerts from detection systems | SIEM alerts, EDR detections |
| `threat_intel` | Threat intelligence indicators | IOCs, APT reports, campaigns |
| `vulnerability` | Vulnerability information | CVE data, scan results |
| `incident` | Security incident records | IR case management |
| `network_event` | Network traffic events | Firewall logs, IDS alerts |
| `endpoint_event` | Endpoint activity | Process creation, file changes |
| `auth_event` | Authentication events | Login attempts, MFA events |

---

## Quick Start

### Installation

```bash
# TypeScript
npm install @wia/security

# Python
pip install wia-security

# Rust
cargo add wia-security
```

### Basic Usage

```typescript
import { WiaSecurityEvent, validateEvent } from '@wia/security';

const alert: WiaSecurityEvent = {
  version: '1.0.0',
  id: crypto.randomUUID(),
  type: 'alert',
  timestamp: new Date().toISOString(),
  severity: 7,
  source: { type: 'siem', name: 'Splunk' },
  data: {
    alert_id: 'ALERT-001',
    title: 'Suspicious Activity Detected',
    category: 'malware',
    status: 'new',
    priority: 'high'
  },
  mitre: {
    tactic: 'TA0002',
    technique: 'T1059.001'
  }
};

const result = validateEvent(alert);
console.log(result.valid); // true
```

### Python Example

```python
from wia_security import WiaSecurityEvent, validate_event

event = WiaSecurityEvent(
    version='1.0.0',
    type='alert',
    severity=7,
    source={'type': 'siem', 'name': 'Splunk'},
    data={
        'alert_id': 'ALERT-001',
        'title': 'Suspicious Activity',
        'category': 'malware',
        'status': 'new',
        'priority': 'high'
    }
)

result = validate_event(event)
print(result.valid)  # True
```

---

## Structure

```
security/
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md      # Data format specification
│   ├── RESEARCH-PHASE-1.md         # Research documentation
│   └── schemas/                    # JSON Schema files
│       ├── wia-security-v1.schema.json
│       ├── alert.schema.json
│       ├── threat-intel.schema.json
│       ├── vulnerability.schema.json
│       ├── incident.schema.json
│       ├── network-event.schema.json
│       ├── endpoint-event.schema.json
│       └── auth-event.schema.json
├── api/
│   ├── typescript/                 # TypeScript SDK
│   │   └── src/
│   │       ├── types.ts           # Type definitions
│   │       ├── validator.ts       # Event validation
│   │       ├── builder.ts         # Fluent builder API
│   │       ├── converter.ts       # STIX, ECS, OCSF converters
│   │       ├── client.ts          # SecurityClient
│   │       ├── protocol/          # Phase 3: WebSocket, TAXII, SIEM
│   │       └── integration/       # Phase 4: Threat Intel, SOAR, Cloud
│   ├── python/                     # Python SDK
│   │   └── wia_security/
│   │       ├── types.py
│   │       ├── validator.py
│   │       ├── builder.py
│   │       ├── converter.py
│   │       ├── client.py
│   │       ├── protocol/          # WebSocket, TAXII, SIEM
│   │       └── integration/       # Threat Intel, SOAR, Cloud
│   └── rust/                       # Rust SDK
│       └── src/
│           ├── types.rs
│           ├── validator.rs
│           ├── builder.rs
│           ├── converter.rs
│           └── client.rs
├── examples/
│   ├── sample-data/               # Sample event JSON files
│   └── validators/                # Validation tools
├── prompts/                       # Claude Code prompts
│   ├── PHASE-1-PROMPT.md
│   ├── PHASE-2-PROMPT.md
│   ├── PHASE-3-PROMPT.md
│   └── PHASE-4-PROMPT.md
└── docs/
```

---

## Phase 3: Communication Protocol

### WebSocket Real-time Streaming

```typescript
import { createWebSocketClient, WebSocketConfig } from '@wia/security';

const client = createWebSocketClient({
  url: 'wss://security.example.com/events',
  authToken: 'your-token',
  reconnect: true
});

await client.connect();
client.subscribe('alerts', { severityMin: 7 });
client.on('alert', (event) => console.log('Alert:', event));
```

### TAXII 2.1 Threat Intelligence

```typescript
import { createTaxiiClient } from '@wia/security';

const taxii = createTaxiiClient({
  serverUrl: 'https://taxii.example.com',
  apiRoot: 'api1',
  apiKey: 'your-api-key'
});

const collections = await taxii.getCollections();
const indicators = await taxii.getObjects('collection-id', { type: ['indicator'] });
```

### SIEM Integration

```typescript
import { createSplunkAdapter, createElasticAdapter } from '@wia/security';

// Splunk HEC
const splunk = createSplunkAdapter({
  url: 'https://splunk.example.com:8088',
  token: 'HEC-token',
  index: 'security'
});
await splunk.send(event);

// Elasticsearch
const elastic = createElasticAdapter({
  url: 'https://elastic.example.com:9200',
  apiKey: 'your-api-key',
  indexPrefix: 'wia-security'
});
await elastic.index(event);
```

---

## Phase 4: Ecosystem Integration

### Threat Intelligence Feeds

```typescript
import { createMispClient, createVirusTotalClient } from '@wia/security';

// MISP
const misp = createMispClient({
  url: 'https://misp.example.com',
  apiKey: 'your-key'
});
const feed = await misp.searchEvents({ type: 'ip-src' });

// VirusTotal
const vt = createVirusTotalClient({
  url: 'https://www.virustotal.com/api/v3',
  apiKey: 'your-key'
});
const report = await vt.getFileReport('hash-value');
```

### SOAR Integration

```typescript
import { createPhantomClient, createXsoarClient } from '@wia/security';

// Splunk Phantom
const phantom = createPhantomClient({
  url: 'https://phantom.example.com',
  apiKey: 'your-key'
});
const containerId = await phantom.createContainer(event);
await phantom.runPlaybook('investigate-alert', containerId);

// Palo Alto XSOAR
const xsoar = createXsoarClient({
  url: 'https://xsoar.example.com',
  apiKey: 'your-key'
});
const incidentId = await xsoar.createIncident(event);
```

### Cloud Security Integration

```typescript
import {
  createAwsSecurityHubClient,
  createAzureSentinelClient,
  createGcpSccClient
} from '@wia/security';

// AWS Security Hub
const securityHub = createAwsSecurityHubClient({
  accountId: '123456789012',
  region: 'us-east-1'
});
await securityHub.importFinding(event);

// Azure Sentinel
const sentinel = createAzureSentinelClient({
  workspaceId: 'workspace-id',
  subscriptionId: 'subscription-id',
  resourceGroup: 'rg-security',
  tenantId: 'tenant-id',
  clientId: 'client-id',
  clientSecret: 'client-secret'
});
await sentinel.createIncident(event);

// GCP Security Command Center
const scc = createGcpSccClient({
  organizationId: 'org-id',
  projectId: 'project-id'
});
await scc.createFinding(event);
```

### Dashboard & Compliance

```typescript
import { createDashboardAggregator, createComplianceTracker } from '@wia/security';

// Dashboard aggregation
const dashboard = createDashboardAggregator({ timeWindowHours: 24 });
dashboard.addEvents(events);
const metrics = dashboard.getMetrics();
console.log('Total events:', metrics.totalEvents);
console.log('MTTD:', metrics.meanTimeToDetect);

// Compliance tracking
const compliance = createComplianceTracker();
const controls = compliance.mapEventToControls(event, 'NIST');
const score = compliance.calculateScoreFromEvents(events, 'NIST');
```

---

## Validation

### Command Line

```bash
# TypeScript
npx ts-node validator.ts event.json

# Python
python validator.py event.json
```

### Programmatic

```typescript
import { validateEvent } from '@wia/security';

const result = validateEvent(event);
if (!result.valid) {
  console.error('Errors:', result.errors);
}
```

---

## MITRE ATT&CK Mapping

All security events support MITRE ATT&CK mapping:

```json
{
  "mitre": {
    "tactic": "TA0002",
    "tactic_name": "Execution",
    "technique": "T1059",
    "technique_name": "Command and Scripting Interpreter",
    "sub_technique": "T1059.001",
    "sub_technique_name": "PowerShell"
  }
}
```

---

## Industry Compatibility

| Standard | Compatibility |
|----------|---------------|
| STIX 2.1 | Bidirectional conversion |
| OCSF | Field mapping |
| ECS (Elastic) | Native support |
| MITRE ATT&CK | Full integration |
| CVE/CVSS | Native support |
| Sigma Rules | Detection format |

---

## Links

| Resource | URL |
|----------|-----|
| **Website** | https://security.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/security |
| **API Docs** | https://api.wia.live/security |

---

## Contributing

We welcome contributions! Please see our [Contributing Guide](../CONTRIBUTING.md).

---

## License

MIT License - This standard belongs to humanity.

---

<div align="center">

**弘益人間** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
