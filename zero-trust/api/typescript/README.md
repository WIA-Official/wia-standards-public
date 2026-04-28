# WIA Zero Trust Security SDK

> Never Trust, Always Verify

Philosophy: **弘益人間 (홍익인간)** - Benefit All Humanity

## Overview

The WIA Zero Trust Security SDK provides a comprehensive implementation of Zero Trust Architecture principles. This SDK enables organizations to implement "never trust, always verify" security controls across their infrastructure.

## Core Principles

1. **Verify explicitly** - Always authenticate and authorize based on all available data points
2. **Use least privilege access** - Limit user access with Just-In-Time and Just-Enough-Access (JIT/JEA)
3. **Assume breach** - Minimize blast radius and segment access. Verify end-to-end encryption

## Installation

```bash
npm install @wia/zero-trust
```

## Quick Start

```typescript
import { createZeroTrust, WIAIdentity, WIADevice, Resource } from '@wia/zero-trust';

// Initialize Zero Trust system
const zeroTrust = createZeroTrust({
  strictMode: true,
  defaultDeny: true,
  continuousVerification: true,
  minimumTrustScore: 70,
});

// Define an identity
const identity: WIAIdentity = {
  id: 'user-123',
  type: 'HUMAN',
  attributes: {
    email: 'user@example.com',
    organizationId: 'org-456',
    role: 'developer',
    lastVerified: new Date(),
    attributes: {},
  },
  credentials: [
    {
      type: 'PASSWORD',
      value: 'hashed-password',
      issuedAt: new Date(),
      issuer: 'auth-system',
      verified: true,
    },
    {
      type: 'MFA_TOKEN',
      value: 'totp-token',
      issuedAt: new Date(),
      issuer: 'mfa-provider',
      verified: true,
    },
  ],
  trustScore: 85,
  verificationLevel: 'MEDIUM',
  metadata: {},
};

// Verify identity
const verificationLevel = await zeroTrust.verifyIdentity(identity);
console.log('Verification Level:', verificationLevel);

// Define a device
const device: WIADevice = {
  id: 'device-789',
  type: 'LAPTOP',
  posture: {
    osVersion: 'macOS 14.2',
    osPatched: true,
    antivirusEnabled: true,
    antivirusUpdated: true,
    encryptionEnabled: true,
    firewallEnabled: true,
    unauthorizedSoftware: [],
    securityAgentRunning: true,
    lastBootTime: new Date(),
  },
  trustScore: 90,
  location: {
    ipAddress: '192.168.1.100',
    networkType: 'CORPORATE',
    isKnownLocation: true,
    isCorporateNetwork: true,
  },
  compliance: {
    isCompliant: true,
    violations: [],
    lastChecked: new Date(),
    policies: ['baseline-security'],
  },
  lastAssessed: new Date(),
  metadata: {},
};

// Assess device posture
const assessedDevice = await zeroTrust.assessDevicePosture(device);
console.log('Device Trust Score:', assessedDevice.trustScore);

// Define a resource
const resource: Resource = {
  type: 'API',
  id: 'api-sensitive-data',
  path: '/api/sensitive/data',
  sensitivity: 'CONFIDENTIAL',
};

// Evaluate access
const decision = await zeroTrust.evaluateAccess(identity, device, resource);
console.log('Access Decision:', decision.decision);
console.log('Reason:', decision.reason);
```

## Features

### 1. Identity Verification

```typescript
// Multi-factor authentication
const verificationLevel = await zeroTrust.verifyIdentity(identity);

// Calculate trust score
const trustScore = zeroTrust.calculateIdentityTrustScore(identity);
```

### 2. Device Posture Assessment

```typescript
// Assess device security posture
const device = await zeroTrust.assessDevicePosture(device);

// Calculate device trust score
const trustScore = zeroTrust.calculateDeviceTrustScore(device);
```

### 3. Policy-Based Access Control

```typescript
import { WIAAccessPolicy } from '@wia/zero-trust';

// Define access policy
const policy: WIAAccessPolicy = {
  id: 'policy-001',
  name: 'Sensitive Data Access',
  description: 'Policy for accessing sensitive customer data',
  resources: [
    {
      type: 'API',
      id: 'api-sensitive-data',
      sensitivity: 'CONFIDENTIAL',
    },
  ],
  subjects: [
    {
      type: 'IDENTITY',
      id: 'user-123',
    },
  ],
  conditions: [
    {
      type: 'DEVICE_TRUST',
      parameters: { minimumScore: 80 },
      required: true,
    },
    {
      type: 'MFA_REQUIRED',
      parameters: {},
      required: true,
    },
  ],
  permissions: [
    {
      action: 'read',
      effect: 'ALLOW',
    },
  ],
  leastPrivilege: {
    enabled: true,
    justInTimeAccess: true,
    autoRevoke: true,
    reviewPeriod: 90,
  },
  ttl: 3600,
};

// Add policy
zeroTrust.addPolicy(policy);

// Evaluate access
const decision = await zeroTrust.evaluateAccess(identity, device, resource);
```

### 4. Continuous Authentication

```typescript
// Start continuous auth session
const session = await zeroTrust.startContinuousAuth(identity, device);

// Monitor session for anomalies
const anomalies = await zeroTrust.monitorSession(session.sessionId);

if (anomalies.length > 0) {
  console.log('Anomalies detected:', anomalies);
}
```

### 5. Network Segmentation

```typescript
import { WIANetworkSegment } from '@wia/zero-trust';

// Define network segment
const segment: WIANetworkSegment = {
  id: 'segment-dmz',
  name: 'DMZ Zone',
  type: 'DMZ',
  trustLevel: 'LOW',
  allowedIdentities: ['user-123'],
  allowedDevices: ['device-789'],
  policies: [
    {
      id: 'policy-dmz-001',
      name: 'DMZ Access Policy',
      rules: [
        {
          id: 'rule-001',
          source: { type: 'SEGMENT', value: 'segment-internal' },
          destination: { type: 'SEGMENT', value: 'segment-dmz' },
          protocol: 'HTTPS',
          port: 443,
          action: 'ALLOW',
          conditions: [],
        },
      ],
      enforcement: 'ENFORCE',
    },
  ],
  microsegments: [],
};

// Add segment
zeroTrust.addNetworkSegment(segment);

// Evaluate network access
const allowed = zeroTrust.evaluateNetworkAccess(
  'segment-internal',
  'segment-dmz',
  identity
);
```

### 6. Threat Detection

```typescript
// Detect threats
const threats = await zeroTrust.detectThreats(identity, device);

if (threats.length > 0) {
  console.log('Threats detected:', threats);
}
```

### 7. Event Handling

```typescript
import { EventType } from '@wia/zero-trust';

// Listen for access events
zeroTrust.on(EventType.ACCESS_GRANTED, (event) => {
  console.log('Access granted:', event);
});

zeroTrust.on(EventType.ACCESS_DENIED, (event) => {
  console.log('Access denied:', event);
});

zeroTrust.on(EventType.THREAT_DETECTED, (event) => {
  console.log('Threat detected:', event);
  // Trigger incident response
});

zeroTrust.on(EventType.ANOMALY_DETECTED, (event) => {
  console.log('Anomaly detected:', event);
  // Re-authenticate user
});
```

## Configuration

```typescript
const config = {
  // Strict mode enforces all security checks
  strictMode: true,

  // Default deny when no policy matches
  defaultDeny: true,

  // Continuously verify identity and device
  continuousVerification: true,

  // Check device posture on each request
  devicePostureChecks: true,

  // Enable micro-segmentation
  microsegmentation: true,

  // Require encryption in transit
  encryptionInTransit: true,

  // Require encryption at rest
  encryptionAtRest: true,

  // Enable audit logging
  auditLogging: true,

  // Enable threat detection
  threatDetection: true,

  // Minimum trust score (0-100)
  minimumTrustScore: 70,

  // Session timeout in seconds
  sessionTimeout: 3600, // 1 hour

  // Re-authentication interval in seconds
  reAuthInterval: 900, // 15 minutes
};

const zeroTrust = createZeroTrust(config);
```

## Architecture

The WIA Zero Trust SDK implements the following architecture:

```
┌─────────────────────────────────────────────────────────────┐
│                    Zero Trust Control Plane                  │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   Identity   │  │    Device    │  │   Network    │      │
│  │ Verification │  │   Posture    │  │ Segmentation │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│                                                               │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │    Policy    │  │  Continuous  │  │    Threat    │      │
│  │ Enforcement  │  │     Auth     │  │  Detection   │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

## Best Practices

1. **Always verify explicitly** - Never assume trust based on network location
2. **Use least privilege** - Grant minimum necessary permissions
3. **Monitor continuously** - Detect and respond to anomalies in real-time
4. **Assume breach** - Design systems to limit blast radius
5. **Encrypt everything** - Protect data in transit and at rest
6. **Segment networks** - Use micro-segmentation to isolate workloads
7. **Verify devices** - Ensure device compliance before granting access
8. **Multi-factor authentication** - Require MFA for sensitive operations
9. **Time-bound access** - Use short-lived credentials and JIT access
10. **Audit everything** - Log all access decisions and security events

## API Reference

### Classes

- `WIAZeroTrust` - Main zero trust security class

### Types

- `WIAIdentity` - Identity representation
- `WIADevice` - Device representation
- `WIAAccessPolicy` - Access policy definition
- `WIAAccessDecision` - Access decision result
- `WIAContinuousAuth` - Continuous authentication session
- `WIANetworkSegment` - Network segment definition
- `WIAZeroTrustEvent` - Security event
- `WIAZeroTrustConfig` - Configuration options

See [types.ts](./src/types.ts) for complete type definitions.

## Examples

See the [examples](./examples) directory for more usage examples:

- Basic authentication
- Policy-based access control
- Continuous authentication
- Network segmentation
- Threat detection
- Event handling

## Contributing

Contributions are welcome! Please read our [Contributing Guide](../../CONTRIBUTING.md) for details.

## License

MIT License - see [LICENSE](../../LICENSE) for details.

## Philosophy

**弘益人間 (홍익인간)** - Benefit All Humanity

This SDK is designed to help organizations implement robust security controls that protect user privacy and data while enabling legitimate access to resources.

## Support

- GitHub Issues: https://github.com/WIA-Official/wia-standards/issues
- Documentation: https://wia-official.github.io/wia-standards/
- Email: support@wia.org

---

© 2025 SmileStory Inc. / WIA (World Certification Industry Association)
