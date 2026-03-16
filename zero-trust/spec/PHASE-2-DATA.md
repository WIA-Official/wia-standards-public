# WIA-SEC-005: Zero Trust Architecture
## Phase 2 - Data Formats and Schemas

**Standard ID:** WIA-SEC-005
**Category:** Security (SEC)
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-25

---

## 1. Overview

This document defines standardized data formats, schemas, and message structures for Zero Trust Architecture implementations. All data MUST be exchanged in JSON format with UTF-8 encoding unless otherwise specified.

---

## 2. Access Request Format

### 2.1 Access Request Schema

```json
{
  "$schema": "https://wia.org/schemas/zero-trust/access-request/v1.json",
  "requestId": "req-7f3a9b2c-4d5e-6789-abcd-ef0123456789",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00.000Z",
  "subject": {
    "userId": "user-12345",
    "userName": "john.doe@example.com",
    "displayName": "John Doe",
    "userType": "employee|contractor|partner|guest",
    "authentication": {
      "method": "mfa|biometric|password|certificate|fido2",
      "factors": ["password", "totp", "biometric"],
      "timestamp": "2025-12-25T10:29:45.000Z",
      "sessionId": "sess-abc123",
      "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
      "tokenType": "Bearer"
    }
  },
  "device": {
    "deviceId": "device-67890",
    "deviceName": "JohnDoe-Laptop",
    "deviceType": "desktop|laptop|mobile|tablet|server",
    "manufacturer": "Dell",
    "model": "XPS 15",
    "operatingSystem": {
      "name": "Windows|macOS|Linux|iOS|Android",
      "version": "11.0.22621",
      "build": "22621.1234",
      "architecture": "x64|arm64|x86"
    },
    "registrationId": "mdm-device-abc123",
    "managementStatus": "managed|unmanaged|byod"
  },
  "resource": {
    "resourceId": "res-finance-001",
    "resourceType": "application|api|database|file|network",
    "resourceName": "Finance Dashboard",
    "resourcePath": "/apps/finance/dashboard",
    "resourceOwner": "finance-team",
    "classificationLevel": "public|internal|confidential|restricted|secret",
    "requestedActions": ["read", "write", "execute", "delete"],
    "dataClassification": {
      "containsPII": true,
      "containsPHI": false,
      "containsPCI": false,
      "dataResidency": "US|EU|APAC|global"
    }
  },
  "context": {
    "sourceIP": "192.168.1.100",
    "sourcePort": 54321,
    "userAgent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)...",
    "geolocation": {
      "country": "US",
      "region": "CA",
      "city": "San Francisco",
      "coordinates": {
        "latitude": 37.7749,
        "longitude": -122.4194
      },
      "timezone": "America/Los_Angeles"
    },
    "networkType": "corporate-lan|corporate-vpn|public-wifi|mobile|home",
    "vpnActive": true,
    "vpnProvider": "Cisco AnyConnect",
    "tlsVersion": "1.3",
    "cipherSuite": "TLS_AES_256_GCM_SHA384"
  },
  "metadata": {
    "correlationId": "corr-xyz789",
    "requestSource": "web-portal|api|cli|mobile-app",
    "clientVersion": "2.5.0",
    "priority": "low|normal|high|critical"
  }
}
```

### 2.2 Field Requirements

| Field | Required | Validation |
|-------|----------|------------|
| requestId | Yes | UUID v4 format |
| version | Yes | Semantic version (X.Y) |
| timestamp | Yes | ISO 8601 with milliseconds |
| subject.userId | Yes | Non-empty string |
| subject.authentication.method | Yes | Enum value |
| device.deviceId | Yes | Non-empty string |
| resource.resourceId | Yes | Non-empty string |
| resource.requestedActions | Yes | Non-empty array |
| context.sourceIP | Yes | Valid IPv4 or IPv6 |

---

## 3. Access Decision Format

### 3.1 Access Decision Schema

```json
{
  "$schema": "https://wia.org/schemas/zero-trust/access-decision/v1.json",
  "decisionId": "dec-9876543210",
  "requestId": "req-7f3a9b2c-4d5e-6789-abcd-ef0123456789",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00.500Z",
  "decision": "allow|deny|step-up|defer",
  "reason": "Trust score meets threshold for requested resource",
  "trustScore": {
    "overall": 91.5,
    "breakdown": {
      "identity": {
        "score": 95,
        "weight": 0.30,
        "contribution": 28.5,
        "factors": {
          "authenticationStrength": 95,
          "credentialAge": 100,
          "mfaVerified": true
        }
      },
      "device": {
        "score": 92,
        "weight": 0.25,
        "contribution": 23.0,
        "factors": {
          "complianceScore": 95,
          "healthScore": 90,
          "managementStatus": "managed"
        }
      },
      "behavior": {
        "score": 88,
        "weight": 0.25,
        "contribution": 22.0,
        "factors": {
          "baselineDeviation": 12,
          "riskIndicators": 0,
          "historicalScore": 90
        }
      },
      "network": {
        "score": 88,
        "weight": 0.20,
        "contribution": 17.6,
        "factors": {
          "ipReputation": 95,
          "networkTrust": 85,
          "locationTrust": 85
        }
      }
    },
    "riskModifiers": {
      "applied": [
        {
          "factor": "unusual_location",
          "impact": -5,
          "reason": "Access from different city than usual"
        }
      ],
      "totalImpact": -5
    }
  },
  "policies": {
    "evaluated": [
      {
        "policyId": "pol-12345",
        "policyName": "Finance Dashboard Access",
        "matched": true,
        "priority": 100
      }
    ],
    "applied": {
      "policyId": "pol-12345",
      "policyName": "Finance Dashboard Access",
      "version": "2.1"
    }
  },
  "accessGrant": {
    "granted": true,
    "grantedActions": ["read"],
    "deniedActions": ["write", "delete"],
    "conditions": {
      "maxDuration": 3600,
      "expiresAt": "2025-12-25T11:30:00.000Z",
      "renewalAllowed": true,
      "requireContinuousVerification": true,
      "verificationInterval": 900,
      "ipPinning": true,
      "allowedIPs": ["192.168.1.100"],
      "monitoringLevel": "enhanced|standard|maximum"
    },
    "accessToken": {
      "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
      "tokenType": "Bearer",
      "expiresIn": 3600,
      "scope": "finance:read"
    }
  },
  "nextActions": {
    "required": ["continuous_monitoring"],
    "recommended": ["mfa_renewal_in_30min"]
  },
  "auditInfo": {
    "policyEngine": "zt-engine-01",
    "processingTime": 235,
    "cacheHit": false
  }
}
```

### 3.2 Decision Types

| Decision | Description | Next Steps |
|----------|-------------|------------|
| `allow` | Access granted with conditions | Issue access token, start monitoring |
| `deny` | Access denied | Log event, notify user, alert if suspicious |
| `step-up` | Additional authentication required | Request additional factors |
| `defer` | Decision delayed | Perform additional checks, re-evaluate |

---

## 4. Trust Score Data Format

### 4.1 Complete Trust Score Object

```json
{
  "$schema": "https://wia.org/schemas/zero-trust/trust-score/v1.json",
  "scoreId": "score-abc123def456",
  "timestamp": "2025-12-25T10:30:00.000Z",
  "validUntil": "2025-12-25T10:35:00.000Z",
  "subject": {
    "userId": "user-12345",
    "deviceId": "device-67890",
    "sessionId": "sess-abc123"
  },
  "overallScore": 91.5,
  "level": "high|medium|low|critical",
  "components": {
    "identity": {
      "score": 95,
      "weight": 0.30,
      "details": {
        "authenticationMethod": "mfa",
        "authenticationStrength": 95,
        "credentialValidity": true,
        "credentialAge": 45,
        "lastPasswordChange": "2025-11-10T00:00:00Z",
        "mfaFactors": 2,
        "biometricVerified": true,
        "riskSignals": {
          "compromisedCredentials": false,
          "suspiciousActivity": false,
          "accountLocked": false
        }
      }
    },
    "device": {
      "score": 92,
      "weight": 0.25,
      "details": {
        "complianceStatus": "compliant",
        "complianceScore": 95,
        "operatingSystem": {
          "upToDate": true,
          "patchLevel": "current",
          "lastPatchDate": "2025-12-20T00:00:00Z",
          "supportedVersion": true
        },
        "security": {
          "encryptionEnabled": true,
          "antivirusActive": true,
          "antivirusUpdated": true,
          "antivirusLastUpdate": "2025-12-24T06:00:00Z",
          "firewallEnabled": true,
          "screenLockConfigured": true,
          "rootDetected": false,
          "jailbreakDetected": false
        },
        "management": {
          "mdmEnrolled": true,
          "compliancePoliciesApplied": true,
          "remoteWipeEnabled": true,
          "lastCheckIn": "2025-12-25T09:00:00Z"
        },
        "health": {
          "diskEncryption": 100,
          "malwareDetected": false,
          "unusualProcesses": false,
          "networkAnomaly": false
        }
      }
    },
    "behavior": {
      "score": 88,
      "weight": 0.25,
      "details": {
        "baselineEstablished": true,
        "baselineConfidence": 0.95,
        "currentPattern": {
          "accessTime": "10:30",
          "accessDay": "Wednesday",
          "resourcesAccessed": ["finance", "email"],
          "actionsPerformed": ["read"],
          "sessionDuration": 45
        },
        "historicalPattern": {
          "typicalAccessTime": "09:00-18:00",
          "typicalDays": ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday"],
          "typicalResources": ["finance", "email", "crm"],
          "averageSessionDuration": 180
        },
        "anomalyDetection": {
          "anomalyDetected": false,
          "anomalyScore": 12,
          "anomalyReasons": [],
          "mlModelVersion": "2.5.0",
          "mlConfidence": 0.92
        },
        "riskIndicators": {
          "massDownload": false,
          "unusualDataAccess": false,
          "privilegeEscalation": false,
          "suspiciousNavigation": false
        }
      }
    },
    "network": {
      "score": 88,
      "weight": 0.20,
      "details": {
        "sourceIP": "192.168.1.100",
        "ipReputation": {
          "score": 95,
          "clean": true,
          "threatListed": false,
          "categories": []
        },
        "geolocation": {
          "country": "US",
          "city": "San Francisco",
          "expectedLocation": true,
          "distanceFromLastLocation": 5.2,
          "possibleVelocityAnomaly": false
        },
        "networkTrust": {
          "networkType": "corporate-vpn",
          "trustedNetwork": true,
          "vpnActive": true,
          "vpnProvider": "Cisco AnyConnect",
          "tlsVersion": "1.3",
          "certificateValid": true
        }
      }
    }
  },
  "riskAssessment": {
    "overallRisk": "low|medium|high|critical",
    "riskScore": 15,
    "riskFactors": [
      {
        "factor": "unusual_location",
        "severity": "low",
        "impact": -5,
        "description": "Access from different city than usual pattern"
      }
    ],
    "mitigations": [
      {
        "mitigation": "enhanced_monitoring",
        "applied": true
      }
    ]
  },
  "calculation": {
    "algorithm": "weighted-sum-v1",
    "version": "1.0.0",
    "timestamp": "2025-12-25T10:30:00.100Z",
    "processingTime": 87
  }
}
```

---

## 5. Device Compliance Data Format

### 5.1 Device Compliance Report

```json
{
  "$schema": "https://wia.org/schemas/zero-trust/device-compliance/v1.json",
  "reportId": "compliance-device-67890-20251225",
  "deviceId": "device-67890",
  "timestamp": "2025-12-25T10:30:00.000Z",
  "complianceStatus": "compliant|non-compliant|unknown",
  "complianceScore": 95,
  "device": {
    "deviceId": "device-67890",
    "deviceName": "JohnDoe-Laptop",
    "deviceType": "laptop",
    "manufacturer": "Dell",
    "model": "XPS 15",
    "serialNumber": "XXXXXXXXXX",
    "assetTag": "ASSET-12345"
  },
  "operatingSystem": {
    "name": "Windows",
    "version": "11",
    "build": "22621.1234",
    "edition": "Pro",
    "architecture": "x64",
    "installDate": "2024-06-15T00:00:00Z",
    "lastBootTime": "2025-12-24T08:00:00Z",
    "uptime": 90000
  },
  "patchStatus": {
    "compliant": true,
    "lastPatchDate": "2025-12-20T00:00:00Z",
    "daysSinceLastPatch": 5,
    "pendingPatches": 0,
    "criticalPatchesMissing": 0,
    "securityUpdatesEnabled": true,
    "autoUpdateEnabled": true,
    "patchSources": ["Windows Update", "WSUS"]
  },
  "securityFeatures": {
    "encryption": {
      "enabled": true,
      "method": "BitLocker|FileVault|LUKS",
      "algorithm": "AES-256",
      "recoveryKeyBackedUp": true
    },
    "antivirus": {
      "installed": true,
      "active": true,
      "product": "Windows Defender",
      "version": "4.18.23110.2",
      "definitionsVersion": "1.405.123.0",
      "lastUpdate": "2025-12-24T06:00:00Z",
      "lastScan": "2025-12-25T02:00:00Z",
      "threatsDetected": 0,
      "realTimeProtectionEnabled": true
    },
    "firewall": {
      "enabled": true,
      "profiles": {
        "domain": "enabled",
        "private": "enabled",
        "public": "enabled"
      }
    },
    "screenLock": {
      "configured": true,
      "timeout": 300,
      "requirePassword": true,
      "passwordComplexity": true
    },
    "tpm": {
      "present": true,
      "version": "2.0",
      "enabled": true,
      "activated": true
    },
    "secureBootEnabled": true,
    "virtualizationSecurity": {
      "vbsEnabled": true,
      "hvciEnabled": true,
      "credentialGuardEnabled": true
    }
  },
  "management": {
    "mdmEnrolled": true,
    "mdmProvider": "Microsoft Intune",
    "enrollmentDate": "2024-06-15T00:00:00Z",
    "lastCheckIn": "2025-12-25T09:00:00Z",
    "policiesApplied": 15,
    "policiesCompliant": 15,
    "policiesNonCompliant": 0,
    "remoteWipeEnabled": true,
    "locationServicesEnabled": true
  },
  "hardware": {
    "processor": "Intel Core i7-1185G7",
    "memory": 16384,
    "diskSpace": {
      "total": 512000,
      "used": 256000,
      "free": 256000,
      "percentUsed": 50
    },
    "battery": {
      "present": true,
      "status": "charging|discharging|full",
      "percentage": 85,
      "health": "good|fair|poor"
    }
  },
  "network": {
    "interfaces": [
      {
        "name": "Ethernet",
        "type": "wired|wireless",
        "macAddress": "AA:BB:CC:DD:EE:FF",
        "ipv4": "192.168.1.100",
        "ipv6": "fe80::1234:5678:90ab:cdef",
        "connected": true
      }
    ],
    "vpnConfigured": true,
    "proxyConfigured": false
  },
  "applications": {
    "installed": 127,
    "unauthorized": 0,
    "vulnerableApps": 0,
    "criticalApps": [
      {
        "name": "Microsoft Office",
        "version": "16.0.17531",
        "vendor": "Microsoft",
        "installed": "2024-07-01T00:00:00Z",
        "lastUpdated": "2025-12-15T00:00:00Z"
      }
    ]
  },
  "threats": {
    "activeThreat": false,
    "threatsDetected": 0,
    "threatsQuarantined": 0,
    "lastThreatDate": null,
    "riskLevel": "low|medium|high|critical"
  },
  "compliance": {
    "policies": [
      {
        "policyId": "pol-encryption",
        "policyName": "Disk Encryption Required",
        "compliant": true,
        "lastChecked": "2025-12-25T10:30:00Z"
      },
      {
        "policyId": "pol-antivirus",
        "policyName": "Antivirus Required and Updated",
        "compliant": true,
        "lastChecked": "2025-12-25T10:30:00Z"
      },
      {
        "policyId": "pol-patches",
        "policyName": "Security Patches Within 30 Days",
        "compliant": true,
        "lastChecked": "2025-12-25T10:30:00Z"
      }
    ],
    "overallCompliance": true,
    "nonCompliantPolicies": []
  }
}
```

---

## 6. Session Data Format

### 6.1 Active Session Object

```json
{
  "$schema": "https://wia.org/schemas/zero-trust/session/v1.json",
  "sessionId": "sess-abc123def456",
  "userId": "user-12345",
  "deviceId": "device-67890",
  "status": "active|expired|revoked|suspended",
  "lifecycle": {
    "created": "2025-12-25T10:30:00.000Z",
    "lastActivity": "2025-12-25T10:45:00.000Z",
    "expiresAt": "2025-12-25T14:30:00.000Z",
    "duration": 900,
    "idle": 60,
    "maxIdleTime": 900
  },
  "authentication": {
    "method": "mfa",
    "factors": ["password", "totp"],
    "timestamp": "2025-12-25T10:29:45.000Z",
    "ipAddress": "192.168.1.100"
  },
  "context": {
    "current": {
      "ipAddress": "192.168.1.100",
      "geolocation": {
        "country": "US",
        "city": "San Francisco"
      },
      "networkType": "corporate-vpn",
      "userAgent": "Mozilla/5.0..."
    },
    "initial": {
      "ipAddress": "192.168.1.100",
      "geolocation": {
        "country": "US",
        "city": "San Francisco"
      },
      "networkType": "corporate-vpn"
    },
    "changes": []
  },
  "trustScore": {
    "current": 91.5,
    "initial": 92.0,
    "minimum": 90.5,
    "lastCalculated": "2025-12-25T10:45:00.000Z",
    "nextVerification": "2025-12-25T11:00:00.000Z"
  },
  "accessGrants": [
    {
      "resourceId": "res-finance-001",
      "actions": ["read"],
      "grantedAt": "2025-12-25T10:30:00.000Z",
      "expiresAt": "2025-12-25T11:30:00.000Z"
    }
  ],
  "monitoring": {
    "level": "enhanced",
    "anomaliesDetected": 0,
    "warnings": 0,
    "alerts": []
  },
  "auditTrail": {
    "eventsLogged": 23,
    "lastEventTimestamp": "2025-12-25T10:45:00.000Z"
  }
}
```

---

## 7. Event Log Format

### 7.1 Zero Trust Event Log

```json
{
  "$schema": "https://wia.org/schemas/zero-trust/event-log/v1.json",
  "eventId": "evt-9876543210",
  "timestamp": "2025-12-25T10:30:00.123Z",
  "eventType": "access_request|access_decision|authentication|trust_score|policy_change|session_event|security_alert",
  "eventCategory": "security|audit|operational|diagnostic",
  "severity": "info|low|medium|high|critical",
  "source": {
    "component": "policy-engine|authentication-service|trust-calculator|...",
    "instance": "pe-01",
    "version": "2.5.0"
  },
  "subject": {
    "userId": "user-12345",
    "userName": "john.doe@example.com",
    "deviceId": "device-67890",
    "sessionId": "sess-abc123",
    "ipAddress": "192.168.1.100"
  },
  "resource": {
    "resourceId": "res-finance-001",
    "resourceType": "application",
    "resourceName": "Finance Dashboard"
  },
  "action": {
    "type": "access|authenticate|evaluate|change|alert",
    "operation": "read|write|delete|update|...",
    "result": "success|failure|denied|pending"
  },
  "details": {
    "requestId": "req-7f3a9b2c",
    "decisionId": "dec-9876543210",
    "decision": "allow|deny|step-up",
    "trustScore": 91.5,
    "policyId": "pol-12345",
    "reason": "Trust score meets threshold",
    "additionalInfo": {}
  },
  "context": {
    "correlationId": "corr-xyz789",
    "parentEventId": "evt-1234567890",
    "traceId": "trace-abc123",
    "environment": "production|staging|development"
  },
  "compliance": {
    "pii": false,
    "phi": false,
    "pci": false,
    "classification": "internal",
    "retentionPeriod": 365
  }
}
```

---

## 8. Verifiable Credential Format

### 8.1 Zero Trust Verifiable Credential

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wia.org/credentials/zero-trust/v1"
  ],
  "id": "https://wia.org/credentials/zt/12345678-90ab-cdef",
  "type": ["VerifiableCredential", "ZeroTrustCredential"],
  "issuer": {
    "id": "did:wia:issuer:corporate-policy-engine",
    "name": "Corporate Zero Trust Policy Engine"
  },
  "issuanceDate": "2025-12-25T10:30:00Z",
  "expirationDate": "2025-12-25T11:30:00Z",
  "credentialSubject": {
    "id": "did:wia:user:12345",
    "trustScore": {
      "overall": 91.5,
      "identity": 95,
      "device": 92,
      "behavior": 88,
      "network": 88
    },
    "deviceCompliance": {
      "deviceId": "device-67890",
      "compliant": true,
      "osPatched": true,
      "antivirusActive": true,
      "encryptionEnabled": true,
      "mdmEnrolled": true
    },
    "accessGrant": {
      "resources": ["res-finance-001"],
      "actions": ["read"],
      "conditions": {
        "maxDuration": 3600,
        "requireContinuousVerification": true,
        "verificationInterval": 900,
        "allowedNetworks": ["corporate-lan", "corporate-vpn"]
      }
    },
    "riskAssessment": {
      "level": "low",
      "score": 15,
      "factors": ["unusual_location"]
    }
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-12-25T10:30:00Z",
    "verificationMethod": "did:wia:issuer:corporate-policy-engine#key-1",
    "proofPurpose": "assertionMethod",
    "proofValue": "z3FXQsWzGdSJnGEkHF3pzMqxtp8iy1r3..."
  }
}
```

---

## 9. Policy Definition Format

### 9.1 Zero Trust Policy Schema

```json
{
  "$schema": "https://wia.org/schemas/zero-trust/policy/v1.json",
  "policyId": "pol-finance-dashboard-access",
  "version": "2.1.0",
  "name": "Finance Dashboard Access Policy",
  "description": "Controls access to the finance dashboard application",
  "enabled": true,
  "priority": 100,
  "effectiveDate": "2025-01-01T00:00:00Z",
  "expirationDate": null,
  "tags": ["finance", "sensitive-data", "pii"],
  "scope": {
    "resources": [
      {
        "resourceId": "res-finance-001",
        "resourceType": "application",
        "includeSubresources": true
      }
    ],
    "actions": ["read", "write", "execute"]
  },
  "conditions": {
    "subject": {
      "userGroups": ["finance-team", "executives"],
      "userRoles": ["analyst", "manager"],
      "userAttributes": {
        "department": ["finance", "accounting"],
        "clearanceLevel": ["confidential", "secret"],
        "employeeStatus": ["active"]
      },
      "excludeUsers": ["user-blocked-001"]
    },
    "device": {
      "minComplianceScore": 90,
      "allowedDeviceTypes": ["desktop", "laptop"],
      "mdmRequired": true,
      "encryptionRequired": true,
      "antivirusRequired": true,
      "osVersions": {
        "Windows": ["11", "10"],
        "macOS": ["14", "13", "12"]
      }
    },
    "network": {
      "allowedNetworks": ["corporate-lan", "corporate-vpn"],
      "blockedCountries": ["XX", "YY"],
      "requireVPN": true,
      "allowedIPRanges": ["192.168.0.0/16", "10.0.0.0/8"],
      "minNetworkTrustScore": 80
    },
    "time": {
      "allowedHours": "06:00-22:00",
      "allowedDays": ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday"],
      "timezone": "America/Los_Angeles",
      "holidays": {
        "deny": true,
        "calendar": "US-Holidays"
      }
    },
    "context": {
      "minTrustScore": 85,
      "maxRiskScore": 30,
      "requireMFA": true,
      "minAuthenticationFactors": 2,
      "allowedAuthMethods": ["mfa", "biometric", "fido2"]
    }
  },
  "actions": {
    "allow": {
      "decision": "allow",
      "grantedActions": ["read"],
      "maxDuration": 3600,
      "renewalAllowed": true,
      "monitoringLevel": "enhanced",
      "notifications": []
    },
    "deny": {
      "decision": "deny",
      "reason": "Trust score below minimum threshold",
      "notifications": ["user", "security-team"],
      "alerts": ["low-trust-access-attempt"]
    },
    "stepUp": {
      "decision": "step-up",
      "threshold": 75,
      "requiredActions": [
        {
          "type": "additional_mfa",
          "methods": ["biometric", "hardware-token"]
        }
      ],
      "maxAttempts": 3
    }
  },
  "audit": {
    "logAllAttempts": true,
    "logLevel": "detailed",
    "retentionDays": 365,
    "complianceRequirements": ["SOX", "PCI-DSS"]
  },
  "metadata": {
    "owner": "security-team",
    "approver": "ciso@example.com",
    "reviewDate": "2025-06-01",
    "lastModified": "2025-01-15T00:00:00Z",
    "modifiedBy": "admin@example.com",
    "changeReason": "Updated minimum trust score"
  }
}
```

---

## 10. API Response Format

### 10.1 Standard API Response Wrapper

```json
{
  "success": true,
  "timestamp": "2025-12-25T10:30:00.000Z",
  "requestId": "req-7f3a9b2c",
  "data": {
    // Actual response data here
  },
  "metadata": {
    "version": "1.0",
    "processingTime": 125,
    "cached": false
  },
  "pagination": {
    "page": 1,
    "pageSize": 50,
    "totalPages": 10,
    "totalItems": 487
  },
  "links": {
    "self": "https://api.wia.org/v1/access-requests/req-7f3a9b2c",
    "next": "https://api.wia.org/v1/access-requests?page=2",
    "prev": null
  }
}
```

### 10.2 Error Response Format

```json
{
  "success": false,
  "timestamp": "2025-12-25T10:30:00.000Z",
  "requestId": "req-7f3a9b2c",
  "error": {
    "code": "INSUFFICIENT_TRUST_SCORE",
    "message": "Access denied: Trust score below minimum threshold",
    "details": {
      "currentTrustScore": 65,
      "requiredTrustScore": 85,
      "factors": [
        "Unusual location detected",
        "Device compliance score low"
      ]
    },
    "remediation": [
      "Ensure device is fully compliant",
      "Complete additional MFA challenge",
      "Contact IT support if issue persists"
    ]
  },
  "metadata": {
    "version": "1.0",
    "processingTime": 87
  }
}
```

---

## 11. Data Validation Rules

### 11.1 Common Validation Rules

| Field Type | Validation Rule |
|------------|----------------|
| UUID | RFC 4122 format |
| Timestamp | ISO 8601 with timezone |
| Email | RFC 5322 format |
| IP Address | Valid IPv4 or IPv6 |
| Score | Number 0-100 |
| Trust Level | Enum: high\|medium\|low\|critical |
| Decision | Enum: allow\|deny\|step-up\|defer |

### 11.2 Business Logic Validation

```javascript
// Trust score must be between 0 and 100
if (trustScore < 0 || trustScore > 100) {
    throw new ValidationError("Trust score must be between 0 and 100");
}

// Access duration cannot exceed 24 hours
if (duration > 86400) {
    throw new ValidationError("Maximum access duration is 24 hours");
}

// At least one action must be requested
if (requestedActions.length === 0) {
    throw new ValidationError("At least one action must be requested");
}

// Expiration date must be in the future
if (new Date(expirationDate) <= new Date()) {
    throw new ValidationError("Expiration date must be in the future");
}
```

---

## 12. Data Encryption Standards

### 12.1 Encryption Requirements

| Data Type | At Rest | In Transit |
|-----------|---------|------------|
| Access Tokens | AES-256-GCM | TLS 1.3 |
| Trust Scores | AES-256-GCM | TLS 1.3 |
| User Credentials | Argon2id | TLS 1.3 |
| Session Data | AES-256-GCM | TLS 1.3 |
| Audit Logs | AES-256-GCM | TLS 1.3 |
| PII Data | AES-256-GCM + Field-level | TLS 1.3 |

### 12.2 Key Management

- Keys rotated every 90 days
- HSM (Hardware Security Module) for key storage
- Separate keys per environment
- Key escrow for compliance

---

## 13. Schema Versioning

### 13.1 Version Format

All schemas use semantic versioning: `MAJOR.MINOR.PATCH`

- **MAJOR**: Incompatible API changes
- **MINOR**: Backward-compatible functionality
- **PATCH**: Backward-compatible bug fixes

### 13.2 Version Negotiation

```json
{
  "Accept-Version": "1.0",
  "X-WIA-Min-Version": "1.0",
  "X-WIA-Max-Version": "2.0"
}
```

---

**Document Status:** ✅ Complete
**Next Phase:** [PHASE-3-PROTOCOL.md](./PHASE-3-PROTOCOL.md)

---

© 2025 SmileStory Inc. / WIA
**弘益人間 (홍익인간)** - Benefit All Humanity
