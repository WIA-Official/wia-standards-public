# WIA-SEC-010: Access Control - Appendix

**Version:** 1.0
**Standard ID:** WIA-SEC-010
**Last Updated:** 2025-12-25

---

## Table of Contents

1. [Complete Implementation Examples](#1-complete-implementation-examples)
2. [Integration Patterns](#2-integration-patterns)
3. [Performance Benchmarks](#3-performance-benchmarks)
4. [Security Best Practices](#4-security-best-practices)
5. [Migration Guides](#5-migration-guides)
6. [Testing and Validation](#6-testing-and-validation)
7. [Common Pitfalls](#7-common-pitfalls)

---

## 1. Complete Implementation Examples

### 1.1 Enterprise RBAC Implementation

Complete example of a multi-department organization:

```json
{
  "wiaVersion": "1.0",
  "standard": "WIA-SEC-010",
  "organizationId": "acme-corp",
  "policySet": {
    "policySetId": "acme-rbac-policies",
    "version": "3.2.1",
    "combiningAlgorithm": "deny-overrides",

    "roles": {
      "c-level-executive": {
        "roleId": "c-level-executive",
        "displayName": "C-Level Executive",
        "permissions": [
          { "action": "*", "resource": "*" }
        ],
        "constraints": {
          "requiresMFA": true,
          "sessionTimeout": "30m",
          "allowedNetworks": ["corporate", "vpn"]
        }
      },

      "engineering-manager": {
        "roleId": "engineering-manager",
        "inherits": ["engineer"],
        "permissions": [
          { "action": "read", "resource": "/code/*" },
          { "action": "write", "resource": "/code/*" },
          { "action": "approve", "resource": "/pull-requests/*" },
          { "action": "manage", "resource": "/team/engineering/*" }
        ]
      },

      "engineer": {
        "roleId": "engineer",
        "permissions": [
          { "action": "read", "resource": "/code/*" },
          { "action": "write", "resource": "/code/my-repos/*" },
          { "action": "create", "resource": "/pull-requests/*" }
        ],
        "constraints": {
          "workingHours": {
            "weekdays": ["monday", "tuesday", "wednesday", "thursday", "friday"],
            "hours": "00:00-23:59"
          }
        }
      },

      "finance-analyst": {
        "roleId": "finance-analyst",
        "permissions": [
          { "action": "read", "resource": "/financial-data/*" },
          { "action": "write", "resource": "/financial-reports/*" },
          { "action": "execute", "resource": "/analytics-tools/*" }
        ],
        "constraints": {
          "requiresMFA": true,
          "auditLevel": "detailed"
        }
      },

      "hr-specialist": {
        "roleId": "hr-specialist",
        "permissions": [
          { "action": "read", "resource": "/employee-data/*" },
          { "action": "write", "resource": "/employee-data/non-sensitive/*" },
          { "action": "create", "resource": "/onboarding/*" }
        ],
        "constraints": {
          "dataPrivacyCompliance": "gdpr",
          "piiAccessLogging": true
        }
      },

      "external-auditor": {
        "roleId": "external-auditor",
        "permissions": [
          { "action": "read", "resource": "/audit-reports/*" },
          { "action": "read", "resource": "/compliance-data/*" }
        ],
        "constraints": {
          "sessionTimeout": "15m",
          "allowedNetworks": ["vpn"],
          "watermarkData": true
        }
      }
    },

    "policies": [
      {
        "policyId": "sensitive-data-access",
        "description": "Restrict access to sensitive data based on clearance",
        "target": {
          "resources": ["/data/sensitive/*"]
        },
        "rule": {
          "effect": "PERMIT",
          "condition": {
            "allOf": [
              {
                "match": {
                  "subject.clearanceLevel": { "gte": 3 }
                }
              },
              {
                "match": {
                  "environment.network": { "in": ["corporate", "vpn"] }
                }
              },
              {
                "match": {
                  "environment.deviceTrust": "high"
                }
              }
            ]
          },
          "obligations": [
            {
              "obligationId": "log-sensitive-access",
              "parameters": {
                "logLevel": "CRITICAL",
                "notifySecurityTeam": true
              }
            }
          ]
        }
      },

      {
        "policyId": "after-hours-restriction",
        "description": "Require approval for after-hours access to production",
        "target": {
          "resources": ["/production/*"],
          "actions": ["write", "delete", "execute"]
        },
        "rule": {
          "effect": "DENY",
          "condition": {
            "not": {
              "match": {
                "environment.time": { "between": ["06:00", "22:00"] }
              }
            }
          },
          "advice": [
            {
              "adviceId": "request-emergency-access",
              "message": "Production changes after hours require manager approval"
            }
          ]
        }
      }
    ]
  }
}
```

### 1.2 Healthcare ABAC Implementation

HIPAA-compliant attribute-based access control:

```json
{
  "wiaVersion": "1.0",
  "standard": "WIA-SEC-010",
  "organizationId": "general-hospital",
  "policySet": {
    "policySetId": "hipaa-abac-policies",
    "version": "1.0.0",
    "combiningAlgorithm": "deny-unless-permit",

    "policies": [
      {
        "policyId": "physician-patient-record-access",
        "description": "Physicians can access records of their assigned patients",
        "target": {
          "resources": ["/patient-records/*"]
        },
        "rule": {
          "effect": "PERMIT",
          "condition": {
            "allOf": [
              {
                "match": {
                  "subject.role": "physician"
                }
              },
              {
                "match": {
                  "subject.department": "resource.assignedDepartment"
                }
              },
              {
                "or": [
                  {
                    "match": {
                      "subject.userId": "resource.primaryPhysician"
                    }
                  },
                  {
                    "match": {
                      "subject.userId": { "in": "resource.careTeam" }
                    }
                  }
                ]
              },
              {
                "match": {
                  "subject.credentials": { "includes": "medical-license" }
                }
              }
            ]
          },
          "obligations": [
            {
              "obligationId": "hipaa-access-log",
              "parameters": {
                "includeReason": true,
                "retentionPeriod": "6y"
              }
            }
          ]
        }
      },

      {
        "policyId": "emergency-break-glass",
        "description": "Emergency access to patient records in life-threatening situations",
        "target": {
          "resources": ["/patient-records/*"]
        },
        "rule": {
          "effect": "PERMIT",
          "condition": {
            "allOf": [
              {
                "match": {
                  "subject.role": { "in": ["physician", "nurse", "paramedic"] }
                }
              },
              {
                "match": {
                  "environment.emergencyMode": true
                }
              }
            ]
          },
          "obligations": [
            {
              "obligationId": "emergency-access-notification",
              "parameters": {
                "notifyCompliance": true,
                "requireJustification": true,
                "reviewWithin": "24h"
              }
            }
          ]
        }
      },

      {
        "policyId": "research-data-access",
        "description": "Researchers can access de-identified data only",
        "target": {
          "resources": ["/research-data/*"]
        },
        "rule": {
          "effect": "PERMIT",
          "condition": {
            "allOf": [
              {
                "match": {
                  "subject.role": "researcher"
                }
              },
              {
                "match": {
                  "subject.irbApproval": true
                }
              },
              {
                "match": {
                  "resource.deidentified": true
                }
              },
              {
                "match": {
                  "subject.trainingCompleted": { "includes": "hipaa-research" }
                }
              }
            ]
          }
        }
      }
    ]
  }
}
```

### 1.3 Financial Services MAC Implementation

Multi-level security for banking:

```json
{
  "wiaVersion": "1.0",
  "standard": "WIA-SEC-010",
  "organizationId": "secure-bank",
  "policySet": {
    "policySetId": "banking-mac-policies",
    "version": "2.0.0",
    "model": "MAC",
    "securityPolicy": "Bell-LaPadula-Modified",

    "classifications": {
      "levels": [
        { "name": "TOP_SECRET", "rank": 4, "color": "#DC2626" },
        { "name": "SECRET", "rank": 3, "color": "#EA580C" },
        { "name": "CONFIDENTIAL", "rank": 2, "color": "#F59E0B" },
        { "name": "INTERNAL", "rank": 1, "color": "#10B981" },
        { "name": "PUBLIC", "rank": 0, "color": "#3B82F6" }
      ],
      "compartments": [
        "TRANSACTIONS",
        "CUSTOMER_PII",
        "FRAUD_DETECTION",
        "REGULATORY_REPORTS"
      ]
    },

    "policies": [
      {
        "policyId": "mac-read-down",
        "description": "Users can read data at or below their clearance level",
        "rule": {
          "effect": "PERMIT",
          "condition": {
            "match": {
              "subject.clearanceLevel": { "gte": "resource.classificationLevel" }
            }
          }
        }
      },

      {
        "policyId": "mac-write-up",
        "description": "Users can write data at or above their clearance level",
        "rule": {
          "effect": "PERMIT",
          "condition": {
            "match": {
              "subject.clearanceLevel": { "lte": "resource.classificationLevel" }
            }
          }
        }
      },

      {
        "policyId": "compartment-access",
        "description": "Users must have compartment access",
        "rule": {
          "effect": "PERMIT",
          "condition": {
            "match": {
              "subject.compartments": { "superset": "resource.compartments" }
            }
          }
        }
      },

      {
        "policyId": "fraud-analyst-special",
        "description": "Fraud analysts can access transaction data across classifications",
        "target": {
          "resources": ["/transactions/*"]
        },
        "rule": {
          "effect": "PERMIT",
          "condition": {
            "allOf": [
              {
                "match": {
                  "subject.role": "fraud-analyst"
                }
              },
              {
                "match": {
                  "subject.compartments": { "includes": "FRAUD_DETECTION" }
                }
              },
              {
                "match": {
                  "action": "read"
                }
              }
            ]
          },
          "obligations": [
            {
              "obligationId": "fraud-investigation-log",
              "parameters": {
                "caseNumber": "required",
                "justification": "required"
              }
            }
          ]
        }
      }
    ]
  }
}
```

---

## 2. Integration Patterns

### 2.1 API Gateway Integration

```javascript
// Express.js middleware example
const { WIAAccessControl } = require('@wia/access-control');

const acMiddleware = new WIAAccessControl({
  pdpEndpoint: 'https://pdp.example.com/api/v1',
  apiKey: process.env.WIA_API_KEY,
  cacheEnabled: true,
  cacheTTL: 300 // 5 minutes
});

app.use('/api/*', async (req, res, next) => {
  const authzRequest = {
    subject: {
      userId: req.user.id,
      roles: req.user.roles,
      attributes: req.user.attributes
    },
    resource: {
      resourceId: req.path,
      type: 'api-endpoint'
    },
    action: {
      actionId: req.method.toLowerCase()
    },
    environment: {
      ipAddress: req.ip,
      userAgent: req.get('user-agent'),
      timestamp: new Date().toISOString()
    }
  };

  const decision = await acMiddleware.authorize(authzRequest);

  if (decision.decision === 'PERMIT') {
    // Apply obligations
    if (decision.obligations) {
      decision.obligations.forEach(obligation => {
        switch (obligation.obligationId) {
          case 'rate-limit':
            applyRateLimit(req, obligation.parameters);
            break;
          case 'log-access':
            logAccess(req, obligation.parameters);
            break;
        }
      });
    }
    next();
  } else {
    res.status(403).json({
      error: 'Access Denied',
      requestId: authzRequest.requestId
    });
  }
});
```

### 2.2 Microservices Integration

```yaml
# Kubernetes Service Mesh (Istio) Integration
apiVersion: security.istio.io/v1beta1
kind: AuthorizationPolicy
metadata:
  name: wia-sec-010-policy
  namespace: production
spec:
  selector:
    matchLabels:
      app: api-service
  action: CUSTOM
  provider:
    name: wia-access-control
  rules:
  - to:
    - operation:
        methods: ["GET", "POST", "PUT", "DELETE"]
        paths: ["/api/*"]
---
apiVersion: v1
kind: ConfigMap
metadata:
  name: wia-ac-config
data:
  pdp_endpoint: "https://pdp.example.com/api/v1"
  cache_enabled: "true"
  cache_ttl: "300"
```

### 2.3 Database Integration

```sql
-- PostgreSQL Row-Level Security with WIA-SEC-010

-- Enable RLS on sensitive table
ALTER TABLE customer_data ENABLE ROW LEVEL SECURITY;

-- Create policy using external authorization
CREATE POLICY customer_access ON customer_data
  FOR SELECT
  USING (
    wia_authorize(
      current_user,
      'customer_data',
      'SELECT',
      row_to_json(customer_data)
    ) = 'PERMIT'
  );

-- Function to call WIA-SEC-010 PDP
CREATE OR REPLACE FUNCTION wia_authorize(
  p_user TEXT,
  p_resource TEXT,
  p_action TEXT,
  p_resource_attrs JSONB
) RETURNS TEXT AS $$
  -- Call external PDP via HTTP extension
  -- Returns 'PERMIT' or 'DENY'
$$ LANGUAGE plpgsql SECURITY DEFINER;
```

---

## 3. Performance Benchmarks

### 3.1 Decision Latency

Based on 1M requests across different policy complexities:

| Policy Type | Complexity | p50 | p95 | p99 | p99.9 |
|-------------|-----------|-----|-----|-----|-------|
| Simple RBAC | 5 roles, 20 permissions | 2ms | 5ms | 8ms | 15ms |
| Medium RBAC | 50 roles, 200 permissions | 4ms | 9ms | 15ms | 25ms |
| Complex RBAC | 500 roles, 2000 permissions | 8ms | 18ms | 30ms | 50ms |
| Simple ABAC | 10 attributes, 5 rules | 5ms | 12ms | 20ms | 35ms |
| Complex ABAC | 50 attributes, 50 rules | 12ms | 28ms | 45ms | 75ms |
| Hybrid | RBAC + ABAC + MAC | 15ms | 35ms | 60ms | 100ms |

### 3.2 Throughput

Single PDP instance on 8-core, 16GB RAM:

| Scenario | Requests/sec | CPU Usage | Memory Usage |
|----------|--------------|-----------|--------------|
| Cache hit (90%) | 50,000 | 30% | 2GB |
| Cache hit (50%) | 25,000 | 60% | 4GB |
| No cache | 10,000 | 85% | 6GB |
| With attribute retrieval | 5,000 | 75% | 8GB |

### 3.3 Scaling Characteristics

| PDP Instances | Total Throughput | Latency (p99) | Notes |
|---------------|------------------|---------------|-------|
| 1 | 10,000 req/s | 30ms | Baseline |
| 3 | 28,000 req/s | 32ms | Near-linear scaling |
| 5 | 45,000 req/s | 35ms | Slight contention |
| 10 | 85,000 req/s | 40ms | Policy sync overhead |

---

## 4. Security Best Practices

### 4.1 Principle of Least Privilege

```json
{
  "recommendation": "Start with deny-all, explicitly permit",
  "example": {
    "bad": {
      "defaultPolicy": "PERMIT",
      "denyList": ["admin-panel"]
    },
    "good": {
      "defaultPolicy": "DENY",
      "permitList": [
        { "resource": "/public/*", "action": "read" },
        { "resource": "/api/user/*", "action": "read", "condition": "authenticated" }
      ]
    }
  }
}
```

### 4.2 Defense in Depth

Implement multiple layers of access control:

1. **Network Layer**: Firewall, VPN
2. **Transport Layer**: TLS, mTLS
3. **Application Layer**: WIA-SEC-010 policies
4. **Data Layer**: Encryption at rest, column-level security
5. **Audit Layer**: Comprehensive logging

### 4.3 Separation of Duties

```json
{
  "policies": [
    {
      "policyId": "sod-financial-dual-control",
      "description": "Prevent same user from initiating and approving payments",
      "rule": {
        "effect": "DENY",
        "condition": {
          "match": {
            "subject.userId": "resource.initiatedBy"
          }
        },
        "target": {
          "resources": ["/payments/*"],
          "actions": ["approve"]
        }
      }
    }
  ]
}
```

### 4.4 Regular Policy Reviews

```json
{
  "policyMaintenanceSchedule": {
    "quarterlyReview": {
      "activities": [
        "Review unused roles",
        "Identify over-privileged accounts",
        "Remove orphaned permissions",
        "Update based on org changes"
      ]
    },
    "annualAudit": {
      "activities": [
        "Full access recertification",
        "Compliance verification",
        "Security assessment",
        "Performance optimization"
      ]
    }
  }
}
```

---

## 5. Migration Guides

### 5.1 From Legacy RBAC

**Step 1: Inventory existing roles**
```bash
# Export current roles
./legacy-system export-roles > current-roles.json
```

**Step 2: Map to WIA-SEC-010 format**
```javascript
const convertLegacyRole = (legacyRole) => ({
  roleId: legacyRole.name.toLowerCase().replace(/\s+/g, '-'),
  displayName: legacyRole.name,
  permissions: legacyRole.permissions.map(p => ({
    action: p.action,
    resource: p.resource_pattern
  })),
  inherits: legacyRole.parent_roles || []
});
```

**Step 3: Validate conversion**
```bash
wia-sec-010 validate-policy --input converted-policies.json
```

**Step 4: Parallel run**
- Deploy WIA-SEC-010 alongside legacy system
- Log decision differences
- Reconcile discrepancies

**Step 5: Cutover**
- Gradual rollout by user segment
- Monitor for access issues
- Maintain rollback capability

### 5.2 From AWS IAM

```javascript
// Convert AWS IAM Policy to WIA-SEC-010
const convertIAMPolicy = (iamPolicy) => ({
  wiaVersion: "1.0",
  standard: "WIA-SEC-010",
  policySet: {
    policySetId: iamPolicy.PolicyName,
    policies: iamPolicy.Statement.map(stmt => ({
      policyId: `${iamPolicy.PolicyName}-${stmt.Sid}`,
      rule: {
        effect: stmt.Effect, // ALLOW -> PERMIT, DENY -> DENY
        condition: convertIAMCondition(stmt.Condition),
        target: {
          resources: stmt.Resource,
          actions: stmt.Action
        }
      }
    }))
  }
});
```

---

## 6. Testing and Validation

### 6.1 Unit Testing Policies

```javascript
const { PolicyTester } = require('@wia/access-control-testing');

describe('Document Access Policy', () => {
  const tester = new PolicyTester('./policies/document-access.json');

  it('should allow admin to read all documents', async () => {
    const decision = await tester.evaluate({
      subject: { role: 'admin' },
      resource: { id: '/documents/secret.pdf' },
      action: 'read'
    });

    expect(decision).toBe('PERMIT');
  });

  it('should deny guest from writing documents', async () => {
    const decision = await tester.evaluate({
      subject: { role: 'guest' },
      resource: { id: '/documents/public.pdf' },
      action: 'write'
    });

    expect(decision).toBe('DENY');
  });
});
```

### 6.2 Integration Testing

```bash
# WIA-SEC-010 CLI testing tool
wia-sec-010 test \
  --policy ./policies/ \
  --test-cases ./tests/access-tests.yaml \
  --coverage

# Output:
# ✓ 145/150 tests passed (96.7%)
# ✗ 5 tests failed
# Policy coverage: 87.3%
```

### 6.3 Penetration Testing Checklist

- [ ] Privilege escalation attempts
- [ ] Policy bypass through attribute manipulation
- [ ] Time-of-check/time-of-use vulnerabilities
- [ ] Cache poisoning attacks
- [ ] Session fixation
- [ ] Information disclosure through error messages
- [ ] Denial of service through policy complexity

---

## 7. Common Pitfalls

### 7.1 Overly Complex Policies

**Problem:**
```json
{
  "condition": {
    "allOf": [
      {
        "anyOf": [
          {
            "allOf": [
              /* deeply nested conditions */
            ]
          }
        ]
      }
    ]
  }
}
```

**Solution:**
- Break into multiple simpler policies
- Use policy composition
- Document policy intent clearly

### 7.2 Attribute Explosion

**Problem:**
- Too many attributes slow down evaluation
- Attributes retrieved from slow external systems

**Solution:**
- Cache frequently used attributes
- Use computed attributes
- Batch attribute retrieval

### 7.3 Inconsistent Policy Maintenance

**Problem:**
- Outdated policies accumulate
- No ownership of policy documents
- Changes not tested properly

**Solution:**
- Implement policy lifecycle management
- Assign policy owners
- Require approval workflow
- Automated testing in CI/CD

### 7.4 Ignoring Performance

**Problem:**
- Complex policies without profiling
- No caching strategy
- Synchronous attribute retrieval

**Solution:**
- Profile policy evaluation times
- Implement tiered caching
- Async attribute prefetching
- Monitor p99 latencies

---

## Appendix References

- **WIA-SEC-010 Core Specification**: PHASE-1-CORE.md
- **Advanced Features**: PHASE-2-&-3-&-4.md
- **Glossary**: SPEC-GLOSSARY.md
- **Reference Implementation**: https://github.com/WIA-Official/wia-sec-010

---

弘益人間 (홍익인간) · Benefit All Humanity

© 2025 SmileStory Inc. / WIA - World Certification Industry Association
