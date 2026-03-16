# WIA-SEC-017: Security Audit
## Appendix

**Standard ID:** WIA-SEC-017
**Category:** Security (SEC)
**Version:** 1.0.0
**Last Updated:** 2025-12-25

---

## Appendix A: Code Examples

### A.1 Complete TypeScript SDK

```typescript
// @wia/sec-017-audit SDK

import { EventEmitter } from 'events';
import crypto from 'crypto';

/**
 * Audit event severity levels
 */
export enum AuditSeverity {
  INFO = 'INFO',
  LOW = 'LOW',
  MEDIUM = 'MEDIUM',
  HIGH = 'HIGH',
  CRITICAL = 'CRITICAL'
}

/**
 * Audit event types
 */
export enum AuditEventType {
  AUTHENTICATION = 'AUTHENTICATION',
  AUTHORIZATION = 'AUTHORIZATION',
  DATA_ACCESS = 'DATA_ACCESS',
  DATA_MODIFICATION = 'DATA_MODIFICATION',
  CONFIGURATION_CHANGE = 'CONFIGURATION_CHANGE',
  PRIVILEGE_ESCALATION = 'PRIVILEGE_ESCALATION',
  SECURITY_EVENT = 'SECURITY_EVENT',
  COMPLIANCE_EVENT = 'COMPLIANCE_EVENT',
  SYSTEM_EVENT = 'SYSTEM_EVENT'
}

/**
 * Actor information
 */
export interface AuditActor {
  user_id: string;
  type: 'HUMAN' | 'SERVICE' | 'SYSTEM';
  ip_address: string;
  user_agent?: string;
  session_id?: string;
  location?: {
    country: string;
    region?: string;
    city?: string;
    coordinates?: { lat: number; lon: number };
  };
}

/**
 * Resource information
 */
export interface AuditResource {
  type: string;
  id: string;
  classification: 'PUBLIC' | 'INTERNAL' | 'CONFIDENTIAL' | 'RESTRICTED';
}

/**
 * Action information
 */
export interface AuditAction {
  operation: 'CREATE' | 'READ' | 'UPDATE' | 'DELETE' | 'EXECUTE' | string;
  status: 'SUCCESS' | 'FAILURE' | 'PARTIAL';
  duration_ms?: number;
  result?: string;
}

/**
 * Complete audit entry
 */
export interface AuditEntry {
  audit_id: string;
  timestamp: string;
  event_type: AuditEventType;
  severity: AuditSeverity;
  actor: AuditActor;
  resource: AuditResource;
  action: AuditAction;
  metadata?: {
    compliance_tags?: string[];
    data_classification?: string;
    retention_days?: number;
    custom_fields?: Record<string, any>;
  };
  cryptographic_proof?: {
    hash: string;
    signature: string;
    chain_link: string;
    verification_key_id: string;
  };
}

/**
 * Audit client configuration
 */
export interface AuditClientConfig {
  endpoint: string;
  apiKey: string;
  region?: string;
  compliance?: string[];
  autoFlush?: boolean;
  flushInterval?: number;
  batchSize?: number;
}

/**
 * WIA Security Audit Client
 */
export class AuditClient extends EventEmitter {
  private config: AuditClientConfig;
  private buffer: AuditEntry[] = [];
  private flushTimer?: NodeJS.Timeout;

  constructor(config: AuditClientConfig) {
    super();
    this.config = {
      autoFlush: true,
      flushInterval: 5000,
      batchSize: 100,
      ...config
    };

    if (this.config.autoFlush) {
      this.startAutoFlush();
    }
  }

  /**
   * Log an audit event
   */
  async log(entry: Omit<AuditEntry, 'audit_id' | 'timestamp' | 'cryptographic_proof'>): Promise<string> {
    const auditEntry: AuditEntry = {
      ...entry,
      audit_id: this.generateAuditId(),
      timestamp: new Date().toISOString(),
      metadata: {
        compliance_tags: this.config.compliance || [],
        ...entry.metadata
      }
    };

    // Add to buffer
    this.buffer.push(auditEntry);

    // Emit event
    this.emit('audit_event', auditEntry);

    // Flush if buffer is full
    if (this.buffer.length >= (this.config.batchSize || 100)) {
      await this.flush();
    }

    return auditEntry.audit_id;
  }

  /**
   * Convenience method: Log authentication event
   */
  async logAuthentication(
    userId: string,
    success: boolean,
    options: { ip?: string; location?: any; mfa?: boolean } = {}
  ): Promise<string> {
    return this.log({
      event_type: AuditEventType.AUTHENTICATION,
      severity: success ? AuditSeverity.INFO : AuditSeverity.MEDIUM,
      actor: {
        user_id: userId,
        type: 'HUMAN',
        ip_address: options.ip || 'unknown',
        location: options.location
      },
      resource: {
        type: 'AUTHENTICATION_SYSTEM',
        id: 'auth',
        classification: 'INTERNAL'
      },
      action: {
        operation: options.mfa ? 'MFA_LOGIN' : 'LOGIN',
        status: success ? 'SUCCESS' : 'FAILURE'
      }
    });
  }

  /**
   * Convenience method: Log data access
   */
  async logDataAccess(
    userId: string,
    resourceId: string,
    operation: 'READ' | 'WRITE' | 'DELETE',
    classification: AuditResource['classification']
  ): Promise<string> {
    return this.log({
      event_type: AuditEventType.DATA_ACCESS,
      severity: classification === 'RESTRICTED' || classification === 'CONFIDENTIAL'
        ? AuditSeverity.MEDIUM
        : AuditSeverity.LOW,
      actor: {
        user_id: userId,
        type: 'HUMAN',
        ip_address: 'internal'
      },
      resource: {
        type: 'DATABASE',
        id: resourceId,
        classification
      },
      action: {
        operation,
        status: 'SUCCESS'
      },
      metadata: {
        data_classification: classification
      }
    });
  }

  /**
   * Flush buffered events to server
   */
  async flush(): Promise<void> {
    if (this.buffer.length === 0) return;

    const batch = [...this.buffer];
    this.buffer = [];

    try {
      const response = await fetch(`${this.config.endpoint}/api/v1/audit/submit`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-Audit-Timestamp': new Date().toISOString(),
          'X-Audit-Nonce': crypto.randomBytes(16).toString('hex')
        },
        body: JSON.stringify({
          protocol_version: '1.0',
          audit_batch: batch,
          batch_metadata: {
            total_events: batch.length,
            batch_hash: this.calculateBatchHash(batch)
          }
        })
      });

      if (!response.ok) {
        throw new Error(`Audit submission failed: ${response.statusText}`);
      }

      const result = await response.json();
      this.emit('flush_success', { count: batch.length, receipt: result.receipt_id });
    } catch (error) {
      // Re-add to buffer on failure
      this.buffer.unshift(...batch);
      this.emit('flush_error', error);
      throw error;
    }
  }

  /**
   * Start auto-flush timer
   */
  private startAutoFlush(): void {
    this.flushTimer = setInterval(
      () => this.flush(),
      this.config.flushInterval || 5000
    );
  }

  /**
   * Stop auto-flush and cleanup
   */
  async close(): Promise<void> {
    if (this.flushTimer) {
      clearInterval(this.flushTimer);
    }
    await this.flush();
  }

  /**
   * Generate unique audit ID
   */
  private generateAuditId(): string {
    const date = new Date();
    const year = date.getFullYear();
    const random = Math.floor(Math.random() * 100000000);
    return `AUD-${year}-${random.toString().padStart(8, '0')}`;
  }

  /**
   * Calculate batch hash for integrity
   */
  private calculateBatchHash(batch: AuditEntry[]): string {
    const data = JSON.stringify(batch);
    return crypto.createHash('sha256').update(data).digest('hex');
  }
}

/**
 * Usage example
 */
async function example() {
  const auditClient = new AuditClient({
    endpoint: 'https://audit.example.com',
    apiKey: process.env.WIA_AUDIT_API_KEY!,
    compliance: ['SOC2', 'ISO27001', 'GDPR']
  });

  // Log authentication
  await auditClient.logAuthentication('user@example.com', true, {
    ip: '192.168.1.100',
    mfa: true
  });

  // Log data access
  await auditClient.logDataAccess(
    'user@example.com',
    '/api/customers/123',
    'READ',
    'CONFIDENTIAL'
  );

  // Custom audit event
  await auditClient.log({
    event_type: AuditEventType.CONFIGURATION_CHANGE,
    severity: AuditSeverity.HIGH,
    actor: {
      user_id: 'admin@example.com',
      type: 'HUMAN',
      ip_address: '10.0.1.50'
    },
    resource: {
      type: 'FIREWALL_RULE',
      id: 'fw-rule-123',
      classification: 'RESTRICTED'
    },
    action: {
      operation: 'UPDATE',
      status: 'SUCCESS',
      result: 'Opened port 443'
    }
  });

  // Cleanup
  await auditClient.close();
}
```

---

## Appendix B: Integration Examples

### B.1 Express.js Middleware

```typescript
import { Request, Response, NextFunction } from 'express';
import { AuditClient } from '@wia/sec-017-audit';

export function auditMiddleware(auditClient: AuditClient) {
  return async (req: Request, res: Response, next: NextFunction) => {
    const startTime = Date.now();

    // Capture original end function
    const originalEnd = res.end;

    // Override end function to log after response
    res.end = function(...args: any[]) {
      const duration = Date.now() - startTime;

      // Log audit event
      auditClient.log({
        event_type: 'DATA_ACCESS',
        severity: res.statusCode >= 400 ? 'MEDIUM' : 'LOW',
        actor: {
          user_id: req.user?.id || 'anonymous',
          type: req.user ? 'HUMAN' : 'SYSTEM',
          ip_address: req.ip || req.socket.remoteAddress || 'unknown',
          user_agent: req.get('user-agent'),
          session_id: req.sessionID
        },
        resource: {
          type: 'API_ENDPOINT',
          id: req.path,
          classification: req.path.includes('admin') ? 'RESTRICTED' : 'INTERNAL'
        },
        action: {
          operation: req.method,
          status: res.statusCode < 400 ? 'SUCCESS' : 'FAILURE',
          duration_ms: duration
        }
      }).catch(err => {
        console.error('Audit logging failed:', err);
      });

      // Call original end
      return originalEnd.apply(res, args);
    };

    next();
  };
}

// Usage
import express from 'express';

const app = express();
const audit = new AuditClient({ /* config */ });

app.use(auditMiddleware(audit));
```

### B.2 AWS Lambda Integration

```typescript
import { AuditClient } from '@wia/sec-017-audit';
import { Handler, Context } from 'aws-lambda';

const audit = new AuditClient({
  endpoint: process.env.AUDIT_ENDPOINT!,
  apiKey: process.env.AUDIT_API_KEY!
});

export const handler: Handler = async (event, context: Context) => {
  const startTime = Date.now();

  try {
    // Your Lambda logic here
    const result = await processEvent(event);

    // Log successful execution
    await audit.log({
      event_type: 'SYSTEM_EVENT',
      severity: 'INFO',
      actor: {
        user_id: context.functionName,
        type: 'SYSTEM',
        ip_address: 'lambda'
      },
      resource: {
        type: 'LAMBDA_FUNCTION',
        id: context.functionArn,
        classification: 'INTERNAL'
      },
      action: {
        operation: 'EXECUTE',
        status: 'SUCCESS',
        duration_ms: Date.now() - startTime
      }
    });

    return result;
  } catch (error) {
    // Log failure
    await audit.log({
      event_type: 'SYSTEM_EVENT',
      severity: 'HIGH',
      actor: {
        user_id: context.functionName,
        type: 'SYSTEM',
        ip_address: 'lambda'
      },
      resource: {
        type: 'LAMBDA_FUNCTION',
        id: context.functionArn,
        classification: 'INTERNAL'
      },
      action: {
        operation: 'EXECUTE',
        status: 'FAILURE',
        duration_ms: Date.now() - startTime,
        result: error.message
      }
    });

    throw error;
  } finally {
    await audit.flush();
  }
};
```

### B.3 Database Trigger Integration

```sql
-- PostgreSQL trigger for automatic audit logging

CREATE OR REPLACE FUNCTION audit_customer_data_access()
RETURNS TRIGGER AS $$
DECLARE
  audit_payload jsonb;
BEGIN
  -- Build audit payload
  audit_payload := jsonb_build_object(
    'event_type', 'DATA_ACCESS',
    'severity', 'MEDIUM',
    'actor', jsonb_build_object(
      'user_id', current_user,
      'type', 'HUMAN',
      'ip_address', inet_client_addr()::text
    ),
    'resource', jsonb_build_object(
      'type', 'DATABASE_TABLE',
      'id', TG_TABLE_NAME || '.' || NEW.id,
      'classification', 'CONFIDENTIAL'
    ),
    'action', jsonb_build_object(
      'operation', TG_OP,
      'status', 'SUCCESS'
    )
  );

  -- Send to audit system (via pg_notify or HTTP)
  PERFORM pg_notify('audit_events', audit_payload::text);

  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Attach trigger to customer table
CREATE TRIGGER customer_audit_trigger
  AFTER INSERT OR UPDATE OR DELETE ON customers
  FOR EACH ROW
  EXECUTE FUNCTION audit_customer_data_access();
```

---

## Appendix C: Compliance Checklists

### C.1 SOC 2 Type II Readiness Checklist

- [ ] **CC6.1 - System Monitoring**
  - [ ] Audit logging enabled on all systems
  - [ ] Real-time monitoring dashboard configured
  - [ ] Security events automatically captured
  - [ ] Regular log review process established

- [ ] **CC6.2 - Change Management**
  - [ ] Configuration changes logged
  - [ ] Change approval workflow with audit trail
  - [ ] Rollback procedures documented
  - [ ] Change history retrievable for 7+ years

- [ ] **CC6.3 - Access Monitoring**
  - [ ] All authentication attempts logged
  - [ ] Authorization decisions recorded
  - [ ] Privileged access separately tracked
  - [ ] Access reviews conducted quarterly

- [ ] **CC7.1 - Security Event Detection**
  - [ ] Anomaly detection system operational
  - [ ] Threat intelligence integration active
  - [ ] Automated alerting configured
  - [ ] Incident response procedures defined

- [ ] **CC7.2 - Incident Response**
  - [ ] Security incidents logged and tracked
  - [ ] Response times within SLA
  - [ ] Post-incident reviews conducted
  - [ ] Lessons learned documented

### C.2 ISO 27001 Implementation Checklist

- [ ] **A.12.4.1 - Event Logging**
  - [ ] User activities logged
  - [ ] Exceptions and errors captured
  - [ ] Security events recorded
  - [ ] Logs reviewed regularly

- [ ] **A.12.4.2 - Protection of Log Information**
  - [ ] Logs protected from tampering
  - [ ] Unauthorized access prevented
  - [ ] Cryptographic integrity protection
  - [ ] Access to logs restricted and audited

- [ ] **A.12.4.3 - Administrator Logs**
  - [ ] Privileged user activities logged
  - [ ] System administrator actions tracked
  - [ ] Root/admin access separately monitored
  - [ ] Privileged operations require approval

- [ ] **A.12.4.4 - Clock Synchronization**
  - [ ] All systems synchronized to NTP
  - [ ] Time source reliable and monitored
  - [ ] Timestamp accuracy verified
  - [ ] Time synchronization logged

### C.3 GDPR Compliance Checklist

- [ ] **Article 30 - Records of Processing**
  - [ ] Processing activities documented
  - [ ] Purposes of processing recorded
  - [ ] Categories of personal data identified
  - [ ] Data recipients logged
  - [ ] Retention periods specified

- [ ] **Article 32 - Security of Processing**
  - [ ] Audit logs encrypted at rest
  - [ ] Access control implemented
  - [ ] Integrity protection active
  - [ ] Regular security testing conducted

- [ ] **Article 33 - Breach Notification**
  - [ ] Breach detection system operational
  - [ ] 72-hour notification capability
  - [ ] Breach impact assessment process
  - [ ] Notification templates prepared

- [ ] **Data Subject Rights**
  - [ ] Right of access: Data retrieval capability
  - [ ] Right to erasure: Deletion audit trail
  - [ ] Right to rectification: Correction logging
  - [ ] Right to data portability: Export functionality

---

## Appendix D: Performance Benchmarks

### D.1 Expected Performance Metrics

| Operation | Target Latency (p95) | Target Throughput |
|-----------|---------------------|-------------------|
| Single event submission | < 10ms | 100,000 events/sec |
| Batch submission (100 events) | < 50ms | 1,000,000 events/sec |
| Query recent logs (last 24h) | < 100ms | 10,000 queries/sec |
| Query historical logs (30 days) | < 500ms | 1,000 queries/sec |
| Chain verification (10,000 entries) | < 1s | 100 verifications/sec |
| Complex analytics query | < 5s | 50 queries/sec |
| Real-time stream subscription | < 50ms latency | 500,000 events/sec |

### D.2 Storage Requirements

| Retention Period | Events/Day | Estimated Storage |
|-----------------|------------|-------------------|
| 90 days (hot) | 10 million | ~500 GB |
| 1 year (warm) | 10 million | ~2 TB |
| 7 years (cold) | 10 million | ~14 TB (compressed) |

### D.3 Scaling Guidelines

**Small Deployment (< 1M events/day)**
- Single region deployment
- PostgreSQL with TimescaleDB
- 2 application servers
- 1 database server (16 vCPU, 64 GB RAM)

**Medium Deployment (1M - 10M events/day)**
- Multi-region deployment
- Elasticsearch cluster (3 nodes)
- 5-10 application servers
- Message queue (Kafka/RabbitMQ)

**Large Deployment (> 10M events/day)**
- Global multi-region deployment
- Distributed storage (Cassandra/DynamoDB)
- 20+ application servers with auto-scaling
- Kafka cluster for event streaming
- Dedicated analytics cluster

---

## Appendix E: Security Hardening Guide

### E.1 Network Security

```
Audit System Network Architecture:

┌─────────────────────────────────────────────────┐
│              Internet (HTTPS only)              │
└─────────────────┬───────────────────────────────┘
                  │
          ┌───────▼────────┐
          │  Load Balancer │
          │   (TLS 1.3)    │
          └───────┬────────┘
                  │
     ┌────────────┼────────────┐
     │            │            │
┌────▼────┐  ┌────▼────┐  ┌────▼────┐
│  App    │  │  App    │  │  App    │
│ Server  │  │ Server  │  │ Server  │
│  (DMZ)  │  │  (DMZ)  │  │  (DMZ)  │
└────┬────┘  └────┬────┘  └────┬────┘
     │            │            │
     └────────────┼────────────┘
                  │
          ┌───────▼────────┐
          │   Firewall     │
          │  (Allow 5432)  │
          └───────┬────────┘
                  │
          ┌───────▼────────┐
          │   Database     │
          │  (Internal)    │
          └────────────────┘
```

### E.2 Encryption Requirements

- **TLS 1.3** for all API communications
- **AES-256-GCM** for data at rest
- **RSA-4096** or **ECDSA P-384** for digital signatures
- **Perfect Forward Secrecy** for TLS sessions
- **Key rotation** every 90 days minimum

### E.3 Access Control

```yaml
# Role-Based Access Control (RBAC) for Audit System

roles:
  - name: audit_writer
    permissions:
      - audit:write
    description: Can submit audit events only

  - name: audit_reader
    permissions:
      - audit:read
      - audit:query
    description: Can read and query audit logs

  - name: audit_admin
    permissions:
      - audit:read
      - audit:query
      - audit:verify
      - audit:export
      - audit:configure
    description: Full audit system administration

  - name: compliance_officer
    permissions:
      - audit:read
      - audit:query
      - audit:export
      - audit:report
    description: Can generate compliance reports
```

---

## Appendix F: Troubleshooting Guide

### F.1 Common Issues

**Issue: High latency in audit submissions**
- Check network connectivity to audit service
- Verify database connection pool size
- Review batch size configuration
- Monitor database query performance

**Issue: Missing audit events**
- Check buffer flush configuration
- Verify network stability
- Review error logs for submission failures
- Check retry queue status

**Issue: Chain integrity verification failures**
- Identify the broken chain segment
- Check for system clock drift
- Verify cryptographic key consistency
- Review recent system changes

**Issue: Storage capacity issues**
- Review retention policies
- Implement data archival
- Enable compression
- Consider data tiering strategy

---

## Appendix G: Migration Guide

### G.1 Migrating from Legacy Audit Systems

**Phase 1: Parallel Operation (Week 1-4)**
1. Deploy WIA-SEC-017 audit system
2. Configure dual logging (legacy + WIA)
3. Validate data consistency
4. Train team on new system

**Phase 2: Gradual Cutover (Week 5-8)**
1. Migrate low-risk systems first
2. Monitor for issues
3. Adjust configurations as needed
4. Continue dual logging

**Phase 3: Complete Migration (Week 9-12)**
1. Migrate remaining systems
2. Disable legacy audit system
3. Archive historical logs
4. Update documentation

### G.2 Data Import Process

```typescript
// Import historical audit logs
async function importHistoricalLogs(legacyLogs: any[]) {
  const auditClient = new AuditClient({ /* config */ });

  for (const legacyLog of legacyLogs) {
    const wiaLog = convertLegacyToWIA(legacyLog);
    await auditClient.log(wiaLog);
  }

  await auditClient.flush();
}

function convertLegacyToWIA(legacyLog: any): AuditEntry {
  return {
    event_type: mapEventType(legacyLog.type),
    severity: mapSeverity(legacyLog.level),
    actor: {
      user_id: legacyLog.user,
      type: 'HUMAN',
      ip_address: legacyLog.ip
    },
    resource: {
      type: 'UNKNOWN',
      id: legacyLog.resource,
      classification: 'INTERNAL'
    },
    action: {
      operation: legacyLog.action,
      status: legacyLog.success ? 'SUCCESS' : 'FAILURE'
    },
    metadata: {
      custom_fields: {
        legacy_id: legacyLog.id,
        imported: true
      }
    }
  };
}
```

---

**Previous:** [PHASE-2-&-3-&-4.md](./PHASE-2-&-3-&-4.md)
**Next:** [SPEC-GLOSSARY.md](./SPEC-GLOSSARY.md)

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
