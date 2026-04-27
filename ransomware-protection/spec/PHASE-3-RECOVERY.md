# PHASE 3 — Recovery & Business Continuity


### 3.1 Disaster Recovery Automation

#### 3.1.1 Recovery Runbooks
Automated playbooks for different ransomware scenarios:

**Scenario 1: Single Workstation Infection**
```yaml
runbook: "workstation_infection"
steps:
  - name: "Isolate Endpoint"
    action: "network_isolation"
    timeout: "30s"

  - name: "Terminate Malicious Process"
    action: "kill_process"
    target: "detected_pid"

  - name: "Scan Network Shares"
    action: "scan_shares"
    scope: "user_accessible"

  - name: "Restore from Backup"
    action: "restore_latest_clean"
    verify: true

  - name: "Malware Scan"
    action: "full_system_scan"

  - name: "Gradual Reconnection"
    action: "monitored_network_access"
    duration: "72h"
```

**Scenario 2: Server Infection**
```yaml
runbook: "server_infection"
priority: "CRITICAL"
steps:
  - name: "Failover to Secondary"
    action: "activate_standby_server"

  - name: "Isolate Infected Server"
    action: "network_isolation"

  - name: "Forensic Capture"
    action: "memory_dump + disk_image"

  - name: "Restore from Immutable Backup"
    action: "restore_from_worm"
    verify: "hash_verification"

  - name: "Integrity Check"
    action: "full_integrity_scan"

  - name: "Gradual Cutover"
    action: "staged_production_return"
```

#### 3.1.2 Prioritized Recovery
Not all systems are equal. Recover in order of business criticality:

**Priority Tiers:**
```json
{
  "tier1_critical": {
    "rto": "15 minutes",
    "systems": [
      "payment_processing",
      "customer_database",
      "authentication_servers"
    ],
    "backup_frequency": "continuous",
    "testing_frequency": "weekly"
  },
  "tier2_important": {
    "rto": "1 hour",
    "systems": [
      "email_servers",
      "file_servers",
      "crm_systems"
    ],
    "backup_frequency": "hourly",
    "testing_frequency": "monthly"
  },
  "tier3_standard": {
    "rto": "4 hours",
    "systems": [
      "internal_tools",
      "development_environments"
    ],
    "backup_frequency": "daily",
    "testing_frequency": "quarterly"
  }
}
```

---

### 3.2 Business Continuity Planning

#### 3.2.1 Continuity of Operations (COOP)
Maintain critical business functions during recovery:

**Alternative Operating Procedures:**
- **Manual Processes**: Paper-based workflows for critical operations
- **Alternative Sites**: Failover to DR site or cloud infrastructure
- **Third-Party Services**: Temporary use of SaaS alternatives
- **Communication Plan**: Stakeholder notification procedures

#### 3.2.2 Tabletop Exercises
Regular exercises to validate recovery procedures:

**Exercise Schedule:**
- **Monthly**: Single-system recovery drill
- **Quarterly**: Department-wide recovery simulation
- **Annually**: Organization-wide disaster recovery exercise

**Sample Scenario:**
```
Scenario: Widespread ransomware attack affects 200 workstations and 5 servers
Objectives:
  1. Isolate infected systems within 5 minutes
  2. Restore critical servers within 30 minutes
  3. Notify stakeholders within 1 hour
  4. Restore 80% of workstations within 4 hours

Success Metrics:
  - Detection time: <2 minutes
  - Isolation time: <5 minutes
  - Server RTO: <30 minutes
  - Communication effectiveness: 90%+
```

---

### 3.3 Post-Incident Analysis

#### 3.3.1 Forensic Investigation
Thorough analysis to understand attack vectors and prevent recurrence:

**Investigation Areas:**
1. **Initial Access**: How did ransomware enter the environment?
   - Phishing email?
   - Exploited vulnerability?
   - Compromised credentials?
   - Supply chain attack?

2. **Lateral Movement**: How did it spread?
   - Network shares?
   - Remote desktop?
   - Exploited trust relationships?

3. **Data Exfiltration**: Was data stolen before encryption?
   - Monitor for double extortion attempts
   - Check for unusual outbound transfers

4. **Lessons Learned**: What can be improved?
   - Detection capabilities
   - Response procedures
   - User training
   - Technical controls

#### 3.3.2 Root Cause Analysis
```markdown
# Incident Report: INC-20251225-001

## Executive Summary
Ransomware infection detected on 1 workstation, contained within 3 minutes, zero data loss.

## Timeline
- 10:35:00 - Malicious executable launched via phishing email
- 10:35:12 - Behavioral detection triggered alert
- 10:35:15 - Automated isolation executed
- 10:35:45 - Process terminated, snapshot created
- 10:40:00 - Workstation restored from backup

## Root Cause
User clicked malicious link in phishing email bypassing email filter.

## Corrective Actions
1. Update email filter rules (completed)
2. Additional phishing awareness training (scheduled)
3. Implement URL sandboxing (in progress)
4. Review and update allow-lists (completed)

## Recommendations
- Deploy application whitelisting
- Implement MFA for all users
- Increase backup frequency for critical users
```

---


## Implementer notes — recovery envelope

Phase 3 covers immutable backup attestation, restore orchestration, and
business-continuity reporting. The structure intentionally aligns with
NIST SP 800-184 (Guide for Cybersecurity Event Recovery) and the
ISO/IEC 27031 ICT readiness for business continuity terminology.

### `backup_snapshot`

```
{
  "snapshot_id": "ULID",
  "tenant_id": "did:wia:tenant:...",
  "captured_at": "RFC 3339",
  "asset_class": "volume" | "object_store" | "database" | "kv_index"
              | "config_repo" | "secret_vault",
  "asset_ref": "opaque (vendor-specific URI)",
  "size_bytes": 0,
  "object_count": 0,
  "encryption": {
    "algorithm": "AES-256-GCM",
    "kms_key_ref": "...",
    "wrapping": "envelope" | "raw"
  },
  "immutability": {
    "retention_until": "RFC 3339",
    "lock_kind": "compliance" | "governance" | "object-lock-s3",
    "broker": "did:wia:storage:..."
  },
  "merkle_root": "SHA-384 over object hashes (canonical traversal)",
  "signature": "Ed25519 JCS"
}
```

The `merkle_root` lets a recovery operator prove that a restored
snapshot is bit-identical to what was attested without re-reading the
entire dataset — only the path from leaf to root is needed.

### `restore_run`

A `restore_run` is a sequence of `restore_step` envelopes; each step
resolves a single asset. The orchestrator MUST emit a final
`restore_summary` containing measured RTO and RPO, computed from
`captured_at` and `incident_started_at` timestamps. RTO/RPO are not
self-reported targets — they are observed values.

### Continuity reporting

The `continuity_report` envelope is designed for board-level reporting
and regulatory disclosure (e.g., SEC 8-K cyber incident filings, EU
NIS 2 reporting, US CIRCIA 72-hour reports). It carries the minimum
fields most jurisdictions require: incident scope, impacted services,
customer notification timeline, and remediation milestones. The
intention is that one signed envelope feeds many regulatory portals.

## Forensic preservation

A subset of `backup_snapshot` envelopes are flagged
`forensic_preservation: true`. These snapshots are exempt from normal
retention expiry until an `incident_closed_at` timestamp is recorded
on the parent incident. This protects evidence that may be required
for downstream criminal proceedings or insurance claims.

Forensic preservation snapshots SHOULD use `lock_kind: compliance` so
that even tenant administrators cannot remove them prior to expiry.
The standard does not mandate which jurisdiction's evidence-handling
rules apply — that is left to the tenant and its legal counsel — but
it does mandate that the chain-of-custody fields below be present:

- `custodian` — DID of the party currently holding the snapshot.
- `transfer_log` — append-only list of `{from, to, transferred_at,
  reason}` triples.
- `tamper_seal` — Ed25519 over `{snapshot_id, custodian, merkle_root}`,
  re-signed at every transfer.

### Recovery posture metrics

Recovery posture is published as a separate `posture_report` envelope,
typically on a daily or weekly cadence. It carries:

- `rto_target_minutes` and `rto_observed_minutes_p95` (from drills).
- `rpo_target_minutes` and `rpo_observed_minutes_p95`.
- `last_drill_at` and `next_drill_at`.
- `immutable_backup_coverage_pct` — fraction of declared critical
  assets that have an attested immutable snapshot in the last 24h.
- `cross_region_coverage_pct` — fraction with a snapshot in a second
  geographic region.

These metrics give regulators and cyber insurers a uniform,
machine-comparable view of recovery readiness across vendors.

## Tabletop and live-fire drill envelopes

The standard treats drills as first-class evidence rather than ad-hoc
internal exercises. A `recovery_drill` envelope captures:

```
{
  "drill_id": "ULID",
  "tenant_id": "did:wia:tenant:...",
  "kind": "tabletop" | "functional" | "full-scale-cutover",
  "scenario_ref": "opaque (scenario library id)",
  "scope": ["asset_class:database", "region:eu-west-1", ...],
  "started_at": "RFC 3339",
  "ended_at": "RFC 3339",
  "rto_observed_minutes": 0,
  "rpo_observed_minutes": 0,
  "issues": [ { "id": "...", "severity": "...", "remediation_due": "..." } ],
  "signed_off_by": "did:wia:bcp-officer:...",
  "signature": "Ed25519 JCS"
}
```

Drill outputs feed the `posture_report` p95 metrics defined above, so
that "recovery readiness" reflects *measured* performance under
realistic conditions rather than aspirational targets.

## Cyber-insurance binding

Cyber insurers increasingly require attested evidence of immutable
backup posture, drill cadence, and identity hardening as preconditions
for coverage. The `insurance_attestation` envelope bundles the
relevant `posture_report`, `recovery_drill`, and
`identity_hardening_attestation` envelopes by reference, signed by
the tenant's risk officer. The intent is that one signed bundle
satisfies a standard market-form questionnaire without bespoke
re-collection of the same evidence each renewal cycle.

## Backwards compatibility

Implementations transitioning from vendor-proprietary backup attestation
formats MAY embed the original attestation as a base64-encoded blob in
the `legacy_attestation` field of `backup_snapshot`. This field is
advisory and MUST NOT be used for restore-time verification; the
`merkle_root` remains the canonical proof.

## Operational considerations

Restore drills SHOULD execute against a forensically isolated network
to avoid recovering compromised credentials or persistence into a
production environment. The standard does not mandate the isolation
mechanism (vendor air-gap, SDN microsegment, restricted VPC) but
REQUIRES that the `recovery_drill` envelope record the isolation
boundary identifier and an attestation that the boundary was active
for the full duration of the drill.

Encryption key rotation following a confirmed compromise MUST occur
before the restored data is re-mounted into production; the rotation
event is recorded as a separate `key_rotation` envelope referenced by
the `restore_summary`.
