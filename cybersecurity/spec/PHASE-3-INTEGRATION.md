# Phase 3: System Integration

## Phase 3: SIEM 통합

### 3.1 SIEM 커넥터

**지원 SIEM 플랫폼**:
- Splunk
- IBM QRadar
- Azure Sentinel
- Elastic Security
- Chronicle Security

```python
class SIEMConnector:
    def __init__(self, siem_type: str, config: Dict):
        self.siem_type = siem_type
        self.config = config
        self.connector = self._get_connector(siem_type)

    def send_event(self, event: SecurityEvent) -> bool:
        """Send security event to SIEM"""
        formatted_event = self._format_for_siem(event)

        try:
            if self.siem_type == 'splunk':
                return self._send_to_splunk(formatted_event)
            elif self.siem_type == 'qradar':
                return self._send_to_qradar(formatted_event)
            elif self.siem_type == 'sentinel':
                return self._send_to_sentinel(formatted_event)
            else:
                return self._send_via_syslog(formatted_event)
        except Exception as e:
            logger.error(f"Failed to send event to SIEM: {e}")
            return False

    def _send_to_splunk(self, event: Dict) -> bool:
        """Send to Splunk HEC"""
        response = requests.post(
            f"{self.config['splunk_url']}/services/collector/event",
            headers={
                'Authorization': f"Splunk {self.config['hec_token']}"
            },
            json={
                'event': event,
                'sourcetype': 'wia:security:event',
                'index': self.config.get('index', 'security')
            },
            verify=self.config.get('verify_ssl', True)
        )
        return response.status_code == 200
```

### 3.2 이벤트 상관 분석

```json
{
  "correlation_rules": [
    {
      "rule_id": "CORR-001",
      "name": "Multiple Failed Logins Followed by Success",
      "severity": "high",
      "conditions": [
        {
          "event_type": "authentication_failed",
          "count": ">= 5",
          "time_window": "5 minutes"
        },
        {
          "event_type": "authentication_success",
          "same_user": true,
          "time_window": "1 minute after"
        }
      ],
      "actions": [
        "create_incident",
        "notify_soc",
        "lock_account"
      ]
    },
    {
      "rule_id": "CORR-002",
      "name": "Data Exfiltration Pattern",
      "severity": "critical",
      "conditions": [
        {
          "event_type": "large_data_transfer",
          "size": "> 1GB",
          "destination": "external"
        },
        {
          "event_type": "unusual_access_time",
          "time": "outside_business_hours"
        }
      ],
      "actions": [
        "create_critical_incident",
        "alert_ciso",
        "quarantine_endpoint",
        "block_network_traffic"
      ]
    }
  ]
}
```

---

## 보안 오케스트레이션

### 3.3 SOAR 플레이북

```yaml
# WIA-SEC-015 SOAR Playbook: Malware Detection Response
playbook:
  name: "Malware Detection and Remediation"
  id: "PB-MALWARE-001"
  trigger:
    event_type: "malware_detected"
    severity: ["high", "critical"]

  steps:
    - step: 1
      name: "Isolate Endpoint"
      action: "quarantine_device"
      timeout: 30s
      on_failure: "continue"

    - step: 2
      name: "Collect Forensics"
      action: "gather_forensic_data"
      parameters:
        - memory_dump: true
        - disk_image: true
        - network_capture: true
      timeout: 5m

    - step: 3
      name: "Analyze Malware"
      action: "sandbox_analysis"
      parameters:
        - sandbox_type: "cuckoo"
        - timeout: 10m

    - step: 4
      name: "Update Threat Intelligence"
      action: "add_to_ioc_database"
      parameters:
        - hash: "${malware.hash}"
        - type: "malware"

    - step: 5
      name: "Scan Network"
      action: "network_wide_scan"
      parameters:
        - ioc: "${malware.hash}"
        - scope: "entire_network"

    - step: 6
      name: "Create Incident"
      action: "create_incident_ticket"
      parameters:
        - severity: "high"
        - assignee: "soc_team"

    - step: 7
      name: "Notify Stakeholders"
      action: "send_notification"
      parameters:
        - channels: ["email", "slack", "pagerduty"]
        - recipients: ["soc_manager", "security_team"]
```

---

## 써드파티 통합

### 3.4 통합 카탈로그

| 카테고리 | 제품 | 통합 방법 | 상태 |
|---------|------|-----------|------|
| **SIEM** | Splunk | REST API | ✅ Supported |
| **SIEM** | QRadar | API + Syslog | ✅ Supported |
| **SIEM** | Sentinel | Log Analytics API | ✅ Supported |
| **EDR** | CrowdStrike | Falcon API | ✅ Supported |
| **EDR** | SentinelOne | Management API | ✅ Supported |
| **Firewall** | Palo Alto | PAN-OS API | ✅ Supported |
| **Firewall** | Fortinet | FortiGate API | ✅ Supported |
| **IAM** | Okta | SCIM + API | ✅ Supported |
| **IAM** | Azure AD | Graph API | ✅ Supported |
| **Vulnerability** | Tenable | Tenable.io API | ✅ Supported |
| **Vulnerability** | Qualys | VMDR API | ✅ Supported |

---

---

## Annex A — Conformance Tier Matrix

WIA conformance for cybersecurity is evaluated across three tiers:

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

- `wia-standards/standards/cybersecurity/api/` — TypeScript SDK skeleton
- `wia-standards/standards/cybersecurity/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/cybersecurity/simulator/` — interactive browser-based simulator for the PHASE protocol

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
