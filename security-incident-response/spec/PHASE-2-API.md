# WIA-SEC-020: Security Incident Response - Phases 2, 3, 4 Specification

**Standard ID:** WIA-SEC-020
**Version:** 1.0
**Status:** ACTIVE
**Last Updated:** 2025-12-25

---

## Phase 2: Advanced Detection and Analysis

### 2.1 Threat Intelligence Integration

#### 2.1.1 STIX/TAXII Integration

**STIX (Structured Threat Information Expression):**
```json
{
  "type": "indicator",
  "spec_version": "2.1",
  "id": "indicator--8e2e2d2b-17d4-4cbf-938f-98ee46b3cd3f",
  "created": "2025-12-25T10:00:00.000Z",
  "modified": "2025-12-25T10:00:00.000Z",
  "name": "Malicious IP Indicator",
  "description": "IP address associated with APT29 C2 infrastructure",
  "indicator_types": ["malicious-activity"],
  "pattern": "[ipv4-addr:value = '192.168.1.100']",
  "pattern_type": "stix",
  "valid_from": "2025-12-25T10:00:00Z",
  "kill_chain_phases": [
    {
      "kill_chain_name": "mitre-attack",
      "phase_name": "command-and-control"
    }
  ]
}
```

**TAXII Server Configuration:**
```python
# TAXII 2.1 Client Implementation
from taxii2client.v21 import Server, Collection

# Connect to threat intelligence server
server = Server("https://threatintel.example.com/taxii/",
                user="csirt", password="***")

# Get available collections
collections = server.api_root.collections

# Poll for new indicators
for collection in collections:
    if collection.title == "APT Indicators":
        indicators = collection.get_objects()
        for indicator in indicators:
            process_threat_indicator(indicator)
```

#### 2.1.2 MITRE ATT&CK Mapping

**Technique Mapping:**
```python
class MitreAttackMapper:
    """Map observed behaviors to MITRE ATT&CK techniques"""

    def __init__(self):
        self.technique_db = self.load_mitre_database()

    def map_behavior(self, observed_behavior):
        """
        Map observed behavior to ATT&CK technique
        """
        mappings = {
            "powershell_execution": {
                "technique": "T1059.001",
                "tactic": "Execution",
                "name": "PowerShell"
            },
            "credential_dumping": {
                "technique": "T1003",
                "tactic": "Credential Access",
                "name": "OS Credential Dumping"
            },
            "lateral_movement_rdp": {
                "technique": "T1021.001",
                "tactic": "Lateral Movement",
                "name": "Remote Desktop Protocol"
            },
            "data_exfiltration_http": {
                "technique": "T1041",
                "tactic": "Exfiltration",
                "name": "Exfiltration Over C2 Channel"
            }
        }

        return mappings.get(observed_behavior, None)

    def build_attack_chain(self, observed_techniques):
        """
        Reconstruct attack chain from observed techniques
        """
        kill_chain = {
            "initial_access": [],
            "execution": [],
            "persistence": [],
            "privilege_escalation": [],
            "defense_evasion": [],
            "credential_access": [],
            "discovery": [],
            "lateral_movement": [],
            "collection": [],
            "exfiltration": [],
            "command_and_control": []
        }

        for technique in observed_techniques:
            tactic = technique.get("tactic", "").lower().replace(" ", "_")
            if tactic in kill_chain:
                kill_chain[tactic].append(technique)

        return kill_chain
```

#### 2.1.3 Automated Threat Hunting

**Hunt Hypothesis Development:**
```python
class ThreatHuntingFramework:
    """Proactive threat hunting using hypothesis-driven approach"""

    def create_hunt_hypothesis(self, threat_intel):
        """
        Create testable hunt hypothesis from threat intelligence
        """
        hypothesis = {
            "id": f"HUNT-{generate_id()}",
            "threat_actor": threat_intel.get("actor"),
            "hypothesis": "",
            "ttps": [],
            "data_sources": [],
            "search_queries": [],
            "expected_evidence": []
        }

        # Example: APT29 hunt
        if threat_intel.get("actor") == "APT29":
            hypothesis.update({
                "hypothesis": "APT29 using compromised credentials for initial access",
                "ttps": ["T1078", "T1071", "T1059.001"],
                "data_sources": [
                    "Windows Event Logs (4624, 4625)",
                    "PowerShell logs",
                    "Network traffic to known C2"
                ],
                "search_queries": [
                    {
                        "query": 'EventID=4624 AND LogonType=3 AND TargetUserName!=*$ AND NOT SourceNetworkAddress IN (known_admin_ips)',
                        "expected_results": "Unusual remote logons from unexpected IPs"
                    },
                    {
                        "query": 'process_name=powershell.exe AND (command_line=*-encodedcommand* OR command_line=*-enc*)',
                        "expected_results": "Encoded PowerShell commands"
                    }
                ]
            })

        return hypothesis

    def execute_hunt(self, hypothesis):
        """
        Execute hunt queries and analyze results
        """
        results = {
            "hypothesis_id": hypothesis["id"],
            "findings": [],
            "iocs_discovered": [],
            "recommended_actions": []
        }

        for query in hypothesis["search_queries"]:
            findings = self.siem.search(query["query"])

            if len(findings) > 0:
                results["findings"].append({
                    "query": query["query"],
                    "hits": len(findings),
                    "sample_events": findings[:10],
                    "severity": self.assess_severity(findings)
                })

        return results
```

### 2.2 Behavioral Analytics

#### 2.2.1 User and Entity Behavior Analytics (UEBA)

**Anomaly Detection Models:**
```python
import numpy as np
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler

class UEBASystem:
    """User and Entity Behavior Analytics"""

    def __init__(self):
        self.model = IsolationForest(contamination=0.1, random_state=42)
        self.scaler = StandardScaler()

    def build_user_baseline(self, user_id, historical_data):
        """
        Build baseline behavior profile for user
        """
        features = self.extract_features(historical_data)

        baseline = {
            "user_id": user_id,
            "typical_login_times": self.analyze_login_patterns(features),
            "typical_locations": self.analyze_locations(features),
            "typical_data_access": self.analyze_data_access(features),
            "typical_network_activity": self.analyze_network(features),
            "peer_group": self.identify_peer_group(user_id)
        }

        return baseline

    def detect_anomalies(self, current_behavior, baseline):
        """
        Detect deviations from baseline behavior
        """
        anomalies = []

        # Login time anomaly
        if self.is_unusual_time(current_behavior["login_time"],
                                baseline["typical_login_times"]):
            anomalies.append({
                "type": "UNUSUAL_LOGIN_TIME",
                "severity": "MEDIUM",
                "description": f"Login at {current_behavior['login_time']} outside typical hours"
            })

        # Geo-location anomaly
        if current_behavior["location"] not in baseline["typical_locations"]:
            distance = self.calculate_distance(
                current_behavior["location"],
                baseline["typical_locations"][-1]
            )
            if distance > 500:  # km
                anomalies.append({
                    "type": "IMPOSSIBLE_TRAVEL",
                    "severity": "HIGH",
                    "description": f"Login from {current_behavior['location']}, {distance}km from last location"
                })

        # Data access anomaly
        if current_behavior["files_accessed"] > baseline["typical_data_access"]["max"] * 3:
            anomalies.append({
                "type": "EXCESSIVE_DATA_ACCESS",
                "severity": "HIGH",
                "description": "Accessing significantly more files than typical"
            })

        return anomalies

    def calculate_risk_score(self, user_id, anomalies):
        """
        Calculate overall risk score based on detected anomalies
        """
        weights = {
            "UNUSUAL_LOGIN_TIME": 2,
            "IMPOSSIBLE_TRAVEL": 8,
            "EXCESSIVE_DATA_ACCESS": 7,
            "PRIVILEGE_ESCALATION": 9,
            "UNUSUAL_COMMAND": 6
        }

        risk_score = sum(weights.get(a["type"], 1) for a in anomalies)

        if risk_score >= 15:
            return {"score": risk_score, "level": "CRITICAL"}
        elif risk_score >= 10:
            return {"score": risk_score, "level": "HIGH"}
        elif risk_score >= 5:
            return {"score": risk_score, "level": "MEDIUM"}
        else:
            return {"score": risk_score, "level": "LOW"}
```

#### 2.2.2 Network Traffic Analysis

**Deep Packet Inspection:**
```python
from scapy.all import *

class NetworkBehaviorAnalyzer:
    """Analyze network traffic for suspicious patterns"""

    def analyze_dns_tunneling(self, packets):
        """
        Detect DNS tunneling for data exfiltration
        """
        dns_queries = [p for p in packets if p.haslayer(DNS) and p[DNS].qr == 0]

        suspicious = []
        for query in dns_queries:
            domain = query[DNS].qd.qname.decode()

            # Check for suspicious patterns
            if len(domain) > 50:  # Unusually long domain
                suspicious.append({
                    "type": "LONG_DNS_QUERY",
                    "domain": domain,
                    "length": len(domain),
                    "src_ip": query[IP].src
                })

            # Check for high entropy (encoded data)
            entropy = self.calculate_entropy(domain)
            if entropy > 4.5:
                suspicious.append({
                    "type": "HIGH_ENTROPY_DNS",
                    "domain": domain,
                    "entropy": entropy,
                    "src_ip": query[IP].src
                })

        return suspicious

    def detect_beaconing(self, connections):
        """
        Detect C2 beaconing behavior
        """
        # Group connections by destination
        conn_groups = {}
        for conn in connections:
            key = (conn["src_ip"], conn["dst_ip"], conn["dst_port"])
            if key not in conn_groups:
                conn_groups[key] = []
            conn_groups[key].append(conn["timestamp"])

        beacons = []
        for key, timestamps in conn_groups.items():
            if len(timestamps) < 10:
                continue

            # Calculate intervals between connections
            intervals = [timestamps[i+1] - timestamps[i]
                        for i in range(len(timestamps)-1)]

            # Check for regular intervals (beaconing)
            mean_interval = np.mean(intervals)
            std_interval = np.std(intervals)

            if std_interval / mean_interval < 0.1:  # Very consistent intervals
                beacons.append({
                    "src_ip": key[0],
                    "dst_ip": key[1],
                    "dst_port": key[2],
                    "interval": mean_interval,
                    "confidence": "HIGH",
                    "description": f"Regular beaconing every {mean_interval:.1f}s"
                })

        return beacons
```

---

---

## Z.1 Audit transport and observability hooks (Phase 2)

Every Phase 2 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `security-incident-response` and `wia.standard.phase` =
`2` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 2)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 2)

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-security-incident-response-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 2)

Phase 2 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 2)

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 2)

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.
