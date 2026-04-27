# PHASE 2 — Advanced Detection & Response


### 2.1 Advanced Behavioral Analysis

#### 2.1.1 User and Entity Behavior Analytics (UEBA)
Establish baselines for normal user and system behavior to detect anomalies indicative of ransomware.

**Baseline Metrics:**
- **File Access Patterns**: Typical files accessed, frequency, time of day
- **Network Behavior**: Normal destinations, data transfer volumes
- **Application Usage**: Commonly used applications, execution times
- **Login Patterns**: Typical login times, locations, devices

**Anomaly Detection:**
```python
anomalies = {
    "unusual_time": "File access at 3 AM (user typically works 9-5)",
    "unusual_volume": "10GB file access (typical: 500MB/day)",
    "unusual_location": "Access from external IP (user always local)",
    "unusual_application": "Unknown executable (never seen before)"
}

risk_score = calculate_risk(anomalies)
if risk_score > 85:
    trigger_investigation()
```

#### 2.1.2 Lateral Movement Detection
Ransomware often spreads across networks. Detect lateral movement attempts:

**Detection Indicators:**
- Multiple failed authentication attempts across systems
- Privilege escalation attempts
- Accessing admin shares (C$, ADMIN$)
- PsExec, WMI, or RDP connections to multiple hosts
- Credential dumping tools (Mimikatz, LaZagne)

**Network Traffic Analysis:**
```
Source: workstation-42 (192.168.1.105)
Targets:
  - fileserver-01 (445/SMB) - FAILED AUTH x5
  - fileserver-02 (445/SMB) - SUCCESS
  - workstation-43 (3389/RDP) - CONNECTION
  - workstation-44 (3389/RDP) - CONNECTION

Verdict: LATERAL_MOVEMENT_DETECTED
Action: ISOLATE_SOURCE + ALERT_SOC
```

#### 2.1.3 Credential Theft Detection
Monitor for credential harvesting activities:

- **LSASS Memory Access**: Detect attempts to read LSASS process memory
- **SAM Database Access**: Monitor access to Security Account Manager database
- **Kerberos Ticket Extraction**: Detect tools extracting Kerberos tickets
- **Registry Key Access**: Monitor HKLM\SAM, HKLM\SECURITY

---

### 2.2 Deception Technology

#### 2.2.1 Honeypot Files
Deploy decoy files throughout the file system to detect ransomware scanning.

**Honeypot Strategy:**
```json
{
  "honeypots": [
    {
      "name": "Financial_Records_2025_CONFIDENTIAL.xlsx",
      "path": "C:\\Users\\jdoe\\Documents\\",
      "canary": true,
      "monitor": "READ|WRITE|MODIFY|DELETE"
    },
    {
      "name": "Passwords.txt",
      "path": "C:\\Users\\jdoe\\Desktop\\",
      "canary": true,
      "trigger": "IMMEDIATE_ISOLATION"
    }
  ],
  "detection": {
    "action": "Any access to honeypot triggers alert",
    "confidence": "99.9% (legitimate users never access these)"
  }
}
```

**Benefits:**
- Early detection (ransomware scans for valuable files first)
- High confidence (false positives near zero)
- Forensic value (tracks attacker behavior)

#### 2.2.2 Canary Tokens
Embed canary tokens in documents, databases, and configuration files.

**Token Types:**
- **DNS Canaries**: Trigger alert when domain is queried
- **Web Beacons**: Alert when specific URL is accessed
- **AWS Keys**: Fake AWS credentials that alert when used
- **Email Addresses**: Alert when email is sent to address

---

### 2.3 Threat Intelligence Integration

#### 2.3.1 Real-Time Threat Feeds
Integrate with global threat intelligence platforms:

**Supported Feeds:**
- **MITRE ATT&CK**: Tactics, techniques, and procedures (TTPs)
- **AlienVault OTX**: Community-driven threat intelligence
- **Abuse.ch**: Ransomware tracker, malware hashes
- **VirusTotal**: File and URL reputation
- **Commercial Feeds**: Recorded Future, ThreatConnect, Anomali

**Feed Integration:**
```json
{
  "threatIntel": {
    "feeds": [
      {
        "provider": "abuse.ch",
        "type": "ransomware_tracker",
        "updateFrequency": "hourly",
        "indicators": ["IP", "domain", "hash", "C2"]
      },
      {
        "provider": "MITRE-ATTACK",
        "type": "TTPs",
        "mapping": "T1486 (Data Encrypted for Impact)"
      }
    ],
    "autoBlock": true,
    "confidence_threshold": 0.8
  }
}
```

#### 2.3.2 IOC Matching
Automatically match observed indicators against threat feeds:

```python
def check_ioc(indicator, indicator_type):
    """
    Check if indicator matches known ransomware IOC
    """
    feeds = load_threat_feeds()

    for feed in feeds:
        if match := feed.search(indicator, indicator_type):
            return {
                "matched": True,
                "threat": match.threat_name,
                "family": match.ransomware_family,
                "confidence": match.confidence,
                "source": feed.name,
                "recommendation": "BLOCK_AND_INVESTIGATE"
            }

    return {"matched": False}

# Example usage
result = check_ioc("192.0.2.100", "IP")
# Result: {"matched": True, "threat": "LockBit C2 Server", ...}
```

---

### 2.4 Memory Forensics

#### 2.4.1 Memory Acquisition
Capture volatile memory when ransomware is detected:

**Tools Integration:**
- **Volatility**: Open-source memory forensics
- **Rekall**: Advanced memory analysis
- **WinPmem**: Windows memory acquisition
- **LiME**: Linux Memory Extractor

**Acquisition Process:**
```bash
# Immediate memory dump on detection
./winpmem-3.3.rc3.exe physmem.raw

# Analyze for ransomware artifacts
volatility -f physmem.raw --profile=Win10x64 malfind
volatility -f physmem.raw --profile=Win10x64 hollowfind
```

#### 2.4.2 Analysis Automation
Automated memory analysis for rapid threat identification:

**Analysis Steps:**
1. **Process Enumeration**: List all running processes
2. **Code Injection Detection**: Identify injected code
3. **Network Connections**: Map active network connections
4. **Registry Keys**: Extract suspicious registry modifications
5. **Encryption Keys**: Attempt to recover ransomware encryption keys

---


## Implementer notes — detection telemetry envelope

The Phase 2 detection envelope is intentionally aligned with the
NIST SP 800-61 Rev. 2 incident handling lifecycle and MITRE ATT&CK
v15 Enterprise tactics so that downstream SOC tooling can correlate
WIA-emitted events with existing playbooks without translation.

Each `detection_event` record carries:

- `event_id` (ULID) — globally unique, monotonically sortable.
- `tenant_id` — DID-formatted issuer; required for federation routing.
- `observed_at` (RFC 3339) — wall-clock time at the agent.
- `mitre_attack` — array of `{tactic, technique, sub_technique}` triples
  (e.g., `TA0040 / T1486` for Data Encrypted for Impact).
- `confidence` (0.0–1.0) — calibrated detector posterior, not a heuristic.
- `evidence_hash` — SHA-384 over the canonical evidence bundle, used to
  bind a later quarantined-file `incident_report` envelope to its origin.
- `signature` — Ed25519 over the canonical JSON form (RFC 8785 JCS).

## Detector classes

The standard distinguishes four detector classes so that response
priority can be derived without re-reading the payload:

1. **Behavioural canary** — file/registry honeytokens triggered.
2. **Cryptographic anomaly** — entropy spike + extension churn + mass
   rename within a short window (matches MITRE T1486 + T1490).
3. **Identity exfiltration** — privileged credential reuse outside of
   established baselines (T1078, T1003).
4. **Lateral movement** — SMB/RDP/PsExec spike (T1021).

Detectors MUST emit at least one `mitre_attack` triple per event.

## Response orchestration

`response_action` envelopes describe the SOAR action taken:

```
{
  "action_id": "ULID",
  "incident_id": "...",
  "kind": "isolate_host" | "revoke_session" | "snapshot_volume"
        | "block_egress_cidr" | "rotate_credential" | "quarantine_file",
  "executed_at": "RFC 3339",
  "executed_by": "did:wia:soar:...",
  "rollback_token": "opaque",
  "signature": "Ed25519 JCS"
}
```

`rollback_token` is REQUIRED; it allows a human IR lead to safely
reverse an automated action that turns out to be a false positive
without re-reading raw playbook state. This is a deliberate departure
from "fire and forget" SOAR primitives.

## Federation rules (Phase 3 cross-reference)

Detection envelopes flagged with `severity ≥ HIGH` and `confidence ≥ 0.85`
are eligible for cross-jurisdictional ISAC propagation. The Phase 3
federation handshake gates this on a per-tenant `share_with` allowlist
to avoid leaking sensitive enterprise telemetry to unrelated partners.

## Triage queue and analyst feedback loop

`detection_event` envelopes do not directly mutate analyst queues;
instead, a downstream triage broker emits `triage_state` envelopes
that pin a current owner, queue, and SLA clock to each event:

```
{
  "triage_id": "ULID",
  "event_id": "...",
  "queue": "L1" | "L2" | "L3" | "DFIR" | "EXEC",
  "owner": "did:wia:analyst:...",
  "sla_due_at": "RFC 3339",
  "state": "open" | "investigating" | "contained" | "closed-tp"
         | "closed-fp" | "closed-bp",
  "transitioned_at": "RFC 3339",
  "signature": "Ed25519 JCS"
}
```

`closed-tp/-fp/-bp` (true positive / false positive / benign positive)
classifications feed back into detector calibration. A standardised
feedback envelope is mandatory because vendor-specific "verdict"
strings have historically been the largest source of cross-tool
analytics drift.

## Threat-intel binding

When a `detection_event` is enriched with external threat intel
(e.g., a STIX 2.1 indicator from an ISAC), the enrichment is carried
in a separate `intel_binding` envelope rather than mutating the
original event. This preserves the cryptographic signature on the
original detection while still allowing intel-derived priority
boosts in downstream queues.

```
{
  "binding_id": "ULID",
  "event_id": "...",
  "indicator_ref": "stix2.1:indicator--<uuid>",
  "source": "did:wia:isac:...",
  "tlp": "RED" | "AMBER+STRICT" | "AMBER" | "GREEN" | "CLEAR",
  "score_delta": -1.0..+1.0,
  "signature": "Ed25519 JCS"
}
```

The `tlp` field is REQUIRED so that downstream sharing decisions
(Phase 3 federation) can honour the source's traffic-light protocol
classification without re-deriving it from the enrichment payload.

## Backwards compatibility

Implementations migrating from a STIX 2.1-only stack MAY emit both a
`detection_event` envelope and a STIX 2.1 `sighting` SDO during a
transitional window. The two SHOULD share an `external_id` carrying
the STIX `sighting.id` value so that downstream correlation does not
double-count the same observation.

## Operational considerations

Detector tuning is a continuous process. The `closed-fp` and `closed-bp`
states defined above feed a calibration pipeline that re-derives
posterior thresholds weekly; thresholds SHOULD be published as a signed
`detector_calibration` envelope so that downstream scoring is auditable.
False-positive feedback that arrives after a 30-day window SHOULD be
ignored for calibration purposes (concept drift dominates beyond that
horizon for ransomware-relevant detectors).

Federation propagation latency is a key SOC-level metric; implementations
SHOULD measure and publish p50/p95/p99 propagation times for each
federated peer as part of their `posture_report`.
