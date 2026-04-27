# WIA-network-security PHASE 1 — Data Format Specification

**Standard:** WIA-network-security
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for network-
security operations: protected-asset inventory, security-
event records, indicator-of-compromise (IOC) records,
threat-intelligence object records, vulnerability records,
incident records, alert records, response-action records,
and the cross-references binding events to incidents and
incidents to remediation. The shape interoperates with
established intelligence-sharing formats (STIX 2.1, TAXII
2.1) and event-detection schemas (OASIS Sigma, MITRE
ATT&CK / D3FEND, OASIS OpenC2).

References (CITATION-POLICY ALLOW only):
- OASIS STIX 2.1 — Structured Threat Information Expression
- OASIS TAXII 2.1 — Trusted Automated Exchange of Intelligence Information
- OASIS OpenC2 — Open Command and Control
- MITRE ATT&CK Framework
- MITRE D3FEND
- OASIS Sigma rule format
- IETF RFC 9356 (Vulnerability Reporting), RFC 9116 (security.txt)
- IETF RFC 4949 (Internet Security Glossary v2), RFC 6545 (RID)
- OASIS Common Security Advisory Framework (CSAF) 2.0
- ISO/IEC 27035 — Information security incident management
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 9162 (Certificate Transparency 2.0)

---

## §1 Scope

This PHASE applies to the data shape used by security-
operations centres (SOCs), threat-intelligence platforms
(TIPs), security-information-and-event-management (SIEM)
systems, and incident-response teams operating networked
infrastructure. It addresses event capture, IOC management,
vulnerability tracking, incident lifecycle, and response
actions; transport protocols are in PHASE 3 and integration
with broader IT/OT operations is in PHASE 4.

The standard is privacy-aware: any field that may carry
personally identifiable information (PII) of users (e.g.,
usernames, email addresses, IP addresses where attributable
to natural persons) carries a `ppiClass` marker so consumers
honour deployment-specific privacy controls before
re-disclosure.

In scope: protected-asset inventory, security events, IOC
records, STIX-aligned threat objects, vulnerability records,
incident records, alert records, response-action records,
analyst-attribution records. Out of scope: cryptographic
primitives (governed by WIA-pq-crypto), endpoint configuration
management (carried opaquely as asset properties), legal-
investigation chain of custody (cross-domain to legal
standards).

## §2 Protected-asset inventory record

Every monitored asset carries a stable inventory record:

| Field             | Source / Binding                                       |
|-------------------|--------------------------------------------------------|
| `assetRef`        | URN of form `urn:wia:nsec:asset:<authority>:<id>`      |
| `assetType`       | `host`, `network-device`, `cloud-workload`, `application`, `service`, `iot-device`, `ot-device`, `identity-account` |
| `criticalityClass`| `tier-1` (mission-critical), `tier-2` (important), `tier-3` (supporting), `tier-4` (informational) |
| `dataClass[]`     | data-classification labels (e.g., `pii`, `phi`, `classified`, `public`) |
| `ownerRef`        | URN of accountable owner                               |
| `locationRef`     | URN of physical or logical zone                        |
| `softwareInventory[]` | references to CPE-formatted software entries      |
| `controlsApplied[]` | references to control-framework entries (ISO 27001, NIST 800-53) |

Asset registry mutations are signed and audit-logged.
Criticality changes propagate to incident severity-mapping
rules within a deployment-declared latency budget.

## §3 Security-event record

Detector-generated events carry a normalised shape:

- `eventId` — URN
- `assetRef` — URN of the affected or observed asset
- `detectorRef` — URN of the detecting source (NDR, EDR,
  IDS, log-analytic rule)
- `observedAt` — RFC 3339 with offset (microsecond precision
  where the source supports it)
- `mitreAttackTechnique[]` — MITRE ATT&CK technique IDs
  (e.g., `T1110.001`)
- `mitreAttackTactic[]` — tactic IDs (e.g., `TA0006`)
- `severity` — closed enum: `low`, `medium`, `high`, `critical`
- `confidence` — `low`, `medium`, `high`
- `rawEventRef` — URI of the underlying log/packet evidence
- `iocRefs[]` — URNs of IOCs that fired
- `ppiClass` — privacy-class for downstream re-disclosure

Events are deduplicated at the boundary on `(detectorRef,
rawEventRef-hash)`. Detector-vendor extensions are accepted
but MUST NOT contradict the canonical fields.

## §4 IOC record

Indicators of compromise:

- `iocId` — URN
- `iocType` — closed enum: `ipv4`, `ipv6`, `cidr`, `domain`,
  `url`, `email`, `file-hash-sha256`, `file-hash-sha1`,
  `tls-cert-fingerprint`, `mutex`, `registry-key`,
  `process-image-name`, `user-agent`, `behavior-pattern`
- `value` — the indicator value (canonicalised per type)
- `firstObserved` — RFC 3339
- `lastObserved` — RFC 3339
- `confidence` — `low`, `medium`, `high`
- `sourceRef` — URN of the contributing source
- `validUntil` — declared expiry (RFC 3339); IOCs without
  validity decay per the deployment's IOC-aging policy
- `tlpMarking` — `clear`, `green`, `amber`, `amber-strict`,
  `red` (Traffic Light Protocol 2.0)
- `relationshipsRefs[]` — URNs of related STIX objects

IOCs without a TLP marking are refused. Marking changes are
audit-chained.

## §5 Threat-intelligence object record

STIX 2.1-aligned objects representing actors, campaigns,
malware, attack patterns, and infrastructure:

- `tioId` — URN bound to a STIX `id` field
- `stixType` — `threat-actor`, `intrusion-set`, `campaign`,
  `malware`, `attack-pattern`, `infrastructure`,
  `vulnerability`, `tool`
- `name` — declared name
- `description` — narrative description
- `aliases[]` — known aliases
- `firstSeen`, `lastSeen` — RFC 3339
- `tlpMarking` — TLP 2.0 marking
- `confidence` — `low`, `medium`, `high`
- `relationships[]` — STIX SROs binding this object to others
- `sources[]` — contributing intelligence sources

The boundary canonicalises STIX objects on intake; downstream
consumers read either the STIX-native shape or the canonical
WIA shape per their preference.

## §6 Vulnerability record

CSAF 2.0-aligned vulnerability records:

- `vulnId` — URN
- `cveRef` — CVE identifier where assigned
- `productRefs[]` — affected product references (CPE)
- `severity` — CVSS v3.1 / v4 base score band (deployment
  declares which CVSS version is authoritative)
- `exploitability` — `unknown`, `unproven`, `proof-of-concept`,
  `functional`, `widespread`
- `kev` — known-exploited-vulnerabilities flag (boolean)
- `vendorAdvisoryRef` — URI of the vendor's advisory
- `csafDocumentRef` — URI of the CSAF 2.0 document
- `affectedAssetCount` — count of own assets (resolved from
  software-inventory join)
- `remediationStatus` — `pending`, `in-progress`, `mitigated`,
  `fixed`, `accepted-risk`

A vulnerability marked `accepted-risk` requires a signed
risk-acceptance record from the asset owner.

## §7 Incident record

ISO/IEC 27035-aligned incident lifecycle:

- `incidentId` — URN
- `state` — `triage`, `investigate`, `contain`, `eradicate`,
  `recover`, `lessons-learned`, `closed`
- `severity` — `low`, `medium`, `high`, `critical`
- `assetRefs[]` — affected assets
- `eventRefs[]` — contributing events
- `iocRefs[]` — related IOCs
- `tioRefs[]` — related threat-intelligence objects
- `assignedAnalystRef` — URN of the assigned analyst
- `openedAt`, `closedAt` — RFC 3339
- `mitreAttackKillChainCoverage[]` — tactics observed
- `responseActionRefs[]` — actions taken
- `lessonsLearnedRef` — post-mortem URI

Incident state transitions are signed by the assigned
analyst (or the SOC lead for cross-shift handoff). A
critical incident open beyond the deployment-declared
escalation threshold notifies the security-leadership chain.

## §8 Alert record

Alerts are the analyst-facing surface above events:

- `alertId` — URN
- `eventRefs[]` — contributing events
- `iocRefs[]` — contributing IOCs
- `severity` — alert severity (may differ from raw events
  after correlation)
- `correlationRuleRef` — URN of the correlation rule that
  fired (Sigma, vendor-native, or deployment-custom)
- `analystState` — `unread`, `triaging`, `in-incident`,
  `false-positive`, `closed-benign`
- `assignedAnalystRef` — URN
- `linkedIncidentRef` — URN of an incident if escalated

Alerts are deduplicated on a deployment-declared correlation
key. False-positive alerts feed back into rule tuning.

## §9 Response-action record

OpenC2-aligned actions executed during response:

- `actionId` — URN
- `incidentRef` — URN of the controlling incident
- `targetRef` — URN of the asset or network construct
- `actionType` — closed enum: `block-network-flow`,
  `quarantine-host`, `disable-account`, `revoke-token`,
  `kill-process`, `apply-patch`, `restore-from-backup`,
  `take-forensic-snapshot`, `re-enable` (lifting an action)
- `actionParameters` — action-specific JSON object
- `executedAt` — RFC 3339
- `executedBy` — URN of executing actor
- `outcome` — `succeeded`, `failed`, `partial`, `pending`
- `evidenceRefs[]` — URNs of supporting evidence

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain references (informative)

| Reference                     | Use site                                                 |
|-------------------------------|----------------------------------------------------------|
| WIA-pq-crypto                 | cipher-suite governance for transport                    |
| WIA-privacy                   | PPI-class controls for re-disclosure                     |
| WIA-supply-chain              | software-inventory join for vulnerability resolution     |

## Annex B — Conformance disclosure

Sections §2, §3, §4, §6, §7, §9 are mandatory; §5 (TIO) is
mandatory for any deployment receiving external intelligence
feeds; §8 is mandatory for any analyst-staffed SOC.

## Annex C — Versioning and deprecation

Versioning follows SemVer 2.0.0. STIX-aligned objects carry
the originating STIX version (2.1 by default) so partners
can verify compatibility before sharing.

## Annex D — Worked event-to-incident chain (informative)

A worked example for a credential-stuffing attack:

```json
{
  "event": {
    "eventId": "urn:wia:nsec:event:soc-x:e-991",
    "assetRef": "urn:wia:nsec:asset:soc-x:auth-1",
    "detectorRef": "urn:wia:nsec:detector:soc-x:siem-rule-cred-stuff",
    "observedAt": "2026-04-27T22:35:14.523000+09:00",
    "mitreAttackTechnique": ["T1110.004"],
    "severity": "high",
    "confidence": "high",
    "iocRefs": ["urn:wia:nsec:ioc:soc-x:ip-198.51.100.42"]
  }
}
```

The event correlates into an alert; the alert is escalated
to an incident; response actions block the source IPs and
trigger forced re-authentication for affected accounts.

## Annex E — Vendor extensions

Detector vendors may extend events and alerts with `x-vendor-*`
fields. Extensions MUST NOT contradict canonical fields and
MUST NOT be required for core conformance.

## Annex F — TLP marking enforcement

TLP markings on IOCs and TIOs flow with the records through
distribution. The boundary refuses re-distribution that
violates the marking (e.g., TLP:RED to a TLP:CLEAR partner)
and audit-chains the refusal.

## Annex G — Conformance level

Implementations declare conformance level (Surface / Verified
/ Anchored). Anchored requires a continuous evidence package
plus an annual ISO/IEC 27001 audit covering the integration
contracts in PHASE 4.

## Annex H — Worked CSAF advisory ingestion (informative)

An ingest of a vendor CSAF 2.0 advisory:

```json
{
  "vulnId": "urn:wia:nsec:vuln:cve-2026-12345",
  "cveRef": "CVE-2026-12345",
  "productRefs": ["cpe:2.3:a:vendor-a:product-b:1.4.2:*:*:*:*:*:*:*"],
  "severity": "high",
  "exploitability": "proof-of-concept",
  "kev": false,
  "vendorAdvisoryRef": "https://vendor-a.example/security/sa-2026-014",
  "csafDocumentRef": "https://vendor-a.example/csaf/sa-2026-014.json",
  "affectedAssetCount": null,
  "remediationStatus": "pending"
}
```

The boundary asynchronously resolves `affectedAssetCount`
from the software-inventory join (PHASE 4 §4) and emits a
follow-up `vuln-resolution-completed` audit-chain entry.

## Annex I — Worker-attribution carve-out

Where security events necessarily reference natural-person
identities (e.g., a credential-stuffing victim), the
`ppiClass` marker on the event drives a parallel
anonymisation policy in the boundary. Events flagged
`ppiClass=user` are accessible by analysts only after a
case-association justification, recorded in the audit chain.
Aggregation queries (e.g., counts of affected users without
identifying them) do not require the justification.
