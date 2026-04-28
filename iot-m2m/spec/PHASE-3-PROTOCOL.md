# WIA-iot-m2m PHASE 3 — PROTOCOL Specification

**Standard:** WIA-iot-m2m
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an IoT-M2M
deployment: spectrum and radio licensing, oneM2M / LwM2M
service-layer alignment, device-identity provisioning, root-
key management, firmware-signing discipline, FOTA campaign
governance, telemetry retention, actuation authorisation,
W3C WoT TD stewardship, decommissioning, and end-of-
deployment data disposition.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO/IEC 30141:2018 (IoT Reference Architecture)
- ISO/IEC 21823 (IoT interoperability)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9147 (DTLS 1.3)
- IETF RFC 7252 (CoAP)
- IETF RFC 8949 (CBOR)
- IETF RFC 9457 (Problem Details)
- oneM2M TS-0001 / TS-0003 (security solutions)
- OMA LwM2M v1.2
- OASIS MQTT v5.0
- W3C WoT TD 1.1
- W3C WoT Architecture 1.1
- IEC 62443 (industrial cybersecurity, where the deployment
  intersects industrial-control environments)

---

## §1 Spectrum and Radio Licensing

Operators that deploy radio-using devices (Wi-Fi, Bluetooth,
Zigbee, Z-Wave, LoRaWAN, NB-IoT, Cat-M, 5G RedCap) MUST
ensure each device's radio operation falls within the
operator's per-jurisdiction radio-band authorisation. ISM-
band devices follow the local ISM-band regulations (FCC
Part 15 in the US, EU EN 300 220 / EN 300 328, KCC
notifications in Korea, equivalent rules elsewhere);
licensed-band devices follow the operator's mobile-
network-operator agreement.

Authorisation expirations or revocations halt new device
provisioning under the affected band.

## §2 Service-Layer Alignment

Deployments that claim oneM2M alignment honour TS-0001
(Functional Architecture) for the resource-tree shape, TS-
0004 for the service-layer protocol, TS-0008 / TS-0009 for
CoAP / HTTP bindings, and TS-0003 for the security
solution. Deployments that claim LwM2M alignment honour
OMA LwM2M v1.2 for the registration / object / observation
model. MQTT-broker deployments honour OASIS MQTT v5.0.

The service-layer alignment is recorded against the
deployment (PHASE-1 §2 `serviceLayerKind`); custom
deployments document the alignment delta against the
nearest standard.

## §3 Device-Identity Provisioning

Devices receive their identity through the operator's
provisioning workflow:

- per-device root-key generation in the manufacturer's
  HSM at production;
- per-device certificate issuance from the operator's
  Internal Certificate Authority (or a recognised public
  CA for cross-operator deployments);
- per-device first-bootstrap to the operator's
  bootstrapping service (LwM2M Bootstrap Server, oneM2M
  M2M Service Subscription, or operator-internal
  equivalent);
- per-device runtime credential rotation per the
  operator's rotation cadence.

Provisioning workflows record the per-device root-key
derivation reference (PHASE-1 §3) but never the key
material; key material is held in HSMs.

## §4 Root-Key Management

Root keys are derived per device through the operator's
key-derivation function (e.g. NIST SP 800-108 KDF,
operator-specified KDF). Key-derivation records are
auditable through the operator's HSM audit log; the WIA
DATA-FORMAT layer carries only the derivation reference,
never the derived key.

Key-rotation policy:

- per-device runtime keys rotate at the operator's
  cadence (typically every 90 days for cellular, longer
  for ISM-band low-bandwidth deployments);
- per-device root keys rotate only on device service-
  visit events (manufacturer recall, decommission,
  re-commission to a different deployment);
- compromised keys revoke through the operator's CRL /
  OCSP or device-blocklist channel.

## §5 Firmware-Signing Discipline

Firmware images are signed by the operator under a
signing-key chain rooted in the operator's firmware-
signing root key (held in HSM, with separation-of-duties
between signing-key custodian and firmware author).
Signing covers:

- the firmware image bytes (signed manifest with image
  digest);
- the firmware version;
- the target device class;
- the signing date and the signing-key identifier.

Devices that load firmware verify the signature before
applying; signatures that fail verification cause the
device to refuse the firmware and emit an audit event
that the operator's lifecycle-management service ingests.

## §6 FOTA Campaign Governance

FOTA campaigns (PHASE-1 §8) follow the operator's roll-out
SOP:

- staged roll-out with per-stage device population (e.g.
  first 1% of fleet, then 10%, then 50%, then 100%);
- per-stage success-rate gating (the next stage proceeds
  only if the prior stage's success rate meets the
  operator's threshold);
- per-campaign rollback procedure with rollback-evidence
  capture (PHASE-1 §8 `rollbackEvidenceRef`);
- per-campaign post-mortem when rollback is invoked.

## §7 Telemetry Retention

Telemetry retention follows the per-vertical-domain
requirements:

- safety-critical telemetry (industrial-control,
  healthcare-remote-monitoring, fleet-telematics): retain
  per the operating jurisdiction's regulator rules
  (typically multi-year);
- consumer-home-automation telemetry: retain per the
  operator's privacy policy and the operating jurisdiction's
  consumer-data law (typically the shorter of operator
  policy and 3 years);
- business-internal telemetry: retain per the operator's
  records-management policy.

The operator's per-vertical retention policy is recorded
against each resource-tree node's `retentionPolicyRef`
(PHASE-1 §5).

## §8 Actuation Authorisation

Actuations (PHASE-1 §7) follow the operator's authorisation
matrix:

- per-application-server scope of allowed actuations;
- per-device class of actuations the device is willing
  to accept;
- per-time-of-day or per-context restrictions for
  safety-affecting actuations.

Actuation requests that violate the authorisation matrix
are rejected at the API gate with type
`urn:wia:iot-m2m:actuation-not-authorised` and audited.

## §9 W3C WoT TD Stewardship

Thing Descriptions (PHASE-1 §4) are content-addressed and
versioned. Stewardship discipline:

- per-device-class TD template approved by the operator's
  TD reviewer;
- per-revision content-address pinning so that consuming
  applications resolve a known TD revision;
- per-revision deprecation notice when a backwards-
  incompatible TD revision retires affordances.

## §10 Decommissioning

Device decommissioning follows the operator's secure-decom
procedure:

- per-device root-key revocation through the operator's
  PKI;
- per-device CRL / blocklist update so that the service
  layer rejects any further contact;
- per-device data disposition per the operator's data-
  retention policy (telemetry retained per §7, device
  metadata frozen and retained for audit-trail
  continuity);
- per-device physical disposition where applicable
  (operator-collection, customer-return, certified
  destruction).

## §11 End-of-Deployment Data Disposition

Deployments that wind down transition every device to
`decommissioned`, then archive deployment records to the
long-term archive (PHASE-4 §10). End-of-deployment data
disposition follows the operator's per-vertical policy
and the operating jurisdiction's data-protection rules.

## §12 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4); device-
side clocks synchronise per the device-class capability
(NTPv4 for non-constrained, simplified time sync per CoAP
extension or SNTP-lite for constrained). Constrained
devices that lack a real-time clock annotate telemetry
with a relative-time stamp that the gateway converts to
absolute time on ingest.

## §13 Privacy

IoT-M2M deployments often process personal data
(consumer-home-automation, healthcare-remote-monitoring,
fleet-telematics with driver-identifiable signals). The
operator's data-protection policy follows ISO/IEC
27701:2019 and the operating jurisdiction's privacy law
(GDPR, K-PIPA, CCPA, equivalent rules elsewhere). The
DATA-FORMAT layer carries device identifiers as opaque
UUIDs; binding to subject identity (where applicable) is
held in the operator's CRM / IDP.

## §14 Quality Dossier

The operator's quality dossier records the spectrum
authorisations, the service-layer alignment, the firmware-
signing PKI, the FOTA campaign history, the per-vertical
retention policy, the W3C WoT TD reviewer register, and
the operator's incident history. The dossier is reviewed
at least annually by the operator's quality manager.

## §15 Cybersecurity and Vulnerability Disclosure

Operators that deploy devices in industrial-control or
critical-infrastructure contexts honour IEC 62443 zone
classifications and the operating jurisdiction's cybersecurity
disclosure rules (NIS 2 in the EU, KISA notification rules
in Korea, CIRCIA in the US, equivalent regimes elsewhere).

Per-device-class vulnerability disclosures follow the
operator's coordinated-vulnerability-disclosure policy:

- a vulnerability researcher submits a finding through the
  operator's published intake channel;
- the operator triages the finding and prepares a fix
  (firmware update, configuration change, replacement
  hardware where unavoidable);
- the operator coordinates disclosure timing with the
  researcher and with affected customers;
- public disclosure honours the agreed embargo window so
  that operator and customers have time to deploy the
  fix.

## §16 Cross-Vertical Reuse Discipline

Operators that reuse infrastructure across multiple
verticals (a smart-city operator running smart-building
sub-deployments, a utility running both metering and
distribution-automation deployments) maintain per-vertical
authorisation isolation: an application server authorised
for one vertical cannot, through cross-vertical resource
access, leak telemetry from another vertical without
explicit re-authorisation.

## §17 Energy and Power Budget Discipline

Constrained-class devices operate under tight energy budgets
(battery-powered sensors, energy-harvesting nodes). The
operator's energy-budget discipline records:

- per-device-class expected battery life under nominal duty
  cycle;
- per-deployment communication-pattern impact (frequent
  uplinks vs sparse uplinks dramatically affect battery
  life);
- per-device replacement / recharge schedule.

FOTA campaigns and aggressive telemetry-rate increases
require energy-budget review before launch; the operator's
fleet-management dashboard tracks per-device-class battery
trajectory against the operator's replacement plan.

## §18 Quality Dossier

The operator's quality dossier records the spectrum and
radio authorisations, the chosen service-layer alignment,
the firmware-signing PKI, the FOTA-campaign history, the
per-vertical retention policy, the W3C WoT TD reviewer
register, the per-device-class battery-life model, and the
operator's incident history. The dossier is reviewed at
least annually by the operator's quality manager.

## §19 Conformance and Auditing

A deployment conformant with WIA-iot-m2m publishes its
spectrum authorisations, its service-layer alignment, the
FOTA campaign catalogue, the per-vertical retention policy,
and the catalogue of Thing Description templates, and
answers an annual self-assessment that maps each clause
of this PHASE to the operator's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-iot-m2m
- **Last Updated:** 2026-04-28
