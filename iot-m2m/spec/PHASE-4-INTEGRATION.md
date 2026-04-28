# WIA-iot-m2m PHASE 4 — INTEGRATION Specification

**Standard:** WIA-iot-m2m
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an IoT-M2M deployment integrates
with the systems that surround it: oneM2M-aligned service
platforms; LwM2M-aligned servers; MQTT brokers; W3C WoT
discovery directories; mobile-network-operator integrations
(for cellular IoT deployments); LPWAN network servers (for
LoRaWAN and Sigfox deployments); device-management vendors;
HSM and PKI providers; analytics and time-series database
back-ends; and long-term archives.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- W3C WoT TD 1.1 / WoT Discovery 1.1
- oneM2M TS-0001 / TS-0008 / TS-0009
- OMA LwM2M v1.2
- OASIS MQTT v5.0
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 oneM2M Service Platform Integration

oneM2M-aligned deployments integrate with the operator's
chosen oneM2M Service Platform (commercial vendor
implementations or open-source platforms such as ACME or
Mobius). Integration carries the platform's identifier,
the per-deployment CSE base URI, the per-AE registration
records, and the per-subscription notification endpoints.

## §2 LwM2M Server Integration

LwM2M deployments integrate with an OMA-aligned LwM2M
server (Leshan, Anjay-Server, vendor-managed servers).
Integration carries the server's identifier, the per-
device bootstrap configuration, the per-Object Definition
support matrix, and the per-observation notification
configuration.

## §3 MQTT Broker Integration

MQTT-broker deployments integrate with MQTT v5.0 brokers
(VerneMQ, Mosquitto, HiveMQ, EMQX, AWS IoT Core, equivalent
managed services). Integration carries the broker's
identifier, the per-deployment topic-namespace allocation,
the per-client authentication scheme (MQTT v5.0 enhanced
authentication), and the per-topic ACL.

## §4 W3C WoT Discovery Integration

Operators that publish device discovery metadata to a W3C
WoT Discovery directory integrate through the directory's
exploration interface. Integration carries the directory's
identifier, the per-deployment discovery scope, and the
per-TD-class crawl-cadence the directory honours.

## §5 Mobile-Network-Operator Integration (Cellular IoT)

Cellular-IoT deployments integrate with the mobile-network
operator (MNO) via the operator's eSIM / IoT SIM management
platform. Integration carries the MNO's identifier, the
per-SIM profile-management endpoint, the per-device data
plan, and the operator's roaming policy. eUICC profile
swaps follow GSMA RSP (Remote SIM Provisioning) rules.

## §6 LPWAN Network Server Integration

LoRaWAN deployments integrate with a LoRaWAN Network Server
(LNS): The Things Stack, ChirpStack, vendor-managed LNS.
Integration carries the LNS's identifier, the per-deployment
application-server registration, the per-device JoinEUI /
DevEUI / AppKey configuration (keys held in HSM,
references-only here), and the LNS-side telemetry-uplink
endpoint.

Sigfox deployments integrate with the Sigfox network's API
through the operator's Sigfox account; integration carries
the Sigfox account identifier and the per-device
provisioning endpoint.

## §7 Device-Management Vendor Integration

Device-management vendors (LwM2M-based vendors, MQTT-based
vendors, custom platforms — e.g. AWS IoT Device Management,
Azure IoT Hub, Google Cloud IoT, Bosch IoT Suite) consume
device records through the API. Integration carries the
vendor's identifier, the per-device-class management
contract, and the FOTA-orchestration interface that the
vendor offers.

## §8 HSM and PKI Provider Integration

Operators integrate with HSM providers (Thales, Utimaco,
nCipher, Yubico, AWS CloudHSM, equivalent) and PKI
providers (DigiCert, GlobalSign, Sectigo, ACME-compatible
CAs, operator-internal Internal CA) for root-key custody
and certificate issuance. Integration carries each
provider's identifier and the per-key-class custody
arrangement.

## §9 Analytics and Time-Series Database Integration

Telemetry flows to analytics back-ends (InfluxDB,
TimescaleDB, Prometheus + remote-write, ClickHouse, AWS
Timestream, Google BigQuery, equivalent platforms).
Integration carries the back-end's identifier, the per-
telemetry-class ingestion endpoint, and the operator's
per-vertical retention SLA.

## §10 Long-Term Archive Integration

Operators designate a long-term archive that holds
deployment records (audit logs, FOTA campaign histories,
quality-dossier snapshots) beyond the operator's primary
retention horizon. Quarterly deposits round-trip content-
addresses; on deployment wind-down, remaining records
transfer to the archive with content-addresses preserved.

## §11 Evidence Package Format

```
evidence/
  manifest.json              — package manifest (signed)
  deployment.json            — deployment record
  devices/                   — per-device records and TD
                                history
  resource-tree-snapshots/   — periodic snapshots of the
                                deployment's resource tree
  telemetry-summaries/       — telemetry summaries for the
                                cited interval (raw telemetry
                                in the analytics back-end,
                                referenced by content-address)
  actuations/                — actuation history
  fota/                      — FOTA campaign history
  audit/                     — API audit log excerpts
```

The package is content-addressable; the manifest is signed
by the operator's HTTP-message-signature key (RFC 9421) and
counter-signed by the operator's quality manager.

## §12 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:iot-m2m:evidence-mismatch`.

## §13 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-iot-m2m` that links to the API root, the
service-layer alignment summary, the published quality
dossier, the deployment's well-known WoT discovery
directory, and the public catalogue of Thing Description
templates.

## §14 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (spectrum
licence status, ISO/IEC 27001 certification, FOTA campaign
success rate) to consumers of W3C Verifiable Credentials
MAY re-issue the attestations as Verifiable Credentials
under the Data Model 2.0 specification. Re-issuance is
optional; the canonical record remains the JSON evidence-
package manifest.

## §15 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds with
`Last-Event-ID` resume support. Subscribers that disconnect
during long FOTA campaigns or telemetry ingest windows
resume from the last seen event identifier without losing
visibility of priority-1 events.

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with
prior-minor clients. Major revisions go through a
deprecation window of at least one full oneM2M / LwM2M /
W3C WoT release cycle so that consuming platforms have
time to migrate.

## §17 Cross-Standard Linkage

Operators that consume adjacent WIA standards (WIA-
industrial-iot, WIA-smart-city, WIA-healthcare-remote-
monitoring, WIA-electric-grid-edge) emit cross-standard
linkage records that name the consuming standard and the
version under which the linkage is claimed.

## §18 Reader Tooling

Operators MAY publish supplementary reader tools (real-
time device-population health dashboards, FOTA campaign
roll-out visualisers, telemetry browse-and-query consoles)
alongside the canonical evidence package; the tools are
non-normative.

## §19 Public Catalogue and Aggregator Feeds

Operators that publish a public catalogue of supported
Thing Description templates and per-vertical-domain
deployment patterns emit a JSON Feed listing the templates
with their evidence-package manifest digests and the
operator-recommended deployment guidance.

## §20 Vulnerability Coordination Body Integration

Operators that handle coordinated-vulnerability disclosures
(PHASE-3 §15) integrate with vulnerability-coordination
bodies (CISA in the US, ENISA in the EU, KrCERT/CC in
Korea, JPCERT/CC in Japan, equivalent national CERTs).
Integration carries the body's identifier, the per-disclosure
intake reference, and the operator's coordinated-disclosure
SLA.

## §21 GSMA RSP Integration (Cellular IoT)

Cellular-IoT deployments using eUICC-based eSIM follow
the GSMA Remote SIM Provisioning architecture for
profile management. Integration carries the GSMA-aligned
SM-DP+ provider identifier, the SM-DS provider identifier,
and the per-device profile-management workflow.

## §22 Battery-Telemetry Aggregator Integration

Constrained-device deployments emit per-device battery /
energy telemetry that the operator consumes through a
fleet-management dashboard. Integration carries the
dashboard's identifier, the per-device-class battery model,
and the operator's replacement-schedule planner.

## §23 Edge-Compute Platform Integration

Operators that run edge-compute workloads (gateway-side
analytics, local rule-engines, on-device ML inference)
integrate the edge-compute platform (KubeEdge, Azure IoT
Edge, AWS Greengrass, Open Horizon, equivalent platforms).
Integration carries the platform's identifier, the per-
deployment-vertical workload allocation, and the per-
workload over-the-air-update channel.

## §24 OTA Workflow Vendor Coordination

Operators that work with OTA-orchestration vendors (Mender,
Memfault, Foundries.io, Pelion, equivalent platforms)
integrate the vendor's campaign-management workflow with
the operator's FOTA discipline (PHASE-3 §6). The
integration carries the vendor's identifier, the per-
campaign workflow contract, and the per-vendor success-
metric mapping.

## §25 Conformance and Sunset

A deployment conformant with PHASE-4 has integrated
successfully with the operator's chosen service-layer
platform (oneM2M, LwM2M, MQTT, or WoT-native), at least
one analytics back-end, the operator's HSM and PKI, and
at least one long-term archive, and has published at
least one externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before
removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-iot-m2m
- **Last Updated:** 2026-04-28
