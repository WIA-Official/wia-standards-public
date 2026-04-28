# WIA-next-gen-data-storage PHASE 4 — Integration Specification

**Standard:** WIA-next-gen-data-storage
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how WIA-next-gen-data-storage
integrates with adjacent compute, network, security,
analytics, and regulatory systems: hyperscale and on-
premises orchestrators (Kubernetes CSI, OpenStack
Cinder / Manila, VMware vSphere, Hyper-V), cloud-
native storage operators (CSI 1.x), backup and
disaster-recovery, key-management services, SIEM /
audit pipelines, ISO 16363-aligned trustworthy-
repository archives, AI/ML training pipelines, OCI
Distribution registries, S3-compatible third-party
clients, and downstream WIA companion standards.

References (CITATION-POLICY ALLOW only):
- DMTF Redfish 2024.x; SNIA Swordfish 1.2.6; SNIA SMI-S 1.8
- NVM Express Base Specification 2.0; NVMe-MI 1.2; NVMe-oF 1.1
- CXL Specification 3.1
- AWS S3 API; OCI Distribution Specification 1.1
- Kubernetes Container Storage Interface (CSI) 1.x
- OpenStack Cinder / Manila reference APIs
- VMware vSphere Storage APIs (sponsor-side integration only)
- ISO 14721 (OAIS); ISO 16363 (Trustworthy Digital Repositories)
- ISO/IEC 27040 — storage security; NIST SP 800-88 — media sanitisation
- NIST SP 800-57 (key-management); NIST FIPS 140-3
- IETF RFC 9110 (HTTP), RFC 7515 (JWS), RFC 8259 (JSON), RFC 8785 (JCS)
- W3C Provenance Notation (PROV-N) — for content-addressed lineage
- CNCF SPIFFE / SPIRE — workload identity (where bound)

---

## §1 Orchestrator integration

| Orchestrator         | Integration profile                              |
|----------------------|--------------------------------------------------|
| Kubernetes (K8s)     | Container Storage Interface (CSI) 1.x driver      |
| OpenStack            | Cinder (block) + Manila (file) drivers            |
| VMware vSphere       | vVols / VAAI integration (sponsor-side only)      |
| Hyper-V              | SMB Direct + storage QoS                         |
| Bare-metal           | NVMe-oF discovery + multipath                    |

CSI 1.x volume snapshots, clones, and resize follow
the upstream CSI spec; per-volume QoS hints publish
through CSI parameters.

## §2 Backup and disaster-recovery integration

| Pattern             | Profile                                          |
|---------------------|--------------------------------------------------|
| Snapshot + replicate| copy snapshots to a separate fault domain         |
| Active-active       | dual-site synchronous replication                 |
| Active-passive      | warm standby; promote on declared disaster        |
| Air-gapped backup   | physically isolated copy on tape / DNA archive    |
| Immutable backup    | WORM-locked snapshots (S3 Object Lock parallel)   |

DR drills emit audit events; recovery-time and
recovery-point measurements record so a regulator or
auditor sees compliance against declared RTO / RPO.

## §3 Key-management service (KMS) integration

| KMS profile         | Use                                              |
|---------------------|--------------------------------------------------|
| KMIP 2.x            | enterprise KMS interconnect                       |
| AWS KMS / Azure Key | cloud KMS                                        |
| Vault               | sponsor-internal secrets / KEK                    |
| HSM-attached        | hardware-security-module bound KEK                |
| FIPS 140-3 module   | NIST-validated module per impact level            |

The encryption-envelope record (PHASE 1 §7) cites the
KMS profile in force; KEK rotation events propagate to
the audit chain.

## §4 SIEM and audit-log integration

```
storage audit event → RFC 5424 syslog →
  SIEM ingestion → SOC monitoring
```

Storage events of interest to security:

- authentication failures
- KEK / DEK access patterns
- unusual access volume
- replication lag spikes
- WORM-policy violations
- firmware-update events
- media-sanitisation events

Events sign with the storage system's audit key so
the SIEM verifies authenticity.

## §5 OAIS / trustworthy-repository integration

For archival workloads:

```
SIP ingestion → AIP storage with replicated copies →
  scheduled fixity → DIP delivery on access request
```

ISO 16363 certification audits the institution's
preservation policies, technical infrastructure, and
governance. The WIA storage records per-AIP
preservation events back to PHASE 3 §5.

## §6 AI/ML training pipeline integration

| Tier               | Use in pipeline                                    |
|--------------------|----------------------------------------------------|
| pmem-tier          | ultra-fast feature cache                           |
| nvme-tier-0        | active training set                                |
| nvme-tier-1        | epoch-cycle training set                           |
| cap-disk           | dataset corpus                                     |
| object-tier        | dataset versioning + lineage                       |
| archive            | post-training artefact retention                   |

Training jobs cite content-addressed dataset digests
(PHASE 1 §9) so re-runs reproduce. Model checkpoints
sign with the training-job's attestation key.

## §7 OCI Distribution registry integration

| OCI client          | Profile                                          |
|---------------------|--------------------------------------------------|
| Container runtimes  | OCI Distribution v1.1                              |
| OCI image clients   | image push / pull                                 |
| OCI artefact clients| arbitrary content-addressed artefact storage       |
| Cosign / sigstore   | signature artefact storage (where the sponsor     |
|                     | uses cosign)                                      |

The implementation participates as an OCI
Distribution origin or as a pull-through cache.

## §8 S3-compatible third-party integration

| Client                 | Profile                                       |
|------------------------|-----------------------------------------------|
| AWS CLI / SDKs         | de-facto S3 contract                           |
| MinIO mc, rclone       | S3 + multi-cloud awareness                     |
| Backup software        | per-vendor S3 binding                         |
| Lakehouse engines      | S3 + Iceberg / Delta / Hudi table format       |
| CDN origin             | S3 origin with signed URLs                     |

The implementation honours signed-URL delegation
(per S3) so end-to-end auth bypasses the storage
fabric for short-lived access.

## §9 Cross-domain WIA bindings

| Companion standard           | Binding purpose                                |
|------------------------------|------------------------------------------------|
| WIA-data-warehouse           | analytical-tier consumption                    |
| WIA-data-lake                | object-tier ingestion                           |
| WIA-edge-ai                  | tiered model serving                            |
| WIA-quantum                  | post-quantum key management (where bound)       |
| WIA-data-encryption          | DEK / KEK lifecycle                             |
| WIA-data-lineage             | artefact lineage                                |
| WIA-data-portability         | artefact export                                 |
| WIA-content-ai               | model artefact storage                          |
| WIA-digital-time-capsule     | deep-cold-archive bridge                        |

Each binding identifies the consumed PHASE.

## §10 Long-term archival

| Tier / context           | Retention                                    |
|--------------------------|----------------------------------------------|
| Fast online              | sponsor policy                                |
| Capacity / cap-disk      | sponsor policy                                |
| Tape archive             | typically ≥ 10 years                          |
| Optical / holographic    | typically ≥ 30 years                          |
| DNA / polymer            | per substrate manufacturer warranty (long)    |
| Audit logs               | per regulator (typ. ≥ 5-7 years)              |
| Sanitisation certificates| per data-classification (≥ 3 years)            |

## §11 Conformance test suite

The reference test suite covers:

- CSI 1.x volume create / snapshot / clone / resize
- Swordfish / Redfish discovery + capability advertisement
- NVMe-oF discovery and namespace attach
- ZNS zone-management commands
- replication-lag measurement under network impairment
- KMS rotation triggers DEK re-encrypt
- WORM-policy enforcement on object mutation
- fixity-check failure → repair-from-replica
- firmware-update rollback on validation failure
- NIST SP 800-88 sanitisation event with certificate

## §12 Internationalisation

Public-API responses honour `Accept-Language` for
human-readable error messages; resource identifiers
(domain / namespace / volume) are language-neutral.

## §13 Security and privacy posture

- Transport: TLS 1.3 with mutual TLS for orchestrator
  ↔ storage and storage ↔ KMS exchanges
- Authentication: SPIFFE / SPIRE workload identity
  (where bound); client_credentials with key
  attestation otherwise
- At-rest: AES-256-GCM (or XTS-AES per IEEE 1619-2018
  for legacy block); per-volume DEK with KMS-managed
  KEK
- Audit: tamper-evident chain (PHASE 3 §9) exportable
  per ISO/IEC 27037
- Privacy: data-erasure executes via cryptographic
  erase (DEK destruction); object-store immutable
  retention honours WORM policy
- FIPS 140-3 module bound where regulator requires;
  module identifier records on the encryption-
  envelope

## §14 Operational metrics

Sponsors / operators report (informationally) on the
WIA registry:

- per-tier capacity / utilisation
- IOPS / throughput / latency vs. target
- replication lag distribution
- fixity-failure rate
- firmware-update success rate
- DR drill outcomes
- sanitisation events

## §15 Recovery and continuity

- Storage-domain outage — orchestrator routes around
  unhealthy domain; replicas serve reads where
  possible
- KMS outage — pre-fetched DEKs serve in-flight
  workload; new writes block until KMS recovers
- network outage — synchronous replication degrades
  to async with bounded staleness; alerts emit
- catastrophic data loss — declared-disaster recovery
  from replicas / backups / archive per the DR plan

## Annex A — Worked end-to-end example (informative)

A hyperscale operator deploys a 60-petabyte
disaggregated NVMe pod with CXL.mem-attached SCM tier.
Kubernetes consumes the array via CSI 1.x; AI/ML
training pipelines pin feature-cache to the SCM tier
and corpus to NVMe-tier-0. Encryption envelopes use a
sponsor-internal KMS with HSM-bound KEKs. Fixity scrub
runs weekly; a single-drive bit-rot detected on day
197 repairs from parity within four hours. A model-
release event publishes the trained checkpoint to the
OCI Distribution origin with a Sigstore-signed
manifest. End-of-life decommission of an older pod
runs NIST SP 800-88 purge with cryptographic erase
followed by physical degaussing for HDD components;
a sanitisation certificate records.

## Annex B — Conformance disclosure

Implementations declare the orchestrator integrations
supported, the KMS profile, the OCI Distribution
version, the S3 API version honoured, the CSI 1.x
revision implemented, and the FIPS 140-3 module
identifiers in service. Disclosure is machine-
readable at `/.well-known/wia-storage-conformance.
json`.

## Annex C — Versioning

Adding a new orchestrator integration is minor;
changing the CSI 1.x revision is major.

## Annex D — Federated-storage federation

Multi-cloud / multi-region storage federations expose
a federation directory:

| Pattern             | Purpose                                          |
|---------------------|--------------------------------------------------|
| Geo-redundant       | replicate across geographic regions               |
| Multi-cloud         | replicate across cloud providers                  |
| Hybrid              | on-prem + cloud federation                        |
| Edge-to-cloud       | edge ingest + cloud archival                      |

Federation events emit to the audit chain so an
operator sees each replica's location and ingestion
time.

## Annex E — Cost / billing integration

Cost-allocation tags propagate from the orchestrator
to the namespace record so a tenant's bill reflects
actual storage consumption per tier. Egress and
operations costs follow the cost-policy record (PHASE
1 Annex E).
