# WIA-next-gen-data-storage PHASE 1 — Data Format Specification

**Standard:** WIA-next-gen-data-storage
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for next-
generation storage systems covering NVMe over Fabrics
(NVMe-oF), Storage-Class Memory (SCM), Compute Express
Link memory expansion (CXL.mem), Zoned Namespace SSDs
(ZNS), DNA / synthetic-polymer archival storage,
holographic and optical-disc storage, S3-compatible
object stores, and OCI-distribution image registries.
Records describe storage capacity, namespace topology,
write / read performance, durability budgets, fault
domains, encryption-at-rest envelopes, content-addressed
artefact catalogues, and the cross-references binding
storage tiers to higher-level data services.

References (CITATION-POLICY ALLOW only):
- SNIA SMI-S 1.8 — Storage Management Initiative Specification
- SNIA Swordfish (DMTF Redfish-aligned storage extension) 1.2.6
- DMTF Redfish 2024.x
- NVM Express Base Specification 2.0; NVMe over Fabrics 1.1
- NVMe ZNS (TP 4053a) — Zoned Namespace Command Set
- NVMe-MI (Management Interface) 1.2
- SNIA Persistent Memory Specification 1.0; SNIA NVM Programming Model 1.2
- CXL Specification 3.1 (Compute Express Link)
- IEEE 11073-10101 (medical device nomenclature; where storage binds clinical use)
- ISO/IEC JTC 1 SC 23 — Digital Recording Media (optical / DNA storage)
- AES-256-GCM (NIST FIPS 197 + SP 800-38D); ESSIV; XTS-AES per IEEE 1619-2018
- AWS S3 API (de-facto industry); S3 Multi-Region Access Points (informative)
- OCI Distribution Specification 1.1 / OCI Image Specification 1.1
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 7515 (JWS), RFC 9530 (Content-Digest)

---

## §1 Scope

This PHASE applies to storage systems serving primary
workload (block / file / object), data-intensive
analytics (data lakehouse), AI/ML training and serving
(GPU-attached storage), edge / archival workloads, and
emerging substrates (DNA / polymer storage, holographic).

In scope: storage-domain record, namespace record,
volume / file-system / bucket record, performance
profile record, durability / availability record,
encryption-envelope record, replication-topology
record, content-addressed artefact catalogue, and
storage-tier classification. Out of scope: per-
application schema (handled by domain standards) and
sponsor-internal storage tooling implementation
(handled by vendor product specifications).

## §2 Storage-domain record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `domainRef`          | UUID (RFC 4122)                                 |
| `topology`           | `single-controller`, `dual-controller`,         |
|                      | `scale-out`, `disaggregated`, `cxl-pod`,        |
|                      | `composable`                                     |
| `mediaProfile`       | NAND TLC / QLC / PLC; SCM (3D-XPoint, MRAM,     |
|                      | ReRAM); SLC-cache; HDD; tape; DNA / polymer;    |
|                      | holographic; optical-disc                        |
| `interconnect`       | NVMe-oF (RDMA / TCP), CXL.mem, FC, iSCSI,       |
|                      | InfiniBand, Ethernet                             |
| `mgmtProtocol`       | SMI-S / Swordfish / Redfish / S3                  |
| `vendorRef`          | manufacturer + product + firmware version        |
| `installedAt`        | ISO 8601                                        |
| `lifecycleStatus`    | `commissioning`, `production`, `degraded`,      |
|                      | `retiring`                                       |

## §3 Namespace record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `namespaceRef`       | UUID                                            |
| `domainRef`          | §2                                              |
| `namespaceKind`      | `nvme-namespace`, `zns-namespace`,               |
|                      | `s3-bucket`, `oci-repository`, `pmem-region`,    |
|                      | `cxl-region`, `tape-pool`, `dna-pool`            |
| `capacityBytes`      | numeric                                          |
| `lbaSize`            | for block: 512 / 4096 / 8192                     |
| `accessMode`         | read-only / read-write / write-once-read-many    |
|                      | (WORM)                                            |
| `featureSet`         | per-namespace feature flags (e.g. NVMe          |
|                      | NVMe-Set, write-zeroes, copy-namespace)          |
| `endurancePlan`      | terabytes-written budget over service life       |

## §4 Volume / file-system / bucket record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `volumeRef`          | UUID                                            |
| `namespaceRef`       | §3                                              |
| `kind`               | `block-volume`, `file-system`, `object-bucket`, |
|                      | `oci-repo`, `kv-store`, `archive-collection`     |
| `mountPath`          | for filesystem-style consumers                   |
| `capacityBytes`      | provisioned                                      |
| `usedBytes`          | current consumption                              |
| `quota`              | hard / soft quota per consumer                   |
| `versioning`         | enabled / disabled (object stores)              |
| `objectLockMode`     | per-bucket lock mode (S3 Object Lock,           |
|                      | governance / compliance)                         |

## §5 Performance-profile record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `performanceRef`     | UUID                                            |
| `namespaceRef`       | §3                                              |
| `iopsTarget`         | rated IOPS per IO block size                     |
| `throughputTarget`   | bytes per second per direction                   |
| `latencyTarget`      | per-percentile (p50 / p99 / p99.9 / p99.99)      |
| `qosClass`           | `tier-0`, `tier-1`, `tier-2`, `tier-archive`     |
| `bandwidthLimit`     | per-consumer bandwidth cap                       |
| `iopsLimit`          | per-consumer IOPS cap                            |

Performance profiles bind to QoS controllers for fair-
share enforcement.

## §6 Durability / availability record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `durabilityRef`      | UUID                                            |
| `namespaceRef`       | §3                                              |
| `replicaCount`       | per-replica-mode integer (e.g. 3 for RF3)        |
| `replicaTopology`    | `same-node`, `same-rack`, `same-zone`,           |
|                      | `multi-zone`, `multi-region`, `multi-continent`  |
| `parityScheme`       | `none`, `RAID-1`, `RAID-5`, `RAID-6`,            |
|                      | `RAID-Z`, `erasure-coding-{k,m}`                  |
| `targetDurabilityNines`| informative target (e.g. 11 nines)              |
| `targetAvailabilityNines`| informative target (e.g. 4 nines)              |
| `faultDomainTopology`| explicit JSON tree of failure domains            |

## §7 Encryption-envelope record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `envelopeRef`        | UUID                                            |
| `namespaceRef`       | §3                                              |
| `kekRef`             | KMS key-encryption-key reference                 |
| `dekScheme`          | per-volume DEK / per-extent DEK / per-object DEK|
| `cipher`             | AES-256-GCM (NIST FIPS 197) / XTS-AES (IEEE     |
|                      | 1619-2018) / ChaCha20-Poly1305 (RFC 8439)       |
| `tweakScheme`        | for XTS: ESSIV / sequential                      |
| `wrapAlgorithm`      | NIST KW / KWP / GCM-KW                           |
| `rotationPolicyRef`  | rotation policy reference                        |

Envelope bindings are sponsor-controlled; the
implementation never stores plaintext DEKs at rest.

## §8 Replication-topology record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `topologyRef`        | UUID                                            |
| `mode`               | `synchronous`, `asynchronous`, `quorum`,         |
|                      | `chain-replication`, `paxos`                     |
| `members[]`          | participating namespace references               |
| `consistency`        | `strong`, `bounded-staleness`, `prefix`,         |
|                      | `eventual`                                       |
| `recoveryPointObjective`| RPO seconds                                     |
| `recoveryTimeObjective`| RTO seconds                                     |
| `splitBrainPolicy`   | quorum policy on partition                       |

## §9 Content-addressed artefact catalogue

For object / OCI / archive workloads:

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `artefactRef`        | UUID                                            |
| `digestAlgorithm`    | SHA-256 / BLAKE3 / SHA-512                       |
| `digest`             | hex-encoded                                     |
| `byteSize`           | numeric                                         |
| `mimeType`           | per IANA registry                                |
| `mediaTypeOci`       | per OCI Image / Distribution media types         |
| `metadata`           | per-artefact metadata (JSON / OCI manifest)      |
| `licenceRef`         | SPDX licence identifier                          |

## §10 Storage-tier classification

| Tier label           | Characteristics                                 |
|----------------------|-------------------------------------------------|
| `pmem-tier`          | persistent-memory; ns-latency; CXL.mem profile   |
| `nvme-tier-0`        | NVMe SSD; <100 µs p99 read                       |
| `nvme-tier-1`        | NVMe SSD; <500 µs p99 read                       |
| `cap-disk`           | high-capacity HDD                               |
| `tape-archive`       | LTO-9 / LTO-10                                   |
| `optical-archive`    | optical disc, MO, holographic                    |
| `dna-archive`        | DNA / polymer; cold; very high density           |

Tier classification is sponsor-policy; per-tier
performance / durability / cost characteristics are
recorded so a placement engine can assign workload to
tier.

## §11 Cross-domain references (informative)

- WIA-data-warehouse — for analytical-tier consumption
- WIA-data-lake — for object-tier ingestion
- WIA-edge-ai — for tiered model serving
- WIA-quantum — for post-quantum key management
- WIA-data-encryption — for DEK / KEK lifecycle

## Annex A — Worked CXL.mem region (informative)

```json
{
  "namespaceRef": "ns-cxl-pod-A-region-1",
  "namespaceKind": "cxl-region",
  "capacityBytes": 274877906944,
  "performanceRef": "perf-cxl-region-1-100ns-p99"
}
```

## Annex B — Conformance disclosure

Implementations declare the management protocols
served (SMI-S, Swordfish, Redfish, S3, NVMe-MI, OCI
Distribution), the encryption ciphers supported, the
durability tiers offered, the OCI media types
honoured, and the CXL revision implemented.

## Annex C — Versioning

Field additions are minor; semantic redefinition is
major.

## Annex D — Data placement-policy record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `placementRef`       | UUID                                            |
| `policyKind`         | `tier-by-access`, `tier-by-age`, `tier-by-      |
|                      | classification`, `tier-by-cost`, `manual`        |
| `triggerThresholds`  | per-rule trigger (e.g. last-access > 30 days)    |
| `actionMatrix`       | rule → tier transition map                       |
| `auditCadence`       | how often the policy re-runs                     |

Placement events emit on the audit chain so an
operator sees what moved when, why, and to where.

## Annex E — Storage-cost policy record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `costPolicyRef`      | UUID                                            |
| `tierCostBps`        | per-tier basis-points cost per byte-month        |
| `egressCostBps`      | egress-cost per byte where applicable            |
| `breakevenAnalysis`  | tier-promotion breakeven cadence                 |

Cost policies feed the placement engine so a tier
promotion / demotion decision balances performance
vs. cost.

## Annex F — Backwards-compatibility commitments

The data format honours:

- legacy LBA sizes (512 / 4096) for new namespaces on
  vendor product lines that still ship 512-LBA-only
  drives
- S3 v2 signature for legacy clients (deprecated; new
  clients use SigV4)
- OCI Image Specification 1.0 for clients that have
  not migrated to 1.1

Deprecation notices publish on the implementation's
public deprecation page.
