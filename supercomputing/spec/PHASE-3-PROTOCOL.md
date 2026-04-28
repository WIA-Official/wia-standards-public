# WIA-COMP-001 — Phase 3: Protocol

> Supercomputing canonical Phase 3: protocols (storage + resource-management + programming-model + performance-monitoring + fault-tolerance + energy-management).

# WIA-COMP-001: Supercomputing Specification v1.0

> **Standard ID:** WIA-COMP-001
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Computing Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [System Architecture](#2-system-architecture)
3. [Compute Resources](#3-compute-resources)
4. [Interconnect Networks](#4-interconnect-networks)
5. [Storage Systems](#5-storage-systems)
6. [Programming Models](#6-programming-models)
7. [Performance Metrics](#7-performance-metrics)
8. [Benchmarking](#8-benchmarking)
9. [Resource Management](#9-resource-management)
10. [Energy Efficiency](#10-energy-efficiency)
11. [Fault Tolerance](#11-fault-tolerance)
12. [Implementation Guidelines](#12-implementation-guidelines)
13. [References](#13-references)

---


## 4. Interconnect Networks

### 4.1 Network Technologies

**InfiniBand:**
- Generations: HDR (200 Gb/s), NDR (400 Gb/s), XDR (800 Gb/s)
- Latency: <1 μs
- Features: RDMA, GPUDirect, adaptive routing

**Ethernet:**
- Speeds: 100/200/400 GbE
- Protocols: RoCE (RDMA over Converged Ethernet)
- Latency: 1-5 μs

**Custom Fabrics:**
- Examples: Slingshot, Tofu, Torus
- Optimized for specific workloads
- Proprietary optimizations

### 4.2 Topology Requirements

**Bandwidth:**
- Injection bandwidth: ≥200 Gb/s per node
- Bisection bandwidth: ≥50% aggregate
- All-to-all: Dragonfly or similar

**Latency:**
- MPI point-to-point: <1 μs
- Collective operations: <10 μs (small messages)
- RDMA read/write: <500 ns

### 4.3 Communication Patterns

**Message Sizes:**
```
Small messages (<1 KB): Latency-sensitive
Medium messages (1 KB - 1 MB): Bandwidth and latency
Large messages (>1 MB): Bandwidth-bound
```

**MPI Collectives:**
- Broadcast, Scatter, Gather
- All-to-all, Reduction operations
- Optimized for topology awareness

---



## 5. Storage Systems

### 5.1 Parallel File Systems

**Lustre:**
```
Architecture: Object Storage Servers (OSS) + Metadata Servers (MDS)
Performance: 1-10 TB/s aggregate bandwidth
Capacity: 10-100+ PB
Features: Striping, distributed metadata
```

**GPFS (IBM Spectrum Scale):**
```
Architecture: Shared-disk cluster file system
Performance: Similar to Lustre
Features: POSIX compliance, data replication
```

### 5.2 Burst Buffers

**Purpose:** Bridge gap between compute and storage
**Technology:** NVMe SSDs, persistent memory
**Capacity:** 1-10 PB
**Bandwidth:** 10-100 TB/s

**Use Cases:**
- Checkpoint/restart acceleration
- I/O staging for large jobs
- Temporary scratch space

### 5.3 Data Tiering

```
Hot Tier: Burst buffers (NVMe) - active job data
Warm Tier: PFS (Lustre/GPFS) - recent results
Cold Tier: Tape/object storage - archives
```

---



## 9. Resource Management

### 9.1 Job Schedulers

**Slurm:**
```bash
#!/bin/bash
#SBATCH --nodes=100
#SBATCH --ntasks-per-node=128
#SBATCH --time=24:00:00
#SBATCH --partition=compute

srun ./my_mpi_application
```

**PBS/Torque:**
```bash
#!/bin/bash
#PBS -l nodes=100:ppn=128
#PBS -l walltime=24:00:00

mpirun -np 12800 ./my_mpi_application
```

### 9.2 Resource Allocation

**Node Sharing:**
- Exclusive: One job per node
- Shared: Multiple jobs per node (with core/memory allocation)

**GPU Allocation:**
- Whole GPU: Dedicated to one job
- MIG (Multi-Instance GPU): Partition single GPU

### 9.3 Quality of Service (QoS)

**Priority Levels:**
1. High: Interactive, debugging (low resource)
2. Normal: Production jobs
3. Low: Backfill, low-priority

---




---

## A.1 Storage-protocol envelope

Storage-protocol envelopes cover: parallel-filesystem semantics
per POSIX + per the filesystem-specific extension (Lustre PFS
locking + striping envelope per OpenSFS Lustre 2.x; GPFS lock
manager + replication envelope per IBM Spectrum-Scale doc; BeeGFS
+ WekaIO + Daos as alternative high-performance filesystems);
metadata-server protocol (MDS / metanode / MDC + per-directory
hashing + lazy-quota envelope per the operator's metadata-policy);
data-mover protocol for HSM (POSIX-archive-mover per the
operator's HSM platform; S3-archive-mover for cloud-tiered
deployments); and the data-integrity protocol envelope (T10-PI per
SCSI; ZFS per-block checksum; Lustre per-stripe checksum; BTRFS
per-block checksum) tied to the operator's data-integrity policy.

## A.2 Resource-management protocol

Resource-management protocols cover: scheduler-policy envelope
(Slurm Multi-Factor Priority + Backfill per Slurm-2x doc; PBS
Pro queues + reservation envelope; LSF Multi-Cluster + Fairshare
per IBM Spectrum-LSF doc), admission-control envelope (per-user
+ per-project soft-cap + hard-cap quotas; per-partition QoS;
emergency-allocation reservation envelope), preemption-policy
envelope (priority-preemption with checkpoint-restart per the
DMTCP / SCR / FTI / VeloC envelope; lower-priority workloads
suspend on higher-priority arrival), and the licence-tracker
envelope (commercial-MPI / commercial-compiler / vendor-library
licences tracked per allocation per the operator's licence-server
envelope).

## A.3 Programming-model deployment protocol

Programming-model deployment protocols cover: MPI-runtime envelope
(OpenMPI per OFI libfabric; MVAPICH per the InfiniBand envelope;
Intel-MPI per Intel-software-stack; Cray-MPICH per CrayOS LE;
HPE Slingshot CXI envelope per the HPE Slingshot doc); thread-
parallel envelope (OpenMP 5.x + 6.x per OpenMP-spec; pthreads
per POSIX 1003.1; TBB per Intel-software-stack); accelerator-
programming envelope (CUDA per NVIDIA-CUDA spec; HIP per AMD-
ROCm; OneAPI / SYCL per Khronos SYCL 2020; OpenACC per OpenACC
spec; OpenMP target offload); and PGAS envelope (UPC + UPC++ per
LBNL Berkeley-UPC; Coarray Fortran per Fortran-2018; Chapel per
Cray Chapel; Charm++ per Illinois Charm++).

## A.4 Performance-monitoring protocol

Performance-monitoring protocols cover: per-job introspection
envelope (per-rank performance-counters per PAPI; communication-
trace envelope per Score-P / VampirTrace / TAU / HPCToolkit; I/O-
trace envelope per Darshan / Recorder); per-cluster monitoring
envelope (LDMS aggregator + per-node samplers; Prometheus +
Node-Exporter for the metric envelope; Grafana for visualisation;
ALERT-MANAGER for the threshold-crossing envelope); per-fabric
monitoring envelope (UFM for InfiniBand; Slingshot manager;
fabric-counter aggregation envelope); and the job-postmortem
envelope (XALT for library + version provenance per the run;
job-failure correlation envelope).

## A.5 Fault-tolerance protocol

Fault-tolerance protocols cover: hardware-fault detection envelope
(IPMI per ASF + DCMI per Intel; baseboard-management envelope
with per-rack LED + per-node ECC + per-link CRC counter); software-
fault detection envelope (per-node liveness probe; per-job
heartbeat; per-rank checkpointing via DMTCP / SCR / FTI / VeloC
with the operator's checkpoint-frequency envelope tuned to the
cluster's MTBF); recovery envelope (per-rank rollback-recovery for
soft-fault; per-job restart-from-checkpoint for hard-fault; per-
node-failure spare-pool draining envelope); and the resilience-
metric envelope (per-job MTTI per HPC-resilience tradition; per-
cluster availability per SLA).

## A.6 Energy-management protocol

Energy-management protocols cover: per-rack power envelope from
PDU + iPDU per ASHRAE TC 9.9 envelope; per-node frequency-scaling
envelope per Intel SST + AMD Cool'n'Quiet; per-accelerator power-
cap envelope per NVIDIA NVML + AMD ROCm-SMI; thermal-management
envelope (cooling-mode envelope: air per ASHRAE A1-A4; immersive-
liquid per the operator's cooling-loop manifest; rear-door heat-
exchanger envelope; warm-water cooling per the operator's water-
loop manifest); workload-aware scheduling envelope (energy-
efficient placement per the operator's GREEN500 envelope); and
the energy-budget envelope per the operator's cluster's PUE
envelope per ASHRAE TC 9.9.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/supercomputing/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-supercomputing-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/supercomputing-host:1.0.0` ships every supercomputing envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/supercomputing.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Supercomputing deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

## Z.6 Logging and observability hooks

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: ISO 8601 UTC timestamp per RFC 3339, host
identifier, tenant identifier, envelope class, envelope identifier,
operation outcome, and an opaque trace identifier propagated end-
to-end per W3C Trace Context (`traceparent` header) so a single
operation can be reconstructed across hosts. Phase 2 surfaces this
trace identifier as the `X-WIA-Trace-Id` response header. Phase 3
protocol exchanges propagate the trace identifier inside the
exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (e.g., Splunk, Elastic, Sumo
Logic, Wazuh, Microsoft Sentinel) per OpenTelemetry semantic
conventions, with `wia.standard.slug` and `wia.standard.phase` as
required attributes.

## Z.7 Versioning, deprecation, and capability discovery

Within the 1.x line, hosts MAY publish a capabilities document at
`/.well-known/wia-supercomputing-capabilities` that enumerates which
optional fields, optional endpoints, and optional protocol exchanges
the host implements. Clients MUST treat unsupported capabilities
as absent rather than as an error condition; a client that needs
a capability the host does not advertise MUST surface a clear
configuration error rather than silently degrade. Hosts moving
from one minor version to the next MUST publish the change in
the host's release notes with the per-capability migration window
per IETF RFC 8594 (Sunset header) + RFC 9745 (Deprecation header)
+ RFC 9651 (Structured Field Values) so machine consumers can
plan migration without waiting for human-channel notification.

## Z.8 Privacy and data-minimisation envelope

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California CPRA
per Cal. Civ. Code §1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Where the envelope ships across jurisdictional borders,
the operator's per-jurisdiction transfer envelope (SCC for EU, UK
IDTA, APEC CBPR, ASEAN MCC) MUST be referenced inside the audit
record. Subject-rights endpoints (access, rectification, erasure,
portability, restriction, objection) compose with WIA-OMNI-API per
its §5 subject-rights surface and need not be re-implemented
per-standard.

弘益人間 — Benefit All Humanity.
