# WIA-COMP-001 — Phase 1: Data Format

> Supercomputing canonical Phase 1: cluster + node + compute-resource + workload-record + storage + interconnect envelopes.

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


## 1. Introduction

### 1.1 Purpose

This specification defines standards for high-performance computing (HPC) systems capable of exascale performance, including hardware architecture, software stack, programming models, and operational best practices.

### 1.2 Scope

The standard covers:
- Compute node architecture (CPU, GPU, accelerators)
- Interconnect networks and topologies
- Parallel file systems and storage
- Programming models (MPI, OpenMP, CUDA)
- Performance benchmarking methodologies
- Resource management and job scheduling

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - Supercomputing technology enables scientific breakthroughs that benefit society: climate prediction, disease cures, clean energy, and AI advancement.

### 1.4 Terminology

- **FLOPS**: Floating-Point Operations Per Second
- **Exascale**: 10^18 FLOPS performance level
- **HPC**: High-Performance Computing
- **MPI**: Message Passing Interface
- **RDMA**: Remote Direct Memory Access
- **NUMA**: Non-Uniform Memory Access
- **PFS**: Parallel File System

---



## 2. System Architecture

### 2.1 Hierarchical Design

```
Supercomputer Hierarchy:
┌─────────────────────────────────────────┐
│  System (1 exaFLOPS+)                   │
│  ├── Racks (1000+)                      │
│  │   ├── Chassis (40+ per rack)        │
│  │   │   ├── Nodes (2-4 per chassis)   │
│  │   │   │   ├── CPUs (2-4 per node)   │
│  │   │   │   ├── GPUs (4-8 per node)   │
│  │   │   │   ├── Memory (256GB - 2TB)  │
│  │   │   │   └── Local Storage (NVMe)  │
│  │   │   └── Network (InfiniBand/RoCE)│
│  │   └── Power & Cooling                │
│  └── Shared Storage (100+ PB)          │
└─────────────────────────────────────────┘
```

### 2.2 Compute Node Types

**CPU-Only Nodes:**
- High core count (64-256 cores)
- Large memory capacity (512 GB - 2 TB)
- Use cases: Memory-intensive, irregular workloads

**GPU-Accelerated Nodes:**
- Multi-GPU (4-8 GPUs per node)
- High compute density (>100 TFLOPS/node)
- Use cases: AI/ML, molecular dynamics, CFD

**FPGA Nodes:**
- Reconfigurable logic
- Custom accelerators for specific algorithms
- Use cases: Genomics, cryptography, signal processing

### 2.3 System Topology

**Fat Tree:**
```
Advantages: Non-blocking, full bisection bandwidth
Layers: Core switches → Aggregation → Edge → Nodes
Scalability: Up to 100,000+ nodes
```

**Dragonfly:**
```
Advantages: Lower cost, reduced diameter
Structure: Groups interconnected by all-to-all links
Scalability: Cost-effective for large systems
```

---



## 3. Compute Resources

### 3.1 CPU Architecture

**Requirements:**
- Architecture: x86-64, ARM, RISC-V, or custom
- Cores: 64-256 per socket
- Clock Speed: 2-4 GHz
- Vector Units: AVX-512, SVE, or equivalent
- Cache: L1 (32 KB/core), L2 (512 KB/core), L3 (shared)

**Performance Targets:**
```
Peak FLOPS per socket:
Single-precision: 4-10 TFLOPS
Double-precision: 2-5 TFLOPS
Mixed-precision: 8-20 TFLOPS (with AI extensions)
```

### 3.2 GPU Architecture

**Requirements:**
- Compute Units: 80-144 SMs/CUs
- Memory: 40-80 GB HBM2/HBM3
- Memory Bandwidth: 1-3 TB/s
- Interconnect: NVLink, Infinity Fabric, or custom

**Performance Targets:**
```
Peak FLOPS per GPU:
FP64 (double): 10-20 TFLOPS
FP32 (single): 20-40 TFLOPS
FP16 (half): 80-160 TFLOPS
INT8 (AI): 160-320 TOPS
```

### 3.3 Memory Hierarchy

| Level | Type | Capacity | Bandwidth | Latency |
|-------|------|----------|-----------|---------|
| L1 Cache | SRAM | 32-64 KB/core | 1-2 TB/s | 3-4 cycles |
| L2 Cache | SRAM | 512 KB-1 MB/core | 500-1000 GB/s | 12-15 cycles |
| L3 Cache | SRAM | 64-256 MB shared | 200-500 GB/s | 40-50 cycles |
| Main Memory | DDR5/HBM | 256 GB - 2 TB | 500-3000 GB/s | 80-100 ns |
| NVMe | Flash | 1-10 TB | 10-50 GB/s | 10-100 μs |
| Network Storage | PFS | 100+ PB | 1-10 TB/s aggregate | 1-10 ms |

---




---

## A.1 Cluster-record envelope

The Phase 1 envelope groups HPC clusters by architectural class
(homogeneous-CPU clusters per the LINPACK / HPCG benchmark
characterisation; heterogeneous CPU+GPU clusters per the GREEN500
+ TOP500 GPU sub-list; vector-processor clusters per NEC SX-Aurora
+ Fujitsu A64FX SVE; many-integrated-core clusters per Intel Xeon
Phi historical baseline; emerging quantum-classical hybrid
platforms per the operator's quantum-computing-roadmap envelope)
with the canonical fields: cluster identifier, deployment site,
operator + funding-agency envelope, peak compute (Rpeak in PFLOPS
double-precision per LINPACK + Rmax actual achieved per HPL run),
node-count + core-count + accelerator-count, memory-capacity
envelope, storage-capacity envelope, interconnect-fabric envelope,
power-envelope (peak + average + effectiveness ratio per the
PUE per ASHRAE TC 9.9), and the audit envelope tied to the site
operator (DOE NNSA, DOE SC, NSF, RIKEN, RIKEN R-CCS, JAMSTEC,
EuroHPC JU, NCSA, EPCC, BSC, GRNET, CSCS, ECP, CINECA).

## A.2 Node-record envelope

A node-record envelope MUST list: node identifier (UUID v7 per RFC
9562 with deterministic site-prefixed encoding), CPU descriptor
(vendor + model + microarchitecture per CPU-Z / lscpu mapping;
core-count + SMT-thread-count; clock-speed envelope including
turbo + sustained-all-core; cache-hierarchy envelope L1+L2+L3
with associativity per the operator's hardware-platform document;
ISA-feature envelope including AVX-512 / SVE / AMX support flags),
accelerator descriptor (NVIDIA SMX board with SM-count + Tensor-
Core-count + memory + NVLink-bandwidth; AMD CDNA accelerator with
matrix-engine envelope; Intel Xe-HPC accelerator; Cerebras WSE +
Graphcore IPU + SambaNova RDU + Tenstorrent for emerging
architectures), memory descriptor (DRAM channel-count + per-
channel bandwidth per JEDEC + capacity per DIMM; HBM stack-count
+ pseudo-channel envelope per JEDEC HBM3/HBM3E; persistent-memory
envelope per SNIA NVM; CXL-attached memory envelope per CXL 3.0
Type-3 device specification), local-storage descriptor (NVMe SSD
+ SAS HDD + tape-class device envelope), and the network-interface
descriptor (HCA / NIC / DPU envelope per OFA OFED).

## A.3 Compute-resource envelope

Compute-resource descriptors carry: peak FLOPs envelope (FP64 +
FP32 + FP16 + BF16 + FP8 + INT8 across the node-class with the
sustained-utilisation envelope per benchmarked workload), memory-
bandwidth envelope (peak per JEDEC theoretical + measured per
STREAM Triad + measured per HPCG matrix-vector kernel), inter-
node bandwidth + latency envelope (peak per the interconnect
specification + measured per IMB MPI-1 / MPI-2 / Pingpong + per
the OSU MPI Microbenchmarks), local-storage bandwidth envelope
(peak per NVMe spec + measured per FIO + measured per IOR), and
the system-software envelope (kernel + scheduler + MPI library
+ math library + compiler envelope per the operator's software-
stack version-of-record).

## A.4 Workload-record envelope

Workload-record envelopes follow the operator's job-record format
(typically Slurm JobInfo per Slurm-2x sched_packjob; LSF, PBS Pro,
Torque + Moab, IBM Platform LSF, OpenPBS, Altair PBS Pro, Adaptive
Computing Maui-Moab as historical baseline): job identifier, user
+ project envelope, requested-resource envelope (node-count +
core-per-node + memory-per-core + accelerator-count + walltime),
allocated-resource envelope (actual nodes + actual cores + actual
memory + actual walltime), event-timeline envelope (submit + queue
+ start + end + checkpoint events), exit-state envelope (success
/ user-cancel / preemption / wallclock-limit / out-of-memory /
node-failure / unclassified-failure), the per-job audit envelope
linking back to the operator's accounting-database, and the
attribution envelope for archival reproducibility.

## A.5 Storage-record envelope

Storage-record envelopes describe per-tier storage with the
performance + capacity + persistence envelope: scratch-tier
(typical lifetime hours-to-days; per-job purged; capacity O(PB);
bandwidth O(TB/s) per Lustre OSS aggregate; protocol POSIX +
Lustre per OpenSFS Lustre 2.x); project-tier (typical lifetime
months-to-years; per-project; capacity O(10s PB); replicated +
erasure-coded; protocol POSIX + GPFS / Spectrum-Scale / BeeGFS /
WekaIO); archive-tier (typical lifetime decade+; tape-backed per
LTO-9/10 per ECMA-319 + IBM TS1170 enterprise-tape envelope;
HSM-managed per IEEE 1244 reference); and the per-tier metadata-
service envelope (Lustre MDS, GPFS metanode, Ceph MDS, namespace-
distribution envelope per CephFS).

## A.6 Interconnect-record envelope

Interconnect-record envelopes describe the network fabric: topology
(fat-tree per Leiserson 1985; dragonfly per Kim 2008; multi-rail
fat-tree; 6D mesh-torus per Cray Aries / Slingshot; full-bisection
HDR / NDR / XDR InfiniBand per OFA + IBTA spec; HPE Slingshot 11/
NDR ROCE per Cray Slingshot 11; AWS EFA per Elastic Fabric Adapter
spec; Tofu D per Fujitsu A64FX), per-link bandwidth + latency
envelope (200 Gb/s HDR; 400 Gb/s NDR; 800 Gb/s XDR per IBTA; 200
Gb/s Slingshot 11; per-stage hop-latency budget envelope), routing
envelope (OpenSM per OFA for InfiniBand; UFM-based managed routing;
adaptive-routing per Slingshot + OFI libfabric; congestion-control
envelope per RoCEv2 PFC + ECN + DCQCN / HPCC / Timely as
appropriate to the fabric), and the operator's per-fabric runbook
+ per-month performance-trend envelope.


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
