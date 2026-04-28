# WIA-COMP-001 — Phase 2: API Interface

> Supercomputing canonical Phase 2: API surface (clusters + jobs + benchmarks + allocations + telemetry + audit).

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


## 6. Programming Models

### 6.1 Distributed Memory (MPI)

**MPI Standard Compliance:**
- MPI-3.1 or later
- Point-to-point: Send, Recv, Isend, Irecv
- Collectives: Bcast, Reduce, Allreduce, Alltoall
- One-sided: Put, Get, Accumulate (RMA)

**Example:**
```c
#include <mpi.h>

int main(int argc, char** argv) {
    MPI_Init(&argc, &argv);

    int rank, size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);

    // Communication code
    double data;
    MPI_Bcast(&data, 1, MPI_DOUBLE, 0, MPI_COMM_WORLD);

    MPI_Finalize();
    return 0;
}
```

### 6.2 Shared Memory (OpenMP)

**OpenMP 5.0+ Features:**
- Parallel loops, sections, tasks
- SIMD directives
- Target offloading (GPU)
- Memory management

**Example:**
```c
#include <omp.h>

void parallel_sum(double* array, int n) {
    double sum = 0.0;

    #pragma omp parallel for reduction(+:sum)
    for (int i = 0; i < n; i++) {
        sum += array[i];
    }

    printf("Sum: %f\n", sum);
}
```

### 6.3 GPU Programming (CUDA/OpenCL)

**CUDA:**
```cuda
__global__ void vectorAdd(float* a, float* b, float* c, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        c[i] = a[i] + b[i];
    }
}

// Launch kernel
vectorAdd<<<gridSize, blockSize>>>(d_a, d_b, d_c, n);
```

### 6.4 Hybrid Models

**MPI + OpenMP:**
```c
MPI_Init_thread(&argc, &argv, MPI_THREAD_FUNNELED, &provided);

#pragma omp parallel
{
    // Thread-level parallelism within each MPI rank
}
```

**MPI + CUDA:**
```c
// Each MPI rank controls multiple GPUs
cudaSetDevice(rank % num_gpus_per_node);
```

---



## 7. Performance Metrics

### 7.1 FLOPS Calculation

**Theoretical Peak:**
```
Peak FLOPS = Cores × Clock Speed × FLOP/Cycle × Nodes

Example (1000-node cluster):
Cores per node: 128
Clock: 2.5 GHz
FLOP/Cycle: 32 (AVX-512 double-precision)
Nodes: 1000

Peak = 128 × 2.5e9 × 32 × 1000 = 10.24 PetaFLOPS
```

**Sustained Performance:**
```
Typically 50-80% of peak for well-optimized codes
Sustained = Peak × Efficiency
```

### 7.2 Scalability Metrics

**Strong Scaling:**
```
Fixed problem size, increasing processors
Ideal: T(1) / T(N) = N
Efficiency: E(N) = T(1) / (N × T(N))
```

**Weak Scaling:**
```
Problem size increases with processors
Ideal: T(N) = T(1) for all N
Efficiency: E(N) = T(1) / T(N)
```

### 7.3 Communication Overhead

**Communication Time:**
```
T_comm = Latency + (Message_Size / Bandwidth)

For small messages: Latency-dominated
For large messages: Bandwidth-dominated
```

---



## 8. Benchmarking

### 8.1 LINPACK (HPL)

**Purpose:** Measure peak FLOPS for TOP500 ranking
**Algorithm:** Solve dense linear system Ax = b
**Metrics:**
- Rmax: Maximum achieved FLOPS
- Rpeak: Theoretical peak FLOPS
- Efficiency: Rmax / Rpeak

### 8.2 HPCG (High-Performance Conjugate Gradient)

**Purpose:** Measure performance on sparse linear systems
**More realistic** than LINPACK for many applications
**Metrics:** GFLOPS achieved

### 8.3 Application-Specific Benchmarks

- **Climate:** CESM, WRF
- **Molecular Dynamics:** LAMMPS, GROMACS
- **CFD:** OpenFOAM, FUN3D
- **AI/ML:** MLPerf Training, ResNet-50

---




---

## A.1 Endpoint reference

```http
POST /supercomputing/v1/clusters                   # register cluster
GET  /supercomputing/v1/clusters/{id}              # fetch cluster record
POST /supercomputing/v1/jobs                       # submit job descriptor
GET  /supercomputing/v1/jobs/{id}                  # fetch job state
POST /supercomputing/v1/benchmarks                 # submit benchmark run
GET  /supercomputing/v1/benchmarks/{id}            # fetch benchmark
POST /supercomputing/v1/allocations                # request allocation
GET  /supercomputing/v1/allocations/{id}           # allocation state
WS   /supercomputing/v1/state/stream               # cluster state stream
GET  /supercomputing/v1/audit/{id}                 # audit trail
```

Every endpoint follows the discovery convention at
`/.well-known/wia-supercomputing`. Allocation-management endpoints
require the requester's project credential plus the per-allocation-
authority signature.

## A.2 Cluster-registration API

`POST /clusters` accepts the Phase 1 §A.1 envelope. The endpoint
validates the architectural-class envelope against the operator's
fabric-and-rack manifest, materialises the per-node Phase 1 §A.2
records into the operator's CMDB, and returns the cluster's
registration URI. Cluster-state transitions through `provisioning`,
`available`, `partial-degraded`, `service-mode`, `retired`; state
transitions emit audit events that survive cluster decommissioning.

## A.3 Job-submission API

`POST /jobs` accepts the Phase 1 §A.4 envelope. The endpoint
maps the requested-resource envelope into the operator's scheduler
queue (Slurm partition + QoS + reservation per the operator's
allocation-policy), enforces project-allocation accounting, and
returns the job descriptor with the queued + priority + estimated-
start envelope. Job-state retrieval returns the rolling event
timeline plus the per-step performance counters where the operator
has enabled per-job performance-introspection per LDMS / Caliper /
TAU instrumentation.

## A.4 Benchmark-submission API

`POST /benchmarks` accepts a benchmark-run envelope (HPL per
TOP500; HPCG per Heroux + Dongarra + Luszczek; HPL-MxP per ECP;
HPCG-MxP; STREAM
per McCalpin; OSU MPI; IOR; mdtest; SPECfp; LINPACK + dgemm; LCB
per LCA; Industry-Standard MLPerf-HPC per MLCommons). The endpoint
verifies the run envelope against the benchmark's published
configuration rules (e.g., HPL-AI vs HPL TOP500 allowed
modifications) and emits a benchmark-result event with the full
provenance envelope (cluster identifier; node + interconnect +
software-stack snapshots; per-rank performance-counter histogram).

## A.5 Allocation-management API

`POST /allocations` accepts an allocation-request envelope: project
identifier, principal-investigator credential, requested core-hours
+ accelerator-hours + storage-allocation envelope, requested-period
envelope, scientific justification envelope, and the
prerequisite-IRB / export-control envelope where the workload
involves regulated data. The endpoint routes the request through
the operator's allocation-committee workflow and emits state events
on review-board action; approved allocations are deposited into the
operator's accounting-database with the per-month soft-cap +
hard-cap envelope per the operator's overcommit policy.

## A.6 Telemetry WebSocket

The state-stream WebSocket multiplexes per-cluster events: per-rack
power + temperature + cooling envelope from PDU + CRAC telemetry;
per-node liveness + accelerator + memory ECC envelope; per-fabric
link-up / link-down + congestion-event envelope; per-storage-tier
fill + bandwidth + IOPS envelope; per-scheduler queue-depth +
job-throughput + waiting-time envelope. Subscribers can filter by
cluster-id, node-class, fabric-segment, and scheduler-queue. Rate
limits: 5000 req/h authenticated; 50000 req/h trusted-partner.
WebSocket subscriptions are bounded at 100 simultaneous per
credential.

## A.7 Audit endpoint envelope

`GET /audit/{id}` returns the immutable audit trail: cluster
registration, every job-submission event, every job-state-
transition event, every benchmark-run event, every allocation-
review event, every operator credential change, every export-
control deviation event, and every retirement event. The audit-
trail integrity is anchored into a Merkle tree per-cluster and
the root is committed to the operator's archival record per
the funding-agency retention policy (DOE NNSA: 30 years; DOE SC:
20 years; NSF: 10 years; RIKEN R-CCS: per JAS regulation;
EuroHPC JU: per Regulation 2021/1173).


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
