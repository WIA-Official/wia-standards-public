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

## 10. Energy Efficiency

### 10.1 Power Metrics

**PUE (Power Usage Effectiveness):**
```
PUE = Total Facility Power / IT Equipment Power

Target: PUE < 1.2 (highly efficient)
Typical: PUE = 1.5-2.0
```

**FLOPS per Watt:**
```
Energy Efficiency = Peak FLOPS / Total System Power

Target: >50 GFLOPS/Watt
Current leaders: >60 GFLOPS/Watt
```

### 10.2 Power Management

**Dynamic Voltage/Frequency Scaling (DVFS):**
- Reduce frequency during I/O-bound phases
- Boost frequency for compute-intensive phases

**Node Power Capping:**
- Limit per-node power consumption
- Prevent oversubscription of facility power

---

## 11. Fault Tolerance

### 11.1 Checkpoint/Restart

**Coordinated Checkpointing:**
```
All processes checkpoint simultaneously
Simple but requires global synchronization
```

**Uncoordinated Checkpointing:**
```
Processes checkpoint independently
More complex recovery but lower overhead
```

### 11.2 Resilience Techniques

**Algorithm-Based Fault Tolerance (ABFT):**
- Checksums in numerical algorithms
- Detect and correct errors without restart

**Redundant Computation:**
- Run critical sections redundantly
- Compare results to detect errors

### 11.3 Mean Time Between Failures (MTBF)

```
System MTBF = Component MTBF / Number of Components

Example:
Node MTBF: 5 years
System with 10,000 nodes: MTBF = 5 × 365 / 10,000 ≈ 4.4 hours

Conclusion: Checkpointing every 2-3 hours recommended
```

---

## 12. Implementation Guidelines

### 12.1 System Deployment

**Phase 1: Acceptance Testing**
- LINPACK benchmark (Rmax/Rpeak)
- Network latency/bandwidth tests
- Storage I/O performance

**Phase 2: User Onboarding**
- Compile and optimize codes
- Benchmark application performance
- Identify bottlenecks

**Phase 3: Production Operations**
- Monitor utilization (>80% target)
- Maintain uptime (>95% target)
- Regular maintenance windows

### 12.2 Software Stack

**Operating System:**
- Linux: RHEL, CentOS, SLES, Ubuntu
- Lightweight kernels: CNL, Catamount

**Compilers:**
- GNU (gcc, g++, gfortran)
- Intel (icc, icpc, ifort)
- NVIDIA (nvcc)
- AMD (aocc)

**Libraries:**
- MPI: OpenMPI, MPICH, Intel MPI
- Math: MKL, BLAS, LAPACK, FFTW
- I/O: HDF5, NetCDF, ADIOS

### 12.3 Optimization Best Practices

**CPU Optimization:**
1. Vectorization: Use SIMD instructions
2. Cache locality: Optimize data layout
3. Threading: Balance load across cores

**GPU Optimization:**
1. Occupancy: Maximize active warps
2. Memory coalescing: Aligned access patterns
3. Kernel fusion: Reduce kernel launch overhead

---

## 13. References

### Standards Bodies
- TOP500: Global supercomputer ranking
- Green500: Energy-efficient HPC systems
- HPCG: Realistic HPC benchmark
- MPI Forum: MPI standard

### Research Papers
1. "Exascale Computing: A New Era in HPC" (IEEE, 2024)
2. "Power-Aware Supercomputing" (ACM, 2024)
3. "Fault Tolerance at Exascale" (SIAM, 2025)

### WIA Standards
- WIA-INTENT: Intent-based job submission
- WIA-OMNI-API: Universal HPC API
- WIA-QUANTUM: Quantum-HPC integration

---

**弘익人間 (Benefit All Humanity)**

*This specification is maintained by the WIA Computing Research Group and is continuously updated to reflect the latest advancements in supercomputing technology.*

*© 2025 SmileStory Inc. / WIA - MIT License*
