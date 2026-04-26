# WIA-COMP-005: Operating System Specification v1.0

> **Standard ID:** WIA-COMP-005
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Computing Research Group

---

## 1. Introduction

### 1.1 Purpose
This specification defines standards for modern operating systems including kernel architecture, scheduling, memory management, and security.

### 1.2 Philosophy
**弘익人間 (Benefit All Humanity)** - Provide secure, efficient operating systems for all users.

## 2. Kernel Architectures

### 2.1 Monolithic Kernel
**Examples:** Linux, Unix, BSD
**Characteristics:** All services in kernel space
**Advantages:** Performance, simplicity
**Disadvantages:** Less modular, larger attack surface

### 2.2 Microkernel
**Examples:** Minix, L4, seL4
**Characteristics:** Minimal kernel, services in user space
**Advantages:** Security, fault isolation, modularity
**Disadvantages:** Performance overhead (IPC)

### 2.3 Hybrid Kernel
**Examples:** Windows NT, macOS (XNU)
**Characteristics:** Combination approach
**Balance:** Performance + modularity

## 3. Process Scheduling

### 3.1 Completely Fair Scheduler (CFS)
**Used by:** Linux
**Algorithm:** Red-black tree of tasks
**Fairness:** Proportional CPU time based on nice values

### 3.2 O(1) Scheduler
**Complexity:** Constant time scheduling
**Structure:** 140 priority queues
**Time Slices:** Priority-based quantum

### 3.3 Real-Time Scheduling
**Policies:** SCHED_FIFO, SCHED_RR, SCHED_DEADLINE
**Guarantee:** Bounded latency
**Priority:** Higher than normal tasks

## 4. Memory Management

### 4.1 Virtual Memory
**Concept:** Each process has own address space
**Size:** 48-bit (256 TB) or 64-bit address space
**Benefits:** Isolation, protection, sharing

### 4.2 Paging
**Page Size:** 4 KB (standard), 2 MB (huge), 1 GB (giant)
**Page Table:** Multi-level hierarchy (4-5 levels)
**TLB:** Translation Lookaside Buffer cache

### 4.3 NUMA (Non-Uniform Memory Access)
**Architecture:** Multiple memory nodes
**Policy:** Local allocation preferred
**Performance:** Avoid remote memory access

## 5. File Systems

### 5.1 ext4 (Extended File System)
**Features:** Journaling, extents, delayed allocation
**Max File Size:** 16 TB
**Max Volume Size:** 1 EB

### 5.2 Btrfs (B-tree File System)
**Features:** Copy-on-write, snapshots, compression
**Reliability:** Checksums, self-healing
**Scalability:** 16 EB

### 5.3 ZFS
**Features:** Pooled storage, RAID-Z, compression
**Integrity:** End-to-end checksums
**Snapshots:** Instant, space-efficient

## 6. Security

### 6.1 Access Control
**DAC:** Discretionary (file permissions)
**MAC:** Mandatory (SELinux, AppArmor)
**Capabilities:** Fine-grained privileges

### 6.2 SELinux (Security-Enhanced Linux)
**Model:** Type enforcement, RBAC, MLS
**Policies:** Targeted, strict, MLS
**Labels:** user:role:type:level

### 6.3 Sandboxing
**Mechanisms:** Containers, VMs, seccomp
**Isolation:** Process, network, filesystem
**Examples:** Docker, LXC, Firejail

## 7. Inter-Process Communication

### 7.1 Pipes
**Types:** Anonymous, named (FIFO)
**Communication:** Unidirectional byte stream
**Usage:** Shell pipelines

### 7.2 Shared Memory
**Mechanism:** mmap(), System V shmget()
**Speed:** Fastest IPC (no copying)
**Synchronization:** Required (mutexes, semaphores)

### 7.3 Signals
**Purpose:** Asynchronous notifications
**Common:** SIGTERM, SIGKILL, SIGSEGV
**Handlers:** User-defined signal handlers

## 8. Device Drivers

### 8.1 Character Devices
**Examples:** Keyboard, mouse, serial port
**Interface:** Byte stream
**Operations:** read(), write(), ioctl()

### 8.2 Block Devices
**Examples:** Hard disk, SSD, USB drive
**Interface:** Fixed-size blocks
**Buffering:** Page cache, I/O scheduler

### 8.3 Network Devices
**Examples:** Ethernet, WiFi, Bluetooth
**Stack:** Link → Network → Transport → Application
**Performance:** Interrupt coalescing, NAPI

## 9. Containerization

### 9.1 Namespaces
**Types:** PID, Network, Mount, UTS, IPC, User, Cgroup
**Purpose:** Resource isolation
**Usage:** Docker, LXC

### 9.2 Cgroups (Control Groups)
**Resources:** CPU, memory, I/O, network
**Limits:** Hard caps, soft limits
**Accounting:** Usage tracking

## 10. Use Cases

1. **Server Infrastructure:** Linux (Ubuntu, RHEL, CentOS)
2. **Desktop:** Windows, macOS, Linux distributions
3. **Mobile:** Android, iOS
4. **Embedded:** FreeRTOS, Zephyr, VxWorks
5. **HPC:** Lightweight kernels (CNL, mOS)
6. **Cloud:** Container orchestration (Kubernetes)
7. **IoT:** Real-time OS (RTOS)
8. **Research:** Teaching OS (Minix, xv6)

---

**弘익人間 (Benefit All Humanity)**

*© 2025 SmileStory Inc. / WIA - MIT License*
