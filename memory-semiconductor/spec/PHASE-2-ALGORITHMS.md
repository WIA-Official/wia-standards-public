# WIA-SEMI-002 PHASE 2: ALGORITHMS SPECIFICATION

**Version:** 1.0  
**Date:** 2025-01-26  
**Status:** Official Standard

## Overview

Phase 2 defines the algorithms and computational methods used in memory semiconductor systems, including bandwidth calculations, error correction codes, wear leveling, refresh optimization, and performance modeling.

## 1. Bandwidth and Performance Algorithms

### 1.1 Theoretical Bandwidth Calculation

#### 1.1.1 DDR Bandwidth Formula
```
Theoretical_BW = (Data_Rate_MT/s × Bus_Width_bits) / 8

Where:
- Data_Rate_MT/s = Mega-transfers per second
- Bus_Width_bits = Interface width in bits (typically 64 for DDR)
- Division by 8 converts bits to bytes

Example: DDR5-6400 with 64-bit bus
Theoretical_BW = (6400 × 64) / 8 = 51,200 MB/s = 51.2 GB/s
```

#### 1.1.2 Multi-Channel Bandwidth
```
Total_BW = Single_Channel_BW × Number_of_Channels

Example: Dual-channel DDR5-6400
Total_BW = 51.2 GB/s × 2 = 102.4 GB/s
```

#### 1.1.3 Effective Bandwidth
```
Effective_BW = Theoretical_BW × Efficiency_Factor

Where Efficiency_Factor accounts for:
- Protocol overhead
- Refresh overhead
- Command/address overhead
- Bank conflicts
- Page misses

Typical Efficiency_Factor: 0.75 - 0.90 (75-90%)
```

### 1.2 Latency Calculations

#### 1.2.1 DDR Access Latency
```
Total_Latency_cycles = tRCD + tCL

Total_Latency_ns = (Total_Latency_cycles × 1000) / Clock_Frequency_MHz

Example: DDR4-3200 with tRCD=14, tCL=14
Clock_Frequency = 3200 / 2 = 1600 MHz
Total_Latency_cycles = 14 + 14 = 28 cycles
Total_Latency_ns = (28 × 1000) / 1600 = 17.5 ns
```

#### 1.2.2 Page Hit vs Page Miss
```
Page_Hit_Latency = tCL
Page_Miss_Latency = tRP + tRCD + tCL

Example: DDR5-4800 (tRP=39, tRCD=39, tCL=40)
Page_Hit_Latency = 40 cycles = 16.7 ns
Page_Miss_Latency = 39 + 39 + 40 = 118 cycles = 49.2 ns
```

## 2. Error Correction Code (ECC) Algorithms

### 2.1 Hamming Code

#### 2.1.1 Syndrome Calculation
```
Parity_Bits_Required = ceil(log2(Data_Bits + Parity_Bits + 1))

For 64-bit data:
Parity_Bits = ceil(log2(64 + p + 1))
Solution: p = 7 bits

Total_Width = 64 + 7 = 71 bits
```

#### 2.1.2 Single Error Correction (SEC)
```python
def hamming_encode(data, n_bits=64):
    """
    Generate Hamming code parity bits
    """
    parity = 0
    for i in range(n_bits):
        if data & (1 << i):
            parity ^= (i + 1)
    return parity

def hamming_decode(codeword, n_bits=64):
    """
    Detect and correct single-bit error
    """
    syndrome = calculate_syndrome(codeword)
    if syndrome != 0:
        # Error at position 'syndrome'
        codeword ^= (1 << (syndrome - 1))
    return codeword
```

### 2.2 BCH Code (Bose-Chaudhuri-Hocquenghem)

#### 2.2.1 BCH Parameters
```
Code_Rate = Data_Bits / (Data_Bits + Parity_Bits)

Example: BCH(1023, 1013) correcting 1 bit
Data_Bits = 1013
Parity_Bits = 10
Code_Rate = 1013 / 1023 = 0.990 (99% efficiency)

For TLC NAND (80-bit correction per 1KB):
Parity_Overhead ≈ 14-15%
```

### 2.3 LDPC Code (Low-Density Parity-Check)

#### 2.3.1 LDPC Iterative Decoding
```python
def ldpc_decode(received_codeword, max_iterations=50):
    """
    Iterative belief propagation decoder
    """
    for iteration in range(max_iterations):
        # Variable node update
        var_messages = update_variable_nodes(received_codeword)
        
        # Check node update
        check_messages = update_check_nodes(var_messages)
        
        # Decision
        decoded = make_hard_decision(var_messages + check_messages)
        
        # Syndrome check
        if check_syndrome(decoded) == 0:
            return decoded  # Success
    
    return None  # Decoding failure
```

## 3. NAND Flash Management Algorithms

### 3.1 Wear Leveling

#### 3.1.1 Dynamic Wear Leveling
```python
def dynamic_wear_leveling(write_address):
    """
    Select physical block with lowest erase count
    """
    free_blocks = get_free_blocks()
    min_erase_count = min(block.erase_count for block in free_blocks)
    target_block = find_block_with_count(min_erase_count)
    
    map_logical_to_physical(write_address, target_block)
    return target_block
```

#### 3.1.2 Static Wear Leveling
```python
def static_wear_leveling(threshold=1000):
    """
    Move static data from low-erase-count blocks
    """
    blocks = get_all_blocks()
    avg_erase_count = sum(b.erase_count for b in blocks) / len(blocks)
    
    for block in blocks:
        if block.is_static and (avg_erase_count - block.erase_count) > threshold:
            # Move static data to high-erase-count block
            new_block = find_high_erase_count_block()
            copy_data(block, new_block)
            mark_as_free(block)
```

### 3.2 Garbage Collection

#### 3.2.1 Greedy Algorithm
```python
def greedy_garbage_collection():
    """
    Select block with most invalid pages
    """
    blocks = get_all_blocks()
    victim = max(blocks, key=lambda b: b.invalid_page_count)
    
    # Copy valid pages
    valid_pages = get_valid_pages(victim)
    for page in valid_pages:
        new_location = allocate_new_page()
        copy_page(page, new_location)
        update_mapping(page.logical_address, new_location)
    
    # Erase block
    erase_block(victim)
    return victim
```

#### 3.2.2 Cost-Benefit Algorithm
```python
def cost_benefit_gc():
    """
    Balance space reclamation with erase wear
    """
    blocks = get_all_blocks()
    
    def cost_benefit(block):
        age = current_time - block.last_modified
        utilization = block.valid_page_count / block.total_pages
        return (1 - utilization) * age / (1 + utilization)
    
    victim = max(blocks, key=cost_benefit)
    perform_garbage_collection(victim)
```

### 3.3 Write Amplification Calculation

```
Write_Amplification_Factor = Physical_Writes / Logical_Writes

Example:
User writes 100 GB of data
SSD internally writes 130 GB (including GC overhead)
WAF = 130 / 100 = 1.3

Lower WAF = Better endurance and performance
```

## 4. DRAM Refresh Algorithms

### 4.1 Auto-Refresh

```python
def auto_refresh(num_rows=8192, refresh_period_ms=64):
    """
    Distributed refresh across period
    """
    interval_us = (refresh_period_ms * 1000) / num_rows
    
    for row in range(num_rows):
        wait(interval_us)
        issue_refresh_command(row)
```

### 4.2 Targeted Row Refresh (TRR)

```python
def targeted_row_refresh(hammer_threshold=10000):
    """
    Detect and mitigate row hammer attacks
    """
    row_activation_count = {}
    
    for activation in memory_accesses:
        row = activation.row_address
        row_activation_count[row] = row_activation_count.get(row, 0) + 1
        
        if row_activation_count[row] > hammer_threshold:
            # Refresh adjacent rows
            refresh_row(row - 1)
            refresh_row(row + 1)
            row_activation_count[row] = 0
```

### 4.3 Refresh Penalty Calculation

```
Refresh_Overhead = (Refresh_Commands × tRFC) / Refresh_Period

Example: DDR4-3200
Refresh_Commands = 8,192
tRFC = 350 ns
Refresh_Period = 64 ms = 64,000,000 ns

Refresh_Overhead = (8192 × 350) / 64,000,000 = 4.48%
```

## 5. Memory Controller Scheduling Algorithms

### 5.1 First-Ready First-Come-First-Serve (FR-FCFS)

```python
def fr_fcfs_schedule(request_queue):
    """
    Prioritize row-hit requests, then oldest
    """
    ready_requests = []
    
    for req in request_queue:
        if can_issue_now(req):
            ready_requests.append(req)
    
    # Prioritize row hits
    row_hits = [r for r in ready_requests if is_row_hit(r)]
    if row_hits:
        return min(row_hits, key=lambda r: r.arrival_time)
    
    # Otherwise, oldest ready request
    return min(ready_requests, key=lambda r: r.arrival_time)
```

### 5.2 Parallelism-Aware Batch Scheduling (PAR-BS)

```python
def par_bs_schedule(request_queue, batch_size=16):
    """
    Batch requests to maximize parallelism
    """
    batch = []
    
    # Form batch
    for req in request_queue[:batch_size]:
        batch.append(req)
    
    # Prioritize by bank/rank to maximize parallelism
    batch.sort(key=lambda r: (r.bank_id, r.rank_id, r.arrival_time))
    
    return batch
```

## 6. HBM Channel Assignment

### 6.1 Round-Robin Interleaving

```python
def assign_hbm_channel(address, num_channels=8):
    """
    Distribute addresses across HBM channels
    """
    channel_bits = log2(num_channels)
    channel_id = (address >> 6) & ((1 << channel_bits) - 1)
    return channel_id
```

### 6.2 XOR-based Interleaving

```python
def xor_channel_assignment(address, num_channels=8):
    """
    XOR high and low bits for better distribution
    """
    low_bits = (address >> 6) & 0x7
    high_bits = (address >> 12) & 0x7
    channel_id = (low_bits ^ high_bits) % num_channels
    return channel_id
```

## 7. Power Calculation Algorithms

### 7.1 DRAM Power Model

```
P_active = (I_DD0 × V_DD × Activity_Factor) + (N_banks × I_DDN × V_DD)
P_standby = I_DD2N × V_DD
P_refresh = (I_DD5 × V_DD × Refresh_Rate)

Total_Power = P_active + P_standby + P_refresh

Where:
- I_DD0 = Operating current per bank
- I_DDN = Activate/precharge current
- I_DD2N = Standby current
- I_DD5 = Refresh current
- V_DD = Operating voltage
```

### 7.2 Energy per Bit

```
Energy_per_Bit = Total_Power / Bandwidth

Example: DDR5-4800 consuming 5W with 38.4 GB/s bandwidth
Energy_per_Bit = 5W / (38.4 × 8 Gbps) = 16.3 pJ/bit
```

## 8. Performance Modeling

### 8.1 Queuing Theory Model

```
Average_Latency = Service_Time / (1 - Utilization)

Where:
- Service_Time = Average time to service one request
- Utilization = Arrival_Rate × Service_Time

Example:
Service_Time = 50 ns
Arrival_Rate = 10 million requests/second
Utilization = (10^7 × 50 × 10^-9) = 0.5

Average_Latency = 50 / (1 - 0.5) = 100 ns
```

### 8.2 Memory Controller Scheduling

WIA-SEMI-002 RECOMMENDS the FR-FCFS (First-Ready, First-Come-First-Served) row-buffer scheduler for general-purpose workloads, with optional bank-aware extensions for QoS-sensitive deployments.

```
priority = (row_hit_bonus, age, criticality_class)

while queue not empty:
    eligible = {req | bus_free(req.bank) and timing_satisfied(req)}
    pick = argmax(eligible, key=priority)
    issue(pick)
```

Schedulers MUST honour the JEDEC tFAW (four-activate window) and tRRD limits to prevent supply-current spikes. Implementations SHOULD expose runtime knobs for the row-hit bonus so tenants can re-tune for their workload.

### 8.3 NAND Flash Translation Layer (FTL) Algorithms

| Algorithm | Mapping granularity | Pros | Trade-off |
|-----------|---------------------|------|-----------|
| Page-level FTL | 4 KiB / 16 KiB | Lowest write amplification | Largest map RAM footprint |
| Block-level FTL | 256 KiB / 4 MiB | Tiny map | Highest WAF |
| Hybrid FTL (DFTL) | Variable | Map cached on demand | More complex GC |

```
GC_score(block) = invalid_pages(block) / total_pages(block)
                  - α × age(block)
                  - β × p/e_count(block) / max_pe
```

Garbage collection MUST be wear-aware: blocks above 90% of their P/E lifetime budget MUST be excluded from victim selection unless the free-block reservoir is below the SLC mirror threshold.

### 8.4 Worst-Case Latency Bounds

For real-time and safety-critical deployments the controller MUST provide bounded latency guarantees:

```
L_worst = L_queue_max + L_arb + L_bus + L_device_max
```

Where `L_queue_max` is the bounded queue length × per-request service time, `L_arb` is the arbitration window, and `L_device_max` is the device-published worst-case service time (e.g. tRC for activate-precharge cycles). `L_worst` MUST be measurable on the deployed hardware via the telemetry stream defined in Phase 1 §11.

### 8.5 RAS Decoder Selection

The repair algorithm MUST be selected per technology node:

| Storage class | Default code | Replacement threshold |
|---------------|--------------|-----------------------|
| DDR5 on-die | SEC-DED Hamming(72,64) | UE rate > 1 FIT/Mbit |
| HBM3 PHY | RS(32,28) over 32 lanes | Lane retraining > 1/min |
| TLC NAND | LDPC, 80–100 bits/KB | RBER > 1×10⁻³ |
| QLC NAND | LDPC, 200+ bits/KB | RBER > 5×10⁻³ |
| ReRAM (emerging) | Concatenated BCH+LDPC | TBD per generation |

Decoder logs MUST emit telemetry frame type 0x05 (Phase 1) every 1 second so a host RAS daemon can integrate via standard Linux EDAC counters.

---

**Document Control:**
- **Author:** WIA Standards Committee
- **Approved By:** WIA Technical Board
- **Next Review:** 2026-01-26
- **Classification:** Public Standard

© 2025 World Certification Industry Association (WIA)
弘益人間 · Benefit All Humanity
