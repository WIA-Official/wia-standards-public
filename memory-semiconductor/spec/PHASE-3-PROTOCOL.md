# WIA-SEMI-002 PHASE 3: PROTOCOL SPECIFICATION

**Version:** 1.0  
**Date:** 2025-01-26  
**Status:** Official Standard

## Overview

Phase 3 defines the communication protocols and interfaces for memory semiconductor devices, including DDR command protocols, NAND Flash interfaces (ONFI), HBM protocols, and emerging standards like CXL memory.

## 1. DDR SDRAM Protocol

### 1.1 Command Encoding

#### 1.1.1 Command Truth Table

| Command | /CS | /RAS | /CAS | /WE | Function |
|---------|-----|------|------|-----|----------|
| ACTIVATE | 0 | 0 | 1 | 1 | Open row for access |
| READ | 0 | 1 | 0 | 1 | Initiate read operation |
| WRITE | 0 | 1 | 0 | 0 | Initiate write operation |
| PRECHARGE | 0 | 0 | 1 | 0 | Close open row |
| REFRESH | 0 | 0 | 0 | 1 | Refresh cells |
| MRS | 0 | 0 | 0 | 0 | Mode Register Set |
| NOP | 0 | 1 | 1 | 1 | No Operation |
| DESELECT | 1 | X | X | X | Chip not selected |

### 1.2 Command Sequences

#### 1.2.1 Read Command Sequence
```
1. ACTIVATE (Bank, Row)
   - Opens specified row in bank
   - Must wait tRCD before column access

2. READ (Bank, Column, Auto-Precharge)
   - Issues read command to column
   - Data appears after tCL cycles
   - Auto-precharge closes row when done

3. Data Transfer
   - Burst length of 8 (DDR4) or 16 (DDR5)
   - Data on both clock edges

4. PRECHARGE (Optional if auto-precharge not used)
   - Closes row, must wait tRP before next activate
```

#### 1.2.2 Write Command Sequence
```
1. ACTIVATE (Bank, Row)
   - Wait tRCD

2. WRITE (Bank, Column, Auto-Precharge)
   - Data input after tCWL (CAS Write Latency)
   - Burst write begins

3. Write Recovery
   - Must wait tWR before precharge

4. PRECHARGE
   - Wait tRP before next activate
```

### 1.3 Timing Constraints

#### 1.3.1 DDR4 Protocol Timing (DDR4-3200)

```
Clock Period:             0.625 ns (1600 MHz)
tCK (Clock Cycle):        0.625 ns

Command Timing:
tRCD (RAS to CAS):        14 tCK = 8.75 ns
tRP (Row Precharge):      14 tCK = 8.75 ns
tRC (Row Cycle):          48 tCK = 30 ns
tRAS (Row Active):        35 tCK = 21.875 ns

Data Timing:
tCL (CAS Latency):        14 tCK = 8.75 ns
tCWL (CAS Write Latency): 12 tCK = 7.5 ns
tWR (Write Recovery):     15 tCK = 9.375 ns

Bank Timing:
tRRD_S (Same Group):      4 tCK = 2.5 ns
tRRD_L (Different Group): 6 tCK = 3.75 ns
tCCD_S (Same Group):      4 tCK
tCCD_L (Different Group): 6 tCK

Refresh:
tREFI (Refresh Interval): 7.8 μs
tRFC (Refresh Cycle):     350 ns (8Gb), 550 ns (16Gb)
```

#### 1.3.2 DDR5 Protocol Timing (DDR5-4800)

```
Clock Period:             0.417 ns (2400 MHz)
tCK (Clock Cycle):        0.417 ns

Command Timing:
tRCD (RAS to CAS):        39 tCK = 16.25 ns
tRP (Row Precharge):      39 tCK = 16.25 ns
tRC (Row Cycle):          115 tCK = 47.9 ns
tRAS (Row Active):        76 tCK = 31.7 ns

Data Timing:
tCL (CAS Latency):        40 tCK = 16.67 ns
tCWL (CAS Write Latency): 38 tCK = 15.83 ns

Refresh:
tREFI (Refresh Interval): 3.9 μs (32ms/8192)
tRFC (Refresh per bank):  295 ns
```

### 1.4 Mode Registers

#### 1.4.1 DDR4 Mode Register 0 (MR0)
```
Bits [2:0]:   Burst Length (8 fixed)
Bits [3]:     Read Burst Type (Sequential/Interleaved)
Bits [6:4,2]: CAS Latency
Bits [8]:     DLL Reset
Bits [11:9]:  Write Recovery and RTP
Bit [12]:     Precharge PD DLL
```

#### 1.4.2 DDR5 Mode Register 0 (MR0)
```
Bits [5:0]:   CAS Latency
Bits [7:6]:   Read Burst Length
Bit [8]:      DFE (Decision Feedback Equalization)
Bits [11:9]:  WR and RTP
```

## 2. NAND Flash Protocol (ONFI)

### 2.1 ONFI Command Set

#### 2.1.1 Basic Commands

| Command | Opcode | Description |
|---------|--------|-------------|
| READ PAGE | 00h, 30h | Read page to buffer |
| PROGRAM PAGE | 80h, 10h | Program data to page |
| ERASE BLOCK | 60h, D0h | Erase entire block |
| READ STATUS | 70h | Get operation status |
| RESET | FFh | Reset device |
| READ ID | 90h | Read device ID |
| READ PARAM | ECh | Read parameter page |

#### 2.1.2 Advanced Commands

| Command | Opcode | Description |
|---------|--------|-------------|
| CHANGE READ COLUMN | 05h, E0h | Change column address during read |
| CHANGE WRITE COLUMN | 85h | Change column address during program |
| CACHE PROGRAM | 15h | Program with caching |
| MULTI-PLANE PROGRAM | 11h | Parallel program to multiple planes |
| COPYBACK | 00h, 8Ah, 10h | Copy page within NAND |

### 2.2 ONFI Timing Modes

#### 2.2.1 Asynchronous Modes

| Mode | tRC (ns) | tWC (ns) | Max Data Rate (MT/s) |
|------|----------|----------|---------------------|
| 0 | 100 | 100 | 10 |
| 1 | 50 | 50 | 20 |
| 2 | 35 | 35 | 28.6 |
| 3 | 30 | 30 | 33.3 |
| 4 | 25 | 25 | 40 |
| 5 | 20 | 20 | 50 |

#### 2.2.2 Synchronous (NV-DDR/NV-DDR2/NV-DDR3) Modes

| Mode | Interface | Max Data Rate |
|------|-----------|--------------|
| NV-DDR | Synchronous DDR | 200 MT/s |
| NV-DDR2 | Enhanced DDR | 400 MT/s |
| NV-DDR3 | High-speed DDR | 800-1200 MT/s |
| NV-DDR4 (ONFI 5.0) | Advanced DDR | 1600 MT/s |

### 2.3 NAND Flash State Machine

```
States:
- IDLE: Ready for command
- COMMAND: Receiving command
- ADDRESS: Receiving address cycles
- DATA_IN: Receiving data for program
- DATA_OUT: Transmitting data for read
- BUSY: Processing operation (program/erase/read)

Transitions:
IDLE --[CMD]-> COMMAND
COMMAND --[ADDR_REQUIRED]-> ADDRESS
ADDRESS --[PROGRAM_CMD]-> DATA_IN
ADDRESS --[READ_CMD]-> BUSY -> DATA_OUT
DATA_IN --[CONFIRM]-> BUSY -> IDLE
```

## 3. HBM Protocol

### 3.1 HBM2/HBM3 Command Structure

#### 3.1.1 Channel Organization

```
HBM2:  8 channels × 128 bits = 1024 bits total
HBM3: 16 channels × 64 bits = 1024 bits total
       (or 8 pseudo-channels × 128 bits)

Each channel operates independently with:
- 8 banks (HBM2) or 16 banks (HBM3) per channel
- Standard DRAM command set
- Simplified protocol vs DDR
```

#### 3.1.2 Command Protocol

```
Commands similar to DDR:
- ACTIVATE
- READ
- WRITE
- PRECHARGE
- REFRESH

Simplified features:
- No dual-rank complications
- Fixed burst length of 4
- Simplified power management
- Lower pin count per channel
```

### 3.2 HBM Timing Parameters

#### 3.2.1 HBM2 Timing (2.0 Gbps)

```
tCK (Clock Cycle):        1.0 ns (1.0 GHz)
tCL (CAS Latency):        14-16 tCK
tRCD (RAS to CAS):        12-14 tCK
tRP (Row Precharge):      12-14 tCK
tRAS (Row Active):        28-33 tCK
tRC (Row Cycle):          40-47 tCK
```

#### 3.2.2 HBM3 Timing (6.4 Gbps)

```
tCK (Clock Cycle):        0.3125 ns (3.2 GHz DDR)
tCL (CAS Latency):        16-20 tCK
tRCD (RAS to CAS):        14-18 tCK
tRP (Row Precharge):      14-18 tCK
tRAS (Row Active):        32-42 tCK
```

## 4. CXL Memory Protocol

### 4.1 CXL Protocol Layers

#### 4.1.1 Three Sub-Protocols

```
CXL.io:
- PCIe-based I/O protocol
- Non-coherent transactions
- Device discovery and enumeration

CXL.cache:
- Device can cache host memory
- Coherent caching protocol
- D2H (Device-to-Host) requests

CXL.mem:
- Host accesses device-attached memory
- H2D (Host-to-Device) requests
- Memory semantics, not I/O
```

### 4.2 CXL.mem Transaction Types

```
Read Transactions:
- MemRd: Standard memory read
- MemRdData: Read with data return
- MemInv: Read with invalidation

Write Transactions:
- MemWr: Standard memory write
- MemWrPtl: Partial write
- MemWrFwd: Write with forward to cache

Cache Coherency:
- SnpData: Snoop with data
- SnpInv: Snoop with invalidation
- SnpCur: Snoop current state
```

### 4.3 CXL Memory Addressing

```
Memory can be:
1. Volatile (DRAM, HBM)
2. Persistent (CXL-attached PMEM)
3. Pooled (shared across multiple hosts)

Address Space:
- Direct-attached: Standard memory range
- Switched: Routed through CXL switch
- Pooled: Global address space
```

## 5. LPDDR Protocol

### 5.1 LPDDR5 Commands

#### 5.1.1 Command Set Extensions

```
Standard DDR commands plus:
- DPD (Deep Power-Down): Ultra-low power state
- SRE/SRX (Self-Refresh Entry/Exit): Enhanced self-refresh
- FSP (Frequency Set Point): Runtime frequency switching
- WCK (Write Clock) control: Separate write clock
```

### 5.2 Multi-Frequency Operation

```
LPDDR5 supports multiple frequency set points:

FSP[0]: High performance (up to 6400 MT/s)
FSP[1]: Medium performance (3200 MT/s typical)
FSP[2]: Low power (800-1600 MT/s)

Switching procedure:
1. Issue MRW to change FSP
2. Disable WCK (Write Clock)
3. Change frequency
4. Enable WCK at new frequency
5. Recalibrate if needed
```

## 6. Signal Integrity Protocols

### 6.1 On-Die Termination (ODT)

```
ODT Control:
- Dynamic ODT: Enabled only during writes
- Static ODT: Always enabled
- RTT_NOM: Nominal termination (40Ω, 48Ω, 60Ω, 120Ω)
- RTT_WR: Write termination (80Ω, 120Ω, 240Ω, ∞)

Mode Register Programming:
MR1[8,2:0]: RTT_NOM settings
MR2[10:9]: RTT_WR settings
```

### 6.2 Data Bus Inversion (DBI)

```
DBI Algorithm:
1. Count number of '1's in data byte
2. If count > 4, invert all bits
3. Set DBI pin to indicate inversion

Benefits:
- Reduces simultaneous switching noise
- Lowers power consumption
- Improves signal integrity

DDR4: DBI optional
DDR5: DBI mandatory
```

### 6.3 Write Leveling and Training

```
Write Leveling:
Purpose: Align DQS with CK at DRAM
Procedure:
1. Enter write leveling mode (MRS)
2. Toggle DQS
3. DRAM samples DQS with CK
4. Adjust DQS delay until aligned
5. Exit write leveling mode

Read Training:
Purpose: Find optimal sampling point for DQ
Procedure:
1. Sweep RX delay across valid window
2. Find passing region
3. Set delay to center of window
4. Repeat for all DQ bits
```

## 7. Protocol Compliance Testing

### 7.1 Protocol Violations

Common violations to test:
- Timing parameter violations (tRCD, tRP, etc.)
- Command sequence errors
- Bank conflict errors
- Refresh timing violations
- Setup/hold time violations

### 7.2 Protocol Analyzers

Requirements:
- Capture all command/address signals
- Decode commands in real-time
- Check timing against JEDEC specs
- Flag protocol violations
- Generate test patterns

---

**Document Control:**
- **Author:** WIA Standards Committee
- **Approved By:** WIA Technical Board
- **Next Review:** 2026-01-26
- **Classification:** Public Standard

© 2025 World Certification Industry Association (WIA)
弘益人間 · Benefit All Humanity
