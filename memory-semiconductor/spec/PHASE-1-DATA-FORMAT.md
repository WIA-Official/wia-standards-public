# WIA-SEMI-002 PHASE 1: DATA FORMAT SPECIFICATION

**Version:** 1.0  
**Date:** 2025-01-26  
**Status:** Official Standard

## Overview

Phase 1 defines the data format specifications for memory semiconductor devices, including capacity organization, timing parameters, electrical characteristics, and interface definitions for DRAM, NAND Flash, HBM, and emerging memory technologies.

## 1. DRAM Data Format

### 1.1 Memory Organization

#### 1.1.1 DDR4 Organization
- **Capacity per Die:** 2Gb, 4Gb, 8Gb, 16Gb
- **Data Width:** x4, x8, x16 configurations
- **Bank Structure:** 4 bank groups × 4 banks = 16 total banks
- **Row Address:** 16-18 bits (65,536 - 262,144 rows)
- **Column Address:** 10-11 bits (1,024 - 2,048 columns)

#### 1.1.2 DDR5 Organization
- **Capacity per Die:** 8Gb, 16Gb, 24Gb, 32Gb
- **Data Width:** x4, x8, x16 configurations
- **Bank Structure:** 8 bank groups × 4 banks = 32 total banks
- **Channel Architecture:** Dual 32-bit channels per module
- **Row Address:** 17-18 bits
- **Column Address:** 10-11 bits

### 1.2 Timing Parameters

#### 1.2.1 DDR4 Timing (DDR4-3200 Example)
```
tCL (CAS Latency):           14-22 cycles
tRCD (RAS to CAS Delay):     14-22 cycles
tRP (Row Precharge Time):    14-22 cycles
tRAS (Row Active Time):      32-52 cycles
tRC (Row Cycle Time):        46-74 cycles
tRFC (Refresh Cycle Time):   350-560 ns
tCCD_S (Short):              4 cycles
tCCD_L (Long):               6 cycles
tWR (Write Recovery):        15-24 cycles
```

#### 1.2.2 DDR5 Timing (DDR5-4800 Example)
```
tCL (CAS Latency):           40 cycles
tRCD (RAS to CAS Delay):     39 cycles
tRP (Row Precharge Time):    39 cycles
tRAS (Row Active Time):      76 cycles
tRC (Row Cycle Time):        115 cycles
tRFC (Refresh Cycle Time):   295 ns (per bank)
tCCD_S (Short):              8 cycles
tCCD_L (Long):               8 cycles
```

### 1.3 Electrical Characteristics

#### 1.3.1 DDR4 Electrical Specifications
- **Operating Voltage (VDD):** 1.2V ± 60mV
- **I/O Voltage (VDDQ):** 1.2V ± 60mV
- **Termination Voltage (VTT):** VDD/2
- **Input High Voltage (VIH):** VREF + 0.1V (min)
- **Input Low Voltage (VIL):** VREF - 0.1V (max)
- **Reference Voltage (VREF):** 0.6V (VDD/2)

#### 1.3.2 DDR5 Electrical Specifications
- **Operating Voltage (VDD):** 1.1V ± 50mV
- **I/O Voltage (VDDQ):** 1.1V ± 50mV
- **Termination Voltage (VTT):** VDD/2
- **Input High Voltage (VIH):** VREF + 0.095V (min)
- **Input Low Voltage (VIL):** VREF - 0.095V (max)

## 2. NAND Flash Data Format

### 2.1 Page and Block Organization

#### 2.1.1 SLC NAND
- **Page Size:** 4KB, 8KB, 16KB
- **Block Size:** 256KB - 4MB (64-256 pages per block)
- **Planes per Die:** 1-4 planes
- **Bits per Cell:** 1 bit (2 voltage levels)

#### 2.1.2 MLC/TLC/QLC NAND
- **MLC:** 2 bits/cell (4 voltage levels)
- **TLC:** 3 bits/cell (8 voltage levels)
- **QLC:** 4 bits/cell (16 voltage levels)
- **Page Types:** Lower, Middle, Upper, Top (for multi-level cells)

### 2.2 3D NAND Specifications

#### 2.2.1 V-NAND Stack Structure
- **Layer Count:** 96, 128, 176, 232 layers
- **String Architecture:** Vertical channel with charge trap
- **Word Line Pitch:** 40-60 nm typical
- **Channel Diameter:** 80-120 nm typical

### 2.3 Timing Parameters

#### 2.3.1 SLC NAND Timing
```
tPROG (Program Time):        200-300 μs
tR (Read Time):              25-50 μs
tBERS (Block Erase Time):    2-5 ms
tWC (Write Cycle Time):      25-50 ns
```

#### 2.3.2 TLC NAND Timing
```
tPROG (Program Time):        600-1500 μs
tR (Read Time):              50-100 μs
tBERS (Block Erase Time):    3-10 ms
```

## 3. HBM Data Format

### 3.1 HBM2 Organization

- **Interface Width:** 1024 bits per stack (8 channels × 128 bits)
- **Capacity per Stack:** 4GB, 8GB, 12GB, 16GB
- **Stack Height:** 4-8 dies
- **Data Rate:** 2.0 Gbps per pin (HBM2), 3.6 Gbps (HBM2E)
- **Bandwidth per Stack:** 256 GB/s (HBM2), 461 GB/s (HBM2E)

### 3.2 HBM3 Organization

- **Interface Width:** 1024 bits per stack (16 channels × 64 bits)
- **Capacity per Stack:** 16GB, 24GB, 32GB
- **Stack Height:** 8-16 dies
- **Data Rate:** 5.2-6.4 Gbps per pin
- **Bandwidth per Stack:** 665-819 GB/s

### 3.3 HBM Timing

```
tCL (CAS Latency):           14-18 cycles
tRCD (RAS to CAS Delay):     12-16 cycles
tRP (Row Precharge):         12-16 cycles
tRAS (Row Active Time):      28-38 cycles
```

## 4. LPDDR Data Format

### 4.1 LPDDR5 Organization

- **Capacity:** 4Gb, 6Gb, 8Gb, 12Gb, 16Gb per die
- **Data Width:** x16 (16-bit)
- **Banks:** 16 banks (8 bank groups × 2 banks)
- **Channels:** Dual-channel architecture
- **Data Rate:** 6400 MT/s (LPDDR5X: 8533 MT/s)

### 4.2 LPDDR5 Electrical Specifications

- **VDD1 (Core Voltage):** 1.05V
- **VDD2 (I/O Voltage):** 0.5V
- **VDDQ (Data I/O Voltage):** 0.5V

## 5. Emerging Memory Data Formats

### 5.1 MRAM (STT-MRAM)

- **Cell Size:** 20-40 F²
- **Read Latency:** 10-20 ns
- **Write Latency:** 10-30 ns
- **Endurance:** >10^15 cycles
- **Retention:** 10+ years at operating temperature

### 5.2 PCM (Phase-Change Memory)

- **Cell Size:** 4-20 F²
- **Read Latency:** 50 ns
- **Write Latency:** 100-200 ns (SET), 50-100 ns (RESET)
- **Endurance:** 10^8 - 10^9 cycles
- **Multi-Level:** 2-4 bits per cell achievable

### 5.3 ReRAM (Resistive RAM)

- **Cell Size:** 4-10 F²
- **Read Latency:** 10-50 ns
- **Write Latency:** 10-100 ns
- **Endurance:** 10^6 - 10^9 cycles
- **Resistance Ratio:** 10-1000x (HRS/LRS)

## 6. Data Format Standards Compliance

### 6.1 JEDEC Standards

- **JESD79-4:** DDR4 SDRAM Standard
- **JESD79-5:** DDR5 SDRAM Standard
- **JESD209-4:** LPDDR4 Standard
- **JESD209-5:** LPDDR5 Standard
- **JESD235A:** HBM2 Standard
- **JESD238:** HBM3 Standard

### 6.2 ONFI Standards

- **ONFI 4.2:** NAND Flash Interface (up to 1200 MT/s)
- **ONFI 5.0:** NAND Flash Interface (up to 1600 MT/s, NV-DDR4)

## 7. Bandwidth Calculations

### 7.1 DDR Bandwidth Formula

```
Bandwidth (GB/s) = (Data Rate in MT/s × Bus Width in bits) / 8

Example DDR4-3200:
Bandwidth = (3200 × 64) / 8 = 25.6 GB/s per channel
```

### 7.2 HBM Bandwidth Formula

```
Bandwidth per Stack = (Data Rate in Gbps × Interface Width in bits) / 8

Example HBM3 at 6.4 Gbps:
Bandwidth = (6.4 × 1024) / 8 = 819.2 GB/s per stack
```

## 8. Data Integrity

### 8.1 ECC Specifications

#### 8.1.1 DDR4 ECC
- **Algorithm:** SEC-DED (Single Error Correction, Double Error Detection)
- **Overhead:** 8 ECC bits per 64 data bits (~12.5%)
- **Data Width:** 72-bit (64 data + 8 ECC)

#### 8.1.2 DDR5 On-Die ECC
- **Algorithm:** On-die ECC within each chip
- **Transparent:** Corrects single-bit errors internally
- **Optional:** Additional system-level sideband ECC

#### 8.1.3 NAND Flash ECC
- **SLC:** Hamming or BCH (1-4 bit correction per 512B)
- **MLC/TLC:** BCH (40-80 bit correction per 1KB)
- **QLC:** LDPC (80-200+ bit correction per 1KB)

## 9. Temperature Specifications

### 9.1 Operating Temperature Ranges

- **Consumer DRAM:** 0°C to 85°C
- **Industrial DRAM:** -40°C to 85°C
- **Automotive (AEC-Q100 Grade 2):** -40°C to 105°C
- **NAND Flash:** -40°C to 85°C (typically)

### 9.2 Temperature-Dependent Parameters

- **Refresh Rate:** Doubles every 10°C above 85°C
- **Data Retention:** Decreases at elevated temperatures
- **tREFI (Refresh Interval):** 7.8 μs (normal), 3.9 μs (high temp)

## 10. Mode Register Encoding

WIA-SEMI-002 reuses the JEDEC mode-register conventions and adds a vendor-portable JSON manifest so that platform firmware, BIOS, and NVMe-MI tooling can interpret a part without proprietary blobs.

### 10.1 DDR5 Mode Register (MR0–MR63) Manifest

```json
{
  "schemaVersion": "wia.semi-002.mr-manifest/1",
  "device": "DDR5-4800B",
  "vendorId": "0x80AD",
  "modeRegisters": [
    {"mr": 0,  "name": "WL/READL/RL", "fields": [
      {"name": "CL", "bits": "5:0", "value": 40, "units": "tCK"},
      {"name": "WL", "bits": "11:6", "value": 38, "units": "tCK"}
    ]},
    {"mr": 12, "name": "Vref(DQ)", "fields": [
      {"name": "VrefDQ", "bits": "5:0", "value": 0x18, "units": "0.65% Vdd step"}
    ]},
    {"mr": 51, "name": "RTT_CK/RTT_NOM", "fields": [
      {"name": "RTT_CK", "bits": "2:0", "value": 5},
      {"name": "RTT_NOM_WR", "bits": "5:3", "value": 4}
    ]}
  ],
  "trainingFlows": ["CA training", "Read training", "Write leveling", "Per-DRAM addressability"]
}
```

### 10.2 NVMe Identify Controller Excerpt

For SSD-class NAND assemblies the standard MUST surface a NVMe Identify Controller (CNS=0x01) blob to OS-level discovery. The manifest mirrors the binary layout but is JSON-encoded for tooling:

```json
{
  "schemaVersion": "wia.semi-002.nvme-id/1",
  "VID": "0x1AF4",
  "SSVID": "0x1AF4",
  "MN": "WIA-CONFORMANT-NVME-001",
  "FR": "1.0.0",
  "RAB": 6,
  "OAES": "0x00000200",
  "CTRATT": "0x0000003F",
  "SQES": {"min": 6, "max": 6},
  "CQES": {"min": 4, "max": 4},
  "MAXCMD": 256,
  "NN": 32,
  "ONCS": "0x005F"
}
```

### 10.3 SPD (Serial Presence Detect) Profile

DIMMs MUST publish a JEDEC SPD page with the following minimum fields populated:

| Byte | Field | Required | Notes |
|------|-------|----------|-------|
| 0x00 | Bytes used | Yes | 0x23 for DDR5 |
| 0x02 | Memory type | Yes | 0x12 = DDR5 |
| 0x03 | Module type | Yes | UDIMM / RDIMM / LRDIMM |
| 0x14 | tCKAVGmin | Yes | ps |
| 0x18 | tAAmin | Yes | ps |
| 0x80 | Module manufacturer | Yes | JEDEC bank+code |
| 0x95 | Module serial number | Yes | 4-byte LE |

Hosts MUST read SPD at every cold boot and validate the WIA hash extension at byte 0x1FE–0x1FF (CRC-16/XMODEM over bytes 0x00–0x1FD).

## 11. Telemetry Stream Format

For real-time observability (PMU counters, refresh-pause events, RAS errors) every device MUST stream telemetry over an out-of-band channel using the WIA telemetry frame:

```
+--------+--------+----------+----------------+----------+
| ver(1) | type(1)| ts_ns(8) | payload_len(2) | payload  |
+--------+--------+----------+----------------+----------+
```

| Type | Meaning |
|------|---------|
| 0x01 | RAS error log entry |
| 0x02 | Thermal event |
| 0x03 | Refresh-rate change |
| 0x04 | Wear-level update (NAND) |
| 0x05 | Bit-error histogram |

Frames are encoded as CBOR per RFC 8949 inside the payload area. Aggregators MAY upgrade to CBOR Sequences (RFC 8742) for high-rate streams.

## 12. Normative References

- ISO/IEC 17025:2017 — Testing and calibration laboratories
- IEC 60068-2 — Environmental testing
- JEDEC JESD79-4D — DDR4 SDRAM
- JEDEC JESD79-5C — DDR5 SDRAM
- JEDEC JESD235D — High Bandwidth Memory (HBM3)
- JEDEC JESD218B — Solid-State Drive Endurance Workloads
- IETF RFC 8949 — Concise Binary Object Representation (CBOR)
- IETF RFC 8742 — CBOR Sequences
- IETF RFC 8259 — JSON Data Interchange Format
- AEC-Q100 — Failure Mechanism-Based Stress Test for ICs

---

**Document Control:**
- **Author:** WIA Standards Committee
- **Approved By:** WIA Technical Board
- **Next Review:** 2026-01-26
- **Classification:** Public Standard

© 2025 World Certification Industry Association (WIA)
弘益人間 · Benefit All Humanity
