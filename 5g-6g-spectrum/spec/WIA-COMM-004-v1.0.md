# WIA-COMM-004: 5G/6G Spectrum Specification v1.0

> **Standard ID:** WIA-COMM-004
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Communication Standards Working Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Frequency Ranges and Bands](#2-frequency-ranges-and-bands)
3. [Dynamic Spectrum Access](#3-dynamic-spectrum-access)
4. [Spectrum Sharing Mechanisms](#4-spectrum-sharing-mechanisms)
5. [Beamforming and MIMO](#5-beamforming-and-mimo)
6. [Carrier Aggregation](#6-carrier-aggregation)
7. [Spectrum Efficiency](#7-spectrum-efficiency)
8. [Interference Management](#8-interference-management)
9. [Regulatory Framework](#9-regulatory-framework)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive standards for 5G/6G spectrum management, allocation, and utilization across Sub-6 GHz, millimeter wave (mmWave), and emerging Terahertz frequency bands.

### 1.2 Scope

The standard covers:
- FR1 (Sub-6 GHz), FR2 (mmWave), and FR3 (proposed 6G) spectrum
- Terahertz bands (100 GHz - 3 THz) for future 6G
- Dynamic spectrum access and sharing mechanisms
- Advanced antenna technologies (beamforming, massive MIMO)
- Carrier aggregation across multiple bands
- Spectrum efficiency metrics and optimization
- Interference management and coexistence
- Regulatory compliance (ITU, 3GPP, national authorities)

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize wireless spectrum access, enabling universal connectivity and bridging the digital divide through efficient, shared, and innovative spectrum utilization.

### 1.4 Terminology

- **5G NR**: 5G New Radio (3GPP specification)
- **FR1**: Frequency Range 1 (Sub-6 GHz)
- **FR2**: Frequency Range 2 (mmWave, 24-52 GHz)
- **FR3**: Frequency Range 3 (proposed, 7-24 GHz)
- **TDD**: Time Division Duplex
- **FDD**: Frequency Division Duplex
- **CBRS**: Citizens Broadband Radio Service
- **DSA**: Dynamic Spectrum Access
- **MIMO**: Multiple-Input Multiple-Output
- **CA**: Carrier Aggregation
- **SCS**: Subcarrier Spacing
- **MCS**: Modulation and Coding Scheme

---

## 2. Frequency Ranges and Bands

### 2.1 3GPP Frequency Range Definitions

#### 2.1.1 FR1 (Sub-6 GHz)

**Specification:**
```
Frequency Range: 410 MHz - 7.125 GHz
Duplex Modes: FDD, TDD, SDL (Supplementary Downlink)
Channel Bandwidths: 5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100 MHz
Subcarrier Spacing: 15 kHz, 30 kHz, 60 kHz
Maximum Bandwidth: 100 MHz per carrier
```

**Characteristics:**
- Wide coverage area (cells: 1-30 km radius)
- Good building penetration
- Lower data rates (100 Mbps - 2 Gbps)
- Primary coverage layer

**Key FR1 Bands:**

| Band | Frequency (UL / DL) | Duplex | Region | Bandwidth |
|------|---------------------|--------|--------|-----------|
| n1 | 1920-1980 / 2110-2170 MHz | FDD | Global | 60 MHz |
| n3 | 1710-1785 / 1805-1880 MHz | FDD | Global | 75 MHz |
| n5 | 824-849 / 869-894 MHz | FDD | Americas | 25 MHz |
| n7 | 2500-2570 / 2620-2690 MHz | FDD | Europe, Asia | 70 MHz |
| n8 | 880-915 / 925-960 MHz | FDD | Global | 35 MHz |
| n20 | 832-862 / 791-821 MHz | FDD | Europe | 30 MHz |
| n28 | 703-748 / 758-803 MHz | FDD | Asia-Pacific | 45 MHz |
| n41 | 2496-2690 MHz | TDD | Global | 194 MHz |
| n71 | 663-698 / 617-652 MHz | FDD | USA | 35 MHz |
| n77 | 3300-4200 MHz | TDD | Global | 900 MHz |
| n78 | 3300-3800 MHz | TDD | Europe, Asia | 500 MHz |
| n79 | 4400-5000 MHz | TDD | China | 600 MHz |

#### 2.1.2 FR2 (mmWave)

**Specification:**
```
Frequency Range: 24.25 GHz - 52.6 GHz
Duplex Mode: TDD only
Channel Bandwidths: 50, 100, 200, 400 MHz
Subcarrier Spacing: 60 kHz, 120 kHz
Maximum Bandwidth: 400 MHz per carrier
```

**Characteristics:**
- Limited coverage (cells: 50-500 meters)
- High path loss and atmospheric absorption
- Ultra-high data rates (1-20 Gbps)
- Beamforming required

**Key FR2 Bands:**

| Band | Frequency Range | Bandwidth | Region | Notes |
|------|----------------|-----------|--------|-------|
| n257 | 26.5-29.5 GHz | 3000 MHz | Global | 28 GHz band |
| n258 | 24.25-27.5 GHz | 3250 MHz | Global | 26 GHz band |
| n260 | 37-40 GHz | 3000 MHz | USA | 39 GHz band |
| n261 | 27.5-28.35 GHz | 850 MHz | Global | Extended 28 GHz |

#### 2.1.3 FR3 (Proposed 6G)

**Specification:**
```
Frequency Range: 7.125 GHz - 24.25 GHz
Status: Under discussion (WRC-23, WRC-27)
Duplex Mode: Primarily TDD
Target Bandwidths: 100-500 MHz
```

**Characteristics:**
- Balance between coverage and capacity
- Bridge between Sub-6 GHz and mmWave
- Moderate building penetration
- Data rates: 1-5 Gbps

**Candidate Bands:**
- 7-8 GHz (IMT-2020 extension)
- 10-10.5 GHz
- 14.8-15.35 GHz
- 15.35-16.2 GHz (co-primary with satellite)

### 2.2 Terahertz Spectrum (6G Vision)

#### 2.2.1 Sub-THz Bands (100-300 GHz)

**D-Band (110-170 GHz):**
```
Bandwidth: 60 GHz
Atmospheric Windows: 130-175 GHz
Path Loss: ~140 dB at 100m
Applications: Indoor wireless, backhaul
```

**G-Band (140-220 GHz):**
```
Bandwidth: 80 GHz
Atmospheric Absorption: Moderate (water vapor)
Path Loss: ~145 dB at 100m
Applications: Wireless fronthaul/backhaul
```

#### 2.2.2 THz Bands (300 GHz - 3 THz)

**Characteristics:**
- Ultra-wide bandwidth (1-10 GHz channels)
- Extreme path loss (>160 dB at 10m)
- Atmospheric absorption peaks (oxygen: 60 GHz, 120 GHz, 180 GHz; water: 22 GHz, 180 GHz)
- Line-of-sight required
- Indoor/short-range only (< 10 meters)

**Potential Bands:**
- 275-296 GHz
- 306-313 GHz
- 318-333 GHz
- 356-450 GHz
- 500-1000 GHz
- 1-3 THz

---

## 3. Dynamic Spectrum Access

### 3.1 Spectrum Sharing Models

#### 3.1.1 Licensed Shared Access (LSA)

**Framework:**
```
Primary Users: Incumbents (government, satellite, etc.)
Secondary Users: Mobile operators
Coordination: LSA repository and controller
Protection: Exclusion zones and power limits
```

**Process Flow:**
1. Secondary user requests spectrum access
2. LSA repository checks incumbent usage
3. LSA controller grants access if available
4. Real-time monitoring and enforcement
5. Immediate vacation if incumbent needs spectrum

**Parameters:**
```json
{
  "lsaRequest": {
    "operator": "MobileOperator-A",
    "frequencyRange": { "min": 2300, "max": 2400 },
    "location": {
      "latitude": 51.5074,
      "longitude": -0.1278,
      "radius": 5000
    },
    "maxPower": 46,
    "duration": 86400
  }
}
```

#### 3.1.2 Citizens Broadband Radio Service (CBRS)

**Three-Tier Framework:**

**Tier 1 - Incumbents:**
- Priority: Highest
- Users: Federal government (Navy radar)
- Frequency: 3550-3700 MHz (all 150 MHz)
- Protection: Exclusion zones (mandatory)

**Tier 2 - Priority Access License (PAL):**
- Priority: Medium
- Users: Licensed operators (auction-based)
- Frequency: 3550-3650 MHz (7 channels × 10 MHz)
- Protection: License area (county-based)
- License Term: 3 years, renewable

**Tier 3 - General Authorized Access (GAA):**
- Priority: Lowest
- Users: Unlicensed (open access)
- Frequency: 3550-3700 MHz (opportunistic)
- Protection: None (best effort)
- Power Limit: 30 dBm EIRP (outdoor), 24 dBm (indoor)

**SAS (Spectrum Access System) Architecture:**

```
┌─────────────────────────────────────────────┐
│         Spectrum Access System (SAS)         │
├─────────────────────────────────────────────┤
│  - Registration and authentication          │
│  - Spectrum allocation engine                │
│  - Interference analysis                     │
│  - Incumbent protection                      │
│  - Grant management                          │
└─────────────────────────────────────────────┘
              ↕                    ↕
      ┌───────────────┐    ┌─────────────┐
      │   CBSD         │    │   ESC       │
      │ (Base Station) │    │ (Sensor)    │
      └───────────────┘    └─────────────┘
```

**CBSD Registration:**
```json
{
  "cbsdId": "CBSD-2024-00123",
  "userId": "operator-xyz",
  "fccId": "ABC123DEF456",
  "cbsdCategory": "B",
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "height": 30,
    "heightType": "AGL",
    "horizontalAccuracy": 10,
    "indoorDeployment": false
  },
  "antennaGain": 17,
  "requestedPower": 46
}
```

#### 3.1.3 Licensed Assisted Access (LAA)

**Specification:**
- Frequency: 5 GHz unlicensed (5150-5925 MHz)
- Technology: LTE-LAA (Rel-13), NR-U (Rel-16)
- Coexistence: Listen-Before-Talk (LBT)
- Channel Bandwidth: 20 MHz
- Aggregation: Up to 5 carriers

**Listen-Before-Talk (LBT):**
```
Categories:
- Cat 1: No LBT (one-shot transmission)
- Cat 2: LBT without random backoff
- Cat 3: LBT with random backoff (defer > 25 μs)
- Cat 4: LBT with extended contention window

Energy Detection Threshold: -82 dBm/MHz (Category 4)
Maximum Channel Occupancy Time: 4 ms (Cat 4)
Idle Period: 5% of channel occupancy time
```

### 3.2 Database-Driven Spectrum Access

#### 3.2.1 Geo-Location Database

**Components:**
```
Database Elements:
- Incumbent transmitter locations
- Exclusion zones and protection contours
- Available channels by location
- Maximum permitted EIRP
- Operational parameters
```

**Query Format:**
```json
{
  "deviceDescriptor": {
    "serialNumber": "DEV-2024-98765",
    "manufacturer": "Acme Wireless",
    "model": "5G-CPE-Pro",
    "rulesetIds": ["US_47_CFR_PART_96"]
  },
  "location": {
    "latitude": 40.7128,
    "longitude": -74.0060,
    "height": 10,
    "uncertainty": 50
  },
  "antenna": {
    "gain": 6,
    "azimuth": 0,
    "beamwidth": 360
  }
}
```

**Response:**
```json
{
  "availableChannels": [
    {
      "frequencyRange": { "low": 3600, "high": 3620 },
      "maxEirp": 36,
      "ruleApplied": "GAA"
    },
    {
      "frequencyRange": { "low": 3640, "high": 3650 },
      "maxEirp": 47,
      "ruleApplied": "PAL"
    }
  ],
  "timestamp": "2025-12-26T10:00:00Z",
  "validUntil": "2025-12-26T11:00:00Z"
}
```

---

## 4. Spectrum Sharing Mechanisms

### 4.1 Interference Mitigation Techniques

#### 4.1.1 Time-Domain Separation

**Almost Blank Subframe (ABS):**
```
Purpose: Reduce inter-cell interference
Mechanism: Aggressor cell mutes transmission in specific subframes
Pattern: Configurable bitmap (e.g., "10001000" = 2 ABS per 8 subframes)
Application: LTE/NR heterogeneous networks
```

**Time Division Multiplex (TDM):**
```
Principle: Orthogonal time allocation between users
Guard Time: 5-50 μs (depends on propagation delay)
Efficiency: ~90% (accounting for guard times and overhead)
```

#### 4.1.2 Frequency-Domain Separation

**Guard Bands:**
```
Co-channel: 0 MHz (orthogonal)
Adjacent channel (5G): 0-5 MHz
Adjacent channel (LTE-5G): 5-10 MHz
Out-of-band: Filter-dependent (30-50 dB rejection)
```

**Filtering Requirements:**
```
Base Station (BS):
- Adjacent Channel Leakage Ratio (ACLR): 45 dB
- Spurious Emissions: < -30 dBm/MHz

User Equipment (UE):
- ACLR: 30 dB
- Spurious Emissions: < -30 dBm/MHz
```

#### 4.1.3 Power Control

**Fractional Power Control:**
```
P_tx = min(P_max, P_0 + α × PL + 10log10(BW) + MCS_offset + Δ_TF)

Where:
- P_max: Maximum transmit power (23 dBm for UE, 46 dBm for BS)
- P_0: Target received power (-100 dBm typical)
- α: Path loss compensation factor (0.8 typical, range: 0-1)
- PL: Path loss estimate (dB)
- BW: Allocated bandwidth (PRBs)
- MCS_offset: Modulation-specific adjustment
- Δ_TF: Transport format offset
```

**Dynamic Power Reduction:**
```
Scenarios:
- Proximity to exclusion zone: -10 to -20 dB
- High interference environment: -3 to -6 dB
- Indoor deployment: -6 to -10 dB
- Energy saving: -3 to -10 dB
```

#### 4.1.4 Spatial Separation

**Beamforming:**
```
Benefits:
- Directional gain: 10-20 dB
- Interference reduction: 15-30 dB (nulling)
- Coverage extension: 2-4x range
- Capacity increase: 3-10x
```

**Beam Nulling:**
```
Principle: Shape antenna pattern to minimize radiation toward interferers
Null Depth: 20-40 dB
Degrees of Freedom: N-1 nulls for N-element array
Application: Interference mitigation in shared spectrum
```

### 4.2 Coexistence Mechanisms

#### 4.2.1 5G-Wi-Fi Coexistence (6 GHz)

**Unlicensed National Information Infrastructure (U-NII):**
```
U-NII-5: 5.925-6.425 GHz (500 MHz)
U-NII-6: 6.425-6.525 GHz (100 MHz)
U-NII-7: 6.525-6.875 GHz (350 MHz)
U-NII-8: 6.875-7.125 GHz (250 MHz)

Total: 1200 MHz unlicensed
```

**5G NR-Unlicensed (NR-U) in 6 GHz:**
```
LBT Category: Cat 4 (with extended CW)
Channel Bandwidth: 20, 40, 80, 160 MHz
Maximum Channel Occupancy: 8 ms
Energy Detection: -82 dBm/MHz
Defer Period: 16-1024 μs (exponential backoff)
```

**Wi-Fi 6E (IEEE 802.11ax):**
```
Channels: 59 × 20 MHz, 29 × 40 MHz, 14 × 80 MHz, 7 × 160 MHz
OFDMA: Yes
MU-MIMO: Yes (up to 8 users)
Maximum Throughput: 9.6 Gbps
```

---

## 5. Beamforming and MIMO

### 5.1 Massive MIMO

#### 5.1.1 Architecture

**Antenna Configurations:**
```
Common Arrays:
- 64 TxRU: 8×8 (8 rows × 8 columns)
- 128 TxRU: 8×16 or 16×8
- 256 TxRU: 16×16
- 512 TxRU: 16×32 (future)

Element Spacing: 0.5λ (half wavelength)
Polarization: Dual ±45° (XPOL)
Effective Elements: Physical × 2 (dual-pol)
```

**Beamforming Gain:**
```
Array Gain (dB) = 10 × log10(N_elements)

Examples:
- 64 elements: 18 dB
- 128 elements: 21 dB
- 256 elements: 24 dB
```

#### 5.1.2 Beamforming Types

**Analog Beamforming:**
```
Implementation: Phase shifters
Beams: Single beam per RF chain
Flexibility: Low
Cost: Low
Application: FR2 (mmWave)
```

**Digital Beamforming:**
```
Implementation: Baseband processing
Beams: Multiple simultaneous beams
Flexibility: High
Cost: High
Application: FR1 (Sub-6 GHz), Massive MIMO
```

**Hybrid Beamforming:**
```
Implementation: Analog + digital combination
Beams: Multiple beams with reduced RF chains
Flexibility: Medium
Cost: Medium
Application: FR2 with multi-user MIMO
```

**Beam Management Procedures:**

1. **Beam Sweeping (P-1):**
   ```
   - gNB transmits SSB in different beam directions
   - UE measures RSRP for each beam
   - Coverage: Full azimuth (0-360°), limited elevation
   - SSB Period: 20 ms (typical)
   - Beams per Burst: 4-64 (depends on frequency)
   ```

2. **Beam Determination (P-2):**
   ```
   - UE selects best gNB Tx beam
   - UE reports preferred beam index
   - Measurement: RSRP, SINR
   - Reporting: Periodic or event-triggered
   ```

3. **Beam Refinement (P-3):**
   ```
   - Fine-tune beam direction
   - Use CSI-RS (channel state information)
   - Narrow beam width (<10°)
   - Higher gain (3-6 dB improvement)
   ```

### 5.2 Multi-User MIMO (MU-MIMO)

**Specifications:**
```
Maximum Users (DL): 12 (FR1), 16 (FR2)
Maximum Users (UL): 4 (FR1), 8 (FR2)
Orthogonalization: Spatial division
Precoding: ZF, MMSE, SVD-based
```

**Spatial Multiplexing Gain:**
```
Capacity = BW × log2(1 + SINR × min(N_tx, N_rx))

Example (4×4 MIMO, SINR=20 dB):
C = 100 MHz × log2(1 + 100 × 4) ≈ 873 Mbps (theoretical)
```

**Precoding Matrices:**

Zero-Forcing (ZF):
```
W = H^H × (H × H^H)^-1

Where:
- H: Channel matrix
- W: Precoding matrix
- H^H: Hermitian transpose
```

### 5.3 Beamforming for mmWave

**Beam Patterns:**
```
Beam Width: 5-30° (depends on array size)
Side Lobe Level: < -20 dB
Front-to-Back Ratio: > 25 dB
Scan Range: ±60° (mechanical + electronic)
```

**Beam Tracking:**
```
Methods:
1. Periodic beam sweep
2. Differential codebook feedback
3. Compressed sensing
4. Machine learning-based prediction

Update Rate: 10-100 ms
Overhead: 5-10% resources
```

---

## 6. Carrier Aggregation

### 6.1 Carrier Aggregation Types

#### 6.1.1 Intra-Band Contiguous CA

**Configuration:**
```
Bands: Single band (e.g., n77)
Carriers: Adjacent frequency blocks
Maximum CCs: 16
Maximum Bandwidth: 16 × 100 MHz = 1600 MHz (theoretical)
Example: n77 (3400-3500 MHz + 3500-3600 MHz)
```

**Advantages:**
- Simpler RF design
- Single PA (power amplifier)
- Lower cost

**Limitations:**
- Limited by available contiguous spectrum

#### 6.1.2 Intra-Band Non-Contiguous CA

**Configuration:**
```
Bands: Single band (e.g., n41)
Carriers: Non-adjacent frequency blocks (gaps between)
Maximum CCs: 16
Example: n41 (2.3-2.4 GHz + 2.5-2.6 GHz)
```

**Challenges:**
- Multiple PAs or wide-band PA
- Increased complexity
- Intermodulation products

#### 6.1.3 Inter-Band CA

**Configuration:**
```
Bands: Multiple different bands
Example: n77 (3.5 GHz) + n78 (3.7 GHz) + n257 (28 GHz)
Maximum CCs: 16
```

**CA Combinations:**

Common combinations:
```
FR1 + FR1:
- n1 + n3 (2.1 GHz + 1.8 GHz)
- n71 + n41 (600 MHz + 2.5 GHz)
- n77 + n78 (3.5 GHz + 3.7 GHz)

FR1 + FR2:
- n77 + n257 (3.5 GHz + 28 GHz)
- n78 + n260 (3.7 GHz + 39 GHz)

FR2 + FR2:
- n257 + n260 (28 GHz + 39 GHz)
```

### 6.2 Dual Connectivity (EN-DC)

**LTE + 5G NR:**
```
Master Node: LTE eNB (anchor)
Secondary Node: 5G gNB
Control Plane: LTE (initial)
User Plane: LTE + NR (aggregated)
```

**Throughput Calculation:**
```
Peak Throughput = T_LTE + T_NR

Example:
- LTE (20 MHz, 256-QAM, 4×4 MIMO): ~400 Mbps
- NR (100 MHz, 256-QAM, 4×4 MIMO): ~2500 Mbps
- Total: ~2900 Mbps
```

### 6.3 Bandwidth Parts (BWP)

**Concept:**
```
Purpose: Flexible bandwidth allocation to UEs
Maximum BWPs: 4 per carrier
Active BWPs: 1 at a time
Switching: Fast (symbols to slots)
```

**Use Cases:**
1. **Power Saving**: Narrow BWP for idle/low activity
2. **Capacity**: Wide BWP for high throughput
3. **Latency**: Narrow BWP for URLLC
4. **Coverage**: Low SCS (15 kHz) for cell edge

**BWP Configuration:**
```json
{
  "bwpId": 1,
  "locationAndBandwidth": 13750,
  "subcarrierSpacing": 30,
  "cyclicPrefix": "normal",
  "bandwidth": 100,
  "startPRB": 0,
  "numPRBs": 273
}
```

---

## 7. Spectrum Efficiency

### 7.1 Modulation and Coding

#### 7.1.1 Modulation Schemes

**5G NR Modulations:**
```
QPSK:     2 bits/symbol
16-QAM:   4 bits/symbol
64-QAM:   6 bits/symbol
256-QAM:  8 bits/symbol (downlink)
1024-QAM: 10 bits/symbol (future, high SNR)
```

**SINR Requirements:**
```
Modulation | SINR (dB) | Spectral Efficiency
-----------|-----------|--------------------
QPSK       | 0-10      | 0.15-1.2 bps/Hz
16-QAM     | 10-15     | 1.2-2.4 bps/Hz
64-QAM     | 15-20     | 2.4-4.8 bps/Hz
256-QAM    | 20-25     | 4.8-7.4 bps/Hz
1024-QAM   | >25       | 7.4-9.0 bps/Hz
```

#### 7.1.2 Channel Coding

**LDPC (Low-Density Parity-Check):**
```
Data Channels: PDSCH, PUSCH
Code Rates: 1/5, 1/3, 2/5, 1/2, 2/3, 3/4, 5/6
Block Size: 8-8448 bits
Base Graphs: BG1 (large blocks), BG2 (small blocks)
```

**Polar Codes:**
```
Control Channels: PBCH, PDCCH, PUCCH
Code Rates: 1/8 to 7/8
Block Size: Up to 512 bits (control), 164 bits (PBCH)
Advantage: Near Shannon limit for small blocks
```

### 7.2 Resource Allocation

#### 7.2.1 Physical Resource Blocks (PRBs)

**PRB Definition:**
```
Frequency: 12 subcarriers
Time: 1 slot (14 OFDM symbols, normal CP)
Size: 12 × 15 kHz = 180 kHz (SCS=15 kHz)
```

**PRBs per Bandwidth:**
```
SCS = 15 kHz:
- 20 MHz: 106 PRBs
- 100 MHz: 273 PRBs

SCS = 30 kHz:
- 100 MHz: 135 PRBs
- 400 MHz: 264 PRBs (FR2)
```

#### 7.2.2 Resource Efficiency

**Overhead:**
```
Components:
- Control channels (PDCCH, PUCCH): 5-10%
- Reference signals (DMRS, CSI-RS, SRS): 5-15%
- Guard bands: 5-10%
- Synchronization (SSB): 1-2%

Total Overhead: ~20-30%
Usable Resources: 70-80%
```

**Throughput Formula:**
```
Throughput = BW × (1 - Overhead) × Efficiency × MIMO_layers

Example (100 MHz, 256-QAM, 4×4 MIMO):
= 100e6 × 0.75 × 5.5 × 4
= 1650 Mbps
```

### 7.3 Spectral Efficiency Benchmarks

**Peak Spectral Efficiency (3GPP):**
```
5G NR Downlink:
- FR1: 30 bps/Hz (256-QAM, 8 layers, ideal)
- FR2: 30 bps/Hz (64-QAM, 8 layers, beamforming)

5G NR Uplink:
- FR1: 15 bps/Hz (256-QAM, 4 layers)
- FR2: 15 bps/Hz (64-QAM, 4 layers)
```

**Typical Deployment:**
```
Urban Macro (FR1):
- Average: 3-5 bps/Hz
- Peak: 8-10 bps/Hz

Dense Urban mmWave (FR2):
- Average: 5-10 bps/Hz
- Peak: 15-20 bps/Hz
```

---

## 8. Interference Management

### 8.1 Co-Channel Interference

**Sources:**
- Same-frequency cells
- Adjacent cell edge users
- Uncoordinated spectrum sharing

**Mitigation Techniques:**

**1. Inter-Cell Interference Coordination (ICIC):**
```
Static ICIC:
- Frequency reuse patterns (1×3, 1×1)
- Power control per frequency block

Dynamic ICIC:
- Load-based resource allocation
- Cell-edge user protection
```

**2. Enhanced ICIC (eICIC):**
```
Features:
- Almost Blank Subframes (ABS)
- Reduced power subframes
- Victim UE protection in HetNets

ABS Pattern Example: "10001000" (25% ABS)
```

**3. Further Enhanced ICIC (FeICIC):**
```
Advanced Features:
- Interference cancellation at UE
- Network assistance signaling
- Per-subframe power reduction
```

### 8.2 Adjacent Channel Interference

**Measurement:**
```
Adjacent Channel Interference Ratio (ACIR):
ACIR = 1 / (1/ACLR + 1/ACS)

Where:
- ACLR: Adjacent Channel Leakage Ratio (transmitter)
- ACS: Adjacent Channel Selectivity (receiver)
```

**Protection Mechanisms:**
```
Guard Bands:
- LTE to 5G: 5-10 MHz
- 5G to 5G: 0 MHz (tight filtering)

Filtering:
- Base Station: 45 dB ACLR, 50 dB ACS
- UE: 30 dB ACLR, 33 dB ACS
```

### 8.3 Interference Modeling

**Path Loss Models:**

**Urban Macro (UMa):**
```
PL = 28.0 + 22log10(d) + 20log10(f_c)  [dB]

Where:
- d: Distance (meters)
- f_c: Carrier frequency (GHz)

Valid: 10m ≤ d ≤ 5000m, 0.5 GHz ≤ f_c ≤ 100 GHz
```

**Urban Micro (UMi):**
```
LOS:
PL = 32.4 + 21log10(d) + 20log10(f_c)

NLOS:
PL = 35.3 + 26log10(d) + 20log10(f_c) + shadow_fading

Valid: 10m ≤ d ≤ 5000m
```

**Indoor Office:**
```
PL = 17.3 + 38.3log10(d) + 20log10(f_c/5)  [dB]

Valid: 1m ≤ d ≤ 150m, f_c ≤ 100 GHz
```

**Interference Calculation:**
```
Interference Power (I) = P_tx + G_tx - PL + G_rx - Penetration_Loss

Protection Criteria:
I/N < -6 dB (co-channel)
I/N < -10 dB (adjacent channel)
```

---

## 9. Regulatory Framework

### 9.1 ITU Radio Regulations

**World Radiocommunication Conference (WRC):**

**WRC-15:**
- Identified 24.25-86 GHz for IMT (5G)
- Initial mmWave allocations

**WRC-19:**
- Additional spectrum: 24.25-27.5 GHz, 37-43.5 GHz, 66-71 GHz
- Coexistence studies with satellite and fixed services

**WRC-23:**
- Under discussion: 7-15 GHz for 6G
- Upper mid-band harmonization

**WRC-27 (Future):**
- THz spectrum framework
- 100-300 GHz allocations

### 9.2 National Regulations

#### 9.2.1 United States (FCC)

**Spectrum Bands:**
```
600 MHz (n71): Auction 1000, licensed
CBRS (3.5 GHz): Three-tier shared access
C-Band (3.7-3.98 GHz): Auction 107, licensed
24 GHz (n258): Auction 101
28 GHz (n257): Auction 101
37/39/47 GHz: Auction 103
```

**CBRS Rules (47 CFR Part 96):**
- PAL: 70 MHz (7 × 10 MHz), county-based, 3-year term
- GAA: Opportunistic, no license required
- SAS: Mandatory registration and coordination
- EIRP: 30 dBm (outdoor), 24 dBm (indoor)

#### 9.2.2 Europe (CEPT/ECC)

**Harmonized Bands:**
```
700 MHz (n28): 2×30 MHz FDD
3.4-3.8 GHz (n78): 400 MHz TDD
26 GHz (n257): 1 GHz pioneer band
```

**Sharing Conditions:**
- Coordination with satellite services (3.4-3.8 GHz)
- Protection of fixed links (26 GHz)
- Cross-border coordination (SEAMCAT modeling)

#### 9.2.3 Asia Pacific

**China (MIIT):**
```
n41: 2515-2675 MHz (160 MHz TDD)
n77: 3300-3600 MHz (300 MHz TDD)
n79: 4800-5000 MHz (200 MHz TDD)
n258: 24.75-27.5 GHz, 37-42.5 GHz
```

**Korea (MSIT):**
```
n77: 3.4-3.7 GHz (300 MHz)
n78: 3.7-4.0 GHz (300 MHz)
n258: 26.5-28.9 GHz (2.4 GHz)
```

### 9.3 Licensing Models

**1. Exclusive Licensing:**
```
Method: Auction or beauty contest
Duration: 10-20 years
Renewal: Automatic or conditional
Example: C-Band auction (USA, $81 billion)
```

**2. Shared Licensing:**
```
Method: Registration + coordination
Duration: 3-10 years
Protection: Limited (vs. incumbents)
Example: CBRS PAL, LSA in Europe
```

**3. General Authorization:**
```
Method: Free access (registration may be required)
Duration: Unlimited
Protection: None
Example: CBRS GAA, U-NII bands
```

**4. Local Licensing:**
```
Method: Area-specific application
Duration: 5-10 years
Use: Private 5G, industrial IoT
Example: Germany (3.7-3.8 GHz), UK (3.8-4.2 GHz)
```

---

## 10. Implementation Guidelines

### 10.1 Network Planning

**Coverage Planning:**
```
Steps:
1. Define coverage objectives (area, capacity, QoS)
2. Select appropriate frequency band(s)
3. Estimate path loss and link budget
4. Determine cell radius and site density
5. Plan antenna configuration (height, tilt, azimuth)
6. Simulate network performance
7. Optimize and iterate
```

**Link Budget Template:**
```
Downlink Link Budget (FR1, 3.5 GHz):

Transmitter (gNB):
- Tx Power: 46 dBm
- Antenna Gain: 17 dBi
- EIRP: 63 dBm
- Cable Loss: -3 dB

Path Loss (1 km, UMa):
- Free Space: 106 dB
- Shadow Fading: 8 dB
- Building Penetration: 15 dB
- Total: 129 dB

Receiver (UE):
- Antenna Gain: 0 dBi
- Noise Figure: 7 dB
- Thermal Noise: -174 dBm/Hz
- Bandwidth: 100 MHz (80 dBHz)
- Noise Power: -87 dBm
- Sensitivity (QPSK): -96 dBm

Margin:
- Required SINR: 3 dB
- Interference Margin: 3 dB
- Fade Margin: 5 dB
- Total Margin: 11 dB

Link Budget:
EIRP - Path Loss + Rx Gain - Noise - SINR - Margin
= 63 - 129 + 0 - (-87) - 3 - 11
= 7 dB (Positive → Link Closes)
```

### 10.2 Spectrum Allocation Strategies

**For Mobile Operators:**
```
1. Acquire anchor low-band (600-900 MHz) for coverage
2. Secure mid-band (3.3-4.2 GHz) for capacity
3. Obtain mmWave (24-40 GHz) for hotspots
4. Leverage shared/unlicensed for supplemental capacity
```

**For Private Networks:**
```
1. Evaluate CBRS/local licensing options
2. Consider unlicensed (Wi-Fi 6E) for indoor
3. Apply for private spectrum if available
4. Plan neutral host or MVNO arrangements
```

### 10.3 Coexistence Best Practices

**Multi-Operator Coexistence:**
```
Techniques:
1. Synchronize TDD frame structure (GPS-based)
2. Coordinate guard band usage
3. Implement cross-link interference mitigation
4. Share site infrastructure (active/passive)
5. Use beamforming for spatial isolation
```

**5G-LTE Coexistence:**
```
Adjacent Channel:
- Use guard bands (5-10 MHz)
- Deploy bandpass filters (45 dB rejection)
- Limit adjacent channel power

Same Site:
- Separate antennas (vertical/horizontal)
- Use combiners with isolation (>30 dB)
- Optimize antenna tilts
```

---

## 11. References

### 11.1 3GPP Specifications

- **TS 38.101**: User Equipment (UE) radio transmission and reception
- **TS 38.104**: Base Station (BS) radio transmission and reception
- **TS 38.211**: Physical channels and modulation
- **TS 38.212**: Multiplexing and channel coding
- **TS 38.213**: Physical layer procedures for control
- **TS 38.214**: Physical layer procedures for data
- **TS 38.300**: Overall description; Stage-2

### 11.2 Regulatory Documents

- **FCC 47 CFR Part 96**: Citizens Broadband Radio Service
- **FCC 15-47**: Spectrum Frontiers Order (mmWave)
- **CEPT ECC Decision (18)06**: Harmonized use of 26 GHz
- **ITU-R M.2150**: Detailed specifications of IMT-2020

### 11.3 Industry Standards

- **CBRS Alliance**: CBRS specifications and certifications
- **WinnForum**: SAS specifications
- **GSMA**: Spectrum positions and policy guidance
- **IEEE 802.11**: Wi-Fi standards (802.11ax, 802.11be)

---

**弘益人間 (Benefit All Humanity)**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
