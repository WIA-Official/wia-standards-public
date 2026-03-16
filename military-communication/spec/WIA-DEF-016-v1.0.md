# WIA-DEF-016: Military Communication Specification v1.0

> **Standard ID:** WIA-DEF-016
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Defense Communications Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Tactical Radio Systems](#2-tactical-radio-systems)
3. [Satellite Communications](#3-satellite-communications)
4. [Tactical Data Links](#4-tactical-data-links)
5. [Network-Centric Warfare](#5-network-centric-warfare)
6. [Frequency Management](#6-frequency-management)
7. [Anti-Jamming Techniques](#7-anti-jamming-techniques)
8. [COMSEC Implementation](#8-comsec-implementation)
9. [Interoperability Standards](#9-interoperability-standards)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [Safety and Security Protocols](#11-safety-and-security-protocols)
12. [References](#12-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for military communication systems, encompassing tactical radios, satellite communications, secure data links, and network-centric warfare capabilities to ensure reliable and secure communications across all military operations.

### 1.2 Scope

The standard covers:
- Tactical radio system specifications (HF/VHF/UHF)
- Military satellite communications (MILSATCOM)
- Secure tactical data link protocols
- Network-centric warfare infrastructure
- Frequency management and spectrum allocation
- Electronic protection and anti-jamming
- Communications security (COMSEC)
- Interoperability requirements

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to provide robust and secure military communications that enhance operational effectiveness, protect forces, and enable mission success while promoting responsible spectrum use and coalition interoperability.

### 1.4 Terminology

- **COMSEC**: Communications Security - Protection of communications from exploitation
- **TRANSEC**: Transmission Security - Protection from interception and traffic analysis
- **ECCM**: Electronic Counter-Countermeasures - Protection from jamming
- **LPI/LPD**: Low Probability of Intercept/Detection - Covert communications
- **MANET**: Mobile Ad-hoc Network - Self-forming tactical network
- **C4ISR**: Command, Control, Communications, Computers, Intelligence, Surveillance, Reconnaissance

---

## 2. Tactical Radio Systems

### 2.1 HF Communications (2-30 MHz)

#### 2.1.1 High Frequency Radio Characteristics

HF communications provide long-range beyond-line-of-sight (BLOS) connectivity:

```
HF Propagation Modes:
- Ground Wave: 0-100 km (depending on terrain, frequency)
- Sky Wave (NVIS): 0-400 km (Near Vertical Incidence)
- Sky Wave (long-range): 500-4000+ km (ionospheric reflection)
```

#### 2.1.2 HF Link Budget

```
P_rx = P_tx + G_tx - L_path + G_rx - L_atm - L_noise
```

Where:
- `P_tx` = Transmit power (typically 20-400W, 13-26 dBW)
- `G_tx` = Transmit antenna gain (0-10 dBi)
- `L_path` = Path loss (varies with ionosphere, 140-180 dB)
- `G_rx` = Receive antenna gain (0-10 dBi)
- `L_atm` = Atmospheric absorption (1-5 dB)
- `L_noise` = Noise floor (-150 to -100 dBm)

#### 2.1.3 Automatic Link Establishment (ALE)

MIL-STD-188-141B ALE protocol:
- Automatic frequency selection
- Link quality analysis (LQA)
- Scanning and sounding
- Handshake protocols

### 2.2 VHF Communications (30-88 MHz)

#### 2.2.1 VHF Tactical Radio

Primary ground force communications:

```
VHF Line-of-Sight Range:
r = 4.12 × (√h_tx + √h_rx)
```

Where:
- `r` = Radio horizon (km)
- `h_tx` = Transmit antenna height (m)
- `h_rx` = Receive antenna height (m)

For typical tactical scenario (h_tx = h_rx = 2m):
```
r = 4.12 × (√2 + √2) = 11.6 km
```

#### 2.2.2 SINCGARS (Single Channel Ground and Airborne Radio System)

Key features:
- Frequency hopping (30-87.975 MHz)
- 25 kHz channel spacing
- Hop rate: 111 hops/second
- Anti-jamming capability
- Voice and data modes

Hopping pattern generation:
```
f_n = f_base + (PRNG(seed, n) mod 2320) × 25 kHz
```

Where:
- `f_n` = nth hop frequency
- `f_base` = Base frequency (30 MHz)
- `PRNG` = Pseudo-random number generator
- `seed` = Transmission security key
- `n` = Hop number

### 2.3 UHF Communications (225-512 MHz)

#### 2.3.1 UHF Tactical Systems

Applications:
- Air-to-air communications
- Air-to-ground coordination
- Satellite uplinks (UHF SATCOM)
- Line-of-sight tactical data

#### 2.3.2 Free-Space Path Loss

```
L_fs(dB) = 20 log₁₀(d) + 20 log₁₀(f) + 32.45
```

For UHF at 300 MHz, 50 km:
```
L_fs = 20 log₁₀(50) + 20 log₁₀(300) + 32.45
L_fs = 34.0 + 49.5 + 32.45 = 115.95 dB
```

### 2.4 Software Defined Radio (SDR)

#### 2.4.1 Joint Tactical Radio System (JTRS)

Software-defined waveforms:
- Wideband Networking Waveform (WNW)
- Soldier Radio Waveform (SRW)
- Mobile User Objective System (MUOS)
- Legacy waveform support

Architecture:
```
SDR Stack:
├── Applications Layer (voice, data, messaging)
├── Waveform Layer (SINCGARS, HAVEQUICK, Link 16)
├── MODEM Layer (modulation/demodulation)
├── Radio Frequency Layer (tuning, filtering, amplification)
└── Hardware Layer (FPGA, DSP, RF front-end)
```

---

## 3. Satellite Communications

### 3.1 Military SATCOM Architecture

#### 3.1.1 Geostationary Satellites (GEO)

Orbital parameters:
- Altitude: 35,786 km
- Period: 23h 56m 4s (sidereal day)
- Velocity: 3.07 km/s

Coverage:
```
Coverage angle (θ) from satellite:
θ = 2 × arcsin(R_earth / (R_earth + h_sat))
θ = 2 × arcsin(6371 / (6371 + 35786)) = 17.4°

Coverage area ≈ 42.4% of Earth surface per satellite
```

#### 3.1.2 Link Budget for SATCOM

Uplink budget:
```
P_rx_sat = P_tx_ground + G_tx_ground - L_fs_uplink + G_rx_sat - L_atm - L_rain
```

Downlink budget:
```
P_rx_ground = P_tx_sat + G_tx_sat - L_fs_downlink + G_rx_ground - L_atm - L_rain
```

For X-band SATCOM (8 GHz downlink, 36,000 km):
```
L_fs = 20 log₁₀(36000) + 20 log₁₀(8000) + 32.45
L_fs = 91.1 + 78.1 + 32.45 = 201.7 dB
```

#### 3.1.3 SATCOM Systems

**WGS (Wideband Global SATCOM)**
- Frequency: X-band (7.25-7.75 GHz down, 7.9-8.4 GHz up)
- Ka-band (20.2-21.2 GHz down, 30-31 GHz up)
- Bandwidth: 3.6 Gbps per satellite
- Coverage: Global with 6+ satellites

**AEHF (Advanced Extremely High Frequency)**
- Frequency: EHF (44 GHz up, 20 GHz down)
- SHF (7.55 GHz up, 8.35 GHz down)
- Data rate: 8.192 Mbps (EHF), 1.544 Mbps (SHF)
- Nuclear hardened, anti-jam

**MUOS (Mobile User Objective System)**
- Frequency: UHF (300-320 MHz up, 240-270 MHz down)
- Data rate: 384 kbps per channel
- Users: 60,000+ simultaneous
- Coverage: Global (90°N to 90°S)

### 3.2 SATCOM Terminal Types

| Terminal Type | Size | Data Rate | Mobility | Use Case |
|---------------|------|-----------|----------|----------|
| Strategic | >2.4 m dish | 1+ Gbps | Fixed | Theater C2 |
| Tactical | 0.6-1.2 m | 100 Mbps | Transportable | Brigade HQ |
| Manpack | <10 kg | 256 kbps | Portable | Special ops |
| Handheld | <2 kg | 9.6 kbps | Mobile | Individual soldier |

---

## 4. Tactical Data Links

### 4.1 Link 16 (JTIDS/MIDS)

#### 4.1.1 Link 16 Architecture

- Frequency: 960-1215 MHz (L-band)
- Modulation: CDMA with Reed-Solomon encoding
- Time slots: 12.8 minutes per frame, 7.8125 ms per slot
- Data rate: 238 kbps aggregate
- Range: 300+ nautical miles (air-air), 150 nmi (surface-air)

#### 4.1.2 Link 16 Message Format

```
J-Series Messages:
- J2.x: Indirect interface unit messages
- J3.x: Fighter-to-fighter messages
- J7.x: Weapons coordination messages
- J12.x: Control messages
- J14.x: Missile messages
```

Message structure:
```
Header (5 bits) | Message Type (3 bits) | Data Field (variable) | CRC (16 bits)
```

#### 4.1.3 Time Division Multiple Access (TDMA)

```
TDMA Frame Structure:
- Frame period: 12.8 minutes (768 seconds)
- Slots per frame: 98,304 (128 slots/second)
- Slot duration: 7.8125 ms
- Net entry: Assigned time slots per participant
```

### 4.2 Link 22 (NATO Improved Link 11)

- Frequency: 225-400 MHz (UHF)
- Data rate: 238 kbps
- Modulation: DSSS (Direct Sequence Spread Spectrum)
- Range: 300+ nmi
- Encryption: NATO classified

### 4.3 SADL (Situational Awareness Data Link)

Army aviation data link:
- Frequency: VHF/UHF
- Data rate: 57.6 kbps
- Range: 100+ km (air-ground), 200+ km (air-air)
- Users: AH-64, UH-60, ground C2

### 4.4 TTNT (Tactical Targeting Network Technology)

High-bandwidth mobile networking:
- Frequency: Ka/Ku-band
- Data rate: 10+ Mbps
- Range: 200+ km
- Waveform: OFDM with MIMO
- Applications: ISR dissemination, video streaming

---

## 5. Network-Centric Warfare

### 5.1 Common Operating Picture (COP)

#### 5.1.1 COP Architecture

```
COP Data Layers:
├── Friendly Forces (Blue Force Tracking)
├── Enemy Forces (Red Force Tracking)
├── Terrain and Weather
├── Critical Infrastructure
├── Airspace Control
└── Fires Coordination
```

#### 5.1.2 Blue Force Tracking

Position update:
```
Position Message:
- Unit ID: 32 bits
- Latitude: 32 bits (decimal degrees × 10⁷)
- Longitude: 32 bits (decimal degrees × 10⁷)
- Altitude: 16 bits (meters)
- Heading: 16 bits (degrees × 100)
- Speed: 16 bits (m/s × 100)
- Timestamp: 32 bits (Unix epoch)
- Status: 8 bits (flags)
Total: 184 bits = 23 bytes per update
```

Update frequency:
- Stationary: 5 minutes
- Moving: 30 seconds
- Combat: 5 seconds

### 5.2 Mobile Ad-Hoc Networks (MANET)

#### 5.2.1 MANET Routing

Ad-hoc On-Demand Distance Vector (AODV):
```
Route Discovery:
1. Source broadcasts Route Request (RREQ)
2. Intermediate nodes forward RREQ (if not duplicate)
3. Destination sends Route Reply (RREP) via reverse path
4. Source receives RREP and begins data transmission
```

Routing metric:
```
Metric = α × Hops + β × Delay + γ × (1/Bandwidth) + δ × Loss
```

Where α, β, γ, δ are weighting factors.

#### 5.2.2 Quality of Service (QoS)

Priority levels:
1. Command and Control (highest)
2. Intelligence/ISR
3. Fires coordination
4. Logistics
5. Administrative (lowest)

---

## 6. Frequency Management

### 6.1 Military Frequency Allocation

#### 6.1.1 ITU Frequency Bands

| Band | Frequency | Wavelength | Military Use |
|------|-----------|------------|--------------|
| HF | 3-30 MHz | 10-100 m | Long-range BLOS |
| VHF | 30-300 MHz | 1-10 m | Tactical ground |
| UHF | 300-3000 MHz | 0.1-1 m | Air-ground, SATCOM |
| L | 1-2 GHz | 15-30 cm | GPS, IFF |
| S | 2-4 GHz | 7.5-15 cm | Radar, SATCOM |
| X | 8-12 GHz | 2.5-3.75 cm | SATCOM, radar |
| Ku | 12-18 GHz | 1.67-2.5 cm | SATCOM |
| Ka | 26-40 GHz | 0.75-1.15 cm | High-bandwidth SATCOM |
| EHF | 30-300 GHz | 1-10 mm | Secure strategic |

#### 6.1.2 Frequency Assignment Process

```
Frequency Assignment Workflow:
1. Request → Unit submits frequency request (DD Form 1494)
2. Coordination → Spectrum manager checks conflicts
3. Allocation → Assign frequency from available pool
4. Authorization → Issue frequency assignment
5. Monitoring → Track usage and interference
6. Renewal → Periodic review and revalidation
```

### 6.2 Dynamic Spectrum Access

Cognitive radio approach:
```
Spectrum Sensing:
1. Scan frequency range
2. Detect occupied channels (energy detection)
3. Classify signals (feature detection)
4. Select best available channel
5. Transmit while monitoring
6. Vacate if primary user detected
```

Channel quality metric:
```
Q = SNR × (1 - PER) × BW / Interference
```

Where:
- `SNR` = Signal-to-noise ratio
- `PER` = Packet error rate
- `BW` = Available bandwidth
- `Interference` = Measured interference level

---

## 7. Anti-Jamming Techniques

### 7.1 Frequency Hopping Spread Spectrum (FHSS)

#### 7.1.1 FHSS Parameters

```
Processing Gain (G_p):
G_p = 10 log₁₀(B_spread / B_data)
```

For SINCGARS (2.32 MHz spread, 16 kbps data):
```
G_p = 10 log₁₀(2320000 / 16000) = 21.6 dB
```

Jamming margin:
```
J/S = G_p - L_implementation - M_required
```

Where:
- `J/S` = Jammer-to-signal ratio tolerance
- `L_implementation` = Implementation losses (3-6 dB)
- `M_required` = Required margin (10 dB)

#### 7.1.2 Hop Set Design

Criteria:
- Minimum hop separation: >25 kHz
- Avoid harmonic relationships
- Uniform distribution across band
- Collision avoidance in multi-net environment

### 7.2 Direct Sequence Spread Spectrum (DSSS)

#### 7.2.1 DSSS Processing Gain

```
G_p = 10 log₁₀(R_chip / R_bit)
```

For GPS (1.023 Mcps, 50 bps):
```
G_p = 10 log₁₀(1023000 / 50) = 43.1 dB
```

#### 7.2.2 PN Code Properties

Gold codes for GPS and Link 16:
- Length: 2^n - 1 (1023, 32767)
- Auto-correlation: Sharp peak at zero offset
- Cross-correlation: Low between different codes
- Balance: Equal number of 1s and 0s

### 7.3 Adaptive Nulling

#### 7.3.1 Antenna Array Processing

Weights calculation:
```
w_optimal = R⁻¹ × s
```

Where:
- `w` = Array weight vector
- `R` = Covariance matrix of received signal
- `s` = Steering vector for desired signal

Null depth:
```
Null depth (dB) = 20 log₁₀(|w^H a_jammer|)
```

Typically achieves 30-50 dB null depth per jammer.

### 7.4 Low Probability of Intercept (LPI)

#### 7.4.1 LPI Waveforms

Characteristics:
- Spread spectrum (FHSS, DSSS)
- Low transmit power
- Directional antennas
- Burst transmission

Detection probability by non-cooperative receiver:
```
P_detect = Q(√(2 × SNR × B × T))
```

Where:
- `Q` = Q-function (complementary error function)
- `SNR` = Signal-to-noise ratio at interceptor
- `B` = Bandwidth
- `T` = Integration time

---

## 8. COMSEC Implementation

### 8.1 Encryption Standards

#### 8.1.1 Symmetric Encryption

**AES-256 (Advanced Encryption Standard)**
- Key size: 256 bits
- Block size: 128 bits
- Rounds: 14
- Security: Top Secret and below

Encryption:
```
C = E_k(P)
P = D_k(C)
```

Where:
- `C` = Ciphertext
- `P` = Plaintext
- `E_k` = Encryption function with key k
- `D_k` = Decryption function with key k

#### 8.1.2 Key Management

EKMS (Electronic Key Management System):
- Key generation: FIPS 140-2 compliant TRNG
- Key distribution: Over-the-air (OTAR), physical (COMSEC material)
- Key storage: Encrypted, tamper-resistant memory
- Key lifecycle: Generate → Distribute → Store → Use → Archive → Destroy

Key effective period:
- Strategic keys: 30 days
- Tactical keys: 24 hours
- Session keys: Per communication session

#### 8.1.3 Authentication

Message authentication:
```
MAC = HMAC-SHA512(K_auth, Message)
```

Digital signature:
```
Signature = Sign(K_private, Hash(Message))
Verify = Verify(K_public, Hash(Message), Signature)
```

### 8.2 Transmission Security (TRANSEC)

#### 8.2.1 Traffic Flow Security

Techniques:
- Constant transmission rate (traffic padding)
- Dummy message insertion
- Message length padding
- Timing randomization

Traffic analysis resistance:
```
TA_resistance = H(Traffic_pattern)
```

Where H is Shannon entropy.

#### 8.2.2 Emission Security (EMSEC)

Tempest standards:
- Limit emanations: <1 nW/cm² at 1 meter
- Shielding: >100 dB attenuation
- Filtering: Power line, signal line
- Zoning: Controlled areas for classified processing

---

## 9. Interoperability Standards

### 9.1 NATO STANAG Standards

Key STANAGs for communications:
- STANAG 4285: HF data modem
- STANAG 4415: Link 16 interfaces
- STANAG 4586: UAV control data link
- STANAG 5066: HF subnet protocol
- STANAG 7085: Narrowband VHF networking

### 9.2 Coalition Interoperability

#### 9.2.1 Gateway Architecture

```
Coalition Gateway:
├── National Network (Classified)
├── Gateway Translation Layer
│   ├── Security domain separation
│   ├── Protocol translation
│   └── Data format conversion
└── Coalition Network (Shared)
```

#### 9.2.2 Combined Enterprise Regional Information Exchange System (CENTRIXS)

Levels:
- SECRET releasable to coalition
- Coalition SECRET
- Unclassified coalition

Security requirements:
- Multilevel security (MLS)
- Release authority enforcement
- Audit logging
- Cross-domain guards

### 9.3 Software Communications Architecture (SCA)

#### 9.3.1 SCA Compliance

JTRS SCA v4.1:
- Component-based architecture
- CORBA middleware
- Waveform portability
- Hardware abstraction

Waveform deployment:
```
Waveform Package:
├── Waveform XML descriptor
├── Component assemblies (.so, .dll)
├── Properties configuration
└── Installation scripts
```

---

## 10. Implementation Guidelines

### 10.1 Required Components

Any WIA-DEF-016 compliant system must include:

1. **Radio Interface**: Hardware abstraction for radios
2. **Link Budget Calculator**: RF propagation modeling
3. **Encryption Engine**: AES-256, RSA-4096 support
4. **Frequency Manager**: Spectrum allocation and coordination
5. **Anti-Jam Controller**: ECCM techniques implementation
6. **Interoperability Gateway**: Protocol translation
7. **Network Manager**: MANET routing, QoS

### 10.2 API Interface

#### 10.2.1 Link Budget Calculation

```typescript
interface LinkBudgetRequest {
  transmitPower: number;      // Watts
  frequency: number;          // MHz
  distance: number;           // meters
  txAntennaGain?: number;     // dBi
  rxAntennaGain?: number;     // dBi
  txHeight?: number;          // meters
  rxHeight?: number;          // meters
  terrain?: 'open' | 'urban' | 'forest' | 'mountainous';
}

interface LinkBudgetResponse {
  receivedPower: number;      // dBm
  pathLoss: number;           // dB
  linkMargin: number;         // dB
  maxDistance: number;        // km
  feasibility: 'excellent' | 'good' | 'marginal' | 'poor';
}
```

#### 10.2.2 Message Encryption

```typescript
interface EncryptionRequest {
  message: string;
  encryptionLevel: 'unclass' | 'secret' | 'top-secret';
  algorithm: 'aes-256-gcm' | 'aes-256-cbc';
  keyId: string;
}

interface EncryptionResponse {
  ciphertext: string;         // Base64 encoded
  iv: string;                 // Initialization vector
  authTag: string;            // Authentication tag (GCM mode)
  timestamp: Date;
  expiration: Date;
}
```

#### 10.2.3 Frequency Validation

```typescript
interface FrequencyValidation {
  frequency: number;          // MHz
  bandwidth: number;          // kHz
  location: {
    latitude: number;
    longitude: number;
  };
  purpose: 'voice' | 'data' | 'video';
}

interface FrequencyResponse {
  isValid: boolean;
  conflicts: Conflict[];
  recommendation: string;
  alternativeFrequencies?: number[];
}
```

### 10.3 Data Formats

#### 10.3.1 Position Report (NATO APP-11)

```json
{
  "messageType": "POSREP",
  "unitId": "ALPHA-6",
  "position": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude": 15,
    "datum": "WGS84"
  },
  "heading": 270,
  "speed": 15,
  "timestamp": "2025-12-27T12:00:00Z",
  "classification": "SECRET"
}
```

#### 10.3.2 Contact Report

```json
{
  "messageType": "CONREP",
  "reportingUnit": "BRAVO-3",
  "contactType": "enemy_infantry",
  "location": {
    "grid": "38SMB4484",
    "latitude": 37.5,
    "longitude": -122.0
  },
  "strength": "squad_size",
  "activity": "moving_north",
  "timestamp": "2025-12-27T12:05:00Z",
  "confidence": "confirmed"
}
```

---

## 11. Safety and Security Protocols

### 11.1 Pre-Transmission Checklist

- [ ] Frequency authorized and coordinated
- [ ] Encryption keys loaded and valid
- [ ] Authentication configured
- [ ] Emission control (EMCON) status verified
- [ ] Antenna properly oriented
- [ ] Power output set correctly
- [ ] Anti-jam mode enabled (if required)
- [ ] Call signs and codes current

### 11.2 OPSEC Procedures

1. **Message Minimization**: Transmit only necessary information
2. **Code Words**: Use approved brevity codes
3. **Transmission Discipline**: Minimize transmit time
4. **Location Security**: Avoid revealing positions
5. **Pattern Avoidance**: Randomize transmission times
6. **Equipment Security**: Prevent unauthorized access

### 11.3 Emergency Procedures

#### 11.3.1 Communications Failure

Priority restoration order:
1. Command and control links
2. Fires coordination
3. ISR dissemination
4. Logistics coordination
5. Administrative traffic

Alternative means:
- Switch to backup frequency
- Use alternate data link
- Employ messenger/runner
- Visual signals (prearranged)

#### 11.3.2 Compromise Procedures

If encryption compromised:
1. Immediately cease transmissions on affected net
2. Initiate emergency zeroization
3. Load backup keys (if available)
4. Report compromise through secure channel
5. Await new key material
6. Resume operations with new keys

### 11.4 Monitoring Requirements

Continuous monitoring:
- Frequency usage and conflicts
- Link quality and bit error rate
- Jamming and interference
- Unauthorized transmissions
- Equipment status and alerts
- Key expiration warnings

---

## 12. References

### 12.1 Military Standards

1. MIL-STD-188-141B: HF Radio Automatic Link Establishment
2. MIL-STD-188-220: Interoperability Standard for Digital Message Transfer Device
3. MIL-STD-6016: Tactical Data Link 16
4. MIL-STD-3011: Link 22
5. MIL-STD-6017: Link 11/11B

### 12.2 NATO Standards

1. STANAG 4285: HF Data Modem
2. STANAG 4415: Link 16 Interfaces
3. STANAG 5066: HF Radio Subnet Protocol
4. STANAG 4586: UAV Control Data Link
5. APP-11: NATO Message Formats

### 12.3 Technical References

| Standard | Title | Application |
|----------|-------|-------------|
| FIPS 140-2 | Cryptographic Module Validation | Encryption |
| IEEE 802.11 | Wireless LAN | Tactical WiFi |
| IEEE 802.16 | WiMAX | Wide-area networking |
| IETF RFC 3561 | AODV Routing | MANET routing |

### 12.4 WIA Standards

- WIA-INTENT: Intent-based radio control
- WIA-OMNI-API: Universal communications API
- WIA-QUANTUM: Quantum encryption
- WIA-SOCIAL: Coalition coordination
- WIA-DEF-010: Military satellite systems

---

## Appendix A: Example Calculations

### A.1 VHF Tactical Radio Range

```
Given:
- Transmit power: 50W (17 dBW)
- Frequency: 150 MHz
- Antenna height (both): 2 meters
- Antenna gain: 3 dBi (both)
- Required SNR: 10 dB

Calculation:
1. Radio horizon: r = 4.12 × (√2 + √2) = 11.6 km

2. Free-space path loss at 11.6 km:
   L_fs = 20 log₁₀(11.6) + 20 log₁₀(150) + 32.45
   L_fs = 21.3 + 43.5 + 32.45 = 97.3 dB

3. Received power:
   P_rx = P_tx + G_tx - L_fs + G_rx
   P_rx = 17 + 3 - 97.3 + 3 = -74.3 dBm

4. Noise floor (10 kHz bandwidth, NF=8dB):
   N = -174 + 10 log₁₀(10000) + 8 = -126 dBm

5. SNR = P_rx - N = -74.3 - (-126) = 51.7 dB

Conclusion: Excellent link margin of 41.7 dB (51.7 - 10)
```

### A.2 SATCOM Link Budget

```
Given:
- Satellite: WGS X-band downlink
- Frequency: 8 GHz
- Distance: 36,000 km
- Satellite EIRP: 50 dBW
- Ground antenna gain: 35 dBi (1.2m dish)

Calculation:
1. Free-space path loss:
   L_fs = 20 log₁₀(36000) + 20 log₁₀(8000) + 32.45
   L_fs = 91.1 + 78.1 + 32.45 = 201.7 dB

2. Atmospheric loss: 2 dB
   Rain loss: 3 dB (moderate rain)

3. Received power:
   P_rx = EIRP - L_fs - L_atm - L_rain + G_rx
   P_rx = 50 - 201.7 - 2 - 3 + 35 = -121.7 dBm

4. Noise temperature (clear sky): 50K
   System noise temp: 150K
   Bandwidth: 5 MHz

5. Noise power:
   N = 10 log₁₀(k T B)
   N = 10 log₁₀(1.38e-23 × 150 × 5e6)
   N = -128.6 dBm

6. C/N = P_rx - N = -121.7 - (-128.6) = 6.9 dB

For 16-QAM requiring 10 dB: Link marginal in rain
Recommendation: Reduce data rate or increase antenna size
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-DEF-016 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
