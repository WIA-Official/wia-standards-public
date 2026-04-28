# WIA-COMM-008 — Phase 3: Protocol

> Wireless-charging canonical Phase 3: protocols (inductive + resonance + Qi + AirFuel + J2954 + FOD-shutdown).

# WIA-COMM-008: Wireless Charging Specification v1.0

> **Standard ID:** WIA-COMM-008
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Communication Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Inductive Coupling](#2-inductive-coupling)
3. [Magnetic Resonance](#3-magnetic-resonance)
4. [Qi Standard (WPC)](#4-qi-standard-wpc)
5. [AirFuel Alliance](#5-airfuel-alliance)
6. [Power Transfer Efficiency](#6-power-transfer-efficiency)
7. [Coil Design](#7-coil-design)
8. [Foreign Object Detection (FOD)](#8-foreign-object-detection-fod)
9. [Alignment and Positioning](#9-alignment-and-positioning)
10. [Thermal Management](#10-thermal-management)
11. [EMF Safety](#11-emf-safety)
12. [EV Wireless Charging](#12-ev-wireless-charging)
13. [Multi-Device Charging](#13-multi-device-charging)
14. [Interoperability Testing](#14-interoperability-testing)
15. [Implementation Guidelines](#15-implementation-guidelines)
16. [References](#16-references)

---


## 6. Power Transfer Efficiency

### 6.1 Efficiency Calculation

```
η = (P_out / P_in) × 100%
```

Where:
- `η` = Efficiency (percentage)
- `P_out` = Output power at receiver (watts)
- `P_in` = Input power at transmitter (watts)

### 6.2 Loss Mechanisms

#### Coil Resistance Losses
```
P_loss_coil = I² × R
```

#### Core Losses
```
P_loss_core = k_h × f × B² + k_e × f² × B²
```

Where:
- `k_h` = Hysteresis loss coefficient
- `k_e` = Eddy current loss coefficient
- `f` = Frequency (Hz)
- `B` = Magnetic flux density (tesla)

#### Rectification Losses
```
P_loss_rect = V_f × I + I² × R_on
```

Where:
- `V_f` = Forward voltage drop (volts)
- `R_on` = On-resistance (ohms)

### 6.3 Typical Efficiencies

| Power Level | Technology | Efficiency | Distance |
|-------------|------------|------------|----------|
| 5W | Qi BPP | 70-75% | 3-5 mm |
| 15W | Qi EPP | 75-80% | 3-8 mm |
| 50W | AirFuel | 65-75% | 10-50 mm |
| 11kW | SAE J2954 | 85-90% | 100-200 mm |

### 6.4 Efficiency Optimization

1. **Impedance matching**: Match coil impedance to load
2. **Resonance tuning**: Operate at resonant frequency
3. **Coil alignment**: Minimize lateral/angular misalignment
4. **High-Q coils**: Use litz wire and ferrite shielding
5. **Active rectification**: Replace diodes with synchronous FETs

---



## 8. Foreign Object Detection (FOD)

### 8.1 Purpose

Foreign objects (coins, keys, metal) can:
- Heat up dangerously (>100°C)
- Reduce efficiency
- Damage charger or receiver
- Create safety hazards

### 8.2 Detection Methods

#### 8.2.1 Q-Factor Measurement

Metal objects reduce coil Q factor:

```
ΔQ = (Q_baseline - Q_measured) / Q_baseline
```

**Threshold**: ΔQ > 10% indicates foreign object

#### 8.2.2 Power Loss Method

Compare expected vs. actual power:

```
P_loss = P_in - P_out - P_expected_loss
```

**Threshold**: P_loss > 1W indicates foreign object

#### 8.2.3 Frequency Detuning

Metal objects shift resonant frequency:

```
Δf = f_baseline - f_measured
```

**Threshold**: Δf > 2% indicates foreign object

### 8.3 Qi FOD Implementation

1. **Baseline calibration**: Measure Q without objects
2. **Continuous monitoring**: Check Q every 1 second
3. **Power cutoff**: Stop charging if FOD detected
4. **User notification**: LED, beep, or app alert

---



## 9. Alignment and Positioning

### 9.1 Alignment Importance

Misalignment reduces:
- **Coupling coefficient**: k drops with lateral offset
- **Efficiency**: η decreases quadratically
- **Power delivery**: May not meet device requirements

### 9.2 Lateral Misalignment

```
k(Δx) ≈ k₀ × exp(-α × Δx²)
```

Where:
- `Δx` = Lateral offset (meters)
- `k₀` = Coupling at perfect alignment
- `α` = Decay constant (depends on coil geometry)

### 9.3 Alignment Feedback

#### Visual
- **LED ring**: Indicates alignment quality (red/yellow/green)
- **On-screen**: App displays alignment score

#### Auditory
- **Beep frequency**: Faster beeps = better alignment
- **Tone pitch**: Higher pitch = better alignment

#### Haptic
- **Vibration pattern**: Stronger vibration = better alignment
- **Pulse rate**: Faster pulses = better alignment

### 9.4 Guided Positioning

Advanced systems use:
- **Magnets**: Passive mechanical alignment
- **Motors**: Active coil positioning
- **Cameras**: Computer vision for alignment
- **Multiple coils**: Adaptive coil selection

---



## 10. Thermal Management

### 10.1 Heat Generation

Heat sources:
- **Coil losses**: I²R heating in windings
- **Core losses**: Hysteresis and eddy currents
- **Rectifier losses**: Diode or FET dissipation
- **Foreign objects**: Induced current heating

### 10.2 Temperature Monitoring

Use NTC thermistors:
- **Location**: Embedded in transmitter coil
- **Sampling rate**: 1-10 Hz
- **Thresholds**: Warning (70°C), cutoff (80°C)

```
T = β / ln(R/R₀) - 273.15
```

Where:
- `T` = Temperature (°C)
- `β` = NTC beta coefficient (K)
- `R` = Measured resistance (Ω)
- `R₀` = Resistance at 25°C (Ω)

### 10.3 Thermal Management Strategies

1. **Dynamic power scaling**: Reduce power if temperature rises
2. **Duty cycle control**: Pulse charging with cooling intervals
3. **Forced air cooling**: Fan for high-power chargers
4. **Heatsink**: Aluminum or copper heat spreader
5. **Thermal pads**: Interface material to chassis

### 10.4 Temperature Limits

| Component | Warning | Cutoff | Material |
|-----------|---------|--------|----------|
| Transmitter coil | 70°C | 80°C | Copper/ferrite |
| Receiver coil | 70°C | 80°C | Copper/ferrite |
| Rectifier | 100°C | 125°C | Silicon (diode/FET) |
| Battery | 40°C | 45°C | Lithium-ion |
| Foreign object | N/A | 100°C | Metal |

---




---

## A.1 Inductive-coupling protocol

Inductive-coupling protocol covers the magnetic-induction power transfer between Tx and Rx coils with the operator's documented operating-frequency, the selection logic between fixed-frequency operation (Qi v1.x, J2954) and variable-frequency operation (some proprietary chargers for receiver-power-tracking), the duty-cycle envelope, and the in-band ASK communication for Qi (Rx → Tx amplitude-shift-keyed packets at 2 kbps for control, error, and end-power-transfer messages). The protocol enforces the maximum-coupling envelope so the receiver does not present a low-impedance load that overheats the Tx coil.

## A.2 Magnetic-resonance protocol

Magnetic-resonance protocol covers the high-Q resonant link at 6.78 MHz (AirFuel) or proprietary frequencies, the BLE side-channel for charger-receiver discovery and control, the impedance-tracking control loop on the Tx side, and the resonator-detuning envelope under thermal drift. Multi-receiver resonant charging requires the protocol's coordination envelope to allocate per-receiver power without exceeding the Tx coil's thermal budget; receivers report their accepted-power and the Tx adjusts the duty-cycle and impedance match.

## A.3 Qi WPC protocol

The Qi protocol per WPC v1.3 / v2.0 (with QiPlus extensions) covers the receiver-detection ping (analog ping + digital ping), the receiver identification (per the Qi v1.3 packet-of-record), the configuration-and-power-transfer phase, the FOD calibration envelope (per Qi v1.3 calibration sequence), and the renegotiation envelope when the receiver requests a power increase (e.g., 5W → 9W → 15W upgrade per v1.2.4 BPP/EPP). End-power-transfer (EPT) packets close the session cleanly with the documented end-reason codes.

## A.4 AirFuel resonant protocol

AirFuel resonant protocol covers the BLE-based device discovery (legacy A4WP profile), the resonator activation handshake, the charge-control-state-machine (sense / charge / regulate / fault), the multi-device round-robin scheduling envelope, and the inter-receiver-coupling-cancellation control where multiple receivers are charging in the same field. The protocol's separation between BLE control and 6.78 MHz power transfer simplifies multi-receiver coordination at the cost of additional BLE-stack latency.

## A.5 J2954 EV-WPT protocol

SAE J2954 protocol covers the EV wireless-power-transfer use case at 79-90 kHz with WPT 1 (3.7 kW) / WPT 2 (7.7 kW) / WPT 3 (11.1 kW) classes, the alignment categories (Z1 small / Z2 medium / Z3 large vehicle ground clearance), the in-band sideband communication per SAE J2847/6 (LIN/CAN-mapped messages over the WPT control channel), the IEC 61980 series cross-reference for harmonised EU deployment, and the dynamic / opportunistic charging envelope for the experimental dynamic-WPT proposals on instrumented road segments.

## A.6 FOD and safety-shutdown protocol

The FOD-and-safety-shutdown protocol covers the periodic FOD re-test cadence (typical every 200-500 ms during active charging), the thermal-threshold envelope (typical 70-80 C maximum coil temperature per Qi v1.3; J2954 specifies its own thermal envelope per WPT class), the alignment-loss envelope (sub-second response when alignment exceeds policy), and the controlled shutdown sequence (graceful power-down → BLE / in-band notification → session-state transition → audit log emission). The protocol's safety state takes priority over performance: an uncertain FOD reading triggers conservative power reduction or shutdown, never aggressive continuation.

## A.7 Coil-design and Q-factor measurement protocol

Coil-design protocol covers the per-Tx-coil parametric envelope (turn count, conductor selection — typical Litz wire AWG 38 / 42 / 46 with strand counts in the 100s for skin-effect minimisation, ferrite shielding geometry, encapsulation envelope). Q-factor measurement protocol covers the per-coil resonant Q measurement (typical 300-1500 for high-Q resonant; 100-300 for inductive Qi at the operating frequency), the per-deployment Q-factor calibration envelope (the Tx records its baseline Q in the absence of a receiver and uses the differential Q vs baseline as a FOD signal per Qi v1.3 calibration sequence), and the temperature-compensated Q envelope to defeat false-positive FOD events caused by ambient-temperature drift.

## A.8 Multi-protocol charger interoperability protocol

Dual-mode chargers that advertise both Qi and AirFuel support carry the per-protocol negotiation envelope: at receiver-detection the charger performs both an analog-ping per Qi and a BLE-discovery per AirFuel; the charger selects the protocol of the responder and continues per that protocol's state machine. The interoperability protocol bans simultaneous activation of both protocols on the same Tx coil (avoiding cross-protocol RF interference); for multi-coil arrays the charger can run different protocols on different coils with the documented inter-coil isolation envelope.

## A.9 Replay and integrity defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for the management-plane control traffic. Charger-record updates and session-management API calls are signed at registration time; the signature chain is anchored in the per-tenant Merkle tree per the operator's policy. Telemetry traffic uses mTLS with per-channel monotonic counters; replay attempts are detected and dropped at the broker. BLE side-channel control links use the Bluetooth Secure Connections pairing per Bluetooth Core Spec 5.x with the operator's documented authentication mode (Just Works / Numeric Comparison / Passkey Entry / Out-of-Band).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/wireless-charging/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-wireless-charging-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/wireless-charging-host:1.0.0` ships every wireless-charging envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/wireless-charging.sh` ships sample envelope generators with no
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
ecosystem. Wireless-charging deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
