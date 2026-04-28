# WIA-COMM-008 — Phase 2: API Interface

> Wireless-charging canonical Phase 2: API surface (chargers + sessions + telemetry + conformance + array).

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


## 14. Interoperability Testing

### 14.1 Qi Certification

WPC certification tests:
1. **Power transfer**: Verify power levels (BPP/EPP)
2. **Communication**: Packet structure and timing
3. **Efficiency**: Meet minimum efficiency targets
4. **FOD**: Detect standard metal objects
5. **EMI/EMC**: Electromagnetic compatibility
6. **Safety**: Temperature, overcurrent, overvoltage

### 14.2 AirFuel Certification

AirFuel Alliance tests:
1. **Resonant frequency**: 6.78 MHz ±5%
2. **BLE communication**: Device pairing and control
3. **Multi-device**: Simultaneous charging capability
4. **Safety**: EMF exposure, thermal limits

### 14.3 Cross-Platform Testing

Test devices on:
- Multiple charger brands
- Different power levels
- Various alignment offsets
- Temperature extremes (-10°C to 45°C)

---




---

## A.1 Endpoint reference

```http
POST /wireless-charging/v1/chargers              # register charger
GET  /wireless-charging/v1/chargers/{id}         # fetch charger record
POST /wireless-charging/v1/sessions              # start charging session
GET  /wireless-charging/v1/sessions/{id}/state   # current session state
POST /wireless-charging/v1/sessions/{id}/stop    # stop session
WS   /wireless-charging/v1/telemetry/stream      # power/temperature telemetry
GET  /wireless-charging/v1/conformance/{id}      # WPC/AirFuel/J2954 cert status
```

Every endpoint follows the discovery convention at `/.well-known/wia-wireless-charging`.

## A.2 Session-management API

`POST /sessions` opens a charging session: charger reference, receiver descriptor (per the device's NFC-detected or BLE-discovered identity), requested-power envelope, target state-of-charge, and the user-consent envelope. The endpoint validates the receiver against the charger's compatibility matrix (Qi-only chargers reject AirFuel-only receivers and vice versa unless the charger advertises multi-protocol support). Session state machine: `negotiating`, `ramping`, `charging`, `topping-off`, `complete`, `aborted-fod`, `aborted-thermal`, `aborted-misalignment`.

## A.3 Telemetry WebSocket

The telemetry WebSocket multiplexes per-session events: instantaneous Tx power, instantaneous Rx power (calculated from the in-band ASK / BLE side-channel), efficiency, Tx and Rx coil temperatures, FOD status, alignment estimate (X/Y offset in mm + tilt where the receiver supports tilt sensing), and the session-progress events (state-of-charge percentage). Subscribers filter by session-id; the broker emits alarm events on FOD detection, thermal threshold crossing, or alignment loss exceeding policy.

## A.4 Conformance and certification API

`GET /conformance/{id}` returns the charger's certification envelope: WPC Qi certification number; AirFuel Alliance certification number where applicable; SAE J2954 conformance tier (WPT 1/2/3 with optional MF-MF or MF-FF or FF-MF or FF-FF combinations); the EMC test report reference per CISPR 11 / FCC 47 CFR Part 15 / ETSI EN 300 330 / EN 301 489; and the EMF safety conformance per ICNIRP / IEEE C95.1.

## A.5 Multi-device and array API

For multi-coil array chargers (spatial-freedom Qi at 15-22 W, AirFuel zone chargers, MultiCharger desks), the API exposes the active-device list with per-device coil-allocation, the per-device power envelope, and the cross-talk envelope. Operator-side scheduling can prioritize charging slots based on declared device priorities (e.g., low-battery first; user-tagged priority device first).

## A.6 Rate-limit envelope

Charger-record endpoints: 1000 req/h authenticated, 5000 req/h trusted partner. Session-management endpoints: rate-limited per charger to 10 sessions/min (defeats coil-thermal abuse). Telemetry WebSocket subscriptions are bounded at 50 simultaneous per credential. Bulk-export (`GET /chargers?after=cursor&limit=N`) max 200 per page.

## A.7 Interoperability-testing API

`POST /interop-tests` registers an interoperability test run between a charger and a receiver pair. The endpoint accepts the per-test envelope (test catalogue identifier per WPC Qi compliance test plan / AirFuel Alliance certification test plan / SAE J2954 conformance test plan), the test-conditions envelope (alignment offsets, ambient temperature, input voltage profile), and the test-house identifier per ISO/IEC 17025. The response carries the per-test pass / fail / advisory verdict with the per-test diagnostic trace; aggregate cross-vendor compatibility matrices are exposed at `GET /interop-tests/matrix` with per-charger-class versus per-receiver-class breakdowns.

## A.8 Webhook delivery for session and FOD events

Operators can subscribe to webhook deliveries on session lifecycle events (negotiating, ramping, charging, complete, aborted) and on FOD detection events. Webhook payloads carry the same envelope shape as the API GET response, signed by the WIA tenant key, and retried with exponential backoff up to 24 h before the broker logs a permanent-failure event for operator review. FOD-event webhook subscribers (typically operator safety teams, manufacturer post-market-surveillance teams) receive the per-detection root-cause trace plus the recommended remediation.


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
