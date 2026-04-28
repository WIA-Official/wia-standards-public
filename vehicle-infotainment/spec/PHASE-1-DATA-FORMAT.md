# WIA-AUTO-010 — Phase 1: Data Format

> Vehicle-infotainment canonical Phase 1: IVI + display + audio + HMI + smartphone-projection envelopes.

# WIA-AUTO-010: Vehicle Infotainment System Specification v1.0

> **Standard ID:** WIA-AUTO-010
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Technology Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [IVI System Architecture](#2-ivi-system-architecture)
3. [Display Technologies](#3-display-technologies)
4. [Audio Systems](#4-audio-systems)
5. [Navigation Integration](#5-navigation-integration)
6. [Smartphone Integration](#6-smartphone-integration)
7. [Voice Assistant Integration](#7-voice-assistant-integration)
8. [Climate Control Integration](#8-climate-control-integration)
9. [HMI Design Guidelines](#9-hmi-design-guidelines)
10. [Data Formats](#10-data-formats)
11. [API Interface](#11-api-interface)
12. [User Experience Protocols](#12-user-experience-protocols)
13. [Safety and Compliance](#13-safety-and-compliance)
14. [References](#14-references)

---


## 10. Data Formats

### 10.1 Media Metadata

```json
{
  "type": "media",
  "source": "bluetooth",
  "metadata": {
    "title": "Song Title",
    "artist": "Artist Name",
    "album": "Album Name",
    "duration": 240,
    "position": 45,
    "artwork": "https://example.com/artwork.jpg",
    "genre": "Rock",
    "year": 2024
  },
  "playback": {
    "state": "playing",
    "volume": 75,
    "repeat": "off",
    "shuffle": false
  }
}
```

### 10.2 Navigation Route

```json
{
  "type": "route",
  "origin": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "address": "San Francisco, CA"
  },
  "destination": {
    "latitude": 37.6213,
    "longitude": -122.3790,
    "address": "San Francisco International Airport"
  },
  "waypoints": [],
  "summary": {
    "distance": 21500,
    "duration": 1380,
    "trafficDuration": 1620
  },
  "steps": [
    {
      "instruction": "Head north on Market St",
      "distance": 450,
      "duration": 60,
      "maneuver": "straight"
    }
  ]
}
```

### 10.3 Vehicle Status

```json
{
  "type": "vehicle_status",
  "timestamp": "2025-12-26T10:30:00Z",
  "powertrain": {
    "type": "electric",
    "batteryLevel": 85,
    "range": 340,
    "charging": false
  },
  "climate": {
    "temperature": 72,
    "fanSpeed": 5,
    "mode": "auto"
  },
  "doors": {
    "driverFront": "closed",
    "passengerFront": "closed",
    "driverRear": "closed",
    "passengerRear": "closed",
    "trunk": "closed",
    "locked": true
  },
  "lights": {
    "headlights": "auto",
    "interior": false
  }
}
```

---




---

## A.1 IVI-record envelope

The Phase 1 envelope groups in-vehicle infotainment (IVI) records by head-unit class (entry / mid / premium / luxury, per the OEM's tiering), display configuration (single CID / dual CID + cluster / pillar-to-pillar / curved with passenger zone / HUD), audio configuration (4.0 / 5.1 / 7.1.4 / Dolby Atmos / spatial audio), the connectivity envelope (Wi-Fi 6/6E/7, Bluetooth 5.x, UWB for digital key, LTE-NR cellular, optional satellite IP), and the OS configuration (Android Automotive OS / QNX / Linux / proprietary RTOS for cluster / GENIVI baseline). Records carry the SOC family identifier and the GPU class for the rendering envelope at Phase 4 §A.1.

## A.2 Display-and-input descriptor

A display descriptor MUST list: panel technology (TFT-LCD / LTPS / IPS / OLED / micro-LED / E-Ink for status); resolution, pixel density (PPI), refresh rate (60/90/120 Hz), peak brightness (typical 800-1500 cd/m^2 for sunlight readability), contrast ratio, colour gamut (sRGB / DCI-P3 / Rec.2020 coverage), and the touch-input class (capacitive multi-touch with palm rejection; haptic feedback per ISO 9241-411 / IEC 60068 mechanical robustness). HUD descriptors carry the projection technology (combiner / windshield-direct / AR-HUD with light-engine class), the eye-box envelope, and the calibration envelope.

## A.3 Audio descriptor

Audio descriptors carry: amplifier configuration (head-unit on-board class-AB / class-D, external DSP amplifier, multi-zone amplifier), speaker count and placement (front, rear, surround, ceiling, headrest, subwoofer), nominal power per channel, frequency response, the noise-cancellation envelope (active road-noise cancellation, engine-order cancellation), the equalisation profile per OEM tuning, and the codec catalogue (AAC, MP3, FLAC, ALAC, WAV, OGG, OPUS, plus Dolby Atmos object/channel decoders where licensed). Audio routing follows the AUTOSAR Adaptive routing graph or the GENIVI Audio Manager equivalent.

## A.4 HMI-design descriptor

HMI descriptors carry: theme catalogue (light / dark / auto-switching with ambient-light input), font catalogue (per-language with CJK and right-to-left support), iconography per ISO 15008 (legibility) plus ISO 2575 (vehicle controls/indicators), driver-distraction policy per NHTSA Visual-Manual Distraction Guidelines (12-second total task time; 2-second per glance), the voice-first interaction envelope, the haptic-feedback envelope, and the multi-occupant zone configuration (driver / front passenger / rear).

## A.5 Smartphone-projection envelope

Smartphone-projection envelopes carry: Apple CarPlay (wired and wireless) configuration; Android Auto (wired and wireless) configuration; the mirroring profile (resolution, refresh rate, audio routing); the capability envelope (Siri / Google Assistant / Alexa hand-off; calendar; messaging; navigation override); and the privacy envelope (which on-phone data is shared with the head-unit, retention, and per-trip ephemeral storage). The envelope cross-references the HMI-design descriptor at Phase 1 §A.4 so the projected experience honours the host vehicle's distraction policy.

## A.6 Profile-and-personalisation envelope

Profile envelopes carry: profile identifier (linked to the user's WIA-OMNI-API tenant credential), language preference per IETF BCP 47, regional preferences (units metric/imperial; date/time format per ISO 8601 + locale conventions), HMI theme and accent-colour preference, navigation home / work / favourite places, audio EQ preset and per-zone volume bias, voice-assistant choice and per-locale wake-word, accessibility preferences (high-contrast theme, large-font flag, audio-description for navigation prompts, haptic-emphasis for hearing-impaired drivers, screen-reader compatibility for passenger-zone content), distraction-policy variant (default / driving-school / chauffeur), and the cloud-sync envelope. Cross-vehicle profile portability is bounded by the operator's connected-fleet-roaming envelope.

## A.7 Telemetry and feedback envelope

Telemetry envelopes carry: per-trip aggregate session data (drive duration, source-of-record for media playback, voice-assistant invocation count and outcome distribution, navigation-prompt-hit-rate, distraction-warning escalation history) under k>=20 spatial bin granularity. The envelope respects the user's per-tier privacy selection per Phase 3 §A.4: maximum-cloud opts in to per-trip detailed analytics; hybrid keeps raw audio and screen recordings on-device; on-device-only emits no telemetry. Per-VIN feedback submissions (`POST /feedback`) are bound to the session-of-record so engineering can correlate user reports with the diagnostic trace; submissions are retention-bound per the operator's per-jurisdiction rules.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/vehicle-infotainment/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-vehicle-infotainment-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/vehicle-infotainment-host:1.0.0` ships every vehicle-infotainment envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/vehicle-infotainment.sh` ships sample envelope generators with no
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
ecosystem. Vehicle-infotainment deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
