# WIA-AUTO-010 — Phase 2: API Interface

> Vehicle-infotainment canonical Phase 2: API surface (units + profiles + media + voice + state + distraction-conformance).

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


## 11. API Interface

### 11.1 Display API

```typescript
interface DisplayAPI {
  // Get display configuration
  getDisplays(): Display[];

  // Set display zone content
  setZoneContent(displayId: string, zoneId: string, content: Content): void;

  // Set brightness
  setBrightness(displayId: string, level: number): void;

  // Set display mode
  setMode(displayId: string, mode: 'day' | 'night' | 'auto'): void;
}
```

### 11.2 Audio API

```typescript
interface AudioAPI {
  // Play audio from source
  play(source: AudioSource): void;

  // Set volume
  setVolume(level: number): void;

  // Set audio profile
  setProfile(profile: AudioProfile): void;

  // Enable/disable spatial audio
  setSpatialAudio(enabled: boolean): void;

  // Enable/disable ANC
  setANC(enabled: boolean): void;
}
```

### 11.3 Navigation API

```typescript
interface NavigationAPI {
  // Start navigation to destination
  navigate(destination: Location, options?: RouteOptions): Route;

  // Get current route
  getCurrentRoute(): Route | null;

  // Cancel navigation
  cancelNavigation(): void;

  // Search POI
  searchPOI(query: string, location?: Location): POI[];
}
```

---



## 12. User Experience Protocols

### 12.1 Startup Sequence

```
IVI Boot Sequence:
1. Hardware Initialization (0-2s):
   - Display power on
   - Audio system init
   - Network interfaces up

2. OS Boot (2-5s):
   - Kernel load
   - Essential services start
   - HMI framework load

3. Application Launch (5-8s):
   - Home screen display
   - Recent state restore
   - Background service start

4. Connected Services (8-15s):
   - Smartphone connection
   - Cloud sync
   - OTA check

Total Boot Time: <15 seconds (cold boot)
Resume from Sleep: <2 seconds
```

### 12.2 Error Handling

```
Error Handling Strategy:
1. Graceful Degradation:
   - Core functions always available
   - Non-critical features fail silently
   - User notified of major failures

2. Error Messages:
   - Clear, concise language
   - Actionable suggestions
   - Technical details hidden

3. Recovery:
   - Automatic retry (3 attempts)
   - Fallback to last known state
   - Manual reset option
```

---




---

## A.1 Endpoint reference

```http
GET    /vehicle-infotainment/v1/units              # list head-unit configurations
GET    /vehicle-infotainment/v1/units/{vin}        # per-VIN configuration
POST   /vehicle-infotainment/v1/profiles           # publish driver/passenger profile
GET    /vehicle-infotainment/v1/profiles/{id}      # fetch profile
POST   /vehicle-infotainment/v1/media-sessions     # create media-session
WS     /vehicle-infotainment/v1/state/stream       # head-unit state stream
GET    /vehicle-infotainment/v1/voice/grammar      # active voice grammar
POST   /vehicle-infotainment/v1/feedback           # in-app user feedback
```

Every endpoint follows the discovery convention at `/.well-known/wia-vehicle-infotainment`. VIN-bound endpoints require an OEM-issued credential plus the per-vehicle owner consent envelope.

## A.2 Profile and personalisation API

`POST /profiles` accepts a driver/passenger profile envelope: identifier, language preference per IETF BCP 47, units (metric / imperial), HMI theme, navigation home/work, audio EQ preset, distraction-policy variant (default / driving-school / chauffeur), accessibility preferences (high-contrast theme, large-font flag, audio-description for navigation prompts, haptic-emphasis), and the cloud-sync envelope. Profiles are bound to the user's WIA-OMNI-API tenant credential so the same profile can be applied across vehicles in a connected fleet.

## A.3 Media-session API

`POST /media-sessions` opens a media session with the requested source (FM/AM/DAB+ broadcast, satellite radio, online streaming via partnered services, USB media, Bluetooth A2DP, Apple CarPlay / Android Auto). The endpoint supports ducking for navigation prompts and phone calls, multi-zone audio routing (rear-passengers different content than front), and the driver-distraction filter that suppresses video playback in the driver zone while the vehicle is moving.

## A.4 Voice-assistant API

`GET /voice/grammar` returns the active voice grammar for the head-unit's configured assistant: built-in OEM assistant, Apple Siri (when CarPlay is active), Google Assistant (Android Auto active), Amazon Alexa (Echo Auto / partnered integrations). The endpoint exposes the locale-specific grammar plus the wake-word policy. Voice-input audio is processed per the privacy envelope: wake-word detection on-device with no network egress until the wake-word is detected; subsequent intent processing per the user's privacy-tier selection.

## A.5 Telemetry and HUD WebSocket

The state-stream WebSocket multiplexes head-unit state events (active source, current track, navigation phase, voice-assistant state, climate-control state where the climate UI is hosted on the IVI), HUD prompts (active turn instruction, speed-limit, hazard alert), and the per-zone audio level. Subscribers can filter by event class. The broker enforces VIN-bound credential validation and rejects connections without the per-vehicle owner consent envelope.

## A.6 Rate-limit and conformance envelope

VIN-bound endpoints: 1000 req/h per VIN authenticated. WebSocket subscriptions: bounded at 5 simultaneous per VIN so multiple in-vehicle apps and a paired smartphone can subscribe without head-unit overload. Voice-grammar refresh follows the operator's release cadence; the API serves the cached grammar with the `X-Grammar-Version` header so clients can detect new grammar versions.

## A.7 Driver-distraction conformance API

`POST /distraction-conformance` submits a UI flow recording (sequence of screens, durations, glance counts) for evaluation against the NHTSA Visual-Manual Driver Distraction Guidelines and the JAMA / Euro-NCAP HMI assessment criteria. The response carries the conformance verdict (pass / fail / advisory), the per-task glance-count and total-task-time metrics, and the recommended remediation (reduce nesting, increase target size, add voice alternative). OEMs use this endpoint during pre-launch HMI certification.


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
