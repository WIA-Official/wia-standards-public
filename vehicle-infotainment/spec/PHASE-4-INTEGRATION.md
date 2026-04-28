# WIA-AUTO-010 — Phase 4: Integration

> Vehicle-infotainment canonical Phase 4: ecosystem integration (NHTSA + ISO 26262 + ISO/SAE 21434 + AVB/TSN + accessibility).

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


## 3. Display Technologies

### 3.1 Display Types

#### 3.1.1 Primary Display (Center Stack)

```
Specifications:
- Size: 10.2" - 17" diagonal
- Resolution: 1920x1080 minimum (Full HD)
- Technology: IPS LCD, OLED, or mini-LED
- Brightness: 500-1000 nits
- Touch: Capacitive multi-touch (10+ points)
- Refresh Rate: 60Hz minimum, 120Hz recommended
- Viewing Angle: 178° (IPS)
- Response Time: <10ms
```

#### 3.1.2 Instrument Cluster Display

```
Specifications:
- Size: 10" - 12.3" diagonal
- Resolution: 1920x720 (21:9) typical
- Technology: TFT LCD or OLED
- Brightness: 800-1500 nits (sunlight readable)
- Refresh Rate: 60Hz minimum
- Response Time: <5ms (critical info display)
```

#### 3.1.3 Head-Up Display (HUD)

```
Specifications:
- Virtual Image Size: 50" - 80" equivalent
- Resolution: 800x480 minimum
- Technology: DLP, TFT, or Laser scanning
- Brightness: Auto-adjust based on ambient light
- Distance: 2.0m - 3.5m virtual image distance
- Field of View: 5° - 10° horizontal
- Content: Speed, navigation, ADAS warnings
```

### 3.2 Multi-Display Configuration

#### 3.2.1 Zone Architecture

```
Display Zones:
┌─────────────────────────────────────────────┐
│  [Cluster]  │    [Primary]    │  [Climate]  │
│   12.3"     │      15.6"      │    8"       │
│             │                 │             │
│  Driver     │   Navigation    │   HVAC      │
│  Info       │   Media         │   Controls  │
│  ADAS       │   Apps          │   Seats     │
└─────────────────────────────────────────────┘
```

#### 3.2.2 Content Distribution

The IVI system must support:
- **Independent content**: Each display shows different content
- **Extended desktop**: Span content across multiple displays
- **Mirrored content**: Duplicate content on multiple displays
- **Dynamic allocation**: Move content between displays

### 3.3 Display Modes

```
Adaptive Display Modes:
1. Day Mode:
   - Brightness: 80-100%
   - Color Temperature: 6500K (neutral)
   - Contrast: Standard

2. Night Mode:
   - Brightness: 20-40%
   - Color Temperature: 3500K (warm)
   - Contrast: Reduced
   - Blue Light Filter: Enabled

3. Auto Mode:
   - Ambient Light Sensor: Continuous adjustment
   - GPS Time: Schedule-based switching
   - Headlight Status: Sync with vehicle lights
```

---



## 4. Audio Systems

### 4.1 Audio Architecture

#### 4.1.1 Channel Configuration

```
Premium Audio System (12-channel):
Front Stage:
  - Left Front: Tweeter + Mid-range + Woofer
  - Right Front: Tweeter + Mid-range + Woofer
  - Center: Full-range or Mid-range

Rear Stage:
  - Left Rear: Coaxial or Component
  - Right Rear: Coaxial or Component

Bass:
  - Subwoofer: 8" - 12" driver(s)

Ambient:
  - Ceiling Speakers: 4-8 speakers (Dolby Atmos)
```

#### 4.1.2 Digital Signal Processing (DSP)

```
DSP Specifications:
- Processing: 32-bit floating point
- Sample Rate: 48kHz, 96kHz, or 192kHz
- Channels: 12-16 independent channels
- Latency: <10ms total system latency

DSP Functions:
1. Time Alignment: Per-channel delay
2. Equalization: 31-band parametric EQ
3. Crossover: 12/24 dB per octave filters
4. Dynamic Range: Compression/limiting
5. Spatial Processing: Surround sound, binaural
6. ANC: Active Noise Cancellation
```

### 4.2 Audio Features

#### 4.2.1 Spatial Audio

```
3D Audio Implementation:
- Format Support: Dolby Atmos, DTS:X, Sony 360
- Height Channels: 4-8 ceiling speakers
- Object-based Audio: Up to 128 audio objects
- Head Tracking: Optional (with camera/sensor)
- Personalization: HRTF (Head-Related Transfer Function)
```

#### 4.2.2 Active Noise Cancellation (ANC)

```
ANC System:
- Microphones: 2-4 reference microphones
- Processing: Real-time adaptive filtering
- Latency: <1ms (critical for effectiveness)
- Frequency Range: 40Hz - 500Hz
- Reduction: Up to 10-15dB in target frequencies
```

### 4.3 Audio Sources

Supported input sources:
- **AM/FM/HD Radio**: Digital and analog tuners
- **Satellite Radio**: SiriusXM, etc.
- **Bluetooth Audio**: A2DP, aptX, AAC, LDAC
- **USB Audio**: Digital audio from USB storage
- **Streaming**: Spotify, Apple Music, etc. (via smartphone)
- **Voice Calls**: Hands-free with echo cancellation
- **Navigation**: Voice guidance with audio ducking
- **ADAS Alerts**: Warning sounds with priority routing

### 4.4 Audio Profiles

```typescript
interface AudioProfile {
  name: string;
  equalizer: {
    bass: number;      // -12 to +12 dB
    mid: number;       // -12 to +12 dB
    treble: number;    // -12 to +12 dB
    bands: number[];   // 31-band EQ values
  };
  balance: number;     // -9 (left) to +9 (right)
  fade: number;        // -9 (rear) to +9 (front)
  volume: number;      // 0-100
  spatialAudio: boolean;
  anc: boolean;
  speedCompensation: boolean;
}
```

---



## 9. HMI Design Guidelines

### 9.1 Design Principles

#### 9.1.1 Safety-First Design

```
NHTSA Guidelines Compliance:
1. Glance Duration:
   - Single glance: <2 seconds
   - Total task glances: <12 seconds
   - Total task time: <24 seconds

2. Task Complexity:
   - Max 6 manual inputs per task
   - Max 30 characters text entry
   - No video playback while driving

3. Lockouts:
   - Complex tasks disabled >5 mph
   - Keyboard disabled while moving
   - Video disabled while moving
```

#### 9.1.2 Touch Targets

```
Touch Target Specifications:
- Minimum Size: 44x44 pixels (11mm x 11mm)
- Recommended Size: 60x60 pixels (15mm x 15mm)
- Spacing: 8 pixels (2mm) between targets
- Reach: Within 750mm from driver seat
- Haptic Feedback: Confirm all touches
```

### 9.2 Visual Design

#### 9.2.1 Typography

```
Font Specifications:
- Primary Font: Sans-serif (Roboto, SF Pro, custom)
- Minimum Size: 16pt for body text
- Title Size: 24-32pt
- Line Height: 1.4-1.6
- Font Weight: Regular, medium, bold
- Contrast: 4.5:1 minimum (WCAG AA)
```

#### 9.2.2 Color System

```
Color Palette (Orange Theme - #F97316):
Primary:
  - Brand Orange: #F97316
  - Dark Orange: #EA580C
  - Light Orange: #FB923C

Semantic:
  - Success: #10B981 (green)
  - Warning: #F59E0B (amber)
  - Error: #EF4444 (red)
  - Info: #3B82F6 (blue)

Neutral:
  - Background: #000000 - #1F1F1F (dark mode)
  - Surface: #2D2D2D - #3D3D3D
  - Text Primary: #FFFFFF
  - Text Secondary: #A0A0A0
```

### 9.3 Interaction Patterns

```
Common UI Patterns:
1. Navigation Bar:
   - Position: Bottom or side
   - Items: 4-6 primary functions
   - Icons: With labels

2. Cards:
   - Content: Grouped information
   - Actions: Swipe, tap, long-press
   - Elevation: Visual hierarchy

3. Lists:
   - Row Height: 60-80 pixels
   - Dividers: Subtle lines
   - Actions: Swipe gestures

4. Modals:
   - Overlay: Semi-transparent backdrop
   - Dismissal: Tap outside, swipe down
   - Focus: Single task completion
```

---



## 13. Safety and Compliance

### 13.1 Driver Distraction Guidelines

```
NHTSA Phase 1 Guidelines:
- Glance time: <2s per glance
- Total task time: <12s
- Manual inputs: <6 per task
- Text entry: <30 characters
- Lockouts: While driving >5 mph
- Video: Prohibited while moving
```

### 13.2 Accessibility

```
Accessibility Features:
- Screen Reader: TTS for visually impaired
- High Contrast: WCAG AAA compliance
- Large Text: Up to 200% scaling
- Voice Control: Full voice navigation
- Color Blind: Not color-dependent
- Haptic: Tactile feedback
- Simplified Mode: Reduced complexity
```

### 13.3 Data Privacy

```
Privacy Principles:
- Data Minimization: Collect only necessary
- User Consent: Explicit opt-in
- Transparency: Clear privacy policy
- Control: User data management
- Encryption: End-to-end where possible
- Retention: Limited storage duration
- Deletion: User-initiated data wipe
```

---



## 14. References

### 14.1 Standards and Guidelines

1. NHTSA (2013). "Visual-Manual NHTSA Driver Distraction Guidelines"
2. ISO 15005:2017. "Road vehicles — Ergonomic aspects of transport information and control systems"
3. SAE J2365. "Calculation of the Time To Complete In-Vehicle Navigation and Route Guidance Tasks"
4. WCAG 2.1. "Web Content Accessibility Guidelines"

### 14.2 Industry Standards

| Standard | Description |
|----------|-------------|
| GENIVI | Automotive middleware |
| AUTOSAR | Automotive software architecture |
| AGL | Automotive Grade Linux |
| Android Automotive | Google automotive platform |

### 14.3 WIA Standards

- WIA-INTENT: Intent-based interfaces
- WIA-OMNI-API: Universal API gateway
- WIA-AUTO-001: Vehicle diagnostics
- WIA-AUTO-005: Autonomous driving systems
- WIA-SOCIAL: Connected vehicle features
- WIA-HOME: Smart home integration

---

## Appendix A: Example Implementations

### A.1 Basic IVI Initialization

```typescript
import { InfotainmentSystem } from '@wia/auto-010';

const ivi = new InfotainmentSystem({
  displays: {
    primary: {
      size: 15.6,
      resolution: { width: 1920, height: 1080 },
      touchEnabled: true
    }
  },
  audio: {
    channels: 12,
    spatialAudio: true
  }
});

// Start navigation
ivi.navigation.navigate({
  address: "123 Main St, San Francisco, CA"
});

// Play media
ivi.media.play({
  source: "bluetooth",
  track: "Song Title"
});

// Connect smartphone
await ivi.smartphone.connect({
  protocol: "carplay",
  wireless: true
});
```

### A.2 Voice Command Handling

```typescript
ivi.voice.on('command', (command) => {
  switch (command.intent) {
    case 'navigation':
      ivi.navigation.navigate(command.destination);
      break;
    case 'climate':
      ivi.climate.setTemperature(command.value);
      break;
    case 'media':
      ivi.media.play(command.track);
      break;
  }
});
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-010 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*



---

## A.1 Standards cross-walk

| Concern                       | Standard                                  |
|-------------------------------|-------------------------------------------|
| Driver distraction (US)       | NHTSA Visual-Manual Driver Distraction Guidelines |
| Driver distraction (JP)       | JAMA HMI Guidelines                       |
| Driver distraction (EU)       | ESoP — European Statement of Principles   |
| Vehicle controls / icons      | ISO 2575                                  |
| Display legibility            | ISO 15008                                 |
| HMI usability                 | ISO 9241 series (ergonomics)              |
| Functional safety             | ISO 26262                                 |
| SOTIF                         | ISO 21448                                 |
| Cybersecurity                 | ISO/SAE 21434 + UN R155                   |
| OTA software updates          | ISO 24089 + UN R156                       |
| Time-sensitive networking     | IEEE 802.1Qav / 802.1Qbv (AVB / TSN)      |
| In-vehicle Ethernet           | IEEE 802.3bw / IEEE 802.3ch (1G / 10G)    |
| Wi-Fi                         | IEEE 802.11ax / 802.11be (Wi-Fi 6/6E/7)   |
| Bluetooth                     | Bluetooth Core Spec 5.x                   |
| UWB digital key               | CCC Digital Key Release 3.0               |
| Audio codecs                  | ISO/IEC 14496-3 (AAC) + Dolby spec        |
| HD radio                      | iBiquity NRSC-5                           |
| DAB+                          | ETSI EN 300 401 / TS 102 563              |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.2 OEM and aftermarket integration

OEM integration captures the head-unit's place in the E/E architecture: domain-controller-hosted IVI vs. zone-controller-hosted; cluster-IVI consolidation via QNX Hypervisor or AGL Virtualisation; the audio-bus topology (A2B / MOST legacy / Ethernet AVB); the bus-of-record for safety-relevant signals (CAN / CAN-FD / Automotive Ethernet); the diagnostic-protocol envelope (UDS ISO 14229 + DoIP ISO 13400 over Automotive Ethernet). Aftermarket integration captures the digital-key envelope for shared/rental vehicles, the smartphone-projection envelope, and the diagnostic-port (OBD-II) envelope where applicable.

## A.3 Content-and-licensing integration

Content-and-licensing integration captures the music / podcast / audiobook / video / radio service envelope, the per-service authentication and DRM envelope (FairPlay / Widevine / PlayReady), the regional-availability envelope per ISRC / ISWC / IPI metadata, the per-trip listening-history retention policy, and the lossless-audio licensing envelope (Hi-Res Audio per JAS / NRSC-5 HD Radio licensing). Live-radio integration covers DAB+ ETSI EN 300 401, HD Radio iBiquity, and satellite radio SiriusXM where adopted.

## A.4 Accessibility integration

Accessibility integration honours WCAG 2.2 plus the in-vehicle adaptations: large-font and high-contrast themes, audio descriptions for navigation prompts, haptic emphasis for hearing-impaired drivers, screen-reader compatibility for passenger-zone content, and the per-language/locale envelope including right-to-left rendering and CJK font support. The integration envelope cross-references the WIA-A11Y attestation chain so accessibility audits are not re-executed each vehicle generation.

## A.5 Future directions

Active research tracks: pillar-to-pillar OLED displays with privacy-zone shading; AR-HUD with full-windshield rendering and lane-overlay; on-device generative-AI assistants with offline operation; cluster-and-IVI consolidation under a single mixed-criticality hypervisor; passenger-zone gaming and streaming with cloud-game-streaming integration; in-vehicle videoconferencing with privacy-aware mute and camera-shutter; software-defined-vehicle (SDV) integration where IVI features ship as containerised microservices under a vehicle OS. The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4.

## A.6 Reference list

- NHTSA Visual-Manual Driver Distraction Guidelines (2013 + 2024 update)
- JAMA HMI Guidelines for In-Vehicle Display Systems
- ESoP — European Statement of Principles on Human Machine Interaction
- ISO 2575 — Road vehicles — symbols for controls, indicators
- ISO 15008 — Ergonomic aspects of transport information and control systems — display legibility
- ISO 9241 series — ergonomics of human-system interaction
- ISO 26262 — road vehicles functional safety
- ISO/SAE 21434 — road vehicles cybersecurity engineering
- ISO 24089 — software updates engineering
- UN ECE WP.29 R155 / R156 — cybersecurity / software updates
- IEEE 802.1Qav / Qbv / Qbu — TSN
- IEEE 802.3bw / 802.3ch — Automotive Ethernet
- CCC Digital Key Release 3.0 — Car Connectivity Consortium UWB digital key
- ISO 14229 (UDS) + ISO 13400 (DoIP) — diagnostics
- Bluetooth Core Spec 5.4
- Wi-Fi 6/6E/7 (IEEE 802.11ax / 802.11be)


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
