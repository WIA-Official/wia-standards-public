# WIA-AUTO-010 — Phase 3: Protocol

> Vehicle-infotainment canonical Phase 3: protocols (architecture + navigation + smartphone-projection + voice-assistant + climate-bridge).

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


## 2. IVI System Architecture

### 2.1 System Components

The IVI system consists of the following core components:

```
IVI System Architecture:
┌─────────────────────────────────────────────────────────┐
│                    Application Layer                     │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐  │
│  │Navigation│ │  Media   │ │  Phone   │ │ Vehicle  │  │
│  │          │ │  Player  │ │          │ │   Info   │  │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘  │
├─────────────────────────────────────────────────────────┤
│                   Middleware Layer                       │
│  ┌──────────────────────────────────────────────────┐  │
│  │     Service Manager (Audio, Display, Input)      │  │
│  └──────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────┤
│                  Hardware Abstraction Layer              │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐  │
│  │ Display  │ │  Audio   │ │   CAN    │ │   USB    │  │
│  │  Driver  │ │  Driver  │ │   Bus    │ │ Ethernet │  │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘  │
├─────────────────────────────────────────────────────────┤
│                     Hardware Layer                       │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐  │
│  │ Screens  │ │ Speakers │ │  Sensors │ │Antennas  │  │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘  │
└─────────────────────────────────────────────────────────┘
```

### 2.2 Processing Architecture

#### 2.2.1 System-on-Chip (SoC)

Minimum requirements:
- **CPU**: 8-core ARM or x86, 2.0GHz+
- **GPU**: Support for 4K displays, OpenGL ES 3.2+
- **RAM**: 8GB DDR4 minimum, 16GB recommended
- **Storage**: 128GB eMMC/UFS minimum, 256GB recommended
- **Connectivity**: Wi-Fi 6, Bluetooth 5.2, LTE/5G modem

#### 2.2.2 Operating System

Supported platforms:
- **Linux-based**: Android Automotive OS, AGL (Automotive Grade Linux)
- **QNX**: Real-time OS for safety-critical functions
- **Custom RTOS**: For resource-constrained systems

### 2.3 Communication Protocols

```
Vehicle Network Architecture:
┌──────────────────────────────────────────┐
│         IVI Head Unit                     │
├──────────────────────────────────────────┤
│  CAN Bus (500 kbps - 1 Mbps)            │  ← Vehicle data
│  CAN-FD (2-5 Mbps)                       │  ← High-speed vehicle
│  Ethernet (100 Mbps - 10 Gbps)           │  ← Camera, sensors
│  MOST (150 Mbps)                         │  ← Audio/video
│  LIN (20 kbps)                           │  ← Climate, lighting
│  USB (480 Mbps - 10 Gbps)                │  ← Smartphone
│  Bluetooth (3 Mbps)                      │  ← Phone, headset
│  Wi-Fi (1.2 Gbps)                        │  ← Smartphone, OTA
└──────────────────────────────────────────┘
```

---



## 5. Navigation Integration

### 5.1 Navigation Features

#### 5.1.1 Core Navigation

```
Navigation Capabilities:
1. Real-time Routing:
   - Traffic-aware routing
   - Dynamic rerouting
   - Multi-stop optimization
   - Alternative route suggestions

2. Map Data:
   - On-board maps: Regional/global
   - Map updates: OTA or USB
   - POI database: 100M+ points
   - 3D landmarks: Major cities

3. Search:
   - Address search
   - POI search by category
   - Natural language search
   - Recent/favorite locations
```

#### 5.1.2 Turn-by-Turn Guidance

```
Guidance Elements:
- Visual: Arrow, distance, street name, lane guidance
- Audio: Voice directions with street names (TTS)
- HUD: Projected turn arrows and distance
- Haptic: Optional steering wheel vibration

Voice Guidance:
- Languages: 40+ languages
- Voice Types: Male, female, neutral
- Volume: Auto-adjust based on speed and audio
- Interruption: Audio ducking, not muting
```

### 5.2 Advanced Navigation Features

#### 5.2.1 Predictive Destinations

```
AI-Powered Predictions:
- Home/Work: Commute time learning
- Frequent Locations: Pattern recognition
- Calendar Integration: Appointment addresses
- Contextual: Time of day, day of week
- Traffic Prediction: Historical data analysis
```

#### 5.2.2 EV-Specific Navigation

```
Electric Vehicle Features:
- Range Calculation: Real-time consumption
- Charging Stations: Database integration
- Route Planning: With charging stops
- Battery Optimization: Eco routing
- Preconditioning: Battery warming/cooling
- Charging Time: Estimated stop duration
```

### 5.3 Map Display

```
Map Rendering:
- Zoom Levels: 20+ levels (global to building)
- Orientation: North-up, heading-up, 3D
- Day/Night: Auto-switching color schemes
- Traffic Colors:
  - Green: Free flow (>50 mph)
  - Yellow: Moderate (25-50 mph)
  - Orange: Slow (10-25 mph)
  - Red: Congestion (<10 mph)
  - Black: Road closure
```

---



## 6. Smartphone Integration

### 6.1 Apple CarPlay

#### 6.1.1 Connection Methods

```
CarPlay Connection:
1. Wired (USB):
   - Cable: Lightning or USB-C
   - Data Rate: USB 2.0 (480 Mbps)
   - Charging: USB PD up to 12W
   - Latency: 50-100ms

2. Wireless:
   - Protocol: Wi-Fi Direct (5GHz)
   - Data Rate: Up to 867 Mbps
   - Pairing: Bluetooth for initial setup
   - Range: Up to 10 meters
   - Latency: 100-150ms
```

#### 6.1.2 CarPlay Features

```
Supported Functions:
- Phone: Calls, contacts, voicemail
- Messages: iMessage, SMS (Siri only)
- Maps: Apple Maps navigation
- Music: Apple Music, podcasts, audiobooks
- Third-party Apps: Approved apps only
- Siri: Voice control
- Now Playing: Media metadata display
- Calendar: Event viewing
```

### 6.2 Android Auto

#### 6.2.1 Connection Methods

```
Android Auto Connection:
1. Wired (USB):
   - Cable: USB-C
   - Data Rate: USB 2.0/3.0
   - Charging: USB PD up to 15W
   - Latency: 50-100ms

2. Wireless:
   - Protocol: Wi-Fi Direct (5GHz)
   - Data Rate: Up to 1.2 Gbps
   - Pairing: Bluetooth for setup
   - Range: Up to 10 meters
   - Latency: 100-150ms
```

#### 6.2.2 Android Auto Features

```
Supported Functions:
- Phone: Calls, contacts
- Messages: SMS via Google Assistant
- Maps: Google Maps, Waze
- Media: Spotify, YouTube Music, podcasts
- Third-party Apps: Wide app support
- Google Assistant: Voice control
- Calendar: Event notifications
- Weather: Current conditions
```

### 6.3 Integration Requirements

```
Display Requirements:
- Minimum Resolution: 800x480
- Aspect Ratio: 16:9, 16:10, 3:2
- Touchscreen: Capacitive multi-touch
- Physical Buttons: Optional (home, back, voice)

Audio Requirements:
- Digital Audio: 16-bit PCM or AAC
- Microphone: Noise-canceling
- Echo Cancellation: Required
- Volume Control: Integrated with vehicle

Network Requirements:
- Bluetooth: 4.0+ for pairing
- Wi-Fi: 802.11ac (5GHz) for wireless
- GPS: Shared location data
```

---



## 7. Voice Assistant Integration

### 7.1 Voice Recognition System

#### 7.1.1 Microphone Array

```
Microphone Configuration:
- Count: 2-4 microphones
- Type: MEMS, omnidirectional
- Placement: Roof console, steering wheel
- Beam-forming: Directional audio capture
- Noise Cancellation: Dual-mic or array-based
- Wind Noise Reduction: Hardware + software
```

#### 7.1.2 Speech Recognition

```
Recognition Engine:
- Type: Cloud-based + On-device
- Languages: 40+ languages
- Wake Word: "Hey [Assistant]"
- Continuous Listening: While driving
- Barge-in: Interrupt prompts
- Multi-modal: Voice + touch
```

### 7.2 Voice Commands

#### 7.2.1 Command Categories

```
Supported Command Types:

1. Navigation:
   - "Navigate to [destination]"
   - "Find nearest [POI type]"
   - "Show traffic on route"
   - "Avoid tolls"

2. Media:
   - "Play [song/artist/playlist]"
   - "Next track"
   - "Volume up/down"
   - "Switch to radio"

3. Phone:
   - "Call [contact]"
   - "Read messages"
   - "Reply [message]"

4. Climate:
   - "Set temperature to [value]"
   - "Turn on air conditioning"
   - "Defrost windshield"

5. Vehicle:
   - "Open sunroof"
   - "Lock doors"
   - "What's my range?"

6. Information:
   - "What's the weather?"
   - "What's my next appointment?"
   - "Find restaurants nearby"
```

### 7.3 Natural Language Processing

```
NLP Capabilities:
- Intent Recognition: Understand command goal
- Entity Extraction: Names, places, numbers
- Context Awareness: Multi-turn conversations
- Disambiguation: Clarifying questions
- Error Handling: Reprompt on misunderstanding
- Personalization: User preference learning
```

---



## 8. Climate Control Integration

### 8.1 HVAC Interface

```
Climate Control Features:
- Temperature: Per-zone control (2-4 zones)
- Fan Speed: Variable (0-10 levels)
- Air Distribution: Face, feet, windshield
- Recirculation: Fresh/recirculated air
- Auto Mode: Automatic climate
- Seat Heating: 3-5 levels
- Seat Cooling: 3 levels (ventilated seats)
- Steering Wheel Heat: On/off or variable
```

### 8.2 Air Quality

```
Air Quality Features:
- Cabin Filter: HEPA or activated carbon
- Air Purification: Ionizer, UV-C
- CO2 Monitoring: Sensor-based
- PM2.5 Monitoring: Particulate sensor
- Auto Recirculation: Based on exterior quality
- Pre-conditioning: Remote climate start
```

---




---

## A.1 IVI-architecture protocol

The IVI-architecture protocol enumerates the host platform classes (Android Automotive OS with CarService / Car-API; QNX with QNX Hypervisor for cluster/IVI consolidation; Linux with AGL / GENIVI baseline; embedded RTOS for cluster), the rendering pipeline (Vulkan / OpenGL ES / DirectX where Windows-derived; the surface compositor catalogue), the IPC fabric (Binder on Android Automotive; SOME/IP per AUTOSAR; D-Bus on AGL/GENIVI; the in-vehicle Ethernet AVB/TSN per IEEE 802.1Qav/Qbv for time-sensitive media), and the safety envelope (cluster-side ISO 26262 ASIL-B for instrument cluster information; IVI itself is QM unless safety-relevant content is hosted in which case ASIL applies).

## A.2 Navigation-integration protocol

Navigation integration honours the head-unit's navigation system (built-in OEM nav with HD-map updates; smartphone-projected nav from CarPlay / Android Auto; cloud-based nav with on-device cache). The protocol covers the route-and-guidance hand-off (smartphone-to-head-unit hand-off when entering the vehicle; head-unit-to-smartphone hand-off when leaving), the HD-map version-control envelope (incremental map updates per ISO/TS 22726), the live-traffic and TPEG-2 envelope, the EV-charging-stop integration (state-of-charge-aware routing, charging-station availability), and the per-route accessibility envelope (avoid stairs / cobblestones / steep grades for VRU passengers).

## A.3 Smartphone-projection protocol

Smartphone-projection protocol covers the wired link (USB-C with the Apple AAP / Google Auto Projection profiles), the wireless link (Wi-Fi Direct + Bluetooth pairing for first-time setup; WPA3 personal for the in-vehicle hotspot in CarPlay/AA), the streaming codec (H.264 / H.265 video; AAC / OPUS audio), the input feedback (touch input from the head-unit forwarded to the phone with reasonable latency), and the privacy envelope (the head-unit forwards user inputs and renders streamed UI but does NOT exfiltrate phone data to the head-unit's cloud unless the user explicitly grants per-purpose consent).

## A.4 Voice-assistant protocol

The voice-assistant protocol covers wake-word detection on-device (no audio egress until wake), the locale-aware natural-language understanding, the action graph (built-in actions: navigation, media, climate, phone, calendar; partnered skills via the operator's voice-platform; on-device fallback for offline operation), the multi-turn context envelope, and the privacy-tier selection (the user can opt for maximum-cloud / hybrid / on-device-only modes per the operator's voice-platform policy). Wake-word false-acceptance and false-rejection rates are measured per the operator's acceptance test; the protocol bans always-on cloud streaming and requires a hardware microphone-mute switch.

## A.5 Climate-control HMI-bridge protocol

Where the IVI hosts the climate-control HMI, the bridge protocol covers the safety-relevant climate-state read (cluster-side ASIL display of fan-speed, temperature setpoint, defrost state) and the QM-side IVI rich-graphics rendering of the same state. The bridge enforces freshness (a state value older than the documented bound is shown as "stale") and the failure envelope (climate ECU loss-of-link → IVI shows "climate temporarily unavailable" rather than stale values). Climate hard-controls (defrost, A/C on/off) MUST remain accessible via physical buttons or via voice when the IVI HMI is unavailable.

## A.6 Replay and integrity defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for the OEM-issued credential control plane. OTA-update bundles for the IVI follow ISO 24089 / UN ECE WP.29 R156 with signed images, rollback protection, and staged rollout. State-stream traffic uses mTLS with per-VIN monotonic counters; replay attempts are detected and dropped at the broker. Cluster-side ASIL safety messages ride on a separate trust domain bridged through a watchdog that quarantines compromised IVI partitions without affecting the cluster.


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
