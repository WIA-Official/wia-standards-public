# WIA-RH-002 Exoskeleton Standard

> Cross-vendor data and protocol layer for rehabilitation exoskeletons

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Standard](https://img.shields.io/badge/Standard-WIA--RH--002-sky.svg)](https://wia.org/standards/rh-002)

## Overview

WIA-RH-002 defines standardized data formats and protocol semantics for
powered rehabilitation exoskeletons, including joint-state telemetry,
gait-cycle markers, intent-detection events, safety-check records, and
session data exchange. The standard is interoperable with ROS 2 (Open
Robotics) joint-state messages and the IEC 80601-2-78 medical-robot
control envelope.

## Key Features

- **Joint-state schema** — per-joint position/velocity/torque framing.
- **Gait cycle markers** — heel-strike, toe-off, swing, stance.
- **Intent detection** — EMG-derived intent envelopes (per Hudgins TD4
  feature family).
- **Safety primitives** — emergency-stop event, joint-limit guards,
  overload-detection events.
- **Session data** — per-session program, progression metrics, and
  outcome envelopes.
- **Conformance tiers** — Surface (data formats), Verified (annual
  third-party audit), Anchored (continuous evidence package).

## Quick Start

```bash
npm install @wia/exoskeleton
```

```typescript
import { ExoStreamClient } from '@wia/exoskeleton';

const client = new ExoStreamClient({
  baseUrl: 'https://api.example.com/exoskeleton',
  apiKey: process.env.WIA_API_KEY!,
});

await client.startSession({ program: 'gait-rehab-A1', operator: 'PT-1' });
client.stream(
  (event) => console.log(event.kind, event.joints),
);
```

## Documentation

- `spec/PHASE-1-DATA-FORMAT.md` — schemas
- `spec/PHASE-2-API-INTERFACE.md` — REST + streaming APIs
- `spec/PHASE-3-PROTOCOL.md` — message protocol
- `spec/PHASE-4-INTEGRATION.md` — integration patterns
- Topical specs: `JOINT-STATE-SPEC.md`, `CONTROL-MODES.md`,
  `INTENT-DETECTION.md`, `EMERGENCY-STOP.md`, `OVERLOAD-DETECTION.md`,
  `SAFETY-CHECKLIST.md`, `JOINT-LIMITS.md`, `GAIT-CYCLE-SPEC.md`,
  `SESSION-DATA-SPEC.md`, `PROGRESS-METRICS.md`, `REHAB-PROGRAM-SPEC.md`

## Compliance Standards

- IEC 80601-2-78:2019 (medical-robot control envelope)
- IEC 60601-1:2005+A1+A2 (general medical-electrical safety)
- ISO 13482:2014 (personal-care robot safety)
- ISO 14971:2019 (risk management for medical devices)
- ISO/IEC 62304:2006+A1:2015 (medical-device software lifecycle)
- ROS 2 Humble (Open Robotics) joint-state and command interfaces

## License

MIT License — © 2025 WIA Standards Committee

弘益人間 (Hongik Ingan) — Benefit All Humanity
