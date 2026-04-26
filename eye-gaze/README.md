# WIA-AAC-006 Eye Gaze Standard

> Cross-vendor data and protocol layer for eye-gaze input devices

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Standard](https://img.shields.io/badge/Standard-WIA--AAC--006-sky.svg)](https://wia.org/standards/aac-006)

## Overview

WIA-AAC-006 defines standardized data formats and protocol semantics
for eye-gaze input devices used in assistive communication, hands-free
control, and accessibility applications. The standard is interoperable
with COGAIN dataset conventions and the Apple/Google accessibility
event APIs.

## Key Features

- **Calibration record** — gaze-target mapping, calibration session,
  per-eye accuracy envelope.
- **Stream framing** — 60–120 Hz fixation/saccade events, blink
  signalling, dwell-time selection states.
- **Privacy primitives** — on-device differential privacy for
  aggregated gaze heatmaps; encrypted off-device transport.
- **Accessibility integrations** — assistive on-screen keyboard
  interface, AAC vocabulary selection, environmental control.
- **Conformance tiers** — Surface (data formats), Verified (annual
  third-party audit), Anchored (continuous evidence package).

## Quick Start

```bash
npm install @wia/eye-gaze
```

```typescript
import { GazeStreamClient } from '@wia/eye-gaze';

const client = new GazeStreamClient({
  baseUrl: 'https://api.example.com/eye-gaze',
  apiKey: process.env.WIA_API_KEY!,
});

await client.calibrate({ targets: 9, durationMs: 1500 });
const subscription = client.stream(
  (event) => console.log(event.kind, event.fixation),
);
```

## Documentation

- `spec/PHASE-1-DATA-FORMAT.md` — schemas
- `spec/PHASE-2-API-INTERFACE.md` — REST + streaming APIs
- `spec/PHASE-3-PROTOCOL.md` — message protocol
- `spec/PHASE-4-INTEGRATION.md` — integration patterns

## Compliance Standards

- ISO 9241-11:2018 (usability of interactive systems)
- ISO 9241-940:2017 (gaze-tracking definitions)
- ISO/IEC 27001:2022 (information security)
- W3C WAI-ARIA 1.2 (accessibility)
- WCAG 2.2 (web content accessibility)

## License

MIT License — © 2025 WIA Standards Committee

弘益人間 (Hongik Ingan) — Benefit All Humanity
