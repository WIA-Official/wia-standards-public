# WIA-ROBOT-005 Delivery Robot Standard

> Cross-vendor data and protocol layer for autonomous delivery robots

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Standard](https://img.shields.io/badge/Standard-WIA--ROBOT--005-sky.svg)](https://wia.org/standards/robot-005)

## Overview

WIA-ROBOT-005 defines standardized data formats and protocol semantics
for last-mile autonomous delivery robots: robot identification, mission
manifests, telemetry framing, hand-off events, and incident reporting.
The standard is interoperable with ISO 13482:2014 (personal-care robot
safety), ISO 22166-1:2021 (modularity in robotics), and the OASIS Common
Alerting Protocol (CAP) 1.2 for incident broadcast.

## Key Features

- **Robot identification** — stable identifiers, device classes, software
  build digest, manifest signing key.
- **Mission manifest** — origin, destination, payload class, route policy.
- **Telemetry framing** — pose, velocity, battery, sensor health.
- **Hand-off events** — pickup, custody chain, delivery, exception.
- **Incident reporting** — collision, theft, environmental hazard.
- **Conformance tiers** — Surface (data formats), Verified (annual
  third-party audit), Anchored (continuous evidence package).

## Quick Start

```bash
npm install @wia/delivery-robot
```

```typescript
import { RobotMissionClient } from '@wia/delivery-robot';

const client = new RobotMissionClient({
  baseUrl: 'https://api.example.com/delivery-robot',
  apiKey: process.env.WIA_API_KEY!,
});

const mission = await client.createMission({
  origin: 'WAREHOUSE-001',
  destination: '5-DOLLAR-ALLEY',
  payloadClass: 'parcel-S',
});
```

## Compliance Standards

- ISO 13482:2014 (personal-care robot safety)
- ISO 22166-1:2021 (modularity in robotics)
- ISO 22737:2023 (low-speed automated driving systems)
- ISO/IEC 27001:2022 (information security management)
- OASIS Common Alerting Protocol (CAP) 1.2
- ROS 2 Humble (Open Robotics)

## License

MIT License — © 2025 WIA Standards Committee

弘益人間 (Hongik Ingan) — Benefit All Humanity
