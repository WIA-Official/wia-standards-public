# WIA Smart Wheelchair Standard

> Open standard for power and assistive wheelchairs: telemetry, navigation, accessibility, safety.

## Overview

The WIA Smart Wheelchair standard defines a unified data and protocol layer for power-assisted and autonomous wheelchairs so that hardware from different vendors can interoperate with care-team dashboards, accessibility services, and the broader WIA family.

The standard is published in four phases:

| Phase | Scope |
|-------|-------|
| 1 — Data Format | Telemetry, navigation, user profile, safety event envelopes |
| 2 — API Interface | HTTP surface for telemetry stream, route, dispatch, status |
| 3 — Protocol | Federation between care providers, replay defence, consent |
| 4 — Integration | ROS 2, CAN bus, WIA-OMNI-API, WIA-ACCESSIBILITY |

## Quick start

The reference implementation lives in `firmware/`, `ros2_ws/`, and `integrations/`. A minimal end-to-end exploration:

```bash
# 1. Run the reference host container
docker run -p 8080:8080 wia/smart-wheelchair-host:1.0.0

# 2. Subscribe to a wheelchair's telemetry
curl -N "http://localhost:8080/sw/telemetry/wheelchair-001" \
     -H "Accept: text/event-stream"

# 3. Dispatch a navigation goal
curl -X POST "http://localhost:8080/sw/route/wheelchair-001" \
     -H "Content-Type: application/json" \
     -d '{ "wia_smart_wheelchair_version":"1.0.0", "type":"route_goal", "goal":{ "x":3.5, "y":-1.2, "frame":"map" } }'
```

## CLI

A reference CLI ships under `cli/smart-wheelchair.sh` with subcommands `validate`, `telemetry`, `route`, `safety`, `consent`, `info`.

## Companion standards

* **ROS 2** — middleware for navigation and motor control nodes
* **CiA 301 / CANopen** — CAN bus profile for motor controllers
* **WIA-OMNI-API** — credential storage for user identity
* **WIA-ACCESSIBILITY** — user accommodation profiles
* **WIA-AIR-SHIELD** — transport hardening for cross-organisation telemetry

## Conformance levels

| Level | Required surfaces |
|-------|-------------------|
| Minimal | Phase 1 envelopes, Phase 2 telemetry stream + route dispatch |
| Core | Plus Phase 3 federation, Phase 4 ROS 2 bridge |
| Full | Plus WIA-AIR-SHIELD scoring, WIA-ACCESSIBILITY enforcement, CAN bus integration |

## License

MIT License — © 2025 WIA Standards Committee

弘益人間 — Benefit All Humanity.
