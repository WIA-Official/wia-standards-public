# WIA-collaborative-robot Collaborative Robot

> Cross-vendor data and protocol layer for Collaborative Robot.

## Overview

WIA-collaborative-robot defines standardized data formats and protocol semantics for
Collaborative Robot. The standard exposes four PHASE documents covering data
format, API interface, protocol, and integration concerns.

## Documentation

- spec/PHASE-1-DATA-FORMAT.md — schemas
- spec/PHASE-2-API-INTERFACE.md — REST + streaming APIs
- spec/PHASE-3-PROTOCOL.md — message protocol
- spec/PHASE-4-INTEGRATION.md — integration patterns

## Conformance Tiers

| Tier      | Scope                                                    |
|-----------|----------------------------------------------------------|
| Surface   | data formats accepted; self-attested                     |
| Verified  | annual third-party audit                                 |
| Anchored  | continuous evidence package per Annex G                  |

## Reference Standards

This standard cites only ALLOW sources per the WIA citation policy:
ISO/IEC, IEEE, RFC, W3C. Schemas use JSON Schema 2020-12 and OpenAPI
3.1. Signatures use Sigstore (DSSE envelope, Rekor transparency log).

## License

MIT License — © 2025 WIA Standards Committee

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Quick Start

```bash
npm install @wia/collaborative-robot
```

## Reference Standards

- ISO 10218-1:2011 — Robots and robotic devices: Safety requirements for industrial robots
- ISO/TS 15066:2016 — Robots and robotic devices: Collaborative robots
- IEC 61508 — Functional safety
- ISO 13482:2014 — Personal-care robot safety
- ROS 2 Humble (Open Robotics)
- OpenAPI Specification 3.1, JSON Schema 2020-12
- Sigstore (DSSE envelope, Rekor transparency log)

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Conformance Tiers

| Tier      | Scope                                                    | Audit cadence |
|-----------|----------------------------------------------------------|---------------|
| Surface   | data formats accepted; no formal audit                   | self-attested |
| Verified  | annual third-party audit                                 | annual        |
| Anchored  | continuous evidence package per Annex G                  | continuous    |
