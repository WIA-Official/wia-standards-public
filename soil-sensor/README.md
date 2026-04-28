# WIA-ENV-016 — Soil Sensor

> 토양 센서 표준 — 토양 수분·EC·온도·pH 센서 · IoT · 농업·환경 모니터링

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
soil sensor hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO 11074:2015 (Soil quality vocabulary)
- ISO 17025 (Lab competence)
- ISO/TS 17892 series (Geotechnical lab tests)
- ASTM D6913 / D7928 (Particle-size)
- ASTM D2974 (Moisture content)
- ISO 11277:2009 (Particle-size distribution)
- IEC 62443 (Industrial cybersecurity)
- ISO/IEC 30141:2018 (IoT RA)
- OGC SensorThings API 1.1

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Volumetric Water Content Sensors
2. Soil EC and Salinity
3. Soil Temperature Profiling
4. pH and Nutrient Sensors
5. Wireless Sensor Networks
6. OGC SensorThings API
7. Sensor Cybersecurity per IEC 62443
8. Calibration and QA per ISO 17025

## CLI

A POSIX shell helper is published at `cli/soil-sensor.sh`. The helper ships
sample envelope generators with no dependencies beyond `jq` and POSIX
shell so integrators can exercise the contract before wiring real
backends.

## Reference simulator

A browser-based reference simulator is published at `simulator/index.html`.
The simulator surfaces every Phase 1 envelope and walks every Phase 2
endpoint exchange so adopters can study the contract behaviour before
integrating with a real backend.

## Conformance

A standard is conformant when:

1. Every Phase 1 envelope it emits validates against the published
   JSON Schema for that envelope class
2. Every Phase 2 endpoint it exposes honours the documented status
   codes, content shapes, and error envelopes
3. Every Phase 3 protocol exchange it participates in honours the
   handshake order, signature requirements, and audit hook contract
4. Phase 4 ecosystem composition reaches the required cross-standard
   capabilities (audit transport, identity, federation) per the
   Phase 4 §4 capability matrix

## Governance

Maintained under the WIA Standards public-benefit governance model.
See `https://wiastandards.com/governance/` for committee composition,
voting rules, and the per-standard release calendar.

弘益人間 — Benefit All Humanity.
