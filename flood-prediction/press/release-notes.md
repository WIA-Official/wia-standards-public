# WIA-ENE-033 Flood Prediction — Release Notes (1.0)

**Status:** Stable
**Release Date:** 2026-04-26
**Sponsor:** WIA Standards working group, climate-resilience track
**Maintainers:** WIA Standards Editorial Team

---

## Overview

WIA-ENE-033 Flood Prediction defines a cross-vendor data and protocol
layer for hydrological observation, ensemble flood forecasting,
warning publication, and downstream alert distribution. The standard
is interoperable with WMO HOMS, OGC WaterML 2.0, and the OASIS Common
Alerting Protocol 1.2.

## What's in this release

- **PHASE-1 Data Format** — gauge identification, river-network
  topology, ensemble-forecast records, warning-event schemas.
- **PHASE-2 API Interface** — REST API for gauge ingestion, forecast
  retrieval, warning publication, simulation submission, and
  evacuation-route query. CAP 1.2 emission for downstream public
  warning systems.
- **PHASE-3 Protocol** — message types for gauge → forecasting node →
  warning publisher → downstream consumer pipelines.
- **PHASE-4 Integration** — regulatory crosswalks (national disaster-
  management acts, Sendai Framework reporting), CycloneDX 1.5 SBOMs,
  in-toto Attestation 1.0 + Sigstore evidence packages.

## Conformance

Implementations follow the three-tier conformance model documented in
the README and Annex A: Surface (data formats), Verified (annual
third-party audit), Anchored (continuous evidence package signed
with Sigstore).

## Known limitations

- Subsurface flood-inundation models that depend on lidar-derived DEMs
  with horizontal resolution finer than 1 m are out of scope of v1.0.
- Storm-surge coupling is documented as a roadmap item for v1.1.

## Press contact

press@wiastandards.com — WIA Standards Editorial Team

弘益人間 (Hongik Ingan) — Benefit All Humanity
