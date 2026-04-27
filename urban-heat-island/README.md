# WIA Urban Heat Island Standard

> Open standard for urban heat island monitoring, mitigation tracking, and cross-jurisdiction climate-adaptation coordination.

**Philosophy**: 弘益人間 — Benefit All Humanity

---

## Why this standard exists

Urban heat islands (UHI) — cities measurably hotter than their
surrounding rural areas due to dark surfaces, reduced vegetation,
and waste heat — are a growing public-health hazard. Heat waves
amplified by UHI killed thousands across Europe in 2003, 2019, and
2022; Pacific Northwest 2021; India and Pakistan 2022 and 2024.
Cities track UHI inconsistently: weather-station siting varies, urban
canopy data is fragmented across vendor satellite imagery, and
mitigation interventions (cool roofs, urban forestry, green
infrastructure) are tracked in city-specific GIS systems that do not
interoperate across jurisdictional boundaries.

WIA Urban Heat Island provides an open wire format and federation
protocol so that surface temperature observations, urban canopy
coverage, mitigation interventions, and public-health heat-event
records can flow across municipal, regional, and national
jurisdictions without forcing any single agency to be the central
database.

---

## 4-Phase architecture

| Phase | Scope |
|-------|-------|
| 1 — Data Format | Surface temperature observation, canopy assessment, mitigation intervention, heat-health event envelopes |
| 2 — API Interface | HTTP surface for publish, query, stream, federation handshake |
| 3 — Federation Protocol | Cross-jurisdiction handshake, replay defence, vulnerable-population audience controls |
| 4 — Integration | Landsat / Sentinel-3 thermal imagery, NOAA NWS heat advisories, EU Copernicus climate, USGS landcover, WIA family |

---

## Quick start

```bash
docker run -p 8080:8080 wia/urban-heat-island-host:1.0.0

# Subscribe to live surface temperature stream for a city
curl -N "http://localhost:8080/uhi/temp/stream?city_id=did:wia:city:portland-or" \
     -H "Accept: text/event-stream"
```

---

## CLI

A reference CLI ships under `cli/urban-heat-island.sh` with subcommands
`validate`, `temp`, `canopy`, `mitigation`, `health-event`, `info`.

---

## Companion standards

* **WMO** thermal climatology standards
* **Landsat / Sentinel-3** satellite thermal infrared products
* **NOAA NWS** heat advisory criteria
* **EU Copernicus** climate change service
* **USGS National Land Cover Database** for impervious surface mapping
* **WIA-OMNI-API** — credential storage for city agency identities
* **WIA Water Scarcity Response** — sister standard for drought + heat compounding
* **WIA-AIR-SHIELD** — transport hardening

---

## Conformance levels

| Level | Required surfaces |
|-------|-------------------|
| Minimal | Phase 1 envelopes, Phase 2 publish + query |
| Core | Plus Phase 3 federation, NOAA NWS heat advisory alignment |
| Full | Plus Landsat / Sentinel-3 thermal bridge, NLCD canopy bridge, EU Copernicus bridge |

---

MIT License — © 2025 WIA (World Certification Industry Association)

弘益人間 — Benefit All Humanity.
