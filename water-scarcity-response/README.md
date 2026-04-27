# WIA Water Scarcity Response Standard

> Open standard for cross-jurisdiction water scarcity coordination — drought monitoring, allocation rules, demand-side management, emergency response.

**Philosophy**: 弘益人間 — Benefit All Humanity

---

## Why this standard exists

Water scarcity is a watershed-level problem with national and
sub-national jurisdictional boundaries that rarely align with the
hydrological boundaries of the affected basin. The Colorado River
basin spans seven US states plus Mexico; the Mekong spans six
nations; the Aral Sea catchment spans five Central Asian republics.
Each jurisdiction tracks its own gauges, allocations, restrictions,
and emergency responses in formats that do not interoperate. When a
multi-year drought arrives, the affected populations cannot get a
unified picture of basin-wide condition, allocation history, or
ongoing response measures.

WIA Water Scarcity Response provides an open wire format and
federation protocol so that drought monitoring, water allocation
records, demand-side management measures, and emergency response
declarations can flow across jurisdictional boundaries without
forcing any single agency to be the central database.

---

## 4-Phase architecture

| Phase | Scope |
|-------|-------|
| 1 — Data Format | Drought index observation, allocation record, restriction declaration, emergency response, supply augmentation envelopes |
| 2 — API Interface | HTTP surface for publish, query, stream, federation handshake |
| 3 — Federation Protocol | Cross-jurisdiction handshake, replay defence, public consultation consent |
| 4 — Integration | WMO drought indices (SPI, SPEI, PDSI), USGS National Water Information System, EU WISE-WFD, FAO AquaStat, WIA family |

---

## Quick start

```bash
docker run -p 8080:8080 wia/water-scarcity-response-host:1.0.0

# Subscribe to drought index stream for a basin
curl -N "http://localhost:8080/wsr/drought/stream?basin_id=colorado-river-basin" \
     -H "Accept: text/event-stream"
```

---

## CLI

A reference CLI ships under `cli/water-scarcity-response.sh` with
subcommands `validate`, `drought`, `allocation`, `restriction`,
`emergency`, `info`.

---

## Companion standards

* **WMO drought-index standards** — SPI, SPEI, PDSI definitions
* **USGS National Water Information System** — US gauge data backend
* **EU WISE-WFD** — European Water Framework Directive reporting
* **FAO AquaStat** — global water resources statistics
* **WIA-OMNI-API** — credential storage for agency identities
* **WIA-AIR-SHIELD** — transport hardening for cross-organisation federation

---

## Conformance levels

| Level | Required surfaces |
|-------|-------------------|
| Minimal | Phase 1 envelopes, Phase 2 publish + query |
| Core | Plus Phase 3 federation, WMO drought-index vocabulary alignment |
| Full | Plus USGS NWIS bridge, EU WISE-WFD bridge, FAO AquaStat bridge |

---

MIT License — © 2025 WIA (World Certification Industry Association)

弘益人間 — Benefit All Humanity.
