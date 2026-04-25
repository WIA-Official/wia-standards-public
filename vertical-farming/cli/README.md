# 🥬 WIA Vertical Farming CLI

Command-line tool for managing indoor vertical-farming environmental control,
nutrient runs, crop tracking, and energy monitoring data conforming to the WIA
Vertical Farming standard.

## Installation

```bash
chmod +x vertical-farming.sh
sudo ln -s "$(pwd)/vertical-farming.sh" /usr/local/bin/vertical-farming
```

Requires `curl` and `jq` (https://jqlang.org).

## Authentication

```bash
export WIA_VERTICAL_FARMING_API_KEY="your-api-key"
export WIA_VERTICAL_FARMING_URL="https://api.wia.live/vertical-farming/v1"
```

## Resources

| Resource | Description |
|----------|-------------|
| `facilities` | Vertical-farming facility registrations |
| `layers` | Multi-tier rack/layer definitions |
| `crops` | Crop growth records (lettuce, basil, strawberry, microgreens) |
| `environment-readings` | Temperature, humidity, CO₂, PPFD, EC, pH samples |
| `nutrient-runs` | Hydroponic / aeroponic nutrient solution runs |
| `energy-meters` | Power consumption per zone |

## Examples

### Submit an environment reading

```bash
cat > reading.json <<'JSON'
{
  "facilityId": "fac-seoul-01",
  "layerId": "layer-tower-3-shelf-7",
  "timestamp": "2026-04-25T10:30:00Z",
  "temperatureC": 22.4,
  "humidityPct": 68.2,
  "co2Ppm": 850,
  "ppfdUmol": 250,
  "ec": 2.1,
  "ph": 5.9
}
JSON

vertical-farming create environment-readings reading.json
```

### List active crops

```bash
vertical-farming list crops --limit 100
```

### Export nutrient-run history

```bash
vertical-farming export nutrient-runs csv > 2026-q2-nutrient-runs.csv
```

## Exit codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | Validation failed |
| 2 | Usage error or missing dependency |

## Reference

- OpenAPI contract: `../api/openapi-3.0.yaml`
- TypeScript SDK: `../api/typescript/` (existing)
- Spec: `../spec/PHASE-2-API-INTERFACE.md`

License: MIT — 弘益人間 (홍익인간) Benefit All Humanity
