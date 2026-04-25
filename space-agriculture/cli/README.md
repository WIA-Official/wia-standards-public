# 🌱 WIA Space Agriculture CLI

Command-line tool conforming to the WIA Space Agriculture standard.

## Installation

```bash
chmod +x space-agriculture.sh
sudo ln -s "$(pwd)/space-agriculture.sh" /usr/local/bin/space-agriculture
```

Requires `curl` and `jq` (https://jqlang.org).

## Authentication

```bash
export WIA_SPACE_AGRICULTURE_API_KEY="your-api-key"
export WIA_SPACE_AGRICULTURE_URL="https://api.wia.live/space-agriculture/v1"
```

## Resources

| Resource | Description |
|----------|-------------|
| `habitats` | Orbital, lunar, or Mars-base growth habitats |
| `crops` | Plant cultivars adapted to micro-/low-gravity |
| `growth-runs` | Time-series growth/yield records |
| `environment-readings` | Light, atmosphere, water, radiation samples |
| `anomalies` | Pathology, stress, or system-failure events |

## Examples

```bash
space-agriculture list habitats --limit 100
space-agriculture get habitats <id>
space-agriculture create crops record.json
space-agriculture validate record.json
space-agriculture export habitats csv > report.csv
```

## Reference

- OpenAPI contract: `../api/openapi-3.0.yaml`
- Spec: `../spec/PHASE-2-API-INTERFACE.md`

License: MIT — 弘益人間 (홍익인간) Benefit All Humanity
