# 🌿 WIA Urban Farming CLI

Command-line tool conforming to the WIA Urban Farming standard.

## Installation

```bash
chmod +x urban-farming.sh
sudo ln -s "$(pwd)/urban-farming.sh" /usr/local/bin/urban-farming
```

Requires `curl` and `jq` (https://jqlang.org).

## Authentication

```bash
export WIA_URBAN_FARMING_API_KEY="your-api-key"
export WIA_URBAN_FARMING_URL="https://api.wia.live/urban-farming/v1"
```

## Resources

| Resource | Description |
|----------|-------------|
| `farms` | Rooftop, vertical, or community farm registrations |
| `crops` | Crop tracking with location and lifecycle |
| `harvests` | Yield records per site |
| `water-runs` | Irrigation and recirculation logs |
| `market-events` | Local distribution and CSA events |

## Examples

```bash
urban-farming list farms --limit 100
urban-farming get farms <id>
urban-farming create crops record.json
urban-farming validate record.json
urban-farming export farms csv > report.csv
```

## Reference

- OpenAPI contract: `../api/openapi-3.0.yaml`
- Spec: `../spec/PHASE-2-API-INTERFACE.md`

License: MIT — 弘益人間 (홍익인간) Benefit All Humanity
