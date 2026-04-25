# 🏙️ WIA AI-City CLI

Command-line tool conforming to the WIA AI-City standard.

## Installation

```bash
chmod +x wia-ai-city.sh
sudo ln -s "$(pwd)/wia-ai-city.sh" /usr/local/bin/wia-ai-city
```

Requires `curl` and `jq` (https://jqlang.org).

## Authentication

```bash
export WIA_AI_CITY_API_KEY="your-api-key"
export WIA_AI_CITY_URL="https://api.wia.live/ai-city/v1"
```

## Resources

| Resource | Description |
|----------|-------------|
| `citizens` | Anonymized citizen identity envelopes |
| `services` | Municipal AI service registrations |
| `incidents` | Public-safety and infrastructure incidents |
| `sensor-streams` | City-scale sensor stream registrations |
| `policies` | AI usage and consent policies |

## Examples

```bash
wia-ai-city list citizens --limit 100
wia-ai-city get citizens <id>
wia-ai-city create services record.json
wia-ai-city validate record.json
wia-ai-city export citizens csv > report.csv
```

## Reference

- OpenAPI contract: `../api/openapi-3.0.yaml`
- Spec: `../spec/PHASE-2-API-INTERFACE.md`

License: MIT — 弘益人間 (홍익인간) Benefit All Humanity
