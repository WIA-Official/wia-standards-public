# 🥗 WIA Food Waste Reduction CLI

Command-line tool conforming to the WIA Food Waste Reduction standard.

## Installation

```bash
chmod +x food-waste-reduction.sh
sudo ln -s "$(pwd)/food-waste-reduction.sh" /usr/local/bin/food-waste-reduction
```

Requires `curl` and `jq` (https://jqlang.org).

## Authentication

```bash
export WIA_FOOD_WASTE_REDUCTION_API_KEY="your-api-key"
export WIA_FOOD_WASTE_REDUCTION_URL="https://api.wia.live/food-waste-reduction/v1"
```

## Resources

| Resource | Description |
|----------|-------------|
| `waste-records` | Per-batch food-waste log entries |
| `redistributions` | Donations and redistribution events |
| `audits` | Facility waste-stream audits |
| `targets` | Reduction targets and KPIs |
| `facilities` | Participating sites (cafeteria, retail, processor) |

## Examples

```bash
food-waste-reduction list waste-records --limit 100
food-waste-reduction get waste-records <id>
food-waste-reduction create redistributions record.json
food-waste-reduction validate record.json
food-waste-reduction export waste-records csv > report.csv
```

## Reference

- OpenAPI contract: `../api/openapi-3.0.yaml`
- Spec: `../spec/PHASE-2-API-INTERFACE.md`

License: MIT — 弘益人間 (홍익인간) Benefit All Humanity
