# 🟢 WIA Edible Algae CLI — WIA-AGRI-033

Command-line tool for managing Spirulina, Chlorella, seaweed, and microalgae
cultivation data conforming to the WIA Edible Algae standard.

## Installation

```bash
chmod +x edible-algae.sh
sudo ln -s "$(pwd)/edible-algae.sh" /usr/local/bin/edible-algae
```

Requires `curl` and `jq` (https://jqlang.org).

## Authentication

Set credentials via environment variables:

```bash
export WIA_EDIBLE_ALGAE_API_KEY="your-api-key"
export WIA_EDIBLE_ALGAE_URL="https://api.wia.live/edible-algae/v1"   # optional
```

## Resources

| Resource | Description |
|----------|-------------|
| `cultivars` | Algae strain catalog (Spirulina, Chlorella, Dunaliella, kelp, …) |
| `batches` | Cultivation batch records (volume, density, dates) |
| `harvests` | Harvest yield, biomass, and quality readings |
| `nutrient-profiles` | Nutritional composition (protein, lipids, vitamins, minerals) |
| `facilities` | Cultivation facility registrations (open-pond, photobioreactor, marine) |

## Examples

### List cultivars

```bash
edible-algae list cultivars --limit 50
```

### Fetch a single batch

```bash
edible-algae get batches batch-2026-04
```

### Submit a new harvest record

```bash
cat > harvest.json <<'JSON'
{
  "batchId": "batch-2026-04",
  "harvestedAt": "2026-04-25T10:00:00Z",
  "wetMassKg": 184.5,
  "drySolidsPct": 18.2,
  "qualityGrade": "A"
}
JSON

edible-algae create harvests harvest.json
```

### Validate before submission

```bash
edible-algae validate harvest.json
```

### Export to CSV

```bash
edible-algae export harvests csv > 2026-q2-harvests.csv
```

## Exit codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | Validation failed / record invalid |
| 2 | Usage error or missing dependency |

## Reference

- OpenAPI contract: `../api/openapi-3.0.yaml`
- TypeScript SDK: `../api/sdk/typescript/`
- Python SDK: `../api/sdk/python/`
- Spec: `../spec/PHASE-2-API-INTERFACE.md`

License: MIT — 弘益人間 (홍익인간) Benefit All Humanity
