# `insect-protein` CLI

Minimal POSIX shell client demonstrating the WIA-AGRI-025 *Insect Protein Standard* request/response contract. Useful for ad‑hoc audits, smoke tests against a Tier 1 deployment, and continuous‑integration probes.

## Requirements

- `bash` (any modern POSIX shell)
- `curl`
- `jq`

## Configuration

```sh
export WIA_API_BASE="https://api.wia-standards.org/v1/insect-protein"
export WIA_API_KEY="EXAMPLE_API_KEY_REPLACE_ME"
# optional
export WIA_API_TIMEOUT=30
export WIA_API_DEBUG=1
```

## Subcommands

```text
list-farms          [--country CC] [--species NAME]
get-farm            --farm-id FARM_ID
list-batches        [--species NAME] [--from ISO8601] [--to ISO8601]
get-batch           --batch-id BATCH_ID
submit-safety-test  --batch-id BATCH_ID --file PATH/TO/REPORT.json
list-audit          [--since ISO8601] [--limit N]
whoami
```

## Examples

List every active mealworm farm in Korea:

```sh
bash insect-protein.sh list-farms --country KR --species "Tenebrio molitor"
```

Pull the most recent month of cricket batches:

```sh
bash insect-protein.sh list-batches \
  --species "Acheta domesticus" \
  --from   "2025-12-01T00:00:00Z" \
  --to     "2025-12-31T23:59:59Z"
```

Submit a heavy‑metal & microbiological safety report (Tier 2 evidence) against a known batch:

```sh
bash insect-protein.sh submit-safety-test \
  --batch-id "BSF-2025-Q4-0042" \
  --file     ./reports/heavy-metals.json
```

The CLI validates that the report file parses as JSON before sending; the API contract (PHASE‑2) defines the canonical schema for the payload.

## Exit codes

| Code | Meaning |
|------|---------|
| 0    | success |
| 2    | usage / unknown flag |
| 3    | `WIA_API_KEY` not set |
| 4    | local JSON validation failed |

## Notes

- The CLI is illustrative; it is *not* a substitute for the typed TypeScript SDK in `../api/typescript/` for production integrations.
- Authenticated calls require a bearer token issued under your WIA conformance tier.
