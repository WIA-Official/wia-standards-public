# `food-crisis-response` CLI

Minimal POSIX shell client that demonstrates the WIA `food-crisis-response` standard's
request/response contract.

## Requirements

- `bash` (any modern POSIX shell will do)
- `curl`
- `jq`

## Usage

```sh
export WIA_API_BASE="https://food-crisis-response.example.com/api/v1"
export WIA_API_TOKEN="<your-bearer-token>"
./food-crisis-response.sh list
./food-crisis-response.sh get <id>
./food-crisis-response.sh create payload.json
./food-crisis-response.sh validate payload.json
./food-crisis-response.sh export <id>
```

This script is **reference material only** — it demonstrates the wire
contract and is not intended for production use. Production deployments
SHOULD use the TypeScript SDK skeleton under `../api/` or a vendor
library that targets the same OpenAPI surface.

## Conformance

The CLI subcommands map onto the resource endpoints documented in
`../spec/PHASE-2-API.md` (or the equivalent SDK/Client API document for
this standard). Behaviour follows the IETF RFC 9457 problem-details
error model when the backend returns non-2xx responses.

---

© 2026 WIA — World Certification Industry Association. Licensed under MIT.
