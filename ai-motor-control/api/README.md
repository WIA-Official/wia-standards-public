# WIA AI Motor Control — API SDK

Motor registration, command issuance, and trajectory tracking for AI-driven actuators

This directory provides a publishable API contract and language SDKs for
implementations conforming to WIA standard `ai-motor-control`.

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/motor-control/v1)
  sdk/
    typescript/             # @wia/ai-motor-control — TypeScript SDK
    python/                 # wia-ai-motor-control  — Python SDK
```

## Resources

The contract defines REST collections for the following resources:

- `Motor` — `/motors`
- `Command` — `/commands`
- `Trajectory` — `/trajectories`
- `Calibration` — `/calibrations`


## Authentication

- `X-API-Key` header (preferred for service-to-service)
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — see repository root.

弘益人間 (홍익인간) · Benefit All Humanity
