# WIA AI Sensor Fusion — API SDK

Multi-sensor fusion pipeline registration and fused-state retrieval

This directory provides a publishable API contract and language SDKs for
implementations conforming to WIA standard `ai-sensor-fusion`.

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/sensor-fusion/v1)
  sdk/
    typescript/             # @wia/ai-sensor-fusion — TypeScript SDK
    python/                 # wia-ai-sensor-fusion  — Python SDK
```

## Resources

The contract defines REST collections for the following resources:

- `Sensor` — `/sensors`
- `FusionPipeline` — `/pipelines`
- `FusedState` — `/fused-states`
- `Calibration` — `/calibrations`


## Authentication

- `X-API-Key` header (preferred for service-to-service)
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — see repository root.

弘益人間 (홍익인간) · Benefit All Humanity
