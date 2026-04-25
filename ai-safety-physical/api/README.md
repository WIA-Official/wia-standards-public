# WIA AI Safety Physical — API SDK

Safety-zone definition and enforcement for physically-acting AI systems

This directory provides a publishable API contract and language SDKs for
implementations conforming to WIA standard `ai-safety-physical`.

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/ai-safety-physical/v1)
  sdk/
    typescript/             # @wia/ai-safety-physical — TypeScript SDK
    python/                 # wia-ai-safety-physical  — Python SDK
```

## Resources

The contract defines REST collections for the following resources:

- `SafetyZone` — `/safety-zones`
- `Hazard` — `/hazards`
- `Incident` — `/incidents`
- `EmergencyStop` — `/emergency-stops`


## Authentication

- `X-API-Key` header (preferred for service-to-service)
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — see repository root.

弘益人間 (홍익인간) · Benefit All Humanity
