# WIA Permafrost Protection — API SDK

Permafrost monitoring station registration, thermal observations, and degradation alerts

This directory provides a publishable API contract and language SDKs for
implementations conforming to WIA standard `permafrost-protection`.

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/permafrost/v1)
  sdk/
    typescript/             # @wia/permafrost-protection — TypeScript SDK
    python/                 # wia-permafrost-protection  — Python SDK
```

## Resources

The contract defines REST collections for the following resources:

- `Station` — `/stations`
- `Observation` — `/observations`
- `Alert` — `/alerts`
- `Borehole` — `/boreholes`


## Authentication

- `X-API-Key` header (preferred for service-to-service)
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — see repository root.

弘益人間 (홍익인간) · Benefit All Humanity
