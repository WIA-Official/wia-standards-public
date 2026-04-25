# WIA AI Robot Interface — API SDK

Unified REST + WSS interface for state, sensor streams, and capability discovery

This directory provides a publishable API contract and language SDKs for
implementations conforming to WIA standard `ai-robot-interface`.

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/ai-robot-interface/v1)
  sdk/
    typescript/             # @wia/ai-robot-interface — TypeScript SDK
    python/                 # wia-ai-robot-interface  — Python SDK
```

## Resources

The contract defines REST collections for the following resources:

- `Robot` — `/robots`
- `State` — `/states`
- `Sensor` — `/sensors`
- `Capability` — `/capabilities`


## Authentication

- `X-API-Key` header (preferred for service-to-service)
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — see repository root.

弘益人間 (홍익인간) · Benefit All Humanity
