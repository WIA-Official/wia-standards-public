# WIA Agricultural Robot — API SDK

Autonomous agricultural-robot fleet registration, telemetry, and task assignment

This directory provides a publishable API contract and language SDKs for
implementations conforming to WIA standard `agricultural-robot`.

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/agricultural-robot/v1)
  sdk/
    typescript/             # @wia/agricultural-robot — TypeScript SDK
    python/                 # wia-agricultural-robot  — Python SDK
```

## Resources

The contract defines REST collections for the following resources:

- `Robot` — `/robots`
- `Telemetry` — `/telemetry`
- `Task` — `/tasks`
- `FieldMap` — `/field-maps`


## Authentication

- `X-API-Key` header (preferred for service-to-service)
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — see repository root.

弘益人間 (홍익인간) · Benefit All Humanity
