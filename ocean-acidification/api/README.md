# WIA Ocean Acidification — API SDK

Seawater pH/carbonate-chemistry station registration and observation submission

This directory provides a publishable API contract and language SDKs for
implementations conforming to WIA standard `ocean-acidification`.

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/ocean-acidification/v1)
  sdk/
    typescript/             # @wia/ocean-acidification — TypeScript SDK
    python/                 # wia-ocean-acidification  — Python SDK
```

## Resources

The contract defines REST collections for the following resources:

- `Station` — `/stations`
- `Observation` — `/observations`
- `Trend` — `/trends`
- `Alert` — `/alerts`


## Authentication

- `X-API-Key` header (preferred for service-to-service)
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — see repository root.

弘益人間 (홍익인간) · Benefit All Humanity
