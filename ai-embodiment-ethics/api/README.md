# WIA AI Embodiment Ethics — API SDK

Ethical-evaluation and consent tracking for embodied AI systems

This directory provides a publishable API contract and language SDKs for
implementations conforming to WIA standard `ai-embodiment-ethics`.

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/embodiment-ethics/v1)
  sdk/
    typescript/             # @wia/ai-embodiment-ethics — TypeScript SDK
    python/                 # wia-ai-embodiment-ethics  — Python SDK
```

## Resources

The contract defines REST collections for the following resources:

- `Agent` — `/agents`
- `EthicsCheck` — `/ethics-checks`
- `Consent` — `/consents`
- `Policy` — `/policies`


## Authentication

- `X-API-Key` header (preferred for service-to-service)
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — see repository root.

弘益人間 (홍익인간) · Benefit All Humanity
