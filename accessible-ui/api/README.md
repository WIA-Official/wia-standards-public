# WIA Accessible UI — API SDK

WCAG 2.1 AAA accessibility validation, contrast checking, ARIA management

This directory provides a publishable API contract and language SDKs for
implementations conforming to WIA standard `accessible-ui`.

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/accessible-ui/v1)
  sdk/
    typescript/             # @wia/accessible-ui — TypeScript SDK
    python/                 # wia-accessible-ui  — Python SDK
```

## Resources

The contract defines REST collections for the following resources:

- `Audit` — `/audits`
- `Violation` — `/violations`
- `Component` — `/components`
- `ContrastCheck` — `/contrast-checks`


## Authentication

- `X-API-Key` header (preferred for service-to-service)
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — see repository root.

弘益人間 (홍익인간) · Benefit All Humanity
