# WIA Pet Welfare Global — API SDK

Global companion-animal welfare indicators and incident reporting

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/pet-welfare-global/v1)
  sdk/typescript/           # @wia/pet-welfare-global
  sdk/python/               # wia-pet-welfare-global
```

## Resources

- `Region` — `/regions`
- `WelfareIndicator` — `/welfare-indicators`
- `Incident` — `/incidents`
- `Organization` — `/organizations`

## Authentication

- `X-API-Key` header
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — 弘益人間 (홍익인간) Benefit All Humanity
