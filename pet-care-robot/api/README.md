# WIA Pet Care Robot — API SDK

Companion-pet robotic care: feeding, monitoring, and welfare interaction

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/pet-care-robot/v1)
  sdk/typescript/           # @wia/pet-care-robot
  sdk/python/               # wia-pet-care-robot
```

## Resources

- `Robot` — `/robots`
- `Pet` — `/pets`
- `CareSession` — `/care-sessions`
- `Schedule` — `/schedules`

## Authentication

- `X-API-Key` header
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — 弘益人間 (홍익인간) Benefit All Humanity
