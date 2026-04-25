# WIA Pet Emotion — API SDK

Pet emotion detection and behavioral state classification

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/pet-emotion/v1)
  sdk/typescript/           # @wia/pet-emotion
  sdk/python/               # wia-pet-emotion
```

## Resources

- `Pet` — `/pets`
- `EmotionReading` — `/emotion-readings`
- `BehaviorEvent` — `/behavior-events`
- `Model` — `/models`

## Authentication

- `X-API-Key` header
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — 弘益人間 (홍익인간) Benefit All Humanity
