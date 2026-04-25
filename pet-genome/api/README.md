# WIA Pet Genome — API SDK

Companion-animal genome data submission and breed/disease screening

## Layout

```
api/
  openapi-3.0.yaml          # OpenAPI 3.0.3 contract (production: https://api.wia.live/pet-genome/v1)
  sdk/typescript/           # @wia/pet-genome
  sdk/python/               # wia-pet-genome
```

## Resources

- `Sample` — `/samples`
- `Sequence` — `/sequences`
- `BreedReport` — `/breed-reports`
- `HealthMarker` — `/health-markers`

## Authentication

- `X-API-Key` header
- OAuth 2.0 authorization-code flow at `https://auth.wia.live/oauth/authorize`

## License

MIT — 弘益人間 (홍익인간) Benefit All Humanity
