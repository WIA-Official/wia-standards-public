# WIA AI-City

> An open standard for AI-driven smart-city services with explicit consent, transparency, and conformance-checkable APIs.

홍익인간 (弘益人間) - Benefit All Humanity

## Overview

The WIA AI-City standard provides protocols for an open standard for ai-driven smart-city services with explicit consent, transparency, and conformance-checkable apis.

## Key Features

- ✅ Open data formats with JSON Schema validation
- ✅ OpenAPI 3.0 REST contract + TypeScript and Python SDKs
- ✅ Command-line conformance tooling
- ✅ EN + KO long-form documentation
- ✅ MIT License — implement freely



## Standard scope

An open standard for AI-driven smart-city services with explicit consent, transparency, and conformance-checkable APIs.

## Conformance levels

| Level | Description |
|-------|-------------|
| **Conformant** | Implementation publishes valid records against the OpenAPI contract in `api/openapi-3.0.yaml`. |
| **Audited** | Implementation passes all WIA conformance tests in `cli/wia-ai-city.sh validate`. |
| **Certified** | Implementation has registered with WIA at https://cert.wiastandards.com and exposes a public credential. |

## Repository layout

```
wia-ai-city/
├── README.md          ← you are here
├── index.html         ← landing page (EN/KO toggle, dark theme)
├── simulator/         ← interactive simulator
├── spec/              ← Phase 1–4 specifications
├── api/               ← OpenAPI 3.0 contract + TS/Python SDKs
├── cli/               ← wia-ai-city command-line tool
├── ebook/             ← long-form documentation (EN + KO)
└── press/             ← editorial articles + DALLE prompts
```

## Quick start

```bash
# Install the CLI
chmod +x cli/wia-ai-city.sh
sudo ln -s "$(pwd)/cli/wia-ai-city.sh" /usr/local/bin/wia-ai-city

# Configure
export WIA_WIA_AI_CITY_API_KEY="your-api-key"
```

## Governance

- All citations follow [`docs/CITATION-POLICY.md`](../../docs/CITATION-POLICY.md): only verifiable primary sources (ISO, IEC, IEEE, RFC, W3C, HL7) appear in body and spec.
- The standard graduates to **Deep Published v3** only when `validate-published-v3.sh` passes 21/21 (17 size/structure + 4 veracity gates).
- Implementations seeking external citation MUST reference the Deep Published version of the standard.

## License

MIT — see repository root.

弘益人間 (홍익인간) · Benefit All Humanity
