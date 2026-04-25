# WIA Identity Management Standard

> **WIA-IDM-001 v1.0** — Identity Management for Interoperable Systems
> 弘益人間 — Benefit All Humanity

This standard defines the data formats, API surface, wire-level protocol, and integration patterns for interoperable identity management across WIA-conformant systems. It is the foundation reused by every WIA standard that needs to identify, authenticate, or authorize a subject.

The PHASE documents under `spec/` are the single normative source. Code under `api/`, `cli/`, and `simulator/` is informative reference material that demonstrates the contract; production implementations may diverge as long as they preserve the PHASE contract.

## Scope

Identity Management addresses three concerns:

1. **Authentication** — verifying that a subject is who they claim to be, including federated and decentralized identifiers.
2. **Authorization** — adjudicating what an authenticated subject is permitted to do, in a way that is auditable and consistent across deployments.
3. **Account lifecycle** — creating, updating, deactivating, and erasing identities, with provenance preserved for the deployment's conformance retention period.

The standard is transport-agnostic, technology-agnostic about the identifier format (it accepts URIs, DIDs, opaque strings), and explicitly cross-walks to the major published frameworks (see Annex B in each PHASE).

## Conformance tiers

WIA conformance for **identity-management** is evaluated across three tiers:

| Tier | Scope | Audit cadence |
|------|-------|---------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier 3 implementations MUST retain conformance evidence for at least seven years and MUST align with ISO/IEC 17065:2012 conformity-assessment requirements.

## Layout

```
standards/identity-management/
├── README.md            # this document
├── index.html           # human-readable landing page
├── simulator/           # interactive browser-based simulator
├── spec/                # PHASE-1..4 normative specifications
├── api/                 # reference TypeScript SDK skeleton
├── cli/                 # POSIX shell client
├── press/               # press kit (article + DALL·E prompts)
└── ebook/{en,ko}/       # eight-chapter ebook editions
```

## Cross-walk to published standards

The PHASE documents under `spec/` reference, but do not republish, the following frameworks:

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 7636 — Proof Key for Code Exchange (PKCE)
- IETF RFC 7644 — System for Cross-domain Identity Management (SCIM 2.0)
- IETF RFC 9457 — Problem Details for HTTP APIs
- W3C Decentralized Identifiers (DIDs) v1.0 — Working Group Recommendation
- W3C Verifiable Credentials Data Model v1.1 — Recommendation
- W3C PROV-DM — Provenance data model
- OpenID Connect Core 1.0 — OpenID Foundation

Implementers MUST obtain authoritative copies of the referenced documents directly from the issuing body. The cross-walk is informative only; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

## Open governance

Comments, proposals, and conformance reports are accepted via the GitHub issues tracker on the WIA-Official organization. Major version bumps follow the WIA Standards governance process documented at <https://wiastandards.com/governance>.

The standard is intentionally minimal at v1.0 so that deployments can start immediately. v1.1 and later minor versions will extend the canonical structures to cover additional concerns such as continuous-access evaluation, attribute-based access control, and decentralized identity wallet integrations.

## Related standards

`identity-management` is the foundation reused by:

- **wia-360** — when verifying who is taking an assessment.
- **wia-academy** — when authenticating learners and instructors.
- **wia-money** — when authorising financial transactions.
- **bci-consent** — when associating consent artifacts with a subject.
- **medical-research-data** — when linking research records to investigators.

Implementations that integrate `identity-management` with one of these adjacent standards SHOULD disclose the integration in their `/v1/health` payload's `wia_integrations` member.

## License

Licensed under the MIT license.

© 2026 WIA — World Certification Industry Association.
弘益人間 — Benefit All Humanity.
