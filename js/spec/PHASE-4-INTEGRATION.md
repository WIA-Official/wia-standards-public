# WIA-js PHASE 4 — Integration Specification

**Standard:** WIA-js
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how WIA-js integrates with adjacent
ecosystems (browser hosts, server runtimes, build tools,
package registries, vulnerability databases, supply-
chain attestation systems, and downstream WIA standards),
how conformance evidence is produced, and how
deployments are governed.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 17065:2012 (Conformity assessment)
- ISO/IEC 27001:2022, ISO/IEC 27034 (application security), ISO/IEC 22275:2018
- IETF RFC 7515 (JWS), RFC 9421 (HTTP Message Signatures)
- WHATWG HTML, Fetch, URL, Encoding, Streams Living Standards
- W3C Subresource Integrity, W3C Content Security Policy 3
- ECMA-262, ECMA-402, ECMA-404
- TC39 Process document
- npm Registry API, JSR HTTP API, Deno KV API (informative)
- WebAssembly Component Model

---

## §1 Scope

This PHASE covers integration points outside the
PHASE-1..3 scope: how WIA-js participants consume
upstream specifications (TC39 editions, WHATWG Living
Standards), how downstream WIA standards reference
WIA-js artifacts, and how conformance is assessed,
recorded, and audited.

## §2 Upstream integration

### 2.1 TC39

WIA-js tracks ECMA-262 editions on a four-year window.
The publication schedule is:

- Year N: ECMA-262 edition N is published (June).
- Year N+1: WIA-js edition N is published (Q1).
- Year N+5: WIA-js edition N is retired (Q4).

Stage 4 proposals from the TC39 process are presumed
required; Stage 3 proposals are recognised but not
required.

### 2.2 WHATWG / W3C

WHATWG Living Standards (HTML, Fetch, URL, Encoding,
Streams, Console) are consumed by reference. WIA-js
does not pin WHATWG snapshot dates; instead, the
runtime profile (PHASE-1 §4) declares which WHATWG
features are exposed.

W3C TR documents (Subresource Integrity, CSP, Web IDL)
are consumed at their latest Recommendation.

### 2.3 IETF

IETF RFCs are consumed at the dated revision; updates
follow RFC 9281 (`<rfc>`-versioning) practice when an
update is announced.

## §3 Build-tool integration

WIA-js does not mandate a specific build tool. The
integration contract is a CLI-level interface:

```
wia-js verify --manifest <path> --runtime <profile>
```

Implementations of this CLI SHOULD live alongside
existing tools (e.g. as `npm pkg verify`, `deno verify`,
`bun pm verify`) so that publishers do not need an
additional installation.

The CLI exits 0 on PASS, non-zero on any verifier
finding. The output stream is line-delimited
JSON-Lines, where each line is a Problem Details
fragment.

## §4 Browser host integration

Browser hosts consume WIA-js artifacts via the standard
HTML loading pipeline. The integration touchpoints are:

- `<script type="importmap">` for the import map
  declared in PHASE-1 §5;
- `integrity="<sha-256-or-512>"` on `<script type="module">`;
- `Content-Security-Policy` directives that mirror the
  capability declaration (PHASE-1 §6) — `script-src`,
  `connect-src`, `worker-src`;
- WebAssembly Component glue exposed via `import`
  attribute `{type: "wasm"}` per the Wasm Component
  Model.

## §5 Conformance

### 5.1 Evidence package

Every published artifact has a conformance evidence
package in the registry under
`/v1/conformance/{packageRef}/evidence`:

- the canonical SBOM (CycloneDX 1.5 or SPDX 2.3);
- the test-vector matrix produced by Annex G of each
  PHASE document;
- the OpenAPI documents declared in PHASE-2;
- the declared runtime profiles (PHASE-1 §4);
- the declared capability profile (PHASE-1 §6);
- a JWS-signed manifest binding the above to the
  publisher's signing key (RFC 7515).

### 5.2 Auditor responsibilities

An auditor under ISO/IEC 17065:2012 verifies the
evidence package:

1. SBOM signature valid and matches the publisher key;
2. Test-vector matrix passes against the declared
   ECMA-262 edition;
3. Capability declaration consistent with the source
   (no `eval` calls if `eval` is `forbid`);
4. Source-map files reachable and unredacted by
   `ignoreList` only at non-private boundaries.

### 5.3 Tier promotion

| Tier            | Auditor                              | Cadence      |
|-----------------|--------------------------------------|--------------|
| Self-declared   | none                                 | annual       |
| Verified        | independent third-party              | 24 months    |
| Anchored        | accredited body (ISO/IEC 17065)      | 12 months    |

A tier downgrade event MUST be reported to the registry
within 30 days; the registry tombstones the previous
attestation and surfaces the new one.

## §6 Cross-domain references (normative)

| Standard                       | Integration point                    |
|--------------------------------|--------------------------------------|
| WIA-language-bridge            | ECMA-402 locale negotiation          |
| WIA-prompts                    | embedded LLM prompt artifacts        |
| WIA-plugins                    | plugin-host runtime                  |
| WIA-pubscript                  | EPUB / Web Publication scripting     |
| WIA-multiverse-interface       | scene-graph script bindings          |

The WIA registry maintains a single source of truth for
each integration point; downstream PHASE documents
reference the registry record rather than embedding
duplicates.

## §7 Privacy

WIA-js artifacts are not personal data. Source-maps
MAY contain personal data (developer email addresses,
internal hostnames, customer-specific identifiers); the
publisher's privacy policy applies. Personal data
inadvertently shipped in `sourcesContent` is handled as
a vulnerability — the publisher MUST republish with the
data removed and tombstone the affected version.

## §8 Security

Vulnerability disclosure follows the publisher's
policy. The WIA registry surfaces OSV.dev advisory
joins (PHASE-2 §7.3) but does not enforce a download
gate. Critical CVEs in the registry's own
infrastructure are disclosed to publishers with a 5-day
embargo and to the public after 30 days.

## §9 Localization

Publisher metadata (description, keywords, repository
URL) is localised in BCP 47 form. The registry exposes
the localised metadata behind `Accept-Language`
negotiation; the canonical record carries every
declared locale.

## §10 Accessibility

The HTML landing page for every package
(`/v1/packages/{name}/{version}/landing`) MUST conform
to WCAG 2.2 AA. Generated documentation (TypeDoc,
JSDoc) is exempt unless the publisher explicitly
declares conformance.

## §11 Migration

A package transitioning between registries MAY publish
a redirect record:

```json
{
  "from": "https://registry.a.example/@scope/name",
  "to":   "https://registry.b.example/@scope/name",
  "since": "2026-04-28",
  "tombstoned": false
}
```

Both registries MUST honour the redirect for at least
one ECMA-262 edition window.

## §12 Sustainability

Build artifacts SHOULD declare an estimated cold-start
energy cost (joules) and the methodology used to
measure it. This is informative; the registry
aggregates declarations into a per-runtime profile
median for ecosystem visibility.

## Annex A — Conformance disclosure

Implementations link to the conformance evidence URL
from their package landing page and from the README
under a `## Conformance` heading. The link uses
`rel="conformance"`.

## Annex B — Worked evidence package (informative)

```json
{
  "packageRef": "f63f4f04-9a76-4d2c-9f53-2c4d09a1bbb1",
  "ecmaEdition": "es2024",
  "sbom": "https://registry.../sboms/sha512-...",
  "testMatrix": "https://registry.../tests/matrix-2026-04-28.json",
  "manifestSig": "eyJhbGciOiJFUzI1NiIsImtpZCI6Imt..."
}
```

## Annex C — Versioning

The version of this PHASE is 1.0. A minor bump tracks
the addition of new integration touchpoints; a major
bump indicates a backwards-incompatible change to the
conformance evidence schema.

## Annex D — Open governance

Issues, errata, and proposals are tracked at
`github.com/WIA-Official/wia-standards/issues` with the
`js` label. The WIA Standards working group reviews
open issues at the start of every minor release cycle
and publishes the resulting decision log alongside the
release notes.

## Annex E — Withdrawal procedure

A publisher may withdraw an artifact by submitting a
tombstone (PHASE-3 §11). Tombstones are immutable;
re-publication requires a new version (semver patch or
higher).

## Annex F — Reproducibility

The conformance evidence is reproducible from publicly
available inputs: the manifest, the archive, and the
runtime profile. Reproduction harnesses are published
under `/v1/conformance/repro` for every supported
runtime profile.

## Annex G — Test vectors

Every normative requirement in PHASE-1..3 has at least
one positive vector and one negative vector under
`tests/phase-vectors/`. Implementations claiming
conformance MUST run all vectors in CI and publish the
resulting pass/fail matrix in their compliance package.

The vector matrix is itself versioned with Semantic
Versioning 2.0.0 so that auditors can pin a specific
matrix release for reproducible audits.

## Annex H — WebAssembly Component Model interop

WIA-js hosts that load WebAssembly Components MUST
honour the Component Model's `wit-bindgen`-generated
JS shims. Capability declarations (PHASE-1 §6) extend
to imported WIT interfaces — a Component that imports
`wasi:filesystem` is treated as if its host JS module
declared `filesystem: read-write` with the path scope
declared in the WIT world.

The Component's `.wit` is published alongside the
package archive and is signed under the same key set
as the manifest.

## Annex I — Tooling matrix

| Surface          | Tool example       | Status                |
|------------------|--------------------|-----------------------|
| Node             | `npm pkg verify`   | reference, MIT        |
| Deno             | `deno verify`      | reference, MIT        |
| Bun              | `bun pm verify`    | reference, MIT        |
| Browser bundler  | `wia-js bundler`   | reference, Apache-2.0 |
| CI               | `wia-js gate`      | reference, Apache-2.0 |

Reference tools are mirrored in the WIA Standards
GitHub organisation under the `wia-js-tools` umbrella
repo. Third-party tools are tracked in a community
catalogue at the same URL, with the publisher's
self-asserted conformance tier next to each entry.

## Annex J — Risk register

| Risk                                  | Mitigation                |
|---------------------------------------|---------------------------|
| Unsigned manifest accepted            | Reject; require JWS sig   |
| Tarball digest mismatch               | Reject install            |
| Capability over-declaration           | Static analysis check     |
| Stale SBOM                            | Re-publish quarterly      |
| Federation peer compromise            | Tombstone + JWKS rotate   |
| Source-map data leak                  | Republish + tombstone     |
| TLS downgrade attempt                 | HSTS preload mandatory    |
| Replay of signed publish              | `iat`/`exp` ≤ 24h         |

The risk register is reviewed at every minor version
of this PHASE. Reviews are minuted and published
alongside the release notes.

## Annex K — Deprecation and removal

A normative requirement that becomes obsolete enters a
12-month deprecation window. During the window the
requirement is marked `Deprecated` in the registry's
requirement catalogue and downgrade events are
informative, not gating. After 12 months the
requirement is removed; conformance evidence packages
no longer need to satisfy it.

A backward-incompatible change to an existing
normative requirement requires a major bump to the
PHASE document, a 90-day overlap window, and an entry
in the WIA Standards working group decision log
referenced by URL from the version-history table.

弘益人間 (Hongik Ingan) — Benefit All Humanity
