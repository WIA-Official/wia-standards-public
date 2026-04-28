# WIA-js PHASE 1 — Data Format Specification

**Standard:** WIA-js
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for WIA-js,
the JavaScript / ECMAScript runtime and package
interoperability standard. The records bind every
deployable JavaScript artifact to its source ECMAScript
edition, its module loader, its runtime profile (browser,
Node.js, Deno, Bun, Cloudflare Workers, edge),
its package manifest, and its capability and policy
declarations so that downstream consumers can reason
about the artifact without reading the source.

References (CITATION-POLICY ALLOW only):
- Ecma International ECMA-262 (ECMAScript Language Specification, 15th edition or later)
- Ecma International ECMA-402 (Internationalization API)
- Ecma International ECMA-404 (JSON)
- Ecma International TC39 process — Stages 0–4 advancement
- ISO/IEC 22275:2018 (ECMAScript edition 5.1 reaffirmation)
- WHATWG HTML, URL, Encoding, Fetch, Streams Living Standards
- W3C Web IDL, W3C ECMAScript Modules
- IETF RFC 8259 (JSON), RFC 8785 (JSON Canonicalisation Scheme), RFC 8927 (JSON Type Definition)
- IETF RFC 6454 (Web Origin), RFC 7515 (JWS), RFC 4122 (UUID)
- IEEE 754:2019 (Floating-Point Arithmetic — Number representation)
- Unicode Standard 15.1, UTS #18 (Regular Expressions)
- npm package.json schema, Semantic Versioning 2.0.0
- JSR (JavaScript Registry) manifest, deno.json
- WebAssembly Core Specification 2.0 and Component Model

---

## §1 Scope

This PHASE applies to artifacts whose execution semantics
are governed by ECMA-262 and whose identity is expressed
through a manifest. The artifact may be a browser bundle,
a Node.js or Deno module, a Bun or Workerd script, a JSR
or npm package, a WebAssembly Component bound by JS host
glue, or a polyglot artifact whose entry point is JS.

In scope: package record, module record, runtime profile,
loader binding, capability declaration, source map binding,
SBOM manifest, and the cross-references binding each
artifact to its license, signature, and registry entry.

Out of scope: TC39 specification authoring (handled by
the TC39 process itself); host-environment APIs that are
not standardised by WHATWG, W3C, or Ecma (handled by the
host's own conformance regime); WebAssembly modules with
no JS binding (governed by WIA-wasm if and when published).

## §2 Package record

Every distributed package carries:

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `packageRef`         | UUID (RFC 4122) opaque to consumers             |
| `name`               | npm scope/name or JSR `@scope/name`             |
| `version`            | Semantic Versioning 2.0.0                       |
| `engines.ecmascript` | edition tag (`es2024`, `es2025`)                |
| `engines.node`       | semver range; absent if not Node-targeted       |
| `engines.deno`       | semver range; absent if not Deno-targeted       |
| `engines.bun`        | semver range; absent if not Bun-targeted        |
| `engines.browsers`   | Browserslist query                              |
| `type`               | `module` (ESM), `commonjs`, or `dual`           |
| `exports`            | conditional-exports map (Node 16+ form)         |
| `imports`            | scoped import map (subpath self-resolution)     |
| `sideEffects`        | boolean or glob list per ECMA-262 §16           |
| `registryRef`        | npm tarball SHA-512, JSR module digest          |
| `licenseRef`         | SPDX expression                                 |

`packageRef` is the only invariant identifier across
re-publication; the npm/JSR coordinates are mutable
because the registry may admit superseding versions.

## §3 Module record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `moduleRef`          | URI; for ESM, the module specifier              |
| `kind`               | `esm`, `cjs`, `system`, `umd`, `iife`           |
| `entry`              | path within `packageRef` to the module          |
| `imports[]`          | resolved import-specifier list                  |
| `dynamicImports[]`   | specifiers known statically; SHOULD be a subset |
| `assertions`         | per Import Attributes (TC39 Stage 4)            |
| `topLevelAwait`      | boolean — TLA per ECMA-262 §16                  |
| `sourceMapRef`       | URI to source-map (RFC 8259) per Source Map v3  |
| `wasmBinding`        | URI to bound `.wasm` if `kind === 'esm'` and    |
|                      | the module imports a Wasm Component             |

ESM modules are the canonical kind. CJS and UMD are
recognised for legacy interop; SystemJS is recognised
for in-browser dynamic loading; IIFE is recognised only
when an SBOM declaration accompanies the artifact.

## §4 Runtime profile record

A runtime profile is the smallest declarable execution
target.

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `profileRef`         | URI; one of `browser`, `node`, `deno`, `bun`,   |
|                      | `workerd`, `edge`, `embedded`                   |
| `ecmaEdition`        | edition the runtime implements (latest 4 yrs)   |
| `intlSupport`        | ECMA-402 conformance level (`tier-1`/`-2`/`-3`) |
| `webApis[]`          | Fetch, Streams, URL, WebCrypto, WebAssembly,    |
|                      | Workers, EventTarget — per WHATWG / W3C         |
| `wasi`               | absent, `preview2`, or later — for Deno / Wasm  |
| `tlaBoundary`        | how the runtime handles TLA at root             |
| `unhandledRejection` | `terminate`, `warn`, or `silent`                |

Browser profile additionally references Browserslist
queries; Node profile references the active Node LTS
schedule; Deno and Bun profiles reference their own
release calendars. Edge profiles (Workers, Cloudflare,
Deno Deploy, Vercel Edge) declare the WinterCG Minimum
Common API surface that the deployment relies on.

## §5 Loader binding record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `loaderRef`          | UUID                                            |
| `kind`               | `static`, `dynamic`, `host-defined`             |
| `resolution`         | resolution algorithm (`node-cjs`, `node-esm`,   |
|                      | `import-maps`, `jsr`, `deno-nm`, `bun-nm`)      |
| `importMapRef`       | URI to import-map JSON if applicable            |
| `mainFields[]`       | for `node-cjs`: `main`, `module`, `browser`     |
| `conditional`        | for `node-esm`: condition list (`import`,       |
|                      | `require`, `default`, `node`, `browser`,        |
|                      | `worker`, `edge-light`, `deno`, `bun`)          |
| `cacheKey`           | hash determining cache identity                 |

Loader bindings are reproducible: the same input
manifests with the same loader and conditions MUST
resolve to the same module graph.

## §6 Capability declaration record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `capabilityRef`      | UUID                                            |
| `network`            | `none`, `same-origin`, `allow-list`, `any`      |
| `filesystem`         | `none`, `read`, `write`, `read-write` with path |
|                      | allow-list                                      |
| `subprocess`         | `none`, or named binary allow-list              |
| `env`                | environment-variable allow-list                 |
| `eval`               | `forbid`, `csp-strict-dynamic`, `permit`        |
| `dynamicCode`        | `Function`, `eval`, `new AsyncFunction` —       |
|                      | declared individually                           |
| `crossOriginIsolated`| boolean — for SharedArrayBuffer / Atomics       |

Capability declarations are advisory at static analysis
and enforceable when the runtime supports a permission
model (Deno `--allow-*`, Node `--permission`, Workerd
bindings, browser CSP).

## §7 SBOM manifest record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `sbomRef`            | URI to CycloneDX 1.5 or SPDX 2.3 JSON           |
| `componentCount`     | total components, including transitive          |
| `licenseSummary`     | SPDX expression union over components           |
| `vulnerabilityRef`   | URI to OSV.dev or GHSA listing, with date       |
| `signatureRef`       | RFC 7515 JWS signature over the SBOM            |

SBOM publication is mandatory for every package
participating in WIA-js; the SBOM URL is the only field
guaranteed to outlive the package's registry presence.

## §8 Source-map record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `sourceMapRef`       | URI to `.map` file                              |
| `version`            | Source Map v3                                   |
| `file`               | generated file the map applies to               |
| `sources[]`          | original source file list                       |
| `sourcesContent[]`   | inline original sources (optional)              |
| `names[]`            | identifier list                                 |
| `mappings`           | VLQ-encoded segment string                      |
| `ignoreList`         | indexes into `sources[]` to suppress in stack   |
|                      | traces (per Chrome `x_google_ignoreList`)       |

Source-maps containing personal data, internal hostnames,
or unredacted developer email addresses MUST be marked
private and served behind authentication.

## §9 Cross-domain references (informative)

- WIA-language-bridge — for ECMA-402 locale exchange
- WIA-prompts — for embedded LLM prompt artifacts
- WIA-plugins — for plugin host runtimes
- WIA-pubscript — for HTML/EPUB Publication scripting

## Annex A — Conformance disclosure

Implementations declare the manifest schemas they accept,
the canonicalisation form (RFC 8785), and the JWS key set
used to sign the SBOM and the package metadata.

## Annex B — TC39 stage and edition policy

WIA-js conformance requires support for ECMA-262
proposals at Stage 4 by the time the WIA edition is
published. Stage 3 proposals are recognised but
optional; Stage 0–2 proposals are out of scope. Edition
support follows a four-year window: a runtime conforms
to WIA-js if it implements the four most recent ECMA-262
editions, regardless of ordering.

## Annex C — Worked package record (informative)

```json
{
  "packageRef": "f63f4f04-9a76-4d2c-9f53-2c4d09a1bbb1",
  "name": "@example/widget",
  "version": "2.4.1",
  "type": "module",
  "engines": {"ecmascript": "es2024", "node": ">=20.10.0"},
  "exports": {".": {"import": "./dist/index.js", "types": "./dist/index.d.ts"}},
  "registryRef": "sha512-...",
  "licenseRef": "Apache-2.0"
}
```

## Annex D — Versioning

PHASE versioning follows the WIA governance procedure.
Field additions are minor; field removals or semantic
redefinition require a major bump synchronised with the
TC39 edition window described in Annex B.

## Annex E — Conformance level

Conformance is "Core" (package + module + runtime profile
+ loader binding + SBOM) or "Full" (adds capability
declaration and signed source-map records).

## Annex F — IEEE 754 numeric edge cases

ECMA-262 §6.1.6.1 defines `Number` as IEEE 754:2019
double-precision binary64. Cross-runtime divergence is
forbidden; in particular:

- `NaN` is a single bit pattern at the wire boundary;
  payload bits are stripped on JSON serialisation.
- `-0` is preserved through `Object.is` and through
  canonicalisation per RFC 8785.
- Subnormal handling follows the IEEE 754 default; flush-
  to-zero modes are rejected when binding a runtime
  profile.

Implementations whose underlying VM diverges from the
above MUST decline to advertise WIA-js conformance
until divergence is corrected.

## Annex G — Unicode and ECMA-402 binding

String values are sequences of UTF-16 code units per
ECMA-262, but the registry-level transport encoding is
UTF-8 (RFC 8259). A package whose identifier or
localised metadata contains lone surrogates is rejected
by the registry validator with `unicode/well-formed`
problem-type.

ECMA-402 (Internationalization) tier-1 conformance
requires the runtime to expose `Intl.Locale`,
`Intl.DateTimeFormat`, `Intl.NumberFormat`,
`Intl.Collator`, and `Intl.RelativeTimeFormat`. Tier-2
adds `Intl.PluralRules`, `Intl.ListFormat`,
`Intl.Segmenter`. Tier-3 adds `Intl.DurationFormat`,
`Intl.DisplayNames` with full CLDR coverage.

## Annex H — JSON Canonicalisation

JSON-bearing records are canonicalised per RFC 8785 (JSON
Canonicalization Scheme) before signature. The canonical
form is UTF-8, sorts object keys lexicographically by
UTF-16 code unit, prints numbers per ECMA-262
`Number.prototype.toString` (RFC 8785 §3.2.2.3), and
strips insignificant whitespace. Implementations that
sign records with a different canonical form are non-
conformant.

弘益人間 (Hongik Ingan) — Benefit All Humanity
