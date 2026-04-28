# WIA-plugins PHASE 1 — Data Format Specification

**Standard:** WIA-plugins
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
WIA-plugins, the cross-host plugin and extension
interoperability standard. The records bind every
plugin manifest, runtime profile, capability
declaration, host binding, and signed release to a
documented identifier scheme, an SBOM, and a
provenance trail so that hosts and users can
install, update, and audit plugins without vendor
lock-in.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 27001:2022, ISO/IEC 27034 (application security)
- IETF RFC 4122 (UUID), RFC 8259 (JSON), RFC 8785 (JCS)
- IETF RFC 7515 (JWS), RFC 7517 (JWK), RFC 9421 (HTTP Message Signatures)
- W3C WebExtensions Manifest (informative)
- VS Code Extension API (informative), JetBrains Plugin SDK (informative)
- Eclipse PDE / OSGi R8 (informative)
- WordPress Plugin Header Convention (informative)
- WebAssembly Component Model
- CycloneDX 1.5 / SPDX 2.3 (SBOM)
- Sigstore-style ephemeral identity for publish-from-CI flows
- Semantic Versioning 2.0.0
- ECMAScript Modules (ECMA-262 §16)
- W3C Content Security Policy 3 (capability surface)

---

## §1 Scope

This PHASE applies to records that describe a
plugin or extension distributed for installation
into a host application. The host may be a code
editor, a content management system, a browser, a
desktop application shell, a server-side platform,
a digital audio workstation, a 3-D modelling
package, or a domain-specific platform that
exposes a plugin host API.

In scope: plugin record, manifest record, capability
declaration, host-binding record, release record,
runtime profile, locale-bundle record, dependency
graph, and the cross-references binding each plugin
to its publisher identity, SBOM, and signed
artefact.

Out of scope: pricing and commerce surfaces of
plugin marketplaces (governed by marketplace
operator); the host application's own internal API
(governed by the host's documentation).

## §2 Plugin record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `pluginRef`          | UUID (RFC 4122) opaque                          |
| `name`               | localised plugin name (BCP 47 keys)             |
| `slug`               | URL-safe slug, scoped to the publisher          |
| `publisherRef`       | publisher identity URI                          |
| `category`           | host-defined category                           |
| `summary`            | one-paragraph summary                           |
| `homepageUrl`        | publisher's plugin landing page                 |
| `repositoryUrl`      | source repository                                |
| `licenseRef`         | SPDX expression                                 |
| `signingKeyRef`      | JWKS URL                                        |

`pluginRef` is the only invariant identifier; the
slug is mutable per publisher policy.

## §3 Manifest record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `manifestRef`        | URI to the canonical manifest                   |
| `pluginRef`          | this PHASE §2                                   |
| `version`            | Semantic Versioning 2.0.0                       |
| `releaseChannel`     | `stable`, `beta`, `nightly`, `lts`              |
| `engines`            | host-and-version compatibility map               |
| `entryPoints[]`      | per-host entry point declarations                |
| `dependencies[]`     | other plugin references with version ranges    |
| `peerDependencies[]` | host-provided modules consumed                  |
| `releaseNotesUrl`    | URI to human-readable release notes              |

The manifest is signed with detached JWS over the
canonical JSON form (RFC 8785).

## §4 Capability declaration record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `capabilityRef`      | UUID                                            |
| `network`            | `none`, `same-origin`, `allow-list`, `any`      |
| `filesystem`         | `none`, `read`, `write`, `read-write` with     |
|                      | path allow-list                                  |
| `subprocess`         | `none`, or named binary allow-list              |
| `clipboard`          | `none`, `read`, `write`, `read-write`           |
| `notifications`      | `none`, `permitted`                             |
| `eval`               | `forbid`, `csp-strict-dynamic`, `permit`        |
| `dynamicCode`        | `Function`, `eval`, `new AsyncFunction` —       |
|                      | declared individually                            |
| `crossOriginIsolated`| boolean                                         |

Capability declarations are advisory at static
analysis and enforceable when the host supports a
permission model (manifest v3 sandbox, OS
permissions, MAC profiles).

## §5 Host-binding record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `hostRef`            | URI; e.g. `host:vscode`, `host:wordpress`,      |
|                      | `host:browser`, `host:obs-studio`                |
| `hostVersionRange`   | semver range                                     |
| `apiSurface`         | URI to host-provided API documentation           |
| `lifecycleHooks[]`   | `onInstall`, `onActivate`, `onDeactivate`,      |
|                      | `onUninstall`, `onUpdate`                        |
| `commandRegistrations[]` | host-command identifiers exposed             |

Multiple host bindings per plugin are permitted
(cross-host plugins) provided each binding declares
its own entry point.

## §6 Release record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `releaseRef`         | URI                                             |
| `manifestRef`        | this PHASE §3                                   |
| `archiveRef`         | URI to the release archive (zip, tar.gz, vsix,  |
|                      | crx, xpi, jar)                                  |
| `archiveDigest`      | SHA-512 of the archive bytes                    |
| `sbomRef`            | URI to CycloneDX 1.5 or SPDX 2.3 SBOM           |
| `signature`          | RFC 7515 detached JWS over the canonical record |
| `releasedAt`         | ISO 8601                                        |

Releases are immutable: a re-published archive
emits a new release record with a new
`archiveDigest`.

## §7 Runtime profile record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `profileRef`         | URI                                             |
| `executionModel`     | `js-esm`, `wasm-component`, `native-process`,   |
|                      | `python-3`, `php-8`, `ruby-3`, `host-script`    |
| `crossOriginIsolated`| boolean                                         |
| `permissionsModel`   | `manifest-only`, `host-prompt`, `os-mediated`   |
| `processIsolation`   | `none`, `worker`, `sandbox`, `os-process`,      |
|                      | `vm`                                             |

The runtime profile is referenced by the host
binding and constrains the capability declaration
that the host is willing to honour.

## §8 Locale-bundle record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `bundleRef`          | URI                                             |
| `language`           | BCP 47 tag                                      |
| `messages`           | key/value JSON document                         |
| `pluralRules`        | ECMA-402 plural rules selection                 |
| `bidi`               | `ltr`, `rtl`, `auto`                             |

Locale bundles are versioned with the plugin. Hosts
fetch the bundle matching the user's locale; if no
match, the fallback chain is BCP 47 lookup.

## §9 Dependency graph record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `dependencyRef`      | UUID                                            |
| `from.pluginRef`     | depending plugin                                |
| `to.pluginRef`       | depended-upon plugin                             |
| `versionRange`       | semver range                                    |
| `kind`               | `runtime`, `peer`, `optional`, `dev`            |

Cyclic dependencies are forbidden; the registry
rejects manifests whose dependency closure
contains a cycle.

## §10 Cross-domain references (informative)

- WIA-js — JS runtime profiles
- WIA-prompts — prompt-mediated extensions
- WIA-multiverse-interface — XR plugin host
- WIA-pubscript — publication-script extensions

## Annex A — Conformance disclosure

Implementations declare the manifest schema URIs
they accept, the canonicalisation form (RFC 8785),
and the JWS key set used to sign manifests and
releases.

## Annex B — Worked manifest record (informative)

```json
{
  "manifestRef": "https://reg.example.org/plugins/foo/1.4.0",
  "pluginRef": "f63f4f04-9a76-4d2c-9f53-2c4d09a1bbb1",
  "version": "1.4.0",
  "engines": {"vscode": "^1.84.0"},
  "entryPoints": [
    {"host": "vscode", "module": "./out/extension.js"}
  ]
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with host-API versioning where
applicable.

## Annex D — Conformance level

Conformance is "Core" (plugin + manifest +
capability + host-binding + release) or "Full"
(adds runtime profile, locale bundle, dependency
graph, and SBOM signature).

## Annex E — Privacy

Plugins that collect personal data declare the data
categories and the legal basis in the manifest's
`privacyDeclaration` block. Hosts surface the
declaration to the user before installation.

## Annex F — Security

Plugins running with elevated capabilities
(`subprocess`, `filesystem: read-write`, `eval:
permit`) carry a security review attestation in
the manifest. The attestation references an
external review under ISO/IEC 27034 or sovereign
equivalent.

## Annex G — SBOM bundling

The release record's `sbomRef` is mandatory for
plugins running in environments where supply-chain
attestation is enforced. SBOM signatures are
verified against the publisher's JWKS prior to
installation.

## Annex H — Update channels

Plugins emit releases on declared channels
(`stable`, `beta`, `nightly`, `lts`). Hosts pin
their consumers to a channel; promotion between
channels is editorial and recorded as a release
event.

## Annex I — Long-term support

Plugins MAY declare an LTS commitment with a
`ltsThrough` ISO 8601 date. The publisher commits
to security back-ports until the LTS expiry. The
registry tombstones the LTS commitment on
non-fulfilment.

## Annex J — Plugin telemetry declaration

Plugins emitting telemetry declare:

| Field             | Source / Binding                              |
|-------------------|-----------------------------------------------|
| `kind`            | `usage`, `error`, `performance`               |
| `endpoint`        | URI to the publisher's collector              |
| `optInDefault`    | boolean                                       |
| `dataCategories[]`| documented categories (e.g. `command-name`,   |
|                   | `error-stack-trace`, `host-version`)          |
| `retention`       | ISO 8601 duration                             |
| `legalBasis`      | GDPR / CCPA / K-PIPA basis                    |

Hosts surface the telemetry declaration to the
user before installation.

## Annex K — Plugin marketplace metadata

Marketplace-specific metadata fields (price,
ratings, screenshots, video URL, author bio) are
namespaced under `marketplace:<operator>` so that
the canonical record remains marketplace-agnostic.
Marketplaces consume the namespaced fields without
mutating the canonical record.

## Annex L — Code-signing key custodianship

Code-signing keys MAY be hardware-bound (HSM,
FIDO2 with attestation, smart card). When
hardware-bound, the registry records the
attestation in the publisher record. Loss of the
key triggers immediate revocation and a republish
under a new key.

## Annex M — Plugin removal grace period

When a plugin is uninstalled, hosts SHOULD honour
a 30-day grace period during which user data
created by the plugin remains accessible. After
the grace period, the data may be purged per the
plugin's data-retention policy.

## Annex N — Update-channel transition

Plugins promote releases between channels (`nightly`
→ `beta` → `stable`) by re-publishing under the
target channel. Promotion preserves the original
release record's signature and emits a
`channel.promoted` event in the audit feed.

## Annex O — Plugin pricing surface (informative)

Pricing surfaces (free, free-trial, paid,
subscription, freemium) are recorded as marketplace-
specific extensions per Annex K. The registry
itself does not adjudicate pricing; marketplaces
are responsible for billing flows.

## Annex P — Quality signals

The registry catalogues optional quality signals:

- `crashRate` — host-reported crash rate;
- `latencyP95` — host-reported activation p95;
- `accessibilityScore` — automated WCAG checker;
- `securityRating` — from the latest CVE join.

Signals are advisory; they do not gate
installation but MAY surface in marketplace
search ranking.

## Annex Q — Plugin permission prompt copy

Hosts SHOULD localise the user-facing permission-
prompt copy from the plugin's manifest. The copy
follows the host's UX guidelines but the source
strings come from the plugin's locale bundle so
that publishers control the wording.

弘益人間 (Hongik Ingan) — Benefit All Humanity
