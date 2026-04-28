# WIA-plugins PHASE 4 — Integration Specification

**Standard:** WIA-plugins
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how WIA-plugins integrates
with adjacent ecosystems (host platforms, OS
permission models, supply-chain attestation
systems, and downstream WIA standards), how
conformance evidence is produced, and how
deployments are governed.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 17065:2012, ISO/IEC 27001:2022, ISO/IEC 27034
- IETF RFC 7515 (JWS), RFC 9421 (HTTP Message Signatures)
- CycloneDX 1.5, SPDX 2.3
- Semantic Versioning 2.0.0
- W3C WebExtensions Manifest, W3C CSP 3
- WebAssembly Component Model
- Sigstore-style ephemeral identity (informative)

---

## §1 Scope

This PHASE covers integration points outside the
PHASE-1..3 scope: how WIA-plugins participants
consume upstream specifications (host APIs, SBOM
formats), how downstream WIA standards reference
plugin artefacts, and how conformance is assessed,
recorded, and audited.

## §2 Upstream integration

### 2.1 Host APIs

Each host (VS Code, JetBrains, WordPress, browser,
OBS Studio, Eclipse, Blender, etc.) publishes its
own API surface. WIA-plugins references the surface
URL in the host record and does not duplicate the
API documentation.

### 2.2 SBOM formats

CycloneDX 1.5 and SPDX 2.3 are accepted
interchangeably. The registry provides crosswalk
between the two on read.

### 2.3 W3C / IETF

Subresource Integrity, CSP 3, and the IETF
signature suite (RFC 9421, RFC 9530) are consumed
at their latest Recommendation / RFC status.

## §3 Marketplace integration

Plugin marketplaces (VS Marketplace, Chrome Web
Store, Firefox Add-ons, WordPress.org Plugin
Directory, JetBrains Marketplace) integrate via the
publisher's upload of releases through the
registry. Marketplace operators may mirror the
registry content under their own discovery
endpoint while preserving the publisher's
signatures.

## §4 OS permission integration

On platforms with OS-mediated permissions (macOS,
iOS, Android, Linux flatpak/snap), the plugin's
capability declaration is mapped to the host's OS
permission grants. The mapping is documented per
host in the host record.

## §5 Conformance

### 5.1 Evidence package

Every conformant deployment publishes:

- declared host catalogue with API surface URLs;
- declared release register with attached SBOMs;
- the test-vector matrix per Annex G of each
  PHASE;
- the JWS signing key set used to sign manifests
  and releases.

### 5.2 Auditor responsibilities

Under ISO/IEC 17065:2012:

1. Manifest signatures verify against the
   publisher's JWKS.
2. Archive digests match the declared digest.
3. SBOM signatures verify and the SBOM closure
   contains only declared components.
4. Capability declarations align with the host's
   permission model.
5. Yank propagation is honoured within the audit
   window.

### 5.3 Tier promotion

| Tier            | Auditor                              | Cadence      |
|-----------------|--------------------------------------|--------------|
| Self-declared   | none                                 | annual       |
| Verified        | independent third-party              | 24 months    |
| Anchored        | accredited body (ISO/IEC 17065)      | 12 months    |

## §6 Cross-domain references (normative)

| Standard                 | Integration point                       |
|--------------------------|-----------------------------------------|
| WIA-js                   | JS runtime profiles                     |
| WIA-prompts              | prompt-mediated extensions              |
| WIA-multiverse-interface | XR plugin host runtime                  |
| WIA-pubscript            | publication-script extensions           |
| WIA-lms                  | LTI-tool plugins on the LMS             |

## §7 Privacy

Plugin telemetry, when enabled, follows the
publisher's privacy policy. Hosts surface the
declaration prior to enabling telemetry. Subject
access requests follow the publisher's privacy
regime.

## §8 Security

Vulnerability disclosure follows ISO/IEC 30111.
Critical CVEs trigger a yank-and-republish flow:
the affected release is yanked; a patched release
is published; the registry surfaces the
vulnerability join in the SBOM endpoint.

## §9 Localisation

Plugin metadata (name, summary, release notes) is
localised in BCP 47 form. UI strings ship in
locale bundles per PHASE-1 §8.

## §10 Accessibility

Hosts that present plugin landing pages MUST honour
WCAG 2.2 AA. Plugins themselves declare their
accessibility evidence URL in the manifest.

## §11 Open governance

Issues at
`github.com/WIA-Official/wia-standards/issues` with
the `plugins` label.

## Annex A — Conformance disclosure

Implementations link to the conformance evidence
URL from the README.

## Annex B — Worked release record (informative)

```json
{
  "releaseRef": "https://reg.example.org/releases/foo/1.4.0",
  "manifestRef": "https://reg.example.org/plugins/foo/1.4.0/manifest",
  "archiveRef": "https://reg.example.org/releases/foo/1.4.0.vsix",
  "archiveDigest": "sha-512=...",
  "sbomRef": "https://reg.example.org/sboms/foo/1.4.0",
  "releasedAt": "2026-04-28T11:32:00+09:00"
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with host-API major revisions.

## Annex D — Open governance

Decision logs are published alongside release
notes for the registry itself.

## Annex E — Withdrawal procedure

Tombstone the publisher's evidence package;
tombstones are immutable.

## Annex F — Reproducibility

Evidence is reproducible from the manifest, the
archive, the SBOM, and the runtime profile.
Reproduction harnesses are published per host.

## Annex G — Test vectors

Every normative requirement in PHASE-1..3 has at
least one positive vector and one negative vector.

## Annex H — Sustainability

Marketplaces SHOULD declare their per-installation
energy estimate (CDN egress + compute) using the
methodology of ISO 14064 or the Green Software
Foundation SCI specification.

## Annex I — Risk register

| Risk                                  | Mitigation                |
|---------------------------------------|---------------------------|
| Unsigned manifest accepted            | Reject; require JWS sig   |
| Archive digest mismatch               | Reject install            |
| Capability over-declaration           | Static analysis check     |
| Stale SBOM                            | Re-publish on each release|
| Federation peer compromise            | Tombstone + JWKS rotate   |
| Yanked release re-served              | Audit-feed surfacing      |
| OS permission spoofing                | Host-mediated grant only  |

## Annex J — Industry binding catalogue

| Host segment           | Bound profile                        |
|------------------------|--------------------------------------|
| Code editor            | VS Code Extension API, JetBrains SDK |
| Browser                | W3C WebExtensions Manifest v3        |
| CMS                    | WordPress Plugin Header              |
| 3-D modelling          | Blender Add-on schema (informative)  |
| DAW                    | VST 3 / CLAP (informative)           |
| OS desktop             | flatpak / snap manifest (informative)|

## Annex K — Continuous improvement programme

Each registry publishes an annual improvement plan
addressing yank latency, SBOM completeness, and
host-API drift.

## Annex L — Reference implementation

A reference WIA-plugins registry is published at
the WIA Standards GitHub umbrella under
`wia-plugins-reference` (Apache-2.0).

## Annex M — Vendor-neutrality commitment

Marketplace operators that mirror the WIA registry
preserve publisher signatures verbatim. The
mirror's secondary signature is an addendum, not a
replacement; consumers MAY verify either signature.

## Annex N — Open-source SBOM tooling

Reference SBOM-generation tools are published per
runtime (CycloneDX-CLI for general use,
spdx-tools, syft, cdxgen) and tracked at
`/v1/registry/sbom-tools`. The catalogue records
the tool version that produced each registered
SBOM so that SBOM reproducibility audits can
reproduce the exact tool environment.

## Annex O — Capability evolution

When a host introduces a new capability, the
registry catalogues it under
`/v1/registry/capabilities` with the host version
that introduced the capability. Plugins targeting
older host versions cannot declare capabilities
that did not yet exist; the registry rejects such
declarations with a Problem Details fragment.

## Annex P — Researcher access

Security researchers with a documented coordinated
disclosure agreement receive scoped access to the
registry's audit feed for the relevant plugin or
publisher.

## Annex Q — Continuous fairness audit

Plugins that incorporate AI-assisted features
(LLM-driven content generation, ML classification)
declare a fairness audit URL in the manifest's
`aiAssistMethod` block. Audits are renewed on a
12-month cadence and tombstoned on expiry.

## Annex R — Plugin marketplace operator
##           certification

Marketplace operators that mirror the registry
apply for operator certification. Certification
verifies that the marketplace preserves publisher
signatures and propagates yanks within the
declared SLA. Certified operators are catalogued
at `/v1/registry/marketplace-operators`.

## Annex S — Cross-host portability

Plugins that target multiple hosts MAY declare a
shared core module that runs across hosts (e.g. a
WebAssembly Component) plus per-host adapters. The
shared core is signed once and referenced by each
host binding.

## Annex T — Vendor neutrality and migration kit

Migration between marketplaces is supported by a
publisher migration kit: the publisher exports
their plugin records, signing key set, and SBOM
register; the destination marketplace imports the
package and verifies signatures. The kit is
reproducible; no vendor-specific transformation is
required.

## Annex U — Annual ecosystem report

The registry publishes an annual ecosystem report
summarising plugin counts by host, SBOM coverage,
yank latency, and CVE response times. The report
is informative and is intended for ecosystem
visibility.

## Annex V — Open-source SDK catalogue

Reference SDKs are published per host and tracked
at `/v1/registry/sdks`. SDKs simplify plugin
authoring against the WIA-plugins contract and
ship under permissive open-source licenses
(Apache-2.0 or MIT).

## Annex W — Plugin starter templates

Starter templates for common plugin shapes (a
command extension, a content provider, a settings
panel, a webview) are published per host and
tracked at `/v1/registry/starters`. The templates
are versioned with the host SDK they target.

## Annex X — Plugin code-signing trust roots

A subset of high-trust publishers carry an
extended-validation attestation tied to a known
identity (organisation, sovereign agency, certified
vendor). The registry records the EV attestation
in the publisher record so that hosts can prefer
EV-signed plugins for installation in restricted
environments (managed enterprise, school,
government).

## Annex Y — Vendor-specific extensions

Marketplace operators that add proprietary fields
(e.g. promotional banners, featured-plugin badges)
namespace the fields under `x-<operator>` so that
the canonical record remains marketplace-agnostic.
The registry preserves the namespacing on
mirroring.

## Annex Z — Disaster recovery

Registry deployments declare RPO and RTO targets
in their evidence package. Default targets: RPO ≤
1h; RTO ≤ 4h. DR drills are run annually with the
results catalogued in the audit feed.

弘益人間 (Hongik Ingan) — Benefit All Humanity
