# WIA-pet-cloning PHASE 4 — INTEGRATION Specification

**Standard:** WIA-pet-cloning
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited pet-cloning programme integrates
with the systems that surround it: veterinary clinics that manage donors,
recipients, and live offspring; breed registries that record verified
identity; insurance carriers that price coverage on cloned animals; the
operator's customer-relationship system that mediates owner contact; and the
regulators and accreditation bodies that read the evidence package. It also
defines the evidence-package format that bundles a complete case for
external publication and audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8615 (well-known URIs)
- IETF RFC 8288 (Web Linking)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 11784 / 11785 (animal RFID)
- ISO 8601 (date and time)
- W3C Verifiable Credentials Data Model 2.0 (the verification report MAY be
  re-issued as a Verifiable Credential for downstream consumers; this is
  optional)

---

## §1 Veterinary Clinic Integration

The clinical custody of the donor, recipient, and live offspring is held by
veterinary clinics that may not be co-located with the programme's
laboratory. Integration with clinic systems is achieved through a thin
adapter that translates clinic-side encounter records into the recipient,
pregnancy, and parturition shapes defined in PHASE-1 §8.

The adapter is owned by the operating programme. Clinic-side integrations
SHOULD use the clinic's existing patient-management system as the system of
record for clinical observations, with the programme's API as the system of
record for cloning-specific records. Conflicts are resolved in favour of the
clinic for clinical fields (e.g. body-condition score) and in favour of the
programme for cloning-specific fields (e.g. embryo IDs); the adapter
documents the resolution rules in the programme's integration dossier.

Clinics that are not running a digital patient-management system MAY
contribute records via a paper-form intake; the operator transcribes the
record into the API. The transcription event is recorded with the
transcribing staff member's identifier and the SHA-256 of the scanned form,
held in the evidence storage described in §3.

## §2 Breed Registry Integration

A verified live offspring is registrable with the breed registry that holds
the donor's pedigree. Integration with the registry is mediated by the
verification report (PHASE-2 §7) and an optional Verifiable Credential
re-issuance (W3C VC 2.0) that carries the same payload in a credential-
format envelope. Registries that prefer a single signed report consume the
JSON form; registries that have adopted Verifiable Credentials consume the
VC form.

In either form, the report's authenticity is established by the issuing
laboratory's signature and at least one witness signature; registries that
require additional witnesses (some pedigree registries require two) consume
multi-witness reports as defined in PHASE-3 §9.

A registry that issues a registration certificate for the offspring SHOULD
back-link to the WIA verification report so that downstream consumers
(buyers, insurers, dog-show secretariats) can resolve the chain of
attestation back to the issuing laboratory.

## §3 Evidence Package Format

The evidence package is the externally-citable artefact for a case. It is
produced by the API endpoint defined in PHASE-2 §8 and is a tarball with the
following layout:

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  case.json                    — case record (PHASE-1 §2)
  donor.json                   — donor record (PHASE-1 §3)
  samples/                     — per-sample records and custody logs
  oocytes/                     — per-oocyte records and custody logs
  reconstructions/             — per-SCNT records
  embryos/                     — per-embryo records, including time-lapse
                                 archive references
  recipients/                  — per-recipient records, with pregnancy and
                                 parturition updates
  verification.json            — verification report (PHASE-2 §7)
  welfare/                     — animal welfare review records (PHASE-3 §4)
  regulatory/                  — regulatory authorisation references
  audit/                       — API audit log excerpts
  instruments/                 — instrument register entries (PHASE-1 §11)
```

The package is content-addressable: the manifest carries the SHA-256 of
each record file, and the manifest itself is signed.

## §4 Manifest and Signatures

The manifest is a JSON document that lists every file in the package, its
SHA-256 digest, its size in bytes, and its content-type. The manifest is
signed by the operating programme's HTTP-message-signature key (RFC 9421)
and counter-signed by the witness laboratory whose signature also appears
on the verification report.

A consumer that receives a package verifies the signatures, recomputes the
file digests, compares them to the manifest, and rejects the package on any
mismatch. Verification tools that follow this PHASE emit Problem-Details
(RFC 9457) responses on rejection, with a `type` of
`urn:wia:pet-cloning:evidence-mismatch` and a `manifestDelta` extension that
identifies the offending entries.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-pet-cloning` (RFC 8615) that links to the API root
(PHASE-2 §2), the public accreditation certificate, the published
welfare-review framework, the published quality dossier, and the
programme's incident-statistics summary (PHASE-3 §10).

The discovery document carries a freshness header indicating the last
update; consumers SHOULD refresh the document at least once per day during
active integrations.

## §6 Owner Identity Mediation

Owner identity is held in the operating programme's CRM, never in the API
data store. The CRM mediates between owners and the integrating systems by
issuing the opaque `ownerReference` token used in PHASE-1 §3 and by
translating consumer requests (e.g. an owner asking for a copy of their
case file) into authenticated API calls.

Owner-facing exports of case material are produced by the CRM from the
evidence package: the CRM strips audit-only sections and adds a
human-readable summary written in the owner's preferred language. The
machine-readable evidence package itself is not delivered to owners
directly; it is intended for laboratories, registries, and regulators.

A subject-access request from an owner returns the human-readable summary
plus the verification report (PHASE-2 §7). The CRM logs the access event
under ISO/IEC 27701:2019 §B.8.5.7 expectations.

## §7 Insurance Carrier Integration

Insurance carriers that price coverage on cloned animals consume the
verification report and selected sections of the evidence package
(notably the parturition record, the offspring's neonatal health
observations, and the welfare review). Carriers do not consume owner
identity; the binding from the verified offspring back to the policy-
holder is held in the carrier's policy administration system.

Programmes SHOULD publish a profile of which evidence sections they are
willing to share with carriers under a standard data-sharing agreement,
and carriers SHOULD limit their requests to that profile. Out-of-profile
requests return `403 Forbidden` from the API with a problem document of
type `urn:wia:pet-cloning:evidence-profile`.

## §8 Regulator and Accreditation Body Access

Regulators and accreditation bodies access the API via dedicated client
certificates issued by the certifying body. Access scopes for these
clients include the full case record set, including welfare-review records
and incident reports; consumer-facing scopes (insurer, registry) are
narrower and are documented in the API's OpenAPI document.

## §9 Time-of-Cite Evidence Pinning

When a case's verification report is cited externally (in a peer-reviewed
publication, a registry catalogue, an insurer policy attachment), the
citing party retrieves the evidence package once and pins the package
content-address (its manifest SHA-256) in the citation. Subsequent
consumers verify the citation by re-fetching the package and comparing
content-addresses; programmes MUST keep evidence packages addressable by
their pinned manifest digests for at least seven years from the citation
event.

## §10 Migration from Pre-Standard Records

Programmes that operated before WIA-pet-cloning reached version 1.0 MAY
migrate historical cases by emitting a synthetic case record that carries
the original case's identifying information plus a `legacyImport` flag.
Synthetic cases are accepted by the registry and insurer profiles but are
not eligible for verification reports; programmes MUST attach a
contemporaneous re-verification (a fresh genetic comparison) to convert a
synthetic case into a fully-verified case.

The migration tooling is published by the operating programme and is
audited as part of the annual ISO/IEC 17025 surveillance audit.

## §11 Public Catalogue and Aggregator Feeds

Programmes that elect to publish a public catalogue of verified cases
(some breed-specific registries do, others do not) emit an Atom or JSON
Feed listing the verification reports. The feed entries link back to the
content-addressed evidence packages and SHOULD NOT carry owner identity
or unredacted clinical detail; the feed is intended as a discovery
mechanism, not a primary record.

## §12 Worked Example: Producing and Pinning an Evidence Package

The following sequence shows how a registry, an insurer, and a regulator
each consume a single evidence package without re-querying the live API.

1. The operating programme, on completing the verification report, posts
   `POST /v1/cases/{caseId}/evidence` and receives a `202 Accepted` with
   a `Location` header pointing at the package resource.
2. The package transitions through `pending` → `building` → `ready`. On
   `ready`, the manifest carries the SHA-256 of every file and is signed
   per §4. The package URL is content-addressed by the manifest digest.
3. The breed registry retrieves the package, verifies the signatures and
   digests, and ingests the verification record. The registry records the
   manifest digest in its registration entry so that downstream consumers
   can audit the registration's provenance.
4. The insurance carrier requests the same package URL but uses an
   evidence-profile scope (§7) that delivers only the parturition,
   neonatal, and welfare sections. The carrier records the same manifest
   digest in the policy attachment.
5. The accreditation body consumes the full package via its dedicated
   client certificate (§8) and uses the included audit-log excerpts to
   verify that custody chains are complete.

Each consumer holds the manifest digest as the citation anchor; the
operating programme keeps the package addressable for at least seven
years from the citation event per §9.

## §13 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully with at
least one veterinary clinic, at least one breed registry that recognises
the verification report, and at least one accreditation body that consumes
the evidence package. The programme's integration dossier records the
list of integrations and the test runs that confirmed each.

Sunsetting an integration (a registry deprecates a profile, a clinic
changes vendor) is announced via the well-known discovery document at
least 90 calendar days before the integration is removed; in-flight cases
that depend on the sunsetting integration are migrated before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-pet-cloning
- **Last Updated:** 2026-04-27
