# WIA-micro-credential PHASE 1 — Data Format Specification

**Standard:** WIA-micro-credential
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for digital
micro-credentials covering issuer, recipient, credential
class, issuance event, evidence record, framework
mapping, endorsement, revocation, and verification
artefacts. The format aligns with W3C Verifiable
Credentials Data Model 2.0, IMS Open Badges 3.0, the
Europass Credentials profile, and the European
Qualifications Framework so credentials are portable
across awarding bodies, learning platforms, and
employer recognition systems.

References (CITATION-POLICY ALLOW only):
- W3C Verifiable Credentials Data Model 2.0
- W3C Decentralized Identifiers (DIDs) 1.0
- W3C Data Integrity 1.0; W3C VC JOSE 1.0; W3C Status List 2021
- 1EdTech Open Badges 3.0 (IMS Open Badges)
- 1EdTech Comprehensive Learner Record (CLR) 2.0
- Europass Credentials specification (European Commission)
- European Learning Model (ELM); ESCO classification (skills, occupations)
- European Qualifications Framework (EQF); national NQF mappings
- ISCED 2011 — International Standard Classification of Education
- ISCED-F 2013 — Fields of Education and Training
- ECTS — European Credit Transfer and Accumulation System
- ISO/IEC 19796-1 — IT for learning, education, and training: quality management
- ISO/IEC 21724-1 — Information technology for learning, education, and training (extensions to OB)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 7515 (JWS), RFC 9530 (Content-Digest)
- IETF RFC 6749, RFC 7636 (PKCE) — for issuer / verifier flows where bearer-token authn applies

---

## §1 Scope

This PHASE applies to systems that issue, hold, present,
verify, or revoke digital micro-credentials covering
short-form learning achievements, micro-degrees,
professional badges, language proficiency attestations,
training-certificate awards, and continuing-education
records issued by accredited or recognised bodies.

In scope: issuer, recipient, credential-class,
issuance, evidence, framework-mapping, endorsement,
revocation, status-list, presentation, and verification
records. Out of scope: full-degree academic transcripts
that span multiple credentials (handled by the academic-
credential standard), and unverifiable participation
acknowledgements that make no learning-outcome claim
(handled by general consumer-record standards).

## §2 Issuer record

| Field                | Source / Binding                                 |
|----------------------|--------------------------------------------------|
| `issuerRef`          | DID (W3C DID 1.0); also UUID for internal index  |
| `legalName`          | localised label (BCP 47)                         |
| `accreditationRef`   | upstream accreditation body identifier (national |
|                      | quality-assurance agency, ISO 21001)             |
| `eqfBindings[]`      | EQF level(s) the issuer is recognised to award   |
| `nqfBindings[]`      | national NQF level(s)                            |
| `domainAuthority`    | per ISCED-F 2013 broad-field codes                |
| `publicKeySet`       | JWKS URL / verification-method list              |
| `revocationEndpoint` | status-list publication URL                      |
| `issuancePolicyRef`  | issuance-policy document hash                    |

Issuers expose their public key set so any verifier can
resolve credential signatures without contacting the
issuer.

## §3 Recipient record

| Field                | Source / Binding                                 |
|----------------------|--------------------------------------------------|
| `recipientRef`       | DID (preferred) or hashed identifier             |
| `pseudonymRef`       | study- or platform-local pseudonym                |
| `controllerKey`      | controller public key (DID Document)             |
| `wallet`             | wallet implementation reference (per W3C VC      |
|                      | implementer registry)                            |

Recipients hold their credentials in a wallet under
their control; issuers issue to a DID and never to an
opaque platform identifier.

## §4 Credential-class record

| Field                | Source / Binding                                 |
|----------------------|--------------------------------------------------|
| `credentialClassRef` | URI                                              |
| `name`               | localised label                                  |
| `kind`               | `badge`, `micro-degree`, `certificate`,          |
|                      | `attestation`, `professional-recognition`         |
| `learningOutcomes`   | array of learning-outcome statements; each tied  |
|                      | to ESCO skills / competences and Bloom's revised |
|                      | taxonomy verb                                    |
| `eqfLevel`           | EQF level (1-8)                                  |
| `iscedLevel`         | ISCED 2011 level                                 |
| `fieldCode`          | ISCED-F 2013                                     |
| `creditValue`        | ECTS points / national equivalent                |
| `notionalLearning`   | hours estimated; assessment hours separate        |
| `assessmentMethod`   | proctored exam / portfolio / capstone /           |
|                      | observation / e-assessment                        |
| `prerequisites`      | optional list of pre-required credential classes |
| `expiry`             | optional default-validity period                  |

Credential classes are catalogued; issuance instances
reference the class so re-issuance shares lineage.

## §5 Issuance record

| Field                | Source / Binding                                 |
|----------------------|--------------------------------------------------|
| `issuanceRef`        | UUID                                             |
| `credentialClassRef` | §4                                               |
| `issuerRef`          | §2                                               |
| `recipientRef`       | §3                                               |
| `issuedAt`           | ISO 8601                                         |
| `validFrom`          | ISO 8601                                         |
| `validUntil`         | ISO 8601 (open if perpetual)                     |
| `evidenceRef[]`      | links to evidence records (§6)                    |
| `endorsementRef[]`   | links to endorsement records (§7)                 |
| `proof`              | W3C Data Integrity proof or JOSE compact form    |
| `revocationStatus`   | active / suspended / revoked                     |
| `statusListUri`      | issuer's W3C Status List 2021 entry              |

The proof binds the issuance to the issuer's signing
key (resolved via the issuer DID Document); the
recipient verifies and stores the issuance in the
wallet.

## §6 Evidence record

| Field                | Source / Binding                                 |
|----------------------|--------------------------------------------------|
| `evidenceRef`        | UUID                                             |
| `kind`               | exam-result / portfolio / assessor-observation / |
|                      | e-portfolio-link / project-deliverable /         |
|                      | proctoring-recording                             |
| `assessorRef`        | credentialed assessor identity                   |
| `assessmentTime`     | ISO 8601                                         |
| `score`              | per-instrument raw and standardised              |
| `narrative`          | localised text (BCP 47)                          |
| `artefactDigest`     | SHA-256 of the artefact (PDF / video / link)     |
| `qaSampleRef`        | quality-assurance sample reference (where the    |
|                      | issuer's QA process samples this evidence)       |

Evidence is held by the issuer (or a sub-processor
identified in the issuance policy) and presented under
recipient consent at verification time.

## §7 Endorsement record

| Field                | Source / Binding                                 |
|----------------------|--------------------------------------------------|
| `endorsementRef`     | UUID                                             |
| `endorserRef`        | DID of the endorsing body / employer / regulator |
| `endorsementText`    | localised text                                   |
| `validFrom`          | ISO 8601                                         |
| `validUntil`         | ISO 8601                                         |
| `proof`              | W3C Data Integrity proof                         |

Endorsements layer trust onto a credential; an
employer endorsement of an issuer's micro-credential
class signals industry recognition.

## §8 Framework-mapping record

| Field                | Source / Binding                                 |
|----------------------|--------------------------------------------------|
| `mappingRef`         | UUID                                             |
| `credentialClassRef` | §4                                               |
| `targetFramework`    | EQF / NQF (per country) / ESCO / O*NET-SOC       |
| `targetCode`         | level / qualification / skill / occupation code  |
| `relation`           | `equivalent`, `partial`, `sub-domain`,           |
|                      | `aligned-broader`, `aligned-narrower`            |
| `attestor`           | who maintains the mapping                        |

Mappings are versioned; framework changes (EQF
revisions, ESCO releases) emit re-mapping events.

## §9 Revocation and status-list record

Revocation aligns with W3C Status List 2021. The
issuer publishes a status list URL; each issuance
records the bit position. Verifiers fetch the status
list, decode, and check the relevant bit. Suspension
uses a separate list per the W3C Status List
specification (revoked vs suspended bits).

| Field                | Source / Binding                                 |
|----------------------|--------------------------------------------------|
| `revocationRef`      | UUID                                             |
| `issuanceRef`        | §5                                               |
| `reason`             | issuer-error / fraud / withdrawn / superseded /  |
|                      | regulatory-direction                             |
| `revokedAt`          | ISO 8601                                         |
| `effectiveListEntry` | bit position on the active status list           |

## §10 Presentation record

A recipient presents one or more credentials to a
verifier as a Verifiable Presentation (VP). Presentation
records capture:

| Field                | Source / Binding                                 |
|----------------------|--------------------------------------------------|
| `presentationRef`    | UUID                                             |
| `holderRef`          | recipient DID                                    |
| `verifierRef`        | verifier DID / domain                             |
| `selectedDisclosure` | which fields of which credentials presented      |
| `proof`              | W3C Data Integrity proof of presentation         |
| `audience`           | verifier identifier or audience claim            |

Selective disclosure (SD-JWT VC, BBS+ signatures, or
analogous schemes) is supported so a recipient can
prove a credential's claims without revealing all
attributes.

## §11 Verification record

| Field                | Source / Binding                                 |
|----------------------|--------------------------------------------------|
| `verificationRef`    | UUID                                             |
| `presentationRef`    | §10                                              |
| `verifierRef`        | the verifying party                              |
| `verificationTime`   | ISO 8601                                         |
| `outcome`            | `valid`, `revoked`, `suspended`, `expired`,      |
|                      | `signature-invalid`, `issuer-not-trusted`        |
| `policyRef`          | the verification policy applied                  |

## §12 Cross-domain references (informative)

- WIA-digital-credential — for parallel-track digital-id
  binding
- WIA-learning-analytics — for outcome-evidence pipeline
- WIA-virtual-classroom — for instructor identity
- WIA-content-ai — for AI-generated evidence governance

## Annex A — Worked Open Badge 3.0 issuance (informative)

```json
{
  "@context": [
    "https://www.w3.org/ns/credentials/v2",
    "https://purl.imsglobal.org/spec/ob/v3p0/context-3.0.3.json"
  ],
  "type": ["VerifiableCredential", "OpenBadgeCredential"],
  "issuer": {"id":"did:web:issuer.example","name":"Issuer Inc."},
  "credentialSubject": {
    "id": "did:key:z6Mk...",
    "achievement": {
      "id": "https://issuer.example/achievements/sql-101",
      "type": ["Achievement"],
      "name": "Introduction to SQL",
      "criteria": {"narrative":"Learner completed all 12 modules and the final assessment."}
    }
  },
  "issuanceDate":"2026-04-12T09:14:00Z"
}
```

## Annex B — Conformance disclosure

Implementations declare the W3C VC Data Model version,
the IMS Open Badges revision, the proof types
supported, the status-list scheme, the EQF / NQF
mappings indexed, and the canonicalisation form.

## Annex C — Versioning

Field additions are minor; semantic redefinition or
removal is major.

## Annex D — Selective-disclosure mechanism

Selective disclosure binds to one of:

- SD-JWT (IETF) — selectively-disclosable JWTs
- BBS+ signatures (W3C VC-DI registry) — multi-message
  signature with disclosure of subset
- Hash-based salting and disclosure (Open Badges)

The mechanism in force is recorded on the issuance
record so verifiers know which scheme to apply.
