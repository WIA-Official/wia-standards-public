# WIA-pet-cloning PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-pet-cloning
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for WIA-pet-cloning. It
covers the records that an accredited cloning programme produces and exchanges
across its four phases: tissue acquisition, somatic cell nuclear transfer
(SCNT), gestation, and post-natal verification. The format is intended for
veterinary cloning facilities, accredited reference laboratories, breed
registries, and the regulators that oversee them.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 17025:2017 (general requirements for testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO 11784 / 11785 (radio-frequency identification of animals — code structure and air-interface)
- ISO/IEC 11578 (UUID)
- IETF RFC 8259 (JSON Data Interchange Format)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 4122 (UUID URN namespace)
- W3C XML Schema Definition 1.1 (only used for legacy registry import)

---

## §1 Scope

This PHASE document defines the persistent on-disk and on-wire shapes for the
records that flow across a complete cloning case, from biopsy through SCNT,
gestation, and post-natal verification. It does not cover the wet-lab
procedure itself, the contracts between owner and provider, or the regulatory
dossiers required by national authorities; those are addressed in PHASE-3 and
PHASE-4 respectively.

Implementations covered by this PHASE include:

- Tissue-bank Laboratory Information Management Systems (LIMS).
- Embryology lab software that drives SCNT workstations.
- Veterinary clinic systems that manage surrogate dams during gestation.
- Breed-registry systems that ingest post-natal verification reports.
- Auditor and regulator tools that read evidence packages.

Owner-facing companion-animal apps and food-animal cloning programmes are
explicitly out of scope; food-animal cloning is covered by separate
agricultural standards.

## §2 Case Identifier

Every cloning programme assigns a deterministic Case Identifier (`caseId`)
that is the primary key for all subsequent records. The identifier is a
UUIDv7 (RFC 4122 / ISO/IEC 11578) so that records sort by acquisition time
when stored in lexicographic order, which simplifies cold-storage audits.

```
caseId            : string (uuidv7, lowercase, 36 chars including hyphens)
caseAcquiredAt    : string (ISO 8601 / RFC 3339, UTC, second precision)
caseProgramme     : string (accredited programme code, registered with WIA)
caseStatus        : enum  ("draft" | "active" | "ceased" | "transferred" | "verified")
```

The `caseProgramme` is a registered code — for example `WIA-PC-K-0012` for
the twelfth accredited programme registered in Korea — and is governed by the
register defined in PHASE-3.

## §3 Donor Animal Record

The donor animal is the source of the somatic cells. Its record is the
identity anchor for the entire case.

```
donor:
  donorId            : string (uuidv7)
  caseId             : string (uuidv7, foreign key to §2)
  taxon              : string (taxonomic identifier; "felis catus" or "canis
                       familiaris" for the two species in routine clinical
                       practice; other taxa permitted)
  rfid               : string (15-digit ISO 11784/11785 code, optional;
                       absent when the donor is post-mortem and never chipped)
  birthdate          : string (ISO 8601 date; precision MAY be reduced to year
                       when the donor is rescued or unknown-history)
  ownerReference     : string (opaque token assigned by the programme)
  consentHash        : string (SHA-256 of the signed owner-consent PDF)
  euthanasiaStatus   : enum ("alive" | "post-mortem")
  postMortemHours    : integer (hours between death and tissue acquisition;
                       MUST be present when euthanasiaStatus = "post-mortem";
                       MUST NOT exceed the species-specific viability window
                       defined in PHASE-3 §5)
```

`ownerReference` is deliberately a programme-internal token rather than a
direct identifier. The DATA-FORMAT layer never carries owner identity by
design; the binding lives only in operator systems and is exposed only on
explicit subject request via the procedure in PHASE-4.

## §4 Tissue Sample Record

Each donor produces one or more tissue samples. The recommended sample is a
4-mm punch biopsy from the inner ear or umbilicus (live donor) or a similar
biopsy from non-traumatised dermis (post-mortem donor).

```
sample:
  sampleId           : string (uuidv7)
  donorId            : string (uuidv7)
  collectedAt        : string (ISO 8601 / RFC 3339)
  collectionSite     : enum  ("inner-ear" | "umbilicus" | "dermis" |
                       "buccal-mucosa" | "other")
  collectionMethod   : enum  ("punch-biopsy" | "needle-biopsy" |
                       "swab" | "other")
  preservation:
    cryoprotectant   : string (e.g. "DMSO/FCS-10", expressed as recipe code)
    freezeProgram    : string (controlled-rate freezer program name)
    storageVessel    : enum  ("LN2-vapour" | "LN2-liquid" | "minus80")
    storageVesselId  : string (institutional vessel ID, plain text)
    rackPosition     : string (rack/box/well, plain text)
  custodyLog         : array of CustodyEvent (see §9)
  qualityControl:
    viabilityAssay   : enum  ("trypan-blue" | "annexin-v" | "amnis" | "none")
    viabilityResult  : number (percent, 0-100; absent when assay = "none")
    fibroblastDoublings : integer (cumulative population doublings recorded
                       at each thaw; appended over the life of the sample)
```

Implementations MUST treat the cryopreservation record as immutable once the
sample has been frozen down for the first time. Subsequent thaw and re-freeze
events are appended to `custodyLog` rather than mutating the original record.

## §5 Oocyte Record

Oocytes are typically obtained from non-related donor animals (slaughterhouse-
derived ovaries for felines, or in-vivo retrieval for some canine programmes).
Records carry sufficient detail to support post-hoc re-derivation of the
embryo's mitochondrial origin, since the recipient inherits its mitochondria
from the oocyte donor and not from the somatic donor.

```
oocyte:
  oocyteId           : string (uuidv7)
  caseId             : string (uuidv7)
  oocyteDonorTaxon   : string (taxonomic identifier; SHOULD match the donor
                       taxon — across-species SCNT is permitted only when
                       declared explicitly under §10)
  retrievalMethod    : enum  ("ovariectomy" | "follicular-aspiration" |
                       "in-vivo-flush")
  maturationStage    : enum  ("GV" | "MI" | "MII")
  maturationMedium   : string (medium recipe code)
  enucleationMethod  : enum  ("aspiration" | "chemically-assisted" |
                       "spindle-view")
  enucleationConfirmed : boolean
  hoechstExposureSec : integer (cumulative seconds of UV exposure to
                       Hoechst-stained spindle, when spindle-view used; MUST
                       be ≤ 30 to remain within recommended dose limits)
```

Programmes that pool oocytes from multiple donors MUST emit one oocyte record
per oocyte and MUST NOT collapse a batch into a single aggregate record.

## §6 Reconstruction (SCNT) Record

The SCNT record links a somatic donor sample to an oocyte and records the
fusion and activation parameters.

```
reconstruction:
  reconstructionId   : string (uuidv7)
  caseId             : string (uuidv7)
  sampleId           : string (uuidv7, references §4)
  oocyteId           : string (uuidv7, references §5)
  scntAt             : string (ISO 8601 / RFC 3339)
  fusionMethod       : enum  ("electrofusion" | "viral" | "phytohaemagglutinin")
  electrofusionParams:
    voltsPerCm       : number
    pulseMicrosec    : integer
    pulseCount       : integer
  activationMethod   : enum  ("chemical" | "electrical" | "combined")
  activationAgents   : array of string (recipe codes — examples: "ionomycin-5",
                       "6-DMAP-2h"; the recipe register is governed by PHASE-3)
  technicianId       : string (programme-internal staff identifier)
  workstationId      : string (instrument identifier registered under §11)
```

## §7 Embryo Culture Record

Each successful reconstruction yields zero or more cultured embryos. The
culture record tracks the embryo through the cleavage stages until it is
either transferred, vitrified, or discarded.

```
embryo:
  embryoId           : string (uuidv7)
  reconstructionId   : string (uuidv7)
  cultureMedium      : string (recipe code)
  developmentLog     : array of DevelopmentEvent
  fate               : enum ("transferred" | "vitrified" | "discarded" |
                       "failed-to-cleave")
  vitrificationRecord: VitrificationRecord (present iff fate = "vitrified")

DevelopmentEvent:
  observedAt         : string (ISO 8601)
  stage              : enum ("1-cell" | "2-cell" | "4-cell" | "8-cell" |
                       "morula" | "early-blastocyst" | "expanded-blastocyst" |
                       "hatched-blastocyst" | "arrested")
  qualityGrade       : string (Gardner grading for blastocysts, omitted
                       at earlier stages)
  imagingArtefactSha : string (SHA-256 of the time-lapse stack archive,
                       when time-lapse imaging was performed)
```

## §8 Recipient (Surrogate Dam) and Pregnancy Record

Embryos transferred for gestation produce a recipient record. The format is
constrained by veterinary clinic workflows and is therefore the most
clinically detailed record in the format.

```
recipient:
  recipientId        : string (uuidv7)
  caseId             : string (uuidv7)
  rfid               : string (ISO 11784/11785 code)
  taxon              : string
  ageAtTransferYears : number
  parityPriorTransfer: integer
  estrousSyncProtocol: string (recipe code)
  transferAt         : string (ISO 8601)
  embryosTransferred : integer
  transferTechnique  : enum ("non-surgical" | "surgical-laparoscopic" |
                       "surgical-laparotomy")
  pregnancyDiagnosis:
    method           : enum ("ultrasonography" | "serum-relaxin" | "manual")
    diagnosedAt      : string (ISO 8601)
    foetusCount      : integer
  parturition:
    plannedAt        : string (ISO 8601 date)
    occurredAt       : string (ISO 8601 / RFC 3339; absent before parturition)
    mode             : enum ("eutocia" | "dystocia-assisted" |
                       "elective-caesarean" | "emergency-caesarean")
    liveOffspring    : integer
    stillborn        : integer
```

## §9 Custody Log

Every record that holds biological material — sample, oocyte, embryo, and
recipient — carries a `custodyLog` field of type `array of CustodyEvent`.

```
CustodyEvent:
  eventAt            : string (ISO 8601)
  eventType          : enum ("created" | "thawed" | "refrozen" | "shipped" |
                       "received" | "transferred" | "discarded")
  fromLocationId     : string (institutional location ID)
  toLocationId       : string (institutional location ID)
  custodianId        : string (programme staff identifier)
  notes              : string (free text; redacted on export when contains PII)
```

Implementations MUST append to this log atomically with the state change that
the event describes; missing custody events are treated by auditors as a
failure of evidence integrity.

## §10 Cross-Species and Edge-Case Declarations

Cross-species SCNT (e.g. domestic cat oocyte cytoplast carrying a wildcat
nucleus) is permitted under this format but MUST be declared with a
`crossSpecies` flag at the case level, accompanied by the regulatory
authorisation reference from PHASE-3. Implementations MUST refuse to mark a
case `verified` until an authorisation reference is present.

## §11 Workstation and Instrument Register

Every instrument that touches biological material — microscopes, electrofusion
chambers, controlled-rate freezers — is registered with a stable identifier so
that operational drift can be traced to the instrument that contributed it.
The register is maintained by the operating programme and is exported as part
of the evidence package described in PHASE-4 §3.

## §12 Conformance Considerations

Implementations claiming conformance to PHASE-1 MUST emit every record listed
in this document for every case, MUST honour the immutability and append-only
constraints, and MUST fail validation cleanly with a Problem Details (RFC 9457)
response when an export is requested for a case that contains incomplete
records. Conformance against this PHASE is independent of conformance against
PHASE-2 / PHASE-3 / PHASE-4, but Deep certification under WIA-pet-cloning
requires all four PHASE conformance attestations.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-pet-cloning
- **Last Updated:** 2026-04-27
