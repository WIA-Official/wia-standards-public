# WIA-infrastructure PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-infrastructure
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for WIA-infrastructure.
The standard governs the records that an accredited civil-infrastructure
asset operator publishes to describe its built assets — bridges, roads,
tunnels, dams, levees, water-distribution mains, wastewater collectors,
storm-drainage networks, urban transmission and distribution lines,
district-heating mains, and the structural supporting works of mass-transit
systems — and to bind those assets to inspection, condition assessment,
maintenance, rehabilitation, and end-of-service-life decisions across a
multi-decade asset lifecycle.

References (CITATION-POLICY ALLOW only):

- ISO 55000 / 55001 / 55002 (asset management)
- ISO 19650-1 / 19650-2 (information management using BIM)
- ISO 16739 (Industry Foundation Classes — IFC)
- ISO 23386 / 23387 (data templates for built assets)
- ISO 8601 (date and time)
- ISO/IEC 11578 (UUID)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- W3C XML Schema Definition 1.1 (legacy CAD/BIM interchange envelope)
- buildingSMART IFC4 / IFC4.3 (asset and infrastructure schema)

---

## §1 Scope

This PHASE document defines the persistent shapes for the records that
an asset-owning programme exchanges across the lifecycle of a civil-
infrastructure asset: planning approval, design, construction, hand-
over, in-service inspection, condition assessment, maintenance work
order, rehabilitation, decommissioning, and archival. It is intended
for use by:

- Public works departments and ministries that own roadway,
  hydraulic, and structural assets.
- Utility owners (water, wastewater, district heating, transmission)
  whose linear assets carry public service.
- Asset-management consultancies and inspection contractors that
  produce condition data on behalf of owners.
- BIM coordinators and design-build joint ventures handing over
  digital asset records at substantial completion.
- Long-term archives that hold lifecycle records of assets after the
  programme that created them is wound down.

Out of scope are building-interior fittings (covered by adjacent
WIA standards governing buildings), transient construction-site
records that do not survive into operation, and the financial
accounting records that owners maintain separately under their
public accounting standards.

## §2 Asset Identifier and Classification

```
assetId            : string (uuidv7)
ownerOrgId         : string (institutional identifier)
assetClass         : enum  ("bridge" | "road-segment" | "tunnel" |
                     "dam" | "levee" | "retaining-wall" |
                     "water-main" | "wastewater-collector" |
                     "storm-drain" | "transmission-line" |
                     "distribution-line" | "district-heating-main" |
                     "transit-viaduct" | "transit-tunnel" |
                     "railway-track" | "harbour-wall" |
                     "lock-gate" | "user-defined")
assetSubclass      : string (operator-controlled vocabulary, e.g.
                     "prestressed-concrete-girder", "ductile-iron-
                     pressurised-main", "earthen-flood-levee")
constructionPeriod : object
  yearOpened       : integer
  yearLastRehab    : integer (nullable)
classificationRefs : array of string (IFC4 entity reference, ISO
                     23386 property-set reference, owner
                     classification scheme reference)
```

The asset identifier is opaque and is generated at first
registration. Identifier reuse is forbidden — superseded assets that
have been retired retain their identifier permanently for citation
purposes; replacement assets get a new identifier and a
`predecessor` reference in their record.

## §3 Spatial Identity

```
spatialIdentity:
  geometry        : object (GeoJSON Geometry; LineString for linear
                     assets, Polygon for areal assets, Point for
                     point assets such as culverts)
  crs             : string (EPSG code; WGS 84 / EPSG:4326 default)
  referenceMonument : string (identifier of the surveyed monument
                     used for geodetic control on this asset)
  alignmentRefs   : array of object (chainage along an alignment
                     and the alignment identifier; required for
                     linear roadway and railway assets)
  bimAuthorRef    : string (IFC4.3 IfcSpatialZone or facility
                     identifier)
  validFromPeriod : object
    from          : string (ISO 8601)
    until         : string (ISO 8601, nullable)
```

The spatial identity is versioned: every realignment, widening,
extension, or partial collapse-and-replacement event emits a new
spatial-identity record with `validFromPeriod` set. Earlier records
remain addressable so historical inspection reports continue to
resolve to the geometry that existed at the time of the inspection.

## §4 Component and Subcomponent Decomposition

Assets are decomposed into engineering components per the operator's
classification scheme; the decomposition is bound to ISO 16739
(IFC4) entities so that the digital twin of the asset is mutually
intelligible across BIM tools.

```
component:
  componentId     : string (uuidv7)
  parentAssetId   : string (uuidv7)
  parentComponent : string (uuidv7, nullable for top-level)
  ifcEntityType   : string (IFC4 class — IfcBeam, IfcColumn,
                     IfcSlab, IfcWall, IfcPipe, IfcPipeFitting,
                     IfcEarthworksFill, IfcCourse, etc.)
  materialClass   : enum  ("reinforced-concrete" |
                     "prestressed-concrete" |
                     "structural-steel" |
                     "weathering-steel" | "ductile-iron" |
                     "vitrified-clay" | "high-density-polyethylene" |
                     "composite-frp" | "stone-masonry" |
                     "earth-fill" | "asphalt-concrete" |
                     "user-defined")
  designReferences : array of string (governing design standard
                     references — AASHTO LRFD edition, Eurocode
                     part, AISC manual edition, ACI 318 edition,
                     EN 1991 / 1992 / 1993 part, AWWA C-series)
  servicePeriod   : object
    inService     : string (ISO 8601)
    designLife    : string (ISO 8601 duration)
```

Component records carry the design-references field so that future
inspection campaigns can resolve the load-rating model that applies.
Operators that retrofit a component under a newer design code emit
a new component record with `predecessor` pointing to the prior
record; both records remain addressable.

## §5 Inspection Record

```
inspection:
  inspectionId    : string (uuidv7)
  assetId         : string (uuidv7)
  componentRefs   : array of string (component identifiers covered)
  inspectionType  : enum  ("routine-visual" | "in-depth" |
                     "underwater" | "ndt-acoustic-emission" |
                     "ndt-ultrasonic" | "ndt-ground-penetrating-
                     radar" | "ndt-half-cell-potential" |
                     "load-test" | "post-event" | "user-defined")
  governingMethod : string (e.g. AASHTO Manual for Bridge Element
                     Inspection, AWWA M77, EN 1990 Annex B, owner-
                     specific procedure reference)
  inspectorRef    : string (institutional inspector identifier;
                     accreditation reference for ISO/IEC 17025
                     where applicable)
  inspectionDate  : string (ISO 8601)
  observations    : array of object (each observation carries
                     componentRef, defectClass per the governing
                     method, severity per the governing method,
                     extent, photographic evidence content-
                     address, and the inspector's narrative)
  conditionStateRef : string (resulting condition state per the
                     governing method's ladder)
```

Inspection records are immutable once signed by the inspector.
Corrections produce a successor inspection record with
`supersedes` set to the prior record's identifier and a stated
correction reason.

## §6 Condition Assessment

```
conditionAssessment:
  assessmentId    : string (uuidv7)
  assetId         : string (uuidv7)
  inspectionRefs  : array of string (inspections used as evidence)
  assessmentDate  : string (ISO 8601)
  ratingScheme    : string (AASHTO general appraisal, ASCE
                     condition index, EN 1990 reliability index,
                     CIRIA Manual on the Use of Rock in Hydraulic
                     Engineering, owner-specific ladder; the
                     scheme name is captured verbatim so that
                     downstream consumers can resolve the ladder)
  componentRatings : array of object (componentRef, rating value,
                     rating narrative, supporting inspection
                     identifiers)
  overallRating   : string (per the chosen scheme)
  remainingServiceLifeEstimate : object
    estimateYears : number
    method        : string (e.g. CSA S6 Annex, AASHTO LRFR,
                     reliability-based deterioration model,
                     owner historical-trend method)
    confidence    : enum ("low" | "medium" | "high")
```

The condition assessment is the bridge between observed inspection
results and the maintenance prioritisation that follows in PHASE-3.
Assessments cite the governing rating scheme so that an auditor or
successor owner can reproduce the rating from the underlying
observations.

## §7 Maintenance Work Order

```
workOrder:
  workOrderId     : string (uuidv7)
  assetId         : string (uuidv7)
  componentRefs   : array of string
  workType        : enum  ("preventive-routine" |
                     "preventive-cyclic" |
                     "corrective-minor" |
                     "corrective-structural" |
                     "rehabilitation" |
                     "replacement" |
                     "emergency")
  triggeringAssessmentRef : string (uuidv7, nullable for routine)
  authorisedScope : string (textual scope authorised by the asset
                     owner)
  contractorRef   : string (institutional contractor identifier)
  scheduledWindow : object
    plannedStart  : string (ISO 8601)
    plannedEnd    : string (ISO 8601)
  executionRecord : object (nullable until executed)
    actualStart   : string (ISO 8601)
    actualEnd     : string (ISO 8601)
    materials     : array of object (material specification
                     reference, quantity, unit)
    laborHours    : number
    asBuiltContentAddress : string (URI of the as-built drawings
                     produced — IFC, DWG, or PDF deliverable)
```

Work-order records bind the maintenance action to the asset
component and to the assessment that triggered it; a
`triggeringAssessmentRef` is required for any non-routine work
type so that downstream auditors can trace the chain of
justification from the inspection through the assessment to the
expended work.

## §8 Hand-Over and Construction-Phase Record

The Hand-Over record records the Federation file (IFC4.3) that the
designer-builder consortium delivers to the owner at substantial
completion together with the verifying surveys and the residual
defect register.

```
handOver:
  handOverId      : string (uuidv7)
  assetId         : string (uuidv7)
  contractorRefs  : array of string (designer-builder consortium
                     parties)
  federationFileContentAddress : string (IFC4.3 federation file
                     content-address; the file MUST validate
                     against ISO 16739 IFC4)
  surveyReportContentAddresses : array of string
  residualDefectRegisterContentAddress : string
  handOverDate    : string (ISO 8601)
  defectLiabilityPeriod : string (ISO 8601 duration)
```

Hand-over records bind the digital twin produced during design and
construction to the operating-phase record set so that a successor
inspector can read the design intent without having to re-trace the
design-build documents that produced the asset.

## §9 Decommissioning Record

```
decommissioning:
  decommissioningId : string (uuidv7)
  assetId         : string (uuidv7)
  reason          : enum  ("end-of-service-life" |
                     "replacement-by-successor" |
                     "demolished-after-event" |
                     "consolidated-into-another-asset")
  decommissionDate : string (ISO 8601)
  successorAssetRef : string (uuidv7, nullable)
  finalDispositionContentAddress : string
  archivalDepositRef : string (long-term archive deposit
                     identifier — see PHASE-4 §6)
```

Decommissioning records do not delete the underlying asset record;
they mark it superseded and bind it to the long-term archive
deposit so that subsequent forensic, insurance, or research
queries can resolve the asset's full lifecycle even after the
asset itself is gone.

## §10 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every asset they hold and honour the
identifier-permanence rules in §2 and the immutability rules in
§5. Records are content-addressable; corrections emit successor
records and never mutate prior records in place.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-infrastructure
- **Last Updated:** 2026-04-28
