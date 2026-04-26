# WIA-IND-030 PHASE 1 — Data Format Specification

**Standard:** WIA-IND-030
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-IND-030: Circular Economy Standard

**Version:** 1.0.0
**Status:** Active
**Category:** IND (Industry)
**Date:** 2025-12-27
**Author:** WIA Industry Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Circular Economy Principles](#5-circular-economy-principles)
6. [Material Passport System](#6-material-passport-system)
7. [Product Lifecycle Tracking](#7-product-lifecycle-tracking)
8. [Circularity Metrics and Assessment](#8-circularity-metrics-and-assessment)
9. [Design for Circularity](#9-design-for-circularity)
10. [Recycling and Recovery](#10-recycling-and-recovery)
11. [Extended Producer Responsibility](#11-extended-producer-responsibility)
12. [Sharing Economy and Product-as-a-Service](#12-sharing-economy-and-product-as-a-service)
13. [Waste Management](#13-waste-management)
14. [Sustainability and Carbon Metrics](#14-sustainability-and-carbon-metrics)
15. [Certification and Compliance](#15-certification-and-compliance)
16. [Data Models](#16-data-models)
17. [API Specifications](#17-api-specifications)
18. [Security and Privacy](#18-security-and-privacy)
19. [Implementation Guidelines](#19-implementation-guidelines)
20. [Appendices](#20-appendices)

---

## 1. Introduction

### 1.1 Purpose

The WIA-IND-030 Circular Economy Standard provides a comprehensive framework for transitioning from linear "take-make-dispose" economic models to circular systems that eliminate waste and continuously cycle resources. This standard enables organizations to:

- Track materials and products throughout their lifecycle
- Design products for circularity and longevity
- Implement take-back and refurbishment programs
- Optimize recycling and material recovery
- Measure and improve circular economy performance
- Comply with extended producer responsibility regulations
- Enable sharing economy and product-as-a-service models

### 1.2 Background

The linear economy has led to resource depletion, environmental degradation, and excessive waste generation. The circular economy represents a systemic shift towards regenerative systems that:

- Keep products and materials in use at their highest value
- Eliminate waste through design
- Regenerate natural systems
- Create economic value while reducing environmental impact

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard embodies the principle of creating systems that benefit all stakeholders: manufacturers, consumers, communities, and the planet. By enabling circular economy practices, we preserve natural resources for future generations while creating sustainable economic value today.

### 1.4 Target Audience

This standard is intended for:

- Product manufacturers and designers
- Supply chain managers
- Waste management and recycling companies
- Sustainability professionals
- Extended producer responsibility organizations
- Sharing economy platforms
- Government agencies and policymakers
- Certification bodies

---

## 2. Scope

### 2.1 Included

This standard covers:

- Material passport creation and management
- Product lifecycle tracking from design to end-of-life
- Circularity assessment methodologies
- Design for circularity principles
- Recycling chain management
- Extended producer responsibility programs
- Sharing economy platforms
- Waste-to-resource conversion
- Carbon footprint calculation
- Sustainability certifications
- Blockchain-based traceability
- API and data interchange formats

### 2.2 Excluded

This standard does not cover:

- Specific product safety regulations (covered by other standards)
- Financial accounting for circular economy
- Detailed chemical composition standards
- Sector-specific regulations (refer to relevant industry standards)

---

## 3. Normative References

The following documents are referenced in this standard:

- ISO 14040:2006 - Environmental management - Life cycle assessment
- ISO 14044:2006 - Environmental management - Life cycle assessment requirements
- ISO 59004:2024 - Circular economy - Terminology, principles and guidance
- ISO 59020:2024 - Circular economy - Measuring and assessing circularity
- Ellen MacArthur Foundation - Circularity Indicators Framework
- Cradle to Cradle Certified Product Standard v4.0
- EU Ecodesign Directive 2009/125/EC
- EU Waste Framework Directive 2008/98/EC

---

## 4. Terms and Definitions

### 4.1 Circular Economy

An economic system aimed at eliminating waste and continuously cycling resources through strategies including reuse, repair, refurbishment, remanufacturing, and recycling.

### 4.2 Material Passport

A digital identity for materials and products containing information about composition, origin, history, and circularity characteristics.

### 4.3 Material Circularity Indicator (MCI)

A metric that measures how restorative material flows are in a product or company, ranging from 0 (linear) to 1 (fully circular).

### 4.4 Extended Producer Responsibility (EPR)

An environmental policy approach where producers bear significant responsibility for the treatment or disposal of post-consumer products.

### 4.5 Product-as-a-Service (PaaS)

A business model where products are used rather than owned, typically through rental, leasing, or subscription arrangements.

### 4.6 Design for Disassembly

Designing products so they can be easily taken apart at end-of-life to facilitate component reuse and material recycling.

### 4.7 Reparability Index

A score (0-10) indicating how easy a product is to repair, considering factors such as documentation, disassembly, availability of spare parts, and price of parts.

### 4.8 Upcycling

The process of transforming waste materials or products into new materials or products of higher quality or value.

### 4.9 Industrial Symbiosis

The process by which waste or by-products from one industry become raw materials for another.

### 4.10 Zero Waste

A waste management approach aiming to divert at least 90% of waste from landfills and incinerators through reduction, reuse, and recycling.

---

## 5. Circular Economy Principles

### 5.1 Fundamental Principles

The circular economy is built on three core principles:

#### 5.1.1 Principle 1: Eliminate Waste and Pollution

Design out waste and pollution from the beginning by:
- Using materials that can be safely returned to nature or technical cycles
- Avoiding hazardous substances
- Optimizing material usage
- Designing for durability and longevity

#### 5.1.2 Principle 2: Keep Products and Materials in Use

Maximize the value extracted from products and materials by:
- Designing for multiple use cycles
- Enabling repair and refurbishment
- Facilitating component reuse
- Implementing high-quality recycling

#### 5.1.3 Principle 3: Regenerate Natural Systems

Return valuable nutrients to the soil and other ecosystems by:
- Using renewable resources where possible
- Avoiding depletion of non-renewable resources
- Supporting biodiversity
- Sequestering carbon

### 5.2 Value Retention Strategies

The circular economy employs various strategies to retain value, arranged in order of priority (highest to lowest):

1. **Refuse**: Eliminate unnecessary products
2. **Reduce**: Minimize resource consumption
3. **Reuse**: Use products multiple times
4. **Repair**: Fix broken products
5. **Refurbish**: Restore products to like-new condition
6. **Remanufacture**: Rebuild products from used components
7. **Repurpose**: Use products for different purposes
8. **Recycle**: Process materials into new products
9. **Recover**: Extract energy or materials from waste

### 5.3 Circular Business Models

#### 5.3.1 Circular Supplies

Provide fully renewable, recyclable, or biodegradable resource inputs that underpin circular production and consumption.

#### 5.3.2 Resource Recovery

Recover useful resources from disposed products or by-products.

#### 5.3.3 Product Life Extension

Extend the lifecycle of products through repair, upgrade, remanufacture, or resale.

#### 5.3.4 Sharing Platforms

Enable increased utilization of products through shared use, access, or ownership.

#### 5.3.5 Product-as-a-Service

Offer product performance rather than ownership, retaining responsibility for maintenance and end-of-life.

---

## 6. Material Passport System

### 6.1 Overview

A material passport is a digital record containing comprehensive information about the materials and components in a product, enabling circular economy practices.

### 6.2 Required Information

#### 6.2.1 Product Identification

- Unique product ID
- Product name and model
- Manufacturer information
- Manufacturing date and location
- Serial number or batch ID

#### 6.2.2 Material Composition

For each material:
- Material type and grade
- Mass and percentage
- Origin (virgin, recycled, bio-based)
- Supplier information
- Certifications
- Hazard classifications
- Critical raw material status
- Recyclability percentage
- Biodegradability

#### 6.2.3 Design Information

- Modular design (yes/no)
- Disassembly instructions
- Fastener types and locations
- Component list with part numbers
- Reparability index
- Upgradeability potential
- Expected lifespan
- Warranty period

#### 6.2.4 End-of-Life Information

- Take-back program details
- Recycling instructions
- Hazardous component warnings
- Expected recovery rate
- Disposal restrictions
- Recommended recycling facilities

### 6.3 Material Passport Format

Material passports SHALL be created in JSON format conforming to the following structure:

```json
{
  "passportId": "MP-XXXXXX",
  "version": "1.0",
  "productId": "PROD-XXXXXX",
  "productName": "EcoBook Pro 15",
  "manufacturer": {
    "id": "MFG-001",
    "name": "GreenTech Inc.",
    "address": "123 Innovation Drive, San Francisco, CA",
    "contact": "sustainability@greentech.com"
  },
  "manufactureDate": "2025-01-15",
  "serialNumber": "SN-2025-001234",
  "materials": [
    {
      "type": "aluminum-6061",
      "mass": 1.2,
      "massUnit": "kg",
      "percentage": 40,
      "origin": "recycled",
      "recycledContent": 85,
      "recyclability": 95,
      "supplier": "RecycledMetals Co.",
      "certifications": ["ASI", "RCS"],
      "criticalMaterial": false,
      "toxicity": "low",
      "biodegradable": false
    }
  ],
  "designPrinciples": {
    "modular": true,
    "disassemblable": true,
    "standardizedComponents": true,
    "standardizedFasteners": true,
    "repairabilityIndex": 9,
    "upgradeability": true,
    "monoMaterial": false
  },
  "expectedLifespan": 2555,
  "warrantyPeriod": 1095,
  "endOfLife": {
    "takebackProgram": true,
    "recyclingInstructions": "Remove battery, separate materials",
    "recoveryRate": 92,
    "disposalRestrictions": ["landfill-prohibited"],
    "hazardousComponents": []
  },
  "blockchain": {
    "network": "ethereum",
    "contractAddress": "0x...",
    "tokenId": "NFT-001234",
    "verified": true,
    "verifiedAt": "2025-01-15T10:00:00Z"
  },
  "certifications": ["Cradle-to-Cradle-Gold", "EU-Ecolabel"],
  "createdAt": "2025-01-15T10:00:00Z",
  "updatedAt": "2025-01-15T10:00:00Z"
}
```

### 6.4 Blockchain Integration

Material passports SHOULD be registered on a blockchain network to ensure:

- Immutability of records
- Transparency across supply chain
- Verification of authenticity
- Traceability throughout lifecycle
- Prevention of counterfeiting

Supported blockchain networks:
- Ethereum
- Polygon
- Hyperledger Fabric
- Private/Permissioned networks

### 6.5 Material Passport Lifecycle

1. **Creation**: Generated at manufacturing
2. **Distribution**: Transferred with product
3. **Updates**: Modified during refurbishment or repairs
4. **Transfer**: Ownership changes recorded
5. **End-of-Life**: Final disposition recorded
6. **Archival**: Retained for regulatory compliance

---

## 7. Product Lifecycle Tracking

### 7.1 Lifecycle Stages

Products SHALL be tracked through the following lifecycle stages:

1. **Design**: Conceptualization and engineering
2. **Production**: Manufacturing and assembly
3. **Distribution**: Transport to point of sale
4. **Use**: Consumer usage period
5. **Collection**: Return or take-back
6. **Refurbishment**: Restoration to functional condition
7. **Recycling**: Material recovery and reprocessing
8. **Disposal**: Final disposition (minimized)
9. **Regeneration**: Return to production cycle

### 7.2 Lifecycle Events

Each significant event in a product's lifecycle SHALL be recorded with:

- Event ID (unique identifier)
- Event type (stage transition, maintenance, etc.)
- Timestamp (ISO 8601 format)
- Location (address and coordinates)
- Actor (person, organization, facility)
- Description (human-readable)
- Condition assessment


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
