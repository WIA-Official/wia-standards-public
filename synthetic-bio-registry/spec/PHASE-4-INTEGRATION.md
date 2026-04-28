# WIA-BIO-021 — Phase 4: Integration

> Synthetic-bio-registry canonical Phase 4: ecosystem integration (cross-walk + tooling + screening).

# WIA-BIO-021: Synthetic Biology Registry Specification v1.0

> **Standard ID:** WIA-BIO-021
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biotechnology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Part Classification System](#2-part-classification-system)
3. [Metadata Standards](#3-metadata-standards)
4. [Sequence Formats](#4-sequence-formats)
5. [SBOL Compliance](#5-sbol-compliance)
6. [Characterization Data](#6-characterization-data)
7. [Version Control](#7-version-control)
8. [Access Control and Licensing](#8-access-control-and-licensing)
9. [Registry Interoperability](#9-registry-interoperability)
10. [Implementation Guidelines](#10-implementation-guidelines)

---


## 10. Implementation Guidelines

### 10.1 REST API Endpoints

#### 10.1.1 Part Registration
```
POST /api/v1/parts
Content-Type: application/json

{
  "partId": "BBa_K123456",
  "name": "Strong GFP Device",
  "type": "composite",
  "sequence": "ATGCGT...",
  ...
}

Response: 201 Created
{
  "partId": "BBa_K123456",
  "version": "1.0.0",
  "url": "https://registry.wiastandards.com/parts/BBa_K123456"
}
```

#### 10.1.2 Part Retrieval
```
GET /api/v1/parts/{partId}?version=1.2.0&format=sbol

Response: 200 OK
Content-Type: application/rdf+xml

<sbol:ComponentDefinition>...</sbol:ComponentDefinition>
```

#### 10.1.3 Part Search
```
GET /api/v1/parts/search?type=promoter&organism=E.coli&strength=high

Response: 200 OK
{
  "results": [
    {
      "partId": "BBa_J23100",
      "name": "Anderson Promoter Collection",
      "strength": 1791,
      "relevance": 0.95
    }
  ],
  "total": 15,
  "page": 1
}
```

#### 10.1.4 Characterization Update
```
PATCH /api/v1/parts/{partId}/characterization
Content-Type: application/json

{
  "measurements": [...]
}

Response: 200 OK
{
  "version": "1.1.0",
  "updated": "2024-08-20T14:45:00Z"
}
```

### 10.2 Data Storage

**Recommended Database Schema**:

```sql
CREATE TABLE parts (
    part_id VARCHAR(20) PRIMARY KEY,
    name VARCHAR(255) NOT NULL,
    type VARCHAR(50) NOT NULL,
    status VARCHAR(20) NOT NULL,
    safety_level VARCHAR(10) NOT NULL,
    sequence TEXT NOT NULL,
    sequence_hash CHAR(64) NOT NULL,
    version VARCHAR(20) NOT NULL,
    created_at TIMESTAMP NOT NULL,
    modified_at TIMESTAMP NOT NULL,
    author_id INTEGER NOT NULL,
    license VARCHAR(20) NOT NULL
);

CREATE TABLE characterization_data (
    id SERIAL PRIMARY KEY,
    part_id VARCHAR(20) REFERENCES parts(part_id),
    measurement_type VARCHAR(50) NOT NULL,
    value NUMERIC NOT NULL,
    unit VARCHAR(20) NOT NULL,
    conditions JSONB NOT NULL,
    performed_by VARCHAR(100),
    performed_at TIMESTAMP NOT NULL
);

CREATE TABLE version_history (
    id SERIAL PRIMARY KEY,
    part_id VARCHAR(20) REFERENCES parts(part_id),
    version VARCHAR(20) NOT NULL,
    changes TEXT NOT NULL,
    sequence_hash CHAR(64) NOT NULL,
    created_at TIMESTAMP NOT NULL,
    created_by INTEGER NOT NULL
);
```

### 10.3 Search Indexing

Implement full-text search on:
- Part ID
- Part name
- Description
- Sequence features
- Author information
- Tags and keywords

**Elasticsearch Example**:
```json
{
  "mappings": {
    "properties": {
      "partId": {"type": "keyword"},
      "name": {"type": "text"},
      "type": {"type": "keyword"},
      "sequence": {"type": "text", "analyzer": "dna_analyzer"},
      "description": {"type": "text"},
      "characterization": {"type": "nested"}
    }
  }
}
```

### 10.4 Validation Rules

**Part ID Validation**:
```javascript
const partIdRegex = /^BBa_[PRBCETKV][0-9]{5,6}$/;

function validatePartId(partId) {
  if (!partIdRegex.test(partId)) {
    throw new Error('Invalid part ID format');
  }
  // Check for collisions
  if (partExists(partId)) {
    throw new Error('Part ID already exists');
  }
}
```

**Sequence Validation**:
```javascript
function validateSequence(sequence, type) {
  const dnaRegex = /^[ATCGNatcgn]+$/;
  const rnaRegex = /^[AUCGNaucgn]+$/;

  if (type === 'dna' && !dnaRegex.test(sequence)) {
    throw new Error('Invalid DNA sequence');
  }

  if (sequence.length < 1 || sequence.length > 1000000) {
    throw new Error('Sequence length out of bounds');
  }

  return true;
}
```

### 10.5 Error Codes

| Code | Meaning | Action |
|------|---------|--------|
| BIO001 | Invalid part ID format | Fix part ID |
| BIO002 | Part already exists | Use different ID or update existing |
| BIO003 | Invalid sequence | Check nucleotide characters |
| BIO004 | Missing required metadata | Add required fields |
| BIO005 | Version conflict | Resolve version history |
| BIO006 | Safety level violation | Review biosafety classification |
| BIO007 | License incompatibility | Update license terms |
| BIO008 | Format conversion failed | Check input format |

---

## Appendix A: Example Part Registration

### Complete Part Registration (Promoter)

```json
{
  "partId": "BBa_J23100",
  "partName": "Anderson Promoter Collection - J23100",
  "type": "promoter",
  "subtype": "constitutive",
  "status": "characterized",
  "safetyLevel": "BSL-1",

  "sequence": {
    "nucleotides": "TTGACGGCTAGCTCAGTCCTAGGTACAGTGCTAGC",
    "length": 35,
    "format": "raw",
    "sequenceType": "dna",
    "checksum": "sha256:a7f8d9c2..."
  },

  "organism": "Escherichia coli",
  "biologicalContext": {
    "expressionHost": "E. coli K12",
    "compartment": "genomic"
  },

  "characterization": {
    "measurements": [
      {
        "type": "promoter_strength",
        "value": 1791,
        "unit": "RPU",
        "stdDev": 85,
        "replicates": 8,
        "method": "Fluorescence assay with GFP reporter"
      }
    ]
  },

  "author": {
    "name": "Anderson Lab",
    "affiliation": "UC Berkeley",
    "email": "anderson@berkeley.edu"
  },

  "version": "1.0.0",
  "license": "CC-BY-4.0",
  "created": "2006-01-15T10:00:00Z",
  "modified": "2024-06-20T15:30:00Z",

  "references": [
    {
      "type": "publication",
      "title": "Promoter library characterization",
      "authors": ["Anderson JC", "et al."],
      "year": 2006,
      "doi": "10.xxxx/xxxxxx"
    }
  ]
}
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-021 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*



---

## A.1 Cross-walk to international standards

| Concern                    | Standard                                       |
|----------------------------|------------------------------------------------|
| SBOL conformance           | SBOL 2.3.0, SBOL 3.x                           |
| Sequence format            | NCBI GenBank, FASTA, FASTQ                     |
| Aligned reads              | SAM/BAM/CRAM (samtools spec)                   |
| Open material transfer     | OpenMTA (BioBricks Foundation)                 |
| Biosafety risk groups      | NIH Guidelines, WHO LBM4                       |
| Dual-use research          | WHO Framework for Responsible Life Sciences    |
| Sequence sharing (security)| IGSC Harmonized Screening Protocol             |
| Personal data              | EU GDPR, US HIPAA (where human samples)        |

## A.2 Tooling integration

Reference adapters and consumers: Benchling (commercial design + ELN), SnapGene (commercial design), Geneious (commercial design), j5 (open-source assembly automation), RAVEN (open-source assembly), Cello (genetic-circuit synthesis). Each adapter consumes the canonical part envelope (Phase 1 §A.1) and emits the analysis envelope expected by the design tool. Adapters live outside the conformance scope but the envelope they consume is conformance-tested.

## A.3 Sequence-screening integration

Contributing labs are expected to run synthesis-order screening against the IGSC Harmonized Screening Protocol or an equivalent. The registry exposes a `screening.passed` flag per part with the screening provider, screening date, and the hash of the screening receipt. Federated registries are expected to honour the same flag; trust-class downgrade applies to peers that do not.

## A.4 Assembly-method cross-walk

| Method                      | Reference                          |
|-----------------------------|------------------------------------|
| BioBrick Assembly (RFC10)   | Knight 2003                        |
| Gibson Assembly             | Gibson 2009                        |
| Golden Gate / MoClo         | Engler 2008 / Weber 2011 / RFC1000 |
| iBrick Assembly             | RFC43                              |
| LCR (Ligase Cycling)        | de Kok 2014                        |
| SLIC (Sequence + Ligase Independent Cloning) | Li 2007             |
| CPEC (Circular Polymerase Extension) | Quan 2009                  |

## A.5 Reference container, CLI, governance

The reference container at `wia/synthetic-bio-registry-host:1.0.0` ships every Phase 2 endpoint with mock data and feeds the conformance suite. The companion CLI at `cli/synthetic-bio-registry.sh` ships sample envelope generators for parts, characterisation submissions, and federation envelopes. WIA Standards composition: WIA-INTENT for workload intent, WIA-OMNI-API for credential storage, WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for federation handshake.

## A.6 Reference list

- SBOL 3.x specification (Synthetic Biology Open Language)
- NCBI GenBank flat-file specification
- iGEM Registry Parts standard
- BioBricks Foundation OpenMTA
- IGSC Harmonized Screening Protocol
- WHO Laboratory Biosafety Manual, 4th edition (LBM4)
- NIH Guidelines for Research Involving Recombinant or Synthetic Nucleic Acid Molecules
- ISO/IEC 17025 — competence of testing and calibration laboratories
- IETF RFC 8032 — Ed25519 (signing)


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/synthetic-bio-registry/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-synthetic-bio-registry-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/synthetic-bio-registry-host:1.0.0` ships every synthetic-bio-registry envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/synthetic-bio-registry.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Synthetic-bio-registry deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
