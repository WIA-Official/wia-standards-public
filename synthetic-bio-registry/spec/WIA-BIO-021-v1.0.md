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

## 1. Introduction

### 1.1 Purpose

This specification defines a unified framework for synthetic biology part registration, storage, and sharing. It enables interoperability between major biological part registries while maintaining data integrity and reproducibility.

### 1.2 Scope

The standard covers:
- BioBrick and non-BioBrick part classification
- Metadata schema for biological parts
- Sequence storage and format conversion
- Part characterization data structures
- Version control mechanisms
- Access control and licensing frameworks
- Integration with existing registries (iGEM, Addgene, SynBioHub)

### 1.3 Philosophy

**弘益인間 (Benefit All Humanity)** - This standard promotes open science and global collaboration in synthetic biology by providing accessible, standardized tools for sharing biological knowledge.

### 1.4 Terminology

- **Part**: A discrete DNA/RNA sequence with defined function
- **BioBrick**: Standardized genetic part with RFC10 or RFC25 assembly
- **Device**: Combination of multiple parts with integrated function
- **SBOL**: Synthetic Biology Open Language
- **RFC**: Request for Comments (BioBrick assembly standards)
- **Characterization**: Experimental data describing part behavior

---

## 2. Part Classification System

### 2.1 Primary Part Types

Parts are classified into seven primary categories:

#### 2.1.1 Promoters (Type: PRO)
Regulatory sequences that initiate transcription.

**Naming Convention**: `BBa_P[number]` or `BBa_J[number]`

**Properties**:
- Strength (relative fluorescence units)
- Inducibility (constitutive/inducible)
- Organism specificity
- Regulatory mechanism

**Examples**:
- `BBa_J23100`: Strong constitutive promoter
- `BBa_R0011`: LacI-regulated promoter

#### 2.1.2 Ribosome Binding Sites (Type: RBS)
Translation initiation sequences.

**Naming Convention**: `BBa_B[number]`

**Properties**:
- Translation initiation rate
- Organism compatibility
- Spacing requirements
- Secondary structure

**Examples**:
- `BBa_B0034`: Medium-strength RBS
- `BBa_B0030`: Strong RBS

#### 2.1.3 Coding Sequences (Type: CDS)
Protein-encoding sequences.

**Naming Convention**: `BBa_C[number]` or `BBa_E[number]`

**Properties**:
- Protein product
- Molecular weight
- Function/activity
- Optimal expression conditions
- Codon optimization status

**Examples**:
- `BBa_E0040`: Green Fluorescent Protein (GFP)
- `BBa_E1010`: mRFP1 (Red Fluorescent Protein)

#### 2.1.4 Terminators (Type: TER)
Transcription termination sequences.

**Naming Convention**: `BBa_B[number]` or `BBa_T[number]`

**Properties**:
- Termination efficiency
- Directionality (forward/reverse)
- Context dependency

**Examples**:
- `BBa_B0015`: Double terminator
- `BBa_B0010`: T1 from E. coli rrnB

#### 2.1.5 Plasmid Backbones (Type: VEC)
Vector sequences for part maintenance and propagation.

**Naming Convention**: `BBa_pSB[number]`

**Properties**:
- Antibiotic resistance
- Origin of replication
- Copy number
- Cloning sites
- Size (bp)

**Examples**:
- `BBa_pSB1C3`: Chloramphenicol resistance, high copy
- `BBa_pSB1A3`: Ampicillin resistance, high copy

#### 2.1.6 Composite Parts (Type: COM)
Multi-part assemblies with integrated function.

**Naming Convention**: `BBa_K[number]`

**Properties**:
- Component parts list
- Assembly method
- Expected function
- Validation status

**Examples**:
- `BBa_K123456`: GFP expression device
- `BBa_K654321`: Inducible protein production system

#### 2.1.7 Regulatory Elements (Type: REG)
Protein binding sites, operators, and other regulatory sequences.

**Properties**:
- Binding protein/molecule
- Regulatory mechanism
- Affinity/binding constant

### 2.2 Part Status

Each part has a lifecycle status:

| Status | Code | Description |
|--------|------|-------------|
| Planning | PL | Part design phase |
| Ordered | OR | DNA synthesis ordered |
| Available | AV | Part available in registry |
| Characterized | CH | Experimental data available |
| Validated | VA | Independently verified |
| Deprecated | DP | Superseded or problematic |
| Deleted | DL | Removed from registry |

### 2.3 Safety Classification

Parts are categorized by biosafety level:

```
BSL-1: Minimal risk (e.g., GFP, standard promoters)
BSL-2: Moderate risk (e.g., mammalian cell work)
BSL-3: Serious risk (e.g., pathogenic organism parts)
BSL-4: Extreme risk (restricted access)
```

---

## 3. Metadata Standards

### 3.1 Required Metadata

Every registered part MUST include:

```json
{
  "partId": "BBa_K123456",
  "partName": "Strong GFP Expression Device",
  "type": "composite",
  "subtype": "reporter",
  "status": "available",
  "safetyLevel": "BSL-1",
  "author": {
    "name": "Team iGEM 2024",
    "affiliation": "MIT",
    "email": "igem@mit.edu"
  },
  "created": "2024-06-15T10:30:00Z",
  "modified": "2024-08-20T14:45:00Z",
  "version": "1.2.0",
  "license": "CC-BY-4.0"
}
```

### 3.2 Sequence Metadata

```json
{
  "sequence": {
    "nucleotides": "ATGCGTAAAGGAGAAGAACTTTTC...",
    "length": 1234,
    "format": "raw|fasta|genbank|sbol",
    "sequenceType": "dna|rna|protein",
    "checksum": "sha256:abc123...",
    "features": [
      {
        "type": "promoter",
        "start": 1,
        "end": 35,
        "strand": "+",
        "name": "J23100"
      },
      {
        "type": "RBS",
        "start": 40,
        "end": 52,
        "strand": "+",
        "name": "B0034"
      }
    ]
  }
}
```

### 3.3 Biological Context

```json
{
  "organism": "Escherichia coli K12",
  "strain": "DH5α",
  "growthConditions": {
    "temperature": 37,
    "medium": "LB",
    "antibiotics": ["chloramphenicol"],
    "inducers": []
  },
  "expression": {
    "expressionHost": "E. coli",
    "compartment": "cytoplasm",
    "tags": ["His6"],
    "fusionPartners": []
  }
}
```

### 3.4 Documentation

```json
{
  "description": "Detailed part description...",
  "designNotes": "Engineering considerations...",
  "references": [
    {
      "type": "publication",
      "doi": "10.1038/nature12345",
      "title": "Novel promoter characterization",
      "authors": ["Smith J", "Doe A"],
      "year": 2024
    }
  ],
  "externalLinks": [
    {
      "database": "GenBank",
      "accession": "AB123456"
    },
    {
      "database": "iGEM",
      "url": "http://parts.igem.org/Part:BBa_K123456"
    }
  ]
}
```

---

## 4. Sequence Formats

### 4.1 Supported Formats

The registry MUST support the following sequence formats:

#### 4.1.1 Raw DNA/RNA
Plain nucleotide sequence:
```
ATGCGTAAAGGAGAAGAACTTTTCACTGGAGTTGTCCCAATTCTTGTTGAATTAGATGGTGATGTTAATGGGCAC
```

#### 4.1.2 FASTA Format
```
>BBa_E0040 Green Fluorescent Protein
ATGCGTAAAGGAGAAGAACTTTTCACTGGAGTTGTCCCAATTCTTGTTGAATTAGATGGT
GATGTTAATGGGCACAAATTTTCTGTCAGTGGAGAGGGTGAAGGTGATGCAACATACGGA
AAACTTACCCTTAAATTTATTTGCACTACTGGAAAACTACCTGTTCCATGGCCAACACTT
```

#### 4.1.3 GenBank Format
```
LOCUS       BBa_E0040                720 bp    DNA     linear   SYN 15-JUN-2024
DEFINITION  Green Fluorescent Protein from jellyfish Aequorea victoria
ACCESSION   BBa_E0040
VERSION     BBa_E0040.1
KEYWORDS    .
SOURCE      synthetic construct
  ORGANISM  synthetic construct
FEATURES             Location/Qualifiers
     source          1..720
                     /organism="synthetic construct"
     CDS             1..720
                     /codon_start=1
                     /product="green fluorescent protein"
                     /protein_id="BBa_E0040"
                     /translation="MSKGEELFTGVVPILVELDGDVNGHKFSVSGEGEGDAT..."
ORIGIN
        1 atgcgtaaag gagaagaact tttcactgga gttgtcccaa ttcttgttga attagatggt
       61 gatgttaatg ggcacaaatt ttctgtcagt ggagagggtg aaggtgatgc aacatacgga
```

#### 4.1.4 SBOL (RDF/XML)
Synthetic Biology Open Language format for maximum interoperability.

### 4.2 Format Conversion

Implementations MUST provide bidirectional conversion between all supported formats.

### 4.3 Sequence Validation

All sequences MUST pass validation:

1. **Character Validation**: Only valid nucleotide codes (A, T, C, G, U, N, etc.)
2. **Length Validation**: Minimum 1 bp, maximum 1,000,000 bp
3. **Checksum Verification**: SHA-256 hash for integrity
4. **Feature Consistency**: Features must not extend beyond sequence boundaries
5. **Biological Validity**: Optional checks for forbidden sequences, restriction sites

---

## 5. SBOL Compliance

### 5.1 SBOL Version Support

Registry MUST support SBOL 2.3.0 or higher.

### 5.2 ComponentDefinition

Each part is represented as an SBOL ComponentDefinition:

```xml
<sbol:ComponentDefinition rdf:about="http://wiastandards.com/BBa_K123456">
    <sbol:persistentIdentity rdf:resource="http://wiastandards.com/BBa_K123456"/>
    <sbol:displayId>BBa_K123456</sbol:displayId>
    <sbol:version>1.2.0</sbol:version>
    <dcterms:title>Strong GFP Expression Device</dcterms:title>
    <dcterms:description>Composite part for constitutive GFP expression</dcterms:description>
    <sbol:type rdf:resource="http://www.biopax.org/release/biopax-level3.owl#DnaRegion"/>
    <sbol:role rdf:resource="http://identifiers.org/so/SO:0000804"/>
    <sbol:sequence rdf:resource="http://wiastandards.com/BBa_K123456_sequence"/>
</sbol:ComponentDefinition>
```

### 5.3 Sequence Object

```xml
<sbol:Sequence rdf:about="http://wiastandards.com/BBa_K123456_sequence">
    <sbol:persistentIdentity rdf:resource="http://wiastandards.com/BBa_K123456_sequence"/>
    <sbol:displayId>BBa_K123456_sequence</sbol:displayId>
    <sbol:version>1.2.0</sbol:version>
    <sbol:elements>ATGCGTAAAGGAGAAGAACTTTTC...</sbol:elements>
    <sbol:encoding rdf:resource="http://www.chem.qmul.ac.uk/iubmb/misc/naseq.html"/>
</sbol:Sequence>
```

### 5.4 Component Hierarchy

Composite parts MUST define component relationships:

```xml
<sbol:Component rdf:about="http://wiastandards.com/BBa_K123456/promoter">
    <sbol:definition rdf:resource="http://wiastandards.com/BBa_J23100"/>
    <sbol:access rdf:resource="http://sbols.org/v2#public"/>
</sbol:Component>
```

---

## 6. Characterization Data

### 6.1 Quantitative Measurements

Characterization data MUST include measurement context:

```json
{
  "characterization": {
    "measurements": [
      {
        "type": "fluorescence",
        "value": 15000,
        "unit": "RFU",
        "stdDev": 1200,
        "replicates": 3,
        "conditions": {
          "temperature": 37,
          "timePoint": 6,
          "timeUnit": "hours",
          "opticalDensity": 0.6
        },
        "method": "Plate reader - Tecan Infinite M200",
        "performedBy": "Lab Team A",
        "date": "2024-08-15"
      }
    ]
  }
}
```

### 6.2 Growth Curves

```json
{
  "growthCurve": {
    "dataPoints": [
      {"time": 0, "od600": 0.05},
      {"time": 1, "od600": 0.08},
      {"time": 2, "od600": 0.15},
      {"time": 3, "od600": 0.32},
      {"time": 4, "od600": 0.65}
    ],
    "maxGrowthRate": 0.42,
    "unit": "1/hour",
    "strain": "E. coli DH5α",
    "medium": "LB + chloramphenicol (25 μg/mL)"
  }
}
```

### 6.3 Expression Levels

For protein-coding parts:

```json
{
  "expression": {
    "transcriptLevel": {
      "value": 500,
      "unit": "copies/cell",
      "method": "qRT-PCR"
    },
    "proteinLevel": {
      "value": 10000,
      "unit": "molecules/cell",
      "method": "Western blot quantification"
    },
    "activity": {
      "specific": 1.5,
      "unit": "μmol/min/mg",
      "substrate": "IPTG"
    }
  }
}
```

### 6.4 Reusability Metrics

Track part usage and success:

```json
{
  "reusability": {
    "timesUsed": 45,
    "successRate": 0.89,
    "averageRating": 4.3,
    "reviews": [
      {
        "user": "researcher123",
        "rating": 5,
        "comment": "Works excellently in our system",
        "date": "2024-09-01"
      }
    ],
    "citations": 12
  }
}
```

---

## 7. Version Control

### 7.1 Semantic Versioning

Parts use semantic versioning: `MAJOR.MINOR.PATCH`

**Version Rules**:
- `MAJOR`: Incompatible sequence changes
- `MINOR`: Backward-compatible additions (new characterization data)
- `PATCH`: Bug fixes, documentation updates

**Examples**:
- `1.0.0` → `1.1.0`: Added characterization data
- `1.1.0` → `2.0.0`: Sequence optimization (incompatible)
- `1.1.0` → `1.1.1`: Fixed typo in description

### 7.2 Version History

```json
{
  "versionHistory": [
    {
      "version": "1.0.0",
      "date": "2024-06-15T10:30:00Z",
      "author": "initial-creator",
      "changes": "Initial part submission",
      "sequenceHash": "sha256:abc123..."
    },
    {
      "version": "1.1.0",
      "date": "2024-08-20T14:45:00Z",
      "author": "lab-member-2",
      "changes": "Added fluorescence characterization",
      "sequenceHash": "sha256:abc123..."
    },
    {
      "version": "2.0.0",
      "date": "2024-10-01T09:00:00Z",
      "author": "optimization-team",
      "changes": "Codon optimization for mammalian cells",
      "sequenceHash": "sha256:def456..."
    }
  ]
}
```

### 7.3 Deprecation Policy

When parts are deprecated:

```json
{
  "deprecation": {
    "deprecated": true,
    "date": "2024-11-15",
    "reason": "Superseded by BBa_K789012 with improved expression",
    "replacementPart": "BBa_K789012",
    "stillAvailable": true
  }
}
```

---

## 8. Access Control and Licensing

### 8.1 Licensing Options

Supported licenses:

| License | Code | Description |
|---------|------|-------------|
| CC-BY-4.0 | CC-BY | Attribution required |
| CC-BY-SA-4.0 | CC-BY-SA | Share-alike required |
| CC0-1.0 | CC0 | Public domain |
| MIT | MIT | Permissive software license |
| Proprietary | PROP | Custom terms required |
| OpenMTA | OMTA | Open Material Transfer Agreement |

### 8.2 Access Levels

```json
{
  "accessControl": {
    "visibility": "public|private|restricted",
    "permissions": {
      "view": ["public"],
      "download": ["registered-users"],
      "modify": ["part-owner", "admin"],
      "characterize": ["registered-users"]
    },
    "embargoUntil": "2025-01-01T00:00:00Z",
    "requiresAgreement": true,
    "agreementUrl": "https://example.com/mta.pdf"
  }
}
```

### 8.3 Material Transfer

```json
{
  "materialTransfer": {
    "available": true,
    "provider": "Addgene",
    "catalogNumber": "12345",
    "cost": 65.00,
    "currency": "USD",
    "shippingRestrictions": ["country:CN", "country:IR"],
    "requiresMTA": true
  }
}
```

---

## 9. Registry Interoperability

### 9.1 iGEM Registry Integration

**Import from iGEM**:
```
GET http://parts.igem.org/xml/part.BBa_K123456
```

**Export to iGEM**:
- Convert to iGEM XML format
- Map metadata fields
- Preserve part ID and version

### 9.2 Addgene Integration

**Plasmid Linking**:
```json
{
  "addgene": {
    "plasmidId": 12345,
    "url": "https://www.addgene.org/12345/",
    "availability": "available",
    "price": 65.00
  }
}
```

### 9.3 SynBioHub Compatibility

Full SBOL 2.3 compliance ensures compatibility with SynBioHub instances.

**Submit to SynBioHub**:
```bash
curl -X POST https://synbiohub.org/submit \
  -H "X-authorization: <token>" \
  -F "file=@part.xml"
```

### 9.4 NCBI GenBank

**Cross-reference**:
```json
{
  "genbank": {
    "accession": "MN123456",
    "gi": "1234567890",
    "url": "https://www.ncbi.nlm.nih.gov/nuccore/MN123456"
  }
}
```

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
