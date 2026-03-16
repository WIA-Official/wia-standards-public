# WIA-BIO-001: Integration & API Specification

## Version 1.0 | 2025-01-15

---

## Table of Contents
1. [Overview](#overview)
2. [REST API](#rest-api)
3. [GA4GH Standards Integration](#ga4gh-standards-integration)
4. [htsget Protocol](#htsget-protocol)
5. [GraphQL API](#graphql-api)
6. [Streaming & Real-time](#streaming--real-time)
7. [SDK & Client Libraries](#sdk--client-libraries)
8. [Interoperability](#interoperability)

---

## Overview

WIA-BIO-001 provides comprehensive APIs for programmatic access to genome sequencing data, ensuring interoperability with existing bioinformatics tools and compliance with global standards (GA4GH, FHIR, HL7).

### API Design Principles

| Principle | Implementation |
|-----------|----------------|
| **RESTful** | HTTP verbs, resource-based URLs |
| **Versioned** | `/v1/`, `/v2/` in URL path |
| **Authenticated** | OAuth 2.0, API keys |
| **Documented** | OpenAPI 3.0 spec |
| **Rate-Limited** | 1000 requests/hour (authenticated) |
| **Paginated** | Limit/offset or cursor-based |

---

## REST API

### Base URL

```
Production:  https://api.wia-bio.org/v1
Staging:     https://api-staging.wia-bio.org/v1
```

### Authentication

**OAuth 2.0 (Recommended):**
```bash
# Get access token
curl -X POST https://auth.wia-bio.org/oauth/token \
  -H "Content-Type: application/json" \
  -d '{
    "grant_type": "client_credentials",
    "client_id": "YOUR_CLIENT_ID",
    "client_secret": "YOUR_CLIENT_SECRET",
    "scope": "genomic_data.read"
  }'

# Response
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600
}

# Use token in requests
curl -H "Authorization: Bearer <access_token>" \
  https://api.wia-bio.org/v1/samples
```

**API Key (Alternative):**
```bash
curl -H "X-API-Key: your_api_key_here" \
  https://api.wia-bio.org/v1/samples
```

### Endpoints

#### 1. Samples

**List Samples**
```http
GET /v1/samples?limit=50&offset=0
```

```json
{
  "data": [
    {
      "id": "SAMPLE_001",
      "status": "sequenced",
      "platform": "ILLUMINA",
      "coverage": 45.2,
      "createdAt": "2025-01-15T10:00:00Z"
    }
  ],
  "pagination": {
    "total": 1523,
    "limit": 50,
    "offset": 0
  }
}
```

**Get Sample Details**
```http
GET /v1/samples/{sampleId}
```

```json
{
  "id": "SAMPLE_001",
  "patientId": "PATIENT_123",
  "sequencing": {
    "platform": "ILLUMINA",
    "instrument": "NovaSeq6000",
    "runDate": "2025-01-15",
    "library": {
      "prepKit": "TruSeq DNA PCR-Free",
      "insertSize": 350
    }
  },
  "quality": {
    "meanCoverage": 45.2,
    "pctQ30": 95.3,
    "pctMapped": 98.1,
    "contamination": 0.002
  },
  "files": {
    "fastq": [
      "s3://genomic-data/SAMPLE_001_R1.fastq.gz",
      "s3://genomic-data/SAMPLE_001_R2.fastq.gz"
    ],
    "bam": "s3://genomic-data/SAMPLE_001.bam",
    "vcf": "s3://genomic-data/SAMPLE_001.vcf.gz"
  }
}
```

**Create Sample**
```http
POST /v1/samples
Content-Type: application/json

{
  "id": "SAMPLE_002",
  "patientId": "PATIENT_124",
  "sequencing": {
    "platform": "PACBIO",
    "runDate": "2025-01-16"
  }
}
```

**Update Sample**
```http
PATCH /v1/samples/{sampleId}
Content-Type: application/json

{
  "quality": {
    "meanCoverage": 50.5
  }
}
```

**Delete Sample**
```http
DELETE /v1/samples/{sampleId}
```

#### 2. Variants

**Query Variants**
```http
GET /v1/samples/{sampleId}/variants?chrom=chr1&start=10000&end=20000&minQual=30
```

```json
{
  "data": [
    {
      "chrom": "chr1",
      "pos": 12345,
      "id": "rs12345",
      "ref": "A",
      "alt": ["G"],
      "qual": 99.0,
      "filter": "PASS",
      "info": {
        "DP": 100,
        "AF": [0.5]
      },
      "genotype": {
        "GT": "0/1",
        "GQ": 99,
        "DP": 100,
        "AD": [50, 50]
      },
      "annotations": {
        "gene": "GENE1",
        "consequence": "missense_variant",
        "clinicalSignificance": "likely_pathogenic"
      }
    }
  ]
}
```

**Annotate Variants**
```http
POST /v1/variants/annotate
Content-Type: application/json

{
  "variants": [
    {"chrom": "chr1", "pos": 12345, "ref": "A", "alt": "G"}
  ],
  "annotators": ["VEP", "ClinVar", "gnomAD"]
}
```

#### 3. Coverage

**Get Coverage Statistics**
```http
GET /v1/samples/{sampleId}/coverage?region=chr1:10000-20000&binSize=100
```

```json
{
  "region": "chr1:10000-20000",
  "binSize": 100,
  "data": [
    {"start": 10000, "end": 10100, "meanCoverage": 45.2},
    {"start": 10100, "end": 10200, "meanCoverage": 47.8}
  ],
  "summary": {
    "meanCoverage": 45.5,
    "medianCoverage": 45.0,
    "pctAbove30x": 98.2
  }
}
```

#### 4. Quality Metrics

**Get QC Report**
```http
GET /v1/samples/{sampleId}/qc
```

```json
{
  "sampleId": "SAMPLE_001",
  "timestamp": "2025-01-15T12:00:00Z",
  "readQuality": {
    "totalReads": 500000000,
    "pctQ30": 95.3,
    "meanQuality": 38.5,
    "gcContent": 41.5
  },
  "alignment": {
    "pctMapped": 98.1,
    "pctProperlyPaired": 96.5,
    "meanMAPQ": 58.2,
    "duplicateRate": 8.3
  },
  "coverage": {
    "meanDepth": 45.2,
    "medianDepth": 44.0,
    "pctAbove10x": 99.8,
    "pctAbove30x": 98.5
  },
  "variants": {
    "totalVariants": 4523678,
    "snps": 4123456,
    "indels": 400222,
    "tiTvRatio": 2.05
  },
  "status": "PASS"
}
```

### Error Handling

**Standard Error Response:**
```json
{
  "error": {
    "code": "INVALID_REGION",
    "message": "Invalid genomic region format",
    "details": "Region must be in format chr:start-end (e.g., chr1:10000-20000)",
    "timestamp": "2025-01-15T12:00:00Z",
    "requestId": "req_abc123"
  }
}
```

**HTTP Status Codes:**

| Code | Meaning | Use Case |
|------|---------|----------|
| 200 | OK | Successful GET/PATCH |
| 201 | Created | Successful POST |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Invalid parameters |
| 401 | Unauthorized | Missing/invalid auth |
| 403 | Forbidden | No permission |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

---

## GA4GH Standards Integration

### htsget Protocol

**Streaming access to genomic data over HTTP.**

**Get Reads (BAM/CRAM):**
```http
GET /ga4gh/htsget/v1/reads/{sampleId}?referenceName=chr1&start=10000&end=20000&format=BAM
```

**Response (Ticket):**
```json
{
  "htsget": {
    "format": "BAM",
    "urls": [
      {
        "url": "https://data.wia-bio.org/SAMPLE_001.bam?range=0-1000000",
        "headers": {
          "Authorization": "Bearer <token>",
          "Range": "bytes=0-1000000"
        }
      },
      {
        "url": "https://data.wia-bio.org/SAMPLE_001.bam?range=1000001-2000000"
      }
    ]
  }
}
```

**Get Variants (VCF):**
```http
GET /ga4gh/htsget/v1/variants/{sampleId}?referenceName=chr1&start=10000&end=20000&format=VCF
```

### Data Repository Service (DRS)

**Resolve data objects across repositories.**

```http
GET /ga4gh/drs/v1/objects/{object_id}
```

```json
{
  "id": "drs://wia-bio.org/abc123",
  "name": "SAMPLE_001.bam",
  "size": 54000000000,
  "created_time": "2025-01-15T10:00:00Z",
  "checksums": [
    {
      "type": "md5",
      "checksum": "8b1a9953c4611296a827abf8c47804d7"
    }
  ],
  "access_methods": [
    {
      "type": "https",
      "access_url": {
        "url": "https://data.wia-bio.org/SAMPLE_001.bam"
      }
    },
    {
      "type": "s3",
      "access_url": {
        "url": "s3://genomic-data/SAMPLE_001.bam"
      }
    }
  ]
}
```

### Phenopackets

**Clinical phenotype representation.**

```json
{
  "id": "PATIENT_123",
  "subject": {
    "id": "PATIENT_123",
    "sex": "MALE",
    "dateOfBirth": "1985-03-15"
  },
  "phenotypicFeatures": [
    {
      "type": {
        "id": "HP:0001250",
        "label": "Seizures"
      },
      "onset": {
        "age": "P5Y"
      }
    }
  ],
  "diseases": [
    {
      "term": {
        "id": "OMIM:607208",
        "label": "Dravet Syndrome"
      }
    }
  ],
  "htsFiles": [
    {
      "uri": "file:///data/SAMPLE_001.vcf.gz",
      "htsFormat": "VCF",
      "genomeAssembly": "GRCh38"
    }
  ]
}
```

---

## GraphQL API

### Endpoint

```
https://api.wia-bio.org/graphql
```

### Schema

```graphql
type Sample {
  id: ID!
  patientId: String
  platform: Platform!
  coverage: Float
  variants(filter: VariantFilter): [Variant!]!
  quality: QualityMetrics
}

type Variant {
  chrom: String!
  pos: Int!
  id: String
  ref: String!
  alt: [String!]!
  qual: Float
  filter: String
  genotype: Genotype
  annotations: [Annotation!]
}

type Genotype {
  GT: String
  GQ: Int
  DP: Int
  AD: [Int!]
}

type Query {
  sample(id: ID!): Sample
  samples(limit: Int, offset: Int): [Sample!]!
  variants(sampleId: ID!, region: Region!): [Variant!]!
}

input Region {
  chrom: String!
  start: Int!
  end: Int!
}

input VariantFilter {
  minQual: Float
  maxAF: Float
  consequences: [String!]
}
```

### Example Query

```graphql
query GetSampleWithVariants {
  sample(id: "SAMPLE_001") {
    id
    coverage
    variants(filter: {minQual: 30, consequences: ["missense_variant"]}) {
      chrom
      pos
      ref
      alt
      genotype {
        GT
        GQ
      }
      annotations {
        gene
        consequence
        clinicalSignificance
      }
    }
  }
}
```

**Response:**
```json
{
  "data": {
    "sample": {
      "id": "SAMPLE_001",
      "coverage": 45.2,
      "variants": [
        {
          "chrom": "chr1",
          "pos": 12345,
          "ref": "A",
          "alt": ["G"],
          "genotype": {
            "GT": "0/1",
            "GQ": 99
          },
          "annotations": [
            {
              "gene": "GENE1",
              "consequence": "missense_variant",
              "clinicalSignificance": "likely_pathogenic"
            }
          ]
        }
      ]
    }
  }
}
```

---

## Streaming & Real-time

### Server-Sent Events (SSE)

**Stream sequencing progress:**
```http
GET /v1/samples/{sampleId}/stream
Accept: text/event-stream
```

```
event: progress
data: {"stage": "alignment", "percent": 45, "eta": 3600}

event: progress
data: {"stage": "variant_calling", "percent": 10, "eta": 7200}

event: complete
data: {"status": "success", "vcf": "s3://data/SAMPLE_001.vcf.gz"}
```

### WebSocket

**Real-time variant stream:**
```javascript
const ws = new WebSocket('wss://api.wia-bio.org/v1/variants/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    sampleId: 'SAMPLE_001',
    filter: { minQual: 30 }
  }));
};

ws.onmessage = (event) => {
  const variant = JSON.parse(event.data);
  console.log(`New variant: ${variant.chrom}:${variant.pos} ${variant.ref}>${variant.alt}`);
};
```

---

## SDK & Client Libraries

### Python SDK

**Installation:**
```bash
pip install wia-bio-sdk
```

**Usage:**
```python
from wia_bio import Client

# Initialize client
client = Client(api_key='your_api_key')

# Get sample
sample = client.samples.get('SAMPLE_001')
print(f"Coverage: {sample.quality.mean_coverage}x")

# Query variants
variants = client.variants.query(
    sample_id='SAMPLE_001',
    region='chr1:10000-20000',
    min_qual=30
)

for v in variants:
    print(f"{v.chrom}:{v.pos} {v.ref}>{v.alt[0]} (GQ={v.genotype.GQ})")

# Stream alignments
with client.alignments.stream('SAMPLE_001', region='chr1:10000-20000') as stream:
    for read in stream:
        print(f"Read: {read.query_name} at {read.pos}")
```

### JavaScript/TypeScript SDK

**Installation:**
```bash
npm install @wia/bio-sdk
```

**Usage:**
```typescript
import { WIABioClient } from '@wia/bio-sdk';

const client = new WIABioClient({ apiKey: 'your_api_key' });

// Get sample
const sample = await client.samples.get('SAMPLE_001');
console.log(`Coverage: ${sample.quality.meanCoverage}x`);

// Query variants
const variants = await client.variants.query({
  sampleId: 'SAMPLE_001',
  region: { chrom: 'chr1', start: 10000, end: 20000 },
  minQual: 30
});

variants.forEach(v => {
  console.log(`${v.chrom}:${v.pos} ${v.ref}>${v.alt[0]}`);
});
```

### R SDK

**Installation:**
```r
install.packages("devtools")
devtools::install_github("WIA-Official/wia-bio-r")
```

**Usage:**
```r
library(wiabio)

# Initialize client
client <- wia_bio_client(api_key = "your_api_key")

# Get sample
sample <- get_sample(client, "SAMPLE_001")
print(paste("Coverage:", sample$quality$mean_coverage))

# Query variants
variants <- query_variants(
  client,
  sample_id = "SAMPLE_001",
  region = "chr1:10000-20000",
  min_qual = 30
)

# Convert to VCF object
vcf <- as_vcf(variants)
```

---

## Interoperability

### FHIR Integration

**Genomic data as FHIR resources (R4):**

```json
{
  "resourceType": "Observation",
  "id": "variant-001",
  "status": "final",
  "code": {
    "coding": [
      {
        "system": "http://loinc.org",
        "code": "69548-6",
        "display": "Genetic variant assessment"
      }
    ]
  },
  "subject": {
    "reference": "Patient/PATIENT_123"
  },
  "valueCodeableConcept": {
    "coding": [
      {
        "system": "http://varnomen.hgvs.org",
        "code": "NC_000001.11:g.12345A>G"
      }
    ]
  },
  "component": [
    {
      "code": {
        "coding": [{"system": "http://loinc.org", "code": "48018-6", "display": "Gene studied"}]
      },
      "valueCodeableConcept": {
        "coding": [{"system": "http://www.genenames.org", "code": "GENE1"}]
      }
    }
  ]
}
```

### HL7 v2 ORU^R01 (Lab Results)

```
MSH|^~\&|GENOMICS_LAB|WIA|EMR|HOSPITAL|20250115120000||ORU^R01|MSG123|P|2.5.1
PID|1||PATIENT_123||DOE^JOHN||19850315|M
OBR|1||ORDER_001|GENOME_SEQ|||20250115
OBX|1|ST|GENE1^Variant||c.12345A>G||||||F
OBX|2|ST|CLIN_SIG^Clinical Significance||Likely Pathogenic||||||F
```

### CSV/TSV Export

**Export variants:**
```http
GET /v1/samples/{sampleId}/variants/export?format=tsv
```

```tsv
chrom	pos	id	ref	alt	qual	filter	GT	GQ	gene	consequence
chr1	12345	rs12345	A	G	99	PASS	0/1	99	GENE1	missense_variant
chr1	67890	.	C	T	85	PASS	1/1	85	GENE2	synonymous_variant
```

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA BIO Working Group
**License:** Apache 2.0
