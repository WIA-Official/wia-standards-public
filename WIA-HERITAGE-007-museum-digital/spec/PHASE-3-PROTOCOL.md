# WIA-HERITAGE-007: Phase 3 - Protocol Specification

> 弘益人間 (Benefit All Humanity)

## Overview

Phase 3 defines communication protocols, data exchange rules, security measures, and blockchain integration for cultural heritage digitization.

## 1. 3D Streaming Protocol

### 1.1 Progressive Mesh Loading

```javascript
// Level of Detail (LOD) streaming
const lodLevels = [
  { distance: 0-10m, vertices: 5M, quality: "ultra" },
  { distance: 10-50m, vertices: 1M, quality: "high" },
  { distance: 50-100m, vertices: 100K, quality: "medium" },
  { distance: 100m+, vertices: 10K, quality: "low" }
];

// Streaming manifest
{
  "artifact": "art-007",
  "lodLevels": 4,
  "chunks": [
    {
      "level": 0,
      "url": "/models/art-007/lod0.bin",
      "vertices": 5000000,
      "size": 52428800
    },
    ...
  ],
  "streaming": {
    "chunkSize": 1048576,
    "compression": "draco",
    "priority": "distance-based"
  }
}
```

### 1.2 Texture Streaming

```yaml
texture_streaming:
  format: KTX2 + Basis Universal
  mipmap_levels: 11 (8K → 4px)
  streaming_strategy: quad-tree
  cache_policy: LRU
  bandwidth_adaptation: true
```

## 2. Provenance Verification Protocol

### 2.1 Blockchain Integration

```solidity
// Ethereum Smart Contract
pragma solidity ^0.8.0;

contract HeritageProvenance {
    struct ProvenanceRecord {
        string artifactId;
        address owner;
        string location;
        uint256 timestamp;
        bytes32 previousHash;
        string metadataURI;
    }

    mapping(string => ProvenanceRecord[]) public provenanceChain;

    event ProvenanceRecorded(
        string indexed artifactId,
        address indexed owner,
        uint256 timestamp
    );

    function recordProvenance(
        string memory artifactId,
        string memory location,
        string memory metadataURI
    ) public returns (bytes32) {
        ProvenanceRecord[] storage chain = provenanceChain[artifactId];
        bytes32 previousHash = chain.length > 0
            ? keccak256(abi.encode(chain[chain.length - 1]))
            : bytes32(0);

        ProvenanceRecord memory record = ProvenanceRecord({
            artifactId: artifactId,
            owner: msg.sender,
            location: location,
            timestamp: block.timestamp,
            previousHash: previousHash,
            metadataURI: metadataURI
        });

        chain.push(record);
        emit ProvenanceRecorded(artifactId, msg.sender, block.timestamp);

        return keccak256(abi.encode(record));
    }

    function verifyChain(string memory artifactId)
        public view returns (bool) {
        ProvenanceRecord[] storage chain = provenanceChain[artifactId];

        for (uint i = 1; i < chain.length; i++) {
            bytes32 computedHash = keccak256(abi.encode(chain[i-1]));
            if (computedHash != chain[i].previousHash) {
                return false;
            }
        }
        return true;
    }
}
```

### 2.2 Decentralized Storage

```yaml
storage_providers:
  primary: IPFS
  secondary: Arweave
  tertiary: Filecoin

ipfs_pinning:
  service: Pinata / Infura
  redundancy: 3 nodes
  metadata: JSON-LD
  images: TIFF/JPEG2000
  models: glTF

arweave_permanent:
  transaction_type: data
  tags:
    - name: Content-Type
      value: application/json
    - name: Standard
      value: WIA-HERITAGE-007
    - name: Artifact-ID
      value: art-007
```

## 3. Rights Management Protocol

### 3.1 Creative Commons Integration

```json
{
  "rights": {
    "license": {
      "type": "Creative Commons",
      "variant": "CC BY-SA 4.0",
      "url": "https://creativecommons.org/licenses/by-sa/4.0/",
      "commercial": true,
      "derivatives": true,
      "attribution": "National Museum of Athens",
      "shareAlike": true
    },
    "copyright": {
      "holder": "National Museum of Athens",
      "year": 2025,
      "statement": "© 2025 National Museum of Athens"
    },
    "access": {
      "level": "public",
      "restrictions": [],
      "embargoed": false
    },
    "reuse": {
      "allowDownload": true,
      "allowPrint": true,
      "allowModification": true,
      "allowCommercial": true,
      "requireAttribution": true
    }
  }
}
```

### 3.2 Digital Rights Management (DRM)

```yaml
drm_policy:
  watermarking:
    visible: optional
    invisible: required (steganography)
    metadata: embedded

  access_control:
    public: web viewer (no download)
    authenticated: low-res download
    researcher: high-res download
    institution: raw data access

  usage_tracking:
    downloads: logged
    views: analytics
    derivatives: tracked
```

## 4. Data Exchange Protocol

### 4.1 OAI-PMH Integration

```xml
<!-- Metadata Harvesting -->
<OAI-PMH xmlns="http://www.openarchives.org/OAI/2.0/">
  <responseDate>2025-01-15T14:30:00Z</responseDate>
  <request verb="GetRecord"
           identifier="oai:wia.org:heritage:art-007"
           metadataPrefix="oai_dc">
    https://api.wia.org/heritage/v1/oai
  </request>
  <GetRecord>
    <record>
      <header>
        <identifier>oai:wia.org:heritage:art-007</identifier>
        <datestamp>2025-01-15</datestamp>
        <setSpec>heritage:pottery</setSpec>
      </header>
      <metadata>
        <oai_dc:dc>
          <dc:title>Ancient Greek Amphora</dc:title>
          <dc:creator>Unknown</dc:creator>
          <dc:date>-450/-400</dc:date>
          <dc:type>PhysicalObject</dc:type>
        </oai_dc:dc>
      </metadata>
    </record>
  </GetRecord>
</OAI-PMH>
```

### 4.2 Linked Open Data (LOD)

```turtle
@prefix dc: <http://purl.org/dc/elements/1.1/> .
@prefix crm: <http://www.cidoc-crm.org/cidoc-crm/> .
@prefix wia: <http://wia.org/heritage/007#> .

<http://wia.org/artifact/art-007> a crm:E22_Human-Made_Object ;
    dc:title "Ancient Greek Amphora" ;
    crm:P45_consists_of <http://vocab.getty.edu/aat/300070669> ; # terracotta
    crm:P108i_was_produced_by [
        a crm:E12_Production ;
        crm:P4_has_time-span [
            crm:P82a_begin_of_the_begin "-450"^^xsd:gYear ;
            crm:P82b_end_of_the_end "-400"^^xsd:gYear
        ]
    ] ;
    wia:hasDigitalRepresentation <http://wia.org/model/model-007> .
```

## 5. Security Protocol

### 5.1 API Security

```yaml
authentication:
  method: JWT (RS256)
  expiration: 1 hour
  refresh_token: 30 days
  mfa: optional

encryption:
  in_transit: TLS 1.3
  at_rest: AES-256-GCM
  keys: AWS KMS / HashiCorp Vault

rate_limiting:
  algorithm: Token Bucket
  per_user: 1000/hour
  per_ip: 100/hour
  burst: 50

input_validation:
  sql_injection: prepared statements
  xss: content sanitization
  csrf: tokens required
```

### 5.2 Data Integrity

```javascript
// SHA-256 checksums for all files
{
  "artifact": "art-007",
  "files": [
    {
      "path": "models/archive.glb",
      "sha256": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
      "size": 52428800
    },
    {
      "path": "metadata/artifact.json",
      "sha256": "d4735e3a265e16eee03f59718b9b5d03019c07d8b6c51f90da3a666eec13ab35",
      "size": 4096
    }
  ],
  "manifest_hash": "a665a45920422f9d417e4867efdc4fb8a04a1f3fff1fa07e998e86f7f7a27ae3"
}
```

## 6. Interoperability Protocol

### 6.1 CrossRef Integration

```yaml
crossref_deposits:
  artifact_doi: 10.5555/wia.heritage.art.007
  model_doi: 10.5555/wia.heritage.model.007
  metadata: Crossmark
  funding: FundRef
```

### 6.2 Museum API Compatibility

```yaml
tms_integration:
  vendor: Gallery Systems
  api_version: v5
  sync: bidirectional
  fields: [accession_number, title, artist, date]

europeana_integration:
  api: Europeana Search + Record API
  format: EDM (Europeana Data Model)
  aggregation: automatic
```

## 7. Quality Assurance Protocol

### 7.1 Validation Rules

```json
{
  "validation": {
    "metadata": {
      "required_fields": ["id", "name", "period", "origin"],
      "dublin_core_compliance": true,
      "date_format": "ISO 8601",
      "coordinate_validation": "WGS84"
    },
    "models": {
      "manifold_geometry": true,
      "uv_range": [0, 1],
      "scale_units": "millimeters",
      "texture_resolution": "≥1024px"
    },
    "images": {
      "min_resolution": "2MP",
      "color_space": ["sRGB", "AdobeRGB"],
      "icc_profile": "embedded"
    }
  }
}
```

### 7.2 Automated Testing

```yaml
ci_cd_pipeline:
  - lint_metadata: JSON Schema validation
  - validate_models: glTF validator
  - check_images: ExifTool verification
  - test_api: Postman/Newman
  - security_scan: OWASP ZAP
  - performance: Lighthouse audit
```

## 8. Version Control Protocol

### 8.1 Semantic Versioning

```
artifact.{major}.{minor}.{patch}

major: Significant changes (re-scan, major restoration)
minor: Metadata updates, new images
patch: Typo fixes, minor corrections

Example: art-007.2.3.5
```

### 8.2 Change Log

```json
{
  "version": "2.3.5",
  "changes": [
    {
      "type": "metadata_update",
      "date": "2025-01-15",
      "description": "Updated dating based on new carbon analysis",
      "author": "Dr. Jane Smith",
      "approved_by": "Chief Curator"
    }
  ],
  "previous_versions": [
    {
      "version": "2.3.4",
      "url": "/artifacts/art-007/versions/2.3.4",
      "archived": true
    }
  ]
}
```

---

**Phase 3 Compliance**: All implementations must support progressive streaming, blockchain provenance, Creative Commons licensing, and data integrity verification.

**Next**: [Phase 4 - Integration](PHASE-4-INTEGRATION.md)

---

弘익人間 (Benefit All Humanity)
© 2025 SmileStory Inc. / WIA
