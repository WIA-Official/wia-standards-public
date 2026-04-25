# Chapter 7: Data Standards and Interoperability

## WIA-SYNTHETIC-BIO-REGISTRY Standard
**Version:** 1.0
**Status:** Official WIA Standard
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 7.1 The Interoperability Imperative

As biological part registries proliferate and synthetic biology tools become more sophisticated, the ability of different systems to exchange information seamlessly becomes critical. Interoperability—the capacity of systems to work together—determines whether synthetic biology will thrive as an integrated ecosystem or fragment into incompatible silos.

### The Cost of Incompatibility

Without standardized data formats and interfaces, the synthetic biology community faces significant challenges:

**Duplicated Effort:**
Researchers must manually re-enter data when moving between tools. A genetic circuit designed in one software package cannot be directly imported into another, requiring reconstruction from scratch.

**Data Loss:**
Critical information gets lost in translation between systems. Characterization data captured in one registry may not transfer completely to another, reducing utility.

**Tool Fragmentation:**
Design tools, registries, and lab automation systems operate in isolation, requiring human intermediaries to bridge gaps. This slows workflows and introduces errors.

**Reduced Innovation:**
Developers waste time on data conversion rather than building new capabilities. Lack of interoperability raises barriers to entry for new tools and registries.

**Limited Reproducibility:**
Without standard data exchange, reproducing designs across laboratories becomes difficult. Critical details may not transfer between systems.

### The Vision of Seamless Integration

The alternative—a fully interoperable ecosystem—offers transformative possibilities:

**Design-to-Build Pipeline:**
```
1. Designer uses CAD tool to create genetic circuit
2. Software queries registry for optimal parts
3. Designer selects parts and optimizes design
4. Design exported in standard format
5. Automated DNA assembly system imports design
6. Lab robot constructs physical DNA
7. Characterization data flows back to registry
8. Updated information available to all users
```

**Cross-Registry Search:**
```
User Query: "Strong constitutive promoters for E. coli, Tier 3+"

System searches:
- iGEM Registry (20,000 parts)
- AddGene (50,000 plasmids)
- Benchling Public Registry (10,000 parts)
- JBEI Public Registry (5,000 parts)
- SynBioHub instances (distributed)

Returns unified results with:
- Part information from all sources
- Standardized quality metrics
- Comparable characterization data
- Direct links to obtain each part
```

**Computational Design Integration:**
```
AI-based design tool:
1. Receives specification (desired circuit behavior)
2. Queries registry APIs for candidate parts
3. Downloads characterization data in standard format
4. Runs simulations using standardized models
5. Optimizes design
6. Outputs design in SBOL format
7. User imports to lab management system
8. Experimental results flow back to train AI
```

## 7.2 Core Data Standards

Several key standards enable interoperability in synthetic biology:

### SBOL (Synthetic Biology Open Language)

SBOL is the leading standard for representing synthetic biology designs:

**Purpose:**
Enable exchange of genetic designs between tools, registries, and researchers.

**Format:**
RDF/XML or JSON-LD (human and machine readable)

**Core Concepts:**

**ComponentDefinition:**
Describes a biological part or composite device.

```xml
<sbol:ComponentDefinition rdf:about="https://parts.igem.org/BBa_J23100">
    <sbol:displayId>BBa_J23100</sbol:displayId>
    <sbol:version>1</sbol:version>
    <sbol:type rdf:resource="http://www.biopax.org/release/biopax-level3.owl#DnaRegion"/>
    <sbol:role rdf:resource="http://identifiers.org/so/SO:0000167"/>
    <dcterms:title>Constitutive promoter (Anderson)</dcterms:title>
    <dcterms:description>Strong constitutive promoter from Anderson library</dcterms:description>

    <sbol:sequence rdf:resource="https://parts.igem.org/BBa_J23100_seq"/>

    <sbol:hasAnnotation>
        <sbol:Annotation>
            <sbol:propertyValue>
                <sbol:name>expression_level</sbol:name>
                <sbol:value>1.0</sbol:value>
            </sbol:propertyValue>
        </sbol:Annotation>
    </sbol:hasAnnotation>
</sbol:ComponentDefinition>

<sbol:Sequence rdf:about="https://parts.igem.org/BBa_J23100_seq">
    <sbol:elements>ttgacggctagctcagtcctaggtacagtgctagc</sbol:elements>
    <sbol:encoding rdf:resource="http://www.chem.qmul.ac.uk/iubmb/misc/naseq.html"/>
</sbol:Sequence>
```

**Component:**
Represents an instance of a ComponentDefinition in a specific design.

**Module:**
Groups components with defined interactions.

**Interaction:**
Describes relationships between components (inhibition, stimulation, etc.).

**Model:**
Links mathematical or computational models to designs.

**SBOL Visual:**
Graphical symbols for parts, enabling visual standardization.

```
Standard Glyphs:
→   Promoter
⭘   RBS
▭   CDS
⊤   Terminator
◇   Operator
○▷  Ori (Origin of replication)
◀   Primer binding site
🔶  Insulator
```

**SBOL Versions:**

**SBOL 1.0 (2011):**
- Basic structural description
- DNA components only
- Simple hierarchy

**SBOL 2.0 (2015):**
- Added proteins, RNA, small molecules
- Interactions and functional descriptions
- Models and experimental data
- Significant community adoption

**SBOL 3.0 (2020):**
- Simplified data model
- Improved constraints and measures
- Better performance metrics
- Enhanced experimental data representation
- Greater flexibility

### GenBank Format

The classic sequence annotation format from NCBI:

**Advantages:**
- Universally recognized
- Supported by all sequence viewers
- Rich feature annotation capability
- Human-readable

**Limitations:**
- Primarily sequence-centric
- Limited structured metadata
- Not designed for synthetic designs
- No standard for characterization data

**Example:**
```
LOCUS       BBa_J23100                35 bp    DNA     linear   SYN 15-JUL-2006
DEFINITION  Constitutive promoter (Anderson family member 100).
ACCESSION   BBa_J23100
VERSION     BBa_J23100.1
KEYWORDS    BioBrick; promoter; constitutive; Anderson; synthetic.
SOURCE      synthetic DNA construct
  ORGANISM  synthetic DNA construct
            other sequences; artificial sequences.
REFERENCE   1  (bases 1 to 35)
  AUTHORS   Anderson,J.C.
  TITLE     Anderson Promoter Collection
  JOURNAL   Parts Registry (2006)
FEATURES             Location/Qualifiers
     regulatory      1..35
                     /regulatory_class="promoter"
                     /label="J23100"
                     /note="Constitutive promoter, strong expression"
                     /standard_name="Anderson J23100"
                     /ApEinfo_fwdcolor="#00a1ff"
                     /ApEinfo_revcolor="#00a1ff"
     -35_signal      1..6
                     /note="TTGACG; -35 box"
                     /label="-35"
     -10_signal      28..33
                     /note="TATAAT; -10 box (consensus)"
                     /label="-10"
ORIGIN
        1 ttgacggcta gctcagtcct aggtacagtg ctagc
//
```

### FASTA Format

Simple sequence format, widely compatible:

```
>BBa_J23100 Constitutive promoter (Anderson)
TTGACGGCTAGCTCAGTCCTAGGTACAGTGCTAGC
```

**Advantages:**
- Extremely simple
- Universal support
- Easy to parse
- Lightweight

**Limitations:**
- Sequence only, no annotations
- Minimal metadata
- Not suitable for complex designs

### JSON Schema for Registry APIs

Modern APIs use JSON for data exchange:

```json
{
  "part_id": "BBa_J23100",
  "name": "Constitutive promoter (Anderson)",
  "type": "promoter",
  "subtype": "constitutive",
  "sequence": "TTGACGGCTAGCTCAGTCCTAGGTACAGTGCTAGC",
  "length": 35,
  "organism": "Escherichia coli",
  "chassis": ["E. coli K12", "E. coli BL21"],
  "quality_tier": 4,
  "characterization": {
    "expression_level": 1.0,
    "expression_unit": "relative to BBa_J23101",
    "replicates": 3,
    "standard_deviation": 0.08,
    "conditions": {
      "media": "LB",
      "temperature": 37,
      "temperature_unit": "Celsius",
      "growth_phase": "exponential"
    }
  },
  "assembly_compatibility": ["BioBrick", "Gibson", "Type IIS"],
  "metadata": {
    "author": "John Anderson",
    "creation_date": "2006-07-15",
    "last_modified": "2024-01-10",
    "license": "OpenMTA",
    "safety_level": 1,
    "references": [
      {
        "title": "Anderson Promoter Collection",
        "doi": "10.xxxx/xxxxx",
        "pmid": 12345678
      }
    ]
  },
  "links": {
    "registry": "https://parts.igem.org/Part:BBa_J23100",
    "sequence_download": "https://parts.igem.org/api/v2/sequences/BBa_J23100",
    "characterization_data": "https://parts.igem.org/api/v2/characterization/BBa_J23100"
  }
}
```

### ISA-TAB for Experimental Metadata

Investigation-Study-Assay format for experimental metadata:

**Structure:**
- **Investigation:** Overall project context
- **Study:** Experimental design and samples
- **Assay:** Measurements and data files

**Application to Registry:**
```
Investigation: Anderson Promoter Characterization
  Study: Promoter Strength Measurement
    Assay: GFP Fluorescence (Plate Reader)
      Sample: E. coli + BBa_J23100 + GFP
      Protocol: InterLab 2024
      Data File: exp_2024_001_raw.csv
```

**Advantages:**
- Captures rich experimental context
- Links data files to metadata
- Supports complex experimental designs
- Used in major biological databases

## 7.3 Ontologies and Controlled Vocabularies

Standardized terminology enables consistent data annotation:

### Sequence Ontology (SO)

Defines genomic feature types:

**Key Terms for Registry:**
- SO:0000167 - promoter
- SO:0000139 - ribosome_entry_site
- SO:0000316 - CDS (coding sequence)
- SO:0000141 - terminator
- SO:0000296 - origin_of_replication
- SO:0000110 - sequence_feature

**Usage:**
```json
{
  "part_id": "BBa_J23100",
  "sequence_ontology": {
    "id": "SO:0001203",
    "term": "constitutive_promoter",
    "definition": "A promoter that is continually active"
  }
}
```

### Systems Biology Ontology (SBO)

Describes mathematical and systems biology concepts:

**Quantitative Parameters:**
- SBO:0000064 - promoter strength
- SBO:0000186 - maximal transcription rate
- SBO:0000356 - decay constant
- SBO:0000009 - kinetic constant

**Model Types:**
- SBO:0000293 - non-spatial continuous framework
- SBO:0000062 - continuous framework

### Gene Ontology (GO)

Describes biological functions:

**Molecular Function:**
- GO:0003700 - DNA-binding transcription factor activity
- GO:0004519 - endonuclease activity
- GO:0016491 - oxidoreductase activity

**Biological Process:**
- GO:0006351 - transcription, DNA-templated
- GO:0006412 - translation
- GO:0009058 - biosynthetic process

### Units of Measurement Ontology (UO)

Standardizes units:

- UO:0000189 - count unit (e.g., molecules/cell)
- UO:0000308 - fluorescence unit
- UO:0000027 - degree Celsius
- UO:0000062 - molar concentration

## 7.4 Registry APIs and Data Exchange

Programmatic interfaces enable automated data access:

### RESTful API Design Principles

**Resource-Oriented:**
```
GET /parts/BBa_J23100
GET /parts?type=promoter&tier=3
POST /parts (create new)
PUT /parts/BBa_J23100 (update)
DELETE /parts/BBa_J23100 (remove)
```

**Stateless:**
Each request contains all necessary information; server maintains no session state.

**Standard HTTP Methods:**
- GET: Retrieve data
- POST: Create new resource
- PUT: Update existing resource
- PATCH: Partial update
- DELETE: Remove resource

**HTTP Status Codes:**
- 200 OK: Success
- 201 Created: New resource created
- 400 Bad Request: Invalid input
- 401 Unauthorized: Authentication required
- 404 Not Found: Resource doesn't exist
- 429 Too Many Requests: Rate limit exceeded
- 500 Internal Server Error: Server problem

### Example API Specification (OpenAPI/Swagger)

```yaml
openapi: 3.0.0
info:
  title: WIA Biological Registry API
  version: 2.0.0
  description: API for accessing biological part registry data

servers:
  - url: https://api.registry.wia.org/v2

paths:
  /parts/{part_id}:
    get:
      summary: Get part information
      parameters:
        - name: part_id
          in: path
          required: true
          schema:
            type: string
          example: BBa_J23100
      responses:
        '200':
          description: Part found
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Part'
        '404':
          description: Part not found

  /parts/search:
    get:
      summary: Search for parts
      parameters:
        - name: type
          in: query
          schema:
            type: string
            enum: [promoter, rbs, cds, terminator]
        - name: tier
          in: query
          schema:
            type: integer
            minimum: 0
            maximum: 4
        - name: organism
          in: query
          schema:
            type: string
      responses:
        '200':
          description: Search results
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/PartSummary'

components:
  schemas:
    Part:
      type: object
      properties:
        part_id:
          type: string
        name:
          type: string
        sequence:
          type: string
        type:
          type: string
        quality_tier:
          type: integer
        characterization:
          $ref: '#/components/schemas/Characterization'

    Characterization:
      type: object
      properties:
        expression_level:
          type: number
        expression_unit:
          type: string
        replicates:
          type: integer
```

### GraphQL Alternative

For more flexible queries:

```graphql
# Query specific fields only
query GetPart($partId: String!) {
  part(id: $partId) {
    partId
    name
    sequence
    characterization {
      expressionLevel
      conditions {
        media
        temperature
      }
    }
  }
}

# Nested query across relationships
query GetCircuit($circuitId: String!) {
  circuit(id: $circuitId) {
    name
    components {
      partId
      name
      type
      sequence
    }
    interactions {
      type
      source {
        partId
      }
      target {
        partId
      }
    }
  }
}
```

**Advantages:**
- Client specifies exact data needed
- Single request can fetch complex nested data
- Strongly typed schema
- Reduced over-fetching and under-fetching

**Disadvantages:**
- More complex to implement
- Caching more difficult
- Learning curve for clients

### Bulk Data Export

For researchers needing complete datasets:

```bash
# Export all parts (JSON)
curl "https://api.registry.wia.org/v2/export/parts?format=json" > registry_parts.json

# Export with filters
curl "https://api.registry.wia.org/v2/export/parts?type=promoter&tier=3&format=csv" > promoters_tier3.csv

# Export in SBOL format
curl "https://api.registry.wia.org/v2/export/parts?format=sbol" > registry.sbol

# Export characterization data
curl "https://api.registry.wia.org/v2/export/characterization?format=json" > characterization_data.json
```

## 7.5 Tool Integration Examples

Real-world integration between registries and design tools:

### CAD Tool Integration

**Benchling + Registry:**
```
Workflow:
1. User designs circuit in Benchling
2. Searches registry from within Benchling interface
3. Drags part from search results into design
4. Sequence and metadata automatically imported
5. Complete design exported as SBOL
6. SBOL file contains registry part IDs
7. Lab can order exact parts for construction
```

**Implementation:**
```javascript
// Benchling API calls registry
async function searchRegistry(query) {
    const response = await fetch(
        `https://api.registry.wia.org/v2/parts/search?query=${query}`,
        { headers: { 'Authorization': `Bearer ${API_KEY}` }}
    );
    const parts = await response.json();

    // Convert to Benchling format
    return parts.map(part => ({
        name: part.name,
        bases: part.sequence,
        annotations: convertFeatures(part),
        registryId: part.part_id,
        registryUrl: part.links.registry
    }));
}
```

### LIMS Integration

**Laboratory Information Management System + Registry:**
```
Workflow:
1. Lab order includes registry part IDs
2. LIMS queries registry for part details
3. Generates protocols automatically
4. Tracks samples through workflow
5. Captures characterization data
6. Submits data back to registry via API
```

**Implementation:**
```python
# LIMS uploads characterization data to registry
import requests

def submit_characterization(part_id, data):
    url = f"https://api.registry.wia.org/v2/characterization/{part_id}"

    payload = {
        "experiment_id": data['exp_id'],
        "measurement_type": "fluorescence",
        "values": data['measurements'],
        "conditions": {
            "media": data['media'],
            "temperature": data['temperature'],
            "growth_phase": data['phase']
        },
        "protocol_doi": "10.17504/protocols.io.xxxxx",
        "replicate_count": len(data['measurements']),
        "measured_by": data['researcher'],
        "institution": data['institution']
    }

    response = requests.post(
        url,
        json=payload,
        headers={"Authorization": f"Bearer {API_KEY}"}
    )

    return response.json()
```

### Automation Platform Integration

**Opentrons + Registry:**
```
Workflow:
1. User specifies parts to assemble
2. Software queries registry for sequences
3. Generates assembly plan
4. Creates robot protocol
5. Robot assembles parts
6. Results verified and logged
```

## 7.6 SynBioHub: Federated Registry Platform

SynBioHub implements distributed registry architecture:

**Architecture:**
```
                    Internet
                        │
        ┌───────────────┼───────────────┐
        │               │               │
   [Hub Instance A] [Hub Instance B] [Hub Instance C]
        │               │               │
   Parts 1-5000    Parts 5001-10000   Specialized
   General Purpose  University Lab      Collection
```

**Features:**

**Distributed Storage:**
- Multiple independent SynBioHub instances
- Each maintains its own parts
- Federated search across instances
- No central authority required

**SBOL Native:**
- Native SBOL storage and retrieval
- Visual circuit rendering
- SBOL Visual standard compliance

**Web Interface:**
- Browse and search parts
- Upload new parts and collections
- Version control
- Permissions management

**API:**
```
# Search across federated instances
GET /search?query=promoter&collection=all

# Retrieve part in SBOL format
GET /parts/BBa_J23100/sbol

# Submit new part
POST /submit
Content-Type: application/sbol+xml
```

**Plugins:**
- Sequence alignment
- BLAST search
- Export to CAD tools
- Custom visualizations

**Instances (2025):**
- SynBioHub.org (public, MIT)
- EU-SynBioHub (Europe-focused)
- iGEM (competition parts)
- JBEI (DoE Bioenergy Research Center)
- University instances (50+)
- Company internal instances (proprietary)

## 7.7 Machine-Readable Characterization Data

Standardizing experimental data enables computational analysis:

### SBOL-Data Extension

SBOL 3.0 includes experimental data representation:

```xml
<sbol:Experiment rdf:about="https://registry.org/experiment/EXP001">
    <sbol:displayId>EXP001</sbol:displayId>
    <dcterms:title>Promoter Strength Measurement</dcterms:title>

    <sbol:measure>
        <sbol:Measure>
            <sbol:type rdf:resource="https://sbo.org/#SBO:0000064"/>
            <sbol:value>1.24</sbol:value>
            <sbol:unit rdf:resource="https://identifiers.org/UO:000186"/>
        </sbol:Measure>
    </sbol:measure>

    <sbol:experimentalData>
        <sbol:ExperimentalData>
            <sbol:dataFile rdf:resource="https://registry.org/data/EXP001_raw.csv"/>
        </sbol:ExperimentalData>
    </sbol:experimentalData>
</sbol:Experiment>
```

### PROV-O for Provenance

Tracking data origin and transformations:

```turtle
@prefix prov: <http://www.w3.org/ns/prov#> .
@prefix registry: <https://registry.org/> .

registry:BBa_J23100_measurement_2024
    a prov:Entity ;
    prov:wasGeneratedBy registry:experiment_20240115 ;
    prov:wasDerivedFrom registry:BBa_J23100 .

registry:experiment_20240115
    a prov:Activity ;
    prov:used registry:protocol_interlab_2024 ;
    prov:wasAssociatedWith registry:researcher_jsmith ;
    prov:startedAtTime "2024-01-15T09:00:00Z"^^xsd:dateTime ;
    prov:endedAtTime "2024-01-15T17:00:00Z"^^xsd:dateTime .

registry:researcher_jsmith
    a prov:Agent ;
    prov:actedOnBehalfOf registry:university_example .
```

### COMBINE Archive

Package for computational models and data:

```
archive.omex (ZIP format)
├── manifest.xml (describes contents)
├── model.xml (SBML model)
├── data.csv (experimental data)
├── protocol.md (experimental protocol)
├── circuit.sbol (genetic design)
└── metadata.rdf (provenance and attribution)
```

## 7.8 Future Directions

### AI-Readable Registries

Structured data enables AI/ML applications:

```python
# AI queries registry for training data
def fetch_training_data():
    # Get all promoters with characterization
    promoters = registry.search(type='promoter', has_data=True)

    training_set = []
    for promoter in promoters:
        # Sequence features
        sequence = promoter.sequence
        gc_content = calculate_gc(sequence)
        motifs = identify_motifs(sequence)

        # Experimental data
        expression = promoter.characterization.expression_level
        conditions = promoter.characterization.conditions

        training_set.append({
            'features': [gc_content] + motifs,
            'label': expression,
            'metadata': conditions
        })

    return training_set

# Train predictive model
model = train_expression_predictor(fetch_training_data())

# Use model to predict novel sequences
new_sequence = "ATGCGTAAAGGAGAA..."
predicted_expression = model.predict(new_sequence)
```

### Blockchain for Provenance

Immutable record of part history:

```
Block 1: Part submission (BBa_J23100) by Anderson, 2006-07-15
Block 2: Characterization data added by Chen, 2008-03-22
Block 3: Part used in Circuit X by Lee, 2010-06-10
Block 4: Part optimized → BBa_K123456 by Smith, 2015-09-03
Block 5: Multi-lab validation by Consortium, 2023-01-15

Each block cryptographically linked to previous
Tampering with history immediately detectable
Transparent attribution and credit assignment
```

### Semantic Web Integration

Linking registries to broader knowledge graphs:

```
Registry Parts ←→ UniProt (proteins)
              ←→ ChEBI (small molecules)
              ←→ GO (biological functions)
              ←→ PubMed (publications)
              ←→ Patent Databases
              ←→ Organism Databases
              ←→ Metabolic Pathway Databases

Enables:
- Complex cross-database queries
- Automated knowledge discovery
- Inference of novel relationships
- Comprehensive biological context
```

---

## Key Takeaways

✓ Interoperability enables seamless data exchange between registries, design tools, and lab systems
✓ SBOL is the primary standard for representing genetic designs in synthetic biology
✓ Ontologies (SO, SBO, GO) provide standardized terminology for consistent annotation
✓ RESTful APIs enable programmatic access to registry data
✓ Tool integration (CAD, LIMS, automation) streamlines design-build-test cycles
✓ SynBioHub provides federated registry architecture for distributed storage
✓ Machine-readable characterization data enables computational analysis and AI applications
✓ Standards continue evolving to meet community needs (SBOL 3.0, provenance tracking)
✓ Future directions include AI-readable formats, blockchain provenance, and semantic web integration
✓ WIA standard embraces open, interoperable data exchange formats

---

**Next Chapter:** Chapter 8 explores the future of bio-registry systems, examining emerging technologies and long-term visions for standardized biology.

---

*WIA-SYNTHETIC-BIO-REGISTRY Standard v1.0*
*© 2025 SmileStory Inc. / WIA*
*弘익人間 (Benefit All Humanity)*
