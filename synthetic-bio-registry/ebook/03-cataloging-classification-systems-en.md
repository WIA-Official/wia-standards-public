# Chapter 3: Cataloging and Classification Systems

## WIA-SYNTHETIC-BIO-REGISTRY Standard
**Version:** 1.0
**Status:** Official WIA Standard
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 3.1 The Importance of Systematic Classification

As biological part registries have grown from hundreds to tens of thousands of entries, effective cataloging and classification systems have become essential. Without systematic organization, users cannot efficiently discover relevant parts, and the registry's value diminishes despite its growing size.

### Classification Challenges in Biology

Unlike manufactured components with discrete, well-defined functions, biological parts present unique classification challenges:

**Functional Overlap:** A single DNA sequence may serve multiple functions depending on context. A strong promoter in one organism may be weak in another.

**Hierarchical Complexity:** Parts exist at multiple levels of organization—from individual motifs to complete pathways—requiring nested classification schemes.

**Context Dependency:** Part behavior varies with chassis organism, genetic context, and environmental conditions, complicating simple categorization.

**Evolutionary Relationships:** Natural sequences carry phylogenetic relationships that may or may not reflect functional similarity.

**Emergent Properties:** Combined parts may exhibit behaviors not predictable from individual components, requiring composite-level classification.

### Goals of Classification Systems

Effective classification systems for biological parts must achieve several objectives:

**Discoverability:** Users should easily find parts relevant to their needs through browsing or searching.

**Interoperability:** Classifications should facilitate cross-registry searches and integration with design tools.

**Extensibility:** New part types and functions should integrate without restructuring existing classifications.

**Precision:** Classifications should be specific enough to distinguish functionally different parts while avoiding excessive granularity.

**Accessibility:** Classification schemes should be understandable to both experts and novice users.

## 3.2 Hierarchical Classification Frameworks

Most biological part registries employ hierarchical classification systems with multiple levels of granularity.

### Primary Functional Categories

The top level distinguishes broad functional classes:

#### Regulatory Parts
Control gene expression timing, level, and conditions
**Examples:** Promoters, operators, enhancers, insulators, terminators

#### Coding Parts
Encode functional products (proteins or RNA molecules)
**Examples:** Genes, open reading frames, RNA coding sequences

#### Recognition Sites
Provide binding or recognition specificity
**Examples:** Restriction sites, recombinase sites, primer binding sites, operator sequences

#### Assembly Components
Facilitate construction and manipulation
**Examples:** Origins of replication, selection markers, multiple cloning sites

#### Structural Parts
Provide physical organization or scaffolding
**Examples:** Spacers, insulators, linker sequences, scaffold attachment regions

### Secondary Subtype Classification

Each primary category subdivides into specific subtypes:

#### Regulatory Parts → Promoters
**Constitutive Promoters:**
- Strong (>1.0 relative units)
- Medium (0.1-1.0 relative units)
- Weak (<0.1 relative units)

**Inducible Promoters:**
- Small molecule inducible (IPTG, arabinose, aTc)
- Environmental inducible (temperature, pH, oxygen)
- Quorum sensing responsive
- Light-inducible (red, blue, green spectra)

**Repressible Promoters:**
- Nutritional repressible (glucose, amino acids)
- Small molecule repressible
- Protein-repressible

**Regulatory Logic Promoters:**
- AND gates (require multiple inputs)
- OR gates (respond to multiple inputs)
- NOT gates (inverting)
- Complex logic (combinations)

#### Coding Parts → Protein Coding Sequences
**Reporter Proteins:**
- Fluorescent proteins (GFP, RFP, YFP, CFP, mCherry)
- Chromogenic proteins (beta-galactosidase, beta-glucuronidase)
- Luminescent proteins (luciferase variants)
- Epitope tags (His, FLAG, HA, Myc)

**Regulatory Proteins:**
- Transcription factors (activators, repressors)
- RNA-binding proteins
- Signaling proteins
- Proteases

**Metabolic Enzymes:**
- Biosynthetic (anabolic pathways)
- Degradative (catabolic pathways)
- Modifying (post-translational modifications)
- Cofactor-requiring (specify cofactor)

**Structural Proteins:**
- Scaffolds
- Assembly components
- Membrane proteins
- Transport proteins

### Tertiary Specificity Classification

Finer distinctions capture specific characteristics:

#### Promoters → Inducible → Small Molecule → Arabinose-Inducible
**Characteristics:**
- Inducer molecule: L-arabinose
- Regulatory protein: AraC
- Organism: E. coli
- Strength: Variable (dose-dependent)
- Dynamic range: 10-1000 fold induction
- Leakiness: Low to moderate
- Response time: 15-30 minutes
- Reversibility: Yes (upon arabinose removal)
- Cross-talk: Minimal with other systems

### Organism-Specific Classification

Parts optimized for specific chassis organisms:

**Prokaryotic Parts:**
- E. coli (primary model organism)
- B. subtilis (Gram-positive model)
- Cyanobacteria (photosynthetic chassis)
- Other bacteria (specialized applications)

**Eukaryotic Parts:**
- S. cerevisiae (yeast model)
- Mammalian (human, mouse, CHO cells)
- Plant (A. thaliana, tobacco)
- Insect (Drosophila, Sf9 cells)

**Cross-Kingdom Parts:**
- Universal promoters
- Conserved coding sequences
- Transferable regulatory elements

### Assembly Standard Classification

Parts classified by compatible assembly methods:

**BioBrick (RFC 10):** Standard prefix/suffix with EcoRI, XbaI, SpeI, PstI sites

**BglBrick (RFC 21):** Scar-less assembly with BglII and BamHI

**Silver (RFC 23):** BioBrick with additional features for fusion proteins

**Type IIS Standards:**
- MoClo (Modular Cloning)
- Golden Gate variants
- Level-based assembly

**Non-Standard Assembly:**
- Gibson assembly compatible
- SLIC compatible
- Gateway compatible
- Custom methods

## 3.3 Ontology-Based Classification

Modern registries increasingly adopt formal ontologies—structured vocabularies with defined relationships between terms.

### Sequence Ontology (SO)

The Sequence Ontology provides a standardized vocabulary for genomic features:

**Core Terms:**
```
SO:0000167 - promoter
├── SO:0001203 - constitutive promoter
├── SO:0001204 - inducible promoter
└── SO:0001205 - repressible promoter

SO:0000316 - CDS (coding sequence)
├── SO:0000101 - enzymatic activity
├── SO:0000357 - transcription factor
└── SO:0000103 - reporter gene

SO:0000141 - terminator
├── SO:0000614 - forward terminator
└── SO:0000615 - reverse terminator
```

**Advantages:**
- Standardized terminology across databases
- Machine-readable relationships
- Computational inference capabilities
- Integration with genomic databases

**Implementation in Registry:**
```json
{
  "part_id": "BBa_J23100",
  "sequence_ontology": {
    "primary": "SO:0001203",
    "term": "constitutive_promoter",
    "parent": "SO:0000167",
    "parent_term": "promoter"
  }
}
```

### Systems Biology Ontology (SBO)

SBO describes quantitative parameters and mathematical expressions:

**Part Characterization Terms:**
- SBO:0000064 - promoter strength
- SBO:0000186 - maximal transcription rate
- SBO:0000009 - kinetic constant
- SBO:0000261 - Hill coefficient

**Model Components:**
- SBO:0000589 - genetic production
- SBO:0000179 - degradation rate
- SBO:0000176 - biochemical reaction

### Gene Ontology (GO)

Describes biological functions, cellular components, and processes:

**Molecular Function (GO:0003674):**
- GO:0003700 - DNA-binding transcription factor activity
- GO:0004553 - hydrolase activity
- GO:0016491 - oxidoreductase activity

**Biological Process (GO:0008150):**
- GO:0009058 - biosynthetic process
- GO:0006351 - transcription regulation
- GO:0006412 - translation

**Cellular Component (GO:0005575):**
- GO:0005737 - cytoplasm
- GO:0005886 - plasma membrane
- GO:0005634 - nucleus

### ChEBI Ontology

Chemical Entities of Biological Interest for inducers and small molecules:

**Inducer Molecules:**
- CHEBI:61448 - IPTG (isopropyl β-D-1-thiogalactopyranoside)
- CHEBI:16977 - arabinose
- CHEBI:27931 - anhydrotetracycline

## 3.4 Metadata Standards

Comprehensive metadata enhances part utility and discoverability.

### Minimal Information Standard

The WIA-SYNTHETIC-BIO-REGISTRY defines minimum metadata requirements:

#### Essential Metadata (Required)
```json
{
  "part_id": "BBa_J23100",
  "name": "Constitutive promoter (Anderson)",
  "type": "regulatory",
  "subtype": "promoter",
  "sequence": "ttgacggctagctcagtcctaggtacagtgctagc",
  "length": 35,
  "author": "John Anderson",
  "creation_date": "2006-07-15",
  "license": "Open MTA",
  "safety_level": 1
}
```

#### Recommended Metadata
```json
{
  "organism": "Escherichia coli",
  "chassis_tested": ["E. coli K12", "E. coli BL21"],
  "assembly_standard": ["BioBrick RFC10", "Gibson"],
  "applications": ["gene expression", "reporter systems"],
  "short_description": "Strong constitutive promoter from Anderson library",
  "long_description": "Member of characterized promoter family...",
  "design_notes": "No known regulatory elements interfere with function",
  "references": ["PMID:12345678", "doi:10.xxxx/xxxxx"]
}
```

#### Extended Metadata
```json
{
  "sequence_features": [
    {"type": "promoter", "start": 1, "end": 35, "strand": 1},
    {"type": "-35_box", "start": 1, "end": 6, "strand": 1},
    {"type": "-10_box", "start": 28, "end": 33, "strand": 1}
  ],
  "characterization_summary": {
    "expression_level": 1.0,
    "expression_unit": "relative to reference",
    "reference_part": "BBa_J23101",
    "measurement_method": "GFP fluorescence",
    "replicates": 3,
    "standard_deviation": 0.12
  },
  "quality_flags": {
    "sequence_verified": true,
    "function_confirmed": true,
    "quantitative_data": true,
    "multi_lab_validated": false
  }
}
```

### Sequence Annotation

Detailed sequence features using standard formats:

#### GenBank Format
```
LOCUS       BBa_J23100                35 bp DNA     linear   SYN 15-JUL-2006
DEFINITION  Constitutive promoter (Anderson family)
ACCESSION   BBa_J23100
VERSION     BBa_J23100.1
KEYWORDS    BioBrick; promoter; constitutive.
SOURCE      synthetic DNA construct
FEATURES             Location/Qualifiers
     promoter        1..35
                     /label=J23100
                     /note="Strong constitutive promoter"
     -35_signal      1..6
                     /note="TTGACG"
     -10_signal      28..33
                     /note="TATAAT"
ORIGIN
        1 ttgacggcta gctcagtcct aggtacagtg ctagc
//
```

#### SBOL (Synthetic Biology Open Language)
```xml
<sbol:ComponentDefinition rdf:about="http://parts.igem.org/BBa_J23100">
  <sbol:displayId>BBa_J23100</sbol:displayId>
  <sbol:type rdf:resource="http://www.biopax.org/release/biopax-level3.owl#DnaRegion"/>
  <sbol:role rdf:resource="http://identifiers.org/so/SO:0000167"/>
  <sbol:sequence rdf:resource="http://parts.igem.org/BBa_J23100_seq"/>
</sbol:ComponentDefinition>
```

### Experimental Context Metadata

Capturing conditions under which parts were characterized:

**Growth Conditions:**
```json
{
  "organism": "E. coli K12 MG1655",
  "media": "LB broth",
  "temperature": 37,
  "temperature_unit": "Celsius",
  "shaking_speed": 250,
  "shaking_unit": "rpm",
  "growth_phase": "exponential",
  "OD600": 0.5
}
```

**Expression Context:**
```json
{
  "vector": "pSB1C3",
  "copy_number": "high (100-300)",
  "selection": "chloramphenicol (25 ug/mL)",
  "genetic_context": {
    "upstream": "BioBrick prefix",
    "downstream": "RBS-GFP-terminator"
  }
}
```

**Measurement Parameters:**
```json
{
  "instrument": "Tecan Infinite M1000 Pro",
  "measurement_type": "fluorescence",
  "excitation": 485,
  "emission": 535,
  "gain": 100,
  "reads_per_well": 10,
  "integration_time": 20,
  "unit": "microseconds"
}
```

## 3.5 Tagging and Faceted Classification

Beyond hierarchical systems, tags and facets enable flexible, multi-dimensional classification.

### User-Generated Tags

Community members add descriptive tags:

**Functional Tags:**
`#biosensor` `#reporter` `#inducible` `#constitutive` `#strong` `#weak` `#tunable`

**Application Tags:**
`#metabolic-engineering` `#therapeutics` `#biofuels` `#biosensing` `#optogenetics`

**Quality Tags:**
`#well-characterized` `#works-reliably` `#needs-optimization` `#context-dependent`

**Organism Tags:**
`#ecoli` `#yeast` `#mammalian` `#plant` `#bacillus`

**Project Tags:**
`#iGEM2024` `#Anderson-collection` `#community-favorite`

### Faceted Search

Multiple independent attributes enable precise filtering:

**Example Search Interface:**
```
Type: [Promoter ▼]
Strength: [Strong] [Medium] [Weak]
Regulation: [Constitutive] [Inducible] [Repressible]
Organism: [E. coli ▼]
Quality Tier: [3+]
Assembly: [BioBrick] [Gibson]
Year: [2020-2025]

Results: 47 parts found
```

### Controlled Vocabularies

Standardized term lists ensure consistency:

**Part Types (Controlled):**
- promoter
- ribosome_binding_site
- coding_sequence
- terminator
- regulatory_element
- origin_of_replication
- marker
- composite

**Strength Descriptors (Controlled):**
- very_weak (< 0.01 relative units)
- weak (0.01 - 0.1)
- medium (0.1 - 1.0)
- strong (1.0 - 10)
- very_strong (> 10)

**Quality Descriptors (Controlled):**
- documented
- confirmed
- well_characterized
- optimized
- certified

## 3.6 Cross-Registry Classification Standards

As multiple registries emerge, interoperability requires shared classification schemes.

### Universal Part Identifier (UPI)

Proposed standard for globally unique part identifiers:

**Format:** `[Registry]:[Collection]:[Part]:[Version]`

**Examples:**
- `iGEM:Anderson:J23100:1` - iGEM Registry, Anderson collection, part J23100, version 1
- `AddGene:CRISPR:12345:2` - AddGene, CRISPR collection, plasmid 12345, version 2
- `WIA:Standard:PROM001:1` - WIA Registry, Standard collection, promoter 001, version 1

### SBOL Visual Standard

Graphical symbols for biological parts enable universal visual classification:

**Standard Symbols:**
```
→   Promoter (arrow pointing right)
⭘   RBS (circle)
▭   CDS (rectangular box)
⊤   Terminator (T-shape)
◇   Operator (diamond)
⬟   Origin of replication (circle with +)
◀   Primer binding site (left arrow)
```

**Color Conventions:**
- Green: Functional and characterized
- Yellow: Partially characterized
- Red: Problematic or deprecated
- Gray: Theoretical or uncharacterized

### SBOL Data Model

Synthetic Biology Open Language provides a data model for exchanging designs:

**Core Classes:**
```
ComponentDefinition: Abstract part description
Component: Specific instance in a design
FunctionalComponent: Role within a module
Module: Functional grouping of components
Interaction: Relationships between components
Model: Mathematical or computational model
```

**Example SBOL Exchange:**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
         xmlns:sbol="http://sbols.org/v2#">

  <sbol:ComponentDefinition rdf:about="https://parts.igem.org/BBa_K123000">
    <sbol:displayId>BBa_K123000</sbol:displayId>
    <sbol:version>1</sbol:version>
    <sbol:type rdf:resource="http://www.biopax.org/release/biopax-level3.owl#DnaRegion"/>
    <sbol:role rdf:resource="http://identifiers.org/so/SO:0000804"/>

    <sbol:component>
      <sbol:Component rdf:about="https://parts.igem.org/BBa_K123000/promoter">
        <sbol:definition rdf:resource="https://parts.igem.org/BBa_J23100"/>
        <sbol:access rdf:resource="http://sbols.org/v2#public"/>
      </sbol:Component>
    </sbol:component>

    <sbol:component>
      <sbol:Component rdf:about="https://parts.igem.org/BBa_K123000/rbs">
        <sbol:definition rdf:resource="https://parts.igem.org/BBa_B0034"/>
        <sbol:access rdf:resource="http://sbols.org/v2#public"/>
      </sbol:Component>
    </sbol:component>

  </sbol:ComponentDefinition>

</rdf:RDF>
```

## 3.7 Machine Learning-Enhanced Classification

Modern registries employ machine learning to improve classification and discovery.

### Automated Feature Extraction

ML models identify sequence features and functional motifs:

**Promoter Prediction:**
- -35 and -10 box identification
- Spacer length optimization
- UP element detection
- Extended -10 recognition

**RBS Strength Prediction:**
- Shine-Dalgarno sequence scoring
- Secondary structure analysis
- Distance to start codon
- Translation initiation rate

**Terminator Efficiency:**
- Hairpin structure stability
- U-rich tract length
- Bidirectionality assessment

### Functional Annotation Prediction

ML models trained on characterized parts predict functions for new sequences:

**Input Features:**
- DNA sequence (k-mer representation)
- Sequence motifs
- Secondary structure
- Codon usage
- GC content
- ORF predictions

**Output Predictions:**
- Functional category probabilities
- Expected expression level
- Organism compatibility
- Assembly compatibility
- Quality tier estimate

**Model Architecture:**
```
Sequence Input → CNN (motif detection)
              ↓
            LSTM (context modeling)
              ↓
         Attention (feature weighting)
              ↓
      Dense Layers (classification)
              ↓
    Output: Category Probabilities
```

### Similarity-Based Clustering

Unsupervised learning identifies related parts:

**Clustering Methods:**
- K-means clustering on sequence embeddings
- Hierarchical clustering by function
- DBSCAN for outlier detection
- t-SNE visualization of part landscape

**Applications:**
- Identify similar parts with different names
- Discover part families
- Suggest alternative parts
- Detect redundant submissions

### Recommendation Systems

Collaborative filtering and content-based recommendations:

**User-Based:**
"Users who used part A also used parts B, C, D"

**Content-Based:**
"Parts similar to A include B, C, D based on sequence and function"

**Hybrid:**
Combines user behavior and part characteristics

**Implementation:**
```python
def recommend_parts(current_part, user_history, n=5):
    # Get part embeddings
    part_embedding = embed_sequence(current_part.sequence)

    # Compute similarity scores
    candidates = registry.search_similar(part_embedding, n=100)

    # Rank by user preferences
    scores = []
    for candidate in candidates:
        content_score = cosine_similarity(part_embedding,
                                         embed_sequence(candidate.sequence))
        collaborative_score = user_preference_score(candidate, user_history)
        combined_score = 0.6 * content_score + 0.4 * collaborative_score
        scores.append((candidate, combined_score))

    # Return top N
    return sorted(scores, key=lambda x: x[1], reverse=True)[:n]
```

## 3.8 Quality-Based Classification

Part quality is itself a critical classification dimension.

### Quality Dimensions

**Sequence Integrity:**
- DNA sequence verified by sequencing
- No unexpected mutations
- Correct length
- No contaminating sequences

**Functional Validation:**
- Behavior confirmed experimentally
- Quantitative measurements provided
- Multiple conditions tested
- Reproducibility demonstrated

**Documentation Completeness:**
- Detailed description
- Design rationale
- Construction methods
- Protocols provided
- References cited

**Community Validation:**
- Multiple users confirm function
- Independent laboratories verify
- Consistent results reported
- Troubleshooting documented

**Characterization Depth:**
- Basic function confirmed (Tier 1)
- Quantitative data (Tier 2)
- Multi-condition testing (Tier 3)
- Optimization studies (Tier 4)

### Quality Metrics

Quantitative scores for each dimension:

**Sequence Quality Score (0-100):**
```
SQS = (verification_status * 40) +
      (feature_annotation * 30) +
      (assembly_compatibility * 30)

where each component ∈ [0, 1]
```

**Functional Quality Score (0-100):**
```
FQS = (experimental_validation * 50) +
      (quantitative_data * 30) +
      (reproducibility * 20)
```

**Documentation Quality Score (0-100):**
```
DQS = (description_completeness * 30) +
      (protocol_detail * 25) +
      (design_rationale * 25) +
      (references * 20)
```

**Overall Quality Score (0-100):**
```
OQS = (SQS * 0.3) + (FQS * 0.4) + (DQS * 0.3)
```

### Quality Visualization

Registry interfaces display quality information:

**Badge System:**
```
🏅 Certified (OQS ≥ 90)
⭐ High Quality (OQS ≥ 75)
✓  Validated (OQS ≥ 60)
📝 Documented (OQS ≥ 40)
❓ Preliminary (OQS < 40)
```

**Quality Dashboard:**
```
Part: BBa_J23100
━━━━━━━━━━━━━━━━━━━━━━━━━
Sequence Verified    ████████████ 100%
Functionally Valid   ██████████░░  85%
Well Documented      ███████████░  92%
Community Support    █████████░░░  75%
━━━━━━━━━━━━━━━━━━━━━━━━━
Overall Quality      ███████████░  88%
Quality Tier: 4 (Certified)
```

## 3.9 Dynamic Classification

As understanding evolves, classification systems must adapt.

### Versioning and Updates

Parts and their classifications can change over time:

**Version Control:**
```
BBa_J23100
├── v1.0 (2006-07-15): Initial submission
├── v1.1 (2008-03-22): Sequence reverified
├── v2.0 (2012-06-10): Additional characterization
├── v2.1 (2015-09-03): New experimental data
└── v3.0 (2023-01-15): Reclassified to Tier 4
```

**Classification Updates:**
- Initial: "Promoter, documented"
- After characterization: "Promoter, strong, constitutive, well-characterized"
- After multi-lab validation: "Promoter, strong, constitutive, certified"

### Deprecation and Retirement

Parts may become obsolete:

**Reasons for Deprecation:**
- Better alternatives available
- Sequence errors discovered
- Inconsistent behavior reported
- Safety concerns identified
- Assembly standard obsolete

**Deprecation Process:**
1. Mark as "Deprecated" with explanation
2. Suggest replacement parts
3. Maintain in registry for historical purposes
4. Update documentation with warnings
5. Remove from standard distributions

### Reclassification

New information may require reclassification:

**Triggers:**
- New experimental data
- Changed understanding of function
- Discovery of additional roles
- Updated organism compatibility
- Revised safety classification

**Process:**
- Curator review of new information
- Community comment period
- Advisory board approval (if major change)
- Update with change log
- Notification to users who have used the part

---

## Key Takeaways

✓ Hierarchical classification provides structured organization from broad categories to specific subtypes
✓ Ontologies (SO, SBO, GO) enable standardized, machine-readable classification
✓ Comprehensive metadata enhances part utility and discoverability
✓ Tags and faceted search enable flexible multi-dimensional classification
✓ Cross-registry standards (UPI, SBOL) facilitate interoperability
✓ Machine learning enhances classification through automated feature extraction and recommendations
✓ Quality-based classification helps users identify reliable, well-characterized parts
✓ Dynamic classification systems adapt as knowledge evolves
✓ Multiple classification dimensions (function, quality, organism, assembly) serve different user needs

---

**Next Chapter:** Chapter 4 examines quality control and characterization processes that ensure biological parts meet standards for reliability and reproducibility.

---

*WIA-SYNTHETIC-BIO-REGISTRY Standard v1.0*
*© 2025 SmileStory Inc. / WIA*
*弘益人間 (Benefit All Humanity)*
