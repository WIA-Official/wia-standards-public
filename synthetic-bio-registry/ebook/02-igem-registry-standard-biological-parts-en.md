# Chapter 2: iGEM Registry of Standard Biological Parts

## WIA-SYNTHETIC-BIO-REGISTRY Standard
**Version:** 1.0
**Status:** Official WIA Standard
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 2.1 The Birth of the iGEM Registry

The Registry of Standard Biological Parts, commonly known as the iGEM Registry, emerged from the intersection of synthetic biology research and undergraduate education. Its story reflects the power of community-driven science and open-source principles.

### Origins at MIT (2003-2004)

In the spring of 2003, a group of researchers at MIT's Artificial Intelligence Laboratory began exploring a radical idea: could biological systems be engineered with the same reliability and predictability as electronic circuits? This question led to the BioBricks concept—standardized DNA parts with compatible interfaces.

**Tom Knight**, a computer scientist turned biologist, championed the idea that biology needed standardization similar to the electronics industry. His vision: create a library of interchangeable genetic parts that could be assembled like LEGO blocks.

**Randy Rettberg**, together with Knight and others, organized the first synthetic biology course at MIT in January 2004. This intensive Independent Activities Period (IAP) course challenged students to design and build biological systems using standardized parts.

The course's success led to a summer version, which evolved into the first international Genetically Engineered Machine (iGEM) competition in the summer of 2004. Five teams participated, each needing access to standardized biological parts. This practical need drove the creation of the Registry of Standard Biological Parts.

### Early Growth (2004-2008)

The registry began modestly:

**2004:** Initial registry contains fewer than 100 parts, primarily promoters, RBS sequences, and reporter genes. Physical samples distributed to 5 teams on paper forms.

**2005:** First web interface launched at parts.mit.edu. Registry expands to 352 parts as more teams contribute their designs. The concept of "experience pages" allows users to share their results.

**2006:** Registry reaches 724 parts. Introduction of standardized measurement protocols. Physical distribution improves with pre-plated DNA samples. Teams begin characterizing existing parts alongside submitting new ones.

**2007:** Breakthrough year with 1,500+ parts. Introduction of part families and collections. First international distribution center established in Europe. Registry moves to parts.igem.org domain.

**2008:** Over 2,700 parts cataloged. Introduction of quality ratings based on user feedback. Development of the first version of the Registry API for programmatic access.

### Maturation Phase (2009-2015)

The registry evolved from a simple catalog to a sophisticated knowledge repository:

**Database Restructuring:** Migration to robust database architecture supporting complex queries and relationships.

**Characterization Emphasis:** Introduction of data sheets requiring experimental verification. Launch of the Measurement Committee to standardize characterization protocols.

**Quality Control:** Implementation of multi-tier quality system. Parts classified based on documentation completeness and experimental validation.

**Community Curation:** Experienced users become curators, reviewing submissions and maintaining quality standards.

**Physical Infrastructure:** Partnership with AddGene for long-term plasmid storage and distribution. Establishment of regional distribution centers on multiple continents.

### Modern Era (2016-Present)

Recent years have seen technological transformation:

**2016-2018:** Mobile-responsive interface redesign. Integration with sequence analysis tools. Enhanced search with machine learning-based recommendations.

**2019-2020:** RESTful API version 2.0 released. Introduction of automated sequence verification. Integration with synthesis providers for direct DNA ordering.

**2021-2022:** Machine learning models for part function prediction. Automated characterization data extraction from publications. Enhanced data visualization tools.

**2023-2025:** AI-assisted design recommendations. Integration with lab automation platforms. Real-time collaborative features. Blockchain-based contribution tracking.

## 2.2 Registry Architecture and Structure

The iGEM Registry employs a sophisticated architecture to manage complexity while maintaining usability.

### Database Schema

The registry uses a relational database with several interconnected tables:

#### Parts Table (Core Entity)
```
part_id: Unique identifier (e.g., BBa_J23100)
part_name: Human-readable name
short_description: Brief functional description
part_type: Category (promoter, RBS, CDS, etc.)
sequence: DNA sequence in plain text
sequence_length: Length in base pairs
author: Original contributor
creation_date: Initial submission date
status: Available, unavailable, planning, deleted
quality_tier: 0-4 rating
safety_level: 1-4 biosafety classification
```

#### Characterization Data Table
```
data_id: Unique identifier for each dataset
part_id: Foreign key to parts table
measurement_type: Expression, activity, toxicity, etc.
protocol_id: Link to experimental protocol
values: Numerical measurements
units: Measurement units
conditions: Experimental conditions JSON
contributor: Laboratory/researcher
validation_status: Pending, verified, disputed
date_added: Timestamp
```

#### Experience Pages Table
```
experience_id: Unique identifier
part_id: Foreign key to parts table
user_id: Contributor identifier
institution: Research institution
description: Detailed experience report
results: Experimental outcomes
protocols: Methods used
troubleshooting: Problems and solutions
date: Submission date
helpful_count: Community ratings
```

#### Assembly Compatibility Table
```
compatibility_id: Unique identifier
part_id: Foreign key to parts table
standard: BioBrick, Type IIS, Gibson, etc.
prefix_sequence: 5' flanking sequence
suffix_sequence: 3' flanking sequence
verified: Boolean flag
notes: Special assembly considerations
```

### Part Naming Convention

Registry parts follow a hierarchical naming system:

**Format:** BBa_[X][#####]

**BB:** BioBrick designation
**a:** Version indicator (a, b, c, etc.)
**X:** Type prefix (optional)
  - J: Promoter
  - K: Regulatory
  - I: Protein coding
  - C: Composite
  - E: Regulatory element
  - R: RNA
**#####:** Sequential number

**Examples:**
- BBa_J23100: Constitutive promoter (Anderson family member)
- BBa_K098995: Composite device from iGEM team
- BBa_I13522: Protein coding sequence (GFP)
- BBa_B0015: Double terminator

### Part Types and Categories

The registry organizes parts into 15 major categories:

#### 1. Promoters (3,200+ parts)
**Constitutive:** Always active, various strengths
**Inducible:** Activated by specific molecules
**Repressible:** Turned off by specific molecules
**Regulatory:** Complex control logic

**Key Collections:**
- Anderson Promoter Library (BBa_J23100 series): 18 characterized constitutive promoters with expression levels ranging from 0.01 to 1.0 relative units
- T7 Promoter Family: Strong, specific promoters for T7 RNA polymerase
- Hybrid Promoters: Synthetic designs combining regulatory elements

#### 2. Ribosome Binding Sites (1,800+ parts)
**Natural RBS:** Sequences from organisms
**Synthetic RBS:** Computationally designed
**Variable Strength:** Range of translation efficiencies

**Key Collections:**
- BCD RBS Calculator Designs: Predictable translation rates
- Community Collection: User-validated RBS sequences

#### 3. Coding Sequences (7,500+ parts)
**Reporter Genes:** Fluorescent proteins, enzymes for detection
**Regulatory Proteins:** Transcription factors, repressors
**Enzymes:** Metabolic pathway components
**Structural Proteins:** Scaffolds, assembly components

**Key Collections:**
- Fluorescent Protein Family: GFP, RFP, YFP, CFP, mCherry variants
- Transcription Factor Library: Activators and repressors
- Metabolic Enzymes: Biosynthesis pathways

#### 4. Terminators (900+ parts)
**Single Terminators:** Basic transcription stop signals
**Double Terminators:** Enhanced termination efficiency
**Bidirectional Terminators:** Function in both directions

**Key Collections:**
- BBa_B0010-0015: Well-characterized E. coli terminators
- T7 Terminators: For T7 expression systems

#### 5. Protein Domains (1,200+ parts)
**Binding Domains:** Protein-protein interaction modules
**Catalytic Domains:** Enzymatic activity regions
**Localization Signals:** Targeting sequences
**Degradation Tags:** Protein stability control

#### 6. Composite Parts (2,800+ parts)
**Expression Cassettes:** Complete expression units
**Reporter Devices:** Measurable output systems
**Genetic Circuits:** Multi-component systems
**Biosensors:** Input-responsive devices

#### 7. Regulatory Elements (1,500+ parts)
**Operators:** Protein binding sites
**Riboswitches:** RNA-based regulation
**Attenuators:** Transcription control
**Enhancers:** Expression boosters

#### 8. Origins of Replication (400+ parts)
**High Copy:** 100-500 copies per cell
**Medium Copy:** 15-100 copies per cell
**Low Copy:** 1-15 copies per cell
**Conditional:** Copy number depends on conditions

#### 9. Plasmid Backbones (600+ parts)
**Standard Backbones:** Common vector frameworks
**Specialized Vectors:** Specific applications
**Multiple Cloning Sites:** Various assembly options

#### 10. Markers (800+ parts)
**Antibiotic Resistance:** Selection markers
**Auxotrophic Markers:** Metabolic selection
**Fluorescent Markers:** Visual identification
**Chromogenic Markers:** Color-based selection

#### 11. Recombinase Sites (300+ parts)
**Cre-lox Systems:** Site-specific recombination
**FLP-FRT Systems:** Yeast recombination
**Gateway Sites:** Cloning system components

#### 12. RNA Parts (900+ parts)
**Ribozymes:** Catalytic RNA
**Aptamers:** Binding RNA
**Small RNAs:** Regulatory molecules
**CRISPR Components:** Guide RNAs

#### 13. DNA Binding Sites (700+ parts)
**Transcription Factor Sites:** Protein binding sequences
**Restriction Sites:** Enzyme recognition sequences
**Recombination Sites:** Site-specific recombination targets

#### 14. Measurement Devices (400+ parts)
**Standard Curves:** Calibration devices
**Reference Standards:** Comparison controls

#### 15. Chassis-Specific Parts (2,000+ parts)
**E. coli Parts:** Optimized for common chassis
**Yeast Parts:** S. cerevisiae specific
**Mammalian Parts:** Human cell lines
**Plant Parts:** A. thaliana and others
**Archaea Parts:** Extremophile applications

## 2.3 Part Submission and Curation Process

The registry maintains quality through structured submission and curation workflows.

### Submission Pipeline

#### Phase 1: Registration and Planning
1. **Team Registration:** Users create accounts linked to institutions
2. **Project Planning:** Teams declare intended contributions
3. **Design Review:** Optional pre-submission design consultation
4. **Sequence Design:** Use of approved assembly standards

#### Phase 2: Physical Construction
1. **DNA Synthesis:** Parts synthesized or assembled from existing parts
2. **Cloning:** Parts inserted into standard backbones
3. **Sequence Verification:** Confirm DNA sequence accuracy
4. **Functional Testing:** Basic characterization performed
5. **Sample Preparation:** Parts submitted as purified DNA or bacterial glycerol stocks

#### Phase 3: Digital Submission
**Online Form Completion:**
- Part name and description
- Sequence upload (GenBank, FASTA, or plain text)
- Functional characterization data
- Assembly standard compatibility
- Authorship and attribution
- Safety and regulatory information

**Documentation Requirements:**
- Design rationale
- Construction methods
- Testing protocols
- Experimental results
- References and sources
- License selection (typically Open MTA)

#### Phase 4: Initial Review
**Automated Checks:**
- Sequence format validation
- Restricted site identification
- Assembly standard compatibility verification
- Duplication detection
- Safety screening (select agent sequences, toxins)

**Manual Curator Review:**
- Description quality assessment
- Documentation completeness
- Experimental data evaluation
- Safety classification
- Quality tier assignment

#### Phase 5: Publication
- Assignment of official part number (BBa_######)
- Integration into main database
- Physical sample storage in registry collection
- Availability notification to submitter
- Public listing on registry website

### Curation Standards

The registry employs a multi-level curation system:

#### Level 1: Automated Curation
**Sequence Analysis:**
- ORF identification
- Restriction site mapping
- Sequence feature annotation
- GC content calculation
- Codon usage analysis

**Quality Checks:**
- Sequence length verification
- Standard prefix/suffix confirmation
- Prohibited sequence detection
- Similarity screening

#### Level 2: Community Curation
**User Feedback:**
- Experience page submissions
- Star ratings (1-5 scale)
- Worked/Failed flags
- Commentary and suggestions

**Peer Review:**
- Expert review by experienced users
- Cross-validation of results
- Protocol verification
- Troubleshooting advice

#### Level 3: Professional Curation
**Registry Staff Review:**
- Comprehensive documentation assessment
- Experimental design evaluation
- Data quality verification
- Literature cross-referencing
- Safety compliance confirmation

**Scientific Advisory Board:**
- Annual review of high-tier parts
- Standards development
- Policy recommendations
- Quality metric refinement

### Quality Tier Criteria

Parts are assigned quality tiers based on standardized criteria:

#### Tier 0: Documented (40% of registry)
**Requirements:**
✓ Sequence submitted and verified
✓ Basic description provided
✓ Assembly standard specified
✓ Authorship documented

**Limitations:**
✗ Minimal experimental validation
✗ May not work as described
✗ Limited characterization data

#### Tier 1: Confirmed (25% of registry)
**Requirements:**
✓ All Tier 0 requirements met
✓ Function confirmed by original submitter
✓ Basic experimental results provided
✓ One laboratory has successful experience

**Data Expected:**
- Growth curves
- Expression confirmation (e.g., fluorescence)
- Basic functional assay

#### Tier 2: Well-Characterized (20% of registry)
**Requirements:**
✓ All Tier 1 requirements met
✓ Quantitative characterization data
✓ Multiple experimental conditions tested
✓ Reproducibility demonstrated
✓ Comprehensive protocol documentation

**Data Expected:**
- Dose-response curves
- Time-course measurements
- Temperature sensitivity
- Media compatibility
- Copy number effects
- Growth impact assessment

#### Tier 3: Optimized (10% of registry)
**Requirements:**
✓ All Tier 2 requirements met
✓ Optimization study performed
✓ Comparison with related parts
✓ Design principles documented
✓ Computational model available

**Data Expected:**
- Systematic mutagenesis studies
- Structure-function relationships
- Performance benchmarking
- Modeling validation

#### Tier 4: Certified (5% of registry)
**Requirements:**
✓ All Tier 3 requirements met
✓ Multi-laboratory validation
✓ Published in peer-reviewed literature
✓ Meets NIST standards (where applicable)
✓ Recommended by Registry Advisory Board

**Data Expected:**
- Inter-laboratory studies
- Uncertainty quantification
- Reference materials
- Standard operating procedures

## 2.4 Registry Access and Usage

The registry provides multiple interfaces for accessing parts and data.

### Web Interface

The primary web portal (parts.igem.org) offers:

**Search Functionality:**
- Text search across names, descriptions, and documentation
- Advanced filters: part type, quality tier, organism, year
- Sequence similarity search (BLAST integration)
- Functional category browsing
- Tag-based navigation
- Recent additions and popular parts

**Part Pages:**
- Complete sequence display (plain text, GenBank, FASTA)
- Functional description and design notes
- Characterization data with interactive plots
- Assembly compatibility information
- Experience pages from community users
- Related parts and recommended combinations
- Physical sample availability
- Citation information

**User Features:**
- Personal part collections
- Custom tags and annotations
- Collaboration tools
- Contribution tracking
- Notification system for updates

**Visualization Tools:**
- Sequence viewer with feature highlighting
- Genetic circuit diagrams
- Expression data plots
- Comparison charts
- 3D structure viewers (for proteins)

### Registry API (RESTful)

Programmatic access enables tool integration:

#### Authentication
```bash
# Obtain API key from user profile
export IGEM_API_KEY="your_api_key_here"
```

#### Part Retrieval
```bash
# Get part information
curl -H "Authorization: Bearer $IGEM_API_KEY" \
  https://parts.igem.org/api/v2/parts/BBa_J23100

# Response (JSON)
{
  "part_id": "BBa_J23100",
  "name": "constitutive promoter (Anderson)",
  "type": "regulatory",
  "subtype": "promoter",
  "sequence": "ttgacggctagctcagtcctaggtacagtgctagc",
  "length": 35,
  "quality_tier": 4,
  "characterization_data": [...]
}
```

#### Search Operations
```bash
# Search by part type
curl "https://parts.igem.org/api/v2/search?type=promoter&tier=4"

# Sequence similarity search
curl -X POST "https://parts.igem.org/api/v2/blast" \
  -d "sequence=ATGCGTAAAGGAGAAGAACT&evalue=0.001"
```

#### Bulk Downloads
```bash
# Download entire registry (requires authentication)
curl -H "Authorization: Bearer $IGEM_API_KEY" \
  "https://parts.igem.org/api/v2/export/all" > registry_complete.json
```

### Physical Distribution

#### Annual Distribution Kit
Each year, iGEM teams receive a Distribution Kit containing:
- 400+ commonly used parts
- Standard plasmid backbones
- Assembly control constructs
- Measurement standards
- Empty vectors for cloning

**Format:** 96-well plates with dried DNA spots
**Revival Protocol:** Add water, transform into competent cells
**Quality Control:** Each batch tested before distribution

#### AddGene Partnership
For individual part requests:
1. Search registry for desired part
2. Check AddGene availability
3. Place order through AddGene platform
4. Receive purified plasmid DNA (5-10 μg)
5. Cost: $65 per plasmid (non-profit pricing)

#### Regional Distribution Centers
**North America:** MIT - Cambridge, MA
**Europe:** BIOSS - Freiburg, Germany
**Asia:** PKU - Beijing, China
**South America:** USP - São Paulo, Brazil

## 2.5 Registry Impact and Statistics

The iGEM Registry has profoundly influenced synthetic biology education and research.

### Usage Statistics (2024-2025)

**Global Reach:**
- Registered users: 75,000+
- Active monthly users: 12,000+
- Countries represented: 67
- Institutions: 1,200+
- Annual downloads: 850,000+ part requests

**Contribution Metrics:**
- Total parts: 20,347
- New parts (2024): 1,623
- Contributing teams (2024): 356
- Updated characterization data: 4,500+ entries

**Quality Distribution:**
- Tier 0 (Documented): 8,140 parts (40%)
- Tier 1 (Confirmed): 5,087 parts (25%)
- Tier 2 (Well-Characterized): 4,069 parts (20%)
- Tier 3 (Optimized): 2,035 parts (10%)
- Tier 4 (Certified): 1,016 parts (5%)

**Popular Parts (Top 10 by downloads):**
1. BBa_J23100 - Constitutive promoter (Anderson family)
2. BBa_E0040 - GFP coding sequence
3. BBa_B0034 - Strong RBS (RBS Calculator)
4. BBa_B0015 - Double terminator
5. BBa_I13522 - RFP coding sequence
6. BBa_K123000 - GFP generator
7. BBa_R0040 - TetR repressible promoter
8. BBa_C0012 - LacI protein coding sequence
9. BBa_E1010 - mRFP coding sequence
10. BBa_K608003 - Constitutive strong promoter

### Research Impact

**Publications:**
- Registry-citing papers: 8,500+
- Annual citations: 1,200+
- High-impact journals: 350+ articles
- Top journals: Nature, Science, Cell, PNAS

**Applications:**
- Metabolic engineering: 35% of citations
- Biosensors: 25% of citations
- Synthetic circuits: 20% of citations
- Therapeutics: 12% of citations
- Other applications: 8% of citations

**Notable Achievements:**
- Artemisinin production pathway (anti-malarial drug)
- Biofuel production systems
- Plastic-degrading bacteria
- Heavy metal biosensors
- Optogenetic tools
- CRISPR systems

### Educational Impact

**iGEM Competition:**
- Total teams (2004-2024): 6,000+
- Participating students: 50,000+
- Countries: 67
- Undergraduate students: 70%
- Graduate students: 20%
- High school students: 10%

**Academic Adoption:**
- Courses using registry: 500+
- Universities: 300+
- Lab exercises developed: 1,200+
- Student projects: 10,000+

**Career Impact:**
- Alumni in synthetic biology careers: 60%
- PhD programs entered: 35%
- Industry positions: 45%
- Entrepreneurship: 15%
- Academic positions: 20%

## 2.6 Registry Maintenance and Sustainability

Long-term viability requires robust operational infrastructure and funding.

### Technical Infrastructure

**Data Center Operations:**
- Primary servers: MIT campus (Cambridge, MA)
- Backup servers: Cloud infrastructure (AWS)
- Database: PostgreSQL with replication
- Storage: 5 TB current, expanding 1 TB/year
- Backup frequency: Daily incremental, weekly full
- Disaster recovery: Geographic redundancy

**Performance Metrics:**
- Uptime target: 99.5%
- Actual uptime (2024): 99.7%
- Average page load: 1.2 seconds
- API response time: 0.3 seconds
- Concurrent users supported: 1,000+

**Security Measures:**
- SSL/TLS encryption for all connections
- Two-factor authentication for submissions
- Regular security audits
- Data integrity checksums
- Access logs and monitoring
- Intrusion detection systems

### Physical Repository

**Sample Storage:**
- Primary location: MIT Synthetic Biology Center
- Format: Bacterial glycerol stocks at -80°C
- Plasmid backups: Purified DNA at -20°C
- Organization: 96-well format plates
- Tracking: Barcode system with database integration
- Capacity: 25,000 samples (expandable)

**Quality Assurance:**
- Annual viability testing: Random sampling (5%)
- Sequence verification: Every 5 years
- Contamination screening: Quarterly
- Backup culture maintenance: Duplicate stocks

### Funding Model

**Revenue Sources:**
- iGEM Foundation: 60% (from competition fees and donations)
- Research Grants: 25% (NSF, NIH, private foundations)
- Institutional Support: 10% (MIT contribution)
- Corporate Sponsorship: 5% (industrial partners)

**Annual Budget (2024):** $1.2 million
- Personnel: $650K (5 FTE staff)
- Infrastructure: $250K (servers, storage, bandwidth)
- Physical repository: $150K (materials, equipment)
- Development: $100K (software improvements)
- Outreach: $50K (documentation, training)

### Governance Structure

**Registry Advisory Board:**
- 12 members: academic researchers, industry representatives, registry users
- Term: 3 years, staggered
- Meetings: Quarterly
- Responsibilities: Strategic direction, quality standards, policy development

**Scientific Committee:**
- 20 members: synthetic biology experts
- Focus: Technical standards, quality metrics, characterization protocols
- Meetings: Bi-annually

**Operations Team:**
- Director: Strategic leadership
- Database Administrator: Technical infrastructure
- Curator (2): Quality control, submissions
- Web Developer: Interface and API
- Laboratory Technician: Physical repository

## 2.7 Community and Culture

The registry's success stems from strong community engagement and shared values.

### Community Principles

**Open Access:** All registry data freely available without restrictions.

**Attribution:** Contributors receive credit for submissions and improvements.

**Quality Over Quantity:** Emphasis on well-characterized parts rather than raw numbers.

**Collaboration:** Encouragement of teamwork and knowledge sharing.

**Education:** Commitment to training the next generation of synthetic biologists.

**Safety:** Responsible research practices and biosafety compliance.

### User Engagement

**Experience Sharing:**
Users document their experiences with parts:
- Success stories
- Failure modes
- Optimization tips
- Protocol variations
- Troubleshooting guides

**Mentorship Programs:**
- Experienced teams mentor newcomers
- Registry tutorials and workshops
- Office hours for submission assistance
- Webinars on best practices

**Recognition Systems:**
- Contributor badges and achievements
- Annual awards for best submissions
- Featured parts highlighting
- Publication support

### Communication Channels

**Primary Platforms:**
- Registry website forums
- Email distribution lists
- Social media (Twitter, LinkedIn)
- Annual iGEM Jamboree
- Regional meetups

**Documentation:**
- Submission guidelines
- Characterization protocols
- Assembly instructions
- Troubleshooting FAQs
- Video tutorials

## 2.8 Future Directions

The registry continues to evolve, adapting to technological advances and community needs.

### Near-Term Developments (2025-2027)

**Enhanced Characterization:**
- Integration with automated characterization facilities
- Standardized measurement protocols adoption
- High-throughput testing programs
- Real-time data upload from laboratories

**Improved Search and Discovery:**
- Machine learning-based recommendations
- Context-aware suggestions
- Semantic search capabilities
- Virtual screening tools

**Expanded Organism Support:**
- Mammalian cell registries
- Plant synthetic biology
- Fungal systems
- Archaeal parts

### Medium-Term Goals (2027-2030)

**Federation with Other Registries:**
- Interoperability protocols
- Cross-registry search
- Unified authentication
- Data synchronization

**Design Tool Integration:**
- API standardization
- Direct design-to-registry workflows
- Automated part selection
- Performance prediction

**Quality Enhancement:**
- AI-assisted curation
- Automated characterization evaluation
- Predictive quality scoring
- Continuous improvement tracking

### Long-Term Vision (2030+)

**Intelligent Registry:**
- AI-designed parts
- Predictive modeling integration
- Automated optimization
- Self-organizing catalog

**Global Infrastructure:**
- Distributed physical repositories
- Regional curation centers
- International collaboration
- Universal standards adoption

**Comprehensive Knowledge Base:**
- Integration with scientific literature
- Automated data extraction
- Meta-analysis capabilities
- Predictive design support

---

## Key Takeaways

✓ The iGEM Registry contains 20,000+ standardized biological parts from 400+ institutions
✓ Parts are organized into 15 major categories with quality tiers from 0-4
✓ Rigorous submission and curation processes ensure quality and safety
✓ Multi-channel access: web interface, RESTful API, physical distribution
✓ Strong community culture emphasizes openness, quality, and collaboration
✓ Sustainable through diverse funding and robust governance
✓ Significant impact on synthetic biology research and education
✓ Continuous evolution incorporating new technologies and community feedback

---

**Next Chapter:** Chapter 3 examines cataloging and classification systems used in biological part registries, exploring taxonomies, ontologies, and organizational frameworks.

---

*WIA-SYNTHETIC-BIO-REGISTRY Standard v1.0*
*© 2025 SmileStory Inc. / WIA*
*弘益人間 (Benefit All Humanity)*
