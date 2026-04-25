# Chapter 6: Global Registry Infrastructure

## WIA-SYNTHETIC-BIO-REGISTRY Standard
**Version:** 1.0
**Status:** Official WIA Standard
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 6.1 The Physical Reality of Biological Registries

Unlike purely digital repositories, biological part registries must maintain physical infrastructure to store, preserve, and distribute actual DNA samples. This requirement creates unique logistical, financial, and technical challenges that shape registry operations worldwide.

### The Digital-Physical Duality

Modern registries operate in two parallel realms:

**Digital Infrastructure:**
- Sequence databases and metadata
- Web interfaces and APIs
- Search and discovery tools
- Documentation and protocols
- User accounts and permissions
- Analytics and monitoring

**Physical Infrastructure:**
- DNA sample storage (-80°C freezers)
- Bacterial strain collections
- Quality control laboratories
- Distribution facilities
- Backup repositories
- Equipment maintenance

**Critical Interdependence:**
Every digital entry must have a corresponding physical sample. If physical samples degrade or are lost, digital information alone cannot reconstitute them without re-synthesis. Conversely, physical samples without proper digital documentation lose much of their value.

### Infrastructure Scale

The iGEM Registry physical infrastructure (2025):

**Primary Repository (MIT, Cambridge, MA):**
- 20,000+ distinct parts stored
- Multiple aliquots per part (typically 4-6)
- Total samples: ~100,000 vials
- Storage capacity: 15 ultra-low freezers (-80°C)
- Backup generators and alarm systems
- Redundant temperature monitoring
- Annual maintenance costs: $125,000

**Backup Repository (AddGene, Cambridge, MA):**
- Duplicate samples of high-use parts (~5,000 parts)
- Independent facility, different building
- Cross-verification protocols
- Geographic diversity (limited but intentional)

**Regional Distribution Centers:**
- North America: MIT (primary)
- Europe: BIOSS Center for Biological Signalling Studies (Freiburg, Germany)
- Asia: Peking University (Beijing, China)
- South America: University of São Paulo (Brazil)
- Each center: 500-2,000 most-used parts

**Laboratory Facilities:**
- Molecular biology labs for quality control
- Sequencing equipment
- PCR and gel electrophoresis
- Bacterial culture facilities
- Plasmid preparation equipment
- Sterile handling infrastructure

## 6.2 Sample Storage and Preservation

Long-term preservation of biological materials requires careful attention to storage conditions:

### Storage Formats

**Bacterial Glycerol Stocks (Primary Format):**
```
Composition:
- Bacterial culture (E. coli or other host)
- Containing plasmid with part
- 15-20% glycerol (cryoprotectant)
- Growth media
- Stored at -80°C

Advantages:
+ Long shelf life (decades if properly maintained)
+ Easy to revive (streak on plate)
+ Maintains episomal plasmids
+ Low cost
+ Proven track record

Disadvantages:
- Requires continuous freezer power
- Risk of contamination upon opening
- Potential for mutation over time
- Requires living cells
```

**Purified Plasmid DNA (Secondary Format):**
```
Composition:
- Purified circular plasmid DNA
- In TE buffer or water
- Stored at -20°C or -80°C

Advantages:
+ Sequence stable
+ No living organisms
+ Easy to ship
+ Quick to use (direct transformation)
+ Quantifiable

Disadvantages:
- Degrades faster than glycerol stocks
- More expensive to prepare
- Requires purification equipment
- Limited shelf life (5-10 years)
```

**Lyophilized (Freeze-Dried) Samples:**
```
Composition:
- Bacterial cells or DNA
- Freeze-dried in protective matrix
- Stored at room temperature or 4°C

Advantages:
+ No freezer required
+ Very stable at room temperature
+ Easy to ship
+ Long shelf life (decades)
+ Power-failure resistant

Disadvantages:
- More complex preparation
- Revival can be less reliable
- Higher initial cost
- Less commonly used in registries
```

**Synthetic DNA (Emerging):**
```
Approach:
- No physical sample stored
- Sequence in database
- Synthesized on-demand when ordered
- Partnered with DNA synthesis companies

Advantages:
+ No physical storage required
+ No degradation or mutation
+ Always "fresh" sample
+ Scalable
+ No contamination risk

Disadvantages:
- Requires synthesis partnership
- Cost per sample (currently $0.07-0.15/bp)
- Synthesis errors possible
- Delivery time (1-2 weeks)
- Dependent on synthesis technology
```

### Storage Best Practices

**Temperature Management:**
```
Critical Requirements:
✓ Temperature: -80°C ± 5°C
✓ Monitoring: Continuous digital logging
✓ Alarms: Immediate alert if temperature rises
✓ Backup power: Generator and UPS systems
✓ Multiple freezers: Distribute samples to reduce risk
✓ Service contracts: Regular preventive maintenance
```

**Organization Systems:**
```
Sample Organization:
- 96-well format boxes
- Barcoded vials and boxes
- Database integration (position tracking)
- Standardized labeling
- Color-coding by collection or year
- Master index with search capability

Example Barcode System:
BBa_J23100-001-A01
│          │   └─── Position in box (A01 = Row A, Column 1)
│          └────── Box number (001)
└───────────────── Part ID
```

**Sample Tracking:**
```sql
CREATE TABLE sample_storage (
    sample_id SERIAL PRIMARY KEY,
    part_id VARCHAR(20) NOT NULL,
    format VARCHAR(20), -- 'glycerol_stock', 'plasmid_DNA', 'lyophilized'
    location VARCHAR(50), -- 'Freezer_3_Box_47_A05'
    preparation_date DATE,
    verified_date DATE,
    status VARCHAR(20), -- 'available', 'depleted', 'quarantine', 'backup'
    notes TEXT,
    FOREIGN KEY (part_id) REFERENCES parts(part_id)
);
```

### Quality Monitoring

**Regular Viability Testing:**
```
Protocol:
1. Random sampling: Select 5% of collection annually
2. Revival test: Streak on agar, incubate overnight
3. Colony PCR: Verify insert presence
4. Sequencing: Confirm sequence integrity (subset)
5. Documentation: Record results in database
6. Action: Replace samples failing any test

Annual Schedule:
January: Parts 0-2000
April: Parts 2001-4000
July: Parts 4001-6000
October: Parts 6001-8000
Rolling: New submissions and flagged parts
```

**Contamination Prevention:**
```
Procedures:
✓ Dedicated freezers for registry samples only
✓ Sterile technique when handling samples
✓ Single-use loops for streaking
✓ Regular decontamination of equipment
✓ Separated work areas (pre-PCR vs. post-PCR)
✓ Personnel training and certification
✓ Quarterly environmental monitoring
```

## 6.3 Distribution Networks

Getting parts from storage to users worldwide requires efficient logistics:

### Distribution Models

#### Annual Distribution Kit (iGEM Model)

**Process:**
```
1. Part Selection (December - January):
   - Survey teams for most-needed parts
   - Select 300-500 parts for inclusion
   - Prioritize high-quality, well-characterized parts
   - Include measurement standards

2. Production (February - April):
   - Inoculate bacterial cultures
   - Grow to high density
   - Spot DNA on 96-well plates
   - Dry under vacuum
   - Quality control testing
   - Package with desiccant

3. Distribution (May - June):
   - Ship to registered teams worldwide
   - International courier (FedEx, DHL)
   - Temperature monitoring in packages
   - Track shipments
   - Confirm receipt

4. Revival (At Team Labs):
   - Add sterile water to dried spots
   - Transform into competent cells
   - Select on appropriate antibiotic
   - Verify by colony PCR
```

**Scale (2024):**
- Kits produced: 400+
- Parts per kit: 350
- Countries reached: 50+
- Total distribution: 140,000+ DNA samples
- Cost per kit: ~$300 (production + shipping)
- Funded by: iGEM competition fees and sponsors

#### On-Demand Distribution (AddGene Model)

**Workflow:**
```
1. User Request:
   - Browse online catalog
   - Add plasmids to cart
   - Complete license agreements
   - Submit institutional information
   - Pay fee ($65 per plasmid for non-profits)

2. Order Processing (AddGene):
   - Verify license compliance
   - Check institutional approval (if required)
   - Queue for production

3. Preparation (3-5 business days):
   - Inoculate bacterial culture
   - Grow overnight
   - Miniprep plasmid DNA
   - Quality control (gel, concentration)
   - Package with documentation

4. Shipping:
   - Standard shipment: 5-10 days (most regions)
   - Express available (additional cost)
   - Temperature-controlled packaging
   - Tracking provided

5. User Receipt:
   - Transform into competent cells
   - Verify by restriction digest or sequencing
   - Report any issues to AddGene
```

**Scale (2024, Registry Parts Subset):**
- Parts distributed: ~50,000 requests/year
- Average delivery time: 8 days (US), 12 days (international)
- Success rate: 98% (verified by user feedback)
- Revenue model: Cost recovery, non-profit operation

#### Regional Hub Model (Emerging)

**Concept:**
Regional centers maintain local stocks of most-used parts, enabling faster, cheaper distribution within regions.

**Hub Locations:**
- **North America Hub:** MIT (Cambridge, MA) - 20,000 parts
- **Europe Hub:** BIOSS (Freiburg, Germany) - 2,000 parts
- **Asia Hub:** Peking University (Beijing, China) - 1,500 parts
- **South America Hub:** USP (São Paulo, Brazil) - 800 parts
- **Africa Hub:** Planned (Nairobi, Kenya or Cape Town, SA) - 500 parts

**Benefits:**
✓ Faster delivery within region (2-3 days vs. 7-14 days)
✓ Lower shipping costs
✓ Avoid customs delays
✓ Local language support
✓ Regional quality control
✓ Easier collaboration

**Challenges:**
✗ Synchronization between hubs
✗ Ensuring consistent quality
✗ Funding regional operations
✗ Differences in regulatory environments
✗ Uneven distribution capacity

### Synthesis-on-Demand Integration

**Partnered Synthesis Model:**
```
Workflow:
1. User browses registry, selects part
2. Option presented: "Order physical sample" or "Synthesize fresh"
3. If synthesis selected:
   - Sequence sent to synthesis partner (Twist, IDT, GenScript, etc.)
   - User pays synthesis cost (~$0.10/bp + cloning)
   - 7-14 day turnaround
   - Sequence guaranteed error-free
   - Delivered in user-specified vector

4. Advantages:
   + No waiting for physical stock
   + Guaranteed sequence accuracy
   + Custom vector options
   + Codon optimization available
   + Eliminates storage limitations

5. Cost Comparison:
   - Physical distribution: $0-65 per part
   - Synthesis: $20-200 per part (depends on length)
   - Break-even: ~400 bp
```

**Registry Partners (2025):**
- Twist Bioscience: NGS-based synthesis, high throughput
- IDT (Integrated DNA Technologies): gBlocks, gene fragments
- GenScript: Full service, global reach
- Synbio Technologies: Asia-focused
- Atum (formerly DNA2.0): High complexity sequences

## 6.4 International Shipping and Regulations

Moving biological materials across borders involves complex regulations:

### Regulatory Framework

**UN Model Regulations:**
Biological materials classified for shipping:

**Division 6.2 - Infectious Substances:**
- Category A: Known pathogens (e.g., Ebola, anthrax)
- Category B: Other infectious substances
- Registry parts typically exempt (non-pathogenic E. coli)

**Exempt Biological Samples:**
Most registry parts qualify as exempt if:
✓ Non-pathogenic organisms
✓ Minimal disease risk
✓ Properly packaged
✓ Labeled "Exempt biological sample"

**Packaging Requirements (UN3373 - Biological Substance Category B):**
```
Triple Packaging System:

1. Primary Container:
   - Watertight (screw cap vial)
   - Volume ≤ 1 L
   - Contains sample

2. Secondary Container:
   - Watertight, leak-proof
   - Absorbent material between primary and secondary
   - Multiple primary containers allowed

3. Outer Package:
   - Rigid (corrugated cardboard box)
   - Labeled with UN3373 marking
   - Contact information
   - "Exempt biological sample" label
```

### Country-Specific Requirements

**United States:**
- **Import:** CDC and USDA permits for certain organisms (E. coli K12 generally exempt)
- **Export:** Commerce Department for dual-use items (most registry parts exempt)
- **APHIS (Plant Protection):** Required for plant pathogens
- **Select Agent Program:** Restricted organisms (registry screens against these)

**European Union:**
- **Import:** Health certificate for biological materials
- **Dual-use Regulation:** Export controls for certain biotechnologies
- **Nagoya Protocol:** ABS considerations for genetic resources
- **REACH Regulation:** Chemical safety (usually not applicable to DNA)

**China:**
- **Import:** MARA (Ministry of Agriculture) permit
- **Biosafety Clearance:** Required for GMOs
- **CITES:** Endangered species (not typical for registry parts)
- **Export:** Restrictions on genetic resources (Nagoya-like system)

**Japan:**
- **Cartagena Protocol:** LMO (Living Modified Organism) import procedures
- **Type 2 LMO:** Most registry parts fall into this category
- **Notification:** Required before import
- **Quarantine:** Plant and animal health regulations

**Challenges:**
- Regulations vary widely
- Frequent changes in requirements
- Language barriers
- Processing delays at customs
- Unexpected costs (duties, inspection fees)
- Paperwork complexity

### Registry Solutions

**Regulatory Compliance Support:**
```
Registry Provides:
✓ Material Safety Data Sheets (MSDS)
✓ Biosafety documentation
✓ Sequence information for customs
✓ Certificate of origin
✓ Statement of non-pathogenicity
✓ License agreements
✓ Country-specific guidance documents
```

**Regional Hub Strategy:**
By maintaining regional centers, registries minimize international shipments and associated regulatory complexity.

**Digital-First Approach:**
Providing sequence data digitally with synthesis-on-demand reduces need for physical international shipping.

## 6.5 Data Infrastructure and Cyberinfrastructure

The digital backbone supporting biological part registries:

### Database Architecture

**Technology Stack (Typical Registry):**
```
Frontend:
- React.js or Vue.js for user interface
- Bootstrap or Material-UI for design
- D3.js for data visualization
- Embedded sequence viewers (SnapGene, Benchling viewers)

Backend:
- Python/Django or Node.js/Express
- RESTful API architecture
- GraphQL for complex queries
- Authentication: OAuth 2.0, JWT

Database:
- PostgreSQL (relational data)
- MongoDB (document storage for variable metadata)
- Redis (caching, session management)
- Elasticsearch (full-text search)

File Storage:
- AWS S3 or similar for large files (sequence files, protocols, images)
- CDN for fast global delivery

Infrastructure:
- Cloud hosting (AWS, Google Cloud, Azure)
- Docker containers for deployment
- Kubernetes for orchestration
- CI/CD pipelines (GitHub Actions, Jenkins)

Monitoring:
- Application performance monitoring (New Relic, Datadog)
- Error tracking (Sentry)
- Analytics (Google Analytics, Mixpanel)
- Uptime monitoring (Pingdom, UptimeRobot)
```

### Data Backup and Redundancy

**Backup Strategy:**
```
Daily Backups:
- Incremental database backups
- Stored in geographically separate location
- Automated via cron jobs
- Verification of backup integrity

Weekly Backups:
- Full database dumps
- Full file system backups
- Tested restoration procedures
- Off-site storage (cloud + physical media)

Version Control:
- Git repository for all code
- GitHub/GitLab for hosting
- Branches for development, staging, production
- Tagged releases

Disaster Recovery:
- Documented recovery procedures
- Recovery Time Objective (RTO): 24 hours
- Recovery Point Objective (RPO): 24 hours (daily backups)
- Annual disaster recovery drills
```

### Computational Infrastructure

**Sequence Analysis Tools:**
```
On-Demand Computation:
- BLAST searches against registry
- Multiple sequence alignments
- Phylogenetic analysis
- Secondary structure prediction
- Codon optimization
- Restriction site analysis
- Primer design

Resource Requirements:
- CPU: 32-64 cores for compute cluster
- Memory: 256 GB RAM for large datasets
- Storage: 10 TB for sequence databases
- Network: High-bandwidth for data transfer
```

**Machine Learning Infrastructure:**
```
Training Pipeline:
- GPU cluster for model training (NVIDIA A100 or similar)
- TensorFlow or PyTorch frameworks
- Automated hyperparameter tuning
- Model versioning (MLflow, DVC)

Inference:
- Deployed models for part quality prediction
- Function classification
- Recommendation systems
- Real-time or batch processing

Data:
- Training data: Curated registry subset
- Labels: Quality tiers, functional categories
- Features: Sequence-derived, metadata-derived
- Regular retraining as registry grows
```

### API and Developer Tools

**RESTful API v2 Specification:**
```
Base URL: https://api.igem.org/v2/

Authentication:
GET /auth/token
POST /auth/register

Parts:
GET /parts/:part_id
GET /parts/search?query=promoter&tier=3
POST /parts (requires auth)
PUT /parts/:part_id (requires auth)

Sequences:
GET /sequences/:part_id (returns GenBank, FASTA)
POST /sequences/blast (sequence similarity search)

Characterization:
GET /characterization/:part_id
POST /characterization (submit data, requires auth)

Collections:
GET /collections (list collections)
GET /collections/:collection_id (parts in collection)

Rate Limiting:
- Public API: 1000 requests/hour
- Authenticated: 5000 requests/hour
- Burst: 100 requests/minute

Response Format: JSON
Error Codes: Standard HTTP (200, 400, 401, 404, 429, 500)
```

**SDK Libraries:**
```python
# Python SDK example
from igem_registry import Registry

# Initialize client
registry = Registry(api_key='your_key_here')

# Search for parts
promoters = registry.search(type='promoter', tier=3, organism='E. coli')

# Get specific part
part = registry.get_part('BBa_J23100')
print(part.sequence)
print(part.characterization_data)

# Submit characterization data
registry.submit_characterization(
    part_id='BBa_J23100',
    data={
        'expression_level': 1.24,
        'replicate_count': 3,
        'conditions': {...}
    }
)
```

```javascript
// JavaScript SDK example
const Registry = require('@igem/registry-sdk');

const registry = new Registry({ apiKey: process.env.IGEM_API_KEY });

// Async/await usage
async function getPart(partId) {
    const part = await registry.parts.get(partId);
    console.log(part.sequence);
    return part;
}

// Search with filters
registry.parts.search({
    type: 'promoter',
    tier: 3,
    organism: 'E. coli'
}).then(results => {
    results.forEach(part => console.log(part.name));
});
```

## 6.6 Global Collaboration Networks

Registries increasingly operate through international partnerships:

### International Synthetic Biology Organizations

**Global Biofoundries Alliance (GBA):**
- Network of automated biofoundries
- Shared part characterization
- Standardized protocols
- Distributed manufacturing
- Registry integration for part access

**Engineering Biology Research Consortium (EBRC):**
- US-based, international participation
- Technical standards development
- Registry best practices
- Quality benchmarks
- Data interoperability

**Global Biofoundry Network (GBN):**
- Academic biofoundries worldwide
- Collaborative research projects
- Student exchange programs
- Shared resources and parts
- Registry contributions

### Regional Registry Initiatives

**European Registry Network:**
- BIOSS (Germany) - 2,000 parts
- Imperial College London (UK) - 1,500 parts
- INSA Lyon (France) - 800 parts
- Synchronized catalog, distributed storage

**Asian Synthetic Biology Consortium:**
- Peking University (China) - lead
- University of Tokyo (Japan)
- KAIST (South Korea)
- IIT Bombay (India)
- Focus: Regional part collections, organism diversity

**Pan-African Synthetic Biology Network:**
- Emerging initiative
- Focus on capacity building
- Adapted parts for local organisms
- Solving regional challenges (agriculture, health)
- Registry node planned for Nairobi or Cape Town

### Academic-Industrial Partnerships

**Joint Development Programs:**
- Companies contribute characterized parts
- Universities provide students and research
- Shared IP under defined agreements
- Registry benefits from both contributions

**Examples:**
- **Ginkgo Bioworks + iGEM:** Characterization support, mentorship
- **Twist Bioscience + Registry:** Free synthesis for registry parts
- **Zymergen + Universities:** Promoter libraries, shared data
- **Benchling + Registry:** Software integration, free licenses for students

## 6.7 Sustainability and Funding Models

Long-term registry operation requires sustainable financing:

### Funding Sources

**Academic Registries (e.g., iGEM Registry):**
```
Revenue (Typical Annual Budget: $1.2M):
- iGEM Competition fees: 60% ($720K)
- Research grants (NSF, NIH): 25% ($300K)
- Institutional support (MIT): 10% ($120K)
- Corporate sponsorship: 5% ($60K)

Expenditures:
- Personnel (5 FTE): 55%
- Infrastructure (servers, freezers): 20%
- Physical repository maintenance: 13%
- Development and upgrades: 8%
- Outreach and documentation: 4%
```

**Repository Service Models (e.g., AddGene):**
```
Revenue (Cost-Recovery Model):
- Per-plasmid fees: $65 (non-profit), $120 (commercial)
- Annual depositor fees: $250/lab
- Premium services (sequence verification, expedited shipping)
- Grant funding for specific collections

Expenditures:
- Personnel: 60%
- Materials and reagents: 15%
- Infrastructure: 12%
- Shipping and distribution: 10%
- Other: 3%
```

**Industry-Sponsored Registries:**
```
Revenue:
- Corporate funding: 80%
- User fees (commercial applications): 15%
- Licensing revenue: 5%

Expenditures:
- Advanced characterization: 35%
- High-throughput testing: 30%
- Personnel: 20%
- Infrastructure: 10%
- Other: 5%
```

### Cost Reduction Strategies

**Automation:**
- Robotic liquid handling reduces labor
- Automated sequence analysis
- Self-service user interfaces
- Reduced curation burden

**Cloud Infrastructure:**
- Pay-for-what-you-use model
- Eliminates capital expenditures
- Scalability without upfront costs
- Managed services reduce admin burden

**Open Source Software:**
- Leverage community development
- Avoid licensing fees
- Customizable to needs
- Community support

**Volunteer Contributions:**
- Student labor (educational value)
- Community curation
- Open science ethos
- Reduced staffing costs

**Partnerships:**
- Shared infrastructure with other initiatives
- In-kind contributions from companies
- Collaborative grants
- Cost-sharing arrangements

---

## Key Takeaways

✓ Biological registries require both digital and physical infrastructure
✓ Long-term sample preservation uses glycerol stocks, plasmid DNA, or emerging synthesis-on-demand
✓ Distribution models include annual kits, on-demand requests, and regional hubs
✓ International shipping faces complex regulatory requirements varying by country
✓ Robust data infrastructure includes databases, APIs, backups, and computational tools
✓ Global collaboration networks enhance registry reach and capabilities
✓ Sustainability requires diverse funding sources and cost-reduction strategies
✓ Regional hubs reduce shipping costs and time while navigating regulations
✓ Synthesis-on-demand integration offers alternative to physical storage
✓ Infrastructure investment essential for registry longevity and impact

---

**Next Chapter:** Chapter 7 explores data standards and interoperability, examining how registries exchange information and integrate with design tools.

---

*WIA-SYNTHETIC-BIO-REGISTRY Standard v1.0*
*© 2025 SmileStory Inc. / WIA*
*弘益人間 (Benefit All Humanity)*
