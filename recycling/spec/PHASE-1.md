# WIA-ENE-023 PHASE-1: Foundation & Standards

**Version**: 1.0  
**Status**: Active  
**Category**: Core Specification  
**Theme Color**: #22C55E

---

## Philosophy

**弘益人間 (Hongik Ingan)** - Widely Benefit All Humanity

Advanced recycling standards serve the global community by establishing frameworks for sustainable material recovery, environmental protection, and circular economy development.

---

## 1. Executive Summary

WIA-ENE-023 Phase 1 establishes foundational standards for advanced recycling systems, defining terminology, measurement methodologies, baseline performance requirements, and data interchange formats. This phase provides the essential infrastructure upon which subsequent phases build advanced capabilities.

### 1.1 Scope

Phase 1 covers:
- Standard terminology and definitions
- Material classification systems
- Baseline performance metrics
- Data format specifications
- Quality measurement protocols
- Safety requirements fundamentals
- Environmental compliance frameworks
- Interoperability standards

### 1.2 Objectives

1. **Standardization**: Establish common terminology and classification systems
2. **Measurement**: Define consistent performance measurement methodologies
3. **Data Exchange**: Specify standard data formats and APIs
4. **Baseline Performance**: Set minimum performance requirements
5. **Safety Foundation**: Define fundamental safety protocols
6. **Environmental Framework**: Establish environmental management requirements
7. **Quality Assurance**: Specify quality control methodologies
8. **Interoperability**: Enable system integration across vendors

### 1.3 Target Audience

- Material Recovery Facility (MRF) operators
- Equipment manufacturers and vendors
- Municipal recycling program managers
- Recycling technology developers
- Regulatory agencies and policymakers
- Sustainability professionals
- Academic researchers
- Investment and finance professionals

---

## 2. Terminology and Definitions

### 2.1 Core Terms

**Advanced Recycling**: Material recovery systems employing optical sorting, automated contamination detection, real-time quality monitoring, and/or chemical recycling processes achieving superior performance versus manual sorting.

**Material Recovery Facility (MRF)**: Specialized facility receiving, separating, and processing mixed recyclable materials into commodity-grade feedstocks for end-market sale.

**Recovery Rate**: Percentage of input material successfully recovered as marketable recyclable commodities, calculated as (output material mass / input material mass) × 100%.

**Purity Level**: Percentage of target material in recovered output stream, calculated as (target material mass / total output mass) × 100%.

**Contamination Rate**: Percentage of non-target or prohibited materials in input or output streams, calculated as (contamination mass / total mass) × 100%.

**Throughput**: Material processing rate measured in tonnes per hour (t/h) or tonnes per day (t/d).

**Single-Stream**: Collection system where all recyclables are commingled in one container.

**Dual-Stream**: Collection system separating paper/cardboard from containers (bottles, cans, plastics).

**Multi-Stream**: Collection system with three or more separate collection streams.

**Optical Sorting**: Automated material identification and separation using electromagnetic spectrum analysis (visible light, near-infrared, X-ray fluorescence).

**Near-Infrared (NIR) Spectroscopy**: Optical sorting technology identifying plastic polymers through analysis of reflected near-infrared light (1000-2500 nm wavelength).

**Eddy Current Separation**: Electromagnetic separation technology removing non-ferrous metals (aluminum, copper) from mixed waste streams.

**Chemical Recycling**: Process deconstructing polymers to molecular components (monomers or hydrocarbon fractions) for repolymerization or chemical feedstock use.

**Pyrolysis**: Thermal decomposition of organic materials in oxygen-free environment producing liquid hydrocarbons, gases, and solid char.

**Depolymerization**: Chemical process breaking polymer chains into constituent monomers.

### 2.2 Material Classifications

#### 2.2.1 Plastic Resins (SPI Codes)

- **PET (Polyethylene Terephthalate)** - SPI Code 1: Beverage bottles, food containers
- **HDPE (High-Density Polyethylene)** - SPI Code 2: Milk jugs, detergent bottles, pipes
- **PVC (Polyvinyl Chloride)** - SPI Code 3: Plumbing pipes, window profiles (generally non-recyclable in curbside)
- **LDPE (Low-Density Polyethylene)** - SPI Code 4: Plastic bags, shrink wrap, squeeze bottles
- **PP (Polypropylene)** - SPI Code 5: Food containers, automotive parts, textiles
- **PS (Polystyrene)** - SPI Code 6: Foam packaging, disposable cups, packaging peanuts
- **Other** - SPI Code 7: Mixed or specialty plastics (polycarbonate, acrylic, nylon, etc.)

#### 2.2.2 Paper and Cardboard

- **OCC (Old Corrugated Containers)**: Corrugated cardboard boxes
- **ONP (Old Newspapers)**: Newspapers and similar newsprint
- **OMG (Old Magazines)**: Magazines, catalogs, glossy publications
- **Sorted Office Paper**: White and colored office papers
- **Mixed Paper**: Varied paper grades commingled

#### 2.2.3 Metals

- **Ferrous Metals**: Steel cans, appliances, structural steel
- **Aluminum UBC (Used Beverage Containers)**: Aluminum beverage cans
- **Aluminum Scrap**: Non-can aluminum including foil, siding, auto parts
- **Copper**: Wire, plumbing, electronic components
- **Brass**: Fittings, valves, decorative items

#### 2.2.4 Glass

- **Clear/Flint Glass**: Colorless glass (highest value)
- **Green Glass**: Green-tinted bottles
- **Amber/Brown Glass**: Brown/amber bottles
- **Mixed Glass**: Commingled glass colors

### 2.3 Quality Grades

#### 2.3.1 PET Quality Grades

- **Premium Grade**: ≥98% PET, PVC <50 ppm, labels <1%, suitable for food-grade applications
- **Grade A**: ≥95% PET, PVC <200 ppm, labels <2%, suitable for bottle-to-bottle recycling
- **Grade B**: ≥90% PET, labels/caps <5%, suitable for fiber and strapping applications
- **Off-Grade**: <90% PET or excess contamination, suitable for low-value applications only

#### 2.3.2 HDPE Quality Grades

- **Natural HDPE**: ≥97% natural (unpigmented) HDPE-2, suitable for food-contact applications
- **Colored HDPE**: ≥95% HDPE-2 (colored), suitable for non-food containers
- **Mixed HDPE**: ≥90% HDPE-2 with color mixing, suitable for plastic lumber and pipes

#### 2.3.3 Paper Quality Grades (PS Standards)

Reference: Paper Stock Industries Circular PS-2023

- **PS-11 (OCC)**: ≥90% old corrugated containers, ≤1% prohibitives, ≤5% moisture
- **PS-54 (Sorted Office Paper)**: ≥98% sorted office paper, <1% outthrows
- **PS-56 (Sorted Residential Papers and News)**: ≥96% newsprint quality

---

## 3. Performance Metrics and Measurement

### 3.1 Recovery Rate Calculation

**Formula**: Recovery Rate (%) = (Output Material Mass / Input Material Mass) × 100

**Measurement Protocol**:
1. Weigh all incoming material over measurement period (e.g., 24 hours)
2. Weigh all recovered material outputs (bales, bins) over same period
3. Calculate ratio and express as percentage
4. Report daily, weekly, and monthly averages
5. Track by material type for detailed analysis

**Industry Benchmarks**:
- Basic MRF (manual sorting): 60-75%
- Standard MRF (mechanical systems): 75-85%
- Advanced MRF (optical sorting): 85-95%
- State-of-art MRF (AI + robotics): 90-97%

### 3.2 Purity Level Measurement

**Formula**: Purity (%) = (Target Material Mass / Total Output Mass) × 100

**Measurement Protocol**:
1. Sample output bales following statistical sampling plans (e.g., ASTM D5231)
2. Manually or mechanically sort samples into target material and contamination
3. Weigh each fraction
4. Calculate purity percentage
5. Test frequency: minimum every 5 bales or 50 tonnes (whichever occurs first)

**Acceptance Criteria**:
- Premium grade: ≥98% purity
- Standard grade: ≥95% purity
- Acceptable grade: ≥90% purity
- Below spec: <90% purity (requires reprocessing or price penalty)

### 3.3 Contamination Rate Monitoring

**Formula**: Contamination (%) = (Contamination Mass / Total Mass) × 100

**Categories**:
- **Physical**: Wrong material types, non-recyclables
- **Chemical**: Oils, solvents, hazardous substances
- **Biological**: Food waste, organic contamination

**Threshold Levels**:
- Excellent: <2% contamination
- Good: 2-5% contamination
- Fair: 5-10% contamination
- Poor: 10-20% contamination
- Unacceptable: >20% contamination

### 3.4 Throughput Measurement

**Measurement Points**:
- Input throughput: Material entering facility (weigh scales at receiving)
- Processing throughput: Material through sorting systems (belt scales)
- Output throughput: Material leaving as bales (bale weights)

**Calculation**: Throughput (t/h) = Total Material Mass (tonnes) / Operating Hours

**Performance Targets**:
- Small MRF: 10-30 t/h
- Medium MRF: 30-75 t/h
- Large MRF: 75-150 t/h
- Mega MRF: 150+ t/h

### 3.5 Equipment Efficiency Metrics

**Sorting Accuracy**: (Correct Sorts / Total Sorts) × 100

Target: ≥98% for optical sorters, ≥95% for manual sorting

**Equipment Uptime**: (Operating Time / Scheduled Time) × 100

Target: ≥95% for well-maintained facilities

**Maintenance Ratio**: (Preventive Maintenance Hours / Total Maintenance Hours) × 100

Target: ≥70% (majority of maintenance should be preventive, not reactive)

---

## 4. Data Standards and Interchange Formats

### 4.1 Material Batch Identifier Standard

**Format**: WIA-REC-{YEAR}-{FACILITY_ID}-{MATERIAL_TYPE}-{YYYYMMDD}-{BATCH_SEQ}

**Example**: WIA-REC-2025-FAC-001-PET-20250115-B0042

**Components**:
- `WIA-REC`: Standard prefix
- `YEAR`: Four-digit year
- `FACILITY_ID`: Unique facility identifier
- `MATERIAL_TYPE`: Material classification code
- `YYYYMMDD`: Processing date
- `BATCH_SEQ`: Batch sequence number

**Uniqueness**: Global uniqueness guaranteed through combination of facility ID, date, and sequence

### 4.2 JSON Data Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENE-023 Material Batch",
  "type": "object",
  "required": ["batchId", "facilityId", "materialType", "processingDate", "batchWeight"],
  "properties": {
    "batchId": {
      "type": "string",
      "pattern": "^WIA-REC-\\d{4}-[A-Z0-9\\-]+-[A-Z0-9\\-]+-\\d{8}-B\\d{4}$"
    },
    "facilityId": {
      "type": "string",
      "description": "Unique facility identifier"
    },
    "materialType": {
      "type": "string",
      "enum": ["PET-1-CLEAR", "PET-1-GREEN", "HDPE-2-NATURAL", "HDPE-2-COLORED", "PP-5", "AL-UBC", "OCC", "MIXED-PAPER"]
    },
    "collectionDate": {
      "type": "string",
      "format": "date-time"
    },
    "processingDate": {
      "type": "string",
      "format": "date-time"
    },
    "batchWeight": {
      "type": "number",
      "minimum": 0,
      "description": "Weight in kilograms"
    },
    "weightUnit": {
      "type": "string",
      "enum": ["kg", "tonnes", "lbs"],
      "default": "kg"
    },
    "purityLevel": {
      "type": "number",
      "minimum": 0,
      "maximum": 100,
      "description": "Percentage purity"
    },
    "contaminationRate": {
      "type": "number",
      "minimum": 0,
      "maximum": 100
    },
    "qualityGrade": {
      "type": "string",
      "enum": ["PREMIUM", "A", "B", "OFF-GRADE"]
    }
  }
}
```

### 4.3 RESTful API Specifications

**Base URL**: `https://api.facility.com/v1/`

**Authentication**: OAuth 2.0 Bearer Token

**Endpoints**:

```
POST /batches
GET  /batches/{batchId}
GET  /batches?facilityId={id}&startDate={date}&endDate={date}
POST /quality/report
GET  /quality/metrics
POST /chain-of-custody
GET  /sustainability/report
```

**HTTP Headers**:
```
Content-Type: application/json
Authorization: Bearer {access_token}
X-WIA-Standard: ENE-023-v1.0
```

**Rate Limits**: 1000 requests per hour per API key

### 4.4 Data Retention Requirements

- **Transactional Data**: Minimum 7 years
- **Quality Test Results**: Minimum 5 years
- **Chain of Custody**: Minimum 10 years
- **Environmental Reports**: Permanent retention
- **Audit Logs**: Minimum 7 years

---

## 5. Safety Requirements

### 5.1 Fundamental Safety Principles

1. **Hazard Elimination**: Remove hazards at source when possible
2. **Engineering Controls**: Physical guards, barriers, interlocks
3. **Administrative Controls**: Procedures, training, supervision
4. **Personal Protective Equipment**: Last line of defense when other controls insufficient

### 5.2 Required Safety Systems

**Emergency Stop Systems**:
- E-stop buttons accessible within 10 feet of all equipment
- Visible and easily identifiable (red mushroom-head buttons)
- Hard-wired circuits (not software-controlled)
- Monthly testing and documentation

**Machine Guarding**:
- All moving parts inaccessible during operation
- Guards interlocked (equipment stops when guard opened)
- Guards designed to prevent bypass
- Regular inspection (weekly minimum)

**Lockout/Tagout**:
- Written energy control procedures for all equipment
- Authorized personnel training
- Locks unique to each worker
- Zero-energy verification before maintenance
- Annual procedure review

**Fire Protection**:
- Automatic sprinkler systems (NFPA 13)
- Smoke detection throughout facility
- Fire extinguishers per NFPA 10 (maximum 75-foot travel distance)
- Annual fire drills
- Hot work permit program

### 5.3 Personal Protective Equipment (PPE)

**Minimum Requirements**:
- Safety glasses (ANSI Z87.1) in all processing areas
- Steel-toed boots (ASTM F2413) for equipment operators
- High-visibility vests in vehicle areas
- Hearing protection in areas exceeding 85 dBA
- Cut-resistant gloves (ANSI A2 minimum) for manual sorting

**Respiratory Protection**:
- Required when dust levels exceed PEL
- Fit testing annually
- Medical evaluation before use
- Cartridge replacement per manufacturer schedule

### 5.4 Safety Performance Targets

- **TRIR (Total Recordable Incident Rate)**: <2.0 (vs. industry average 4.2)
- **LTIR (Lost Time Incident Rate)**: <0.5 (vs. industry average 2.1)
- **Near Miss Reporting Rate**: >50 per 100 employees annually (leading indicator)
- **Safety Training**: Minimum 40 hours per employee annually

---

## 6. Environmental Compliance Framework

### 6.1 ISO 14001 Core Requirements

**Environmental Policy**:
- Top management commitment
- Commitment to legal compliance
- Commitment to pollution prevention
- Framework for environmental objectives
- Documented and communicated

**Environmental Aspects**:
- Identify activities, products, services with environmental impact
- Determine significant aspects
- Consider life cycle perspective
- Document and update regularly

**Objectives and Targets**:
- Consistent with environmental policy
- Measurable when practicable
- Monitored and reviewed
- Communicated to relevant parties

### 6.2 Key Environmental Aspects for MRFs

**Air Quality**:
- Dust emissions from material handling
- Vehicle exhaust from collection trucks, equipment
- Potential odors from organic contamination
- Control: dust collection systems, equipment maintenance, proper ventilation

**Water Quality**:
- Stormwater runoff (potential contamination from outdoor storage)
- Wash water from plastic recycling (if applicable)
- Control: stormwater management plan, treatment systems, spill prevention

**Waste Management**:
- Residual waste (non-recyclable materials)
- Equipment maintenance waste (oils, filters, hydraulic fluids)
- Office/administrative waste
- Control: minimization, proper disposal, hazardous waste protocols

**Energy Consumption**:
- Electrical power for sorting equipment, lighting, HVAC
- Diesel/gasoline for mobile equipment
- Control: energy efficiency upgrades, equipment optimization, renewable energy

**Noise**:
- Equipment operations (conveyors, sorters, balers)
- Vehicle movements
- Control: equipment enclosures, operational hour limits, maintenance

### 6.3 Permits and Reporting

**Typical Required Permits**:
- Air quality permit (if emissions exceed thresholds)
- Stormwater discharge permit (NPDES)
- Solid waste facility permit
- Building and zoning permits
- Fire safety permit

**Reporting Requirements**:
- Annual environmental report to regulatory agencies
- Greenhouse gas inventory (if applicable)
- Waste diversion metrics to municipalities
- Sustainability reporting to stakeholders

---

## 7. Quality Assurance Programs

### 7.1 Incoming Material Quality Control

**Procedures**:
- Visual inspection of incoming loads
- Random sampling and analysis (minimum 5% of loads)
- Contamination rate measurement
- Documentation of problem loads
- Feedback to haulers and municipalities

**Rejection Criteria**:
- Contamination exceeding 25%
- Presence of hazardous materials
- Excessive moisture (>10%)
- Large bulky items requiring special handling

### 7.2 Process Quality Control

**In-Process Monitoring**:
- Continuous monitoring via sensor systems
- Visual inspection at key process points
- Regular sampling from conveyor belts
- Real-time data dashboards
- Automated alerts for deviations

**Statistical Process Control**:
- Control charts for key metrics (purity, contamination, throughput)
- Identification of special cause vs. common cause variation
- Corrective action when trends approach control limits
- Documentation of adjustments

### 7.3 Output Quality Control

**Testing Requirements**:
- Bale sampling per ASTM D5231 or equivalent
- Purity testing every 5 bales or 50 tonnes
- Full material characterization monthly
- Third-party certification quarterly (for certified programs)

**Nonconformance Management**:
- Immediate segregation of off-spec material
- Root cause analysis
- Corrective action implementation
- Reprocessing or downgrading of material
- Customer notification when required

---

## 8. Implementation Roadmap

### 8.1 Phase 1 Milestones (Months 1-6)

**Month 1-2: Assessment**
- Gap analysis against WIA-ENE-023 requirements
- Baseline performance measurement
- Team formation and training
- Vendor evaluation (if new equipment needed)

**Month 3-4: Planning**
- Detailed implementation plan development
- Budget allocation and approval
- Equipment procurement initiation
- Procedure documentation

**Month 5-6: Initial Implementation**
- Core system deployment
- Staff training programs
- Pilot testing
- Initial data collection and reporting

### 8.2 Success Criteria

- All required safety systems operational
- Environmental management system documented
- Quality assurance program implemented
- Data reporting capability established
- Staff trained and competent
- Baseline performance metrics documented

### 8.3 Transition to Phase 2

Phase 1 completion prerequisites:
- ✓ 90% of safety requirements implemented
- ✓ Environmental policy and procedures documented
- ✓ Quality testing capability operational
- ✓ Data systems generating required reports
- ✓ Performance metrics meeting baseline targets
- ✓ Management review completed and approved

---

## 9. Compliance Verification

### 9.1 Self-Assessment Checklist

Organizations complete comprehensive checklist covering:
- Terminology and classification systems adopted
- Performance measurement systems operational
- Data interchange capabilities functional
- Safety systems implemented
- Environmental framework established
- Quality programs operational

### 9.2 Third-Party Audit

Optional but recommended:
- Independent verification of compliance
- Gap identification and remediation guidance
- Certification issuance (if applicable)
- Annual surveillance audits
- Three-year recertification cycle

### 9.3 Certification Levels

- **WIA-ENE-023-BASIC**: Phase 1 compliance
- **WIA-ENE-023-ADVANCED**: Phase 1+2 compliance
- **WIA-ENE-023-EXCELLENCE**: Phase 1+2+3 compliance
- **WIA-ENE-023-PLATINUM**: All phases + superior performance metrics

---

## 10. References

### 10.1 Normative References

- ISO 14001:2015 - Environmental management systems
- ISO 45001:2018 - Occupational health and safety management systems
- ASTM D5231 - Standard Test Method for Determination of the Composition of Unprocessed Municipal Solid Waste
- NFPA 13 - Standard for the Installation of Sprinkler Systems
- NFPA 10 - Standard for Portable Fire Extinguishers

### 10.2 Informative References

- Association of Plastic Recyclers (APR) Design Guide
- Paper Stock Industries (PSI) Circular PS-2023
- Ellen MacArthur Foundation Circular Economy Resources
- EPA WARM Model (Waste Reduction Model)

---

**Document Control**

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-01-15 | WIA Standards Committee | Initial release |

**Copyright** © 2025 World Industry Association  
**License**: WIA Open Standard License v1.0  
**弘益人間** · Benefit All Humanity
