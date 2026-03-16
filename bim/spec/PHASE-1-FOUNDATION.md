# WIA-CITY-006: Building Information Modeling - Phase 1 Foundation

**Version:** 1.0
**Status:** Draft
**Category:** CITY
**Last Updated:** 2025-12-25

---

## 1. Executive Summary

WIA-CITY-006 establishes a comprehensive standard for Building Information Modeling (BIM) implementation across the architecture, engineering, and construction (AEC) industry. This standard enables digital representation of physical and functional characteristics of facilities, supporting the entire project lifecycle from conceptual design through construction to facility operations.

### Philosophy
弘益人間 (홍익인간) - Benefit All Humanity

By standardizing BIM practices, we enable:
- Seamless collaboration across project stakeholders
- Reduced construction errors and rework
- Improved sustainability through better planning
- Enhanced facility management throughout building lifecycle

---

## 2. Scope and Objectives

### 2.1 Scope

This standard covers:

1. **Data Formats**
   - IFC (Industry Foundation Classes) schema compliance
   - Native format interoperability (Revit, ArchiCAD, Tekla)
   - Point cloud and reality capture integration

2. **Level of Development (LOD)**
   - LOD 100: Conceptual representation
   - LOD 200: Approximate geometry
   - LOD 300: Precise geometry
   - LOD 350: Construction documentation
   - LOD 400: Fabrication details
   - LOD 500: As-built verification

3. **Process Standards**
   - Model creation workflows
   - Coordination procedures
   - Quality assurance protocols
   - Change management

4. **Technology Integration**
   - 4D scheduling integration
   - 5D cost estimation
   - Digital twin connectivity
   - Facility management systems

### 2.2 Objectives

- **Interoperability:** Enable data exchange between all major BIM platforms
- **Quality:** Ensure model accuracy and reliability through LOD standards
- **Efficiency:** Reduce project delivery time through early conflict detection
- **Sustainability:** Support lifecycle analysis and sustainable design decisions
- **Transparency:** Provide verifiable credentials for model quality and compliance

---

## 3. Core Concepts

### 3.1 Building Information Modeling Fundamentals

**Definition:** BIM is an intelligent, 3D model-based process that provides architecture, engineering, and construction professionals with insights and tools to plan, design, construct, and manage buildings and infrastructure more efficiently.

**Key Characteristics:**
- Parametric modeling with intelligent objects
- Embedded metadata (materials, specifications, cost)
- Multi-disciplinary coordination
- Lifecycle information management

### 3.2 Information Hierarchy

```
Project Information Model
├── Architectural Model
│   ├── Walls, Floors, Roofs
│   ├── Doors, Windows
│   └── Spaces, Rooms
├── Structural Model
│   ├── Foundations
│   ├── Columns, Beams
│   └── Slabs, Structural Walls
├── MEP Model
│   ├── Mechanical (HVAC)
│   ├── Electrical
│   └── Plumbing
└── Site Model
    ├── Topography
    ├── Civil infrastructure
    └── Landscape
```

### 3.3 Level of Development (LOD) Framework

**LOD 100 - Conceptual**
- Symbolic or generic representation
- Volume, area, orientation only
- Use: Feasibility studies, massing

**LOD 200 - Approximate Geometry**
- Generic systems with approximate quantities
- Use: Preliminary design, cost estimates

**LOD 300 - Precise Geometry**
- Specific assemblies with precise quantities
- Use: Construction documentation, coordination

**LOD 350 - Construction Coordination**
- Precise geometry with interfaces to other systems
- Use: MEP coordination, clash detection

**LOD 400 - Fabrication**
- Sufficient detail for fabrication and assembly
- Use: Shop drawings, prefabrication

**LOD 500 - As-Built**
- Verified representation of as-constructed conditions
- Use: Facility management, renovation planning

---

## 4. Technical Architecture

### 4.1 Data Layer

**IFC Schema Support:**
```
IFC4 (ISO 16739:2018)
├── Spatial Structure
│   ├── IfcProject
│   ├── IfcSite
│   ├── IfcBuilding
│   └── IfcBuildingStorey
├── Building Elements
│   ├── IfcWall
│   ├── IfcSlab
│   ├── IfcBeam
│   └── IfcColumn
└── Systems
    ├── IfcDistributionSystem
    └── IfcBuildingSystem
```

**Coordinate System:**
- Project Base Point (PBP): Project origin in local coordinates
- Survey Point (SP): Real-world georeferencing
- True North: Orientation alignment

### 4.2 Metadata Standards

Required metadata for all BIM elements:

```json
{
  "element_id": "unique_guid",
  "ifc_type": "IfcWall",
  "lod": 300,
  "created_date": "2025-01-15T10:30:00Z",
  "modified_date": "2025-02-20T14:45:00Z",
  "author": "architect_name",
  "discipline": "architecture",
  "phase": "construction_documentation",
  "properties": {
    "material": "concrete",
    "fire_rating": "2_hour",
    "thermal_transmittance": 0.25,
    "structural_usage": "load_bearing"
  }
}
```

### 4.3 File Formats

**Primary Exchange Format:**
- IFC4 (.ifc) - ISO 16739:2018
- IFC4 ADD2 TC1 for latest features

**Native Formats (vendor-specific):**
- Autodesk Revit (.rvt)
- Graphisoft ArchiCAD (.pln)
- Trimble Tekla (.db1)
- Bentley MicroStation (.dgn)

**Supporting Formats:**
- Point Clouds: .e57, .rcp, .rcs
- Mesh: .obj, .fbx, .dae
- GIS Integration: .shp, .gml

---

## 5. Quality Standards

### 5.1 Model Quality Requirements

**Geometric Accuracy:**
- Tolerance: ±5mm for structural elements
- Tolerance: ±10mm for architectural elements
- Tolerance: ±2mm for fabrication (LOD 400)

**Data Integrity:**
- All elements must have valid IFC classification
- Required properties populated for current LOD
- No duplicate or overlapping geometry
- Valid spatial relationships

### 5.2 Clash Detection Standards

**Hard Clashes:** Physical geometric interference
- Zero tolerance for structural-structural clashes
- <10mm tolerance for architectural-structural
- Document and resolve all MEP-structural clashes

**Soft Clashes:** Clearance violations
- Minimum clearances per building codes
- Access and maintenance space requirements
- Construction sequencing conflicts

### 5.3 Validation Protocols

**Automated Checks:**
```
1. IFC schema validation
2. Geometric clash detection
3. Property completeness verification
4. Spatial hierarchy integrity
5. Coordinate system alignment
6. Unit consistency
7. Naming convention compliance
```

**Manual Reviews:**
- Discipline coordination meetings (weekly)
- Design review milestones (per project phase)
- Third-party quality audits (pre-construction)

---

## 6. Governance and Roles

### 6.1 BIM Execution Plan (BxP)

Required for all projects using WIA-CITY-006:

**Contents:**
1. Project information and goals
2. BIM uses and deliverables
3. LOD requirements by phase
4. Roles and responsibilities
5. Software and version standards
6. Coordinate system definition
7. File naming conventions
8. Collaboration procedures
9. Quality control process
10. Model delivery schedule

### 6.2 Project Roles

**BIM Manager:**
- Overall BIM strategy and standards enforcement
- Coordinate between disciplines
- Manage model federation
- Quality assurance oversight

**BIM Coordinators (per discipline):**
- Discipline model management
- Clash detection and resolution
- LOD compliance verification
- Data extraction and reporting

**BIM Modelers:**
- Create and update discipline models
- Follow modeling standards
- Document model changes
- Participate in coordination meetings

---

## 7. Security and Access Control

### 7.1 Access Levels

**Level 1 - View Only:**
- Read access to published models
- No editing capabilities
- Stakeholder review, client presentations

**Level 2 - Discipline Edit:**
- Full edit access to assigned discipline
- View-only to other disciplines
- Standard project team members

**Level 3 - Coordination:**
- Edit access to multiple disciplines
- Federation management
- BIM coordinators, leads

**Level 4 - Administration:**
- Full access to all models and settings
- Standards configuration
- BIM manager, project director

### 7.2 Data Protection

**In Transit:**
- TLS 1.3 encryption for cloud collaboration
- VPN required for external access

**At Rest:**
- AES-256 encryption for archived models
- Secure backup (3-2-1 rule)

**Audit Trail:**
- Version control with change attribution
- Access logs retained for project duration + 7 years

---

## 8. Compliance and Certification

### 8.1 Conformance Requirements

To claim WIA-CITY-006 compliance, implementations must:

1. Support IFC4 import/export
2. Implement LOD 100-500 framework
3. Provide clash detection capabilities
4. Support coordinate system definitions
5. Enable metadata management per specification
6. Implement validation protocols
7. Support verifiable credentials

### 8.2 Certification Process

**Self-Certification:**
- Complete conformance checklist
- Submit test cases
- Publish conformance statement

**Third-Party Certification:**
- Independent testing
- Interoperability validation
- Annual renewal

---

## 9. Implementation Guidelines

### 9.1 Getting Started

**Phase 1: Assessment (Weeks 1-2)**
- Review current BIM capabilities
- Identify gaps vs. WIA-CITY-006 requirements
- Define implementation scope

**Phase 2: Planning (Weeks 3-4)**
- Develop BIM Execution Plan
- Establish naming conventions
- Configure software environments
- Train team members

**Phase 3: Pilot (Weeks 5-8)**
- Small-scale project implementation
- Test workflows and procedures
- Refine standards and templates

**Phase 4: Deployment (Week 9+)**
- Roll out to full project portfolio
- Continuous improvement
- Regular audits and updates

### 9.2 Best Practices

1. **Start with clear goals:** Define specific BIM uses and expected outcomes
2. **Invest in training:** Ensure team competency before project start
3. **Use templates:** Develop project templates with standards pre-configured
4. **Regular coordination:** Weekly clash detection and coordination meetings
5. **Document decisions:** Maintain record of design decisions and changes
6. **Plan for FM:** Consider facility management needs from project start

---

## 10. References

### 10.1 Standards

- ISO 16739:2018 - Industry Foundation Classes (IFC)
- ISO 19650 - Organization of information about construction works
- ISO 12006-3 - Building construction — Organization of information about construction works
- BuildingSMART IFC Specifications
- AIA G202-2013 - Project Building Information Modeling Protocol Form

### 10.2 Related WIA Standards

- WIA-CITY-001: Smart City Data Exchange
- WIA-CITY-005: Digital Twin Architecture
- WIA-INTENT: Intent-based Interaction
- WIA-OMNI-API: Universal API Gateway

---

## 11. Glossary

**BIM:** Building Information Modeling - Digital representation of physical and functional characteristics

**IFC:** Industry Foundation Classes - Open, neutral data format for BIM

**LOD:** Level of Development - Specification of content reliability at various stages

**Clash Detection:** Process of identifying geometric conflicts between building elements

**Federation:** Combining multiple discipline models into unified coordination model

**OpenBIM:** Universal approach to collaborative design, based on open standards

**COBie:** Construction Operations Building information exchange - FM data standard

**BCF:** BIM Collaboration Format - Standard for issue tracking

---

## Appendix A: File Naming Convention

```
[PROJECT]_[BUILDING]_[DISCIPLINE]_[ZONE]_[LEVEL]_[LOD]_[VERSION]

Examples:
CITYTR_TOWA_ARCH_CORE_L03_300_V12.rvt
CITYTR_TOWA_STRUC_ALL_FNDT_400_V05.rvt
CITYTR_TOWA_MEP_EAST_L02_300_V08.rvt

Components:
- PROJECT: 6-char project code
- BUILDING: 4-char building identifier
- DISCIPLINE: ARCH, STRUC, MEP, CIVIL
- ZONE: Building zone or ALL
- LEVEL: Floor level or system
- LOD: 100, 200, 300, 350, 400, 500
- VERSION: Sequential number
```

---

## Appendix B: Property Schema

Required properties by element type - see Phase 2 Data specification.

---

**Document Control:**
- Version: 1.0
- Date: 2025-12-25
- Status: Draft
- Author: WIA Standards Committee
- License: CC BY 4.0

---

弘益人間 (홍익인간) - Benefit All Humanity

© 2025 WIA (World Certification Industry Association)
