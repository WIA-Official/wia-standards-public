# WIA-CITY-006: Building Information Modeling - Phase 3 Protocols

**Version:** 1.0
**Status:** Draft
**Category:** CITY
**Last Updated:** 2025-12-25

---

## 1. Overview

This specification defines the protocols, workflows, and communication standards for BIM collaboration under WIA-CITY-006. It establishes openBIM practices, coordination procedures, and quality assurance protocols.

---

## 2. OpenBIM Principles

### 2.1 Core Tenets

**Open Standards:**
- Vendor-neutral data formats (IFC, BCF)
- Transparent specifications
- Public documentation

**Interoperability:**
- Cross-platform data exchange
- No vendor lock-in
- Software-agnostic workflows

**Collaboration:**
- Multi-disciplinary coordination
- Shared data environments
- Transparent communication

### 2.2 BuildingSMART Standards

**Adopted Standards:**
- IFC4 (ISO 16739-1:2018) - Data schema
- BCF 3.0 - Issue management
- bsDD (buildingSMART Data Dictionary) - Terminology
- IDM (Information Delivery Manual) - Process definitions
- MVD (Model View Definition) - Exchange requirements

---

## 3. IFC Exchange Protocol

### 3.1 Export Protocol

**Pre-Export Checklist:**
```
1. Model validation complete
2. Coordinate system verified
3. Units consistent (SI preferred)
4. Element classification assigned
5. Required properties populated
6. Naming conventions followed
7. Spatial hierarchy intact
8. Duplicate elements removed
```

**Export Settings Template:**
```json
{
  "ifc_export_configuration": {
    "ifc_version": "IFC4",
    "view_definition": "CoordinationView_V2.0",
    "exchange_requirement": "ARCHITECTURE|STRUCTURE|MEP",
    "geometry": {
      "type": "solid_models",
      "tessellation": "medium",
      "include_openings": true,
      "include_space_boundaries": true
    },
    "properties": {
      "base_quantities": true,
      "custom_property_sets": true,
      "material_properties": true,
      "classification": true
    },
    "spatial_structure": {
      "site": true,
      "building": true,
      "building_storey": true,
      "spaces": true
    },
    "units": {
      "length": "METRE",
      "area": "SQUARE_METRE",
      "volume": "CUBIC_METRE",
      "angle": "DEGREE"
    },
    "coordinate_system": {
      "use_project_base_point": true,
      "include_georeferencing": true
    }
  }
}
```

**Export Process:**
```
1. Clean model (remove temporary elements)
2. Run internal validation
3. Apply export configuration
4. Generate IFC file
5. Validate exported IFC
6. Generate checksum (SHA-256)
7. Create metadata manifest
8. Upload to CDE (Common Data Environment)
9. Notify stakeholders
```

### 3.2 Import Protocol

**Pre-Import Validation:**
```python
def validate_ifc_import(ifc_file):
    """Validate IFC file before import"""
    checks = {
        # File integrity
        "file_readable": check_file_integrity(ifc_file),
        "checksum_valid": verify_checksum(ifc_file),

        # Schema validation
        "ifc_version": check_ifc_version(ifc_file),
        "schema_valid": validate_schema(ifc_file),

        # Content validation
        "has_project": has_ifcproject(ifc_file),
        "spatial_structure": validate_spatial_hierarchy(ifc_file),
        "units_defined": check_units(ifc_file),

        # Coordinate system
        "coordinate_system": validate_coordinates(ifc_file),
        "georeferencing": check_georef(ifc_file)
    }

    return {
        "valid": all(checks.values()),
        "checks": checks,
        "warnings": generate_warnings(checks),
        "errors": generate_errors(checks)
    }
```

**Import Settings:**
```json
{
  "ifc_import_configuration": {
    "coordinate_alignment": {
      "method": "project_base_point|survey_point|manual",
      "tolerance": "5mm"
    },
    "unit_conversion": {
      "auto_convert": true,
      "target_units": "project_units"
    },
    "duplicate_handling": {
      "check_guid": true,
      "action": "skip|overwrite|rename"
    },
    "property_mapping": {
      "map_to_native_types": true,
      "preserve_custom_psets": true
    },
    "geometry_options": {
      "import_openings": true,
      "import_spaces": true,
      "simplification": "none|low|medium"
    }
  }
}
```

### 3.3 Round-Trip Testing

**Validation Procedure:**
```
1. Export model to IFC (Model A → IFC1)
2. Import IFC into different software (IFC1 → Model B)
3. Re-export from second software (Model B → IFC2)
4. Compare IFC1 and IFC2
5. Document differences
6. Verify critical data preserved:
   - Element count
   - Geometry accuracy (±5mm)
   - Property preservation (>95%)
   - Spatial relationships
   - Classification codes
```

**Acceptance Criteria:**
```
- Geometry deviation: <5mm
- Element loss: <1%
- Property preservation: >95%
- Relationship integrity: 100%
- Classification retention: 100%
```

---

## 4. BCF (BIM Collaboration Format)

### 4.1 BCF Protocol

**BCF 3.0 Implementation:**

BCF enables issue-based collaboration without exchanging full models.

**Use Cases:**
- Clash detection issues
- Design coordination comments
- RFI (Request for Information)
- Quality assurance findings
- Change requests

### 4.2 BCF Issue Structure

**Issue Creation:**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<Markup>
  <Header>
    <File IfcProject="3rNg3TA5D0ABT0XbC$IHiC"
          IfcSpatialStructureElement="..."
          isExternal="false">
      <Filename>project_model.ifc</Filename>
      <Date>2025-12-25T10:00:00Z</Date>
      <Reference>https://bim.server.com/model/latest.ifc</Reference>
    </File>
  </Header>

  <Topic Guid="issue-guid-1234"
         TopicType="Clash"
         TopicStatus="Open">
    <ReferenceLink>
      https://bim.server.com/issues/issue-guid-1234
    </ReferenceLink>
    <Title>Structural beam conflicts with HVAC duct at Level 3</Title>
    <Priority>High</Priority>
    <Index>1</Index>
    <Labels>Coordination MEP Structural</Labels>
    <CreationDate>2025-12-25T10:00:00Z</CreationDate>
    <CreationAuthor>coordination@example.com</CreationAuthor>
    <ModifiedDate>2025-12-25T10:00:00Z</ModifiedDate>
    <ModifiedAuthor>coordination@example.com</ModifiedAuthor>
    <DueDate>2025-12-30T23:59:59Z</DueDate>
    <AssignedTo>structural@example.com</AssignedTo>
    <Stage>Construction Documentation</Stage>
    <Description>
      Main structural beam (BEAM-L3-A12) intersects with primary
      supply duct (DUCT-L3-S01) in Grid A-B, near column A3.
      Conflict volume approximately 0.12 m³.
    </Description>

    <BimSnippet SnippetType="IfcWall"
                 isExternal="true">
      <Reference>https://server.com/snippets/beam-a12.ifc</Reference>
      <ReferenceSchema>IFC4</ReferenceSchema>
    </BimSnippet>

    <DocumentReference Guid="doc-ref-001"
                       isExternal="true">
      <ReferencedDocument>
        https://server.com/docs/clash-report-2025-12-25.pdf
      </ReferencedDocument>
      <Description>Automated clash detection report</Description>
    </DocumentReference>

    <RelatedTopic Guid="issue-guid-0987"/>
  </Topic>

  <Comment Guid="comment-001">
    <Date>2025-12-25T11:30:00Z</Date>
    <Author>structural@example.com</Author>
    <Comment>
      Reviewed with MEP. Proposing to raise beam by 150mm.
      Will not impact ceiling height. Updated model uploaded.
    </Comment>
    <Viewpoint Guid="viewpoint-001"/>
    <ModifiedDate>2025-12-25T11:30:00Z</ModifiedDate>
    <ModifiedAuthor>structural@example.com</ModifiedAuthor>
  </Comment>

  <Viewpoints>
    <ViewPoint Guid="viewpoint-001">
      <Viewpoint>viewpoint-001.bcfv</Viewpoint>
      <Snapshot>snapshot-001.png</Snapshot>
      <Index>0</Index>
    </ViewPoint>
  </Viewpoints>
</Markup>
```

**Viewpoint Definition:**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<VisualizationInfo Guid="viewpoint-001">
  <Components>
    <Selection>
      <Component IfcGuid="beam-guid-a12"/>
      <Component IfcGuid="duct-guid-s01"/>
    </Selection>
    <Visibility DefaultVisibility="false">
      <Exceptions>
        <Component IfcGuid="beam-guid-a12"/>
        <Component IfcGuid="duct-guid-s01"/>
      </Exceptions>
    </Visibility>
    <Coloring>
      <Color Color="FF0000">
        <Component IfcGuid="beam-guid-a12"/>
      </Color>
      <Color Color="00FF00">
        <Component IfcGuid="duct-guid-s01"/>
      </Color>
    </Coloring>
  </Components>

  <OrthogonalCamera>
    <CameraViewPoint>
      <X>10.5</X><Y>25.3</Y><Z>12.8</Z>
    </CameraViewPoint>
    <CameraDirection>
      <X>-0.5</X><Y>-0.7</Y><Z>-0.5</Z>
    </CameraDirection>
    <CameraUpVector>
      <X>0</X><Y>0</Y><Z>1</Z>
    </CameraUpVector>
    <ViewToWorldScale>50.0</ViewToWorldScale>
  </OrthogonalCamera>

  <ClippingPlanes>
    <ClippingPlane>
      <Location><X>10</X><Y>25</Y><Z>10</Z></Location>
      <Direction><X>0</X><Y>0</Y><Z>1</Z></Direction>
    </ClippingPlane>
  </ClippingPlanes>
</VisualizationInfo>
```

### 4.3 BCF Workflow

```
Issue Lifecycle:
1. Creation → Open
2. Assignment → In Progress
3. Resolution → Resolved (pending verification)
4. Verification → Closed
5. If rejected → Reopened

Status Values:
- Open
- In Progress
- Resolved
- Closed
- Reopened

Priority Values:
- Critical
- High
- Medium
- Low

Topic Types:
- Clash
- Design
- Safety
- Construction
- Quality
- Request
- Other
```

---

## 5. Coordination Protocols

### 5.1 Model Federation

**Federation Strategy:**

```
Federated Model
├── Architecture (reference)
├── Structure (reference)
├── MEP (reference)
│   ├── Mechanical (reference)
│   ├── Electrical (reference)
│   └── Plumbing (reference)
├── Civil (reference)
└── Coordination Spaces (editable)
```

**Reference Model Protocol:**
```json
{
  "reference_configuration": {
    "update_frequency": "daily|weekly|milestone",
    "auto_reload": false,
    "coordinate_monitoring": {
      "enabled": true,
      "alert_on_mismatch": true
    },
    "version_control": {
      "track_versions": true,
      "allow_rollback": true
    },
    "visibility": {
      "default": "visible",
      "override_allowed": true
    },
    "selectability": {
      "elements": false,
      "for_measurement": true
    }
  }
}
```

### 5.2 Clash Detection Protocol

**Detection Frequency:**
- Daily: Automated lightweight checks
- Weekly: Full coordination review
- Milestone: Comprehensive validation

**Clash Types:**

```python
class ClashType(Enum):
    HARD_CLASH = "geometric_intersection"
    SOFT_CLASH = "clearance_violation"
    WORKFLOW_CLASH = "construction_sequence"
    DUPLICATE = "duplicate_element"

class ClashSeverity(Enum):
    CRITICAL = "immediate_resolution_required"
    HIGH = "resolve_before_milestone"
    MEDIUM = "coordinate_in_next_meeting"
    LOW = "document_and_monitor"
```

**Clash Matrix:**
```
Priority Matrix (Discipline A vs Discipline B):

           ARCH  STRUC  MECH  ELEC  PLUMB  CIVIL
ARCH        N/A   HIGH   MED   MED   MED    HIGH
STRUC      HIGH   N/A   CRIT  CRIT  CRIT   CRIT
MECH       MED   CRIT   MED   HIGH  HIGH   MED
ELEC       MED   CRIT   HIGH  LOW   MED    MED
PLUMB      MED   CRIT   HIGH  MED   LOW    MED
CIVIL      HIGH  CRIT   MED   MED   MED    N/A

CRIT = Critical - Resolve immediately
HIGH = High - Resolve within 48 hours
MED = Medium - Resolve within 1 week
LOW = Low - Document and monitor
```

**Clash Resolution Workflow:**
```
1. Detection (automated)
   ↓
2. Classification (BIM Coordinator)
   ↓
3. BCF Issue Creation
   ↓
4. Assignment to responsible party
   ↓
5. Design review and resolution
   ↓
6. Model update
   ↓
7. Re-check (automated)
   ↓
8. Verification (BIM Coordinator)
   ↓
9. Close BCF Issue
```

### 5.3 Coordination Meetings

**Weekly Coordination Meeting Protocol:**

**Agenda Template:**
```
1. Review of previous action items (15 min)
2. New clash report review (30 min)
   - Critical clashes
   - High priority clashes
   - Trends and patterns
3. Design changes discussion (20 min)
4. Upcoming milestones (10 min)
5. Action item assignment (10 min)
6. Next meeting schedule (5 min)

Total: 90 minutes
```

**Meeting Outputs:**
- Updated clash report with assignments
- BCF issues created/updated
- Action item register
- Meeting minutes

---

## 6. Common Data Environment (CDE)

### 6.1 CDE Structure (ISO 19650)

**Information Containers:**
```
Common Data Environment
├── Work in Progress (WIP)
│   ├── Discipline folders
│   └── Individual working files
├── Shared
│   ├── Submitted for review
│   └── Pending approval
├── Published
│   ├── Approved for use
│   └── Version controlled
└── Archive
    ├── Superseded versions
    └── Historical records
```

**File Naming in CDE:**
```
[PROJECT]-[ORIGINATOR]-[VOLUME]-[LEVEL]-[TYPE]-[ROLE]-[NUMBER]-[SUITABILITY]-[REVISION]

Example:
CITYTR-ABC-ZZ-XX-M3-A-001-S2-P03

Where:
- CITYTR: Project code
- ABC: Originator (company/discipline)
- ZZ: Volume (building zone)
- XX: Level (floor or system)
- M3: Type (Model - IFC)
- A: Role (Architecture)
- 001: Sequential number
- S2: Suitability code (see below)
- P03: Revision (P=Published)

Suitability Codes (ISO 19650):
- S0: Work in Progress
- S1: Suitable for Coordination
- S2: Suitable for Information
- S3: Suitable for Review & Comment
- S4: Suitable for Stage Approval
- S6: Suitable for PIM Authorization (handover)
- S7: Suitable for AIM Authorization (operations)
```

### 6.2 Version Control Protocol

**Version Numbering:**
```
Major.Minor.Revision

Examples:
- 1.0.0: Initial model
- 1.1.0: Minor design update
- 1.1.1: Correction/fix
- 2.0.0: Major milestone (e.g., CD complete)

Published versions:
- P01, P02, P03... (incremental)

Working versions:
- WIP01, WIP02... (not shared)
- D01, D02... (drafts for internal review)
```

**Change Log:**
```json
{
  "version": "2.1.3",
  "date": "2025-12-25",
  "author": "architect@example.com",
  "changes": [
    {
      "type": "addition",
      "description": "Added curtain wall on west facade",
      "elements_affected": 15,
      "reason": "Client request"
    },
    {
      "type": "modification",
      "description": "Adjusted column grid A-3 spacing",
      "elements_affected": 8,
      "reason": "Structural coordination"
    },
    {
      "type": "deletion",
      "description": "Removed temporary construction walls",
      "elements_affected": 6,
      "reason": "Model cleanup"
    }
  ],
  "clashes_resolved": 12,
  "new_clashes": 3,
  "lod_level": 300,
  "validation_status": "passed"
}
```

---

## 7. Quality Assurance Protocol

### 7.1 Model Quality Checks

**Automated QA Checklist:**
```python
def run_qa_checks(model):
    """Comprehensive model quality assurance"""

    results = {}

    # Geometric checks
    results['geometry'] = {
        'no_duplicates': check_duplicate_elements(model),
        'no_overlaps': check_overlapping_geometry(model),
        'valid_geometry': validate_all_geometry(model),
        'within_tolerance': check_geometric_tolerance(model)
    }

    # Data checks
    results['data'] = {
        'properties_complete': check_required_properties(model),
        'classification_assigned': check_classification(model),
        'materials_defined': check_materials(model),
        'lod_compliance': validate_lod_requirements(model)
    }

    # Coordination checks
    results['coordination'] = {
        'coordinate_system': validate_coordinate_system(model),
        'spatial_hierarchy': check_spatial_structure(model),
        'element_relationships': validate_relationships(model)
    }

    # Standards compliance
    results['standards'] = {
        'naming_convention': check_naming_standards(model),
        'ifc_export_ready': validate_ifc_export(model),
        'bcf_compatible': check_bcf_compatibility(model)
    }

    results['overall_score'] = calculate_quality_score(results)
    results['pass'] = results['overall_score'] >= 95

    return results
```

**Quality Score Calculation:**
```
Quality Score = Weighted Average of:
- Geometry Quality (25%)
- Data Completeness (30%)
- Coordination Compliance (25%)
- Standards Adherence (20%)

Pass Threshold: ≥95%
Warning Threshold: 90-94%
Fail Threshold: <90%
```

### 7.2 Review Gates

**Project Phase Gates:**

```
Conceptual Design (LOD 100):
✓ Massing approved
✓ Basic program validated
✓ Site constraints identified

Schematic Design (LOD 200):
✓ Major systems defined
✓ Preliminary coordination complete
✓ Cost estimate within budget (±20%)

Design Development (LOD 300):
✓ All disciplines coordinated
✓ Zero critical clashes
✓ MEP systems routed
✓ Cost estimate refined (±10%)
✓ Permit-ready documentation

Construction Documentation (LOD 350):
✓ Comprehensive clash detection
✓ All disciplines coordinated
✓ Constructability review complete
✓ Shop drawing coordination
✓ Cost estimate final (±5%)

Fabrication (LOD 400):
✓ Fabrication details complete
✓ Shop drawings approved
✓ Material procurement initiated

As-Built (LOD 500):
✓ Field verification complete
✓ Commissioning data integrated
✓ O&M manuals linked
✓ FM system handover ready
```

---

## 8. Security and Access Control

### 8.1 Authentication Protocol

**User Authentication:**
```json
{
  "authentication": {
    "method": "SSO|OAuth2|SAML",
    "mfa_required": true,
    "session_timeout": 3600,
    "password_policy": {
      "min_length": 12,
      "complexity": "high",
      "rotation_days": 90
    }
  }
}
```

### 8.2 Authorization Matrix

```
Role-Based Access Control (RBAC):

Role              WIP    Shared  Published  Archive  BCF    Settings
─────────────────────────────────────────────────────────────────────
BIM Manager       RW     RW      RW         RW       RW     RW
Discipline Lead   RW     RW      R          R        RW     R
Modeler           RW     R       R          R        RW     -
Coordinator       RW     RW      R          R        RW     R
Reviewer          R      R       R          R        RW     -
Client            -      -       R          -        R      -
Contractor        -      R       R          R        RW     -

R = Read, W = Write, - = No Access
```

### 8.3 Audit Trail

**Logging Requirements:**
```json
{
  "audit_log_entry": {
    "timestamp": "ISO-8601",
    "user_id": "string",
    "action": "create|read|update|delete|download|export",
    "resource_type": "model|document|issue",
    "resource_id": "string",
    "ip_address": "string",
    "user_agent": "string",
    "result": "success|failure",
    "changes": {
      "before": {},
      "after": {}
    }
  }
}
```

**Retention Policy:**
- Active project: Real-time access
- Post-completion: 7 years minimum
- Legal hold: Indefinite (as required)

---

## 9. API Protocol (WIA-OMNI-API Integration)

### 9.1 RESTful API Endpoints

**Model Access API:**
```
GET    /api/v1/projects/{project_id}/models
GET    /api/v1/projects/{project_id}/models/{model_id}
POST   /api/v1/projects/{project_id}/models
PUT    /api/v1/projects/{project_id}/models/{model_id}
DELETE /api/v1/projects/{project_id}/models/{model_id}

GET    /api/v1/models/{model_id}/elements
GET    /api/v1/models/{model_id}/elements/{element_guid}
GET    /api/v1/models/{model_id}/elements/{element_guid}/properties

GET    /api/v1/models/{model_id}/clashes
POST   /api/v1/models/{model_id}/clashes/detect

GET    /api/v1/projects/{project_id}/bcf/issues
POST   /api/v1/projects/{project_id}/bcf/issues
PUT    /api/v1/projects/{project_id}/bcf/issues/{issue_id}
```

**Example API Request:**
```http
GET /api/v1/models/abc123/elements/3rNg3TA5D0ABT0XbC$IHiC HTTP/1.1
Host: api.bim.example.com
Authorization: Bearer eyJhbGc...
Accept: application/json
```

**Example API Response:**
```json
{
  "element": {
    "guid": "3rNg3TA5D0ABT0XbC$IHiC",
    "ifc_type": "IfcWall",
    "name": "Exterior Wall - West",
    "lod": 300,
    "geometry": {
      "bounding_box": {
        "min": [0, 0, 0],
        "max": [6, 0.3, 3],
        "unit": "m"
      },
      "volume": 5.4,
      "area": 18.0
    },
    "properties": {
      "is_external": true,
      "fire_rating": "2_hour",
      "thermal_transmittance": 0.25
    },
    "classification": [
      {
        "system": "Uniformat",
        "code": "B2010.10"
      }
    ],
    "relationships": {
      "contained_in": "IfcBuildingStorey-guid",
      "connects": ["IfcWall-guid-2", "IfcSlab-guid-1"]
    }
  }
}
```

---

## 10. Appendices

### Appendix A: BCF Implementation Guide

Step-by-step guide for BCF server setup and client integration.

### Appendix B: Clash Detection Rules Library

Pre-configured clash detection rules for common scenarios.

### Appendix C: API Reference

Complete API documentation for WIA-CITY-006 services.

### Appendix D: Certification Checklist

Self-assessment checklist for WIA-CITY-006 protocol compliance.

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
