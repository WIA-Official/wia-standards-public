# WIA-CITY-006: Building Information Modeling - Phase 2 Data Formats

**Version:** 1.0
**Status:** Draft
**Category:** CITY
**Last Updated:** 2025-12-25

---

## 1. Overview

This specification defines the data formats, schemas, and structures for BIM implementation under WIA-CITY-006. It establishes standards for IFC data exchange, metadata requirements, and LOD-specific content.

---

## 2. IFC Data Format

### 2.1 IFC Schema Version

**Mandated Version:** IFC4 ADD2 TC1 (ISO 16739-1:2018)

**Backward Compatibility:**
- IFC2x3 TC1 (read-only for legacy projects)
- IFC4 (full support)
- IFC4.3 (forward compatibility planned)

### 2.2 IFC File Structure

**Header Section:**
```
ISO-10303-21;
HEADER;
FILE_DESCRIPTION(
  ('ViewDefinition [CoordinationView_V2.0]', 'LOD 300'),
  '2;1'
);
FILE_NAME(
  'project_name.ifc',
  '2025-12-25T10:00:00',
  ('Author Name'),
  ('Organization Name'),
  'Preprocessor Version',
  'Originating System',
  'Authorization'
);
FILE_SCHEMA(('IFC4'));
ENDSEC;
```

**Data Section Requirements:**
```
DATA;
/* Mandatory Root Elements */
#1=IFCPROJECT(guid, owner, name, description, ...);
#2=IFCUNITASSIGNMENT((length, area, volume, ...));
#3=IFCGEOMETRICREPRESENTATIONCONTEXT(...);

/* Spatial Hierarchy */
#10=IFCSITE(...);
#20=IFCBUILDING(...);
#30=IFCBUILDINGSTOREY(...);

/* Building Elements */
#100=IFCWALL(...);
#200=IFCSLAB(...);
...
ENDSEC;
END-ISO-10303-21;
```

### 2.3 Required IFC Entities

**All Projects Must Include:**

1. **Project Structure**
   - IfcProject (exactly 1)
   - IfcSite (1 or more)
   - IfcBuilding (1 or more)
   - IfcBuildingStorey (1 or more per building)

2. **Representation Context**
   - IfcGeometricRepresentationContext
   - IfcGeometricRepresentationSubContext (for different views)

3. **Units**
   - IfcUnitAssignment
   - Length: METRE or MILLIMETRE
   - Area: SQUARE_METRE
   - Volume: CUBIC_METRE
   - Angle: RADIAN or DEGREE

4. **Coordinate System**
   - IfcAxis2Placement3D (world coordinate system)
   - IfcMapConversion (georeferencing - if applicable)

### 2.4 IFC Element Classification

**Building Elements by Category:**

```
Architecture:
├── IfcWall (standard, curtain, foundation)
├── IfcWallStandardCase (optimized walls)
├── IfcSlab (floor, roof, base slab)
├── IfcRoof
├── IfcStair
├── IfcRailing
├── IfcDoor
├── IfcWindow
├── IfcCovering (ceiling, flooring, cladding)
└── IfcSpace (room definition)

Structure:
├── IfcBeam
├── IfcColumn
├── IfcFooting
├── IfcPile
├── IfcReinforcingBar
├── IfcReinforcingMesh
└── IfcTendon

MEP:
├── IfcDistributionElement (base class)
├── IfcFlowSegment (ducts, pipes)
├── IfcFlowFitting (elbows, tees)
├── IfcFlowTerminal (diffusers, fixtures)
├── IfcEnergyConversionDevice (AHU, chiller)
└── IfcFlowController (dampers, valves)
```

---

## 3. LOD-Specific Data Requirements

### 3.1 LOD 100 - Conceptual

**Geometry:**
- Simple volumes (boxes, cylinders)
- Overall building massing
- No detailed elements

**Properties:**
```json
{
  "lod": 100,
  "required_properties": [
    "gross_floor_area",
    "building_height",
    "occupancy_type",
    "number_of_floors"
  ],
  "geometry_type": "bounding_box",
  "accuracy": "±20%"
}
```

**Use Cases:**
- Feasibility studies
- Conceptual cost estimates
- Site planning

### 3.2 LOD 200 - Approximate Geometry

**Geometry:**
- Generic systems with approximate sizes
- Major building components
- Generic assemblies

**Properties:**
```json
{
  "lod": 200,
  "required_properties": [
    "element_type",
    "material_category",
    "approximate_dimensions",
    "location",
    "generic_system_type"
  ],
  "geometry_type": "simplified_3d",
  "accuracy": "±10%"
}
```

**Example - Wall at LOD 200:**
```
Wall Properties:
- Type: Exterior Wall - Generic
- Thickness: 300mm (approximate)
- Material: Masonry/Concrete (category only)
- Height: Floor to floor
- Fire Rating: Required/Not Required
- Thermal: Insulated/Uninsulated
```

### 3.3 LOD 300 - Precise Geometry

**Geometry:**
- Precise quantities, size, shape, location, orientation
- Specific assemblies and connections
- Accurate dimensions

**Properties:**
```json
{
  "lod": 300,
  "required_properties": [
    "element_type",
    "specific_material",
    "precise_dimensions",
    "structural_properties",
    "thermal_properties",
    "acoustic_properties",
    "fire_rating",
    "manufacturer_generic",
    "cost_code",
    "phasing"
  ],
  "geometry_type": "detailed_3d",
  "accuracy": "±2%"
}
```

**Example - Wall at LOD 300:**
```json
{
  "element_id": "WALL-EXT-001",
  "ifc_type": "IfcWall",
  "lod": 300,
  "assembly": {
    "layers": [
      {"material": "Brick Veneer", "thickness": 100, "unit": "mm"},
      {"material": "Air Gap", "thickness": 25, "unit": "mm"},
      {"material": "Rigid Insulation", "thickness": 75, "unit": "mm", "r_value": 3.5},
      {"material": "Concrete Block", "thickness": 200, "unit": "mm"},
      {"material": "Gypsum Board", "thickness": 12.5, "unit": "mm"}
    ]
  },
  "dimensions": {
    "length": 6.000,
    "height": 3.000,
    "total_thickness": 412.5,
    "unit": "mm"
  },
  "properties": {
    "structural": {
      "load_bearing": true,
      "design_load": "15 kN/m"
    },
    "thermal": {
      "u_value": 0.28,
      "unit": "W/m2K"
    },
    "fire": {
      "rating": "2_hour",
      "standard": "ASTM_E119"
    },
    "acoustic": {
      "stc_rating": 52,
      "standard": "ASTM_E90"
    }
  },
  "cost": {
    "code": "03.2100.10",
    "unit_cost": 185.50,
    "unit": "USD/m2"
  }
}
```

### 3.4 LOD 350 - Construction Coordination

**Geometry:**
- LOD 300 + interfacing elements
- Connection points
- Support systems
- Access and maintenance requirements

**Additional Properties:**
```json
{
  "lod": 350,
  "additional_to_lod300": [
    "connection_details",
    "support_requirements",
    "installation_sequence",
    "clearance_requirements",
    "access_provisions",
    "coordination_status"
  ]
}
```

**MEP Coordination Example:**
```json
{
  "element_id": "DUCT-SUPPLY-L2-01",
  "ifc_type": "IfcDuctSegment",
  "lod": 350,
  "size": {
    "width": 600,
    "height": 300,
    "unit": "mm"
  },
  "connections": [
    {
      "end": "start",
      "connects_to": "AHU-01",
      "connection_type": "flange"
    },
    {
      "end": "end",
      "connects_to": "DUCT-SUPPLY-L2-02",
      "connection_type": "slip_joint"
    }
  ],
  "supports": [
    {
      "location": 1.5,
      "type": "trapeze_hanger",
      "load": 45,
      "unit": "kg"
    }
  ],
  "clearances": {
    "top": 150,
    "bottom": 300,
    "sides": 100,
    "unit": "mm",
    "purpose": "maintenance_access"
  },
  "coordination": {
    "clash_checked": true,
    "last_check_date": "2025-12-20",
    "conflicts_resolved": 3,
    "status": "approved"
  }
}
```

### 3.5 LOD 400 - Fabrication

**Geometry:**
- Fabrication-ready detail
- Shop drawing level
- All connections and fasteners
- Exact dimensions for manufacturing

**Properties:**
```json
{
  "lod": 400,
  "fabrication_data": [
    "exact_dimensions",
    "tolerances",
    "material_grade",
    "surface_finish",
    "fastener_specifications",
    "welding_details",
    "assembly_instructions",
    "part_numbering",
    "manufacturer_specific_data"
  ]
}
```

**Steel Connection Example:**
```json
{
  "element_id": "BEAM-COLUMN-CONNECTION-A23",
  "ifc_type": "IfcMechanicalFastener",
  "lod": 400,
  "connection_type": "bolted_end_plate",
  "components": [
    {
      "part": "end_plate",
      "material": "ASTM_A572_Grade_50",
      "dimensions": {
        "width": 200,
        "height": 300,
        "thickness": 12,
        "unit": "mm"
      },
      "holes": [
        {"diameter": 22, "location": [50, 75]},
        {"diameter": 22, "location": [150, 75]},
        {"diameter": 22, "location": [50, 225]},
        {"diameter": 22, "location": [150, 225]}
      ]
    },
    {
      "part": "bolts",
      "specification": "ASTM_A325",
      "size": "M20",
      "length": 80,
      "quantity": 4,
      "torque": "350 N⋅m"
    }
  ],
  "welding": {
    "type": "fillet_weld",
    "size": "6mm",
    "length": "full_perimeter",
    "process": "FCAW",
    "inspection": "UT_100_percent"
  },
  "fabrication_tolerance": "±1mm",
  "shop_drawing": "SD-STL-025.dwg"
}
```

### 3.6 LOD 500 - As-Built

**Geometry:**
- As-constructed verification
- Field-verified dimensions
- Actual material used

**Properties:**
```json
{
  "lod": 500,
  "as_built_data": [
    "actual_dimensions",
    "actual_materials_used",
    "installation_date",
    "installer_information",
    "test_results",
    "warranty_information",
    "serial_numbers",
    "manufacturer_data",
    "o_and_m_manuals"
  ]
}
```

**As-Built Equipment Example:**
```json
{
  "element_id": "AHU-ROOF-01",
  "ifc_type": "IfcAirToAirHeatRecovery",
  "lod": 500,
  "equipment_data": {
    "manufacturer": "Carrier Corporation",
    "model": "39M-5000",
    "serial_number": "CR25-8847-XY",
    "installation_date": "2025-08-15",
    "installed_by": "ABC Mechanical Inc.",
    "capacity": {
      "airflow": 5000,
      "unit": "CFM"
    }
  },
  "dimensions_actual": {
    "length": 2438,
    "width": 1220,
    "height": 915,
    "unit": "mm",
    "verified_by": "field_measurement",
    "verification_date": "2025-08-16"
  },
  "testing": {
    "commissioning_date": "2025-08-20",
    "test_results": "PASS",
    "report": "CommissioningReport-AHU-ROOF-01.pdf"
  },
  "warranty": {
    "start_date": "2025-08-15",
    "duration_years": 5,
    "coverage": "parts_and_labor",
    "warranty_document": "Warranty-AHU-ROOF-01.pdf"
  },
  "manuals": [
    "AHU-Installation-Manual.pdf",
    "AHU-Operation-Manual.pdf",
    "AHU-Maintenance-Schedule.pdf"
  ],
  "maintenance": {
    "filter_change_interval": "quarterly",
    "annual_service_date": "August",
    "service_contractor": "XYZ Service Co."
  }
}
```

---

## 4. Metadata Schema

### 4.1 Core Metadata (All Elements)

```json
{
  "core_metadata": {
    "element_guid": "3rNg3TA5D0ABT0XbC$IHiC",
    "ifc_type": "string",
    "lod": 100-500,
    "created": {
      "timestamp": "ISO-8601",
      "author": "string",
      "application": "string"
    },
    "modified": {
      "timestamp": "ISO-8601",
      "author": "string",
      "application": "string"
    },
    "discipline": "ARCH|STRUC|MEP|CIVIL",
    "phase": "string",
    "status": "PROPOSED|EXISTING|DEMOLISHED|NEW"
  }
}
```

### 4.2 Geometric Metadata

```json
{
  "geometry": {
    "representation_type": "SweptSolid|Brep|CSG|...",
    "bounding_box": {
      "min": [x, y, z],
      "max": [x, y, z],
      "unit": "m"
    },
    "volume": "number",
    "surface_area": "number",
    "length": "number",
    "coordinate_system": "project|local|survey"
  }
}
```

### 4.3 Material Metadata

```json
{
  "material": {
    "primary_material": "string",
    "material_grade": "string",
    "finish": "string",
    "properties": {
      "density": {"value": "number", "unit": "kg/m3"},
      "thermal_conductivity": {"value": "number", "unit": "W/mK"},
      "embodied_carbon": {"value": "number", "unit": "kgCO2e/kg"}
    },
    "sustainability": {
      "recycled_content": "percentage",
      "recyclable": "boolean",
      "eol_treatment": "string"
    }
  }
}
```

### 4.4 Classification Systems

**Supported Classification Standards:**

1. **Uniformat II** (CSI)
   - A - Substructure
   - B - Shell
   - C - Interiors
   - D - Services
   - E - Equipment
   - F - Special Construction
   - G - Building Sitework

2. **Masterformat** (CSI)
   - Division 00-49

3. **Uniclass 2015** (UK)
4. **OmniClass** (buildingSMART)

**Implementation:**
```json
{
  "classification": [
    {
      "system": "Uniformat_II",
      "code": "B2010.10",
      "title": "Exterior Walls - Load Bearing"
    },
    {
      "system": "Masterformat",
      "code": "04 22 00",
      "title": "Concrete Unit Masonry"
    },
    {
      "system": "OmniClass",
      "code": "23-13 11 11",
      "title": "Concrete Masonry Unit Walls"
    }
  ]
}
```

---

## 5. Property Sets (Psets)

### 5.1 Common Property Sets

**Pset_WallCommon:**
```
- Reference: Reference designation
- IsExternal: Boolean
- LoadBearing: Boolean
- FireRating: Duration in minutes
- AcousticRating: Sound transmission class
- ThermalTransmittance: U-value (W/m²K)
- Combustible: Boolean
```

**Pset_SlabCommon:**
```
- Reference: Reference designation
- IsExternal: Boolean
- LoadBearing: Boolean
- FireRating: Duration
- AcousticRating: Impact sound rating
- ThermalTransmittance: U-value
- PitchAngle: For roof slabs
```

**Pset_DoorCommon:**
```
- Reference: Reference designation
- IsExternal: Boolean
- FireRating: Duration
- SecurityRating: Security classification
- AcousticRating: Sound reduction index
- HandicapAccessible: Boolean
- FireExit: Boolean
- SmokeStop: Boolean
```

### 5.2 MEP Property Sets

**Pset_AirTerminalTypeCommon:**
```
- Shape: Rectangular, Round, Slot
- FlowPattern: Linear, Displacement, Radial
- AirflowRateRange: Min-Max CFM
- PressureDrop: Pa at rated flow
- NominalSize: Neck diameter
- NoiseLevel: dBA
```

**Pset_PipeFittingTypeCommon:**
```
- PipeFittingType: Bend, Tee, Reducer, etc.
- ConnectionType: Threaded, Welded, Flanged, etc.
- PressureClass: Rated pressure
- MaterialGrade: Material specification
```

### 5.3 Custom Property Sets

**WIA-CITY-006 Extended Properties:**

```json
{
  "Pset_WIA_BIM_QA": {
    "ModelValidated": "boolean",
    "ValidationDate": "date",
    "ClashStatus": "NO_CLASH|RESOLVED|PENDING",
    "LODVerified": "boolean",
    "QAApprover": "string",
    "CertificationID": "string"
  },
  "Pset_WIA_Sustainability": {
    "EmbodiedCarbon": "kgCO2e",
    "RecycledContent": "percentage",
    "LocallySourced": "boolean",
    "EPDAvailable": "boolean",
    "EPD_Reference": "url"
  },
  "Pset_WIA_DigitalTwin": {
    "SensorID": "string",
    "IoTPlatform": "string",
    "DataStreamURL": "url",
    "TwinStatus": "ACTIVE|PLANNED|INACTIVE"
  }
}
```

---

## 6. 3D Model Metadata

### 6.1 Point Cloud Data

**Format:** E57 (ASTM E2807)

```json
{
  "point_cloud": {
    "format": "E57",
    "scan_date": "ISO-8601",
    "scanner_model": "string",
    "point_count": "integer",
    "accuracy": {"value": "number", "unit": "mm"},
    "coordinate_system": "project|survey",
    "registration_method": "target_based|cloud_to_cloud",
    "registration_error": {"value": "number", "unit": "mm"},
    "color_data": "boolean",
    "intensity_data": "boolean",
    "scan_positions": [
      {
        "id": "string",
        "location": [x, y, z],
        "orientation": [yaw, pitch, roll]
      }
    ]
  }
}
```

### 6.2 Photogrammetry/Mesh

**Formats:** OBJ, FBX, DAE

```json
{
  "mesh_model": {
    "format": "OBJ",
    "creation_date": "ISO-8601",
    "software": "string",
    "polygon_count": "integer",
    "texture_resolution": "2048x2048",
    "texture_count": "integer",
    "accuracy": {"value": "number", "unit": "mm"},
    "geo_referenced": "boolean",
    "coordinate_system": "string"
  }
}
```

---

## 7. Data Validation

### 7.1 Schema Validation

**IFC Schema Compliance:**
```python
def validate_ifc_schema(ifc_file):
    checks = {
        "iso_10303_21_header": check_iso_header(),
        "file_schema_ifc4": check_schema_version(),
        "required_entities": check_required_entities(),
        "entity_references": check_entity_references(),
        "data_types": validate_data_types(),
        "spatial_hierarchy": validate_spatial_structure()
    }
    return all(checks.values())
```

### 7.2 LOD Validation

**LOD-Specific Checks:**
```python
def validate_lod(element, declared_lod):
    if declared_lod == 300:
        required_properties = [
            'specific_material',
            'precise_dimensions',
            'structural_properties',
            'thermal_properties',
            'fire_rating',
            'cost_code'
        ]
        return all(has_property(element, prop)
                  for prop in required_properties)
```

### 7.3 Data Quality Metrics

```json
{
  "quality_metrics": {
    "completeness": {
      "elements_with_required_properties": "percentage",
      "elements_with_geometry": "percentage",
      "elements_with_material": "percentage"
    },
    "accuracy": {
      "geometric_tolerance": "±mm",
      "duplicate_elements": "count",
      "orphaned_elements": "count"
    },
    "consistency": {
      "unit_consistency": "boolean",
      "naming_convention": "percentage_compliant",
      "classification_assigned": "percentage"
    }
  }
}
```

---

## 8. Data Exchange Protocols

### 8.1 Export Requirements

**Minimum Export Settings:**
```
IFC Version: IFC4
View Definition: Coordination View 2.0
Geometry: Solid models (not surfaces)
Properties: All custom property sets
Spatial Structure: Full hierarchy
Units: SI (metres)
Coordinate System: Shared project coordinates
```

### 8.2 Import Requirements

**Validation on Import:**
1. Schema version compatibility check
2. Spatial structure integrity
3. Unit conversion (if necessary)
4. Coordinate system alignment
5. Duplicate detection
6. Metadata preservation

---

## 9. Appendices

### Appendix A: IFC Entity Reference

Complete list of supported IFC entities - see buildingSMART IFC4 documentation.

### Appendix B: Property Set Templates

Downloadable templates for common property sets in multiple formats.

### Appendix C: Sample Data Files

Reference IFC files demonstrating LOD 100-500 implementations.

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
