# WIA-CITY-006: Building Information Modeling - Phase 4 Integration

**Version:** 1.0
**Status:** Draft
**Category:** CITY
**Last Updated:** 2025-12-25

---

## 1. Overview

This specification defines integration standards for connecting BIM systems with external platforms including 4D scheduling, 5D cost estimation, facility management systems, digital twins, and IoT platforms. It ensures seamless data flow throughout the building lifecycle.

---

## 2. 4D Construction Simulation

### 2.1 Schedule Integration

**Data Mapping:**
```json
{
  "schedule_integration": {
    "bim_element": {
      "guid": "element_guid",
      "ifc_type": "IfcWall",
      "name": "Ground Floor Slab"
    },
    "schedule_task": {
      "task_id": "TASK-001",
      "name": "Pour Ground Floor Slab",
      "wbs": "1.2.3.4",
      "start_date": "2025-02-01",
      "end_date": "2025-02-08",
      "duration": 7,
      "duration_unit": "days",
      "predecessors": ["TASK-000"],
      "successors": ["TASK-002"],
      "resource_requirements": {
        "crew": 5,
        "equipment": ["concrete_pump", "vibrator"],
        "materials": {
          "concrete": {"quantity": 120, "unit": "m3"}
        }
      }
    },
    "visualization": {
      "appearance_mode": "construction_sequence",
      "color_scheme": {
        "not_started": "#808080",
        "in_progress": "#FFA500",
        "completed": "#008000"
      },
      "visibility_rule": "show_when_active"
    }
  }
}
```

**Supported Scheduling Formats:**
- Primavera P6 (XER, XML)
- Microsoft Project (MPP, XML)
- Asta Powerproject (PP)
- Oracle Primavera Cloud

### 2.2 4D Visualization Protocol

**Timeline Animation:**
```javascript
{
  "4d_animation": {
    "project_duration": {
      "start": "2025-01-15",
      "end": "2026-12-31",
      "total_days": 716
    },
    "playback_settings": {
      "time_scale": "1_day_per_second",
      "frame_rate": 30,
      "interpolation": "linear"
    },
    "element_states": [
      {
        "date": "2025-02-01",
        "elements": [
          {
            "guid": "foundation-guid-001",
            "state": "start_construction",
            "visibility": true,
            "color": "#FFA500",
            "opacity": 0.5
          }
        ]
      },
      {
        "date": "2025-02-08",
        "elements": [
          {
            "guid": "foundation-guid-001",
            "state": "complete",
            "visibility": true,
            "color": "#008000",
            "opacity": 1.0
          }
        ]
      }
    ]
  }
}
```

**Constructability Analysis:**
```python
def analyze_constructability(model, schedule):
    """Analyze construction sequence feasibility"""

    issues = []

    # Check spatial conflicts
    for task in schedule.tasks:
        active_tasks = get_simultaneous_tasks(task, schedule)
        for other_task in active_tasks:
            if check_workspace_conflict(task, other_task, model):
                issues.append({
                    "type": "workspace_conflict",
                    "task_a": task.id,
                    "task_b": other_task.id,
                    "severity": "high",
                    "recommendation": "Adjust schedule or work areas"
                })

    # Check logical sequence
    for element in model.elements:
        construction_task = get_task_for_element(element, schedule)
        supporting_elements = get_structural_support(element, model)

        for support in supporting_elements:
            support_task = get_task_for_element(support, schedule)
            if construction_task.start < support_task.end:
                issues.append({
                    "type": "sequence_violation",
                    "element": element.guid,
                    "issue": "Construction before support complete",
                    "severity": "critical"
                })

    # Check resource availability
    for date in schedule.date_range:
        resources = calculate_resource_demand(date, schedule)
        if resources.crew > project.max_crew:
            issues.append({
                "type": "resource_overallocation",
                "date": date,
                "resource": "crew",
                "required": resources.crew,
                "available": project.max_crew
            })

    return {
        "total_issues": len(issues),
        "by_severity": categorize_issues(issues),
        "issues": issues
    }
```

### 2.3 Progress Tracking

**As-Built vs. Planned:**
```json
{
  "progress_tracking": {
    "date": "2025-06-15",
    "overall_progress": {
      "planned": 45.5,
      "actual": 42.3,
      "variance": -3.2,
      "unit": "percent"
    },
    "element_status": [
      {
        "guid": "wall-ext-001",
        "planned_status": "completed",
        "actual_status": "in_progress",
        "planned_completion": "2025-06-10",
        "actual_completion": null,
        "delay_days": 5,
        "reasons": ["weather", "material_delivery"]
      }
    ],
    "critical_path_impact": {
      "on_critical_path": false,
      "float_days": 12,
      "project_delay_risk": "low"
    }
  }
}
```

---

## 3. 5D Cost Estimation

### 3.1 Cost Database Integration

**Quantity Takeoff:**
```json
{
  "quantity_takeoff": {
    "element": {
      "guid": "wall-guid-001",
      "ifc_type": "IfcWall",
      "name": "Exterior Wall Type A"
    },
    "quantities": {
      "area": {
        "net": 18.5,
        "gross": 20.0,
        "unit": "m2"
      },
      "volume": {
        "concrete": 5.4,
        "insulation": 1.35,
        "unit": "m3"
      },
      "length": {
        "perimeter": 12.0,
        "unit": "m"
      }
    },
    "materials": [
      {
        "name": "Concrete Block",
        "quantity": 5.4,
        "unit": "m3",
        "cost_code": "04.22.00.10",
        "unit_cost": 180.00,
        "total_cost": 972.00,
        "currency": "USD"
      },
      {
        "name": "Rigid Insulation",
        "quantity": 13.5,
        "unit": "m2",
        "cost_code": "07.21.13.10",
        "unit_cost": 12.50,
        "total_cost": 168.75,
        "currency": "USD"
      }
    ],
    "labor": {
      "trade": "mason",
      "hours": 24.0,
      "rate": 65.00,
      "total": 1560.00,
      "currency": "USD"
    },
    "equipment": {
      "items": ["scaffolding", "mixer"],
      "cost": 125.00,
      "currency": "USD"
    },
    "summary": {
      "material_cost": 1140.75,
      "labor_cost": 1560.00,
      "equipment_cost": 125.00,
      "subtotal": 2825.75,
      "overhead": 423.86,
      "profit": 324.96,
      "total": 3574.57,
      "currency": "USD"
    }
  }
}
```

**Cost Database Schema:**
```sql
CREATE TABLE cost_items (
    id UUID PRIMARY KEY,
    cost_code VARCHAR(20) NOT NULL,
    description TEXT,
    unit VARCHAR(10),
    unit_cost DECIMAL(10,2),
    currency VARCHAR(3),
    effective_date DATE,
    region VARCHAR(50),
    source VARCHAR(100),
    inflation_index DECIMAL(5,2),
    last_updated TIMESTAMP
);

CREATE TABLE element_costs (
    element_guid UUID PRIMARY KEY,
    model_id UUID,
    total_material_cost DECIMAL(12,2),
    total_labor_cost DECIMAL(12,2),
    total_equipment_cost DECIMAL(12,2),
    overhead_cost DECIMAL(12,2),
    profit DECIMAL(12,2),
    total_cost DECIMAL(12,2),
    currency VARCHAR(3),
    cost_date DATE,
    confidence_level VARCHAR(10)
);
```

### 3.2 Change Order Management

**Cost Impact Analysis:**
```json
{
  "change_order": {
    "id": "CO-023",
    "date": "2025-07-15",
    "description": "Add glass curtain wall on west facade",
    "status": "pending_approval",
    "bim_changes": {
      "elements_added": 45,
      "elements_modified": 12,
      "elements_deleted": 8
    },
    "cost_impact": {
      "baseline": {
        "total_project_cost": 12500000.00
      },
      "revised": {
        "new_work": {
          "material": 85000.00,
          "labor": 52000.00,
          "equipment": 8000.00,
          "subtotal": 145000.00
        },
        "deleted_work": {
          "credit": -18000.00
        },
        "modification_cost": 7500.00,
        "design_fees": 12000.00,
        "contingency": 14650.00,
        "total_change": 161150.00
      },
      "new_total": 12661150.00,
      "percent_change": 1.29,
      "currency": "USD"
    },
    "schedule_impact": {
      "delay_days": 14,
      "critical_path_affected": true,
      "new_completion_date": "2026-12-15"
    },
    "approvals": {
      "architect": "approved",
      "structural_engineer": "approved",
      "owner": "pending",
      "contractor": "approved"
    }
  }
}
```

### 3.3 Value Engineering

**Alternative Analysis:**
```json
{
  "value_engineering": {
    "study_id": "VE-005",
    "component": "Exterior Wall System",
    "baseline": {
      "description": "Brick veneer with concrete block backup",
      "total_cost": 1250000.00,
      "lifecycle_cost": 1450000.00,
      "performance": {
        "thermal": 0.25,
        "fire_rating": 120,
        "maintenance_years": 5
      }
    },
    "alternatives": [
      {
        "option": "A",
        "description": "Metal panel with improved insulation",
        "initial_cost": 980000.00,
        "lifecycle_cost": 1150000.00,
        "savings": 300000.00,
        "performance": {
          "thermal": 0.18,
          "fire_rating": 120,
          "maintenance_years": 10
        },
        "pros": [
          "Lower initial cost",
          "Better thermal performance",
          "Reduced maintenance"
        ],
        "cons": [
          "Different aesthetic",
          "Longer lead time"
        ],
        "recommendation": "approved"
      }
    ]
  }
}
```

---

## 4. Facility Management Integration

### 4.1 COBie Data Exchange

**COBie Worksheet Mapping:**

```json
{
  "cobie_mapping": {
    "facility": {
      "source": "IfcBuilding",
      "fields": {
        "name": "IfcBuilding.Name",
        "category": "IfcBuilding.ObjectType",
        "project_name": "IfcProject.Name",
        "site_name": "IfcSite.Name",
        "linear_units": "IfcUnitAssignment.LengthUnit",
        "area_units": "IfcUnitAssignment.AreaUnit"
      }
    },
    "floor": {
      "source": "IfcBuildingStorey",
      "fields": {
        "name": "IfcBuildingStorey.Name",
        "elevation": "IfcBuildingStorey.Elevation",
        "height": "Pset_BuildingStoreyCommon.GrossHeight"
      }
    },
    "space": {
      "source": "IfcSpace",
      "fields": {
        "name": "IfcSpace.Name",
        "number": "Pset_SpaceCommon.Reference",
        "category": "IfcSpace.ObjectType",
        "floor_name": "ContainedInStructure.RelatingStructure.Name",
        "gross_area": "Qto_SpaceBaseQuantities.GrossFloorArea",
        "net_area": "Qto_SpaceBaseQuantities.NetFloorArea"
      }
    },
    "type": {
      "source": "IfcTypeObject",
      "fields": {
        "name": "IfcTypeObject.Name",
        "category": "IfcTypeObject.ObjectType",
        "manufacturer": "Pset_ManufacturerTypeInformation.Manufacturer",
        "model_number": "Pset_ManufacturerTypeInformation.ModelReference",
        "warranty_duration": "Pset_Warranty.WarrantyPeriod"
      }
    },
    "component": {
      "source": "IfcElement",
      "fields": {
        "name": "IfcElement.Name",
        "type_name": "IfcElement.ObjectType",
        "space": "ContainedInStructure.RelatingStructure.Name",
        "serial_number": "Pset_ManufacturerOccurrence.SerialNumber",
        "installation_date": "Pset_ServiceLife.InstallationDate",
        "warranty_start": "Pset_Warranty.WarrantyStartDate"
      }
    }
  }
}
```

**COBie Export Process:**
```python
def export_to_cobie(ifc_model):
    """Export IFC model to COBie format"""

    cobie_data = {
        "Contact": extract_contacts(ifc_model),
        "Facility": extract_facility_info(ifc_model),
        "Floor": extract_floors(ifc_model),
        "Space": extract_spaces(ifc_model),
        "Zone": extract_zones(ifc_model),
        "Type": extract_type_objects(ifc_model),
        "Component": extract_components(ifc_model),
        "System": extract_systems(ifc_model),
        "Assembly": extract_assemblies(ifc_model),
        "Connection": extract_connections(ifc_model),
        "Spare": extract_spare_parts(ifc_model),
        "Resource": extract_resources(ifc_model),
        "Job": extract_job_plans(ifc_model),
        "Impact": extract_impact_data(ifc_model),
        "Document": extract_documents(ifc_model),
        "Attribute": extract_attributes(ifc_model),
        "Coordinate": extract_coordinates(ifc_model),
        "Issue": extract_issues(ifc_model)
    }

    # Validate COBie data
    validation_results = validate_cobie_data(cobie_data)

    if validation_results.valid:
        # Export to Excel
        export_cobie_xlsx(cobie_data, "project_cobie.xlsx")
        # Export to XML
        export_cobie_xml(cobie_data, "project_cobie.xml")
        # Export to JSON
        export_cobie_json(cobie_data, "project_cobie.json")

    return validation_results
```

### 4.2 CMMS Integration

**Maintenance Schedule:**
```json
{
  "maintenance_integration": {
    "equipment": {
      "id": "AHU-ROOF-01",
      "guid": "ahu-guid-001",
      "name": "Rooftop Air Handling Unit",
      "location": {
        "building": "Tower A",
        "floor": "Roof",
        "space": "Mechanical Penthouse",
        "coordinates": [125.5, 45.3, 52.1]
      },
      "manufacturer": "Carrier",
      "model": "39M-5000",
      "serial": "CR25-8847-XY",
      "install_date": "2025-08-15",
      "warranty_end": "2030-08-15"
    },
    "maintenance_tasks": [
      {
        "task_id": "PM-AHU-001-01",
        "description": "Replace air filters",
        "frequency": "quarterly",
        "duration_hours": 2,
        "trade": "HVAC technician",
        "parts": [
          {
            "part_number": "FILTER-24X24-MERV13",
            "quantity": 4,
            "cost": 85.00
          }
        ],
        "next_due": "2025-12-30",
        "criticality": "high"
      },
      {
        "task_id": "PM-AHU-001-02",
        "description": "Inspect and lubricate fan bearings",
        "frequency": "semi_annual",
        "duration_hours": 3,
        "trade": "HVAC technician",
        "next_due": "2026-02-15",
        "criticality": "medium"
      },
      {
        "task_id": "PM-AHU-001-03",
        "description": "Full system commissioning check",
        "frequency": "annual",
        "duration_hours": 8,
        "trade": "certified_technician",
        "next_due": "2026-08-15",
        "criticality": "high"
      }
    ],
    "work_orders": [
      {
        "wo_number": "WO-2025-1234",
        "type": "corrective",
        "priority": "urgent",
        "created_date": "2025-12-20",
        "description": "Unit making unusual noise",
        "status": "in_progress",
        "assigned_to": "John Smith",
        "estimated_completion": "2025-12-22"
      }
    ]
  }
}
```

**CMMS API Integration:**
```javascript
// POST equipment data to CMMS
POST /api/v1/assets
{
  "asset_tag": "AHU-ROOF-01",
  "name": "Rooftop Air Handling Unit",
  "category": "HVAC Equipment",
  "location_id": "BLDG-A-ROOF-MECH",
  "manufacturer": "Carrier",
  "model": "39M-5000",
  "serial_number": "CR25-8847-XY",
  "install_date": "2025-08-15",
  "warranty_expiry": "2030-08-15",
  "criticality": "high",
  "custom_fields": {
    "bim_guid": "ahu-guid-001",
    "ifc_type": "IfcAirToAirHeatRecovery",
    "bim_model_link": "https://bim.server.com/viewer?element=ahu-guid-001"
  }
}

// GET maintenance schedule
GET /api/v1/assets/AHU-ROOF-01/maintenance-schedule

// POST work order
POST /api/v1/work-orders
{
  "asset_id": "AHU-ROOF-01",
  "type": "preventive_maintenance",
  "description": "Quarterly filter replacement",
  "scheduled_date": "2025-12-30",
  "priority": "normal",
  "estimated_hours": 2
}
```

### 4.3 Space Management

**Space Utilization Tracking:**
```json
{
  "space_management": {
    "space": {
      "guid": "space-guid-301",
      "number": "301",
      "name": "Conference Room A",
      "floor": "Level 3",
      "gross_area": 45.5,
      "net_area": 42.0,
      "unit": "m2",
      "category": "Meeting",
      "occupancy_type": "Assembly",
      "max_occupancy": 20
    },
    "allocation": {
      "department": "Executive",
      "cost_center": "CC-1000",
      "allocated_area": 42.0,
      "rate_per_sqm": 350.00,
      "annual_cost": 14700.00,
      "currency": "USD"
    },
    "utilization": {
      "booking_system_id": "CONF-A-301",
      "average_occupancy": 65.5,
      "peak_hours": ["09:00-11:00", "14:00-16:00"],
      "utilization_rate": 78.2,
      "unit": "percent"
    },
    "furnishings": [
      {
        "item": "Conference Table",
        "quantity": 1,
        "asset_tag": "FURN-2025-0234"
      },
      {
        "item": "Office Chairs",
        "quantity": 20,
        "asset_tags": ["FURN-2025-0235", "..."]
      }
    ],
    "technology": [
      {
        "item": "Video Conferencing System",
        "asset_tag": "AV-2025-0156",
        "warranty_end": "2028-06-30"
      },
      {
        "item": "Projection Screen",
        "asset_tag": "AV-2025-0157"
      }
    ]
  }
}
```

---

## 5. Digital Twin Integration

### 5.1 Digital Twin Architecture

**Twin Components:**
```
Digital Twin Platform
├── Physical Layer (Real Building)
│   ├── IoT Sensors
│   ├── BAS (Building Automation System)
│   ├── Access Control
│   └── Energy Meters
├── Data Layer
│   ├── Time-series data (sensor readings)
│   ├── Event streams (alarms, occupancy)
│   ├── Static data (BIM, documents)
│   └── Analytical results
├── Model Layer
│   ├── BIM Geometry (from WIA-CITY-006)
│   ├── System Models (HVAC, electrical, etc.)
│   ├── Analytical Models (energy, CFD)
│   └── Simulation Models
└── Application Layer
    ├── Visualization
    ├── Analytics
    ├── Optimization
    └── Predictive Maintenance
```

### 5.2 IoT Sensor Mapping

**Sensor-to-BIM Linkage:**
```json
{
  "sensor_mapping": {
    "sensor": {
      "id": "TEMP-L3-301-01",
      "type": "temperature",
      "protocol": "BACnet",
      "address": "192.168.1.100:47808",
      "object_type": "analog_input",
      "object_instance": 1
    },
    "bim_element": {
      "space_guid": "space-guid-301",
      "space_name": "Conference Room A",
      "floor": "Level 3",
      "location": {
        "x": 125.5,
        "y": 45.3,
        "z": 10.5,
        "unit": "m"
      }
    },
    "data_stream": {
      "endpoint": "https://iot.platform.com/api/sensors/TEMP-L3-301-01",
      "protocol": "MQTT",
      "topic": "building/tower-a/level-3/space-301/temperature",
      "sample_rate": "1_minute",
      "retention": "90_days"
    },
    "thresholds": {
      "min": 20.0,
      "max": 24.0,
      "unit": "celsius",
      "alarm_on_violation": true
    }
  }
}
```

**Real-Time Data Integration:**
```python
import asyncio
import aiohttp
from datetime import datetime

class DigitalTwinIntegration:
    def __init__(self, bim_model, iot_platform):
        self.bim_model = bim_model
        self.iot_platform = iot_platform
        self.sensor_map = self.create_sensor_mapping()

    async def stream_sensor_data(self):
        """Stream real-time sensor data to digital twin"""

        async with aiohttp.ClientSession() as session:
            for sensor_id, bim_element in self.sensor_map.items():
                # Subscribe to sensor data stream
                async with session.ws_connect(
                    f'wss://iot.platform.com/sensors/{sensor_id}/stream'
                ) as ws:
                    async for msg in ws:
                        data = json.loads(msg.data)

                        # Update digital twin
                        await self.update_twin_state(
                            element_guid=bim_element['guid'],
                            sensor_type=data['type'],
                            value=data['value'],
                            timestamp=data['timestamp']
                        )

                        # Check thresholds
                        if self.check_threshold_violation(data):
                            await self.trigger_alarm(data)

    async def update_twin_state(self, element_guid, sensor_type, value, timestamp):
        """Update digital twin with sensor reading"""

        twin_update = {
            "element_guid": element_guid,
            "property": f"realtime_{sensor_type}",
            "value": value,
            "timestamp": timestamp,
            "source": "iot_sensor"
        }

        await self.iot_platform.post('/api/twin/update', json=twin_update)

    def run_analytics(self):
        """Run analytics on digital twin data"""

        # Energy consumption analysis
        energy_data = self.get_energy_consumption()
        energy_insights = self.analyze_energy_patterns(energy_data)

        # Occupancy analysis
        occupancy_data = self.get_occupancy_data()
        space_utilization = self.analyze_space_usage(occupancy_data)

        # Predictive maintenance
        equipment_data = self.get_equipment_performance()
        maintenance_predictions = self.predict_failures(equipment_data)

        return {
            "energy": energy_insights,
            "occupancy": space_utilization,
            "maintenance": maintenance_predictions
        }
```

### 5.3 Simulation and Optimization

**Energy Optimization:**
```json
{
  "energy_optimization": {
    "baseline": {
      "annual_consumption": 2850000,
      "unit": "kWh",
      "cost": 285000.00,
      "carbon": 1425,
      "carbon_unit": "tCO2e"
    },
    "digital_twin_analysis": {
      "inefficiencies_detected": [
        {
          "system": "HVAC-Zone-East",
          "issue": "Overcooling during unoccupied hours",
          "waste": "12,500 kWh/year",
          "cost_impact": 1250.00
        },
        {
          "system": "Lighting-Level-3",
          "issue": "Lights on during daylight hours",
          "waste": "8,200 kWh/year",
          "cost_impact": 820.00
        }
      ]
    },
    "optimization_strategy": {
      "actions": [
        {
          "action": "Implement occupancy-based HVAC scheduling",
          "estimated_savings": "45,000 kWh/year",
          "cost_savings": 4500.00,
          "implementation_cost": 8000.00,
          "payback_period": "1.8 years"
        },
        {
          "action": "Daylight harvesting for lighting zones",
          "estimated_savings": "32,000 kWh/year",
          "cost_savings": 3200.00,
          "implementation_cost": 12000.00,
          "payback_period": "3.8 years"
        }
      ],
      "total_savings": {
        "annual_kwh": 77000,
        "annual_cost": 7700.00,
        "carbon_reduction": 38.5,
        "carbon_unit": "tCO2e"
      }
    }
  }
}
```

---

## 6. GIS Integration

### 6.1 Georeferencing

**Coordinate Systems:**
```json
{
  "georeferencing": {
    "project_crs": {
      "name": "WGS 84 / UTM zone 52N",
      "epsg_code": "EPSG:32652",
      "unit": "metre"
    },
    "bim_origin": {
      "project_base_point": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "unit": "m"
      },
      "survey_point": {
        "easting": 345678.50,
        "northing": 4567890.25,
        "elevation": 125.30,
        "crs": "EPSG:32652"
      }
    },
    "transformation": {
      "translation": [345678.50, 4567890.25, 125.30],
      "rotation": 15.5,
      "rotation_unit": "degrees",
      "scale": 1.0
    }
  }
}
```

**GIS Data Exchange:**
```python
def export_to_gis(bim_model, output_format='shapefile'):
    """Export BIM elements to GIS format"""

    # Extract building footprint
    footprint = extract_building_footprint(bim_model)
    footprint_geom = convert_to_polygon(footprint)

    # Extract building levels
    levels = []
    for storey in bim_model.building_storeys:
        level_footprint = extract_storey_footprint(storey)
        levels.append({
            'geometry': convert_to_polygon(level_footprint),
            'properties': {
                'level_number': storey.name,
                'elevation': storey.elevation,
                'height': storey.height,
                'gross_area': storey.gross_area,
                'building_name': bim_model.building.name
            }
        })

    # Extract individual spaces
    spaces = []
    for space in bim_model.spaces:
        space_boundary = extract_space_boundary(space)
        spaces.append({
            'geometry': convert_to_polygon(space_boundary),
            'properties': {
                'space_number': space.number,
                'space_name': space.name,
                'floor_area': space.floor_area,
                'occupancy_type': space.occupancy_type,
                'department': space.department
            }
        })

    # Export to format
    if output_format == 'shapefile':
        export_shapefile(spaces, 'building_spaces.shp')
    elif output_format == 'geojson':
        export_geojson(spaces, 'building_spaces.geojson')
    elif output_format == 'kml':
        export_kml(spaces, 'building_spaces.kml')

    return {
        'footprint': footprint_geom,
        'levels': levels,
        'spaces': spaces
    }
```

### 6.2 Urban Context Integration

**City-Scale BIM:**
```json
{
  "urban_integration": {
    "building": {
      "id": "BLDG-TOWER-A",
      "name": "City Tower A",
      "address": "123 Main Street",
      "city": "Metropolis",
      "coordinates": {
        "latitude": 37.5665,
        "longitude": 126.9780
      }
    },
    "context": {
      "district": "Central Business District",
      "zoning": "Commercial High-Rise",
      "land_use": "Mixed Use",
      "adjacent_buildings": [
        {
          "id": "BLDG-TOWER-B",
          "distance": 25.5,
          "height": 180.0,
          "shadow_impact": "moderate"
        }
      ]
    },
    "infrastructure": {
      "roads": [
        {
          "name": "Main Street",
          "type": "arterial",
          "distance_to_building": 15.0
        }
      ],
      "transit": [
        {
          "type": "metro_station",
          "name": "City Center Station",
          "distance": 250.0
        }
      ],
      "utilities": {
        "power_substation": "SS-CENTRAL-01",
        "water_main": "WM-MAIN-12",
        "sewer_line": "SL-TRUNK-05"
      }
    },
    "environmental": {
      "solar_exposure": "high",
      "wind_exposure": "moderate_high",
      "flood_zone": "no",
      "seismic_zone": "moderate"
    }
  }
}
```

---

## 7. Sustainability Analytics

### 7.1 Energy Modeling Integration

**Energy Analysis:**
```json
{
  "energy_model": {
    "software": "IES-VE",
    "standard": "ASHRAE_90.1_2019",
    "climate_zone": "4A",
    "building_data": {
      "geometry_source": "bim_model_guid",
      "conditioned_area": 12500.0,
      "envelope_area": 8200.0,
      "window_to_wall_ratio": 0.35,
      "unit": "m2"
    },
    "systems": {
      "hvac": {
        "type": "VAV with reheat",
        "efficiency": "ASHRAE_90.1_baseline",
        "ventilation": "demand_controlled"
      },
      "lighting": {
        "power_density": 9.5,
        "unit": "W/m2",
        "controls": "occupancy_daylight"
      }
    },
    "results": {
      "eui": {
        "total": 185.5,
        "heating": 45.2,
        "cooling": 62.3,
        "lighting": 38.5,
        "equipment": 39.5,
        "unit": "kWh/m2/year"
      },
      "performance": {
        "baseline_eui": 228.0,
        "percent_savings": 18.6,
        "leed_points": 8
      }
    }
  }
}
```

### 7.2 Carbon Footprint

**Embodied Carbon:**
```json
{
  "embodied_carbon": {
    "total": 2850.5,
    "unit": "tCO2e",
    "per_sqm": 228.0,
    "breakdown": [
      {
        "category": "Structure",
        "material": "Concrete",
        "quantity": 2500.0,
        "unit": "m3",
        "carbon_factor": 0.350,
        "total_carbon": 875.0
      },
      {
        "category": "Structure",
        "material": "Structural Steel",
        "quantity": 450.0,
        "unit": "tonnes",
        "carbon_factor": 2.100,
        "total_carbon": 945.0
      },
      {
        "category": "Envelope",
        "material": "Glass Curtain Wall",
        "quantity": 2800.0,
        "unit": "m2",
        "carbon_factor": 0.085,
        "total_carbon": 238.0
      }
    ],
    "reduction_strategies": [
      {
        "strategy": "Use high-flyash concrete",
        "carbon_savings": 175.0,
        "cost_impact": -12000.00
      },
      {
        "strategy": "Optimize structural design",
        "carbon_savings": 125.0,
        "cost_impact": 0.00
      }
    ]
  }
}
```

---

## 8. API Standards

### 8.1 RESTful API Design

**Endpoint Structure:**
```
Base URL: https://api.wia-city-006.org/v1

Authentication:
  Bearer token (OAuth 2.0)

Endpoints:

Projects:
  GET    /projects
  GET    /projects/{project_id}
  POST   /projects
  PUT    /projects/{project_id}
  DELETE /projects/{project_id}

Models:
  GET    /projects/{project_id}/models
  GET    /models/{model_id}
  POST   /models
  GET    /models/{model_id}/download

Elements:
  GET    /models/{model_id}/elements
  GET    /elements/{element_guid}
  GET    /elements/{element_guid}/properties
  PUT    /elements/{element_guid}/properties

Integration:
  POST   /models/{model_id}/export/cobie
  POST   /models/{model_id}/export/gis
  GET    /elements/{element_guid}/sensors
  GET    /elements/{element_guid}/maintenance

Digital Twin:
  GET    /twin/{building_id}/state
  POST   /twin/{building_id}/update
  GET    /twin/{building_id}/analytics
```

**Example Request:**
```http
GET /api/v1/models/abc123/elements?type=IfcWall&lod=300 HTTP/1.1
Host: api.wia-city-006.org
Authorization: Bearer eyJhbGc...
Accept: application/json
```

**Example Response:**
```json
{
  "status": "success",
  "data": {
    "total_count": 245,
    "page": 1,
    "page_size": 50,
    "elements": [
      {
        "guid": "3rNg3TA5D0ABT0XbC$IHiC",
        "ifc_type": "IfcWall",
        "name": "Exterior Wall - West",
        "lod": 300,
        "links": {
          "self": "/elements/3rNg3TA5D0ABT0XbC$IHiC",
          "properties": "/elements/3rNg3TA5D0ABT0XbC$IHiC/properties",
          "sensors": "/elements/3rNg3TA5D0ABT0XbC$IHiC/sensors"
        }
      }
    ]
  }
}
```

---

## 9. Appendices

### Appendix A: Integration Checklists

Complete checklists for each integration type (4D, 5D, FM, Digital Twin).

### Appendix B: API Documentation

Full API reference with examples and SDKs.

### Appendix C: Sample Data Files

Reference files demonstrating all integration formats.

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
