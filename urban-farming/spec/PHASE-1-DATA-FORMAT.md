# WIA-AGRI-031: Urban Farming Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines the data formats for urban farming operations, including rooftop gardens, community gardens, container farming, food forests, and city-integrated agriculture systems.

### 1.1 Design Principles

- **Accessibility**: Simple enough for community gardeners, comprehensive for city planners
- **Scalability**: Support from balcony gardens to multi-acre urban farms
- **Community-Focused**: Track social impact, engagement, and educational outcomes
- **Sustainability**: Monitor resource efficiency and environmental benefits
- **Integration**: Compatible with city planning, food supply, and educational systems

---

## 2. Core Data Structures

### 2.1 Urban Farm Configuration

Basic information about the urban farming site.

```json
{
  "farmConfig": {
    "farmId": "UF-NYC-001",
    "farmName": "Brooklyn Community Garden",
    "farmType": "COMMUNITY",
    "location": {
      "address": "123 Brooklyn Ave, Brooklyn, NY 11201",
      "latitude": 40.6782,
      "longitude": -73.9442,
      "timezone": "America/New_York",
      "neighborhood": "Brooklyn Heights",
      "urbanZone": "RESIDENTIAL"
    },
    "infrastructure": {
      "totalArea": 150,
      "usableArea": 135,
      "numberOfPlots": 12,
      "sharedSpaces": ["tool_shed", "composting_area", "gathering_space"],
      "waterSource": "MUNICIPAL",
      "irrigationType": "DRIP",
      "sunExposure": "FULL_SUN",
      "soilType": "RAISED_BED"
    },
    "features": {
      "composting": true,
      "rainwaterHarvesting": true,
      "solarPower": false,
      "greenhouseStructure": false,
      "accessibleDesign": true,
      "childrenArea": true
    },
    "ownership": {
      "type": "COMMUNITY_COOPERATIVE",
      "landlord": "NYC Parks Department",
      "leaseExpiration": "2030-12-31",
      "managementModel": "MEMBER_RUN"
    },
    "operationalSince": "2020-04-15",
    "certifications": ["ORGANIC", "COMMUNITY_SUPPORTED", "PESTICIDE_FREE"]
  }
}
```

**Farm Types:**
- `ROOFTOP`: Rooftop gardens on buildings
- `COMMUNITY`: Community-managed gardens with multiple members
- `CONTAINER`: Container-based farming in urban spaces
- `FOOD_FOREST`: Perennial edible landscapes
- `BALCONY`: Private balcony/terrace gardens
- `VERTICAL_WALL`: Living walls and vertical gardens
- `SCHOOLYARD`: Educational gardens in schools

**Water Sources:**
- `MUNICIPAL`: City water supply
- `RAINWATER`: Harvested rainwater
- `GREYWATER`: Recycled household water
- `WELL`: On-site well water
- `HYBRID`: Multiple sources

### 2.2 Plot Assignment Data

Individual plot allocation and management.

```json
{
  "plotData": {
    "plotId": "UF-NYC-001-P07",
    "farmId": "UF-NYC-001",
    "plotNumber": 7,
    "dimensions": {
      "length": 4.0,
      "width": 3.0,
      "area": 12.0,
      "unit": "meters"
    },
    "gardener": {
      "gardenerId": "G-2345",
      "name": "Jane Smith",
      "contactEmail": "jane@example.com",
      "memberSince": "2022-03-15",
      "membershipType": "FULL_MEMBER",
      "volunteerHours": 45
    },
    "currentCrops": [
      {
        "cropId": "C-001",
        "cropName": "Tomatoes",
        "variety": "Heirloom Beefsteak",
        "plantedDate": "2025-05-01",
        "expectedHarvest": "2025-07-15",
        "plantCount": 6,
        "growingMethod": "RAISED_BED"
      },
      {
        "cropId": "C-002",
        "cropName": "Basil",
        "variety": "Sweet Basil",
        "plantedDate": "2025-05-15",
        "expectedHarvest": "2025-06-30",
        "plantCount": 12,
        "growingMethod": "RAISED_BED"
      }
    ],
    "soilHealth": {
      "lastTested": "2025-04-01",
      "ph": 6.8,
      "organicMatter": "HIGH",
      "nutrients": {
        "nitrogen": "MEDIUM",
        "phosphorus": "HIGH",
        "potassium": "MEDIUM"
      }
    },
    "status": "ACTIVE",
    "notes": "Companion planting tomatoes with basil"
  }
}
```

### 2.3 Harvest Record Data

Tracking crop yields and production.

```json
{
  "harvestRecord": {
    "harvestId": "HRV-2025-001234",
    "farmId": "UF-NYC-001",
    "plotId": "UF-NYC-001-P07",
    "gardenerId": "G-2345",
    "harvestDate": "2025-07-20",
    "crop": {
      "cropId": "C-001",
      "cropName": "Tomatoes",
      "variety": "Heirloom Beefsteak"
    },
    "yield": {
      "quantity": 12.5,
      "unit": "kg",
      "quality": "GRADE_A",
      "estimatedValue": 75.00,
      "currency": "USD"
    },
    "distribution": {
      "personalUse": 8.0,
      "donated": 2.5,
      "sold": 2.0,
      "shared": 0
    },
    "growingPeriod": {
      "plantedDate": "2025-05-01",
      "harvestedDate": "2025-07-20",
      "daysToHarvest": 80
    },
    "timestamp": "2025-07-20T14:30:00Z"
  }
}
```

### 2.4 Environmental Monitoring Data

Microclimate and environmental conditions.

```json
{
  "environmentData": {
    "farmId": "UF-NYC-001",
    "sensorId": "SENSOR-001",
    "location": "PLOT_AREA_A",
    "timestamp": "2025-06-15T12:00:00Z",
    "weather": {
      "temperature": {
        "value": 24.5,
        "unit": "celsius",
        "ambient": 28.0
      },
      "humidity": {
        "value": 62,
        "unit": "percent"
      },
      "windSpeed": {
        "value": 12,
        "unit": "km/h",
        "direction": "SW"
      },
      "precipitation": {
        "last24h": 2.5,
        "unit": "mm"
      },
      "uvIndex": 7
    },
    "soil": {
      "moisture": {
        "value": 68,
        "unit": "percent",
        "depth": 15,
        "status": "OPTIMAL"
      },
      "temperature": {
        "value": 22.0,
        "unit": "celsius"
      }
    },
    "airQuality": {
      "pm25": 12,
      "pm10": 18,
      "quality": "GOOD"
    }
  }
}
```

### 2.5 Water Management Data

Irrigation and water usage tracking.

```json
{
  "waterData": {
    "farmId": "UF-NYC-001",
    "date": "2025-06-15",
    "waterSource": {
      "municipal": {
        "volume": 120,
        "unit": "liters",
        "cost": 0.15
      },
      "rainwater": {
        "volume": 45,
        "unit": "liters",
        "tankLevel": 65
      }
    },
    "distribution": {
      "irrigation": 140,
      "washing": 15,
      "composting": 10
    },
    "efficiency": {
      "totalUsed": 165,
      "perSqMeter": 1.22,
      "savedVsConventional": 45,
      "unit": "liters"
    },
    "rainwaterHarvesting": {
      "tankCapacity": 500,
      "currentLevel": 325,
      "lastRefill": "2025-06-10"
    }
  }
}
```

### 2.6 Community Engagement Data

Member participation and social impact.

```json
{
  "communityData": {
    "farmId": "UF-NYC-001",
    "period": "2025-06",
    "membership": {
      "totalMembers": 24,
      "activeGardeners": 18,
      "waitlist": 15,
      "newMembers": 2,
      "demographics": {
        "ageGroups": {
          "under18": 3,
          "18to35": 8,
          "36to55": 9,
          "over55": 4
        }
      }
    },
    "participation": {
      "totalVolunteerHours": 320,
      "workdayAttendance": 22,
      "eventsHeld": 4,
      "eventAttendance": 85
    },
    "education": {
      "workshopsHeld": 3,
      "workshopTopics": ["Composting", "Seed Saving", "Companion Planting"],
      "participantsReached": 45,
      "schoolVisits": 2
    },
    "socialImpact": {
      "foodDonated": 78.5,
      "mealsProvided": 260,
      "foodInsecureServed": 35,
      "communityEvents": 6
    }
  }
}
```

### 2.7 Sustainability Metrics

Environmental impact tracking.

```json
{
  "sustainabilityMetrics": {
    "farmId": "UF-NYC-001",
    "period": "2025-Q2",
    "carbonFootprint": {
      "sequestered": 145.5,
      "saved": 89.2,
      "net": 234.7,
      "unit": "kg_co2"
    },
    "resourceEfficiency": {
      "waterSaved": 4200,
      "wasteComposted": 185,
      "organicMatterReturned": 95
    },
    "biodiversity": {
      "plantSpecies": 24,
      "pollinatorSpecies": 8,
      "beneficialInsects": ["ladybugs", "lacewings", "bees"],
      "nativePlants": 12
    },
    "urbanBenefits": {
      "surfaceTemperatureReduction": 3.5,
      "stormwaterManaged": 2300,
      "airQualityImprovement": "MODERATE",
      "greenSpaceCreated": 135
    },
    "foodMiles": {
      "averageDistance": 0.5,
      "savedVsConventional": 2400,
      "unit": "km"
    }
  }
}
```

---

## 3. Enumeration Types

### 3.1 Farm Type Enums

```typescript
enum FarmType {
  ROOFTOP = "ROOFTOP",
  COMMUNITY = "COMMUNITY",
  CONTAINER = "CONTAINER",
  FOOD_FOREST = "FOOD_FOREST",
  BALCONY = "BALCONY",
  VERTICAL_WALL = "VERTICAL_WALL",
  SCHOOLYARD = "SCHOOLYARD"
}

enum WaterSource {
  MUNICIPAL = "MUNICIPAL",
  RAINWATER = "RAINWATER",
  GREYWATER = "GREYWATER",
  WELL = "WELL",
  HYBRID = "HYBRID"
}

enum IrrigationType {
  DRIP = "DRIP",
  SPRINKLER = "SPRINKLER",
  MANUAL = "MANUAL",
  SOAKER_HOSE = "SOAKER_HOSE",
  OLLAS = "OLLAS"
}

enum SoilType {
  RAISED_BED = "RAISED_BED",
  IN_GROUND = "IN_GROUND",
  CONTAINER = "CONTAINER",
  HYDROPONICS = "HYDROPONICS",
  AQUAPONICS = "AQUAPONICS"
}

enum GrowingSeason {
  YEAR_ROUND = "YEAR_ROUND",
  SPRING_SUMMER = "SPRING_SUMMER",
  FALL_WINTER = "FALL_WINTER",
  SEASONAL = "SEASONAL"
}

enum MembershipType {
  FULL_MEMBER = "FULL_MEMBER",
  ASSOCIATE = "ASSOCIATE",
  VOLUNTEER = "VOLUNTEER",
  STUDENT = "STUDENT",
  SENIOR = "SENIOR"
}
```

---

## 4. Data Validation Rules

### 4.1 Required Fields

All data structures must include:
- Unique identifiers (farmId, plotId, etc.)
- Timestamps in ISO 8601 format
- Location data (coordinates or address)
- Status indicators

### 4.2 Value Constraints

- Coordinates: Valid latitude (-90 to 90) and longitude (-180 to 180)
- Area measurements: Positive numbers, minimum 1 square meter
- Dates: ISO 8601 format (YYYY-MM-DD or full timestamp)
- Quantities: Non-negative numbers with appropriate units
- Email addresses: RFC 5322 compliant
- pH values: 0.0 to 14.0

### 4.3 Data Integrity

- Harvest dates must be after planting dates
- Water usage cannot exceed source capacity
- Plot areas must not exceed farm total area
- Member counts must be non-negative integers

---

## 5. Units and Standards

### 5.1 Measurement Units

- **Area**: Square meters (m²)
- **Volume**: Liters (L) for water, cubic meters (m³) for bulk
- **Weight**: Kilograms (kg)
- **Distance**: Meters (m), kilometers (km)
- **Temperature**: Celsius (°C)
- **Time**: ISO 8601 format

### 5.2 Currency

- Primary: USD (United States Dollar)
- Support for local currencies with ISO 4217 codes
- Include exchange rates when applicable

---

## 6. Sample Complete Dataset

```json
{
  "urbanFarmData": {
    "farmConfig": {
      "farmId": "UF-NYC-001",
      "farmName": "Brooklyn Community Garden",
      "farmType": "COMMUNITY",
      "location": {
        "address": "123 Brooklyn Ave, Brooklyn, NY 11201",
        "latitude": 40.6782,
        "longitude": -73.9442,
        "timezone": "America/New_York"
      },
      "totalArea": 150,
      "numberOfPlots": 12,
      "features": {
        "composting": true,
        "rainwaterHarvesting": true,
        "solarPower": false
      }
    },
    "plots": [
      {
        "plotId": "UF-NYC-001-P01",
        "area": 12.0,
        "gardener": {
          "gardenerId": "G-2345",
          "name": "Jane Smith"
        },
        "currentCrops": [
          {
            "cropName": "Tomatoes",
            "variety": "Heirloom",
            "plantedDate": "2025-05-01"
          }
        ]
      }
    ],
    "harvests": [
      {
        "harvestId": "HRV-001",
        "plotId": "UF-NYC-001-P01",
        "crop": "Tomatoes",
        "quantity": 12.5,
        "unit": "kg",
        "harvestDate": "2025-07-20"
      }
    ],
    "communityMetrics": {
      "totalMembers": 24,
      "volunteerHours": 320,
      "foodDonated": 78.5
    }
  }
}
```

---

## 7. Version History

- **v1.0.0** (2025-01-01): Initial release

---

**Next Phase:** [PHASE-2-API-INTERFACE.md](PHASE-2-API-INTERFACE.md)

© 2025 WIA - World Certification Industry Association
弘益人間 · Benefit All Humanity
