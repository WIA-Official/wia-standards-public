# WIA Pet Welfare Global Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12-18
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #F59E0B (Amber)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Data Schema](#data-schema)
5. [Field Specifications](#field-specifications)
6. [Welfare Metrics](#welfare-metrics)
7. [Validation Rules](#validation-rules)
8. [Examples](#examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Pet Welfare Global Data Format Standard defines a unified JSON-based format for recording, transmitting, and managing international pet welfare data, animal health records, cross-border adoption tracking, shelter operations, and animal protection enforcement across borders, NGOs, governments, and welfare organizations worldwide.

**Core Objectives**:
- Standardize animal welfare metrics across all countries and organizations
- Enable interoperability between shelters, rescue organizations, and government agencies
- Ensure data integrity throughout the animal lifecycle from birth to adoption
- Support evidence-based animal welfare policy development
- Facilitate cross-border adoption and animal transport compliance
- Track and prevent animal abuse through unified reporting systems
- Coordinate international wildlife and exotic pet regulations

### 1.2 Scope

This standard covers the following data domains:

| Domain | Description |
|--------|-------------|
| Animal Identity | Microchip, biometric, and registration data |
| Welfare Metrics | Health, behavioral, and environmental welfare scoring |
| Shelter Operations | Intake, housing, medical care, and adoption records |
| Cross-Border Movement | International adoption, transport, and quarantine |
| Health Passport | Vaccination, medical history, and health certificates |
| Abuse Reporting | Incident tracking, investigation, and enforcement |
| Wildlife Regulation | Exotic species, CITES compliance, and conservation |
| Organization Data | Shelter, rescue, NGO, and government agency profiles |

### 1.3 Design Principles

1. **Animal-Centric**: Prioritize animal welfare and protection in all data structures
2. **Global Interoperability**: Compatible with international standards (ISO, IATA, CITES)
3. **Privacy & Security**: Protect sensitive data while enabling necessary sharing
4. **Real-time Tracking**: Support live monitoring of animal welfare conditions
5. **Evidence-Based**: Enable data-driven policy and welfare improvements
6. **Multi-Lingual**: Support international language and cultural requirements

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Animal Subject** | Individual animal tracked in the system |
| **Welfare Score** | Quantitative assessment of animal wellbeing (0-100 scale) |
| **Shelter Organization** | Facility providing temporary care for animals |
| **Cross-Border Adoption** | International transfer of animal ownership |
| **Health Passport** | Complete medical and vaccination record |
| **Abuse Incident** | Documented case of animal mistreatment |
| **CITES Permit** | Convention on International Trade in Endangered Species authorization |
| **IATA Compliance** | International Air Transport Association animal transport standards |
| **Quarantine Period** | Mandatory isolation period for disease prevention |
| **Microchip ID** | ISO 11784/11785 compliant identification transponder |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"WIA-PET-2025-001"` |
| `number` | IEEE 754 double precision | `85.5`, `42.3` |
| `integer` | Signed 64-bit integer | `365`, `14` |
| `boolean` | Boolean value | `true`, `false` |
| `timestamp` | ISO 8601 datetime | `"2025-12-18T14:30:00Z"` |
| `uuid` | UUID v4 identifier | `"550e8400-e29b-41d4-a716-446655440000"` |
| `iso3166` | ISO 3166-1 alpha-2 country code | `"US"`, `"KR"`, `"GB"` |
| `microchip` | ISO 11784/11785 15-digit code | `"900123456789012"` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Field must be present |
| **OPTIONAL** | Field may be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Root Document Format

All WIA Pet Welfare Global documents follow this base structure:

```json
{
  "wia_version": "1.0.0",
  "standard": "pet-welfare-global",
  "document_id": "uuid",
  "document_type": "string",
  "created_at": "timestamp",
  "updated_at": "timestamp",
  "metadata": {
    "organization_id": "uuid",
    "country_code": "iso3166",
    "language": "string",
    "timezone": "string"
  },
  "data": {}
}
```

### 3.2 Document Types

| Type | Purpose | Key Data |
|------|---------|----------|
| `animal_profile` | Individual animal record | Identity, health, history |
| `shelter_intake` | Animal admission to shelter | Intake details, condition |
| `adoption_record` | Adoption transaction | Adopter, animal, terms |
| `health_passport` | Medical history | Vaccinations, treatments |
| `transport_manifest` | Cross-border movement | Route, carrier, permits |
| `abuse_report` | Incident documentation | Details, evidence, status |
| `welfare_assessment` | Periodic evaluation | Metrics, observations |
| `organization_profile` | Facility information | Capacity, services, staff |

### 3.3 Identifier Standards

| Type | Format | Example |
|------|--------|---------|
| Animal ID | `WIA-PET-{YEAR}-{SEQUENCE}` | `WIA-PET-2025-000001` |
| Organization ID | `WIA-ORG-{COUNTRY}-{SEQUENCE}` | `WIA-ORG-US-001234` |
| Incident ID | `WIA-INC-{YEAR}-{SEQUENCE}` | `WIA-INC-2025-005678` |
| Transport ID | `WIA-TRN-{YEAR}-{SEQUENCE}` | `WIA-TRN-2025-009012` |

---

## Data Schema

### 4.1 Animal Profile Schema

The complete animal profile contains identity, physical characteristics, behavioral traits, and welfare history.

```json
{
  "animal_profile": {
    "wia_id": "WIA-PET-2025-000001",
    "identifiers": {
      "microchip_id": "900123456789012",
      "passport_number": "PET-US-2025-001",
      "registration_number": "REG-CA-2025-1234",
      "tattoo_id": "ABC123",
      "dna_profile_id": "DNA-LAB-001-2025"
    },
    "basic_info": {
      "name": "Max",
      "species": "canis_lupus_familiaris",
      "breed": "Golden Retriever",
      "breed_mix": ["Golden Retriever", "Labrador Retriever"],
      "sex": "male",
      "neutered": true,
      "date_of_birth": "2023-06-15",
      "age_estimate": {
        "years": 2,
        "months": 6,
        "confidence": "high"
      },
      "color": "golden",
      "markings": "white chest patch",
      "size_category": "large"
    },
    "physical_characteristics": {
      "weight_kg": 32.5,
      "height_cm": 58,
      "body_condition_score": 5,
      "distinguishing_features": [
        "Scar on left hind leg",
        "Black spot on tongue"
      ]
    },
    "current_status": {
      "status": "shelter_care",
      "location": {
        "organization_id": "WIA-ORG-US-001234",
        "facility_name": "Happy Paws Animal Shelter",
        "country_code": "US",
        "region": "California",
        "city": "San Francisco",
        "address": "123 Animal Way, San Francisco, CA 94102",
        "gps_coordinates": {
          "latitude": 37.7749,
          "longitude": -122.4194
        }
      },
      "since": "2025-03-15T10:00:00Z",
      "available_for_adoption": true,
      "special_needs": false,
      "requires_experienced_owner": false
    },
    "origin": {
      "source_type": "stray_pickup",
      "source_location": "San Francisco, CA",
      "intake_date": "2025-03-15T10:00:00Z",
      "previous_owner": null,
      "circumstances": "Found wandering in Golden Gate Park",
      "original_country": "US"
    },
    "welfare_scores": {
      "overall_score": 87,
      "physical_health": 90,
      "mental_wellbeing": 85,
      "social_behavior": 88,
      "environmental_enrichment": 82,
      "last_assessed": "2025-12-18T14:00:00Z",
      "assessed_by": "Dr. Sarah Johnson, DVM",
      "assessment_protocol": "WIA-WELFARE-ASSESSMENT-v1.0"
    },
    "behavior_profile": {
      "temperament": "friendly_outgoing",
      "energy_level": "high",
      "socialization": {
        "good_with_dogs": true,
        "good_with_cats": false,
        "good_with_children": true,
        "age_range_children": "8+",
        "good_with_strangers": true
      },
      "training_level": "basic_commands",
      "known_commands": ["sit", "stay", "come", "down"],
      "behavioral_issues": [],
      "special_considerations": [
        "Needs daily exercise",
        "Prefers active families"
      ]
    }
  }
}
```

### 4.2 Health Passport Schema

Comprehensive medical history and vaccination records compliant with international transport requirements.

```json
{
  "health_passport": {
    "passport_id": "PET-US-2025-001",
    "animal_id": "WIA-PET-2025-000001",
    "issued_date": "2025-03-20T10:00:00Z",
    "issuing_authority": {
      "organization": "California Department of Food and Agriculture",
      "country": "US",
      "veterinarian": {
        "name": "Dr. Sarah Johnson",
        "license_number": "CA-VET-12345",
        "contact": "sarah.johnson@happypaws.org"
      }
    },
    "vaccinations": [
      {
        "vaccine_id": "VAC-2025-001",
        "vaccine_name": "Rabies",
        "vaccine_type": "killed_virus",
        "manufacturer": "Merial",
        "batch_number": "RAB-2025-0312",
        "date_administered": "2025-03-20T11:00:00Z",
        "expiry_date": "2028-03-20",
        "next_due_date": "2028-03-20",
        "administered_by": "Dr. Sarah Johnson",
        "site": "right_shoulder",
        "route": "subcutaneous",
        "dose_ml": 1.0,
        "adverse_reactions": []
      },
      {
        "vaccine_id": "VAC-2025-002",
        "vaccine_name": "DHPP",
        "vaccine_type": "modified_live_virus",
        "components": [
          "Distemper",
          "Hepatitis",
          "Parvovirus",
          "Parainfluenza"
        ],
        "manufacturer": "Zoetis",
        "batch_number": "DHPP-2025-0315",
        "date_administered": "2025-03-20T11:15:00Z",
        "expiry_date": "2026-03-20",
        "next_due_date": "2026-03-20",
        "administered_by": "Dr. Sarah Johnson",
        "site": "left_shoulder",
        "route": "subcutaneous",
        "dose_ml": 1.0,
        "adverse_reactions": []
      }
    ],
    "medical_history": [
      {
        "record_id": "MED-2025-001",
        "date": "2025-03-15T10:30:00Z",
        "type": "examination",
        "veterinarian": "Dr. Sarah Johnson",
        "diagnosis": "Healthy adult dog, minor skin abrasion on left hind leg",
        "treatment": "Wound cleaning and topical antibiotic",
        "medications": [
          {
            "name": "Neosporin",
            "dosage": "topical application",
            "frequency": "twice daily",
            "duration": "7 days"
          }
        ],
        "follow_up_required": false,
        "notes": "Wound healing well, no complications"
      }
    ],
    "parasite_control": {
      "last_deworming": "2025-03-20T12:00:00Z",
      "product": "Panacur",
      "next_due": "2025-06-20",
      "flea_tick_prevention": {
        "product": "Frontline Plus",
        "last_applied": "2025-12-01T10:00:00Z",
        "next_due": "2026-01-01"
      },
      "heartworm": {
        "last_test_date": "2025-03-20",
        "test_result": "negative",
        "prevention_product": "Heartgard Plus",
        "last_dose": "2025-12-01",
        "next_due": "2026-01-01"
      }
    },
    "health_certificates": [
      {
        "certificate_id": "HC-US-2025-001",
        "type": "international_travel",
        "issuing_country": "US",
        "destination_country": "GB",
        "issue_date": "2025-12-15T10:00:00Z",
        "valid_until": "2026-01-15",
        "certified_by": "Dr. Sarah Johnson, DVM",
        "accreditation_number": "USDA-APHIS-12345",
        "health_status": "fit_to_travel",
        "special_notes": "Compliant with UK pet travel scheme requirements"
      }
    ],
    "genetic_testing": [
      {
        "test_id": "DNA-LAB-001-2025",
        "test_date": "2025-03-25T10:00:00Z",
        "laboratory": "Embark Veterinary",
        "test_type": "breed_identification",
        "results": {
          "primary_breed": "Golden Retriever",
          "percentage": 87.5,
          "secondary_breed": "Labrador Retriever",
          "percentage_secondary": 12.5
        }
      }
    ],
    "spay_neuter_record": {
      "procedure_date": "2024-01-15T09:00:00Z",
      "veterinarian": "Dr. Michael Chen",
      "facility": "Bay Area Veterinary Clinic",
      "method": "ovariohysterectomy",
      "complications": "none",
      "recovery_notes": "Excellent recovery, no complications"
    }
  }
}
```

### 4.3 Shelter Intake Schema

Standardized intake documentation for animals entering shelter care.

```json
{
  "shelter_intake": {
    "intake_id": "INTAKE-2025-12345",
    "animal_id": "WIA-PET-2025-000001",
    "organization_id": "WIA-ORG-US-001234",
    "intake_date": "2025-03-15T10:00:00Z",
    "intake_type": "stray",
    "intake_officer": {
      "name": "Jessica Martinez",
      "employee_id": "EMP-001",
      "contact": "jessica.martinez@happypaws.org"
    },
    "source_information": {
      "brought_by": "animal_control",
      "animal_control_agency": "San Francisco Animal Care & Control",
      "case_number": "SFACC-2025-0315-001",
      "pickup_location": {
        "address": "Golden Gate Park, San Francisco, CA",
        "gps_coordinates": {
          "latitude": 37.7694,
          "longitude": -122.4862
        }
      },
      "circumstances": "Found wandering alone, appeared friendly and well-cared for but no collar or ID",
      "finder_contact": null
    },
    "initial_assessment": {
      "physical_condition": "good",
      "body_condition_score": 5,
      "estimated_age": "2 years",
      "temperament": "friendly",
      "obvious_injuries": false,
      "urgent_medical_needs": false,
      "behavioral_concerns": false,
      "assessed_by": "Jessica Martinez",
      "notes": "Well-socialized dog, likely lost or abandoned. No signs of abuse or neglect."
    },
    "intake_photos": [
      {
        "photo_id": "PHOTO-001",
        "url": "https://storage.wia.org/pet-welfare/photos/WIA-PET-2025-000001-intake-01.jpg",
        "timestamp": "2025-03-15T10:15:00Z",
        "view": "right_side",
        "photographer": "Jessica Martinez"
      },
      {
        "photo_id": "PHOTO-002",
        "url": "https://storage.wia.org/pet-welfare/photos/WIA-PET-2025-000001-intake-02.jpg",
        "timestamp": "2025-03-15T10:16:00Z",
        "view": "left_side",
        "photographer": "Jessica Martinez"
      }
    ],
    "microchip_scan": {
      "scanned": true,
      "chip_found": true,
      "microchip_id": "900123456789012",
      "manufacturer": "HomeAgain",
      "registration_status": "not_registered",
      "owner_contact_attempted": true,
      "owner_contact_result": "number_disconnected"
    },
    "hold_status": {
      "hold_type": "stray_hold",
      "hold_start": "2025-03-15T10:00:00Z",
      "hold_end": "2025-03-20T10:00:00Z",
      "hold_duration_days": 5,
      "reason": "Statutory stray hold period",
      "can_be_adopted_after": "2025-03-20T10:00:00Z"
    },
    "housing_assignment": {
      "kennel_id": "K-15",
      "building": "Main Shelter",
      "wing": "North",
      "assignment_date": "2025-03-15T11:00:00Z",
      "kennel_type": "standard_large_dog",
      "indoor_outdoor": true,
      "special_requirements": []
    }
  }
}
```

### 4.4 Cross-Border Transport Schema

Complete documentation for international animal movement compliant with IATA and customs requirements.

```json
{
  "transport_manifest": {
    "transport_id": "WIA-TRN-2025-009012",
    "animal_id": "WIA-PET-2025-000001",
    "transport_type": "international_adoption",
    "status": "in_transit",
    "origin": {
      "country_code": "US",
      "organization_id": "WIA-ORG-US-001234",
      "facility_name": "Happy Paws Animal Shelter",
      "address": "123 Animal Way, San Francisco, CA 94102",
      "contact": {
        "name": "Jessica Martinez",
        "phone": "+1-415-555-0123",
        "email": "jessica.martinez@happypaws.org"
      },
      "departure_date": "2025-12-20T08:00:00Z"
    },
    "destination": {
      "country_code": "GB",
      "organization_id": "WIA-ORG-GB-005678",
      "facility_name": "London Pet Rescue",
      "address": "456 Animal Street, London, UK E1 6AN",
      "contact": {
        "name": "James Thompson",
        "phone": "+44-20-7946-0958",
        "email": "james.thompson@londonpetrescue.org.uk"
      },
      "expected_arrival": "2025-12-21T14:00:00Z"
    },
    "route": [
      {
        "leg_number": 1,
        "departure_airport": "SFO",
        "arrival_airport": "JFK",
        "departure_time": "2025-12-20T10:00:00Z",
        "arrival_time": "2025-12-20T18:30:00Z",
        "airline": "American Airlines",
        "flight_number": "AA100",
        "booking_reference": "ABC123"
      },
      {
        "leg_number": 2,
        "departure_airport": "JFK",
        "arrival_airport": "LHR",
        "departure_time": "2025-12-20T22:00:00Z",
        "arrival_time": "2025-12-21T10:00:00Z",
        "airline": "British Airways",
        "flight_number": "BA178",
        "booking_reference": "XYZ789"
      }
    ],
    "iata_compliance": {
      "live_animals_regulations_version": "LAR-49",
      "container_requirement": "CR82",
      "container_id": "CRATE-2025-001",
      "container_dimensions": {
        "length_cm": 91,
        "width_cm": 61,
        "height_cm": 66
      },
      "ventilation_compliance": true,
      "food_water_provision": true,
      "handler_instructions": "Handle with care. Friendly dog. Water every 4 hours.",
      "special_requirements": []
    },
    "health_documentation": {
      "health_certificate_id": "HC-US-2025-001",
      "rabies_certificate_id": "RAB-US-2025-001",
      "veterinary_examination_date": "2025-12-15T10:00:00Z",
      "fit_to_travel": true,
      "certifying_veterinarian": "Dr. Sarah Johnson",
      "usda_endorsement": true,
      "usda_endorsement_number": "USDA-2025-001234"
    },
    "import_permits": [
      {
        "permit_id": "UK-IMPORT-2025-001",
        "issuing_authority": "UK Department for Environment, Food and Rural Affairs",
        "issue_date": "2025-12-01T10:00:00Z",
        "valid_until": "2026-01-01",
        "permit_type": "pet_travel_scheme",
        "special_conditions": [
          "Must be microchipped",
          "Must have valid rabies vaccination",
          "Must have tapeworm treatment 1-5 days before entry"
        ]
      }
    ],
    "quarantine_requirements": {
      "required": false,
      "reason": "Compliant with UK Pet Travel Scheme",
      "facility": null,
      "duration_days": 0
    },
    "customs_declaration": {
      "declared_value_usd": 0,
      "purpose": "adoption",
      "commercial": false,
      "customs_form_number": "UK-CUSTOMS-2025-001234"
    },
    "adopter_information": {
      "name": "Emily Wilson",
      "address": "789 Pet Lane, London, UK E1 7AA",
      "phone": "+44-20-7946-1234",
      "email": "emily.wilson@email.co.uk",
      "adoption_contract_id": "ADOPT-2025-001"
    }
  }
}
```

### 4.5 Abuse Report Schema

Standardized documentation for animal abuse and neglect incidents.

```json
{
  "abuse_report": {
    "incident_id": "WIA-INC-2025-005678",
    "report_date": "2025-11-15T14:30:00Z",
    "incident_date": "2025-11-15T10:00:00Z",
    "status": "under_investigation",
    "severity_level": "moderate",
    "incident_type": "neglect",
    "reporter": {
      "reporter_type": "citizen",
      "anonymous": false,
      "name": "John Smith",
      "contact": {
        "phone": "+1-415-555-9876",
        "email": "john.smith@email.com"
      },
      "address": "123 Main Street, San Francisco, CA 94101",
      "relationship_to_animals": "neighbor"
    },
    "location": {
      "address": "456 Oak Street, San Francisco, CA 94102",
      "gps_coordinates": {
        "latitude": 37.7749,
        "longitude": -122.4194
      },
      "property_type": "single_family_residence",
      "access_restrictions": "private_property"
    },
    "animals_involved": [
      {
        "description": "Large mixed-breed dog, brown and white",
        "estimated_age": "5-7 years",
        "sex": "male",
        "microchip_scanned": false,
        "current_location": "seized_by_authority",
        "shelter_id": "WIA-ORG-US-001234",
        "animal_id": "WIA-PET-2025-000099"
      }
    ],
    "incident_details": {
      "description": "Dog found chained outside without shelter, food, or water in freezing temperatures. Animal appears malnourished.",
      "duration": "unknown",
      "environmental_conditions": {
        "temperature_celsius": -2,
        "weather": "freezing_rain",
        "exposure": "no_shelter"
      },
      "visible_injuries": true,
      "injury_description": "Matted fur, visible ribs, pressure sores from chain",
      "behavioral_indicators": "lethargic, fearful"
    },
    "evidence": [
      {
        "evidence_id": "EVID-001",
        "type": "photograph",
        "url": "https://secure.wia.org/evidence/WIA-INC-2025-005678-001.jpg",
        "timestamp": "2025-11-15T10:30:00Z",
        "description": "Photo of dog chained outside",
        "collected_by": "Officer Maria Garcia"
      },
      {
        "evidence_id": "EVID-002",
        "type": "veterinary_report",
        "url": "https://secure.wia.org/evidence/WIA-INC-2025-005678-002.pdf",
        "timestamp": "2025-11-15T14:00:00Z",
        "description": "Initial veterinary examination",
        "collected_by": "Dr. Sarah Johnson"
      }
    ],
    "investigation": {
      "assigned_to": "Officer Maria Garcia",
      "agency": "San Francisco Animal Care & Control",
      "case_number": "SFACC-INV-2025-1115-001",
      "investigation_start": "2025-11-15T11:00:00Z",
      "last_updated": "2025-12-15T16:00:00Z",
      "actions_taken": [
        {
          "action_date": "2025-11-15T11:30:00Z",
          "action": "animal_seizure",
          "description": "Dog seized under emergency authority",
          "performed_by": "Officer Maria Garcia"
        },
        {
          "action_date": "2025-11-15T12:00:00Z",
          "action": "owner_notification",
          "description": "Owner notified of seizure and pending investigation",
          "performed_by": "Officer Maria Garcia"
        },
        {
          "action_date": "2025-11-15T14:00:00Z",
          "action": "veterinary_examination",
          "description": "Complete medical examination performed",
          "performed_by": "Dr. Sarah Johnson"
        }
      ],
      "findings": "Evidence of long-term neglect. Animal malnourished and exposed to dangerous weather conditions without adequate shelter, food, or water.",
      "recommendations": "Recommend criminal charges for animal neglect. Animal should not be returned to owner."
    },
    "legal_proceedings": {
      "charges_filed": true,
      "charge_date": "2025-11-20T10:00:00Z",
      "charges": [
        {
          "statute": "California Penal Code 597",
          "description": "Animal neglect",
          "severity": "misdemeanor"
        }
      ],
      "court_date": "2026-01-15T09:00:00Z",
      "case_number": "SF-2025-CR-1234",
      "prosecutor": "San Francisco District Attorney's Office"
    },
    "outcome": {
      "animal_disposition": "permanent_seizure",
      "available_for_adoption": true,
      "owner_surrender": false,
      "conviction": null,
      "penalties": null
    }
  }
}
```

### 4.6 Organization Profile Schema

Comprehensive shelter and rescue organization information.

```json
{
  "organization_profile": {
    "organization_id": "WIA-ORG-US-001234",
    "basic_info": {
      "name": "Happy Paws Animal Shelter",
      "legal_name": "Happy Paws Animal Welfare Society, Inc.",
      "organization_type": "nonprofit_shelter",
      "founded_date": "1995-06-15",
      "tax_id": "12-3456789",
      "registration_number": "CA-NONPROFIT-001234"
    },
    "contact": {
      "primary_address": {
        "street": "123 Animal Way",
        "city": "San Francisco",
        "region": "California",
        "postal_code": "94102",
        "country_code": "US"
      },
      "phone": "+1-415-555-0123",
      "email": "info@happypaws.org",
      "website": "https://www.happypaws.org",
      "social_media": {
        "facebook": "happypawssf",
        "instagram": "happypawssf",
        "twitter": "happypawssf"
      }
    },
    "operations": {
      "services_provided": [
        "animal_shelter",
        "adoption_services",
        "veterinary_care",
        "spay_neuter_clinic",
        "behavioral_training",
        "foster_program",
        "volunteer_program"
      ],
      "species_accepted": [
        "dogs",
        "cats",
        "rabbits",
        "small_animals"
      ],
      "intake_policy": "limited_admission",
      "euthanasia_policy": "no_kill",
      "operating_hours": {
        "monday": "09:00-17:00",
        "tuesday": "09:00-17:00",
        "wednesday": "09:00-17:00",
        "thursday": "09:00-19:00",
        "friday": "09:00-17:00",
        "saturday": "10:00-16:00",
        "sunday": "10:00-16:00"
      }
    },
    "capacity": {
      "total_capacity": 150,
      "current_occupancy": 127,
      "dogs": {
        "capacity": 80,
        "current": 68
      },
      "cats": {
        "capacity": 60,
        "current": 52
      },
      "other": {
        "capacity": 10,
        "current": 7
      }
    },
    "licensing": [
      {
        "license_type": "animal_shelter",
        "license_number": "CA-SHELTER-001234",
        "issuing_authority": "California Department of Food and Agriculture",
        "issue_date": "2025-01-01",
        "expiry_date": "2026-01-01",
        "status": "active"
      }
    ],
    "certifications": [
      {
        "certification": "ASPCA_partnership",
        "certified_date": "2020-06-15",
        "valid_until": "2026-06-15"
      }
    ],
    "staff": {
      "full_time": 15,
      "part_time": 8,
      "volunteers": 45,
      "veterinarians": 2,
      "veterinary_technicians": 4
    }
  }
}
```

---

## Welfare Metrics

### 5.1 Welfare Scoring System

The WIA Welfare Score is a comprehensive 0-100 scale assessment across five domains:

| Domain | Weight | Description |
|--------|--------|-------------|
| Physical Health | 25% | Medical condition, nutrition, fitness |
| Mental Wellbeing | 20% | Stress levels, anxiety, depression indicators |
| Social Behavior | 20% | Interaction quality, socialization |
| Environmental Enrichment | 20% | Housing quality, stimulation, exercise |
| Care Standards | 15% | Quality of care, staff expertise, resources |

### 5.2 Scoring Criteria

| Score Range | Classification | Description |
|-------------|----------------|-------------|
| 90-100 | Excellent | Optimal welfare across all domains |
| 75-89 | Good | High welfare with minor improvements possible |
| 60-74 | Acceptable | Adequate welfare, some concerns |
| 45-59 | Poor | Significant welfare issues requiring intervention |
| 0-44 | Critical | Severe welfare compromise, immediate action needed |

### 5.3 Assessment Protocol

Welfare assessments must be conducted:
- Within 24 hours of intake
- Monthly for animals in long-term care
- Before and after transport
- Before adoption placement
- Following any significant health or behavioral event

---

## Validation Rules

### 6.1 Required Field Validation

| Field | Validation Rule |
|-------|----------------|
| `microchip_id` | Must be 15 digits, ISO 11784/11785 compliant |
| `country_code` | Must be valid ISO 3166-1 alpha-2 code |
| `date_of_birth` | Must not be in the future |
| `welfare_score` | Must be integer 0-100 |
| `weight_kg` | Must be positive number |
| `vaccination_date` | Must not be in the future |

### 6.2 Business Rules

1. Animals cannot be adopted during mandatory hold periods
2. International transport requires valid health certificates
3. Rabies vaccination must be current for cross-border movement
4. Welfare scores below 45 trigger mandatory intervention protocols
5. Abuse reports must be assigned within 24 hours

### 6.3 Cross-Field Validation

- `available_for_adoption` cannot be true if `hold_status.can_be_adopted_after` is in the future
- `neutered` must be true for adoption if `age_months` > 6
- `transport_type` = "international" requires `health_certificate_id`

---

## Examples

### Example 1: Complete Animal Profile with International Adoption

```json
{
  "wia_version": "1.0.0",
  "standard": "pet-welfare-global",
  "document_id": "550e8400-e29b-41d4-a716-446655440000",
  "document_type": "animal_profile",
  "created_at": "2025-03-15T10:00:00Z",
  "updated_at": "2025-12-18T14:30:00Z",
  "metadata": {
    "organization_id": "WIA-ORG-US-001234",
    "country_code": "US",
    "language": "en",
    "timezone": "America/Los_Angeles"
  },
  "data": {
    "animal_profile": {
      "wia_id": "WIA-PET-2025-000001",
      "identifiers": {
        "microchip_id": "900123456789012",
        "passport_number": "PET-US-2025-001"
      },
      "basic_info": {
        "name": "Max",
        "species": "canis_lupus_familiaris",
        "breed": "Golden Retriever",
        "sex": "male",
        "neutered": true,
        "date_of_birth": "2023-06-15"
      },
      "welfare_scores": {
        "overall_score": 87,
        "physical_health": 90,
        "mental_wellbeing": 85,
        "social_behavior": 88,
        "environmental_enrichment": 82,
        "last_assessed": "2025-12-18T14:00:00Z"
      }
    }
  }
}
```

### Example 2: Abuse Report with Investigation

```json
{
  "wia_version": "1.0.0",
  "standard": "pet-welfare-global",
  "document_id": "660f9500-f39c-52e5-b827-557766551111",
  "document_type": "abuse_report",
  "created_at": "2025-11-15T14:30:00Z",
  "updated_at": "2025-12-15T16:00:00Z",
  "metadata": {
    "organization_id": "WIA-ORG-US-005678",
    "country_code": "US",
    "language": "en",
    "timezone": "America/Los_Angeles"
  },
  "data": {
    "abuse_report": {
      "incident_id": "WIA-INC-2025-005678",
      "report_date": "2025-11-15T14:30:00Z",
      "incident_type": "neglect",
      "severity_level": "moderate",
      "status": "under_investigation",
      "incident_details": {
        "description": "Dog found chained outside without shelter, food, or water in freezing temperatures.",
        "visible_injuries": true
      }
    }
  }
}
```

### Example 3: International Transport Manifest

```json
{
  "wia_version": "1.0.0",
  "standard": "pet-welfare-global",
  "document_id": "770fa611-g40d-63f6-c938-668877662222",
  "document_type": "transport_manifest",
  "created_at": "2025-12-18T08:00:00Z",
  "updated_at": "2025-12-20T10:00:00Z",
  "metadata": {
    "organization_id": "WIA-ORG-US-001234",
    "country_code": "US",
    "language": "en",
    "timezone": "America/Los_Angeles"
  },
  "data": {
    "transport_manifest": {
      "transport_id": "WIA-TRN-2025-009012",
      "animal_id": "WIA-PET-2025-000001",
      "transport_type": "international_adoption",
      "status": "in_transit",
      "origin": {
        "country_code": "US",
        "departure_date": "2025-12-20T08:00:00Z"
      },
      "destination": {
        "country_code": "GB",
        "expected_arrival": "2025-12-21T14:00:00Z"
      },
      "iata_compliance": {
        "live_animals_regulations_version": "LAR-49",
        "container_requirement": "CR82"
      }
    }
  }
}
```

### Example 4: Welfare Assessment

```json
{
  "wia_version": "1.0.0",
  "standard": "pet-welfare-global",
  "document_id": "880fb722-h51e-74g7-d049-779988773333",
  "document_type": "welfare_assessment",
  "created_at": "2025-12-18T14:00:00Z",
  "updated_at": "2025-12-18T14:00:00Z",
  "metadata": {
    "organization_id": "WIA-ORG-US-001234",
    "country_code": "US",
    "language": "en",
    "timezone": "America/Los_Angeles"
  },
  "data": {
    "welfare_assessment": {
      "assessment_id": "ASSESS-2025-001",
      "animal_id": "WIA-PET-2025-000001",
      "assessment_date": "2025-12-18T14:00:00Z",
      "assessed_by": "Dr. Sarah Johnson, DVM",
      "protocol_version": "WIA-WELFARE-ASSESSMENT-v1.0",
      "scores": {
        "physical_health": {
          "score": 90,
          "observations": "Excellent body condition, healthy coat, no injuries",
          "metrics": {
            "body_condition_score": 5,
            "mobility": "excellent",
            "pain_indicators": "none"
          }
        },
        "mental_wellbeing": {
          "score": 85,
          "observations": "Alert and engaged, some anxiety around loud noises",
          "metrics": {
            "stress_level": "low",
            "anxiety_indicators": "mild",
            "behavioral_health": "good"
          }
        },
        "social_behavior": {
          "score": 88,
          "observations": "Friendly with people and other dogs, well-socialized",
          "metrics": {
            "human_interaction": "positive",
            "animal_interaction": "positive",
            "play_behavior": "appropriate"
          }
        },
        "environmental_enrichment": {
          "score": 82,
          "observations": "Good housing, adequate exercise, could benefit from more mental stimulation",
          "metrics": {
            "housing_quality": "good",
            "exercise_frequency": "daily",
            "enrichment_activities": "moderate"
          }
        },
        "care_standards": {
          "score": 90,
          "observations": "Excellent care from qualified staff, all needs met",
          "metrics": {
            "nutrition": "optimal",
            "veterinary_care": "current",
            "staff_training": "excellent"
          }
        },
        "overall_score": 87
      },
      "recommendations": [
        "Continue current care protocols",
        "Increase mental enrichment activities",
        "Consider puzzle toys and training sessions"
      ],
      "next_assessment_due": "2026-01-18T14:00:00Z"
    }
  }
}
```

### Example 5: Shelter Organization Profile

```json
{
  "wia_version": "1.0.0",
  "standard": "pet-welfare-global",
  "document_id": "990fc833-i62f-85h8-e150-880099884444",
  "document_type": "organization_profile",
  "created_at": "2025-01-01T10:00:00Z",
  "updated_at": "2025-12-18T10:00:00Z",
  "metadata": {
    "organization_id": "WIA-ORG-US-001234",
    "country_code": "US",
    "language": "en",
    "timezone": "America/Los_Angeles"
  },
  "data": {
    "organization_profile": {
      "organization_id": "WIA-ORG-US-001234",
      "basic_info": {
        "name": "Happy Paws Animal Shelter",
        "organization_type": "nonprofit_shelter",
        "founded_date": "1995-06-15"
      },
      "contact": {
        "primary_address": {
          "street": "123 Animal Way",
          "city": "San Francisco",
          "region": "California",
          "country_code": "US"
        },
        "phone": "+1-415-555-0123",
        "email": "info@happypaws.org"
      },
      "operations": {
        "services_provided": [
          "animal_shelter",
          "adoption_services",
          "veterinary_care"
        ],
        "euthanasia_policy": "no_kill"
      },
      "capacity": {
        "total_capacity": 150,
        "current_occupancy": 127
      }
    }
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-18 | Initial release of WIA Pet Welfare Global Data Format Standard |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
