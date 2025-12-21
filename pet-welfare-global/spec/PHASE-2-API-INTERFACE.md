# WIA Pet Welfare Global API Interface Standard
## Phase 2 Specification

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
2. [Authentication](#authentication)
3. [API Endpoints](#api-endpoints)
4. [Request/Response Format](#requestresponse-format)
5. [Error Handling](#error-handling)
6. [Rate Limiting](#rate-limiting)
7. [Webhooks](#webhooks)
8. [Code Examples](#code-examples)
9. [API Reference](#api-reference)

---

## Overview

### 1.1 Purpose

The WIA Pet Welfare Global API Interface Standard defines RESTful API endpoints, authentication mechanisms, data exchange protocols, and integration patterns for animal welfare systems, shelters, rescue organizations, government agencies, and cross-border coordination platforms.

**Core Objectives**:
- Provide standardized API access to animal welfare data
- Enable real-time updates and notifications
- Support cross-border data exchange and adoption workflows
- Facilitate integration with existing shelter management systems
- Ensure secure and authorized data access
- Enable abuse reporting and tracking systems

### 1.2 API Architecture

| Component | Technology | Description |
|-----------|------------|-------------|
| Protocol | HTTPS/REST | RESTful API over secure HTTP |
| Format | JSON | All requests and responses in JSON |
| Authentication | OAuth 2.0 + API Keys | Token-based authentication |
| Versioning | URI Versioning | Version in URL path (/v1/) |
| Rate Limiting | Token Bucket | 1000 requests/hour base tier |

### 1.3 Base URL Structure

```
https://api.wia.org/pet-welfare-global/v1/{resource}
```

**Environments**:
- Production: `https://api.wia.org/pet-welfare-global/v1/`
- Staging: `https://api-staging.wia.org/pet-welfare-global/v1/`
- Sandbox: `https://api-sandbox.wia.org/pet-welfare-global/v1/`

---

## Authentication

### 2.1 Authentication Methods

| Method | Use Case | Security Level |
|--------|----------|----------------|
| OAuth 2.0 | User-delegated access | High |
| API Key | Service-to-service | Medium |
| JWT Token | Session management | High |
| Client Credentials | Server applications | High |

### 2.2 OAuth 2.0 Flow

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=your_client_id
&client_secret=your_client_secret
&scope=animal:read animal:write shelter:manage
```

**Response**:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "animal:read animal:write shelter:manage"
}
```

### 2.3 API Key Authentication

```http
GET /animals/WIA-PET-2025-000001
Authorization: Bearer YOUR_API_KEY
X-WIA-Organization-ID: WIA-ORG-US-001234
```

### 2.4 Permission Scopes

| Scope | Access Level | Description |
|-------|--------------|-------------|
| `animal:read` | Read | View animal profiles and data |
| `animal:write` | Write | Create and update animal records |
| `animal:delete` | Delete | Remove animal records |
| `shelter:read` | Read | View shelter information |
| `shelter:manage` | Admin | Manage shelter operations |
| `adoption:process` | Write | Process adoption applications |
| `transport:manage` | Write | Manage international transport |
| `abuse:report` | Write | Submit abuse reports |
| `abuse:investigate` | Admin | Access investigation data |
| `health:read` | Read | View health records |
| `health:write` | Write | Update medical records |
| `admin:full` | Super Admin | Full system access |

---

## API Endpoints

### 3.1 Animal Management Endpoints

#### 3.1.1 Create Animal Profile

```http
POST /animals
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "identifiers": {
    "microchip_id": "900123456789012",
    "passport_number": "PET-US-2025-001"
  },
  "basic_info": {
    "name": "Max",
    "species": "canis_lupus_familiaris",
    "breed": "Golden Retriever",
    "sex": "male",
    "date_of_birth": "2023-06-15"
  },
  "current_status": {
    "status": "shelter_care",
    "organization_id": "WIA-ORG-US-001234"
  }
}
```

**Response** (201 Created):
```json
{
  "success": true,
  "data": {
    "wia_id": "WIA-PET-2025-000001",
    "created_at": "2025-12-18T10:00:00Z",
    "animal_profile": {
      "wia_id": "WIA-PET-2025-000001",
      "identifiers": {
        "microchip_id": "900123456789012"
      },
      "basic_info": {
        "name": "Max",
        "species": "canis_lupus_familiaris"
      }
    }
  },
  "links": {
    "self": "/animals/WIA-PET-2025-000001",
    "health_passport": "/animals/WIA-PET-2025-000001/health-passport",
    "welfare_assessments": "/animals/WIA-PET-2025-000001/welfare-assessments"
  }
}
```

#### 3.1.2 Get Animal Profile

```http
GET /animals/{animal_id}
Authorization: Bearer {token}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "animal_profile": {
      "wia_id": "WIA-PET-2025-000001",
      "identifiers": {
        "microchip_id": "900123456789012"
      },
      "basic_info": {
        "name": "Max",
        "species": "canis_lupus_familiaris",
        "breed": "Golden Retriever"
      },
      "welfare_scores": {
        "overall_score": 87,
        "last_assessed": "2025-12-18T14:00:00Z"
      }
    }
  }
}
```

#### 3.1.3 Update Animal Profile

```http
PATCH /animals/{animal_id}
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "current_status": {
    "status": "adopted",
    "adoption_date": "2025-12-20T10:00:00Z"
  },
  "welfare_scores": {
    "overall_score": 92
  }
}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "wia_id": "WIA-PET-2025-000001",
    "updated_at": "2025-12-20T10:00:00Z"
  }
}
```

#### 3.1.4 Search Animals

```http
GET /animals/search
Authorization: Bearer {token}
```

**Query Parameters**:

| Parameter | Type | Description |
|-----------|------|-------------|
| `species` | string | Filter by species |
| `breed` | string | Filter by breed |
| `status` | string | Filter by current status |
| `organization_id` | string | Filter by organization |
| `min_welfare_score` | integer | Minimum welfare score |
| `available_for_adoption` | boolean | Available for adoption |
| `country_code` | string | ISO country code |
| `age_min` | integer | Minimum age in months |
| `age_max` | integer | Maximum age in months |
| `page` | integer | Page number (default: 1) |
| `limit` | integer | Results per page (default: 20, max: 100) |

**Example Request**:
```http
GET /animals/search?species=canis_lupus_familiaris&status=shelter_care&available_for_adoption=true&page=1&limit=20
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "animals": [
      {
        "wia_id": "WIA-PET-2025-000001",
        "name": "Max",
        "species": "canis_lupus_familiaris",
        "breed": "Golden Retriever",
        "welfare_score": 87,
        "available_for_adoption": true
      }
    ],
    "pagination": {
      "page": 1,
      "limit": 20,
      "total_results": 156,
      "total_pages": 8
    }
  },
  "links": {
    "self": "/animals/search?page=1&limit=20",
    "next": "/animals/search?page=2&limit=20",
    "last": "/animals/search?page=8&limit=20"
  }
}
```

#### 3.1.5 Delete Animal Profile

```http
DELETE /animals/{animal_id}
Authorization: Bearer {token}
X-Reason: Duplicate record
```

**Response** (204 No Content)

### 3.2 Health Passport Endpoints

#### 3.2.1 Create Health Passport

```http
POST /animals/{animal_id}/health-passport
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "issued_date": "2025-03-20T10:00:00Z",
  "issuing_authority": {
    "organization": "California Department of Food and Agriculture",
    "country": "US",
    "veterinarian": {
      "name": "Dr. Sarah Johnson",
      "license_number": "CA-VET-12345"
    }
  }
}
```

**Response** (201 Created):
```json
{
  "success": true,
  "data": {
    "passport_id": "PET-US-2025-001",
    "animal_id": "WIA-PET-2025-000001",
    "created_at": "2025-03-20T10:00:00Z"
  }
}
```

#### 3.2.2 Add Vaccination Record

```http
POST /animals/{animal_id}/health-passport/vaccinations
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "vaccine_name": "Rabies",
  "vaccine_type": "killed_virus",
  "manufacturer": "Merial",
  "batch_number": "RAB-2025-0312",
  "date_administered": "2025-03-20T11:00:00Z",
  "expiry_date": "2028-03-20",
  "administered_by": "Dr. Sarah Johnson",
  "dose_ml": 1.0
}
```

**Response** (201 Created):
```json
{
  "success": true,
  "data": {
    "vaccine_id": "VAC-2025-001",
    "animal_id": "WIA-PET-2025-000001",
    "vaccine_name": "Rabies",
    "date_administered": "2025-03-20T11:00:00Z",
    "next_due_date": "2028-03-20"
  }
}
```

#### 3.2.3 Get Health Passport

```http
GET /animals/{animal_id}/health-passport
Authorization: Bearer {token}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "health_passport": {
      "passport_id": "PET-US-2025-001",
      "animal_id": "WIA-PET-2025-000001",
      "vaccinations": [
        {
          "vaccine_id": "VAC-2025-001",
          "vaccine_name": "Rabies",
          "date_administered": "2025-03-20T11:00:00Z",
          "expiry_date": "2028-03-20"
        }
      ],
      "medical_history": []
    }
  }
}
```

### 3.3 Adoption Management Endpoints

#### 3.3.1 Submit Adoption Application

```http
POST /adoptions/applications
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "animal_id": "WIA-PET-2025-000001",
  "applicant": {
    "name": "Emily Wilson",
    "email": "emily.wilson@email.com",
    "phone": "+44-20-7946-1234",
    "address": {
      "street": "789 Pet Lane",
      "city": "London",
      "country_code": "GB",
      "postal_code": "E1 7AA"
    }
  },
  "household_info": {
    "adults": 2,
    "children": 1,
    "children_ages": [8],
    "other_pets": true,
    "other_pets_details": "One cat, age 5"
  },
  "adoption_type": "international"
}
```

**Response** (201 Created):
```json
{
  "success": true,
  "data": {
    "application_id": "APP-2025-001",
    "animal_id": "WIA-PET-2025-000001",
    "status": "pending_review",
    "submitted_at": "2025-12-18T10:00:00Z"
  }
}
```

#### 3.3.2 Get Adoption Application

```http
GET /adoptions/applications/{application_id}
Authorization: Bearer {token}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "application": {
      "application_id": "APP-2025-001",
      "animal_id": "WIA-PET-2025-000001",
      "status": "approved",
      "applicant": {
        "name": "Emily Wilson"
      },
      "submitted_at": "2025-12-18T10:00:00Z",
      "reviewed_at": "2025-12-19T14:00:00Z"
    }
  }
}
```

#### 3.3.3 Approve Adoption

```http
POST /adoptions/applications/{application_id}/approve
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "approved_by": "Jessica Martinez",
  "notes": "Excellent home for Max, experienced dog owner",
  "adoption_fee": 250.00,
  "adoption_date": "2025-12-20T10:00:00Z"
}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "application_id": "APP-2025-001",
    "status": "approved",
    "adoption_contract_id": "ADOPT-2025-001",
    "adoption_date": "2025-12-20T10:00:00Z"
  }
}
```

### 3.4 Transport Management Endpoints

#### 3.4.1 Create Transport Manifest

```http
POST /transport/manifests
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "animal_id": "WIA-PET-2025-000001",
  "transport_type": "international_adoption",
  "origin": {
    "country_code": "US",
    "organization_id": "WIA-ORG-US-001234",
    "departure_date": "2025-12-20T08:00:00Z"
  },
  "destination": {
    "country_code": "GB",
    "organization_id": "WIA-ORG-GB-005678",
    "expected_arrival": "2025-12-21T14:00:00Z"
  },
  "route": [
    {
      "leg_number": 1,
      "departure_airport": "SFO",
      "arrival_airport": "JFK",
      "flight_number": "AA100"
    }
  ]
}
```

**Response** (201 Created):
```json
{
  "success": true,
  "data": {
    "transport_id": "WIA-TRN-2025-009012",
    "animal_id": "WIA-PET-2025-000001",
    "status": "scheduled",
    "created_at": "2025-12-18T10:00:00Z"
  }
}
```

#### 3.4.2 Update Transport Status

```http
PATCH /transport/manifests/{transport_id}/status
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "status": "in_transit",
  "current_location": "JFK",
  "updated_at": "2025-12-20T18:30:00Z",
  "notes": "Successfully transferred to connecting flight"
}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "transport_id": "WIA-TRN-2025-009012",
    "status": "in_transit",
    "updated_at": "2025-12-20T18:30:00Z"
  }
}
```

#### 3.4.3 Get Transport Manifest

```http
GET /transport/manifests/{transport_id}
Authorization: Bearer {token}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "transport_manifest": {
      "transport_id": "WIA-TRN-2025-009012",
      "animal_id": "WIA-PET-2025-000001",
      "status": "completed",
      "origin": {
        "country_code": "US"
      },
      "destination": {
        "country_code": "GB"
      }
    }
  }
}
```

### 3.5 Abuse Reporting Endpoints

#### 3.5.1 Submit Abuse Report

```http
POST /abuse/reports
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "incident_date": "2025-11-15T10:00:00Z",
  "incident_type": "neglect",
  "severity_level": "moderate",
  "reporter": {
    "reporter_type": "citizen",
    "anonymous": false,
    "name": "John Smith",
    "contact": {
      "phone": "+1-415-555-9876",
      "email": "john.smith@email.com"
    }
  },
  "location": {
    "address": "456 Oak Street, San Francisco, CA 94102",
    "country_code": "US"
  },
  "incident_details": {
    "description": "Dog found chained outside without shelter, food, or water in freezing temperatures.",
    "visible_injuries": true
  }
}
```

**Response** (201 Created):
```json
{
  "success": true,
  "data": {
    "incident_id": "WIA-INC-2025-005678",
    "status": "pending_review",
    "reported_at": "2025-11-15T14:30:00Z",
    "case_number": "Case assigned upon review"
  }
}
```

#### 3.5.2 Get Abuse Report

```http
GET /abuse/reports/{incident_id}
Authorization: Bearer {token}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "abuse_report": {
      "incident_id": "WIA-INC-2025-005678",
      "status": "under_investigation",
      "incident_type": "neglect",
      "severity_level": "moderate",
      "investigation": {
        "assigned_to": "Officer Maria Garcia",
        "case_number": "SFACC-INV-2025-1115-001"
      }
    }
  }
}
```

#### 3.5.3 Update Investigation Status

```http
PATCH /abuse/reports/{incident_id}/investigation
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "status": "charges_filed",
  "actions_taken": [
    {
      "action_date": "2025-11-20T10:00:00Z",
      "action": "charges_filed",
      "description": "Criminal charges filed for animal neglect"
    }
  ],
  "findings": "Evidence of long-term neglect confirmed"
}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "incident_id": "WIA-INC-2025-005678",
    "status": "charges_filed",
    "updated_at": "2025-11-20T10:00:00Z"
  }
}
```

### 3.6 Welfare Assessment Endpoints

#### 3.6.1 Create Welfare Assessment

```http
POST /animals/{animal_id}/welfare-assessments
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "assessment_date": "2025-12-18T14:00:00Z",
  "assessed_by": "Dr. Sarah Johnson, DVM",
  "protocol_version": "WIA-WELFARE-ASSESSMENT-v1.0",
  "scores": {
    "physical_health": {
      "score": 90,
      "observations": "Excellent body condition"
    },
    "mental_wellbeing": {
      "score": 85,
      "observations": "Alert and engaged"
    },
    "social_behavior": {
      "score": 88,
      "observations": "Friendly with people and dogs"
    },
    "environmental_enrichment": {
      "score": 82,
      "observations": "Good housing, adequate exercise"
    },
    "care_standards": {
      "score": 90,
      "observations": "Excellent care from staff"
    }
  }
}
```

**Response** (201 Created):
```json
{
  "success": true,
  "data": {
    "assessment_id": "ASSESS-2025-001",
    "animal_id": "WIA-PET-2025-000001",
    "overall_score": 87,
    "assessed_at": "2025-12-18T14:00:00Z",
    "next_assessment_due": "2026-01-18T14:00:00Z"
  }
}
```

#### 3.6.2 Get Welfare Assessment History

```http
GET /animals/{animal_id}/welfare-assessments
Authorization: Bearer {token}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "assessments": [
      {
        "assessment_id": "ASSESS-2025-001",
        "assessment_date": "2025-12-18T14:00:00Z",
        "overall_score": 87,
        "assessed_by": "Dr. Sarah Johnson, DVM"
      },
      {
        "assessment_id": "ASSESS-2025-002",
        "assessment_date": "2025-11-18T14:00:00Z",
        "overall_score": 85,
        "assessed_by": "Dr. Sarah Johnson, DVM"
      }
    ],
    "score_trend": "improving"
  }
}
```

### 3.7 Organization Management Endpoints

#### 3.7.1 Register Organization

```http
POST /organizations
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "basic_info": {
    "name": "Happy Paws Animal Shelter",
    "organization_type": "nonprofit_shelter",
    "country_code": "US"
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
    "email": "info@happypaws.org"
  },
  "operations": {
    "services_provided": [
      "animal_shelter",
      "adoption_services"
    ]
  }
}
```

**Response** (201 Created):
```json
{
  "success": true,
  "data": {
    "organization_id": "WIA-ORG-US-001234",
    "created_at": "2025-01-01T10:00:00Z",
    "verification_status": "pending"
  }
}
```

#### 3.7.2 Get Organization Profile

```http
GET /organizations/{organization_id}
Authorization: Bearer {token}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "organization_profile": {
      "organization_id": "WIA-ORG-US-001234",
      "basic_info": {
        "name": "Happy Paws Animal Shelter",
        "organization_type": "nonprofit_shelter"
      },
      "capacity": {
        "total_capacity": 150,
        "current_occupancy": 127
      }
    }
  }
}
```

#### 3.7.3 Update Organization Capacity

```http
PATCH /organizations/{organization_id}/capacity
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "current_occupancy": 128,
  "dogs": {
    "current": 69
  },
  "cats": {
    "current": 52
  }
}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "organization_id": "WIA-ORG-US-001234",
    "capacity": {
      "total_capacity": 150,
      "current_occupancy": 128,
      "occupancy_rate": 85.3
    },
    "updated_at": "2025-12-18T15:00:00Z"
  }
}
```

### 3.8 Shelter Intake Endpoints

#### 3.8.1 Create Intake Record

```http
POST /shelters/{organization_id}/intakes
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "animal_id": "WIA-PET-2025-000001",
  "intake_date": "2025-03-15T10:00:00Z",
  "intake_type": "stray",
  "intake_officer": {
    "name": "Jessica Martinez",
    "employee_id": "EMP-001"
  },
  "source_information": {
    "brought_by": "animal_control",
    "pickup_location": {
      "address": "Golden Gate Park, San Francisco, CA"
    }
  }
}
```

**Response** (201 Created):
```json
{
  "success": true,
  "data": {
    "intake_id": "INTAKE-2025-12345",
    "animal_id": "WIA-PET-2025-000001",
    "organization_id": "WIA-ORG-US-001234",
    "intake_date": "2025-03-15T10:00:00Z"
  }
}
```

### 3.9 Statistics and Reporting Endpoints

#### 3.9.1 Get Organization Statistics

```http
GET /organizations/{organization_id}/statistics
Authorization: Bearer {token}
```

**Query Parameters**:

| Parameter | Type | Description |
|-----------|------|-------------|
| `start_date` | timestamp | Statistics start date |
| `end_date` | timestamp | Statistics end date |
| `period` | string | Aggregation period (day, week, month, year) |

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "period": {
      "start_date": "2025-01-01T00:00:00Z",
      "end_date": "2025-12-18T23:59:59Z"
    },
    "statistics": {
      "total_intakes": 450,
      "total_adoptions": 380,
      "current_population": 127,
      "adoption_rate": 84.4,
      "average_stay_days": 28,
      "welfare_score_average": 86,
      "species_breakdown": {
        "dogs": 68,
        "cats": 52,
        "other": 7
      },
      "intake_sources": {
        "stray": 180,
        "owner_surrender": 120,
        "transfer": 90,
        "seized": 60
      }
    }
  }
}
```

#### 3.9.2 Get Global Welfare Statistics

```http
GET /statistics/global-welfare
Authorization: Bearer {token}
```

**Query Parameters**:

| Parameter | Type | Description |
|-----------|------|-------------|
| `country_code` | string | Filter by country |
| `region` | string | Filter by region |
| `timeframe` | string | Time period (month, quarter, year) |

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "global_statistics": {
      "total_animals_tracked": 1250000,
      "countries_participating": 45,
      "organizations_registered": 8500,
      "average_welfare_score": 78,
      "cross_border_adoptions": 12500,
      "abuse_reports": {
        "total": 8900,
        "resolved": 7200,
        "under_investigation": 1500,
        "pending": 200
      },
      "top_welfare_countries": [
        {
          "country_code": "NL",
          "average_score": 92,
          "rank": 1
        },
        {
          "country_code": "CH",
          "average_score": 91,
          "rank": 2
        }
      ]
    }
  }
}
```

### 3.10 Microchip Registry Endpoints

#### 3.10.1 Register Microchip

```http
POST /microchips/register
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body**:
```json
{
  "microchip_id": "900123456789012",
  "animal_id": "WIA-PET-2025-000001",
  "implant_date": "2024-01-10T10:00:00Z",
  "implant_location": "between_shoulder_blades",
  "veterinarian": {
    "name": "Dr. Michael Chen",
    "license_number": "CA-VET-67890"
  },
  "owner": {
    "name": "Emily Wilson",
    "phone": "+44-20-7946-1234",
    "email": "emily.wilson@email.com"
  }
}
```

**Response** (201 Created):
```json
{
  "success": true,
  "data": {
    "microchip_id": "900123456789012",
    "registration_status": "active",
    "registered_at": "2025-12-18T10:00:00Z"
  }
}
```

#### 3.10.2 Lookup Microchip

```http
GET /microchips/{microchip_id}
Authorization: Bearer {token}
```

**Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "microchip_id": "900123456789012",
    "animal_id": "WIA-PET-2025-000001",
    "registration_status": "active",
    "animal_info": {
      "name": "Max",
      "species": "canis_lupus_familiaris",
      "breed": "Golden Retriever"
    },
    "owner": {
      "name": "Emily Wilson",
      "contact_allowed": true
    }
  }
}
```

---

## Request/Response Format

### 4.1 Standard Request Headers

| Header | Required | Description |
|--------|----------|-------------|
| `Authorization` | Yes | Bearer token or API key |
| `Content-Type` | Yes (POST/PATCH) | application/json |
| `X-WIA-Organization-ID` | Conditional | Organization context |
| `X-WIA-Request-ID` | Optional | Idempotency key |
| `Accept-Language` | Optional | Preferred language (en, ko, es, etc.) |

### 4.2 Standard Response Format

**Success Response**:
```json
{
  "success": true,
  "data": {},
  "meta": {
    "request_id": "req_abc123",
    "timestamp": "2025-12-18T10:00:00Z",
    "version": "1.0.0"
  },
  "links": {
    "self": "/resource/id"
  }
}
```

**Error Response**:
```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid microchip ID format",
    "details": [
      {
        "field": "microchip_id",
        "issue": "Must be 15 digits"
      }
    ]
  },
  "meta": {
    "request_id": "req_abc123",
    "timestamp": "2025-12-18T10:00:00Z"
  }
}
```

### 4.3 Pagination Format

```json
{
  "data": [],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total_results": 156,
    "total_pages": 8
  },
  "links": {
    "self": "/resource?page=1",
    "first": "/resource?page=1",
    "prev": null,
    "next": "/resource?page=2",
    "last": "/resource?page=8"
  }
}
```

---

## Error Handling

### 5.1 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET/PATCH request |
| 201 | Created | Successful POST request |
| 204 | No Content | Successful DELETE request |
| 400 | Bad Request | Invalid request data |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | Duplicate or conflicting data |
| 422 | Unprocessable Entity | Validation error |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Temporary outage |

### 5.2 Error Codes

| Code | Description |
|------|-------------|
| `AUTHENTICATION_FAILED` | Invalid credentials |
| `AUTHORIZATION_FAILED` | Insufficient permissions |
| `VALIDATION_ERROR` | Request data validation failed |
| `RESOURCE_NOT_FOUND` | Requested resource doesn't exist |
| `DUPLICATE_RESOURCE` | Resource already exists |
| `RATE_LIMIT_EXCEEDED` | Too many requests |
| `MICROCHIP_ALREADY_REGISTERED` | Microchip already in system |
| `ANIMAL_NOT_AVAILABLE` | Animal not available for adoption |
| `INVALID_HEALTH_CERTIFICATE` | Health certificate validation failed |
| `TRANSPORT_PERMISSION_DENIED` | Cannot transport to destination country |

### 5.3 Error Response Examples

**Validation Error**:
```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Request validation failed",
    "details": [
      {
        "field": "date_of_birth",
        "issue": "Date cannot be in the future",
        "value": "2026-01-01"
      }
    ]
  }
}
```

**Rate Limit Error**:
```json
{
  "success": false,
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit of 1000 requests per hour exceeded",
    "retry_after": 1800
  }
}
```

---

## Rate Limiting

### 6.1 Rate Limit Tiers

| Tier | Requests/Hour | Requests/Day | Burst |
|------|---------------|--------------|-------|
| Free | 100 | 1,000 | 10 |
| Basic | 1,000 | 10,000 | 50 |
| Professional | 10,000 | 100,000 | 200 |
| Enterprise | Unlimited | Unlimited | 1,000 |

### 6.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1703001600
X-RateLimit-Retry-After: 1800
```

---

## Webhooks

### 7.1 Webhook Events

| Event | Trigger |
|-------|---------|
| `animal.created` | New animal profile created |
| `animal.updated` | Animal profile updated |
| `animal.adopted` | Animal adopted |
| `adoption.application.submitted` | New adoption application |
| `adoption.application.approved` | Adoption approved |
| `transport.status.updated` | Transport status changed |
| `abuse.report.submitted` | New abuse report |
| `welfare.assessment.completed` | Welfare assessment done |
| `health.vaccination.added` | Vaccination recorded |

### 7.2 Webhook Configuration

```http
POST /webhooks
Authorization: Bearer {token}
Content-Type: application/json
```

**Request**:
```json
{
  "url": "https://your-server.com/webhooks/wia",
  "events": [
    "animal.adopted",
    "abuse.report.submitted"
  ],
  "secret": "your_webhook_secret"
}
```

**Response**:
```json
{
  "success": true,
  "data": {
    "webhook_id": "wh_abc123",
    "url": "https://your-server.com/webhooks/wia",
    "events": ["animal.adopted", "abuse.report.submitted"],
    "created_at": "2025-12-18T10:00:00Z"
  }
}
```

### 7.3 Webhook Payload

```json
{
  "event": "animal.adopted",
  "timestamp": "2025-12-20T10:00:00Z",
  "data": {
    "animal_id": "WIA-PET-2025-000001",
    "adoption_contract_id": "ADOPT-2025-001",
    "adopter": {
      "name": "Emily Wilson",
      "country_code": "GB"
    }
  },
  "signature": "sha256_signature_here"
}
```

---

## Code Examples

### Example 1: Create Animal Profile (Python)

```python
import requests
import json

API_BASE = "https://api.wia.org/pet-welfare-global/v1"
API_KEY = "your_api_key_here"

headers = {
    "Authorization": f"Bearer {API_KEY}",
    "Content-Type": "application/json",
    "X-WIA-Organization-ID": "WIA-ORG-US-001234"
}

animal_data = {
    "identifiers": {
        "microchip_id": "900123456789012"
    },
    "basic_info": {
        "name": "Max",
        "species": "canis_lupus_familiaris",
        "breed": "Golden Retriever",
        "sex": "male",
        "date_of_birth": "2023-06-15"
    },
    "current_status": {
        "status": "shelter_care",
        "organization_id": "WIA-ORG-US-001234"
    }
}

response = requests.post(
    f"{API_BASE}/animals",
    headers=headers,
    json=animal_data
)

if response.status_code == 201:
    animal = response.json()
    print(f"Animal created: {animal['data']['wia_id']}")
else:
    print(f"Error: {response.json()['error']['message']}")
```

### Example 2: Search Available Animals (JavaScript)

```javascript
const API_BASE = 'https://api.wia.org/pet-welfare-global/v1';
const API_KEY = 'your_api_key_here';

async function searchAnimals(filters) {
  const params = new URLSearchParams(filters);

  const response = await fetch(`${API_BASE}/animals/search?${params}`, {
    method: 'GET',
    headers: {
      'Authorization': `Bearer ${API_KEY}`,
      'Content-Type': 'application/json'
    }
  });

  if (!response.ok) {
    throw new Error(`HTTP error! status: ${response.status}`);
  }

  const data = await response.json();
  return data.data.animals;
}

// Search for adoptable dogs
searchAnimals({
  species: 'canis_lupus_familiaris',
  status: 'shelter_care',
  available_for_adoption: true,
  page: 1,
  limit: 20
}).then(animals => {
  console.log(`Found ${animals.length} adoptable dogs`);
  animals.forEach(animal => {
    console.log(`- ${animal.name} (${animal.breed}): Score ${animal.welfare_score}`);
  });
});
```

### Example 3: Submit Abuse Report (cURL)

```bash
curl -X POST https://api.wia.org/pet-welfare-global/v1/abuse/reports \
  -H "Authorization: Bearer your_api_key_here" \
  -H "Content-Type: application/json" \
  -d '{
    "incident_date": "2025-11-15T10:00:00Z",
    "incident_type": "neglect",
    "severity_level": "moderate",
    "reporter": {
      "reporter_type": "citizen",
      "anonymous": false,
      "name": "John Smith",
      "contact": {
        "phone": "+1-415-555-9876"
      }
    },
    "location": {
      "address": "456 Oak Street, San Francisco, CA 94102",
      "country_code": "US"
    },
    "incident_details": {
      "description": "Dog found chained outside without shelter",
      "visible_injuries": true
    }
  }'
```

### Example 4: Process International Adoption (Ruby)

```ruby
require 'net/http'
require 'json'

class WIAPetWelfareAPI
  BASE_URL = 'https://api.wia.org/pet-welfare-global/v1'

  def initialize(api_key, org_id)
    @api_key = api_key
    @org_id = org_id
  end

  def create_transport_manifest(animal_id, destination_country)
    uri = URI("#{BASE_URL}/transport/manifests")

    request = Net::HTTP::Post.new(uri)
    request['Authorization'] = "Bearer #{@api_key}"
    request['Content-Type'] = 'application/json'
    request['X-WIA-Organization-ID'] = @org_id

    request.body = {
      animal_id: animal_id,
      transport_type: 'international_adoption',
      origin: {
        country_code: 'US',
        organization_id: @org_id,
        departure_date: (Time.now + 2*24*60*60).iso8601
      },
      destination: {
        country_code: destination_country,
        expected_arrival: (Time.now + 3*24*60*60).iso8601
      }
    }.to_json

    response = Net::HTTP.start(uri.hostname, uri.port, use_ssl: true) do |http|
      http.request(request)
    end

    JSON.parse(response.body)
  end
end

# Usage
api = WIAPetWelfareAPI.new('your_api_key', 'WIA-ORG-US-001234')
result = api.create_transport_manifest('WIA-PET-2025-000001', 'GB')
puts "Transport created: #{result['data']['transport_id']}"
```

### Example 5: Webhook Handler (Node.js/Express)

```javascript
const express = require('express');
const crypto = require('crypto');

const app = express();
app.use(express.json());

const WEBHOOK_SECRET = 'your_webhook_secret';

function verifyWebhookSignature(payload, signature) {
  const hmac = crypto.createHmac('sha256', WEBHOOK_SECRET);
  const digest = hmac.update(JSON.stringify(payload)).digest('hex');
  return crypto.timingSafeEqual(
    Buffer.from(signature),
    Buffer.from(digest)
  );
}

app.post('/webhooks/wia', (req, res) => {
  const signature = req.headers['x-wia-signature'];

  if (!verifyWebhookSignature(req.body, signature)) {
    return res.status(401).send('Invalid signature');
  }

  const { event, data } = req.body;

  switch (event) {
    case 'animal.adopted':
      console.log(`Animal ${data.animal_id} was adopted!`);
      // Send notification to staff
      break;

    case 'abuse.report.submitted':
      console.log(`New abuse report: ${data.incident_id}`);
      // Alert investigation team
      break;

    case 'transport.status.updated':
      console.log(`Transport ${data.transport_id}: ${data.status}`);
      // Update tracking system
      break;
  }

  res.status(200).send('Webhook received');
});

app.listen(3000, () => {
  console.log('Webhook server listening on port 3000');
});
```

### Example 6: Batch Animal Updates (Go)

```go
package main

import (
    "bytes"
    "encoding/json"
    "fmt"
    "net/http"
    "time"
)

const (
    APIBase = "https://api.wia.org/pet-welfare-global/v1"
    APIKey  = "your_api_key_here"
)

type WelfareAssessment struct {
    AssessmentDate string `json:"assessment_date"`
    AssessedBy     string `json:"assessed_by"`
    Scores         struct {
        PhysicalHealth struct {
            Score        int    `json:"score"`
            Observations string `json:"observations"`
        } `json:"physical_health"`
        MentalWellbeing struct {
            Score        int    `json:"score"`
            Observations string `json:"observations"`
        } `json:"mental_wellbeing"`
    } `json:"scores"`
}

func createWelfareAssessment(animalID string, assessment WelfareAssessment) error {
    url := fmt.Sprintf("%s/animals/%s/welfare-assessments", APIBase, animalID)

    jsonData, err := json.Marshal(assessment)
    if err != nil {
        return err
    }

    req, err := http.NewRequest("POST", url, bytes.NewBuffer(jsonData))
    if err != nil {
        return err
    }

    req.Header.Set("Authorization", "Bearer "+APIKey)
    req.Header.Set("Content-Type", "application/json")

    client := &http.Client{Timeout: 10 * time.Second}
    resp, err := client.Do(req)
    if err != nil {
        return err
    }
    defer resp.Body.Close()

    if resp.StatusCode != 201 {
        return fmt.Errorf("API error: %d", resp.StatusCode)
    }

    fmt.Printf("Assessment created for animal %s\n", animalID)
    return nil
}

func main() {
    animalIDs := []string{
        "WIA-PET-2025-000001",
        "WIA-PET-2025-000002",
        "WIA-PET-2025-000003",
    }

    for _, animalID := range animalIDs {
        assessment := WelfareAssessment{
            AssessmentDate: time.Now().Format(time.RFC3339),
            AssessedBy:     "Dr. Sarah Johnson, DVM",
        }
        assessment.Scores.PhysicalHealth.Score = 90
        assessment.Scores.PhysicalHealth.Observations = "Excellent condition"

        if err := createWelfareAssessment(animalID, assessment); err != nil {
            fmt.Printf("Error: %v\n", err)
        }
    }
}
```

---

## API Reference

### 9.1 Complete Endpoint Summary

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/animals` | POST | Create animal profile |
| `/animals/{id}` | GET | Get animal profile |
| `/animals/{id}` | PATCH | Update animal profile |
| `/animals/{id}` | DELETE | Delete animal profile |
| `/animals/search` | GET | Search animals |
| `/animals/{id}/health-passport` | POST | Create health passport |
| `/animals/{id}/health-passport` | GET | Get health passport |
| `/animals/{id}/health-passport/vaccinations` | POST | Add vaccination |
| `/animals/{id}/welfare-assessments` | POST | Create welfare assessment |
| `/animals/{id}/welfare-assessments` | GET | Get assessment history |
| `/adoptions/applications` | POST | Submit adoption application |
| `/adoptions/applications/{id}` | GET | Get adoption application |
| `/adoptions/applications/{id}/approve` | POST | Approve adoption |
| `/transport/manifests` | POST | Create transport manifest |
| `/transport/manifests/{id}` | GET | Get transport manifest |
| `/transport/manifests/{id}/status` | PATCH | Update transport status |
| `/abuse/reports` | POST | Submit abuse report |
| `/abuse/reports/{id}` | GET | Get abuse report |
| `/abuse/reports/{id}/investigation` | PATCH | Update investigation |
| `/organizations` | POST | Register organization |
| `/organizations/{id}` | GET | Get organization profile |
| `/organizations/{id}/capacity` | PATCH | Update capacity |
| `/organizations/{id}/statistics` | GET | Get organization stats |
| `/shelters/{id}/intakes` | POST | Create intake record |
| `/microchips/register` | POST | Register microchip |
| `/microchips/{id}` | GET | Lookup microchip |
| `/statistics/global-welfare` | GET | Get global statistics |
| `/webhooks` | POST | Configure webhook |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
