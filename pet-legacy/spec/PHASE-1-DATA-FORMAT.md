# WIA PET-LEGACY PHASE 1: DATA FORMAT SPECIFICATION

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-18
**Primary Color:** #F59E0B (Amber - PET Series)

---

## Table of Contents

1. [Overview](#overview)
2. [Core Data Models](#core-data-models)
3. [Memorial Profile Schema](#memorial-profile-schema)
4. [Media Asset Schema](#media-asset-schema)
5. [Timeline Event Schema](#timeline-event-schema)
6. [Family Member Schema](#family-member-schema)
7. [Data Validation Rules](#data-validation-rules)
8. [File Format Specifications](#file-format-specifications)
9. [Metadata Standards](#metadata-standards)
10. [Security and Privacy](#security-and-privacy)

---

## 1. Overview

### 1.1 Purpose

The WIA PET-LEGACY Phase 1 specification defines standardized data formats for digital memorial and legacy preservation of deceased pets. This standard ensures interoperability between memorial platforms, veterinary systems, and cemetery/crematorium services while preserving the dignity and memory of beloved companion animals.

### 1.2 Scope

This specification covers:
- Pet memorial profile data structures
- Media asset management and metadata
- Timeline and life history documentation
- Family member access and relationship modeling
- Grief support resource integration
- Cross-platform data exchange formats

### 1.3 Key Principles

| Principle | Description | Implementation |
|-----------|-------------|----------------|
| **Dignity** | Treat pet memories with respect and reverence | Enforce content policies, quality standards |
| **Permanence** | Ensure long-term data preservation | Use stable formats, versioning, migration paths |
| **Privacy** | Protect sensitive family information | Encryption, access controls, consent management |
| **Accessibility** | Enable universal access to memories | Multi-format support, assistive technology compatibility |
| **Interoperability** | Allow seamless data exchange | Standard JSON schemas, API compatibility |

### 1.4 Terminology

| Term | Definition |
|------|------------|
| **Memorial Profile** | Complete digital representation of a deceased pet's life and legacy |
| **Guardian** | Primary owner or caretaker responsible for the memorial |
| **Contributor** | Family member or friend with permission to add memories |
| **Timeline Event** | Significant moment in the pet's life with associated media |
| **Memorial Asset** | Photo, video, document, or other media preserving memories |
| **Legacy Package** | Exportable archive of all memorial data and assets |

---

## 2. Core Data Models

### 2.1 Data Model Hierarchy

```
Memorial Profile (Root)
├── Pet Identity
│   ├── Basic Information
│   ├── Physical Characteristics
│   └── Identification Records
├── Life Timeline
│   ├── Birth/Adoption Events
│   ├── Milestone Events
│   ├── Medical Events
│   └── Passing Information
├── Media Library
│   ├── Photos
│   ├── Videos
│   ├── Audio Recordings
│   └── Documents
├── Stories & Memories
│   ├── Written Tributes
│   ├── Favorite Moments
│   └── Personality Traits
├── Family Network
│   ├── Guardians
│   ├── Contributors
│   └── Viewers
└── Memorial Settings
    ├── Privacy Controls
    ├── Sharing Preferences
    └── Display Customization
```

### 2.2 Entity Relationship Diagram

| Entity | Relationships | Cardinality |
|--------|---------------|-------------|
| Memorial Profile | has many Timeline Events | 1:N |
| Memorial Profile | has many Media Assets | 1:N |
| Memorial Profile | has many Family Members | 1:N |
| Memorial Profile | has one Guardian | 1:1 |
| Timeline Event | has many Media Assets | 1:N |
| Media Asset | belongs to many Timeline Events | N:N |
| Family Member | creates many Timeline Events | 1:N |
| Family Member | uploads many Media Assets | 1:N |

### 2.3 Data Storage Requirements

| Data Type | Min Size | Max Size | Retention | Backup Frequency |
|-----------|----------|----------|-----------|------------------|
| Memorial Profile | 1 KB | 100 MB | Perpetual | Daily |
| Photo Asset | 100 KB | 50 MB | Perpetual | Daily |
| Video Asset | 1 MB | 500 MB | Perpetual | Daily |
| Audio Asset | 100 KB | 100 MB | Perpetual | Daily |
| Document Asset | 10 KB | 25 MB | Perpetual | Daily |
| Timeline Event | 500 B | 10 KB | Perpetual | Daily |

---

## 3. Memorial Profile Schema

### 3.1 Complete Memorial Profile JSON Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "PetMemorialProfile",
  "type": "object",
  "required": ["profileId", "petIdentity", "guardianInfo", "memorialStatus"],
  "properties": {
    "profileId": {
      "type": "string",
      "format": "uuid",
      "description": "Unique identifier for the memorial profile"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "default": "1.0.0",
      "description": "Schema version for data migration"
    },
    "createdAt": {
      "type": "string",
      "format": "date-time",
      "description": "Timestamp when memorial was created"
    },
    "lastModified": {
      "type": "string",
      "format": "date-time",
      "description": "Timestamp of last modification"
    },
    "petIdentity": {
      "type": "object",
      "required": ["name", "species"],
      "properties": {
        "name": {
          "type": "string",
          "minLength": 1,
          "maxLength": 100,
          "description": "Pet's name"
        },
        "nickname": {
          "type": "array",
          "items": {"type": "string"},
          "description": "Alternative names or nicknames"
        },
        "species": {
          "type": "string",
          "enum": ["dog", "cat", "bird", "rabbit", "hamster", "guinea_pig", "reptile", "fish", "horse", "other"],
          "description": "Species classification"
        },
        "breed": {
          "type": "string",
          "maxLength": 100,
          "description": "Breed or species variant"
        },
        "gender": {
          "type": "string",
          "enum": ["male", "female", "unknown"],
          "description": "Biological gender"
        },
        "birthDate": {
          "type": "string",
          "format": "date",
          "description": "Date of birth or estimated birth date"
        },
        "birthDateEstimated": {
          "type": "boolean",
          "default": false,
          "description": "Whether birth date is an estimate"
        },
        "adoptionDate": {
          "type": "string",
          "format": "date",
          "description": "Date adopted by guardian"
        },
        "passingDate": {
          "type": "string",
          "format": "date",
          "description": "Date of passing"
        },
        "ageAtPassing": {
          "type": "object",
          "properties": {
            "years": {"type": "integer", "minimum": 0},
            "months": {"type": "integer", "minimum": 0, "maximum": 11},
            "days": {"type": "integer", "minimum": 0, "maximum": 30}
          }
        },
        "microchipId": {
          "type": "string",
          "pattern": "^[0-9]{15}$",
          "description": "15-digit microchip identification number"
        },
        "registrationIds": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "type": {"type": "string"},
              "number": {"type": "string"},
              "organization": {"type": "string"}
            }
          }
        }
      }
    },
    "physicalCharacteristics": {
      "type": "object",
      "properties": {
        "furColor": {
          "type": "array",
          "items": {"type": "string"},
          "description": "Primary and secondary fur/feather/scale colors"
        },
        "eyeColor": {
          "type": "string",
          "description": "Eye color description"
        },
        "weight": {
          "type": "object",
          "properties": {
            "value": {"type": "number", "minimum": 0},
            "unit": {"type": "string", "enum": ["kg", "lb", "g", "oz"]}
          }
        },
        "height": {
          "type": "object",
          "properties": {
            "value": {"type": "number", "minimum": 0},
            "unit": {"type": "string", "enum": ["cm", "in", "m"]}
          }
        },
        "distinguishingMarks": {
          "type": "array",
          "items": {"type": "string"},
          "description": "Unique physical features or markings"
        }
      }
    },
    "personality": {
      "type": "object",
      "properties": {
        "traits": {
          "type": "array",
          "items": {"type": "string"},
          "description": "Personality characteristics"
        },
        "favoriteActivities": {
          "type": "array",
          "items": {"type": "string"}
        },
        "favoriteToys": {
          "type": "array",
          "items": {"type": "string"}
        },
        "favoriteFoods": {
          "type": "array",
          "items": {"type": "string"}
        },
        "quirks": {
          "type": "array",
          "items": {"type": "string"},
          "description": "Unique behaviors or habits"
        }
      }
    },
    "guardianInfo": {
      "type": "object",
      "required": ["userId", "role"],
      "properties": {
        "userId": {
          "type": "string",
          "format": "uuid",
          "description": "Primary guardian user ID"
        },
        "name": {
          "type": "string",
          "description": "Guardian name"
        },
        "role": {
          "type": "string",
          "enum": ["owner", "guardian", "caretaker"],
          "default": "owner"
        },
        "relationship": {
          "type": "string",
          "description": "Description of relationship with pet"
        },
        "contactEmail": {
          "type": "string",
          "format": "email"
        }
      }
    },
    "memorialStatus": {
      "type": "object",
      "required": ["isPublic", "status"],
      "properties": {
        "status": {
          "type": "string",
          "enum": ["draft", "active", "archived", "private"],
          "default": "draft"
        },
        "isPublic": {
          "type": "boolean",
          "default": false,
          "description": "Whether memorial is publicly viewable"
        },
        "publishedAt": {
          "type": "string",
          "format": "date-time"
        },
        "viewCount": {
          "type": "integer",
          "minimum": 0,
          "default": 0
        },
        "tributeCount": {
          "type": "integer",
          "minimum": 0,
          "default": 0
        }
      }
    },
    "passingInformation": {
      "type": "object",
      "properties": {
        "cause": {
          "type": "string",
          "description": "Cause of passing (if guardian chooses to share)"
        },
        "location": {
          "type": "string",
          "description": "Location where pet passed"
        },
        "wasEuthanized": {
          "type": "boolean"
        },
        "veterinarianInfo": {
          "type": "object",
          "properties": {
            "clinicName": {"type": "string"},
            "veterinarianName": {"type": "string"},
            "contactInfo": {"type": "string"}
          }
        },
        "funeralArrangements": {
          "type": "object",
          "properties": {
            "type": {
              "type": "string",
              "enum": ["burial", "cremation", "other", "none"]
            },
            "location": {"type": "string"},
            "providerName": {"type": "string"},
            "providerContact": {"type": "string"},
            "ceremonyDate": {"type": "string", "format": "date-time"},
            "plotNumber": {"type": "string"},
            "urnDescription": {"type": "string"}
          }
        }
      }
    },
    "memorialCustomization": {
      "type": "object",
      "properties": {
        "themeColor": {
          "type": "string",
          "pattern": "^#[0-9A-Fa-f]{6}$",
          "default": "#F59E0B"
        },
        "coverPhoto": {
          "type": "string",
          "format": "uri",
          "description": "URL to cover photo"
        },
        "profilePhoto": {
          "type": "string",
          "format": "uri",
          "description": "URL to main profile photo"
        },
        "epitaph": {
          "type": "string",
          "maxLength": 500,
          "description": "Memorial inscription or tribute message"
        },
        "musicUrl": {
          "type": "string",
          "format": "uri",
          "description": "Background music for memorial page"
        },
        "displayLayout": {
          "type": "string",
          "enum": ["timeline", "gallery", "story", "mixed"],
          "default": "mixed"
        }
      }
    },
    "statistics": {
      "type": "object",
      "properties": {
        "totalPhotos": {"type": "integer", "minimum": 0},
        "totalVideos": {"type": "integer", "minimum": 0},
        "totalStories": {"type": "integer", "minimum": 0},
        "totalTimelineEvents": {"type": "integer", "minimum": 0},
        "totalContributors": {"type": "integer", "minimum": 0},
        "lastActivityDate": {"type": "string", "format": "date-time"}
      }
    }
  }
}
```

### 3.2 Example Memorial Profile

```json
{
  "profileId": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.0",
  "createdAt": "2024-03-15T10:30:00Z",
  "lastModified": "2024-12-18T14:22:00Z",
  "petIdentity": {
    "name": "Max",
    "nickname": ["Maxie", "Maxy Boy"],
    "species": "dog",
    "breed": "Golden Retriever",
    "gender": "male",
    "birthDate": "2012-05-20",
    "birthDateEstimated": false,
    "adoptionDate": "2012-07-15",
    "passingDate": "2024-03-10",
    "ageAtPassing": {
      "years": 11,
      "months": 9,
      "days": 20
    },
    "microchipId": "985112345678901",
    "registrationIds": [
      {
        "type": "AKC",
        "number": "DN12345678",
        "organization": "American Kennel Club"
      }
    ]
  },
  "physicalCharacteristics": {
    "furColor": ["golden", "cream"],
    "eyeColor": "brown",
    "weight": {
      "value": 32.5,
      "unit": "kg"
    },
    "height": {
      "value": 58,
      "unit": "cm"
    },
    "distinguishingMarks": ["white patch on chest", "scar on left ear"]
  },
  "personality": {
    "traits": ["loyal", "gentle", "playful", "loving"],
    "favoriteActivities": ["swimming", "fetch", "hiking"],
    "favoriteToys": ["tennis ball", "rope toy"],
    "favoriteFoods": ["chicken treats", "peanut butter"],
    "quirks": ["always greeted at door", "slept upside down"]
  },
  "guardianInfo": {
    "userId": "660e8400-e29b-41d4-a716-446655440001",
    "name": "Sarah Johnson",
    "role": "owner",
    "relationship": "Best friend and companion for 12 years",
    "contactEmail": "sarah.j@example.com"
  },
  "memorialStatus": {
    "status": "active",
    "isPublic": true,
    "publishedAt": "2024-03-15T10:30:00Z",
    "viewCount": 1247,
    "tributeCount": 89
  },
  "passingInformation": {
    "cause": "Natural causes - old age",
    "location": "At home, surrounded by family",
    "wasEuthanized": false,
    "veterinarianInfo": {
      "clinicName": "Compassionate Care Veterinary",
      "veterinarianName": "Dr. Emily Chen",
      "contactInfo": "555-0123"
    },
    "funeralArrangements": {
      "type": "cremation",
      "location": "Rainbow Bridge Pet Memorial",
      "providerName": "Rainbow Bridge Services",
      "providerContact": "555-0456",
      "ceremonyDate": "2024-03-12T14:00:00Z",
      "urnDescription": "Wooden urn with engraved name and dates"
    }
  },
  "memorialCustomization": {
    "themeColor": "#F59E0B",
    "coverPhoto": "https://cdn.example.com/max-cover.jpg",
    "profilePhoto": "https://cdn.example.com/max-profile.jpg",
    "epitaph": "Forever in our hearts. The best friend anyone could ask for.",
    "displayLayout": "mixed"
  },
  "statistics": {
    "totalPhotos": 324,
    "totalVideos": 47,
    "totalStories": 15,
    "totalTimelineEvents": 52,
    "totalContributors": 12,
    "lastActivityDate": "2024-12-18T14:22:00Z"
  }
}
```

---

## 4. Media Asset Schema

### 4.1 Media Asset JSON Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "MediaAsset",
  "type": "object",
  "required": ["assetId", "memorialId", "type", "url", "uploadedBy"],
  "properties": {
    "assetId": {
      "type": "string",
      "format": "uuid"
    },
    "memorialId": {
      "type": "string",
      "format": "uuid",
      "description": "Reference to parent memorial profile"
    },
    "type": {
      "type": "string",
      "enum": ["photo", "video", "audio", "document"]
    },
    "url": {
      "type": "string",
      "format": "uri",
      "description": "CDN URL to the asset"
    },
    "thumbnailUrl": {
      "type": "string",
      "format": "uri"
    },
    "fileName": {
      "type": "string"
    },
    "fileSize": {
      "type": "integer",
      "minimum": 0,
      "description": "File size in bytes"
    },
    "mimeType": {
      "type": "string"
    },
    "uploadedBy": {
      "type": "string",
      "format": "uuid",
      "description": "User ID who uploaded the asset"
    },
    "uploadedAt": {
      "type": "string",
      "format": "date-time"
    },
    "capturedAt": {
      "type": "string",
      "format": "date-time",
      "description": "When photo/video was taken"
    },
    "title": {
      "type": "string",
      "maxLength": 200
    },
    "description": {
      "type": "string",
      "maxLength": 2000
    },
    "tags": {
      "type": "array",
      "items": {"type": "string"}
    },
    "location": {
      "type": "object",
      "properties": {
        "latitude": {"type": "number"},
        "longitude": {"type": "number"},
        "placeName": {"type": "string"}
      }
    },
    "mediaMetadata": {
      "type": "object",
      "properties": {
        "width": {"type": "integer"},
        "height": {"type": "integer"},
        "duration": {"type": "number", "description": "Duration in seconds for video/audio"},
        "bitrate": {"type": "integer"},
        "codec": {"type": "string"},
        "orientation": {"type": "integer"},
        "cameraMake": {"type": "string"},
        "cameraModel": {"type": "string"}
      }
    },
    "processingStatus": {
      "type": "string",
      "enum": ["uploading", "processing", "ready", "failed"],
      "default": "uploading"
    },
    "privacyLevel": {
      "type": "string",
      "enum": ["public", "family_only", "private"],
      "default": "family_only"
    },
    "isFeatured": {
      "type": "boolean",
      "default": false
    },
    "linkedTimelineEvents": {
      "type": "array",
      "items": {"type": "string", "format": "uuid"}
    }
  }
}
```

### 4.2 Supported Media Formats

| Media Type | Supported Formats | Max Size | Recommended Resolution |
|------------|------------------|----------|----------------------|
| Photo | JPEG, PNG, HEIC, WebP | 50 MB | 4096x4096 px |
| Video | MP4, MOV, AVI, WebM | 500 MB | 1920x1080 (1080p) |
| Audio | MP3, WAV, AAC, OGG | 100 MB | 320 kbps |
| Document | PDF, DOCX, TXT | 25 MB | N/A |

### 4.3 Example Photo Asset

```json
{
  "assetId": "770e8400-e29b-41d4-a716-446655440002",
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "photo",
  "url": "https://cdn.example.com/assets/max-birthday-2020.jpg",
  "thumbnailUrl": "https://cdn.example.com/thumbs/max-birthday-2020-thumb.jpg",
  "fileName": "max-birthday-2020.jpg",
  "fileSize": 3457280,
  "mimeType": "image/jpeg",
  "uploadedBy": "660e8400-e29b-41d4-a716-446655440001",
  "uploadedAt": "2024-03-15T11:00:00Z",
  "capturedAt": "2020-05-20T15:30:00Z",
  "title": "Max's 8th Birthday Celebration",
  "description": "Max enjoying his birthday cake made of dog treats. He was so happy!",
  "tags": ["birthday", "celebration", "cake", "happy"],
  "location": {
    "placeName": "Our backyard"
  },
  "mediaMetadata": {
    "width": 4032,
    "height": 3024,
    "orientation": 1,
    "cameraMake": "Apple",
    "cameraModel": "iPhone 12"
  },
  "processingStatus": "ready",
  "privacyLevel": "public",
  "isFeatured": true,
  "linkedTimelineEvents": ["880e8400-e29b-41d4-a716-446655440003"]
}
```

---

## 5. Timeline Event Schema

### 5.1 Timeline Event JSON Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "TimelineEvent",
  "type": "object",
  "required": ["eventId", "memorialId", "eventType", "date", "title"],
  "properties": {
    "eventId": {
      "type": "string",
      "format": "uuid"
    },
    "memorialId": {
      "type": "string",
      "format": "uuid"
    },
    "eventType": {
      "type": "string",
      "enum": [
        "birth",
        "adoption",
        "medical",
        "milestone",
        "travel",
        "celebration",
        "achievement",
        "loss",
        "memorial_service",
        "other"
      ]
    },
    "date": {
      "type": "string",
      "format": "date-time"
    },
    "dateEstimated": {
      "type": "boolean",
      "default": false
    },
    "title": {
      "type": "string",
      "minLength": 1,
      "maxLength": 200
    },
    "description": {
      "type": "string",
      "maxLength": 5000
    },
    "location": {
      "type": "object",
      "properties": {
        "placeName": {"type": "string"},
        "address": {"type": "string"},
        "city": {"type": "string"},
        "state": {"type": "string"},
        "country": {"type": "string"},
        "coordinates": {
          "type": "object",
          "properties": {
            "latitude": {"type": "number"},
            "longitude": {"type": "number"}
          }
        }
      }
    },
    "attachedMedia": {
      "type": "array",
      "items": {"type": "string", "format": "uuid"},
      "description": "Array of media asset IDs"
    },
    "createdBy": {
      "type": "string",
      "format": "uuid"
    },
    "createdAt": {
      "type": "string",
      "format": "date-time"
    },
    "visibility": {
      "type": "string",
      "enum": ["public", "family_only", "private"],
      "default": "family_only"
    },
    "medicalInfo": {
      "type": "object",
      "properties": {
        "veterinarianName": {"type": "string"},
        "clinicName": {"type": "string"},
        "diagnosis": {"type": "string"},
        "treatment": {"type": "string"},
        "medications": {
          "type": "array",
          "items": {"type": "string"}
        },
        "outcome": {"type": "string"}
      }
    },
    "celebrationInfo": {
      "type": "object",
      "properties": {
        "occasionType": {
          "type": "string",
          "enum": ["birthday", "adoption_anniversary", "holiday", "achievement", "other"]
        },
        "attendees": {
          "type": "array",
          "items": {"type": "string"}
        },
        "gifts": {
          "type": "array",
          "items": {"type": "string"}
        }
      }
    }
  }
}
```

### 5.2 Event Type Classifications

| Event Type | Icon | Color | Typical Usage |
|------------|------|-------|---------------|
| Birth | 🐾 | #10B981 | Pet's birth or estimated birth |
| Adoption | ❤️ | #F59E0B | Day pet joined the family |
| Medical | 🏥 | #EF4444 | Veterinary visits, treatments |
| Milestone | ⭐ | #8B5CF6 | First steps, training achievements |
| Travel | ✈️ | #3B82F6 | Trips, vacations with pet |
| Celebration | 🎉 | #F59E0B | Birthdays, holidays |
| Achievement | 🏆 | #FBBF24 | Awards, certifications |
| Loss | 🕊️ | #6B7280 | Day of passing |
| Memorial Service | 🌹 | #A855F7 | Funeral or memorial ceremony |

### 5.3 Example Timeline Events

```json
[
  {
    "eventId": "880e8400-e29b-41d4-a716-446655440003",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "eventType": "adoption",
    "date": "2012-07-15T10:00:00Z",
    "dateEstimated": false,
    "title": "Welcome Home, Max!",
    "description": "The day we brought Max home from the shelter. He was just 8 weeks old and fit in my arms. From the moment we met, we knew he was meant to be part of our family.",
    "location": {
      "placeName": "City Animal Shelter",
      "city": "Portland",
      "state": "OR",
      "country": "USA"
    },
    "attachedMedia": ["770e8400-e29b-41d4-a716-446655440004", "770e8400-e29b-41d4-a716-446655440005"],
    "createdBy": "660e8400-e29b-41d4-a716-446655440001",
    "createdAt": "2024-03-15T11:30:00Z",
    "visibility": "public"
  },
  {
    "eventId": "880e8400-e29b-41d4-a716-446655440006",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "eventType": "milestone",
    "date": "2013-01-10T14:00:00Z",
    "title": "Passed Obedience Training!",
    "description": "Max completed his beginner obedience course with flying colors. He was the star pupil and loved showing off his new tricks.",
    "attachedMedia": ["770e8400-e29b-41d4-a716-446655440007"],
    "createdBy": "660e8400-e29b-41d4-a716-446655440001",
    "createdAt": "2024-03-15T11:45:00Z",
    "visibility": "public",
    "celebrationInfo": {
      "occasionType": "achievement",
      "gifts": ["graduation certificate", "new collar"]
    }
  }
]
```

---

## 6. Family Member Schema

### 6.1 Family Member JSON Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "FamilyMember",
  "type": "object",
  "required": ["memberId", "memorialId", "userId", "role", "permissions"],
  "properties": {
    "memberId": {
      "type": "string",
      "format": "uuid"
    },
    "memorialId": {
      "type": "string",
      "format": "uuid"
    },
    "userId": {
      "type": "string",
      "format": "uuid"
    },
    "displayName": {
      "type": "string",
      "maxLength": 100
    },
    "relationshipToPet": {
      "type": "string",
      "maxLength": 200,
      "description": "How they knew the pet"
    },
    "role": {
      "type": "string",
      "enum": ["guardian", "contributor", "viewer"],
      "description": "Access level role"
    },
    "permissions": {
      "type": "object",
      "required": ["canView", "canComment", "canUpload", "canEdit"],
      "properties": {
        "canView": {"type": "boolean", "default": true},
        "canComment": {"type": "boolean", "default": false},
        "canUpload": {"type": "boolean", "default": false},
        "canEdit": {"type": "boolean", "default": false},
        "canDelete": {"type": "boolean", "default": false},
        "canManageMembers": {"type": "boolean", "default": false},
        "canManageSettings": {"type": "boolean", "default": false}
      }
    },
    "invitedBy": {
      "type": "string",
      "format": "uuid"
    },
    "invitedAt": {
      "type": "string",
      "format": "date-time"
    },
    "joinedAt": {
      "type": "string",
      "format": "date-time"
    },
    "status": {
      "type": "string",
      "enum": ["invited", "active", "suspended", "removed"],
      "default": "invited"
    },
    "contributionStats": {
      "type": "object",
      "properties": {
        "photosUploaded": {"type": "integer", "minimum": 0},
        "videosUploaded": {"type": "integer", "minimum": 0},
        "storiesWritten": {"type": "integer", "minimum": 0},
        "eventsCreated": {"type": "integer", "minimum": 0},
        "commentsPosted": {"type": "integer", "minimum": 0},
        "lastContribution": {"type": "string", "format": "date-time"}
      }
    }
  }
}
```

### 6.2 Permission Matrix

| Role | View | Comment | Upload Media | Create Events | Edit Memorial | Delete Content | Manage Members | Manage Settings |
|------|------|---------|--------------|---------------|---------------|----------------|----------------|-----------------|
| **Guardian** | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| **Contributor** | ✓ | ✓ | ✓ | ✓ | Own only | Own only | ✗ | ✗ |
| **Viewer** | ✓ | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |

### 6.3 Example Family Members

```json
[
  {
    "memberId": "990e8400-e29b-41d4-a716-446655440008",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "userId": "660e8400-e29b-41d4-a716-446655440001",
    "displayName": "Sarah Johnson",
    "relationshipToPet": "Primary guardian and owner",
    "role": "guardian",
    "permissions": {
      "canView": true,
      "canComment": true,
      "canUpload": true,
      "canEdit": true,
      "canDelete": true,
      "canManageMembers": true,
      "canManageSettings": true
    },
    "joinedAt": "2024-03-15T10:30:00Z",
    "status": "active",
    "contributionStats": {
      "photosUploaded": 198,
      "videosUploaded": 32,
      "storiesWritten": 8,
      "eventsCreated": 35,
      "commentsPosted": 45,
      "lastContribution": "2024-12-18T14:22:00Z"
    }
  },
  {
    "memberId": "990e8400-e29b-41d4-a716-446655440009",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "userId": "660e8400-e29b-41d4-a716-446655440010",
    "displayName": "Michael Johnson",
    "relationshipToPet": "Co-owner, Sarah's husband",
    "role": "contributor",
    "permissions": {
      "canView": true,
      "canComment": true,
      "canUpload": true,
      "canEdit": true,
      "canDelete": false,
      "canManageMembers": false,
      "canManageSettings": false
    },
    "invitedBy": "660e8400-e29b-41d4-a716-446655440001",
    "invitedAt": "2024-03-15T10:35:00Z",
    "joinedAt": "2024-03-15T11:00:00Z",
    "status": "active",
    "contributionStats": {
      "photosUploaded": 126,
      "videosUploaded": 15,
      "storiesWritten": 7,
      "eventsCreated": 17,
      "commentsPosted": 89,
      "lastContribution": "2024-12-15T09:30:00Z"
    }
  }
]
```

---

## 7. Data Validation Rules

### 7.1 Required Field Validation

| Field | Validation Rule | Error Message |
|-------|----------------|---------------|
| Pet Name | Min 1 char, Max 100 chars | "Pet name is required and must be 1-100 characters" |
| Species | Must be from enum list | "Please select a valid species" |
| Birth Date | Must be valid date, not in future | "Birth date must be a valid past date" |
| Passing Date | Must be >= Birth Date | "Passing date cannot be before birth date" |
| Guardian ID | Must be valid UUID, user must exist | "Invalid guardian reference" |
| Media URL | Must be valid HTTPS URL | "Media URL must be a valid secure URL" |
| File Size | Must not exceed type limits | "File size exceeds maximum allowed for this type" |

### 7.2 Data Consistency Rules

```javascript
// Example validation function
function validateMemorialProfile(profile) {
  const errors = [];

  // Date consistency
  if (profile.petIdentity.passingDate && profile.petIdentity.birthDate) {
    const birthDate = new Date(profile.petIdentity.birthDate);
    const passingDate = new Date(profile.petIdentity.passingDate);

    if (passingDate < birthDate) {
      errors.push("Passing date cannot be before birth date");
    }
  }

  // Age calculation verification
  if (profile.petIdentity.ageAtPassing) {
    const calculated = calculateAge(
      profile.petIdentity.birthDate,
      profile.petIdentity.passingDate
    );

    if (!agesMatch(calculated, profile.petIdentity.ageAtPassing)) {
      errors.push("Age at passing does not match dates");
    }
  }

  // Memorial status logic
  if (profile.memorialStatus.status === 'active' && !profile.memorialStatus.publishedAt) {
    errors.push("Active memorials must have a published date");
  }

  // Guardian validation
  if (!profile.guardianInfo || !profile.guardianInfo.userId) {
    errors.push("Memorial must have a designated guardian");
  }

  return errors;
}
```

### 7.3 Content Moderation Rules

| Content Type | Moderation Requirements | Action on Violation |
|--------------|------------------------|---------------------|
| Profile Photos | No offensive content, must contain pet | Flag for review, hide until approved |
| Descriptions | No profanity, spam, or promotional content | Auto-filter, notify user |
| Comments | Respectful, on-topic, no harassment | Remove, warn user, potential ban |
| External Links | Must be HTTPS, no malware/phishing | Block, security scan |
| File Uploads | Virus scan, format validation | Reject upload, notify user |

---

## 8. File Format Specifications

### 8.1 Export Package Format

Memorial profiles can be exported as complete packages for backup or migration:

```json
{
  "exportFormat": "WIA-PET-LEGACY-1.0",
  "exportDate": "2024-12-18T14:22:00Z",
  "exportedBy": "660e8400-e29b-41d4-a716-446655440001",
  "contents": {
    "memorialProfile": "profile.json",
    "timelineEvents": "timeline/",
    "mediaAssets": "media/",
    "familyMembers": "family.json",
    "comments": "comments.json",
    "metadata": "export-metadata.json"
  },
  "totalSize": 2457280000,
  "fileCount": {
    "photos": 324,
    "videos": 47,
    "documents": 12,
    "jsonFiles": 8
  }
}
```

### 8.2 Directory Structure

```
memorial-export-max-20241218/
├── export-manifest.json
├── profile.json
├── family.json
├── comments.json
├── timeline/
│   ├── events.json
│   └── events-by-year/
│       ├── 2012.json
│       ├── 2013.json
│       └── ...
├── media/
│   ├── photos/
│   │   ├── 2012/
│   │   ├── 2013/
│   │   └── ...
│   ├── videos/
│   ├── audio/
│   └── documents/
└── thumbnails/
    ├── photos/
    └── videos/
```

### 8.3 Import Validation Checklist

- [ ] Verify export format version compatibility
- [ ] Validate JSON schema compliance for all files
- [ ] Check file integrity (checksums)
- [ ] Scan media files for corruption
- [ ] Verify all referenced media assets exist
- [ ] Validate user/member references
- [ ] Check date consistency across timeline
- [ ] Verify total size within import limits
- [ ] Ensure no duplicate IDs across system

---

## 9. Metadata Standards

### 9.1 Dublin Core Metadata

All memorial profiles include Dublin Core metadata for archival compatibility:

```json
{
  "dublinCore": {
    "title": "Memorial for Max - Golden Retriever",
    "creator": "Sarah Johnson",
    "subject": ["pet memorial", "dog", "golden retriever", "companion animal"],
    "description": "Digital memorial preserving the life and legacy of Max, beloved Golden Retriever (2012-2024)",
    "publisher": "WIA Pet Legacy Platform",
    "contributor": ["Sarah Johnson", "Michael Johnson", "Emily Chen"],
    "date": "2024-03-15",
    "type": "InteractiveResource",
    "format": "application/json",
    "identifier": "wia:pet-legacy:550e8400-e29b-41d4-a716-446655440000",
    "source": "https://petlegacy.example.com/memorial/550e8400",
    "language": "en",
    "relation": "Part of WIA Pet Legacy Collection",
    "coverage": "2012-2024",
    "rights": "© 2024 Sarah Johnson. Shared under memorial access permissions."
  }
}
```

### 9.2 EXIF Data Preservation

When processing uploaded photos, preserve relevant EXIF data:

```json
{
  "exifData": {
    "DateTimeOriginal": "2020:05:20 15:30:45",
    "Make": "Apple",
    "Model": "iPhone 12",
    "Orientation": 1,
    "XResolution": 72,
    "YResolution": 72,
    "Software": "iOS 14.5",
    "GPSLatitude": 45.523064,
    "GPSLongitude": -122.676483,
    "GPSAltitude": 15.2
  }
}
```

---

## 10. Security and Privacy

### 10.1 Data Encryption Standards

| Data State | Encryption Method | Key Management |
|------------|------------------|----------------|
| At Rest | AES-256 | AWS KMS / Azure Key Vault |
| In Transit | TLS 1.3 | Certificate rotation every 90 days |
| Backups | AES-256 + Compression | Separate key hierarchy |
| Exports | Optional password protection (AES-256) | User-provided passphrase |

### 10.2 Privacy Controls

```json
{
  "privacySettings": {
    "memorialVisibility": {
      "type": "string",
      "enum": ["public", "unlisted", "private"],
      "default": "unlisted"
    },
    "searchEngineIndexing": {
      "type": "boolean",
      "default": false,
      "description": "Allow search engines to index memorial"
    },
    "socialSharing": {
      "type": "boolean",
      "default": true,
      "description": "Enable social media sharing"
    },
    "requireLoginToView": {
      "type": "boolean",
      "default": false
    },
    "allowPublicComments": {
      "type": "boolean",
      "default": false
    },
    "showGuardianInfo": {
      "type": "boolean",
      "default": true
    },
    "showStatistics": {
      "type": "boolean",
      "default": true
    },
    "geolocationSharing": {
      "type": "string",
      "enum": ["none", "city_only", "full"],
      "default": "city_only"
    }
  }
}
```

### 10.3 Data Retention Policy

| Data Type | Active Period | Archive Period | Deletion Policy |
|-----------|---------------|----------------|-----------------|
| Memorial Profiles | Indefinite | N/A | Only upon guardian request |
| Media Assets | Indefinite | N/A | Only upon guardian request |
| Comments | Indefinite | N/A | Deletable by author/guardian |
| Activity Logs | 2 years | 5 years | Auto-delete after 7 years |
| Analytics Data | 1 year | 2 years | Anonymized after 3 years |
| Deleted Content | 30 days (recoverable) | N/A | Permanent after 30 days |

### 10.4 GDPR Compliance

```javascript
// Example data subject rights implementation
const dataSubjectRights = {
  rightToAccess: {
    endpoint: '/api/v1/memorials/{id}/export',
    format: 'JSON or PDF',
    deliveryTime: '30 days'
  },
  rightToRectification: {
    endpoint: '/api/v1/memorials/{id}',
    method: 'PATCH',
    allowedFields: ['all non-system fields']
  },
  rightToErasure: {
    endpoint: '/api/v1/memorials/{id}',
    method: 'DELETE',
    confirmation: 'required',
    graceperiod: '30 days'
  },
  rightToDataPortability: {
    endpoint: '/api/v1/memorials/{id}/export',
    format: 'JSON (machine-readable)',
    includesAllData: true
  },
  rightToObject: {
    endpoint: '/api/v1/memorials/{id}/privacy',
    options: ['opt-out of analytics', 'restrict processing']
  }
};
```

---

## Appendix A: Version History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0.0 | 2025-12-18 | Initial draft specification | WIA Standards Committee |

---

## Appendix B: References

- ISO 8601: Date and time format
- RFC 3986: URI Generic Syntax
- RFC 7946: GeoJSON Format
- Dublin Core Metadata Initiative
- GDPR (EU) 2016/679
- JSON Schema Draft 2020-12

---

## Appendix C: Acknowledgments

This specification was developed with input from:
- Pet memorial service providers
- Veterinary professionals
- Pet loss grief counselors
- Animal welfare organizations
- Pet cemetery and crematorium operators
- Bereaved pet guardians and families

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
