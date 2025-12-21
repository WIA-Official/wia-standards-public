# WIA Digital Funeral Standard - Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #64748B (Slate)
**Series:** Digital Death Services

---

## Table of Contents

1. [Introduction](#introduction)
2. [Core Data Models](#core-data-models)
3. [Funeral Service Models](#funeral-service-models)
4. [Attendee and Guest Management](#attendee-and-guest-management)
5. [Memorial and Tribute Models](#memorial-and-tribute-models)
6. [Financial and Donation Models](#financial-and-donation-models)
7. [Cemetery and Interment Models](#cemetery-and-interment-models)
8. [Data Validation Rules](#data-validation-rules)
9. [Reference Tables](#reference-tables)
10. [Implementation Examples](#implementation-examples)

---

## 1. Introduction

### 1.1 Purpose

This specification defines standardized data formats for digital funeral services, enabling interoperability between funeral homes, memorial platforms, streaming services, and related service providers. The standard supports both traditional and modern funeral practices, including virtual ceremonies, live streaming, and digital memorials.

### 1.2 Scope

The data format specification covers:
- Deceased person information and obituaries
- Funeral service planning and scheduling
- Virtual and hybrid ceremony configuration
- Guest management and RSVP tracking
- Memorial tributes and condolence messages
- Financial transactions and donations
- Cemetery and crematorium integration
- Pre-need funeral planning

### 1.3 Design Principles

- **Privacy First**: Sensitive information protected with appropriate access controls
- **Cultural Sensitivity**: Support for diverse religious and cultural practices
- **Accessibility**: Information available in multiple formats for all users
- **Permanence**: Long-term data preservation for memorial purposes
- **Flexibility**: Adaptable to various funeral service types and traditions

---

## 2. Core Data Models

### 2.1 Deceased Person Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "DeceasedPerson",
  "type": "object",
  "required": [
    "id",
    "legalName",
    "dateOfBirth",
    "dateOfDeath",
    "privacy"
  ],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid",
      "description": "Unique identifier for the deceased person"
    },
    "legalName": {
      "type": "object",
      "required": ["firstName", "lastName"],
      "properties": {
        "prefix": {
          "type": "string",
          "examples": ["Mr.", "Mrs.", "Dr.", "Rev."]
        },
        "firstName": {
          "type": "string",
          "minLength": 1,
          "maxLength": 100
        },
        "middleName": {
          "type": "string",
          "maxLength": 100
        },
        "lastName": {
          "type": "string",
          "minLength": 1,
          "maxLength": 100
        },
        "suffix": {
          "type": "string",
          "examples": ["Jr.", "Sr.", "III", "PhD"]
        },
        "preferredName": {
          "type": "string",
          "description": "Name the person was commonly known by"
        }
      }
    },
    "dateOfBirth": {
      "type": "string",
      "format": "date",
      "description": "ISO 8601 date format"
    },
    "dateOfDeath": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 date-time format"
    },
    "placeOfBirth": {
      "type": "object",
      "properties": {
        "city": {"type": "string"},
        "state": {"type": "string"},
        "country": {"type": "string", "pattern": "^[A-Z]{2}$"}
      }
    },
    "placeOfDeath": {
      "type": "object",
      "properties": {
        "facility": {"type": "string"},
        "city": {"type": "string"},
        "state": {"type": "string"},
        "country": {"type": "string", "pattern": "^[A-Z]{2}$"}
      }
    },
    "biography": {
      "type": "object",
      "properties": {
        "short": {
          "type": "string",
          "maxLength": 500,
          "description": "Brief obituary summary"
        },
        "full": {
          "type": "string",
          "maxLength": 10000,
          "description": "Complete life story"
        },
        "achievements": {
          "type": "array",
          "items": {"type": "string"},
          "description": "Notable accomplishments"
        },
        "hobbies": {
          "type": "array",
          "items": {"type": "string"}
        }
      }
    },
    "family": {
      "type": "object",
      "properties": {
        "spouse": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "status": {
                "type": "string",
                "enum": ["current", "predeceased", "divorced"]
              }
            }
          }
        },
        "children": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "status": {
                "type": "string",
                "enum": ["surviving", "predeceased"]
              }
            }
          }
        },
        "parents": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "status": {
                "type": "string",
                "enum": ["surviving", "predeceased"]
              }
            }
          }
        },
        "siblings": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "status": {
                "type": "string",
                "enum": ["surviving", "predeceased"]
              }
            }
          }
        }
      }
    },
    "photos": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "url": {"type": "string", "format": "uri"},
          "caption": {"type": "string"},
          "isPrimary": {"type": "boolean"},
          "yearTaken": {"type": "integer"},
          "uploadedAt": {"type": "string", "format": "date-time"}
        }
      }
    },
    "privacy": {
      "type": "object",
      "required": ["level"],
      "properties": {
        "level": {
          "type": "string",
          "enum": ["public", "family-only", "private"],
          "description": "Overall privacy level for deceased information"
        },
        "showDateOfBirth": {"type": "boolean", "default": true},
        "showPlaceOfBirth": {"type": "boolean", "default": true},
        "showFullBiography": {"type": "boolean", "default": true},
        "showFamilyMembers": {"type": "boolean", "default": true},
        "allowPublicCondolences": {"type": "boolean", "default": true}
      }
    },
    "metadata": {
      "type": "object",
      "properties": {
        "createdAt": {"type": "string", "format": "date-time"},
        "updatedAt": {"type": "string", "format": "date-time"},
        "createdBy": {"type": "string", "description": "User ID"},
        "verificationStatus": {
          "type": "string",
          "enum": ["pending", "verified", "disputed"]
        }
      }
    }
  }
}
```

### 2.2 Obituary Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Obituary",
  "type": "object",
  "required": ["id", "deceasedId", "title", "content", "status"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "deceasedId": {
      "type": "string",
      "format": "uuid",
      "description": "Reference to DeceasedPerson"
    },
    "title": {
      "type": "string",
      "maxLength": 200,
      "examples": ["In Loving Memory of John Smith"]
    },
    "content": {
      "type": "object",
      "properties": {
        "text": {
          "type": "string",
          "description": "Plain text obituary"
        },
        "html": {
          "type": "string",
          "description": "HTML formatted obituary"
        },
        "markdown": {
          "type": "string",
          "description": "Markdown formatted obituary"
        }
      }
    },
    "publishedDate": {
      "type": "string",
      "format": "date-time"
    },
    "expiryDate": {
      "type": "string",
      "format": "date-time",
      "description": "When obituary should be unpublished"
    },
    "newspapers": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "name": {"type": "string"},
          "publicationDate": {"type": "string", "format": "date"},
          "url": {"type": "string", "format": "uri"},
          "cost": {"type": "number"}
        }
      }
    },
    "status": {
      "type": "string",
      "enum": ["draft", "pending-approval", "published", "archived"]
    },
    "viewCount": {
      "type": "integer",
      "minimum": 0
    },
    "metadata": {
      "type": "object",
      "properties": {
        "language": {"type": "string", "pattern": "^[a-z]{2}$"},
        "createdAt": {"type": "string", "format": "date-time"},
        "updatedAt": {"type": "string", "format": "date-time"}
      }
    }
  }
}
```

---

## 3. Funeral Service Models

### 3.1 Funeral Service Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "FuneralService",
  "type": "object",
  "required": [
    "id",
    "deceasedId",
    "serviceType",
    "startDateTime",
    "venue"
  ],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "deceasedId": {
      "type": "string",
      "format": "uuid"
    },
    "serviceType": {
      "type": "string",
      "enum": [
        "viewing",
        "visitation",
        "wake",
        "funeral",
        "memorial",
        "celebration-of-life",
        "graveside",
        "committal",
        "virtual"
      ]
    },
    "religiousTradition": {
      "type": "string",
      "enum": [
        "christian",
        "catholic",
        "jewish",
        "muslim",
        "buddhist",
        "hindu",
        "non-religious",
        "interfaith",
        "other"
      ]
    },
    "startDateTime": {
      "type": "string",
      "format": "date-time"
    },
    "endDateTime": {
      "type": "string",
      "format": "date-time"
    },
    "timezone": {
      "type": "string",
      "examples": ["America/New_York", "Europe/London"]
    },
    "venue": {
      "type": "object",
      "required": ["type"],
      "properties": {
        "type": {
          "type": "string",
          "enum": ["funeral-home", "church", "cemetery", "crematorium", "private-residence", "virtual", "other"]
        },
        "name": {"type": "string"},
        "address": {
          "type": "object",
          "properties": {
            "street": {"type": "string"},
            "city": {"type": "string"},
            "state": {"type": "string"},
            "postalCode": {"type": "string"},
            "country": {"type": "string"}
          }
        },
        "coordinates": {
          "type": "object",
          "properties": {
            "latitude": {"type": "number", "minimum": -90, "maximum": 90},
            "longitude": {"type": "number", "minimum": -180, "maximum": 180}
          }
        },
        "capacity": {"type": "integer"},
        "accessibility": {
          "type": "object",
          "properties": {
            "wheelchairAccessible": {"type": "boolean"},
            "parkingAvailable": {"type": "boolean"},
            "assistiveListeningDevices": {"type": "boolean"}
          }
        }
      }
    },
    "virtualService": {
      "type": "object",
      "properties": {
        "enabled": {"type": "boolean", "default": false},
        "streamingUrl": {"type": "string", "format": "uri"},
        "platform": {
          "type": "string",
          "enum": ["zoom", "youtube", "facebook", "custom"]
        },
        "accessCode": {"type": "string"},
        "recordingEnabled": {"type": "boolean"},
        "recordingUrl": {"type": "string", "format": "uri"},
        "chatEnabled": {"type": "boolean"},
        "maxVirtualAttendees": {"type": "integer"}
      }
    },
    "program": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "order": {"type": "integer"},
          "type": {
            "type": "string",
            "enum": ["prayer", "eulogy", "music", "reading", "video-tribute", "open-mic", "other"]
          },
          "title": {"type": "string"},
          "description": {"type": "string"},
          "duration": {"type": "integer", "description": "Duration in minutes"},
          "performer": {"type": "string"}
        }
      }
    },
    "officiant": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "title": {"type": "string"},
        "organization": {"type": "string"},
        "contact": {
          "type": "object",
          "properties": {
            "phone": {"type": "string"},
            "email": {"type": "string", "format": "email"}
          }
        }
      }
    },
    "dresscode": {
      "type": "string",
      "examples": ["formal", "business casual", "casual", "traditional mourning"]
    },
    "specialInstructions": {
      "type": "string",
      "maxLength": 1000
    },
    "rsvpRequired": {
      "type": "boolean",
      "default": false
    },
    "rsvpDeadline": {
      "type": "string",
      "format": "date-time"
    },
    "expectedAttendance": {
      "type": "integer"
    },
    "status": {
      "type": "string",
      "enum": ["planned", "confirmed", "in-progress", "completed", "cancelled", "postponed"]
    },
    "metadata": {
      "type": "object",
      "properties": {
        "createdAt": {"type": "string", "format": "date-time"},
        "updatedAt": {"type": "string", "format": "date-time"},
        "createdBy": {"type": "string"}
      }
    }
  }
}
```

### 3.2 Pre-Need Planning Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "PreNeedPlan",
  "type": "object",
  "required": ["id", "planHolderId", "status"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "planHolderId": {
      "type": "string",
      "format": "uuid",
      "description": "Person for whom the plan is made"
    },
    "status": {
      "type": "string",
      "enum": ["draft", "active", "paid-in-full", "cancelled", "executed"]
    },
    "createdDate": {
      "type": "string",
      "format": "date-time"
    },
    "servicePreferences": {
      "type": "object",
      "properties": {
        "serviceType": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["viewing", "funeral", "memorial", "celebration-of-life", "graveside"]
          }
        },
        "religiousTradition": {"type": "string"},
        "preferredVenue": {"type": "string"},
        "musicSelections": {
          "type": "array",
          "items": {"type": "string"}
        },
        "readings": {
          "type": "array",
          "items": {"type": "string"}
        },
        "specialRequests": {"type": "string"}
      }
    },
    "dispositionPreferences": {
      "type": "object",
      "properties": {
        "method": {
          "type": "string",
          "enum": ["burial", "cremation", "natural-burial", "aquamation", "donation"]
        },
        "casketType": {"type": "string"},
        "urnType": {"type": "string"},
        "vaultRequired": {"type": "boolean"},
        "cemetery": {"type": "string"},
        "plotLocation": {"type": "string"}
      }
    },
    "financialPlan": {
      "type": "object",
      "properties": {
        "estimatedCost": {"type": "number"},
        "amountPaid": {"type": "number"},
        "paymentPlan": {
          "type": "string",
          "enum": ["lump-sum", "installments", "insurance-funded", "trust-funded"]
        },
        "insurancePolicy": {"type": "string"},
        "trustAccount": {"type": "string"}
      }
    },
    "documents": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "type": {
            "type": "string",
            "enum": ["contract", "insurance-policy", "will", "advance-directive", "other"]
          },
          "url": {"type": "string", "format": "uri"},
          "uploadedAt": {"type": "string", "format": "date-time"}
        }
      }
    },
    "contacts": {
      "type": "object",
      "properties": {
        "primaryContact": {
          "type": "object",
          "properties": {
            "name": {"type": "string"},
            "relationship": {"type": "string"},
            "phone": {"type": "string"},
            "email": {"type": "string", "format": "email"}
          }
        },
        "alternateContacts": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "relationship": {"type": "string"},
              "phone": {"type": "string"},
              "email": {"type": "string", "format": "email"}
            }
          }
        }
      }
    }
  }
}
```

---

## 4. Attendee and Guest Management

### 4.1 Guest RSVP Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "GuestRSVP",
  "type": "object",
  "required": ["id", "serviceId", "attendeeInfo", "response"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "serviceId": {
      "type": "string",
      "format": "uuid"
    },
    "attendeeInfo": {
      "type": "object",
      "required": ["name", "email"],
      "properties": {
        "name": {"type": "string"},
        "email": {"type": "string", "format": "email"},
        "phone": {"type": "string"},
        "relationship": {"type": "string"}
      }
    },
    "response": {
      "type": "string",
      "enum": ["attending", "not-attending", "maybe", "virtual-only"]
    },
    "attendanceType": {
      "type": "string",
      "enum": ["in-person", "virtual", "both"]
    },
    "numberOfGuests": {
      "type": "integer",
      "minimum": 1,
      "maximum": 10
    },
    "guestNames": {
      "type": "array",
      "items": {"type": "string"}
    },
    "dietaryRestrictions": {
      "type": "array",
      "items": {"type": "string"}
    },
    "accessibilityNeeds": {
      "type": "string"
    },
    "submittedAt": {
      "type": "string",
      "format": "date-time"
    },
    "updatedAt": {
      "type": "string",
      "format": "date-time"
    }
  }
}
```

---

## 5. Memorial and Tribute Models

### 5.1 Condolence Message Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "CondolenceMessage",
  "type": "object",
  "required": ["id", "deceasedId", "authorName", "message"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "deceasedId": {
      "type": "string",
      "format": "uuid"
    },
    "authorName": {
      "type": "string",
      "maxLength": 100
    },
    "authorEmail": {
      "type": "string",
      "format": "email"
    },
    "relationship": {
      "type": "string",
      "examples": ["family", "friend", "colleague", "neighbor", "other"]
    },
    "message": {
      "type": "string",
      "minLength": 1,
      "maxLength": 5000
    },
    "isPublic": {
      "type": "boolean",
      "default": true
    },
    "moderationStatus": {
      "type": "string",
      "enum": ["pending", "approved", "rejected", "flagged"],
      "default": "pending"
    },
    "attachments": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "type": {"type": "string", "enum": ["photo", "video", "audio"]},
          "url": {"type": "string", "format": "uri"},
          "caption": {"type": "string"}
        }
      }
    },
    "postedAt": {
      "type": "string",
      "format": "date-time"
    },
    "metadata": {
      "type": "object",
      "properties": {
        "ipAddress": {"type": "string"},
        "userAgent": {"type": "string"}
      }
    }
  }
}
```

### 5.2 Memorial Tribute Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "MemorialTribute",
  "type": "object",
  "required": ["id", "deceasedId", "type", "content"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "deceasedId": {
      "type": "string",
      "format": "uuid"
    },
    "type": {
      "type": "string",
      "enum": ["story", "photo-album", "video-montage", "audio-recording", "poem", "artwork"]
    },
    "title": {
      "type": "string",
      "maxLength": 200
    },
    "content": {
      "type": "object",
      "properties": {
        "text": {"type": "string"},
        "mediaUrls": {
          "type": "array",
          "items": {"type": "string", "format": "uri"}
        },
        "embedCode": {"type": "string"}
      }
    },
    "author": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "relationship": {"type": "string"}
      }
    },
    "createdAt": {
      "type": "string",
      "format": "date-time"
    },
    "isPublic": {
      "type": "boolean",
      "default": true
    },
    "viewCount": {
      "type": "integer",
      "minimum": 0
    }
  }
}
```

---

## 6. Financial and Donation Models

### 6.1 Donation Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Donation",
  "type": "object",
  "required": ["id", "deceasedId", "amount", "currency", "donorInfo"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "deceasedId": {
      "type": "string",
      "format": "uuid"
    },
    "amount": {
      "type": "number",
      "minimum": 0
    },
    "currency": {
      "type": "string",
      "pattern": "^[A-Z]{3}$",
      "default": "USD"
    },
    "donorInfo": {
      "type": "object",
      "required": ["name"],
      "properties": {
        "name": {"type": "string"},
        "email": {"type": "string", "format": "email"},
        "isAnonymous": {"type": "boolean", "default": false}
      }
    },
    "recipient": {
      "type": "object",
      "properties": {
        "type": {
          "type": "string",
          "enum": ["charity", "family", "memorial-fund", "other"]
        },
        "name": {"type": "string"},
        "taxId": {"type": "string"}
      }
    },
    "inMemoryOf": {
      "type": "string",
      "description": "Name of deceased"
    },
    "message": {
      "type": "string",
      "maxLength": 500
    },
    "paymentMethod": {
      "type": "string",
      "enum": ["credit-card", "debit-card", "paypal", "venmo", "bank-transfer", "check"]
    },
    "transactionId": {
      "type": "string"
    },
    "status": {
      "type": "string",
      "enum": ["pending", "completed", "failed", "refunded"]
    },
    "donatedAt": {
      "type": "string",
      "format": "date-time"
    },
    "receiptUrl": {
      "type": "string",
      "format": "uri"
    }
  }
}
```

### 6.2 Floral Arrangement Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "FloralArrangement",
  "type": "object",
  "required": ["id", "serviceId", "type", "sender"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "serviceId": {
      "type": "string",
      "format": "uuid"
    },
    "type": {
      "type": "string",
      "examples": ["standing-spray", "casket-spray", "wreath", "basket", "bouquet"]
    },
    "description": {
      "type": "string"
    },
    "florist": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "phone": {"type": "string"},
        "orderNumber": {"type": "string"}
      }
    },
    "sender": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "organization": {"type": "string"},
        "relationship": {"type": "string"}
      }
    },
    "cardMessage": {
      "type": "string",
      "maxLength": 200
    },
    "deliveryDate": {
      "type": "string",
      "format": "date"
    },
    "deliveryLocation": {
      "type": "string",
      "enum": ["funeral-home", "church", "cemetery", "residence"]
    },
    "cost": {
      "type": "number"
    },
    "photoUrl": {
      "type": "string",
      "format": "uri"
    }
  }
}
```

---

## 7. Cemetery and Interment Models

### 7.1 Interment Record Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "IntermentRecord",
  "type": "object",
  "required": ["id", "deceasedId", "cemetery", "intermentDate"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "deceasedId": {
      "type": "string",
      "format": "uuid"
    },
    "cemetery": {
      "type": "object",
      "required": ["name"],
      "properties": {
        "id": {"type": "string", "format": "uuid"},
        "name": {"type": "string"},
        "address": {
          "type": "object",
          "properties": {
            "street": {"type": "string"},
            "city": {"type": "string"},
            "state": {"type": "string"},
            "postalCode": {"type": "string"},
            "country": {"type": "string"}
          }
        },
        "coordinates": {
          "type": "object",
          "properties": {
            "latitude": {"type": "number"},
            "longitude": {"type": "number"}
          }
        }
      }
    },
    "plotLocation": {
      "type": "object",
      "properties": {
        "section": {"type": "string"},
        "lot": {"type": "string"},
        "grave": {"type": "string"},
        "coordinates": {
          "type": "object",
          "properties": {
            "latitude": {"type": "number"},
            "longitude": {"type": "number"}
          }
        }
      }
    },
    "intermentType": {
      "type": "string",
      "enum": ["full-body", "cremains", "above-ground", "mausoleum", "columbarium"]
    },
    "intermentDate": {
      "type": "string",
      "format": "date-time"
    },
    "casketVault": {
      "type": "object",
      "properties": {
        "casketType": {"type": "string"},
        "vaultType": {"type": "string"},
        "vaultRequired": {"type": "boolean"}
      }
    },
    "monument": {
      "type": "object",
      "properties": {
        "type": {"type": "string", "enum": ["headstone", "marker", "monument", "plaque", "none"]},
        "material": {"type": "string"},
        "inscription": {"type": "string"},
        "unveilingDate": {"type": "string", "format": "date"}
      }
    },
    "perpetualCare": {
      "type": "boolean"
    },
    "deedHolder": {
      "type": "string"
    }
  }
}
```

---

## 8. Data Validation Rules

### 8.1 Date and Time Validation

| Field | Rule | Description |
|-------|------|-------------|
| dateOfDeath | Must be <= current date | Cannot be in the future |
| dateOfBirth | Must be < dateOfDeath | Must precede death date |
| serviceStartDateTime | Should be >= dateOfDeath | Typically after death |
| rsvpDeadline | Should be < serviceStartDateTime | RSVP closes before service |
| expiryDate | Should be > publishedDate | Expiry after publication |

### 8.2 Privacy and Access Control

| Privacy Level | Accessible To | Restrictions |
|--------------|---------------|--------------|
| public | Anyone with link | All information visible |
| family-only | Authenticated family members | Requires authentication |
| private | Authorized users only | Full access control required |

### 8.3 Content Moderation Rules

| Content Type | Moderation Level | Auto-Approval Criteria |
|-------------|------------------|------------------------|
| Condolence Messages | Moderate | Length < 500 chars, no flagged words |
| Photos | Strict | File size < 10MB, approved formats |
| Video Tributes | Strict | Duration < 10 min, approved formats |
| Obituaries | Review | Submitted by verified users |

---

## 9. Reference Tables

### 9.1 Service Type Categories

| Category | Service Types | Typical Duration |
|----------|---------------|------------------|
| Pre-Service | Viewing, Visitation, Wake | 2-4 hours |
| Main Service | Funeral, Memorial, Celebration of Life | 1-2 hours |
| Post-Service | Graveside, Committal, Reception | 30-60 minutes |
| Virtual | Live-Streamed Service | Matches in-person service |

### 9.2 Cultural and Religious Traditions

| Tradition | Typical Practices | Timeline |
|-----------|------------------|----------|
| Christian | Viewing, funeral service, burial | 3-7 days after death |
| Catholic | Wake, funeral mass, burial | 3-7 days after death |
| Jewish | Immediate burial, shiva, unveiling | Within 24 hours, 7 days, 1 year |
| Muslim | Ritual washing, burial (no embalming) | Within 24 hours |
| Buddhist | Chanting, cremation, memorial services | Varies by tradition |
| Hindu | Cremation, scattering of ashes | Within 24 hours if possible |

### 9.3 Disposition Methods

| Method | Process | Environmental Impact |
|--------|---------|---------------------|
| Traditional Burial | Embalming, casket, vault | Moderate to High |
| Cremation | High-temperature burning | Moderate |
| Natural Burial | No embalming, biodegradable casket | Low |
| Aquamation | Water-based cremation | Low |
| Body Donation | Medical/scientific research | N/A |

### 9.4 Cost Ranges (USD)

| Service Component | Low Range | Mid Range | High Range |
|------------------|-----------|-----------|------------|
| Funeral Service | $2,000 | $7,000 | $15,000+ |
| Cremation Only | $800 | $2,500 | $5,000 |
| Casket | $500 | $2,500 | $10,000+ |
| Burial Plot | $1,000 | $3,500 | $10,000+ |
| Headstone | $500 | $2,000 | $8,000+ |
| Virtual Streaming | $200 | $500 | $1,500 |

---

## 10. Implementation Examples

### 10.1 Complete Deceased Person Record

```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "legalName": {
    "prefix": "Dr.",
    "firstName": "Margaret",
    "middleName": "Anne",
    "lastName": "Johnson",
    "suffix": "PhD",
    "preferredName": "Maggie"
  },
  "dateOfBirth": "1945-03-15",
  "dateOfDeath": "2025-12-10T08:30:00Z",
  "placeOfBirth": {
    "city": "Boston",
    "state": "Massachusetts",
    "country": "US"
  },
  "placeOfDeath": {
    "facility": "St. Mary's Hospital",
    "city": "Portland",
    "state": "Oregon",
    "country": "US"
  },
  "biography": {
    "short": "Dr. Margaret Johnson, beloved mother, grandmother, and retired professor of literature, passed away peacefully at age 80.",
    "full": "Dr. Margaret 'Maggie' Johnson dedicated her life to education and family. She taught English literature at Portland State University for 40 years, inspiring thousands of students with her passion for poetry and classic novels. Born in Boston during World War II, she was the first in her family to attend college, earning her PhD from Harvard University in 1972. She authored three books on American poetry and was a frequent contributor to literary journals. Maggie is survived by her loving husband of 55 years, Robert; three children, Sarah, Michael, and Jennifer; and seven grandchildren. She will be remembered for her kindness, wit, and unwavering dedication to her students and family.",
    "achievements": [
      "Professor Emerita, Portland State University",
      "Author of three books on American poetry",
      "Recipient of the Distinguished Teaching Award (1998)",
      "Founded the Portland Poetry Society"
    ],
    "hobbies": [
      "Reading",
      "Gardening",
      "Traveling",
      "Volunteering at the local library"
    ]
  },
  "family": {
    "spouse": [
      {
        "name": "Robert Johnson",
        "status": "current"
      }
    ],
    "children": [
      {"name": "Sarah Martinez", "status": "surviving"},
      {"name": "Michael Johnson", "status": "surviving"},
      {"name": "Jennifer Chen", "status": "surviving"}
    ],
    "parents": [
      {"name": "Thomas O'Brien", "status": "predeceased"},
      {"name": "Catherine O'Brien", "status": "predeceased"}
    ],
    "siblings": [
      {"name": "Patrick O'Brien", "status": "surviving"},
      {"name": "Maureen Walsh", "status": "predeceased"}
    ]
  },
  "photos": [
    {
      "url": "https://cdn.example.com/photos/maggie-primary.jpg",
      "caption": "Dr. Johnson at her retirement celebration, 2010",
      "isPrimary": true,
      "yearTaken": 2010,
      "uploadedAt": "2025-12-11T10:00:00Z"
    }
  ],
  "privacy": {
    "level": "public",
    "showDateOfBirth": true,
    "showPlaceOfBirth": true,
    "showFullBiography": true,
    "showFamilyMembers": true,
    "allowPublicCondolences": true
  },
  "metadata": {
    "createdAt": "2025-12-10T12:00:00Z",
    "updatedAt": "2025-12-11T15:30:00Z",
    "createdBy": "user-12345",
    "verificationStatus": "verified"
  }
}
```

### 10.2 Hybrid Funeral Service

```json
{
  "id": "660e8400-e29b-41d4-a716-446655440001",
  "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
  "serviceType": "memorial",
  "religiousTradition": "non-religious",
  "startDateTime": "2025-12-18T14:00:00-08:00",
  "endDateTime": "2025-12-18T16:00:00-08:00",
  "timezone": "America/Los_Angeles",
  "venue": {
    "type": "funeral-home",
    "name": "Peaceful Rest Memorial Chapel",
    "address": {
      "street": "1234 Oak Street",
      "city": "Portland",
      "state": "Oregon",
      "postalCode": "97201",
      "country": "US"
    },
    "coordinates": {
      "latitude": 45.5152,
      "longitude": -122.6784
    },
    "capacity": 150,
    "accessibility": {
      "wheelchairAccessible": true,
      "parkingAvailable": true,
      "assistiveListeningDevices": true
    }
  },
  "virtualService": {
    "enabled": true,
    "streamingUrl": "https://stream.example.com/service/660e8400",
    "platform": "custom",
    "accessCode": "MAGGIE2025",
    "recordingEnabled": true,
    "recordingUrl": "https://cdn.example.com/recordings/660e8400.mp4",
    "chatEnabled": true,
    "maxVirtualAttendees": 500
  },
  "program": [
    {
      "order": 1,
      "type": "music",
      "title": "Prelude",
      "description": "Classical piano performance",
      "duration": 10,
      "performer": "Emily Chen, pianist"
    },
    {
      "order": 2,
      "type": "reading",
      "title": "Opening Words",
      "description": "Poem: 'Do Not Stand at My Grave and Weep'",
      "duration": 5,
      "performer": "Sarah Martinez, daughter"
    },
    {
      "order": 3,
      "type": "eulogy",
      "title": "Life Remembrance",
      "description": "Reflections on Dr. Johnson's life and legacy",
      "duration": 15,
      "performer": "Dr. James Wilson, colleague"
    },
    {
      "order": 4,
      "type": "video-tribute",
      "title": "A Life in Pictures",
      "description": "Photo and video montage",
      "duration": 10
    },
    {
      "order": 5,
      "type": "open-mic",
      "title": "Sharing of Memories",
      "description": "Open invitation for attendees to share memories",
      "duration": 30
    },
    {
      "order": 6,
      "type": "music",
      "title": "Closing Song",
      "description": "'Amazing Grace' performed by Portland Choir",
      "duration": 5,
      "performer": "Portland Community Choir"
    }
  ],
  "officiant": {
    "name": "Rev. Susan Matthews",
    "title": "Celebrant",
    "organization": "Independent Celebrant Services",
    "contact": {
      "phone": "+1-503-555-0123",
      "email": "susan@celebrantservices.com"
    }
  },
  "dresscode": "business casual",
  "specialInstructions": "In lieu of flowers, the family requests donations to the Portland Public Library. A reception will follow in the adjacent hall.",
  "rsvpRequired": true,
  "rsvpDeadline": "2025-12-16T23:59:59-08:00",
  "expectedAttendance": 120,
  "status": "confirmed",
  "metadata": {
    "createdAt": "2025-12-11T09:00:00Z",
    "updatedAt": "2025-12-13T14:30:00Z",
    "createdBy": "user-12345"
  }
}
```

### 10.3 Guest RSVP Response

```json
{
  "id": "770e8400-e29b-41d4-a716-446655440002",
  "serviceId": "660e8400-e29b-41d4-a716-446655440001",
  "attendeeInfo": {
    "name": "David Miller",
    "email": "david.miller@email.com",
    "phone": "+1-503-555-0199",
    "relationship": "former student"
  },
  "response": "attending",
  "attendanceType": "in-person",
  "numberOfGuests": 2,
  "guestNames": ["David Miller", "Lisa Miller"],
  "dietaryRestrictions": ["vegetarian"],
  "accessibilityNeeds": "None",
  "submittedAt": "2025-12-14T10:30:00Z",
  "updatedAt": "2025-12-14T10:30:00Z"
}
```

### 10.4 Condolence Message

```json
{
  "id": "880e8400-e29b-41d4-a716-446655440003",
  "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
  "authorName": "Rebecca Thompson",
  "authorEmail": "rebecca.t@email.com",
  "relationship": "former student",
  "message": "Dr. Johnson was the most inspiring professor I ever had. Her passion for literature was contagious, and she truly cared about each of her students. The lessons she taught me extend far beyond the classroom. She will be deeply missed. My thoughts are with the Johnson family during this difficult time.",
  "isPublic": true,
  "moderationStatus": "approved",
  "attachments": [],
  "postedAt": "2025-12-12T16:45:00Z",
  "metadata": {
    "ipAddress": "192.168.1.100",
    "userAgent": "Mozilla/5.0..."
  }
}
```

### 10.5 Memorial Donation

```json
{
  "id": "990e8400-e29b-41d4-a716-446655440004",
  "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
  "amount": 100.00,
  "currency": "USD",
  "donorInfo": {
    "name": "Portland State University Alumni Association",
    "email": "alumni@psu.edu",
    "isAnonymous": false
  },
  "recipient": {
    "type": "charity",
    "name": "Portland Public Library Foundation",
    "taxId": "12-3456789"
  },
  "inMemoryOf": "Dr. Margaret Johnson",
  "message": "In honor of Dr. Johnson's lifelong dedication to literacy and education.",
  "paymentMethod": "credit-card",
  "transactionId": "txn_1234567890",
  "status": "completed",
  "donatedAt": "2025-12-13T09:15:00Z",
  "receiptUrl": "https://receipts.example.com/990e8400"
}
```

### 10.6 Pre-Need Funeral Plan

```json
{
  "id": "aa0e8400-e29b-41d4-a716-446655440005",
  "planHolderId": "bb0e8400-e29b-41d4-a716-446655440006",
  "status": "active",
  "createdDate": "2023-06-15T10:00:00Z",
  "servicePreferences": {
    "serviceType": ["memorial", "celebration-of-life"],
    "religiousTradition": "non-religious",
    "preferredVenue": "Garden Chapel at Green Valley Cemetery",
    "musicSelections": [
      "What a Wonderful World - Louis Armstrong",
      "Here Comes the Sun - The Beatles",
      "Imagine - John Lennon"
    ],
    "readings": [
      "The Road Not Taken - Robert Frost",
      "Desiderata - Max Ehrmann"
    ],
    "specialRequests": "Please encourage guests to wear bright colors instead of black. I'd like the service to be a celebration of life rather than a somber affair."
  },
  "dispositionPreferences": {
    "method": "cremation",
    "casketType": "simple pine casket for cremation",
    "urnType": "biodegradable urn for scattering",
    "vaultRequired": false,
    "cemetery": "Green Valley Cemetery",
    "plotLocation": "Memorial Garden, Section C"
  },
  "financialPlan": {
    "estimatedCost": 8500.00,
    "amountPaid": 5100.00,
    "paymentPlan": "installments",
    "insurancePolicy": "POL-789456123",
    "trustAccount": "TRU-456789012"
  },
  "documents": [
    {
      "type": "contract",
      "url": "https://docs.example.com/preneed/contract-aa0e8400.pdf",
      "uploadedAt": "2023-06-15T10:30:00Z"
    },
    {
      "type": "advance-directive",
      "url": "https://docs.example.com/preneed/directive-aa0e8400.pdf",
      "uploadedAt": "2023-06-15T11:00:00Z"
    }
  ],
  "contacts": {
    "primaryContact": {
      "name": "Jessica Anderson",
      "relationship": "daughter",
      "phone": "+1-503-555-0234",
      "email": "jessica.anderson@email.com"
    },
    "alternateContacts": [
      {
        "name": "Mark Anderson",
        "relationship": "son",
        "phone": "+1-503-555-0235",
        "email": "mark.anderson@email.com"
      }
    ]
  }
}
```

### 10.7 Interment Record

```json
{
  "id": "cc0e8400-e29b-41d4-a716-446655440007",
  "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
  "cemetery": {
    "id": "dd0e8400-e29b-41d4-a716-446655440008",
    "name": "River View Memorial Park",
    "address": {
      "street": "5678 Cemetery Lane",
      "city": "Portland",
      "state": "Oregon",
      "postalCode": "97202",
      "country": "US"
    },
    "coordinates": {
      "latitude": 45.4876,
      "longitude": -122.6543
    }
  },
  "plotLocation": {
    "section": "Garden of Memories",
    "lot": "42",
    "grave": "3",
    "coordinates": {
      "latitude": 45.4878,
      "longitude": -122.6545
    }
  },
  "intermentType": "cremains",
  "intermentDate": "2025-12-20T11:00:00-08:00",
  "casketVault": {
    "casketType": "n/a",
    "vaultType": "n/a",
    "vaultRequired": false
  },
  "monument": {
    "type": "marker",
    "material": "granite",
    "inscription": "Dr. Margaret Anne Johnson\n1945 - 2025\nBeloved Wife, Mother, and Teacher\n'Education is the most powerful weapon'",
    "unveilingDate": "2026-12-10"
  },
  "perpetualCare": true,
  "deedHolder": "Johnson Family Trust"
}
```

---

## Conclusion

This Phase 1 specification establishes comprehensive data formats for digital funeral services. The schemas provide flexibility for various cultural traditions, service types, and modern features like virtual attendance and digital memorials. Implementation of these formats enables interoperability across the funeral service industry while respecting privacy and cultural sensitivities.

**Next Phase:** Phase 2 will define the API interfaces for creating, retrieving, updating, and managing funeral service data using these standardized formats.

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
