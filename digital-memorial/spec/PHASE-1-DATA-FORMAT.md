# WIA-DIGITAL_MEMORIAL PHASE 1 — Data Format Specification

**Standard:** WIA-DIGITAL_MEMORIAL
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA Digital Memorial Standard - Phase 1: Data Format Specification

<div style="background: linear-gradient(135deg, #64748B 0%, #475569 100%); color: white; padding: 2rem; border-radius: 8px; margin-bottom: 2rem;">

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-18
**Category:** Digital Death - Memorial Services
**Color Code:** #64748B (Slate)

</div>

## Table of Contents

1. [Overview](#overview)
2. [Core Data Models](#core-data-models)
3. [Memorial Profile Schema](#memorial-profile-schema)
4. [Biography and Life Story Format](#biography-and-life-story-format)
5. [Media Gallery Structure](#media-gallery-structure)
6. [Interactive Elements Schema](#interactive-elements-schema)
7. [Virtual Cemetery Integration](#virtual-cemetery-integration)
8. [Data Validation Rules](#data-validation-rules)
9. [Code Examples](#code-examples)
10. [Best Practices](#best-practices)

---

## Overview

The WIA Digital Memorial Data Format Specification defines standardized structures for creating, storing, and exchanging digital memorial information. This specification ensures interoperability between memorial platforms, preservation systems, and legacy services while maintaining dignity, privacy, and cultural sensitivity.

### Design Principles

- **Dignity First**: All data structures respect the deceased and their loved ones
- **Cultural Sensitivity**: Support for diverse cultural and religious practices
- **Long-term Preservation**: Formats designed for multi-generational durability
- **Privacy Control**: Granular access controls for sensitive information
- **Interoperability**: Standard formats for cross-platform compatibility
- **Extensibility**: Allow custom fields for unique memorial needs

### Scope

This phase covers:
- Core memorial profile data structures
- Biography and life story templates
- Photo and video gallery organization
- Interactive element definitions (candles, flowers, messages)
- Virtual cemetery location and mapping
- Metadata for preservation and discovery

---

## Core Data Models

### Memorial Entity Hierarchy

```
Memorial Site
├── Profile (Core Identity)
├── Biography (Life Story)
├── Media Gallery
│   ├── Photos
│   ├── Videos
│   ├── Audio
│   └── Documents
├── Interactive Elements
│   ├── Virtual Candles
│   ├── Flower Tributes
│   └── Visitor Messages
├── Timeline (Life Events)
├── Virtual Cemetery Plot
└── Preservation Metadata
```

### Data Format Support Matrix

| Format Type | Primary Format | Alternative Formats | Preservation Format |
|-------------|----------------|---------------------|---------------------|
| Profile Data | JSON | XML, YAML | JSON-LD |
| Biography | Markdown | HTML, Plain Text | PDF/A, EPUB |
| Photos | JPEG | PNG, WebP, HEIC | TIFF (uncompressed) |
| Videos | MP4 (H.264) | WebM, MOV | FFV1 (lossless) |
| Audio | MP3 | AAC, OGG | FLAC, WAV |
| Documents | PDF | DOCX, ODT | PDF/A-2 |

### Character Encoding Standards

| Data Element | Encoding | Fallback | Notes |
|--------------|----------|----------|-------|
| All Text Fields | UTF-8 | UTF-16 | Support for all languages and scripts |
| Names | Unicode NFC | ASCII transliteration | Normalized composed form |
| Dates | ISO 8601 | Unix timestamp | With timezone information |
| URLs | RFC 3986 | Punycode for IDN | Internationalized domain names |

---

## Memorial Profile Schema

### Core Profile Structure

```json
{
  "$schema": "https://wia-standards.org/schemas/digital-memorial/v1/profile.json",
  "memorialId": "uuid-v4",
  "version": "1.0.0",
  "createdAt": "2025-12-18T10:30:00Z",
  "lastModified": "2025-12-18T10:30:00Z",
  "status": "active|archived|suspended",

  "deceased": {
    "legalName": {
      "full": "string",
      "given": "string",
      "middle": "string",
      "family": "string",
      "suffix": "string",
      "prefix": "string"
    },
    "preferredName": "string",
    "otherNames": [
      {
        "name": "string",
        "type": "maiden|nickname|professional|cultural",
        "culture": "ISO 639-1 language code"
      }
    ],
    "birthDate": {
      "date": "YYYY-MM-DD",
      "precision": "exact|month|year|circa",
      "displayFormat": "string",
      "calendar": "gregorian|lunar|hijri|hebrew"
    },
    "deathDate": {
      "date": "YYYY-MM-DD",
      "precision": "exact|month|year|circa",
      "displayFormat": "string",
      "calendar": "gregorian|lunar|hijri|hebrew"
    },
    "age": {
      "years": "integer",
      "months": "integer",
      "days": "integer"
    },
    "gender": "string",
    "pronouns": ["string"],
    "nationality": ["ISO 3166-1 alpha-2"],
    "ethnicity": ["string"],
    "religion": "string",
    "occupation": ["string"],
    "epitaph": "string",
    "motto": "string"
  },

  "photography": {
    "primaryPhoto": {
      "url": "string",
      "thumbnailUrl": "string",
      "alt": "string",
      "capturedDate": "YYYY-MM-DD",
      "photographer": "string",
      "license": "all-rights-reserved|cc-by|cc-by-sa|cc-by-nd|family-only",
      "format": "jpeg|png|webp",
      "dimensions": {"width": "integer", "height": "integer"},
      "fileSize": "integer (bytes)",
      "checksums": {
        "md5": "string",
        "sha256": "string"
      }
    },
    "coverPhoto": {
      "url": "string",
      "alt": "string",
      "position": {"x": "percentage", "y": "percentage"}
    }
  },

  "location": {
    "birthPlace": {
      "name": "string",
      "address": {
        "street": "string",
        "city": "string",
        "state": "string",
        "country": "ISO 3166-1 alpha-2",
        "postalCode": "string"
      },
      "coordinates": {
        "latitude": "decimal",
        "longitude": "decimal",
        "accuracy": "integer (meters)"
      }
    },
    "deathPlace": {
      "name": "string",
      "address": "object (same as birthPlace)",
      "coordinates": "object (same as birthPlace)"
    },
    "residences": [
      {
        "address": "object",
        "fromDate": "YYYY-MM-DD",
        "toDate": "YYYY-MM-DD",
        "significance": "string"
      }
    ]
  },

  "relationships": [
    {
      "relationshipId": "uuid",
      "personId": "uuid|external",
      "name": "string",
      "relationship": "spouse|partner|parent|child|sibling|friend|colleague",
      "startDate": "YYYY-MM-DD",
      "endDate": "YYYY-MM-DD",
      "isPublic": "boolean",
      "significance": "string"
    }
  ],

  "privacy": {
    "visibility": "public|unlisted|private|family-only",
    "requiresAuthentication": "boolean",
    "allowSearchEngines": "boolean",
    "allowComments": "boolean",
    "allowTributes": "boolean",
    "allowMediaDownload": "boolean",
    "sensitiveContent": "boolean",
    "minorProtection": "boolean",
    "expirationDate": "YYYY-MM-DD|null"
  },

  "cultural": {
    "culturalBackground": ["string"],
    "languages": ["ISO 639-1"],
    "religiousRites": "string",
    "customTraditions": ["string"],
    "memorialPreferences": {
      "color": "hex color code",
      "theme": "string",
      "music": ["string"],
      "symbols": ["string"]
    }
  },

  "preservation": {
    "preservationLevel": "basic|standard|premium|perpetual",
    "guaranteedUntil": "YYYY-MM-DD|perpetual",
    "backupFrequency": "daily|weekly|monthly",
    "redundancyLevel": "integer",
    "archivalFormat": "boolean",
    "blockchainVerification": "boolean"
  }
}
```

### Extended Profile Attributes

```json
{
  "achievements": [
    {
      "title": "string",
      "description": "string",
      "date": "YYYY-MM-DD",
      "category": "education|career|community|personal|creative",
      "evidence": ["url"],
      "significance": "string"
    }
  ],

  "military": {
    "served": "boolean",
    "branch": "string",
    "rank": "string",
    "serviceYears": {"from": "YYYY", "to": "YYYY"},
    "decorations": ["string"],
    "conflicts": ["string"],
    "veteranStatus": "string"
  },

  "education": [
    {
      "institution": "string",
      "degree": "string",
      "field": "string",
      "graduationYear": "YYYY",
      "honors": ["string"]
    }
  ],

  "organizations": [
    {
      "name": "string",
      "role": "string",
      "fromDate": "YYYY-MM-DD",
      "toDate": "YYYY-MM-DD",
      "significance": "string"
    }
  ],

  "hobbies": ["string"],
  "passions": ["string"],
  "personality": ["string"],

  "funeralInfo": {
    "funeralDate": "YYYY-MM-DD",
    "funeralLocation": "string",
    "burialLocation": "string",
    "burialType": "burial|cremation|natural|sea|other",
    "graveLocation": {
      "cemetery": "string",
      "section": "string",
      "plot": "string",
      "gps": {"latitude": "decimal", "longitude": "decimal"}
    },
    "memorialService": {
      "date": "YYYY-MM-DD",
      "location": "string",
      "type": "string"
    }
  }
}
```

---

## Biography and Life Story Format

### Biography Structure

The biography follows a structured narrative format with metadata to enable rich storytelling while maintaining machine readability.

```json
{
  "$schema": "https://wia-standards.org/schemas/digital-memorial/v1/biography.json",
  "memorialId": "uuid-v4",
  "version": "1.0.0",
  "language": "ISO 639-1",
  "format": "markdown|html|plaintext",

  "summary": {
    "short": "string (280 chars max)",
    "medium": "string (1000 chars max)",
    "full": "string (unlimited)"
  },

  "narrative": {
    "introduction": {
      "content": "markdown/html string",
      "author": "string",
      "writtenDate": "YYYY-MM-DD",
      "tone": "formal|informal|poetic|journalistic"
    },

    "chapters": [
      {
        "chapterId": "uuid",
        "title": "string",
        "subtitle": "string",
        "order": "integer",
        "period": {
          "from": "YYYY-MM-DD",
          "to": "YYYY-MM-DD",
          "label": "Early Years|Youth|Adulthood|Later Years"
        },
        "content": "markdown/html string",
        "themes": ["string"],
        "keywords": ["string"],
        "relatedMedia": ["mediaId"],
        "relatedPeople": ["personId"],
        "relatedEvents": ["eventId"],
        "author": "string",
        "contributors": ["string"]
      }
    ],

    "epilogue": {
      "content": "markdown/html string",
      "author": "string",
      "writtenDate": "YYYY-MM-DD"
    }
  },

  "timeline": [
    {
      "eventId": "uuid",
      "date": {
        "start": "YYYY-MM-DD",
        "end": "YYYY-MM-DD|null",
        "precision": "exact|month|year|decade|circa"
      },
      "title": "string",
      "description": "string",
      "category": "birth|education|career|family|achievement|milestone|death",
      "significance": "major|notable|minor",
      "location": "string",
      "relatedMedia": ["mediaId"],
      "relatedPeople": ["personId"],
      "source": "string",
      "isPublic": "boolean"
    }
  ],

  "quotes": [
    {
      "text": "string",
      "context": "string",
      "date": "YYYY-MM-DD",
      "source": "string",
      "attribution": "verified|reported|remembered"
    }
  ],

  "stories": [
    {
      "storyId": "uuid",
      "title": "string",
      "narrator": "string",
      "narratorRelationship": "string",
      "recordedDate": "YYYY-MM-DD",
      "content": "markdown/html string",
      "mediaType": "text|audio|video",
      "mediaUrl": "string",
      "transcription": "string",
      "themes": ["string"],
      "isPublic": "boolean",
      "approvalRequired": "boolean"
    }
  ],

  "legacy": {
    "impact": "string",
    "causes": ["string"],
    "charities": [
      {
        "name": "string",
        "url": "string",
        "relationship": "string"
      }
    ],
    "memorialFunds": [
      {
        "name": "string",
        "purpose": "string",
        "url": "string",
        "donationInfo": "string"
      }
    ],
    "continuingWork": "string"
  },

  "metadata": {
    "biographer": "string",
    "lastUpdated": "ISO 8601 datetime",
    "sources": ["string"],
    "factChecked": "boolean",
    "familyApproved": "boolean",
    "completeness": "percentage",
    "readingTime": "integer (minutes)",
    "wordCount": "integer"
  }
}
```

### Life Story Templates

| Template Type | Use Case | Typical Length | Key Sections |
|---------------|----------|----------------|--------------|
| Brief Memorial | Quick tribute page | 500-1000 words | Introduction, Key Life Events, Legacy |
| Standard Biography | Full memorial site | 2000-5000 words | Early Life, Education, Career, Family, Achievements, Legacy |
| Extended Life Story | Comprehensive memorial | 5000+ words | Multiple chapters, detailed timeline, stories, quotes |
| Military Service | Veteran memorials | 1500-3000 words | Service history, deployments, decorations, post-service |
| Cultural Heritage | Cultural/religious focus | 2000-4000 words | Cultural background, traditions, community impact |
| Creative Legacy | Artists/creators | 2000-5000 words | Creative works, artistic journey, influence, portfolio |

---
