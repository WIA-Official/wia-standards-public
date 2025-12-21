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

## Media Gallery Structure

### Gallery Organization Schema

```json
{
  "$schema": "https://wia-standards.org/schemas/digital-memorial/v1/gallery.json",
  "memorialId": "uuid-v4",
  "version": "1.0.0",

  "collections": [
    {
      "collectionId": "uuid",
      "title": "string",
      "description": "string",
      "type": "photos|videos|audio|documents|mixed",
      "coverMedia": "mediaId",
      "order": "integer",
      "createdDate": "ISO 8601",
      "privacy": "public|unlisted|private|family",
      "itemCount": "integer",

      "items": [
        {
          "mediaId": "uuid",
          "type": "photo|video|audio|document",
          "title": "string",
          "description": "string",
          "caption": "string",

          "file": {
            "url": "string",
            "filename": "string",
            "mimeType": "string",
            "fileSize": "integer (bytes)",
            "format": "string",
            "checksums": {
              "md5": "string",
              "sha256": "string"
            }
          },

          "thumbnail": {
            "url": "string",
            "width": "integer",
            "height": "integer"
          },

          "variants": [
            {
              "type": "thumbnail|small|medium|large|original|archival",
              "url": "string",
              "width": "integer",
              "height": "integer",
              "fileSize": "integer"
            }
          ],

          "metadata": {
            "capturedDate": "YYYY-MM-DD",
            "uploadedDate": "ISO 8601",
            "uploadedBy": "string",
            "location": {
              "name": "string",
              "coordinates": {"latitude": "decimal", "longitude": "decimal"}
            },
            "peopleTagged": [
              {
                "personId": "uuid",
                "name": "string",
                "position": {"x": "percentage", "y": "percentage"}
              }
            ],
            "tags": ["string"],
            "keywords": ["string"],
            "occasion": "string",
            "photographer": "string",
            "copyright": "string",
            "license": "string"
          },

          "technical": {
            "width": "integer",
            "height": "integer",
            "aspectRatio": "string",
            "duration": "integer (seconds, for video/audio)",
            "bitrate": "integer (for video/audio)",
            "codec": "string",
            "colorSpace": "string",
            "exif": "object"
          },

          "preservation": {
            "originalFormat": "string",
            "archivalCopy": "boolean",
            "archivalUrl": "string",
            "preservationLevel": "standard|archival",
            "verified": "boolean",
            "lastVerified": "ISO 8601"
          },

          "access": {
            "downloadable": "boolean",
            "printable": "boolean",
            "shareable": "boolean",
            "watermark": "boolean",
            "requiresAuth": "boolean"
          },

          "engagement": {
            "views": "integer",
            "likes": "integer",
            "comments": "integer",
            "shares": "integer"
          }
        }
      ]
    }
  ],

  "smartAlbums": [
    {
      "albumId": "uuid",
      "title": "string",
      "type": "date-range|people|location|event|automatic",
      "rules": {
        "dateRange": {"from": "YYYY-MM-DD", "to": "YYYY-MM-DD"},
        "people": ["personId"],
        "locations": ["string"],
        "tags": ["string"],
        "autoUpdate": "boolean"
      }
    }
  ],

  "featured": {
    "mediaIds": ["uuid"],
    "rotationType": "sequential|random|curated",
    "updateFrequency": "daily|weekly|never"
  }
}
```

### Media Quality Standards

| Media Type | Minimum Quality | Recommended Quality | Archival Quality |
|------------|----------------|---------------------|------------------|
| Photos | 1200x800 px, 72 DPI | 2400x1600 px, 150 DPI | 4000x3000 px, 300 DPI, TIFF |
| Profile Photo | 400x400 px | 800x800 px | 1600x1600 px |
| Videos | 720p, 30fps, 2Mbps | 1080p, 60fps, 5Mbps | 4K, 60fps, 20Mbps, lossless codec |
| Audio | 128 kbps MP3 | 256 kbps AAC | FLAC lossless |
| Documents | 150 DPI scan | 300 DPI scan | 600 DPI, PDF/A-2 |

---

## Interactive Elements Schema

### Virtual Tributes Structure

```json
{
  "$schema": "https://wia-standards.org/schemas/digital-memorial/v1/tributes.json",
  "memorialId": "uuid-v4",
  "version": "1.0.0",

  "candles": [
    {
      "candleId": "uuid",
      "litBy": {
        "userId": "uuid|anonymous",
        "name": "string",
        "relationship": "string"
      },
      "litAt": "ISO 8601 datetime",
      "expiresAt": "ISO 8601 datetime",
      "duration": "integer (hours)",
      "candleType": "tea-light|votive|pillar|eternal",
      "color": "white|yellow|red|blue|purple|custom",
      "customColor": "hex color code",
      "message": "string (optional)",
      "isPublic": "boolean",
      "autoRelight": "boolean"
    }
  ],

  "flowers": [
    {
      "tributeId": "uuid",
      "sentBy": {
        "userId": "uuid|anonymous",
        "name": "string",
        "relationship": "string"
      },
      "sentAt": "ISO 8601 datetime",
      "arrangement": {
        "type": "roses|lilies|carnations|mixed|custom",
        "colors": ["string"],
        "size": "small|medium|large|wreath",
        "customization": "string"
      },
      "message": {
        "text": "string",
        "cardStyle": "string"
      },
      "visibility": "public|private",
      "displayDuration": "integer (days)"
    }
  ],

  "messages": [
    {
      "messageId": "uuid",
      "author": {
        "userId": "uuid|anonymous",
        "name": "string",
        "relationship": "string",
        "email": "string (optional, for notifications)",
        "profilePhoto": "url (optional)"
      },
      "postedAt": "ISO 8601 datetime",
      "content": {
        "text": "string",
        "format": "plaintext|markdown",
        "language": "ISO 639-1"
      },
      "attachments": [
        {
          "type": "photo|video|audio|document",
          "url": "string",
          "thumbnailUrl": "string",
          "title": "string"
        }
      ],
      "sentiment": "tribute|memory|condolence|celebration|prayer",
      "visibility": "public|family-only|private",
      "moderationStatus": "pending|approved|rejected",
      "moderatedBy": "userId",
      "moderatedAt": "ISO 8601 datetime",
      "isPinned": "boolean",
      "reactions": {
        "heart": "integer",
        "prayer": "integer",
        "candle": "integer",
        "flower": "integer"
      },
      "replies": [
        {
          "replyId": "uuid",
          "author": "object (same as message author)",
          "postedAt": "ISO 8601 datetime",
          "content": "object (same as message content)",
          "moderationStatus": "string"
        }
      ]
    }
  ],

  "guestbook": {
    "enabled": "boolean",
    "requiresModeration": "boolean",
    "allowAnonymous": "boolean",
    "allowAttachments": "boolean",
    "entries": ["messageId references"],
    "statistics": {
      "totalEntries": "integer",
      "uniqueVisitors": "integer",
      "countries": "integer",
      "lastEntry": "ISO 8601 datetime"
    }
  },

  "prayers": [
    {
      "prayerId": "uuid",
      "offeredBy": {
        "userId": "uuid|anonymous",
        "name": "string"
      },
      "offeredAt": "ISO 8601 datetime",
      "prayerType": "general|religious|spiritual|meditation",
      "tradition": "christian|jewish|islamic|buddhist|hindu|other",
      "message": "string (optional)",
      "isPublic": "boolean"
    }
  ],

  "donations": [
    {
      "donationId": "uuid",
      "donorName": "string",
      "amount": "decimal",
      "currency": "ISO 4217",
      "charity": {
        "name": "string",
        "charityId": "string"
      },
      "message": "string (optional)",
      "isAnonymous": "boolean",
      "donatedAt": "ISO 8601 datetime"
    }
  ]
}
```

### Interaction Analytics Schema

```json
{
  "memorialId": "uuid-v4",
  "period": {
    "from": "ISO 8601 datetime",
    "to": "ISO 8601 datetime"
  },

  "visits": {
    "total": "integer",
    "unique": "integer",
    "returning": "integer",
    "avgDuration": "integer (seconds)",
    "peakDates": [
      {
        "date": "YYYY-MM-DD",
        "visits": "integer",
        "reason": "anniversary|birthday|holiday"
      }
    ]
  },

  "geography": [
    {
      "country": "ISO 3166-1 alpha-2",
      "region": "string",
      "city": "string",
      "visits": "integer",
      "percentage": "decimal"
    }
  ],

  "engagement": {
    "candlesLit": "integer",
    "flowersSent": "integer",
    "messagesPosted": "integer",
    "photosViewed": "integer",
    "videosWatched": "integer",
    "sharesExternal": "integer",
    "downloadRequests": "integer"
  },

  "referrals": [
    {
      "source": "direct|search|social|obituary|email",
      "count": "integer",
      "percentage": "decimal"
    }
  ]
}
```

---

## Virtual Cemetery Integration

### Cemetery Plot Schema

```json
{
  "$schema": "https://wia-standards.org/schemas/digital-memorial/v1/cemetery.json",
  "memorialId": "uuid-v4",
  "version": "1.0.0",

  "virtualPlot": {
    "plotId": "uuid",
    "cemeteryId": "uuid",
    "cemeteryName": "string",
    "section": "string",
    "row": "string",
    "plot": "string",
    "coordinates": {
      "virtual": {
        "x": "decimal",
        "y": "decimal",
        "z": "decimal (for 3D environments)"
      },
      "physical": {
        "latitude": "decimal",
        "longitude": "decimal"
      }
    },
    "plotType": "single|companion|family|mausoleum|columbarium|memorial-garden",
    "size": {
      "width": "decimal (meters)",
      "depth": "decimal (meters)"
    },
    "acquisitionDate": "YYYY-MM-DD",
    "perpetualCare": "boolean"
  },

  "monument": {
    "type": "headstone|marker|plaque|statue|memorial-bench|tree",
    "material": "granite|marble|bronze|wood|virtual",
    "color": "string",
    "shape": "upright|flat|slant|custom",
    "dimensions": {
      "height": "decimal (meters)",
      "width": "decimal (meters)",
      "depth": "decimal (meters)"
    },
    "inscription": {
      "primaryText": "string",
      "epitaph": "string",
      "dates": "string",
      "symbols": ["string"],
      "font": "string",
      "layout": "string"
    },
    "model3D": {
      "url": "string (glTF/GLB format)",
      "preview": "url (image)",
      "polyCount": "integer",
      "fileSize": "integer (bytes)"
    },
    "customization": {
      "baseStyle": "traditional|modern|religious|military|nature|artistic",
      "decorations": ["cross|star|flower|angel|photo|qr-code"],
      "lighting": "boolean",
      "seasonalDecor": "boolean"
    }
  },

  "environment": {
    "landscaping": {
      "groundCover": "grass|gravel|flowers|stone",
      "plants": ["string"],
      "seasonalFlowers": "boolean"
    },
    "neighbors": [
      {
        "plotId": "uuid",
        "direction": "north|south|east|west|northeast|northwest|southeast|southwest",
        "distance": "decimal (meters)"
      }
    ],
    "landmarks": ["string"],
    "atmosphere": {
      "timeOfDay": "dawn|morning|noon|afternoon|evening|dusk|night",
      "season": "spring|summer|autumn|winter",
      "weather": "clear|cloudy|rainy|snowy|foggy",
      "soundscape": "url (ambient audio)"
    }
  },

  "visiting": {
    "virtualVisits": "integer",
    "lastVisited": "ISO 8601 datetime",
    "recentVisitors": [
      {
        "visitorId": "uuid|anonymous",
        "name": "string",
        "relationship": "string",
        "visitedAt": "ISO 8601 datetime",
        "duration": "integer (seconds)",
        "actions": ["viewed|lit-candle|left-flowers|said-prayer"]
      }
    ],
    "virtualRealitySupport": "boolean",
    "vrPlatforms": ["oculus|vive|webxr"]
  },

  "physicalLocation": {
    "hasPhysicalGrave": "boolean",
    "cemetery": {
      "name": "string",
      "address": "object",
      "phone": "string",
      "website": "url",
      "visitingHours": "string"
    },
    "directions": "string",
    "accessibility": {
      "wheelchairAccessible": "boolean",
      "parkingAvailable": "boolean",
      "publicTransport": "string"
    }
  }
}
```

### Cemetery Integration Standards

| Feature | Virtual Only | Hybrid (Virtual + Physical) | Physical Only |
|---------|--------------|----------------------------|---------------|
| 3D Monument | Required | Optional | N/A |
| GPS Coordinates | Virtual space | Both virtual and physical | Physical only |
| Visitor Tracking | Full analytics | Separate virtual/physical | Limited |
| Environmental Control | Full customization | Virtual only | N/A |
| QR Code Link | Points to virtual | Bridges physical to virtual | Points to memorial |

---

## Data Validation Rules

### Validation Requirements Table

| Field Category | Required Fields | Optional Fields | Validation Rules |
|----------------|-----------------|-----------------|------------------|
| Core Identity | Legal name, death date | Birth date, preferred name | Name: 1-200 chars, Dates: Valid ISO 8601 |
| Photography | Primary photo URL | Cover photo, thumbnails | URL: Valid HTTPS, Image: Valid format |
| Biography | Summary (short) | All other narrative | Summary: 1-280 chars, No profanity |
| Media Items | Media ID, type, file URL | All metadata | File size: Max 500MB, Valid mime type |
| Tributes | Author info, posted date | Message content | Message: Max 5000 chars, No spam |
| Cemetery Plot | Plot ID, cemetery ID | 3D models, environment | Coordinates: Valid decimal degrees |

### Data Quality Checks

```json
{
  "validationRules": {
    "deceased": {
      "legalName": {
        "required": true,
        "minLength": 1,
        "maxLength": 200,
        "pattern": "^[\\p{L}\\p{M}\\s.'-]+$",
        "sanitization": "trim|normalize"
      },
      "birthDate": {
        "required": false,
        "format": "YYYY-MM-DD",
        "minDate": "1800-01-01",
        "maxDate": "current date",
        "mustBeBefore": "deathDate"
      },
      "deathDate": {
        "required": true,
        "format": "YYYY-MM-DD",
        "minDate": "1800-01-01",
        "maxDate": "current date + 7 days",
        "mustBeAfter": "birthDate"
      }
    },
    "photography": {
      "primaryPhoto": {
        "required": true,
        "format": ["jpeg", "png", "webp"],
        "maxFileSize": 10485760,
        "minDimensions": {"width": 400, "height": 400},
        "aspectRatioRange": [0.5, 2.0],
        "contentModeration": true
      }
    },
    "biography": {
      "summary": {
        "short": {
          "required": true,
          "minLength": 50,
          "maxLength": 280,
          "noHTML": true
        }
      },
      "narrative": {
        "chapters": {
          "maxCount": 50,
          "maxLengthPerChapter": 100000
        }
      }
    },
    "messages": {
      "content": {
        "maxLength": 5000,
        "noSpam": true,
        "moderationRequired": true,
        "profanityFilter": "strict|moderate|none"
      }
    }
  }
}
```

---

## Code Examples

### Example 1: Creating a Basic Memorial Profile

```javascript
// Create a new memorial profile
const memorialProfile = {
  memorialId: crypto.randomUUID(),
  version: "1.0.0",
  createdAt: new Date().toISOString(),
  lastModified: new Date().toISOString(),
  status: "active",

  deceased: {
    legalName: {
      full: "Jane Marie Johnson",
      given: "Jane",
      middle: "Marie",
      family: "Johnson",
      suffix: "",
      prefix: ""
    },
    preferredName: "Jane",
    otherNames: [
      {
        name: "Jane Smith",
        type: "maiden",
        culture: "en"
      }
    ],
    birthDate: {
      date: "1945-06-15",
      precision: "exact",
      displayFormat: "June 15, 1945",
      calendar: "gregorian"
    },
    deathDate: {
      date: "2025-12-01",
      precision: "exact",
      displayFormat: "December 1, 2025",
      calendar: "gregorian"
    },
    age: {
      years: 80,
      months: 5,
      days: 16
    },
    gender: "female",
    pronouns: ["she", "her"],
    nationality: ["US"],
    religion: "Christian",
    occupation: ["Teacher", "Author"],
    epitaph: "A life devoted to teaching and inspiring others",
    motto: "Never stop learning"
  },

  photography: {
    primaryPhoto: {
      url: "https://cdn.memorial.example/photos/jane-profile.jpg",
      thumbnailUrl: "https://cdn.memorial.example/photos/jane-profile-thumb.jpg",
      alt: "Jane Johnson smiling at her retirement party",
      capturedDate: "2010-05-20",
      photographer: "Family Photo",
      license: "family-only",
      format: "jpeg",
      dimensions: {width: 1200, height: 1200},
      fileSize: 245678,
      checksums: {
        md5: "5d41402abc4b2a76b9719d911017c592",
        sha256: "2c26b46b68ffc68ff99b453c1d30413413422d706483bfa0f98a5e886266e7ae"
      }
    }
  },

  privacy: {
    visibility: "public",
    requiresAuthentication: false,
    allowSearchEngines: true,
    allowComments: true,
    allowTributes: true,
    allowMediaDownload: false,
    sensitiveContent: false,
    minorProtection: false,
    expirationDate: null
  },

  preservation: {
    preservationLevel: "premium",
    guaranteedUntil: "perpetual",
    backupFrequency: "daily",
    redundancyLevel: 3,
    archivalFormat: true,
    blockchainVerification: true
  }
};

// Validate the profile
function validateMemorialProfile(profile) {
  const errors = [];

  if (!profile.deceased.legalName.full) {
    errors.push("Legal name is required");
  }

  if (!profile.deceased.deathDate.date) {
    errors.push("Death date is required");
  }

  if (profile.deceased.birthDate.date && profile.deceased.deathDate.date) {
    const birth = new Date(profile.deceased.birthDate.date);
    const death = new Date(profile.deceased.deathDate.date);
    if (death <= birth) {
      errors.push("Death date must be after birth date");
    }
  }

  if (!profile.photography.primaryPhoto.url) {
    errors.push("Primary photo is required");
  }

  return {
    valid: errors.length === 0,
    errors: errors
  };
}

// Save the profile
const validation = validateMemorialProfile(memorialProfile);
if (validation.valid) {
  console.log("Memorial profile is valid and ready to save");
  // API call to save profile
} else {
  console.error("Validation errors:", validation.errors);
}
```

### Example 2: Building a Biography with Timeline

```javascript
// Create a comprehensive biography
const biography = {
  memorialId: memorialProfile.memorialId,
  version: "1.0.0",
  language: "en",
  format: "markdown",

  summary: {
    short: "Jane Johnson was a beloved teacher who inspired thousands of students over her 40-year career. She was also an accomplished author and devoted family matriarch.",
    medium: "Jane Marie Johnson (1945-2025) dedicated her life to education and family. Born in rural Iowa, she overcame numerous challenges to become one of the most respected educators in her state. Her passion for teaching extended beyond the classroom through her five published books on education methodology. Jane touched countless lives as a teacher, mentor, mother, and friend.",
    full: "A comprehensive biography would go here..."
  },

  narrative: {
    introduction: {
      content: `## Remembering Jane Johnson

Jane Marie Johnson lived a life that exemplified dedication, compassion, and lifelong learning. Born during the final months of World War II, she grew up in a changing America and became an agent of positive change herself through education.

Her journey from a small farmhouse in Iowa to becoming an award-winning teacher and published author is a testament to her determination and love of knowledge. But beyond her professional achievements, Jane was cherished as a loving mother, devoted wife, and generous friend who always put others first.`,
      author: "Family",
      writtenDate: "2025-12-10",
      tone: "formal"
    },

    chapters: [
      {
        chapterId: crypto.randomUUID(),
        title: "Early Years in Iowa",
        subtitle: "1945-1963",
        order: 1,
        period: {
          from: "1945-06-15",
          to: "1963-06-01",
          label: "Early Years"
        },
        content: `Jane was born on June 15, 1945, in Cedar Rapids, Iowa, to Robert and Margaret Smith. She was the youngest of four children and grew up on the family farm...

Despite limited resources, Jane's parents instilled in her a love of reading and learning. By age 7, she had read every book in the small town library...`,
        themes: ["childhood", "family", "education", "rural-life"],
        keywords: ["Iowa", "farm", "reading", "family"],
        relatedMedia: ["photo-001", "photo-002"],
        relatedPeople: ["person-001", "person-002"],
        author: "Daughter Sarah"
      },
      {
        chapterId: crypto.randomUUID(),
        title: "College and Teaching Career",
        subtitle: "1963-2010",
        order: 2,
        period: {
          from: "1963-09-01",
          to: "2010-05-31",
          label: "Career"
        },
        content: `Jane earned a scholarship to the University of Iowa, where she studied English Literature and Education...

Her teaching career spanned 40 years at Lincoln High School, where she became known for her innovative teaching methods and genuine care for her students...`,
        themes: ["education", "career", "teaching", "achievement"],
        keywords: ["university", "teaching", "students", "innovation"],
        relatedMedia: ["photo-010", "photo-015", "video-001"],
        author: "Former Student Michael Chen"
      }
    ]
  },

  timeline: [
    {
      eventId: crypto.randomUUID(),
      date: {
        start: "1945-06-15",
        end: null,
        precision: "exact"
      },
      title: "Birth",
      description: "Born in Cedar Rapids, Iowa",
      category: "birth",
      significance: "major",
      location: "Cedar Rapids, Iowa, USA",
      isPublic: true
    },
    {
      eventId: crypto.randomUUID(),
      date: {
        start: "1963-09-01",
        end: "1967-05-31",
        precision: "exact"
      },
      title: "University of Iowa",
      description: "Earned BA in English Literature and Education",
      category: "education",
      significance: "major",
      location: "Iowa City, Iowa, USA",
      isPublic: true
    },
    {
      eventId: crypto.randomUUID(),
      date: {
        start: "1968-07-20",
        end: null,
        precision: "exact"
      },
      title: "Marriage to John Johnson",
      description: "Married at First Presbyterian Church",
      category: "family",
      significance: "major",
      location: "Cedar Rapids, Iowa, USA",
      relatedPeople: ["john-johnson-uuid"],
      isPublic: true
    },
    {
      eventId: crypto.randomUUID(),
      date: {
        start: "1985-01-01",
        end: null,
        precision: "year"
      },
      title: "Published First Book",
      description: "\"Teaching with Heart: A Guide for New Educators\"",
      category: "achievement",
      significance: "major",
      isPublic: true
    },
    {
      eventId: crypto.randomUUID(),
      date: {
        start: "2010-05-31",
        end: null,
        precision: "exact"
      },
      title: "Retirement",
      description: "Retired after 40 years of teaching",
      category: "milestone",
      significance: "major",
      location: "Lincoln High School",
      isPublic: true
    }
  ],

  quotes: [
    {
      text: "The best teachers are those who never stop being students themselves.",
      context: "From her retirement speech",
      date: "2010-05-31",
      source: "Video recording of retirement ceremony",
      attribution: "verified"
    },
    {
      text: "Every child deserves to know that someone believes in them.",
      context: "Frequently said to colleagues",
      source: "Remembered by former principal",
      attribution: "remembered"
    }
  ],

  legacy: {
    impact: "Jane's influence lives on through the thousands of students she taught and the teachers she mentored. Her books continue to guide new educators, and the Jane Johnson Scholarship Fund supports aspiring teachers from rural communities.",
    causes: ["Education equality", "Literacy programs", "Rural education"],
    charities: [
      {
        name: "Rural Education Foundation",
        url: "https://ruraleducation.org",
        relationship: "Long-time supporter and board member"
      }
    ],
    memorialFunds: [
      {
        name: "Jane Johnson Teacher Scholarship",
        purpose: "Supporting education students from rural backgrounds",
        url: "https://uiowa.edu/jane-johnson-scholarship",
        donationInfo: "Tax-deductible donations accepted online"
      }
    ]
  }
};
```

### Example 3: Media Gallery Management

```javascript
// Create a photo gallery collection
const galleryCollection = {
  memorialId: memorialProfile.memorialId,
  version: "1.0.0",

  collections: [
    {
      collectionId: crypto.randomUUID(),
      title: "Life in Pictures",
      description: "A photographic journey through Jane's remarkable life",
      type: "photos",
      order: 1,
      createdDate: new Date().toISOString(),
      privacy: "public",
      itemCount: 45,

      items: [
        {
          mediaId: "photo-001",
          type: "photo",
          title: "Jane as a Young Girl",
          description: "Jane on the family farm, age 8",
          caption: "Already showing her love of books",

          file: {
            url: "https://cdn.memorial.example/media/jane-childhood-001.jpg",
            filename: "jane-childhood-001.jpg",
            mimeType: "image/jpeg",
            fileSize: 1245678,
            format: "JPEG",
            checksums: {
              md5: "098f6bcd4621d373cade4e832627b4f6",
              sha256: "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
            }
          },

          thumbnail: {
            url: "https://cdn.memorial.example/media/jane-childhood-001-thumb.jpg",
            width: 300,
            height: 400
          },

          variants: [
            {
              type: "thumbnail",
              url: "https://cdn.memorial.example/media/jane-childhood-001-thumb.jpg",
              width: 300,
              height: 400,
              fileSize: 45678
            },
            {
              type: "medium",
              url: "https://cdn.memorial.example/media/jane-childhood-001-med.jpg",
              width: 800,
              height: 1067,
              fileSize: 234567
            },
            {
              type: "original",
              url: "https://cdn.memorial.example/media/jane-childhood-001.jpg",
              width: 2400,
              height: 3200,
              fileSize: 1245678
            },
            {
              type: "archival",
              url: "https://archive.memorial.example/media/jane-childhood-001.tiff",
              width: 4000,
              height: 5333,
              fileSize: 64000000
            }
          ],

          metadata: {
            capturedDate: "1953-07-15",
            uploadedDate: new Date().toISOString(),
            uploadedBy: "daughter-sarah-uuid",
            location: {
              name: "Smith Family Farm, Cedar Rapids, Iowa",
              coordinates: {
                latitude: 41.9779,
                longitude: -91.6656
              }
            },
            peopleTagged: [
              {
                personId: memorialProfile.memorialId,
                name: "Jane Johnson",
                position: {x: 50, y: 60}
              }
            ],
            tags: ["childhood", "farm", "Iowa", "1950s", "books"],
            keywords: ["rural", "family", "education"],
            occasion: "Summer on the farm",
            photographer: "Robert Smith (father)",
            copyright: "Family copyright",
            license: "family-only"
          },

          technical: {
            width: 2400,
            height: 3200,
            aspectRatio: "3:4",
            colorSpace: "sRGB",
            exif: {
              Make: "Kodak",
              Model: "Brownie",
              DateTimeOriginal: "1953:07:15 14:30:00"
            }
          },

          preservation: {
            originalFormat: "physical-print",
            archivalCopy: true,
            archivalUrl: "https://archive.memorial.example/media/jane-childhood-001.tiff",
            preservationLevel: "archival",
            verified: true,
            lastVerified: new Date().toISOString()
          },

          access: {
            downloadable: false,
            printable: true,
            shareable: true,
            watermark: true,
            requiresAuth: false
          },

          engagement: {
            views: 1247,
            likes: 89,
            comments: 12,
            shares: 5
          }
        }
      ]
    },
    {
      collectionId: crypto.randomUUID(),
      title: "Teaching Years",
      description: "Photos from Jane's 40-year teaching career",
      type: "photos",
      order: 2,
      privacy: "public",
      itemCount: 67
    },
    {
      collectionId: crypto.randomUUID(),
      title: "Family Memories",
      description: "Cherished moments with family",
      type: "mixed",
      order: 3,
      privacy: "family",
      itemCount: 156
    }
  ],

  smartAlbums: [
    {
      albumId: crypto.randomUUID(),
      title: "1960s",
      type: "date-range",
      rules: {
        dateRange: {
          from: "1960-01-01",
          to: "1969-12-31"
        },
        autoUpdate: true
      }
    },
    {
      albumId: crypto.randomUUID(),
      title: "With Family",
      type: "people",
      rules: {
        people: ["daughter-sarah-uuid", "son-michael-uuid", "husband-john-uuid"],
        autoUpdate: true
      }
    }
  ],

  featured: {
    mediaIds: ["photo-001", "photo-015", "photo-042", "video-001"],
    rotationType: "curated",
    updateFrequency: "weekly"
  }
};

// Function to add media to gallery
function addMediaToGallery(collectionId, mediaData) {
  const collection = galleryCollection.collections.find(
    c => c.collectionId === collectionId
  );

  if (!collection) {
    throw new Error("Collection not found");
  }

  // Validate media data
  if (!mediaData.file || !mediaData.file.url) {
    throw new Error("Media file URL is required");
  }

  // Generate media ID
  mediaData.mediaId = crypto.randomUUID();

  // Set defaults
  mediaData.metadata = mediaData.metadata || {};
  mediaData.metadata.uploadedDate = new Date().toISOString();

  // Add to collection
  collection.items.push(mediaData);
  collection.itemCount = collection.items.length;

  return mediaData.mediaId;
}
```

### Example 4: Interactive Tributes System

```javascript
// Light a virtual candle
function lightCandle(memorialId, candleData) {
  const candle = {
    candleId: crypto.randomUUID(),
    litBy: {
      userId: candleData.userId || "anonymous",
      name: candleData.name,
      relationship: candleData.relationship
    },
    litAt: new Date().toISOString(),
    expiresAt: new Date(Date.now() + (24 * 60 * 60 * 1000)).toISOString(), // 24 hours
    duration: 24,
    candleType: candleData.candleType || "tea-light",
    color: candleData.color || "white",
    message: candleData.message || "",
    isPublic: candleData.isPublic !== false,
    autoRelight: candleData.autoRelight || false
  };

  return candle;
}

// Send virtual flowers
function sendFlowers(memorialId, flowerData) {
  const tribute = {
    tributeId: crypto.randomUUID(),
    sentBy: {
      userId: flowerData.userId || "anonymous",
      name: flowerData.name,
      relationship: flowerData.relationship
    },
    sentAt: new Date().toISOString(),
    arrangement: {
      type: flowerData.arrangementType || "roses",
      colors: flowerData.colors || ["red"],
      size: flowerData.size || "medium",
      customization: flowerData.customization || ""
    },
    message: {
      text: flowerData.message || "",
      cardStyle: flowerData.cardStyle || "classic"
    },
    visibility: flowerData.visibility || "public",
    displayDuration: flowerData.displayDuration || 7 // days
  };

  return tribute;
}

// Post a memorial message
function postMessage(memorialId, messageData) {
  const message = {
    messageId: crypto.randomUUID(),
    author: {
      userId: messageData.userId || "anonymous",
      name: messageData.name,
      relationship: messageData.relationship,
      email: messageData.email,
      profilePhoto: messageData.profilePhoto
    },
    postedAt: new Date().toISOString(),
    content: {
      text: messageData.text,
      format: messageData.format || "plaintext",
      language: messageData.language || "en"
    },
    attachments: messageData.attachments || [],
    sentiment: messageData.sentiment || "tribute",
    visibility: messageData.visibility || "public",
    moderationStatus: "pending",
    isPinned: false,
    reactions: {
      heart: 0,
      prayer: 0,
      candle: 0,
      flower: 0
    },
    replies: []
  };

  // Basic content moderation
  const forbiddenWords = ["spam", "advertisement"];
  const containsForbidden = forbiddenWords.some(word =>
    message.content.text.toLowerCase().includes(word)
  );

  if (containsForbidden) {
    message.moderationStatus = "rejected";
  } else if (message.content.text.length > 1000) {
    message.moderationStatus = "pending"; // Requires manual review
  } else {
    message.moderationStatus = "approved";
  }

  return message;
}

// Example usage
const newCandle = lightCandle(memorialProfile.memorialId, {
  name: "Sarah Johnson",
  relationship: "Daughter",
  candleType: "pillar",
  color: "white",
  message: "Missing you every day, Mom. Your light continues to guide us.",
  isPublic: true,
  autoRelight: false
});

const newFlowers = sendFlowers(memorialProfile.memorialId, {
  name: "Lincoln High School Class of 2005",
  relationship: "Former Students",
  arrangementType: "mixed",
  colors: ["red", "white", "blue"],
  size: "large",
  message: "Thank you for believing in us and changing our lives.",
  visibility: "public"
});

const newMessage = postMessage(memorialProfile.memorialId, {
  name: "Michael Chen",
  relationship: "Former Student",
  text: "Mrs. Johnson was the teacher who changed my life. When I was struggling in 10th grade, she stayed after school every day to help me. Because of her, I went to college and became a teacher myself. I try every day to live up to her example.",
  sentiment: "tribute",
  visibility: "public"
});
```

### Example 5: Virtual Cemetery Plot Configuration

```javascript
// Create a virtual cemetery plot
const virtualCemeteryPlot = {
  memorialId: memorialProfile.memorialId,
  version: "1.0.0",

  virtualPlot: {
    plotId: crypto.randomUUID(),
    cemeteryId: "memorial-gardens-uuid",
    cemeteryName: "Eternal Memorial Gardens",
    section: "Garden of Peace",
    row: "12",
    plot: "A",
    coordinates: {
      virtual: {
        x: 234.5,
        y: 156.8,
        z: 0.0
      },
      physical: {
        latitude: 41.9812,
        longitude: -91.6708
      }
    },
    plotType: "single",
    size: {
      width: 1.2,
      depth: 2.4
    },
    acquisitionDate: "2025-12-05",
    perpetualCare: true
  },

  monument: {
    type: "headstone",
    material: "granite",
    color: "gray",
    shape: "upright",
    dimensions: {
      height: 1.0,
      width: 0.75,
      depth: 0.15
    },
    inscription: {
      primaryText: "Jane Marie Johnson",
      epitaph: "A life devoted to teaching and inspiring others",
      dates: "June 15, 1945 - December 1, 2025",
      symbols: ["book", "flower"],
      font: "Garamond",
      layout: "centered"
    },
    model3D: {
      url: "https://cdn.memorial.example/models/monument-jane-johnson.glb",
      preview: "https://cdn.memorial.example/models/monument-jane-johnson-preview.jpg",
      polyCount: 15000,
      fileSize: 2456789
    },
    customization: {
      baseStyle: "traditional",
      decorations: ["flower", "book", "qr-code"],
      lighting: true,
      seasonalDecor: true
    }
  },

  environment: {
    landscaping: {
      groundCover: "grass",
      plants: ["roses", "lilies"],
      seasonalFlowers: true
    },
    neighbors: [
      {
        plotId: "neighbor-uuid-1",
        direction: "east",
        distance: 2.5
      }
    ],
    landmarks: ["Oak Tree Pathway", "Memorial Fountain"],
    atmosphere: {
      timeOfDay: "morning",
      season: "spring",
      weather: "clear",
      soundscape: "https://cdn.memorial.example/audio/peaceful-garden.mp3"
    }
  },

  visiting: {
    virtualVisits: 1523,
    lastVisited: new Date().toISOString(),
    recentVisitors: [
      {
        visitorId: "daughter-sarah-uuid",
        name: "Sarah Johnson",
        relationship: "Daughter",
        visitedAt: new Date(Date.now() - 3600000).toISOString(),
        duration: 300,
        actions: ["viewed", "lit-candle", "left-flowers"]
      }
    ],
    virtualRealitySupport: true,
    vrPlatforms: ["webxr", "oculus"]
  },

  physicalLocation: {
    hasPhysicalGrave: true,
    cemetery: {
      name: "Oakwood Memorial Park",
      address: {
        street: "1234 Memorial Drive",
        city: "Cedar Rapids",
        state: "Iowa",
        country: "US",
        postalCode: "52402"
      },
      phone: "+1-319-555-0123",
      website: "https://oakwoodmemorial.example",
      visitingHours: "Dawn to Dusk, Daily"
    },
    directions: "Enter through main gate, proceed to Garden of Peace section, Row 12",
    accessibility: {
      wheelchairAccessible: true,
      parkingAvailable: true,
      publicTransport: "Bus route 15 stops at cemetery entrance"
    }
  }
};

// Function to visit virtual plot
function visitVirtualPlot(plotData, visitorData) {
  const visit = {
    visitorId: visitorData.userId || crypto.randomUUID(),
    name: visitorData.name,
    relationship: visitorData.relationship,
    visitedAt: new Date().toISOString(),
    duration: 0, // Will be updated on visit end
    actions: []
  };

  // Track visit start
  console.log(`${visit.name} is visiting ${plotData.monument.inscription.primaryText}'s memorial`);

  return {
    visitId: crypto.randomUUID(),
    startedAt: visit.visitedAt,
    recordAction: (action) => {
      visit.actions.push(action);
      console.log(`Action recorded: ${action}`);
    },
    endVisit: () => {
      visit.duration = Math.floor((Date.now() - new Date(visit.visitedAt)) / 1000);
      plotData.visiting.recentVisitors.unshift(visit);
      plotData.visiting.virtualVisits++;
      plotData.visiting.lastVisited = new Date().toISOString();

      // Keep only last 50 visitors
      if (plotData.visiting.recentVisitors.length > 50) {
        plotData.visiting.recentVisitors = plotData.visiting.recentVisitors.slice(0, 50);
      }

      return visit;
    }
  };
}

// Example visit
const visit = visitVirtualPlot(virtualCemeteryPlot, {
  name: "Former Student",
  relationship: "Student"
});

visit.recordAction("viewed");
visit.recordAction("lit-candle");
visit.recordAction("said-prayer");

setTimeout(() => {
  const completedVisit = visit.endVisit();
  console.log(`Visit completed. Duration: ${completedVisit.duration} seconds`);
}, 5000);
```

---

## Best Practices

### Data Organization Guidelines

1. **Consistent Naming**: Use clear, descriptive field names following camelCase convention
2. **UUID Usage**: Use UUID v4 for all unique identifiers to ensure global uniqueness
3. **Date Formats**: Always use ISO 8601 format for dates and timestamps
4. **UTF-8 Encoding**: Ensure all text fields support international characters
5. **Validation**: Implement both client-side and server-side validation
6. **Sanitization**: Clean all user input to prevent XSS and injection attacks

### Privacy and Security

1. **Access Controls**: Implement granular permission systems
2. **Encryption**: Use HTTPS for all data transmission, encrypt sensitive data at rest
3. **Consent**: Obtain proper consent before creating memorials or sharing data
4. **Right to Delete**: Provide mechanisms for authorized deletion requests
5. **Minor Protection**: Extra safeguards for memorials of minors
6. **Sensitive Content**: Flag and handle traumatic circumstances appropriately

### Preservation Standards

1. **Format Migration**: Plan for regular migration to current standard formats
2. **Redundancy**: Maintain multiple backup copies in geographically diverse locations
3. **Checksum Verification**: Regular integrity checks using MD5 and SHA-256
4. **Archival Formats**: Store master copies in preservation-grade formats (PDF/A, TIFF, FLAC)
5. **Metadata Preservation**: Maintain comprehensive metadata for long-term discoverability
6. **Blockchain Verification**: Optional tamper-proof verification for critical data

### Cultural Sensitivity

1. **Naming Conventions**: Support multiple naming formats across cultures
2. **Calendar Systems**: Support Gregorian, Lunar, Hijri, Hebrew, and other calendars
3. **Religious Customs**: Allow customization for different religious traditions
4. **Language Support**: Provide multilingual interfaces and content
5. **Symbol Systems**: Support diverse cultural and religious symbols
6. **Mourning Practices**: Respect different cultural approaches to death and remembrance

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
