# WIA-DIGITAL_MEMORIAL PHASE 2 — API Interface Specification

**Standard:** WIA-DIGITAL_MEMORIAL
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

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
