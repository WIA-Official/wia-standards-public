# WIA-DIGITAL_MEMORIAL PHASE 4 — Integration Specification

**Standard:** WIA-DIGITAL_MEMORIAL
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

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
