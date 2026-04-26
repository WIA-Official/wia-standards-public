# WIA-UNI-011: Cultural Exchange Data Standard 🎭

> **Inter-Korean Cultural Exchange Data Format & Protocol**
> Version 2.0 | Status: Active | Category: Unification/Peace (UNI)

---

## Overview

WIA-UNI-011 defines standardized data formats, APIs, and protocols for inter-Korean cultural exchange activities including:

- 🎨 **Arts & Exhibitions** - Joint art shows, theater productions, cultural showcases
- ⚽ **Sports Events** - Unified teams, athletic exchanges, competitions
- 🎵 **Music Collaborations** - Concerts, festivals, traditional/modern performances
- 🎉 **Cultural Festivals** - Shared celebrations, commemorative events
- 🏛️ **Heritage Preservation** - UNESCO sites, traditional crafts, historical documentation
- 📚 **Educational Exchange** - Student programs, academic conferences, research

### Philosophy

Embodies **홍익인간 (弘益人間)** (Hongik Ingan) - "benefit all humanity."
Cultural exchange builds bridges where politics cannot, creating sustainable peace through shared artistic expression and heritage preservation.

---

## Quick Start

### 🎮 Try the Simulator

Experience WIA-UNI-011 in action:
```
https://wiastandards.com/uni-011/simulator/
```

### 📚 Read the Documentation

Complete guide with implementation details:
```
https://wiabook.com/cultural-exchange-data/
```

### 💻 Use the TypeScript SDK

```bash
npm install @wia/cultural-exchange-data
```

```typescript
import { createClient } from '@wia/cultural-exchange-data';

const client = createClient({
  apiKey: process.env.WIA_API_KEY
});

// List upcoming music events
const events = await client.events.list({
  type: 'music',
  startDate: new Date().toISOString(),
  limit: 20
});

// Create new event
const event = await client.events.create({
  eventType: 'music',
  title: {
    ko: '남북 평화 음악회',
    en: 'Inter-Korean Peace Concert'
  },
  startDate: '2025-08-01T19:00:00+09:00',
  endDate: '2025-08-01T22:00:00+09:00',
  location: {
    name: { en: 'DMZ Peace Park', ko: 'DMZ 평화공원' },
    coordinates: { lat: 37.8813, lng: 126.7569 }
  }
});
```

---

## Key Features

### 🌐 Multilingual by Design
Native support for Korean (South and North variations), English, Hanja, and other languages.

### 🔐 Security & Privacy First
- AES-256 encryption for sensitive data
- OAuth 2.0 authentication
- Role-based access control
- GDPR and PIPA compliance
- Comprehensive audit logging

### 📊 Rich Data Model
- Cultural event schemas with comprehensive metadata
- Participant/artist profiles with privacy controls
- Heritage item documentation with regional variations
- Media asset management with rights tracking

### 🔄 Cross-Border Synchronization
Secure data exchange between North and South Korean institutions via DMZ-hosted sync servers.

### 🏛️ UNESCO Integration
Bidirectional sync with UNESCO World Heritage, Intangible Cultural Heritage, and Memory of the World databases.

---

## Architecture

### Four-Phase Implementation

| Phase | Focus | Deliverables |
|-------|-------|-------------|
| **Phase 1** | Data Format | JSON schemas, validation rules, field definitions |
| **Phase 2** | API Interface | REST APIs, SDKs (TS/Python/Java/Go), authentication |
| **Phase 3** | Protocol | Sync protocols, security, cross-border data exchange |
| **Phase 4** | Integration | UNESCO, tourism, education platforms, certification |

### Technology Stack

- **Data Formats:** JSON, JSON-LD, Protocol Buffers
- **APIs:** REST (OpenAPI 3.0), GraphQL, WebSockets, gRPC
- **Security:** TLS 1.3, OAuth 2.0, JWT, AES-256, Ed25519
- **Languages:** TypeScript, Python, Java, Go

---

## Directory Structure

```
cultural-exchange-data/
├── index.html              # Landing page
├── README.md               # This file
├── simulator/
│   └── index.html          # Interactive simulator
├── ebook/
│   ├── en/                 # English documentation (9 files)
│   │   ├── index.html
│   │   └── chapter1-8.html
│   └── ko/                 # Korean documentation (9 files)
│       ├── index.html
│       └── chapter1-8.html
├── spec/                   # Technical specifications
│   ├── WIA-UNI-011-spec-v1.0.md
│   ├── WIA-UNI-011-spec-v1.1.md
│   ├── WIA-UNI-011-spec-v1.2.md
│   └── WIA-UNI-011-spec-v2.0.md
└── api/
    └── typescript/         # TypeScript SDK
        ├── package.json
        ├── src/
        │   ├── types.ts    # Type definitions
        │   └── index.ts    # SDK implementation
        └── README.md
```

---

## Use Cases

### 🎭 Joint Theater Productions
Coordinate inter-Korean theater collaborations, traditional Pansori performances, modern drama productions, and cultural festivals.

### ⚽ Unified Sports Teams
Manage joint Korean sports delegations for Olympics, Asian Games, and World Championships.

### 🎵 K-Culture Collaborations
Enable music collaborations between North and South Korean artists, joint concerts, and cultural festivals.

### 🏛️ UNESCO Heritage Projects
Document and preserve shared Korean cultural heritage sites, traditional crafts, and intangible cultural properties.

### 📺 Media & Broadcasting
Facilitate joint documentary productions, cultural program exchanges, and broadcast cooperation.

### 🎨 Art Exhibitions
Organize joint art exhibitions, cultural showcases, museum collaborations, and traveling exhibitions.

---

## Data Model Examples

### Cultural Event

```json
{
  "standard": "WIA-UNI-011",
  "version": "2.0",
  "eventId": "UNI-011-MUSIC-2025-042",
  "eventType": "music",
  "title": {
    "ko": "남북 통일 음악 축제",
    "en": "Inter-Korean Unity Music Festival",
    "ko-north": "북남 통일 음악 축전"
  },
  "startDate": "2025-06-15T18:00:00+09:00",
  "endDate": "2025-06-17T22:00:00+09:00",
  "location": {
    "name": { "ko": "DMZ 평화공원", "en": "DMZ Peace Park" },
    "coordinates": { "lat": 37.8813, "lng": 126.7569 }
  },
  "unescoCategory": "intangible",
  "expectedParticipants": 500
}
```

### Heritage Item

```json
{
  "heritageId": "HER-KR-MUSIC-001",
  "name": {
    "ko": "아리랑",
    "en": "Arirang",
    "hanja": "阿里郞"
  },
  "category": "music",
  "unescoStatus": "registered",
  "regionalVariations": [
    {
      "region": "south",
      "localName": "정선아리랑",
      "characteristics": "Slower tempo, unique melodic structure"
    },
    {
      "region": "north",
      "localName": "평양아리랑",
      "characteristics": "Faster tempo, orchestral arrangements"
    }
  ]
}
```

---

## API Endpoints

### Events API
- `GET /events` - List events with filtering
- `GET /events/{id}` - Get event details
- `POST /events` - Create new event
- `PUT /events/{id}` - Update event
- `DELETE /events/{id}` - Delete event

### Heritage API
- `GET /heritage` - Search heritage items
- `GET /heritage/{id}` - Get heritage details
- `GET /heritage/search` - Full-text search

### Media API
- `POST /media/upload-url` - Request upload URL
- `GET /media/{id}` - Get media asset

### Analytics API
- `GET /analytics/events/trends` - Event trends
- `GET /analytics/participation` - Participation metrics

---

## Certification

Organizations can pursue official WIA-UNI-011 certification:

| Level | Requirements | Benefits |
|-------|-------------|----------|
| 🥉 **Bronze** | Phase 1 (Data Format) | Listed in WIA directory |
| 🥈 **Silver** | Phases 1-2 (+ API) | API ecosystem access |
| 🥇 **Gold** | Phases 1-3 (+ Protocol) | Cross-border sync access |
| 💎 **Platinum** | All 4 phases | Premium support, leadership recognition |

**Apply:** https://cert.wiastandards.com/uni-011

---

## Resources

### Documentation
- **Specification:** [/spec/WIA-UNI-011-spec-v2.0.md](/spec/WIA-UNI-011-spec-v2.0.md)
- **Ebook (English):** [/ebook/en/index.html](/ebook/en/index.html)
- **Ebook (Korean):** [/ebook/ko/index.html](/ebook/ko/index.html)

### Tools
- **Simulator:** [/simulator/index.html](/simulator/index.html)
- **TypeScript SDK:** [@wia/cultural-exchange-data](https://npmjs.com/@wia/cultural-exchange-data)

### Community
- **GitHub:** https://github.com/WIA-Official/wia-standards
- **Forum:** https://community.wia-standards.org
- **Slack:** wia-standards.slack.com #uni-011

### Support
- **Documentation:** https://docs.wia-standards.org/uni-011
- **Email:** support@wiastandards.com
- **Commercial Support:** Certified WIA partners

---

## Contributing

We welcome contributions! Please see:
- [CONTRIBUTING.md](../../CONTRIBUTING.md) for guidelines
- [CODE_OF_CONDUCT.md](../../CODE_OF_CONDUCT.md) for community standards

---

## License

MIT License - See [LICENSE](../../LICENSE) for details

---

## Acknowledgments

Developed by SmileStory Inc. in collaboration with:
- Ministry of Culture, Sports and Tourism (South Korea)
- UNESCO Korean National Commission
- Cultural heritage preservation organizations
- Academic and research institutions

Special thanks to cultural exchange pioneers who demonstrated that art transcends political divisions.

---

**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. · MIT License*

---

## Related Standards

- **WIA-UNI-001:** Inter-Korean Data Exchange
- **WIA-UNI-003:** Family Reunion Data
- **WIA-EDU-001:** Educational Exchange
- **WIA-MEDIA-001:** Media Distribution

*For complete WIA standards catalog: https://wiastandards.com*
