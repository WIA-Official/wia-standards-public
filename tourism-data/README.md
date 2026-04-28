# 🗺️ WIA-IND-017: Tourism Data Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-IND-017
> **Version:** 1.0.0
> **Status:** Active
> **Category:** IND / Industry
> **Color:** Amber (#F59E0B)

---

## 🌟 Overview

The WIA-IND-017 standard defines a comprehensive framework for tourism data management, including standardized formats for tourist attractions, visitor analytics, destination information, points of interest (POI), cultural heritage data, and real-time tourism services.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to make tourism data accessible, interoperable, and inclusive, enabling seamless travel experiences while preserving cultural heritage and supporting sustainable tourism development worldwide.

## 🎯 Key Features

- **Tourist Attraction Data**: Standardized formats for attractions, monuments, and landmarks
- **Visitor Analytics**: Real-time and historical visitor statistics and patterns
- **Destination Information**: Comprehensive destination profiles and metadata
- **POI Database**: Structured points of interest with rich attributes
- **Seasonality Data**: Tourism trends, peak seasons, and demand forecasting
- **Cultural Heritage**: UNESCO sites, heritage information, and preservation data
- **Local Experiences**: Authentic local activities and immersive experiences
- **Accessibility Information**: Comprehensive accessibility data for all abilities
- **Multi-language Support**: Content in multiple languages and scripts
- **Real-time Crowd Density**: Live tracking of tourist congestion and capacity
- **Sustainable Tourism**: Environmental impact metrics and eco-friendly options
- **Safety & Security**: Travel advisories, health information, and safety ratings

## 📊 Core Concepts

### 1. Data Categories

```
Tourism Data Hierarchy:
├── Destinations
│   ├── Countries
│   ├── Regions
│   ├── Cities
│   └── Neighborhoods
├── Attractions
│   ├── Natural (parks, beaches, mountains)
│   ├── Cultural (museums, galleries, theaters)
│   ├── Historical (monuments, heritage sites)
│   ├── Entertainment (theme parks, venues)
│   └── Religious (temples, churches, mosques)
├── Services
│   ├── Accommodation (hotels, hostels, rentals)
│   ├── Dining (restaurants, cafes, street food)
│   ├── Transportation (public, private, rental)
│   └── Activities (tours, excursions, experiences)
└── Infrastructure
    ├── Airports, stations, ports
    ├── Tourist information centers
    ├── Accessibility facilities
    └── Emergency services
```

### 2. Data Quality Tiers

| Tier | Coverage | Update Frequency | Accuracy | Languages |
|------|----------|------------------|----------|-----------|
| Gold | 100% | Real-time | >99% | 20+ |
| Silver | >90% | Hourly | >95% | 10+ |
| Bronze | >75% | Daily | >90% | 5+ |
| Basic | >50% | Weekly | >80% | 1-2 |

### 3. Visitor Metrics

```
Key Performance Indicators:
- Daily/Monthly/Annual Visitors
- Average Visit Duration
- Peak Hours/Days/Seasons
- Visitor Demographics
- Origin Countries/Regions
- Satisfaction Scores
- Revenue per Visitor
- Return Visit Rate
```

## 🔧 Components

### TypeScript SDK

```typescript
import {
  searchAttractions,
  getDestinationInfo,
  getVisitorStats,
  getCrowdDensity,
  searchPOI,
  getAccessibilityInfo,
  getCulturalHeritage
} from '@wia/ind-017';

// Search tourist attractions
const attractions = await searchAttractions({
  location: { lat: 48.8566, lng: 2.3522 }, // Paris
  radius: 5000, // 5km
  categories: ['cultural', 'historical'],
  minRating: 4.0,
  accessibility: ['wheelchair', 'audio-guide'],
  languages: ['en', 'fr', 'ja']
});

// Get real-time crowd density
const crowd = await getCrowdDensity({
  attractionId: 'louvre-museum',
  realtime: true
});

console.log(`Current occupancy: ${crowd.current}/${crowd.capacity} (${crowd.percentage}%)`);
console.log(`Wait time: ${crowd.estimatedWaitTime} minutes`);

// Get visitor statistics
const stats = await getVisitorStats({
  attractionId: 'eiffel-tower',
  period: '2024',
  granularity: 'monthly',
  metrics: ['visitors', 'revenue', 'satisfaction']
});

// Search POI with filters
const pois = searchPOI({
  bounds: {
    north: 48.9,
    south: 48.8,
    east: 2.4,
    west: 2.3
  },
  types: ['restaurant', 'cafe', 'museum'],
  priceRange: '$$',
  openNow: true,
  sustainable: true
});
```

### CLI Tool

```bash
# Search attractions near coordinates
wia-ind-017 search-attractions --lat 35.6762 --lng 139.6503 --radius 2000

# Get destination information
wia-ind-017 get-destination --id "tokyo-japan" --lang en

# Get real-time crowd data
wia-ind-017 crowd-density --attraction-id "tokyo-tower" --realtime

# Get visitor statistics
wia-ind-017 visitor-stats --attraction-id "sensoji-temple" --period 2024 --granularity monthly

# Search POI by category
wia-ind-017 search-poi --category restaurants --city tokyo --cuisine japanese --rating 4.5

# Get accessibility information
wia-ind-017 accessibility --attraction-id "national-museum-tokyo"

# Export tourism data
wia-ind-017 export --destination "kyoto-japan" --format json --output kyoto-data.json
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-IND-017-v1.0.md](./spec/WIA-IND-017-v1.0.md) | Complete tourism data specification |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-ind-017.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/tourism-data

# Run installation script
./install.sh

# Verify installation
wia-ind-017 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/ind-017

# Or yarn
yarn add @wia/ind-017
```

```typescript
import { TourismDataSDK } from '@wia/ind-017';

const sdk = new TourismDataSDK({
  apiKey: 'your-api-key',
  language: 'en',
  units: 'metric'
});

// Get destination details
const destination = await sdk.getDestination({
  id: 'barcelona-spain',
  include: ['attractions', 'weather', 'events', 'safety']
});

console.log(`${destination.name} - ${destination.tagline}`);
console.log(`Best time to visit: ${destination.bestSeasons.join(', ')}`);
console.log(`Average daily budget: ${destination.averageBudget.currency} ${destination.averageBudget.amount}`);

// Get cultural heritage sites
const heritage = await sdk.getCulturalHeritage({
  country: 'ES',
  unescoOnly: true,
  category: 'cultural'
});

heritage.forEach(site => {
  console.log(`${site.name} - UNESCO ${site.inscriptionYear}`);
  console.log(`  ${site.criteria.join(', ')}`);
});
```

## 🔬 Technical Specifications

### Data Formats

| Format | Usage | Compression | Schema |
|--------|-------|-------------|--------|
| JSON-LD | Web/API | None/GZIP | schema.org/TouristAttraction |
| GeoJSON | Mapping | None | RFC 7946 |
| Protocol Buffers | High-volume | Built-in | Custom |
| CSV/TSV | Bulk export | ZIP/GZIP | Custom |
| XML | Legacy systems | None/GZIP | Custom XSD |

### Coordinate Systems

- **WGS84 (EPSG:4326)**: Primary standard for GPS coordinates
- **Web Mercator (EPSG:3857)**: Web mapping applications
- **Local projections**: Region-specific when required

### Language Codes

Based on ISO 639-1 (2-letter) and ISO 639-3 (3-letter) codes:
- Primary: en, es, fr, de, it, ja, ko, zh, ar, ru, pt
- Regional variants: en-US, en-GB, zh-CN, zh-TW, pt-BR, pt-PT

### Rating Systems

```
Unified Rating Scale (0-5):
5.0 - Exceptional
4.5-4.9 - Excellent
4.0-4.4 - Very Good
3.5-3.9 - Good
3.0-3.4 - Average
2.5-2.9 - Below Average
2.0-2.4 - Poor
<2.0 - Very Poor
```

## ⚠️ Implementation Guidelines

1. **Data Privacy**: Comply with GDPR, CCPA, and local privacy regulations
2. **Real-time Updates**: Cache with appropriate TTL based on data volatility
3. **Accessibility**: Include comprehensive accessibility data (WCAG 2.1 Level AA)
4. **Multi-language**: Provide professional translations, not machine-translated
5. **Cultural Sensitivity**: Respect local customs, religious sites, and sacred places
6. **Sustainable Tourism**: Promote eco-friendly options and capacity management
7. **Data Quality**: Verify information through multiple sources
8. **Rate Limiting**: Implement appropriate API rate limits (1000 req/hour recommended)

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based tourism search and recommendations
- **WIA-OMNI-API**: Universal tourism API gateway
- **WIA-SOCIAL**: Social travel planning and sharing
- **WIA-AI**: AI-powered personalized travel recommendations
- **WIA-LOCATION**: Advanced geolocation and mapping services
- **WIA-TRANSLATION**: Real-time multi-language support
- **WIA-PAYMENT**: Integrated booking and payment systems
- **WIA-HEALTH**: Health and safety information integration

## 📖 Use Cases

### 1. Smart Tourism Apps
Real-time attraction information, crowd avoidance, personalized recommendations

### 2. Destination Management
Capacity planning, visitor flow optimization, sustainable tourism metrics

### 3. Cultural Preservation
Digital heritage catalogs, conservation tracking, educational resources

### 4. Accessibility Services
Comprehensive accessibility information for travelers with disabilities

### 5. Crisis Management
Real-time safety updates, emergency notifications, evacuation planning

### 6. Economic Analysis
Tourism impact studies, revenue forecasting, ROI analysis

### 7. Marketing & Promotion
Data-driven campaigns, target audience identification, seasonal strategies

### 8. Research & Academia
Tourism trends analysis, behavioral studies, policy development

## 🎨 Example Schemas

### Attraction Schema
```json
{
  "id": "attraction-001",
  "name": "Eiffel Tower",
  "type": "monument",
  "category": "historical",
  "location": {
    "coordinates": { "lat": 48.8584, "lng": 2.2945 },
    "address": "Champ de Mars, 5 Av. Anatole France, 75007 Paris",
    "city": "Paris",
    "country": "FR"
  },
  "description": {
    "en": "Iconic iron lattice tower...",
    "fr": "Tour en treillis de fer emblématique...",
    "ja": "象徴的な鉄骨塔..."
  },
  "ratings": {
    "overall": 4.7,
    "count": 125000,
    "breakdown": {
      "5": 85000,
      "4": 30000,
      "3": 8000,
      "2": 1500,
      "1": 500
    }
  },
  "visitInfo": {
    "openingHours": {
      "monday": "09:30-23:45",
      "tuesday": "09:30-23:45"
    },
    "tickets": {
      "adult": { "amount": 26.10, "currency": "EUR" },
      "child": { "amount": 13.10, "currency": "EUR" }
    },
    "averageVisitDuration": 120
  },
  "accessibility": {
    "wheelchair": true,
    "elevator": true,
    "audioGuide": ["en", "fr", "es", "de", "it", "ja", "zh"],
    "braille": true,
    "signLanguage": ["FR-SL", "IS-SL"]
  },
  "crowdData": {
    "current": 2500,
    "capacity": 3000,
    "waitTime": 45
  }
}
```

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
