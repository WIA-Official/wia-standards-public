# WIA-IND-017 — Phase 4: Integration

> Tourism-data canonical Phase 4: ecosystem integration (GDPR + DMO partnership + ISO/TC 228 + safety + future).

# WIA-IND-017: Tourism Data Specification v1.0

> **Standard ID:** WIA-IND-017
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Industry Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Core Data Models](#2-core-data-models)
3. [Tourist Attractions](#3-tourist-attractions)
4. [Destination Information](#4-destination-information)
5. [Points of Interest](#5-points-of-interest)
6. [Visitor Analytics](#6-visitor-analytics)
7. [Crowd Density & Real-time Data](#7-crowd-density--real-time-data)
8. [Cultural Heritage](#8-cultural-heritage)
9. [Accessibility Standards](#9-accessibility-standards)
10. [Multi-language Support](#10-multi-language-support)
11. [Seasonality & Trends](#11-seasonality--trends)
12. [Local Experiences](#12-local-experiences)
13. [Safety & Health Information](#13-safety--health-information)
14. [Sustainable Tourism](#14-sustainable-tourism)
15. [Data Quality & Verification](#15-data-quality--verification)
16. [API Specifications](#16-api-specifications)
17. [Security & Privacy](#17-security--privacy)
18. [Implementation Guidelines](#18-implementation-guidelines)
19. [Use Cases](#19-use-cases)
20. [References](#20-references)

---


## 9. Accessibility Standards

### 9.1 Accessibility Information Model

Comprehensive accessibility data:

```json
{
  "wheelchair": true,
  "elevator": true,
  "accessibleParking": true,
  "accessibleRestrooms": true,
  "audioGuide": ["en", "fr", "es", "de"],
  "braille": true,
  "signLanguage": ["ASL", "FSL", "BSL"],
  "serviceAnimals": true,
  "hearingAssistance": true,
  "visualAssistance": true,
  "tactileExperiences": true,
  "sensorySensitive": {
    "available": true,
    "quietHours": "First Sunday of month, 9-10 AM"
  },
  "mobilityAids": {
    "wheelchairRental": true,
    "scooterRental": true,
    "walkingAids": true,
    "cost": "free"
  },
  "routes": {
    "accessibleRoute": true,
    "stepFreeRoute": true,
    "alternativeRoute": "Via west entrance"
  },
  "assistance": {
    "available": true,
    "advance": false,
    "contact": "+33140205317"
  },
  "notes": {
    "en": "Detailed accessibility information...",
    "fr": "Informations détaillées sur l'accessibilité..."
  },
  "rating": 4.8,
  "lastVerified": "2025-12-01"
}
```

### 9.2 WCAG Compliance

All digital tourism content SHOULD comply with WCAG 2.1 Level AA standards:

- Perceivable: Information must be presentable to users in ways they can perceive
- Operable: User interface components must be operable
- Understandable: Information and operation must be understandable
- Robust: Content must be robust enough for assistive technologies

### 9.3 Physical Accessibility Requirements

**Minimum Standards:**

1. **Entrances**
   - At least one accessible entrance
   - Width ≥ 32 inches (81 cm)
   - No steps or with ramp alternative

2. **Ramps**
   - Maximum slope: 1:12 (8.33%)
   - Handrails on both sides
   - Landing every 30 feet (9 m)

3. **Elevators**
   - Minimum car size: 54" x 36" (137 x 91 cm)
   - Door width ≥ 36 inches (91 cm)
   - Audio and visual indicators

4. **Restrooms**
   - Accessible stall with grab bars
   - Sink height ≤ 34 inches (86 cm)
   - Lever or automatic faucets

5. **Parking**
   - Accessible spaces near entrance
   - Width ≥ 96 inches (244 cm)
   - Access aisle ≥ 60 inches (152 cm)

---



## 10. Multi-language Support

### 10.1 Language Priority

**Tier 1 Languages** (Essential):
- English (en)
- Spanish (es)
- French (fr)
- German (de)
- Chinese Simplified (zh-CN)

**Tier 2 Languages** (Highly Recommended):
- Italian (it)
- Portuguese (pt)
- Japanese (ja)
- Korean (ko)
- Russian (ru)
- Arabic (ar)

**Tier 3 Languages** (Regional):
- Dutch (nl), Polish (pl), Turkish (tr), Hindi (hi), Thai (th)

### 10.2 Translation Quality

**Gold Standard:**
- Professional human translation
- Native speaker review
- Cultural adaptation
- Regular updates

**Silver Standard:**
- Professional translation
- Periodic review
- Context-aware

**Bronze Standard:**
- Machine translation with human review
- Basic accuracy

### 10.3 Content Localization

Beyond literal translation:
- Cultural adaptation
- Date/time formats
- Currency conversion
- Measurement units
- Local customs and etiquette
- Region-specific information

### 10.4 Multilingual Data Structure

```json
{
  "name": "Eiffel Tower",
  "names": {
    "en": "Eiffel Tower",
    "fr": "Tour Eiffel",
    "es": "Torre Eiffel",
    "de": "Eiffelturm",
    "it": "Torre Eiffel",
    "pt": "Torre Eiffel",
    "ja": "エッフェル塔",
    "ko": "에펠탑",
    "zh": "艾菲尔铁塔",
    "zh-CN": "艾菲尔铁塔",
    "zh-TW": "艾菲爾鐵塔",
    "ar": "برج إيفل",
    "ru": "Эйфелева башня"
  },
  "description": {
    "en": "Iconic iron lattice tower...",
    "fr": "Tour en treillis de fer emblématique...",
    "ja": "象徴的な鉄骨塔..."
  }
}
```

---



## 12. Local Experiences

### 12.1 Local Experience Model

```json
{
  "id": "string",
  "name": "string",
  "description": {},
  "type": "workshop",
  "location": {},
  "duration": 180,
  "groupSize": {
    "min": 2,
    "max": 10
  },
  "price": {
    "amount": 75,
    "currency": "EUR"
  },
  "languages": ["en", "fr"],
  "rating": 4.9,
  "includes": [
    "All materials",
    "Light refreshments",
    "Take-home creation"
  ],
  "requirements": [
    "Minimum age: 12",
    "No prior experience needed"
  ],
  "difficulty": "easy",
  "bookingRequired": true,
  "bookingUrl": "https://example.com/book",
  "sustainable": true,
  "authentic": true,
  "provider": {
    "name": "Local Artisan Cooperative",
    "contact": {}
  }
}
```

### 12.2 Experience Types

1. **Tours**
   - Walking tours
   - Bicycle tours
   - Food tours
   - Historical tours
   - Nature tours

2. **Workshops**
   - Cooking classes
   - Art workshops
   - Craft workshops
   - Photography classes

3. **Classes**
   - Language lessons
   - Dance classes
   - Music lessons
   - Sports instruction

4. **Tastings**
   - Wine tasting
   - Food tasting
   - Coffee/tea tasting
   - Beer tasting

5. **Performances**
   - Traditional music
   - Dance performances
   - Theater
   - Cultural shows

6. **Adventures**
   - Hiking
   - Water sports
   - Rock climbing
   - Zip-lining

7. **Cultural Immersion**
   - Home visits
   - Festival participation
   - Religious ceremonies
   - Community projects

### 12.3 Authenticity Criteria

**Authentic Local Experiences:**
- Led by local residents
- Showcases local culture/traditions
- Benefits local community
- Small group size
- Off-the-beaten-path
- Sustainable practices
- Fair compensation for locals

---



## 13. Safety & Health Information

### 13.1 Safety Information Model

```json
{
  "overall": 4.0,
  "crimeLevel": "low",
  "advisories": [
    {
      "level": 2,
      "message": "Exercise increased caution",
      "source": "US State Department",
      "date": "2025-11-15",
      "details": "Petty crime in tourist areas"
    }
  ],
  "healthRisks": ["seasonal flu"],
  "vaccinations": {
    "required": [],
    "recommended": ["routine", "hepatitis-a"]
  },
  "emergency": {
    "police": "17",
    "ambulance": "15",
    "fire": "18",
    "tourist": "112"
  },
  "safeAreas": [
    "1st Arrondissement",
    "7th Arrondissement",
    "16th Arrondissement"
  ],
  "areasToAvoid": [
    "Certain areas after dark"
  ],
  "tips": {
    "en": "Watch for pickpockets in crowded areas...",
    "fr": "Attention aux pickpockets dans les zones bondées..."
  }
}
```

### 13.2 Travel Advisory Levels

```
Level 1: Exercise Normal Precautions
Level 2: Exercise Increased Caution
Level 3: Reconsider Travel
Level 4: Do Not Travel
```

### 13.3 Health Information

**Required Data:**
- Vaccination requirements
- Health risks (malaria, dengue, etc.)
- Water safety
- Food safety
- Air quality
- Medical facilities availability
- Emergency medical contacts
- Travel insurance recommendations

### 13.4 Emergency Contacts

Every destination MUST provide:
- Police
- Ambulance
- Fire department
- Tourist police/helpline
- Embassy/consulate contacts
- Poison control
- Coast guard (coastal areas)
- Mountain rescue (mountainous areas)

---



## 14. Sustainable Tourism

### 14.1 Sustainability Metrics

```json
{
  "sustainabilityScore": 4.2,
  "certifications": [
    "Green Key",
    "EU Ecolabel",
    "Travelife Gold"
  ],
  "environmental": {
    "carbonFootprint": {
      "value": 2.5,
      "unit": "kg CO2e per visitor"
    },
    "energySource": {
      "renewable": 75,
      "nonRenewable": 25
    },
    "wasteManagement": {
      "recyclingRate": 80,
      "compostingRate": 15
    },
    "waterConservation": {
      "waterSaving": 40
    }
  },
  "social": {
    "localEmployment": 85,
    "fairWages": true,
    "communityBenefit": true
  },
  "economic": {
    "localProcurement": 70,
    "supportLocalBusiness": true
  }
}
```

### 14.2 Eco-certifications

**Recognized Certifications:**
- Green Key
- EU Ecolabel
- Travelife
- EarthCheck
- Green Globe
- LEED
- ISO 14001
- Rainforest Alliance

### 14.3 Carbon Footprint

Calculate and display carbon footprint for:
- Transportation to/from destination
- Accommodation
- Activities
- Food and dining
- Shopping

Provide carbon offset options.

### 14.4 Capacity Management

**Sustainable Visitor Management:**
- Daily visitor limits
- Time-slot reservations
- Peak hour pricing
- Off-season incentives
- Alternative destination suggestions
- Impact monitoring

---



## 18. Implementation Guidelines

### 18.1 Data Collection

**Best Practices:**

1. **Official Sources First**
   - Government tourism boards
   - Attraction official websites
   - UNESCO databases
   - Statistical agencies

2. **Verification Protocol**
   - Verify all data points
   - Cross-reference sources
   - Regular updates
   - User feedback loop

3. **Photography Standards**
   - High resolution (min 2000px)
   - Multiple angles
   - Different seasons/times
   - Accessibility features visible
   - Professional quality preferred

4. **Translation Guidelines**
   - Professional translators
   - Native speaker review
   - Cultural adaptation
   - Consistency checks
   - Regular updates

### 18.2 Integration Patterns

**API Integration:**
```javascript
// Example SDK usage
const tourism = new TourismDataSDK({
  apiKey: 'your-api-key',
  language: 'en',
  units: 'metric'
});

const attractions = await tourism.searchAttractions({
  location: { lat: 48.8566, lng: 2.3522 },
  radius: 5000,
  categories: ['cultural', 'historical']
});
```

**Webhook Support:**
```json
{
  "event": "crowd.density.high",
  "attractionId": "eiffel-tower",
  "data": {
    "current": 2800,
    "capacity": 3000,
    "percentage": 93
  },
  "timestamp": "2025-12-27T14:30:00Z"
}
```

### 18.3 Caching Strategy

**Cache TTL Recommendations:**
```
Attractions (basic info): 24 hours
Opening hours: 1 hour
Crowd density: 5 minutes
Visitor stats: 1 day
Reviews: 1 hour
Media: 7 days
```

### 18.4 Performance Optimization

**Best Practices:**
- CDN for media files
- Gzip compression
- Response caching
- Database indexing
- Load balancing
- Lazy loading
- Image optimization

---



## 19. Use Cases

### 19.1 Smart Tourism Mobile App

**Requirements:**
- Real-time attraction information
- Crowd avoidance features
- Personalized recommendations
- Offline map access
- Multi-language support
- Accessibility filters
- AR features

**Implementation:**
```javascript
// Get nearby attractions
const nearby = await tourism.searchAttractions({
  location: userLocation,
  radius: 2000,
  openNow: true,
  accessibility: ['wheelchair'],
  minRating: 4.0
});

// Check crowd levels
const crowd = await tourism.getCrowdDensity({
  attractionId: 'louvre-museum',
  realtime: true
});

if (crowd.level === 'high') {
  // Suggest alternative times
  const forecast = crowd.forecast;
  // Show recommendations
}
```

### 19.2 Destination Management System

**Features:**
- Capacity management
- Visitor flow optimization
- Revenue tracking
- Sustainability metrics
- Marketing analytics

### 19.3 Accessibility Travel Platform

**Features:**
- Comprehensive accessibility filters
- Verified accessibility data
- User reviews from disabled travelers
- Route planning with accessibility
- Equipment rental information

### 19.4 Cultural Heritage Preservation

**Features:**
- Digital heritage catalogs
- Conservation tracking
- Educational resources
- Virtual tours
- Crowdsourced monitoring

### 19.5 Sustainable Tourism Portal

**Features:**
- Carbon footprint calculator
- Eco-certified attractions
- Sustainable transportation
- Local community experiences
- Impact reporting

---



## 20. References

### 20.1 Standards

- ISO 639-1: Language codes
- ISO 3166-1: Country codes
- ISO 4217: Currency codes
- ISO 8601: Date and time format
- WGS84 (EPSG:4326): Geographic coordinates
- WCAG 2.1: Web accessibility
- GDPR: Data protection
- Schema.org: Structured data

### 20.2 Organizations

- UNWTO: United Nations World Tourism Organization
- UNESCO: United Nations Educational, Scientific and Cultural Organization
- WTTC: World Travel & Tourism Council
- PATA: Pacific Asia Travel Association
- IATA: International Air Transport Association

### 20.3 Industry Resources

- TripAdvisor API Documentation
- Google Places API
- OpenStreetMap
- Wikidata
- GeoNames

### 20.4 Accessibility Standards

- ADA: Americans with Disabilities Act
- EN 301 549: European accessibility standard
- ISO 21542: Accessibility in buildings
- WCAG 2.1: Web Content Accessibility Guidelines

---

## Appendix A: Data Schema

Complete JSON schemas available at:
`https://schemas.wiastandards.com/tourism/v1/`

**Available Schemas:**
- attraction.schema.json
- destination.schema.json
- poi.schema.json
- visitor-stats.schema.json
- crowd-density.schema.json
- accessibility.schema.json
- heritage-site.schema.json
- local-experience.schema.json

---

## Appendix B: API Examples

Complete API documentation and interactive examples:
`https://api.wiastandards.com/tourism/v1/docs`

---

## Appendix C: Code Examples

Sample implementations in multiple languages:
- JavaScript/TypeScript
- Python
- Java
- PHP
- Ruby
- Go
- Swift
- Kotlin

Available at: `https://github.com/WIA-Official/tourism-data-examples`

---

## Appendix D: Glossary

**Accessibility:** Design enabling people with disabilities to access services
**Attraction:** Place of tourist interest
**Crowd Density:** Number of people per unit area
**Destination:** Geographic location for travel
**Heritage Site:** Culturally/naturally significant place
**POI:** Point of Interest
**Seasonality:** Variation in tourism patterns over time
**Sustainable Tourism:** Environmentally and socially responsible travel
**UNESCO:** United Nations agency for education, science, and culture
**Visitor Analytics:** Statistical analysis of tourism data
**WGS84:** World Geodetic System 1984 coordinate system

---

## Appendix E: Change Log

**Version 1.0.0 (2025-12-27)**
- Initial release
- Core data models defined
- API specifications established
- Accessibility standards integrated
- Multi-language support framework
- Sustainability metrics included

---

## Appendix F: Future Enhancements

**Planned for v1.1:**
- AR/VR integration guidelines
- Blockchain for review verification
- AI-powered personalization
- IoT sensor integration
- Predictive analytics framework

**Planned for v2.0:**
- Metaverse tourism standards
- Neural interface compatibility
- Quantum encryption support
- Interplanetary tourism framework

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

*This specification is maintained by the WIA Industry Standards Group. For updates, contributions, or questions, visit https://wiastandards.com or https://github.com/WIA-Official/wia-standards*



---

## A.1 GDPR / privacy integration

Visitor-data processing follows GDPR Articles 5, 6, 13, 25, and 32 plus equivalents in CCPA, PIPL (China), PIPA (Korea), APPI (Japan), LGPD (Brazil), and PIPEDA (Canada). The integration envelope at Phase 2 §A.5 captures the lawful-basis declaration (consent for marketing analytics; legitimate interest for safety routing; contract for reservations), the retention-policy envelope (typically 90-day tenant retention for raw events; indefinite for aggregate counts at k>=20), and the data-subject-rights envelope (access, rectification, erasure, portability) with response SLAs.

## A.2 Partner-DMO integration

Destination Marketing Organisations integrate via the partnership envelope, which carries the partnership-tier (referral, integrated-content, white-label), the API rate-limit override, the data-share clauses (which fields the partner can read; which the partner can publish back), and the SLA. The integration model preserves the operator-of-record concept: the originating partner remains the authoritative source for attraction records they own, and other partners surface the attraction via reference rather than copy.

## A.3 Multi-language and accessibility cross-walk

| Concern                     | Standard                                |
|-----------------------------|------------------------------------------|
| Language tagging            | IETF BCP 47                              |
| Tourism vocabulary          | ISO/TC 228 (tourism and related services)|
| Accessibility audit         | WCAG 2.2 + ICF                           |
| Currency representation     | ISO 4217                                 |
| Country/sub-division codes  | ISO 3166-1 / 3166-2                      |
| Geo coordinates             | WGS 84 (EPSG:4326)                       |
| Heritage classification     | UNESCO WHC + Getty AAT                   |
| Sustainable tourism         | GSTC destination criteria                |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.4 Safety and incident integration

Safety-advisory ingestion pulls from UK FCDO, US Department of State, KR MOFA, JP MOFA, EU EEAS Crisis Response, plus the WHO Disease Outbreak News feed. The integration envelope normalises the advisory tiers across sources into a single 1-4 scale (1: normal precautions, 4: do not travel) and exposes both the unified tier and the per-source raw advisories so SDKs can display a defensible synthesised score with an audit trail.

## A.5 Future directions

Active research tracks: AI-generated personalised itineraries with on-device privacy (federated learning of preference embeddings), AR overlays for cultural-heritage interpretation, IoT-based visitor-flow management at scale, blockchain-anchored heritage-provenance for movable cultural property, and real-time multilingual translation pipelines for guide audio. The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4.

## A.6 Reference list

- ISO/TC 228 — tourism and related services
- IETF BCP 47 / RFC 5646 — tags for identifying languages
- ISO 3166-1 / 3166-2 — country codes and country sub-division codes
- ISO 4217 — currency codes
- WGS 84 (EPSG:4326) — geodetic reference frame
- UNESCO World Heritage Convention 1972 + 1992 cultural-landscape extension
- Getty AAT — Art & Architecture Thesaurus
- WCAG 2.2 — Web Content Accessibility Guidelines
- GSTC Destination Criteria v2.0
- GDPR Regulation (EU) 2016/679 + national equivalents (CCPA, PIPL, PIPA, APPI, LGPD, PIPEDA)
- IPCC AR6 — emission factors for transport modes


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/tourism-data/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-tourism-data-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/tourism-data-host:1.0.0` ships every tourism-data envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/tourism-data.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Tourism-data deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
