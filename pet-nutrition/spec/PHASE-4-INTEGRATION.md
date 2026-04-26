# WIA-PET-009: Phase 4 - Integration & Certification

**Version:** 1.0 | **Status:** Final | **Last Updated:** 2025-12-25

## Certification Levels

### Level 1: Basic Compliance
**Requirements:**
- JSON schema validation for core entities (Pet, Nutrition, Product)
- Data format compliance
- Unit standardization (kg, kcal, ISO dates)

**Ideal For:** Pet food manufacturers, product databases, simple apps

**Certification Process:**
1. Self-assessment checklist
2. Submit JSON examples
3. Automated schema validation
4. Certificate issued within 5 business days

**Fee:** $500 (annual renewal: $200)

### Level 2: Advanced Features
**Requirements:**
- Level 1 PLUS
- RESTful API implementation
- OAuth 2.0 / JWT authentication
- Extended entities (Allergies, Diet Plans, Feeding Logs)
- Rate limiting

**Ideal For:** Veterinary clinics, diet planning apps, weight management platforms

**Certification Process:**
1. Level 1 completion
2. API documentation submission (OpenAPI spec)
3. Endpoint testing (automated + manual)
4. Security review
5. Certificate issued within 15 business days

**Fee:** $2,000 (annual renewal: $800)

### Level 3: Full Integration
**Requirements:**
- Level 2 PLUS
- Real-time data synchronization
- IoT device integration
- Smart feeder compatibility
- Security audit (penetration testing)
- Veterinary EHR interoperability

**Ideal For:** Smart feeder manufacturers, comprehensive platforms, enterprise solutions

**Certification Process:**
1. Level 2 completion
2. Integration testing with reference implementations
3. Independent security audit
4. Privacy compliance review
5. End-to-end workflow validation
6. Certificate issued within 30 business days

**Fee:** $10,000 (annual renewal: $4,000)

## Integration Patterns

### Veterinary Clinic → Smart Feeder
```
1. Veterinarian prescribes diet plan in clinic EHR
2. Diet plan exported as WIA-PET-009 JSON
3. Pet owner authorizes smart feeder access
4. Smart feeder imports diet plan via API
5. Feeder dispenses portions per plan
6. Consumption logs sync back to EHR
7. Veterinarian reviews compliance remotely
```

### Multi-System Integration
```
┌─────────────┐
│ Vet EHR     │──┐
└─────────────┘  │
                 │    ┌──────────────────┐
┌─────────────┐  ├───→│ WIA-PET-009      │
│ Mobile App  │──┤    │ Central Hub      │
└─────────────┘  │    │ (API Gateway)    │
                 │    └──────────────────┘
┌─────────────┐  │           │
│Smart Feeder │──┘           │
└─────────────┘              │
                             ↓
┌─────────────┐    ┌──────────────────┐
│Smart Scale  │───→│ Owner Dashboard  │
└─────────────┘    └──────────────────┘
```

## Certification Testing

### Automated Test Suite
```bash
# Clone WIA test harness
git clone https://github.com/WIA-Official/wia-pet-009-test-suite

# Install dependencies
npm install

# Configure your API endpoint
cp .env.example .env
# Edit .env with your API_BASE_URL and credentials

# Run Level 1 tests (data format)
npm run test:level1

# Run Level 2 tests (API)
npm run test:level2

# Run Level 3 tests (integration)
npm run test:level3
```

### Manual Test Scenarios

**Scenario 1: Weight Loss Program**
1. Create pet profile (BCS 7, 35kg, target 30kg)
2. Calculate nutritional requirements
3. Create weight loss diet plan (1,344 kcal/day)
4. Log 8 weeks of feeding
5. Update weight measurements weekly
6. Verify calorie adjustments based on progress

**Scenario 2: Multi-Pet Allergy Management**
1. Create household with 3 pets
2. Pet A: chicken allergy
3. Pet B: beef allergy  
4. Pet C: no allergies
5. Search products safe for each pet
6. Verify allergen warnings triggered correctly

## Reference Implementations

**Backend (Node.js + Express):**
```
https://github.com/WIA-Official/wia-pet-009-backend
```

**Mobile SDK (React Native):**
```
https://github.com/WIA-Official/wia-pet-009-mobile-sdk
```

**Smart Feeder (Arduino/ESP32):**
```
https://github.com/WIA-Official/wia-pet-009-iot-feeder
```

## Certification Badge

Upon certification, organizations receive:
- Digital badge (SVG, PNG formats)
- Certificate PDF
- Listing in WIA directory
- Technical support access
- Logo usage rights

**Badge Levels:**
```
🥉 Level 1 Certified - Basic
🥈 Level 2 Certified - Advanced
🥇 Level 3 Certified - Full Integration
```

## Support

**Technical Questions:** support@wia.org  
**Certification Inquiries:** certification@wia.org  
**Standards Committee:** standards@wia.org  
**Documentation:** https://docs.wia.org/pet-nutrition

## Veterinary Practice Integration

WIA-PET-009 is designed to drop into existing practice management systems:

| System type | Integration | Reference adapter |
|-------------|-------------|-------------------|
| EMR / PIMS | HL7 FHIR R4 messages, custom WIA pet-record extensions | `@wia/pet-009-fhir` |
| POS / inventory | Webhook-driven feed inventory + reorder triggers | `@wia/pet-009-shopify`, `@wia/pet-009-square` |
| Telehealth | Calendly/Acuity scheduling + ration plan attachment | `@wia/pet-009-telehealth` |
| Pet apps | OAuth 2.0 with `pet.read pet.feeding.write` scopes | `@wia/pet-009-mobile-sdk` |

Adapters MUST translate WIA-PET-009 events into the upstream format and vice versa, never lossily flatten or rename core fields.

## Privacy & Data Protection

Pet data is not "personal data" in most jurisdictions, but linkage to the human owner often is. Implementations MUST:

- Treat the owner contact block as personal data subject to GDPR / PIPA / PIPEDA / CCPA, depending on residency.
- Allow the owner to delete their account; on deletion the platform MUST anonymise associated pet records by replacing identifiable owner fields with random tokens within 30 days.
- Encrypt the owner contact block at rest with AES-GCM-256 keys held in KMS.
- Tag every analytics export with the residency tag of the originating tenant so cross-border processing remains auditable.

## Internationalisation

Recipes MUST carry locale-aware fields:

```json
{
  "name": {
    "en": "Senior Salmon & Pumpkin",
    "ko": "시니어 연어 호박",
    "ja": "シニアサーモン＆パンプキン"
  },
  "ingredients": [
    {
      "id": "salmon",
      "label": {"en": "Salmon", "ko": "연어", "ja": "サーモン"},
      "grams": 120
    }
  ]
}
```

Locale fallback order MUST be: requested → tenant default → `en`. Missing translations MUST return the fallback value rather than empty strings.

## Conformance Suite

```
wia-pet-009 conformance run \
  --target https://api.example.com \
  --suite all \
  --report ./conformance-report.json
```

Suites: schema validation (Phase 1), API replay (Phase 2), engine fixtures (Phase 3), integration smoke tests (Phase 4). A clean run is required to claim a Level 2 or Level 3 certification.

## Recall & Safety Notice Workflow

When an ingredient or commercial recipe is recalled by a regulator (FDA, AAFCO member states, FEDIAF members, or local equivalents) the platform MUST:

1. Receive the recall notice via the official feed (FDA Pet Food Recall RSS, equivalent national feeds).
2. Match affected lots / SKUs against the catalogue and tag every plan referencing the offending recipe.
3. Notify the owner and the clinic on file via webhook within 1 hour of feed publication.
4. Offer at least one alternative recipe automatically and require the owner to acknowledge the swap before the next meal.
5. Log the entire flow in the audit trail with the regulator notice ID, affected lots, and substitution choice.

## Telemetry Pipeline

Production deployments SHOULD emit metrics over OpenTelemetry to either Prometheus, Datadog, or a similar collector:

| Metric | Type | Notes |
|--------|------|-------|
| `wia_pet009_calc_total{species,outcome}` | counter | per request |
| `wia_pet009_plan_generation_duration_seconds` | histogram | p50/p95/p99 |
| `wia_pet009_recall_match_total{regulator}` | counter | recall handling |
| `wia_pet009_webhook_delivery_total{result}` | counter | success/failure |
| `wia_pet009_active_plans_gauge{tenant}` | gauge | current plan count |

Operators MUST publish a status page that aggregates these metrics into traffic-light indicators for clinics.

## Multi-Region Deployment

Clinics located in different jurisdictions SHOULD be served from the closest region. The platform MUST:

- Pin owner contact data to the residency declared at signup.
- Replicate non-personal data (recipes, nutrient tables) globally for low-latency reads.
- Provide a tenant-level `dataResidencyZone` that prevents cross-region failover when regulations require.
- Honour the `X-WIA-Region` header for explicit routing.

## Disaster Recovery

| Scenario | RTO | RPO | Mechanism |
|----------|-----|-----|-----------|
| Region failure | < 30 min | 5 min | Cross-region replication of plans + recipes |
| Recipe registry outage | < 60 min | 0 | Cached registry on every replica |
| Webhook downstream down | < 15 min | 0 | Retry queue with exponential backoff up to 24 h |

Quarterly DR drills MUST exercise at least one of the above scenarios and produce a runbook update.

## Vendor Integration Templates

Common vendor adapters published alongside the standard:

| Adapter | Direction | Description |
|---------|-----------|-------------|
| `@wia/pet-009-fhir` | bidirectional | Map WIA pet/owner records to HL7 FHIR R4 Patient/RelatedPerson with WIA `pet-009` extensions |
| `@wia/pet-009-shopify` | inbound | Sync recipe inventory + stock levels |
| `@wia/pet-009-square` | inbound | POS sales feed for clinic retail |
| `@wia/pet-009-acuity` | bidirectional | Schedule wellness visits and attach plans |
| `@wia/pet-009-mailchimp` | outbound | Owner outreach (recall notices, weight goals) |
| `@wia/pet-009-twilio` | outbound | SMS reminders for feeding schedules |
| `@wia/pet-009-stripe` | inbound | Subscription billing for nutrition plans |

Each adapter MUST publish:

- A capability matrix (what is and isn't translated)
- An end-to-end test fixture
- A retry policy (exponential, jittered, capped)
- A failure-mode table mapping adapter errors to user-visible messages

## Migration from Legacy Systems

Customers often migrate from spreadsheets or legacy clinic software. The recommended path:

1. **Discovery** — inventory existing pet records and recipe libraries.
2. **Field mapping** — produce a CSV template with the customer's columns mapped to WIA fields.
3. **Dry-run** — `wia-pet-009 import --dry-run` reports validation errors without writing data.
4. **Pilot** — migrate one clinic location, run for 30 days, gather feedback.
5. **Cutover** — full migration with double-write to legacy for 14 days.
6. **Sunset** — disable legacy writes, archive legacy data per the customer's retention policy.

The migration tooling MUST never delete legacy data; archival is the customer's responsibility, but the platform MUST hand back a clean export of the legacy snapshot for their records.

## Continuous Improvement

The certification program is not "set and forget." Implementations MUST:

- Re-run the conformance suite on every release.
- Subscribe to the WIA-PET-009 changelog feed and apply MINOR updates within 90 days.
- Honour CRITICAL security advisories within 7 days.
- Submit annual review reports to certification@wia.org.

A certification badge MAY be revoked if the implementation falls behind on these obligations.

---

**弘익人間 (홍익인간) · Benefit All Humanity**

© 2025 WIA (World Certification Industry Association)  
Standard ID: WIA-PET-009 | Version: 1.0 | Phase: 4  
Category: PET | Primary Color: Amber #F59E0B
