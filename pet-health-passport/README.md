# WIA Pet Health Passport

**전 세계 어디서든 인정되는 반려동물 건강 기록**
*Globally recognized pet health records*

홍익인간 (弘益人間) - Benefit All Humanity & Animals

---

## Overview

WIA Pet Health Passport is a comprehensive standard for managing pet health records that can be recognized internationally. It enables seamless pet travel, emergency veterinary care, and secure health data sharing.

### Key Features

- **Universal Identity**: ISO 11784/11785 compliant microchip identification
- **Vaccination Records**: Complete immunization history with batch tracking
- **Quarantine Eligibility**: Automatic verification for international travel
- **Emergency Access**: Quick retrieval of critical health information via QR code
- **Genetic Profile**: Breed-specific health risk assessment
- **Privacy-First**: Distributed storage using REFUGEE-CREDENTIAL structure

---

## Specifications

| Phase | Document | Description |
|-------|----------|-------------|
| 1 | [PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md) | Data structures and formats |
| 2 | [PHASE-2-ALGORITHMS.md](spec/PHASE-2-ALGORITHMS.md) | Processing algorithms |
| 3 | [PHASE-3-PROTOCOL.md](spec/PHASE-3-PROTOCOL.md) | Communication protocol |
| 4 | [PHASE-4-INTEGRATION.md](spec/PHASE-4-INTEGRATION.md) | Ecosystem integration |

---

## Quick Start

### Rust API

```bash
cd api/rust

# Build
cargo build --release

# Run server
cargo run

# Run tests
cargo test
```

### API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/health` | Health check |
| GET | `/version` | API version |
| POST | `/api/v1/passports` | Create passport |
| GET | `/api/v1/passports/:id` | Get passport |
| POST | `/api/v1/passports/:id/vaccinations` | Add vaccination |
| POST | `/api/v1/quarantine/check` | Check quarantine eligibility |
| GET | `/api/v1/emergency/:id` | Get emergency info |
| WS | `/ws` | Real-time updates |

### Example: Create Passport

```bash
curl -X POST http://localhost:8080/api/v1/passports \
  -H "Content-Type: application/json" \
  -d '{
    "pet": {
      "name": "Max",
      "species": "Dog",
      "breed": "Golden Retriever",
      "birth_date": "2020-05-15",
      "sex": "Male",
      "color": "Golden",
      "weight_kg": 32.5
    },
    "guardian": {
      "name": "John Doe",
      "email": "john@example.com",
      "phone": "+1-555-0123",
      "country": "US"
    }
  }'
```

### Example: Check Quarantine Eligibility

```bash
curl -X POST http://localhost:8080/api/v1/quarantine/check \
  -H "Content-Type: application/json" \
  -d '{
    "passport_id": "01234567-89ab-cdef-0123-456789abcdef",
    "destination_country": "AU"
  }'
```

---

## Data Model

```
PetHealthPassport
├── PetIdentity (name, species, breed, birth_date)
├── MicrochipInfo (ISO 11784/11785 compliant)
├── GuardianInfo (owner contact)
├── VaccinationRecord[] (vaccines, dates, batch numbers)
├── SurgeryRecord[] (surgeries, procedures)
├── AllergyRecord[] (known allergies)
├── ChronicCondition[] (ongoing conditions)
├── MedicationRecord[] (current medications)
├── GeneticProfile (breed-specific markers)
└── TravelRecord[] (international travel history)
```

---

## Supported Countries

The quarantine eligibility system supports:

| Country | Code | Quarantine Period | Notes |
|---------|------|-------------------|-------|
| Australia | AU | 10 days | Rabies titer required |
| Japan | JP | 0 days | 180-day waiting period |
| EU Countries | EU | 0 days | EU Pet Passport accepted |
| United States | US | 0 days | Health certificate required |
| South Korea | KR | 0 days | Rabies titer required |
| United Kingdom | GB | 0 days | AHC required |

---

## WIA Ecosystem Integration

Pet Health Passport integrates with other WIA standards:

- **WIA-INTENT**: Natural language queries for pet health
- **WIA-OMNI-API**: Universal API access
- **WIA-AIR-SHIELD**: Emergency response coordination
- **WIA-SOCIAL**: Pet community features
- **WIA-HOME**: Smart home pet monitoring
- **WIA-PQ-CRYPTO**: Post-quantum secure signatures

---

## Security

- End-to-end encryption for sensitive data
- Digital signatures for document verification
- OAuth 2.0 / DID authentication
- GDPR/HIPAA compliant data handling
- Distributed storage for data sovereignty

---

## License

MIT License

Copyright (c) 2025 WIA - World Certification Industry Association

---

**Document ID**: WIA-PET-HEALTH-PASSPORT
**Version**: 1.0.0
**Date**: 2025-12-16
