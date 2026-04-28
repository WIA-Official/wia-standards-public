# WIA-IND-014 — Phase 4: Integration

> Security, privacy, and references that ground the consent-and-PII handling discipline.

## 12. Security and Privacy

### 12.1 Data Protection

**GDPR Compliance:**
- Member data encryption at rest (AES-256)
- Encryption in transit (TLS 1.3)
- Right to access, modify, delete personal data
- Data retention policies (workout data: 7 years, video: 30 days)
- Consent management for video recording in facility
- Anonymized analytics data

**Access Control:**
```typescript
interface AccessControl {
  roles: {
    member: ['view-own-data', 'book-classes', 'use-equipment'];
    trainer: ['view-client-data', 'create-plans', 'conduct-classes'];
    staff: ['checkin-members', 'manage-equipment', 'view-facility-stats'];
    manager: ['manage-members', 'financial-reports', 'staff-management'];
    admin: ['full-access'];
  };

  dataAccess: {
    ownWorkouts: 'member',
    allWorkouts: 'admin',
    personalInfo: 'member+staff',
    healthData: 'member+trainer',
    financialData: 'manager+admin',
    facilityControls: 'staff+manager+admin'
  };
}
```

### 12.2 Video & Camera Privacy

**Camera Zones:**
```typescript
interface CameraPrivacy {
  zones: {
    publicAreas: {
      recording: true;
      retention: '30days';
      purpose: 'security+analytics';
      signageRequired: true;
    };

    workoutFloor: {
      recording: true;
      retention: '7days';
      purpose: 'form-analysis+safety';
      optOut: true; // members can opt-out of form analysis
      faceBlurring: true; // auto-blur faces in stored videos
    };

    lockerRooms: {
      recording: false;
      cameras: 'entrance-only'; // never inside
    };

    privateTraining: {
      recording: 'consent-required';
      retention: '90days';
      memberOwned: true; // member owns the video
    };
  };
}
```

---


## 13. References

### 13.1 Related Standards

- **WIA-IND-012**: Fitness Tracking Standard
- **WIA-IND-011**: Sports Tech Standard
- **WIA-MED-001**: Health Records Integration
- **WIA-IOT-005**: IoT Sensor Networks
- **ISO 20957**: Stationary training equipment standards
- **EN 957**: Fitness equipment safety standards

### 13.2 Scientific References

- ACSM's Guidelines for Exercise Testing and Prescription (11th Edition)
- NSCA's Essentials of Strength Training and Conditioning (4th Edition)
- Periodization Training for Sports (Tudor Bompa, 3rd Edition)
- Science and Practice of Strength Training (Zatsiorsky & Kraemer, 3rd Edition)

### 13.3 Technical Standards

- **MQTT**: IoT messaging protocol
- **WebRTC**: Real-time video streaming
- **FHIR**: Healthcare data exchange
- **IEEE 802.11ax**: WiFi 6 standard
- **Bluetooth 5.2**: BLE connectivity

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*


## A.1 Security and privacy integration

Smart-gym data is sensitive PII (biometric measurements, location,
behavioural patterns). The standard requires:

- WIA Secure Enclave sealing for personal-fitness profiles
- WIA-OMNI-API credentials for member identity
- Per-aggregate consent envelope chain (no aggregate without
  documented member consent for inclusion)
- Lawful-intercept compatibility per jurisdiction (declared in
  discovery document)

## A.2 Cross-standard composition

This Phase composes with: WIA-OMNI-API (member identity), WIA Secure
Enclave (sealed profiles), WIA-AIR-SHIELD (trust list), WIA-SOCIAL
Phase 3 §5 (cross-facility federation), and WIA-INFRA-MONITORING for
the IoT facility-condition stream.

## A.3 Bridges to existing systems

The bridge profile maps the standard's envelopes to: Mindbody (gym
management), MyZone (heart-rate monitoring), Polar (heart-rate),
Garmin Connect (multi-modal fitness), Strava (workout sharing),
Apple HealthKit (iOS), Google Fit (Android).

## A.4 Roadmap

| Version | Focus |
|---------|-------|
| 1.0.0 | Initial publication: connected equipment, member management, virtual training stable |
| 1.1.x | Additive: AR/VR fitness experiences, gamification |
| 1.2.x | Additive: confidential analytics inside TEEs |
| 2.0.0 | Possible breaking change: post-quantum signatures |


## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-gym/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-gym-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-gym-host:1.0.0` ships every Phase 2 endpoint with mock
data for integrators to test against. The companion CLI at
`cli/smart-gym.sh` ships sample envelope generators with no dependencies
beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, signature, and
audit machinery rather than maintaining N parallel implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up reference
container; run conformance suite against it; replace mock backend
with real backend one endpoint at a time; wire audit log
replication; onboard a single trusted peer for federation; expand
to multiple peers; promote to production with warning-envelope
subscription.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the standards four-Phase architecture: Phase
1 envelopes are the wire-format contract; Phase 2 surfaces them
through HTTPS; Phase 3 wraps them in protocol exchanges that cross
trust boundaries; Phase 4 integrates with the broader ecosystem.

弘益人間 — Benefit All Humanity.


## A.5 Bridges to existing systems (detail)

The bridge profile maps the standards envelopes to:

- **Mindbody / Glofox / TeamUp** — gym management software
- **MyZone / Polar / Garmin** — heart-rate and biometric monitoring
- **Strava / Wahoo / Zwift** — workout sharing and competitive metrics
- **Apple HealthKit / Google Fit** — mobile platform health stores
- **Tonal / Tempo / Mirror** — connected home-fitness systems
- **Peloton / Echelon / NordicTrack** — connected cardio equipment

Each bridge ships at `https://github.com/WIA-Official/wia-smart-gym-bridges`.

## A.6 Roadmap

| Version | Focus |
|---------|-------|
| 1.0.0 | Initial publication: connected equipment, member management, virtual training stable |
| 1.1.x | Additive: AR/VR fitness, gamification, more bridges |
| 1.2.x | Additive: confidential analytics inside TEEs |
| 2.0.0 | Possible breaking change: post-quantum signatures |

## A.7 Closing implementer note

Smart-gym deployments balance member experience against operational
efficiency. The standards consent-envelope discipline ensures
member data is processed only with documented consent; the audit
log makes every aggregate computation traceable. Operators that
treat the discipline as a starting point rather than a ceiling
typically build deeper member trust and lower churn.

## A.8 References (detail)

- ISO/IEC 27001 — information security management
- ISO/IEC 25010 — software product quality
- IEEE Std 11073 — medical device communication (for biometric monitoring)
- HL7 FHIR R5 — health data exchange (for clinical-grade integrations)
- W3C DID Core — decentralised identifiers
- IETF RFC 8446 — TLS 1.3
- IETF RFC 8785 — JSON Canonicalization Scheme
- BIPM SI brochure — TAI conventions for time


## A.9 Operator's quick-reference card

A laminated quick-reference card for facility operators contains:
the conformance container start command, the audit log replication
status indicator, the federation peer trust list refresh schedule,
and the on-call escalation path for member-safety events.

