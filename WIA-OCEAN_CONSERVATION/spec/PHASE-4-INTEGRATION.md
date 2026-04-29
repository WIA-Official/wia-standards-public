# WIA-OCEAN_CONSERVATION: Phase 4 - Integration Specification
## Version 1.0 | 弘익人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines integration patterns for the WIA-OCEAN_CONSERVATION standard with external systems, including satellite data providers, marine research databases, enforcement agencies, citizen science platforms, and international conservation networks. Seamless integration enables comprehensive ocean conservation through data sharing and coordinated action.

**Philosophy**: 弘益人間 - Connected systems amplify conservation impact for all humanity.

---

## 2. Satellite Data Integration

### 2.1 Earth Observation Satellites

#### 2.1.1 Copernicus Sentinel Integration

**Service**: Copernicus Marine Environment Monitoring Service (CMEMS)

**Data Products**:
- Sea Surface Temperature (SST): Daily, 0.05° resolution
- Ocean Color (Chlorophyll-a): 300m resolution
- Sea Level Anomaly: Weekly, 0.25° resolution
- Sea Ice Concentration: Daily updates

**Integration Method**:
```javascript
// WIA SDK Integration
import { OceanConservationSDK } from '@wia/ocean-conservation';
import { CopernicusClient } from '@copernicus/marine';

const wia = new OceanConservationSDK({ apiKey: 'your-key' });
const copernicus = new CopernicusClient({ credentials });

// Fetch and integrate SST data for bleaching prediction
const sstData = await copernicus.getSST({
  bbox: [-18.5, 146.5, -17.5, 148.5],
  date: '2026-01-12'
});

// Submit to WIA for coral bleaching risk assessment
const bleachingRisk = await wia.ecosystems.assessBleachingRisk({
  reefId: 'great-barrier-reef-a12',
  temperatureData: sstData,
  threshold: 30.5 // Coral bleaching threshold in °C
});

if (bleachingRisk.level === 'HIGH') {
  await wia.alerts.create({
    type: 'BLEACHING_RISK',
    severity: 'HIGH',
    region: 'GREAT_BARRIER_REEF'
  });
}
```

**API Endpoint**: `https://marine.copernicus.eu/api/v1`

**Authentication**: OAuth 2.0

**Update Frequency**: Daily

#### 2.1.2 NASA Ocean Color Integration

**Service**: NASA Ocean Biology Processing Group (OBPG)

**Data Products**:
- MODIS Aqua: Ocean color, SST
- VIIRS: Ocean color, SST
- SeaWiFS: Historical ocean color data

**Integration Method**:
```bash
# Automated data retrieval and processing
curl -X GET "https://oceandata.sci.gsfc.nasa.gov/api/file_search" \
  -d "sensor=MODIS-Aqua" \
  -d "sdate=2026-01-01" \
  -d "edate=2026-01-12" \
  -d "dtype=L3b" \
  -d "prod=CHL" | \
  jq -r '.[] | .filename' | \
  xargs -I {} wia-ocean-conservation ingest-satellite --source nasa --file {}
```

#### 2.1.3 Planet Labs High-Resolution Imagery

**Service**: Planet Dove Constellation

**Use Cases**:
- Illegal fishing vessel detection
- Marine debris mapping
- Coastal habitat monitoring
- Coral reef change detection

**Integration Pattern**:
```javascript
// Real-time vessel detection
const planetClient = new PlanetClient({ apiKey });

// Subscribe to Area of Interest (AOI) updates
await planetClient.subscriptions.create({
  name: 'MPA-monitoring',
  source: {
    type: 'Dove',
    geometry: mpaPolygon
  },
  delivery: {
    type: 'webhook',
    url: 'https://api.wia.org/ocean-conservation/v1/webhooks/planet'
  }
});

// Webhook handler processes imagery and detects vessels
app.post('/webhooks/planet', async (req, res) => {
  const { imageId, geometry } = req.body;

  // Download and analyze image
  const vessels = await detectVessels(imageId);

  // Check against authorized vessels
  for (const vessel of vessels) {
    const authorized = await wia.mpas.checkAuthorization({
      mpaId: 'target-mpa',
      vesselLocation: vessel.location,
      timestamp: new Date()
    });

    if (!authorized) {
      await wia.enforcement.reportSuspiciousActivity({
        vessel,
        evidence: imageId,
        detectionMethod: 'SATELLITE'
      });
    }
  }

  res.sendStatus(200);
});
```

### 2.2 Satellite AIS Integration

#### 2.2.1 exactEarth Integration

**Service**: exactEarth Satellite AIS

**Coverage**: Global, including beyond coastal VHF range

**Integration Method**:
```javascript
// Real-time vessel tracking in remote areas
const exactEarth = new ExactEarthClient({ apiKey });

// Stream AIS data for specific region
const stream = exactEarth.streamAIS({
  bbox: [-60, -180, -30, -150], // Remote Pacific
  vesselTypes: ['FISHING']
});

stream.on('data', async (aisMessage) => {
  // Check if vessel is in protected area
  const inMPA = await wia.mpas.checkIntersection({
    location: {
      latitude: aisMessage.lat,
      longitude: aisMessage.lon
    }
  });

  if (inMPA.result) {
    // Check authorization
    const authorized = await wia.mpas.checkAuthorization({
      mpaId: inMPA.mpaId,
      vesselId: aisMessage.mmsi,
      activity: 'FISHING'
    });

    if (!authorized.compliant) {
      await wia.enforcement.reportViolation({
        vesselId: aisMessage.mmsi,
        violation: 'MPA_INTRUSION',
        evidence: aisMessage
      });
    }
  }
});
```

---

## 3. Marine Research Database Integration

### 3.1 OBIS (Ocean Biogeographic Information System)

**Purpose**: Global database of marine species distributions

**Integration Type**: Bidirectional data exchange

**Data Flow**:
- **Export to OBIS**: Species observations from WIA → OBIS
- **Import from OBIS**: Historical species data OBIS → WIA

**Export Configuration**:
```javascript
// Automated export of verified observations
const obis = new OBISClient({ credentials });

// Fetch verified observations from WIA
const observations = await wia.species.getObservations({
  verified: true,
  exportedToOBIS: false,
  limit: 1000
});

// Transform to Darwin Core format
const darwinCoreRecords = observations.map(obs => ({
  occurrenceID: obs.observationId,
  scientificName: obs.scientificName,
  decimalLatitude: obs.location.latitude,
  decimalLongitude: obs.location.longitude,
  eventDate: obs.timestamp,
  basisOfRecord: 'HumanObservation',
  institutionCode: 'WIA',
  collectionCode: 'OCEAN-CONSERVATION'
}));

// Submit to OBIS
await obis.uploadRecords(darwinCoreRecords);

// Mark as exported in WIA
await wia.species.updateObservations({
  ids: observations.map(o => o.observationId),
  exportedToOBIS: true
});
```

### 3.2 GBIF (Global Biodiversity Information Facility)

**Purpose**: Biodiversity data aggregation and sharing

**Integration Method**: IPT (Integrated Publishing Toolkit)

**Setup**:
```bash
# Configure IPT instance for WIA data
wia-ocean-conservation configure-ipt \
  --endpoint https://ipt.wia.org \
  --dataset "WIA Marine Species Observations" \
  --license CC-BY-4.0 \
  --updateFrequency MONTHLY

# Publish dataset to GBIF
wia-ocean-conservation publish-gbif \
  --dataset wia-marine-species \
  --from 2025-01-01 \
  --to 2025-12-31
```

### 3.3 WoRMS (World Register of Marine Species)

**Purpose**: Taxonomic authority for marine species

**Integration**: Species name validation and synonymy

**Usage**:
```javascript
// Validate species names against WoRMS
const worms = new WoRMSClient();

async function validateSpeciesName(scientificName) {
  const result = await worms.matchNames([scientificName]);

  if (result[0].match_type === 'exact') {
    return {
      valid: true,
      aphiaID: result[0].AphiaID,
      acceptedName: result[0].scientificname,
      authority: result[0].authority
    };
  } else if (result[0].match_type === 'near') {
    return {
      valid: false,
      suggestion: result[0].scientificname,
      similarity: result[0].similarity
    };
  }
}

// Integrate into observation submission
app.post('/api/species/observations', async (req, res) => {
  const validation = await validateSpeciesName(req.body.scientificName);

  if (!validation.valid) {
    return res.status(400).json({
      error: 'Invalid species name',
      suggestion: validation.suggestion
    });
  }

  req.body.aphiaID = validation.aphiaID;
  const observation = await wia.species.submitObservation(req.body);
  res.status(201).json(observation);
});
```

### 3.4 Coral Trait Database

**Purpose**: Coral species traits and functional groups

**Integration**: Enhance coral assessments with trait data

**Example**:
```javascript
// Enrich coral assessment with trait data
const coralTraits = new CoralTraitClient();

const assessment = await wia.ecosystems.getCoralAssessment('reef-123');

for (const species of assessment.dominantSpecies) {
  const traits = await coralTraits.getTraits(species.scientificName);

  species.traits = {
    growthForm: traits.growth_form,
    calcificationRate: traits.calcification_rate,
    bleachingSusceptibility: traits.bleaching_susceptibility,
    thermalTolerance: traits.thermal_tolerance
  };
}

// Use traits for vulnerability assessment
const vulnerability = assessReefVulnerability(assessment);
```

---

## 4. Enforcement Agency Integration

### 4.1 Coast Guard Integration

**Purpose**: Real-time sharing of illegal fishing detections

**Protocol**: Automated alert system with secure communication

**Integration Architecture**:
```
WIA Detection System → Secure API → Coast Guard C2 System → Response Units
```

**Implementation**:
```javascript
// Coast Guard API Integration
const coastGuard = new CoastGuardAPI({
  endpoint: process.env.COASTGUARD_API,
  certificate: fs.readFileSync('./certs/coastguard.pem'),
  key: fs.readFileSync('./certs/private.key')
});

// Alert workflow
async function reportToCoastGuard(detection) {
  // Enrich detection with additional intelligence
  const vesselHistory = await wia.enforcement.getVesselHistory(detection.vesselId);
  const threatAssessment = await assessThreat(detection, vesselHistory);

  // Submit to Coast Guard
  const caseNumber = await coastGuard.submitAlert({
    priority: threatAssessment.priority,
    vessel: {
      mmsi: detection.vessel.mmsi,
      name: detection.vessel.name,
      location: detection.location,
      heading: detection.heading,
      speed: detection.speed
    },
    violation: detection.violation,
    evidence: {
      satellite: detection.satelliteImageUrl,
      ais: detection.aisTrack,
      photos: detection.photos
    },
    recommendedAction: threatAssessment.action
  });

  // Track case in WIA system
  await wia.enforcement.linkCase({
    detectionId: detection.detectionId,
    externalCaseNumber: caseNumber,
    agency: 'COAST_GUARD'
  });

  return caseNumber;
}
```

### 4.2 Fisheries Management Organizations (FMOs)

**Organizations**:
- ICCAT (International Commission for the Conservation of Atlantic Tunas)
- WCPFC (Western and Central Pacific Fisheries Commission)
- IOTC (Indian Ocean Tuna Commission)

**Integration**: Vessel registry verification and catch reporting

**Data Exchange**:
```javascript
// Check vessel authorization against FMO registries
async function checkFMOAuthorization(vessel, location) {
  const fmo = determineFMO(location); // Returns relevant FMO
  const client = new FMOClient(fmo);

  const authorized = await client.checkVesselAuthorization({
    vesselId: vessel.imo || vessel.mmsi,
    date: new Date(),
    area: location,
    activity: 'FISHING'
  });

  return {
    authorized: authorized.status === 'ACTIVE',
    license: authorized.licenseNumber,
    quotaRemaining: authorized.quotaRemaining,
    restrictions: authorized.restrictions
  };
}
```

### 4.3 INTERPOL Environmental Security

**Purpose**: Cross-border illegal fishing investigations

**Integration**: Secure data sharing for serious violations

**Protocol**:
```javascript
// Report to INTERPOL for serious violations
async function escalateToINTERPOL(detection) {
  // Criteria for INTERPOL reporting
  const serious = (
    detection.violation.severity === 'CRITICAL' ||
    detection.vesselHistory.violations > 3 ||
    detection.estimatedValue > 100000
  );

  if (!serious) return;

  const interpol = new INTERPOLClient({ credentials });

  await interpol.submitEnvironmentalCrime({
    type: 'ILLEGAL_FISHING',
    suspects: {
      vessel: detection.vessel,
      owner: detection.vesselOwner,
      captain: detection.captain
    },
    location: detection.location,
    evidence: detection.evidence,
    estimatedDamage: detection.estimatedDamage,
    requestingAgency: 'WIA-OCEAN-CONSERVATION'
  });
}
```

---

## 5. Citizen Science Platform Integration

### 5.1 iNaturalist Integration

**Purpose**: Engage public in marine species observations

**Integration**: Bidirectional sync of marine observations

**Setup**:
```javascript
// Sync iNaturalist marine observations to WIA
const iNat = new iNaturalistClient({ apiKey });

// Fetch new marine observations
const observations = await iNat.getObservations({
  iconic_taxa: 'Animalia',
  place_id: 'marine',
  created_since: '2026-01-01',
  quality_grade: 'research'
});

// Import to WIA
for (const obs of observations) {
  await wia.species.submitObservation({
    externalId: `inat-${obs.id}`,
    scientificName: obs.taxon.name,
    location: {
      latitude: obs.location[0],
      longitude: obs.location[1]
    },
    timestamp: obs.observed_on,
    observer: {
      type: 'CITIZEN_SCIENTIST',
      externalId: obs.user.id
    },
    media: obs.photos.map(p => ({
      type: 'PHOTO',
      url: p.url
    }))
  });
}
```

### 5.2 Zooniverse Project Integration

**Purpose**: Crowdsourced image analysis

**Use Cases**:
- Coral reef photo classification
- Marine debris identification
- Whale identification from photos

**Integration**:
```javascript
// Submit images for crowdsourced analysis
const zooniverse = new ZooniverseClient({ credentials });

// Create project for coral health assessment
const project = await zooniverse.createProject({
  name: 'WIA Coral Health Monitoring',
  description: 'Help classify coral health from reef surveys',
  workflow: {
    tasks: [
      {
        type: 'question',
        question: 'What is the overall coral cover?',
        answers: ['0-25%', '25-50%', '50-75%', '75-100%']
      },
      {
        type: 'question',
        question: 'Is there evidence of coral bleaching?',
        answers: ['None', 'Mild', 'Moderate', 'Severe']
      }
    ]
  }
});

// Upload images from WIA reef assessments
const reefImages = await wia.ecosystems.getReefImages({
  needsClassification: true,
  limit: 100
});

await zooniverse.uploadSubjects({
  projectId: project.id,
  subjects: reefImages.map(img => ({
    location: img.url,
    metadata: {
      reefId: img.reefId,
      date: img.captureDate,
      depth: img.depth
    }
  }))
});

// Retrieve classifications
const classifications = await zooniverse.getClassifications({
  projectId: project.id,
  completedOnly: true
});

// Update WIA with results
for (const classification of classifications) {
  await wia.ecosystems.updateReefAssessment({
    imageId: classification.subject.metadata.imageId,
    crowdsourcedData: {
      coralCover: classification.answers.coral_cover,
      bleaching: classification.answers.bleaching,
      confidence: classification.consensus
    }
  });
}
```

---

## 6. International Conservation Network Integration

### 6.1 IUCN Red List Integration

**Purpose**: Conservation status and threat assessment

**Integration**: Real-time species status updates

**Usage**:
```javascript
const iucn = new IUCNClient({ apiKey });

// Enrich species observations with conservation status
async function enrichWithConservationStatus(observation) {
  const assessment = await iucn.getSpeciesAssessment(
    observation.scientificName
  );

  observation.conservationStatus = {
    category: assessment.category, // CR, EN, VU, NT, LC
    criteria: assessment.criteria,
    population: assessment.populationTrend,
    threats: assessment.threats,
    actions: assessment.conservationActions
  };

  // Trigger alert for critically endangered species
  if (assessment.category === 'CR' || assessment.category === 'EN') {
    await wia.alerts.create({
      type: 'THREATENED_SPECIES_SIGHTING',
      severity: 'HIGH',
      species: observation.scientificName,
      location: observation.location
    });
  }
}
```

### 6.2 Marine Stewardship Council (MSC)

**Purpose**: Sustainable fisheries certification

**Integration**: Verify certified fisheries and traceability

**Example**:
```javascript
const msc = new MSCClient({ apiKey });

// Check if fishing activity is MSC certified
async function checkFisheryStatus(vessel, location, species) {
  const fishery = await msc.findFishery({
    species: species,
    location: location,
    gear: vessel.gearType
  });

  if (fishery && fishery.certified) {
    return {
      certified: true,
      certificateNumber: fishery.certificateNumber,
      validUntil: fishery.expiryDate,
      conditions: fishery.conditions
    };
  }

  return { certified: false };
}
```

### 6.3 UNESCO World Heritage Marine Sites

**Purpose**: Coordination for World Heritage marine site protection

**Integration**: Enhanced monitoring and reporting

**Implementation**:
```javascript
// Special monitoring for World Heritage sites
const whc = new WHCClient({ credentials });

const heritageSites = await whc.getMarineSites();

for (const site of heritageSites) {
  // Enhanced monitoring protocol
  await wia.mpas.updateMonitoring({
    mpaId: site.wiaId,
    protocol: 'WORLD_HERITAGE',
    reportingFrequency: 'MONTHLY',
    reportRecipients: [
      'whc-marine-programme@unesco.org',
      site.nationalAuthority
    ]
  });
}

// Automatic quarterly reporting to UNESCO
cron.schedule('0 0 1 */3 *', async () => {
  for (const site of heritageSites) {
    const report = await wia.reports.generate({
      mpaId: site.wiaId,
      type: 'UNESCO_WORLD_HERITAGE',
      period: 'QUARTERLY'
    });

    await whc.submitReport({
      siteId: site.whcId,
      report: report
    });
  }
});
```

---

## 7. Climate and Oceanographic Data Integration

### 7.1 NOAA Climate Data Integration

**Services**:
- NOAA Coral Reef Watch
- NOAA Ocean Acidification Program
- NOAA Sea Level Rise Viewer

**Integration**:
```javascript
const noaa = {
  coralWatch: new NOAACoralWatchClient(),
  oap: new NOAAOceanAcidificationClient(),
  slr: new NOAASeaLevelRiseClient()
};

// Coral bleaching alert degree heating weeks (DHW)
const dhw = await noaa.coralWatch.getDHW({
  bbox: [-18.5, 146.5, -17.5, 148.5], // Great Barrier Reef
  date: '2026-01-12'
});

if (dhw.max > 4) { // Bleaching likely
  await wia.alerts.create({
    type: 'BLEACHING_ALERT',
    severity: dhw.max > 8 ? 'CRITICAL' : 'HIGH',
    region: 'GREAT_BARRIER_REEF',
    data: { dhw: dhw.max }
  });
}

// Ocean acidification projections
const acidification = await noaa.oap.getProjections({
  location: { latitude: -18.0, longitude: 147.0 },
  year: 2050
});

await wia.ecosystems.updateCoralReef({
  reefId: 'gbr-sector-a12',
  projections: {
    pH2050: acidification.pH,
    aragoniteSaturation2050: acidification.omega_ar
  }
});
```

### 7.2 Argo Float Data Integration

**Purpose**: In-situ ocean temperature and salinity profiles

**Integration**: Validate and supplement monitoring data

**Usage**:
```javascript
const argo = new ArgoClient();

// Retrieve Argo float data near monitoring site
const profiles = await argo.getProfiles({
  bbox: [-19, 146, -18, 148],
  startDate: '2026-01-01',
  endDate: '2026-01-12',
  parameters: ['TEMP', 'PSAL', 'DOXY']
});

// Integrate with reef monitoring
for (const profile of profiles) {
  await wia.ecosystems.addOceanographicData({
    source: 'ARGO',
    floatId: profile.floatId,
    location: profile.location,
    timestamp: profile.date,
    temperatureProfile: profile.temp,
    salinityProfile: profile.psal,
    oxygenProfile: profile.doxy
  });
}
```

---

## 8. Mobile App Integration

### 8.1 Navigation App Integration (Navionics, Garmin)

**Purpose**: Alert boaters about MPAs and sensitive areas

**Integration**:
```javascript
// Provide MPA boundaries to navigation apps
app.get('/api/v1/mpas/geojson', async (req, res) => {
  const mpas = await wia.mpas.getAllMPAs();

  const geojson = {
    type: 'FeatureCollection',
    features: mpas.map(mpa => ({
      type: 'Feature',
      geometry: mpa.boundary,
      properties: {
        name: mpa.name,
        protectionLevel: mpa.protectionLevel,
        regulations: mpa.regulations,
        contactInfo: mpa.contactInfo
      }
    }))
  };

  res.json(geojson);
});
```

### 8.2 Scuba Diving Apps (PADI, DiveLog)

**Purpose**: Engage divers in reef monitoring

**Integration**: Submit dive site conditions

```javascript
// Receive dive log data with reef observations
app.post('/api/v1/diving/log', async (req, res) => {
  const {diveLog} = req.body;

  // Extract reef condition data
  const reefCondition = {
    reefId: diveLog.site.reefId,
    assessmentDate: diveLog.date,
    visibility: diveLog.visibility,
    temperature: diveLog.temperature,
    coralHealth: diveLog.observations.coralHealth,
    fishAbundance: diveLog.observations.fishAbundance,
    pollution: diveLog.observations.pollution,
    observer: {
      type: 'RECREATIONAL_DIVER',
      certification: diveLog.diver.certification
    }
  };

  await wia.ecosystems.submitReefObservation(reefCondition);

  res.status(201).json({
    message: 'Thank you for contributing to reef monitoring!',
    points: 10 // Gamification
  });
});
```

---

## 9. Webhook and Event-Driven Integration

### 9.1 Webhook Specifications

**Event Types**:
- `species.observation.created`
- `species.observation.verified`
- `ecosystem.alert.created`
- `mpa.violation.detected`
- `pollution.event.reported`
- `enforcement.case.resolved`

**Webhook Payload Example**:
```json
{
  "eventType": "mpa.violation.detected",
  "eventId": "evt-uuid-123",
  "timestamp": "2026-01-12T14:30:00Z",
  "data": {
    "detectionId": "detect-uuid-456",
    "mpaId": "mpa-uuid-789",
    "vessel": {
      "mmsi": "123456789",
      "name": "Unknown Vessel"
    },
    "violation": {
      "type": "UNAUTHORIZED_FISHING",
      "severity": "HIGH"
    }
  },
  "signature": "sha256=..."
}
```

**Webhook Registration**:
```bash
curl -X POST https://api.wia.org/ocean-conservation/v1/webhooks \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "url": "https://your-system.com/wia-webhook",
    "events": ["mpa.violation.detected", "ecosystem.alert.created"],
    "secret": "your-webhook-secret"
  }'
```

---

## 10. Data Security and Privacy

### 10.1 Sensitive Location Data

**Protection**: Endangered species locations are not publicly disclosed

**Access Control**:
```javascript
// Implement location fuzzing for endangered species
function protectSensitiveLocation(observation) {
  if (observation.conservationStatus === 'CR' ||
      observation.conservationStatus === 'EN') {
    // Reduce precision to 0.1 degree (~11km)
    observation.location.latitude = Math.round(observation.location.latitude * 10) / 10;
    observation.location.longitude = Math.round(observation.location.longitude * 10) / 10;
    observation.locationPrecision = 'COARSE';
    observation.exactLocationAvailable = false;
  }
  return observation;
}
```

### 10.2 Vessel Identity Protection

**Whistleblower Protection**: Anonymous reporting capability

**Implementation**:
```javascript
// Anonymous violation reporting
app.post('/api/v1/enforcement/anonymous-report', async (req, res) => {
  const report = req.body;

  // Remove identifying information
  delete report.reporterId;
  delete report.reporterLocation;

  // Generate anonymous case ID
  const caseId = await wia.enforcement.createAnonymousCase(report);

  res.json({
    caseId: caseId,
    message: 'Report submitted anonymously. Your identity is protected.'
  });
});
```

---

## 11. Performance and Scalability

### 11.1 Caching Strategy

**Implementation**:
```javascript
// Cache frequently accessed data
const redis = new Redis({ host: 'cache.wia.org' });

async function getMPAWithCache(mpaId) {
  // Check cache first
  const cached = await redis.get(`mpa:${mpaId}`);
  if (cached) return JSON.parse(cached);

  // Fetch from database
  const mpa = await wia.mpas.get(mpaId);

  // Cache for 1 hour
  await redis.setex(`mpa:${mpaId}`, 3600, JSON.stringify(mpa));

  return mpa;
}
```

### 11.2 Rate Limiting

**Configuration**:
```javascript
const rateLimit = require('express-rate-limit');

// Different limits for different endpoints
const limiter = rateLimit({
  windowMs: 15 * 60 * 1000, // 15 minutes
  max: 100, // Limit each IP to 100 requests per window
  standardHeaders: true,
  legacyHeaders: false
});

app.use('/api/v1/', limiter);

// Higher limits for verified partners
const partnerLimiter = rateLimit({
  windowMs: 15 * 60 * 1000,
  max: 10000,
  skip: (req) => req.user && req.user.tier === 'PARTNER'
});
```

---

© 2025 SmileStory Inc. / WIA
弘益人間 · Protecting Oceans for All Humanity
