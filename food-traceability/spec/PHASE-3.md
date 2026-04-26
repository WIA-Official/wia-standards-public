# WIA-AGRI-016: Food Traceability Standard
## PHASE 3: IoT Integration & Real-Time Monitoring

**Version:** 1.0
**Status:** Stable
**Last Updated:** 2025-12-26

---

## 1. Overview

PHASE 3 introduces real-time monitoring capabilities through IoT sensor integration, cold chain tracking, predictive quality analytics, and advanced recall management. This phase enables proactive quality management and instant visibility across the supply chain.

### 1.1 Objectives

- Implement IoT sensor networks for real-time data
- Monitor cold chain compliance continuously
- Predict quality issues before they occur
- Enable rapid recall and withdrawal procedures
- Integrate AI/ML for pattern detection

### 1.2 Scope

PHASE 3 covers:
- Temperature, humidity, and environmental sensors
- Real-time location tracking (GPS, RFID, BLE)
- Automated alert systems
- Predictive quality modeling
- Rapid recall management tools
- Cross-facility batch correlation
- Advanced analytics dashboards

---

## 2. IoT Sensor Integration

### 2.1 Sensor Types

| Sensor Type | Measurement | Use Case | Frequency |
|-------------|-------------|----------|-----------|
| Temperature | -20°C to 50°C | Cold chain monitoring | 1 min |
| Humidity | 0-100% RH | Storage conditions | 5 min |
| Light | Lux | Photosensitive products | 10 min |
| Shock/Impact | G-force | Handling quality | Event-based |
| GPS | Lat/Long | Real-time location | 5 min |
| Gas (O2, CO2) | ppm | Modified atmosphere | 15 min |
| Vibration | Frequency/amplitude | Transport conditions | Continuous |

### 2.2 Sensor Data Format

```json
{
  "sensorId": "TEMP-SENSOR-12345",
  "sensorType": "temperature",
  "deviceModel": "TempTrack Pro v2.1",
  "associatedBatch": "01234567890128.LOT2025001",
  "associatedContainer": "urn:epc:id:sscc:0123456.7890123456",
  "timestamp": "2025-12-26T10:15:30Z",
  "location": {
    "latitude": 47.6062,
    "longitude": -122.3321,
    "accuracy": 5,
    "gln": "1234567890123"
  },
  "readings": {
    "temperature": {
      "value": 4.2,
      "unit": "celsius",
      "threshold": {
        "min": 2.0,
        "max": 6.0
      },
      "status": "normal"
    },
    "humidity": {
      "value": 75,
      "unit": "percent",
      "status": "normal"
    }
  },
  "battery": {
    "level": 87,
    "estimatedDaysRemaining": 45
  },
  "calibrationDate": "2025-11-01",
  "nextCalibration": "2026-05-01"
}
```

### 2.3 Real-Time Streaming

```javascript
// WebSocket connection for real-time sensor data
const sensorStream = new WebSocket('wss://iot.wia-trace.org/stream');

sensorStream.on('message', (data) => {
  const reading = JSON.parse(data);

  // Check thresholds
  if (reading.readings.temperature.value < reading.readings.temperature.threshold.min ||
      reading.readings.temperature.value > reading.readings.temperature.threshold.max) {

    // Trigger alert
    triggerAlert({
      severity: 'high',
      type: 'temperature_deviation',
      batchId: reading.associatedBatch,
      sensorId: reading.sensorId,
      currentValue: reading.readings.temperature.value,
      threshold: reading.readings.temperature.threshold,
      location: reading.location,
      timestamp: reading.timestamp
    });
  }

  // Store in time-series database
  storeTimeSeriesData(reading);
});
```

---

## 3. Cold Chain Monitoring

### 3.1 Temperature Excursion Detection

```json
{
  "excursionId": "EXCUR-20251226-001",
  "batchId": "01234567890128.LOT2025001",
  "containerId": "urn:epc:id:sscc:0123456.7890123456",
  "excursionType": "temperature_high",
  "detectedAt": "2025-12-26T10:30:00Z",
  "duration": 450,
  "durationUnit": "seconds",
  "severity": "medium",
  "details": {
    "threshold": {
      "max": 6.0,
      "unit": "celsius"
    },
    "peak": {
      "value": 8.5,
      "timestamp": "2025-12-26T10:33:00Z"
    },
    "deviation": 2.5,
    "timeOutOfRange": 450
  },
  "location": {
    "gln": "1234567890123",
    "name": "Transport Truck #456",
    "latitude": 47.6062,
    "longitude": -122.3321
  },
  "qualityImpact": {
    "estimated": "low",
    "shelfLifeReduction": 0.5,
    "shelfLifeReductionUnit": "days",
    "actionRequired": "quality_inspection"
  },
  "actions": [
    {
      "type": "notification",
      "recipients": ["warehouse-manager@example.com", "qc-team@example.com"],
      "sentAt": "2025-12-26T10:31:00Z"
    },
    {
      "type": "flag_for_inspection",
      "status": "pending",
      "assignedTo": "QC-Inspector-7"
    }
  ]
}
```

### 3.2 Time-Temperature Integration (TTI)

```javascript
// Calculate remaining shelf life based on temperature history
function calculateRemainingShelfLife(batchId) {
  const temperatureHistory = getTemperatureHistory(batchId);
  const product = getProductInfo(batchId);

  let remainingShelfLife = product.nominalShelfLife; // days

  temperatureHistory.forEach((reading) => {
    const temp = reading.temperature;
    const duration = reading.duration; // hours

    // Q10 model: reaction rate doubles every 10°C
    const referenceTemp = 4; // °C
    const q10 = 2.5;
    const tempDiff = temp - referenceTemp;

    const accelerationFactor = Math.pow(q10, tempDiff / 10);
    const effectiveTime = duration * accelerationFactor;

    remainingShelfLife -= (effectiveTime / 24); // convert to days
  });

  return {
    batchId: batchId,
    nominalShelfLife: product.nominalShelfLife,
    remainingShelfLife: Math.max(0, remainingShelfLife),
    degradation: ((product.nominalShelfLife - remainingShelfLife) / product.nominalShelfLife * 100).toFixed(2),
    recommendation: remainingShelfLife < 7 ? 'expedite_sale' : 'normal_distribution'
  };
}
```

---

## 4. Predictive Quality Analytics

### 4.1 Quality Prediction Model

```json
{
  "predictionId": "PRED-20251226-100",
  "batchId": "01234567890128.LOT2025001",
  "generatedAt": "2025-12-26T12:00:00Z",
  "model": {
    "name": "FreshProduce_QualityML_v3.2",
    "algorithm": "Random Forest Regression",
    "accuracy": 94.5,
    "lastTrained": "2025-12-01"
  },
  "inputs": {
    "currentTemperature": 4.2,
    "avgTemperatureLast24h": 4.5,
    "tempExcursionCount": 0,
    "humidityAvg": 75,
    "daysInTransit": 3,
    "harvestDate": "2025-12-01",
    "varietyRiskScore": 3.2,
    "historicalDefectRate": 2.1
  },
  "predictions": {
    "qualityScoreAt7Days": 92,
    "qualityScoreAt14Days": 84,
    "qualityScoreAt21Days": 71,
    "probDefectAt7Days": 3.5,
    "probDefectAt14Days": 8.2,
    "estimatedOptimalSaleDate": "2026-01-05",
    "confidence": 91
  },
  "recommendations": [
    "Maintain current temperature range (2-6°C)",
    "Prioritize distribution to high-turnover retailers",
    "Schedule quality inspection by 2026-01-03"
  ]
}
```

### 4.2 Anomaly Detection

```javascript
// Detect unusual patterns in sensor data
class AnomalyDetector {
  constructor(batchId) {
    this.batchId = batchId;
    this.baselineModel = this.loadBaselineModel();
  }

  async detectAnomalies() {
    const recentData = await this.getRecentSensorData(24); // last 24 hours

    const anomalies = [];

    // Statistical outlier detection
    const tempStats = this.calculateStats(recentData.map(r => r.temperature));
    const zscore = (value, mean, stddev) => (value - mean) / stddev;

    recentData.forEach(reading => {
      const z = zscore(reading.temperature, tempStats.mean, tempStats.stddev);

      if (Math.abs(z) > 3) {
        anomalies.push({
          type: 'statistical_outlier',
          timestamp: reading.timestamp,
          parameter: 'temperature',
          value: reading.temperature,
          zscore: z,
          severity: Math.abs(z) > 4 ? 'high' : 'medium'
        });
      }
    });

    // Pattern-based anomalies
    const patterns = this.detectPatterns(recentData);
    if (patterns.rapidFluctuation) {
      anomalies.push({
        type: 'rapid_fluctuation',
        severity: 'high',
        description: 'Unstable temperature control detected',
        recommendation: 'Check refrigeration unit'
      });
    }

    return {
      batchId: this.batchId,
      analysisTime: new Date().toISOString(),
      anomaliesDetected: anomalies.length,
      anomalies: anomalies
    };
  }
}
```

---

## 5. Rapid Recall Management

### 5.1 Recall Initiation

```json
{
  "recallId": "RECALL-2025-001",
  "initiatedAt": "2025-12-26T14:00:00Z",
  "initiatedBy": {
    "name": "Food Safety Director",
    "userId": "FSD-123",
    "reason": "Precautionary recall - potential contamination"
  },
  "scope": {
    "product": "Organic Apple Juice",
    "gtin": "01234567890140",
    "affectedBatches": [
      "01234567890140.LOT2025200",
      "01234567890140.LOT2025201",
      "01234567890140.LOT2025202"
    ],
    "productionDates": {
      "start": "2025-12-05",
      "end": "2025-12-07"
    },
    "estimatedUnits": 15000,
    "estimatedRetailValue": 75000
  },
  "severity": "Class II",
  "distribution": {
    "countries": ["USA", "Canada"],
    "states": ["WA", "OR", "CA", "BC"],
    "retailers": 250,
    "distributionCenters": 12
  },
  "status": "in_progress",
  "timeline": {
    "initiated": "2025-12-26T14:00:00Z",
    "notificationsSent": "2025-12-26T14:05:00Z",
    "publicAnnouncementTarget": "2025-12-26T18:00:00Z",
    "completionTarget": "2025-12-30T23:59:59Z"
  }
}
```

### 5.2 Rapid Trace & Notification

```javascript
// Instantly identify all locations of recalled product
async function executeRecall(recallId) {
  const recall = await getRecallDetails(recallId);

  // Trace all affected batches
  const traceResults = await Promise.all(
    recall.scope.affectedBatches.map(batch => traceBatchDistribution(batch))
  );

  // Aggregate all locations
  const affectedLocations = new Set();
  const affectedCustomers = new Set();

  traceResults.forEach(trace => {
    trace.currentLocations.forEach(loc => affectedLocations.add(loc.gln));
    trace.customerPurchases.forEach(cust => affectedCustomers.add(cust.id));
  });

  // Send notifications
  const notifications = [];

  // Notify retailers
  for (const gln of affectedLocations) {
    const location = await getLocationDetails(gln);
    notifications.push(
      sendRecallNotification({
        recipient: location.contacts.foodSafety,
        method: ['email', 'sms', 'app_push'],
        urgency: 'high',
        recallDetails: recall,
        actionRequired: 'Remove from shelves immediately',
        reportingUrl: `https://recall.wia.org/report/${recallId}`
      })
    );
  }

  // Notify consumers (if identifiable through loyalty programs)
  for (const customerId of affectedCustomers) {
    const customer = await getCustomerDetails(customerId);
    notifications.push(
      sendRecallNotification({
        recipient: customer.contact,
        method: ['email', 'sms'],
        urgency: 'high',
        recallDetails: recall,
        actionRequired: 'Do not consume. Return to store for refund.',
        refundInstructions: recall.refundProcess
      })
    );
  }

  await Promise.all(notifications);

  return {
    recallId: recallId,
    locationsNotified: affectedLocations.size,
    customersNotified: affectedCustomers.size,
    notificationsSent: notifications.length,
    completedAt: new Date().toISOString()
  };
}
```

### 5.3 Recall Effectiveness Tracking

```json
{
  "recallId": "RECALL-2025-001",
  "effectivenessReport": {
    "generatedAt": "2025-12-28T10:00:00Z",
    "elapsedTime": "50 hours",
    "totalUnitsAffected": 15000,
    "unitsRecovered": 12450,
    "recoveryRate": 83.0,
    "breakdownByChannel": [
      {
        "channel": "distribution_centers",
        "unitsAffected": 8000,
        "unitsRecovered": 7800,
        "recoveryRate": 97.5
      },
      {
        "channel": "retail_stores",
        "unitsAffected": 6000,
        "unitsRecovered": 4200,
        "recoveryRate": 70.0
      },
      {
        "channel": "consumer_purchases",
        "unitsAffected": 1000,
        "unitsRecovered": 450,
        "recoveryRate": 45.0
      }
    ],
    "notificationStats": {
      "emailsSent": 350,
      "emailsOpened": 328,
      "smsSent": 150,
      "appPushSent": 200,
      "websiteVisits": 1250
    },
    "status": "on_track",
    "nextUpdate": "2025-12-29T10:00:00Z"
  }
}
```

---

## 6. Cross-Facility Batch Correlation

### 6.1 Shared Supplier Analysis

```javascript
// Identify batches from the same supplier/timeframe
async function correlateSupplierBatches(concernedBatchId) {
  const concernedBatch = await getBatchDetails(concernedBatchId);
  const inputs = await getTransformationInputs(concernedBatchId);

  const correlations = [];

  for (const input of inputs) {
    // Find other batches using the same supplier batch
    const siblingBatches = await database.query({
      collection: 'transformation_inputs',
      filter: {
        batch_id: input.batchId,
        transformation_id: { $ne: concernedBatch.transformationId }
      }
    });

    siblingBatches.forEach(sibling => {
      correlations.push({
        relatedBatchId: sibling.outputBatchId,
        sharedIngredient: input.ingredientName,
        sharedSupplierBatch: input.batchId,
        supplier: input.supplier,
        riskLevel: 'medium',
        recommendation: 'Investigate and test'
      });
    });
  }

  // Find batches processed on same equipment/facility
  const equipmentSiblings = await findBatchesSameEquipment(
    concernedBatch.processingFacility,
    concernedBatch.equipmentId,
    concernedBatch.productionDate
  );

  return {
    concernedBatch: concernedBatchId,
    correlatedBatches: correlations.length + equipmentSiblings.length,
    bySharedIngredient: correlations,
    bySharedEquipment: equipmentSiblings,
    totalRiskExposure: correlations.length + equipmentSiblings.length
  };
}
```

---

## 7. Time-Series Database Integration

### 7.1 InfluxDB Schema

```javascript
// Store sensor data in InfluxDB for efficient time-series queries
const influxClient = new InfluxDB({
  url: 'https://influx.wia-trace.org',
  token: process.env.INFLUX_TOKEN
});

async function writeSensorData(reading) {
  const point = new Point('sensor_readings')
    .tag('sensor_id', reading.sensorId)
    .tag('batch_id', reading.associatedBatch)
    .tag('sensor_type', reading.sensorType)
    .tag('location_gln', reading.location.gln)
    .floatField('temperature', reading.readings.temperature.value)
    .floatField('humidity', reading.readings.humidity.value)
    .floatField('latitude', reading.location.latitude)
    .floatField('longitude', reading.location.longitude)
    .intField('battery_level', reading.battery.level)
    .timestamp(new Date(reading.timestamp));

  await influxClient.writePoint(point);
}

// Query temperature history
async function getTemperatureHistory(batchId, hours = 24) {
  const query = `
    from(bucket: "traceability")
      |> range(start: -${hours}h)
      |> filter(fn: (r) => r._measurement == "sensor_readings")
      |> filter(fn: (r) => r.batch_id == "${batchId}")
      |> filter(fn: (r) => r._field == "temperature")
  `;

  const result = await influxClient.query(query);
  return result;
}
```

---

## 8. Analytics Dashboard

### 8.1 Real-Time Monitoring Dashboard

```javascript
// Dashboard data aggregation
async function getDashboardData() {
  const now = new Date();
  const last24h = new Date(now - 24 * 60 * 60 * 1000);

  return {
    overview: {
      activeBatches: await countActiveBatches(),
      batchesInTransit: await countBatchesInTransit(),
      activeAlerts: await countActiveAlerts(),
      temperatureExcursions: await countExcursions(last24h)
    },
    alerts: await getActiveAlerts({
      limit: 10,
      severity: ['high', 'critical']
    }),
    coldChainCompliance: {
      compliant: await calculateCompliance('compliant'),
      excursion: await calculateCompliance('excursion'),
      critical: await calculateCompliance('critical')
    },
    topRisks: await identifyTopRisks(),
    recentEvents: await getRecentEvents(20),
    sensorHealth: {
      online: await countSensors('online'),
      offline: await countSensors('offline'),
      lowBattery: await countSensors('low_battery')
    }
  };
}
```

---

## 9. API Endpoints

### 9.1 IoT Sensor APIs

```
POST /api/v1/sensors/readings
GET /api/v1/sensors/{sensorId}/readings
GET /api/v1/batches/{batchId}/sensors
GET /api/v1/sensors/{sensorId}/status
PUT /api/v1/sensors/{sensorId}/calibrate
```

### 9.2 Cold Chain APIs

```
GET /api/v1/batches/{batchId}/temperature-history
GET /api/v1/batches/{batchId}/excursions
GET /api/v1/batches/{batchId}/shelf-life
GET /api/v1/alerts?severity={level}
POST /api/v1/alerts/{alertId}/acknowledge
```

### 9.3 Recall Management APIs

```
POST /api/v1/recalls
GET /api/v1/recalls/{recallId}
POST /api/v1/recalls/{recallId}/execute
GET /api/v1/recalls/{recallId}/effectiveness
GET /api/v1/batches/{batchId}/correlations
```

---

## 10. Implementation Checklist

- [ ] Deploy IoT sensor infrastructure
- [ ] Set up time-series database (InfluxDB/TimescaleDB)
- [ ] Configure real-time alert system
- [ ] Implement predictive quality models
- [ ] Create recall management workflows
- [ ] Build analytics dashboard
- [ ] Integrate with existing ERP/WMS systems
- [ ] Train staff on alert response procedures
- [ ] Establish SLAs for alert response times
- [ ] Conduct recall simulation exercises

---

## 11. Next Steps to PHASE 4

PHASE 4 will introduce:
- Blockchain-based immutable traceability
- Global multi-enterprise networks
- AI-powered supply chain optimization
- Consumer engagement platforms
- Regulatory compliance automation

---

**Document Control:**
- **Author:** WIA Standards Committee
- **Review Cycle:** Annual
- **Next Review:** 2026-12-26

© 2025 SmileStory Inc. / WIA
弘익人間 (홍익인간) · Benefit All Humanity


## Annex E — Implementation Notes for PHASE-3

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3 with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3 does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
