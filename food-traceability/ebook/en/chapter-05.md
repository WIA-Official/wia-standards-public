# Chapter 5: IoT Sensors and Real-Time Monitoring

**WIA-AGRI-016 eBook Series**

---

## The IoT Revolution in Food Traceability

Traditional traceability captures discrete events at specific moments. IoT sensors enable **continuous monitoring** throughout the supply chain, providing real-time visibility into conditions that affect food quality and safety.

### Key Benefits

- **Proactive Quality Management:** Detect issues before they become problems
- **Cold Chain Compliance:** Continuous temperature monitoring
- **Automated Data Capture:** Reduce manual entry errors
- **Predictive Analytics:** ML models trained on sensor data
- **Instant Alerts:** Immediate notification of excursions

---

## Sensor Types for Food Traceability

### 1. Temperature Sensors

**Technology:** Thermistor, RTD, Thermocouple

**Use Cases:**
- Cold chain monitoring (refrigerated transport)
- Storage facility compliance
- Processing parameter verification
- Cooking/pasteurization validation

**Specifications:**
```
Range: -40°C to +85°C
Accuracy: ±0.2°C
Reading Interval: 1-5 minutes
Battery Life: 30-90 days
Communication: BLE, LoRaWAN, Cellular
```

**Implementation:**
```javascript
class TemperatureSensor {
  constructor(sensorId, thresholds) {
    this.sensorId = sensorId;
    this.minTemp = thresholds.min;
    this.maxTemp = thresholds.max;
    this.readings = [];
  }

  recordReading(temperature, timestamp) {
    const reading = {
      sensorId: this.sensorId,
      temperature: temperature,
      timestamp: timestamp,
      status: this.checkThreshold(temperature)
    };

    this.readings.push(reading);

    if (reading.status !== 'normal') {
      this.triggerAlert(reading);
    }

    return reading;
  }

  checkThreshold(temperature) {
    if (temperature < this.minTemp) return 'too_cold';
    if (temperature > this.maxTemp) return 'too_warm';
    return 'normal';
  }

  async triggerAlert(reading) {
    await sendAlert({
      severity: 'high',
      type: 'temperature_excursion',
      sensorId: this.sensorId,
      temperature: reading.temperature,
      threshold: { min: this.minTemp, max: this.maxTemp },
      timestamp: reading.timestamp
    });
  }
}
```

### 2. Humidity Sensors

**Applications:**
- Storage condition monitoring
- Packaging integrity
- Mold/spoilage prevention

**Data Integration:**
```json
{
  "sensorId": "HUMID-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "humidity": 75,
  "unit": "percent",
  "threshold": {
    "min": 60,
    "max": 80
  },
  "status": "normal"
}
```

### 3. GPS/Location Sensors

**Applications:**
- Real-time shipment tracking
- Route optimization
- Geofencing alerts
- Origin verification

**Example:**
```javascript
async function trackShipment(shipmentId) {
  const tracker = await getGPSTracker(shipmentId);

  // Real-time location updates
  tracker.on('location', async (location) => {
    await recordEPCISEvent({
      type: 'ObjectEvent',
      eventTime: new Date().toISOString(),
      epcList: [shipmentId],
      action: 'OBSERVE',
      bizStep: 'transporting',
      readPoint: {
        id: `geo:${location.lat},${location.lon}`,
        latitude: location.lat,
        longitude: location.lon
      }
    });

    // Check geofences
    if (isOutsideRoute(location)) {
      await alertDeviationFromRoute(shipmentId, location);
    }
  });
}
```

### 4. Shock/Impact Sensors

**Applications:**
- Handling quality monitoring
- Damage detection
- Liability determination

**Threshold-Based Alerts:**
```javascript
const impactSensor = {
  threshold: 3.0,  // G-force
  onImpact: async (gForce, timestamp, location) => {
    if (gForce > impactSensor.threshold) {
      await createQualityInspectionTask({
        reason: `Impact detected: ${gForce}G`,
        timestamp: timestamp,
        location: location,
        priority: 'high'
      });
    }
  }
};
```

---

## IoT Data Integration with EPCIS

### Sensor Data in EPCIS 2.0

```json
{
  "type": "ObjectEvent",
  "eventTime": "2025-12-26T10:30:00Z",
  "eventTimeZoneOffset": "+00:00",
  "epcList": ["urn:epc:id:sscc:0123456.1234567890"],
  "action": "OBSERVE",
  "bizStep": "transporting",
  "disposition": "in_transit",

  "sensorElementList": [
    {
      "sensorMetadata": {
        "time": "2025-12-26T10:30:00Z",
        "deviceID": "urn:epc:id:giai:0123456.SENSOR.T123",
        "deviceMetadata": "TempLogger Model XYZ v2.1"
      },
      "sensorReport": [
        {
          "type": "Temperature",
          "value": 4.2,
          "uom": "CEL",
          "minValue": 3.8,
          "maxValue": 4.5,
          "meanValue": 4.1,
          "sDev": 0.2
        },
        {
          "type": "Humidity",
          "value": 75,
          "uom": "P1"
        }
      ]
    }
  ]
}
```

### Time-Series Database Storage

Use specialized databases for sensor data:

**InfluxDB Example:**
```javascript
const { InfluxDB, Point } = require('@influxdata/influxdb-client');

class SensorDataStore {
  constructor(config) {
    this.client = new InfluxDB({
      url: config.url,
      token: config.token
    });
    this.writeApi = this.client.getWriteApi(config.org, config.bucket);
  }

  async recordTemperature(sensorId, batchId, temperature, location) {
    const point = new Point('temperature')
      .tag('sensor_id', sensorId)
      .tag('batch_id', batchId)
      .tag('location', location)
      .floatField('value', temperature)
      .timestamp(new Date());

    this.writeApi.writePoint(point);
    await this.writeApi.flush();
  }

  async queryTemperatureHistory(batchId, hours = 24) {
    const queryApi = this.client.getQueryApi(this.org);

    const query = `
      from(bucket: "traceability")
        |> range(start: -${hours}h)
        |> filter(fn: (r) => r._measurement == "temperature")
        |> filter(fn: (r) => r.batch_id == "${batchId}")
    `;

    const results = [];
    for await (const { values, tableMeta } of queryApi.iterateRows(query)) {
      const row = tableMeta.toObject(values);
      results.push(row);
    }

    return results;
  }
}
```

---

## Cold Chain Monitoring

### Temperature Excursion Detection

```javascript
class ColdChainMonitor {
  constructor(batchId, thresholds) {
    this.batchId = batchId;
    this.minTemp = thresholds.min;
    this.maxTemp = thresholds.max;
    this.excursionStart = null;
  }

  async processReading(temperature, timestamp) {
    const outOfRange = temperature < this.minTemp || temperature > this.maxTemp;

    if (outOfRange && !this.excursionStart) {
      // Excursion started
      this.excursionStart = timestamp;

      await this.createExcursionRecord({
        batchId: this.batchId,
        type: temperature < this.minTemp ? 'too_cold' : 'too_warm',
        startTime: timestamp,
        temperature: temperature
      });

    } else if (!outOfRange && this.excursionStart) {
      // Excursion ended
      const duration = (timestamp - this.excursionStart) / 1000; // seconds

      await this.finalizeExcursion({
        batchId: this.batchId,
        startTime: this.excursionStart,
        endTime: timestamp,
        duration: duration
      });

      // Calculate quality impact
      const impact = await this.calculateQualityImpact(duration, temperature);

      if (impact.shelfLifeReduction > 0) {
        await this.updateShelfLife(this.batchId, impact.shelfLifeReduction);
      }

      this.excursionStart = null;
    }
  }

  async calculateQualityImpact(duration, peakTemp) {
    // Q10 model: reaction rate doubles every 10°C
    const referenceTemp = 4;  // °C
    const q10 = 2.5;
    const tempDiff = peakTemp - referenceTemp;

    const accelerationFactor = Math.pow(q10, tempDiff / 10);
    const effectiveTime = (duration / 3600) * accelerationFactor; // hours

    return {
      shelfLifeReduction: effectiveTime / 24, // days
      qualityScore: this.calculateQualityScore(effectiveTime)
    };
  }
}
```

### Real-Time Dashboard

```javascript
async function getColdChainDashboard() {
  const activeShipments = await getActiveShipments();

  const dashboard = {
    overview: {
      totalShipments: activeShipments.length,
      compliant: 0,
      excursion: 0,
      critical: 0
    },
    shipments: []
  };

  for (const shipment of activeShipments) {
    const currentTemp = await getCurrentTemperature(shipment.trackerId);
    const history = await getTemperatureHistory(shipment.batchId, 1); // last hour

    const status = classifyTemperatureStatus(currentTemp, shipment.thresholds);

    dashboard.shipments.push({
      batchId: shipment.batchId,
      product: shipment.product,
      currentTemp: currentTemp,
      status: status,
      destination: shipment.destination,
      eta: shipment.eta,
      avgTempLastHour: calculateAverage(history)
    });

    dashboard.overview[status]++;
  }

  return dashboard;
}
```

---

## Predictive Quality Analytics

### ML Model Training

```python
import pandas as pd
from sklearn.ensemble import RandomForestRegressor

class QualityPredictionModel:
    def __init__(self):
        self.model = RandomForestRegressor(n_estimators=100)

    def train(self, historical_data):
        """
        Train model on historical sensor data + quality outcomes
        """
        features = historical_data[[
            'avg_temperature',
            'temp_excursion_count',
            'avg_humidity',
            'days_in_transit',
            'shock_events',
            'product_variety'
        ]]

        target = historical_data['final_quality_score']

        self.model.fit(features, target)

    def predict_quality(self, sensor_data):
        """
        Predict final quality based on current sensor data
        """
        features = self.extract_features(sensor_data)
        predicted_quality = self.model.predict([features])[0]

        return {
            'predicted_quality_score': round(predicted_quality, 2),
            'confidence': self.calculate_confidence(features),
            'risk_factors': self.identify_risk_factors(features, predicted_quality)
        }

    def extract_features(self, sensor_data):
        return [
            sensor_data['avg_temperature'],
            sensor_data['excursion_count'],
            sensor_data['avg_humidity'],
            sensor_data['days_in_transit'],
            sensor_data['shock_events'],
            sensor_data['product_variety_code']
        ]
```

### Real-Time Prediction

```javascript
async function predictBatchQuality(batchId) {
  // Gather sensor data
  const tempHistory = await getTemperatureHistory(batchId);
  const excursions = await getExcursions(batchId);
  const shockEvents = await getShockEvents(batchId);

  const sensorData = {
    avg_temperature: calculateAverage(tempHistory.map(r => r.value)),
    excursion_count: excursions.length,
    avg_humidity: await getAverageHumidity(batchId),
    days_in_transit: calculateDaysInTransit(batchId),
    shock_events: shockEvents.length,
    product_variety_code: await getProductVarietyCode(batchId)
  };

  // Call ML prediction service
  const prediction = await fetch('https://ml.wia-trace.org/predict', {
    method: 'POST',
    body: JSON.stringify({ batchId, sensorData })
  }).then(r => r.json());

  return {
    batchId: batchId,
    predictedQuality: prediction.predicted_quality_score,
    confidence: prediction.confidence,
    recommendations: generateRecommendations(prediction)
  };
}

function generateRecommendations(prediction) {
  const recommendations = [];

  if (prediction.predicted_quality_score < 80) {
    recommendations.push({
      priority: 'high',
      action: 'Expedite distribution to high-turnover retailers',
      reason: 'Quality degradation detected'
    });
  }

  if (prediction.risk_factors.includes('temperature_excursion')) {
    recommendations.push({
      priority: 'medium',
      action: 'Schedule quality inspection',
      reason: 'Temperature excursion detected'
    });
  }

  return recommendations;
}
```

---

## Chapter Summary

IoT sensors transform food traceability from reactive to proactive:

**Key Sensor Types:**
- Temperature (cold chain)
- Humidity (storage conditions)
- GPS (location tracking)
- Shock/impact (handling quality)

**Integration:**
- EPCIS 2.0 sensor elements
- Time-series databases
- Real-time dashboards
- Predictive analytics

**Benefits:**
- Continuous monitoring
- Automated alerts
- Quality prediction
- Reduced waste

---

## Next Chapter

**Chapter 6: Recall Management and Crisis Response**

Learn how to implement rapid recall systems and manage food safety incidents.

---

© 2025 SmileStory Inc. / WIA
弘익인間 (홍익인간) · Benefit All Humanity
