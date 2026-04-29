# WIA-FLOOD_PREDICTION TypeScript SDK

Official TypeScript/JavaScript SDK for the WIA-FLOOD_PREDICTION API.

AI-powered flood prediction system using satellite imagery, weather data, hydrological models, and machine learning to forecast floods 7-14 days in advance with 85%+ accuracy.

## Installation

```bash
npm install @wia/flood-prediction
# or
yarn add @wia/flood-prediction
```

## Quick Start

```typescript
import { FloodPredictionClient } from '@wia/flood-prediction';

// Initialize client with API key
const client = new FloodPredictionClient({
  apiKey: 'wia_live_your_api_key_here',
  baseURL: 'https://api.wia-flood.org/v1' // optional
});

// Get flood predictions for a location
const predictions = await client.getPredictions({
  lat: 38.9072,
  lng: -77.0369,
  radius_km: 20,
  min_probability: 0.5
});

console.log(predictions);
// [{
//   id: 'pred_2026011114_38.9072_-77.0369',
//   location: { lat: 38.9072, lng: -77.0369 },
//   risk_level: 'high',
//   probability: 0.87,
//   predicted_depth_meters: 2.4,
//   ...
// }]
```

## Features

- **Flood Predictions**: Get AI-powered flood forecasts for any location
- **River Gauges**: Access real-time and forecasted water levels
- **Alerts**: Subscribe to flood alerts via SMS, email, or WebSocket
- **Satellite Imagery**: Download Sentinel-1/2 flood extent maps
- **Weather Data**: Retrieve precipitation forecasts
- **Type Safety**: Full TypeScript support with detailed type definitions
- **Validation**: Built-in data validation and error handling
- **Utilities**: Helper functions for distance calculation, unit conversion, etc.

## API Reference

### FloodPredictionClient

Main client for interacting with the WIA-FLOOD_PREDICTION API.

#### Constructor

```typescript
new FloodPredictionClient(config: ClientConfig)
```

**ClientConfig**:
- `apiKey` (string, required): Your API key (get one at https://wia-flood.org/signup)
- `baseURL` (string, optional): API base URL (default: `https://api.wia-flood.org/v1`)
- `timeout` (number, optional): Request timeout in milliseconds (default: 30000)

#### Methods

##### `getPredictions(params: GetPredictionsParams): Promise<FloodPrediction[]>`

Get flood predictions for a location.

**Parameters**:
```typescript
interface GetPredictionsParams {
  lat: number;              // Latitude (-90 to 90)
  lng: number;              // Longitude (-180 to 180)
  radius_km?: number;       // Search radius (default: 10, max: 100)
  start_date?: string;      // Filter by forecast date (ISO 8601)
  end_date?: string;        // Filter end (ISO 8601)
  risk_level?: RiskLevel;   // 'low' | 'medium' | 'high' | 'extreme'
  min_probability?: number; // Minimum flood probability (0.0-1.0)
}
```

**Example**:
```typescript
const predictions = await client.getPredictions({
  lat: 38.9072,
  lng: -77.0369,
  radius_km: 20,
  min_probability: 0.5,
  risk_level: 'high'
});
```

##### `getPrediction(id: string): Promise<FloodPrediction>`

Get a specific prediction by ID.

```typescript
const prediction = await client.getPrediction('pred_2026011114_38.9072_-77.0369');
```

##### `getBatchPredictions(locations: GeoPoint[], forecast_date?: string): Promise<FloodPrediction[]>`

Get predictions for multiple locations at once.

```typescript
const predictions = await client.getBatchPredictions([
  { lat: 38.9072, lng: -77.0369 },
  { lat: 40.7128, lng: -74.0060 },
  { lat: 29.7604, lng: -95.3698 }
]);
```

##### `getGauges(params: GetGaugesParams): Promise<RiverGauge[]>`

Get river gauges within an area.

```typescript
const gauges = await client.getGauges({
  lat: 38.9072,
  lng: -77.0369,
  radius_km: 50,
  status: 'flood'
});
```

##### `getGauge(id: string): Promise<RiverGauge>`

Get detailed gauge data with forecast.

```typescript
const gauge = await client.getGauge('01646500'); // USGS site code
```

##### `getGaugeHistory(id: string, start_date: string, end_date: string): Promise<GaugeTimeSeries>`

Get historical water level data.

```typescript
const history = await client.getGaugeHistory(
  '01646500',
  '2026-01-01T00:00:00Z',
  '2026-01-11T00:00:00Z'
);
```

##### `getAlerts(params: GetAlertsParams): Promise<FloodAlert[]>`

Get active flood alerts.

```typescript
const alerts = await client.getAlerts({
  lat: 38.9072,
  lng: -77.0369,
  radius_km: 50,
  severity: 'warning'
});
```

##### `subscribeToAlerts(params: AlertSubscription): Promise<Subscription>`

Subscribe to flood alerts.

```typescript
const subscription = await client.subscribeToAlerts({
  location: { lat: 38.9072, lng: -77.0369 },
  radius_km: 10,
  channels: ['email', 'sms'],
  contact: {
    email: 'user@example.com',
    phone: '+12025551234'
  },
  filters: {
    min_severity: 'watch',
    alert_types: ['flash_flood', 'river_flood']
  }
});
```

##### `connectWebSocket(callbacks: WebSocketCallbacks): WebSocketConnection`

Connect to real-time WebSocket feed.

```typescript
const ws = client.connectWebSocket({
  onPrediction: (prediction) => {
    console.log('New prediction:', prediction);
  },
  onAlert: (alert) => {
    console.log('ALERT:', alert);
  },
  onError: (error) => {
    console.error('WebSocket error:', error);
  }
});

// Subscribe to location
ws.subscribeToPredictions({ lat: 38.9072, lng: -77.0369, radius_km: 10 });
ws.subscribeToAlerts({ lat: 38.9072, lng: -77.0369, radius_km: 50 });

// Close connection
ws.close();
```

## Data Types

### FloodPrediction

```typescript
interface FloodPrediction {
  id: string;
  location: GeoPoint;
  forecast_date: string;
  issued_at: string;
  risk_level: 'low' | 'medium' | 'high' | 'extreme';
  probability: number;              // 0.0 - 1.0
  confidence: number;               // 0.0 - 1.0
  predicted_depth_meters: number;
  predicted_velocity_ms: number;
  affected_area_km2: number;
  peak_time: string;
  model_version: string;
  data_sources: string[];
  uncertainty_range: {
    depth_min: number;
    depth_max: number;
  };
}
```

### RiverGauge

```typescript
interface RiverGauge {
  id: string;
  name: string;
  location: GeoPoint;
  river_name: string;
  current_level_m: number;
  current_discharge_m3s: number;
  measured_at: string;
  flood_stage_m: number;
  moderate_flood_stage_m: number;
  major_flood_stage_m: number;
  forecast: GaugeForecast[];
  status: 'normal' | 'action' | 'minor_flood' | 'moderate_flood' | 'major_flood';
}
```

### FloodAlert

```typescript
interface FloodAlert {
  id: string;
  alert_type: 'flash_flood' | 'river_flood' | 'coastal_flood' | 'dam_failure';
  severity: 'advisory' | 'watch' | 'warning' | 'emergency';
  affected_area: GeoJSON.Polygon;
  headline: string;
  description: string;
  instructions: string;
  issued_at: string;
  effective_at: string;
  expires_at: string;
}
```

## Utilities

### Distance Calculation

```typescript
import { calculateDistance } from '@wia/flood-prediction/utils';

const distance = calculateDistance(
  { lat: 38.9072, lng: -77.0369 },
  { lat: 40.7128, lng: -74.0060 }
);
console.log(`${distance.toFixed(0)} km`); // 328 km
```

### Unit Conversion

```typescript
import { metersToFeet, m3sToFt3s, formatDepth } from '@wia/flood-prediction/utils';

const depthFeet = metersToFeet(2.4);        // 7.87 feet
const dischargeFt3s = m3sToFt3s(142.5);     // 5,032 ft³/s

const formattedDepth = formatDepth(2.4, 'imperial'); // "7.9 ft"
```

### Filtering & Sorting

```typescript
import {
  sortPredictionsByRisk,
  filterByProbability,
  filterByDistance
} from '@wia/flood-prediction/utils';

// Sort by risk level
const sorted = sortPredictionsByRisk(predictions);

// Filter by probability
const highProb = filterByProbability(predictions, 0.7);

// Filter by distance
const nearby = filterByDistance(predictions, { lat: 38.9, lng: -77.0 }, 10);
```

## Validation

The SDK includes comprehensive validation for all data types:

```typescript
import { validateGeoPoint, validateFloodPrediction } from '@wia/flood-prediction/validators';

try {
  validateGeoPoint({ lat: 95, lng: 0 }); // Throws ValidationError
} catch (error) {
  console.error(error.message); // "location.lat must be between -90 and 90"
}
```

## Error Handling

```typescript
try {
  const predictions = await client.getPredictions({
    lat: 38.9072,
    lng: -77.0369
  });
} catch (error) {
  if (error.code === 'RATE_LIMIT_EXCEEDED') {
    console.error('Rate limit exceeded. Retry after:', error.retryAfter);
  } else if (error.code === 'INVALID_API_KEY') {
    console.error('Invalid API key');
  } else {
    console.error('Unexpected error:', error);
  }
}
```

## Rate Limiting

Free tier: 100 requests/hour

To check your rate limit status:

```typescript
const response = await client.getPredictions({ lat: 38.9, lng: -77.0 });
console.log('Rate limit remaining:', client.rateLimitRemaining);
console.log('Rate limit resets at:', new Date(client.rateLimitReset * 1000));
```

## Examples

### Emergency Manager Dashboard

```typescript
import { FloodPredictionClient, sortPredictionsByRisk } from '@wia/flood-prediction';

const client = new FloodPredictionClient({ apiKey: process.env.WIA_API_KEY });

// Get high-risk predictions for Washington DC metro area
const predictions = await client.getPredictions({
  lat: 38.9072,
  lng: -77.0369,
  radius_km: 50,
  min_probability: 0.5
});

// Sort by risk
const sorted = sortPredictionsByRisk(predictions);

// Display top 5 highest risk
sorted.slice(0, 5).forEach((p) => {
  console.log(`[${p.risk_level.toUpperCase()}] ${p.probability * 100}% probability`);
  console.log(`  Location: ${p.location.lat}, ${p.location.lng}`);
  console.log(`  Predicted depth: ${p.predicted_depth_meters.toFixed(1)}m`);
  console.log(`  Peak time: ${new Date(p.peak_time).toLocaleString()}`);
  console.log('');
});
```

### Real-Time Monitoring

```typescript
const ws = client.connectWebSocket({
  onPrediction: (prediction) => {
    if (prediction.risk_level === 'extreme') {
      // Send alert to emergency team
      sendEmergencyAlert(prediction);
    }
  },
  onAlert: (alert) => {
    // Display alert on dashboard
    displayAlert(alert);
  }
});

ws.subscribeToPredictions({ lat: 38.9072, lng: -77.0369, radius_km: 100 });
ws.subscribeToAlerts({ lat: 38.9072, lng: -77.0369, radius_km: 100 });
```

## Support

- Documentation: https://docs.wia-flood.org
- API Status: https://status.wia-flood.org
- Email: support@wia-flood.org
- GitHub Issues: https://github.com/WIA-Official/flood-prediction-sdk-ts

## License

MIT License - see [LICENSE](./LICENSE) file

## About WIA

WIA (World Certification Industry Association) develops open standards for global challenges.

**Philosophy**: 弘益人間 (홍익인간) - Benefit All Humanity

---

© 2026 WIA | https://wia-flood.org
