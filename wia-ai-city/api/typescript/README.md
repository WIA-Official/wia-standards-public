# WIA-AI-CITY TypeScript SDK

AI-powered Urban Management and Smart City Intelligence

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity

## Installation

```bash
npm install @wia/wia-ai-city
```

## Quick Start

```typescript
import { createAICity, AICityConfig, AIModel } from '@wia/wia-ai-city';

// Configure your AI city
const config: AICityConfig = {
  cityId: 'seoul-001',
  cityName: 'Seoul',
  population: 9776000,
  area: 605.2,
  timezone: 'Asia/Seoul',
  models: [
    {
      id: 'traffic-pred-001',
      name: 'Traffic Prediction Model',
      type: 'traffic_prediction',
      version: '1.0.0',
      accuracy: 0.92,
      trainedAt: new Date(),
      lastUpdated: new Date(),
      parameters: {},
      status: 'active',
    },
  ],
  sensors: 5000,
  updateInterval: 60000, // 1 minute
  dataRetention: 90, // days
};

// Create AI city instance with event handlers
const aiCity = createAICity(config, {
  onPrediction: (prediction) => {
    console.log('New prediction:', prediction);
  },
  onAnomaly: (anomaly) => {
    console.log('Anomaly detected:', anomaly);
  },
  onAlert: (alert) => {
    console.log('Alert:', alert);
  },
});

// Start monitoring
await aiCity.start();
```

## Features

### 1. Traffic Management

```typescript
// Predict traffic conditions
const trafficPrediction = await aiCity.predictTraffic('Gangnam District');
console.log('Congestion level:', trafficPrediction.congestionLevel);
console.log('Average speed:', trafficPrediction.averageSpeed);

// Optimize traffic flow
const optimization = await aiCity.optimizeTraffic('intersection-001');
console.log('Expected improvement:', optimization.expectedImprovement);
```

### 2. Energy Management

```typescript
// Predict energy demand
const energyPrediction = await aiCity.predictEnergyDemand('Zone A');
console.log('Predicted demand:', energyPrediction.predictedDemand);
console.log('Renewable ratio:', energyPrediction.renewableRatio);

// Optimize energy distribution
const energyOpt = await aiCity.optimizeEnergy('zone-001');
console.log('Savings potential:', energyOpt.savingsPotential);
```

### 3. Safety & Security

```typescript
// Analyze safety conditions
const safetyAnalysis = await aiCity.analyzeSafety('downtown');
console.log('Risk level:', safetyAnalysis.riskLevel);
console.log('Emergency readiness:', safetyAnalysis.emergencyReadiness);

// Detect anomalies
const anomalies = await aiCity.detectAnomalies('sensor-001');
anomalies.forEach(anomaly => {
  console.log('Anomaly type:', anomaly.anomalyType);
  console.log('Severity:', anomaly.severity);
});
```

### 4. Resource Allocation

```typescript
// Optimize resource allocation
const allocation = await aiCity.optimizeResources('vehicle');
console.log('Efficiency:', allocation.efficiency);
console.log('Cost savings:', allocation.costSavings);
```

### 5. Decision Support

```typescript
// Generate AI recommendations
const recommendations = await aiCity.generateRecommendations('traffic');
recommendations.forEach(rec => {
  console.log('Priority:', rec.priority);
  console.log('Title:', rec.title);
  console.log('AI Confidence:', rec.aiConfidence);
  console.log('Alternatives:', rec.alternatives);
});

// Make autonomous decision
const decision = await aiCity.makeDecision('emergency_response', {
  severity: 'high',
  location: 'downtown',
});
console.log('Decision:', decision);
```

### 6. City Metrics & Analytics

```typescript
// Get overall city metrics
const metrics = await aiCity.getCityMetrics();
console.log('Traffic efficiency:', metrics.trafficEfficiency);
console.log('Energy efficiency:', metrics.energyEfficiency);
console.log('Safety score:', metrics.safetyScore);
console.log('Citizen satisfaction:', metrics.citizenSatisfaction);
```

### 7. Simulation & Forecasting

```typescript
// Run simulation scenario
const scenario = {
  id: 'scenario-001',
  name: 'Peak Hour Traffic Management',
  description: 'Simulate rush hour with new signal timing',
  parameters: {
    signalTiming: 'adaptive',
    trafficVolume: 'high',
  },
  predictions: [
    {
      metric: 'average_speed',
      currentValue: 30,
      predictedValue: 0,
      confidence: 0,
    },
  ],
  risks: ['Increased congestion'],
  opportunities: ['Improved flow'],
};

const result = await aiCity.runSimulation(scenario);
result.predictions.forEach(pred => {
  console.log(`${pred.metric}: ${pred.currentValue} → ${pred.predictedValue}`);
});
```

### 8. AI Model Management

```typescript
// Register new AI model
await aiCity.registerModel({
  id: 'energy-opt-001',
  name: 'Energy Optimization Model',
  type: 'energy_optimization',
  version: '2.0.0',
  accuracy: 0.95,
  trainedAt: new Date(),
  lastUpdated: new Date(),
  parameters: {
    algorithm: 'deep_learning',
    layers: 5,
  },
  status: 'active',
});

// Train model with new data
await aiCity.trainModel('energy-opt-001', trainingData);

// Get model by type
const trafficModels = aiCity.getModelsByType('traffic_prediction');
console.log('Traffic models:', trafficModels.length);
```

## Event Handlers

```typescript
const aiCity = createAICity(config, {
  onPrediction: (prediction) => {
    // Handle new predictions
    console.log('Model:', prediction.modelId);
    console.log('Confidence:', prediction.confidence);
  },

  onAnomaly: (anomaly) => {
    // Handle detected anomalies
    console.log('Anomaly detected:', anomaly.anomalyType);
    console.log('Suggested actions:', anomaly.suggestedActions);
  },

  onIncident: (incident) => {
    // Handle safety incidents
    console.log('Incident type:', incident.type);
    console.log('Severity:', incident.severity);
  },

  onOptimization: (optimization) => {
    // Handle optimization results
    console.log('Optimization completed');
  },

  onAlert: (alert) => {
    // Handle system alerts
    console.log(`[${alert.level}] ${alert.message}`);
  },
});
```

## API Reference

### WIAAICity Class

#### AI Model Management
- `registerModel(model: AIModel): Promise<void>`
- `getModel(modelId: string): AIModel | undefined`
- `getModelsByType(type: AIModelType): AIModel[]`
- `updateModelStatus(modelId: string, status: string): Promise<void>`
- `trainModel(modelId: string, trainingData: any): Promise<void>`

#### Prediction Services
- `predictTraffic(location: string): Promise<TrafficPrediction>`
- `predictEnergyDemand(zone: string): Promise<EnergyDemandPrediction>`
- `predictEvents(): Promise<EventPrediction[]>`

#### Optimization
- `optimizeTraffic(intersectionId: string): Promise<TrafficOptimization>`
- `optimizeEnergy(zoneId: string): Promise<EnergyOptimization>`
- `optimizeResources(resourceType: string): Promise<ResourceAllocation>`

#### Decision Support
- `generateRecommendations(category: string): Promise<DecisionRecommendation[]>`
- `makeDecision(scenario: string, parameters: any): Promise<string>`

#### Safety & Security
- `analyzeSafety(zoneId: string): Promise<SafetyAnalysis>`
- `detectAnomalies(sensorId: string): Promise<AnomalyDetection[]>`

#### Analytics
- `getCityMetrics(): Promise<CityMetrics>`
- `runSimulation(scenario: SimulationScenario): Promise<SimulationScenario>`

#### System Control
- `start(): Promise<void>`
- `stop(): Promise<void>`
- `getStatus(): object`

## Types

All TypeScript types are exported from the main module:

```typescript
import {
  AIModel,
  AIModelType,
  TrafficPrediction,
  EnergyDemandPrediction,
  SafetyAnalysis,
  AnomalyDetection,
  ResourceAllocation,
  DecisionRecommendation,
  CityMetrics,
  // ... and more
} from '@wia/wia-ai-city';
```

## Use Cases

### Smart Traffic Management
- Real-time traffic prediction
- Adaptive signal control
- Incident detection and response
- Route optimization

### Energy Efficiency
- Demand forecasting
- Grid balancing
- Renewable integration
- Peak load management

### Public Safety
- Anomaly detection
- Emergency response optimization
- Risk assessment
- Surveillance analytics

### Resource Optimization
- Fleet management
- Waste collection routing
- Maintenance scheduling
- Service allocation

### Urban Planning
- Impact simulation
- Growth forecasting
- Infrastructure optimization
- Policy analysis

## Development

```bash
# Install dependencies
npm install

# Build
npm run build

# Development mode
npm run dev

# Run tests
npm test

# Lint
npm run lint

# Format
npm run format
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

MIT License - see LICENSE file for details

## Support

- GitHub Issues: https://github.com/WIA-Official/wia-standards/issues
- Documentation: https://github.com/WIA-Official/wia-standards/tree/main/wia-ai-city

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
