# WIA Water Resource Management - TypeScript SDK

> 弘益人間 (Benefit All Humanity)

TypeScript SDK for the WIA Water Resource Management Standard - Comprehensive tools for sustainable water resource conservation and management.

## Installation

```bash
npm install @wia/water-resource-management
```

## Quick Start

```typescript
import { createWaterResourceManagement, WatershedData } from '@wia/water-resource-management';

// Create instance
const waterMgmt = createWaterResourceManagement({
  enableAutoMonitoring: true,
  monitoringInterval: 3600000, // 1 hour
});

// Register a watershed
const watershed: WatershedData = {
  id: 'ws-001',
  name: 'Colorado River Basin',
  area: 637000, // km²
  location: {
    latitude: 37.0,
    longitude: -109.0,
  },
  tributaries: ['Green River', 'Gunnison River'],
  // ... other properties
};

waterMgmt.registerWatershed(watershed);

// Monitor watershed health
const health = waterMgmt.monitorWatershedHealth('ws-001');
console.log(`Watershed health: ${health.overallHealth}%`);
```

## Core Features

### 1. Watershed Monitoring

Track and analyze watershed health, water quality, and ecosystem metrics.

```typescript
// Register watershed
waterMgmt.registerWatershed(watershedData);

// Update watershed data
waterMgmt.updateWatershed('ws-001', {
  waterQuality: {
    pH: 7.2,
    dissolvedOxygen: 8.5,
    turbidity: 2.3,
    temperature: 18.5,
    conductivity: 450,
    nutrients: {
      nitrogen: 1.2,
      phosphorus: 0.08,
      potassium: 3.5,
    },
    contaminants: {
      heavyMetals: {},
      pesticides: {},
      bacteria: 5,
    },
    overallQuality: 'good',
  },
});

// Get health report
const healthReport = waterMgmt.monitorWatershedHealth('ws-001');
```

### 2. Water Allocation Management

Manage water distribution across different sectors with priority-based allocation.

```typescript
import { WaterAllocation, AllocationPriority } from '@wia/water-resource-management';

const allocation: WaterAllocation = {
  id: 'alloc-001',
  source: 'surface',
  totalAvailable: 1000000000, // m³
  allocated: {
    domestic: 300000000,
    agricultural: 500000000,
    industrial: 150000000,
    environmental: 40000000,
    recreational: 10000000,
  },
  reserved: 50000000, // Environmental flow
  restrictions: [],
  timestamp: new Date(),
};

waterMgmt.createAllocation(allocation);

// Check sustainability
const sustainability = waterMgmt.checkAllocationSustainability('alloc-001');
if (!sustainability.isSustainable) {
  console.log('Recommendations:', sustainability.recommendations);
}
```

### 3. Reservoir Operations

Monitor and manage reservoir levels, inflows, outflows, and operations.

```typescript
import { ReservoirData } from '@wia/water-resource-management';

const reservoir: ReservoirData = {
  id: 'res-001',
  name: 'Hoover Dam',
  location: {
    latitude: 36.015,
    longitude: -114.738,
  },
  capacity: {
    total: 35200000000, // m³
    active: 28500000000,
    dead: 6700000000,
    flood: 10000000000,
    conservation: 25200000000,
  },
  currentLevel: {
    elevation: 350,
    volume: 25000000000,
    percentage: 71,
    status: 'normal',
  },
  inflow: 850, // m³/s
  outflow: 750,
  storage: 25000000000,
  waterQuality: waterQualityData,
  operations: operationsData,
  purpose: ['water-supply', 'flood-control', 'hydropower'],
};

waterMgmt.registerReservoir(reservoir);

// Update reservoir level
waterMgmt.updateReservoirLevel('res-001', 348, 24500000000);

// Calculate optimal release
const optimalRelease = waterMgmt.calculateOptimalRelease('res-001', 800);
```

### 4. Groundwater Tracking

Monitor aquifer levels, recharge rates, and extraction sustainability.

```typescript
import { GroundwaterData } from '@wia/water-resource-management';

const aquifer: GroundwaterData = {
  id: 'aq-001',
  name: 'Ogallala Aquifer',
  type: 'unconfined',
  location: {
    latitude: 39.5,
    longitude: -101.0,
  },
  depth: {
    top: 10,
    bottom: 300,
    thickness: 290,
  },
  waterTable: {
    currentDepth: 45,
    historicalDepth: historicalData,
    seasonalVariation: 3,
    trend: 'declining',
  },
  recharge: {
    natural: 15000000,
    artificial: 5000000,
    total: 20000000,
    sources: ['precipitation', 'surface water infiltration'],
  },
  extraction: {
    permitted: 30000000,
    actual: 28000000,
    wells: wellsData,
    sustainability: 71,
  },
  quality: waterQualityData,
  sustainabilityIndex: 71,
};

waterMgmt.registerAquifer(aquifer);

// Update groundwater level
waterMgmt.updateGroundwaterLevel('aq-001', 46.5);

// Check sustainability
const gwSustainability = waterMgmt.checkGroundwaterSustainability('aq-001');
```

### 5. Demand Forecasting

Project future water demand based on historical data and growth rates.

```typescript
import { DemandForecast } from '@wia/water-resource-management';

// Create forecast
const forecast: DemandForecast = {
  id: 'fc-001',
  region: 'Southwest Region',
  timeframe: {
    startDate: new Date('2025-01-01'),
    endDate: new Date('2030-12-31'),
    granularity: 'yearly',
  },
  sectors: sectorDemands,
  totalDemand: 500000000,
  confidence: 85,
  scenarios: scenarios,
};

waterMgmt.createForecast(forecast);

// Calculate future demand
const projection = waterMgmt.calculateFutureDemand('Southwest Region', 60, 0.025);
console.log(`Projected demand in 5 years: ${projection.projectedDemand} m³`);
```

### 6. Conservation Planning

Develop and track water conservation measures and targets.

```typescript
import { ConservationPlan, ConservationMeasure } from '@wia/water-resource-management';

const conservationPlan: ConservationPlan = {
  id: 'cp-001',
  name: 'Regional Water Conservation Initiative',
  status: 'normal',
  measures: [
    {
      id: 'cm-001',
      type: 'efficiency',
      description: 'Install smart irrigation systems',
      expectedSavings: 50000000, // m³/year
      cost: 5000000,
      priority: 'high',
      status: 'active',
    },
    {
      id: 'cm-002',
      type: 'education',
      description: 'Public awareness campaign',
      expectedSavings: 20000000,
      cost: 500000,
      priority: 'medium',
      status: 'active',
    },
  ],
  targets: [
    {
      metric: 'Total water savings',
      current: 30000000,
      target: 70000000,
      deadline: new Date('2026-12-31'),
      progress: 43,
    },
  ],
  effectiveness: 75,
  implementation: implementationPlan,
};

waterMgmt.createConservationPlan(conservationPlan);

// Update conservation status
waterMgmt.updateConservationStatus('cp-001', 'advisory');

// Calculate effectiveness
const effectiveness = waterMgmt.calculateConservationEffectiveness('cp-001');
```

### 7. Event Handling

Subscribe to water resource events and alerts.

```typescript
import { WaterResourceEvent } from '@wia/water-resource-management';

// Register event handlers
waterMgmt.on('reservoir-level-change', (event: WaterResourceEvent) => {
  console.log('Reservoir level changed:', event.data);
  if (event.severity === 'critical') {
    // Trigger emergency protocols
  }
});

waterMgmt.on('water-quality-alert', (event: WaterResourceEvent) => {
  console.log('Water quality alert:', event.data);
  // Send notifications
});

waterMgmt.on('groundwater-alert', (event: WaterResourceEvent) => {
  console.log('Groundwater alert:', event.data);
  // Adjust extraction rates
});

waterMgmt.on('conservation-status-change', (event: WaterResourceEvent) => {
  console.log('Conservation status changed:', event.data);
  // Update public advisories
});
```

## Types

The SDK provides comprehensive TypeScript types for all water resource management entities:

- **Watershed Types**: `WatershedData`, `WaterQualityMetrics`, `EcosystemHealth`
- **Allocation Types**: `WaterAllocation`, `AllocationBreakdown`, `AllocationRestriction`
- **Reservoir Types**: `ReservoirData`, `ReservoirCapacity`, `WaterLevel`
- **Groundwater Types**: `GroundwaterData`, `AquiferType`, `WaterTableData`
- **Irrigation Types**: `IrrigationSystem`, `IrrigationType`, `IrrigationSchedule`
- **Water Rights Types**: `WaterRight`, `WaterRightType`, `RightCondition`
- **Forecasting Types**: `DemandForecast`, `SectorDemand`, `ForecastScenario`
- **Conservation Types**: `ConservationPlan`, `ConservationMeasure`, `ConservationTarget`

## Configuration

```typescript
import { WaterResourceConfig } from '@wia/water-resource-management';

const config: WaterResourceConfig = {
  enableAutoMonitoring: true,
  monitoringInterval: 3600000, // 1 hour in milliseconds
  alertThresholds: {
    reservoirLowLevel: 30, // percentage
    groundwaterDepthCritical: 100, // meters
    waterQualityMinimum: 'fair',
  },
};

const waterMgmt = createWaterResourceManagement(config);
```

## Best Practices

1. **Regular Monitoring**: Update watershed, reservoir, and groundwater data regularly
2. **Event Handling**: Subscribe to critical events for timely response
3. **Sustainability Checks**: Regularly assess allocation and extraction sustainability
4. **Conservation Planning**: Implement and track conservation measures
5. **Demand Forecasting**: Plan ahead with accurate demand projections
6. **Water Quality**: Monitor and maintain water quality standards
7. **Environmental Flow**: Always reserve water for environmental needs

## Example: Complete Water Management System

```typescript
import { createWaterResourceManagement } from '@wia/water-resource-management';

// Initialize
const waterMgmt = createWaterResourceManagement({
  enableAutoMonitoring: true,
});

// Register critical event handlers
waterMgmt.on('reservoir-level-change', (event) => {
  if (event.severity === 'critical') {
    console.log('ALERT: Critical reservoir level!');
    // Implement emergency measures
  }
});

// Register resources
waterMgmt.registerWatershed(watershed);
waterMgmt.registerReservoir(reservoir);
waterMgmt.registerAquifer(aquifer);

// Create management plans
waterMgmt.createAllocation(allocation);
waterMgmt.createConservationPlan(conservationPlan);
waterMgmt.createForecast(forecast);

// Monitor and respond
setInterval(() => {
  const watershedHealth = waterMgmt.monitorWatershedHealth('ws-001');
  const allocationSustainability = waterMgmt.checkAllocationSustainability('alloc-001');
  const gwSustainability = waterMgmt.checkGroundwaterSustainability('aq-001');

  console.log('System Status:', {
    watershedHealth: watershedHealth.overallHealth,
    allocationSustainable: allocationSustainability.isSustainable,
    groundwaterSustainable: gwSustainability.isSustainable,
  });
}, 3600000); // Every hour
```

## Contributing

Contributions are welcome! Please see the [main repository](https://github.com/WIA-Official/wia-standards) for contribution guidelines.

## License

MIT

## Philosophy

**弘益人間 (Benefit All Humanity)** - This standard is developed with the goal of benefiting all humanity through sustainable water resource management.

---

© 2025 WIA (World Certification Industry Association)
