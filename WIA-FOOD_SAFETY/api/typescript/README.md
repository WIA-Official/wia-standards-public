# WIA-FOOD_SAFETY TypeScript SDK

Official TypeScript/JavaScript SDK for WIA-FOOD_SAFETY blockchain-based food traceability and safety management system.

## Features

- Food product registration and traceability
- HACCP compliance monitoring
- Critical Control Point (CCP) recording
- Temperature monitoring and alerts
- Recall management
- Laboratory testing integration
- Blockchain verification
- Type-safe API with full TypeScript support

## Installation

```bash
npm install @wia/food-safety
# or
yarn add @wia/food-safety
```

## Quick Start

```typescript
import { FoodSafetySDK } from '@wia/food-safety';

// Initialize SDK
const sdk = new FoodSafetySDK({
  apiKey: 'your-api-key',
  baseUrl: 'https://api.wia-food-safety.org/v1'
});

// Register a food product
const product = await sdk.products.register({
  productName: 'Organic Romaine Lettuce',
  category: 'FRESH_PRODUCE',
  batchId: 'FARM-2026-001234',
  origin: {
    farmName: 'Green Valley Farms',
    location: {
      latitude: 36.1699,
      longitude: -115.1398,
      address: '123 Farm Road, Salinas, CA 93901'
    },
    certifications: ['USDA_ORGANIC', 'GAP_CERTIFIED']
  },
  harvestDate: '2026-01-10T08:30:00Z',
  quantity: { value: 500, unit: 'kg' },
  expirationDate: '2026-01-17T23:59:59Z'
});

console.log('Product registered:', product.blockchainTxHash);
console.log('QR Code:', product.qrCode);

// Trace supply chain (< 2.2 seconds)
const trace = await sdk.products.trace('FARM-2026-001234');
console.log('Traceability time:', trace.traceabilityTime);
console.log('Supply chain:', trace.data.supplyChain);

// Record Critical Control Point (CCP)
const ccpRecord = await sdk.haccp.recordCCP({
  batchId: 'FARM-2026-001234',
  ccpType: 'COLD_STORAGE',
  measurement: {
    temperature: 3.5,
    unit: 'CELSIUS',
    humidity: 85,
    timestamp: new Date().toISOString()
  },
  criticalLimit: { min: 0, max: 4, unit: 'CELSIUS' },
  sensorId: 'TEMP-SENSOR-001',
  location: 'Warehouse A, Zone 3'
});

console.log('CCP recorded:', ccpRecord.blockchainTxHash);
```

## API Reference

### Products

#### `sdk.products.register(data: ProductRegistrationData)`

Register a new food product on the blockchain.

```typescript
interface ProductRegistrationData {
  productName: string;
  category: ProductCategory;
  batchId: string;
  origin: Origin;
  harvestDate: string;
  quantity: Quantity;
  expirationDate: string;
  haccp?: HACCPPlan;
}
```

**Returns:** `Promise<ProductRegistrationResponse>`

#### `sdk.products.trace(batchId: string)`

Trace complete supply chain history (< 2.2 seconds).

**Returns:** `Promise<TraceabilityResponse>`

#### `sdk.products.scanQR(qrCode: string)`

Scan QR code and retrieve product information.

**Returns:** `Promise<ProductInfo>`

### HACCP

#### `sdk.haccp.recordCCP(data: CCPRecordData)`

Log Critical Control Point measurement.

```typescript
interface CCPRecordData {
  batchId: string;
  ccpType: CCPType;
  measurement: {
    temperature?: number;
    unit: string;
    humidity?: number;
    timestamp: string;
  };
  criticalLimit: {
    min?: number;
    max?: number;
    unit: string;
  };
  sensorId: string;
  location: string;
}
```

**Returns:** `Promise<CCPRecordResponse>`

#### `sdk.haccp.getViolations(params: ViolationParams)`

Retrieve CCP violations requiring corrective action.

**Returns:** `Promise<ViolationResponse>`

### IoT Sensors

#### `sdk.iot.logTemperature(data: TemperatureLogData)`

Bulk upload temperature sensor data.

**Returns:** `Promise<TemperatureLogResponse>`

#### `sdk.iot.getTemperatureHistory(batchId: string)`

Retrieve complete temperature history for a batch.

**Returns:** `Promise<TemperatureHistoryResponse>`

### Recalls

#### `sdk.recalls.initiate(data: RecallInitiationData)`

Initiate product recall.

```typescript
interface RecallInitiationData {
  batchIds: string[];
  reason: string;
  severity: RecallClass;
  recallScope: RecallScope;
  contactInfo: ContactInfo;
  labReport?: LabReport;
}
```

**Returns:** `Promise<RecallResponse>`

#### `sdk.recalls.getStatus(recallId: string)`

Track recall progress and effectiveness.

**Returns:** `Promise<RecallStatusResponse>`

### Laboratory Testing

#### `sdk.lab.submitResults(data: LabTestData)`

Submit laboratory test results.

**Returns:** `Promise<LabTestResponse>`

## Types

All TypeScript types are exported from the main package:

```typescript
import {
  FoodProduct,
  BatchRecord,
  TemperatureLog,
  HACCPPlan,
  RecallNotice,
  LabTestResult,
  ProductCategory,
  CCPType,
  RecallClass
} from '@wia/food-safety';
```

## Validation

The SDK includes built-in validation functions:

```typescript
import { validators } from '@wia/food-safety';

// Validate batch ID format
if (!validators.validateBatchId('FARM-2026-001234')) {
  throw new Error('Invalid batch ID format');
}

// Validate temperature
if (!validators.validateTemperature(3.5)) {
  throw new Error('Temperature out of range');
}

// Validate coordinates
if (!validators.validateCoordinates(36.1699, -115.1398)) {
  throw new Error('Invalid GPS coordinates');
}

// Comprehensive product validation
const validation = validators.validateProductRegistration(productData);
if (!validation.valid) {
  console.error('Validation errors:', validation.errors);
}
```

## Utilities

Helper functions for common operations:

```typescript
import { utils } from '@wia/food-safety';

// Generate batch ID
const batchId = utils.generateBatchId('FARM', new Date(), 1234);
// => "FARM-2026-001234"

// Calculate hash
const hash = utils.calculateHash(data);

// Calculate Merkle root for batch temperature logs
const merkleRoot = utils.calculateMerkleRoot(temperatureReadings);

// Format temperature
const formatted = utils.formatTemperature(3.5, 'F');
// => "38.3°F"

// Calculate distance between two locations
const distance = utils.calculateDistance(36.1699, -115.1398, 34.0522, -118.2437);
// => 346.7 km

// Check expiration
const daysUntil = utils.calculateDaysUntilExpiration(new Date('2026-01-17'));
// => 6
```

## Error Handling

The SDK throws typed errors for common failure scenarios:

```typescript
import { FoodSafetyError, ErrorCode } from '@wia/food-safety';

try {
  await sdk.products.trace('INVALID-BATCH-ID');
} catch (error) {
  if (error instanceof FoodSafetyError) {
    switch (error.code) {
      case ErrorCode.PRODUCT_NOT_FOUND:
        console.error('Product not found:', error.message);
        break;
      case ErrorCode.BLOCKCHAIN_ERROR:
        console.error('Blockchain transaction failed:', error.message);
        break;
      case ErrorCode.UNAUTHORIZED:
        console.error('Invalid API key:', error.message);
        break;
      default:
        console.error('Error:', error.message);
    }
  }
}
```

## Configuration

### Environment Variables

```bash
# API Configuration
WIA_FOOD_SAFETY_API_KEY=your-api-key
WIA_FOOD_SAFETY_BASE_URL=https://api.wia-food-safety.org/v1

# Blockchain Configuration
ETHEREUM_RPC_URL=https://polygon-rpc.com
ETHEREUM_PRIVATE_KEY=your-private-key

# Optional
WIA_FOOD_SAFETY_TIMEOUT=30000
WIA_FOOD_SAFETY_RETRY_ATTEMPTS=3
```

### Advanced Configuration

```typescript
const sdk = new FoodSafetySDK({
  apiKey: process.env.WIA_FOOD_SAFETY_API_KEY,
  baseUrl: process.env.WIA_FOOD_SAFETY_BASE_URL,
  timeout: 30000, // 30 seconds
  retryAttempts: 3,
  blockchain: {
    rpcUrl: process.env.ETHEREUM_RPC_URL,
    privateKey: process.env.ETHEREUM_PRIVATE_KEY,
    network: 'polygon' // 'ethereum', 'polygon', 'fabric'
  }
});
```

## Examples

See the [examples](./examples) directory for complete examples:

- [Basic Product Registration](./examples/01-register-product.ts)
- [Supply Chain Tracing](./examples/02-trace-supply-chain.ts)
- [HACCP Compliance](./examples/03-haccp-compliance.ts)
- [Temperature Monitoring](./examples/04-temperature-monitoring.ts)
- [Recall Management](./examples/05-recall-management.ts)
- [Lab Testing Integration](./examples/06-lab-testing.ts)

## Development

```bash
# Install dependencies
npm install

# Build
npm run build

# Run tests
npm test

# Lint
npm run lint

# Type check
npm run type-check
```

## Contributing

Contributions are welcome! Please read our [Contributing Guide](../../CONTRIBUTING.md) for details.

## License

MIT License - see [LICENSE](../../LICENSE) file for details.

## Support

- Documentation: https://docs.wia-food-safety.org
- API Reference: https://api.wia-food-safety.org/docs
- GitHub Issues: https://github.com/WIA-Official/wia-standards/issues
- Email: support@wia-food-safety.org

---

**© 2026 WIA | 弘益人間 (Benefit All Humanity)**
