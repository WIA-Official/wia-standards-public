# WIA-SEMI-004 TypeScript SDK

TypeScript SDK for interacting with AI chips compliant with WIA-SEMI-004 standard.

## Installation

```bash
npm install @wia/semi-004-sdk
```

## Usage

```typescript
import { AIChip, Precision, ChipUtils } from '@wia/semi-004-sdk';

// Initialize chip
const chip = new AIChip("0");
await chip.initialize();

// Get device info
const info = await chip.getDeviceInfo();
console.log(info);

// Load model
const model = await chip.loadModel({
  path: "model.onnx",
  precision: Precision.INT8,
  batchSize: 32
});

// Run inference
const result = await model.inference({
  inputs: [/* tensors */]
});

// Run benchmark
const benchmark = await chip.runBenchmark(
  "ResNet-50",
  32,
  Precision.INT8
);

// Cleanup
await model.unload();
await chip.shutdown();
```

## License

MIT © 2025 WIA
