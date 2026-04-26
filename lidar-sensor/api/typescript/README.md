# @wia/lidar-sensor

WIA-SEMI-014 LiDAR Sensor TypeScript SDK

## Installation

```bash
npm install @wia/lidar-sensor
```

## Quick Start

```typescript
import { LiDARSensor, PointCloudProcessor } from '@wia/lidar-sensor';

// Connect to LiDAR sensor
const sensor = new LiDARSensor({ host: '192.168.1.201' });

sensor.on('pointCloud', (cloud) => {
  console.log(`Received ${cloud.pointCount} points`);
  
  // Filter by range
  const filtered = PointCloudProcessor.filterByRange(cloud, 1.0, 100.0);
  
  // Downsample
  const downsampled = PointCloudProcessor.voxelGridDownsample(filtered, 0.1);
  
  console.log(`After processing: ${downsampled.pointCount} points`);
});

await sensor.connect();
```

## Features

- TypeScript type definitions for WIA-SEMI-014 specification
- Point cloud streaming and processing
- Sensor configuration and status monitoring
- Coordinate transformations and calibration
- Filtering, clustering, and ground plane estimation

## API Documentation

See `src/types.ts` for complete type definitions.

## License

MIT

## Copyright

© 2025 WIA (World Industry Association)
弘益人間 (홍익인간) - Benefit All Humanity
