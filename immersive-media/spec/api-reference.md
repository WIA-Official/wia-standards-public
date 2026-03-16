# WIA-EDU-021: API Reference

**Version:** 1.0.0
**Language:** TypeScript/JavaScript

## Overview

The WIA Immersive Media SDK provides a unified API for creating, managing, and delivering immersive educational content across VR/AR/XR platforms.

## Installation

```bash
npm install @wia/immersive-media
```

## Quick Start

```typescript
import { ImmersiveMedia } from '@wia/immersive-media';

// Initialize SDK
const im = new ImmersiveMedia({
  apiKey: 'your-api-key',
  region: 'us-west-1'
});

// Create 360° video experience
const video360 = im.create360Video({
  source: 'ancient-rome-tour-8k.mp4',
  projection: 'equirectangular',
  stereoMode: 'mono'
});

await video360.play();
```

## Core Classes

### ImmersiveMedia

Main SDK class for managing immersive content.

#### Constructor

```typescript
constructor(config: ImmersiveMediaConfig)
```

**Parameters:**
- `config.apiKey` (string): Your WIA API key
- `config.region?` (string): Deployment region (default: 'us-west-1')
- `config.quality?` ('low' | 'medium' | 'high' | 'auto'): Default quality setting

**Example:**
```typescript
const im = new ImmersiveMedia({
  apiKey: process.env.WIA_API_KEY,
  region: 'eu-west-1',
  quality: 'high'
});
```

## 360° Video API

### Video360

Class for managing 360-degree video content.

#### Methods

##### `create360Video(config: Video360Config): Video360`

Create a new 360° video instance.

```typescript
const video = im.create360Video({
  source: 'chemistry-lab-tour.mp4',
  projection: 'equirectangular',
  stereoMode: 'top-bottom',
  autoplay: false,
  controls: true,
  fov: 110,
  initialView: { yaw: 0, pitch: 0 }
});
```

##### `addHotspot(config: HotspotConfig): Hotspot`

Add interactive hotspot to 360° video.

```typescript
video.addHotspot({
  position: { yaw: 45, pitch: 10 },
  time: 30.5, // seconds
  content: 'This is the Bunsen burner',
  type: 'info',
  onClick: () => showDetailedInfo()
});
```

##### `play(): Promise<void>`
##### `pause(): void`
##### `seek(time: number): void`
##### `setQuality(quality: '4k' | '5.7k' | '8k'): void`

## 3D Model API

### Model3D

Class for interactive 3D models and visualizations.

#### Methods

##### `create3DModel(config: Model3DConfig): Model3D`

Load and display 3D models.

```typescript
const model = im.create3DModel({
  source: 'dna-molecule.glb',
  position: [0, 1.5, -3],
  scale: [2, 2, 2],
  rotation: [0, 0, 0],
  animations: ['rotate', 'unzip'],
  lighting: 'realistic'
});

await model.load();
```

##### `playAnimation(name: string, loop?: boolean): void`

```typescript
model.playAnimation('unzip', false);
```

##### `addAnnotation(config: AnnotationConfig): Annotation`

```typescript
model.addAnnotation({
  position: [0, 2, 0],
  label: 'Nucleotide Base Pair',
  description: 'Adenine-Thymine bond',
  autoShow: true
});
```

##### `setMaterial(materialConfig: MaterialConfig): void`
##### `rotate(axis: Vector3, angle: number): void`
##### `scale(factor: number | Vector3): void`

## Spatial Audio API

### SpatialAudio

Class for 3D positional audio.

#### Methods

##### `createSpatialAudio(config: SpatialAudioConfig): SpatialAudio`

```typescript
const audio = im.createSpatialAudio({
  environment: 'classroom',
  reverb: 0.3,
  attenuation: 'inverse'
});
```

##### `addSource(config: AudioSourceConfig): AudioSource`

```typescript
const narration = audio.addSource({
  id: 'narrator',
  position: [0, 1.7, 2],
  audio: 'narration.mp3',
  volume: 0.8,
  loop: false,
  spatial: true
});
```

##### `setListenerPosition(position: Vector3): void`

```typescript
// Update listener position (typically user's head position)
audio.setListenerPosition([0, 1.7, 0]);
```

##### `updateSourcePosition(id: string, position: Vector3): void`

## Haptic Feedback API

### Haptic

Class for haptic feedback control.

#### Methods

##### `createHaptic(config: HapticConfig): Haptic`

```typescript
const haptic = im.createHaptic({
  device: 'controller',
  hand: 'right'
});
```

##### `trigger(effect: HapticEffect): Promise<void>`

```typescript
await haptic.trigger({
  type: 'impact',
  intensity: 0.8,
  duration: 100
});
```

##### `playPattern(pattern: HapticPattern[]): Promise<void>`

```typescript
await haptic.playPattern([
  { intensity: 0.5, duration: 100 },
  { intensity: 0, duration: 50 },
  { intensity: 0.8, duration: 150 }
]);
```

## XR Device API

### XRDevice

Class for managing VR/AR device features.

#### Methods

##### `getCapabilities(): Promise<DeviceCapabilities>`

```typescript
const caps = await im.getCapabilities();
console.log('VR Supported:', caps.vr);
console.log('AR Supported:', caps.ar);
console.log('Hand Tracking:', caps.handTracking);
```

##### `startSession(mode: 'immersive-vr' | 'immersive-ar' | 'inline'): Promise<XRSession>`

```typescript
const session = await im.startSession('immersive-vr');

session.on('controller-connected', (controller) => {
  console.log('Controller:', controller.handedness);
});
```

##### `enableHandTracking(): Promise<void>`
##### `enablePassthrough(): Promise<void>`

## Learning Analytics API

### Analytics

Class for tracking educational metrics.

#### Methods

##### `trackEvent(event: AnalyticsEvent): void`

```typescript
im.analytics.trackEvent({
  type: 'interaction',
  target: 'dna-molecule',
  action: 'rotate',
  timestamp: Date.now()
});
```

##### `trackProgress(config: ProgressConfig): void`

```typescript
im.analytics.trackProgress({
  lessonId: 'molecular-biology-101',
  section: 'dna-structure',
  completionPercentage: 75,
  score: 85
});
```

##### `getEngagementMetrics(): Promise<EngagementMetrics>`

## Events

### Event Types

All major classes extend `EventEmitter` and emit standard events:

```typescript
video360.on('play', () => console.log('Playing'));
video360.on('pause', () => console.log('Paused'));
video360.on('ended', () => console.log('Completed'));
video360.on('error', (error) => console.error(error));
video360.on('hotspot-click', (hotspot) => console.log(hotspot.content));

model.on('loaded', () => console.log('Model ready'));
model.on('animation-complete', (name) => console.log(`${name} finished`));

audio.on('source-added', (source) => console.log('Audio source added'));
audio.on('playback-started', () => console.log('Audio playing'));

session.on('controller-connected', (controller) => {});
session.on('hand-tracking-enabled', () => {});
session.on('session-ended', () => {});
```

## Type Definitions

### ImmersiveMediaConfig

```typescript
interface ImmersiveMediaConfig {
  apiKey: string;
  region?: string;
  quality?: 'low' | 'medium' | 'high' | 'auto';
  logging?: boolean;
}
```

### Video360Config

```typescript
interface Video360Config {
  source: string;
  projection: 'equirectangular' | 'cubemap' | 'fisheye';
  stereoMode: 'mono' | 'top-bottom' | 'left-right';
  autoplay?: boolean;
  controls?: boolean;
  fov?: number; // 60-120
  initialView?: { yaw: number; pitch: number };
}
```

### Model3DConfig

```typescript
interface Model3DConfig {
  source: string;
  position?: Vector3;
  scale?: Vector3;
  rotation?: Vector3;
  animations?: string[];
  lighting?: 'realistic' | 'stylized' | 'none';
  castShadows?: boolean;
}
```

### SpatialAudioConfig

```typescript
interface SpatialAudioConfig {
  environment: 'outdoor' | 'classroom' | 'auditorium' | 'cave';
  reverb?: number; // 0.0-1.0
  attenuation?: 'linear' | 'inverse' | 'exponential';
}
```

### Vector3

```typescript
type Vector3 = [number, number, number]; // [x, y, z]
```

## Error Handling

```typescript
try {
  const video = await im.create360Video(config);
  await video.play();
} catch (error) {
  if (error instanceof ImmersiveMediaError) {
    console.error('IM Error:', error.code, error.message);
  }
}
```

### Error Codes

- `INVALID_CONFIG`: Configuration validation failed
- `RESOURCE_NOT_FOUND`: Asset file not found
- `UNSUPPORTED_FORMAT`: File format not supported
- `DEVICE_NOT_SUPPORTED`: Device lacks required capabilities
- `NETWORK_ERROR`: Network request failed
- `PLAYBACK_ERROR`: Media playback error

---

For complete TypeScript definitions, see [types.ts](../api/typescript/src/types.ts)

© 2025 SmileStory Inc. / WIA
