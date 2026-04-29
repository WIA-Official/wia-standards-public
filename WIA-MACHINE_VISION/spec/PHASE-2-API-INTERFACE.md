# WIA-MACHINE_VISION: PHASE 2 - API Interface Specification

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2026-01-12
**Philosophy:** 弘益人間 (Benefit All Humanity)

## Overview

This specification defines the API interfaces for machine vision operations, including model inference, image processing, object detection, classification, segmentation, and model management. It supports both synchronous and asynchronous operations across multiple deployment targets.

## 1. Core API Interface

### 1.1 Vision Client

```typescript
interface VisionClient {
  // Initialization
  initialize(config: VisionConfig): Promise<void>;

  // Model management
  loadModel(modelPath: string, options?: LoadOptions): Promise<Model>;
  unloadModel(modelId: string): Promise<void>;
  listModels(): Model[];

  // Inference operations
  predict(input: ImageInput, options?: PredictOptions): Promise<InferenceResult>;
  batchPredict(inputs: ImageInput[], options?: BatchOptions): Promise<InferenceResult[]>;

  // Streaming inference
  createStream(options: StreamOptions): VisionStream;

  // Resource management
  getStatus(): ClientStatus;
  dispose(): Promise<void>;
}

interface VisionConfig {
  backend: 'cpu' | 'cuda' | 'tensorrt' | 'openvino' | 'coreml' | 'web';
  device?: string;
  precision?: 'fp32' | 'fp16' | 'int8';
  batchSize?: number;
  maxConcurrency?: number;
  enableTelemetry?: boolean;
  logLevel?: 'debug' | 'info' | 'warn' | 'error';
}
```

### 1.2 Model Interface

```typescript
interface Model {
  id: string;
  metadata: ModelMetadata;

  // Inference
  predict(input: Tensor | ImageData, options?: InferenceOptions): Promise<Tensor>;

  // Model info
  getInputSpec(): InputSpecification;
  getOutputSpec(): OutputSpecification;
  getPerformance(): PerformanceMetrics;

  // Export
  export(format: ExportFormat, path: string): Promise<void>;

  // Warm up
  warmup(iterations?: number): Promise<void>;
}

interface InferenceOptions {
  preprocessor?: Preprocessor;
  postprocessor?: Postprocessor;
  confidenceThreshold?: number;
  iouThreshold?: number;
  maxDetections?: number;
  returnTensor?: boolean;
}
```

## 2. Detection API

### 2.1 Object Detection

```typescript
interface ObjectDetector {
  // Single image detection
  detect(image: ImageInput, options?: DetectionOptions): Promise<DetectionResult>;

  // Batch detection
  detectBatch(images: ImageInput[], options?: BatchDetectionOptions): Promise<DetectionResult[]>;

  // Video detection
  detectVideo(video: VideoInput, options?: VideoDetectionOptions): AsyncGenerator<FrameDetection>;

  // Model management
  loadWeights(path: string): Promise<void>;
  getClasses(): string[];
}

interface DetectionOptions {
  confidenceThreshold?: number;
  iouThreshold?: number;
  maxDetections?: number;
  classes?: string[];
  agnostic?: boolean;
  returnVisualization?: boolean;
}

interface DetectionResult {
  imageId: string;
  detections: Detection[];
  processingTime: number;
  visualization?: ImageData;
}

interface FrameDetection extends DetectionResult {
  frameNumber: number;
  timestamp: number;
}
```

### 2.2 YOLO-Specific API

```typescript
interface YOLODetector extends ObjectDetector {
  // YOLO26 specific features
  version: 'yolov12' | 'yolo26';

  // NMS-free inference (YOLO26)
  detectNMSFree(image: ImageInput, options?: YOLOOptions): Promise<DetectionResult>;

  // Export options
  exportONNX(path: string, options?: ONNXExportOptions): Promise<void>;
  exportTensorFlow(path: string, format: 'SavedModel' | 'GraphDef' | 'TFLite'): Promise<void>;
  exportTensorRT(path: string, precision: 'fp32' | 'fp16' | 'int8'): Promise<void>;
}

interface YOLOOptions extends DetectionOptions {
  nmsFree?: boolean;  // YOLO26 feature
  dfl?: boolean;  // Distribution Focal Loss
  progLoss?: boolean;  // Progressive Loss
  stal?: boolean;  // Spatial-Temporal Attention Layer
}
```

## 3. Classification API

### 3.1 Image Classification

```typescript
interface ImageClassifier {
  // Single classification
  classify(image: ImageInput, topK?: number): Promise<ClassificationResult>;

  // Batch classification
  classifyBatch(images: ImageInput[], topK?: number): Promise<ClassificationResult[]>;

  // Feature extraction
  extractFeatures(image: ImageInput, layer?: string): Promise<Tensor>;

  // Model info
  getClasses(): string[];
  getArchitecture(): string;
}

interface ClassificationResult {
  imageId: string;
  predictions: ClassPrediction[];
  topK: number;
  processingTime: number;
  features?: Tensor;
}
```

### 3.2 CNN Architectures

```typescript
interface CNNClassifier extends ImageClassifier {
  architecture: 'ResNet' | 'EfficientNet' | 'VGG' | 'Inception' | 'MobileNet' | 'ViT';
  depth?: number;

  // Transfer learning
  freezeLayers(layerNames: string[]): void;
  unfreezeLayers(layerNames: string[]): void;

  // Fine-tuning
  fineTune(dataset: Dataset, config: TrainingConfig): AsyncGenerator<TrainingProgress>;
}
```

## 4. Segmentation API

### 4.1 Semantic Segmentation

```typescript
interface SemanticSegmenter {
  // Segment image
  segment(image: ImageInput, options?: SegmentationOptions): Promise<SegmentationResult>;

  // Batch segmentation
  segmentBatch(images: ImageInput[]): Promise<SegmentationResult[]>;

  // Get classes
  getClasses(): ClassDefinition[];
}

interface SegmentationOptions {
  returnMask?: boolean;
  returnVisualization?: boolean;
  overlay?: boolean;
  alpha?: number;
}

interface SegmentationResult {
  imageId: string;
  mask: Uint8Array | Uint16Array;
  classes: ClassDefinition[];
  visualization?: ImageData;
  processingTime: number;
}
```

### 4.2 Instance Segmentation

```typescript
interface InstanceSegmenter {
  // Segment instances
  segment(image: ImageInput, options?: InstanceOptions): Promise<InstanceSegmentationResult>;

  // Get instance masks
  getInstanceMasks(result: InstanceSegmentationResult): SegmentationMask[];
}

interface InstanceSegmentationResult {
  imageId: string;
  instances: Instance[];
  processingTime: number;
  visualization?: ImageData;
}

interface Instance {
  id: string;
  class: string;
  classId: number;
  confidence: number;
  bbox: BoundingBox;
  mask: SegmentationMask;
  area: number;
}
```

## 5. OCR API

### 5.1 Text Detection and Recognition

```typescript
interface OCREngine {
  // Full OCR pipeline
  recognize(image: ImageInput, options?: OCROptions): Promise<OCRResult>;

  // Text detection only
  detectText(image: ImageInput): Promise<TextRegion[]>;

  // Text recognition only
  recognizeRegion(image: ImageInput, bbox: BoundingBox): Promise<string>;

  // Batch OCR
  recognizeBatch(images: ImageInput[]): Promise<OCRResult[]>;

  // Language support
  getSupportedLanguages(): string[];
  setLanguage(language: string): void;
}

interface OCROptions {
  language?: string;
  detectOrientation?: boolean;
  psm?: number;  // Page segmentation mode
  oem?: number;  // OCR engine mode
  whitelist?: string;
  blacklist?: string;
}

interface OCRResult {
  imageId: string;
  text: string;
  regions: TextRegion[];
  confidence: number;
  processingTime: number;
  metadata?: {
    orientation?: number;
    script?: string;
    language?: string;
  };
}
```

## 6. Tracking API

### 6.1 Object Tracking

```typescript
interface ObjectTracker {
  // Initialize tracker
  initialize(frame: ImageData, bbox: BoundingBox): void;

  // Update tracker
  update(frame: ImageData): Promise<TrackingResult>;

  // Multi-object tracking
  trackMultiple(frame: ImageData, detections: Detection[]): Promise<Track[]>;

  // Reset tracker
  reset(): void;
}

interface TrackingResult {
  bbox: BoundingBox;
  confidence: number;
  velocity?: [number, number];
  lost: boolean;
}

interface Track {
  id: string;
  bbox: BoundingBox;
  class: string;
  confidence: number;
  trajectory: Array<{
    frameNumber: number;
    bbox: BoundingBox;
    timestamp: number;
  }>;
  active: boolean;
}
```

## 7. Image Processing API

### 7.1 Preprocessing

```typescript
interface Preprocessor {
  // Resize operations
  resize(image: ImageData, width: number, height: number, method?: 'bilinear' | 'bicubic' | 'nearest'): ImageData;
  letterbox(image: ImageData, targetSize: [number, number], color?: [number, number, number]): ImageData;

  // Color operations
  normalize(image: ImageData, mean: number[], std: number[]): ImageData;
  convertColorSpace(image: ImageData, from: string, to: string): ImageData;

  // Augmentation
  augment(image: ImageData, config: AugmentationConfig): ImageData;

  // Batching
  createBatch(images: ImageData[]): Tensor;
}

interface AugmentationConfig {
  flip?: 'horizontal' | 'vertical' | 'both';
  rotate?: number;
  brightness?: number;
  contrast?: number;
  saturation?: number;
  hue?: number;
  crop?: CropConfig;
  noise?: NoiseConfig;
}
```

### 7.2 Postprocessing

```typescript
interface Postprocessor {
  // NMS operations
  nonMaxSuppression(boxes: BoundingBox[], scores: number[], iouThreshold: number): number[];

  // Soft NMS
  softNMS(boxes: BoundingBox[], scores: number[], options?: SoftNMSOptions): [BoundingBox[], number[]];

  // Mask operations
  decodeMask(encoded: any, format: 'rle' | 'polygon'): Uint8Array;
  encodeMask(mask: Uint8Array, format: 'rle' | 'polygon'): any;

  // Visualization
  drawDetections(image: ImageData, detections: Detection[], options?: DrawOptions): ImageData;
  drawSegmentation(image: ImageData, mask: Uint8Array, classes: ClassDefinition[], alpha?: number): ImageData;
}

interface DrawOptions {
  lineWidth?: number;
  fontSize?: number;
  showConfidence?: boolean;
  showClass?: boolean;
  colors?: Record<string, [number, number, number]>;
}
```

## 8. Model Management API

### 8.1 Model Registry

```typescript
interface ModelRegistry {
  // Register models
  register(model: Model, metadata: ModelMetadata): Promise<string>;

  // Query models
  getModel(modelId: string): Promise<Model>;
  listModels(filter?: ModelFilter): Promise<ModelMetadata[]>;
  searchModels(query: string): Promise<ModelMetadata[]>;

  // Version control
  getVersions(modelName: string): Promise<string[]>;
  getLatestVersion(modelName: string): Promise<ModelMetadata>;

  // Model lifecycle
  updateModel(modelId: string, metadata: Partial<ModelMetadata>): Promise<void>;
  deleteModel(modelId: string): Promise<void>;

  // Model download
  download(modelId: string, destination: string): AsyncGenerator<DownloadProgress>;
}

interface ModelFilter {
  task?: ModelTask;
  framework?: string;
  minAccuracy?: number;
  maxLatency?: number;
  maxSize?: number;
}
```

### 8.2 Model Optimization

```typescript
interface ModelOptimizer {
  // Quantization
  quantize(model: Model, method: 'dynamic' | 'static' | 'qat', precision: 'int8' | 'fp16'): Promise<Model>;

  // Pruning
  prune(model: Model, ratio: number, method: 'magnitude' | 'structured'): Promise<Model>;

  // Knowledge distillation
  distill(teacherModel: Model, studentModel: Model, dataset: Dataset): AsyncGenerator<TrainingProgress>;

  // Optimization
  optimize(model: Model, target: 'cpu' | 'gpu' | 'edge'): Promise<Model>;
}
```

## 9. Streaming API

### 9.1 Real-time Processing

```typescript
interface VisionStream {
  // Stream control
  start(): void;
  stop(): void;
  pause(): void;
  resume(): void;

  // Frame processing
  processFrame(frame: ImageData): Promise<InferenceResult>;

  // Events
  on(event: 'frame' | 'detection' | 'error', handler: Function): void;
  off(event: string, handler: Function): void;

  // Stats
  getStats(): StreamStats;
}

interface StreamStats {
  fps: number;
  avgLatency: number;
  framesProcessed: number;
  framesDropped: number;
  uptime: number;
}

interface StreamOptions {
  source: 'camera' | 'video' | 'rtsp' | 'custom';
  sourceConfig: any;
  model: Model;
  fps?: number;
  skipFrames?: number;
  bufferSize?: number;
}
```

## 10. Batch Processing API

### 10.1 Batch Operations

```typescript
interface BatchProcessor {
  // Configure batch
  configure(config: BatchConfig): void;

  // Process batch
  process(inputs: ImageInput[]): AsyncGenerator<BatchProgress>;

  // Results
  getResults(): InferenceResult[];
  exportResults(path: string, format: 'json' | 'csv'): Promise<void>;
}

interface BatchConfig {
  batchSize: number;
  maxConcurrency: number;
  saveResults?: boolean;
  outputDir?: string;
  onProgress?: (progress: BatchProgress) => void;
}

interface BatchProgress {
  processed: number;
  total: number;
  percentage: number;
  currentBatch: number;
  eta: number;
  errors: number;
}
```

## 11. REST API Endpoints

### 11.1 HTTP Endpoints

```
POST   /api/v1/predict              - Single prediction
POST   /api/v1/predict/batch        - Batch prediction
POST   /api/v1/detect               - Object detection
POST   /api/v1/classify             - Image classification
POST   /api/v1/segment              - Segmentation
POST   /api/v1/ocr                  - OCR
POST   /api/v1/track                - Object tracking

GET    /api/v1/models               - List models
POST   /api/v1/models               - Upload model
GET    /api/v1/models/:id           - Get model info
DELETE /api/v1/models/:id           - Delete model
GET    /api/v1/models/:id/download  - Download model

GET    /api/v1/status               - Server status
GET    /api/v1/health               - Health check
```

### 11.2 Request/Response Format

```typescript
// Request
interface PredictRequest {
  image: string;  // base64 or URL
  modelId?: string;
  options?: InferenceOptions;
}

// Response
interface PredictResponse {
  success: boolean;
  data?: InferenceResult;
  error?: {
    code: string;
    message: string;
  };
  metadata: {
    requestId: string;
    timestamp: string;
    processingTime: number;
  };
}
```

## Implementation Examples

### Example 1: Object Detection

```typescript
const client = new VisionClient();
await client.initialize({ backend: 'cuda' });

const model = await client.loadModel('yolo26.onnx');
const result = await model.predict(imageData, {
  confidenceThreshold: 0.5,
  iouThreshold: 0.45
});

console.log(`Found ${result.detections.length} objects`);
```

### Example 2: Batch Classification

```typescript
const classifier = new ImageClassifier('efficientnet-b0');
const results = await classifier.classifyBatch(images, 5);

results.forEach(result => {
  console.log(`Top prediction: ${result.predictions[0].class}`);
});
```

---

**© 2025 SmileStory Inc. / WIA**
**弘익人間 (Benefit All Humanity)**
