# WIA-MACHINE_VISION: PHASE 1 - Data Format Specification

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2026-01-12
**Philosophy:** 弘益人間 (Benefit All Humanity)

## Overview

This specification defines the data formats, image standards, annotation structures, and model data representations for the WIA Machine Vision standard. It ensures interoperability across computer vision systems, frameworks, and deployment platforms.

## 1. Image Data Formats

### 1.1 Supported Image Formats

```typescript
enum ImageFormat {
  JPEG = 'image/jpeg',
  PNG = 'image/png',
  WEBP = 'image/webp',
  TIFF = 'image/tiff',
  BMP = 'image/bmp',
  RAW = 'image/raw',
  HEIF = 'image/heif'
}

interface ImageMetadata {
  format: ImageFormat;
  width: number;
  height: number;
  channels: 1 | 3 | 4;  // Grayscale, RGB, RGBA
  colorSpace: 'RGB' | 'BGR' | 'HSV' | 'LAB' | 'YCbCr';
  bitDepth: 8 | 16 | 32;
  dpi?: number;
  exif?: Record<string, any>;
  captureTimestamp?: string;
  deviceInfo?: DeviceInfo;
}

interface DeviceInfo {
  manufacturer?: string;
  model?: string;
  sensor?: string;
  lens?: string;
}
```

### 1.2 Image Data Structure

```typescript
interface ImageData {
  id: string;
  metadata: ImageMetadata;
  data: ArrayBuffer | Uint8Array | Float32Array;
  encoding: 'raw' | 'base64' | 'binary';
  preprocessing?: PreprocessingInfo[];
  hash: string;  // SHA-256 hash for integrity
}

interface PreprocessingInfo {
  operation: string;
  parameters: Record<string, any>;
  timestamp: string;
  version: string;
}
```

## 2. Annotation Standards

### 2.1 Object Detection Annotations

```typescript
interface BoundingBox {
  format: 'xyxy' | 'xywh' | 'cxcywh';  // Format type
  coordinates: [number, number, number, number];
  normalized: boolean;  // Normalized to [0, 1] or absolute pixels
}

interface ObjectAnnotation {
  id: string;
  bbox: BoundingBox;
  class: string;
  classId: number;
  confidence?: number;
  attributes?: Record<string, any>;
  occluded?: boolean;
  truncated?: boolean;
  difficult?: boolean;
}

interface DetectionAnnotation {
  imageId: string;
  objects: ObjectAnnotation[];
  format: 'COCO' | 'PASCAL_VOC' | 'YOLO' | 'WIA';
  version: string;
  annotator?: string;
  timestamp: string;
}
```

### 2.2 Segmentation Annotations

```typescript
interface SegmentationMask {
  format: 'rle' | 'polygon' | 'bitmap';
  data: any;  // Format-specific data
  class: string;
  classId: number;
}

interface InstanceSegmentation {
  imageId: string;
  instances: Array<{
    id: string;
    mask: SegmentationMask;
    bbox: BoundingBox;
    class: string;
    classId: number;
    area: number;
    confidence?: number;
  }>;
}

interface SemanticSegmentation {
  imageId: string;
  mask: Uint8Array | Uint16Array;  // Class ID per pixel
  classes: Array<{
    id: number;
    name: string;
    color: [number, number, number];
  }>;
  width: number;
  height: number;
}
```

### 2.3 Keypoint Annotations

```typescript
interface Keypoint {
  x: number;
  y: number;
  visibility: 0 | 1 | 2;  // 0: not labeled, 1: labeled but not visible, 2: labeled and visible
  confidence?: number;
}

interface KeypointAnnotation {
  imageId: string;
  personId?: string;
  keypoints: Keypoint[];
  skeleton?: Array<[number, number]>;  // Connections between keypoints
  bbox?: BoundingBox;
}
```

### 2.4 OCR Annotations

```typescript
interface TextRegion {
  bbox: BoundingBox;
  text: string;
  confidence: number;
  language?: string;
  direction: 'ltr' | 'rtl' | 'ttb';
}

interface OCRAnnotation {
  imageId: string;
  regions: TextRegion[];
  fullText?: string;
  language?: string;
  engine?: string;
  timestamp: string;
}
```

## 3. Model Data Structures

### 3.1 Model Metadata

```typescript
interface ModelMetadata {
  name: string;
  version: string;
  framework: 'PyTorch' | 'TensorFlow' | 'ONNX' | 'OpenCV' | 'TensorRT';
  architecture: string;  // e.g., 'YOLOv26', 'ResNet50', 'EfficientNet'
  task: ModelTask;
  inputSpec: InputSpecification;
  outputSpec: OutputSpecification;
  performance: PerformanceMetrics;
  trainingInfo?: TrainingInfo;
  license?: string;
  created: string;
  modified: string;
}

enum ModelTask {
  CLASSIFICATION = 'classification',
  DETECTION = 'detection',
  SEGMENTATION = 'segmentation',
  KEYPOINT = 'keypoint',
  OCR = 'ocr',
  TRACKING = 'tracking',
  MULTI_TASK = 'multi_task'
}
```

### 3.2 Input/Output Specifications

```typescript
interface InputSpecification {
  shape: number[];  // e.g., [1, 3, 640, 640]
  dtype: 'float32' | 'float16' | 'uint8' | 'int8';
  format: 'NCHW' | 'NHWC';
  normalization?: {
    mean: number[];
    std: number[];
    scale: number;
  };
  preprocessing: string[];  // e.g., ['resize', 'normalize', 'letterbox']
}

interface OutputSpecification {
  format: string;
  shape: number[];
  dtype: string;
  postprocessing: string[];
  threshold?: number;
  nms?: {
    iouThreshold: number;
    confidenceThreshold: number;
  };
}
```

### 3.3 Performance Metrics

```typescript
interface PerformanceMetrics {
  accuracy?: number;
  precision?: number;
  recall?: number;
  f1Score?: number;
  mAP?: number;  // Mean Average Precision
  mAP50?: number;  // mAP at IoU=0.5
  mAP95?: number;  // mAP at IoU=0.95
  fps?: number;
  latency?: {
    cpu?: number;  // ms
    gpu?: number;  // ms
    edge?: number;  // ms
  };
  modelSize?: number;  // bytes
  flops?: number;
  parameters?: number;
}
```

### 3.4 Training Information

```typescript
interface TrainingInfo {
  dataset: string;
  datasetSize: number;
  epochs: number;
  batchSize: number;
  optimizer: string;
  learningRate: number;
  losses: Record<string, number>;
  augmentations?: string[];
  hardware?: string;
  trainingTime?: number;  // seconds
  hyperparameters?: Record<string, any>;
}
```

## 4. Dataset Formats

### 4.1 Dataset Manifest

```typescript
interface DatasetManifest {
  name: string;
  version: string;
  description: string;
  task: ModelTask;
  format: 'COCO' | 'PASCAL_VOC' | 'YOLO' | 'ImageNet' | 'WIA';
  statistics: DatasetStatistics;
  splits: {
    train: DatasetSplit;
    val: DatasetSplit;
    test?: DatasetSplit;
  };
  classes: ClassDefinition[];
  license: string;
  created: string;
}

interface DatasetStatistics {
  totalImages: number;
  totalAnnotations: number;
  avgAnnotationsPerImage: number;
  imageDistribution: Record<string, number>;
  classDistribution: Record<string, number>;
}

interface DatasetSplit {
  name: string;
  imageCount: number;
  annotationCount: number;
  path: string;
  manifestFile: string;
}

interface ClassDefinition {
  id: number;
  name: string;
  supercategory?: string;
  color?: [number, number, number];
  description?: string;
}
```

## 5. Inference Results Format

### 5.1 Detection Results

```typescript
interface DetectionResult {
  imageId: string;
  timestamp: string;
  modelId: string;
  detections: Detection[];
  processingTime: number;  // ms
}

interface Detection {
  bbox: BoundingBox;
  class: string;
  classId: number;
  confidence: number;
  mask?: SegmentationMask;
  keypoints?: Keypoint[];
  attributes?: Record<string, any>;
}
```

### 5.2 Classification Results

```typescript
interface ClassificationResult {
  imageId: string;
  timestamp: string;
  modelId: string;
  predictions: ClassPrediction[];
  topK: number;
  processingTime: number;
}

interface ClassPrediction {
  class: string;
  classId: number;
  confidence: number;
  rank: number;
}
```

## 6. Video Data Formats

### 6.1 Video Metadata

```typescript
interface VideoMetadata {
  id: string;
  format: 'mp4' | 'avi' | 'mov' | 'webm' | 'mkv';
  codec: string;
  width: number;
  height: number;
  fps: number;
  duration: number;  // seconds
  frameCount: number;
  bitrate?: number;
}

interface VideoFrame {
  frameNumber: number;
  timestamp: number;  // ms
  image: ImageData;
  annotations?: any;
}
```

## 7. Serialization Formats

### 7.1 Supported Serialization

- **JSON**: Human-readable, portable
- **MessagePack**: Binary, compact
- **Protocol Buffers**: Efficient, typed
- **CBOR**: Binary JSON alternative
- **HDF5**: Large datasets, arrays
- **Parquet**: Columnar, analytical

### 7.2 File Extensions

```
.json    - JSON format
.msgpack - MessagePack format
.pb      - Protocol Buffers
.cbor    - CBOR format
.h5      - HDF5 format
.parquet - Parquet format
```

## 8. Versioning and Compatibility

### 8.1 Version Format

```
<major>.<minor>.<patch>
Example: 1.0.0
```

### 8.2 Compatibility Rules

- **Major version**: Breaking changes
- **Minor version**: New features, backward compatible
- **Patch version**: Bug fixes

## Implementation Notes

1. **Image Loading**: Support OpenCV, PIL, scikit-image
2. **Tensor Conversion**: NumPy, PyTorch, TensorFlow interop
3. **Validation**: Schema validation for all formats
4. **Compression**: Support JPEG, PNG compression levels
5. **Streaming**: Support for large images and videos

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**
