/**
 * WIA-MACHINE_VISION Type Definitions
 *
 * Comprehensive TypeScript interfaces for machine vision operations
 * including image processing, object detection, classification, and deep learning
 *
 * @packageDocumentation
 * @module @wia/machine-vision
 */

// ============================================================================
// Image Data Types
// ============================================================================

export enum ImageFormat {
  JPEG = 'image/jpeg',
  PNG = 'image/png',
  WEBP = 'image/webp',
  TIFF = 'image/tiff',
  BMP = 'image/bmp',
  RAW = 'image/raw',
  HEIF = 'image/heif'
}

export enum ColorSpace {
  RGB = 'RGB',
  BGR = 'BGR',
  HSV = 'HSV',
  LAB = 'LAB',
  YCbCr = 'YCbCr',
  GRAYSCALE = 'GRAYSCALE'
}

export interface ImageMetadata {
  format: ImageFormat;
  width: number;
  height: number;
  channels: 1 | 3 | 4;
  colorSpace: ColorSpace;
  bitDepth: 8 | 16 | 32;
  dpi?: number;
  exif?: Record<string, any>;
  captureTimestamp?: string;
  deviceInfo?: DeviceInfo;
}

export interface DeviceInfo {
  manufacturer?: string;
  model?: string;
  sensor?: string;
  lens?: string;
}

export interface ImageData {
  id: string;
  metadata: ImageMetadata;
  data: ArrayBuffer | Uint8Array | Float32Array;
  encoding: 'raw' | 'base64' | 'binary';
  preprocessing?: PreprocessingInfo[];
  hash: string;
}

export interface PreprocessingInfo {
  operation: string;
  parameters: Record<string, any>;
  timestamp: string;
  version: string;
}

export type ImageInput = ImageData | string | Buffer | ArrayBuffer;

// ============================================================================
// Bounding Box & Annotations
// ============================================================================

export enum BoundingBoxFormat {
  XYXY = 'xyxy',
  XYWH = 'xywh',
  CXCYWH = 'cxcywh'
}

export interface BoundingBox {
  format: BoundingBoxFormat;
  coordinates: [number, number, number, number];
  normalized: boolean;
}

export interface ObjectAnnotation {
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

export interface DetectionAnnotation {
  imageId: string;
  objects: ObjectAnnotation[];
  format: 'COCO' | 'PASCAL_VOC' | 'YOLO' | 'WIA';
  version: string;
  annotator?: string;
  timestamp: string;
}

// ============================================================================
// Segmentation Types
// ============================================================================

export enum SegmentationFormat {
  RLE = 'rle',
  POLYGON = 'polygon',
  BITMAP = 'bitmap'
}

export interface SegmentationMask {
  format: SegmentationFormat;
  data: any;
  class: string;
  classId: number;
}

export interface InstanceSegmentation {
  imageId: string;
  instances: Instance[];
}

export interface Instance {
  id: string;
  mask: SegmentationMask;
  bbox: BoundingBox;
  class: string;
  classId: number;
  area: number;
  confidence?: number;
}

export interface SemanticSegmentation {
  imageId: string;
  mask: Uint8Array | Uint16Array;
  classes: ClassDefinition[];
  width: number;
  height: number;
}

export interface ClassDefinition {
  id: number;
  name: string;
  supercategory?: string;
  color?: [number, number, number];
  description?: string;
}

// ============================================================================
// Keypoint Detection
// ============================================================================

export enum KeypointVisibility {
  NOT_LABELED = 0,
  LABELED_NOT_VISIBLE = 1,
  LABELED_AND_VISIBLE = 2
}

export interface Keypoint {
  x: number;
  y: number;
  visibility: KeypointVisibility;
  confidence?: number;
}

export interface KeypointAnnotation {
  imageId: string;
  personId?: string;
  keypoints: Keypoint[];
  skeleton?: Array<[number, number]>;
  bbox?: BoundingBox;
}

// ============================================================================
// OCR Types
// ============================================================================

export enum TextDirection {
  LTR = 'ltr',
  RTL = 'rtl',
  TTB = 'ttb'
}

export interface TextRegion {
  bbox: BoundingBox;
  text: string;
  confidence: number;
  language?: string;
  direction: TextDirection;
}

export interface OCRAnnotation {
  imageId: string;
  regions: TextRegion[];
  fullText?: string;
  language?: string;
  engine?: string;
  timestamp: string;
}

export interface OCRResult {
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

export interface OCROptions {
  language?: string;
  detectOrientation?: boolean;
  psm?: number;
  oem?: number;
  whitelist?: string;
  blacklist?: string;
}

// ============================================================================
// Model Types
// ============================================================================

export enum ModelTask {
  CLASSIFICATION = 'classification',
  DETECTION = 'detection',
  SEGMENTATION = 'segmentation',
  KEYPOINT = 'keypoint',
  OCR = 'ocr',
  TRACKING = 'tracking',
  MULTI_TASK = 'multi_task'
}

export enum ModelFramework {
  PYTORCH = 'PyTorch',
  TENSORFLOW = 'TensorFlow',
  ONNX = 'ONNX',
  OPENCV = 'OpenCV',
  TENSORRT = 'TensorRT'
}

export interface ModelMetadata {
  name: string;
  version: string;
  framework: ModelFramework;
  architecture: string;
  task: ModelTask;
  inputSpec: InputSpecification;
  outputSpec: OutputSpecification;
  performance: PerformanceMetrics;
  trainingInfo?: TrainingInfo;
  license?: string;
  created: string;
  modified: string;
}

export interface InputSpecification {
  shape: number[];
  dtype: 'float32' | 'float16' | 'uint8' | 'int8';
  format: 'NCHW' | 'NHWC';
  normalization?: {
    mean: number[];
    std: number[];
    scale: number;
  };
  preprocessing: string[];
}

export interface OutputSpecification {
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

export interface PerformanceMetrics {
  accuracy?: number;
  precision?: number;
  recall?: number;
  f1Score?: number;
  mAP?: number;
  mAP50?: number;
  mAP95?: number;
  fps?: number;
  latency?: {
    cpu?: number;
    gpu?: number;
    edge?: number;
  };
  modelSize?: number;
  flops?: number;
  parameters?: number;
}

export interface TrainingInfo {
  dataset: string;
  datasetSize: number;
  epochs: number;
  batchSize: number;
  optimizer: string;
  learningRate: number;
  losses: Record<string, number>;
  augmentations?: string[];
  hardware?: string;
  trainingTime?: number;
  hyperparameters?: Record<string, any>;
}

// ============================================================================
// Detection Types
// ============================================================================

export interface Detection {
  bbox: BoundingBox;
  class: string;
  classId: number;
  confidence: number;
  mask?: SegmentationMask;
  keypoints?: Keypoint[];
  attributes?: Record<string, any>;
}

export interface DetectionResult {
  imageId: string;
  timestamp: string;
  modelId: string;
  detections: Detection[];
  processingTime: number;
  visualization?: ImageData;
}

export interface DetectionOptions {
  confidenceThreshold?: number;
  iouThreshold?: number;
  maxDetections?: number;
  classes?: string[];
  agnostic?: boolean;
  returnVisualization?: boolean;
}

export interface BatchDetectionOptions extends DetectionOptions {
  batchSize?: number;
  maxConcurrency?: number;
}

// ============================================================================
// Classification Types
// ============================================================================

export interface ClassPrediction {
  class: string;
  classId: number;
  confidence: number;
  rank: number;
}

export interface ClassificationResult {
  imageId: string;
  timestamp: string;
  modelId: string;
  predictions: ClassPrediction[];
  topK: number;
  processingTime: number;
  features?: Tensor;
}

// ============================================================================
// Video Types
// ============================================================================

export interface VideoMetadata {
  id: string;
  format: 'mp4' | 'avi' | 'mov' | 'webm' | 'mkv';
  codec: string;
  width: number;
  height: number;
  fps: number;
  duration: number;
  frameCount: number;
  bitrate?: number;
}

export interface VideoFrame {
  frameNumber: number;
  timestamp: number;
  image: ImageData;
  annotations?: any;
}

export interface FrameDetection extends DetectionResult {
  frameNumber: number;
  timestamp: number;
}

export type VideoInput = string | Buffer | ReadableStream;

// ============================================================================
// Tracking Types
// ============================================================================

export interface TrackingResult {
  bbox: BoundingBox;
  confidence: number;
  velocity?: [number, number];
  lost: boolean;
}

export interface Track {
  id: string;
  bbox: BoundingBox;
  class: string;
  confidence: number;
  trajectory: TrajectoryPoint[];
  active: boolean;
}

export interface TrajectoryPoint {
  frameNumber: number;
  bbox: BoundingBox;
  timestamp: number;
}

// ============================================================================
// Tensor & Processing Types
// ============================================================================

export interface Tensor {
  data: Float32Array | Int32Array | Uint8Array;
  shape: number[];
  dtype: string;
}

export interface AugmentationConfig {
  flip?: 'horizontal' | 'vertical' | 'both';
  rotate?: number;
  brightness?: number;
  contrast?: number;
  saturation?: number;
  hue?: number;
  crop?: CropConfig;
  noise?: NoiseConfig;
}

export interface CropConfig {
  x: number;
  y: number;
  width: number;
  height: number;
}

export interface NoiseConfig {
  type: 'gaussian' | 'salt_pepper' | 'poisson';
  amount?: number;
  mean?: number;
  std?: number;
}

// ============================================================================
// Configuration Types
// ============================================================================

export enum Backend {
  CPU = 'cpu',
  CUDA = 'cuda',
  TENSORRT = 'tensorrt',
  OPENVINO = 'openvino',
  COREML = 'coreml',
  WEB = 'web'
}

export enum Precision {
  FP32 = 'fp32',
  FP16 = 'fp16',
  INT8 = 'int8'
}

export interface VisionConfig {
  backend: Backend;
  device?: string;
  precision?: Precision;
  batchSize?: number;
  maxConcurrency?: number;
  enableTelemetry?: boolean;
  logLevel?: 'debug' | 'info' | 'warn' | 'error';
}

export interface InferenceOptions {
  preprocessor?: any;
  postprocessor?: any;
  confidenceThreshold?: number;
  iouThreshold?: number;
  maxDetections?: number;
  returnTensor?: boolean;
}

export interface LoadOptions {
  warmup?: boolean;
  warmupIterations?: number;
  optimize?: boolean;
}

// ============================================================================
// Dataset Types
// ============================================================================

export interface DatasetManifest {
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

export interface DatasetStatistics {
  totalImages: number;
  totalAnnotations: number;
  avgAnnotationsPerImage: number;
  imageDistribution: Record<string, number>;
  classDistribution: Record<string, number>;
}

export interface DatasetSplit {
  name: string;
  imageCount: number;
  annotationCount: number;
  path: string;
  manifestFile: string;
}

export interface Dataset {
  manifest: DatasetManifest;
  getImage(index: number): Promise<ImageData>;
  getAnnotation(index: number): Promise<any>;
  size(): number;
}

// ============================================================================
// Streaming Types
// ============================================================================

export interface StreamOptions {
  source: 'camera' | 'video' | 'rtsp' | 'custom';
  sourceConfig: any;
  model: Model;
  fps?: number;
  skipFrames?: number;
  bufferSize?: number;
}

export interface StreamStats {
  fps: number;
  avgLatency: number;
  framesProcessed: number;
  framesDropped: number;
  uptime: number;
}

// ============================================================================
// Client & Model Interfaces
// ============================================================================

export interface ClientStatus {
  initialized: boolean;
  backend: Backend;
  device: string;
  modelsLoaded: number;
  memoryUsage?: number;
  uptime: number;
}

export interface Model {
  id: string;
  metadata: ModelMetadata;
  predict(input: Tensor | ImageData, options?: InferenceOptions): Promise<Tensor>;
  getInputSpec(): InputSpecification;
  getOutputSpec(): OutputSpecification;
  getPerformance(): PerformanceMetrics;
  export(format: ExportFormat, path: string): Promise<void>;
  warmup(iterations?: number): Promise<void>;
}

export enum ExportFormat {
  ONNX = 'onnx',
  TENSORFLOW_SAVED_MODEL = 'tensorflow_saved_model',
  TENSORFLOW_GRAPHDEF = 'tensorflow_graphdef',
  TFLITE = 'tflite',
  TENSORRT = 'tensorrt',
  OPENVINO = 'openvino'
}

export interface InferenceResult {
  success: boolean;
  data?: any;
  error?: string;
  processingTime: number;
  timestamp: string;
}

// ============================================================================
// Batch Processing Types
// ============================================================================

export interface BatchConfig {
  batchSize: number;
  maxConcurrency: number;
  saveResults?: boolean;
  outputDir?: string;
  onProgress?: (progress: BatchProgress) => void;
}

export interface BatchProgress {
  processed: number;
  total: number;
  percentage: number;
  currentBatch: number;
  eta: number;
  errors: number;
}

// ============================================================================
// Draw & Visualization Types
// ============================================================================

export interface DrawOptions {
  lineWidth?: number;
  fontSize?: number;
  showConfidence?: boolean;
  showClass?: boolean;
  colors?: Record<string, [number, number, number]>;
}

// ============================================================================
// Error Types
// ============================================================================

export enum VisionErrorCode {
  INVALID_INPUT = 'INVALID_INPUT',
  MODEL_NOT_FOUND = 'MODEL_NOT_FOUND',
  INFERENCE_FAILED = 'INFERENCE_FAILED',
  UNSUPPORTED_FORMAT = 'UNSUPPORTED_FORMAT',
  DEVICE_NOT_AVAILABLE = 'DEVICE_NOT_AVAILABLE',
  OUT_OF_MEMORY = 'OUT_OF_MEMORY',
  TIMEOUT = 'TIMEOUT',
  NETWORK_ERROR = 'NETWORK_ERROR'
}

export class VisionError extends Error {
  constructor(
    public code: VisionErrorCode,
    message: string,
    public details?: any
  ) {
    super(message);
    this.name = 'VisionError';
  }
}

// ============================================================================
// YOLO-Specific Types
// ============================================================================

export enum YOLOVersion {
  YOLOv8 = 'yolov8',
  YOLOv11 = 'yolov11',
  YOLOv12 = 'yolov12',
  YOLO26 = 'yolo26'
}

export interface YOLOOptions extends DetectionOptions {
  nmsFree?: boolean;
  dfl?: boolean;
  progLoss?: boolean;
  stal?: boolean;
}

export interface YOLOModel extends Model {
  version: YOLOVersion;
  detectNMSFree(image: ImageInput, options?: YOLOOptions): Promise<DetectionResult>;
  exportONNX(path: string, options?: ONNXExportOptions): Promise<void>;
  exportTensorFlow(path: string, format: 'SavedModel' | 'GraphDef' | 'TFLite'): Promise<void>;
  exportTensorRT(path: string, precision: Precision): Promise<void>;
}

export interface ONNXExportOptions {
  opsetVersion?: number;
  dynamicAxes?: Record<string, any>;
  inputNames?: string[];
  outputNames?: string[];
}

// ============================================================================
// Additional Utility Types
// ============================================================================

export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
}

export interface MotionVector {
  x: number;
  y: number;
  magnitude: number;
  angle: number;
}

export interface Event {
  type: string;
  timestamp: number;
  data: any;
}

export interface TemporalFeatures {
  motion: MotionVector[];
  objectTracking: Track[];
  sceneChanges: number[];
  events: Event[];
}

// ============================================================================
// Type Guards
// ============================================================================

export function isImageData(input: any): input is ImageData {
  return input && typeof input === 'object' && 'id' in input && 'metadata' in input && 'data' in input;
}

export function isBoundingBox(input: any): input is BoundingBox {
  return input && typeof input === 'object' && 'format' in input && 'coordinates' in input;
}

export function isDetectionResult(input: any): input is DetectionResult {
  return input && typeof input === 'object' && 'detections' in input && Array.isArray(input.detections);
}

export function isClassificationResult(input: any): input is ClassificationResult {
  return input && typeof input === 'object' && 'predictions' in input && Array.isArray(input.predictions);
}
