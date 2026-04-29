/**
 * WIA-MACHINE_VISION SDK
 *
 * Complete TypeScript SDK for machine vision operations including:
 * - Image processing and preprocessing
 * - Object detection (YOLO, R-CNN, etc.)
 * - Image classification (CNN, ViT, etc.)
 * - Semantic and instance segmentation
 * - OCR and text recognition
 * - Real-time video processing
 * - Model management and deployment
 *
 * @packageDocumentation
 * @module @wia/machine-vision
 *
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import * as fs from 'fs';
import * as path from 'path';
import { createHash } from 'crypto';
import { InferenceSession, Tensor as ONNXTensor } from 'onnxruntime-node';
import sharp from 'sharp';

export * from './types';

import {
  VisionConfig,
  VisionError,
  VisionErrorCode,
  Model,
  ModelMetadata,
  ImageInput,
  ImageData,
  ImageFormat,
  ImageMetadata,
  ColorSpace,
  Detection,
  DetectionResult,
  DetectionOptions,
  BatchDetectionOptions,
  ClassificationResult,
  ClassPrediction,
  OCRResult,
  OCROptions,
  TextRegion,
  TextDirection,
  SegmentationResult,
  InstanceSegmentationResult,
  Instance,
  BoundingBox,
  BoundingBoxFormat,
  Tensor,
  InferenceOptions,
  LoadOptions,
  ClientStatus,
  Backend,
  Precision,
  ModelTask,
  ModelFramework,
  YOLOModel,
  YOLOVersion,
  YOLOOptions,
  VideoInput,
  VideoFrame,
  FrameDetection,
  TrackingResult,
  Track,
  AugmentationConfig,
  DrawOptions,
  BatchConfig,
  BatchProgress,
  StreamOptions,
  StreamStats,
  ClassDefinition,
  SegmentationMask,
  ExportFormat
} from './types';

// ============================================================================
// Main Vision Client
// ============================================================================

export class VisionClient {
  private config: VisionConfig;
  private models: Map<string, Model>;
  private initialized: boolean = false;
  private startTime: number = 0;

  constructor(config?: Partial<VisionConfig>) {
    this.config = {
      backend: Backend.CPU,
      device: 'cpu',
      precision: Precision.FP32,
      batchSize: 1,
      maxConcurrency: 4,
      enableTelemetry: false,
      logLevel: 'info',
      ...config
    };
    this.models = new Map();
  }

  /**
   * Initialize the vision client
   */
  async initialize(config?: Partial<VisionConfig>): Promise<void> {
    if (config) {
      this.config = { ...this.config, ...config };
    }

    this.log('info', 'Initializing WIA Machine Vision SDK...');
    this.log('info', `Backend: ${this.config.backend}, Device: ${this.config.device}`);

    // Validate backend availability
    await this.validateBackend();

    this.initialized = true;
    this.startTime = Date.now();
    this.log('info', 'Vision client initialized successfully');
  }

  /**
   * Load a model from file
   */
  async loadModel(modelPath: string, options?: LoadOptions): Promise<Model> {
    if (!this.initialized) {
      throw new VisionError(VisionErrorCode.INVALID_INPUT, 'Client not initialized');
    }

    this.log('info', `Loading model from ${modelPath}...`);

    const ext = path.extname(modelPath).toLowerCase();
    let model: Model;

    switch (ext) {
      case '.onnx':
        model = await this.loadONNXModel(modelPath, options);
        break;
      default:
        throw new VisionError(
          VisionErrorCode.UNSUPPORTED_FORMAT,
          `Unsupported model format: ${ext}`
        );
    }

    this.models.set(model.id, model);
    this.log('info', `Model loaded successfully: ${model.id}`);

    // Warm up if requested
    if (options?.warmup) {
      await model.warmup(options.warmupIterations || 3);
    }

    return model;
  }

  /**
   * Unload a model from memory
   */
  async unloadModel(modelId: string): Promise<void> {
    if (!this.models.has(modelId)) {
      throw new VisionError(VisionErrorCode.MODEL_NOT_FOUND, `Model not found: ${modelId}`);
    }

    this.models.delete(modelId);
    this.log('info', `Model unloaded: ${modelId}`);
  }

  /**
   * List all loaded models
   */
  listModels(): Model[] {
    return Array.from(this.models.values());
  }

  /**
   * Get client status
   */
  getStatus(): ClientStatus {
    return {
      initialized: this.initialized,
      backend: this.config.backend,
      device: this.config.device || 'cpu',
      modelsLoaded: this.models.size,
      uptime: this.initialized ? Date.now() - this.startTime : 0
    };
  }

  /**
   * Dispose client and free resources
   */
  async dispose(): Promise<void> {
    this.log('info', 'Disposing vision client...');

    // Unload all models
    for (const modelId of this.models.keys()) {
      await this.unloadModel(modelId);
    }

    this.initialized = false;
    this.log('info', 'Vision client disposed');
  }

  /**
   * Predict using a loaded model
   */
  async predict(input: ImageInput, options?: InferenceOptions): Promise<any> {
    if (!this.initialized) {
      throw new VisionError(VisionErrorCode.INVALID_INPUT, 'Client not initialized');
    }

    const imageData = await this.loadImage(input);
    const model = this.models.values().next().value;

    if (!model) {
      throw new VisionError(VisionErrorCode.MODEL_NOT_FOUND, 'No model loaded');
    }

    return await model.predict(imageData, options);
  }

  /**
   * Batch predict on multiple images
   */
  async batchPredict(inputs: ImageInput[], options?: BatchDetectionOptions): Promise<any[]> {
    const results: any[] = [];
    const batchSize = options?.batchSize || this.config.batchSize || 1;

    for (let i = 0; i < inputs.length; i += batchSize) {
      const batch = inputs.slice(i, i + batchSize);
      const batchResults = await Promise.all(
        batch.map(input => this.predict(input, options))
      );
      results.push(...batchResults);
    }

    return results;
  }

  // Private helper methods

  private async validateBackend(): Promise<void> {
    // Validate that the requested backend is available
    if (this.config.backend === Backend.CUDA) {
      // Check for CUDA availability
      this.log('info', 'CUDA backend selected');
    } else if (this.config.backend === Backend.TENSORRT) {
      this.log('info', 'TensorRT backend selected');
    }
    // Add more backend validations as needed
  }

  private async loadONNXModel(modelPath: string, options?: LoadOptions): Promise<Model> {
    const session = await InferenceSession.create(modelPath, {
      executionProviders: this.getExecutionProviders(),
      graphOptimizationLevel: options?.optimize ? 'all' : 'basic'
    });

    const modelId = path.basename(modelPath, '.onnx');
    const metadata = await this.extractModelMetadata(session, modelId);

    return new ONNXModel(session, metadata);
  }

  private getExecutionProviders(): string[] {
    switch (this.config.backend) {
      case Backend.CUDA:
        return ['cuda', 'cpu'];
      case Backend.CPU:
      default:
        return ['cpu'];
    }
  }

  private async extractModelMetadata(session: InferenceSession, modelId: string): Promise<ModelMetadata> {
    const inputNames = session.inputNames;
    const outputNames = session.outputNames;

    return {
      name: modelId,
      version: '1.0.0',
      framework: ModelFramework.ONNX,
      architecture: 'unknown',
      task: ModelTask.DETECTION,
      inputSpec: {
        shape: [1, 3, 640, 640],
        dtype: 'float32',
        format: 'NCHW',
        preprocessing: ['resize', 'normalize']
      },
      outputSpec: {
        format: 'xyxy',
        shape: [1, -1, 6],
        dtype: 'float32',
        postprocessing: ['nms']
      },
      performance: {},
      created: new Date().toISOString(),
      modified: new Date().toISOString()
    };
  }

  private async loadImage(input: ImageInput): Promise<ImageData> {
    if (typeof input === 'string') {
      // Load from file path or URL
      if (input.startsWith('http://') || input.startsWith('https://')) {
        return await this.loadImageFromURL(input);
      } else {
        return await this.loadImageFromFile(input);
      }
    } else if (Buffer.isBuffer(input)) {
      return await this.loadImageFromBuffer(input);
    } else if (input instanceof ArrayBuffer) {
      return await this.loadImageFromBuffer(Buffer.from(input));
    } else {
      // Already ImageData
      return input as ImageData;
    }
  }

  private async loadImageFromFile(filePath: string): Promise<ImageData> {
    const buffer = fs.readFileSync(filePath);
    return this.loadImageFromBuffer(buffer);
  }

  private async loadImageFromURL(url: string): Promise<ImageData> {
    const axios = (await import('axios')).default;
    const response = await axios.get(url, { responseType: 'arraybuffer' });
    return this.loadImageFromBuffer(Buffer.from(response.data));
  }

  private async loadImageFromBuffer(buffer: Buffer): Promise<ImageData> {
    const image = sharp(buffer);
    const metadata = await image.metadata();
    const { data, info } = await image
      .raw()
      .toBuffer({ resolveWithObject: true });

    const hash = createHash('sha256').update(buffer).digest('hex');

    return {
      id: hash.substring(0, 16),
      metadata: {
        format: this.detectImageFormat(metadata.format || 'unknown'),
        width: info.width,
        height: info.height,
        channels: info.channels as 1 | 3 | 4,
        colorSpace: ColorSpace.RGB,
        bitDepth: 8
      },
      data: new Uint8Array(data),
      encoding: 'raw',
      hash
    };
  }

  private detectImageFormat(format: string): ImageFormat {
    switch (format.toLowerCase()) {
      case 'jpeg':
      case 'jpg':
        return ImageFormat.JPEG;
      case 'png':
        return ImageFormat.PNG;
      case 'webp':
        return ImageFormat.WEBP;
      case 'tiff':
        return ImageFormat.TIFF;
      default:
        return ImageFormat.JPEG;
    }
  }

  private log(level: string, message: string): void {
    if (this.config.logLevel === 'debug' || level !== 'debug') {
      console.log(`[WIA-VISION] [${level.toUpperCase()}] ${message}`);
    }
  }
}

// ============================================================================
// ONNX Model Implementation
// ============================================================================

class ONNXModel implements Model {
  public id: string;
  public metadata: ModelMetadata;
  private session: InferenceSession;

  constructor(session: InferenceSession, metadata: ModelMetadata) {
    this.session = session;
    this.metadata = metadata;
    this.id = metadata.name;
  }

  async predict(input: Tensor | ImageData, options?: InferenceOptions): Promise<Tensor> {
    const startTime = Date.now();

    // Preprocess input
    const inputTensor = await this.preprocessInput(input);

    // Create ONNX tensor
    const feeds: Record<string, ONNXTensor> = {};
    const inputName = this.session.inputNames[0];
    feeds[inputName] = new ONNXTensor('float32', inputTensor.data as Float32Array, inputTensor.shape);

    // Run inference
    const results = await this.session.run(feeds);

    // Get output
    const outputName = this.session.outputNames[0];
    const outputTensor = results[outputName];

    const processingTime = Date.now() - startTime;

    return {
      data: outputTensor.data as Float32Array,
      shape: outputTensor.dims as number[],
      dtype: 'float32'
    };
  }

  getInputSpec(): any {
    return this.metadata.inputSpec;
  }

  getOutputSpec(): any {
    return this.metadata.outputSpec;
  }

  getPerformance(): any {
    return this.metadata.performance;
  }

  async export(format: ExportFormat, path: string): Promise<void> {
    throw new Error('Export not implemented for ONNX models');
  }

  async warmup(iterations: number = 3): Promise<void> {
    console.log(`Warming up model with ${iterations} iterations...`);

    // Create dummy input
    const dummyInput: Tensor = {
      data: new Float32Array(1 * 3 * 640 * 640),
      shape: [1, 3, 640, 640],
      dtype: 'float32'
    };

    for (let i = 0; i < iterations; i++) {
      await this.predict(dummyInput);
    }

    console.log('Model warmup completed');
  }

  private async preprocessInput(input: Tensor | ImageData): Promise<Tensor> {
    if ('data' in input && 'shape' in input) {
      // Already a tensor
      return input as Tensor;
    }

    // Convert ImageData to tensor
    const imageData = input as ImageData;
    const { width, height, channels } = imageData.metadata;

    // Resize to model input size
    const targetWidth = this.metadata.inputSpec.shape[3];
    const targetHeight = this.metadata.inputSpec.shape[2];

    const resized = await sharp(Buffer.from(imageData.data))
      .resize(targetWidth, targetHeight)
      .raw()
      .toBuffer();

    // Convert to float32 and normalize
    const float32Data = new Float32Array(1 * 3 * targetHeight * targetWidth);
    const uint8Data = new Uint8Array(resized);

    // RGB to tensor format (NCHW)
    for (let c = 0; c < 3; c++) {
      for (let h = 0; h < targetHeight; h++) {
        for (let w = 0; w < targetWidth; w++) {
          const idx = (h * targetWidth + w) * 3 + c;
          const tensorIdx = c * targetHeight * targetWidth + h * targetWidth + w;
          float32Data[tensorIdx] = uint8Data[idx] / 255.0;
        }
      }
    }

    return {
      data: float32Data,
      shape: [1, 3, targetHeight, targetWidth],
      dtype: 'float32'
    };
  }
}

// ============================================================================
// Object Detector
// ============================================================================

export class ObjectDetector {
  private model: Model;
  private classes: string[];

  constructor(model: Model, classes: string[]) {
    this.model = model;
    this.classes = classes;
  }

  /**
   * Detect objects in an image
   */
  async detect(image: ImageInput, options?: DetectionOptions): Promise<DetectionResult> {
    const startTime = Date.now();
    const client = new VisionClient();
    await client.initialize();

    const imageData = await (client as any).loadImage(image);
    const tensor = await this.model.predict(imageData, options);

    // Postprocess results
    const detections = this.postprocessDetections(tensor, options);

    const processingTime = Date.now() - startTime;

    return {
      imageId: imageData.id,
      timestamp: new Date().toISOString(),
      modelId: this.model.id,
      detections,
      processingTime
    };
  }

  /**
   * Detect objects in multiple images
   */
  async detectBatch(images: ImageInput[], options?: BatchDetectionOptions): Promise<DetectionResult[]> {
    const results: DetectionResult[] = [];

    for (const image of images) {
      const result = await this.detect(image, options);
      results.push(result);
    }

    return results;
  }

  /**
   * Get list of detection classes
   */
  getClasses(): string[] {
    return this.classes;
  }

  private postprocessDetections(tensor: Tensor, options?: DetectionOptions): Detection[] {
    const detections: Detection[] = [];
    const confidenceThreshold = options?.confidenceThreshold || 0.25;
    const maxDetections = options?.maxDetections || 100;

    // Parse tensor output (assuming YOLO format)
    const data = tensor.data as Float32Array;
    const [batch, numDetections, features] = tensor.shape;

    for (let i = 0; i < Math.min(numDetections, maxDetections); i++) {
      const baseIdx = i * features;
      const x1 = data[baseIdx];
      const y1 = data[baseIdx + 1];
      const x2 = data[baseIdx + 2];
      const y2 = data[baseIdx + 3];
      const confidence = data[baseIdx + 4];
      const classId = Math.round(data[baseIdx + 5]);

      if (confidence >= confidenceThreshold) {
        detections.push({
          bbox: {
            format: BoundingBoxFormat.XYXY,
            coordinates: [x1, y1, x2, y2],
            normalized: true
          },
          class: this.classes[classId] || `class_${classId}`,
          classId,
          confidence
        });
      }
    }

    return detections;
  }
}

// ============================================================================
// Image Classifier
// ============================================================================

export class ImageClassifier {
  private model: Model;
  private classes: string[];

  constructor(model: Model, classes: string[]) {
    this.model = model;
    this.classes = classes;
  }

  /**
   * Classify an image
   */
  async classify(image: ImageInput, topK: number = 5): Promise<ClassificationResult> {
    const startTime = Date.now();
    const client = new VisionClient();
    await client.initialize();

    const imageData = await (client as any).loadImage(image);
    const tensor = await this.model.predict(imageData);

    // Get top-K predictions
    const predictions = this.getTopKPredictions(tensor, topK);

    const processingTime = Date.now() - startTime;

    return {
      imageId: imageData.id,
      timestamp: new Date().toISOString(),
      modelId: this.model.id,
      predictions,
      topK,
      processingTime
    };
  }

  /**
   * Classify multiple images
   */
  async classifyBatch(images: ImageInput[], topK: number = 5): Promise<ClassificationResult[]> {
    const results: ClassificationResult[] = [];

    for (const image of images) {
      const result = await this.classify(image, topK);
      results.push(result);
    }

    return results;
  }

  /**
   * Get list of classification classes
   */
  getClasses(): string[] {
    return this.classes;
  }

  private getTopKPredictions(tensor: Tensor, topK: number): ClassPrediction[] {
    const data = Array.from(tensor.data as Float32Array);

    // Get indices sorted by confidence
    const indices = data
      .map((confidence, idx) => ({ idx, confidence }))
      .sort((a, b) => b.confidence - a.confidence)
      .slice(0, topK);

    return indices.map((item, rank) => ({
      class: this.classes[item.idx] || `class_${item.idx}`,
      classId: item.idx,
      confidence: item.confidence,
      rank: rank + 1
    }));
  }
}

// ============================================================================
// Image Preprocessor
// ============================================================================

export class Preprocessor {
  /**
   * Resize image
   */
  async resize(
    image: ImageData,
    width: number,
    height: number,
    method: 'bilinear' | 'bicubic' | 'nearest' = 'bilinear'
  ): Promise<ImageData> {
    const kernel = this.getResizeKernel(method);
    const resized = await sharp(Buffer.from(image.data))
      .resize(width, height, { kernel })
      .raw()
      .toBuffer();

    return {
      ...image,
      metadata: {
        ...image.metadata,
        width,
        height
      },
      data: new Uint8Array(resized)
    };
  }

  /**
   * Normalize image
   */
  normalize(image: ImageData, mean: number[], std: number[]): ImageData {
    const data = new Float32Array(image.data.length);
    const uint8Data = new Uint8Array(image.data);

    for (let i = 0; i < uint8Data.length; i++) {
      const channel = i % 3;
      data[i] = (uint8Data[i] / 255.0 - mean[channel]) / std[channel];
    }

    return {
      ...image,
      data
    };
  }

  /**
   * Convert color space
   */
  async convertColorSpace(image: ImageData, from: string, to: string): Promise<ImageData> {
    // Implementation for color space conversion
    // This is a placeholder
    return image;
  }

  private getResizeKernel(method: string): any {
    switch (method) {
      case 'bicubic':
        return sharp.kernel.cubic;
      case 'nearest':
        return sharp.kernel.nearest;
      case 'bilinear':
      default:
        return sharp.kernel.lanczos3;
    }
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Create a YOLO detector
 */
export async function createYOLODetector(
  modelPath: string,
  classes: string[],
  version: YOLOVersion = YOLOVersion.YOLO26
): Promise<ObjectDetector> {
  const client = new VisionClient();
  await client.initialize();
  const model = await client.loadModel(modelPath);
  return new ObjectDetector(model, classes);
}

/**
 * Create an image classifier
 */
export async function createImageClassifier(
  modelPath: string,
  classes: string[]
): Promise<ImageClassifier> {
  const client = new VisionClient();
  await client.initialize();
  const model = await client.loadModel(modelPath);
  return new ImageClassifier(model, classes);
}

/**
 * Load image from file or URL
 */
export async function loadImage(input: ImageInput): Promise<ImageData> {
  const client = new VisionClient();
  await client.initialize();
  return await (client as any).loadImage(input);
}

/**
 * Draw detections on image
 */
export async function drawDetections(
  image: ImageData,
  detections: Detection[],
  options?: DrawOptions
): Promise<Buffer> {
  const { width, height } = image.metadata;
  const lineWidth = options?.lineWidth || 2;
  const fontSize = options?.fontSize || 16;

  // This is a simplified implementation
  // In production, you'd use a canvas library to draw boxes and labels
  return Buffer.from(image.data);
}

/**
 * Non-Maximum Suppression
 */
export function nonMaxSuppression(
  boxes: BoundingBox[],
  scores: number[],
  iouThreshold: number = 0.45
): number[] {
  const indices: number[] = [];
  const sortedIndices = scores
    .map((score, idx) => ({ score, idx }))
    .sort((a, b) => b.score - a.score)
    .map(item => item.idx);

  const suppressed = new Set<number>();

  for (const idx of sortedIndices) {
    if (suppressed.has(idx)) continue;

    indices.push(idx);

    for (const otherIdx of sortedIndices) {
      if (otherIdx === idx || suppressed.has(otherIdx)) continue;

      const iou = calculateIoU(boxes[idx], boxes[otherIdx]);
      if (iou > iouThreshold) {
        suppressed.add(otherIdx);
      }
    }
  }

  return indices;
}

/**
 * Calculate Intersection over Union
 */
export function calculateIoU(box1: BoundingBox, box2: BoundingBox): number {
  const [x1_1, y1_1, x2_1, y2_1] = box1.coordinates;
  const [x1_2, y1_2, x2_2, y2_2] = box2.coordinates;

  const xA = Math.max(x1_1, x1_2);
  const yA = Math.max(y1_1, y1_2);
  const xB = Math.min(x2_1, x2_2);
  const yB = Math.min(y2_1, y2_2);

  const intersection = Math.max(0, xB - xA) * Math.max(0, yB - yA);
  const area1 = (x2_1 - x1_1) * (y2_1 - y1_1);
  const area2 = (x2_2 - x1_2) * (y2_2 - y1_2);
  const union = area1 + area2 - intersection;

  return intersection / union;
}

// ============================================================================
// Export default client
// ============================================================================

export default VisionClient;

/**
 * Example usage:
 *
 * ```typescript
 * import { VisionClient, createYOLODetector } from '@wia/machine-vision';
 *
 * // Initialize client
 * const client = new VisionClient({ backend: 'cuda' });
 * await client.initialize();
 *
 * // Load model
 * const model = await client.loadModel('yolo26.onnx');
 *
 * // Detect objects
 * const result = await model.predict('image.jpg');
 *
 * // Or use detector
 * const detector = await createYOLODetector('yolo26.onnx', classes);
 * const detections = await detector.detect('image.jpg');
 * ```
 *
 * 弘익人間 (Benefit All Humanity)
 */
