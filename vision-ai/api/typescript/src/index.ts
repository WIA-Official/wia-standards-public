/**
 * WIA-AI-021 Vision AI TypeScript SDK
 * Version: 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * Official TypeScript SDK for the WIA-AI-021 Vision AI Standard
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import * as Types from './types';

export * from './types';

/**
 * Main VisionAI Client Class
 */
export class VisionAI {
  private client: AxiosInstance;
  private config: Types.VisionAIConfig;

  constructor(config: Types.VisionAIConfig) {
    this.config = {
      baseURL: config.baseURL || 'https://api.wia-ai.org',
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
      apiVersion: config.apiVersion || 'v1',
      ...config,
    };

    this.client = axios.create({
      baseURL: `${this.config.baseURL}/api/${this.config.apiVersion}`,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.config.apiKey}`,
        'X-WIA-SDK': 'typescript-1.0.0',
      },
    });

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error) => this.handleError(error)
    );
  }

  // ============================================================================
  // Phase 1: Classification & Detection
  // ============================================================================

  /**
   * Classify an image
   * @param request Classification request parameters
   * @returns Classification predictions
   */
  async classify(
    request: Types.ClassificationRequest
  ): Promise<Types.ClassificationResponse> {
    const response = await this.client.post('/classify', request);
    return response.data;
  }

  /**
   * Detect objects in an image
   * @param request Detection request parameters
   * @returns Object detections with bounding boxes
   */
  async detect(
    request: Types.DetectionRequest
  ): Promise<Types.DetectionResponse> {
    const response = await this.client.post('/detect', request);
    return response.data;
  }

  // ============================================================================
  // Phase 2: Segmentation & OCR
  // ============================================================================

  /**
   * Perform semantic segmentation on an image
   * @param image Base64 encoded image
   * @param options Segmentation options
   * @returns Segmentation map and class statistics
   */
  async segmentSemantic(
    image: string,
    options?: { model?: string; output_format?: string }
  ): Promise<Types.SemanticSegmentationResponse> {
    const response = await this.client.post('/segment', {
      image,
      ...options,
    });
    return response.data;
  }

  /**
   * Perform instance segmentation on an image
   * @param image Base64 encoded image
   * @returns Instance masks and bounding boxes
   */
  async segmentInstance(
    image: string
  ): Promise<Types.InstanceSegmentationResponse> {
    const response = await this.client.post('/segment/instance', { image });
    return response.data;
  }

  /**
   * Extract text from an image using OCR
   * @param request OCR request parameters
   * @returns Extracted text and word-level details
   */
  async ocr(request: Types.OCRRequest): Promise<Types.OCRResponse> {
    const response = await this.client.post('/ocr', request);
    return response.data;
  }

  /**
   * Estimate human pose in an image
   * @param image Base64 encoded image
   * @param options Pose estimation options
   * @returns Detected poses with keypoints
   */
  async estimatePose(
    image: string,
    options?: { max_persons?: number; min_confidence?: number }
  ): Promise<Types.PoseEstimationResponse> {
    const response = await this.client.post('/pose', {
      image,
      ...options,
    });
    return response.data;
  }

  /**
   * Detect faces in an image
   * @param request Face detection request parameters
   * @returns Detected faces with landmarks and attributes
   */
  async detectFaces(
    request: Types.FaceDetectionRequest
  ): Promise<Types.FaceDetectionResponse> {
    const response = await this.client.post('/face/detect', request);
    return response.data;
  }

  // ============================================================================
  // Phase 3: Video Analysis
  // ============================================================================

  /**
   * Track objects across video frames
   * @param videoId Video identifier
   * @param options Tracking options
   * @returns Object tracks across frames
   */
  async track(
    videoId: string,
    options?: {
      initial_bbox?: Types.BoundingBox;
      algorithm?: string;
    }
  ): Promise<Types.TrackingResponse> {
    const response = await this.client.post('/track', {
      video_id: videoId,
      ...options,
    });
    return response.data;
  }

  /**
   * Recognize actions in a video clip
   * @param videoId Video identifier
   * @param clipStart Start time in milliseconds
   * @param clipEnd End time in milliseconds
   * @param topK Number of top predictions
   * @returns Recognized actions with confidence scores
   */
  async recognizeAction(
    videoId: string,
    clipStart: number,
    clipEnd: number,
    topK: number = 5
  ): Promise<Types.ActionRecognitionResponse> {
    const response = await this.client.post('/recognize_action', {
      video_id: videoId,
      clip_start_ms: clipStart,
      clip_end_ms: clipEnd,
      top_k: topK,
    });
    return response.data;
  }

  /**
   * Compute optical flow between frames
   * @param frame1 First frame (base64 encoded)
   * @param frame2 Second frame (base64 encoded)
   * @returns Optical flow field
   */
  async computeOpticalFlow(
    frame1: string,
    frame2: string
  ): Promise<Types.OpticalFlowResponse> {
    const response = await this.client.post('/optical_flow', {
      frame1,
      frame2,
    });
    return response.data;
  }

  // ============================================================================
  // Phase 4: 3D Vision
  // ============================================================================

  /**
   * Estimate depth from a single image
   * @param image Base64 encoded image
   * @param model Depth estimation model to use
   * @returns Depth map with confidence
   */
  async estimateDepth(
    image: string,
    model: string = 'midas-v3'
  ): Promise<Types.DepthEstimationResponse> {
    const response = await this.client.post('/depth/estimate', {
      image,
      model,
    });
    return response.data;
  }

  /**
   * Detect objects in 3D space
   * @param image Base64 encoded image or point cloud
   * @returns 3D object detections
   */
  async detect3D(image: string): Promise<Types.Detection3DResponse> {
    const response = await this.client.post('/detect_3d', { image });
    return response.data;
  }

  /**
   * Segment a point cloud
   * @param pointCloud Point cloud data
   * @returns Segmented clusters
   */
  async segmentPointCloud(
    pointCloud: string
  ): Promise<Types.PointCloudSegmentationResponse> {
    const response = await this.client.post('/pointcloud/segment', {
      point_cloud: pointCloud,
    });
    return response.data;
  }

  // ============================================================================
  // Batch Processing
  // ============================================================================

  /**
   * Process multiple images in batch
   * @param requests Array of classification requests
   * @returns Batch classification results
   */
  async classifyBatch(
    requests: Types.ClassificationRequest[]
  ): Promise<Types.BatchResponse<Types.ClassificationResponse>> {
    const response = await this.client.post('/batch/classify', {
      items: requests,
    });
    return response.data;
  }

  /**
   * Detect objects in multiple images in batch
   * @param requests Array of detection requests
   * @returns Batch detection results
   */
  async detectBatch(
    requests: Types.DetectionRequest[]
  ): Promise<Types.BatchResponse<Types.DetectionResponse>> {
    const response = await this.client.post('/batch/detect', {
      items: requests,
    });
    return response.data;
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Convert File/Blob to base64 string
   * @param file File or Blob object
   * @returns Base64 encoded string
   */
  static async fileToBase64(file: File | Blob): Promise<string> {
    return new Promise((resolve, reject) => {
      const reader = new FileReader();
      reader.onload = () => {
        const base64 = reader.result as string;
        // Remove data URL prefix (e.g., "data:image/png;base64,")
        const base64Data = base64.split(',')[1] || base64;
        resolve(base64Data);
      };
      reader.onerror = reject;
      reader.readAsDataURL(file);
    });
  }

  /**
   * Load image from URL and convert to base64
   * @param url Image URL
   * @returns Base64 encoded string
   */
  static async urlToBase64(url: string): Promise<string> {
    const response = await fetch(url);
    const blob = await response.blob();
    return VisionAI.fileToBase64(blob);
  }

  /**
   * Calculate IoU (Intersection over Union) between two bounding boxes
   * @param box1 First bounding box
   * @param box2 Second bounding box
   * @returns IoU value (0-1)
   */
  static calculateIoU(
    box1: Types.BoundingBox,
    box2: Types.BoundingBox
  ): number {
    const x1 = Math.max(box1.x_min, box2.x_min);
    const y1 = Math.max(box1.y_min, box2.y_min);
    const x2 = Math.min(box1.x_max, box2.x_max);
    const y2 = Math.min(box1.y_max, box2.y_max);

    const intersection = Math.max(0, x2 - x1) * Math.max(0, y2 - y1);

    const area1 = (box1.x_max - box1.x_min) * (box1.y_max - box1.y_min);
    const area2 = (box2.x_max - box2.x_min) * (box2.y_max - box2.y_min);
    const union = area1 + area2 - intersection;

    return union > 0 ? intersection / union : 0;
  }

  /**
   * Handle API errors
   * @param error Axios error object
   */
  private handleError(error: AxiosError): never {
    if (error.response) {
      // Server responded with error
      const errorData = error.response.data as Types.VisionAIError;
      throw new VisionAIError(
        errorData.error.message,
        errorData.error.code as Types.ErrorCode,
        errorData.error.details
      );
    } else if (error.request) {
      // No response received
      throw new VisionAIError(
        'No response from server',
        'NETWORK_ERROR' as Types.ErrorCode
      );
    } else {
      // Request setup error
      throw new VisionAIError(
        error.message,
        'REQUEST_ERROR' as Types.ErrorCode
      );
    }
  }

  /**
   * Get current API configuration
   */
  getConfig(): Types.VisionAIConfig {
    return { ...this.config };
  }

  /**
   * Update API configuration
   * @param config Partial configuration to update
   */
  updateConfig(config: Partial<Types.VisionAIConfig>): void {
    this.config = { ...this.config, ...config };

    // Update axios instance
    this.client.defaults.baseURL = `${this.config.baseURL}/api/${this.config.apiVersion}`;
    this.client.defaults.timeout = this.config.timeout;

    if (config.apiKey) {
      this.client.defaults.headers.common['Authorization'] =
        `Bearer ${config.apiKey}`;
    }
  }
}

/**
 * Custom Error Class for Vision AI
 */
export class VisionAIError extends Error {
  constructor(
    message: string,
    public code: Types.ErrorCode,
    public details?: string
  ) {
    super(message);
    this.name = 'VisionAIError';
  }
}

/**
 * Create a new VisionAI client instance
 * @param config Client configuration
 * @returns VisionAI client instance
 */
export function createClient(config: Types.VisionAIConfig): VisionAI {
  return new VisionAI(config);
}

// Default export
export default VisionAI;
