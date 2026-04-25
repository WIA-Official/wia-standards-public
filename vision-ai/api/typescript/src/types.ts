/**
 * WIA-AI-021 Vision AI TypeScript SDK - Type Definitions
 * Version: 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Common Types
// ============================================================================

export interface BoundingBox {
  x_min: number;
  y_min: number;
  x_max: number;
  y_max: number;
  format?: 'xyxy' | 'xywh';
}

export interface Point2D {
  x: number;
  y: number;
}

export interface Point3D {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  qw: number;
  qx: number;
  qy: number;
  qz: number;
}

export interface Confidence {
  value: number;
  level?: 'low' | 'medium' | 'high';
}

// ============================================================================
// Configuration
// ============================================================================

export interface VisionAIConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  retries?: number;
  apiVersion?: 'v1' | 'v2' | 'v3' | 'v4';
}

// ============================================================================
// Phase 1: Classification & Detection
// ============================================================================

export interface ClassificationPrediction {
  class_id: number;
  class_name: string;
  confidence: number;
  bounding_box?: BoundingBox | null;
}

export interface ClassificationResponse {
  predictions: ClassificationPrediction[];
  processing_time_ms: number;
  model_version: string;
  timestamp: string;
}

export interface ClassificationRequest {
  image: string; // base64 encoded
  top_k?: number;
  min_confidence?: number;
}

export interface Detection {
  class_id: number;
  class_name: string;
  confidence: number;
  bounding_box: BoundingBox;
}

export interface DetectionResponse {
  detections: Detection[];
  processing_time_ms: number;
  model_version: string;
  image_size: [number, number];
  timestamp: string;
}

export interface DetectionRequest {
  image: string; // base64 encoded
  confidence_threshold?: number;
  iou_threshold?: number;
  max_detections?: number;
}

// ============================================================================
// Phase 2: Segmentation & OCR
// ============================================================================

export interface SegmentClass {
  id: number;
  name: string;
  pixel_count: number;
  percentage: number;
}

export interface SemanticSegmentationResponse {
  segmentation_map: string; // base64 encoded
  classes: SegmentClass[];
  image_size: [number, number];
  processing_time_ms: number;
}

export interface InstanceSegmentation {
  instance_id: number;
  class_id: number;
  class_name: string;
  confidence: number;
  bounding_box: BoundingBox;
  mask: string; // base64 encoded
  area: number;
}

export interface InstanceSegmentationResponse {
  instances: InstanceSegmentation[];
  processing_time_ms: number;
}

export interface TextRegion {
  region_id: number;
  polygon: Point2D[];
  confidence: number;
  text: string;
}

export interface Word {
  text: string;
  confidence: number;
  bbox: [number, number, number, number];
}

export interface OCRResponse {
  text: string;
  confidence: number;
  words: Word[];
  text_regions?: TextRegion[];
}

export interface OCRRequest {
  image: string; // base64 encoded
  languages?: string[];
  detect_orientation?: boolean;
}

export interface Keypoint {
  id: number;
  name: string;
  x: number;
  y: number;
  confidence: number;
  visible: boolean;
}

export interface Pose {
  person_id: number;
  keypoints: Keypoint[];
  bounding_box: BoundingBox;
  overall_confidence: number;
}

export interface PoseEstimationResponse {
  poses: Pose[];
}

export interface FaceLandmarks {
  left_eye: Point2D;
  right_eye: Point2D;
  nose: Point2D;
  left_mouth: Point2D;
  right_mouth: Point2D;
}

export interface Face {
  face_id: number;
  bounding_box: BoundingBox;
  confidence: number;
  landmarks?: FaceLandmarks;
  attributes?: {
    age?: number;
    gender?: string;
    emotion?: string;
  };
}

export interface FaceDetectionResponse {
  faces: Face[];
  anonymization?: {
    method: string;
    anonymized_image?: string;
  };
}

export interface FaceDetectionRequest {
  image: string; // base64 encoded
  detect_landmarks?: boolean;
  anonymize?: boolean;
}

// ============================================================================
// Phase 3: Video Analysis
// ============================================================================

export interface FrameDetection {
  frame_number: number;
  timestamp_ms: number;
  bounding_box: BoundingBox;
  confidence: number;
}

export interface Track {
  track_id: number;
  class: string;
  first_frame: number;
  last_frame: number;
  trajectory: FrameDetection[];
}

export interface TrackingResponse {
  tracks: Track[];
  metrics?: {
    mota?: number;
    motp?: number;
    idf1?: number;
  };
}

export interface Action {
  action: string;
  confidence: number;
  start_time_ms: number;
  end_time_ms: number;
  clip_info: {
    start_frame: number;
    end_frame: number;
    duration_frames: number;
  };
}

export interface ActionRecognitionResponse {
  actions: Action[];
}

export interface OpticalFlowResponse {
  flow_field: {
    width: number;
    height: number;
    flow_x: string; // base64 encoded
    flow_y: string; // base64 encoded
    magnitude: string; // base64 encoded
    angle: string; // base64 encoded
  };
  average_motion: {
    magnitude: number;
    direction_degrees: number;
  };
}

// ============================================================================
// Phase 4: 3D Vision
// ============================================================================

export interface DepthMap {
  format: 'disparity' | 'depth';
  width: number;
  height: number;
  data: string; // base64 encoded
  min_depth_m: number;
  max_depth_m: number;
  unit: 'meters';
}

export interface DepthEstimationResponse {
  depth_map: DepthMap;
  confidence_map?: string; // base64 encoded
  model: string;
  processing_time_ms: number;
}

export interface BoundingBox3D {
  center: Point3D;
  dimensions: {
    length: number;
    width: number;
    height: number;
  };
  rotation: {
    yaw: number;
    pitch: number;
    roll: number;
  };
}

export interface Detection3D {
  object_id: number;
  class: string;
  confidence: number;
  bbox_3d: BoundingBox3D;
  distance_m: number;
}

export interface Detection3DResponse {
  detections_3d: Detection3D[];
}

export interface PointCloudCluster {
  cluster_id: number;
  class: string;
  num_points: number;
  centroid: Point3D;
}

export interface PointCloudSegmentationResponse {
  segmentation: {
    num_points: number;
    num_clusters: number;
    clusters: PointCloudCluster[];
  };
}

// ============================================================================
// Error Handling
// ============================================================================

export interface VisionAIError {
  status: 'error';
  error: {
    code: string;
    message: string;
    details?: string;
  };
  timestamp: string;
}

export type ErrorCode =
  | 'INVALID_IMAGE'
  | 'IMAGE_TOO_LARGE'
  | 'INVALID_PARAMETERS'
  | 'MODEL_ERROR'
  | 'RATE_LIMIT_EXCEEDED'
  | 'AUTHENTICATION_FAILED';

// ============================================================================
// Request Options
// ============================================================================

export interface RequestOptions {
  timeout?: number;
  headers?: Record<string, string>;
  retries?: number;
}

// ============================================================================
// Utility Types
// ============================================================================

export type ImageInput = string | Buffer | File | Blob;

export interface ProcessingMetrics {
  processing_time_ms: number;
  model_version: string;
  timestamp: string;
}

// ============================================================================
// Model Configuration
// ============================================================================

export interface ModelConfig {
  name: string;
  version: string;
  quantization?: 'fp32' | 'fp16' | 'int8';
  device?: 'cpu' | 'cuda' | 'auto';
}

// ============================================================================
// Batch Processing
// ============================================================================

export interface BatchRequest<T> {
  items: T[];
  batch_size?: number;
  parallel?: boolean;
}

export interface BatchResponse<T> {
  results: T[];
  failed_indices?: number[];
  total_processing_time_ms: number;
}

// ============================================================================
// Streaming
// ============================================================================

export interface StreamConfig {
  stream_url: string;
  tasks: ('detection' | 'tracking' | 'action_recognition')[];
  callback_url?: string;
  frame_skip?: number;
}

// ============================================================================
// Exports
// ============================================================================

export type {
  VisionAIConfig,
  ClassificationPrediction,
  ClassificationResponse,
  ClassificationRequest,
  Detection,
  DetectionResponse,
  DetectionRequest,
  SegmentClass,
  SemanticSegmentationResponse,
  InstanceSegmentation,
  InstanceSegmentationResponse,
  TextRegion,
  Word,
  OCRResponse,
  OCRRequest,
  Keypoint,
  Pose,
  PoseEstimationResponse,
  FaceLandmarks,
  Face,
  FaceDetectionResponse,
  FaceDetectionRequest,
  FrameDetection,
  Track,
  TrackingResponse,
  Action,
  ActionRecognitionResponse,
  OpticalFlowResponse,
  DepthMap,
  DepthEstimationResponse,
  BoundingBox3D,
  Detection3D,
  Detection3DResponse,
  PointCloudCluster,
  PointCloudSegmentationResponse,
  VisionAIError,
  ErrorCode,
  RequestOptions,
  ImageInput,
  ProcessingMetrics,
  ModelConfig,
  BatchRequest,
  BatchResponse,
  StreamConfig,
};
