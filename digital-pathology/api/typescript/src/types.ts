/**
 * WIA-MED-008: Digital Pathology Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 timestamp string
 */
export type Timestamp = string;

/**
 * Unique identifier for slides
 */
export type SlideID = string;

/**
 * Unique identifier for specimens
 */
export type SpecimenID = string;

/**
 * Unique identifier for analysis results
 */
export type AnalysisID = string;

// ============================================================================
// Enums
// ============================================================================

/**
 * Supported stain types in digital pathology
 */
export enum StainType {
  HE = 'H&E',                          // Hematoxylin and Eosin
  IHC = 'IHC',                         // Immunohistochemistry
  FLUORESCENCE = 'fluorescence',       // Fluorescence
  MULTISPECTRAL = 'multispectral',     // Multispectral
  SPECIAL_STAIN = 'special_stain'      // Special stains (PAS, Masson, etc.)
}

/**
 * Compression formats
 */
export enum CompressionType {
  JPEG = 'JPEG',
  JPEG2000_LOSSY = 'JPEG2000_lossy',
  JPEG2000_LOSSLESS = 'JPEG2000_lossless',
  WEBP = 'WebP',
  DEFLATE = 'Deflate',
  LZW = 'LZW'
}

/**
 * Magnification levels
 */
export enum Magnification {
  MAG_1_25X = 1.25,
  MAG_2_5X = 2.5,
  MAG_5X = 5,
  MAG_10X = 10,
  MAG_20X = 20,
  MAG_40X = 40
}

/**
 * Analysis types
 */
export enum AnalysisType {
  CELL_DETECTION = 'cell_detection',
  TISSUE_CLASSIFICATION = 'tissue_classification',
  TUMOR_SEGMENTATION = 'tumor_segmentation',
  MITOSIS_COUNTING = 'mitosis_counting',
  KI67_INDEX = 'ki67_index',
  PDL1_SCORING = 'pdl1_scoring',
  HER2_SCORING = 'her2_scoring'
}

/**
 * Slide status
 */
export enum SlideStatus {
  PENDING = 'pending',
  SCANNING = 'scanning',
  READY = 'ready',
  ANALYZING = 'analyzing',
  COMPLETE = 'complete',
  ERROR = 'error'
}

/**
 * Organ types
 */
export enum OrganType {
  LUNG = 'lung',
  LIVER = 'liver',
  KIDNEY = 'kidney',
  BRAIN = 'brain',
  BREAST = 'breast',
  COLON = 'colon',
  PROSTATE = 'prostate',
  SKIN = 'skin',
  LYMPH_NODE = 'lymph_node',
  STOMACH = 'stomach',
  OTHER = 'other'
}

// ============================================================================
// Specimen Information
// ============================================================================

export interface SpecimenInfo {
  /** Unique accession number (e.g., S25-00123) */
  accession_number: string;

  /** Specimen ID */
  specimen_id: SpecimenID;

  /** Organ type */
  organ: OrganType;

  /** Specimen type (e.g., biopsy, resection) */
  specimen_type: string;

  /** Collection date */
  collection_date?: Timestamp;

  /** Clinical diagnosis */
  diagnosis?: string;

  /** Patient ID (de-identified) */
  patient_id?: string;
}

// ============================================================================
// Scan Information
// ============================================================================

export interface ScanInfo {
  /** Scanner manufacturer and model */
  scanner_model: string;

  /** Scan timestamp */
  scan_date: Timestamp;

  /** Magnification level */
  magnification: Magnification;

  /** Resolution in micrometers per pixel */
  resolution_um_per_pixel: number;

  /** Stain type */
  stain_type: StainType;

  /** Scanner serial number */
  scanner_serial?: string;

  /** Scan duration in seconds */
  scan_duration_seconds?: number;
}

// ============================================================================
// Image Information
// ============================================================================

export interface ImageInfo {
  /** Image width in pixels */
  width_pixels: number;

  /** Image height in pixels */
  height_pixels: number;

  /** Number of pyramid levels */
  pyramid_levels: number;

  /** Tile size (typically 256 or 512) */
  tile_size: number;

  /** Compression type */
  compression: CompressionType;

  /** Color space (typically sRGB) */
  color_space: string;

  /** Bits per pixel */
  bits_per_pixel?: number;

  /** File size in bytes */
  file_size_bytes?: number;
}

// ============================================================================
// Quality Metrics
// ============================================================================

export interface QualityMetrics {
  /** Focus quality score (0.0 to 1.0) */
  focus_quality_score: number;

  /** Whether slide passed QC */
  passed_qc: boolean;

  /** Color accuracy (Delta E) */
  color_accuracy_delta_e?: number;

  /** Signal-to-noise ratio (dB) */
  snr_db?: number;

  /** MTF50 (line pairs per mm) */
  mtf50_lp_per_mm?: number;

  /** Overall quality grade (A-E) */
  quality_grade?: 'A' | 'B' | 'C' | 'D' | 'E';
}

// ============================================================================
// WSI Metadata
// ============================================================================

export interface WSIMetadata {
  /** WIA standard version */
  wia_version: string;

  /** Specimen information */
  specimen: SpecimenInfo;

  /** Scan information */
  scan: ScanInfo;

  /** Image information */
  image: ImageInfo;

  /** Quality metrics */
  quality: QualityMetrics;

  /** Processing information */
  processing?: {
    fixation?: string;
    embedding?: string;
    sectioning_thickness_um?: number;
  };

  /** Annotations (GeoJSON format) */
  annotations?: any[];

  /** Custom metadata */
  custom?: Record<string, any>;

  /** Creation timestamp */
  timestamp: Timestamp;
}

// ============================================================================
// Whole Slide Image
// ============================================================================

export interface WholeSlideImage {
  /** Unique slide ID */
  slide_id: SlideID;

  /** File path or URL */
  file_path: string;

  /** File format (e.g., .wdps, .tif) */
  file_format: string;

  /** Complete metadata */
  metadata: WSIMetadata;

  /** Current status */
  status: SlideStatus;

  /** Creation timestamp */
  created_at: Timestamp;

  /** Last updated timestamp */
  updated_at: Timestamp;
}

// ============================================================================
// Tile Request
// ============================================================================

export interface TileRequest {
  /** Slide ID */
  slide_id: SlideID;

  /** Pyramid level (0 = highest resolution) */
  level: number;

  /** X coordinate in pixels */
  x: number;

  /** Y coordinate in pixels */
  y: number;

  /** Tile width (default: 256) */
  width?: number;

  /** Tile height (default: 256) */
  height?: number;
}

// ============================================================================
// AI Analysis Request
// ============================================================================

export interface AnalysisRequest {
  /** Slide ID to analyze */
  slide_id: SlideID;

  /** Type of analysis */
  analysis_type: AnalysisType;

  /** AI model to use */
  model?: string;

  /** Analysis parameters */
  parameters?: {
    /** Confidence threshold (0.0 to 1.0) */
    confidence_threshold?: number;

    /** Region of interest */
    roi?: {
      x: number;
      y: number;
      width: number;
      height: number;
    };

    /** Additional parameters */
    [key: string]: any;
  };
}

// ============================================================================
// AI Analysis Result
// ============================================================================

export interface AnalysisResult {
  /** Unique analysis ID */
  analysis_id: AnalysisID;

  /** Slide ID */
  slide_id: SlideID;

  /** Analysis type */
  analysis_type: AnalysisType;

  /** AI model used */
  model: string;

  /** Analysis results */
  results: {
    /** Cell count (for cell detection) */
    cell_count?: number;

    /** Tumor percentage (for tumor segmentation) */
    tumor_percentage?: number;

    /** Mitotic count (for mitosis counting) */
    mitotic_count?: number;

    /** Ki-67 index (for Ki-67 analysis) */
    ki67_index?: number;

    /** PD-L1 TPS (for PD-L1 scoring) */
    pdl1_tps?: number;

    /** Grade or classification */
    grade?: string;

    /** Detected objects or regions */
    detections?: Array<{
      class: string;
      confidence: number;
      bbox: [number, number, number, number]; // [x, y, w, h]
    }>;

    /** Additional results */
    [key: string]: any;
  };

  /** Processing time in seconds */
  processing_time_seconds: number;

  /** Analysis timestamp */
  timestamp: Timestamp;

  /** Status */
  status: 'pending' | 'processing' | 'complete' | 'error';

  /** Error message (if any) */
  error?: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface APIResponse<T = any> {
  /** Status code */
  status: number;

  /** Success flag */
  success: boolean;

  /** Response message */
  message: string;

  /** Response data */
  data?: T;

  /** Error details (if any) */
  error?: {
    code: string;
    message: string;
    details?: any;
  };

  /** Response timestamp */
  timestamp: Timestamp;
}

export interface PaginatedResponse<T> {
  /** Items */
  items: T[];

  /** Total count */
  total: number;

  /** Current page */
  page: number;

  /** Page size */
  page_size: number;

  /** Total pages */
  total_pages: number;

  /** Has next page */
  has_next: boolean;

  /** Has previous page */
  has_previous: boolean;
}

// ============================================================================
// Upload Types
// ============================================================================

export interface UploadRequest {
  /** File name */
  file_name: string;

  /** File size in bytes */
  file_size_bytes: number;

  /** Specimen metadata */
  specimen: SpecimenInfo;

  /** Scan metadata */
  scan: Partial<ScanInfo>;
}

export interface UploadResponse {
  /** Upload ID */
  upload_id: string;

  /** Slide ID (assigned after upload) */
  slide_id: SlideID;

  /** Upload URL (for direct upload) */
  upload_url?: string;

  /** Status */
  status: 'pending' | 'uploading' | 'processing' | 'complete' | 'error';
}

// ============================================================================
// Integration Types
// ============================================================================

export interface LISIntegration {
  /** LIS system name */
  lis_system: string;

  /** Accession number */
  accession_number: string;

  /** Order ID */
  order_id?: string;

  /** Synchronization status */
  sync_status: 'pending' | 'synced' | 'error';
}

export interface PACSIntegration {
  /** PACS system name */
  pacs_system: string;

  /** Study instance UID (DICOM) */
  study_instance_uid?: string;

  /** Series instance UID (DICOM) */
  series_instance_uid?: string;
}

// ============================================================================
// Validation Types
// ============================================================================

export interface ValidationResult {
  /** Overall validation status */
  is_valid: boolean;

  /** Validation tests */
  tests: Array<{
    name: string;
    result: 'PASS' | 'FAIL' | 'WARNING';
    score?: number;
    message?: string;
  }>;

  /** Overall compliance status */
  compliance: 'COMPLIANT' | 'NON_COMPLIANT' | 'PARTIAL';

  /** Certification status */
  certification?: {
    certified: boolean;
    certificate_id?: string;
    valid_until?: Timestamp;
  };
}
