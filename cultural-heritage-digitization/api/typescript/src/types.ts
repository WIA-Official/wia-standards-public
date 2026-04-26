/**
 * WIA-EDU-023 Cultural Heritage Digitization Standard - TypeScript Type Definitions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * WIA Cultural Heritage Standard Version
 */
export type WIAVersion = '1.0';

/**
 * Scanning technology types
 */
export type ScanTechnology =
  | 'photogrammetry'
  | 'lidar'
  | 'structured_light'
  | 'ct_scan'
  | 'manual_modeling';

/**
 * Virtual experience types
 */
export type ExperienceType =
  | 'vr'
  | 'ar'
  | '360_panorama'
  | '3d_viewer'
  | 'guided_video';

/**
 * WCAG Accessibility Level
 */
export type WCAGLevel = 'A' | 'AA' | 'AAA';

/**
 * Cultural artifact metadata following WIA-EDU-023 Phase 1
 */
export interface ArtifactMetadata {
  /** Unique identifier */
  id: string;
  /** WIA Registry ID */
  wiaId?: string;
  /** Artifact title */
  title: string;
  /** Alternative titles */
  alternativeTitles?: string[];
  /** Culture or civilization */
  culture: string;
  /** Historical period */
  period: string;
  /** Creation date (ISO 8601 or description) */
  date?: string;
  /** Materials used */
  materials: string[];
  /** Physical dimensions */
  dimensions: Dimensions;
  /** Current location */
  location: Location;
  /** Origin/provenance */
  origin?: string;
  /** Description */
  description: string;
  /** Historical significance */
  significance?: string;
  /** Conservation status */
  conservationStatus: ConservationStatus;
  /** Categories/classifications */
  categories: string[];
  /** Related artifacts */
  relatedArtifacts?: string[];
}

/**
 * Dimensions structure
 */
export interface Dimensions {
  height?: number;
  width?: number;
  depth?: number;
  diameter?: number;
  weight?: number;
  unit: 'cm' | 'mm' | 'm' | 'in' | 'ft';
}

/**
 * Location information
 */
export interface Location {
  /** Current institution/museum */
  current: string;
  /** Geographic coordinates */
  coordinates?: {
    latitude: number;
    longitude: number;
  };
  /** Original discovery/creation location */
  origin?: string;
}

/**
 * Conservation status
 */
export type ConservationStatus =
  | 'excellent'
  | 'good'
  | 'fair'
  | 'poor'
  | 'fragment'
  | 'restored';

/**
 * 3D Model information
 */
export interface Model3D {
  /** Model identifier */
  id: string;
  /** Artifact this model represents */
  artifactId: string;
  /** File format */
  format: ModelFormat;
  /** File URL or path */
  url: string;
  /** File size in bytes */
  fileSize: number;
  /** Quality level */
  quality: 'master' | 'high' | 'medium' | 'low' | 'thumbnail';
  /** Resolution metrics */
  resolution: {
    vertices: number;
    faces: number;
    textureResolution?: string; // e.g., "4096x4096"
  };
  /** Capture metadata */
  captureMetadata: CaptureMetadata;
  /** Processing metadata */
  processingMetadata: ProcessingMetadata;
}

/**
 * Supported 3D model formats
 */
export type ModelFormat =
  | 'glTF'
  | 'GLB'
  | 'USDZ'
  | 'OBJ'
  | 'FBX'
  | 'PLY'
  | 'STL'
  | 'E57'
  | 'X3D';

/**
 * Capture metadata
 */
export interface CaptureMetadata {
  /** Capture date */
  date: string; // ISO 8601
  /** Technology used */
  technology: ScanTechnology;
  /** Equipment details */
  equipment: {
    make?: string;
    model?: string;
    lens?: string;
    settings?: Record<string, any>;
  };
  /** Operator/technician */
  operator: string;
  /** Environmental conditions */
  environment?: {
    lighting?: string;
    temperature?: number;
    humidity?: number;
  };
  /** Quality metrics */
  quality: {
    geometricAccuracy: string; // e.g., "±0.05mm"
    colorAccuracy?: string; // e.g., "ΔE < 2.0"
    completeness?: string; // e.g., "98.5%"
  };
}

/**
 * Processing metadata
 */
export interface ProcessingMetadata {
  /** Processing date */
  date: string; // ISO 8601
  /** Software used */
  software: {
    name: string;
    version: string;
  }[];
  /** Processing steps */
  steps: string[];
  /** Processor/technician */
  processor: string;
  /** File checksums */
  checksums?: {
    md5?: string;
    sha256?: string;
  };
}

/**
 * Dublin Core metadata
 */
export interface DublinCoreMetadata {
  title: string;
  creator: string;
  subject: string[];
  description: string;
  publisher: string;
  contributor?: string[];
  date: string;
  type: string;
  format: string;
  identifier: string;
  source?: string;
  language: string[];
  relation?: string[];
  coverage: {
    spatial?: string;
    temporal?: string;
  };
  rights: string;
}

/**
 * Virtual heritage tour
 */
export interface VirtualTour {
  /** Tour identifier */
  id: string;
  /** Tour name */
  name: string;
  /** Tour type */
  type: ExperienceType;
  /** Description */
  description: string;
  /** Creation date */
  created: string; // ISO 8601
  /** Tour duration estimate */
  duration?: string;
  /** Tour stops/points of interest */
  stops: TourStop[];
  /** Navigation settings */
  navigation: NavigationSettings;
  /** Multimedia assets */
  multimedia: MultimediaAssets;
  /** Accessibility features */
  accessibility: AccessibilityFeatures;
}

/**
 * Tour stop/point of interest
 */
export interface TourStop {
  /** Stop identifier */
  id: string;
  /** Stop name */
  name: string;
  /** Position in 3D space */
  position: {x: number; y: number; z: number};
  /** Description */
  description?: string;
  /** Related artifact IDs */
  artifacts?: string[];
  /** Media assets for this stop */
  media: {
    panorama?: string;
    model3d?: string;
    audio?: Record<string, string>; // language -> URL
    video?: string;
  };
  /** Information hotspots */
  hotspots?: Hotspot[];
}

/**
 * Information hotspot
 */
export interface Hotspot {
  id: string;
  position: {x: number; y: number; z: number};
  title: string;
  content: string;
  mediaUrl?: string;
}

/**
 * Navigation settings
 */
export interface NavigationSettings {
  mode: 'free-roam' | 'guided' | 'teleport-only';
  controls: string[];
  minimap: boolean;
  compass: boolean;
}

/**
 * Multimedia assets
 */
export interface MultimediaAssets {
  audioGuides: string[]; // Language codes
  videoClips: number;
  interactiveElements: number;
  totalAssets: number;
}

/**
 * Accessibility features
 */
export interface AccessibilityFeatures {
  wcagCompliance: WCAGLevel;
  features: AccessibilityFeature[];
  subtitles: boolean;
  audioDescription: boolean;
  signLanguage: boolean;
}

/**
 * Specific accessibility features
 */
export type AccessibilityFeature =
  | 'subtitles'
  | 'audio_description'
  | 'sign_language'
  | 'high_contrast'
  | 'screen_reader'
  | 'keyboard_nav'
  | 'voice_control';

/**
 * Historical reconstruction
 */
export interface HistoricalReconstruction {
  /** Reconstruction identifier */
  id: string;
  /** Subject artifact/monument */
  subjectId: string;
  /** Reconstruction method */
  method: ReconstructionMethod;
  /** Confidence level (0-100) */
  confidenceLevel: number;
  /** Evidence sources */
  sources: EvidenceSource[];
  /** Uncertainty map URL */
  uncertaintyMapUrl?: string;
  /** Validation status */
  validated: boolean;
  /** Expert reviewers */
  reviewers?: string[];
  /** Created date */
  created: string; // ISO 8601
}

/**
 * Reconstruction methods
 */
export type ReconstructionMethod =
  | 'ai_ml'
  | 'photogrammetric_analysis'
  | 'comparative_study'
  | 'manual_modeling'
  | 'hybrid';

/**
 * Evidence source
 */
export interface EvidenceSource {
  type: 'photograph' | 'drawing' | 'text' | 'comparative_artifact' | 'archaeological_record';
  description: string;
  url?: string;
  reliability: 'high' | 'medium' | 'low';
}

/**
 * API Client configuration
 */
export interface ClientConfig {
  baseURL: string;
  accessToken: string;
  timeout?: number;
  enableCaching?: boolean;
}

/**
 * API Response wrapper
 */
export interface APIResponse<T> {
  data: T;
  status: number;
  message?: string;
  errors?: string[];
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  limit?: number;
  offset?: number;
  sortBy?: string;
  order?: 'asc' | 'desc';
}

/**
 * Search parameters
 */
export interface SearchParams extends PaginationParams {
  query?: string;
  culture?: string;
  period?: string;
  material?: string;
  category?: string;
  location?: string;
}
