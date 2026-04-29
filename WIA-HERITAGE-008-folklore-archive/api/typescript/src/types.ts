/**
 * WIA-HERITAGE-008: Folklore & Oral History Archive Standard
 * TypeScript Type Definitions
 *
 * 弘益人間 (Benefit All Humanity)
 * © 2025 SmileStory Inc. / WIA
 */

/**
 * Cultural artifact metadata following Dublin Core and CIDOC-CRM standards
 */
export interface ArtifactMetadata {
  /** Unique identifier for the artifact */
  id: string;

  /** Artifact name/title */
  name: string;

  /** Description of the artifact */
  description: string;

  /** Historical period (e.g., "Bronze Age", "Medieval", "Renaissance") */
  period: string;

  /** Geographic origin */
  origin: {
    country: string;
    region?: string;
    site?: string;
    coordinates?: {
      latitude: number;
      longitude: number;
    };
  };

  /** Material composition */
  materials: Material[];

  /** Dimensions in millimeters */
  dimensions: {
    length: number;
    width: number;
    height: number;
    weight?: number; // grams
    diameter?: number;
  };

  /** Dating information */
  dating: {
    period: string;
    startYear?: number;
    endYear?: number;
    method?: 'carbon-dating' | 'stratigraphic' | 'stylistic' | 'documentary';
    confidence?: number; // 0-1
  };

  /** Current condition assessment */
  condition: ConditionReport;

  /** Cultural significance */
  significance: {
    cultural: string;
    historical: string;
    artistic: string;
    scientific: string;
  };

  /** Classification */
  classification: {
    category: string; // e.g., "Pottery", "Sculpture", "Manuscript"
    type: string;
    style: string;
    function: string;
  };

  /** Creation timestamp */
  created: Date;

  /** Last update timestamp */
  updated: Date;
}

/**
 * Material information
 */
export interface Material {
  /** Material type */
  type: string;

  /** Material composition */
  composition?: string;

  /** Percentage of total (0-100) */
  percentage?: number;

  /** Analysis method */
  analysisMethod?: 'XRF' | 'FTIR' | 'Raman' | 'SEM' | 'visual';
}

/**
 * Artifact condition report
 */
export interface ConditionReport {
  /** Overall condition rating */
  overall: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';

  /** Detailed condition notes */
  notes: string;

  /** Identified damages */
  damages: Damage[];

  /** Conservation history */
  conservationHistory: ConservationRecord[];

  /** Assessment date */
  assessmentDate: Date;

  /** Assessor information */
  assessor: {
    name: string;
    institution: string;
    credentials: string;
  };
}

/**
 * Damage record
 */
export interface Damage {
  /** Damage type */
  type: 'crack' | 'chip' | 'break' | 'discoloration' | 'corrosion' | 'wear' | 'loss';

  /** Severity (0-1) */
  severity: number;

  /** Location description */
  location: string;

  /** Extent description */
  extent: string;

  /** Cause if known */
  cause?: string;
}

/**
 * Conservation record
 */
export interface ConservationRecord {
  /** Date of conservation */
  date: Date;

  /** Conservator information */
  conservator: {
    name: string;
    institution: string;
  };

  /** Treatment performed */
  treatment: string;

  /** Materials used */
  materialsUsed: string[];

  /** Before/after images */
  images?: {
    before: string[];
    after: string[];
  };
}

/**
 * 3D model data
 */
export interface Model3D {
  /** Model ID */
  id: string;

  /** Artifact ID this model represents */
  artifactId: string;

  /** Model format */
  format: 'gltf' | 'obj' | 'ply' | 'fbx' | 'usdz';

  /** File URL */
  url: string;

  /** Thumbnail URL */
  thumbnail?: string;

  /** Model statistics */
  stats: {
    vertices: number;
    faces: number;
    textures: number;
    fileSize: number; // bytes
  };

  /** Capture method */
  captureMethod: 'photogrammetry' | 'laser-scan' | 'structured-light' | 'manual-modeling';

  /** Capture metadata */
  captureMetadata: {
    date: Date;
    equipment: string;
    software: string;
    resolution: number; // millimeters
    accuracy: number; // millimeters
    captureTime: number; // minutes
    photoCount?: number;
  };

  /** Textures */
  textures: Texture[];

  /** Created timestamp */
  created: Date;
}

/**
 * Texture information
 */
export interface Texture {
  /** Texture type */
  type: 'diffuse' | 'normal' | 'roughness' | 'metallic' | 'ao' | 'emissive';

  /** File URL */
  url: string;

  /** Resolution */
  resolution: {
    width: number;
    height: number;
  };

  /** File format */
  format: 'png' | 'jpg' | 'tiff' | 'exr';

  /** File size in bytes */
  size: number;
}

/**
 * Provenance record using blockchain
 */
export interface ProvenanceRecord {
  /** Record ID */
  id: string;

  /** Artifact ID */
  artifactId: string;

  /** Blockchain transaction hash */
  txHash: string;

  /** Blockchain network */
  blockchain: 'ethereum' | 'polygon' | 'arweave' | 'filecoin';

  /** Owner information */
  owner: {
    name: string;
    institution?: string;
    wallet?: string;
  };

  /** Location */
  location: {
    institution: string;
    address: string;
    country: string;
  };

  /** Transfer information */
  transfer?: {
    from: string;
    to: string;
    date: Date;
    method: 'purchase' | 'donation' | 'loan' | 'repatriation' | 'excavation';
    documentation: string[];
  };

  /** Timestamp */
  timestamp: Date;

  /** Previous record hash (for chain integrity) */
  previousHash?: string;
}

/**
 * Virtual exhibition configuration
 */
export interface VirtualExhibition {
  /** Exhibition ID */
  id: string;

  /** Exhibition title */
  title: string;

  /** Description */
  description: string;

  /** Theme/topic */
  theme: string;

  /** Curator information */
  curator: {
    name: string;
    institution: string;
    bio: string;
  };

  /** Artifacts included */
  artifacts: string[]; // artifact IDs

  /** Exhibition layout */
  layout: ExhibitionLayout;

  /** Languages available */
  languages: string[];

  /** Access settings */
  access: {
    public: boolean;
    requiresRegistration: boolean;
    allowsDownload: boolean;
  };

  /** Created timestamp */
  created: Date;

  /** Exhibition URL */
  url: string;
}

/**
 * Exhibition layout configuration
 */
export interface ExhibitionLayout {
  /** Layout type */
  type: 'gallery' | 'timeline' | 'map' | 'thematic' | 'custom';

  /** Room/section definitions */
  sections: ExhibitionSection[];

  /** Navigation settings */
  navigation: {
    enableVR: boolean;
    enableAR: boolean;
    enableGuided: boolean;
  };
}

/**
 * Exhibition section
 */
export interface ExhibitionSection {
  /** Section ID */
  id: string;

  /** Section title */
  title: string;

  /** Description */
  description: string;

  /** Artifacts in this section */
  artifacts: string[];

  /** Multimedia content */
  media: {
    images?: string[];
    videos?: string[];
    audio?: string[];
    text?: string;
  };

  /** Position in exhibition */
  order: number;
}

/**
 * Scan request parameters
 */
export interface ScanRequest {
  /** Input images */
  images: string[];

  /** Artifact metadata */
  metadata: ArtifactMetadata;

  /** Scan options */
  options?: {
    resolution?: 'low' | 'medium' | 'high' | 'ultra';
    includeTextures?: boolean;
    generateNormals?: boolean;
    optimize?: boolean;
  };
}

/**
 * Scan result
 */
export interface ScanResult {
  /** Result ID */
  id: string;

  /** Status */
  status: 'processing' | 'completed' | 'failed';

  /** Progress (0-1) */
  progress: number;

  /** Generated 3D model */
  model?: Model3D;

  /** Processing time in seconds */
  processingTime?: number;

  /** Error message if failed */
  error?: string;
}

/**
 * API configuration
 */
export interface WIAHeritage008Config {
  /** API key for authentication */
  apiKey: string;

  /** API endpoint URL */
  endpoint?: string;

  /** Blockchain network for provenance */
  blockchain?: 'ethereum' | 'polygon' | 'arweave' | 'filecoin';

  /** Default language */
  language?: string;

  /** Timeout in milliseconds */
  timeout?: number;
}

/**
 * API response wrapper
 */
export interface APIResponse<T> {
  /** Success status */
  success: boolean;

  /** Response data */
  data?: T;

  /** Error message */
  error?: string;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Search query parameters
 */
export interface SearchQuery {
  /** Search text */
  query?: string;

  /** Filters */
  filters?: {
    period?: string[];
    origin?: string[];
    material?: string[];
    category?: string[];
  };

  /** Sorting */
  sort?: {
    field: string;
    order: 'asc' | 'desc';
  };

  /** Pagination */
  pagination?: {
    page: number;
    limit: number;
  };
}

/**
 * Search results
 */
export interface SearchResults {
  /** Total results count */
  total: number;

  /** Current page */
  page: number;

  /** Results per page */
  limit: number;

  /** Artifact results */
  artifacts: ArtifactMetadata[];
}
