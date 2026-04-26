/**
 * WIA-MED-015: Medical Imaging Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Type Aliases
// ============================================================================

export type Timestamp = string;
export type StudyID = string;
export type SeriesID = string;
export type ImageID = string;
export type PatientID = string;

// ============================================================================
// Enums
// ============================================================================

export enum Modality {
  XRAY = 'XRAY',
  CT = 'CT',
  MRI = 'MRI',
  ULTRASOUND = 'US',
  PET = 'PET',
  MAMMOGRAPHY = 'MAMMO',
  NUCLEAR_MEDICINE = 'NM',
  DIGITAL_XRAY = 'DX',
  COMPUTED_RADIOGRAPHY = 'CR',
  FLUOROSCOPY = 'FL',
}

export enum StudyStatus {
  SCHEDULED = 'scheduled',
  IN_PROGRESS = 'in_progress',
  COMPLETED = 'completed',
  CANCELLED = 'cancelled',
  PENDING_REVIEW = 'pending_review',
  REVIEWED = 'reviewed',
}

export enum ImageFormat {
  DICOM = 'DICOM',
  NIFTI = 'NIFTI',
  PNG = 'PNG',
  JPEG = 'JPEG',
  TIFF = 'TIFF',
}

export enum AnnotationType {
  MEASUREMENT = 'measurement',
  ROI = 'roi',
  TEXT = 'text',
  ARROW = 'arrow',
  FREEHAND = 'freehand',
  ELLIPSE = 'ellipse',
  RECTANGLE = 'rectangle',
}

// ============================================================================
// DICOM Types
// ============================================================================

export interface DICOMTags {
  studyInstanceUID: string;
  seriesInstanceUID: string;
  sopInstanceUID: string;
  patientName?: string;
  patientId?: string;
  studyDate?: string;
  modality?: Modality;
  bodyPartExamined?: string;
  institutionName?: string;
  referringPhysician?: string;
  accessionNumber?: string;
}

export interface ImageMetadata {
  rows: number;
  columns: number;
  bitsAllocated: number;
  bitsStored?: number;
  pixelSpacing?: [number, number];
  sliceThickness?: number;
  windowCenter?: number;
  windowWidth?: number;
  rescaleIntercept?: number;
  rescaleSlope?: number;
  photometricInterpretation?: string;
}

// ============================================================================
// Image Types
// ============================================================================

export interface MedicalImage {
  format: 'WIA-MEDICAL-IMAGING-v1.0';
  imageId: ImageID;
  studyId: StudyID;
  seriesId: SeriesID;
  patientId: PatientID;
  modality: Modality;
  bodyPart: string;
  dicomTags: DICOMTags;
  imageMetadata: ImageMetadata;
  fileInfo: FileInfo;
  acquisitionDate: Timestamp;
  createdAt: Timestamp;
}

export interface FileInfo {
  sizeBytes: number;
  format: ImageFormat;
  compression?: string;
  checksum?: string;
  storageLocation: string;
}

// ============================================================================
// Study Types
// ============================================================================

export interface ImagingStudy {
  studyId: StudyID;
  patientId: PatientID;
  accessionNumber?: string;
  studyDate: Timestamp;
  modalities: Modality[];
  seriesCount: number;
  instanceCount: number;
  description?: string;
  referringPhysician?: string;
  status: StudyStatus;
  series: SeriesInfo[];
}

export interface SeriesInfo {
  seriesId: SeriesID;
  seriesInstanceUID: string;
  seriesNumber: number;
  modality: Modality;
  seriesDescription?: string;
  instanceCount: number;
  bodyPart: string;
  laterality?: 'left' | 'right' | 'bilateral';
}

// ============================================================================
// Annotation Types
// ============================================================================

export interface ImageAnnotation {
  annotationId: string;
  imageId: ImageID;
  annotator: string;
  annotatorRole: string;
  timestamp: Timestamp;
  type: AnnotationType;
  coordinates: number[][];
  measurement?: Measurement;
  text?: string;
  color?: string;
  visible: boolean;
}

export interface Measurement {
  value: number;
  unit: string;
  type: 'length' | 'area' | 'angle' | 'hounsfield';
}

// ============================================================================
// Report Types
// ============================================================================

export interface RadiologyReport {
  reportId: string;
  studyId: StudyID;
  radiologistId: string;
  status: 'draft' | 'preliminary' | 'final' | 'amended';
  findings: string;
  impression: string;
  recommendations?: string;
  createdAt: Timestamp;
  signedAt?: Timestamp;
}

// ============================================================================
// API Types
// ============================================================================

export interface WIAConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T = unknown> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: Timestamp;
}

export interface APIError {
  code: string;
  message: string;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalCount: number;
  };
}
