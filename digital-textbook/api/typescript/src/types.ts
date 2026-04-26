/**
 * WIA-EDU-008 Digital Textbook Standard - TypeScript Type Definitions
 * Version: 2.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * WIA Digital Textbook Standard Version
 */
export type WIAVersion = '1.0' | '1.1' | '1.2' | '2.0';

/**
 * WCAG Accessibility Level
 */
export type WCAGLevel = 'A' | 'AA' | 'AAA';

/**
 * Textbook metadata following WIA-EDU-008 Phase 1
 */
export interface TextbookMetadata {
  /** ISBN or other unique identifier */
  id: string;
  /** WIA Registry ID */
  wiaId?: string;
  /** Textbook title */
  title: string;
  /** Subtitle (optional) */
  subtitle?: string;
  /** Authors */
  authors: Author[];
  /** Publisher information */
  publisher: Publisher;
  /** Edition (e.g., "3rd Edition") */
  edition?: string;
  /** Publication date (ISO 8601) */
  publicationDate: string;
  /** Languages (ISO 639) */
  language: string[];
  /** ISBN-13 */
  isbn?: string;
  /** DOI */
  doi?: string;
  /** Format (always EPUB3 for WIA-EDU-008) */
  format: 'EPUB3';
  /** File size in bytes */
  fileSize: number;
  /** Page count */
  pageCount?: number;
  /** Chapter count */
  chapterCount: number;
  /** WIA compliance status */
  wiaCompliant: boolean;
  /** WCAG accessibility level */
  wcagLevel: WCAGLevel;
  /** Subject categories */
  subjects: string[];
  /** Target education level */
  educationLevel: string[];
  /** Feature flags */
  features: TextbookFeatures;
  /** Links to resources */
  links: TextbookLinks;
  /** Content version */
  contentVersion: string;
}

/**
 * Author information
 */
export interface Author {
  /** Author name */
  name: string;
  /** ORCID identifier */
  orcid?: string;
  /** Author affiliation */
  affiliation?: string;
}

/**
 * Publisher information
 */
export interface Publisher {
  /** Publisher name */
  name: string;
  /** WIA Publisher ID */
  wiaId?: string;
  /** Publisher country */
  country?: string;
  /** Publisher website */
  website?: string;
}

/**
 * Textbook feature flags
 */
export interface TextbookFeatures {
  /** Contains video content */
  hasVideo: boolean;
  /** Contains audio content */
  hasAudio: boolean;
  /** Contains interactive elements */
  hasInteractive: boolean;
  /** Contains MathML */
  hasMathML: boolean;
  /** Contains assessments/quizzes */
  hasAssessments: boolean;
  /** Supports offline access */
  hasOfflineSupport?: boolean;
}

/**
 * Links to textbook resources
 */
export interface TextbookLinks {
  /** Download EPUB link */
  download: string;
  /** Cover image link */
  cover: string;
  /** Sample chapters link */
  sample: string;
  /** Table of contents link */
  toc: string;
}

/**
 * Annotation types
 */
export type AnnotationType = 'highlight' | 'note' | 'bookmark' | 'question';

/**
 * Annotation visibility
 */
export type AnnotationVisibility = 'private' | 'shared' | 'public';

/**
 * User annotation
 */
export interface Annotation {
  /** Unique annotation ID */
  id: string;
  /** Textbook ID */
  textbookId: string;
  /** Chapter ID */
  chapterId: string;
  /** Annotation type */
  type: AnnotationType;
  /** Text selector */
  selector: TextSelector;
  /** Highlight color (hex) */
  color?: string;
  /** Note text */
  note?: string;
  /** Tags */
  tags?: string[];
  /** Visibility */
  visibility: AnnotationVisibility;
  /** Created timestamp */
  created: string;
  /** Modified timestamp */
  modified: string;
  /** Sync status */
  syncStatus: SyncStatus;
}

/**
 * Text position selector
 */
export interface TextSelector {
  /** Selector type */
  type: 'TextPositionSelector' | 'FragmentSelector';
  /** Start position */
  start: number;
  /** End position */
  end: number;
  /** XPointer (for FragmentSelector) */
  value?: string;
}

/**
 * Sync status
 */
export type SyncStatus = 'pending' | 'syncing' | 'synced' | 'conflict' | 'error';

/**
 * Reading progress
 */
export interface ReadingProgress {
  /** Textbook ID */
  textbookId: string;
  /** Current chapter */
  currentChapter: number;
  /** Current page/position */
  currentPosition: number;
  /** Total progress (0-1) */
  progress: number;
  /** Last read timestamp */
  lastRead: string;
  /** Total reading time (ISO 8601 duration) */
  totalTime: string;
}

/**
 * Reading analytics
 */
export interface ReadingAnalytics {
  /** User ID */
  userId: string;
  /** Textbook ID */
  textbookId: string;
  /** Analysis period */
  period: string;
  /** Total reading time (seconds) */
  totalReadingTime: number;
  /** Chapters read */
  chaptersRead: number[];
  /** Average reading speed (words/minute) */
  averageSpeed: number;
  /** Comprehension score (0-1) */
  comprehensionScore?: number;
  /** Annotations created */
  annotationsCreated: number;
  /** Quizzes completed */
  quizzesCompleted: number;
  /** Quiz average score (0-1) */
  quizAverageScore?: number;
}

/**
 * Verifiable Credential for course completion
 */
export interface CourseCompletionCredential {
  /** W3C context */
  '@context': string[];
  /** Credential type */
  type: string[];
  /** Issuer DID */
  issuer: string | CredentialIssuer;
  /** Issuance date */
  issuanceDate: string;
  /** Expiration date (optional) */
  expirationDate?: string;
  /** Credential subject */
  credentialSubject: CredentialSubject;
  /** Cryptographic proof */
  proof: CredentialProof;
}

/**
 * Credential issuer
 */
export interface CredentialIssuer {
  /** Issuer DID */
  id: string;
  /** Issuer name */
  name: string;
  /** WIA Publisher ID */
  wiaId?: string;
}

/**
 * Credential subject (student)
 */
export interface CredentialSubject {
  /** Student DID */
  id: string;
  /** Student name */
  name: string;
  /** Student ID */
  studentId?: string;
  /** Completed course information */
  completedCourse: CompletedCourse;
  /** Achievement details */
  achievements?: Achievements;
}

/**
 * Completed course information
 */
export interface CompletedCourse {
  /** Course name */
  name: string;
  /** ISBN */
  isbn: string;
  /** WIA ID */
  wiaId?: string;
  /** Publisher */
  publisher: string;
  /** Completion date */
  completionDate: string;
  /** Grade */
  grade?: string;
  /** Grade scale */
  gradeScale?: string;
  /** Percentage score */
  percentageScore?: number;
  /** Credits */
  credits?: number;
  /** Duration */
  duration?: string;
}

/**
 * Student achievements
 */
export interface Achievements {
  /** Chapters completed */
  chaptersCompleted: number;
  /** Quiz average */
  quizAverage: number;
  /** Reading completeness (0-1) */
  readingCompleteness: number;
  /** Annotations created */
  annotationsCreated: number;
  /** Interactive elements completed */
  interactiveElementsCompleted: number;
}

/**
 * Cryptographic proof
 */
export interface CredentialProof {
  /** Proof type */
  type: string;
  /** Creation timestamp */
  created: string;
  /** Verification method */
  verificationMethod: string;
  /** Proof purpose */
  proofPurpose: string;
  /** Proof value (signature) */
  proofValue: string;
}

/**
 * xAPI Statement
 */
export interface XAPIStatement {
  /** Actor (student) */
  actor: XAPIActor;
  /** Verb (action) */
  verb: XAPIVerb;
  /** Object (activity) */
  object: XAPIObject;
  /** Result */
  result?: XAPIResult;
  /** Context */
  context?: XAPIContext;
  /** Timestamp */
  timestamp: string;
}

/**
 * xAPI Actor
 */
export interface XAPIActor {
  /** Object type */
  objectType: 'Agent';
  /** Name */
  name: string;
  /** Email */
  mbox?: string;
  /** Account */
  account?: {
    homePage: string;
    name: string;
  };
}

/**
 * xAPI Verb
 */
export interface XAPIVerb {
  /** Verb ID */
  id: string;
  /** Display */
  display: { [lang: string]: string };
}

/**
 * xAPI Object (Activity)
 */
export interface XAPIObject {
  /** Object type */
  objectType: 'Activity';
  /** Activity ID */
  id: string;
  /** Definition */
  definition: {
    name: { [lang: string]: string };
    description?: { [lang: string]: string };
    type: string;
    extensions?: { [key: string]: any };
  };
}

/**
 * xAPI Result
 */
export interface XAPIResult {
  /** Completion */
  completion?: boolean;
  /** Success */
  success?: boolean;
  /** Score */
  score?: {
    scaled?: number;
    raw?: number;
    min?: number;
    max?: number;
  };
  /** Duration */
  duration?: string;
}

/**
 * xAPI Context
 */
export interface XAPIContext {
  /** Context activities */
  contextActivities?: {
    parent?: XAPIObject[];
    category?: XAPIObject[];
  };
  /** Platform */
  platform?: string;
  /** Language */
  language?: string;
}

/**
 * Sync message types
 */
export type SyncMessageType =
  | 'sync.annotation.create'
  | 'sync.annotation.update'
  | 'sync.annotation.delete'
  | 'sync.progress.update'
  | 'sync.preferences.update'
  | 'sync.ack';

/**
 * Sync message
 */
export interface SyncMessage {
  /** Message type */
  type: SyncMessageType;
  /** Message ID */
  messageId: string;
  /** Timestamp */
  timestamp: string;
  /** Device ID */
  deviceId: string;
  /** Vector clock */
  vectorClock: { [deviceId: string]: number };
  /** Data payload */
  data?: any;
}

/**
 * API Configuration
 */
export interface APIConfig {
  /** Base API URL */
  baseURL: string;
  /** Access token */
  accessToken?: string;
  /** Timeout (ms) */
  timeout?: number;
  /** Enable WebSocket sync */
  enableSync?: boolean;
  /** WebSocket URL */
  syncURL?: string;
}

/**
 * API Error
 */
export interface APIError {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** Additional details */
  details?: {
    requestId: string;
    timestamp: string;
    [key: string]: any;
  };
}
