/**
 * WIA-LANG TypeScript Type Definitions
 * 弘益人間 · Benefit All Humanity
 */

export interface WIALangConfig {
  apiKey: string;
  endpoint?: string;
  version?: string;
  timeout?: number;
}

export interface LanguageMetadata {
  name: string;
  isoCode: string;
  family: string;
  endangerment: EndangermentLevel;
  speakers: SpeakerData;
  regions: string[];
}

export type EndangermentLevel = 
  | 'safe'
  | 'vulnerable'
  | 'definitely_endangered'
  | 'severely_endangered'
  | 'critically_endangered'
  | 'extinct';

export interface SpeakerData {
  native: number;
  l2: number;
  lastUpdated: string;
}

export interface RecordingOptions {
  format: AudioFormat;
  sampleRate: number;
  bitDepth: number;
  channels: number;
  metadata?: RecordingMetadata;
}

export type AudioFormat = 'flac' | 'wav' | 'mp3' | 'ogg';

export interface RecordingMetadata {
  speaker?: SpeakerInfo;
  location?: GeographicLocation;
  context?: string;
  tags?: string[];
}

export interface SpeakerInfo {
  id?: string;
  name?: string;
  age?: number;
  gender?: 'male' | 'female' | 'other';
  nativeStatus: boolean;
}

export interface GeographicLocation {
  latitude: number;
  longitude: number;
  accuracy?: number;
  address?: string;
}

export interface AnalysisResult {
  id: string;
  status: 'pending' | 'processing' | 'completed' | 'failed';
  features: AnalysisFeatures;
  timestamp: string;
}

export interface AnalysisFeatures {
  phonemes?: PhonemeData[];
  prosody?: ProsodyData;
  morphology?: MorphologyData;
  syntax?: SyntaxData;
}

export interface PhonemeData {
  symbol: string;
  ipa: string;
  count: number;
  distribution: number[];
}

export interface ProsodyData {
  pitch: number[];
  intensity: number[];
  duration: number[];
}

export interface MorphologyData {
  morphemes: string[];
  affixes: {
    prefixes: string[];
    suffixes: string[];
  };
}

export interface SyntaxData {
  parseTree: object;
  dependencies: Dependency[];
}

export interface Dependency {
  head: string;
  dependent: string;
  relation: string;
}

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  meta?: ResponseMetadata;
}

export interface APIError {
  code: string;
  message: string;
  details?: unknown;
}

export interface ResponseMetadata {
  timestamp: string;
  requestId: string;
  rateLimit: RateLimitInfo;
}

export interface RateLimitInfo {
  limit: number;
  remaining: number;
  resetAt: string;
}

export class WIALangClient {
  constructor(config: WIALangConfig);
  
  languages: {
    list(): Promise<APIResponse<LanguageMetadata[]>>;
    get(isoCode: string): Promise<APIResponse<LanguageMetadata>>;
    analyze(text: string, options?: AnalysisOptions): Promise<APIResponse<AnalysisResult>>;
  };
  
  recordings: {
    create(options: RecordingOptions): Promise<APIResponse<Recording>>;
    get(id: string): Promise<APIResponse<Recording>>;
    list(filters?: RecordingFilters): Promise<APIResponse<Recording[]>>;
    delete(id: string): Promise<APIResponse<void>>;
  };
  
  archive: {
    upload(file: File | Buffer, metadata: RecordingMetadata): Promise<APIResponse<ArchiveEntry>>;
    search(query: SearchQuery): Promise<APIResponse<ArchiveEntry[]>>;
    download(id: string): Promise<APIResponse<Buffer>>;
  };
}

export interface AnalysisOptions {
  features?: string[];
  language?: string;
  format?: 'json' | 'xml' | 'text';
}

export interface Recording {
  id: string;
  language: string;
  format: AudioFormat;
  duration: number;
  size: number;
  metadata: RecordingMetadata;
  createdAt: string;
}

export interface RecordingFilters {
  language?: string;
  dateFrom?: string;
  dateTo?: string;
  speaker?: string;
  tags?: string[];
}

export interface ArchiveEntry {
  id: string;
  filename: string;
  contentType: string;
  size: number;
  metadata: RecordingMetadata;
  url: string;
  createdAt: string;
}

export interface SearchQuery {
  q?: string;
  filters?: {
    language?: string;
    type?: string;
    dateRange?: {
      start: string;
      end: string;
    };
  };
  limit?: number;
  offset?: number;
}
