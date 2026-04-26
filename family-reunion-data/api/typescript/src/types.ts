/**
 * WIA-UNI-003: Family Reunion Data Standard - TypeScript Types
 * 
 * @module @wia/family-reunion-sdk/types
 * @version 1.0.0
 */

export interface Name {
  fullName: string;
  language: string;
  script: 'Latn' | 'Kore' | 'Arab' | 'Cyrl' | 'Hans' | 'Hant';
  isPrimary: boolean;
}

export interface BirthDate {
  date: string; // ISO 8601 or partial (YYYY or YYYY-MM)
  isApproximate: boolean;
  accuracy: 'exact' | 'year' | 'decade';
}

export interface Location {
  city?: string;
  region?: string;
  country: string; // ISO 3166-1 alpha-2
  coordinates?: { lat: number; lng: number };
}

export interface SeparationDetails {
  separationDate: string;
  separationEvent: 'KOREAN_WAR' | 'SYRIAN_CRISIS' | 'UKRAINE_WAR' | 'NATURAL_DISASTER' | 'OTHER';
  circumstances: string;
}

export interface BiometricData {
  dnaProfileId?: string;
  photoIds?: string[];
  fingerprints?: string;
}

export interface Relationship {
  relationshipId: string;
  relatedPersonId: string;
  relationshipType: 'PARENT' | 'CHILD' | 'SIBLING' | 'SPOUSE' | 'COUSIN' | 'GRANDPARENT' | 'GRANDCHILD';
  confidence: number; // 0-100
  verified: boolean;
  source: 'DNA' | 'PHOTO' | 'DOCUMENT' | 'TESTIMONY';
}

export interface PrivacySettings {
  allowDNAMatching: boolean;
  allowPhotoMatching: boolean;
  allowContactByMatches: boolean;
  dataRetentionYears: number | 'INDEFINITE';
}

export interface Person {
  personId: string;
  names: Name[];
  birthDate: BirthDate;
  birthPlace: Location;
  gender: 'MALE' | 'FEMALE' | 'OTHER' | 'UNKNOWN';
  currentStatus: 'SEEKING' | 'FOUND' | 'DECEASED' | 'UNKNOWN';
  lastKnownLocation?: Location;
  separationDetails: SeparationDetails;
  biometricData?: BiometricData;
  relationships?: Relationship[];
  privacySettings: PrivacySettings;
  consentGiven: boolean;
  consentDate: string;
  createdAt: string;
  updatedAt: string;
}

export interface SearchCriteria {
  name?: string;
  birthDate?: string;
  birthPlace?: string;
  lastKnownLocation?: string;
  separationEvent?: string;
  familyMembers?: string[];
}

export interface SearchOptions {
  fuzzyMatching?: boolean;
  maxResults?: number;
  confidenceThreshold?: number;
}

export interface SearchMatch {
  personId: string;
  matchScore: number;
  matchReasons: string[];
  person: Person;
  relationshipProbability?: Record<string, number>;
}

export interface SearchResponse {
  matches: SearchMatch[];
  totalMatches: number;
  searchId: string;
}

export interface DNAProfile {
  dnaProfileId: string;
  personId: string;
  sampleType: 'SALIVA' | 'BLOOD' | 'BUCCAL_SWAB';
  collectionDate: string;
  ethnicity: {
    primary: string;
    percentages: Record<string, number>;
  };
  qualityScore: number;
  processingLab: string;
  privacyTier: 'FULL_MATCH' | 'HASH_ONLY' | 'NO_SHARING';
}

export interface DNAMatch {
  matchedPersonId: string;
  sharedDNA: number; // percentage
  sharedSegments: number; // centiMorgans
  relationship: 'FULL_SIBLING' | 'HALF_SIBLING' | 'PARENT' | 'CHILD' | 'COUSIN' | 'UNKNOWN';
  confidence: number;
}

export interface DNAMatchResponse {
  matches: DNAMatch[];
}

export interface PhotoMetadata {
  photoId: string;
  personId: string;
  uploadDate: string;
  photoDate: string;
  photoAge: number;
  quality: 'EXCELLENT' | 'GOOD' | 'FAIR' | 'POOR';
  restorationApplied: boolean;
  ageProgressionApplied: boolean;
  metadata?: {
    location?: string;
    occasion?: string;
    description?: string;
  };
}

export interface PhotoAnalysisResponse {
  facesDetected: number;
  matches: Array<{
    personId: string;
    confidence: number;
    ageInPhoto: number;
  }>;
  enhancedPhotoUrl?: string;
}

export interface ReunionRequest {
  seekerId: string;
  matchId: string;
  message: string;
  consentForContact: boolean;
}

export interface ReunionResponse {
  reunionId: string;
  status: 'PENDING_APPROVAL' | 'APPROVED' | 'DECLINED' | 'IN_PROGRESS' | 'COMPLETED';
  mediatorAssigned: boolean;
}

export interface ClientConfig {
  apiKey: string;
  environment?: 'production' | 'sandbox';
  region?: 'global' | 'asia' | 'europe' | 'americas';
  baseURL?: string;
}
