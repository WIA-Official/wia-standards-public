/**
 * WIA-UNI-006 TypeScript Type Definitions
 * Telecommunications Unification Standard
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

export type Region = 'north' | 'south';

export type NetworkMode = '5g' | 'lte' | '3g' | 'auto';

export type CallType = 'voice' | 'video' | 'video-hd' | 'video-4k';

export type MessageType = 'sms' | 'mms' | 'rcs';

export type ServicePriority = 'normal' | 'high' | 'critical' | 'emergency';

export type CallStatus = 'idle' | 'dialing' | 'ringing' | 'connected' | 'ended' | 'failed';

export type RoamingStatus = 'home' | 'roaming' | 'dmz-crossing';

export type NetworkSlice =
  | 'domestic'
  | 'cross-border-family'
  | 'business'
  | 'emergency'
  | 'internet';

// ============================================================================
// Configuration
// ============================================================================

export interface TelecomConfig {
  region: Region;
  carrier?: string;
  mode?: NetworkMode;
  roaming?: boolean;
  translation?: boolean;
  apiKey?: string;
  baseUrl?: string;
}

// ============================================================================
// Network & Location
// ============================================================================

export interface NetworkInfo {
  mode: NetworkMode;
  signalStrength: number; // 0-100
  carrier: string;
  region: Region;
  roaming: boolean;
  slice: NetworkSlice;
}

export interface Location {
  region: Region;
  city?: string;
  coordinates?: {
    latitude: number;
    longitude: number;
  };
  dmzProximity?: number; // km from DMZ
}

// ============================================================================
// Call & Communication
// ============================================================================

export interface PhoneNumber {
  region: Region;
  number: string; // International format
  verified?: boolean;
}

export interface CallOptions {
  from: PhoneNumber;
  to: PhoneNumber;
  type: CallType;
  translation?: boolean;
  quality?: 'sd' | 'hd' | '4k';
  priority?: ServicePriority;
  record?: boolean;
}

export interface Call {
  id: string;
  from: PhoneNumber;
  to: PhoneNumber;
  type: CallType;
  status: CallStatus;
  startedAt?: Date;
  endedAt?: Date;
  duration?: number; // seconds
  quality: {
    bitrate: number;
    latency: number;
    packetLoss: number;
    mos: number; // Mean Opinion Score (1-5)
  };
  humanitarian?: boolean;
  cost: number; // Won (₩)
}

export interface Message {
  id: string;
  type: MessageType;
  from: PhoneNumber;
  to: PhoneNumber;
  content: string;
  timestamp: Date;
  status: 'pending' | 'sent' | 'delivered' | 'read' | 'failed';
  translated?: boolean;
  encrypted: boolean;
  attachments?: MessageAttachment[];
}

export interface MessageAttachment {
  id: string;
  type: 'photo' | 'video' | 'audio' | 'document';
  url: string;
  filename: string;
  size: number; // bytes
  mimeType: string;
}

// ============================================================================
// Roaming & Billing
// ============================================================================

export interface RoamingInfo {
  active: boolean;
  status: RoamingStatus;
  location: Location;
  network: NetworkInfo;
  crossBorderCharges: number; // 0 for family calls
  dataUsed: number; // MB
  callMinutes: number;
}

export interface BillingInfo {
  period: {
    start: Date;
    end: Date;
  };
  plan: {
    name: string;
    price: number; // Won
    dataLimit: number; // MB
  };
  usage: {
    data: number; // MB used
    domesticCalls: number; // minutes
    crossBorderCalls: number; // minutes
    familyCalls: number; // minutes (free)
    messages: number;
  };
  charges: {
    basePlan: number;
    overageData: number;
    businessCalls: number;
    international: number;
    total: number;
  };
}

// ============================================================================
// Family Verification
// ============================================================================

export interface FamilyRelationship {
  userId: string;
  relatedUserId: string;
  relationship: 'parent' | 'child' | 'sibling' | 'spouse' | 'grandparent' | 'grandchild' | 'other';
  verified: boolean;
  verifiedBy: 'red-cross' | 'government' | 'pending';
  verifiedAt?: Date;
}

export interface FamilyMember {
  userId: string;
  name?: string;
  phoneNumber: PhoneNumber;
  region: Region;
  relationship: FamilyRelationship;
  lastContact?: Date;
}

// ============================================================================
// Emergency Services
// ============================================================================

export interface EmergencyCall {
  id: string;
  type: '112' | '119'; // Police or Medical/Fire
  from: PhoneNumber;
  location: Location;
  timestamp: Date;
  status: 'active' | 'dispatched' | 'resolved';
  priority: 'critical';
  transcript?: string;
  responseTime?: number; // seconds
}

// ============================================================================
// Translation
// ============================================================================

export interface TranslationOptions {
  enabled: boolean;
  sourceDialect?: 'north' | 'south' | 'auto';
  targetDialect?: 'north' | 'south';
  preserveEmotion?: boolean;
  realtime?: boolean;
}

export interface TranslationResult {
  original: string;
  translated: string;
  sourceDialect: 'north' | 'south';
  confidence: number; // 0-1
  latency: number; // ms
}

// ============================================================================
// Network Statistics
// ============================================================================

export interface NetworkStats {
  timestamp: Date;
  network: NetworkInfo;
  performance: {
    downloadSpeed: number; // Mbps
    uploadSpeed: number; // Mbps
    latency: number; // ms
    jitter: number; // ms
    packetLoss: number; // percentage
  };
  coverage: {
    latitude: number;
    longitude: number;
    signalStrength: number;
  };
}

// ============================================================================
// API Response Types
// ============================================================================

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: Date;
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  hasMore: boolean;
}

// ============================================================================
// Events
// ============================================================================

export type TelecomEventType =
  | 'call-started'
  | 'call-ended'
  | 'message-received'
  | 'roaming-status-changed'
  | 'network-changed'
  | 'emergency-alert';

export interface TelecomEvent {
  type: TelecomEventType;
  timestamp: Date;
  data: any;
}

export type EventHandler = (event: TelecomEvent) => void;

// ============================================================================
// Service Options
// ============================================================================

export interface SpeedTestResult {
  download: number; // Mbps
  upload: number; // Mbps
  latency: number; // ms
  server: string;
  timestamp: Date;
}

export interface DataUsage {
  period: {
    start: Date;
    end: Date;
  };
  total: number; // MB
  byApp: {
    [appName: string]: number;
  };
  byType: {
    domestic: number;
    roaming: number;
  };
}
