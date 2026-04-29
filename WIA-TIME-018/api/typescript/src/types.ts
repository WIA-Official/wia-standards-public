/**
 * WIA-TIME-018: Temporal Communication - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Temporal Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Three-dimensional vector for spatial coordinates
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Temporal coordinates in 4D spacetime
 */
export interface TemporalCoordinate {
  /** Time coordinate (ISO 8601 format) */
  time: Date | string;

  /** Spatial position in meters [x, y, z] */
  position: Vector3 | [number, number, number];

  /** Reference frame identifier */
  referenceFrame?: string;

  /** Timeline identifier */
  timeline?: string;
}

/**
 * Temporal entity (sender or recipient)
 */
export interface TemporalEntity {
  /** Unique entity identifier */
  id: string;

  /** Display name */
  name?: string;

  /** Public key for encryption/signatures */
  publicKey: string;

  /** Entity's temporal coordinates */
  temporalCoordinates: TemporalCoordinate;

  /** Timeline ID */
  timeline: string;

  /** Temporal public key for temporal signatures */
  temporalPublicKey?: string;
}

// ============================================================================
// Message Types
// ============================================================================

/**
 * Message types
 */
export type MessageType =
  | 'text'
  | 'data'
  | 'command'
  | 'query'
  | 'response'
  | 'alert'
  | 'broadcast'
  | 'sync';

/**
 * Message priority levels
 */
export type MessagePriority = 'low' | 'medium' | 'high' | 'critical';

/**
 * Message content structure
 */
export interface MessageContent {
  /** Content type */
  type: 'text' | 'binary' | 'json' | 'quantum' | 'multipart';

  /** Encoding format */
  encoding: string;

  /** Is content encrypted? */
  encrypted: boolean;

  /** Actual message data */
  data: string | ArrayBuffer | object;

  /** MIME type (for binary data) */
  mimeType?: string;

  /** Data compression (if applied) */
  compression?: 'none' | 'gzip' | 'temporal' | 'quantum';
}

/**
 * Routing information
 */
export interface RoutingInfo {
  /** Routing method */
  method: 'quantum-entangled' | 'wormhole' | 'direct' | 'multi-hop' | 'ctc' | 'field';

  /** Hops through spacetime */
  hops: TemporalCoordinate[];

  /** Estimated latency in seconds */
  latency: number;

  /** Energy cost in joules */
  energyCost?: number;

  /** Route reliability (0-1) */
  reliability?: number;

  /** Wormhole ID (if applicable) */
  wormholeId?: string;

  /** Quantum channel ID (if applicable) */
  quantumChannelId?: string;
}

/**
 * Security information
 */
export interface SecurityInfo {
  /** Encryption algorithm */
  encryption: string;

  /** Digital signature */
  signature: string;

  /** Temporal signature */
  temporalSignature?: string;

  /** Signature timestamp */
  timestamp: Date | string;

  /** Novikov consistency hash */
  novikovHash: string;

  /** Temporal hash */
  temporalHash?: string;

  /** Security level */
  level: SecurityLevel;

  /** Encryption key ID */
  keyId?: string;
}

/**
 * Message metadata
 */
export interface MessageMetadata {
  /** Message priority */
  priority: MessagePriority;

  /** Require acknowledgment? */
  requireAck: boolean;

  /** Time to live in seconds */
  ttl: number;

  /** Is Novikov consistent? */
  novikovConsistent: boolean;

  /** Message tags */
  tags?: string[];

  /** Custom metadata */
  custom?: Record<string, unknown>;

  /** Conversation thread ID */
  threadId?: string;

  /** Reply to message ID */
  replyTo?: string;
}

/**
 * Complete temporal message structure
 */
export interface TemporalMessage {
  /** Unique message identifier */
  id: string;

  /** Protocol version */
  version: string;

  /** Message type */
  type: MessageType;

  /** Origin time */
  originTime: Date | string;

  /** Target time */
  targetTime: Date | string;

  /** Origin timeline */
  originTimeline: string;

  /** Target timeline */
  targetTimeline: string;

  /** Sender information */
  sender: TemporalEntity;

  /** Recipient information */
  recipient: TemporalEntity;

  /** Message content */
  content: MessageContent;

  /** Routing information */
  routing: RoutingInfo;

  /** Security information */
  security: SecurityInfo;

  /** Metadata */
  metadata: MessageMetadata;

  /** Created timestamp */
  created: Date | string;
}

// ============================================================================
// Message Operations
// ============================================================================

/**
 * Send message request
 */
export interface SendMessageRequest {
  /** Message content */
  content: string | ArrayBuffer | object;

  /** Target temporal coordinates */
  target: {
    time: Date;
    timeline: string;
    recipientId?: string;
    position?: Vector3;
  };

  /** Send options */
  options?: {
    priority?: MessagePriority;
    encryption?: SecurityLevel;
    requireAck?: boolean;
    timeout?: number;
    routing?: 'auto' | 'wormhole' | 'quantum' | 'direct' | 'multi-hop';
    ttl?: number;
    tags?: string[];
    threadId?: string;
    replyTo?: string;
  };
}

/**
 * Send message response
 */
export interface SendMessageResponse {
  /** Message ID */
  messageId: string;

  /** Send status */
  status: 'sent' | 'queued' | 'failed' | 'rejected';

  /** Estimated delivery time */
  estimatedDelivery: Date;

  /** Actual route used */
  route: RoutingInfo;

  /** Energy cost in joules */
  energyCost: number;

  /** Error message if failed */
  error?: string;
}

/**
 * Message acknowledgment
 */
export interface MessageAcknowledgment {
  /** Original message ID */
  messageId: string;

  /** Reception timestamp */
  receivedAt: Date;

  /** Acknowledgment status */
  status: 'delivered' | 'failed' | 'pending' | 'rejected';

  /** Recipient's signature */
  signature: string;

  /** Novikov consistency check result */
  novikovConsistent: boolean;

  /** Receiving timeline */
  timeline: string;

  /** Rejection reason (if rejected) */
  rejectionReason?: string;
}

/**
 * Received message handler
 */
export interface MessageReceived {
  /** Received message */
  message: TemporalMessage;

  /** Reception timestamp */
  receivedAt: Date;

  /** Verification result */
  verification: VerificationResult;

  /** Acknowledge message */
  acknowledge(): Promise<void>;

  /** Reject message */
  reject(reason: string): Promise<void>;

  /** Forward message to other recipients */
  forward(targets: TemporalEntity[]): Promise<void>;
}

// ============================================================================
// Channel Types
// ============================================================================

/**
 * Channel types
 */
export type ChannelType =
  | 'quantum-entangled'
  | 'wormhole'
  | 'direct'
  | 'field-modulated'
  | 'ctc-loop';

/**
 * Channel status
 */
export type ChannelStatus =
  | 'initializing'
  | 'active'
  | 'unstable'
  | 'degraded'
  | 'failed'
  | 'shutdown';

/**
 * Temporal channel configuration
 */
export interface TemporalChannel {
  /** Unique channel identifier */
  id: string;

  /** Channel type */
  type: ChannelType;

  /** Endpoint A */
  endpointA: TemporalCoordinate;

  /** Endpoint B */
  endpointB: TemporalCoordinate;

  /** Channel bandwidth in bits/second */
  bandwidth: number;

  /** Current capacity utilization (0-1) */
  utilization: number;

  /** Channel latency in seconds */
  latency: number;

  /** Channel reliability (0-1) */
  reliability: number;

  /** Channel status */
  status: ChannelStatus;

  /** Security level */
  security: SecurityLevel;

  /** Created timestamp */
  created: Date;

  /** Last activity timestamp */
  lastActivity: Date;

  /** Channel metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Create channel request
 */
export interface CreateChannelRequest {
  /** Channel type */
  type: ChannelType;

  /** Endpoint A coordinates */
  endpointA: TemporalCoordinate;

  /** Endpoint B coordinates */
  endpointB: TemporalCoordinate;

  /** Desired bandwidth in bits/second */
  bandwidth: number;

  /** Security level */
  security: SecurityLevel;

  /** Channel options */
  options?: {
    minReliability?: number;
    maxLatency?: number;
    redundancy?: boolean;
    encryption?: string;
  };
}

/**
 * Create channel response
 */
export interface CreateChannelResponse {
  /** Channel ID */
  channelId: string;

  /** Actual capacity in bits/second */
  capacity: number;

  /** Actual latency in seconds */
  latency: number;

  /** Actual reliability (0-1) */
  reliability: number;

  /** Channel status */
  status: ChannelStatus;

  /** Setup energy cost in joules */
  setupCost?: number;

  /** Maintenance energy per second */
  maintenanceCost?: number;
}

// ============================================================================
// Quantum Channel Types
// ============================================================================

/**
 * Quantum channel configuration
 */
export interface QuantumChannel extends TemporalChannel {
  type: 'quantum-entangled';

  /** Entanglement strength (0-1) */
  entanglementStrength: number;

  /** Number of entangled pairs */
  entangledPairs: number;

  /** Decoherence rate */
  decoherenceRate: number;

  /** Bell state fidelity (0-1) */
  fidelity: number;

  /** Quantum error correction code */
  qecCode?: string;
}

/**
 * Create quantum channel request
 */
export interface CreateQuantumChannelRequest {
  /** Endpoint A */
  endpointA: TemporalCoordinate;

  /** Endpoint B */
  endpointB: TemporalCoordinate;

  /** Desired entanglement strength (0-1) */
  entanglementStrength: number;

  /** Number of entangled pairs to generate */
  pairCount?: number;

  /** Quantum error correction */
  errorCorrection?: boolean;

  /** Distribution method */
  distributionMethod?: 'wormhole' | 'classical' | 'teleportation';
}

// ============================================================================
// Broadcasting Types
// ============================================================================

/**
 * Broadcast types
 */
export type BroadcastType = 'parallel' | 'sequential' | 'cascading';

/**
 * Parallel broadcast configuration
 */
export interface ParallelBroadcast {
  /** Broadcast type */
  type: 'parallel';

  /** Message to broadcast */
  message: TemporalMessage | string;

  /** Target timelines */
  targetTimelines: string[];

  /** Synchronize delivery? */
  synchronize: boolean;

  /** Require all acknowledgments? */
  requireAllAcks: boolean;

  /** Timeout in seconds */
  timeout: number;
}

/**
 * Sequential broadcast configuration
 */
export interface SequentialBroadcast {
  /** Broadcast type */
  type: 'sequential';

  /** Message to broadcast */
  message: TemporalMessage | string;

  /** Ordered timeline sequence */
  timelineSequence: string[];

  /** Delay between sends in seconds */
  delayBetween: number;

  /** Propagate responses to other timelines? */
  propagateResponses: boolean;
}

/**
 * Cascading broadcast configuration
 */
export interface CascadingBroadcast {
  /** Broadcast type */
  type: 'cascading';

  /** Message to broadcast */
  message: TemporalMessage | string;

  /** Root timeline */
  rootTimeline: string;

  /** Maximum cascade depth */
  maxDepth: number;

  /** Branch factor (children per node) */
  branchFactor: number;

  /** Prune Novikov-inconsistent branches? */
  pruneInconsistent: boolean;
}

/**
 * Broadcast result
 */
export interface BroadcastResult {
  /** Broadcast ID */
  broadcastId: string;

  /** Total messages sent */
  totalSent: number;

  /** Successful deliveries */
  successful: number;

  /** Failed deliveries */
  failed: number;

  /** Delivery details per timeline */
  details: {
    timeline: string;
    status: 'success' | 'failed' | 'pending';
    messageId?: string;
    error?: string;
  }[];

  /** Total energy cost */
  energyCost: number;

  /** Total duration */
  duration: number;
}

// ============================================================================
// Security Types
// ============================================================================

/**
 * Security levels
 */
export type SecurityLevel = 1 | 2 | 3 | 4 | 5;

/**
 * Encryption schemes
 */
export type EncryptionScheme =
  | 'aes-128'
  | 'aes-256'
  | 'temporal-aes-256'
  | 'kyber-768'
  | 'quantum-temporal'
  | 'custom';

/**
 * Signature algorithms
 */
export type SignatureAlgorithm =
  | 'ecdsa-p256'
  | 'ecdsa-p521'
  | 'rsa-4096'
  | 'dilithium-3'
  | 'lattice-based'
  | 'temporal-quantum';

/**
 * Verification result
 */
export interface VerificationResult {
  /** Overall validity */
  valid: boolean;

  /** Standard signature valid? */
  standardSignature: boolean;

  /** Temporal signature valid? */
  temporalSignature: boolean;

  /** Novikov consistent? */
  novikovConsistent: boolean;

  /** Temporal hash valid? */
  temporalHash: boolean;

  /** Timeline compatible? */
  timelineCompatible: boolean;

  /** Errors encountered */
  errors: string[];

  /** Warnings */
  warnings: string[];

  /** Verification timestamp */
  verifiedAt: Date;
}

/**
 * Access control list entry
 */
export interface TemporalACL {
  /** Resource identifier */
  resourceId: string;

  /** Permissions */
  permissions: {
    /** Allowed time range */
    timeRange: {
      start: Date;
      end: Date;
    };

    /** Allowed timelines */
    allowedTimelines: string[];

    /** Allowed entities */
    allowedEntities: {
      entityId: string;
      permissions: ('read' | 'write' | 'forward' | 'broadcast')[];
    }[];

    /** Restrictions */
    restrictions: {
      maxMessageSize: number;
      maxFrequency: number;
      requiredEncryption: SecurityLevel;
      allowedMessageTypes: MessageType[];
    };
  };
}

// ============================================================================
// Timeline Types
// ============================================================================

/**
 * Timeline information
 */
export interface Timeline {
  /** Unique timeline identifier */
  id: string;

  /** Timeline name/description */
  name?: string;

  /** Parent timeline (if branched) */
  parentTimeline?: string;

  /** Branch point timestamp */
  branchPoint?: Date;

  /** Timeline divergence from parent (0-1) */
  divergence?: number;

  /** Timeline integrity score (0-1) */
  integrity: number;

  /** Is timeline accessible? */
  accessible: boolean;

  /** Timeline status */
  status: 'active' | 'inactive' | 'unstable' | 'collapsed';

  /** Key events in timeline */
  events?: TimelineEvent[];
}

/**
 * Timeline event
 */
export interface TimelineEvent {
  /** Event identifier */
  id: string;

  /** Event timestamp */
  time: Date;

  /** Event description */
  description: string;

  /** Affected entities */
  affected?: string[];

  /** Causality chain */
  causes?: string[];

  /** Effects */
  effects?: string[];

  /** Can this event be changed? */
  mutable: boolean;
}

/**
 * Timeline discovery parameters
 */
export interface TimelineDiscoveryParams {
  /** Origin timeline */
  origin: string;

  /** Maximum number of timelines to discover */
  maxTimelines?: number;

  /** Maximum divergence threshold (0-1) */
  maxDivergence?: number;

  /** Search depth */
  searchDepth?: number;

  /** Include inactive timelines? */
  includeInactive?: boolean;
}

// ============================================================================
// Routing Types
// ============================================================================

/**
 * Routing optimization criteria
 */
export type RoutingOptimization =
  | 'latency'
  | 'energy'
  | 'reliability'
  | 'security'
  | 'balanced';

/**
 * Route segment
 */
export interface RouteSegment {
  /** Starting point */
  from: TemporalCoordinate;

  /** Ending point */
  to: TemporalCoordinate;

  /** Segment method */
  method: 'quantum' | 'wormhole' | 'direct' | 'field';

  /** Segment latency */
  latency: number;

  /** Segment energy cost */
  energyCost: number;

  /** Segment reliability (0-1) */
  reliability: number;
}

/**
 * Complete route
 */
export interface Route {
  /** Route identifier */
  id: string;

  /** Origin point */
  origin: TemporalCoordinate;

  /** Destination point */
  destination: TemporalCoordinate;

  /** Route segments */
  segments: RouteSegment[];

  /** Total latency */
  totalLatency: number;

  /** Total energy cost */
  totalEnergyCost: number;

  /** Overall reliability */
  overallReliability: number;

  /** Route quality score (0-100) */
  qualityScore: number;
}

// ============================================================================
// Paradox Detection Types
// ============================================================================

/**
 * Paradox types
 */
export type ParadoxType =
  | 'grandfather'
  | 'bootstrap'
  | 'predestination'
  | 'ontological'
  | 'causal-loop'
  | 'information'
  | 'unknown';

/**
 * Paradox detection result
 */
export interface ParadoxDetection {
  /** Paradox detected? */
  detected: boolean;

  /** Paradox type */
  type?: ParadoxType;

  /** Paradox probability (0-1) */
  probability: number;

  /** Paradox severity */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Description */
  description: string;

  /** Affected events */
  affectedEvents: string[];

  /** Possible resolutions */
  resolutions?: string[];

  /** Should block transmission? */
  shouldBlock: boolean;
}

/**
 * Novikov consistency check
 */
export interface NovikovConsistency {
  /** Is consistent? */
  consistent: boolean;

  /** Consistency probability (0-1) */
  probability: number;

  /** Detected contradictions */
  contradictions: string[];

  /** Warnings */
  warnings: string[];

  /** Recommendation */
  recommendation: 'proceed' | 'proceed-with-caution' | 'abort' | 'impossible';

  /** Alternative consistent timelines */
  alternativeTimelines?: string[];
}

// ============================================================================
// Event Handlers
// ============================================================================

/**
 * Message event handler
 */
export type MessageHandler = (message: MessageReceived) => void | Promise<void>;

/**
 * Channel event handler
 */
export type ChannelEventHandler = (event: ChannelEvent) => void;

/**
 * Channel event types
 */
export type ChannelEventType =
  | 'created'
  | 'active'
  | 'unstable'
  | 'degraded'
  | 'failed'
  | 'shutdown';

/**
 * Channel event
 */
export interface ChannelEvent {
  /** Event type */
  type: ChannelEventType;

  /** Channel ID */
  channelId: string;

  /** Event timestamp */
  timestamp: Date;

  /** Event details */
  details?: Record<string, unknown>;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Temporal communication error codes
 */
export enum TemporalCommErrorCode {
  INVALID_TIME = 'TC001',
  TIMELINE_UNREACHABLE = 'TC002',
  PARADOX_DETECTED = 'TC003',
  CHANNEL_UNAVAILABLE = 'TC004',
  ENCRYPTION_FAILED = 'TC005',
  SIGNATURE_INVALID = 'TC006',
  NOVIKOV_VIOLATION = 'TC007',
  BANDWIDTH_EXCEEDED = 'TC008',
  ACCESS_DENIED = 'TC009',
  RATE_LIMIT = 'TC010',
}

/**
 * Temporal communication error
 */
export class TemporalCommError extends Error {
  constructor(
    public code: TemporalCommErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TemporalCommError';
  }
}

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Temporal communicator configuration
 */
export interface TemporalCommConfig {
  /** Timeline identifier */
  timelineId: string;

  /** Enable quantum entanglement? */
  quantumEntanglement?: boolean;

  /** Default encryption scheme */
  encryption?: EncryptionScheme;

  /** Default security level */
  securityLevel?: SecurityLevel;

  /** Enable Novikov checking? */
  novikovChecking?: boolean;

  /** Default routing optimization */
  routingOptimization?: RoutingOptimization;

  /** Message rate limit (messages/hour) */
  rateLimit?: number;

  /** Maximum message size (bytes) */
  maxMessageSize?: number;

  /** Enable automatic acknowledgments? */
  autoAck?: boolean;

  /** Network endpoints */
  endpoints?: {
    api?: string;
    quantum?: string;
    wormhole?: string;
  };

  /** Custom configuration */
  custom?: Record<string, unknown>;
}

// ============================================================================
// Statistics Types
// ============================================================================

/**
 * Communication statistics
 */
export interface CommStatistics {
  /** Total messages sent */
  messagesSent: number;

  /** Total messages received */
  messagesReceived: number;

  /** Total broadcasts */
  broadcasts: number;

  /** Total energy consumed (joules) */
  energyConsumed: number;

  /** Average latency (seconds) */
  averageLatency: number;

  /** Success rate (0-1) */
  successRate: number;

  /** Active channels */
  activeChannels: number;

  /** Paradoxes detected */
  paradoxesDetected: number;

  /** Time period */
  period: {
    start: Date;
    end: Date;
  };
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Temporal communication constants
 */
export const TEMPORAL_COMM_CONSTANTS = {
  /** Maximum messages per hour */
  MAX_MESSAGE_RATE: 1000,

  /** Maximum broadcast recipients */
  MAX_BROADCAST_RECIPIENTS: 100,

  /** Maximum message size (bytes) */
  MAX_MESSAGE_SIZE: 10 * 1024 * 1024, // 10 MB

  /** Maximum temporal displacement (seconds) */
  MAX_DISPLACEMENT: 100 * 365.25 * 24 * 3600, // ±100 years

  /** Minimum broadcast interval (seconds) */
  MIN_BROADCAST_INTERVAL: 60,

  /** Temporal coherence time (seconds) */
  TEMPORAL_COHERENCE: 1,

  /** Default message TTL (seconds) */
  DEFAULT_TTL: 3600,

  /** Default security level */
  DEFAULT_SECURITY_LEVEL: 3 as SecurityLevel,

  /** Paradox threshold */
  PARADOX_THRESHOLD: 0.1,
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core
  Vector3,
  TemporalCoordinate,
  TemporalEntity,

  // Messages
  MessageType,
  MessagePriority,
  MessageContent,
  RoutingInfo,
  SecurityInfo,
  MessageMetadata,
  TemporalMessage,

  // Operations
  SendMessageRequest,
  SendMessageResponse,
  MessageAcknowledgment,
  MessageReceived,

  // Channels
  ChannelType,
  ChannelStatus,
  TemporalChannel,
  CreateChannelRequest,
  CreateChannelResponse,

  // Quantum
  QuantumChannel,
  CreateQuantumChannelRequest,

  // Broadcasting
  BroadcastType,
  ParallelBroadcast,
  SequentialBroadcast,
  CascadingBroadcast,
  BroadcastResult,

  // Security
  SecurityLevel,
  EncryptionScheme,
  SignatureAlgorithm,
  VerificationResult,
  TemporalACL,

  // Timeline
  Timeline,
  TimelineEvent,
  TimelineDiscoveryParams,

  // Routing
  RoutingOptimization,
  RouteSegment,
  Route,

  // Paradox
  ParadoxType,
  ParadoxDetection,
  NovikovConsistency,

  // Events
  MessageHandler,
  ChannelEventHandler,
  ChannelEventType,
  ChannelEvent,

  // Config
  TemporalCommConfig,

  // Stats
  CommStatistics,
};

export { TemporalCommErrorCode, TemporalCommError, TEMPORAL_COMM_CONSTANTS };
