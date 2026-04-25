/**
 * WIA Internet Host Protocol (WIHP) - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Network Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Host Identification Types
// ============================================================================

/**
 * Unique host identifier in WIHP network
 * Format: wihp:[hash-algorithm]:[base58-encoded-hash]
 * Example: wihp:sha256:QmYwAPJzv5CZsnA6m7T6JzQ9pqj3mz9R4dK2k5T6bxmr8z
 */
export type HostId = string;

/**
 * Host address types supported by WIHP
 */
export type AddressFamily = 'IPv4' | 'IPv6' | 'WIHP' | 'Hybrid';

/**
 * Host address information
 */
export interface HostAddress {
  /** Address family type */
  family: AddressFamily;

  /** IP address or WIHP address */
  address: string;

  /** Port number */
  port: number;

  /** Protocol (TCP, UDP, QUIC, etc.) */
  protocol: 'TCP' | 'UDP' | 'QUIC' | 'SCTP';

  /** Address priority (0-255, higher is better) */
  priority?: number;

  /** Weight for load balancing */
  weight?: number;
}

/**
 * Cryptographic fingerprint of a host
 */
export interface HostFingerprint {
  /** Hash algorithm used */
  algorithm: 'SHA-256' | 'SHA-512' | 'BLAKE3';

  /** Hex-encoded fingerprint */
  fingerprint: string;

  /** Fingerprint creation timestamp */
  created_at: Date | string;

  /** Fingerprint validity period in seconds */
  validity_seconds: number;
}

/**
 * Host metadata and capabilities
 */
export interface HostMetadata {
  /** Host identifier */
  host_id: HostId;

  /** Human-readable hostname */
  hostname: string;

  /** Available addresses */
  addresses: HostAddress[];

  /** Host fingerprint */
  fingerprint: HostFingerprint;

  /** Supported protocol versions */
  protocol_versions: string[];

  /** Host capabilities */
  capabilities: string[];

  /** Public key (PEM format) */
  public_key: string;

  /** Last seen timestamp */
  last_seen?: Date | string;
}

// ============================================================================
// Routing Types
// ============================================================================

/**
 * Route status
 */
export type RouteStatus = 'active' | 'backup' | 'deprecated' | 'unreachable';

/**
 * Path metrics for routing decisions
 */
export interface PathMetrics {
  /** Round-trip time in milliseconds */
  rtt_ms: number;

  /** Packet loss rate (0-1) */
  loss_rate: number;

  /** Available bandwidth in Mbps */
  bandwidth_mbps: number;

  /** Hop count */
  hop_count: number;

  /** Path reliability score (0-1) */
  reliability: number;

  /** Congestion level (0-1) */
  congestion: number;

  /** Last updated timestamp */
  updated_at: Date | string;
}

/**
 * Next hop information
 */
export interface NextHop {
  /** Next hop host ID */
  host_id: HostId;

  /** Next hop address */
  address: HostAddress;

  /** Interface to use */
  interface?: string;

  /** Path metrics to next hop */
  metrics: PathMetrics;
}

/**
 * Routing table entry
 */
export interface RouteEntry {
  /** Destination host ID or network prefix */
  destination: HostId | string;

  /** Next hop information */
  next_hop: NextHop;

  /** Route status */
  status: RouteStatus;

  /** Route preference (higher is better) */
  preference: number;

  /** Route age in seconds */
  age_seconds: number;

  /** Route source (static, dynamic, learned) */
  source: 'static' | 'dynamic' | 'learned' | 'default';

  /** Route tags for policy-based routing */
  tags?: string[];
}

/**
 * Complete routing table
 */
export interface RoutingTable {
  /** Local host ID */
  local_host_id: HostId;

  /** Routing entries */
  routes: RouteEntry[];

  /** Default route */
  default_route?: RouteEntry;

  /** Routing protocol in use */
  protocol: 'STATIC' | 'OSPF' | 'BGP' | 'WIHP-DYNAMIC';

  /** Last update timestamp */
  updated_at: Date | string;

  /** Table version number */
  version: number;
}

// ============================================================================
// Protocol Message Types
// ============================================================================

/**
 * WIHP message types
 */
export enum MessageType {
  /** Connection handshake initiation */
  HANDSHAKE_INIT = 'HANDSHAKE_INIT',
  /** Handshake response */
  HANDSHAKE_RESPONSE = 'HANDSHAKE_RESPONSE',
  /** Handshake acknowledgment */
  HANDSHAKE_ACK = 'HANDSHAKE_ACK',
  /** Data transfer */
  DATA = 'DATA',
  /** Acknowledgment */
  ACK = 'ACK',
  /** Keep-alive ping */
  PING = 'PING',
  /** Ping response */
  PONG = 'PONG',
  /** Route advertisement */
  ROUTE_ADVERTISE = 'ROUTE_ADVERTISE',
  /** Route request */
  ROUTE_REQUEST = 'ROUTE_REQUEST',
  /** Host discovery */
  DISCOVER = 'DISCOVER',
  /** Connection termination */
  DISCONNECT = 'DISCONNECT',
  /** Error notification */
  ERROR = 'ERROR',
}

/**
 * WIHP message header
 */
export interface WIHPHeader {
  /** Protocol version */
  version: string;

  /** Message type */
  message_type: MessageType;

  /** Source host ID */
  source: HostId;

  /** Destination host ID */
  destination: HostId;

  /** Message sequence number */
  sequence: number;

  /** Message priority (0-7, higher is better) */
  priority: number;

  /** Time-to-live (hop count limit) */
  ttl: number;

  /** Payload length in bytes */
  payload_length: number;

  /** Message timestamp */
  timestamp: Date | string;

  /** Optional flags */
  flags?: string[];
}

/**
 * WIHP message payload
 */
export interface WIHPPayload {
  /** Payload data */
  data: unknown;

  /** Payload encoding */
  encoding: 'json' | 'msgpack' | 'protobuf' | 'raw';

  /** Payload compression */
  compression?: 'gzip' | 'brotli' | 'zstd' | 'none';

  /** Checksum for integrity */
  checksum: string;
}

/**
 * Complete WIHP message
 */
export interface WIHPMessage {
  /** Message header */
  header: WIHPHeader;

  /** Message payload */
  payload: WIHPPayload;

  /** Digital signature (optional) */
  signature?: string;
}

// ============================================================================
// Connection Management Types
// ============================================================================

/**
 * Connection state
 */
export enum ConnectionState {
  /** Initial state */
  IDLE = 'IDLE',
  /** Handshake in progress */
  HANDSHAKING = 'HANDSHAKING',
  /** Connection established */
  ESTABLISHED = 'ESTABLISHED',
  /** Connection active with data transfer */
  ACTIVE = 'ACTIVE',
  /** Connection closing */
  CLOSING = 'CLOSING',
  /** Connection closed */
  CLOSED = 'CLOSED',
  /** Connection failed */
  FAILED = 'FAILED',
}

/**
 * Handshake parameters
 */
export interface HandshakeParams {
  /** Supported protocol versions */
  protocol_versions: string[];

  /** Encryption algorithms supported */
  encryption_algorithms: string[];

  /** Authentication methods supported */
  auth_methods: string[];

  /** Maximum message size in bytes */
  max_message_size: number;

  /** Keep-alive interval in seconds */
  keepalive_interval: number;

  /** Connection timeout in seconds */
  timeout_seconds: number;

  /** Client certificate (optional) */
  certificate?: string;
}

/**
 * Handshake result
 */
export interface HandshakeResult {
  /** Handshake success */
  success: boolean;

  /** Negotiated protocol version */
  protocol_version: string;

  /** Selected encryption algorithm */
  encryption: string;

  /** Selected authentication method */
  auth_method: string;

  /** Session ID */
  session_id: string;

  /** Session key (encrypted) */
  session_key?: string;

  /** Error message if failed */
  error?: string;
}

/**
 * Connection instance
 */
export interface Connection {
  /** Connection ID */
  connection_id: string;

  /** Local host ID */
  local_host: HostId;

  /** Remote host ID */
  remote_host: HostId;

  /** Remote address */
  remote_address: HostAddress;

  /** Current connection state */
  state: ConnectionState;

  /** Handshake result */
  handshake?: HandshakeResult;

  /** Connection established timestamp */
  established_at?: Date | string;

  /** Last activity timestamp */
  last_activity: Date | string;

  /** Bytes sent */
  bytes_sent: number;

  /** Bytes received */
  bytes_received: number;

  /** Messages sent */
  messages_sent: number;

  /** Messages received */
  messages_received: number;

  /** Connection metadata */
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Security Types
// ============================================================================

/**
 * Certificate type
 */
export type CertificateType = 'X509' | 'WIHP' | 'PGP';

/**
 * Host certificate
 */
export interface HostCertificate {
  /** Certificate type */
  type: CertificateType;

  /** Certificate version */
  version: string;

  /** Subject host ID */
  subject: HostId;

  /** Issuer host ID */
  issuer: HostId;

  /** Certificate serial number */
  serial_number: string;

  /** Public key (PEM format) */
  public_key: string;

  /** Signature algorithm */
  signature_algorithm: string;

  /** Certificate signature */
  signature: string;

  /** Valid from timestamp */
  valid_from: Date | string;

  /** Valid until timestamp */
  valid_until: Date | string;

  /** Certificate extensions */
  extensions?: Record<string, unknown>;
}

/**
 * Trust level
 */
export type TrustLevel = 'trusted' | 'unknown' | 'untrusted' | 'revoked';

/**
 * Trust chain entry
 */
export interface TrustChainEntry {
  /** Certificate */
  certificate: HostCertificate;

  /** Trust level */
  trust_level: TrustLevel;

  /** Verification status */
  verified: boolean;

  /** Verification timestamp */
  verified_at: Date | string;
}

/**
 * Complete trust chain
 */
export interface TrustChain {
  /** Chain entries from subject to root */
  chain: TrustChainEntry[];

  /** Overall chain validity */
  valid: boolean;

  /** Trust score (0-1) */
  trust_score: number;

  /** Verification errors */
  errors?: string[];
}

/**
 * Authentication result
 */
export interface AuthenticationResult {
  /** Authentication success */
  authenticated: boolean;

  /** Host ID that was authenticated */
  host_id: HostId;

  /** Authentication method used */
  method: string;

  /** Trust chain */
  trust_chain?: TrustChain;

  /** Authentication timestamp */
  authenticated_at: Date | string;

  /** Session token */
  token?: string;

  /** Token expiry */
  token_expires_at?: Date | string;

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// Network Topology Types
// ============================================================================

/**
 * Node type in the network
 */
export type NodeType = 'edge' | 'relay' | 'gateway' | 'supernode';

/**
 * Peer information
 */
export interface PeerInfo {
  /** Peer host ID */
  host_id: HostId;

  /** Peer addresses */
  addresses: HostAddress[];

  /** Connection to peer */
  connection?: Connection;

  /** Peer metrics */
  metrics: PathMetrics;

  /** Peer reputation score (0-1) */
  reputation: number;

  /** Peer capabilities */
  capabilities: string[];

  /** Last contact timestamp */
  last_seen: Date | string;
}

/**
 * Network node
 */
export interface NetworkNode {
  /** Node host ID */
  host_id: HostId;

  /** Node type */
  node_type: NodeType;

  /** Node metadata */
  metadata: HostMetadata;

  /** Connected peers */
  peers: PeerInfo[];

  /** Node uptime in seconds */
  uptime_seconds: number;

  /** Geographic location (optional) */
  location?: {
    latitude: number;
    longitude: number;
    region?: string;
  };
}

/**
 * Network topology
 */
export interface NetworkTopology {
  /** Local node */
  local_node: NetworkNode;

  /** Known nodes in the network */
  nodes: NetworkNode[];

  /** Total network size estimate */
  estimated_network_size: number;

  /** Topology generation timestamp */
  generated_at: Date | string;

  /** Topology version */
  version: number;
}

// ============================================================================
// Quality of Service Types
// ============================================================================

/**
 * QoS priority levels
 */
export enum QoSPriority {
  /** Best effort (default) */
  BEST_EFFORT = 0,
  /** Bulk data transfer */
  BULK = 1,
  /** Standard data */
  STANDARD = 2,
  /** Low latency */
  LOW_LATENCY = 3,
  /** Real-time */
  REAL_TIME = 4,
  /** Critical */
  CRITICAL = 5,
}

/**
 * Bandwidth specification
 */
export interface Bandwidth {
  /** Minimum guaranteed bandwidth in Mbps */
  min_mbps: number;

  /** Maximum bandwidth in Mbps */
  max_mbps: number;

  /** Burst size in MB */
  burst_mb?: number;
}

/**
 * Latency requirements
 */
export interface LatencyRequirements {
  /** Maximum acceptable latency in ms */
  max_latency_ms: number;

  /** Maximum jitter in ms */
  max_jitter_ms: number;

  /** Percentile requirement (e.g., 95.0 for P95) */
  percentile?: number;
}

/**
 * QoS policy configuration
 */
export interface QoSPolicy {
  /** Policy name */
  name: string;

  /** Priority level */
  priority: QoSPriority;

  /** Bandwidth specification */
  bandwidth?: Bandwidth;

  /** Latency requirements */
  latency?: LatencyRequirements;

  /** Maximum packet loss rate (0-1) */
  max_loss_rate?: number;

  /** Traffic shaping enabled */
  traffic_shaping?: boolean;

  /** Policy enabled */
  enabled: boolean;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIHP error codes
 */
export enum WIHPErrorCode {
  /** Unknown error */
  UNKNOWN = 'WIHP_ERR_UNKNOWN',
  /** Invalid host ID */
  INVALID_HOST_ID = 'WIHP_ERR_INVALID_HOST_ID',
  /** Host not found */
  HOST_NOT_FOUND = 'WIHP_ERR_HOST_NOT_FOUND',
  /** Connection timeout */
  CONNECTION_TIMEOUT = 'WIHP_ERR_CONNECTION_TIMEOUT',
  /** Connection refused */
  CONNECTION_REFUSED = 'WIHP_ERR_CONNECTION_REFUSED',
  /** Handshake failed */
  HANDSHAKE_FAILED = 'WIHP_ERR_HANDSHAKE_FAILED',
  /** Authentication failed */
  AUTH_FAILED = 'WIHP_ERR_AUTH_FAILED',
  /** Certificate invalid */
  INVALID_CERTIFICATE = 'WIHP_ERR_INVALID_CERTIFICATE',
  /** Route not found */
  NO_ROUTE = 'WIHP_ERR_NO_ROUTE',
  /** Message too large */
  MESSAGE_TOO_LARGE = 'WIHP_ERR_MESSAGE_TOO_LARGE',
  /** Protocol version mismatch */
  VERSION_MISMATCH = 'WIHP_ERR_VERSION_MISMATCH',
  /** Network unreachable */
  NETWORK_UNREACHABLE = 'WIHP_ERR_NETWORK_UNREACHABLE',
  /** QoS violation */
  QOS_VIOLATION = 'WIHP_ERR_QOS_VIOLATION',
}

/**
 * WIHP protocol error
 */
export class WIHPError extends Error {
  constructor(
    public code: WIHPErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'WIHPError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * WIHP configuration options
 */
export interface WIHPConfig {
  /** Local host ID */
  host_id?: HostId;

  /** Listen addresses */
  listen_addresses?: HostAddress[];

  /** Bootstrap nodes */
  bootstrap_nodes?: HostId[];

  /** Enable encryption */
  encryption_enabled?: boolean;

  /** Certificate for authentication */
  certificate?: HostCertificate;

  /** Private key (PEM format) */
  private_key?: string;

  /** Routing protocol */
  routing_protocol?: 'STATIC' | 'DYNAMIC';

  /** QoS enabled */
  qos_enabled?: boolean;

  /** Connection timeout in seconds */
  connection_timeout?: number;

  /** Maximum connections */
  max_connections?: number;

  /** Keep-alive interval in seconds */
  keepalive_interval?: number;
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Host types
  HostId,
  AddressFamily,
  HostAddress,
  HostFingerprint,
  HostMetadata,

  // Routing types
  RouteStatus,
  PathMetrics,
  NextHop,
  RouteEntry,
  RoutingTable,

  // Message types
  MessageType,
  WIHPHeader,
  WIHPPayload,
  WIHPMessage,

  // Connection types
  ConnectionState,
  HandshakeParams,
  HandshakeResult,
  Connection,

  // Security types
  CertificateType,
  HostCertificate,
  TrustLevel,
  TrustChainEntry,
  TrustChain,
  AuthenticationResult,

  // Topology types
  NodeType,
  PeerInfo,
  NetworkNode,
  NetworkTopology,

  // QoS types
  QoSPriority,
  Bandwidth,
  LatencyRequirements,
  QoSPolicy,

  // Config
  WIHPConfig,
};

export { WIHPErrorCode, WIHPError };
