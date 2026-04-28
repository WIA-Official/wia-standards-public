/**
 * WIA-COMM-016: VPN Protocol - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * VPN Protocol types
 */
export type VPNProtocol =
  | 'ipsec-ikev2'
  | 'ipsec-ikev1'
  | 'openvpn'
  | 'wireguard'
  | 'l2tp-ipsec'
  | 'sstp'
  | 'pptp';

/**
 * VPN mode (site-to-site or remote access)
 */
export type VPNMode = 'site-to-site' | 'remote-access';

/**
 * VPN tunnel mode
 */
export type TunnelMode = 'tunnel' | 'transport';

/**
 * Encryption algorithms
 */
export type EncryptionAlgorithm =
  | 'AES-256-GCM'
  | 'AES-128-GCM'
  | 'AES-256-CBC'
  | 'AES-128-CBC'
  | 'ChaCha20-Poly1305'
  | '3DES';

/**
 * Authentication algorithms
 */
export type AuthenticationAlgorithm =
  | 'SHA-256'
  | 'SHA-384'
  | 'SHA-512'
  | 'SHA-1'
  | 'MD5';

/**
 * Diffie-Hellman groups
 */
export type DHGroup =
  | 'modp1024'
  | 'modp1536'
  | 'modp2048'
  | 'modp3072'
  | 'modp4096'
  | 'ecp256'
  | 'ecp384'
  | 'ecp521'
  | 'curve25519';

// ============================================================================
// IPsec Types
// ============================================================================

/**
 * IPsec configuration
 */
export interface IPsecConfig {
  /** IKE version */
  protocol: 'IKEv1' | 'IKEv2';

  /** Tunnel or transport mode */
  mode: TunnelMode;

  /** Encryption algorithm */
  encryption: EncryptionAlgorithm;

  /** Authentication algorithm */
  authentication: AuthenticationAlgorithm;

  /** Diffie-Hellman group */
  dhGroup: DHGroup;

  /** Perfect Forward Secrecy enabled */
  pfsEnabled: boolean;

  /** PFS DH group (if different from main) */
  pfsDhGroup?: DHGroup;

  /** Local identity */
  localId: string;

  /** Remote identity */
  remoteId: string;

  /** Local subnet (CIDR) */
  localSubnet?: string;

  /** Remote subnet (CIDR) */
  remoteSubnet?: string;

  /** Pre-shared key */
  psk?: string;

  /** Certificate path */
  certificate?: string;

  /** Private key path */
  privateKey?: string;

  /** CA certificate path */
  caCertificate?: string;

  /** Phase 1 lifetime (seconds) */
  phase1Lifetime?: number;

  /** Phase 2 lifetime (seconds) */
  phase2Lifetime?: number;

  /** Dead Peer Detection interval (seconds) */
  dpdInterval?: number;

  /** NAT traversal enabled */
  natTraversal?: boolean;
}

/**
 * IPsec tunnel status
 */
export interface IPsecTunnelStatus {
  /** Tunnel identifier */
  id: string;

  /** Tunnel state */
  state: 'down' | 'connecting' | 'established' | 'rekeying' | 'error';

  /** Local endpoint */
  localEndpoint: string;

  /** Remote endpoint */
  remoteEndpoint: string;

  /** Established timestamp */
  establishedAt?: Date;

  /** Last rekey timestamp */
  lastRekey?: Date;

  /** Bytes sent */
  bytesSent: number;

  /** Bytes received */
  bytesReceived: number;

  /** Packets sent */
  packetsSent: number;

  /** Packets received */
  packetsReceived: number;

  /** Current encryption */
  currentEncryption: EncryptionAlgorithm;

  /** SPI (Security Parameter Index) */
  spiIn: string;
  spiOut: string;
}

/**
 * ESP (Encapsulating Security Payload) configuration
 */
export interface ESPConfig {
  /** ESP mode */
  mode: TunnelMode;

  /** Encryption algorithm */
  encryption: EncryptionAlgorithm;

  /** Authentication algorithm */
  authentication?: AuthenticationAlgorithm;

  /** Use AEAD cipher (no separate auth) */
  useAEAD: boolean;

  /** Replay window size */
  replayWindow?: number;
}

// ============================================================================
// OpenVPN Types
// ============================================================================

/**
 * OpenVPN configuration
 */
export interface OpenVPNConfig {
  /** Server or client mode */
  mode: 'server' | 'client';

  /** Protocol (UDP or TCP) */
  protocol: 'udp' | 'tcp';

  /** Port */
  port: number;

  /** Device type */
  device: 'tun' | 'tap';

  /** Server address (for client) */
  serverAddress?: string;

  /** Server network */
  serverNetwork?: string;

  /** Server netmask */
  serverNetmask?: string;

  /** Cipher */
  cipher: EncryptionAlgorithm;

  /** Authentication digest */
  auth: AuthenticationAlgorithm;

  /** TLS version minimum */
  tlsVersionMin: '1.2' | '1.3';

  /** CA certificate */
  caCert: string;

  /** Server/Client certificate */
  cert: string;

  /** Private key */
  key: string;

  /** Diffie-Hellman parameters */
  dhParams?: string;

  /** TLS auth key */
  tlsAuth?: string;

  /** TLS crypt key */
  tlsCrypt?: string;

  /** Compression */
  compression?: 'lzo' | 'lz4' | 'none';

  /** Routes to push (server mode) */
  routes?: string[];

  /** DNS servers to push */
  dnsServers?: string[];

  /** Redirect gateway */
  redirectGateway?: boolean;

  /** Keep-alive ping */
  keepalive?: {
    interval: number;
    timeout: number;
  };

  /** Verbosity level */
  verbosity?: number;
}

/**
 * OpenVPN connection status
 */
export interface OpenVPNStatus {
  /** Connection state */
  state: 'disconnected' | 'connecting' | 'connected' | 'reconnecting' | 'error';

  /** Connected clients (server mode) */
  clients?: OpenVPNClient[];

  /** Connection established timestamp */
  connectedAt?: Date;

  /** Virtual IP address */
  virtualIP?: string;

  /** Bytes sent */
  bytesSent: number;

  /** Bytes received */
  bytesReceived: number;

  /** Server address */
  serverAddress?: string;

  /** TLS cipher */
  tlsCipher?: string;

  /** Data cipher */
  dataCipher?: string;
}

/**
 * OpenVPN client (server mode)
 */
export interface OpenVPNClient {
  /** Client common name */
  commonName: string;

  /** Real IP address */
  realAddress: string;

  /** Virtual IP address */
  virtualAddress: string;

  /** Connected since */
  connectedSince: Date;

  /** Bytes sent to client */
  bytesSent: number;

  /** Bytes received from client */
  bytesReceived: number;
}

// ============================================================================
// WireGuard Types
// ============================================================================

/**
 * WireGuard configuration
 */
export interface WireGuardConfig {
  /** Interface name */
  interfaceName?: string;

  /** Private key (base64) */
  privateKey: string;

  /** Public key (derived from private) */
  publicKey?: string;

  /** Listen port */
  listenPort: number;

  /** Interface addresses (CIDR) */
  addresses: string[];

  /** DNS servers */
  dns?: string[];

  /** MTU */
  mtu?: number;

  /** Firewall mark */
  fwmark?: number;

  /** Peers */
  peers: WireGuardPeer[];

  /** Post-up commands */
  postUp?: string[];

  /** Post-down commands */
  postDown?: string[];
}

/**
 * WireGuard peer configuration
 */
export interface WireGuardPeer {
  /** Peer public key */
  publicKey: string;

  /** Pre-shared key (optional) */
  presharedKey?: string;

  /** Endpoint (IP:port) */
  endpoint?: string;

  /** Allowed IPs (CIDR) */
  allowedIPs: string[];

  /** Persistent keepalive (seconds) */
  persistentKeepalive?: number;
}

/**
 * WireGuard interface status
 */
export interface WireGuardStatus {
  /** Interface name */
  interfaceName: string;

  /** Public key */
  publicKey: string;

  /** Listen port */
  listenPort: number;

  /** Firewall mark */
  fwmark?: number;

  /** Peers */
  peers: WireGuardPeerStatus[];
}

/**
 * WireGuard peer status
 */
export interface WireGuardPeerStatus {
  /** Peer public key */
  publicKey: string;

  /** Endpoint */
  endpoint?: string;

  /** Allowed IPs */
  allowedIPs: string[];

  /** Latest handshake */
  latestHandshake?: Date;

  /** Transfer */
  transfer: {
    sent: number;
    received: number;
  };

  /** Persistent keepalive */
  persistentKeepalive?: number;
}

/**
 * WireGuard key pair
 */
export interface WireGuardKeyPair {
  /** Private key (base64) */
  privateKey: string;

  /** Public key (base64) */
  publicKey: string;

  /** Pre-shared key (base64, optional) */
  presharedKey?: string;
}

// ============================================================================
// L2TP/IPsec Types
// ============================================================================

/**
 * L2TP/IPsec configuration
 */
export interface L2TPConfig {
  /** IPsec configuration */
  ipsec: IPsecConfig;

  /** L2TP settings */
  l2tp: {
    /** L2TP server address */
    server: string;

    /** L2TP port */
    port?: number;

    /** Authentication method */
    authMethod: 'pap' | 'chap' | 'mschap' | 'mschapv2';

    /** Username */
    username: string;

    /** Password */
    password: string;

    /** Compression enabled */
    compression?: boolean;
  };
}

// ============================================================================
// SSL/TLS VPN Types
// ============================================================================

/**
 * SSL VPN configuration
 */
export interface SSLVPNConfig {
  /** VPN type */
  type: 'portal' | 'tunnel';

  /** Server address */
  serverAddress: string;

  /** Port */
  port: number;

  /** TLS version */
  tlsVersion: '1.2' | '1.3';

  /** Certificate validation */
  validateCert: boolean;

  /** Client certificate */
  clientCert?: string;

  /** Client key */
  clientKey?: string;

  /** CA certificate */
  caCert?: string;

  /** Username/password authentication */
  credentials?: {
    username: string;
    password: string;
  };

  /** Two-factor authentication */
  twoFactor?: {
    enabled: boolean;
    method: 'totp' | 'sms' | 'push';
  };
}

// ============================================================================
// VPN Connection Types
// ============================================================================

/**
 * VPN connection request
 */
export interface VPNConnectionRequest {
  /** VPN protocol */
  protocol: VPNProtocol;

  /** VPN mode */
  mode: VPNMode;

  /** Configuration (protocol-specific) */
  config:
    | IPsecConfig
    | OpenVPNConfig
    | WireGuardConfig
    | L2TPConfig
    | SSLVPNConfig;

  /** Connection name */
  name?: string;

  /** Auto-connect on startup */
  autoConnect?: boolean;

  /** Connection timeout (seconds) */
  timeout?: number;
}

/**
 * VPN connection response
 */
export interface VPNConnectionResponse {
  /** Connection ID */
  id: string;

  /** Connection state */
  state: 'connected' | 'disconnected' | 'connecting' | 'error';

  /** Virtual IP address assigned */
  virtualIP?: string;

  /** Gateway IP */
  gatewayIP?: string;

  /** DNS servers */
  dnsServers?: string[];

  /** Routes */
  routes?: string[];

  /** Error message (if state is error) */
  error?: string;

  /** Connection details */
  details?: IPsecTunnelStatus | OpenVPNStatus | WireGuardStatus;
}

// ============================================================================
// VPN Metrics Types
// ============================================================================

/**
 * VPN performance metrics
 */
export interface VPNMetrics {
  /** Connection ID */
  connectionId: string;

  /** Protocol */
  protocol: VPNProtocol;

  /** Uptime (seconds) */
  uptime: number;

  /** Throughput */
  throughput: {
    /** Upload speed (bytes/sec) */
    upload: number;

    /** Download speed (bytes/sec) */
    download: number;

    /** Peak upload (bytes/sec) */
    peakUpload: number;

    /** Peak download (bytes/sec) */
    peakDownload: number;
  };

  /** Total transfer */
  transfer: {
    /** Bytes sent */
    sent: number;

    /** Bytes received */
    received: number;

    /** Packets sent */
    packetsSent: number;

    /** Packets received */
    packetsReceived: number;
  };

  /** Latency */
  latency: {
    /** Current latency (ms) */
    current: number;

    /** Average latency (ms) */
    average: number;

    /** Minimum latency (ms) */
    min: number;

    /** Maximum latency (ms) */
    max: number;
  };

  /** Packet loss (%) */
  packetLoss: number;

  /** Jitter (ms) */
  jitter: number;
}

/**
 * Tunnel overhead calculation
 */
export interface TunnelOverhead {
  /** VPN protocol */
  protocol: VPNProtocol;

  /** Original MTU */
  originalMTU: number;

  /** VPN overhead (bytes) */
  overhead: number;

  /** Effective MTU */
  effectiveMTU: number;

  /** MSS (Maximum Segment Size) */
  mss: number;

  /** Fragmentation risk */
  fragmentationRisk: 'low' | 'medium' | 'high';

  /** Recommended MTU */
  recommendedMTU: number;
}

// ============================================================================
// VPN Security Types
// ============================================================================

/**
 * VPN security audit result
 */
export interface VPNSecurityAudit {
  /** Connection ID */
  connectionId: string;

  /** Audit timestamp */
  timestamp: Date;

  /** Security level */
  securityLevel: 'excellent' | 'good' | 'acceptable' | 'weak' | 'vulnerable';

  /** Findings */
  findings: VPNSecurityFinding[];

  /** Score (0-100) */
  score: number;

  /** Recommendations */
  recommendations: string[];
}

/**
 * Security finding
 */
export interface VPNSecurityFinding {
  /** Severity */
  severity: 'critical' | 'high' | 'medium' | 'low' | 'info';

  /** Category */
  category: 'encryption' | 'authentication' | 'configuration' | 'protocol';

  /** Title */
  title: string;

  /** Description */
  description: string;

  /** Remediation */
  remediation?: string;
}

/**
 * Certificate information
 */
export interface CertificateInfo {
  /** Subject */
  subject: string;

  /** Issuer */
  issuer: string;

  /** Valid from */
  validFrom: Date;

  /** Valid to */
  validTo: Date;

  /** Serial number */
  serialNumber: string;

  /** Fingerprint (SHA-256) */
  fingerprint: string;

  /** Key algorithm */
  keyAlgorithm: string;

  /** Key size (bits) */
  keySize: number;

  /** Signature algorithm */
  signatureAlgorithm: string;

  /** Is expired */
  isExpired: boolean;

  /** Days until expiry */
  daysUntilExpiry: number;
}

// ============================================================================
// Split Tunneling Types
// ============================================================================

/**
 * Split tunneling configuration
 */
export interface SplitTunnelingConfig {
  /** Split tunneling enabled */
  enabled: boolean;

  /** Mode */
  mode: 'include' | 'exclude';

  /** Routes/subnets to include or exclude */
  routes: string[];

  /** DNS handling */
  dns: 'tunnel-all' | 'tunnel-split' | 'local';

  /** Application-based split tunneling */
  applications?: {
    /** Application path or name */
    app: string;

    /** Route through VPN */
    useTunnel: boolean;
  }[];
}

// ============================================================================
// VPN Concentrator Types
// ============================================================================

/**
 * VPN concentrator configuration
 */
export interface VPNConcentratorConfig {
  /** Concentrator ID */
  id: string;

  /** Supported protocols */
  supportedProtocols: VPNProtocol[];

  /** Maximum concurrent sessions */
  maxSessions: number;

  /** Session timeout (seconds) */
  sessionTimeout: number;

  /** Idle timeout (seconds) */
  idleTimeout: number;

  /** Load balancing */
  loadBalancing?: {
    enabled: boolean;
    algorithm: 'round-robin' | 'least-connections' | 'source-ip';
  };

  /** High availability */
  highAvailability?: {
    enabled: boolean;
    mode: 'active-passive' | 'active-active';
    peerAddress?: string;
  };
}

/**
 * VPN concentrator status
 */
export interface VPNConcentratorStatus {
  /** Concentrator ID */
  id: string;

  /** Status */
  status: 'online' | 'offline' | 'degraded';

  /** Active sessions */
  activeSessions: number;

  /** CPU usage (%) */
  cpuUsage: number;

  /** Memory usage (%) */
  memoryUsage: number;

  /** Network throughput */
  throughput: {
    in: number; // bytes/sec
    out: number; // bytes/sec
  };

  /** Session statistics */
  sessionStats: {
    total: number;
    active: number;
    idle: number;
    failed: number;
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * VPN error codes
 */
export enum VPNErrorCode {
  INVALID_CONFIG = 'VPN001',
  AUTH_FAILED = 'VPN002',
  CONNECTION_TIMEOUT = 'VPN003',
  TUNNEL_FAILED = 'VPN004',
  CERTIFICATE_INVALID = 'VPN005',
  KEY_EXCHANGE_FAILED = 'VPN006',
  ENCRYPTION_FAILED = 'VPN007',
  PROTOCOL_ERROR = 'VPN008',
  NETWORK_UNREACHABLE = 'VPN009',
  UNSUPPORTED_PROTOCOL = 'VPN010',
}

/**
 * VPN error
 */
export class VPNError extends Error {
  constructor(
    public code: VPNErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'VPNError';
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

// ============================================================================
// Constants
// ============================================================================

/**
 * Protocol overhead sizes (bytes)
 */
export const PROTOCOL_OVERHEAD = {
  'ipsec-ikev2': 52,
  'ipsec-ikev1': 52,
  openvpn: 70,
  wireguard: 60,
  'l2tp-ipsec': 76,
  sstp: 40,
  pptp: 16,
} as const;

/**
 * Default ports
 */
export const DEFAULT_PORTS = {
  'ipsec-ike': 500,
  'ipsec-nat-t': 4500,
  openvpn: 1194,
  wireguard: 51820,
  l2tp: 1701,
  sstp: 443,
  pptp: 1723,
} as const;

/**
 * Recommended MTU sizes
 */
export const RECOMMENDED_MTU = {
  'ipsec-ikev2': 1400,
  'ipsec-ikev1': 1400,
  openvpn: 1420,
  wireguard: 1420,
  'l2tp-ipsec': 1380,
  sstp: 1460,
  pptp: 1460,
} as const;

// ============================================================================
// Export All
// ============================================================================

export type {
  VPNProtocol,
  VPNMode,
  TunnelMode,
  EncryptionAlgorithm,
  AuthenticationAlgorithm,
  DHGroup,
  IPsecConfig,
  IPsecTunnelStatus,
  ESPConfig,
  OpenVPNConfig,
  OpenVPNStatus,
  OpenVPNClient,
  WireGuardConfig,
  WireGuardPeer,
  WireGuardStatus,
  WireGuardPeerStatus,
  WireGuardKeyPair,
  L2TPConfig,
  SSLVPNConfig,
  VPNConnectionRequest,
  VPNConnectionResponse,
  VPNMetrics,
  TunnelOverhead,
  VPNSecurityAudit,
  VPNSecurityFinding,
  CertificateInfo,
  SplitTunnelingConfig,
  VPNConcentratorConfig,
  VPNConcentratorStatus,
};

export { VPNErrorCode, VPNError, PROTOCOL_OVERHEAD, DEFAULT_PORTS, RECOMMENDED_MTU };
