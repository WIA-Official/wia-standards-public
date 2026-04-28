/**
 * WIA-TLS-LITE Types
 * Lightweight Transport Layer Security for IoT and Constrained Devices
 *
 * @packageDocumentation
 */

/**
 * TLS Protocol Versions
 * Focused on TLS 1.2 and 1.3 for modern security with lightweight options
 */
export enum TLSVersion {
  /** TLS 1.2 - Widely supported, good balance of security and compatibility */
  TLS_1_2 = '1.2',
  /** TLS 1.3 - Latest standard, improved performance and security */
  TLS_1_3 = '1.3',
  /** TLS 1.2 Lite - Reduced cipher suite for constrained devices */
  TLS_1_2_LITE = '1.2-lite',
  /** TLS 1.3 Lite - Optimized for IoT with minimal overhead */
  TLS_1_3_LITE = '1.3-lite'
}

/**
 * Cipher Suites optimized for IoT devices
 * Prioritizes lightweight cryptography with strong security
 */
export enum CipherSuite {
  // TLS 1.3 Cipher Suites (AEAD only)
  /** AES-128-GCM with SHA256 - Best balance for IoT */
  TLS_AES_128_GCM_SHA256 = 'TLS_AES_128_GCM_SHA256',
  /** AES-256-GCM with SHA384 - Higher security, more resource intensive */
  TLS_AES_256_GCM_SHA384 = 'TLS_AES_256_GCM_SHA384',
  /** ChaCha20-Poly1305 with SHA256 - Excellent for software-only devices */
  TLS_CHACHA20_POLY1305_SHA256 = 'TLS_CHACHA20_POLY1305_SHA256',

  // TLS 1.2 Cipher Suites
  /** ECDHE-RSA with AES-128-GCM - Widely supported */
  TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256 = 'TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256',
  /** ECDHE-ECDSA with AES-128-GCM - Lower compute for ECDSA */
  TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256 = 'TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256',
  /** ECDHE-ECDSA with ChaCha20-Poly1305 - Best for constrained devices */
  TLS_ECDHE_ECDSA_WITH_CHACHA20_POLY1305_SHA256 = 'TLS_ECDHE_ECDSA_WITH_CHACHA20_POLY1305_SHA256',

  // Pre-Shared Key (PSK) for ultra-lightweight scenarios
  /** PSK with AES-128-GCM - Minimal handshake overhead */
  TLS_PSK_WITH_AES_128_GCM_SHA256 = 'TLS_PSK_WITH_AES_128_GCM_SHA256',
  /** PSK with ChaCha20-Poly1305 - Ultra-lightweight */
  TLS_PSK_WITH_CHACHA20_POLY1305_SHA256 = 'TLS_PSK_WITH_CHACHA20_POLY1305_SHA256'
}

/**
 * Key Exchange Algorithms
 */
export enum KeyExchange {
  /** Elliptic Curve Diffie-Hellman Ephemeral - Standard forward secrecy */
  ECDHE = 'ECDHE',
  /** Pre-Shared Key - No certificate overhead */
  PSK = 'PSK',
  /** X25519 - Modern, efficient curve (TLS 1.3) */
  X25519 = 'X25519',
  /** secp256r1 (P-256) - Widely supported NIST curve */
  SECP256R1 = 'secp256r1',
  /** secp384r1 (P-384) - Higher security NIST curve */
  SECP384R1 = 'secp384r1'
}

/**
 * Certificate Types for IoT
 */
export enum CertificateType {
  /** X.509 certificates - Standard PKI */
  X509 = 'X.509',
  /** Raw Public Key - No certificate overhead */
  RAW_PUBLIC_KEY = 'RawPublicKey',
  /** Pre-Shared Key identity */
  PSK = 'PSK',
  /** Compact X.509 - Reduced size for IoT */
  COMPACT_X509 = 'CompactX509'
}

/**
 * Certificate format and encoding
 */
export interface Certificate {
  /** Certificate type */
  type: CertificateType;
  /** Raw certificate data */
  data: Buffer | Uint8Array;
  /** Subject information */
  subject?: string;
  /** Issuer information */
  issuer?: string;
  /** Validity period start */
  notBefore?: Date;
  /** Validity period end */
  notAfter?: Date;
  /** Public key */
  publicKey?: Buffer | Uint8Array;
  /** Certificate fingerprint */
  fingerprint?: string;
}

/**
 * Handshake State
 */
export enum HandshakeState {
  /** Initial state */
  IDLE = 'IDLE',
  /** Client sending ClientHello */
  CLIENT_HELLO_SENT = 'CLIENT_HELLO_SENT',
  /** Server sending ServerHello */
  SERVER_HELLO_SENT = 'SERVER_HELLO_SENT',
  /** Certificate exchange in progress */
  CERTIFICATE_EXCHANGE = 'CERTIFICATE_EXCHANGE',
  /** Key exchange in progress */
  KEY_EXCHANGE = 'KEY_EXCHANGE',
  /** Handshake completed successfully */
  COMPLETED = 'COMPLETED',
  /** Handshake failed */
  FAILED = 'FAILED'
}

/**
 * TLS Handshake Configuration
 */
export interface HandshakeConfig {
  /** Preferred TLS version */
  version: TLSVersion;
  /** Allowed cipher suites in preference order */
  cipherSuites: CipherSuite[];
  /** Server Name Indication */
  serverName?: string;
  /** Application-Layer Protocol Negotiation */
  alpnProtocols?: string[];
  /** Client certificates */
  clientCertificates?: Certificate[];
  /** Pre-shared key */
  psk?: {
    identity: string;
    key: Buffer | Uint8Array;
  };
  /** Session resumption ticket */
  sessionTicket?: Buffer | Uint8Array;
  /** Timeout for handshake (ms) */
  timeout?: number;
  /** Enable session resumption */
  enableSessionResumption?: boolean;
  /** Enable 0-RTT (TLS 1.3 only) */
  enable0RTT?: boolean;
}

/**
 * TLS Session Information
 */
export interface TLSSession {
  /** Session ID */
  id: string;
  /** Negotiated TLS version */
  version: TLSVersion;
  /** Negotiated cipher suite */
  cipherSuite: CipherSuite;
  /** Server name */
  serverName?: string;
  /** Negotiated ALPN protocol */
  alpnProtocol?: string;
  /** Peer certificate */
  peerCertificate?: Certificate;
  /** Session creation time */
  createdAt: Date;
  /** Session last used time */
  lastUsedAt: Date;
  /** Session expiry time */
  expiresAt: Date;
  /** Session resumption ticket */
  ticket?: Buffer | Uint8Array;
  /** Session master secret (encrypted) */
  masterSecret?: Buffer | Uint8Array;
  /** Session is resumable */
  resumable: boolean;
}

/**
 * ALPN (Application-Layer Protocol Negotiation) Protocols
 */
export enum ALPNProtocol {
  /** HTTP/1.1 */
  HTTP_1_1 = 'http/1.1',
  /** HTTP/2 */
  HTTP_2 = 'h2',
  /** HTTP/3 */
  HTTP_3 = 'h3',
  /** MQTT */
  MQTT = 'mqtt',
  /** CoAP */
  COAP = 'coap',
  /** Custom IoT protocol */
  IOT_CUSTOM = 'iot-custom'
}

/**
 * SNI (Server Name Indication) Configuration
 */
export interface SNIConfig {
  /** Server hostname */
  hostname: string;
  /** Certificate for this hostname */
  certificate?: Certificate;
  /** Private key for this hostname */
  privateKey?: Buffer | Uint8Array;
}

/**
 * TLS Event Types
 */
export enum TLSEventType {
  /** Handshake started */
  HANDSHAKE_START = 'handshake:start',
  /** Handshake completed */
  HANDSHAKE_COMPLETE = 'handshake:complete',
  /** Handshake failed */
  HANDSHAKE_FAILED = 'handshake:failed',
  /** Session created */
  SESSION_CREATED = 'session:created',
  /** Session resumed */
  SESSION_RESUMED = 'session:resumed',
  /** Session expired */
  SESSION_EXPIRED = 'session:expired',
  /** Secure data received */
  DATA_RECEIVED = 'data:received',
  /** Secure data sent */
  DATA_SENT = 'data:sent',
  /** Connection closed */
  CONNECTION_CLOSED = 'connection:closed',
  /** Error occurred */
  ERROR = 'error'
}

/**
 * TLS Event
 */
export interface TLSEvent {
  /** Event type */
  type: TLSEventType;
  /** Event timestamp */
  timestamp: Date;
  /** Session ID if applicable */
  sessionId?: string;
  /** Event data */
  data?: any;
  /** Error if applicable */
  error?: Error;
}

/**
 * Certificate Validation Options
 */
export interface CertificateValidationOptions {
  /** Trust anchors (CA certificates) */
  trustAnchors?: Certificate[];
  /** Allow self-signed certificates */
  allowSelfSigned?: boolean;
  /** Check certificate revocation */
  checkRevocation?: boolean;
  /** Custom validation function */
  customValidator?: (cert: Certificate) => Promise<boolean>;
  /** Skip hostname verification */
  skipHostnameVerification?: boolean;
}

/**
 * Secure Channel Configuration
 */
export interface SecureChannelConfig {
  /** Session information */
  session: TLSSession;
  /** Maximum record size (bytes) */
  maxRecordSize?: number;
  /** Enable compression (not recommended) */
  enableCompression?: boolean;
  /** Heartbeat interval (ms) for keep-alive */
  heartbeatInterval?: number;
}

/**
 * TLS Statistics
 */
export interface TLSStats {
  /** Total handshakes performed */
  totalHandshakes: number;
  /** Successful handshakes */
  successfulHandshakes: number;
  /** Failed handshakes */
  failedHandshakes: number;
  /** Session resumptions */
  sessionResumptions: number;
  /** Bytes sent */
  bytesSent: number;
  /** Bytes received */
  bytesReceived: number;
  /** Average handshake time (ms) */
  avgHandshakeTime: number;
  /** Active sessions */
  activeSessions: number;
}
