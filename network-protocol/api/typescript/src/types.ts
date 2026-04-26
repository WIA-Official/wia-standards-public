/**
 * WIA-COMM-020: Network Protocol - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// OSI Layer Types
// ============================================================================

export type OSILayer = 1 | 2 | 3 | 4 | 5 | 6 | 7;

export interface OSILayerInfo {
  layer: OSILayer;
  name: string;
  pdu: string;
  protocols: string[];
  description: string;
}

// ============================================================================
// IP Address Types
// ============================================================================

/**
 * IPv4 address configuration
 */
export interface IPv4Address {
  /** IP address in dotted decimal notation */
  address: string;

  /** Network address */
  network: string;

  /** Broadcast address */
  broadcast: string;

  /** Subnet mask */
  subnetMask: string;

  /** CIDR prefix length */
  cidr: number;

  /** Number of usable host addresses */
  hostCount: number;

  /** First usable host address */
  firstHost: string;

  /** Last usable host address */
  lastHost: string;

  /** Is this a private IP address? */
  isPrivate: boolean;

  /** Address class (A, B, C, D, E) */
  class: 'A' | 'B' | 'C' | 'D' | 'E';
}

/**
 * IPv6 address configuration
 */
export interface IPv6Address {
  /** Original address */
  address: string;

  /** Canonical form (no compression) */
  canonical: string;

  /** Compressed form (with ::) */
  compressed: string;

  /** Network prefix */
  prefix: string;

  /** Prefix length */
  prefixLength: number;

  /** Interface identifier */
  interfaceID: string;

  /** Address scope */
  scope: 'global' | 'link-local' | 'unique-local' | 'multicast' | 'loopback';

  /** Address type */
  type: 'unicast' | 'multicast' | 'anycast';

  /** Is this an IPv4-mapped address? */
  isIPv4Mapped: boolean;
}

// ============================================================================
// TCP Types
// ============================================================================

/**
 * TCP flags
 */
export interface TCPFlags {
  URG?: boolean; // Urgent
  ACK?: boolean; // Acknowledgment
  PSH?: boolean; // Push
  RST?: boolean; // Reset
  SYN?: boolean; // Synchronize
  FIN?: boolean; // Finish
}

/**
 * TCP connection state
 */
export type TCPState =
  | 'LISTEN'
  | 'SYN_SENT'
  | 'SYN_RECEIVED'
  | 'ESTABLISHED'
  | 'FIN_WAIT_1'
  | 'FIN_WAIT_2'
  | 'CLOSE_WAIT'
  | 'CLOSING'
  | 'LAST_ACK'
  | 'TIME_WAIT'
  | 'CLOSED';

/**
 * TCP segment configuration
 */
export interface TCPSegment {
  /** Source port (0-65535) */
  sourcePort: number;

  /** Destination port (0-65535) */
  destPort: number;

  /** Sequence number */
  sequenceNumber: number;

  /** Acknowledgment number */
  ackNumber: number;

  /** Data offset (header length in 32-bit words) */
  dataOffset: number;

  /** TCP flags */
  flags: TCPFlags;

  /** Window size (flow control) */
  windowSize: number;

  /** Checksum */
  checksum: number;

  /** Urgent pointer */
  urgentPointer: number;

  /** Options */
  options?: Buffer;

  /** Payload data */
  data: Buffer;
}

/**
 * TCP connection configuration
 */
export interface TCPConnection {
  /** Connection ID */
  id: string;

  /** Source IP address */
  sourceIP: string;

  /** Destination IP address */
  destinationIP: string;

  /** Source port */
  sourcePort: number;

  /** Destination port */
  destinationPort: number;

  /** Connection state */
  state: TCPState;

  /** Send sequence number */
  sendSeq: number;

  /** Receive sequence number */
  receiveSeq: number;

  /** Send window size */
  sendWindow: number;

  /** Receive window size */
  receiveWindow: number;

  /** Connection created timestamp */
  created: Date;

  /** Last activity timestamp */
  lastActivity: Date;
}

// ============================================================================
// UDP Types
// ============================================================================

/**
 * UDP datagram
 */
export interface UDPDatagram {
  /** Source port (0-65535) */
  sourcePort: number;

  /** Destination port (0-65535) */
  destPort: number;

  /** Length (header + data) */
  length: number;

  /** Checksum */
  checksum: number;

  /** Payload data */
  data: Buffer;
}

// ============================================================================
// IP Packet Types
// ============================================================================

/**
 * IPv4 packet
 */
export interface IPv4Packet {
  /** Version (4) */
  version: number;

  /** Internet Header Length (5-15) */
  ihl: number;

  /** Type of Service / DSCP */
  tos: number;

  /** Total length */
  totalLength: number;

  /** Identification */
  identification: number;

  /** Flags (DF, MF) */
  flags: {
    DF?: boolean; // Don't Fragment
    MF?: boolean; // More Fragments
  };

  /** Fragment offset */
  fragmentOffset: number;

  /** Time to Live */
  ttl: number;

  /** Protocol (TCP=6, UDP=17, ICMP=1) */
  protocol: number;

  /** Header checksum */
  checksum: number;

  /** Source IP address */
  sourceIP: string;

  /** Destination IP address */
  destIP: string;

  /** Options */
  options?: Buffer;

  /** Payload data */
  data: Buffer;
}

/**
 * IPv6 packet
 */
export interface IPv6Packet {
  /** Version (6) */
  version: number;

  /** Traffic class */
  trafficClass: number;

  /** Flow label */
  flowLabel: number;

  /** Payload length */
  payloadLength: number;

  /** Next header (protocol) */
  nextHeader: number;

  /** Hop limit (TTL equivalent) */
  hopLimit: number;

  /** Source IPv6 address */
  sourceIP: string;

  /** Destination IPv6 address */
  destIP: string;

  /** Extension headers */
  extensionHeaders?: Buffer[];

  /** Payload data */
  data: Buffer;
}

// ============================================================================
// Routing Protocol Types
// ============================================================================

/**
 * BGP configuration
 */
export interface BGPConfig {
  /** Autonomous System Number */
  asn: number;

  /** Router ID */
  routerID: string;

  /** BGP peers */
  peers: BGPPeer[];

  /** Local preference */
  localPreference?: number;

  /** Multi-Exit Discriminator */
  med?: number;
}

/**
 * BGP peer
 */
export interface BGPPeer {
  /** Peer IP address */
  peerIP: string;

  /** Remote AS number */
  remoteASN: number;

  /** Peer type (eBGP or iBGP) */
  type: 'eBGP' | 'iBGP';

  /** State */
  state: 'Idle' | 'Connect' | 'Active' | 'OpenSent' | 'OpenConfirm' | 'Established';

  /** Hold time (seconds) */
  holdTime?: number;

  /** Keepalive interval (seconds) */
  keepalive?: number;
}

/**
 * OSPF configuration
 */
export interface OSPFConfig {
  /** Router ID */
  routerID: string;

  /** OSPF areas */
  areas: OSPFArea[];

  /** Networks to advertise */
  networks: OSPFNetwork[];

  /** Hello interval (seconds) */
  helloInterval?: number;

  /** Dead interval (seconds) */
  deadInterval?: number;
}

/**
 * OSPF area
 */
export interface OSPFArea {
  /** Area ID (e.g., 0.0.0.0) */
  id: string;

  /** Area type */
  type: 'backbone' | 'standard' | 'stub' | 'totally-stub' | 'nssa';

  /** Interfaces in this area */
  interfaces?: string[];
}

/**
 * OSPF network
 */
export interface OSPFNetwork {
  /** Network address with CIDR */
  network: string;

  /** Area ID */
  area: string;

  /** Cost metric */
  cost?: number;
}

/**
 * Routing table entry
 */
export interface RouteEntry {
  /** Destination network */
  destination: string;

  /** Next hop IP */
  nextHop: string;

  /** Outgoing interface */
  interface: string;

  /** Metric/cost */
  metric: number;

  /** Protocol (static, OSPF, BGP, etc.) */
  protocol: string;

  /** Administrative distance */
  adminDistance: number;

  /** Route age (seconds) */
  age?: number;
}

// ============================================================================
// Application Protocol Types
// ============================================================================

/**
 * HTTP/2 request
 */
export interface HTTP2Request {
  /** HTTP method */
  method: 'GET' | 'POST' | 'PUT' | 'DELETE' | 'PATCH' | 'HEAD' | 'OPTIONS';

  /** Request path */
  path: string;

  /** HTTP headers */
  headers: Record<string, string>;

  /** Request body */
  body?: Buffer | string;

  /** Stream priority */
  priority?: {
    weight: number;
    dependency: number;
    exclusive?: boolean;
  };

  /** Stream ID */
  streamID?: number;
}

/**
 * HTTP/2 response
 */
export interface HTTP2Response {
  /** Status code */
  status: number;

  /** HTTP headers */
  headers: Record<string, string>;

  /** Response body */
  body?: Buffer | string;

  /** Stream ID */
  streamID?: number;
}

/**
 * HTTP/3 request (over QUIC)
 */
export interface HTTP3Request {
  /** HTTP method */
  method: 'GET' | 'POST' | 'PUT' | 'DELETE' | 'PATCH' | 'HEAD' | 'OPTIONS';

  /** Full URL */
  url: string;

  /** HTTP headers */
  headers: Record<string, string>;

  /** Request body */
  body?: Buffer | string;

  /** QUIC stream ID */
  streamID?: number;
}

/**
 * DNS query
 */
export interface DNSQuery {
  /** Domain name */
  name: string;

  /** Query type (A, AAAA, CNAME, MX, etc.) */
  type: 'A' | 'AAAA' | 'CNAME' | 'MX' | 'NS' | 'TXT' | 'SOA' | 'PTR';

  /** Query class (usually IN for Internet) */
  class?: 'IN' | 'CS' | 'CH' | 'HS';

  /** Recursion desired */
  recursionDesired?: boolean;
}

/**
 * DNS response
 */
export interface DNSResponse {
  /** Query that was answered */
  query: DNSQuery;

  /** Answer records */
  answers: DNSRecord[];

  /** Authority records */
  authority?: DNSRecord[];

  /** Additional records */
  additional?: DNSRecord[];

  /** Response code */
  rcode: 'NOERROR' | 'FORMERR' | 'SERVFAIL' | 'NXDOMAIN' | 'NOTIMP' | 'REFUSED';
}

/**
 * DNS record
 */
export interface DNSRecord {
  /** Record name */
  name: string;

  /** Record type */
  type: string;

  /** Record class */
  class: string;

  /** Time to live (seconds) */
  ttl: number;

  /** Record data */
  data: string;
}

/**
 * gRPC service definition
 */
export interface GRPCService {
  /** Service name */
  name: string;

  /** Service methods */
  methods: GRPCMethod[];

  /** Package name */
  package?: string;
}

/**
 * gRPC method
 */
export interface GRPCMethod {
  /** Method name */
  name: string;

  /** Input message type */
  inputType: string;

  /** Output message type */
  outputType: string;

  /** Is client streaming? */
  clientStreaming: boolean;

  /** Is server streaming? */
  serverStreaming: boolean;
}

// ============================================================================
// Protocol Analysis Types
// ============================================================================

/**
 * Packet capture (similar to pcap)
 */
export interface PacketCapture {
  /** Capture ID */
  id: string;

  /** Timestamp */
  timestamp: Date;

  /** Captured packets */
  packets: CapturedPacket[];

  /** Capture duration (ms) */
  duration: number;

  /** Total bytes captured */
  totalBytes: number;
}

/**
 * Captured packet
 */
export interface CapturedPacket {
  /** Packet number */
  number: number;

  /** Timestamp */
  timestamp: Date;

  /** Packet length */
  length: number;

  /** Protocol stack */
  protocols: string[];

  /** Source address */
  source: string;

  /** Destination address */
  destination: string;

  /** Raw data */
  data: Buffer;

  /** Parsed layers */
  layers?: {
    ethernet?: any;
    ip?: IPv4Packet | IPv6Packet;
    transport?: TCPSegment | UDPDatagram;
    application?: any;
  };
}

/**
 * Network statistics
 */
export interface NetworkStats {
  /** Total packets */
  totalPackets: number;

  /** Total bytes */
  totalBytes: number;

  /** Packets per second */
  packetsPerSecond: number;

  /** Bytes per second */
  bytesPerSecond: number;

  /** Protocol distribution */
  protocolDistribution: Record<string, number>;

  /** Top talkers (source IPs) */
  topSources: Array<{ ip: string; packets: number; bytes: number }>;

  /** Top destinations */
  topDestinations: Array<{ ip: string; packets: number; bytes: number }>;

  /** Error count */
  errors: number;
}

// ============================================================================
// Validation and Error Types
// ============================================================================

/**
 * Packet validation result
 */
export interface PacketValidation {
  /** Is packet valid? */
  isValid: boolean;

  /** Validation errors */
  errors: string[];

  /** Warnings */
  warnings: string[];

  /** Checksum valid? */
  checksumValid: boolean;

  /** TTL/Hop Limit acceptable? */
  ttlValid: boolean;

  /** Header format valid? */
  headerValid: boolean;
}

/**
 * Protocol error codes
 */
export enum ProtocolErrorCode {
  INVALID_PACKET = 'P001',
  CHECKSUM_MISMATCH = 'P002',
  TTL_EXPIRED = 'P003',
  DEST_UNREACHABLE = 'P004',
  PORT_UNREACHABLE = 'P005',
  FRAGMENTATION_NEEDED = 'P006',
  INVALID_HEADER = 'P007',
  PROTOCOL_ERROR = 'P008',
}

/**
 * Protocol error
 */
export class ProtocolError extends Error {
  constructor(
    public code: ProtocolErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ProtocolError';
  }
}

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Network interface configuration
 */
export interface NetworkInterface {
  /** Interface name */
  name: string;

  /** MAC address */
  macAddress: string;

  /** IPv4 addresses */
  ipv4Addresses: IPv4Address[];

  /** IPv6 addresses */
  ipv6Addresses: IPv6Address[];

  /** MTU (Maximum Transmission Unit) */
  mtu: number;

  /** Interface state */
  state: 'up' | 'down';

  /** Interface speed (Mbps) */
  speed?: number;

  /** Duplex mode */
  duplex?: 'half' | 'full';
}

/**
 * Subnet calculation
 */
export interface SubnetInfo {
  /** Network address */
  network: string;

  /** Broadcast address */
  broadcast: string;

  /** Subnet mask */
  mask: string;

  /** CIDR notation */
  cidr: string;

  /** Wildcard mask */
  wildcard: string;

  /** Number of hosts */
  hostCount: number;

  /** First usable address */
  firstHost: string;

  /** Last usable address */
  lastHost: string;

  /** IP class */
  class: 'A' | 'B' | 'C';
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Protocol numbers (for IP protocol field)
 */
export const PROTOCOL_NUMBERS = {
  ICMP: 1,
  IGMP: 2,
  TCP: 6,
  UDP: 17,
  IPV6: 41,
  GRE: 47,
  ESP: 50,
  AH: 51,
  ICMPV6: 58,
  OSPF: 89,
  SCTP: 132,
} as const;

/**
 * Well-known ports
 */
export const WELL_KNOWN_PORTS = {
  FTP_DATA: 20,
  FTP_CONTROL: 21,
  SSH: 22,
  TELNET: 23,
  SMTP: 25,
  DNS: 53,
  HTTP: 80,
  POP3: 110,
  IMAP: 143,
  SNMP: 161,
  HTTPS: 443,
  SMTPS: 465,
  IMAPS: 993,
  POP3S: 995,
  MYSQL: 3306,
  POSTGRESQL: 5432,
  MONGODB: 27017,
} as const;

/**
 * MTU sizes
 */
export const MTU_SIZES = {
  ETHERNET: 1500,
  JUMBO_FRAME: 9000,
  PPPoE: 1492,
  LOOPBACK: 65536,
  IPV6_MIN: 1280,
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  OSILayerInfo,
  IPv4Address,
  IPv6Address,
  TCPFlags,
  TCPState,
  TCPSegment,
  TCPConnection,
  UDPDatagram,
  IPv4Packet,
  IPv6Packet,
  BGPConfig,
  BGPPeer,
  OSPFConfig,
  OSPFArea,
  OSPFNetwork,
  RouteEntry,
  HTTP2Request,
  HTTP2Response,
  HTTP3Request,
  DNSQuery,
  DNSResponse,
  DNSRecord,
  GRPCService,
  GRPCMethod,
  PacketCapture,
  CapturedPacket,
  NetworkStats,
  PacketValidation,
  NetworkInterface,
  SubnetInfo,
};

export { ProtocolErrorCode, ProtocolError };
export { PROTOCOL_NUMBERS, WELL_KNOWN_PORTS, MTU_SIZES };
