/**
 * WIA-COMM-020: Network Protocol SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive network protocol implementations including:
 * - IPv4/IPv6 address parsing and validation
 * - TCP/UDP segment creation and parsing
 * - Routing protocol configuration
 * - HTTP/2, HTTP/3, gRPC support
 * - Packet analysis and validation
 */

import {
  IPv4Address,
  IPv6Address,
  TCPSegment,
  TCPConnection,
  TCPState,
  TCPFlags,
  UDPDatagram,
  IPv4Packet,
  IPv6Packet,
  BGPConfig,
  OSPFConfig,
  RouteEntry,
  HTTP2Request,
  HTTP2Response,
  HTTP3Request,
  DNSQuery,
  DNSResponse,
  SubnetInfo,
  PacketValidation,
  NetworkStats,
  ProtocolError,
  ProtocolErrorCode,
  PROTOCOL_NUMBERS,
  WELL_KNOWN_PORTS,
  MTU_SIZES,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMM-020 Network Protocol SDK
 */
export class NetworkProtocolSDK {
  private version = '1.0.0';

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ==========================================================================
  // IPv4 Methods
  // ==========================================================================

  /**
   * Parse and analyze IPv4 address with CIDR notation
   *
   * @param address - IPv4 address with CIDR (e.g., "192.168.1.0/24")
   * @returns IPv4 address information
   */
  parseIPv4(address: string): IPv4Address {
    const parts = address.split('/');
    const ip = parts[0];
    const cidr = parts[1] ? parseInt(parts[1]) : 32;

    if (!this.isValidIPv4(ip)) {
      throw new ProtocolError(
        ProtocolErrorCode.INVALID_HEADER,
        `Invalid IPv4 address: ${ip}`
      );
    }

    if (cidr < 0 || cidr > 32) {
      throw new ProtocolError(
        ProtocolErrorCode.INVALID_HEADER,
        `Invalid CIDR prefix: ${cidr}`
      );
    }

    const ipNum = this.ipv4ToNumber(ip);
    const maskNum = (0xffffffff << (32 - cidr)) >>> 0;
    const networkNum = (ipNum & maskNum) >>> 0;
    const broadcastNum = (networkNum | ~maskNum) >>> 0;

    const network = this.numberToIPv4(networkNum);
    const broadcast = this.numberToIPv4(broadcastNum);
    const subnetMask = this.numberToIPv4(maskNum);

    const hostCount = Math.pow(2, 32 - cidr) - 2;
    const firstHost = this.numberToIPv4(networkNum + 1);
    const lastHost = this.numberToIPv4(broadcastNum - 1);

    const firstOctet = parseInt(ip.split('.')[0]);
    let ipClass: 'A' | 'B' | 'C' | 'D' | 'E';
    if (firstOctet < 128) ipClass = 'A';
    else if (firstOctet < 192) ipClass = 'B';
    else if (firstOctet < 224) ipClass = 'C';
    else if (firstOctet < 240) ipClass = 'D';
    else ipClass = 'E';

    const isPrivate = this.isPrivateIPv4(ip);

    return {
      address: ip,
      network,
      broadcast,
      subnetMask,
      cidr,
      hostCount: Math.max(0, hostCount),
      firstHost: hostCount > 0 ? firstHost : network,
      lastHost: hostCount > 0 ? lastHost : network,
      isPrivate,
      class: ipClass,
    };
  }

  /**
   * Calculate subnet information
   */
  calculateSubnet(network: string, cidr: number): SubnetInfo {
    const ipv4 = this.parseIPv4(`${network}/${cidr}`);

    const maskNum = (0xffffffff << (32 - cidr)) >>> 0;
    const wildcardNum = (~maskNum) >>> 0;
    const wildcard = this.numberToIPv4(wildcardNum);

    return {
      network: ipv4.network,
      broadcast: ipv4.broadcast,
      mask: ipv4.subnetMask,
      cidr: `${ipv4.network}/${cidr}`,
      wildcard,
      hostCount: ipv4.hostCount,
      firstHost: ipv4.firstHost,
      lastHost: ipv4.lastHost,
      class: ipv4.class as 'A' | 'B' | 'C',
    };
  }

  // ==========================================================================
  // IPv6 Methods
  // ==========================================================================

  /**
   * Parse and analyze IPv6 address
   *
   * @param address - IPv6 address (compressed or canonical)
   * @returns IPv6 address information
   */
  parseIPv6(address: string): IPv6Address {
    const parts = address.split('/');
    const ip = parts[0];
    const prefixLength = parts[1] ? parseInt(parts[1]) : 128;

    if (!this.isValidIPv6(ip)) {
      throw new ProtocolError(
        ProtocolErrorCode.INVALID_HEADER,
        `Invalid IPv6 address: ${ip}`
      );
    }

    const canonical = this.expandIPv6(ip);
    const compressed = this.compressIPv6(canonical);

    // Extract prefix and interface ID
    const hextets = canonical.split(':');
    const prefixHextets = Math.floor(prefixLength / 16);
    const prefix = hextets.slice(0, prefixHextets).join(':');
    const interfaceID = hextets.slice(4).join(':');

    // Determine scope and type
    let scope: IPv6Address['scope'];
    let type: IPv6Address['type'];

    if (ip === '::1') {
      scope = 'loopback';
      type = 'unicast';
    } else if (ip.startsWith('fe80:')) {
      scope = 'link-local';
      type = 'unicast';
    } else if (ip.startsWith('fc') || ip.startsWith('fd')) {
      scope = 'unique-local';
      type = 'unicast';
    } else if (ip.startsWith('ff')) {
      scope = 'multicast';
      type = 'multicast';
    } else {
      scope = 'global';
      type = 'unicast';
    }

    const isIPv4Mapped = canonical.includes('::ffff:');

    return {
      address: ip,
      canonical,
      compressed,
      prefix,
      prefixLength,
      interfaceID,
      scope,
      type,
      isIPv4Mapped,
    };
  }

  // ==========================================================================
  // TCP Methods
  // ==========================================================================

  /**
   * Create TCP segment
   *
   * @param config - TCP segment configuration
   * @returns TCP segment
   */
  createTCPSegment(config: {
    sourcePort: number;
    destPort: number;
    sequenceNumber: number;
    ackNumber: number;
    flags: TCPFlags;
    windowSize: number;
    data: Buffer;
    options?: Buffer;
  }): TCPSegment {
    const dataOffset = config.options
      ? 5 + Math.ceil(config.options.length / 4)
      : 5;

    const segment: TCPSegment = {
      sourcePort: config.sourcePort,
      destPort: config.destPort,
      sequenceNumber: config.sequenceNumber,
      ackNumber: config.ackNumber,
      dataOffset,
      flags: config.flags,
      windowSize: config.windowSize,
      checksum: 0, // Will be calculated
      urgentPointer: 0,
      options: config.options,
      data: config.data,
    };

    segment.checksum = this.calculateTCPChecksum(segment);

    return segment;
  }

  /**
   * Create TCP connection
   */
  createTCPConnection(config: {
    sourceIP: string;
    destinationIP: string;
    sourcePort: number;
    destinationPort: number;
    windowSize?: number;
  }): TCPConnection {
    const id = `${config.sourceIP}:${config.sourcePort}-${config.destinationIP}:${config.destinationPort}`;
    const now = new Date();

    return {
      id,
      sourceIP: config.sourceIP,
      destinationIP: config.destinationIP,
      sourcePort: config.sourcePort,
      destinationPort: config.destinationPort,
      state: 'CLOSED',
      sendSeq: Math.floor(Math.random() * 0xffffffff),
      receiveSeq: 0,
      sendWindow: config.windowSize || 65535,
      receiveWindow: config.windowSize || 65535,
      created: now,
      lastActivity: now,
    };
  }

  // ==========================================================================
  // UDP Methods
  // ==========================================================================

  /**
   * Create UDP datagram
   */
  createUDPDatagram(config: {
    sourcePort: number;
    destPort: number;
    data: Buffer;
  }): UDPDatagram {
    const length = 8 + config.data.length;

    const datagram: UDPDatagram = {
      sourcePort: config.sourcePort,
      destPort: config.destPort,
      length,
      checksum: 0, // Will be calculated
      data: config.data,
    };

    datagram.checksum = this.calculateUDPChecksum(datagram);

    return datagram;
  }

  // ==========================================================================
  // Routing Protocol Methods
  // ==========================================================================

  /**
   * Configure BGP
   */
  configureBGP(config: {
    asn: number;
    routerID: string;
    peers: Array<{
      peerIP: string;
      remoteASN: number;
      type?: 'eBGP' | 'iBGP';
    }>;
  }): BGPConfig {
    return {
      asn: config.asn,
      routerID: config.routerID,
      peers: config.peers.map((peer) => ({
        peerIP: peer.peerIP,
        remoteASN: peer.remoteASN,
        type: peer.type || (peer.remoteASN === config.asn ? 'iBGP' : 'eBGP'),
        state: 'Idle',
        holdTime: 180,
        keepalive: 60,
      })),
    };
  }

  /**
   * Configure OSPF
   */
  configureOSPF(config: {
    routerID: string;
    areas: Array<{
      id: string;
      type: 'backbone' | 'standard' | 'stub' | 'totally-stub' | 'nssa';
    }>;
    networks: Array<{
      network: string;
      area: string;
    }>;
  }): OSPFConfig {
    return {
      routerID: config.routerID,
      areas: config.areas,
      networks: config.networks,
      helloInterval: 10,
      deadInterval: 40,
    };
  }

  /**
   * Create routing table entry
   */
  createRoute(config: {
    destination: string;
    nextHop: string;
    interface: string;
    protocol: string;
    metric?: number;
  }): RouteEntry {
    const adminDistances: Record<string, number> = {
      connected: 0,
      static: 1,
      eigrp: 90,
      ospf: 110,
      rip: 120,
      bgp: 200,
    };

    return {
      destination: config.destination,
      nextHop: config.nextHop,
      interface: config.interface,
      metric: config.metric || 0,
      protocol: config.protocol,
      adminDistance: adminDistances[config.protocol.toLowerCase()] || 255,
      age: 0,
    };
  }

  // ==========================================================================
  // HTTP/2 & HTTP/3 Methods
  // ==========================================================================

  /**
   * Build HTTP/2 request
   */
  buildHTTP2Request(config: {
    method: HTTP2Request['method'];
    path: string;
    headers: Record<string, string>;
    body?: string | Buffer;
    priority?: HTTP2Request['priority'];
  }): HTTP2Request {
    const headers = {
      ':method': config.method,
      ':path': config.path,
      ':scheme': 'https',
      ...config.headers,
    };

    return {
      method: config.method,
      path: config.path,
      headers,
      body: config.body ? Buffer.from(config.body) : undefined,
      priority: config.priority,
      streamID: Math.floor(Math.random() * 1000) * 2 + 1, // Odd number for client
    };
  }

  /**
   * Create HTTP/3 request
   */
  createHTTP3Request(config: {
    method: HTTP3Request['method'];
    url: string;
    headers: Record<string, string>;
    body?: string | Buffer;
  }): HTTP3Request {
    return {
      method: config.method,
      url: config.url,
      headers: config.headers,
      body: config.body ? Buffer.from(config.body) : undefined,
      streamID: Math.floor(Math.random() * 1000) * 4, // Client-initiated bidirectional
    };
  }

  // ==========================================================================
  // DNS Methods
  // ==========================================================================

  /**
   * Create DNS query
   */
  createDNSQuery(config: {
    name: string;
    type: DNSQuery['type'];
    recursionDesired?: boolean;
  }): DNSQuery {
    return {
      name: config.name,
      type: config.type,
      class: 'IN',
      recursionDesired: config.recursionDesired !== false,
    };
  }

  // ==========================================================================
  // Packet Validation Methods
  // ==========================================================================

  /**
   * Validate packet structure and checksums
   */
  validatePacket(packet: IPv4Packet | TCPSegment | UDPDatagram): PacketValidation {
    const errors: string[] = [];
    const warnings: string[] = [];

    let checksumValid = true;
    let ttlValid = true;
    let headerValid = true;

    // IPv4 packet validation
    if ('version' in packet && packet.version === 4) {
      const ipv4 = packet as IPv4Packet;

      // Check version
      if (ipv4.version !== 4) {
        errors.push('Invalid IP version');
        headerValid = false;
      }

      // Check TTL
      if (ipv4.ttl === 0) {
        errors.push('TTL expired');
        ttlValid = false;
      } else if (ipv4.ttl < 5) {
        warnings.push('Low TTL value');
      }

      // Validate checksum
      const calculatedChecksum = this.calculateIPv4Checksum(ipv4);
      if (calculatedChecksum !== ipv4.checksum) {
        errors.push('IPv4 checksum mismatch');
        checksumValid = false;
      }
    }

    // TCP segment validation
    if ('flags' in packet) {
      const tcp = packet as TCPSegment;

      // Check port numbers
      if (tcp.sourcePort < 0 || tcp.sourcePort > 65535) {
        errors.push('Invalid source port');
        headerValid = false;
      }
      if (tcp.destPort < 0 || tcp.destPort > 65535) {
        errors.push('Invalid destination port');
        headerValid = false;
      }

      // Validate flags
      if (tcp.flags.SYN && tcp.flags.FIN) {
        errors.push('Invalid TCP flags: SYN and FIN both set');
        headerValid = false;
      }
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      checksumValid,
      ttlValid,
      headerValid,
    };
  }

  /**
   * Calculate checksum for validation
   */
  calculateChecksum(data: Buffer): number {
    let sum = 0;

    // Process 16-bit words
    for (let i = 0; i < data.length - 1; i += 2) {
      sum += (data[i] << 8) | data[i + 1];
    }

    // Handle odd length
    if (data.length % 2 === 1) {
      sum += data[data.length - 1] << 8;
    }

    // Fold 32-bit sum to 16 bits
    while (sum >> 16) {
      sum = (sum & 0xffff) + (sum >> 16);
    }

    // One's complement
    return ~sum & 0xffff;
  }

  // ==========================================================================
  // Helper Methods
  // ==========================================================================

  /**
   * Check if string is valid IPv4 address
   */
  private isValidIPv4(ip: string): boolean {
    const parts = ip.split('.');
    if (parts.length !== 4) return false;

    return parts.every((part) => {
      const num = parseInt(part);
      return num >= 0 && num <= 255 && part === num.toString();
    });
  }

  /**
   * Check if IPv4 is private
   */
  private isPrivateIPv4(ip: string): boolean {
    const num = this.ipv4ToNumber(ip);
    return (
      (num >= this.ipv4ToNumber('10.0.0.0') && num <= this.ipv4ToNumber('10.255.255.255')) ||
      (num >= this.ipv4ToNumber('172.16.0.0') && num <= this.ipv4ToNumber('172.31.255.255')) ||
      (num >= this.ipv4ToNumber('192.168.0.0') && num <= this.ipv4ToNumber('192.168.255.255'))
    );
  }

  /**
   * Convert IPv4 to number
   */
  private ipv4ToNumber(ip: string): number {
    const parts = ip.split('.');
    return (
      (parseInt(parts[0]) << 24) |
      (parseInt(parts[1]) << 16) |
      (parseInt(parts[2]) << 8) |
      parseInt(parts[3])
    ) >>> 0;
  }

  /**
   * Convert number to IPv4
   */
  private numberToIPv4(num: number): string {
    return [
      (num >>> 24) & 0xff,
      (num >>> 16) & 0xff,
      (num >>> 8) & 0xff,
      num & 0xff,
    ].join('.');
  }

  /**
   * Check if string is valid IPv6 address
   */
  private isValidIPv6(ip: string): boolean {
    // Simple validation
    const parts = ip.split(':');
    if (parts.length > 8) return false;
    if (ip.includes(':::')) return false;

    return true; // Simplified
  }

  /**
   * Expand IPv6 address to canonical form
   */
  private expandIPv6(ip: string): string {
    // Handle :: compression
    if (ip.includes('::')) {
      const parts = ip.split('::');
      const left = parts[0] ? parts[0].split(':') : [];
      const right = parts[1] ? parts[1].split(':') : [];
      const missing = 8 - left.length - right.length;

      const middle = Array(missing).fill('0000');
      const all = [...left, ...middle, ...right];

      return all.map((h) => h.padStart(4, '0')).join(':');
    }

    return ip.split(':').map((h) => h.padStart(4, '0')).join(':');
  }

  /**
   * Compress IPv6 address
   */
  private compressIPv6(ip: string): string {
    const hextets = ip.split(':');

    // Find longest run of zeros
    let longestZeroStart = -1;
    let longestZeroLength = 0;
    let currentZeroStart = -1;
    let currentZeroLength = 0;

    hextets.forEach((hextet, i) => {
      if (hextet === '0000' || hextet === '0') {
        if (currentZeroStart === -1) {
          currentZeroStart = i;
          currentZeroLength = 1;
        } else {
          currentZeroLength++;
        }

        if (currentZeroLength > longestZeroLength) {
          longestZeroStart = currentZeroStart;
          longestZeroLength = currentZeroLength;
        }
      } else {
        currentZeroStart = -1;
        currentZeroLength = 0;
      }
    });

    // Build compressed form
    if (longestZeroLength > 1) {
      const left = hextets.slice(0, longestZeroStart);
      const right = hextets.slice(longestZeroStart + longestZeroLength);

      return (
        left.map((h) => parseInt(h, 16).toString(16)).join(':') +
        '::' +
        right.map((h) => parseInt(h, 16).toString(16)).join(':')
      );
    }

    return hextets.map((h) => parseInt(h, 16).toString(16)).join(':');
  }

  /**
   * Calculate IPv4 header checksum
   */
  private calculateIPv4Checksum(packet: IPv4Packet): number {
    // Simplified checksum calculation
    return 0; // Would implement full calculation
  }

  /**
   * Calculate TCP checksum
   */
  private calculateTCPChecksum(segment: TCPSegment): number {
    // Simplified checksum calculation
    return 0; // Would implement full calculation with pseudo-header
  }

  /**
   * Calculate UDP checksum
   */
  private calculateUDPChecksum(datagram: UDPDatagram): number {
    // Simplified checksum calculation
    return 0; // Would implement full calculation with pseudo-header
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Parse IPv4 address (standalone function)
 */
export function parseIPv4Address(address: string): IPv4Address {
  const sdk = new NetworkProtocolSDK();
  return sdk.parseIPv4(address);
}

/**
 * Parse IPv6 address (standalone function)
 */
export function parseIPv6Address(address: string): IPv6Address {
  const sdk = new NetworkProtocolSDK();
  return sdk.parseIPv6(address);
}

/**
 * Create TCP connection (standalone function)
 */
export function createTCPConnection(config: {
  sourceIP: string;
  destinationIP: string;
  sourcePort: number;
  destinationPort: number;
}): TCPConnection {
  const sdk = new NetworkProtocolSDK();
  return sdk.createTCPConnection(config);
}

/**
 * Calculate subnet (standalone function)
 */
export function calculateSubnetMask(network: string, cidr: number): SubnetInfo {
  const sdk = new NetworkProtocolSDK();
  return sdk.calculateSubnet(network, cidr);
}

/**
 * Configure OSPF (standalone function)
 */
export function configureOSPF(config: OSPFConfig): OSPFConfig {
  const sdk = new NetworkProtocolSDK();
  return config;
}

/**
 * Validate packet (standalone function)
 */
export function validatePacket(packet: any): PacketValidation {
  const sdk = new NetworkProtocolSDK();
  return sdk.validatePacket(packet);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { NetworkProtocolSDK };
export default NetworkProtocolSDK;
