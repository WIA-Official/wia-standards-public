/**
 * WIA-COMM-016: VPN Protocol SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for VPN protocols including:
 * - IPsec configuration and validation
 * - OpenVPN setup and management
 * - WireGuard key generation and configuration
 * - VPN performance calculations
 * - Security auditing
 */

import {
  VPNProtocol,
  IPsecConfig,
  OpenVPNConfig,
  WireGuardConfig,
  WireGuardKeyPair,
  L2TPConfig,
  TunnelOverhead,
  VPNMetrics,
  VPNSecurityAudit,
  VPNSecurityFinding,
  CertificateInfo,
  SplitTunnelingConfig,
  VPNConnectionRequest,
  VPNConnectionResponse,
  VPNErrorCode,
  VPNError,
  PROTOCOL_OVERHEAD,
  DEFAULT_PORTS,
  RECOMMENDED_MTU,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMM-016 VPN Protocol SDK
 */
export class VPNSDK {
  private version = '1.0.0';
  private initialized = false;

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Configure IPsec VPN
   *
   * @param config - IPsec configuration
   * @returns Validated IPsec configuration
   */
  configureIPsec(config: IPsecConfig): IPsecConfig {
    // Validate configuration
    this.validateIPsecConfig(config);

    // Set defaults
    const fullConfig: IPsecConfig = {
      ...config,
      phase1Lifetime: config.phase1Lifetime || 28800, // 8 hours
      phase2Lifetime: config.phase2Lifetime || 3600, // 1 hour
      dpdInterval: config.dpdInterval || 30,
      natTraversal: config.natTraversal !== false, // Default true
    };

    return fullConfig;
  }

  /**
   * Validate IPsec configuration
   */
  private validateIPsecConfig(config: IPsecConfig): void {
    if (!config.localId || !config.remoteId) {
      throw new VPNError(
        VPNErrorCode.INVALID_CONFIG,
        'Local and remote IDs are required'
      );
    }

    if (!config.psk && !config.certificate) {
      throw new VPNError(
        VPNErrorCode.INVALID_CONFIG,
        'Either PSK or certificate must be provided'
      );
    }

    if (config.encryption === '3DES') {
      console.warn('3DES is deprecated, consider using AES-256-GCM');
    }

    if (config.dhGroup === 'modp1024' || config.dhGroup === 'modp1536') {
      throw new VPNError(
        VPNErrorCode.INVALID_CONFIG,
        'DH groups below modp2048 are insecure'
      );
    }
  }

  /**
   * Configure OpenVPN
   *
   * @param config - OpenVPN configuration
   * @returns Validated OpenVPN configuration
   */
  configureOpenVPN(config: OpenVPNConfig): OpenVPNConfig {
    // Validate configuration
    this.validateOpenVPNConfig(config);

    // Set defaults
    const fullConfig: OpenVPNConfig = {
      ...config,
      device: config.device || 'tun',
      tlsVersionMin: config.tlsVersionMin || '1.3',
      compression: config.compression || 'none', // Disabled for security
      verbosity: config.verbosity || 3,
      keepalive:
        config.keepalive ||
        {
          interval: 10,
          timeout: 60,
        },
    };

    return fullConfig;
  }

  /**
   * Validate OpenVPN configuration
   */
  private validateOpenVPNConfig(config: OpenVPNConfig): void {
    if (!config.caCert || !config.cert || !config.key) {
      throw new VPNError(
        VPNErrorCode.INVALID_CONFIG,
        'CA certificate, certificate, and key are required'
      );
    }

    if (config.mode === 'client' && !config.serverAddress) {
      throw new VPNError(
        VPNErrorCode.INVALID_CONFIG,
        'Server address is required for client mode'
      );
    }

    if (config.tlsVersionMin === '1.2') {
      console.warn('TLS 1.3 is recommended for better security');
    }
  }

  /**
   * Setup WireGuard
   *
   * @param config - WireGuard configuration
   * @returns Validated WireGuard configuration
   */
  setupWireGuard(config: WireGuardConfig): WireGuardConfig {
    // Validate configuration
    this.validateWireGuardConfig(config);

    // Derive public key if not provided
    if (!config.publicKey) {
      config.publicKey = this.derivePublicKey(config.privateKey);
    }

    // Set defaults
    const fullConfig: WireGuardConfig = {
      ...config,
      interfaceName: config.interfaceName || 'wg0',
      mtu: config.mtu || 1420,
    };

    return fullConfig;
  }

  /**
   * Validate WireGuard configuration
   */
  private validateWireGuardConfig(config: WireGuardConfig): void {
    if (!config.privateKey) {
      throw new VPNError(
        VPNErrorCode.INVALID_CONFIG,
        'Private key is required'
      );
    }

    if (!config.listenPort) {
      throw new VPNError(VPNErrorCode.INVALID_CONFIG, 'Listen port is required');
    }

    if (config.peers.length === 0) {
      throw new VPNError(
        VPNErrorCode.INVALID_CONFIG,
        'At least one peer is required'
      );
    }

    // Validate each peer
    for (const peer of config.peers) {
      if (!peer.publicKey) {
        throw new VPNError(
          VPNErrorCode.INVALID_CONFIG,
          'Peer public key is required'
        );
      }

      if (!peer.allowedIPs || peer.allowedIPs.length === 0) {
        throw new VPNError(
          VPNErrorCode.INVALID_CONFIG,
          'Peer allowed IPs are required'
        );
      }
    }
  }

  /**
   * Generate WireGuard key pair
   *
   * @returns WireGuard key pair
   */
  generateWireGuardKeys(): WireGuardKeyPair {
    // In a real implementation, this would use crypto libraries
    // For demonstration, we simulate key generation
    const privateKey = this.generateBase64Key(32);
    const publicKey = this.derivePublicKey(privateKey);
    const presharedKey = this.generateBase64Key(32);

    return {
      privateKey,
      publicKey,
      presharedKey,
    };
  }

  /**
   * Derive public key from private key (simulated)
   */
  private derivePublicKey(privateKey: string): string {
    // In real implementation, use Curve25519
    // This is a simulation
    return Buffer.from(`pub_${privateKey}`).toString('base64');
  }

  /**
   * Generate base64 key (simulated)
   */
  private generateBase64Key(bytes: number): string {
    // In real implementation, use crypto.randomBytes
    // This is a simulation
    const randomBytes = Array.from({ length: bytes }, () =>
      Math.floor(Math.random() * 256)
    );
    return Buffer.from(randomBytes).toString('base64');
  }

  /**
   * Calculate tunnel overhead
   *
   * @param protocol - VPN protocol
   * @param mtu - Original MTU (default: 1500)
   * @returns Tunnel overhead calculation
   */
  calculateTunnelOverhead(
    protocol: VPNProtocol,
    mtu: number = 1500
  ): TunnelOverhead {
    const overhead = PROTOCOL_OVERHEAD[protocol];
    const effectiveMTU = mtu - overhead;
    const mss = effectiveMTU - 40; // IP (20) + TCP (20) headers

    // Fragmentation risk assessment
    let fragmentationRisk: 'low' | 'medium' | 'high';
    if (effectiveMTU >= 1400) {
      fragmentationRisk = 'low';
    } else if (effectiveMTU >= 1300) {
      fragmentationRisk = 'medium';
    } else {
      fragmentationRisk = 'high';
    }

    const recommendedMTU = RECOMMENDED_MTU[protocol];

    return {
      protocol,
      originalMTU: mtu,
      overhead,
      effectiveMTU,
      mss,
      fragmentationRisk,
      recommendedMTU,
    };
  }

  /**
   * Estimate VPN throughput
   *
   * @param protocol - VPN protocol
   * @param linkSpeed - Link speed in bps
   * @param cpuCores - Number of CPU cores
   * @param hardwareAcceleration - Hardware acceleration available
   * @returns Estimated throughput in bps
   */
  estimateThroughput(
    protocol: VPNProtocol,
    linkSpeed: number,
    cpuCores: number = 4,
    hardwareAcceleration: boolean = false
  ): number {
    // Base efficiency factors
    const efficiencyFactors: Record<VPNProtocol, number> = {
      wireguard: 0.95,
      'ipsec-ikev2': hardwareAcceleration ? 0.9 : 0.7,
      'ipsec-ikev1': hardwareAcceleration ? 0.85 : 0.65,
      openvpn: 0.6,
      'l2tp-ipsec': 0.65,
      sstp: 0.7,
      pptp: 0.8, // Fast but insecure
    };

    // CPU scaling factor (diminishing returns)
    const cpuFactor = Math.min(1 + Math.log2(cpuCores) * 0.2, 2.0);

    // Calculate estimated throughput
    const baseThroughput = linkSpeed * efficiencyFactors[protocol];
    const estimatedThroughput = baseThroughput * cpuFactor;

    // Cap at link speed
    return Math.min(estimatedThroughput, linkSpeed);
  }

  /**
   * Validate VPN configuration
   *
   * @param config - VPN connection request
   * @returns Validation result
   */
  validateVPNConfig(config: VPNConnectionRequest): {
    isValid: boolean;
    errors: string[];
    warnings: string[];
  } {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Protocol-specific validation
    try {
      switch (config.protocol) {
        case 'ipsec-ikev2':
        case 'ipsec-ikev1':
          this.validateIPsecConfig(config.config as IPsecConfig);
          break;
        case 'openvpn':
          this.validateOpenVPNConfig(config.config as OpenVPNConfig);
          break;
        case 'wireguard':
          this.validateWireGuardConfig(config.config as WireGuardConfig);
          break;
        case 'pptp':
          warnings.push('PPTP is insecure and deprecated');
          break;
      }
    } catch (error) {
      if (error instanceof VPNError) {
        errors.push(error.message);
      } else {
        errors.push('Unknown validation error');
      }
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
    };
  }

  /**
   * Perform VPN security audit
   *
   * @param protocol - VPN protocol
   * @param config - VPN configuration
   * @returns Security audit result
   */
  performSecurityAudit(
    protocol: VPNProtocol,
    config: IPsecConfig | OpenVPNConfig | WireGuardConfig | L2TPConfig
  ): VPNSecurityAudit {
    const findings: VPNSecurityFinding[] = [];
    let score = 100;

    // Protocol-specific checks
    if (protocol === 'pptp') {
      findings.push({
        severity: 'critical',
        category: 'protocol',
        title: 'PPTP is insecure',
        description:
          'PPTP has known vulnerabilities and should not be used',
        remediation: 'Migrate to WireGuard, IKEv2, or OpenVPN',
      });
      score -= 50;
    }

    if (protocol === 'ipsec-ikev2' || protocol === 'ipsec-ikev1') {
      const ipsecConfig = config as IPsecConfig;

      // Check encryption
      if (ipsecConfig.encryption === '3DES') {
        findings.push({
          severity: 'high',
          category: 'encryption',
          title: 'Weak encryption algorithm',
          description: '3DES is deprecated',
          remediation: 'Use AES-256-GCM',
        });
        score -= 20;
      }

      // Check DH group
      if (
        ipsecConfig.dhGroup === 'modp1024' ||
        ipsecConfig.dhGroup === 'modp1536'
      ) {
        findings.push({
          severity: 'critical',
          category: 'encryption',
          title: 'Weak DH group',
          description: 'DH groups below 2048 bits are insecure',
          remediation: 'Use modp2048 or higher',
        });
        score -= 30;
      }

      // Check PFS
      if (!ipsecConfig.pfsEnabled) {
        findings.push({
          severity: 'medium',
          category: 'configuration',
          title: 'Perfect Forward Secrecy disabled',
          description: 'PFS should be enabled for enhanced security',
          remediation: 'Enable PFS in configuration',
        });
        score -= 15;
      }
    }

    if (protocol === 'openvpn') {
      const ovpnConfig = config as OpenVPNConfig;

      // Check TLS version
      if (ovpnConfig.tlsVersionMin === '1.2') {
        findings.push({
          severity: 'low',
          category: 'configuration',
          title: 'TLS 1.2 in use',
          description: 'TLS 1.3 offers better security',
          remediation: 'Upgrade to TLS 1.3',
        });
        score -= 5;
      }

      // Check compression
      if (ovpnConfig.compression && ovpnConfig.compression !== 'none') {
        findings.push({
          severity: 'medium',
          category: 'configuration',
          title: 'Compression enabled',
          description: 'Compression can lead to VORACLE attacks',
          remediation: 'Disable compression',
        });
        score -= 10;
      }
    }

    // Determine security level
    let securityLevel: 'excellent' | 'good' | 'acceptable' | 'weak' | 'vulnerable';
    if (score >= 90) {
      securityLevel = 'excellent';
    } else if (score >= 75) {
      securityLevel = 'good';
    } else if (score >= 60) {
      securityLevel = 'acceptable';
    } else if (score >= 40) {
      securityLevel = 'weak';
    } else {
      securityLevel = 'vulnerable';
    }

    // Generate recommendations
    const recommendations: string[] = [];
    if (score < 100) {
      recommendations.push('Review and address all security findings');
    }
    if (protocol !== 'wireguard' && protocol !== 'ipsec-ikev2') {
      recommendations.push('Consider migrating to WireGuard or IKEv2');
    }
    recommendations.push('Enable Perfect Forward Secrecy');
    recommendations.push('Use strong encryption (AES-256-GCM or ChaCha20-Poly1305)');
    recommendations.push('Implement multi-factor authentication');

    return {
      connectionId: 'audit-' + Date.now(),
      timestamp: new Date(),
      securityLevel,
      findings,
      score: Math.max(0, score),
      recommendations,
    };
  }

  /**
   * Calculate optimal split tunneling configuration
   *
   * @param corporateNetworks - Corporate network CIDRs
   * @param localNetworks - Local network CIDRs to exclude
   * @returns Split tunneling configuration
   */
  calculateSplitTunneling(
    corporateNetworks: string[],
    localNetworks: string[] = []
  ): SplitTunnelingConfig {
    return {
      enabled: true,
      mode: 'include',
      routes: corporateNetworks,
      dns: 'tunnel-split',
    };
  }

  /**
   * Estimate connection latency
   *
   * @param protocol - VPN protocol
   * @param baseLatency - Base network latency (ms)
   * @returns Estimated VPN latency (ms)
   */
  estimateLatency(protocol: VPNProtocol, baseLatency: number): number {
    const latencyOverhead: Record<VPNProtocol, number> = {
      wireguard: 0.1,
      'ipsec-ikev2': 0.2,
      'ipsec-ikev1': 0.3,
      openvpn: 0.5,
      'l2tp-ipsec': 0.4,
      sstp: 0.3,
      pptp: 0.2,
    };

    return baseLatency + latencyOverhead[protocol];
  }

  /**
   * Generate VPN configuration file
   *
   * @param protocol - VPN protocol
   * @param config - VPN configuration
   * @returns Configuration file content
   */
  generateConfigFile(
    protocol: VPNProtocol,
    config: IPsecConfig | OpenVPNConfig | WireGuardConfig
  ): string {
    switch (protocol) {
      case 'wireguard':
        return this.generateWireGuardConfig(config as WireGuardConfig);
      case 'openvpn':
        return this.generateOpenVPNConfigFile(config as OpenVPNConfig);
      case 'ipsec-ikev2':
      case 'ipsec-ikev1':
        return this.generateIPsecConfig(config as IPsecConfig);
      default:
        throw new VPNError(
          VPNErrorCode.UNSUPPORTED_PROTOCOL,
          `Configuration generation not supported for ${protocol}`
        );
    }
  }

  /**
   * Generate WireGuard configuration file
   */
  private generateWireGuardConfig(config: WireGuardConfig): string {
    let configFile = '[Interface]\n';
    configFile += `PrivateKey = ${config.privateKey}\n`;
    configFile += `ListenPort = ${config.listenPort}\n`;

    if (config.addresses.length > 0) {
      configFile += `Address = ${config.addresses.join(', ')}\n`;
    }

    if (config.dns && config.dns.length > 0) {
      configFile += `DNS = ${config.dns.join(', ')}\n`;
    }

    if (config.mtu) {
      configFile += `MTU = ${config.mtu}\n`;
    }

    for (const peer of config.peers) {
      configFile += '\n[Peer]\n';
      configFile += `PublicKey = ${peer.publicKey}\n`;

      if (peer.presharedKey) {
        configFile += `PresharedKey = ${peer.presharedKey}\n`;
      }

      if (peer.endpoint) {
        configFile += `Endpoint = ${peer.endpoint}\n`;
      }

      configFile += `AllowedIPs = ${peer.allowedIPs.join(', ')}\n`;

      if (peer.persistentKeepalive) {
        configFile += `PersistentKeepalive = ${peer.persistentKeepalive}\n`;
      }
    }

    return configFile;
  }

  /**
   * Generate OpenVPN configuration file
   */
  private generateOpenVPNConfigFile(config: OpenVPNConfig): string {
    let configFile = '';

    configFile += `# OpenVPN Configuration\n`;
    configFile += config.mode === 'client' ? 'client\n' : 'server\n';
    configFile += `dev ${config.device}\n`;
    configFile += `proto ${config.protocol}\n`;
    configFile += `port ${config.port}\n\n`;

    if (config.mode === 'client' && config.serverAddress) {
      configFile += `remote ${config.serverAddress} ${config.port}\n`;
    }

    if (config.mode === 'server' && config.serverNetwork) {
      configFile += `server ${config.serverNetwork} ${config.serverNetmask}\n`;
    }

    configFile += `\nca ${config.caCert}\n`;
    configFile += `cert ${config.cert}\n`;
    configFile += `key ${config.key}\n`;

    if (config.dhParams) {
      configFile += `dh ${config.dhParams}\n`;
    }

    configFile += `\ncipher ${config.cipher}\n`;
    configFile += `auth ${config.auth}\n`;
    configFile += `tls-version-min ${config.tlsVersionMin}\n`;

    if (config.tlsAuth) {
      configFile += `tls-auth ${config.tlsAuth}\n`;
    }

    if (config.keepalive) {
      configFile += `\nkeepalive ${config.keepalive.interval} ${config.keepalive.timeout}\n`;
    }

    configFile += `\nverb ${config.verbosity || 3}\n`;

    return configFile;
  }

  /**
   * Generate IPsec configuration
   */
  private generateIPsecConfig(config: IPsecConfig): string {
    let configFile = `# IPsec Configuration\n`;
    configFile += `conn ${config.localId}-to-${config.remoteId}\n`;
    configFile += `    type=tunnel\n`;
    configFile += `    auto=start\n`;
    configFile += `    left=%defaultroute\n`;
    configFile += `    leftid=${config.localId}\n`;
    configFile += `    right=${config.remoteId}\n`;
    configFile += `    ike=${config.encryption}-${config.authentication}-${config.dhGroup}\n`;
    configFile += `    esp=${config.encryption}-${config.authentication}\n`;

    if (config.pfsEnabled) {
      configFile += `    pfs=yes\n`;
    }

    if (config.localSubnet) {
      configFile += `    leftsubnet=${config.localSubnet}\n`;
    }

    if (config.remoteSubnet) {
      configFile += `    rightsubnet=${config.remoteSubnet}\n`;
    }

    return configFile;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Configure IPsec (standalone)
 */
export function configureIPsec(config: IPsecConfig): IPsecConfig {
  const sdk = new VPNSDK();
  return sdk.configureIPsec(config);
}

/**
 * Configure OpenVPN (standalone)
 */
export function configureOpenVPN(config: OpenVPNConfig): OpenVPNConfig {
  const sdk = new VPNSDK();
  return sdk.configureOpenVPN(config);
}

/**
 * Setup WireGuard (standalone)
 */
export function setupWireGuard(config: WireGuardConfig): WireGuardConfig {
  const sdk = new VPNSDK();
  return sdk.setupWireGuard(config);
}

/**
 * Generate WireGuard keys (standalone)
 */
export function generateWireGuardKeys(): WireGuardKeyPair {
  const sdk = new VPNSDK();
  return sdk.generateWireGuardKeys();
}

/**
 * Calculate tunnel overhead (standalone)
 */
export function calculateTunnelOverhead(
  protocol: VPNProtocol,
  mtu?: number
): TunnelOverhead {
  const sdk = new VPNSDK();
  return sdk.calculateTunnelOverhead(protocol, mtu);
}

/**
 * Validate VPN configuration (standalone)
 */
export function validateVPNConfig(
  config: VPNConnectionRequest
): { isValid: boolean; errors: string[]; warnings: string[] } {
  const sdk = new VPNSDK();
  return sdk.validateVPNConfig(config);
}

/**
 * Perform security audit (standalone)
 */
export function performSecurityAudit(
  protocol: VPNProtocol,
  config: IPsecConfig | OpenVPNConfig | WireGuardConfig | L2TPConfig
): VPNSecurityAudit {
  const sdk = new VPNSDK();
  return sdk.performSecurityAudit(protocol, config);
}

// ============================================================================
// Export All
// ============================================================================

export * from './types';
export { VPNSDK };
