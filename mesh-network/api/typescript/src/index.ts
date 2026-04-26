/**
 * WIA-COMM-017: Mesh Network SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communications Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for mesh networking including:
 * - Node creation and management
 * - Routing protocol implementation
 * - Network topology optimization
 * - Self-healing mechanisms
 * - QoS management
 */

import {
  MeshNetwork,
  MeshNetworkConfig,
  MeshNode,
  MeshNodeConfig,
  MeshProtocol,
  RoutingRequest,
  RoutingResult,
  DiscoveryConfig,
  Peer,
  NetworkTopology,
  NetworkHealth,
  Route,
  Neighbor,
  Link,
  Path,
  TopologyOptimization,
  MESH_CONSTANTS,
  MeshErrorCode,
  MeshNetworkError,
  Priority,
  QoSConfig,
  NodeStatus,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMM-017 Mesh Network SDK
 */
export class MeshNetworkSDK {
  private version = '1.0.0';
  private networks: Map<string, MeshNetwork> = new Map();

  constructor() {
    // SDK initialization
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Create a new mesh network
   *
   * @param config - Network configuration
   * @returns Created mesh network
   */
  createMeshNetwork(config: MeshNetworkConfig): MeshNetwork {
    // Validate configuration
    if (!config.networkId) {
      throw new MeshNetworkError(
        MeshErrorCode.INVALID_PROTOCOL,
        'Network ID is required'
      );
    }

    if (this.networks.has(config.networkId)) {
      throw new MeshNetworkError(
        MeshErrorCode.TOPOLOGY_ERROR,
        `Network ${config.networkId} already exists`
      );
    }

    // Create network
    const network: MeshNetwork = {
      config,
      nodes: new Map(),
      topology: {
        networkId: config.networkId,
        protocol: config.protocol,
        nodes: [],
        links: [],
        gateways: [],
        diameter: 0,
        avgHopCount: 0,
        updated: new Date(),
      },
      health: this.initializeHealth(),
      stats: this.initializeStats(),
      created: new Date(),
      updated: new Date(),
    };

    this.networks.set(config.networkId, network);
    return network;
  }

  /**
   * Create a mesh node
   *
   * @param config - Node configuration
   * @returns Created mesh node
   */
  createMeshNode(config: MeshNodeConfig): MeshNode {
    // Validate configuration
    if (!config.id) {
      throw new MeshNetworkError(
        MeshErrorCode.NODE_NOT_FOUND,
        'Node ID is required'
      );
    }

    // Create node
    const node: MeshNode = {
      config,
      neighbors: [],
      routes: [],
      status: 'active',
      lastSeen: new Date(),
      stats: {
        txPackets: 0,
        rxPackets: 0,
        fwdPackets: 0,
        droppedPackets: 0,
        txBytes: 0,
        rxBytes: 0,
        avgRssi: 0,
        uptime: 0,
      },
    };

    return node;
  }

  /**
   * Add node to network
   *
   * @param networkId - Network ID
   * @param node - Node to add
   */
  addNodeToNetwork(networkId: string, node: MeshNode): void {
    const network = this.networks.get(networkId);
    if (!network) {
      throw new MeshNetworkError(
        MeshErrorCode.NODE_NOT_FOUND,
        `Network ${networkId} not found`
      );
    }

    network.nodes.set(node.config.id, node);
    network.topology.nodes.push(node.config.id);

    // Update gateway list if applicable
    if (node.config.role === 'gateway') {
      network.topology.gateways.push(node.config.id);
    }

    network.updated = new Date();
    this.updateTopology(networkId);
  }

  /**
   * Discover peers in the network
   *
   * @param config - Discovery configuration
   * @returns List of discovered peers
   */
  async discoverPeers(config: DiscoveryConfig): Promise<Peer[]> {
    // Simulate peer discovery
    // In a real implementation, this would use protocol-specific discovery

    const peers: Peer[] = [];
    const discoveryTimeout = config.timeout || 5000;

    // Simulate discovery delay
    await new Promise((resolve) => setTimeout(resolve, Math.min(discoveryTimeout, 1000)));

    // Generate sample peers based on protocol
    const peerCount = this.estimatePeerCount(config.protocol);

    for (let i = 0; i < peerCount; i++) {
      const rssi = this.generateRssi(config.protocol, config.rssiThreshold);

      if (rssi >= (config.rssiThreshold || -100)) {
        peers.push({
          id: `peer-${config.protocol}-${i + 1}`,
          address: this.generateMacAddress(),
          meshAddress: `0x${(0x1000 + i).toString(16)}`,
          rssi,
          lqi: this.rssiToLqi(rssi),
          capabilities: this.generateCapabilities(config.protocol),
          networkId: `${config.protocol}-network`,
          lastSeen: new Date(),
          metadata: {
            protocol: config.protocol,
          },
        });
      }
    }

    return peers;
  }

  /**
   * Route a packet through the mesh network
   *
   * @param networkId - Network ID
   * @param request - Routing request
   * @returns Routing result
   */
  async routePacket(networkId: string, request: RoutingRequest): Promise<RoutingResult> {
    const network = this.networks.get(networkId);
    if (!network) {
      throw new MeshNetworkError(
        MeshErrorCode.NODE_NOT_FOUND,
        `Network ${networkId} not found`
      );
    }

    const sourceNode = network.nodes.get(request.source);
    const destNode = network.nodes.get(request.destination);

    if (!sourceNode) {
      return {
        success: false,
        path: [],
        latency: 0,
        bandwidth: 0,
        hopCount: 0,
        cost: 0,
        error: `Source node ${request.source} not found`,
      };
    }

    if (!destNode) {
      return {
        success: false,
        path: [],
        latency: 0,
        bandwidth: 0,
        hopCount: 0,
        cost: 0,
        error: `Destination node ${request.destination} not found`,
      };
    }

    // Find path using routing protocol
    const path = this.findPath(network, request.source, request.destination, request.maxHops);

    if (!path) {
      return {
        success: false,
        path: [],
        latency: 0,
        bandwidth: 0,
        hopCount: 0,
        cost: 0,
        error: MeshErrorCode.ROUTE_NOT_AVAILABLE,
      };
    }

    // Calculate path metrics
    const latency = this.calculatePathLatency(network, path);
    const bandwidth = this.calculatePathBandwidth(network, path);
    const cost = this.calculatePathCost(network, path);

    // Update statistics
    this.updateRoutingStats(network, request, path);

    return {
      success: true,
      path: path.nodes,
      latency,
      bandwidth,
      hopCount: path.nodes.length - 1,
      cost,
    };
  }

  /**
   * Get network topology
   *
   * @param networkId - Network ID
   * @returns Network topology
   */
  getNetworkTopology(networkId: string): NetworkTopology {
    const network = this.networks.get(networkId);
    if (!network) {
      throw new MeshNetworkError(
        MeshErrorCode.NODE_NOT_FOUND,
        `Network ${networkId} not found`
      );
    }

    return network.topology;
  }

  /**
   * Get network health
   *
   * @param networkId - Network ID
   * @returns Network health status
   */
  getNetworkHealth(networkId: string): NetworkHealth {
    const network = this.networks.get(networkId);
    if (!network) {
      throw new MeshNetworkError(
        MeshErrorCode.NODE_NOT_FOUND,
        `Network ${networkId} not found`
      );
    }

    // Update health metrics
    this.updateHealthMetrics(network);

    return network.health;
  }

  /**
   * Optimize network topology
   *
   * @param networkId - Network ID
   * @param metric - Optimization metric
   * @returns Optimization result
   */
  optimizeTopology(
    networkId: string,
    metric: 'latency' | 'bandwidth' | 'power' | 'reliability' = 'latency'
  ): TopologyOptimization {
    const network = this.networks.get(networkId);
    if (!network) {
      throw new MeshNetworkError(
        MeshErrorCode.NODE_NOT_FOUND,
        `Network ${networkId} not found`
      );
    }

    const originalCost = this.calculateNetworkCost(network, metric);

    // Generate optimization suggestions
    const suggestions = this.generateOptimizationSuggestions(network, metric);

    // Calculate optimized cost (simulated)
    const optimizedCost = originalCost * 0.75; // 25% improvement (simulated)

    return {
      metric,
      originalCost,
      optimizedCost,
      improvement: ((originalCost - optimizedCost) / originalCost) * 100,
      suggestions,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Initialize network health
   */
  private initializeHealth(): NetworkHealth {
    return {
      score: 100,
      status: 'healthy',
      activeNodes: 0,
      failedNodes: 0,
      partitions: 0,
      avgLinkQuality: 1.0,
      utilization: 0.0,
      issues: [],
      lastChecked: new Date(),
    };
  }

  /**
   * Initialize network statistics
   */
  private initializeStats() {
    return {
      totalTxPackets: 0,
      totalRxPackets: 0,
      totalDroppedPackets: 0,
      totalTxBytes: 0,
      totalRxBytes: 0,
      avgLatency: 0,
      avgThroughput: 0,
      deliveryRatio: 1.0,
      uptime: 0,
    };
  }

  /**
   * Update network topology
   */
  private updateTopology(networkId: string): void {
    const network = this.networks.get(networkId);
    if (!network) return;

    // Update links based on node neighbors
    const links: Link[] = [];
    network.nodes.forEach((node) => {
      node.neighbors.forEach((neighbor) => {
        links.push({
          from: node.config.id,
          to: neighbor.id,
          quality: neighbor.linkQuality,
          bandwidth: this.estimateBandwidth(network.config.protocol),
          latency: this.calculateLatency(neighbor.rssi),
          packetLoss: 1 - neighbor.linkQuality,
          cost: this.calculateLinkCost(neighbor),
          bidirectional: true,
        });
      });
    });

    network.topology.links = links;
    network.topology.diameter = this.calculateDiameter(network);
    network.topology.avgHopCount = this.calculateAvgHopCount(network);
    network.topology.updated = new Date();
  }

  /**
   * Find path between source and destination
   */
  private findPath(
    network: MeshNetwork,
    source: string,
    destination: string,
    maxHops?: number
  ): Path | null {
    // Simple Dijkstra's algorithm for shortest path
    const distances = new Map<string, number>();
    const previous = new Map<string, string>();
    const unvisited = new Set(network.topology.nodes);

    // Initialize
    network.topology.nodes.forEach((nodeId) => {
      distances.set(nodeId, Infinity);
    });
    distances.set(source, 0);

    while (unvisited.size > 0) {
      // Find node with minimum distance
      let current: string | null = null;
      let minDistance = Infinity;

      unvisited.forEach((nodeId) => {
        const dist = distances.get(nodeId) || Infinity;
        if (dist < minDistance) {
          minDistance = dist;
          current = nodeId;
        }
      });

      if (current === null || current === destination) break;

      unvisited.delete(current);

      // Update neighbors
      const currentNode = network.nodes.get(current);
      if (currentNode) {
        currentNode.neighbors.forEach((neighbor) => {
          if (unvisited.has(neighbor.id)) {
            const alt = (distances.get(current!) || 0) + neighbor.linkQuality;
            if (alt < (distances.get(neighbor.id) || Infinity)) {
              distances.set(neighbor.id, alt);
              previous.set(neighbor.id, current!);
            }
          }
        });
      }
    }

    // Reconstruct path
    if (!previous.has(destination) && source !== destination) {
      return null;
    }

    const nodes: string[] = [];
    let current = destination;

    while (current !== source) {
      nodes.unshift(current);
      const prev = previous.get(current);
      if (!prev) break;
      current = prev;
    }
    nodes.unshift(source);

    // Check max hops
    if (maxHops && nodes.length - 1 > maxHops) {
      return null;
    }

    // Build path links
    const links: Link[] = [];
    for (let i = 0; i < nodes.length - 1; i++) {
      const link = network.topology.links.find(
        (l) => l.from === nodes[i] && l.to === nodes[i + 1]
      );
      if (link) links.push(link);
    }

    return {
      source,
      destination,
      nodes,
      links,
      totalCost: distances.get(destination) || 0,
      totalLatency: links.reduce((sum, l) => sum + l.latency, 0),
      minBandwidth: Math.min(...links.map((l) => l.bandwidth), Infinity),
      valid: true,
    };
  }

  /**
   * Calculate path latency
   */
  private calculatePathLatency(network: MeshNetwork, path: Path): number {
    return path.totalLatency;
  }

  /**
   * Calculate path bandwidth
   */
  private calculatePathBandwidth(network: MeshNetwork, path: Path): number {
    return path.minBandwidth;
  }

  /**
   * Calculate path cost
   */
  private calculatePathCost(network: MeshNetwork, path: Path): number {
    const weights = MESH_CONSTANTS.QOS_WEIGHTS;
    let totalCost = 0;

    path.links.forEach((link) => {
      const cost =
        weights.bandwidth * (1 / link.bandwidth) +
        weights.latency * link.latency +
        weights.packetLoss * link.packetLoss;
      totalCost += cost;
    });

    return totalCost;
  }

  /**
   * Update routing statistics
   */
  private updateRoutingStats(
    network: MeshNetwork,
    request: RoutingRequest,
    path: Path
  ): void {
    network.stats.totalTxPackets++;
    network.stats.totalTxBytes += request.data.length;

    // Update node statistics
    path.nodes.forEach((nodeId) => {
      const node = network.nodes.get(nodeId);
      if (node && node.stats) {
        node.stats.txPackets++;
        node.stats.txBytes += request.data.length;
        if (nodeId !== request.source && nodeId !== request.destination) {
          node.stats.fwdPackets++;
        }
      }
    });
  }

  /**
   * Update health metrics
   */
  private updateHealthMetrics(network: MeshNetwork): void {
    let activeNodes = 0;
    let failedNodes = 0;
    let totalLinkQuality = 0;
    let linkCount = 0;

    network.nodes.forEach((node) => {
      if (node.status === 'active') {
        activeNodes++;
      } else if (node.status === 'failed') {
        failedNodes++;
      }

      node.neighbors.forEach((neighbor) => {
        totalLinkQuality += neighbor.linkQuality;
        linkCount++;
      });
    });

    network.health.activeNodes = activeNodes;
    network.health.failedNodes = failedNodes;
    network.health.avgLinkQuality = linkCount > 0 ? totalLinkQuality / linkCount : 1.0;

    // Calculate health score
    const healthScore =
      100 *
      (network.health.avgLinkQuality * 0.4 +
        (activeNodes / (activeNodes + failedNodes || 1)) * 0.3 +
        (1 - network.health.utilization) * 0.3);

    network.health.score = Math.round(healthScore);

    if (healthScore >= 80) {
      network.health.status = 'healthy';
    } else if (healthScore >= 60) {
      network.health.status = 'degraded';
    } else if (healthScore >= 40) {
      network.health.status = 'critical';
    } else {
      network.health.status = 'failed';
    }

    network.health.lastChecked = new Date();
  }

  /**
   * Calculate network diameter
   */
  private calculateDiameter(network: MeshNetwork): number {
    // Simple estimation based on node count
    const nodeCount = network.nodes.size;
    if (nodeCount === 0) return 0;

    return Math.ceil(Math.sqrt(nodeCount));
  }

  /**
   * Calculate average hop count
   */
  private calculateAvgHopCount(network: MeshNetwork): number {
    const diameter = this.calculateDiameter(network);
    return diameter / 2;
  }

  /**
   * Calculate network cost
   */
  private calculateNetworkCost(
    network: MeshNetwork,
    metric: 'latency' | 'bandwidth' | 'power' | 'reliability'
  ): number {
    let cost = 0;

    switch (metric) {
      case 'latency':
        network.topology.links.forEach((link) => {
          cost += link.latency;
        });
        break;

      case 'bandwidth':
        network.topology.links.forEach((link) => {
          cost += 1 / link.bandwidth;
        });
        break;

      case 'power':
        // Estimate based on link count
        cost = network.topology.links.length * 100; // mW
        break;

      case 'reliability':
        network.topology.links.forEach((link) => {
          cost += link.packetLoss;
        });
        break;
    }

    return cost;
  }

  /**
   * Generate optimization suggestions
   */
  private generateOptimizationSuggestions(network: MeshNetwork, metric: string) {
    const suggestions = [];

    // Add channel optimization suggestion
    suggestions.push({
      type: 'change-channel' as const,
      details: `Switch to less congested channel for ${metric} optimization`,
      impact: 0.15,
    });

    // Add node placement suggestion
    if (network.nodes.size > 5) {
      suggestions.push({
        type: 'add-node' as const,
        details: `Add relay node to reduce network diameter and improve ${metric}`,
        impact: 0.20,
      });
    }

    return suggestions;
  }

  /**
   * Estimate peer count based on protocol
   */
  private estimatePeerCount(protocol: MeshProtocol): number {
    switch (protocol) {
      case 'wifi-mesh':
        return Math.floor(Math.random() * 10) + 5;
      case 'bluetooth-mesh':
        return Math.floor(Math.random() * 20) + 10;
      case 'thread':
        return Math.floor(Math.random() * 15) + 8;
      case 'lora-mesh':
        return Math.floor(Math.random() * 8) + 3;
      default:
        return Math.floor(Math.random() * 10) + 5;
    }
  }

  /**
   * Generate RSSI value
   */
  private generateRssi(protocol: MeshProtocol, threshold?: number): number {
    const min = threshold || -90;
    const max = -30;
    return Math.floor(Math.random() * (max - min) + min);
  }

  /**
   * Convert RSSI to LQI
   */
  private rssiToLqi(rssi: number): number {
    // Convert RSSI (-100 to -30 dBm) to LQI (0-255)
    const normalized = (rssi + 100) / 70;
    return Math.floor(Math.max(0, Math.min(1, normalized)) * 255);
  }

  /**
   * Generate capabilities
   */
  private generateCapabilities(protocol: MeshProtocol): string[] {
    const caps = ['routing'];

    if (Math.random() > 0.7) {
      caps.push('gateway');
    }

    if (protocol === 'thread' && Math.random() > 0.8) {
      caps.push('border-router');
    }

    return caps;
  }

  /**
   * Generate MAC address
   */
  private generateMacAddress(): string {
    const octets = [];
    for (let i = 0; i < 6; i++) {
      octets.push(Math.floor(Math.random() * 256).toString(16).padStart(2, '0'));
    }
    return octets.join(':').toUpperCase();
  }

  /**
   * Estimate bandwidth
   */
  private estimateBandwidth(protocol: MeshProtocol): number {
    switch (protocol) {
      case 'wifi-mesh':
        return 100; // Mbps
      case 'bluetooth-mesh':
        return 1; // Mbps
      case 'thread':
      case 'zigbee':
        return 0.25; // Mbps
      case 'lora-mesh':
        return 0.05; // Mbps
      default:
        return 10;
    }
  }

  /**
   * Calculate latency from RSSI
   */
  private calculateLatency(rssi: number): number {
    // Estimate latency based on signal strength
    // Better signal = lower latency
    const normalized = (rssi + 100) / 70;
    return 1 + (1 - normalized) * 10; // 1-11 ms
  }

  /**
   * Calculate link cost
   */
  private calculateLinkCost(neighbor: Neighbor): number {
    const weights = MESH_CONSTANTS.QOS_WEIGHTS;
    return (
      weights.bandwidth * (1 / neighbor.linkQuality) +
      weights.latency * this.calculateLatency(neighbor.rssi) +
      weights.packetLoss * (1 - neighbor.linkQuality)
    );
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create a mesh node (standalone function)
 */
export function createMeshNode(config: MeshNodeConfig): MeshNode {
  const sdk = new MeshNetworkSDK();
  return sdk.createMeshNode(config);
}

/**
 * Discover peers (standalone function)
 */
export async function discoverPeers(config: DiscoveryConfig): Promise<Peer[]> {
  const sdk = new MeshNetworkSDK();
  return sdk.discoverPeers(config);
}

/**
 * Calculate link cost
 */
export function calculateLinkCost(
  bandwidth: number,
  latency: number,
  packetLoss: number,
  weights = MESH_CONSTANTS.QOS_WEIGHTS
): number {
  return (
    weights.bandwidth * (1 / bandwidth) +
    weights.latency * latency +
    weights.packetLoss * packetLoss
  );
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { MeshNetworkSDK };
export default MeshNetworkSDK;
