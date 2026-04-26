/**
 * WIA-COMM-017: Mesh Network - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communications Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Mesh Types
// ============================================================================

/**
 * Supported mesh network protocols
 */
export type MeshProtocol = 'wifi-mesh' | 'bluetooth-mesh' | 'thread' | 'zigbee' | 'lora-mesh';

/**
 * Mesh node roles
 */
export type NodeRole = 'router' | 'gateway' | 'end-device' | 'border-router' | 'coordinator';

/**
 * Routing protocols
 */
export type RoutingProtocol = 'aodv' | 'olsr' | 'batman' | 'hwmp' | 'flooding' | 'distance-vector';

/**
 * Node status
 */
export type NodeStatus = 'active' | 'inactive' | 'sleeping' | 'failed' | 'initializing';

/**
 * Traffic priority levels
 */
export type Priority = 'background' | 'low' | 'medium' | 'high' | 'critical';

// ============================================================================
// Network Configuration
// ============================================================================

/**
 * Mesh network configuration
 */
export interface MeshNetworkConfig {
  /** Network identifier */
  networkId: string;

  /** Mesh protocol */
  protocol: MeshProtocol;

  /** Network name/SSID */
  name?: string;

  /** Channel number */
  channel?: number;

  /** Routing protocol */
  routingProtocol: RoutingProtocol;

  /** Security configuration */
  security: SecurityConfig;

  /** Maximum hop count */
  maxHops?: number;

  /** Auto-healing enabled */
  autoHealing?: boolean;

  /** QoS enabled */
  qosEnabled?: boolean;
}

/**
 * Security configuration
 */
export interface SecurityConfig {
  /** Encryption enabled */
  enabled: boolean;

  /** Encryption type */
  encryptionType?: 'wpa3-sae' | 'aes-ccm' | 'dtls' | 'none';

  /** Network key */
  networkKey?: string;

  /** Application key (Bluetooth Mesh) */
  appKey?: string;

  /** Authentication method */
  authMethod?: 'sae' | 'psk' | 'oob' | 'open';

  /** Pre-shared key */
  psk?: string;
}

// ============================================================================
// Mesh Nodes
// ============================================================================

/**
 * Mesh node configuration
 */
export interface MeshNodeConfig {
  /** Unique node identifier */
  id: string;

  /** Mesh protocol */
  protocol: MeshProtocol;

  /** Node role */
  role: NodeRole;

  /** Node capabilities */
  capabilities: string[];

  /** MAC address */
  macAddress?: string;

  /** IP address */
  ipAddress?: string;

  /** Mesh address (protocol-specific) */
  meshAddress?: string;

  /** Security configuration */
  security?: SecurityConfig;

  /** Transmit power (dBm) */
  txPower?: number;

  /** Maximum peer links */
  maxPeers?: number;
}

/**
 * Mesh node
 */
export interface MeshNode {
  /** Node configuration */
  config: MeshNodeConfig;

  /** List of neighbors */
  neighbors: Neighbor[];

  /** Routing table */
  routes: Route[];

  /** Node status */
  status: NodeStatus;

  /** Last seen timestamp */
  lastSeen: Date;

  /** Statistics */
  stats?: NodeStatistics;
}

/**
 * Node statistics
 */
export interface NodeStatistics {
  /** Packets transmitted */
  txPackets: number;

  /** Packets received */
  rxPackets: number;

  /** Packets forwarded */
  fwdPackets: number;

  /** Packets dropped */
  droppedPackets: number;

  /** Total bytes transmitted */
  txBytes: number;

  /** Total bytes received */
  rxBytes: number;

  /** Average RSSI */
  avgRssi: number;

  /** Uptime in seconds */
  uptime: number;
}

// ============================================================================
// Neighbors and Links
// ============================================================================

/**
 * Neighbor node
 */
export interface Neighbor {
  /** Neighbor node ID */
  id: string;

  /** MAC address */
  macAddress: string;

  /** Mesh address */
  meshAddress?: string;

  /** RSSI (Received Signal Strength Indicator) in dBm */
  rssi: number;

  /** LQI (Link Quality Indicator) 0-255 */
  lqi: number;

  /** Hop count to this neighbor */
  hopCount: number;

  /** Link quality (0-1) */
  linkQuality: number;

  /** Last seen timestamp */
  lastSeen: Date;

  /** Neighbor capabilities */
  capabilities?: string[];
}

/**
 * Link between two nodes
 */
export interface Link {
  /** Source node ID */
  from: string;

  /** Destination node ID */
  to: string;

  /** Link quality (0-1) */
  quality: number;

  /** Bandwidth in Mbps */
  bandwidth: number;

  /** Latency in milliseconds */
  latency: number;

  /** Packet loss rate (0-1) */
  packetLoss: number;

  /** Link cost metric */
  cost: number;

  /** Bidirectional link */
  bidirectional: boolean;
}

// ============================================================================
// Routing
// ============================================================================

/**
 * Route table entry
 */
export interface Route {
  /** Destination node ID */
  destination: string;

  /** Next hop node ID */
  nextHop: string;

  /** Route metric/cost */
  metric: number;

  /** Hop count */
  hopCount: number;

  /** Sequence number (AODV) */
  sequenceNumber?: number;

  /** Route lifetime */
  lifetime?: number;

  /** Route valid */
  valid: boolean;

  /** Last updated */
  updated: Date;
}

/**
 * Routing request
 */
export interface RoutingRequest {
  /** Source node ID */
  source: string;

  /** Destination node ID */
  destination: string;

  /** Data payload */
  data: Buffer;

  /** Traffic priority */
  priority: Priority;

  /** Maximum hop count */
  maxHops?: number;

  /** QoS configuration */
  qos?: QoSConfig;

  /** Timeout in milliseconds */
  timeout?: number;
}

/**
 * Routing result
 */
export interface RoutingResult {
  /** Routing success */
  success: boolean;

  /** Path taken (node IDs) */
  path: string[];

  /** Total latency in milliseconds */
  latency: number;

  /** Bandwidth in Mbps */
  bandwidth: number;

  /** Hop count */
  hopCount: number;

  /** Route cost */
  cost: number;

  /** Error message if failed */
  error?: string;
}

/**
 * Path information
 */
export interface Path {
  /** Source node */
  source: string;

  /** Destination node */
  destination: string;

  /** Nodes in path */
  nodes: string[];

  /** Links in path */
  links: Link[];

  /** Total cost */
  totalCost: number;

  /** Total latency */
  totalLatency: number;

  /** Minimum bandwidth */
  minBandwidth: number;

  /** Path valid */
  valid: boolean;
}

// ============================================================================
// Quality of Service (QoS)
// ============================================================================

/**
 * QoS configuration
 */
export interface QoSConfig {
  /** Traffic class */
  trafficClass: 'voice' | 'video' | 'data' | 'background';

  /** DSCP marking */
  dscp?: number;

  /** Bandwidth requirement in Mbps */
  bandwidthRequired?: number;

  /** Maximum latency in milliseconds */
  maxLatency?: number;

  /** Maximum jitter in milliseconds */
  maxJitter?: number;

  /** Maximum packet loss (0-1) */
  maxPacketLoss?: number;

  /** Priority level */
  priority: Priority;
}

/**
 * QoS statistics
 */
export interface QoSStatistics {
  /** Traffic class */
  trafficClass: string;

  /** Average latency */
  avgLatency: number;

  /** Average jitter */
  avgJitter: number;

  /** Packet loss rate */
  packetLoss: number;

  /** Throughput in Mbps */
  throughput: number;

  /** Packets delivered */
  packetsDelivered: number;

  /** Packets dropped */
  packetsDropped: number;
}

// ============================================================================
// Discovery and Pairing
// ============================================================================

/**
 * Discovery configuration
 */
export interface DiscoveryConfig {
  /** Mesh protocol */
  protocol: MeshProtocol;

  /** Discovery timeout in milliseconds */
  timeout: number;

  /** Channels to scan */
  channels?: number[];

  /** Minimum RSSI threshold */
  rssiThreshold?: number;

  /** Passive scan only */
  passive?: boolean;
}

/**
 * Discovered peer
 */
export interface Peer {
  /** Peer node ID */
  id: string;

  /** MAC address */
  address: string;

  /** Mesh address */
  meshAddress?: string;

  /** RSSI in dBm */
  rssi: number;

  /** LQI (0-255) */
  lqi: number;

  /** Peer capabilities */
  capabilities: string[];

  /** Network ID */
  networkId?: string;

  /** Last seen timestamp */
  lastSeen: Date;

  /** Protocol-specific data */
  metadata?: Record<string, unknown>;
}

/**
 * Pairing request
 */
export interface PairingRequest {
  /** Local node ID */
  localNodeId: string;

  /** Peer node ID */
  peerNodeId: string;

  /** Pairing method */
  method: 'auto' | 'manual' | 'oob';

  /** Authentication data */
  authData?: string;

  /** Timeout in milliseconds */
  timeout?: number;
}

/**
 * Pairing result
 */
export interface PairingResult {
  /** Pairing success */
  success: boolean;

  /** Paired peer */
  peer?: Peer;

  /** Shared keys */
  keys?: {
    networkKey?: string;
    pairwiseKey?: string;
  };

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// Network Topology
// ============================================================================

/**
 * Network topology
 */
export interface NetworkTopology {
  /** Network ID */
  networkId: string;

  /** Protocol */
  protocol: MeshProtocol;

  /** All nodes in network */
  nodes: string[];

  /** All links in network */
  links: Link[];

  /** Gateway nodes */
  gateways: string[];

  /** Network diameter (max hops) */
  diameter: number;

  /** Average hop count */
  avgHopCount: number;

  /** Node degree distribution */
  degreeDistribution?: Record<number, number>;

  /** Topology updated timestamp */
  updated: Date;
}

/**
 * Topology optimization result
 */
export interface TopologyOptimization {
  /** Optimization metric */
  metric: 'latency' | 'bandwidth' | 'power' | 'reliability';

  /** Original topology cost */
  originalCost: number;

  /** Optimized topology cost */
  optimizedCost: number;

  /** Improvement percentage */
  improvement: number;

  /** Suggested changes */
  suggestions: TopologyChange[];
}

/**
 * Topology change suggestion
 */
export interface TopologyChange {
  /** Change type */
  type: 'add-node' | 'remove-node' | 'move-node' | 'adjust-power' | 'change-channel';

  /** Affected node ID */
  nodeId?: string;

  /** Change details */
  details: string;

  /** Expected impact */
  impact: number;
}

// ============================================================================
// Monitoring and Health
// ============================================================================

/**
 * Network health status
 */
export interface NetworkHealth {
  /** Overall health score (0-100) */
  score: number;

  /** Health status */
  status: 'healthy' | 'degraded' | 'critical' | 'failed';

  /** Active nodes */
  activeNodes: number;

  /** Failed nodes */
  failedNodes: number;

  /** Network partitions */
  partitions: number;

  /** Average link quality */
  avgLinkQuality: number;

  /** Network utilization (0-1) */
  utilization: number;

  /** Issues detected */
  issues: HealthIssue[];

  /** Last checked */
  lastChecked: Date;
}

/**
 * Health issue
 */
export interface HealthIssue {
  /** Issue severity */
  severity: 'info' | 'warning' | 'error' | 'critical';

  /** Issue type */
  type: string;

  /** Issue description */
  description: string;

  /** Affected nodes */
  affectedNodes?: string[];

  /** Recommended action */
  recommendation?: string;

  /** Detected timestamp */
  detected: Date;
}

/**
 * Network statistics
 */
export interface NetworkStatistics {
  /** Total packets transmitted */
  totalTxPackets: number;

  /** Total packets received */
  totalRxPackets: number;

  /** Total packets dropped */
  totalDroppedPackets: number;

  /** Total bytes transmitted */
  totalTxBytes: number;

  /** Total bytes received */
  totalRxBytes: number;

  /** Average latency */
  avgLatency: number;

  /** Average throughput in Mbps */
  avgThroughput: number;

  /** Packet delivery ratio */
  deliveryRatio: number;

  /** Network uptime in seconds */
  uptime: number;
}

// ============================================================================
// Self-Healing
// ============================================================================

/**
 * Failure detection event
 */
export interface FailureEvent {
  /** Failure type */
  type: 'link-failure' | 'node-failure' | 'partition' | 'congestion';

  /** Failed node/link ID */
  targetId: string;

  /** Failure detection time */
  detected: Date;

  /** Affected routes */
  affectedRoutes: string[];

  /** Healing status */
  healingStatus: 'pending' | 'in-progress' | 'completed' | 'failed';

  /** Healing actions taken */
  actions: HealingAction[];
}

/**
 * Healing action
 */
export interface HealingAction {
  /** Action type */
  type: 'route-repair' | 'path-switch' | 'power-increase' | 'add-relay';

  /** Action description */
  description: string;

  /** Execution time */
  executed: Date;

  /** Action success */
  success: boolean;

  /** Result details */
  result?: string;
}

// ============================================================================
// WiFi Mesh (802.11s) Specific
// ============================================================================

/**
 * WiFi mesh configuration
 */
export interface WiFiMeshConfig {
  /** Mesh ID (SSID) */
  meshId: string;

  /** Channel number */
  channel: number;

  /** Channel width */
  channelWidth: 20 | 40 | 80 | 160;

  /** Frequency in MHz */
  frequency: number;

  /** Beacon interval in ms */
  beaconInterval: number;

  /** Max peer links */
  maxPeerLinks: number;

  /** HWMP root mode */
  hwmpRootMode: 0 | 1 | 2 | 3 | 4;

  /** Path refresh time in ms */
  pathRefreshTime: number;

  /** Encryption (SAE or open) */
  encryption: 'sae' | 'open';

  /** Mesh password (for SAE) */
  meshPassword?: string;
}

// ============================================================================
// Bluetooth Mesh Specific
// ============================================================================

/**
 * Bluetooth mesh configuration
 */
export interface BluetoothMeshConfig {
  /** Network name */
  networkName: string;

  /** Network key (128-bit hex) */
  netKey: string;

  /** Application keys */
  appKeys: string[];

  /** IV index */
  ivIndex: number;

  /** Unicast address range */
  unicastAddressRange: {
    start: number;
    end: number;
  };

  /** Default TTL */
  defaultTTL: number;

  /** Relay enabled */
  relayEnabled: boolean;

  /** Friend feature enabled */
  friendEnabled: boolean;

  /** Low power feature enabled */
  lowPowerEnabled: boolean;
}

/**
 * Bluetooth mesh element
 */
export interface BluetoothMeshElement {
  /** Element index */
  index: number;

  /** Unicast address */
  unicastAddress: number;

  /** Location descriptor */
  location: number;

  /** Models */
  models: BluetoothMeshModel[];
}

/**
 * Bluetooth mesh model
 */
export interface BluetoothMeshModel {
  /** Model ID */
  modelId: number;

  /** Vendor ID (for vendor models) */
  vendorId?: number;

  /** Bound app keys */
  appKeys: number[];

  /** Subscribed addresses */
  subscriptions: number[];

  /** Publication settings */
  publication?: {
    address: number;
    period: number;
    ttl: number;
  };
}

// ============================================================================
// Thread Mesh Specific
// ============================================================================

/**
 * Thread network configuration
 */
export interface ThreadNetworkConfig {
  /** Network name */
  networkName: string;

  /** Extended PAN ID (64-bit) */
  extendedPanId: string;

  /** PAN ID (16-bit) */
  panId: number;

  /** Channel */
  channel: number;

  /** Network key (128-bit) */
  networkKey: string;

  /** Mesh-local prefix */
  meshLocalPrefix: string;

  /** On-mesh prefix */
  onMeshPrefix?: string;

  /** Commissioner enabled */
  commissionerEnabled: boolean;

  /** Border router */
  borderRouter?: boolean;
}

// ============================================================================
// LoRa Mesh Specific
// ============================================================================

/**
 * LoRa mesh configuration
 */
export interface LoRaMeshConfig {
  /** Network ID */
  networkId: number;

  /** Spreading factor (7-12) */
  spreadingFactor: 7 | 8 | 9 | 10 | 11 | 12;

  /** Bandwidth in kHz */
  bandwidth: 125 | 250 | 500;

  /** Coding rate */
  codingRate: '4/5' | '4/6' | '4/7' | '4/8';

  /** Frequency in MHz */
  frequency: number;

  /** Transmit power in dBm */
  txPower: number;

  /** Preamble length */
  preambleLength: number;

  /** Sync word */
  syncWord: number;

  /** Duty cycle (0-1) */
  dutyCycle?: number;
}

/**
 * LoRa link budget
 */
export interface LoRaLinkBudget {
  /** Transmit power in dBm */
  txPower: number;

  /** Receiver sensitivity in dBm */
  rxSensitivity: number;

  /** Path loss in dB */
  pathLoss: number;

  /** Fade margin in dB */
  fadeMargin: number;

  /** Link margin in dB */
  linkMargin: number;

  /** Estimated range in meters */
  estimatedRange: number;
}

// ============================================================================
// Mesh Network
// ============================================================================

/**
 * Complete mesh network
 */
export interface MeshNetwork {
  /** Network configuration */
  config: MeshNetworkConfig;

  /** All nodes in network */
  nodes: Map<string, MeshNode>;

  /** Network topology */
  topology: NetworkTopology;

  /** Network health */
  health: NetworkHealth;

  /** Network statistics */
  stats: NetworkStatistics;

  /** Creation timestamp */
  created: Date;

  /** Last updated */
  updated: Date;
}

// ============================================================================
// Physical Constants and Limits
// ============================================================================

/**
 * Mesh networking constants
 */
export const MESH_CONSTANTS = {
  /** WiFi range in meters */
  WIFI_RANGE: { min: 50, typical: 150, max: 300 },

  /** Bluetooth range in meters */
  BLE_RANGE: { min: 10, typical: 30, max: 50 },

  /** Thread range in meters */
  THREAD_RANGE: { min: 10, typical: 50, max: 100 },

  /** LoRa range in meters */
  LORA_RANGE: { min: 2000, typical: 8000, max: 15000 },

  /** Maximum recommended hops */
  MAX_HOPS: 10,

  /** Minimum neighbor count */
  MIN_NEIGHBORS: 2,

  /** Recommended neighbor count */
  RECOMMENDED_NEIGHBORS: 3,

  /** Routing overhead percentage */
  ROUTING_OVERHEAD: 0.15,

  /** Default QoS weights */
  QOS_WEIGHTS: {
    bandwidth: 0.4,
    latency: 0.3,
    packetLoss: 0.3,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-COMM-017 error codes
 */
export enum MeshErrorCode {
  NODE_NOT_FOUND = 'M001',
  ROUTE_NOT_AVAILABLE = 'M002',
  INSUFFICIENT_BANDWIDTH = 'M003',
  SECURITY_FAILURE = 'M004',
  NETWORK_PARTITION = 'M005',
  HOP_LIMIT_EXCEEDED = 'M006',
  INVALID_PROTOCOL = 'M007',
  DISCOVERY_TIMEOUT = 'M008',
  PAIRING_FAILED = 'M009',
  TOPOLOGY_ERROR = 'M010',
}

/**
 * Mesh network error
 */
export class MeshNetworkError extends Error {
  constructor(
    public code: MeshErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MeshNetworkError';
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
// Export All
// ============================================================================

export type {
  // Configuration
  MeshNetworkConfig,
  SecurityConfig,
  MeshNodeConfig,

  // Nodes
  MeshNode,
  NodeStatistics,
  Neighbor,
  Link,

  // Routing
  Route,
  RoutingRequest,
  RoutingResult,
  Path,

  // QoS
  QoSConfig,
  QoSStatistics,

  // Discovery
  DiscoveryConfig,
  Peer,
  PairingRequest,
  PairingResult,

  // Topology
  NetworkTopology,
  TopologyOptimization,
  TopologyChange,

  // Monitoring
  NetworkHealth,
  HealthIssue,
  NetworkStatistics,

  // Self-Healing
  FailureEvent,
  HealingAction,

  // Protocol-specific
  WiFiMeshConfig,
  BluetoothMeshConfig,
  BluetoothMeshElement,
  BluetoothMeshModel,
  ThreadNetworkConfig,
  LoRaMeshConfig,
  LoRaLinkBudget,

  // Network
  MeshNetwork,
};

export { MESH_CONSTANTS, MeshErrorCode, MeshNetworkError };
