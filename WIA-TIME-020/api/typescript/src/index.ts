/**
 * WIA-TIME-020: Temporal Beacon SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for temporal beacon deployment, positioning,
 * navigation, and emergency response across time and space.
 */

import {
  TemporalBeacon,
  BeaconType,
  BeaconStatus,
  BeaconSignal,
  BeaconNetwork,
  TemporalPosition,
  TriangulationParams,
  TemporalWaypoint,
  NavigationRoute,
  RoutePlanningParams,
  EmergencyBeacon,
  EmergencyResponse,
  BeaconDeployment,
  DeploymentResult,
  CoverageMap,
  CoveragePoint,
  CalibrationData,
  SyncResult,
  Vector3,
  SpacetimeCoordinates,
  BEACON_CONSTANTS,
  BeaconErrorCode,
  BeaconError,
  BeaconHealth,
  NetworkHealth,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-TIME-020 Temporal Beacon SDK
 */
export class TemporalBeaconSDK {
  private version = '1.0.0';
  private beacons: Map<string, TemporalBeacon> = new Map();
  private networks: Map<string, BeaconNetwork> = new Map();

  constructor() {
    // Initialize SDK
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Deploy a new temporal beacon
   *
   * @param deployment - Beacon deployment parameters
   * @returns Deployment result with coverage estimate
   */
  deployBeacon(deployment: BeaconDeployment): DeploymentResult {
    const {
      id,
      type,
      position,
      temporalAnchor,
      signalPower,
      range,
      frequency,
      networkId,
      priority = 1,
      metadata,
    } = deployment;

    // Validate deployment parameters
    if (signalPower <= 0) {
      throw new BeaconError(
        BeaconErrorCode.INVALID_DEPLOYMENT,
        'Signal power must be positive'
      );
    }

    if (range.spatial <= 0 || range.temporal <= 0) {
      throw new BeaconError(
        BeaconErrorCode.INVALID_DEPLOYMENT,
        'Range values must be positive'
      );
    }

    // Create beacon
    const beacon: TemporalBeacon = {
      id,
      type,
      position,
      temporalAnchor: new Date(temporalAnchor),
      signal: {
        frequency,
        power: signalPower,
        modulation: 'TPSK',
        phaseOffset: this.calculatePhaseOffset(id),
      },
      range,
      network: {
        networkId,
        level: this.determineNetworkLevel(type),
        priority,
      },
      status: 'active',
      deployed: new Date(),
      metadata,
    };

    // Store beacon
    this.beacons.set(id, beacon);

    // Add to network
    this.addBeaconToNetwork(networkId, beacon);

    // Estimate coverage
    const estimatedCoverage = this.estimateCoverage(beacon);

    return {
      success: true,
      beaconId: id,
      activationTime: new Date(),
      estimatedCoverage,
      warnings: this.validateBeaconPlacement(beacon),
    };
  }

  /**
   * Calculate position from multiple beacon signals (triangulation)
   *
   * @param params - Triangulation parameters
   * @returns Calculated temporal position
   */
  triangulatePosition(params: TriangulationParams): TemporalPosition {
    const {
      signals,
      beacons,
      minBeacons = 4,
      weighted = true,
      kalman = false,
      previousPosition,
    } = params;

    // Validate minimum beacons
    if (signals.length < minBeacons) {
      throw new BeaconError(
        BeaconErrorCode.TOO_FEW_BEACONS,
        `Need at least ${minBeacons} beacons, got ${signals.length}`
      );
    }

    // Filter valid signals (strong enough)
    const validSignals = signals.filter(
      (s) => s.signalStrength >= 0.1 // 10% minimum
    );

    if (validSignals.length < minBeacons) {
      throw new BeaconError(
        BeaconErrorCode.INSUFFICIENT_SIGNAL,
        'Too few strong signals for positioning'
      );
    }

    // Get beacon positions
    const beaconMap = new Map(beacons.map((b) => [b.id, b]));

    // Calculate distances from time delays
    const measurements = validSignals.map((signal) => {
      const beacon = beaconMap.get(signal.beaconId);
      if (!beacon) {
        throw new BeaconError(
          BeaconErrorCode.BEACON_OFFLINE,
          `Beacon ${signal.beaconId} not found`
        );
      }

      const distance = (BEACON_CONSTANTS.SPEED_OF_LIGHT * signal.timeDelay) / 2;

      return {
        beacon,
        distance,
        weight: weighted ? signal.signalStrength : 1.0,
      };
    });

    // Solve position using least squares
    const position = weighted
      ? this.weightedLeastSquares(measurements)
      : this.leastSquares(measurements);

    // Apply Kalman filter if requested
    const finalPosition = kalman && previousPosition
      ? this.kalmanFilter(position, previousPosition)
      : position;

    // Calculate GDOP
    const GDOP = this.calculateGDOP(measurements.map((m) => m.beacon));

    // Calculate accuracy
    const timingAccuracy = 1e-9; // 1 nanosecond
    const spatialAccuracy =
      BEACON_CONSTANTS.SPEED_OF_LIGHT * timingAccuracy * GDOP;
    const temporalAccuracy = timingAccuracy * GDOP;

    return {
      coordinates: finalPosition.coordinates,
      time: finalPosition.time,
      accuracy: {
        spatial: spatialAccuracy,
        temporal: temporalAccuracy,
      },
      confidence: this.calculateConfidence(validSignals, GDOP),
      beaconsUsed: validSignals.map((s) => s.beaconId),
      GDOP,
      method: weighted ? 'weighted_least_squares' : 'trilateration',
      rawSignals: validSignals,
    };
  }

  /**
   * Deploy emergency beacon with distress signal
   *
   * @param emergency - Emergency beacon configuration
   * @returns Emergency response information
   */
  deployEmergencyBeacon(emergency: EmergencyBeacon): EmergencyResponse {
    const { id, severity, type, position, message, vitals, systemStatus } =
      emergency;

    // Transmit on emergency frequency
    const emergencyFreq = BEACON_CONSTANTS.FREQUENCIES.EMERGENCY;
    const emergencyPower = BEACON_CONSTANTS.POWER.EMERGENCY;

    // Create emergency beacon
    const beacon: TemporalBeacon = {
      id: `EMERGENCY-${id}`,
      type: 'emergency',
      position: position.position,
      temporalAnchor: new Date(position.time),
      signal: {
        frequency: emergencyFreq,
        power: emergencyPower,
        modulation: 'TPSK',
      },
      range: BEACON_CONSTANTS.RANGE.PRIMARY,
      network: {
        networkId: 'EMERGENCY-NETWORK',
        level: 0,
        priority: 999, // Highest priority
      },
      status: 'emergency',
      deployed: new Date(),
    };

    // Store emergency beacon
    this.beacons.set(beacon.id, beacon);

    // Simulate emergency response
    const response: EmergencyResponse = {
      responseId: `RESP-${Date.now()}`,
      emergencyId: id,
      estimatedArrival: this.calculateResponseTime(severity, position),
      rescueAssets: this.mobilizeRescueAssets(severity, position),
      instructions: this.generateEmergencyInstructions(type, systemStatus),
      status: 'dispatched',
      contactFrequency: emergencyFreq,
    };

    return response;
  }

  /**
   * Plan optimal route between two points
   *
   * @param params - Route planning parameters
   * @returns Optimal navigation route
   */
  planRoute(params: RoutePlanningParams): NavigationRoute {
    const {
      start,
      destination,
      waypoints = [],
      maxRisk = 'medium',
      optimize = 'shortest',
      avoid = [],
    } = params;

    // Convert start/destination to waypoints if needed
    const startWaypoint = this.toWaypoint(start, 'start');
    const destWaypoint = this.toWaypoint(destination, 'destination');

    // Filter available waypoints
    const availableWaypoints = waypoints.filter(
      (wp) => !avoid.includes(wp.id)
    );

    // Build graph
    const graph = this.buildRouteGraph([
      startWaypoint,
      ...availableWaypoints,
      destWaypoint,
    ]);

    // Find optimal path (A* or Dijkstra)
    const path =
      optimize === 'shortest'
        ? this.dijkstra(graph, startWaypoint.id, destWaypoint.id)
        : this.aStar(graph, startWaypoint.id, destWaypoint.id);

    // Calculate route properties
    const routeWaypoints = path.map((id) =>
      [startWaypoint, ...availableWaypoints, destWaypoint].find(
        (wp) => wp.id === id
      )
    ) as TemporalWaypoint[];

    const totalDistance = this.calculateTotalDistance(routeWaypoints);
    const riskLevel = this.assessRouteRisk(routeWaypoints);
    const safetyScore = this.calculateSafetyScore(routeWaypoints);

    return {
      id: `ROUTE-${Date.now()}`,
      waypoints: routeWaypoints,
      totalDistance,
      estimatedDuration: totalDistance.temporal,
      riskLevel,
      safetyScore,
    };
  }

  /**
   * Get beacon network by ID
   */
  getNetwork(networkId: string): BeaconNetwork | undefined {
    return this.networks.get(networkId);
  }

  /**
   * Create a new beacon network
   */
  createNetwork(
    networkId: string,
    name: string,
    coverage: 'global' | 'continental' | 'regional' | 'local'
  ): BeaconNetwork {
    const network: BeaconNetwork = {
      networkId,
      name,
      coverage,
      temporalRange: {
        start: new Date('2000-01-01'),
        end: new Date('2100-12-31'),
      },
      beacons: [],
      syncProtocol: 'PTP',
      status: 'operational',
    };

    this.networks.set(networkId, network);
    return network;
  }

  /**
   * Synchronize beacon network clocks
   */
  synchronizeNetwork(networkId: string): SyncResult {
    const network = this.networks.get(networkId);
    if (!network) {
      throw new BeaconError(
        BeaconErrorCode.NETWORK_PARTITION,
        `Network ${networkId} not found`
      );
    }

    const masterBeacon = network.masterBeacon || network.beacons[0]?.id;
    if (!masterBeacon) {
      throw new BeaconError(
        BeaconErrorCode.NETWORK_PARTITION,
        'No master beacon in network'
      );
    }

    // Simulate synchronization
    const syncErrors = network.beacons.map(() => Math.random() * 5); // 0-5 ns
    const maxSyncError = Math.max(...syncErrors);
    const avgSyncError = syncErrors.reduce((a, b) => a + b, 0) / syncErrors.length;

    return {
      beacons: network.beacons.map((b) => b.id),
      masterBeacon,
      timestamp: new Date(),
      maxSyncError,
      avgSyncError,
      success: maxSyncError <= BEACON_CONSTANTS.SYNC_ACCURACY * 10,
    };
  }

  /**
   * Calibrate beacon
   */
  calibrateBeacon(beaconId: string): CalibrationData {
    const beacon = this.beacons.get(beaconId);
    if (!beacon) {
      throw new BeaconError(
        BeaconErrorCode.BEACON_OFFLINE,
        `Beacon ${beaconId} not found`
      );
    }

    // Simulate calibration measurements
    const frequencyDrift = (Math.random() - 0.5) * 2000; // ±1000 Hz
    const powerVariation = (Math.random() - 0.5) * 0.1; // ±5%
    const syncError = Math.random() * 20; // 0-20 ns
    const positionDrift = Math.random() * 2; // 0-2 meters

    const passed =
      Math.abs(frequencyDrift) <= BEACON_CONSTANTS.CALIBRATION.MAX_FREQ_DRIFT &&
      Math.abs(powerVariation) <= BEACON_CONSTANTS.CALIBRATION.MAX_POWER_VAR &&
      syncError <= BEACON_CONSTANTS.CALIBRATION.MAX_SYNC_ERROR &&
      positionDrift <= BEACON_CONSTANTS.CALIBRATION.MAX_POS_DRIFT;

    return {
      beaconId,
      timestamp: new Date(),
      frequencyDrift,
      powerVariation,
      syncError,
      positionDrift,
      passed,
      adjustments: passed
        ? undefined
        : {
            frequency: -frequencyDrift,
            power: -powerVariation,
          },
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Calculate phase offset from beacon ID
   */
  private calculatePhaseOffset(id: string): number {
    // Simple hash function for demo
    let hash = 0;
    for (let i = 0; i < id.length; i++) {
      hash = (hash << 5) - hash + id.charCodeAt(i);
      hash |= 0;
    }
    return (hash % 360) * (Math.PI / 180); // Convert to radians
  }

  /**
   * Determine network level from beacon type
   */
  private determineNetworkLevel(type: BeaconType): number {
    const levels: Record<BeaconType, number> = {
      fixed_primary: 0,
      fixed_secondary: 1,
      mobile: 2,
      emergency: 0,
      micro: 3,
    };
    return levels[type];
  }

  /**
   * Add beacon to network
   */
  private addBeaconToNetwork(networkId: string, beacon: TemporalBeacon): void {
    let network = this.networks.get(networkId);
    if (!network) {
      // Create network if it doesn't exist
      network = this.createNetwork(networkId, networkId, 'regional');
    }
    network.beacons.push(beacon);
  }

  /**
   * Estimate coverage for a beacon
   */
  private estimateCoverage(beacon: TemporalBeacon): CoverageMap {
    // Simplified coverage estimation
    const grid: CoveragePoint[] = [];
    const resolution = { spatial: 1000, temporal: 86400 }; // 1km, 1 day

    // Sample a few points around the beacon
    for (let dx = -beacon.range.spatial; dx <= beacon.range.spatial; dx += resolution.spatial * 10) {
      for (let dy = -beacon.range.spatial; dy <= beacon.range.spatial; dy += resolution.spatial * 10) {
        const distance = Math.sqrt(dx * dx + dy * dy);
        if (distance <= beacon.range.spatial) {
          const signalStrength = this.calculateSignalStrength(
            beacon,
            distance,
            0
          );

          grid.push({
            position: {
              position: {
                x: beacon.position.x + dx,
                y: beacon.position.y + dy,
                z: beacon.position.z,
              },
              time: beacon.temporalAnchor,
            },
            beaconCount: signalStrength > 0.1 ? 1 : 0,
            quality: this.determineQuality(1),
            avgSignalStrength: signalStrength,
            GDOP: 5.0, // Placeholder
          });
        }
      }
    }

    return {
      networkId: beacon.network.networkId,
      grid,
      resolution,
      statistics: {
        totalVolume: Math.PI * Math.pow(beacon.range.spatial, 2),
        excellentCoverage: 0,
        goodCoverage: 50,
        poorCoverage: 50,
      },
    };
  }

  /**
   * Calculate signal strength at distance
   */
  private calculateSignalStrength(
    beacon: TemporalBeacon,
    spatialDistance: number,
    temporalDistance: number
  ): number {
    // Spatial attenuation: 1/(4πr²)
    const spatialAtten = 1 / (4 * Math.PI * spatialDistance * spatialDistance);

    // Temporal attenuation: e^(-αΔt)
    const temporalAtten = Math.exp(
      -BEACON_CONSTANTS.TEMPORAL_ATTENUATION * Math.abs(temporalDistance)
    );

    return spatialAtten * temporalAtten;
  }

  /**
   * Validate beacon placement
   */
  private validateBeaconPlacement(beacon: TemporalBeacon): string[] {
    const warnings: string[] = [];

    // Check for nearby beacons on same frequency
    for (const [id, other] of this.beacons) {
      if (id === beacon.id) continue;
      if (other.signal.frequency === beacon.signal.frequency) {
        const distance = this.spatialDistance(
          beacon.position,
          other.position
        );
        if (distance < beacon.range.spatial * 0.1) {
          warnings.push(
            `Beacon ${id} is very close and may cause interference`
          );
        }
      }
    }

    return warnings;
  }

  /**
   * Least squares position solver
   */
  private leastSquares(
    measurements: Array<{ beacon: TemporalBeacon; distance: number }>
  ): { coordinates: Vector3; time: Date } {
    // Simplified least squares (in real implementation, use proper matrix math)
    // For now, just average the beacon positions weighted by inverse distance
    let totalWeight = 0;
    let sumX = 0,
      sumY = 0,
      sumZ = 0;

    for (const m of measurements) {
      const weight = 1 / (m.distance + 1);
      totalWeight += weight;
      sumX += m.beacon.position.x * weight;
      sumY += m.beacon.position.y * weight;
      sumZ += m.beacon.position.z * weight;
    }

    return {
      coordinates: {
        x: sumX / totalWeight,
        y: sumY / totalWeight,
        z: sumZ / totalWeight,
      },
      time: new Date(),
    };
  }

  /**
   * Weighted least squares solver
   */
  private weightedLeastSquares(
    measurements: Array<{
      beacon: TemporalBeacon;
      distance: number;
      weight: number;
    }>
  ): { coordinates: Vector3; time: Date } {
    let totalWeight = 0;
    let sumX = 0,
      sumY = 0,
      sumZ = 0;

    for (const m of measurements) {
      const combinedWeight = m.weight / (m.distance + 1);
      totalWeight += combinedWeight;
      sumX += m.beacon.position.x * combinedWeight;
      sumY += m.beacon.position.y * combinedWeight;
      sumZ += m.beacon.position.z * combinedWeight;
    }

    return {
      coordinates: {
        x: sumX / totalWeight,
        y: sumY / totalWeight,
        z: sumZ / totalWeight,
      },
      time: new Date(),
    };
  }

  /**
   * Kalman filter for position smoothing
   */
  private kalmanFilter(
    current: { coordinates: Vector3; time: Date },
    previous: TemporalPosition
  ): { coordinates: Vector3; time: Date } {
    // Simplified Kalman filter (alpha blending)
    const alpha = 0.7; // Trust current measurement 70%

    return {
      coordinates: {
        x: alpha * current.coordinates.x + (1 - alpha) * previous.coordinates.x,
        y: alpha * current.coordinates.y + (1 - alpha) * previous.coordinates.y,
        z: alpha * current.coordinates.z + (1 - alpha) * previous.coordinates.z,
      },
      time: current.time,
    };
  }

  /**
   * Calculate GDOP (Geometric Dilution of Precision)
   */
  private calculateGDOP(beacons: TemporalBeacon[]): number {
    // Simplified GDOP calculation
    // In practice, compute from geometry matrix
    if (beacons.length < 4) return 100; // Very poor
    if (beacons.length >= 8) return 1.5; // Excellent
    return 3.0; // Good
  }

  /**
   * Calculate confidence from signals and GDOP
   */
  private calculateConfidence(signals: BeaconSignal[], GDOP: number): number {
    const avgSignalStrength =
      signals.reduce((sum, s) => sum + s.signalStrength, 0) / signals.length;
    const gdopFactor = Math.max(0, 1 - GDOP / 10);
    return avgSignalStrength * gdopFactor;
  }

  /**
   * Calculate emergency response time
   */
  private calculateResponseTime(
    severity: string,
    position: SpacetimeCoordinates
  ): number {
    const baseTimes: Record<string, number> = {
      CRITICAL: 300, // 5 minutes
      HIGH: 1800, // 30 minutes
      MEDIUM: 7200, // 2 hours
      LOW: 86400, // 24 hours
    };
    return baseTimes[severity] || 3600;
  }

  /**
   * Mobilize rescue assets
   */
  private mobilizeRescueAssets(
    severity: string,
    position: SpacetimeCoordinates
  ): any[] {
    return [
      {
        id: 'RESCUE-1',
        type: 'temporal_vehicle',
        position,
        eta: this.calculateResponseTime(severity, position),
        status: 'deployed',
      },
    ];
  }

  /**
   * Generate emergency instructions
   */
  private generateEmergencyInstructions(
    type: string,
    systemStatus?: any
  ): string {
    const instructions: Record<string, string> = {
      DISPLACEMENT:
        'Stay calm. Do not attempt further temporal displacement. Await rescue.',
      ENERGY:
        'Conserve remaining energy. Disable non-essential systems. Rescue en route.',
      PARADOX:
        'Minimize interactions with local timeline. Await temporal repair team.',
      MEDICAL: 'Administer first aid if able. Rescue team includes medic.',
      EQUIPMENT: 'Document failure. Switch to backup systems if available.',
    };
    return instructions[type] || 'Stay calm. Help is on the way.';
  }

  /**
   * Convert to waypoint
   */
  private toWaypoint(
    point: TemporalWaypoint | SpacetimeCoordinates,
    label: string
  ): TemporalWaypoint {
    if ('id' in point) {
      return point;
    }
    return {
      id: `WP-${label}`,
      name: label,
      coordinates: point,
      importance: 'medium',
      safetyLevel: 'safe',
      type: 'navigation',
    };
  }

  /**
   * Build route graph
   */
  private buildRouteGraph(waypoints: TemporalWaypoint[]): Map<string, Map<string, number>> {
    const graph = new Map<string, Map<string, number>>();

    for (const wp of waypoints) {
      graph.set(wp.id, new Map());
    }

    // Connect all waypoints (fully connected graph for simplicity)
    for (let i = 0; i < waypoints.length; i++) {
      for (let j = i + 1; j < waypoints.length; j++) {
        const distance = this.waypointDistance(waypoints[i], waypoints[j]);
        graph.get(waypoints[i].id)!.set(waypoints[j].id, distance);
        graph.get(waypoints[j].id)!.set(waypoints[i].id, distance);
      }
    }

    return graph;
  }

  /**
   * Dijkstra's shortest path
   */
  private dijkstra(
    graph: Map<string, Map<string, number>>,
    start: string,
    end: string
  ): string[] {
    const distances = new Map<string, number>();
    const previous = new Map<string, string>();
    const unvisited = new Set(graph.keys());

    for (const node of graph.keys()) {
      distances.set(node, node === start ? 0 : Infinity);
    }

    while (unvisited.size > 0) {
      // Find node with minimum distance
      let minNode: string | null = null;
      let minDist = Infinity;
      for (const node of unvisited) {
        const dist = distances.get(node)!;
        if (dist < minDist) {
          minDist = dist;
          minNode = node;
        }
      }

      if (!minNode || minNode === end) break;

      unvisited.delete(minNode);

      // Update neighbors
      const neighbors = graph.get(minNode)!;
      for (const [neighbor, weight] of neighbors) {
        if (!unvisited.has(neighbor)) continue;
        const newDist = distances.get(minNode)! + weight;
        if (newDist < distances.get(neighbor)!) {
          distances.set(neighbor, newDist);
          previous.set(neighbor, minNode);
        }
      }
    }

    // Reconstruct path
    const path: string[] = [];
    let current: string | undefined = end;
    while (current) {
      path.unshift(current);
      current = previous.get(current);
    }

    return path;
  }

  /**
   * A* pathfinding
   */
  private aStar(
    graph: Map<string, Map<string, number>>,
    start: string,
    end: string
  ): string[] {
    // Simplified: just use Dijkstra for now
    return this.dijkstra(graph, start, end);
  }

  /**
   * Calculate total route distance
   */
  private calculateTotalDistance(waypoints: TemporalWaypoint[]): {
    spatial: number;
    temporal: number;
  } {
    let spatial = 0;
    let temporal = 0;

    for (let i = 0; i < waypoints.length - 1; i++) {
      const d = this.waypointDistance(waypoints[i], waypoints[i + 1]);
      spatial += d;
      const t1 = new Date(waypoints[i].coordinates.time).getTime();
      const t2 = new Date(waypoints[i + 1].coordinates.time).getTime();
      temporal += Math.abs(t2 - t1) / 1000;
    }

    return { spatial, temporal };
  }

  /**
   * Assess route risk
   */
  private assessRouteRisk(waypoints: TemporalWaypoint[]): 'low' | 'medium' | 'high' | 'extreme' {
    const riskScores = {
      safe: 0,
      caution: 1,
      danger: 2,
      prohibited: 3,
    };

    const maxRisk = Math.max(
      ...waypoints.map((wp) => riskScores[wp.safetyLevel])
    );

    if (maxRisk >= 3) return 'extreme';
    if (maxRisk >= 2) return 'high';
    if (maxRisk >= 1) return 'medium';
    return 'low';
  }

  /**
   * Calculate safety score
   */
  private calculateSafetyScore(waypoints: TemporalWaypoint[]): number {
    const safetyValues = {
      safe: 100,
      caution: 70,
      danger: 40,
      prohibited: 0,
    };

    const avgSafety =
      waypoints.reduce((sum, wp) => sum + safetyValues[wp.safetyLevel], 0) /
      waypoints.length;

    return avgSafety;
  }

  /**
   * Calculate distance between waypoints
   */
  private waypointDistance(wp1: TemporalWaypoint, wp2: TemporalWaypoint): number {
    const spatial = this.spatialDistance(
      wp1.coordinates.position,
      wp2.coordinates.position
    );
    const t1 = new Date(wp1.coordinates.time).getTime();
    const t2 = new Date(wp2.coordinates.time).getTime();
    const temporal = Math.abs(t2 - t1) / 1000;

    return spatial + BEACON_CONSTANTS.SPEED_OF_LIGHT * temporal;
  }

  /**
   * Calculate spatial distance
   */
  private spatialDistance(p1: Vector3, p2: Vector3): number {
    const dx = p2.x - p1.x;
    const dy = p2.y - p1.y;
    const dz = p2.z - p1.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }

  /**
   * Determine coverage quality from beacon count
   */
  private determineQuality(count: number): 'excellent' | 'good' | 'adequate' | 'poor' {
    if (count >= 8) return 'excellent';
    if (count >= 6) return 'good';
    if (count >= 4) return 'adequate';
    return 'poor';
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Deploy temporal beacon (standalone function)
 */
export function deployBeacon(deployment: BeaconDeployment): DeploymentResult {
  const sdk = new TemporalBeaconSDK();
  return sdk.deployBeacon(deployment);
}

/**
 * Triangulate position (standalone function)
 */
export function triangulatePosition(
  params: TriangulationParams
): TemporalPosition {
  const sdk = new TemporalBeaconSDK();
  return sdk.triangulatePosition(params);
}

/**
 * Deploy emergency beacon (standalone function)
 */
export function deployEmergencyBeacon(
  emergency: EmergencyBeacon
): EmergencyResponse {
  const sdk = new TemporalBeaconSDK();
  return sdk.deployEmergencyBeacon(emergency);
}

/**
 * Plan route (standalone function)
 */
export function planRoute(params: RoutePlanningParams): NavigationRoute {
  const sdk = new TemporalBeaconSDK();
  return sdk.planRoute(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { TemporalBeaconSDK };
export default TemporalBeaconSDK;
