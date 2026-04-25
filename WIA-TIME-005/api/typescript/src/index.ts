/**
 * WIA-TIME-005: Timeline Anchor Standard - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @description Complete SDK for Timeline Anchor operations
 *
 * 弘益人間 (Benefit All Humanity)
 * © 2025 SmileStory Inc. / WIA
 */

import {
  AnchorCoordinates,
  TimelineAnchor,
  AnchorPoint,
  AnchorHealth,
  TemporalDrift,
  DriftCorrection,
  AnchorChain,
  EmergencyAnchorConfig,
  BeaconSignal,
  StabilityReport,
  CreateAnchorRequest,
  CreateAnchorResponse,
  UpdateAnchorRequest,
  CorrectDriftRequest,
  CreateChainRequest,
  DeployEmergencyAnchorRequest,
  SDKConfig,
  SDKEnvironment,
  APIResponse,
  PaginatedResponse,
  PaginationParams,
  EventCallback,
  WebSocketEventType,
  EventSubscription,
  ChainNavigation,
  ChainOptions,
} from './types';

// ============================================================================
// Constants
// ============================================================================

const API_URLS: Record<SDKEnvironment, string> = {
  production: 'https://api.wiastandards.com/time-005/v1',
  staging: 'https://api-staging.wiastandards.com/time-005/v1',
  development: 'http://localhost:3000/time-005/v1',
};

const DEFAULT_CONFIG = {
  timeout: 30000,
  debug: false,
  retry: {
    maxAttempts: 3,
    initialDelay: 1000,
    maxDelay: 10000,
  },
};

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * Timeline Anchor SDK
 *
 * Provides complete API access to WIA-TIME-005 Timeline Anchor services
 *
 * @example
 * ```typescript
 * const sdk = new TimelineAnchorSDK({
 *   apiKey: 'your-api-key',
 *   environment: 'production'
 * });
 *
 * // Create an anchor
 * const anchor = await sdk.createAnchor({
 *   name: 'Origin Point',
 *   coordinates: { t: Date.now(), x: 0, y: 0, z: 0, d: 0, q: 0 },
 *   strength: 2.42,
 *   type: 'primary',
 *   beacon: { frequency: 432, range: 1000, signalType: 'quantum-entangled' }
 * });
 * ```
 */
export class TimelineAnchorSDK {
  private config: Required<SDKConfig>;
  private baseUrl: string;
  private ws: WebSocket | null = null;
  private subscriptions: Map<string, EventSubscription> = new Map();

  constructor(config: SDKConfig) {
    this.config = {
      ...DEFAULT_CONFIG,
      ...config,
      baseUrl: config.baseUrl || API_URLS[config.environment],
    };

    this.baseUrl = this.config.baseUrl;

    if (this.config.debug) {
      console.log('[WIA-TIME-005 SDK] Initialized', {
        environment: config.environment,
        baseUrl: this.baseUrl,
      });
    }
  }

  // ==========================================================================
  // Anchor Management
  // ==========================================================================

  /**
   * Create a new timeline anchor
   *
   * @param request - Anchor creation parameters
   * @returns Created anchor with deployment information
   *
   * @example
   * ```typescript
   * const anchor = await sdk.createAnchor({
   *   name: 'Alpha Origin',
   *   coordinates: { t: Date.now(), x: 0, y: 0, z: 0, d: 0, q: 0 },
   *   strength: 2.42,
   *   type: 'primary',
   *   beacon: { frequency: 432, range: 1000, signalType: 'quantum-entangled' }
   * });
   * ```
   */
  async createAnchor(
    request: CreateAnchorRequest
  ): Promise<CreateAnchorResponse> {
    this.validateAnchorRequest(request);

    const response = await this.request<CreateAnchorResponse>('POST', '/anchors', request);

    if (this.config.debug) {
      console.log('[WIA-TIME-005 SDK] Anchor created', response);
    }

    return response;
  }

  /**
   * Establish an anchor point at specific coordinates
   *
   * @param coordinates - Spacetime coordinates for anchor
   * @param options - Additional anchor options
   * @returns Established anchor point
   *
   * @example
   * ```typescript
   * const anchor = await sdk.establishAnchorPoint(
   *   { t: Date.now(), x: 0, y: 0, z: 0, d: 0, q: 0 },
   *   { strength: 2.42, type: 'primary' }
   * );
   * ```
   */
  async establishAnchorPoint(
    coordinates: AnchorCoordinates,
    options: {
      name?: string;
      strength?: number;
      type?: 'primary' | 'secondary' | 'waypoint' | 'emergency';
    } = {}
  ): Promise<TimelineAnchor> {
    const request: CreateAnchorRequest = {
      name: options.name || `Anchor_${Date.now()}`,
      coordinates,
      strength: options.strength || 2.42,
      type: options.type || 'primary',
      beacon: {
        frequency: 432,
        range: 1000,
        signalType: 'quantum-entangled',
      },
    };

    const response = await this.createAnchor(request);
    return response.anchor;
  }

  /**
   * Get anchor by ID
   *
   * @param anchorId - Anchor identifier
   * @returns Anchor details
   */
  async getAnchor(anchorId: string): Promise<TimelineAnchor> {
    return this.request<TimelineAnchor>('GET', `/anchors/${anchorId}`);
  }

  /**
   * List all anchors with optional pagination and filtering
   *
   * @param params - Pagination and filter parameters
   * @returns Paginated list of anchors
   */
  async listAnchors(
    params: PaginationParams & {
      type?: string;
      status?: string;
      minHealth?: number;
    } = {}
  ): Promise<PaginatedResponse<TimelineAnchor>> {
    const queryParams = new URLSearchParams(
      Object.entries(params)
        .filter(([_, v]) => v !== undefined)
        .map(([k, v]) => [k, String(v)])
    );

    return this.request<PaginatedResponse<TimelineAnchor>>(
      'GET',
      `/anchors?${queryParams}`
    );
  }

  /**
   * Update anchor configuration
   *
   * @param anchorId - Anchor identifier
   * @param updates - Update parameters
   * @returns Updated anchor
   */
  async updateAnchor(
    anchorId: string,
    updates: UpdateAnchorRequest
  ): Promise<TimelineAnchor> {
    return this.request<TimelineAnchor>(
      'PATCH',
      `/anchors/${anchorId}`,
      updates
    );
  }

  /**
   * Delete/decommission an anchor
   *
   * @param anchorId - Anchor identifier
   * @param reason - Decommissioning reason
   */
  async deleteAnchor(anchorId: string, reason?: string): Promise<void> {
    await this.request('DELETE', `/anchors/${anchorId}`, { reason });
  }

  // ==========================================================================
  // Health Monitoring
  // ==========================================================================

  /**
   * Monitor anchor health in real-time
   *
   * @param anchorId - Anchor identifier
   * @returns Current health status
   *
   * @example
   * ```typescript
   * const health = await sdk.monitorAnchorHealth('anchor_001');
   * console.log(`Anchor stability: ${health.overall}%`);
   * ```
   */
  async monitorAnchorHealth(anchorId: string): Promise<AnchorHealth> {
    const health = await this.request<AnchorHealth>(
      'GET',
      `/anchors/${anchorId}/health`
    );

    if (health.overall < 70) {
      console.warn(
        `[WIA-TIME-005 SDK] Anchor ${anchorId} health critical: ${health.overall}%`
      );
    }

    return health;
  }

  /**
   * Get stability report for anchor
   *
   * @param anchorId - Anchor identifier
   * @returns Stability report
   */
  async getStabilityReport(anchorId: string): Promise<StabilityReport> {
    return this.request<StabilityReport>(
      'GET',
      `/anchors/${anchorId}/stability`
    );
  }

  /**
   * Continuous health monitoring with callback
   *
   * @param anchorId - Anchor identifier
   * @param callback - Function to call with health updates
   * @param intervalMs - Monitoring interval in milliseconds
   * @returns Stop function to cancel monitoring
   */
  monitorHealthContinuous(
    anchorId: string,
    callback: (health: AnchorHealth) => void,
    intervalMs: number = 5000
  ): () => void {
    const interval = setInterval(async () => {
      try {
        const health = await this.monitorAnchorHealth(anchorId);
        callback(health);
      } catch (error) {
        console.error('[WIA-TIME-005 SDK] Health monitoring error:', error);
      }
    }, intervalMs);

    return () => clearInterval(interval);
  }

  // ==========================================================================
  // Drift Detection & Correction
  // ==========================================================================

  /**
   * Calculate current temporal drift
   *
   * @param anchorId - Anchor identifier
   * @returns Drift measurement
   *
   * @example
   * ```typescript
   * const drift = await sdk.calculateDrift('anchor_001');
   * if (drift.magnitude > 0.01) {
   *   await sdk.correctTemporalDrift('anchor_001', drift);
   * }
   * ```
   */
  async calculateDrift(anchorId: string): Promise<TemporalDrift> {
    const drift = await this.request<TemporalDrift>(
      'GET',
      `/anchors/${anchorId}/drift`
    );

    if (!drift.acceptable) {
      console.warn(
        `[WIA-TIME-005 SDK] Unacceptable drift detected: ${drift.magnitude} TU`
      );
    }

    return drift;
  }

  /**
   * Correct temporal drift
   *
   * @param anchorId - Anchor identifier
   * @param drift - Detected drift to correct
   * @param strategy - Correction strategy (default: 'gradual')
   * @returns Correction plan and result
   *
   * @example
   * ```typescript
   * const drift = await sdk.calculateDrift('anchor_001');
   * const correction = await sdk.correctTemporalDrift('anchor_001', drift, 'immediate');
   * ```
   */
  async correctTemporalDrift(
    anchorId: string,
    drift: TemporalDrift,
    strategy: 'gradual' | 'immediate' | 'emergency' = 'gradual'
  ): Promise<DriftCorrection> {
    const request: CorrectDriftRequest = { strategy };

    const correction = await this.request<DriftCorrection>(
      'POST',
      `/anchors/${anchorId}/correct-drift`,
      request
    );

    if (this.config.debug) {
      console.log('[WIA-TIME-005 SDK] Drift correction initiated', correction);
    }

    return correction;
  }

  /**
   * Auto-correct drift if exceeds threshold
   *
   * @param anchorId - Anchor identifier
   * @param threshold - Drift threshold (default: 0.01 TU)
   * @returns Correction result (null if no correction needed)
   */
  async autoCorrectDrift(
    anchorId: string,
    threshold: number = 0.01
  ): Promise<DriftCorrection | null> {
    const drift = await this.calculateDrift(anchorId);

    if (drift.magnitude > threshold) {
      const strategy = drift.magnitude > 0.1 ? 'emergency' : 'gradual';
      return this.correctTemporalDrift(anchorId, drift, strategy);
    }

    return null;
  }

  // ==========================================================================
  // Anchor Chains
  // ==========================================================================

  /**
   * Chain multiple anchors together
   *
   * @param anchorIds - Array of anchor IDs to chain
   * @param options - Chain configuration options
   * @returns Created anchor chain
   *
   * @example
   * ```typescript
   * const chain = await sdk.chainAnchors(
   *   ['anchor_001', 'anchor_002', 'anchor_003'],
   *   { topology: 'linear', redundancyLevel: 2 }
   * );
   * ```
   */
  async chainAnchors(
    anchorIds: string[],
    options: {
      name?: string;
      topology?: 'linear' | 'branching' | 'mesh';
      redundancyLevel?: number;
    } = {}
  ): Promise<AnchorChain> {
    if (anchorIds.length < 2) {
      throw new Error('Chain requires at least 2 anchors');
    }

    return this.request<AnchorChain>('POST', '/chains', {
      name: options.name || `Chain_${Date.now()}`,
      anchorIds,
      topology: options.topology || 'linear',
      redundancyLevel: options.redundancyLevel || 1,
    });
  }

  /**
   * Create anchor chain between two points
   *
   * @param origin - Origin coordinates
   * @param destination - Destination coordinates
   * @param options - Chain options
   * @returns Created anchor chain
   */
  async createAnchorChain(
    origin: AnchorCoordinates,
    destination: AnchorCoordinates,
    options: ChainOptions
  ): Promise<AnchorChain> {
    const request: CreateChainRequest = {
      name: `Chain_${origin.t}_to_${destination.t}`,
      origin,
      destination,
      options,
    };

    return this.request<AnchorChain>('POST', '/chains/create', request);
  }

  /**
   * Get anchor chain by ID
   *
   * @param chainId - Chain identifier
   * @returns Chain details
   */
  async getChain(chainId: string): Promise<AnchorChain> {
    return this.request<AnchorChain>('GET', `/chains/${chainId}`);
  }

  /**
   * Navigate along anchor chain
   *
   * @param chainId - Chain identifier
   * @returns Current navigation state
   */
  async navigateChain(chainId: string): Promise<ChainNavigation> {
    return this.request<ChainNavigation>('GET', `/chains/${chainId}/navigate`);
  }

  /**
   * List all chains with optional filtering
   *
   * @param params - Pagination and filter parameters
   * @returns Paginated list of chains
   */
  async listChains(
    params: PaginationParams = {}
  ): Promise<PaginatedResponse<AnchorChain>> {
    const queryParams = new URLSearchParams(
      Object.entries(params)
        .filter(([_, v]) => v !== undefined)
        .map(([k, v]) => [k, String(v)])
    );

    return this.request<PaginatedResponse<AnchorChain>>(
      'GET',
      `/chains?${queryParams}`
    );
  }

  // ==========================================================================
  // Emergency Operations
  // ==========================================================================

  /**
   * Activate emergency anchor
   *
   * @param config - Emergency anchor configuration
   * @returns Deployed emergency anchor
   *
   * @example
   * ```typescript
   * const emergencyAnchor = await sdk.activateEmergencyAnchor({
   *   priority: 'critical',
   *   deployment: {
   *     method: 'instant',
   *     location: 'auto',
   *     strength: 10.0
   *   },
   *   lifespan: { minimum: 3600, target: 24, powerBudget: 5.0 },
   *   notification: { alerts: ['admin@example.com'], broadcast: true, escalation: true }
   * });
   * ```
   */
  async activateEmergencyAnchor(
    config: EmergencyAnchorConfig
  ): Promise<TimelineAnchor> {
    const request: DeployEmergencyAnchorRequest = {
      config,
      reason: 'Emergency anchor activation requested',
    };

    const anchor = await this.request<TimelineAnchor>(
      'POST',
      '/anchors/emergency',
      request
    );

    console.warn('[WIA-TIME-005 SDK] Emergency anchor deployed:', anchor.id);

    return anchor;
  }

  /**
   * Activate emergency protocol
   *
   * @param anchorId - Anchor identifier (optional, will find nearest if not provided)
   * @param reason - Reason for emergency activation
   * @param severity - Emergency severity (1-10)
   * @returns Emergency protocol ID
   */
  async activateEmergencyProtocol(
    anchorId: string | undefined,
    reason: string,
    severity: number = 10
  ): Promise<string> {
    const response = await this.request<{ protocolId: string }>(
      'POST',
      '/emergency/activate',
      { anchorId, reason, severity }
    );

    console.error(
      '[WIA-TIME-005 SDK] Emergency protocol activated:',
      response.protocolId
    );

    return response.protocolId;
  }

  // ==========================================================================
  // Beacon Operations
  // ==========================================================================

  /**
   * Get beacon signal from anchor
   *
   * @param anchorId - Anchor identifier
   * @returns Latest beacon signal
   */
  async getBeaconSignal(anchorId: string): Promise<BeaconSignal> {
    return this.request<BeaconSignal>('GET', `/anchors/${anchorId}/beacon`);
  }

  /**
   * Triangulate position using multiple beacon signals
   *
   * @param beaconIds - Array of beacon/anchor IDs
   * @returns Calculated position with confidence
   */
  async triangulatePosition(beaconIds: string[]): Promise<{
    position: AnchorCoordinates;
    confidence: number;
    error: number;
  }> {
    return this.request('POST', '/beacons/triangulate', { beaconIds });
  }

  // ==========================================================================
  // WebSocket Real-Time Events
  // ==========================================================================

  /**
   * Connect to WebSocket for real-time updates
   */
  private connectWebSocket(): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      return;
    }

    const wsUrl = this.baseUrl.replace('http', 'ws') + '/ws';

    this.ws = new WebSocket(wsUrl);

    this.ws.onopen = () => {
      if (this.config.debug) {
        console.log('[WIA-TIME-005 SDK] WebSocket connected');
      }

      // Authenticate
      this.ws?.send(
        JSON.stringify({
          type: 'auth',
          apiKey: this.config.apiKey,
        })
      );
    };

    this.ws.onmessage = (event) => {
      const message = JSON.parse(event.data);

      // Find matching subscriptions
      this.subscriptions.forEach((sub) => {
        if (
          sub.eventType === message.type &&
          (!sub.anchorId || sub.anchorId === message.anchorId)
        ) {
          sub.callback(message.data);
        }
      });
    };

    this.ws.onerror = (error) => {
      console.error('[WIA-TIME-005 SDK] WebSocket error:', error);
    };

    this.ws.onclose = () => {
      if (this.config.debug) {
        console.log('[WIA-TIME-005 SDK] WebSocket disconnected');
      }

      // Attempt reconnect after 5 seconds
      setTimeout(() => this.connectWebSocket(), 5000);
    };
  }

  /**
   * Subscribe to real-time events
   *
   * @param eventType - Type of event to subscribe to
   * @param callback - Function to call when event occurs
   * @param anchorId - Optional anchor ID to filter events
   * @returns Subscription ID (use to unsubscribe)
   *
   * @example
   * ```typescript
   * sdk.on('health-update', (health) => {
   *   console.log('Health update:', health);
   * }, 'anchor_001');
   * ```
   */
  on<T = any>(
    eventType: WebSocketEventType,
    callback: EventCallback<T>,
    anchorId?: string
  ): string {
    this.connectWebSocket();

    const subscription: EventSubscription = {
      id: `sub_${Date.now()}_${Math.random()}`,
      eventType,
      anchorId,
      callback: callback as EventCallback<any>,
      subscribedAt: Date.now(),
    };

    this.subscriptions.set(subscription.id, subscription);

    // Send subscription message
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(
        JSON.stringify({
          type: 'subscribe',
          eventType,
          anchorId,
        })
      );
    }

    return subscription.id;
  }

  /**
   * Unsubscribe from events
   *
   * @param subscriptionId - Subscription ID returned from on()
   */
  off(subscriptionId: string): void {
    const subscription = this.subscriptions.get(subscriptionId);

    if (subscription) {
      // Send unsubscribe message
      if (this.ws?.readyState === WebSocket.OPEN) {
        this.ws.send(
          JSON.stringify({
            type: 'unsubscribe',
            eventType: subscription.eventType,
            anchorId: subscription.anchorId,
          })
        );
      }

      this.subscriptions.delete(subscriptionId);
    }
  }

  /**
   * Disconnect WebSocket and clear all subscriptions
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }

    this.subscriptions.clear();
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  /**
   * Validate anchor creation request
   */
  private validateAnchorRequest(request: CreateAnchorRequest): void {
    if (request.strength < 1.21 || request.strength > 10) {
      throw new Error('Anchor strength must be between 1.21 and 10.0 Gigawatts');
    }

    if (!request.coordinates) {
      throw new Error('Coordinates are required');
    }

    if (!request.beacon) {
      throw new Error('Beacon configuration is required');
    }
  }

  /**
   * Make HTTP request to API
   */
  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<T> {
    const url = `${this.baseUrl}${path}`;

    let lastError: Error | null = null;

    for (let attempt = 0; attempt < this.config.retry.maxAttempts; attempt++) {
      try {
        const response = await fetch(url, {
          method,
          headers: {
            'Content-Type': 'application/json',
            'X-WIA-API-Key': this.config.apiKey,
          },
          body: body ? JSON.stringify(body) : undefined,
          signal: AbortSignal.timeout(this.config.timeout),
        });

        if (!response.ok) {
          const error = await response.json();
          throw new Error(error.message || `HTTP ${response.status}`);
        }

        const data = await response.json();
        return data as T;
      } catch (error) {
        lastError = error as Error;

        if (attempt < this.config.retry.maxAttempts - 1) {
          const delay = Math.min(
            this.config.retry.initialDelay * Math.pow(2, attempt),
            this.config.retry.maxDelay
          );

          if (this.config.debug) {
            console.log(
              `[WIA-TIME-005 SDK] Retry attempt ${attempt + 1} after ${delay}ms`
            );
          }

          await new Promise((resolve) => setTimeout(resolve, delay));
        }
      }
    }

    throw lastError || new Error('Request failed');
  }
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Calculate distance between two coordinates in spacetime
 *
 * @param a - First coordinate
 * @param b - Second coordinate
 * @returns Distance in spacetime units
 */
export function calculateSpacetimeDistance(
  a: AnchorCoordinates,
  b: AnchorCoordinates
): number {
  const dt = (a.t - b.t) / 1000 / 31536000; // Years
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  const dz = a.z - b.z;
  const dd = a.d - b.d;
  const dq = a.q - b.q;

  // Minkowski-like metric with quantum component
  return Math.sqrt(
    dt * dt + dx * dx + dy * dy + dz * dz + dd * dd + dq * dq
  );
}

/**
 * Generate anchor coordinates from timestamp and location
 *
 * @param timestamp - Time coordinate (Unix ms)
 * @param location - Spatial location
 * @param dimension - Dimension index (default: 0)
 * @param quantumPhase - Quantum phase (default: 0)
 * @returns Anchor coordinates
 */
export function createCoordinates(
  timestamp: number,
  location: { x: number; y: number; z: number },
  dimension: number = 0,
  quantumPhase: number = 0
): AnchorCoordinates {
  return {
    t: timestamp,
    x: location.x,
    y: location.y,
    z: location.z,
    d: dimension,
    q: quantumPhase,
  };
}

/**
 * Check if anchor health is acceptable
 *
 * @param health - Anchor health data
 * @param threshold - Minimum acceptable health (default: 80%)
 * @returns True if health is acceptable
 */
export function isHealthyAnchor(
  health: AnchorHealth,
  threshold: number = 80
): boolean {
  return health.overall >= threshold && health.components.quantumLock >= 90;
}

/**
 * Format coordinates as human-readable string
 *
 * @param coords - Coordinates to format
 * @returns Formatted string
 */
export function formatCoordinates(coords: AnchorCoordinates): string {
  const date = new Date(coords.t);
  return `T:${date.toISOString()} X:${coords.x.toFixed(2)}m Y:${coords.y.toFixed(2)}m Z:${coords.z.toFixed(2)}m D:${coords.d} Q:${coords.q.toFixed(4)}`;
}

// ============================================================================
// Export SDK and utilities
// ============================================================================

export default TimelineAnchorSDK;

export * from './types';

// 弘益人間 (Benefit All Humanity)
