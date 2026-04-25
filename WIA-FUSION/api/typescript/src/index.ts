/**
 * WIA-FUSION TypeScript SDK
 * Nuclear Fusion Energy Standard
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 * © 2025 WIA - World Certification Industry Association
 */

import {
  FusionClientConfig,
  PlasmaState,
  RecordStateRequest,
  RecordStateResponse,
  DisruptionPredictionRequest,
  DisruptionPredictionResponse,
  ControlOptimizeRequest,
  ControlOptimizeResponse,
  EnergyBalanceResponse,
  StreamSubscription,
  StreamMessage,
  FusionAPIError,
  ReactorType,
} from './types';

export * from './types';

const DEFAULT_BASE_URL = 'https://api.fusion.wia.live/v1';
const DEFAULT_TIMEOUT = 30000;

/**
 * WIA-FUSION API Client
 *
 * @example
 * ```typescript
 * const client = new FusionClient({
 *   apiKey: process.env.WIA_FUSION_API_KEY!,
 *   reactor: 'KSTAR'
 * });
 *
 * // Record plasma state
 * await client.plasma.recordState({
 *   temperature_keV: { ion: 10, electron: 10 },
 *   density_m3: { value: 1.0, unit: '1e20/m3' }
 * });
 *
 * // Get disruption prediction
 * const prediction = await client.fusion.predictDisruption({
 *   plasma_state: { temperature_keV: 10, density_m3: 1.0, plasma_current_ma: 15, beta_percent: 2.5 }
 * });
 * ```
 */
export class FusionClient {
  private config: Required<FusionClientConfig>;
  public plasma: PlasmaAPI;
  public fusion: FusionAPI;
  public stream: StreamAPI;

  constructor(config: FusionClientConfig) {
    this.config = {
      apiKey: config.apiKey,
      baseUrl: config.baseUrl || DEFAULT_BASE_URL,
      reactor: config.reactor || 'ITER',
      timeout: config.timeout || DEFAULT_TIMEOUT,
    };

    this.plasma = new PlasmaAPI(this.config);
    this.fusion = new FusionAPI(this.config);
    this.stream = new StreamAPI(this.config);
  }

  /**
   * Verify API connectivity
   */
  async ping(): Promise<boolean> {
    try {
      const response = await this.request('GET', '/health');
      return response.status === 'ok';
    } catch {
      return false;
    }
  }

  private async request<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.baseUrl}${path}`;
    const headers: Record<string, string> = {
      'Authorization': `Bearer ${this.config.apiKey}`,
      'Content-Type': 'application/json',
    };

    const options: RequestInit = {
      method,
      headers,
      body: body ? JSON.stringify(body) : undefined,
    };

    const response = await fetch(url, options);

    if (!response.ok) {
      const error = await response.json();
      throw new FusionAPIError(error.error || error);
    }

    return response.json();
  }
}

/**
 * Plasma State API
 */
class PlasmaAPI {
  constructor(private config: Required<FusionClientConfig>) {}

  /**
   * Record a new plasma state
   */
  async recordState(state: Partial<PlasmaState>): Promise<RecordStateResponse> {
    const fullState: PlasmaState = {
      shot_id: state.shot_id || `${this.config.reactor}-${Date.now()}`,
      timestamp: state.timestamp || new Date().toISOString(),
      reactor: state.reactor || this.config.reactor,
      core_parameters: state.core_parameters!,
      performance: state.performance,
      stability: state.stability,
      control: state.control,
    };

    return this.request<RecordStateResponse>('POST', '/plasma/state', {
      plasma_state: fullState,
    });
  }

  /**
   * Get plasma state for a specific shot
   */
  async getState(shotId: string): Promise<{ plasma_state: PlasmaState }> {
    return this.request<{ plasma_state: PlasmaState }>('GET', `/plasma/state/${shotId}`);
  }

  /**
   * Get the latest plasma state
   */
  async getLatest(reactor?: ReactorType): Promise<{ plasma_state: PlasmaState; age_ms: number }> {
    const query = reactor ? `?reactor=${reactor}` : '';
    return this.request<{ plasma_state: PlasmaState; age_ms: number }>('GET', `/plasma/state/latest${query}`);
  }

  /**
   * Analyze plasma stability
   */
  async analyzeStability(shotId: string, timeWindow?: number): Promise<{
    stability_analysis: {
      disruption_risk: number;
      confidence: number;
      mhd_modes: string[];
      recommendations: string[];
    };
  }> {
    const query = timeWindow ? `?time_window_s=${timeWindow}` : '';
    return this.request('GET', `/plasma/stability/${shotId}${query}`);
  }

  private async request<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.baseUrl}${path}`;
    const headers: Record<string, string> = {
      'Authorization': `Bearer ${this.config.apiKey}`,
      'Content-Type': 'application/json',
    };

    const response = await fetch(url, { method, headers, body: body ? JSON.stringify(body) : undefined });
    if (!response.ok) {
      const error = await response.json();
      throw new FusionAPIError(error.error || error);
    }
    return response.json();
  }
}

/**
 * Fusion Analysis API
 */
class FusionAPI {
  constructor(private config: Required<FusionClientConfig>) {}

  /**
   * Predict disruption probability
   */
  async predictDisruption(request: DisruptionPredictionRequest): Promise<DisruptionPredictionResponse> {
    return this.request<DisruptionPredictionResponse>('POST', '/fusion/predict/disruption', request);
  }

  /**
   * Optimize control parameters
   */
  async optimizeControl(request: ControlOptimizeRequest): Promise<ControlOptimizeResponse> {
    return this.request<ControlOptimizeResponse>('POST', '/plasma/control/optimize', request);
  }

  /**
   * Get energy balance analysis
   */
  async getEnergyBalance(shotId: string): Promise<EnergyBalanceResponse> {
    return this.request<EnergyBalanceResponse>('GET', `/fusion/energy-balance/${shotId}`);
  }

  /**
   * Calculate Triple Product
   */
  calculateTripleProduct(temperature_keV: number, density_1e20_m3: number, confinement_s: number): number {
    return temperature_keV * density_1e20_m3 * confinement_s;
  }

  /**
   * Calculate Q Factor (simplified model)
   */
  calculateQFactor(fusionPower_MW: number, inputPower_MW: number): number {
    if (inputPower_MW <= 0) return 0;
    return fusionPower_MW / inputPower_MW;
  }

  private async request<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.baseUrl}${path}`;
    const headers: Record<string, string> = {
      'Authorization': `Bearer ${this.config.apiKey}`,
      'Content-Type': 'application/json',
    };

    const response = await fetch(url, { method, headers, body: body ? JSON.stringify(body) : undefined });
    if (!response.ok) {
      const error = await response.json();
      throw new FusionAPIError(error.error || error);
    }
    return response.json();
  }
}

/**
 * Real-time Streaming API
 */
class StreamAPI {
  private ws: WebSocket | null = null;
  private listeners: Map<string, ((data: StreamMessage) => void)[]> = new Map();

  constructor(private config: Required<FusionClientConfig>) {}

  /**
   * Connect to real-time plasma stream
   */
  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      const wsUrl = this.config.baseUrl.replace('https://', 'wss://').replace('http://', 'ws://');
      this.ws = new WebSocket(`${wsUrl}/stream/plasma?token=${this.config.apiKey}`);

      this.ws.onopen = () => resolve();
      this.ws.onerror = (error) => reject(error);
      this.ws.onmessage = (event) => {
        const message: StreamMessage = JSON.parse(event.data);
        const channelListeners = this.listeners.get(message.channel) || [];
        channelListeners.forEach(listener => listener(message));
      };
    });
  }

  /**
   * Subscribe to channels
   */
  subscribe(subscription: StreamSubscription): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket not connected. Call connect() first.');
    }
    this.ws.send(JSON.stringify(subscription));
  }

  /**
   * Add listener for a channel
   */
  on(channel: string, callback: (data: StreamMessage) => void): void {
    if (!this.listeners.has(channel)) {
      this.listeners.set(channel, []);
    }
    this.listeners.get(channel)!.push(callback);
  }

  /**
   * Disconnect from stream
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
    this.listeners.clear();
  }
}

// Default export
export default FusionClient;
