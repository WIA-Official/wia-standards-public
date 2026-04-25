/**
 * WIA-CONSCIOUSNESS SDK
 * Scientific Consciousness Measurement Standard
 *
 * 弘益人間 (弘益人間) - Benefit All Humanity
 * © 2025 WIA Standards · MIT License
 */

import {
  ConsciousnessIndex,
  ConsciousnessState,
  ConsciousnessCondition,
  MeasurementMethod,
  MeasurementRequest,
  MeasurementResponse,
  IITMetrics,
  GNWMetrics,
  PracticalMeasures,
  PCIMeasure,
  PCIClassification,
  AIConsciousnessAssessment,
  AISystemType,
  MonitoringUpdate,
  TheoryComparison,
  PCI_THRESHOLD,
  PCI_THRESHOLDS,
  SCHEMA_VERSION,
  SCHEMA_URL,
} from './types';

// Re-export all types
export * from './types';

// ============================================================================
// Client Configuration
// ============================================================================

/**
 * Client configuration options
 */
export interface ConsciousnessClientConfig {
  /** API key for authentication */
  apiKey: string;
  /** Base URL for API (default: https://api.wia.live/consciousness/v1) */
  baseUrl?: string;
  /** Request timeout in milliseconds (default: 30000) */
  timeout?: number;
}

// ============================================================================
// Consciousness Client
// ============================================================================

/**
 * WIA-CONSCIOUSNESS API Client
 *
 * @example
 * ```typescript
 * import { ConsciousnessClient } from '@wia/consciousness';
 *
 * const client = new ConsciousnessClient({
 *   apiKey: 'your-api-key'
 * });
 *
 * // Measure consciousness
 * const measurement = await client.measure({
 *   subject_id: 'patient-001',
 *   method: 'TMS-EEG'
 * });
 *
 * // Get PCI value
 * const pci = await client.getPCI(measurement.id);
 * console.log(`PCI: ${pci.value}, State: ${pci.classification}`);
 * ```
 */
export class ConsciousnessClient {
  private apiKey: string;
  private baseUrl: string;
  private timeout: number;

  constructor(config: ConsciousnessClientConfig) {
    this.apiKey = config.apiKey;
    this.baseUrl = config.baseUrl || 'https://api.wia.live/consciousness/v1';
    this.timeout = config.timeout || 30000;
  }

  /**
   * Initiate a consciousness measurement
   */
  async measure(request: MeasurementRequest): Promise<MeasurementResponse> {
    return this.post<MeasurementResponse>('/measure', request);
  }

  /**
   * Get measurement results by ID
   */
  async getMeasurement(measurementId: string): Promise<ConsciousnessIndex> {
    return this.get<ConsciousnessIndex>(`/measure/${measurementId}`);
  }

  /**
   * Get consciousness state for a subject
   */
  async getState(subjectId: string): Promise<ConsciousnessState> {
    const response = await this.get<{ state: ConsciousnessState }>(
      `/state/${subjectId}`
    );
    return response.state;
  }

  /**
   * Get PCI measurement for a subject
   */
  async getPCI(subjectId: string): Promise<PCIMeasure> {
    const response = await this.get<{ pci: PCIMeasure }>(`/pci/${subjectId}`);
    return response.pci;
  }

  /**
   * Get Φ (phi) estimate for a subject
   */
  async getPhi(subjectId: string): Promise<IITMetrics> {
    return this.get<IITMetrics>(`/phi/${subjectId}`);
  }

  /**
   * Compare theory predictions for a measurement
   */
  async compareTheories(
    measurementId: string,
    theories: Array<'IIT' | 'GNW' | 'HOT' | 'PP' | 'AST'> = ['IIT', 'GNW']
  ): Promise<TheoryComparison[]> {
    const response = await this.post<{ comparisons: TheoryComparison[] }>(
      '/compare',
      { measurement_id: measurementId, theories }
    );
    return response.comparisons;
  }

  /**
   * Assess AI system consciousness indicators
   */
  async assessAI(
    systemId: string,
    systemType: AISystemType,
    modelInfo?: { name: string; version?: string; parameters?: number }
  ): Promise<AIConsciousnessAssessment> {
    return this.post<AIConsciousnessAssessment>('/ai/assess', {
      system_id: systemId,
      system_type: systemType,
      model_info: modelInfo,
    });
  }

  /**
   * Subscribe to real-time monitoring updates
   */
  monitor(
    subjectId: string,
    callback: (update: MonitoringUpdate) => void
  ): () => void {
    const wsUrl = this.baseUrl
      .replace('https://', 'wss://')
      .replace('http://', 'ws://');

    const ws = new WebSocket(`${wsUrl}/monitor/${subjectId}`);

    ws.onopen = () => {
      ws.send(JSON.stringify({
        action: 'subscribe',
        metrics: ['pci', 'phi', 'state'],
      }));
    };

    ws.onmessage = (event) => {
      const update = JSON.parse(event.data) as MonitoringUpdate;
      callback(update);
    };

    // Return unsubscribe function
    return () => {
      ws.send(JSON.stringify({ action: 'unsubscribe' }));
      ws.close();
    };
  }

  // Private HTTP methods
  private async get<T>(path: string): Promise<T> {
    const response = await fetch(`${this.baseUrl}${path}`, {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      throw new Error(`API error: ${response.status} ${response.statusText}`);
    }

    return response.json() as Promise<T>;
  }

  private async post<T>(path: string, body: unknown): Promise<T> {
    const response = await fetch(`${this.baseUrl}${path}`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(body),
    });

    if (!response.ok) {
      throw new Error(`API error: ${response.status} ${response.statusText}`);
    }

    return response.json() as Promise<T>;
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Classify PCI value into clinical category
 */
export function classifyPCI(pciValue: number): PCIClassification {
  if (pciValue < PCI_THRESHOLDS.VS_UWS.max) return 'VS_UWS';
  if (pciValue < PCI_THRESHOLDS.MCS_minus.max) return 'MCS_minus';
  if (pciValue < PCI_THRESHOLDS.MCS_plus.max) return 'MCS_plus';
  if (pciValue < PCI_THRESHOLDS.EMCS.max) return 'EMCS';
  if (pciValue < PCI_THRESHOLDS.LIS.max) return 'LIS';
  return 'conscious';
}

/**
 * Check if PCI indicates consciousness (above threshold)
 */
export function isConscious(pciValue: number): boolean {
  return pciValue >= PCI_THRESHOLD;
}

/**
 * Create a new ConsciousnessIndex record
 */
export function createConsciousnessIndex(
  subjectId: string,
  method: MeasurementMethod,
  options?: Partial<Omit<ConsciousnessIndex, 'subject_id' | 'timestamp' | 'measurement'>>
): ConsciousnessIndex {
  return {
    subject_id: subjectId,
    timestamp: new Date().toISOString(),
    measurement: { method },
    ...options,
  };
}

/**
 * Validate a ConsciousnessIndex record
 */
export function validateConsciousnessIndex(data: unknown): data is ConsciousnessIndex {
  if (typeof data !== 'object' || data === null) return false;

  const record = data as Record<string, unknown>;

  // Check required fields
  if (typeof record.subject_id !== 'string') return false;
  if (typeof record.timestamp !== 'string') return false;
  if (typeof record.measurement !== 'object') return false;

  // Validate measurement
  const measurement = record.measurement as Record<string, unknown>;
  if (typeof measurement.method !== 'string') return false;

  // Validate optional numeric ranges
  if (record.iit_metrics) {
    const iit = record.iit_metrics as Record<string, unknown>;
    if (iit.cause_effect_power !== undefined) {
      if (typeof iit.cause_effect_power !== 'number') return false;
      if (iit.cause_effect_power < 0 || iit.cause_effect_power > 1) return false;
    }
  }

  if (record.practical_measures) {
    const practical = record.practical_measures as Record<string, unknown>;
    if (practical.pci) {
      const pci = practical.pci as Record<string, unknown>;
      if (typeof pci.value !== 'number') return false;
      if (pci.value < 0 || pci.value > 1) return false;
    }
  }

  return true;
}

/**
 * Calculate expected PCI from consciousness state
 */
export function expectedPCIRange(condition: ConsciousnessCondition): { min: number; max: number } {
  const ranges: Record<ConsciousnessCondition, { min: number; max: number }> = {
    awake: { min: 0.45, max: 0.65 },
    drowsy: { min: 0.40, max: 0.55 },
    sleep_N1: { min: 0.35, max: 0.45 },
    sleep_N2: { min: 0.25, max: 0.35 },
    sleep_N3: { min: 0.15, max: 0.25 },
    REM: { min: 0.40, max: 0.55 },
    anesthesia_light: { min: 0.25, max: 0.35 },
    anesthesia_moderate: { min: 0.15, max: 0.25 },
    anesthesia_deep: { min: 0.10, max: 0.20 },
    coma: { min: 0.05, max: 0.20 },
    VS_UWS: { min: 0.10, max: 0.25 },
    MCS_minus: { min: 0.31, max: 0.40 },
    MCS_plus: { min: 0.37, max: 0.50 },
    EMCS: { min: 0.45, max: 0.55 },
    LIS: { min: 0.50, max: 0.65 },
    meditation: { min: 0.45, max: 0.70 },
    psychedelic: { min: 0.50, max: 0.80 },
    hypnosis: { min: 0.35, max: 0.55 },
    seizure: { min: 0.10, max: 0.40 },
  };

  return ranges[condition] || { min: 0, max: 1 };
}

// ============================================================================
// Default Export
// ============================================================================

export default ConsciousnessClient;
