/**
 * WIA-SEMI-007 Neuromorphic Chip TypeScript SDK
 * Core Type Definitions
 */

export interface NeuronParameters {
  /** Membrane time constant (ms) */
  tau_m?: number;
  /** Resting potential (mV) */
  v_rest?: number;
  /** Spike threshold (mV) */
  v_threshold?: number;
  /** Reset potential (mV) */
  v_reset?: number;
  /** Membrane resistance (MΩ) */
  resistance?: number;
  /** Refractory period (ms) */
  tau_ref?: number;
}

export interface SynapseParameters {
  /** Synaptic weight */
  weight: number;
  /** Synaptic time constant (ms) */
  tau_syn?: number;
  /** Enable STDP learning */
  learning_enabled?: boolean;
  /** STDP potentiation amplitude */
  a_plus?: number;
  /** STDP depression amplitude */
  a_minus?: number;
  /** STDP potentiation time constant (ms) */
  tau_plus?: number;
  /** STDP depression time constant (ms) */
  tau_minus?: number;
}

export interface SpikeEvent {
  /** Neuron ID that spiked */
  neuron_id: number;
  /** Time of spike (ms) */
  timestamp: number;
  /** Spike polarity (0=inhibitory, 1=excitatory) */
  polarity?: number;
}

export interface NetworkConfig {
  /** Layer sizes [input, hidden..., output] */
  layer_sizes: number[];
  /** Time step (ms) */
  dt?: number;
  /** Default neuron parameters */
  neuron_params?: NeuronParameters;
  /** Default synapse parameters */
  synapse_params?: Partial<SynapseParameters>;
}

export interface EncodingConfig {
  /** Encoding method */
  method: 'rate' | 'latency' | 'population' | 'temporal';
  /** Maximum firing rate (Hz) for rate coding */
  max_rate?: number;
  /** Time window (ms) */
  duration?: number;
  /** Population size for population coding */
  population_size?: number;
}

export interface SimulationResult {
  /** Output spike trains */
  spike_trains: SpikeEvent[][];
  /** Membrane voltages over time */
  voltages?: number[][];
  /** Simulation time (ms) */
  simulation_time: number;
  /** Number of spikes generated */
  total_spikes: number;
}

export interface BenchmarkResult {
  /** Benchmark name */
  name: string;
  /** Accuracy (0-1) */
  accuracy: number;
  /** Average latency (ms) */
  latency_ms: number;
  /** Energy per inference (μJ) */
  energy_uj: number;
  /** Throughput (samples/sec) */
  throughput: number;
}

export type NeuronModel = 'LIF' | 'Izhikevich' | 'HodgkinHuxley';
export type LearningRule = 'STDP' | 'SurrogateGradient' | 'None';
