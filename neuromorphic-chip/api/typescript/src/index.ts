/**
 * WIA-SEMI-007 Neuromorphic Chip TypeScript SDK
 * Main Entry Point
 */

export * from './types';

// Neuron Models
export class LIFNeuron {
  private v: number;
  private t_last_spike: number;

  constructor(
    public tau_m: number = 20,
    public v_rest: number = -70,
    public v_threshold: number = -55,
    public v_reset: number = -70,
    public resistance: number = 10,
    public tau_ref: number = 2
  ) {
    this.v = v_rest;
    this.t_last_spike = -Infinity;
  }

  /**
   * Simulate one time step
   * @param current Input current (nA)
   * @param dt Time step (ms)
   * @param t Current time (ms)
   * @returns true if spike occurred
   */
  step(current: number, dt: number = 0.1, t: number = 0): boolean {
    // Check refractory period
    if (t - this.t_last_spike < this.tau_ref) {
      return false;
    }

    // Update voltage
    const dv = (-(this.v - this.v_rest) + this.resistance * current) / this.tau_m;
    this.v += dv * dt;

    // Check threshold
    if (this.v >= this.v_threshold) {
      this.v = this.v_reset;
      this.t_last_spike = t;
      return true;
    }

    return false;
  }

  /** Get current membrane voltage */
  getVoltage(): number {
    return this.v;
  }

  /** Reset neuron state */
  reset(): void {
    this.v = this.v_rest;
    this.t_last_spike = -Infinity;
  }
}

// Synapse Models
export class STDPSynapse {
  private trace_pre: number = 0;
  private trace_post: number = 0;

  constructor(
    public weight: number = 0.5,
    public a_plus: number = 0.005,
    public a_minus: number = 0.00525,
    public tau_plus: number = 20,
    public tau_minus: number = 20,
    public w_min: number = 0,
    public w_max: number = 1
  ) {}

  /**
   * Handle presynaptic spike
   */
  preSpike(): void {
    this.weight -= this.a_minus * this.trace_post;
    this.weight = Math.max(this.w_min, Math.min(this.w_max, this.weight));
    this.trace_pre += 1.0;
  }

  /**
   * Handle postsynaptic spike
   */
  postSpike(): void {
    this.weight += this.a_plus * this.trace_pre;
    this.weight = Math.max(this.w_min, Math.min(this.w_max, this.weight));
    this.trace_post += 1.0;
  }

  /**
   * Decay traces
   * @param dt Time step (ms)
   */
  step(dt: number = 0.1): void {
    this.trace_pre *= Math.exp(-dt / this.tau_plus);
    this.trace_post *= Math.exp(-dt / this.tau_minus);
  }

  /**
   * Compute synaptic current
   * @param pre_spike Presynaptic spike occurred
   * @returns Current contribution
   */
  computeCurrent(pre_spike: boolean): number {
    return pre_spike ? this.weight : 0;
  }
}

// Encoding Functions
export class SpikeEncoder {
  /**
   * Rate coding encoder
   * @param value Input value [0, 1]
   * @param duration Encoding window (ms)
   * @param max_rate Maximum firing rate (Hz)
   * @returns Array of spike times
   */
  static rateEncode(
    value: number,
    duration: number = 100,
    max_rate: number = 200
  ): number[] {
    const rate = value * max_rate / 1000; // Convert to spikes/ms
    const spike_times: number[] = [];

    let t = 0;
    while (t < duration) {
      if (rate > 0) {
        const isi = -Math.log(Math.random()) / rate;
        t += isi;
        if (t < duration) {
          spike_times.push(t);
        }
      } else {
        break;
      }
    }

    return spike_times;
  }

  /**
   * Latency coding encoder
   * @param value Input value [0, 1]
   * @param time_window Time window (ms)
   * @returns Spike time or null if below threshold
   */
  static latencyEncode(value: number, time_window: number = 20): number | null {
    if (value < 0.1) return null;
    return (1 - value) * time_window;
  }

  /**
   * Population coding encoder
   * @param value Input value [0, 1]
   * @param n_neurons Population size
   * @param sigma Tuning curve width
   * @param max_rate Maximum firing rate (Hz)
   * @returns Array of firing rates for each neuron
   */
  static populationEncode(
    value: number,
    n_neurons: number = 20,
    sigma: number = 0.1,
    max_rate: number = 100
  ): number[] {
    const preferred_values = Array.from(
      { length: n_neurons },
      (_, i) => i / (n_neurons - 1)
    );

    return preferred_values.map(pv => {
      const response = Math.exp(-Math.pow(value - pv, 2) / (2 * sigma * sigma));
      return max_rate * response;
    });
  }
}

// Spiking Neural Network
export class SpikingNeuralNetwork {
  private neurons: LIFNeuron[][];
  private weights: number[][][];
  private dt: number;

  constructor(layer_sizes: number[], dt: number = 0.1) {
    this.dt = dt;
    this.neurons = [];
    this.weights = [];

    // Create neurons for each layer
    for (const size of layer_sizes) {
      const layer = Array.from({ length: size }, () => new LIFNeuron());
      this.neurons.push(layer);
    }

    // Initialize weights randomly
    for (let i = 0; i < layer_sizes.length - 1; i++) {
      const w = Array.from({ length: layer_sizes[i] }, () =>
        Array.from({ length: layer_sizes[i + 1] }, () => Math.random() * 0.1)
      );
      this.weights.push(w);
    }
  }

  /**
   * Forward pass through network
   * @param input_spikes Boolean array of input spikes
   * @param t Current time (ms)
   * @returns Output layer spikes
   */
  forward(input_spikes: boolean[], t: number): boolean[] {
    const layer_spikes: boolean[][] = [input_spikes];

    for (let layer_idx = 0; layer_idx < this.neurons.length - 1; layer_idx++) {
      // Calculate currents for next layer
      const next_layer_size = this.neurons[layer_idx + 1].length;
      const input_currents = new Array(next_layer_size).fill(0);

      for (let i = 0; i < layer_spikes[layer_idx].length; i++) {
        if (layer_spikes[layer_idx][i]) {
          for (let j = 0; j < next_layer_size; j++) {
            input_currents[j] += this.weights[layer_idx][i][j];
          }
        }
      }

      // Update neurons and collect spikes
      const next_spikes = this.neurons[layer_idx + 1].map((neuron, i) =>
        neuron.step(input_currents[i], this.dt, t)
      );

      layer_spikes.push(next_spikes);
    }

    return layer_spikes[layer_spikes.length - 1];
  }

  /**
   * Train network using STDP
   * @param input_pattern Function that generates input spikes
   * @param duration Training duration (ms)
   * @param learning_rate Learning rate
   */
  trainSTDP(
    input_pattern: (t: number) => boolean[],
    duration: number = 100,
    learning_rate: number = 0.001
  ): void {
    const steps = Math.floor(duration / this.dt);

    for (let step = 0; step < steps; step++) {
      const t = step * this.dt;
      const input_spikes = input_pattern(t);
      this.forward(input_spikes, t);
      // STDP weight updates would go here
    }
  }

  /**
   * Reset all neurons
   */
  reset(): void {
    for (const layer of this.neurons) {
      for (const neuron of layer) {
        neuron.reset();
      }
    }
  }

  /**
   * Get network architecture
   */
  getArchitecture(): number[] {
    return this.neurons.map(layer => layer.length);
  }
}

// Utility Functions
export const Utils = {
  /**
   * Calculate firing rate from spike train
   * @param spike_times Array of spike times
   * @param duration Total duration (ms)
   * @returns Firing rate (Hz)
   */
  calculateFiringRate(spike_times: number[], duration: number): number {
    return (spike_times.length / duration) * 1000;
  },

  /**
   * Estimate energy consumption
   * @param n_spikes Total spikes
   * @param energy_per_spike Energy per spike (J)
   * @returns Total energy (J)
   */
  estimateEnergy(n_spikes: number, energy_per_spike: number = 5e-12): number {
    return n_spikes * energy_per_spike;
  },

  /**
   * Convert spike count to class prediction
   * @param spike_counts Array of spike counts per output neuron
   * @returns Predicted class index
   */
  decodeSpikeCounts(spike_counts: number[]): number {
    return spike_counts.indexOf(Math.max(...spike_counts));
  }
};

// Export version
export const VERSION = '1.0.0';
export const STANDARD = 'WIA-SEMI-007';
