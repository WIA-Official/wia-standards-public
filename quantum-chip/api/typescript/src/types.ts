/**
 * Platform types for quantum processors
 */
export enum QuantumPlatform {
  SUPERCONDUCTING = 'superconducting',
  ION_TRAP = 'ion_trap',
  PHOTONIC = 'photonic'
}

export interface QubitMetrics {
  t1: number;
  t2: number;
  frequency: number;
}

export interface GateFidelity {
  gate: string;
  fidelity: number;
  duration: number;
  qubitIds: number[];
}

export interface QuantumProcessor {
  id: string;
  name: string;
  platform: QuantumPlatform;
  vendor: string;
  qubitCount: number;
  qubitMetrics: QubitMetrics[];
  gateFidelities: GateFidelity[];
  quantumVolume?: number;
  calibrationDate: Date;
}
