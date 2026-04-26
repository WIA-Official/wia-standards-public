/**
 * WIA-SEMI-002 Memory Semiconductor TypeScript SDK
 * Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 */

// Memory Types
export enum MemoryType {
  SRAM = 'SRAM',
  DRAM = 'DRAM',
  DDR4 = 'DDR4',
  DDR5 = 'DDR5',
  LPDDR4 = 'LPDDR4',
  LPDDR5 = 'LPDDR5',
  HBM2 = 'HBM2',
  HBM3 = 'HBM3',
  NAND_FLASH = 'NAND_FLASH',
  MRAM = 'MRAM',
  PCM = 'PCM',
  RERAM = 'RERAM',
}

export interface MemorySpec {
  type: MemoryType;
  capacity: number;
  dataRate: number;
  busWidth: number;
  voltage: number;
}

export interface DRAMTiming {
  tCL: number;
  tRCD: number;
  tRP: number;
  tRAS: number;
}

export interface BandwidthCalculation {
  dataRate: number;
  busWidth: number;
  channels: number;
  theoreticalBandwidth: number;
  effectiveBandwidth?: number;
}
