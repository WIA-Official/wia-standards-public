/**
 * WIA-COMP-007: Virtualization - TypeScript Type Definitions
 * @version 1.0.0
 * @license MIT
 * 弘익人間 (Benefit All Humanity)
 */

export type VMState = 'creating' | 'running' | 'paused' | 'stopped' | 'deleted';
export type HypervisorType = 'kvm' | 'xen' | 'vmware' | 'hyperv' | 'virtualbox';

export interface VMConfig {
  name: string;
  os: string;
  cpu: number;
  memory: string;
  disk: string;
  network?: string;
  hypervisor?: HypervisorType;
}

export interface VMInfo {
  id: string;
  name: string;
  state: VMState;
  cpu: number;
  memory: string;
  disk: string;
  created: Date;
  ipAddress?: string;
}

export interface ResourceConfig {
  cpu?: { cores?: number; shares?: number; reservation?: number };
  memory?: { size?: string; reservation?: string; limit?: string };
  disk?: { iops?: number };
}

export interface SnapshotConfig {
  vmId: string;
  name: string;
  description?: string;
  memory?: boolean;
}

export interface SnapshotInfo {
  id: string;
  name: string;
  vmId: string;
  created: Date;
  size: number;
}

export interface MigrationConfig {
  vmId: string;
  targetHost: string;
  live?: boolean;
  storageMotion?: boolean;
}

export enum VirtualizationErrorCode {
  VM_NOT_FOUND = 'V001',
  INSUFFICIENT_RESOURCES = 'V002',
  MIGRATION_FAILED = 'V003',
  SNAPSHOT_FAILED = 'V004',
}

export class VirtualizationError extends Error {
  constructor(
    public code: VirtualizationErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'VirtualizationError';
  }
}

export type { VMConfig, VMInfo, ResourceConfig, SnapshotConfig, SnapshotInfo, MigrationConfig };
export { VirtualizationErrorCode, VirtualizationError };
