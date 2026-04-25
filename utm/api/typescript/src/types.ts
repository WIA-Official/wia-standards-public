/**
 * WIA UTM Standard - TypeScript Types
 * Version: 1.0
 * Philosophy: 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

export type OperationType = 'VLOS' | 'BVLOS' | 'EVLOS';
export type OperationState = 'PROPOSED' | 'ACCEPTED' | 'ACTIVATED' | 'CLOSED';

export interface Position {
  latitude: number;
  longitude: number;
  altitude: number;
}

export interface Operation {
  id: string;
  uasId: string;
  type: OperationType;
  state: OperationState;
  volumes: OperationVolume[];
  ussProvider: string;
}

export interface OperationVolume {
  volumeId: string;
  timeStart: string;
  timeEnd: string;
  minAltitude: number;
  maxAltitude: number;
  polygon: Position[];
}

export interface WIAConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
  };
}
