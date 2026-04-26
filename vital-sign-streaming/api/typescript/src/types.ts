/**
 * WIA Vital Sign Streaming Standard - TypeScript Type Definitions
 * Version: 1.0.0
 */

export type StreamProtocol = 'websocket' | 'mqtt' | 'grpc' | 'sse';
export type VitalSignType = 'ecg' | 'ppg' | 'eeg' | 'emg' | 'spo2' | 'blood_pressure';

export interface VitalSignStream {
  format: 'WIA-VITAL-SIGN-STREAMING-v1.0';
  timestamp: string;
  stream_id: string;
  patient_id: string;
  vital_sign_type: VitalSignType;
  protocol: StreamProtocol;
  sample_rate_hz: number;
  data_points: Array<{
    timestamp: string;
    value: number | number[];
    unit: string;
  }>;
  metadata: {
    device_id: string;
    quality_score?: number;
    signal_strength?: number;
  };
}

export interface StreamConfiguration {
  stream_id: string;
  patient_id: string;
  vital_sign_types: VitalSignType[];
  protocol: StreamProtocol;
  sample_rate_hz: number;
  buffer_size: number;
  compression_enabled: boolean;
  encryption_enabled: boolean;
}

export interface StreamMetrics {
  stream_id: string;
  uptime_seconds: number;
  packets_sent: number;
  packets_lost: number;
  average_latency_ms: number;
  bandwidth_kbps: number;
}
