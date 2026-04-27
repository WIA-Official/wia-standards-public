/**
 * WIA-MED-021 Sleep Monitoring - Type Definitions
 * @version 1.0.0
 */

export type SleepStage = 'awake' | 'light' | 'deep' | 'rem';
export type SleepQuality = 'poor' | 'fair' | 'good' | 'excellent';

export interface SleepSession {
  id: string;
  userId: string;
  startTime: string;
  endTime: string;
  totalDuration: number;
  stages: SleepStageData[];
  quality: SleepQuality;
  score: number;
  metrics: SleepMetrics;
}

export interface SleepStageData {
  stage: SleepStage;
  startTime: string;
  duration: number;
}

export interface SleepMetrics {
  deepSleepMinutes: number;
  remMinutes: number;
  lightSleepMinutes: number;
  awakeMinutes: number;
  efficiency: number;
  latency: number;
  interruptions: number;
  heartRateVariability?: number;
  respirationRate?: number;
}

export interface SDKConfig {
  apiKey: string;
  environment: 'production' | 'staging' | 'development';
  baseUrl?: string;
}
