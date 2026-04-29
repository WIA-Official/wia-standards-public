/**
 * WIA-CHILD-003-content-rating: TypeScript Type Definitions
 * 弘益人間 - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 SmileStory Inc. / WIA
 */

export type ThreatLevel = 'safe' | 'low' | 'medium' | 'high' | 'critical';
export type AgeRange = 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17;
export type ActionType = 'allow' | 'warn' | 'block' | 'report';

export interface Config {
  apiKey: string;
  environment?: 'production' | 'staging' | 'development';
  childAge: AgeRange;
  realTimeMonitoring?: boolean;
  privacyMode?: 'strict' | 'balanced';
}

export interface AnalysisResult {
  threatLevel: ThreatLevel;
  confidence: number;
  action: ActionType;
  timestamp: number;
}

export interface Alert {
  id: string;
  level: 'info' | 'warning' | 'critical';
  message: string;
  timestamp: number;
}

export interface APIResponse<T = any> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
  };
  timestamp: number;
}

/**
 * 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */
