/**
 * WIA-MENTAL-010-mindfulness-app: Mindfulness App
 * TypeScript Type Definitions
 * 弘益人間 (Benefit All Humanity)
 */

export interface Config {
  id: string;
  type: string;
  settings: Record<string, any>;
}

export interface Assessment {
  score: number;
  severity: string;
  timestamp: Date;
}

export interface ProgressMetrics {
  effectiveness: number;
  engagement: number;
  improvement: number;
}

export default Config;
