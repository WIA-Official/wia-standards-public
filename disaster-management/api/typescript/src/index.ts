/**
 * WIA-CITY-018: Disaster Management API
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

import { LightingConfig, SystemMetrics, ZoneConfig } from './types';

export * from './types';

export class SmartLighting {
  private config: LightingConfig;
  private zones: Map<string, ZoneConfig> = new Map();

  constructor(config: LightingConfig) {
    this.config = config;
  }

  async start(): Promise<void> {
    console.log('🚨 Starting WIA-CITY-018 Disaster Management System...');
    console.log('弘益人間 - Benefit All Humanity\n');
    if (this.config.zones) {
      this.config.zones.forEach(zone => this.addZone(zone));
    }
    console.log('✅ Lighting system started successfully');
  }

  async stop(): Promise<void> {
    console.log('⏹️  Stopping lighting system...');
    console.log('✅ System stopped');
  }

  addZone(zone: ZoneConfig): void {
    const zoneId = zone.id || `zone-${this.zones.size + 1}`;
    this.zones.set(zoneId, { ...zone, id: zoneId });
    console.log(`➕ Added zone: ${zone.name}`);
  }

  getStatus(): SystemMetrics {
    return {
      energySavings: 70,
      efficacy: 150,
      brightness: this.config.brightness,
      colorTemperature: this.config.colorTemperature || 4000,
      occupancy: 12,
      illuminance: 500
    };
  }

  async setBrightness(level: number): Promise<void> {
    this.config.brightness = Math.max(0, Math.min(100, level));
    console.log(`🚨 Brightness set to ${this.config.brightness}%`);
  }
}

export function getDefaultConfig(): LightingConfig {
  return {
    mode: 'auto',
    brightness: 75,
    colorTemperature: 4000,
    certificationLevel: 'silver',
    zones: []
  };
}
