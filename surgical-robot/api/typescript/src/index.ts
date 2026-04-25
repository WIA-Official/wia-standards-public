/**
 * WIA Surgical Robot Standard - SDK
 * Version: 1.0.0
 */

import { SurgicalRobotData, SurgicalProcedure, SurgeryType } from './types';

export * from './types';

export class SurgicalRobotSDK {
  private apiEndpoint: string;
  private apiKey: string;

  constructor(apiEndpoint: string, apiKey: string) {
    this.apiEndpoint = apiEndpoint;
    this.apiKey = apiKey;
  }

  async getRobotStatus(robotId: string): Promise<SurgicalRobotData> {
    const response = await fetch(\`\${this.apiEndpoint}/robots/\${robotId}/status\`, {
      headers: { 'X-API-Key': this.apiKey }
    });

    return await response.json();
  }

  async startProcedure(procedure: Partial<SurgicalProcedure>): Promise<{ success: boolean; procedure_id: string }> {
    const response = await fetch(\`\${this.apiEndpoint}/procedures\`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.apiKey
      },
      body: JSON.stringify(procedure)
    });

    const data = await response.json();
    return { success: response.ok, procedure_id: data.procedure_id };
  }

  async sendCommand(robotId: string, command: string, parameters: Record<string, any>): Promise<{ success: boolean }> {
    const response = await fetch(\`\${this.apiEndpoint}/robots/\${robotId}/command\`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.apiKey
      },
      body: JSON.stringify({ command, parameters })
    });

    return { success: response.ok };
  }

  static generateProcedureId(): string {
    return 'PROC-' + Date.now() + '-' + Math.random().toString(36).substr(2, 9).toUpperCase();
  }
}

export default SurgicalRobotSDK;
