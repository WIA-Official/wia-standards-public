/**
 * WIA Telemedicine Standard - SDK
 * Version: 1.0.0
 */

import { TelemedicineSession, ConsultationType, TelehealthProvider } from './types';

export * from './types';

export class TelemedicineSDK {
  private apiEndpoint: string;
  private apiKey: string;

  constructor(apiEndpoint: string, apiKey: string) {
    this.apiEndpoint = apiEndpoint;
    this.apiKey = apiKey;
  }

  async createSession(
    patientId: string,
    providerId: string,
    consultationType: ConsultationType
  ): Promise<TelemedicineSession> {
    const response = await fetch(\`\${this.apiEndpoint}/sessions\`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.apiKey
      },
      body: JSON.stringify({ patient_id: patientId, provider_id: providerId, consultation_type: consultationType })
    });

    return await response.json();
  }

  async getSession(sessionId: string): Promise<TelemedicineSession> {
    const response = await fetch(\`\${this.apiEndpoint}/sessions/\${sessionId}\`, {
      headers: { 'X-API-Key': this.apiKey }
    });

    return await response.json();
  }

  async listProviders(specialty?: string): Promise<TelehealthProvider[]> {
    const params = specialty ? \`?specialty=\${specialty}\` : '';
    const response = await fetch(\`\${this.apiEndpoint}/providers\${params}\`, {
      headers: { 'X-API-Key': this.apiKey }
    });

    return await response.json();
  }

  static generateSessionId(): string {
    return 'TM-' + Date.now() + '-' + Math.random().toString(36).substr(2, 9).toUpperCase();
  }
}

export default TelemedicineSDK;
