/**
 * WIA Vital Sign Streaming Standard - SDK
 * Version: 1.0.0
 */

import { VitalSignStream, StreamConfiguration, StreamMetrics, VitalSignType } from './types';

export * from './types';

export class VitalSignStreamingSDK {
  private apiEndpoint: string;
  private apiKey: string;
  private ws: WebSocket | null = null;

  constructor(apiEndpoint: string, apiKey: string) {
    this.apiEndpoint = apiEndpoint;
    this.apiKey = apiKey;
  }

  async startStream(config: StreamConfiguration): Promise<{ success: boolean; stream_id: string }> {
    const response = await fetch(\`\${this.apiEndpoint}/streams/start\`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.apiKey
      },
      body: JSON.stringify(config)
    });

    const data = await response.json();
    return { success: response.ok, stream_id: data.stream_id };
  }

  async stopStream(streamId: string): Promise<{ success: boolean }> {
    const response = await fetch(\`\${this.apiEndpoint}/streams/\${streamId}/stop\`, {
      method: 'POST',
      headers: { 'X-API-Key': this.apiKey }
    });

    return { success: response.ok };
  }

  async getStreamMetrics(streamId: string): Promise<StreamMetrics> {
    const response = await fetch(\`\${this.apiEndpoint}/streams/\${streamId}/metrics\`, {
      headers: { 'X-API-Key': this.apiKey }
    });

    return await response.json();
  }

  connectWebSocket(streamId: string, onData: (data: VitalSignStream) => void): void {
    const wsUrl = this.apiEndpoint.replace('http', 'ws') + \`/streams/\${streamId}/ws\`;
    this.ws = new WebSocket(wsUrl);

    this.ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      onData(data);
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };
  }

  disconnectWebSocket(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  static generateStreamId(): string {
    return 'STREAM-' + Date.now() + '-' + Math.random().toString(36).substr(2, 9).toUpperCase();
  }
}

export default VitalSignStreamingSDK;
