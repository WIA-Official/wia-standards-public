import axios from 'axios';
import { FloodPrediction, RiverGauge } from './types';

export class FloodClient {
  private baseUrl: string;
  private apiKey: string;

  constructor(config: {apiKey: string; baseUrl?: string}) {
    this.apiKey = config.apiKey;
    this.baseUrl = config.baseUrl || 'https://api.wia-flood.io/v1';
  }

  async getPrediction(lat: number, lng: number): Promise<FloodPrediction> {
    const response = await axios.get(`${this.baseUrl}/predictions`, {
      params: {lat, lng},
      headers: {'X-API-Key': this.apiKey}
    });
    return response.data;
  }

  async getRiverGauges(region: string): Promise<RiverGauge[]> {
    const response = await axios.get(`${this.baseUrl}/gauges`, {
      params: {region},
      headers: {'X-API-Key': this.apiKey}
    });
    return response.data.gauges;
  }
}

export * from './types';
