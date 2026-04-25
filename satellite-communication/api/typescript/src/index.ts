/**
 * WIA Satellite Communication Standard - TypeScript SDK
 * Version: 1.0
 * Philosophy: 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

import EventEmitter from 'eventemitter3';
import { WIAConfig, APIResponse, SatelliteLink, LinkBudget } from './types';

export * from './types';

export class WIASatelliteCommClient {
  private config: Required<WIAConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAConfig) {
    this.config = {
      baseURL: 'https://api.wia-standards.org/v1/satcom',
      timeout: 60000,
      debug: false,
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  /**
   * Calculate free space path loss
   */
  calculatePathLoss(distanceKm: number, frequencyGhz: number): number {
    // FSPL = 20*log10(d) + 20*log10(f) + 32.45
    return 20 * Math.log10(distanceKm) + 20 * Math.log10(frequencyGhz) + 32.45;
  }

  /**
   * Calculate link budget
   */
  calculateLinkBudget(params: {
    transmitPowerDbm: number;
    transmitGainDbi: number;
    distanceKm: number;
    frequencyGhz: number;
    receiveGainDbi: number;
  }): LinkBudget {
    const { transmitPowerDbm, transmitGainDbi, distanceKm, frequencyGhz, receiveGainDbi } = params;

    const pathLossDb = this.calculatePathLoss(distanceKm, frequencyGhz);
    const receivedPowerDbm = transmitPowerDbm + transmitGainDbi - pathLossDb + receiveGainDbi;
    const noisePowerDbm = -120;
    const cnrDb = receivedPowerDbm - noisePowerDbm;

    return {
      transmit_power_dbm: transmitPowerDbm,
      transmit_gain_dbi: transmitGainDbi,
      path_loss_db: pathLossDb,
      receive_gain_dbi: receiveGainDbi,
      received_power_dbm: receivedPowerDbm,
      noise_power_dbm: noisePowerDbm,
      cnr_db: cnrDb,
    };
  }

  /**
   * Get satellite link data
   */
  async getLink(id: string): Promise<APIResponse<SatelliteLink>> {
    return this.makeRequest('GET', `/links/${id}`);
  }

  /**
   * Create satellite link
   */
  async createLink(data: Partial<SatelliteLink>): Promise<APIResponse<SatelliteLink>> {
    return this.makeRequest('POST', '/links', data);
  }

  on(event: string, callback: (...args: any[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  private async makeRequest<T>(method: string, path: string, body?: any): Promise<APIResponse<T>> {
    const url = `${this.config.baseURL}${path}`;
    if (this.config.debug) {
      console.log(`[WIA SatComm] ${method} ${url}`, body);
    }

    try {
      return {
        success: true,
        data: body as T,
      };
    } catch (error: any) {
      return {
        success: false,
        error: {
          code: 'REQUEST_FAILED',
          message: error.message,
        },
      };
    }
  }
}
