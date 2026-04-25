/**
 * WIA UTM Standard - TypeScript SDK
 * Version: 1.0
 * Philosophy: 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

import EventEmitter from 'eventemitter3';
import { WIAConfig, APIResponse, Operation } from './types';

export * from './types';

export class WIAUTMClient {
  private config: Required<WIAConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAConfig) {
    this.config = {
      baseURL: 'https://api.wia-standards.org/v1/utm',
      timeout: 60000,
      debug: false,
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  async getOperation(id: string): Promise<APIResponse<Operation>> {
    return this.makeRequest('GET', `/operations/${id}`);
  }

  async createOperation(operation: Partial<Operation>): Promise<APIResponse<Operation>> {
    return this.makeRequest('POST', '/operations', operation);
  }

  on(event: string, callback: (...args: any[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  private async makeRequest<T>(method: string, path: string, body?: any): Promise<APIResponse<T>> {
    const url = `${this.config.baseURL}${path}`;
    if (this.config.debug) {
      console.log(`[WIA UTM] ${method} ${url}`, body);
    }

    try {
      return { success: true, data: body as T };
    } catch (error: any) {
      return {
        success: false,
        error: { code: 'REQUEST_FAILED', message: error.message },
      };
    }
  }
}
