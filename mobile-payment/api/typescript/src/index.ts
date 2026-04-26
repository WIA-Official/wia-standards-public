/**
 * WIA-FIN-013 Mobile Payment Standard SDK
 * @version 1.0.0
 * @license MIT
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import {
  WIAConfig,
  WIAResponse,
  Transaction,
  Wallet,
  Token,
  QRCode,
  BiometricAuthentication,
  P2PTransfer,
  ProcessPaymentRequest,
  CreateWalletRequest,
  AddPaymentMethodRequest,
  GenerateQRCodeRequest,
  BiometricAuthRequest,
  Amount,
} from './types';

/**
 * Main WIA Mobile Payment SDK class
 */
export class WIAMobilePayment {
  private client: AxiosInstance;
  private config: Required<WIAConfig>;

  constructor(config: WIAConfig) {
    this.config = {
      apiKey: config.apiKey,
      environment: config.environment || 'production',
      baseUrl: config.baseUrl || this.getDefaultBaseUrl(config.environment || 'production'),
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
      debug: config.debug || false,
    };

    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.config.apiKey,
        'User-Agent': 'WIA-Mobile-Payment-SDK/1.0.0',
      },
    });

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error) => this.handleError(error)
    );

    if (this.config.debug) {
      this.enableDebugLogging();
    }
  }

  /**
   * Get default base URL based on environment
   */
  private getDefaultBaseUrl(environment: string): string {
    switch (environment) {
      case 'production':
        return 'https://api.wiastandards.com/mobile-payment/v1';
      case 'test':
        return 'https://api-test.wiastandards.com/mobile-payment/v1';
      case 'sandbox':
        return 'https://api-sandbox.wiastandards.com/mobile-payment/v1';
      default:
        return 'https://api.wiastandards.com/mobile-payment/v1';
    }
  }

  /**
   * Enable debug logging
   */
  private enableDebugLogging(): void {
    this.client.interceptors.request.use((config) => {
      console.log('[WIA SDK] Request:', config.method?.toUpperCase(), config.url);
      console.log('[WIA SDK] Data:', config.data);
      return config;
    });

    this.client.interceptors.response.use((response) => {
      console.log('[WIA SDK] Response:', response.status, response.data);
      return response;
    });
  }

  /**
   * Handle API errors
   */
  private handleError(error: AxiosError): Promise<never> {
    if (error.response) {
      // Server responded with error
      const errorData = error.response.data as any;
      throw new Error(
        errorData?.error?.message || `API Error: ${error.response.status}`
      );
    } else if (error.request) {
      // No response received
      throw new Error('Network error: No response from server');
    } else {
      // Request setup error
      throw new Error(`Request error: ${error.message}`);
    }
  }

  /**
   * Normalize amount to standard format
   */
  private normalizeAmount(amount: number | Amount, currency?: string): Amount {
    if (typeof amount === 'number') {
      return {
        value: amount.toFixed(2),
        currency: currency || 'USD',
        minorUnits: 2,
      };
    }
    return amount;
  }

  // ============================================================================
  // Payment Operations
  // ============================================================================

  /**
   * Process a payment transaction
   */
  async processPayment(
    request: ProcessPaymentRequest
  ): Promise<WIAResponse<Transaction>> {
    const data = {
      ...request,
      amount: this.normalizeAmount(request.amount, request.currency),
    };

    const response = await this.client.post<WIAResponse<Transaction>>(
      '/payments',
      data
    );

    return response.data;
  }

  /**
   * Get transaction details
   */
  async getTransaction(transactionId: string): Promise<WIAResponse<Transaction>> {
    const response = await this.client.get<WIAResponse<Transaction>>(
      `/payments/${transactionId}`
    );

    return response.data;
  }

  /**
   * Refund a payment
   */
  async refundPayment(
    transactionId: string,
    amount?: number | Amount,
    reason?: string
  ): Promise<WIAResponse<Transaction>> {
    const data: any = { reason };

    if (amount) {
      data.amount = this.normalizeAmount(amount);
    }

    const response = await this.client.post<WIAResponse<Transaction>>(
      `/payments/${transactionId}/refund`,
      data
    );

    return response.data;
  }

  /**
   * Cancel a payment
   */
  async cancelPayment(transactionId: string): Promise<WIAResponse<Transaction>> {
    const response = await this.client.post<WIAResponse<Transaction>>(
      `/payments/${transactionId}/cancel`
    );

    return response.data;
  }

  // ============================================================================
  // Wallet Operations
  // ============================================================================

  /**
   * Create a new wallet
   */
  async createWallet(request: CreateWalletRequest): Promise<WIAResponse<Wallet>> {
    const response = await this.client.post<WIAResponse<Wallet>>(
      '/wallets',
      request
    );

    return response.data;
  }

  /**
   * Get wallet details
   */
  async getWallet(walletId: string): Promise<WIAResponse<Wallet>> {
    const response = await this.client.get<WIAResponse<Wallet>>(
      `/wallets/${walletId}`
    );

    return response.data;
  }

  /**
   * Add payment method to wallet
   */
  async addPaymentMethod(
    request: AddPaymentMethodRequest
  ): Promise<WIAResponse<Wallet>> {
    const { walletId, ...data } = request;

    const response = await this.client.post<WIAResponse<Wallet>>(
      `/wallets/${walletId}/payment-methods`,
      data
    );

    return response.data;
  }

  /**
   * Remove payment method from wallet
   */
  async removePaymentMethod(
    walletId: string,
    paymentMethodId: string
  ): Promise<WIAResponse<Wallet>> {
    const response = await this.client.delete<WIAResponse<Wallet>>(
      `/wallets/${walletId}/payment-methods/${paymentMethodId}`
    );

    return response.data;
  }

  /**
   * Get wallet balance
   */
  async getWalletBalance(walletId: string): Promise<WIAResponse<Amount>> {
    const response = await this.client.get<WIAResponse<Amount>>(
      `/wallets/${walletId}/balance`
    );

    return response.data;
  }

  // ============================================================================
  // Token Operations
  // ============================================================================

  /**
   * Create payment token
   */
  async createToken(
    cardNumber: string,
    expiryMonth: number,
    expiryYear: number,
    cvv: string,
    deviceId: string
  ): Promise<WIAResponse<Token>> {
    const data = {
      cardNumber,
      expiryMonth,
      expiryYear,
      cvv,
      deviceId,
    };

    const response = await this.client.post<WIAResponse<Token>>('/tokens', data);

    return response.data;
  }

  /**
   * Get token details
   */
  async getToken(tokenId: string): Promise<WIAResponse<Token>> {
    const response = await this.client.get<WIAResponse<Token>>(
      `/tokens/${tokenId}`
    );

    return response.data;
  }

  /**
   * Delete token
   */
  async deleteToken(tokenId: string): Promise<WIAResponse<void>> {
    const response = await this.client.delete<WIAResponse<void>>(
      `/tokens/${tokenId}`
    );

    return response.data;
  }

  /**
   * Suspend token
   */
  async suspendToken(tokenId: string): Promise<WIAResponse<Token>> {
    const response = await this.client.post<WIAResponse<Token>>(
      `/tokens/${tokenId}/suspend`
    );

    return response.data;
  }

  /**
   * Resume suspended token
   */
  async resumeToken(tokenId: string): Promise<WIAResponse<Token>> {
    const response = await this.client.post<WIAResponse<Token>>(
      `/tokens/${tokenId}/resume`
    );

    return response.data;
  }

  // ============================================================================
  // QR Code Operations
  // ============================================================================

  /**
   * Generate QR code for payment
   */
  async generateQRCode(
    request: GenerateQRCodeRequest
  ): Promise<WIAResponse<QRCode>> {
    const data = {
      ...request,
      amount: request.amount
        ? this.normalizeAmount(request.amount, request.currency)
        : undefined,
    };

    const response = await this.client.post<WIAResponse<QRCode>>(
      '/qr-codes',
      data
    );

    return response.data;
  }

  /**
   * Get QR code details
   */
  async getQRCode(qrCodeId: string): Promise<WIAResponse<QRCode>> {
    const response = await this.client.get<WIAResponse<QRCode>>(
      `/qr-codes/${qrCodeId}`
    );

    return response.data;
  }

  /**
   * Scan and process QR code
   */
  async scanQRCode(qrData: string): Promise<WIAResponse<Transaction>> {
    const data = { qrData };

    const response = await this.client.post<WIAResponse<Transaction>>(
      '/qr-codes/scan',
      data
    );

    return response.data;
  }

  // ============================================================================
  // Biometric Operations
  // ============================================================================

  /**
   * Authenticate with biometric
   */
  async authenticateBiometric(
    request?: BiometricAuthRequest
  ): Promise<WIAResponse<BiometricAuthentication>> {
    const response = await this.client.post<
      WIAResponse<BiometricAuthentication>
    >('/auth/biometric', request || {});

    return response.data;
  }

  /**
   * Verify biometric authentication
   */
  async verifyBiometric(
    authenticationId: string
  ): Promise<WIAResponse<BiometricAuthentication>> {
    const response = await this.client.get<
      WIAResponse<BiometricAuthentication>
    >(`/auth/biometric/${authenticationId}`);

    return response.data;
  }

  // ============================================================================
  // P2P Transfer Operations
  // ============================================================================

  /**
   * Send P2P transfer
   */
  async sendP2PTransfer(
    senderWalletId: string,
    recipientWalletId: string,
    amount: number | Amount,
    message?: string
  ): Promise<WIAResponse<P2PTransfer>> {
    const data = {
      senderWalletId,
      recipientWalletId,
      amount: this.normalizeAmount(amount),
      message,
    };

    const response = await this.client.post<WIAResponse<P2PTransfer>>(
      '/p2p-transfers',
      data
    );

    return response.data;
  }

  /**
   * Get P2P transfer details
   */
  async getP2PTransfer(transferId: string): Promise<WIAResponse<P2PTransfer>> {
    const response = await this.client.get<WIAResponse<P2PTransfer>>(
      `/p2p-transfers/${transferId}`
    );

    return response.data;
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Check SDK health/status
   */
  async healthCheck(): Promise<WIAResponse<{ status: string; version: string }>> {
    const response = await this.client.get<
      WIAResponse<{ status: string; version: string }>
    >('/health');

    return response.data;
  }

  /**
   * Get supported currencies
   */
  async getSupportedCurrencies(): Promise<WIAResponse<string[]>> {
    const response = await this.client.get<WIAResponse<string[]>>('/currencies');

    return response.data;
  }

  /**
   * Get supported payment methods
   */
  async getSupportedPaymentMethods(): Promise<WIAResponse<string[]>> {
    const response = await this.client.get<WIAResponse<string[]>>(
      '/payment-methods'
    );

    return response.data;
  }
}

// Export all types
export * from './types';

// Default export
export default WIAMobilePayment;
