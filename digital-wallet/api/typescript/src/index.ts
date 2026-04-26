/**
 * WIA-FIN-015 Digital Wallet Standard - TypeScript SDK
 * @module @wia/digital-wallet-sdk
 */

import * as bip39 from 'bip39';
import { EventEmitter } from 'events';
import type {
  Wallet,
  WalletType,
  WalletConfig,
  Transaction,
  Balance,
  SendPaymentRequest,
  SendPaymentOptions,
  PaymentRequest,
  NFCConfig,
  QRCodeOptions,
  ExchangeRate,
  ExchangeRequest,
  ExchangeResult,
  TransactionFilter,
  WalletEvent,
  EventHandler,
  Currency
} from './types';

export * from './types';

/**
 * Main Digital Wallet class
 */
export class DigitalWallet extends EventEmitter {
  private walletData: Wallet | null = null;
  private apiBaseUrl: string;
  private apiKey: string | null = null;

  constructor(config?: { apiBaseUrl?: string; apiKey?: string }) {
    super();
    this.apiBaseUrl = config?.apiBaseUrl || 'https://api.wia.org/v1';
    this.apiKey = config?.apiKey || null;
  }

  /**
   * Create a new wallet
   */
  static async create(config: WalletConfig): Promise<DigitalWallet> {
    const wallet = new DigitalWallet();

    // Generate recovery phrase for HD wallets
    let recoveryPhrase: string | undefined;
    if (config.type === WalletType.HD) {
      recoveryPhrase = bip39.generateMnemonic(128); // 12 words
    }

    // Make API call to create wallet
    const response = await wallet.makeApiCall('POST', '/wallets', {
      type: config.type,
      currencies: config.currencies,
      security: config.security,
      testMode: config.testMode,
      recoveryPhrase
    });

    wallet.walletData = {
      ...response.wallet,
      recoveryPhrase: config.type === WalletType.HD ? recoveryPhrase : undefined,
      createdAt: new Date(response.wallet.createdAt)
    };

    return wallet;
  }

  /**
   * Load wallet from recovery phrase
   */
  static async fromRecoveryPhrase(
    phrase: string,
    config?: Partial<WalletConfig>
  ): Promise<DigitalWallet> {
    // Validate mnemonic
    if (!bip39.validateMnemonic(phrase)) {
      throw new Error('Invalid recovery phrase');
    }

    const wallet = new DigitalWallet();

    const response = await wallet.makeApiCall('POST', '/wallets/recover', {
      recoveryPhrase: phrase,
      currencies: config?.currencies || ['USD', 'BTC', 'ETH']
    });

    wallet.walletData = {
      ...response.wallet,
      createdAt: new Date(response.wallet.createdAt)
    };

    return wallet;
  }

  /**
   * Load wallet from private key
   */
  static async fromPrivateKey(
    privateKey: string,
    config?: Partial<WalletConfig>
  ): Promise<DigitalWallet> {
    const wallet = new DigitalWallet();

    const response = await wallet.makeApiCall('POST', '/wallets/import', {
      privateKey,
      type: config?.type || WalletType.SIMPLE,
      currencies: config?.currencies || ['BTC', 'ETH']
    });

    wallet.walletData = {
      ...response.wallet,
      createdAt: new Date(response.wallet.createdAt)
    };

    return wallet;
  }

  /**
   * Get wallet address
   */
  get address(): string {
    if (!this.walletData) {
      throw new Error('Wallet not initialized');
    }
    return this.walletData.supportedCurrencies[0]?.addresses?.[0]?.address || '';
  }

  /**
   * Get wallet ID
   */
  get id(): string {
    return this.walletData?.walletId || '';
  }

  /**
   * Get recovery phrase (only available during wallet creation)
   */
  get recoveryPhrase(): string | undefined {
    return this.walletData?.recoveryPhrase;
  }

  /**
   * Get balance for a specific currency
   */
  async getBalance(currency: string): Promise<Balance> {
    const response = await this.makeApiCall(
      'GET',
      `/wallets/${this.id}/balance?currency=${currency}`
    );

    return {
      ...response.balances[0],
      updatedAt: new Date(response.updatedAt)
    };
  }

  /**
   * Get balances for all currencies
   */
  async getAllBalances(): Promise<Balance[]> {
    const response = await this.makeApiCall('GET', `/wallets/${this.id}/balance`);

    return response.balances.map((b: any) => ({
      ...b,
      updatedAt: new Date(response.updatedAt)
    }));
  }

  /**
   * Send payment
   */
  async send(
    request: SendPaymentRequest,
    options?: SendPaymentOptions
  ): Promise<Transaction> {
    const response = await this.makeApiCall('POST', `/wallets/${this.id}/send`, {
      ...request,
      authentication: options?.authenticate
    });

    const transaction: Transaction = {
      ...response.transaction,
      timestamp: new Date(response.transaction.timestamp)
    };

    // Emit event
    this.emit(WalletEvent.PAYMENT_SENT, transaction);

    return transaction;
  }

  /**
   * Create payment request
   */
  async createPaymentRequest(options: {
    amount: number;
    currency: string;
    merchant?: string;
    expires?: number;
  }): Promise<PaymentRequest> {
    const response = await this.makeApiCall(
      'POST',
      `/wallets/${this.id}/payment-request`,
      options
    );

    return {
      ...response,
      expires: response.expires ? new Date(response.expires) : undefined
    };
  }

  /**
   * Get transaction history
   */
  async getTransactions(filter?: TransactionFilter): Promise<Transaction[]> {
    const params = new URLSearchParams();
    if (filter?.limit) params.append('limit', filter.limit.toString());
    if (filter?.currency) params.append('currency', filter.currency);
    if (filter?.type) params.append('type', filter.type);

    const response = await this.makeApiCall(
      'GET',
      `/wallets/${this.id}/transactions?${params.toString()}`
    );

    return response.transactions.map((tx: any) => ({
      ...tx,
      timestamp: new Date(tx.timestamp)
    }));
  }

  /**
   * Get specific transaction
   */
  async getTransaction(transactionId: string): Promise<Transaction> {
    const response = await this.makeApiCall(
      'GET',
      `/wallets/${this.id}/transactions/${transactionId}`
    );

    return {
      ...response.transaction,
      timestamp: new Date(response.transaction.timestamp)
    };
  }

  /**
   * Get exchange rate
   */
  async getExchangeRate(from: string, to: string): Promise<ExchangeRate> {
    const response = await this.makeApiCall(
      'GET',
      `/exchange-rate?from=${from}&to=${to}`
    );

    return {
      from,
      to,
      rate: response.rate,
      timestamp: new Date(response.timestamp)
    };
  }

  /**
   * Exchange currency
   */
  async exchange(request: ExchangeRequest): Promise<ExchangeResult> {
    const response = await this.makeApiCall(
      'POST',
      `/wallets/${this.id}/exchange`,
      request
    );

    return response;
  }

  /**
   * NFC payment functionality
   */
  get nfc() {
    return {
      /**
       * Check if NFC is available
       */
      isAvailable: async (): Promise<boolean> => {
        // Check device capabilities
        return 'NDEFReader' in window;
      },

      /**
       * Enable NFC payments
       */
      enable: async (config: NFCConfig) => {
        const response = await this.makeApiCall(
          'POST',
          `/wallets/${this.id}/nfc/enable`,
          config
        );

        return {
          on: (event: 'tap', handler: EventHandler) => {
            this.on(`nfc_${event}`, handler);
          }
        };
      },

      /**
       * Process NFC payment
       */
      processPayment: async (paymentData: any) => {
        const response = await this.makeApiCall(
          'POST',
          `/wallets/${this.id}/nfc/pay`,
          paymentData
        );

        this.emit(WalletEvent.PAYMENT_SENT, response.transaction);
        return response;
      }
    };
  }

  /**
   * QR code functionality
   */
  get qr() {
    return {
      /**
       * Generate QR code
       */
      generate: async (options: QRCodeOptions) => {
        const response = await this.makeApiCall(
          'POST',
          `/wallets/${this.id}/qr/generate`,
          options
        );

        return {
          data: response.qrData,
          toImage: async (format?: { format?: string; size?: number }) => {
            // Generate QR code image
            const QRCode = require('qrcode');
            return QRCode.toDataURL(response.qrData, {
              width: format?.size || 300
            });
          }
        };
      },

      /**
       * Start QR scanner (browser only)
       */
      startScanner: async (options?: { camera?: string; continuous?: boolean }) => {
        // This would integrate with device camera
        // Implementation depends on platform
        return {
          on: (event: 'detected', handler: EventHandler) => {
            this.on(`qr_${event}`, handler);
          },
          stop: () => {
            // Stop camera
          }
        };
      },

      /**
       * Parse payment request from QR data
       */
      parsePaymentRequest: async (qrData: string) => {
        const response = await this.makeApiCall(
          'POST',
          `/wallets/${this.id}/qr/parse`,
          { qrData }
        );

        return response.paymentRequest;
      }
    };
  }

  /**
   * Security functionality
   */
  get security() {
    return {
      /**
       * Biometric authentication
       */
      biometric: {
        isAvailable: async () => {
          // Check if biometric is available
          return 'credentials' in navigator;
        },
        enable: async (options?: { type?: 'face' | 'fingerprint'; fallback?: string }) => {
          const response = await this.makeApiCall(
            'POST',
            `/wallets/${this.id}/security/biometric/enable`,
            options
          );
          return response;
        }
      },

      /**
       * Two-factor authentication
       */
      enable2FA: async (options?: { method?: 'totp' | 'sms'; phoneNumber?: string }) => {
        const response = await this.makeApiCall(
          'POST',
          `/wallets/${this.id}/security/2fa/enable`,
          options
        );

        return {
          qrCode: response.qrCode,
          secret: response.secret,
          verify: async (code: string) => {
            const verifyResponse = await this.makeApiCall(
              'POST',
              `/wallets/${this.id}/security/2fa/verify`,
              { code }
            );
            return verifyResponse.verified;
          }
        };
      }
    };
  }

  /**
   * DeFi functionality
   */
  get defi() {
    return {
      /**
       * Stake cryptocurrency
       */
      stake: async (options: {
        currency: string;
        amount: number;
        protocol: string;
        duration?: number;
      }) => {
        const response = await this.makeApiCall(
          'POST',
          `/wallets/${this.id}/stake`,
          options
        );
        return response;
      },

      /**
       * Get staking information
       */
      getStakes: async () => {
        const response = await this.makeApiCall('GET', `/wallets/${this.id}/stakes`);
        return response.stakes;
      }
    };
  }

  /**
   * Make API call
   */
  private async makeApiCall(method: string, endpoint: string, data?: any): Promise<any> {
    const url = `${this.apiBaseUrl}${endpoint}`;

    const headers: Record<string, string> = {
      'Content-Type': 'application/json'
    };

    if (this.apiKey) {
      headers['Authorization'] = `Bearer ${this.apiKey}`;
    }

    const options: RequestInit = {
      method,
      headers
    };

    if (data && (method === 'POST' || method === 'PUT')) {
      options.body = JSON.stringify(data);
    }

    const response = await fetch(url, options);

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message || 'API request failed');
    }

    return response.json();
  }
}

/**
 * Utility functions
 */
export const Utils = {
  /**
   * Validate address format
   */
  isValidAddress: (address: string, currency: string): boolean => {
    // Add validation logic for different currencies
    if (currency === 'BTC') {
      return /^(bc1|[13])[a-zA-HJ-NP-Z0-9]{25,90}$/.test(address);
    }
    if (currency === 'ETH' || currency.startsWith('ERC')) {
      return /^0x[a-fA-F0-9]{40}$/.test(address);
    }
    return true; // Fiat currencies use different formats
  },

  /**
   * Format currency amount
   */
  formatAmount: (amount: number, currency: string): string => {
    if (['USD', 'EUR', 'GBP'].includes(currency)) {
      return new Intl.NumberFormat('en-US', {
        style: 'currency',
        currency
      }).format(amount);
    }
    return `${amount} ${currency}`;
  },

  /**
   * Generate wallet ID
   */
  generateWalletId: (): string => {
    return `wlt_${Math.random().toString(36).substring(2, 15)}`;
  }
};

// Default export
export default DigitalWallet;
