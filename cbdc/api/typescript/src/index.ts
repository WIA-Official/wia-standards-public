/**
 * WIA CBDC Standard - TypeScript SDK
 * Standard: WIA-FIN-005
 * Version: 1.0.0
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import * as WebSocket from 'ws';
import {
  CBDCConfig,
  CBDCTransaction,
  Wallet,
  CreateWalletRequest,
  TransferRequest,
  CrossBorderTransferRequest,
  QueryBalanceRequest,
  QueryTransactionRequest,
  TransactionHistoryRequest,
  PaymentRequest,
  SmartContract,
  CrossBorderExchange,
  APIResponse,
  ValidationResult,
  CBDCEvent,
  CBDCStatistics,
  PaginatedResponse,
  NetworkType,
  TransactionType,
  TransactionStatus,
  PrivacyLevel,
  CertificationLevel
} from './types';

// Re-export types for convenience
export * from './types';

/**
 * Main CBDC SDK class
 */
export class WIACBDC {
  private config: CBDCConfig;
  private api: AxiosInstance;
  private ws?: WebSocket;
  private eventHandlers: Map<string, Function[]>;

  constructor(config: CBDCConfig) {
    this.config = {
      network: NetworkType.MAINNET,
      ...config
    };

    this.api = axios.create({
      baseURL: config.apiEndpoint || this.getDefaultEndpoint(),
      headers: {
        'Content-Type': 'application/json',
        'X-CBDC-ID': config.cbdcId,
        'X-WIA-Standard': 'WIA-FIN-005',
        'X-WIA-Version': '1.0.0'
      },
      timeout: 30000
    });

    this.eventHandlers = new Map();

    if (this.config.debug) {
      console.log('[WIACBDC] Initialized with config:', this.config);
    }
  }

  /**
   * Get default API endpoint based on network
   */
  private getDefaultEndpoint(): string {
    const network = this.config.network;
    const cbdcId = this.config.cbdcId;

    const endpoints: Record<NetworkType, string> = {
      [NetworkType.MAINNET]: `https://api.wia.live/cbdc/${cbdcId}`,
      [NetworkType.TESTNET]: `https://testnet-api.wia.live/cbdc/${cbdcId}`,
      [NetworkType.DEVNET]: `https://devnet-api.wia.live/cbdc/${cbdcId}`,
      [NetworkType.SANDBOX]: `http://localhost:8080/cbdc/${cbdcId}`
    };

    return endpoints[network];
  }

  // ============================================================================
  // Wallet Management
  // ============================================================================

  /**
   * Create a new CBDC wallet
   */
  async createWallet(request: CreateWalletRequest): Promise<APIResponse<Wallet>> {
    try {
      const response = await this.api.post<APIResponse<Wallet>>('/wallet', request);

      if (this.config.debug) {
        console.log('[WIACBDC] Wallet created:', response.data.data?.walletId);
      }

      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  /**
   * Get wallet details
   */
  async getWallet(walletId: string): Promise<APIResponse<Wallet>> {
    try {
      const response = await this.api.get<APIResponse<Wallet>>(`/wallet/${walletId}`);
      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  /**
   * Query wallet balance
   */
  async getBalance(request: QueryBalanceRequest): Promise<APIResponse<Wallet['balance']>> {
    try {
      const response = await this.api.get<APIResponse<Wallet['balance']>>(
        `/wallet/${request.walletId}/balance`
      );
      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  // ============================================================================
  // Transactions
  // ============================================================================

  /**
   * Create and sign a transaction
   */
  async signTransaction(tx: Partial<CBDCTransaction>): Promise<CBDCTransaction> {
    // In production, this would use actual cryptographic signing
    // For now, we create a properly formatted transaction
    const transaction: CBDCTransaction = {
      version: '1.0.0',
      standard: 'WIA-FIN-005',
      timestamp: new Date().toISOString(),
      currency: tx.currency || {
        code: 'XXX',
        cbdcId: this.config.cbdcId,
        issuer: 'CENTRAL_BANK',
        precision: 2
      },
      type: tx.type || TransactionType.TRANSFER,
      sender: tx.sender || { walletId: '', signature: '' },
      recipient: tx.recipient || { walletId: '' },
      amount: tx.amount || { value: '0.00', precision: 2 },
      fee: tx.fee || { value: '0.00', payer: 'sender' },
      privacy: tx.privacy || { level: PrivacyLevel.STANDARD, technique: 'zksnark' },
      compliance: tx.compliance || {
        aml: true,
        sanctions: 'checked',
        taxReporting: 'required'
      },
      metadata: tx.metadata,
      ...tx
    };

    // Sign the transaction (placeholder - real implementation would use private key)
    if (this.config.privateKey) {
      transaction.sender.signature = await this.sign(JSON.stringify(transaction));
    }

    return transaction;
  }

  /**
   * Submit a signed transaction
   */
  async submitTransaction(tx: CBDCTransaction): Promise<APIResponse<CBDCTransaction>> {
    try {
      const response = await this.api.post<APIResponse<CBDCTransaction>>('/transaction', tx);

      if (this.config.debug) {
        console.log('[WIACBDC] Transaction submitted:', response.data.data?.transactionId);
      }

      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  /**
   * Simple transfer helper
   */
  async transfer(request: TransferRequest): Promise<APIResponse<CBDCTransaction>> {
    const tx = await this.signTransaction({
      type: TransactionType.TRANSFER,
      sender: { walletId: request.from },
      recipient: { walletId: request.to },
      amount: { value: request.amount, precision: 2 },
      privacy: request.privacy,
      metadata: request.metadata
    });

    return this.submitTransaction(tx);
  }

  /**
   * Get transaction details
   */
  async getTransaction(request: QueryTransactionRequest): Promise<APIResponse<CBDCTransaction>> {
    try {
      const response = await this.api.get<APIResponse<CBDCTransaction>>(
        `/transaction/${request.transactionId}`
      );
      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  /**
   * Get transaction history
   */
  async getTransactionHistory(
    request: TransactionHistoryRequest
  ): Promise<APIResponse<PaginatedResponse<CBDCTransaction>>> {
    try {
      const params = new URLSearchParams();
      if (request.startDate) params.append('startDate', request.startDate);
      if (request.endDate) params.append('endDate', request.endDate);
      if (request.limit) params.append('limit', request.limit.toString());
      if (request.offset) params.append('offset', request.offset.toString());
      if (request.types) params.append('types', request.types.join(','));

      const response = await this.api.get<APIResponse<PaginatedResponse<CBDCTransaction>>>(
        `/wallet/${request.walletId}/transactions?${params}`
      );
      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  // ============================================================================
  // Cross-Border Payments
  // ============================================================================

  /**
   * Initiate cross-border transfer
   */
  async crossBorderTransfer(
    request: CrossBorderTransferRequest
  ): Promise<APIResponse<CrossBorderExchange>> {
    try {
      const response = await this.api.post<APIResponse<CrossBorderExchange>>(
        '/cross-border/transfer',
        request
      );

      if (this.config.debug) {
        console.log('[WIACBDC] Cross-border exchange initiated:', response.data.data?.exchangeId);
      }

      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  /**
   * Get exchange status
   */
  async getExchangeStatus(exchangeId: string): Promise<APIResponse<CrossBorderExchange>> {
    try {
      const response = await this.api.get<APIResponse<CrossBorderExchange>>(
        `/cross-border/exchange/${exchangeId}`
      );
      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  // ============================================================================
  // Payment Requests
  // ============================================================================

  /**
   * Create payment request
   */
  async createPaymentRequest(
    amount: string,
    merchant: string,
    options?: Partial<PaymentRequest>
  ): Promise<APIResponse<PaymentRequest>> {
    try {
      const request: Partial<PaymentRequest> = {
        merchant,
        amount: { value: amount, precision: 2 },
        currency: {
          code: 'XXX',
          cbdcId: this.config.cbdcId,
          issuer: 'CENTRAL_BANK',
          precision: 2
        },
        acceptedCBDCs: [this.config.cbdcId],
        expiresAt: new Date(Date.now() + 3600000).toISOString(),
        ...options
      };

      const response = await this.api.post<APIResponse<PaymentRequest>>(
        '/payment-request',
        request
      );
      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  /**
   * Pay a payment request
   */
  async payRequest(
    requestId: string,
    fromWallet: string
  ): Promise<APIResponse<CBDCTransaction>> {
    try {
      const response = await this.api.post<APIResponse<CBDCTransaction>>(
        `/payment-request/${requestId}/pay`,
        { fromWallet }
      );
      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  // ============================================================================
  // Smart Contracts
  // ============================================================================

  /**
   * Deploy smart contract
   */
  async deployContract(contract: Partial<SmartContract>): Promise<APIResponse<SmartContract>> {
    try {
      const response = await this.api.post<APIResponse<SmartContract>>('/contract', contract);
      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  /**
   * Execute smart contract function
   */
  async executeContract(
    contractId: string,
    functionName: string,
    parameters: any[]
  ): Promise<APIResponse<any>> {
    try {
      const response = await this.api.post<APIResponse<any>>(
        `/contract/${contractId}/execute`,
        { functionName, parameters }
      );
      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  // ============================================================================
  // Statistics and Monitoring
  // ============================================================================

  /**
   * Get CBDC statistics
   */
  async getStatistics(): Promise<APIResponse<CBDCStatistics>> {
    try {
      const response = await this.api.get<APIResponse<CBDCStatistics>>('/statistics');
      return response.data;
    } catch (error) {
      return this.handleError(error);
    }
  }

  // ============================================================================
  // Validation
  // ============================================================================

  /**
   * Validate transaction
   */
  validateTransaction(tx: CBDCTransaction): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Check required fields
    if (!tx.version) errors.push('Missing version');
    if (tx.standard !== 'WIA-FIN-005') errors.push('Invalid standard');
    if (!tx.sender?.walletId) errors.push('Missing sender wallet ID');
    if (!tx.recipient?.walletId) errors.push('Missing recipient wallet ID');
    if (!tx.amount?.value || parseFloat(tx.amount.value) <= 0) {
      errors.push('Invalid amount');
    }

    // Check timestamp
    const txTime = new Date(tx.timestamp).getTime();
    const now = Date.now();
    if (Math.abs(txTime - now) > 300000) { // 5 minutes
      warnings.push('Transaction timestamp outside acceptable range');
    }

    // Check signature
    if (!tx.sender.signature) {
      errors.push('Missing signature');
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined,
      warnings: warnings.length > 0 ? warnings : undefined
    };
  }

  // ============================================================================
  // Real-Time Events (WebSocket)
  // ============================================================================

  /**
   * Connect to WebSocket for real-time events
   */
  async connect(): Promise<void> {
    const wsEndpoint = this.config.wsEndpoint || this.getDefaultEndpoint().replace('https', 'wss');

    this.ws = new WebSocket(wsEndpoint);

    this.ws.on('open', () => {
      if (this.config.debug) {
        console.log('[WIACBDC] WebSocket connected');
      }
      this.emit('connected', {});
    });

    this.ws.on('message', (data: WebSocket.Data) => {
      try {
        const event: CBDCEvent = JSON.parse(data.toString());
        this.emit(event.type, event);
        this.emit('*', event); // Wildcard for all events
      } catch (error) {
        console.error('[WIACBDC] Failed to parse WebSocket message:', error);
      }
    });

    this.ws.on('error', (error) => {
      console.error('[WIACBDC] WebSocket error:', error);
      this.emit('error', error);
    });

    this.ws.on('close', () => {
      if (this.config.debug) {
        console.log('[WIACBDC] WebSocket disconnected');
      }
      this.emit('disconnected', {});
    });
  }

  /**
   * Disconnect WebSocket
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = undefined;
    }
  }

  /**
   * Register event handler
   */
  on(event: string, handler: Function): void {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, []);
    }
    this.eventHandlers.get(event)!.push(handler);
  }

  /**
   * Unregister event handler
   */
  off(event: string, handler: Function): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      const index = handlers.indexOf(handler);
      if (index >= 0) {
        handlers.splice(index, 1);
      }
    }
  }

  /**
   * Emit event to handlers
   */
  private emit(event: string, data: any): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      handlers.forEach(handler => {
        try {
          handler(data);
        } catch (error) {
          console.error(`[WIACBDC] Event handler error for '${event}':`, error);
        }
      });
    }
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Sign data (placeholder - real implementation would use actual cryptography)
   */
  private async sign(data: string): Promise<string> {
    // In production, this would use the private key to create a signature
    // Using quantum-resistant algorithms like CRYSTALS-Dilithium
    return `signature_${Buffer.from(data).toString('base64').substring(0, 64)}`;
  }

  /**
   * Handle API errors
   */
  private handleError(error: any): APIResponse<any> {
    const message = error.response?.data?.error?.message || error.message || 'Unknown error';
    const code = error.response?.data?.error?.code || 'UNKNOWN_ERROR';

    if (this.config.debug) {
      console.error('[WIACBDC] Error:', message);
    }

    return {
      success: false,
      error: {
        code,
        message,
        details: error.response?.data?.error?.details
      }
    };
  }

  /**
   * Get SDK version
   */
  static getVersion(): string {
    return '1.0.0';
  }

  /**
   * Get supported standards
   */
  static getSupportedStandards(): string[] {
    return ['WIA-FIN-005'];
  }
}

/**
 * Create a CBDC instance
 */
export function createCBDC(config: CBDCConfig): WIACBDC {
  return new WIACBDC(config);
}

/**
 * Default export
 */
export default WIACBDC;
