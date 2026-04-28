/**
 * WIA-FIN-007 Smart Contract SDK
 * @packageDocumentation
 */

import { ethers, BigNumber } from 'ethers';
import type {
  WIAContractConfig,
  WIAContractMetadata,
  TxOptions,
  TransactionResponse,
  TransactionReceipt,
  EventListener,
  EventFilter,
  MulticallRequest,
  MulticallResponse,
  GasEstimate,
  GasStrategy,
  ContractMetrics,
  Address,
  ChainId,
} from './types';

export * from './types';

/**
 * Main WIA Smart Contract SDK class
 * Provides standardized interface for interacting with WIA-compliant smart contracts
 */
export class WIASmartContract {
  private contract: ethers.Contract;
  private provider: ethers.Provider;
  private signer?: ethers.Signer;
  private config: WIAContractConfig;
  private metadata?: WIAContractMetadata;
  private currentChain: ChainId;

  /**
   * Creates a new WIA Smart Contract instance
   * @param config - Configuration options
   */
  constructor(config: WIAContractConfig) {
    this.config = config;

    if (!config.provider && !config.signer) {
      throw new Error('Either provider or signer must be provided');
    }

    this.provider = config.provider || config.signer!.provider!;
    this.signer = config.signer;

    const address = this.getContractAddress();
    this.contract = new ethers.Contract(
      address,
      config.abi,
      this.signer || this.provider
    );

    this.currentChain = this.resolveChainId(config.chain || config.defaultChain);
  }

  // ============================================================================
  // Core Read Methods
  // ============================================================================

  /**
   * Get ERC-20 token name
   */
  async name(): Promise<string> {
    return await this.contract.name();
  }

  /**
   * Get ERC-20 token symbol
   */
  async symbol(): Promise<string> {
    return await this.contract.symbol();
  }

  /**
   * Get token decimals
   */
  async decimals(): Promise<number> {
    return await this.contract.decimals();
  }

  /**
   * Get total supply
   */
  async totalSupply(): Promise<BigNumber> {
    return await this.contract.totalSupply();
  }

  /**
   * Get balance of an address
   * @param address - Address to check
   */
  async balanceOf(address: Address): Promise<BigNumber> {
    return await this.contract.balanceOf(address);
  }

  /**
   * Get allowance for spender
   * @param owner - Token owner address
   * @param spender - Spender address
   */
  async allowance(owner: Address, spender: Address): Promise<BigNumber> {
    return await this.contract.allowance(owner, spender);
  }

  /**
   * Get WIA contract metadata
   */
  async getMetadata(): Promise<WIAContractMetadata> {
    if (this.metadata) {
      return this.metadata;
    }

    try {
      const metadata = await this.contract.getWIAMetadata();
      this.metadata = metadata;
      return metadata;
    } catch (error) {
      throw new Error('Contract does not implement WIA metadata');
    }
  }

  // ============================================================================
  // Core Write Methods
  // ============================================================================

  /**
   * Transfer tokens to recipient
   * @param to - Recipient address
   * @param amount - Amount to transfer
   * @param options - Transaction options
   */
  async transfer(
    to: Address,
    amount: BigNumber,
    options?: TxOptions
  ): Promise<TransactionResponse> {
    this.requireSigner();
    const txOptions = this.prepareTxOptions(options);

    try {
      const tx = await this.contract.transfer(to, amount, txOptions);
      return this.wrapTransaction(tx);
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Approve spender to spend tokens
   * @param spender - Spender address
   * @param amount - Amount to approve
   * @param options - Transaction options
   */
  async approve(
    spender: Address,
    amount: BigNumber,
    options?: TxOptions
  ): Promise<TransactionResponse> {
    this.requireSigner();
    const txOptions = this.prepareTxOptions(options);

    try {
      const tx = await this.contract.approve(spender, amount, txOptions);
      return this.wrapTransaction(tx);
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Transfer tokens from one address to another
   * @param from - Sender address
   * @param to - Recipient address
   * @param amount - Amount to transfer
   * @param options - Transaction options
   */
  async transferFrom(
    from: Address,
    to: Address,
    amount: BigNumber,
    options?: TxOptions
  ): Promise<TransactionResponse> {
    this.requireSigner();
    const txOptions = this.prepareTxOptions(options);

    try {
      const tx = await this.contract.transferFrom(from, to, amount, txOptions);
      return this.wrapTransaction(tx);
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ============================================================================
  // Event Handling
  // ============================================================================

  /**
   * Listen to contract events
   * @param event - Event name
   * @param listener - Event listener callback
   */
  on(event: string, listener: EventListener): void {
    this.contract.on(event, listener);
  }

  /**
   * Listen to event once
   * @param event - Event name
   * @param listener - Event listener callback
   */
  once(event: string, listener: EventListener): void {
    this.contract.once(event, listener);
  }

  /**
   * Remove event listener
   * @param event - Event name
   * @param listener - Event listener callback
   */
  off(event: string, listener: EventListener): void {
    this.contract.off(event, listener);
  }

  /**
   * Remove all event listeners
   * @param event - Event name (optional)
   */
  removeAllListeners(event?: string): void {
    this.contract.removeAllListeners(event);
  }

  /**
   * Query historical events
   * @param filter - Event filter
   * @param fromBlock - Starting block
   * @param toBlock - Ending block
   */
  async queryFilter(
    filter: EventFilter | string,
    fromBlock?: number,
    toBlock?: number
  ) {
    return await this.contract.queryFilter(filter, fromBlock, toBlock);
  }

  // ============================================================================
  // Gas Management
  // ============================================================================

  /**
   * Estimate gas for a method call
   * @param method - Method name
   * @param args - Method arguments
   */
  async estimateGas(method: string, args: any[]): Promise<BigNumber> {
    const gasEstimate = await this.contract.estimateGas[method](...args);
    return gasEstimate;
  }

  /**
   * Get current gas prices
   */
  async getGasPrice(): Promise<BigNumber> {
    return await this.provider.getGasPrice();
  }

  /**
   * Get EIP-1559 fee data
   */
  async getFeeData() {
    return await this.provider.getFeeData();
  }

  /**
   * Send transaction with gas optimization
   * @param method - Method name
   * @param args - Method arguments
   * @param strategy - Gas strategy
   */
  async sendOptimized(
    method: string,
    args: any[],
    strategy: GasStrategy = 'standard'
  ): Promise<TransactionResponse> {
    this.requireSigner();

    const feeData = await this.getFeeData();
    const gasLimit = await this.estimateGas(method, args);

    const multipliers: Record<GasStrategy, number> = {
      slow: 0.8,
      standard: 1.0,
      fast: 1.2,
      instant: 1.5,
    };

    const multiplier = multipliers[strategy];
    const maxPriorityFeePerGas = feeData.maxPriorityFeePerGas!.mul(Math.floor(multiplier * 100)).div(100);
    const maxFeePerGas = feeData.maxFeePerGas!.mul(Math.floor(multiplier * 100)).div(100);

    const tx = await this.contract[method](...args, {
      gasLimit: gasLimit.mul(120).div(100), // 20% buffer
      maxPriorityFeePerGas,
      maxFeePerGas,
    });

    return this.wrapTransaction(tx);
  }

  // ============================================================================
  // Multi-Call Support
  // ============================================================================

  /**
   * Execute multiple read calls in a single request
   * @param calls - Array of method calls
   */
  async multicall(calls: MulticallRequest[]): Promise<MulticallResponse[]> {
    const results = await Promise.allSettled(
      calls.map((call) => this.contract[call.method](...call.args))
    );

    return results.map((result, index) => {
      if (result.status === 'fulfilled') {
        return {
          success: true,
          returnData: result.value,
        };
      } else {
        return {
          success: false,
          returnData: null,
          error: result.reason,
        };
      }
    });
  }

  // ============================================================================
  // Multi-Chain Support
  // ============================================================================

  /**
   * Switch to a different chain
   * @param chain - Chain ID or name
   */
  async switchChain(chain: ChainId | string): Promise<void> {
    const chainId = this.resolveChainId(chain);

    if (!this.config.addresses) {
      throw new Error('Multi-chain addresses not configured');
    }

    const address = this.config.addresses[chainId];
    if (!address) {
      throw new Error(`Contract not deployed on chain ${chainId}`);
    }

    this.currentChain = chainId;
    this.contract = new ethers.Contract(
      address,
      this.config.abi,
      this.signer || this.provider
    );
  }

  /**
   * Get current chain ID
   */
  getCurrentChain(): ChainId {
    return this.currentChain;
  }

  /**
   * Execute method on specific chain
   * @param chain - Chain ID or name
   */
  onChain(chain: ChainId | string): WIASmartContract {
    const chainId = this.resolveChainId(chain);

    if (!this.config.addresses) {
      throw new Error('Multi-chain addresses not configured');
    }

    const address = this.config.addresses[chainId];
    if (!address) {
      throw new Error(`Contract not deployed on chain ${chainId}`);
    }

    return new WIASmartContract({
      ...this.config,
      address,
      chain: chainId,
    });
  }

  // ============================================================================
  // Transaction Management
  // ============================================================================

  /**
   * Wait for transaction confirmation
   * @param hash - Transaction hash
   * @param confirmations - Number of confirmations
   */
  async waitForTransaction(
    hash: string,
    confirmations: number = 1
  ): Promise<TransactionReceipt> {
    const receipt = await this.provider.waitForTransaction(hash, confirmations);
    return receipt as TransactionReceipt;
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Connect a signer to the contract
   * @param signer - Ethers signer
   */
  connect(signer: ethers.Signer): WIASmartContract {
    return new WIASmartContract({
      ...this.config,
      signer,
    });
  }

  /**
   * Get the underlying ethers contract
   */
  getEthersContract(): ethers.Contract {
    return this.contract;
  }

  /**
   * Get contract address for current chain
   */
  getAddress(): Address {
    return this.contract.address;
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private getContractAddress(): Address {
    if (this.config.address) {
      return this.config.address;
    }

    if (this.config.addresses) {
      const chainId = this.resolveChainId(this.config.chain || this.config.defaultChain);
      const address = this.config.addresses[chainId];
      if (!address) {
        throw new Error(`Contract not deployed on chain ${chainId}`);
      }
      return address;
    }

    throw new Error('No contract address provided');
  }

  private resolveChainId(chain?: ChainId | string): ChainId {
    if (typeof chain === 'number') {
      return chain;
    }

    const chainMap: Record<string, ChainId> = {
      ethereum: 1,
      polygon: 137,
      arbitrum: 42161,
      optimism: 10,
      bsc: 56,
      avalanche: 43114,
    };

    return chain ? chainMap[chain] || 1 : 1;
  }

  private prepareTxOptions(options?: TxOptions): any {
    if (!options) return {};

    return {
      gasLimit: options.gasLimit,
      maxPriorityFeePerGas: options.maxPriorityFeePerGas,
      maxFeePerGas: options.maxFeePerGas,
      nonce: options.nonce,
      value: options.value,
    };
  }

  private wrapTransaction(tx: any): TransactionResponse {
    return {
      ...tx,
      wait: async (confirmations?: number) => {
        return await tx.wait(confirmations);
      },
    };
  }

  private requireSigner(): void {
    if (!this.signer) {
      throw new Error('Signer required for write operations');
    }
  }

  private handleError(error: any): Error {
    // Parse and wrap contract errors
    if (error.code === 'UNPREDICTABLE_GAS_LIMIT') {
      return new Error('Transaction would fail. Simulation error: ' + error.message);
    }

    if (error.reason) {
      return new Error(error.reason);
    }

    return error;
  }
}

/**
 * Create a new WIA Smart Contract instance
 * @param config - Configuration options
 */
export function createWIAContract(config: WIAContractConfig): WIASmartContract {
  return new WIASmartContract(config);
}

// Default export
export default WIASmartContract;
