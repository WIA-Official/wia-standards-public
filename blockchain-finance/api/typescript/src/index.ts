/**
 * WIA Blockchain Finance Standard - TypeScript SDK
 *
 * Main SDK for interacting with blockchain finance operations,
 * DeFi protocols, and cross-chain bridges following WIA standards.
 *
 * @module @wia/blockchain-finance
 * @license MIT
 */

import { ethers } from 'ethers';
import {
  BlockchainNetwork,
  BlockchainTransaction,
  TransactionStatus,
  ERC20Token,
  ERC20Transfer,
  ERC721Transfer,
  ERC1155Transfer,
  SwapOperation,
  StakeOperation,
  LendOperation,
  LiquidityPoolOperation,
  BridgeOperation,
  DeFiOperationType,
  WIACompliance,
  WIATransaction,
  ComplianceLevel,
  WalletInfo,
  SignatureRequest,
  SignatureResult,
  WIAError,
  WIAErrorCode,
} from './types';

// Export all types
export * from './types';

/**
 * Configuration for WIA Blockchain Finance SDK
 */
export interface WIABlockchainFinanceConfig {
  /** RPC endpoint URL */
  rpcUrl: string;
  /** Network */
  network: BlockchainNetwork;
  /** Private key (optional, for signing) */
  privateKey?: string;
  /** WIA API key (optional, for advanced features) */
  apiKey?: string;
  /** Enable debug logging */
  debug?: boolean;
}

/**
 * Main SDK class for WIA Blockchain Finance Standard
 */
export class WIABlockchainFinance {
  private provider: ethers.JsonRpcProvider;
  private signer?: ethers.Wallet;
  private config: WIABlockchainFinanceConfig;

  /**
   * Create a new WIABlockchainFinance instance
   */
  constructor(config: WIABlockchainFinanceConfig) {
    this.config = config;
    this.provider = new ethers.JsonRpcProvider(config.rpcUrl);

    if (config.privateKey) {
      this.signer = new ethers.Wallet(config.privateKey, this.provider);
    }

    this.log('WIA Blockchain Finance SDK initialized', {
      network: config.network,
      hasSigner: !!this.signer,
    });
  }

  // ============================================================================
  // Transaction Management
  // ============================================================================

  /**
   * Create a basic blockchain transaction
   */
  async createTransaction(params: {
    to: string;
    value: string;
    data?: string;
    gasLimit?: number;
  }): Promise<BlockchainTransaction> {
    this.log('Creating transaction', params);

    if (!this.signer) {
      throw new WIAError(
        WIAErrorCode.INVALID_SIGNATURE,
        'Signer not configured. Provide privateKey in config.'
      );
    }

    try {
      const nonce = await this.provider.getTransactionCount(
        this.signer.address
      );
      const feeData = await this.provider.getFeeData();

      const transaction: BlockchainTransaction = {
        hash: '',
        from: this.signer.address,
        to: params.to,
        value: params.value,
        gasPrice: feeData.gasPrice?.toString(),
        gasLimit: params.gasLimit || 21000,
        nonce,
        data: params.data,
        chainId: (await this.provider.getNetwork()).chainId as unknown as number,
        status: TransactionStatus.PENDING,
        timestamp: Date.now(),
      };

      return transaction;
    } catch (error) {
      throw new WIAError(
        WIAErrorCode.TRANSACTION_FAILED,
        'Failed to create transaction',
        error
      );
    }
  }

  /**
   * Validate a transaction before sending
   */
  async validateTransaction(
    tx: BlockchainTransaction
  ): Promise<{ valid: boolean; errors: string[] }> {
    const errors: string[] = [];

    // Validate addresses
    if (!ethers.isAddress(tx.from)) {
      errors.push('Invalid sender address');
    }
    if (!ethers.isAddress(tx.to)) {
      errors.push('Invalid recipient address');
    }

    // Validate balance
    if (this.signer) {
      const balance = await this.provider.getBalance(tx.from);
      const totalCost = BigInt(tx.value) + BigInt(tx.gasLimit! * (tx.gasPrice ? BigInt(tx.gasPrice) : 0n));
      if (balance < totalCost) {
        errors.push('Insufficient balance');
      }
    }

    // Validate nonce
    const expectedNonce = await this.provider.getTransactionCount(tx.from);
    if (tx.nonce < expectedNonce) {
      errors.push('Nonce too low');
    }

    return {
      valid: errors.length === 0,
      errors,
    };
  }

  /**
   * Sign and send a transaction
   */
  async sendTransaction(tx: BlockchainTransaction): Promise<string> {
    this.log('Sending transaction', tx);

    if (!this.signer) {
      throw new WIAError(
        WIAErrorCode.INVALID_SIGNATURE,
        'Signer not configured'
      );
    }

    // Validate before sending
    const validation = await this.validateTransaction(tx);
    if (!validation.valid) {
      throw new WIAError(
        WIAErrorCode.TRANSACTION_FAILED,
        'Transaction validation failed',
        validation.errors
      );
    }

    try {
      const txRequest = {
        to: tx.to,
        value: tx.value,
        data: tx.data,
        gasLimit: tx.gasLimit,
        gasPrice: tx.gasPrice,
        nonce: tx.nonce,
      };

      const sentTx = await this.signer.sendTransaction(txRequest);
      this.log('Transaction sent', { hash: sentTx.hash });

      return sentTx.hash;
    } catch (error) {
      throw new WIAError(
        WIAErrorCode.TRANSACTION_FAILED,
        'Failed to send transaction',
        error
      );
    }
  }

  /**
   * Wait for transaction confirmation
   */
  async waitForTransaction(
    txHash: string,
    confirmations: number = 1
  ): Promise<BlockchainTransaction> {
    this.log('Waiting for transaction', { txHash, confirmations });

    try {
      const receipt = await this.provider.waitForTransaction(
        txHash,
        confirmations
      );

      if (!receipt) {
        throw new Error('Transaction receipt not found');
      }

      const transaction: BlockchainTransaction = {
        hash: receipt.hash,
        from: receipt.from,
        to: receipt.to || '',
        value: '0',
        gasPrice: receipt.gasPrice?.toString(),
        gasLimit: Number(receipt.gasLimit),
        nonce: receipt.nonce || 0,
        chainId: (await this.provider.getNetwork()).chainId as unknown as number,
        blockNumber: receipt.blockNumber,
        status:
          receipt.status === 1
            ? TransactionStatus.CONFIRMED
            : TransactionStatus.FAILED,
        timestamp: Date.now(),
      };

      return transaction;
    } catch (error) {
      throw new WIAError(
        WIAErrorCode.TRANSACTION_FAILED,
        'Failed to get transaction receipt',
        error
      );
    }
  }

  // ============================================================================
  // ERC-20 Token Operations
  // ============================================================================

  /**
   * Get ERC-20 token information
   */
  async getERC20Token(tokenAddress: string): Promise<ERC20Token> {
    this.log('Getting ERC-20 token info', { tokenAddress });

    if (!ethers.isAddress(tokenAddress)) {
      throw new WIAError(WIAErrorCode.INVALID_ADDRESS, 'Invalid token address');
    }

    try {
      const tokenContract = new ethers.Contract(
        tokenAddress,
        [
          'function name() view returns (string)',
          'function symbol() view returns (string)',
          'function decimals() view returns (uint8)',
          'function totalSupply() view returns (uint256)',
        ],
        this.provider
      );

      const [name, symbol, decimals, totalSupply] = await Promise.all([
        tokenContract.name(),
        tokenContract.symbol(),
        tokenContract.decimals(),
        tokenContract.totalSupply(),
      ]);

      return {
        standard: 'ERC20' as any,
        address: tokenAddress,
        name,
        symbol,
        decimals: Number(decimals),
        totalSupply: totalSupply.toString(),
        network: this.config.network,
      };
    } catch (error) {
      throw new WIAError(
        WIAErrorCode.CONTRACT_ERROR,
        'Failed to get token info',
        error
      );
    }
  }

  /**
   * Transfer ERC-20 tokens
   */
  async transferERC20(transfer: ERC20Transfer): Promise<string> {
    this.log('Transferring ERC-20 tokens', transfer);

    if (!this.signer) {
      throw new WIAError(
        WIAErrorCode.INVALID_SIGNATURE,
        'Signer not configured'
      );
    }

    try {
      const tokenContract = new ethers.Contract(
        transfer.tokenAddress,
        ['function transfer(address to, uint256 amount) returns (bool)'],
        this.signer
      );

      const tx = await tokenContract.transfer(transfer.to, transfer.amount);
      this.log('ERC-20 transfer sent', { hash: tx.hash });

      return tx.hash;
    } catch (error) {
      throw new WIAError(
        WIAErrorCode.TRANSACTION_FAILED,
        'Failed to transfer tokens',
        error
      );
    }
  }

  /**
   * Get ERC-20 token balance
   */
  async getERC20Balance(
    tokenAddress: string,
    ownerAddress: string
  ): Promise<string> {
    try {
      const tokenContract = new ethers.Contract(
        tokenAddress,
        ['function balanceOf(address) view returns (uint256)'],
        this.provider
      );

      const balance = await tokenContract.balanceOf(ownerAddress);
      return balance.toString();
    } catch (error) {
      throw new WIAError(
        WIAErrorCode.CONTRACT_ERROR,
        'Failed to get balance',
        error
      );
    }
  }

  // ============================================================================
  // DeFi Integration Helpers
  // ============================================================================

  /**
   * Execute a token swap on a DEX
   */
  async executeSwap(swap: SwapOperation): Promise<string> {
    this.log('Executing swap', swap);

    if (!this.signer) {
      throw new WIAError(
        WIAErrorCode.INVALID_SIGNATURE,
        'Signer not configured'
      );
    }

    // This is a simplified example - actual implementation would depend on the DEX
    try {
      // Example: Uniswap V2 Router interface
      const routerABI = [
        'function swapExactTokensForTokens(uint amountIn, uint amountOutMin, address[] path, address to, uint deadline) returns (uint[] amounts)',
      ];

      // Note: Router address would be determined by the protocol
      const routerAddress = this.getRouterAddress(swap.protocol);
      const router = new ethers.Contract(routerAddress, routerABI, this.signer);

      const tx = await router.swapExactTokensForTokens(
        swap.amountIn,
        swap.amountOutMin,
        swap.path,
        swap.recipient,
        swap.deadline
      );

      this.log('Swap executed', { hash: tx.hash });
      return tx.hash;
    } catch (error) {
      throw new WIAError(
        WIAErrorCode.TRANSACTION_FAILED,
        'Failed to execute swap',
        error
      );
    }
  }

  /**
   * Stake tokens
   */
  async stake(stakeOp: StakeOperation): Promise<string> {
    this.log('Staking tokens', stakeOp);

    if (!this.signer) {
      throw new WIAError(
        WIAErrorCode.INVALID_SIGNATURE,
        'Signer not configured'
      );
    }

    try {
      // Simplified staking interface
      const stakingABI = ['function stake(uint256 amount) returns (bool)'];

      const stakingAddress = this.getStakingAddress(stakeOp.protocol);
      const staking = new ethers.Contract(
        stakingAddress,
        stakingABI,
        this.signer
      );

      const tx = await staking.stake(stakeOp.amount);
      this.log('Staking transaction sent', { hash: tx.hash });

      return tx.hash;
    } catch (error) {
      throw new WIAError(
        WIAErrorCode.TRANSACTION_FAILED,
        'Failed to stake',
        error
      );
    }
  }

  /**
   * Add liquidity to a pool
   */
  async addLiquidity(liquidityOp: LiquidityPoolOperation): Promise<string> {
    this.log('Adding liquidity', liquidityOp);

    if (!this.signer) {
      throw new WIAError(
        WIAErrorCode.INVALID_SIGNATURE,
        'Signer not configured'
      );
    }

    try {
      const routerABI = [
        'function addLiquidity(address tokenA, address tokenB, uint amountA, uint amountB, uint minA, uint minB, address to, uint deadline) returns (uint, uint, uint)',
      ];

      const routerAddress = this.getRouterAddress(liquidityOp.protocol);
      const router = new ethers.Contract(routerAddress, routerABI, this.signer);

      const deadline = Math.floor(Date.now() / 1000) + 60 * 20; // 20 minutes

      const tx = await router.addLiquidity(
        liquidityOp.tokenA,
        liquidityOp.tokenB,
        liquidityOp.amountA,
        liquidityOp.amountB,
        liquidityOp.minAmounts?.tokenA || '0',
        liquidityOp.minAmounts?.tokenB || '0',
        this.signer.address,
        deadline
      );

      this.log('Liquidity added', { hash: tx.hash });
      return tx.hash;
    } catch (error) {
      throw new WIAError(
        WIAErrorCode.TRANSACTION_FAILED,
        'Failed to add liquidity',
        error
      );
    }
  }

  // ============================================================================
  // Cross-Chain Bridge Utilities
  // ============================================================================

  /**
   * Bridge tokens across chains
   */
  async bridgeTokens(bridge: BridgeOperation): Promise<string> {
    this.log('Bridging tokens', bridge);

    if (!this.signer) {
      throw new WIAError(
        WIAErrorCode.INVALID_SIGNATURE,
        'Signer not configured'
      );
    }

    try {
      // Simplified bridge interface
      const bridgeABI = [
        'function bridge(address token, uint256 amount, uint256 destChainId, address recipient) returns (bytes32)',
      ];

      const bridgeAddress = this.getBridgeAddress(bridge.protocol);
      const bridgeContract = new ethers.Contract(
        bridgeAddress,
        bridgeABI,
        this.signer
      );

      const destChainId = this.getChainId(bridge.destinationChain);

      const tx = await bridgeContract.bridge(
        bridge.token,
        bridge.amount,
        destChainId,
        bridge.recipient
      );

      this.log('Bridge transaction sent', { hash: tx.hash });
      return tx.hash;
    } catch (error) {
      throw new WIAError(
        WIAErrorCode.BRIDGE_ERROR,
        'Failed to bridge tokens',
        error
      );
    }
  }

  // ============================================================================
  // Wallet and Signature Operations
  // ============================================================================

  /**
   * Get wallet information
   */
  async getWalletInfo(address?: string): Promise<WalletInfo> {
    const walletAddress = address || this.signer?.address;

    if (!walletAddress) {
      throw new WIAError(
        WIAErrorCode.INVALID_ADDRESS,
        'No wallet address provided'
      );
    }

    try {
      const balance = await this.provider.getBalance(walletAddress);

      return {
        type: 'metamask' as any,
        address: walletAddress,
        network: this.config.network,
        balance: balance.toString(),
      };
    } catch (error) {
      throw new WIAError(
        WIAErrorCode.NETWORK_ERROR,
        'Failed to get wallet info',
        error
      );
    }
  }

  /**
   * Sign a message
   */
  async signMessage(request: SignatureRequest): Promise<SignatureResult> {
    this.log('Signing message', request);

    if (!this.signer) {
      throw new WIAError(
        WIAErrorCode.INVALID_SIGNATURE,
        'Signer not configured'
      );
    }

    try {
      const signature = await this.signer.signMessage(request.message);
      const sig = ethers.Signature.from(signature);

      return {
        signature,
        signer: this.signer.address,
        v: sig.v,
        r: sig.r,
        s: sig.s,
      };
    } catch (error) {
      throw new WIAError(
        WIAErrorCode.INVALID_SIGNATURE,
        'Failed to sign message',
        error
      );
    }
  }

  // ============================================================================
  // WIA Compliance
  // ============================================================================

  /**
   * Wrap transaction with WIA compliance metadata
   */
  wrapWithCompliance(
    tx: BlockchainTransaction,
    compliance: WIACompliance
  ): WIATransaction {
    return {
      ...tx,
      wiaCompliance: compliance,
    };
  }

  /**
   * Create basic WIA compliance metadata
   */
  createComplianceMetadata(level: ComplianceLevel = ComplianceLevel.BASIC): WIACompliance {
    return {
      level,
      version: '1.0.0',
      certificationDate: new Date().toISOString(),
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Get router address for a DEX protocol
   */
  private getRouterAddress(protocol: string): string {
    // This would be a mapping of protocol names to router addresses
    // For now, returning a placeholder
    const routers: Record<string, string> = {
      uniswap: '0x7a250d5630B4cF539739dF2C5dAcb4c659F2488D',
      pancakeswap: '0x10ED43C718714eb63d5aA57B78B54704E256024E',
      sushiswap: '0xd9e1cE17f2641f24aE83637ab66a2cca9C378B9F',
    };

    return routers[protocol.toLowerCase()] || ethers.ZeroAddress;
  }

  /**
   * Get staking address for a protocol
   */
  private getStakingAddress(protocol: string): string {
    // Placeholder - would be a real mapping
    return ethers.ZeroAddress;
  }

  /**
   * Get bridge address for a protocol
   */
  private getBridgeAddress(protocol: string): string {
    // Placeholder - would be a real mapping
    return ethers.ZeroAddress;
  }

  /**
   * Get chain ID for a network
   */
  private getChainId(network: BlockchainNetwork): number {
    const chainIds: Record<BlockchainNetwork, number> = {
      [BlockchainNetwork.ETHEREUM]: 1,
      [BlockchainNetwork.POLYGON]: 137,
      [BlockchainNetwork.BSC]: 56,
      [BlockchainNetwork.AVALANCHE]: 43114,
      [BlockchainNetwork.ARBITRUM]: 42161,
      [BlockchainNetwork.OPTIMISM]: 10,
      [BlockchainNetwork.SOLANA]: 0, // Solana doesn't use EVM chain IDs
      [BlockchainNetwork.POLKADOT]: 0,
      [BlockchainNetwork.COSMOS]: 0,
      [BlockchainNetwork.NEAR]: 0,
    };

    return chainIds[network] || 0;
  }

  /**
   * Debug logging
   */
  private log(message: string, data?: any): void {
    if (this.config.debug) {
      console.log(`[WIA Blockchain Finance] ${message}`, data || '');
    }
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Validate Ethereum address
 */
export function isValidAddress(address: string): boolean {
  return ethers.isAddress(address);
}

/**
 * Format token amount with decimals
 */
export function formatTokenAmount(amount: string, decimals: number): string {
  return ethers.formatUnits(amount, decimals);
}

/**
 * Parse token amount to smallest unit
 */
export function parseTokenAmount(amount: string, decimals: number): string {
  return ethers.parseUnits(amount, decimals).toString();
}

/**
 * Convert wei to ether
 */
export function weiToEther(wei: string): string {
  return ethers.formatEther(wei);
}

/**
 * Convert ether to wei
 */
export function etherToWei(ether: string): string {
  return ethers.parseEther(ether).toString();
}
