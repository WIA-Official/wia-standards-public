/**
 * WIA-GAME Validators
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * Validation functions for GAME operations
 */

import { Address, BigIntish, SUPPORTED_CHAINS } from './types';

/**
 * Validate Ethereum address format
 */
export function validateAddress(address: string): asserts address is Address {
  if (!address) {
    throw new Error('Address is required');
  }

  if (!/^0x[a-fA-F0-9]{40}$/.test(address)) {
    throw new Error(`Invalid address format: ${address}`);
  }

  // Check for zero address
  if (address === '0x0000000000000000000000000000000000000000') {
    throw new Error('Zero address is not allowed');
  }
}

/**
 * Validate multiple addresses
 */
export function validateAddresses(addresses: string[]): asserts addresses is Address[] {
  if (!Array.isArray(addresses)) {
    throw new Error('Addresses must be an array');
  }

  if (addresses.length === 0) {
    throw new Error('At least one address is required');
  }

  addresses.forEach((address, index) => {
    try {
      validateAddress(address);
    } catch (error) {
      throw new Error(`Invalid address at index ${index}: ${(error as Error).message}`);
    }
  });
}

/**
 * Validate amount (must be positive)
 */
export function validateAmount(amount: BigIntish): void {
  if (amount === ungamened || amount === null) {
    throw new Error('Amount is required');
  }

  let bigIntAmount: bigint;

  try {
    if (typeof amount === 'bigint') {
      bigIntAmount = amount;
    } else if (typeof amount === 'string') {
      bigIntAmount = BigInt(amount);
    } else if (typeof amount === 'number') {
      if (!Number.isInteger(amount)) {
        throw new Error('Amount must be an integer');
      }
      bigIntAmount = BigInt(amount);
    } else {
      throw new Error('Invalid amount type');
    }
  } catch (error) {
    throw new Error(`Invalid amount format: ${amount}`);
  }

  if (bigIntAmount <= 0n) {
    throw new Error('Amount must be greater than zero');
  }

  // Check for reasonable upper bound (to prevent overflow)
  const MAX_UINT256 = BigInt('115792089237316195423570985008687907853269984665640564039457584007913129639935');
  if (bigIntAmount > MAX_UINT256) {
    throw new Error('Amount exceeds maximum uint256 value');
  }
}

/**
 * Validate chain ID
 */
export function validateChainId(chainId: number): void {
  if (!Number.isInteger(chainId) || chainId <= 0) {
    throw new Error('Chain ID must be a positive integer');
  }

  const supportedChainIds = Object.values(SUPPORTED_CHAINS).map(chain => chain.chainId);
  if (!supportedChainIds.includes(chainId)) {
    throw new Error(
      `Unsupported chain ID: ${chainId}. Supported chains: ${supportedChainIds.join(', ')}`
    );
  }
}

/**
 * Validate slippage tolerance (0-100%)
 */
export function validateSlippage(slippage: number): void {
  if (typeof slippage !== 'number') {
    throw new Error('Slippage must be a number');
  }

  if (slippage < 0 || slippage > 100) {
    throw new Error('Slippage must be between 0 and 100');
  }

  if (slippage > 50) {
    console.warn('Warning: High slippage tolerance detected (>50%)');
  }
}

/**
 * Validate deadline (Unix timestamp)
 */
export function validateDeadline(deadline: number): void {
  if (!Number.isInteger(deadline) || deadline <= 0) {
    throw new Error('Deadline must be a positive integer (Unix timestamp)');
  }

  const now = Math.floor(Date.now() / 1000);
  if (deadline <= now) {
    throw new Error('Deadline must be in the future');
  }

  // Check if deadline is too far in the future (more than 1 year)
  const oneYearFromNow = now + 365 * 24 * 60 * 60;
  if (deadline > oneYearFromNow) {
    throw new Error('Deadline is too far in the future (max 1 year)');
  }
}

/**
 * Validate token decimals
 */
export function validateDecimals(decimals: number): void {
  if (!Number.isInteger(decimals)) {
    throw new Error('Decimals must be an integer');
  }

  if (decimals < 0 || decimals > 77) {
    throw new Error('Decimals must be between 0 and 77');
  }
}

/**
 * Validate percentage (0-100)
 */
export function validatePercentage(percentage: number, fieldName: string = 'percentage'): void {
  if (typeof percentage !== 'number') {
    throw new Error(`${fieldName} must be a number`);
  }

  if (percentage < 0 || percentage > 100) {
    throw new Error(`${fieldName} must be between 0 and 100`);
  }
}

/**
 * Validate APR/APY (can be negative in some cases)
 */
export function validateAPR(apr: number): void {
  if (typeof apr !== 'number' || !isFinite(apr)) {
    throw new Error('APR must be a finite number');
  }

  if (apr < -100 || apr > 1000000) {
    throw new Error('APR value seems unrealistic');
  }
}

/**
 * Validate pool ID format
 */
export function validatePoolId(poolId: string): void {
  if (!poolId || typeof poolId !== 'string') {
    throw new Error('Pool ID is required and must be a string');
  }

  if (poolId.length < 3 || poolId.length > 100) {
    throw new Error('Pool ID length must be between 3 and 100 characters');
  }

  // Pool ID should contain only alphanumeric, dash, and underscore
  if (!/^[a-zA-Z0-9\-_]+$/.test(poolId)) {
    throw new Error('Pool ID contains invalid characters');
  }
}

/**
 * Validate protocol ID format
 */
export function validateProtocolId(protocolId: string): void {
  if (!protocolId || typeof protocolId !== 'string') {
    throw new Error('Protocol ID is required and must be a string');
  }

  // Common protocol IDs: uniswap-v4, aave-v4, lido-v3, etc.
  if (!/^[a-z][a-z0-9\-]*$/.test(protocolId)) {
    throw new Error('Protocol ID must start with lowercase letter and contain only lowercase letters, numbers, and dashes');
  }
}

/**
 * Validate transaction hash
 */
export function validateTxHash(txHash: string): void {
  if (!txHash || typeof txHash !== 'string') {
    throw new Error('Transaction hash is required and must be a string');
  }

  if (!/^0x[a-fA-F0-9]{64}$/.test(txHash)) {
    throw new Error('Invalid transaction hash format');
  }
}

/**
 * Validate hex string
 */
export function validateHex(hex: string): void {
  if (!hex || typeof hex !== 'string') {
    throw new Error('Hex string is required');
  }

  if (!hex.startsWith('0x')) {
    throw new Error('Hex string must start with 0x');
  }

  if (!/^0x[a-fA-F0-9]*$/.test(hex)) {
    throw new Error('Invalid hex string format');
  }
}

/**
 * Validate signature
 */
export function validateSignature(signature: string): void {
  validateHex(signature);

  // Standard ECDSA signature length (65 bytes = 130 hex chars + 0x prefix)
  if (signature.length !== 132) {
    throw new Error('Signature must be 65 bytes (130 hex characters)');
  }
}

/**
 * Validate health factor (lending protocols)
 */
export function validateHealthFactor(healthFactor: number): void {
  if (typeof healthFactor !== 'number' || !isFinite(healthFactor)) {
    throw new Error('Health factor must be a finite number');
  }

  if (healthFactor < 0) {
    throw new Error('Health factor cannot be negative');
  }

  if (healthFactor < 1) {
    console.warn('Warning: Health factor below 1.0 - position may be liquidated');
  }
}

/**
 * Validate liquidation threshold
 */
export function validateLiquidationThreshold(threshold: number): void {
  validatePercentage(threshold, 'liquidation threshold');

  if (threshold === 0 || threshold === 100) {
    throw new Error('Liquidation threshold cannot be 0% or 100%');
  }
}

/**
 * Validate collateral factor
 */
export function validateCollateralFactor(factor: number): void {
  validatePercentage(factor, 'collateral factor');

  if (factor === 0) {
    throw new Error('Collateral factor cannot be 0%');
  }
}

/**
 * Validate tick (for Uniswap v3/v4)
 */
export function validateTick(tick: number): void {
  if (!Number.isInteger(tick)) {
    throw new Error('Tick must be an integer');
  }

  // Uniswap v3/v4 tick range: -887272 to 887272
  const MIN_TICK = -887272;
  const MAX_TICK = 887272;

  if (tick < MIN_TICK || tick > MAX_TICK) {
    throw new Error(`Tick must be between ${MIN_TICK} and ${MAX_TICK}`);
  }
}

/**
 * Validate tick range (lower < upper)
 */
export function validateTickRange(tickLower: number, tickUpper: number): void {
  validateTick(tickLower);
  validateTick(tickUpper);

  if (tickLower >= tickUpper) {
    throw new Error('Lower tick must be less than upper tick');
  }
}

/**
 * Validate fee tier (Uniswap v3/v4)
 */
export function validateFeeTier(feeTier: number): void {
  if (!Number.isInteger(feeTier) || feeTier < 0) {
    throw new Error('Fee tier must be a non-negative integer');
  }

  // Common fee tiers: 100 (0.01%), 500 (0.05%), 3000 (0.3%), 10000 (1%)
  const commonFeeTiers = [100, 500, 3000, 10000];
  if (!commonFeeTiers.includes(feeTier)) {
    console.warn(`Warning: Uncommon fee tier ${feeTier}. Common tiers: ${commonFeeTiers.join(', ')}`);
  }
}

/**
 * Validate interest rate mode
 */
export function validateInterestRateMode(mode: string): asserts mode is 'stable' | 'variable' {
  if (mode !== 'stable' && mode !== 'variable') {
    throw new Error('Interest rate mode must be either "stable" or "variable"');
  }
}

/**
 * Validate API key format
 */
export function validateApiKey(apiKey: string): void {
  if (!apiKey || typeof apiKey !== 'string') {
    throw new Error('API key is required and must be a string');
  }

  if (apiKey.length < 16) {
    throw new Error('API key is too short (minimum 16 characters)');
  }

  if (apiKey.length > 256) {
    throw new Error('API key is too long (maximum 256 characters)');
  }
}

/**
 * Validate network name
 */
export function validateNetwork(network: string): asserts network is keyof typeof SUPPORTED_CHAINS {
  const supportedNetworks = Object.keys(SUPPORTED_CHAINS);
  if (!supportedNetworks.includes(network)) {
    throw new Error(
      `Unsupported network: ${network}. Supported networks: ${supportedNetworks.join(', ')}`
    );
  }
}

/**
 * Sanitize user input to prevent injection attacks
 */
export function sanitizeInput(input: string): string {
  if (typeof input !== 'string') {
    throw new Error('Input must be a string');
  }

  // Remove any non-printable characters
  return input.replace(/[^\x20-\x7E]/g, '').trim();
}

/**
 * Validate query limit and offset
 */
export function validatePagination(limit?: number, offset?: number): void {
  if (limit !== ungamened) {
    if (!Number.isInteger(limit) || limit <= 0) {
      throw new Error('Limit must be a positive integer');
    }
    if (limit > 1000) {
      throw new Error('Limit cannot exceed 1000');
    }
  }

  if (offset !== ungamened) {
    if (!Number.isInteger(offset) || offset < 0) {
      throw new Error('Offset must be a non-negative integer');
    }
  }
}
