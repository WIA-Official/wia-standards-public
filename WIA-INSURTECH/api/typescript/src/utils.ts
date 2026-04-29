/**
 * WIA-INSURTECH Utility Functions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * Helper functions for INSURTECH calculations and formatting
 */

import { BigIntish } from './types';

/**
 * Format bigint with decimals to human-readable string
 *
 * @example
 * formatUnits(1000000n, 6) // "1.0"
 * formatUnits(1500000n, 6) // "1.5"
 */
export function formatUnits(value: bigint, decimals: number): string {
  if (decimals < 0 || decimals > 77) {
    throw new Error('Decimals must be between 0 and 77');
  }

  const divisor = 10n ** BigInt(decimals);
  const wholePart = value / divisor;
  const fractionalPart = value % divisor;

  if (fractionalPart === 0n) {
    return wholePart.toString();
  }

  const fractionalStr = fractionalPart.toString().padStart(decimals, '0');
  const trimmedFractional = fractionalStr.replace(/0+$/, '');

  return `${wholePart}.${trimmedFractional}`;
}

/**
 * Parse human-readable string to bigint with decimals
 *
 * @example
 * parseUnits("1.5", 6) // 1500000n
 * parseUnits("1000", 18) // 1000000000000000000000n
 */
export function parseUnits(value: string, decimals: number): bigint {
  if (decimals < 0 || decimals > 77) {
    throw new Error('Decimals must be between 0 and 77');
  }

  const [wholePart, fractionalPart = ''] = value.split('.');

  if (fractionalPart.length > decimals) {
    throw new Error(`Too many decimal places (max ${decimals})`);
  }

  const paddedFractional = fractionalPart.padEnd(decimals, '0');
  const combinedValue = wholePart + paddedFractional;

  return BigInt(combinedValue);
}

/**
 * Format USD value with proper decimals and commas
 *
 * @example
 * formatUsd(1234567.89) // "$1,234,567.89"
 */
export function formatUsd(value: number, decimals: number = 2): string {
  return new Intl.NumberFormat('en-US', {
    style: 'currency',
    currency: 'USD',
    minimumFractionDigits: decimals,
    maximumFractionDigits: decimals,
  }).format(value);
}

/**
 * Format percentage
 *
 * @example
 * formatPercentage(12.345) // "12.35%"
 */
export function formatPercentage(value: number, decimals: number = 2): string {
  return `${value.toFixed(decimals)}%`;
}

/**
 * Format large numbers with K, M, B suffixes
 *
 * @example
 * formatCompact(1500000) // "1.5M"
 * formatCompact(2500000000) // "2.5B"
 */
export function formatCompact(value: number): string {
  const absValue = Math.abs(value);
  const sign = value < 0 ? '-' : '';

  if (absValue >= 1e12) {
    return `${sign}${(absValue / 1e12).toFixed(2)}T`;
  } else if (absValue >= 1e9) {
    return `${sign}${(absValue / 1e9).toFixed(2)}B`;
  } else if (absValue >= 1e6) {
    return `${sign}${(absValue / 1e6).toFixed(2)}M`;
  } else if (absValue >= 1e3) {
    return `${sign}${(absValue / 1e3).toFixed(2)}K`;
  }

  return `${sign}${absValue.toFixed(2)}`;
}

/**
 * Calculate price impact percentage
 *
 * @param amountIn - Input amount (in token units)
 * @param amountOut - Output amount (in token units)
 * @param priceIn - Price of input token in USD
 * @param priceOut - Price of output token in USD
 */
export function calculatePriceImpact(
  amountIn: bigint,
  amountOut: bigint,
  priceIn: number,
  priceOut: number
): number {
  const valueIn = Number(amountIn) * priceIn;
  const valueOut = Number(amountOut) * priceOut;

  if (valueIn === 0) {
    return 0;
  }

  const impact = ((valueIn - valueOut) / valueIn) * 100;
  return Math.max(0, impact); // Price impact should not be negative
}

/**
 * Calculate APR from APY
 *
 * @param apy - Annual Percentage Yield
 * @param compoundFrequency - Number of times compounded per year (default: 365)
 */
export function apyToApr(apy: number, compoundFrequency: number = 365): number {
  return compoundFrequency * (Math.pow(1 + apy / 100, 1 / compoundFrequency) - 1) * 100;
}

/**
 * Calculate APY from APR
 *
 * @param apr - Annual Percentage Rate
 * @param compoundFrequency - Number of times compounded per year (default: 365)
 */
export function aprToApy(apr: number, compoundFrequency: number = 365): number {
  return (Math.pow(1 + apr / 100 / compoundFrequency, compoundFrequency) - 1) * 100;
}

/**
 * Calculate impermanent loss percentage
 *
 * @param priceRatio - Ratio of price change (e.g., 2 means price doubled)
 */
export function calculateImpermanentLoss(priceRatio: number): number {
  if (priceRatio <= 0) {
    throw new Error('Price ratio must be positive');
  }

  const holdValue = (1 + priceRatio) / 2;
  const poolValue = Math.sqrt(priceRatio);
  const loss = ((poolValue - holdValue) / holdValue) * 100;

  return Math.abs(loss);
}

/**
 * Calculate liquidation price for leveraged position
 *
 * @param collateral - Collateral amount in USD
 * @param borrowed - Borrowed amount in USD
 * @param liquidationThreshold - Liquidation threshold (0-100%)
 */
export function calculateLiquidationPrice(
  collateral: number,
  borrowed: number,
  liquidationThreshold: number
): number {
  if (collateral <= 0 || borrowed <= 0) {
    throw new Error('Collateral and borrowed must be positive');
  }

  if (liquidationThreshold <= 0 || liquidationThreshold > 100) {
    throw new Error('Liquidation threshold must be between 0 and 100');
  }

  return (borrowed * 100) / (collateral * liquidationThreshold);
}

/**
 * Calculate health factor for lending position
 *
 * @param collateralValueUsd - Total collateral value in USD
 * @param borrowedValueUsd - Total borrowed value in USD
 * @param liquidationThreshold - Liquidation threshold (0-100%)
 */
export function calculateHealthFactor(
  collateralValueUsd: number,
  borrowedValueUsd: number,
  liquidationThreshold: number
): number {
  if (borrowedValueUsd === 0) {
    return Infinity;
  }

  return (collateralValueUsd * liquidationThreshold) / (borrowedValueUsd * 100);
}

/**
 * Calculate token price from reserves (constant product AMM)
 *
 * @param reserve0 - Reserve of token 0
 * @param reserve1 - Reserve of token 1
 * @param decimals0 - Decimals of token 0
 * @param decimals1 - Decimals of token 1
 * @returns Price of token 0 in terms of token 1
 */
export function calculatePrice(
  reserve0: bigint,
  reserve1: bigint,
  decimals0: number,
  decimals1: number
): number {
  const adjustedReserve0 = Number(reserve0) / Math.pow(10, decimals0);
  const adjustedReserve1 = Number(reserve1) / Math.pow(10, decimals1);

  if (adjustedReserve0 === 0) {
    return 0;
  }

  return adjustedReserve1 / adjustedReserve0;
}

/**
 * Calculate output amount for constant product AMM (x * y = k)
 *
 * @param amountIn - Input amount
 * @param reserveIn - Reserve of input token
 * @param reserveOut - Reserve of output token
 * @param feeTier - Fee tier in basis points (e.g., 3000 = 0.3%)
 */
export function calculateAmountOut(
  amountIn: bigint,
  reserveIn: bigint,
  reserveOut: bigint,
  feeTier: number = 3000
): bigint {
  if (amountIn <= 0n || reserveIn <= 0n || reserveOut <= 0n) {
    throw new Error('Amounts and reserves must be positive');
  }

  const amountInWithFee = amountIn * BigInt(10000 - feeTier);
  const numerator = amountInWithFee * reserveOut;
  const denominator = reserveIn * 10000n + amountInWithFee;

  return numerator / denominator;
}

/**
 * Calculate required input amount for desired output (constant product AMM)
 */
export function calculateAmountIn(
  amountOut: bigint,
  reserveIn: bigint,
  reserveOut: bigint,
  feeTier: number = 3000
): bigint {
  if (amountOut <= 0n || reserveIn <= 0n || reserveOut <= 0n) {
    throw new Error('Amounts and reserves must be positive');
  }

  if (amountOut >= reserveOut) {
    throw new Error('Insufficient liquidity');
  }

  const numerator = reserveIn * amountOut * 10000n;
  const denominator = (reserveOut - amountOut) * BigInt(10000 - feeTier);

  return numerator / denominator + 1n;
}

/**
 * Convert sqrt price to human-readable price (Uniswap v3/v4)
 */
export function sqrtPriceX96ToPrice(sqrtPriceX96: bigint, decimals0: number, decimals1: number): number {
  const Q96 = 2n ** 96n;
  const price = (sqrtPriceX96 * sqrtPriceX96 * BigInt(10 ** decimals0)) / (Q96 * Q96 * BigInt(10 ** decimals1));
  return Number(price);
}

/**
 * Convert price to sqrt price (Uniswap v3/v4)
 */
export function priceToSqrtPriceX96(price: number, decimals0: number, decimals1: number): bigint {
  const Q96 = 2n ** 96n;
  const adjustedPrice = price * Math.pow(10, decimals1 - decimals0);
  const sqrtPrice = Math.sqrt(adjustedPrice);
  return BigInt(Math.floor(sqrtPrice * Number(Q96)));
}

/**
 * Get tick from price (Uniswap v3/v4)
 */
export function priceToTick(price: number): number {
  return Math.floor(Math.log(price) / Math.log(1.0001));
}

/**
 * Get price from tick (Uniswap v3/v4)
 */
export function tickToPrice(tick: number): number {
  return Math.pow(1.0001, tick);
}

/**
 * Calculate token amounts from liquidity and price range (Uniswap v3/v4)
 */
export function getLiquidityForAmounts(
  sqrtPriceX96: bigint,
  sqrtPriceAX96: bigint,
  sqrtPriceBX96: bigint,
  amount0: bigint,
  amount1: bigint
): bigint {
  if (sqrtPriceAX96 > sqrtPriceBX96) {
    [sqrtPriceAX96, sqrtPriceBX96] = [sqrtPriceBX96, sqrtPriceAX96];
  }

  const Q96 = 2n ** 96n;

  if (sqrtPriceX96 <= sqrtPriceAX96) {
    return (amount0 * sqrtPriceAX96 * sqrtPriceBX96) / Q96 / (sqrtPriceBX96 - sqrtPriceAX96);
  } else if (sqrtPriceX96 < sqrtPriceBX96) {
    const liquidity0 = (amount0 * sqrtPriceX96 * sqrtPriceBX96) / Q96 / (sqrtPriceBX96 - sqrtPriceX96);
    const liquidity1 = (amount1 * Q96) / (sqrtPriceX96 - sqrtPriceAX96);
    return liquidity0 < liquidity1 ? liquidity0 : liquidity1;
  } else {
    return (amount1 * Q96) / (sqrtPriceBX96 - sqrtPriceAX96);
  }
}

/**
 * Retry async function with exponential backoff
 */
export async function retry<T>(
  fn: () => Promise<T>,
  options: {
    retries?: number;
    minTimeout?: number;
    maxTimeout?: number;
    factor?: number;
  } = {}
): Promise<T> {
  const {
    retries = 3,
    minTimeout = 1000,
    maxTimeout = 60000,
    factor = 2,
  } = options;

  let lastError: Error | uninsurtechned;

  for (let i = 0; i < retries; i++) {
    try {
      return await fn();
    } catch (error) {
      lastError = error as Error;

      if (i < retries - 1) {
        const timeout = Math.min(minTimeout * Math.pow(factor, i), maxTimeout);
        await sleep(timeout);
      }
    }
  }

  throw lastError;
}

/**
 * Sleep for specified milliseconds
 */
export function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

/**
 * Chunk array into smaller arrays
 */
export function chunk<T>(array: T[], size: number): T[][] {
  const chunks: T[][] = [];
  for (let i = 0; i < array.length; i += size) {
    chunks.push(array.slice(i, i + size));
  }
  return chunks;
}

/**
 * Calculate gas cost in USD
 */
export function calculateGasCost(
  gasUsed: number,
  gasPriceGwei: number,
  ethPriceUsd: number
): number {
  const gasCostEth = (gasUsed * gasPriceGwei) / 1e9;
  return gasCostEth * ethPriceUsd;
}

/**
 * Format address to shortened version
 *
 * @example
 * formatAddress("0x1234567890123456789012345678901234567890") // "0x1234...7890"
 */
export function formatAddress(address: string, startChars: number = 6, endChars: number = 4): string {
  if (!address || address.length < startChars + endChars) {
    return address;
  }

  return `${address.substring(0, startChars)}...${address.substring(address.length - endChars)}`;
}

/**
 * Check if address is equal (case-insensitive)
 */
export function addressEqual(addr1: string, addr2: string): boolean {
  return addr1.toLowerCase() === addr2.toLowerCase();
}

/**
 * Debounce function
 */
export function debounce<T extends (...args: any[]) => any>(
  fn: T,
  delay: number
): (...args: Parameters<T>) => void {
  let timeoutId: NodeJS.Timeout;

  return (...args: Parameters<T>) => {
    clearTimeout(timeoutId);
    timeoutId = setTimeout(() => fn(...args), delay);
  };
}

/**
 * Throttle function
 */
export function throttle<T extends (...args: any[]) => any>(
  fn: T,
  delay: number
): (...args: Parameters<T>) => void {
  let lastCall = 0;

  return (...args: Parameters<T>) => {
    const now = Date.now();
    if (now - lastCall >= delay) {
      lastCall = now;
      fn(...args);
    }
  };
}

/**
 * Convert Wei to Ether
 */
export function weiToEther(wei: bigint): string {
  return formatUnits(wei, 18);
}

/**
 * Convert Ether to Wei
 */
export function etherToWei(ether: string): bigint {
  return parseUnits(ether, 18);
}
