/**
 * WIA-FINTECH_INNOVATION Utilities
 * Helper functions for fintech operations
 */

/**
 * Format amount with currency symbol
 */
export function formatCurrency(amount: number, currency: string): string {
  const currencySymbols: Record<string, string> = {
    'USD': '$',
    'EUR': '€',
    'GBP': '£',
    'JPY': '¥',
    'AUD': 'A$',
    'CAD': 'C$',
    'CHF': 'CHF',
    'CNY': '¥',
    'INR': '₹',
    'KRW': '₩'
  };

  const symbol = currencySymbols[currency] || currency;
  const formattedAmount = new Intl.NumberFormat('en-US', {
    minimumFractionDigits: 2,
    maximumFractionDigits: 2
  }).format(amount);

  return `${symbol}${formattedAmount}`;
}

/**
 * Calculate BNPL installment amount
 */
export function calculateInstallmentAmount(
  totalAmount: number,
  installments: number,
  interestRate: number = 0
): {
  installmentAmount: number;
  totalWithInterest: number;
  interestAmount: number;
} {
  if (interestRate === 0) {
    // Simple division for 0% APR
    const baseAmount = totalAmount / installments;
    const installmentAmount = Math.ceil(baseAmount * 100) / 100;
    return {
      installmentAmount,
      totalWithInterest: installmentAmount * installments,
      interestAmount: 0
    };
  }

  // Calculate with compound interest
  const monthlyRate = interestRate / 12 / 100;
  const installmentAmount =
    (totalAmount * monthlyRate * Math.pow(1 + monthlyRate, installments)) /
    (Math.pow(1 + monthlyRate, installments) - 1);

  const roundedInstallment = Math.ceil(installmentAmount * 100) / 100;
  const totalWithInterest = roundedInstallment * installments;
  const interestAmount = totalWithInterest - totalAmount;

  return {
    installmentAmount: roundedInstallment,
    totalWithInterest,
    interestAmount
  };
}

/**
 * Generate payment schedule for BNPL
 */
export function generatePaymentSchedule(
  startDate: Date,
  installments: number,
  installmentAmount: number,
  frequency: 'weekly' | 'biweekly' | 'monthly' = 'monthly'
): Array<{ dueDate: Date; amount: number; installmentNumber: number }> {
  const schedule: Array<{ dueDate: Date; amount: number; installmentNumber: number }> = [];

  const frequencyDays: Record<typeof frequency, number> = {
    weekly: 7,
    biweekly: 14,
    monthly: 30
  };

  const dayIncrement = frequencyDays[frequency];

  for (let i = 0; i < installments; i++) {
    const dueDate = new Date(startDate);
    dueDate.setDate(dueDate.getDate() + dayIncrement * i);

    schedule.push({
      dueDate,
      amount: installmentAmount,
      installmentNumber: i + 1
    });
  }

  return schedule;
}

/**
 * Calculate payment processing fee
 */
export function calculateProcessingFee(
  amount: number,
  paymentMethod: 'card' | 'bank_account' | 'digital_wallet'
): {
  feeAmount: number;
  feePercentage: number;
  netAmount: number;
} {
  const feeRates: Record<typeof paymentMethod, number> = {
    card: 2.9,
    bank_account: 0.8,
    digital_wallet: 3.4
  };

  const feePercentage = feeRates[paymentMethod];
  const feeAmount = (amount * feePercentage) / 100 + 0.30; // 2.9% + $0.30
  const netAmount = amount - feeAmount;

  return {
    feeAmount: Math.round(feeAmount * 100) / 100,
    feePercentage,
    netAmount: Math.round(netAmount * 100) / 100
  };
}

/**
 * Retry function with exponential backoff
 */
export async function retryWithBackoff<T>(
  fn: () => Promise<T>,
  maxRetries: number = 3,
  initialDelay: number = 1000
): Promise<T> {
  let lastError: Error;

  for (let attempt = 0; attempt < maxRetries; attempt++) {
    try {
      return await fn();
    } catch (error) {
      lastError = error as Error;

      if (attempt < maxRetries - 1) {
        const delay = initialDelay * Math.pow(2, attempt);
        await new Promise((resolve) => setTimeout(resolve, delay));
      }
    }
  }

  throw lastError!;
}

/**
 * Mask sensitive data for logging
 */
export function maskSensitiveData(data: any): any {
  if (typeof data !== 'object' || data === null) {
    return data;
  }

  const masked = { ...data };
  const sensitiveFields = [
    'password',
    'pin',
    'cvv',
    'card_number',
    'account_number',
    'routing_number',
    'ssn',
    'tax_id'
  ];

  for (const key in masked) {
    if (sensitiveFields.some((field) => key.toLowerCase().includes(field))) {
      if (typeof masked[key] === 'string') {
        masked[key] = '***' + masked[key].slice(-4);
      }
    } else if (typeof masked[key] === 'object') {
      masked[key] = maskSensitiveData(masked[key]);
    }
  }

  return masked;
}

/**
 * Generate idempotency key
 */
export function generateIdempotencyKey(
  customerId: string,
  operation: string,
  timestamp?: Date
): string {
  const date = timestamp || new Date();
  const dateStr = date.toISOString().split('T')[0];
  const random = Math.random().toString(36).substring(2, 10);
  return `${operation}_${customerId}_${dateStr}_${random}`;
}

/**
 * Convert cents to dollars
 */
export function centsToDollars(cents: number): number {
  return Math.round(cents) / 100;
}

/**
 * Convert dollars to cents
 */
export function dollarsToCents(dollars: number): number {
  return Math.round(dollars * 100);
}

/**
 * Calculate APR from interest rate
 */
export function calculateAPR(
  interestRate: number,
  compoundingFrequency: number = 12
): number {
  const apr = (Math.pow(1 + interestRate / compoundingFrequency, compoundingFrequency) - 1) * 100;
  return Math.round(apr * 100) / 100;
}

/**
 * Validate and normalize IBAN
 */
export function normalizeIBAN(iban: string): string {
  return iban.replace(/\s/g, '').toUpperCase();
}

/**
 * Format phone number to E.164
 */
export function formatPhoneE164(phone: string, defaultCountryCode: string = '+1'): string {
  // Remove all non-digit characters
  const digits = phone.replace(/\D/g, '');

  // If doesn't start with country code, add default
  if (!phone.startsWith('+')) {
    return `${defaultCountryCode}${digits}`;
  }

  return `+${digits}`;
}

/**
 * Calculate business days between two dates
 */
export function businessDaysBetween(startDate: Date, endDate: Date): number {
  let count = 0;
  const current = new Date(startDate);

  while (current <= endDate) {
    const dayOfWeek = current.getDay();
    if (dayOfWeek !== 0 && dayOfWeek !== 6) {
      // Not Sunday (0) or Saturday (6)
      count++;
    }
    current.setDate(current.getDate() + 1);
  }

  return count;
}

/**
 * Generate payment reference
 */
export function generatePaymentReference(prefix: string = 'PAY'): string {
  const timestamp = Date.now().toString(36).toUpperCase();
  const random = Math.random().toString(36).substring(2, 8).toUpperCase();
  return `${prefix}-${timestamp}-${random}`;
}

/**
 * Deep clone object
 */
export function deepClone<T>(obj: T): T {
  return JSON.parse(JSON.stringify(obj));
}

/**
 * Sleep/delay utility
 */
export function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}
