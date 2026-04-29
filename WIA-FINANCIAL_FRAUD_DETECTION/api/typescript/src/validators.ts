/**
 * WIA-FINANCIAL_FRAUD_DETECTION - Validation Functions
 *
 * @version 1.0.0
 * @standard WIA-FINANCIAL_FRAUD_DETECTION
 * @organization WIA (World Certification Industry Association)
 */

import {
  Transaction,
  FraudFeedback,
  FeedbackType,
  TransactionType,
  CardPayment,
  BankPayment,
  WalletPayment,
} from './types';

/**
 * Validate email format
 */
export function isValidEmail(email: string): boolean {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
}

/**
 * Validate phone number (E.164 format)
 */
export function isValidPhone(phone: string): boolean {
  const phoneRegex = /^\+[1-9]\d{1,14}$/;
  return phoneRegex.test(phone);
}

/**
 * Validate ISO 8601 timestamp
 */
export function isValidISO8601(timestamp: string): boolean {
  const iso8601Regex = /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d{3})?Z?$/;
  if (!iso8601Regex.test(timestamp)) {
    return false;
  }
  const date = new Date(timestamp);
  return !isNaN(date.getTime());
}

/**
 * Validate ISO 3166-1 alpha-2 country code
 */
export function isValidCountryCode(code: string): boolean {
  return /^[A-Z]{2}$/.test(code);
}

/**
 * Validate ISO 4217 currency code
 */
export function isValidCurrencyCode(code: string): boolean {
  return /^[A-Z]{3}$/.test(code);
}

/**
 * Validate card BIN (first 6 digits)
 */
export function isValidCardBin(bin: string): boolean {
  return /^\d{6}$/.test(bin);
}

/**
 * Validate card last 4 digits
 */
export function isValidCardLast4(last4: string): boolean {
  return /^\d{4}$/.test(last4);
}

/**
 * Validate Merchant Category Code (4 digits)
 */
export function isValidMCC(mcc: string): boolean {
  return /^\d{4}$/.test(mcc);
}

/**
 * Validate transaction amount
 */
function validateAmount(amount: number): string | null {
  if (typeof amount !== 'number') {
    return 'Amount must be a number';
  }
  if (amount < 0) {
    return 'Amount must be non-negative';
  }
  if (!isFinite(amount)) {
    return 'Amount must be finite';
  }
  return null;
}

/**
 * Validate customer information
 */
function validateCustomer(customer: Transaction['customer']): string | null {
  if (!customer.id) {
    return 'Customer ID is required';
  }
  if (!customer.email) {
    return 'Customer email is required';
  }
  if (!isValidEmail(customer.email)) {
    return 'Invalid customer email format';
  }
  if (customer.phone && !isValidPhone(customer.phone)) {
    return 'Invalid customer phone format (expected E.164)';
  }
  if (!customer.account_created_at) {
    return 'Customer account_created_at is required';
  }
  if (!isValidISO8601(customer.account_created_at)) {
    return 'Invalid customer account_created_at format (expected ISO 8601)';
  }
  if (typeof customer.account_age_days !== 'number' || customer.account_age_days < 0) {
    return 'Customer account_age_days must be a non-negative number';
  }
  return null;
}

/**
 * Validate merchant information
 */
function validateMerchant(merchant: Transaction['merchant']): string | null {
  if (!merchant.id) {
    return 'Merchant ID is required';
  }
  if (!merchant.name) {
    return 'Merchant name is required';
  }
  if (!merchant.mcc) {
    return 'Merchant MCC is required';
  }
  if (!isValidMCC(merchant.mcc)) {
    return 'Invalid merchant MCC format (expected 4 digits)';
  }
  if (!merchant.country) {
    return 'Merchant country is required';
  }
  if (!isValidCountryCode(merchant.country)) {
    return 'Invalid merchant country code (expected ISO 3166-1 alpha-2)';
  }
  return null;
}

/**
 * Validate card payment method
 */
function validateCardPayment(payment: CardPayment): string | null {
  if (!payment.card_bin) {
    return 'Card BIN is required';
  }
  if (!isValidCardBin(payment.card_bin)) {
    return 'Invalid card BIN format (expected 6 digits)';
  }
  if (!payment.card_last4) {
    return 'Card last4 is required';
  }
  if (!isValidCardLast4(payment.card_last4)) {
    return 'Invalid card last4 format (expected 4 digits)';
  }
  if (!payment.card_brand) {
    return 'Card brand is required';
  }
  if (!payment.card_funding) {
    return 'Card funding is required';
  }
  if (!payment.card_country) {
    return 'Card country is required';
  }
  if (!isValidCountryCode(payment.card_country)) {
    return 'Invalid card country code (expected ISO 3166-1 alpha-2)';
  }
  if (typeof payment.cvv_provided !== 'boolean') {
    return 'cvv_provided must be a boolean';
  }
  if (payment.expiry_month !== undefined) {
    if (payment.expiry_month < 1 || payment.expiry_month > 12) {
      return 'Expiry month must be between 1 and 12';
    }
  }
  if (payment.expiry_year !== undefined) {
    if (payment.expiry_year < 2020 || payment.expiry_year > 2099) {
      return 'Expiry year must be between 2020 and 2099';
    }
  }
  if (payment.three_d_secure) {
    const tds = payment.three_d_secure;
    if (!tds.version || (tds.version !== '1.0' && tds.version !== '2.0')) {
      return '3D Secure version must be "1.0" or "2.0"';
    }
    if (typeof tds.authenticated !== 'boolean') {
      return '3D Secure authenticated must be a boolean';
    }
    if (!tds.eci) {
      return '3D Secure ECI is required';
    }
  }
  return null;
}

/**
 * Validate bank payment method
 */
function validateBankPayment(payment: BankPayment): string | null {
  if (!payment.account_type) {
    return 'Bank account type is required';
  }
  if (payment.account_type !== 'checking' && payment.account_type !== 'savings') {
    return 'Bank account type must be "checking" or "savings"';
  }
  if (!payment.bank_country) {
    return 'Bank country is required';
  }
  if (!isValidCountryCode(payment.bank_country)) {
    return 'Invalid bank country code (expected ISO 3166-1 alpha-2)';
  }
  return null;
}

/**
 * Validate wallet payment method
 */
function validateWalletPayment(payment: WalletPayment): string | null {
  if (!payment.wallet_provider) {
    return 'Wallet provider is required';
  }
  if (!payment.wallet_id) {
    return 'Wallet ID is required';
  }
  if (payment.wallet_email && !isValidEmail(payment.wallet_email)) {
    return 'Invalid wallet email format';
  }
  return null;
}

/**
 * Validate payment method
 */
function validatePaymentMethod(paymentMethod: Transaction['payment_method']): string | null {
  if (!paymentMethod) {
    return 'Payment method is required';
  }
  if (!paymentMethod.type) {
    return 'Payment method type is required';
  }

  if (paymentMethod.type === 'card') {
    return validateCardPayment(paymentMethod as CardPayment);
  } else if (
    paymentMethod.type === 'bank_transfer' ||
    paymentMethod.type === 'ach' ||
    paymentMethod.type === 'wire'
  ) {
    return validateBankPayment(paymentMethod as BankPayment);
  } else if (paymentMethod.type === 'wallet') {
    return validateWalletPayment(paymentMethod as WalletPayment);
  } else {
    return 'Unknown payment method type';
  }
}

/**
 * Validate address
 */
function validateAddress(address: Transaction['shipping_address'], fieldName: string): string | null {
  if (!address) {
    return null; // Addresses are optional
  }
  if (!address.line1) {
    return `${fieldName} line1 is required`;
  }
  if (!address.city) {
    return `${fieldName} city is required`;
  }
  if (!address.postal_code) {
    return `${fieldName} postal_code is required`;
  }
  if (!address.country) {
    return `${fieldName} country is required`;
  }
  if (!isValidCountryCode(address.country)) {
    return `Invalid ${fieldName} country code (expected ISO 3166-1 alpha-2)`;
  }
  if (address.latitude !== undefined) {
    if (address.latitude < -90 || address.latitude > 90) {
      return `${fieldName} latitude must be between -90 and 90`;
    }
  }
  if (address.longitude !== undefined) {
    if (address.longitude < -180 || address.longitude > 180) {
      return `${fieldName} longitude must be between -180 and 180`;
    }
  }
  return null;
}

/**
 * Validate device information
 */
function validateDevice(device: Transaction['device']): string | null {
  if (!device) {
    return 'Device information is required';
  }
  if (!device.fingerprint) {
    return 'Device fingerprint is required';
  }
  if (!device.ip_address) {
    return 'Device IP address is required';
  }
  if (!device.user_agent) {
    return 'Device user agent is required';
  }
  if (!device.accept_language) {
    return 'Device accept_language is required';
  }
  if (typeof device.timezone_offset !== 'number') {
    return 'Device timezone_offset must be a number';
  }
  if (typeof device.is_vpn !== 'boolean') {
    return 'Device is_vpn must be a boolean';
  }
  if (typeof device.is_proxy !== 'boolean') {
    return 'Device is_proxy must be a boolean';
  }
  if (typeof device.is_tor !== 'boolean') {
    return 'Device is_tor must be a boolean';
  }

  // Validate IP geolocation if present
  if (device.ip_geolocation) {
    const geo = device.ip_geolocation;
    if (!geo.country || !isValidCountryCode(geo.country)) {
      return 'Invalid IP geolocation country code';
    }
    if (!geo.region) {
      return 'IP geolocation region is required';
    }
    if (!geo.city) {
      return 'IP geolocation city is required';
    }
    if (typeof geo.latitude !== 'number' || geo.latitude < -90 || geo.latitude > 90) {
      return 'IP geolocation latitude must be between -90 and 90';
    }
    if (typeof geo.longitude !== 'number' || geo.longitude < -180 || geo.longitude > 180) {
      return 'IP geolocation longitude must be between -180 and 180';
    }
    if (!geo.isp) {
      return 'IP geolocation ISP is required';
    }
  }

  return null;
}

/**
 * Validate session information
 */
function validateSession(session: Transaction['session']): string | null {
  if (!session) {
    return null; // Session is optional
  }
  if (!session.id) {
    return 'Session ID is required';
  }
  if (!session.started_at) {
    return 'Session started_at is required';
  }
  if (!isValidISO8601(session.started_at)) {
    return 'Invalid session started_at format (expected ISO 8601)';
  }
  if (typeof session.duration_seconds !== 'number' || session.duration_seconds < 0) {
    return 'Session duration_seconds must be a non-negative number';
  }
  if (typeof session.pages_viewed !== 'number' || session.pages_viewed < 0) {
    return 'Session pages_viewed must be a non-negative number';
  }
  return null;
}

/**
 * Validate transaction object
 *
 * @param transaction - Transaction to validate
 * @returns Error message if invalid, null if valid
 *
 * @example
 * ```typescript
 * const error = validateTransaction(transaction);
 * if (error) {
 *   console.error('Validation error:', error);
 * }
 * ```
 */
export function validateTransaction(transaction: Transaction): string | null {
  // Required fields
  if (!transaction.id) {
    return 'Transaction ID is required';
  }
  if (!transaction.merchant_id) {
    return 'Merchant ID is required';
  }
  if (!transaction.customer_id) {
    return 'Customer ID is required';
  }

  // Validate amount
  const amountError = validateAmount(transaction.amount);
  if (amountError) {
    return amountError;
  }

  // Validate currency
  if (!transaction.currency) {
    return 'Currency is required';
  }
  if (!isValidCurrencyCode(transaction.currency)) {
    return 'Invalid currency code (expected ISO 4217)';
  }

  // Validate timestamp
  if (!transaction.timestamp) {
    return 'Timestamp is required';
  }
  if (!isValidISO8601(transaction.timestamp)) {
    return 'Invalid timestamp format (expected ISO 8601)';
  }

  // Validate transaction type
  if (!transaction.type) {
    return 'Transaction type is required';
  }
  if (!Object.values(TransactionType).includes(transaction.type)) {
    return 'Invalid transaction type';
  }

  // Validate nested objects
  const customerError = validateCustomer(transaction.customer);
  if (customerError) {
    return customerError;
  }

  const merchantError = validateMerchant(transaction.merchant);
  if (merchantError) {
    return merchantError;
  }

  const paymentMethodError = validatePaymentMethod(transaction.payment_method);
  if (paymentMethodError) {
    return paymentMethodError;
  }

  const deviceError = validateDevice(transaction.device);
  if (deviceError) {
    return deviceError;
  }

  const shippingAddressError = validateAddress(transaction.shipping_address, 'Shipping address');
  if (shippingAddressError) {
    return shippingAddressError;
  }

  const billingAddressError = validateAddress(transaction.billing_address, 'Billing address');
  if (billingAddressError) {
    return billingAddressError;
  }

  const sessionError = validateSession(transaction.session);
  if (sessionError) {
    return sessionError;
  }

  return null;
}

/**
 * Validate fraud feedback
 *
 * @param feedback - Fraud feedback to validate
 * @returns Error message if invalid, null if valid
 *
 * @example
 * ```typescript
 * const error = validateFeedback(feedback);
 * if (error) {
 *   console.error('Validation error:', error);
 * }
 * ```
 */
export function validateFeedback(feedback: FraudFeedback): string | null {
  if (!feedback.transaction_id) {
    return 'Transaction ID is required';
  }

  if (!feedback.feedback_type) {
    return 'Feedback type is required';
  }

  if (!Object.values(FeedbackType).includes(feedback.feedback_type)) {
    return 'Invalid feedback type';
  }

  // If feedback_type is CONFIRMED_FRAUD, fraud_type is required
  if (feedback.feedback_type === FeedbackType.CONFIRMED_FRAUD && !feedback.fraud_type) {
    return 'Fraud type is required when feedback_type is CONFIRMED_FRAUD';
  }

  if (!feedback.submitted_by) {
    return 'submitted_by is required';
  }

  return null;
}

/**
 * Validate date range
 *
 * @param startDate - Start date (ISO 8601)
 * @param endDate - End date (ISO 8601)
 * @returns Error message if invalid, null if valid
 */
export function validateDateRange(startDate: string, endDate: string): string | null {
  if (!isValidISO8601(startDate)) {
    return 'Invalid start date format (expected ISO 8601)';
  }
  if (!isValidISO8601(endDate)) {
    return 'Invalid end date format (expected ISO 8601)';
  }

  const start = new Date(startDate);
  const end = new Date(endDate);

  if (start > end) {
    return 'Start date must be before end date';
  }

  // Maximum date range: 1 year
  const maxRange = 365 * 24 * 60 * 60 * 1000; // 1 year in milliseconds
  if (end.getTime() - start.getTime() > maxRange) {
    return 'Date range must be within 1 year';
  }

  return null;
}

// ============================================================================
// Exports
// ============================================================================

export default {
  validateTransaction,
  validateFeedback,
  validateDateRange,
  isValidEmail,
  isValidPhone,
  isValidISO8601,
  isValidCountryCode,
  isValidCurrencyCode,
  isValidCardBin,
  isValidCardLast4,
  isValidMCC,
};
