/**
 * WIA-FINTECH_INNOVATION Validators
 * Input validation utilities for fintech operations
 */

export interface ValidationError {
  field: string;
  message: string;
  code: string;
}

/**
 * Validate payment amount
 */
export function validateAmount(amount: number, currency: string): ValidationError | null {
  if (typeof amount !== 'number' || isNaN(amount)) {
    return {
      field: 'amount',
      message: 'Amount must be a valid number',
      code: 'invalid_amount'
    };
  }

  if (amount <= 0) {
    return {
      field: 'amount',
      message: 'Amount must be greater than zero',
      code: 'amount_too_small'
    };
  }

  // Currency-specific validation
  const maxAmounts: Record<string, number> = {
    'USD': 1000000,
    'EUR': 1000000,
    'GBP': 1000000,
    'JPY': 100000000
  };

  const maxAmount = maxAmounts[currency] || 1000000;
  if (amount > maxAmount) {
    return {
      field: 'amount',
      message: `Amount exceeds maximum ${maxAmount} ${currency}`,
      code: 'amount_too_large'
    };
  }

  return null;
}

/**
 * Validate currency code (ISO 4217)
 */
export function validateCurrency(currency: string): ValidationError | null {
  const validCurrencies = [
    'USD', 'EUR', 'GBP', 'JPY', 'AUD', 'CAD', 'CHF', 'CNY', 'HKD', 'NZD',
    'SEK', 'KRW', 'SGD', 'NOK', 'MXN', 'INR', 'RUB', 'ZAR', 'TRY', 'BRL'
  ];

  if (!currency || typeof currency !== 'string') {
    return {
      field: 'currency',
      message: 'Currency is required',
      code: 'currency_required'
    };
  }

  if (currency.length !== 3) {
    return {
      field: 'currency',
      message: 'Currency must be 3-character ISO 4217 code',
      code: 'invalid_currency_format'
    };
  }

  if (!validCurrencies.includes(currency.toUpperCase())) {
    return {
      field: 'currency',
      message: `Currency ${currency} is not supported`,
      code: 'unsupported_currency'
    };
  }

  return null;
}

/**
 * Validate email address
 */
export function validateEmail(email: string): ValidationError | null {
  const emailRegex = /^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$/;

  if (!email || typeof email !== 'string') {
    return {
      field: 'email',
      message: 'Email is required',
      code: 'email_required'
    };
  }

  if (!emailRegex.test(email)) {
    return {
      field: 'email',
      message: 'Invalid email format',
      code: 'invalid_email'
    };
  }

  if (email.length > 254) {
    return {
      field: 'email',
      message: 'Email is too long (max 254 characters)',
      code: 'email_too_long'
    };
  }

  return null;
}

/**
 * Validate phone number (E.164 format)
 */
export function validatePhone(phone: string): ValidationError | null {
  const phoneRegex = /^\+[1-9]\d{1,14}$/;

  if (!phone || typeof phone !== 'string') {
    return {
      field: 'phone',
      message: 'Phone number is required',
      code: 'phone_required'
    };
  }

  if (!phoneRegex.test(phone)) {
    return {
      field: 'phone',
      message: 'Phone must be in E.164 format (e.g., +14155551234)',
      code: 'invalid_phone'
    };
  }

  return null;
}

/**
 * Validate IBAN (International Bank Account Number)
 */
export function validateIBAN(iban: string): ValidationError | null {
  if (!iban || typeof iban !== 'string') {
    return {
      field: 'iban',
      message: 'IBAN is required',
      code: 'iban_required'
    };
  }

  // Remove spaces and convert to uppercase
  const cleanIban = iban.replace(/\s/g, '').toUpperCase();

  // IBAN format: 2 letters (country) + 2 digits (check) + up to 30 alphanumeric
  const ibanRegex = /^[A-Z]{2}[0-9]{2}[A-Z0-9]+$/;
  if (!ibanRegex.test(cleanIban)) {
    return {
      field: 'iban',
      message: 'Invalid IBAN format',
      code: 'invalid_iban_format'
    };
  }

  if (cleanIban.length < 15 || cleanIban.length > 34) {
    return {
      field: 'iban',
      message: 'IBAN length must be between 15 and 34 characters',
      code: 'invalid_iban_length'
    };
  }

  // Mod-97 check
  const rearranged = cleanIban.slice(4) + cleanIban.slice(0, 4);
  const numericIban = rearranged.replace(/[A-Z]/g, (char) =>
    (char.charCodeAt(0) - 55).toString()
  );

  let remainder = '';
  for (const digit of numericIban) {
    remainder = (parseInt(remainder + digit, 10) % 97).toString();
  }

  if (parseInt(remainder, 10) !== 1) {
    return {
      field: 'iban',
      message: 'Invalid IBAN checksum',
      code: 'invalid_iban_checksum'
    };
  }

  return null;
}

/**
 * Validate US routing number (ABA routing number)
 */
export function validateRoutingNumber(routingNumber: string): ValidationError | null {
  if (!routingNumber || typeof routingNumber !== 'string') {
    return {
      field: 'routing_number',
      message: 'Routing number is required',
      code: 'routing_number_required'
    };
  }

  if (!/^\d{9}$/.test(routingNumber)) {
    return {
      field: 'routing_number',
      message: 'Routing number must be 9 digits',
      code: 'invalid_routing_number'
    };
  }

  // ABA routing number checksum algorithm
  const digits = routingNumber.split('').map(Number);
  const checksum =
    3 * (digits[0] + digits[3] + digits[6]) +
    7 * (digits[1] + digits[4] + digits[7]) +
    1 * (digits[2] + digits[5] + digits[8]);

  if (checksum % 10 !== 0) {
    return {
      field: 'routing_number',
      message: 'Invalid routing number checksum',
      code: 'invalid_routing_checksum'
    };
  }

  return null;
}

/**
 * Validate date string (ISO 8601)
 */
export function validateDate(date: string, fieldName: string = 'date'): ValidationError | null {
  if (!date || typeof date !== 'string') {
    return {
      field: fieldName,
      message: `${fieldName} is required`,
      code: `${fieldName}_required`
    };
  }

  const parsedDate = new Date(date);
  if (isNaN(parsedDate.getTime())) {
    return {
      field: fieldName,
      message: `Invalid ${fieldName} format (use ISO 8601)`,
      code: `invalid_${fieldName}`
    };
  }

  return null;
}

/**
 * Validate BNPL plan parameters
 */
export function validateBNPLPlan(
  amount: number,
  installments: number
): ValidationError | null {
  const amountError = validateAmount(amount, 'USD');
  if (amountError) return amountError;

  if (!Number.isInteger(installments)) {
    return {
      field: 'installments',
      message: 'Installments must be an integer',
      code: 'invalid_installments'
    };
  }

  const validInstallments = [3, 4, 6, 12, 24, 36];
  if (!validInstallments.includes(installments)) {
    return {
      field: 'installments',
      message: `Installments must be one of: ${validInstallments.join(', ')}`,
      code: 'unsupported_installments'
    };
  }

  const minAmount = 50;
  const maxAmount = 30000;

  if (amount < minAmount) {
    return {
      field: 'amount',
      message: `BNPL minimum amount is ${minAmount}`,
      code: 'amount_below_bnpl_minimum'
    };
  }

  if (amount > maxAmount) {
    return {
      field: 'amount',
      message: `BNPL maximum amount is ${maxAmount}`,
      code: 'amount_above_bnpl_maximum'
    };
  }

  return null;
}

/**
 * Validate all payment fields
 */
export function validatePayment(payment: {
  amount: number;
  currency: string;
  customer?: { email?: string; phone?: string };
}): ValidationError[] {
  const errors: ValidationError[] = [];

  const amountError = validateAmount(payment.amount, payment.currency);
  if (amountError) errors.push(amountError);

  const currencyError = validateCurrency(payment.currency);
  if (currencyError) errors.push(currencyError);

  if (payment.customer?.email) {
    const emailError = validateEmail(payment.customer.email);
    if (emailError) errors.push(emailError);
  }

  if (payment.customer?.phone) {
    const phoneError = validatePhone(payment.customer.phone);
    if (phoneError) errors.push(phoneError);
  }

  return errors;
}
