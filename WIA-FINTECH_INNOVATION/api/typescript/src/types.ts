// WIA-FINTECH_INNOVATION TypeScript Types
export enum PaymentMethod { CARD = 'card', BANK_TRANSFER = 'bank_transfer', BNPL = 'bnpl', WALLET = 'wallet' }
export enum TransactionStatus { PENDING = 'pending', COMPLETED = 'completed', FAILED = 'failed', REFUNDED = 'refunded' }

export interface BankAccount {
  id: string;
  account_number: string;
  routing_number: string;
  account_type: 'checking' | 'savings';
  balance: number;
  currency: string;
  institution_name: string;
}

export interface Payment {
  id: string;
  amount: number;
  currency: string;
  method: PaymentMethod;
  status: TransactionStatus;
  timestamp: string;
  metadata?: Record<string, any>;
}

export interface BNPLPlan {
  id: string;
  total_amount: number;
  installments: number;
  installment_amount: number;
  frequency: 'weekly' | 'biweekly' | 'monthly';
  first_payment_date: string;
}

export interface OpenBankingConsent {
  id: string;
  customer_id: string;
  institution_id: string;
  scopes: string[];
  granted_at: string;
  expires_at: string;
  status: 'active' | 'expired' | 'revoked';
}
