/**
 * WIA-FIN-022 Open Banking Standard - TypeScript Type Definitions
 * @module @wia/open-banking-sdk
 */

export interface OpenBankingConfig {
  clientId: string;
  clientSecret?: string;
  certificatePath?: string;
  privateKeyPath?: string;
  signingCertPath?: string;
  baseUrl: string;
  environment?: 'sandbox' | 'production';
  timeout?: number;
  version?: 'v1.0' | 'v1.1' | 'v1.2' | 'v2.0';
}

export interface Amount {
  Amount: string;
  Currency: string;
}

export interface Account {
  AccountId: string;
  Currency: string;
  AccountType: 'Personal' | 'Business';
  AccountSubType: string;
  Nickname?: string;
  Account?: AccountIdentification[];
}

export interface AccountIdentification {
  SchemeName: string;
  Identification: string;
  Name?: string;
  SecondaryIdentification?: string;
}

export interface Balance {
  AccountId: string;
  Amount: Amount;
  CreditDebitIndicator: 'Credit' | 'Debit';
  Type: 'InterimAvailable' | 'InterimBooked' | 'Expected' | 'OpeningAvailable';
  DateTime: string;
}

export interface Transaction {
  TransactionId: string;
  BookingDateTime: string;
  ValueDateTime?: string;
  Amount: Amount;
  CreditDebitIndicator: 'Credit' | 'Debit';
  Status: 'Booked' | 'Pending';
  MerchantDetails?: MerchantDetails;
  ProprietaryBankTransactionCode?: ProprietaryBankTransactionCode;
  TransactionInformation?: string;
}

export interface MerchantDetails {
  MerchantName?: string;
  MerchantCategoryCode?: string;
}

export interface ProprietaryBankTransactionCode {
  Code: string;
  Issuer?: string;
}

export interface AccountAccessConsent {
  ConsentId: string;
  Status: ConsentStatus;
  StatusUpdateDateTime?: string;
  CreationDateTime: string;
  Permissions: Permission[];
  ExpirationDateTime?: string;
  TransactionFromDateTime?: string;
  TransactionToDateTime?: string;
}

export type ConsentStatus =
  | 'AwaitingAuthorisation'
  | 'Authorised'
  | 'Rejected'
  | 'Revoked'
  | 'Expired';

export type Permission =
  | 'ReadAccountsBasic'
  | 'ReadAccountsDetail'
  | 'ReadBalances'
  | 'ReadTransactionsBasic'
  | 'ReadTransactionsDetail'
  | 'ReadTransactionsCredits'
  | 'ReadTransactionsDebits';

export interface PaymentInitiation {
  InstructionIdentification: string;
  EndToEndIdentification: string;
  InstructedAmount: Amount;
  CreditorAccount: AccountIdentification;
  CreditorAgent?: CreditorAgent;
  RemittanceInformation?: RemittanceInformation;
}

export interface CreditorAgent {
  SchemeName: string;
  Identification: string;
}

export interface RemittanceInformation {
  Unstructured?: string;
  Reference?: string;
}

export interface DomesticPaymentConsent {
  ConsentId: string;
  Status: ConsentStatus;
  CreationDateTime: string;
  StatusUpdateDateTime?: string;
  Initiation: PaymentInitiation;
  Authorisation?: PaymentAuthorisation;
}

export interface PaymentAuthorisation {
  AuthorisationType: 'Single' | 'Any';
  CompletionDateTime?: string;
}

export interface DomesticPayment {
  DomesticPaymentId: string;
  ConsentId: string;
  Status: PaymentStatus;
  CreationDateTime: string;
  StatusUpdateDateTime?: string;
  Initiation: PaymentInitiation;
}

export type PaymentStatus =
  | 'Pending'
  | 'AcceptedSettlementInProcess'
  | 'AcceptedSettlementCompleted'
  | 'Rejected';

export interface FundsConfirmation {
  FundsConfirmationId: string;
  ConsentId?: string;
  FundsAvailable: boolean;
  InstructedAmount: Amount;
  DebtorAccount: AccountIdentification;
}

export interface VRPConsent {
  ConsentId: string;
  Status: ConsentStatus;
  CreationDateTime: string;
  StatusUpdateDateTime?: string;
  ControlParameters: VRPControlParameters;
  Initiation: VRPInitiation;
}

export interface VRPControlParameters {
  ValidFromDateTime: string;
  ValidToDateTime: string;
  MaximumIndividualAmount: Amount;
  PeriodicLimits?: PeriodicLimit[];
}

export interface PeriodicLimit {
  PeriodType: 'Day' | 'Week' | 'Fortnight' | 'Month' | 'Half-year' | 'Year';
  Amount: Amount;
}

export interface VRPInitiation {
  CreditorAccount: AccountIdentification;
  CreditorAgent?: CreditorAgent;
}

export interface VRPPayment {
  DomesticVRPId: string;
  ConsentId: string;
  Status: PaymentStatus;
  CreationDateTime: string;
  StatusUpdateDateTime?: string;
  Instruction: VRPInstruction;
}

export interface VRPInstruction {
  InstructionIdentification: string;
  InstructedAmount: Amount;
  CreditorAccount?: AccountIdentification;
  RemittanceInformation?: RemittanceInformation;
}

export interface OAuthTokens {
  access_token: string;
  token_type: 'Bearer';
  expires_in: number;
  refresh_token?: string;
  scope: string;
}

export interface APIError {
  ErrorCode: string;
  Message: string;
  Path?: string;
  Url?: string;
}

export interface APIResponse<T> {
  Data: T;
  Links?: {
    Self: string;
    First?: string;
    Prev?: string;
    Next?: string;
    Last?: string;
  };
  Meta?: {
    TotalPages?: number;
    FirstAvailableDateTime?: string;
    LastAvailableDateTime?: string;
  };
}

export interface RequestOptions {
  headers?: Record<string, string>;
  timeout?: number;
  idempotencyKey?: string;
}

export interface WebhookEvent {
  event_type: string;
  timestamp: string;
  resource_id: string;
  data: Record<string, any>;
}

export type WebhookEventType =
  | 'payment.completed'
  | 'payment.failed'
  | 'consent.revoked'
  | 'account.transaction';
