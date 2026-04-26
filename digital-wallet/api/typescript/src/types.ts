/**
 * WIA-FIN-015 Digital Wallet Standard - TypeScript Type Definitions
 * @module @wia/digital-wallet-sdk
 */

/**
 * Wallet types
 */
export enum WalletType {
  HD = 'hd',
  SIMPLE = 'simple',
  MULTISIG = 'multisig',
  HARDWARE = 'hardware',
  SMART_CONTRACT = 'smart_contract'
}

/**
 * Currency types
 */
export enum CurrencyType {
  FIAT = 'fiat',
  CRYPTOCURRENCY = 'cryptocurrency',
  STABLECOIN = 'stablecoin',
  CBDC = 'cbdc'
}

/**
 * Transaction types
 */
export enum TransactionType {
  SEND = 'send',
  RECEIVE = 'receive',
  EXCHANGE = 'exchange',
  STAKE = 'stake',
  WITHDRAW = 'withdraw',
  DEPOSIT = 'deposit'
}

/**
 * Transaction status
 */
export enum TransactionStatus {
  PENDING = 'pending',
  PROCESSING = 'processing',
  CONFIRMED = 'confirmed',
  FAILED = 'failed',
  CANCELLED = 'cancelled',
  REFUNDED = 'refunded'
}

/**
 * Payment methods
 */
export enum PaymentMethod {
  NFC = 'nfc',
  QR_CODE = 'qr',
  P2P = 'p2p',
  BANK_TRANSFER = 'bank',
  CARD = 'card',
  INSTANT = 'instant'
}

/**
 * Authentication methods
 */
export enum AuthMethod {
  PASSWORD = 'password',
  PIN = 'pin',
  BIOMETRIC = 'biometric',
  TWO_FACTOR = '2fa',
  HARDWARE_TOKEN = 'hardware_token'
}

/**
 * Currency information
 */
export interface Currency {
  /** Currency code (USD, BTC, ETH, etc.) */
  code: string;
  /** Currency name */
  name: string;
  /** Currency symbol */
  symbol: string;
  /** Currency type */
  type: CurrencyType;
  /** Number of decimal places */
  decimals: number;
  /** Blockchain network (for crypto) */
  network?: string;
  /** Contract address (for tokens) */
  contractAddress?: string;
}

/**
 * Wallet address
 */
export interface WalletAddress {
  /** Address index in HD wallet */
  index?: number;
  /** Address string */
  address: string;
  /** Current balance */
  balance: string;
  /** Public key */
  publicKey?: string;
  /** Derivation path (for HD wallets) */
  derivationPath?: string;
}

/**
 * Linked account information
 */
export interface LinkedAccount {
  /** Account type */
  type: 'bank_account' | 'card' | 'external_wallet';
  /** Account ID */
  accountId: string;
  /** Routing number (for bank accounts) */
  routingNumber?: string;
  /** Last 4 digits of account number */
  accountNumberLast4?: string;
  /** Nickname */
  nickname?: string;
}

/**
 * Currency configuration in wallet
 */
export interface WalletCurrency {
  /** Currency code */
  code: string;
  /** Currency type */
  type: CurrencyType;
  /** Linked account for fiat */
  linkedAccount?: LinkedAccount;
  /** Addresses for cryptocurrency */
  addresses?: WalletAddress[];
  /** Network (mainnet, testnet, etc.) */
  network?: string;
}

/**
 * Security settings
 */
export interface SecuritySettings {
  /** Encryption algorithm */
  encryption: string;
  /** Biometric authentication enabled */
  biometricEnabled: boolean;
  /** Two-factor authentication enabled */
  twoFactorEnabled: boolean;
  /** Backup status */
  backupStatus: 'none' | 'pending' | 'completed';
  /** Last backup timestamp */
  lastBackup?: Date;
  /** Hardware wallet connected */
  hardwareWallet?: HardwareWallet;
}

/**
 * Hardware wallet information
 */
export interface HardwareWallet {
  /** Device type */
  type: 'ledger_nano_s' | 'ledger_nano_x' | 'trezor_one' | 'trezor_t';
  /** Firmware version */
  firmwareVersion: string;
  /** Device serial */
  serial: string;
  /** Connection status */
  connected: boolean;
}

/**
 * Multi-signature signer
 */
export interface MultiSigSigner {
  /** Signer ID */
  id: string;
  /** Public key */
  publicKey: string;
  /** Signer role */
  role: 'owner' | 'co-owner' | 'guardian' | 'operator';
  /** Signature weight */
  weight: number;
}

/**
 * Wallet configuration
 */
export interface WalletConfig {
  /** Wallet type */
  type: WalletType;
  /** Supported currencies */
  currencies: string[];
  /** Security settings */
  security?: Partial<SecuritySettings>;
  /** Test mode flag */
  testMode?: boolean;
  /** Custom RPC endpoints */
  rpcEndpoints?: Record<string, string>;
  /** Multi-sig configuration */
  multiSig?: {
    requiredSignatures: number;
    totalSigners: number;
    signers: MultiSigSigner[];
  };
}

/**
 * Complete wallet information
 */
export interface Wallet {
  /** Unique wallet ID */
  walletId: string;
  /** Wallet type */
  type: WalletType;
  /** Version */
  version: string;
  /** Creation timestamp */
  createdAt: Date;
  /** Master fingerprint (for HD wallets) */
  masterFingerprint?: string;
  /** Derivation paths */
  derivationPaths?: Record<string, string>;
  /** Supported currencies */
  supportedCurrencies: WalletCurrency[];
  /** Security settings */
  security: SecuritySettings;
  /** Recovery phrase (only available during creation) */
  recoveryPhrase?: string;
}

/**
 * Transaction participant
 */
export interface TransactionParticipant {
  /** Wallet ID */
  walletId?: string;
  /** Address */
  address: string;
  /** Name or label */
  name?: string;
  /** Merchant ID (if applicable) */
  merchantId?: string;
}

/**
 * Money amount
 */
export interface Amount {
  /** Amount value */
  value: number;
  /** Currency code */
  currency: string;
}

/**
 * Transaction metadata
 */
export interface TransactionMetadata {
  /** Merchant category */
  merchantCategory?: string;
  /** Location */
  location?: {
    city: string;
    state?: string;
    country: string;
    coordinates?: {
      lat: number;
      lon: number;
    };
  };
  /** Device information */
  device?: {
    type: 'mobile' | 'desktop' | 'tablet' | 'pos';
    model?: string;
    os?: string;
  };
  /** Receipt URL */
  receiptUrl?: string;
  /** Custom tags */
  tags?: string[];
  /** Notes */
  notes?: string;
}

/**
 * Transaction security information
 */
export interface TransactionSecurity {
  /** Cryptographic signature */
  signature: string;
  /** Authentication method used */
  authenticationMethod: AuthMethod;
  /** Fraud check result */
  fraudCheck: 'passed' | 'failed' | 'pending';
  /** Risk score (0-1) */
  riskScore: number;
}

/**
 * Transaction information
 */
export interface Transaction {
  /** Unique transaction ID */
  transactionId: string;
  /** Wallet ID */
  walletId: string;
  /** Transaction timestamp */
  timestamp: Date;
  /** Transaction type */
  type: TransactionType;
  /** Transaction status */
  status: TransactionStatus;
  /** Sender */
  from: TransactionParticipant;
  /** Recipient */
  to: TransactionParticipant;
  /** Transaction amount */
  amount: Amount;
  /** Transaction fee */
  fee: Amount;
  /** Payment method */
  method: PaymentMethod;
  /** Network information */
  network?: {
    name: string;
    confirmationTime?: string;
  };
  /** Number of confirmations */
  confirmations?: number;
  /** Metadata */
  metadata?: TransactionMetadata;
  /** Security information */
  security?: TransactionSecurity;
  /** Transaction hash (for blockchain) */
  hash?: string;
}

/**
 * Balance information
 */
export interface Balance {
  /** Currency code */
  currency: string;
  /** Available balance */
  available: number;
  /** Pending balance */
  pending: number;
  /** Total balance */
  total: number;
  /** Last updated */
  updatedAt: Date;
}

/**
 * Send payment request
 */
export interface SendPaymentRequest {
  /** Recipient address */
  to: string;
  /** Amount to send */
  amount: number;
  /** Currency code */
  currency: string;
  /** Payment method */
  method?: PaymentMethod;
  /** Transaction note */
  note?: string;
  /** Custom metadata */
  metadata?: Record<string, any>;
}

/**
 * Send payment options
 */
export interface SendPaymentOptions {
  /** Authentication methods required */
  authenticate?: AuthMethod | AuthMethod[];
  /** Gas price (for crypto) */
  gasPrice?: number;
  /** Gas limit (for crypto) */
  gasLimit?: number;
}

/**
 * Payment request
 */
export interface PaymentRequest {
  /** Request ID */
  id: string;
  /** Amount */
  amount: number;
  /** Currency */
  currency: string;
  /** Merchant name */
  merchant?: string;
  /** Expiration timestamp */
  expires?: Date;
  /** QR code data */
  qrCode: string;
  /** Payment link */
  link: string;
}

/**
 * NFC payment configuration
 */
export interface NFCConfig {
  /** Merchant ID */
  merchantId: string;
  /** Default currency */
  defaultCurrency?: string;
  /** Auto-confirm threshold */
  autoConfirmThreshold?: number;
}

/**
 * QR code options
 */
export interface QRCodeOptions {
  /** QR code type */
  type: 'payment_request' | 'address' | 'authentication';
  /** Amount (for payment requests) */
  amount?: number;
  /** Currency */
  currency?: string;
  /** Merchant name */
  merchant?: string;
  /** Order ID */
  orderId?: string;
  /** Expiration in seconds */
  expiresIn?: number;
}

/**
 * Exchange rate
 */
export interface ExchangeRate {
  /** Source currency */
  from: string;
  /** Target currency */
  to: string;
  /** Exchange rate */
  rate: number;
  /** Timestamp */
  timestamp: Date;
}

/**
 * Currency exchange request
 */
export interface ExchangeRequest {
  /** Source currency */
  from: string;
  /** Target currency */
  to: string;
  /** Amount to exchange */
  amount: number;
}

/**
 * Exchange result
 */
export interface ExchangeResult {
  /** Transaction ID */
  transactionId: string;
  /** Amount sent */
  sent: Amount;
  /** Amount received */
  received: Amount;
  /** Exchange rate used */
  rate: number;
  /** Fee charged */
  fee: Amount;
}

/**
 * API Error
 */
export interface WalletError {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** Additional details */
  details?: Record<string, any>;
  /** Timestamp */
  timestamp: Date;
}

/**
 * Transaction filter options
 */
export interface TransactionFilter {
  /** Maximum number of results */
  limit?: number;
  /** Offset for pagination */
  offset?: number;
  /** Filter by currency */
  currency?: string;
  /** Filter by type */
  type?: TransactionType | 'all';
  /** Start date */
  startDate?: Date;
  /** End date */
  endDate?: Date;
  /** Filter by status */
  status?: TransactionStatus;
}

/**
 * Event types
 */
export enum WalletEvent {
  PAYMENT_RECEIVED = 'payment_received',
  PAYMENT_SENT = 'payment_sent',
  BALANCE_UPDATED = 'balance_updated',
  TRANSACTION_CONFIRMED = 'transaction_confirmed',
  TRANSACTION_FAILED = 'transaction_failed',
  NFC_TAP = 'nfc_tap',
  QR_SCANNED = 'qr_scanned',
  SECURITY_ALERT = 'security_alert'
}

/**
 * Event handler type
 */
export type EventHandler<T = any> = (data: T) => void | Promise<void>;
