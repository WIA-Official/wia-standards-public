/**
 * WIA-IND-020: Retail Tech - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Retail Types
// ============================================================================

/**
 * Currency codes (ISO 4217)
 */
export type CurrencyCode = 'USD' | 'EUR' | 'GBP' | 'JPY' | 'CNY' | 'KRW' | 'CAD' | 'AUD';

/**
 * Transaction status
 */
export type TransactionStatus =
  | 'pending'
  | 'processing'
  | 'completed'
  | 'cancelled'
  | 'refunded'
  | 'partially_refunded'
  | 'failed';

/**
 * Payment method types
 */
export type PaymentMethod =
  | 'credit_card'
  | 'debit_card'
  | 'cash'
  | 'digital_wallet'
  | 'bank_transfer'
  | 'buy_now_pay_later'
  | 'gift_card'
  | 'cryptocurrency'
  | 'check';

/**
 * Retail channel types
 */
export type RetailChannel =
  | 'in_store'
  | 'online'
  | 'mobile_app'
  | 'social_commerce'
  | 'marketplace'
  | 'popup_store'
  | 'kiosk'
  | 'phone_order';

// ============================================================================
// Product and Inventory Types
// ============================================================================

/**
 * Product information
 */
export interface Product {
  /** Stock Keeping Unit - unique identifier */
  sku: string;

  /** Universal Product Code */
  upc?: string;

  /** European Article Number */
  ean?: string;

  /** Product name */
  name: string;

  /** Product description */
  description?: string;

  /** Product category */
  category: string;

  /** Product brand */
  brand?: string;

  /** Base price */
  price: number;

  /** Cost of goods sold */
  cost?: number;

  /** Currency code */
  currency: CurrencyCode;

  /** Tax category */
  taxCategory: TaxCategory;

  /** Product attributes */
  attributes?: Record<string, string>;

  /** Product variants (size, color, etc.) */
  variants?: ProductVariant[];

  /** Product images */
  images?: string[];

  /** Product weight in kg */
  weight?: number;

  /** Product dimensions */
  dimensions?: Dimensions;

  /** Is product active */
  active: boolean;

  /** Created timestamp */
  createdAt: Date;

  /** Updated timestamp */
  updatedAt: Date;
}

/**
 * Product variant
 */
export interface ProductVariant {
  /** Variant SKU */
  sku: string;

  /** Variant name */
  name: string;

  /** Variant attributes */
  attributes: Record<string, string>;

  /** Price difference from base */
  priceDelta?: number;

  /** Inventory levels */
  inventory?: InventoryLevel;
}

/**
 * Product dimensions
 */
export interface Dimensions {
  /** Length in cm */
  length: number;

  /** Width in cm */
  width: number;

  /** Height in cm */
  height: number;
}

/**
 * Inventory level
 */
export interface InventoryLevel {
  /** Product SKU */
  sku: string;

  /** Store or warehouse ID */
  locationId: string;

  /** Available quantity */
  availableQuantity: number;

  /** Reserved quantity (in carts/orders) */
  reservedQuantity: number;

  /** In-transit quantity */
  inTransitQuantity: number;

  /** Reorder point */
  reorderPoint: number;

  /** Reorder quantity */
  reorderQuantity: number;

  /** Last stock check */
  lastStockCheck?: Date;
}

/**
 * Stock movement types
 */
export type StockMovementType =
  | 'purchase'
  | 'sale'
  | 'return'
  | 'adjustment'
  | 'transfer'
  | 'damaged'
  | 'theft'
  | 'cycle_count';

/**
 * Stock movement record
 */
export interface StockMovement {
  /** Movement ID */
  id: string;

  /** Product SKU */
  sku: string;

  /** Location ID */
  locationId: string;

  /** Movement type */
  type: StockMovementType;

  /** Quantity change */
  quantity: number;

  /** Reference ID (order, transfer, etc.) */
  referenceId?: string;

  /** Notes */
  notes?: string;

  /** Employee ID */
  employeeId?: string;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Transaction Types
// ============================================================================

/**
 * Retail transaction
 */
export interface Transaction {
  /** Transaction ID */
  id: string;

  /** Store ID */
  storeId: string;

  /** Register/POS ID */
  registerId: string;

  /** Channel */
  channel: RetailChannel;

  /** Transaction items */
  items: TransactionItem[];

  /** Customer information */
  customer?: Customer;

  /** Subtotal before tax and discounts */
  subtotal: number;

  /** Total tax amount */
  tax: number;

  /** Total discount amount */
  discount: number;

  /** Shipping amount */
  shipping: number;

  /** Total amount */
  total: number;

  /** Currency */
  currency: CurrencyCode;

  /** Payment information */
  payments: Payment[];

  /** Transaction status */
  status: TransactionStatus;

  /** Employee/cashier ID */
  employeeId?: string;

  /** Applied promotions */
  promotions?: Promotion[];

  /** Loyalty points earned */
  loyaltyPointsEarned?: number;

  /** Loyalty points redeemed */
  loyaltyPointsRedeemed?: number;

  /** Receipt number */
  receiptNumber: string;

  /** Notes */
  notes?: string;

  /** Created timestamp */
  createdAt: Date;

  /** Updated timestamp */
  updatedAt: Date;
}

/**
 * Transaction line item
 */
export interface TransactionItem {
  /** Line item ID */
  id: string;

  /** Product SKU */
  sku: string;

  /** Product name */
  name: string;

  /** Quantity */
  quantity: number;

  /** Unit price */
  price: number;

  /** Original price (before discounts) */
  originalPrice?: number;

  /** Tax rate */
  taxRate: number;

  /** Tax amount */
  taxAmount: number;

  /** Discount amount */
  discountAmount: number;

  /** Line total */
  total: number;

  /** Applied discounts */
  discounts?: Discount[];

  /** Product metadata */
  metadata?: Record<string, any>;
}

// ============================================================================
// Payment Types
// ============================================================================

/**
 * Payment information
 */
export interface Payment {
  /** Payment ID */
  id: string;

  /** Transaction ID */
  transactionId: string;

  /** Payment method */
  method: PaymentMethod;

  /** Payment amount */
  amount: number;

  /** Currency */
  currency: CurrencyCode;

  /** Payment status */
  status: PaymentStatus;

  /** Authorization code */
  authorizationCode?: string;

  /** Card information (tokenized) */
  cardInfo?: CardInfo;

  /** Digital wallet info */
  walletInfo?: DigitalWalletInfo;

  /** Processing fee */
  processingFee?: number;

  /** Payment processor */
  processor?: string;

  /** Timestamp */
  timestamp: Date;

  /** Settlement date */
  settlementDate?: Date;
}

/**
 * Payment status
 */
export type PaymentStatus =
  | 'pending'
  | 'authorized'
  | 'captured'
  | 'settled'
  | 'failed'
  | 'cancelled'
  | 'refunded';

/**
 * Card information (tokenized)
 */
export interface CardInfo {
  /** Card token */
  token: string;

  /** Last 4 digits */
  last4: string;

  /** Card brand */
  brand: CardBrand;

  /** Expiry month */
  expiryMonth: number;

  /** Expiry year */
  expiryYear: number;

  /** Cardholder name */
  cardholderName?: string;
}

/**
 * Card brand types
 */
export type CardBrand = 'visa' | 'mastercard' | 'amex' | 'discover' | 'jcb' | 'unionpay';

/**
 * Digital wallet information
 */
export interface DigitalWalletInfo {
  /** Wallet type */
  type: DigitalWalletType;

  /** Wallet account ID */
  accountId: string;

  /** Transaction ID from wallet provider */
  providerTransactionId?: string;
}

/**
 * Digital wallet types
 */
export type DigitalWalletType =
  | 'apple_pay'
  | 'google_pay'
  | 'samsung_pay'
  | 'paypal'
  | 'venmo'
  | 'alipay'
  | 'wechat_pay';

// ============================================================================
// Customer Types
// ============================================================================

/**
 * Customer information
 */
export interface Customer {
  /** Customer ID */
  id: string;

  /** Email address */
  email?: string;

  /** Phone number */
  phone?: string;

  /** First name */
  firstName?: string;

  /** Last name */
  lastName?: string;

  /** Full name */
  fullName?: string;

  /** Date of birth */
  dateOfBirth?: Date;

  /** Loyalty program membership */
  loyaltyMembership?: LoyaltyMembership;

  /** Addresses */
  addresses?: Address[];

  /** Customer preferences */
  preferences?: CustomerPreferences;

  /** Customer segment */
  segment?: string;

  /** Lifetime value */
  lifetimeValue?: number;

  /** Total purchases */
  totalPurchases?: number;

  /** Average order value */
  averageOrderValue?: number;

  /** Created date */
  createdAt: Date;

  /** Last purchase date */
  lastPurchaseAt?: Date;
}

/**
 * Customer address
 */
export interface Address {
  /** Address ID */
  id: string;

  /** Address type */
  type: AddressType;

  /** Street address line 1 */
  street1: string;

  /** Street address line 2 */
  street2?: string;

  /** City */
  city: string;

  /** State/province */
  state: string;

  /** Postal/ZIP code */
  postalCode: string;

  /** Country code */
  country: string;

  /** Is default address */
  isDefault?: boolean;
}

/**
 * Address types
 */
export type AddressType = 'billing' | 'shipping' | 'both';

/**
 * Customer preferences
 */
export interface CustomerPreferences {
  /** Email marketing opt-in */
  emailMarketing?: boolean;

  /** SMS marketing opt-in */
  smsMarketing?: boolean;

  /** Preferred language */
  language?: string;

  /** Preferred currency */
  currency?: CurrencyCode;

  /** Preferred communication channel */
  communicationChannel?: 'email' | 'sms' | 'phone' | 'mail';
}

// ============================================================================
// Loyalty Program Types
// ============================================================================

/**
 * Loyalty membership
 */
export interface LoyaltyMembership {
  /** Membership ID */
  id: string;

  /** Loyalty program ID */
  programId: string;

  /** Membership number */
  membershipNumber: string;

  /** Current points balance */
  pointsBalance: number;

  /** Tier level */
  tier: LoyaltyTier;

  /** Points earned to date */
  lifetimePoints: number;

  /** Member since */
  memberSince: Date;

  /** Tier expiration date */
  tierExpirationDate?: Date;
}

/**
 * Loyalty tier
 */
export interface LoyaltyTier {
  /** Tier name */
  name: string;

  /** Tier level (1 = lowest) */
  level: number;

  /** Points required */
  pointsRequired: number;

  /** Benefits */
  benefits: string[];

  /** Discount percentage */
  discountPercentage?: number;
}

/**
 * Loyalty transaction
 */
export interface LoyaltyTransaction {
  /** Transaction ID */
  id: string;

  /** Customer ID */
  customerId: string;

  /** Points change */
  pointsChange: number;

  /** Transaction type */
  type: LoyaltyTransactionType;

  /** Reference ID (sale, return, etc.) */
  referenceId?: string;

  /** Description */
  description?: string;

  /** Timestamp */
  timestamp: Date;

  /** Expiration date */
  expirationDate?: Date;
}

/**
 * Loyalty transaction types
 */
export type LoyaltyTransactionType =
  | 'earned'
  | 'redeemed'
  | 'bonus'
  | 'adjustment'
  | 'expired'
  | 'refund';

// ============================================================================
// Discount and Promotion Types
// ============================================================================

/**
 * Discount
 */
export interface Discount {
  /** Discount ID */
  id: string;

  /** Discount code */
  code?: string;

  /** Discount type */
  type: DiscountType;

  /** Discount value */
  value: number;

  /** Discount amount (calculated) */
  amount: number;

  /** Description */
  description?: string;
}

/**
 * Discount types
 */
export type DiscountType = 'percentage' | 'fixed_amount' | 'bogo' | 'free_shipping';

/**
 * Promotion
 */
export interface Promotion {
  /** Promotion ID */
  id: string;

  /** Promotion name */
  name: string;

  /** Promotion code */
  code?: string;

  /** Promotion type */
  type: PromotionType;

  /** Discount type */
  discountType: DiscountType;

  /** Discount value */
  discountValue: number;

  /** Minimum purchase amount */
  minimumPurchase?: number;

  /** Maximum discount amount */
  maximumDiscount?: number;

  /** Applicable products (SKUs) */
  applicableProducts?: string[];

  /** Applicable categories */
  applicableCategories?: string[];

  /** Start date */
  startDate: Date;

  /** End date */
  endDate: Date;

  /** Usage limit per customer */
  usageLimitPerCustomer?: number;

  /** Total usage limit */
  totalUsageLimit?: number;

  /** Current usage count */
  usageCount?: number;

  /** Is stackable with other promotions */
  stackable: boolean;

  /** Active status */
  active: boolean;
}

/**
 * Promotion types
 */
export type PromotionType =
  | 'sale'
  | 'coupon'
  | 'loyalty'
  | 'clearance'
  | 'seasonal'
  | 'flash_sale'
  | 'bundle';

// ============================================================================
// Tax Types
// ============================================================================

/**
 * Tax category
 */
export type TaxCategory =
  | 'standard'
  | 'reduced'
  | 'zero'
  | 'exempt'
  | 'food'
  | 'clothing'
  | 'digital_goods';

/**
 * Tax calculation
 */
export interface TaxCalculation {
  /** Taxable amount */
  taxableAmount: number;

  /** Tax rate */
  taxRate: number;

  /** Tax amount */
  taxAmount: number;

  /** Tax jurisdiction */
  jurisdiction?: string;

  /** Tax breakdown by type */
  breakdown?: TaxBreakdown[];
}

/**
 * Tax breakdown
 */
export interface TaxBreakdown {
  /** Tax type */
  type: TaxType;

  /** Tax rate */
  rate: number;

  /** Tax amount */
  amount: number;

  /** Tax name */
  name: string;
}

/**
 * Tax types
 */
export type TaxType = 'sales_tax' | 'vat' | 'gst' | 'pst' | 'city_tax' | 'county_tax';

// ============================================================================
// Return and Refund Types
// ============================================================================

/**
 * Return request
 */
export interface ReturnRequest {
  /** Return ID */
  id: string;

  /** Original transaction ID */
  transactionId: string;

  /** Customer ID */
  customerId: string;

  /** Store ID */
  storeId: string;

  /** Return items */
  items: ReturnItem[];

  /** Return reason */
  reason: ReturnReason;

  /** Return method */
  method: ReturnMethod;

  /** Refund amount */
  refundAmount: number;

  /** Restocking fee */
  restockingFee?: number;

  /** Return status */
  status: ReturnStatus;

  /** Created date */
  createdAt: Date;

  /** Processed date */
  processedAt?: Date;

  /** Notes */
  notes?: string;
}

/**
 * Return item
 */
export interface ReturnItem {
  /** Original transaction item ID */
  transactionItemId: string;

  /** Product SKU */
  sku: string;

  /** Quantity to return */
  quantity: number;

  /** Return reason */
  reason: ReturnReason;

  /** Condition */
  condition: ItemCondition;
}

/**
 * Return reasons
 */
export type ReturnReason =
  | 'defective'
  | 'wrong_item'
  | 'not_as_described'
  | 'changed_mind'
  | 'size_fit'
  | 'damaged_shipping'
  | 'other';

/**
 * Return methods
 */
export type ReturnMethod = 'in_store' | 'mail' | 'pickup';

/**
 * Return status
 */
export type ReturnStatus =
  | 'requested'
  | 'approved'
  | 'received'
  | 'inspected'
  | 'refunded'
  | 'rejected'
  | 'cancelled';

/**
 * Item condition
 */
export type ItemCondition = 'new' | 'like_new' | 'good' | 'damaged' | 'defective';

// ============================================================================
// Analytics and Reporting Types
// ============================================================================

/**
 * Sales report
 */
export interface SalesReport {
  /** Report period start */
  startDate: Date;

  /** Report period end */
  endDate: Date;

  /** Total sales */
  totalSales: number;

  /** Total transactions */
  totalTransactions: number;

  /** Average transaction value */
  averageTransactionValue: number;

  /** Total items sold */
  totalItemsSold: number;

  /** Total tax collected */
  totalTax: number;

  /** Total discounts */
  totalDiscounts: number;

  /** Gross profit */
  grossProfit: number;

  /** Gross margin percentage */
  grossMargin: number;

  /** Sales by channel */
  salesByChannel: Record<RetailChannel, number>;

  /** Sales by category */
  salesByCategory: Record<string, number>;

  /** Top products */
  topProducts: ProductSales[];
}

/**
 * Product sales metrics
 */
export interface ProductSales {
  /** Product SKU */
  sku: string;

  /** Product name */
  name: string;

  /** Quantity sold */
  quantitySold: number;

  /** Total revenue */
  revenue: number;

  /** Total profit */
  profit: number;

  /** Profit margin */
  profitMargin: number;
}

/**
 * Inventory report
 */
export interface InventoryReport {
  /** Report date */
  reportDate: Date;

  /** Total SKUs */
  totalSkus: number;

  /** Total inventory value (cost) */
  totalInventoryValue: number;

  /** Total inventory value (retail) */
  totalRetailValue: number;

  /** Total units */
  totalUnits: number;

  /** Out of stock items */
  outOfStockItems: number;

  /** Low stock items */
  lowStockItems: number;

  /** Inventory turnover ratio */
  inventoryTurnover: number;

  /** Days inventory outstanding */
  daysInventoryOutstanding: number;

  /** Shrinkage amount */
  shrinkageAmount: number;

  /** Shrinkage percentage */
  shrinkagePercentage: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Retail error codes
 */
export enum RetailErrorCode {
  // General errors
  INVALID_PARAMETERS = 'INVALID_PARAMETERS',
  RESOURCE_NOT_FOUND = 'RESOURCE_NOT_FOUND',
  UNAUTHORIZED = 'UNAUTHORIZED',
  FORBIDDEN = 'FORBIDDEN',

  // Transaction errors
  INVALID_TRANSACTION = 'INVALID_TRANSACTION',
  TRANSACTION_CANCELLED = 'TRANSACTION_CANCELLED',
  PAYMENT_FAILED = 'PAYMENT_FAILED',
  INSUFFICIENT_FUNDS = 'INSUFFICIENT_FUNDS',

  // Inventory errors
  INSUFFICIENT_INVENTORY = 'INSUFFICIENT_INVENTORY',
  INVALID_SKU = 'INVALID_SKU',
  INVENTORY_LOCKED = 'INVENTORY_LOCKED',

  // Customer errors
  INVALID_CUSTOMER = 'INVALID_CUSTOMER',
  DUPLICATE_CUSTOMER = 'DUPLICATE_CUSTOMER',

  // Discount errors
  INVALID_DISCOUNT_CODE = 'INVALID_DISCOUNT_CODE',
  DISCOUNT_EXPIRED = 'DISCOUNT_EXPIRED',
  DISCOUNT_LIMIT_REACHED = 'DISCOUNT_LIMIT_REACHED',

  // Return errors
  RETURN_WINDOW_EXPIRED = 'RETURN_WINDOW_EXPIRED',
  NON_RETURNABLE_ITEM = 'NON_RETURNABLE_ITEM',
  INVALID_RETURN = 'INVALID_RETURN',
}

/**
 * Retail error
 */
export class RetailError extends Error {
  constructor(
    public code: RetailErrorCode,
    message: string,
    public details?: any
  ) {
    super(message);
    this.name = 'RetailError';
  }
}

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Retail SDK configuration
 */
export interface RetailSDKConfig {
  /** API key */
  apiKey: string;

  /** Store ID */
  storeId: string;

  /** Environment */
  environment: 'production' | 'staging' | 'development';

  /** API base URL */
  apiUrl?: string;

  /** Timeout in milliseconds */
  timeout?: number;

  /** Default currency */
  defaultCurrency?: CurrencyCode;

  /** Enable logging */
  logging?: boolean;
}

/**
 * Store configuration
 */
export interface StoreConfig {
  /** Store ID */
  id: string;

  /** Store name */
  name: string;

  /** Store type */
  type: 'retail' | 'warehouse' | 'popup' | 'kiosk';

  /** Time zone */
  timezone: string;

  /** Currency */
  currency: CurrencyCode;

  /** Tax settings */
  taxSettings: TaxSettings;

  /** Address */
  address: Address;

  /** Contact information */
  contact: ContactInfo;

  /** Operating hours */
  operatingHours: OperatingHours[];

  /** Active status */
  active: boolean;
}

/**
 * Tax settings
 */
export interface TaxSettings {
  /** Default tax rate */
  defaultTaxRate: number;

  /** Tax inclusive pricing */
  taxInclusive: boolean;

  /** Tax by category */
  taxByCategory: Record<TaxCategory, number>;

  /** Tax jurisdiction */
  jurisdiction?: string;
}

/**
 * Contact information
 */
export interface ContactInfo {
  /** Email */
  email?: string;

  /** Phone */
  phone?: string;

  /** Website */
  website?: string;
}

/**
 * Operating hours
 */
export interface OperatingHours {
  /** Day of week (0 = Sunday) */
  dayOfWeek: number;

  /** Open time (HH:MM) */
  openTime: string;

  /** Close time (HH:MM) */
  closeTime: string;

  /** Is closed */
  isClosed: boolean;
}

// ============================================================================
// Export all types
// ============================================================================

export * from './types';

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
