# WIA-IND-020 — Phase 3: Protocol

> Pricing, promotions, payment-processing, and returns/refunds operational protocol layer. Every protocol exchange has documented audit trail for tax and reconciliation.

## 9. Price Management

### 9.1 Pricing Strategies

**1. Cost-Plus Pricing**
```
Selling Price = Cost + (Cost × Markup Percentage)

Example:
Cost: $50
Markup: 40%
Price = $50 + ($50 × 0.40) = $70
```

**2. Competitive Pricing**
```
Monitor competitor prices and adjust accordingly:
- Match competitor prices
- Undercut by X%
- Premium pricing (higher than competitors)
```

**3. Dynamic Pricing**
```typescript
interface DynamicPricing {
  sku: string;
  basePrice: number;
  rules: PricingRule[];
}

interface PricingRule {
  condition: 'time' | 'demand' | 'inventory' | 'customer_segment';
  operator: 'increase' | 'decrease';
  value: number;
  valueType: 'percentage' | 'fixed';
  priority: number;
}

// Example: Increase price during peak hours
{
  condition: 'time',
  operator: 'increase',
  value: 10,
  valueType: 'percentage',
  priority: 1,
  timeRange: { start: '09:00', end: '17:00' }
}
```

### 9.2 Price Tiers

```typescript
interface PriceTier {
  sku: string;
  tiers: {
    minQuantity: number;
    maxQuantity?: number;
    price: number;
    discount?: number;
  }[];
}

// Example: Volume discounts
{
  sku: 'WIDGET-001',
  tiers: [
    { minQuantity: 1, maxQuantity: 9, price: 10.00 },
    { minQuantity: 10, maxQuantity: 49, price: 9.00, discount: 10 },
    { minQuantity: 50, price: 8.00, discount: 20 }
  ]
}
```

### 9.3 Customer-Specific Pricing

```typescript
interface CustomerPricing {
  customerId: string;
  segment: string;
  priceList?: string;
  discountPercentage?: number;
  products: {
    sku: string;
    specialPrice?: number;
    discountPercentage?: number;
  }[];
}
```

---


## 10. Promotions and Discounts

### 10.1 Promotion Types

**1. Percentage Discount**
```
Discount = Original Price × (Discount % / 100)
Final Price = Original Price - Discount

Example:
Original: $100
Discount: 20%
Final = $100 - ($100 × 0.20) = $80
```

**2. Fixed Amount Discount**
```
Final Price = Original Price - Discount Amount

Example:
Original: $100
Discount: $15
Final = $100 - $15 = $85
```

**3. Buy One Get One (BOGO)**
```typescript
interface BOGOPromotion {
  buyQuantity: number;
  getQuantity: number;
  getDiscount: number;          // 0-100 (100 = free)
  applicableSkus: string[];
}

// Example: Buy 2, Get 1 Free
{
  buyQuantity: 2,
  getQuantity: 1,
  getDiscount: 100,
  applicableSkus: ['SKU-001', 'SKU-002']
}
```

**4. Bundle Pricing**
```typescript
interface BundlePromotion {
  bundleId: string;
  name: string;
  requiredProducts: {
    sku: string;
    quantity: number;
  }[];
  bundlePrice: number;
  savings: number;
}

// Example: Laptop + Mouse + Bag bundle
{
  bundleId: 'TECH-BUNDLE-001',
  name: 'Work From Home Bundle',
  requiredProducts: [
    { sku: 'LAPTOP-001', quantity: 1 },
    { sku: 'MOUSE-002', quantity: 1 },
    { sku: 'BAG-003', quantity: 1 }
  ],
  bundlePrice: 999,
  savings: 150
}
```

### 10.2 Promotion Rules

```typescript
interface Promotion {
  id: string;
  code?: string;
  name: string;
  description: string;

  // Type
  type: 'percentage' | 'fixed_amount' | 'bogo' | 'bundle' | 'free_shipping';

  // Value
  discountValue?: number;

  // Conditions
  minimumPurchase?: number;
  maximumDiscount?: number;
  applicableProducts?: string[];
  applicableCategories?: string[];
  excludedProducts?: string[];

  // Customer restrictions
  customerSegments?: string[];
  newCustomersOnly?: boolean;

  // Usage limits
  usageLimitPerCustomer?: number;
  totalUsageLimit?: number;
  currentUsageCount: number;

  // Timing
  startDate: Date;
  endDate: Date;

  // Stacking
  stackable: boolean;
  priority: number;

  // Status
  active: boolean;
}
```

### 10.3 Promotion Application Logic

```typescript
function applyPromotions(
  cart: ShoppingCart,
  promotions: Promotion[]
): DiscountCalculation {
  // Sort by priority
  const sortedPromotions = promotions.sort((a, b) => a.priority - b.priority);

  let totalDiscount = 0;
  const appliedPromotions: string[] = [];

  for (const promo of sortedPromotions) {
    // Check if promotion is valid
    if (!isPromotionValid(promo, cart)) continue;

    // Calculate discount
    const discount = calculatePromoDiscount(promo, cart);

    // Apply maximum discount limit
    const finalDiscount = promo.maximumDiscount
      ? Math.min(discount, promo.maximumDiscount)
      : discount;

    totalDiscount += finalDiscount;
    appliedPromotions.push(promo.id);

    // If not stackable, stop here
    if (!promo.stackable) break;
  }

  return {
    totalDiscount,
    appliedPromotions
  };
}
```

---


## 11. Payment Processing

### 11.1 Supported Payment Methods

**Credit/Debit Cards:**
- Visa
- Mastercard
- American Express
- Discover
- JCB
- UnionPay

**Digital Wallets:**
- Apple Pay
- Google Pay
- Samsung Pay
- PayPal
- Venmo

**Alternative Methods:**
- Buy Now, Pay Later (Afterpay, Klarna, Affirm)
- Bank Transfer (ACH, Wire)
- Gift Cards
- Store Credit
- Cash (in-store only)
- Cryptocurrency

### 11.2 Payment Processing Flow

```
1. Payment Initiation
   ├─> Customer selects payment method
   ├─> Enter payment details
   └─> Confirm amount

2. Tokenization
   ├─> Convert card details to token
   ├─> Token sent to payment gateway
   └─> Original card data never stored

3. Authorization
   ├─> Send authorization request
   ├─> Check customer funds
   ├─> Apply fraud checks
   └─> Receive authorization code

4. Capture
   ├─> Capture authorized amount
   ├─> Funds transferred from customer
   └─> Payment confirmed

5. Settlement
   ├─> Batch processing (typically daily)
   ├─> Funds deposited to merchant account
   └─> Settlement complete
```

### 11.3 Payment Data Model

```typescript
interface Payment {
  id: string;
  transactionId: string;

  // Amount
  amount: number;
  currency: string;

  // Method
  method: PaymentMethod;
  cardInfo?: CardInfo;
  walletInfo?: DigitalWalletInfo;

  // Processing
  status: 'pending' | 'authorized' | 'captured' | 'settled' |
          'failed' | 'cancelled' | 'refunded';
  authorizationCode?: string;
  processor: string;
  processingFee: number;

  // Security
  riskScore?: number;
  fraudChecks: FraudCheck[];

  // Timestamps
  authorizedAt?: Date;
  capturedAt?: Date;
  settledAt?: Date;
  timestamp: Date;
}

interface CardInfo {
  token: string;                // Tokenized card number
  last4: string;
  brand: 'visa' | 'mastercard' | 'amex' | 'discover';
  expiryMonth: number;
  expiryYear: number;
  cardholderName?: string;
}
```

### 11.4 Security Standards

**PCI DSS Compliance:**
- Build and maintain secure network
- Protect cardholder data
- Maintain vulnerability management program
- Implement strong access control measures
- Regularly monitor and test networks
- Maintain information security policy

**Tokenization:**
- Replace sensitive card data with tokens
- Tokens useless if intercepted
- Original data stored in secure vault
- Reduces PCI compliance scope

**EMV Chip Cards:**
- Dynamic authentication for each transaction
- Prevents card cloning
- Liability shift to merchants for non-chip transactions

**3D Secure (3DS):**
- Additional authentication layer
- Customer verifies with password/biometric
- Reduces fraud, shifts liability to issuer

---


## 12. Returns and Refunds

### 12.1 Return Policy

**Standard Return Window:**
- 30 days for most items
- 90 days for electronics
- 14 days for apparel (hygiene)
- No returns on final sale items

**Return Conditions:**
- Items in original condition
- Original packaging included
- Receipt or proof of purchase
- Tags still attached (apparel)

### 12.2 Return Process

```typescript
interface ReturnRequest {
  id: string;
  transactionId: string;
  customerId: string;

  // Items
  items: ReturnItem[];

  // Details
  reason: ReturnReason;
  method: 'in_store' | 'mail' | 'pickup';
  condition: 'new' | 'like_new' | 'good' | 'damaged';

  // Financial
  refundAmount: number;
  restockingFee: number;
  refundMethod: 'original_payment' | 'store_credit' | 'exchange';

  // Status
  status: 'requested' | 'approved' | 'received' |
          'inspected' | 'refunded' | 'rejected';

  // Tracking
  returnLabel?: string;
  trackingNumber?: string;

  // Timestamps
  createdAt: Date;
  receivedAt?: Date;
  inspectedAt?: Date;
  refundedAt?: Date;

  // Notes
  customerNotes?: string;
  internalNotes?: string;
}

interface ReturnItem {
  transactionItemId: string;
  sku: string;
  quantity: number;
  reason: ReturnReason;
  condition: ItemCondition;
  refundAmount: number;
}

type ReturnReason =
  | 'defective'
  | 'wrong_item'
  | 'not_as_described'
  | 'changed_mind'
  | 'size_fit'
  | 'damaged_shipping'
  | 'other';
```

### 12.3 Refund Calculation

```typescript
function calculateRefund(returnRequest: ReturnRequest): RefundCalculation {
  let refundAmount = 0;

  // Sum item refunds
  for (const item of returnRequest.items) {
    refundAmount += item.refundAmount;
  }

  // Apply restocking fee if applicable
  const restockingFee = calculateRestockingFee(returnRequest);
  refundAmount -= restockingFee;

  // Refund shipping if product was defective
  if (returnRequest.reason === 'defective') {
    refundAmount += getOriginalShipping(returnRequest.transactionId);
  }

  // Deduct return shipping if customer's fault
  if (!isDefectiveReturn(returnRequest.reason)) {
    const returnShipping = 9.99; // Standard return shipping
    refundAmount -= returnShipping;
  }

  return {
    subtotal: refundAmount,
    restockingFee,
    returnShipping: returnShipping,
    total: Math.max(0, refundAmount)
  };
}
```

---



## A.1 Pricing and promotions protocol

The pricing protocol layer carries: list price, regional price,
markdown schedule, promotion eligibility, and the rule chain that
produced the final price. Every promotional discount is auditable
from envelopes alone — a regulator inquiring about misleading
pricing can reconstruct the price chain for any transaction.

## A.2 Payment processing

Payment processing wraps PCI-DSS-compliant tokenisation and 3DS2
authentication. The protocol envelopes carry token references
rather than raw card numbers; full PANs never appear in any
standardised envelope.

## A.3 Returns and refunds

Returns and refunds carry the original-transaction reference, the
reason code, the inspector's identity (for high-value items), and
the refund destination. The envelope is signed by the returning
store and the original-sale store; reconciliation happens at the
operator's accounting period close.

## A.4 Replay defence and audit

Standard 96-bit nonce + 300-second skew window + 600-second
seen-nonce cache. Audit envelopes are written to an append-only log
with retention sized to the regulatory requirement (typically 7
years for tax purposes, 10 years for consumer-protection appeals).

## A.5 Federation across operators

Cross-operator federation (e.g., loyalty alliances, shared returns
acceptance) uses WIA-SOCIAL Phase 3 §5 federation receipts. Each
operator publishes a trust list naming the federation peers; loyalty
points earned at one operator can be burned at another with full
audit chain.


## Z.1 Glossary

The companion glossary at `https://wiastandards.com/retail-tech/glossary/`
expands every term used throughout this Phase. Implementers
unfamiliar with the domain should treat it as load-bearing reading.

## Z.2 Cross-standard composition

This Phase composes with: **WIA-OMNI-API** (credential storage),
**WIA-AIR-SHIELD** (runtime trust list), **WIA-SOCIAL Phase 3 §5**
(federation handshake), and **WIA-INTENT** (workload intent
declaration).

## Z.3 Conformance test suite + reference container

A black-box conformance test suite at
`https://github.com/WIA-Official/wia-retail-tech-conformance` walks
every public endpoint and protocol exchange. The reference
container at `wia/retail-tech-host:1.0.0` implements every Phase 2
endpoint with mock data so integrators exercise their bridge
before production. The companion CLI at `cli/retail-tech.sh` ships
sample envelope generators (validate, info, plus phase-specific
subcommands) so an implementer can produce conformant payloads
without hand-rolling JSON.

## Z.4 Implementation runbook

A first implementation typically follows: (1) stand up reference
container, (2) run conformance suite against it, (3) replace mock
backend with real backend one endpoint at a time, (4) wire up audit
log replication, (5) onboard a single trusted peer for federation,
(6) expand to multiple peers, (7) promote to production with
warning-envelope subscription.

## Z.5 Backwards-compatibility promise + governance

Within the 1.x line every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable.
Hosts MAY add optional fields and new envelopes; hosts MUST NOT
remove existing ones. Breaking changes ride a major version bump
with a 12-month deprecation window per IETF RFC 8594 / 9745, and
require a two-thirds Committee vote.

弘益人間 — Benefit All Humanity.
