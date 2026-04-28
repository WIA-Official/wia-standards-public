# WIA-IND-020 — Phase 2: API

> Customer-facing API surface: CRM, loyalty, product-information management, and inventory management — each presented as a worked endpoint specification.

## 5. Customer Relationship Management

### 5.1 Customer Data Model

```typescript
interface Customer {
  // Identity
  id: string;
  email: string;
  phone?: string;
  firstName: string;
  lastName: string;
  dateOfBirth?: Date;

  // Addresses
  addresses: Address[];
  defaultShippingAddress?: string;
  defaultBillingAddress?: string;

  // Loyalty
  loyaltyMembership?: LoyaltyMembership;

  // Preferences
  preferences: CustomerPreferences;

  // Metrics
  lifetimeValue: number;
  totalPurchases: number;
  totalSpent: number;
  averageOrderValue: number;
  purchaseFrequency: number;

  // Segmentation
  segment: 'vip' | 'regular' | 'new' | 'at_risk' | 'churned';
  tags: string[];

  // Communication
  marketingOptIn: boolean;
  smsOptIn: boolean;

  // Timestamps
  firstPurchaseAt?: Date;
  lastPurchaseAt?: Date;
  createdAt: Date;
  updatedAt: Date;
}

interface Address {
  id: string;
  type: 'shipping' | 'billing' | 'both';
  street1: string;
  street2?: string;
  city: string;
  state: string;
  postalCode: string;
  country: string;
  isDefault: boolean;
}
```

### 5.2 Customer Segmentation

**Segmentation Criteria:**

| Segment | Definition | Strategy |
|---------|-----------|----------|
| VIP | CLV > $10,000, Frequent buyer | Exclusive offers, early access |
| Regular | CLV $1,000-$10,000 | Standard loyalty benefits |
| New | < 3 months since first purchase | Onboarding, welcome offers |
| At Risk | No purchase in 6+ months | Win-back campaigns |
| Churned | No purchase in 12+ months | Reactivation campaigns |

### 5.3 Customer Lifetime Value Calculation

```
CLV = (Average Order Value × Purchase Frequency × Customer Lifespan) - Acquisition Cost

Where:
- Average Order Value = Total Revenue / Number of Orders
- Purchase Frequency = Number of Orders / Number of Unique Customers
- Customer Lifespan = Average time customer remains active (in years)
- Acquisition Cost = Marketing spend / New customers acquired
```

**Example:**
```
Average Order Value: $75
Purchase Frequency: 8 times/year
Customer Lifespan: 3 years
Acquisition Cost: $50

CLV = ($75 × 8 × 3) - $50
CLV = $1,800 - $50
CLV = $1,750
```

### 5.4 Customer Communication

**Communication Channels:**
- Email: Transactional and marketing emails
- SMS: Order updates, pickup notifications
- Push Notifications: Mobile app alerts
- In-App Messages: Personalized offers
- Direct Mail: Special occasions, catalogs

**Triggered Communications:**
```typescript
interface CommunicationTrigger {
  type: 'welcome' | 'abandoned_cart' | 'order_confirmation' |
        'shipping_notification' | 'delivery_confirmation' |
        'review_request' | 'win_back' | 'birthday';
  channel: 'email' | 'sms' | 'push' | 'in_app';
  timing: {
    delay: number;              // minutes
    condition?: string;         // Additional conditions
  };
  template: string;
  personalization: Record<string, any>;
}
```

---


## 6. Loyalty Programs

### 6.1 Loyalty Program Types

**1. Points-Based Programs**
```
Earn Points: 1 point per $1 spent
Redeem Points: 100 points = $1 discount

Example:
Purchase: $100
Points Earned: 100
Points Balance: 500
Available Discount: $5
```

**2. Tiered Programs**
```
Bronze Tier: 0-499 points
- 1x points on purchases
- Birthday bonus

Silver Tier: 500-1,999 points
- 1.5x points on purchases
- Free shipping
- Birthday bonus

Gold Tier: 2,000+ points
- 2x points on purchases
- Free shipping
- Priority support
- Early access to sales
- Birthday bonus + gift
```

**3. Cashback Programs**
```
Standard: 1% cashback on all purchases
Premium: 3% cashback (annual fee)

Example:
Purchase: $1,000
Cashback: $10 (standard) or $30 (premium)
```

### 6.2 Loyalty Data Model

```typescript
interface LoyaltyMembership {
  id: string;
  customerId: string;
  programId: string;
  membershipNumber: string;

  // Points
  pointsBalance: number;
  lifetimePoints: number;
  expiringPoints: ExpiringPoints[];

  // Tier
  tier: LoyaltyTier;
  tierProgress: number;         // Points toward next tier
  tierExpirationDate?: Date;

  // Status
  status: 'active' | 'suspended' | 'cancelled';
  memberSince: Date;
}

interface LoyaltyTier {
  name: string;
  level: number;
  pointsRequired: number;
  benefits: string[];
  pointsMultiplier: number;
  discountPercentage?: number;
}

interface ExpiringPoints {
  points: number;
  expirationDate: Date;
}

interface LoyaltyTransaction {
  id: string;
  customerId: string;
  pointsChange: number;
  type: 'earned' | 'redeemed' | 'bonus' | 'expired' | 'adjustment';
  referenceId?: string;         // Related order ID
  description: string;
  timestamp: Date;
  expirationDate?: Date;
}
```

### 6.3 Points Calculation

**Earning Points:**
```typescript
function calculatePointsEarned(
  amount: number,
  tierMultiplier: number,
  baseRate: number = 1
): number {
  return Math.floor(amount * baseRate * tierMultiplier);
}

// Example
const purchase = 125.50;
const tierMultiplier = 1.5;  // Silver tier
const points = calculatePointsEarned(purchase, tierMultiplier);
// Result: 188 points
```

**Redeeming Points:**
```typescript
function convertPointsToDiscount(
  points: number,
  conversionRate: number = 0.01
): number {
  return points * conversionRate;
}

// Example
const pointsToRedeem = 500;
const discount = convertPointsToDiscount(pointsToRedeem);
// Result: $5.00 discount
```

### 6.4 Bonus Point Opportunities

- **Welcome Bonus**: 500 points for joining
- **Birthday Bonus**: 2x points on birthday month
- **Product Reviews**: 25 points per review
- **Referrals**: 100 points per successful referral
- **Social Sharing**: 10 points per share
- **Survey Completion**: 50 points
- **Anniversary Bonus**: 100 points per year of membership

---


## 7. Product Information Management

### 7.1 Product Data Model

```typescript
interface Product {
  // Identity
  sku: string;                  // Unique identifier
  upc?: string;                 // Universal Product Code
  ean?: string;                 // European Article Number
  gtin?: string;                // Global Trade Item Number

  // Basic Information
  name: string;
  description: string;
  shortDescription?: string;
  brand: string;
  manufacturer?: string;

  // Classification
  category: string;
  subcategory?: string;
  tags: string[];

  // Pricing
  price: number;
  compareAtPrice?: number;      // Original price for sale items
  cost?: number;                // Cost of goods sold
  currency: string;

  // Tax
  taxCategory: 'standard' | 'reduced' | 'zero' | 'exempt';
  taxable: boolean;

  // Inventory
  trackInventory: boolean;
  inventoryPolicy: 'deny' | 'continue';  // Out of stock behavior

  // Physical Attributes
  weight?: number;              // kg
  dimensions?: {
    length: number;             // cm
    width: number;
    height: number;
  };

  // Media
  images: ProductImage[];
  videos?: ProductVideo[];

  // Variants
  hasVariants: boolean;
  variants?: ProductVariant[];
  variantAttributes?: string[]; // e.g., ['size', 'color']

  // SEO
  seo: {
    title: string;
    description: string;
    keywords: string[];
    slug: string;
  };

  // Status
  status: 'active' | 'draft' | 'archived';
  publishedAt?: Date;
  createdAt: Date;
  updatedAt: Date;
}

interface ProductVariant {
  sku: string;
  name: string;
  attributes: Record<string, string>;  // e.g., {size: 'Large', color: 'Blue'}
  price?: number;                      // If different from base
  compareAtPrice?: number;
  weight?: number;
  image?: string;
  inventory?: InventoryLevel;
}

interface ProductImage {
  id: string;
  url: string;
  alt: string;
  position: number;
  isDefault: boolean;
}
```

### 7.2 Product Categorization

**Category Hierarchy:**
```
Electronics
├── Computers
│   ├── Laptops
│   ├── Desktops
│   └── Tablets
├── Mobile Phones
│   ├── Smartphones
│   └── Accessories
└── Audio
    ├── Headphones
    └── Speakers

Clothing
├── Men
│   ├── Shirts
│   ├── Pants
│   └── Shoes
├── Women
│   ├── Dresses
│   ├── Tops
│   └── Shoes
└── Accessories
    ├── Bags
    └── Jewelry
```

### 7.3 Product Attributes

**Standard Attributes:**
- Brand
- Model Number
- Color
- Size
- Material
- Country of Origin
- Care Instructions
- Warranty

**Custom Attributes (by category):**
```typescript
interface CategoryAttributes {
  categoryId: string;
  attributes: Attribute[];
}

interface Attribute {
  name: string;
  type: 'text' | 'number' | 'select' | 'multiselect' | 'boolean';
  required: boolean;
  options?: string[];           // For select/multiselect
  unit?: string;                // For number (e.g., 'inches', 'kg')
}

// Example: Electronics > Laptops
{
  categoryId: 'electronics-laptops',
  attributes: [
    { name: 'Processor', type: 'text', required: true },
    { name: 'RAM', type: 'select', required: true,
      options: ['4GB', '8GB', '16GB', '32GB'], unit: 'GB' },
    { name: 'Storage', type: 'select', required: true,
      options: ['256GB SSD', '512GB SSD', '1TB SSD'] },
    { name: 'Screen Size', type: 'number', required: true, unit: 'inches' },
    { name: 'Operating System', type: 'select', required: true,
      options: ['Windows 11', 'macOS', 'Linux'] }
  ]
}
```

### 7.4 Product Search and Filters

**Search Capabilities:**
- Full-text search across name, description, SKU
- Synonym matching (e.g., 'laptop' → 'notebook')
- Spell correction
- Autocomplete suggestions
- Search result ranking by relevance

**Filter Options:**
- Price range
- Category/subcategory
- Brand
- Color
- Size
- Rating
- Availability
- Custom attributes

---


## 8. Inventory Management

### 8.1 Inventory Tracking

```typescript
interface InventoryLevel {
  sku: string;
  locationId: string;

  // Quantities
  availableQuantity: number;    // Available for sale
  reservedQuantity: number;     // In carts/pending orders
  inTransitQuantity: number;    // Incoming shipments
  damagedQuantity: number;      // Damaged/unsellable

  // Thresholds
  reorderPoint: number;         // Trigger reorder
  reorderQuantity: number;      // Amount to reorder
  maxStockLevel: number;        // Maximum to stock

  // Metadata
  lastStockCheck?: Date;
  lastRestocked?: Date;
  bin Location?: string;        // Warehouse location
}
```

### 8.2 Stock Movements

```typescript
interface StockMovement {
  id: string;
  sku: string;
  locationId: string;

  // Movement details
  type: 'purchase' | 'sale' | 'return' | 'adjustment' |
        'transfer' | 'damaged' | 'theft' | 'cycle_count';
  quantity: number;             // Positive or negative

  // References
  referenceId?: string;         // Order ID, PO, etc.
  fromLocation?: string;        // For transfers
  toLocation?: string;

  // Tracking
  employeeId?: string;
  notes?: string;
  timestamp: Date;
}
```

### 8.3 Inventory Calculations

**Available to Sell:**
```
Available = On Hand - Reserved - Damaged

Example:
On Hand: 100 units
Reserved: 15 units (in carts)
Damaged: 2 units
Available = 100 - 15 - 2 = 83 units
```

**Inventory Turnover Ratio:**
```
Inventory Turnover = Cost of Goods Sold / Average Inventory Value

Example:
Annual COGS: $500,000
Average Inventory: $100,000
Turnover = $500,000 / $100,000 = 5.0

This means inventory is sold and replaced 5 times per year.
```

**Days Inventory Outstanding (DIO):**
```
DIO = 365 / Inventory Turnover

Example:
Inventory Turnover: 5.0
DIO = 365 / 5.0 = 73 days

On average, inventory sits for 73 days before being sold.
```

**Economic Order Quantity (EOQ):**
```
EOQ = √((2 × D × S) / H)

Where:
D = Annual demand
S = Order cost per order
H = Holding cost per unit per year

Example:
Annual demand: 10,000 units
Order cost: $100
Holding cost: $2 per unit
EOQ = √((2 × 10,000 × 100) / 2)
EOQ = √(1,000,000)
EOQ = 1,000 units
```

### 8.4 Multi-Location Inventory

```typescript
interface InventoryAllocation {
  sku: string;
  totalAvailable: number;
  locations: LocationInventory[];
}

interface LocationInventory {
  locationId: string;
  locationName: string;
  locationType: 'store' | 'warehouse' | 'supplier';
  available: number;
  reserved: number;
  inTransit: number;
  distanceFromCustomer?: number; // km
  shippingTime?: number;         // days
}
```

**Allocation Strategy:**
```
1. Check customer's preferred store
2. If insufficient stock, find nearest location with stock
3. If no nearby stock, check warehouse
4. If multiple options, optimize for:
   - Fastest delivery
   - Lowest shipping cost
   - Best inventory distribution
```

### 8.5 Stockouts and Backorders

**Stockout Handling:**
```typescript
interface StockoutPolicy {
  sku: string;
  policy: 'deny' | 'backorder' | 'notify';

  // For backorder policy
  maxBackorderQuantity?: number;
  expectedRestockDate?: Date;
  notifyOnRestock: boolean;

  // Alternative actions
  suggestAlternatives: boolean;
  alternativeSkus?: string[];
}
```

---



## A.1 Endpoint reference

```http
POST /retail/v1/pos/transaction       # record a POS transaction
POST /retail/v1/order/place           # place an e-commerce order
GET  /retail/v1/inventory/{sku}       # query inventory by SKU
PUT  /retail/v1/inventory/{sku}       # update inventory state
POST /retail/v1/loyalty/earn          # record a loyalty-program event
GET  /retail/v1/customer/{id}/history # query customer purchase history
```

Every endpoint follows the discovery convention at
`/.well-known/wia-retail-tech`.

## A.2 CRM endpoint

The CRM endpoint reads and writes customer profiles with a
PII-classification annotation. Sensitive fields (date of birth,
phone number, address) are encrypted at rest with the standard's
sealed-data envelope; only authorised retail staff with a
documented need-to-know can decrypt.

## A.3 Loyalty endpoint

Loyalty programs run on a points-economy with documented earn and
burn rates. The endpoint records every loyalty event with the
attributable transaction so accountants can reconcile the program's
liability against the float on the books.

## A.4 Product Information Management endpoint

PIM endpoints serve canonical product attributes (title,
description, dimensions, materials, certifications) with i18n
support for multilingual catalogues. The endpoint returns the
locale-appropriate variant per BCP 47 language tag.


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
