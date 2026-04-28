# WIA-IND-020 — Phase 1: Data Format

> Retail-tech canonical envelopes: POS transaction, e-commerce order, omnichannel inventory state, and the runtime conventions that fix the wire format for every retail protocol below.

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for retail technology systems, providing standardized interfaces and protocols for modern retail operations across physical stores, e-commerce platforms, and omnichannel environments.

### 1.2 Scope

The standard covers:
- Point of Sale (POS) transaction processing
- E-commerce platform integration
- Omnichannel retail experiences
- Customer data and relationship management
- Loyalty program operations
- Product information and catalog management
- Real-time inventory tracking
- Dynamic pricing and promotions
- Payment processing and security
- Returns and refund workflows
- Business analytics and reporting

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to create seamless, efficient, and customer-centric retail experiences that benefit both merchants and consumers through standardized, interoperable technology systems.

### 1.4 Terminology

- **SKU (Stock Keeping Unit)**: Unique identifier for products
- **POS (Point of Sale)**: System for processing retail transactions
- **Omnichannel**: Integrated multi-channel retail approach
- **CLV (Customer Lifetime Value)**: Predicted total revenue from customer
- **BOPIS**: Buy Online, Pick Up In Store
- **Inventory Turnover**: Rate at which inventory is sold and replaced
- **Conversion Rate**: Percentage of visitors who make purchases

---


## 2. Point of Sale (POS) Systems

### 2.1 POS Architecture

Modern POS systems consist of:

```
┌─────────────────────────────────────────┐
│           POS Terminal                   │
├─────────────────────────────────────────┤
│  • Touch Screen Interface               │
│  • Barcode Scanner                      │
│  • Receipt Printer                      │
│  • Card Reader (EMV/NFC)                │
│  • Cash Drawer                          │
│  • Customer Display                     │
└─────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────┐
│      POS Software Layer                  │
├─────────────────────────────────────────┤
│  • Transaction Management               │
│  • Payment Processing                   │
│  • Inventory Updates                    │
│  • Customer Lookup                      │
│  • Reporting                            │
└─────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────┐
│      Backend Systems                     │
├─────────────────────────────────────────┤
│  • Database                             │
│  • Payment Gateway                      │
│  • Inventory Management                 │
│  • CRM System                           │
└─────────────────────────────────────────┘
```

### 2.2 Transaction Processing Flow

```
1. Item Scan/Entry
   ├─> Lookup product in database
   ├─> Verify inventory availability
   ├─> Add to transaction
   └─> Calculate price (including promotions)

2. Apply Discounts
   ├─> Manual discounts
   ├─> Promotion codes
   ├─> Loyalty rewards
   └─> Employee discounts

3. Calculate Totals
   ├─> Subtotal = Sum(item prices × quantities)
   ├─> Tax = Subtotal × Tax Rate
   ├─> Discount = Sum(all discounts)
   └─> Total = Subtotal + Tax - Discount + Shipping

4. Payment Processing
   ├─> Accept payment method
   ├─> Process payment
   ├─> Verify authorization
   └─> Capture payment

5. Transaction Completion
   ├─> Update inventory
   ├─> Record customer purchase
   ├─> Award loyalty points
   ├─> Print/email receipt
   └─> Close transaction
```

### 2.3 Transaction Data Model

```typescript
interface Transaction {
  id: string;                    // Unique transaction ID
  storeId: string;               // Store identifier
  registerId: string;            // POS register ID
  receiptNumber: string;         // Human-readable receipt number

  // Items
  items: TransactionItem[];

  // Customer
  customerId?: string;
  customerEmail?: string;

  // Financial
  subtotal: number;              // Before tax/discounts
  tax: number;                   // Total tax amount
  discount: number;              // Total discount amount
  shipping: number;              // Shipping charges
  total: number;                 // Final total
  currency: string;              // ISO 4217 currency code

  // Payment
  payments: Payment[];

  // Status
  status: 'pending' | 'completed' | 'cancelled' | 'refunded';

  // Metadata
  employeeId?: string;           // Cashier/associate
  channel: 'in_store' | 'online' | 'mobile' | 'kiosk';
  createdAt: Date;
  updatedAt: Date;
}

interface TransactionItem {
  id: string;
  sku: string;                   // Stock keeping unit
  name: string;
  quantity: number;
  price: number;                 // Unit price
  originalPrice?: number;        // Before discounts
  taxRate: number;
  taxAmount: number;
  discountAmount: number;
  total: number;                 // Line total
}
```

### 2.4 POS System Requirements

**Hardware Requirements:**
- Processor: Intel Core i3 or equivalent
- RAM: 4GB minimum, 8GB recommended
- Storage: 128GB SSD minimum
- Network: Gigabit Ethernet or WiFi 5/6
- Peripherals: Barcode scanner, receipt printer, cash drawer

**Software Requirements:**
- Operating System: Windows 10/11, Linux, or iOS
- Database: PostgreSQL, MySQL, or cloud-based
- Network connectivity for payment processing
- Offline mode capability for network outages
- Backup and recovery systems

**Performance Requirements:**
- Transaction processing: < 2 seconds
- Barcode scan response: < 500ms
- Payment authorization: < 5 seconds
- Receipt printing: < 3 seconds
- System uptime: 99.9% minimum

---


## 3. E-commerce Platform Integration

### 3.1 E-commerce Architecture

```
┌─────────────────────────────────────────┐
│        E-commerce Frontend               │
│  • Product Catalog                      │
│  • Shopping Cart                        │
│  • Checkout                             │
│  • Customer Account                     │
└─────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────┐
│      E-commerce Backend API              │
│  • Product Management                   │
│  • Order Processing                     │
│  • Payment Integration                  │
│  • Customer Management                  │
│  • Inventory Sync                       │
└─────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────┐
│      Integration Layer                   │
│  • POS System Sync                      │
│  • Inventory Management                 │
│  • CRM Integration                      │
│  • Analytics Platform                   │
└─────────────────────────────────────────┘
```

### 3.2 RESTful API Endpoints

**Product Endpoints:**
```
GET    /api/v1/products              List products
GET    /api/v1/products/:id          Get product details
POST   /api/v1/products              Create product
PUT    /api/v1/products/:id          Update product
DELETE /api/v1/products/:id          Delete product
GET    /api/v1/products/:id/inventory Inventory levels
```

**Order Endpoints:**
```
GET    /api/v1/orders                List orders
GET    /api/v1/orders/:id            Get order details
POST   /api/v1/orders                Create order
PUT    /api/v1/orders/:id            Update order
DELETE /api/v1/orders/:id            Cancel order
POST   /api/v1/orders/:id/fulfill    Fulfill order
```

**Customer Endpoints:**
```
GET    /api/v1/customers             List customers
GET    /api/v1/customers/:id         Get customer details
POST   /api/v1/customers             Create customer
PUT    /api/v1/customers/:id         Update customer
GET    /api/v1/customers/:id/orders  Customer orders
GET    /api/v1/customers/:id/loyalty Loyalty information
```

### 3.3 Shopping Cart Management

```typescript
interface ShoppingCart {
  id: string;
  customerId?: string;
  sessionId: string;
  items: CartItem[];
  subtotal: number;
  tax: number;
  shipping: number;
  discount: number;
  total: number;
  currency: string;
  expiresAt: Date;
  createdAt: Date;
  updatedAt: Date;
}

interface CartItem {
  id: string;
  sku: string;
  name: string;
  quantity: number;
  price: number;
  imageUrl?: string;
  attributes?: Record<string, string>;
}
```

### 3.4 Checkout Process

```
1. Cart Review
   ├─> Display cart items
   ├─> Apply discount codes
   └─> Calculate totals

2. Customer Information
   ├─> Login or guest checkout
   ├─> Shipping address
   └─> Billing address

3. Shipping Method
   ├─> Standard shipping
   ├─> Express shipping
   ├─> Free shipping (if eligible)
   └─> Pick up in store

4. Payment
   ├─> Enter payment details
   ├─> Apply gift cards
   └─> Process payment

5. Order Confirmation
   ├─> Generate order number
   ├─> Send confirmation email
   ├─> Update inventory
   └─> Award loyalty points
```

---


## 4. Omnichannel Retail

### 4.1 Omnichannel Strategy

Omnichannel retail provides seamless customer experience across all channels:

**Key Principles:**
1. **Unified Inventory**: Real-time visibility across all channels
2. **Consistent Pricing**: Same prices online and in-store
3. **Flexible Fulfillment**: BOPIS, ship from store, home delivery
4. **Single Customer View**: Unified customer data across channels
5. **Channel-Agnostic Returns**: Return anywhere regardless of purchase channel

### 4.2 Buy Online, Pick Up In Store (BOPIS)

```typescript
interface BOPISOrder {
  orderId: string;
  customerId: string;
  items: OrderItem[];
  pickupStore: string;
  pickupTime: Date;
  notificationPreference: 'email' | 'sms' | 'both';
  status: 'pending' | 'ready' | 'picked_up' | 'cancelled';
  preparationTime: number;      // minutes
  readyAt?: Date;
  pickedUpAt?: Date;
}
```

**BOPIS Workflow:**
```
1. Customer places order online
   └─> Select "Pick up in store" option
   └─> Choose preferred store and pickup time

2. Order routing
   └─> Check inventory at selected store
   └─> If unavailable, offer alternative stores
   └─> Create BOPIS order

3. Store preparation
   └─> Notify store staff
   └─> Pick items from inventory
   └─> Package for pickup
   └─> Mark as ready

4. Customer notification
   └─> Send pickup ready notification
   └─> Provide pickup instructions
   └─> Include QR code for easy pickup

5. Pickup
   └─> Customer arrives at store
   └─> Verify identity (QR code or ID)
   └─> Hand over order
   └─> Mark as picked up
```

### 4.3 Ship From Store

Ship from store turns retail locations into micro-fulfillment centers:

```typescript
interface ShipFromStoreOrder {
  orderId: string;
  fulfillmentStore: string;
  shippingAddress: Address;
  shippingMethod: 'standard' | 'express' | 'overnight';
  trackingNumber?: string;
  carrier?: string;
  estimatedDelivery: Date;
  status: 'pending' | 'picked' | 'packed' | 'shipped' | 'delivered';
}
```

**Benefits:**
- Faster delivery from nearby stores
- Reduced shipping costs
- Better inventory utilization
- Lower warehouse requirements

### 4.4 Endless Aisle

Allow in-store customers to purchase items not in stock:

```
1. Customer browses in-store
   └─> Item not available in current store

2. Associate checks inventory
   └─> Look up in endless aisle system
   └─> Show availability at other stores or online

3. Customer places order
   └─> Ship to home or other store
   └─> Process payment in-store
   └─> Receive order confirmation

4. Order fulfillment
   └─> Route to nearest fulfillment location
   └─> Ship to customer
```

---



## A.1 Canonical envelope conventions

Every Phase 1 retail envelope follows the WIA family baseline:
UTF-8 JSON, RFC 8785 canonical form, Ed25519 signatures, ULID
identifiers. Currency amounts use ISO 4217 codes with values in
minor units (cents, won, sen) to avoid floating-point drift.

## A.2 POS transaction envelope

```json
{
  "wia_retail_version": "1.0.0",
  "type": "pos_transaction",
  "transaction_id": "tx_01HX...",
  "store_id": "store_01HX...",
  "register_id": "reg_07",
  "items": [
    { "sku": "...", "qty": 2, "unit_price_minor": 12500, "currency": "KRW", "tax_rate": 0.10 }
  ],
  "subtotal_minor": 25000,
  "tax_minor": 2500,
  "total_minor": 27500,
  "tendered": [{ "method": "card", "scheme": "visa", "amount_minor": 27500 }],
  "issued_at": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## A.3 E-commerce order envelope

E-commerce orders extend the POS shape with shipping address, ship
method, and fulfilment status. The envelope is signed by the
e-commerce platform; the fulfilment partner re-signs as the order
moves through the supply chain.

## A.4 Omnichannel inventory state

Omnichannel inventory state carries per-SKU per-location quantity
plus the reservation state (reserved by orders not yet shipped).
The envelope is reconciled across channels every minute so the
e-commerce site doesn't oversell stock that POS sold a moment ago.


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
