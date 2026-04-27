# PHASE 1 — Data Format

> Restaurant-tech canonical envelopes: order, reservation,
> kitchen-ticket, inventory-item, customer, payment, and the
> standard JSON/CSV interchange shapes. All envelopes are signed
> with Ed25519 over the canonical JSON form (RFC 8785 JCS) and
> versioned via the `wia_restaurant_version` field so older POS
> kiosks can validate-and-skip unknown extensions.

## 1.1 Scope and terminology

This phase defines the persistent shapes that POS, KDS,
reservation desk, inventory, payment, and customer-analytics
subsystems exchange. The shapes are deliberately decoupled from
the wire protocol (Phase 2) and the federation protocol (Phase 3)
so that the same envelope can be persisted, signed, and replayed
without re-encoding.

Industry abbreviations used throughout the data format:

- **POS** — Point of Sale (order/payment terminal)
- **KDS** — Kitchen Display System (ticket display for line)
- **Cover** — one customer/guest (industry standard unit)
- **Table Turn** — complete service cycle for one party
- **RevPASH** — Revenue Per Available Seat Hour
- **Par Level** — minimum inventory quantity to maintain
- **86'd** — item temporarily unavailable
- **Ticket Time** — duration from order placed to food ready
- **COGS** — Cost of Goods Sold
- **Prime Cost** — combined food + labor cost
- **FOH / BOH** — Front of House / Back of House

## 1.2 Order envelope

The `order` envelope is the canonical document persisted at every
POS terminal and replicated to chain HQ in Phase 3. Every field
listed is required at envelope sign time except those marked
optional (`?:`); optional fields MUST be omitted (not null) when
not present so that the canonical-JSON signature is stable.

```typescript
enum OrderType {
  DINE_IN = 'dine_in',
  TAKEOUT = 'takeout',
  DELIVERY = 'delivery',
  CURBSIDE = 'curbside',
  DRIVE_THROUGH = 'drive_through',
  CATERING = 'catering',
  BAR = 'bar'
}

interface Order {
  orderId: string;
  orderNumber: number;        // sequential daily number
  orderType: OrderType;
  status: OrderStatus;
  tableNumber?: number;
  serverId: string;
  serverName: string;
  createdAt: Date;
  modifiedAt: Date;
  items: OrderItem[];
  subtotal: number;
  discounts: Discount[];
  tax: number;
  tip: number;
  total: number;
  paymentStatus: PaymentStatus;
  specialInstructions?: string;
  courseTiming?: CourseTiming[];
  guestCount?: number;
}

interface OrderItem {
  itemId: string;
  menuItemId: string;
  name: string;
  category: string;
  quantity: number;
  unitPrice: number;
  modifiers: Modifier[];
  specialRequests?: string;
  station: KitchenStation;
  courseNumber?: number;
  seat?: number;
  status: ItemStatus;
  prepTime: number;          // estimated minutes
  firedAt?: Date;
  readyAt?: Date;
}

interface Modifier {
  modifierId: string;
  name: string;
  type: 'add' | 'remove' | 'substitute' | 'portion' | 'preparation';
  priceAdjustment: number;   // signed; negative for remove
  affectsCost: boolean;
}

interface Discount {
  discountId: string;
  name: string;
  type: 'percentage' | 'fixed_amount' | 'happy_hour'
      | 'loyalty' | 'employee' | 'comp' | 'promotional';
  value: number;
  applicableItems?: string[];
  requiresManagerApproval: boolean;
  reason?: string;
}
```

Total computation MUST be reproducible from the items+discounts
fields; `subtotal`, `tax`, `tip`, `total` are persisted for audit
and reconciliation, not as the primary source of truth.

## 1.3 Reservation envelope

```typescript
interface Reservation {
  reservationId: string;
  confirmationCode: string;
  customerId?: string;
  customerName: string;
  partySize: number;
  dateTime: Date;
  duration: number;          // expected minutes
  status: 'pending' | 'confirmed' | 'seated' | 'completed'
        | 'cancelled' | 'no_show' | 'waitlist';
  tablePreference?: string;
  seatingArea?: string;      // 'indoor' | 'outdoor' | 'bar' | etc.
  occasion?: string;
  specialRequests?: string;
  dietaryRestrictions?: string[];
  contactPhone: string;
  contactEmail: string;
  createdAt: Date;
  createdBy: string;         // staff id or 'online'
  deposit?: number;
  noShowHistory?: number;
  vipStatus?: boolean;
}

interface WaitlistEntry {
  entryId: string;
  customerName: string;
  partySize: number;
  contactPhone: string;
  addedAt: Date;
  quotedWait: number;
  actualWait?: number;
  status: 'waiting' | 'notified' | 'seated' | 'cancelled' | 'no_show';
  priority: number;
  notified: boolean;
  notifiedAt?: Date;
  specialRequests?: string;
}
```

## 1.4 Kitchen ticket envelope

```typescript
interface KitchenTicket {
  ticketId: string;
  orderNumber: number;
  tableNumber?: number;
  serverName: string;
  items: KitchenItem[];
  priority: 'low' | 'normal' | 'high' | 'urgent' | 'vip';
  status: TicketStatus;
  firedAt: Date;
  dueTime?: Date;
  elapsedTime: number;       // seconds since fired
  specialInstructions?: string;
}

interface KitchenItem {
  name: string;
  quantity: number;
  modifiers: string[];       // flattened textual list for line cooks
  seat?: number;
  status: 'pending' | 'in_progress' | 'ready' | 'completed' | 'recalled';
  startedAt?: Date;
  completedAt?: Date;
}

interface CourseTiming {
  courseNumber: number;
  items: string[];
  fireTime: Date;
  targetReadyTime: Date;
  previousCourse?: number;
  minimumGap: number;        // minutes between courses
}
```

`KitchenTicket` is derived from `Order` at fire time — the ticket
is the snapshot the kitchen line cooks see, while the originating
`Order` continues to evolve as the table modifies, voids, and
adds.

## 1.5 Inventory item envelope

```typescript
interface InventoryItem {
  itemId: string;
  name: string;
  category: 'produce' | 'meat' | 'seafood' | 'dairy' | 'dry_goods'
          | 'beverages' | 'alcohol' | 'supplies' | 'cleaning';
  unit: 'each' | 'lb' | 'kg' | 'oz' | 'g' | 'l' | 'gal' | 'case' | 'box';
  currentQuantity: number;
  parLevel: number;          // minimum to maintain
  maxLevel: number;
  reorderPoint: number;
  reorderQuantity: number;
  unitCost: number;
  supplier: string;
  supplierId: string;
  shelfLife?: number;        // days to expiry
  storageLocation: string;
  trackByLot: boolean;
  trackBySerial: boolean;
}

interface StockMovement {
  movementId: string;
  itemId: string;
  type: 'received' | 'used' | 'wasted' | 'sold' | 'transferred' | 'adjustment';
  quantity: number;
  unit: string;
  timestamp: Date;
  userId: string;
  reference?: string;        // PO id, order id, etc.
  cost?: number;
  notes?: string;
}
```

## 1.6 Customer envelope

```typescript
interface Customer {
  customerId: string;
  firstName: string;
  lastName: string;
  email: string;
  phone: string;
  dateOfBirth?: Date;
  joinDate: Date;
  totalVisits: number;
  totalSpent: number;
  averageCheckSize: number;
  lastVisit: Date;
  favoriteItems: string[];
  dietaryRestrictions: string[];
  allergens: string[];
  vipStatus: boolean;
  loyaltyPoints: number;
  segment: 'new' | 'occasional' | 'regular' | 'loyal'
         | 'vip' | 'at_risk' | 'lost';
  communicationPreferences: {
    email: boolean;
    sms: boolean;
    phone: boolean;
  };
}
```

The `segment` field is a derived projection; producers MAY recompute
on each visit, but the canonical signed envelope MUST carry the
projection at envelope-time so retroactive comparisons are stable.

## 1.7 Payment envelope

```typescript
interface Payment {
  paymentId: string;
  orderId: string;
  method: 'cash' | 'credit_card' | 'debit_card' | 'gift_card'
        | 'mobile_payment' | 'cryptocurrency'
        | 'account_charge' | 'comp';
  amount: number;
  tipAmount: number;
  totalAmount: number;
  status: 'pending' | 'authorized' | 'captured'
        | 'refunded' | 'failed' | 'cancelled';
  processedAt: Date;
  processorReference?: string;
  last4?: string;
  cardBrand?: string;
  authCode?: string;
}

interface SplitPayment {
  orderId: string;
  totalAmount: number;
  splits: PaymentSplit[];
  status: 'in_progress' | 'complete' | 'partial';
}

interface PaymentSplit {
  splitId: string;
  customerId?: string;
  amount: number;
  tipAmount: number;
  method: string;            // PaymentMethod enum
  items?: string[];
  status: string;            // PaymentStatus enum
}
```

Per PCI DSS, full PAN MUST NOT be persisted in the envelope; the
`last4` and `cardBrand` are the only card-derived values allowed.
Tokenized card references live in the payment processor's vault
and are linked through `processorReference`.

## 1.8 Standard JSON exchange

The canonical wire shape for an order on the public REST surface:

```json
{
  "order": {
    "orderId": "ord_abc123",
    "orderNumber": 42,
    "orderType": "dine_in",
    "tableNumber": 12,
    "server": { "id": "srv_001", "name": "Jane Smith" },
    "items": [
      {
        "itemId": "item_001",
        "name": "Grilled Salmon",
        "quantity": 1,
        "unitPrice": 24.99,
        "modifiers": [
          { "name": "no lemon",          "price": 0    },
          { "name": "extra vegetables",  "price": 2.00 }
        ],
        "subtotal": 26.99
      }
    ],
    "totals": {
      "subtotal": 26.99,
      "tax":       2.16,
      "tip":       4.86,
      "total":    34.01
    },
    "timestamp": "2025-12-27T19:30:00Z"
  }
}
```

All monetary values are decimal, in the establishment's locale
currency (declared once in the location envelope of Phase 3),
serialized with two fractional digits regardless of currency.

## 1.9 CSV export shape

```csv
Date,Order ID,Order Type,Table,Server,Items,Subtotal,Tax,Tip,Total,Payment Method
2025-12-27,ord_001,dine_in,12,Jane Smith,2,45.50,3.64,8.19,57.33,credit_card
```

The CSV is a downstream projection used for accounting export
(QuickBooks, Xero); it is NOT a signed envelope and MUST NOT be
relied on for audit. The signed `order` envelopes remain the
authoritative record.

## 1.10 Canonical signing rule

Implementations MUST canonicalize every envelope per RFC 8785 JCS
before applying the Ed25519 signature. The signature field
(`signature.value`) is excluded from the canonical form. Signers
SHOULD use a per-establishment signing key; chain HQ MAY
counter-sign for cross-location replay (see Phase 3).

弘益人間 — the data format is the contract; everything else is
implementation.

## 1.11 Implementer notes

### 1.11.1 Currency, locale, and rounding

Restaurants are unusually exposed to rounding bugs because every
order touches three multiplicative steps (modifier rollup, tax,
tip) and a final addition. Implementations MUST settle the
following questions at envelope creation, not at display time:

- **Currency selection** — declared once on the location envelope
  in Phase 3 §3.4.1; orders MUST NOT carry a currency field
  separate from their location.
- **Decimal precision** — two fractional digits regardless of
  ISO 4217 minor unit (Korean Won and Japanese Yen are stored
  with two zeros after the decimal so the canonical-JSON shape
  is uniform across all locales; the display layer formats out).
- **Rounding mode** — banker's rounding (half-to-even) on every
  individual computation; the Phase-2 preview endpoint MUST
  produce the same result the kitchen-side POS will produce on
  commit, byte-for-byte.

### 1.11.2 ID format

All `*Id` fields use ULID; sequential `orderNumber` is for
human-facing receipts and resets at the end of the business day.
ULIDs are generated client-side at the POS terminal so offline
operation works without coordinator round-trip.

### 1.11.3 Time and timezone

All `Date` fields are RFC 3339 timestamps in UTC at envelope
sign time. The location envelope (Phase 3) carries the timezone
that the front-of-house staff and the receipt printer should
project into. The "business day" boundary is the location's
4 AM local time per common restaurant convention; reports query
in business-day terms.

### 1.11.4 Schema evolution

The `wia_restaurant_version` field opens an additive evolution
path. Producers MAY emit additional fields beyond those listed
above; consumers MUST validate the known fields and ignore
unknown ones (the canonical-JSON form preserves them, so
signatures remain stable through forward-compat upgrades).

### 1.11.5 Personally identifiable information

The customer envelope (§1.6) and the reservation envelope (§1.3)
both carry PII. The signed envelope MUST be encrypted at rest
(AES-256-GCM with a per-establishment key); raw signed bytes
MUST NOT leave the establishment without explicit consent
captured per the privacy regime in Phase 4 §4.5.2.
