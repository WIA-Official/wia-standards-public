# PHASE 2 — API

> Restaurant-tech REST surface and the deterministic computations
> that an API consumer is entitled to assume. Every endpoint
> accepts and returns the Phase-1 envelopes; this phase covers
> verb semantics, idempotency, error model, and the algorithmic
> guarantees behind pricing, availability, ticket timing, and
> kitchen routing.

## 2.1 REST endpoints

The base path is `/api/v1` and all endpoints accept and emit
`application/json` per Phase 1.

```
POST   /api/orders                     # create order
GET    /api/orders/:id                 # read order
PUT    /api/orders/:id                 # update order (idempotent on hash)
DELETE /api/orders/:id                 # void order (manager auth required)

POST   /api/reservations               # create reservation
GET    /api/reservations/:id           # read reservation
PUT    /api/reservations/:id           # update reservation
GET    /api/reservations/availability  # check availability window

GET    /api/menu                       # current menu (location-scoped)
GET    /api/inventory                  # current inventory snapshot
POST   /api/inventory/adjust           # apply StockMovement

POST   /api/tickets/:id/bump           # mark item complete
POST   /api/tickets/:id/recall         # remake an item

POST   /api/payments                   # create Payment
POST   /api/payments/:id/capture       # capture authorized
POST   /api/payments/:id/refund        # refund captured

GET    /api/reports/sales              # sales reports
GET    /api/reports/labor              # labor reports
GET    /api/analytics/customer/:id     # customer analytics
```

Idempotency: every `POST` accepts an `Idempotency-Key` header.
When provided, repeated requests with the same key MUST return
the original response unchanged for at least 24 hours.

## 2.2 Pricing calculation

The `POST /api/orders/preview` endpoint accepts a draft order
shape and returns the totals the same way the kitchen-side POS
will compute them on commit. The contract:

```
Subtotal       = Σ(unitPrice × quantity + Σ modifier.priceAdjustment)
DiscountAmount = applyDiscounts(Subtotal, discounts[])
Taxable        = Subtotal − DiscountAmount
Tax            = Taxable × jurisdictionTaxRate(orderType, items)
SuggestedTip   = Subtotal × tipPercent
Total          = Subtotal − DiscountAmount + Tax + Tip
```

Tax rates differ by jurisdiction and item class:

```typescript
interface TaxConfiguration {
  jurisdiction: string;
  foodTaxRate: number;
  alcoholTaxRate: number;
  retailTaxRate: number;
  cateringTaxRate?: number;
  deliveryTaxRate?: number;
  taxOnTax?: boolean;        // some jurisdictions tax tax
}
```

Suggested tip endpoint:

```typescript
function calculateSuggestedTips(subtotal: number) {
  return {
    subtotal,
    suggestedTips: {
      percent15: Math.round(subtotal * 0.15 * 100) / 100,
      percent18: Math.round(subtotal * 0.18 * 100) / 100,
      percent20: Math.round(subtotal * 0.20 * 100) / 100,
      percent25: Math.round(subtotal * 0.25 * 100) / 100
    }
  };
}
```

## 2.3 Table management

```typescript
interface Table {
  tableId: string;
  tableNumber: number;
  section: string;
  capacity: number;
  currentPartySize?: number;
  status: 'available' | 'occupied' | 'reserved'
        | 'dirty' | 'cleaning' | 'maintenance';
  serverId?: string;
  serverName?: string;
  seatedAt?: Date;
  estimatedDuration: number; // minutes
  shape: TableShape;
  combinable: boolean;
  position: Coordinates;
}

interface Section {
  sectionId: string;
  name: string;
  tables: string[];
  serverId?: string;
  capacity: number;
  status: SectionStatus;
  openTime?: Date;
  closeTime?: Date;
}
```

Turnover is derived (not persisted):

```
TurnoverRate = PartiesServed / (NumTables × OperatingHours)

Example: 25 tables × 8 hours, 85 parties served
       = 85 / 200 = 0.425 parties / table / hour
       ≈ 141 minutes average per party (60 / 0.425)
```

## 2.4 Reservation availability

```typescript
function checkAvailability(check: AvailabilityCheck): AvailabilityResult {
  const { date, time, partySize, duration = 90 } = check;

  const suitable = findSuitableTables(partySize);

  const available = suitable.filter(table => {
    const startTime = time;
    const endTime = new Date(time.getTime() + duration * 60_000);
    return !hasConflict(table, startTime, endTime);
  });

  return {
    available: available.length > 0,
    suggestedTimes: available.length === 0 ? findAlternativeTimes(check) : [],
    availableTables: available
  };
}
```

Allocation strategies are advisory parameters on the request:

```typescript
interface AllocationStrategy {
  strategy: 'first_available' | 'optimize_turnover' | 'section_balance';
  preferences: {
    preferredSection?: string;
    preferredTable?: number;
    windowSeating?: boolean;
    quietArea?: boolean;
    accessibility?: boolean;
  };
}
```

The allocation function considers party-size-vs-capacity (avoid
oversizing), server workload balance, customer preferences,
section rotation, and VIP status before selecting the optimal
table.

## 2.5 Wait-time estimation

```typescript
function estimateWaitTime(partySize: number): number {
  const turnoverRate    = calculateCurrentTurnover();
  const partiesAhead    = getWaitlistCount(partySize);
  const avgTableTime    = getAverageTableTime(partySize);

  const estimate = Math.ceil(
    (partiesAhead + 1) * avgTableTime / turnoverRate
  );

  // 15% buffer (industry-standard "underpromise, overdeliver")
  return Math.ceil(estimate * 1.15);
}
```

The endpoint returns the estimate AND the buffered figure. The
quoted-vs-actual gap MUST be persisted on the WaitlistEntry on
seating so the establishment can self-tune.

## 2.6 Kitchen ticket timing

```typescript
function calculateAverageTicketTime(
  station: string,
  period: 'last_hour' | 'today' | 'last_week'
): number {
  const tickets = getCompletedTickets(station, period);
  const total = tickets.reduce((sum, t) =>
    sum + (t.completedAt.getTime() - t.firedAt.getTime()), 0);
  return total / tickets.length / 60_000; // minutes
}
```

Course coordination works backwards from desired service time:

```typescript
function coordinateCourses(
  order: Order,
  courses: CourseTiming[]
): FireSchedule {
  const schedule: FireSchedule = {};

  for (let i = courses.length - 1; i >= 0; i--) {
    const course   = courses[i];
    const prepMin  = calculateCoursePrepTime(course.items);

    if (i === courses.length - 1) {
      schedule[course.courseNumber] = {
        fireAt: new Date(course.targetReadyTime.getTime() - prepMin * 60_000)
      };
    } else {
      const next = courses[i + 1];
      const fireAt = new Date(
        schedule[next.courseNumber].fireAt.getTime()
        - course.minimumGap * 60_000
        - prepMin * 60_000
      );
      schedule[course.courseNumber] = { fireAt };
    }
  }

  return schedule;
}
```

## 2.7 Bump and recall

`POST /api/tickets/:ticketId/bump` marks an item complete:

```typescript
function bumpItem(ticketId: string, itemId: string): void {
  updateItemStatus(itemId, ItemStatus.COMPLETED);

  const ticket = getTicket(ticketId);
  const allComplete = ticket.items.every(i => i.status === 'completed');

  if (allComplete) {
    removeTicket(ticketId);
    notifyTicketReady(ticketId);
    recordTicketMetrics(ticket);
  }
}
```

`POST /api/tickets/:ticketId/recall` returns an item to the line
with elevated priority:

```typescript
interface RecallReason {
  reason: 'wrong_item' | 'quality_issue' | 'customer_change'
        | 'dropped' | 'other';
  description?: string;
  remakePriority: 'low' | 'normal' | 'high' | 'urgent' | 'vip';
}

function recallItem(ticketId: string, itemId: string, reason: RecallReason) {
  const original = getTicket(ticketId);
  const item     = original.items.find(i => i.itemId === itemId);

  const recall = createTicket({
    ...original,
    ticketId: generateId(),
    items: [item],
    priority: reason.remakePriority,
    isRecall: true,
    recallReason: reason
  });

  recordRecall(original, item, reason);
}
```

## 2.8 KDS color-coding API

```typescript
interface ColorCodingScheme {
  under5min:   string;       // green
  under10min:  string;       // yellow
  under15min:  string;       // orange
  over15min:   string;       // red

  vip:         string;       // purple
  urgent:      string;       // red flash

  inProgress:  string;
  ready:       string;
  recalled:    string;
}

interface KDSDisplay {
  displayId: string;
  station: KitchenStation;
  screenSize: 'small' | 'medium' | 'large';
  columns: number;
  maxTickets: number;
  sortOrder: 'fifo' | 'priority' | 'course_timing' | 'table_number';
  colorCoding: ColorCodingScheme;
  soundAlerts: boolean;
  language: string;
}
```

The KDS pulls tickets by long-poll (`GET /api/tickets/stream`)
and pushes status changes back via the bump/recall endpoints.

## 2.9 Error model

Every endpoint MUST return a structured error body on non-2xx:

```json
{
  "error": {
    "code": "ORDER_TICKET_LOCKED",
    "message": "Ticket is currently being modified by another terminal",
    "retryable": true,
    "retryAfterMs": 250
  }
}
```

The error codes form a closed enumeration so client SDKs can
branch deterministically; new codes MUST be added in a minor
version bump only.

## 2.10 Authentication

All endpoints require a per-terminal bearer token issued by the
location's auth service. Bearer scope follows the staff role
matrix declared in Phase 3 (manager-only endpoints reject lower
roles with `code: AUTH_INSUFFICIENT_ROLE`). Mutating endpoints
also require an X-Idempotency-Key for safe retries on flaky
networks (universally common in restaurant POS deployments).

## 2.11 Rate limiting and concurrency

The API surface is consumed by both fixed POS terminals (low
volume, high reliability) and mobile servers carrying tablets
(high volume, network-flaky). The contract:

- **Per-terminal limit**: 30 requests/second sustained burst,
  600 requests/minute rolling. Limit headers (`X-RateLimit-*`)
  are returned on every response.
- **Concurrency on a single ticket**: at most one writer at a
  time; subsequent writers receive `code: ORDER_TICKET_LOCKED`
  with `retryAfterMs` populated for predictable back-off.
- **Reservation-availability calls**: opaquely cached server-side
  for 30 seconds (the customer-facing kiosk doesn't need
  per-request freshness).

## 2.12 Webhook contract

Outbound webhooks deliver six event classes:

```
order.created        order.modified       order.voided
ticket.fired         ticket.bumped        ticket.recalled
payment.captured     payment.refunded
reservation.created  reservation.cancelled
inventory.below_par  shift.swap_requested
```

Every webhook payload is itself a Phase-1 signed envelope with
an `event` wrapper carrying the event class, the producing
terminal, and an Ed25519 signature. Receivers MUST verify the
signature before acting; replay protection is the receiver's
responsibility (a 5-minute deduplication window over `eventId`
is sufficient in practice).

Retry policy: exponential back-off with jitter, 6 attempts,
maximum 1 hour total elapsed. After the 6th failure the event
is parked in a dead-letter queue accessible via
`GET /api/webhooks/dlq` for manual replay by chain ops.

## 2.13 Versioning and deprecation

The API surface is versioned by URL prefix (`/api/v1`). Backward-
incompatible changes go in a new prefix; backward-compatible
additions land in the current prefix and are announced via the
`API-Deprecation` response header at least six months ahead of
removal. Webhook event names follow the same evolution rule —
existing event consumers SHOULD subscribe to a wildcard suffix
pattern so additions don't require code changes.
