# PHASE 3 — Protocol / Federation

> Restaurant-tech operational protocols: kitchen routing, staff
> scheduling, inventory replenishment, and the cross-location
> federation that lets a chain HQ aggregate, standardize, and
> arbitrate across many establishments without dictating their
> moment-to-moment service.

## 3.1 Kitchen station routing

Each kitchen is modelled as a graph of stations:

```typescript
interface KitchenStation {
  stationId: string;
  name: string;
  type: 'grill' | 'saute' | 'fry' | 'cold_prep'
      | 'salad' | 'dessert' | 'bar' | 'expo';
  capacity: number;          // concurrent orders
  currentLoad: number;
  averageTicketTime: number; // minutes
  staff: string[];
  equipment: string[];
}
```

Routing partitions an order's items by station and re-balances
for capacity:

```typescript
function routeOrderItems(order: Order): Map<string, OrderItem[]> {
  const routing = new Map<string, OrderItem[]>();

  for (const item of order.items) {
    const station = determineStation(item);
    if (!routing.has(station)) routing.set(station, []);
    routing.get(station)!.push(item);
  }

  return optimizeRouting(routing, order.courseTiming);
}
```

The expo station is special: it acts as a join-point that fires
to the server when every other station has bumped its half of
the ticket. Course coordination from Phase 2 §2.6 informs the
fire times that this protocol uses.

## 3.2 Staff scheduling protocol

### 3.2.1 Coverage requirements

```typescript
interface CoverageRequirement {
  dayOfWeek: number;         // 0–6 (Sun–Sat)
  timeSlot: TimeSlot;
  role: StaffRole;
  minimumStaff: number;
  optimalStaff: number;
  skillLevel: SkillLevel;
}

interface TimeSlot {
  start: string;             // HH:MM
  end: string;
  expectedCovers: number;
  coverageMultiplier: number; // servers per X covers
}

enum StaffRole {
  SERVER = 'server',
  BARTENDER = 'bartender',
  HOST = 'host',
  COOK = 'cook',
  PREP_COOK = 'prep_cook',
  DISHWASHER = 'dishwasher',
  BUSSER = 'busser',
  MANAGER = 'manager',
  SOMMELIER = 'sommelier'
}

enum SkillLevel {
  TRAINEE = 'trainee',
  JUNIOR = 'junior',
  EXPERIENCED = 'experienced',
  SENIOR = 'senior',
  LEAD = 'lead'
}
```

### 3.2.2 Schedule optimization

```typescript
function optimizeSchedule(reqs: ScheduleRequirements): Schedule {
  const required = calculateRequiredHours(reqs.expectedCovers);

  const candidates = generateScheduleCandidates(
    reqs.availableStaff, required, reqs.constraints
  );

  const scored = candidates.map(s => ({
    schedule: s,
    score: scoreSchedule(s, {
      laborCostTarget: reqs.laborCostTarget,
      coverageRequirements: required,
      staffPreferences: reqs.availableStaff.map(x => x.preferences)
    })
  }));

  return scored.sort((a, b) => b.score - a.score)[0].schedule;
}

function scoreSchedule(s: Schedule, c: ScoringCriteria): number {
  let score = 0;
  const laborCostScore = 1 - Math.abs(s.laborPercent - c.laborCostTarget) / 100;
  score += laborCostScore * 0.4;
  score += evaluateCoverage(s, c.coverageRequirements) * 0.4;
  score += evaluatePreferences(s, c.staffPreferences) * 0.2;
  return score;
}
```

Labor cost target is typically 25–35% of projected revenue;
the scoring weights (40/40/20) are configurable per-establishment.

### 3.2.3 Time and attendance

```typescript
interface TimeRecord {
  employeeId: string;
  date: Date;
  clockIn: Date;
  clockOut?: Date;
  scheduledStart: Date;
  scheduledEnd: Date;
  breakStart?: Date[];
  breakEnd?: Date[];
  totalHours: number;
  regularHours: number;
  overtimeHours: number;
  status: 'on_time' | 'late' | 'early_departure'
        | 'no_show' | 'absent' | 'excused';
}

interface BreakPolicy {
  jurisdiction: string;
  shiftLengthHours: number;
  requiredBreaks: BreakRequirement[];
}

interface BreakRequirement {
  type: 'meal' | 'rest';
  durationMinutes: number;
  paid: boolean;
  requiredByHour: number;
}
```

Break compliance MUST be validated on every clock-out; missing
or short breaks generate a penalty event reported to HQ.

### 3.2.4 Shift trading

```typescript
interface ShiftTradeRequest {
  requestId: string;
  fromEmployee: string;
  toEmployee?: string;       // null for open trade
  shift: StaffShift;
  reason?: string;
  status: 'pending' | 'approved' | 'denied' | 'cancelled';
  createdAt: Date;
  approvedBy?: string;
  approvedAt?: Date;
}

function validateShiftTrade(req: ShiftTradeRequest, to: Employee) {
  if (!hasRequiredSkills(to, req.shift.role))
    return { valid: false, reason: 'Insufficient qualifications' };
  if (hasConflict(to, req.shift))
    return { valid: false, reason: 'Scheduling conflict' };
  if (wouldExceedOvertimeLimit(to, req.shift))
    return { valid: false, reason: 'Would exceed overtime limit' };
  return { valid: true };
}
```

## 3.3 Inventory replenishment protocol

### 3.3.1 Recipe-driven depletion

```typescript
interface Recipe {
  recipeId: string;
  menuItemId: string;
  name: string;
  yield: number;
  ingredients: Ingredient[];
  cost: number;
  instructions?: string[];
  prepTime: number;
  cookTime: number;
  allergens: string[];
  version: number;
}

interface Ingredient {
  itemId: string;
  name: string;
  quantity: number;
  unit: string;
  cost: number;
  optional: boolean;
  substituteFor?: string;
}

function depleteInventory(order: Order): void {
  for (const item of order.items) {
    const recipe = getRecipe(item.menuItemId);
    for (const ing of recipe.ingredients) {
      const move: StockMovement = {
        movementId: generateId(),
        itemId: ing.itemId,
        type: 'used',
        quantity: ing.quantity * item.quantity,
        unit: ing.unit,
        timestamp: new Date(),
        userId: order.serverId,
        reference: order.orderId
      };
      recordMovement(move);
      updateQuantity(ing.itemId, -move.quantity);
      checkReorderPoint(ing.itemId);
    }
  }
}
```

### 3.3.2 Purchase orders and auto-reorder

```typescript
interface PurchaseOrder {
  poId: string;
  poNumber: string;
  supplierId: string;
  supplierName: string;
  orderDate: Date;
  expectedDeliveryDate: Date;
  actualDeliveryDate?: Date;
  status: 'draft' | 'submitted' | 'confirmed'
        | 'partially_received' | 'received'
        | 'invoiced' | 'paid' | 'cancelled';
  items: PurchaseOrderItem[];
  subtotal: number;
  tax: number;
  shipping: number;
  total: number;
  paymentTerms: string;
  deliveryAddress: string;
  orderedBy: string;
}

function checkReorderPoints(): PurchaseOrder[] {
  const toReorder = getInventoryItems().filter(i =>
    i.currentQuantity <= i.reorderPoint
  );
  const bySupplier = groupBy(toReorder, 'supplierId');

  return Object.entries(bySupplier).map(([supplierId, items]) =>
    createPurchaseOrder({
      supplierId,
      items: items.map(i => ({
        itemId: i.itemId,
        quantity: i.reorderQuantity,
        unitCost: i.unitCost
      }))
    })
  );
}
```

## 3.4 Multi-location federation

### 3.4.1 Chain hierarchy

```typescript
interface RestaurantChain {
  chainId: string;
  name: string;
  headquarters: Address;
  locations: Location[];
  standardMenus: Menu[];
  corporatePolicies: Policy[];
}

interface Location {
  locationId: string;
  name: string;
  address: Address;
  phone: string;
  timezone: string;
  managerId: string;
  operatingHours: OperatingHours[];
  seatingCapacity: number;
  status: LocationStatus;
  openDate: Date;
  format: 'full_service' | 'quick_service' | 'fast_casual'
        | 'food_truck' | 'kiosk' | 'ghost_kitchen';
}
```

### 3.4.2 Standard menu vs local variation

```typescript
interface StandardMenu {
  menuId: string;
  name: string;
  version: number;
  effectiveDate: Date;
  items: StandardMenuItem[];
  requiredItems: string[];   // must appear at every location
  optionalItems: string[];   // location may add
}

interface LocalMenuVariation {
  locationId: string;
  baseMenuId: string;
  additions: MenuItem[];
  removals: string[];        // not offered locally
  priceAdjustments: PriceAdjustment[];
}

interface PriceAdjustment {
  itemId: string;
  standardPrice: number;
  localPrice: number;
  reason: string;            // e.g. "higher cost of living"
}
```

The chain HQ broadcasts the `StandardMenu` envelope counter-signed
by the chain's signing key; the location runs a delta computation
to produce its effective menu and persists the resulting
`LocalMenuVariation`.

### 3.4.3 Consolidated reporting

```typescript
interface ConsolidatedReport {
  chainId: string;
  period: DateRange;
  locations: LocationReport[];
  totals: AggregateMetrics;
  comparisons: LocationComparison[];
}

interface LocationReport {
  locationId: string;
  name: string;
  revenue: number;
  covers: number;
  averageCheck: number;
  laborCost: number;
  laborPercent: number;
  foodCost: number;
  foodCostPercent: number;
  profitMargin: number;
}

interface LocationComparison {
  metric: string;
  best: { locationId: string; value: number };
  worst: { locationId: string; value: number };
  average: number;
  standardDeviation: number;
}
```

Each location signs its own `LocationReport`; HQ aggregates and
counter-signs the `ConsolidatedReport`. Disputed metrics retain
the raw signed inputs so the chain auditor can re-derive.

## 3.5 Replication and reconciliation

Locations operate in an offline-tolerant mode: the local POS
continues to take orders even when the WAN to chain HQ is down.
Reconciliation proceeds in three steps when the link is restored:

1. **Forward replay** — the location pushes signed envelopes
   in monotonic `createdAt` order; HQ accepts the first envelope
   for each `orderId` and rejects subsequent claims.
2. **Backward delta** — HQ pushes any `StandardMenu` /
   `Policy` updates issued during the outage; the location
   re-derives its effective menu and records the variation.
3. **Audit reconciliation** — HQ runs the consolidated-report
   pipeline. Any envelope whose signature fails verification is
   quarantined and an exception report is raised to the chain
   auditor; quarantine does NOT remove the envelope from the
   local store.

Replication is best-effort within a 24-hour window; longer
outages MUST be flagged to chain ops.

## 3.6 Operational notes

### 3.6.1 Kitchen routing edge cases

Real kitchens deviate from the clean station partition in three
predictable ways, and the protocol MUST tolerate each:

- **Cross-station composites** — a single menu item (e.g. surf
  and turf) routes to both grill and seafood stations and joins
  at expo. The router emits two `KitchenItem` rows with the
  same `seat` and `courseNumber`; expo waits for both bumps.
- **Allergen modifiers** — an "extreme allergy" modifier escalates
  the ticket priority to URGENT and pins routing to the
  designated allergen-clean station regardless of the default
  station for that item.
- **Comp / VOID propagation** — a manager-authorized comp on a
  table mid-service emits a recall-style notification to expo
  so the line cooks know to halt rather than continue producing
  the dish.

### 3.6.2 Schedule constraints in practice

The optimizer in §3.2.2 is the formal version; in production the
constraint set typically also includes minor-employee labor laws
(e.g. limits on late-evening shifts for under-18 staff in many
US states), pre-declared time-off requests, and the
"no-clopen" rule (no closing shift followed by opening the
next morning under the threshold rest period). These are
encoded as hard constraints (filters) on
`generateScheduleCandidates` rather than as soft scoring
adjustments — they MUST NOT be tradeable for marginal labor-cost
gains.

### 3.6.3 Inventory reconciliation cadence

The auto-reorder pipeline of §3.3.2 fires on the `inventory.below_par`
webhook (Phase 2 §2.12). Recommended cadence:

- **Real-time** for high-volume perishables (produce, dairy,
  proteins on the active menu).
- **Daily batch** for dry goods and supplies (avoids over-firing
  POs on a noisy demand signal).
- **Weekly review** for backstock items with long shelf life
  (cleaning supplies, paper goods).

The cadence is configured per-supplier in the `Supplier`
envelope (not specified in this document — see WIA-SUPPLIER for
the canonical supplier shape).

### 3.6.4 Multi-location data residency

Locations in different jurisdictions may face data-residency
requirements that prevent raw envelope replication to chain HQ.
The protocol provides an aggregation-only mode: the location
computes its own `LocationReport` locally, signs it, and pushes
the report (not the underlying envelopes) to HQ. The chain
auditor cannot drill down into individual orders for these
locations; this is an explicit trade-off the chain operator
accepts when expanding into a regulated jurisdiction.
