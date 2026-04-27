# PHASE 4 — Integration

> Restaurant-tech integration with the broader commerce, payment,
> analytics, and compliance landscape. This phase covers customer
> analytics, menu engineering, payment splits and tip pools,
> reporting / KPIs, security and privacy compliance, and the
> third-party integrations every commercial restaurant POS will
> ultimately encounter.

## 4.1 Customer analytics integration

### 4.1.1 Customer Lifetime Value

```typescript
function calculateCustomerLifetimeValue(c: Customer): CLV {
  const historicalCLV     = c.totalSpent;
  const avgVisitValue     = c.averageCheckSize;
  const visitFrequency    = calculateVisitFrequency(c);
  const estimatedLifetime = estimateCustomerLifetime(c);

  const predictedCLV  = avgVisitValue * visitFrequency * estimatedLifetime;
  const retention     = calculateRetentionProbability(c);
  const adjustedCLV   = predictedCLV * retention;

  return {
    historical: historicalCLV,
    predicted:  predictedCLV,
    adjusted:   adjustedCLV,
    retentionProbability: retention
  };
}

function calculateVisitFrequency(c: Customer): number {
  const days = (Date.now() - c.joinDate.getTime()) / (1000 * 60 * 60 * 24);
  return c.totalVisits / (days / 30);
}
```

### 4.1.2 Dining patterns and churn prediction

```typescript
interface DiningPattern {
  customerId: string;
  preferredDays: number[];
  preferredTimes: ('breakfast' | 'lunch' | 'afternoon'
                 | 'dinner' | 'late_night')[];
  averagePartySize: number;
  averageDuration: number;
  peakSeasonMonths: number[];
  orderPatterns: OrderPattern[];
}

interface OrderPattern {
  category: string;
  frequency: number;         // % of visits
  avgSpend: number;
  trending: 'up' | 'down' | 'stable';
}

function predictChurn(c: Customer): ChurnPrediction {
  let churn = 0;

  const daysSince = (Date.now() - c.lastVisit.getTime()) / 86_400_000;
  const expected  = 30 / calculateVisitFrequency(c);
  if (daysSince > expected * 2) churn += 0.3;

  const recent  = calculateVisitFrequency(c, 90);
  const overall = calculateVisitFrequency(c);
  if (recent < overall * 0.7) churn += 0.3;

  const recentAvg = calculateAverageSpend(c, 90);
  if (recentAvg < c.averageCheckSize * 0.8) churn += 0.2;

  if (hasRecentNegativeFeedback(c)) churn += 0.2;

  return {
    churnScore: Math.min(churn, 1),
    risk: churn > 0.6 ? 'high' : churn > 0.3 ? 'medium' : 'low',
    recommendedActions: generateRetentionActions(churn)
  };
}
```

### 4.1.3 Loyalty programs

```typescript
interface LoyaltyProgram {
  programId: string;
  name: string;
  pointsPerDollar: number;
  pointsExpireDays?: number;
  tiers: LoyaltyTier[];
  rewards: Reward[];
}

interface LoyaltyTier {
  tierName: string;
  requiredPoints: number;
  benefits: string[];
  pointsMultiplier: number;
  discountPercent: number;
}

interface Reward {
  rewardId: string;
  name: string;
  description: string;
  pointsCost: number;
  type: 'discount_percent' | 'discount_fixed' | 'free_item'
      | 'upgrade' | 'experience';
  value: number;
  expiryDays: number;
}
```

## 4.2 Menu engineering

### 4.2.1 Menu mix classification

```typescript
function classifyMenuItem(
  item: MenuMixAnalysis,
  avgProfitMargin: number,
  avgSalesPercent: number
): 'star' | 'plow_horse' | 'puzzle' | 'dog' {
  const highProfit     = item.profitMargin > avgProfitMargin;
  const highPopularity = item.salesPercent > avgSalesPercent;

  if ( highProfit &&  highPopularity) return 'star';
  if (!highProfit &&  highPopularity) return 'plow_horse';
  if ( highProfit && !highPopularity) return 'puzzle';
  return 'dog';
}
```

### 4.2.2 Pricing strategies

```typescript
function calculateOptimalPrice(
  item: MenuItem,
  strategy: PricingStrategy
): number {
  switch (strategy.method) {
    case 'cost_plus':
      return item.cost * 3.0;             // industry-typical 3x markup

    case 'target_margin':
      return item.cost / (1 - strategy.targetMargin!);

    case 'competitive':
      const avg = strategy.competitorPrices!
        .reduce((a, b) => a + b) / strategy.competitorPrices!.length;
      return avg;

    case 'psychological':
      const base = calculateOptimalPrice(item, { method: 'cost_plus' });
      return Math.ceil(base) - 0.01;      // round to .99

    default:
      return item.cost * 3.0;
  }
}
```

### 4.2.3 ABC analysis

```typescript
function performABCAnalysis(items: MenuItem[]): ABCAnalysis {
  const sorted = items.sort((a, b) => b.revenue - a.revenue);
  const total  = sorted.reduce((s, i) => s + i.revenue, 0);

  let cumulative = 0;
  const analyzed = sorted.map(i => {
    cumulative += i.revenue;
    const pct = (cumulative / total) * 100;
    return {
      itemId: i.id,
      name: i.name,
      revenue: i.revenue,
      cumulativeRevenue: cumulative,
      cumulativePercent: pct,
      class: pct <= 80 ? 'A' : pct <= 95 ? 'B' : 'C'
    };
  });

  return { items: analyzed, classBreakdown: summarizeClasses(analyzed) };
}
```

## 4.3 Payment integration

### 4.3.1 Split payments

```typescript
function processSplitPayment(
  order: Order,
  splitType: 'even' | 'by_item' | 'custom'
): SplitPayment {
  switch (splitType) {
    case 'even':    return splitEvenly(order);
    case 'by_item': return splitByItem(order);
    case 'custom':  return createCustomSplit(order);
  }
}
```

### 4.3.2 Tip pooling

```typescript
interface TipPool {
  date: Date;
  totalTips: number;
  distribution: TipDistribution[];
  method: 'hours_worked' | 'points' | 'equal' | 'role_based';
}

interface TipDistribution {
  employeeId: string;
  role: StaffRole;
  hoursWorked: number;
  points: number;
  amount: number;
  percent: number;
}

function distributeTips(pool: TipPool, employees: Employee[]): TipDistribution[] {
  switch (pool.method) {
    case 'hours_worked': return distributeByHours(pool, employees);
    case 'points':       return distributeByPoints(pool, employees);
    case 'equal':        return distributeEqually(pool, employees);
    case 'role_based':   return distributeByRole(pool, employees);
  }
}
```

## 4.4 Reporting and KPIs

### 4.4.1 Daily sales report

```typescript
interface DailySalesReport {
  date: Date;
  location: string;

  totalRevenue:    number;
  foodRevenue:     number;
  beverageRevenue: number;
  alcoholRevenue:  number;

  totalCovers:     number;
  totalOrders:     number;

  averageCheck:           number;
  averageCoversPerTable:  number;

  breakfast: PeriodSales;
  lunch:     PeriodSales;
  dinner:    PeriodSales;

  dineIn:   number;
  takeout:  number;
  delivery: number;

  cash:        number;
  creditCard:  number;
  other:       number;

  topSellingItems:  TopItem[];

  totalDiscounts:  number;
  discountPercent: number;
}
```

### 4.4.2 Essential KPIs

```typescript
interface RestaurantKPIs {
  // Revenue
  dailySales:    number;
  monthlySales:  number;
  salesGrowth:   number;     // % vs prior period
  revPASH:       number;     // revenue per available seat hour

  // Cost
  primeCost:          number;
  primeCostPercent:   number; // target < 60%
  foodCostPercent:    number; // target 28–35%
  laborCostPercent:   number; // target 25–35%

  // Operations
  tableTurnover:     number;
  averageTicketTime: number;
  orderAccuracy:     number;
  wastePercent:      number;

  // Customer
  customerRetention:   number;
  repeatCustomerRate:  number;
  averageCheckSize:    number;
  netPromoterScore:    number;

  // Employee
  employeeTurnover:        number;
  averageTenure:           number;     // months
  trainingCompletionRate:  number;
}
```

These KPIs are computed by HQ from the `LocationReport` rollup
(Phase 3 §3.4.3) and surfaced to chain ops dashboards.

## 4.5 Security and compliance

### 4.5.1 PCI DSS

- Never store full PAN; only `last4` and `cardBrand`.
- Tokenize payment data via the processor's vault.
- Encrypt sensitive data at rest (AES-256-GCM) and in transit
  (TLS 1.2+).
- Implement role-based access controls (Phase 2 §2.10).
- Quarterly external security audit + annual SAQ.

### 4.5.2 Data privacy

- GDPR for EU customers — right to access, right to erasure,
  consent capture for marketing communications.
- CCPA for California — disclosure of categories collected,
  opt-out of sale.
- Customer data export (machine-readable JSON) and deletion
  endpoints MUST be provided.
- Retention policy: order envelopes 7 years (audit), customer
  PII configurable per jurisdiction.

## 4.6 Third-party integrations

The standard recognises five integration domains and prescribes
a webhook + REST contract each:

- **Payment processors** — Stripe, Square, Toast: vault-based
  tokenization, 3DS step-up, auth/capture/refund mapping.
- **Reservation platforms** — OpenTable, Resy, Yelp Reservations:
  inbound reservation push + outbound availability webhook.
- **Delivery services** — Uber Eats, DoorDash, Grubhub: order
  push to KDS, status broadcast, payout reconciliation.
- **Accounting** — QuickBooks, Xero: nightly journal export
  using the CSV shape of Phase 1 §1.9.
- **Loyalty / review aggregators** — points reconciliation,
  review-event ingestion for the dining-pattern feature.

## 4.7 References

### Industry standards
- National Restaurant Association operating guidelines
- ServSafe food-safety certification
- PCI DSS v4.0 payment-security requirements
- OSHA workplace-safety regulations

### Related WIA standards
- **WIA-PAYMENT** — payment processing
- **WIA-DATA** — generic data format envelope
- **WIA-SECURITY** — security baseline

弘益人間 — Benefit All Humanity. The integration phase keeps the
standard honest: a restaurant POS that can't talk to its
processor, its delivery aggregator, and its accounting back-end
is just a cash drawer with a screen.

## 4.8 Implementation guidance

### 4.8.1 Onboarding a new payment processor

A processor adapter is conformant when it:

- Maps the processor's auth/capture/refund verbs to the canonical
  `Payment` envelope state machine of Phase 1 §1.7.
- Translates the processor's tokenization scheme into the
  `processorReference` field; `last4` and `cardBrand` MUST be
  populated for receipt printing.
- Surfaces processor-side declined/3DS-required responses as
  `code: PAYMENT_AUTH_REQUIRED` with `nextActionUrl` populated
  so the POS can hand the customer the step-up page.
- Persists the raw processor response in a write-once log keyed
  by `processorReference` for chargeback defense.

### 4.8.2 Delivery integration normalization

Delivery aggregator orders arrive in different shapes; the
adapter MUST normalize:

- Customer name → first 8 characters + last name initial on
  the kitchen ticket (privacy preserving).
- Aggregator's order ID → secondary index on the canonical
  Phase-1 `Order` envelope (`externalReferences[]`).
- Aggregator-side tip → only credited to the staff if the
  aggregator's contract explicitly remits the tip to the
  restaurant; otherwise it remains in the aggregator's wallet
  and MUST NOT appear in the tip pool.

### 4.8.3 Accounting export reconciliation

The nightly journal export (CSV per Phase 1 §1.9) is the
primary feed for QuickBooks / Xero. Implementations MUST also
produce a separate signed JSON envelope per business day so
the auditor can verify the CSV is a faithful projection. The
auditor pipeline:

```
1. Fetch the day's signed JSON envelope from /api/reports/sales
   (Phase 2 §2.1).
2. Verify the location's signature against its public key.
3. Re-derive the CSV by applying the published projection rules.
4. Diff the derived CSV against the actual export; any non-zero
   diff is an integrity event reported to chain audit.
```

### 4.8.4 Loyalty platform integration

External loyalty platforms (Toast Loyalty, Square Loyalty, etc.)
may provide their own points engine. The standard recognises
this and provides a "passthrough" loyalty mode in which the
restaurant POS:

- Calls the external platform's points-earning API at order
  commit, passing the canonical `Order` envelope.
- Receives back a points balance and any auto-redeemed reward
  references.
- Records the points and reward IDs on the order's
  `externalReferences[]` so the receipt prints accurately.

The native loyalty engine of §4.1.3 remains available for
restaurants that prefer to keep the points logic in-house.

### 4.8.5 Compliance reviewer handover

When the establishment is audited (PCI annual, GDPR data-subject
request, jurisdictional health/labor inspection), the auditor
is given a read-only role bound to the `audit:read` scope; this
scope can read all envelopes but cannot modify anything and
cannot trigger any webhook. All audit reads are themselves
logged to the audit-trail topic for after-the-fact review.
