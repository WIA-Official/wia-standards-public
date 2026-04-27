# PHASE 1 — Data Format

> Smart-store canonical envelopes: customer session, virtual
> cart, smart-shelf weight tick, computer-vision detection,
> payment receipt, and the architecture map that all of these
> shapes inhabit. All envelopes are signed Ed25519 over the
> RFC 8785 JCS canonical form so that any edge node can verify
> upstream provenance without re-running inference.

## 1. Introduction

### 1.1 Purpose

This specification defines the Smart Store standard, covering automated checkout systems, computer vision-based product recognition, intelligent inventory management, and customer analytics for next-generation retail environments.

### 1.2 Scope

The standard covers:
- Automated checkout and walk-out shopping experiences
- Computer vision systems for product recognition and tracking
- Smart shelf technology with sensors and real-time monitoring
- Customer tracking, analytics, and heatmap generation
- Digital signage and electronic shelf label systems
- Smart shopping carts with self-scanning capabilities
- Indoor navigation and wayfinding systems
- AI-powered personalized recommendations
- IoT sensor networks for inventory management
- Secure payment processing and fraud detection

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - Smart Store technology aims to revolutionize retail by providing seamless shopping experiences, reducing wait times, improving inventory accuracy, and enabling data-driven insights while respecting customer privacy.

### 1.4 Terminology

- **Cashierless Store**: Retail environment with automated checkout
- **Just Walk Out (JWO)**: Technology enabling walk-in, grab, and go shopping
- **Computer Vision**: AI-powered visual recognition and tracking
- **RFID**: Radio-Frequency Identification for product tagging
- **ESL**: Electronic Shelf Labels with digital displays
- **SKU**: Stock Keeping Unit (product identifier)
- **Heatmap**: Visual representation of customer density and movement
- **Edge Computing**: Local data processing for real-time response
- **Digital Twin**: Virtual replica of physical store
- **Planogram**: Visual diagram of product placement

---

## 2. Architecture Overview

### 2.1 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   Smart Store Platform                       │
├─────────────────────────────────────────────────────────────┤
│  Application Layer                                           │
│  ├─ Customer App (iOS/Android)                              │
│  ├─ Staff Management Dashboard                              │
│  ├─ Analytics & Reporting                                   │
│  └─ Admin Console                                           │
├─────────────────────────────────────────────────────────────┤
│  Business Logic Layer                                        │
│  ├─ Checkout Orchestration                                  │
│  ├─ Inventory Management                                    │
│  ├─ Customer Analytics                                      │
│  ├─ Recommendation Engine                                   │
│  └─ Payment Processing                                      │
├─────────────────────────────────────────────────────────────┤
│  AI/ML Layer                                                 │
│  ├─ Computer Vision (Product Recognition)                   │
│  ├─ Customer Tracking (Pose Estimation)                     │
│  ├─ Behavior Analysis (Pattern Recognition)                 │
│  ├─ Demand Forecasting                                      │
│  └─ Fraud Detection                                         │
├─────────────────────────────────────────────────────────────┤
│  IoT Device Layer                                            │
│  ├─ Cameras (RGB, Depth, Thermal)                          │
│  ├─ Smart Shelves (Weight, RFID, Optical)                  │
│  ├─ Digital Signage Displays                                │
│  ├─ Electronic Shelf Labels                                 │
│  ├─ Smart Shopping Carts                                    │
│  ├─ Entry/Exit Gates                                        │
│  └─ Environmental Sensors                                   │
├─────────────────────────────────────────────────────────────┤
│  Edge Computing Layer                                        │
│  ├─ Real-time Video Processing                              │
│  ├─ Local Model Inference                                   │
│  ├─ Data Aggregation                                        │
│  └─ Failover Mechanism                                      │
├─────────────────────────────────────────────────────────────┤
│  Network Layer                                               │
│  ├─ 5G/WiFi 6 Connectivity                                  │
│  ├─ Edge-to-Cloud Sync                                      │
│  ├─ Secure VPN Tunnels                                      │
│  └─ Load Balancing                                          │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Store Zones

Smart stores are divided into functional zones:

| Zone | Purpose | Technologies | Coverage |
|------|---------|--------------|----------|
| Entry | Customer identification | Face recognition, QR scanner | 100% |
| Shopping | Product browsing | Cameras, smart shelves | 100% |
| Hot Zones | High-traffic areas | Dense sensors, analytics | Targeted |
| Checkout | Payment processing | POS, automated gates | 100% |
| Exit | Verification | RFID gates, weight check | 100% |
| Backroom | Inventory storage | RFID readers, temp sensors | 100% |

### 2.3 Data Flow

```
Customer Entry → Authentication → Tracking Initiation
                                          ↓
Product Interaction → Vision Recognition → Cart Update
                                          ↓
Movement Tracking → Analytics Update → Heatmap Generation
                                          ↓
Customer Exit → Cart Finalization → Payment Processing
                                          ↓
Receipt Generation → Feedback Collection → Session Close
```


### 3.3 Session Management

```typescript
interface CheckoutSession {
  sessionId: string;
  customerId: string;
  entryTime: Date;
  exitTime?: Date;
  authMethod: 'app' | 'card' | 'biometric';
  virtualCart: CartItem[];
  totalAmount: number;
  status: 'active' | 'completed' | 'disputed' | 'cancelled';
  confidence: number;
}

interface CartItem {
  productId: string;
  productName: string;
  quantity: number;
  price: number;
  addedAt: Date;
  confidence: number;
  detectionMethod: 'vision' | 'rfid' | 'weight' | 'hybrid';
}
```

### 3.4 Dispute Resolution

Handling checkout discrepancies:

1. **Low Confidence Items**: Flagged for manual review
2. **Customer Dispute**: Review video footage (privacy-compliant)
3. **Missing Products**: Cross-reference multiple sensor data
4. **Refund Process**: Automated for verified disputes
5. **Audit Trail**: Complete session recording for 30 days

---

## 6. Customer Tracking and Analytics

### 6.1 Movement Tracking

Track customer paths through the store:

**Position Estimation:**
```
Methods:
1. Camera Triangulation: Use multiple cameras to determine 3D position
2. WiFi Fingerprinting: Approximate location via WiFi signal strength
3. Sensor Fusion: Combine vision + WiFi for accuracy

Position Data:
- X, Y coordinates: Meters from store origin (0, 0)
- Z coordinate: Floor level (for multi-story stores)
- Timestamp: Millisecond precision
- Accuracy: ±0.5 meters
```

**Trajectory Tracking:**
```typescript
interface CustomerTrajectory {
  sessionId: string;
  customerId: string; // Anonymous ID
  path: PathPoint[];
  totalDistance: number; // meters
  avgSpeed: number; // meters/second
  visitedZones: string[];
  dwellTimes: Map<string, number>; // zone -> milliseconds
  startTime: Date;
  endTime?: Date;
}

interface PathPoint {
  x: number;
  y: number;
  timestamp: Date;
  zone: string;
  action?: 'browsing' | 'picking' | 'examining' | 'moving';
}
```

### 6.2 Heatmap Generation

Visualize customer density and movement:

**Heatmap Types:**

1. **Density Heatmap**: Where customers spend time
   ```
   Grid: Divide store into 1m x 1m cells
   Value: Sum of time spent in each cell
   Visualization: Color gradient (blue=low, red=high)
   ```

2. **Flow Heatmap**: Direction and volume of movement
   ```
   Vector field: Direction arrows
   Magnitude: Customer count passing through
   Use: Identify traffic patterns, bottlenecks
   ```

3. **Engagement Heatmap**: Product interaction hotspots
   ```
   Overlay on product locations
   Value: Number of interactions per product
   Use: Identify popular products, optimize placement
   ```

**Heatmap Generation Algorithm:**
```python
def generate_heatmap(trajectories, grid_size=1.0):
    # Initialize grid
    max_x = store.width
    max_y = store.length
    grid = np.zeros((int(max_y/grid_size), int(max_x/grid_size)))

    # Accumulate dwell times
    for traj in trajectories:
        for i in range(len(traj.path) - 1):
            p1, p2 = traj.path[i], traj.path[i+1]

            # Cell coordinates
            cell_x = int(p1.x / grid_size)
            cell_y = int(p1.y / grid_size)

            # Dwell time (seconds)
            dwell = (p2.timestamp - p1.timestamp).total_seconds()

            # Accumulate
            grid[cell_y, cell_x] += dwell

    # Normalize
    grid = grid / len(trajectories)

    return grid
```

### 6.3 Dwell Time Analysis

Measure time spent in zones and at products:

**Zone Dwell Time:**
```typescript
interface ZoneDwellTime {
  zone: string;
  totalCustomers: number;
  avgDwellTime: number; // seconds
  minDwellTime: number;
  maxDwellTime: number;
  percentile50: number; // median
  percentile90: number;
}

// Dwell time benchmarks
const DWELL_BENCHMARKS = {
  'entrance': { avg: 5, target: 3 }, // Reduce entry friction
  'produce': { avg: 120, target: 90 }, // Browsing zone
  'dairy': { avg: 30, target: 30 }, // Quick pick
  'checkout': { avg: 180, target: 30 }, // Minimize wait
  'total': { avg: 600, target: 480 } // 8 minutes total
};
```

**Product Dwell Time:**
```
Engagement Score = (Dwell Time) × (Interaction Count) × (View Count)

High Engagement (Score > 100):
→ Popular product, good placement
→ Maintain visibility

Low Engagement (Score < 20):
→ Poor placement or unpopular
→ Consider repositioning or promotion
```

### 6.4 Conversion Funnel Analysis

Track the customer journey from entry to purchase:

```
Funnel Stages:
1. Store Entry: All customers (100%)
2. Zone Visit: Visited product category (85%)
3. Product Interaction: Picked up product (60%)
4. Cart Addition: Kept product (45%)
5. Purchase: Completed checkout (40%)

Conversion Rate = Purchases / Store Entries = 40%

Drop-off Analysis:
- Entry → Visit: 15% drop (improve signage/layout)
- Visit → Interaction: 25% drop (enhance product visibility)
- Interaction → Cart: 15% drop (pricing/quality concerns)
- Cart → Purchase: 5% drop (normal abandonment rate)
```

### 6.5 A/B Testing

Test store layout and product placement:

```typescript
interface ABTest {
  testId: string;
  name: string;
  startDate: Date;
  endDate: Date;
  variants: {
    control: TestVariant;
    treatment: TestVariant;
  };
  metrics: {
    conversionRate: number;
    avgBasketSize: number;
    dwellTime: number;
    revenue: number;
  };
  sampleSize: number;
  significance: number; // p-value
  winner?: 'control' | 'treatment';
}

interface TestVariant {
  name: string;
  changes: string[]; // Description of changes
  allocation: number; // % of traffic (50/50 or 80/20)
  results: ABTestResults;
}

// Example test
const test = {
  testId: 'test-001',
  name: 'Dairy Section Placement',
  variants: {
    control: {
      name: 'Current Layout',
      changes: ['Dairy at back of store'],
      allocation: 0.5
    },
    treatment: {
      name: 'Front Placement',
      changes: ['Dairy at front-right'],
      allocation: 0.5
    }
  }
};
```

---


