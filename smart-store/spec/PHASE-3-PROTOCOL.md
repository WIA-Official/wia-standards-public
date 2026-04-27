# PHASE 3 — Protocol

> Smart-store hardware coordination: smart-shelf weight/RFID
> sensors, smart shopping carts, indoor wayfinding mesh, and
> the IoT inventory sensor fleet. This phase describes how the
> physical layer reports up to the AI/ML layer and how chain-HQ
> drives planogram updates back down to the floor.

## 5. Smart Shelf Technology

### 5.1 Sensor Types

Multi-sensor shelves for accurate inventory tracking:

**Weight Sensors:**
- Type: Load cells with 0.1g precision
- Capacity: 10kg per shelf section
- Sampling rate: 100 Hz
- Calibration: Auto-calibration every 24 hours
- Power: Low-power, battery or PoE

**RFID Readers:**
- Standard: UHF Gen 2 (860-960 MHz)
- Read range: Up to 15 meters
- Read rate: 1000+ tags/second
- Antenna: Directional, per shelf section
- Integration: Wired or wireless to edge gateway

**Optical Sensors:**
- Type: Infrared beam break sensors
- Purpose: Detect product removal/return
- Placement: Front of shelf, 5cm intervals
- Response time: <10ms
- Power: Ultra-low power LEDs

**Temperature Sensors:**
- Use case: Cold chain monitoring (dairy, frozen)
- Range: -20°C to +50°C
- Accuracy: ±0.5°C
- Sampling: Every 60 seconds
- Alerts: Real-time threshold violations

### 5.2 Smart Shelf Configuration

```typescript
interface SmartShelf {
  shelfId: string;
  location: {
    zone: string;
    aisle: string;
    section: string;
    level: number; // 1=bottom, 5=top
  };
  sensors: {
    weightSensors: WeightSensor[];
    rfidReaders: RFIDReader[];
    opticalSensors: OpticalSensor[];
    tempSensor?: TempSensor;
  };
  products: ShelfProduct[];
  capacity: {
    maxWeight: number; // kg
    maxProducts: number;
  };
  status: 'active' | 'maintenance' | 'offline';
}

interface ShelfProduct {
  productId: string;
  sku: string;
  name: string;
  expectedWeight: number; // grams
  rfidTag: string;
  quantity: number;
  minStock: number; // reorder threshold
  maxStock: number;
  position: {
    x: number; // cm from left
    y: number; // cm from front
    facings: number; // number of items facing forward
  };
}
```

### 5.3 Inventory Monitoring

Real-time stock tracking:

**Stock Level Detection:**
```
Current Stock = floor(Total Weight / Unit Weight)

Example:
- Product: Milk carton (1000g each)
- Shelf weight: 8500g
- Tray weight: 500g
- Product weight: 8000g
- Stock level: 8000g / 1000g = 8 units

Validation:
- RFID count: 8 tags detected ✓
- Optical sensors: 8 facings detected ✓
- Confidence: HIGH
```

**Restock Alerts:**
```
if (currentStock <= minStock) {
  alert = {
    priority: currentStock == 0 ? 'CRITICAL' : 'HIGH',
    shelfId: shelf.id,
    productId: product.id,
    currentStock: currentStock,
    neededQuantity: maxStock - currentStock,
    message: `Restock ${product.name} at ${shelf.location}`
  };
  sendToStaffApp(alert);
}
```

### 5.4 Planogram Compliance

Ensure products are placed according to plan:

```typescript
interface Planogram {
  storeId: string;
  version: string;
  effectiveDate: Date;
  shelves: PlanogramShelf[];
}

interface PlanogramShelf {
  shelfId: string;
  products: PlanogramProduct[];
}

interface PlanogramProduct {
  productId: string;
  position: { x: number; y: number; };
  facings: number;
  priority: number; // Eye-level = highest priority
}

// Compliance checking
function checkPlanogramCompliance(shelf, planogram) {
  const compliance = {
    isCompliant: true,
    issues: [],
    score: 1.0
  };

  // Check each product position
  for (const planned of planogram.products) {
    const actual = shelf.products.find(p => p.productId === planned.productId);

    if (!actual) {
      compliance.issues.push(`Missing: ${planned.productId}`);
      compliance.score -= 0.1;
    } else if (
      Math.abs(actual.position.x - planned.position.x) > 5 || // 5cm tolerance
      actual.facings !== planned.facings
    ) {
      compliance.issues.push(`Misplaced: ${planned.productId}`);
      compliance.score -= 0.05;
    }
  }

  compliance.isCompliant = compliance.score >= 0.9;
  return compliance;
}
```

---


## 9. Smart Shopping Carts

### 9.1 Cart Technology

Intelligent shopping carts with built-in features:

**Hardware Components:**
- **Weight Sensors**: Track items added/removed
- **Barcode Scanner**: Self-scanning option
- **Display Screen**: 10" touchscreen
- **Camera**: Computer vision for auto-detection
- **RFID Reader**: Passive tag reading
- **Battery**: 12-24 hours of operation
- **Wheels**: Motorized with anti-theft lock
- **Connectivity**: WiFi, 4G/5G, Bluetooth

### 9.2 Smart Cart Features

```typescript
interface SmartShoppingCart {
  cartId: string;
  sessionId?: string;
  customerId?: string;
  location: {
    x: number;
    y: number;
    lastUpdate: Date;
  };
  items: CartItem[];
  totalWeight: number; // kg
  totalValue: number; // currency
  battery: {
    level: number; // 0-100%
    charging: boolean;
  };
  navigation: {
    enabled: boolean;
    destination?: string; // Product or zone
    route?: PathPoint[];
  };
  features: {
    selfCheckout: boolean;
    productRecommendations: boolean;
    couponNotifications: boolean;
    receiptPrinting: boolean;
  };
  status: 'available' | 'in-use' | 'charging' | 'maintenance';
}
```

### 9.3 Self-Scanning Workflow

Customer self-scanning process:

```
1. Customer picks up smart cart
2. Scans loyalty card or opens app to link session
3. Browses store and picks items
4. For each item:
   a. Customer scans barcode manually, OR
   b. Cart camera auto-detects product, OR
   c. Cart RFID reader detects tagged item
   d. Item added to cart display
   e. Running total updated
5. Cart display shows:
   - Items in cart
   - Running total
   - Available coupons
   - Savings summary
6. At exit:
   - Customer taps "Pay" on cart display
   - Payment processed via linked card/app
   - Cart verified (weight check, random audit)
   - Receipt printed or emailed
```

### 9.4 Navigation Assistance

Guide customers to products:

**Product Locator:**
```typescript
function findProduct(productName: string) {
  // Search product database
  const product = productDB.search(productName);

  // Get shelf location
  const shelf = shelfDB.find(s =>
    s.products.some(p => p.productId === product.id)
  );

  // Calculate route from current cart position
  const route = calculateRoute(
    cart.location,
    shelf.location,
    { avoidCrowds: true, shortestPath: true }
  );

  // Display on cart screen
  cart.navigation = {
    enabled: true,
    destination: product.name,
    route: route,
    distance: calculateDistance(route),
    estimatedTime: estimateTime(route, cart.location.speed)
  };

  return route;
}
```

**Turn-by-Turn Directions:**
```
Display on cart screen:
→ "Turn left at end of aisle"
→ "Continue straight for 10 meters"
→ "Milk is on your right, shelf 3, level 2"
→ "Arrived at destination"

Visual aids:
- Arrow indicators on screen
- Augmented reality overlay (future)
- LED strip on cart handle (left/right/straight)
```

### 9.5 Anti-Theft Measures

Prevent cart theft and fraud:

```
Geofencing:
- Cart locked to store perimeter
- If cart crosses boundary → wheels lock
- Alert sent to security

Exit Verification:
- Random audits (10-20% of carts)
- Weight verification: Cart weight vs. expected weight
- RFID scan: Verify all tagged items accounted for
- Video review: If discrepancy detected

Cart Recovery:
- GPS tracking for stolen carts
- Remote wheel lock activation
- Audible alarm
```

---


## 10. Indoor Navigation System

### 10.1 Positioning Technologies

Accurate indoor location tracking:

**Positioning Methods:**

1. **WiFi Fingerprinting**:
   - Accuracy: 3-5 meters
   - Setup: WiFi access points throughout store
   - Method: RSSI-based positioning
   - Pros: Works with customer smartphones
   - Cons: Lower accuracy than other methods

2. **Bluetooth Beacons**:
   - Accuracy: 1-3 meters
   - Setup: BLE beacons every 5-10 meters
   - Method: Trilateration using signal strength
   - Pros: Good accuracy, low power
   - Cons: Requires beacons infrastructure

3. **Ultra-Wideband (UWB)**:
   - Accuracy: 10-30 cm
   - Setup: UWB anchors at known positions
   - Method: Time-of-flight measurement
   - Pros: Very high accuracy
   - Cons: Requires UWB-enabled devices

4. **Computer Vision**:
   - Accuracy: <1 meter
   - Setup: Ceiling cameras with known positions
   - Method: Visual tracking and triangulation
   - Pros: No device required, accurate
   - Cons: Privacy concerns, compute-intensive

### 10.2 Store Mapping

Digital representation of store layout:

```typescript
interface StoreMap {
  storeId: string;
  version: string;
  dimensions: {
    width: number; // meters
    length: number; // meters
    height: number; // meters
  };
  origin: { x: 0, y: 0 }; // Reference point (usually entrance)
  zones: Zone[];
  aisles: Aisle[];
  shelves: Shelf[];
  landmarks: Landmark[];
  obstacles: Obstacle[];
}

interface Zone {
  zoneId: string;
  name: string;
  category: string; // "produce", "dairy", "bakery"
  polygon: Point[]; // Boundary coordinates
  floor: number;
}

interface Aisle {
  aisleId: string;
  number: string; // "Aisle 1", "Aisle 2"
  start: Point;
  end: Point;
  width: number;
  products: string[]; // Product categories in this aisle
}

interface Landmark {
  landmarkId: string;
  name: string; // "Main Entrance", "Checkout 1"
  position: Point;
  type: 'entrance' | 'exit' | 'checkout' | 'restroom' | 'service-desk';
}
```

### 10.3 Routing Algorithm

Optimal path calculation:

```typescript
interface RouteRequest {
  start: Point;
  destination: Point | string; // Coordinates or product name
  preferences: {
    shortest: boolean;
    avoidCrowds: boolean;
    accessibility: boolean; // Wheelchair-friendly
  };
}

interface Route {
  waypoints: Waypoint[];
  distance: number; // meters
  estimatedTime: number; // seconds
  instructions: Instruction[];
}

interface Waypoint {
  position: Point;
  type: 'start' | 'turn' | 'destination' | 'intermediate';
  direction?: number; // degrees from north
}

interface Instruction {
  text: string;
  distance: number; // meters to this instruction
  icon: 'straight' | 'left' | 'right' | 'uturn';
}

// A* pathfinding algorithm
function calculateRoute(request: RouteRequest): Route {
  const start = request.start;
  const goal = resolveDestination(request.destination);

  // A* algorithm
  const openSet = new PriorityQueue();
  const cameFrom = new Map();
  const gScore = new Map();
  const fScore = new Map();

  openSet.enqueue(start, 0);
  gScore.set(start, 0);
  fScore.set(start, heuristic(start, goal));

  while (!openSet.isEmpty()) {
    const current = openSet.dequeue();

    if (current.equals(goal)) {
      return reconstructPath(cameFrom, current);
    }

    for (const neighbor of getNeighbors(current)) {
      const tentativeGScore = gScore.get(current) + distance(current, neighbor);

      // Apply crowd penalty
      if (request.preferences.avoidCrowds) {
        tentativeGScore += getCrowdDensity(neighbor) * 10;
      }

      if (tentativeGScore < (gScore.get(neighbor) || Infinity)) {
        cameFrom.set(neighbor, current);
        gScore.set(neighbor, tentativeGScore);
        fScore.set(neighbor, tentativeGScore + heuristic(neighbor, goal));

        if (!openSet.contains(neighbor)) {
          openSet.enqueue(neighbor, fScore.get(neighbor));
        }
      }
    }
  }

  return null; // No path found
}
```

### 10.4 Wayfinding UI

User interface for navigation:

**Mobile App Display:**
```
┌──────────────────────────────────┐
│  Store Map                    [X]│
├──────────────────────────────────┤
│                                  │
│      [Your location: 📍]        │
│           ↓                      │
│      ┌─────┐                     │
│      │ A1  │  ← Turn left here   │
│      └─────┘                     │
│           ↓                      │
│      ┌─────┐                     │
│      │ A2  │                     │
│      └─────┘                     │
│           ↓                      │
│      ┌─────┐                     │
│      │ A3  │  ← Milk (🎯)        │
│      └─────┘                     │
│                                  │
│  Distance: 25m  |  ETA: 1 min   │
└──────────────────────────────────┘
```

**Smart Cart Display:**
```
Simple directional arrows:
→ Straight ahead (15m)
← Turn left (Aisle 3)
🎯 Destination ahead (5m)
✓ Arrived
```

---


## 12. IoT Inventory Sensors

### 12.1 Sensor Network

Comprehensive inventory monitoring:

**Sensor Types and Placement:**

1. **Shelf Sensors** (every shelf):
   - Weight sensors (load cells)
   - RFID readers
   - Optical sensors

2. **Freezer/Refrigerator Sensors**:
   - Temperature sensors (every unit)
   - Door open/close sensors
   - Defrost cycle monitors

3. **Warehouse Sensors**:
   - Pallet weight sensors
   - RFID gate readers
   - Environmental sensors (temp, humidity)

4. **Expiry Monitors**:
   - Camera-based date recognition
   - RFID tags with expiry data
   - FIFO compliance tracking

### 12.2 Real-Time Inventory Tracking

```typescript
interface InventoryItem {
  productId: string;
  sku: string;
  locations: ItemLocation[];
  totalStock: number;
  status: 'in-stock' | 'low-stock' | 'out-of-stock' | 'overstock';
  reorderPoint: number;
  reorderQuantity: number;
  expiryTracking: boolean;
}

interface ItemLocation {
  locationType: 'shelf' | 'backroom' | 'warehouse' | 'transit';
  locationId: string;
  quantity: number;
  lastCounted: Date;
  lastMovement?: Date;
  expiryDates?: Date[]; // For perishables
}

// Real-time stock update
function updateInventory(event: InventoryEvent) {
  const item = getInventoryItem(event.productId);
  const location = item.locations.find(l => l.locationId === event.locationId);

  switch (event.type) {
    case 'sale':
      location.quantity -= event.quantity;
      item.totalStock -= event.quantity;
      break;

    case 'restock':
      location.quantity += event.quantity;
      item.totalStock += event.quantity;
      if (event.expiryDate) {
        location.expiryDates.push(event.expiryDate);
      }
      break;

    case 'shrinkage':
      location.quantity -= event.quantity;
      item.totalStock -= event.quantity;
      logShrinkage(event);
      break;
  }

  // Update status
  if (item.totalStock === 0) {
    item.status = 'out-of-stock';
    sendAlert({ type: 'stockout', productId: item.productId });
  } else if (item.totalStock <= item.reorderPoint) {
    item.status = 'low-stock';
    sendAlert({ type: 'reorder', productId: item.productId });
  }

  // Broadcast to connected systems
  broadcastInventoryUpdate(item);

  saveInventoryItem(item);
}
```

### 12.3 Automated Reordering

Smart replenishment system:

```typescript
interface ReorderRule {
  productId: string;
  method: 'fixed-point' | 'periodic' | 'predictive';
  parameters: {
    reorderPoint?: number; // Units
    reorderQuantity?: number;
    reviewPeriod?: number; // Days
    leadTime: number; // Days
    safetyStock: number;
  };
  supplier: string;
  autoApprove: boolean; // Auto-submit order or require approval
}

// Predictive reordering
function predictiveReorder(productId: string) {
  const item = getInventoryItem(productId);
  const history = getSalesHistory(productId, 90); // Last 90 days

  // Calculate average daily sales
  const avgDailySales = history.reduce((sum, day) => sum + day.quantity, 0) / 90;

  // Predict next 30 days demand
  const forecast = forecastDemand(history, 30);
  const predictedDemand = forecast.reduce((sum, day) => sum + day.quantity, 0);

  // Current stock
  const currentStock = item.totalStock;

  // Lead time demand
  const leadTimeDemand = avgDailySales * item.reorderRule.parameters.leadTime;

  // Safety stock (2 weeks of average sales)
  const safetyStock = avgDailySales * 14;

  // Reorder point
  const reorderPoint = leadTimeDemand + safetyStock;

  // Check if reorder needed
  if (currentStock < reorderPoint) {
    const reorderQty = predictedDemand + safetyStock - currentStock;

    createPurchaseOrder({
      productId: productId,
      quantity: Math.ceil(reorderQty),
      supplier: item.reorderRule.supplier,
      expectedDelivery: addDays(new Date(), item.reorderRule.parameters.leadTime),
      reason: 'predictive-reorder',
      autoApprove: item.reorderRule.autoApprove
    });
  }
}
```

### 12.4 Shrinkage Detection

Identify inventory loss:

```
Shrinkage = (Book Inventory - Physical Inventory) / Book Inventory × 100%

Causes:
1. Theft (shoplifting, employee theft): 60-70%
2. Administrative errors (pricing, scanning): 15-20%
3. Vendor fraud: 5-10%
4. Damage/expiry: 5-10%

Detection Methods:
1. Continuous monitoring: Real-time sensor discrepancies
2. Cycle counting: Regular physical counts vs. system
3. Variance analysis: Expected vs. actual stock levels
4. Video audit: Review footage for theft events
```

```typescript
function detectShrinkage(productId: string, locationId: string) {
  const bookQty = getBookInventory(productId, locationId);
  const physicalQty = getPhysicalInventory(productId, locationId);

  const variance = bookQty - physicalQty;
  const shrinkageRate = (variance / bookQty) * 100;

  if (Math.abs(shrinkageRate) > 2) { // 2% threshold
    const alert: ShrinkageAlert = {
      productId: productId,
      locationId: locationId,
      bookQuantity: bookQty,
      physicalQuantity: physicalQty,
      variance: variance,
      shrinkageRate: shrinkageRate,
      timestamp: new Date(),
      severity: shrinkageRate > 10 ? 'high' : 'medium',
      suggestedAction: shrinkageRate > 10 ?
        'Immediate investigation required' :
        'Schedule cycle count'
    };

    // Log and alert
    logShrinkageEvent(alert);
    sendAlertToManagement(alert);

    // Trigger video review if available
    if (shrinkageRate > 5) {
      requestVideoAudit(locationId, getLastHours(24));
    }
  }
}
```

---


