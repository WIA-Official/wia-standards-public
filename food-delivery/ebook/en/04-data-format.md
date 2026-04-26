# Chapter 4: Data Formats and Models

---

## 4.1 Overview

This chapter provides detailed specifications for all data structures used in the WIA-IND-009 standard. All data is exchanged in JSON format with strongly-typed TypeScript interfaces for implementation guidance.

### Design Principles

1. **Clarity**: Field names are self-explanatory
2. **Consistency**: Same patterns across all entities
3. **Extensibility**: metadata fields for future additions
4. **Validation**: Clear constraints and formats
5. **Interoperability**: Standard formats (ISO 8601, E.164, etc.)

---

## 4.2 Order Entity

### 4.2.1 Complete Order Structure

```typescript
interface Order {
  // ========== Identity ==========
  id: string;                       // UUID v4
  externalId?: string;              // Partner system reference
  confirmationCode: string;         // Human-readable (e.g., "ABC123")

  // ========== Parties ==========
  restaurantId: string;             // UUID
  customerId: string;               // UUID
  driverId?: string;                // UUID (assigned later)

  // ========== Items ==========
  items: OrderItem[];
  subtotal: number;                 // USD cents (2500 = $25.00)
  tax: number;                      // USD cents
  deliveryFee: number;              // USD cents
  serviceFee: number;               // USD cents
  smallOrderFee: number;            // USD cents
  discount: number;                 // USD cents (negative)
  tip: number;                      // USD cents
  total: number;                    // USD cents

  // ========== Locations ==========
  pickupLocation: Location;
  deliveryLocation: Location;

  // ========== Timing ==========
  createdAt: Date;                  // ISO 8601
  confirmedAt?: Date;
  preparedAt?: Date;
  readyAt?: Date;
  assignedAt?: Date;
  pickedUpAt?: Date;
  deliveredAt?: Date;
  estimatedDelivery: Date;          // Promise to customer
  actualDelivery?: Date;
  scheduledFor?: Date;              // For scheduled orders
  deliveryWindow?: TimeWindow;

  // ========== Status ==========
  status: OrderStatus;
  statusHistory: StatusChange[];

  // ========== Logistics ==========
  route?: Route;
  distance: number;                 // Kilometers
  duration: number;                 // Minutes (estimated)
  actualDuration?: number;          // Minutes (actual)
  temperatureRequirement: TemperatureType;
  vehicleType?: VehicleType;

  // ========== Preferences ==========
  contactlessDelivery: boolean;
  utensils: boolean;
  specialInstructions?: string;     // Max 500 chars
  deliveryInstructions?: string;    // Address-specific

  // ========== Quality ==========
  temperatureLogs?: TemperatureReading[];
  photos?: OrderPhoto[];
  rating?: OrderRating;
  issues?: OrderIssue[];

  // ========== Compliance ==========
  cancelledBy?: 'customer' | 'restaurant' | 'driver' | 'system';
  cancellationReason?: string;
  refundAmount?: number;            // USD cents
  refundReason?: string;

  // ========== Metadata ==========
  metadata?: Record<string, any>;   // Extensibility
  version: number;                  // For optimistic locking
}
```

### 4.2.2 Order Item

```typescript
interface OrderItem {
  id: string;                       // UUID
  externalId?: string;              // Menu item ID
  name: string;
  description?: string;
  quantity: number;                 // Must be > 0
  unitPrice: number;                // USD cents
  totalPrice: number;               // quantity × unitPrice
  temperature: TemperatureType;

  // Customization
  modifiers?: ItemModifier[];
  specialRequests?: string;
  allergens?: string[];

  // Metadata
  sku?: string;
  category?: string;
  image?: string;                   // URL
}

interface ItemModifier {
  id: string;
  name: string;                     // "Extra cheese", "No onions"
  price: number;                    // USD cents (can be 0)
  quantity: number;                 // Usually 1
}

type TemperatureType = 'hot' | 'cold' | 'ambient' | 'frozen';
```

### 4.2.3 Location

```typescript
interface Location {
  // Coordinates
  latitude: number;                 // -90 to 90
  longitude: number;                // -180 to 180
  accuracy?: number;                // Meters

  // Address
  address: string;                  // Full formatted address
  addressLine1?: string;            // Street address
  addressLine2?: string;            // Apt/Suite
  city: string;
  state?: string;                   // State/Province
  postalCode?: string;
  country: string;                  // ISO 3166-1 alpha-2 (US, CA, etc.)

  // Details
  locationType?: 'restaurant' | 'residence' | 'office' | 'hotel' | 'other';
  parkingNotes?: string;
  accessCode?: string;              // Building entry code
  buzzerCode?: string;

  // Verification
  verified: boolean;                // Geocoding successful
  placeId?: string;                 // Google Places ID
}
```

### 4.2.4 Order Status

```typescript
type OrderStatus =
  | 'pending'         // Order created, awaiting confirmation
  | 'confirmed'       // Payment authorized, restaurant notified
  | 'preparing'       // Restaurant preparing food
  | 'ready'           // Food ready for pickup
  | 'assigned'        // Driver assigned
  | 'picked_up'       // Driver has food
  | 'in_transit'      // Driver en route to customer
  | 'arriving'        // Driver < 2 min away
  | 'delivered'       // Food delivered (awaiting confirmation)
  | 'completed'       // Order complete, payment captured
  | 'cancelled'       // Order cancelled
  | 'failed';         // Delivery failed

interface StatusChange {
  from: OrderStatus;
  to: OrderStatus;
  timestamp: Date;
  actor: string;                    // User/Driver/System ID
  reason?: string;
  location?: Location;
}
```

### 4.2.5 Example Order JSON

```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "confirmationCode": "ABC123",
  "restaurantId": "rest_789",
  "customerId": "cust_456",
  "driverId": "drv_321",

  "items": [
    {
      "id": "item_1",
      "name": "Margherita Pizza",
      "quantity": 1,
      "unitPrice": 1499,
      "totalPrice": 1499,
      "temperature": "hot",
      "modifiers": [
        {
          "id": "mod_1",
          "name": "Extra Cheese",
          "price": 200,
          "quantity": 1
        }
      ]
    },
    {
      "id": "item_2",
      "name": "Caesar Salad",
      "quantity": 1,
      "unitPrice": 899,
      "totalPrice": 899,
      "temperature": "cold"
    }
  ],

  "subtotal": 2598,
  "tax": 234,
  "deliveryFee": 499,
  "serviceFee": 390,
  "smallOrderFee": 0,
  "discount": -300,
  "tip": 500,
  "total": 3921,

  "pickupLocation": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "address": "123 Market St, San Francisco, CA 94103",
    "city": "San Francisco",
    "state": "CA",
    "postalCode": "94103",
    "country": "US",
    "locationType": "restaurant",
    "verified": true
  },

  "deliveryLocation": {
    "latitude": 37.7858,
    "longitude": -122.4068,
    "address": "456 Mission St, Apt 12B, San Francisco, CA 94105",
    "addressLine1": "456 Mission St",
    "addressLine2": "Apt 12B",
    "city": "San Francisco",
    "state": "CA",
    "postalCode": "94105",
    "country": "US",
    "locationType": "residence",
    "buzzerCode": "12B",
    "verified": true
  },

  "createdAt": "2025-01-15T18:00:00Z",
  "confirmedAt": "2025-01-15T18:00:15Z",
  "estimatedDelivery": "2025-01-15T18:45:00Z",

  "status": "in_transit",
  "statusHistory": [
    {
      "from": "pending",
      "to": "confirmed",
      "timestamp": "2025-01-15T18:00:15Z",
      "actor": "system"
    },
    {
      "from": "confirmed",
      "to": "preparing",
      "timestamp": "2025-01-15T18:05:00Z",
      "actor": "rest_789"
    }
  ],

  "distance": 2.3,
  "duration": 25,
  "temperatureRequirement": "hot",
  "contactlessDelivery": true,
  "utensils": false,
  "specialInstructions": "Please ring doorbell",

  "version": 5
}
```

---

## 4.3 Driver Entity

### 4.3.1 Complete Driver Structure

```typescript
interface Driver {
  // ========== Identity ==========
  id: string;                       // UUID
  externalId?: string;

  // ========== Personal ==========
  firstName: string;
  lastName: string;
  email: string;                    // Unique
  phone: string;                    // E.164 format: +14155551234
  photo?: string;                   // URL
  dateOfBirth: Date;

  // ========== Vehicle ==========
  vehicleType: VehicleType;
  vehicleMake?: string;             // "Honda"
  vehicleModel?: string;            // "Civic"
  vehicleYear?: number;             // 2020
  vehicleColor?: string;            // "Silver"
  licensePlate?: string;            // "ABC1234"

  // ========== Status ==========
  status: DriverStatus;
  isOnline: boolean;
  isAvailable: boolean;             // Online but not on active delivery

  // ========== Location ==========
  location: Location;
  heading?: number;                 // Degrees 0-359 (0 = North)
  speed?: number;                   // km/h
  lastLocationUpdate: Date;

  // ========== Performance ==========
  rating: number;                   // 0-5 (e.g., 4.87)
  totalDeliveries: number;
  completionRate: number;           // 0-1 (0.98 = 98%)
  onTimeRate: number;               // 0-1
  cancellationRate: number;         // 0-1
  acceptanceRate: number;           // 0-1
  responseTime: number;             // Seconds to accept order (avg)

  // ========== Equipment ==========
  equipment: DriverEquipment;

  // ========== Capacity ==========
  maxOrders: number;                // Simultaneous orders
  currentOrders: string[];          // Order IDs
  maxWeight: number;                // Kilograms
  maxVolume: number;                // Liters

  // ========== Earnings ==========
  earnings: DriverEarnings;

  // ========== Verification ==========
  backgroundCheckStatus: 'pending' | 'approved' | 'rejected' | 'expired';
  backgroundCheckDate?: Date;
  licenseVerified: boolean;
  licenseNumber?: string;
  licenseExpiration?: Date;
  insuranceVerified: boolean;
  insurancePolicyNumber?: string;
  insuranceExpiration?: Date;
  foodSafetyCertified: boolean;
  foodSafetyCertDate?: Date;

  // ========== Account ==========
  accountStatus: 'active' | 'inactive' | 'suspended' | 'deactivated';
  suspensionReason?: string;
  tier: 'bronze' | 'silver' | 'gold' | 'platinum';
  joinedAt: Date;
  lastActiveAt: Date;

  // ========== Preferences ==========
  preferredZones?: string[];        // Zone IDs
  maxDistance: number;              // km (willing to travel)
  workingHours?: TimeWindow[];

  // ========== Metadata ==========
  metadata?: Record<string, any>;
  version: number;
}

type VehicleType =
  | 'bike'           // Bicycle
  | 'ebike'          // Electric bicycle
  | 'scooter'        // Motor scooter
  | 'motorcycle'     // Motorcycle
  | 'car';           // Automobile

type DriverStatus =
  | 'offline'        // Not working
  | 'online'         // Available for orders
  | 'assigned'       // Order assigned, heading to pickup
  | 'picking_up'     // At restaurant
  | 'in_transit'     // Delivering
  | 'delivering'     // At customer location
  | 'break';         // On break

interface DriverEquipment {
  hasHotBag: boolean;
  hasColdBag: boolean;
  hasFrozenStorage: boolean;
  hasTemperatureSensor: boolean;
  hasInsulatedContainer: boolean;
  hasSmartphone: boolean;
  smartphoneModel?: string;

  // Bag specifications
  hotBagCapacity?: number;          // Liters
  coldBagCapacity?: number;         // Liters
}

interface DriverEarnings {
  today: number;                    // USD cents
  week: number;
  month: number;
  allTime: number;

  // Breakdown
  deliveryFees: number;             // Base + distance + time
  tips: number;
  bonuses: number;                  // Peak hour, quests, etc.

  // Deductions
  fees: number;                     // Platform fees
  adjustments: number;              // Corrections, penalties
}
```

### 4.3.2 Driver Metrics

```typescript
interface DriverMetrics {
  // Efficiency
  ordersPerHour: number;            // Average
  avgDeliveryTime: number;          // Minutes
  avgDistancePerOrder: number;      // Kilometers
  utilizationRate: number;          // Active time / Online time

  // Quality
  onTimeDeliveryRate: number;       // 0-1
  customerRating: number;           // 0-5
  orderAccuracy: number;            // 0-1
  temperatureCompliance: number;    // 0-1

  // Reliability
  completionRate: number;           // 0-1
  cancellationRate: number;         // 0-1
  acceptanceRate: number;           // 0-1
  responseTime: number;             // Seconds

  // Earnings
  totalEarnings: number;            // USD cents
  avgEarningsPerHour: number;
  avgEarningsPerDelivery: number;

  // Period
  period: TimePeriod;
}

interface TimePeriod {
  start: Date;
  end: Date;
}
```

### 4.3.3 Example Driver JSON

```json
{
  "id": "drv_321",
  "firstName": "John",
  "lastName": "Doe",
  "email": "john.doe@example.com",
  "phone": "+14155551234",
  "photo": "https://cdn.example.com/drivers/drv_321.jpg",

  "vehicleType": "ebike",
  "vehicleMake": "Rad Power",
  "vehicleModel": "RadRunner",
  "vehicleYear": 2024,
  "vehicleColor": "Black",

  "status": "in_transit",
  "isOnline": true,
  "isAvailable": false,

  "location": {
    "latitude": 37.7800,
    "longitude": -122.4150,
    "accuracy": 10,
    "address": "789 Howard St, San Francisco, CA 94103",
    "city": "San Francisco",
    "state": "CA",
    "country": "US",
    "verified": true
  },
  "heading": 45,
  "speed": 22,
  "lastLocationUpdate": "2025-01-15T18:30:00Z",

  "rating": 4.87,
  "totalDeliveries": 1523,
  "completionRate": 0.984,
  "onTimeRate": 0.923,
  "cancellationRate": 0.016,
  "acceptanceRate": 0.89,
  "responseTime": 12,

  "equipment": {
    "hasHotBag": true,
    "hasColdBag": true,
    "hasFrozenStorage": false,
    "hasTemperatureSensor": true,
    "hasInsulatedContainer": true,
    "hasSmartphone": true,
    "smartphoneModel": "iPhone 14",
    "hotBagCapacity": 40,
    "coldBagCapacity": 30
  },

  "maxOrders": 3,
  "currentOrders": ["order_123"],
  "maxWeight": 15,
  "maxVolume": 60,

  "earnings": {
    "today": 12450,
    "week": 85320,
    "month": 342180,
    "allTime": 4567890,
    "deliveryFees": 3200000,
    "tips": 1200000,
    "bonuses": 180000,
    "fees": 12110,
    "adjustments": 0
  },

  "backgroundCheckStatus": "approved",
  "backgroundCheckDate": "2024-06-15T00:00:00Z",
  "licenseVerified": true,
  "insuranceVerified": true,
  "foodSafetyCertified": true,
  "foodSafetyCertDate": "2024-07-01T00:00:00Z",

  "accountStatus": "active",
  "tier": "gold",
  "joinedAt": "2023-03-10T00:00:00Z",
  "lastActiveAt": "2025-01-15T18:30:00Z",

  "maxDistance": 15,

  "version": 42
}
```

---

## 4.4 Route Entity

### 4.4.1 Route Structure

```typescript
interface Route {
  id: string;                       // UUID
  driverId: string;
  orders: string[];                 // Order IDs

  // Route details
  stops: RouteStop[];
  totalDistance: number;            // Kilometers
  totalDuration: number;            // Minutes
  optimizationAlgorithm: string;    // "2-opt", "nearest-neighbor"
  optimizationTime: number;         // Milliseconds to compute

  // Waypoints (detailed path)
  waypoints: GeoPoint[];
  encodedPolyline?: string;         // Google polyline encoding

  // Performance
  estimatedCost: number;            // USD cents (fuel, time)
  fuelConsumption?: number;         // Liters
  co2Emissions?: number;            // Kilograms

  // Status
  status: 'planned' | 'active' | 'completed' | 'cancelled';
  startedAt?: Date;
  completedAt?: Date;

  // Deviations
  deviations: RouteDeviation[];
  reroutes: number;                 // Count of dynamic re-routes

  // Metadata
  metadata?: Record<string, any>;
}

interface RouteStop {
  sequence: number;                 // 1, 2, 3, ...
  type: 'pickup' | 'delivery';
  orderId: string;
  location: Location;

  // Timing
  estimatedArrival: Date;
  actualArrival?: Date;
  estimatedDeparture: Date;
  actualDeparture?: Date;
  serviceDuration: number;          // Minutes

  // Status
  completed: boolean;
  skipped: boolean;
  notes?: string;
}

interface GeoPoint {
  latitude: number;
  longitude: number;
  timestamp?: Date;
}

interface RouteDeviation {
  timestamp: Date;
  location: Location;
  reason: string;                   // "traffic", "driver_deviation", "customer_unavailable"
  distanceOff: number;              // Meters from planned route
  timeImpact: number;               // Minutes added
}
```

### 4.4.2 Example Route JSON

```json
{
  "id": "route_555",
  "driverId": "drv_321",
  "orders": ["order_123", "order_124"],

  "stops": [
    {
      "sequence": 1,
      "type": "pickup",
      "orderId": "order_123",
      "location": {
        "latitude": 37.7749,
        "longitude": -122.4194,
        "address": "123 Market St, San Francisco, CA"
      },
      "estimatedArrival": "2025-01-15T18:10:00Z",
      "actualArrival": "2025-01-15T18:11:30Z",
      "estimatedDeparture": "2025-01-15T18:15:00Z",
      "actualDeparture": "2025-01-15T18:14:00Z",
      "serviceDuration": 5,
      "completed": true
    },
    {
      "sequence": 2,
      "type": "pickup",
      "orderId": "order_124",
      "location": {
        "latitude": 37.7755,
        "longitude": -122.4185,
        "address": "150 Market St, San Francisco, CA"
      },
      "estimatedArrival": "2025-01-15T18:17:00Z",
      "serviceDuration": 5,
      "completed": false
    },
    {
      "sequence": 3,
      "type": "delivery",
      "orderId": "order_123",
      "location": {
        "latitude": 37.7858,
        "longitude": -122.4068,
        "address": "456 Mission St, San Francisco, CA"
      },
      "estimatedArrival": "2025-01-15T18:30:00Z",
      "serviceDuration": 3,
      "completed": false
    },
    {
      "sequence": 4,
      "type": "delivery",
      "orderId": "order_124",
      "location": {
        "latitude": 37.7865,
        "longitude": -122.4055,
        "address": "500 Mission St, San Francisco, CA"
      },
      "estimatedArrival": "2025-01-15T18:35:00Z",
      "serviceDuration": 3,
      "completed": false
    }
  ],

  "totalDistance": 4.8,
  "totalDuration": 25,
  "optimizationAlgorithm": "2-opt",
  "optimizationTime": 342,

  "waypoints": [
    {"latitude": 37.7749, "longitude": -122.4194},
    {"latitude": 37.7755, "longitude": -122.4192},
    {"latitude": 37.7760, "longitude": -122.4180}
  ],

  "estimatedCost": 850,
  "fuelConsumption": 0.3,
  "co2Emissions": 0.8,

  "status": "active",
  "startedAt": "2025-01-15T18:10:00Z",

  "deviations": [],
  "reroutes": 0
}
```

---

## 4.5 Temperature Monitoring

### 4.5.1 Temperature Reading

```typescript
interface TemperatureReading {
  id: string;                       // UUID
  timestamp: Date;                  // ISO 8601

  // Association
  orderId: string;
  driverId: string;
  sensorId: string;

  // Measurement
  temperature: number;              // Celsius
  humidity?: number;                // Percentage 0-100
  batteryLevel: number;             // Percentage 0-100

  // Location
  location: Location;

  // Status
  inCompliance: boolean;
  alertLevel?: 'warning' | 'critical' | 'severe';

  // Metadata
  sensorType: 'bluetooth' | 'wifi' | 'cellular';
  accuracy: number;                 // ±degrees
}

interface TemperatureAlert {
  id: string;
  timestamp: Date;
  orderId: string;
  driverId: string;
  level: 'warning' | 'critical' | 'severe';
  temperature: number;              // Celsius
  threshold: number;                // Celsius
  duration: number;                 // Seconds out of compliance
  action: string;                   // Action taken
  resolved: boolean;
  resolvedAt?: Date;
}
```

### 4.5.2 Example Temperature Reading JSON

```json
{
  "id": "temp_999",
  "timestamp": "2025-01-15T18:25:00Z",
  "orderId": "order_123",
  "driverId": "drv_321",
  "sensorId": "sensor_ABC123",

  "temperature": 62.5,
  "humidity": 45,
  "batteryLevel": 78,

  "location": {
    "latitude": 37.7820,
    "longitude": -122.4100,
    "accuracy": 15
  },

  "inCompliance": true,
  "sensorType": "bluetooth",
  "accuracy": 0.5
}
```

---

## 4.6 Validation Rules

### 4.6.1 Order Validation

```typescript
class OrderValidator {
  validateOrder(order: Order): ValidationResult {
    const errors: string[] = [];

    // Required fields
    if (!order.restaurantId) errors.push("restaurantId is required");
    if (!order.customerId) errors.push("customerId is required");
    if (!order.items || order.items.length === 0) {
      errors.push("At least one item is required");
    }

    // Items validation
    order.items.forEach((item, index) => {
      if (item.quantity <= 0) {
        errors.push(`Item ${index}: quantity must be > 0`);
      }
      if (item.unitPrice < 0) {
        errors.push(`Item ${index}: unitPrice cannot be negative`);
      }
    });

    // Location validation
    if (!this.isValidLatitude(order.deliveryLocation.latitude)) {
      errors.push("Invalid delivery latitude");
    }
    if (!this.isValidLongitude(order.deliveryLocation.longitude)) {
      errors.push("Invalid delivery longitude");
    }

    // Financial validation
    const calculatedSubtotal = order.items.reduce(
      (sum, item) => sum + item.totalPrice, 0
    );
    if (Math.abs(calculatedSubtotal - order.subtotal) > 1) {
      errors.push("Subtotal mismatch");
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }

  private isValidLatitude(lat: number): boolean {
    return lat >= -90 && lat <= 90;
  }

  private isValidLongitude(lng: number): boolean {
    return lng >= -180 && lng <= 180;
  }
}
```

---

## 4.7 Summary

This chapter defined all core data structures:
- **Order**: Complete order lifecycle with items, locations, status
- **Driver**: Profile, equipment, performance, earnings
- **Route**: Optimized multi-stop routes with deviations
- **Temperature**: Real-time monitoring with alerts

All structures use:
- **JSON** for data exchange
- **TypeScript** for type safety
- **Standard formats** (ISO 8601, E.164, UUID)
- **Clear validation** rules
- **Extensibility** via metadata fields

---

**Next Chapter**: [Chapter 5: API Interface Specification →](05-api-interface.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
