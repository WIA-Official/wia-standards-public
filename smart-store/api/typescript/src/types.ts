/**
 * WIA-IND-021: Smart Store - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * 2D position in store
 */
export interface Point {
  x: number; // meters from origin
  y: number; // meters from origin
}

/**
 * Geographic coordinates
 */
export interface GeoCoordinate {
  latitude: number;
  longitude: number;
  altitude?: number;
}

/**
 * Store zone types
 */
export type ZoneType =
  | 'entry'
  | 'shopping'
  | 'checkout'
  | 'exit'
  | 'backroom'
  | 'produce'
  | 'dairy'
  | 'bakery'
  | 'meat'
  | 'frozen'
  | 'beverages'
  | 'snacks'
  | 'household'
  | 'electronics'
  | 'apparel';

/**
 * Authentication methods
 */
export type AuthMethod =
  | 'mobile-app'
  | 'payment-card'
  | 'face-recognition'
  | 'qr-code'
  | 'fingerprint';

/**
 * Detection methods for product identification
 */
export type DetectionMethod = 'vision' | 'rfid' | 'weight' | 'hybrid';

/**
 * Customer action types
 */
export type CustomerAction = 'browsing' | 'picking' | 'examining' | 'returning' | 'moving';

// ============================================================================
// Checkout Session
// ============================================================================

/**
 * Checkout session status
 */
export type SessionStatus = 'active' | 'completed' | 'disputed' | 'cancelled';

/**
 * Checkout session
 */
export interface CheckoutSession {
  /** Unique session identifier */
  sessionId: string;

  /** Customer identifier */
  customerId: string;

  /** Store identifier */
  storeId: string;

  /** Entry timestamp */
  entryTime: Date;

  /** Exit timestamp (if completed) */
  exitTime?: Date;

  /** Authentication method used */
  authMethod: AuthMethod;

  /** Virtual shopping cart */
  virtualCart: CartItem[];

  /** Total amount (currency) */
  totalAmount: number;

  /** Tax amount */
  taxAmount: number;

  /** Discount amount */
  discountAmount: number;

  /** Session status */
  status: SessionStatus;

  /** Overall confidence score (0-1) */
  confidence: number;

  /** Payment method ID */
  paymentMethodId?: string;

  /** Transaction ID (after payment) */
  transactionId?: string;
}

/**
 * Cart item
 */
export interface CartItem {
  /** Product identifier */
  productId: string;

  /** Product SKU */
  sku: string;

  /** Product name */
  productName: string;

  /** Quantity */
  quantity: number;

  /** Unit price */
  price: number;

  /** Total price (quantity × price) */
  totalPrice: number;

  /** Timestamp when added */
  addedAt: Date;

  /** Confidence score (0-1) */
  confidence: number;

  /** Detection method */
  detectionMethod: DetectionMethod;

  /** Shelf ID where picked */
  shelfId?: string;
}

// ============================================================================
// Product & Inventory
// ============================================================================

/**
 * Product information
 */
export interface Product {
  /** Product identifier */
  productId: string;

  /** Stock keeping unit */
  sku: string;

  /** Product name */
  name: string;

  /** Product category */
  category: string;

  /** Brand name */
  brand: string;

  /** Price information */
  price: {
    amount: number;
    currency: string;
    unit: string; // "each", "kg", "lb"
  };

  /** Weight information */
  weight: {
    value: number;
    unit: 'g' | 'kg' | 'lb' | 'oz';
  };

  /** Dimensions */
  dimensions?: {
    length: number;
    width: number;
    height: number;
    unit: 'cm' | 'in';
  };

  /** Barcode */
  barcode: string;

  /** RFID tag (if applicable) */
  rfidTag?: string;

  /** Product images */
  images: string[];

  /** Nutritional information */
  nutrition?: NutritionInfo;

  /** Additional metadata */
  metadata: Record<string, any>;
}

/**
 * Nutrition information
 */
export interface NutritionInfo {
  servingSize: string;
  calories: number;
  totalFat: number; // grams
  saturatedFat: number;
  cholesterol: number; // mg
  sodium: number; // mg
  totalCarbohydrate: number; // grams
  dietaryFiber: number;
  sugars: number;
  protein: number; // grams
  allergens: string[];
}

/**
 * Inventory item
 */
export interface InventoryItem {
  /** Product identifier */
  productId: string;

  /** SKU */
  sku: string;

  /** Locations where product is stored */
  locations: ItemLocation[];

  /** Total stock across all locations */
  totalStock: number;

  /** Stock status */
  status: 'in-stock' | 'low-stock' | 'out-of-stock' | 'overstock';

  /** Reorder point */
  reorderPoint: number;

  /** Reorder quantity */
  reorderQuantity: number;

  /** Track expiry dates? */
  expiryTracking: boolean;

  /** Last updated */
  lastUpdated: Date;
}

/**
 * Item location
 */
export interface ItemLocation {
  /** Location type */
  locationType: 'shelf' | 'backroom' | 'warehouse' | 'transit';

  /** Location identifier */
  locationId: string;

  /** Quantity at this location */
  quantity: number;

  /** Last counted */
  lastCounted: Date;

  /** Last movement */
  lastMovement?: Date;

  /** Expiry dates (for perishables) */
  expiryDates?: Date[];
}

// ============================================================================
// Smart Shelf
// ============================================================================

/**
 * Smart shelf
 */
export interface SmartShelf {
  /** Shelf identifier */
  shelfId: string;

  /** Location information */
  location: {
    zone: ZoneType;
    aisle: string;
    section: string;
    level: number; // 1=bottom, 5=top
  };

  /** Position in store */
  position: Point;

  /** Sensors attached to shelf */
  sensors: {
    weightSensors: WeightSensor[];
    rfidReaders: RFIDReader[];
    opticalSensors: OpticalSensor[];
    tempSensor?: TempSensor;
  };

  /** Products on shelf */
  products: ShelfProduct[];

  /** Capacity */
  capacity: {
    maxWeight: number; // kg
    maxProducts: number;
  };

  /** Status */
  status: 'active' | 'maintenance' | 'offline';

  /** Last maintenance */
  lastMaintenance?: Date;
}

/**
 * Weight sensor
 */
export interface WeightSensor {
  sensorId: string;
  currentWeight: number; // grams
  maxCapacity: number; // grams
  precision: number; // grams
  lastCalibration: Date;
  status: 'active' | 'error';
}

/**
 * RFID reader
 */
export interface RFIDReader {
  readerId: string;
  frequency: string; // "860-960 MHz"
  readRange: number; // meters
  tagsDetected: string[];
  status: 'active' | 'error';
}

/**
 * Optical sensor
 */
export interface OpticalSensor {
  sensorId: string;
  type: 'infrared' | 'laser';
  isBlocked: boolean;
  status: 'active' | 'error';
}

/**
 * Temperature sensor
 */
export interface TempSensor {
  sensorId: string;
  currentTemp: number; // Celsius
  minThreshold: number;
  maxThreshold: number;
  status: 'active' | 'warning' | 'critical' | 'error';
}

/**
 * Product on shelf
 */
export interface ShelfProduct {
  /** Product identifier */
  productId: string;

  /** SKU */
  sku: string;

  /** Product name */
  name: string;

  /** Expected unit weight */
  expectedWeight: number; // grams

  /** RFID tag */
  rfidTag?: string;

  /** Current quantity */
  quantity: number;

  /** Minimum stock level */
  minStock: number;

  /** Maximum stock level */
  maxStock: number;

  /** Position on shelf */
  position: {
    x: number; // cm from left
    y: number; // cm from front
    facings: number; // number of items facing forward
  };
}

// ============================================================================
// Customer Tracking & Analytics
// ============================================================================

/**
 * Customer trajectory
 */
export interface CustomerTrajectory {
  /** Session identifier */
  sessionId: string;

  /** Anonymous customer ID */
  customerId: string;

  /** Path through store */
  path: PathPoint[];

  /** Total distance traveled */
  totalDistance: number; // meters

  /** Average speed */
  avgSpeed: number; // meters/second

  /** Zones visited */
  visitedZones: string[];

  /** Dwell time per zone */
  dwellTimes: Map<string, number>; // zone -> milliseconds

  /** Start time */
  startTime: Date;

  /** End time */
  endTime?: Date;
}

/**
 * Point on customer path
 */
export interface PathPoint {
  /** X coordinate */
  x: number;

  /** Y coordinate */
  y: number;

  /** Timestamp */
  timestamp: Date;

  /** Current zone */
  zone: string;

  /** Customer action */
  action?: CustomerAction;
}

/**
 * Heatmap data
 */
export interface Heatmap {
  /** Store identifier */
  storeId: string;

  /** Heatmap type */
  type: 'density' | 'flow' | 'engagement';

  /** Grid size */
  gridSize: number; // meters

  /** Grid dimensions */
  dimensions: {
    width: number; // cells
    height: number; // cells
  };

  /** Grid data (2D array) */
  grid: number[][];

  /** Time period */
  period: {
    startTime: Date;
    endTime: Date;
  };

  /** Number of trajectories used */
  sampleSize: number;

  /** Metadata */
  metadata: {
    maxValue: number;
    minValue: number;
    avgValue: number;
  };
}

/**
 * Zone dwell time statistics
 */
export interface ZoneDwellTime {
  /** Zone identifier */
  zone: string;

  /** Total customers */
  totalCustomers: number;

  /** Average dwell time */
  avgDwellTime: number; // seconds

  /** Minimum dwell time */
  minDwellTime: number;

  /** Maximum dwell time */
  maxDwellTime: number;

  /** Median dwell time */
  percentile50: number;

  /** 90th percentile */
  percentile90: number;
}

// ============================================================================
// Digital Signage & ESL
// ============================================================================

/**
 * Digital signage display
 */
export interface DigitalSignage {
  /** Display identifier */
  displayId: string;

  /** Location */
  location: {
    zone: string;
    coordinates: Point;
  };

  /** Hardware specifications */
  hardware: {
    size: string; // "32-inch"
    resolution: string; // "1920x1080"
    orientation: 'portrait' | 'landscape';
  };

  /** Content */
  content: {
    currentPlaylist: Playlist;
    schedule: ContentSchedule[];
  };

  /** Capabilities */
  capabilities: {
    touchscreen: boolean;
    audio: boolean;
    camera: boolean;
  };

  /** Status */
  status: 'online' | 'offline' | 'error';
}

/**
 * Content playlist
 */
export interface Playlist {
  /** Playlist identifier */
  playlistId: string;

  /** Playlist name */
  name: string;

  /** Content items */
  items: ContentItem[];

  /** Total duration */
  duration: number; // seconds

  /** Loop mode */
  loopMode: 'continuous' | 'scheduled';
}

/**
 * Content item
 */
export interface ContentItem {
  /** Item identifier */
  itemId: string;

  /** Content type */
  type: 'video' | 'image' | 'html' | 'live-data';

  /** Source URL or path */
  source: string;

  /** Duration (for images) */
  duration: number; // seconds

  /** Transition effect */
  transition: 'fade' | 'slide' | 'none';

  /** Conditional triggers */
  triggers?: Trigger[];
}

/**
 * Content trigger
 */
export interface Trigger {
  /** Trigger type */
  type: 'time' | 'proximity' | 'weather' | 'inventory';

  /** Condition */
  condition: string;

  /** Action */
  action: 'play' | 'skip' | 'switch-playlist';
}

/**
 * Content schedule
 */
export interface ContentSchedule {
  /** Schedule identifier */
  scheduleId: string;

  /** Playlist to play */
  playlistId: string;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Days of week (0=Sunday, 6=Saturday) */
  daysOfWeek: number[];

  /** Priority (higher = precedence) */
  priority: number;
}

/**
 * Electronic shelf label
 */
export interface ElectronicShelfLabel {
  /** Label identifier */
  labelId: string;

  /** Product identifier */
  productId: string;

  /** Shelf identifier */
  shelfId: string;

  /** Display specifications */
  display: {
    size: string; // "2.9-inch"
    template: 'standard' | 'promotional' | 'nutritional';
  };

  /** Content */
  content: {
    productName: string;
    price: number;
    unit: string; // "per kg", "each"
    promotionTag?: string; // "SALE", "NEW"
    barcode: string;
    additionalInfo?: string;
  };

  /** Battery status */
  battery: {
    level: number; // 0-100%
    lastChanged: Date;
  };

  /** Last update */
  lastUpdate: Date;

  /** Update frequency */
  updateFrequency: 'realtime' | 'hourly' | 'daily';
}

// ============================================================================
// Smart Shopping Cart
// ============================================================================

/**
 * Smart shopping cart
 */
export interface SmartShoppingCart {
  /** Cart identifier */
  cartId: string;

  /** Session identifier (if in use) */
  sessionId?: string;

  /** Customer identifier (if linked) */
  customerId?: string;

  /** Current location */
  location: {
    x: number;
    y: number;
    lastUpdate: Date;
  };

  /** Items in cart */
  items: CartItem[];

  /** Total weight */
  totalWeight: number; // kg

  /** Total value */
  totalValue: number;

  /** Battery status */
  battery: {
    level: number; // 0-100%
    charging: boolean;
  };

  /** Navigation */
  navigation: {
    enabled: boolean;
    destination?: string;
    route?: PathPoint[];
  };

  /** Features enabled */
  features: {
    selfCheckout: boolean;
    productRecommendations: boolean;
    couponNotifications: boolean;
    receiptPrinting: boolean;
  };

  /** Status */
  status: 'available' | 'in-use' | 'charging' | 'maintenance';
}

// ============================================================================
// Recommendations
// ============================================================================

/**
 * Recommendation request
 */
export interface RecommendationRequest {
  /** Customer identifier */
  customerId: string;

  /** Context */
  context: {
    currentLocation?: ZoneType;
    cartItems?: string[]; // Product IDs
    time: Date;
    weather?: WeatherCondition;
  };

  /** Maximum recommendations */
  limit: number;
}

/**
 * Product recommendation
 */
export interface Recommendation {
  /** Product identifier */
  productId: string;

  /** Relevance score (0-1) */
  score: number;

  /** Explanation */
  reason: string;

  /** Recommendation type */
  type: 'collaborative' | 'content' | 'contextual' | 'complementary';
}

/**
 * Weather condition
 */
export interface WeatherCondition {
  temperature: number; // Celsius
  condition: 'sunny' | 'rainy' | 'cloudy' | 'snowy' | 'windy';
  humidity: number; // percentage
}

/**
 * Customer profile
 */
export interface CustomerProfile {
  /** Customer identifier */
  customerId: string;

  /** Demographics */
  demographics: {
    ageGroup?: string;
    location?: string;
  };

  /** Preferences */
  preferences: {
    categories: Map<string, number>; // Category → affinity score
    brands: Map<string, number>;
    priceRange: { min: number; max: number };
    dietaryRestrictions: string[];
  };

  /** Purchase history */
  purchaseHistory: {
    totalPurchases: number;
    avgBasketSize: number;
    avgVisitFrequency: number; // days
    favoriteProducts: string[];
    lastPurchase: Date;
  };

  /** Behavior */
  behavior: {
    avgDwellTime: number; // seconds
    preferredShoppingTime: string;
    avgPathLength: number; // meters
    visitedZones: Map<string, number>;
  };

  /** Loyalty tier */
  loyaltyTier: 'bronze' | 'silver' | 'gold' | 'platinum';
}

// ============================================================================
// Store Configuration
// ============================================================================

/**
 * Store configuration
 */
export interface StoreConfig {
  /** Store identifier */
  storeId: string;

  /** Store name */
  name: string;

  /** Location */
  location: GeoCoordinate;

  /** Store dimensions */
  dimensions: {
    width: number; // meters
    length: number; // meters
    height: number; // meters
  };

  /** Features enabled */
  features: {
    automatedCheckout: boolean;
    computerVision: boolean;
    smartShelves: boolean;
    digitalSignage: boolean;
    smartCarts: boolean;
    indoorNavigation: boolean;
    recommendations: boolean;
  };

  /** Operating hours */
  hours: {
    monday: { open: string; close: string };
    tuesday: { open: string; close: string };
    wednesday: { open: string; close: string };
    thursday: { open: string; close: string };
    friday: { open: string; close: string };
    saturday: { open: string; close: string };
    sunday: { open: string; close: string };
  };

  /** Capacity */
  capacity: {
    maxCustomers: number;
    currentOccupancy: number;
  };

  /** Status */
  status: 'open' | 'closed' | 'maintenance';
}

// ============================================================================
// Analytics & Reporting
// ============================================================================

/**
 * Store analytics
 */
export interface StoreAnalytics {
  /** Store identifier */
  storeId: string;

  /** Time period */
  period: {
    startDate: Date;
    endDate: Date;
  };

  /** Traffic metrics */
  traffic: {
    totalVisitors: number;
    uniqueVisitors: number;
    avgVisitDuration: number; // seconds
    peakHours: string[];
  };

  /** Sales metrics */
  sales: {
    totalTransactions: number;
    totalRevenue: number;
    avgBasketSize: number;
    avgBasketValue: number;
    conversionRate: number; // percentage
  };

  /** Inventory metrics */
  inventory: {
    stockoutEvents: number;
    shrinkageRate: number; // percentage
    inventoryTurnover: number;
    avgStockLevel: number;
  };

  /** Performance metrics */
  performance: {
    avgCheckoutTime: number; // seconds
    systemUptime: number; // percentage
    recognitionAccuracy: number; // percentage
    customerSatisfaction: number; // 0-5 rating
  };
}

// ============================================================================
// Events & Alerts
// ============================================================================

/**
 * Event types
 */
export type EventType =
  | 'checkout-start'
  | 'checkout-complete'
  | 'product-interaction'
  | 'inventory-low'
  | 'inventory-out'
  | 'sensor-error'
  | 'system-error'
  | 'security-alert'
  | 'fraud-detected'
  | 'customer-feedback';

/**
 * Store event
 */
export interface StoreEvent {
  /** Event identifier */
  eventId: string;

  /** Event type */
  type: EventType;

  /** Timestamp */
  timestamp: Date;

  /** Store identifier */
  storeId: string;

  /** Severity */
  severity: 'info' | 'warning' | 'error' | 'critical';

  /** Event data */
  data: Record<string, any>;

  /** Acknowledged */
  acknowledged: boolean;

  /** Resolved */
  resolved: boolean;
}

/**
 * Alert configuration
 */
export interface AlertConfig {
  /** Alert identifier */
  alertId: string;

  /** Event type to monitor */
  eventType: EventType;

  /** Condition */
  condition: string;

  /** Recipients */
  recipients: string[]; // Email addresses or user IDs

  /** Notification channels */
  channels: ('email' | 'sms' | 'push' | 'webhook')[];

  /** Enabled */
  enabled: boolean;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * API response wrapper
 */
export interface APIResponse<T> {
  /** Success flag */
  success: boolean;

  /** Response data */
  data?: T;

  /** Error message */
  error?: string;

  /** Timestamp */
  timestamp: Date;

  /** Request ID (for tracking) */
  requestId: string;
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  /** Page number (1-indexed) */
  page: number;

  /** Items per page */
  limit: number;

  /** Sort field */
  sortBy?: string;

  /** Sort order */
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Items */
  items: T[];

  /** Total items */
  total: number;

  /** Current page */
  page: number;

  /** Items per page */
  limit: number;

  /** Total pages */
  totalPages: number;

  /** Has next page */
  hasNext: boolean;

  /** Has previous page */
  hasPrevious: boolean;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Zone categories
 */
export const ZONE_CATEGORIES = {
  ENTRY: 'entry',
  SHOPPING: 'shopping',
  CHECKOUT: 'checkout',
  EXIT: 'exit',
  PRODUCE: 'produce',
  DAIRY: 'dairy',
  BAKERY: 'bakery',
  MEAT: 'meat',
  FROZEN: 'frozen',
} as const;

/**
 * Default confidence thresholds
 */
export const CONFIDENCE_THRESHOLDS = {
  HIGH: 0.95,
  MEDIUM: 0.85,
  LOW: 0.70,
  MINIMUM: 0.60,
} as const;

/**
 * Sensor status codes
 */
export const SENSOR_STATUS = {
  ACTIVE: 'active',
  ERROR: 'error',
  MAINTENANCE: 'maintenance',
  OFFLINE: 'offline',
} as const;

/**
 *弘益人間 (Benefit All Humanity)
 */
