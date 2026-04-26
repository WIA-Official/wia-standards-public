/**
 * WIA-IND-009: Food Delivery Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Food Delivery Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * Comprehensive type definitions for food delivery systems including:
 * - Order management and lifecycle
 * - Driver logistics and tracking
 * - Route optimization
 * - Temperature monitoring
 * - Quality assurance
 */

// ============================================================================
// Core Enums and Types
// ============================================================================

/**
 * Order lifecycle states
 * 주문 생명주기 상태
 */
export type OrderStatus =
  | 'pending'           // 대기중 - Order created, awaiting confirmation
  | 'confirmed'         // 확인됨 - Restaurant confirmed order
  | 'preparing'         // 준비중 - Food being prepared
  | 'ready'             // 준비완료 - Food ready for pickup
  | 'assigned'          // 배정됨 - Driver assigned
  | 'picked_up'         // 픽업완료 - Driver picked up order
  | 'in_transit'        // 배달중 - En route to customer
  | 'arriving'          // 도착예정 - Driver approaching (< 2 min)
  | 'delivered'         // 배달완료 - Order delivered
  | 'completed'         // 완료 - Order completed and paid
  | 'cancelled'         // 취소됨 - Order cancelled
  | 'failed';           // 실패 - Delivery failed

/**
 * Driver status states
 * 배달원 상태
 */
export type DriverStatus =
  | 'offline'           // 오프라인 - Not working
  | 'online'            // 온라인 - Available to receive orders
  | 'available'         // 대기중 - Ready for assignment
  | 'assigned'          // 배정됨 - Order assigned, heading to pickup
  | 'en_route_pickup'   // 픽업이동중 - Driving to restaurant
  | 'at_restaurant'     // 레스토랑도착 - At pickup location
  | 'picking_up'        // 픽업중 - Collecting order
  | 'loaded'            // 적재완료 - Order loaded, ready to deliver
  | 'in_transit'        // 배달중 - Delivering to customer
  | 'delivering'        // 전달중 - At customer location, handing off
  | 'returning';        // 복귀중 - Returning to service area

/**
 * Vehicle types for delivery
 * 배달 수단 종류
 */
export type VehicleType =
  | 'bike'              // 자전거 - Bicycle
  | 'ebike'             // 전기자전거 - Electric bicycle
  | 'scooter'           // 스쿠터 - Electric scooter
  | 'motorcycle'        // 오토바이 - Motorcycle
  | 'car'               // 자동차 - Car
  | 'van';              // 밴 - Van for large orders

/**
 * Food temperature categories
 * 음식 온도 카테고리
 */
export type TemperatureCategory =
  | 'hot'               // 뜨거운 음식 - ≥60°C (pizza, hot meals)
  | 'cold'              // 차가운 음식 - ≤4°C (salads, dairy)
  | 'frozen'            // 냉동 음식 - ≤-18°C (ice cream)
  | 'ambient';          // 상온 음식 - 15-25°C (packaged snacks)

/**
 * Route optimization algorithms
 * 경로 최적화 알고리즘
 */
export type OptimizationAlgorithm =
  | 'nearest_neighbor'  // 최근접 이웃 - Greedy approach
  | 'tsp_exact'         // TSP 정확해 - Exact solution (small n)
  | 'tsp_2opt'          // TSP 2-opt - 2-opt heuristic
  | 'genetic'           // 유전 알고리즘 - Genetic algorithm
  | 'vrp'               // VRP - Vehicle routing problem
  | 'dynamic';          // 동적 최적화 - Real-time dynamic routing

/**
 * Delivery priority levels
 * 배달 우선순위
 */
export type DeliveryPriority =
  | 'low'               // 낮음 - Standard delivery
  | 'normal'            // 보통 - Default priority
  | 'high'              // 높음 - Priority delivery
  | 'urgent'            // 긴급 - Express delivery
  | 'scheduled';        // 예약 - Scheduled for specific time

/**
 * Payment methods
 * 결제 수단
 */
export type PaymentMethod =
  | 'credit_card'       // 신용카드
  | 'debit_card'        // 직불카드
  | 'digital_wallet'    // 디지털 지갑 (Apple Pay, Google Pay)
  | 'cash'              // 현금
  | 'voucher'           // 상품권
  | 'corporate';        // 법인카드

// ============================================================================
// Location and Geography Types
// ============================================================================

/**
 * Geographic coordinates
 * 지리적 좌표
 */
export interface GeoLocation {
  /** Latitude in decimal degrees (-90 to 90) */
  latitude: number;

  /** Longitude in decimal degrees (-180 to 180) */
  longitude: number;

  /** Accuracy in meters (optional) */
  accuracy?: number;

  /** Altitude in meters (optional) */
  altitude?: number;

  /** Heading/bearing in degrees (0-360, optional) */
  heading?: number;

  /** Speed in km/h (optional) */
  speed?: number;

  /** Timestamp of location reading */
  timestamp: Date;
}

/**
 * Complete address information
 * 완전한 주소 정보
 */
export interface Address {
  /** Street address line 1 */
  street1: string;

  /** Street address line 2 (apt, suite, etc.) */
  street2?: string;

  /** City */
  city: string;

  /** State/province */
  state: string;

  /** Postal/ZIP code */
  postalCode: string;

  /** Country */
  country: string;

  /** Geographic coordinates */
  location: GeoLocation;

  /** Formatted address string */
  formatted?: string;

  /** Delivery instructions */
  instructions?: string;
}

/**
 * Service area boundary
 * 서비스 지역 경계
 */
export interface ServiceArea {
  /** Area identifier */
  id: string;

  /** Area name */
  name: string;

  /** Center point */
  center: GeoLocation;

  /** Radius in kilometers */
  radius: number;

  /** Polygon boundary (if not circular) */
  boundary?: GeoLocation[];

  /** Is area currently active */
  active: boolean;

  /** Minimum order value for this area */
  minimumOrder?: number;

  /** Delivery fee for this area */
  deliveryFee?: number;
}

// ============================================================================
// Order Management Types
// ============================================================================

/**
 * Order item details
 * 주문 항목 상세
 */
export interface OrderItem {
  /** Item identifier */
  id: string;

  /** Item name */
  name: string;

  /** Quantity */
  quantity: number;

  /** Unit price */
  price: number;

  /** Total price (quantity × price) */
  total: number;

  /** Temperature category */
  temperature: TemperatureCategory;

  /** Item modifiers/customizations */
  modifiers?: string[];

  /** Special preparation requests */
  specialRequests?: string;

  /** Item photo URL */
  photo?: string;

  /** Calories (optional) */
  calories?: number;

  /** Allergens (optional) */
  allergens?: string[];

  /** Spice level (optional, 0-5) */
  spiceLevel?: number;
}

/**
 * Complete order details
 * 완전한 주문 정보
 */
export interface Order {
  // ========== Identity ==========
  /** Unique order identifier */
  id: string;

  /** External/partner system ID */
  externalId?: string;

  /** Order number (human-readable) */
  orderNumber: string;

  // ========== Parties ==========
  /** Restaurant ID */
  restaurantId: string;

  /** Customer ID */
  customerId: string;

  /** Assigned driver ID */
  driverId?: string;

  // ========== Items and Pricing ==========
  /** Order items */
  items: OrderItem[];

  /** Subtotal (items only) */
  subtotal: number;

  /** Delivery fee */
  deliveryFee: number;

  /** Service fee */
  serviceFee: number;

  /** Small order fee */
  smallOrderFee: number;

  /** Tax amount */
  tax: number;

  /** Tip amount */
  tip: number;

  /** Discount amount */
  discount: number;

  /** Grand total */
  total: number;

  /** Surge multiplier (1.0 = no surge) */
  surgeMultiplier: number;

  // ========== Locations ==========
  /** Pickup location (restaurant) */
  pickupLocation: Address;

  /** Delivery location (customer) */
  deliveryLocation: Address;

  /** Distance in kilometers */
  distance: number;

  // ========== Timing ==========
  /** Order created timestamp */
  createdAt: Date;

  /** Order confirmed timestamp */
  confirmedAt?: Date;

  /** Food preparation started */
  preparingAt?: Date;

  /** Food ready for pickup */
  readyAt?: Date;

  /** Driver assigned timestamp */
  assignedAt?: Date;

  /** Order picked up timestamp */
  pickedUpAt?: Date;

  /** Order delivered timestamp */
  deliveredAt?: Date;

  /** Order completed timestamp */
  completedAt?: Date;

  /** Estimated delivery time */
  estimatedDelivery: Date;

  /** Promised delivery time (to customer) */
  promisedDelivery: Date;

  /** Actual delivery time */
  actualDelivery?: Date;

  /** Scheduled delivery time (if scheduled) */
  scheduledTime?: Date;

  // ========== Status and Tracking ==========
  /** Current order status */
  status: OrderStatus;

  /** Status change history */
  statusHistory: StatusChange[];

  /** Delivery priority */
  priority: DeliveryPriority;

  // ========== Logistics ==========
  /** Optimized route */
  route?: Route;

  /** Temperature monitoring log */
  temperatureLog?: TemperatureReading[];

  /** Photo proof of delivery */
  deliveryPhoto?: string;

  /** Signature (if required) */
  signature?: string;

  // ========== Preferences ==========
  /** Contactless delivery requested */
  contactlessDelivery: boolean;

  /** Include utensils */
  includeUtensils: boolean;

  /** Special instructions */
  specialInstructions?: string;

  /** Leave at door */
  leaveAtDoor: boolean;

  /** Ring doorbell */
  ringDoorbell: boolean;

  // ========== Payment ==========
  /** Payment method */
  paymentMethod: PaymentMethod;

  /** Payment status */
  paymentStatus: 'pending' | 'authorized' | 'captured' | 'refunded' | 'failed';

  /** Transaction ID */
  transactionId?: string;

  /** Promo code applied */
  promoCode?: string;

  // ========== Quality and Feedback ==========
  /** Customer rating */
  rating?: OrderRating;

  /** Reported issues */
  issues?: OrderIssue[];

  /** Quality score (0-100) */
  qualityScore?: number;

  // ========== Metadata ==========
  /** Additional custom data */
  metadata?: Record<string, any>;

  /** Order source (app, web, phone) */
  source: 'app' | 'web' | 'phone' | 'kiosk' | 'third_party';

  /** Platform (if aggregated) */
  platform?: 'ubereats' | 'doordash' | 'grubhub' | 'native';
}

/**
 * Order status change record
 * 주문 상태 변경 기록
 */
export interface StatusChange {
  /** Previous status */
  from: OrderStatus;

  /** New status */
  to: OrderStatus;

  /** Timestamp of change */
  timestamp: Date;

  /** User/system that made the change */
  changedBy: string;

  /** Reason for change (optional) */
  reason?: string;

  /** Additional notes */
  notes?: string;
}

/**
 * Order creation request
 * 주문 생성 요청
 */
export interface CreateOrderRequest {
  restaurantId: string;
  customerId: string;
  items: OrderItem[];
  pickupLocation: Address;
  deliveryLocation: Address;
  paymentMethod: PaymentMethod;
  contactlessDelivery?: boolean;
  specialInstructions?: string;
  scheduledTime?: Date;
  promoCode?: string;
  tip?: number;
}

/**
 * Order update request
 * 주문 업데이트 요청
 */
export interface UpdateOrderRequest {
  status?: OrderStatus;
  driverId?: string;
  estimatedDelivery?: Date;
  deliveryPhoto?: string;
  signature?: string;
  notes?: string;
}

// ============================================================================
// Driver Management Types
// ============================================================================

/**
 * Driver profile
 * 배달원 프로필
 */
export interface Driver {
  // ========== Identity ==========
  /** Driver identifier */
  id: string;

  /** External ID */
  externalId?: string;

  // ========== Personal Information ==========
  /** First name */
  firstName: string;

  /** Last name */
  lastName: string;

  /** Email address */
  email: string;

  /** Phone number (E.164 format) */
  phone: string;

  /** Profile photo URL */
  photo?: string;

  /** Date of birth */
  dateOfBirth?: Date;

  /** Languages spoken */
  languages?: string[];

  // ========== Vehicle Information ==========
  /** Vehicle type */
  vehicleType: VehicleType;

  /** License plate */
  licensePlate?: string;

  /** Vehicle make and model */
  vehicleModel?: string;

  /** Vehicle year */
  vehicleYear?: number;

  /** Vehicle color */
  vehicleColor?: string;

  // ========== Current Status ==========
  /** Driver status */
  status: DriverStatus;

  /** Is driver online */
  isOnline: boolean;

  /** Current location */
  location: GeoLocation;

  /** Last location update */
  lastLocationUpdate: Date;

  /** Current heading (direction) */
  heading?: number;

  // ========== Performance Metrics ==========
  /** Average rating (0-5) */
  rating: number;

  /** Total completed deliveries */
  totalDeliveries: number;

  /** Completion rate (0-1) */
  completionRate: number;

  /** On-time delivery rate (0-1) */
  onTimeRate: number;

  /** Average delivery time (minutes) */
  avgDeliveryTime: number;

  /** Orders per hour */
  ordersPerHour: number;

  /** Customer satisfaction score */
  satisfactionScore: number;

  // ========== Equipment ==========
  /** Equipment capabilities */
  equipment: {
    /** Has insulated hot bag */
    hasHotBag: boolean;

    /** Has insulated cold bag */
    hasColdBag: boolean;

    /** Has temperature sensor */
    hasTemperatureSensor: boolean;

    /** Has smart container */
    hasSmartContainer: boolean;

    /** Has thermal imaging */
    hasThermalImaging: boolean;

    /** Bag capacity (liters) */
    bagCapacity: number;
  };

  // ========== Capacity ==========
  /** Maximum concurrent orders */
  maxOrders: number;

  /** Currently active orders */
  currentOrders: string[];

  /** Maximum weight capacity (kg) */
  maxWeight: number;

  // ========== Service Areas ==========
  /** Assigned service areas */
  serviceAreas: string[];

  /** Home base location */
  homeBase?: GeoLocation;

  // ========== Financials ==========
  /** Earnings breakdown */
  earnings: {
    /** Today's earnings */
    today: number;

    /** This week's earnings */
    week: number;

    /** This month's earnings */
    month: number;

    /** All-time earnings */
    allTime: number;

    /** Average per delivery */
    avgPerDelivery: number;

    /** Average per hour */
    avgPerHour: number;
  };

  /** Payment information */
  payment: {
    /** Payment method (bank, wallet) */
    method: string;

    /** Bank account (last 4 digits) */
    accountLast4?: string;

    /** Payment schedule */
    schedule: 'daily' | 'weekly' | 'biweekly' | 'monthly';
  };

  // ========== Verification ==========
  /** Background check status */
  backgroundCheckStatus: 'pending' | 'approved' | 'rejected';

  /** Driver license verified */
  licenseVerified: boolean;

  /** Insurance verified */
  insuranceVerified: boolean;

  /** Identity verified */
  identityVerified: boolean;

  /** Food safety certification */
  foodSafetyCert?: {
    certified: boolean;
    expiresAt?: Date;
    certNumber?: string;
  };

  // ========== Availability ==========
  /** Working hours schedule */
  schedule?: DriverSchedule[];

  /** Currently available for new orders */
  availableForOrders: boolean;

  // ========== Metadata ==========
  /** Account created date */
  createdAt: Date;

  /** Last active timestamp */
  lastActiveAt: Date;

  /** Custom metadata */
  metadata?: Record<string, any>;
}

/**
 * Driver schedule/availability
 * 배달원 스케줄
 */
export interface DriverSchedule {
  /** Day of week (0=Sunday, 6=Saturday) */
  dayOfWeek: number;

  /** Start time (HH:MM) */
  startTime: string;

  /** End time (HH:MM) */
  endTime: string;

  /** Is this a regular schedule */
  recurring: boolean;

  /** Specific date (for one-time schedules) */
  date?: Date;
}

/**
 * Driver performance metrics
 * 배달원 성과 지표
 */
export interface DriverMetrics {
  driverId: string;
  period: 'today' | 'week' | 'month' | 'all_time';

  // ========== Efficiency Metrics ==========
  /** Orders completed per hour */
  ordersPerHour: number;

  /** Average delivery time (minutes) */
  avgDeliveryTime: number;

  /** Average distance per order (km) */
  avgDistancePerOrder: number;

  /** Active time / Online time */
  utilizationRate: number;

  /** Total distance traveled (km) */
  totalDistance: number;

  // ========== Quality Metrics ==========
  /** On-time delivery rate (0-100%) */
  onTimeRate: number;

  /** Customer rating (0-5) */
  customerRating: number;

  /** Order accuracy rate (0-100%) */
  orderAccuracy: number;

  /** Temperature compliance rate (0-100%) */
  temperatureCompliance: number;

  // ========== Reliability Metrics ==========
  /** Completion rate (0-100%) */
  completionRate: number;

  /** Cancellation rate (0-100%) */
  cancellationRate: number;

  /** Average response time (seconds) */
  responseTime: number;

  /** No-show rate (0-100%) */
  noShowRate: number;

  // ========== Financial Metrics ==========
  /** Total earnings */
  totalEarnings: number;

  /** Average earnings per hour */
  avgEarningsPerHour: number;

  /** Average earnings per delivery */
  avgEarningsPerDelivery: number;

  /** Total tips received */
  totalTips: number;

  /** Average tip amount */
  avgTip: number;

  // ========== Volume Metrics ==========
  /** Total orders completed */
  totalOrders: number;

  /** Total active hours */
  totalActiveHours: number;

  /** Total online hours */
  totalOnlineHours: number;
}

// ============================================================================
// Route Optimization Types
// ============================================================================

/**
 * Route stop (pickup or delivery point)
 * 경로 정류장
 */
export interface RouteStop {
  /** Stop sequence number */
  sequence: number;

  /** Stop type */
  type: 'pickup' | 'delivery';

  /** Associated order ID */
  orderId: string;

  /** Stop location */
  location: GeoLocation;

  /** Address */
  address: Address;

  /** Expected arrival time */
  arrivalTime: Date;

  /** Expected departure time */
  departureTime?: Date;

  /** Actual arrival time */
  actualArrival?: Date;

  /** Actual departure time */
  actualDeparture?: Date;

  /** Stop duration (minutes) */
  duration: number;

  /** Stop completed */
  completed: boolean;

  /** Distance from previous stop (km) */
  distanceFromPrevious: number;

  /** Time from previous stop (minutes) */
  timeFromPrevious: number;

  /** Special instructions */
  instructions?: string;
}

/**
 * Optimized delivery route
 * 최적화된 배달 경로
 */
export interface Route {
  /** Route identifier */
  id: string;

  /** Assigned driver */
  driverId: string;

  /** Orders in this route */
  orders: string[];

  // ========== Route Details ==========
  /** Ordered list of stops */
  stops: RouteStop[];

  /** Total route distance (km) */
  totalDistance: number;

  /** Total route duration (minutes) */
  totalDuration: number;

  /** Optimization algorithm used */
  optimizationAlgorithm: OptimizationAlgorithm;

  /** Optimization score (0-100) */
  optimizationScore: number;

  // ========== Waypoints and Path ==========
  /** Route waypoints */
  waypoints: GeoLocation[];

  /** Encoded polyline (Google format) */
  encodedPolyline?: string;

  /** Turn-by-turn directions */
  directions?: RouteDirection[];

  // ========== Performance ==========
  /** Estimated cost */
  estimatedCost: number;

  /** Estimated fuel consumption (liters) */
  fuelConsumption?: number;

  /** Estimated CO2 emissions (kg) */
  co2Emissions?: number;

  /** Efficiency score (orders/km) */
  efficiency: number;

  // ========== Status ==========
  /** Route status */
  status: 'planned' | 'active' | 'completed' | 'cancelled';

  /** Route started timestamp */
  startedAt?: Date;

  /** Route completed timestamp */
  completedAt?: Date;

  // ========== Deviations ==========
  /** Route deviations */
  deviations: RouteDeviation[];

  /** Total deviation distance (km) */
  totalDeviationDistance: number;

  /** Total delay (minutes) */
  totalDelay: number;
}

/**
 * Turn-by-turn direction
 * 턴바이턴 방향 안내
 */
export interface RouteDirection {
  /** Step number */
  step: number;

  /** Instruction text */
  instruction: string;

  /** Distance for this step (meters) */
  distance: number;

  /** Duration for this step (seconds) */
  duration: number;

  /** Maneuver type */
  maneuver: 'straight' | 'left' | 'right' | 'slight_left' | 'slight_right' | 'sharp_left' | 'sharp_right' | 'uturn';

  /** Start location */
  startLocation: GeoLocation;

  /** End location */
  endLocation: GeoLocation;
}

/**
 * Route deviation/detour
 * 경로 이탈
 */
export interface RouteDeviation {
  /** Deviation timestamp */
  timestamp: Date;

  /** Location where deviation occurred */
  location: GeoLocation;

  /** Reason for deviation */
  reason: string;

  /** Distance off route (meters) */
  distanceOff: number;

  /** Time impact (minutes) */
  timeImpact: number;

  /** Action taken */
  action: 'rerouted' | 'continued' | 'returned';
}

/**
 * Route optimization request
 * 경로 최적화 요청
 */
export interface OptimizeRouteRequest {
  /** Order IDs to optimize */
  orders: string[];

  /** Driver ID (optional, for assignment) */
  driverId?: string;

  /** Starting location */
  startLocation: GeoLocation;

  /** Optimization algorithm */
  algorithm?: OptimizationAlgorithm;

  /** Optimization objectives */
  objectives?: {
    /** Minimize distance */
    minimizeDistance?: number;    // Weight 0-1

    /** Minimize time */
    minimizeTime?: number;        // Weight 0-1

    /** Maximize orders */
    maximizeOrders?: number;      // Weight 0-1
  };

  /** Constraints */
  constraints?: {
    /** Maximum route distance (km) */
    maxDistance?: number;

    /** Maximum route time (minutes) */
    maxTime?: number;

    /** Maximum orders */
    maxOrders?: number;

    /** Temperature compatibility */
    temperatureCompatible?: boolean;
  };
}

// ============================================================================
// Temperature Monitoring Types
// ============================================================================

/**
 * Temperature reading from sensor
 * 온도 센서 측정값
 */
export interface TemperatureReading {
  /** Reading timestamp */
  timestamp: Date;

  /** Order ID */
  orderId: string;

  /** Sensor ID */
  sensorId: string;

  /** Temperature in Celsius */
  temperature: number;

  /** Humidity percentage (0-100) */
  humidity?: number;

  /** Sensor battery level (0-100) */
  batteryLevel: number;

  /** Location when reading taken */
  location: GeoLocation;

  /** Alert level */
  alertLevel?: 'normal' | 'warning' | 'critical' | 'severe';

  /** Is within safe range */
  withinSafeRange: boolean;
}

/**
 * Temperature alert
 * 온도 경고
 */
export interface TemperatureAlert {
  /** Alert ID */
  id: string;

  /** Order ID */
  orderId: string;

  /** Alert severity */
  severity: 'warning' | 'critical' | 'severe';

  /** Temperature reading that triggered alert */
  reading: TemperatureReading;

  /** Alert message */
  message: string;

  /** Timestamp */
  timestamp: Date;

  /** Actions taken */
  actions: string[];

  /** Resolved flag */
  resolved: boolean;

  /** Resolution notes */
  resolutionNotes?: string;
}

/**
 * Temperature compliance report
 * 온도 준수 보고서
 */
export interface TemperatureComplianceReport {
  orderId: string;

  /** Total monitoring duration (minutes) */
  totalDuration: number;

  /** Time in safe range (minutes) */
  timeInRange: number;

  /** Time out of range (minutes) */
  timeOutOfRange: number;

  /** Compliance percentage (0-100) */
  compliancePercentage: number;

  /** Minimum temperature recorded */
  minTemperature: number;

  /** Maximum temperature recorded */
  maxTemperature: number;

  /** Average temperature */
  avgTemperature: number;

  /** Number of alerts */
  alertCount: number;

  /** Passed compliance check */
  passed: boolean;

  /** Violation details */
  violations?: TemperatureViolation[];
}

/**
 * Temperature violation
 * 온도 위반
 */
export interface TemperatureViolation {
  /** Violation timestamp */
  timestamp: Date;

  /** Duration out of range (minutes) */
  duration: number;

  /** Temperature at time of violation */
  temperature: number;

  /** Required range */
  requiredRange: {
    min?: number;
    max?: number;
  };

  /** Severity */
  severity: 'minor' | 'major' | 'critical';
}

// ============================================================================
// Quality and Feedback Types
// ============================================================================

/**
 * Order rating and feedback
 * 주문 평가 및 피드백
 */
export interface OrderRating {
  /** Order ID */
  orderId: string;

  /** Customer ID */
  customerId: string;

  /** Rating timestamp */
  timestamp: Date;

  // ========== Ratings (1-5 stars) ==========
  /** Overall experience rating */
  overallRating: number;

  /** Food quality rating */
  foodQuality: number;

  /** Food temperature rating */
  foodTemperature: number;

  /** Packaging quality rating */
  packaging: number;

  /** Delivery speed rating */
  deliverySpeed: number;

  /** Driver professionalism rating */
  driverProfessionalism: number;

  /** Communication rating */
  communication: number;

  // ========== Feedback ==========
  /** Written comments */
  comments?: string;

  /** Would recommend (yes/no) */
  wouldRecommend: boolean;

  /** Issues encountered */
  issues?: string[];

  /** Evidence photos */
  photos?: string[];

  // ========== Tips ==========
  /** Tip given */
  tipAmount?: number;

  /** Tip percentage */
  tipPercentage?: number;
}

/**
 * Order issue/complaint
 * 주문 문제/불만
 */
export interface OrderIssue {
  /** Issue ID */
  id: string;

  /** Order ID */
  orderId: string;

  /** Issue type */
  type: 'missing_item' | 'wrong_item' | 'cold_food' | 'spilled' | 'late_delivery' | 'poor_quality' | 'other';

  /** Issue severity */
  severity: 'minor' | 'moderate' | 'severe';

  /** Description */
  description: string;

  /** Reported by */
  reportedBy: string;

  /** Reported at */
  reportedAt: Date;

  /** Evidence photos */
  photos?: string[];

  /** Resolution status */
  status: 'reported' | 'investigating' | 'resolved' | 'closed';

  /** Resolution details */
  resolution?: {
    /** Resolution type */
    type: 'refund' | 'credit' | 'redelivery' | 'none';

    /** Amount refunded */
    refundAmount?: number;

    /** Resolution notes */
    notes: string;

    /** Resolved at */
    resolvedAt: Date;
  };
}

// ============================================================================
// Analytics Types
// ============================================================================

/**
 * Platform analytics dashboard
 * 플랫폼 분석 대시보드
 */
export interface PlatformAnalytics {
  /** Analytics period */
  period: {
    start: Date;
    end: Date;
  };

  // ========== Order Metrics ==========
  orders: {
    /** Total orders */
    total: number;

    /** Completed orders */
    completed: number;

    /** Cancelled orders */
    cancelled: number;

    /** Average order value */
    avgValue: number;

    /** Total revenue */
    totalRevenue: number;
  };

  // ========== Driver Metrics ==========
  drivers: {
    /** Total active drivers */
    total: number;

    /** Average rating */
    avgRating: number;

    /** Average earnings per driver */
    avgEarnings: number;

    /** Utilization rate */
    utilizationRate: number;
  };

  // ========== Performance Metrics ==========
  performance: {
    /** Average delivery time (minutes) */
    avgDeliveryTime: number;

    /** On-time rate (%) */
    onTimeRate: number;

    /** Customer satisfaction (0-5) */
    customerSatisfaction: number;

    /** Temperature compliance (%) */
    temperatureCompliance: number;
  };

  // ========== Geographic Metrics ==========
  geography: {
    /** Top service areas */
    topAreas: Array<{ area: string; orders: number }>;

    /** Average distance per order */
    avgDistance: number;
  };
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * SDK configuration options
 * SDK 설정 옵션
 */
export interface FoodDeliveryConfig {
  /** API key for authentication */
  apiKey: string;

  /** API base URL */
  baseUrl?: string;

  /** Service region */
  region?: 'us' | 'eu' | 'asia' | 'global';

  /** Enable temperature monitoring */
  enableTemperatureMonitoring?: boolean;

  /** Enable route optimization */
  enableRouteOptimization?: boolean;

  /** Default optimization algorithm */
  defaultOptimizationAlgorithm?: OptimizationAlgorithm;

  /** Timeout in milliseconds */
  timeout?: number;

  /** Retry configuration */
  retry?: {
    maxRetries: number;
    backoff: 'linear' | 'exponential';
  };

  /** WebSocket configuration */
  websocket?: {
    enabled: boolean;
    reconnect: boolean;
    heartbeatInterval: number;
  };
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  OrderStatus,
  DriverStatus,
  VehicleType,
  TemperatureCategory,
  OptimizationAlgorithm,
  DeliveryPriority,
  PaymentMethod,
};
