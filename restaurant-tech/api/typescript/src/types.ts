/**
 * WIA-IND-010: Restaurant Tech Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Restaurant Types
// ============================================================================

/**
 * Restaurant format types
 */
export type RestaurantFormat =
  | 'full_service'
  | 'quick_service'
  | 'fast_casual'
  | 'fine_dining'
  | 'cafe'
  | 'food_truck'
  | 'ghost_kitchen'
  | 'kiosk';

/**
 * Cuisine types
 */
export type CuisineType =
  | 'american'
  | 'italian'
  | 'chinese'
  | 'japanese'
  | 'mexican'
  | 'french'
  | 'indian'
  | 'thai'
  | 'mediterranean'
  | 'fusion'
  | 'other';

/**
 * Service style
 */
export type ServiceStyle =
  | 'table_service'
  | 'counter_service'
  | 'buffet'
  | 'family_style'
  | 'cafeteria';

// ============================================================================
// POS System Types
// ============================================================================

/**
 * Order types
 */
export enum OrderType {
  DINE_IN = 'dine_in',
  TAKEOUT = 'takeout',
  DELIVERY = 'delivery',
  CURBSIDE = 'curbside',
  DRIVE_THROUGH = 'drive_through',
  CATERING = 'catering',
  BAR = 'bar'
}

/**
 * Order status
 */
export enum OrderStatus {
  PENDING = 'pending',
  CONFIRMED = 'confirmed',
  IN_PROGRESS = 'in_progress',
  READY = 'ready',
  COMPLETED = 'completed',
  CANCELLED = 'cancelled',
  REFUNDED = 'refunded'
}

/**
 * Item status
 */
export enum ItemStatus {
  PENDING = 'pending',
  FIRED = 'fired',
  IN_PREP = 'in_prep',
  READY = 'ready',
  SERVED = 'served',
  CANCELLED = 'cancelled'
}

/**
 * Complete order structure
 */
export interface Order {
  /** Unique order identifier */
  orderId: string;

  /** Sequential daily order number */
  orderNumber: number;

  /** Type of order */
  orderType: OrderType;

  /** Current order status */
  status: OrderStatus;

  /** Table number (for dine-in) */
  tableNumber?: number;

  /** Server/employee ID */
  serverId: string;

  /** Server name */
  serverName: string;

  /** Order creation timestamp */
  createdAt: Date;

  /** Last modification timestamp */
  modifiedAt: Date;

  /** Order items */
  items: OrderItem[];

  /** Subtotal before tax and tip */
  subtotal: number;

  /** Applied discounts */
  discounts: Discount[];

  /** Tax amount */
  tax: number;

  /** Tip/gratuity amount */
  tip: number;

  /** Total amount */
  total: number;

  /** Payment status */
  paymentStatus: PaymentStatus;

  /** Special instructions */
  specialInstructions?: string;

  /** Course timing for multi-course meals */
  courseTiming?: CourseTiming[];

  /** Number of guests */
  guestCount?: number;

  /** Customer ID if known */
  customerId?: string;
}

/**
 * Order item details
 */
export interface OrderItem {
  /** Item identifier */
  itemId: string;

  /** Menu item reference */
  menuItemId: string;

  /** Item name */
  name: string;

  /** Menu category */
  category: string;

  /** Quantity ordered */
  quantity: number;

  /** Price per unit */
  unitPrice: number;

  /** Item modifiers */
  modifiers: Modifier[];

  /** Special requests */
  specialRequests?: string;

  /** Kitchen station assignment */
  station: KitchenStation;

  /** Course number (1, 2, 3, etc.) */
  courseNumber?: number;

  /** Seat position */
  seat?: number;

  /** Item status */
  status: ItemStatus;

  /** Estimated prep time in minutes */
  prepTime: number;

  /** When sent to kitchen */
  firedAt?: Date;

  /** When completed */
  readyAt?: Date;

  /** Item subtotal */
  subtotal: number;
}

/**
 * Item modifiers
 */
export interface Modifier {
  /** Modifier ID */
  modifierId: string;

  /** Modifier name */
  name: string;

  /** Type of modification */
  type: ModifierType;

  /** Price adjustment (can be negative) */
  priceAdjustment: number;

  /** Whether it affects food cost */
  affectsCost: boolean;
}

/**
 * Modifier types
 */
export enum ModifierType {
  ADD = 'add',
  REMOVE = 'remove',
  SUBSTITUTE = 'substitute',
  PORTION = 'portion',
  PREPARATION = 'preparation'
}

/**
 * Discount structure
 */
export interface Discount {
  /** Discount ID */
  discountId: string;

  /** Discount name */
  name: string;

  /** Type of discount */
  type: DiscountType;

  /** Discount value (percentage or fixed) */
  value: number;

  /** Items this applies to */
  applicableItems?: string[];

  /** Requires manager approval */
  requiresManagerApproval: boolean;

  /** Discount reason */
  reason?: string;

  /** Who authorized the discount */
  authorizedBy?: string;
}

/**
 * Discount types
 */
export enum DiscountType {
  PERCENTAGE = 'percentage',
  FIXED_AMOUNT = 'fixed_amount',
  HAPPY_HOUR = 'happy_hour',
  LOYALTY = 'loyalty',
  EMPLOYEE = 'employee',
  COMP = 'comp',
  PROMOTIONAL = 'promotional'
}

/**
 * Course timing for multi-course meals
 */
export interface CourseTiming {
  /** Course number */
  courseNumber: number;

  /** Items in this course */
  items: string[];

  /** When to fire this course */
  fireTime: Date;

  /** Target ready time */
  targetReadyTime: Date;

  /** Previous course */
  previousCourse?: number;

  /** Minimum gap between courses (minutes) */
  minimumGap: number;
}

// ============================================================================
// Table Management Types
// ============================================================================

/**
 * Table structure
 */
export interface Table {
  /** Table identifier */
  tableId: string;

  /** Table number */
  tableNumber: number;

  /** Section/area */
  section: string;

  /** Maximum capacity */
  capacity: number;

  /** Current party size */
  currentPartySize?: number;

  /** Table status */
  status: TableStatus;

  /** Assigned server ID */
  serverId?: string;

  /** Server name */
  serverName?: string;

  /** When party was seated */
  seatedAt?: Date;

  /** Estimated duration (minutes) */
  estimatedDuration: number;

  /** Table shape */
  shape: TableShape;

  /** Can combine with other tables */
  combinable: boolean;

  /** Position coordinates */
  position: Coordinates;
}

/**
 * Table status
 */
export enum TableStatus {
  AVAILABLE = 'available',
  OCCUPIED = 'occupied',
  RESERVED = 'reserved',
  DIRTY = 'dirty',
  CLEANING = 'cleaning',
  MAINTENANCE = 'maintenance'
}

/**
 * Table shapes
 */
export type TableShape = 'round' | 'square' | 'rectangular' | 'booth' | 'bar';

/**
 * 2D coordinates
 */
export interface Coordinates {
  x: number;
  y: number;
}

/**
 * Restaurant section
 */
export interface Section {
  /** Section ID */
  sectionId: string;

  /** Section name */
  name: string;

  /** Tables in section */
  tables: string[];

  /** Assigned server */
  serverId?: string;

  /** Total capacity */
  capacity: number;

  /** Section status */
  status: SectionStatus;

  /** Opening time */
  openTime?: Date;

  /** Closing time */
  closeTime?: Date;
}

/**
 * Section status
 */
export type SectionStatus = 'open' | 'closed' | 'limited';

// ============================================================================
// Reservation Types
// ============================================================================

/**
 * Reservation structure
 */
export interface Reservation {
  /** Reservation ID */
  reservationId: string;

  /** Confirmation code */
  confirmationCode: string;

  /** Customer ID (if registered) */
  customerId?: string;

  /** Customer name */
  customerName: string;

  /** Party size */
  partySize: number;

  /** Reservation date and time */
  dateTime: Date;

  /** Expected duration (minutes) */
  duration: number;

  /** Reservation status */
  status: ReservationStatus;

  /** Table preference */
  tablePreference?: string;

  /** Seating area preference */
  seatingArea?: string;

  /** Special occasion */
  occasion?: string;

  /** Special requests */
  specialRequests?: string;

  /** Dietary restrictions */
  dietaryRestrictions?: string[];

  /** Contact phone */
  contactPhone: string;

  /** Contact email */
  contactEmail: string;

  /** Creation timestamp */
  createdAt: Date;

  /** Created by (staff or 'online') */
  createdBy: string;

  /** Deposit amount */
  deposit?: number;

  /** No-show history count */
  noShowHistory?: number;

  /** VIP status */
  vipStatus?: boolean;

  /** Assigned table */
  assignedTable?: number;
}

/**
 * Reservation status
 */
export enum ReservationStatus {
  PENDING = 'pending',
  CONFIRMED = 'confirmed',
  SEATED = 'seated',
  COMPLETED = 'completed',
  CANCELLED = 'cancelled',
  NO_SHOW = 'no_show',
  WAITLIST = 'waitlist'
}

/**
 * Waitlist entry
 */
export interface WaitlistEntry {
  /** Entry ID */
  entryId: string;

  /** Customer name */
  customerName: string;

  /** Party size */
  partySize: number;

  /** Contact phone */
  contactPhone: string;

  /** Added timestamp */
  addedAt: Date;

  /** Quoted wait time (minutes) */
  quotedWait: number;

  /** Actual wait time (minutes) */
  actualWait?: number;

  /** Entry status */
  status: WaitlistStatus;

  /** Priority level */
  priority: number;

  /** Has been notified */
  notified: boolean;

  /** Notification timestamp */
  notifiedAt?: Date;

  /** Special requests */
  specialRequests?: string;
}

/**
 * Waitlist status
 */
export enum WaitlistStatus {
  WAITING = 'waiting',
  NOTIFIED = 'notified',
  SEATED = 'seated',
  CANCELLED = 'cancelled',
  NO_SHOW = 'no_show'
}

// ============================================================================
// Kitchen Display System Types
// ============================================================================

/**
 * Kitchen station types
 */
export type KitchenStation =
  | 'grill'
  | 'saute'
  | 'fry'
  | 'cold_prep'
  | 'salad'
  | 'dessert'
  | 'bar'
  | 'expo';

/**
 * Kitchen ticket
 */
export interface KitchenTicket {
  /** Ticket ID */
  ticketId: string;

  /** Order number */
  orderNumber: number;

  /** Table number */
  tableNumber?: number;

  /** Server name */
  serverName: string;

  /** Items to prepare */
  items: KitchenItem[];

  /** Priority level */
  priority: Priority;

  /** Ticket status */
  status: TicketStatus;

  /** When fired to kitchen */
  firedAt: Date;

  /** Due time (for courses) */
  dueTime?: Date;

  /** Elapsed time (seconds) */
  elapsedTime: number;

  /** Special instructions */
  specialInstructions?: string;

  /** Is this a remake */
  isRecall?: boolean;

  /** Recall reason */
  recallReason?: string;
}

/**
 * Kitchen item
 */
export interface KitchenItem {
  /** Item name */
  name: string;

  /** Quantity */
  quantity: number;

  /** Modifiers */
  modifiers: string[];

  /** Seat number */
  seat?: number;

  /** Item status */
  status: ItemStatus;

  /** Started prep timestamp */
  startedAt?: Date;

  /** Completed timestamp */
  completedAt?: Date;
}

/**
 * Priority levels
 */
export enum Priority {
  LOW = 'low',
  NORMAL = 'normal',
  HIGH = 'high',
  URGENT = 'urgent',
  VIP = 'vip'
}

/**
 * Ticket status
 */
export enum TicketStatus {
  PENDING = 'pending',
  IN_PROGRESS = 'in_progress',
  READY = 'ready',
  SERVED = 'served',
  CANCELLED = 'cancelled'
}

// ============================================================================
// Staff Management Types
// ============================================================================

/**
 * Staff roles
 */
export enum StaffRole {
  SERVER = 'server',
  BARTENDER = 'bartender',
  HOST = 'host',
  COOK = 'cook',
  PREP_COOK = 'prep_cook',
  DISHWASHER = 'dishwasher',
  BUSSER = 'busser',
  MANAGER = 'manager',
  SOMMELIER = 'sommelier',
  CHEF = 'chef'
}

/**
 * Skill levels
 */
export enum SkillLevel {
  TRAINEE = 'trainee',
  JUNIOR = 'junior',
  EXPERIENCED = 'experienced',
  SENIOR = 'senior',
  LEAD = 'lead'
}

/**
 * Employee information
 */
export interface Employee {
  /** Employee ID */
  employeeId: string;

  /** Full name */
  name: string;

  /** Email */
  email: string;

  /** Phone */
  phone: string;

  /** Primary role */
  role: StaffRole;

  /** Skill level */
  skillLevel: SkillLevel;

  /** Hourly rate */
  hourlyRate: number;

  /** Hire date */
  hireDate: Date;

  /** Available for scheduling */
  active: boolean;

  /** Certifications */
  certifications: string[];

  /** Work preferences */
  preferences: WorkPreferences;
}

/**
 * Work preferences
 */
export interface WorkPreferences {
  /** Preferred days */
  preferredDays: number[];

  /** Unavailable days */
  unavailableDays: number[];

  /** Maximum hours per week */
  maxHoursPerWeek: number;

  /** Minimum hours per week */
  minHoursPerWeek: number;

  /** Preferred shifts */
  preferredShifts: ShiftTime[];
}

/**
 * Shift time periods
 */
export type ShiftTime = 'breakfast' | 'lunch' | 'dinner' | 'late_night';

/**
 * Staff shift
 */
export interface StaffShift {
  /** Shift ID */
  shiftId: string;

  /** Employee ID */
  employeeId: string;

  /** Employee name */
  employeeName: string;

  /** Role for this shift */
  role: StaffRole;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Hourly rate */
  hourlyRate: number;

  /** Break times */
  breaks: Break[];

  /** Section assignment */
  section?: string;

  /** Shift status */
  status: ShiftStatus;
}

/**
 * Break period
 */
export interface Break {
  /** Break type */
  type: 'meal' | 'rest';

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Paid break */
  paid: boolean;
}

/**
 * Shift status
 */
export type ShiftStatus = 'scheduled' | 'in_progress' | 'completed' | 'no_show' | 'cancelled';

// ============================================================================
// Inventory Types
// ============================================================================

/**
 * Inventory categories
 */
export enum InventoryCategory {
  PRODUCE = 'produce',
  MEAT = 'meat',
  SEAFOOD = 'seafood',
  DAIRY = 'dairy',
  DRY_GOODS = 'dry_goods',
  BEVERAGES = 'beverages',
  ALCOHOL = 'alcohol',
  SUPPLIES = 'supplies',
  CLEANING = 'cleaning'
}

/**
 * Measurement units
 */
export enum Unit {
  EACH = 'each',
  POUND = 'lb',
  KILOGRAM = 'kg',
  OUNCE = 'oz',
  GRAM = 'g',
  LITER = 'l',
  GALLON = 'gal',
  CASE = 'case',
  BOX = 'box',
  CUP = 'cup',
  TABLESPOON = 'tbsp',
  TEASPOON = 'tsp'
}

/**
 * Inventory item
 */
export interface InventoryItem {
  /** Item ID */
  itemId: string;

  /** Item name */
  name: string;

  /** Category */
  category: InventoryCategory;

  /** Measurement unit */
  unit: Unit;

  /** Current quantity */
  currentQuantity: number;

  /** Par level (minimum) */
  parLevel: number;

  /** Maximum level */
  maxLevel: number;

  /** Reorder point */
  reorderPoint: number;

  /** Reorder quantity */
  reorderQuantity: number;

  /** Unit cost */
  unitCost: number;

  /** Supplier */
  supplier: string;

  /** Supplier ID */
  supplierId: string;

  /** Shelf life (days) */
  shelfLife?: number;

  /** Storage location */
  storageLocation: string;

  /** Track by lot number */
  trackByLot: boolean;

  /** Track by serial */
  trackBySerial: boolean;

  /** Last updated */
  lastUpdated: Date;
}

/**
 * Stock movement types
 */
export enum MovementType {
  RECEIVED = 'received',
  USED = 'used',
  WASTED = 'wasted',
  SOLD = 'sold',
  TRANSFERRED = 'transferred',
  ADJUSTMENT = 'adjustment'
}

/**
 * Stock movement record
 */
export interface StockMovement {
  /** Movement ID */
  movementId: string;

  /** Item ID */
  itemId: string;

  /** Movement type */
  type: MovementType;

  /** Quantity */
  quantity: number;

  /** Unit */
  unit: Unit;

  /** Timestamp */
  timestamp: Date;

  /** User who recorded */
  userId: string;

  /** Reference (order ID, etc.) */
  reference?: string;

  /** Cost */
  cost?: number;

  /** Notes */
  notes?: string;
}

// ============================================================================
// Menu Types
// ============================================================================

/**
 * Menu item
 */
export interface MenuItem {
  /** Item ID */
  menuItemId: string;

  /** Item name */
  name: string;

  /** Description */
  description: string;

  /** Category */
  category: string;

  /** Price */
  price: number;

  /** Cost to make */
  cost: number;

  /** Recipe ID */
  recipeId: string;

  /** Allergens */
  allergens: string[];

  /** Dietary tags */
  dietaryTags: string[];

  /** Available */
  available: boolean;

  /** Prep time (minutes) */
  prepTime: number;

  /** Image URL */
  imageUrl?: string;

  /** Modifiers available */
  availableModifiers: string[];

  /** Popularity score */
  popularityScore?: number;
}

/**
 * Recipe
 */
export interface Recipe {
  /** Recipe ID */
  recipeId: string;

  /** Menu item ID */
  menuItemId: string;

  /** Recipe name */
  name: string;

  /** Yield (servings) */
  yield: number;

  /** Ingredients */
  ingredients: Ingredient[];

  /** Total cost */
  cost: number;

  /** Instructions */
  instructions?: string[];

  /** Prep time (minutes) */
  prepTime: number;

  /** Cook time (minutes) */
  cookTime: number;

  /** Allergens */
  allergens: string[];

  /** Version */
  version: number;
}

/**
 * Recipe ingredient
 */
export interface Ingredient {
  /** Inventory item ID */
  itemId: string;

  /** Ingredient name */
  name: string;

  /** Quantity needed */
  quantity: number;

  /** Unit */
  unit: Unit;

  /** Cost */
  cost: number;

  /** Optional ingredient */
  optional: boolean;

  /** Substitute for */
  substituteFor?: string;
}

// ============================================================================
// Payment Types
// ============================================================================

/**
 * Payment methods
 */
export enum PaymentMethod {
  CASH = 'cash',
  CREDIT_CARD = 'credit_card',
  DEBIT_CARD = 'debit_card',
  GIFT_CARD = 'gift_card',
  MOBILE_PAYMENT = 'mobile_payment',
  CRYPTOCURRENCY = 'cryptocurrency',
  ACCOUNT_CHARGE = 'account_charge',
  COMP = 'comp'
}

/**
 * Payment status
 */
export enum PaymentStatus {
  PENDING = 'pending',
  AUTHORIZED = 'authorized',
  CAPTURED = 'captured',
  REFUNDED = 'refunded',
  FAILED = 'failed',
  CANCELLED = 'cancelled'
}

/**
 * Payment transaction
 */
export interface Payment {
  /** Payment ID */
  paymentId: string;

  /** Order ID */
  orderId: string;

  /** Payment method */
  method: PaymentMethod;

  /** Amount paid */
  amount: number;

  /** Tip amount */
  tipAmount: number;

  /** Total amount */
  totalAmount: number;

  /** Payment status */
  status: PaymentStatus;

  /** Processing timestamp */
  processedAt: Date;

  /** Processor reference */
  processorReference?: string;

  /** Last 4 digits of card */
  last4?: string;

  /** Card brand */
  cardBrand?: string;

  /** Authorization code */
  authCode?: string;
}

// ============================================================================
// Customer Types
// ============================================================================

/**
 * Customer profile
 */
export interface Customer {
  /** Customer ID */
  customerId: string;

  /** First name */
  firstName: string;

  /** Last name */
  lastName: string;

  /** Email */
  email: string;

  /** Phone */
  phone: string;

  /** Date of birth */
  dateOfBirth?: Date;

  /** Join date */
  joinDate: Date;

  /** Total visits */
  totalVisits: number;

  /** Total spent */
  totalSpent: number;

  /** Average check size */
  averageCheckSize: number;

  /** Last visit */
  lastVisit: Date;

  /** Favorite items */
  favoriteItems: string[];

  /** Dietary restrictions */
  dietaryRestrictions: string[];

  /** Allergens */
  allergens: string[];

  /** VIP status */
  vipStatus: boolean;

  /** Loyalty points */
  loyaltyPoints: number;

  /** Customer segment */
  segment: CustomerSegment;

  /** Communication preferences */
  communicationPreferences: CommunicationPreferences;
}

/**
 * Customer segments
 */
export enum CustomerSegment {
  NEW = 'new',
  OCCASIONAL = 'occasional',
  REGULAR = 'regular',
  LOYAL = 'loyal',
  VIP = 'vip',
  AT_RISK = 'at_risk',
  LOST = 'lost'
}

/**
 * Communication preferences
 */
export interface CommunicationPreferences {
  /** Email opt-in */
  email: boolean;

  /** SMS opt-in */
  sms: boolean;

  /** Phone opt-in */
  phone: boolean;

  /** Push notifications */
  push: boolean;
}

// ============================================================================
// Analytics Types
// ============================================================================

/**
 * Sales analytics
 */
export interface SalesAnalytics {
  /** Date */
  date: Date;

  /** Total revenue */
  totalRevenue: number;

  /** Food revenue */
  foodRevenue: number;

  /** Beverage revenue */
  beverageRevenue: number;

  /** Total covers */
  totalCovers: number;

  /** Average check */
  averageCheck: number;

  /** Table turnover */
  tableTurnover: number;

  /** RevPASH */
  revpash: number;
}

/**
 * Performance metrics
 */
export interface PerformanceMetrics {
  /** Labor cost percentage */
  laborCostPercent: number;

  /** Food cost percentage */
  foodCostPercent: number;

  /** Prime cost percentage */
  primeCostPercent: number;

  /** Average ticket time */
  averageTicketTime: number;

  /** Order accuracy rate */
  orderAccuracy: number;

  /** Customer satisfaction */
  customerSatisfaction: number;
}

// ============================================================================
// Calculation Functions
// ============================================================================

/**
 * Calculate order total
 */
export interface OrderTotalParams {
  subtotal: number;
  taxRate: number;
  tipPercent?: number;
  discounts?: Discount[];
}

export interface OrderTotal {
  subtotal: number;
  discountAmount: number;
  taxableAmount: number;
  tax: number;
  tip: number;
  total: number;
}

/**
 * Calculate table turnover
 */
export interface TurnoverParams {
  partiesServed: number;
  numberOfTables: number;
  hoursOpen: number;
}

/**
 * Calculate RevPASH
 */
export interface RevPASHParams {
  totalRevenue: number;
  numberOfSeats: number;
  hoursOpen: number;
}

/**
 * Calculate food cost percentage
 */
export interface FoodCostParams {
  costOfGoodsSold: number;
  foodSalesRevenue: number;
}

/**
 * Calculate labor cost percentage
 */
export interface LaborCostParams {
  totalLaborCost: number;
  totalRevenue: number;
}

// ============================================================================
// Location Types
// ============================================================================

/**
 * Restaurant location
 */
export interface Location {
  /** Location ID */
  locationId: string;

  /** Location name */
  name: string;

  /** Address */
  address: Address;

  /** Phone */
  phone: string;

  /** Timezone */
  timezone: string;

  /** Manager ID */
  managerId: string;

  /** Operating hours */
  operatingHours: OperatingHours[];

  /** Seating capacity */
  seatingCapacity: number;

  /** Location status */
  status: LocationStatus;

  /** Opening date */
  openDate: Date;

  /** Restaurant format */
  format: RestaurantFormat;
}

/**
 * Address
 */
export interface Address {
  street: string;
  city: string;
  state: string;
  postalCode: string;
  country: string;
}

/**
 * Operating hours
 */
export interface OperatingHours {
  dayOfWeek: number;
  openTime: string;
  closeTime: string;
  closed: boolean;
}

/**
 * Location status
 */
export type LocationStatus = 'open' | 'closed' | 'under_renovation' | 'coming_soon';

// ============================================================================
// Export all types
// ============================================================================

export * from './index';
