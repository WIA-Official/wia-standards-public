/**
 * WIA-IND-009: Food Delivery Standard - Type Definitions
 * @module wia-ind-009
 */

export enum OrderStatus {
  Pending = 'pending', Confirmed = 'confirmed', Preparing = 'preparing',
  ReadyForPickup = 'ready_for_pickup', PickedUp = 'picked_up',
  InTransit = 'in_transit', Delivered = 'delivered', Cancelled = 'cancelled'
}

export enum DeliveryMode {
  Standard = 'standard', Express = 'express', Scheduled = 'scheduled',
  Contactless = 'contactless', DroneDelivery = 'drone', RobotDelivery = 'robot'
}

export enum PaymentMethod {
  CreditCard = 'credit_card', DebitCard = 'debit_card', DigitalWallet = 'digital_wallet',
  CashOnDelivery = 'cash_on_delivery', Crypto = 'crypto'
}

export interface Location {
  latitude: number;
  longitude: number;
  address: string;
  city: string;
  postalCode: string;
  country: string;
  instructions?: string;
}

export interface Restaurant {
  id: string;
  name: string;
  cuisine: string[];
  rating: number;
  reviewCount: number;
  location: Location;
  openingHours: { day: string; open: string; close: string }[];
  deliveryRadius: number;
  minOrderAmount: number;
  averagePrepTime: number;
  isOpen: boolean;
}

export interface MenuItem {
  id: string;
  name: string;
  description: string;
  price: number;
  currency: string;
  category: string;
  dietary: string[];
  allergens: string[];
  calories?: number;
  image?: string;
  customizations?: Customization[];
  available: boolean;
}

export interface Customization {
  name: string;
  options: { name: string; price: number }[];
  required: boolean;
  maxSelections?: number;
}

export interface OrderItem {
  menuItem: MenuItem;
  quantity: number;
  customizations: { name: string; selection: string }[];
  specialInstructions?: string;
  subtotal: number;
}

export interface Order {
  id: string;
  restaurantId: string;
  customerId: string;
  items: OrderItem[];
  status: OrderStatus;
  deliveryMode: DeliveryMode;
  deliveryLocation: Location;
  paymentMethod: PaymentMethod;
  subtotal: number;
  deliveryFee: number;
  tax: number;
  tip: number;
  total: number;
  createdAt: number;
  estimatedDelivery: number;
  actualDelivery?: number;
  driverId?: string;
}

export interface Driver {
  id: string;
  name: string;
  phone: string;
  vehicleType: 'bicycle' | 'motorcycle' | 'car' | 'drone' | 'robot';
  currentLocation: Location;
  rating: number;
  activeOrders: string[];
  available: boolean;
}

export interface DeliveryTracking {
  orderId: string;
  driverId: string;
  currentLocation: Location;
  estimatedArrival: number;
  distanceRemaining: number;
  status: OrderStatus;
  route: Location[];
}

export interface Review {
  id: string;
  orderId: string;
  customerId: string;
  restaurantRating: number;
  deliveryRating: number;
  comment?: string;
  photos?: string[];
  createdAt: number;
}

export enum CertificationLevel { Bronze = 'bronze', Silver = 'silver', Gold = 'gold' }

export interface ComplianceReport {
  standard: 'WIA-IND-009';
  testDate: string;
  platformId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean }[];
  compliant: boolean;
}

export type FoodDeliveryEventType = 'order-update' | 'driver-assigned' | 'location-update' | 'delivered' | 'error';
export type EventCallback<T = unknown> = (data: T) => void;
