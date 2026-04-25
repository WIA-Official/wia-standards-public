/**
 * WIA Service Robot Standard - Type Definitions
 * @module wia-service-robot
 */

export enum ServiceType {
  Concierge = 'concierge', Delivery = 'delivery', Cleaning = 'cleaning',
  Waiter = 'waiter', Receptionist = 'receptionist', Guide = 'guide',
  Assistant = 'assistant', Companion = 'companion'
}

export enum VenueType {
  Hotel = 'hotel', Restaurant = 'restaurant', Hospital = 'hospital',
  Airport = 'airport', Mall = 'mall', Office = 'office',
  RetailStore = 'retail_store', Museum = 'museum', ResidentialCare = 'residential_care'
}

export enum InteractionMode {
  Voice = 'voice', Touch = 'touch', Gesture = 'gesture', QRCode = 'qr_code', App = 'app'
}

export interface ServiceRobotSpec {
  standard: 'WIA-SERVICE-ROBOT';
  version: string;
  robotId: string;
  name: string;
  serviceType: ServiceType;
  venue: VenueType;
  languages: string[];
  interactionModes: InteractionMode[];
  displaySize?: number;
  payloadCapacity?: number;
  batteryLife: number;
  chargingTime: number;
}

export interface ServiceRequest {
  id: string;
  type: string;
  customerId?: string;
  location: { x: number; y: number; floor?: number };
  priority: 'low' | 'normal' | 'high' | 'urgent';
  details: Record<string, unknown>;
  status: 'pending' | 'assigned' | 'in_progress' | 'completed' | 'cancelled';
  createdAt: number;
  completedAt?: number;
  rating?: number;
  feedback?: string;
}

export interface DeliveryTask {
  id: string;
  items: { name: string; quantity: number; special?: string }[];
  origin: { location: string; floor?: number };
  destination: { location: string; floor?: number };
  recipient?: string;
  pickupTime?: number;
  deliveryTime?: number;
  status: 'pending' | 'picked_up' | 'in_transit' | 'delivered' | 'failed';
  attempts: number;
}

export interface CustomerInteraction {
  id: string;
  sessionId: string;
  timestamp: number;
  type: 'greeting' | 'inquiry' | 'request' | 'feedback' | 'farewell';
  input: { mode: InteractionMode; content: string };
  response: { text: string; actions?: string[] };
  language: string;
  sentiment?: 'positive' | 'neutral' | 'negative';
  duration: number;
}

export interface FAQ {
  id: string;
  question: string;
  answer: string;
  category: string;
  language: string;
  keywords: string[];
  usageCount: number;
}

export interface Reservation {
  id: string;
  type: 'table' | 'room' | 'service' | 'appointment';
  customerName: string;
  customerContact?: string;
  dateTime: string;
  duration?: number;
  partySize?: number;
  specialRequests?: string[];
  status: 'confirmed' | 'pending' | 'cancelled' | 'completed';
}

export interface GuidedTour {
  id: string;
  name: string;
  duration: number;
  stops: TourStop[];
  languages: string[];
  accessibility: boolean;
  currentParticipants?: number;
  maxParticipants: number;
}

export interface TourStop {
  id: string;
  name: string;
  location: { x: number; y: number };
  duration: number;
  narration: { language: string; text: string; audioUrl?: string }[];
  interactiveElements?: string[];
}

export interface ServiceMetrics {
  robotId: string;
  date: string;
  totalInteractions: number;
  completedRequests: number;
  averageResponseTime: number;
  customerSatisfaction: number;
  distanceTraveled: number;
  uptime: number;
}

export interface CustomerProfile {
  id: string;
  name?: string;
  preferences: { language: string; interactionMode: InteractionMode };
  visitHistory: { date: string; services: string[] }[];
  loyaltyPoints?: number;
  notes?: string[];
}

export enum CertificationLevel { Bronze = 'bronze', Silver = 'silver', Gold = 'gold' }

export interface ComplianceReport {
  standard: 'WIA-SERVICE-ROBOT';
  testDate: string;
  robotId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean }[];
  compliant: boolean;
}

export type ServiceEventType = 'request-received' | 'task-completed' | 'interaction-ended' | 'low-battery' | 'assistance-needed';
export type EventCallback<T = unknown> = (data: T) => void;
