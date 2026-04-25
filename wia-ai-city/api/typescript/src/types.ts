/**
 * WIA-AI-CITY TypeScript Types
 * AI-powered Urban Management and Smart City Intelligence
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

// ============================================================================
// AI Model Types
// ============================================================================

export type AIModelType =
  | 'traffic_prediction'
  | 'energy_optimization'
  | 'safety_detection'
  | 'resource_allocation'
  | 'demand_forecasting'
  | 'anomaly_detection'
  | 'event_prediction'
  | 'climate_adaptation';

export interface AIModel {
  id: string;
  name: string;
  type: AIModelType;
  version: string;
  accuracy: number;
  trainedAt: Date;
  lastUpdated: Date;
  parameters: Record<string, any>;
  status: 'active' | 'training' | 'inactive' | 'deprecated';
}

export interface ModelPrediction {
  modelId: string;
  timestamp: Date;
  confidence: number;
  result: any;
  metadata: Record<string, any>;
}

// ============================================================================
// City Prediction Types
// ============================================================================

export interface TrafficPrediction {
  location: string;
  coordinates: { lat: number; lon: number };
  timestamp: Date;
  predictedVolume: number;
  congestionLevel: 'low' | 'medium' | 'high' | 'critical';
  averageSpeed: number;
  incidents: number;
  confidence: number;
  recommendations: string[];
}

export interface EnergyDemandPrediction {
  zone: string;
  timestamp: Date;
  predictedDemand: number; // MW
  peakTime: Date;
  loadFactor: number;
  renewableRatio: number;
  confidence: number;
  recommendations: string[];
}

export interface EventPrediction {
  type: 'traffic' | 'energy' | 'safety' | 'weather' | 'public';
  severity: 'low' | 'medium' | 'high' | 'critical';
  probability: number;
  expectedTime: Date;
  location: string;
  impact: string;
  mitigation: string[];
}

// ============================================================================
// Traffic AI Types
// ============================================================================

export interface TrafficFlowData {
  sensorId: string;
  location: string;
  timestamp: Date;
  vehicleCount: number;
  averageSpeed: number;
  occupancy: number;
  vehicleTypes: Record<string, number>;
}

export interface TrafficOptimization {
  intersectionId: string;
  currentPhase: string;
  optimizedPhase: string;
  expectedImprovement: number;
  affectedRoutes: string[];
  implementAt: Date;
}

export interface RouteOptimization {
  origin: { lat: number; lon: number };
  destination: { lat: number; lon: number };
  optimalRoute: Array<{ lat: number; lon: number }>;
  estimatedTime: number;
  distance: number;
  trafficConditions: string;
  alternativeRoutes: number;
}

// ============================================================================
// Energy AI Types
// ============================================================================

export interface EnergyOptimization {
  zoneId: string;
  timestamp: Date;
  currentLoad: number;
  optimizedLoad: number;
  savingsPotential: number;
  renewableIntegration: number;
  batteryStorage: number;
  recommendations: string[];
}

export interface GridBalancing {
  gridId: string;
  timestamp: Date;
  supply: number;
  demand: number;
  imbalance: number;
  frequency: number;
  voltage: number;
  actions: Array<{
    type: 'increase' | 'decrease' | 'store' | 'release';
    source: string;
    amount: number;
  }>;
}

export interface RenewableForecasting {
  source: 'solar' | 'wind' | 'hydro' | 'geothermal';
  location: string;
  timestamp: Date;
  predictedOutput: number;
  confidence: number;
  weatherConditions: Record<string, any>;
}

// ============================================================================
// Safety AI Types
// ============================================================================

export interface SafetyIncident {
  id: string;
  type: 'crime' | 'fire' | 'medical' | 'infrastructure' | 'environmental';
  severity: 'low' | 'medium' | 'high' | 'critical';
  location: { lat: number; lon: number };
  timestamp: Date;
  description: string;
  status: 'detected' | 'responding' | 'resolved' | 'escalated';
  aiConfidence: number;
}

export interface SafetyAnalysis {
  zoneId: string;
  timestamp: Date;
  riskLevel: number;
  threats: SafetyIncident[];
  vulnerabilities: string[];
  recommendations: string[];
  emergencyReadiness: number;
}

export interface AnomalyDetection {
  sensorId: string;
  timestamp: Date;
  anomalyType: string;
  severity: number;
  normalPattern: any;
  detectedPattern: any;
  confidence: number;
  suggestedActions: string[];
}

// ============================================================================
// Resource Allocation AI Types
// ============================================================================

export interface ResourceAllocation {
  resourceType: 'vehicle' | 'personnel' | 'equipment' | 'energy' | 'water';
  currentDistribution: Record<string, number>;
  optimizedDistribution: Record<string, number>;
  efficiency: number;
  costSavings: number;
  timestamp: Date;
}

export interface ServiceOptimization {
  service: 'waste' | 'transit' | 'maintenance' | 'emergency' | 'utilities';
  currentRoutes: string[];
  optimizedRoutes: string[];
  improvement: number;
  resourceSavings: number;
  environmentalImpact: number;
}

export interface DemandForecasting {
  service: string;
  zone: string;
  timestamp: Date;
  predictedDemand: number;
  currentCapacity: number;
  gap: number;
  recommendations: string[];
}

// ============================================================================
// Decision Support Types
// ============================================================================

export interface DecisionRecommendation {
  id: string;
  category: string;
  priority: 'low' | 'medium' | 'high' | 'critical';
  title: string;
  description: string;
  analysis: string;
  alternatives: Array<{
    option: string;
    pros: string[];
    cons: string[];
    cost: number;
    impact: number;
  }>;
  aiConfidence: number;
  dataSources: string[];
}

export interface CityMetrics {
  timestamp: Date;
  trafficEfficiency: number;
  energyEfficiency: number;
  safetyScore: number;
  resourceUtilization: number;
  citizenSatisfaction: number;
  environmentalHealth: number;
  economicVitality: number;
}

export interface SimulationScenario {
  id: string;
  name: string;
  description: string;
  parameters: Record<string, any>;
  predictions: Array<{
    metric: string;
    currentValue: number;
    predictedValue: number;
    confidence: number;
  }>;
  risks: string[];
  opportunities: string[];
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface AICityConfig {
  cityId: string;
  cityName: string;
  population: number;
  area: number;
  timezone: string;
  models: AIModel[];
  sensors: number;
  updateInterval: number;
  dataRetention: number;
  apiEndpoint?: string;
}

export interface EventHandler {
  onPrediction?: (prediction: ModelPrediction) => void;
  onAnomaly?: (anomaly: AnomalyDetection) => void;
  onIncident?: (incident: SafetyIncident) => void;
  onOptimization?: (optimization: any) => void;
  onAlert?: (alert: { level: string; message: string }) => void;
}
