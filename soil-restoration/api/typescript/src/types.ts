/**
 * WIA-ENV-SOIL-001 Soil Restoration Standard - Type Definitions
 * Philosophy: 弘益人間 (홍익인간) - Benefit All Humanity
 */

export interface SoilHealthMetrics {
  siteId: string;
  timestamp: string; // ISO 8601
  organicMatter: number; // percentage (0-100)
  pH: number; // pH value (0-14)
  moisture: number; // percentage (0-100)
  temperature: number; // °C
  bulkDensity: number; // g/cm³
  porosity: number; // percentage (0-100)
  waterHoldingCapacity: number; // mm/m
  infiltrationRate: number; // mm/hour
  aggregateStability: number; // percentage (0-100)
  carbonSequestration: number; // tonnes CO₂/hectare/year
  healthScore: number; // composite score (0-100)
  status: SoilStatus;
}

export enum SoilStatus {
  HEALTHY = "HEALTHY",
  DEGRADED = "DEGRADED",
  CONTAMINATED = "CONTAMINATED",
  RECOVERING = "RECOVERING",
  CRITICAL = "CRITICAL"
}

export interface ContaminationAssessment {
  assessmentId: string;
  siteId: string;
  timestamp: string;
  contaminants: Contaminant[];
  totalContaminationLevel: number; // 0-100 scale
  riskLevel: RiskLevel;
  remediationRequired: boolean;
  regulatoryCompliance: boolean;
}

export interface Contaminant {
  type: ContaminantType;
  concentration: number; // mg/kg
  regulatoryLimit: number; // mg/kg
  exceedsLimit: boolean;
  source?: string;
  mobility: 'HIGH' | 'MEDIUM' | 'LOW';
}

export enum ContaminantType {
  HEAVY_METALS = "HEAVY_METALS",
  PESTICIDES = "PESTICIDES",
  PETROLEUM = "PETROLEUM",
  INDUSTRIAL = "INDUSTRIAL",
  MICROPLASTICS = "MICROPLASTICS",
  PFAS = "PFAS",
  RADIOACTIVE = "RADIOACTIVE",
  OTHER = "OTHER"
}

export enum RiskLevel {
  LOW = "LOW",
  MODERATE = "MODERATE",
  HIGH = "HIGH",
  SEVERE = "SEVERE"
}

export interface RemediationTechnique {
  techniqueId: string;
  name: string;
  type: RemediationType;
  applicableContaminants: ContaminantType[];
  effectiveness: number; // percentage (0-100)
  duration: number; // months
  cost: number; // USD per hectare
  environmentalImpact: number; // score (0-100, lower is better)
  requiredConditions: {
    minpH?: number;
    maxpH?: number;
    minMoisture?: number;
    maxMoisture?: number;
    temperature?: { min: number; max: number };
  };
}

export enum RemediationType {
  BIOREMEDIATION = "BIOREMEDIATION",
  PHYTOREMEDIATION = "PHYTOREMEDIATION",
  CHEMICAL_TREATMENT = "CHEMICAL_TREATMENT",
  SOIL_WASHING = "SOIL_WASHING",
  THERMAL_TREATMENT = "THERMAL_TREATMENT",
  ELECTROKINETIC = "ELECTROKINETIC",
  SOIL_AMENDMENT = "SOIL_AMENDMENT",
  EXCAVATION = "EXCAVATION"
}

export interface SoilComposition {
  sand: number; // percentage
  silt: number; // percentage
  clay: number; // percentage
  textureClass: string; // e.g., "Sandy Loam", "Clay"
  cec: number; // cation exchange capacity (cmol/kg)
  basesSaturation: number; // percentage
}

export interface NutrientProfile {
  nitrogen: number; // mg/kg
  phosphorus: number; // mg/kg
  potassium: number; // mg/kg
  calcium: number; // mg/kg
  magnesium: number; // mg/kg
  sulfur: number; // mg/kg
  micronutrients: {
    iron: number;
    manganese: number;
    zinc: number;
    copper: number;
    boron: number;
    molybdenum: number;
  };
  cnRatio: number; // Carbon to Nitrogen ratio
  availableNutrients: {
    nitrogen: number;
    phosphorus: number;
    potassium: number;
  };
}

export interface MicrobiomeAnalysis {
  analysisId: string;
  siteId: string;
  timestamp: string;
  totalBiomass: number; // μg/g soil
  bacterialDiversity: number; // Shannon index
  fungalDiversity: number; // Shannon index
  beneficialMicrobes: number; // CFU/g
  pathogenicMicrobes: number; // CFU/g
  mycorrhizalColonization: number; // percentage
  enzymaticActivity: {
    dehydrogenase: number;
    urease: number;
    phosphatase: number;
    betaGlucosidase: number;
  };
  respirationRate: number; // mg CO₂/kg soil/day
  nitrificationRate: number; // mg N/kg soil/day
  microbialQuotient: number; // percentage
}

export interface LandUseHistory {
  siteId: string;
  historicalUses: Array<{
    period: { start: string; end: string };
    useType: string;
    intensityLevel: 'LOW' | 'MEDIUM' | 'HIGH';
    degradationImpact: number; // 0-100
    notes?: string;
  }>;
  currentUse: string;
  plannedUse: string;
}

export interface RestorationPlan {
  planId: string;
  siteId: string;
  createdDate: string;
  targetHealthScore: number;
  estimatedDuration: number; // months
  phases: RestorationPhase[];
  totalCost: number; // USD
  expectedOutcomes: {
    organicMatterIncrease: number; // percentage points
    contaminationReduction: number; // percentage
    biodiversityImprovement: number; // percentage
    carbonSequestration: number; // tonnes CO₂/hectare/year
  };
  status: 'DRAFT' | 'APPROVED' | 'IN_PROGRESS' | 'COMPLETED' | 'SUSPENDED';
}

export interface RestorationPhase {
  phaseNumber: number;
  name: string;
  duration: number; // months
  techniques: string[]; // RemediationTechnique IDs
  activities: Array<{
    name: string;
    schedule: string;
    resources: string[];
  }>;
  milestones: Array<{
    description: string;
    targetDate: string;
    completed: boolean;
  }>;
  budget: number; // USD
}

export interface MonitoringRecord {
  recordId: string;
  siteId: string;
  timestamp: string;
  measurements: SoilHealthMetrics;
  nutrientProfile: NutrientProfile;
  contaminationLevels?: ContaminationAssessment;
  vegetationCover: number; // percentage
  erosionRate: number; // tonnes/hectare/year
  complianceStatus: boolean;
  notes?: string;
}

export interface CertificationRecord {
  certificationId: string;
  siteId: string;
  certificationDate: string;
  certificationLevel: 'BRONZE' | 'SILVER' | 'GOLD' | 'PLATINUM';
  validUntil: string;
  certifyingBody: string;
  criteria: {
    soilHealth: number;
    contaminationFree: boolean;
    biodiversity: number;
    carbonSequestration: number;
    sustainablePractices: boolean;
  };
  status: 'ACTIVE' | 'EXPIRED' | 'SUSPENDED' | 'REVOKED';
}

export interface APIConfig {
  baseURL: string;
  apiKey: string;
  timeout?: number;
  retryAttempts?: number;
}

export interface SoilAnalysisReport {
  reportId: string;
  siteId: string;
  generatedDate: string;
  reportType: 'BASELINE' | 'PROGRESS' | 'COMPLETION' | 'ANNUAL';
  healthMetrics: SoilHealthMetrics;
  composition: SoilComposition;
  nutrients: NutrientProfile;
  microbiome?: MicrobiomeAnalysis;
  contamination?: ContaminationAssessment;
  recommendations: string[];
  complianceStatus: 'COMPLIANT' | 'NON_COMPLIANT' | 'UNDER_REVIEW';
}
