/**
 * WIA Carbon Credit Micro Standard - Type Definitions
 * @packageDocumentation
 * @module wia-carbon-credit-micro
 */

export enum CreditType {
  Avoidance = 'avoidance',
  Removal = 'removal',
  Reduction = 'reduction',
  Sequestration = 'sequestration'
}

export enum ProjectType {
  Reforestation = 'reforestation',
  Renewable = 'renewable',
  EnergyEfficiency = 'energy_efficiency',
  CleanCooking = 'clean_cooking',
  WasteManagement = 'waste_management',
  Agriculture = 'agriculture',
  BlueCarbon = 'blue_carbon',
  DirectAirCapture = 'direct_air_capture'
}

export enum VerificationStandard {
  VCS = 'vcs',
  GoldStandard = 'gold_standard',
  CDM = 'cdm',
  ACR = 'acr',
  CAR = 'car',
  Puro = 'puro',
  Internal = 'internal'
}

export enum ActivityCategory {
  Transport = 'transport',
  Energy = 'energy',
  Food = 'food',
  Shopping = 'shopping',
  Home = 'home',
  Travel = 'travel',
  Digital = 'digital',
  Other = 'other'
}

export interface MicroCredit {
  id: string;
  projectId: string;
  amount: number;
  unit: 'grams' | 'kg' | 'tonnes';
  type: CreditType;
  vintage: number;
  serialNumber: string;
  issuedAt: Date;
  retiredAt?: Date;
  retiredBy?: string;
  price: number;
  currency: string;
}

export interface CarbonProject {
  id: string;
  name: string;
  description: string;
  type: ProjectType;
  location: ProjectLocation;
  standard: VerificationStandard;
  startDate: Date;
  endDate?: Date;
  totalCredits: number;
  availableCredits: number;
  pricePerTonne: number;
  currency: string;
  verified: boolean;
  verifier?: string;
  documents: string[];
}

export interface ProjectLocation {
  country: string;
  region?: string;
  latitude: number;
  longitude: number;
}

export interface CarbonFootprint {
  userId: string;
  period: FootprintPeriod;
  totalEmissions: number;
  breakdown: EmissionBreakdown[];
  offset: number;
  netEmissions: number;
  calculatedAt: Date;
}

export interface FootprintPeriod {
  start: Date;
  end: Date;
  type: 'day' | 'week' | 'month' | 'year';
}

export interface EmissionBreakdown {
  category: ActivityCategory;
  emissions: number;
  percentage: number;
  activities: Activity[];
}

export interface Activity {
  id: string;
  category: ActivityCategory;
  name: string;
  emissions: number;
  unit: string;
  quantity: number;
  emissionFactor: number;
  timestamp: Date;
  source?: string;
}

export interface OffsetTransaction {
  id: string;
  userId: string;
  credits: MicroCredit[];
  totalAmount: number;
  totalCost: number;
  currency: string;
  status: 'pending' | 'completed' | 'cancelled' | 'refunded';
  purchasedAt: Date;
  retiredAt?: Date;
  certificate?: Certificate;
}

export interface Certificate {
  id: string;
  transactionId: string;
  userId: string;
  amount: number;
  projectName: string;
  issuedAt: Date;
  certificateUrl: string;
  qrCode: string;
}

export interface Wallet {
  userId: string;
  balance: number;
  currency: string;
  creditsOwned: MicroCredit[];
  totalOffset: number;
  transactions: OffsetTransaction[];
}

export interface EmissionFactor {
  id: string;
  category: ActivityCategory;
  activity: string;
  factor: number;
  unit: string;
  source: string;
  validFrom: Date;
  validTo?: Date;
  region?: string;
}

export interface Challenge {
  id: string;
  name: string;
  description: string;
  targetReduction: number;
  duration: number;
  participants: number;
  startDate: Date;
  endDate: Date;
  rewards: Reward[];
}

export interface Reward {
  type: 'credits' | 'badge' | 'discount';
  value: number;
  description: string;
}

export interface Leaderboard {
  period: FootprintPeriod;
  entries: LeaderboardEntry[];
}

export interface LeaderboardEntry {
  rank: number;
  userId: string;
  displayName: string;
  reduction: number;
  offset: number;
  score: number;
}

export interface CarbonCreditMicroConfig {
  apiEndpoint: string;
  apiKey?: string;
  defaultCurrency: string;
  minCreditAmount: number;
  enableGamification: boolean;
  autoOffset: boolean;
}

export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

export interface ComplianceReport {
  standard: 'WIA-CARBON-CREDIT-MICRO';
  testDate: string;
  config: CarbonCreditMicroConfig;
  targetLevel: CertificationLevel;
  tests: TestResult[];
  passed: boolean;
  achievedLevel?: CertificationLevel;
}

export interface TestResult {
  testName: string;
  passed: boolean;
  notes?: string;
}
