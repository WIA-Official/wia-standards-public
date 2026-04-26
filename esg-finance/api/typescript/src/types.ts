/**
 * WIA-FIN-024 ESG Finance Standard
 * TypeScript Type Definitions
 *
 * @version 2.0.0
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export type CompanyId = string;
export type BondId = string;
export type ReportId = string;
export type Currency = string; // ISO 4217

// ============================================================================
// ESG Scoring Types
// ============================================================================

export interface ESGScore {
  companyId: CompanyId;
  calculatedAt: Date;
  period: string; // e.g., "2024-FY", "2024-Q4"
  environmental: EnvironmentalScore;
  social: SocialScore;
  governance: GovernanceScore;
  overallScore: number; // 0-100
  rating: ESGRating;
}

export enum ESGRating {
  AAA = 'AAA',
  AA = 'AA',
  A = 'A',
  BBB = 'BBB',
  BB = 'BB',
  B = 'B'
}

// ============================================================================
// Environmental Pillar
// ============================================================================

export interface EnvironmentalScore {
  score: number; // 0-100
  components: {
    climateEnergy: ClimateEnergyMetrics;
    natureBiodiversity: NatureBiodiversityMetrics;
    resourceEfficiency: ResourceEfficiencyMetrics;
    pollutionPrevention: PollutionPreventionMetrics;
  };
}

export interface ClimateEnergyMetrics {
  score: number;
  ghgEmissions: {
    scope1: number; // tCO2e
    scope2: number; // tCO2e
    scope3: number; // tCO2e
    total: number; // tCO2e
    intensity: number; // tCO2e per $M revenue
  };
  renewableEnergy: {
    percentage: number; // 0-100
    absoluteMWh: number;
  };
  netZeroTarget?: {
    targetYear: number;
    validated: boolean; // SBTi validated
    baselineYear: number;
    reductionTarget: number; // percentage
  };
}

export interface NatureBiodiversityMetrics {
  score: number;
  biodiversityImpact: {
    landUseHectares: number;
    sensitiveAreasCount: number;
    threatenedSpeciesAffected: number;
  };
  restorationInvestment: number; // USD
  natureBasedSolutions: boolean;
}

export interface ResourceEfficiencyMetrics {
  score: number;
  water: {
    withdrawalM3: number;
    consumptionM3: number;
    recyclingRate: number; // percentage
    stressedLocations: string[]; // location identifiers
  };
  circularEconomy: {
    materialCircularityRate: number; // percentage
    recycledMaterialsRate: number; // percentage
    productTakebackRate: number; // percentage
  };
}

export interface PollutionPreventionMetrics {
  score: number;
  airPollution: {
    noxEmissions: number; // tons
    soxEmissions: number; // tons
    particulateMatter: number; // tons
  };
  waterPollution: {
    dischargeMethCubic: number;
    wasteWaterTreated: number; // percentage
  };
  wasteManagement: {
    totalWasteTons: number;
    recycledPercentage: number;
    landfillPercentage: number;
  };
}

// ============================================================================
// Social Pillar
// ============================================================================

export interface SocialScore {
  score: number; // 0-100
  components: {
    humanCapital: HumanCapitalMetrics;
    laborPractices: LaborPracticesMetrics;
    supplyChain: SupplyChainMetrics;
    stakeholderEngagement: StakeholderEngagementMetrics;
  };
}

export interface HumanCapitalMetrics {
  score: number;
  diversity: {
    genderDiversityPercent: number;
    racialDiversityPercent: number;
    boardDiversityPercent: number;
    payEquityGap: number; // percentage gap
  };
  wellbeing: {
    employeeSatisfactionScore: number; // 0-10
    turnoverRate: number; // percentage
    benefitsScore: number; // 0-100
  };
  development: {
    trainingHoursPerEmployee: number;
    internalPromotionRate: number; // percentage
    skillsDevelopmentInvestment: number; // USD per employee
  };
}

export interface LaborPracticesMetrics {
  score: number;
  wages: {
    livingWageCompliance: number; // percentage of workforce
    minimumWageRatio: number; // ratio to local minimum wage
  };
  safety: {
    trir: number; // Total Recordable Incident Rate
    ltifr: number; // Lost Time Injury Frequency Rate
    fatalitiesCount: number;
  };
  rights: {
    freedomOfAssociation: boolean;
    collectiveBargainingCoverage: number; // percentage
    childLaborRisk: 'Low' | 'Medium' | 'High';
    forcedLaborRisk: 'Low' | 'Medium' | 'High';
  };
}

export interface SupplyChainMetrics {
  score: number;
  assessments: {
    tier1Coverage: number; // percentage
    auditsCompleted: number;
    criticalIssuesIdentified: number;
    remediationRate: number; // percentage
  };
  humanRights: {
    dueDiligenceCompleted: boolean;
    grievanceMechanismActive: boolean;
    modernSlaveryRisk: 'Low' | 'Medium' | 'High';
  };
  diversity: {
    diverseSupplierSpend: number; // USD
    diverseSupplierPercentage: number;
  };
}

export interface StakeholderEngagementMetrics {
  score: number;
  community: {
    investmentUSD: number;
    volunteerHours: number;
    localHiringRate: number; // percentage
  };
  customer: {
    satisfactionScore: number; // NPS or CSAT
    privacyIncidents: number;
    productRecalls: number;
  };
}

// ============================================================================
// Governance Pillar
// ============================================================================

export interface GovernanceScore {
  score: number; // 0-100
  components: {
    boardEffectiveness: BoardEffectivenessMetrics;
    executiveAccountability: ExecutiveAccountabilityMetrics;
    transparency: TransparencyMetrics;
    riskManagement: RiskManagementMetrics;
  };
}

export interface BoardEffectivenessMetrics {
  score: number;
  composition: {
    independencePercentage: number;
    womenPercentage: number;
    diversityPercentage: number;
    averageTenureYears: number;
  };
  esgOversight: {
    esgCommitteeExists: boolean;
    esgExpertiseOnBoard: boolean;
    esgMeetingsPerYear: number;
  };
}

export interface ExecutiveAccountabilityMetrics {
  score: number;
  compensation: {
    esgLinkedPercentage: number; // % of comp tied to ESG
    ceoPayRatio: number; // CEO to median employee
    sayOnPayApproval: number; // percentage approval
  };
  ethics: {
    codeOfConductExists: boolean;
    ethicsTrainingCompletion: number; // percentage
    ethicsViolations: number;
    whistleblowerReports: number;
  };
}

export interface TransparencyMetrics {
  score: number;
  disclosure: {
    sustainabilityReportPublished: boolean;
    tcfdAligned: boolean;
    sasbAligned: boolean;
    griAligned: boolean;
    assuranceLevel: 'None' | 'Limited' | 'Reasonable';
  };
  stakeholderComm: {
    investorMeetings: number;
    publicDisclosureScore: number; // 0-100
  };
}

export interface RiskManagementMetrics {
  score: number;
  climateRisk: {
    scenarioAnalysisCompleted: boolean;
    climateVaRCalculated: boolean;
    transitionPlanPublished: boolean;
  };
  cybersecurity: {
    boardOversight: boolean;
    incidentResponsePlan: boolean;
    dataBreaches: number;
  };
}

// ============================================================================
// Green Finance Types
// ============================================================================

export interface GreenBond {
  id: BondId;
  issuer: CompanyId;
  amount: number;
  currency: Currency;
  issueDate: Date;
  maturityDate: Date;
  couponRate: number;
  useOfProceeds: UseOfProceeds;
  certification: GreenBondCertification[];
  impactReporting: ImpactReport[];
}

export interface UseOfProceeds {
  renewableEnergy?: number; // percentage
  energyEfficiency?: number;
  cleanTransportation?: number;
  sustainableWater?: number;
  climateAdaptation?: number;
  circularEconomy?: number;
  biodiversity?: number;
}

export enum GreenBondCertification {
  CLIMATE_BONDS_STANDARD = 'Climate Bonds Standard',
  GREEN_BOND_PRINCIPLES = 'Green Bond Principles',
  EU_GREEN_BOND_STANDARD = 'EU Green Bond Standard'
}

export interface ImpactReport {
  period: string;
  reportDate: Date;
  co2Avoided: number; // tCO2e
  renewableEnergyGenerated: number; // MWh
  waterSaved: number; // m³
  wasteDiverted: number; // tons
  beneficiariesCount: number;
  verified: boolean;
}

export interface SustainabilityLinkedLoan {
  id: string;
  borrower: CompanyId;
  amount: number;
  currency: Currency;
  baselineRate: number;
  kpis: SustainabilityKPI[];
  verification: boolean;
}

export interface SustainabilityKPI {
  name: string;
  baseline: number;
  target: number;
  deadline: Date;
  rateAdjustment: number; // basis points
  achieved?: boolean;
}

// ============================================================================
// Reporting Types
// ============================================================================

export interface SustainabilityReport {
  id: ReportId;
  companyId: CompanyId;
  period: string;
  publishDate: Date;
  frameworks: ReportingFramework[];
  esgScore: ESGScore;
  assurance: AssuranceStatement;
  narratives: {
    strategy: string;
    materiality: string;
    targets: string;
    progress: string;
  };
}

export enum ReportingFramework {
  TCFD = 'TCFD',
  SASB = 'SASB',
  GRI = 'GRI',
  CDP = 'CDP',
  ISSB = 'ISSB',
  CSRD = 'CSRD'
}

export interface AssuranceStatement {
  provider: string;
  level: 'None' | 'Limited' | 'Reasonable';
  scope: string[];
  issueDate: Date;
  opinion: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface CalculateESGScoreRequest {
  companyId: CompanyId;
  period: string;
  environmental: Partial<EnvironmentalScore['components']>;
  social: Partial<SocialScore['components']>;
  governance: Partial<GovernanceScore['components']>;
}

export interface CalculateESGScoreResponse {
  success: boolean;
  data: ESGScore;
  errors?: string[];
}

export interface IssueGreenBondRequest {
  issuer: CompanyId;
  amount: number;
  currency: Currency;
  maturity: string; // ISO date
  couponRate: number;
  useOfProceeds: UseOfProceeds;
  certification: GreenBondCertification[];
}

export interface IssueGreenBondResponse {
  success: boolean;
  data: GreenBond;
  errors?: string[];
}

export interface GenerateReportRequest {
  companyId: CompanyId;
  frameworks: ReportingFramework[];
  period: string;
  format: 'pdf' | 'html' | 'json' | 'xbrl';
  includeAssurance?: boolean;
}

export interface GenerateReportResponse {
  success: boolean;
  reportId: ReportId;
  downloadUrl: string;
  errors?: string[];
}
