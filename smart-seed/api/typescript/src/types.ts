/**
 * WIA Smart Seed Standard - TypeScript Types
 * @version 1.0.0
 * @license MIT
 */

/**
 * Crop types
 */
export type CropType = 'grain' | 'vegetable' | 'fruit' | 'oilseed' | 'fiber' | 'forage' | 'ornamental' | 'medicinal' | 'cover-crop';

/**
 * Seed generations
 */
export type SeedGeneration = 'Breeder' | 'Foundation' | 'Registered' | 'Certified' | 'Commercial';

/**
 * Certification standards
 */
export type CertificationStandard = 'OECD' | 'AOSCA' | 'ISTA' | 'EU-Seed-Directive' | 'National-Certified';

/**
 * Disease status
 */
export type DiseaseStatus = 'clean' | 'infected' | 'unknown' | 'not-tested';

/**
 * License types for intellectual property
 */
export type LicenseType = 'open-source' | 'proprietary' | 'hybrid';

/**
 * Test standards
 */
export type TestStandard = 'ISTA' | 'AOSA' | 'National';

/**
 * Vigour classification
 */
export type VigourClassification = 'high' | 'medium' | 'low';

/**
 * Breeder information
 */
export interface Breeder {
  organizationName: string;
  breederName: string;
  country: string;
  did?: string;
}

/**
 * Parent lines for breeding
 */
export interface ParentLines {
  maternal: string;
  paternal: string;
  crossingDate: string;
}

/**
 * Variety characteristics
 */
export interface VarietyCharacteristics {
  maturityDays: number;
  plantHeight: number;
  yieldPotential: number;
  diseaseResistance: string[];
  climateAdaptation: string[];
  specialTraits: string[];
}

/**
 * Intellectual property information
 */
export interface IntellectualProperty {
  pvpNumber?: string;
  patentNumber?: string;
  upovRegistration?: string;
  registrationDate: string;
  protectionExpiry?: string;
  licenseType: LicenseType;
}

/**
 * Registration authority
 */
export interface RegistrationAuthority {
  country: string;
  agency: string;
  registrationNumber: string;
}

/**
 * Seed variety entity
 */
export interface SeedVariety {
  varietyId: string;
  varietyName: string;
  scientificName: string;
  commonName: string;
  cropType: CropType;
  breeder: Breeder;
  parentLines?: ParentLines;
  characteristics: VarietyCharacteristics;
  intellectualProperty: IntellectualProperty;
  registrationAuthority: RegistrationAuthority;
}

/**
 * Seed producer information
 */
export interface SeedProducer {
  name: string;
  license: string;
  location: string;
  did?: string;
}

/**
 * Field location
 */
export interface FieldLocation {
  latitude: number;
  longitude: number;
  elevation: number;
  soilType: string;
  previousCrop: string;
}

/**
 * Quantity measurement
 */
export interface Quantity {
  value: number;
  unit: string;
}

/**
 * Production information
 */
export interface ProductionInfo {
  producer: SeedProducer;
  productionDate: string;
  harvestDate: string;
  processingDate: string;
  quantity: Quantity;
  generation: SeedGeneration;
  fieldLocation: FieldLocation;
}

/**
 * Disease information
 */
export interface DiseaseInfo {
  tested: boolean;
  diseases: string[];
  status: DiseaseStatus;
}

/**
 * Quality metrics for seed lot
 */
export interface QualityMetrics {
  germinationRate: number;
  purity: number;
  moisture: number;
  vigourIndex: number;
  weightPer1000Seeds: number;
  diseaseStatus: DiseaseInfo;
  weedSeedContent: number;
  inertMatter: number;
}

/**
 * Testing records
 */
export interface TestingRecords {
  laboratoryName: string;
  istaAccredited: boolean;
  testDate: string;
  testMethod: string;
  testerName: string;
  certificateNumber: string;
}

/**
 * Seed lot data
 */
export interface SeedLot {
  lotId: string;
  varietyId: string;
  productionInfo: ProductionInfo;
  qualityMetrics: QualityMetrics;
  testingRecords: TestingRecords;
}

/**
 * Laboratory information
 */
export interface Laboratory {
  name: string;
  accreditation: string;
  location: string;
  did?: string;
}

/**
 * Test conditions
 */
export interface TestConditions {
  temperature: number;
  substrate: string;
  duration: number;
  lightCondition: string;
  replicates: number;
  seedsPerReplicate: number;
}

/**
 * Germination test results
 */
export interface GerminationResults {
  normalSeedlings: number;
  abnormalSeedlings: number;
  deadSeeds: number;
  dormantSeeds: number;
  germinationPercentage: number;
  vigourClassification: VigourClassification;
  speedOfGermination: number;
}

/**
 * Test image
 */
export interface TestImage {
  imageUrl: string;
  captureDate: string;
  description: string;
}

/**
 * Verification data
 */
export interface Verification {
  verifiedBy: string;
  signature: string;
  timestamp: string;
}

/**
 * Germination test record
 */
export interface GerminationTest {
  testId: string;
  lotId: string;
  testStandard: TestStandard;
  testDate: string;
  laboratory: Laboratory;
  testConditions: TestConditions;
  results: GerminationResults;
  images?: TestImage[];
  verification: Verification;
}

/**
 * Issuing authority for certification
 */
export interface IssuingAuthority {
  name: string;
  country: string;
  accreditation: string;
  did?: string;
}

/**
 * Certification criteria
 */
export interface CertificationCriteria {
  minimumGermination: number;
  minimumPurity: number;
  maximumMoisture: number;
  maximumWeedSeeds: number;
  diseaseStatus: string;
}

/**
 * Compliance status
 */
export interface ComplianceStatus {
  germination: boolean;
  purity: boolean;
  moisture: boolean;
  diseaseScreen: boolean;
  fieldInspection: boolean;
  overallCompliance: boolean;
}

/**
 * Field inspection data
 */
export interface FieldInspection {
  inspectionDate: string;
  inspectorName: string;
  isolationDistance: number;
  varietyPurity: number;
  offTypes: number;
  diseaseIncidence: number;
}

/**
 * Blockchain record
 */
export interface BlockchainRecord {
  transactionHash: string;
  blockNumber: number;
  network: string;
  timestamp: string;
}

/**
 * Seed certification
 */
export interface SeedCertification {
  certificateId: string;
  lotId: string;
  certificationType: SeedGeneration;
  certificationScheme: CertificationStandard;
  issuingAuthority: IssuingAuthority;
  issueDate: string;
  validUntil: string;
  certificationCriteria: CertificationCriteria;
  complianceStatus: ComplianceStatus;
  fieldInspection: FieldInspection;
  blockchainRecord?: BlockchainRecord;
}

/**
 * Seed origin information
 */
export interface SeedOrigin {
  country: string;
  region: string;
  producer: string;
  productionYear: number;
}

/**
 * Quality summary for passport
 */
export interface QualitySummary {
  germinationRate: number;
  purity: number;
  certificationStatus: string;
  organicCertified: boolean;
  gmoCertified: string;
}

/**
 * Traceability chain event
 */
export interface TraceabilityEvent {
  stage: string;
  actor: string;
  location: string;
  timestamp: string;
  action: string;
  blockchainHash?: string;
}

/**
 * Verifiable credential proof
 */
export interface VCProof {
  type: string;
  created: string;
  proofPurpose: string;
  verificationMethod: string;
  proofValue: string;
}

/**
 * Credential subject
 */
export interface CredentialSubject {
  id: string;
  lotId: string;
  varietyName: string;
  certificationStatus: string;
}

/**
 * Verifiable credential
 */
export interface VerifiableCredential {
  '@context': string[];
  type: string[];
  issuer: string;
  issuanceDate: string;
  credentialSubject: CredentialSubject;
  proof: VCProof;
}

/**
 * Digital seed passport
 */
export interface DigitalSeedPassport {
  passportId: string;
  qrCode: string;
  lotId: string;
  varietyId: string;
  issueDate: string;
  expiryDate: string;
  seedOrigin: SeedOrigin;
  qualitySummary: QualitySummary;
  traceabilityChain: TraceabilityEvent[];
  verifiableCredential: VerifiableCredential;
}

/**
 * API Response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
  timestamp: string;
}

/**
 * Query parameters
 */
export interface SeedQuery {
  varietyId?: string;
  lotId?: string;
  cropType?: CropType;
  producer?: string;
  limit?: number;
  offset?: number;
}
