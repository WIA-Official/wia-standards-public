/**
 * WIA-SOC-020 Labor Market Data Standard - TypeScript Type Definitions
 * Version: 1.0.0
 * 
 * 弘益人間 (Benefit All Humanity)
 */

/**
 * Worker identifier format: WKR-YYYY-NNNNNN
 */
export type WorkerId = string;

/**
 * Employer identifier format: ORG-NNNNNN
 */
export type EmployerId = string;

/**
 * Job posting identifier format: JOB-YYYY-NNNNNN
 */
export type JobPostingId = string;

/**
 * Skill identifier format: SKL-NNNNNN
 */
export type SkillId = string;

/**
 * Employment types
 */
export enum EmploymentType {
  FullTime = 'full-time',
  PartTime = 'part-time',
  Contract = 'contract',
  Freelance = 'freelance',
  Temporary = 'temporary',
  Seasonal = 'seasonal',
  Internship = 'internship',
  Apprenticeship = 'apprenticeship'
}

/**
 * Work arrangements
 */
export enum WorkArrangement {
  OnSite = 'on-site',
  Remote = 'remote',
  Hybrid = 'hybrid',
  Distributed = 'distributed',
  FieldBased = 'field-based'
}

/**
 * Education levels (ISCED)
 */
export enum EducationLevel {
  EarlyChildhood = 0,
  Primary = 1,
  LowerSecondary = 2,
  UpperSecondary = 3,
  PostSecondary = 4,
  ShortCycleTertiary = 5,
  Bachelor = 6,
  Master = 7,
  Doctoral = 8
}

/**
 * Geographic location
 */
export interface Location {
  city?: string;
  state?: string;
  country: string;
  region?: string;
  postalCode?: string;
  coordinates?: {
    latitude: number;
    longitude: number;
  };
}

/**
 * Industry classification
 */
export interface IndustryCode {
  naics?: string;
  isic?: string;
  nace?: string;
  description: string;
}

/**
 * Occupation classification
 */
export interface OccupationCode {
  soc?: string;
  isco?: string;
  onet?: string;
  title: string;
}

/**
 * Skill definition
 */
export interface Skill {
  skillId: SkillId;
  name: string;
  aliases?: string[];
  category: 'Technical' | 'Professional' | 'Soft Skill' | 'Industry-Specific';
  subcategory?: string;
  proficiencyLevel: 1 | 2 | 3 | 4 | 5;
  verification?: {
    method: 'certification' | 'assessment' | 'experience' | 'self-reported';
    source?: string;
    date?: string;
    expiryDate?: string;
  };
  relatedSkills?: SkillId[];
  demandIndex?: number;
  trendDirection?: 'increasing' | 'stable' | 'decreasing';
}

/**
 * Compensation details
 */
export interface Compensation {
  baseSalary: number;
  currency: string;
  period: 'hourly' | 'daily' | 'weekly' | 'monthly' | 'annual';
  benefits?: string[];
  equity?: boolean;
  bonusStructure?: string;
}

/**
 * Employment record
 */
export interface Employment {
  employmentId: string;
  employer: {
    employerId: EmployerId;
    name: string;
    industry: IndustryCode;
    size?: string;
    location: Location;
  };
  position: {
    title: string;
    occupationCode: OccupationCode;
    level?: string;
    department?: string;
  };
  period: {
    startDate: string;
    endDate?: string | null;
    isCurrent: boolean;
    tenure?: string;
  };
  employmentType: EmploymentType;
  workArrangement: WorkArrangement;
  hoursPerWeek?: number;
  compensation: Compensation;
}

/**
 * Education record
 */
export interface Education {
  institution: string;
  degree: string;
  field: string;
  level: EducationLevel;
  startDate: string;
  endDate?: string;
  graduated: boolean;
  gpa?: number;
  location: Location;
}

/**
 * Certification record
 */
export interface Certification {
  name: string;
  issuer: string;
  issueDate: string;
  expiryDate?: string;
  credentialId?: string;
  verificationUrl?: string;
}

/**
 * Worker profile
 */
export interface WorkerProfile {
  workerId: WorkerId;
  personalInfo: {
    name: string;
    dateOfBirth?: string;
    gender?: 'male' | 'female' | 'non-binary' | 'prefer-not-to-say';
    location: Location;
  };
  skills: Skill[];
  experience: Employment[];
  education: Education[];
  certifications?: Certification[];
  metadata?: {
    createdAt: string;
    updatedAt: string;
    version: string;
  };
}

/**
 * Job posting
 */
export interface JobPosting {
  postingId: JobPostingId;
  employer: EmployerId;
  title: string;
  description: string;
  requirements: {
    education?: {
      level: EducationLevel;
      field?: string;
    };
    experience?: {
      minYears: number;
      maxYears?: number;
      preferred?: number;
    };
    skills: {
      required: SkillId[];
      preferred?: SkillId[];
    };
    certifications?: {
      required?: string[];
      preferred?: string[];
    };
  };
  compensation: {
    salaryRange?: {
      min: number;
      max: number;
      currency: string;
      period: 'annual' | 'monthly' | 'hourly';
    };
    equity?: boolean;
    benefits?: string;
    bonusStructure?: string;
  };
  location: Location & {
    remote?: WorkArrangement;
    travelRequired?: string;
  };
  posted: string;
  validUntil: string;
  status: 'open' | 'closed' | 'filled' | 'cancelled';
}

/**
 * Wage statistics
 */
export interface WageStatistics {
  occupation: string;
  region: string;
  period: string;
  statistics: {
    mean: number;
    median: number;
    mode?: number;
    percentile10: number;
    percentile25: number;
    percentile75: number;
    percentile90: number;
    standardDeviation?: number;
  };
  currency: string;
  sampleSize: number;
  confidenceLevel?: number;
  marginOfError?: number;
  lastUpdated: string;
  purchasingPowerParity?: {
    localValue: number;
    normalizedUSD: number;
    colIndex: number;
    pppFactor: number;
  };
}

/**
 * Labor market statistics
 */
export interface LaborMarketStatistics {
  region: string;
  period: string;
  employment: {
    employed: number;
    unemployed: number;
    laborForce: number;
    participationRate: number;
    unemploymentRate: number;
  };
  demographics?: {
    ageGroups: Record<string, number>;
    gender: Record<string, number>;
    education: Record<string, number>;
  };
  industries?: Record<string, number>;
  occupations?: Record<string, number>;
  trends?: {
    direction: 'increasing' | 'stable' | 'decreasing';
    changePercent: number;
    period: string;
  };
}

/**
 * Quality metrics
 */
export interface QualityMetrics {
  completeness: number;
  freshness: string;
  coverage: {
    populationSize: number;
    sampleSize: number;
    samplingMethod: string;
  };
  accuracy?: {
    confidenceLevel: number;
    marginOfError: number;
  };
  provenance: {
    source: string;
    trustScore: number;
    verificationMethod: string;
  };
}

/**
 * API response wrapper
 */
export interface ApiResponse<T> {
  data: T;
  metadata: {
    requestId: string;
    timestamp: string;
    version: string;
  };
  qualityMetrics?: QualityMetrics;
}

/**
 * API error response
 */
export interface ApiError {
  code: string;
  message: string;
  details?: Record<string, any>;
  timestamp: string;
  requestId: string;
}
