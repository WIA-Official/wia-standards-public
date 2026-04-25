/**
 * WIA-SOC-016 Census Data Standard - TypeScript Type Definitions
 *
 * @packageDocumentation
 * @module @wia/census-data-sdk
 */

/**
 * Enumeration for relationship to household head
 */
export enum Relationship {
  HEAD = 'HEAD',
  SPOUSE = 'SPOUSE',
  CHILD = 'CHILD',
  PARENT = 'PARENT',
  OTHER_RELATIVE = 'OTHER_RELATIVE',
  NONRELATIVE = 'NONRELATIVE',
}

/**
 * Enumeration for biological sex
 */
export enum Sex {
  MALE = 'MALE',
  FEMALE = 'FEMALE',
  OTHER = 'OTHER',
  PREFER_NOT_TO_SAY = 'PREFER_NOT_TO_SAY',
}

/**
 * Enumeration for marital status
 */
export enum MaritalStatus {
  SINGLE = 'SINGLE',
  MARRIED = 'MARRIED',
  DIVORCED = 'DIVORCED',
  WIDOWED = 'WIDOWED',
  SEPARATED = 'SEPARATED',
}

/**
 * Enumeration for citizenship status
 */
export enum Citizenship {
  CITIZEN_BIRTH = 'CITIZEN_BIRTH',
  CITIZEN_NATURALIZED = 'CITIZEN_NATURALIZED',
  PERMANENT_RESIDENT = 'PERMANENT_RESIDENT',
  TEMPORARY = 'TEMPORARY',
  NONE = 'NONE',
}

/**
 * Enumeration for education levels
 */
export enum EducationLevel {
  NONE = 'NONE',
  PRIMARY = 'PRIMARY',
  LOWER_SECONDARY = 'LOWER_SECONDARY',
  UPPER_SECONDARY = 'UPPER_SECONDARY',
  POST_SECONDARY_NON_TERTIARY = 'POST_SECONDARY_NON_TERTIARY',
  SHORT_CYCLE_TERTIARY = 'SHORT_CYCLE_TERTIARY',
  BACHELORS = 'BACHELORS',
  MASTERS = 'MASTERS',
  DOCTORATE = 'DOCTORATE',
}

/**
 * Enumeration for employment status
 */
export enum EmploymentStatus {
  EMPLOYED_FULL_TIME = 'EMPLOYED_FULL_TIME',
  EMPLOYED_PART_TIME = 'EMPLOYED_PART_TIME',
  SELF_EMPLOYED = 'SELF_EMPLOYED',
  UNEMPLOYED_LOOKING = 'UNEMPLOYED_LOOKING',
  UNEMPLOYED_NOT_LOOKING = 'UNEMPLOYED_NOT_LOOKING',
  RETIRED = 'RETIRED',
  STUDENT = 'STUDENT',
  HOMEMAKER = 'HOMEMAKER',
  UNABLE_TO_WORK = 'UNABLE_TO_WORK',
}

/**
 * Enumeration for housing types
 */
export enum HousingType {
  DETACHED_HOUSE = 'DETACHED_HOUSE',
  SEMI_DETACHED = 'SEMI_DETACHED',
  TOWNHOUSE = 'TOWNHOUSE',
  APARTMENT = 'APARTMENT',
  MOBILE_HOME = 'MOBILE_HOME',
  INSTITUTIONAL = 'INSTITUTIONAL',
  HOMELESS = 'HOMELESS',
  OTHER = 'OTHER',
}

/**
 * Enumeration for housing tenure
 */
export enum Tenure {
  OWNED_OUTRIGHT = 'OWNED_OUTRIGHT',
  OWNED_MORTGAGE = 'OWNED_MORTGAGE',
  RENTED = 'RENTED',
  PROVIDED_FREE = 'PROVIDED_FREE',
  OTHER = 'OTHER',
}

/**
 * Geographic location information
 */
export interface Geography {
  /** Geographic code (e.g., USA-CA-LOS-ANGELES) */
  code: string;
  /** Geographic name */
  name: string;
  /** Geographic level */
  level: 'nation' | 'state' | 'county' | 'city' | 'tract' | 'block';
  /** Country code (ISO 3166-1 alpha-3) */
  country: string;
  /** Latitude */
  latitude?: number;
  /** Longitude */
  longitude?: number;
}

/**
 * Person demographic data
 */
export interface Person {
  /** Unique person identifier */
  personId: string;
  /** Reference to household */
  householdId: string;
  /** Relationship to household head */
  relationship: Relationship;
  /** Age in years */
  age: number;
  /** Age group category */
  ageGroup?: string;
  /** Biological sex */
  sex: Sex;
  /** Gender identity (optional) */
  gender?: string;
  /** Marital status */
  maritalStatus: MaritalStatus;
  /** Citizenship status */
  citizenship: Citizenship;
  /** Country of birth (ISO 3166-1 alpha-3) */
  countryOfBirth: string;
  /** Languages spoken (ISO 639-2 codes) */
  languagesSpoken: string[];
  /** Ethnicity */
  ethnicity?: string[];
  /** Race */
  race?: string[];
  /** Religion (optional) */
  religion?: string;
  /** Has disability */
  disability: boolean;
  /** Types of disabilities */
  disabilityTypes?: string[];
}

/**
 * Education information
 */
export interface Education {
  /** Reference to person */
  personId: string;
  /** Highest education level completed */
  highestLevelCompleted: EducationLevel;
  /** Current school attendance */
  schoolAttendance?: string;
  /** Field of study (ISCED-F code) */
  fieldOfStudy?: string;
  /** Literacy status */
  literacy: boolean;
}

/**
 * Economic activity information
 */
export interface EconomicActivity {
  /** Reference to person */
  personId: string;
  /** Employment status */
  employmentStatus: EmploymentStatus;
  /** Occupation (ISCO-08 code) */
  occupation?: string;
  /** Industry (ISIC Rev.4 code) */
  industry?: string;
  /** Hours worked per week */
  workHours?: number;
  /** Annual income */
  income?: number;
  /** Income currency (ISO 4217) */
  currency?: string;
  /** Income range category */
  incomeRange?: string;
  /** Commute mode */
  commuteMode?: string;
  /** Commute time in minutes */
  commuteTime?: number;
}

/**
 * Household information
 */
export interface Household {
  /** Unique household identifier */
  householdId: string;
  /** Reference to dwelling */
  dwellingId: string;
  /** Household type */
  householdType: string;
  /** Number of people in household */
  householdSize: number;
  /** Number of children */
  numberChildren: number;
  /** Number of adults */
  numberAdults: number;
  /** Reference to head of household */
  headOfHouseholdId: string;
  /** Total household income */
  householdIncome?: number;
  /** Income currency */
  currency?: string;
  /** Receives government benefits */
  receivesBenefits: boolean;
  /** Types of benefits received */
  benefitTypes?: string[];
}

/**
 * Dwelling/housing information
 */
export interface Dwelling {
  /** Unique dwelling identifier */
  dwellingId: string;
  /** Street address */
  address: {
    streetNumber?: string;
    streetName?: string;
    apartment?: string;
    city: string;
    stateProvince: string;
    postalCode: string;
    country: string;
  };
  /** Geographic coordinates */
  geocode?: {
    latitude: number;
    longitude: number;
    censusBlock?: string;
    censusTract?: string;
    county?: string;
    state?: string;
  };
  /** Type of housing */
  housingType: HousingType;
  /** Ownership status */
  tenure: Tenure;
  /** Year built */
  yearBuilt?: number;
  /** Number of rooms */
  rooms?: number;
  /** Number of bedrooms */
  bedrooms?: number;
  /** Number of bathrooms */
  bathrooms?: number;
  /** Floor area in square meters */
  floorArea?: number;
  /** Available utilities */
  utilities?: {
    electricity: boolean;
    naturalGas: boolean;
    water: boolean;
    sewage: boolean;
    internet: boolean;
    internetType?: string;
  };
  /** Monthly housing cost */
  monthlyHousingCost?: number;
  /** Property value */
  propertyValue?: number;
}

/**
 * Population statistics response
 */
export interface PopulationStatistics {
  /** Geographic area */
  geography: Geography;
  /** Census year */
  year: number;
  /** Total population */
  total: number;
  /** Male population */
  male: number;
  /** Female population */
  female: number;
  /** Median age */
  medianAge: number;
  /** Population by age group */
  byAgeGroup?: Record<string, number>;
  /** Population by sex and age */
  byAgeAndSex?: Array<{
    ageGroup: string;
    sex: Sex;
    count: number;
  }>;
}

/**
 * Data quality metadata
 */
export interface DataQuality {
  /** Response rate */
  responseRate: number;
  /** Coverage rate */
  coverageRate: number;
  /** Imputation rates by variable */
  imputationRates: Record<string, number>;
  /** Standard errors */
  standardErrors?: Record<string, number>;
  /** Confidence level */
  confidenceLevel: number;
  /** Privacy method applied */
  privacyMethod: string;
  /** Privacy budget (epsilon) */
  privacyBudget?: number;
}

/**
 * API response wrapper
 */
export interface CensusResponse<T> {
  /** Response data */
  data: T;
  /** Metadata about the response */
  metadata: {
    /** Data source */
    source: string;
    /** Census year */
    year: number;
    /** Timestamp */
    timestamp: string;
    /** Privacy method */
    privacyMethod?: string;
    /** Privacy budget */
    epsilon?: number;
  };
  /** Data quality indicators */
  quality?: DataQuality;
}

/**
 * API error response
 */
export interface CensusError {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** HTTP status code */
  status: number;
  /** Additional error details */
  details?: Record<string, any>;
}

/**
 * Query parameters for population endpoint
 */
export interface PopulationQueryParams {
  /** Geographic code */
  geoCode?: string;
  /** Census year */
  year?: number;
  /** Geographic level */
  geoLevel?: 'nation' | 'state' | 'county' | 'tract' | 'block';
  /** Age group filter */
  ageGroup?: string;
  /** Sex filter */
  sex?: Sex;
  /** Response format */
  format?: 'json' | 'csv' | 'xml';
}

/**
 * Pagination metadata
 */
export interface Pagination {
  /** Current page number */
  page: number;
  /** Items per page */
  perPage: number;
  /** Total number of items */
  total: number;
  /** Total number of pages */
  totalPages: number;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Data items */
  data: T[];
  /** Pagination info */
  pagination: Pagination;
  /** Navigation links */
  links: {
    first?: string;
    prev?: string;
    next?: string;
    last?: string;
  };
}

/**
 * API configuration options
 */
export interface CensusAPIConfig {
  /** API base URL */
  baseURL?: string;
  /** API key for authentication */
  apiKey: string;
  /** Request timeout in milliseconds */
  timeout?: number;
  /** Enable retry on failure */
  retry?: boolean;
  /** Maximum number of retries */
  maxRetries?: number;
}
