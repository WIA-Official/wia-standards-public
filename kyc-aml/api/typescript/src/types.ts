/**
 * WIA-FIN-011 KYC/AML TypeScript SDK - Type Definitions
 * @version 1.0.0
 */

// ==================== Core Types ====================

export type CountryCode = string; // ISO 3166-1 alpha-3
export type CurrencyCode = string; // ISO 4217
export type PhoneNumber = string; // E.164 format
export type Email = string;
export type UUID = string;

// ==================== Person Data ====================

export interface PersonalName {
  firstName: string;
  middleName?: string;
  lastName: string;
  suffix?: string;
  fullName: string;
}

export interface FormerName {
  name: string;
  effectiveDate: Date;
  endDate?: Date;
}

export interface PlaceOfBirth {
  city: string;
  state?: string;
  country: CountryCode;
}

export type Gender = 'M' | 'F' | 'X' | 'U';

export interface PersonalInfo {
  legalName: PersonalName;
  formerNames?: FormerName[];
  dateOfBirth: Date;
  placeOfBirth?: PlaceOfBirth;
  gender?: Gender;
  nationality: CountryCode[];
  citizenship: CountryCode[];
}

// ==================== Identification Documents ====================

export type DocumentType = 'passport' | 'drivers_license' | 'national_id' | 'other';
export type VerificationStatus = 'verified' | 'pending' | 'failed' | 'expired';

export interface DocumentImage {
  imageType: 'front' | 'back' | 'full_page';
  imageUrl: string;
  imageHash: string;
  uploadDate: Date;
}

export interface IdentificationDocument {
  documentType: DocumentType;
  documentNumber: string;
  issuingCountry: CountryCode;
  issuingAuthority: string;
  issueDate: Date;
  expiryDate: Date;
  verificationStatus: VerificationStatus;
  verificationDate?: Date;
  verificationMethod?: string;
  documentImages?: DocumentImage[];
}

// ==================== Address ====================

export type AddressType = 'residential' | 'mailing' | 'business';

export interface Address {
  addressType: AddressType;
  isPrimary: boolean;
  streetAddress1: string;
  streetAddress2?: string;
  city: string;
  stateProvince?: string;
  postalCode: string;
  country: CountryCode;
  effectiveDate: Date;
  endDate?: Date;
  verificationStatus?: 'verified' | 'unverified';
  verificationMethod?: string;
  verificationDate?: Date;
}

// ==================== Contact Information ====================

export interface EmailContact {
  email: Email;
  emailType: 'personal' | 'work';
  isPrimary: boolean;
  verified: boolean;
  verificationDate?: Date;
}

export interface PhoneContact {
  phoneNumber: PhoneNumber;
  phoneType: 'mobile' | 'home' | 'work';
  isPrimary: boolean;
  verified: boolean;
  verificationDate?: Date;
}

export interface ContactInfo {
  emails: EmailContact[];
  phoneNumbers: PhoneContact[];
}

// ==================== Tax Identifiers ====================

export type TaxIdType = 'ssn' | 'tin' | 'vat' | 'other';

export interface TaxIdentifier {
  taxIdType: TaxIdType;
  taxId: string;
  issuingCountry: CountryCode;
  verified: boolean;
}

// ==================== Occupation ====================

export type EmploymentStatus = 'employed' | 'self_employed' | 'retired' | 'student' | 'unemployed';

export interface Occupation {
  employmentStatus: EmploymentStatus;
  occupation: string;
  industryCode?: string; // NAICS code
  employer?: string;
  employerAddress?: Address;
  positionTitle?: string;
  yearsEmployed?: number;
}

// ==================== PEP ====================

export type PEPType = 'domestic' | 'foreign' | 'international_organization';
export type PEPRelationship = 'self' | 'family_member' | 'close_associate';

export interface PEPInfo {
  isPEP: boolean;
  pepType?: PEPType;
  pepRole?: string;
  pepCountry?: CountryCode;
  pepStartDate?: Date;
  pepEndDate?: Date;
  isFormerPEP: boolean;
  relationship: PEPRelationship;
}

// ==================== Person ====================

export interface Person {
  personId: UUID;
  personalInfo: PersonalInfo;
  identificationDocuments: IdentificationDocument[];
  addresses: Address[];
  contactInfo: ContactInfo;
  taxIdentifiers?: TaxIdentifier[];
  occupation?: Occupation;
  politicallyExposedPerson?: PEPInfo;
}

// ==================== Legal Entity ====================

export type EntityType = 'corporation' | 'llc' | 'partnership' | 'trust' | 'foundation' | 'ngo' | 'other';

export interface EntityInfo {
  legalName: string;
  tradingNames?: string[];
  formerNames?: FormerName[];
  entityType: EntityType;
  incorporationDate: Date;
  jurisdictionOfIncorporation: CountryCode;
  registrationNumber: string;
  taxIdentifiers: TaxIdentifier[];
  businessAddresses: {
    addressType: 'registered' | 'principal' | 'branch';
    address: Address;
    effectiveDate: Date;
  }[];
  contactInfo: {
    mainPhone: PhoneNumber;
    mainEmail: Email;
    website?: string;
  };
  businessActivities: {
    primaryActivity: string;
    industryCode: string;
    description: string;
    licenses?: {
      licenseType: string;
      licenseNumber: string;
      issuingAuthority: string;
      issueDate: Date;
      expiryDate: Date;
    }[];
  };
}

export interface BeneficialOwner {
  ownerId: UUID;
  personRef: Person | UUID;
  ownershipPercentage: number;
  ownershipType: 'direct' | 'indirect';
  controlMechanism: 'equity' | 'voting_rights' | 'other';
  effectiveDate: Date;
}

export interface ControllingPerson {
  personRef: Person | UUID;
  controlType: 'senior_officer' | 'authorized_signatory' | 'other';
  position: string;
}

export interface OwnershipStructure {
  beneficialOwners: BeneficialOwner[];
  controllingPersons: ControllingPerson[];
  corporateStructure?: {
    parentEntity: UUID;
    childEntity: UUID;
    ownershipPercentage: number;
    relationshipType: 'subsidiary' | 'affiliate' | 'branch';
  }[];
}

export interface LegalEntity {
  entityId: UUID;
  entityInfo: EntityInfo;
  ownershipStructure: OwnershipStructure;
  financialInfo?: {
    annualRevenue: {
      amount: number;
      currency: CurrencyCode;
      fiscalYear: number;
    };
    employeeCount: number;
    sourceOfFunds: string;
    expectedTransactionVolume: {
      volumeRange: 'low' | 'medium' | 'high' | 'very_high';
      monthlyAmount: number;
      currency: CurrencyCode;
    };
  };
}

// ==================== Customer Due Diligence ====================

export type CustomerType = 'individual' | 'entity';
export type CDDType = 'standard' | 'simplified' | 'enhanced';

export interface CDDRecord {
  cddId: UUID;
  customerId: UUID;
  customerType: CustomerType;
  cddType: CDDType;
  cddDate: Date;
  performedBy: {
    userId: UUID;
    userName: string;
    department: string;
  };
  identificationVerification: {
    method: 'documentary' | 'non_documentary' | 'biometric' | 'digital_identity';
    verificationStatus: 'passed' | 'failed' | 'partial';
    verificationDate: Date;
    documentsReviewed: UUID[];
    dataSourcesUsed: string[];
    verificationNotes?: string;
  };
  beneficialOwnershipVerification?: {
    verificationStatus: 'complete' | 'incomplete' | 'not_applicable';
    beneficialOwnersIdentified: number;
    verificationDate: Date;
    ownershipChart?: string;
    verificationNotes?: string;
  };
  sourceOfFundsWealth: {
    sourceOfFunds: 'employment' | 'business' | 'inheritance' | 'investment' | 'other';
    sourceOfFundsDescription: string;
    sourceOfWealth: string;
    sourceOfWealthDescription: string;
    verificationEvidence: UUID[];
    verificationStatus: 'verified' | 'partially_verified' | 'unverified';
  };
  purposeOfRelationship: {
    accountPurpose: string;
    intendedAccountActivity: string;
    expectedTransactionTypes: string[];
    expectedGeographicScope: CountryCode[];
  };
  riskAssessment: RiskAssessment;
  screeningResults: ScreeningResults;
  ongoingMonitoring: {
    reviewFrequency: 'continuous' | 'monthly' | 'quarterly' | 'annually' | 'biannually';
    lastReviewDate: Date;
    nextReviewDate: Date;
    monitoringAlerts: number;
    transactionVolume?: Record<string, any>;
  };
  documentation: {
    documentType: string;
    documentRef: string;
    uploadDate: Date;
    expiryDate?: Date;
  }[];
  complianceNotes?: string;
  recordStatus: 'active' | 'inactive' | 'closed';
  recordVersion: number;
  lastUpdated: Date;
}

// ==================== Risk Assessment ====================

export type RiskRating = 'low' | 'medium' | 'high' | 'prohibited';

export interface RiskFactor {
  factorType: 'customer' | 'geographic' | 'product' | 'delivery_channel';
  factorDescription: string;
  riskLevel: RiskRating;
  riskScore: number;
}

export interface RiskAssessment {
  overallRiskRating: RiskRating;
  riskScore: number; // 0-100
  riskFactors: RiskFactor[];
  mitigatingFactors?: string[];
  aggravatingFactors?: string[];
  assessmentDate: Date;
  assessedBy: UUID;
  approvalRequired: boolean;
  approvedBy?: UUID;
  approvalDate?: Date;
}

// ==================== Screening ====================

export interface ScreeningMatch {
  listName: string;
  matchedName: string;
  matchScore: number;
  matchStatus: 'true_match' | 'false_positive' | 'under_review';
  reviewedBy?: UUID;
  reviewDate?: Date;
  reviewNotes?: string;
}

export interface ScreeningResults {
  sanctionsScreening: {
    screeningDate: Date;
    listsChecked: string[];
    matchesFound: number;
    matches: ScreeningMatch[];
    clearanceStatus: 'clear' | 'hit' | 'under_review';
  };
  pepScreening: {
    screeningDate: Date;
    pepStatus: 'not_pep' | 'pep' | 'former_pep';
    pepDetails?: any;
    clearanceStatus: 'clear' | 'hit' | 'under_review';
  };
  adverseMediaScreening: {
    screeningDate: Date;
    articlesFound: number;
    relevantArticles: {
      source: string;
      date: Date;
      title: string;
      summary: string;
      relevanceScore: number;
      category: 'financial_crime' | 'corruption' | 'sanctions' | 'other';
    }[];
    riskIndicator: 'none' | 'low' | 'medium' | 'high';
  };
}

// ==================== Transaction ====================

export type TransactionType = 'deposit' | 'withdrawal' | 'transfer' | 'wire' | 'payment' | 'exchange';

export interface Transaction {
  transactionId: UUID;
  transactionType: TransactionType;
  transactionDate: Date;
  valueDate: Date;
  amount: {
    value: number;
    currency: CurrencyCode;
  };
  convertedAmount?: {
    value: number;
    currency: CurrencyCode;
    exchangeRate: number;
  };
  originator: {
    customerId?: UUID;
    accountNumber: string;
    name: string;
    address: Address;
    identificationNumber?: string;
  };
  beneficiary: {
    customerId?: UUID;
    accountNumber: string;
    name: string;
    address: Address;
    identificationNumber?: string;
  };
  intermediaries?: {
    institutionName: string;
    institutionIdentifier: string;
    country: CountryCode;
  }[];
  transactionDetails: {
    paymentPurpose: string;
    description: string;
    referenceNumber: string;
    relatedTransactions?: UUID[];
  };
  geographicInfo: {
    originatingCountry: CountryCode;
    destinationCountry: CountryCode;
    transactionLocation?: string;
  };
  channelInfo: {
    deliveryChannel: 'branch' | 'atm' | 'online' | 'mobile' | 'phone';
    initiationMethod: 'customer' | 'automated' | 'third_party';
    deviceInfo?: {
      deviceId: string;
      ipAddress: string;
      geolocation?: string;
    };
  };
  riskIndicators: {
    riskScore: number;
    riskLevel: RiskRating;
    riskFactors: string[];
    unusualIndicators: string[];
  };
  monitoringStatus: {
    reviewed: boolean;
    reviewedBy?: UUID;
    reviewDate?: Date;
    alerts: Alert[];
  };
  complianceFlags: {
    ctrFiled: boolean;
    ctrFilingDate?: Date;
    sarFiled: boolean;
    sarFilingDate?: Date;
    blocked: boolean;
    blockReason?: string;
  };
}

export interface Alert {
  alertId: UUID;
  alertType: string;
  alertDate: Date;
  alertStatus: 'open' | 'investigating' | 'closed';
  disposition?: 'false_positive' | 'suspicious' | 'escalated';
}

// ==================== SAR ====================

export type SARReportType = 'initial' | 'continuing' | 'corrected' | 'late';
export type SARActivityType = 
  | 'structuring'
  | 'layering'
  | 'trade_based_ml'
  | 'terrorist_financing'
  | 'fraud'
  | 'identity_theft'
  | 'elder_abuse'
  | 'human_trafficking'
  | 'cybercrime'
  | 'sanctions_violation'
  | 'other';

export interface SuspiciousActivityReport {
  sarId: UUID;
  filingInstitution: {
    institutionName: string;
    institutionIdentifier: string;
    institutionAddress: Address;
    contactPerson: {
      name: string;
      title: string;
      phone: PhoneNumber;
      email: Email;
    };
  };
  reportType: SARReportType;
  reportDate: Date;
  regulatoryFilingId?: string;
  filedWith: string[];
  subject: {
    subjectType: 'individual' | 'entity' | 'both';
    subjects: {
      subjectId: UUID;
      role: 'primary' | 'secondary';
      customerRef: UUID;
      relationshipToInstitution: 'customer' | 'employee' | 'other';
      accountsInvolved: string[];
    }[];
  };
  suspiciousActivity: {
    activityType: SARActivityType[];
    activityStartDate: Date;
    activityEndDate: Date;
    totalAmount: {
      value: number;
      currency: CurrencyCode;
    };
    transactionsInvolved: UUID[];
    narrative: string;
    redFlags: string[];
    investigationSummary: string;
  };
  documentationAttached?: {
    documentType: string;
    documentRef: string;
    description: string;
  }[];
  priorReports?: {
    priorSARId: UUID;
    priorFilingDate: Date;
    relationship: string;
  }[];
  lawEnforcementContact?: {
    contacted: boolean;
    contactDate?: Date;
    agency?: string;
    caseNumber?: string;
  };
  internalTracking: {
    caseNumber: string;
    assignedTo: UUID;
    investigationStartDate: Date;
    investigationCloseDate?: Date;
    approvals: {
      approverName: string;
      approverTitle: string;
      approvalDate: Date;
    }[];
  };
  confidentiality: {
    restrictedAccess: boolean;
    accessList: UUID[];
    safeguards: string;
  };
}

// ==================== API Types ====================

export interface APIConfig {
  baseUrl: string;
  apiKey?: string;
  accessToken?: string;
  timeout?: number;
  retries?: number;
}

export interface APIResponse<T> {
  data: T;
  status: number;
  message?: string;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    limit: number;
    totalPages: number;
    totalResults: number;
  };
  links: {
    first: string;
    next?: string;
    prev?: string;
    last: string;
  };
}

export interface ErrorResponse {
  error: {
    code: string;
    message: string;
    field?: string;
    details?: string;
    requestId: string;
  };
}
