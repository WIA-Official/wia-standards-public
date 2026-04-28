/**
 * WIA-COMP-016: Software License - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * SPDX license identifier
 */
export type SPDXIdentifier =
  | 'MIT'
  | 'Apache-2.0'
  | 'BSD-2-Clause'
  | 'BSD-3-Clause'
  | 'ISC'
  | 'GPL-2.0-only'
  | 'GPL-2.0-or-later'
  | 'GPL-3.0-only'
  | 'GPL-3.0-or-later'
  | 'LGPL-2.1-only'
  | 'LGPL-2.1-or-later'
  | 'LGPL-3.0-only'
  | 'LGPL-3.0-or-later'
  | 'AGPL-3.0-only'
  | 'AGPL-3.0-or-later'
  | 'MPL-2.0'
  | 'EPL-2.0'
  | 'CC0-1.0'
  | 'Unlicense'
  | 'CC-BY-4.0'
  | 'CC-BY-SA-4.0'
  | 'CC-BY-NC-4.0'
  | 'Proprietary'
  | string;

/**
 * License category
 */
export type LicenseCategory =
  | 'permissive'
  | 'weak-copyleft'
  | 'strong-copyleft'
  | 'public-domain'
  | 'proprietary'
  | 'creative-commons';

/**
 * License information
 */
export interface License {
  /** SPDX identifier */
  identifier: SPDXIdentifier;

  /** Full license name */
  name: string;

  /** License category */
  category: LicenseCategory;

  /** License text */
  text?: string;

  /** URL to license text */
  url?: string;

  /** Copyright holder */
  copyrightHolder?: string;

  /** Copyright year or range */
  copyrightYear?: string | number;

  /** OSI approved */
  osiApproved: boolean;

  /** FSF approved */
  fsfApproved: boolean;

  /** Allows commercial use */
  allowsCommercialUse: boolean;

  /** Requires attribution */
  requiresAttribution: boolean;

  /** Requires source disclosure */
  requiresSourceDisclosure: boolean;

  /** Includes patent grant */
  includesPatentGrant: boolean;

  /** Allows modifications */
  allowsModifications: boolean;

  /** Can be sublicensed */
  canSublicense: boolean;
}

// ============================================================================
// License Validation
// ============================================================================

/**
 * License validation parameters
 */
export interface LicenseValidation {
  /** License identifier to validate */
  identifier: SPDXIdentifier;

  /** Copyright holder (optional) */
  copyrightHolder?: string;

  /** Copyright year (optional) */
  copyrightYear?: string | number;

  /** Project name (optional) */
  projectName?: string;
}

/**
 * License validation result
 */
export interface LicenseValidationResult {
  /** Is valid? */
  isValid: boolean;

  /** License information */
  license?: License;

  /** Validation errors */
  errors: string[];

  /** Validation warnings */
  warnings: string[];

  /** Suggestions */
  suggestions: string[];
}

// ============================================================================
// License Compatibility
// ============================================================================

/**
 * Dependency license information
 */
export interface DependencyLicense {
  /** Dependency name */
  name: string;

  /** Dependency version */
  version?: string;

  /** License identifier */
  license: SPDXIdentifier;

  /** License expression (e.g., "MIT OR Apache-2.0") */
  licenseExpression?: string;

  /** Is direct dependency */
  direct: boolean;

  /** Parent dependency (if transitive) */
  parent?: string;
}

/**
 * License compatibility check parameters
 */
export interface LicenseCompatibility {
  /** Main project license */
  mainLicense: SPDXIdentifier;

  /** Dependency licenses */
  dependencies: DependencyLicense[];

  /** Check transitive dependencies */
  checkTransitive?: boolean;
}

/**
 * License compatibility result
 */
export interface LicenseCompatibilityResult {
  /** Are all licenses compatible? */
  compatible: boolean;

  /** Compatibility score (0-100) */
  score: number;

  /** Conflicts found */
  conflicts: LicenseConflict[];

  /** Warnings */
  warnings: string[];

  /** Suggestions */
  suggestions: string[];

  /** Compatible dependencies */
  compatibleDeps: DependencyLicense[];

  /** Incompatible dependencies */
  incompatibleDeps: DependencyLicense[];
}

/**
 * License conflict
 */
export interface LicenseConflict {
  /** Dependency causing conflict */
  dependency: DependencyLicense;

  /** Reason for conflict */
  reason: string;

  /** Severity */
  severity: 'error' | 'warning' | 'info';

  /** Resolution suggestions */
  resolutions: string[];
}

// ============================================================================
// SPDX Document
// ============================================================================

/**
 * SPDX document
 */
export interface SPDXDocument {
  /** SPDX version */
  spdxVersion: string;

  /** Data license (usually CC0-1.0) */
  dataLicense: string;

  /** SPDX identifier */
  SPDXID: string;

  /** Document name */
  name: string;

  /** Document namespace */
  documentNamespace: string;

  /** Creation info */
  creationInfo: SPDXCreationInfo;

  /** Packages */
  packages: SPDXPackage[];

  /** Files (optional) */
  files?: SPDXFile[];

  /** Relationships */
  relationships: SPDXRelationship[];
}

/**
 * SPDX creation info
 */
export interface SPDXCreationInfo {
  /** Creation timestamp */
  created: string;

  /** Creators */
  creators: string[];

  /** License list version */
  licenseListVersion?: string;

  /** Comment */
  comment?: string;
}

/**
 * SPDX package
 */
export interface SPDXPackage {
  /** SPDX identifier */
  SPDXID: string;

  /** Package name */
  name: string;

  /** Version */
  versionInfo?: string;

  /** Download location */
  downloadLocation: string;

  /** Files analyzed? */
  filesAnalyzed: boolean;

  /** Concluded license */
  licenseConcluded: SPDXIdentifier | string;

  /** Declared license */
  licenseDeclared: SPDXIdentifier | string;

  /** Copyright text */
  copyrightText: string;

  /** Supplier */
  supplier?: string;

  /** Originator */
  originator?: string;

  /** Homepage */
  homepage?: string;

  /** Summary */
  summary?: string;

  /** Description */
  description?: string;

  /** Checksums */
  checksums?: SPDXChecksum[];
}

/**
 * SPDX file
 */
export interface SPDXFile {
  /** SPDX identifier */
  SPDXID: string;

  /** File name */
  fileName: string;

  /** File types */
  fileTypes?: string[];

  /** Checksums */
  checksums: SPDXChecksum[];

  /** Concluded license */
  licenseConcluded: SPDXIdentifier | string;

  /** Copyright text */
  copyrightText: string;
}

/**
 * SPDX checksum
 */
export interface SPDXChecksum {
  /** Algorithm */
  algorithm: 'SHA1' | 'SHA256' | 'MD5';

  /** Checksum value */
  checksumValue: string;
}

/**
 * SPDX relationship
 */
export interface SPDXRelationship {
  /** Source SPDX ID */
  spdxElementId: string;

  /** Relationship type */
  relationshipType: SPDXRelationshipType;

  /** Related SPDX ID */
  relatedSpdxElement: string;

  /** Comment */
  comment?: string;
}

/**
 * SPDX relationship types
 */
export type SPDXRelationshipType =
  | 'DESCRIBES'
  | 'DESCRIBED_BY'
  | 'CONTAINS'
  | 'CONTAINED_BY'
  | 'DEPENDS_ON'
  | 'DEPENDENCY_OF'
  | 'BUILD_DEPENDENCY_OF'
  | 'RUNTIME_DEPENDENCY_OF'
  | 'GENERATED_FROM'
  | 'GENERATES';

// ============================================================================
// Dependency Scanning
// ============================================================================

/**
 * Dependency scan parameters
 */
export interface DependencyScan {
  /** Package manager */
  packageManager: 'npm' | 'yarn' | 'pnpm' | 'pip' | 'maven' | 'gradle' | 'go' | 'cargo';

  /** Project path */
  projectPath: string;

  /** Include dev dependencies */
  includeDev?: boolean;

  /** Scan transitive dependencies */
  scanTransitive?: boolean;

  /** Maximum depth */
  maxDepth?: number;
}

/**
 * Dependency scan result
 */
export interface DependencyScanResult {
  /** Total dependencies found */
  totalDependencies: number;

  /** Direct dependencies */
  directDependencies: number;

  /** Transitive dependencies */
  transitiveDependencies: number;

  /** All dependencies */
  dependencies: DependencyLicense[];

  /** Unique licenses found */
  uniqueLicenses: SPDXIdentifier[];

  /** License distribution */
  licenseDistribution: Record<SPDXIdentifier, number>;

  /** Scan timestamp */
  timestamp: Date;

  /** Scan duration (ms) */
  duration: number;

  /** Errors encountered */
  errors: string[];
}

// ============================================================================
// License Templates
// ============================================================================

/**
 * License template parameters
 */
export interface LicenseTemplate {
  /** Template identifier */
  template: SPDXIdentifier;

  /** Copyright holder */
  copyrightHolder: string;

  /** Copyright year */
  copyrightYear: string | number;

  /** Project name (for some licenses) */
  projectName?: string;

  /** Project description (for GPL) */
  projectDescription?: string;

  /** Additional parameters */
  params?: Record<string, string>;
}

/**
 * License template result
 */
export interface LicenseTemplateResult {
  /** License identifier */
  identifier: SPDXIdentifier;

  /** License name */
  name: string;

  /** Generated license text */
  text: string;

  /** Header comment (for source files) */
  header: string;

  /** Copyright notice */
  copyrightNotice: string;

  /** SPDX identifier line */
  spdxIdentifier: string;
}

// ============================================================================
// Compliance and Auditing
// ============================================================================

/**
 * Compliance check parameters
 */
export interface ComplianceCheck {
  /** Project license */
  projectLicense: SPDXIdentifier;

  /** Dependencies */
  dependencies: DependencyLicense[];

  /** Required attributions */
  attributions?: Attribution[];

  /** Check source disclosure */
  checkSourceDisclosure?: boolean;
}

/**
 * Compliance check result
 */
export interface ComplianceCheckResult {
  /** Is compliant? */
  compliant: boolean;

  /** Compliance score (0-100) */
  score: number;

  /** Compliance issues */
  issues: ComplianceIssue[];

  /** Required actions */
  requiredActions: string[];

  /** Recommendations */
  recommendations: string[];

  /** Attribution list */
  attributions: Attribution[];

  /** Source disclosure required? */
  requiresSourceDisclosure: boolean;
}

/**
 * Compliance issue
 */
export interface ComplianceIssue {
  /** Issue type */
  type: 'missing-license' | 'incompatible-license' | 'missing-attribution' | 'missing-source' | 'other';

  /** Severity */
  severity: 'critical' | 'high' | 'medium' | 'low';

  /** Description */
  description: string;

  /** Affected component */
  component?: string;

  /** Resolution steps */
  resolution: string[];
}

/**
 * Attribution
 */
export interface Attribution {
  /** Component name */
  component: string;

  /** Version */
  version?: string;

  /** License */
  license: SPDXIdentifier;

  /** Copyright notice */
  copyright: string;

  /** License text URL */
  licenseUrl?: string;

  /** Homepage */
  homepage?: string;
}

// ============================================================================
// Commercial Licensing
// ============================================================================

/**
 * Dual licensing configuration
 */
export interface DualLicense {
  /** Open source license */
  openSourceLicense: SPDXIdentifier;

  /** Commercial license terms */
  commercialLicense: CommercialLicense;

  /** Default license for distribution */
  defaultLicense: 'open-source' | 'commercial';
}

/**
 * Commercial license
 */
export interface CommercialLicense {
  /** License name */
  name: string;

  /** License type */
  type: 'perpetual' | 'subscription' | 'trial';

  /** Pricing tier */
  tier?: 'community' | 'professional' | 'enterprise';

  /** Price */
  price?: number;

  /** Currency */
  currency?: string;

  /** Licensed users */
  users?: number;

  /** Licensed devices */
  devices?: number;

  /** License duration (days) */
  duration?: number;

  /** Support level */
  support?: 'none' | 'community' | 'email' | 'phone' | 'priority';

  /** Features included */
  features?: string[];

  /** Restrictions */
  restrictions?: string[];
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * License compatibility matrix
 */
export type CompatibilityMatrix = Record<SPDXIdentifier, Record<SPDXIdentifier, boolean>>;

/**
 * Result type for operations
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

// ============================================================================
// Error Types
// ============================================================================

/**
 * License error codes
 */
export enum LicenseErrorCode {
  INVALID_LICENSE = 'L001',
  INCOMPATIBLE_LICENSE = 'L002',
  MISSING_LICENSE = 'L003',
  MISSING_COPYRIGHT = 'L004',
  INVALID_SPDX = 'L005',
  DEPENDENCY_SCAN_FAILED = 'L006',
  COMPLIANCE_VIOLATION = 'L007',
  TEMPLATE_ERROR = 'L008',
  UNKNOWN_LICENSE = 'L009',
}

/**
 * License error
 */
export class LicenseError extends Error {
  constructor(
    public code: LicenseErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'LicenseError';
  }
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Well-known permissive licenses
 */
export const PERMISSIVE_LICENSES: SPDXIdentifier[] = [
  'MIT',
  'Apache-2.0',
  'BSD-2-Clause',
  'BSD-3-Clause',
  'ISC',
];

/**
 * Well-known weak copyleft licenses
 */
export const WEAK_COPYLEFT_LICENSES: SPDXIdentifier[] = [
  'LGPL-2.1-only',
  'LGPL-2.1-or-later',
  'LGPL-3.0-only',
  'LGPL-3.0-or-later',
  'MPL-2.0',
  'EPL-2.0',
];

/**
 * Well-known strong copyleft licenses
 */
export const STRONG_COPYLEFT_LICENSES: SPDXIdentifier[] = [
  'GPL-2.0-only',
  'GPL-2.0-or-later',
  'GPL-3.0-only',
  'GPL-3.0-or-later',
  'AGPL-3.0-only',
  'AGPL-3.0-or-later',
];

/**
 * Public domain licenses
 */
export const PUBLIC_DOMAIN_LICENSES: SPDXIdentifier[] = ['CC0-1.0', 'Unlicense'];

// ============================================================================
// Export All
// ============================================================================

export type {
  SPDXIdentifier,
  LicenseCategory,
  License,
  LicenseValidation,
  LicenseValidationResult,
  DependencyLicense,
  LicenseCompatibility,
  LicenseCompatibilityResult,
  LicenseConflict,
  SPDXDocument,
  SPDXCreationInfo,
  SPDXPackage,
  SPDXFile,
  SPDXChecksum,
  SPDXRelationship,
  SPDXRelationshipType,
  DependencyScan,
  DependencyScanResult,
  LicenseTemplate,
  LicenseTemplateResult,
  ComplianceCheck,
  ComplianceCheckResult,
  ComplianceIssue,
  Attribution,
  DualLicense,
  CommercialLicense,
  CompatibilityMatrix,
};

export {
  LicenseErrorCode,
  LicenseError,
  PERMISSIVE_LICENSES,
  WEAK_COPYLEFT_LICENSES,
  STRONG_COPYLEFT_LICENSES,
  PUBLIC_DOMAIN_LICENSES,
};
