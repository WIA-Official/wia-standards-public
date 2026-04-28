/**
 * WIA-COMP-016: Software License SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides tools for software licensing including:
 * - License validation and verification
 * - License compatibility checking
 * - SPDX document generation
 * - Dependency scanning
 * - Compliance auditing
 */

import {
  License,
  LicenseValidation,
  LicenseValidationResult,
  LicenseCompatibility,
  LicenseCompatibilityResult,
  LicenseConflict,
  DependencyLicense,
  SPDXIdentifier,
  SPDXDocument,
  SPDXPackage,
  SPDXRelationship,
  DependencyScan,
  DependencyScanResult,
  LicenseTemplate,
  LicenseTemplateResult,
  ComplianceCheck,
  ComplianceCheckResult,
  ComplianceIssue,
  Attribution,
  LicenseError,
  LicenseErrorCode,
  PERMISSIVE_LICENSES,
  WEAK_COPYLEFT_LICENSES,
  STRONG_COPYLEFT_LICENSES,
  PUBLIC_DOMAIN_LICENSES,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMP-016 Software License SDK
 */
export class LicenseSDK {
  private version = '1.0.0';
  private licenseDatabase: Map<SPDXIdentifier, License> = new Map();

  constructor() {
    this.initializeLicenseDatabase();
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Validate a license
   */
  validateLicense(validation: LicenseValidation): LicenseValidationResult {
    const { identifier, copyrightHolder, copyrightYear } = validation;
    const errors: string[] = [];
    const warnings: string[] = [];
    const suggestions: string[] = [];

    // Check if license exists
    const license = this.licenseDatabase.get(identifier);
    if (!license) {
      // Check if it's a valid SPDX expression
      if (this.isValidSPDXExpression(identifier)) {
        warnings.push(`License expression "${identifier}" appears valid but should be verified`);
      } else {
        errors.push(`Unknown license identifier: ${identifier}`);
        suggestions.push('Use a valid SPDX identifier (e.g., MIT, Apache-2.0, GPL-3.0-only)');
      }
    }

    // Check copyright holder
    if (license?.requiresAttribution && !copyrightHolder) {
      warnings.push('Copyright holder should be specified for proper attribution');
    }

    // Check copyright year
    if (copyrightHolder && !copyrightYear) {
      warnings.push('Copyright year should be specified');
    }

    // Validate year format
    if (copyrightYear) {
      const yearStr = copyrightYear.toString();
      if (!/^\d{4}(-\d{4})?$/.test(yearStr)) {
        warnings.push('Copyright year should be in format: YYYY or YYYY-YYYY');
      }
    }

    const isValid = errors.length === 0;

    return {
      isValid,
      license,
      errors,
      warnings,
      suggestions,
    };
  }

  /**
   * Check license compatibility
   */
  checkLicenseCompatibility(
    compatibility: LicenseCompatibility
  ): LicenseCompatibilityResult {
    const { mainLicense, dependencies, checkTransitive = true } = compatibility;

    const conflicts: LicenseConflict[] = [];
    const warnings: string[] = [];
    const suggestions: string[] = [];
    const compatibleDeps: DependencyLicense[] = [];
    const incompatibleDeps: DependencyLicense[] = [];

    // Get main license info
    const mainLicenseInfo = this.licenseDatabase.get(mainLicense);
    if (!mainLicenseInfo) {
      warnings.push(`Unknown main license: ${mainLicense}`);
    }

    // Check each dependency
    for (const dep of dependencies) {
      if (!checkTransitive && !dep.direct) {
        continue;
      }

      const isCompatible = this.isLicenseCompatible(mainLicense, dep.license);

      if (isCompatible) {
        compatibleDeps.push(dep);
      } else {
        incompatibleDeps.push(dep);

        const conflict: LicenseConflict = {
          dependency: dep,
          reason: this.getIncompatibilityReason(mainLicense, dep.license),
          severity: this.getConflictSeverity(mainLicense, dep.license),
          resolutions: this.getConflictResolutions(mainLicense, dep.license, dep.name),
        };

        conflicts.push(conflict);
      }
    }

    // Calculate compatibility score
    const totalDeps = compatibleDeps.length + incompatibleDeps.length;
    const score = totalDeps > 0 ? (compatibleDeps.length / totalDeps) * 100 : 100;

    // Add suggestions
    if (incompatibleDeps.length > 0) {
      suggestions.push(`Consider changing main license to a more permissive one`);
      suggestions.push(`Or replace incompatible dependencies with compatible alternatives`);
    }

    return {
      compatible: conflicts.length === 0,
      score: Math.round(score),
      conflicts,
      warnings,
      suggestions,
      compatibleDeps,
      incompatibleDeps,
    };
  }

  /**
   * Generate SPDX document
   */
  generateSPDX(params: {
    projectName: string;
    projectVersion: string;
    projectLicense: SPDXIdentifier;
    dependencies: DependencyLicense[];
    copyrightHolder: string;
    namespace?: string;
  }): SPDXDocument {
    const {
      projectName,
      projectVersion,
      projectLicense,
      dependencies,
      copyrightHolder,
      namespace = `https://wiastandards.com/spdx/${projectName}-${projectVersion}`,
    } = params;

    const timestamp = new Date().toISOString();

    // Create main package
    const mainPackage: SPDXPackage = {
      SPDXID: 'SPDXRef-Package',
      name: projectName,
      versionInfo: projectVersion,
      downloadLocation: 'NOASSERTION',
      filesAnalyzed: false,
      licenseConcluded: projectLicense,
      licenseDeclared: projectLicense,
      copyrightText: `Copyright ${new Date().getFullYear()} ${copyrightHolder}`,
    };

    // Create dependency packages
    const depPackages: SPDXPackage[] = dependencies.map((dep, index) => ({
      SPDXID: `SPDXRef-Package-${index + 1}`,
      name: dep.name,
      versionInfo: dep.version,
      downloadLocation: 'NOASSERTION',
      filesAnalyzed: false,
      licenseConcluded: dep.license,
      licenseDeclared: dep.license,
      copyrightText: 'NOASSERTION',
    }));

    // Create relationships
    const relationships: SPDXRelationship[] = [
      {
        spdxElementId: 'SPDXRef-DOCUMENT',
        relationshipType: 'DESCRIBES',
        relatedSpdxElement: 'SPDXRef-Package',
      },
    ];

    // Add dependency relationships
    depPackages.forEach((pkg, index) => {
      const dep = dependencies[index];
      relationships.push({
        spdxElementId: 'SPDXRef-Package',
        relationshipType: dep.direct ? 'DEPENDS_ON' : 'RUNTIME_DEPENDENCY_OF',
        relatedSpdxElement: pkg.SPDXID,
      });
    });

    return {
      spdxVersion: 'SPDX-2.3',
      dataLicense: 'CC0-1.0',
      SPDXID: 'SPDXRef-DOCUMENT',
      name: projectName,
      documentNamespace: namespace,
      creationInfo: {
        created: timestamp,
        creators: [`Tool: WIA-COMP-016-${this.version}`],
        licenseListVersion: '3.21',
      },
      packages: [mainPackage, ...depPackages],
      relationships,
    };
  }

  /**
   * Scan dependencies for licenses
   */
  scanDependencies(scan: DependencyScan): DependencyScanResult {
    const startTime = Date.now();
    const { packageManager, projectPath, includeDev = false, maxDepth = 10 } = scan;

    // This is a simplified implementation
    // In production, this would actually parse package.json, requirements.txt, etc.
    const dependencies: DependencyLicense[] = [];
    const errors: string[] = [];

    try {
      // Simulate scanning (in real implementation, would read and parse files)
      // For now, return empty result
      console.log(`Scanning ${packageManager} dependencies in ${projectPath}...`);
    } catch (error) {
      errors.push(`Failed to scan dependencies: ${error}`);
    }

    const duration = Date.now() - startTime;

    // Calculate statistics
    const directDeps = dependencies.filter((d) => d.direct).length;
    const transitiveDeps = dependencies.length - directDeps;

    const uniqueLicenses = Array.from(
      new Set(dependencies.map((d) => d.license))
    ) as SPDXIdentifier[];

    const licenseDistribution: Record<SPDXIdentifier, number> = {};
    for (const dep of dependencies) {
      licenseDistribution[dep.license] = (licenseDistribution[dep.license] || 0) + 1;
    }

    return {
      totalDependencies: dependencies.length,
      directDependencies: directDeps,
      transitiveDependencies: transitiveDeps,
      dependencies,
      uniqueLicenses,
      licenseDistribution,
      timestamp: new Date(),
      duration,
      errors,
    };
  }

  /**
   * Create license from template
   */
  createLicenseFromTemplate(template: LicenseTemplate): LicenseTemplateResult {
    const { template: templateId, copyrightHolder, copyrightYear, projectName } = template;

    const license = this.licenseDatabase.get(templateId);
    if (!license) {
      throw new LicenseError(
        LicenseErrorCode.TEMPLATE_ERROR,
        `Unknown license template: ${templateId}`
      );
    }

    const year = copyrightYear.toString();
    const copyrightNotice = `Copyright (c) ${year} ${copyrightHolder}`;

    // Generate license text
    const text = this.generateLicenseText(templateId, copyrightHolder, year, projectName);

    // Generate header comment
    const header = this.generateLicenseHeader(templateId, copyrightHolder, year, projectName);

    // SPDX identifier line
    const spdxIdentifier = `SPDX-License-Identifier: ${templateId}`;

    return {
      identifier: templateId,
      name: license.name,
      text,
      header,
      copyrightNotice,
      spdxIdentifier,
    };
  }

  /**
   * Check compliance
   */
  checkCompliance(check: ComplianceCheck): ComplianceCheckResult {
    const { projectLicense, dependencies } = check;

    const issues: ComplianceIssue[] = [];
    const requiredActions: string[] = [];
    const recommendations: string[] = [];
    const attributions: Attribution[] = [];

    // Check license compatibility
    const compatibility = this.checkLicenseCompatibility({
      mainLicense: projectLicense,
      dependencies,
    });

    // Add compatibility issues
    for (const conflict of compatibility.conflicts) {
      issues.push({
        type: 'incompatible-license',
        severity: conflict.severity === 'error' ? 'critical' : 'high',
        description: `Incompatible license: ${conflict.dependency.name} (${conflict.dependency.license})`,
        component: conflict.dependency.name,
        resolution: conflict.resolutions,
      });
    }

    // Check for missing licenses
    for (const dep of dependencies) {
      if (!dep.license || dep.license === 'UNKNOWN') {
        issues.push({
          type: 'missing-license',
          severity: 'high',
          description: `Missing license information for ${dep.name}`,
          component: dep.name,
          resolution: ['Contact package maintainer', 'Check package repository'],
        });
      }
    }

    // Generate attributions
    for (const dep of dependencies) {
      const depLicense = this.licenseDatabase.get(dep.license);
      if (depLicense?.requiresAttribution) {
        attributions.push({
          component: dep.name,
          version: dep.version,
          license: dep.license,
          copyright: `Copyright holders of ${dep.name}`,
        });
      }
    }

    // Check source disclosure requirement
    const projectLicenseInfo = this.licenseDatabase.get(projectLicense);
    const requiresSourceDisclosure = projectLicenseInfo?.requiresSourceDisclosure || false;

    if (requiresSourceDisclosure) {
      requiredActions.push('Prepare source code distribution or written offer');
      requiredActions.push('Include installation instructions');
    }

    // Add required actions for attributions
    if (attributions.length > 0) {
      requiredActions.push('Include license notices for all dependencies');
      requiredActions.push('Create NOTICE or ATTRIBUTION file');
    }

    // Calculate compliance score
    const totalChecks = dependencies.length + 2; // deps + license + attribution
    const passedChecks = totalChecks - issues.length;
    const score = Math.round((passedChecks / totalChecks) * 100);

    return {
      compliant: issues.length === 0,
      score: Math.max(0, score),
      issues,
      requiredActions,
      recommendations,
      attributions,
      requiresSourceDisclosure,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Check if two licenses are compatible
   */
  private isLicenseCompatible(mainLicense: SPDXIdentifier, depLicense: SPDXIdentifier): boolean {
    // Permissive licenses are compatible with everything
    if (PERMISSIVE_LICENSES.includes(depLicense)) {
      return true;
    }

    // Public domain is compatible with everything
    if (PUBLIC_DOMAIN_LICENSES.includes(depLicense)) {
      return true;
    }

    // Strong copyleft (GPL/AGPL) requires same license
    if (STRONG_COPYLEFT_LICENSES.includes(depLicense)) {
      return mainLicense === depLicense || mainLicense.startsWith('GPL') || mainLicense.startsWith('AGPL');
    }

    // Weak copyleft (LGPL/MPL) can link with anything
    if (WEAK_COPYLEFT_LICENSES.includes(depLicense)) {
      return true;
    }

    // Proprietary can only use permissive and weak copyleft
    if (mainLicense === 'Proprietary') {
      return !STRONG_COPYLEFT_LICENSES.includes(depLicense);
    }

    // Default to compatible
    return true;
  }

  /**
   * Get incompatibility reason
   */
  private getIncompatibilityReason(mainLicense: SPDXIdentifier, depLicense: SPDXIdentifier): string {
    if (STRONG_COPYLEFT_LICENSES.includes(depLicense)) {
      return `${depLicense} is a strong copyleft license that requires the entire work to be licensed under ${depLicense}`;
    }
    return `Licenses ${mainLicense} and ${depLicense} are not compatible`;
  }

  /**
   * Get conflict severity
   */
  private getConflictSeverity(mainLicense: SPDXIdentifier, depLicense: SPDXIdentifier): 'error' | 'warning' | 'info' {
    if (STRONG_COPYLEFT_LICENSES.includes(depLicense)) {
      return 'error';
    }
    return 'warning';
  }

  /**
   * Get conflict resolutions
   */
  private getConflictResolutions(
    mainLicense: SPDXIdentifier,
    depLicense: SPDXIdentifier,
    depName: string
  ): string[] {
    const resolutions: string[] = [];

    if (STRONG_COPYLEFT_LICENSES.includes(depLicense)) {
      resolutions.push(`Change main license to ${depLicense}`);
      resolutions.push(`Replace ${depName} with a permissive-licensed alternative`);
      resolutions.push(`Contact ${depName} maintainers for dual-licensing options`);
    }

    return resolutions;
  }

  /**
   * Check if string is valid SPDX expression
   */
  private isValidSPDXExpression(expression: string): boolean {
    // Simple check for SPDX operators
    return /\b(AND|OR|WITH)\b/.test(expression);
  }

  /**
   * Generate license text from template
   */
  private generateLicenseText(
    templateId: SPDXIdentifier,
    copyrightHolder: string,
    year: string,
    projectName?: string
  ): string {
    switch (templateId) {
      case 'MIT':
        return this.getMITLicenseText(copyrightHolder, year);
      case 'Apache-2.0':
        return this.getApacheLicenseText(copyrightHolder, year);
      case 'GPL-3.0-only':
      case 'GPL-3.0-or-later':
        return this.getGPLLicenseText(copyrightHolder, year, projectName || 'This Program');
      default:
        return `License: ${templateId}\nCopyright (c) ${year} ${copyrightHolder}\n\nSee https://spdx.org/licenses/${templateId}.html for full license text.`;
    }
  }

  /**
   * Generate license header
   */
  private generateLicenseHeader(
    templateId: SPDXIdentifier,
    copyrightHolder: string,
    year: string,
    projectName?: string
  ): string {
    const spdx = `SPDX-License-Identifier: ${templateId}`;
    const copyright = `Copyright (c) ${year} ${copyrightHolder}`;

    return `/**
 * ${spdx}
 * ${copyright}
 */`;
  }

  /**
   * Get MIT license text
   */
  private getMITLicenseText(holder: string, year: string): string {
    return `MIT License

Copyright (c) ${year} ${holder}

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.`;
  }

  /**
   * Get Apache 2.0 license text (header only)
   */
  private getApacheLicenseText(holder: string, year: string): string {
    return `Copyright ${year} ${holder}

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.`;
  }

  /**
   * Get GPL 3.0 license text (header only)
   */
  private getGPLLicenseText(holder: string, year: string, programName: string): string {
    return `${programName}
Copyright (C) ${year}  ${holder}

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.`;
  }

  /**
   * Initialize license database
   */
  private initializeLicenseDatabase(): void {
    // MIT
    this.licenseDatabase.set('MIT', {
      identifier: 'MIT',
      name: 'MIT License',
      category: 'permissive',
      url: 'https://opensource.org/licenses/MIT',
      osiApproved: true,
      fsfApproved: true,
      allowsCommercialUse: true,
      requiresAttribution: true,
      requiresSourceDisclosure: false,
      includesPatentGrant: false,
      allowsModifications: true,
      canSublicense: true,
    });

    // Apache-2.0
    this.licenseDatabase.set('Apache-2.0', {
      identifier: 'Apache-2.0',
      name: 'Apache License 2.0',
      category: 'permissive',
      url: 'https://www.apache.org/licenses/LICENSE-2.0',
      osiApproved: true,
      fsfApproved: true,
      allowsCommercialUse: true,
      requiresAttribution: true,
      requiresSourceDisclosure: false,
      includesPatentGrant: true,
      allowsModifications: true,
      canSublicense: true,
    });

    // GPL-3.0
    this.licenseDatabase.set('GPL-3.0-only', {
      identifier: 'GPL-3.0-only',
      name: 'GNU General Public License v3.0 only',
      category: 'strong-copyleft',
      url: 'https://www.gnu.org/licenses/gpl-3.0.html',
      osiApproved: true,
      fsfApproved: true,
      allowsCommercialUse: true,
      requiresAttribution: true,
      requiresSourceDisclosure: true,
      includesPatentGrant: true,
      allowsModifications: true,
      canSublicense: false,
    });

    // Add more licenses as needed...
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Validate license (standalone)
 */
export function validateLicense(validation: LicenseValidation): LicenseValidationResult {
  const sdk = new LicenseSDK();
  return sdk.validateLicense(validation);
}

/**
 * Check license compatibility (standalone)
 */
export function checkLicenseCompatibility(
  compatibility: LicenseCompatibility
): LicenseCompatibilityResult {
  const sdk = new LicenseSDK();
  return sdk.checkLicenseCompatibility(compatibility);
}

/**
 * Generate SPDX document (standalone)
 */
export function generateSPDX(params: {
  projectName: string;
  projectVersion: string;
  projectLicense: SPDXIdentifier;
  dependencies: DependencyLicense[];
  copyrightHolder: string;
}): SPDXDocument {
  const sdk = new LicenseSDK();
  return sdk.generateSPDX(params);
}

/**
 * Scan dependencies (standalone)
 */
export function scanDependencies(scan: DependencyScan): DependencyScanResult {
  const sdk = new LicenseSDK();
  return sdk.scanDependencies(scan);
}

/**
 * Create license from template (standalone)
 */
export function createLicenseFromTemplate(template: LicenseTemplate): LicenseTemplateResult {
  const sdk = new LicenseSDK();
  return sdk.createLicenseFromTemplate(template);
}

/**
 * Check compliance (standalone)
 */
export function checkCompliance(check: ComplianceCheck): ComplianceCheckResult {
  const sdk = new LicenseSDK();
  return sdk.checkCompliance(check);
}

// ============================================================================
// Export All
// ============================================================================

export * from './types';
export { LicenseSDK };
