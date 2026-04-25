/**
 * WIA-AI-020: AI Governance Standard - TypeScript SDK
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 */

export * from './types';

import {
  AISystem,
  RiskAssessment,
  RiskLevel,
  RiskCategory,
  ImpactAssessment,
  ComplianceCheck,
  ComplianceStatus,
  GovernanceFramework,
  ValidationResult,
  FairnessMetric,
  BiasTestingResult
} from './types';

// ============================================================================
// Risk Assessment Engine
// ============================================================================

export class AIRiskAssessmentEngine {
  /**
   * Calculate risk priority number (RPN)
   * RPN = Likelihood × Impact × (6 - Detectability)
   */
  calculateRiskScore(likelihood: number, impact: number, detectability: number): number {
    if (likelihood < 1 || likelihood > 5 || impact < 1 || impact > 5 || detectability < 1 || detectability > 5) {
      throw new Error('Likelihood, Impact, and Detectability must be between 1 and 5');
    }
    return likelihood * impact * (6 - detectability);
  }

  /**
   * Determine risk level based on risk score
   */
  determineRiskLevel(riskScore: number): RiskLevel {
    if (riskScore >= 60) return RiskLevel.CRITICAL;
    if (riskScore >= 40) return RiskLevel.HIGH;
    if (riskScore >= 20) return RiskLevel.MEDIUM;
    return RiskLevel.LOW;
  }

  /**
   * Assess AI system across all risk categories
   */
  assessAISystem(system: AISystem): RiskAssessment[] {
    const risks: RiskAssessment[] = [];

    // Example risk assessment - in production, this would be more sophisticated
    const categories = Object.values(RiskCategory);

    for (const category of categories) {
      const likelihood = this.estimateLikelihood(system, category);
      const impact = this.estimateImpact(system, category);
      const detectability = this.estimateDetectability(system, category);

      const riskScore = this.calculateRiskScore(likelihood, impact, detectability);
      const riskLevel = this.determineRiskLevel(riskScore);

      risks.push({
        id: `${system.id}-${category}`,
        systemId: system.id,
        category,
        title: `${category} Risk`,
        description: `Potential ${category.toLowerCase()} risk for ${system.name}`,
        affectedStakeholders: [],
        likelihood,
        impact,
        detectability,
        riskScore,
        riskLevel,
        currentControls: [],
        mitigationPlan: {
          strategies: [],
          timeline: 'TBD',
          resourcesRequired: [],
          successMetrics: [],
          owner: system.owner,
          status: 'PLANNED'
        },
        residualRisk: riskScore,
        owner: system.owner,
        status: 'IDENTIFIED',
        assessmentDate: new Date()
      });
    }

    return risks;
  }

  private estimateLikelihood(system: AISystem, category: RiskCategory): number {
    // Placeholder logic - would be more sophisticated in production
    return 3;
  }

  private estimateImpact(system: AISystem, category: RiskCategory): number {
    // Higher impact for high-risk systems
    if (system.riskClassification === 'HIGH_RISK') return 4;
    if (system.riskClassification === 'LIMITED_RISK') return 2;
    return 1;
  }

  private estimateDetectability(system: AISystem, category: RiskCategory): number {
    // Placeholder logic
    return 3;
  }
}

// ============================================================================
// Fairness Evaluation Engine
// ============================================================================

export class FairnessEvaluationEngine {
  /**
   * Calculate demographic parity difference
   * DPD = |P(Ŷ=1|A=0) - P(Ŷ=1|A=1)|
   */
  calculateDemographicParity(
    positiveRateGroup0: number,
    positiveRateGroup1: number
  ): number {
    return Math.abs(positiveRateGroup0 - positiveRateGroup1);
  }

  /**
   * Calculate equal opportunity difference
   * EOD = |TPR_0 - TPR_1|
   */
  calculateEqualOpportunity(
    truePositiveRateGroup0: number,
    truePositiveRateGroup1: number
  ): number {
    return Math.abs(truePositiveRateGroup0 - truePositiveRateGroup1);
  }

  /**
   * Calculate predictive parity difference
   * PPD = |PPV_0 - PPV_1|
   */
  calculatePredictiveParity(
    positivePredictivenessGroup0: number,
    positivePredictivenessGroup1: number
  ): number {
    return Math.abs(positivePredictivenessGroup0 - positivePredictivenessGroup1);
  }

  /**
   * Evaluate fairness across multiple metrics
   */
  evaluateFairness(
    predictions: number[][],
    protectedAttributes: number[],
    threshold: number = 0.1
  ): FairnessMetric[] {
    const metrics: FairnessMetric[] = [];

    // Calculate metrics for each protected attribute group
    // This is a simplified example - production implementation would be more robust

    const demographicParity = this.calculateDemographicParity(0.5, 0.6);
    metrics.push({
      name: 'Demographic Parity',
      value: demographicParity,
      threshold,
      compliant: demographicParity < threshold,
      description: 'Difference in positive prediction rates between groups'
    });

    return metrics;
  }
}

// ============================================================================
// Compliance Assessment Engine
// ============================================================================

export class ComplianceAssessmentEngine {
  /**
   * Assess GDPR compliance
   */
  assessGDPRCompliance(system: AISystem): ComplianceCheck {
    const requirements = [
      {
        requirementId: 'GDPR-LAWFUL-BASIS',
        description: 'Lawful basis for processing documented',
        status: ComplianceStatus.NOT_APPLICABLE,
        evidence: [],
        gaps: []
      },
      {
        requirementId: 'GDPR-DATA-MIN',
        description: 'Data minimization practiced',
        status: ComplianceStatus.NOT_APPLICABLE,
        evidence: [],
        gaps: []
      },
      {
        requirementId: 'GDPR-RIGHTS',
        description: 'Individual rights implementable',
        status: ComplianceStatus.NOT_APPLICABLE,
        evidence: [],
        gaps: []
      }
    ];

    return {
      id: `${system.id}-GDPR`,
      systemId: system.id,
      regulation: 'GDPR',
      requirements,
      overallStatus: ComplianceStatus.NOT_APPLICABLE,
      findings: [],
      assessor: 'Automated Assessment',
      assessmentDate: new Date(),
      nextReviewDate: new Date(Date.now() + 365 * 24 * 60 * 60 * 1000)
    };
  }

  /**
   * Assess EU AI Act compliance
   */
  assessEUAIActCompliance(system: AISystem): ComplianceCheck {
    const requirements = [
      {
        requirementId: 'AI-ACT-RISK-MGMT',
        description: 'Risk management system implemented',
        status: ComplianceStatus.NOT_APPLICABLE,
        evidence: [],
        gaps: []
      },
      {
        requirementId: 'AI-ACT-DATA-GOV',
        description: 'Data governance standards met',
        status: ComplianceStatus.NOT_APPLICABLE,
        evidence: [],
        gaps: []
      }
    ];

    return {
      id: `${system.id}-AI-ACT`,
      systemId: system.id,
      regulation: 'EU AI Act',
      requirements,
      overallStatus: ComplianceStatus.NOT_APPLICABLE,
      findings: [],
      assessor: 'Automated Assessment',
      assessmentDate: new Date(),
      nextReviewDate: new Date(Date.now() + 365 * 24 * 60 * 60 * 1000)
    };
  }
}

// ============================================================================
// Governance Framework Manager
// ============================================================================

export class GovernanceFrameworkManager {
  /**
   * Initialize a new governance framework
   */
  initializeFramework(organizationId: string): GovernanceFramework {
    return {
      organizationId,
      phase: 'PHASE_1',
      strategicLayer: {
        aiVision: 'To develop AI systems that benefit all humanity (弘益인간)',
        governancePrinciples: [
          'Transparency',
          'Accountability',
          'Fairness',
          'Privacy',
          'Safety',
          'Human Oversight'
        ],
        executiveOversight: {
          boardOversight: true,
          chiefAIOfficer: 'TBD',
          oversightCommittee: {
            name: 'AI Oversight Committee',
            members: [],
            meetingFrequency: 'Monthly',
            responsibilities: [
              'Review AI initiatives',
              'Approve policies',
              'Monitor compliance'
            ]
          }
        },
        stakeholderStrategy: {
          stakeholders: [],
          engagementStrategy: 'TBD',
          feedbackMechanisms: []
        }
      },
      tacticalLayer: {
        policies: [],
        riskFramework: {
          methodology: 'WIA-AI-020',
          riskCategories: Object.values(RiskCategory),
          assessmentFrequency: 'Quarterly',
          escalationCriteria: ['CRITICAL risks', 'HIGH risks affecting fundamental rights']
        },
        complianceProgram: {
          applicableRegulations: [],
          complianceChecks: [],
          auditSchedule: 'Annual',
          reportingFrequency: 'Quarterly'
        },
        ethicsBoard: {
          name: 'AI Ethics Board',
          members: [],
          meetingFrequency: 'Monthly',
          responsibilities: [
            'Review high-risk systems',
            'Provide ethical guidance',
            'Evaluate impact assessments'
          ],
          charter: 'TBD',
          reviewProcess: {
            submissionCriteria: [],
            evaluationCriteria: [],
            decisionTimeline: '2 weeks',
            appealProcess: 'TBD'
          },
          decisionAuthority: ['Approve', 'Reject', 'Require Modifications']
        },
        performanceMetrics: []
      },
      operationalLayer: {
        developmentGuidelines: [],
        testingProtocols: [],
        deploymentControls: [],
        monitoringSystem: {
          metrics: [],
          alertThresholds: [],
          reportingFrequency: 'Real-time'
        },
        incidentResponse: {
          detectionMethods: [],
          classificationCriteria: [],
          responseSteps: [],
          escalationPath: []
        }
      },
      maturityScore: 0,
      lastAssessmentDate: new Date()
    };
  }

  /**
   * Calculate governance maturity score (0-100)
   */
  calculateMaturityScore(framework: GovernanceFramework): number {
    let score = 0;
    let maxScore = 0;

    // Strategic layer (30 points)
    maxScore += 30;
    if (framework.strategicLayer.aiVision) score += 5;
    if (framework.strategicLayer.governancePrinciples.length >= 5) score += 5;
    if (framework.strategicLayer.executiveOversight.boardOversight) score += 10;
    if (framework.strategicLayer.executiveOversight.chiefAIOfficer !== 'TBD') score += 10;

    // Tactical layer (40 points)
    maxScore += 40;
    if (framework.tacticalLayer.policies.length >= 5) score += 15;
    if (framework.tacticalLayer.ethicsBoard.members.length >= 5) score += 10;
    if (framework.tacticalLayer.complianceProgram.applicableRegulations.length > 0) score += 10;
    if (framework.tacticalLayer.performanceMetrics.length >= 5) score += 5;

    // Operational layer (30 points)
    maxScore += 30;
    if (framework.operationalLayer.developmentGuidelines.length > 0) score += 10;
    if (framework.operationalLayer.testingProtocols.length > 0) score += 10;
    if (framework.operationalLayer.deploymentControls.length > 0) score += 10;

    return Math.round((score / maxScore) * 100);
  }
}

// ============================================================================
// Validation Utilities
// ============================================================================

export class AIGovernanceValidator {
  /**
   * Validate AI system data
   */
  validateAISystem(system: AISystem): ValidationResult {
    const errors = [];
    const warnings = [];

    if (!system.id) errors.push({ field: 'id', message: 'ID is required', code: 'REQUIRED' });
    if (!system.name) errors.push({ field: 'name', message: 'Name is required', code: 'REQUIRED' });
    if (!system.owner) errors.push({ field: 'owner', message: 'Owner is required', code: 'REQUIRED' });

    if (!system.riskClassification) {
      warnings.push({
        field: 'riskClassification',
        message: 'Risk classification not set',
        recommendation: 'Classify system as UNACCEPTABLE, HIGH_RISK, LIMITED_RISK, or MINIMAL_RISK'
      });
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings
    };
  }

  /**
   * Validate risk assessment data
   */
  validateRiskAssessment(assessment: RiskAssessment): ValidationResult {
    const errors = [];
    const warnings = [];

    if (assessment.likelihood < 1 || assessment.likelihood > 5) {
      errors.push({ field: 'likelihood', message: 'Likelihood must be between 1 and 5', code: 'INVALID_RANGE' });
    }

    if (assessment.impact < 1 || assessment.impact > 5) {
      errors.push({ field: 'impact', message: 'Impact must be between 1 and 5', code: 'INVALID_RANGE' });
    }

    if (!assessment.mitigationPlan || assessment.mitigationPlan.strategies.length === 0) {
      warnings.push({
        field: 'mitigationPlan',
        message: 'No mitigation strategies defined',
        recommendation: 'Define at least one mitigation strategy'
      });
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings
    };
  }
}

// ============================================================================
// Main SDK Export
// ============================================================================

export const AIGovernance = {
  RiskAssessment: AIRiskAssessmentEngine,
  FairnessEvaluation: FairnessEvaluationEngine,
  ComplianceAssessment: ComplianceAssessmentEngine,
  FrameworkManager: GovernanceFrameworkManager,
  Validator: AIGovernanceValidator
};

export default AIGovernance;

/**
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK embodies the principle of 弘익人間 by providing tools to build
 * AI systems that prioritize human welfare, fairness, transparency, and
 * accountability. Use these tools to create AI that truly serves all of humanity.
 */
