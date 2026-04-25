# PHASE 2: Algorithm Specification

## WIA-PET-WELFARE-GLOBAL Assessment Algorithms

### 2.1 Five Freedoms Scoring

```python
"""
Five Freedoms Assessment Calculator
Comprehensive welfare scoring based on international standards
"""

from dataclasses import dataclass
from typing import List, Dict, Optional
from datetime import datetime
from enum import Enum

class Freedom(Enum):
    HUNGER_THIRST = "hunger_thirst"
    DISCOMFORT = "discomfort"
    PAIN_INJURY = "pain_injury"
    NORMAL_BEHAVIOR = "normal_behavior"
    FEAR_DISTRESS = "fear_distress"

@dataclass
class IndicatorResult:
    indicator_id: str
    name: str
    score: float  # 0-100
    weight: float
    evidence: str
    critical: bool

@dataclass
class FreedomScore:
    freedom: Freedom
    score: float  # 0-100
    indicators: List[IndicatorResult]
    critical_issues: bool
    notes: str

class FiveFreedomsCalculator:
    """
    Calculate welfare scores based on Five Freedoms framework
    """

    def __init__(self):
        self.indicator_weights = self._load_indicator_weights()
        self.thresholds = self._load_thresholds()

    def assess_entity(
        self,
        entity_id: str,
        observations: Dict[str, any],
        species: str
    ) -> 'FiveFreedomsAssessment':
        """
        Perform complete Five Freedoms assessment
        """
        freedom_scores = {}

        # Assess each freedom
        freedom_scores[Freedom.HUNGER_THIRST] = self._assess_hunger_thirst(
            observations, species
        )
        freedom_scores[Freedom.DISCOMFORT] = self._assess_discomfort(
            observations, species
        )
        freedom_scores[Freedom.PAIN_INJURY] = self._assess_pain_injury(
            observations, species
        )
        freedom_scores[Freedom.NORMAL_BEHAVIOR] = self._assess_normal_behavior(
            observations, species
        )
        freedom_scores[Freedom.FEAR_DISTRESS] = self._assess_fear_distress(
            observations, species
        )

        # Calculate overall score
        overall_score = self._calculate_overall_score(freedom_scores)

        # Generate findings and recommendations
        findings = self._generate_findings(freedom_scores)
        recommendations = self._generate_recommendations(freedom_scores)

        return FiveFreedomsAssessment(
            entity_id=entity_id,
            assessed_at=datetime.now(),
            freedoms=freedom_scores,
            overall_score=overall_score,
            findings=findings,
            recommendations=recommendations
        )

    def _assess_hunger_thirst(
        self,
        observations: Dict,
        species: str
    ) -> FreedomScore:
        """
        Freedom from hunger and thirst assessment
        """
        indicators = []

        # Water availability
        water_score = self._score_indicator(
            name="Water Availability",
            value=observations.get('water_access', {}),
            criteria={
                'fresh': 30,
                'accessible': 30,
                'sufficient': 20,
                'clean': 20
            }
        )
        indicators.append(water_score)

        # Food quality
        food_quality = self._score_indicator(
            name="Food Quality",
            value=observations.get('food_quality', {}),
            criteria={
                'appropriate': 25,
                'fresh': 25,
                'nutritionally_complete': 30,
                'species_appropriate': 20
            }
        )
        indicators.append(food_quality)

        # Feeding frequency
        feeding_freq = self._score_indicator(
            name="Feeding Frequency",
            value=observations.get('feeding_schedule', {}),
            criteria={
                'regular': 40,
                'appropriate_for_species': 30,
                'documented': 30
            }
        )
        indicators.append(feeding_freq)

        # Body condition
        body_condition = self._score_indicator(
            name="Body Condition",
            value=observations.get('body_condition', {}),
            criteria={
                'healthy_weight': 50,
                'no_malnutrition': 30,
                'no_dehydration': 20
            }
        )
        indicators.append(body_condition)

        # Calculate freedom score
        total_score = self._calculate_weighted_score(indicators)
        critical = any(i.critical for i in indicators)

        return FreedomScore(
            freedom=Freedom.HUNGER_THIRST,
            score=total_score,
            indicators=indicators,
            critical_issues=critical,
            notes=self._generate_notes(indicators)
        )

    def _assess_discomfort(
        self,
        observations: Dict,
        species: str
    ) -> FreedomScore:
        """
        Freedom from discomfort assessment
        """
        indicators = []

        # Housing adequacy
        housing = self._score_indicator(
            name="Housing Adequacy",
            value=observations.get('housing', {}),
            criteria={
                'sufficient_space': 25,
                'appropriate_substrate': 20,
                'shelter_available': 20,
                'clean': 20,
                'well_maintained': 15
            }
        )
        indicators.append(housing)

        # Temperature control
        temperature = self._score_indicator(
            name="Temperature Control",
            value=observations.get('environment', {}),
            criteria={
                'within_range': 40,
                'heating_available': 20,
                'cooling_available': 20,
                'monitored': 20
            }
        )
        indicators.append(temperature)

        # Ventilation
        ventilation = self._score_indicator(
            name="Ventilation",
            value=observations.get('ventilation', {}),
            criteria={
                'adequate_airflow': 40,
                'no_drafts': 30,
                'ammonia_acceptable': 30
            }
        )
        indicators.append(ventilation)

        # Lighting
        lighting = self._score_indicator(
            name="Lighting",
            value=observations.get('lighting', {}),
            criteria={
                'natural_light_access': 30,
                'appropriate_cycle': 40,
                'intensity_suitable': 30
            }
        )
        indicators.append(lighting)

        # Noise levels
        noise = self._score_indicator(
            name="Noise Levels",
            value=observations.get('noise', {}),
            criteria={
                'within_limits': 50,
                'no_constant_loud': 30,
                'quiet_periods': 20
            }
        )
        indicators.append(noise)

        total_score = self._calculate_weighted_score(indicators)
        critical = any(i.critical for i in indicators)

        return FreedomScore(
            freedom=Freedom.DISCOMFORT,
            score=total_score,
            indicators=indicators,
            critical_issues=critical,
            notes=self._generate_notes(indicators)
        )

    def _assess_pain_injury(
        self,
        observations: Dict,
        species: str
    ) -> FreedomScore:
        """
        Freedom from pain, injury, and disease assessment
        """
        indicators = []

        # Veterinary care
        vet_care = self._score_indicator(
            name="Veterinary Care",
            value=observations.get('veterinary', {}),
            criteria={
                'regular_checkups': 25,
                'emergency_access': 25,
                'vaccination_current': 25,
                'records_maintained': 25
            }
        )
        indicators.append(vet_care)

        # Disease prevention
        disease_prevention = self._score_indicator(
            name="Disease Prevention",
            value=observations.get('health_protocols', {}),
            criteria={
                'quarantine_procedures': 25,
                'parasite_control': 25,
                'hygiene_protocols': 25,
                'monitoring': 25
            }
        )
        indicators.append(disease_prevention)

        # Physical condition
        physical = self._score_indicator(
            name="Physical Condition",
            value=observations.get('physical_exam', {}),
            criteria={
                'no_injuries': 30,
                'healthy_coat': 20,
                'normal_mobility': 25,
                'no_pain_signs': 25
            }
        )
        indicators.append(physical)

        # Injury prevention
        injury_prevention = self._score_indicator(
            name="Injury Prevention",
            value=observations.get('safety', {}),
            criteria={
                'safe_enclosures': 30,
                'no_hazards': 30,
                'appropriate_grouping': 20,
                'supervision': 20
            }
        )
        indicators.append(injury_prevention)

        total_score = self._calculate_weighted_score(indicators)
        critical = any(i.critical for i in indicators)

        return FreedomScore(
            freedom=Freedom.PAIN_INJURY,
            score=total_score,
            indicators=indicators,
            critical_issues=critical,
            notes=self._generate_notes(indicators)
        )

    def _assess_normal_behavior(
        self,
        observations: Dict,
        species: str
    ) -> FreedomScore:
        """
        Freedom to express normal behavior assessment
        """
        indicators = []

        # Space for movement
        space = self._score_indicator(
            name="Space for Movement",
            value=observations.get('space', {}),
            criteria={
                'meets_minimum': 30,
                'allows_exercise': 30,
                'outdoor_access': 20,
                'vertical_space': 20
            }
        )
        indicators.append(space)

        # Environmental enrichment
        enrichment = self._score_indicator(
            name="Environmental Enrichment",
            value=observations.get('enrichment', {}),
            criteria={
                'toys_provided': 20,
                'variety': 20,
                'species_appropriate': 30,
                'regularly_changed': 30
            }
        )
        indicators.append(enrichment)

        # Social interaction
        social = self._score_indicator(
            name="Social Interaction",
            value=observations.get('social', {}),
            criteria={
                'conspecific_contact': 30,
                'human_interaction': 25,
                'appropriate_grouping': 25,
                'choice_available': 20
            }
        )
        indicators.append(social)

        # Natural behaviors
        natural = self._score_indicator(
            name="Natural Behaviors",
            value=observations.get('behaviors', {}),
            criteria={
                'can_perform_natural': 40,
                'not_restricted': 30,
                'opportunities_provided': 30
            }
        )
        indicators.append(natural)

        total_score = self._calculate_weighted_score(indicators)
        critical = any(i.critical for i in indicators)

        return FreedomScore(
            freedom=Freedom.NORMAL_BEHAVIOR,
            score=total_score,
            indicators=indicators,
            critical_issues=critical,
            notes=self._generate_notes(indicators)
        )

    def _assess_fear_distress(
        self,
        observations: Dict,
        species: str
    ) -> FreedomScore:
        """
        Freedom from fear and distress assessment
        """
        indicators = []

        # Stress indicators
        stress = self._score_indicator(
            name="Stress Indicators",
            value=observations.get('stress_signs', {}),
            criteria={
                'no_stereotypies': 30,
                'normal_behavior': 30,
                'no_self_harm': 20,
                'relaxed_demeanor': 20
            }
        )
        indicators.append(stress)

        # Handling practices
        handling = self._score_indicator(
            name="Handling Practices",
            value=observations.get('handling', {}),
            criteria={
                'gentle_handling': 30,
                'trained_staff': 25,
                'minimal_restraint': 25,
                'positive_methods': 20
            }
        )
        indicators.append(handling)

        # Environmental security
        security = self._score_indicator(
            name="Environmental Security",
            value=observations.get('security', {}),
            criteria={
                'hiding_places': 25,
                'predictable_routine': 25,
                'quiet_areas': 25,
                'escape_routes': 25
            }
        )
        indicators.append(security)

        # Human-animal relationship
        relationship = self._score_indicator(
            name="Human-Animal Relationship",
            value=observations.get('human_animal', {}),
            criteria={
                'positive_interactions': 35,
                'no_punishment': 30,
                'trust_evident': 35
            }
        )
        indicators.append(relationship)

        total_score = self._calculate_weighted_score(indicators)
        critical = any(i.critical for i in indicators)

        return FreedomScore(
            freedom=Freedom.FEAR_DISTRESS,
            score=total_score,
            indicators=indicators,
            critical_issues=critical,
            notes=self._generate_notes(indicators)
        )

    def _calculate_overall_score(
        self,
        freedom_scores: Dict[Freedom, FreedomScore]
    ) -> float:
        """
        Calculate overall welfare score from freedom scores
        """
        weights = {
            Freedom.HUNGER_THIRST: 0.20,
            Freedom.DISCOMFORT: 0.20,
            Freedom.PAIN_INJURY: 0.25,
            Freedom.NORMAL_BEHAVIOR: 0.15,
            Freedom.FEAR_DISTRESS: 0.20
        }

        total = 0
        for freedom, score in freedom_scores.items():
            # Critical issues cap the score
            if score.critical_issues:
                capped_score = min(score.score, 40)
            else:
                capped_score = score.score

            total += capped_score * weights[freedom]

        return round(total, 1)
```

### 2.2 Certification Tier Calculator

```python
"""
Certification Tier Determination
Based on assessment scores and compliance
"""

from dataclasses import dataclass
from typing import List, Dict, Optional
from enum import Enum

class CertificationTier(Enum):
    NONE = "none"
    BRONZE = "bronze"
    SILVER = "silver"
    GOLD = "gold"
    PLATINUM = "platinum"

@dataclass
class TierRequirement:
    tier: CertificationTier
    min_overall_score: float
    min_freedom_scores: Dict[str, float]
    max_critical_issues: int
    max_major_nc: int
    max_minor_nc: int
    additional_requirements: List[str]

class CertificationTierCalculator:
    """
    Determine certification tier based on assessment
    """

    def __init__(self):
        self.requirements = self._define_requirements()

    def _define_requirements(self) -> Dict[CertificationTier, TierRequirement]:
        """
        Define requirements for each tier
        """
        return {
            CertificationTier.BRONZE: TierRequirement(
                tier=CertificationTier.BRONZE,
                min_overall_score=60,
                min_freedom_scores={
                    'hunger_thirst': 60,
                    'discomfort': 55,
                    'pain_injury': 65,
                    'normal_behavior': 50,
                    'fear_distress': 55
                },
                max_critical_issues=0,
                max_major_nc=3,
                max_minor_nc=10,
                additional_requirements=[
                    'legal_compliance',
                    'basic_documentation'
                ]
            ),
            CertificationTier.SILVER: TierRequirement(
                tier=CertificationTier.SILVER,
                min_overall_score=75,
                min_freedom_scores={
                    'hunger_thirst': 75,
                    'discomfort': 70,
                    'pain_injury': 80,
                    'normal_behavior': 65,
                    'fear_distress': 70
                },
                max_critical_issues=0,
                max_major_nc=1,
                max_minor_nc=5,
                additional_requirements=[
                    'legal_compliance',
                    'full_documentation',
                    'staff_training',
                    'veterinary_relationship'
                ]
            ),
            CertificationTier.GOLD: TierRequirement(
                tier=CertificationTier.GOLD,
                min_overall_score=85,
                min_freedom_scores={
                    'hunger_thirst': 85,
                    'discomfort': 85,
                    'pain_injury': 90,
                    'normal_behavior': 80,
                    'fear_distress': 85
                },
                max_critical_issues=0,
                max_major_nc=0,
                max_minor_nc=2,
                additional_requirements=[
                    'legal_compliance',
                    'comprehensive_documentation',
                    'advanced_training',
                    'enrichment_program',
                    'health_monitoring',
                    'continuous_improvement'
                ]
            ),
            CertificationTier.PLATINUM: TierRequirement(
                tier=CertificationTier.PLATINUM,
                min_overall_score=95,
                min_freedom_scores={
                    'hunger_thirst': 95,
                    'discomfort': 92,
                    'pain_injury': 95,
                    'normal_behavior': 90,
                    'fear_distress': 95
                },
                max_critical_issues=0,
                max_major_nc=0,
                max_minor_nc=0,
                additional_requirements=[
                    'legal_compliance',
                    'best_practice_documentation',
                    'certified_staff',
                    'comprehensive_enrichment',
                    'proactive_health',
                    'innovation_program',
                    'community_contribution',
                    'transparency_commitment'
                ]
            )
        }

    def determine_tier(
        self,
        assessment: 'FiveFreedomsAssessment',
        audit_result: 'AuditRecord',
        additional_checks: Dict[str, bool]
    ) -> CertificationTier:
        """
        Determine highest achievable tier
        """
        # Start from highest and work down
        for tier in [CertificationTier.PLATINUM, CertificationTier.GOLD,
                     CertificationTier.SILVER, CertificationTier.BRONZE]:

            if self._meets_tier_requirements(
                tier, assessment, audit_result, additional_checks
            ):
                return tier

        return CertificationTier.NONE

    def _meets_tier_requirements(
        self,
        tier: CertificationTier,
        assessment: 'FiveFreedomsAssessment',
        audit_result: 'AuditRecord',
        additional_checks: Dict[str, bool]
    ) -> bool:
        """
        Check if all requirements for tier are met
        """
        req = self.requirements[tier]

        # Check overall score
        if assessment.overall_score < req.min_overall_score:
            return False

        # Check individual freedom scores
        for freedom_key, min_score in req.min_freedom_scores.items():
            freedom = Freedom(freedom_key)
            if assessment.freedoms[freedom].score < min_score:
                return False

        # Check critical issues
        critical_count = sum(
            1 for f in assessment.freedoms.values()
            if f.critical_issues
        )
        if critical_count > req.max_critical_issues:
            return False

        # Check non-conformities
        critical_nc = len([
            nc for nc in audit_result.non_conformities
            if nc.severity == 'CRITICAL'
        ])
        major_nc = len([
            nc for nc in audit_result.non_conformities
            if nc.severity == 'MAJOR'
        ])
        minor_nc = len([
            nc for nc in audit_result.non_conformities
            if nc.severity == 'MINOR'
        ])

        if critical_nc > 0:
            return False
        if major_nc > req.max_major_nc:
            return False
        if minor_nc > req.max_minor_nc:
            return False

        # Check additional requirements
        for req_name in req.additional_requirements:
            if not additional_checks.get(req_name, False):
                return False

        return True

    def get_improvement_path(
        self,
        current_tier: CertificationTier,
        assessment: 'FiveFreedomsAssessment',
        audit_result: 'AuditRecord'
    ) -> Dict:
        """
        Generate roadmap to next tier
        """
        if current_tier == CertificationTier.PLATINUM:
            return {'message': 'Already at highest tier'}

        next_tier = self._get_next_tier(current_tier)
        req = self.requirements[next_tier]
        gaps = []

        # Score gaps
        if assessment.overall_score < req.min_overall_score:
            gaps.append({
                'area': 'Overall Score',
                'current': assessment.overall_score,
                'required': req.min_overall_score,
                'gap': req.min_overall_score - assessment.overall_score
            })

        # Freedom score gaps
        for freedom_key, min_score in req.min_freedom_scores.items():
            freedom = Freedom(freedom_key)
            current = assessment.freedoms[freedom].score
            if current < min_score:
                gaps.append({
                    'area': f'{freedom.value} freedom',
                    'current': current,
                    'required': min_score,
                    'gap': min_score - current
                })

        # Non-conformity requirements
        major_nc = len([
            nc for nc in audit_result.non_conformities
            if nc.severity == 'MAJOR'
        ])
        if major_nc > req.max_major_nc:
            gaps.append({
                'area': 'Major Non-Conformities',
                'current': major_nc,
                'required': f'<= {req.max_major_nc}',
                'action': 'Resolve major non-conformities'
            })

        return {
            'current_tier': current_tier.value,
            'target_tier': next_tier.value,
            'gaps': gaps,
            'additional_requirements': req.additional_requirements
        }
```

### 2.3 Audit Scoring

```python
"""
Audit Scoring and Non-Conformity Classification
"""

from dataclasses import dataclass
from typing import List, Dict, Optional
from datetime import datetime

@dataclass
class AuditCriteria:
    criteria_id: str
    clause: str
    description: str
    category: str
    weight: float
    mandatory: bool
    evidence_required: List[str]

class AuditScoring:
    """
    Audit assessment and scoring system
    """

    def __init__(self, standard_version: str):
        self.criteria = self._load_criteria(standard_version)

    def evaluate_criteria(
        self,
        criteria_id: str,
        observations: Dict,
        evidence: List[str]
    ) -> 'CriteriaResult':
        """
        Evaluate a single audit criteria
        """
        criteria = self.criteria[criteria_id]

        # Check evidence completeness
        evidence_complete = self._check_evidence(
            criteria.evidence_required, evidence
        )

        # Score the criteria
        score = self._score_observations(criteria, observations)

        # Determine compliance level
        if score >= 90:
            compliance = 'FULLY_COMPLIANT'
        elif score >= 70:
            compliance = 'MOSTLY_COMPLIANT'
        elif score >= 50:
            compliance = 'PARTIALLY_COMPLIANT'
        else:
            compliance = 'NON_COMPLIANT'

        # Determine non-conformity severity if applicable
        nc_severity = None
        if compliance in ['PARTIALLY_COMPLIANT', 'NON_COMPLIANT']:
            nc_severity = self._determine_severity(
                criteria, score, observations
            )

        return CriteriaResult(
            criteria_id=criteria_id,
            score=score,
            compliance=compliance,
            evidence_complete=evidence_complete,
            nc_severity=nc_severity,
            observations=observations
        )

    def _determine_severity(
        self,
        criteria: AuditCriteria,
        score: float,
        observations: Dict
    ) -> str:
        """
        Determine non-conformity severity
        """
        # Critical: Immediate animal welfare risk
        if observations.get('immediate_risk', False):
            return 'CRITICAL'

        # Critical: Mandatory criteria completely failed
        if criteria.mandatory and score < 30:
            return 'CRITICAL'

        # Major: Significant deviation from requirements
        if score < 50:
            return 'MAJOR'

        # Major: Mandatory criteria partially failed
        if criteria.mandatory and score < 70:
            return 'MAJOR'

        # Minor: Small deviation
        return 'MINOR'

    def calculate_audit_score(
        self,
        results: List['CriteriaResult']
    ) -> Dict:
        """
        Calculate overall audit score
        """
        total_weight = sum(
            self.criteria[r.criteria_id].weight for r in results
        )
        weighted_score = sum(
            r.score * self.criteria[r.criteria_id].weight
            for r in results
        )

        overall_score = weighted_score / total_weight if total_weight > 0 else 0

        # Count non-conformities
        nc_count = {
            'CRITICAL': 0,
            'MAJOR': 0,
            'MINOR': 0
        }
        for r in results:
            if r.nc_severity:
                nc_count[r.nc_severity] += 1

        # Determine result
        if nc_count['CRITICAL'] > 0:
            result = 'FAILED'
        elif nc_count['MAJOR'] > 3:
            result = 'FAILED'
        elif nc_count['MAJOR'] > 0:
            result = 'PASSED_WITH_CONDITIONS'
        else:
            result = 'PASSED'

        return {
            'overall_score': round(overall_score, 1),
            'result': result,
            'non_conformities': nc_count,
            'criteria_count': len(results),
            'fully_compliant': len([
                r for r in results if r.compliance == 'FULLY_COMPLIANT'
            ])
        }
```

### 2.4 Risk Assessment

```python
"""
Welfare Risk Assessment
Identify and prioritize welfare risks
"""

from dataclasses import dataclass
from typing import List, Dict
from enum import Enum

class RiskLevel(Enum):
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4

@dataclass
class WelfareRisk:
    risk_id: str
    category: str
    description: str
    likelihood: int      # 1-5
    impact: int          # 1-5
    risk_level: RiskLevel
    affected_animals: int
    mitigation: str
    priority: int

class WelfareRiskAssessment:
    """
    Assess welfare risks for entities
    """

    def assess_risks(
        self,
        entity_id: str,
        entity_type: str,
        observations: Dict
    ) -> List[WelfareRisk]:
        """
        Identify and score welfare risks
        """
        risks = []

        # Species-specific risks
        for species in observations.get('species', []):
            species_risks = self._assess_species_risks(
                species, observations
            )
            risks.extend(species_risks)

        # Facility risks
        facility_risks = self._assess_facility_risks(
            entity_type, observations
        )
        risks.extend(facility_risks)

        # Operational risks
        operational_risks = self._assess_operational_risks(
            observations
        )
        risks.extend(operational_risks)

        # Sort by priority
        risks.sort(key=lambda r: (r.risk_level.value, r.priority), reverse=True)

        return risks

    def _assess_species_risks(
        self,
        species: str,
        observations: Dict
    ) -> List[WelfareRisk]:
        """
        Assess species-specific risks
        """
        risks = []
        species_data = observations.get(f'{species}_data', {})

        # Overcrowding risk
        if species_data.get('density_ratio', 0) > 0.8:
            risks.append(WelfareRisk(
                risk_id=f'{species}_overcrowding',
                category='Housing',
                description=f'High density of {species}',
                likelihood=4,
                impact=4,
                risk_level=RiskLevel.HIGH,
                affected_animals=species_data.get('count', 0),
                mitigation='Reduce numbers or increase space',
                priority=1
            ))

        # Disease risk
        if not species_data.get('vaccination_current', True):
            risks.append(WelfareRisk(
                risk_id=f'{species}_disease',
                category='Health',
                description=f'Vaccination not current for {species}',
                likelihood=3,
                impact=4,
                risk_level=RiskLevel.HIGH,
                affected_animals=species_data.get('count', 0),
                mitigation='Update vaccination protocol',
                priority=2
            ))

        # Behavioral risk
        if species_data.get('stereotypies_observed', False):
            risks.append(WelfareRisk(
                risk_id=f'{species}_behavior',
                category='Behavior',
                description=f'Stereotypic behaviors in {species}',
                likelihood=5,
                impact=3,
                risk_level=RiskLevel.MEDIUM,
                affected_animals=species_data.get('affected_count', 0),
                mitigation='Increase enrichment and review housing',
                priority=3
            ))

        return risks

    def _calculate_risk_level(
        self,
        likelihood: int,
        impact: int
    ) -> RiskLevel:
        """
        Calculate risk level from likelihood and impact
        """
        score = likelihood * impact

        if score >= 16:
            return RiskLevel.CRITICAL
        elif score >= 9:
            return RiskLevel.HIGH
        elif score >= 4:
            return RiskLevel.MEDIUM
        else:
            return RiskLevel.LOW

    def generate_risk_report(
        self,
        risks: List[WelfareRisk]
    ) -> Dict:
        """
        Generate risk assessment report
        """
        critical = [r for r in risks if r.risk_level == RiskLevel.CRITICAL]
        high = [r for r in risks if r.risk_level == RiskLevel.HIGH]
        medium = [r for r in risks if r.risk_level == RiskLevel.MEDIUM]
        low = [r for r in risks if r.risk_level == RiskLevel.LOW]

        total_animals_at_risk = sum(r.affected_animals for r in critical + high)

        return {
            'summary': {
                'critical_count': len(critical),
                'high_count': len(high),
                'medium_count': len(medium),
                'low_count': len(low),
                'total_risks': len(risks),
                'animals_at_high_risk': total_animals_at_risk
            },
            'critical_risks': [self._risk_to_dict(r) for r in critical],
            'high_risks': [self._risk_to_dict(r) for r in high],
            'priority_actions': self._generate_priority_actions(critical + high),
            'overall_risk_level': self._overall_level(critical, high)
        }
```

### 2.5 Complaint Analysis

```python
"""
Complaint Analysis and Prioritization
"""

from dataclasses import dataclass
from typing import List, Dict, Optional
from datetime import datetime, timedelta

@dataclass
class ComplaintAnalysis:
    complaint_id: str
    priority_score: float
    urgency: str
    credibility_score: float
    recommended_action: str
    investigation_required: bool
    estimated_response_time: str

class ComplaintAnalyzer:
    """
    Analyze and prioritize welfare complaints
    """

    def analyze_complaint(
        self,
        complaint: 'WelfareComplaint',
        entity_history: Dict
    ) -> ComplaintAnalysis:
        """
        Analyze complaint and determine priority
        """
        # Calculate urgency based on category and severity
        urgency = self._calculate_urgency(complaint)

        # Assess credibility
        credibility = self._assess_credibility(complaint)

        # Check entity history
        history_factor = self._check_history(
            complaint.entity_id, entity_history
        )

        # Calculate priority score
        priority = self._calculate_priority(
            urgency, credibility, history_factor, complaint.severity
        )

        # Determine recommended action
        action = self._determine_action(priority, complaint)

        # Estimate response time
        response_time = self._estimate_response_time(priority)

        return ComplaintAnalysis(
            complaint_id=complaint.complaint_id,
            priority_score=priority,
            urgency=urgency,
            credibility_score=credibility,
            recommended_action=action,
            investigation_required=priority > 0.6,
            estimated_response_time=response_time
        )

    def _calculate_urgency(self, complaint) -> str:
        """
        Calculate urgency level
        """
        urgent_categories = ['ABUSE', 'NEGLECT']
        high_categories = ['POOR_CONDITIONS', 'INADEQUATE_CARE']

        if complaint.category in urgent_categories:
            return 'IMMEDIATE'
        elif complaint.category in high_categories:
            return 'HIGH'
        elif complaint.severity in ['CRITICAL', 'HIGH']:
            return 'HIGH'
        else:
            return 'STANDARD'

    def _assess_credibility(self, complaint) -> float:
        """
        Assess complaint credibility (0-1)
        """
        score = 0.5  # Base score

        # Evidence provided
        if complaint.evidence:
            score += 0.1 * min(len(complaint.evidence), 3)

        # Reporter type
        if complaint.reporter_type == 'INSPECTOR':
            score += 0.3
        elif complaint.reporter_type == 'EMPLOYEE':
            score += 0.2
        elif complaint.reporter_type == 'PUBLIC':
            score += 0.1

        # Specific details
        if len(complaint.description) > 200:
            score += 0.1

        return min(score, 1.0)

    def _check_history(
        self,
        entity_id: str,
        history: Dict
    ) -> float:
        """
        Check entity's complaint history
        """
        past_complaints = history.get('complaints', [])
        past_violations = history.get('violations', [])

        factor = 1.0

        # Recent complaints increase priority
        recent = [
            c for c in past_complaints
            if c['date'] > datetime.now() - timedelta(days=365)
        ]
        factor += 0.1 * len(recent)

        # Past substantiated complaints
        substantiated = [
            c for c in past_complaints
            if c['outcome'] == 'SUBSTANTIATED'
        ]
        factor += 0.2 * len(substantiated)

        # Current violations
        factor += 0.3 * len(past_violations)

        return min(factor, 2.0)

    def _calculate_priority(
        self,
        urgency: str,
        credibility: float,
        history_factor: float,
        severity: str
    ) -> float:
        """
        Calculate overall priority score (0-1)
        """
        urgency_scores = {
            'IMMEDIATE': 1.0,
            'HIGH': 0.7,
            'STANDARD': 0.4
        }
        severity_scores = {
            'CRITICAL': 1.0,
            'HIGH': 0.8,
            'MEDIUM': 0.5,
            'LOW': 0.3
        }

        base = urgency_scores.get(urgency, 0.5)
        severity_factor = severity_scores.get(severity, 0.5)

        priority = (base * 0.4 +
                   credibility * 0.3 +
                   severity_factor * 0.3) * history_factor

        return min(priority, 1.0)

    def _determine_action(
        self,
        priority: float,
        complaint
    ) -> str:
        """
        Determine recommended action
        """
        if priority > 0.8:
            return 'IMMEDIATE_INVESTIGATION'
        elif priority > 0.6:
            return 'PRIORITY_INVESTIGATION'
        elif priority > 0.4:
            return 'SCHEDULED_INSPECTION'
        else:
            return 'MONITOR_AND_REVIEW'
```

---

## Document Information

| Field | Value |
|-------|-------|
| Standard | WIA-PET-WELFARE-GLOBAL Version 1.0.0 |
| Phase | 2 - Algorithms |
| Status | Active |
| Philosophy | Hongik Ingan |
