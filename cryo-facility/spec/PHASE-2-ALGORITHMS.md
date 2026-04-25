# CRYO-FACILITY Phase 2: Algorithms Specification

## 1. Overview

This document defines the algorithms for facility certification scoring, inspection processing, risk assessment, and transfer management.

## 2. Certification Scoring Engine

### 2.1 Certification Scorer

```python
from dataclasses import dataclass
from enum import Enum
from typing import List, Dict, Optional
from datetime import datetime, timedelta

class CertificationLevel(Enum):
    LEVEL_1_BASIC = "LEVEL_1_BASIC"
    LEVEL_2_STANDARD = "LEVEL_2_STANDARD"
    LEVEL_3_ADVANCED = "LEVEL_3_ADVANCED"
    LEVEL_4_EXCELLENCE = "LEVEL_4_EXCELLENCE"

class ScoreCategory(Enum):
    INFRASTRUCTURE = "INFRASTRUCTURE"
    SAFETY = "SAFETY"
    REDUNDANCY = "REDUNDANCY"
    PERSONNEL = "PERSONNEL"
    FINANCIAL = "FINANCIAL"
    DOCUMENTATION = "DOCUMENTATION"
    EMERGENCY = "EMERGENCY"
    PATIENT_CARE = "PATIENT_CARE"

@dataclass
class CategoryScore:
    category: ScoreCategory
    score: float
    max_score: float
    weight: float
    findings: List[str]
    recommendations: List[str]

@dataclass
class CertificationScore:
    facility_id: str
    evaluation_date: datetime
    category_scores: Dict[ScoreCategory, CategoryScore]
    total_score: float
    max_score: float
    percentage: float
    achieved_level: CertificationLevel
    eligible_for_upgrade: bool
    critical_deficiencies: List[str]
    expiration_date: datetime

class CertificationScorer:
    """Calculates facility certification scores across all categories."""

    CATEGORY_WEIGHTS = {
        ScoreCategory.INFRASTRUCTURE: 0.20,
        ScoreCategory.SAFETY: 0.20,
        ScoreCategory.REDUNDANCY: 0.15,
        ScoreCategory.PERSONNEL: 0.10,
        ScoreCategory.FINANCIAL: 0.15,
        ScoreCategory.DOCUMENTATION: 0.05,
        ScoreCategory.EMERGENCY: 0.10,
        ScoreCategory.PATIENT_CARE: 0.05
    }

    LEVEL_THRESHOLDS = {
        CertificationLevel.LEVEL_4_EXCELLENCE: 95,
        CertificationLevel.LEVEL_3_ADVANCED: 85,
        CertificationLevel.LEVEL_2_STANDARD: 70,
        CertificationLevel.LEVEL_1_BASIC: 50
    }

    def __init__(self, facility_data: dict, inspection_data: dict):
        self.facility = facility_data
        self.inspection = inspection_data
        self.category_scores = {}

    def calculate_total_score(self) -> CertificationScore:
        """Calculate overall certification score."""
        self._score_infrastructure()
        self._score_safety()
        self._score_redundancy()
        self._score_personnel()
        self._score_financial()
        self._score_documentation()
        self._score_emergency()
        self._score_patient_care()

        total = sum(
            cs.score * cs.weight
            for cs in self.category_scores.values()
        )
        max_total = sum(
            cs.max_score * cs.weight
            for cs in self.category_scores.values()
        )
        percentage = (total / max_total) * 100 if max_total > 0 else 0

        achieved_level = self._determine_level(percentage)
        critical = self._find_critical_deficiencies()

        return CertificationScore(
            facility_id=self.facility['facility_id'],
            evaluation_date=datetime.now(),
            category_scores=self.category_scores,
            total_score=total,
            max_score=max_total,
            percentage=percentage,
            achieved_level=achieved_level,
            eligible_for_upgrade=self._check_upgrade_eligibility(achieved_level),
            critical_deficiencies=critical,
            expiration_date=self._calculate_expiration(achieved_level)
        )

    def _score_infrastructure(self) -> None:
        """Score infrastructure components."""
        score = 0
        max_score = 100
        findings = []
        recommendations = []

        infra = self.facility.get('infrastructure', {})

        # Building quality (25 points)
        buildings = infra.get('buildings', [])
        if buildings:
            avg_class = self._average_building_class(buildings)
            building_score = {
                'CLASS_A': 25, 'CLASS_B': 20,
                'CLASS_C': 15, 'CLASS_D': 10
            }.get(avg_class, 10)
            score += building_score
            if building_score < 20:
                recommendations.append("Upgrade building structural class")

        # Storage units (25 points)
        capacity = self.facility.get('capacity', {})
        units = capacity.get('storage_units', [])
        if units:
            unit_score = min(25, len(units) * 5)
            score += unit_score

            # Check maintenance status
            poor_units = [u for u in units if u.get('maintenance_status') == 'POOR']
            if poor_units:
                findings.append(f"{len(poor_units)} storage units need maintenance")
                score -= len(poor_units) * 2

        # Power systems (25 points)
        power = infra.get('power_systems', {})
        if power.get('redundancy_level') in ['2N', '2N+1']:
            score += 25
        elif power.get('redundancy_level') == 'N+1':
            score += 20
        else:
            score += 10
            recommendations.append("Improve power system redundancy")

        # Monitoring (25 points)
        monitoring = infra.get('monitoring_systems', {})
        if monitoring.get('central_monitoring', {}).get('redundancy'):
            score += 15
        else:
            score += 10
        if monitoring.get('central_monitoring', {}).get('staffing_level') == '24_7':
            score += 10
        else:
            score += 5
            recommendations.append("Consider 24/7 monitoring staffing")

        self.category_scores[ScoreCategory.INFRASTRUCTURE] = CategoryScore(
            category=ScoreCategory.INFRASTRUCTURE,
            score=max(0, min(score, max_score)),
            max_score=max_score,
            weight=self.CATEGORY_WEIGHTS[ScoreCategory.INFRASTRUCTURE],
            findings=findings,
            recommendations=recommendations
        )

    def _score_safety(self) -> None:
        """Score safety systems and compliance."""
        score = 0
        max_score = 100
        findings = []
        recommendations = []

        infra = self.facility.get('infrastructure', {})

        # Fire protection (30 points)
        fire = infra.get('fire_protection', {})
        if fire.get('suppression_type') == 'CLEAN_AGENT':
            score += 30
        elif fire.get('suppression_type'):
            score += 20
        else:
            findings.append("Fire suppression system not documented")
            recommendations.append("Install clean agent fire suppression")

        # Security systems (30 points)
        security = infra.get('security_systems', {})
        security_features = ['access_control', 'surveillance', 'intrusion_detection']
        for feature in security_features:
            if security.get(feature):
                score += 10

        # Alarm systems (20 points)
        monitoring = infra.get('monitoring_systems', {})
        alarm = monitoring.get('alarm_system', {})
        if alarm.get('levels'):
            score += 10
        if alarm.get('escalation_procedure'):
            score += 10
        else:
            recommendations.append("Document alarm escalation procedures")

        # Compliance certifications (20 points)
        certifications = self.facility.get('compliance_certifications', [])
        score += min(20, len(certifications) * 5)

        self.category_scores[ScoreCategory.SAFETY] = CategoryScore(
            category=ScoreCategory.SAFETY,
            score=max(0, min(score, max_score)),
            max_score=max_score,
            weight=self.CATEGORY_WEIGHTS[ScoreCategory.SAFETY],
            findings=findings,
            recommendations=recommendations
        )

    def _score_redundancy(self) -> None:
        """Score redundancy and backup systems."""
        score = 0
        max_score = 100
        findings = []
        recommendations = []

        infra = self.facility.get('infrastructure', {})
        power = infra.get('power_systems', {})

        # Power redundancy (40 points)
        redundancy_scores = {'2N+1': 40, '2N': 35, 'N+1': 25, 'N': 10}
        score += redundancy_scores.get(power.get('redundancy_level', 'N'), 10)

        # Backup generators (20 points)
        generators = power.get('backup_generators', [])
        if len(generators) >= 2:
            score += 20
        elif len(generators) == 1:
            score += 10
            recommendations.append("Add secondary backup generator")
        else:
            findings.append("No backup generators")

        # Battery backup (15 points)
        battery = power.get('battery_backup', {})
        autonomy = battery.get('autonomy_duration', 0)
        if autonomy >= 60:
            score += 15
        elif autonomy >= 30:
            score += 10
        else:
            recommendations.append("Increase battery backup duration")

        # Cryogen supply (25 points)
        cryogen = infra.get('cryogen_supply', {})
        autonomy_days = cryogen.get('autonomy_days', 0)
        if autonomy_days >= 30:
            score += 15
        elif autonomy_days >= 14:
            score += 10
        else:
            findings.append(f"Cryogen autonomy only {autonomy_days} days")

        suppliers = cryogen.get('suppliers', [])
        if len(suppliers) >= 2:
            score += 10
        else:
            recommendations.append("Establish secondary cryogen supplier")

        self.category_scores[ScoreCategory.REDUNDANCY] = CategoryScore(
            category=ScoreCategory.REDUNDANCY,
            score=max(0, min(score, max_score)),
            max_score=max_score,
            weight=self.CATEGORY_WEIGHTS[ScoreCategory.REDUNDANCY],
            findings=findings,
            recommendations=recommendations
        )

    def _score_personnel(self) -> None:
        """Score personnel qualifications and training."""
        score = 0
        max_score = 100
        findings = []
        recommendations = []

        personnel = self.facility.get('personnel', {})

        # Staff qualifications (40 points)
        qualified_ratio = personnel.get('qualified_staff_ratio', 0)
        score += int(qualified_ratio * 40)
        if qualified_ratio < 0.8:
            recommendations.append("Improve staff qualification ratio")

        # Training compliance (30 points)
        training_compliance = personnel.get('training_compliance_rate', 0)
        score += int(training_compliance * 30)
        if training_compliance < 0.9:
            findings.append("Training compliance below 90%")

        # Staff retention (15 points)
        retention_rate = personnel.get('retention_rate', 0)
        score += int(retention_rate * 15)

        # Coverage adequacy (15 points)
        if personnel.get('24_7_coverage'):
            score += 15
        elif personnel.get('on_call_available'):
            score += 10
        else:
            recommendations.append("Establish 24/7 coverage or on-call system")

        self.category_scores[ScoreCategory.PERSONNEL] = CategoryScore(
            category=ScoreCategory.PERSONNEL,
            score=max(0, min(score, max_score)),
            max_score=max_score,
            weight=self.CATEGORY_WEIGHTS[ScoreCategory.PERSONNEL],
            findings=findings,
            recommendations=recommendations
        )

    def _score_financial(self) -> None:
        """Score financial stability and reserves."""
        score = 0
        max_score = 100
        findings = []
        recommendations = []

        financials = self.facility.get('financials', {})

        # Operating reserves (30 points)
        runway_years = financials.get('years_of_runway', 0)
        if runway_years >= 25:
            score += 30
        elif runway_years >= 10:
            score += 25
        elif runway_years >= 5:
            score += 15
        else:
            findings.append(f"Only {runway_years} years of runway")
            score += 5

        # Perpetuity fund (25 points)
        perpetuity = financials.get('perpetuity_fund', {})
        if perpetuity.get('amount', 0) > 0:
            score += 25
        else:
            recommendations.append("Establish perpetuity fund")

        # Insurance coverage (25 points)
        insurance = self.facility.get('insurance_coverage', [])
        required_types = ['GENERAL_LIABILITY', 'PROPERTY', 'PATIENT_PROTECTION']
        covered_types = [i.get('coverage_type') for i in insurance]
        for req in required_types:
            if req in covered_types:
                score += 8
            else:
                findings.append(f"Missing {req} insurance")

        # Audit results (20 points)
        audit_result = financials.get('audit_result', 'DISCLAIMER')
        audit_scores = {
            'UNQUALIFIED': 20,
            'QUALIFIED': 15,
            'ADVERSE': 5,
            'DISCLAIMER': 0
        }
        score += audit_scores.get(audit_result, 0)

        self.category_scores[ScoreCategory.FINANCIAL] = CategoryScore(
            category=ScoreCategory.FINANCIAL,
            score=max(0, min(score, max_score)),
            max_score=max_score,
            weight=self.CATEGORY_WEIGHTS[ScoreCategory.FINANCIAL],
            findings=findings,
            recommendations=recommendations
        )

    def _score_documentation(self) -> None:
        """Score documentation and compliance records."""
        score = 0
        max_score = 100
        findings = []
        recommendations = []

        docs = self.facility.get('documentation', {})

        required_docs = [
            'policies_procedures',
            'emergency_plans',
            'maintenance_records',
            'training_records',
            'incident_reports',
            'inspection_reports'
        ]

        for doc in required_docs:
            if docs.get(doc, {}).get('current'):
                score += 15
            else:
                findings.append(f"Missing or outdated: {doc}")

        # Document control (10 points)
        if docs.get('version_control'):
            score += 10

        self.category_scores[ScoreCategory.DOCUMENTATION] = CategoryScore(
            category=ScoreCategory.DOCUMENTATION,
            score=max(0, min(score, max_score)),
            max_score=max_score,
            weight=self.CATEGORY_WEIGHTS[ScoreCategory.DOCUMENTATION],
            findings=findings,
            recommendations=recommendations
        )

    def _score_emergency(self) -> None:
        """Score emergency preparedness."""
        score = 0
        max_score = 100
        findings = []
        recommendations = []

        emergency = self.facility.get('emergency_plan', {})

        # Emergency plan exists and current (30 points)
        if emergency.get('plan_id'):
            score += 15
            if emergency.get('last_review_date'):
                days_since_review = (datetime.now() -
                    datetime.fromisoformat(emergency['last_review_date'])).days
                if days_since_review <= 365:
                    score += 15
                else:
                    findings.append("Emergency plan review overdue")
        else:
            findings.append("No emergency plan documented")

        # Scenarios covered (25 points)
        scenarios = emergency.get('scenarios', [])
        critical_scenarios = ['POWER_FAILURE', 'CRYOGEN_LEAK', 'FIRE', 'BANKRUPTCY']
        for scenario in critical_scenarios:
            if any(s.get('type') == scenario for s in scenarios):
                score += 6
            else:
                recommendations.append(f"Add scenario: {scenario}")

        # Backup facilities (25 points)
        backup = emergency.get('backup_facilities', [])
        if len(backup) >= 2:
            score += 25
        elif len(backup) == 1:
            score += 15
        else:
            findings.append("No backup facility arrangements")

        # Drills conducted (20 points)
        last_drill = emergency.get('last_drill_date')
        if last_drill:
            days_since_drill = (datetime.now() -
                datetime.fromisoformat(last_drill)).days
            if days_since_drill <= 180:
                score += 20
            elif days_since_drill <= 365:
                score += 10
            else:
                findings.append("Emergency drill overdue")

        self.category_scores[ScoreCategory.EMERGENCY] = CategoryScore(
            category=ScoreCategory.EMERGENCY,
            score=max(0, min(score, max_score)),
            max_score=max_score,
            weight=self.CATEGORY_WEIGHTS[ScoreCategory.EMERGENCY],
            findings=findings,
            recommendations=recommendations
        )

    def _score_patient_care(self) -> None:
        """Score patient care and service quality."""
        score = 0
        max_score = 100
        findings = []
        recommendations = []

        patient_data = self.facility.get('patient_statistics', {})

        # Incident rate (40 points)
        incident_rate = patient_data.get('incident_rate_per_year', 1.0)
        if incident_rate == 0:
            score += 40
        elif incident_rate < 0.01:
            score += 35
        elif incident_rate < 0.05:
            score += 25
        else:
            findings.append(f"Incident rate: {incident_rate:.2%}")
            score += 10

        # Temperature excursions (30 points)
        excursions = patient_data.get('temperature_excursions_per_year', 0)
        if excursions == 0:
            score += 30
        elif excursions < 5:
            score += 20
        else:
            findings.append(f"{excursions} temperature excursions per year")

        # Family satisfaction (30 points)
        satisfaction = patient_data.get('family_satisfaction_score', 0)
        score += int(satisfaction * 30 / 100)

        self.category_scores[ScoreCategory.PATIENT_CARE] = CategoryScore(
            category=ScoreCategory.PATIENT_CARE,
            score=max(0, min(score, max_score)),
            max_score=max_score,
            weight=self.CATEGORY_WEIGHTS[ScoreCategory.PATIENT_CARE],
            findings=findings,
            recommendations=recommendations
        )

    def _determine_level(self, percentage: float) -> CertificationLevel:
        """Determine certification level from percentage score."""
        for level, threshold in self.LEVEL_THRESHOLDS.items():
            if percentage >= threshold:
                return level
        return CertificationLevel.LEVEL_1_BASIC

    def _find_critical_deficiencies(self) -> List[str]:
        """Identify critical deficiencies that block certification."""
        critical = []
        for category, score in self.category_scores.items():
            # Any category below 30% is critical
            if (score.score / score.max_score) < 0.3:
                critical.append(f"Critical deficiency in {category.value}")
        return critical

    def _check_upgrade_eligibility(self, current: CertificationLevel) -> bool:
        """Check if facility is eligible for upgrade."""
        # No critical deficiencies
        if self._find_critical_deficiencies():
            return False
        # All categories above 60%
        for score in self.category_scores.values():
            if (score.score / score.max_score) < 0.6:
                return False
        return True

    def _calculate_expiration(self, level: CertificationLevel) -> datetime:
        """Calculate certification expiration date."""
        validity_years = {
            CertificationLevel.LEVEL_4_EXCELLENCE: 3,
            CertificationLevel.LEVEL_3_ADVANCED: 2,
            CertificationLevel.LEVEL_2_STANDARD: 2,
            CertificationLevel.LEVEL_1_BASIC: 1
        }
        years = validity_years.get(level, 1)
        return datetime.now() + timedelta(days=365 * years)

    def _average_building_class(self, buildings: List[dict]) -> str:
        """Calculate average building class."""
        class_values = {'CLASS_A': 4, 'CLASS_B': 3, 'CLASS_C': 2, 'CLASS_D': 1}
        if not buildings:
            return 'CLASS_D'
        avg = sum(class_values.get(b.get('structural_class', 'CLASS_D'), 1)
                  for b in buildings) / len(buildings)
        if avg >= 3.5:
            return 'CLASS_A'
        elif avg >= 2.5:
            return 'CLASS_B'
        elif avg >= 1.5:
            return 'CLASS_C'
        return 'CLASS_D'
```

## 3. Risk Assessment Engine

### 3.1 Facility Risk Assessor

```python
from dataclasses import dataclass
from typing import List, Tuple
from datetime import datetime
import math

class RiskCategory(Enum):
    OPERATIONAL = "OPERATIONAL"
    FINANCIAL = "FINANCIAL"
    NATURAL_DISASTER = "NATURAL_DISASTER"
    TECHNICAL = "TECHNICAL"
    REGULATORY = "REGULATORY"
    REPUTATIONAL = "REPUTATIONAL"

@dataclass
class RiskFactor:
    factor_id: str
    category: RiskCategory
    description: str
    likelihood: float          # 0-1
    impact: float              # 0-1
    risk_score: float          # likelihood * impact
    mitigations: List[str]
    residual_risk: float

@dataclass
class RiskAssessment:
    facility_id: str
    assessment_date: datetime
    risk_factors: List[RiskFactor]
    overall_risk_score: float
    risk_level: str            # LOW, MEDIUM, HIGH, CRITICAL
    priority_actions: List[str]
    next_review_date: datetime

class FacilityRiskAssessor:
    """Assesses operational and strategic risks for facilities."""

    RISK_WEIGHTS = {
        RiskCategory.OPERATIONAL: 0.30,
        RiskCategory.FINANCIAL: 0.25,
        RiskCategory.TECHNICAL: 0.20,
        RiskCategory.NATURAL_DISASTER: 0.10,
        RiskCategory.REGULATORY: 0.10,
        RiskCategory.REPUTATIONAL: 0.05
    }

    def __init__(self, facility: dict):
        self.facility = facility
        self.risk_factors = []

    def assess_all_risks(self) -> RiskAssessment:
        """Perform comprehensive risk assessment."""
        self._assess_operational_risks()
        self._assess_financial_risks()
        self._assess_technical_risks()
        self._assess_natural_disaster_risks()
        self._assess_regulatory_risks()
        self._assess_reputational_risks()

        overall = self._calculate_overall_risk()
        level = self._determine_risk_level(overall)
        actions = self._prioritize_actions()

        return RiskAssessment(
            facility_id=self.facility['facility_id'],
            assessment_date=datetime.now(),
            risk_factors=self.risk_factors,
            overall_risk_score=overall,
            risk_level=level,
            priority_actions=actions,
            next_review_date=self._determine_review_date(level)
        )

    def _assess_operational_risks(self) -> None:
        """Assess operational risk factors."""
        infra = self.facility.get('infrastructure', {})
        power = infra.get('power_systems', {})

        # Power failure risk
        grid = power.get('grid_connection', {})
        outages = grid.get('average_outages_per_year', 10)
        redundancy = power.get('redundancy_level', 'N')

        likelihood = min(1.0, outages / 20)
        impact = {'N': 0.9, 'N+1': 0.5, '2N': 0.2, '2N+1': 0.1}.get(redundancy, 0.9)

        self.risk_factors.append(RiskFactor(
            factor_id="OP-001",
            category=RiskCategory.OPERATIONAL,
            description="Power failure affecting patient storage",
            likelihood=likelihood,
            impact=impact,
            risk_score=likelihood * impact,
            mitigations=["Backup generators", "Battery systems", "Grid redundancy"],
            residual_risk=likelihood * impact * 0.3
        ))

        # Cryogen supply disruption
        cryogen = infra.get('cryogen_supply', {})
        autonomy = cryogen.get('autonomy_days', 0)
        suppliers = len(cryogen.get('suppliers', []))

        likelihood = 0.3 if suppliers < 2 else 0.1
        impact = max(0.1, 1.0 - (autonomy / 30))

        self.risk_factors.append(RiskFactor(
            factor_id="OP-002",
            category=RiskCategory.OPERATIONAL,
            description="Cryogen supply disruption",
            likelihood=likelihood,
            impact=impact,
            risk_score=likelihood * impact,
            mitigations=["Multiple suppliers", "On-site production", "Large reserves"],
            residual_risk=likelihood * impact * 0.4
        ))

        # Equipment failure
        units = self.facility.get('capacity', {}).get('storage_units', [])
        old_units = sum(1 for u in units if u.get('age_years', 0) > 15)
        old_ratio = old_units / max(1, len(units))

        likelihood = 0.1 + (old_ratio * 0.4)
        impact = 0.6

        self.risk_factors.append(RiskFactor(
            factor_id="OP-003",
            category=RiskCategory.OPERATIONAL,
            description="Storage equipment failure",
            likelihood=likelihood,
            impact=impact,
            risk_score=likelihood * impact,
            mitigations=["Preventive maintenance", "Equipment replacement", "Redundant units"],
            residual_risk=likelihood * impact * 0.3
        ))

    def _assess_financial_risks(self) -> None:
        """Assess financial risk factors."""
        financials = self.facility.get('financials', {})
        runway = financials.get('years_of_runway', 0)

        # Insufficient reserves
        likelihood = max(0, 1.0 - (runway / 10))
        impact = 0.95

        self.risk_factors.append(RiskFactor(
            factor_id="FI-001",
            category=RiskCategory.FINANCIAL,
            description="Insufficient financial reserves",
            likelihood=likelihood,
            impact=impact,
            risk_score=likelihood * impact,
            mitigations=["Perpetuity fund", "Insurance", "Consortium backup"],
            residual_risk=likelihood * impact * 0.5
        ))

        # Insurance gaps
        insurance = self.facility.get('insurance_coverage', [])
        coverage_types = [i.get('coverage_type') for i in insurance]
        required = ['GENERAL_LIABILITY', 'PROPERTY', 'PATIENT_PROTECTION', 'BUSINESS_INTERRUPTION']
        gaps = [r for r in required if r not in coverage_types]

        likelihood = 0.2
        impact = len(gaps) * 0.2

        self.risk_factors.append(RiskFactor(
            factor_id="FI-002",
            category=RiskCategory.FINANCIAL,
            description="Insurance coverage gaps",
            likelihood=likelihood,
            impact=impact,
            risk_score=likelihood * impact,
            mitigations=["Complete insurance portfolio", "Higher coverage limits"],
            residual_risk=likelihood * impact * 0.3
        ))

    def _assess_technical_risks(self) -> None:
        """Assess technical and IT risks."""
        infra = self.facility.get('infrastructure', {})
        monitoring = infra.get('monitoring_systems', {})

        # Monitoring system failure
        redundancy = monitoring.get('central_monitoring', {}).get('redundancy', False)
        likelihood = 0.3 if not redundancy else 0.1
        impact = 0.7

        self.risk_factors.append(RiskFactor(
            factor_id="TE-001",
            category=RiskCategory.TECHNICAL,
            description="Monitoring system failure",
            likelihood=likelihood,
            impact=impact,
            risk_score=likelihood * impact,
            mitigations=["Redundant monitoring", "Manual backup procedures"],
            residual_risk=likelihood * impact * 0.4
        ))

        # Cyber attack
        security = infra.get('security_systems', {})
        cyber_protection = security.get('cyber_security_level', 'BASIC')

        likelihood = {'BASIC': 0.4, 'STANDARD': 0.2, 'ADVANCED': 0.1}.get(cyber_protection, 0.4)
        impact = 0.5

        self.risk_factors.append(RiskFactor(
            factor_id="TE-002",
            category=RiskCategory.TECHNICAL,
            description="Cyber security breach",
            likelihood=likelihood,
            impact=impact,
            risk_score=likelihood * impact,
            mitigations=["Cyber security program", "Air-gapped critical systems"],
            residual_risk=likelihood * impact * 0.4
        ))

    def _assess_natural_disaster_risks(self) -> None:
        """Assess natural disaster risks based on location."""
        location = self.facility.get('location', {})

        # Earthquake risk
        seismic_zone = location.get('seismic_zone', 'ZONE_2A')
        seismic_likelihood = {
            'ZONE_0': 0.01, 'ZONE_1': 0.05, 'ZONE_2A': 0.1,
            'ZONE_2B': 0.2, 'ZONE_3': 0.3, 'ZONE_4': 0.5
        }.get(seismic_zone, 0.1)

        building_class = self.facility.get('infrastructure', {}).get(
            'buildings', [{}])[0].get('structural_class', 'CLASS_C')
        impact = {'CLASS_A': 0.3, 'CLASS_B': 0.5, 'CLASS_C': 0.7, 'CLASS_D': 0.9}.get(building_class, 0.7)

        self.risk_factors.append(RiskFactor(
            factor_id="ND-001",
            category=RiskCategory.NATURAL_DISASTER,
            description="Earthquake damage",
            likelihood=seismic_likelihood,
            impact=impact,
            risk_score=seismic_likelihood * impact,
            mitigations=["Seismic building design", "Geographic backup"],
            residual_risk=seismic_likelihood * impact * 0.5
        ))

        # Flood risk
        flood_zone = location.get('flood_zone', 'MODERATE')
        flood_likelihood = {
            'MINIMAL': 0.01, 'LOW': 0.05, 'MODERATE': 0.15,
            'HIGH': 0.3, 'SPECIAL_HAZARD': 0.5
        }.get(flood_zone, 0.15)

        self.risk_factors.append(RiskFactor(
            factor_id="ND-002",
            category=RiskCategory.NATURAL_DISASTER,
            description="Flood damage",
            likelihood=flood_likelihood,
            impact=0.8,
            risk_score=flood_likelihood * 0.8,
            mitigations=["Elevated storage", "Flood barriers", "Relocation planning"],
            residual_risk=flood_likelihood * 0.8 * 0.4
        ))

    def _assess_regulatory_risks(self) -> None:
        """Assess regulatory compliance risks."""
        cert = self.facility.get('certification', {})

        # Certification expiration
        expiry = cert.get('expiration_date')
        if expiry:
            days_to_expiry = (datetime.fromisoformat(expiry) - datetime.now()).days
            likelihood = max(0, min(1, (180 - days_to_expiry) / 180))
        else:
            likelihood = 0.5

        self.risk_factors.append(RiskFactor(
            factor_id="RE-001",
            category=RiskCategory.REGULATORY,
            description="Certification lapse",
            likelihood=likelihood,
            impact=0.8,
            risk_score=likelihood * 0.8,
            mitigations=["Proactive renewal", "Continuous compliance monitoring"],
            residual_risk=likelihood * 0.8 * 0.3
        ))

    def _assess_reputational_risks(self) -> None:
        """Assess reputational risk factors."""
        incidents = self.facility.get('incident_history', [])
        recent_incidents = [i for i in incidents
            if (datetime.now() - datetime.fromisoformat(i['date'])).days < 365]

        likelihood = min(1.0, len(recent_incidents) * 0.2)
        impact = 0.6

        self.risk_factors.append(RiskFactor(
            factor_id="RP-001",
            category=RiskCategory.REPUTATIONAL,
            description="Reputation damage from incidents",
            likelihood=likelihood,
            impact=impact,
            risk_score=likelihood * impact,
            mitigations=["Incident prevention", "Communication plan", "Transparency"],
            residual_risk=likelihood * impact * 0.5
        ))

    def _calculate_overall_risk(self) -> float:
        """Calculate weighted overall risk score."""
        category_risks = {}
        for factor in self.risk_factors:
            cat = factor.category
            if cat not in category_risks:
                category_risks[cat] = []
            category_risks[cat].append(factor.risk_score)

        weighted_sum = 0
        for category, weight in self.RISK_WEIGHTS.items():
            if category in category_risks:
                avg_risk = sum(category_risks[category]) / len(category_risks[category])
                weighted_sum += avg_risk * weight

        return weighted_sum

    def _determine_risk_level(self, score: float) -> str:
        """Determine risk level from score."""
        if score >= 0.7:
            return "CRITICAL"
        elif score >= 0.5:
            return "HIGH"
        elif score >= 0.3:
            return "MEDIUM"
        return "LOW"

    def _prioritize_actions(self) -> List[str]:
        """Generate prioritized action list."""
        # Sort by risk score descending
        sorted_factors = sorted(self.risk_factors, key=lambda f: f.risk_score, reverse=True)

        actions = []
        for factor in sorted_factors[:5]:
            if factor.risk_score > 0.3:
                actions.append(f"[{factor.factor_id}] Address: {factor.description}")
                actions.extend([f"  - {m}" for m in factor.mitigations[:2]])

        return actions

    def _determine_review_date(self, level: str) -> datetime:
        """Determine next review date based on risk level."""
        days = {
            'CRITICAL': 30,
            'HIGH': 90,
            'MEDIUM': 180,
            'LOW': 365
        }.get(level, 180)
        return datetime.now() + timedelta(days=days)
```

## 4. Transfer Management Engine

### 4.1 Bankruptcy Transfer Manager

```python
@dataclass
class TransferPlan:
    plan_id: str
    origin_facility_id: str
    destination_facility_id: str
    patients_to_transfer: int
    estimated_duration_days: int
    total_cost: float
    phases: List['TransferPhase']
    risks: List[str]
    contingencies: List[str]

@dataclass
class TransferPhase:
    phase_number: int
    name: str
    duration_days: int
    patients_count: int
    tasks: List[str]
    dependencies: List[int]
    responsible_party: str

class BankruptcyTransferManager:
    """Manages patient transfers during facility bankruptcy or closure."""

    def __init__(self, origin_facility: dict, consortium_facilities: List[dict]):
        self.origin = origin_facility
        self.consortium = consortium_facilities

    def create_transfer_plan(self) -> TransferPlan:
        """Create comprehensive transfer plan."""
        patients = self.origin.get('current_occupancy', 0)

        # Find suitable destination facilities
        destinations = self._find_destinations(patients)

        if not destinations:
            raise ValueError("No suitable destination facilities available")

        # Primary destination
        primary = destinations[0]

        # Create phased plan
        phases = self._create_phases(patients, primary)

        # Calculate costs
        total_cost = self._estimate_costs(patients, primary)

        # Identify risks
        risks = self._identify_risks(primary)

        # Define contingencies
        contingencies = self._define_contingencies(destinations)

        return TransferPlan(
            plan_id=f"TXF-{datetime.now().strftime('%Y%m%d')}-{self.origin['facility_id'][-6:]}",
            origin_facility_id=self.origin['facility_id'],
            destination_facility_id=primary['facility_id'],
            patients_to_transfer=patients,
            estimated_duration_days=sum(p.duration_days for p in phases),
            total_cost=total_cost,
            phases=phases,
            risks=risks,
            contingencies=contingencies
        )

    def _find_destinations(self, patient_count: int) -> List[dict]:
        """Find facilities with capacity to receive patients."""
        suitable = []

        for facility in self.consortium:
            available = facility.get('capacity', {}).get('available_slots', 0)
            cert_level = facility.get('certification_level', 'LEVEL_1_BASIC')
            operational = facility.get('operational_status') == 'OPERATIONAL'

            if available >= patient_count * 0.5 and operational:
                suitable.append({
                    **facility,
                    '_available': available,
                    '_score': self._score_destination(facility)
                })

        # Sort by suitability score
        suitable.sort(key=lambda f: f['_score'], reverse=True)
        return suitable

    def _score_destination(self, facility: dict) -> float:
        """Score destination suitability."""
        score = 0

        # Certification level
        level_scores = {
            'LEVEL_4_EXCELLENCE': 40,
            'LEVEL_3_ADVANCED': 30,
            'LEVEL_2_STANDARD': 20,
            'LEVEL_1_BASIC': 10
        }
        score += level_scores.get(facility.get('certification_level'), 10)

        # Available capacity
        available = facility.get('capacity', {}).get('available_slots', 0)
        score += min(30, available)

        # Geographic proximity (simplified)
        same_country = (facility.get('location', {}).get('country') ==
                       self.origin.get('location', {}).get('country'))
        score += 20 if same_country else 0

        # Financial health
        runway = facility.get('financials', {}).get('years_of_runway', 0)
        score += min(10, runway)

        return score

    def _create_phases(self, patients: int, destination: dict) -> List[TransferPhase]:
        """Create phased transfer plan."""
        phases = []

        # Phase 1: Regulatory and legal
        phases.append(TransferPhase(
            phase_number=1,
            name="Regulatory Approval",
            duration_days=30,
            patients_count=0,
            tasks=[
                "Notify regulatory authorities",
                "Obtain transfer approval",
                "Court approval for bankruptcy transfer",
                "Notify patient families/guardians"
            ],
            dependencies=[],
            responsible_party="Legal/Regulatory Team"
        ))

        # Phase 2: Preparation
        phases.append(TransferPhase(
            phase_number=2,
            name="Transfer Preparation",
            duration_days=14,
            patients_count=0,
            tasks=[
                "Prepare destination facility capacity",
                "Arrange transport containers",
                "Coordinate logistics",
                "Staff training at destination"
            ],
            dependencies=[1],
            responsible_party="Operations Team"
        ))

        # Phase 3-N: Patient transfers (batches of 10)
        batch_size = 10
        batches = math.ceil(patients / batch_size)

        for i in range(batches):
            batch_patients = min(batch_size, patients - (i * batch_size))
            phases.append(TransferPhase(
                phase_number=3 + i,
                name=f"Patient Transfer Batch {i + 1}",
                duration_days=3,
                patients_count=batch_patients,
                tasks=[
                    f"Transfer {batch_patients} patients",
                    "Temperature monitoring during transport",
                    "Documentation and chain of custody",
                    "Verification at destination"
                ],
                dependencies=[2] if i == 0 else [2 + i],
                responsible_party="Medical Transport Team"
            ))

        # Final phase: Closeout
        phases.append(TransferPhase(
            phase_number=3 + batches,
            name="Transfer Closeout",
            duration_days=7,
            patients_count=0,
            tasks=[
                "Final verification of all transfers",
                "Documentation completion",
                "Regulatory notification of completion",
                "Origin facility decommissioning"
            ],
            dependencies=[2 + batches],
            responsible_party="Project Management"
        ))

        return phases

    def _estimate_costs(self, patients: int, destination: dict) -> float:
        """Estimate total transfer costs."""
        # Base costs
        per_patient_transport = 5000
        regulatory_fees = 25000
        logistics = 50000
        contingency_factor = 1.2

        # Distance factor (simplified)
        same_country = (destination.get('location', {}).get('country') ==
                       self.origin.get('location', {}).get('country'))
        distance_multiplier = 1.0 if same_country else 2.5

        transport_cost = patients * per_patient_transport * distance_multiplier
        total = (transport_cost + regulatory_fees + logistics) * contingency_factor

        return total

    def _identify_risks(self, destination: dict) -> List[str]:
        """Identify transfer risks."""
        risks = [
            "Temperature excursion during transport",
            "Transport vehicle breakdown",
            "Regulatory delays",
            "Destination capacity constraints",
            "Weather-related delays",
            "Documentation errors"
        ]

        # Add specific risks
        if destination.get('location', {}).get('country') != self.origin.get('location', {}).get('country'):
            risks.append("Cross-border regulatory complications")
            risks.append("Customs clearance delays")

        return risks

    def _define_contingencies(self, destinations: List[dict]) -> List[str]:
        """Define contingency plans."""
        contingencies = []

        if len(destinations) > 1:
            contingencies.append(f"Secondary destination: {destinations[1]['facility_id']}")

        contingencies.extend([
            "Emergency cryogen supply arrangements",
            "Mobile storage unit standby",
            "Alternative transport providers on call",
            "Extended timeline authorization"
        ])

        return contingencies

    def execute_transfer(self, plan: TransferPlan) -> 'TransferExecution':
        """Execute transfer plan with tracking."""
        return TransferExecution(
            plan=plan,
            start_date=datetime.now(),
            status='IN_PROGRESS'
        )

@dataclass
class TransferExecution:
    plan: TransferPlan
    start_date: datetime
    status: str
    completed_phases: List[int] = None
    current_phase: int = 1
    issues: List[str] = None

    def __post_init__(self):
        self.completed_phases = []
        self.issues = []

    def complete_phase(self, phase_number: int) -> None:
        """Mark phase as complete."""
        self.completed_phases.append(phase_number)
        self.current_phase = phase_number + 1

        if self.current_phase > len(self.plan.phases):
            self.status = 'COMPLETED'

    def report_issue(self, issue: str) -> None:
        """Report an issue during transfer."""
        self.issues.append(f"[{datetime.now().isoformat()}] {issue}")
```

## 5. Inspection Scheduler

```python
class InspectionScheduler:
    """Schedules and manages facility inspections."""

    INSPECTION_INTERVALS = {
        'LEVEL_4_EXCELLENCE': {'annual': 1, 'unannounced': 2},
        'LEVEL_3_ADVANCED': {'annual': 1, 'unannounced': 1},
        'LEVEL_2_STANDARD': {'annual': 1, 'unannounced': 1},
        'LEVEL_1_BASIC': {'annual': 1, 'unannounced': 2}
    }

    def __init__(self, facilities: List[dict]):
        self.facilities = facilities

    def generate_annual_schedule(self, year: int) -> List[dict]:
        """Generate inspection schedule for the year."""
        schedule = []

        for facility in self.facilities:
            level = facility.get('certification_level', 'LEVEL_1_BASIC')
            intervals = self.INSPECTION_INTERVALS.get(level, {'annual': 1, 'unannounced': 2})

            # Annual inspection
            last_inspection = facility.get('last_inspection_date')
            if last_inspection:
                next_annual = datetime.fromisoformat(last_inspection) + timedelta(days=365)
            else:
                next_annual = datetime(year, 1, 1) + timedelta(days=random.randint(0, 90))

            schedule.append({
                'facility_id': facility['facility_id'],
                'inspection_type': 'ANNUAL_AUDIT',
                'scheduled_date': next_annual.isoformat(),
                'duration_days': 3,
                'inspectors_required': 2
            })

            # Unannounced inspections (dates not disclosed)
            for _ in range(intervals['unannounced']):
                schedule.append({
                    'facility_id': facility['facility_id'],
                    'inspection_type': 'UNANNOUNCED',
                    'scheduled_window': f"{year}-Q{random.randint(1, 4)}",
                    'duration_days': 1,
                    'inspectors_required': 1
                })

        return schedule
```
