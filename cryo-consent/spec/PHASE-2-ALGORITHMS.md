# CRYO-CONSENT Phase 2: Algorithms Specification

## Overview

This document defines the algorithms for consent validation, conflict detection, revival condition evaluation, and amendment management.

## 1. Consent Validation Engine

### ConsentValidator Class

```python
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
from enum import Enum
from datetime import datetime, timedelta

class ValidationResult(Enum):
    VALID = "valid"
    INVALID = "invalid"
    INCOMPLETE = "incomplete"
    REQUIRES_REVIEW = "requires_review"

@dataclass
class ValidationError:
    field: str
    error_code: str
    message: str
    severity: str  # "error", "warning", "info"

@dataclass
class ValidationReport:
    document_id: str
    validation_date: datetime
    result: ValidationResult
    errors: List[ValidationError]
    warnings: List[ValidationError]
    completeness_score: float
    legal_compliance_score: float

class ConsentValidator:
    """
    Validates cryopreservation consent documents for completeness,
    consistency, and legal compliance.
    """

    REQUIRED_SECTIONS = [
        "subject",
        "consent_declarations",
        "revival_conditions",
        "signatures"
    ]

    MINIMUM_WITNESSES = 2
    MINIMUM_GUARDIANS = 3

    def validate_document(
        self,
        document: Dict
    ) -> ValidationReport:
        """
        Perform comprehensive validation of consent document.
        """
        errors: List[ValidationError] = []
        warnings: List[ValidationError] = []

        # Section completeness
        section_errors = self._validate_sections(document)
        errors.extend(section_errors)

        # Subject validation
        subject_errors = self._validate_subject(document.get("subject", {}))
        errors.extend(subject_errors)

        # Mental capacity
        capacity_errors = self._validate_mental_capacity(document)
        errors.extend(capacity_errors)

        # Consent declarations
        consent_errors = self._validate_consent_declarations(
            document.get("consent_declarations", {})
        )
        errors.extend(consent_errors)

        # Revival conditions
        revival_warnings = self._validate_revival_conditions(
            document.get("revival_conditions", {})
        )
        warnings.extend(revival_warnings)

        # Family agreements
        family_warnings = self._validate_family_agreements(
            document.get("family_agreements", {})
        )
        warnings.extend(family_warnings)

        # Signatures
        sig_errors = self._validate_signatures(
            document.get("signatures", {})
        )
        errors.extend(sig_errors)

        # Financial arrangements
        fin_warnings = self._validate_financial(
            document.get("financial_arrangements", {})
        )
        warnings.extend(fin_warnings)

        # Calculate scores
        completeness = self._calculate_completeness(document)
        compliance = self._calculate_legal_compliance(document, errors)

        # Determine overall result
        if any(e.severity == "error" for e in errors):
            result = ValidationResult.INVALID
        elif completeness < 0.9:
            result = ValidationResult.INCOMPLETE
        elif warnings:
            result = ValidationResult.REQUIRES_REVIEW
        else:
            result = ValidationResult.VALID

        return ValidationReport(
            document_id=document.get("document_id", "UNKNOWN"),
            validation_date=datetime.utcnow(),
            result=result,
            errors=errors,
            warnings=warnings,
            completeness_score=completeness,
            legal_compliance_score=compliance
        )

    def _validate_mental_capacity(
        self,
        document: Dict
    ) -> List[ValidationError]:
        """
        Validate mental capacity assessment requirements.
        """
        errors = []
        subject = document.get("subject", {})
        assessment = subject.get("mental_capacity_assessment", {})

        if not assessment:
            errors.append(ValidationError(
                field="subject.mental_capacity_assessment",
                error_code="MISSING_CAPACITY_ASSESSMENT",
                message="Mental capacity assessment is required",
                severity="error"
            ))
            return errors

        # Check assessment completeness
        understanding = assessment.get("understanding", {})
        required_understandings = [
            "preservation_process",
            "risks_and_limitations",
            "no_guarantees",
            "financial_implications",
            "family_impact"
        ]

        for req in required_understandings:
            if not understanding.get(req):
                errors.append(ValidationError(
                    field=f"mental_capacity_assessment.understanding.{req}",
                    error_code="INCOMPLETE_UNDERSTANDING",
                    message=f"Subject must acknowledge understanding of {req}",
                    severity="error"
                ))

        # Check assessor credentials
        assessor = assessment.get("assessor", {})
        if not assessor.get("license_number"):
            errors.append(ValidationError(
                field="mental_capacity_assessment.assessor",
                error_code="MISSING_ASSESSOR_LICENSE",
                message="Assessor must have valid license number",
                severity="error"
            ))

        # Check overall capacity determination
        if assessment.get("overall_capacity") == "INCAPABLE":
            errors.append(ValidationError(
                field="mental_capacity_assessment.overall_capacity",
                error_code="SUBJECT_INCAPABLE",
                message="Subject assessed as incapable; legal representative required",
                severity="error"
            ))

        return errors

    def _validate_signatures(
        self,
        signatures: Dict
    ) -> List[ValidationError]:
        """
        Validate signature requirements.
        """
        errors = []

        # Subject signature
        if not signatures.get("subject_signature"):
            errors.append(ValidationError(
                field="signatures.subject_signature",
                error_code="MISSING_SUBJECT_SIGNATURE",
                message="Subject signature is required",
                severity="error"
            ))

        # Witness signatures
        witness_sigs = signatures.get("witness_signatures", [])
        if len(witness_sigs) < self.MINIMUM_WITNESSES:
            errors.append(ValidationError(
                field="signatures.witness_signatures",
                error_code="INSUFFICIENT_WITNESSES",
                message=f"Minimum {self.MINIMUM_WITNESSES} witnesses required",
                severity="error"
            ))

        # Check witness independence
        for ws in witness_sigs:
            if ws.get("signer_role") in ["SPOUSE", "CHILD", "GUARDIAN"]:
                errors.append(ValidationError(
                    field="signatures.witness_signatures",
                    error_code="INVALID_WITNESS",
                    message="Witnesses cannot be family members or guardians",
                    severity="error"
                ))

        # Notarization (if status requires it)
        # This would be checked based on document status

        return errors

    def _calculate_completeness(self, document: Dict) -> float:
        """
        Calculate document completeness score (0.0 to 1.0).
        """
        total_fields = 0
        completed_fields = 0

        def count_fields(obj, prefix=""):
            nonlocal total_fields, completed_fields

            if isinstance(obj, dict):
                for key, value in obj.items():
                    count_fields(value, f"{prefix}.{key}")
            elif isinstance(obj, list):
                for i, item in enumerate(obj):
                    count_fields(item, f"{prefix}[{i}]")
            else:
                total_fields += 1
                if obj is not None and obj != "" and obj != []:
                    completed_fields += 1

        count_fields(document)

        return completed_fields / total_fields if total_fields > 0 else 0.0

    def _calculate_legal_compliance(
        self,
        document: Dict,
        errors: List[ValidationError]
    ) -> float:
        """
        Calculate legal compliance score (0.0 to 1.0).
        """
        critical_requirements = [
            "MISSING_CAPACITY_ASSESSMENT",
            "SUBJECT_INCAPABLE",
            "MISSING_SUBJECT_SIGNATURE",
            "INSUFFICIENT_WITNESSES",
            "INVALID_WITNESS"
        ]

        critical_failures = sum(
            1 for e in errors
            if e.error_code in critical_requirements
        )

        max_critical = len(critical_requirements)
        compliance = 1.0 - (critical_failures / max_critical)

        return max(0.0, compliance)
```

## 2. Conflict Detection

### ConflictDetector Class

```python
@dataclass
class Conflict:
    conflict_id: str
    conflict_type: str
    description: str
    parties: List[str]
    resolution_suggestion: str
    severity: str  # "critical", "major", "minor"

class ConflictDetector:
    """
    Detects conflicts in consent documents and family agreements.
    """

    def detect_conflicts(
        self,
        document: Dict
    ) -> List[Conflict]:
        """
        Detect all conflicts in consent document.
        """
        conflicts = []

        # Family agreement conflicts
        conflicts.extend(self._detect_family_conflicts(document))

        # Revival condition conflicts
        conflicts.extend(self._detect_revival_conflicts(document))

        # Financial conflicts
        conflicts.extend(self._detect_financial_conflicts(document))

        # Guardian conflicts
        conflicts.extend(self._detect_guardian_conflicts(document))

        return conflicts

    def _detect_family_conflicts(
        self,
        document: Dict
    ) -> List[Conflict]:
        """
        Detect conflicts in family agreements.
        """
        conflicts = []
        family = document.get("family_agreements", {})

        # Spouse objections
        spouse = family.get("spouse_agreement", {})
        if spouse and not spouse.get("consent_given"):
            consent_scope = spouse.get("consent_scope", {})
            objections = consent_scope.get("objections", [])

            if objections:
                conflicts.append(Conflict(
                    conflict_id=f"FAMILY-{document['document_id']}-001",
                    conflict_type="SPOUSE_OBJECTION",
                    description=f"Spouse has objections: {', '.join(objections)}",
                    parties=["subject", "spouse"],
                    resolution_suggestion="Mediation or negotiated terms required",
                    severity="critical"
                ))

        # Children concerns
        children = family.get("children_agreements", [])
        for child in children:
            if child.get("acknowledgment", {}).get("concerns"):
                conflicts.append(Conflict(
                    conflict_id=f"FAMILY-{document['document_id']}-{child['child_id']}",
                    conflict_type="CHILD_CONCERN",
                    description=f"Child {child['child_name']} has concerns",
                    parties=["subject", child["child_name"]],
                    resolution_suggestion="Family discussion recommended",
                    severity="minor"
                ))

        return conflicts

    def _detect_revival_conflicts(
        self,
        document: Dict
    ) -> List[Conflict]:
        """
        Detect conflicts in revival conditions.
        """
        conflicts = []
        conditions = document.get("revival_conditions", {})

        condition_list = conditions.get("conditions", [])

        # Check for mutually exclusive conditions
        time_conditions = [
            c for c in condition_list
            if c.get("parameters", {}).get("condition_type") == "TIME"
        ]

        if len(time_conditions) > 1:
            min_durations = []
            max_durations = []

            for tc in time_conditions:
                params = tc.get("parameters", {})
                if params.get("minimum_duration"):
                    min_durations.append(params["minimum_duration"])
                if params.get("maximum_duration"):
                    max_durations.append(params["maximum_duration"])

            # Check if min > max anywhere
            # (Simplified; real implementation would parse durations)
            if min_durations and max_durations:
                conflicts.append(Conflict(
                    conflict_id=f"REVIVAL-{document['document_id']}-001",
                    conflict_type="CONFLICTING_TIME_CONDITIONS",
                    description="Time conditions may conflict",
                    parties=["subject"],
                    resolution_suggestion="Review and reconcile time conditions",
                    severity="major"
                ))

        # Check for impossible combinations
        medical_conditions = [
            c for c in condition_list
            if c.get("parameters", {}).get("condition_type") == "MEDICAL"
        ]

        if conditions.get("condition_logic") == "ALL" and len(medical_conditions) > 3:
            conflicts.append(Conflict(
                conflict_id=f"REVIVAL-{document['document_id']}-002",
                conflict_type="POTENTIALLY_IMPOSSIBLE_CONDITIONS",
                description="Requiring ALL medical conditions may be impossible",
                parties=["subject"],
                resolution_suggestion="Consider changing to ANY logic or reducing conditions",
                severity="major"
            ))

        return conflicts

    def _detect_guardian_conflicts(
        self,
        document: Dict
    ) -> List[Conflict]:
        """
        Detect conflicts in guardian designations.
        """
        conflicts = []
        family = document.get("family_agreements", {})
        guardians = family.get("guardian_designations", [])

        # Check for overlapping responsibilities without clear priority
        responsibility_map: Dict[str, List[str]] = {}

        for guardian in guardians:
            for resp in guardian.get("responsibilities", []):
                if resp not in responsibility_map:
                    responsibility_map[resp] = []
                responsibility_map[resp].append(guardian["guardian_id"])

        for resp, guardian_ids in responsibility_map.items():
            if len(guardian_ids) > 1:
                # Check if vote weights are equal (potential deadlock)
                weights = [
                    g["vote_weight"] for g in guardians
                    if g["guardian_id"] in guardian_ids
                ]
                if len(set(weights)) == 1:
                    conflicts.append(Conflict(
                        conflict_id=f"GUARDIAN-{document['document_id']}-{resp}",
                        conflict_type="GUARDIAN_DEADLOCK_RISK",
                        description=f"Multiple guardians with equal weight for {resp}",
                        parties=guardian_ids,
                        resolution_suggestion="Assign different weights or tiebreaker",
                        severity="minor"
                    ))

        return conflicts
```

## 3. Revival Condition Evaluator

### RevivalConditionEvaluator Class

```python
@dataclass
class ConditionEvaluation:
    condition_id: str
    status: str  # "MET", "NOT_MET", "PENDING", "WAIVED"
    confidence: float
    evidence: List[str]
    evaluated_at: datetime
    evaluator: str

@dataclass
class RevivalDecision:
    document_id: str
    decision: str  # "APPROVE", "DENY", "PENDING"
    conditions_met: List[str]
    conditions_not_met: List[str]
    conditions_pending: List[str]
    overall_score: float
    recommendation: str
    evaluated_at: datetime

class RevivalConditionEvaluator:
    """
    Evaluates revival conditions to determine if revival should proceed.
    """

    def evaluate_all_conditions(
        self,
        document: Dict,
        current_evidence: Dict
    ) -> RevivalDecision:
        """
        Evaluate all revival conditions against current evidence.
        """
        conditions_config = document.get("revival_conditions", {})
        conditions = conditions_config.get("conditions", [])
        logic = conditions_config.get("condition_logic", "ALL")

        evaluations = []
        for condition in conditions:
            evaluation = self._evaluate_condition(condition, current_evidence)
            evaluations.append(evaluation)

        # Categorize results
        met = [e.condition_id for e in evaluations if e.status == "MET"]
        not_met = [e.condition_id for e in evaluations if e.status == "NOT_MET"]
        pending = [e.condition_id for e in evaluations if e.status == "PENDING"]
        waived = [e.condition_id for e in evaluations if e.status == "WAIVED"]

        # Apply logic
        decision, score = self._apply_logic(
            logic,
            conditions,
            evaluations,
            conditions_config.get("custom_logic")
        )

        # Generate recommendation
        recommendation = self._generate_recommendation(
            decision, met, not_met, pending, conditions
        )

        return RevivalDecision(
            document_id=document["document_id"],
            decision=decision,
            conditions_met=met + waived,
            conditions_not_met=not_met,
            conditions_pending=pending,
            overall_score=score,
            recommendation=recommendation,
            evaluated_at=datetime.utcnow()
        )

    def _evaluate_condition(
        self,
        condition: Dict,
        evidence: Dict
    ) -> ConditionEvaluation:
        """
        Evaluate a single condition.
        """
        condition_type = condition.get("parameters", {}).get("condition_type")
        condition_id = condition["condition_id"]

        if condition_type == "MEDICAL":
            return self._evaluate_medical_condition(condition, evidence)
        elif condition_type == "TECHNOLOGY":
            return self._evaluate_technology_condition(condition, evidence)
        elif condition_type == "TIME":
            return self._evaluate_time_condition(condition, evidence)
        elif condition_type == "SOCIAL":
            return self._evaluate_social_condition(condition, evidence)
        else:
            return ConditionEvaluation(
                condition_id=condition_id,
                status="PENDING",
                confidence=0.0,
                evidence=[],
                evaluated_at=datetime.utcnow(),
                evaluator="SYSTEM"
            )

    def _evaluate_medical_condition(
        self,
        condition: Dict,
        evidence: Dict
    ) -> ConditionEvaluation:
        """
        Evaluate medical condition (e.g., disease cure available).
        """
        params = condition["parameters"]
        target = params["target_condition"]
        required_outcome = params["required_outcome"]

        medical_evidence = evidence.get("medical_advances", {})
        condition_status = medical_evidence.get(target, {})

        status = "NOT_MET"
        confidence = 0.0
        evidence_list = []

        if condition_status:
            current_level = condition_status.get("treatment_level")
            outcome_levels = ["CURABLE", "TREATABLE", "MANAGEABLE"]

            if current_level:
                evidence_list.append(f"Current status: {current_level}")

                required_index = outcome_levels.index(required_outcome)
                current_index = outcome_levels.index(current_level) if current_level in outcome_levels else -1

                if current_index >= 0 and current_index <= required_index:
                    status = "MET"
                    confidence = condition_status.get("confidence", 0.9)

        return ConditionEvaluation(
            condition_id=condition["condition_id"],
            status=status,
            confidence=confidence,
            evidence=evidence_list,
            evaluated_at=datetime.utcnow(),
            evaluator="MEDICAL_EVALUATOR"
        )

    def _evaluate_time_condition(
        self,
        condition: Dict,
        evidence: Dict
    ) -> ConditionEvaluation:
        """
        Evaluate time-based condition.
        """
        params = condition["parameters"]
        min_duration = params.get("minimum_duration")
        max_duration = params.get("maximum_duration")
        reference = params.get("reference_point", "PRESERVATION_START")

        # Get reference date from evidence
        reference_date = evidence.get("preservation_dates", {}).get(reference)
        if not reference_date:
            return ConditionEvaluation(
                condition_id=condition["condition_id"],
                status="PENDING",
                confidence=0.0,
                evidence=["Reference date not found"],
                evaluated_at=datetime.utcnow(),
                evaluator="TIME_EVALUATOR"
            )

        # Parse duration (simplified)
        current_date = datetime.utcnow()
        preservation_start = datetime.fromisoformat(reference_date)
        elapsed = current_date - preservation_start

        # Parse minimum duration (e.g., "P50Y" = 50 years)
        min_years = self._parse_duration_years(min_duration) if min_duration else 0
        max_years = self._parse_duration_years(max_duration) if max_duration else float('inf')

        elapsed_years = elapsed.days / 365.25

        status = "NOT_MET"
        if elapsed_years >= min_years:
            if elapsed_years <= max_years:
                status = "MET"
            else:
                status = "NOT_MET"  # Exceeded maximum

        return ConditionEvaluation(
            condition_id=condition["condition_id"],
            status=status,
            confidence=1.0,
            evidence=[f"Elapsed: {elapsed_years:.1f} years, Required: {min_years}+ years"],
            evaluated_at=datetime.utcnow(),
            evaluator="TIME_EVALUATOR"
        )

    def _parse_duration_years(self, duration: str) -> float:
        """
        Parse ISO 8601 duration to years.
        """
        if not duration:
            return 0

        # Simple parser for "P50Y" format
        if duration.startswith("P") and duration.endswith("Y"):
            return float(duration[1:-1])

        return 0

    def _apply_logic(
        self,
        logic: str,
        conditions: List[Dict],
        evaluations: List[ConditionEvaluation],
        custom_logic: Optional[str]
    ) -> Tuple[str, float]:
        """
        Apply condition logic to determine overall decision.
        """
        met_count = sum(1 for e in evaluations if e.status == "MET")
        total = len(evaluations)
        pending_count = sum(1 for e in evaluations if e.status == "PENDING")

        if logic == "ALL":
            if met_count == total:
                return "APPROVE", 1.0
            elif pending_count > 0 and (met_count + pending_count) == total:
                return "PENDING", met_count / total
            else:
                return "DENY", met_count / total

        elif logic == "ANY":
            if met_count > 0:
                return "APPROVE", met_count / total
            elif pending_count > 0:
                return "PENDING", 0.0
            else:
                return "DENY", 0.0

        elif logic == "CUSTOM" and custom_logic:
            # Would implement custom boolean logic parser
            return "PENDING", 0.5

        return "PENDING", 0.0

    def _generate_recommendation(
        self,
        decision: str,
        met: List[str],
        not_met: List[str],
        pending: List[str],
        conditions: List[Dict]
    ) -> str:
        """
        Generate human-readable recommendation.
        """
        if decision == "APPROVE":
            return f"Revival approved. All {len(met)} conditions met."
        elif decision == "DENY":
            return f"Revival not approved. {len(not_met)} conditions not met: review required."
        else:
            return f"Decision pending. {len(pending)} conditions awaiting evaluation."
```

## 4. Amendment Manager

### AmendmentManager Class

```python
class AmendmentManager:
    """
    Manages consent document amendments.
    """

    CRITICAL_SECTIONS = [
        "revival_conditions",
        "withdrawal",
        "guardian_designations"
    ]

    def create_amendment(
        self,
        document: Dict,
        changes: List[Dict],
        initiator: str,
        reason: str
    ) -> Dict:
        """
        Create a new amendment proposal.
        """
        amendment_number = len(document.get("amendments", [])) + 1

        amendment = {
            "amendment_id": f"AMD-{document['document_id']}-{amendment_number:03d}",
            "amendment_number": amendment_number,
            "effective_date": None,
            "created_at": datetime.utcnow().isoformat(),
            "amendment_type": self._determine_amendment_type(changes),
            "description": self._generate_description(changes),
            "changes": changes,
            "initiator": initiator,
            "reason": reason,
            "approvals": [],
            "required_approvals": self._calculate_required_approvals(document, changes),
            "status": "PENDING",
            "signatures": [],
            "previous_version": document.get("version")
        }

        return amendment

    def _determine_amendment_type(self, changes: List[Dict]) -> str:
        """
        Determine amendment type based on changes.
        """
        sections_affected = set(c["section"] for c in changes)

        if "withdrawal" in str(changes).lower():
            return "COMPLETE_WITHDRAWAL"
        elif "revival_conditions" in sections_affected:
            return "REVIVAL_CONDITIONS"
        elif "guardian" in str(sections_affected).lower():
            return "GUARDIAN_CHANGE"
        elif "financial" in str(sections_affected).lower():
            return "FINANCIAL_ADJUSTMENT"
        else:
            return "OTHER"

    def _calculate_required_approvals(
        self,
        document: Dict,
        changes: List[Dict]
    ) -> int:
        """
        Calculate required approvals based on change criticality.
        """
        sections = set(c["section"] for c in changes)

        is_critical = any(
            critical in section
            for section in sections
            for critical in self.CRITICAL_SECTIONS
        )

        guardians = document.get("family_agreements", {}).get("guardian_designations", [])
        total_guardians = len(guardians)

        if is_critical:
            # Require 2/3 majority for critical changes
            return max(2, int(total_guardians * 2 / 3) + 1)
        else:
            # Simple majority for non-critical
            return max(1, int(total_guardians / 2) + 1)

    def process_approval(
        self,
        amendment: Dict,
        approver_id: str,
        decision: str,
        signature: Dict
    ) -> Dict:
        """
        Process an approval for an amendment.
        """
        approval = {
            "approver_id": approver_id,
            "approver_role": "GUARDIAN",
            "decision": decision,
            "timestamp": datetime.utcnow().isoformat(),
            "signature": signature
        }

        amendment["approvals"].append(approval)

        # Check if threshold reached
        approve_count = sum(
            1 for a in amendment["approvals"]
            if a["decision"] == "APPROVE"
        )

        reject_count = sum(
            1 for a in amendment["approvals"]
            if a["decision"] == "REJECT"
        )

        if approve_count >= amendment["required_approvals"]:
            amendment["status"] = "APPROVED"
            amendment["effective_date"] = datetime.utcnow().isoformat()
        elif reject_count > (len(amendment["approvals"]) - amendment["required_approvals"]):
            amendment["status"] = "REJECTED"

        return amendment

    def apply_amendment(
        self,
        document: Dict,
        amendment: Dict
    ) -> Dict:
        """
        Apply approved amendment to document.
        """
        if amendment["status"] != "APPROVED":
            raise ValueError("Amendment must be approved before applying")

        # Create new version
        new_document = document.copy()

        for change in amendment["changes"]:
            section = change["section"]
            field = change["field"]
            new_value = change["new_value"]

            # Apply change (simplified - real implementation would handle nested paths)
            if section in new_document:
                if isinstance(new_document[section], dict):
                    new_document[section][field] = new_value

        # Update version
        current_version = document.get("version", "1.0.0")
        major, minor, patch = map(int, current_version.split("."))
        new_document["version"] = f"{major}.{minor + 1}.0"

        # Add amendment to history
        if "amendments" not in new_document:
            new_document["amendments"] = []
        new_document["amendments"].append(amendment)

        new_document["last_modified"] = datetime.utcnow().isoformat()

        return new_document
```

## 5. Withdrawal Processor

### WithdrawalProcessor Class

```python
@dataclass
class WithdrawalRequest:
    request_id: str
    document_id: str
    withdrawal_type: str  # "PRE_PRESERVATION", "DURING_PRESERVATION", "REVIVAL_REFUSAL"
    initiator: str
    initiator_type: str  # "SUBJECT", "GUARDIAN", "LEGAL_REPRESENTATIVE"
    reason: str
    requested_at: datetime

@dataclass
class WithdrawalResult:
    request_id: str
    status: str  # "APPROVED", "DENIED", "PENDING"
    refund_amount: Optional[float]
    next_steps: List[str]
    notifications_sent: List[str]

class WithdrawalProcessor:
    """
    Processes consent withdrawal requests.
    """

    def process_withdrawal(
        self,
        document: Dict,
        request: WithdrawalRequest
    ) -> WithdrawalResult:
        """
        Process a withdrawal request.
        """
        if request.withdrawal_type == "PRE_PRESERVATION":
            return self._process_pre_preservation(document, request)
        elif request.withdrawal_type == "DURING_PRESERVATION":
            return self._process_during_preservation(document, request)
        elif request.withdrawal_type == "REVIVAL_REFUSAL":
            return self._process_revival_refusal(document, request)
        else:
            raise ValueError(f"Unknown withdrawal type: {request.withdrawal_type}")

    def _process_pre_preservation(
        self,
        document: Dict,
        request: WithdrawalRequest
    ) -> WithdrawalResult:
        """
        Process withdrawal before preservation begins.
        Subject can freely withdraw.
        """
        refund_policy = document.get("financial_arrangements", {}).get("refund_policy", {})
        pre_policy = refund_policy.get("pre_preservation_withdrawal", {})

        refund_amount = None
        if pre_policy.get("full_refund"):
            total = document.get("financial_arrangements", {}).get("total_commitment", {})
            refund_amount = total.get("amount", 0)
        else:
            percentage = pre_policy.get("refund_percentage", 80)
            total = document.get("financial_arrangements", {}).get("total_commitment", {})
            refund_amount = total.get("amount", 0) * (percentage / 100)

        return WithdrawalResult(
            request_id=request.request_id,
            status="APPROVED",
            refund_amount=refund_amount,
            next_steps=[
                "Document destruction scheduled",
                "Refund processing initiated",
                "Records anonymization pending"
            ],
            notifications_sent=[
                "Subject confirmation",
                "Facility notification",
                "Financial institution notification"
            ]
        )

    def _process_during_preservation(
        self,
        document: Dict,
        request: WithdrawalRequest
    ) -> WithdrawalResult:
        """
        Process withdrawal during preservation (guardian-initiated).
        Requires threshold approval.
        """
        if request.initiator_type != "GUARDIAN":
            return WithdrawalResult(
                request_id=request.request_id,
                status="DENIED",
                refund_amount=None,
                next_steps=["Only guardians can initiate during-preservation withdrawal"],
                notifications_sent=[]
            )

        # Check guardian threshold
        guardians = document.get("family_agreements", {}).get("guardian_designations", [])
        threshold = len(guardians) * 2 // 3 + 1  # 2/3 majority

        return WithdrawalResult(
            request_id=request.request_id,
            status="PENDING",
            refund_amount=None,
            next_steps=[
                f"Requires {threshold} guardian approvals",
                "Voting period: 30 days",
                "Legal review mandatory"
            ],
            notifications_sent=[
                "All guardians notified",
                "Legal counsel notified",
                "Facility notified"
            ]
        )
```

---

*WIA Technical Committee - Cryopreservation Working Group*
