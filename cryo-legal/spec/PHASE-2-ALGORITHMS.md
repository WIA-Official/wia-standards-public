# CRYO-LEGAL Phase 2: Algorithm Specification

## Overview

This document defines the algorithms for legal status management, jurisdiction handling, document processing, and restoration workflows.

## Core Algorithms

### Legal Status Manager

```python
from enum import Enum
from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import List, Optional, Dict, Any
import logging

class LegalStatus(Enum):
    LIVING = "LIVING"
    LEGALLY_DECEASED = "LEGALLY_DECEASED"
    PRESERVED_STATUS = "PRESERVED_STATUS"
    REVIVAL_PENDING = "REVIVAL_PENDING"
    LEGALLY_RESTORED = "LEGALLY_RESTORED"
    PERMANENT_DEATH = "PERMANENT_DEATH"

@dataclass
class StatusTransition:
    transition_id: str
    from_status: LegalStatus
    to_status: LegalStatus
    transition_date: datetime
    authority_id: str
    document_ref: str
    reason: str

class LegalStatusManager:
    """Manages legal status transitions for preserved individuals."""

    # Valid status transitions
    VALID_TRANSITIONS = {
        LegalStatus.LIVING: [LegalStatus.LEGALLY_DECEASED],
        LegalStatus.LEGALLY_DECEASED: [LegalStatus.PRESERVED_STATUS],
        LegalStatus.PRESERVED_STATUS: [
            LegalStatus.REVIVAL_PENDING,
            LegalStatus.PERMANENT_DEATH
        ],
        LegalStatus.REVIVAL_PENDING: [
            LegalStatus.LEGALLY_RESTORED,
            LegalStatus.PRESERVED_STATUS,  # If revival fails, re-preserved
            LegalStatus.PERMANENT_DEATH
        ],
        LegalStatus.LEGALLY_RESTORED: [],  # Terminal state
        LegalStatus.PERMANENT_DEATH: []    # Terminal state
    }

    def __init__(self, record_id: str):
        self.record_id = record_id
        self.current_status = LegalStatus.LIVING
        self.status_history: List[StatusTransition] = []
        self.logger = logging.getLogger(f"LegalStatusManager:{record_id}")

    def can_transition(self, to_status: LegalStatus) -> bool:
        """Check if transition to specified status is valid."""
        valid_targets = self.VALID_TRANSITIONS.get(self.current_status, [])
        return to_status in valid_targets

    def transition(
        self,
        to_status: LegalStatus,
        authority_id: str,
        document_ref: str,
        reason: str
    ) -> tuple[bool, Optional[StatusTransition], str]:
        """Attempt to transition to a new legal status."""

        if not self.can_transition(to_status):
            error_msg = (
                f"Invalid transition: {self.current_status.value} -> {to_status.value}"
            )
            self.logger.warning(error_msg)
            return False, None, error_msg

        transition = StatusTransition(
            transition_id=f"TRANS-{self.record_id}-{len(self.status_history)+1:03d}",
            from_status=self.current_status,
            to_status=to_status,
            transition_date=datetime.utcnow(),
            authority_id=authority_id,
            document_ref=document_ref,
            reason=reason
        )

        self.status_history.append(transition)
        old_status = self.current_status
        self.current_status = to_status

        self.logger.info(
            f"Status transition: {old_status.value} -> {to_status.value}"
        )

        return True, transition, "Transition successful"

    def get_status_duration(self, status: LegalStatus) -> Optional[timedelta]:
        """Get total duration spent in a specific status."""
        total_duration = timedelta()
        in_status = False
        start_time = None

        for transition in self.status_history:
            if transition.to_status == status:
                in_status = True
                start_time = transition.transition_date
            elif in_status and transition.from_status == status:
                in_status = False
                if start_time:
                    total_duration += transition.transition_date - start_time

        # If currently in this status
        if self.current_status == status and start_time:
            total_duration += datetime.utcnow() - start_time

        return total_duration if total_duration.total_seconds() > 0 else None

    def get_history_summary(self) -> Dict[str, Any]:
        """Get summary of status history."""
        return {
            "record_id": self.record_id,
            "current_status": self.current_status.value,
            "transitions_count": len(self.status_history),
            "history": [
                {
                    "from": t.from_status.value,
                    "to": t.to_status.value,
                    "date": t.transition_date.isoformat(),
                    "reason": t.reason
                }
                for t in self.status_history
            ]
        }
```

### Jurisdiction Handler

```python
from dataclasses import dataclass
from typing import List, Dict, Optional, Set
from enum import Enum

class RecognitionStatus(Enum):
    FULL_RECOGNITION = "FULL_RECOGNITION"
    PARTIAL_RECOGNITION = "PARTIAL_RECOGNITION"
    PENDING_RECOGNITION = "PENDING_RECOGNITION"
    NOT_RECOGNIZED = "NOT_RECOGNIZED"
    TREATY_BASED = "TREATY_BASED"

class LegalSystem(Enum):
    COMMON_LAW = "COMMON_LAW"
    CIVIL_LAW = "CIVIL_LAW"
    RELIGIOUS_LAW = "RELIGIOUS_LAW"
    CUSTOMARY_LAW = "CUSTOMARY_LAW"
    MIXED = "MIXED"

@dataclass
class Jurisdiction:
    country: str
    country_code: str
    region: Optional[str]
    legal_system: LegalSystem
    treaty_member: bool
    bilateral_agreements: List[str]

@dataclass
class JurisdictionRecognition:
    jurisdiction: Jurisdiction
    recognition_status: RecognitionStatus
    local_status_equivalent: str
    registration_id: str
    registration_date: str
    requirements: List[str]

class JurisdictionHandler:
    """Handles multi-jurisdiction legal status recognition."""

    # Treaty members automatically recognize preserved status
    TREATY_MEMBERS = {
        "KR", "JP", "US", "GB", "DE", "FR", "AU", "CA", "CH", "SG"
    }

    # Bilateral agreement mappings
    BILATERAL_AGREEMENTS = {
        ("KR", "US"): "KR-US Cryopreservation Recognition Agreement 2030",
        ("KR", "JP"): "KR-JP Mutual Recognition Protocol 2028",
        ("US", "GB"): "US-GB Legal Status Framework 2029",
    }

    # Legal system compatibility for status translation
    STATUS_TRANSLATION = {
        LegalSystem.COMMON_LAW: {
            "PRESERVED_STATUS": "Suspended Legal Death",
            "REVIVAL_PENDING": "Pending Restoration of Rights"
        },
        LegalSystem.CIVIL_LAW: {
            "PRESERVED_STATUS": "Status Juridique Suspendu",
            "REVIVAL_PENDING": "Restauration en Cours"
        }
    }

    def __init__(self, primary_jurisdiction: Jurisdiction):
        self.primary_jurisdiction = primary_jurisdiction
        self.recognized_jurisdictions: Dict[str, JurisdictionRecognition] = {}

    def check_recognition(
        self,
        target_jurisdiction: Jurisdiction
    ) -> JurisdictionRecognition:
        """Check if preserved status is recognized in target jurisdiction."""

        # Check if treaty member
        if target_jurisdiction.country_code in self.TREATY_MEMBERS:
            return JurisdictionRecognition(
                jurisdiction=target_jurisdiction,
                recognition_status=RecognitionStatus.TREATY_BASED,
                local_status_equivalent=self._translate_status(target_jurisdiction),
                registration_id="",
                registration_date="",
                requirements=["Treaty registration only"]
            )

        # Check bilateral agreement
        pair = tuple(sorted([
            self.primary_jurisdiction.country_code,
            target_jurisdiction.country_code
        ]))
        if pair in self.BILATERAL_AGREEMENTS:
            return JurisdictionRecognition(
                jurisdiction=target_jurisdiction,
                recognition_status=RecognitionStatus.FULL_RECOGNITION,
                local_status_equivalent=self._translate_status(target_jurisdiction),
                registration_id="",
                registration_date="",
                requirements=["Bilateral agreement registration"]
            )

        # Otherwise, recognition may be limited or require court action
        return JurisdictionRecognition(
            jurisdiction=target_jurisdiction,
            recognition_status=RecognitionStatus.PENDING_RECOGNITION,
            local_status_equivalent="Unknown",
            registration_id="",
            registration_date="",
            requirements=[
                "Court petition required",
                "Expert legal opinion",
                "Document apostille",
                "Local registration"
            ]
        )

    def _translate_status(self, jurisdiction: Jurisdiction) -> str:
        """Translate status to local legal equivalent."""
        translations = self.STATUS_TRANSLATION.get(
            jurisdiction.legal_system, {}
        )
        return translations.get("PRESERVED_STATUS", "Preserved Legal Status")

    def register_recognition(
        self,
        recognition: JurisdictionRecognition
    ) -> None:
        """Register jurisdiction recognition."""
        self.recognized_jurisdictions[
            recognition.jurisdiction.country_code
        ] = recognition

    def get_recognition_map(self) -> Dict[str, str]:
        """Get map of all jurisdiction recognitions."""
        return {
            code: rec.recognition_status.value
            for code, rec in self.recognized_jurisdictions.items()
        }

    def check_conflict_of_laws(
        self,
        jurisdictions: List[Jurisdiction]
    ) -> List[Dict[str, Any]]:
        """Check for conflicts between jurisdictions."""
        conflicts = []

        for i, j1 in enumerate(jurisdictions):
            for j2 in jurisdictions[i+1:]:
                conflict = self._check_pair_conflict(j1, j2)
                if conflict:
                    conflicts.append(conflict)

        return conflicts

    def _check_pair_conflict(
        self,
        j1: Jurisdiction,
        j2: Jurisdiction
    ) -> Optional[Dict[str, Any]]:
        """Check for conflict between two jurisdictions."""
        # Different legal systems may have conflicts
        if j1.legal_system != j2.legal_system:
            return {
                "jurisdictions": [j1.country_code, j2.country_code],
                "conflict_type": "LEGAL_SYSTEM_DIFFERENCE",
                "resolution": "Apply treaty rules or primary jurisdiction law"
            }
        return None
```

### Document Processor

```python
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Dict
from enum import Enum
import hashlib

class DocumentType(Enum):
    DEATH_CERTIFICATE = "DEATH_CERTIFICATE"
    PRESERVATION_DECLARATION = "PRESERVATION_DECLARATION"
    RESTORATION_CERTIFICATE = "RESTORATION_CERTIFICATE"
    COURT_ORDER = "COURT_ORDER"
    GUARDIANSHIP_ORDER = "GUARDIANSHIP_ORDER"
    TRUST_DOCUMENT = "TRUST_DOCUMENT"
    POWER_OF_ATTORNEY = "POWER_OF_ATTORNEY"
    IDENTITY_VERIFICATION = "IDENTITY_VERIFICATION"

class DocumentStatus(Enum):
    DRAFT = "DRAFT"
    PENDING_SIGNATURE = "PENDING_SIGNATURE"
    ACTIVE = "ACTIVE"
    SUPERSEDED = "SUPERSEDED"
    REVOKED = "REVOKED"
    EXPIRED = "EXPIRED"

@dataclass
class LegalDocument:
    document_id: str
    document_type: DocumentType
    title: str
    content_hash: str
    status: DocumentStatus
    created_at: datetime
    effective_date: datetime
    expiration_date: Optional[datetime]
    signatures: List[Dict]
    notarized: bool
    apostille: bool
    jurisdiction: str

class DocumentProcessor:
    """Processes and validates legal documents."""

    # Required signatures by document type
    SIGNATURE_REQUIREMENTS = {
        DocumentType.DEATH_CERTIFICATE: {
            "required": ["CERTIFYING_PHYSICIAN", "REGISTRAR"],
            "optional": ["WITNESS"]
        },
        DocumentType.PRESERVATION_DECLARATION: {
            "required": ["ISSUING_AUTHORITY", "FACILITY_REPRESENTATIVE"],
            "optional": ["WITNESS", "NOTARY"]
        },
        DocumentType.RESTORATION_CERTIFICATE: {
            "required": ["COURT_OFFICIAL", "MEDICAL_AUTHORITY", "IDENTITY_VERIFIER"],
            "optional": ["WITNESS"]
        },
        DocumentType.COURT_ORDER: {
            "required": ["JUDGE", "CLERK"],
            "optional": []
        },
        DocumentType.GUARDIANSHIP_ORDER: {
            "required": ["JUDGE", "GUARDIAN"],
            "optional": ["WITNESS", "NOTARY"]
        }
    }

    # Validity periods by document type
    VALIDITY_PERIODS = {
        DocumentType.DEATH_CERTIFICATE: None,  # No expiration
        DocumentType.PRESERVATION_DECLARATION: None,
        DocumentType.RESTORATION_CERTIFICATE: None,
        DocumentType.COURT_ORDER: None,
        DocumentType.GUARDIANSHIP_ORDER: 365 * 5,  # 5 years review
        DocumentType.POWER_OF_ATTORNEY: 365 * 3,   # 3 years
    }

    def __init__(self):
        self.documents: Dict[str, LegalDocument] = {}
        self.document_counter = 0

    def create_document(
        self,
        document_type: DocumentType,
        title: str,
        content: bytes,
        jurisdiction: str,
        effective_date: datetime
    ) -> LegalDocument:
        """Create a new legal document."""
        self.document_counter += 1
        document_id = f"DOC-{document_type.value[:4]}-{self.document_counter:06d}"

        content_hash = hashlib.sha256(content).hexdigest()

        validity_days = self.VALIDITY_PERIODS.get(document_type)
        expiration = None
        if validity_days:
            expiration = effective_date + timedelta(days=validity_days)

        document = LegalDocument(
            document_id=document_id,
            document_type=document_type,
            title=title,
            content_hash=content_hash,
            status=DocumentStatus.DRAFT,
            created_at=datetime.utcnow(),
            effective_date=effective_date,
            expiration_date=expiration,
            signatures=[],
            notarized=False,
            apostille=False,
            jurisdiction=jurisdiction
        )

        self.documents[document_id] = document
        return document

    def add_signature(
        self,
        document_id: str,
        signer_id: str,
        signer_name: str,
        signer_role: str,
        signature_type: str,
        digital_signature: Optional[str] = None
    ) -> tuple[bool, str]:
        """Add signature to document."""
        if document_id not in self.documents:
            return False, "Document not found"

        document = self.documents[document_id]

        if document.status not in [DocumentStatus.DRAFT, DocumentStatus.PENDING_SIGNATURE]:
            return False, f"Cannot sign document in status: {document.status.value}"

        signature = {
            "signer_id": signer_id,
            "signer_name": signer_name,
            "signer_role": signer_role,
            "signature_type": signature_type,
            "signature_date": datetime.utcnow().isoformat(),
            "digital_signature": digital_signature,
            "verified": digital_signature is not None
        }

        document.signatures.append(signature)
        document.status = DocumentStatus.PENDING_SIGNATURE

        return True, "Signature added"

    def validate_signatures(self, document_id: str) -> tuple[bool, List[str]]:
        """Validate that all required signatures are present."""
        if document_id not in self.documents:
            return False, ["Document not found"]

        document = self.documents[document_id]
        requirements = self.SIGNATURE_REQUIREMENTS.get(document.document_type)

        if not requirements:
            return True, []

        issues = []
        signed_roles = {s["signer_role"] for s in document.signatures}

        for required_role in requirements["required"]:
            if required_role not in signed_roles:
                issues.append(f"Missing required signature: {required_role}")

        return len(issues) == 0, issues

    def finalize_document(self, document_id: str) -> tuple[bool, str]:
        """Finalize document after all signatures collected."""
        valid, issues = self.validate_signatures(document_id)

        if not valid:
            return False, f"Cannot finalize: {', '.join(issues)}"

        document = self.documents[document_id]
        document.status = DocumentStatus.ACTIVE

        return True, "Document finalized and active"

    def notarize_document(
        self,
        document_id: str,
        notary_id: str,
        notary_name: str,
        notary_jurisdiction: str
    ) -> tuple[bool, str]:
        """Add notarization to document."""
        if document_id not in self.documents:
            return False, "Document not found"

        document = self.documents[document_id]

        if document.status != DocumentStatus.ACTIVE:
            return False, "Document must be active before notarization"

        document.notarized = True
        document.signatures.append({
            "signer_id": notary_id,
            "signer_name": notary_name,
            "signer_role": "NOTARY",
            "signature_type": "NOTARY_SEAL",
            "signature_date": datetime.utcnow().isoformat(),
            "jurisdiction": notary_jurisdiction
        })

        return True, "Document notarized"

    def add_apostille(
        self,
        document_id: str,
        apostille_number: str,
        issuing_country: str,
        issuing_authority: str
    ) -> tuple[bool, str]:
        """Add apostille for international recognition."""
        if document_id not in self.documents:
            return False, "Document not found"

        document = self.documents[document_id]

        if not document.notarized:
            return False, "Document must be notarized before apostille"

        document.apostille = True
        # Store apostille details...

        return True, f"Apostille {apostille_number} added"

    def check_document_validity(self, document_id: str) -> Dict[str, Any]:
        """Check if document is currently valid."""
        if document_id not in self.documents:
            return {"valid": False, "reason": "Document not found"}

        document = self.documents[document_id]

        if document.status != DocumentStatus.ACTIVE:
            return {"valid": False, "reason": f"Status: {document.status.value}"}

        if document.expiration_date and datetime.utcnow() > document.expiration_date:
            document.status = DocumentStatus.EXPIRED
            return {"valid": False, "reason": "Document expired"}

        return {
            "valid": True,
            "document_id": document_id,
            "document_type": document.document_type.value,
            "effective_date": document.effective_date.isoformat(),
            "expiration_date": (
                document.expiration_date.isoformat()
                if document.expiration_date else None
            )
        }
```

### Restoration Workflow Engine

```python
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Dict, Any
from enum import Enum

class RestorationStage(Enum):
    INITIATED = "INITIATED"
    IDENTITY_VERIFICATION = "IDENTITY_VERIFICATION"
    MEDICAL_CLEARANCE = "MEDICAL_CLEARANCE"
    LEGAL_REVIEW = "LEGAL_REVIEW"
    COURT_PETITION = "COURT_PETITION"
    COURT_HEARING = "COURT_HEARING"
    RESTORATION_GRANTED = "RESTORATION_GRANTED"
    RIGHTS_RESTORATION = "RIGHTS_RESTORATION"
    COMPLETED = "COMPLETED"
    DENIED = "DENIED"

@dataclass
class RestorationTask:
    task_id: str
    stage: RestorationStage
    description: str
    assigned_to: str
    due_date: datetime
    completed: bool
    completion_date: Optional[datetime]
    notes: str

@dataclass
class RestorationCase:
    case_id: str
    subject_id: str
    revival_procedure_id: str
    current_stage: RestorationStage
    stages_completed: List[RestorationStage]
    tasks: List[RestorationTask]
    documents: List[str]
    court_case_number: Optional[str]
    judge_assigned: Optional[str]
    hearing_date: Optional[datetime]
    decision: Optional[str]
    created_at: datetime
    last_updated: datetime

class RestorationWorkflowEngine:
    """Manages the legal restoration workflow."""

    # Stage order
    STAGE_ORDER = [
        RestorationStage.INITIATED,
        RestorationStage.IDENTITY_VERIFICATION,
        RestorationStage.MEDICAL_CLEARANCE,
        RestorationStage.LEGAL_REVIEW,
        RestorationStage.COURT_PETITION,
        RestorationStage.COURT_HEARING,
        RestorationStage.RESTORATION_GRANTED,
        RestorationStage.RIGHTS_RESTORATION,
        RestorationStage.COMPLETED
    ]

    # Required tasks per stage
    STAGE_TASKS = {
        RestorationStage.IDENTITY_VERIFICATION: [
            "Verify biometric identity (DNA)",
            "Verify biometric identity (fingerprint)",
            "Cross-reference CRYO-IDENTITY records",
            "Obtain witness statements"
        ],
        RestorationStage.MEDICAL_CLEARANCE: [
            "Complete medical examination",
            "Assess mental capacity",
            "Document functional status",
            "Obtain medical clearance certificate"
        ],
        RestorationStage.LEGAL_REVIEW: [
            "Review preservation documents",
            "Check outstanding legal obligations",
            "Review family status changes",
            "Prepare legal summary"
        ],
        RestorationStage.COURT_PETITION: [
            "Draft petition for restoration",
            "Compile supporting documents",
            "File petition with court",
            "Serve notice to interested parties"
        ],
        RestorationStage.COURT_HEARING: [
            "Attend court hearing",
            "Present evidence",
            "Respond to objections",
            "Await court decision"
        ],
        RestorationStage.RIGHTS_RESTORATION: [
            "Obtain new/restored identification",
            "Restore social security status",
            "Restore voting rights",
            "Restore professional licenses",
            "Update family records"
        ]
    }

    def __init__(self):
        self.cases: Dict[str, RestorationCase] = {}
        self.case_counter = 0

    def initiate_restoration(
        self,
        subject_id: str,
        revival_procedure_id: str
    ) -> RestorationCase:
        """Initiate a new restoration case."""
        self.case_counter += 1
        case_id = f"RESTORE-{datetime.utcnow().year}-{self.case_counter:04d}"

        case = RestorationCase(
            case_id=case_id,
            subject_id=subject_id,
            revival_procedure_id=revival_procedure_id,
            current_stage=RestorationStage.INITIATED,
            stages_completed=[],
            tasks=[],
            documents=[],
            court_case_number=None,
            judge_assigned=None,
            hearing_date=None,
            decision=None,
            created_at=datetime.utcnow(),
            last_updated=datetime.utcnow()
        )

        self.cases[case_id] = case

        # Generate initial tasks
        self._generate_stage_tasks(case, RestorationStage.IDENTITY_VERIFICATION)

        return case

    def _generate_stage_tasks(
        self,
        case: RestorationCase,
        stage: RestorationStage
    ) -> None:
        """Generate tasks for a stage."""
        task_descriptions = self.STAGE_TASKS.get(stage, [])

        for i, description in enumerate(task_descriptions):
            task = RestorationTask(
                task_id=f"{case.case_id}-{stage.value[:4]}-{i+1:02d}",
                stage=stage,
                description=description,
                assigned_to="",
                due_date=datetime.utcnow(),  # Should be calculated properly
                completed=False,
                completion_date=None,
                notes=""
            )
            case.tasks.append(task)

    def complete_task(
        self,
        case_id: str,
        task_id: str,
        completed_by: str,
        notes: str
    ) -> tuple[bool, str]:
        """Mark a task as completed."""
        if case_id not in self.cases:
            return False, "Case not found"

        case = self.cases[case_id]
        task = next((t for t in case.tasks if t.task_id == task_id), None)

        if not task:
            return False, "Task not found"

        task.completed = True
        task.completion_date = datetime.utcnow()
        task.assigned_to = completed_by
        task.notes = notes
        case.last_updated = datetime.utcnow()

        # Check if stage is complete
        self._check_stage_completion(case)

        return True, "Task completed"

    def _check_stage_completion(self, case: RestorationCase) -> None:
        """Check if current stage is complete and advance if so."""
        current_tasks = [
            t for t in case.tasks
            if t.stage == case.current_stage
        ]

        all_complete = all(t.completed for t in current_tasks)

        if all_complete and current_tasks:
            case.stages_completed.append(case.current_stage)

            # Advance to next stage
            current_idx = self.STAGE_ORDER.index(case.current_stage)
            if current_idx < len(self.STAGE_ORDER) - 1:
                next_stage = self.STAGE_ORDER[current_idx + 1]
                case.current_stage = next_stage
                self._generate_stage_tasks(case, next_stage)

    def file_court_petition(
        self,
        case_id: str,
        court_name: str,
        jurisdiction: str
    ) -> tuple[bool, str, Optional[str]]:
        """File petition with court."""
        if case_id not in self.cases:
            return False, "Case not found", None

        case = self.cases[case_id]

        if case.current_stage != RestorationStage.COURT_PETITION:
            return False, "Not at court petition stage", None

        # Generate court case number
        court_case_number = f"CIVIL-{datetime.utcnow().year}-RESTORE-{self.case_counter}"
        case.court_case_number = court_case_number
        case.last_updated = datetime.utcnow()

        return True, "Petition filed", court_case_number

    def record_court_decision(
        self,
        case_id: str,
        decision: str,
        judge_name: str,
        decision_date: datetime,
        order_text: str
    ) -> tuple[bool, str]:
        """Record court decision."""
        if case_id not in self.cases:
            return False, "Case not found"

        case = self.cases[case_id]
        case.decision = decision
        case.judge_assigned = judge_name

        if decision == "GRANTED":
            case.current_stage = RestorationStage.RESTORATION_GRANTED
            case.stages_completed.append(RestorationStage.COURT_HEARING)
            self._generate_stage_tasks(case, RestorationStage.RIGHTS_RESTORATION)
        elif decision == "DENIED":
            case.current_stage = RestorationStage.DENIED

        case.last_updated = datetime.utcnow()
        return True, f"Decision recorded: {decision}"

    def get_case_status(self, case_id: str) -> Dict[str, Any]:
        """Get current status of restoration case."""
        if case_id not in self.cases:
            return {"error": "Case not found"}

        case = self.cases[case_id]

        pending_tasks = [
            {
                "task_id": t.task_id,
                "description": t.description,
                "stage": t.stage.value
            }
            for t in case.tasks
            if not t.completed
        ]

        return {
            "case_id": case.case_id,
            "subject_id": case.subject_id,
            "current_stage": case.current_stage.value,
            "stages_completed": [s.value for s in case.stages_completed],
            "pending_tasks": pending_tasks,
            "court_case_number": case.court_case_number,
            "decision": case.decision,
            "last_updated": case.last_updated.isoformat()
        }
```

### Guardianship Manager

```python
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Dict
from enum import Enum

class GuardianshipStatus(Enum):
    ACTIVE = "ACTIVE"
    SUSPENDED = "SUSPENDED"
    TERMINATED = "TERMINATED"
    TRANSFERRED = "TRANSFERRED"

@dataclass
class Guardian:
    guardian_id: str
    name: str
    relationship: str
    priority: int
    contact_info: Dict[str, str]
    appointment_date: datetime
    active: bool

@dataclass
class Guardianship:
    guardianship_id: str
    subject_id: str
    status: GuardianshipStatus
    guardians: List[Guardian]
    scope: Dict[str, bool]
    appointment_authority: str
    court_order_ref: str
    created_at: datetime
    review_date: datetime

class GuardianshipManager:
    """Manages guardianship for preserved individuals."""

    DEFAULT_SCOPE = {
        "personal_decisions": True,
        "medical_decisions": True,
        "financial_decisions": True,
        "legal_representation": True,
        "revival_decisions": True
    }

    def __init__(self):
        self.guardianships: Dict[str, Guardianship] = {}
        self.guardianship_counter = 0

    def create_guardianship(
        self,
        subject_id: str,
        guardians: List[Dict],
        scope: Optional[Dict[str, bool]],
        appointment_authority: str,
        court_order_ref: str
    ) -> Guardianship:
        """Create new guardianship."""
        self.guardianship_counter += 1
        guardianship_id = f"GUARD-{subject_id}-{self.guardianship_counter:03d}"

        guardian_objects = [
            Guardian(
                guardian_id=f"{guardianship_id}-G{i+1}",
                name=g["name"],
                relationship=g["relationship"],
                priority=g.get("priority", i + 1),
                contact_info=g.get("contact_info", {}),
                appointment_date=datetime.utcnow(),
                active=True
            )
            for i, g in enumerate(guardians)
        ]

        guardianship = Guardianship(
            guardianship_id=guardianship_id,
            subject_id=subject_id,
            status=GuardianshipStatus.ACTIVE,
            guardians=guardian_objects,
            scope=scope or self.DEFAULT_SCOPE,
            appointment_authority=appointment_authority,
            court_order_ref=court_order_ref,
            created_at=datetime.utcnow(),
            review_date=datetime.utcnow()  # Should calculate review period
        )

        self.guardianships[guardianship_id] = guardianship
        return guardianship

    def get_active_guardian(self, guardianship_id: str) -> Optional[Guardian]:
        """Get the highest priority active guardian."""
        if guardianship_id not in self.guardianships:
            return None

        guardianship = self.guardianships[guardianship_id]
        active_guardians = [g for g in guardianship.guardians if g.active]

        if not active_guardians:
            return None

        return min(active_guardians, key=lambda g: g.priority)

    def can_make_decision(
        self,
        guardianship_id: str,
        guardian_id: str,
        decision_type: str
    ) -> tuple[bool, str]:
        """Check if guardian can make specific type of decision."""
        if guardianship_id not in self.guardianships:
            return False, "Guardianship not found"

        guardianship = self.guardianships[guardianship_id]

        if guardianship.status != GuardianshipStatus.ACTIVE:
            return False, f"Guardianship is {guardianship.status.value}"

        guardian = next(
            (g for g in guardianship.guardians if g.guardian_id == guardian_id),
            None
        )

        if not guardian or not guardian.active:
            return False, "Guardian not found or inactive"

        scope_key = f"{decision_type}_decisions"
        if scope_key not in guardianship.scope:
            return False, f"Unknown decision type: {decision_type}"

        if not guardianship.scope[scope_key]:
            return False, f"Guardian does not have {decision_type} authority"

        return True, "Authorized"

    def terminate_guardianship(
        self,
        guardianship_id: str,
        reason: str,
        authority: str
    ) -> tuple[bool, str]:
        """Terminate guardianship (e.g., upon successful revival)."""
        if guardianship_id not in self.guardianships:
            return False, "Guardianship not found"

        guardianship = self.guardianships[guardianship_id]
        guardianship.status = GuardianshipStatus.TERMINATED

        for guardian in guardianship.guardians:
            guardian.active = False

        return True, f"Guardianship terminated: {reason}"

    def transfer_guardianship(
        self,
        guardianship_id: str,
        from_guardian_id: str,
        to_guardian_id: str,
        reason: str
    ) -> tuple[bool, str]:
        """Transfer guardianship between guardians."""
        if guardianship_id not in self.guardianships:
            return False, "Guardianship not found"

        guardianship = self.guardianships[guardianship_id]

        from_guardian = next(
            (g for g in guardianship.guardians if g.guardian_id == from_guardian_id),
            None
        )
        to_guardian = next(
            (g for g in guardianship.guardians if g.guardian_id == to_guardian_id),
            None
        )

        if not from_guardian or not to_guardian:
            return False, "Guardian not found"

        from_guardian.active = False
        to_guardian.active = True
        to_guardian.priority = from_guardian.priority

        return True, f"Guardianship transferred: {reason}"
```

## JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/cryo-legal/algorithms/v1",
  "definitions": {
    "StatusTransitionRequest": {
      "type": "object",
      "required": ["to_status", "authority_id", "document_ref", "reason"],
      "properties": {
        "to_status": { "$ref": "#/definitions/LegalStatus" },
        "authority_id": { "type": "string" },
        "document_ref": { "type": "string" },
        "reason": { "type": "string" }
      }
    },
    "RestorationStage": {
      "type": "string",
      "enum": [
        "INITIATED", "IDENTITY_VERIFICATION", "MEDICAL_CLEARANCE",
        "LEGAL_REVIEW", "COURT_PETITION", "COURT_HEARING",
        "RESTORATION_GRANTED", "RIGHTS_RESTORATION", "COMPLETED", "DENIED"
      ]
    }
  }
}
```

---

*WIA Technical Committee - Cryopreservation Working Group*
